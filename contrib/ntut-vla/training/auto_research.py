"""
AUTO-RESEARCH CONTROLLER — train a deployable flight policy automatically
until it clears the quality gate. Maximized for this machine:

    - dataset generation + evaluation: sharded across 12 CPU workers
    - training: RTX 4080 SUPER, TF32 + AMP, batch 8192
    - live monitoring: writes training/status.json every stage/epoch;
      pair with training/dashboard.py (http://127.0.0.1:8085)

Loop per iteration: generate (escalating, + DAgger from it2) -> train ->
closed-loop eval (held-out, shield OFF = true learned safety) -> gate ->
pass: export TorchScript + ONNX; fail: escalate config, repeat.
Lab notebook: training/research_log.md.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from training.evaluate_policy import evaluate_parallel, passes   # noqa: E402
from training.features import FEAT_V3_DIM                        # noqa: E402
from training.generate_v2 import generate                        # noqa: E402
from training.status import StatusWriter                         # noqa: E402

LOG = ROOT / "training" / "research_log.md"
MODELS = ROOT / "models"
WORKERS = 12

# Improvement campaign: warm-start style — begin at the recipe that passed
# gate 1 (8k/256x3/dagger .2) and escalate. Gate 2 thresholds: reach>=98%,
# NFZ<=0.25%, interventions<=2.5%, judged on 400 held-out episodes.
SCHEDULE = [
    dict(eps=10000, hidden=256, depth=3, epochs=110, dagger=0.25),
    dict(eps=16000, hidden=512, depth=3, epochs=130, dagger=0.35),
    dict(eps=24000, hidden=512, depth=4, epochs=150, dagger=0.40),
    dict(eps=34000, hidden=768, depth=4, epochs=170, dagger=0.45),
]


def build_model(hidden: int, depth: int) -> nn.Module:
    layers: list[nn.Module] = [nn.Linear(FEAT_V3_DIM, hidden), nn.ReLU()]
    for _ in range(depth - 1):
        layers += [nn.Linear(hidden, hidden), nn.ReLU()]
    layers += [nn.Linear(hidden, 3), nn.Tanh()]
    return nn.Sequential(*layers)


def train(X, Y, hidden, depth, epochs, status: StatusWriter):
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

    Xt = torch.tensor(X, device=dev)
    Yt = torch.tensor(Y, device=dev).clamp(-1, 1)
    n = len(Xt)
    idx = torch.randperm(n, device=dev)
    n_val = n // 10
    vi, ti = idx[:n_val], idx[n_val:]

    torch.manual_seed(0)
    model = build_model(hidden, depth).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=1.5e-3, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    lossf = nn.MSELoss()
    scaler = torch.amp.GradScaler(enabled=dev == "cuda")
    batch = 8192

    best_val = float("inf")
    best_state = None
    patience, bad = 18, 0
    hist = []
    for ep in range(epochs):
        model.train()
        perm = ti[torch.randperm(len(ti), device=dev)]
        tot = 0.0
        for i in range(0, len(perm), batch):
            b = perm[i:i + batch]
            opt.zero_grad(set_to_none=True)
            with torch.amp.autocast(device_type="cuda", enabled=dev == "cuda"):
                loss = lossf(model(Xt[b]), Yt[b])
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            tot += loss.item() * len(b)
        sched.step()
        model.eval()
        with torch.no_grad():
            vl = lossf(model(Xt[vi]), Yt[vi]).item()
        hist.append(round(vl, 6))
        status.update(train={"epoch": ep + 1, "epochs": epochs,
                             "train_loss": round(tot / len(ti), 6),
                             "val_loss": round(vl, 6),
                             "best_val": round(min(best_val, vl), 6),
                             "loss_hist": hist[-200:]})
        if vl < best_val - 1e-5:
            best_val = vl
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
            bad = 0
        else:
            bad += 1
            if bad >= patience:
                break
    model.load_state_dict(best_state)
    return model.cpu(), best_val


def log_line(text: str) -> None:
    with LOG.open("a", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text, flush=True)


def main() -> int:
    MODELS.mkdir(exist_ok=True)
    status = StatusWriter()
    dev = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU (!)"
    if not LOG.exists():
        LOG.write_text("# Auto-research log — deployable flight policy\n\n",
                       encoding="utf-8")
    log_line(f"\n## Run started {time.strftime('%Y-%m-%d %H:%M')} (device: {dev}, "
             f"{WORKERS} CPU workers)\n")
    log_line("| it | episodes | dagger | net | samples | val MSE | reach | "
             "NFZ entry | interv. | time | verdict |")
    log_line("|---|---|---|---|---|---|---|---|---|---|---|")
    status.update(stage="starting", device=dev)

    best_score = -1.0
    # Warm start: if a gate-passing model exists, let IT drive the DAgger
    # rollouts from iteration 1 (state coverage starts from a good pilot).
    warm = MODELS / "vla_policy_v2_gate1.pt"
    dagger_path = str(warm) if warm.exists() else None
    if dagger_path:
        log_line(f"(warm start: DAgger pilot = {warm.name})")
    tmp_model = str(MODELS / "_iter_candidate.pt")
    for it, cfg in enumerate(SCHEDULE, 1):
        t0 = time.time()
        status.update(stage="generating", iteration=it, config=cfg,
                      gen={"done": 0, "total": cfg["eps"], "samples": 0})
        X, Y, teacher_rr = generate(
            cfg["eps"], seed=100 + it, dagger_path=dagger_path,
            dagger_frac=cfg["dagger"], workers=WORKERS,
            progress=lambda d, t, s: status.update(
                gen={"done": d, "total": t, "samples": s}))

        status.update(stage="training",
                      train={"epoch": 0, "epochs": cfg["epochs"], "loss_hist": []})
        model, val = train(X, Y, cfg["hidden"], cfg["depth"], cfg["epochs"], status)
        scripted = torch.jit.script(model)
        scripted.save(tmp_model)

        status.update(stage="evaluating", eval={})
        m = evaluate_parallel(tmp_model, episodes=400, workers=WORKERS)
        ok, fails = passes(m)
        status.update(eval=m)

        score = m["reach_rate"] - 5 * m["nfz_entry_rate"] - m["intervention_rate"]
        verdict = "PASS" if ok else "fail: " + "; ".join(fails)
        row = (f"| {it} | {cfg['eps']} | {cfg['dagger']} | "
               f"{cfg['hidden']}x{cfg['depth']} | {len(X)} | {val:.4f} | "
               f"{m['reach_rate']:.1%} | {m['nfz_entry_rate']:.1%} | "
               f"{m['intervention_rate']:.1%} | {m['mean_time_s']:.0f}s | {verdict} |")
        log_line(row)
        log_line(f"  (wall {time.time() - t0:.0f}s, teacher reach {teacher_rr:.1%})")
        status.update(iterations=status.state["iterations"] + [
            {"it": it, "cfg": f"{cfg['eps']}ep/{cfg['hidden']}x{cfg['depth']}/dg{cfg['dagger']}",
             "val": round(val, 4), **{k: round(v, 4) for k, v in m.items()},
             "verdict": verdict, "wall_s": round(time.time() - t0)}])

        if score > best_score:
            best_score = score
            scripted.save(str(MODELS / "vla_policy_v2.pt"))
            (MODELS / "vla_policy_v2.metrics.json").write_text(
                json.dumps({"iteration": it, "config": cfg, "metrics": m,
                            "val_mse": val}, indent=2), encoding="utf-8")
        dagger_path = tmp_model

        if ok:
            log_line(f"\n**GATE PASSED at iteration {it}.**")
            try:
                torch.onnx.export(model, torch.zeros(1, FEAT_V3_DIM),
                                  str(MODELS / "vla_policy_v2.onnx"),
                                  input_names=["features"], output_names=["action"],
                                  dynamic_axes={"features": {0: "batch"}})
                log_line("ONNX exported: models/vla_policy_v2.onnx (Jetson-ready)")
            except Exception as e:                       # noqa: BLE001
                log_line(f"ONNX export failed (non-fatal): {e}")
            status.update(stage="done", done=True, verdict=f"PASS at iteration {it}")
            return 0

    log_line("\n**Schedule exhausted — best model kept (see metrics.json).**")
    status.update(stage="done", done=True,
                  verdict="schedule exhausted; best model kept")
    return 1


if __name__ == "__main__":
    sys.exit(main())
