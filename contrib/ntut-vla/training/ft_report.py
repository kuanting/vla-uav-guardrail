"""
Fine-tuning campaign report — loss curve + before/after comparison charts.

Reads:
    training/ft_loss_log.csv          (from finetune_aerialvla.py)
    training/eval_<name>.json         (from eval_aerialvla.py, >=2 of them)

Writes:
    docs/aerialvla-ft-report.md       tables + findings
    docs/img/ft_loss_curve.png
    docs/img/ft_comparison.png

Run:  python training/ft_report.py --evals baseline ft_e0 ft_e1
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--evals", nargs="+", required=True,
                    help="eval names in order, baseline first")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    IMG.mkdir(parents=True, exist_ok=True)

    # ---- loss curve ----
    steps, losses = [], []
    csv_path = ROOT / "training" / "ft_loss_log.csv"
    if csv_path.exists():
        with csv_path.open(encoding="utf-8") as f:
            for row in csv.DictReader(f):
                steps.append(int(row["step"]))
                losses.append(float(row["loss"]))
    if steps:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(steps, losses, color="tab:blue")
        ax.set_xlabel("optimizer step")
        ax.set_ylabel("action-token loss")
        ax.set_title("AerialVLA QLoRA fine-tuning — training loss")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(IMG / "ft_loss_curve.png", dpi=130)
        print(f"[report] {IMG / 'ft_loss_curve.png'}")

    # ---- comparison ----
    evals = []
    for name in args.evals:
        p = ROOT / "training" / f"eval_{name}.json"
        evals.append(json.loads(p.read_text(encoding="utf-8")))

    names = [e["name"] for e in evals]
    reached = [e["reached_rate"] * 100 for e in evals]
    eff = [e["mean_efficiency"] for e in evals]
    interv = [e["mean_interventions"] for e in evals]

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    for ax, vals, title, fmt in [
        (axes[0], reached, "Target reached (%)", "{:.0f}%"),
        (axes[1], eff, "Path efficiency (straight/actual)", "{:.3f}"),
        (axes[2], interv, "Shield interventions (mean)", "{:.0f}"),
    ]:
        colors = ["#9AA3AD"] + ["#249DB2"] * (len(vals) - 1)
        bars = ax.bar(names, vals, color=colors)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                    fmt.format(v), ha="center", va="bottom", fontsize=10)
        ax.set_title(title)
        ax.grid(alpha=0.3, axis="y")
    fig.suptitle("AerialVLA fine-tuning — closed-loop eval (pure VLA, no guidance assist)")
    fig.tight_layout()
    fig.savefig(IMG / "ft_comparison.png", dpi=130)
    print(f"[report] {IMG / 'ft_comparison.png'}")

    # ---- markdown ----
    md = [f"# AerialVLA Fine-Tuning Report — {date.today()}", ""]
    md += ["Base: openvla-7b (4-bit NF4) + AerialVLA LoRA (r=64), fine-tuned with "
           "QLoRA on self-collected AirSimNH expert flights "
           "(front+down mosaic, semantic-direction prompt, 99-bin action text).", ""]
    md += ["## Closed-loop evaluation (pure VLA — goal assist OFF)", ""]
    md += ["| adapter | episodes | reached | mean efficiency | mean interventions | NFZ s |",
           "|---|---|---|---|---|---|"]
    for e in evals:
        md.append(f"| {e['name']} | {e['episodes']} | {e['reached_rate']:.0%} | "
                  f"{e['mean_efficiency']} | {e['mean_interventions']} | "
                  f"{e['nfz_total_s']} |")
    md += ["", "![comparison](img/ft_comparison.png)", ""]
    if steps:
        md += ["## Training loss", "", "![loss](img/ft_loss_curve.png)", ""]
    md += ["## Notes",
           "- Eval flights are PURE VLA (`--goal-blend 0`): the score measures the",
           "  model itself; the tuned guidance assist would raise all variants further.",
           "- NFZ safety is the guardrail's job in every configuration (P0 escape = 0",
           "  regardless of the model in the slot)."]
    out = ROOT / "docs" / "aerialvla-ft-report.md"
    out.write_text("\n".join(md), encoding="utf-8")
    print(f"[report] {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
