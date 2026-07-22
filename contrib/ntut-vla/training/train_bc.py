"""
Behavior-cloning trainer — small MLP that imitates shield-corrected flying.

    python training/train_bc.py                       # trains models/bc_policy.pt

Architecture: 8 -> 128 -> 128 -> 3 (tanh output, scaled by ACTION_SCALE at
runtime). ~20k parameters — trains in minutes on CPU; deliberately small so
it can run at 10 Hz with margin on any machine (grant monitor budget).
Exported as TorchScript so the runtime needs no model class import.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]


def build_model() -> nn.Module:
    return nn.Sequential(
        nn.Linear(8, 128), nn.ReLU(),
        nn.Linear(128, 128), nn.ReLU(),
        nn.Linear(128, 3), nn.Tanh(),      # outputs in [-1, 1] = action / 4.0
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(ROOT / "training" / "data" / "bc_dataset.npz"))
    ap.add_argument("--out", default=str(ROOT / "models" / "bc_policy.pt"))
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--lr", type=float, default=1e-3)
    args = ap.parse_args()

    d = np.load(args.data)
    X = torch.tensor(d["X"])
    Y = torch.tensor(d["Y"]).clamp(-1, 1)
    n = len(X)
    idx = torch.randperm(n)
    n_val = n // 10
    val_i, tr_i = idx[:n_val], idx[n_val:]
    Xtr, Ytr, Xval, Yval = X[tr_i], Y[tr_i], X[val_i], Y[val_i]
    print(f"train {len(Xtr)}  val {len(Xval)}")

    torch.manual_seed(0)
    model = build_model()
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    lossf = nn.MSELoss()

    best = float("inf")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(len(Xtr))
        tot = 0.0
        for i in range(0, len(Xtr), args.batch):
            b = perm[i:i + args.batch]
            opt.zero_grad()
            loss = lossf(model(Xtr[b]), Ytr[b])
            loss.backward()
            opt.step()
            tot += loss.item() * len(b)
        model.eval()
        with torch.no_grad():
            vl = lossf(model(Xval), Yval).item()
        marker = ""
        if vl < best:
            best = vl
            torch.jit.script(model).save(str(out))
            marker = "  <- saved"
        print(f"epoch {ep + 1:2d}/{args.epochs}  train {tot / len(Xtr):.5f}  val {vl:.5f}{marker}")

    print(f"best val MSE {best:.5f} -> {out}")
    # sanity: typical action error in m/s = sqrt(MSE) * 4
    print(f"(~{(best ** 0.5) * 4:.2f} m/s typical velocity error)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
