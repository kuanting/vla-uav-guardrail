"""
Copy the tracked source into the fork's contrib/ntut-vla/, one direction only.

Why this exists
---------------
Our work is shared with Prof. Lai through `kuanting-vla-uav-guardrail`, a fork
whose `contrib/ntut-vla/` subtree holds a copy of this repository's source. That
copy used to be maintained by hand, and by 2026-08-20 it had drifted a long way:
the last sync was commit fe0dc0f, and everything after it - the target-state
estimator, the chroma colour gate, the determinism manifest, the KPI module, the
turning route, the recorder - existed only in the working tree.

A hand-maintained mirror drifts silently, and a silent drift is worse than an
obvious one: the fork looks current.

So the mirror is generated, never edited. This script is the only thing that
writes to `contrib/ntut-vla/`.

What it copies
--------------
Exactly what `git ls-files` reports, which means `.gitignore` decides. Flight
artefacts, weights, datasets, build output and generated documents are excluded
by the same rules that keep them out of this repository, so a large file cannot
reach the fork by a different route.

Files present in the destination that are no longer tracked here are DELETED, so
the mirror cannot accumulate copies of things that have been renamed or removed.
That is the whole point; a sync that only ever adds is how the drift started.

What it does not do
-------------------
It does not `git add`, `git commit`, or `git push`. Publishing to a remote is a
decision, and the fork's `origin` is the professor's repository. Review the diff
in the fork and commit there yourself.

Usage:
    python tools/publish_to_fork.py --dry-run      # show what would change
    python tools/publish_to_fork.py
"""
from __future__ import annotations

import argparse
import filecmp
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "kuanting-vla-uav-guardrail" / "contrib" / "ntut-vla"

# Tracked here, but not this project's to republish. The grant PDFs are the
# funder's documents, the meeting notes record other people's words, and this
# repository's own tooling files mean nothing inside a subtree of someone
# else's repository.
SKIP_PREFIXES = (
    "meeting notes/",
    "kuanting-vla-uav-guardrail/",
    ".claude/",
)
SKIP_SUFFIXES = (
    " Simulation Framework Survey.pdf",
)
SKIP_EXACT = (
    ".gitignore",
)


def tracked_files() -> list[str]:
    # -z, because `git ls-files` C-quotes any path with a non-ASCII byte in it:
    # the grant PDFs come back as "Architecture constraints - ... \342\200\224
    # ....pdf", quotes and escapes included, and every string comparison against
    # them silently fails. NUL-delimited output is the raw path.
    r = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-z"],
                       capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        raise SystemExit("git ls-files failed; is this a repository?")
    out = []
    for p in r.stdout.split("\0"):
        p = p.strip()
        if not p:
            continue
        if (p.startswith(SKIP_PREFIXES) or p.endswith(SKIP_SUFFIXES)
                or p in SKIP_EXACT):
            continue
        out.append(p)
    return sorted(out)


def existing_files() -> set[str]:
    if not DEST.is_dir():
        return set()
    return {str(p.relative_to(DEST)).replace("\\", "/")
            for p in DEST.rglob("*") if p.is_file()}


# Paths under contrib/ntut-vla/ that are AUTHORED IN THE FORK and must survive
# a sync. The mirror is generated, but the directory it writes into is not
# purely generated - these were written there, for Prof. Lai, and never existed
# in this repository.
#
# Without this the prune below treated them as drift and deleted them: the
# fork's README.md, MODELS.md (which carries the asset provenance for models
# that are deliberately not in git), the two slide decks, and every
# results/*/metrics.json - 42 files on the run that found this.
#
# The anti-drift rule still holds for everything the mirror actually produces.
# It just no longer assumes that "not tracked here" means "stale".
KEEP_IN_FORK = (
    "README.md",
    "MODELS.md",
    "docs/VLA-Guardrail-FineTuned-Jul2026.pptx",
    "docs/VLA-Guardrail-Follow-Aug2026.pptx",
)
KEEP_PREFIXES = ("results/",)


def fork_authored(rel: str) -> bool:
    """True for a mirror path this script must not delete."""
    rel = rel.replace("\\", "/")
    return rel in KEEP_IN_FORK or rel.startswith(KEEP_PREFIXES)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="report what would change and write nothing")
    args = ap.parse_args()

    if not DEST.parent.parent.is_dir():
        raise SystemExit(f"fork not found at {DEST.parent.parent}")

    want = tracked_files()
    have = existing_files()
    wanted = set(want)

    added, updated, unchanged = [], [], []
    for rel in want:
        src, dst = ROOT / rel, DEST / rel
        if not dst.exists():
            added.append(rel)
        elif not filecmp.cmp(src, dst, shallow=False):
            updated.append(rel)
        else:
            unchanged.append(rel)

    removed = sorted(r for r in (have - wanted) if not fork_authored(r))
    preserved = sorted(r for r in (have - wanted) if fork_authored(r))

    print(f"source : {ROOT}")
    print(f"mirror : {DEST}")
    print(f"  add     {len(added)}")
    print(f"  update  {len(updated)}")
    print(f"  delete  {len(removed)}")
    print(f"  keep    {len(preserved)}   (authored in the fork, never mirrored)")
    print(f"  same    {len(unchanged)}")

    for label, items in (("add", added), ("update", updated), ("delete", removed)):
        for rel in items[:20]:
            print(f"    {label:<7} {rel}")
        if len(items) > 20:
            print(f"    {label:<7} ... and {len(items) - 20} more")

    if args.dry_run:
        print("\ndry run; nothing written")
        return 0

    for rel in added + updated:
        src, dst = ROOT / rel, DEST / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    for rel in removed:
        (DEST / rel).unlink(missing_ok=True)

    # Leave no empty directories behind after deletions.
    for d in sorted((p for p in DEST.rglob("*") if p.is_dir()),
                    key=lambda p: len(p.parts), reverse=True):
        try:
            d.rmdir()
        except OSError:
            pass

    print(f"\nwrote {len(added) + len(updated)} files, deleted {len(removed)}")
    print("Nothing was committed or pushed. Review the diff in the fork:")
    print(f"    git -C {DEST.parent.parent} status")
    return 0


if __name__ == "__main__":
    sys.exit(main())
