"""
Check a .pptx for the defects that make PowerPoint refuse to open it.

PowerPoint validates an OPC package strictly and, on failure, says only
"PowerPoint could not open the file" - no slide number, no part name, no reason.
Every defect below was found by bisecting a deck one slide at a time, which is
an expensive way to learn that one text box had a negative height.

Checked here:

  1. NEGATIVE OR ZERO EXTENTS. A shape with <a:ext cy="-100584"> makes the whole
     package unopenable. This happens whenever a layout helper computes a height
     by subtraction and the container is smaller than its own header.
  2. DANGLING CONTENT-TYPE OVERRIDES. [Content_Types].xml declaring parts that
     were never written.
  3. DIRECTORY ENTRIES. Zero-byte ZIP members ending in "/", which are not parts.
  4. DANGLING RELATIONSHIPS. An .rels target that resolves to a missing part.
  5. EMPTY SLIDES. Not fatal, but a slide with no text is almost always a bug.

Exit code is non-zero if any fatal defect is present, so a build script can
refuse to ship the file.

Usage:
    python tools/audit_pptx.py docs/deck.pptx
"""
from __future__ import annotations

import re
import sys
import zipfile
from pathlib import Path

EXT = re.compile(rb'<a:ext\s+cx="(-?\d+)"\s+cy="(-?\d+)"')
SLIDE = re.compile(r"ppt/slides/slide(\d+)\.xml$")


def resolve(base: str, target: str) -> str:
    """Resolve a relationship target against the part's base directory."""
    if target.startswith("/"):
        return target.lstrip("/")
    p = f"{base}/{target}" if base else target
    p = p.replace("/./", "/")
    while "/../" in p:
        p = re.sub(r"[^/]+/\.\./", "", p, count=1)
    return p.lstrip("/")


def audit(path: Path) -> tuple[list[str], list[str]]:
    fatal: list[str] = []
    warn: list[str] = []

    z = zipfile.ZipFile(path)
    names = set(z.namelist())

    # 3 - directory entries
    dirs = [n for n in names if n.endswith("/")]
    if dirs:
        fatal.append(f"{len(dirs)} directory entries in the archive "
                     f"({', '.join(sorted(dirs)[:4])}...)")

    # 2 - dangling content-type overrides
    ct = z.read("[Content_Types].xml").decode("utf-8")
    over = re.findall(r'PartName="([^"]+)"', ct)
    dangling = [p for p in over if p.lstrip("/") not in names]
    if dangling:
        fatal.append(f"{len(dangling)} content-type overrides name missing parts "
                     f"({', '.join(dangling[:3])}...)")

    # 4 - dangling relationships
    bad_rels: list[str] = []
    for rels in sorted(n for n in names if n.endswith(".rels")):
        base = rels.rsplit("/_rels/", 1)[0] if "/_rels/" in rels else ""
        body = z.read(rels).decode("utf-8")
        for block in re.findall(r"<Relationship\b[^>]*/>", body):
            if 'TargetMode="External"' in block:
                continue
            m = re.search(r'Target="([^"]+)"', block)
            if not m or m.group(1).startswith(("http://", "https://")):
                continue
            got = resolve(base, m.group(1))
            if got not in names:
                bad_rels.append(f"{rels} -> {m.group(1)}")
    if bad_rels:
        fatal.append(f"{len(bad_rels)} dangling relationships "
                     f"({'; '.join(bad_rels[:3])})")

    # 1 and 5 - per-slide geometry and content
    for name in sorted(names):
        m = SLIDE.match(name)
        if not m:
            continue
        idx = int(m.group(1))
        raw = z.read(name)
        for cx, cy in EXT.findall(raw):
            cx, cy = int(cx), int(cy)
            if cx < 0 or cy < 0:
                fatal.append(f"slide {idx}: negative extent cx={cx} cy={cy}. "
                             f"A shape sized by subtraction went below zero; "
                             f"PowerPoint rejects the whole package.")
        text = re.findall(r"<a:t>(.*?)</a:t>", raw.decode("utf-8"), re.S)
        if not [t for t in text if t.strip()]:
            warn.append(f"slide {idx}: no text")

    return fatal, warn


def main() -> int:
    rc = 0
    for arg in sys.argv[1:]:
        p = Path(arg)
        fatal, warn = audit(p)
        n_slides = len([n for n in zipfile.ZipFile(p).namelist() if SLIDE.match(n)])
        if fatal:
            print(f"audit  : {p.name} FAILED ({n_slides} slides)")
            for f in fatal:
                print(f"   FATAL {f}")
            rc = 1
        else:
            print(f"audit  : {p.name} clean ({n_slides} slides)")
        for w in warn:
            print(f"   warn  {w}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
