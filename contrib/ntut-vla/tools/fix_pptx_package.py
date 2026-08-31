"""
Repair a pptxgenjs .pptx so PowerPoint will open it.

A .pptx is an OPC package: a ZIP whose members are all parts declared in
[Content_Types].xml. PowerPoint validates that relationship strictly and, when
it fails, reports only "PowerPoint could not open the file" - no part name, no
reason. Every slide can be valid XML and the package still be rejected.

Two defects in the pptxgenjs 4.0.1 output are repaired here.

1. DANGLING CONTENT-TYPE OVERRIDES - this is the one that blocks opening.
   The generated [Content_Types].xml declares overrides for parts that were
   never written: slideMaster2..7 and a matching run of slideLayouts, 60
   overrides against a package holding one master and one layout. PowerPoint
   refuses a package whose content types reference absent parts.

2. DIRECTORY ENTRIES. The ZIP format permits zero-byte members whose names end
   in "/" (_rels/, docProps/, ppt/media/ ...) and JSZip writes them. They are
   not parts, and they are removed here as well.

Both are easy to miss, because a deck that has ever been opened and re-saved by
PowerPoint comes back normalised - the directory entries gone, the overrides
pruned, docProps/thumbnail.jpeg added. So an older deck sitting in the same
folder opens fine while a freshly generated one does not, which points suspicion
at the content rather than the packaging.

The archive is rewritten keeping only real parts, with [Content_Types].xml
first as the OPC specification requires.

Usage:
    python tools/fix_pptx_package.py docs/deck.pptx
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
import zipfile
from pathlib import Path


OVERRIDE = re.compile(rb'<Override\s+PartName="([^"]+)"[^>]*/>')


def prune_content_types(raw: bytes, present: set[str]) -> tuple[bytes, list[str]]:
    """Drop <Override> elements naming parts the package does not contain."""
    dropped: list[str] = []

    def keep(m: "re.Match[bytes]") -> bytes:
        name = m.group(1).decode("utf-8")
        if name.lstrip("/") in present:
            return m.group(0)
        dropped.append(name)
        return b""

    return OVERRIDE.sub(keep, raw), dropped


def fix(path: Path, keep_backup: bool = False) -> dict:
    src = zipfile.ZipFile(path, "r")
    infos = src.infolist()

    dirs = [i for i in infos if i.filename.endswith("/") or i.is_dir()]
    parts = [i for i in infos if not (i.filename.endswith("/") or i.is_dir())]
    present = {i.filename for i in parts}

    ct_raw = src.read("[Content_Types].xml")
    ct_new, dropped = prune_content_types(ct_raw, present)

    if not dirs and not dropped:
        src.close()
        return {"changed": False, "removed": 0, "dropped": [], "parts": len(parts)}

    # [Content_Types].xml must lead the archive.
    parts.sort(key=lambda i: (i.filename != "[Content_Types].xml",))

    tmp = path.with_suffix(".pptx.fixed")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as out:
        for i in parts:
            data = ct_new if i.filename == "[Content_Types].xml" else src.read(i.filename)
            zi = zipfile.ZipInfo(i.filename, date_time=i.date_time)
            zi.compress_type = i.compress_type
            zi.external_attr = i.external_attr
            out.writestr(zi, data)
    src.close()

    if keep_backup:
        shutil.copy2(path, path.with_suffix(".pptx.bak"))
    tmp.replace(path)

    return {"changed": True, "removed": len(dirs), "dropped": dropped,
            "parts": len(parts), "names": [d.filename for d in dirs]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="+")
    ap.add_argument("--backup", action="store_true")
    args = ap.parse_args()

    for p in args.path:
        path = Path(p)
        r = fix(path, args.backup)
        if r["changed"]:
            print(f"{path.name}: {r['parts']} parts kept")
            if r["removed"]:
                print(f"   directory entries removed: {r['removed']}  "
                      + ", ".join(r["names"][:6])
                      + (" ..." if len(r["names"]) > 6 else ""))
            if r["dropped"]:
                print(f"   dangling content-type overrides pruned: {len(r['dropped'])}  "
                      + ", ".join(r["dropped"][:4])
                      + (" ..." if len(r["dropped"]) > 4 else ""))
        else:
            print(f"{path.name}: already clean ({r['parts']} parts)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
