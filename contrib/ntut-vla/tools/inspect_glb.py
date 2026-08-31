"""
Say what is actually inside a GLB, before anything is spawned from it.

Why this matters here
---------------------
Project AirSim imports a spawned glTF through Assimp into a PROCEDURAL MESH
(`AssimpToProcMesh.cpp`). That converter keeps no bones and there is no
animation API, so a RIGGED model arrives in its bind pose - for a character,
a T-pose standing in the street, which looks worse than no figure at all.

Whether a file is rigged is therefore the single fact that decides how much work
it takes to use, and it is not visible from a product page. This reports it, plus
the bounding box, because a model authored in centimetres spawned as metres is a
seventeen-metre pedestrian.

Stdlib only - `json` and `struct` - matching tools/embed_glb_textures.py, which
already hand-parses GLB in this repository without a dependency.

Usage:
    python tools/inspect_glb.py D:/models/quaternius_people/glb/*.glb
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

GLB_MAGIC = 0x46546C67          # "glTF"
CHUNK_JSON = 0x4E4F534A         # "JSON"
CHUNK_BIN = 0x004E4942          # "BIN\0"

# glTF accessor component types -> (struct code, byte size)
COMPONENT = {
    5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2),
    5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4),
}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4,
         "MAT2": 4, "MAT3": 9, "MAT4": 16}


def read_glb(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    magic, version, _total = struct.unpack_from("<III", raw, 0)
    if magic != GLB_MAGIC:
        raise ValueError(f"{path.name} is not a GLB (magic {magic:#x})")
    off, js, bin_ = 12, None, b""
    while off < len(raw):
        clen, ctype = struct.unpack_from("<II", raw, off)
        data = raw[off + 8: off + 8 + clen]
        if ctype == CHUNK_JSON:
            js = json.loads(data.decode("utf-8"))
        elif ctype == CHUNK_BIN:
            bin_ = data
        off += 8 + clen + ((4 - clen % 4) % 4 if clen % 4 else 0)
    if js is None:
        raise ValueError(f"{path.name} has no JSON chunk")
    return js, bin_


def accessor_bounds(js: dict, idx: int) -> tuple[list, list] | None:
    """min/max straight from the accessor - glTF requires them on POSITION."""
    acc = js["accessors"][idx]
    if "min" in acc and "max" in acc:
        return acc["min"], acc["max"]
    return None


def _node_matrices(js: dict) -> dict:
    """World matrix per node index, walking down each scene root.

    Accessor min/max are in the primitive's OWN space. A root or armature node
    carrying a scale - 0.01 is the usual one, for a model authored in
    centimetres - is not in those numbers. Reporting them raw is precisely the
    mistake this tool exists to catch: it would print "tallest axis 175.00" for
    a file that spawns correctly at 1.75 m, or call a 17 m pedestrian fine.

    Deliberately the same walk `tools/bake_glb_poses.py::global_matrices` does,
    so the two tools cannot disagree about the size of the same file.
    """
    nodes = js.get("nodes", [])
    parent = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            parent[c] = i

    def local(n: dict):
        if "matrix" in n:
            m = [n["matrix"][r::4] for r in range(4)]        # column- -> row-major
            return [list(row) for row in m]
        t = n.get("translation", [0.0, 0.0, 0.0])
        r = n.get("rotation", [0.0, 0.0, 0.0, 1.0])          # x, y, z, w
        s = n.get("scale", [1.0, 1.0, 1.0])
        x, y, z, w = r
        rot = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        m = [[rot[i][j] * s[j] for j in range(3)] + [t[i]] for i in range(3)]
        return m + [[0.0, 0.0, 0.0, 1.0]]

    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
                for i in range(4)]

    cache: dict = {}

    def world(i: int):
        if i in cache:
            return cache[i]
        m = local(nodes[i])
        p = parent.get(i)
        cache[i] = m if p is None else mul(world(p), m)
        return cache[i]

    return {i: world(i) for i in range(len(nodes))}


def describe(path: Path) -> dict:
    js, _bin = read_glb(path)

    skins = js.get("skins", [])
    anims = js.get("animations", [])

    mats = _node_matrices(js)
    # Which node draws which mesh. A mesh drawn by no node contributes nothing
    # to what gets spawned, so it contributes nothing to the bounding box.
    drawn = [(n["mesh"], i) for i, n in enumerate(js.get("nodes", [])) if "mesh" in n]

    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    n_verts = 0
    no_bounds = 0
    for mesh_idx, node_idx in drawn:
        m = mats[node_idx]
        for prim in js["meshes"][mesh_idx].get("primitives", []):
            pidx = prim.get("attributes", {}).get("POSITION")
            if pidx is None:
                continue
            n_verts += js["accessors"][pidx].get("count", 0)
            b = accessor_bounds(js, pidx)
            if not b:
                no_bounds += 1
                continue
            # Transform all eight corners: a rotation turns the box, so mapping
            # only min and max would understate it.
            for cx in (b[0][0], b[1][0]):
                for cy in (b[0][1], b[1][1]):
                    for cz in (b[0][2], b[1][2]):
                        for k in range(3):
                            v = (m[k][0] * cx + m[k][1] * cy
                                 + m[k][2] * cz + m[k][3])
                            lo[k] = min(lo[k], v)
                            hi[k] = max(hi[k], v)

    size = [round(hi[k] - lo[k], 3) if hi[k] > lo[k] else None for k in range(3)]

    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "meshes": len(js.get("meshes", [])),
        "vertices": n_verts,
        "nodes": len(js.get("nodes", [])),
        "skins": len(skins),
        "animations": [a.get("name", f"anim{i}") for i, a in enumerate(anims)],
        "rigged": bool(skins),
        "bbox_size": size,
        "prims_without_bounds": no_bounds,
        "up_axis_guess": ("Y" if size[1] and size[1] == max(x for x in size if x)
                          else "Z" if size[2] and size[2] == max(x for x in size if x)
                          else "?"),
    }


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    paths: list[Path] = []
    for a in args:
        p = Path(a)
        paths.extend(sorted(p.parent.glob(p.name)) if "*" in p.name else [p])

    any_rigged = False
    for p in paths:
        if not p.is_file():
            print(f"  missing: {p}")
            continue
        d = describe(p)
        any_rigged |= d["rigged"]
        tall = max((x for x in d["bbox_size"] if x), default=0)
        print(f"{d['file']:<16} {d['bytes']:>8,} B  verts {d['vertices']:>6}  "
              f"nodes {d['nodes']:>3}  skins {d['skins']}  "
              f"bbox {d['bbox_size']}  tallest axis {tall:.2f}")
        if d["prims_without_bounds"]:
            print(f"{'':16} NOTE {d['prims_without_bounds']} primitive(s) declare no "
                  f"POSITION min/max; the box above is only what could be measured")
        if d["animations"]:
            print(f"{'':16} animations: {', '.join(d['animations'][:8])}")

    print()
    if any_rigged:
        print("  RIGGED. Project AirSim's importer keeps no bones, so these will")
        print("  arrive in their BIND POSE - a T-pose for a character. Bake a")
        print("  natural pose before spawning: tools/bake_glb_poses.py")
    else:
        print("  Not rigged: these are static meshes and can be spawned as they are.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
