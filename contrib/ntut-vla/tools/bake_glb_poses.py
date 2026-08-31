"""
Freeze a rigged glTF into a static mesh, posed.

Why
---
Project AirSim spawns a glTF through Assimp into a PROCEDURAL MESH
(`AssimpToProcMesh.cpp`). That path keeps no bones, and the World API has no
animation call at all, so a rigged character arrives in its BIND POSE - arms out,
T-posed, standing in the street. Worse than no figure.

The fix is to do the skinning ourselves, once, offline: evaluate the animation at
a chosen time, apply the joint transforms to the vertices, and write the result
out as an ordinary static mesh. The simulator then has nothing left to animate,
which is fine, because the pose is already correct.

    v' = SUM_i  w_i * (globalJoint_i * inverseBind_i) * v

For decoration one frame of `Man_Idle` is enough. Several frames of a locomotion
clip give a set of poses that can be cycled to fake a gait.

How the output is built
-----------------------
The original JSON is kept and the posed positions are APPENDED as a new buffer
view and accessor, with each primitive's POSITION repointed at it. `skins`,
`animations`, and the JOINTS_0/WEIGHTS_0 attributes are then dropped, and the
mesh node's transform is reset - a skinned mesh ignores its node transform by
specification, so leaving it would move the baked mesh.

The superseded vertex data stays in the buffer. It is a few hundred kilobytes on
a model of this size and rewriting the buffer to reclaim it would mean remapping
every index in the file, which is a great deal of risk for no benefit.

Usage:
    python tools/bake_glb_poses.py IN.glb --anim Man_Idle --out OUT.glb
    python tools/bake_glb_poses.py IN.glb --anim Man_Run --phases 4 --out-dir DIR
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

GLB_MAGIC = 0x46546C67
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN = 0x004E4942

COMPONENT = {5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2),
             5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


# ------------------------------------------------------------------ reading --

def read_glb(path: Path):
    raw = path.read_bytes()
    magic, _ver, _tot = struct.unpack_from("<III", raw, 0)
    if magic != GLB_MAGIC:
        raise SystemExit(f"{path.name} is not a GLB")
    off, js, bin_ = 12, None, b""
    while off < len(raw):
        clen, ctype = struct.unpack_from("<II", raw, off)
        data = raw[off + 8: off + 8 + clen]
        if ctype == CHUNK_JSON:
            js = json.loads(data.decode("utf-8"))
        elif ctype == CHUNK_BIN:
            bin_ = data
        pad = ((4 - clen % 4) % 4) if clen % 4 else 0
        off += 8 + clen + pad
    return js, bytearray(bin_)


def read_accessor(js, bin_, idx) -> np.ndarray:
    """Accessor -> (count, ncomp) float/int array. Handles byteStride."""
    acc = js["accessors"][idx]
    n = acc["count"]
    ncomp = NCOMP[acc["type"]]
    code, csize = COMPONENT[acc["componentType"]]
    bv = js["bufferViews"][acc["bufferView"]]
    base = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride") or (ncomp * csize)

    out = np.empty((n, ncomp), dtype=np.float64 if code == "f" else np.int64)
    for i in range(n):
        o = base + i * stride
        out[i] = struct.unpack_from("<" + code * ncomp, bin_, o)
    if acc.get("normalized") and code in ("B", "H"):
        out = out / (255.0 if code == "B" else 65535.0)
    return out


# ------------------------------------------------------------------- maths --

def trs_matrix(node) -> np.ndarray:
    if "matrix" in node:
        # glTF matrices are column-major
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    m = np.eye(4)
    if "scale" in node:
        m = np.diag(list(node["scale"]) + [1.0]) @ m
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1]], dtype=np.float64)
        m = r @ m
    if "translation" in node:
        t = np.eye(4)
        t[:3, 3] = node["translation"]
        m = t @ m
    return m


def slerp(a, b, u):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    d = float(np.dot(a, b))
    if d < 0.0:                      # take the short way round
        b, d = -b, -d
    if d > 0.9995:
        q = a + u * (b - a)
        return q / np.linalg.norm(q)
    th0 = np.arccos(d)
    th = th0 * u
    s0 = np.sin(th0 - th) / np.sin(th0)
    s1 = np.sin(th) / np.sin(th0)
    return s0 * a + s1 * b


def sample_channel(times, values, t, path, interp):
    """Value of one animated property at time t."""
    if len(times) == 1 or t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    i = int(np.searchsorted(times, t) - 1)
    i = max(0, min(i, len(times) - 2))
    t0, t1 = times[i], times[i + 1]
    u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    if interp == "STEP":
        return values[i]
    if path == "rotation":
        return slerp(values[i], values[i + 1], u)
    return values[i] + u * (values[i + 1] - values[i])


# ------------------------------------------------------------------ baking --

def pose_nodes(js, bin_, anim_idx, t):
    """Copy of the node list with the animation applied at time t."""
    nodes = [dict(n) for n in js["nodes"]]
    if anim_idx is None:
        return nodes
    anim = js["animations"][anim_idx]
    for ch in anim["channels"]:
        tgt = ch["target"]
        node_i, path = tgt.get("node"), tgt.get("path")
        if node_i is None or path == "weights":
            continue
        smp = anim["samplers"][ch["sampler"]]
        times = read_accessor(js, bin_, smp["input"])[:, 0]
        vals = read_accessor(js, bin_, smp["output"])
        interp = smp.get("interpolation", "LINEAR")
        if interp == "CUBICSPLINE":
            vals = vals[1::3]        # keep the value, drop the tangents
            interp = "LINEAR"
        v = sample_channel(times, vals, t, path, interp)
        nodes[node_i].pop("matrix", None)
        nodes[node_i][path] = list(np.asarray(v, float))
    return nodes


def global_matrices(js, nodes):
    """World matrix per node, walking down each scene root."""
    parent = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            parent[c] = i
    out = [None] * len(nodes)

    def resolve(i):
        if out[i] is not None:
            return out[i]
        local = trs_matrix(nodes[i])
        out[i] = local if i not in parent else resolve(parent[i]) @ local
        return out[i]

    for i in range(len(nodes)):
        resolve(i)
    return out


def anim_duration(js, bin_, anim_idx) -> float:
    anim = js["animations"][anim_idx]
    end = 0.0
    for s in anim["samplers"]:
        ts = read_accessor(js, bin_, s["input"])[:, 0]
        end = max(end, float(ts[-1]))
    return end


def bake(js, bin_, anim_idx, t):
    """Return {mesh_node_index: posed positions} and the new bbox."""
    nodes = pose_nodes(js, bin_, anim_idx, t)
    gmat = global_matrices(js, nodes)

    posed = {}
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)

    for ni, node in enumerate(js["nodes"]):
        if "mesh" not in node:
            continue
        mesh = js["meshes"][node["mesh"]]
        skin_i = node.get("skin")

        for prim in mesh["primitives"]:
            attrs = prim["attributes"]
            pos = read_accessor(js, bin_, attrs["POSITION"])
            v4 = np.hstack([pos, np.ones((len(pos), 1))])

            if skin_i is None:
                out = (gmat[ni] @ v4.T).T[:, :3]
            else:
                skin = js["skins"][skin_i]
                joints = skin["joints"]
                ibm = read_accessor(js, bin_, skin["inverseBindMatrices"])
                ibm = ibm.reshape(-1, 4, 4).transpose(0, 2, 1)   # column-major
                jm = np.stack([gmat[joints[k]] @ ibm[k] for k in range(len(joints))])

                ji = read_accessor(js, bin_, attrs["JOINTS_0"]).astype(int)
                w = read_accessor(js, bin_, attrs["WEIGHTS_0"]).astype(float)
                wsum = w.sum(axis=1, keepdims=True)
                w = np.divide(w, wsum, out=np.zeros_like(w), where=wsum > 0)

                out = np.zeros((len(pos), 3))
                for k in range(ji.shape[1]):
                    m = jm[ji[:, k]]                       # (n,4,4)
                    contrib = np.einsum("nij,nj->ni", m, v4)[:, :3]
                    out += w[:, k:k + 1] * contrib

            posed[(ni, id(prim))] = (prim, out)
            lo = np.minimum(lo, out.min(axis=0))
            hi = np.maximum(hi, out.max(axis=0))

    return posed, lo, hi


def _normals(pts, idx):
    """Area-weighted vertex normals for the POSED geometry.

    The original NORMAL accessor describes the BIND pose, so reusing it after
    skinning lights the figure wrong. Recomputing from the posed triangles is
    exact for the mesh actually being written, and it removes any dependence on
    what the source file happened to store.
    """
    nrm = np.zeros_like(pts)
    tri = pts[idx.reshape(-1, 3)]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    for k in range(3):
        np.add.at(nrm, idx.reshape(-1, 3)[:, k], fn)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    return np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-12)


def write_glb(js, bin_, posed, out_path: Path):
    """Write a FRESH minimal static GLB - do not edit the rigged one in place.

    Editing in place was tried first and Project AirSim refused every file with
    `ERROR code: 2.0` while the same call spawned the car fleet happily. Editing
    leaves too much of the source behind to reason about: the skinning
    accessors stay in the buffer, the 45-node armature hierarchy stays in the
    scene graph, and - the actual defect - stripping the transform from a mesh
    node does nothing about the transforms on its ANCESTORS, so an armature
    scale would still be applied on top of world-space baked vertices.

    So: emit only what a static mesh needs. One node per primitive, no
    hierarchy, no transforms, no rig, POSITION and NORMAL and indices only.
    Materials keep their base colour and drop everything else - these models
    carry no images, so nothing is lost. The result is roughly a quarter the
    size and has no leftovers to misread.
    """
    buf = bytearray()
    bviews, accs, prims_out = [], [], []

    def put(arr, comp_type, kind, target):
        while len(buf) % 4:
            buf.append(0)
        off = len(buf)
        data = arr.tobytes()
        buf.extend(data)
        bviews.append({"buffer": 0, "byteOffset": off, "byteLength": len(data),
                       "target": target})
        acc = {"bufferView": len(bviews) - 1, "componentType": comp_type,
               "count": int(arr.shape[0]), "type": kind}
        if kind == "VEC3":
            acc["min"] = [float(v) for v in arr.min(axis=0)]
            acc["max"] = [float(v) for v in arr.max(axis=0)]
        accs.append(acc)
        return len(accs) - 1

    for (_ni, _pid), (prim_orig, pts) in posed.items():
        pts = np.asarray(pts, dtype=np.float32)
        if "indices" in prim_orig:
            idx = read_accessor(js, bin_, prim_orig["indices"]).astype(np.uint32).ravel()
        else:
            idx = np.arange(len(pts), dtype=np.uint32)
        if len(idx) % 3:
            idx = idx[: len(idx) - len(idx) % 3]

        a_pos = put(pts, 5126, "VEC3", 34962)
        a_nrm = put(_normals(pts, idx).astype(np.float32), 5126, "VEC3", 34962)
        a_idx = put(idx, 5125, "SCALAR", 34963)

        prim = {"attributes": {"POSITION": a_pos, "NORMAL": a_nrm},
                "indices": a_idx, "mode": 4}
        if "material" in prim_orig:
            prim["material"] = prim_orig["material"]
        prims_out.append(prim)

    # Base colour only. These packs ship untextured, so a full material would
    # carry references to sampler and texture arrays that are not being written.
    mats = []
    for m in js.get("materials", []):
        pbr = m.get("pbrMetallicRoughness", {})
        mats.append({
            "name": m.get("name", "mat"),
            "pbrMetallicRoughness": {
                "baseColorFactor": pbr.get("baseColorFactor", [0.8, 0.8, 0.8, 1.0]),
                "metallicFactor": float(pbr.get("metallicFactor", 0.0)),
                "roughnessFactor": float(pbr.get("roughnessFactor", 0.9)),
            },
            "doubleSided": bool(m.get("doubleSided", True)),
        })

    out = {
        "asset": {"version": "2.0", "generator": "bake_glb_poses.py"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(prims_out)))}],
        "nodes": [{"mesh": i} for i in range(len(prims_out))],
        "meshes": [{"primitives": [p]} for p in prims_out],
        "materials": mats,
        "accessors": accs,
        "bufferViews": bviews,
        "buffers": [{"byteLength": len(buf)}],
    }
    if not mats:
        out.pop("materials")
        for m in out["meshes"]:
            m["primitives"][0].pop("material", None)

    js_bytes = json.dumps(out, separators=(",", ":")).encode("utf-8")
    js_bytes += b" " * ((4 - len(js_bytes) % 4) % 4)
    while len(buf) % 4:
        buf.append(0)

    total = 12 + 8 + len(js_bytes) + 8 + len(buf)
    with out_path.open("wb") as f:
        f.write(struct.pack("<III", GLB_MAGIC, 2, total))
        f.write(struct.pack("<II", len(js_bytes), CHUNK_JSON))
        f.write(js_bytes)
        f.write(struct.pack("<II", len(buf), CHUNK_BIN))
        f.write(buf)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("source")
    ap.add_argument("--anim", default=None,
                    help="animation name or substring, e.g. Man_Idle")
    ap.add_argument("--time", type=float, default=None,
                    help="seconds into the clip; default is its midpoint")
    ap.add_argument("--phases", type=int, default=1,
                    help="bake N evenly spaced frames of the clip")
    ap.add_argument("--out", default=None)
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    src = Path(args.source)
    js, bin_ = read_glb(src)

    anim_idx = None
    if args.anim:
        for i, a in enumerate(js.get("animations", [])):
            if args.anim.lower() in a.get("name", "").lower():
                anim_idx = i
                break
        if anim_idx is None:
            names = [a.get("name") for a in js.get("animations", [])]
            raise SystemExit(f"no animation matching {args.anim!r}; have {names}")

    dur = anim_duration(js, bin_, anim_idx) if anim_idx is not None else 0.0

    outs = []
    for k in range(max(1, args.phases)):
        if args.time is not None:
            t = args.time
        elif args.phases > 1:
            t = dur * k / args.phases
        else:
            t = dur * 0.5
        posed, lo, hi = bake(js, bin_, anim_idx, t)
        size = hi - lo

        if args.phases > 1:
            d = Path(args.out_dir or src.parent)
            d.mkdir(parents=True, exist_ok=True)
            out = d / f"{src.stem}_{args.anim or 'bind'}_{k:02d}.glb"
        else:
            out = Path(args.out) if args.out else src.with_name(src.stem + "_posed.glb")

        write_glb(js, bin_, posed, out)
        outs.append(out)
        print(f"  {out.name:<34} t={t:5.2f}s  bbox "
              f"[{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}]  "
              f"tallest {max(size):.3f}  {out.stat().st_size:,} B")

    print(f"\n  baked {len(outs)} file(s) from {src.name}"
          + (f", clip {js['animations'][anim_idx]['name']!r} ({dur:.2f}s)"
             if anim_idx is not None else ", bind pose"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
