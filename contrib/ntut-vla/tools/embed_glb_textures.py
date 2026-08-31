"""Repack Kenney's car-kit GLBs with their texture INSIDE the file.

Why this is needed. `world.spawn_object_from_file` takes one byte array and
nothing else, so a glTF that references its texture by URI has no way to resolve
it — the sim gets geometry and no colour. Kenney's GLBs do exactly that:

    "images": [{"uri": "Textures/colormap.png", "name": "colormap"}]

Project AirSim's own sample GLBs (AirTaxi, Cesium_Air, BasicLandingPad,
wind_turbine) all carry the image as an embedded bufferView instead, which is the
path the loader is proven on. This script converts the former into the latter.

It matters because the shared 512x512 atlas is where ALL the colour lives —
measured, it is 21.5% orange, 17.8% red, 12.5% white, 8.8% blue, 6.2% green,
3.2% yellow — and each model's UVs pick a different region. Embed the atlas and a
firetruck is red, a taxi is yellow, an ambulance is white, with no /Game/...
material involved. That is the whole point: this build binds exactly one material
(M_Orange) and refuses every other, so colour has to travel inside the mesh file.

Usage:
    python tools/embed_glb_textures.py --zip <kenney_car-kit.zip> --out <dir>

The zip is NOT committed: it is a 4.8 MB third-party asset. Fetch it from
https://opengameart.org/content/car-kit (CC0, by Kenney) and point --zip at it.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import struct
import zipfile

GLB_MAGIC = b"glTF"
JSON_CHUNK = 0x4E4F534A
BIN_CHUNK = 0x004E4942


def _pad4(b: bytes, fill: bytes) -> bytes:
    """glTF requires every chunk to be a multiple of 4 bytes. JSON pads with
    spaces and BIN with zeros — using the wrong filler makes strict loaders
    reject the file even though the length is right."""
    return b + fill * ((4 - len(b) % 4) % 4)


def _split(raw: bytes):
    if raw[:4] != GLB_MAGIC:
        raise ValueError("not a GLB")
    n = struct.unpack("<I", raw[8:12])[0]
    off, js, bn = 12, None, b""
    while off < n:
        ln, ty = struct.unpack("<II", raw[off:off + 8])
        data = raw[off + 8: off + 8 + ln]
        if ty == JSON_CHUNK:
            js = json.loads(data)
        elif ty == BIN_CHUNK:
            bn = data
        off += 8 + ln
    if js is None:
        raise ValueError("GLB has no JSON chunk")
    return js, bn


def embed(raw: bytes, png: bytes) -> bytes:
    """Return a GLB whose images[] are bufferViews rather than URIs."""
    js, bn = _split(raw)
    images = js.get("images") or []
    if not images or "uri" not in images[0]:
        return raw                       # already embedded, or has no texture

    bn = _pad4(bn, b"\x00")
    view = {"buffer": 0, "byteOffset": len(bn), "byteLength": len(png)}
    js.setdefault("bufferViews", []).append(view)
    idx = len(js["bufferViews"]) - 1
    for im in images:
        im.pop("uri", None)
        im["bufferView"] = idx
        im["mimeType"] = "image/png"
    bn = bn + png

    js.setdefault("buffers", [{}])
    js["buffers"][0].pop("uri", None)     # a GLB's buffer 0 IS the BIN chunk
    js["buffers"][0]["byteLength"] = len(bn)

    jb = _pad4(json.dumps(js, separators=(",", ":")).encode("utf-8"), b" ")
    bb = _pad4(bn, b"\x00")
    total = 12 + 8 + len(jb) + 8 + len(bb)
    return (GLB_MAGIC + struct.pack("<II", 2, total)
            + struct.pack("<II", len(jb), JSON_CHUNK) + jb
            + struct.pack("<II", len(bb), BIN_CHUNK) + bb)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--zip", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--glb-dir", default="Models/GLB format/")
    ap.add_argument("--texture", default="Models/GLB format/Textures/colormap.png")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    z = zipfile.ZipFile(args.zip)
    png = z.read(args.texture)
    print(f"[embed] atlas {args.texture} — {len(png)} bytes")

    n_done = n_skip = 0
    for name in z.namelist():
        if not (name.startswith(args.glb_dir) and name.lower().endswith(".glb")):
            continue
        raw = z.read(name)
        new = embed(raw, png)
        (out / pathlib.Path(name).name).write_bytes(new)
        if new is raw:
            n_skip += 1
        else:
            n_done += 1
    print(f"[embed] {n_done} rewritten, {n_skip} already embedded -> {out}")

    # Verify by re-reading, rather than trusting the write.
    bad = []
    for f in sorted(out.glob("*.glb")):
        js, _ = _split(f.read_bytes())
        for im in js.get("images", []):
            if "uri" in im or "bufferView" not in im:
                bad.append(f.name)
    print(f"[verify] {'ALL EMBEDDED' if not bad else 'STILL EXTERNAL: ' + str(bad[:5])}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
