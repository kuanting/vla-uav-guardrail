# Survey: free, openly-licensed 3D car models in glTF/GLB

Date: 2026-08-11. Desk research only. **Nothing was downloaded.** Every size, poly
count and licence below is quoted from the page that publishes it; where a page did
not state a number, this document says so instead of guessing.

Purpose: find a car mesh that can be spawned with
`world.spawn_object_from_file(name, "gltf", byte_array, is_binary, pose, scale, physics)`
so that `moving_car.py` can put *several visually distinct cars* in the Japanese City
map without a UE project rebuild, and so that a colour word in the follow prompt has
something real to bind to. Today only `/Game/Geometry/Materials/M_Orange` renders
correctly, so every car in the scene is orange and the colour half of the task is
untestable.

---

## 1. Two constraints decide this survey, not the model count

### 1.1 The loader is only proven on *textured* GLB

Project AirSim ships four GLB files as the reference payloads for
`spawn_object_from_file` (`D:\ProjectAirSim\repo\client\python\example_user_scripts\assets`).
I parsed their JSON chunks locally with
`C:\Users\natha\.conda\envs\vla-real\python.exe` (header + JSON chunk only, no import,
no simulator):

| sample GLB | size | materials | images | tris | `extensionsUsed` | colour comes from |
|---|---|---|---|---|---|---|
| `Cesium_Air.glb` | 0.59 MB | 2 | 2 | ~5 840 | none | `baseColorTexture` |
| `BasicLandingPad.glb` | 3.30 MB | 1 | 1 | ~92 | none | `baseColorTexture` |
| `wind_turbine.glb` | 0.11 MB | 1 | 1 | ~1 480 | none | `baseColorTexture` |
| `AirTaxi.glb` | 13.64 MB | 1 | 1 | ~72 292 | none | `baseColorTexture` |

Three consequences, and they reorder the whole candidate list:

* **A model that carries an embedded `baseColorTexture` is on the proven path.** This is
  the way out of the material problem: the colour travels inside the GLB, so no
  `/Game/...` material asset is involved and `set_object_material` is not needed.
* **A model whose colour is only `pbrMetallicRoughness.baseColorFactor` (flat RGB, no
  image) is on an unproven path.** Not one shipped sample works that way. If the
  runtime loader builds its dynamic material from the base-colour *texture* slot only,
  every such car renders white/grey — the same failure mode already measured for
  "Yellow" and `MI_Emissive_Red`. **This is exactly how the CC0 low-poly packs
  (Quaternius, Kenney) are built**, so they inherit the known weak point rather than
  escaping it.
* **13.64 MB and 72 k triangles are demonstrably fine**, and no glTF extension is
  exercised by any sample. So prefer plain glTF 2.0; avoid Draco / KTX2-BasisU /
  `KHR_materials_variants` variants — unproven, and a loader that silently ignores
  `KHR_materials_variants` gives you the default paint colour with no way to switch it.

### 1.2 `.glb` or nothing

`spawn_object_from_file` takes **one** byte array. A `.gltf` that references external
`.bin` and `.png` files has no way to resolve those URIs through this API. So:

* `.glb` → `is_binary=True`, single self-contained payload. **Use this.**
* `.gltf` → `is_binary=False`, only viable if it is self-contained with `data:` URIs.
  A downloaded `.gltf` + `.bin` + texture folder must be converted to `.glb` first.

### 1.3 What OWL-ViT needs

Measured on our own flights: OWL-ViT does **not** discriminate colour ("a car" and
"a blue car" returned an identical box, centre 196,73 size 62×19), and the HSV check
supplies all colour discrimination. So the mesh must satisfy two separate things:

* a silhouette OWL-ViT scores as *car* at roughly 40–80 px wide in a 400×225 frame,
  seen from 10–15 m up through a camera pitched 20° down;
* a **large, flat, saturated painted area** for `colour_match()` to sample. A photoreal
  car with a dark, high-gloss, environment-reflecting clearcoat can read as near-black
  in HSV even when a human sees "red". Matte or semi-gloss paint is *better* for us
  than showroom-grade paint.

OWL-ViT is trained on photographs. A chunky toy-proportioned car (fat wheels, 2:1
length-to-height, no window/pillar detail) is the risky class, and at 60 px wide the
silhouette is nearly all the evidence the detector has. I have **not** measured
detection on any of these meshes — §6 gives the cheap probe that would.

---

## 2. Candidates

### A. Khronos glTF Sample Assets — **Car Concept**

* Page: https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept
* Direct GLB (from the GitHub contents API, not fetched): `https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/CarConcept/glTF-Binary/CarConcept.glb` — **11 778 688 bytes**
* Licence: **CC BY 4.0 International**. `Models.md` names the copyright holder
  "Darmstadt Graphics Group GmbH"; the README credits © 2024 Darmstadt Graphics Group
  GmbH, model and textures by Eric Chadwick, concept based on a public-domain "Unity Fan"
  model. `LICENSE.md` is present in the model folder. **Attribution required** — a line
  in the repo README/docs is enough.
* Format: `.glb` in `glTF-Binary/` (**`is_binary=True`**), plus `glTF/` (11.7 MB, external
  PNGs), `glTF-JPG`, `glTF-WEBP`, `glTF-KTX-BasisU-Draco` (10.3 MB). **Take the GLB; skip
  the Draco/KTX one.**
* Poly count: not stated by Khronos.
* Textures/materials: **yes, full PBR** — metallic flake normal map, clearcoat, and
  *material variants* named Carmine Candy, Pearly Swirly, Torched Graphite.
* Realism: **photoreal.** This is the highest-fidelity car with a licence I could verify
  end-to-end and a direct download URL that needs no account.
* Caveats: it is a *concept* car (sleek, futuristic), not a street sedan — car-like
  silhouette, but not the family shape OWL-ViT sees most often in training photos. The
  three paint colours are `KHR_materials_variants`, an extension no shipped AirSim
  sample uses: expect to get whichever variant is default and **no runtime colour
  switching**. Clearcoat + flakes is precisely the "photoreal paint reads dark in HSV"
  risk from §1.3.

### B. Khronos glTF Sample Assets — **ToyCar**

* Page: https://github.com/KhronosGroup/glTF-Sample-Assets/blob/main/Models/ToyCar/README.md
* Direct GLB: `https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/ToyCar/glTF-Binary/ToyCar.glb` — **5 422 412 bytes**
* Licence: **CC0 / public domain** (README: model by Guido Odendahl, materials edited by
  Eric Chadwick). Cleanest licence in this survey. No attribution required.
* Format: `.glb` (`is_binary=True`).
* Poly count: not stated.
* Textures: yes, PBR, and it deliberately exercises `KHR_materials_sheen`,
  `KHR_materials_transmission`, `KHR_materials_clearcoat` — three extensions no AirSim
  sample uses. A loader that ignores them still renders base colour, so this is a
  cosmetic risk, not a load failure.
* Realism: **photoreal *rendering* of a toy.** Bounding box is ~7.3 cm
  (Khronos issue #63 discusses the model being tiny), so reaching 3.7 m means `scale`
  ≈ 50×. Scaling a toy to car size keeps toy proportions.
* Verdict: best licence, wrong object. Good as a *pipeline* test payload, weak as a
  detection target.

### C. Sketchfab, realistic street cars under a verified CC licence

Sketchfab is now Fab; free CC models are downloadable but **a logged-in account is
required** (I could not verify the help article — its TLS chain failed my fetch — but
every model page shows a login/sign-up prompt around the download control). I cannot
create accounts; the human must do this download.

Sketchfab serves an auto-converted glTF alongside the original, so the format is
usually available; whether it arrives as `.glb` or `.gltf`+`.bin` must be checked after
download (it decides `is_binary`).

| model | URL | licence as labelled on the page | geometry | textures |
|---|---|---|---|---|
| Generic Sedan Car — MMC Works | https://sketchfab.com/3d-models/generic-sedan-car-58c33766470d46e7b2aed542650494e5 | **CC Attribution** (CC BY) | 113.2 k tris / 58.8 k verts | yes — paint, bottom, mechanics, interior maps |
| Sedan (glb format, with interior) — 3DHA | https://sketchfab.com/3d-models/sedan-glb-format-with-interior-fe0d953112044d6893d04dbbec75ac82 | **CC Attribution** (CC BY) | 18.2 k tris / 9.5 k verts | page does not say |

* Realism: **these are the right shape** — ordinary, unbranded, four-door sedans. Closest
  match to "a car" as OWL-ViT learned it, and both are within the proven size/complexity
  envelope (§1.1).
* Flag on the MMC Works model: the page carries a **NoAI designation** (no use in
  generative-AI datasets or development). Using it as a simulator prop to test a
  perception stack is not dataset training, but this project is a VLA project — read
  the term before it goes near `dataset/` or `training/`.
* Flag on Sketchfab generally: **prefer unbranded "generic sedan" uploads.** Branded
  models (Mazda RX-7, Mercedes, McLaren, Ferrari) may be CC-BY on the *mesh* while the
  vehicle design and marks remain the manufacturer's. Not worth the argument in an
  academic repo.

### D. Poly Pizza / Quaternius — **Cars Bundle**

* Bundle page: https://poly.pizza/bundle/Cars-Bundle-FE5IWe6OMk (Quaternius, 8 models:
  sports car, police car, taxi, SUV, plain cars)
* Author's own page: https://quaternius.com/packs/cars.html
* Example model page: https://poly.pizza/m/BwwnUrWGmV (Police Car, published 2021-08-15)
* Author index: https://poly.pizza/u/Quaternius
* Licence: **Public Domain (CC0)**, stated on both the Poly Pizza bundle page and
  quaternius.com ("free to use in both personal and commercial projects"). No attribution
  required. Cleanest possible licence position.
* Format: Poly Pizza offers **FBX and glTF/GLB** per model and for the bundle
  ("Download GLTF"). quaternius.com itself offers only FBX, OBJ and .blend — so **go
  through Poly Pizza for glTF.** Whether the file is `.glb` or `.gltf` decides
  `is_binary`; the page does not say, and no direct CDN URL is exposed in the page
  markup I could read.
* Poly count / file size: **not published** on either the bundle page or the individual
  model pages.
* Textures: **quaternius.com marks the Cars Pack "Textured: ✗".** Flat material colours
  only — i.e. the `baseColorFactor` path from §1.1 that no AirSim sample exercises.
* Realism: **stylised low-poly.** Rounded, toy-like proportions, no window/pillar detail.
* Verdict: perfect licence, wrong two properties. If the loader honours `baseColorFactor`
  this pack does give 8 distinct flat-coloured cars for free, which is a lot of colour
  variety very cheaply — but both the render and the detection are unproven, and it
  fails the "does not test anything" bar if a stylised car turns out to be undetectable.

### E. Kenney — **Car Kit**

* Page: https://kenney.nl/assets/car-kit · mirrors: https://kenney-assets.itch.io/car-kit ,
  https://opengameart.org/content/car-kit
* Direct zip visible on the page: `https://kenney.nl/media/pages/assets/car-kit/1a312ec241-1775131960/kenney_car-kit.zip`
* Licence: **CC0 1.0 Universal** — "You're allowed to use these game assets in any
  project including commercial ones." No attribution required.
* Format: **OBJ, FBX and glTF** (itch page). Zip is **3.4 MB** for **45+ assets** (sedan,
  van, ambulance, plus 8 separate wheel models and debris). Whether the glTF folder holds
  `.glb` or `.gltf`+`.bin` must be confirmed after unzip — that is the `is_binary` flag.
* Poly count: not stated. 3.4 MB for 45 models implies very low poly.
* Textures: not stated on the page. Kenney's 3D kits are flat-colour or single small
  palette texture; **treat as the unproven `baseColorFactor` path until the GLB is
  probed** (the probe in §6 answers this in one command, offline).
* Realism: **clearly stylised / toy.** Chunky bodies, oversized wheels. Of everything
  here this is the least likely to be detected as "a car" from 12 m up at 60 px wide.
* Repackaged as GLB by a third party: "Car Kit GLB Pack — 50 Free CC0 3D Models",
  https://eclair-assets.itch.io/car-kit-glb-pack-50-free-cc0-3d-models — **50 GLB files,
  1.3 MB total**, name-your-own-price, CC0, and it explicitly credits Kenney as the
  original and calls itself "a convenience redistribution that keeps the license/source
  information together". Format-wise this is exactly what we want (GLB, tiny). 1.3 MB /
  50 models ≈ 26 KB each confirms flat-colour low-poly. **Licence is inherited, not
  original** — CC0 permits this redistribution, but the authority is Kenney's page, so
  cite Kenney, not the repackager.

### F. Objaverse (Allen Institute) — bulk source

* Pages: https://objaverse.allenai.org/objaverse-1.0/ ·
  https://huggingface.co/datasets/allenai/objaverse · API docs
  https://objaverse.allenai.org/docs/objaverse-1.0/
* Licence: **the dataset as a whole is ODC-By 1.0; each object keeps its own licence, and
  the per-object metadata states which** (the 800 K objects are Sketchfab CC uploads, so
  the mix is CC0 / CC-BY / CC-BY-SA / CC-BY-NC). Objaverse-XL adds GitHub-sourced objects
  with a wider, messier licence spread.
* Format: **`.glb`** — the native distribution format. No account needed for the public
  HF dataset.
* Realism: the pool contains hundreds of realistic cars. That is the point of it.
* This is the only source that could realistically give **a dozen visually distinct,
  differently coloured, realistic cars at once** — the actual end goal.
* Cost, stated honestly: you must filter by licence per object (and drop NC ones if the
  work is ever considered commercial), then normalise scale, up-axis and forward-axis per
  model, and inspect each one. Nothing is curated for you.

### G. Epic / Fab — the UE-native route (requires a project rebuild)

* **City Sample Vehicles** — https://www.fab.com/listings/2909157b-ddfa-4cef-a925-69dc2467021f
  (my fetch returned HTTP 403; details below come from Epic's docs and press coverage).
  **13 driveable vehicles** from *The Matrix Awakens* City Sample: several cars, pickup,
  semi, garbage truck, taxi, bus, delivery van. Free. Obtained via Fab → "Add to My
  Library" → Epic Games Launcher → Unreal Engine → Library → Fab Library.
* **Vehicle Variety Pack** (+ Volume 2) — Epic's "Free For Life" collection, 5 game-ready
  drivable vehicles with interiors (sedan, SUV, box truck, campervan).
  https://www.unrealengine.com/marketplace/en-US/product/bbcb90a03f844edbb20c8b89ee16ea32/reviews
* Licence: **I could not verify this and I am not going to assert it.** Fab changed the
  licensing of formerly UE-Marketplace content, and community threads disagree about
  whether specific Epic-published free packs are under the Epic Content License
  (Unreal-only) or the Fab Standard License (any engine). Epic's content EULA is at
  https://www.unrealengine.com/eula/content. **Read the licence box on the listing while
  logged in before relying on this.** For our purpose — assets used inside a UE 5.7
  project — even the restrictive reading is satisfied, but say so from the listing, not
  from this document.
* Format: **Unreal-native `.uasset`/Blueprint** (City Sample vehicles live at
  `Content/Vehicle/vehCar_vehicle02` etc.), **not glTF.** So this route does **not** go
  through `spawn_object_from_file`. It means: migrate the assets into `PASBlocks`,
  rebuild and re-cook, and reference them by `/Game/...` asset path like
  `SM_Offroad_Body` today.
* Realism: **the best in this survey by a wide margin** — these are AAA photoreal city
  cars with proper material instances, and material instances are the mechanism our
  colour problem actually lives in, so they come with a real chance of working colour
  variants.
* Cost, not a blocker: one UE project rebuild, plus the same "does this material render"
  verification that already burned us on blue/yellow. It also does not give the
  no-rebuild, spawn-from-bytes property that motivated this survey.

### H. Checked and rejected (recorded so nobody re-searches these)

* **Poly Haven — Covered Car**, https://polyhaven.com/a/covered_car — CC0, photoreal,
  glTF, 50.41 MB, no login. Rejected: it is a car *under a tarpaulin*. A featureless
  lump; OWL-ViT has no car silhouette to find. Poly Haven's model library otherwise has
  no street cars (`old_tyre` is the only other vehicle-adjacent asset).
* **three.js `examples/models/gltf/ferrari.glb`** — a photoreal car GLB with a direct,
  no-login GitHub URL, which is why it is tempting. **Rejected on licence:** the
  directory (https://github.com/mrdoob/three.js/tree/dev/examples/models/gltf) has no
  LICENSE or README covering the models, and the only provenance is a credit line in the
  example page ("Ferrari 458 Italia model by vicent091036"). three.js itself is MIT; that
  does not automatically cover third-party example assets. Plus it is a branded Ferrari.
  **Do not use without an explicit grant from the author.**
* **Sketchfab "FREE Concept Car 003 / 006 / 025" by unityfan777** —
  e.g. https://sketchfab.com/3d-models/free-concept-car-025-public-domain-cc0-e3a65443d3e44c33b594cec591c01c05 .
  **Licence cannot be verified: the page's licence label reads "Free Standard" (a
  Sketchfab/Fab store licence, not a CC licence) while the author's description says
  "public domain (CC0) … No attribution required".** Those two do not agree, and a wrong
  licence claim is worse than no candidate. Also impractical: **483.7 k triangles**
  (Car 025) and **761.7 k** (Car 003) — 7–10× the heaviest shipped sample.
* **Meshy free asset library**, https://www.meshy.ai/tags/car — offers `.glb` with
  embedded PBR (Albedo/Normal/Roughness) and claims premade assets are CC0 while
  free-plan *generations* are CC BY 4.0 requiring credit to Meshy. **Licence wording
  found only in marketing copy on a tag page, not in a terms document — unverified.**
  Also AI-generated: shapes are typically soft and topologically noisy, which is the
  wrong direction for a detection target. Account required for download.
* **OpenGameArt CC0 vehicle collection**,
  https://opengameart.org/content/cc0-3d-vehicles-and-cars — contains CC0 entries incl.
  Fiat Topolino 1936, Mercedes W136, Ford Model-T, plus several low-poly cars. Formats and
  texture status are **not listed at collection level** and must be checked per item;
  mostly `.blend`/`.obj`, and the interesting ones are 1930s–50s vehicles, not modern
  street cars.
* **Smithsonian Open Access 3D** (https://www.si.edu/openaccess, https://3d.si.edu) —
  genuinely CC0, genuinely glTF/GLB, ~2 000+ scanned objects. **I found no ordinary
  street car**; the collection is museum objects (Apollo 11 command module, specimens,
  sculpture). Not worth further search time for this use case.
* **ShapeNet / Google Scanned Objects** — ShapeNet has thousands of car meshes but is
  research-use-only (not an open licence), untextured, and OBJ. GSO is CC-BY but is
  scanned household items and toys. Neither fits.
* **Cesium Milk Truck** (in the Khronos set) — CC-BY 4.0 with trademark limitations;
  cartoon truck. Mentioned only for completeness.
* **TurboSquid / CGTrader / RenderHub free sections** — "free" there means a proprietary
  royalty-free EULA, not an open licence, and terms vary per item. Excluded on principle:
  not openly licensed.

---

## 3. Ranking for *this* use case

The task is: 3.7 × 1.8 × 1.2 m car, viewed from 10–15 m up at 400×225, must be detected
by OWL-ViT as "a car", must render in a colour `colour_match()` can classify, and should
arrive as a single `.glb` byte array.

1. **Khronos Car Concept** (CC BY 4.0, GLB, 11.78 MB) — *first thing to try.* Only
   candidate that is simultaneously photoreal, textured, verified-licence, and directly
   downloadable with no account. Risks: concept-car silhouette; clearcoat paint may read
   dark in HSV; colour variants need an extension we have not exercised.
2. **Sketchfab "Generic Sedan Car" (MMC Works) and "Sedan with interior" (3DHA)**,
   both CC BY — *the right object.* Ordinary unbranded sedans, textured, 18 k–113 k tris,
   well inside the proven envelope. Costs: a Sketchfab/Fab login (the human must do the
   download), attribution, and reading the NoAI term on the MMC Works one. If the goal is
   "will OWL-ViT call it a car", this is the most honest test.
3. **Objaverse CC0/CC-BY subset** — *the only path to a whole city of distinct, realistic,
   differently coloured cars.* Highest ceiling, highest labour: per-object licence
   filtering plus scale/axis normalisation. Right answer if this stops being a one-car
   demo.
4. **Epic City Sample Vehicles / Vehicle Variety Pack** — *best-looking cars available,
   and the only ones with production material instances*, which is where our colour
   problem actually lives. Not a glTF spawn: costs a `PASBlocks` rebuild and re-cook, and
   the licence must be read on the Fab listing first.
5. **Khronos ToyCar** (CC0, GLB, 5.42 MB) — safest licence in the survey, and a fine
   payload for proving the spawn path end to end. As a detection target it is a toy
   scaled ×50.
6. **Quaternius Cars Bundle via Poly Pizza** (CC0, 8 models, glTF) — unbeatable licence,
   8 colours for free, no attribution. But stylised *and* marked untextured, so it is
   doubly exposed: the render may come out grey and the silhouette may not detect. Use it
   to test *many cars at once* only after the loader's `baseColorFactor` behaviour is
   known.
7. **Kenney Car Kit** (CC0, 45+ assets, 3.4 MB) / the 1.3 MB **GLB repackage** — same
   licence virtue, most stylised geometry of all. Best use is as a cheap probe payload
   (see §6), not as the demo car.

Rejected outright: Poly Haven Covered Car, three.js `ferrari.glb`, unityfan777 concept
cars, Meshy, ShapeNet, the commercial "free" marketplaces. Reasons in §2H.

**Licences I could not verify, stated plainly:** the unityfan777 Sketchfab cars
(page label contradicts the author's description), the Meshy free library (marketing copy
only), Epic's Fab listings (403 on fetch; Fab's licence model changed and the community
disagrees), and the *contents* of the Kenney and Quaternius zips regarding texture
inclusion (the licences are certain, the texture status is not). Treat all four as open
questions, not as findings.

---

## 4. Which candidate actually solves the material problem

The reason to prefer a textured GLB over the current `SM_Offroad_Body` + `M_Orange`
arrangement: **the colour stops being a `/Game` material asset.** A GLB with an embedded
`baseColorTexture` carries its paint as an image inside the byte array, which is the code
path all four shipped AirSim samples use. That sidesteps the failure mode we already
measured — blue rendering invisible, "Yellow" rendering pale grey, `MI_Emissive_Red`
rendering pale pink-white.

The corollary is that colour variety then comes from **editing the embedded texture**, not
from `set_object_material`: recolour the base-colour PNG inside a copy of the GLB and
spawn N differently painted copies of the same mesh. That is fully offline, needs no UE
asset, and makes `COLOUR_HUE` finally meaningful. It also means candidate quality should
be judged on "does it have one big paintable base-colour texture region", which the
Sketchfab sedans (separate paint map) satisfy better than Car Concept (flakes + clearcoat
+ variants).

## 5. Dimensions to remember when one is adopted

`CarSpec` in `demo/moving_car.py` declares 3.7 × 1.8 × 1.2 m with `unit_scale=True`, and
the follower's inverse-range proxy is box width, so **body length drives the range
estimate**. Any replacement mesh changes that calibration:

* glTF is metres by convention, but only by convention. Car Concept is a full-size car
  (longer than 3.7 m); ToyCar's bounding box is ~7.3 cm and needs `scale ≈ 50`.
* Up-axis and forward-axis differ per source. `pose_at()` interpolates heading the
  short way round, so a mesh whose forward is +X instead of −Y will drive sideways with
  no error message.
* Update `CarSpec` dimensions and `desc_match` together, or the HSV verification and the
  clearance policies will be reasoning about the old box.

## 6. Cheapest way to turn this survey into facts (no simulator needed for step 1)

1. **Offline, zero risk:** once the human has downloaded any candidate, run the same
   header/JSON probe I used in §1.1 against it and record `materials`, `images`,
   `extensionsUsed`, triangle count and whether colour is a texture or a factor. That
   answers the single biggest open question — whether the CC0 low-poly packs are on the
   proven path — without touching UE.
2. **One simulator pass, main session only:** spawn *one* candidate GLB at a fixed pose
   with `spawn_object_from_file(..., is_binary=True, enable_physics=False)`, grab one
   FrontCamera frame from 12 m, and run the existing OWL-ViT + `colour_match()` pair on it
   offline. Two numbers decide everything: does "a car" return a box, and does the HSV
   check name the colour. `demo/out/<tag>/detections.jsonl` already has the right shape
   for logging this.
3. Only then decide between the glTF route (no rebuild, colour by texture edit) and the
   Epic route (rebuild, colour by material instance).

## 7. Sources

* Quaternius Cars Pack — https://quaternius.com/packs/cars.html
* Quaternius Cars Bundle on Poly Pizza — https://poly.pizza/bundle/Cars-Bundle-FE5IWe6OMk ·
  https://poly.pizza/u/Quaternius · https://poly.pizza/m/BwwnUrWGmV · https://poly.pizza/search/car
* Kenney Car Kit — https://kenney.nl/assets/car-kit · https://kenney-assets.itch.io/car-kit ·
  https://opengameart.org/content/car-kit
* Kenney Car Kit GLB repackage — https://eclair-assets.itch.io/car-kit-glb-pack-50-free-cc0-3d-models
* Khronos glTF Sample Assets — https://github.com/KhronosGroup/glTF-Sample-Assets ·
  https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept ·
  https://github.com/KhronosGroup/glTF-Sample-Assets/blob/main/Models/ToyCar/README.md ·
  https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/Models.md
* Sketchfab — https://sketchfab.com/3d-models/generic-sedan-car-58c33766470d46e7b2aed542650494e5 ·
  https://sketchfab.com/3d-models/sedan-glb-format-with-interior-fe0d953112044d6893d04dbbec75ac82 ·
  https://sketchfab.com/3d-models/free-concept-car-025-public-domain-cc0-e3a65443d3e44c33b594cec591c01c05 ·
  https://sketchfab.com/3d-models/free-concept-car-003-public-domain-cc0-77664fc474c444f4947e9834ed0d30ad ·
  https://sketchfab.com/tags/cc0
* Objaverse — https://objaverse.allenai.org/objaverse-1.0/ ·
  https://huggingface.co/datasets/allenai/objaverse · https://objaverse.allenai.org/docs/objaverse-1.0/
* Poly Haven — https://polyhaven.com/a/covered_car · https://polyhaven.com/models
* Epic / Fab — https://www.fab.com/listings/2909157b-ddfa-4cef-a925-69dc2467021f (403 on fetch) ·
  https://dev.epicgames.com/documentation/unreal-engine/city-sample-project-unreal-engine-demonstration ·
  https://www.unrealengine.com/marketplace/en-US/product/bbcb90a03f844edbb20c8b89ee16ea32/reviews ·
  https://www.unrealengine.com/eula/content
* OpenGameArt CC0 vehicles — https://opengameart.org/content/cc0-3d-vehicles-and-cars
* Smithsonian Open Access — https://www.si.edu/openaccess · https://3d.si.edu
* Meshy — https://www.meshy.ai/tags/car
* three.js example models — https://github.com/mrdoob/three.js/tree/dev/examples/models/gltf
* CC0 asset index — https://github.com/madjin/awesome-cc0
