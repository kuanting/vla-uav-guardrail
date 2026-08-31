# Midterm deck build

Builds `docs/VLA-Guardrail-Midterm-Aug2026.pptx` on the nathan-deck design
system (Poppins, teal `#249DB2`, rounded title pills, page badges).

## Run

```bash
npm install          # pptxgenjs, react, react-dom, react-icons, sharp
node build_midterm_deck.js
```

The build reads `demo/out/<tag>/metrics.json`, `flight_log.jsonl`,
`view/recorder.json` and `docs/data/chase_resolution_study.json`, then repairs
and audits the package before it will report success.

## Two rules the script enforces

**Every number is read from the artefacts at build time.** Nothing is typed into
a slide. `tools/build_report_results.py` generates section 6 of the written
report from the same files, so the deck and the report cannot disagree with each
other or with the flight logs.

**No video is embedded.** The previous deck reached **486 MB** because six raw
`.mp4` files were placed inside it, which is unusable as an email attachment.
Videos ship alongside and are referenced by filename; the build refuses any deck
over 10 MB.

## Why there is a repair step

pptxgenjs 4.0.1 emits a package PowerPoint will not open, and PowerPoint reports
this only as *"PowerPoint could not open the file"* — no part name, no slide
number, no reason. Three separate defects were found by bisecting the deck one
slide at a time:

| Defect | Effect |
|---|---|
| `[Content_Types].xml` declares overrides for `slideMaster2..7` and layouts that were never written | Package refused |
| ZIP directory entries (`ppt/`, `ppt/media/`, …) which are not OPC parts | Non-conformant |
| A `card()` shorter than its own header produced `<a:ext cy="-100584">` | Package refused |

The first two are repaired by `tools/fix_pptx_package.py`. The third is now
impossible: `card()` throws at build time with the card title and the height it
needs, and `tools/audit_pptx.py` fails the build if any negative extent, dangling
override, dangling relationship or directory entry survives.

This is easy to misdiagnose, because a deck PowerPoint has ever opened and
re-saved comes back normalised. An older deck in the same folder therefore opens
while a freshly generated one does not, which points suspicion at the slide
content rather than the packaging.

## Export to PDF

```bash
powershell -File ../office_to_pdf.ps1 -Path ../../docs/VLA-Guardrail-Midterm-Aug2026.pptx
```
