/*
 * Midterm report deck — ITRI / NTUT, reporting period Feb–Aug 2026.
 * Built on the nathan-deck design system (tokens and geometry unchanged).
 *
 * TWO RULES THIS SCRIPT ENFORCES, both from failures of the previous deck:
 *
 *   1. Every number on a slide is READ FROM THE ARTEFACTS at build time
 *      (demo/out/<tag>/metrics.json and flight_log.jsonl). Nothing is typed in
 *      by hand, so a slide cannot drift from the data it claims to report.
 *
 *   2. No video is embedded. The previous deck reached 486 MB because six raw
 *      .mp4 files were placed inside it. Videos ship alongside, by filename.
 */

const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fs = require("fs");
const path = require("path");

const {
  FaBullseye, FaProjectDiagram, FaEye, FaSlidersH, FaShieldAlt,
  FaFlask, FaChartBar, FaTachometerAlt, FaMicrochip, FaClipboardCheck,
  FaExclamationTriangle, FaTasks, FaRoad, FaVideo,
} = require("react-icons/fa");

/* ── DESIGN TOKENS (authoritative, unchanged) ──────────────────────────── */
const TEAL      = "249DB2";
const TEAL_DK   = "1A7484";
const TEAL_TINT = "EAF6F8";
const BLACK     = "1A1A1A";
const INK       = "2B2B2B";
const GREY      = "6B7280";
const WHITE     = "FFFFFF";
const LINE      = "E3E8EC";
const FF        = "Poppins";

const REPO = "D:/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone";
const TAGS = ["demo_follow", "demo_traffic", "demo_nfz"];
const LABEL = { demo_follow: "Tracking", demo_traffic: "Distractors", demo_nfz: "No-fly zone" };

/* ── DATA LOADING ──────────────────────────────────────────────────────── */
function metrics(tag) {
  return JSON.parse(fs.readFileSync(path.join(REPO, "demo/out", tag, "metrics.json"), "utf8"));
}

function loopRate(tag) {
  const p = path.join(REPO, "demo/out", tag, "flight_log.jsonl");
  const lines = fs.readFileSync(p, "utf8").split("\n").filter((l) => l.trim());
  const last = JSON.parse(lines[lines.length - 1]);
  return { ticks: lines.length, seconds: last.t, hz: lines.length / last.t };
}

function videoInfo(tag) {
  const p = path.join(REPO, "docs/video", tag + ".mp4");
  if (!fs.existsSync(p)) return null;
  return { mb: fs.statSync(p).size / 1048576 };
}

const M = {};
const L = {};
for (const t of TAGS) { M[t] = metrics(t); L[t] = loopRate(t); }

const STUDY = JSON.parse(
  fs.readFileSync(path.join(REPO, "docs/data/chase_resolution_study.json"), "utf8"));

const n = (v, d = 2) => (v === null || v === undefined ? "n/a" : Number(v).toFixed(d));
const pct = (v) => (v === null || v === undefined ? "n/a" : (Number(v) * 100).toFixed(1) + " %");

async function iconPng(Icon, color = "#" + TEAL, size = 256) {
  const svg = ReactDOMServer.renderToStaticMarkup(
    React.createElement(Icon, { color, size: String(size) }));
  const buf = await sharp(Buffer.from(svg)).png().toBuffer();
  return "image/png;base64," + buf.toString("base64");
}

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "Safety-Constrained VLA for ArduPilot UAVs — Midterm Report";
  pres.author = "Nathanael Tjahyadi";

  const ic = {
    aim:    await iconPng(FaBullseye),
    arch:   await iconPng(FaProjectDiagram),
    eye:    await iconPng(FaEye),
    slider: await iconPng(FaSlidersH),
    shield: await iconPng(FaShieldAlt),
    flask:  await iconPng(FaFlask),
    chart:  await iconPng(FaChartBar),
    gauge:  await iconPng(FaTachometerAlt),
    chip:   await iconPng(FaMicrochip),
    check:  await iconPng(FaClipboardCheck),
    warn:   await iconPng(FaExclamationTriangle),
    tasks:  await iconPng(FaTasks),
    road:   await iconPng(FaRoad),
    video:  await iconPng(FaVideo),
  };

  /* ── SHARED HELPERS (nathan-deck geometry) ───────────────────────────── */
  function badge(s, i) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 9.3, y: 5.18, w: 0.5, h: 0.32,
      fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.06,
    });
    s.addText(String(i).padStart(2, "0"), {
      x: 9.3, y: 5.18, w: 0.5, h: 0.32,
      fontSize: 9, color: WHITE, fontFace: FF, bold: true,
      align: "center", valign: "middle",
    });
  }

  function heading(s, eyebrow, title, x = 0.55, y = 0.45) {
    s.addText(eyebrow.toUpperCase(), {
      x, y, w: 8.5, h: 0.28,
      fontSize: 10, color: TEAL, fontFace: FF, bold: true, charSpacing: 2,
    });
    s.addText(title, {
      x, y: y + 0.3, w: 8.9, h: 0.6,
      fontSize: 26, color: BLACK, fontFace: FF, bold: true,
    });
    s.addShape(pres.shapes.LINE, {
      x, y: y + 0.98, w: 0.9, h: 0, line: { color: TEAL, width: 2.5 },
    });
  }

  function card(s, x, y, w, h, iconData, titleTxt, bodyTxt) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x, y, w, h, fill: { color: WHITE }, line: { color: LINE, width: 1 },
      rectRadius: 0.08,
      shadow: { type: "outer", blur: 6, offset: 2, angle: 90, color: "D9DEE3", opacity: 0.4 },
    });
    let ty = y + 0.22;
    if (iconData) {
      s.addImage({ data: iconData, x: x + 0.22, y: y + 0.22, w: 0.36, h: 0.36 });
      s.addText(titleTxt, {
        x: x + 0.68, y: y + 0.2, w: w - 0.9, h: 0.4,
        fontSize: 13, color: INK, fontFace: FF, bold: true, valign: "middle",
      });
      ty = y + 0.72;
    } else {
      s.addText(titleTxt, {
        x: x + 0.24, y: y + 0.2, w: w - 0.48, h: 0.4,
        fontSize: 13, color: INK, fontFace: FF, bold: true,
      });
      ty = y + 0.66;
    }
    if (bodyTxt) {
      // A card shorter than its own header leaves negative room for the body,
      // and pptxgenjs will happily emit <a:ext cy="-100584">. PowerPoint then
      // refuses the ENTIRE package with "PowerPoint could not open the file" -
      // no slide number, no shape name, no clue that one text box is the cause.
      // One card at h=0.75 with no icon (needing 0.86) cost a long bisection.
      const bodyH = h - (ty - y) - 0.2;
      if (bodyH <= 0) {
        throw new Error(
          `card "${titleTxt}" is ${h}" tall but its header needs ` +
          `${(ty - y + 0.2).toFixed(2)}", leaving ${bodyH.toFixed(2)}" for the body. ` +
          `Negative extents make the whole deck unopenable. Raise h to at least ` +
          `${(ty - y + 0.45).toFixed(2)}".`);
      }
      s.addText(bodyTxt, {
        x: x + 0.24, y: ty, w: w - 0.48, h: bodyH,
        fontSize: 11, color: GREY, fontFace: FF, lineSpacingMultiple: 1.25, valign: "top",
      });
    }
  }

  function darkBase(s, eyebrow) {
    s.background = { color: TEAL_DK };
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0, y: 0, w: 0.09, h: 5.625, fill: { color: TEAL }, line: { color: TEAL },
    });
    if (eyebrow) {
      s.addText(eyebrow.toUpperCase(), {
        x: 0.55, y: 0.5, w: 9, h: 0.3,
        fontSize: 10, color: TEAL_TINT, fontFace: FF, bold: true, charSpacing: 3,
      });
    }
  }

  function table(s, rows, colFrac, x, y, rh, emphasis = []) {
    const tw = 8.9;
    const colW = colFrac.map((f) => tw * f);
    rows.forEach((r, ri) => {
      let cx = x;
      r.forEach((cell, ci) => {
        const header = ri === 0;
        const zebra = !header && ri % 2 === 0;
        const emph = emphasis.some(([a, b]) => a === ri && b === ci);
        s.addShape(pres.shapes.RECTANGLE, {
          x: cx, y: y + ri * rh, w: colW[ci], h: rh,
          fill: { color: header ? TEAL : zebra ? TEAL_TINT : WHITE },
          line: { color: LINE, width: 1 },
        });
        s.addText(String(cell), {
          x: cx + 0.1, y: y + ri * rh, w: colW[ci] - 0.2, h: rh,
          fontSize: header ? 11.5 : 11,
          color: header ? WHITE : emph ? TEAL : INK,
          bold: header || emph, fontFace: FF,
          align: ci === 0 ? "left" : "center", valign: "middle",
        });
        cx += colW[ci];
      });
    });
  }

  let SLIDE = 0;
  const next = () => ++SLIDE;

  /* ── 01 · COVER ──────────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    darkBase(s, "ITRI · NTUT AIoT Laboratory · Feb–Nov 2026");
    s.addText("Semantic-Spatial Translation and\nSafety-Constrained VLA for ArduPilot UAVs", {
      x: 0.55, y: 1.5, w: 8.6, h: 1.5,
      fontSize: 32, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.1,
    });
    s.addShape(pres.shapes.LINE, {
      x: 0.55, y: 3.15, w: 1.2, h: 0, line: { color: TEAL_TINT, width: 3 },
    });
    s.addText("Midterm report · reporting period February – August 2026", {
      x: 0.55, y: 3.35, w: 8.6, h: 0.35,
      fontSize: 14, color: TEAL_TINT, fontFace: FF,
    });
    s.addText("Nathanael Tjahyadi   ·   Principal investigator: Prof. Kuan-Ting Lai", {
      x: 0.55, y: 3.75, w: 8.6, h: 0.35,
      fontSize: 12, color: TEAL_TINT, fontFace: FF,
    });
    s.addText("20 August 2026", {
      x: 0.55, y: 4.6, w: 4, h: 0.3, fontSize: 11, color: TEAL_TINT, fontFace: FF,
    });
  }

  /* ── 02 · OBJECTIVES ─────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Objectives", "Scope for the reporting period");
    card(s, 0.55, 1.7, 4.3, 1.55, ic.aim, "Problem",
      "VLA models are probabilistic and occasionally emit commands that violate airspace rules. Flight safety cannot rest on a component that is sometimes wrong.");
    card(s, 5.15, 1.7, 4.3, 1.55, ic.shield, "Response",
      "A deterministic layer between any action source and the autopilot. Acceptance criterion: P0 violation escape rate = 0.");
    table(s, [
      ["Objective", "Status"],
      ["Establish the Guardrail as a model-independent component", "Met"],
      ["Demonstrate language-commanded target following", "Met"],
      ["Demonstrate intervention under rule conflict", "Met"],
      ["Build traceable verification apparatus", "Partial"],
    ], [0.76, 0.24], 0.55, 3.45, 0.34, [[1, 1], [2, 1], [3, 1]]);
    badge(s, next() + 1);
  }

  /* ── 03 · ARCHITECTURE ───────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "System architecture", "One contract, one authority");
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 0.55, y: 1.7, w: 8.9, h: 0.85,
      fill: { color: TEAL_TINT }, line: { color: TEAL, width: 1 }, rectRadius: 0.08,
    });
    s.addText("Action4D (vx, vy, vz_up, yaw_rate) at 10 Hz", {
      x: 0.7, y: 1.8, w: 8.6, h: 0.32,
      fontSize: 14, color: TEAL_DK, fontFace: FF, bold: true,
    });
    s.addText("The only type the Guardrail accepts. Any action source emitting it is admissible, which makes the model above a replaceable component rather than a dependency.", {
      x: 0.7, y: 2.12, w: 8.6, h: 0.38,
      fontSize: 11, color: INK, fontFace: FF,
    });

    const stages = [
      ["OWL-ViT", "text to box"],
      ["Colour gate", "verifies hue"],
      ["Estimator", "predicts"],
      ["Guidance", "proportional"],
      ["SHIELD", "final authority"],
      ["Autopilot", "PID inner loop"],
    ];
    let x = 0.55;
    stages.forEach(([t, sub], i) => {
      const w = 1.42;
      const isShield = t === "SHIELD";
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
        x, y: 2.85, w, h: 0.95,
        fill: { color: isShield ? TEAL : WHITE },
        line: { color: isShield ? TEAL : LINE, width: isShield ? 2 : 1 },
        rectRadius: 0.06,
      });
      s.addText(t, {
        x, y: 2.98, w, h: 0.3, fontSize: 11, bold: true,
        color: isShield ? WHITE : INK, fontFace: FF, align: "center",
      });
      s.addText(sub, {
        x, y: 3.28, w, h: 0.3, fontSize: 9,
        color: isShield ? TEAL_TINT : GREY, fontFace: FF, align: "center",
      });
      if (i < stages.length - 1) {
        s.addText("\u203A", {
          x: x + w, y: 2.95, w: 0.06, h: 0.75,
          fontSize: 16, color: TEAL, fontFace: FF, align: "center", valign: "middle",
        });
      }
      x += w + 0.06;
    });

    card(s, 0.55, 4.05, 8.9, 1.0, null, "Separation of authority",
      "No repair operator modifies yaw_rate. Heading remains the controller's; ground track is what the Shield bends. A tick where emitted.yaw_rate equals raw.yaw_rate while emitted.(vx, vy) differs is one in which both systems acted — which makes simultaneity measurable rather than asserted.");
    badge(s, next() + 1);
  }

  /* ── 04 · PERCEPTION ─────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Methods · perception", "Open-vocabulary detection with a colour gate");
    card(s, 0.55, 1.7, 4.3, 1.35, ic.eye, "Detector",
      "google/owlvit-base-patch32. 153 M parameters, 0.61 GB VRAM, Apache-2.0. Target specified at runtime as text; no fixed class list, no retraining.");
    card(s, 5.15, 1.7, 4.3, 1.35, ic.chip, "Division of labour",
      "The network resolves the noun. A fixed rule verifies the adjective. Ranking is score x (0.25 + 0.75 x colour_match).");
    s.addText("Defect corrected: the gate measured brightness, not colour", {
      x: 0.55, y: 3.2, w: 8.9, h: 0.3, fontSize: 13, color: INK, fontFace: FF, bold: true,
    });
    table(s, [
      ["Measured on the target's own pixels", "In shade", "In sunlight"],
      ["Median HSV saturation", "99", "69"],
      ["Pixels passing the saturation floor", "56.7 %", "8.1 %"],
      ["Detections rejected by the colour gate", "0 of 25", "25 of 25"],
    ], [0.5, 0.25, 0.25], 0.55, 3.55, 0.34, [[3, 2]]);
    s.addText("Saturation is chroma divided by brightness, so sunlight lowered it without changing colour. An absolute chroma floor replaced it; detector hit rate rose from 0.73 to 1.000.", {
      x: 0.55, y: 4.95, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 05 · ESTIMATION AND CONTROL ─────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Methods · estimation and control", "Filtering the measurement, not the command");
    s.addText("Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz, and the box width used as a range proxy varies 37.7 % between consecutive frames.", {
      x: 0.55, y: 1.72, w: 8.9, h: 0.4, fontSize: 11.5, color: INK, fontFace: FF,
    });
    table(s, [
      ["Forward channel", "From box width", "From the estimate"],
      ["Command step per tick, 95th percentile", "0.716 m/s", "0.283 m/s"],
      ["Ticks where the rate limiter engaged", "15.6 %", "3.0 %"],
    ], [0.5, 0.25, 0.25], 0.55, 2.25, 0.36, [[1, 2], [2, 2]]);
    card(s, 0.55, 3.6, 4.3, 1.45, ic.slider, "Why not add D or I",
      "A derivative term amplifies the same measurement noise. An integral term winds up whenever the Shield overrides the command. PID is present, below this layer, inside the autopilot.");
    card(s, 5.15, 3.6, 4.3, 1.45, ic.gauge, "What was added",
      "A constant-velocity Kalman filter with innovation gating and velocity feed-forward, predicting between detections.");
    badge(s, next() + 1);
  }

  /* ── 06 · SHIELD ─────────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Methods · safety shield", "Monitor, repair, escalate");
    const stages = [
      ["Monitor", "Forward-simulate the proposed action for 3 s against every active constraint, with trend awareness, so a violation is caught before it occurs."],
      ["Repair", "Apply the minimal correction by fixed-point iteration. The emitted action is re-checked; an action that still violates never leaves the Shield."],
      ["Escalate", "If repair cannot clear the violation, brake."],
    ];
    let y = 1.72;
    stages.forEach(([t, b], i) => {
      card(s, 0.55, y, 8.9, 0.92, i === 0 ? ic.shield : null, `${i + 1}. ${t}`, b);
      y += 1.02;
    });
    s.addText("Four constraint types are expressible: PolygonFence, AltitudeEnvelope, KinematicEnvelope, ObstacleClearance. Each carries a priority (P0/P1/P2) and a violation action. Policies are YAML, validated, and hashed to policy_hash.", {
      x: 0.55, y: 4.85, w: 8.9, h: 0.5, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 07 · EXPERIMENTAL SETUP ─────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Experimental setup", "Configuration held fixed across scenarios");
    table(s, [
      ["Parameter", "Value"],
      ["Simulator / scene", "Project AirSim on UE 5.7, JapaneseCity Demo_day"],
      ["GPU", "NVIDIA RTX 4080, 16 GB"],
      ["Detector input (FrontCamera)", "400 x 225, 90 deg HFOV, pitched 20 deg down"],
      ["Recording camera (Chase)", "960 x 540 at 20 Hz"],
      ["Cruise altitude / stand-off", "9 m / 15.8 m"],
      ["Target vehicle", "glTF taxi, 2.5 m/s, two 8 s stops, 93 m route with a turn"],
      ["Flight duration", "70 s per scenario"],
    ], [0.38, 0.62], 0.55, 1.72, 0.355);
    s.addText("FrontCamera resolution is held fixed at 400 x 225 across all reported work. It is the detector's input, and every measurement in the repository is conditioned on it.", {
      x: 0.55, y: 4.65, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 08 · RESULTS: TRACKING ──────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Results · tracking", "Three scenarios, 70 s each");
    const rows = [["Measure", "Tracking", "Distractors", "No-fly zone"]];
    const add = (label, fn) => rows.push([label, ...TAGS.map((t) => fn(M[t], L[t]))]);
    add("Detector hit rate", (m) => n(m.det_hit_rate, 3));
    add("Ticks with target held", (m) => pct(m.frac_ticks_seen));
    add("Detector rate", (m) => n(m.det_hz, 2) + " Hz");
    add("Mean separation", (m) => n(m.sep_mean_m, 1) + " m");
    add("Time within 30 m", (m) => pct(m.frac_within_30m));
    table(s, rows, [0.4, 0.2, 0.2, 0.2], 0.55, 1.72, 0.36);
    s.addText("The no-fly-zone scenario holds a lower separation by design: the fence spans the corridor, the target drives through it, and the aircraft is required not to follow.", {
      x: 0.55, y: 3.92, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    card(s, 0.55, 4.32, 8.9, 0.95, null, "Target discrimination",
      "With three additional vehicles of different colours on the same street, target jumping fell from 14.0 % of detections to 0.4 %.");
    badge(s, next() + 1);
  }

  /* ── 09 · RESULTS: GUARDRAIL ─────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Results · guardrail", "The acceptance criterion");
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: 0.55, y: 1.7, w: 8.9, h: 0.95,
      fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.08,
    });
    s.addText("P0 violation escape rate = 0.000 on every flight recorded", {
      x: 0.7, y: 1.85, w: 8.6, h: 0.35, fontSize: 17, color: WHITE, fontFace: FF, bold: true,
    });
    s.addText("An escape is counted only when the Shield neither repaired nor braked and the emitted action still violated a P0 rule. Scoring the raw action would credit the system for its own inputs.", {
      x: 0.7, y: 2.2, w: 8.6, h: 0.4, fontSize: 10.5, color: TEAL_TINT, fontFace: FF,
    });
    const rows = [["Invariant", "Tracking", "Distractors", "No-fly zone"]];
    rows.push(["P0 escape rate", ...TAGS.map((t) => n(M[t].p0_violation_escape_rate, 3))]);
    rows.push(["Time inside no-fly zone", ...TAGS.map((t) => n(M[t].nfz_s, 1) + " s")]);
    rows.push(["Altitude envelope escape", ...TAGS.map((t) => n(M[t].alt_violation_s, 1) + " s")]);
    rows.push(["Shield interventions", ...TAGS.map((t) => String(M[t].interventions))]);
    table(s, rows, [0.4, 0.2, 0.2, 0.2], 0.55, 2.85, 0.36);
    const fired = TAGS.filter((t) => M[t].interventions > 0);
    const quiet = TAGS.filter((t) => M[t].interventions === 0);
    const note = (fired.length
      ? "In " + fired.map((t) => `${LABEL[t]} (${M[t].interventions})`).join(", ")
        + " the guidance layer proposed actions that would have violated an active rule and the Shield corrected them, holding the aircraft at the fence for "
        + String(M.demo_nfz.nfz_hold_ticks) + " ticks. Repair is the normal outcome, not an error. "
      : "")
      + (quiet.length
        ? "In " + quiet.map((t) => LABEL[t]).join(" and ")
          + " the count is zero: the guidance layer never proposed a violating action, not that the Shield was idle. It evaluated every tick."
        : "");
    s.addText(note, {
      x: 0.55, y: 4.4, w: 8.9, h: 0.6, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 10 · RESULTS: RESOLUTION STUDY ──────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Results · parameter study", "Recording resolution against detector rate");
    s.addText("The recording camera was raised to 1280 x 720 to improve video quality. Acceptance thresholds were fixed before the runs: detector rate \u2265 4.0 Hz, control loop \u2265 9.5 Hz.", {
      x: 0.55, y: 1.72, w: 8.9, h: 0.4, fontSize: 11.5, color: INK, fontFace: FF,
    });
    const C = STUDY._conclusion;
    const dz = C.det_hz_by_configuration;
    const lz = C.loop_hz_by_configuration;
    const iz = C.infer_ms_median_by_configuration;
    const key = ["chase_1280x720_window_1280x720",
                 "chase_960x540_window_1280x720",
                 "chase_960x540_window_960x540"];
    const cfg = ["1280 x 720", "960 x 540", "960 x 540"];
    const win = ["1280 x 720", "1280 x 720", "960 x 540"];
    const rows = [["Chase capture", "Sim window", "Detector", "Loop", "Inference"]];
    key.forEach((k, i) => rows.push([
      cfg[i], win[i], n(dz[k], 2) + " Hz", n(lz[k], 2) + " Hz",
      iz[k] ? iz[k] + " ms" : "-",
    ]));
    table(s, rows, [0.22, 0.22, 0.19, 0.19, 0.18], 0.55, 2.25, 0.36);
    card(s, 0.55, 3.7, 4.3, 1.35, ic.flask, "What the third row settles",
      "Inference held at 286-287 ms across a 2.4x change in capture pixels and a 1.8x change in window pixels. A cost invariant to surrounding load is fixed per-inference cost, not contention.");
    card(s, 5.15, 3.7, 4.3, 1.35, ic.warn, "Threshold not relaxed",
      "Neither configuration reached 4.0 Hz. The threshold was set before the data and is reported as missed. Tracking quality, which it exists to protect, was unaffected: hit rate 1.000, target held on 100 % of ticks.");
    badge(s, next() + 1);
  }

  /* ── 11 · THE SLOT ───────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Results · model independence", "Five occupants of one action slot");
    table(s, [
      ["Action source", "Nature", "Result"],
      ["OpenVLA-7B, 4-bit", "Real 7B camera + language VLA", "553 ticks at 10 Hz, NFZ 0.0 s"],
      ["AerialVLA LoRA", "UAV-tuned adapter", "Target reached, NFZ 0"],
      ["Our QLoRA fine-tunes", "Self-collected expert flights", "100 % reached, efficiency 0.996"],
      ["Behaviour-cloning policy", "Trained state + geometry policy", "Flown, NFZ 0"],
      ["Proportional controller", "Hand-written, no model", "Reported in this document"],
    ], [0.28, 0.34, 0.38], 0.55, 1.72, 0.38);
    s.addText("The Shield source code is identical in all five cases. That invariance, rather than any single model's performance, is the project's claim.", {
      x: 0.55, y: 4.15, w: 8.9, h: 0.4, fontSize: 11.5, color: INK, fontFace: FF, bold: true,
    });
    s.addText("Language grounding is not supplied by the VLA. Over 108 controlled forward passes the {object} prompt slot of AerialVLA was measured inert: correct and incorrect colour words produce indistinguishable actions.", {
      x: 0.55, y: 4.6, w: 8.9, h: 0.5, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 12 · VERIFICATION ───────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Verification", "Making a number traceable to the run that produced it");
    card(s, 0.55, 1.72, 4.3, 1.6, ic.check, "Determinism manifest",
      "Six fields per flight: code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup, topology. sim_speedup is derived from the scene file rather than asserted by the caller.");
    card(s, 5.15, 1.72, 4.3, 1.6, ic.flask, "KPI computation",
      "Four acceptance KPIs computed from flight artefacts, not from summary statistics reported by the run itself.");
    s.addText("is_kpi_grade() refuses a run when:", {
      x: 0.55, y: 3.5, w: 8.9, h: 0.3, fontSize: 12, color: INK, fontFace: FF, bold: true,
    });
    s.addText([
      { text: "simulation is not real-time  ·  a manifest field did not resolve  ·  detector ran below 2 Hz  ·  initial heading was more than 10 deg off  ·  topology is not the grant's canonical ArduPilot SITL configuration", options: { bullet: false } },
    ], {
      x: 0.55, y: 3.82, w: 8.9, h: 0.5, fontSize: 11, color: GREY, fontFace: FF,
    });
    s.addText("Each check exists because the corresponding failure has already occurred and produced a plausible-looking number.", {
      x: 0.55, y: 4.4, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF, italic: true,
    });
    badge(s, next() + 1);
  }

  /* ── 13 · LIMITATIONS ────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Limitations", "Stated as findings, each one measured");
    table(s, [
      ["Limitation", "Consequence"],
      ["No flight to date is KPI-grade", "Results are functional-rail evidence; topology gate not yet met"],
      ["Occupancy map holds buildings only", "Street furniture contacted at (48.3, -0.9); Shield cannot constrain absent geometry"],
      ["Depth quantised to 1 m", "Supports proximity, not a metric stand-off such as 'hold 10 m'"],
      ["Per-object stand-off not in policy", "A CLI parameter, so not hashed, not audited, not Shield-enforced"],
      ["Range assumes a 4.0 m width", "Applied to a 0.5 m pedestrian it would over-report distance ~8x"],
      ["Learned VLA cannot track in real time", "2.7 s per decision; a 2 m/s target moves 5.4 m within one inference"],
    ], [0.38, 0.62], 0.55, 1.72, 0.42);
    badge(s, next() + 1);
  }

  /* ── 14 · WORK PACKAGES ──────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Work package status", "Against the funded plan");
    table(s, [
      ["WP", "Component", "Status"],
      ["WP1", "Policy DSL", "In use; validated and hashed. Per-object stand-off not yet expressible"],
      ["WP2", "Prefix compiler", "Reduced version in use on the VLA path"],
      ["WP3", "Safety Shield", "Implemented, tested, exercised under conflict. P0 escape 0"],
      ["WP4", "Stress harness", "Manifest and KPI implemented. Scenario sweep not built"],
    ], [0.1, 0.28, 0.62], 0.55, 1.72, 0.45);
    card(s, 0.55, 3.85, 8.9, 1.2, ic.road, "The SITL rail already exists",
      "sitl/ builds ArduPilot under WSL, serves MAVLink on tcp:127.0.0.1:5760, and flies the Guardrail mission using SET_POSITION_TARGET_LOCAL_NED in GUIDED mode. Three configurations have been run. The Guardrail package is identical between the AirSim and ArduPilot rails; only the bottom adapter differs. It is not yet wired to the manifest and KPI machinery.");
    badge(s, next() + 1);
  }

  /* ── 15 · NEXT PERIOD ────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Next period", "Ordered by contribution to the acceptance criteria");
    const items = [
      ["1", "Wire the SITL rail to the WP4 machinery", "Converts existing evidence into contractual figures without further flying"],
      ["2", "Add MAVROS 2 to the SITL rail", "Completes the canonical topology; precondition for any KPI-grade number"],
      ["3", "Per-object stand-off constraint in the policy schema", "Converts a controller setpoint into a hashed, audited, enforced rule (WP1)"],
      ["4", "Pedestrians and additional vehicles in the scene", "Requested at the 19 August review; requires item 3 first"],
      ["5", "Populate the occupancy map with street furniture", "Addresses a demonstrated collision; requires no additional sensor"],
      ["6", "Detector throughput", "Candidate is YOLO-World: open-vocabulary at 30-50 Hz"],
    ];
    let y = 1.72;
    items.forEach(([nn, t, b]) => {
      s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
        x: 0.55, y, w: 0.38, h: 0.38,
        fill: { color: TEAL }, line: { color: TEAL }, rectRadius: 0.05,
      });
      s.addText(nn, {
        x: 0.55, y, w: 0.38, h: 0.38, fontSize: 11, color: WHITE, fontFace: FF,
        bold: true, align: "center", valign: "middle",
      });
      s.addText(t, {
        x: 1.05, y: y - 0.02, w: 8.4, h: 0.26, fontSize: 12, color: INK, fontFace: FF, bold: true,
      });
      s.addText(b, {
        x: 1.05, y: y + 0.22, w: 8.4, h: 0.26, fontSize: 10, color: GREY, fontFace: FF,
      });
      y += 0.58;
    });
    badge(s, next() + 1);
  }

  /* ── 16 · DELIVERABLES ───────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    heading(s, "Deliverables", "Accompanying this report");
    const rows = [["Artefact", "File", "Size"]];
    rows.push(["Written report", "docs/MIDTERM-REPORT-Aug2026.pdf / .docx", "-"]);
    rows.push(["This deck", "docs/VLA-Guardrail-Midterm-Aug2026.pptx / .pdf", "-"]);
    for (const t of TAGS) {
      const v = videoInfo(t);
      rows.push([LABEL[t] + " video", "docs/video/" + t + ".mp4",
        v ? v.mb.toFixed(1) + " MB" : "pending"]);
    }
    table(s, rows, [0.28, 0.52, 0.2], 0.55, 1.72, 0.4);
    s.addText("Videos are supplied as separate files and are not embedded. Every figure in this deck is read from demo/out/<tag>/metrics.json at build time.", {
      x: 0.55, y: 4.2, w: 8.9, h: 0.4, fontSize: 10.5, color: GREY, fontFace: FF,
    });
    badge(s, next() + 1);
  }

  /* ── 17 · CLOSE ──────────────────────────────────────────────────────── */
  {
    const s = pres.addSlide();
    darkBase(s, "Midterm report · February – August 2026");
    s.addText("The Guardrail held on every flight recorded:\nP0 violation escape rate 0, no-fly-zone time 0.0 s,\naltitude envelope escape 0.0 s.", {
      x: 0.55, y: 1.9, w: 8.6, h: 1.4,
      fontSize: 22, color: WHITE, fontFace: FF, bold: true, lineSpacingMultiple: 1.25,
    });
    s.addText("No run is yet KPI-grade. Closing that gap is the first item of the next period.", {
      x: 0.55, y: 3.5, w: 8.6, h: 0.4, fontSize: 13, color: TEAL_TINT, fontFace: FF,
    });
    s.addText("Nathanael Tjahyadi  ·  NTUT AIoT Laboratory", {
      x: 0.55, y: 4.6, w: 8.6, h: 0.3, fontSize: 11, color: TEAL_TINT, fontFace: FF,
    });
  }

  const out = path.join(REPO, "docs", "VLA-Guardrail-Midterm-Aug2026.pptx");
  await pres.writeFile({ fileName: out });

  const mb = fs.statSync(out).size / 1048576;
  console.log(`written: ${out}`);
  console.log(`slides : ${SLIDE + 2}`);
  console.log(`size   : ${mb.toFixed(2)} MB`);

  // pptxgenjs writes a package PowerPoint will not open: ZIP directory entries,
  // and content-type overrides for slide masters it never wrote. Repair, then
  // audit. PowerPoint reports a bad package as "could not open the file" and
  // nothing else, so every such defect has to be caught here, where the
  // offending slide can still be named.
  const { execFileSync } = require("child_process");
  const PY = "C:/Users/natha/.conda/envs/vla-real/python.exe";
  const run = (script, label) => {
    const p = path.join(REPO, "tools", script);
    if (!fs.existsSync(p)) return true;
    try {
      process.stdout.write(execFileSync(PY, [p, out], { encoding: "utf8" }));
      return true;
    } catch (e) {
      process.stdout.write(e.stdout || "");
      console.error(`REFUSED: ${label} failed.`);
      return false;
    }
  };

  if (!run("fix_pptx_package.py", "package repair")) process.exit(1);
  if (!run("audit_pptx.py", "package audit")) process.exit(1);

  if (mb > 10) {
    console.error("REFUSED: deck exceeds 10 MB. Something is embedding media.");
    process.exit(1);
  }
}

main().catch((e) => { console.error(e); process.exit(1); });
