"""FrameRecorder — is the reported capture rate the rate frames were captured at?

Run either way:
    pytest tests/test_recorder.py -v
    python tests/test_recorder.py

This exists because the answer was no for the whole midterm campaign, and the
consequence reached the deliverables.

The recorder is started at scene setup but writes nothing until the control loop
first calls `set_hud()` — after arming and the climb to cruise, about 27 s later.
`summary()` divided the frame count by the recorder's ENTIRE lifetime, so those
27 s of deliberate silence counted as recording time. Every run on disk reported
14.6–16.7 Hz for a recorder that was hitting 20.00 Hz exactly.

`tools/make_demo_video.py` takes the video's fps straight from that number, so
every clip delivered before 2026-08-25 plays about 1.3x slower than real time —
while the tool prints that the duration is correct.
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from recorder import FrameRecorder                              # noqa: E402


def _rec(tmp: Path, hz: float = 20.0) -> FrameRecorder:
    """A recorder that is never started. summary() is pure arithmetic over
    counters, so the thread is not needed and the simulator certainly is not."""
    return FrameRecorder(obs=None, out_dir=tmp, annotate_fn=None, hz=hz)


def _tmp(name: str) -> Path:
    import tempfile
    return Path(tempfile.mkdtemp(prefix=f"rec_{name}_"))


def test_the_rate_is_measured_over_the_window_that_wrote_frames():
    """The regression. 27 s of pre-mission silence then 87 s at 20 Hz."""
    r = _rec(_tmp("window"))
    now = time.time()
    r.t0 = now                       # recorder started
    r.t_first_write = now + 27.0     # first frame, after arming and the climb
    r.t_stop = now + 27.0 + 87.0
    r.n_written = int(87.0 * 20.0)
    r.n_skipped = int(27.0 * 20.0)

    s = r.summary()
    assert abs(s["achieved_hz"] - 20.0) < 0.05, (
        f"reported {s['achieved_hz']} Hz for a recorder that captured at 20; "
        f"dividing by the whole lifetime would give "
        f"{r.n_written / (r.t_stop - r.t0):.2f}")
    assert s["seconds"] > s["writing_seconds"], "both windows must be reported"
    assert "WARNING" not in s, f"a recorder meeting its target must not warn: {s}"


def test_a_recorder_that_really_is_slow_still_warns():
    """The warning has to survive the fix, or the fix has only hidden it.

    It fired on every run of the campaign and was a false alarm every time,
    which teaches a reader to ignore it. That is worse than no warning, so what
    it judges is now the real capture rate.
    """
    r = _rec(_tmp("slow"))
    now = time.time()
    r.t0 = now
    r.t_first_write = now + 10.0
    r.t_stop = now + 10.0 + 100.0
    r.n_written = 1200                       # 12 Hz against a 20 Hz target
    r.n_skipped = 200

    s = r.summary()
    assert abs(s["achieved_hz"] - 12.0) < 0.05
    assert "WARNING" in s, "a genuine 60 % shortfall must still be reported"
    assert s["rate_ratio"] < 0.7


def test_skipped_slots_are_still_counted_and_reported():
    """`skipped_no_hud` was never wrong — it was only the wrong thing to leave
    in the denominator. It stays, because it is how a reader sees that the
    recorder outlived the mission clock at both ends."""
    r = _rec(_tmp("skips"))
    now = time.time()
    r.t0 = now
    r.t_first_write = now + 5.0
    r.t_stop = now + 55.0
    r.n_written = 1000
    r.n_skipped = 100

    s = r.summary()
    assert s["skipped_no_hud"] == 100
    assert s["frames"] == 1000


def test_a_recorder_that_never_wrote_reports_no_rate_rather_than_zero():
    """No frames is not '0 Hz achieved'; it is 'there is no rate to report'.
    Returning a number would let the video builder encode at it."""
    r = _rec(_tmp("empty"))
    now = time.time()
    r.t0 = now
    r.t_stop = now + 60.0
    r.n_written = 0
    r.n_skipped = 1200

    s = r.summary()
    assert s["achieved_hz"] is None, s
    assert "rate_ratio" not in s


def test_the_real_runs_on_disk_would_now_report_their_true_rate():
    """Against the sidecars actually written during the campaign.

    Every one of them recorded `frames`, `seconds`, `skipped_no_hud` and
    `target_hz`, which is enough to reconstruct the writing window. If the
    corrected arithmetic does not land on the target for runs that were pacing
    perfectly, the fix is wrong.
    """
    import json
    outs = sorted((ROOT / "demo" / "out").glob("*/view/recorder.json"))
    if not outs:
        return
    checked = 0
    for p in outs:
        d = json.loads(p.read_text(encoding="utf-8"))
        n, secs = d.get("frames", 0), d.get("seconds", 0.0)
        sk, tgt = d.get("skipped_no_hud", 0), d.get("target_hz", 0)
        if not (n > 100 and tgt and secs > 1):
            continue                     # abandoned or trivial run
        window = secs - sk / tgt
        assert window > 1, p
        true_hz = n / window
        assert abs(true_hz - tgt) < 0.5 * tgt, (
            f"{p.parent.parent.name}: reconstructed {true_hz:.2f} Hz against a "
            f"{tgt} Hz target")
        checked += 1
    assert checked, "no usable sidecars found to check against"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
