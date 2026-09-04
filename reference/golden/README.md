# The frozen numerical baseline

What the analytic simulator produces, written down, so that a change claiming to
move no numbers can be **checked** instead of trusted.

```bash
python tvc.py golden           # capture
python tvc.py golden --check   # compare; exit 1 and name every drifted field
```

---

## Why this exists

Some changes are supposed to move no numbers: a rename, a file move, a refactor
that only relocates code. Without a record taken beforehand that claim is
untestable — and an untestable claim is exactly how a rename quietly becomes a
physics change.

It has already earned its keep. When the rocket axis convention was adopted (roll
= the thrust axis), the acceptance criterion was that this baseline survive
unchanged. It did: **137 numeric values compared under the key relabelling, zero
differences.**

`--check` walks the JSON and reports every changed field with its old and new
value, so a drift report names the quantity rather than just failing.

---

## What is frozen

| block | covers |
|---|---|
| `vehicle` | mass, inertia, lever arm, thrust and gimbal limits |
| `authority_at_hover` | roll headroom, the signed feasible set, and its variation with thrust |
| `allocation_roundtrip` | that `allocate()` realizes the moment it was asked for |
| `scenarios` | all five, pass/fail plus every metric |
| `smoke_test` | the default `SimConfig` run |

The `vehicle` block exists to make a mismatch **attributable**: a changed metric
with an unchanged vehicle block is a code change, while both changing is a
re-measured airframe — which is a legitimate reason for the numbers to move.

`validate_baseline.txt` is the same run in human-readable form, for reading by
eye alongside a diff.

### Two deliberate exceptions to exact comparison

Both exist to stop the file flapping for reasons that are not the code's fault.

- **Metrics are rounded to 9 decimals.** Far tighter than any legitimate
  refactor-era change, loose enough to survive a different BLAS or numpy build.
- **The allocation round-trip stores a bound (`< 10⁻⁹ N·m`), not the residual.**
  The residual is ~4.3 × 10⁻¹³ and its last digits are platform-dependent; the
  bound is the actual invariant. The measured value is still printed for
  information.

---

## What is NOT frozen, and why it matters

**The Gazebo flight.** `harness/gz.py` needs `gz-transport13`, which exists only
in the devcontainer, and this baseline was captured on a Windows host. So the
flight that is the *only* thing that has actually flown has no golden.

That is the largest gap in the baseline, and it is not academic: the standalone
Gazebo demo used to carry its own independent controller and now calls the shared
flight code. The gains are the same set, ported into inertia-normalized units, so
the loop dynamics should be identical — but *should be* is an argument, not a
measurement.

Capture it in the devcontainer:

```bash
bash gazebo/run_hover.sh --duration 30 --altitude 2.0 \
     --log reference/golden/hover_baseline.csv
```

Compare on **metrics** — final altitude error, peak tilt, lateral drift, settling
time — rather than sample by sample. The harness now steps on odometry arrival
rather than on a wall clock, so the run *is* reproducible in simulated time; a
sample-wise comparison should become meaningful once that has been confirmed on
two consecutive runs.

---

## When the golden legitimately changes

Only a deliberate physics or parameter change may move these numbers, and when it
does, **the update is its own commit that says which numbers moved and why.**

**Never regenerate the golden to make a failing check pass.** If `--check` fires
during a change that was supposed to be behaviour-preserving, the change is
wrong — not the baseline.
