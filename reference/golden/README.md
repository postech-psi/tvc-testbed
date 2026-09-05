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

## The Gazebo baseline

`harness/gz.py` needs `gz-transport13`, which exists only in the devcontainer,
so this used to be the largest gap in the record: the pipeline with a real
physics engine under it had no baseline at all. It has one now.

```bash
bash gazebo/run_hover.sh --duration 30            # fly it
python tvc.py hover --check-golden --unpause tvc_flight
python tvc.py hover --capture-golden --unpause tvc_flight   # re-freeze
```

| file | what |
|---|---|
| `hover_baseline.json` | fourteen metrics of a 30 s hover, plus the per-metric tolerance the check uses |
| `hover_baseline.png` | the same run as a picture, for reading by eye |

### Metrics, not samples — and that is a measurement

The run is reproducible in **aggregate** and not sample by sample. Two
consecutive 30 s runs differ by at most 1.5° of roll, 0.6° of gimbal and 12 mm
of altitude at any one sample, because the first accepted odometry message can
still land one or two physics steps apart. A sample-wise comparison against
numbers that move by that much would fail on nothing.

Every aggregate in the file agrees between those same two runs to far better
than its tolerance, and the tolerances are set from that measured spread rather
than chosen as performance targets. A tolerance tight enough to catch ordinary
retuning is a tolerance people learn to regenerate.

It was much worse before. When the shell script owned the unpause, the same two
runs peaked at **14.6° and 58.0°** of thrust-axis roll — a factor of four, from
nothing but how fast the host started a Python process. The controller now
unpauses the world itself. That is what made a baseline possible at all.

### Why there is no committed CSV

The per-sample log is 1.8 MB and nothing compares against it, so committing it
would be storing a large file to be ignored. The PNG is regenerated from the
same run whenever the golden is re-captured, and is the artefact a person
actually reads.

---

## When the golden legitimately changes

Only a deliberate physics or parameter change may move these numbers, and when it
does, **the update is its own commit that says which numbers moved and why.**

**Never regenerate the golden to make a failing check pass.** If `--check` fires
during a change that was supposed to be behaviour-preserving, the change is
wrong — not the baseline.
