# Golden references

Frozen numbers from **before** the flight-software restructure, so that later
claims about it can be checked instead of asserted.

The one that matters: the axis rename (Phase 6 of the unification plan) is
supposed to be a *pure rename*. A pure rename moves no numbers. Without a
record taken beforehand, that is an untestable claim — which is exactly how a
rename quietly becomes a physics change.

## What is frozen

| file | covers | captured by | checked by |
|---|---|---|---|
| `analytic_baseline.json` | vehicle constants, control-authority budget, allocation round-trip, the 4 validation scenarios, the headless smoke test | `python sim/capture_golden.py` | `python sim/capture_golden.py --check` |
| `validate_baseline.txt` | the human-readable authority report + scenario lines | `python sim/validate_control.py --verbose` | read by eye |

`--check` walks the JSON and reports every changed field with its old and new
value, so a drift report names the quantity rather than just failing.

Two deliberate exceptions to exact comparison, both to stop the file flapping
for reasons that are not the code's fault:

- Metrics are rounded to 9 decimals. Far tighter than any legitimate rename-era
  change, loose enough to survive a different BLAS or numpy build.
- The allocation round-trip stores a **bound** (`< 1e-9 N·m`), not the residual.
  The residual is ~4e-13 and its last digits are platform-dependent; the bound
  is the actual invariant. The measured value is still printed for information.

## What is NOT frozen — and why

**The Gazebo flight.** `sim/hover.py` needs `gz-transport13`, which exists only
in the devcontainer, and this baseline was captured on the Windows host. So the
flight that is currently the *only* thing that actually flies has no golden yet.

That is the largest gap in the baseline, because Phase 2 (splitting `physics.py`
into flight code and plant) and Phase 3 (adopting `hover.py`'s gains) both put
that flight at risk, and its acceptance gate is "still matches the baseline".

Capture it in the devcontainer before starting Phase 2:

```bash
bash sim/run_hover.sh --duration 30 --altitude 2.0 --log sim/golden/hover_baseline.csv
```

Compare on **metrics** (final altitude error, peak tilt, lateral drift, settling
time), not sample-by-sample: `hover.py` currently steps on a wall-clock
`time.sleep()` loop, so the run is not bit-reproducible. Phase 5 replaces that
loop with one that steps on odometry arrival, at which point a sample-wise
comparison becomes meaningful and this note should be revisited.

## When the golden legitimately changes

Only a deliberate physics or parameter change may move these numbers, and when
it does the update is its own commit that says which numbers moved and why. The
`vehicle` block exists to make that attributable: a changed metric with an
unchanged vehicle block is a code change; both changing is a re-measured
airframe.

Never regenerate the golden to make a failing check pass.
