# Mass budget and part positions

Where every mass number and position in the simulation comes from, and how
much to trust it. Inertia for the Gazebo model is derived from this — see
[../sim/README.md](../sim/README.md).

## Coordinate system

Positions are from `TVC Ver3.step`, in **millimeters**, in the assembly's own
frame: **+z is the vehicle's long/thrust axis**, origin near the gimbal.
The airframe spans z = −81 mm (feet) to +652 mm (top).

This matches the `tvc_control/physics.py` convention where body **+z is the
thrust axis** and `L` is the axial gimbal-pivot→CM distance.

> **Nested-subassembly caveat.** Parts inside `CRM2413 assembly` (the motor,
> propellers, cone screw, extension rods, base ring) report positions in *that
> subassembly's* local frame, not the global one — visible as their y ≈ 36–80,
> z ≈ 0 pattern, where the rest of the vehicle uses z as the long axis. To get
> their global position, compose with the parent placement
> `CRM2413 assembly:1 = (−12.31, −25.35, 47.15)`. They are listed below as
> reported; do not mix them with global-frame rows without composing.

## Measured masses

Weighed / assigned in CAD (2026-07-28). These are the trustworthy numbers.

| Part | Mass (g) | Position (mm, global frame) | Mesh body |
|---|---|---|---|
| Outer gimbal | 43.00 | (−5.22, −18.22, 89.64) | idx 3 ¹ |
| Inner gimbal | 15.75 | (−5.35, −19.12, 89.71) | idx 3 ¹ |
| Lower body adapter | 98.00 | (−5.22, −18.22, 142.64) | idx 0 ² |
| Center body (`Body Plate:1`) | 64.00 | (−5.22, −18.22, 222.83) | idx 1 |
| Battery holder | 50.00 | (−5.22, −18.22, 305.84) | idx 2 |
| Feet (3 × 6.67) | 20.00 | (30.44, −218.44, −78.52), (−196.28, 51.44, −78.53), (150.73, 112.31, −78.53) | idx 11, 12, 13 |
| Carbon pipe legs (3 × 16.67) | 50.00 | z = −69.46, radius ≈ 216 | idx 4, 5, 6 |
| Body pipes (2 × 20.0) | 40.00 | (−5.22, −68.22, 152.64), (−5.22, 31.78, 150.64) | idx 8 ³ |
| **Subtotal** | **380.75** | | |

¹ The two gimbal rings **merge into a single mesh body** — they touch in the
export, so the vertex-sharing split can't separate them. Recorded as one
58.75 g body at idx 3. Derived density 1151 kg/m³, consistent with a PLA print.

² `idx 0` is a large merged blob (bbox 161×155×616 mm) spanning much of the
airframe — several touching parts collapsed into one body. The lower body
adapter's 98 g is **not** assigned to it in `cad_parts.csv`, because that
would attribute the whole blob's geometry to one part's mass.

³ Only one 10×10×500 mm body-pipe body is separable in the mesh (idx 8);
`Body Pipe:2` merges into the main structure. Both are 20 g.

**Cross-check on the legs.** The 3 legs come from the 16×12×1000 mm carbon
stock. Two independent routes agree: cluster volume 15,764 mm³ each implies
179 mm of 16×12 annulus per leg (537 mm total, from a 1000 mm stock), and
50 g over that volume gives 1057 kg/m³ either way. Feet come out at
921 kg/m³ — right for PLA at partial infill.

The three feet sit at nearly the same z but ~220 mm out radially, so they
matter more for roll/pitch inertia than their mass suggests.

## In CAD but not yet weighed

Positions are known, so only a mass is needed. Add to
[../tools/cad_parts.csv](../tools/cad_parts.csv).

| Part | Instances | Position (mm) | Likely mass source |
|---|---|---|---|
| `CRM2413 assembly` (motor + 8038 props) | 1 | (−12.31, −25.35, 47.15) | spreadsheet: 180 g ⁴ |
| `PTK8515` (servos) | 2 | (−2.90, 30.28, 112.97), (−46.35, −11.23, 80.44) | in electronics table (56 g w/ horns) ⁵ |
| `PTK Servo Horn` | 2 | (−39.85, −45.37, 76.79), (−36.40, 23.78, 104.61) | same 56 g entry ⁵ |
| `Pushrod` + `Pushrod Connector` | 2 each | z ≈ 59–122 | not weighed |
| `Base ring`, `Extension rods` ×4, `Cone screw` | 6 | subassembly frame ⁶ | not weighed |
| `Propeller 1/2 cover`, `Propeller 2 base` | 3 | subassembly frame ⁶ | not weighed |

⁴ Spreadsheet lists "AEORC CRM2413 motor + 9050 propeller pair = 180 g". The
CAD names the props `CW/CCW 8038` — naming difference only, same hardware.

⁵ **Double-count risk.** The servos and horns exist as CAD bodies *and* as a
56 g line in the electronics table below. Assign them in one place only —
currently they are counted in the electronics table, so leave `mass_g` blank
for them in `cad_parts.csv`.

⁶ These sit inside `CRM2413 assembly`; see the nested-frame caveat above.

## Electronics — not in CAD at all

No battery, Pixhawk, ESC, UBEC, GPS, Pi, or radio body exists in the STEP
file. They are tracked as **located point masses** in
[../tools/components.yaml](../tools/components.yaml): each carries an assumed
mounting position, so it contributes both mass and the parallel-axis `m·d²`
inertia term. Its own spin inertia is neglected, which for a GPS puck or an
ESC is genuinely negligible next to `m·d²` at 100–600 mm from the CG.

Positions marked *est* are **assumptions**, not measurements — nothing in CAD
can supply them. They are the largest remaining uncertainty in the inertia.

| Component | Mass (g) | z (mm) | Position |
|---|---|---|---|
| 3S LiPo pack | 307 ¹ | 306 | est — in the battery holder |
| 2× Hobbywing UBEC 10A | 62 ² | 135 | est |
| Holybro M10 GPS + mount | 53 | 600 | est — needs sky view, so high |
| Leads, bolts, wiring, XT60 | 50 | 200 | est — distributed, placed at airframe centroid |
| Raspberry Pi 5 | 48 | 250 | est |
| PM07 power module | 45 | 275 | est — inline with battery leads |
| 2× Hobbywing XRotor 40A ESC | 37 | 125 | est — near the motor leads |
| Pixhawk 6C aluminum case | 35 | 223 | est — on the center body |
| Pixhawk cables, vibration mount | 30 | 230 | est |
| 2× PTK 8515 servos + horns | 56 | 113 / 80 | **from CAD** (`PTK8515:1` / `:2`) |
| Holybro SiK Radio V3 | 19 | 450 | est |
| Receiver | 6 | 450 | est |
| **Subtotal** | **748** | | |

¹ Stale: this is the 4200 mAh pack. A 2200 mAh swap is planned — that removes
roughly 130–150 g from the **top** of the vehicle (battery holder at z = 306 mm),
which lowers the CG and cuts pitch/roll inertia noticeably. Re-derive after
the swap rather than scaling.

² Spreadsheet notes only one of the two UBECs is actually used.

## Totals

| | Mass (g) |
|---|---|
| Measured CAD parts | 380.75 |
| Electronics + loose hardware | 748 |
| Unweighed structure (remainder) | 199.25 |
| **Design target** | **1328** |

380.75 + 748 = 1128.75 g itemized, leaving **199.25 g** of unweighed structure
to reach the 1328 g target — consistent with the remaining unweighed items
(motor + prop assembly ~180 g, pushrods, extension rods, prop covers).

`mass_properties.py` adds that 199.25 g back as a single `unmodeled remainder`
point mass at z = 150 mm, so the finalized figures below describe the whole
vehicle rather than only the itemized fraction. Weighing the motor assembly
would replace most of it with a real number.

## Finalized mass properties

From `python tools/mass_properties.py tools/components.yaml` — CAD solids with
real mesh-derived inertia, electronics as located point masses, plus a
remainder to the 1328 g design total:

```
m  = 1.3280 kg
CG = (-1.7, -1.0, 211.1) mm       computed, not assumed
Ix = 0.022616   Iy = 0.022581   Iz = 0.001957   kg*m^2
```

`Iz` is ~12x smaller than `Ix`/`Iy`, as expected for a long thin airframe —
and it is the yaw axis, the one the gimbal cannot control at all.

The CG at z = 211 mm sits **121 mm above the gimbal pivot** (z = 90 mm), which
is the moment arm `L` that gives the gimbal its pitch/roll authority.

**Biggest remaining uncertainty:** electronics positions are estimates (748 g,
over half the vehicle). The battery alone is 307 g at z = 306 mm — 95 mm above
the CG — so its assumed height visibly moves both CG and inertia. Re-measure by
balancing the assembled vehicle on a knife edge and set
`center_of_mass_override_mm` in `components.yaml`.

## Thrust margin

Max thrust comes from the **2026-07-20** runs — the only ones that reach full
throttle (PWM 2000); every later session stops at 1850 µs. Those drove both
rotors from a single signal, i.e. the **balanced** case, which is the right
basis: unequal rotors make net yaw torque, and yaw is the one axis the gimbal
cannot trim, so thrust bought with A ≠ B is not usable in steady hover.

| Run | Thrust at full throttle |
|---|---|
| `data_20260720_183744` (clean staircase, n=717) | 18.07 N |
| `data_20260720_164717` | 19.34 N |
| `data_20260720_183415` | 19.96 N (peak 20.23 N at 1951 µs) |

**Adopted: 20.0 N** → **T/W ≈ 1.54** against 13.03 N of weight. A workable
VTVL margin; hover sits at ~81% of max rotor speed.

Independently corroborated: the 2026-07-24 coax data, voltage-normalized to a
fresh pack (thrust ∝ V^1.26) and extrapolated to PWM 2000, predicts **17.8 N** —
against 18.07 N actually measured. Two different sessions and methods agreeing
to ~1.5%.

> Earlier revisions of this document quoted 14.68 N and T/W ≈ 1.13. That was
> the raw maximum from the 2026-07-24 map, which (a) never reached full
> throttle and (b) was measured on a partly drained pack. Both corrections
> push the real figure up.

Caveats: the 07-20 files carry **no voltage record**, so pack state is unknown;
and they predate the Pi/Pixhawk, so it is assumed — not proven — that they used
the same rotors as the later coax sessions.

## How the CAD numbers were obtained

`TVC Ver3.stl` has no part boundaries, so
[../tools/stl_mesh.py](../tools/stl_mesh.py) recovers them by grouping
triangles that share vertices — 103,400 triangles → 31 distinct solid bodies —
then computes exact volume/centroid/inertia per body by tetrahedron
decomposition (validated against a unit cube). Names and positions come
separately from the STEP assembly graph, since STL stores neither.

Matching names to mesh bodies is **not fully automatic**: STEP placement
origins are not mesh centroids, so co-located parts (the two gimbals; legs vs.
feet) are ambiguous by proximity alone. The measured-mass table above is
therefore maintained by hand rather than auto-joined.
