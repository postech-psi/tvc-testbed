"""
Final vehicle mass properties: total mass, center of gravity, and inertia
tensor about the CG -> the VehicleParams used by tvc_control/physics.py and
the <inertial> blocks in sim/models/tvc_vehicle/model.sdf.

Three mass sources, combined in one place:

1. tools/cad_parts.csv   -- solids from TVC Ver3.step/.stl with REAL geometry.
   A row with `mass_g` contributes its true inertia tensor (computed from the
   mesh, scaled by mass/volume). A row with only a `material_group` borrows
   density from same-group rows that do have a mass.

2. tools/components.yaml `point_masses` -- electronics and hardware that were
   never modeled in CAD. Point masses: they contribute mass and, via the
   parallel-axis theorem, the m*d^2 term -- which for items 200-600 mm off the
   CG dominates their contribution anyway. Their own spin inertia is
   neglected, which for a GPS puck or an ESC is genuinely negligible.

3. A remainder to reach `design_total_mass_g`, so the answer reflects the
   whole vehicle rather than only the parts that happen to be itemized.

CG is COMPUTED from all three unless `center_of_mass_override_mm` is set.
Everything is reported in one table with its provenance, so it is always
visible which numbers are measured and which are estimated.

Usage:
    python tools/mass_properties.py tools/components.yaml [cad_parts.csv]
"""
import csv
import os
import sys

import numpy as np
import yaml

sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_cad_parts(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def resolve_cad_parts(rows):
    """Attach a mass to each CAD body, directly or via its material group."""
    parsed = []
    for r in rows:
        volume_m3 = float(r["volume_mm3"]) * 1e-9
        cg_m = np.array([float(r["cg_x_mm"]), float(r["cg_y_mm"]),
                         float(r["cg_z_mm"])]) / 1000.0
        I_per_density = np.array([
            [float(r["Ixx_per_kgm3"]), float(r["Ixy_per_kgm3"]), float(r["Izx_per_kgm3"])],
            [float(r["Ixy_per_kgm3"]), float(r["Iyy_per_kgm3"]), float(r["Iyz_per_kgm3"])],
            [float(r["Izx_per_kgm3"]), float(r["Iyz_per_kgm3"]), float(r["Izz_per_kgm3"])],
        ])
        mass_g = (r.get("mass_g") or "").strip()
        parsed.append({
            "idx": r["idx"], "name": r.get("part_name") or ("part_%s" % r["idx"]),
            "group": (r.get("material_group") or "").strip(),
            "volume_m3": volume_m3, "cg_m": cg_m, "I_per_density": I_per_density,
            "mass_kg": float(mass_g) / 1000.0 if mass_g else None,
        })

    group_density = {}
    for p in parsed:
        if p["mass_kg"] is not None and p["group"]:
            group_density.setdefault(p["group"], []).append(p["mass_kg"] / p["volume_m3"])
    group_density = {g: float(np.mean(d)) for g, d in group_density.items()}

    unresolved = []
    for p in parsed:
        if p["mass_kg"] is not None:
            p["source"] = "cad+mass"
        elif p["group"] in group_density:
            p["mass_kg"] = group_density[p["group"]] * p["volume_m3"]
            p["source"] = "cad+group"
        else:
            unresolved.append(p)
    resolved = [p for p in parsed if p["mass_kg"] is not None]
    return resolved, unresolved, group_density


def build_items(cad_parts, doc):
    """One flat list of {name, mass_kg, pos_m, I_local, source} to sum over."""
    items = []

    for p in cad_parts:
        density = p["mass_kg"] / p["volume_m3"]
        items.append({
            "name": p["name"], "mass_kg": p["mass_kg"], "pos_m": p["cg_m"],
            # Real mesh inertia about the part's own centroid.
            "I_local": density * p["I_per_density"], "source": p["source"],
        })

    for c in doc.get("point_masses") or []:
        items.append({
            "name": c["name"], "mass_kg": c["mass_g"] / 1000.0,
            "pos_m": np.array(c["position_mm"], dtype=float) / 1000.0,
            "I_local": np.zeros((3, 3)),   # point mass: m*d^2 only
            "source": "point(est)" if c.get("estimated", True) else "point(cad)",
        })

    design_total = doc.get("design_total_mass_g")
    if design_total:
        known = sum(i["mass_kg"] for i in items) * 1000.0
        gap = design_total - known
        if gap > 1.0:
            items.append({
                "name": "unmodeled remainder", "mass_kg": gap / 1000.0,
                "pos_m": np.array(doc.get("remainder_position_mm", [0, 0, 150]),
                                  dtype=float) / 1000.0,
                "I_local": np.zeros((3, 3)), "source": "remainder",
            })
        elif gap < -1.0:
            print("WARNING: itemized mass (%.1f g) exceeds the design total "
                  "(%.1f g) by %.1f g -- something is double-counted.\n"
                  % (known, design_total, -gap))
    return items


def combine(items, cg_override_mm=None):
    total_mass = sum(i["mass_kg"] for i in items)
    if cg_override_mm is not None:
        cg = np.array(cg_override_mm, dtype=float) / 1000.0
        cg_source = "override (measured balance point)"
    else:
        cg = sum(i["mass_kg"] * i["pos_m"] for i in items) / total_mass
        cg_source = "computed from the masses below"

    I = np.zeros((3, 3))
    for i in items:
        r = i["pos_m"] - cg
        I += i["I_local"] + i["mass_kg"] * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return total_mass, cg, cg_source, I


def report(items, unresolved, group_density, total_mass, cg, cg_source, I):
    print("%-46s %9s %8s %8s %8s  %-11s"
          % ("item", "mass_g", "x_mm", "y_mm", "z_mm", "source"))
    print("-" * 96)
    for it in sorted(items, key=lambda i: -i["mass_kg"]):
        p = it["pos_m"] * 1000
        print("%-46s %9.2f %8.1f %8.1f %8.1f  %-11s"
              % (it["name"][:46], it["mass_kg"] * 1000, p[0], p[1], p[2], it["source"]))
    print("-" * 96)
    print("%-46s %9.2f" % ("TOTAL", total_mass * 1000))

    if group_density:
        print("\nDensities derived per material_group (kg/m^3):")
        for g, d in sorted(group_density.items()):
            print("  %-22s %8.0f" % (g, d))

    if unresolved:
        print("\n%d CAD bodies still have no mass and no usable material_group; "
              "their geometry is ignored (their mass is absorbed by the "
              "remainder instead):" % len(unresolved))
        for p in unresolved[:8]:
            print("  idx=%-3s vol=%9.1f mm^3  cg_z=%7.1f mm"
                  % (p["idx"], p["volume_m3"] * 1e9, p["cg_m"][2] * 1000))
        if len(unresolved) > 8:
            print("  ... and %d more" % (len(unresolved) - 8))

    print("\n" + "=" * 60)
    print("Total mass:  %.4f kg  (%.1f g)" % (total_mass, total_mass * 1000))
    print("CG:          x=%.1f  y=%.1f  z=%.1f mm   [%s]"
          % (cg[0] * 1000, cg[1] * 1000, cg[2] * 1000, cg_source))
    print("\nInertia tensor about the CG (kg*m^2):")
    for row in I:
        print("   [%10.6f %10.6f %10.6f]" % tuple(row))

    off = np.abs(I - np.diag(np.diag(I))).max()
    dm = np.abs(np.diag(I)).mean()
    if dm > 0 and off / dm > 0.05:
        print("\nNOTE: off-diagonal terms are %.1f%% of the mean diagonal. "
              "VehicleParams models only Ix/Iy/Iz, so that asymmetry is "
              "dropped there -- carry the full tensor into the SDF instead, "
              "which does accept products of inertia." % (100 * off / dm))

    print("\n--- VehicleParams (tvc_control/physics.py) ---")
    print("    m: float = %.4f" % total_mass)
    print("    Ix: float = %.6f" % I[0, 0])
    print("    Iy: float = %.6f" % I[1, 1])
    print("    Iz: float = %.6f" % I[2, 2])
    print("    L: float = %.4f    # axial gimbal-pivot -> CM distance" % cg[2])
    if abs(cg[0]) > 1e-4 or abs(cg[1]) > 1e-4:
        print("    dx: float = %.4f   # lateral CG offset (disturbance term)" % cg[0])
        print("    dy: float = %.4f" % cg[1])

    print("\n--- SDF <inertial> for base_link (sim/models/tvc_vehicle/model.sdf) ---")
    print("    Note the SDF's base_link <pose> should place the CG at z=%.3f m." % cg[2])
    print("      <mass>%.4f</mass>" % total_mass)
    print("      <inertia>")
    print("        <ixx>%.6f</ixx>" % I[0, 0])
    print("        <iyy>%.6f</iyy>" % I[1, 1])
    print("        <izz>%.6f</izz>" % I[2, 2])
    print("        <ixy>%.6f</ixy><ixz>%.6f</ixz><iyz>%.6f</iyz>"
          % (I[0, 1], I[0, 2], I[1, 2]))
    print("      </inertia>")


def emit_params_yaml(path, total_mass, cg, I):
    """Rewrite ONLY the mass_properties block (between the EMIT sentinel
    markers) of the single-source-of-truth sim/vehicle_params.yaml, preserving
    every other line and comment. This keeps physics.py / hover.py / the SDF
    generator all reading one authoritative set of numbers -- see that file's
    header for the full contract."""
    start = "# <<<EMIT:mass_properties"
    end = "# >>>EMIT:mass_properties"

    block = (
        "%s\n"
        "mass_properties:\n"
        "  mass_kg: %.4f\n"
        "  cg_mm: [%.1f, %.1f, %.1f]\n"
        "  inertia_kg_m2:\n"
        "    ixx: %.6f\n"
        "    iyy: %.6f\n"
        "    izz: %.6f\n"
        "    ixy: %.6f\n"
        "    ixz: %.6f\n"
        "    iyz: %.6f\n"
        "%s"
    ) % (start, total_mass,
         cg[0] * 1000, cg[1] * 1000, cg[2] * 1000,
         I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2], end)

    with open(path, encoding="utf-8") as f:
        text = f.read()
    if start not in text or end not in text:
        print("ERROR: %s has no EMIT:mass_properties markers to write into.\n"
              "Add the sentinel comments around the mass_properties block first."
              % path)
        return False
    head, _, rest = text.partition(start)
    _, _, tail = rest.partition(end)
    with open(path, "w", encoding="utf-8") as f:
        f.write(head + block + tail)
    print("\nWrote mass_properties block to %s" % path)
    return True


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]

    # Pull out the optional `--emit PATH` flag; the rest are positional.
    emit_path = None
    positional = []
    it = iter(argv)
    for a in it:
        if a == "--emit":
            emit_path = next(it, None)
            if emit_path is None:
                print("--emit needs a path, e.g. --emit sim/vehicle_params.yaml")
                return 1
        else:
            positional.append(a)

    if not positional:
        print("usage: python tools/mass_properties.py components.yaml "
              "[cad_parts.csv] [--emit sim/vehicle_params.yaml]")
        return 1

    doc = load_yaml(positional[0])
    cad_csv = positional[1] if len(positional) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "cad_parts.csv")

    if os.path.exists(cad_csv):
        resolved, unresolved, group_density = resolve_cad_parts(load_cad_parts(cad_csv))
    else:
        print("%s not found -- run gen_cad_parts.py first.\n" % cad_csv)
        resolved, unresolved, group_density = [], [], {}

    items = build_items(resolved, doc)
    total_mass, cg, cg_source, I = combine(
        items, doc.get("center_of_mass_override_mm"))
    report(items, unresolved, group_density, total_mass, cg, cg_source, I)

    if emit_path:
        if not emit_params_yaml(emit_path, total_mass, cg, I):
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
