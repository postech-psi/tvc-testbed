"""
One-off: split an STL into its distinct solid bodies and write a fillable
CSV -- this is "how you give per-part CAD masses": fill in mass_g for
whichever rows you already assigned a real mass to in CAD, tag the rest with
a material_group shared with a row you did assign, and mass_properties.py
derives density for the unfilled ones from their same-group siblings.

Geometry columns (volume_mm3, cg_*_mm, bbox_*_mm, I*_per_kg_m3) are always
regenerated from the STL. part_name/mass_g/material_group are preserved
across re-runs, matched by row index -- stable as long as the STL's body
count and order don't change.

Usage:
    python tools/gen_cad_parts.py "path/to/model.stl" [tools/cad_parts.csv]
"""
import csv
import os
import sys

from stl_mesh import load_binary_stl, component_labels, mass_properties


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("usage: python tools/gen_cad_parts.py model.stl [out.csv]")
        return 1
    stl_path = argv[0]
    out_path = argv[1] if len(argv) > 1 else os.path.join(
        os.path.dirname(__file__), "cad_parts.csv")

    existing = {}
    if os.path.exists(out_path):
        with open(out_path, newline="", encoding="utf-8") as f:
            for i, row in enumerate(csv.DictReader(f)):
                existing[i] = (row.get("part_name", ""), row.get("mass_g", ""),
                              row.get("material_group", ""))

    tris_mm = load_binary_stl(stl_path)
    comp_id = component_labels(tris_mm)
    n_comp = comp_id.max() + 1
    print("%d triangles -> %d distinct solid bodies" % (len(tris_mm), n_comp))

    rows = []
    for i in range(n_comp):
        sub_mm = tris_mm[comp_id == i]
        sub_m = sub_mm / 1000.0  # CAD files here are millimeters (see docs)

        mp_mm = mass_properties(sub_mm, density=1.0)   # for human-readable volume/cg
        mp_m = mass_properties(sub_m, density=1.0)      # I_cg here is per unit density
                                                          # (kg/m^3), in kg*m^2 -- ready
                                                          # to scale by a real density.
        bb = sub_mm.reshape(-1, 3).max(axis=0) - sub_mm.reshape(-1, 3).min(axis=0)
        rows.append((i, sub_mm, mp_mm, mp_m, bb))

    rows.sort(key=lambda r: -r[2]["volume"])

    fields = ["idx", "part_name", "mass_g", "material_group",
              "volume_mm3", "cg_x_mm", "cg_y_mm", "cg_z_mm",
              "bbox_x_mm", "bbox_y_mm", "bbox_z_mm", "n_triangles",
              "Ixx_per_kgm3", "Iyy_per_kgm3", "Izz_per_kgm3",
              "Ixy_per_kgm3", "Iyz_per_kgm3", "Izx_per_kgm3"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for new_idx, (orig_i, sub_mm, mp_mm, mp_m, bb) in enumerate(rows):
            name, mass_g, group = existing.get(new_idx, ("", "", ""))
            I = mp_m["I_cg"]
            w.writerow({
                "idx": new_idx, "part_name": name, "mass_g": mass_g, "material_group": group,
                "volume_mm3": round(mp_mm["volume"], 1),
                "cg_x_mm": round(mp_mm["cg"][0], 2), "cg_y_mm": round(mp_mm["cg"][1], 2),
                "cg_z_mm": round(mp_mm["cg"][2], 2),
                "bbox_x_mm": round(bb[0], 1), "bbox_y_mm": round(bb[1], 1),
                "bbox_z_mm": round(bb[2], 1), "n_triangles": len(sub_mm),
                # Stored exactly as they'd sit in a rigid-body inertia tensor
                # (I[0,1] etc. are already the standard NEGATIVE product-of-
                # inertia convention) -- reconstruct with no further sign flip.
                "Ixx_per_kgm3": I[0, 0], "Iyy_per_kgm3": I[1, 1], "Izz_per_kgm3": I[2, 2],
                "Ixy_per_kgm3": I[0, 1], "Iyz_per_kgm3": I[1, 2], "Izx_per_kgm3": I[2, 0],
            })
    print("wrote %s -- fill in part_name/mass_g (or material_group) and re-run "
          "mass_properties.py" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
