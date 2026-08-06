"""
Exact volume / centroid / inertia tensor of a closed triangle mesh (binary
STL), by tetrahedron decomposition from the origin -- closed form over the
standard simplex, no CAD kernel, no Monte Carlo. Also splits a merged STL
(one file, many CAD bodies) into its distinct watertight solids via
connected-component analysis on shared vertices, since STL itself carries no
part boundaries or names.

Validated against a unit cube in `_validate()` -- run this file directly to
check that still passes before trusting it on a new mesh.
"""
import struct

import numpy as np


def load_binary_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        tris = np.empty((n, 3, 3), dtype=np.float64)
        for i in range(n):
            rec = f.read(50)
            vals = struct.unpack("<12fH", rec)
            tris[i, 0] = vals[3:6]
            tris[i, 1] = vals[6:9]
            tris[i, 2] = vals[9:12]
    return tris


def mass_properties(tris, density=1.0):
    """tris: (M, 3, 3) array of triangle vertices [A,B,C], outward-normal winding.

    Returns volume/mass/cg in the mesh's own units, and I_cg (3x3 inertia
    tensor about the centroid, standard rigid-body sign convention: diagonal
    terms are moments of inertia, off-diagonal are NEGATIVE products of
    inertia -- i.e. I_cg is ready to use directly as a rigid body inertia
    tensor, not a raw second-moment matrix).
    """
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]

    # signed_vol_factor = det[A,B,C] = A . (B x C) = 6 * signed tetra volume
    svf = np.einsum("ij,ij->i", A, np.cross(B, C))
    V = svf.sum() / 6.0

    Mx = (svf / 24.0 * (A[:, 0] + B[:, 0] + C[:, 0])).sum()
    My = (svf / 24.0 * (A[:, 1] + B[:, 1] + C[:, 1])).sum()
    Mz = (svf / 24.0 * (A[:, 2] + B[:, 2] + C[:, 2])).sum()
    cg = np.array([Mx, My, Mz]) / V

    def second_moment(P, Q):
        p0, p1, p2 = A[:, P], B[:, P], C[:, P]
        q0, q1, q2 = A[:, Q], B[:, Q], C[:, Q]
        if P == Q:
            term = (p0**2 + p1**2 + p2**2) + (p0*p1 + p1*p2 + p2*p0)
            return (svf / 60.0 * term).sum()
        term = (2*(p0*q0 + p1*q1 + p2*q2)
                + (p0*q1 + p1*q0) + (p1*q2 + p2*q1) + (p2*q0 + p0*q2))
        return (svf / 120.0 * term).sum()

    Sxx, Syy, Szz = second_moment(0, 0), second_moment(1, 1), second_moment(2, 2)
    Sxy, Syz, Szx = second_moment(0, 1), second_moment(1, 2), second_moment(2, 0)

    # parallel-axis shift from origin to CG, on the raw second-moment matrix
    Sxx -= V * cg[0] * cg[0]
    Syy -= V * cg[1] * cg[1]
    Szz -= V * cg[2] * cg[2]
    Sxy -= V * cg[0] * cg[1]
    Syz -= V * cg[1] * cg[2]
    Szx -= V * cg[2] * cg[0]

    I_cg = density * np.array([
        [Syy + Szz, -Sxy, -Szx],
        [-Sxy, Sxx + Szz, -Syz],
        [-Szx, -Syz, Sxx + Syy],
    ])

    return {"volume": V, "mass": density * V, "cg": cg, "I_cg": I_cg}


def component_labels(tris, snap_decimals=3):
    """Component id per triangle (0..n_components-1), grouping triangles that
    share a vertex (within `snap_decimals`) -- i.e. recovering the distinct
    watertight solids STL itself doesn't record boundaries for.

    Two bodies that were genuinely separate in CAD but happen to touch or
    overlap in the export would merge into one component here; that's a
    mesh-geometry limit, not a bug.
    """
    verts = tris.reshape(-1, 3)
    keys = np.round(verts, snap_decimals)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1, 3)

    n_unique = inverse.max() + 1
    parent = np.arange(n_unique)

    def find(x):
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for a, b, c in inverse:
        union(a, b)
        union(b, c)

    roots = np.array([find(i) for i in range(n_unique)])
    tri_root = roots[inverse[:, 0]]
    _, comp_id = np.unique(tri_root, return_inverse=True)
    return comp_id


def split_components(tris, snap_decimals=3):
    """Split a merged mesh into its distinct watertight solids.

    Returns a list of (triangles_subset, mass_properties_dict) -- properties
    computed in whatever units/density `tris` and `density` (default 1) are
    in -- sorted by volume descending.
    """
    comp_id = component_labels(tris, snap_decimals)
    out = [(tris[comp_id == i], mass_properties(tris[comp_id == i], density=1.0))
           for i in range(comp_id.max() + 1)]
    out.sort(key=lambda pair: -pair[1]["volume"])
    return out


def _make_cube(s=1.0, center=(0, 0, 0)):
    c = np.array(center, dtype=float)
    h = s / 2.0

    def V(xb, yb, zb):
        return np.array([(2*xb-1)*h, (2*yb-1)*h, (2*zb-1)*h]) + c

    raw_faces = [
        (V(0,0,0), V(0,1,0), V(0,1,1)), (V(0,0,0), V(0,1,1), V(0,0,1)),
        (V(1,0,0), V(1,1,0), V(1,1,1)), (V(1,0,0), V(1,1,1), V(1,0,1)),
        (V(0,0,0), V(1,0,0), V(1,0,1)), (V(0,0,0), V(1,0,1), V(0,0,1)),
        (V(0,1,0), V(1,1,0), V(1,1,1)), (V(0,1,0), V(1,1,1), V(0,1,1)),
        (V(0,0,0), V(1,0,0), V(1,1,0)), (V(0,0,0), V(1,1,0), V(0,1,0)),
        (V(0,0,1), V(1,0,1), V(1,1,1)), (V(0,0,1), V(1,1,1), V(0,1,1)),
    ]
    faces = []
    for A, B, C in raw_faces:
        n = np.cross(B - A, C - A)
        tri_center = (A + B + C) / 3.0
        if np.dot(n, tri_center - c) < 0:
            A, B = B, A
        faces.append((A, B, C))
    return np.array(faces)


def _validate():
    tris = _make_cube(s=1.0, center=(0, 0, 0))
    r = mass_properties(tris, density=1.0)
    assert abs(r["volume"] - 1.0) < 1e-9, r["volume"]
    assert np.allclose(r["cg"], [0, 0, 0], atol=1e-9), r["cg"]
    assert np.allclose(r["I_cg"], np.eye(3) / 6.0, atol=1e-9), r["I_cg"]

    tris2 = _make_cube(s=1.0, center=(5.0, -3.0, 2.0))
    r2 = mass_properties(tris2, density=1.0)
    assert abs(r2["volume"] - 1.0) < 1e-9
    assert np.allclose(r2["cg"], [5, -3, 2], atol=1e-9), r2["cg"]
    assert np.allclose(r2["I_cg"], np.eye(3) / 6.0, atol=1e-7), r2["I_cg"]
    print("validation OK: unit cube volume=1, I_cg=1/6 diag, translation-invariant")


if __name__ == "__main__":
    _validate()
