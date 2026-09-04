"""
Animated 3D view of a run produced by harness/mil.py.
================================================================================
    python tvc.py view3d

Renders the REAL vehicle -- the same tvc_vehicle.stl mesh Gazebo shows -- posed
from the logged attitude quaternion, with the gimballed thrust vector drawn at
the rotor plane, so the attitude response can be watched instead of read off a
plot. This is the lightweight counterpart to the Gazebo view: no ROS2, no
physics engine, just the analytic 6-DOF result animated with matplotlib.

Opened from the "3D View" button in the GUI, and runnable standalone (which
runs the default case and animates it).

Depends only on numpy / matplotlib / tkinter, like the rest of the GUI. Poses
come from the quaternion history (never Euler angles), so the view stays correct
through the +/-90 deg attitudes where Euler angles would go singular.
"""

import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import tkinter as tk
from tkinter import ttk

from ..config import load_vehicle_params
from ..gnc.mathx import quat_to_rotmat
from ..gnc.params import VehicleParams

# repo/src/tvc_control/tvc_control/apps -> repo
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", "..", "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
# The SAME mesh Gazebo renders, so the analytic view and the physics-engine view
# show one vehicle rather than two drawings of it.
STL_PATH = os.path.join(_REPO, "gazebo", "models", "tvc_vehicle", "meshes",
                        "tvc_vehicle.stl")
CACHE_DIR = os.path.dirname(STL_PATH)

# The SDF declares <scale>0.001 0.001 0.001</scale> -- the STL is authored in mm.
MESH_SCALE = 0.001

# Vertex-clustering cell size [m] per detail level. The full mesh is 103k
# triangles; matplotlib's 3D renderer costs roughly 20 us per triangle per frame,
# so the whole thing is far too slow to animate. These levels trade silhouette
# fidelity against frame rate -- "Medium" keeps the mast, legs and gimbal ring
# clearly readable at ~4.7k triangles.
DETAIL_LEVELS = {"Low": 0.016, "Medium": 0.010, "High": 0.006}
DEFAULT_DETAIL = "Medium"


# =============================================================================
# STL loading + decimation
# =============================================================================

def load_binary_stl(path):
    """Read a binary STL into an (n_tri, 3, 3) array of vertices, in metres."""
    with open(path, "rb") as f:
        data = f.read()

    n_tri = int(np.frombuffer(data, "<u4", 1, 80)[0])
    expected = 84 + 50 * n_tri
    if len(data) < expected:
        raise ValueError(f"{path}: truncated STL ({len(data)} bytes, expected {expected})")

    # Each 50-byte record is: normal(3f) + 3 vertices(9f) + attribute(u2). The
    # explicit offsets/itemsize keep numpy from inserting alignment padding.
    dt = np.dtype({
        "names": ["v"],
        "formats": [("<f4", (3, 3))],
        "offsets": [12],
        "itemsize": 50,
    })
    tris = np.frombuffer(data, dtype=dt, count=n_tri, offset=84)["v"]
    return tris.astype(np.float64) * MESH_SCALE


def decimate(tris, cell):
    """Vertex-clustering decimation: snap vertices onto a `cell`-sized grid,
    collapse each occupied cell to its member centroid, then drop the triangles
    that became degenerate or duplicated. Preserves overall silhouette and the
    open truss structure, which uniform triangle-dropping would shred."""
    pts = tris.reshape(-1, 3)
    keys = np.floor(pts / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)

    counts = np.bincount(inverse)
    centroids = np.stack(
        [np.bincount(inverse, weights=pts[:, i]) / counts for i in range(3)], axis=1
    )

    tri_idx = inverse.reshape(-1, 3)
    non_degenerate = (
        (tri_idx[:, 0] != tri_idx[:, 1])
        & (tri_idx[:, 1] != tri_idx[:, 2])
        & (tri_idx[:, 0] != tri_idx[:, 2])
    )
    tri_idx = tri_idx[non_degenerate]

    # Collapse duplicate faces (same 3 cells, any winding) that the snap created.
    _, unique_rows = np.unique(np.sort(tri_idx, axis=1), axis=0, return_index=True)
    tri_idx = tri_idx[unique_rows]

    return centroids[tri_idx]


def _vehicle_cg():
    """CG offset of the mesh frame, so the drawn body rotates about its real CM.

    The STL is exported in the CAD assembly frame, whose origin is the gimbal
    pivot; the vehicle rotates about its CM, 211 mm above that. Drawing without
    this offset makes the body swing about the wrong point, which reads as a
    much larger motion than the simulation actually produced.
    """
    from ..config import load
    return np.array(load().cg, dtype=float)


def load_vehicle_mesh(params: VehicleParams, detail=DEFAULT_DETAIL):
    """Return the decimated vehicle mesh as (n_tri, 3, 3), expressed in the BODY
    frame with the origin at the CM -- the frame the dynamics integrate in.

    Each detail level's result is cached next to the STL, so only the first open
    at a given level pays for the decimation. Returns None if the mesh isn't
    available (see build_fallback_geometry)."""
    if not os.path.isfile(STL_PATH):
        return None

    cell = DETAIL_LEVELS[detail]
    cache = os.path.join(CACHE_DIR, f".tvc_vehicle_{int(round(cell * 1000))}mm.npz")

    try:
        if os.path.isfile(cache) and os.path.getmtime(cache) >= os.path.getmtime(STL_PATH):
            return np.load(cache)["tris"]

        tris = decimate(load_binary_stl(STL_PATH), cell)
        # Mesh vertices sit in the model frame (the SDF places base_link's visual
        # so the mesh lands at the model origin); shift to put the CM at (0,0,0).
        tris = tris - _vehicle_cg()
        try:
            np.savez_compressed(cache, tris=tris)
        except OSError:
            pass                                   # read-only checkout: just skip the cache
        return tris
    except Exception as e:
        print(f"[view3d] could not load {STL_PATH}: {e}; using placeholder body")
        return None


def build_fallback_geometry(params: VehicleParams):
    """Simple stand-in body, used only when the STL mesh is missing so the viewer
    still opens: a tube from the rotor plane up past the CM, plus a nose taper."""
    L = params.L
    faces = []
    for z0, z1, r0, r1 in ((-L, L * 0.75, 0.045, 0.045), (L * 0.75, L * 0.75 + 0.09, 0.045, 0.004)):
        th = np.linspace(0, 2 * np.pi, 20, endpoint=False)
        ring0 = np.stack([r0 * np.cos(th), r0 * np.sin(th), np.full(20, z0)], axis=1)
        ring1 = np.stack([r1 * np.cos(th), r1 * np.sin(th), np.full(20, z1)], axis=1)
        for i in range(20):
            j = (i + 1) % 20
            faces.append([ring0[i], ring0[j], ring1[j]])
            faces.append([ring0[i], ring1[j], ring1[i]])
    return np.array(faces)


# =============================================================================
# Shading
# =============================================================================

LIGHT_DIR = np.array([0.4, -0.7, 0.6])
LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)
BASE_RGB = np.array([0.45, 0.50, 0.58])      # matches the SDF <diffuse> for base_link


def shade(tris_inertial):
    """Lambert-shade each face from its inertial-frame normal, so the body reads
    as a solid object while it rotates (flat fill turns the truss into a blob)."""
    e1 = tris_inertial[:, 1] - tris_inertial[:, 0]
    e2 = tris_inertial[:, 2] - tris_inertial[:, 0]
    n = np.cross(e1, e2)
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.maximum(norm, 1e-12)

    lambert = np.abs(n @ LIGHT_DIR)              # abs(): STL winding isn't trusted
    intensity = 0.35 + 0.65 * lambert            # ambient floor + diffuse term
    return np.clip(intensity[:, None] * BASE_RGB, 0, 1)


def thrust_vector_body(delta1_deg, delta2_deg, length):
    """Gimballed thrust direction in the body frame, using exactly the mapping the
    dynamics use: F = T [sin d1, -sin d2 cos d1, cos d1 cos d2]. Returned scaled
    to `length`, pointing along +F (the direction the thrust pushes the vehicle)."""
    d1, d2 = np.deg2rad(delta1_deg), np.deg2rad(delta2_deg)
    f = np.array([np.sin(d1), -np.sin(d2) * np.cos(d1), np.cos(d1) * np.cos(d2)])
    return f * length


class View3DWindow(tk.Toplevel):
    """Playback window: 3D vehicle animation with play/pause, scrub, and speed."""

    def __init__(self, master, result, params: VehicleParams, cfg=None):
        super().__init__(master)
        self.title("TVC — 3D Attitude View")
        self.geometry("820x760")
        self.minsize(560, 520)

        self.params = params
        self.detail = DEFAULT_DETAIL
        self._load_geometry(params)

        self.playing = False
        self._job = None
        self._wall_start = None       # wall-clock anchor for real-time playback
        self._sim_start = 0.0

        self._build_widgets()
        self.set_result(result)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    def _load_geometry(self, params):
        """Fetch the vehicle mesh (real STL, or the placeholder if it's missing)
        and size the view box to whatever we ended up with."""
        mesh = load_vehicle_mesh(params, self.detail)
        self.using_real_mesh = mesh is not None
        self.faces_body = mesh if self.using_real_mesh else build_fallback_geometry(params)

        # Half-extent of a cube that comfortably contains the body in any attitude.
        self.view_radius = float(np.abs(self.faces_body.reshape(-1, 3)).max()) * 1.25

    # ------------------------------------------------------------------
    def _build_widgets(self):
        self.fig = Figure(figsize=(7, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        bar = ttk.Frame(self, padding=(8, 6))
        bar.pack(fill="x")

        self.play_button = ttk.Button(bar, text="▶ Play", width=9, command=self.toggle_play)
        self.play_button.pack(side="left")

        ttk.Button(bar, text="⏮ Restart", width=10, command=self.restart).pack(side="left", padx=(6, 12))

        self.frame_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(bar, from_=0, to=1, variable=self.frame_var,
                                command=self._on_scrub, orient="horizontal")
        self.slider.pack(side="left", fill="x", expand=True)

        ttk.Label(bar, text="Speed").pack(side="left", padx=(12, 4))
        self.speed_var = tk.StringVar(value="1x")
        speed = ttk.Combobox(bar, textvariable=self.speed_var, width=5, state="readonly",
                             values=("0.25x", "0.5x", "1x", "2x", "4x"))
        speed.pack(side="left")

        ttk.Label(bar, text="Detail").pack(side="left", padx=(12, 4))
        self.detail_var = tk.StringVar(value=self.detail)
        detail_box = ttk.Combobox(bar, textvariable=self.detail_var, width=7, state="readonly",
                                  values=("Low", "Medium", "High"))
        detail_box.pack(side="left")
        detail_box.bind("<<ComboboxSelected>>", self._on_detail_change)

        self.time_label = ttk.Label(bar, text="t = 0.00 s", width=12, font=("Courier", 10))
        self.time_label.pack(side="left", padx=(10, 0))

    # ------------------------------------------------------------------
    def set_result(self, result, params: VehicleParams = None):
        """Load (or reload, after a re-run) a simulation result and show frame 0.
        Pass `params` when the vehicle was re-parameterized, so the drawn geometry
        (lever arm, nozzle station) follows the numbers that were simulated."""
        self.playing = False
        self.play_button.configure(text="▶ Play")
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None

        if params is not None:
            self.params = params
            self._load_geometry(self.params)

        self.quat = result["quat"]
        self.t = result["t"]
        self.delta = result["delta_deg"]
        self.n = len(self.t)
        self.dt = float(self.t[1] - self.t[0]) if self.n > 1 else 0.01

        self.frame = 0

        self.slider.configure(to=max(self.n - 1, 1))
        self._draw_static()
        self.show_frame(0)

    # ------------------------------------------------------------------
    def _draw_static(self):
        """Axes, ground plane, and the artists that get updated each frame."""
        ax = self.ax
        ax.clear()

        R = self.view_radius
        ax.set_xlim(-R, R)
        ax.set_ylim(-R, R)
        ax.set_zlim(-R, R)
        try:
            ax.set_box_aspect((1, 1, 1))
        except AttributeError:      # matplotlib < 3.3
            pass

        ax.set_xlabel("x  [m]")
        ax.set_ylabel("y  [m]")
        ax.set_zlabel("z  [m]")
        ax.view_init(elev=18, azim=-60)
        ax.grid(True, alpha=0.25)

        # Faint ground/reference plane at the bottom of the box for depth cues.
        g = np.linspace(-R, R, 2)
        gx, gy = np.meshgrid(g, g)
        ax.plot_surface(gx, gy, np.full_like(gx, -R), color="#8899aa", alpha=0.12,
                        shade=False, zorder=0)

        # Inertial vertical reference: where the thrust axis would point at rest.
        ax.plot([0, 0], [0, 0], [-R, R], color="#889", ls=":", lw=1.0, alpha=0.7)

        # No per-face edges: at ~10k triangles the wireframe swamps the surface.
        self.body = Poly3DCollection(self.faces_body, edgecolors="none", linewidths=0)
        ax.add_collection3d(self.body)

        # Thrust vector and body-z axis, redrawn each frame.
        self.plume_line, = ax.plot([], [], [], color="#ff8c1a", lw=4, alpha=0.95,
                                   solid_capstyle="round")
        self.axis_line, = ax.plot([], [], [], color="#2b2f36", lw=1.2, ls="--", alpha=0.8)

    # ------------------------------------------------------------------
    def show_frame(self, k):
        k = int(np.clip(k, 0, self.n - 1))
        self.frame = k

        R_bi = quat_to_rotmat(self.quat[k])          # body -> inertial

        verts = self.faces_body @ R_bi.T             # rotate every face's vertices
        self.body.set_verts(verts)
        self.body.set_facecolor(shade(verts))

        L = self.params.L
        # Thrust vector at the rotor plane (z = -L below the CM), drawn along the
        # commanded gimbal direction -- the force actually pushing the vehicle.
        rotor_b = np.array([0.0, 0.0, -L])
        tip_b = rotor_b + thrust_vector_body(self.delta[k, 0], self.delta[k, 1], L * 1.1)
        p0, p1 = R_bi @ rotor_b, R_bi @ tip_b
        self.plume_line.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        self.plume_line.set_3d_properties([p0[2], p1[2]])

        # Body +z axis, so the tilt against the dotted inertial vertical is legible.
        a0, a1 = R_bi @ np.array([0, 0, -L]), R_bi @ np.array([0, 0, L * 1.5])
        self.axis_line.set_data([a0[0], a1[0]], [a0[1], a1[1]])
        self.axis_line.set_3d_properties([a0[2], a1[2]])

        self.time_label.configure(text=f"t = {self.t[k]:5.2f} s")
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def _on_scrub(self, _value):
        if self.playing:
            return                                  # slider is being driven by playback
        self.show_frame(self.frame_var.get())

    def toggle_play(self):
        self.playing = not self.playing
        self.play_button.configure(text="⏸ Pause" if self.playing else "▶ Play")
        if self.playing:
            if self.frame >= self.n - 1:
                self.frame = 0
            self._wall_start = time.monotonic()
            self._sim_start = self.frame * self.dt
            self._tick()
        elif self._job is not None:
            self.after_cancel(self._job)
            self._job = None

    def restart(self):
        self.frame = 0
        self.frame_var.set(0)
        self._wall_start = time.monotonic()
        self._sim_start = 0.0
        self.show_frame(0)

    def _on_detail_change(self, _event=None):
        """Swap mesh resolution without losing the current playback position."""
        self.detail = self.detail_var.get()
        self._load_geometry(self.params)
        self._draw_static()
        self.show_frame(self.frame)
        # Re-anchor so the skipped time during reloading isn't replayed at speed.
        self._wall_start = time.monotonic()
        self._sim_start = self.frame * self.dt

    def _tick(self):
        """Advance to whichever frame matches elapsed wall time.

        Deliberately time-driven rather than frame-driven: matplotlib's 3D
        renderer costs ~100-200 ms per frame on this mesh, so stepping one frame
        per tick would play a 5 s manoeuvre back over half a minute. Instead we
        map wall time onto sim time and skip whatever frames we couldn't draw, so
        playback always runs at the selected multiple of real time -- coarser in
        time when rendering is slow, but never wrong about the rate."""
        if not self.playing:
            return

        speed = float(self.speed_var.get().rstrip("x"))
        elapsed = time.monotonic() - self._wall_start
        sim_t = self._sim_start + elapsed * speed

        frame = int(round(sim_t / self.dt))
        if frame >= self.n - 1:
            frame = self.n - 1
            self.playing = False
            self.play_button.configure(text="\u25b6 Play")

        self.frame_var.set(frame)
        self.show_frame(frame)

        if self.playing:
            self._job = self.after(10, self._tick)

    def _on_close(self):
        self.playing = False
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None
        self.destroy()


def main(argv=None):
    """Standalone: run the default case and animate it.

    Gains come from control_gains.yaml like every other pipeline. Constructing
    a bare ControlGains() here would silently animate the `analytic_legacy`
    defaults instead of the flown set, and the two behave visibly differently.
    """
    from ..config import load_gains
    from ..harness.mil import SimConfig, simulate

    vp, gains, cfg = load_vehicle_params(), load_gains(), SimConfig()
    result = simulate(vp, gains, cfg)

    root = tk.Tk()
    root.withdraw()
    win = View3DWindow(root, result, vp, cfg)
    win.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
