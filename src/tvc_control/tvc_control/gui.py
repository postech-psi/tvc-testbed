"""Settings, plots, CSV export and CAD playback for the Python simulation."""
import math
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from .config import load_gains, load_vehicle_params
from .plotting import make_figure, save_run, series_from_run
from .simulation import SimConfig, simulate


RUN_FIELDS = [
    ("Duration [s]", "t_final"), ("Control period [s]", "dt_ctrl"),
    ("Target altitude [m]", "z_des"), ("Initial altitude [m]", "init_z"),
    ("Target x [m]", "x_des"), ("Target y [m]", "y_des"),
    ("Target pitch [deg]", "att_pitch_des_deg"),
    ("Target yaw [deg]", "att_yaw_des_deg"),
    ("Target roll [deg]", "att_roll_des_deg"),
    ("Initial pitch [deg]", "init_att_pitch_deg"),
    ("Initial yaw [deg]", "init_att_yaw_deg"),
    ("Initial roll [deg]", "init_att_roll_deg"),
]
VEHICLE_FIELDS = [
    ("Mass [kg]", "m"), ("Inertia xx [kg m²]", "Ix"),
    ("Inertia yy [kg m²]", "Iy"), ("Inertia zz [kg m²]", "Iz"),
    ("Inertia xy [kg m²]", "Ixy"), ("Inertia xz [kg m²]", "Ixz"),
    ("Inertia yz [kg m²]", "Iyz"), ("TVC lever arm [m]", "L"),
    ("Maximum thrust [N]", "T_max"), ("Minimum thrust [N]", "T_min"),
]
GAIN_FIELDS = [
    ("Attitude P", "kp_angle"), ("Rate P", "kp_rate"),
    ("Rate I", "ki_rate"), ("Rate D", "kd_rate"),
    ("Roll attitude P", "kp_angle_roll"), ("Roll rate P", "kp_rate_roll"),
    ("Altitude P", "kp_alt"), ("Vertical speed P", "kp_vz"),
    ("Position P", "kp_pos"), ("Position D", "kd_pos"),
]


class TVCSimulatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TVC simulation")
        self.geometry("1250x900")
        self.minsize(980, 760)
        self.vparams, self.gains, self.cfg = load_vehicle_params(), load_gains(), SimConfig()
        self.last_result = None
        self.view3d = None
        self.fields = []

        left = ttk.Frame(self, padding=12)
        left.pack(side="left", fill="y")
        ttk.Label(left, text="Python simulation", font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
        ttk.Label(left, text="Edits apply to this session.\nPermanent settings: settings/*.yaml").pack(anchor="w", pady=(4, 12))
        tabs = ttk.Notebook(left)
        tabs.pack(fill="both", expand=True)
        for title, source, specs in (("Run", "cfg", RUN_FIELDS),
                                     ("Vehicle", "vparams", VEHICLE_FIELDS),
                                     ("Gains", "gains", GAIN_FIELDS)):
            panel = ttk.Frame(tabs, padding=8)
            tabs.add(panel, text=title)
            for row, (label, attr) in enumerate(specs):
                ttk.Label(panel, text=label).grid(row=row, column=0, sticky="w", pady=5)
                var = tk.StringVar(value=f"{getattr(getattr(self, source), attr):.8g}")
                ttk.Entry(panel, textvariable=var, width=12).grid(row=row, column=1, padx=(10, 0))
                self.fields.append((source, attr, var))
            if source == "cfg":
                for row, (label, attr) in enumerate((
                    ("Altitude hold", "altitude_hold"),
                    ("Horizontal position hold", "position_hold"),
                    ("Tilt compensation", "tilt_compensation"),
                    ("Battery sag (older bench fit)", "battery_sag"),
                ), start=len(specs)):
                    var = tk.BooleanVar(value=getattr(self.cfg, attr))
                    ttk.Checkbutton(panel, text=label, variable=var).grid(row=row, columnspan=2, sticky="w", pady=4)
                    self.fields.append((source, attr, var))
                ttk.Label(panel, text="Position hold generates pitch/yaw targets.\nDisable it to use the angle targets above.").grid(
                    row=len(specs)+4, columnspan=2, sticky="w", pady=8)

        self.run_button = ttk.Button(left, text="Run simulation", command=self.run_simulation)
        self.run_button.pack(fill="x", pady=(12, 4))
        ttk.Button(left, text="Reset settings", command=self.reset_defaults).pack(fill="x", pady=4)
        ttk.Button(left, text="Save CSV…", command=self.save_csv).pack(fill="x", pady=4)
        ttk.Button(left, text="Play CAD motion", command=self.open_3d_view).pack(fill="x", pady=4)
        self.status = tk.StringVar(value="Ready")
        ttk.Label(left, textvariable=self.status, wraplength=310).pack(anchor="w", pady=10)

        right = ttk.Frame(self, padding=8)
        right.pack(side="right", fill="both", expand=True)
        self.fig = Figure(figsize=(8, 9), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        toolbar = NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False)
        toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.after(100, self.run_simulation)

    def reset_defaults(self):
        self.vparams, self.gains, self.cfg = load_vehicle_params(), load_gains(), SimConfig()
        for source, attr, var in self.fields:
            value = getattr(getattr(self, source), attr)
            var.set(value if isinstance(var, tk.BooleanVar) else f"{value:.8g}")
        self.run_simulation()

    def run_simulation(self):
        try:
            for source, attr, var in self.fields:
                value = var.get() if isinstance(var, tk.BooleanVar) else float(var.get())
                if not math.isfinite(value):
                    raise ValueError(f"{attr} must be finite")
                setattr(getattr(self, source), attr, value)
            if min(self.vparams.m, self.vparams.Ix, self.vparams.Iy, self.vparams.Iz, self.vparams.L) <= 0:
                raise ValueError("Mass, diagonal inertias and lever arm must be positive")
            self.run_button.configure(state="disabled")
            self.status.set("Simulating…")
            self.update_idletasks()
            result = simulate(self.vparams, self.gains, self.cfg)
            self.last_result = result
            make_figure(series_from_run(result), target_m=self.cfg.z_des if self.cfg.altitude_hold else None,
                        figure=self.fig)
            self.canvas.draw_idle()
            m = result["metrics"]
            self.status.set(f"Altitude {m['final_z_m']:.3f} m\n"
                            f"Pitch {m['final_pitch_deg']:.2f}°, yaw {m['final_yaw_deg']:.2f}°, "
                            f"roll {m['final_roll_deg']:.2f}°")
            if self.view3d is not None and self.view3d.winfo_exists():
                self.view3d.set_result(result, self.vparams)
        except (ValueError, RuntimeError) as error:
            self.status.set("Check the settings")
            messagebox.showerror("Simulation", str(error), parent=self)
        finally:
            self.run_button.configure(state="normal")

    def save_csv(self):
        if self.last_result is None:
            return
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv", initialfile="simulation.csv",
                                          filetypes=[("Simulation data", "*.csv")])
        if path:
            save_run(self.last_result, path)
            self.status.set(f"Saved {path}")

    def open_3d_view(self):
        if self.last_result is None:
            return
        if self.view3d is not None and self.view3d.winfo_exists():
            self.view3d.lift()
            return
        from .view3d import View3DWindow
        self.view3d = View3DWindow(self, self.last_result, self.vparams, self.cfg)


def main():
    TVCSimulatorApp().mainloop()
