"""
tvc_gui.py
==========
Interactive desktop GUI for the TVC 6-DOF attitude simulator.

Lets you adjust vehicle parameters, control gains, and initial/target attitude
in a form on the left, click "Run Simulation", and see the attitude / rate /
gimbal-command response update live in embedded plots on the right, plus a
summary metrics readout at the bottom.

Usage:
    python3 tvc_gui.py

Requires a display (opens a native Tk window). Depends only on numpy, scipy,
matplotlib, and tkinter (all in the Python standard toolchain / requirements.txt).
"""

import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from tvc_physics import VehicleParams, ControlGains, SimConfig, simulate


# =============================================================================
# Field specification: (label, attr_name, default, kind, group)
# kind is 'float' for numeric entries.
# =============================================================================

VEHICLE_FIELDS = [
    ("Mass  m  [kg]", "m"),
    ("Roll inertia  Ix  [kg·m²]", "Ix"),
    ("Pitch inertia  Iy  [kg·m²]", "Iy"),
    ("Yaw inertia  Iz  [kg·m²]", "Iz"),
    ("Axial lever arm  L  [m]", "L"),
    ("Lateral misalign  dx  [m]", "dx"),
    ("Lateral misalign  dy  [m]", "dy"),
]

ACTUATOR_FIELDS = [
    ("Max thrust  T_max  [N]", "T_max"),
    ("Min thrust  T_min  [N]", "T_min"),
    ("Gimbal limit  [deg]", "gimbal_max_deg"),
    ("Gimbal slew rate  [deg/s]", "gimbal_rate_max_deg"),
]

GAIN_FIELDS = [
    ("Outer loop  Kp (angle)", "kp_angle"),
    ("Inner loop  Kp (rate)", "kp_rate"),
    ("Inner loop  Ki (rate)", "ki_rate"),
    ("Inner loop  Kd (rate)", "kd_rate"),
    ("Rate integrator clamp", "i_limit"),
]

SIM_FIELDS = [
    ("Sim duration  [s]", "t_final"),
    ("Control period  dt  [s]", "dt_ctrl"),
    ("Target roll  [deg]", "roll_des_deg"),
    ("Target pitch  [deg]", "pitch_des_deg"),
    ("Initial roll disturbance  [deg]", "init_roll_deg"),
    ("Initial pitch disturbance  [deg]", "init_pitch_deg"),
]


class LabeledEntryGroup(ttk.LabelFrame):
    """A titled group of label+entry rows bound to attributes of a dataclass instance.

    Label sits above its entry (stacked) rather than side-by-side, so numeric
    values are never clipped regardless of how long the label text is."""

    def __init__(self, parent, title, field_specs, dataclass_instance, **kwargs):
        super().__init__(parent, text=title, padding=(8, 4), **kwargs)
        self.instance = dataclass_instance
        self.vars = {}

        for row, (label, attr) in enumerate(field_specs):
            ttk.Label(self, text=label, font=("TkDefaultFont", 9)).grid(
                row=2 * row, column=0, sticky="w", padx=2, pady=(4, 0)
            )
            var = tk.StringVar(value=str(getattr(dataclass_instance, attr)))
            entry = ttk.Entry(self, textvariable=var, width=16, justify="left")
            entry.grid(row=2 * row + 1, column=0, sticky="ew", padx=2, pady=(0, 4))
            self.vars[attr] = var

        self.columnconfigure(0, weight=1)

    def apply_to_instance(self):
        """Parse current entry text back into the dataclass instance. Raises
        ValueError with a helpful message if a field doesn't parse as float."""
        for attr, var in self.vars.items():
            text = var.get().strip()
            try:
                value = float(text)
            except ValueError:
                raise ValueError(f"Field '{attr}' has invalid number: '{text}'")
            setattr(self.instance, attr, value)


class TVCSimulatorApp(tk.Tk):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.title("TVC 6-DOF Attitude Simulator")
        self.geometry("1400x860")
        self.minsize(1100, 640)

        # Backing dataclasses (kept alive across runs so edits persist)
        self.vparams = VehicleParams()
        self.gains = ControlGains()
        self.cfg = SimConfig()

        self._build_layout()
        # Run once at startup so the window isn't blank
        self.after(100, self.run_simulation)

    # ------------------------------------------------------------------
    def _build_layout(self):
        # Two-pane layout: scrollable control form on the left, plots on the right
        main = ttk.Frame(self)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main, width=300)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        right = ttk.Frame(main)
        right.pack(side="right", fill="both", expand=True)

        self._build_control_panel(left)
        self._build_plot_panel(right)

    # ------------------------------------------------------------------
    def _build_control_panel(self, parent):
        # Scrollable canvas so the form works even on smaller screens
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        form = ttk.Frame(canvas)

        form.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=form, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Mouse-wheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        pad = dict(fill="x", padx=8, pady=6)

        ttk.Label(form, text="Vehicle & TVC Configuration", font=("TkDefaultFont", 13, "bold")).pack(
            anchor="w", padx=8, pady=(10, 2)
        )

        self.vehicle_group = LabeledEntryGroup(form, "Vehicle Parameters", VEHICLE_FIELDS, self.vparams)
        self.vehicle_group.pack(**pad)

        self.actuator_group = LabeledEntryGroup(form, "Actuator Limits", ACTUATOR_FIELDS, self.vparams)
        self.actuator_group.pack(**pad)

        self.gain_group = LabeledEntryGroup(form, "Cascaded PID Gains", GAIN_FIELDS, self.gains)
        self.gain_group.pack(**pad)

        self.sim_group = LabeledEntryGroup(form, "Simulation / Target", SIM_FIELDS, self.cfg)
        self.sim_group.pack(**pad)

        btn_frame = ttk.Frame(form)
        btn_frame.pack(fill="x", padx=8, pady=(12, 4))

        self.run_button = ttk.Button(btn_frame, text="Run Simulation", command=self.run_simulation)
        self.run_button.pack(side="left", expand=True, fill="x", padx=(0, 4))

        reset_button = ttk.Button(btn_frame, text="Reset Defaults", command=self.reset_defaults)
        reset_button.pack(side="left", expand=True, fill="x", padx=(4, 0))

        # Metrics readout
        self.metrics_frame = ttk.LabelFrame(form, text="Last Run — Summary Metrics", padding=(8, 6))
        self.metrics_frame.pack(fill="x", padx=8, pady=(10, 20))
        self.metrics_text = tk.Text(self.metrics_frame, height=9, width=36, font=("Courier", 10),
                                     relief="flat", background=self.cget("background"))
        self.metrics_text.pack(fill="both", expand=True)
        self.metrics_text.configure(state="disabled")

    # ------------------------------------------------------------------
    def _build_plot_panel(self, parent):
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax_att = self.fig.add_subplot(311)
        self.ax_rate = self.fig.add_subplot(312, sharex=self.ax_att)
        self.ax_gimbal = self.fig.add_subplot(313, sharex=self.ax_att)
        self.fig.subplots_adjust(hspace=0.35, left=0.10, right=0.97, top=0.95, bottom=0.08)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, parent)
        toolbar.update()

    # ------------------------------------------------------------------
    def reset_defaults(self):
        self.vparams = VehicleParams()
        self.gains = ControlGains()
        self.cfg = SimConfig()

        for group, instance in [
            (self.vehicle_group, self.vparams),
            (self.actuator_group, self.vparams),
            (self.gain_group, self.gains),
            (self.sim_group, self.cfg),
        ]:
            group.instance = instance
            for attr, var in group.vars.items():
                var.set(str(getattr(instance, attr)))

        self.run_simulation()

    # ------------------------------------------------------------------
    def run_simulation(self):
        try:
            self.vehicle_group.apply_to_instance()
            self.actuator_group.apply_to_instance()
            self.gain_group.apply_to_instance()
            self.sim_group.apply_to_instance()
        except ValueError as e:
            messagebox.showerror("Invalid input", str(e))
            return

        # Basic sanity checks to avoid divide-by-zero / nonsensical runs
        if self.vparams.m <= 0 or self.vparams.Ix <= 0 or self.vparams.Iy <= 0 or self.vparams.Iz <= 0:
            messagebox.showerror("Invalid input", "Mass and all inertias must be positive.")
            return
        if self.vparams.L <= 0:
            messagebox.showerror("Invalid input", "Axial lever arm L must be positive.")
            return
        if self.cfg.dt_ctrl <= 0 or self.cfg.t_final <= 0:
            messagebox.showerror("Invalid input", "Simulation duration and dt must be positive.")
            return

        try:
            result = self.run_button_disabled_call()
        except Exception as e:
            messagebox.showerror("Simulation error", f"{type(e).__name__}: {e}")
            return

        self._update_plots(result)
        self._update_metrics(result["metrics"])

    def run_button_disabled_call(self):
        self.run_button.configure(state="disabled")
        self.update_idletasks()
        try:
            result = simulate(self.vparams, self.gains, self.cfg)
        finally:
            self.run_button.configure(state="normal")
        return result

    # ------------------------------------------------------------------
    def _update_plots(self, result):
        t = result["t"]
        euler = result["euler_deg"]
        delta = result["delta_deg"]
        omega = result["omega_deg"]

        for ax in (self.ax_att, self.ax_rate, self.ax_gimbal):
            ax.clear()

        # Attitude
        self.ax_att.plot(t, euler[:, 0], color="tab:blue", lw=1.6, label="roll")
        self.ax_att.plot(t, euler[:, 1], color="tab:red", lw=1.6, label="pitch")
        self.ax_att.axhline(self.cfg.roll_des_deg, color="tab:blue", ls="--", lw=0.8, alpha=0.6)
        self.ax_att.axhline(self.cfg.pitch_des_deg, color="tab:red", ls="--", lw=0.8, alpha=0.6)
        self.ax_att.set_ylabel("Euler angle [deg]")
        self.ax_att.set_title("Attitude Response")
        self.ax_att.legend(loc="upper right", fontsize=8)
        self.ax_att.grid(True, alpha=0.3)

        # Angular rates
        self.ax_rate.plot(t, omega[:, 0], color="tab:blue", lw=1.0, alpha=0.8, label="ω_x")
        self.ax_rate.plot(t, omega[:, 1], color="tab:red", lw=1.0, alpha=0.8, label="ω_y")
        self.ax_rate.plot(t, omega[:, 2], color="tab:green", lw=1.0, alpha=0.8, label="ω_z")
        self.ax_rate.set_ylabel("Angular rate [deg/s]")
        self.ax_rate.set_title("Body Angular Velocity")
        self.ax_rate.legend(loc="upper right", fontsize=8)
        self.ax_rate.grid(True, alpha=0.3)

        # Gimbal commands
        glim = self.vparams.gimbal_max_deg
        self.ax_gimbal.plot(t, delta[:, 0], color="tab:blue", lw=1.6, label="δ₁ (pitch-plane)")
        self.ax_gimbal.plot(t, delta[:, 1], color="tab:red", lw=1.6, label="δ₂ (roll-plane)")
        self.ax_gimbal.axhline(glim, color="gray", ls=":", lw=1.0, alpha=0.7, label=f"±{glim:g}° limit")
        self.ax_gimbal.axhline(-glim, color="gray", ls=":", lw=1.0, alpha=0.7)
        self.ax_gimbal.set_ylabel("Gimbal angle [deg]")
        self.ax_gimbal.set_xlabel("Time [s]")
        self.ax_gimbal.set_title("Gimbal Command")
        self.ax_gimbal.legend(loc="upper right", fontsize=8)
        self.ax_gimbal.grid(True, alpha=0.3)

        self.canvas.draw_idle()

    def _update_metrics(self, m):
        glim = self.vparams.gimbal_max_deg
        util1 = 100.0 * m["max_delta1_deg"] / glim if glim > 0 else 0.0
        util2 = 100.0 * m["max_delta2_deg"] / glim if glim > 0 else 0.0

        lines = [
            f"Final roll:   {m['final_roll_deg']:+7.3f} deg",
            f"Final pitch:  {m['final_pitch_deg']:+7.3f} deg",
            f"Roll error:   {m['roll_error_deg']:+7.3f} deg",
            f"Pitch error:  {m['pitch_error_deg']:+7.3f} deg",
            f"Settling (2%,pitch): {m['settling_time_s']:6.2f} s",
            f"Max |δ1|: {m['max_delta1_deg']:6.2f} deg  ({util1:5.1f}% of limit)",
            f"Max |δ2|: {m['max_delta2_deg']:6.2f} deg  ({util2:5.1f}% of limit)",
        ]

        warn = []
        if util1 > 90 or util2 > 90:
            warn.append("⚠ Gimbal near saturation — reduce disturbance or retune gains.")
        if warn:
            lines.append("")
            lines.extend(warn)

        self.metrics_text.configure(state="normal")
        self.metrics_text.delete("1.0", "end")
        self.metrics_text.insert("1.0", "\n".join(lines))
        self.metrics_text.configure(state="disabled")


def main():
    app = TVCSimulatorApp()
    app.mainloop()


if __name__ == "__main__":
    main()
