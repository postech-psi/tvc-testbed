"""Four commands for running and inspecting the TVC simulation."""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(prog="tvc.py", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    sim = commands.add_parser("sim", help="run the Python simulation; save CSV and PNG")
    sim.add_argument("--duration", type=float, default=5.0, help="simulated seconds")
    sim.add_argument("--dt", type=float, default=0.01, help="control period, seconds")
    sim.add_argument("--altitude", type=float, default=2.0, help="target altitude, metres")
    sim.add_argument("--initial-altitude", type=float, default=2.0)
    sim.add_argument("--pitch", type=float, default=0.0, help="target pitch, degrees (requires --no-position-hold)")
    sim.add_argument("--yaw", type=float, default=0.0, help="target yaw, degrees (requires --no-position-hold)")
    sim.add_argument("--roll", type=float, default=0.0, help="target roll, degrees")
    sim.add_argument("--initial-pitch", type=float, default=3.0)
    sim.add_argument("--initial-yaw", type=float, default=-4.0)
    sim.add_argument("--altitude-hold", action=argparse.BooleanOptionalAction, default=True)
    sim.add_argument("--position-hold", action=argparse.BooleanOptionalAction, default=True)
    sim.add_argument("--battery-sag", action="store_true", help="enable the older bench battery fit")
    sim.add_argument("--output", type=Path, default=Path("out/simulation"), help="output prefix")
    commands.add_parser("gui", help="open settings, plots and CAD playback")
    gazebo = commands.add_parser("gazebo", help="run Gazebo through ROS 2 (Linux/container)")
    gazebo.add_argument("--headless", action="store_true", help="run without the Gazebo window")
    gazebo.add_argument("--altitude", type=float, default=2.0)
    gazebo.add_argument("--log", type=Path, default=Path("out/gazebo.csv"))
    plot = commands.add_parser("plot", help="plot a saved simulation CSV")
    plot.add_argument("log", type=Path)
    plot.add_argument("output", nargs="?", type=Path, default=Path("out/flight.png"))
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        parser.print_help()
        return 0
    args = parser.parse_args(argv)

    if args.command == "gui":
        from .gui import main as gui_main
        return gui_main()
    if args.command == "plot":
        from .plotting import main as plot_main
        return plot_main([str(args.log), str(args.output)])
    if args.command == "gazebo":
        if shutil.which("ros2") is None:
            parser.error("ROS 2 is unavailable. Use the Linux devcontainer and source install/setup.bash; see README.md.")
        return subprocess.call([
            "ros2", "launch", "tvc_control", "gazebo.launch.py",
            f"gui:={'false' if args.headless else 'true'}",
            f"z_des:={args.altitude}", f"log_path:={args.log.resolve()}",
        ])

    from .config import load_gains, load_vehicle_params
    from .simulation import SimConfig, simulate
    from .plotting import four_panel, save_run, series_from_run
    cfg = SimConfig(
        t_final=args.duration, dt_ctrl=args.dt, z_des=args.altitude,
        init_z=args.initial_altitude, att_pitch_des_deg=args.pitch,
        att_yaw_des_deg=args.yaw, att_roll_des_deg=args.roll,
        init_att_pitch_deg=args.initial_pitch, init_att_yaw_deg=args.initial_yaw,
        altitude_hold=args.altitude_hold, position_hold=args.position_hold,
        battery_sag=args.battery_sag,
    )
    if cfg.t_final <= 0 or cfg.dt_ctrl <= 0:
        parser.error("duration and dt must be positive")
    result = simulate(load_vehicle_params(), load_gains(), cfg)
    save_run(result, str(args.output) + ".csv")
    four_panel(series_from_run(result), str(args.output) + ".png",
               target_m=cfg.z_des if cfg.altitude_hold else None)
    m = result["metrics"]
    print(f"Final altitude {m['final_z_m']:.3f} m; "
          f"pitch {m['final_pitch_deg']:.2f}, yaw {m['final_yaw_deg']:.2f}, "
          f"roll {m['final_roll_deg']:.2f} degrees")
    print(f"CSV: {args.output}.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
