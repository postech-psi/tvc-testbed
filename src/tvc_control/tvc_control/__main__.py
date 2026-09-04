"""
The one entry point. Everything runnable in this repository is a subcommand.
================================================================================
    python tvc.py --help

WHY ONE ENTRY POINT
    There used to be six executable scripts scattered across three directories,
    each with its own `sys.path.insert` prelude, and no way to discover them
    except by reading a README that listed five of them. `--help` is now the
    list, and the path setup happens once in tvc.py.

Subcommands are grouped by what they need:

    no dependencies beyond Python
        validate   the five closed-loop scenarios (the fast inner loop)
        trace      every computation in one control step, with the numbers
                   (docs/2-WALKTHROUGH.md is this output, annotated)
        golden     capture or check the frozen numerical baseline
        params     print every vehicle parameter with its provenance
        plot       render a Gazebo flight log as plots

    needs a display
        gui        the Tkinter parameter/gains form over the analytic plant
        view3d     animate a run on the real CAD mesh

    needs the devcontainer (gz-transport)
        hover      fly the Gazebo vehicle under the shared flight code
        record     capture the Gazebo chase camera into a GIF

Each subcommand's own `--help` documents its options. `--` is not needed:
    python tvc.py validate --verbose
    python tvc.py hover --duration 30 --altitude 2.0
"""
import argparse
import sys

# (name, help line, "module:function" resolved lazily)
#
# Lazy on purpose. `gui` needs tkinter, `hover` needs gz-transport, `plot` needs
# matplotlib -- and none of those exist everywhere. Importing them all up front
# would make `python tvc.py validate` fail on a machine that merely lacks a
# display, which is the machine most likely to be running it.
COMMANDS = [
    ("validate", "run the five closed-loop scenarios (no ROS, no Gazebo)",
     "tvc_control.verify.scenarios:main"),
    ("golden", "capture (or --check) the frozen numerical baseline",
     "tvc_control.verify.golden:main"),
    ("params", "print every vehicle parameter with its provenance",
     None),
    ("trace", "print every computation in one control step, with numbers",
     "tvc_control.verify.trace:main"),
    ("gui", "open the Tkinter simulator GUI (needs a display)",
     "tvc_control.apps.gui:main"),
    ("view3d", "animate a run on the CAD mesh (needs a display)",
     "tvc_control.apps.view3d:main"),
    ("plot", "plot a Gazebo flight log CSV",
     "tvc_control.apps.plot:main"),
    ("hover", "fly the Gazebo vehicle (needs the devcontainer)",
     "tvc_control.harness.gz:main"),
    ("record", "record the Gazebo chase camera to a GIF (devcontainer)",
     "tvc_control.apps.record:main"),
]


def _resolve(spec):
    mod_name, func_name = spec.split(":")
    import importlib
    return getattr(importlib.import_module(mod_name), func_name)


def _params(argv):
    ap = argparse.ArgumentParser(prog="tvc.py params")
    ap.add_argument("--markdown", action="store_true",
                    help="emit the markdown table docs/5-PARAMETERS.md carries")
    args = ap.parse_args(argv)
    from .config import format_parameter_table, load, parameter_rows
    if args.markdown:
        print(format_parameter_table(markdown=True))
        return 0

    v, rows = load(), parameter_rows()
    tally = {}
    for _, _, _, _, src, _ in rows:
        tally[src] = tally.get(src, 0) + 1
    print("%.4f kg, T/W %.2f, hover at %.0f%% throttle"
          % (v.mass, v.thrust_at_max_n / v.weight_n,
             100 * v.weight_n / v.thrust_at_max_n))
    print("%d parameters: %s"
          % (len(rows), ", ".join("%d %s" % (n, s) for s, n
                                  in sorted(tally.items(), key=lambda kv: -kv[1]))))
    print("provenance and uncertainty: docs/5-PARAMETERS.md")
    print(format_parameter_table(rows, markdown=False))
    return 0


def main(argv=None):
    """Dispatch one subcommand. With no arguments, print the command list."""
    argv = list(sys.argv[1:] if argv is None else argv)
    names = [c[0] for c in COMMANDS]

    if not argv or argv[0] in ("-h", "--help", "help"):
        width = max(len(n) for n in names)
        print(__doc__.strip().split("\n\n")[0])
        print("\nusage: python tvc.py <command> [options]\n")
        for name, helptext, _ in COMMANDS:
            print("  %-*s  %s" % (width, name, helptext))
        print("\n  python tvc.py <command> --help   for that command's options")
        return 0

    cmd, rest = argv[0], argv[1:]
    if cmd not in names:
        print("unknown command %r. Known: %s" % (cmd, ", ".join(names)),
              file=sys.stderr)
        return 2

    if cmd == "params":
        return _params(rest)
    return _resolve(dict((c[0], c[2]) for c in COMMANDS)[cmd])(rest) or 0


if __name__ == "__main__":
    sys.exit(main())
