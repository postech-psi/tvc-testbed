"""
gen_docs.py -- generate the documentation sections that are made of numbers.
================================================================================
    python tools/gen_docs.py            # rewrite the generated sections
    python tools/gen_docs.py --check    # fail if any of them is stale (CI)

WHY
    A number written in a document can drift from the source of truth, and it
    will. This repository has the receipts: a superseded 20.0 N max thrust
    survived in three separate documents after the bench measured 17.79 N, and a
    180 deg/s gimbal slew outlived its measurement at 235/403. Nobody noticed,
    because prose does not fail a test.

    So the rule is: a documented number that can disagree with the YAML is a
    number that must be GENERATED from the YAML. This tool does that, and the
    CI job runs --check.

WHAT IS GENERATED
    docs/1-CODE-MAP.md     the index of every module, class and public function,
                           each described by its own docstring
    docs/2-WALKTHROUGH.md  a real control-path trace, produced by running
                           `tvc.py trace`
    docs/5-PARAMETERS.md   the parameter tables and the control-authority budget
    docs/7-CREDIBILITY.md  what the test suite covers, counted from the suite

HOW
    Generated regions are delimited by sentinel comments, the same mechanism
    tools/mass_properties.py --emit already uses on vehicle_params.yaml:

        <!-- <<<EMIT:name -->
        ...anything here is rewritten...
        <!-- >>>EMIT:name -->

    Everything outside the markers is hand-written and is never touched, so a
    document can be mostly prose with a generated table inside it.
"""
import argparse
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "src", "tvc_control"))

from tvc_control.config import (format_parameter_table, load,  # noqa: E402
                                parameter_rows)


def _authority_table():
    """The control-authority budget: what the vehicle can actually do at hover.

    Generated rather than written because every entry is a product of numbers
    that change when the airframe is re-measured, and a stale authority claim is
    the most expensive kind of stale number here -- it is what gain choices rest
    on.
    """
    import math

    from tvc_control.config import load_vehicle_params
    from tvc_control.gnc.allocation import roll_headroom, roll_limits

    vp = load_vehicle_params()
    T = vp.m * vp.g
    travel = min(min(abs(d) for d in vp.delta_min),
                 min(abs(d) for d in vp.delta_max))
    lat = T * vp.L * math.sin(travel)
    cap = roll_headroom(T, vp)
    lo, hi = roll_limits(T, vp)

    out = [
        "| quantity | value | how it is obtained |",
        "|---|---|---|",
        "| hover thrust | %.2f N (%.0f%% of the %.2f N ceiling) | m*g |"
        % (T, 100 * T / vp.T_max, vp.T_max),
        "| lateral moment | %.4f N*m | T*L*sin(travel), travel = %.2f deg, "
        "the tighter stop |" % (lat, math.degrees(travel)),
        "| lateral angular accel | %.2f rad/s^2 | lateral moment / Ixx |"
        % (lat / vp.Ix),
        "| roll moment, guaranteed both ways | %.4f N*m | the measured feasible "
        "set at hover thrust |" % cap,
        "| roll moment, signed interval | %+.4f .. %+.4f N*m | asymmetric: the "
        "coax wake makes one direction stronger |" % (lo, hi),
        "| roll angular accel | %.2f rad/s^2 | roll moment / Izz, and Izz is "
        "%.1fx smaller than Ixx |" % (cap / vp.Iz, vp.Ix / vp.Iz),
        "| roll : lateral accel ratio | %.1fx | authority-rich, but through a "
        "3x slower actuator |" % ((cap / vp.Iz) / (lat / vp.Ix)),
        "| tau_P cross-coupling | %.1f%% of T*L | rotates the allocation by "
        "%.2f deg, %.0f%% of the tighter axis's travel |"
        % (100 * cap / (T * vp.L), math.degrees(math.atan(cap / (T * vp.L))),
           100 * math.atan(cap / (T * vp.L)) / travel),
        "",
        "Roll authority against total thrust -- it peaks near half throttle and",
        "collapses at both ends, because tau_P is bought with a thrust split:",
        "",
        "| thrust | % of ceiling | reachable tau_P |",
        "|---|---|---|",
    ]
    for frac in (0.30, 0.50, 0.65, T / vp.T_max, 0.85, 0.95):
        Tx = frac * vp.T_max
        a, b = roll_limits(Tx, vp)
        tag = " (hover)" if abs(Tx - T) < 1e-6 else ""
        out.append("| %.2f N%s | %.0f%% | %+.4f .. %+.4f N*m |"
                   % (Tx, tag, 100 * frac, a, b))
    return "\n".join(out)


def _test_coverage():
    """What the test suite covers, counted from the suite itself.

    The count and the one-line summary both come from the test files -- the
    count by parsing for `def test_`, the summary from the first line of each
    module's docstring. A test-suite description maintained by hand in a
    document is a description that goes stale the first time someone adds a
    file, which is the failure this whole generator exists to prevent.
    """
    import ast

    tests_dir = os.path.join(REPO, "tests")
    rows = ["| file | test functions | what it guards |", "|---|---|---|"]
    total = 0
    for name in sorted(os.listdir(tests_dir)):
        if not (name.startswith("test_") and name.endswith(".py")):
            continue
        tree = ast.parse(io.open(os.path.join(tests_dir, name),
                                 encoding="utf-8").read())
        n = sum(1 for node in ast.walk(tree)
                if isinstance(node, ast.FunctionDef)
                and node.name.startswith("test_"))
        doc = (ast.get_docstring(tree) or "").strip().split("\n")[0]
        rows.append("| `%s` | %d | %s |" % (name, n, doc))
        total += n
    rows.append("| | **%d** | |" % total)
    return "\n".join(rows)


# Layer order below is the DEPENDENCY order, which is also the reading order.
_LAYERS = [
    ("gnc", "FLIGHT CODE -- runs on the vehicle. Pure Python by enforced rule."),
    ("plant", "SIMULATION ONLY -- none of this ever flies."),
    ("hal", "Transport adapters: one ActuatorSetpoint into one transport's "
            "units. No control."),
    ("harness", "Owns the clock and the I/O. Picks a plant, wires it to gnc/."),
    ("nodes", "ROS 2 wrappers. Thin by rule: no control math here."),
    ("apps", "Desktop tools. They read the simulator; they are not part of it."),
    ("verify", "Verification: the scenarios, the frozen baseline, the tracer."),
    ("", "Top level: the parameter loader and the CLI."),
]


def _first_line(doc):
    """The opening sentence of a docstring, collapsed onto one line.

    The first line is the summary by convention, but several here wrap, so this
    joins until the first blank line and then cuts at the first sentence end.
    That way no docstring has to be written for this table.
    """
    if not doc:
        return ""
    head = []
    for line in doc.strip().split(chr(10)):
        if not line.strip():
            break
        head.append(line.strip())
    text = " ".join(head).split("=====")[0].strip()
    m = re.search(r"\.(\s|$)", text)
    if m and m.start() > 25:
        text = text[:m.start() + 1]
    return text.replace("|", "\\|").strip()


def _signature(node):
    """`(a, b, c=)` -- argument names, with `=` marking the ones that default."""
    a = node.args
    defaults = [None] * (len(a.args) - len(a.defaults)) + list(a.defaults)
    args = [arg.arg + ("" if d is None else "=")
            for arg, d in zip(a.args, defaults) if arg.arg not in ("self", "cls")]
    if a.vararg:
        args.append("*" + a.vararg.arg)
    if a.kwarg:
        args.append("**" + a.kwarg.arg)
    return "(" + ", ".join(args) + ")"


def _code_index():
    """Every module, class and public function in the package, one line each.

    Parsed from the source, with each description taken from the thing's own
    docstring. A hand-written index of a hundred functions is a promise nobody
    keeps past the second refactor; this one cannot disagree with the code,
    because it IS the code.
    """
    import ast

    base = os.path.join(REPO, "src", "tvc_control", "tvc_control")
    out = []
    for layer, blurb in _LAYERS:
        d = os.path.join(base, layer) if layer else base
        names = sorted(n for n in os.listdir(d)
                       if n.endswith(".py") and n != "__init__.py"
                       and os.path.isfile(os.path.join(d, n)))
        if not names:
            continue
        title = ("`%s/`" % layer) if layer else "`tvc_control/` — top level"
        out += ["", "### %s" % title, "", blurb]
        for n in names:
            tree = ast.parse(io.open(os.path.join(d, n), encoding="utf-8").read())
            out += ["", "**`%s`** — %s" % (n, _first_line(ast.get_docstring(tree)))]
            rows = []
            for node in tree.body:
                if isinstance(node, ast.ClassDef):
                    rows.append(("class `%s`" % node.name,
                                 _first_line(ast.get_docstring(node))))
                    for sub in node.body:
                        if (isinstance(sub, ast.FunctionDef)
                                and not sub.name.startswith("_")):
                            rows.append(
                                ("&nbsp;&nbsp;&nbsp;`.%s%s`"
                                 % (sub.name, _signature(sub)),
                                 _first_line(ast.get_docstring(sub))))
                elif (isinstance(node, ast.FunctionDef)
                      and not node.name.startswith("_")):
                    rows.append(("`%s%s`" % (node.name, _signature(node)),
                                 _first_line(ast.get_docstring(node))))
            if rows:
                out += ["", "| | |", "|---|---|"]
                out += ["| %s | %s |" % (a, b) for a, b in rows]
    return chr(10).join(out).strip()


def _trace_output():
    """A real control-path trace, produced by running the tracer.

    Not a transcription. `tvc.py trace` recomputes each stage with the same
    functions the controller calls and then checks itself against
    TvcController.update(), so what appears in the document either matches the
    real control path or says MISMATCH.
    """
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(REPO, "tvc.py"), "trace"],
                       cwd=REPO, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("tvc.py trace failed:" + chr(10) + r.stdout + r.stderr)
    body = r.stdout.replace(chr(13) + chr(10), chr(10)).rstrip()
    return "```" + chr(10) + body + chr(10) + "```"


def _provenance_summary():
    """How many of the model's numbers are measured, and how many are guesses."""
    rows = parameter_rows()
    counts = {}
    for _, _, _, _, src, _ in rows:
        counts[src] = counts.get(src, 0) + 1
    total = len(rows)
    out = ["| provenance | count | share |", "|---|---|---|"]
    for src in sorted(counts, key=lambda s: -counts[s]):
        out.append("| %s | %d | %.0f%% |"
                   % (src, counts[src], 100.0 * counts[src] / total))
    out.append("| **total** | **%d** | |" % total)
    return "\n".join(out)


# document path -> {marker name: generator}
SECTIONS = {
    os.path.join(REPO, "docs", "1-CODE-MAP.md"): {
        "index": _code_index,
    },
    os.path.join(REPO, "docs", "2-WALKTHROUGH.md"): {
        "trace": _trace_output,
    },
    os.path.join(REPO, "docs", "5-PARAMETERS.md"): {
        "parameters": lambda: format_parameter_table(markdown=True),
        "authority": _authority_table,
        "provenance": _provenance_summary,
    },
    os.path.join(REPO, "docs", "7-CREDIBILITY.md"): {
        "tests": _test_coverage,
    },
}

_PAT = ("<!-- <<<EMIT:%s -->", "<!-- >>>EMIT:%s -->")


def splice(text, name, body):
    start, end = _PAT[0] % name, _PAT[1] % name
    m = re.search(re.escape(start) + r".*?" + re.escape(end), text, re.S)
    if not m:
        raise SystemExit("no EMIT:%s markers found" % name)
    return text[:m.start()] + start + "\n" + body + "\n" + end + text[m.end():]


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate the numeric doc sections.")
    ap.add_argument("--check", action="store_true",
                    help="do not write; exit 1 if any section is stale")
    args = ap.parse_args(argv)

    stale = []
    for path, gens in SECTIONS.items():
        if not os.path.isfile(path):
            raise SystemExit("missing document: %s" % path)
        original = io.open(path, encoding="utf-8").read()
        updated = original
        for name, gen in gens.items():
            updated = splice(updated, name, gen())
        rel = os.path.relpath(path, REPO)
        if updated == original:
            print("  up to date  %s" % rel)
            continue
        if args.check:
            stale.append(rel)
            print("  STALE       %s" % rel)
        else:
            io.open(path, "w", encoding="utf-8", newline="\n").write(updated)
            print("  wrote       %s" % rel)

    if stale:
        print("\n%d document(s) disagree with vehicle_params.yaml. Run\n"
              "    python tools/gen_docs.py\n"
              "and commit the result." % len(stale))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
