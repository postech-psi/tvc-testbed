"""
Enforce the porting discipline on the flight code.
================================================================================
tvc_control/gnc/ is the directory a PX4 module will be a port of. These tests
are what keep that true, because the discipline is invisible in a diff: nothing
breaks the day someone imports numpy for one convenient call, and by the time
anyone tries the port the dependency is load-bearing in six places.

Each rule below exists for a reason stated in its own test. They are checked by
parsing the source rather than by running it, so a violation on a rarely-taken
branch is caught too.
"""
import ast
import os

import pytest

GNC = None
FILES = []


def _collect(repo):
    global GNC, FILES
    GNC = os.path.join(repo, "src", "tvc_control", "tvc_control", "gnc")
    FILES = sorted(
        os.path.join(GNC, f) for f in os.listdir(GNC)
        if f.endswith(".py") and not f.startswith("__pycache__")
    )
    return FILES


@pytest.fixture(scope="module", autouse=True)
def _setup(repo):
    _collect(repo)


def _parse(path):
    with open(path, encoding="utf-8") as f:
        return ast.parse(f.read(), filename=path)


# Everything a Python-only firmware port can rely on. math and dataclasses map
# onto C++ <cmath> and a plain struct; typing vanishes at compile time.
ALLOWED_IMPORTS = {"math", "dataclasses", "typing"}


def test_no_forbidden_imports():
    """No numpy, scipy, yaml, ROS, or anything else with no firmware equivalent.

    numpy is the one that matters most: an ndarray has no fixed-size C++
    counterpart, so every array in the control path becomes a design decision
    during the port instead of a mechanical translation. scipy is worse -- a
    scipy call is an algorithm someone would have to reimplement, under time
    pressure, without the tests that covered the original.
    """
    bad = []
    for path in FILES:
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.Import):
                names = [a.name.split(".")[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:            # relative import, inside gnc
                    continue
                names = [(node.module or "").split(".")[0]]
            else:
                continue
            for n in names:
                if n and n not in ALLOWED_IMPORTS:
                    bad.append("%s: %s (line %d)"
                               % (os.path.basename(path), n, node.lineno))
    assert not bad, (
        "flight code may only import %s. Found:\n  %s"
        % (sorted(ALLOWED_IMPORTS), "\n  ".join(bad)))


def test_no_file_io():
    """Parameters are injected once at startup, never looked up at runtime.

    This is how PX4's parameter system works, and it is also what makes a
    control step's worst-case time independent of a filesystem. The loader lives
    in tvc_control/config.py, outside this package, on purpose.
    """
    banned = {"open", "input", "print"}
    bad = []
    for path in FILES:
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                if node.func.id in banned:
                    bad.append("%s: %s() at line %d"
                               % (os.path.basename(path), node.func.id, node.lineno))
    assert not bad, "flight code does no I/O. Found:\n  " + "\n  ".join(bad)


def test_no_module_level_mutable_state():
    """No global mutable state -- it defeats multiple instances and testing.

    Module-level constants are fine. A module-level list or dict is a shared
    buffer waiting to be discovered by the second controller instance.
    """
    bad = []
    for path in FILES:
        for node in _parse(path).body:
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                value = node.value
                if isinstance(value, (ast.List, ast.Dict, ast.Set)):
                    bad.append("%s line %d" % (os.path.basename(path), node.lineno))
    assert not bad, ("module-level mutable state in flight code:\n  "
                     + "\n  ".join(bad))


def test_every_loop_is_bounded():
    """No `while` in the flight code: every iteration count must be stated.

    Worst-case execution time has to be a number someone can write down. The
    numerical iterations here (the allocation Newton step, the surface inverse)
    all run a fixed count from a `range`, so the bound is visible at the call
    site rather than argued from convergence.
    """
    bad = []
    for path in FILES:
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.While):
                bad.append("%s line %d" % (os.path.basename(path), node.lineno))
    assert not bad, ("unbounded loop in flight code:\n  " + "\n  ".join(bad))


def test_gnc_never_imports_the_plant():
    """The dependency is one-way. A controller that reaches into the plant
    cannot fly, and this is the check that makes that structural rather than a
    review convention."""
    bad = []
    for path in FILES:
        src = open(path, encoding="utf-8").read()
        for marker in ("from ..plant", "from ..harness", "import plant",
                       "tvc_control.plant", "tvc_control.harness"):
            if marker in src:
                bad.append("%s: %s" % (os.path.basename(path), marker))
    assert not bad, "flight code imports simulation code:\n  " + "\n  ".join(bad)


def test_the_package_actually_imports_without_numpy(monkeypatch):
    """Belt and braces: block numpy at the import hook and load gnc anyway.

    The AST checks above can be satisfied by a lazy import inside a function.
    This one cannot.
    """
    import builtins
    import sys

    for mod in [m for m in sys.modules if m.startswith("tvc_control.gnc")]:
        del sys.modules[mod]

    real_import = builtins.__import__

    def guard(name, *a, **k):
        if name.split(".")[0] in ("numpy", "scipy", "yaml", "rclpy"):
            raise AssertionError("flight code imported %s at runtime" % name)
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", guard)
    import tvc_control.gnc.allocation  # noqa: F401
    import tvc_control.gnc.attitude    # noqa: F401
    import tvc_control.gnc.altitude    # noqa: F401
    import tvc_control.gnc.controller  # noqa: F401
    import tvc_control.gnc.effectiveness  # noqa: F401
    import tvc_control.gnc.position    # noqa: F401
