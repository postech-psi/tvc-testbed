#!/usr/bin/env python3
"""
tvc.py -- the entry point for everything in this repository.

    python tvc.py --help

It exists so there is exactly one runnable file at the top level, and so that
the sys.path setup for the source tree happens in exactly one place. Every
command it dispatches to lives in src/tvc_control/tvc_control/; see
docs/1-CODE-MAP.md for what is where.

Inside the devcontainer after `colcon build`, `python -m tvc_control` does the
same thing without this file, because the package is then installed.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "src", "tvc_control"))

from tvc_control.__main__ import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
