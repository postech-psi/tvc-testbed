#!/usr/bin/env python3
"""Source-checkout entry point. Run `python tvc.py --help` to see the commands."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "src", "tvc_control"))

from tvc_control.__main__ import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
