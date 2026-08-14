#!/usr/bin/env python3
"""源码树直接运行 go2_navigation.footprint_calibrator。"""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2_navigation.footprint_calibrator import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
