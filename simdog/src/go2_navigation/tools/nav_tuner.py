#!/usr/bin/env python3
"""兼容从源码目录直接启动 nav_tuner。"""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2_navigation.nav_tuner import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
