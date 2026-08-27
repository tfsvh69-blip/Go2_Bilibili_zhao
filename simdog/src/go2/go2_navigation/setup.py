import os

from setuptools import find_packages, setup

package_name = "go2_navigation"


def package_data_files():
    """收集 config、launch、rviz、scripts、tools 和 docs 等资源文件。"""
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ]

    def collect_under(rel_dir):
        """把子目录内所有文件安装到 share/<pkg>/<rel_dir>。"""
        entries = []
        base = os.path.join(os.path.dirname(__file__), rel_dir)
        for root, directories, files in os.walk(base):
            directories[:] = [
                name for name in directories
                if name not in {"__pycache__", ".pytest_cache"}
            ]
            for f in files:
                if f.endswith((".pyc", ".pyo")):
                    continue
                full = os.path.join(root, f)
                rel = os.path.relpath(full, base)
                destination = os.path.join(
                    "share", package_name, rel_dir, os.path.dirname(rel)
                )
                entries.append((destination, [full]))
        return entries

    data_files += collect_under("config")
    data_files += collect_under("behavior_trees")
    data_files += collect_under("launch")
    data_files += collect_under("rviz")
    data_files += collect_under("scripts")
    data_files += collect_under("tools")
    data_files += collect_under("docs")
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=package_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hao",
    maintainer_email="hao@example.com",
    description="Go2 室内平地自主导航：同源地图包管理、NDT 定位、Nav2 规划控制与安全控制链。",
    license="BSD-3-Clause",
    # 让 colcon 的 Python 测试步骤选择 pytest，而不是只会发现 unittest.TestCase
    # 的回退执行器；本包测试均采用 pytest 函数形式。
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "build_map_bundle = go2_navigation.build_map_bundle:main",
            "validate_map_bundle = go2_navigation.validate_map_bundle:main",
            "health_check = go2_navigation.health_check:main",
            "goal_guard = go2_navigation.goal_guard:main",
            "rotation_diagnostics = go2_navigation.rotation_diagnostics:main",
            "safety_supervisor = go2_navigation.safety_supervisor:main",
            "simulation_odom = go2_navigation.simulation_odom:main",
            "nav_tuner = go2_navigation.nav_tuner:main",
            "obstacle_probe = go2_navigation.obstacle_probe:main",
            "footprint_calibrator = go2_navigation.footprint_calibrator:main",
        ],
    },
)
