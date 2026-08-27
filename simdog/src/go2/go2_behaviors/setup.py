from setuptools import find_packages, setup


package_name = "go2_behaviors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hao",
    maintainer_email="hao@example.com",
    description="基于 ros2_control 标准关节轨迹接口的 Go2 仿真动作。",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "go2_behavior = go2_behaviors.behavior_runner:main",
            "go2_behavior_server = go2_behaviors.behavior_server:main",
        ],
    },
)
