from glob import glob
from setuptools import find_packages, setup


package_name = "go2_unitree_sim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hao",
    maintainer_email="hao@example.com",
    description="Gazebo/CHAMP 到 Unitree Go2 ROS 2 Sport API 的兼容桥。",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "unitree_sim_bridge = go2_unitree_sim_bridge.bridge:main",
        ],
    },
)
