from setuptools import setup
import os
from glob import glob

package_name = "brick_wall"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="mohan",
    maintainer_email="mohan@todo.todo",
    description="Brick wall simulation",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "turn_90 = brick_wall.turn_90:main",
            "joint_state_gui_bridge = brick_wall.joint_state_gui_bridge:main",
            "gap_detector = brick_wall.gap_detector:main",
            "auto_navigator = brick_wall.auto_navigator:main",
        ],
    },
)
