import os
from glob import glob

from setuptools import find_packages, setup


package_name = "screw_moveit_integration"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liuhao",
    maintainer_email="liuhao@example.com",
    description="MoveIt and ros2_control integration for the screw project.",
    license="Apache-2.0",
)
