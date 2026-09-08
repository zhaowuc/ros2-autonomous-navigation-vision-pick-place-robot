from glob import glob
import os

from setuptools import find_packages, setup


package_name = "n300pro_imu_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="car1",
    maintainer_email="car1@example.com",
    description="ROS 2 serial driver for N300Pro HiPNUC HI91 data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "n300pro_imu_node = n300pro_imu_driver.n300pro_imu_node:main",
        ],
    },
)
