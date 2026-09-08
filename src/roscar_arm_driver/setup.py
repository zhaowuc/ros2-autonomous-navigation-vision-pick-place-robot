from setuptools import find_packages, setup


package_name = "roscar_arm_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/scripts", ["scripts/setup_arm_usb.sh"]),
        ("share/" + package_name + "/udev", ["udev/99-roscar-arm.rules"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="car1",
    maintainer_email="car1@example.com",
    description="ROS 2 serial driver for the Zhongling servo board.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_driver_node = roscar_arm_driver.arm_driver_node:main",
        ],
    },
)
