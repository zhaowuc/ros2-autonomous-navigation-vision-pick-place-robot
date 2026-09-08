from setuptools import setup

package_name = 'c50c_base_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/c50c_base.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='ROS2 serial driver for the WHEELTEC C50C mecanum base.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'c50c_base_driver_node = c50c_base_driver.c50c_base_driver_node:main',
        ],
    },
)
