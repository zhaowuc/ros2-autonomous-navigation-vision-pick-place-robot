from setuptools import setup

package_name = 'scan_front_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/fake_scan_filter_test.launch.py',
            'launch/scan_front_filter.launch.py',
        ]),
        ('share/' + package_name + '/docs', [
            'docs/global_costmap_wall_filter.md',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='Protected and map-aware LaserScan filters for ROSCAR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_scan_publisher = scan_front_filter.fake_scan_publisher:main',
            'map_aware_global_scan_filter_node = '
            'scan_front_filter.map_aware_global_scan_filter_node:main',
            'scan_front_filter_node = scan_front_filter.scan_front_filter_node:main',
        ],
    },
)
