from setuptools import setup

package_name = 'roscar_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_online_async.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/slam.launch.py',
            'launch/save_map.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='SLAM Toolbox configuration and map saving helpers for roscar.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arena_boundary = roscar_slam.arena_boundary:main',
            'save_map = roscar_slam.save_map:main',
        ],
    },
)
