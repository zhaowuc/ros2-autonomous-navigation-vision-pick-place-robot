from setuptools import setup

package_name = 'roscar_wall_align'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_align.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='Pre-SLAM wall alignment node for ROSCAR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wall_align_node = roscar_wall_align.wall_align_node:main',
        ],
    },
)
