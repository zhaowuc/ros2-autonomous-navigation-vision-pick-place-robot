from glob import glob
from setuptools import setup

package_name = 'roscar_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='Nav2 route execution for the ROSCAR C50C mecanum robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_localize = roscar_nav.auto_localize:main',
            'localization_guard = roscar_nav.localization_guard:main',
            'recover_localization_lifecycle = roscar_nav.recover_localization_lifecycle:main',
            'route_executor = roscar_nav.route_executor:main',
        ],
    },
)
