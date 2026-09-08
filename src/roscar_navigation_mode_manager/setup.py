from setuptools import find_packages, setup

package_name = 'roscar_navigation_mode_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='codex@car1.local',
    description='One-way ROSCAR Nav2 controller selector.',
    license='MIT',
    entry_points={'console_scripts': [
        'navigation_mode_manager = roscar_navigation_mode_manager.mode_manager:main',
    ]},
)
