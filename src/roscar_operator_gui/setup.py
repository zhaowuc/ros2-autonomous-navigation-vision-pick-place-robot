from glob import glob
from setuptools import find_packages, setup

package_name = 'roscar_operator_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='codex@car1.local',
    description='Native PyQt5 direct-ROS operator GUI.',
    license='MIT',
    entry_points={'console_scripts': [
        'operator_gui = roscar_operator_gui.operator_gui:main',
    ]},
)
