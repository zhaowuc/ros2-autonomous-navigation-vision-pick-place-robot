from glob import glob
from setuptools import setup


package_name = 'roscar_base_interface'


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='Backend-neutral base interface for ROSCar C50C.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_interface_node = roscar_base_interface.base_interface_node:main',
        ],
    },
)
