from setuptools import find_packages, setup

package_name = 'roscar_command_arbiter'

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
    description='Single velocity authority for ROSCAR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'command_arbiter = roscar_command_arbiter.command_arbiter:main',
            'nav_command_limiter = roscar_command_arbiter.nav_command_limiter:main',
        ],
    },
)
