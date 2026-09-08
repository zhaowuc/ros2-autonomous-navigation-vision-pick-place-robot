from setuptools import find_packages, setup


package_name = 'c50c_mock_mcu'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description='Byte-accurate pseudo-terminal C50C STM32 protocol simulator.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'c50c_mock_mcu = c50c_mock_mcu.mock_mcu:main',
        ],
    },
)
