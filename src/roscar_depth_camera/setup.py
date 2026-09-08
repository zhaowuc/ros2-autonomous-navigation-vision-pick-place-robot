from setuptools import find_packages, setup


package_name = 'roscar_depth_camera'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            ['launch/astra_pro_plus.launch.py', 'launch/gemini_alignment.launch.py'],
        ),
        (
            'share/' + package_name + '/systemd',
            ['systemd/roscar-usb-camera-buffer.service'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roscar',
    maintainer_email='user@example.com',
    description=(
        'Astra Pro Plus integration, web previews, and local-obstacle '
        'point-cloud processing.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_pipeline = roscar_depth_camera.depth_pipeline:main',
            'gemini_alignment = roscar_depth_camera.gemini_alignment:main',
        ],
    },
)
