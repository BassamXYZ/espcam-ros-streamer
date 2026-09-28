from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'esp32_camera_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bassam',
    maintainer_email='bassam.essam.ahmad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'streamer = esp32_camera_streamer.streamer_node:main',
            'aruco_transformer = esp32_camera_streamer.aruco_transformer:main',
            'aruco_detector_node = esp32_camera_streamer.aruco_detector_node:main',
            'laptop_aruco_detector = esp32_camera_streamer.laptop_aruco_detector:main',
            'phone_camera_publisher = esp32_camera_streamer.phone_camera_publisher:main',
        ],
    },
)

