from setuptools import setup
import os
from glob import glob

package_name = 'face_hmi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Reo Himiya',
    maintainer_email='your.email@example.com',
    description='ROS2 HMI package for robot face display with dual eyes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_hmi = face_hmi.face_hmi_node:main',
        ],
    },
)
