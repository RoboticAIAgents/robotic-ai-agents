from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'microsim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install scenario files
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='MicroSim - Minimal deterministic ROS 2 simulator for Drone and Rover agents',
    license='MIT',
    entry_points={
        'console_scripts': [
            'microsim_node = microsim.microsim_node:main',
        ],
    },
)
