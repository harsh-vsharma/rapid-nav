from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rapid_nav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Vardhan',
    maintainer_email='harsh.v.sharma@outlook.com',
    description='Optical flow for the RAPID-NAV robot: backends, back-projection, flow node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flow_node = rapid_nav_perception.flow_node:main',
            'flow_tracker = rapid_nav_perception.flow_tracker:main',
        ],
    },
)
