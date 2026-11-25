from glob import glob

from setuptools import find_packages, setup

package_name = 'rapid_nav_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/experiments', glob('config/experiments/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Vardhan',
    maintainer_email='harsh.v.sharma@outlook.com',
    description='Checks of the RAPID-NAV simulation and planners, and the path viewer.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_check = rapid_nav_tools.sim_check:main',
            'planner_check = rapid_nav_tools.planner_check:main',
            'path_viewer = rapid_nav_tools.path_viewer:main',
            'nav_run = rapid_nav_tools.nav_run:main',
            'obstacle_driver = rapid_nav_tools.obstacle_driver:main',
            'perception_check = rapid_nav_tools.perception_check:main',
            'oracle_obstacles = rapid_nav_tools.oracle_obstacles:main',
            'track_check = rapid_nav_tools.track_check:main',
            'experiment_report = rapid_nav_tools.experiment_report:main',
            'demo_mission = rapid_nav_tools.demo_mission:main',
        ],
    },
)
