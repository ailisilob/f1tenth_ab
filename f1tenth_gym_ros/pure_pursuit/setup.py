from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
            'waypoint_logger = pure_pursuit.waypoint_logger_node:main',
        ],
    },
)