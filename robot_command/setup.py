from glob import glob
from setuptools import setup

package_name = 'robot_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngnepiepaye',
    maintainer_email='stephane.ngnepiepaye@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_top = robot_command.camera_top:main',
        'command = robot_command.command:main',
        'waypoint = robot_command.waypoint_gen:main'
        ],
    },
)
