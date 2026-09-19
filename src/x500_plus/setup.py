import os
from glob import glob
from setuptools import setup

package_name = 'x500_plus'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu',
    maintainer_email='liu@todo.todo',
    description='x500_plus gz-sim to ROS2 bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = x500_plus.odom_tf_broadcaster:main',
            'depth_fixer = x500_plus.depth_fixer:main',
            'wind_injector = x500_plus.wind_injector:main',
            'livo_to_px4 = x500_plus.livo_to_px4:main',
        ],
    },
)
