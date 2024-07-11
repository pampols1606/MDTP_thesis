from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'anafi_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluesky',
    maintainer_email='pampolst@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'af_tracker = anafi_ros2.af_tracker:main',
            'af_3D_bbox = anafi_ros2.af_3D_bbox:main',
            'af_frame_pub = anafi_ros2.af_frame_pub:main',
            'af_pnp = anafi_ros2.af_pnp:main',
            'af_save_data = anafi_ros2.af_save_data:main',
            'af_pursuer = anafi_ros2.af_pursuer:main',
            'af_control = anafi_ros2.af_control:main',
            'af_vicon_step = anafi_ros2.af_vicon_step:main',
        ],
    },
)
