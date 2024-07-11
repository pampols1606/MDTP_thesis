from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'sphinx_ros2'

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
            'af_pursuer_sim = sphinx_ros2.af_pursuer_sim:main',
            'af_control_sim = sphinx_ros2.af_control_sim:main',
            'af_step_sim = sphinx_ros2.af_step_sim:main',
		
	],
    },
)
