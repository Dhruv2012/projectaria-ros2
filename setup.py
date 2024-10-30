from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)

package_name = 'eve'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author_email='skareer@gatech.edu',
    author='Simar Kareer',
    maintainer='Simar Kareer',
    maintainer_email='skareer@gatech.edu',
    description='EgoMimic: Scaling Imitation Learning through Egocentric Video',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['stream_aria_ros = eve.stream_aria_ros:main'
        ],
    },
)
