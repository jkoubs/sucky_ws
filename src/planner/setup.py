from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  
        (os.path.join('share', package_name, 'msg'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tilla22',
    maintainer_email='rakhimjonovrikhsitilla16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_planner = planner.my_planner:main',
            'path_coverage_ros = planner.path_coverage_ros:main',
        ],
    },
)
