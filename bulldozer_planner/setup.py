from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bulldozer_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), \
         glob(os.path.join('launch', '*.launch.py'))),
        # Include all .yaml files.
        (os.path.join('share', package_name, 'config'), \
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dinara',
    maintainer_email='d.yaryeva@innopolis.university',
    description='Global planner node for bulldozer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'global_planner = {package_name}.global_planner:main',
        ],
    },
)
