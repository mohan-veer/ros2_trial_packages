from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Two-wheeled robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_py.robot_controller:main',
        ],
    },
)