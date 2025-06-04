from setuptools import find_packages, setup

package_name = 'ros2_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mveeraghanta',
    maintainer_email='mohankalyan01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_publisher = ros2_pub_sub.number_publisher:main",
            "number_subscriber = ros2_pub_sub.number_counter:main"
        ],
    },
)
