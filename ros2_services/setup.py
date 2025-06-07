from setuptools import find_packages, setup

package_name = 'ros2_services'

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
            "add_ints_server = ros2_services.add_two_ints_server:main",
            "add_ints_client_no_oop = ros2_services.add_two_ints_client_no_oop:main",
            "add_ints_client_oop = ros2_services.add_two_ints_client:main",
            "battery = ros2_services.battery_node:main",
            "panel = ros2_services.led_panel_node:main"
        ],
    },
)
