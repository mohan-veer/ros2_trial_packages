from setuptools import find_packages, setup

package_name = 'first_pkg'

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
    maintainer_email='mveeraghanta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = first_pkg.first_node:main",
            "robot_news_station = first_pkg.robot_news_station:main",
            "smart_phone = first_pkg.smartphone:main",
            "verify_hardware = first_pkg.verify_new_interface:main"
        ],
    },
)
