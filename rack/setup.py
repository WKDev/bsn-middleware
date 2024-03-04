from setuptools import find_packages, setup
import os, glob


package_name = 'rack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sid',
    maintainer_email='sid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gpio_driver = rack.gpio_driver:main",
            "rs485_driver = rack.rs485_driver:main",
            "env_manager = rack.env_manager:main",
            'teleop_rack = agv.teleop_rack:main',
        ],
    },
)
