from setuptools import find_packages, setup
import os, glob

package_name = 'agv'

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
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_middleware = agv.api_middleware:main',
            'teleop = agv.teleop:main',
            'led_manager = agv.led_manager:main',
            'teleop_gamepad = agv.teleop_gamepad:main',
            'work_manager = agv.work_manager:main',
            'dummy_feeder = agv.dummy_feeder:main',
            'integration = agv.integration:main'
        ],

    },
)
