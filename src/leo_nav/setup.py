import os
from glob import glob
from setuptools import setup

package_name = 'leo_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfredo',
    maintainer_email='amamanisai@unsa.edu.pe',
    description='This Package implement Postion Control for Diferential Mobile Robot (Turtlebot3)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'turtle_pos_controller = leo_nav.pos_controller:main',
        	'leo_pos_controller = leo_nav.leo_pos_controller:main',
        ],
    },
)
