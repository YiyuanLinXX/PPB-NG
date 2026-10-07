from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'amiga_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'simple-pid',
        'pyserial',
        'pyproj',
    ],
    zip_safe=True,
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Dual-antenna UM982 RTK navigation for the Farm-ng Amiga',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_driver = amiga_navigation.um982_driver:main',
            'datum_publisher = amiga_navigation.datum_publisher:main',
            'gnss_waypoint_keyboard_logger = amiga_navigation.gnss_waypoint_keyboard_logger:main',
            'waypoint_follower = amiga_navigation.waypoint_follower:main',
            'robot_odom_logger = amiga_navigation.robot_odom_logger:main',
            'nav_topic_debug_logger = amiga_navigation.nav_topic_debug_logger:main',
            'rtk_monitor = amiga_navigation.rtk_monitor:main',
            'thermal_nuc_safety_monitor = amiga_navigation.thermal_nuc_safety_monitor:main',
            'amiga_serial_bridge = amiga_navigation.amiga_serial_bridge:main',
        ],
    },
)
