from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ppbng_imaging'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yl3663',
    maintainer_email='yiyuanlin47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher_node = ppbng_imaging.gps_publisher_node:main',
            'gps_logger_node = ppbng_imaging.gps_logger_node:main',
            'thermal_acquisition_node = ppbng_imaging.thermal_acquisition_node:main',
            'rgb_acquisition_node = ppbng_imaging.rgb_acquisition_node:main',
            'arduino_com_node = ppbng_imaging.arduino_com_node:main',
        ],
    },
)
