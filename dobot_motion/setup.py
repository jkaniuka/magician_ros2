from setuptools import setup
import os
from glob import glob

package_name = 'dobot_motion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Dobot Magician motion execution using ROS 2 actions.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PTP_server = dobot_motion.PTP_server:main',
            'sliding_rail_server = dobot_motion.sliding_rail_server:main',
        ],
    },
)
