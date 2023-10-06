from setuptools import setup
import os
from glob import glob

package_name = 'dobot_diagnostics'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Diagnostics module for Dobot Magician',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alarms_parser = dobot_diagnostics.alarms_parser:main',
        ],
    },
)
