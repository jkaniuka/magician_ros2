from setuptools import setup
import os
from glob import glob

package_name = 'dobot_alarm_clear'

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
    maintainer='Filip Szyszko',
    maintainer_email='01168917@pw.edu.pl',
    description='Dobot Magician alarm clearing service.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alarm_clear = dobot_alarm_clear.alarm_clear:main',
        ],
    },
)
