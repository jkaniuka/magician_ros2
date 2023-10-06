from setuptools import setup
import os
from glob import glob

package_name = 'dobot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Bringup package for Dobot Magician',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_tool_null = dobot_bringup.set_tool_null:main',
        ],
    },
)
