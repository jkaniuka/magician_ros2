from setuptools import setup
import os
from glob import glob

package_name = 'dobot_visualization_tools'
meshes_folder = 'dae_models'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, meshes_folder), glob('dae_models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Useful tools for visualization',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_camera_FOV = dobot_visualization_tools.camera_range:main',
            'show_dobot_range = dobot_visualization_tools.dobot_range:main',
            'show_trajectory = dobot_visualization_tools.trajectory_markers:main',
            'int_marker = dobot_visualization_tools.int_marker:main',
        ],
    },
)
