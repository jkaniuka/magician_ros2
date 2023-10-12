from setuptools import setup
import os
from glob import glob

package_name = 'dobot_kinematics'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes/collision/additional_elements/'), glob('meshes/collision/additional_elements/*.dae')),
        (os.path.join('share', package_name, 'meshes/collision/core/'), glob('meshes/collision/core/*.dae')),
        (os.path.join('share', package_name, 'meshes/collision/end_tools/'), glob('meshes/collision/end_tools/*.dae')),
        (os.path.join('share', package_name, 'meshes/collision/urdf_models/'), glob('meshes/collision/urdf_models/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Trajectory validation with collision detection',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_validator_client = dobot_kinematics.trajectory_validator_client:main',
            'trajectory_validator_server = dobot_kinematics.trajectory_validator_server:main',
        ],
    },
)
