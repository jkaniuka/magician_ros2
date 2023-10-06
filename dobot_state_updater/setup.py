from setuptools import setup

package_name = 'dobot_state_updater'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Kaniuka',
    maintainer_email='kan.jan@wp.pl',
    description='Receiving information about robot position and alarms',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = dobot_state_updater.dobot_state_publ:main',
        ],
    },
)
