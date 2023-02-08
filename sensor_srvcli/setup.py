from setuptools import setup

package_name = 'sensor_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_srvcli.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pryor',
    maintainer_email='pryorvo@gmail.com',
    description='Simple ROS2 example for a service client to talk to two service servers'
                'and publish the data',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['server = sensor_srvcli.sensor_server:main',
                            'client = sensor_srvcli.sensor_client:main',
        ],
    },
)
