from setuptools import find_packages, setup

package_name = 'janus_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/janus_publisher/launch', ['launch/streamer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='workspace',
    maintainer_email='anggayunus139@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streamer = janus_publisher.streamer_node:main',
        ],
    },
)
