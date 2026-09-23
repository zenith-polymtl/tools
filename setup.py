from setuptools import find_packages, setup

package_name = 'tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Colin Cormier',
    maintainer_email='colinc131@gmail.com',
    description='Shared ROS 2 utilities for Zenith mission repos: topic names, heartbeats',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gcs_heartbeat = tools.gcs_heartbeat:main',
            'drone_heartbeat = tools.drone_heartbeat:main',
        ],
    },
)
