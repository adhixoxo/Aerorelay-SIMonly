import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aerorelay_parcel_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Parcel visualizer for AeroRelay Hub',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'parcel_visualizer_node = aerorelay_parcel_visualizer.parcel_visualizer_node:main',
    ]},
)
