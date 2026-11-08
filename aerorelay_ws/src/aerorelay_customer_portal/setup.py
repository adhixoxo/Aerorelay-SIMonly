import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aerorelay_customer_portal'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('aerorelay_customer_portal/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Customer portal for AeroRelay Hub',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'customer_portal_node = aerorelay_customer_portal.customer_portal_node:main',
    ]},
)
