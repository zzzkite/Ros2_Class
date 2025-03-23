from setuptools import setup
from glob import glob
import os
package_name = 'originbot_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),     
        (os.path.join('share', package_name, 'rviz'), glob('rviz/**')),   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GuYueHome',
    maintainer_email='support@ps-micro.com',
    description='OriginBot visualization with Rviz2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
