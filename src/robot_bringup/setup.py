 


from setuptools import setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],            # 需要有 your_robot_bringup/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Bringup for my robot',
    license='Apache-2.0',
)
