# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

import os
from setuptools import setup 
from glob import glob

package_name = 'roboclaw_ros'
submodules = 'roboclaw_ros/driver'


setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
    ('share/ament_index/resource_index/packages',
       ['resource/' + package_name]), 
    ('share/' + package_name, ['package.xml']),
    ( os.path.join('share', package_name, 'launch'), glob('launch/*.py')   ),
   
   
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Chris Kohlhardt',
 maintainer_email='chrisk+github@vidog.com',
 description='A ROS2 node for the RoboClaw motor driver',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'roboclaw_node_exec = roboclaw_ros.roboclaw_node:main'
     ],
   },
)

