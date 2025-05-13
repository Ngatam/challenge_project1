from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'challenge_project1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch file 
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ngatam Thiébaut',
    maintainer_email='ngatam.thiebaut@ensam.eu',
    description='Node for the challenge 1 of the ROS project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
         'node_challenge1 = challenge_project1.node_challenge1:main'
        
        ],
    },
)
