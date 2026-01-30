from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'goal_seek'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='earth',
    maintainer_email='sugheert@buffalo.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = goal_seek.publish_odom:main',
            ###############################################
            # Add the executables here
            ###############################################
        ],
    },
)
