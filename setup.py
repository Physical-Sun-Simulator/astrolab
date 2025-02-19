import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'astrolab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aqiel Oostenbrug',
    maintainer_email='a.oostenbrug@student.tue.nl',
    description='ROS2 package which provides the user interface, arm, table and digital twin nodes and connections for the Physical Sunlight Simulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_interface = astrolab.user_interface_node:main',
            'arm = astrolab.arm_node:main',
            'table = astrolab.table_node:main',
            'visualization = astrolab.visualization_node:main',
            'state_publisher = prototype.state_publisher:main'
        ],
    },
)
