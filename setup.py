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
            'digital_twin = astrolab.digital_twin_node:main'
        ],
    },
)
