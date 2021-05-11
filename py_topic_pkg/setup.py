import os
from glob import glob
from setuptools import setup

package_name = 'py_topic_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swimming',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_pub_node = py_topic_pkg.cmd_vel_pub:main',
            'laser_sub_node   = py_topic_pkg.laser_sub:main',
            'laser_icp_node   = py_topic_pkg.laser_icp:main',
            'parking_node     = py_topic_pkg.parking:main',
        ],
    },
)
