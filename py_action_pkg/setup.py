from setuptools import setup

package_name = 'py_action_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'fibonacci_action_server = py_action_pkg.fibonacci_action_server:main',
            'fibonacci_action_client = py_action_pkg.fibonacci_action_client:main',
            'maze_action_server      = py_action_pkg.maze_action_server:main',
            'test_maze_action_server = py_action_pkg.test_maze_action_server:main',
            'img_subscriber_node     = py_action_pkg.image_sub:main',
            'odome_sub_node          = py_action_pkg.odom_sub:main',
            'robot_controller        = py_action_pkg.robot_controller:main',
        ],
    },
)
