from setuptools import setup

package_name = 'basic_action_pkg'

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
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_action_server = basic_action_pkg.fibonacci_action_server:main',
            'fibonacci_action_client = basic_action_pkg.fibonacci_action_client:main',
            'timed_move_server       = basic_action_pkg.timed_move_server:main',
            'turtle_turning_client   = basic_action_pkg.turtle_turning_client:main',
            'pose_fb_server          = basic_action_pkg.pose_fb_server:main',
        ],
    },
)
