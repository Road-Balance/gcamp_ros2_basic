from setuptools import setup

package_name = 'basic_service_pkg'

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
    maintainer_email='kimsooyoung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawn_client = basic_service_pkg.turtle_spawn:main',
            'turtle_circle_server = basic_service_pkg.move_turtle_server:main',
            'turtle_jail = basic_service_pkg.turtle_jail:main',
        ],
    },
)
