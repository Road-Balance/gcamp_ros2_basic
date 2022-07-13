from setuptools import setup

package_name = 'basic_topic_pkg'

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
            'example_node_1 = basic_topic_pkg.node_example_1:main',
            'example_node_2 = basic_topic_pkg.node_example_2:main',
            'example_node_3 = basic_topic_pkg.node_example_3:main',
            'example_node_4 = basic_topic_pkg.node_example_4:main',
            'example_node_5 = basic_topic_pkg.node_example_5:main',
            'example_node_6 = basic_topic_pkg.node_example_6:main',
            'topic_pub_node = basic_topic_pkg.topic_example_1_publisher:main',
            'topic_sub_node = basic_topic_pkg.topic_example_2_subscriber:main',
            'mimic_node     = basic_topic_pkg.topic_example_4_mimic:main',
        ],
    },
)
