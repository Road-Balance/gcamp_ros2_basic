from setuptools import setup

package_name = "py_service_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kimsooyoung",
    maintainer_email="tge1375@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "basic_client = py_service_pkg.basic_srv_client:main",
            "basic_server = py_service_pkg.basic_srv_server:main",
            "gazebo_model_spawner = py_service_pkg.spawn_model:main",
            "custom_srv_server = py_service_pkg.custom_srv_server:main",
            "custom_srv_client = py_service_pkg.custom_srv_client:main",
            "robot_turning_server = py_service_pkg.robot_turning_srv:main",
        ],
    },
)
