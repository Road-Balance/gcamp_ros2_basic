# gcamp_ros2_basic

## Create Package

```
ros2 pkg create --build-type ament_cmake  cpp_srvcli   --dependencies rclcpp example_interfaces
ros2 pkg create --build-type ament_python py_first_pkg --dependencies rclpy

$ ros2 pkg create my_python_pkg --build-type ament_python rclpy
$ ros2 pkg create my_cpp_py_pkg --build-type ament_cmake
```