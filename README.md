# gcamp_ros2_basic

## Create Package

```
ros2 launch gcamp_gazebo gcamp_world.launch.py 
ros2 run py_topic_pkg cmd_vel_pub_node 

#TODO : robot state publisher

ros2 pkg create --build-type ament_cmake  cpp_srvcli   --dependencies rclcpp example_interfaces
ros2 pkg create --build-type ament_python py_first_pkg --dependencies rclpy
ros2 pkg create --build-type ament_python py_topic_pkg --dependencies rclpy sensor_msgs geometry_msgs


ros2 interface show geometry_msgs/msg/Twist

$ ros2 pkg create my_python_pkg --build-type ament_python rclpy
$ ros2 pkg create my_cpp_py_pkg --build-type ament_cmake
```

https://answers.ros.org/question/302037/ros2-how-to-call-a-service-from-the-callback-function-of-a-subscriber/