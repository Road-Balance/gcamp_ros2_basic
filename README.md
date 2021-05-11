# gcamp_ros2_basic

## Create Package

```
rosdep install -i --from-path src --rosdistro foxy -y

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/skidbot

ros2 bag record -a -o skidbot_record 

ros2 launch gcamp_gazebo gcamp_world.launch.py 
ros2 launch gcamp_gazebo maze_world.launch.py 
#TODO : robot state publisher

ros2 run py_topic_pkg cmd_vel_pub_node 
ros2 run py_topic_pkg laser_sub_node
ros2 run py_topic_pkg parking_node

ros2 interface show custom_interfaces/srv/AddThreeInts
# "rosfoxy" required after custom msg/srv build


ros2 run py_service_pkg gazebo_model_spawner
ros2 run py_service_pkg robot_turning_server
ros2 service call /turn_robot custom_interfaces/srv/TurningControl "{time_duration: 5, angular_vel_z: 1.0, linear_vel_x: 0.5}"
ros2 run py_service_pkg robot_turning_client

ros2 run py_action_pkg fibonacci_action_server 
ros2 action send_goal fibonacci custom_interfaces/action/Fibonacci "{order: 5}"
ros2 action send_goal --feedback fibonacci custom_interfaces/action/Fibonacci "{order: 5}"
ros2 run py_action_pkg fibonacci_action_client 

ros2 run image_view image_view --ros-args --remap /image:=/skidbot/camera_sensor/image_raw
ros2 run py_action_pkg img_subscriber_node 



ros2 pkg create --build-type ament_cmake  cpp_srvcli     --dependencies rclcpp example_interfaces
ros2 pkg create --build-type ament_cmake  custom_interfaces

ros2 pkg create --build-type ament_python py_first_pkg   --dependencies rclpy
ros2 pkg create --build-type ament_python py_topic_pkg   --dependencies rclpy sensor_msgs geometry_msgs
ros2 pkg create --build-type ament_python py_service_pkg --dependencies rclpy gazebo_msgs
ros2 pkg create --build-type ament_python py_action_pkg --dependencies rclpy gazebo_msgs custom_interfaces image_transport cv_bridge sensor_msgs std_msgs opencv2

ros2 interface show geometry_msgs/msg/Twist

$ ros2 pkg create my_python_pkg --build-type ament_python rclpy
$ ros2 pkg create my_cpp_py_pkg --build-type ament_cmake
```

https://answers.ros.org/question/302037/ros2-how-to-call-a-service-from-the-callback-function-of-a-subscriber/