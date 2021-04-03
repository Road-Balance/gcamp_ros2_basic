cmake_minimum_required(VERSION 3.5)
project(cpp_topic_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# for custom msg
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  DEPENDENCIES builtin_interfaces
)


function(custom_executable target)
  add_executable(${target}_node src/${target}.cpp)
  ament_target_dependencies(${target}_node
    "rclcpp"
    "std_msgs")
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# add_executable(topic_pub_node src/topic_pub)
# ament_target_dependencies(topic_pub_node rclcpp)


# add_executable(topic_pub_oop_node src/topic_pub_oop)
# ament_target_dependencies(topic_pub_oop_node rclcpp std_msgs)

# add_executable(topic_sub_node src/topic_sub)
# ament_target_dependencies(topic_sub_node rclcpp)

# add_executable(topic_sub_oop_node src/topic_sub_oop)
# ament_target_dependencies(topic_sub_oop_node rclcpp)

# install(TARGETS
#   topic_pub_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   topic_pub_oop_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   topic_sub_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   topic_sub_oop_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
