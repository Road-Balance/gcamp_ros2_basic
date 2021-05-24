cmake_minimum_required(VERSION 3.5)
project(cpp_service_pkg)

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
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

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


function(basic_custom_executable target)
  add_executable(${target}_node src/${target}.cpp)
  ament_target_dependencies(${target}_node
    "rclcpp"
    "example_interfaces")
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

function(advanced_custom_executable target)
  add_executable(${target}_node src/${target}.cpp)
  ament_target_dependencies(${target}_node
    "rclcpp"
    "example_interfaces"
    "geometry_msgs"
    "custom_interfaces")
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

basic_custom_executable(basic_server)
basic_custom_executable(basic_client)

advanced_custom_executable(robot_turning_server)
advanced_custom_executable(robot_turning_client)

ament_package()
