cmake_minimum_required(VERSION 3.5)
project(cpp_first_pkg)

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

# file(GLOB CPP_FILES src/*.cpp)
# # https://4whomtbts.tistory.com/74
# foreach( FILE_PATH ${CPP_FILES} )
#     # I used a simple string replace, to cut off .cpp.
#     get_filename_component(FILE_NAME ${FILE_PATH} NAME_WE)
#     add_executable( ${FILE_NAME}_node ${FILE_PATH} )

#     # Make sure YourLib is linked to each app
#     ament_target_dependencies(${FILE_NAME}_node rclcpp )

#     install(TARGETS
#       ${FILE_NAME}_node
#       DESTINATION lib/${PROJECT_NAME}
#     )
# endforeach( FILE_PATH ${APP_SOURCES} )

function(custom_executable target)
  add_executable(${target}_node src/${target}.cpp)
  ament_target_dependencies(${target}_node
    "rclcpp")
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(lifecycle)
custom_executable(simple_loop)
custom_executable(simple_oop)
custom_executable(simple_without_oop)
custom_executable(simple)

ament_package()

# add_executable(simple_node src/simple.cpp)
# ament_target_dependencies(simple_node rclcpp)

# add_executable(simple_loop_node src/simple_loop.cpp)
# ament_target_dependencies(simple_loop_node rclcpp)

# add_executable(simple_oop_node src/simple_oop)
# ament_target_dependencies(simple_oop_node rclcpp)

# add_executable(simple_without_oop_node src/simple_without_oop)
# ament_target_dependencies(simple_without_oop_node rclcpp)

# install(TARGETS
#   simple_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   simple_loop_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   simple_oop_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   simple_without_oop_node
#   DESTINATION lib/${PROJECT_NAME}
# )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


