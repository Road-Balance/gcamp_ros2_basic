cmake_minimum_required(VERSION 3.5)
project(cpp_topic_pkg)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# add an abstract target to actually trigger the builds
# add_custom_target(media_files ALL DEPENDS ${urdf_files})

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

# file(GLOB cpp_files src/*.cpp)


# # if error occurs, comment below lines and try colcon build first
# # then, build with uncomment again
# foreach(it ${cpp_files})
#   # # remove .xacro extension
#   string(REGEX MATCH "(.*)[.]cpp$" unused ${it})
#   set(output_filename ${CMAKE_MATCH_1})
#   message(${output_filename})

#   add_executable(${output_filename} src/${output_filename})
#   ament_target_dependencies(${output_filename}_node rclcpp)

#   install(TARGETS
#     ${output_filename}_node
#     DESTINATION lib/${PROJECT_NAME}
#   ) 

#   # # create a rule to generate ${output_filename} from {it}
#   # xacro_add_xacro_file(${it} ${output_filename})

#   # list(APPEND urdf_files ${output_filename})
# endforeach(it)

file(GLOB CPP_FILES src/*.cpp)

# https://4whomtbts.tistory.com/74
foreach( FILE_PATH ${CPP_FILES} )
    # I used a simple string replace, to cut off .cpp.
    get_filename_component(FILE_NAME ${FILE_PATH} NAME_WE)
    
    add_executable( ${FILE_NAME}_node ${FILE_PATH} )

    # Make sure YourLib is linked to each app
    ament_target_dependencies(${FILE_NAME}_node rclcpp sensor_msgs geometry_msgs )

    install(TARGETS
      ${FILE_NAME}_node
      DESTINATION lib/${PROJECT_NAME}
    )

endforeach( FILE_PATH ${APP_SOURCES} )

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
