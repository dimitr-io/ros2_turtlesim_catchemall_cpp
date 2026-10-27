cmake_minimum_required(VERSION 3.8)
project(turtlesim_catchemall_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom interfaces:
set(msg_files
  "msg/TurtleInfo.msg"
  "srv/GetClosestTurtle.srv"
  "srv/CatchTurtle.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

# For launch and parameter files:
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Executables:
add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp turtlesim geometry_msgs)
 
# For interfaces in this package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(turtle_spawner
  ${cpp_typesupport_target}
)
target_link_libraries(turtle_controller
  ${cpp_typesupport_target}
)
 

# install
install(TARGETS
  turtle_spawner
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
