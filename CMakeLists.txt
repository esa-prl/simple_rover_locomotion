cmake_minimum_required(VERSION 3.5)
project(simple_rover_locomotion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(urdf REQUIRED)

include_directories(include)


set(msg_files
  "msg/JointCommand.msg"
  "msg/JointCommandArray.msg"
  )

set(srv_files
  "srv/Activate.srv"
  "srv/ChangeLocomotionMode.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces)

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "std_msgs" "sensor_msgs" "std_srvs" "urdf")

add_executable(simple_rover_locomotion_node src/simple_rover_locomotion.cpp src/locomotion_mode.cpp)
ament_target_dependencies(simple_rover_locomotion_node ${dependencies})
rosidl_target_interfaces(simple_rover_locomotion_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  simple_rover_locomotion_node
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
