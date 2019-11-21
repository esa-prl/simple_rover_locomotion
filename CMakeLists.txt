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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_rover_locomotion src/simple_rover_locomotion.cpp)
ament_target_dependencies(simple_rover_locomotion rclcpp std_msgs)

install(TARGETS
  simple_rover_locomotion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
