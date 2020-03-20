cmake_minimum_required(VERSION 3.5)
project(simple_rover_locomotion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(locomotion_mode REQUIRED)

include_directories(include)

set(dependencies
  "locomotion_mode")

add_executable(simple_rover_locomotion_node src/simple_rover_locomotion.cpp)
ament_target_dependencies(simple_rover_locomotion_node ${dependencies})

install(TARGETS
  simple_rover_locomotion_node
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()