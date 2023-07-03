cmake_minimum_required(VERSION 3.5)
project(kangaroo_ros2)

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
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)

## Declare a cpp library
add_library(kang_lib src/kang_lib.cpp)

include_directories(
  include include/
)

include_directories(  
  ${Boost_INCLUDE_DIRS}
)

add_executable(kangaroo_driver_node 
    src/kangaroo_driver_node.cpp 
    src/kangaroo_driver.cpp
)
ament_target_dependencies(kangaroo_driver_node rclcpp std_msgs trajectory_msgs sensor_msgs)

target_link_libraries(kangaroo_driver_node 
    kang_lib
)

install (TARGETS
  kangaroo_driver_node
  DESTINATION lib/${PROJECT_NAME}  
)   

ament_package()
