cmake_minimum_required(VERSION 3.5)
project(sensor_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_library(NIDAQMX_LIB nidaqmx)

add_executable(talker src/daq_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

target_link_libraries(talker ${NIDAQMX_LIB})

add_executable(listener src/force_moment_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()