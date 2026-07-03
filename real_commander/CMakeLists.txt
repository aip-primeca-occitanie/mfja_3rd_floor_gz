cmake_minimum_required(VERSION 3.8) 
project(real_commander)  


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")   
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 

set(CMAKE_CXX_STANDARD 17)  

# ---- Depedencies ---- 
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)  
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(staubli_tx2_60l_controller REQUIRED)
find_library(NIDAQMX_LIB nidaqmx)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---- Controller Node ---- 
add_executable(position_commander src/position_commander.cpp) 
ament_target_dependencies(position_commander rclcpp sensor_msgs trajectory_msgs)

add_executable(target_force_publisher src/target_force_publisher.cpp)
ament_target_dependencies(target_force_publisher rclcpp std_msgs)

add_executable(force_pid_controller src/force_pid_controller.cpp)
ament_target_dependencies(force_pid_controller rclcpp geometry_msgs std_msgs staubli_tx2_60l_controller)

add_executable(sinus_mvt src/sin_mvt.cpp)
ament_target_dependencies(sinus_mvt rclcpp geometry_msgs)

# ---- Other Nodes ---- 
add_executable(cartesian_converter src/cartesian_converter.cpp)
ament_target_dependencies(cartesian_converter rclcpp sensor_msgs geometry_msgs tf2_eigen ament_index_cpp)
target_link_libraries(cartesian_converter pinocchio::pinocchio)

add_executable(cartesian_publisher src/cartesian_publisher.cpp)
ament_target_dependencies(cartesian_publisher rclcpp sensor_msgs geometry_msgs trajectory_msgs tf2_eigen ament_index_cpp)
target_link_libraries(cartesian_publisher pinocchio::pinocchio)

add_executable(force_simulation src/force_simulation.cpp)
ament_target_dependencies(force_simulation rclcpp geometry_msgs std_msgs)

add_executable(sensor src/daq_publisher.cpp)
ament_target_dependencies(sensor rclcpp std_msgs)
target_link_libraries(sensor ${NIDAQMX_LIB})

add_executable(sensor_filter src/force_filter.cpp)
ament_target_dependencies(sensor_filter rclcpp  std_msgs)

# ---- exe ---- 
install(TARGETS
  position_commander
  cartesian_converter
  cartesian_publisher
  force_simulation
  force_pid_controller
  target_force_publisher
  sensor
  sensor_filter
  sinus_mvt
  DESTINATION lib/${PROJECT_NAME} )

install(PROGRAMS
  src/realtime_plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)
  
# ---- Ressources ---- 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME} )

ament_package()