cmake_minimum_required(VERSION 3.8) 
project(staubli_tx2_60l_position_commander)  

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")   
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 

set(CMAKE_CXX_STANDARD 17)  

# ---- Depedencies ---- 
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)  
find_package(controller_manager_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ---- Controller Node ---- 
add_executable(position_command
  src/position_command.cpp) 

ament_target_dependencies(position_command 
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs controller_manager_msgs trajectory_msgs)

# ---- exe ---- 
install(TARGETS
  position_command
  DESTINATION lib/${PROJECT_NAME} )  
  
# ---- Ressources ---- 
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME} )  

ament_package()