cmake_minimum_required(VERSION 3.8) 
project(staubli_tx2_60l_controller)  


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
add_executable(position_commander
  src/position_commander.cpp) 

ament_target_dependencies(position_commander 
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs controller_manager_msgs trajectory_msgs)

add_executable(staubli_controller
  src/staubli_controller.cpp) 

ament_target_dependencies(staubli_controller 
  rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs controller_manager_msgs)

target_include_directories(staubli_controller PUBLIC   
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   
  $<INSTALL_INTERFACE:include> ) 

# ---- exe ---- 
install(TARGETS
  position_commander
  DESTINATION lib/${PROJECT_NAME} )  

install(TARGETS
  staubli_controller
  DESTINATION lib/${PROJECT_NAME} )  

# ---- Ressources ---- 
install(DIRECTORY
  launch config include
  DESTINATION share/${PROJECT_NAME} )  

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
ament_package()