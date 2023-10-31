cmake_minimum_required(VERSION 3.0.2)
project(grabbybot_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  hardware_interface
)

add_service_files(
  FILES
  AnglesConverter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
  controller_manager
  hardware_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(angles_converter_node src/angles_converter.cpp)
add_dependencies(angles_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angles_converter_node  ${catkin_LIBRARIES})

add_executable(grabbybot_interface_node src/grabbybot_interface.cpp)
add_dependencies(grabbybot_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grabbybot_interface_node  ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/angles_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)