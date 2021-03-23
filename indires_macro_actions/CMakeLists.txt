cmake_minimum_required(VERSION 2.8.3)
project(indires_macro_actions)

#add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib
  actionlib_msgs
  cmake_modules
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosconsole
  roscpp
  std_msgs
  angles
  tf
  tf2_ros
  adapted_move_base
  #dynamic_reconfigure
)


find_package(Boost REQUIRED COMPONENTS system)

 add_action_files(
   	DIRECTORY action
   	FILES 
 	NavigateWaypoint.action
	NavigateHome.action
	Exploration.action
    Teleoperation.action
 )


generate_messages(
   	DEPENDENCIES
   	std_msgs
	geometry_msgs
	move_base_msgs
	nav_msgs
	actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES indires_macro_actions
  CATKIN_DEPENDS actionlib actionlib_msgs cmake_modules geometry_msgs move_base_msgs rosconsole roscpp std_msgs angles tf tf2_ros adapted_move_base
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(indires_macro_actions 
	src/Indires_macro_actions.cpp
	src/Indires_macro_actions_node.cpp)


add_dependencies(indires_macro_actions indires_macro_actions_gencpp)
add_dependencies(indires_macro_actions geometry_msgs_gencpp)
add_dependencies(indires_macro_actions nav_msgs_gencpp)
add_dependencies(indires_macro_actions move_base_msgs_gencpp)
add_dependencies(indires_macro_actions actionlib_msgs_gencpp)
add_dependencies(indires_macro_actions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(indires_macro_actions
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


