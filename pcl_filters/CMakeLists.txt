cmake_minimum_required(VERSION 2.8.3)
project(pcl_filters)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
#add_definitions("-std=c++11")
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf2_ros
  message_generation
)

find_package(PCL REQUIRED)

 add_service_files(
   FILES
   GetFilteredPC.srv
   ChangeCropboxSize.srv
 )

 generate_messages(
   DEPENDENCIES
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_filters
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs tf tf2_ros
  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_filters_node src/pcl_filters.cpp )
add_dependencies(pcl_filters_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (pcl_filters_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pcl_costmap_node src/pcl_costmap.cpp )
add_dependencies(pcl_costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (pcl_costmap_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(alternate_pc_node src/alternate_pc.cpp )
add_dependencies(alternate_pc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (alternate_pc_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pc2_to_ply src/pc2_to_ply.cpp )
add_dependencies(pc2_to_ply ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (pc2_to_ply ${PCL_LIBRARIES} ${catkin_LIBRARIES})
