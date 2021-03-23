cmake_minimum_required(VERSION 2.8.3)
project(navigation_features_3d)

## Add support for C++11, supported in ROS Kinetic and newer
#add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
  pcl_msgs
  roscpp
  roslib
  rospy
  std_msgs
  tf
  tf2_ros
  pcl_filters
  nav_msgs
  visualization_msgs
  #message_generation
  #dynamic_reconfigure
)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
	nav_features3d
  CATKIN_DEPENDS 
	cmake_modules  
	geometry_msgs 
	laser_geometry 
	pcl_conversions 
	pcl_ros 
	pcl_msgs
	roscpp 
	roslib 
	rospy 
	std_msgs 
	tf 
        tf2_ros
	pcl_filters 
	nav_msgs 
	visualization_msgs
	#message_runtime
	#dynamic_reconfigure
  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

#add_definitions(${EIGEN_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## Declare a C++ library
 add_library(nav_features3d
   src/nav_features3d.cpp
 )


 add_dependencies(nav_features3d ${navigation_features_3d_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(nav_features3d navigation_features_3d_gencpp)
 add_dependencies(nav_features3d navigation_features_3d_gencfg)


 target_link_libraries(nav_features3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    #${Eigen_LIBRARIES}
    Eigen3::Eigen
 )



 
 install(TARGETS nav_features3d
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


