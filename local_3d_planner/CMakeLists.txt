cmake_minimum_required(VERSION 2.8.3)
project(local_3d_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
    COMPONENTS
    cmake_modules
    roscpp
    tf2_ros
    #message_generation
    dynamic_reconfigure
    nav_core
    pcl_conversions
    rostest
    costmap_2d
    pluginlib
    angles
    navigation_features_3d
)

find_package(Boost REQUIRED
    COMPONENTS
    thread
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
#add_definitions(${EIGEN_DEFINITIONS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        local_3d_planner
        local_3d_planner_ros
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        #message_generation
        tf2_ros
        pluginlib
        costmap_2d
        nav_core
        angles
        navigation_features_3d
)



add_library(local_3d_planner
	src/goal_functions.cpp
	src/odometry_helper_ros.cpp
	src/collision_detection.cpp
 )
add_dependencies(local_3d_planner local_3d_planner_gencpp)
add_dependencies(local_3d_planner nav_msgs_gencpp)
target_link_libraries(local_3d_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    #${Eigen_LIBRARIES}
    Eigen3::Eigen
    )

add_library(local_3d_planner_ros
	src/local_3d_planner.cpp
	src/local_3d_planner_ros.cpp)
add_dependencies(local_3d_planner_ros nav_msgs_gencpp)
target_link_libraries(local_3d_planner_ros
     local_3d_planner)


install(TARGETS
            local_3d_planner
            local_3d_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES lp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


