cmake_minimum_required(VERSION 2.8.3)
project(rrt_planners)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set (CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED
    COMPONENTS
		angles
        cmake_modules
		geometry_msgs
  		nav_msgs
		#pcl_conversions
  		#pcl_ros
        roscpp
		rosconsole
        tf
  		std_msgs
  		visualization_msgs
  		#upo_msgs
		navigation_features_3d
		#dynamic_reconfigure
		message_generation
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED
#    COMPONENTS
#        thread
#)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
	#${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/RRTRosWrapper.cfg
#)


## Generate messages in the 'msg' folder
#add_message_files(
#    DIRECTORY msg
#    FILES
#    Position2DInt.msg
#	PointCost.msg
#	FeaturesWeights.msg
# )


add_service_files(
    FILES
#   PointCosts.srv
	MakePlan.srv
#	MakePlanCostmap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs nav_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES 
		rrt_planners
		rrt_planners_ros
    CATKIN_DEPENDS
        angles
        cmake_modules
		geometry_msgs
  		nav_msgs
		#pcl_conversions
		#pcl_ros
        roscpp
		rosconsole
        tf
  		std_msgs
  		visualization_msgs
  		#upo_msgs
		navigation_features_3d
		#gmm_sampling
		#dynamic_reconfigure
		message_runtime
	#DEPENDS system_lib
)


# upo_rrt_planners (ROS independent)
add_library(rrt_planners
	src/planners/Planner.cpp
	src/planners/simple/SimpleRRT.cpp
	src/planners/simple/SimpleRRTstar.cpp
	src/planners/control/Rrt.cpp
	src/planners/control/RRTstar.cpp
	src/planners/control/HalfRRTstar.cpp
	src/Action.cpp
	src/State.cpp
	src/Node.cpp
	src/StateSpace.cpp
	src/RandomNumbers.cpp
	src/steering/Steering.cpp
)

add_dependencies(rrt_planners rrt_planners_gencpp)


target_link_libraries(rrt_planners
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	flann
    )



# upo_rrt_planners (ROS wrapper)
add_library(rrt_planners_ros
	src/ros/ValidityChecker3D.cpp
	src/ros/RRT_ros_wrapper.cpp
)

add_dependencies(rrt_planners_ros rrt_planners_gencpp)
add_dependencies(rrt_planners_ros rrt_planners_ros_gencpp)
add_dependencies(rrt_planners_ros upo_msgs_gencpp)
add_dependencies(rrt_planners_ros geometry_msgs_gencpp)
add_dependencies(rrt_planners_ros nav_msgs_gencpp)
add_dependencies(rrt_planners_ros navigation_features_3d_gencpp)
#add_dependencies(upo_rrt_planners_ros gmm_sampling_gencpp)

target_link_libraries(rrt_planners_ros
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	#${PCL_LIBRARIES}
	nav_features3d
	#gmm_sampling
	rrt_planners
    )


# upo_rrt_planners (ROS wrapper with use an external costmap)
add_executable(rrt_ros_wrapper_node
	src/ros/ValidityChecker3D.cpp
	src/ros/RRT_ros_wrapper.cpp
	src/ros/ros_wrapper_node.cpp
)

add_dependencies(rrt_ros_wrapper_node rrt_planners_gencpp)
#add_dependencies(rrt_ros_wrapper_node upo_msgs_gencpp)
add_dependencies(rrt_ros_wrapper_node geometry_msgs_gencpp)
add_dependencies(rrt_ros_wrapper_node nav_msgs_gencpp)

target_link_libraries(rrt_ros_wrapper_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	rrt_planners
)


# upo_rrt_planners2 (ROS wrapper with use an external costmap)
#add_executable(rrt_ros_wrapper2_node
#	src/ros/ValidityChecker2.cpp
#	src/ros/RRT_ros_wrapper2.cpp
#	src/ros/ros_wrapper2_node.cpp
#)

#add_dependencies(rrt_ros_wrapper2_node upo_rrt_planners_gencpp)
#add_dependencies(rrt_ros_wrapper2_node upo_msgs_gencpp)
#add_dependencies(rrt_ros_wrapper2_node geometry_msgs_gencpp)
#add_dependencies(rrt_ros_wrapper2_node nav_msgs_gencpp)

#target_link_libraries(rrt_ros_wrapper2_node
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${Eigen_LIBRARIES}
#	upo_rrt_planners
#)



# upo_rrt_planners3_ros (ROS wrapper)
#add_library(upo_rrt_planners3_ros
#	src/ros/capture.cpp
#	src/ros/ValidityChecker3.cpp
#	src/ros/RRT_ros_wrapper3.cpp
#)

#add_dependencies(upo_rrt_planners3_ros upo_rrt_planners_gencpp)
#add_dependencies(upo_rrt_planners3_ros upo_rrt_planners3_ros_gencpp)
#add_dependencies(upo_rrt_planners3_ros upo_msgs_gencpp)
#add_dependencies(upo_rrt_planners3_ros geometry_msgs_gencpp)
#add_dependencies(upo_rrt_planners3_ros nav_msgs_gencpp)

#target_link_libraries(upo_rrt_planners3_ros
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${Eigen_LIBRARIES}
#	upo_rrt_planners
#    )




# upo_rrt_planners2 (ROS wrapper with use an external costmap)
#add_executable(rrt_ros_wrapper3_node
#	src/ros/capture.cpp
#	src/ros/ValidityChecker3.cpp
#	src/ros/RRT_ros_wrapper3.cpp
#	src/ros/ros_wrapper3_node.cpp
#)

#add_dependencies(rrt_ros_wrapper3_node upo_rrt_planners_gencpp)
#add_dependencies(rrt_ros_wrapper3_node upo_msgs_gencpp)
#add_dependencies(rrt_ros_wrapper3_node geometry_msgs_gencpp)
#add_dependencies(rrt_ros_wrapper3_node nav_msgs_gencpp)

#target_link_libraries(rrt_ros_wrapper3_node
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${Eigen_LIBRARIES}
#	upo_rrt_planners
#    )







install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )


install(
    TARGETS
        rrt_planners
		rrt_planners_ros
		#upo_rrt_planners3_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
