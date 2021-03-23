cmake_minimum_required(VERSION 2.8.3)
project(adapted_move_base)

#add_compile_options(-std=c++11)
#add_definitions(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
		tf
		global_rrt_planner
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(adapted_move_base
  src/move_base.cpp
)
target_link_libraries(adapted_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(adapted_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(adapted_move_base_node
  src/move_base_node.cpp
)
add_dependencies(adapted_move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(adapted_move_base_node adapted_move_base)
set_target_properties(adapted_move_base_node PROPERTIES OUTPUT_NAME adapted_move_base_node)


