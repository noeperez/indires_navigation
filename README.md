# indires_navigation
ROS metapackage for ground robot navigation and exploration developed for the European Project INDIRES (http://indires.eu/)

This metapackage contains:

* *rrt_planners*:  
C++ Library and ROS wrapper for path planning in 3D. It contains the following planners:

    - *Simple RRTStar*: RRT* planner in x,y,z coordinates without reasoning about kinodynamic constraints.
    - *RRT*: kinodynamic RRT planner in x,y,z,yaw coordinates.

* *navigation_features_3d*:  
Package for calculation of sample validity and feature functions employed by the cost function of the RRT planners for robot path planning.

* *pcl_filters*:  
ROS package to apply different filters to pointclouds. It makes use of PCL 1.9.

* *rrt_planner_plugin*:
ROS plugin that allows to employ the RRT planners as global planner in the move_base architecture for navigation of ROS.


#### TODO
- [ ] Add a local controller for robot motion. 
- [ ] Integrate the path planning with the local controller and manage the navigation.

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
