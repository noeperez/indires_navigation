# indires_navigation

THIS PACKAGE IS NOT LONGER MAINTAINED!

ROS metapackage for ground robot 3D navigation and exploration developed for the European Project INDIRES (http://indires.eu/).
Further details can be found in:

* N. Pérez-Higueras, A. Jardón, A.J. Rodríguez, C. Balaguer. 3D Exploration and Navigation with Optimal-RRT Planners for Ground Robots in Indoor Incidents. Sensors 2020, 20(1), 220. https://doi.org/10.3390/s20010220 

![alt text](https://github.com/noeperez/indires_navigation/blob/master/indires_navigation/images/rrt_planning.png)

This metapackage contains:

* *rrt_planners*:  
C++ Library and ROS wrapper for path planning in 3D. It contains the following planners:

    - *Simple RRTStar* [1] : RRT* planner in x,y,z coordinates without reasoning about kinodynamic constraints.
    - *RRT* [2] : kinodynamic RRT planner in x,y,z,yaw coordinates.
    - *Quick RRTStar*  [3] : Quick-RRT* planner in x,y,z coordinates without reasoning about kinodynamic constraints.

* *navigation_features_3d*:  
Package for calculation of sample validity and feature functions employed by the cost functions of the RRT planners for robot path planning and exploration.

* *pcl_filters*:  
ROS package to apply different filters to pointclouds. It makes use of PCL 1.9.

* *global_rrt_planner*:   
ROS plugin that allows to employ the RRT planners as global planner in the move_base architecture for navigation under ROS.

* *local_3d_planner*:   
A local controller in 3D to follow a given global path. It follows the ROS BaseLocalPlanner plugin (Kinetic Distro) that allows to employ the controller as local planner in the move_base architecture for navigation under ROS.

* *adapted_move_base*:   
This is a modified version of the original move_base package of ROS Kinectic distro. This modification allows to use other global or local planners  (following the ROS move_base premises) that do not use the standard ROS Costamps for planning. Two new boolean parameters are added to indicate the use of the global and/or local costmap (use_global_costmap2d and use_local_costmap2d).

* *indires_macro_actions*:
A set of navigation macro-actions have been implemented by using the *actionlib* library of ROS. This way, the navigation system is employed to perform different actions as reaching an indicated goal, perform an autonomous exploration, or teleoperate the robot.

* *control_state_machine*:
Python scripts that contains the finite state machine for the interaction between the actionlib macro-actions defined in indires_macro_actions. 


The following image shows an example of the ros node graph of a simulation of the exploration and navigation system also using Gazebo as robot and environment simulator (nodes from indires_macro_actions and control_state_machine are not shown), and ethzasl_icp_mapper as SLAM algorithm [4] .

![alt text](https://github.com/noeperez/indires_navigation/blob/master/indires_navigation/images/rosgraph.png)



## Configuration

The system does not include any SLAM or mapping algorithm. It relies on any external mapping algorithm which must be publishing an online map in the form of point cloud. 


The files for simulation in Gazebo of the robot and the environments are not included. 


* *What you need to configure*:

	- Chose a SLAM or mapping algorithm that provides an online map in the form of pointcloud. The system listens to it in the topic /point_map [sensor_msgs/PointCloud2]. This can be changed through the parameter "pointcloud" indicated in the launch file adapted_move_base.launch. 
	- Your robot (real or simulated) must be publishing a "reliable" odometry topic. The name of the odometry topic need to be indicated in different places:
		- Parameters "robot_odom_topic" and "odometry_topic" of the configuration file navigation_params.yaml of the package adapted_move_base.
		- Parameter "robot_topic" in the launch file pcl_filters.launch.
		- Parameter "odom_topic" in the launch file indires_macro_actions.launch.
	- You need a proper TF tree of your system. The robot frame (usually /base_link) and the frame of the sensor that is publishing the pointcloud (a rgb-d camera for instance) must be indicated in the files navigation_params.yaml and pcl_filters.launch. 
	- The name of your topic for robot control commands [geometry_msgs/Twist] must be indicated in adapted_move_base.launch.



## Functioning

After configuring your system, your simulation (or real robot) with the mapping algorithm must be launched. Then, you can try the navigation and exploration system by launching three launch files:

* Navigation and exploration system:
> roslaunch adapted_move_base adapted_move_base.launch
* Navigation macro-actions: 
> roslaunch indires_macro_actions indires_macro_actions.launch
* Finite State Machine that controls the macro-actions and a simple command-line program for testing:
> roslaunch control_state_machine control_tester.launch 

## Version

This package has been tested under ROS MELODIC distribution.

Last modifications:

* Removing dependency on FLANN library for NearestNeiborghs search. It has been replaced by a custom kdtree.

* Update to ROS Melodic distro:

	- Dependency on dedicated costmap_2d package has been removed.
	- Adapted_move_base has been updated to be used in Melodic.
	- Tf1 replaced by tf2.


[1] Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. The International Journal of Robotics Research, 30(7), 846–894. https://doi.org/10.1177/0278364911406761

[2] Lavalle, S. M. (1998). Rapidly-Exploring Random Trees: A New Tool for Path Planning. In (Vol. 129). https://doi.org/10.1.1.35.1853

[3] In-Bae Jeong, Seung-Jae Lee, Jong-Hwan Kim (2019)- Quick-RRT*: Triangular inequality-based implementation of RRT* with improved initial solution and convergence rate, Expert Systems with Applications, Volume 123, 2019, Pages 82-90, ISSN 0957-4174, https://doi.org/10.1016/j.eswa.2019.01.032.

[4] F. Pomerleau F., Colas F., Siegwart R, and Magnenat S. (2013) Comparing ICP variants on real-world data sets. Autonomous Robots, 34(3), pages 133-148.


