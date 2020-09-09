# rrt_planners
C++ library of some RRT planners and API for its use with ROS

This is a catkin package of ROS that contains two libraries:

* *upo_rrt_planners*: C++ library that contains the following RRT planners:

	- *simple RRTstar*: RRT* planner in x,y,z coordinates without reasoning about kinodynamic constraints.
	- *RRT*: RRT planner with kinodynamic constrains.
	- *Quick-RRTStar* [1] : planner in x,y,z coordinates without reasoning about kinodynamic constraints.

* *upo_rrt_planners_ros*: C++ library that wraps the previous library in order to be used in ROS. 


## Dependences

* Package **navigation_features_3d**.

## Functioning

The planner works through a ROS service called "/RRT_ros_wrapper/makeRRTPlan". The message type is "rrt_planners/MakePlan" which has the parameter "goal" [geometry_msgs/PoseStamped]. So, a goal position can be passed to the planner to plan a path to it in the given planning time. The service returns a vector of the poses of the path. Note that the goal position must be inside the space size given to the planner.
If the frame_id of the given goal is empty, the planner directly switch to exploration mode, and computes the most promising goal to move according to pre-defined criteria.

This planner also can work as a global planner in the move_base architecture or ROS navigation. See the packages global_rrt_planner and adapted_move_base. 



## Parameters

* **rrt_planner_type**. Type of RRT planner to be used. Default: 2
	- 1 RRT. *x,y,z* state space (no dynamics).
	- 2 RRT*. *x,y,z* state space (no dynamics).
	- 3 Quick-RRT*. *x,y,z* state space (no dynamics).
* **rrt_solve_time**. Time in seconds that the RRT* planner is allowed to plan a path. Maximum time to find a path in the case of the RRT.
* **planning_frame**. TF frame in which the planner would be planning (usually "odom").
* **rrt_goal_bias**. probability bias to sample the goal.
* **rrt_max_insertion_dist**. Maximum distance (m) to insert a new node from the nearest node of the sample.
* **rrt_goal_xyz_tol**. Tolerance (m) to consider that the goal has been reached in the x,y space.
* **rrt_goal_th_tol**. Tolerance (radians) to consider that the goal has been reached in the angular space.
* **rrt_interpolate_path_dist**. Distance (m) between nodes to perform an interpolation of the resulting path. Use value 0 for no interpolation.

Only for RRT* planner:
* **rrtstar_use_k_nearest**. Boolean to indicate whether to use k-nearest or radius search to find the neighbors in the tree.
* **quick_rrt_depth**. Depth of the parents employed by Quick-RRT* algorithm (in case of being using this planner). 


Only for kinodynamic planners:
* **kino_time_step**. Time step (seconds) to propagate the movement of the robot.
* **kino_max_control_steps**. Maximum number of time steps to extend the movement of the robot.
* **kino_linear_acc**. Maximum linear acceleration of the robot (*m/s²*).
* **kino_angular_acc**. Maximum angular acceleration of the robot (*m/s²*).
* **Kino_steering_type**. Extend function to use. Two options.
	- 1 POSQ method.
	- 2 Modified version of the POSQ for more flexible turns.

State Space:
* **rrt_dimensions**. It can be 2 (*x,y* state space), or 3 (*x,y* and *yaw* or *z*).
* **dimensions_type**. Only applicable if *rrt_dimensions* = 3. Use value 1 for using *x,y* and *yaw* as dimensions or value 2, for using dimensions *x,y* and *z*.
* **distance_type**. Functions available to calculate the distance between nodes necessary to obtain the nearest neighbor. The options are:
	- 1 Distance calculated as *(x1-x2)+(y1-y2)*.
	- 2 Euclidean distance.
	- 3 If the yaw is in the state space. *w1 * ED + w2 * QD*, where ED is the euclidean distance and QD is the heading changes obtained as *(1 - |qi+1 - qi|)²*. 
* **motion_cost_type**. Function to calculate the cost of moving from one node to the next one. Usually option 2 is employed.
	- 1 Average of the costs. *(Cost1 + Cost2)/2*.
	- 2 Average of the costs and distance between nodes. *(Cost1 + Cost2)/2 * ||node1 - node2||*
	- 3 Average of the costs and exponential of the distance. *(Cost1 + Cost2)/2 * exp(||node1 - node2||)*
	- 4 Sum of the costs. *Cost1 + Cost2*.
* **rrt_size_x**. Size in meters of the *x* dimension. So, the range is [-x, x].
* **rrt_size_y**. Size in meters of the *y* dimension. So, the range is [-y, y].
* **rrt_size_z**. Size in meters of the *z* dimension. So, the range is [-z, z].
* **rrt_xyz_resolution**. Resolution of the *x,y,z* space.
* **robot_radius**. Radius of the inscribed circunference of the robot (meters).

Path smoothing:
* **path_smoothing**. Boolean to indicate whether to perform a smoothing of the RRT path obtained.
* **smoothing_samples**. Integer value to indicate the number of samples to include in the sliding window for path smoothing.

Visualization options:
* **visualize_rrt_tree**. Boolean to indicate whether to publish the tree or not as a marker in the topic *~/rrt_tree*. (NOTE: the points of the path obtained are published in the topic *~/rrt_path_points*).
* **show_rrt_statistics**. If it is enabled (boolean to true), some statistics about the RRT execution are shown on the screen.


Using a point cloud as sample space instead of uniform sampling of the space:
* **use_external_pc_as_samples**. If it is enabled, the 3D points of a point cloud would be used as sampling space for the planner.
* **pc_topic**. If 'use_external_pc_as_samples' is true, name of the ROS topic where the point cloud is being published (the type must be sensor_msgs/PointCloud2).
* **robot_base_frame**. TF frame of the robot base. Usually "base_link".
* **robot_odom_frame**. TF frame of the robot odometry. Usually "odom".
* **robot_pc_sensor_frame**. TF frame of the sensor that is publishing the point cloud.
* **features_name**. String that would be used as a prefix to look for the parameters required by the navigation_features_3d in the parameter server since it is used here for traversability analysis.    


## Topics published

* **rrt_tree**. Marker for visualization of the RRT tree in RViz (messsage type: visualization_msgs/Marker).
* **rrt_leaves**. Marker for visualization of the RRT leaves in RViz (messsage type: visualization_msgs/Marker).
* **rrt_goal**. Position of the goal given to the RRT planner (messsage type: geometry_msgs/PoseStamped). 
* **rrt_goal_marker**. Marker for visualization of the goal given to the RRT planner in RViz (messsage type: visualization_msgs/Marker).


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


The upo_rrt_planners library uses nearest neighbor data structures through the FLANN library. See: M. Muja and D.G. Lowe, "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration", in International Conference on Computer Vision Theory and Applications (VISAPP'09), 2009. http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN


[1] In-Bae Jeong, Seung-Jae Lee, Jong-Hwan Kim (2019)- Quick-RRT*: Triangular inequality-based implementation of RRT* with improved initial solution and convergence rate, Expert Systems with Applications, Volume 123, 2019, Pages 82-90, ISSN 0957-4174, https://doi.org/10.1016/j.eswa.2019.01.032.
