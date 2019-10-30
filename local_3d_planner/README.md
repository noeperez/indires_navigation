# local_3d_planner 
A 3D controller based on pure pursuit and DWA path tracking has been extended to command velocities to the differential robot so as to follow the path smoothly.
This path tracker has been extended to perform a collision detection checking based on pointcloud as sensory input. If the forward projection of the robot movement given by the control law is detected as a possible collision, a valid command is tried to be found by sampling variations of the given linear and angular velocities. 
The controller follows the structure of the standard base local planner of ROS. However it does not use the local costmap of ROS and the collision detection is performed based on subscriptions to range source (sensor_msgs/PointCloud2 ROS message type). 

## Parameters

* **Robot Configuration Parameters**
	- max_trans_acc. Maximum acceleration in translation (m/s^2).
  	- max_rot_acc. Maximum acceleration in rotation (rad/s^2).
  	- max_trans_vel. Maximum linear velocity (m/s).
  	- min_trans_vel. Minimum linear velocity (m/s).
  	- max_rot_vel. Maximum angular velocity (rad/s).
  	- min_rot_vel. Minimum angular velocity (rad/s).
  	- min_in_place_rot_vel. Angular velocity of rotations in place (rad/s).
	- robot_radius. Radius of the circunscribed sphere of the robot.

* **Goal Tolerance Parameters**
	- yaw_goal_tolerance. Tolerance in angular distance (rad) to consider that the goal has been reached.
	- xyz_goal_tolerance. Tolerance in euclidean distance (m) to consider that the goal has been reached.
	- wp_tolerance. Distance (m) from the robot to look for the point of the global plan to follow.
  
* **Forward Simulation Parameters**
	- sim_time. Time (seconds) to expand the robot movement and check for collisions. (default: 0.5).
	- sim_granularity. Resolution in meters to split the expanded trayectory and check for collisions (Default: 0.025).
	- angular_sim_granularity. Resolution in radians to split the expanded angular movement and check for collisions (Default: 0.025).
	- controller_freq. Frequency of execution in Hz (Default: 15.0).
	- local_area_radius. Radius around the robot to be considered in the pointcloud for local planning purposes. 

* **Collision detection Parameters**
	- features_name. String that would be used as a prefix to look for the parameters required by the navigation_features_3d in the parameter server since it is used here for traversability analysis. See parameters of the navigation_features_3d package.
	- odometry_topic. name of the topic that is publishing the odometry of the vehicle.
	- base_frame. name of the base TF frame of the vehicle. Default: base_link.
	- global_frame. TF frame in which the local planner works. Default: odom.

## Topics published

* **global_plan**. Path for visualization in RViz. ROS Message type: nav_msgs/Path
* **local_plan**. Path for visualization in RViz. ROS Message type: nav_msgs/Path


## Dependences

* **navigation_features_3d**
* **costmap_2d**


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
