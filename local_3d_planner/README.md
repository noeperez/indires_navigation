# local_3d_planner 
A 3D controller based on pure pursuit and DWA path tracking has been extended to command velocities to the differential robot so as to follow the path smoothly.
This path tracker has been extended to perform a collision detection checking based on pointcloud as primary sensory input. As optional input, a 2D laser range finder [ROS message type: sensor_msgs/LaserScan] is allowed as well as different sonar range finders [ROS message type: sensor_msgs/Range] publishing in several topics.
If the forward projection of the robot movement given by the control law is detected as a possible collision, a valid command is tried to be found by sampling variations of the given linear and angular velocities. 
The controller follows the structure of the standard base local planner of ROS. However it does not use the local costmap of ROS and the collision detection is performed based on subscriptions to range source (sensor_msgs/PointCloud2 ROS message type). 

## Parameters

An example of the parameters can be observed in the file navigation_params.yaml inside the package adapted_move_base.

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

* **Primary Collision detection Parameters**
	- features_name. String that would be used as a prefix to look for the parameters required by the navigation_features_3d in the parameter server since it is used here for traversability analysis. See parameters of the navigation_features_3d package.
	- odometry_topic. name of the topic that is publishing the odometry of the vehicle.
	- base_frame. name of the base TF frame of the vehicle. Default: base_link.
	- global_frame. TF frame in which the local planner works. Default: odom.

* **Optional Collision detection Parameters**
	- use_laser. Boolean to indicate whether a laser scan is employed for collision detection.
	- laser_topic. If the parameters "use_laser" is True, the name of the topic publishing the laser scan must be indicated here.
	- use_range. Boolean to indicate whether a set of sensors publishing sensors_msgs/Range messages are used. If true, the name of the topics where the ranges are published must be indicated through the parameters "range_topic_X", in which "X" is a positive integer value. For example, if we have two sonars publishing in the topics "sonar_range_up" and "sonar_range_down", we must indicate:
	> range_topic_0:    sonar_range_up
	> range_topic_1:    sonar_range_down



## Topics published

* **global_plan**. Path for visualization in RViz. ROS Message type: nav_msgs/Path
* **local_plan**. Path for visualization in RViz. ROS Message type: nav_msgs/Path


## Dependences

* **navigation_features_3d**
* **costmap_2d**


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
