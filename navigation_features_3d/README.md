# navigation_features_3d 
Package that calculates the feature functions employed by different cost functions for robot navigation and exploration.

## Parameters

* **Feature set parameters for navigation**

	- feature_set. Two sets of features are available: 
	
		Value '1' for the set of 3 features: pitch inclination, roll inclination, and roughness. 
		Cost function C(x) = w1 f1(x) + w2 f2(x) + w3 f3(x) 
		
		Value '2' for the set of 7 features: pitch inclination, roll inclination, roughness, distance between the mean of a set of points and the sample evaluated, standard deviation of the set of points, inversed num of points (1-num_points) of the set, and distance to the given goal (not applied in case of exploration). All these features are normalized, [0,1]. 
		Cost function C(x) = w1 f1(x) + w2 f2(x) + w3 f3(x) + w4 f4(x) + w5 f5(x) + w6 f6(x) + w7 f7(x)
		
	- w1. Normalized weight for the feature of pitch inclination.
	- w2. Normalized weight for the feature of of roll inclination.
	- w3. Normalized weight for the feature of roughness of the terrain area.
	- w4. Normalized weight for the feature of distance between the mean of the set of points and the RRT sample.
	- w5. Normalized weight for the feature of standard deviation of the set of points.
	- w6. Normalized weight for the feature of number of points of the set.
	- w7. Normalized weight for the feature of distance to the goal.

* **Other parameters for navigation**

	- pointcloud_topic: name of the topic where the point cloud used for analysis of the traversability of the terrain is being published.
	- robot_circuns_radius: radius, in meters, of the sphere that cicumscribe the robot shape.
	- max_pitch_inclination: maximum pitch angle allowed, in radians, to determine whether a sampled area is valid.
	- max_roll_inclination: maximum roll angle allowed, in radians, to determine whether a sampled area is valid.
	- max_roughness: maximum value of roughness allowed for a valid sampled area.
	- min_points_allowed: minimum number of points that a sampled area must contain to be considered valid.
	- robot_base_frame: TF frame of the robot base. Usually "base_link".
	- robot_odom_frame: TF frame of the odometry. Usually "odom".
	- robot_odom_topic: topic where the odometry is being published.

	
* **Weight set parameters for exploration**

	The cost function for selecting the most promising frontier for exploration (leaf node of the RRT* tree) is based on three features: the number of points and their distribution around the leaf, the proximity between the leaf and the closest point of the trajectory of the robot so far, and the cost of the RRT* path to reach the leaf. This frontier evaluation has been called Cost Function Exploration (CFE), and it is used when the parameters nf_exploration and bf_exploration are false. 

	- wexp1. Normalized weight for the feature of frontier cost. The frontier cost is computed as:

      frontier_cost(leaf) = 0.7 * points(leaf)/max_points + 0.3 * stddev(leaf)/max_stddev. 

      Where points(leaf) is the number of points in a sphere of radius 1.5 meters around the leaf point. stddev(leaf) is the standard deviation of the set of points in the previous sphere. max_points and max_stddev are saturation values chosen for normalization purposes. 
	- wexp2. Normalized weight for the feature of of proximity between the leaf and the traveled path of the robot.
	- wexp3. Normalized weight for the feature of RRT* path cost.

* **Other parameters for exploration**

	- exp_pc_service_name: name of the ROS service that must be called when the system requires a copy of the point cloud employed for exploration analysis. This service is provided by the node pcl_filters_node. See package pcl_filters.
	- nf_exploration: if true, enables the use of the Nearest Frontier Exploration (NFE) approach for frontier evaluation in exploration. This is an adaptation of the well-known Nearest Frontier approach
[1] to 3D point clouds. It is based on proximity criteria by selecting the frontier with the smallest Euclidean distance to the robot ignoring the existence of obstacles.
	- bf_exploration: if true, enables the use of the Biggest Frontier Exploration (BFE) approach for frontier evaluation in exploration. It is based on size criteria. The frontier with less information (bigger volume without points) is selected as the goal.
	- threshold_frontier: Value in range [0,1]. It determines whether a leaf of the tree is considered as a frontier or not based on a frontier cost for each leaf. If the cost is over the threshold_frontier value, the leaf is discarded as possible frontier. Default: 0.4.
	- adaptative_threshold: if true, enables the option of dinamically increasing the value of the frontier_threshold if no frontiers are detected and the planning size has reached its maximum value (max_planning_size parameter). Default: True.
	- min_planning_size: mimimum desired size of the boxed area employed in the navigation and exploration. Size in meters from the center of the box. So, the range for the x, y and x coordinates is [-min_planning_size, min_planning_size].
	- max_planning_size: maximum desired size of the boxed area employed in the navigation and exploration. The exploration system changes the planning size dinamically (in the given ranges) based on the number of frontier found and their frontier cost associated. 
	- cell_size_grid_exp: size in meters of the cells of the voxel grid in which the point cloud for exploration is stored. Default: 0.7

## Dependences

* **pcl_filters** ros package contained in the metapackage indires_navigation.
* **PCL** is required in its version 1.9.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.

[1] B. Yamauchi. A Frontier-Based Approach for Autonomous Exploration. in In Proceedings of the IEEE International Symposium on Computational Intelligence, Robotics and Automation, 1997, pp. 146–151.
