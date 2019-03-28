# navigation_features_3d 
Package that calculates the feature functions employed by a cost function for robot navigation.

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
	
* **Weight set parameters for exploration**
The cost function for selecting the most promising frontier for exploration (leaf node of the RRT* tree) is based on three features: the number of points detected around the leaf, the proximity between the leaf and the closest point of the trajectory of the robot so far, and the cost of the RRT* path to reach the leaf.  

	- wexp1. Normalized weight for the feature of number of point around the leaf.
	- wexp2. Normalized weight for the feature of of proximity between the leaf and the traveled path of the robot.
	- wexp3. Normalized weight for the feature of RRT* path cost.

* **Other parameters**

	- pointcloud_topic: name of the topic where the point cloud used for analysis of the traversability of the terrain is being published.
	- robot_circuns_radius: radius, in meters, of the sphere that cicumscribe the robot shape.
	- max_pitch_inclination: maximum pitch angle allowed, in radians, to determine whether a sampled area is valid.
	- max_roll_inclination: maximum roll angle allowed, in radians, to determine whether a sampled area is valid.
	- max_roughness: maximum value of roughness allowed for a valid sampled area.
	- min_points_allowed: minimum number of points that a sampled area must contain to be considered valid.
	- robot_base_frame: TF frame of the robot base. Usually "base_link".
	- robot_odom_frame: TF frame of the odometry. Usually "odom".
	- robot_odom_topic: topic where the odometry is being published.




## Dependences

* **PCL** is required in its version 1.9.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
