# navigation_features_3d 
Package that calculates the feature functions employed by a cost function for robot navigation.

## Parameters

* **Feature set parameters**

	- upo_featureset. Two sets of features are available. 
	
		Value '1' for the set of 3 features: pitch inclination, roll inclination, and roughness. 
		Cost function C(x) = w1 f1(x) + w2 f2(x) + w3 f3(x) 
		
		Value '2' for the set of 7 features: pitch inclination, roll inclination, roughness, distance between the mean of a set of points the sample evaluated, standard deviation of the set of points, inversed num of points (1-num_points) of the set, and distance to the given goal (not applied in case of exploration). All these features are normalized, [0,1]. 
		Cost function C(x) = w1 f1(x) + w2 f2(x) + w3 f3(x) + w4 f4(x) + w5 f5(x) + w6 f6(x) + w7 f7(x)
		
	- w1. Normalized weight for the feature of pitch inclination.
	- w2. Normalized weight for the feature of of roll inclination.
	- w3. Normalized weight for the feature of roughness of the terrain area.
	- w4. Normalized weight for the feature of distance between the mean of the set of points and the RRT sample.
	- w5. Normalized weight for the feature of standard deviation of the set of points.
	- w6. Normalized weight for the feature of number of points of the set.
	- w7. Normalized weight for the feature of distance to the goal. 



* **Parameters for obstacle distance feature**

	- use_laser_projection. Boolean to indicate whether to use a projection of the laser readings onto the map and a distance transform to calculate the feature of the distance to closest obstacle. If not, the ROS costmaps are used.
	- pc_topic. Name of the topic to subscribe to with the point cloud of the sensor readings for projection onto the map.
	- pc_type. Indicate the type of ROS point cloud message. Value '1' for sensor_msgs/PointCloud, Value '2' for sensor_msgs/PointCloud2.



## Dependences

* **PCL** is required in its version 1.9.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
