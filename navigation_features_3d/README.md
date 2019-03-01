# navigation_features 
Package that calculates the feature functions employed by a cost function for robot navigation.

## Parameters

* **Feature set parameters**

	- upo_featureset. Two sets of features are available. 
		Value '0' for the set of 3 features: goal distance, obstacle distance and Gaussian function over people.
		Value '1' for the set of 5 features: goal distance, obstacle distance, Gaussian function in the person front, Gaussian function in the person back and Gaussian function in the person right side.
	- w1. Normalized weight for the feature of goal distance.
	- w2. Normalized weight for the feature of distance to the closest obstacle.
	- w3. If 'upo_featureset' is '0', this value is the normalized weight for the feature of the maximum value of the Gaussian functions deployed over the people. If 'upo_featureset' is '1', the weight corresponds to the feature of the Gaussian function in the front of the people.
	- w4. Normalized weight for the feature of the Gaussian function in the back of the people. Only used if 'upo_featureset' is '1'.
	- w5. Normalized weight for the  feature of the Gaussian function in the right side of the people. Only used if 'upo_featureset' is '1'.


* **Gaussian functions parameters**

	- stddev_person_front. Value of the standard deviation in the X axis of the Gaussian function in the front of the person.
	- stddev_person_aside. Value of the standard deviation in the Y axis of the Gaussian function in the front of the person. This value is also used to define the standard deviation in X and Y axis of the Gaussian function in the back of the person
	- stddev_person_right. Value of the standard deviation in the X axis of the Gausssian function in the right side of the person. 


* **Parameters for obstacle distance feature**

	- use_laser_projection. Boolean to indicate whether to use a projection of the laser readings onto the map and a distance transform to calculate the feature of the distance to closest obstacle. If not, the ROS costmaps are used.
	- pc_topic. Name of the topic to subscribe to with the point cloud of the sensor readings for projection onto the map.
	- pc_type. Indicate the type of ROS point cloud message. Value '1' for sensor_msgs/PointCloud, Value '2' for sensor_msgs/PointCloud2.



## Dependences

* **upo_msgs** is required.
* **Open_CV** is also needed.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
