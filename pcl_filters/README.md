# pcl_filters 
ROS package to apply different filters to point clouds. It is used to provide two filtered point clouds to the rrt_planner (see launch/pcl_filters.launch). One is employed as sampling space, and the other one is used for traversability analysis of the 3D terrain.

## General Parameters

* *pointcloud_topic*: ROS topic where the point cloud to be filteres is being published. 

* *output_topic*: name of the ROS to topic where to publish the output point cloud filtered.

* *odom_frame*: TF frame where the odometry is being published.

* *sensor_frame*: TF frame of the sensor that is publishing the input point cloud.

* *local_grid_radius*: radius in meters of the local area considered around the robot where the global point cloud must be cropped.

* *filter_ceiling*: boolean to indicate whether the ceiling and the walls should be removed (partially) from the point cloud. 

* *show_normal_arrows*: boolean to indicate if the normal vectors of the surface should be published to be visualized in RViz.

* *pith_max_inclination*: surfaces that exceed this maximum inclination of the pitch will be removed. (Only applicable if filter_ceiling is true).

* *roll_max_inclination*: surfaces that exceed this maximum inclination of the roll will be removed. (Only applicable if filter_ceiling is true).

* *max_roughness*: surfaces that exceed this maximum terrain roughness will be removed. (Only applicable if filter_ceiling is true).


## PCL filters Parameters

These are provided through a yaml file for sake of clarity (see examples launch/rrt_filter_params.yaml and launch/3dfeatures_params.yaml).

Possible filters:

* **StatisticalOutlierFilter**

* **PassThroughFilter**

* **CropBoxFilter**

* **VoxelGridFilter**

* **VoxelGridCovarianceFilter**




## Dependences

* **PCL** is required in its version 1.9.1.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
