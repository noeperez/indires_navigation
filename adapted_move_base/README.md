# adapted_move_base
This is a modified version of the original move_base package of ROS. This modification allows to use other global or local planners (following the ROS move_base premises) that do not use the standard ROS Costamps for planning. See the documentation of the original package (http://wiki.ros.org/move_base?distro=kinetic).

## Dependences

* The same as the standard move_base package or ROS.


## Parameters

* **use_global_costmap2d**. Boolean to indicate whether the global planner is going to use a ROS Costmap2d or not.
* **use_local_costmap2d**. Boolean to indicate whether the local planner is going to use a ROS Costmap2d or not.
* The rest of regular move_base parameters. See http://wiki.ros.org/move_base?distro=kinetic


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
