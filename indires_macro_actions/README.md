# indires_macro_actions. 
A set of navigation macro-actions have been implemented by using the *actionlib* of ROS. This way, the navigation system is employed to perform different actions as reaching a simple goal, perform an autonomous exploration, or teleoperate the robot. A launch file to test the macro-actions is inside the launch directory.


## Actions implemented

* **NavigateWaypoint**. This is the regular navigation action in which a goal point must be provided by the user.
* **NavigateHome**. This action drives the robot to the home position (start position).
* **Exploration**. This action initiates an autonomous exploration of the area. The exploration continues until the user cancels the action.
* **Teleoperation**. This action is intended to be triggered when the user wants to control the robot manually. Inmediately, the macro-action in execution (NavigateWaypoint, NavigateHome or Exploration) is canceled. The action ends a few seconds after receiving the last manual control command. 

* Note: See the package [control_state_machine](https://github.com/noeperez/indires_navigation/blob/master/control_state_machine) to see the relations and transitions between the different actions.


## Parameters

* **control_frequency**. Frequency in Hz of execution, publishing and checking of the status of the macro-actions.
* **odom_topic**. Odometry topic of the robot.
* **secs_to_check_block**. It is used to check a blocked situation. Time in seconds that the robot stays still (during navigation) to consider that the path is blocked and the robot can not move forward. 
* **block_dist**. It is used to check a blocked situation. Maximum distance in meters that the robot moves during the time indicated by "secs_to_check_block" in order to consider that the robot is blocked during navigation.


## Functioning

* Besides your proper robot simulation, the adapted_move_base node (see [adapted_move_base.launch](https://github.com/noeperez/indires_navigation/blob/master/adapted_move_base/launch/adapted_move_base.launch)) must be running. The macro-actions are programmed by using a client of the actionlib move_baser server. 

* To run the actionlib actions, launch the file indires_macro_actions.launch in the launch directory. 


## Dependences

* **actionlib**
* **adapted_move_base**


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
