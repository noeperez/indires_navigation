# control_state_machine
Python scripts that contains the finite state machine for the navigation macro-actions and the interaction with the corresponding actions defined through the ROS actionlib library (see package indires_macro_actions).

The programmed state machine is shown in the next Figure:

![state machine](https://github.com/noeperez/indires_navigation/blob/master/indires_navigation/images/state_machine.png)


## Functioning

* Besides your proper robot simulation, the adapted_move_base node (see [adapted_move_base.launch](https://github.com/noeperez/indires_navigation/blob/master/adapted_move_base/launch/adapted_move_base.launch)), and the indires_macro_actions node (see [indires_macro_actions.launch](https://github.com/noeperez/indires_navigation/blob/master/indires_macro_actions/launch/indires_macro_actions.launch)).

* Then, run the nodes event_mananger.py and nav_behavior_fsm.py (see the launch file control_tester.launch). To test the system, an event simulator through the command line is also provided. It shows a menu to chose the macro-action to perform or the event to be triggered. The tester (control_tester.py) is also launched in the previous launch file. 

* You can also run the smach viewer to visualize the finite state machine and to watch the transitions:
> rosrun smach_viewer smach_viewer.py 

A capture of the smach viewer can be seen in the following image:

![smach viewer](https://github.com/noeperez/indires_navigation/blob/master/indires_navigation/images/smach_fsm.png)

* NOTE: the teleoperation macro-action is triggered through the detection of twist messages in the topic "manual_control_vel".


## Dependences

* **indires_macro_actions** . Set of navigation actions implemented by using the *actionlib* of ROS and controlled through the finite state machine. 
* **smach** is Python library to build hierarchical state machines (http://wiki.ros.org/smach). Install the package smach_ros under your ROS distribution. 


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
