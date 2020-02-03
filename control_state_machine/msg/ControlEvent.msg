#This message encodes control events that are internal to the robot or
#directly related to the execution of its macro-actions

#standard header, can include frame information
time stamp

#The type of the event that was detected
uint8 type

#serialized data
#uint8[] data

uint8 NAV_GOAL_REACHED=0
uint8 GO_HOME=1
uint8 LOW_BATT=2
uint8 NAV_TO_WAYPOINT_REQUESTED=3
uint8 EXPLORATION_REQUESTED=4
uint8 TELEOPERATION=5
uint8 CANCEL=6
