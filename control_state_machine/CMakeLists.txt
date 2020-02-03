cmake_minimum_required(VERSION 2.8.3)
project(control_state_machine)

SET ( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules )

find_package(catkin REQUIRED dynamic_reconfigure 
                             message_generation
                             actionlib_msgs
                             actionlib )

#find_package( MADP )

add_message_files(
   FILES
   ControlEvent.msg
   StateInfo.msg
   #IDArray.msg
   #HRIControlMode.msg
   #HRIFeedbackEvent.msg
   #HRIFeedbackFromInterface.msg
   #HRIFeedbackStateInfo.msg
)

#add_service_files(
#   FILES
#   TestPolicy.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#   InformedLookAround.action
#   LookAround.action
#   LookForInteractionTargets.action
#   MaintainHeadingToInteractionTargets.action
#   MaintainInteractionRange.action
#   MoveToInteractionRange.action
#   StabilizeHeadPose.action
#   RepositionForConversation.action
#   TurnToInteractionTargets.action
#   NNPolicy.action
#)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/HRIActionParams.cfg
#  cfg/PIDParams.cfg
#)

catkin_package(
  CATKIN_DEPENDS #mdm_library
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

