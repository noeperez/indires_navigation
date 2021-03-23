cmake_minimum_required(VERSION 2.8.3)
project(control_state_machine)

SET ( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules )

find_package(catkin REQUIRED dynamic_reconfigure 
                             message_generation
                             actionlib_msgs
                             actionlib )


add_message_files(
   FILES
   ControlEvent.msg
   StateInfo.msg
)



generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)



catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

