
#include <upo_rrt_planners/ros/RRT_ros_wrapper3.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ros_wrapper3_node");
  tf::TransformListener tf(ros::Duration(10));

  upo_RRT_ros::RRT_ros_wrapper3 rrt_wrapper(&tf);

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
