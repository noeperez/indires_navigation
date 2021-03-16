#include <ros/ros.h>
#include <indires_macro_actions/Indires_macro_actions.h>

int main(int argc, char** argv)
{
 	ros::init(argc, argv, "indires_macro_actions");
 
	ROS_INFO("Starting indires_macro_actions...");
	//tf::TransformListener tf(ros::Duration(10));
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	//upo_nav::UpoNavigation UpoNav(tf, true);

	Indires_macro_actions macro(&tfBuffer); //(tf, &UpoNav);
	
 	ros::spin();

 	return 0;
}
