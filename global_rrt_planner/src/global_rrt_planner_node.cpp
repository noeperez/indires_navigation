#include <ros/ros.h>
#include <global_rrt_planner/global_rrt_planner_ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "global_rrt_planner_move_base");

    //tf2_ros::Buffer buffer(ros::Duration(10));
    //tf2_ros::TransformListener tf(buffer);
	tf::TransformListener tf(ros::Duration(10));

    //costmap_2d::Costmap2DROS lcr("costmap", buffer);

    global_rrt_planner::GlobalRRTPlanner planner("rrt_planner_move_base", "indires_rover/odom", &tf);

    ros::spin();
    return 0;
}
