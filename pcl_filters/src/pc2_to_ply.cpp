
#include <iostream>
#include <math.h>
#include <mutex>
#include <time.h>


//PCL
/*
//This solves the problem of compiling pcl search with C++11
#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE 
#include <pcl/impl/instantiate.hpp> 
#include <pcl/point_types.h> 
PCL_INSTANTIATE(Search, PCL_POINT_TYPES) 
#endif // PCL_NO_PRECOMPILE
*/
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>



//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Bool.h>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>




std::mutex mutex; 

bool store = false;

std::string output_file;

tf::TransformListener *listener;















void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	//Transform the coordinates of the pointcloud
	sensor_msgs::PointCloud2 local;
	pcl_ros::transformPointCloud("indires_rover/odom", *msg, local, *listener);

	//pc_frame = msg->header.frame_id;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromROSMsg(*msg, *cloud);
	pcl::fromROSMsg(local, *cloud);
	

	mutex.lock();
	if(store)
	{	
		printf("\nSTORING FILE TO PLY IN:\n %s \n", output_file.c_str());
		pcl::io::savePLYFileBinary(output_file, *cloud);
		pcl::io::savePLYFileASCII(output_file+"_ascci.ply", *cloud);

		//pcl::PCLPointCloud2 cloud
		//pcl::PLYWriter writer;
 		//writer.write (filename, cloud, Eigen::Vector4f::Zero (),
               //Eigen::Quaternionf::Identity (), binary, use_camera);

	}
	store = false;
	mutex.unlock();


}





/*void poseCallback(const nav_msgs::OdometryConstPtr &msg)
{
	geometry_msgs::PoseStamped pose;

	pose.header = msg->header;
	pose.pose = msg->pose.pose;

	geometry_msgs::PoseStamped pose_out;
	//if(!pc_frame.empty())
	pose_out = transformPoseTo(pose, odom_frame, false);

	mutex.lock();
	robot_pose = pose_out;
	mutex.unlock();


}*/


void storeCallback(const std_msgs::BoolConstPtr &msg)
{
	mutex.lock();
	store = true;
	mutex.unlock();
}



int main(int argc, char **argv)
{


	ros::init(argc, argv, "pcl_filters");
	ROS_INFO("---PCL to PLY---");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");

	listener = new tf::TransformListener(ros::Duration(10));

	ros::Publisher store_pub = n.advertise<std_msgs::Bool>("store_ply", 1);

	std::string pointcloud_topic = "indires_rover/front_rgbd_camera/front_rgbd_camera/depth/points";
	//camera_tf = "indires_rover/front_rgbd_camera/depth_frame";
	nh.getParam("pointcloud_topic", pointcloud_topic);
	output_file = "PointCloud_filtered";
	nh.getParam("output_file", output_file);

	

	ros::Subscriber sub;
	sub  = n.subscribe(pointcloud_topic, 1, pcCallback);

	//ros::Subscriber sub2;
	//if(use_robot_pose)
	//	sub2 = n.subscribe(robot_pose_topic, 1, poseCallback);

	ros::Subscriber action;
	action = n.subscribe("store_ply", 1, storeCallback); 

	ros::spin();

}





