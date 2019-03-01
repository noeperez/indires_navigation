
#include <iostream>
#include <math.h>
#include <mutex>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>



std::string sensor_frame;
bool publishFirst = true;

ros::Publisher pc_pub;

tf::TransformListener *listener;

sensor_msgs::PointCloud2 pc2;
std::mutex mutex2;

sensor_msgs::PointCloud2 pc1;
std::mutex mutex1;



void pc1Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	mutex1.lock();
	sensor_frame = msg->header.frame_id;
	pc1 = *msg;
	mutex1.unlock();
}




void pc2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	mutex2.lock();
	pc2 = *msg;
	if(!sensor_frame.empty() && msg->header.frame_id != sensor_frame)
	{
		//Transform the coordinates of the pointcloud
		pcl_ros::transformPointCloud(sensor_frame, *msg, pc2, *listener);
	}
	mutex2.unlock();
}



void publish_pc()
{
	if(!sensor_frame.empty())
	{

		sensor_msgs::PointCloud2 pc;

		if(publishFirst)
		{
			mutex1.lock();
			pc = pc1;
			mutex1.unlock();
			publishFirst = false;
			pc_pub.publish(pc);
		} else
		{
			mutex2.lock();
			pc = pc2;
			mutex2.unlock();
			publishFirst = true;
			pc_pub.publish(pc);
		}
	}

}
  



int main(int argc, char **argv)
{


	ros::init(argc, argv, "alternate");
	ROS_INFO("---ALTERNATE POINTCLOUDS---");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");

	listener = new tf::TransformListener(ros::Duration(10));

	std::string pc_topic1 = "indires_rover/front_rgbd_camera/depth/points";
	std::string pc_topic2 = "indires_rover/above_rgbd_camera/depth/points";
	//camera_tf = "indires_rover/front_rgbd_camera/depth_frame";
	nh.getParam("in_pc_topic_1", pc_topic1);
	nh.getParam("in_pc_topic_2", pc_topic2);
	std::string output_topic = "PointCloud_filtered";
	nh.getParam("output_topic", output_topic);

	ros::Subscriber sub1;
	sub1  = n.subscribe(pc_topic1, 1, pc1Callback);

	ros::Subscriber sub2;
	sub2 = n.subscribe(pc_topic2, 1, pc2Callback);

	pc_pub = nh.advertise<sensor_msgs::PointCloud2>( output_topic, 0);

	ros::Rate r(20);
	while(ros::ok())
	{		
		ros::spinOnce();
		publish_pc();
		r.sleep();
	}
}

