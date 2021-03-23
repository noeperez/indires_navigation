
#include <math.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <upo_msgs/PersonPoseUPO.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>
//PCL
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/register_point_struct.h>

//Boost
//#include <boost/thread.hpp>  // Mutex 
#include <mutex>          // std::mutex

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

class Capture {

	public:
	
		Capture(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal);
		Capture();
		~Capture();

		void setup();

		void setScenario(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal);

		//void TransformToLocal(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal, vector<geometry_msgs::PoseStamped>* agent);

		bool addObstacles(cv::Mat* img);

		bool addPeople(cv::Mat* img);

		bool addGoal(cv::Mat* img);

		bool addPath(cv::Mat* img);

		vector<int> generateImg(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal);
		cv::Mat generateImg();

		float normalizeAngle(float val, float min, float max);
		bool draw_triangle(cv::Mat* img, geometry_msgs::Point32 center, float orientation, float radius);


		// retorna "a - b" en segundos 
		double timeval_diff(struct timeval *a, struct timeval *b)
		{
  			return	(double)(a->tv_sec + (double)a->tv_usec/1000000) -
    			(double)(b->tv_sec + (double)b->tv_usec/1000000);
		}



	private:

		pair<int,int> worldToImg(geometry_msgs::Point32* world_point);
		const string currentDateTime();

		bool							save_img_;
		unsigned int					save_count_;
		string							save_path_;

		// Global parameters and object
		bool 							use_static_map_;
		bool							add_demo_path_;
		bool							only_path_label_;
		std::string 					store_dir_;
		bool							goal_ok_;
		bool							greyscale_;
 		bool 							people_orientation_;

		// Image map data
		float							resolution_;	// m/px
		int								px_height_; 	//200px * 0.05m/px = 10m
		int								px_width_;		//px
		pair<int,int> 					px_origin_;		//px
		pair<float,float> 				w_origin_;		//m 

    

		// Point cloud (obstacles)
		sensor_msgs::PointCloud* 		obs_;
	
		// people array	
		vector<upo_msgs::PersonPoseUPO>* people_;

		// Goal
		geometry_msgs::PoseStamped*		goal_;

		// Robot Path
		vector<geometry_msgs::PoseStamped>*	agent_;

};
