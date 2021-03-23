

#ifndef UPO_RRT_CHECKER3_
#define UPO_RRT_CHECKER3_


//upo RRT library
#include <upo_rrt_planners/StateChecker.h>

//C++
#include <vector>
#include <list>
#include <cmath>
#include <math.h> 
#include <iostream>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atof */
#include <exception>      // std::exception
#include <time.h>       /* time */
#include <sys/time.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
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

//creation of network input
#include <upo_rrt_planners/ros/capture.h>

//Service for network prediction
#include <path_prediction/PathPrediction.h>

//#include <boost/thread.hpp>  /* Mutex */
#include <mutex>

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;



namespace upo_RRT_ros
{

	class ValidityChecker3 : public upo_RRT::StateChecker
	{
		public:
		
 		ValidityChecker3(tf::TransformListener* tf, float resol, int width, int height, unsigned int dimensions, int distType);

		//ValidityChecker3(tf::TransformListener* tf, unsigned int dimensions, int distType);

		virtual ~ValidityChecker3();

		void setup();

		void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
			
		void peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg);
			
		void pcCallback(const sensor_msgs::PointCloud::ConstPtr& pc_in);   //PointCloud
		void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& pc_in); //PointCloud2

		void setObstacles(sensor_msgs::PointCloud2 obs);

		void setupStaticMap(ros::NodeHandle nh);
		void setupNoMapProjection();

		void updateDistTransform();

		bool predictionService(vector<int> input, int rows, int cols);

		//vector<> get_prediction_points();
		
		bool isValid(upo_RRT::State* s) const;
		
		//Distance function between two states
		float distance(upo_RRT::State* s1, upo_RRT::State* s2) const;
		
		float getCost(upo_RRT::State* s);
		
		//std::vector<float> getFeatures(upo_RRT::State* s);
		
		void setGoal2(upo_RRT::State* g) { 
			goal_mutex_.lock();
			goal_.header.frame_id = "base_link";
			goal_.header.stamp = ros::Time();
			goal_.pose.position.x = g->getX();
			goal_.pose.position.y = g->getY();
			goal_.pose.position.z = 0.0;
			goal_.pose.orientation = tf::createQuaternionMsgFromYaw(g->getYaw());
			goal_mutex_.unlock();
		}


		void setGoal(geometry_msgs::PoseStamped g)
		{
			goal_mutex_.lock();
			goal_ = g;
			goal_mutex_.unlock();
		}
		

		//Implemented for learning purposes
		//void setPeople(upo_msgs::PersonPoseArrayUPO p);
		
		
		geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped* pose_in, string frame_out, bool usetime) const;
		
		bool isQuaternionValid(const geometry_msgs::Quaternion q);

		vector<int> worldToMap(geometry_msgs::Point32* world_point, nav_msgs::MapMetaData* map_metadata) const;
		vector<int> BaseLinkWorldToImg(geometry_msgs::Point32* point) const;

		vector<int> poseToCostmapCell(vector<float> pose) const;
		vector<float> costmapCellToPose(vector<int> cell) const;
		
			
		//Pre-computations needed just before starting planning
		void preplanning_computations();


		vector<upo_RRT::State> getPredictedPointList();

		
		void setInitialTime(ros::Time t) {
			time_ = t;
		}


		void publish_costmap_ros(ros::Time t);


		inline float normalizeAngle(float val, float min, float max) const {
			float norm = 0.0;
			if (val >= min)
				norm = min + fmod((val - min), (max-min));
			else
				norm = max - fmod((min - val), (max-min));
					
			return norm;
		}

		
		private:
		
		Capture* 							capture_;

		ros::ServiceClient 					cnn_client_;

		tf::TransformListener*				tf_;
		
		//Local costmap
		vector<float>						costmap_;
		mutex								costmap_mutex_;
		float								cm_resolution_;
		float 								cm_width_; //m
		float								cm_height_; //m
		int									cm_width_pixs_;
		int									cm_height_pixs_;
		vector<float>						cm_origin_;
		ros::Publisher						costmap_pub_;
		float								path_threshold_;

		//Static map
		nav_msgs::MapMetaData 				map_metadata_;
		double 								map_resolution_;
		vector<float> 						map_origin_;
 		cv::Mat 							map_image_;
		mutex 								dt_mutex_;
		cv::Mat 							distance_transform_;
		bool								project_onto_map_;
		int									people_paint_area_;

		//Point cloud
		ros::Subscriber 					sub_pc_;
		sensor_msgs::PointCloud2 			laser_cloud_;
		mutex 								pc_mutex_;
	
		// list of person objects	
		std::vector<upo_msgs::PersonPoseUPO> people_;
		ros::Subscriber 					sub_people_;
		mutex 								people_mutex_;

		//Goal
		geometry_msgs::PoseStamped			goal_;
		mutex								goal_mutex_;
		ros::Subscriber						goal_sub_;

		
		ros::Time							time_;
		unsigned int						dimensions_;
		int									distanceType_;
		double								insc_radius_robot_;
		
		

	};

}
#endif 

