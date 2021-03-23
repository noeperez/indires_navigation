

#ifndef UPO_RRT_CHECKER2_
#define UPO_RRT_CHECKER2_


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

//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <upo_msgs/PersonPoseUPO.h>
//#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>

//Features for navigation cost functions
//#include <navigation_features/nav_features.h>

//Mutex
#include <mutex> 


//using namespace std;



namespace upo_RRT_ros
{

	class ValidityChecker2 : public upo_RRT::StateChecker
	{
		public:
		
		//ValidityChecker(bool use_fc_costmap, tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, unsigned int dimensions, int distType); 
 		ValidityChecker2(nav_msgs::OccupancyGrid* costmap, tf::TransformListener* tf, unsigned int dimensions, int distType);

		ValidityChecker2(tf::TransformListener* tf, unsigned int dimensions, int distType);

		virtual ~ValidityChecker2();

		void updateCostmap(nav_msgs::OccupancyGrid* costmap);
		
		bool isValid(upo_RRT::State* s) const;
		
		
		//Distance function between two states
		float distance(upo_RRT::State* s1, upo_RRT::State* s2) const;
		
		float getCost(upo_RRT::State* s);
		
		//std::vector<float> getFeatures(upo_RRT::State* s);
		
		/*void setGoal(upo_RRT::State* g) { 
			geometry_msgs::PoseStamped goal;
			goal.header.frame_id = "base_link";
			goal.header.stamp = ros::Time();
			goal.pose.position.x = g->getX();
			goal.pose.position.y = g->getY();
			goal.pose.position.z = 0.0;
			goal.pose.orientation = tf::createQuaternionMsgFromYaw(g->getYaw());
			navfeatures_->setGoal(goal);
		}*/
		
		//Implemented for learning purposes
		//void setPeople(upo_msgs::PersonPoseArrayUPO p);
		
		
		geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
		
		bool isQuaternionValid(const geometry_msgs::Quaternion q);


		std::vector<int> poseToCell(std::vector<float> pose) const;
		std::vector<float> cellToPose(std::vector<int> cell) const;
		
		//void setWeights(std::vector<float> we);
		
		/*void setUseLossFunc(bool l, std::vector<geometry_msgs::PoseStamped> path) {
			navfeatures_->set_use_loss_func(l);
			navfeatures_->set_demo_path(path);
		}*/
			
		//Pre-computations needed just before starting planning
		void preplanning_computations();
		
		void setInitialTime(ros::Time t) {
			time_ = t;
		}


		inline float normalizeAngle(float val, float min, float max) const {
			float norm = 0.0;
			if (val >= min)
				norm = min + fmod((val - min), (max-min));
			else
				norm = max - fmod((min - val), (max-min));
					
			return norm;
		}
		
		
		private:
		
		//features::NavFeatures* 				navfeatures_;
		
		//const costmap_2d::Costmap2D* 		loc_costmap_;
		//const costmap_2d::Costmap2D* 		glo_costmap_;
		//bool 								use_global_costmap_;
		tf::TransformListener*				tf_;

		std::vector<int>					costmap_;
		std::mutex							costmap_mutex_;
		float								resolution_;
		float 								width_;
		float								height_;
		std::vector<float>					origin_; //The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map
		
		unsigned int 						dimensions_;
		int 								distanceType_;
		//bool								get_cost_from_costmap_;
		
		ros::Time							time_;
		

	};

}
#endif 

