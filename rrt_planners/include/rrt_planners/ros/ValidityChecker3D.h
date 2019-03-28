

#ifndef RRT_CHECKER_
#define RRT_CHECKER_


// RRT library
#include <rrt_planners/StateChecker.h>

//C++
#include <vector>
#include <list>
#include <cmath>
#include <math.h> 
#include <iostream>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atof, srand, rand */
#include <exception>      // std::exception
#include <time.h>       /* time */
#include <random>

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
#include <navigation_features_3d/nav_features3d.h>

//Mutex
#include <mutex> 


namespace RRT_ros
{

	class ValidityChecker3D : public RRT::StateChecker
	{
		public:
		
		ValidityChecker3D(tf::TransformListener* tf, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, float size_z, float res, unsigned int dimensions, int distType, std::string planning_frame); 

		virtual ~ValidityChecker3D();
		
		bool isValid(RRT::State* s) const;

		bool getValid3dState(RRT::State* s) const;
		
		
		//Distance function between two states
		float distance(RRT::State* s1, RRT::State* s2) const;
		
		float getCost(RRT::State* s);
		
		std::vector<float> getFeatures(RRT::State* s);
		
		
		//This two methods are for exploring purposes
		std::vector<RRT::Node*> clusterize_leaves(std::vector<RRT::Node*>* leaves) const;
		RRT::Node* evaluate_exploration(std::vector<RRT::Node*>* leaves) const;
		
		
		inline std::string getPlanningFrame() {return planning_frame_; }
		
		void setGoal(RRT::State* g) { 
			geometry_msgs::PoseStamped goal;
			//For exploration
			if(g == NULL) {
				features_->setGoal(goal);
				return;
			}
			goal.header.frame_id = planning_frame_;
			goal.header.stamp = ros::Time();
			goal.pose.position.x = g->getX();
			goal.pose.position.y = g->getY();
			goal.pose.position.z = g->getZ();
			goal.pose.orientation = tf::createQuaternionMsgFromYaw(g->getYaw());
			features_->setGoal(goal);
		}
		
		//Implemented for learning purposes
		//void setPeople(upo_msgs::PersonPoseArrayUPO p);
		
		
		geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
		
		bool isQuaternionValid(const geometry_msgs::Quaternion q);
		
		
		void setWeights(std::vector<float> we);
		
		/*void setUseLossFunc(bool l, std::vector<geometry_msgs::PoseStamped> path) {
			navfeatures_->set_use_loss_func(l);
			navfeatures_->set_demo_path(path);
		}*/
			
		//Pre-computations needed just before starting planning
		//void preplanning_computations();
		
		void setInitialTime(ros::Time t) {
			time_ = t;
		}
		
		
		private:
		
		//features::NavFeatures* 			navfeatures_;
		nav3d::Features3D*					features_;
		
		//const costmap_2d::Costmap2D* 		loc_costmap_;
		//const costmap_2d::Costmap2D* 		glo_costmap_;
		//bool 								use_global_costmap_;
		tf::TransformListener*				tf_;
		
		unsigned int 						dimensions_;
		int 								distanceType_;
		
		ros::Time							time_;

		std::string							planning_frame_;
		//std::string							robot_odom_frame_;
		
		ros::Publisher						explore_pub_;
		
		int 								rrt_planner_type_;
		
		bool								nfe_exploration_;
		

	};

}
#endif 

