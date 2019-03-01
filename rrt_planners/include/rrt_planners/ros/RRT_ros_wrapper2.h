/********************************************************************
*
* Software License Agreement (BSD License)
*
*  Author: Noé Pérez Higueras
*********************************************************************/

#ifndef UPO_RRT_ROS_WRAPPER2_
#define UPO_RRT_ROS_WRAPPER2_

//C++
#include <vector>
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>

//Mutex
#include <mutex> 

//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

//GMM sampling services
//#include <gmm_sampling/GetApproachGMMSamples.h>
//#include <gmm_sampling/GetApproachGMMProbs.h>

//RRT library
#include <upo_rrt_planners/planners/Planner.h>
#include <upo_rrt_planners/planners/simple/SimpleRRT.h>
#include <upo_rrt_planners/planners/simple/SimpleRRTstar.h>
#include <upo_rrt_planners/planners/control/RRT.h>
#include <upo_rrt_planners/planners/control/RRTstar.h>
#include <upo_rrt_planners/planners/control/HalfRRTstar.h>
//Planning service
#include <upo_rrt_planners/MakePlan.h>
//Planning service over a costmap
#include <upo_rrt_planners/MakePlanCostmap.h>

#include <upo_rrt_planners/ros/ValidityChecker2.h>


//Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <upo_rrt_planners/RRTRosWrapperConfig.h>



namespace upo_RRT_ros {

	class RRT_ros_wrapper2
	{
		public:

			RRT_ros_wrapper2();
			RRT_ros_wrapper2(tf::TransformListener* tf);	
			
			~RRT_ros_wrapper2();

			void setup();
			
			////Only for RRT as a local controller
			//void setup_controller(float controller_freq, float path_stddev, int planner_type);
	
			std::vector<geometry_msgs::PoseStamped> RRT_plan(geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal, float start_lin_vel, float start_ang_vel);

			void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

			float get_rrt_planning_radius();
			
			void visualizeTree(ros::Time t);
			
			void publish_feature_costmap(ros::Time t);
			
			//void publish_gmm_costmap(geometry_msgs::PoseStamped person);

			//void setWeights(std::vector<float> w) {
			//	checker_->setWeights(w);
			//}
			
			//void setUseLossFunc(bool l, std::vector<geometry_msgs::PoseStamped> path) {
			//	checker_->setUseLossFunc(l, path);
			//}
			
			//For full path biasing using the kinodynamic RRT local controller
			//int RRT_local_plan(std::vector<geometry_msgs::PoseStamped> path_to_follow, float start_lin_vel, float start_ang_vel, geometry_msgs::Twist& cmd_vel);
			
			void setBiasingPath(std::vector<geometry_msgs::PoseStamped>* path_to_follow);
			
			std::vector<geometry_msgs::PoseStamped> simple_path_smoothing(std::vector<geometry_msgs::PoseStamped>* path);
			

			//Planning service
			bool makePlanService(upo_rrt_planners::MakePlan::Request &req, upo_rrt_planners::MakePlan::Response &res);
			//Planning service over costmap
			bool makePlanCostmapService(upo_rrt_planners::MakePlanCostmap::Request &req, upo_rrt_planners::MakePlanCostmap::Response &res);
			
			
			
			//float get_path_cost(std::vector<geometry_msgs::PoseStamped>* path);
			float get_path_cost();
			
			
			std::vector<geometry_msgs::PoseStamped> path_interpolation(std::vector<geometry_msgs::PoseStamped> path, float step_distance);
			
			float get_xy_tol() { return goal_xy_tol_;};
			float get_th_tol() { return goal_th_tol_;};
			void get_vel_ranges(float& max_lin_vel, float& min_lin_vel, float& max_ang_vel, float& min_ang_vel) {
				max_lin_vel = max_lin_vel_;
				min_lin_vel = min_lin_vel_;
				max_ang_vel = max_ang_vel_;
				min_ang_vel = min_ang_vel_;
			};
			
			
			//bool set_approaching_gmm_sampling(float orientation, int num_samp, geometry_msgs::PoseStamped person);
			
			
			inline float normalizeAngle(float val, float min, float max) {
				float norm = 0.0;
				if (val >= min)
					norm = min + fmod((val - min), (max-min));
				else
					norm = max - fmod((min - val), (max-min));
						
				return norm;
			}


		private:

			//boost::recursive_mutex configuration_mutex_;
			////boost::mutex reconf_mutex_;
			//dynamic_reconfigure::Server<upo_rrt_planners::RRTRosWrapperConfig> *dsrv_;
			//void reconfigureCB(upo_rrt_planners::RRTRosWrapperConfig &config, uint32_t level);

			//ROS
			//costmap_2d::Costmap2DROS* 		global_costmap_ros_; 
			//costmap_2d::Costmap2DROS* 		local_costmap_ros_;  //The ROS wrapper for the costmap the controller will use

			//costmap_2d::Costmap2D* 			global_costmap_; 
			//costmap_2d::Costmap2D* 			local_costmap_;   //The costmap the controller will use
			
			tf::TransformListener* 			tf_;
			

			//Robot
			float							inscribed_radius_;
			float							circumscribed_radius_;
			std::vector<geometry_msgs::Point> footprint_;
			
			//RRT
			upo_RRT::Planner*				rrt_planner_;
			float							solve_time_;
			std::vector<geometry_msgs::PoseStamped> rrt_plan_;
			upo_RRT_ros::ValidityChecker2* 	checker_;
			int 							rrt_planner_type_;

			//Services
			ros::ServiceServer 				plan_srv_;
			ros::ServiceServer 				planCostmap_srv_;

			//Subscription to the costmap
			ros::Subscriber					costmap_sub_;
			
			int 							motionCostType_;
			
			float 							kino_timeStep_;
			

			//float							equal_path_percentage_;
			float 							path_cost_;

			//StateSpace
			int 							dimensions_;
			float 							size_x_;
			float							size_y_;
			
			//Visualization
			bool							visualize_tree_;
			bool							visualize_costmap_;
			bool							show_statistics_;
			bool							show_intermediate_states_;
			float 							interpolate_path_distance_;
			ros::Publisher 					local_goal_pub_;
			ros::Publisher					rrt_goal_pub_;
			ros::Publisher					costmap_pub_;
			ros::Publisher 					tree_pub_;
			ros::Publisher					path_points_pub_;
			ros::Publisher					path_interpol_points_pub_;
			
			//Path smoothing
			bool							path_smoothing_;
			int								smoothing_samples_;
			
			/*//GMM biasing
			bool							gmm_biasing_;
			float 							gmm_bias_;
			ros::ServiceClient 				gmm_samples_client_;
			ros::ServiceClient 				gmm_probs_client_;
			std::vector< std::pair<float,float> > gmm_samples_;
			geometry_msgs::PoseStamped		gmm_person_;
			float 							gmm_person_ori_;
			boost::mutex 					gmm_mutex_;
			ros::Publisher					gmm_costmap_pub_;
			*/
			
			//-------------------------------------------
			float 							goal_bias_;
			float 							max_range_;
			bool 							rrtstar_use_k_nearest_;
			bool 							rrtstar_first_path_biasing_;
			float 							rrtstar_first_path_bias_;
			float 							rrtstar_first_path_stddev_bias_;
			float 							rrtstar_rewire_factor_;
			bool							full_path_biasing_;
			float 							full_path_stddev_;
			float 							full_path_bias_;
			
			int 							kino_minControlSteps_;
			int 							kino_maxControlSteps_;
			float 							kino_linAcc_;
			float 							kino_angAcc_;
			int 							kino_steeringType_;
			float 							xy_res_;
			float 							yaw_res_;
			float 							min_lin_vel_;
			float 							max_lin_vel_;
			float 							lin_vel_res_;
			float 							max_ang_vel_;
			float 							min_ang_vel_;
			float 							ang_vel_res_;
			float 							goal_xy_tol_;
			float 							goal_th_tol_;
			int 							nn_params_;
			int								distanceType_;
			
	};
} 
#endif
