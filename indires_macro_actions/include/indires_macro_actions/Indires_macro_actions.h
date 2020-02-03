//#ifndef MACRO_ACTION_H_
//#define MACRO_ACTION_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <indires_macro_actions/NavigateWaypointAction.h>
#include <indires_macro_actions/NavigateHomeAction.h>
#include <indires_macro_actions/ExplorationAction.h>
#include <indires_macro_actions/TeleoperationAction.h>

//Probably it is not required
//#include <adapted_move_base/move_base.h>

//#include <upo_navigation_macro_actions/Yield.h>


#include <mutex> //Mutex

//Dynamic reconfigure
/*#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <indires_macro_actions/NavigationMacroActionsConfig.h>
*/


//namespace macroactions {


	class Indires_macro_actions
	{
	
		public:
		
			//enum datatype{INT_TYPE=1, DOUBLE_TYPE=2, BOOL_TYPE=3, STRING_TYPE=4, GROUP_TYPE=5};
		
			Indires_macro_actions(tf::TransformListener& tf);
			~Indires_macro_actions();

			void navigateWaypointCB(const indires_macro_actions::NavigateWaypointGoal::ConstPtr& goal);
			void navigateHomeCB(const indires_macro_actions::NavigateHomeGoal::ConstPtr& goal);
			void explorationCB(const indires_macro_actions::ExplorationGoal::ConstPtr& goal);
			void teleoperationCB(const indires_macro_actions::TeleoperationGoal::ConstPtr& goal);
			
		
			//void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
			void robotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
			void rrtGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

			//void changeParametersNarrowPlaces();
			//void changeParametersNarrowPlaces2();
			//bool reconfigureParameters(std::string node, std::string param_name, std::string value, const datatype type);
			

		private:

			//upo_nav::UpoNavigation* UpoNav_;
			
			
			//Dynamic reconfigure
			//boost::recursive_mutex configuration_mutex_;
			//dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig> *dsrv_;
			//void reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig &config, uint32_t level);
			

			void fixFrame(std::string& cad);
			float normalizeAngle(float val, float min, float max);
			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out);
			//geometry_msgs::PoseStamped approachIT(upo_msgs::PersonPoseUPO* person);
			//geometry_msgs::PoseStamped approachIT(int id);
			
		

			//ADD CLIENT FOR THE MOVE_BASE SERVER
            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
            moveBaseClient* moveBaseClient_; 


			tf::TransformListener* tf_listener_;

			ros::NodeHandle nh_;
			ros::NodeHandle nh1_;
			ros::NodeHandle nh2_;
			ros::NodeHandle nh3_;
			ros::NodeHandle nh4_;
		

			typedef actionlib::SimpleActionServer<indires_macro_actions::NavigateWaypointAction> NWActionServer;
			NWActionServer* NWActionServer_;
			typedef actionlib::SimpleActionServer<indires_macro_actions::NavigateHomeAction> NHActionServer;
			NHActionServer* NHActionServer_;
			typedef actionlib::SimpleActionServer<indires_macro_actions::ExplorationAction> ExActionServer;
			ExActionServer* ExActionServer_;
			typedef actionlib::SimpleActionServer<indires_macro_actions::TeleoperationAction> TOActionServer;
			TOActionServer* TOActionServer_;


			indires_macro_actions::NavigateWaypointFeedback nwfeedback_;
			indires_macro_actions::NavigateWaypointResult nwresult_;

			indires_macro_actions::NavigateHomeFeedback nhfeedback_;
			indires_macro_actions::NavigateHomeResult nhresult_;

			indires_macro_actions::ExplorationFeedback exfeedback_;
			indires_macro_actions::ExplorationResult exresult_;
			
			indires_macro_actions::TeleoperationFeedback tofeedback_;
			indires_macro_actions::TeleoperationResult toresult_;


			//Parameters to be read
			double control_frequency_;
			
			// Wait
			double secs_to_check_block_;
			double block_dist_;
			double secs_to_wait_;
		
			/*	
			// Yield
			Yield* yield_;
			double secs_to_yield_;
			std::string yieldmap_;
			bool robot_inzone_;
			bool robot_inzone2_;
			bool person_inzone_;
			//bool person_inzone2_;
			boost::mutex rinzone_mutex_;
			boost::mutex pinzone_mutex_;
			geometry_msgs::Pose2D rrtgoal_;
			boost::mutex goal_mutex_;
			geometry_msgs::Pose2D robot_global_pose_;
			boost::mutex global_pose_mutex_;
			bool isYieldDirectionCorrect();
			int people_counter_;
			*/


			//Assisted steering
			bool manual_control_;
			
			
			ros::Subscriber pose_sub_;
			nav_msgs::Odometry odom_pose_;
			std::mutex pose_mutex_;
			ros::Subscriber rrtgoal_sub_;
			geometry_msgs::PoseStamped rrtgoal_;
			std::mutex goal_mutex_;

			

	};
//};
//#endif
