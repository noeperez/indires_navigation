/*******************************************************************
*
* Software License Agreement (BSD License)
*
*  Author: Noé Pérez Higueras
*********************************************************************/



#include <upo_rrt_planners/ros/RRT_ros_wrapper2.h>



upo_RRT_ros::RRT_ros_wrapper2::RRT_ros_wrapper2() : tf_(NULL) {}



upo_RRT_ros::RRT_ros_wrapper2::RRT_ros_wrapper2(tf::TransformListener* tf)
{
	tf_ = tf;
	rrt_planner_ = NULL;

	setup();

}



upo_RRT_ros::RRT_ros_wrapper2::~RRT_ros_wrapper2() {
		delete checker_;
		delete rrt_planner_;
}


void upo_RRT_ros::RRT_ros_wrapper2::setup()
{
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);	


	ros::NodeHandle private_nh("~/RRT_ros_wrapper");


	private_nh.param<int>("rrt_planner_type", rrt_planner_type_, 1);
	printf("RRT_ros_wrapper2. rrt_planner_type = %i\n",  rrt_planner_type_);
      	
	//RRT 
	double aux;
 	private_nh.param<double>("rrt_solve_time", aux, 0.5);
 	solve_time_ = (float)aux;
	printf("RRT_ros_wrapper2. rrt_solve_time = %.2f\n",  solve_time_);
 	
	private_nh.param<double>("rrt_goal_bias", aux, 0.1);
	goal_bias_ = (float)aux;
	printf("RRT_ros_wrapper2. goal_bias = %.2f\n",  goal_bias_);
	
	private_nh.param<double>("rrt_max_insertion_dist", aux, 0.2);
	max_range_ = (float) aux;
	printf("RRT_ros_wrapper2. max_range_ = %.2f\n",  max_range_);
	
	double robot_radius;
	private_nh.param<double>("robot_radius", robot_radius, 0.4);
	printf("RRT_ros_wrapper2. robot_radius = %.2f\n",  robot_radius);
	
	//RRT* 
	if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		private_nh.param<bool>("rrtstar_use_k_nearest", rrtstar_use_k_nearest_, true);
		printf("RRT_ros_wrapper2. rrtstar_use_k_nearest = %i\n",  rrtstar_use_k_nearest_);
		private_nh.param<bool>("rrtstar_first_path_biasing", rrtstar_first_path_biasing_, false);
		printf("RRT_ros_wrapper2. rrtstar_first_path_biasing_ = %i\n",  rrtstar_first_path_biasing_);
		private_nh.param<double>("rrtstar_first_path_bias", aux, 0.5);
		rrtstar_first_path_bias_ = (float)aux;
		printf("RRT_ros_wrapper2. rrtstar_first_path_bias_ = %.2f\n",  rrtstar_first_path_bias_);
		private_nh.param<double>("rrtstar_first_path_stddev", aux, 0.8);
		rrtstar_first_path_stddev_bias_ = (float)aux;
		printf("RRT_ros_wrapper2. rrtstar_first_path_stddev_bias_ = %.2f\n",  rrtstar_first_path_stddev_bias_);
		private_nh.param<double>("rrtstar_rewire_factor", aux, 1.1);
		rrtstar_rewire_factor_ = (float)aux;
		printf("RRT_ros_wrapper2. rrtstar_rewire_factor_ = %.2f\n",  rrtstar_rewire_factor_);
	}
	
	//All
	private_nh.param<bool>("full_path_biasing", full_path_biasing_, false);
	printf("RRT_ros_wrapper2. full_path_biasing_ = %i\n",  full_path_biasing_);
	private_nh.param<double>("full_path_stddev", aux, 1.2);
	full_path_stddev_ = (float)aux;
	printf("RRT_ros_wrapper2. full_path_stddev_ = %.2f\n",  full_path_stddev_);
	private_nh.param<double>("full_path_bias", aux, 0.8);
	full_path_bias_ = (float)aux;
	printf("RRT_ros_wrapper2. full_path_bias_ = %.2f\n",  full_path_bias_);
	
	//private_nh.param<bool>("gmm_biasing", gmm_biasing_, false);
	//private_nh.param<double>("gmm_bias", aux, 0.95);
	//gmm_bias_ = (float)aux;
	
	
	//if RRT or RRT* are kinodynamics
	//float kino_linAcc, kino_angAcc;
	//int kino_minControlSteps, kino_maxControlSteps;
	//int kino_steeringType;
	if(rrt_planner_type_ > 2) {
		
		private_nh.param<double>("kino_time_step", aux, 0.067);
		kino_timeStep_ = (float)aux;
		private_nh.param<int>("kino_min_control_steps", kino_minControlSteps_, 5);
		private_nh.param<int>("kino_max_control_steps", kino_maxControlSteps_, 30);
		//Robot accelerations
		private_nh.param<double>("kino_linear_acc", aux, 0.6);
		kino_linAcc_ = (float)aux;
		private_nh.param<double>("kino_angular_acc", aux, 1.57);
		kino_angAcc_ = (float)aux;
		private_nh.param<int>("kino_steering_type", kino_steeringType_, 1); 
		
	}
	
	//Steering parameters for kinodynamic planning
	private_nh.param<double>("kino_steer_kp", aux, 0.5);
	float kino_steer_kp = (float)aux;
	printf("RRT_ros_wrapper. kino_steer_kp = %.2f\n",  kino_steer_kp);
	private_nh.param<double>("kino_steer_kv", aux, 3.0);
	float kino_steer_kv = (float)aux;
	printf("RRT_ros_wrapper. kino_steer_kv = %.2f\n",  kino_steer_kv);
	private_nh.param<double>("kino_steer_ka", aux, 2.0);
	float kino_steer_ka = (float)aux;
	printf("RRT_ros_wrapper. kino_steer_ka = %.2f\n",  kino_steer_ka);
	private_nh.param<double>("kino_steer_ko", aux, 0.25);
	float kino_steer_ko = (float)aux;
	printf("RRT_ros_wrapper. kino_steer_ko = %.2f\n",  kino_steer_ko);
	

	//RRT State Space
	private_nh.param<int>("rrt_dimensions", dimensions_, 2);
	printf("RRT_ros_wrapper. rrt_dimensions = %i\n",  dimensions_);
  	private_nh.param<double>("rrt_size_x", aux, 5.0);
  	size_x_ = (float)aux;
	printf("RRT_ros_wrapper. size_x_ = %.2f\n",  size_x_);
	private_nh.param<double>("rrt_size_y", aux, 5.0);
	size_y_ = (float)aux;
	printf("RRT_ros_wrapper. size_y_ = %.2f\n",  size_y_);
	private_nh.param<double>("rrt_xy_resolution", aux, 0.1);
	xy_res_ = (float)aux;
	private_nh.param<double>("rrt_yaw_resolution", aux, 0.02);
	yaw_res_ = (float)aux;
	private_nh.param<double>("rrt_min_linear_vel", aux, 0.0);
	min_lin_vel_ = (float)aux;
	private_nh.param<double>("rrt_max_linear_vel", aux, 0.5);
	max_lin_vel_ = (float)aux;
	private_nh.param<double>("rrt_lin_vel_resolution", aux, 0.05);
	lin_vel_res_ = (float)aux;
	private_nh.param<double>("rrt_max_angular_vel", aux, 0.5);
	max_ang_vel_ = (float)aux;
	private_nh.param<double>("rrt_min_angular_vel", aux, 0.3);
	min_ang_vel_ = (float)aux;
	private_nh.param<double>("rrt_ang_vel_resolution", aux, 0.1);
	ang_vel_res_ = (float)aux;
	private_nh.param<double>("rrt_goal_xy_tol", aux, 0.15);
	goal_xy_tol_ = (float)aux;
	private_nh.param<double>("rrt_goal_th_tol", aux, 0.15);
	goal_th_tol_ = (float)aux;
	private_nh.param<int>("rrt_nn_type", nn_params_, 1);
	//int distanceType;
	private_nh.param<int>("distance_type", distanceType_, 1);
	private_nh.param<int>("motion_cost_type", motionCostType_, 1);
	
	//Visualization
  	private_nh.param<bool>("visualize_rrt_tree", visualize_tree_, false);
   	private_nh.param<bool>("visualize_nav_costmap", visualize_costmap_, false);
	private_nh.param<bool>("show_rrt_statistics", show_statistics_, false);
	//private_nh.param<double>("equal_path_percentage", aux, 0.5);
	//equal_path_percentage_ = (float)aux;
	private_nh.param<double>("rrt_interpolate_path_dist", aux, 0.05);
	interpolate_path_distance_ = (float)aux;
	private_nh.param<bool>("show_intermediate_states", show_intermediate_states_, false);
	
	//path_smoothing
	private_nh.param<bool>("path_smoothing", path_smoothing_, true);
	private_nh.param<int>("smoothing_samples", smoothing_samples_, 15);
	
	
	//if the planner is an RRT, the nav costmap can not be visualized
	if(rrt_planner_type_ == 1 || rrt_planner_type_ == 3)
		visualize_costmap_ = false;
		
	ros::NodeHandle n;
	if(visualize_costmap_) {
		printf("Visualize_costmap = true, initializing costmap_pub\n");
		costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("rrt_costmap", 5);
	}
	
	//gmm_costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("gmm_costmap", 5);
	
	
	if(visualize_tree_) {
		printf("Visualize_tree = true, initializing tree_pub\n");
		tree_pub_ = n.advertise<visualization_msgs::Marker>("rrt_tree", 5);
	}
	
	rrt_goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("rrt_goal", 5);
		
	local_goal_pub_ = n.advertise<visualization_msgs::Marker>("rrt_goal_marker", 5);
	path_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_points", 5);
	path_interpol_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_interpol_points", 5);

	if(size_x_ != size_y_) {
		ROS_ERROR("X size and Y size of the State Space has to be equal!!!");
		return;
	}

	


	//costmap_sub_ = n.subscribe<nav_msgs::OccupancyGrid>("cnn_costmap", 1, &RRT_ros_wrapper2::costmapCallback, this);
	

	
	inscribed_radius_  = (float)robot_radius;
	circumscribed_radius_ = (float)robot_radius;
	//printf("Before initializing checker!!\n");
	checker_ = new ValidityChecker2(tf_, dimensions_, distanceType_);
	//printf("After initializing checker!!\n");
	

	//GMM sampling service client
	//gmm_samples_client_ = n.serviceClient<gmm_sampling::GetApproachGMMSamples>("/gmm_sampling/GetApproachGMMSamples");
	//GMM probs service client
	//gmm_probs_client_ = n.serviceClient<gmm_sampling::GetApproachGMMProbs>("/gmm_sampling/GetApproachGMMProbs");
	
	
	
	switch(rrt_planner_type_)
	{
		// ----- simple RRT --------------------
		case 1:
			printf("\n-------- Using simple RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRT();
			rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
			break;
			
		// ----- simple RRT* --------------------
		case 2:
			printf("\n-------- Using simple RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRTstar();
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}
			break;
		
		// ----- kinodynamic RRT --------------------
		case 3:
			printf("\n-------- Using Kinodynamic RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::RRT();
			rrt_planner_->as<upo_RRT::RRT>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::RRT>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::RRT>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			break;
			
		// ----- kinodynamic RRT* --------------------
		case 4:
			printf("\n-------- Using Kinodynamic RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::RRTstar();
			rrt_planner_->as<upo_RRT::RRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<upo_RRT::RRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::RRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setMotionCostType(motionCostType_); 
			if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}
			break;
			
		// ----- kinodynamic simplified RRT* --------------------
		case 5:
			printf("\n-------- Using Kinodynamic simplified RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::HalfRRTstar();
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setMotionCostType(motionCostType_); 
			if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}
			break;
			
		default:
			printf("\n-------- Using default simple RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRT();
			rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
	}
	
	
	rrt_planner_->setup(checker_, nn_params_, dimensions_, size_x_, size_y_, xy_res_, yaw_res_, min_lin_vel_, max_lin_vel_, lin_vel_res_, max_ang_vel_, ang_vel_res_,
					kino_steer_kp, kino_steer_kv, kino_steer_ka, kino_steer_ko);
				
	rrt_planner_->setGoalBias(goal_bias_);
	rrt_planner_->setGoalTolerance(goal_xy_tol_, goal_th_tol_);
	rrt_planner_->setStoreTree(visualize_tree_);
	
	if(full_path_biasing_) {
		rrt_planner_->setFullBiasing(full_path_biasing_);
		rrt_planner_->setPathBias(full_path_bias_);
		rrt_planner_->setPathBias_stddev(full_path_stddev_);
		//rrt_planner_->setGoalBias(0.0);
	}
	
	//Planning server
	ros::NodeHandle nhandle("RRT_ros_wrapper");
	plan_srv_ = nhandle.advertiseService("makeRRTPlan", &upo_RRT_ros::RRT_ros_wrapper2::makePlanService, this);
	//Planning with costmap
	planCostmap_srv_ = nhandle.advertiseService("makeRRTPlanCostmap", &upo_RRT_ros::RRT_ros_wrapper2::makePlanCostmapService, this);
	
}




void upo_RRT_ros::RRT_ros_wrapper2::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	nav_msgs::OccupancyGrid og;
	og.header = msg->header;
	og.info = msg->info;
	og.data = msg->data;
	checker_->updateCostmap(&og);
	size_x_ = (msg->info.width * msg->info.resolution); //m
	size_y_ = (msg->info.height * msg->info.resolution); //m
}




std::vector<geometry_msgs::PoseStamped> upo_RRT_ros::RRT_ros_wrapper2::RRT_plan(geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal, float start_lin_vel, float start_ang_vel)
{
	
	
	if(!rrt_planner_->setStartAndGoal(start.x, start.y, start.theta, goal.x, goal.y, goal.theta)){
		ROS_ERROR("RRT_plan. Goal state is not valid!!!");
		rrt_plan_.clear();
		geometry_msgs::PoseStamped p;
		p.pose.position.x = -100.0;
		p.pose.position.y = -100.0;
		p.pose.position.z = -100.0;
		rrt_plan_.push_back(p);
		return rrt_plan_;
	}

	upo_RRT::State* g = NULL;
	
	//if we use costs
	/*if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		//if(!use_fc_costmap_) {
			//Set the goal in the state checker
			g = new upo_RRT::State(goal.x, goal.y, goal.theta);
			checker_->setGoal(g);
		//}
		
	}*/

	geometry_msgs::PoseStamped rg;
	rg.header.frame_id = "base_link";
	rg.header.stamp = ros::Time();
	rg.pose.position.x = goal.x;
	rg.pose.position.y = goal.y;
	rg.pose.position.z = 0.0;
	rg.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);
	rrt_goal_pub_.publish(rg);
	
	//visualize rrt goal
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link"; //robot_base_frame_ = "/base_link"
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = goal.x;
	marker.pose.position.y = goal.y;
	marker.pose.position.z = 0.5;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);
	// Set the scale of the marker 
	marker.scale.x = 0.7;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.5f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	// Publish the marker
	local_goal_pub_.publish(marker);
	
	ros::Time time = ros::Time::now();
	
	rrt_planner_->setInitialActionState(start_lin_vel, 0.0, start_ang_vel, 1);
	

	//-------- GET THE RRT PATH ------------------------------

	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	

	
	//computations needed before starting planning
	//In this case, we calculate the gaussian functions over the current people
	checker_->setInitialTime(time);
	//checker_->preplanning_computations();
	
	std::vector<upo_RRT::Node> path;
	switch(rrt_planner_type_)
	{
		case 1:
 			//printf("RRT_PLAN 1 HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
			break;
			
		case 2:
			//printf("RRT_PLAN 2 HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::SimpleRRTstar>()->solve(solve_time_);
			break;
			
		case 3:
            //printf("RRT_PLAN 3 HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::RRT>()->solve(solve_time_);
			break;
			
		case 4:
   			//printf("RRT_PLAN 4 HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::RRTstar>()->solve(solve_time_);
			break;
			
		case 5:
			//printf("RRT_PLAN 5 HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::HalfRRTstar>()->solve(solve_time_);
			break;
			
		default:
			//printf("RRT_PLAN default HAS BEEN CALLED!!!!!!!");
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
	}

	if(path.empty()) {
		//rrt_plan_.clear();
		return rrt_plan_; 
	}
	if(show_statistics_) {
		upo_RRT::Planner::statistics stats = rrt_planner_->getStatistics();
		printf("Planning time:   %.4f secs\n", stats.planning_time);
		printf("First sol time:  %.4f secs\n", stats.first_sol_time);
		printf("Total samples:   %u \n", stats.total_samples);
		printf("Valid samples:   %u \n", stats.valid_samples);
		printf("Goal samples:    %u \n",  stats.goal_samples);
		printf("Tree nodes:      %u \n",  stats.tree_nodes);
		printf("Path nodes:      %u \n\n",  stats.path_nodes);
	}
	
	
	
	// Build the path in ROS format
	rrt_plan_.clear();
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.header.stamp = time;
	int cont = 0;
	for(int i=path.size()-1; i>=0; --i)
	{
		pose.pose.position.z = 0.1;
		pose.pose.position.x = (double)path[i].getState()->getX();
		pose.pose.position.y = (double)path[i].getState()->getY();
		try {
			pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)path[i].getState()->getYaw());
		} catch (tf::TransformException ex) {
			printf("TransformException in getting sol path: %s",ex.what());
		}
		
		rrt_plan_.push_back(pose);
		//printf("Getting coordinates of node %i\n", cont);
		
		if(show_intermediate_states_ && path[i].hasIntermediateStates()) {	
			std::vector<upo_RRT::State>* inter = path[i].getIntermediateStates();
			for(unsigned int j=0; j<inter->size(); j++) 
			{
				//if(i != 0 || j != (inter.size()-1)) {
					pose.pose.position.x = (double)inter->at(j).getX();
					pose.pose.position.y = (double)inter->at(j).getY();
					try {
						pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)inter->at(j).getYaw());
					} catch (tf::TransformException ex) {
						printf("TransformException in getting sol path: %s",ex.what());
					}
					pose.pose.position.z = 0.0;
					rrt_plan_.push_back(pose);
				//}
			}
				
		}
		cont++;
	}
	
	
	// Build the command list in ROS format
	std::vector<geometry_msgs::Twist> vels;
	if(rrt_planner_type_ > 2) {
		unsigned int cont = 0;
		unsigned int c = 0;
		for(unsigned int i=path.size()-1; i>0; i--)
		{
			std::vector<upo_RRT::Action>* acts = path[i].getAction();
			float vx, vy, vth; 
			unsigned int steps;
			for(unsigned int j=0; j<acts->size(); j++) {
				acts->at(j).getAction(vx, vy, vth, steps);
				geometry_msgs::Twist v;
				v.linear.x = vx;
				v.linear.y = vy;
				v.angular.z = vth;
				vels.push_back(v);
				cont+=steps;
				//printf("Action %u.  lv: %.2f, av: %.2f, Steps: %u\n", c, vx, vth, steps);
			}
			c++;
		}
		printf("Approximated Total Path Time: %.3f secs\n", kino_timeStep_*cont);
	}
	
	

	//Visualize the tree nodes of the resulting path
	/*visualization_msgs::Marker points;
	  
	points.header.frame_id = "base_link"; //robot_base_frame_; 
	points.header.stamp = time;
	points.ns = "basic_shapes";
	points.id = 0;
	points.type = visualization_msgs::Marker::SPHERE_LIST;
	points.action = visualization_msgs::Marker::ADD;
	points.pose.position.x = 0.0;
	points.pose.position.y = 0.0;
	points.pose.position.z = 0.1; 
	points.scale.x = 0.12;
	points.scale.y = 0.12;
	points.color.r = 0.0f;
	points.color.g = 1.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;
	points.lifetime = ros::Duration();
		
	for(unsigned int i=0; i<rrt_plan_.size(); i++)
	{
		geometry_msgs::Point p = rrt_plan_[i].pose.position;
		points.points.push_back(p);
	}
	path_points_pub_.publish(points);
	*/

	if(visualize_tree_) 
		visualizeTree(time);
	
	
	if(visualize_costmap_) 
		publish_feature_costmap(time);
		
	//reconf_mutex_.unlock();

		
	if(interpolate_path_distance_ > 0.0)
	{
		rrt_plan_ = path_interpolation(rrt_plan_, interpolate_path_distance_);
		
		//Visualize the interpolated path nodes
		/*visualization_msgs::Marker mar;
		  
		mar.header.frame_id = "base_link"; //robot_base_frame_; 
		mar.header.stamp = time;
		mar.ns = "basic_shapes";
		mar.id = 2;
		mar.type = visualization_msgs::Marker::SPHERE_LIST;
		mar.action = visualization_msgs::Marker::ADD;
		mar.pose.position.x = 0.0;
		mar.pose.position.y = 0.0;
		mar.pose.position.z = 0.05; 
		mar.scale.x = 0.08;
		mar.scale.y = 0.08;
		mar.color.r = 0.0f;
		mar.color.g = 1.0f;
		mar.color.b = 1.0f;
		mar.color.a = 1.0;
		mar.lifetime = ros::Duration();
			
		for(unsigned int i=0; i<rrt_plan_.size(); i++)
		{
			geometry_msgs::Point p = rrt_plan_[i].pose.position;
			mar.points.push_back(p);
		}
		path_interpol_points_pub_.publish(mar);
		*/
	}
	
	
	if(path_smoothing_)
	{
		rrt_plan_ = simple_path_smoothing(&rrt_plan_);
	}
	

	if(g)
		delete g;
	//rrt_planner_->freeTreeMemory();
	
	//delete rrt_planner_;

	return rrt_plan_;
}



bool upo_RRT_ros::RRT_ros_wrapper2::makePlanService(upo_rrt_planners::MakePlan::Request &req, upo_rrt_planners::MakePlan::Response &res)
{
	geometry_msgs::PoseStamped p = req.goal;
	//printf("makePlanService. x:%.2f, y:%.2f, z:%.2f, w:%.2f\n", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
	p = checker_->transformPoseTo(p, "base_link", false);
	
	geometry_msgs::Pose2D start;
	start.x = 0.0;
	start.y = 0.0;
	start.theta = 0.0;
	geometry_msgs::Pose2D goal;
	goal.x = p.pose.position.x;
	goal.y = p.pose.position.y;
	goal.theta = tf::getYaw(p.pose.orientation);	
	
	std::vector<geometry_msgs::PoseStamped> path = RRT_plan(start, goal, 0.0, 0.0);
	
	
	//Visualize the tree nodes of the resulting path
	if(!path.empty())
	{
		visualization_msgs::Marker points;
		  
		points.header.frame_id = "base_link"; //robot_base_frame_; 
		points.header.stamp = ros::Time::now();
		points.ns = "basic_shapes";
		points.id = 0;
		points.type = visualization_msgs::Marker::SPHERE_LIST;
		points.action = visualization_msgs::Marker::ADD;
		points.pose.position.x = 0.0;
		points.pose.position.y = 0.0;
		points.pose.position.z = 0.1; 
		points.scale.x = 0.12;
		points.scale.y = 0.12;
		points.color.r = 0.0f;
		points.color.g = 1.0f;
		points.color.b = 0.0f;
		points.color.a = 1.0;
		points.lifetime = ros::Duration();
			
		for(unsigned int i=0; i<path.size(); i++)
		{
			geometry_msgs::Point p = path[i].pose.position;
			points.points.push_back(p);
		}
		path_points_pub_.publish(points);
	} else
		return false;
	
	
	res.ok = true;
	res.path = path;
	return true;
}




bool upo_RRT_ros::RRT_ros_wrapper2::makePlanCostmapService(upo_rrt_planners::MakePlanCostmap::Request &req, upo_rrt_planners::MakePlanCostmap::Response &res)
{

	checker_->updateCostmap(&req.costmap);
	printf("MakePlanCostmap called!!!!!\n");

	geometry_msgs::PoseStamped p = req.goal;
	//printf("makePlanService. x:%.2f, y:%.2f, z:%.2f, w:%.2f\n", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
	p = checker_->transformPoseTo(p, "base_link", false);
	
	geometry_msgs::Pose2D start;
	start.x = 0.0;
	start.y = 0.0;
	start.theta = 0.0;
	geometry_msgs::Pose2D goal;
	goal.x = p.pose.position.x;
	goal.y = p.pose.position.y;
	goal.theta = tf::getYaw(p.pose.orientation);	
	
	std::vector<geometry_msgs::PoseStamped> path = RRT_plan(start, goal, 0.0, 0.0);
	
	
	//Visualize the tree nodes of the resulting path
	if(!path.empty())
	{
		visualization_msgs::Marker points;
		  
		points.header.frame_id = "base_link"; //robot_base_frame_; 
		points.header.stamp = ros::Time::now();
		points.ns = "basic_shapes";
		points.id = 0;
		points.type = visualization_msgs::Marker::SPHERE_LIST;
		points.action = visualization_msgs::Marker::ADD;
		points.pose.position.x = 0.0;
		points.pose.position.y = 0.0;
		points.pose.position.z = 0.1; 
		points.scale.x = 0.12;
		points.scale.y = 0.12;
		points.color.r = 0.0f;
		points.color.g = 1.0f;
		points.color.b = 0.0f;
		points.color.a = 1.0;
		points.lifetime = ros::Duration();
			
		for(unsigned int i=0; i<path.size(); i++)
		{
			geometry_msgs::Point p = path[i].pose.position;
			points.points.push_back(p);
		}
		path_points_pub_.publish(points);
	}
	
	
	res.ok = true;
	res.path = path;
	return true;
}









std::vector<geometry_msgs::PoseStamped> upo_RRT_ros::RRT_ros_wrapper2::simple_path_smoothing(std::vector<geometry_msgs::PoseStamped>* path)
{
	
	int s = smoothing_samples_;
	if(path->size() < (s+1))
		return *path;
	
	std::vector<geometry_msgs::PoseStamped> newpath;
	
	//Add first point
	newpath.push_back(path->at(0));
	
	//Smoothing
	for(unsigned int i=1; i<path->size()-1; i++) 
	{
		newpath.push_back(path->at(i));
		geometry_msgs::Pose2D p;
		p.x = 0.0;
		p.y = 0.0;
		p.theta = 0.0;
		int cont = 0;
		int half = (int)floor(s/2 + 0.5);
		if(i < half)
			half = i;
		else if((i+half) > path->size())
			half = path->size()-i;
		
		for(unsigned int j=i-half; j<(i+half); j++)
		{
			p.x += path->at(j).pose.position.x;
			p.y += path->at(j).pose.position.y;
			p.theta += tf::getYaw(path->at(j).pose.orientation);
			cont++;
		}
		//printf("i:%i, Half:%i, Cont: %i\n", i, half, cont);
		p.x = p.x/cont;
		p.y = p.y/cont;
		p.theta = normalizeAngle((p.theta/cont), -M_PI, M_PI);
		
		newpath[i].pose.position.x = p.x;
		newpath[i].pose.position.y = p.y;
		newpath[i].pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
	
	}
	
	//Add last point
	newpath.push_back(path->at(path->size()-1));
	
	
	return newpath;
}




std::vector<geometry_msgs::PoseStamped> upo_RRT_ros::RRT_ros_wrapper2::path_interpolation(std::vector<geometry_msgs::PoseStamped> path, float step_distance)
{
	std::vector<geometry_msgs::PoseStamped> pathnew;
	for(unsigned int i=0; i<path.size()-1; i++)
	{
		geometry_msgs::PoseStamped p1 = path[i];
		geometry_msgs::PoseStamped p2 = path[i+1];
		
		float dx = p2.pose.position.x - p1.pose.position.x;
		float dy = p2.pose.position.y - p1.pose.position.y;
		float dis = sqrt(dx*dx + dy*dy);
		
		geometry_msgs::PoseStamped intermediate = p1;
		
		pathnew.push_back(p1);
		
		float steps = dis/step_distance;
		//printf("Steps: %.2f\n", steps);
		if(steps > 1.0)
		{
			intermediate.header = path[0].header;
			intermediate.pose.position.z = p1.pose.position.z;
			for(unsigned int i=1; i<steps; i++)
			{
				float xr = (dx)*cos(0.0) + (dy)*sin(0.0);
				float yr =-(dx)*sin(0.0) + (dy)*cos(0.0);
				float tr = atan2(yr, xr);
				
				float newx = intermediate.pose.position.x + step_distance*cos(tr);
				float newy = intermediate.pose.position.y + step_distance*sin(tr);
				
				intermediate.pose.position.x = newx;
				intermediate.pose.position.y = newy;
				intermediate.pose.orientation = tf::createQuaternionMsgFromYaw(tr);
				pathnew.push_back(intermediate);
				
				dx = p2.pose.position.x - intermediate.pose.position.x;
				dy = p2.pose.position.y - intermediate.pose.position.y;
			}
		}
	}
	pathnew.push_back(path[path.size()-1]);
	//printf("Path interpolation. Original size: %u, new size: %u\n", (unsigned int)path.size(), (unsigned int)pathnew.size());
	return pathnew;
}




void upo_RRT_ros::RRT_ros_wrapper2::setBiasingPath(std::vector<geometry_msgs::PoseStamped>* path_to_follow) {
	
	if(full_path_biasing_) {
		//Transform path_to_be_followed into a vector of RRT states
		std::vector<upo_RRT::State> state_path;
		for(unsigned int i=0; i<path_to_follow->size(); i++)
		{
			geometry_msgs::PoseStamped p = path_to_follow->at(i);
			float x = p.pose.position.x;
			float y = p.pose.position.y;
			float yaw = tf::getYaw(p.pose.orientation);
			upo_RRT::State state(x, y, yaw);
			state_path.push_back(state);
		}
		rrt_planner_->setBiasingPath(&state_path);
	}
}






float upo_RRT_ros::RRT_ros_wrapper2::get_rrt_planning_radius() { return size_x_; }


void upo_RRT_ros::RRT_ros_wrapper2::visualizeTree(ros::Time t) 
{
	//std::vector<upo_RRT::Node*> tree_nodes;
	//rrt_planner_->getTree(tree_nodes);
	std::vector<upo_RRT::State> tree_states = rrt_planner_->getTree();
		
	visualization_msgs::Marker edges;
	edges.header.frame_id = "base_link";
	edges.header.stamp = t;
	edges.ns = "rrt_tree";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.action = visualization_msgs::Marker::ADD;
	edges.pose.position.x = 0.0;
	edges.pose.position.y = 0.0;
	edges.pose.position.z = 0.05;
	edges.scale.x = 0.03;
	edges.scale.y = 0.03;
	edges.color.r = 1.0f;
	edges.color.g = 1.0f;
	edges.color.b = 1.0f;
	edges.color.a = 0.3;
	edges.lifetime = ros::Duration();
	
	//printf("VisualizeTree. tree nodes: %u\n", (unsigned int)tree_states.size());
	for(unsigned int i=0; i<tree_states.size()-1; i=i+2)
	{
		//printf("Node %u. x:%.2f, y:%.2f\n", i+1, tree_states[i].getX(), tree_states[i].getY());
		//Node* parent = tree_nodes[i].getParent();
		//if(parent != NULL) {
			geometry_msgs::Point p1;
			p1.x = tree_states[i].getX();
			p1.y = tree_states[i].getY();
			p1.z = 0.05;
			geometry_msgs::Point p2;
			p2.x = tree_states[i+1].getX();
			p2.y = tree_states[i+1].getY();
			p2.z = 0.05;
			edges.points.push_back(p1);
			edges.points.push_back(p2);
		//}							
	}
	tree_pub_.publish(edges);
}


void upo_RRT_ros::RRT_ros_wrapper2::publish_feature_costmap(ros::Time t)
  {
		//Get the robot coordinates in odom frame
		tf::StampedTransform transform;
		try{
			tf_->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
			tf_->lookupTransform("/odom", "/base_link",  t, transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("Publish_feature_costmap. TF exception: %s",ex.what());
		}
	  
		nav_msgs::OccupancyGrid cmap;
		cmap.header.frame_id = "odom"; //"base_link";
		cmap.header.stamp = t;
		//time map_load_time. The time at which the map was loaded
		cmap.info.map_load_time = t;
		double cell_size = 0.25; // m/cell
		//float32 resolution. The map resolution [m/cell]
		cmap.info.resolution = cell_size;  //0.25 m/cell
		//uint32 width. Map width [cells]
		cmap.info.width = (size_x_*2.0)/cell_size;
		//uint32 height. Map height [cells]
		cmap.info.height = (size_y_*2.0)/cell_size;
		//geometry_msgs/Pose origin. The origin of the map [m, m, rad].  This is the real-world pose of the
		// cell (0,0) in the map.
		geometry_msgs::Pose p;
		p.position.x = transform.getOrigin().x()-size_x_;
		p.position.y = transform.getOrigin().y()-size_y_;
		p.position.z = 0.0;
		p.orientation = tf::createQuaternionMsgFromYaw(0.0); //robot_odom_h
		cmap.info.origin = p;
		//int8[] cmap.data. The map data, in row-major order, starting with (0,0).  Occupancy
		// probabilities are in the range [0,100].  Unknown is -1.
		std::vector<signed char> data; // size =(cmap.info.width*cmap.info.height)
		double cost = 0.0;
		for(int i=0; i<cmap.info.height; i++) 
		{
			for(unsigned int j=0; j<cmap.info.width; j++)
			{
					geometry_msgs::PoseStamped robotp;
					robotp.header.stamp = ros::Time();
					robotp.header.frame_id = "odom";
					robotp.pose.position.x = (transform.getOrigin().x()-size_x_ + cell_size*j) + (cell_size/2.0); //i
					robotp.pose.position.y = (transform.getOrigin().y()-size_y_ + cell_size*i) + (cell_size/2.0); //j
					robotp.pose.position.z = 0.0;
					tf::quaternionTFToMsg(transform.getRotation(), robotp.pose.orientation);
					
					geometry_msgs::PoseStamped robot_frame_pose = checker_->transformPoseTo(robotp, "base_link", true);
					upo_RRT::State* s = new upo_RRT::State(robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, tf::getYaw(robot_frame_pose.pose.orientation)); 
					cost = checker_->getCost(s);
					//printf("publish_feature_map. x:%.2f, y:%.2f, cost:%.2f\n", robotp.pose.position.x, robotp.pose.position.y, cost);
						
					//Transform cost into the scale[0,100]  
					data.push_back((int)round(cost*100.0)); 
			}
		}
		cmap.data = data;
		costmap_pub_.publish(cmap);
  }
  


float upo_RRT_ros::RRT_ros_wrapper2::get_path_cost()
{
	return rrt_planner_->getCost();
}










