/*******************************************************************
*
* Software License Agreement (BSD License)
*
*  Author: Noé Pérez Higueras
*********************************************************************/



#include <rrt_planners/ros/RRT_ros_wrapper.h>



RRT_ros::RRT_ros_wrapper::RRT_ros_wrapper() : tf_(NULL) {}



RRT_ros::RRT_ros_wrapper::RRT_ros_wrapper(tf::TransformListener* tf)
{
	tf_ = tf;
	rrt_planner_ = NULL;

	//ros::NodeHandle n("~/RRT_ros_wrapper");
	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<upo_rrt_planners::RRTRosWrapperConfig>(n);
	//dynamic_reconfigure::Server<upo_rrt_planners::RRTRosWrapperConfig>::CallbackType cb = boost::bind(&RRT_ros_wrapper::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);

	setup();

}

/*
RRT_ros::RRT_ros_wrapper::RRT_ros_wrapper(tf::TransformListener* tf, float controller_freq, float path_stddev, int planner_type)
{
	tf_ = tf;
	rrt_planner_ = NULL;

	//ros::NodeHandle n("~/RRT_ros_wrapper");
	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<upo_rrt_planners::RRTRosWrapperConfig>(n);
	//dynamic_reconfigure::Server<upo_rrt_planners::RRTRosWrapperConfig>::CallbackType cb = boost::bind(&RRT_ros_wrapper::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);

	//setup();
	setup_controller(controller_freq, path_stddev, planner_type);


}*/

RRT_ros::RRT_ros_wrapper::~RRT_ros_wrapper() {
		delete checker_;
		delete rrt_planner_;
}


void RRT_ros::RRT_ros_wrapper::setup()
{
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);	


	ros::NodeHandle private_nh("~/RRT_ros_wrapper");


	private_nh.param<int>("rrt_planner_type", rrt_planner_type_, 1);
	printf("RRT_ros_wrapper. rrt_planner_type = %i\n",  rrt_planner_type_);
      	
   	//private_nh.param<bool>("use_fc_in_costmap", use_fc_costmap_, false); 
	//printf("RRT_ros_wrapper. use_fc_in_costmap = %i\n",  use_fc_costmap_);
	
	//RRT 
	double aux;
 	private_nh.param<double>("rrt_solve_time", aux, 0.5);
 	solve_time_ = (float)aux;
	printf("RRT_ros_wrapper. rrt_solve_time = %.2f\n",  solve_time_);
 	
	private_nh.param<double>("rrt_goal_bias", aux, 0.1);
	goal_bias_ = (float)aux;
	printf("RRT_ros_wrapper. goal_bias = %.2f\n",  goal_bias_);
	
	private_nh.param<double>("rrt_max_insertion_dist", aux, 0.2);
	max_range_ = (float) aux;
	printf("RRT_ros_wrapper. max_range_ = %.2f\n",  max_range_);
	
	double robot_radius;
	private_nh.param<double>("robot_radius", robot_radius, 0.4);
	printf("RRT_ros_wrapper. robot_radius = %.2f\n",  robot_radius);

	
	private_nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
	private_nh.param<std::string>("robot_odom_frame", robot_odom_frame_, "odom");
	private_nh.param<std::string>("robot_pc_sensor_frame", robot_pc_sensor_frame_, "optical_frame");

	
	//RRT* 
	if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		private_nh.param<bool>("rrtstar_use_k_nearest", rrtstar_use_k_nearest_, true);
		printf("RRT_ros_wrapper. rrtstar_use_k_nearest = %i\n",  rrtstar_use_k_nearest_);
		//private_nh.param<bool>("rrtstar_first_path_biasing", rrtstar_first_path_biasing_, false);
		//printf("RRT_ros_wrapper. rrtstar_first_path_biasing_ = %i\n",  rrtstar_first_path_biasing_);
		//private_nh.param<double>("rrtstar_first_path_bias", aux, 0.5);
		//rrtstar_first_path_bias_ = (float)aux;
		//printf("RRT_ros_wrapper. rrtstar_first_path_bias_ = %.2f\n",  rrtstar_first_path_bias_);
		//private_nh.param<double>("rrtstar_first_path_stddev", aux, 0.8);
		//rrtstar_first_path_stddev_bias_ = (float)aux;
		//printf("RRT_ros_wrapper. rrtstar_first_path_stddev_bias_ = %.2f\n",  rrtstar_first_path_stddev_bias_);
		private_nh.param<double>("rrtstar_rewire_factor", aux, 1.1);
		rrtstar_rewire_factor_ = (float)aux;
		printf("RRT_ros_wrapper. rrtstar_rewire_factor_ = %.2f\n",  rrtstar_rewire_factor_);
	}
	
	//All
	/*private_nh.param<bool>("full_path_biasing", full_path_biasing_, false);
	printf("RRT_ros_wrapper. full_path_biasing_ = %i\n",  full_path_biasing_);
	private_nh.param<double>("full_path_stddev", aux, 1.2);
	full_path_stddev_ = (float)aux;
	printf("RRT_ros_wrapper. full_path_stddev_ = %.2f\n",  full_path_stddev_);
	private_nh.param<double>("full_path_bias", aux, 0.8);
	full_path_bias_ = (float)aux;
	printf("RRT_ros_wrapper. full_path_bias_ = %.2f\n",  full_path_bias_);
	*/

	private_nh.param<bool>("use_external_pc_as_samples", use_external_pc_samples_, false);
	ros::NodeHandle nh;
	if(use_external_pc_samples_) 
	{
		std::string pc_topic;
		private_nh.param<std::string>("pc_topic", pc_topic, "/cloud/points");
		pc_sub_  = nh.subscribe(pc_topic, 1, &RRT_ros::RRT_ros_wrapper::pcCallback, this);
		//pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pc_topic, 1, boost::bind(&RRT_ros::RRT_ros_wrapper::pcCallback, this, _1));
	}
	
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
	private_nh.param<int>("dimensions_type", dim_type_, 2);
	if(dimensions_ == 3)
		printf("RRT_ros_wrapper. 3d type = %i\n",  dim_type_);
  	private_nh.param<double>("rrt_size_x", aux, 5.0);
  	size_x_ = (float)aux;
	printf("RRT_ros_wrapper. size_x_ = %.2f\n",  size_x_);
	private_nh.param<double>("rrt_size_y", aux, 5.0);
	size_y_ = (float)aux;
	printf("RRT_ros_wrapper. size_y_ = %.2f\n",  size_y_);
	private_nh.param<double>("rrt_size_z", aux, 5.0);
	size_z_ = (float)aux;
	printf("RRT_ros_wrapper. size_z_ = %.2f\n",  size_z_);
	private_nh.param<double>("rrt_xyz_resolution", aux, 0.05);
	xyz_res_ = (float)aux;
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
	private_nh.param<double>("rrt_goal_xyz_tol", aux, 0.15);
	goal_xyz_tol_ = (float)aux;
	private_nh.param<double>("rrt_goal_th_tol", aux, 0.15);
	goal_th_tol_ = (float)aux;
	private_nh.param<int>("rrt_nn_type", nn_params_, 1);
	//int distanceType;
	private_nh.param<int>("distance_type", distanceType_, 1);
	private_nh.param<int>("motion_cost_type", motionCostType_, 1);
	
	//Visualization
  	private_nh.param<bool>("visualize_rrt_tree", visualize_tree_, false);
  	private_nh.param<bool>("visualize_rrt_leaves", visualize_leaves_, true);
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
		costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("rrt_costmap", 1);
	}
	
	//gmm_costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("gmm_costmap", 5);
	
	
	if(visualize_tree_) {
		printf("Visualize_tree = true, initializing tree_pub\n");
		tree_pub_ = n.advertise<visualization_msgs::Marker>("rrt_tree", 1);
	}
	if(visualize_leaves_) {
		printf("Visualize_leaves = true, initializing tree_pub\n");
		leaves_pub_ = n.advertise<visualization_msgs::Marker>("rrt_leaves", 1);
	}
	
	rrt_goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("rrt_goal", 1);
		
	local_goal_pub_ = n.advertise<visualization_msgs::Marker>("rrt_goal_marker", 1);
	path_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_points", 1);
	path_interpol_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_interpol_points", 1);

	if(size_x_ != size_y_) {
		ROS_ERROR("X size and Y size of the State Space has to be equal!!!");
		return;
	}

	//This is not working properly-------------
	//double irad, crad;
	//costmap_2d::calculateMinAndMaxDistances(footprint_, irad, crad);
	//inscribed_radius_ = irad;
	//circumscribed_radius_ = crad;
	//------------------------------------------


	
	inscribed_radius_  = (float)robot_radius;
	circumscribed_radius_ = (float)robot_radius;
	//printf("Before initializing checker!!\n");
	checker_ = new RRT_ros::ValidityChecker3D(tf_, &footprint_, inscribed_radius_, size_x_, size_y_, size_z_, xyz_res_, dimensions_, distanceType_, robot_base_frame_, robot_odom_frame_);
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
			rrt_planner_ = new RRT::SimpleRRT();
			rrt_planner_->as<RRT::SimpleRRT>()->setMaxRange(max_range_);
			break;
			
		// ----- simple RRT* --------------------
		case 2:
			printf("\n-------- Using simple RRT* planner ----------\n");
			rrt_planner_ = new RRT::SimpleRRTstar();
			rrt_planner_->as<RRT::SimpleRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<RRT::SimpleRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			//rrt_planner_->as<RRT::SimpleRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<RRT::SimpleRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			/*if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}*/
			break;
		
		// ----- kinodynamic RRT --------------------
		case 3:
			printf("\n-------- Using Kinodynamic RRT planner ----------\n");
			rrt_planner_ = new RRT::Rrt();
			rrt_planner_->as<RRT::Rrt>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<RRT::Rrt>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<RRT::Rrt>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			break;
			
		// ----- kinodynamic RRT* --------------------
		case 4:
			printf("\n-------- Using Kinodynamic RRT* planner ----------\n");
			rrt_planner_ = new RRT::RRTstar();
			rrt_planner_->as<RRT::RRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<RRT::RRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<RRT::RRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<RRT::RRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<RRT::RRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			//rrt_planner_->as<RRT::RRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<RRT::RRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<RRT::RRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<RRT::RRTstar>()->setMotionCostType(motionCostType_); 
			/*if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}*/
			break;
			
		// ----- kinodynamic simplified RRT* --------------------
		case 5:
			printf("\n-------- Using Kinodynamic simplified RRT* planner ----------\n");
			rrt_planner_ = new RRT::HalfRRTstar();
			rrt_planner_->as<RRT::HalfRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<RRT::HalfRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			//rrt_planner_->as<RRT::HalfRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<RRT::HalfRRTstar>()->setMotionCostType(motionCostType_); 
			/*if(rrtstar_first_path_biasing_ && !full_path_biasing_) {
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias(rrtstar_first_path_bias_);
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
			}*/
			break;
			
		default:
			printf("\n-------- Using default simple RRT planner ----------\n");
			rrt_planner_ = new RRT::SimpleRRT();
			rrt_planner_->as<RRT::SimpleRRT>()->setMaxRange(max_range_);
	}
	
	
	rrt_planner_->setup(checker_, nn_params_, dimensions_, dim_type_, size_x_, size_y_, size_z_, xyz_res_, yaw_res_, min_lin_vel_, max_lin_vel_, lin_vel_res_, max_ang_vel_, ang_vel_res_,
					kino_steer_kp, kino_steer_kv, kino_steer_ka, kino_steer_ko);
				
	rrt_planner_->setGoalBias(goal_bias_);
	rrt_planner_->setGoalTolerance(goal_xyz_tol_, goal_th_tol_);
	rrt_planner_->setStoreTree(visualize_tree_);
	
	/*if(full_path_biasing_) {
		rrt_planner_->setFullBiasing(full_path_biasing_);
		rrt_planner_->setPathBias(full_path_bias_);
		rrt_planner_->setPathBias_stddev(full_path_stddev_);
		//rrt_planner_->setGoalBias(0.0);
	}*/
	
	//Planning server
	ros::NodeHandle nhandle("RRT_ros_wrapper");
	plan_srv_ = nhandle.advertiseService("makeRRTPlan", &RRT_ros::RRT_ros_wrapper::makePlanService, this);
	
}




void RRT_ros::RRT_ros_wrapper::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	//Transform the coordinates of the pointcloud from sensor frame to base robot frame 
	sensor_msgs::PointCloud2 local;
	if(pcl_ros::transformPointCloud(robot_base_frame_, *msg, local, *tf_))
	{
		pc_mutex_.lock();
		//pc_ = *msg;
		pc_ = local;
		pc_mutex_.unlock();
		//setSamplingSpace(msg);
	}
	else 
		ROS_WARN("RRT_ros_wrapper. pcCallback. Error transforming received cloud");

}


void RRT_ros::RRT_ros_wrapper::setSamplingSpace()
{
	pc_mutex_.lock();
	sensor_msgs::PointCloud2 pc = pc_;
	pc_mutex_.unlock();
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromROSMsg(pc, *cloud);
	//std::vector<pcl::PointXYZ> data = cloud->points;

    sensor_msgs::PointCloud out_cloud;
    if(!sensor_msgs::convertPointCloud2ToPointCloud(pc, out_cloud)) {
		ROS_WARN("RRT_ros_wrapper. setSamplingSpace. Error converting PointCloud2 to PointCloud");
		return;	
	}

	std::vector<RRT::State> pc_rrt;

	for(int i = 0 ; i < out_cloud.points.size(); ++i)
	{
		//if(out_cloud.points[i].z < 1.5) { //WATCH OUT! A poor ceiling filter
			RRT::State s(out_cloud.points[i].x, out_cloud.points[i].y, out_cloud.points[i].z);
			pc_rrt.push_back(s);
		//}
	}
	if(pc_rrt.empty())
		ROS_WARN("setSamplingSpace. rrt_samples is empty!!!!");	
	//else
	//	ROS_INFO("SetSamplingSpace. rrt_samples: %u", (unsigned int)pc_rrt.size());

	rrt_planner_->setSamplingSpace(&pc_rrt);

}


/**
 * Only in case of using the RRT planner as a local controller
 * */
/*void upo_RRT_ros::RRT_ros_wrapper::setup_controller(float controller_freq, float path_stddev, int planner_type)
{
	
	ros::NodeHandle private_nh("~/RRT_ros_wrapper");


	switch(planner_type)
	{
		case 1:  //Kino RRT	
			rrt_planner_type_ = 3;
			break;
			
		case 2: //kino RRT*
			rrt_planner_type_ = 4;
			break;
			
		case 3: //kino simplified RRT*
			rrt_planner_type_ = 5;
			break;
			
		default:
			rrt_planner_type_ = 3;
	}
	
      	
   	//private_nh.param<bool>("use_fc_in_costmap", use_fc_costmap_, false); 
	
	 
	
 	solve_time_ = 1/controller_freq;
 	
 	double aux;
	private_nh.param<double>("rrt_goal_bias", aux, 0.1);
	goal_bias_ = (float)aux;
	
	private_nh.param<double>("rrt_max_insertion_dist", aux, 0.2);
	max_range_ = (float) aux;
	
	double robot_radius;
	private_nh.param<double>("robot_radius", robot_radius, 0.4);
	
	//RRT* 
	if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		private_nh.param<bool>("rrtstar_use_k_nearest", rrtstar_use_k_nearest_, true);
		rrtstar_first_path_biasing_ = false;
		rrtstar_first_path_bias_ = 0.0;
		rrtstar_first_path_stddev_bias_ = 0.0;
		private_nh.param<double>("rrtstar_rewire_factor", aux, 1.1);
		rrtstar_rewire_factor_ = (float)aux;
	}
	
	
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
	private_nh.param<double>("kino_steer_kv", aux, 3.0);
	float kino_steer_kv = (float)aux;
	private_nh.param<double>("kino_steer_ka", aux, 2.0);
	float kino_steer_ka = (float)aux;
	private_nh.param<double>("kino_steer_ko", aux, 0.25);
	float kino_steer_ko = (float)aux;
	

	//RRT State Space
	private_nh.param<int>("rrt_dimensions", dimensions_, 3);
  	private_nh.param<double>("rrt_size_x", aux, 5.0);
  	size_x_ = (float)aux;
	private_nh.param<double>("rrt_size_y", aux, 5.0);
	size_y_ = (float)aux;
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
	private_nh.param<int>("smoothing_samples", smoothing_samples_, 10);
	
	
	//private_nh.param<bool>("gmm_biasing", gmm_biasing_, false);
	//private_nh.param<double>("gmm_bias", aux, 0.95);
	//gmm_bias_ = (float)aux;
	
	
	//if the planner is an RRT, the nav costmap can not be visualized
	if(rrt_planner_type_ == 1 || rrt_planner_type_ == 3)
		visualize_costmap_ = false;
		
	ros::NodeHandle n;
	if(visualize_costmap_) {
		costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("rrt_costmap", 1);
	}
	if(visualize_tree_) {
		tree_pub_ = n.advertise<visualization_msgs::Marker>("rrt_tree", 1);
	}
	
	//gmm_costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("gmm_costmap", 5);
	
	
	rrt_goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("rrt_goal", 1);
		
	local_goal_pub_ = n.advertise<visualization_msgs::Marker>("rrt_goal_marker", 1);
	path_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_points", 1);
	path_interpol_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_interpol_points", 1);

	if(size_x_ != size_y_) {
		ROS_ERROR("X size and Y size of the State Space has to be equal!!!");
		return;
	}


	//GMM sampling service client
	//gmm_samples_client_ = n.serviceClient<gmm_sampling::GetApproachGMMSamples>("/gmm_sampling/GetApproachGMMSamples");
	//GMM probs service client
	//gmm_probs_client_ = n.serviceClient<gmm_sampling::GetApproachGMMProbs>("/gmm_sampling/GetApproachGMMProbs");


	//This is not working properly-------------
	//double irad, crad;
	//costmap_2d::calculateMinAndMaxDistances(footprint_, irad, crad);
	//inscribed_radius_ = irad;
	//circumscribed_radius_ = crad;
	//------------------------------------------
	inscribed_radius_  = (float)robot_radius;
	circumscribed_radius_ = (float)robot_radius;

	checker_ = new ValidityChecker(use_fc_costmap_, tf_, &footprint_, inscribed_radius_, size_x_, size_y_, xy_res_, dimensions_, distanceType_);
	
	switch(rrt_planner_type_)
	{
	
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
			break;
	}
	
	
	rrt_planner_->setup(checker_, nn_params_, dimensions_, size_x_, size_y_, xy_res_, yaw_res_, min_lin_vel_, max_lin_vel_, lin_vel_res_, max_ang_vel_, ang_vel_res_, 
		kino_steer_kp, kino_steer_kv, kino_steer_ka, kino_steer_ko);
				
	rrt_planner_->setGoalBias(goal_bias_);
	rrt_planner_->setGoalTolerance(goal_xy_tol_, goal_th_tol_);
	rrt_planner_->setStoreTree(visualize_tree_);
	
	//Full path biasing
	full_path_biasing_ = true;
	full_path_stddev_ = path_stddev;
	rrt_planner_->setFullBiasing(full_path_biasing_);
	rrt_planner_->setPathBias(1.0);
	rrt_planner_->setPathBias_stddev(full_path_stddev_);
	//rrt_planner_->setGoalBias(0.0);
	
	//Planning server
	plan_srv_ = private_nh.advertiseService("makeRRTPlan", &upo_RRT_ros::RRT_ros_wrapper::makePlanService, this);
	
}*/





/*
 void upo_RRT_ros::RRT_ros_wrapper::reconfigureCB(upo_rrt_planners::RRTRosWrapperConfig &config, uint32_t level){
    
	printf("---IN RECONFIGURE---\n");
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//reconf_mutex_.lock();
    
	//Parametes used in this class
	solve_time_ = config.solve_time;

	if(config.visualize_rrt_tree == true) {
		ros::NodeHandle n;
		if(!visualize_tree_) {
			tree_pub_ = n.advertise<visualization_msgs::Marker>("rrt_tree", 1);
		}
	}
	visualize_tree_ = config.visualize_rrt_tree;

	if(config.visualize_costmap == true) {
		ros::NodeHandle n;
		if(!visualize_costmap_) {
			costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("rrt_costmap", 1);
		}
	}
	visualize_costmap_ = config.visualize_costmap;
	
	//path smoothing
	path_smoothing_ = config.path_smoothing;
	smoothing_samples_ = config.smoothing_samples;
	

	show_statistics_ = config.show_statistics;

	//Parameters used in Planner
	goal_bias_ = config.goal_bias;
	if(rrt_planner_) {
		rrt_planner_->setGoalBias(goal_bias_);

		max_range_ = config.max_rrt_insertion_dist;
		rrtstar_first_path_biasing_ = config.rrtstar_first_path_biasing;
		rrtstar_first_path_bias_ = config.rrtstar_first_path_bias;
		rrtstar_first_path_stddev_bias_ = config.rrtstar_first_path_stddev;
		full_path_biasing_ = config.full_path_biasing;
		full_path_bias_ = config.full_path_bias;
		full_path_stddev_ = config.full_path_stddev;
		
		//gmm_biasing_ = config.gmm_biasing;
		//gmm_bias_ = config.gmm_bias;

		if(full_path_biasing_ && rrtstar_first_path_biasing_ ) //&& !gmm_biasing_
			rrtstar_first_path_biasing_ = false;

		rrt_planner_->setFullBiasing(full_path_biasing_);

		switch(rrt_planner_type_)
		{
			// ----- simple RRT --------------------
			case 1:
				rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
				break;
			
			// ----- simple RRT* --------------------
			case 2:
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setMaxRange(max_range_);
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_);
				if(rrtstar_first_path_biasing_) {
					rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias(rrtstar_first_path_bias_);
					rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
				} else {
					rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias(full_path_bias_);
					rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias_stddev(full_path_stddev_);
				}
				
				break;
		
			// ----- kinodynamic RRT --------------------
			case 3:
				break;
			
			// ----- kinodynamic RRT* --------------------
			case 4:
				rrt_planner_->as<upo_RRT::RRTstar>()->setMaxRange(max_range_);
				rrt_planner_->as<upo_RRT::RRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_); 
				if(rrtstar_first_path_biasing_) {
					rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias(rrtstar_first_path_bias_);
					rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
				} else {
					rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias(full_path_bias_);
					rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias_stddev(full_path_stddev_);
				}
				break;
			
			// ----- kinodynamic simplified RRT* --------------------
			case 5:
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setMaxRange(max_range_);
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->set_useFirstPathBiasing(rrtstar_first_path_biasing_); 
				if(rrtstar_first_path_biasing_) {
					rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias(rrtstar_first_path_bias_);
					rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias_stddev(rrtstar_first_path_stddev_bias_);
				} else {
					rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias(full_path_bias_);
					rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias_stddev(full_path_stddev_);
				}
				break;
			
			default:
				rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
		}

	}
	//reconf_mutex_.unlock();
	printf("---OUT RECONFIGURE---\n");

}*/










/**
* The coordinates of the start and goal should be in robot base frame (base_link) 
*/
std::vector<geometry_msgs::PoseStamped> RRT_ros::RRT_ros_wrapper::RRT_plan(bool exploration, geometry_msgs::Pose start, geometry_msgs::Pose goal, float start_lin_vel, float start_ang_vel)
{
	rrt_planner_->setExploration(exploration);
	RRT::State* g = NULL;
	if(exploration)
	{
		if(!rrt_planner_->setStart(start.position.x, start.position.y, start.position.z, tf::getYaw(start.orientation)))
		{
			ROS_ERROR("RRT_plan. Start state is not valid!!!");
			rrt_plan_.clear();
			geometry_msgs::PoseStamped p;
			p.pose.position.x = -100.0;
			p.pose.position.y = -100.0;
			p.pose.position.z = -100.0;
			rrt_plan_.push_back(p);
			return rrt_plan_;
		}
		//We set a NULL goal for exploration
		checker_->setGoal(g);
	}
	
	else {
		
		
		//printf("exploration = %i", (int)exploration);
		if(!rrt_planner_->setStartAndGoal(start.position.x, start.position.y, start.position.z, tf::getYaw(start.orientation), goal.position.x, goal.position.y, goal.position.z, tf::getYaw(goal.orientation))){
			ROS_ERROR("RRT_plan. Goal state is not valid!!!");
			rrt_plan_.clear();
			geometry_msgs::PoseStamped p;
			p.pose.position.x = -100.0;
			p.pose.position.y = -100.0;
			p.pose.position.z = -100.0;
			rrt_plan_.push_back(p);
			return rrt_plan_;
		}
	
		
		//if we use costs (RRT* versions)
		if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
			//Set the goal in the state checker
			g = new RRT::State(goal.position.x, goal.position.y, goal.position.z, tf::getYaw(goal.orientation));
			checker_->setGoal(g);
			
		}

		geometry_msgs::PoseStamped rg;
		rg.header.frame_id = robot_base_frame_;
		rg.header.stamp = ros::Time();
		rg.pose = goal;
		rrt_goal_pub_.publish(rg);
		
		//visualize rrt goal
		visualization_msgs::Marker marker;
		marker.header.frame_id = robot_base_frame_; //robot_base_frame_ = "/base_link"
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position = goal.position;
		marker.pose.orientation = goal.orientation;
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
	}
	
	
	ros::Time time = ros::Time::now();
	
	rrt_planner_->setInitialActionState(start_lin_vel, 0.0, start_ang_vel, 1);
	

	//-------- GET THE RRT PATH ------------------------------
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	

	
	//computations needed before starting planning
	//In this case, we calculate the gaussian functions over the current people
	checker_->setInitialTime(time);
	//checker_->preplanning_computations();

	//printf("rrt_ros_wrapper. RRT_plan. Before setting external samples\n");
	//Noé-----------
	if(use_external_pc_samples_)
		setSamplingSpace();


	//printf("rrt_ros_wrapper. RRT_plan. after setting external samples\n");

	std::vector<RRT::Node> path;
	switch(rrt_planner_type_)
	{
		case 1:
			path = rrt_planner_->as<RRT::SimpleRRT>()->solve(solve_time_);
			break;
			
		case 2:
			path = rrt_planner_->as<RRT::SimpleRRTstar>()->solve(solve_time_);
			break;
			
		case 3:
			path = rrt_planner_->as<RRT::Rrt>()->solve(solve_time_);
			break;
			
		case 4:
			path = rrt_planner_->as<RRT::RRTstar>()->solve(solve_time_);
			break;
			
		case 5:
			path = rrt_planner_->as<RRT::HalfRRTstar>()->solve(solve_time_);
			break;
			
		default:
			path = rrt_planner_->as<RRT::SimpleRRT>()->solve(solve_time_);
	}

	if(path.empty()) {
		printf("rrt_ros_wrapper. RRT_plan. Calculated path is empty!\n");
		return rrt_plan_; 
	}
	if(show_statistics_) {
		RRT::Planner::statistics stats = rrt_planner_->getStatistics();
		printf("Planning time:   %.4f secs\n", stats.planning_time);
		printf("First sol time:  %.4f secs\n", stats.first_sol_time);
		printf("Total samples:   %u \n", stats.total_samples);
		printf("Valid samples:   %u \n", stats.valid_samples);
		printf("Goal samples:    %u \n",  stats.goal_samples);
		printf("Tree nodes:      %u \n",  stats.tree_nodes);
		printf("Leaf nodes:      %u \n",  stats.leaf_nodes);
		printf("Path nodes:      %u \n\n",  stats.path_nodes);
	}
	
	
	
	// Build the path in ROS format
	rrt_plan_.clear();
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = robot_base_frame_;
	pose.header.stamp = time;
	int cont = 0;
	for(int i=path.size()-1; i>=0; --i)
	{
		pose.pose.position.x = (double)path[i].getState()->getX();
		pose.pose.position.y = (double)path[i].getState()->getY();
		pose.pose.position.z = (double)path[i].getState()->getZ();
		//printf("RRT_plan. Recovering path z: %.3f\n", pose.pose.position.z);
		try {
			pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)path[i].getState()->getYaw());
		} catch (tf::TransformException ex) {
			printf("TransformException in getting sol path: %s",ex.what());
		}
		
		rrt_plan_.push_back(pose);
		//printf("Getting coordinates of node %i\n", cont);
		
		if(show_intermediate_states_ && path[i].hasIntermediateStates()) {	
			std::vector<RRT::State>* inter = path[i].getIntermediateStates();
			for(unsigned int j=0; j<inter->size(); j++) 
			{
				//if(i != 0 || j != (inter.size()-1)) {
					pose.pose.position.x = (double)inter->at(j).getX();
					pose.pose.position.y = (double)inter->at(j).getY();
					pose.pose.position.z = (double)inter->at(j).getZ();
					try {
						pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)inter->at(j).getYaw());
					} catch (tf::TransformException ex) {
						printf("TransformException in getting sol path: %s",ex.what());
					}
					//pose.pose.position.z = 0.0;
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
			std::vector<RRT::Action>* acts = path[i].getAction();
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
	  
	points.header.frame_id = robot_base_frame_; 
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
	
	if(visualize_leaves_)
		visualizeLeaves(time);
	
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



bool RRT_ros::RRT_ros_wrapper::makePlanService(rrt_planners::MakePlan::Request &req, rrt_planners::MakePlan::Response &res)
{
	printf("rrt_ros_wrapper. make plan service received!\n");
	geometry_msgs::Pose goal;
	bool explore = false;
	if(req.goal.header.frame_id.empty())
	{
		printf("rrt_ros_wrapper. Goal empty! Passing to EXPLORATION MODE!!! \n");
		explore = true;
	} else {
		geometry_msgs::PoseStamped p = req.goal;
		//printf("makePlanService. x:%.2f, y:%.2f, z:%.2f, w:%.2f\n", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
		p = checker_->transformPoseTo(p, robot_base_frame_, false);
		goal.position = p.pose.position;
		goal.orientation = p.pose.orientation;
	}
	
	geometry_msgs::Pose start;
	start.position.x = 0.0;
	start.position.y = 0.0;
	start.position.z = 0.0;
	start.orientation = tf::createQuaternionMsgFromYaw(0.0);
	
	//goal.theta = tf::getYaw(p.pose.orientation);	

	
	std::vector<geometry_msgs::PoseStamped> path = RRT_plan(explore, start, goal, 0.0, 0.0);
	
	
	//Visualize the tree nodes of the resulting path
	if(!path.empty())
	{
		visualization_msgs::Marker points;
		  
		points.header.frame_id = robot_base_frame_; 
		points.header.stamp = ros::Time::now();
		points.ns = "rrt_path";
		points.id = 0;
		points.type = visualization_msgs::Marker::SPHERE_LIST;
		points.action = visualization_msgs::Marker::ADD;
		//points.pose.position.x = 0.0;
		//points.pose.position.y = 0.0;
		//points.pose.position.z = 0.1; 
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








/**
 * This method is used only in the case of using a kinodynamic RRT (or RRT*) planner as local controller
 * */
/*
int upo_RRT_ros::RRT_ros_wrapper::RRT_local_plan(std::vector<geometry_msgs::PoseStamped> path_to_follow, float start_lin_vel, float start_ang_vel, geometry_msgs::Twist& cmd_vel)
{
	
	if(path_to_follow.empty()) {
		ROS_ERROR("RRT_wrapper. Global plan to follow is empty!");
		return -1;
	}
	
	geometry_msgs::PoseStamped goal = path_to_follow.at(path_to_follow.size()-1);
	float goal_x = goal.pose.position.x;
	float goal_y = goal.pose.position.y;
	float goal_th = tf::getYaw(goal.pose.orientation);
	
	if(!rrt_planner_->setStartAndGoal(0.0, 0.0, 0.0, goal_x, goal_y, goal_th)){
		ROS_ERROR("RRT_plan. Goal state is not valid!!!");
		return -2;
	}
	
	
	upo_RRT::State* g = NULL;
	
	//if we use costs
	if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		if(!use_fc_costmap_) {
			//Set the goal in the state checker
			g = new upo_RRT::State(goal_x, goal_y, goal_th);
			checker_->setGoal(g);
		}
		
	}
	geometry_msgs::PoseStamped rg;
	rg.header.frame_id = "base_link";
	rg.header.stamp = ros::Time();
	rg.pose.position = goal.pose.position;
	rg.pose.orientation = goal.pose.orientation;
	rrt_goal_pub_.publish(rg);
	
	//visualize rrt goal
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link"; //robot_base_frame_ = "/base_link"
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = goal_x;
	marker.pose.position.y = goal_y;
	marker.pose.position.z = 0.5;
	marker.pose.orientation = goal.pose.orientation;
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
	
	*/
	
	//Check the distance between the robot and the goal
	//if it is satisfied only turns
	//if angle is also satisfied return 1     (copy from simple_local_planner)
	/*float distance_to_goal = sqrt(goal_x*goal_x + goal_y*goal_y);
	
	//printf("wrapper. RRT_local_plan. Distance to the goal: %.2f\n", distance_to_goal);
	
	if(distance_to_goal <= goal_xy_tol_)
	{
		// Stop the robot
		float vx = 0.0;
		float vy = 0.0;
		float vt = 0.0;
		
		// Rotate at minumin velocity until reaching the goal angle
		if(fabs(goal_th) < goal_th_tol_)
		{
			vt = 0.0;
			cmd_vel.linear.x = vx;
			cmd_vel.linear.y = vy;
			cmd_vel.linear.z = 0.0;
			cmd_vel.angular.x = 0.0;
			cmd_vel.angular.y = 0.0;
			cmd_vel.angular.z = vt;
			return 1;  //Goal reached
			
		}
		else if(goal_th < 0.0)
			vt = min_ang_vel_;
		else
			vt = -min_ang_vel_;
			
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = vy;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = vt;
		return 0;
	}*/
	
	/*
	//Transform path_to_be_followed into a vector of RRT states
	std::vector<upo_RRT::State> state_path;
	for(unsigned int i=0; i<path_to_follow.size(); i++)
	{
		geometry_msgs::PoseStamped p = path_to_follow.at(i);
		float x = p.pose.position.x;
		float y = p.pose.position.y;
		float yaw = tf::getYaw(p.pose.orientation);
		upo_RRT::State state(x, y, yaw);
		state_path.push_back(state);
	}
	rrt_planner_->setBiasingPath(&state_path);
	
	
	rrt_planner_->setInitialActionState(start_lin_vel, 0.0, start_ang_vel, 1);
	
	ros::Time time = ros::Time::now();

	
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//reconf_mutex_.lock();

	//-------- GET THE RRT PATH ------------------------------

	
	//computations needed before starting planning
	//In this case, we calculate the gaussian functions over the current people
	checker_->preplanning_computations();
	
	std::vector<upo_RRT::Node> path;
	switch(rrt_planner_type_)
	{
		case 1:
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
			break;
			
		case 2:
			path = rrt_planner_->as<upo_RRT::SimpleRRTstar>()->solve(solve_time_);
			break;
			
		case 3:
			path = rrt_planner_->as<upo_RRT::RRT>()->solve(solve_time_);
			break;
			
		case 4:
			path = rrt_planner_->as<upo_RRT::RRTstar>()->solve(solve_time_);
			break;
			
		case 5:
			path = rrt_planner_->as<upo_RRT::HalfRRTstar>()->solve(solve_time_);
			break;
			
		default:
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
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
	
	
	//Take the actions to command the robot
	//std::vector<upo_RRT::Action>* actsIni = path[path.size()-1].getAction();
	//float vx2, vy2, vth2; 
	//unsigned int steps2;
	//actsIni->at(0).getAction(vx2, vy2, vth2, steps2);
	//printf("Action InitialState. lv: %.2f, av: %.2f st:%u\n", vx2, vth2, steps2);
	
	std::vector<upo_RRT::Action>* acts = path[path.size()-2].getAction();
	float vx, vy, vth; 
	unsigned int steps;
	acts->at(0).getAction(vx, vy, vth, steps);
	cmd_vel.linear.x = vx;
	cmd_vel.linear.y = vy;
	cmd_vel.angular.z = vth;
	//printf("wrapper. init_lv: %.2f. Action 1. lv: %.2f, st:%u\n", start_lin_vel, vx, steps);
	

	
	
	// Build the command list in ROS format
	std::vector<geometry_msgs::Twist> vels;
	if(rrt_planner_type_ > 2) {
		unsigned int cont = 0;
		unsigned int c = 0;
		for(unsigned int i=path.size()-1; i>0; i--)
		{
			std::vector<upo_RRT::Action>* acts = path[i].getAction();
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
		//printf("Approximated Total Path Time: %.3f secs\n", kino_timeStep_*cont);
	}
	
	
	
	

	//Visualize the tree nodes of the resulting path
	visualization_msgs::Marker points;
	  
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
	

	if(visualize_tree_) 
		visualizeTree(time);
	
	
	if(visualize_costmap_) 
		publish_feature_costmap(time);
	
	//reconf_mutex_.unlock();

		
	if(interpolate_path_distance_ > 0.0)
	{
		rrt_plan_ = path_interpolation(rrt_plan_, interpolate_path_distance_);
		
		//Visualize the interpolated path nodes
		visualization_msgs::Marker mar;
		  
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
	}

	if(g)
		delete g;
	//rrt_planner_->freeTreeMemory();
	
	//delete rrt_planner_;

	return 0;
}
*/



std::vector<geometry_msgs::PoseStamped> RRT_ros::RRT_ros_wrapper::simple_path_smoothing(std::vector<geometry_msgs::PoseStamped>* path)
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
		newpath[i].pose.position.z = path->at(i).pose.position.z;
		newpath[i].pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
	
	}
	
	//Add last point
	newpath.push_back(path->at(path->size()-1));
	
	
	return newpath;
}




std::vector<geometry_msgs::PoseStamped> RRT_ros::RRT_ros_wrapper::path_interpolation(std::vector<geometry_msgs::PoseStamped> path, float step_distance)
{
	std::vector<geometry_msgs::PoseStamped> pathnew;
	for(unsigned int i=0; i<path.size()-1; i++)
	{
		geometry_msgs::PoseStamped p1 = path[i];
		geometry_msgs::PoseStamped p2 = path[i+1];
		
		float dx = p2.pose.position.x - p1.pose.position.x;
		float dy = p2.pose.position.y - p1.pose.position.y;
		float dz = p2.pose.position.z - p1.pose.position.z;
		float dis = sqrt(dx*dx + dy*dy + dz*dz);
		
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



/*
void RRT_ros::RRT_ros_wrapper::setBiasingPath(std::vector<geometry_msgs::PoseStamped>* path_to_follow) {
	
	if(full_path_biasing_) {
		//Transform path_to_be_followed into a vector of RRT states
		std::vector<RRT::State> state_path;
		for(unsigned int i=0; i<path_to_follow->size(); i++)
		{
			geometry_msgs::PoseStamped p = path_to_follow->at(i);
			float x = p.pose.position.x;
			float y = p.pose.position.y;
			float yaw = tf::getYaw(p.pose.orientation);
			RRT::State state(x, y, yaw);
			state_path.push_back(state);
		}
		rrt_planner_->setBiasingPath(&state_path);
	}
}*/






float RRT_ros::RRT_ros_wrapper::get_rrt_planning_radius() { return size_x_; }


void RRT_ros::RRT_ros_wrapper::visualizeTree(ros::Time t) 
{
	//std::vector<upo_RRT::Node*> tree_nodes;
	//rrt_planner_->getTree(tree_nodes);
	std::vector<RRT::State> tree_states = rrt_planner_->getTree();
		
	visualization_msgs::Marker edges;
	edges.header.frame_id = robot_base_frame_;
	edges.header.stamp = t;
	edges.ns = "rrt_tree";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.action = visualization_msgs::Marker::ADD;
	//edges.pose.position.x = 0.0;
	//edges.pose.position.y = 0.0;
	//edges.pose.position.z = 0.05;
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
			p1.z = tree_states[i].getZ();
			geometry_msgs::Point p2;
			p2.x = tree_states[i+1].getX();
			p2.y = tree_states[i+1].getY();
			p2.z = tree_states[i+1].getZ();
			edges.points.push_back(p1);
			edges.points.push_back(p2);
		//}							
	}
	tree_pub_.publish(edges);
}




void RRT_ros::RRT_ros_wrapper::visualizeLeaves(ros::Time t) 
{
	//std::vector<upo_RRT::Node*> tree_nodes;
	//rrt_planner_->getTree(tree_nodes);
	std::vector<RRT::Node> leaves = rrt_planner_->getLeaves();
		
	visualization_msgs::Marker l;
	l.header.frame_id = robot_base_frame_;
	l.header.stamp = t;
	l.ns = "rrt_tree_leaves";
	l.id = 2;
	l.type = visualization_msgs::Marker::SPHERE_LIST;
	l.action = visualization_msgs::Marker::ADD;
	//l.pose.position.x = 0.0;
	//l.pose.position.y = 0.0;
	//l.pose.position.z = 0.1; 
	l.scale.x = 0.08;
	l.scale.y = 0.08;
	l.color.r = 0.0f;
	l.color.g = 0.0f;
	l.color.b = 1.0f;
	l.color.a = 1.0;
	l.lifetime = ros::Duration();
			
	for(unsigned int i=0; i<leaves.size(); i++)
	{
		RRT::State* s = leaves[i].getState();
		geometry_msgs::Point p;
		p.x = s->getX();
		p.y = s->getY();
		p.z = s->getZ();
		l.points.push_back(p);
	}
	leaves_pub_.publish(l);
}







void RRT_ros::RRT_ros_wrapper::publish_feature_costmap(ros::Time t)
  {
		//Get the robot coordinates in odom frame
		tf::StampedTransform transform;
		try{
			tf_->waitForTransform(robot_odom_frame_, robot_base_frame_, ros::Time(0), ros::Duration(2.0));
			tf_->lookupTransform(robot_odom_frame_, robot_base_frame_,  ros::Time(0), transform); //t
		}
		catch (tf::TransformException ex){
			ROS_ERROR("Publish_feature_costmap. TF exception: %s",ex.what());
		}
	  
		nav_msgs::OccupancyGrid cmap;
		cmap.header.frame_id = robot_odom_frame_; //"base_link";
		cmap.header.stamp = ros::Time::now(); //t;
		//time map_load_time. The time at which the map was loaded
		cmap.info.map_load_time = ros::Time::now(); //t;
		double cell_size = 0.10; // m/cell 0.25
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
		p.position.z = 0.0; //transform.getOrigin().z()-size_z_;
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
					robotp.header.frame_id = robot_odom_frame_;
					robotp.pose.position.x = (transform.getOrigin().x()-size_x_ + cell_size*j) + (cell_size/2.0); //i
					robotp.pose.position.y = (transform.getOrigin().y()-size_y_ + cell_size*i) + (cell_size/2.0); //j
					robotp.pose.position.z = 0.0;
					//tf::quaternionTFToMsg(transform.getRotation(), robotp.pose.orientation);
					robotp.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));
					
					geometry_msgs::PoseStamped robot_frame_pose = checker_->transformPoseTo(robotp, robot_base_frame_, true); //true
					RRT::State* s = new RRT::State(robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, 0.0, tf::getYaw(robot_frame_pose.pose.orientation)); 
					cost = checker_->getCost(s);
					//printf("publish_feature_map. x:%.2f, y:%.2f, cost:%.2f\n", robotp.pose.position.x, robotp.pose.position.y, cost);
						
					//Transform cost into the scale[0,100]  
					data.push_back((int)round(cost*100.0)); 
			}
		}
		cmap.data = data;
		costmap_pub_.publish(cmap);
  }
  
  
  
  
  /*void upo_RRT_ros::RRT_ros_wrapper::publish_gmm_costmap(geometry_msgs::PoseStamped person)
  {
	    //Transform person to odom frame
	    geometry_msgs::PoseStamped pr = checker_->transformPoseTo(person, "odom", false);
	    //printf("person frame: %s\n", person.header.frame_id.c_str());
	    float px = pr.pose.position.x;
	    float py = pr.pose.position.y;
	    float pth  = tf::getYaw(pr.pose.orientation);
	  
		//Get the robot coordinates in odom frame
		tf::StampedTransform transform;
		try{
			tf_->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
			tf_->lookupTransform("/odom", "/base_link",  ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("Publish_gmm_costmap. TF exception: %s",ex.what());
		}
	  
		float sizex = 4.0;
		float sizey = 4.0;
	  
		nav_msgs::OccupancyGrid cmap;
		cmap.header.frame_id = "odom"; //"base_link";
		cmap.header.stamp = ros::Time::now();
		//time map_load_time. The time at which the map was loaded
		cmap.info.map_load_time = ros::Time::now();
		double cell_size = 0.05; //0.25; // m/cell
		//float32 resolution. The map resolution [m/cell]
		cmap.info.resolution = cell_size;  
		//uint32 width. Map width [cells]
		cmap.info.width = (sizex*2.0)/cell_size;
		//uint32 height. Map height [cells]
		cmap.info.height = (sizey*2.0)/cell_size;
		//geometry_msgs/Pose origin. The origin of the map [m, m, rad].  This is the real-world pose of the
		// cell (0,0) in the map.
		geometry_msgs::Pose p;
		p.position.x = transform.getOrigin().x()-sizex; //size_x_
		p.position.y = transform.getOrigin().y()-sizey; //size_y_
		p.position.z = 0.0;
		p.orientation = tf::createQuaternionMsgFromYaw(0.0); //robot_odom_h
		cmap.info.origin = p;
		
		std::vector<float> x_person;
		std::vector<float> y_person;
		
		float d, o;
		float ox, oy;
		float nx, ny;
		for(int i=0; i<cmap.info.height; i++) 
		{
			for(unsigned int j=0; j<cmap.info.width; j++)
			{
					ox = (transform.getOrigin().x()-sizex + cell_size*j) + (cell_size/2.0); //odom
					oy = (transform.getOrigin().y()-sizey + cell_size*i) + (cell_size/2.0); //odom
					
					//Transform the points from odom coordinates to person frame 
					//							|cos(th)  sin(th)  0|
					//	Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
					//							|  0        0      1|
					//								 
					//	x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
					//	y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
					//
					nx = (ox - px)*cos(pth) + (oy - py)*sin(pth);
					ny = (ox - px)*(-sin(pth)) + (oy - py)*cos(pth);
					x_person.push_back(nx);
					y_person.push_back(ny);
			}
		}
		
		//Call the gmm service to get the costs of the samples
		geometry_msgs::PoseStamped per = checker_->transformPoseTo(person, "base_link", false);
		float xp = per.pose.position.x;
		float yp = per.pose.position.y;
		float thp = tf::getYaw(per.pose.orientation); 
		//Now transform the robot (0,0) into person frame to get the orientation
		float xx = (0.0-xp)*cos(thp) + (0.0-yp)*sin(thp);
		float yy = (0.0-xp)*(-sin(thp)) + (0.0-yp)*cos(thp);
		float thh = atan2(yy, xx);
		gmm_sampling::GetApproachGMMProbs gmm_probs_srv;
		gmm_probs_srv.request.person_orientation = thh; 
		gmm_probs_srv.request.x = x_person;
		gmm_probs_srv.request.y = y_person;

		if(!gmm_probs_client_.call(gmm_probs_srv))
		{
			ROS_ERROR("RRT_ros_wrapper. Error calling service 'GetApproachGMMProbs'");
			return;
		}
		
		//int8[] cmap.data. The map data, in row-major order, starting with (0,0).  Occupancy
		// probabilities are in the range [0,100].  Unknown is -1.
		std::vector<signed char> data; // size =(cmap.info.width*cmap.info.height)
		double cost = 0.0;
		for(int i=0; i<gmm_probs_srv.response.probs.size(); i++) 
		{
			cost = gmm_probs_srv.response.probs[i];
			if(cost < 0.01)
				cost = 0.01;
			if(cost > 1.0) {
				//printf("Publish_gmm_costmap. Cost higher than 1: %.3f\n", cost); 
				cost = 1.0;
			}
			data.push_back((int)round(cost*100.0)); //*100.0 
		}
		cmap.data = data;
		gmm_costmap_pub_.publish(cmap);
  }*/
  




/*
std::vector<float> upo_RRT_ros::RRT_ros_wrapper::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path)
{
	
	std::vector<float> feature_counts;
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare("/base_link") != 0 && goal_frame.compare("base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, "/base_link", false);
	upo_RRT::State* g;
	g = new upo_RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, "/base_link", false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, "/base_link", false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double d = hypot(dx,dy);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			checker_->preplanning_computations();
			//if(visualize_costmap_) 
			//	publish_feature_costmap(ros::Time());
			
			upo_RRT::State* robot1;
			robot1 = new upo_RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation));
			std::vector<float> features1 = checker_->getFeatures(robot1);
			upo_RRT::State* robot2;
			robot2 = new upo_RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, tf::getYaw(robot_pose2.pose.orientation));
			std::vector<float> features2 = checker_->getFeatures(robot2);

			if(i==0)
				feature_counts.assign(features1.size(), 0);
			
			switch(motionCostType_)
			{	
				case 1: 
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))/2.0;
					}
					break;
				
				case 2:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))*d/2.0;
					}
					break;
					
				case 3:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + ((features1.at(j) + features2.at(j))/2.0)*exp(d);
					}
					break;
					
				case 4:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j));
					}
					break;
			}
	  }
	  return feature_counts;
	
}*/


/*
std::vector<float> upo_RRT_ros::RRT_ros_wrapper::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people)
{
	std::vector<float> feature_counts;
	
	//Set the goal
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare("/base_link") != 0 && goal_frame.compare("base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, "/base_link", false);
	upo_RRT::State* g;
	g = new upo_RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, "/base_link", false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, "/base_link", false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double d = hypot(dx,dy);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			if(!people->empty()) {
				upo_msgs::PersonPoseArrayUPO p = people->at(i);
				checker_->setPeople(p);
			}
			
			checker_->preplanning_computations();
			
			upo_RRT::State* robot1;
			robot1 = new upo_RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation));
			std::vector<float> features1 = checker_->getFeatures(robot1);
			upo_RRT::State* robot2;
			robot2 = new upo_RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, tf::getYaw(robot_pose2.pose.orientation));
			std::vector<float> features2 = checker_->getFeatures(robot2);

			if(i==0)
				feature_counts.assign(features1.size(), 0);
				
			switch(motionCostType_)
			{	
				case 1: 
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))/2.0;
					}
					break;
				
				case 2:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))*d/2.0;
					}
					break;
					
				case 3:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + ((features1.at(j) + features2.at(j))/2.0)*exp(d);
					}
					break;
					
				case 4:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j));
					}
					break;
			}
	  }
	  return feature_counts;
	
}*/



float RRT_ros::RRT_ros_wrapper::get_path_cost()
{
	return rrt_planner_->getCost();
}



float RRT_ros::RRT_ros_wrapper::get_path_cost(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path)
{
	float cost = 0.0;
	
	//Set the goal
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare(robot_base_frame_.c_str()) != 0) // && goal_frame.compare("indires_rover/base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, robot_base_frame_, false);
	RRT::State* g;
	g = new RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, mygoal.pose.position.z, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare(robot_base_frame_.c_str()) != 0) // && robot_pose.header.frame_id.compare("/indires_rover/base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, robot_base_frame_, false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare(robot_base_frame_.c_str()) != 0) // && robot_pose2.header.frame_id.compare("/indires_rover/base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, robot_base_frame_, false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double dz = robot_pose.pose.position.z - robot_pose2.pose.position.z;
			//double d = hypot(dx,dy);
			double d = sqrt(dx*dx + dy*dy + dz*dz);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			//upo_msgs::PersonPoseArrayUPO p = people->at(i);
			//checker_->setPeople(p);
			
			RRT::State* robot1;
			robot1 = new RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z, tf::getYaw(robot_pose.pose.orientation));
			float c1 = checker_->getCost(robot1);
			RRT::State* robot2;
			robot2 = new RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, robot_pose2.pose.position.z, tf::getYaw(robot_pose2.pose.orientation));
			float c2 = checker_->getCost(robot2);

			
				
			switch(motionCostType_)
			{	
				case 1: 
					cost = cost + (c1 + c2)/2.0;
					break;
				
				case 2:
					cost = cost + (c1 + c2)*d/2.0;
					break;
					
				case 3:
					cost = cost + ((c1 + c2)/2.0)*exp(d);
					break;
					
				case 4:
					cost = cost + (c1 + c2);
					break;
					
				default:
					cost = cost + (c1 + c2)*d/2.0;
			}
	  }
	  
	  return cost;
	
}









