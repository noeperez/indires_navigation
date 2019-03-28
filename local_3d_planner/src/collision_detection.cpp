
#include <local_3d_planner/collision_detection.h>


namespace local_3d_planner {

CollisionDetection::CollisionDetection(std::string name, tf::TransformListener* tf, local_3d_planner::OdometryHelperRos* oh,
		double max_lv, double max_av, double lin_acc, double ang_acc, double sim_t, double r_radius, double local_radius,
						double granularity) {
	
	tf_ = tf;
	odom_helper_ = NULL;
	if(oh)
		odom_helper_ = oh;

	max_lin_vel_ = max_lv;
	max_ang_vel_ = max_av;
	max_lin_acc_ = lin_acc;
	max_ang_acc_ = ang_acc;
	sim_time_ = sim_t;
	robot_radius_ = r_radius;
	local_radius_ = local_radius;
	granularity_ = granularity;
		
	setup(name);
}


CollisionDetection::~CollisionDetection() {
	//delete odom_helper_;
}


void CollisionDetection::setup(std::string name) {

	//ros::NodeHandle n("~");
	ros::NodeHandle n("~/" + name);

	n.param<std::string>("odometry_topic", odom_topic_, std::string("odom"));
	n.param<std::string>("base_frame", base_frame_, std::string("base_link"));

	std::string features_name;
	n.param<std::string>("features_name", features_name, std::string("nav_features_3d"));
	//printf("CollisionDetection. Features_name: %s\n", features_name.c_str());
	
	max_lv_var_ = max_lin_acc_ * sim_time_;
	max_av_var_ = max_ang_acc_ * sim_time_;


	features_ = new nav3d::Features3D(features_name, tf_, local_radius_, local_radius_, local_radius_);


	if(odom_helper_ == NULL)
		odom_helper_ = new OdometryHelperRos(odom_topic_);

	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>::CallbackType cb = boost::bind(&AssistedSteering::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);

}


/*void CollisionDetection::reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t level){
    
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    
	max_lin_vel_ = config.max_lin_vel;
	max_ang_vel_ = config.max_ang_vel;
	max_lin_acc_ = config.max_lin_acc;
	max_ang_acc_ = config.max_ang_acc;
	time_step_ = config.time_step;
	robot_radius_ = config.robot_radius;
	granularity_ = config.granularity;
	isActive_ = config.is_active;
	ang_vel_inc_ = config.ang_vel_inc;
	lin_vel_inc_ = config.lin_vel_inc;

	max_lv_var_ = max_lin_acc_ * time_step_;
	max_av_var_ = max_ang_acc_ * time_step_;

}*/




void CollisionDetection::saturateVelocities(geometry_msgs::Twist* twist)
{
	float lv = twist->linear.x;
	float av = twist->angular.z;

	float rvx = robot_vel_.getOrigin().getX();
	float rvy = robot_vel_.getOrigin().getY();
	float rvt = tf::getYaw(robot_vel_.getRotation());

	// acc linear
	if(fabs(rvx - lv) > max_lv_var_) {
		lv = (lv < rvx) ? (rvx - max_lv_var_) : (rvx + max_lv_var_);
	} 
	// acc angular
	if(fabs(rvt - av) > max_av_var_) {
		av = (av < rvt) ? (rvt - max_av_var_) : (rvt + max_av_var_);
	} 
	
	//Check maximum velocities
	if(lv > max_lin_vel_)
		lv = max_lin_vel_;
	else if(lv < (-max_lin_vel_))
		lv = max_lin_vel_*(-1);
	
	if(av > max_ang_vel_)
		av = max_ang_vel_;
	else if(av < (-max_ang_vel_))
		av = max_ang_vel_*(-1);

	twist->linear.x = lv;
	twist->angular.z = av;

}





/**
* @brief  Generate and check a single trajectory
* @param cvx The current x velocity of the robot  
* @param cvy The current y velocity of the robot  
* @param cvth The current angular velocity of the robot
* @param tvx The x velocity used to seed the trajectory
* @param tvy The y velocity used to seed the trajectory
* @param tvth The theta velocity used to seed the trajectory
* @param px will be filled with the final x point of the trajectory (robot frame)
* @param py will be filled with the final y point of the trajectory (robot frame)
* @param pz will be filled with the final z point of the trajectory (robot frame)
* @param pth will be filled with the final th point of the trajectory (robot frame)
* @return True if the trajectory is legal, false otherwise
*/
bool CollisionDetection::checkTraj(double cvx, double cvy, double cvth, double tvx, double tvy, double tvth, double& px, double& py, double& pz, double& pth)
{
	//boost::recursive_mutex::scoped_lock l(configuration_mutex_);


	px = 0.0;
	py = 0.0;
	pz = 0.0;
	pth = 0.0;

	//printf("\nCollisionDetection. CheckTraj with vels vx:%.2f, vy:%.2f th:%.2f\n", tvx, tvy, tvth);

	double max_lv = computeNewVelocity(tvx, cvx, max_lin_acc_, sim_time_);
	float vel_mag = sqrt(max_lv*max_lv);
	float steps = (vel_mag*sim_time_)/granularity_;
	float dt = sim_time_ / steps;
	float x=0.0, y=0.0, z=0.0, th=0.0;


	double lv = cvx;
	double av = cvth;

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time(); //ros::Time::now();
	pose.header.frame_id = features_->getRobotBaseFrame(); //base_frame_;

	//int ini = floor(steps/2.0 + 0.5);
	for(unsigned int i=0; i<steps; i++)
	{
		lv = computeNewVelocity(tvx, lv, max_lin_acc_, dt);
		av = computeNewVelocity(tvth, av, max_ang_acc_, dt);

		float lin_dist = lv * dt;
		th = th + (av * dt);
		//normalization just in case
		th = normalizeAngle(th, -M_PI, M_PI);
		x = x + lin_dist*cos(th); //cos(th+av*dt/2.0)
		y = y + lin_dist*sin(th); 

		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = z;
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

		//printf("CollisionDetection. CheckTraj. step %u - x:%.2f, y:%.2f, z:%.2f frame: %s -", i, x, y, z, pose.header.frame_id.c_str());

		//tomar el punto x,y,z=0 (inicialmente) y coger los vecinos en el radio.
		//Coger el valor de z del mean
		bool valid = features_->pose3dValid(&pose); 
		//pose3dValid transforma el pose a odom frame, pero después lo vuelve a base_link para coger el z correcto
		z = pose.pose.position.z;

		if(!valid){
			//printf("NOT VALID\n");
			return false;
		}
		//printf("VALID\n");
	}

	px = x;
	py = y;
	pz = z;
	pth = th; 
		
	return true;
}



} /* namespace collision detection */
