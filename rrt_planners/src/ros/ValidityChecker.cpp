
#include <upo_rrt_planners/ros/ValidityChecker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>


upo_RRT_ros::ValidityChecker::ValidityChecker(bool use_fc_costmap, tf::TransformListener* tf, std::vector<geometry_msgs::Point>* footprint, 
	float insc_radius, float size_x, float size_y, float res, unsigned int dimensions, int distType) : StateChecker()
{
	
	get_cost_from_costmap_ = use_fc_costmap;
	
	//if(!get_cost_from_costmap_) {
		//printf("Initialization of nav_features\n");
		navfeatures_ = new features::NavFeatures(tf, footprint, insc_radius, size_x, size_y, res);
	/*}else {
		printf("----Using cost function to build a costmap-----\n");
		loc_costmap_ = loc_costmap;
		glo_costmap_ = glob_costmap;
		tf_ = tf;
	}*/
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();
}


upo_RRT_ros::ValidityChecker::~ValidityChecker() {
	
	delete navfeatures_;
}


bool upo_RRT_ros::ValidityChecker::isValid(upo_RRT::State* s) const
{
	geometry_msgs::PoseStamped p_in;
	p_in.header.frame_id = "base_link"; 
	//p_in.header.stamp = ros::Time(0); //this is a problem when the planning time is long. the time stamp should be the time when the rrt started to plan.
	if((ros::Time::now()-time_).toSec() > 2.0) {
			//time_ = ros::Time::now();
			p_in.header.stamp = ros::Time(0);
	} else 
		p_in.header.stamp = time_;
	
	p_in.pose.position.x = s->getX();
	p_in.pose.position.y = s->getY();
	p_in.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	
	if(!get_cost_from_costmap_)  
	{
		//If we calculate the validity in a normal way 
		return navfeatures_->poseValid(&p_in);
		
		
	} else {  
		//we check the validity checking the value of the costmap built by using the RRT* cost function
		printf("\nERROR!! Validity checking by using costmap is not available!!!!\n");
		return false;
		
	}
}


void upo_RRT_ros::ValidityChecker::preplanning_computations()
{
	if(!get_cost_from_costmap_)
		navfeatures_->update();
}


float upo_RRT_ros::ValidityChecker::distance(upo_RRT::State* s1, upo_RRT::State* s2) const
{
	float dx = s1->getX() - s2->getX();
	float dy = s1->getY() - s2->getY();
	//float dist = sqrt(dx*dx + dy*dy);
	float dist = dx*dx + dy*dy;
	
	switch(distanceType_) {
		
		case 1:
			return dist;

		case 2:
			return sqrt(dist);

		case 3:
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//SUM w1*|| Pi+1 - Pi|| + w2*(1-|Qi+1 * Qi|)²
				float euc_dist = sqrt(dist);
		
				tf::Quaternion q1 = tf::createQuaternionFromYaw(s1->getYaw());
				tf::Quaternion q2 = tf::createQuaternionFromYaw(s2->getYaw());
				float dot_prod = q1.dot(q2);
				float angle_dist =  (1 - fabs(dot_prod))*(1 - fabs(dot_prod));
				//printf("eu_dist: %.2f, angle_dist: %.3f, dist: %.3f\n", euc_dist, angle_dist, 0.8*euc_dist + 0.2*angle_dist);
				return 0.7*euc_dist + 0.3*angle_dist;
			}
			
		case 4:
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				// Another option
				/*
				First, transform the robot location into person location frame: 
											|cos(th)  sin(th)  0|
					Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
											|  0        0      1|
												 
					x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
					y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
				*/
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				return (0.8*sqrt(dist)+0.2*fabs(alpha));
			}
			
		case 5:  
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//UPO. Dist + sum of the angles of both points regarding the intersection line
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float beta = s2->getYaw() - alpha;
				beta = navfeatures_->normalizeAngle(beta, -M_PI, M_PI);
				return (0.6*sqrt(dist)+0.4*(fabs(alpha)+fabs(beta)));
			}
			
		case 6:  
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//Paper IROS2015 "Feedback motion planning via non-holonomic RRT* for mobile robots"
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float phi = s2->getYaw() - alpha;
				phi = navfeatures_->normalizeAngle(phi, -M_PI, M_PI);
				float ka = 0.5;
				float ko = ka/8.0;
				dist = sqrt(dist);
				// two options
				float alpha_prime = atan(-ko*phi);
				//float alpha_prime = atan(-ko*ko * phi/(dist*dist));
				float r = navfeatures_->normalizeAngle((alpha-alpha_prime), -M_PI, M_PI);
				return (sqrt(dist*dist + ko*ko + phi*phi) + ka*fabs(r));
			}
			
		default:
			return sqrt(dist);
	}
	
}



float upo_RRT_ros::ValidityChecker::getCost(upo_RRT::State* s)
{
	
	if(get_cost_from_costmap_) {
		printf("\nERROR!! ValidityChecker. getCost from costmap is not available!!!!\n");
	}

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link"; 
	if((ros::Time::now()-time_).toSec() > 2.0)
		time_ = ros::Time::now();
	pose.header.stamp = time_; 
	//pose.header.stamp = ros::Time(0);
	pose.pose.position.x = s->getX();
	pose.pose.position.y = s->getY();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	//printf("ValidityChecker. x: %.2f, y:%.2f, th: %.2f\n", pose.pose.position.x, pose.pose.position.y, s->getYaw());
	float cost = navfeatures_->getCost(&pose);
	return cost;
	
	
}



std::vector<float> upo_RRT_ros::ValidityChecker::getFeatures(upo_RRT::State* s) 
{
	
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link"; 
	pose.header.stamp = ros::Time(0);
	pose.pose.position.x = s->getX();
	pose.pose.position.y = s->getY();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	std::vector<float> features = navfeatures_->getFeatures(&pose);
	
	return features;
}




void upo_RRT_ros::ValidityChecker::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	navfeatures_->setPeople(p);
}


void upo_RRT_ros::ValidityChecker::setWeights(std::vector<float> we) {
	navfeatures_->setWeights(we);
}


geometry_msgs::PoseStamped upo_RRT_ros::ValidityChecker::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime) {
	
	return navfeatures_->transformPoseTo(pose_in, frame_out, usetime);
	
}

bool upo_RRT_ros::ValidityChecker::isQuaternionValid(const geometry_msgs::Quaternion q) {
	
	return navfeatures_->isQuaternionValid(q);
}





