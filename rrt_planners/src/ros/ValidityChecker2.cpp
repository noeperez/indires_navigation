
#include <upo_rrt_planners/ros/ValidityChecker2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>


upo_RRT_ros::ValidityChecker2::ValidityChecker2(nav_msgs::OccupancyGrid* costmap, tf::TransformListener* tf, unsigned int dimensions, int distType) : StateChecker()
{
    costmap_mutex_.lock();
	costmap_.clear();
	for(unsigned int i=0; i<costmap->data.size(); i++)
		costmap_.push_back((int)costmap->data[i]);

	resolution_ = (float)costmap->info.resolution; 	//m/cell
	width_ = costmap->info.width;					//cells
 	height_ = costmap->info.height;					//cells
	origin_.clear();
	origin_.push_back(costmap->info.origin.position.x); //m
	origin_.push_back(costmap->info.origin.position.y); //m
	origin_.push_back(tf::getYaw(costmap->info.origin.orientation)); //rad
	costmap_mutex_.unlock();

	tf_ = tf;
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();
}


upo_RRT_ros::ValidityChecker2::ValidityChecker2(tf::TransformListener* tf, unsigned int dimensions, int distType) : StateChecker()
{
	tf_ = tf;
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();

	//Build a default costmap
	resolution_ = 0.05; 	//m/cell
	width_ = 100;					//cells
 	height_ = 100;					//cells
	origin_.clear();
	origin_.push_back(2.5); //m
	origin_.push_back(2.5); //m
	origin_.push_back(0.0); //rad
    costmap_mutex_.lock();
	costmap_.clear();
	for(unsigned int i=0; i<(width_*height_); i++)
		costmap_.push_back((int)0);
	costmap_mutex_.unlock();
}


upo_RRT_ros::ValidityChecker2::~ValidityChecker2() {
	
	//delete navfeatures_;
}


void upo_RRT_ros::ValidityChecker2::updateCostmap(nav_msgs::OccupancyGrid* costmap)
{
	costmap_mutex_.lock();
	costmap_.clear();
	for(unsigned int i=0; i<costmap->data.size(); i++)
		costmap_.push_back((int)costmap->data[i]);
	
	resolution_ = (float)costmap->info.resolution;
	width_ = costmap->info.width;
 	height_ = costmap->info.height;
	origin_.clear();
	origin_.push_back(costmap->info.origin.position.x);
	origin_.push_back(costmap->info.origin.position.y);
	origin_.push_back(tf::getYaw(costmap->info.origin.orientation));
	costmap_mutex_.unlock();
}







std::vector<int> upo_RRT_ros::ValidityChecker2::poseToCell(std::vector<float> pose) const
{
	//Be careful, I'm not taking into account the rotation because is zero usually.
	//Pose is in robot frame coordinates, and the robot is centered in the costmap
	int x =  fabs(origin_[0])/resolution_ + pose[0]/resolution_;
	int y =  fabs(origin_[1])/resolution_ + pose[1]/resolution_;
 	std::vector<int> cell;
	cell.push_back(x);
	cell.push_back(y);
	return cell;
}

std::vector<float> upo_RRT_ros::ValidityChecker2::cellToPose(std::vector<int> cell) const
{
	//Be careful, I'm not taking into account the rotation because is zero usually.
	float x = (resolution_*cell[1] - origin_[0]); // + (resolution_/2.0); 
	float y = (resolution_*cell[0] - origin_[1]); // + (resolution_/2.0);
	std::vector<float> pose;
	pose.push_back(x);
	pose.push_back(y);
	return pose;
}



bool upo_RRT_ros::ValidityChecker2::isValid(upo_RRT::State* s) const
{
	std::vector<float> pose;
	pose.push_back((float)s->getX());
	pose.push_back((float)s->getY());
	//costmap_mutex_.lock();
	std::vector<int> cell = poseToCell(pose); 
 	//printf("IsValid cell[%i][%i]\n", cell[0],cell[1]);
	int ind = cell[0]*200 + cell[1];
    //printf("Indice: %i\n", ind);
	float cost = costmap_[ind];
	//costmap_mutex_.unlock();

	if(cost == -1 || cost > 100) //cost == 100
		return false;
	else
		return true;
	
}


void upo_RRT_ros::ValidityChecker2::preplanning_computations()
{
}


float upo_RRT_ros::ValidityChecker2::distance(upo_RRT::State* s1, upo_RRT::State* s2) const
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
				beta = normalizeAngle(beta, -M_PI, M_PI);
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
				phi = normalizeAngle(phi, -M_PI, M_PI);
				float ka = 0.5;
				float ko = ka/8.0;
				dist = sqrt(dist);
				// two options
				float alpha_prime = atan(-ko*phi);
				//float alpha_prime = atan(-ko*ko * phi/(dist*dist));
				float r = normalizeAngle((alpha-alpha_prime), -M_PI, M_PI);
				return (sqrt(dist*dist + ko*ko + phi*phi) + ka*fabs(r));
			}
			
		default:
			return sqrt(dist);
	}
	
}



float upo_RRT_ros::ValidityChecker2::getCost(upo_RRT::State* s)
{
	//cell.first = x = column, cell.second = y = row
	std::vector<float> pose;
	pose.push_back((float)s->getX());
	pose.push_back((float)s->getY());
	costmap_mutex_.lock();
	std::vector<int> cell = poseToCell(pose);
	int ind = (cell[1]+1) * cell[0]; //row*column
	float cost = costmap_[ind];
	costmap_mutex_.unlock();
	return cost;

}




/*void upo_RRT_ros::ValidityChecker::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	navfeatures_->setPeople(p);
}


void upo_RRT_ros::ValidityChecker::setWeights(std::vector<float> we) {
	navfeatures_->setWeights(we);
}*/


geometry_msgs::PoseStamped upo_RRT_ros::ValidityChecker2::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime) {
	
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
			
	geometry_msgs::PoseStamped pose_out;
		
	try {
		tf_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("ValidityChecker2. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	return pose_out;
	
}

bool upo_RRT_ros::ValidityChecker2::isQuaternionValid(const geometry_msgs::Quaternion q) {
	
	
		//first we need to check if the quaternion has nan's or infs
		if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
			ROS_ERROR("Quaternion has infs!!!!");
			return false;
		}
		if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
			ROS_ERROR("Quaternion has nans !!!");
			return false;
		}
		
		if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
			ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
			return false;
		}

		tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

		//next, we need to check if the length of the quaternion is close to zero
		if(tf_q.length2() < 1e-6){
		  ROS_ERROR("Quaternion has length close to zero... discarding.");
		  return false;
		}

		//next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
		tf_q.normalize();

		tf::Vector3 up(0, 0, 1);

		double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

		if(fabs(dot - 1) > 1e-3){
		  ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
		  return false;
		}

		return true;
	
}





