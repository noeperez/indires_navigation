
#include <upo_rrt_planners/ros/ValidityChecker3.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>

using namespace std;

upo_RRT_ros::ValidityChecker3::ValidityChecker3(tf::TransformListener* tf, float resol, int width, int height, unsigned int dimensions, int distType) : StateChecker()
{

	capture_ = new Capture();

	cm_resolution_ = resol; //m/cell
	cm_width_ = width * 2.0;	//m
	cm_height_ = height * 2.0;	//m
 
	cm_width_pixs_ = cm_width_/cm_resolution_;
	cm_height_pixs_ = cm_height_/cm_resolution_;

	//The robot position (center) from the upper-right corner
	cm_origin_.clear();
	cm_origin_.push_back(cm_width_/2.0);  //x
	cm_origin_.push_back(cm_height_/2.0); //y  //negative earlier

	costmap_.clear();
	costmap_.assign((cm_width_pixs_*cm_height_pixs_), 1.0);


	tf_ = tf;
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();

	setup();
}


/*upo_RRT_ros::ValidityChecker3::ValidityChecker3(tf::TransformListener* tf, unsigned int dimensions, int distType) : StateChecker()
{
	capture_ = new Capture();


	tf_ = tf;
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();

	//Build a default costmap
	resolution_ = 0.05; 	//m/cell
	width_ = 100;			//cells
 	height_ = 100;			//cells
	origin_.clear();
	origin_.push_back(2.5); //m
	origin_.push_back(2.5); //m
	origin_.push_back(0.0); //rad
    costmap_mutex_.lock();
	costmap_.clear();
	for(unsigned int i=0; i<(width_*height_); i++)
		costmap_.push_back((int)0);
	costmap_mutex_.unlock();

	setup();
}*/


upo_RRT_ros::ValidityChecker3::~ValidityChecker3() {
	
	delete capture_;
}


void upo_RRT_ros::ValidityChecker3::setup()
{
	ros::NodeHandle nh;
	

	ros::NodeHandle n("~/Validity_checker");
	string pc_topic;
	n.param<string>("pc_topic", pc_topic, std::string("/scan360/point_cloud3")); 
	printf("ValidityChecker3. pc_topic: %s\n", pc_topic.c_str());
	int pc_type;
	n.param<int>("pc_type", pc_type, 2); //1->PointCloud, 2->PointCloud2
	printf("ValidityChecker3. pc_type: %i\n", pc_type);

 	n.param<bool>("project_onto_map", project_onto_map_, false);

	double thres;
	n.param<double>("path_threshold", thres, 0.0);
	path_threshold_ = (float)thres;

	n.param<double>("robot_radius", insc_radius_robot_, 0.30);
	printf("ValidityChecker3. Robot Radius: %.2f\n", insc_radius_robot_);

	goal_sub_ = n.subscribe("/rrt_goal", 1, &ValidityChecker3::goalCallback, this);
	
	sub_people_ = n.subscribe("/people/navigation", 1, &ValidityChecker3::peopleCallback, this); 

 	if(pc_type == 1) 	//pointCloud
		sub_pc_ = n.subscribe(pc_topic, 1, &ValidityChecker3::pcCallback, this);
	else   				//pointCloud2
		sub_pc_ = n.subscribe(pc_topic, 1, &ValidityChecker3::pc2Callback, this);

	//Network prediction service
  	cnn_client_ = nh.serviceClient<path_prediction::PathPrediction>("path_prediction");

	costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("prediction_rewardmap", 1);

	if(project_onto_map_)
		setupStaticMap(nh);
	else
		setupNoMapProjection();

}


void upo_RRT_ros::ValidityChecker3::setupStaticMap(ros::NodeHandle nh)
{

	people_paint_area_ = 25;
	
	ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
	while (! ros::service::waitForService("/static_map",1)){
		ROS_INFO("Waiting for map service");
	}

	nav_msgs::GetMap srv;
	map_client.call(srv);
	ROS_INFO_STREAM(srv.response.map.info);
	map_image_ = cv::Mat(srv.response.map.info.height, srv.response.map.info.width,CV_8UC1, cv::Scalar(0));
	map_metadata_ =srv.response.map.info;
	map_resolution_ = (double) map_metadata_.resolution;
	map_origin_.push_back(map_metadata_.origin.position.x);
	map_origin_.push_back(map_metadata_.origin.position.y);
	map_origin_.push_back(tf::getYaw(map_metadata_.origin.orientation));
	uint8_t *myData = map_image_.data;
	for (int i=0;i<srv.response.map.data.size();i++){
		if (srv.response.map.data.at(i)==100  || srv.response.map.data.at(i)==-1 ){
		}
		else {
			map_image_.data[i] = 255;
		}
	}

	dt_mutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	dt_mutex_.unlock();
}


void upo_RRT_ros::ValidityChecker3::setupNoMapProjection()
{
	people_paint_area_ = 25;
	
	map_image_ = cv::Mat(cm_width_pixs_, cm_height_pixs_, CV_8UC1, cv::Scalar(255));

	map_metadata_.resolution = cm_resolution_;
	map_metadata_.width = cm_width_pixs_;   //Cells
	map_metadata_.height = cm_height_pixs_; //Cells
	geometry_msgs::Pose orig;
	orig.position.x = cm_origin_[0];
	orig.position.y = cm_origin_[1];
	orig.position.z = 0.0;
	orig.orientation = tf::createQuaternionMsgFromYaw(cm_origin_[2]);
	map_metadata_.origin = orig; //m

	dt_mutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	//cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	dt_mutex_.unlock();
 
}





//Update map with point cloud data
void upo_RRT_ros::ValidityChecker3::updateDistTransform(){
	
	sensor_msgs::PointCloud temp_pt_cloud;
	pc_mutex_.lock();  
	sensor_msgs::PointCloud2 lcloud = laser_cloud_;
	pc_mutex_.unlock();
	
	if(lcloud.data.size() <= 0) {
		ROS_WARN("No cloud updated");
		return;
	}
	
	bool done = sensor_msgs::convertPointCloud2ToPointCloud(lcloud, temp_pt_cloud);
	if(!done) 
		ROS_ERROR("\n\nUpdateDistTransform. convertPointCloud2toPoingCloud!!!!!\n");
	
	//Add the laser readings (pointcloud) to the map
	dt_mutex_.lock();
	cv::Mat map_copy = map_image_.clone();
	dt_mutex_.unlock();
	for (int i =0;i<temp_pt_cloud.points.size();i++){
		vector<int> pix;
		if(project_onto_map_)
			pix = worldToMap(&(temp_pt_cloud.points[i]),&map_metadata_);
		else
			pix = BaseLinkWorldToImg(&(temp_pt_cloud.points[i]));

		if(pix[0] >= 0.0 && pix[0] < map_metadata_.width && pix[1] >= 0.0 && pix[1] < map_metadata_.height) 
			map_copy.at<unsigned char>(pix[1],pix[0]) = 0;	
		//else
			//ROS_WARN("\n\nNAV FEATURES. UPDATEDT. pixel out of range!\n");
	}
	
	//Remove the people detected from the map
	/*people_mutex_.lock();
	std::vector<upo_msgs::PersonPoseUPO> per = people_;
	people_mutex_.unlock();
	for (int i=0; i<per.size(); i++){
		geometry_msgs::Point32 temp_point;
		temp_point.x = per[i].position.x;
		temp_point.y = per[i].position.y;
		vector<int> pix;
		if(project_onto_map_)
			pix = worldToMap(&temp_point,&map_metadata_);
		else
			pix = BaseLinkWorldToImg(&temp_point);

		
		if (pix[0] >= 0.0 && pix[0] < map_metadata_.width && pix[1] >= 0.0 && pix[1] < map_metadata_.height)
		{
			for (int j = -floor(people_paint_area_/2);j<ceil(people_paint_area_/2);j++){
				for (int k = -floor(people_paint_area_/2);k<ceil(people_paint_area_/2);k++){
					if (pix[1]+k>=0 && pix[0]+j>=0){
						map_copy.at<unsigned char>(pix[1]+k,pix[0]+j) =255;
					}
				}

			}
		}
	}*/
	
	cv::Mat dt;
	try{
		cv::distanceTransform(map_copy, dt, CV_DIST_L1, 3);
	} catch(...){
		ROS_ERROR("\n\nValidityChecker3. UpdateDistTrans. cv::distanceTransform!!!!!\n");
	} 

	dt_mutex_.lock();
	distance_transform_ = dt.clone();
	dt_mutex_.unlock(); 
	
	//gettimeofday(&stop, NULL);
	//printf("took %lu\n", stop.tv_usec - start.tv_usec);
	imwrite(ros::package::getPath("upo_rrt_planners").append("/test_write.jpg"), map_copy);
}



//People callback
void upo_RRT_ros::ValidityChecker3::peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg) 
{
	ROS_INFO_ONCE("People received!!");
	people_mutex_.lock();
	people_ = msg->personPoses;
	/*if(people_.size() > 0) {
		people_frame_id_ = msg->header.frame_id;
	}*/
	people_mutex_.unlock();
	
}



//Point_cloud2 callback
void upo_RRT_ros::ValidityChecker3::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& pc_in){
	
	setObstacles(*pc_in);
}


//Point_cloud callback
void upo_RRT_ros::ValidityChecker3::pcCallback(const sensor_msgs::PointCloud::ConstPtr& pc_in){
	
	sensor_msgs::PointCloud2 pc2;
	bool ok = sensor_msgs::convertPointCloudToPointCloud2(*pc_in, pc2);
	if(!ok) {
		ROS_WARN("NavFeatures. Error transforming pointCloud to pointCloud2");
	}
	
	setObstacles(pc2);
	
}



//Goal callback
void upo_RRT_ros::ValidityChecker3::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO_ONCE("Goal received!!");
	setGoal(*msg);
}



void upo_RRT_ros::ValidityChecker3::setObstacles(sensor_msgs::PointCloud2 obs)
{
	ROS_INFO_ONCE("Obstacles received!\n");
	sensor_msgs::PointCloud2 lcloud;
	obs.header.stamp = ros::Time();
	try{  
		if(project_onto_map_) {
			if(!pcl_ros::transformPointCloud("/map", obs, lcloud, *tf_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		} else {
			if(!pcl_ros::transformPointCloud("/base_link", obs, lcloud, *tf_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		}			

	} catch (tf::TransformException ex){
		ROS_WARN("NAV FEATURES. pcCallback. TransformException: %s", ex.what());
	}
	
	pc_mutex_.lock();
	laser_cloud_ = lcloud;
	pc_mutex_.unlock();
}




//Publish the path planning prediction into a ros costmap for visualization
void upo_RRT_ros::ValidityChecker3::publish_costmap_ros(ros::Time t)
{
	//Get the robot coordinates in odom frame
	tf::StampedTransform transform;
	try{
		tf_->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
		tf_->lookupTransform("/odom", "/base_link",  ros::Time(0), transform); //t
	}
	catch (tf::TransformException ex){
		ROS_ERROR("Publish_feature_costmap. TF exception: %s",ex.what());
	}
	  
	nav_msgs::OccupancyGrid cmap;
	cmap.header.frame_id = "odom"; //"base_link";
	cmap.header.stamp = ros::Time::now(); //t;
	//time map_load_time. The time at which the map was loaded
	cmap.info.map_load_time = cmap.header.stamp; //t;
	//float32 resolution. The map resolution [m/cell]
	cmap.info.resolution = cm_resolution_;  //0.25 m/cell
	//uint32 width. Map width [cells]
	cmap.info.width = cm_height_pixs_; //x
	//uint32 height. Map height [cells]
	cmap.info.height = cm_width_pixs_; //y
	//geometry_msgs/Pose origin. The origin of the map [m, m, rad].  This is the real-world pose of the
	// cell (0,0) in the map.
	geometry_msgs::Pose p;
	p.position.x = transform.getOrigin().x()-(cm_height_/2.0); //x
	p.position.y = transform.getOrigin().y()-(cm_width_/2.0);  //y (substraction initially)
	p.position.z = 0.0;
	p.orientation = tf::createQuaternionMsgFromYaw(0.0); //robot_odom_h
	cmap.info.origin = p;
	//int8[] cmap.data. The map data, in row-major order, starting with (0,0).  Occupancy
	// probabilities are in the range [0,100].  Unknown is -1.
	std::vector<signed char> data; // size =(cmap.info.width*cmap.info.height)
	double cost = 0.0;
	//for(int i=0; i<cmap.info.height; i++)
	for(int i=(cmap.info.height-1); i>0; i--) 
	{
		for(unsigned int j=0; j<cmap.info.width; j++)
		{
			float reward = costmap_[i*cm_width_pixs_ + j]>1.0?1.0:costmap_[i*cm_width_pixs_ + j];
			//cost = (1 - reward);						
			//Transform cost into the scale[0,100]  
			data.push_back((int)round(reward*100.0)); 
		}
	}
	cmap.data = data;
	costmap_pub_.publish(cmap);
}







bool upo_RRT_ros::ValidityChecker3::predictionService(vector<int> input, int rows, int cols)
{
	path_prediction::PathPrediction srv;
  	srv.request.input = input;
  	srv.request.input_rows = rows;
	srv.request.input_cols = cols;
  	if (cnn_client_.call(srv))
  	{
    	ROS_INFO("Service path prediction called!");
		costmap_mutex_.lock();
		costmap_.clear();
		costmap_ = srv.response.prediction;
		cm_width_pixs_ = srv.response.pred_cols;
		cm_height_pixs_ = srv.response.pred_rows;
		cm_width_ = cm_width_pixs_ * cm_resolution_;
		cm_height_ = cm_height_pixs_ * cm_resolution_;
		costmap_mutex_.unlock();


		/*printf("Prediction received:\n");
		printf("Vector size: %u\n", (unsigned int)costmap_.size());
		for(unsigned int i=0; i<cm_height_pixs_; i++)
		{
			for(unsigned int j=0; j<cm_width_pixs_; j++)
				printf("%.2f ", costmap_[i*cm_width_pixs_ + j]);
			printf("\n");
		}*/


		return true;		
  	}
  	else
  	{
    	ROS_ERROR("Failed to call service path_prediction");
    	return false;
  	}

}



//Update the navigation scene and get the path planning prediction
void upo_RRT_ros::ValidityChecker3::preplanning_computations()
{
	struct timeval t_ini1, t_fin1;
	gettimeofday(&t_ini1, NULL);
	//if(project_onto_map_)
		updateDistTransform();

	//Take point cloud
	pc_mutex_.lock();
	sensor_msgs::PointCloud2 pc2 = laser_cloud_; //base_link
	pc_mutex_.unlock();
	sensor_msgs::PointCloud pc;
	bool done = sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
	if(!done)
		ROS_ERROR("\n\nPreplanning computations. convertPointCloud2toPointCloud!!!!!\n");

	//Take people
	people_mutex_.lock();
	vector<upo_msgs::PersonPoseUPO> p = people_;	//odom
	people_mutex_.unlock();

	//Take goal
	goal_mutex_.lock();
	geometry_msgs::PoseStamped g = goal_;			//base_link
	goal_mutex_.unlock();

	
	//Form current navigation scene
	vector<int> input = capture_->generateImg(&pc, &p, &g);
	gettimeofday(&t_fin1, NULL);
	double secs1 = capture_->timeval_diff(&t_fin1, &t_ini1);
  	

	/*printf("Nav scene vector generated: \n");
	for(unsigned int i=0; i<200; i++) {
		for(unsigned int j=0; j<200; j++)
			printf("%i", input[((i*200)+j)]);
		printf("\n");
	}*/

	struct timeval t_ini2, t_fin2;
	gettimeofday(&t_ini2, NULL);
	//Update the path prediction (costmap)
	predictionService(input, cm_height_pixs_, cm_width_pixs_);
	gettimeofday(&t_fin2, NULL);
	double secs2 = capture_->timeval_diff(&t_fin2, &t_ini2);
	printf("Preplanning times:\n \tGenerate input: %.3f msecs\n \tGeneratePrediction: %.3f msecs\n", (secs1*1000.0), (secs2*1000.0) );
}




vector<upo_RRT::State> upo_RRT_ros::ValidityChecker3::getPredictedPointList()
{
	vector<upo_RRT::State> points;
	costmap_mutex_.lock();
	for(unsigned int i=0; i<cm_height_pixs_; i++) {
		for(unsigned int j=0; j<cm_width_pixs_; j++) {
			if(costmap_[i*cm_width_pixs_+j] > path_threshold_) {
				vector<int> cell;
				cell.push_back(j); //x
				cell.push_back(i); //y
				vector<float> pos = costmapCellToPose(cell);
				upo_RRT::State state(pos[0], pos[1], 0.0); //x, y, yaw 
				points.push_back(state);
			}
		}
	} 
	costmap_mutex_.unlock();
	return points;
}



// Transform a point in the world to the pixels in the static map
vector<int> upo_RRT_ros::ValidityChecker3::worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata) const {
	vector<int> pixels;
	float x_map = world_point->x - map_metadata->origin.position.x;
	float y_map = world_point->y - map_metadata->origin.position.y;
	pixels.push_back((int)floor(x_map/map_metadata->resolution ));
	pixels.push_back((int)floor(y_map/map_metadata->resolution));
	return pixels;
}

/**
* transform point in base link frame (m) to pixel
*/
vector<int> upo_RRT_ros::ValidityChecker3::BaseLinkWorldToImg(geometry_msgs::Point32* point) const
{ 
	vector<int> pix;
	float wx = cm_origin_[0] + point->x;
	float wy = cm_origin_[1] + point->y; 
	pix.push_back((int)floor(wx/cm_resolution_));
	pix.push_back((int)floor(wy/cm_resolution_));
	return pix;

}


vector<int> upo_RRT_ros::ValidityChecker3::poseToCostmapCell(vector<float> pose) const
{
	//Be careful, I'm not taking into account the rotation because is zero usually.
	//Pose is in robot frame coordinates, and the robot is centered in the costmap
	int x =  floor(fabs((pose[0] + cm_origin_[0])/cm_resolution_)); //x
	int y =  floor(fabs((pose[1] - cm_origin_[1])/cm_resolution_)); //y
 	vector<int> cell;
	cell.push_back(x);
	cell.push_back(y);
	return cell;
}



vector<float> upo_RRT_ros::ValidityChecker3::costmapCellToPose(vector<int> cell) const
{
	//Be careful, I'm not taking into account the rotation because is zero usually.
	float x = (cell[0]*cm_resolution_) - cm_origin_[0]  + (cm_resolution_/2.0);  //-
	float y = (-1)*((cell[1]*cm_resolution_) - cm_origin_[1]  + (cm_resolution_/2.0)); //+
	vector<float> pose;
	pose.push_back(x);
	pose.push_back(y);
	return pose;
}


//This checking is done over the static map (updated with the point cloud or not)
bool upo_RRT_ros::ValidityChecker3::isValid(upo_RRT::State* s) const
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

	//Transform the coordinates
	float res;
	vector<int> pix;
	geometry_msgs::PoseStamped sm;
	if(project_onto_map_) {
		res = map_metadata_.resolution;
		sm = transformPoseTo(&p_in, string("/map"), false);
		geometry_msgs::Point32 point;
		point.x = sm.pose.position.x;
		point.y = sm.pose.position.y;
		point.z = 0.0;
		//pix = worldToMap(&point, &map_metadata_);
		float x_map = point.x - map_metadata_.origin.position.x;
		float y_map = point.y - map_metadata_.origin.position.y;
		pix.push_back((int)floor(x_map/map_metadata_.resolution));
		pix.push_back((int)floor(y_map/map_metadata_.resolution));

	}else {
		res = cm_resolution_;
		//sm = transformPoseTo(&p_in, string("/base_link"), false);
		geometry_msgs::Point32 point;
		point.x = s->getX(); //sm.pose.position.x;
		point.y = s->getY(); //sm.pose.position.y;
		point.z = 0.0;
		pix = BaseLinkWorldToImg(&point);
	}

	float px = pix[0];
	float py = pix[1];
	float distance = 0.0;
	//dt_mutex_.lock();
	if (py<0 || px<0  || px > map_image_.cols || py > map_image_.rows) {
		distance = 0.0;
	} else{
		try{
			
			distance = distance_transform_.at<float>(py,px)*res;
			
		} catch(...)
		{
			ROS_ERROR("ERROR. IS_VALID. px:%.2f, py:%.2f, distance:%.2f", px, py, distance);
			distance = 0.0;
		}
	}
	//dt_mutex_.unlock();
	// Take into account the robot radius 
	if(distance <= insc_radius_robot_) {
		return false;
	}else {
		
		return true;
	}

}



float upo_RRT_ros::ValidityChecker3::distance(upo_RRT::State* s1, upo_RRT::State* s2) const
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



float upo_RRT_ros::ValidityChecker3::getCost(upo_RRT::State* s)
{
	//cell.first = x = column, cell.second = y = row
	vector<float> pose;
	pose.push_back((float)s->getX());
	pose.push_back((float)s->getY());
	costmap_mutex_.lock();
	vector<int> cell = poseToCostmapCell(pose);
	//vector<int> cell = BaseLinkWorldToImg(&p);
	int ind = (cell[1]*cm_width_pixs_) + cell[0]; 
	//if(ind < 0 || ind >= (200*200))
	//	printf("GetCost. Ind out of range: %i\n", ind); 
	float reward = costmap_[ind]>1.0?1.0:costmap_[ind];
	//if(reward == 0.0)
	//	return 10.0;
	//printf("r: %.2f\t", reward);
	costmap_mutex_.unlock();
	return (1-reward); //*1000.0 //Value between [0,1]
}



geometry_msgs::PoseStamped upo_RRT_ros::ValidityChecker3::transformPoseTo(geometry_msgs::PoseStamped* pose_in, std::string frame_out, bool usetime) const {
	
	geometry_msgs::PoseStamped in = *pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
			
	geometry_msgs::PoseStamped pose_out;
		
	try {
		tf_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("ValidityChecker3. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	return pose_out;
	
}

bool upo_RRT_ros::ValidityChecker3::isQuaternionValid(const geometry_msgs::Quaternion q) {
	
	
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





