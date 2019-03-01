
#include <upo_rrt_planners/ros/capture.h>

using namespace std;




Capture::Capture(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal)
{
	setScenario(obs, people, goal);
	setup();
	
}


Capture::Capture()
{

	//tf_listener_ = new tf::TransformListener(ros::Duration(10));
	setup();
}


Capture::~Capture()
{
}


void Capture::setScenario(sensor_msgs::PointCloud* obs, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal) {
	
	//Take all the trajectory data
	obs_ = obs;
	people_ = people;
	goal_ = goal;
	//agent_ = agent;

	//TransformToLocal(pc_, people_, goal_, agent_);
}




void Capture::setup()
{
	
	goal_ok_ = false;

	ros::NodeHandle n("~/Capture");

	n.param("use_static_map", use_static_map_, false);

	n.param("save_img", save_img_, false);
	save_count_ = 1;
	save_path_ = ros::package::getPath("upo_rrt_planners");

	
	//Add the demo path
	n.param("add_demo_path", add_demo_path_, false);
	//if add_demo_path is true
	// we can chose whether to draw only the path or path+scenario
    n.param("only_path_label", only_path_label_, false);

	//With and height in pixels
	n.param("img_pixels", px_height_, 400);

	//Image resolution m/pix
	n.param("img_resolution", resolution_, (float)0.025);

    n.param("use_greyscale", greyscale_, false);

	n.param("add_people_orientation", people_orientation_, false);

	px_width_ = px_height_; //400; //400px*0.025m/px = 10m
	px_origin_ = make_pair((int)px_width_/2, (int)px_height_/2);
	w_origin_ = make_pair(px_origin_.first*resolution_, px_origin_.second*resolution_);

	cout << "---- Capture navigation images ----" << endl;
	cout << "px_width: " << px_width_ << " px" << endl;
	cout << "px_height: " << px_height_ << " px" << endl;
	cout << "px_origin x: " << px_origin_.first << " px" << endl;
	cout << "px_origin y: " << px_origin_.second << " px" << endl;
	cout << "resolution: " << resolution_ << " m/px" << endl;
	cout << "w_origin x: " << w_origin_.first << " m" << endl;
	cout << "w_origin y: " << w_origin_.second << " m" << endl;
	cout << "-----------------------------------" << endl;


}



/** 
* Get current date/time, format is YYYY-MM-DD.HH:mm:ss
*/
const string Capture::currentDateTime() {
    time_t     now = std::time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}




/**
* transform world point (m) to pixel
*/
pair<int,int> Capture::worldToImg(geometry_msgs::Point32* world_point)
{ 
	float wx = w_origin_.first + world_point->x;
	float wy = w_origin_.second - world_point->y;
	pair<int,int> pix = make_pair((int)floor(wx/resolution_),(int)floor(wy/resolution_));
	return pix;
}




/**
* Add the laser readings (obstacles) to the image
*/
bool Capture::addObstacles(cv::Mat* img)
{
	/*sensor_msgs::PointCloud temp_pc;

	//Transform pointCloud2 to pointCloud
	bool done = sensor_msgs::convertPointCloud2ToPointCloud(*pc_, temp_pc);
	if(!done) { 
		ROS_ERROR("\n\nGenerateImg. ERROR in convertPointCloud2toPoingCloud!!!!!\n");
		return false;
	}*/

	cv::Vec3b intensity;
	intensity.val[0] = 0; //blue
	intensity.val[1] = 255; //green
	intensity.val[2] = 0; //red

    uchar intensity_grey = 204;

	//printf("\naddObstacles!!! points.size: %u\n\n", (unsigned int)obs_->points.size());

	for(int i =0;i<obs_->points.size();i++){
		pair<int,int> pix = worldToImg(&obs_->points[i]); 
		//printf("pix x: %i, y: %i\n", pix.first, pix.second);
		if(pix.first >= 0 && pix.first < px_width_ && pix.second >= 0 && pix.second < px_height_) {
			if(greyscale_) {
				img->at<uchar>(pix.second, pix.first) = intensity_grey;
			} else
				img->at<cv::Vec3b>(pix.second, pix.first) = intensity;
		}
	}

	return true;
}



/*bool Capture::draw_half_circle(cv::Mat* img, radius, start_angle):
    height, width = img.shape[0:2]
    // Ellipse parameters
    radius = 100
    center = (width / 2, height - 25)
    axes = (radius, radius)
    angle = 0
    startAngle = 180
    endAngle = 360
    thickness = 10

    # http://docs.opencv.org/modules/core/doc/drawing_functions.html#ellipse
    cv2.ellipse(image, center, axes, angle, startAngle, endAngle, BLACK, thickness)
*/


float Capture::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}



bool Capture::draw_triangle(cv::Mat* img, geometry_msgs::Point32 center, float orientation, float radius)  
{
	int lineType = 8;

	cv::Point points[1][3];
	//Point 1
	float ori1 = orientation+(M_PI/2.0);
	geometry_msgs::Point32 p1;
	p1.x = center.x + radius*cos(normalizeAngle(ori1,-M_PI, M_PI));
	p1.y = center.y + radius*sin(normalizeAngle(ori1, -M_PI, M_PI));
	pair<int,int> pix1 = worldToImg(&p1);
	points[0][0] = cv::Point(pix1.first, pix1.second);
	//Point 2
	float ori2 = orientation-(M_PI/2.0);
	geometry_msgs::Point32 p2;
	p2.x = center.x + radius*cos(normalizeAngle(ori2,-M_PI, M_PI));
	p2.y = center.y + radius*sin(normalizeAngle(ori2, -M_PI, M_PI));
	pair<int,int> pix2 = worldToImg(&p2);
	points[0][1] = cv::Point(pix2.first, pix2.second);

	//Point 3
    geometry_msgs::Point32 p3;
	p3.x = center.x + (radius+0.15)*cos(orientation);
	p3.y = center.y + (radius+0.15)*sin(orientation);
	pair<int,int> pix3 = worldToImg(&p3);
	points[0][2] = cv::Point(pix3.first, pix3.second);


	const cv::Point* ppt[1] = { points[0] };
  	int npt[] = { 3 };

	if(greyscale_)
  		cv::fillPoly(*img, ppt, npt, 1, cv::Scalar(153), lineType );
	else
		cv::fillPoly(*img, ppt, npt, 1, cv::Scalar( 255, 255, 0), lineType );

	return true;
}


/**
* Add the people to the image
*/
bool Capture::addPeople(cv::Mat* img)
{
	float radius = 0.3; //m //0.3m/0.025m/px = 12 px
	int thickness = -1;
	int lineType = 8;

	for (int i=0; i<people_->size(); i++){
		//Transform people to base_link
		//geometry_msgs::PoseStamped p;
		//p.header = people_->at(i).header;
		//p.pose.position = people_->at(i).position;
		//p.pose.orientation = people_->at(i).orientation;
		//p = transformPoseTo(p, "base_link", true);
		geometry_msgs::Point32 center;
		center.x = people_->at(i).position.x;  //p.pose.position.x;
		center.y = people_->at(i).position.y;  //p.pose.position.y;
		pair<int,int> pix = worldToImg(&center);
		if(pix.first >= 0 && pix.first < px_width_ && pix.second >= 0 && pix.second < px_height_) {
			if(greyscale_)
				cv::circle(*img, cv::Point(pix.first, pix.second), radius/resolution_, cv::Scalar(102), thickness, lineType); 
			else
				cv::circle(*img, cv::Point(pix.first, pix.second), radius/resolution_, cv::Scalar(0,0,255), thickness, lineType); //Red

			if(people_orientation_) {
				//Add a line with the orientation
				//geometry_msgs::Point32 p2;
				//p2.x = center.x + radius*cos(tf::getYaw(people_->at(i).orientation));
				//p2.y = center.y + radius*sin(tf::getYaw(people_->at(i).orientation));
				//pair<int,int> pix2 = worldToImg(&p2);
				//if(greyscale_) {
					//cv::line(*img, cv::Point(pix.first, pix.second), cv::Point(pix2.first, pix2.second), cv::Scalar(0), 2);  //black greyscale
				//} else {
					//cv::line(*img, cv::Point(pix.first, pix.second), cv::Point(pix2.first, pix2.second), cv::Scalar(255,255,0), 2);  //cyan
				//}
				draw_triangle(img, center, tf::getYaw(people_->at(i).orientation), radius);
			}
		}	
	}
	return true;
}


/**
* Add the goal to the image
*/
bool Capture::addGoal(cv::Mat* img)
{
	goal_ok_ = false;
	float radius = 0.20; //m 
	int thickness = -1;
	int lineType = 8;
	//geometry_msgs::PoseStamped p = transformPoseTo(*goal_, "base_link", true);
	geometry_msgs::Point32 center;
	center.x = goal_->pose.position.x;   //p.pose.position.x;
	center.y = goal_->pose.position.y;   //p.pose.position.y;
	pair<int,int> pix = worldToImg(&center);
	if(pix.first >= 0 && pix.first < px_width_ && pix.second >= 0 && pix.second < px_height_) {
		if(greyscale_)
			cv::circle(*img, cv::Point(pix.first, pix.second), radius/resolution_, cv::Scalar(63), thickness, lineType); //greyscale
		else
			cv::circle(*img, cv::Point(pix.first, pix.second), radius/resolution_, cv::Scalar(255,0,0), thickness, lineType); //Blue
		goal_ok_ = true;
	}
	return true;
}


/**
* Add the demo path to the image
*/
bool Capture::addPath(cv::Mat* img)
{
	unsigned int lastPoint=0;
	for (unsigned int i=0; i<(agent_->size()-1); i++){
		//geometry_msgs::PoseStamped p = transformPoseTo(path_->at(i), "base_link", true);
		geometry_msgs::Point32 p1;
		p1.x = agent_->at(i).pose.position.x;   //p.pose.position.x;
		p1.y = agent_->at(i).pose.position.y;   //p.pose.position.y;
		pair<int,int> pix1 = worldToImg(&p1);
		//p = transformPoseTo(path_->at(i+1), "base_link", true);
		geometry_msgs::Point32 p2;
		p2.x = agent_->at(i+1).pose.position.x; //p.pose.position.x;
		p2.y = agent_->at(i+1).pose.position.y;   //p.pose.position.y;
		pair<int,int> pix2 = worldToImg(&p2);
		if(pix2.first >= 0 && pix2.first < px_width_ && pix2.second >= 0 && pix2.second < px_height_) {
			if(greyscale_)
				cv::line(*img, cv::Point(pix1.first, pix1.second), cv::Point(pix2.first, pix2.second), cv::Scalar(255), 3); //greyscale
			else
				cv::line(*img, cv::Point(pix1.first, pix1.second), cv::Point(pix2.first, pix2.second), cv::Scalar(255,255,255), 3); //white
			lastPoint = i+1;
		}
	}

	// We check if the path is outside the boundaries of the image.
	// In that case, we cut the path and take the last point as the goal
	if(lastPoint < (agent_->size()-1) && !goal_ok_){
		goal_->header = agent_->at(lastPoint).header;
		goal_->pose = agent_->at(lastPoint).pose;
	}

	return true;
}




/**
* Generate the image
*/
std::vector<int> Capture::generateImg(sensor_msgs::PointCloud* pc, vector<upo_msgs::PersonPoseUPO>* people, geometry_msgs::PoseStamped* goal)
{
	setScenario(pc, people, goal);
	cv::Mat imgmat = generateImg();

	if(save_img_){
		char buf[10];
		sprintf(buf, "input_%u", save_count_);
		string name = string(buf);
		string filename = save_path_ + "/saved_input/" + name + ".jpg";
		imwrite(filename.c_str(), imgmat);
		save_count_++;
	}

	//cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
	//cv::imshow( "Display window", imgmat ); 
	//cv::waitKey(0);

	/*printf("Capture. Generating navigation image:\n");
    //std::vector<int> img_array;
 	//Transform mat to 1D array
	for(int i=0; i < imgmat.rows; i++) {
    	for (int j =0; j < imgmat.cols; j++){
                //img_array.push_back((int)imgmat.at<uchar>(i,j));
				printf("%u", imgmat.at<uchar>(i,j));
		}
		printf("\n");
	}*/

	std::vector<int> img_array;
	if (imgmat.isContinuous()) {
  		img_array.assign(imgmat.datastart, imgmat.dataend);
	} else {
  		for (int i = 0; i < imgmat.rows; ++i) {
    		img_array.insert(img_array.end(), imgmat.ptr<uchar>(i), imgmat.ptr<uchar>(i)+imgmat.cols);
  		}
	}

	return img_array; 
}


/**
* Generate and store the image
*/
cv::Mat Capture::generateImg()
{
	cv::Mat img;
	//if(use_static_map)
	//{
	//	img = map_image_.clone();
	//} else {
	
	// Create OpenCV mat image
	if(greyscale_)
		img = cv::Mat(px_width_, px_height_, CV_8UC1, cv::Scalar(0));	//greyscale (black)
	else
		img = cv::Mat(px_width_, px_height_, CV_8UC3, cv::Scalar::all(0));  //black

	//0. Draw robot axes in the center
	/*float longitude = 0.5; //m
	pair<int,int> p2;
	p2.first = px_origin_.first + (longitude/resolution_);
	p2.second = px_origin_.second; 
	cv::line(img, cv::Point(px_origin_.first, px_origin_.second), cv::Point(p2.first, p2.second), cv::Scalar(0,0,255), 3);	
	pair<int,int> p3;
	p3.first = px_origin_.first;
	p3.second = px_origin_.second-(longitude/resolution_);	
	cv::line(img, cv::Point(px_origin_.first, px_origin_.second), cv::Point(p3.first, p3.second), cv::Scalar(0,255,0), 3);	*/
	//Draw the robot position
	//float radius = 0.1; //m 
	//int thickness = -1;
	//int lineType = 8;
	//cv::circle(img, cv::Point(px_origin_.first, px_origin_.second), radius/resolution_, cv::Scalar(255,255,255), thickness, lineType); //white


	//1. Add the laser readings (pointcloud) to the image
	addObstacles(&img);

	//2. Add the people detected to the image
	addPeople(&img);

	//3. Add the goal to the image
	addGoal(&img);

	return img;

	//5. Write file image
	//string time = currentDateTime();	
	//string filename = store_dir_ + name + ".jpg";
	//imwrite(filename.c_str(), img);
	//cout << "Capture " << filename << " stored!!!" << endl; 

	//6. If the demonstration path too
	/*if(add_demo_path_) {
		filename = store_dir_ + "labels/" + name + ".jpg";
		if(only_path_label_)
		{
			cv::Mat img2;
			if(greyscale_)
				img2 = cv::Mat(px_width_, px_height_, CV_8UC1, cv::Scalar(0));	//greyscale
			else
				img2= cv::Mat(px_width_, px_height_, CV_8UC3, cv::Scalar::all(0));  //black
			addPath(&img2);
			imwrite(filename.c_str(), img2);
		} else {
			addPath(&img);
			imwrite(filename.c_str(), img);
		}
		cout << "Capture " << filename << " stored!!!" << endl;
	}*/
}



