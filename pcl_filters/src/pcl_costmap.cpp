
#include <iostream>
#include <math.h>
#include <mutex>
#include <time.h>

//PCL
/*
//This solves the problem of compiling pcl search with C++11
#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE 
#include <pcl/impl/instantiate.hpp> 
#include <pcl/point_types.h> 
PCL_INSTANTIATE(Search, PCL_POINT_TYPES) 
#endif // PCL_NO_PRECOMPILE
*/
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

//PCL filters
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
//the shadow filter requires to compute the normals previously
#include <pcl/filters/shadowpoints.h>

//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>




//Filters
pcl::VoxelGridCovariance<pcl::PointXYZ>* vgc;
bool voxelgridcov = false;
pcl::VoxelGrid<pcl::PointXYZ>* vg;
bool voxelgrid = false;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>* sor;
bool statistical = false;
pcl::PassThrough<pcl::PointXYZ>* pass;
bool passthrough = false;
float pass_cx = 0.0;
float pass_cy = 0.0;
float pass_cz = 0.0;
float pass_rx = 100.0;
float pass_ry = 100.0;
float pass_rz = 100.0;
pcl::CropBox<pcl::PointXYZ>* box;
bool cropbox = false;

std::string pc_frame;
bool use_robot_pose = false;
bool filter_ceiling = false;
float local_grid_radius = 3.0;
float max_height_allowed = 2.0;
std::string robot_pose_topic;
std::string odom_frame;
std::string sensor_frame;
geometry_msgs::PoseStamped robot_pose;
std::mutex mutex; 

bool print_arrows = false;

double pitch_max = M_PI/2.0;
double roll_max = M_PI/2.0;
double roughness = 0.03;


double pitch_low;  
double pitch_high;
double roll_low; 
double roll_high;

bool assign_costs = true;


ros::Publisher pc_pub;
ros::Publisher arrow_pub;

tf::TransformListener *listener;










bool isQuaternionValid(const geometry_msgs::Quaternion q){
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
    /*tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }*/

    return true;
}




geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime)
{
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time(); //ros::Time::now(); //ros::Time();
		
	geometry_msgs::PoseStamped pose_out;
	
	geometry_msgs::Quaternion q = in.pose.orientation;
	if(!isQuaternionValid(q))
	{
		ROS_WARN("pcl_filters. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	try {
		listener->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("pcl_filters. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	return pose_out;
}





pcl::PointCloud<pcl::PointXYZ> removeWallsAndCeiling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	visualization_msgs::MarkerArray marray;
	pcl::PointCloud<pcl::PointXYZ> filtered;
	filtered.header = cloud->header;
	filtered.is_dense = cloud->is_dense;
	filtered.height = cloud->height;
	filtered.sensor_origin_ = cloud->sensor_origin_;
	filtered.sensor_orientation_ = cloud->sensor_orientation_;

	const std::map< size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf >* leaves = &(vgc->getLeaves());

	if(!cloud->isOrganized()) //The height value must be different than 1 for a dataset to be organized
	{
		//the index of the width should match with the leaf index???
		pcl::IndicesPtr indices(new std::vector<int>); //typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr
		//std::vector<int> indices2;

		int ind=0;
		for(const auto& it : *leaves) //const 
		{
			const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *l = &(it.second);

			Eigen::Vector3d mean = l->getMean();
			Eigen::Matrix3d evecs = l->getEvecs();
			Eigen::Vector3d evals = l->getEvals();

			pcl::PointXYZ p;
			p.x = mean[0];
			p.y = mean[1];
			p.z = mean[2];
		
		
			Eigen::Quaternion<double> q(evecs);

			auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2); //0->yaw, 1->pitch, 2->roll
			double yaw = euler[0]; 
			double pitch = euler[1]; 
			double roll = euler[2];


			//Identify the lowest eigenvalue
			//TODO: the eigen values are ordered. 
			//The smallest is the last one. CHECK THIS! 
			int i=0;
			if(evals(0) < evals(1) && evals(0) < evals(2))
				i = 0;
			else if(evals(1) < evals(0) && evals(1) < evals(2))
				i = 1;
			else 
				i = 2;  
			
			float nx = (float)evecs(0,i);
			float ny = (float)evecs(1,i);
			float nz = (float)evecs(2,i);

			
			
			//create a pose
			//geometry_msgs::PoseStamped pose;
			//pose.header.frame_id = odom_frame;
			//pose.header.stamp = ros::Time::now();
			tf::Quaternion aux(q.x(), q.y(), q.z(), q.w());
			aux.normalize();
			//pose.pose.position.x = mean[0];
			//pose.pose.position.y = mean[1];
			//pose.pose.position.z = mean[2];
			
			//pose.pose.orientation.x = aux.x();
			//pose.pose.orientation.y = aux.y();
			//pose.pose.orientation.z = aux.z();
			//pose.pose.orientation.w = aux.w();
			//Transform pose to odom frame
			//geometry_msgs::PoseStamped meanpose = transformPoseTo(pose, sensor_frame, false);
					
			
			mutex.lock();
			geometry_msgs::PoseStamped rpose = robot_pose;
			mutex.unlock();
			
			
			//WATCH OUT!!!
			//This is a very poor filter of the ceiling
			//I compare the height of the point with the height of the robot
			if(fabs(rpose.pose.position.z - p.z) > max_height_allowed)
				continue;
			
			
			
			/*

			//geometry_msgs::PoseStamped rpose = transformPoseTo(aux_pose, odom_frame, false);
			
			// correct if n . (pv - Pi) > 0
			// n  -> evec with the lowest eigenvalue (= normal vector)
			// pv -> (0,0,0)
			// pi -> mean
			// See if we need to flip any plane normals
			
			// Dot product between the (viewpoint - point) and the plane normal
			float cos_theta = ((rpose.pose.position.x-mean[0]) * nx + (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz); 
			// Flip the plane normal
			if (cos_theta < 0)
			{

				pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y, rpose.pose.position.z, nx, ny, nz);	
				
				Eigen::Vector3d norm(nx, ny, nz);
				norm = norm.normalized();
				//evecs(0,i) = nx;
				//evecs(1,i) = ny;
				//evecs(2,i) = nz;
				evecs.col(i) =	norm;				
				//evecs = -1.0*evecs;
					
				Eigen::Vector3d norm2;
				Eigen::Vector3d norm3;
				if ((fabs((double)norm(0)) > 0.001) || (fabs((double)norm(1)) > 0.001)) {
    				norm2 << -norm(1), norm(0), 0;
				} else {
    				norm2 << 0, norm(2), -norm(1);
				}
				norm2.normalize();

				norm3 = norm.cross(norm2);

				Eigen::Matrix3d R;  // Rotation matrix defining orientation
				R.col(0) = norm;
				R.col(1) = norm2;
				R.col(2) = norm3;
					

				//Eigen::Quaternion<double> q2(evecs); //evecs_trans
				Eigen::Quaternion<double> q2(R);
				tf::Quaternion aux2(q2.x(), q2.y(), q2.z(), q2.w());
				aux2.normalize();
				aux = aux2;

				tf::Matrix3x3 m(aux2);
     			//double roll2, pitch2, yaw2;
      			m.getRPY(roll, pitch, yaw);

				//pose.pose.orientation.x = aux2.x();
				//pose.pose.orientation.y = aux2.y();
				//pose.pose.orientation.z = aux2.z();
				//pose.pose.orientation.w = aux2.w();	
			}*/
			
			
			//if(fabs(pitch) < pitch_low || fabs(pitch) > pitch_high && fabs(yaw) < roll_low || fabs(yaw) > roll_high) 
			//if(pitch < pitch_high && pitch > pitch_low && yaw < roll_high && yaw > roll_low) // && fabs(evals(i)) < roughness))
			if(fabs(pitch) < pitch_high && fabs(pitch) > pitch_low && fabs(roll) < roll_high && fabs(roll) > roll_low) //ok
			//if(-pitch < pitch_high && -pitch > pitch_low && -roll < roll_high && -roll > roll_low)
			{
				//printf("pose3d VALID. pitch:%.2f, roll:%.2f, upper:%.2f, lower:%.2f\n", fabs(pitch), fabs(roll), pitch_high, pitch_low);
				indices->push_back(ind);
				//indices2.push_back(ind);

				filtered.push_back(p);


				if(print_arrows)
				{
					visualization_msgs::Marker marker;
					marker.header.frame_id = odom_frame; //frame_out;
					marker.header.stamp = ros::Time();
					marker.ns = "normal_arrows";
					marker.id = ind; //key;
					marker.type = visualization_msgs::Marker::ARROW;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.position.x = mean[0];  //m[0]
					marker.pose.position.y = mean[1];	//m[1]
					marker.pose.position.z = mean[2];  //m[2]
					//marker.pose.position = meanpose.pose.position;
					//tf::Quaternion aux(q.x(), q.y(), q.z(), q.w());
					//aux.normalize();
					marker.pose.orientation.x = aux.x();
					marker.pose.orientation.y = aux.y();
					marker.pose.orientation.z = aux.z();
					marker.pose.orientation.w = aux.w();
					//marker.pose.orientation = meanpose.pose.orientation;
		
					marker.scale.x = 9.0*evals(i);
					marker.scale.y = 0.03; //2.0*evals(1);
					marker.scale.z = 0.03; //2.0*evals(2);
			
					marker.color.a = 1.0;
					marker.color.r = 0.0;
					marker.color.g = 0.0;
					marker.color.b = 1.0;

					marker.lifetime = ros::Duration(2.0);

					marray.markers.push_back(marker);

				}

			} else {
				//printf("pose3d INVALID. pitch:%.2f, roll:%.2f, upper:%.2f, lower:%.2f\n", fabs(pitch), fabs(roll), pitch_high, pitch_low);
			}
			ind++;
			
		}

		pcl::ExtractIndices<pcl::PointXYZ> eifilter(false); // Initializing with true will allow us to extract the removed indices
		eifilter.setInputCloud(cloud);
		eifilter.setIndices(indices);
		//eifilter.filter (*cloud_out);
		eifilter.filterDirectly(cloud);
		//eifilter.filter(indices2);
		
		arrow_pub.publish(marray);
		return filtered;
	} else {
		printf("pcl_filters. The point cloud is ordered!!!\n");
	}

/*

En setIndices, indices_in es un pcl::IndicesPtr, 
que es un puntero a vector: typedef boost::shared_ptr<std::vector<int> >

pcl::ExtractIndices<PointType> eifilter (true); // Initializing with true will allow us to extract the removed indices
eifilter.setInputCloud (cloud_in);
eifilter.setIndices (indices_in);
eifilter.filter (*cloud_out);
Alternativamente a filter, podemos usar:
eifilter.filterDirectly (cloud_in);
This will directly modify cloud_in instead of creating a copy of the cloud

*/

}








pcl::PointCloud<pcl::PointXYZRGB> assignCosts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	visualization_msgs::MarkerArray marray;
	pcl::PointCloud<pcl::PointXYZRGB> coloured;
	coloured.header = cloud->header;
	coloured.is_dense = cloud->is_dense;
	coloured.height = cloud->height;
	coloured.sensor_origin_ = cloud->sensor_origin_;
	coloured.sensor_orientation_ = cloud->sensor_orientation_;
	
	std::vector<float> weights;

	const std::map< size_t, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf >* leaves = &(vgc->getLeaves());

	if(!cloud->isOrganized()) //The height value must be different than 1 for a dataset to be organized
	{
		//the index of the width should match with the leaf index???
		//pcl::IndicesPtr indices(new std::vector<int>); //typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr
		

		int ind=0;
		for(const auto& it : *leaves) //const 
		{
			const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf *l = &(it.second);

			Eigen::Vector3d mean = l->getMean();
			Eigen::Matrix3d evecs = l->getEvecs();
			Eigen::Vector3d evals = l->getEvals();
			
			float num_points = (float)l->getPointCount();
			int points = l->getPointCount();

			pcl::PointXYZRGB p;
			p.x = mean[0];
			p.y = mean[1];
			p.z = mean[2];
		
		
			Eigen::Quaternion<double> q(evecs);

			auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2); //0->yaw, 1->pitch, 2->roll
			double yaw = euler[0]; 
			double pitch = euler[1]; 
			double roll = euler[2];


			//Identify the lowest eigenvalue
			//TODO: the eigen values are ordered. 
			//The smallest is the last one. CHECK THIS! 
			int i=0;
			if(evals(0) < evals(1) && evals(0) < evals(2))
				i = 0;
			else if(evals(1) < evals(0) && evals(1) < evals(2))
				i = 1;
			else 
				i = 2;  
			
			float nx = (float)evecs(0,i);
			float ny = (float)evecs(1,i);
			float nz = (float)evecs(2,i);

			
			
			//create a pose
			//geometry_msgs::PoseStamped pose;
			//pose.header.frame_id = odom_frame;
			//pose.header.stamp = ros::Time::now();
			tf::Quaternion aux(q.x(), q.y(), q.z(), q.w());
			aux.normalize();
			//pose.pose.position.x = mean[0];
			//pose.pose.position.y = mean[1];
			//pose.pose.position.z = mean[2];
			
			//pose.pose.orientation.x = aux.x();
			//pose.pose.orientation.y = aux.y();
			//pose.pose.orientation.z = aux.z();
			//pose.pose.orientation.w = aux.w();
			//Transform pose to odom frame
			//geometry_msgs::PoseStamped meanpose = transformPoseTo(pose, sensor_frame, false);
					
			
			mutex.lock();
			geometry_msgs::PoseStamped rpose = robot_pose;
			mutex.unlock();
			
			
			
			if(filter_ceiling)
			{
				//WATCH OUT!!!
				//This is a very poor filter of the ceiling
				//I compare the height of the point with the height of the robot
				if(fabs(rpose.pose.position.z - p.z) > max_height_allowed)
					continue;
					
				if(!(fabs(pitch) < pitch_high && fabs(pitch) > pitch_low && fabs(roll) < roll_high && fabs(roll) > roll_low)) //!ok
					continue;

				
			} 
			
			std::vector<float> features;	
				
			//Normalize pitch and yaw
			float zero = M_PI/2.0;
			pitch = fabs(zero - fabs(pitch));
			pitch = pitch/zero;
			roll = fabs(zero - fabs(roll));
			roll = roll/zero;
			
			if (pitch > 0.55)
				pitch = 1.0;
			if(roll > 0.55)
				roll = 1.0;
			
			features.push_back(pitch);
			features.push_back(roll);
			
	
			//Normalize roughness
			//Maximum value of roughness?????
			float rough = fabs(evals(2));
			if (rough > roughness) rough = roughness;
			rough = rough/roughness; 
			
			features.push_back(rough);
	
			//normalize point dist. max dist = robot_radius_
			//point_dist = point_dist / robot_radius_;
	
			//Normalize stddev
			//stddev = 1.0 - (stddev / max_stddev);
	
			//Normalize num_points
			float max_points = 50.0;
			if(num_points > max_points) num_points = max_points;
			num_points = 1.0 - (num_points/max_points);	
			
			features.push_back(num_points);
			std::vector<float> weights;
			weights.assign(4, 0.25);

			float cost = 0.0;
			for(unsigned int i=0; i<features.size(); i++)
				cost += features[i] * weights[i];
				
			
			//Filter walls, high ceiling and high roughness
			//if(pitch > 0.6 || roll > 0.6 || rough > 0.6 || (fabs(rpose.pose.position.z - p.z) > max_height_allowed) ) {
			if(!filter_ceiling && fabs(rpose.pose.position.z - p.z) > max_height_allowed) { 
				cost = 1.0;
			} 	
			
			if(pitch == 1.0 || roll == 1.0 || rough > 0.75)
				cost = cost>0.8? 0.8 : cost;
			
			//Convert scale [0-1] of the cost to scale [0-255];
			int c = (int)round(cost * 255);
			
			//printf("cost:%.3f, color:%i, p:%.2f, r:%.2f, roug:%.2f, np:%i, pc:%.2f\n", cost, c, pitch, roll, rough, points, num_points);
			
			p.r = c;
			p.g = 255 - c;
			p.b = 0;	
				
			coloured.push_back(p);
			ind++;
			
			
		
			
			/*

			//geometry_msgs::PoseStamped rpose = transformPoseTo(aux_pose, odom_frame, false);
			
			// correct if n . (pv - Pi) > 0
			// n  -> evec with the lowest eigenvalue (= normal vector)
			// pv -> (0,0,0)
			// pi -> mean
			// See if we need to flip any plane normals
			
			// Dot product between the (viewpoint - point) and the plane normal
			float cos_theta = ((rpose.pose.position.x-mean[0]) * nx + (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz); 
			// Flip the plane normal
			if (cos_theta < 0)
			{

				pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y, rpose.pose.position.z, nx, ny, nz);	
				
				Eigen::Vector3d norm(nx, ny, nz);
				norm = norm.normalized();
				//evecs(0,i) = nx;
				//evecs(1,i) = ny;
				//evecs(2,i) = nz;
				evecs.col(i) =	norm;				
				//evecs = -1.0*evecs;
					
				Eigen::Vector3d norm2;
				Eigen::Vector3d norm3;
				if ((fabs((double)norm(0)) > 0.001) || (fabs((double)norm(1)) > 0.001)) {
    				norm2 << -norm(1), norm(0), 0;
				} else {
    				norm2 << 0, norm(2), -norm(1);
				}
				norm2.normalize();

				norm3 = norm.cross(norm2);

				Eigen::Matrix3d R;  // Rotation matrix defining orientation
				R.col(0) = norm;
				R.col(1) = norm2;
				R.col(2) = norm3;
					

				//Eigen::Quaternion<double> q2(evecs); //evecs_trans
				Eigen::Quaternion<double> q2(R);
				tf::Quaternion aux2(q2.x(), q2.y(), q2.z(), q2.w());
				aux2.normalize();
				aux = aux2;

				tf::Matrix3x3 m(aux2);
     			//double roll2, pitch2, yaw2;
      			m.getRPY(roll, pitch, yaw);

				//pose.pose.orientation.x = aux2.x();
				//pose.pose.orientation.y = aux2.y();
				//pose.pose.orientation.z = aux2.z();
				//pose.pose.orientation.w = aux2.w();	
			}*/
			
			
		}
		coloured.width = ind;
		return coloured;
		
	} else {
		ROS_ERROR("pcl_filters. The point cloud is ordered!!!\n");
		return coloured;
	}


}








void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	//Transform the coordinates of the pointcloud
	sensor_msgs::PointCloud2 local;
	pcl_ros::transformPointCloud(odom_frame, *msg, local, *listener);

	//pc_frame = msg->header.frame_id;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromROSMsg(*msg, *cloud);
	pcl::fromROSMsg(local, *cloud);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::PointCloud<pcl::PointXYZ> cloud_out = *cloud;


	sensor_msgs::PointCloud2 pc_out;



	if(passthrough)
	{
		//TODO: this is not the most efficient way to do it. Try to use indices

		//TODO: Try conditional removal
		/*
		pcl::ConditionOr<PointT>::Ptr range_cond (new pcl::ConditionOr<PointT> ()); 
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, minX)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, maxX)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, minY)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, maxY)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, minZ)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, maxZ)));

		pcl::ConditionalRemoval<PointT> range_filt;
		range_filt.setInputCloud(body);
		range_filt.setCondition (range_cond);
		range_filt.filter(*bodyFiltered);
		*/

		pass->setInputCloud(cloud);

		//filter X axis
		pass->setFilterFieldName ("x");
		pass->setFilterLimits((pass_cx - pass_rx), (pass_cx + pass_rx));
		//std::vector<int> indices_x;
		//pcl::IndicesPtr indices_x;
		//ROS_INFO("before filter X");
		//pass->filter(*indices_x);
		pass->filter(*cloud_fil);
		*cloud = *cloud_fil;
		
		//filter Y axis
		//pass->setIndices(indices_x);
		pass->setInputCloud(cloud);
		pass->setFilterFieldName ("y");
		pass->setFilterLimits ((pass_cy - pass_ry), (pass_cy + pass_ry));
		//pass->setNegative(true);
		//std::vector<int>* indices_xy;
		//pcl::IndicesPtr indices_xy;
		pass->filter(*cloud_fil);
		*cloud = *cloud_fil;

		//filter Z axis
		//pass->setIndices(indices_xy);
		pass->setInputCloud(cloud);
		pass->setFilterFieldName ("z");
		pass->setFilterLimits ((pass_cz - pass_rz), (pass_cz + pass_rz));
		//pass->setNegative(true);
		pass->filter(*cloud_fil);
		*cloud = *cloud_fil;
	}


	if(cropbox)
	{
		box->setInputCloud(cloud);
		if(use_robot_pose)
		{
			mutex.lock();
			geometry_msgs::PoseStamped p = robot_pose;
			mutex.unlock();

			//geometry_msgs::PoseStamped p;
			//p = transformPoseTo(pose, odom_frame, true);
			
			float min_x = p.pose.position.x - local_grid_radius;
			float max_x = p.pose.position.x + local_grid_radius;
			float min_y = p.pose.position.y - local_grid_radius;
			float max_y = p.pose.position.y + local_grid_radius;
			float min_z = p.pose.position.z - local_grid_radius;
			float max_z = p.pose.position.z + local_grid_radius;

			box->setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
			box->setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
		}
		box->filter(*cloud_fil);
		*cloud = *cloud_fil;
	}


	if(statistical)
	{
		sor->setInputCloud(cloud);
		sor->filter(*cloud_fil);
		*cloud = *cloud_fil;
	}
	if(voxelgrid)
	{
		vg->setInputCloud(cloud);
		vg->filter(*cloud_fil);
		*cloud = *cloud_fil;

	}


	pcl::PointCloud<pcl::PointXYZRGB> coloured;
	if(voxelgridcov && !voxelgrid)
	{
		vgc->setInputCloud(cloud);
		vgc->filter(*cloud_fil, true);
		*cloud = *cloud_fil;

		//if(filter_ceiling)
		//{
		//	filtered = removeWallsAndCeiling(cloud);
		//	*cloud = filtered;
		//}
		
		if(assign_costs)
		{
			coloured = assignCosts(cloud);
			pcl::toROSMsg(coloured, pc_out);
			pc_pub.publish(pc_out);
			return;
		}
		
		
	}


	pcl::toROSMsg(*cloud, pc_out);


	//Transform the coordinates of the pointcloud
	//sensor_msgs::PointCloud2 local;
	//pcl_ros::transformPointCloud("/indires_rover/base_link", pc_out, local, *listener);

	pc_pub.publish(pc_out);
	//pc_pub.publish(local);


}







void poseCallback(const nav_msgs::OdometryConstPtr &msg)
{
	geometry_msgs::PoseStamped pose;

	pose.header = msg->header;
	pose.pose = msg->pose.pose;

	geometry_msgs::PoseStamped pose_out;
	//if(!pc_frame.empty())
	pose_out = transformPoseTo(pose, odom_frame, false);

	mutex.lock();
	robot_pose = pose_out;
	mutex.unlock();


}




int main(int argc, char **argv)
{

	ros::init(argc, argv, "pcl_costmap");
	ROS_INFO("---PCL COSTMAP---");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");

	listener = new tf::TransformListener(ros::Duration(10));

	std::string pointcloud_topic = "indires_rover/front_rgbd_camera/front_rgbd_camera/depth/points";
	//camera_tf = "indires_rover/front_rgbd_camera/depth_frame";
	nh.getParam("pointcloud_topic", pointcloud_topic);
	std::string output_topic = "PointCloud_filtered";
	nh.getParam("output_topic", output_topic);

	nh.getParam("use_robot_pose", use_robot_pose);
	nh.getParam("robot_pose_topic", robot_pose_topic);
	nh.getParam("local_grid_radius", local_grid_radius);

	nh.getParam("max_height_allowed", max_height_allowed);
	nh.getParam("odom_frame", odom_frame);
	nh.getParam("sensor_frame", sensor_frame);

	//nh.getParam("show_normal_arrows", print_arrows);

	nh.getParam("filter_ceiling_and_walls", filter_ceiling);
	nh.getParam("pitch_max_inclination", pitch_max);
	nh.getParam("roll_max_inclination", roll_max);
	nh.getParam("max_roughness", roughness);

	//pitch_low = -M_PI/2.0 - pitch_max; 
	//pitch_high = -M_PI/2.0 + pitch_max;
	//roll_low = -M_PI/2.0 - roll_max;
	//roll_high = -M_PI/2.0 + roll_max;
	
	//pitch_low = pitch_max;
	//pitch_high = M_PI - pitch_max;
	//roll_low = roll_max;
	//roll_high = M_PI - roll_max;
	
	pitch_low = M_PI/2.0 - pitch_max;
	pitch_high = M_PI/2.0 + pitch_max;
	roll_low = M_PI/2.0 - roll_max;
	roll_high = M_PI/2.0 + roll_max;


	//sleep a bit in order to fill the TF buffer
	sleep(6.0);


	if(nh.hasParam("PassThroughFilter"))
	{
		ros::NodeHandle n("~PassThroughFilter");
		n.getParam("enable", passthrough);
		if(passthrough) {
			ROS_INFO("Using PassThrough filter:");
			n.getParam("center_x", pass_cx);
			n.getParam("center_y", pass_cy);
			n.getParam("center_z", pass_cz);
			ROS_INFO("\tcx: %.2f, cy: %.2f, cz: %.2f", pass_cx, pass_cy, pass_cz);
			n.getParam("rad_x", pass_rx);
			n.getParam("rad_y", pass_ry);
			n.getParam("rad_z", pass_rz); 
			ROS_INFO("\trx: %.2f, ry: %.2f, rz: %.2f", pass_rx, pass_ry, pass_rz);
			pass = new pcl::PassThrough<pcl::PointXYZ>(false);
		}
	}


	if(nh.hasParam("CropBoxFilter"))
	{
		ros::NodeHandle n("~CropBoxFilter");
		n.getParam("enable", cropbox);
		if(cropbox) {
			ROS_INFO("Using CropBox filter:");
			float min_x = -10.0;
			float min_y = -10.0;
			float min_z = -10.0;
   			float max_x = 10.0;
			float max_y = 10.0;
			float max_z = 10.0;
			n.getParam("cb_min_x", min_x);
			n.getParam("cb_min_y", min_y);
			n.getParam("cb_min_z", min_z);
			n.getParam("cb_max_x", max_x);
			n.getParam("cb_max_y", max_y);
			n.getParam("cb_max_z", max_z);
			ROS_INFO("\tminx: %.2f, miny: %.2f, minz: %.2f", min_x, min_y, min_z);
			ROS_INFO("\tmaxx: %.2f, maxy: %.2f, maxz: %.2f", max_x, max_y, max_z);
			box = new pcl::CropBox<pcl::PointXYZ>();
			box->setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
			box->setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));

		}
	}


	if(nh.hasParam("StatisticalOutlierFilter"))
	{
		ros::NodeHandle n("~StatisticalOutlierFilter");
		n.getParam("enable", statistical);
		if(statistical) {  
			ROS_INFO("Using StatisticalOutlierRemoval filter:");
			int kneighbors = 80;
			n.getParam("mean_k_neighbors", kneighbors);
			ROS_INFO("\tMean_k_neighbors: %i", kneighbors);
			float stddev = 2.0; 
			n.getParam("std_dev", stddev);
			ROS_INFO("\tStd_dev: %.2f", stddev);

			sor = new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>();
			sor->setMeanK (kneighbors);
  			sor->setStddevMulThresh (stddev);  
		}
	}



	if(nh.hasParam("VoxelGridFilter"))
	{
		ros::NodeHandle n("~VoxelGridFilter");
		n.getParam("enable", voxelgrid);
		if(voxelgrid) {
			ROS_INFO("Using VoxelGrid filter:");
			float leaf_size = 0.20;
			n.getParam("leaf_size", leaf_size);
			ROS_INFO("\tLeaf_size: %.2f", leaf_size);
			int min_point_per_voxel = 10; 
			n.getParam("leaf_size", leaf_size);
			n.getParam("min_point_per_voxel", min_point_per_voxel);
			ROS_INFO("\tmin_point_per_voxel: %i", min_point_per_voxel);
			vg = new pcl::VoxelGrid<pcl::PointXYZ>();
			vg->setLeafSize(leaf_size, leaf_size, leaf_size);
			//vg->setMinPointPerVoxel(min_point_per_voxel);  
			vg->setMinimumPointsNumberPerVoxel(min_point_per_voxel); 
		}
	}


	if(nh.hasParam("VoxelGridCovarianceFilter") && !voxelgrid)
	{
		ros::NodeHandle n("~VoxelGridCovarianceFilter");
		n.getParam("enable", voxelgridcov);
		if(voxelgridcov) {
			ROS_INFO("Using VoxelGridCovariance filter:");
			float leaf_size = 0.20;
			n.getParam("leaf_size", leaf_size);
			ROS_INFO("\tLeaf_size: %.2f", leaf_size);
			int min_point_per_voxel = 10; 
			n.getParam("leaf_size", leaf_size);
			n.getParam("min_point_per_voxel", min_point_per_voxel);
			vgc = new pcl::VoxelGridCovariance<pcl::PointXYZ>();
			vgc->setLeafSize(leaf_size, leaf_size, leaf_size); //0.01f, 0.01f, 0.01f	//inherited from pcl::VoxelGrid
			vgc->setMinPointPerVoxel(min_point_per_voxel);  
		}
	}



	arrow_pub = nh.advertise<visualization_msgs::MarkerArray>( "pcl_filter_arrows", 0);
	//filter_pub = nh.advertise<visualization_msgs::MarkerArray>( "NDTMAP_TRANS", 0);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>( output_topic, 0);

	ros::Subscriber sub;
	sub  = n.subscribe(pointcloud_topic, 1, pcCallback);

	ros::Subscriber sub2;
	if(use_robot_pose)
		sub2 = n.subscribe(robot_pose_topic, 1, poseCallback);

	

	ros::spin();

}





