
#include <navigation_features_3d/nav_features3d.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>
#include <sys/time.h>
#include <numeric>
#include <string>


using namespace std;


nav3d::Features3D::Features3D() {}


nav3d::Features3D::Features3D(tf::TransformListener* tf, float size_x, float size_y, float size_z) 
{
	tf_listener_ = tf;
	size_x_ = size_x*2.0;  	//m
	size_y_ = size_y*2.0;  	//m	
	size_z_ = size_z*2.0;  	//m
	//resolution_ = res;		//m/cell	
	max_planning_dist_ = sqrt((size_x_*size_x_)+(size_y_*size_y_)+(size_z_*size_z_));

	setParams();

}




nav3d::Features3D::Features3D(tf::TransformListener* tf, vector<geometry_msgs::Point>* footprint, float size_x, float size_y, float size_z)
{
	
	tf_listener_ = tf;
	//robot_radius_ = insc_radius;
	size_x_ = size_x*2.0;  	//m
	size_y_ = size_y*2.0;  	//m
	size_z_ = size_z*2.0;  	//m	
	//resolution_ = res;		//m/cell	
	max_planning_dist_ = sqrt((size_x_*size_x_)+(size_y_*size_y_)+(size_z_*size_z_));
	
	myfootprint_ = footprint;
	
	
	setParams();

	

}





void nav3d::Features3D::setParams()
{
	
	ROS_INFO("NAVIGATION FEATURES 3D. SETTING PARAMETERS...");
	//Read the ROS params from the server
	ros::NodeHandle n("~/navigation_features_3d");


	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);
    
    feature_set_ = 1;
	n.getParam("feature_set", feature_set_);
	
	//Load weights "w1, w2, ..."
	w_.clear();
	w_.push_back(1.0); //weight for the valid feature (boolean [0,1])
	
	
	if(feature_set_ == 1)
	{
		unsigned int i = 1;
		while(i<=3)
		{
			char buf[10];
			sprintf(buf, "w%u", i);
			string st = string(buf);
			
			if(n.hasParam(st.c_str())){
				double wg = 0.0;
				n.getParam(st.c_str(), wg);
				w_.push_back((float)wg);	
				printf("NavFeatures3d. weight %u= %.3f loaded\n", i, wg);
			}
			i++;
		}
	} else {
	
		bool ok = true;
		unsigned int i = 1;
		while(ok)
		{
			char buf[10];
			sprintf(buf, "w%u", i);
			string st = string(buf);
					
			if(n.hasParam(st.c_str())){
				double wg = 0.0;
				n.getParam(st.c_str(), wg);
				w_.push_back((float)wg);	
				printf("NavFeatures3d. weight %u= %.3f loaded\n", i, wg);
			} else {
				//printf("param '%s' not found\n", st.c_str());
				ok = false;
			}
			i++;
		}
	}

	
	string cloud_topic;
	n.param<string>("pointcloud_topic", cloud_topic, string("/scan360/point_cloud")); 

	double pitch_max = M_PI/2.0;
	double roll_max = M_PI/2.0;
	roughness_ = 0.03;
	n.getParam("max_pitch_inclination", pitch_max);
	n.getParam("max_roll_inclination", roll_max);
	n.getParam("max_roughness", roughness_);
	min_points_allowed_ = 20;
	n.getParam("min_points_allowed", min_points_allowed_);

	//ros::NodeHandle nh("~");
	n.getParam("robot_base_frame", robot_base_frame_);
	n.getParam("robot_odom_frame", robot_odom_frame_);
	n.getParam("robot_odom_topic", robot_odom_topic_);
	
	nfe_exploration_ = false;
	n.getParam("nfe_exploration", nfe_exploration_);
	if(nfe_exploration_)
		printf("NFE EXPLORATION WILL BE EMPLOYED!!!\n");

	//pitch_low_ = -M_PI/2.0 - pitch_max;
	//pitch_low2_ = M_PI/2.0 - pitch_max; 
	//pitch_high_ = -M_PI/2.0 + pitch_max;
	//pitch_high2_ = M_PI/2.0 + pitch_max;
	//roll_low_ = -M_PI/2.0 - roll_max;
	//roll_low2_ = M_PI/2.0 - roll_max;
	//roll_high_ = -M_PI/2.0 + roll_max;
	//roll_high2_ = M_PI/2.0 + roll_max;
	pitch_low_ = pitch_max;
	pitch_high_ = M_PI - pitch_max;
	roll_low_ = roll_max;
	roll_high_ = M_PI - roll_max;


	robot_radius_ = 0.25;
	n.getParam("robot_circuns_radius", robot_radius_);


	kdtree_ = new pcl::KdTreeFLANN<pcl::PointXYZ>();
	//pca_ =  new pcl::PCA<pcl::PointXYZ>();
	
	
	//octree_resolution_ = 128.0f;
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc(128.0);
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr oc(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution_));
	//octree_ = oc;
	//octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution_);
	//octree_ = new pcl::search::Octree<pcl::PointXYZ>(octree_resolution_);
	//octree_->setResolution(octree_resolution_);
	//octree_.setResolution(octree_resolution_);



	//goal_sub_ = nh_.subscribe("/rrt_goal", 1, &Features3D::goalCallback, this);
	
	cloud_sub_ = nh_.subscribe(cloud_topic, 1, &Features3D::cloudCallback, this);

	pose_sub_ = nh_.subscribe(robot_odom_topic_, 1, &Features3D::poseCallback, this);
	
	explore_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrt_explore_costs", 1);
	
	
	//pub_gaussian_markers_ = n.advertise<visualization_msgs::MarkerArray>("gaussian_markers", 5);
	
	//ros::NodeHandle nd("navigation_features");
	//ros::ServiceServer service = nd.advertiseService("setApproachingIT", setApproachingIT);
	//loss_srv_ = nd.advertiseService("set_use_loss_func", &features::NavFeatures::setLossService, this);
	//valid_srv_ = nd.advertiseService("is_pose_valid", &features::NavFeatures::isPoseValidService, this);
	//weights_srv_ = nd.advertiseService("setWeights", &features::NavFeatures::setWeightsService, this);
	//init_weights_srv_ = nd.advertiseService("initWeights", &features::NavFeatures::initializeWeightsService, this);
	//scenario_srv_ = nd.advertiseService("setScenario", &features::NavFeatures::setScenarioService, this);
	//features_srv_ = nd.advertiseService("getPathFeatureCount", &features::NavFeatures::getFeatureCountService, this);


	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);
	
}


nav3d::Features3D::~Features3D() {
	delete kdtree_;
	//delete octree_;
}




//Point_cloud2 callback
void nav3d::Features3D::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){


	//Transform the coordinates of the pointcloud
	sensor_msgs::PointCloud2 local;
	if(!pcl_ros::transformPointCloud(robot_odom_frame_, *msg, local, *tf_listener_)) {
		ROS_WARN("nav_features3d. CloudCallback. TransformPointCloud failed!!!!!");
	} else {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	//pcl::fromROSMsg(*msg, *cloud);
    	
    	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc(octree_resolution_);

		tree_mutex_.lock();
		pcl::fromROSMsg(local, *cloud);
		pcl_cloud_ = *cloud;
		//ros::WallTime t1 = ros::WallTime::now();
		kdtree_->setInputCloud(cloud);
		//ros::WallTime t2 = ros::WallTime::now();
		//pca_->setInputCloud(cloud);
		
		//ros::WallTime t3 = ros::WallTime::now();
		//octree_->deleteTree();
		//octree_->setInputCloud(cloud);
		//octree_->addPointsFromInputCloud();
		//ros::WallTime t4 = ros::WallTime::now();
		//double t_kdtree = (t2-t1).toSec();
		//double t_octree = (t4-t3).toSec();
		//printf("cloudCallback kdtree: %.4f, octree:%.4f\n", t_kdtree, t_octree);
		tree_mutex_.unlock();
	}
}




void nav3d::Features3D::poseCallback(const nav_msgs::OdometryConstPtr &msg)
{
	geometry_msgs::PoseStamped pose;

	pose.header = msg->header;
	pose.pose = msg->pose.pose;

	geometry_msgs::PoseStamped pose_out;
	//if(!pc_frame.empty())
	pose_out = transformPoseTo(pose, robot_odom_frame_, true);
	
	float d = 5.0;
	if(!robot_traj_.empty()){
		geometry_msgs::PoseStamped t = robot_traj_.back();
		float xt = t.pose.position.x;
		float yt = t.pose.position.y;
		float zt = t.pose.position.z;
		float xr = pose_out.pose.position.x;
		float yr = pose_out.pose.position.y;
		float zr = pose_out.pose.position.z;
		d = sqrt((xt-xr)*(xt-xr) + (yt-yr)*(yt-yr) + (zt-zr)*(zt-zr));
	}
		

	pose_mutex_.lock();
	robot_pose_ = pose_out;
	if(d > 0.03) {
		robot_traj_.push_back(pose_out);
		//printf("poseCallback. inserting x:%.2f, y:%.2f, z:%.2f, size: %u\n", pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z, (unsigned int)robot_traj_.size());
	}
	pose_mutex_.unlock();


}




bool nav3d::Features3D::poseValid(geometry_msgs::PoseStamped* s)
{
	std::vector<float> f = getFeatures(s);
	//for(unsigned int i=0; i<f.size(); i++)
	//	printf("feature[%u] = %.2f", i, f[i]);
	//printf("\n");
	if(f[0] == 1.0)
		return true;
	else
		return false;
}







std::vector<std::vector<int> > nav3d::Features3D::clusterize_leaves(std::vector<geometry_msgs::Point>* points, float radius)
{
	//Create the pcl pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr ps(new pcl::PointCloud<pcl::PointXYZ>);
	ps->width = points->size();
	ps->height = 1;
	ps->is_dense = true;
	for(unsigned int i=0; i<points->size(); i++)
	{
		pcl::PointXYZ p(points->at(i).x, points->at(i).y, points->at(i).z);
		ps->points.push_back(p);
	}
	//printf("Clusterize_leaves. pointcloud in size: %u\n", (unsigned int)ps->width);
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(ps);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(radius); 
	ec.setMinClusterSize(2);
	ec.setMaxClusterSize(4);
	ec.setSearchMethod(tree);
	ec.setInputCloud(ps);
	ec.extract(cluster_indices);


	std::vector<std::vector<int> > clusters;

	if(cluster_indices.empty())
	{
		printf("Clusterize_leaves. No clusters found!!!\n");
		return clusters;
	}	

	
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		std::vector<int> ind;
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
			//cloud_cluster->points.push_back(ps->points[*pit]);
			ind.push_back((int)*pit); 
			//printf("nav_features3d. Cluster: %i, indice: %i \n", j, (int)*pit);
		}
		//cloud_cluster->width = cloud_cluster->points.size();
		//cloud_cluster->height = 1;
		//cloud_cluster->is_dense = true;
		//printf("nav_features3d. cluster %i filled\n", j);
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		//std::stringstream ss;
		//ss << "cloud_cluster_" << j << ".pcd";
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		
		clusters.push_back(ind);
		j++;
	}
	
	//printf("nav_features3d. returning %u clusters \n", (unsigned int)clusters.size());
	return clusters;
	
}




std::vector<float> nav3d::Features3D::evaluate_leaves(std::vector<geometry_msgs::Point>* points, float radius)
{
	
	//publish exploration goals and costs
	visualization_msgs::MarkerArray ma;
	
	visualization_msgs::Marker l;
	l.header.frame_id = robot_base_frame_;
	l.header.stamp = ros::Time();
	l.ns = "rrt_exploration_costs";
	l.type = visualization_msgs::Marker::CYLINDER;
	l.action = visualization_msgs::Marker::ADD;
	l.scale.x = 0.1;
	l.scale.y = 0.1;
	l.scale.z = 0.03;
	l.color.r = 1.0f;
	l.color.g = 1.0f;
	l.color.b = 0.0f;
	l.color.a = 1.0;
	l.lifetime = ros::Duration();
	
	visualization_msgs::Marker t;
	t = l;
	t.scale.x = 0.15;
	t.scale.y = 0.15;
	t.scale.z = 0.15;
	t.color.r = 1.0f;
	t.color.g = 1.0f;
	t.color.b = 1.0f;
	t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			
	
	
	std::vector<float> gain_info;
	
	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = robot_base_frame_;
	ps.header.stamp = ros::Time::now();
	
	
	if(nfe_exploration_)
	{
		
		float max_dist = 5.0;
		for(unsigned int i=0; i<points->size(); i++)
		{
			ps.pose.position = points->at(i);
			ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			geometry_msgs::PoseStamped st = transformPoseTo(ps, robot_odom_frame_, false);
			float dcost = no_return_cost(&st);
			
			geometry_msgs::Point p = points->at(i);
			float d = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
			if (d>max_dist)
				d = max_dist;
			float nfe_cost = d/max_dist;
			
			float cost = 0.6*nfe_cost + 0.4*dcost;
			gain_info.push_back(cost);
			
			l.id = i+300;
			//l.text = std::to_string(cost);
			l.pose.position = points->at(i);
			l.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			//printf(", id: %i, pcost: %.3f, dcost: %.3f, totalc: %.3f\n", (int)l.id, num_points, dcost, cost);
			ma.markers.push_back(l);
			t.id = i+600;
			t.pose.position = points->at(i);
			t.pose.position.z = t.pose.position.z + 0.05; 
			t.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			t.text = std::to_string(cost);
			ma.markers.push_back(t);
		}
		explore_pub_.publish(ma);
		return gain_info;
	}
	
	
	
	for(unsigned int i=0; i<points->size(); i++)
	{
		ps.pose.position = points->at(i);
		ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		geometry_msgs::PoseStamped st = transformPoseTo(ps, robot_odom_frame_, false);
		
		pcl::PointXYZ sp;
		sp.x = st.pose.position.x;
		sp.y = st.pose.position.y;
		sp.z = st.pose.position.z;
		

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		tree_mutex_.lock();

		float num_points = kdtree_->radiusSearch(sp, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		//printf("3DNavFeat. Evaluate_leaves. leaf %i, num_points: %.1f", i, num_points);
		
		if(num_points > 0.0)
		{
			//Normalize num_points
			float max_points = 10000.0;
			if(num_points > max_points) num_points = max_points;
			num_points = (num_points/max_points);
				
		} else
			num_points = 1.0;
			
		float dcost = no_return_cost(&st);
			
		tree_mutex_.unlock();
		
		float cost = 0.5*num_points + 0.3*dcost; // + 0.1*point_dist + 0.1*stddev;
		gain_info.push_back(cost);
		
		l.id = i+300;
		//l.text = std::to_string(cost);
		l.pose.position = points->at(i);
		l.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		//printf(", id: %i, pcost: %.3f, dcost: %.3f, totalc: %.3f\n", (int)l.id, num_points, dcost, cost);
		ma.markers.push_back(l);
		t.id = i+600;
		t.pose.position = points->at(i);
		t.pose.position.z = t.pose.position.z + 0.05; 
		t.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		t.text = std::to_string(cost);
		ma.markers.push_back(t);
	 
	}
	
	explore_pub_.publish(ma);
	return gain_info;
}




float nav3d::Features3D::no_return_cost(geometry_msgs::PoseStamped* p)
{
	pose_mutex_.lock();
	std::vector<geometry_msgs::PoseStamped> traj = robot_traj_;
	pose_mutex_.unlock();
	
	float xp = p->pose.position.x;
	float yp = p->pose.position.y;
	float zp = p->pose.position.z;
	
	float max_dist = 2.5;
	float cost = 0.0;
	float dmin = 1000.0;
	for(unsigned int i=0; i<traj.size(); i++)
	{
		float x = traj[i].pose.position.x;
		float y = traj[i].pose.position.y;
		float z = traj[i].pose.position.z;
		float dist = sqrt((xp-x)*(xp-x) + (yp-y)*(yp-y) + (zp-z)*(zp-z));
		if(dist < dmin)
			dmin = dist;
	}
	if(dmin > max_dist)
		dmin = max_dist;

	//The cost decreases linearly from 1 to 0 in a distance of 2 meters
	cost = ((max_dist-dmin)/max_dist);
	
	return cost;

}









/*
template<typename PointT> bool
   64 pcl::PCA<PointT>::initCompute () 
   65 {
   66   if(!Base::initCompute ())
   67   {
   68     PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] failed");
   69     return (false);
   70   }
   71   if(indices_->size () < 3)
   72   {
   73     PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] number of points < 3");
   74     return (false);
   75   }
   76   
   77   // Compute mean
   78   mean_ = Eigen::Vector4f::Zero ();
   79   compute3DCentroid (*input_, *indices_, mean_);  
   80   // Compute demeanished cloud
   81   Eigen::MatrixXf cloud_demean;
   82   demeanPointCloud (*input_, *indices_, mean_, cloud_demean);
   83   assert (cloud_demean.cols () == int (indices_->size ()));
   84   // Compute the product cloud_demean * cloud_demean^T
   85   Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());
   86   
   87   // Compute eigen vectors and values
   88   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (alpha);
   89   // Organize eigenvectors and eigenvalues in ascendent order
   90   for (int i = 0; i < 3; ++i)
   91   {
   92     eigenvalues_[i] = evd.eigenvalues () [2-i];
   93     eigenvectors_.col (i) = evd.eigenvectors ().col (2-i);
   94   }
   95   // If not basis only then compute the coefficients
   96 
   97   if (!basis_only_)
   98     coefficients_ = eigenvectors_.transpose() * cloud_demean.topRows<3> ();
   99   compute_done_ = true;
  100   return (true);
  101 }
*/






bool nav3d::Features3D::pose3dValid(geometry_msgs::PoseStamped* s)
{

	geometry_msgs::PoseStamped st = transformPoseTo(*s, robot_odom_frame_, false);
	
	pcl::PointXYZ sp;
	sp.x = st.pose.position.x;
	sp.y = st.pose.position.y;
	//We have to calculate the approximate height z based on the neighbors
	sp.z = st.pose.position.z;
	//printf("3dnav_features. pose3dValid point x:%.2f, y:%.2f, z:%.2f\n", sp.x, sp.y, sp.z);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;

	tree_mutex_.lock();

	//std::vector<int> pointIdxKSearch(10);
  	//std::vector<float> pointKSquaredDistance(10);
	//int found_k = kdtree_->nearestKSearch (sp, 10, pointIdxKSearch, pointKSquaredDistance);
	//printf("3denav_features. pose3dValid. kdtree ksearch found %i neighbors\n", found_k);
	//for(unsigned int j=0; j<found_k; j++)
	//	printf("point k %u, dist: %.2f\n", j, pointKSquaredDistance[j]);


	int found = kdtree_->radiusSearch(sp, (robot_radius_*1.0), pointIdxRadiusSearch, pointRadiusSquaredDistance);
	
	//int found2 = octree_->radiusSearch(sp, (robot_radius_*1.0), pointIdxRadiusSearch, pointRadiusSquaredDistance);
	
	if ( found > 0 )
	{
		//printf("3dnav_features. pose3dValid. kdtree radiussearch found %i neighbors in the radius %.2f\n", found, (robot_radius_*1.5));
		tree_mutex_.unlock();
				
		if(pointIdxRadiusSearch.size() < min_points_allowed_) {
			printf("3dnav_features. pose3dValid. %u points found less than minimum: %i\n", (unsigned int)pointIdxRadiusSearch.size(), min_points_allowed_);
			return false;
		}

		// Generate pointcloud data
  		//patch->width = (int)pointIdxRadiusSearch.size();
  		//patch->height = 1;
  		//patch->points.resize(patch->width * patch->height);

		float new_height = 0.0;
		int cont = 0;
		for (size_t i=0; i < pointIdxRadiusSearch.size(); ++i)
		{
			//printf("3dnav_features. pose3dValid. radiusSquare[%u] = %.2f\t", (unsigned int)i, pointRadiusSquaredDistance[i]);
			if((sqrt(pointRadiusSquaredDistance[i])) <= 0.25)
			{
				cont++;
				new_height += pcl_cloud_.points[ pointIdxRadiusSearch[i] ].z;
			}
			//patch->points[i] = pcl_cloud_.points[ pointIdxRadiusSearch[i] ];
		}
		if(cont > 0) {
			new_height = new_height/(float)cont;
			s->pose.position.z = new_height;
		}
	
	} else {
		tree_mutex_.unlock();
		printf("nav_features3d. pose3dValid. kdtree radiussearch found %i neighbors in the radius %.2f\n", found, robot_radius_);
		return false;
	}

	

	//pca_->setInputCloud(patch);
	//pca_->setIndices(const IndicesPtr &indices)
	//pca_->setIndices (const IndicesConstPtr &indices)
	//typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr

	//pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr indices(new std::vector<int>(pointIdxRadiusSearch));
	//printf("3dnav_features. pose3dValid. pind[0]:%i, pind[1]:%i, pind[2]:%i\n", pointIdxRadiusSearch[0], pointIdxRadiusSearch[1], pointIdxRadiusSearch[2]); 
	//printf("3dnav_features. pose3dValid. ptr[0]:%i, ptr[1]:%i, ptr[2]:%i\n", indices->at(0), indices->at(1), indices->at(2));
	tree_mutex_.lock();
	//pca_->setIndices(indices);
	

	Eigen::Vector4f mean;
	Eigen::Vector3f evals;
	Eigen::Matrix3f evecs;

	try{
		//mean = pca_->getMean();
		//evals = pca_->getEigenValues();
		//evecs = pca_->getEigenVectors();


		mean = Eigen::Vector4f::Zero ();
		pcl::compute3DCentroid (pcl_cloud_, *indices, mean);   //input ->const pcl::PointCloud< PointT > &cloud
		// Compute demeanished cloud
		Eigen::MatrixXf cloud_demean;
		pcl::demeanPointCloud (pcl_cloud_, *indices, mean, cloud_demean);
		assert (cloud_demean.cols () == int (indices->size ()));
		// Compute the product cloud_demean * cloud_demean^T
		Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());
   
		// Compute eigen vectors and values
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (alpha);
		// Organize eigenvectors and eigenvalues in ascendent order
		for (int i = 0; i < 3; ++i)
		{
			evals[i] = evd.eigenvalues () [2-i];
			evecs.col(i) = evd.eigenvectors ().col (2-i);
		}



		tree_mutex_.unlock();
		//printf("3dnav_features. pose3dValid. mean x:%.2f, y:%.2f, z:%.2f\n", mean[0], mean[1], mean[2]);

	} catch(pcl::InitFailedException e)  {
		ROS_WARN("nav_features3d. pose3dValid. Calculation of mean, eigenvectors and eigenvalues failed!");
		tree_mutex_.unlock();
		return false;
	}

	//we can check how far is the centroid from the point sp
	//float dx = sp.x - mean[0];
	//float dy = sp.y - mean[1];
	//float dz = sp.z - mean[2];
	//float point_dist = sqrt(dx*dx + dy*dy + dz*dz); 


	Eigen::Quaternion<float> q(evecs);

	auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2); //0->yaw, 1->pitch, 2->roll
	double yaw = euler[0]; 
	double pitch = euler[1]; 
	double roll = euler[2];


	//Identify the lowest eigenvalue
	//TODO: the eigen values are ordered. 
	//The smallest is the last one. CHECK THIS! 
	
	//Normal vector		 
	float nx = (float)evecs(0,2);
	float ny = (float)evecs(1,2);
	float nz = (float)evecs(2,2);

 
	pose_mutex_.lock();
	geometry_msgs::PoseStamped rpose = robot_pose_;
	pose_mutex_.unlock();



	//Check the direction of the normal
	// correct if n . (pv - Pi) > 0
	// n  -> evec with the lowest eigenvalue (= normal vector)
	// pv -> (0,0,0) Point of view
	// pi -> mean
	// See if we need to flip any plane normals
	/*		
	// Dot product between the (viewpoint - point) and the plane normal
	float cos_theta = ((rpose.pose.position.x-mean[0]) * nx + (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz); 
	// Flip the plane normal
	if (cos_theta < 0)
	{
		pcl::PointXYZ p;
		p.x = mean[0];
		p.y = mean[1];
		p.z = mean[2];
		pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y, rpose.pose.position.z, nx, ny, nz);

		Eigen::Vector3d norm(nx, ny, nz);
		norm = norm.normalized();
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
					
		Eigen::Quaternion<double> q(R);
		tf::Quaternion qu(q.x(), q.y(), q.z(), q.w());
		qu.normalize();
		tf::Matrix3x3 m(qu);
      	m.getRPY(roll, pitch, yaw);
	}
	*/
	/*
	//float curvature = fabsf(evals(0) / (evals(0) + evals(1) + evals(2)) );

	
	float sum = 0;
	for (size_t i = 0; i < patch->size(); ++i)
	{
		pcl::PointXYZ p = patch->at(i);   //(i,0);
		float s = (p.x - mean[0]) + (p.y - mean[1]);
		float sq = s * s;
		sum += sq;
	}
	//float me = sum / patch->size();
	float variance = sum / (patch->size () - 1);
	float stddev = sqrt (variance);
	*/

	//if(pitch > pitch_high_ || pitch < pitch_low_ || yaw > roll_high_ || yaw < roll_low_ || fabs(evals(2)) > roughness_) { 
	//if(fabs(pitch) < (M_PI/4.0) || fabs(pitch) > (3.0*M_PI/4.0) || fabs(yaw) < (M_PI/4.0) || fabs(yaw) > (3.0*M_PI/4.0) || fabs(evals(2)) > roughness_) {  
	if((fabs(pitch) > pitch_low_ && fabs(pitch) < pitch_high_) || (fabs(yaw) > roll_low_ && fabs(yaw) < roll_high_) || fabs(evals(2)) > roughness_) {
	//if( fabs(evals(2)) > roughness_) {
		//printf("pose3d INVALID. pitch:%.2f, yaw:%.2f, r:%.2f, upper:%.2f, lower:%.2f, roughness:%.2f\n", fabs(pitch), fabs(yaw), fabs(evals(2)), pitch_high2_, pitch_low2_, roughness_);
		return false;
	} else {
		//printf("pose3d VALID. pitch:%.2f, yaw:%.2f, r:%.2f, upper:%.2f, lower:%.2f, roughness:%.2f\n", fabs(pitch), fabs(yaw), fabs(evals(2)), pitch_high2_, pitch_low2_, roughness_);
		
		
		// mean[2] = height z in odom coordinates. I need to transform it to base frame and update this value in pose s.
		geometry_msgs::PoseStamped meanpose;
		meanpose.header.frame_id = rpose.header.frame_id;
		meanpose.header.stamp = ros::Time::now();
		meanpose.pose.position.x = mean[0];
		meanpose.pose.position.y = mean[1];
		meanpose.pose.position.z = mean[2];
		meanpose.pose.orientation = rpose.pose.orientation;
		geometry_msgs::PoseStamped sbase = transformPoseTo(meanpose, robot_base_frame_, false);
		s->pose.position.z = sbase.pose.position.z;
		
		return true;
	}


}










/**
* TODO TRY TO NORMALIZE THE VALUE OF THE FEATURES!!!
*/
std::vector<float> nav3d::Features3D::getFeatures(geometry_msgs::PoseStamped* s)
{
	std::vector<float> features;
	
	if(feature_set_ == 1)
		features.assign(4, 0.0);  // features[4] = {valid, pitch, roll, roughness}
	else
		features.assign(8, 0.0); //features[8] = {valid, pitch, roll, roughness, point_dist, stddev, num_points, goal_dist} //curvature	 

	geometry_msgs::PoseStamped st = transformPoseTo(*s, robot_odom_frame_, false);
	
	pcl::PointXYZ sp;
	sp.x = st.pose.position.x;
	sp.y = st.pose.position.y;
	//We have to calculate the approximate height z based on the neighbors
	sp.z = st.pose.position.z;


	//pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;

	tree_mutex_.lock();

	//std::vector<int> pointIdxKSearch(10);
  	//std::vector<float> pointKSquaredDistance(10);
	//int found_k = kdtree_->nearestKSearch (sp, 10, pointIdxKSearch, pointKSquaredDistance);
	//printf("3denav_features. pose3dValid. kdtree ksearch found %i neighbors\n", found_k);
	//for(unsigned int j=0; j<found_k; j++)
	//	printf("point k %u, dist: %.2f\n", j, pointKSquaredDistance[j]);


	int found = kdtree_->radiusSearch(sp, (robot_radius_*1.0), pointIdxRadiusSearch, pointRadiusSquaredDistance);
	float num_points = (float)found;
	if ( found > 0 )
	{
		//printf("3dnav_features. pose3dValid. kdtree radiussearch found %i neighbors in the radius %.2f\n", found, (robot_radius_*1.5));
		tree_mutex_.unlock();
				
		if(pointIdxRadiusSearch.size() < min_points_allowed_) {
			//printf("3dnav_features. pose3dValid. indices less than 3\n");
			return features;
		}

		// Generate pointcloud data
  		//patch->width = (int)pointIdxRadiusSearch.size();
  		//patch->height = 1;
  		//patch->points.resize(patch->width * patch->height);

		float new_height = 0.0;
		int cont = 0;
		float sum = 0;
		for (size_t i=0; i < pointIdxRadiusSearch.size(); ++i)
		{
			//printf("3dnav_features. pose3dValid. radiusSquare[%u] = %.2f\t", (unsigned int)i, pointRadiusSquaredDistance[i]);
			if((sqrt(pointRadiusSquaredDistance[i])) <= 0.25)
			{
				cont++;
				new_height += pcl_cloud_.points[ pointIdxRadiusSearch[i] ].z;
			}
			//patch->points[i] = pcl_cloud_.points[ pointIdxRadiusSearch[i] ];

		}
		if(cont > 0) {
			new_height = new_height/(float)cont;
			s->pose.position.z = new_height;
		}
	
	} else {
		tree_mutex_.unlock();
		printf("nav_features3d. getFeatures. kdtree radiussearch found %i neighbors in the radius %.2f\n", found, robot_radius_);
		return features;
	}


	//pca_->setInputCloud(patch);
	//pca_->setIndices(const IndicesPtr &indices)
	//pca_->setIndices (const IndicesConstPtr &indices)
	//typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr

	//pcl::IndicesPtr indices(new std::vector<int>);
	pcl::IndicesPtr indices(new std::vector<int>(pointIdxRadiusSearch));
	tree_mutex_.lock();
	//pca_->setIndices(indices);

	Eigen::Vector4f mean;
	Eigen::Vector3f evals;
	Eigen::Matrix3f evecs;

	try{
		//mean = pca_->getMean();
		//evals = pca_->getEigenValues();
		//evecs = pca_->getEigenVectors();
		mean = Eigen::Vector4f::Zero ();
		pcl::compute3DCentroid (pcl_cloud_, *indices, mean);   //input ->const pcl::PointCloud< PointT > &cloud
		// Compute demeanished cloud
		Eigen::MatrixXf cloud_demean;
		pcl::demeanPointCloud (pcl_cloud_, *indices, mean, cloud_demean);
		assert (cloud_demean.cols () == int (indices->size ()));
		// Compute the product cloud_demean * cloud_demean^T
		Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());
   
		// Compute eigen vectors and values
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (alpha);
		// Organize eigenvectors and eigenvalues in ascendent order
		for (int i = 0; i < 3; ++i)
		{
			evals[i] = evd.eigenvalues () [2-i];
			evecs.col(i) = evd.eigenvectors ().col (2-i);
		}
		tree_mutex_.unlock();

	} catch(pcl::InitFailedException e)  {
		ROS_WARN("nav_features3d. getFeatures. PCA failed!");
		tree_mutex_.unlock();
		return features;
	}


	float sum = 0;
	float max_d = 0;;
	for (size_t i=0; i < pointIdxRadiusSearch.size(); ++i)
	{
		float s = (pcl_cloud_.points[ pointIdxRadiusSearch[i] ].x - mean[0]) + (pcl_cloud_.points[ pointIdxRadiusSearch[i] ].y - mean[1]);
		float sq = s * s;
		sum += sq;
		
		//For normalization
		float n = robot_radius_ + robot_radius_;
		float sqn = n * n;
		max_d += sqn;
	}
	//float me = sum / pointIdxRadiusSearch.size();
	float variance = sum / (pointIdxRadiusSearch.size() - 1);
	float stddev = sqrt (variance);
	
	//for normalization
	float varn = max_d /  (pointIdxRadiusSearch.size() - 1);
	float max_stddev = sqrt(varn); 	 


	//we can check how far is the centroid from the point sp
	float dx = sp.x - mean[0];
	float dy = sp.y - mean[1];
	float dz = sp.z - mean[2];
	float point_dist = sqrt(dx*dx + dy*dy + dz*dz); 


	Eigen::Quaternion<float> q(evecs);

	auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2); //0->yaw, 1->pitch, 2->roll
	double yaw = euler[0]; 
	double pitch = euler[1]; 
	double roll = euler[2];


	//Identify the lowest eigenvalue
	//TODO: the eigen values are ordered. 
	//The smallest is the last one. CHECK THIS! 
	
	//Normal vector		 
	float nx = (float)evecs(0,2);
	float ny = (float)evecs(1,2);
	float nz = (float)evecs(2,2);

 
	pose_mutex_.lock(); 
	geometry_msgs::PoseStamped rpose = robot_pose_;
	pose_mutex_.unlock();



	//Check the direction of the normal
	// correct if n . (pv - Pi) > 0
	// n  -> evec with the lowest eigenvalue (= normal vector)
	// pv -> (0,0,0) Point of view
	// pi -> mean
	// See if we need to flip any plane normals
			
	// Dot product between the (viewpoint - point) and the plane normal
	/*float cos_theta = ((rpose.pose.position.x-mean[0]) * nx + (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz); 
	// Flip the plane normal
	if (cos_theta < 0)
	{
		pcl::PointXYZ p;
		p.x = mean[0];
		p.y = mean[1];
		p.z = mean[2];
		pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y, rpose.pose.position.z, nx, ny, nz);

		Eigen::Vector3d norm(nx, ny, nz);
		norm = norm.normalized();
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
					
		Eigen::Quaternion<double> q(R);
		tf::Quaternion qu(q.x(), q.y(), q.z(), q.w());
		qu.normalize();
		tf::Matrix3x3 m(qu);
      	m.getRPY(roll, pitch, yaw);
	} */


	float curvature = fabsf(evals(0) / (evals(0) + evals(1) + evals(2)) );
	
	
	
	
	//Normalize pitch and yaw
	//if(fabs(pitch) > M_PI/2.0)
	//	pitch = M_PI - fabs(pitch);
	//pitch = fabs(pitch)/M_PI;
	//if(fabs(yaw) > M_PI/2.0)
	//	yaw = M_PI - yaw;
	//yaw = yaw/M_PI;
	
	
	float zero = M_PI/2.0;
	if(fabs(pitch) > zero)
		pitch = M_PI - fabs(pitch); 
	pitch = fabs(pitch)/zero;
	if(fabs(yaw) > zero)
		yaw = M_PI - fabs(yaw); 
	yaw = fabs(yaw)/zero;
			
	
	//Normalize roughness
	//Maximum value of roughness?????
	float max_rough = 1.5; //3.0;
	float rough = fabs(evals(2));
	if (rough > max_rough) rough = max_rough;
	rough = rough/max_rough; 
	
	//normalize point dist. max dist = robot_radius_
	point_dist = point_dist / robot_radius_;
	
	//Normalize stddev
	stddev = 1.0 - (stddev / max_stddev);
	
	//Normalize num_points
	float area_circ = M_PI*(robot_radius_*robot_radius_);
	float max_points = area_circ * 100.0; //1 point per centimeter = 100p/m  //30.0;
	if(num_points > max_points) num_points = max_points;
	num_points = 1.0 - (num_points/max_points);
	
	float dist_cost = goalDistFeature(s);


	//Things to evaluate the position:
	// - Inclination (pitch and roll)
	// - roughness (lowest eigenvalue)
	// - distance between the mean and the point evaluated
	// - curvature of the plain
	// - dispersion in the sfere, stddev in xy? High stddev is better
	// - TODO: add a point density feature in the area
	
	//features[8] = {valid, pitch, roll, roughness, point_dist, curvature, stddev, num_points}
	features[0] = 1.0;
	features[1] = pitch;
	features[2] = yaw; //the frame is turned, so the roll is the yaw now
	features[3] = rough;
	if(feature_set_ == 1)
		return features;
	
	features[4] = point_dist; 
	//features[5] = curvature;    //not normalized
	features[5] = stddev;
	features[6] = num_points; // point_density; ????
	features[7] = dist_cost;
	return features;
	
}





/*
 void nav3d::Features3D::reconfigureCB(navigation_features::nav_featuresConfig &config, uint32_t level){

    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//configuration_mutex_.lock();
    
    upo_featureset_ = config.upo_featureset;
	//use_global_map_ = config.use_global_map;
	sigmas_[0] = config.stddev_person_front;
	sigmas_[1] = config.stddev_person_aside;
	sigmas_[2] = sigmas_[1];
	sigmas_[3] = sigmas_[1];
	sigmas_[4] = config.stddev_person_right;
	sigmas_[5] = sigmas_[4]/2.5;
	grouping_ = config.enable_grouping;
	stddev_group_ = config.stddev_group;
	grouping_distance_ = config.grouping_distance;
	it_id_ = config.interaction_target_id;
	it_remove_gauss_ = config.it_remove_gaussian;
	//printf("upo_featureset. it_id:%i\n", it_id_);

	//configuration_mutex_.unlock();
}*/



//Service
/*bool nav3d::Features3D::setWeightsService(navigation_features::SetWeights::Request  &req, navigation_features::SetWeights::Response &res)
{
	setWeights(req.weights);
	return true;
}
*/


//Service
/*bool nav3d::Features3D::isPoseValidService(navigation_features::PoseValid::Request &req, navigation_features::PoseValid::Response &res)
{
	geometry_msgs::PoseStamped p = req.pose;
	res.ok = poseValid(&p);
}*/

//Service
/*bool nav3d::Features3D::setScenarioService(navigation_features::SetScenario::Request &req, navigation_features::SetScenario::Response &res)
{
	setScenario(req.obstacles, req.people, req.goal); 

	return true;
}*/

//Service
/*bool nav3d::Features3D::getFeatureCountService(navigation_features::GetFeatureCount::Request &req, navigation_features::GetFeatureCount::Response &res)
{ 
	res.fc = getPathFeatureCount(&req.path);
	return true;
}*/

//Service
/*bool nav3d::Features3D::initializeWeightsService(navigation_features::InitWeights::Request &req, navigation_features::InitWeights::Response &res)
{
	vector<float> w;
	if(req.random) {
		srand(time(NULL));
		for(int i=0; i<w_.size(); i++){
			float v = rand() % 10 + 2;
			w.push_back(v);
		}
	} else {
		w.assign((int)w_.size(), 0.0);
	}

	//Normalize weights
	if(req.normalize && req.random) {	
		float total = accumulate(w.begin(), w.end(), 0);
		for(unsigned int i=0; i<w.size(); i++) 
			w[i] = w[i]/total;
	}
	w_ = w;
	res.weights = w;
	return true;
}*/




/*void nav3d::Features3D::setScenario(sensor_msgs::PointCloud2 obs, geometry_msgs::PoseStamped goal)
{
	setGoal(goal);
	//setPeople(people);
	setObstacles(obs);
}*/


/*
void nav3d::Features3D::setObstacles(sensor_msgs::PointCloud2 obs)
{
	sensor_msgs::PointCloud2 lcloud;
	obs.header.stamp = ros::Time();
	try{  
		if(use_global_map_) {
			if(!pcl_ros::transformPointCloud("/map", obs, lcloud, *tf_listener_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		} else {
			if(!pcl_ros::transformPointCloud("/base_link", obs, lcloud, *tf_listener_))
				ROS_WARN("TransformPointCloud failed!!!!!");
		}			

	} catch (tf::TransformException ex){
		ROS_WARN("NAV FEATURES. pcCallback. TransformException: %s", ex.what());
	}
	
	cloudMutex_.lock();
	cloud_ = lcloud;
	cloudMutex_.unlock();
}
*/









/*
void nav3d::Features3D::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//printf("NavFeatures. Receiving goal!\n");
	setGoal(*msg);
}*/






void nav3d::Features3D::setGoal(geometry_msgs::PoseStamped g) {

	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//printf("NavFeatures. Receiving goal point!\n");
	goal_ = g;
}



float nav3d::Features3D::goalDistFeature(geometry_msgs::PoseStamped* s)  {
	
	//if we don't have a goal yet - Exploration
	if(goal_.header.frame_id.empty()){
			//printf("features3d. goalDistFeature zero. goal empty\n");
			return (float) 0.0;
	}
	
	//if the frame_id are different, transform s to goal frame
	geometry_msgs::PoseStamped p = transformPoseTo(*s, goal_.header.frame_id, false);
	float dx = goal_.pose.position.x - p.pose.position.x;
	float dy = goal_.pose.position.y - p.pose.position.y;
	float dz = goal_.pose.position.z - p.pose.position.z;
	float dist = (float)(sqrt(dx*dx + dy*dy + dz*dz)/max_planning_dist_);
	if(dist > 1.0) {
		//printf("NavFeatures. goal dist feature %.2f > 1.0. d: %.2f, max_dist: %.2f\n", dist, (float)sqrt(dx*dx + dy*dy), max_planning_dist_);
		dist = 1.0;
	}
	return dist;
}




void nav3d::Features3D::setWeights(vector<float> we) {
	printf("nav_features3d. Setting weights: \n");
	w_.clear();
	for(unsigned int i=0; i<we.size(); i++) 
	{
		//if(we[i] != 0.0) {
			w_.push_back(we[i]);
			//printf("we%u: %.3f, w_%u: %.3f\n", (i+1), we[i], (i+1), w_[i]);
		//}
	}
	
	for(unsigned int i=0; i<w_.size(); i++) 
	{
		printf("w_%u: %.3f\n", (i+1), w_[i]);
	}
	
}






float nav3d::Features3D::getCost(geometry_msgs::PoseStamped* s)
{
	//boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	
	//configuration_mutex_.lock();
	
	vector<float> features = getFeatures(s);

	if(features[0] == 0.0) {
		printf("nav_features3d. getCost. Returning maximum cost\n");
		return (1.0);
	}
	float cost = 0.0;
	for(unsigned int i=1; i<w_.size(); i++) 
	{
		cost += (w_[i] * features[i]);
	}

	//printf("3dnav_features. get Cost: %.3f\n", cost);
	return cost;
}
	











geometry_msgs::PoseStamped nav3d::Features3D::transformPoseTo(geometry_msgs::PoseStamped pose_in, string frame_out, bool usetime)
{
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
		
	geometry_msgs::PoseStamped pose_out;
	
	geometry_msgs::Quaternion q = in.pose.orientation;
	if(!isQuaternionValid(q))
	{
		ROS_WARN("nav_features3d. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	try {
		tf_listener_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("nav_features3d. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	return pose_out;
}




bool nav3d::Features3D::isQuaternionValid(const geometry_msgs::Quaternion q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
		ROS_ERROR("nav_features3d. Quaternion has infs!!!!");
		return false;
    }
    if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
		ROS_ERROR("nav_features3d. Quaternion has nans !!!");
		return false;
	}
	
	if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
		ROS_ERROR("nav_features3d. Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
		return false;
	}

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("nav_features3d. Quaternion has length close to zero... discarding.");
      return false;
    }

	/*
    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }*/

    return true;
}



float nav3d::Features3D::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}








