
#include <navigation_features_3d/nav_features3d.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>
#include <sys/time.h>
#include <numeric>
#include <string>


using namespace std;


nav3d::Features3D::Features3D()
{
  tf_ = new tf2_ros::Buffer();
  tf2_ros::TransformListener tfListener(*tf_);
  // tf_ = new tf::TransformListener(ros::Duration(10));
  size_x_ = 5.0 * 2.0;  // m
  size_y_ = 5.0 * 2.0;  // m
  size_z_ = 5.0 * 2.0;  // m
  // resolution_ = res;		//m/cell
  current_size_ = size_x_ / 2.0;
  min_size_ = current_size_;
  max_size_ = 11.0;
  max_planning_dist_ = sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));

  setParams(std::string("navigation_features_3d"));
}


nav3d::Features3D::Features3D(std::string name, tf2_ros::Buffer* tf, float size_x,
                              float size_y, float size_z)
{
  tf_ = tf;
  size_x_ = size_x * 2.0;  // m
  size_y_ = size_y * 2.0;  // m
  size_z_ = size_z * 2.0;  // m
  current_size_ = size_x_ / 2.0;
  min_size_ = current_size_;
  max_size_ = 11.0;
  // resolution_ = res;		//m/cell
  max_planning_dist_ = sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));

  setParams(name);
}



nav3d::Features3D::Features3D(std::string name, tf2_ros::Buffer* tf,
                              vector<geometry_msgs::Point>* footprint, float size_x,
                              float size_y, float size_z)
{
  tf_ = tf;
  // robot_radius_ = insc_radius;
  size_x_ = size_x * 2.0;  // m
  size_y_ = size_y * 2.0;  // m
  size_z_ = size_z * 2.0;  // m
  current_size_ = size_x_ / 2.0;
  min_size_ = current_size_;
  max_size_ = 11.0;
  // resolution_ = res;		//m/cell
  max_planning_dist_ = sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));

  myfootprint_ = footprint;


  setParams(name);
}



void nav3d::Features3D::setParams(std::string name)
{
  ROS_INFO("%s. SETTING PARAMETERS...", name.c_str());
  // Read the ROS params from the server
  ros::NodeHandle n(("~/" + name));

  name_ = name;

  // Dynamic reconfigure
  // dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n);
  // //ros::NodeHandle("~")
  // dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb
  // = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);

  feature_set_ = 1;
  n.getParam("feature_set", feature_set_);

  // LOAD THE WEIGHTS OF THE COST FUNCTION FOR NAVIGATION
  w_.clear();
  w_.push_back(1.0);  // weight for the valid feature (boolean [0,1])


  if (feature_set_ == 1)
  {
    unsigned int i = 1;
    while (i <= 3)
    {
      char buf[10];
      sprintf(buf, "w%u", i);
      string st = string(buf);

      if (n.hasParam(st.c_str()))
      {
        double wg = 0.0;
        n.getParam(st.c_str(), wg);
        w_.push_back((float)wg);
        printf("%s. weight %u= %.3f loaded\n", name_.c_str(), i, wg);
      }
      i++;
    }
  }
  else
  {
    bool ok = true;
    unsigned int i = 1;
    while (ok)
    {
      char buf[10];
      sprintf(buf, "w%u", i);
      string st = string(buf);

      if (n.hasParam(st.c_str()))
      {
        double wg = 0.0;
        n.getParam(st.c_str(), wg);
        w_.push_back((float)wg);
        printf("%s. weight %u= %.3f loaded\n", name_.c_str(), i, wg);
      }
      else
      {
        // printf("param '%s' not found\n", st.c_str());
        ok = false;
      }
      i++;
    }
  }


  // LOAD THE WEIGHTS OF THE COST FUNCTION FOR EXPLORATION
  wexp_.clear();
  bool ok = true;
  unsigned int i = 1;
  while (ok)
  {
    char buf[10];
    sprintf(buf, "wexp%u", i);
    string st = string(buf);

    if (n.hasParam(st.c_str()))
    {
      double wg = 0.0;
      n.getParam(st.c_str(), wg);
      wexp_.push_back((float)wg);
      printf("%s. exploration weight %u= %.3f loaded\n", name_.c_str(), i, wg);
    }
    else
    {
      ok = false;
    }
    i++;
  }



  string cloud_topic;
  n.param<string>("pointcloud_topic", cloud_topic, string("/scan360/point_cloud"));
  printf("\n %s. topic subscribed: %s \n", name_.c_str(), cloud_topic.c_str());

  double pitch_max = M_PI / 2.0;
  double roll_max = M_PI / 2.0;
  roughness_ = 0.03;
  n.getParam("max_pitch_inclination", pitch_max);
  n.getParam("max_roll_inclination", roll_max);
  n.getParam("max_roughness", roughness_);
  printf("%s. max pitch inclination:%.2f, max roll inclination:%.2f \n", name_.c_str(),
         pitch_max, roll_max);
  printf("%s. max roughness:%.2f\n", name_.c_str(), roughness_);
  min_points_allowed_ = 20;
  n.getParam("min_points_allowed", min_points_allowed_);


  exp_min_dist_goals_ = 1.0;
  n.getParam("exp_min_dist_goals", exp_min_dist_goals_);

  // ros::NodeHandle nh("~");
  n.getParam("robot_base_frame", robot_base_frame_);
  n.getParam("robot_odom_frame", robot_odom_frame_);
  n.getParam("robot_odom_topic", robot_odom_topic_);


  threshold_frontier_ = 0.7;
  n.getParam("threshold_frontier", threshold_frontier_);
  printf("%s. threshold_frontier:%.2f\n", name_.c_str(), threshold_frontier_);
  adaptative_threshold_ = false;
  n.getParam("adaptative_threshold", adaptative_threshold_);
  printf("%s. adaptative_threshold:%i\n", name_.c_str(), (int)adaptative_threshold_);

  num_points_saturation_ = 3400.0;
  n.getParam("num_points_saturation", num_points_saturation_);
  // printf("%s. num_points_saturation:%i\n", name_.c_str(), (num_points_saturation_);


  numpoint_cost_limit_ = threshold_frontier_ - 0.05;
  // n.getParam("numpoint_cost_limit", numpoint_cost_limit_);
  // printf("%s. numpoint_cost_limit:%.2f\n", name_.c_str(), numpoint_cost_limit_);
  percentage_limit_ = 0.80;
  n.getParam("percentage_limit", percentage_limit_);
  printf("%s. percentage_limit:%.2f\n", name_.c_str(), percentage_limit_);

  n.getParam("min_planning_size", min_size_);
  printf("%s. min_planning_size: %.2f\n", name_.c_str(), min_size_);
  n.getParam("max_planning_size", max_size_);
  printf("%s. max_planning_size: %.2f\n", name_.c_str(), max_size_);


  nfe_ = false;
  n.getParam("nf_exploration", nfe_);
  if (nfe_)
  {
    printf("NEIREST FRONTIER EXPLORATION WILL BE EMPLOYED!!!\n");
  }

  bfe_ = false;
  n.getParam("bf_exploration", bfe_);

  if (nfe_ && bfe_)
    printf("NFE and BFE can not be used at the same time...\n");

  if (bfe_)
  {
    nfe_ = false;
    printf("BIGGEST FRONTIER EXPLORATION WILL BE EMPLOYED!!!\n");
  }



  exp_pc_service_name_ = "";
  n.getParam("exp_pc_service_name", exp_pc_service_name_);

  ros::NodeHandle nh;
  exp_client_ = nh.serviceClient<pcl_filters::GetFilteredPC>(exp_pc_service_name_.c_str());


  // pitch_low_ = -M_PI/2.0 - pitch_max;
  // pitch_low2_ = M_PI/2.0 - pitch_max;
  // pitch_high_ = -M_PI/2.0 + pitch_max;
  // pitch_high2_ = M_PI/2.0 + pitch_max;
  // roll_low_ = -M_PI/2.0 - roll_max;
  // roll_low2_ = M_PI/2.0 - roll_max;
  // roll_high_ = -M_PI/2.0 + roll_max;
  // roll_high2_ = M_PI/2.0 + roll_max;
  pitch_low_ = pitch_max;
  pitch_high_ = M_PI - pitch_max;
  roll_low_ = roll_max;
  roll_high_ = M_PI - roll_max;


  pitch_low2_ = M_PI / 2.0 - 0.80;   // pitch_max;
  pitch_high2_ = M_PI / 2.0 + 0.80;  // pitch_max;
  roll_low2_ = M_PI / 2.0 - 0.80;    // roll_max;
  roll_high2_ = M_PI / 2.0 + 0.80;   // roll_max;


  robot_radius_ = 0.25;
  n.getParam("robot_circuns_radius", robot_radius_);



  visited_region_enabled_ = true;
  n.getParam("visited_region_enabled", visited_region_enabled_);
  printf("%s. visited_region_enabled: %i\n", name_.c_str(), visited_region_enabled_);

  remove_wall_frontier_enabled_ = true;
  n.getParam("remove_wall_frontier_enabled", remove_wall_frontier_enabled_);
  printf("%s. remove_wall_frontier_enabled: %i\n", name_.c_str(), remove_wall_frontier_enabled_);

  variable_size_enabled_ = true;
  n.getParam("variable_size_enabled", variable_size_enabled_);
  printf("%s. variable_size_enabled: %i\n", name_.c_str(), variable_size_enabled_);



  visualize_visited_reg_ = true;
  n.getParam("visualize_visited_reg", visualize_visited_reg_);
  visualize_frontiers_ = true;
  n.getParam("visualize_frontiers", visualize_frontiers_);
  visualize_wall_leaves_ = true;
  n.getParam("visualize_wall_leaves", visualize_wall_leaves_);



  kdtree_ = new pcl::KdTreeFLANN<pcl::PointXYZ>();
  // kdtree_exp_ = new pcl::KdTreeFLANN<pcl::PointXYZ>();
  // pca_ =  new pcl::PCA<pcl::PointXYZ>();
  vgc_ = new pcl::VoxelGridCovariance<pcl::PointXYZ>();
  cell_size_ = 0.07;
  n.getParam("cell_size_grid_exp", cell_size_);
  vgc_->setLeafSize(cell_size_, cell_size_,
                    cell_size_);  // 0.01f, 0.01f, 0.01f	//inherited from pcl::VoxelGrid
  vgc_->setMinPointPerVoxel(4);


  wall_cloud_.width = 0;
  wall_cloud_.height = 1;
  wall_cloud_.is_dense = false;
  wall_cloud_.header.frame_id = robot_odom_frame_;


  // octree_resolution_ = 128.0f;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc(128.0);
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr oc(new
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution_));
  // octree_ = oc;
  // octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution_);
  // octree_ = new pcl::search::Octree<pcl::PointXYZ>(octree_resolution_);
  // octree_->setResolution(octree_resolution_);
  // octree_.setResolution(octree_resolution_);



  // goal_sub_ = nh_.subscribe("/rrt_goal", 1, &Features3D::goalCallback, this);

  cloud_sub_ = nh_.subscribe(cloud_topic, 1, &Features3D::cloudCallback, this);

  pose_sub_ = nh_.subscribe(robot_odom_topic_, 1, &Features3D::poseCallback, this);

  explore_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrt_explore_costs", 1);

  wall_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_points", 2);

  frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("frontiers_points", 2);

  visited_pub_ = nh_.advertise<visualization_msgs::Marker>("visited_regions", 2);

  pc_wall_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_pointcloud", 1);


  rrt_time_pub_ = nh_.advertise<std_msgs::Float32>("RRT_ros_wrapper/ChangeRRTSolveTime", 1);


  // pub_gaussian_markers_ =
  // n.advertise<visualization_msgs::MarkerArray>("gaussian_markers", 5);

  // ros::NodeHandle nd("navigation_features");
  // ros::ServiceServer service = nd.advertiseService("setApproachingIT",
  // setApproachingIT);
  // loss_srv_ = nd.advertiseService("set_use_loss_func",
  // &features::NavFeatures::setLossService, this);
  // valid_srv_ = nd.advertiseService("is_pose_valid",
  // &features::NavFeatures::isPoseValidService, this);
  // weights_srv_ = nd.advertiseService("setWeights",
  // &features::NavFeatures::setWeightsService, this);
  // init_weights_srv_ = nd.advertiseService("initWeights",
  // &features::NavFeatures::initializeWeightsService, this);
  // scenario_srv_ = nd.advertiseService("setScenario",
  // &features::NavFeatures::setScenarioService, this);
  // features_srv_ = nd.advertiseService("getPathFeatureCount",
  // &features::NavFeatures::getFeatureCountService, this);


  // Dynamic reconfigure
  // dsrv_ = new dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>(n);
  // //ros::NodeHandle("~")
  // dynamic_reconfigure::Server<navigation_features::nav_featuresConfig>::CallbackType cb
  // = boost::bind(&NavFeatures::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);
}


nav3d::Features3D::~Features3D()
{
  delete kdtree_;
  delete vgc_;
  // delete kdtree_exp_;
  // delete octree_;
}



// Point_cloud2 callback
void nav3d::Features3D::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Transform the coordinates of the pointcloud
  sensor_msgs::PointCloud2 local;
  if (!pcl_ros::transformPointCloud(robot_odom_frame_, *msg, local, *tf_))
  {
    ROS_WARN("%s. CloudCallback. TransformPointCloud failed!!!!!", name_.c_str());
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *cloud);

    // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> oc(octree_resolution_);

    tree_mutex_.lock();
    pcl::fromROSMsg(local, *cloud);
    pcl_cloud_ = *cloud;
    // ros::WallTime t1 = ros::WallTime::now();
    kdtree_->setInputCloud(cloud);
    // ros::WallTime t2 = ros::WallTime::now();
    // pca_->setInputCloud(cloud);

    // ros::WallTime t3 = ros::WallTime::now();
    // octree_->deleteTree();
    // octree_->setInputCloud(cloud);
    // octree_->addPointsFromInputCloud();
    // ros::WallTime t4 = ros::WallTime::now();
    // double t_kdtree = (t2-t1).toSec();
    // double t_octree = (t4-t3).toSec();
    // printf("cloudCallback kdtree: %.4f, octree:%.4f\n", t_kdtree, t_octree);
    tree_mutex_.unlock();
  }
}



void nav3d::Features3D::poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
  geometry_msgs::PoseStamped pose;

  pose.header = msg->header;
  pose.pose = msg->pose.pose;

  geometry_msgs::PoseStamped pose_out;
  // if(!pc_frame.empty())
  pose_out = transformPoseTo(pose, robot_odom_frame_, true);

  float d = 5.0;
  if (!robot_traj_.empty())
  {
    geometry_msgs::PoseStamped t = robot_traj_.back();
    float xt = t.pose.position.x;
    float yt = t.pose.position.y;
    float zt = t.pose.position.z;
    float xr = pose_out.pose.position.x;
    float yr = pose_out.pose.position.y;
    float zr = pose_out.pose.position.z;
    d = sqrt((xt - xr) * (xt - xr) + (yt - yr) * (yt - yr) + (zt - zr) * (zt - zr));
  }


  pose_mutex_.lock();
  robot_pose_ = pose_out;
  if (d > 0.03)
  {
    robot_traj_.push_back(pose_out);
    // printf("poseCallback. inserting x:%.2f, y:%.2f, z:%.2f, size: %u\n",
    // pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z,
    // (unsigned int)robot_traj_.size());
  }
  pose_mutex_.unlock();
}



bool nav3d::Features3D::poseValid(geometry_msgs::PoseStamped* s)
{
  std::vector<float> f = getFeatures(s);
  // for(unsigned int i=0; i<f.size(); i++)
  //	printf("feature[%u] = %.2f", i, f[i]);
  // printf("\n");
  if (f[0] == 1.0)
    return true;
  else
    return false;
}



std::vector<std::vector<int> > nav3d::Features3D::clusterize_leaves(
    std::vector<geometry_msgs::Point>* points, float radius)
{
  // Create the pcl pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ps(new pcl::PointCloud<pcl::PointXYZ>);
  ps->width = points->size();
  ps->height = 1;
  ps->is_dense = true;
  for (unsigned int i = 0; i < points->size(); i++)
  {
    pcl::PointXYZ p(points->at(i).x, points->at(i).y, points->at(i).z);
    ps->points.push_back(p);
  }
  // printf("Clusterize_leaves. pointcloud in size: %u\n", (unsigned int)ps->width);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
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


  // GET THE POINT CLOUD FOR EVALUATION
  pcl_filters::GetFilteredPC srv;
  if (!exp_client_.call(srv))
  {
    ROS_ERROR("%s. evaluate_leaves. Error getting filtered point cloud", name_.c_str());
    return clusters;
  }

  sensor_msgs::PointCloud2 local;
  if (!pcl_ros::transformPointCloud(robot_odom_frame_, srv.response.pc, local, *tf_))
  {
    ROS_WARN("%s. evaluate_leaves. TransformPointCloud failed!!!!!", name_.c_str());
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(local, *cloud);
    // exp_cloud_ = cloud;
    // exp_cloud_ = *cloud;
    // kdtree_exp_->setInputCloud(cloud);
    // kdtree_exp_->setInputCloud(exp_cloud_);
    // printf("\nBefore setInputCloud\n");
    vgc_->setInputCloud(cloud);
    // printf("\nBefore vgc_filter\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZ>);
    vgc_->filter(*cloud_fil, true);
    // printf("\nAfter filter\n");
    //*exp_cloud_ = *cloud_fil;
    // printf("\nAfter exp_cloud = cloud_fil\n");
  }


  if (cluster_indices.empty())
  {
    printf("%s. Clusterize_leaves. No clusters found!!!\n", name_.c_str());
    return clusters;
  }


  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it)
  {
    std::vector<int> ind;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new
    // pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
    {
      ind.push_back((int)*pit);
    }

    clusters.push_back(ind);
    j++;
  }

  // printf("nav_features3d. returning %u clusters \n", (unsigned int)clusters.size());
  return clusters;
}



float nav3d::Features3D::evaluate_leaf(geometry_msgs::Point* p, float rrt_cost,
                                       std::string frame, float radius, float& npoints)
{
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.header.stamp = ros::Time::now();
  ps.pose.position = *p;
  ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  geometry_msgs::PoseStamped st = transformPoseTo(ps, robot_odom_frame_, false);

  pcl::PointXYZ sp;
  sp.x = st.pose.position.x;
  sp.y = st.pose.position.y;
  sp.z = st.pose.position.z;


  std::vector<float> pointRadiusSquaredDistance;
  std::vector<const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf*> k_leaves;



  //--------------------------------------------------------------------------------
  // 1.- REMOVE FRONTIERS CLOSE TO WALLS
  //--------------------------------------------------------------------------------
  if (remove_wall_frontier_enabled_)
  {
    int n = vgc_->radiusSearch(sp, (robot_radius_ + 0.30), k_leaves, pointRadiusSquaredDistance);

    if (n > 0)
    {
      // THROUGH NORMALS of VOXEL GRID COVARIANCE
      int cont = 0;
      for (const auto& it : k_leaves)  // const
      {
        const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf* l = it;  //&(it->second);

        Eigen::Vector3d mean = l->getMean();
        Eigen::Matrix3d evecs = l->getEvecs();
        Eigen::Vector3d evals = l->getEvals();

        Eigen::Quaternion<double> q(evecs);

        auto euler =
            q.toRotationMatrix().eulerAngles(0, 1, 2);  // 0->yaw, 1->pitch, 2->roll
        double yaw = euler[0];
        double pitch = euler[1];
        double roll = euler[2];


        if (!((fabs(pitch) < pitch_high2_ && fabs(pitch) > pitch_low2_) ||
              (fabs(yaw) < roll_high2_ && fabs(yaw) > roll_low2_)))
        {
          cont++;
        }
      }
      if (cont >= 30)
      {
        wall_points_.push_back(sp);
        return 1.0;
      }
      else
      {
        // printf("Total points: %i, wall points: %i, NO WALL!!!!\n", n, cont);
      }
    }
    else
    {
      return 1.0;
    }
  }


  //--------------------------------------------------------------------------------
  // 2.- REMOVE FRONTIERS VISITED TWICE
  //--------------------------------------------------------------------------------
  k_leaves.clear();
  pointRadiusSquaredDistance.clear();
  float num_points = vgc_->radiusSearch(sp, radius, k_leaves, pointRadiusSquaredDistance);
  npoints = num_points;


  // float goal_cost = 0.1;
  if (visited_region_enabled_)
  {
    // Evaluate if the region was visited
    if (!exp_regions_.empty())
    {
      for (unsigned int i = 0; i < exp_regions_.size(); i++)
      {
        geometry_msgs::Point p = exp_regions_[i].p;
        // printf("Goal %u: x:%.1f, y:%.1f, z:%.1f, npoints:%.1f, visits:%i\n", i,
        // p.x,p.y,p.z,exp_regions_[i].num_points, exp_regions_[i].visits);
        float r = (sp.x - p.x) * (sp.x - p.x) + (sp.y - p.y) * (sp.y - p.y) +
                  (sp.z - p.z) * (sp.z - p.z);
        float d = sqrt(r);
        if (d < exp_min_dist_goals_)
        {
          if (exp_regions_[i].visits >= 2)
          {
            printf("REGION %u visited %i times. Returning cost 1!!\n", i, exp_regions_[i].visits);
            // goal_cost = 1.0;
            return 1.0;
          }
          // float similarity;
          // if(exp_regions_[i].num_points > num_points)
          //	similarity = num_points/exp_regions_[i].num_points;
          // else
          //	similarity = exp_regions_[i].num_points/num_points;

          // if the similary in the number of points is above the 85%, we
          // consider that we gain little info, so we can give it a new chance
          /*if(similarity > 0.85) {
            goal_cost = 0.3;
            break;
          } else { //If the number of points has increased after the first visit,
               //we consider that is more difficult to get more information
            goal_cost = 0.6;
            break;
          }*/
          // goal_cost = 0.5;
        }
      }
    }
  }



  //--------------------------------------------------------------------------------
  // 3.- EVALUATE POSSIBLE VOIDS ACCORDING TO THE NUM OF POINTS AND STD DEV
  //--------------------------------------------------------------------------------
  if (num_points > 0.0)
  {
    // Normalize num_points
    // float max_points = 3400.0;
    if (num_points > num_points_saturation_)
      num_points = num_points_saturation_;
    num_points = (num_points / num_points_saturation_);
  }
  else
  {
    num_points = 1.0;
    return num_points;
  }

  // Compute Standard deviation of the point set
  Eigen::Vector3d mean;
  mean[0] = mean[1] = mean[2] = 0.0;
  for (const auto& it : k_leaves)  // const
  {
    const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf* l = it;  //&(it->second);
    Eigen::Vector3d m = l->getMean();
    mean[0] = mean[0] + m[0];
    mean[1] = mean[1] + m[1];
    mean[2] = mean[2] + m[2];
  }
  mean[0] = mean[0] / k_leaves.size();
  mean[1] = mean[1] / k_leaves.size();
  mean[2] = mean[2] / k_leaves.size();

  float sum = 0;
  float max_d = 0;
  ;
  for (const auto& it : k_leaves)  // const
  {
    const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf* l = it;  //&(it->second);
    Eigen::Vector3d m = l->getMean();
    float s = (m[0] - mean[0]) + (m[1] - mean[1]) + (m[2] - mean[2]);
    float sq = s * s;
    sum += sq;

    // For normalization
    float n = radius;  // + radius; // + radius;
    float sqn = n * n;
    max_d += sqn;
  }
  // float me = sum / pointIdxRadiusSearch.size();
  float variance = sum / (float)(k_leaves.size() - 1);
  float stddev = sqrt(variance);

  // for normalization
  float varn = max_d / (float)(k_leaves.size() - 1);
  float max_stddev = sqrt(varn);

  float stddev_norm = stddev / max_stddev;
  if (stddev_norm > 1.0)
    stddev_norm = 1.0;
  // printf("Evaluate leaf: points: %.2f, stddev: %.4f, max_std: %.4f, std_norm: %.4f\n",
  // num_points, stddev, max_stddev, stddev_norm);

  float num_point_cost = 0.7 * num_points + 0.3 * stddev_norm;


  if (num_point_cost > threshold_frontier_)
    return 1.0;


  num_points_.push_back(num_point_cost);



  //--------------------------------------------------------------------------------
  // NFE COST
  //--------------------------------------------------------------------------------
  if (nfe_)
  {
    pose_mutex_.lock();
    geometry_msgs::PoseStamped rpose = robot_pose_;
    pose_mutex_.unlock();
    float rx = rpose.pose.position.x;
    float ry = rpose.pose.position.y;
    float rz = rpose.pose.position.z;
    float max_dist = current_size_ * current_size_ * current_size_;

    float d = sqrt((p->x - rx) * (p->x - rx) + (p->y - ry) * (p->y - ry) +
                   (p->z - rz) * (p->z - rz));
    // float d = (p->x*p->x + p->y*p->y + p->z*p->z);
    if (d > max_dist)
      d = max_dist;
    float nfe_cost = d / max_dist;
    return nfe_cost;
  }
  else if (bfe_)
  {
    //----------------------------------------------------------------------------
    // BFE COST
    //----------------------------------------------------------------------------
    return num_point_cost;
  }



  //--------------------------------------------------------------------------------
  // 4.- NO-RETURN COST
  //--------------------------------------------------------------------------------
  float dcost = no_return_cost(&st);


  //--------------------------------------------------------------------------------
  // 5.- TOTAL COST
  //--------------------------------------------------------------------------------
  float exp_cost;
  if (rrt_cost == -1.0)
    exp_cost = (wexp_[0] + wexp_[2] / 2.0) * num_point_cost + (wexp_[1] + wexp_[2] / 2.0) * dcost;
  else
    exp_cost = wexp_[0] * num_point_cost + wexp_[1] * dcost +
               wexp_[2] * rrt_cost;  // + wexp_[3]*goal_cost;//0.35, 0.3, 0.175, 0.175



  return exp_cost;
}


int nav3d::Features3D::evaluate_leaves(std::vector<geometry_msgs::Point>* points,
                                       std::vector<float>* pcosts, std::string frame, float radius)
{
  float min_cost = 1000.0;
  int ind_goal;
  float exploration_cost;
  float num_points;
  for (unsigned int i = 0; i < points->size(); i++)
  {
    float npoints;
    // printf("\nBefore evaluate_leaf\n");
    exploration_cost = evaluate_leaf(&(points->at(i)), pcosts->at(i), frame, radius, npoints);
    // printf("\nAfter evaluate leaf\n");
    if (exploration_cost < 1.0)
      frontier_points_.push_back(points->at(i));
    if (exploration_cost < min_cost)
    {
      min_cost = exploration_cost;
      ind_goal = i;
      num_points = npoints;
    }
  }


  if (visualize_wall_leaves_)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = robot_odom_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "wall_points";
    marker.id = 3000;  // key;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    for (unsigned int i = 0; i < wall_points_.size(); i++)
    {
      geometry_msgs::Point p;
      p.x = wall_points_[i].x;
      p.y = wall_points_[i].y;
      p.z = wall_points_[i].z;

      marker.points.push_back(p);
    }

    wall_pub_.publish(marker);
  }
  wall_points_.clear();

  // sensor_msgs::PointCloud2 msg;
  // pcl::toROSMsg(wall_cloud_, msg);
  // pc_wall_pub_.publish(msg);


  if (visualize_frontiers_)
  {
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = robot_odom_frame_;
    marker2.header.stamp = ros::Time();
    marker2.ns = "frontier_points";
    marker2.id = 4000;  // key;
    marker2.type = visualization_msgs::Marker::POINTS;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.scale.x = 0.15;
    marker2.scale.y = 0.15;
    marker2.scale.z = 0.15;
    marker2.color.a = 1.0;
    marker2.color.r = 1.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    // marker.lifetime = ros::Duration(2.0);
    for (unsigned int i = 0; i < frontier_points_.size(); i++)
    {
      frontier_points_[i].z = frontier_points_[i].z + 0.02;
      marker2.points.push_back(frontier_points_[i]);
    }

    frontier_pub_.publish(marker2);
  }
  frontier_points_.clear();



  // Transform goal to the correct frame
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.header.stamp = ros::Time::now();
  ps.pose.position = points->at(ind_goal);
  ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  geometry_msgs::PoseStamped st = transformPoseTo(ps, robot_odom_frame_, false);

  geometry_msgs::Point p;
  p.x = st.pose.position.x;
  p.y = st.pose.position.y;
  p.z = st.pose.position.z;

  //-----------------------------------------------------------------------------
  //  EVALUATE IF THE REGION WAS ALREADY VISITED
  //-----------------------------------------------------------------------------
  if (visited_region_enabled_)
  {
    visualization_msgs::Marker marker3;
    marker3.header.frame_id = robot_odom_frame_;
    marker3.header.stamp = ros::Time();
    marker3.ns = "frontier_points";
    marker3.id = 5000;  // key;
    marker3.type = visualization_msgs::Marker::SPHERE_LIST;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.scale.x = exp_min_dist_goals_;
    marker3.scale.y = exp_min_dist_goals_;
    marker3.scale.z = exp_min_dist_goals_;
    marker3.color.a = 0.6;
    marker3.color.r = 0.72;
    marker3.color.g = 0.2;
    marker3.color.b = 1.0;
    // printf("Evaluating point x:%.1f, y:%.1f, z:%.1f with goals:\n", p.x, p.y, p.z);
    for (unsigned int i = 0; i < exp_regions_.size(); i++)
    {
      geometry_msgs::Point t = exp_regions_[i].p;

      if (exp_regions_[i].visits >= 2 && visualize_visited_reg_)
      {
        marker3.points.push_back(t);
      }

      float r =
          (p.x - t.x) * (p.x - t.x) + (p.y - t.y) * (p.y - t.y) + (p.z - t.z) * (p.z - t.z);
      float d = sqrt(r);
      // printf("Goal %u: x:%.1f, y:%.1f, z:%.1f, d: %.2f, visits:%i\n", i, t.x,t.y,t.z,
      // d, exp_regions_[i].visits);
      if (d <= exp_min_dist_goals_)
      {
        // printf("Evaluate_leaves. Region previously visited. Adding the visit!\n\n");
        exp_regions_[i].visits = exp_regions_[i].visits + 1;
      }
    }
    if (visualize_visited_reg_)
      visited_pub_.publish(marker3);
  }



  //-----------------------------------------------------------------------------
  //  EVALUATE THE VOID COSTS IN ORDER TO INCREASE/DECREASE THE EXPLORATION SIZE
  //-----------------------------------------------------------------------------
  if (variable_size_enabled_)
  {
    ros::NodeHandle nh;
    ros::ServiceClient rrtfloor = nh.serviceClient<pcl_filters::ChangeCropboxSize>(
        "pcl_filters_node_1/change_cropbox_size");
    ros::ServiceClient features = nh.serviceClient<pcl_filters::ChangeCropboxSize>(
        "pcl_filters_node_2/change_cropbox_size");
    ros::ServiceClient explore = nh.serviceClient<pcl_filters::ChangeCropboxSize>(
        "pcl_filters_node_4/change_cropbox_size");
    // ros::ServiceClient rrttime =
    // nh.serviceClient<rrt_planners::ChangeSolveTime>("RRT_ros_wrapper/changeRRTSolveTime");
    if (!num_points_.empty())
    {
      int count = 0;
      for (unsigned int i = 0; i < num_points_.size(); i++)
      {
        // printf("n_point: %.2f\t", num_points_[i]);
        if (num_points_[i] >= numpoint_cost_limit_)
        {
          count++;
        }
      }
      printf("\n");
      printf("count: %i, size: %i, per: %.2f.\n", count, (int)num_points_.size(),
             ((float)count / (float)num_points_.size()));

      if (((float)count / (float)num_points_.size()) >= percentage_limit_ && current_size_ < max_size_)
      {
        // printf("Increase planning size!\n\n");

        pcl_filters::ChangeCropboxSize srv;
        current_size_ = current_size_ + 2.0;
        if (current_size_ > max_size_)
        {
          current_size_ = max_size_;
        }
        printf("THRES: %.2f, INCREASING PLANNING SIZE TO %.2f!!!\n\n",
               threshold_frontier_, current_size_);
        size_x_ = size_y_ = size_z_ = current_size_ * 2.0;
        max_planning_dist_ =
            sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));
        srv.request.size = current_size_;
        if (rrtfloor.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        if (features.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        float newsize = current_size_ + 2.0;
        srv.request.size = newsize;
        if (explore.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        // rrt_planners::ChangeSolveTime t_srv;
        // t_srv.request.solve_time = rrt_time_;
        ros::NodeHandle nd("~");
        float st = 4.0;
        nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
        printf("RRT solve time readed: %.2f\n", st);
        std_msgs::Float32 msg;
        msg.data = st + 0.8;
        rrt_time_pub_.publish(msg);
        nd.setParam("RRT_ros_wrapper/rrt_solve_time", msg.data);
        nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
        printf("RRT solve time updated: %.2f\n", st);

        /*if(first_rrt_time_) {
          t_srv.request.solve_time = rrt_time_;
          printf("first time. solve_time: %.2f\n", t_srv.request.solve_time);
          if(rrttime.call(t_srv))
          {
            rrt_time_ = t_srv.response.previous_time;
          }
          printf("After calling service\n");
          t_srv.request.solve_time = rrt_time_ + 0.8;
          rrttime.call(t_srv);
          rrt_time_ = t_srv.response.new_time;
          first_rrt_time_ = false;
        } else {
          t_srv.request.solve_time = rrt_time_ + 0.8;
          rrttime.call(t_srv);
          rrt_time_ = t_srv.response.new_time;
        }*/
      }
      else if (current_size_ > min_size_)
      {
        // printf("Decrease planning size!\n\n");

        pcl_filters::ChangeCropboxSize srv;
        current_size_ = current_size_ - 2.0;
        if (current_size_ < min_size_)
        {
          current_size_ = min_size_;
        }
        printf("THRES: %.2f, DECREASING PLANNING SIZE TO %.2f!!!\n\n",
               threshold_frontier_, current_size_);
        size_x_ = size_y_ = size_z_ = current_size_ * 2.0;
        max_planning_dist_ =
            sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));
        srv.request.size = current_size_;
        if (!rrtfloor.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        if (!features.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        float newsize = current_size_ + 2.0;
        srv.request.size = newsize;
        if (!explore.call(srv))
        {
          bool outbounds = srv.response.outofbounds;
        }
        // rrt_planners::ChangeSolveTime t_srv;
        // t_srv.request.solve_time = rrt_time_ - 0.8;
        // rrttime.call(t_srv);
        // rrt_time_ = t_srv.response.new_time;
        ros::NodeHandle nd("~");
        float st = 4.0;
        nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
        printf("RRT solve time readed: %.2f\n", st);
        std_msgs::Float32 msg;
        msg.data = st - 0.8;
        rrt_time_pub_.publish(msg);
        nd.setParam("RRT_ros_wrapper/rrt_solve_time", msg.data);
        nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
        printf("RRT solve time updated: %.2f\n", st);
      }

      num_points_.clear();
    }
    else if (current_size_ < max_size_)
    {
      // if the num_points is empty, we have not detected frontiers -> increase size

      pcl_filters::ChangeCropboxSize srv;
      current_size_ = current_size_ + 2.0;
      if (current_size_ > max_size_)
      {
        current_size_ = max_size_;
      }
      printf("NO FRONTIERS FOUND. THRES: %.2f, INCREASING PLANNING SIZE TO %.2f!!!\n\n",
             threshold_frontier_, current_size_);
      size_x_ = size_y_ = size_z_ = current_size_ * 2.0;
      max_planning_dist_ =
          sqrt((size_x_ * size_x_) + (size_y_ * size_y_) + (size_z_ * size_z_));
      srv.request.size = current_size_;
      if (rrtfloor.call(srv))
      {
        bool outbounds = srv.response.outofbounds;
      }
      if (features.call(srv))
      {
        bool outbounds = srv.response.outofbounds;
      }
      float newsize = current_size_ + 2.0;
      srv.request.size = newsize;
      if (explore.call(srv))
      {
        bool outbounds = srv.response.outofbounds;
      }
      // rrt_planners::ChangeSolveTime t_srv;
      // t_srv.request.solve_time = rrt_time_;
      ros::NodeHandle nd("~");
      float st = 4.0;
      nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
      printf("RRT solve time readed: %.2f\n", st);
      std_msgs::Float32 msg;
      msg.data = st + 0.8;
      rrt_time_pub_.publish(msg);
      nd.setParam("RRT_ros_wrapper/rrt_solve_time", msg.data);
      nd.getParam("RRT_ros_wrapper/rrt_solve_time", st);
      printf("RRT solve time updated: %.2f\n", st);
    }
    else if (adaptative_threshold_)
    {
      // last option, we increase the tolerable cost of the void detection
      ros::NodeHandle nd("~");
      threshold_frontier_ = threshold_frontier_ + 0.1;
      numpoint_cost_limit_ = threshold_frontier_ - 0.05;
      if (threshold_frontier_ > 0.9)
      {
        threshold_frontier_ = 0.9;
        numpoint_cost_limit_ = threshold_frontier_ + 0.05;
      }
      printf("NO FRONTIERS FOUND. INCREASING VOID DETECTION THRESHOLD TO %.2f!!!\n\n",
             threshold_frontier_);
      nd.setParam("threshold_frontier", threshold_frontier_);
      float f;
      nd.getParam("threshold_frontier", f);
      printf("threshold frontier updated: %.2f\n", f);
    }
  }


  // Fill and store the struct exp_goal
  exp_goal g;
  g.p = p;
  g.num_points = num_points;
  g.visits = 1;
  exp_regions_.push_back(g);


  return ind_goal;
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
  for (unsigned int i = 0; i < traj.size(); i++)
  {
    float x = traj[i].pose.position.x;
    float y = traj[i].pose.position.y;
    float z = traj[i].pose.position.z;
    float dist = sqrt((xp - x) * (xp - x) + (yp - y) * (yp - y) + (zp - z) * (zp - z));
    if (dist < dmin)
      dmin = dist;
  }
  if (dmin > max_dist)
    dmin = max_dist;

  // The cost decreases linearly from 1 to 0 in a distance of 2 meters
  cost = ((max_dist - dmin) / max_dist);

  return cost;
}



bool nav3d::Features3D::pose3dValid(geometry_msgs::PoseStamped* s)
{
  geometry_msgs::PoseStamped st = transformPoseTo(*s, robot_odom_frame_, false);  // false

  pcl::PointXYZ sp;
  sp.x = st.pose.position.x;
  sp.y = st.pose.position.y;
  // We have to calculate the approximate height z based on the neighbors
  sp.z = st.pose.position.z;
  // printf("3dnav_features. pose3dValid point x:%.2f, y:%.2f, z:%.2f\n", sp.x, sp.y,
  // sp.z);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  tree_mutex_.lock();

  // std::vector<int> pointIdxKSearch(10);
  // std::vector<float> pointKSquaredDistance(10);
  // int found_k = kdtree_->nearestKSearch (sp, 10, pointIdxKSearch,
  // pointKSquaredDistance);
  // printf("3denav_features. pose3dValid. kdtree ksearch found %i neighbors\n", found_k);
  // for(unsigned int j=0; j<found_k; j++)
  //	printf("point k %u, dist: %.2f\n", j, pointKSquaredDistance[j]);


  int found = kdtree_->radiusSearch(sp, (robot_radius_ * 1.0), pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance);

  // int found2 = octree_->radiusSearch(sp, (robot_radius_*1.0), pointIdxRadiusSearch,
  // pointRadiusSquaredDistance);

  float new_height = 0.0;
  if (found > 0)
  {
    // printf("3dnav_features. pose3dValid. kdtree radiussearch found %i neighbors in the
    // radius %.2f\n", found, (robot_radius_*1.5));
    tree_mutex_.unlock();

    if (pointIdxRadiusSearch.size() < min_points_allowed_)
    {
      // printf("%s. pose3dValid. %u points found. Minimum: %i\n", name_.c_str(),
      // (unsigned int)pointIdxRadiusSearch.size(), min_points_allowed_);
      return false;
    }

    // Generate pointcloud data
    // patch->width = (int)pointIdxRadiusSearch.size();
    // patch->height = 1;
    // patch->points.resize(patch->width * patch->height);

    int cont = 0;
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      // printf("3dnav_features. pose3dValid. radiusSquare[%u] = %.2f\t", (unsigned int)i,
      // pointRadiusSquaredDistance[i]);
      if ((sqrt(pointRadiusSquaredDistance[i])) <= 0.25)
      {
        cont++;
        new_height += pcl_cloud_.points[pointIdxRadiusSearch[i]].z;
      }
      // patch->points[i] = pcl_cloud_.points[ pointIdxRadiusSearch[i] ];
    }
    if (cont > 0)
    {
      new_height = new_height / (float)cont;
      // s->pose.position.z = new_height;
    }
  }
  else
  {
    tree_mutex_.unlock();
    // printf("%s. pose3dValid. kdtree found %i neighbors in the radius %.2f\n",
    // name_.c_str(), found, robot_radius_);
    return false;
  }



  // pca_->setInputCloud(patch);
  // pca_->setIndices(const IndicesPtr &indices)
  // pca_->setIndices (const IndicesConstPtr &indices)
  // typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr

  // pcl::IndicesPtr indices(new std::vector<int>);
  pcl::IndicesPtr indices(new std::vector<int>(pointIdxRadiusSearch));
  // printf("3dnav_features. pose3dValid. pind[0]:%i, pind[1]:%i, pind[2]:%i\n",
  // pointIdxRadiusSearch[0], pointIdxRadiusSearch[1], pointIdxRadiusSearch[2]);
  // printf("3dnav_features. pose3dValid. ptr[0]:%i, ptr[1]:%i, ptr[2]:%i\n",
  // indices->at(0), indices->at(1), indices->at(2));
  tree_mutex_.lock();
  // pca_->setIndices(indices);


  Eigen::Vector4f mean;
  Eigen::Vector3f evals;
  Eigen::Matrix3f evecs;

  try
  {
    // mean = pca_->getMean();
    // evals = pca_->getEigenValues();
    // evecs = pca_->getEigenVectors();


    mean = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(pcl_cloud_, *indices,
                           mean);  // input ->const pcl::PointCloud< PointT > &cloud
    // Compute demeanished cloud
    Eigen::MatrixXf cloud_demean;
    pcl::demeanPointCloud(pcl_cloud_, *indices, mean, cloud_demean);
    assert(cloud_demean.cols() == int(indices->size()));
    // Compute the product cloud_demean * cloud_demean^T
    Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f>(
        cloud_demean.topRows<3>() * cloud_demean.topRows<3>().transpose());

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd(alpha);
    // Organize eigenvectors and eigenvalues in ascendent order
    for (int i = 0; i < 3; ++i)
    {
      evals[i] = evd.eigenvalues()[2 - i];
      evecs.col(i) = evd.eigenvectors().col(2 - i);
    }



    tree_mutex_.unlock();
    // printf("3dnav_features. pose3dValid. mean x:%.2f, y:%.2f, z:%.2f\n", mean[0],
    // mean[1], mean[2]);
  }
  catch (pcl::InitFailedException e)
  {
    ROS_WARN("%s. pose3dValid. Calculation of mean, eigenvectors and eigenvalues failed!",
             name_.c_str());
    tree_mutex_.unlock();
    return false;
  }



  Eigen::Quaternion<float> q(evecs);

  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);  // 0->yaw, 1->pitch, 2->roll
  double yaw = euler[0];
  double pitch = euler[1];
  double roll = euler[2];


  // Identify the lowest eigenvalue
  // TODO: the eigen values are ordered.
  // The smallest is the last one. CHECK THIS!

  // Normal vector
  float nx = (float)evecs(0, 2);
  float ny = (float)evecs(1, 2);
  float nz = (float)evecs(2, 2);


  pose_mutex_.lock();
  geometry_msgs::PoseStamped rpose = robot_pose_;
  pose_mutex_.unlock();



  // Check the direction of the normal
  // correct if n . (pv - Pi) > 0
  // n  -> evec with the lowest eigenvalue (= normal vector)
  // pv -> (0,0,0) Point of view
  // pi -> mean
  // See if we need to flip any plane normals
  /*
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = ((rpose.pose.position.x-mean[0]) * nx +
  (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    pcl::PointXYZ p;
    p.x = mean[0];
    p.y = mean[1];
    p.z = mean[2];
    pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y,
  rpose.pose.position.z, nx, ny, nz);

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

  // if(pitch > pitch_high_ || pitch < pitch_low_ || yaw > roll_high_ || yaw < roll_low_
  // || fabs(evals(2)) > roughness_) {
  // if(fabs(pitch) < (M_PI/4.0) || fabs(pitch) > (3.0*M_PI/4.0) || fabs(yaw) < (M_PI/4.0)
  // || fabs(yaw) > (3.0*M_PI/4.0) || fabs(evals(2)) > roughness_) {
  if ((fabs(pitch) > pitch_low_ && fabs(pitch) < pitch_high_) ||
      /*(fabs(yaw) > roll_low_ && fabs(yaw) < roll_high_) ||*/ fabs(evals(2)) > roughness_)
  {
    // if( fabs(evals(2)) > roughness_) {
    // if(name_ == std::string("local_features_3d"))
    // printf("%s. pose3d INVALID. p:%.2f, y:%.2f, r:%.2f, upper:%.2f, lower:%.2f\n",
    // name_.c_str(), fabs(pitch), fabs(yaw), fabs(roll), pitch_high_, pitch_low_);
    return false;
  }
  else
  {
    // if(name_ == std::string("local_features_3d"))
    // printf("%s. pose3d VALID. p:%.2f, y:%.2f, r:%.2f, upper:%.2f, lower:%.2f\n",
    // name_.c_str(), fabs(pitch), fabs(yaw), fabs(roll), pitch_high_, pitch_low_);


    // mean[2] = height z in odom coordinates. I need to transform it to base frame and
    // update this value in pose s.
    // geometry_msgs::PoseStamped samplepose;
    // meanpose.header.frame_id = robot_odom_frame_; //rpose.header.frame_id;
    // meanpose.header.stamp = ros::Time::now();
    // meanpose.pose.position.x = st.pose.position.x; //mean[0];
    // meanpose.pose.position.y = st.pose.position.y; //mean[1];
    st.header.stamp = ros::Time::now();
    if (new_height == 0.0)
      st.pose.position.z = mean[2];
    else
      st.pose.position.z = new_height;  // mean[2];
    st.pose.orientation = rpose.pose.orientation;
    geometry_msgs::PoseStamped sbase = transformPoseTo(st, s->header.frame_id, false);  // robot_base_frame_
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

  if (feature_set_ == 1)
    features.assign(4, 0.0);  // features[4] = {valid, pitch, roll, roughness}
  else
    features.assign(8, 0.0);  // features[8] = {valid, pitch, roll, roughness, point_dist,
                              // stddev, num_points, goal_dist} //curvature

  geometry_msgs::PoseStamped st = transformPoseTo(*s, robot_odom_frame_, false);

  pcl::PointXYZ sp;
  sp.x = st.pose.position.x;
  sp.y = st.pose.position.y;
  // We have to calculate the approximate height z based on the neighbors
  sp.z = st.pose.position.z;


  // pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  tree_mutex_.lock();

  // std::vector<int> pointIdxKSearch(10);
  // std::vector<float> pointKSquaredDistance(10);
  // int found_k = kdtree_->nearestKSearch (sp, 10, pointIdxKSearch,
  // pointKSquaredDistance);
  // printf("3denav_features. pose3dValid. kdtree ksearch found %i neighbors\n", found_k);
  // for(unsigned int j=0; j<found_k; j++)
  //	printf("point k %u, dist: %.2f\n", j, pointKSquaredDistance[j]);


  int found = kdtree_->radiusSearch(sp, (robot_radius_ * 1.0), pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance);
  float num_points = (float)found;
  if (found > 0)
  {
    // printf("3dnav_features. pose3dValid. kdtree radiussearch found %i neighbors in the
    // radius %.2f\n", found, (robot_radius_*1.5));
    tree_mutex_.unlock();

    if (pointIdxRadiusSearch.size() < min_points_allowed_)
    {
      // printf("3dnav_features. pose3dValid. indices less than 3\n");
      return features;
    }

    // Generate pointcloud data
    // patch->width = (int)pointIdxRadiusSearch.size();
    // patch->height = 1;
    // patch->points.resize(patch->width * patch->height);

    float new_height = 0.0;
    int cont = 0;
    float sum = 0;
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      // printf("3dnav_features. pose3dValid. radiusSquare[%u] = %.2f\t", (unsigned int)i,
      // pointRadiusSquaredDistance[i]);
      if ((sqrt(pointRadiusSquaredDistance[i])) <= 0.25)
      {
        cont++;
        new_height += pcl_cloud_.points[pointIdxRadiusSearch[i]].z;
      }
      // patch->points[i] = pcl_cloud_.points[ pointIdxRadiusSearch[i] ];
    }
    if (cont > 0)
    {
      new_height = new_height / (float)cont;
      s->pose.position.z = new_height;
    }
  }
  else
  {
    tree_mutex_.unlock();
    printf("%s. getFeatures. kdtree radiussearch found %i neighbors in the radius %.2f\n",
           name_.c_str(), found, robot_radius_);
    return features;
  }


  // pca_->setInputCloud(patch);
  // pca_->setIndices(const IndicesPtr &indices)
  // pca_->setIndices (const IndicesConstPtr &indices)
  // typedef boost::shared_ptr<std::vector<int> > pcl::IndicesPtr

  // pcl::IndicesPtr indices(new std::vector<int>);
  pcl::IndicesPtr indices(new std::vector<int>(pointIdxRadiusSearch));
  tree_mutex_.lock();
  // pca_->setIndices(indices);

  Eigen::Vector4f mean;
  Eigen::Vector3f evals;
  Eigen::Matrix3f evecs;

  try
  {
    // mean = pca_->getMean();
    // evals = pca_->getEigenValues();
    // evecs = pca_->getEigenVectors();
    mean = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(pcl_cloud_, *indices,
                           mean);  // input ->const pcl::PointCloud< PointT > &cloud
    // Compute demeanished cloud
    Eigen::MatrixXf cloud_demean;
    pcl::demeanPointCloud(pcl_cloud_, *indices, mean, cloud_demean);
    assert(cloud_demean.cols() == int(indices->size()));
    // Compute the product cloud_demean * cloud_demean^T
    Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f>(
        cloud_demean.topRows<3>() * cloud_demean.topRows<3>().transpose());

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd(alpha);
    // Organize eigenvectors and eigenvalues in ascendent order
    for (int i = 0; i < 3; ++i)
    {
      evals[i] = evd.eigenvalues()[2 - i];
      evecs.col(i) = evd.eigenvectors().col(2 - i);
    }
    tree_mutex_.unlock();
  }
  catch (pcl::InitFailedException e)
  {
    ROS_WARN("%s. getFeatures. PCA failed!", name_.c_str());
    tree_mutex_.unlock();
    return features;
  }


  float sum = 0;
  float max_d = 0;
  ;
  for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
  {
    float s = (pcl_cloud_.points[pointIdxRadiusSearch[i]].x - mean[0]) +
              (pcl_cloud_.points[pointIdxRadiusSearch[i]].y - mean[1]);
    float sq = s * s;
    sum += sq;

    // For normalization
    float n = robot_radius_ + robot_radius_;
    float sqn = n * n;
    max_d += sqn;
  }
  // float me = sum / pointIdxRadiusSearch.size();
  float variance = sum / (pointIdxRadiusSearch.size() - 1);
  float stddev = sqrt(variance);

  // for normalization
  float varn = max_d / (pointIdxRadiusSearch.size() - 1);
  float max_stddev = sqrt(varn);


  // we can check how far is the centroid from the point sp
  float dx = sp.x - mean[0];
  float dy = sp.y - mean[1];
  float dz = sp.z - mean[2];
  float point_dist = sqrt(dx * dx + dy * dy + dz * dz);


  Eigen::Quaternion<float> q(evecs);

  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);  // 0->yaw, 1->pitch, 2->roll
  double yaw = euler[0];
  double pitch = euler[1];
  double roll = euler[2];


  // Identify the lowest eigenvalue
  // TODO: the eigen values are ordered.
  // The smallest is the last one. CHECK THIS!

  // Normal vector
  float nx = (float)evecs(0, 2);
  float ny = (float)evecs(1, 2);
  float nz = (float)evecs(2, 2);


  pose_mutex_.lock();
  geometry_msgs::PoseStamped rpose = robot_pose_;
  pose_mutex_.unlock();



  // Check the direction of the normal
  // correct if n . (pv - Pi) > 0
  // n  -> evec with the lowest eigenvalue (= normal vector)
  // pv -> (0,0,0) Point of view
  // pi -> mean
  // See if we need to flip any plane normals

  // Dot product between the (viewpoint - point) and the plane normal
  /*float cos_theta = ((rpose.pose.position.x-mean[0]) * nx +
  (rpose.pose.position.y-mean[1]) * ny + (rpose.pose.position.z-mean[2]) * nz);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    pcl::PointXYZ p;
    p.x = mean[0];
    p.y = mean[1];
    p.z = mean[2];
    pcl::flipNormalTowardsViewpoint(p, rpose.pose.position.x, rpose.pose.position.y,
  rpose.pose.position.z, nx, ny, nz);

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


  float curvature = fabsf(evals(0) / (evals(0) + evals(1) + evals(2)));



  // Normalize pitch and yaw
  // if(fabs(pitch) > M_PI/2.0)
  //	pitch = M_PI - fabs(pitch);
  // pitch = fabs(pitch)/M_PI;
  // if(fabs(yaw) > M_PI/2.0)
  //	yaw = M_PI - yaw;
  // yaw = yaw/M_PI;


  float zero = M_PI / 2.0;
  if (fabs(pitch) > zero)
    pitch = M_PI - fabs(pitch);
  pitch = fabs(pitch) / zero;
  if (fabs(yaw) > zero)
    yaw = M_PI - fabs(yaw);
  yaw = fabs(yaw) / zero;


  // Normalize roughness
  // Maximum value of roughness?????
  float max_rough = 1.5;  // 3.0;
  float rough = fabs(evals(2));
  if (rough > max_rough)
    rough = max_rough;
  rough = rough / max_rough;

  // normalize point dist. max dist = robot_radius_
  point_dist = point_dist / robot_radius_;

  // Normalize stddev
  stddev = 1.0 - (stddev / max_stddev);

  // Normalize num_points
  float area_circ = M_PI * (robot_radius_ * robot_radius_);
  float max_points = area_circ * 100.0;  // 1 point per centimeter = 100p/m  //30.0;
  if (num_points > max_points)
    num_points = max_points;
  num_points = 1.0 - (num_points / max_points);

  float dist_cost = goalDistFeature(s);


  // Things to evaluate the position:
  // - Inclination (pitch and roll)
  // - roughness (lowest eigenvalue)
  // - distance between the mean and the point evaluated
  // - curvature of the plain
  // - dispersion in the sfere, stddev in xy? High stddev is better
  // - TODO: add a point density feature in the area

  // features[8] = {valid, pitch, roll, roughness, point_dist, curvature, stddev,
  // num_points}
  features[0] = 1.0;
  features[1] = pitch;
  features[2] = yaw;  // the frame is turned, so the roll is the yaw now
  features[3] = rough;
  if (feature_set_ == 1)
    return features;

  features[4] = point_dist;
  // features[5] = curvature;    //not normalized
  features[5] = stddev;
  features[6] = num_points;  // point_density; ????
  features[7] = dist_cost;
  return features;
}



/*
 void nav3d::Features3D::reconfigureCB(navigation_features::nav_featuresConfig &config,
uint32_t level){

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



// Service
/*bool nav3d::Features3D::setWeightsService(navigation_features::SetWeights::Request
&req, navigation_features::SetWeights::Response &res)
{
  setWeights(req.weights);
  return true;
}
*/


// Service
/*bool nav3d::Features3D::isPoseValidService(navigation_features::PoseValid::Request &req,
navigation_features::PoseValid::Response &res)
{
  geometry_msgs::PoseStamped p = req.pose;
  res.ok = poseValid(&p);
}*/

// Service
/*bool nav3d::Features3D::setScenarioService(navigation_features::SetScenario::Request
&req, navigation_features::SetScenario::Response &res)
{
  setScenario(req.obstacles, req.people, req.goal);

  return true;
}*/

// Service
/*bool
nav3d::Features3D::getFeatureCountService(navigation_features::GetFeatureCount::Request
&req, navigation_features::GetFeatureCount::Response &res)
{
  res.fc = getPathFeatureCount(&req.path);
  return true;
}*/

// Service
/*bool
nav3d::Features3D::initializeWeightsService(navigation_features::InitWeights::Request
&req, navigation_features::InitWeights::Response &res)
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



/*void nav3d::Features3D::setScenario(sensor_msgs::PointCloud2 obs,
geometry_msgs::PoseStamped goal)
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



void nav3d::Features3D::setGoal(geometry_msgs::PoseStamped g)
{
  // boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
  // printf("NavFeatures. Receiving goal point!\n");
  goal_ = g;
}



float nav3d::Features3D::goalDistFeature(geometry_msgs::PoseStamped* s)
{
  // if we don't have a goal yet - Exploration
  if (goal_.header.frame_id.empty())
  {
    // printf("features3d. goalDistFeature zero. goal empty\n");
    return (float)0.0;
  }

  // if the frame_id are different, transform s to goal frame
  geometry_msgs::PoseStamped p = transformPoseTo(*s, goal_.header.frame_id, false);
  float dx = goal_.pose.position.x - p.pose.position.x;
  float dy = goal_.pose.position.y - p.pose.position.y;
  float dz = goal_.pose.position.z - p.pose.position.z;
  float dist = (float)(sqrt(dx * dx + dy * dy + dz * dz) / max_planning_dist_);
  if (dist > 1.0)
  {
    // printf("NavFeatures. goal dist feature %.2f > 1.0. d: %.2f, max_dist: %.2f\n",
    // dist, (float)sqrt(dx*dx + dy*dy), max_planning_dist_);
    dist = 1.0;
  }
  return dist;
}



void nav3d::Features3D::setWeights(vector<float> we)
{
  printf("%s. Setting weights: \n", name_.c_str());
  w_.clear();
  for (unsigned int i = 0; i < we.size(); i++)
  {
    // if(we[i] != 0.0) {
    w_.push_back(we[i]);
    // printf("we%u: %.3f, w_%u: %.3f\n", (i+1), we[i], (i+1), w_[i]);
    //}
  }

  for (unsigned int i = 0; i < w_.size(); i++)
  {
    printf("w_%u: %.3f\n", (i + 1), w_[i]);
  }
}



float nav3d::Features3D::getCost(geometry_msgs::PoseStamped* s)
{
  // boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);

  // configuration_mutex_.lock();

  vector<float> features = getFeatures(s);

  if (features[0] == 0.0)
  {
    printf("%s. getCost. Returning maximum cost\n", name_.c_str());
    return (1.0);
  }
  float cost = 0.0;
  for (unsigned int i = 1; i < w_.size(); i++)
  {
    cost += (w_[i] * features[i]);
  }

  // printf("3dnav_features. get Cost: %.3f\n", cost);
  return cost;
}



geometry_msgs::PoseStamped nav3d::Features3D::transformPoseTo(geometry_msgs::PoseStamped pose_in,
                                                              string frame_out, bool usetime)
{
  geometry_msgs::PoseStamped in = pose_in;
  if (!usetime)
    in.header.stamp = ros::Time();

  geometry_msgs::PoseStamped pose_out;

  geometry_msgs::Quaternion q = in.pose.orientation;
  if (!isQuaternionValid(q))
  {
    ROS_WARN("%s. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0",
             name_.c_str());
    in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }
  try
  {
    tf_->transform(pose_out, in, frame_out.c_str());
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("%s. TransformException in method transformPoseTo. TargetFrame: %s : %s",
             name_.c_str(), frame_out.c_str(), ex.what());
  }
  // printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f,
  // y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y,
  // frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
  return pose_out;
}



bool nav3d::Features3D::isQuaternionValid(const geometry_msgs::Quaternion q)
{
  // first we need to check if the quaternion has nan's or infs
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    ROS_ERROR("%s. Quaternion has infs!!!!", name_.c_str());
    return false;
  }
  if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w))
  {
    ROS_ERROR("%s. Quaternion has nans !!!", name_.c_str());
    return false;
  }

  if (std::fabs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01)
  {
    ROS_ERROR("%s. Quaternion malformed, magnitude: %.3f should be 1.0", name_.c_str(),
              (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w));
    return false;
  }

  tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

  // next, we need to check if the length of the quaternion is close to zero
  if (tf_q.length2() < 1e-6)
  {
    ROS_ERROR("%s. Quaternion has length close to zero... discarding.", name_.c_str());
    return false;
  }

  /*
    //next, we'll normalize the quaternion and check that it transforms the vertical
    vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must
    be close to vertical.");
      return false;
    }*/

  return true;
}



float nav3d::Features3D::normalizeAngle(float val, float min, float max)
{
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max - min));
  else
    norm = max - fmod((min - val), (max - min));

  return norm;
}
