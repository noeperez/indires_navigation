
//#ifndef NAV_FEATURES_
//#define NAV_FEATURES_

#include <math.h>
#include <vector>
#include <ctime>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
//#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>

// PCL
/*
//This solves the problem of compiling pcl search with C++11
#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE
*/
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/register_point_struct.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid_covariance.h>
//#include <pcl/octree/octree.h>
//#include <pcl/octree/octree_search.h>
//#include <pcl/search/octree.h>
//#include <pcl/search/search.h>



// Mutex
#include <mutex>

// OpenCV
//#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"



// Service msg
#include "pcl_filters/GetFilteredPC.h"
#include "pcl_filters/ChangeCropboxSize.h"


// Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <3D_navigation_features/nav_featuresConfig.h>


using namespace std;


namespace nav3d
{
class Features3D
{
public:
  struct exp_goal
  {
    geometry_msgs::Point p;
    float num_points;
    int visits;
  };

  Features3D();

  Features3D(string name, tf2_ros::Buffer* tf, float size_x, float size_y, float size_z);



  Features3D(string name, tf2_ros::Buffer* tf,
             vector<geometry_msgs::Point>* footprint, float size_x, float size_y, float size_z);

  ~Features3D();

  void setParams(string name);

  bool poseValid(geometry_msgs::PoseStamped* s);
  // bool poseValid2(geometry_msgs::PoseStamped* s);
  // bool poseValid3(geometry_msgs::PoseStamped* s);

  bool pose3dValid(geometry_msgs::PoseStamped* s);

  float getCost(geometry_msgs::PoseStamped* s);

  vector<float> getFeatures(geometry_msgs::PoseStamped* s);


  vector<vector<int> > clusterize_leaves(vector<geometry_msgs::Point>* points,
                                                   float radius);
  int evaluate_leaves(vector<geometry_msgs::Point>* points,
                      vector<float>* pcosts, string frame, float radius);
  float evaluate_leaf(geometry_msgs::Point* p, float rrt_cost, string frame,
                      float radius, float& npoints);
  // float evaluate_leaf_nfe_bfe(geometry_msgs::Point* p, float rrt_cost, std::string
  // frame, float radius, float &npoints);
  float no_return_cost(geometry_msgs::PoseStamped* p);



  // std::vector<float> getPathFeatureCount(vector<geometry_msgs::PoseStamped>* path);

  // void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void poseCallback(const nav_msgs::OdometryConstPtr& msg);


  // Feature: Distance to the goal
  float goalDistFeature(geometry_msgs::PoseStamped* s);


  // Feature: inclination in pitch and roll
  // float inclinationFeature(geometry_msgs::PoseStamped* s);


  // Feature: roughness of the terrain
  // float roughnessFeature(geometry_msgs::PoseStamped* s);


  // Feature: Distance to the closest obstacle (based on costmaps or map image)
  // float obstacleDistFeature(geometry_msgs::PoseStamped* s);


  geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in,
                                             string frame_out, bool usetime);

  bool isQuaternionValid(const geometry_msgs::Quaternion q);

  // pcl::normAngle(float alpha) //norm angle [-PI, PI]
  float normalizeAngle(float val, float min, float max);

  void setWeights(vector<float> we);

  void setGoal(geometry_msgs::PoseStamped g);

  inline string getRobotBaseFrame()
  {
    return robot_base_frame_;
  };

  inline string getRobotOdomFrame()
  {
    return robot_odom_frame_;
  };

  // void setObstacles(sensor_msgs::PointCloud2 obs);

  // void setScenario(sensor_msgs::PointCloud2 obs, geometry_msgs::PoseStamped goal);


  // Services
  // bool setWeightsService(navigation_features::SetWeights::Request  &req,
  // navigation_features::SetWeights::Response &res);
  // bool initializeWeightsService(navigation_features::InitWeights::Request  &req,
  // navigation_features::InitWeights::Response &res);
  // bool setLossService(navigation_features::SetLossCost::Request &req,
  // navigation_features::SetLossCost::Response &res);
  // bool setScenarioService(navigation_features::SetScenario::Request  &req,
  // navigation_features::SetScenario::Response &res);
  // bool isPoseValidService(navigation_features::PoseValid::Request &req,
  // navigation_features::PoseValid::Response &res);
  // bool getFeatureCountService(navigation_features::GetFeatureCount::Request &req,
  // navigation_features::GetFeatureCount::Response &res);



private:
  //tf::TransformListener* tf1_listener_;
  tf2_ros::TransformListener* tf_listener_;
  tf2_ros::Buffer* tf_;

  string name_;

  ros::Subscriber cloud_sub_;
  ros::ServiceClient exp_client_;
  string exp_pc_service_name_;
  // pcl::KdTreeFLANN<pcl::PointXYZ>*	kdtree_exp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr exp_cloud_;
  pcl::VoxelGridCovariance<pcl::PointXYZ>* vgc_;
  float cell_size_;
  vector<pcl::PointXYZ> wall_points_;
  ros::Publisher wall_pub_;
  vector<geometry_msgs::Point> frontier_points_;
  ros::Publisher frontier_pub_;
  ros::Publisher visited_pub_;
  sensor_msgs::PointCloud2 cloud_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> wall_cloud_;
  ros::Publisher pc_wall_pub_;
  vector<float> num_points_;
  float num_points_saturation_;
  // mutex 								cloudMutex_;


  pcl::KdTreeFLANN<pcl::PointXYZ>* kdtree_;
  mutex tree_mutex_;
  // boost::mutex						tree_mutex_;
  float robot_radius_;


  // const double						octree_resolution_ = 128.0;
  // float 								octree_resolution_;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_();
  // pcl::search::Octree<pcl::PointXYZ>* octree_;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_; //esto da error


  // pcl::PCA<pcl::PointXYZ>*			pca_;

  double pitch_high_;
  // double								pitch_high2_;
  double pitch_low_;
  // double								pitch_low2_;
  double roll_high_;
  // double 								roll_high2_;
  double roll_low_;
  // double 								roll_low2_;
  double roughness_;
  int min_points_allowed_;

  double pitch_high2_;
  double pitch_low2_;
  double roll_high2_;
  double roll_low2_;


  int feature_set_;


  vector<geometry_msgs::Point>* myfootprint_;

  string robot_base_frame_;
  string robot_odom_frame_;
  string robot_odom_topic_;


  ros::Subscriber pose_sub_;
  geometry_msgs::PoseStamped robot_pose_;
  vector<geometry_msgs::PoseStamped> robot_traj_;
  mutex pose_mutex_;
  // boost::mutex						pose_mutex_;

  geometry_msgs::PoseStamped goal_;
  float max_planning_dist_;
  float size_x_;
  float size_y_;
  float size_z_;
  float current_size_;
  float max_size_;
  float min_size_;

  ros::NodeHandle nh_;


  // ros::Subscriber 					goal_sub_;
  vector<float> w_;
  vector<float> wexp_;

  ros::Publisher explore_pub_;
  bool nfe_;  // Neirest Frontier Exploration
  bool bfe_;  // Biggest Frontier Exploration
  float threshold_frontier_;
  bool adaptative_threshold_;
  vector<exp_goal> exp_regions_;
  float exp_min_dist_goals_;
  float numpoint_cost_limit_;
  float percentage_limit_;
  bool first_rrt_time_;
  float rrt_time_;
  ros::Publisher rrt_time_pub_;

  bool visited_region_enabled_;
  bool remove_wall_frontier_enabled_;
  bool variable_size_enabled_;

  bool visualize_visited_reg_;
  bool visualize_frontiers_;
  bool visualize_wall_leaves_;



  // Dynamic reconfigure
  // boost::recursive_mutex configuration_mutex_;
  // mutex								configuration_mutex_;
  // dynamic_reconfigure::Server<3D_navigation_features::nav_featuresConfig> *dsrv_;
  // void reconfigureCB(3D_navigation_features::nav_featuresConfig &config, uint32_t
  // level);
};
}
//#endif
