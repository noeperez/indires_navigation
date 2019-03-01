
#ifndef NAV_FEATURES_
#define NAV_FEATURES_

#include <math.h>
#include <vector>
#include <ctime>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <upo_msgs/PersonPoseUPO.h>
//#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>

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
//#include <pcl/octree/octree.h>
//#include <pcl/octree/octree_search.h>
//#include <pcl/search/octree.h>
//#include <pcl/search/search.h>





//Mutex
#include <mutex> 
//#include <boost/thread/mutex.hpp> 

//OpenCV
//#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"



//Service msg
//#include <navigation_features/SetWeights.h>
//#include <navigation_features/SetLossCost.h>
//#include <navigation_features/SetScenario.h>
//#include <navigation_features/PoseValid.h>
//#include <navigation_features/GetFeatureCount.h>
//#include <navigation_features/InitWeights.h>

//Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <3D_navigation_features/nav_featuresConfig.h>


using namespace std;


namespace nav3d {

	class Features3D {

		public:
		
			/*enum gaussian_type{
				FRONT, 
				BACK, 
				LEFT, 
				RIGHT, 
				AROUND, 
				FRONT_APPROACH,
				AROUND_APPROACH
			};
			
			enum dist_type{LINEAR_INC,LOG_INC,EXP_INC,INVERSE_DEC,LOG_DEC,EXP_DEC};
			*/

			Features3D();
			
			Features3D(tf::TransformListener* tf, float size_x, float size_y, float size_z);
			
			//NavFeatures(tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y);

			Features3D(tf::TransformListener* tf, vector<geometry_msgs::Point>* footprint, float size_x, float size_y, float size_z);

			~Features3D();

			void setParams();

			bool poseValid(geometry_msgs::PoseStamped* s);
			//bool poseValid2(geometry_msgs::PoseStamped* s);
			//bool poseValid3(geometry_msgs::PoseStamped* s);

			bool pose3dValid(geometry_msgs::PoseStamped* s);

			float getCost(geometry_msgs::PoseStamped* s);
		
			std::vector<float> getFeatures(geometry_msgs::PoseStamped* s);
			
			
			std::vector<std::vector<int> > clusterize_leaves(std::vector<geometry_msgs::Point>* points, float radius);
			std::vector<float> evaluate_leaves(std::vector<geometry_msgs::Point>* points, float radius);
			float no_return_cost(geometry_msgs::PoseStamped* p);
			

			//std::vector<float> getPathFeatureCount(vector<geometry_msgs::PoseStamped>* path);
			
			//void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
			
			void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

			void poseCallback(const nav_msgs::OdometryConstPtr &msg);
			
		
			//Feature: Distance to the goal
			float goalDistFeature(geometry_msgs::PoseStamped* s);


			//Feature: inclination in pitch and roll
			//float inclinationFeature(geometry_msgs::PoseStamped* s);


			//Feature: roughness of the terrain
			//float roughnessFeature(geometry_msgs::PoseStamped* s);

		
			//Feature: Distance to the closest obstacle (based on costmaps or map image)
			//float obstacleDistFeature(geometry_msgs::PoseStamped* s);
		

			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
		
			bool isQuaternionValid(const geometry_msgs::Quaternion q);
			
			//pcl::normAngle(float alpha) //norm angle [-PI, PI]
			float normalizeAngle(float val, float min, float max);
		
			void setWeights(std::vector<float> we);

			void setGoal(geometry_msgs::PoseStamped g); 

			//void setObstacles(sensor_msgs::PointCloud2 obs);

			//void setScenario(sensor_msgs::PointCloud2 obs, geometry_msgs::PoseStamped goal);


			//Services
			//bool setWeightsService(navigation_features::SetWeights::Request  &req, navigation_features::SetWeights::Response &res);
			//bool initializeWeightsService(navigation_features::InitWeights::Request  &req, navigation_features::InitWeights::Response &res);
			//bool setLossService(navigation_features::SetLossCost::Request &req, navigation_features::SetLossCost::Response &res);
			//bool setScenarioService(navigation_features::SetScenario::Request  &req, navigation_features::SetScenario::Response &res);
			//bool isPoseValidService(navigation_features::PoseValid::Request &req, navigation_features::PoseValid::Response &res);
			//bool getFeatureCountService(navigation_features::GetFeatureCount::Request &req, navigation_features::GetFeatureCount::Response &res);
			


		private:

			tf::TransformListener* 				tf_listener_;
		
			ros::Subscriber 					cloud_sub_;
			sensor_msgs::PointCloud2 			cloud_;
			pcl::PointCloud<pcl::PointXYZ>		pcl_cloud_;
			//mutex 								cloudMutex_;


			pcl::KdTreeFLANN<pcl::PointXYZ>*	kdtree_;
			mutex 								tree_mutex_;
			//boost::mutex						tree_mutex_;
			float								robot_radius_;
			
			
			//const double						octree_resolution_ = 128.0;
			//float 								octree_resolution_;
			//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_();
			//pcl::search::Octree<pcl::PointXYZ>* octree_;
			//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
			//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_; //esto da error


			//pcl::PCA<pcl::PointXYZ>*			pca_;

			double								pitch_high_;
			double								pitch_high2_;
			double								pitch_low_;
			double								pitch_low2_;
			double 								roll_high_;
			double 								roll_high2_;
			double 								roll_low_;
			double 								roll_low2_;
			double								roughness_;
			int 								min_points_allowed_;

			
			int									feature_set_;
			
			
			vector<geometry_msgs::Point>* 		myfootprint_;

			std::string							robot_base_frame_;
			std::string							robot_odom_frame_;
			std::string							robot_odom_topic_;

			ros::Subscriber						pose_sub_;
			geometry_msgs::PoseStamped			robot_pose_;
			vector<geometry_msgs::PoseStamped>	robot_traj_;
			mutex								pose_mutex_;
			//boost::mutex						pose_mutex_;
		
			geometry_msgs::PoseStamped 			goal_;
			float 								max_planning_dist_;
			float 								size_x_;
			float 								size_y_;
			float								size_z_;

			ros::NodeHandle 					nh_;
		
			
			//ros::Subscriber 					goal_sub_;
			vector<float>	 					w_;
			
			ros::Publisher						explore_pub_;
			bool								nfe_exploration_;

			
			
			//Dynamic reconfigure
			//boost::recursive_mutex configuration_mutex_;
			//mutex								configuration_mutex_;
			//dynamic_reconfigure::Server<3D_navigation_features::nav_featuresConfig> *dsrv_;
			//void reconfigureCB(3D_navigation_features::nav_featuresConfig &config, uint32_t level);
			

	};

}
#endif
