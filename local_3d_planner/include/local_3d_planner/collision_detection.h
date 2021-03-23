#ifndef COLLISION_DETECTION_H_
#define COLLISION_DETECTION_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
//#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <local_3d_planner/odometry_helper_ros.h>

// Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <upo_local_planner/AssistedSteeringConfig.h>

//#include <boost/thread/mutex.hpp> //Mutex
#include <mutex>

#include <sys/time.h>

#include <navigation_features_3d/nav_features3d.h>



namespace local_3d_planner
{
class CollisionDetection
{
public:
  struct range
  {
    std::string id;
    float range;
    float polar_angle;
    float polar_dist;
    float fov;
    float min_dist;
    float max_dist;
  };


  // CollisionDetection();
  CollisionDetection(std::string name, tf2_ros::Buffer* tf,
                     local_3d_planner::OdometryHelperRos* oh, double max_lv,
                     double max_av, double lin_acc, double ang_acc, double sim_t,
                     double r_radius, double local_radius, double granularity);


  ~CollisionDetection();

  void setup(std::string name);


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
  bool checkTraj(double cvx, double cvy, double cvth, double tvx, double tvy, double tvth,
                 double& px, double& py, double& pz, double& pth);



  void saturateVelocities(geometry_msgs::Twist* twist);

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  std::vector<geometry_msgs::Point> laser_polar2euclidean(sensor_msgs::LaserScan* scan);

  void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void initiateRanges();


  bool inRangeCollision(float x, float y);
  bool inLaserCollision(float x, float y, std::vector<geometry_msgs::Point>* scanpoints);
  bool inCollision(float x, float y, std::vector<geometry_msgs::Point>* scanpoints);


private:
  float inline normalizeAngle(float val, float min, float max)
  {
    float norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max - min));
    else
      norm = max - fmod((min - val), (max - min));

    return norm;
  }



  /**
   * @brief  Compute x position based on velocity
   * @param  xi The current x position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new x position
   */
  inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt)
  {
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute y position based on velocity
   * @param  yi The current y position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new y position
   */
  inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt)
  {
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute orientation based on velocity
   * @param  thetai The current orientation
   * @param  vth The current theta velocity
   * @param  dt The timestep to take
   * @return The new orientation
   */
  inline double computeNewThetaPosition(double thetai, double vth, double dt)
  {
    return thetai + vth * dt;
  }

  // compute velocity based on acceleration
  /**
   * @brief  Compute velocity based on acceleration
   * @param vg The desired velocity, what we're accelerating up to
   * @param vi The current velocity
   * @param a_max An acceleration limit
   * @param  dt The timestep to take
   * @return The new velocity
   */
  inline double computeNewVelocity(double vg, double vi, double a_max, double dt)
  {
    if ((vg - vi) >= 0)
    {
      return std::min(vg, vi + a_max * dt);
    }
    return std::max(vg, vi - a_max * dt);
  }


  nav3d::Features3D* features_;


  //tf::TransformListener* tf_;
  tf2_ros::Buffer* tf_;


  std::string odom_topic_;
  std::string base_frame_;


  OdometryHelperRos* odom_helper_;



  double max_lin_vel_;
  double max_ang_vel_;
  double max_lin_acc_;
  double max_ang_acc_;
  double sim_time_;
  double robot_radius_;
  double robot_radius_aug_;
  double local_radius_;
  double granularity_;

  float max_lv_var_;
  float max_av_var_;

  tf::Stamped<tf::Pose> robot_vel_;


  // in case a laser range finder is also used
  bool use_laser_;
  ros::Subscriber laser_sub_;
  sensor_msgs::LaserScan laser_scan_;
  std::mutex laser_mutex_;


  // Sonar ranges employed
  bool use_range_;
  int num_ranges_;
  std::vector<std::string> range_topics_;
  std::vector<std::string> range_frames_;
  std::vector<ros::Subscriber> range_subscribers_;
  // ros::Subscriber 				range_sub_;
  std::vector<range> ranges_;
  std::mutex range_mutex_;
  std::vector<bool> ranges_initiated_;
  bool ranges_ready_;

  // Dynamic reconfigure
  // boost::recursive_mutex 		configuration_mutex_;
  // dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig> *dsrv_;
  // void reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t
  // level);
};
}
#endif
