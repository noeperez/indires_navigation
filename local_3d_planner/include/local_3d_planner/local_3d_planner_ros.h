/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Author: Noé Pérez Higueras
*********************************************************************/
#ifndef UPO_PLANNER_ROS_H_
#define UPO_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <local_3d_planner/local_3d_planner.h>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <thread>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

// Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <simple_local_planner/SimpleLocalPlannerConfig.h>


#include <local_3d_planner/odometry_helper_ros.h>


namespace local_3d_planner
{
/**
 * @class UpoPlannerROS
 * @brief A ROS wrapper for the trajectory controller that queries the param server to
 * construct a controller
 */
class Local3DPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief  Default constructor for the ros wrapper
   */
  Local3DPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  Local3DPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Destructor for the wrapper
   */
  ~Local3DPlannerROS();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot
   * base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();


  bool isInitialized()
  {
    return initialized_;
  }

  /** @brief Return the inner TrajectoryPlanner object.  Only valid after initialize(). */
  Local3DPlanner* getPlanner() const
  {
    return tc_;
  }



private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  // void reconfigureCB(SimpleLocalPlannerConfig &config, uint32_t level);


  Local3DPlanner* tc_;  ///< @brief The trajectory controller

  // tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
  tf2_ros::Buffer* tf_;

  std::string global_frame_;      ///< @brief The frame in which the controller will run
  std::string robot_base_frame_;  ///< @brief Used as the base frame id of the robot

  std::vector<geometry_msgs::PoseStamped> global_plan_;


  // Controller freq
  double controller_freq_;

  // Robot Configuration Parameters
  double max_vel_x_, min_vel_x_;
  double max_vel_th_, min_vel_th_;
  double max_trans_acc_;
  double max_rot_acc_;
  double min_in_place_vel_th_;

  double robot_radius_;
  double local_area_radius_;

  // Goal tolerance parameters
  double yaw_goal_tolerance_;
  double xyz_goal_tolerance_;
  double wp_tolerance_;

  double sim_time_;
  double sim_granularity_;
  double angular_sim_granularity_;


  double sim_period_;
  bool rotating_to_goal_;
  bool reached_goal_;


  ros::Publisher g_plan_pub_, l_plan_pub_;

  // dynamic_reconfigure::Server<SimpleLocalPlannerConfig> *dsrv_;
  // simple_local_planner::SimpleLocalPlannerConfig default_config_;
  bool setup_;


  bool initialized_;

  local_3d_planner::OdometryHelperRos odom_helper_;

  // std::vector<geometry_msgs::Point> footprint_spec_;
};
};
#endif
