/*********************************************************************
*
* Based on the goal_functions.h of the base_local_planner of ROS.
* Modified by: Noé Pérez Higueras
*********************************************************************/
#ifndef LOCAL_3D_PLANNER_GOAL_FUNCTIONS_H_
#define LOCAL_3D_PLANNER_GOAL_FUNCTIONS_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <string>
#include <cmath>

#include <angles/angles.h>
//#include <costmap_2d/costmap_2d.h>

namespace local_3d_planner {

  /**
   * @brief  return squared distance to check if the goal position has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return distance to goal
   */
  double getGoalPositionDistance(const tf::Stamped<tf::Pose>& global_pose, double goal_x, double goal_y);

  /**
   * @brief  return angle difference to goal to check if the goal orientation has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return angular difference
   */
  double getGoalOrientationAngleDifference(const tf::Stamped<tf::Pose>& global_pose, double goal_th);

  /**
   * @brief  Publish a plan for visualization purposes
   * @param  path The plan to publish
   * @param  pub The published to use
   * @param  r,g,b,a The color and alpha value to use when publishing
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

  /**
   * @brief  Trim off parts of the global plan that are far enough behind the robot
   * @param global_pose The pose of the robot in the global frame
   * @param plan The plan to be pruned
   * @param global_plan The plan to be pruned in the frame of the planner
   */
  void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan);

  /**
   * @brief  Transforms the global plan of the robot from the planner frame to the frame of the costmap,
   * selects only the (first) part of the plan that is within the costmap area.
   * @param tf A reference to a transform listener
   * @param global_plan The plan to be transformed
   * @param robot_pose The pose of the robot in the global frame (same as costmap)
   * @param global_frame The frame to transform the plan to
   * @param local_area_radius The radius of the local area around the robot considered by the planner
   * @param transformed_plan Populated with the transformed plan
   */
  bool transformGlobalPlan(const tf::TransformListener& tf,
	  const std::vector<geometry_msgs::PoseStamped>& global_plan,
	  const tf::Stamped<tf::Pose>& global_robot_pose,
	  const std::string& global_frame,
	  const double& local_area_radius,
	  std::vector<geometry_msgs::PoseStamped>& transformed_plan);


  /**
     * @brief  Returns last pose in plan
     * @param tf A reference to a transform listener
     * @param global_plan The plan being followed
     * @param global_frame The global frame of the local planner
     * @param goal_pose the pose to copy into
     * @return True if achieved, false otherwise
     */
  bool getGoalPose(const tf::TransformListener& tf,
  		  const std::vector<geometry_msgs::PoseStamped>& global_plan,
  		  const std::string& global_frame,
  		  tf::Stamped<tf::Pose> &goal_pose);

  /**
   * @brief  Check if the goal pose has been achieved
   * @param tf A reference to a transform listener
   * @param global_plan The plan being followed
   * @param global_frame The global frame of the local planner
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_vel The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_vel The translational velocity below which the robot is considered stopped
   * @param xy_goal_tolerance The translational tolerance on reaching the goal
   * @param yaw_goal_tolerance The rotational tolerance on reaching the goal
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(const tf::TransformListener& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      //const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      tf::Stamped<tf::Pose>& global_pose,
      const nav_msgs::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance);

  /**
   * @brief  Check whether the robot is stopped or not
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_velocity The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_velocity The translational velocity below which the robot is considered stopped
   * @return True if the robot is stopped, false otherwise
   */
  bool stopped(const nav_msgs::Odometry& base_odom, 
      const double& rot_stopped_velocity,
      const double& trans_stopped_velocity);
};
#endif
