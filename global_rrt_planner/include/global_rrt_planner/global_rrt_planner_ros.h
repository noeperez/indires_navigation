
#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <costmap_2d/costmap_2d.h>
//#include <dynamic_reconfigure/server.h>
#include <rrt_planners/ros/RRT_ros_wrapper.h>


namespace global_rrt_planner
{
class GlobalRRTPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
       * @brief  Default constructor for the PlannerCore object
       */
  GlobalRRTPlanner();

  /**
   * @brief  Constructor for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  frame_id Frame of the costmap
   */
  GlobalRRTPlanner(std::string name, std::string frame_id, tf2_ros::Buffer* tf);

  /**
   * @brief  Default deconstructor for the PlannerCore object
   */
  ~GlobalRRTPlanner();

  /**
   * @brief  Initialization function for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, std::string frame_id);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance The tolerance on the goal point for the planner
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan, double& cost);


  /**
   * @brief  Publish a path for visualization purposes
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

  bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);


private:
  tf2_ros::Buffer* tf_;
  RRT_ros::RRT_ros_wrapper* rrt_wrapper_;

  float default_tolerance_;
  bool initialized_;
  std::string frame_id_;

  ros::Publisher plan_pub_;
  ros::ServiceServer make_plan_srv_;
};

}  // end namespace global_rrt_planner

#endif
