/*********************************************************************
*
* Software License Agreement (BSD License)
*
*
* Author: Noé Pérez-Higueras
*********************************************************************/

#include <local_3d_planner/local_3d_planner_ros.h>

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <local_3d_planner/goal_functions.h>
#include <nav_msgs/Path.h>



// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_3d_planner::Local3DPlannerROS, nav_core::BaseLocalPlanner)

namespace local_3d_planner
{
// void PurePlannerROS::reconfigureCB(SimpleLocalPlannerConfig &config, uint32_t level) {
/*if (setup_ && config.restore_defaults) {
  config = default_config_;
  //Avoid looping
  config.restore_defaults = false;
}
if ( ! setup_) {
  default_config_ = config;
  setup_ = true;
}*/
// tc_->reconfigure(config);
// reached_goal_ = false;
//}

Local3DPlannerROS::Local3DPlannerROS()
  : tc_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom")
{
}

Local3DPlannerROS::Local3DPlannerROS(std::string name, tf2_ros::Buffer* tf,
                                     costmap_2d::Costmap2DROS* costmap_ros)
  : tc_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom")
{
  // initialize the planner
  initialize(name, tf, NULL);
}



void Local3DPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                   costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);


    tf_ = tf;


    ros::NodeHandle nh("~/" + name);


    std::string odom_topic;
    nh.param("odometry_topic", odom_topic, std::string("odom"));
    odom_helper_.setOdomTopic(odom_topic);


    nh.param("global_frame", global_frame_, std::string("odom"));
    // printf("local_3d_planner_ros.cpp. global_frame: %s\n", global_frame_.c_str());
    nh.param("base_frame", robot_base_frame_, std::string("base_link"));

    // Robot Configuration Parameters
    nh.param("max_trans_vel", max_vel_x_, 0.6);
    nh.param("min_trans_vel", min_vel_x_, 0.1);
    nh.param("max_rot_vel", max_vel_th_, 0.8);
    nh.param("min_rot_vel", min_vel_th_, 0.1);
    nh.param("max_trans_acc", max_trans_acc_, 1.0);
    nh.param("max_rot_acc", max_rot_acc_, 1.0);
    nh.param("min_in_place_rot_vel", min_in_place_vel_th_, 0.5);

    // Goal tolerance parameters
    nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.12);
    nh.param("xyz_goal_tolerance", xyz_goal_tolerance_, 0.20);
    nh.param("wp_tolerance", wp_tolerance_, 0.5);
    nh.param("sim_time", sim_time_, 1.0);
    nh.param("sim_granularity", sim_granularity_, 0.025);
    nh.param("angular_sim_granularity", angular_sim_granularity_, sim_granularity_);


    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.
    nh.param("controller_freq", controller_freq_, 15.0);

    bool dwa;
    nh.param("dwa", dwa, true);

    nh.param("robot_radius", robot_radius_, 0.345);
    nh.param("local_area_radius", local_area_radius_, 1.0);


    // footprint_spec_ = costmap_ros_->getRobotFootprint();

    tc_ = new Local3DPlanner(name, tf_, &odom_helper_, robot_radius_, local_area_radius_,
                             controller_freq_, max_vel_x_, min_vel_x_, max_vel_th_,
                             min_vel_th_, min_in_place_vel_th_, max_trans_acc_, max_rot_acc_,
                             yaw_goal_tolerance_, xyz_goal_tolerance_, wp_tolerance_,
                             sim_time_, sim_granularity_, angular_sim_granularity_, dwa);

    initialized_ = true;

    // BE CAREFUL, this will load the values of cfg params overwritting the read ones from
    // the yaml file.
    // dsrv_ = new dynamic_reconfigure::Server<SimpleLocalPlannerConfig>(private_nh);
    // dynamic_reconfigure::Server<SimpleLocalPlannerConfig>::CallbackType cb =
    // boost::bind(&PurePlannerROS::reconfigureCB, this, _1, _2);
    // dsrv_->setCallback(cb);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing");
  }
}



Local3DPlannerROS::~Local3DPlannerROS()
{
  // make sure to clean things up
  // delete dsrv_;

  if (tc_ != NULL)
    delete tc_;
}



bool Local3DPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset the goal flag
  reached_goal_ = false;

  return true;
}



bool Local3DPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> local_plan;

  /*
  tf::Stamped<tf::Pose> global_pose;
  tf::Stamped<tf::Pose> robot_orig;
  geometry_msgs::PoseStamped ro;
  ro.header.frame_id = robot_base_frame_;
  ro.header.stamp = ros::Time();
  ro.pose.position.x = 0.0;
  ro.pose.position.y = 0.0;
  ro.pose.position.z = 0.0;
  ro.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  tf::poseStampedMsgToTF(ro, robot_orig);
  tf_->transformPose(global_frame_, robot_orig, global_pose);
  */

  // tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped global_pose;
  nav_msgs::Odometry odom;
  odom_helper_.getOdom(odom);
  // tf::Stamped<tf::Pose> rg;
  geometry_msgs::PoseStamped ro;
  ro.header = odom.header;
  ro.pose.position = odom.pose.pose.position;
  ro.pose.orientation = odom.pose.pose.orientation;
  // tf::poseStampedMsgToTF(ro, rg);
  if (odom.header.frame_id != global_frame_)
  {
    global_pose = tf_->transform(ro, global_frame_);
  }
  else
    global_pose = ro;



  // TODO: Check here if we have already reached the goal


  // In case that the global plan is in map frame, we tranform it to local frame (odom)
  // and we cut the part that lies on the local area
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, global_frame_,
                           local_area_radius_, transformed_plan))
  {  // TransformGlobalPlan belongs to goal_functions.cpp
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }


  // now we'll prune the plan based on the position of the robot
  // if(prune_plan_)
  prunePlan(global_pose, transformed_plan,
            global_plan_);  // PrunePlan belongs to goal_functions.cpp



  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  /* For timing uncomment
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  */

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
    return false;


  tc_->updatePlan(transformed_plan);

  geometry_msgs::Twist drive_cmds;
  bool ok = tc_->findBestAction(global_pose, robot_vel, drive_cmds);



  /* For timing uncomment
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  ROS_INFO("Cycle time: %.9f", t_diff);
  */

  // pass along drive commands
  cmd_vel = drive_cmds;
  if (!ok)
  {
    ROS_DEBUG_NAMED("local_3d_planner_ros",
                    "The local_3d_planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
    ROS_WARN("ComputeVelocityCommands. The local_3d_planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
    publishPlan(transformed_plan, g_plan_pub_);
    return false;
  }


  // publish information to the visualizer
  publishPlan(transformed_plan, g_plan_pub_);
  // publishPlan(local_plan, l_plan_pub_);
  return true;
}



bool Local3DPlannerROS::isGoalReached()
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  // return flag set in controller
  // return reached_goal_;
  return tc_->isGoalReached();
}
};
