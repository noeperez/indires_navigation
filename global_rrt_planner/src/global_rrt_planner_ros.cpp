

#include <global_rrt_planner/global_rrt_planner_ros.h>
#include <pluginlib/class_list_macros.h>


// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_rrt_planner::GlobalRRTPlanner, nav_core::BaseGlobalPlanner)


namespace global_rrt_planner
{
GlobalRRTPlanner::GlobalRRTPlanner()
{
  initialize(std::string("GlobalRRTPlanner"), std::string(""));
}



GlobalRRTPlanner::GlobalRRTPlanner(std::string name, std::string frame_id, tf2_ros::Buffer* tf)
{
  tf_ = tf;
  // initialize the planner
  initialize(name, frame_id);
}



GlobalRRTPlanner::~GlobalRRTPlanner()
{
  if (rrt_wrapper_)
    delete rrt_wrapper_;
  if (tf_)
    delete tf_;
}


void GlobalRRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, std::string(""));
}



void GlobalRRTPlanner::initialize(std::string name, std::string frame_id)
{
  if (!initialized_)
  {
    tf_ = new tf2_ros::Buffer();
    tf2_ros::TransformListener tfListener(*tf_);
    sleep(5.0);
    ros::NodeHandle private_nh("~/" + name);

    if (frame_id.empty())
    {
      ros::NodeHandle nh("~/");
      nh.param(std::string("global_costmap/global_frame"), frame_id_, std::string("odom"));
    }
    else
      frame_id_ = frame_id;

    rrt_wrapper_ = new RRT_ros::RRT_ros_wrapper(tf_);


    // private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    // potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

    make_plan_srv_ =
        private_nh.advertiseService("make_plan", &GlobalRRTPlanner::makePlanService, this);

    // dsrv_ = new
    // dynamic_reconfigure::Server<global_rrt_planner::GlobalRRTPlannerConfig>(ros::NodeHandle("~/"
    // + name));
    // dynamic_reconfigure::Server<global_rrt_planner::GlobalRRTPlannerConfig>::CallbackType
    // cb = boost::bind(
    //        &GlobalRRTPlanner::reconfigureCB, this, _1, _2);
    // dsrv_->setCallback(cb);

    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}



bool GlobalRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
  double cost = 0.0;
  return makePlan(start, goal, plan, cost);
}


bool GlobalRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan, double& cost)
{
  // bool GlobalRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const
  // geometry_msgs::PoseStamped& goal, double tolerance,
  // std::vector<geometry_msgs::PoseStamped>& plan) {

  plan.clear();

  bool explore = false;
  if (goal.header.frame_id.empty())
  {
    printf("Global rrt planner. Goal empty! Passing to EXPLORATION MODE!!! \n");
    explore = true;
  }

  // geometry_msgs::PoseStamped start;
  // start.header.stamp = ros::Time::now();
  // start.header.frame_id = "base_link";
  // start.pose.position.x = 0.0;
  // start.pose.position.y = 0.0;
  // start.pose.position.z = 0.0;
  // start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Inside RRT_plan method, start and goal are transformed to the local frame
  plan = rrt_wrapper_->RRT_plan(explore, start, goal, 0.0, 0.0);
  cost = (double)rrt_wrapper_->get_path_cost();

  // Visualize the tree nodes of the resulting path
  if (plan.empty())
    return false;


  // Transform plan to the given frame_id
  // in case that it is different
  if (plan[0].header.frame_id != frame_id_)
  {
    for (unsigned int i = 0; i < plan.size(); i++)
    {
      // geometry_msgs::PoseStamped p = plan[i];
      geometry_msgs::PoseStamped pt;
      try
      {
        //tf_->transformPose(frame_id_, plan[i], pt);
        tf_->transform(pt, plan[i], frame_id_);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR("global_rrt_planner_ros. MakePlan. TransformException: %s", ex.what());
        continue;
      }
      plan[i] = pt;
    }
  }

  publishPlan(plan);
  return true;
}



void GlobalRRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  gui_path.header.frame_id = frame_id_;
  gui_path.header.stamp = ros::Time::now();

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}



bool GlobalRRTPlanner::makePlanService(nav_msgs::GetPlan::Request& req,
                                       nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);

  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}



}  // end namespace global_rrt_planner
