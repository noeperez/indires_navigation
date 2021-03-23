#include <indires_macro_actions/Indires_macro_actions.h>


/*
Status can take this values:
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started
executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
(Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server
due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being
processed,
                            #    because the goal was unattainable or invalid (Terminal
State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started
executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started
executing,
                            #    but the action server has not yet confirmed that the goal
is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started
executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This
should not be
                            #    sent over the wire by an action server
*/

// namespace macroactions {

Indires_macro_actions::Indires_macro_actions(tf2_ros::Buffer* tf)
{
  tf_ = tf;

  // UpoNav_ = nav;

  ros::NodeHandle n("~");

  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);



  n.param<double>("secs_to_check_block", secs_to_check_block_, 5.0);  // seconds
  n.param<double>("block_dist", block_dist_, 0.4);                    // meters
  // n.param<double>("secs_to_wait", secs_to_wait_, 8.0);  //seconds
  n.param<double>("control_frequency", control_frequency_, 15.0);
  std::string odom_topic = "";
  n.param<std::string>("odom_topic", odom_topic, "odom");

  manual_control_ = false;


  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
  // dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType
  // cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);


  ros::NodeHandle nh;

  rrtgoal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
      "/rrt_goal", 1, &Indires_macro_actions::rrtGoalCallback, this);
  pose_sub_ = nh.subscribe<nav_msgs::Odometry>(
      odom_topic.c_str(), 1, &Indires_macro_actions::robotPoseCallback, this);


  // Services for walking side by side
  // start_client_ = nh.serviceClient<teresa_wsbs::start>("/wsbs/start");
  // stop_client_ = nh.serviceClient<teresa_wsbs::stop>("/wsbs/stop");
  // wsbs_status_sub_ = nh.subscribe<std_msgs::UInt8>("/wsbs/status", 1,
  // &Upo_navigation_macro_actions::wsbsCallback, this);


  moveBaseClient_ =
      new moveBaseClient("move_base", true);  // true-> do not need ros::spin()
  ROS_INFO("Waiting for action server to start...");
  moveBaseClient_->waitForServer();
  ROS_INFO("Action server connected!");


  // Initialize action servers
  NWActionServer_ = new NWActionServer(
      nh1_, "NavigateWaypoint",
      boost::bind(&Indires_macro_actions::navigateWaypointCB, this, _1), false);
  NHActionServer_ = new NHActionServer(
      nh2_, "NavigateHome", boost::bind(&Indires_macro_actions::navigateHomeCB, this, _1), false);
  ExActionServer_ = new ExActionServer(
      nh3_, "Exploration", boost::bind(&Indires_macro_actions::explorationCB, this, _1), false);
  TOActionServer_ =
      new TOActionServer(nh4_, "Teleoperation",
                         boost::bind(&Indires_macro_actions::teleoperationCB, this, _1), false);

  NWActionServer_->start();
  NHActionServer_->start();
  ExActionServer_->start();
  TOActionServer_->start();

  // ros::NodeHandle nodeh("~/RRT_ros_wrapper");
  // nodeh.getParam("full_path_stddev", initial_stddev_);
}


Indires_macro_actions::~Indires_macro_actions()
{
  if (NWActionServer_)
    delete NWActionServer_;
  if (NHActionServer_)
    delete NHActionServer_;
  if (ExActionServer_)
    delete ExActionServer_;
  if (TOActionServer_)
    delete TOActionServer_;

  // if(UpoNav_)
  //	delete UpoNav_;
  // if(dsrv_)
  //	delete dsrv_;
}



/*
void
Upo_navigation_macro_actions::reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig
&config, uint32_t level){

    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    control_frequency_ = config.control_frequency;
    secs_to_check_block_ = config.secs_to_check_block;
  block_dist_ = config.block_dist;
  secs_to_wait_ = config.secs_to_wait;
  social_approaching_type_ = config.social_approaching_type;
  secs_to_yield_ = config.secs_to_yield;
  //use_leds_ = config.use_leds;
  //leds_number_ = config.leds_number;

}*/



/*
//Receive feedback messages from upo_navigation
void Upo_navigation_macro_actions::feedbackReceived(const
move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {

  pose_mutex_.lock();
  robot_pose_ = msg->feedback.base_position;
  pose_mutex_.unlock();
  if((unsigned int)(std::string(robot_pose_.header.frame_id).size()) < 3)
    robot_pose_.header.frame_id = "map";
}


//Receive status messages from upo_navigation
void Upo_navigation_macro_actions::statusReceived(const
actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  unsigned int actions = msg->status_list.size();
  if(actions != 0)
  {
    status_mutex_.lock();
    nav_status_ = msg->status_list.at(0).status;
    nav_text_ = msg->status_list.at(0).text;
    goal_id_ = msg->status_list.at(0).goal_id.id;
    status_mutex_.unlock();
  } else {
    status_mutex_.lock();
    nav_status_ = -1;
    nav_text_ = " ";
    goal_id_ = " ";
    status_mutex_.unlock();
  }
}


//This topic publishes only when the action finishes (because of reaching the goal or
cancelation)
void Upo_navigation_macro_actions::resultReceived(const
move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  action_end_ = true;
}*/



/*
MoveBase server:
-----------------
Action Subscribed topics:
move_base/goal		[move_base_msgs::MoveBaseActionGoal]
move_base/cancel 	[actionlib_msgs::GoalID]

Action Published topcis:
move_base/feedback	[move_base_msgs::MoveBaseActionFeedback]
move_base/status	[actionlib_msgs::GoalStatusArray]
move_base/result	[move_base_msgs::MoveBaseAcionResult]
*/

void Indires_macro_actions::navigateWaypointCB(
    const indires_macro_actions::NavigateWaypointGoal::ConstPtr& goal)
{
  printf("¡¡¡¡¡¡¡MacroAction navigatetoWaypoint  -->  started!!!!!!\n");
  // printf("Goal  x: %.2f, y: %.2f frame: %s\n", goal->target_pose.pose.position.x,
  // goal->target_pose.pose.position.y, goal->target_pose.header.frame_id.c_str());
  // UpoNav_->stopRRTPlanning();

  move_base_msgs::MoveBaseGoal g;
  g.target_pose = goal->target_pose;
  moveBaseClient_->sendGoal(g);

  // moveBaseClient_->waitForResult();

  actionlib::SimpleClientGoalState state = moveBaseClient_->getState();


  // moveBaseClient_->cancelAllGoals()
  // moveBaseClient_->cancelGoal()
  // moveBaseClient_->getResult()


  if (moveBaseClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Success!!!");
  } /*else {
    ROS_INFO("Failed!");
  }*/

  /*bool ok = UpoNav_->executeNavigation(goal->target_pose);
  if(!ok)
  {
    ROS_INFO("Setting ABORTED state 1");
    nwresult_.result = "Aborted. Navigation error";
    nwresult_.value = 2;
    NWActionServer_->setAborted(nwresult_, "Navigation aborted");
    //UpoNav_->stopRRTPlanning();
    return;
  }*/

  ros::Rate r(control_frequency_);
  // int pursue_status = 0;
  bool exit = false;
  ros::Time time_init = ros::Time::now();
  bool first = true;
  nav_msgs::Odometry pose_init;
  // ros::WallTime startt;
  while (nh1_.ok())
  {
    // startt = ros::WallTime::now();

    if (NWActionServer_->isPreemptRequested())
    {
      if (NWActionServer_->isNewGoalAvailable())
      {
        indires_macro_actions::NavigateWaypointGoal new_goal =
            *NWActionServer_->acceptNewGoal();

        g.target_pose = new_goal.target_pose;
        moveBaseClient_->sendGoal(g);


        /*bool ok = UpoNav_->executeNavigation(new_goal.target_pose);
        if(!ok) {
    ROS_INFO("Setting ABORTED state 1");
    nwresult_.result = "Aborted. Navigation error";
    nwresult_.value = 2;
    NWActionServer_->setAborted(nwresult_, "Navigation aborted");
    UpoNav_->stopRRTPlanning();
    if(use_leds_)
      setLedColor(WHITE);
    return;
  }*/
        first = true;
      }
      else
      {
        // if we've been preempted explicitly we need to shut things down
        // UpoNav_->resetState();

        // Cancel?????

        // notify the ActionServer that we've successfully preempted
        nwresult_.result = "Preempted";
        nwresult_.value = 1;
        ROS_DEBUG_NAMED("indires_macro_actions", "indires_navigation preempting the current goal");
        NWActionServer_->setPreempted(nwresult_, "Navigation preempted");
        // we'll actually return from execute after preempting
        return;
      }
    }

    pose_mutex_.lock();
    nav_msgs::Odometry new_pose = odom_pose_;
    pose_mutex_.unlock();

    // pursue_status = UpoNav_->pathFollow(new_pose);

    // Posible states:
    // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    actionlib::SimpleClientGoalState state = moveBaseClient_->getState();


    if (first)
    {
      pose_init = new_pose;
      time_init = ros::Time::now();
      first = false;
    }



    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // Goal reached
      ROS_INFO("Setting SUCCEEDED state");
      nwresult_.result = "Navigation succeeded";
      nwresult_.value = 0;
      NWActionServer_->setSucceeded(nwresult_, "Goal Reached");
      nwfeedback_.text = "Succeeded";
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      // Goal not reached, continue navigating
      nwfeedback_.text = "Navigating";
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      // Aborted
      nwfeedback_.text = "Aborted";
      ROS_INFO("Setting ABORTED state");
      nwresult_.result = "Aborted";
      nwresult_.value = 3;
      NWActionServer_->setAborted(nwresult_, "Navigation aborted");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      nwfeedback_.text = "Pending";
      // ROS_INFO("Setting ABORTED state");
      // nwresult_.result = "";
      // nwresult_.value = 3;
      // NWActionServer_->setAborted(nwresult_, "Navigation aborted");
      // exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::RECALLED)
    {
      nwfeedback_.text = "Recalled";
    }
    else if (state == actionlib::SimpleClientGoalState::REJECTED)
    {
      // Rejected
      nwfeedback_.text = "Rejected";
      ROS_INFO("Setting ABORTED (rejected) state");
      nwresult_.result = "Rejected";
      nwresult_.value = 3;
      NWActionServer_->setAborted(nwresult_, "Navigation aborted(rejected)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::LOST)
    {
      // Rejected
      nwfeedback_.text = "Lost";
      ROS_INFO("Setting ABORTED (lost) state");
      nwresult_.result = "Lost";
      nwresult_.value = 3;
      NWActionServer_->setAborted(nwresult_, "Navigation aborted(lost)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      nwfeedback_.text = "Preempted";
    }


    // push the feedback out
    geometry_msgs::PoseStamped aux;
    aux.header = new_pose.header;
    aux.pose.position = new_pose.pose.pose.position;
    aux.pose.orientation = new_pose.pose.pose.orientation;
    nwfeedback_.base_position = aux;
    NWActionServer_->publishFeedback(nwfeedback_);

    if (exit)
    {
      // UpoNav_->stopRRTPlanning();
      return;
    }


    // check the blocked situation.
    double time = (ros::Time::now() - time_init).toSec();
    // printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(),
    // time_init.toSec(), time);
    if (time > secs_to_check_block_)
    {
      double xinit = pose_init.pose.pose.position.x;
      double yinit = pose_init.pose.pose.position.y;
      double hinit = tf::getYaw(pose_init.pose.pose.orientation);
      double xnow = new_pose.pose.pose.position.x;
      double ynow = new_pose.pose.pose.position.y;
      double hnow = tf::getYaw(new_pose.pose.pose.orientation);
      double dist = sqrt(pow((xinit - xnow), 2) + pow((yinit - ynow), 2));
      double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
      // printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
      if (dist <= block_dist_ && yaw_diff < 0.79)
      {  // 0.79 = 45º
        ROS_INFO("Setting ABORTED state because of blocked situation");
        nwresult_.result = "Aborted. Blocked situation";
        nwresult_.value = 5;
        NWActionServer_->setAborted(nwresult_, "Navigation aborted. blocked");
        nwfeedback_.text = "Blocked";
        NWActionServer_->publishFeedback(nwfeedback_);
        // UpoNav_->stopRRTPlanning();
        return;
      }
      else
      {
        pose_init = new_pose;
        time_init = ros::Time::now();
      }
    }

    // ros::WallDuration dur = ros::WallTime::now() - startt;
    // printf("Loop time: %.4f secs\n", dur.toSec());

    r.sleep();
  }

  ROS_INFO("Setting ABORTED state");
  nwresult_.result = "Aborted. System is shuting down";
  nwresult_.value = 6;
  NWActionServer_->setAborted(nwresult_, "Navigation aborted because the node has been killed");
}



void Indires_macro_actions::navigateHomeCB(const indires_macro_actions::NavigateHomeGoal::ConstPtr& goal)
{
  printf("¡¡¡¡¡¡¡MacroAction NavigateHome  -->  started!!!!!!\n");
  // printf("Goal  x: %.2f, y: %.2f frame: %s\n", goal->home_pose.pose.position.x,
  // goal->home_pose.pose.position.y, goal->home_pose.header.frame_id.c_str());
  // UpoNav_->stopRRTPlanning();

  // boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Put the goal to map origin???
  move_base_msgs::MoveBaseGoal g;
  g.target_pose = goal->home_pose;
  moveBaseClient_->sendGoal(g);


  ros::Rate r(control_frequency_);
  // int pursue_status = 0;
  bool exit = false;
  ros::Time time_init = ros::Time::now();
  bool first = true;
  nav_msgs::Odometry pose_init;

  while (nh2_.ok())
  {
    // startt = ros::WallTime::now();

    if (NHActionServer_->isPreemptRequested())
    {
      if (NHActionServer_->isNewGoalAvailable())
      {
        indires_macro_actions::NavigateHomeGoal new_goal = *NHActionServer_->acceptNewGoal();

        g.target_pose = new_goal.home_pose;
        moveBaseClient_->sendGoal(g);
        first = true;
      }
      else
      {
        // if we've been preempted explicitly we need to shut things down
        // UpoNav_->resetState();

        // notify the ActionServer that we've successfully preempted
        ROS_DEBUG_NAMED("indires_macro_actions", "indires_navigation preempting the current goal");
        nhresult_.result = "Preempted";
        nhresult_.value = 1;
        NHActionServer_->setPreempted(nhresult_, "Navigation preempted");
        // we'll actually return from execute after preempting
        return;
      }
    }

    pose_mutex_.lock();
    nav_msgs::Odometry new_pose = odom_pose_;
    pose_mutex_.unlock();

    // pursue_status = UpoNav_->pathFollow(new_pose);

    // Posible states:
    // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    actionlib::SimpleClientGoalState state = moveBaseClient_->getState();


    if (first)
    {
      pose_init = new_pose;
      time_init = ros::Time::now();
      first = false;
    }



    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // Goal reached
      ROS_INFO("Setting SUCCEEDED state");
      nhresult_.result = "Navigation succeeded";
      nhresult_.value = 0;
      NHActionServer_->setSucceeded(nhresult_, "Goal Reached");
      nhfeedback_.text = "Succeeded";
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      // Goal not reached, continue navigating
      nhfeedback_.text = "Navigating";
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      // Aborted
      nhfeedback_.text = "Aborted";
      ROS_INFO("Setting ABORTED state");
      nhresult_.result = "Aborted";
      nhresult_.value = 3;
      NHActionServer_->setAborted(nhresult_, "Navigation aborted");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      nhfeedback_.text = "Pending";
      // ROS_INFO("Setting ABORTED state");
      // nwresult_.result = "";
      // nwresult_.value = 3;
      // NWActionServer_->setAborted(nwresult_, "Navigation aborted");
      // exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::RECALLED)
    {
      nhfeedback_.text = "Recalled";
    }
    else if (state == actionlib::SimpleClientGoalState::REJECTED)
    {
      // Rejected
      nhfeedback_.text = "Rejected";
      ROS_INFO("Setting ABORTED (rejected) state");
      nhresult_.result = "Rejected";
      nhresult_.value = 3;
      NHActionServer_->setAborted(nhresult_, "Navigation aborted(rejected)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::LOST)
    {
      // Rejected
      nhfeedback_.text = "Lost";
      ROS_INFO("Setting ABORTED (lost) state");
      nhresult_.result = "Lost";
      nhresult_.value = 3;
      NHActionServer_->setAborted(nhresult_, "Navigation aborted(lost)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      nhfeedback_.text = "Preempted";
    }


    // push the feedback out
    geometry_msgs::PoseStamped aux;
    aux.header = new_pose.header;
    aux.pose.position = new_pose.pose.pose.position;
    aux.pose.orientation = new_pose.pose.pose.orientation;
    nhfeedback_.base_position = aux;
    NHActionServer_->publishFeedback(nhfeedback_);

    if (exit)
    {
      // UpoNav_->stopRRTPlanning();
      return;
    }



    // check the blocked situation.
    double time = (ros::Time::now() - time_init).toSec();
    // printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(),
    // time_init.toSec(), time);
    if (time > secs_to_check_block_)
    {
      double xinit = pose_init.pose.pose.position.x;
      double yinit = pose_init.pose.pose.position.y;
      double hinit = tf::getYaw(pose_init.pose.pose.orientation);
      double xnow = new_pose.pose.pose.position.x;
      double ynow = new_pose.pose.pose.position.y;
      double hnow = tf::getYaw(new_pose.pose.pose.orientation);
      double dist = sqrt(pow((xinit - xnow), 2) + pow((yinit - ynow), 2));
      double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
      // printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
      if (dist <= block_dist_ && yaw_diff < 0.79)
      {  // 0.79 = 45º
        ROS_INFO("Setting ABORTED state because of blocked situation");
        nhresult_.result = "Aborted. Blocked situation";
        nhresult_.value = 5;
        NHActionServer_->setAborted(nhresult_, "Navigation aborted. blocked");
        nhfeedback_.text = "Blocked";
        NHActionServer_->publishFeedback(nhfeedback_);
        // UpoNav_->stopRRTPlanning();
        return;
      }
      else
      {
        pose_init = new_pose;
        time_init = ros::Time::now();
      }
    }

    // ros::WallDuration dur = ros::WallTime::now() - startt;
    // printf("Loop time: %.4f secs\n", dur.toSec());

    r.sleep();
  }

  ROS_INFO("Setting ABORTED state");
  nhresult_.result = "Aborted. system is shuting down";
  nhresult_.value = 6;
  NHActionServer_->setAborted(nhresult_, "Navigation aborted because the node has been killed");
}



void Indires_macro_actions::explorationCB(const indires_macro_actions::ExplorationGoal::ConstPtr& goal)
{
  printf("¡¡¡¡¡¡¡MacroAction Exploration  -->  started!!!!!!\n");
  // printf("Goal  x: %.2f, y: %.2f frame: %s\n", goal->target_pose.pose.position.x,
  // goal->target_pose.pose.position.y, goal->target_pose.header.frame_id.c_str());
  // UpoNav_->stopRRTPlanning();

  move_base_msgs::MoveBaseGoal g;
  g.target_pose = goal->empty;
  g.target_pose.header.frame_id = "";
  g.target_pose.pose.orientation.x = 0.0;
  g.target_pose.pose.orientation.y = 0.0;
  g.target_pose.pose.orientation.z = 0.0;
  g.target_pose.pose.orientation.w = 1.0;
  moveBaseClient_->sendGoal(g);

  // moveBaseClient_->waitForResult();

  actionlib::SimpleClientGoalState state = moveBaseClient_->getState();


  // moveBaseClient_->cancelAllGoals()
  // moveBaseClient_->cancelGoal()
  // moveBaseClient_->getResult()


  if (moveBaseClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Success!!!");
  } /*else {
    ROS_INFO("Failed!");
  }*/

  /*bool ok = UpoNav_->executeNavigation(goal->target_pose);
  if(!ok)
  {
    ROS_INFO("Setting ABORTED state 1");
    nwresult_.result = "Aborted. Navigation error";
    nwresult_.value = 2;
    NWActionServer_->setAborted(nwresult_, "Navigation aborted");
    //UpoNav_->stopRRTPlanning();
    return;
  }*/

  ros::Rate r(control_frequency_);
  // int pursue_status = 0;
  bool exit = false;
  ros::Time time_init = ros::Time::now();
  bool first = true;
  nav_msgs::Odometry pose_init;
  // ros::WallTime startt;
  while (nh3_.ok())
  {
    // startt = ros::WallTime::now();

    if (ExActionServer_->isPreemptRequested())
    {
      if (ExActionServer_->isNewGoalAvailable())
      {
        indires_macro_actions::ExplorationGoal new_goal = *ExActionServer_->acceptNewGoal();

        g.target_pose = new_goal.empty;
        moveBaseClient_->sendGoal(g);


        /*bool ok = UpoNav_->executeNavigation(new_goal.target_pose);
        if(!ok) {
    ROS_INFO("Setting ABORTED state 1");
    nwresult_.result = "Aborted. Navigation error";
    nwresult_.value = 2;
    NWActionServer_->setAborted(nwresult_, "Navigation aborted");
    UpoNav_->stopRRTPlanning();
    if(use_leds_)
      setLedColor(WHITE);
    return;
  }*/
        first = true;
      }
      else
      {
        // if we've been preempted explicitly we need to shut things down
        // UpoNav_->resetState();

        // Cancel?????

        // notify the ActionServer that we've successfully preempted
        exresult_.result = "Preempted";
        exresult_.value = 1;
        ROS_DEBUG_NAMED("indires_macro_actions", "indires_exploration preempting the current goal");
        ExActionServer_->setPreempted(exresult_, "Exploration preempted");
        // we'll actually return from execute after preempting
        return;
      }
    }

    pose_mutex_.lock();
    nav_msgs::Odometry new_pose = odom_pose_;
    pose_mutex_.unlock();

    // pursue_status = UpoNav_->pathFollow(new_pose);

    // Posible states:
    // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    state = moveBaseClient_->getState();


    if (first)
    {
      pose_init = new_pose;
      time_init = ros::Time::now();
      first = false;
    }



    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // Goal reached
      // WE MUST TO CONTINUE THE EXPLORATION
      ROS_INFO("Goal reached. Exploration continues...");
      // exresult_.result = "Exploration succeeded";
      // exresult_.value = 0;
      // ExActionServer_->setSucceeded(exresult_, "Goal Reached");
      // exfeedback_.text = "Succeeded";
      // exit = true;
      exfeedback_.text = "goal reached. Exploration continues";
      moveBaseClient_->sendGoal(g);
    }
    else if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      // Goal not reached, continue navigating
      exfeedback_.text = "Navigating";
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      // Aborted
      exfeedback_.text = "Aborted";
      ROS_INFO("Setting ABORTED state");
      exresult_.result = "Aborted";
      exresult_.value = 3;
      ExActionServer_->setAborted(exresult_, "Exploration aborted");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      exfeedback_.text = "Pending";
      // ROS_INFO("Setting ABORTED state");
      // nwresult_.result = "";
      // nwresult_.value = 3;
      // NWActionServer_->setAborted(nwresult_, "Navigation aborted");
      // exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::RECALLED)
    {
      exfeedback_.text = "Recalled";
    }
    else if (state == actionlib::SimpleClientGoalState::REJECTED)
    {
      // Rejected
      exfeedback_.text = "Rejected";
      ROS_INFO("Setting ABORTED (rejected) state");
      exresult_.result = "Rejected";
      exresult_.value = 3;
      ExActionServer_->setAborted(exresult_, "Exploration aborted(rejected)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::LOST)
    {
      // Rejected
      exfeedback_.text = "Lost";
      ROS_INFO("Setting ABORTED (lost) state");
      exresult_.result = "Lost";
      exresult_.value = 3;
      ExActionServer_->setAborted(exresult_, "Exploration aborted(lost)");
      exit = true;
    }
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      nwfeedback_.text = "Preempted";
    }


    // push the feedback out
    geometry_msgs::PoseStamped aux;
    aux.header = new_pose.header;
    aux.pose.position = new_pose.pose.pose.position;
    aux.pose.orientation = new_pose.pose.pose.orientation;
    exfeedback_.base_position = aux;
    ExActionServer_->publishFeedback(exfeedback_);

    if (exit)
    {
      // UpoNav_->stopRRTPlanning();
      return;
    }


    // check the blocked situation.
    double time = (ros::Time::now() - time_init).toSec();
    // printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(),
    // time_init.toSec(), time);
    if (time > secs_to_check_block_)
    {
      double xinit = pose_init.pose.pose.position.x;
      double yinit = pose_init.pose.pose.position.y;
      double hinit = tf::getYaw(pose_init.pose.pose.orientation);
      double xnow = new_pose.pose.pose.position.x;
      double ynow = new_pose.pose.pose.position.y;
      double hnow = tf::getYaw(new_pose.pose.pose.orientation);
      double dist = sqrt(pow((xinit - xnow), 2) + pow((yinit - ynow), 2));
      double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
      // printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
      if (dist <= block_dist_ && yaw_diff < 0.79)
      {  // 0.79 = 45º
        ROS_INFO("Setting ABORTED state because of blocked situation");
        exresult_.result = "Aborted. Blocked situation";
        exresult_.value = 5;
        ExActionServer_->setAborted(exresult_, "Exploration aborted. blocked");
        exfeedback_.text = "Blocked";
        ExActionServer_->publishFeedback(exfeedback_);
        // UpoNav_->stopRRTPlanning();
        return;
      }
      else
      {
        pose_init = new_pose;
        time_init = ros::Time::now();
      }
    }

    // ros::WallDuration dur = ros::WallTime::now() - startt;
    // printf("Loop time: %.4f secs\n", dur.toSec());

    r.sleep();
  }

  ROS_INFO("Setting ABORTED state");
  exresult_.result = "Aborted. System is shuting down";
  exresult_.value = 6;
  ExActionServer_->setAborted(exresult_, "Exploration aborted because the node has been killed");
}



/*
bool Indires_macro_actions::reconfigureParameters(std::string node, std::string
param_name, std::string value, const datatype type)
{
  //printf("RECONFIGURE PARAMETERS METHOD\n");
  dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter param1;
    dynamic_reconfigure::BoolParameter param2;
    dynamic_reconfigure::DoubleParameter param3;
    dynamic_reconfigure::StrParameter param4;
    dynamic_reconfigure::Config conf;

    switch(type)
    {
    case INT_TYPE:
      param1.name = param_name.c_str();
      param1.value = stoi(value);
      conf.ints.push_back(param1);
      break;

    case DOUBLE_TYPE:
      param3.name = param_name.c_str();
      //printf("type double. Value: %s\n", param3.name.c_str());
      param3.value = stod(value);
      //printf("conversion to double: %.3f\n", param3.value);
      conf.doubles.push_back(param3);
      break;

    case BOOL_TYPE:
      param2.name = param_name.c_str();
      param2.value = stoi(value);
      conf.bools.push_back(param2);
      break;

    case STRING_TYPE:
      param4.name = param_name.c_str();
      param4.value = value;
      conf.strs.push_back(param4);
      break;

    default:
      ROS_ERROR("indires_macro_actions. ReconfigureParameters. datatype not valid!");
  }
    srv_req.config = conf;

    std::string service = node + "/set_parameters";
    if (!ros::service::call(service, srv_req, srv_resp)) {
      ROS_ERROR("Could not call the service %s reconfigure the param %s to %s",
service.c_str(), param_name.c_str(), value.c_str());
      return false;
    }
    return true;
}
*/



void Indires_macro_actions::teleoperationCB(const indires_macro_actions::TeleoperationGoal::ConstPtr& goal)
{
  if (!manual_control_)
  {
    printf("¡¡¡¡¡¡¡MacroAction AssistedSteering  -->  started!!!!!!\n");
    manual_control_ = true;

    moveBaseClient_->cancelAllGoals();

    // UpoNav_->stopRRTPlanning();
    // stop the current wsbs if it is running
    // teresa_wsbs::stop stop_srv;
    // stop_client_.call(stop_srv);
  }


  ros::Time time_init;
  time_init = ros::Time::now();
  bool exit = false;


  // boost::recursive_mutex::scoped_lock l(configuration_mutex_);


  ros::Rate r(30.0);
  while (nh4_.ok())
  {
    if (TOActionServer_->isPreemptRequested())
    {
      if (TOActionServer_->isNewGoalAvailable())
      {
        // if we're active and a new goal is available, we'll accept it, but we won't shut
        // anything down
        // ROS_INFO("Accepting new goal");
        indires_macro_actions::TeleoperationGoal new_goal = *TOActionServer_->acceptNewGoal();
        time_init = ros::Time::now();
      }
      else
      {
        TOActionServer_->setPreempted(toresult_, "Teleoperation preempted");
        return;
      }
    }

    tofeedback_.text = "Robot manually controlled";


    // Check the time without receiving commands from the interface
    double time = (ros::Time::now() - time_init).toSec();
    if (time > 5.0)
    {
      tofeedback_.text = "Teleoperation finished";
      toresult_.result = "Teleoperation Succeeded";
      toresult_.value = 0;
      TOActionServer_->setSucceeded(toresult_, "Teleoperation succeeded");
      exit = true;
    }

    TOActionServer_->publishFeedback(tofeedback_);

    if (exit)
    {
      manual_control_ = false;
      return;
    }

    r.sleep();
  }
  manual_control_ = false;
  ROS_INFO("Setting ABORTED state");
  TOActionServer_->setAborted(toresult_, "Teleoperation aborted because the node has been killed");
}



// void Upo_navigation_macro_actions::poseCallback(const
// geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
void Indires_macro_actions::robotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_mutex_.lock();
  odom_pose_ = *msg;
  // robot_global_pose_.y = msg->pose.pose.position.y;
  // robot_global_pose_.theta = tf::getYaw(msg->pose.pose.orientation);
  pose_mutex_.unlock();
}



void Indires_macro_actions::rrtGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  /*geometry_msgs::PoseStamped out;
  out = transformPoseTo(*msg, "map");
  geometry_msgs::Pose2D p;
  p.x = out.pose.position.x;
  p.y = out.pose.position.y;
  p.theta = 0.0;
  */

  goal_mutex_.lock();
  rrtgoal_ = *msg;
  goal_mutex_.unlock();
}



geometry_msgs::PoseStamped Indires_macro_actions::transformPoseTo(geometry_msgs::PoseStamped pose_in,
                                                                  std::string frame_out)
{
  geometry_msgs::PoseStamped in = pose_in;
  in.header.stamp = ros::Time();
  geometry_msgs::PoseStamped pose_out;
  try
  {
    pose_out = tf_->transform(in, frame_out.c_str());
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("Macro-Action class. TransformException in method transformPoseTo: %s", ex.what());
    pose_out.header = in.header;
    pose_out.header.stamp = ros::Time::now();
    pose_out.pose.position.x = 0.0;
    pose_out.pose.position.y = 0.0;
    pose_out.pose.position.z = 0.0;
    pose_out.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }

  return pose_out;
}


// This method removes the initial slash from the frame names
// in order to compare the string names easily
void Indires_macro_actions::fixFrame(std::string& cad)
{
  if (cad[0] == '/')
  {
    cad.erase(0, 1);
  }
}


float Indires_macro_actions::normalizeAngle(float val, float min, float max)
{
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max - min));
  else
    norm = max - fmod((min - val), (max - min));

  return norm;
}
