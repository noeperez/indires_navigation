
#include <local_3d_planner/collision_detection.h>


namespace local_3d_planner
{
CollisionDetection::CollisionDetection(std::string name, tf2_ros::Buffer* tf,
                                       local_3d_planner::OdometryHelperRos* oh,
                                       double max_lv, double max_av, double lin_acc,
                                       double ang_acc, double sim_t, double r_radius,
                                       double local_radius, double granularity)
{
  tf_ = tf;
  odom_helper_ = NULL;
  if (oh)
    odom_helper_ = oh;

  max_lin_vel_ = max_lv;
  max_ang_vel_ = max_av;
  max_lin_acc_ = lin_acc;
  max_ang_acc_ = ang_acc;
  sim_time_ = sim_t;
  robot_radius_ = r_radius;
  robot_radius_aug_ = robot_radius_;
  local_radius_ = local_radius;
  granularity_ = granularity;
  use_laser_ = false;
  use_range_ = false;
  num_ranges_ = 0;
  ranges_ready_ = false;

  setup(name);
}


CollisionDetection::~CollisionDetection()
{
  // delete odom_helper_;
}


void CollisionDetection::setup(std::string name)
{
  // ros::NodeHandle n("~");
  ros::NodeHandle n("~/" + name);

  n.param<std::string>("odometry_topic", odom_topic_, std::string("odom"));
  n.param<std::string>("base_frame", base_frame_, std::string("base_link"));


  std::string features_name;
  n.param<std::string>("features_name", features_name, std::string("nav_features_3d"));
  // printf("CollisionDetection. Features_name: %s\n", features_name.c_str());


  double uncertainty = 0.0;
  n.getParam("sensor_uncertainty", uncertainty);
  robot_radius_aug_ = robot_radius_ + uncertainty;

  n.getParam("use_laser", use_laser_);
  std::string laser_topic;
  ros::NodeHandle nh;
  if (use_laser_)
  {
    n.param<std::string>("laser_topic", laser_topic, std::string("scan"));
    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
        laser_topic.c_str(), 1, &CollisionDetection::laserCallback, this);
  }

  unsigned int i = 0;
  n.param<bool>("use_range", use_range_, false);
  if (use_range_)
  {
    bool ok = true;
    while (ok)
    {
      char buf[25];
      sprintf(buf, "range_topic_%u", i);
      string st = string(buf);

      if (n.hasParam(st.c_str()))
      {
        std::string rt;
        n.getParam(st.c_str(), rt);
        range_topics_.push_back(rt);
        // ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>(rt.c_str(), 10,
        // &CollisionDetection::rangeCallback, this);
        ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>(
            rt.c_str(), 1, boost::bind(&CollisionDetection::rangeCallback, this, _1));
        range_subscribers_.push_back(sub);
        printf("%s. subscribed to topic: %s\n", name.c_str(), rt.c_str());
        i++;
      }
      else
        ok = false;
    }
    num_ranges_ = (int)i;
    ranges_initiated_.assign(num_ranges_, false);
    // range_frames_.assign(num_ranges_, "");
  }



  max_lv_var_ = max_lin_acc_ * sim_time_;
  max_av_var_ = max_ang_acc_ * sim_time_;


  features_ =
      new nav3d::Features3D(features_name, tf_, local_radius_, local_radius_, local_radius_);


  if (odom_helper_ == NULL)
    odom_helper_ = new OdometryHelperRos(odom_topic_);

  // Dynamic reconfigure
  // dsrv_ = new
  // dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>(n);
  // //ros::NodeHandle("~")
  // dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>::CallbackType
  // cb = boost::bind(&AssistedSteering::reconfigureCB, this, _1, _2);
  // dsrv_->setCallback(cb);
}


/*void CollisionDetection::reconfigureCB(assisted_steering::AssistedSteeringConfig
&config, uint32_t level){

    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  max_lin_vel_ = config.max_lin_vel;
  max_ang_vel_ = config.max_ang_vel;
  max_lin_acc_ = config.max_lin_acc;
  max_ang_acc_ = config.max_ang_acc;
  time_step_ = config.time_step;
  robot_radius_ = config.robot_radius;
  granularity_ = config.granularity;
  isActive_ = config.is_active;
  ang_vel_inc_ = config.ang_vel_inc;
  lin_vel_inc_ = config.lin_vel_inc;

  max_lv_var_ = max_lin_acc_ * time_step_;
  max_av_var_ = max_ang_acc_ * time_step_;

}*/



void CollisionDetection::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_ONCE("Collision detector: Laser received!");
  // IMPORTANT: the frame of the laser should be the center of the robot (base_link)
  // Otherwise we should include a shift to the center in the calculations.
  laser_mutex_.lock();
  laser_scan_ = *msg;
  // scan1_time_ = ros::Time::now();
  laser_mutex_.unlock();
}


void CollisionDetection::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO_ONCE("Collision detector: range received! Detecting configuration...");
  if (!ranges_ready_)
  {
    if (range_frames_.size() != (unsigned int)num_ranges_)
    {
      bool found = false;
      for (unsigned int i = 0; i < range_frames_.size(); i++)
      {
        if (range_frames_[i].compare(msg->header.frame_id) == 0)
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        range_frames_.push_back(msg->header.frame_id);

        range r;
        r.range = msg->range;
        r.id = msg->header.frame_id;
        r.min_dist = msg->min_range;
        r.max_dist = msg->max_range;
        r.fov = msg->field_of_view;
        // listen to the TF to know the position of the sonar range
        // tf::StampedTransform transform;
        geometry_msgs::TransformStamped ts;
        try
        {
          // tf_->waitForTransform(base_frame_.c_str(), r.id.c_str(), ros::Time(0),
          //                      ros::Duration(3.0));
          // tf_->lookupTransform(base_frame_.c_str(), r.id.c_str(), ros::Time(0),
          // transform);
          ts = tf_->lookupTransform(base_frame_.c_str(), r.id.c_str(), ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
        float x = ts.transform.translation.x;
        float y = ts.transform.translation.y;
        r.polar_dist = sqrt(x * x + y * y);
        // tf::Matrix3x3 m(ts.transform.rotation);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        r.polar_angle = tf::getYaw(ts.transform.rotation);

        range_mutex_.lock();
        ranges_.push_back(r);
        range_mutex_.unlock();
        printf("Collision detector: Range %s configured.\n", msg->header.frame_id.c_str());
      }
    }
    else
    {
      ranges_ready_ = true;
      printf("Collision detector: all the range sensors configured!!!\n\n");
    }
  }
  else
  {
    range_mutex_.lock();
    for (int i = 0; i < (int)ranges_.size(); i++)
    {
      if (ranges_[i].id.compare(msg->header.frame_id) == 0)
      {
        ranges_[i].range = msg->range;
        break;
      }
    }
    range_mutex_.unlock();
  }
}



void CollisionDetection::saturateVelocities(geometry_msgs::Twist* twist)
{
  float lv = twist->linear.x;
  float av = twist->angular.z;

  float rvx = robot_vel_.getOrigin().getX();
  float rvy = robot_vel_.getOrigin().getY();
  float rvt = tf::getYaw(robot_vel_.getRotation());

  // acc linear
  if (fabs(rvx - lv) > max_lv_var_)
  {
    lv = (lv < rvx) ? (rvx - max_lv_var_) : (rvx + max_lv_var_);
  }
  // acc angular
  if (fabs(rvt - av) > max_av_var_)
  {
    av = (av < rvt) ? (rvt - max_av_var_) : (rvt + max_av_var_);
  }

  // Check maximum velocities
  if (lv > max_lin_vel_)
    lv = max_lin_vel_;
  else if (lv < (-max_lin_vel_))
    lv = max_lin_vel_ * (-1);

  if (av > max_ang_vel_)
    av = max_ang_vel_;
  else if (av < (-max_ang_vel_))
    av = max_ang_vel_ * (-1);

  twist->linear.x = lv;
  twist->angular.z = av;
}



bool CollisionDetection::inCollision(float x, float y, std::vector<geometry_msgs::Point>* scanpoints)
{
  if (use_range_ && inRangeCollision(x, y))
  {
    printf("Possible collision detected!");
    return true;
  }
  if (use_laser_ && inLaserCollision(x, y, scanpoints))
  {
    return true;
  }
  return false;
}



bool CollisionDetection::inRangeCollision(float x, float y)
{
  range_mutex_.lock();
  for (unsigned int i = 0; i < ranges_.size(); i++)
  {
    // Main point
    float rx = (ranges_[i].polar_dist + ranges_[i].range) * cos(ranges_[i].polar_angle);
    float ry = (ranges_[i].polar_dist + ranges_[i].range) * sin(ranges_[i].polar_angle);
    float dx = (x - rx);
    float dy = (y - ry);
    float dist = dx * dx + dy * dy;
    if (dist <= (robot_radius_aug_ * robot_radius_aug_))
    {
      ROS_INFO("POSSIBLE COLLISION DETECTED IN FRAME: %s", ranges_[i].id.c_str());
      range_mutex_.unlock();
      return true;
    }

    if (ranges_[i].fov > 0.2)
    {
      // second point
      rx = (ranges_[i].polar_dist + ranges_[i].range) *
           cos(ranges_[i].polar_angle + (ranges_[i].fov / 2.0));
      ry = (ranges_[i].polar_dist + ranges_[i].range) *
           sin(ranges_[i].polar_angle + (ranges_[i].fov / 2.0));
      dx = (x - rx);
      dy = (y - ry);
      dist = dx * dx + dy * dy;
      if (dist <= (robot_radius_aug_ * robot_radius_aug_))
      {
        ROS_INFO("POSSIBLE COLLISION DETECTED (p2) IN FRAME: %s", ranges_[i].id.c_str());
        range_mutex_.unlock();
        return true;
      }

      // third point
      rx = (ranges_[i].polar_dist + ranges_[i].range) *
           cos(ranges_[i].polar_angle - (ranges_[i].fov / 2.0));
      ry = (ranges_[i].polar_dist + ranges_[i].range) *
           sin(ranges_[i].polar_angle - (ranges_[i].fov / 2.0));
      dx = (x - rx);
      dy = (y - ry);
      dist = dx * dx + dy * dy;
      if (dist <= (robot_radius_aug_ * robot_radius_aug_))
      {
        ROS_INFO("POSSIBLE COLLISION DETECTED (p3) IN FRAME: %s", ranges_[i].id.c_str());
        range_mutex_.unlock();
        return true;
      }
    }
  }
  range_mutex_.unlock();
  return false;
}



std::vector<geometry_msgs::Point> CollisionDetection::laser_polar2euclidean(sensor_msgs::LaserScan* scan)
{
  std::vector<geometry_msgs::Point> points;
  points.reserve(scan->ranges.size());
  geometry_msgs::Point p;
  p.z = 0.0;
  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    // laser measure polar coordinates
    float laser_d = scan->ranges[i];
    float laser_th = scan->angle_min + scan->angle_increment * i;
    // transform to x,y
    p.x = laser_d * cos(laser_th);
    p.y = laser_d * sin(laser_th);
    points.push_back(p);
  }
  return points;
}



bool CollisionDetection::inLaserCollision(float x, float y,
                                          std::vector<geometry_msgs::Point>* scanpoints)
{
  for (unsigned int i = 0; i < scanpoints->size(); i++)
  {
    float dx = (x - scanpoints->at(i).x);
    float dy = (y - scanpoints->at(i).y);
    // float dist = sqrt(dx*dx + dy*dy);
    // if(dist <= robot_radius_)
    float dist = dx * dx + dy * dy;
    if (dist <= (robot_radius_aug_ * robot_radius_aug_))
      return true;
  }
  return false;
}



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
bool CollisionDetection::checkTraj(double cvx, double cvy, double cvth, double tvx,
                                   double tvy, double tvth, double& px, double& py,
                                   double& pz, double& pth)
{
  // boost::recursive_mutex::scoped_lock l(configuration_mutex_);


  px = 0.0;
  py = 0.0;
  pz = 0.0;
  pth = 0.0;

  // printf("\nCollisionDetection. CheckTraj with vels vx:%.2f, vy:%.2f th:%.2f\n", tvx,
  // tvy, tvth);

  double max_lv = computeNewVelocity(tvx, cvx, max_lin_acc_, sim_time_);
  float vel_mag = sqrt(max_lv * max_lv);
  float steps = (vel_mag * sim_time_) / granularity_;
  float dt = sim_time_ / steps;
  float x = 0.0, y = 0.0, z = 0.0, th = 0.0;


  double lv = cvx;
  double av = cvth;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time();                        // ros::Time::now();
  pose.header.frame_id = features_->getRobotBaseFrame();  // base_frame_;


  std::vector<geometry_msgs::Point> laser_points;
  if (use_laser_)
  {
    laser_mutex_.lock();
    sensor_msgs::LaserScan l = laser_scan_;
    laser_mutex_.unlock();
    laser_points = laser_polar2euclidean(&l);
  }

  // int ini = floor(steps/2.0 + 0.5);
  for (unsigned int i = 0; i < steps; i++)
  {
    lv = computeNewVelocity(tvx, lv, max_lin_acc_, dt);
    av = computeNewVelocity(tvth, av, max_ang_acc_, dt);

    float lin_dist = lv * dt;
    th = th + (av * dt);
    // normalization just in case
    th = normalizeAngle(th, -M_PI, M_PI);
    x = x + lin_dist * cos(th);  // cos(th+av*dt/2.0)
    y = y + lin_dist * sin(th);

    if (inCollision(x, y, &laser_points))
      return false;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

    // printf("CollisionDetection. CheckTraj. step %u - x:%.2f, y:%.2f, z:%.2f frame: %s
    // -", i, x, y, z, pose.header.frame_id.c_str());

    // tomar el punto x,y,z=0 (inicialmente) y coger los vecinos en el radio.
    // Coger el valor de z del mean
    bool valid = features_->pose3dValid(&pose);
    // pose3dValid transforma el pose a odom frame, pero después lo vuelve a base_link
    // para coger el z correcto
    z = pose.pose.position.z;

    if (!valid)
    {
      // printf("NOT VALID\n");
      return false;
    }
    // printf("VALID\n");
  }

  px = x;
  py = y;
  pz = z;
  pth = th;

  return true;
}



} /* namespace collision detection */
