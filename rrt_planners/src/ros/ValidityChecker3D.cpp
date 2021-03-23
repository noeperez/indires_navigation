
#include <rrt_planners/ros/ValidityChecker3D.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>


RRT_ros::ValidityChecker3D::ValidityChecker3D(tf2_ros::Buffer* tf,
                                              std::vector<geometry_msgs::Point>* footprint,
                                              float insc_radius, float size_x, float size_y,
                                              float size_z, float res, unsigned int dimensions,
                                              int distType, std::string planning_frame)
  : StateChecker()
{
  // get_cost_from_costmap_ = use_fc_costmap;

  // if(!get_cost_from_costmap_) {
  // printf("Initialization of nav_features\n");
  // navfeatures_ = new features::NavFeatures(tf, footprint, insc_radius, size_x, size_y,
  // res);
  /*}else {
    printf("----Using cost function to build a costmap-----\n");
    loc_costmap_ = loc_costmap;
    glo_costmap_ = glob_costmap;
    tf_ = tf;
  }*/

  ros::NodeHandle nh("~/RRT_ros_wrapper");
  rrt_planner_type_ = 2;
  nh.getParam("rrt_planner_type", rrt_planner_type_);

  // nfe_exploration_ = false;
  // nh.getParam("nfe_exploration", nfe_exploration_);

  std::string features_name("navigation_features_3d");
  nh.getParam("features_name", features_name);

  features_ = new nav3d::Features3D(features_name, tf, footprint, size_x, size_y, size_z);


  dimensions_ = dimensions;
  distanceType_ = distType;
  time_ = ros::Time::now();

  planning_frame_ = planning_frame;
  // robot_odom_frame_ = robot_odom_frame;

  ros::NodeHandle n;
  // explore_pub_ = n.advertise<visualization_msgs::Marker>("rrt_explore_goals", 1);
  explore_pub_ = n.advertise<visualization_msgs::MarkerArray>("rrt_explore_goals", 1);



  // srand (time(NULL));
}


RRT_ros::ValidityChecker3D::~ValidityChecker3D()
{
  delete features_;
}


bool RRT_ros::ValidityChecker3D::isValid(RRT::State* s) const
{
  geometry_msgs::PoseStamped p_in;
  p_in.header.frame_id = planning_frame_;
  // p_in.header.stamp = ros::Time(0); //this is a problem when the planning time is long.
  // the time stamp should be the time when the rrt started to plan.
  if ((ros::Time::now() - time_).toSec() > 2.0)
  {
    // time_ = ros::Time::now();
    p_in.header.stamp = ros::Time(0);
  }
  else
    p_in.header.stamp = time_;

  p_in.pose.position.x = s->getX();
  p_in.pose.position.y = s->getY();
  p_in.pose.position.z = s->getZ();
  p_in.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());


  return features_->poseValid(&p_in);
}


bool RRT_ros::ValidityChecker3D::getValid3dState(RRT::State* s) const
{
  geometry_msgs::PoseStamped p_in;
  p_in.header.frame_id = planning_frame_;
  // p_in.header.stamp = ros::Time(0); //this is a problem when the planning time is long.
  // the time stamp should be the time when the rrt started to plan.
  if ((ros::Time::now() - time_).toSec() > 2.0)
  {
    // time_ = ros::Time::now();
    p_in.header.stamp = ros::Time(0);
  }
  else
    p_in.header.stamp = time_;

  p_in.pose.position.x = s->getX();
  p_in.pose.position.y = s->getY();
  p_in.pose.position.z = s->getZ();
  p_in.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());


  bool ok = features_->pose3dValid(&p_in);

  if (ok)  // we need to update the height z of the State s from p_in
    s->setZ(p_in.pose.position.z);

  return ok;
}



/*
void RRT_ros::ValidityChecker3D::preplanning_computations()
{
  //if(!get_cost_from_costmap_)
  features_->update();
}*/


float RRT_ros::ValidityChecker3D::distance(RRT::State* s1, RRT::State* s2) const
{
  float dx = s1->getX() - s2->getX();
  float dy = s1->getY() - s2->getY();
  float dz = s1->getZ() - s2->getZ();
  // float dist = sqrt(dx*dx + dy*dy);
  float dist = dx * dx + dy * dy + dz * dz;

  switch (distanceType_)
  {
    case 1:
      return dist;

    case 2:
      return sqrt(dist);

    case 3:
      if (dimensions_ == 2)
        return sqrt(dist);
      else
      {
        // SUM w1*|| Pi+1 - Pi|| + w2*(1-|Qi+1 * Qi|)²
        float euc_dist = sqrt(dist);

        tf::Quaternion q1 = tf::createQuaternionFromYaw(s1->getYaw());
        tf::Quaternion q2 = tf::createQuaternionFromYaw(s2->getYaw());
        float dot_prod = q1.dot(q2);
        float angle_dist = (1 - fabs(dot_prod)) * (1 - fabs(dot_prod));
        // printf("eu_dist: %.2f, angle_dist: %.3f, dist: %.3f\n", euc_dist, angle_dist,
        // 0.8*euc_dist + 0.2*angle_dist);
        return 0.7 * euc_dist + 0.3 * angle_dist;
      }

    case 4:
      if (dimensions_ == 2)
        return sqrt(dist);
      else
      {
        // Another option
        /*
        First, transform the robot location into person location frame:
                      |cos(th)  sin(th)  0|
          Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
                      |  0        0      1|

          x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
          y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
        */
        float x = (s2->getX() - s1->getX()) * cos(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * sin(s1->getYaw());
        float y = -(s2->getX() - s1->getX()) * sin(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * cos(s1->getYaw());
        float alpha = atan2(y, x);
        return (0.8 * sqrt(dist) + 0.2 * fabs(alpha));
      }

    case 5:
      if (dimensions_ == 2)
        return sqrt(dist);
      else
      {
        // UPO. Dist + sum of the angles of both points regarding the intersection line
        float x = (s2->getX() - s1->getX()) * cos(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * sin(s1->getYaw());
        float y = -(s2->getX() - s1->getX()) * sin(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * cos(s1->getYaw());
        float alpha = atan2(y, x);
        float beta = s2->getYaw() - alpha;
        beta = features_->normalizeAngle(beta, -M_PI, M_PI);
        return (0.6 * sqrt(dist) + 0.4 * (fabs(alpha) + fabs(beta)));
      }

    case 6:
      if (dimensions_ == 2)
        return sqrt(dist);
      else
      {
        // Paper IROS2015 "Feedback motion planning via non-holonomic RRT* for mobile
        // robots"
        float x = (s2->getX() - s1->getX()) * cos(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * sin(s1->getYaw());
        float y = -(s2->getX() - s1->getX()) * sin(s1->getYaw()) +
                  (s2->getY() - s1->getY()) * cos(s1->getYaw());
        float alpha = atan2(y, x);
        float phi = s2->getYaw() - alpha;
        phi = features_->normalizeAngle(phi, -M_PI, M_PI);
        float ka = 0.5;
        float ko = ka / 8.0;
        dist = sqrt(dist);
        // two options
        float alpha_prime = atan(-ko * phi);
        // float alpha_prime = atan(-ko*ko * phi/(dist*dist));
        float r = features_->normalizeAngle((alpha - alpha_prime), -M_PI, M_PI);
        return (sqrt(dist * dist + ko * ko + phi * phi) + ka * fabs(r));
      }

    default:
      return sqrt(dist);
  }
}



std::vector<RRT::Node> RRT_ros::ValidityChecker3D::clusterize_leaves(std::vector<RRT::Node>& leaves) const
{
  float radius = 0.3;
  // printf("ValidityChecker3D. clusterize leaves called!\n");
  // std::vector<RRT::Node*> clu_pointers; // = *leaves;
  std::vector<RRT::Node> clusters;
  std::vector<geometry_msgs::Point> points;
  if (leaves.empty())
  {
    printf("ValidityChecker3D. leaves is empty!!!\n");
    return clusters;
  }

  for (unsigned int i = 0; i < leaves.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = leaves[i](0);  //.getState()->getX();
    p.y = leaves[i](1);  //.getState()->getY();
    p.z = leaves[i](2);  //.getState()->getZ();
    points.push_back(p);
  }

  std::vector<std::vector<int> > clusters_ind = features_->clusterize_leaves(&points, radius);
  if (clusters_ind.empty())
  {
    printf("ValidityChecker3D. clusters is empty!!\n");
    /*visualization_msgs::Marker l;
    l.header.frame_id = robot_base_frame_;
    l.header.stamp = ros::Time();
    l.ns = "rrt_exploration_goals";
    l.id = 3;
    l.type = visualization_msgs::Marker::SPHERE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    //l.pose.position.x = 0.0;
    //l.pose.position.y = 0.0;
    //l.pose.position.z = 0.1;
    l.scale.x = 0.15;
    l.scale.y = 0.15;
    l.color.r = 1.0f;
    l.color.g = 1.0f;
    l.color.b = 0.0f;
    l.color.a = 1.0;
    l.lifetime = ros::Duration();

    for(unsigned int i=0; i<leaves->size(); i++)
    {
      RRT::State* s = leaves->at(i)->getState();
      geometry_msgs::Point p;
      p.x = s->getX();
      p.y = s->getY();
      p.z = s->getZ();
      l.points.push_back(p);
    }
    explore_pub_.publish(l);
    */
    return leaves;
  }

  printf("\tleaves size: %u\n", (unsigned int)leaves.size());
  std::vector<int> invalid;

  for (unsigned int i = 0; i < clusters_ind.size(); i++)
  {
    float min_cost = 10.0;
    int ind = 0;
    for (unsigned int j = 0; j < clusters_ind[i].size(); j++)
    {
      int index = clusters_ind[i][j];
      float c = leaves[index].getAccCost();
      if (c < min_cost)
      {
        min_cost = c;
        ind = index;
      }
    }
    // printf("ValidityChecker3D. Cluster %u. Taking leaf %i from %u leaves\n", i, ind,
    // (unsigned int)clusters_ind[i].size());
    for (unsigned int k = 0; k < clusters_ind[i].size(); k++)
    {
      if (clusters_ind[i][k] != ind)
      {
        invalid.push_back(clusters_ind[i][k]);
        // poor efficiency. I should use a list or other structure
        // printf("ValidityChecker3D. Removing index %i from cluster %i...\n",
        // clusters_ind[i][k], i);

        // AQUÍ PETA PORQUE CADA VEZ QUE QUITAMOS UN ELEMENTO YA CAMBIA EL INDICE PARA LOS
        // DEMAS!!!!
        // clu_pointers.erase(clu_pointers.begin() + clusters_ind[i][k]);
      }
    }
    // clu_pointers.push_back(leaves->at(clusters_ind[i][ind]));
  }

  for (unsigned int i = 0; i < leaves.size(); i++)
  {
    bool insert = true;
    for (unsigned int j = 0; j < invalid.size(); j++)
    {
      if (i == invalid[j])
        insert = false;
    }
    if (insert)
      clusters.push_back(leaves[i]);
  }


  // printf("\tclu_points size after removing: %u\n", (unsigned int)clu_pointers.size());

  /*
  visualization_msgs::MarkerArray ma;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,1.0);
  for(unsigned int i=0; i<clusters_ind.size(); i++)
  {
    visualization_msgs::Marker l;
    l.header.frame_id = robot_base_frame_;
    l.header.stamp = ros::Time();
    l.ns = "rrt_exploration_goals";
    l.id = i+100;
    l.type = visualization_msgs::Marker::SPHERE_LIST;
    l.action = visualization_msgs::Marker::ADD;
    //l.pose.position.x = 0.0;
    //l.pose.position.y = 0.0;
    //l.pose.position.z = 0.1;
    l.scale.x = 0.15;
    l.scale.y = 0.15;
    l.color.r = 1.0f - distribution(generator);  //(i+1.0)/clusters_ind.size();
    l.color.g = 1.0f - distribution(generator);  //(i+1.0)/clusters_ind.size();
    l.color.b = 0.0f + distribution(generator);  //(i+1.0)/clusters_ind.size();
  //1.0/(i+1.0);
    l.color.a = 1.0;
    l.lifetime = ros::Duration();

    for(unsigned int j=0; j<clusters_ind[i].size(); j++)
    {
      printf("Adding point %u of cluster %u, value: %i\n", j, i, clusters_ind[i][j]);
      RRT::State* s = leaves->at(clusters_ind[i][j])->getState();
      geometry_msgs::Point p;
      p.x = s->getX();
      p.y = s->getY();
      p.z = s->getZ();
      l.points.push_back(p);
    }
    ma.markers.push_back(l);
  }
  explore_pub_.publish(ma);
  */

  /*
  visualization_msgs::Marker l;
  l.header.frame_id = robot_base_frame_;
  l.header.stamp = ros::Time();
  l.ns = "rrt_exploration_goals";
  l.id = 3;
  l.type = visualization_msgs::Marker::SPHERE_LIST;
  l.action = visualization_msgs::Marker::ADD;
  //l.pose.position.x = 0.0;
  //l.pose.position.y = 0.0;
  //l.pose.position.z = 0.1;
  l.scale.x = 0.15;
  l.scale.y = 0.15;
  l.color.r = 1.0f;
  l.color.g = 1.0f;
  l.color.b = 0.0f;
  l.color.a = 1.0;
  l.lifetime = ros::Duration();

  for(unsigned int i=0; i<clu_pointers.size(); i++)
  {
    RRT::State* s = clu_pointers[i]->getState();
    geometry_msgs::Point p;
    p.x = s->getX();
    p.y = s->getY();
    p.z = s->getZ();
    l.points.push_back(p);
  }
  explore_pub_.publish(l);
  */

  return clusters;
}



RRT::Node RRT_ros::ValidityChecker3D::evaluate_exploration(std::vector<RRT::Node>& leaves) const
{
  float radius = 1.5;
  // printf("ValidityChecker3D. evaluate_exploration called!\n");
  std::vector<geometry_msgs::Point> points;
  std::vector<float> costs;
  for (unsigned int i = 0; i < leaves.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = leaves[i](0);  //.getState()->getX();
    p.y = leaves[i](1);  //.getState()->getY();
    p.z = leaves[i](2);  //.getState()->getZ();
    points.push_back(p);
    if (rrt_planner_type_ == 1 || rrt_planner_type_ == 4)
      costs.push_back(-1.0);
    else
      costs.push_back(leaves[i].getAccCost());
  }
  // std::vector<float> gain_info = features_->evaluate_leaves(&points, planning_frame_,
  // radius);
  int ind_goal = features_->evaluate_leaves(&points, &costs, planning_frame_, radius);

  /*float min_cost = 100.0;
  int ind_goal;
  float exploration_cost;
  for(unsigned int i=0; i<leaves->size(); i++)
  {

    float ig_cost = gain_info[i];
    if(rrt_planner_type_ == 1 || rrt_planner_type_ == 3 || nfe_exploration_) //RRT (no
  star)
    {
      exploration_cost = ig_cost;
    }else {
      float rrt_cost = leaves->at(i)->getAccCost();
      exploration_cost = 0.2*rrt_cost + ig_cost;
    }
    if(exploration_cost < min_cost)
    {
      min_cost = exploration_cost;
      ind_goal = i;
    }
  }*/

  return leaves[ind_goal];
}



/*
RRT::Node* RRT_ros::ValidityChecker3D::evaluate_exploration(std::vector<RRT::Node*>*
leaves) const
{

  float radius = 2.0;
  printf("ValidityChecker3D. evaluate_exploration called!\n");
  std::vector<geometry_msgs::Point> points;
  for(unsigned int i=0; i<leaves->size(); i++)
  {
    geometry_msgs::Point p;
    p.x = leaves->at(i)->getState()->getX();
    p.y = leaves->at(i)->getState()->getY();
    p.z = leaves->at(i)->getState()->getZ();
    points.push_back(p);
  }

  float min_cost = 1000.0;
  int ind_goal;
  float exploration_cost;
  for(unsigned int i=0; i<leaves->size(); i++)
  {
    if(rrt_planner_type_ == 1 || rrt_planner_type_ == 4 || nfe_exploration_) //RRT (no
star)
    {
      exploration_cost = features_->evaluate_leaf(&points[i], -1.0, planning_frame_,
radius);
    }else {
      exploration_cost = features_->evaluate_leaf(&points[i], leaves->at(i)->getAccCost(),
planning_frame_, radius);
    }
    if(exploration_cost < min_cost)
    {
      min_cost = exploration_cost;
      ind_goal = i;
    }
  }

  return leaves->at(ind_goal);

}*/



float RRT_ros::ValidityChecker3D::getCost(RRT::State* s)
{
  // if(get_cost_from_costmap_) {
  //	printf("\nERROR!! ValidityChecker. getCost from costmap is not available!!!!\n");
  //}

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = planning_frame_;
  if ((ros::Time::now() - time_).toSec() > 2.0)
    time_ = ros::Time::now();
  pose.header.stamp = time_;
  // pose.header.stamp = ros::Time(0);
  pose.pose.position.x = s->getX();
  pose.pose.position.y = s->getY();
  pose.pose.position.z = s->getZ();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
  // printf("ValidityChecker. x: %.2f, y:%.2f, th: %.2f\n", pose.pose.position.x,
  // pose.pose.position.y, s->getYaw());
  float cost = features_->getCost(&pose);
  return cost;
}



std::vector<float> RRT_ros::ValidityChecker3D::getFeatures(RRT::State* s)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = planning_frame_;
  pose.header.stamp = ros::Time(0);
  pose.pose.position.x = s->getX();
  pose.pose.position.y = s->getY();
  pose.pose.position.z = s->getZ();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());

  std::vector<float> features = features_->getFeatures(&pose);

  return features;
}



/*void upo_RRT_ros::ValidityChecker3D::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
  navfeatures_->setPeople(p);
}*/


void RRT_ros::ValidityChecker3D::setWeights(std::vector<float> we)
{
  features_->setWeights(we);
}


geometry_msgs::PoseStamped RRT_ros::ValidityChecker3D::transformPoseTo(
    geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime)
{
  return features_->transformPoseTo(pose_in, frame_out, usetime);
}

bool RRT_ros::ValidityChecker3D::isQuaternionValid(const geometry_msgs::Quaternion q)
{
  return features_->isQuaternionValid(q);
}
