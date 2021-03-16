#include <rrt_planners/StateSpace.h>
//#include <ros/ros.h>

#define PI 3.14159

RRT::StateSpace::StateSpace()
{
  dimensions_ = 3;
  type_ = 2;
  weights_.push_back(1.0);  // x
  weights_.push_back(1.0);  // y
  weights_.push_back(0.3);  // th

  size_x_ = 4.0;  //[-4,4]
  size_y_ = 4.0;
  size_z_ = 4.0;
  xyz_resolution_ = 0.1;
  rpy_resolution_ = 0.02;

  min_lin_vel_ = 0.1;
  max_lin_vel_ = 0.5;
  lin_vel_res_ = 0.1;

  max_ang_vel_ = 0.5;
  ang_vel_res_ = 0.25;

  goal_xyz_tolerance_ = 0.1;
  goal_th_tolerance_ = 0.15;

  space_volume_ = (size_x_ - (-size_x_)) * (size_y_ - (-size_y_)) * (size_z_ - (-size_z_));
  unit_ball_measure_ = calculeUnitBallMeasure(dimensions_, 1.0);
}


RRT::StateSpace::StateSpace(StateChecker* stateChecker, unsigned int dim, unsigned int dim_type,
                            float sx, float sy, float sz, float xyzres, float yawres,
                            float min_lv, float max_lv, float lvres, float max_av, float avres)
  : dimensions_(dim)
  , type_(dim_type)
  , size_x_(sx)
  , size_y_(sy)
  , size_z_(sz)
  , xyz_resolution_(xyzres)
  , rpy_resolution_(yawres)
  , min_lin_vel_(min_lv)
  , max_lin_vel_(max_lv)
  , lin_vel_res_(lvres)
  , max_ang_vel_(max_av)
  , ang_vel_res_(avres)
{
  stateChecker_ = stateChecker;

  switch (dimensions_)
  {
    case 2:
      weights_.push_back(1.0);  // x
      weights_.push_back(1.0);  // y
      break;
    case 3:
      if (type_ == 1)
      {
        weights_.push_back(1.0);  // x
        weights_.push_back(1.0);  // y
        weights_.push_back(0.3);  // th
      }
      else
      {
        weights_.push_back(1.0);  // x
        weights_.push_back(1.0);  // y
        weights_.push_back(1.0);  // z
      }
      break;
    case 4:
      weights_.push_back(1.0);  // x
      weights_.push_back(1.0);  // y
      weights_.push_back(1.0);  // z
      weights_.push_back(0.3);  // th

    case 5:
      weights_.push_back(1.0);  // x
      weights_.push_back(1.0);  // y
      weights_.push_back(0.4);  // th
      weights_.push_back(0.1);  // lv
      weights_.push_back(0.1);  // av
      break;
    default:
      printf("Number of dimensions not valid");
  }

  if (dimensions_ == 2 || (dimensions_ == 3 && type_ == 1))
    space_volume_ = (size_x_ - (-size_x_)) * (size_y_ - (-size_y_));
  else
    space_volume_ = (size_x_ - (-size_x_)) * (size_y_ - (-size_y_)) * (size_z_ - (-size_z_));

  goal_xyz_tolerance_ = 0.1;
  goal_th_tolerance_ = 0.15;

  unit_ball_measure_ = calculeUnitBallMeasure(dimensions_, 1.0);
}

RRT::StateSpace::~StateSpace()
{
  // if(stateChecker_)
  //	delete stateChecker_;
  if (start_)
    delete start_;
  if (goal_)
    delete goal_;
}

float RRT::StateSpace::calculeUnitBallMeasure(unsigned int d, double r)
{
  return std::pow(std::sqrt(boost::math::constants::pi<double>()) * r, static_cast<double>(d)) /
         boost::math::tgamma(static_cast<double>(d) / 2.0 + 1.0);
}

float RRT::StateSpace::getUnitBallMeasure()
{
  return unit_ball_measure_;
}

RRT::State* RRT::StateSpace::sampleState()
{
  if (!start_)
  {
    return NULL;
  }

  if (use_external_samples_)
    return sampleStateExternal();

  float x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, lv = 0.0, av = 0.0;
  x = random_.uniformReal(start_->getX() - size_x_, start_->getX() + size_x_);
  y = random_.uniformReal(start_->getY() - size_y_, start_->getY() + size_y_);
  if (dimensions_ == 3)
  {
    if (type_ == 1)
      yaw = random_.uniformReal(-PI, PI);
    else
      z = random_.uniformReal(start_->getZ() - size_z_, start_->getZ() + size_z_);
  }
  if (dimensions_ > 3)
  {
    z = random_.uniformReal(start_->getZ() - size_z_, start_->getZ() + size_z_);
    lv = random_.uniformReal(min_lin_vel_, max_lin_vel_);
    av = random_.uniformReal(-max_ang_vel_, max_ang_vel_);
  }
  State* sample = new RRT::State(x, y, z, yaw, 0.0, 0.0, lv, av);
  return sample;
}


RRT::State* RRT::StateSpace::sampleStateFree()
{
  if (use_external_samples_)
    return sampleStateExternal();

  float x, y, z = 0.0;
  State* sample;
  do
  {
    x = random_.uniformReal(start_->getX() - size_x_, start_->getX() + size_x_);
    y = random_.uniformReal(start_->getY() - size_y_, start_->getY() + size_y_);
    z = random_.uniformReal(start_->getZ() - size_z_, start_->getZ() + size_z_);
    sample = new RRT::State(x, y, z);
  } while (!isStateValid(sample));
  if (dimensions_ == 2)
    return sample;

  if (dimensions_ == 3 && type_ == 1)
  {
    float yaw = random_.uniformReal(-PI, PI);
    sample->setYaw(yaw);
    return sample;
  }
  float yaw = random_.uniformReal(-PI, PI);
  sample->setYaw(yaw);
  float lv = random_.uniformReal(min_lin_vel_, max_lin_vel_);
  float av = random_.uniformReal(-max_ang_vel_, max_ang_vel_);
  sample->setLv(lv);
  sample->setAv(av);
  return sample;
}

float RRT::StateSpace::sampleUniform()
{
  return (float)random_.uniform01();
}

RRT::State* RRT::StateSpace::sampleStateNear(State* st)
{
  float low_x = st->getX() - 1.0;
  if (low_x < -size_x_)
    low_x = -size_x_;
  float high_x = st->getX() + 1.0;
  if (high_x > size_x_)
    high_x = size_x_;
  float x = random_.uniformReal(low_x, high_x);

  float low_y = st->getY() - 1.0;
  if (low_y < -size_y_)
    low_y = -size_y_;
  float high_y = st->getY() + 1.0;
  if (high_y > size_y_)
    high_y = size_y_;
  float y = random_.uniformReal(low_y, high_y);

  float low_z = st->getZ() - 1.0;
  if (low_z < -size_z_)
    low_z = -size_z_;
  float high_z = st->getZ() + 1.0;
  if (high_z > size_z_)
    high_z = size_z_;
  float z = random_.uniformReal(low_z, high_z);

  float low_h = st->getYaw() - 0.35;  // 0.35rad = 20º
  if (low_h < -PI)
    low_h = -PI;
  float high_h = st->getYaw() + 0.35;
  if (high_h > PI)
    high_h = PI;
  float yaw = random_.uniformReal(low_h, high_h);

  float lv = 0.0;
  float av = 0.0;
  if (dimensions_ > 3)
  {
    float low_lv = st->getLinVel() - 0.1;
    if (low_lv < 0.0)
      low_lv = 0.0;
    float high_lv = st->getLinVel() + 0.1;
    if (high_lv > max_lin_vel_)
      high_lv = max_lin_vel_;
    lv = random_.uniformReal(low_lv, high_lv);

    float low_av = st->getAngVel() - 0.1;
    if (low_av < -max_ang_vel_)
      low_av = -max_ang_vel_;
    float high_av = st->getAngVel() + 0.1;
    if (high_av > max_ang_vel_)
      high_av = max_ang_vel_;
    av = random_.uniformReal(low_av, high_av);
  }

  State* sample = new State(x, y, z, yaw, 0.0, 0.0, lv, av);
  return sample;
}


/*
RRT::State* RRT::StateSpace::samplePathBiasing(std::vector<State>* path, float stddev,
float yawdev) {

  int ind = random_.uniformInt(1, int(path->size()-1));
  float x = path->at(ind).getX();
  float y = path->at(ind).getY();
  float x_sample = random_.gaussian(x, stddev);
  float y_sample = random_.gaussian(y, stddev);

  if(dimensions_ > 2) //we also have to sample the yaw according to the path
  {
    float z = path->at(ind).getZ();
    float yaw = path->at(ind).getYaw();
    float z_sample = random_.gaussian(z, stddev);
    float yaw_sample = random_.uniformReal(yaw-yawdev, yaw+yawdev);
    yaw_sample = normalizeAngle(yaw_sample, -M_PI, M_PI);
    return new State(x_sample, y_sample, z_sample, yaw_sample);
  }

  return new State(x_sample, y_sample);
}*/



RRT::State* RRT::StateSpace::sampleStateExternal()
{
  ext_mutex_.lock();
  if (!external_samples_.empty())
  {
    int ind = 0;
    bool ok;
    int cont = 0;
    do
    {
      ok = true;
      ind = random_.uniformInt(0, int(external_samples_.size() - 1));
      for (unsigned int i = 0; i < used_indices_.size(); i++)
      {
        if (ind == used_indices_[i])
        {
          cont++;
          if (cont == external_samples_.size())
            used_indices_.clear();
          else
            ok = false;
        }
      }
      // printf("StateSpace. sampled index %i. ok: %i, total: %u\n", ind, (int)ok,
      // (unsigned int)external_samples_.size());

    } while (!ok);
    used_indices_.push_back(ind);

    // printf("StateSpace. SampleStateExternal. index %i sampled from %u posibilities\n",
    // ind, (unsigned int)external_samples_.size());

    RRT::State* st = &(external_samples_[ind]);
    // printf("StateSpace. sampleStateExternal. state z: %.3f\n", st->getZ());
    ext_mutex_.unlock();
    return st;
  }
  else
  {
    printf("StateSpace. External samples is empty!!!!!");
    ext_mutex_.unlock();
    return new State();
  }
}



RRT::State* RRT::StateSpace::sampleStateExternal(std::vector<RRT::State>* space)
{
  if (!space->empty())
  {
    int ind = random_.uniformInt(0, int(space->size() - 1));
    RRT::State* st = &(space->at(ind));
    return st;
  }
  else
  {
    printf("StateSpace. External samples is empty!");
    return new State();
  }
}


void RRT::StateSpace::setExternalSamples(std::vector<RRT::State>* space)
{
  // if(space->empty)
  //	printf("StateSpace. setExternalSamples. Samples is empty!");
  ext_mutex_.lock();
  use_external_samples_ = true;
  used_indices_.clear();
  external_samples_.clear();
  external_samples_ = *space;
  ext_mutex_.unlock();
  printf("StateSpace. SetExternalSamples, size: %u\n", (unsigned int)external_samples_.size());
}



RRT::Node RRT::StateSpace::exploreLeafStates(std::vector<RRT::Node>& leaves)
{
  std::vector<RRT::Node> cluster = stateChecker_->clusterize_leaves(leaves);

  RRT::Node goal = stateChecker_->evaluate_exploration(cluster);

  return goal;
}



float RRT::StateSpace::distance(State* s1, State* s2)
{
  return stateChecker_->distance(s1, s2);
  // return euclideanDistance(s1, s2);
}

float RRT::StateSpace::euclideanDistance(State* s1, State* s2)
{
  float dx = s1->getX() - s2->getX();
  float dy = s1->getY() - s2->getY();
  float dz = s1->getZ() - s2->getZ();
  return sqrt(dx * dx + dy * dy + dz * dz);
}

bool RRT::StateSpace::isSimpleGoalToleranceSatisfied(State* st, float& dist)
{
  // float dx = goal->getX() - st->getX();
  // float dy = goal->getY() - st->getY();
  // dist = sqrt(pow(dx,2) + pow(dy,2));
  // printf("StateSpace. px: %.2f, py: %.2f, gx: %.2f, gy:%.2f\n", st->getX(), st->getY(),
  // goal_->getX(), goal_->getY());
  dist = euclideanDistance(goal_, st);
  if (dist <= goal_xyz_tolerance_)
    return true;

  return false;
}

bool RRT::StateSpace::isGoalToleranceSatisfied(State* st, float& dist)
{
  dist = euclideanDistance(goal_, st);
  if (dimensions_ > 2)
  {
    float phi = st->getYaw() - goal_->getYaw();
    // Normalize phi
    phi = normalizeAngle(phi, -M_PI, M_PI);
    if (dist <= goal_xyz_tolerance_ && fabs(phi) <= goal_th_tolerance_)
      return true;
  }
  else
  {
    if (dist <= goal_xyz_tolerance_)
      return true;
  }
  return false;
}


float RRT::StateSpace::normalizeAngle(float val, float min, float max)
{
  float norm = 0.0;
  if (val >= min)
    norm = min + fmod((val - min), (max - min));
  else
    norm = max - fmod((min - val), (max - min));

  return norm;
}

float RRT::StateSpace::getSpaceMeasure()
{
  return space_volume_;
}


bool RRT::StateSpace::setStart(State* s)
{
  if (s == NULL)
    return false;
  else
    start_ = s;
  return true;

  // if(!isStateValid(s))
  //	return false;
  // start_ = s;
  // return true;
}

bool RRT::StateSpace::setGoal(State* g)
{
  if (!isStateValid(g))
    return false;
  goal_ = g;
  return true;
}

bool RRT::StateSpace::setStartAndGoal(State* s, State* g)
{
  if (s == NULL || g == NULL)
    return false;

  // if(!isStateValid(g))
  //	return false;

  start_ = s;
  goal_ = g;
  return true;
}


bool RRT::StateSpace::isStateValid(RRT::State* s)
{
  return stateChecker_->isValid(s);
}


float RRT::StateSpace::getCost(RRT::State* s)
{
  return stateChecker_->getCost(s);
}


bool RRT::StateSpace::getValid3dState(State* s)
{
  return stateChecker_->getValid3dState(s);
}



unsigned int RRT::StateSpace::getDimensions()
{
  return dimensions_;
}
std::vector<unsigned int> RRT::StateSpace::getWeights()
{
  return weights_;
}
float RRT::StateSpace::getSizeX()
{
  return size_x_;
}
float RRT::StateSpace::getSizeY()
{
  return size_y_;
}
float RRT::StateSpace::getXYZresolution()
{
  return xyz_resolution_;
}
float RRT::StateSpace::getRPYResolution()
{
  return rpy_resolution_;
}
float RRT::StateSpace::getMinLinVel()
{
  return min_lin_vel_;
}
float RRT::StateSpace::getMaxLinVel()
{
  return max_lin_vel_;
}
float RRT::StateSpace::getLinVelResolution()
{
  return lin_vel_res_;
}
float RRT::StateSpace::getMaxAngVel()
{
  return max_ang_vel_;
}
float RRT::StateSpace::getAngVelResolution()
{
  return ang_vel_res_;
}
float RRT::StateSpace::getGoalXYZTolerance()
{
  return goal_xyz_tolerance_;
}
float RRT::StateSpace::getGoalTHTolerance()
{
  return goal_th_tolerance_;
}
RRT::State* RRT::StateSpace::getStart()
{
  return start_;
}
RRT::State* RRT::StateSpace::getGoal()
{
  return goal_;
}


void RRT::StateSpace::setDimensions(unsigned int d)
{
  dimensions_ = d;
}
void RRT::StateSpace::setWeights(std::vector<unsigned int> w)
{
  weights_.clear();
  for (unsigned int i = 0; i < w.size(); i++)
    weights_.push_back(w[i]);
}
void RRT::StateSpace::setSizeX(float sx)
{
  size_x_ = sx;
}
void RRT::StateSpace::setSizeY(float sy)
{
  size_y_ = sy;
}
void RRT::StateSpace::setXYZresolution(float res)
{
  xyz_resolution_ = res;
}
void RRT::StateSpace::setRPYResolution(float res)
{
  rpy_resolution_ = res;
}
void RRT::StateSpace::setMinLinVel(float v)
{
  min_lin_vel_ = v;
}
void RRT::StateSpace::setMaxLinVel(float v)
{
  max_lin_vel_ = v;
}
void RRT::StateSpace::setLinVelResolution(float res)
{
  lin_vel_res_ = res;
}
void RRT::StateSpace::setMaxAngVel(float v)
{
  max_ang_vel_ = v;
}
void RRT::StateSpace::setAngVelResolution(float res)
{
  ang_vel_res_ = res;
}
void RRT::StateSpace::setGoalTolerance(float xy_tol, float th_tol)
{
  goal_xyz_tolerance_ = xy_tol;
  goal_th_tolerance_ = th_tol;
}
