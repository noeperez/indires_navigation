#include <rrt_planners/State.h>

// Constructor
RRT::State::State()
{
  x_ = 0.0;
  y_ = 0.0;
  z_ = 0.0;
  yaw_ = 0.0;
  roll_ = 0.0;
  pitch_ = 0.0;
  lin_vel_ = 0.0;
  ang_vel_ = 0.0;
}

// Constructor
RRT::State::State(float x, float y, float z, float yaw, float roll, float pitch, float lv, float av)
{
  x_ = x;
  y_ = y;
  z_ = z;
  yaw_ = yaw;
  roll_ = roll;
  pitch_ = pitch;
  lin_vel_ = lv;
  ang_vel_ = av;
}

// Destructor
RRT::State::~State()
{
}


void RRT::State::getState(float &x, float &y, float &z, float &yaw, float &roll,
                          float &pitch, float &lv, float &av)
{
  x = x_;
  y = y_;
  z = z_;
  yaw = yaw_;
  roll = roll_;
  pitch = pitch_;
  lv = lin_vel_;
  av = ang_vel_;
}

float RRT::State::getX() const
{
  return x_;
}
float RRT::State::getY() const
{
  return y_;
}
float RRT::State::getZ() const
{
  return z_;
}
float RRT::State::getYaw() const
{
  return yaw_;
}
float RRT::State::getRoll() const
{
  return roll_;
}
float RRT::State::getPitch() const
{
  return pitch_;
}
float RRT::State::getLinVel() const
{
  return lin_vel_;
}
float RRT::State::getAngVel() const
{
  return ang_vel_;
}

void RRT::State::setX(float x)
{
  x_ = x;
}
void RRT::State::setY(float y)
{
  y_ = y;
}
void RRT::State::setZ(float z)
{
  z_ = z;
}
void RRT::State::setYaw(float yaw)
{
  yaw_ = yaw;
}
void RRT::State::setRoll(float roll)
{
  roll_ = roll;
}
void RRT::State::setPitch(float pitch)
{
  pitch_ = pitch;
}
void RRT::State::setLv(float lv)
{
  lin_vel_ = lv;
}
void RRT::State::setAv(float av)
{
  ang_vel_ = av;
}
