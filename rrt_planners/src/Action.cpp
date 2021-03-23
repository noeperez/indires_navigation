#include <rrt_planners/Action.h>

// Constructor
RRT::Action::Action()
{
  vx_ = 0.0;
  vy_ = 0.0;
  vth_ = 0.0;
  steps_ = 1;
}

// Constructor
RRT::Action::Action(float vx, float vy, float vth, unsigned int steps)
{
  vx_ = vx;
  vy_ = vy;
  vth_ = vth;
  steps_ = steps;
}

// Destructor
RRT::Action::~Action()
{
}


void RRT::Action::getAction(float &vx, float &vy, float &vth, unsigned int &steps)
{
  vx = vx_;
  vy = vy_;
  vth = vth_;
  steps = steps_;
}


float RRT::Action::getVx()
{
  return vx_;
}
float RRT::Action::getVy()
{
  return vy_;
}
float RRT::Action::getVth()
{
  return vth_;
}
unsigned int RRT::Action::getSteps()
{
  return steps_;
}
