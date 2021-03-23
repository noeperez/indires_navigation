#ifndef RRT_SIMPLE_RRT_
#define RRT_SIMPLE_RRT_

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <rrt_planners/StateChecker.h>
//#include <rrt_planners/NearestNeighborsFLANN.h>
//#include <rrt_planners/NearestNeighbors.h>
#include <rrt_planners/planners/Planner.h>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>


namespace RRT
{
class SimpleRRT : public RRT::Planner
{
public:
  SimpleRRT();
  ~SimpleRRT();

  State* steer(State* fromState, State* toState, std::vector<State>& istates);

  std::vector<RRT::Node> solve(float secs);

  // void setRobotPosition();


  void setMaxRange(float range)
  {
    steering_->setMaxRange(range);
  }


  // private:

  // float 				maxRange_; //max distance to insert a new node
};
}
#endif
