#ifndef RRT_SIMPLE_QUICK_RRT_STAR_
#define RRT_SIMPLE_QUICK_RRT_STAR_

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <rrt_planners/StateChecker.h>
#include <rrt_planners/planners/Planner.h>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

namespace RRT
{
class SimpleQuickRRTstar : public RRT::Planner
{
public:
  SimpleQuickRRTstar();
  ~SimpleQuickRRTstar();

  State* steer(State* fromState, State* toState, std::vector<State>& istates);

  std::vector<RRT::Node> solve(float secs);


  void setMaxRange(float range)
  {
    maxRange_ = range;
    steering_->setMaxRange(range);
  }


  void setDepth(int d)
  {
    depth_ = d;
  }


  void set_useKnearest(bool b)
  {
    useKnearest_ = b;
  }

  void setRewireFactor(float f)
  {
    rewire_factor_ = f;
  }

  void set_useFirstPathBiasing(bool b)
  {
    useFirstPathBiasing_ = b;
  }



  // void getNearestNeighbors(Node* node, std::vector<Node*> &nbrs);
  void getNearestNeighbors(KDTree<RRT::Node>& nn, const Node& node,
                           std::vector<Node>& nbrs);

  void getAncestors(std::vector<Node>& nbrs, int depth);

  void calculateParamsNearest();

  // float motionCost(Node* n1, Node* n2);

  bool collisionFree(State* fromState, State* toState, std::vector<State>& istates);



private:
  // float 				timeStep_; //should be 1/freq  with freq = freq of the
  // controller(15~20Hz)
  // float 				minControlSteps_; //minTime = timeStep*minControlDuration
  // flaot 				maxControlSteps_;

  float maxRange_;  // max distance to insert a new node

  int depth_;

  bool useKnearest_;
  double k_rrt_;
  double r_rrt_;
  float rewire_factor_;

  bool useFirstPathBiasing_;
};
}
#endif
