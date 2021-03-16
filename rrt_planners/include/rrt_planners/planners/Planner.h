#ifndef RRT_PLANNER_
#define RRT_PLANNER_

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <rrt_planners/steering/Steering.h>
#include <rrt_planners/StateChecker.h>
//#include <rrt_planners/NearestNeighborsFLANN.h>
//#include <rrt_planners/NearestNeighbors.h>
#include <rrt_planners/kdtree.h>

#include <vector>
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <memory> //shared_ptr


namespace RRT
{
class Planner
{
public:
  Planner();


  virtual ~Planner();


  virtual std::vector<RRT::Node> solve(float secs)
  {
    std::vector<RRT::Node> empty;
    return empty;
  }

  virtual RRT::State* steer(State* fromState, State* toState)
  {
    return NULL;
  }

  virtual RRT::Node* steer(Node* fromState, Node* toState)
  {
    return NULL;
  }


  void setup(StateChecker* sch, unsigned int nn_params, unsigned int dim,
             unsigned int dim_type, float sx, float sy, float sz = 0.0, float xyzres = 0.1,
             float yawres = 0.02, float min_lv = 0.0, float max_lv = 0.5,
             float lvres = 0.05, float max_av = 0.5, float avres = 0.1, float steer_kp = 0.5,
             float steer_kv = 3.0, float steer_ka = 2.0, float steer_ko = 0.25);


  void inline setSamplingSpace(std::vector<RRT::State>* sp)
  {
    // external_sampling_space_ = true;
    space_->setExternalSamples(sp);
  }

  /*void set_external_samples(bool b) {
    external_sampling_space_ = b;
  }*/


  template <class T>
  T* as()
  {
    // BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<T*>(this);
  }

  template <class T>
  const T* as() const
  {
    // BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<const T*>(this);
  }



  float distanceFunction(Node* s1, Node* s2)
  {
    return space_->distance(s1->getState(), s2->getState());
  }


  void setGoalBias(float b)
  {
    goalBias_ = b;
  }

  void setGoalTolerance(float xyz_tol, float th_tol)
  {
    space_->setGoalTolerance(xyz_tol, th_tol);
  }

  bool setStartAndGoal(float start_x, float start_y, float start_z, float start_h,
                       float goal_x, float goal_y, float goal_z, float goal_h);

  bool setStart(float start_x, float start_y, float start_z, float start_h);

  bool setGoal(float goal_x, float goal_y, float goal_z, float goal_h);


  void setStoreTree(bool s)
  {
    storeTree_ = s;
  }

  void setExploration(bool e)
  {
    exploration_ = e;
  }

  struct statistics
  {
    float planning_time;
    float first_sol_time;
    unsigned int total_samples;
    unsigned int valid_samples;
    unsigned int goal_samples;
    unsigned int tree_nodes;
    unsigned int leaf_nodes;
    unsigned int path_nodes;
  };

  statistics getStatistics()
  {
    return stats_;
  }


  std::vector<RRT::State> getTree()
  {
    return tree_;
  }

  std::vector<RRT::Node> getLeaves()
  {
    return leaves_;
  }

  float getCost()
  {
    return path_cost_;
  }



  void copyState(State* destination, const State* source) const
  {
    memcpy(destination, source, sizeof(*source));
  }
  void copyAction(Action* destination, const Action* source) const
  {
    memcpy(destination, source, sizeof(*source));
  }
  void copyNode(Node* destination, const Node* source) const
  {
    memcpy(destination, source, sizeof(*source));
  }

  // void freeTreeMemory();

  /*void setNearestNeighbors(unsigned int nn_params)
  {
    nn_.reset(new NearestNeighborsFLANN<RRT::Node*>(nn_params));
  }*/

  void storeTree(std::vector<Node> list);

  void storeLeafNodes(std::vector<Node> list);

  // void setBiasingPath(std::vector<RRT::State>* path);

  /*void setFullBiasing(bool b) {
    fullBiasing_ = b;
  }*/

  /*void setPathBias(float f) {
    pathBias_ = f;
  }*/

  /*void setPathBias_stddev(float f) {
    pathBias_stddev_ = f;
  }*/

  void setInitialActionState(float vx, float vy, float vth, int steps)
  {
    init_action_state_ = new Action(vx, vy, vth, steps);
  }

  /*void inline set_gmm_sampling(bool gmm_sampling, float gmm_bias, std::vector<
  std::pair<float,float> > samples) {
    gmm_sampling_ = gmm_sampling;
    gmm_bias_ = gmm_bias;
    gmm_samples_ = samples;
  }*/


protected:
  float goalBias_;

  int dimensions_;

  StateSpace* space_;

  // bool 				external_sampling_space_;

  // boost::shared_ptr< NearestNeighborsFLANN<RRT::Node*> > nn_;

  Steering* steering_;

  State* start_;
  State* goal_;

  Action* init_action_state_;

  float path_cost_;

  statistics stats_;

  bool storeTree_;
  std::vector<RRT::State> tree_;

  bool storeLeaves_;
  std::vector<RRT::Node> leaves_;


  bool exploration_;
  // std::vector<RRT::States> leaves_states_;

  // Used to bias the sampling to points nearby
  // std::vector<RRT::State> first_path_;
  // bool 				fullBiasing_;
  // float				pathBias_;
  // float 			pathBias_stddev_;

  // GMM sampling
  // bool				gmm_sampling_;
  // float 				gmm_bias_;
  // std::queue< std::pair<float,float> > gmm_samples_;
  // std::vector< std::pair<float,float> > gmm_samples_;
};
}
#endif
