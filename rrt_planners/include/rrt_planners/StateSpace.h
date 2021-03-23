
#ifndef RRT_STATE_SPACE_
#define RRT_STATE_SPACE_


#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <rrt_planners/StateChecker.h>
#include <rrt_planners/RandomNumbers.h>

// For pre C++ 11 gamma function
#include <boost/math/special_functions/gamma.hpp>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <mutex>


namespace RRT
{
class StateSpace
{
public:
  StateSpace();

  StateSpace(StateChecker* stateChecker, unsigned int dim, unsigned int dim_type,
             float sx, float sy, float sz = 0.0, float xyzres = 0.1, float yawres = 0.02,
             float min_lv = 0.1, float max_lv = 0.5, float lvres = 0.05,
             float max_av = 0.5, float avres = 0.1);

  ~StateSpace();

  State* sampleState();
  State* sampleStateFree();
  State* sampleStateNear(State* st);
  // State* sampleStateNearFree(State* st);

  State* sampleStateExternal();

  State* sampleStateExternal(std::vector<State>* space);

  void setExternalSamples(std::vector<State>* space);


  // State* samplePathBiasing(std::vector<State>* path, float stddev, float yawdev = 0.2);

  float sampleUniform();  // value between [0, 1]

  float distance(State* s1, State* s2);
  float euclideanDistance(State* s1, State* s2);

  bool isStateValid(State* s);

  float getCost(State* s);

  bool getValid3dState(State* s);

  Node exploreLeafStates(std::vector<Node>& leaves);

  bool isSimpleGoalToleranceSatisfied(State* st, float& dist);

  bool isGoalToleranceSatisfied(State* st, float& dist);

  float getSpaceMeasure();
  float calculeUnitBallMeasure(unsigned int d, double r);
  float getUnitBallMeasure();


  float normalizeAngle(float val, float min, float max);


  unsigned int getDimensions();
  unsigned int getType();
  std::vector<unsigned int> getWeights();
  float getSizeX();
  float getSizeY();
  float getSizeZ();
  float getXYZresolution();
  float getRPYResolution();
  float getMinLinVel();
  float getMaxLinVel();
  float getLinVelResolution();
  float getMaxAngVel();
  float getAngVelResolution();
  float getGoalXYZTolerance();
  float getGoalTHTolerance();
  State* getStart();
  State* getGoal();

  void setDimensions(unsigned int d);
  void setWeights(std::vector<unsigned int> w);
  void setSizeX(float sx);
  void setSizeY(float sy);
  void setSizeZ(float sz);
  void setXYZresolution(float res);
  void setRPYResolution(float res);
  void setMinLinVel(float v);
  void setMaxLinVel(float v);
  void setLinVelResolution(float res);
  void setMaxAngVel(float v);
  void setAngVelResolution(float res);
  void setGoalTolerance(float xyz_tol, float th_tol);
  bool setStart(State* s);
  bool setGoal(State* g);
  bool setStartAndGoal(State* s, State* g);


  StateChecker* stateChecker_;



private:

  std::vector<State> external_samples_;
  std::mutex ext_mutex_;
  std::vector<int> used_indices_;
  bool use_external_samples_;

  RNG random_;

  unsigned int dimensions_;

  // if dimensions is 3, type 1-> X,Y,th  type 2->X,Y,Z
  unsigned int type_;

  std::vector<unsigned int> weights_;

  float size_x_;
  float size_y_;
  float size_z_;
  float xyz_resolution_;
  float rpy_resolution_;

  float space_volume_;
  float unit_ball_measure_;

  float min_lin_vel_;
  float max_lin_vel_;
  float lin_vel_res_;
  std::vector<float> lin_vels_;

  float max_ang_vel_;
  float ang_vel_res_;
  std::vector<float> ang_vels_;

  float goal_xyz_tolerance_;
  float goal_th_tolerance_;

  State* start_;
  State* goal_;
};
}
#endif
