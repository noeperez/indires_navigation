#include <rrt_planners/planners/Planner.h>


RRT::Planner::Planner()
{
  goalBias_ = 0.05;
  storeTree_ = false;
  storeLeaves_ = false;
  exploration_ = false;
  space_ = NULL;
  steering_ = new Steering();
  start_ = NULL;
  goal_ = NULL;
  path_cost_ = 0.0;
  dimensions_ = 2;
  // fullBiasing_ = false;
  // pathBias_ = 0.0;
  // pathBias_stddev_ = 0.0;
}

RRT::Planner::~Planner()
{
  // freeTreeMemory();
  tree_.clear();
  leaves_.clear();
  // nn_.reset();
  delete steering_;
  delete space_;
  delete start_;
  delete goal_;
}

/*
void RRT::Planner::freeTreeMemory() {
  if (nn_)
  {
    std::vector<Node*> nodes;
    nn_->list(nodes);
    for (unsigned int i = 0; i <nodes.size() ; ++i)
    {
      if(nodes[i]) {
        delete nodes[i];
      }
    }
    nodes.clear();
    nn_->clear();
    //if(goal_)
      //delete goal_;
    //if(start_)
      //delete start_;
  }
}*/


void RRT::Planner::setup(StateChecker* sch, unsigned int nn_params, unsigned int dim,
                         unsigned int dim_type, float sx, float sy, float sz,
                         float xyzres, float yawres, float min_lv, float max_lv,
                         float lvres, float max_av, float avres, float steer_kp,
                         float steer_kv, float steer_ka, float steer_ko)
{
  // if(!nn_) {
  //	setNearestNeighbors(nn_params);
  //}
  // nn_->setDistanceFunction(boost::bind(&Planner::distanceFunction, this, _1, _2));
  dimensions_ = dim;

  // dimensions, size_x, size_y, xyres, yawres, min_lv, max_lv, lvres, max_av, avres
  space_ = new StateSpace(sch, dim, dim_type, sx, sy, sz, xyzres, yawres, min_lv, max_lv,
                          lvres, max_av, avres);

  if (steering_ == NULL)
    steering_ = new Steering(space_);
  else
    steering_->setStateSpace(space_);

  steering_->setSteeringParams(steer_kp, steer_kv, steer_ka, steer_ko);
}


bool RRT::Planner::setStartAndGoal(float start_x, float start_y, float start_z, float start_h,
                                   float goal_x, float goal_y, float goal_z, float goal_h)
{
  start_ = new State(start_x, start_y, start_z, start_h);
  goal_ = new State(goal_x, goal_y, goal_z, goal_h);
  // printf("START x:%.1f, y:%.1f, h:%.1f  GOAL x:%.2f, y:%.2f, h:%.2f\n", start_x,
  // start_y, start_h, goal_x, goal_y, goal_h);
  if (!space_->setStartAndGoal(start_, goal_))
  {
    printf("UPO_RRT. Goal state is not valid!!!\n");
    return false;
  }
  return true;
}


bool RRT::Planner::setStart(float start_x, float start_y, float start_z, float start_h)
{
  start_ = new State(start_x, start_y, start_z, start_h);
  if (!space_->setStart(start_))
  {
    printf("UPO_RRT. Start state is not valid!!!\n");
    return false;
  }
  return true;
}

bool RRT::Planner::setGoal(float goal_x, float goal_y, float goal_z, float goal_h)
{
  goal_ = new State(goal_x, goal_y, goal_z, goal_h);
  if (!space_->setGoal(goal_))
  {
    printf("UPO_RRT. Goal state is not valid!!!\n");
    return false;
  }
  return true;
}


/*
void RRT::Planner::setBiasingPath(std::vector<RRT::State>* path)
{
  //printf("Planner. SetBiasingPath. path size: %u\n", (unsigned int)path->size());
  first_path_ = *path;
}*/



void RRT::Planner::storeTree(std::vector<Node> list)
{
  if (tree_.size() > 0)
    tree_.clear();

  unsigned int fathers = 0;
  unsigned int leaves = 0;
  for (unsigned int j = 0; j < list.size(); j++)
  {
    Node* parent = list[j].getParent();
    if (list[j].hasChildren())
      fathers++;
    else
      leaves++;
    if (parent != nullptr)
    {
      // printf("storeTree. parent %u, hasChildren %i\n",j, (int)parent->hasChildren());
      // Store parent
      State state_p = *(parent->getState());
      // copyState(&state_p, parent->getState());
      tree_.push_back(state_p);
      // Store child
      State state_c = *(list[j].getState());
      // copyState(&state_c, list[j]->getState());
      tree_.push_back(state_c);
    }  // else
    //	printf("storeTree. node %u has not parent\n", j);
  }
  // printf("Store tree. list %u, tree: %u, fathers: %u, leaves:%u\n", (unsigned
  // int)list.size(), (unsigned int)tree_.size(), fathers, leaves);
}


void RRT::Planner::storeLeafNodes(std::vector<Node> list)
{
  // std::vector<Node> leaves_aux;
  // nn_->list(nodes);

  // if(leaves_.size() > 0) {
  leaves_.clear();
  // leaves_states_.clear();
  //}

  // std::vector<Node> leaves;
  for (unsigned int j = 0; j < list.size(); j++)
  {
    if (list[j].hasChildren() == false)
    {
      // Store leaf
      leaves_.push_back(list[j]);
      // leaves_pointers.push_back(list[j]);
      // printf("storeLeafNodes. node %u is a leaf\n", j);
    } /*else
     {
       printf("storeLeafNodes. node %u has children\n", j);
     }*/
  }
  //return leaves_;
}
