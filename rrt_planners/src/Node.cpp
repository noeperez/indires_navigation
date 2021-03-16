#include <rrt_planners/Node.h>


RRT::Node::Node() : parent_(nullptr)
{
  // state_ = new State();
  // Action* a = new Action();
  // control_.push_back(a);
  cost_ = 0.0;
  incCost_ = 0.0;
  accCost_ = 0.0;
  exist_intermediate_states_ = false;
  has_children_ = false;
}


RRT::Node::Node(State s) : parent_(nullptr)
{
  state_ = s;
  // Action* a = new Action();
  // control_.push_back(a);
  cost_ = 0.0;
  incCost_ = 0.0;
  accCost_ = 0.0;
  exist_intermediate_states_ = false;
  has_children_ = false;
}

RRT::Node::Node(State s, Action a) : parent_(nullptr)
{
  state_ = s;
  state_.setLv(a.getVx());
  state_.setAv(a.getVth());
  control_.clear();
  control_.push_back(a);
  cost_ = 0.0;
  incCost_ = 0.0;
  accCost_ = 0.0;
  exist_intermediate_states_ = false;
  has_children_ = false;
}


RRT::Node::~Node()
{
  // parent_ = NULL;

  // intermediate_states_.clear();
  // control_.clear();
}


/*bool upo_RRT::Node::deleteStateAndControl() {

  if(state_)
    delete state_;

  for(unsigned int i=0; i<control_.size(); i++)
    delete control_[i];

  //for(unsigned int j=0; j<intermediate_states_.size(); j++)
  //	delete intermediate_states_[j];

  intermediate_states_.clear();
  control_.clear();
  exist_intermediate_states_ = false;
}*/


RRT::State* RRT::Node::getState()
{
  return &state_;
}


std::vector<RRT::Action>* RRT::Node::getAction()
{
  return &control_;
}

RRT::Node* RRT::Node::getParent()
{
  return parent_;
}

/*std::vector<RRT::Node*> RRT::Node::getChildren() {
  return children_;
}*/

std::vector<RRT::State>* RRT::Node::getIntermediateStates()
{
  if (!exist_intermediate_states_)
    printf("Node. No intermediate states exist!!! Prone to error!\n");
  return &intermediate_states_;
}

float RRT::Node::getCost()
{
  return cost_;
}

float RRT::Node::getIncCost()
{
  return incCost_;
}

float RRT::Node::getAccCost()
{
  return accCost_;
}


void RRT::Node::setParent(Node* p)
{
  parent_ = p;
  p->setChildren();
}

void RRT::Node::setChildren()
{
  has_children_ = true;
  // printf("Setting has_children to true\n");
}

bool RRT::Node::hasChildren()
{
  return has_children_;
}

void RRT::Node::setState(State s)
{
  state_ = s;
}

void RRT::Node::addAction(Action a)
{
  control_.push_back(a);
}

void RRT::Node::setAction(std::vector<Action> a)
{
  control_.clear();
  control_ = a;
}

void RRT::Node::addIntermediateState(State s)
{
  exist_intermediate_states_ = true;
  intermediate_states_.push_back(s);
}

void RRT::Node::setIntermediateStates(std::vector<State> s)
{
  intermediate_states_.clear();
  intermediate_states_ = s;
  exist_intermediate_states_ = true;
}

bool RRT::Node::hasIntermediateStates()
{
  return exist_intermediate_states_;
}

void RRT::Node::setCost(float c)
{
  cost_ = c;
}
void RRT::Node::setIncCost(float c)
{
  incCost_ = c;
}
void RRT::Node::setAccCost(float c)
{
  accCost_ = c;
}
