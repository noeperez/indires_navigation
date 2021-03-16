#ifndef RRT_NODE_
#define RRT_NODE_

#include <rrt_planners/State.h>
#include <rrt_planners/Action.h>

#include <vector>
#include <iostream>  //NULL
#include <stdio.h>

namespace RRT
{
class Node
{
public:
  Node();
  Node(State s);
  Node(State s, Action a);
  ~Node();

  State* getState();
  std::vector<Action>* getAction();
  Node* getParent();
  // std::vector<Node*> getChildren();
  float getCost();
  float getIncCost();
  float getAccCost();


  void setParent(Node* p);
  void setChildren();
  void setState(State s);
  void addAction(Action a);
  void setAction(std::vector<Action> a);

  void setCost(float c);
  void setIncCost(float c);
  void setAccCost(float c);

  void addIntermediateState(State s);
  void setIntermediateStates(std::vector<State> s);
  std::vector<State>* getIntermediateStates();
  bool hasIntermediateStates();

  bool hasChildren();

  float distance(const Node& other) const
  {
    return (state_.getX() - other.state_.getX()) * (state_.getX() - other.state_.getX()) +
           (state_.getY() - other.state_.getY()) * (state_.getY() - other.state_.getY()) +
           (state_.getZ() - other.state_.getZ()) * (state_.getZ() - other.state_.getZ());
  }

  float operator()(int index) const
  {
    return state_(index);
  }
  float operator[](int index) const
  {
    return state_[index];
  }
  bool operator==(const Node& other) const
  {
    return state_ == other.state_;
  }


  // bool deleteStateAndControl();

private:
  // The state contained by the node
  State state_;

  // The action contained by the node
  std::vector<Action> control_;

  // Intermediate points if they exists
  std::vector<State> intermediate_states_;

  bool exist_intermediate_states_;

  // The parent node in the exploration tree
  Node* parent_;

  // if this node is a leaf node
  bool has_children_;

  // The set of nodes descending from the current node
  // std::vector<Node*> children_;

  // The cost up to this node
  float accCost_;

  // The incremental cost of this node's parent to this node
  float incCost_;

  // The cost associated to this single node
  float cost_;
};
}
#endif
