#include <rrt_planners/planners/control/Rrt.h>

//#define PRINT_STATISTICS false


RRT::Rrt::Rrt() : Planner()
{
  // accompany_steer_ = false;
}

RRT::Rrt::~Rrt()
{
}



bool RRT::Rrt::steer(Node* fromNode, Node* toNode, Node* newNode)
{
  // if(accompany_steer_)
  //	return steering_->accompany_steer(fromNode, toNode, newNode);

  return steering_->rrt_3d_steer(fromNode, toNode, newNode);
}



std::vector<RRT::Node> RRT::Rrt::solve(float secs)
{
  /**************************************************
  GENERATE_RRT(Xinit, K, t)
    V<-{Xinit}, E<-0;
    for k=1 to K do
      Xrand <- SampleFree;
      Xnearest <- Nearest(G=(V,E), Xrand);
      Unew <- SelectInput(Xrand, Xnearest, Xrand);
      Xnew <- Steer(Xnearest, Unew, t);
      V <- V U {Xnew};
      E <- E U {(Xnearest, Xnew, Unew)};
    return G = (V,E);
  ***************************************************/

  // Clear datastructure and initilize it
  // nn_->clear();
  KDTree<RRT::Node> nn(dimensions_);

  // Action* act = new Action(0.0, 0.0, 0.0, 5);
  // Node* ini = new Node(*start_, *init_action_state_);
  Node ini(*start_, *init_action_state_);
  nn.add(ini);

  tree_.clear();

  // Statistics
  unsigned int total_samples = 0;
  unsigned int valid_samples = 0;
  unsigned int goal_samples = 0;
  unsigned int tree_nodes = 1;
  unsigned int path_nodes = 0;
  float time = 0.0;

  // unsigned int fathers = 0;

  bool solved = false;
  bool end = false;
  Node solution;
  Node approxSolution;
  float approxDist = std::numeric_limits<float>::max();
  double t1, t2;
  struct timeval stop, start;
  gettimeofday(&start, NULL);
  t1 = start.tv_sec + (start.tv_usec / 1000000.0);
  while (!end)
  {
    // Node* randNode;
    State randState;


    if (space_->sampleUniform() < goalBias_ &&
        !exploration_)  // sample goal according to the bias parameter
    {
      randState = *goal_;
      // State* randState = goal_;
      // randNode = new Node(*randState);
      goal_samples++;
      valid_samples++;
      total_samples++;
      // delete randState;
    }
    else
    {
      // Sample a random valid state
      unsigned int cont = 0;
      do
      {
        randState = *space_->sampleState();

        cont++;
        if (cont > 1)
          total_samples++;
        else
        {
          valid_samples++;
          total_samples++;
        }
        // delete randState;
      } while (!space_->getValid3dState(&randState));
    }

    Node randNode(randState);

    // Find nearest node in the tree
    // Node* nearNode = nn_->nearest(&randNode);
    std::shared_ptr<Node> nearNode = std::make_shared<Node>(nn.nearest(randNode));
    /*if(!nearNode->hasChildren()){
      fathers++;
      nearNode->setChildren();
    }*/

    Node newNode;

    // Add the new node to the tree
    if (steer(nearNode.get(), &randNode, &newNode))
    {
      newNode.setParent(nearNode.get());
      nearNode->setChildren();

      nn.add(newNode);
      tree_nodes++;

      if (!exploration_)
      {
        float dist = 0.0;
        solved = space_->isSimpleGoalToleranceSatisfied(newNode.getState(), dist);  // isGoalToleranceSatisfied

        if (solved)
        {
          approxDist = dist;
          solution = newNode;
          // break;
        }
        else if (dist < approxDist)
        {
          approxDist = dist;
          approxSolution = newNode;
        }
      }
    }
    else
    {
      // printf("Node NULL!!!!\n");
    }

    gettimeofday(&stop, NULL);
    t2 = stop.tv_sec + (stop.tv_usec / 1000000.0);
    time = t2 - t1;
    if (time >= secs || solved)
    {
      end = true;
    }
  }

  if (!solved && !exploration_)
  {
    printf("\nRRT. Approximate solution found. dist to goal: %.3f\n", approxDist);
    solution = approxSolution;
  }
  else
  {
    printf("\nRRT. Exlporation performed during %.6f secs\n", time);
  }

  unsigned int tree_total_nodes = 0;
  if (storeTree_)
  {
    std::vector<Node> nodes;
    nn.list(nodes);
    tree_total_nodes = (unsigned int)nodes.size();
    storeTree(nodes);
    if (exploration_)
    {
      storeLeafNodes(nodes);
      solution = space_->exploreLeafStates(leaves_);
    }
  }
  else if (exploration_)
  {
    std::vector<Node> nodes;
    nn.list(nodes);
    storeLeafNodes(nodes);
    solution = space_->exploreLeafStates(leaves_);
  }

  // Construct the solution path
  std::vector<RRT::Node> path;
  // solution->getState()->setYaw(goal_->getYaw());
  Node* current = &solution;
  path_cost_ = current->getAccCost();
  while (current != nullptr)
  {
    Node node = *current;
    // copyNode(&node, current);
    path.push_back(node);
    current = current->getParent();
    path_nodes++;
  }
  /*for(unsigned int k=0; k<path.size(); k++) {
    printf("Solution. Action.size(): %u, lv:%.2f, av:%.2f\n", (unsigned
  int)path[k].getAction().size(), path[k].getAction().at(0)->getVx(),
  path[k].getAction().at(0)->getVth());
  }*/

  stats_.planning_time = time;
  stats_.first_sol_time = time;
  stats_.total_samples = total_samples;
  stats_.valid_samples = valid_samples;
  stats_.goal_samples = goal_samples;
  stats_.tree_nodes = tree_nodes;
  stats_.leaf_nodes = leaves_.size();
  stats_.path_nodes = path_nodes;
  // printf("Rrt.cpp. fathers: %u\n", fathers);
  // printf("Rrt.cpp. Total tree nodes: %u\n", tree_total_nodes);

  /*if(PRINT_STATISTICS)
  {
    printf("Planning time:   %.4f secs\n", time);
    printf("Total samples:   %u \n", total_samples);
    printf("Valid samples:   %u \n", valid_samples);
    printf("Goal samples:    %u \n",  goal_samples);
    printf("Tree nodes:      %u \n",  tree_nodes);
    printf("Path nodes:      %u \n\n",  path_nodes);
  }*/

  delete current;
  // delete ini;
  // delete solution;
  // delete approxSolution;

  // freeTreeMemory();

  return path;
}
