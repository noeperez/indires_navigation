#include <rrt_planners/planners/simple/SimpleRRTstar.h>

RRT::SimpleRRTstar::SimpleRRTstar() : Planner()
{
  maxRange_ = 0.5;
  useKnearest_ = false;
  k_rrt_ = 0.0;
  r_rrt_ = 0.0;
  rewire_factor_ = 1.1;

  // useFirstPathBiasing_ = false;
  // pathBias_ = 0.0;
  // pathBias_stddev_ = 0.0;
}

RRT::SimpleRRTstar::~SimpleRRTstar()
{
}


RRT::State* RRT::SimpleRRTstar::steer(State* fromState, State* toState, std::vector<State>& istates)
{
  return steering_->simple3dSteer(fromState, toState, istates);
}


bool RRT::SimpleRRTstar::collisionFree(State* fromState, State* toState, std::vector<State>& istates)
{
  return steering_->simple3dCollisionFree(fromState, toState, istates);
}



std::vector<RRT::Node> RRT::SimpleRRTstar::solve(float secs)
{
  /******************************************************************************************
  V<-{Xinit}, E<-0;
  for int i=0,...,n do
    Xrand <- SampleFree;
    Xnearest <- Nearest(G=(V,E), Xrand);
    {(Xnew, Unew, Tnew)} <- Steer(Xnearest, Xrand);
    if(ObstacleFree(Xnearest, Xnew)) then
      Xs_near <- Near(G=(V,E), Xnew, min{gamma(log(Card(V)/Card(V))^(1/d), eta});
      V <- V U {Xnew}
      Xmin <- Xnearest;
      Cmin <- Cost(Xnearest) + C(Line(Xnearest, Xnew));
      for each Xnear in Xs_near do   	//Connect along a minimum-cost path
        if CollisionFree(Xnear, Xnew) && Cost(Xnear)+C(Line(Xnear, Xnew)) < Cmin then
          Xmin <- Xnear;
          Cmin <- Cost(Xnear) + C(Line(Xnear, Xnear));
      E <- E U {(Xmin, Xnew)};
      for each Xnear in Xs_near do 	//Rewire the tree
        if CollisionFree(Xnew, Xnear) && Cost(Xnew) + C(Line(Xnew, Xnear)) < Cost(Xnear)
  then
          Xparent <- Parent(Xnear);
        E <- (E\{(Xparent, Xnear)}) U {(Xnew, Xnear)};
  return G=(V,E);
  ***********************************************************************************************/

  //Initialize kdtree estructure
  KDTree<RRT::Node> nn(dimensions_);


  Node ini(*start_);
  float singleCost = space_->getCost(start_);
  ini.setCost(singleCost);
  ini.setIncCost(singleCost);
  ini.setAccCost(singleCost);
  nn.add(ini);

  calculateParamsNearest();
  tree_.clear();


  // Statistics
  unsigned int total_samples = 0;
  unsigned int valid_samples = 0;
  unsigned int goal_samples = 0;
  unsigned int tree_nodes = 1;
  unsigned int path_nodes = 0;
  float time = 0.0;
  float first_sol_time = 0.0;

  bool solved = false;
  bool end = false;
  bool first_sol = true;
  Node solution;
  Node approxSolution;
  float approxDist = std::numeric_limits<float>::max();



  // Goal node
  Node* goalNode;
  if (!exploration_)
  {
    goalNode = new Node(*goal_);
    goalNode->setCost(space_->getCost(goal_));
  }

  unsigned int cont_null = 0;

  double t1, t2;
  struct timeval stop, start, first_stop;
  gettimeofday(&start, NULL);
  t1 = start.tv_sec + (start.tv_usec / 1000000.0);
  while (!end)
  {
    State randState;

    // sample goal according to the bias parameter
    if (first_sol && space_->sampleUniform() < goalBias_ && !exploration_)
    {
      randState = *goalNode->getState();
      goal_samples++;
      valid_samples++;
      total_samples++;
    }
    else
    {
      // Sample a random valid state
      unsigned int cont = 0;
      do
      {
        // Regular state sampling
        randState = *space_->sampleState();

        cont++;
        if (cont > 1)
          total_samples++;
        else
        {
          valid_samples++;
          total_samples++;
        }

      } while (!space_->getValid3dState(&randState));
    }


    Node randNode(randState);

    // Find nearest node in the tree
    std::shared_ptr<Node> nearNode = std::make_shared<Node>(nn.nearest(randNode));
    //Node& nearNode = nn.nearest(randNode);

    if (nearNode == nullptr)
      printf("SimpleRRTStar. nearNode = NULL\n");

    // Steer from nearState to randState and reach the new state
    std::vector<State> inter_states;
    State* newState = steer(nearNode->getState(), randNode.getState(), inter_states);

    if (newState != nullptr)
    {
      Node newNode(*newState);
      newNode.setIntermediateStates(inter_states);
      newNode.setCost(space_->getCost(newNode.getState()));

      // Use the neighbors of the new node to find the best parent
      std::vector<RRT::Node> nbrs;
      getNearestNeighbors(nn, newNode, nbrs);

      Node* node_min = nearNode.get();
      float inc_cost = steering_->motionCost(nearNode.get(), &newNode);
      float cost_min = nearNode->getAccCost() + inc_cost;
      // Check the nodes costs to chose the parent with
      // a lower connection cost

      std::vector<unsigned int> nbrs_ind;

      std::vector<State> intermediates;
      intermediates = inter_states;
      for (unsigned int i = 0; i < nbrs.size(); i++)
      {
        if ((&nbrs[i]) != nearNode.get())
        {
          float total_cost = nbrs[i].getAccCost() + steering_->motionCost(&nbrs[i], &newNode);
          if (total_cost < cost_min)
          {
            if (collisionFree(nbrs[i].getState(), newNode.getState(), inter_states))
            {
              nbrs_ind.push_back(i);
              node_min = &nbrs[i];
              cost_min = total_cost;
              intermediates = inter_states;
            }
          }
        }
      }

      newNode.setParent(node_min);
      newNode.setIncCost(steering_->motionCost(node_min, &newNode));
      newNode.setAccCost(cost_min);
      newNode.setIntermediateStates(intermediates);

      // Add the new node to the tree
      nn.add(newNode);
      tree_nodes++;


      // Rewire the tree
      for (unsigned int i = 0; i < nbrs.size(); i++)
      {
        if (&nbrs[i] != node_min)  //&& collisionFree(newState, nbrs_aux[i]->getState())
                                   ////Only valid for non-kinematics RRT*
        {
          float total_cost = newNode.getAccCost() + steering_->motionCost(&newNode, &nbrs[i]);
          if (total_cost < nbrs[i].getAccCost())
          {
            bool colfree = false;
            for (unsigned int f = 0; f < nbrs_ind.size(); f++)
            {
              if (nbrs_ind[f] == i)
              {
                colfree = true;
                break;
              }
            }
            if (colfree || collisionFree(newNode.getState(), nbrs[i].getState(), inter_states))
            {
              // newNode->setChildren(nbrs[i]);
              nbrs[i].setParent(&newNode);
              nbrs[i].setIncCost(steering_->motionCost(&newNode, &nbrs[i]));
              nbrs[i].setAccCost(total_cost);
            }
          }
        }
      }



      if (!exploration_)
      {
        float dist = 0.0;
        solved = space_->isSimpleGoalToleranceSatisfied(newNode.getState(), dist);

        if (solved)
        {
          approxDist = dist;
          solution = newNode;
          if (first_sol)
          {
            gettimeofday(&first_stop, NULL);
            double t3 = first_stop.tv_sec + (first_stop.tv_usec / 1000000.0);
            first_sol_time = (t3 - t1);

            // Store the first solution to draw samples from it.
            /*if(useFirstPathBiasing_) {
              Node* current = solution;
              while (current != NULL)
              {
                State state = *current->getState();
                //copyState(&state, current->getState());
                first_path_.push_back(state);
                current = current->getParent();
              }

            }*/
            first_sol = false;
          }
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
      cont_null++;
    }


    gettimeofday(&stop, NULL);
    t2 = stop.tv_sec + (stop.tv_usec / 1000000.0);
    time = t2 - t1;
    if (time >= secs)
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
    printf("\nRRT. Algorithm executed for %.6f secs\n", time);
  }

  if (storeTree_)
  {
    std::vector<Node> nodes;
    nn.list(nodes);
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

  // path for a weird bug
  if (approxDist > 1000.0 && !exploration_)
  {
    printf("\nSimpleRRTStar. Error calculating the path!!!!\n\n");
    //freeTreeMemory();
    return path;
  }

  if (!exploration_)
    solution.getState()->setYaw(goal_->getYaw());
  Node* current = &solution;
  path_cost_ = current->getAccCost();
  if (path_cost_ == 0)
    printf("\n----------SimpleRRTStar. path_cost = 0!!!!-------------\n");
  else
    printf("Path cost: %.4f\n", path_cost_);


  while (current != NULL)
  {
    Node node = *current;
    // copyNode(&node, current);
    path.push_back(node);
    current = current->getParent();
    path_nodes++;
  }

  stats_.planning_time = time;
  stats_.first_sol_time = first_sol_time;
  stats_.total_samples = total_samples;
  stats_.valid_samples = valid_samples;
  stats_.goal_samples = goal_samples;
  stats_.tree_nodes = tree_nodes;
  stats_.leaf_nodes = leaves_.size();
  stats_.path_nodes = path_nodes;


  delete current;
  // delete ini;
  if (!exploration_)
    delete goalNode;
  // delete solution;
  // delete approxSolution;

  // freeTreeMemory();
  // first_path_.clear();

  return path;
}


void RRT::SimpleRRTstar::getNearestNeighbors(KDTree<RRT::Node>& nn,
                                             const Node& node, std::vector<Node>& nbrs)
{
  double size = static_cast<double>(nn.size() + 1u);
  if (useKnearest_)
  {
    // k-nearest RRT*
    unsigned int k = std::ceil(k_rrt_ * log(size));
    // printf("k: %u\n", k);
    nbrs = nn.knearest(node, k);
  }
  else
  {
    // Nearest in radius RRT*
    double r = std::min((double)maxRange_,
                        r_rrt_ * std::pow(log(size) / size,
                                          1 / static_cast<double>(space_->getDimensions())));
    // printf("Neighbors in Radius: %.3f\n", r);
    nbrs = nn.radiusSearch(node, r * r);
  }
}

void RRT::SimpleRRTstar::calculateParamsNearest()
{
  double dim = (double)space_->getDimensions();

  // K-nearest
  if (useKnearest_)
  {
    // k_rrt_ = 1.1 * (boost::math::constants::e<double>() +
    // (boost::math::constants::e<double>() / dim));
    k_rrt_ = rewire_factor_ * (exp(1) + (exp(1) / dim));
  }
  else
  {
    // Radius
    float free_volume = space_->getSpaceMeasure() / 2.0;
    float unitBall = space_->getUnitBallMeasure();
    r_rrt_ = rewire_factor_ * 2 * pow((1 + 1 / dim) * (free_volume / unitBall), 1 / dim);
  }
}


/*float RRT::SimpleRRTstar::motionCost(Node* n1, Node* n2) {

  float dist = sqrt(distanceFunction(n1, n2));
  //Normalize
  //float max_d =
sqrt((space_->getSizeX()*2*space_->getSizeX()*2)+(space_->getSizeY()*2*space_->getSizeY()*2));
  //dist = dist / max_d;

  //printf("Dist: %.3f, sqrt(dist): %.3f, avg cost: %.3f\n", dist, sqrt(dist),
(n1->getCost() + n2->getCost())/2.0 );
  return ((n1->getCost() + n2->getCost()) / 2.0)  *  dist;
  //return ((n1->getCost() + n2->getCost()) / 2.0) + dist;

}*/
