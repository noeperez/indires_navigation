#include <rrt_planners/planners/control/HalfRRTstar.h>

RRT::HalfRRTstar::HalfRRTstar() : Planner()
{
  maxRange_ = 0.5;
  useKnearest_ = true;
  k_rrt_ = 0.0;
  r_rrt_ = 0.0;
  rewire_factor_ = 1.1;

  // useFirstPathBiasing_ = false;
  // pathBias_ = 0.0;
  // pathBias_stddev_ = 0.0;
}

RRT::HalfRRTstar::~HalfRRTstar()
{
}


bool RRT::HalfRRTstar::steer(Node* fromNode, Node* toNode, Node* newNode)
{
  if (space_->getDimensions() == 2)
    return steering_->steer2(fromNode, toNode, newNode);
  else
    return steering_->steer3(fromNode, toNode, newNode);
}

bool RRT::HalfRRTstar::collisionFree(Node* fromNode, Node* toNode, std::vector<Action>& acts,
                                     std::vector<State>& istates, float& motCost)
{
  if (space_->getDimensions() == 2)
  {
    return steering_->collisionFree2(fromNode, toNode, acts, istates, motCost);
  }
  else
  {
    return steering_->collisionFree3(fromNode, toNode, acts, istates, motCost);
  }
}



std::vector<RRT::Node> RRT::HalfRRTstar::solve(float secs)
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

  // Clear datastructure and initilize it
  // nn_->clear();
  KDTree<RRT::Node> nn(dimensions_);

  Node ini(*start_, *init_action_state_);
  std::vector<Action>* actss = ini.getAction();
  float vxx = 100.0, vyy = 100.0, vtt = 100.0;
  unsigned int vs;
  actss->at(actss->size() - 1).getAction(vxx, vyy, vtt, vs);
  printf("halfRRTstar solve. Ini x:%.2f, y:%.2f, vx:%.2f, vth:%.2f\n",
         ini.getState()->getX(), ini.getState()->getY(), vxx, vtt);
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

  unsigned int total_rewirings = 0;
  unsigned int posible_rewirings = 0;

  bool solved = false;
  bool end = false;
  bool first_sol = true;
  Node solution;
  Node approxSolution;
  float approxDist = std::numeric_limits<float>::max();

  // Goal node
  Node* goalNode = new Node(*goal_);
  goalNode->setCost(space_->getCost(goal_));

  // unsigned int cont_null = 0;

  double t1, t2;
  struct timeval stop, start, first_stop;
  gettimeofday(&start, NULL);
  t1 = start.tv_sec + (start.tv_usec / 1000000.0);
  while (!end)
  {
    State randState;

    // sample goal according to the bias parameter
    if (first_sol && space_->sampleUniform() < goalBias_)
    {
      randState = *goal_;
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
        randState = *space_->sampleState();

        cont++;
        if (cont > 1)
          total_samples++;
        else
        {
          valid_samples++;
          total_samples++;
        }


      } while (!space_->isStateValid(&randState));
    }

    Node randNode(randState);

    // Find nearest node in the tree
    // Node* nearNode = nn_->nearest(&randNode);
    std::shared_ptr<Node> nearNode = std::make_shared<Node>(nn.nearest(randNode));
    if (nearNode == nullptr)
      printf("\n\nNo nearest node!!!!!\n\n");


    Node newNode;

    if (steer(nearNode.get(), &randNode, &newNode))
    {
      // Use the neighbors of the new node to find the best parent
      std::vector<RRT::Node> nbrs;
      getNearestNeighbors(nn, newNode, nbrs);
      // printf("Neighbors of newnode obtained: %u\n", (unsigned int)nbrs.size());

      Node* node_min = nearNode.get();
      float inc_cost = newNode.getIncCost();
      float cost_min = nearNode->getAccCost() + inc_cost;


      // Check the nodes costs to chose the parent with
      // a lower cost connection
      std::vector<Action> acts;
      std::vector<Action> acts_aux;
      State st;
      float aux_inc_cost = 0.0;
      std::vector<State> i_aux;
      std::vector<State> istates;
      for (unsigned int i = 0; i < nbrs.size(); i++)
      {
        if (&nbrs[i] != nearNode.get() &&
            collisionFree(&nbrs[i], &newNode, acts_aux, i_aux, aux_inc_cost))
        {
          float total_cost = nbrs[i].getAccCost() + aux_inc_cost;
          if (total_cost < cost_min)
          {
            // printf("Changing parent\n");
            node_min = &nbrs[i];
            cost_min = total_cost;
            inc_cost = aux_inc_cost;
            acts = acts_aux;
            st = i_aux.at(i_aux.size() - 1);
            istates = i_aux;
          }
        }
      }

      // node_min->setChildren(newNode);
      newNode.setParent(node_min);
      if (node_min != nearNode.get())
      {
        // printf("Connecting parent!!\n");
        newNode.setIncCost(inc_cost);
        newNode.setAccCost(cost_min);
        newNode.setAction(acts);
        if (space_->getDimensions() == 2)
          newNode.getState()->setYaw(st.getYaw());

        newNode.getState()->setLv(st.getLinVel());
        newNode.getState()->setAv(st.getAngVel());

        newNode.setIntermediateStates(istates);

        // if we have a cost based on the
        // yaw, lv or av, we will have to recalculate it again later!!!
        // newNode->setCost(space_->getCost(newNode->getState()));
      }


      nn.add(newNode);
      tree_nodes++;


      // Rewire the tree
      /*std::vector<Action> acts2;
      State st2;
      for(unsigned int i=0; i<nbrs.size(); i++)
      {
        if(nbrs[i]!=node_min && collisionFree(newNode, nbrs[i], acts2, istates,
      aux_inc_cost))
        {
          posible_rewirings++;
          //Node* node_aux = new Node(&st_aux2);
          //node_aux->setAction(acts2);
          //aux_inc_cost = motionCost(newNode, node_aux);
          float total_cost = newNode->getAccCost() + aux_inc_cost;
          if(total_cost < nbrs[i]->getAccCost()) {
            total_rewirings++;
            //newNode->setChildren(nbrs[i]);
            nbrs[i]->setParent(newNode);
            nbrs[i]->setIncCost(aux_inc_cost);
            nbrs[i]->setAccCost(total_cost);
            nbrs[i]->setAction(acts2);
            st2 = istates.at(istates.size()-1);
            if(space_->getDimensions() == 2) {

              nbrs[i]->getState()->setYaw(st2.getYaw());
            }
            nbrs[i]->getState()->setLv(st2.getLinVel());
            nbrs[i]->getState()->setAv(st2.getAngVel());
            nbrs[i]->setIntermediateStates(istates);

            //if we have a cost based on the
            //yaw, lv or av, we will have to recalculate it again later!!!
            //nbrs[i]->setCost(space_->getCost(nbrs[i]->getState()));
          }
        }
      }*/

      float dist = 0.0;
      solved = space_->isGoalToleranceSatisfied(newNode.getState(), dist);

      if (solved)
      {
        // We re-establish the correct orientation of the goal.
        // Probably we changed it if we were planning in 2 dimensions.
        if (space_->getDimensions() == 2)
          newNode.getState()->setYaw(goalNode->getState()->getYaw());
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
              State state;
              copyState(&state, current->getState());
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
    else
    {
      // cont_null++;
    }

    gettimeofday(&stop, NULL);
    t2 = stop.tv_sec + (stop.tv_usec / 1000000.0);
    time = t2 - t1;
    // printf("Time: %.3f, fin: %.3f\n", time, secs);
    if (time >= secs)
    {
      end = true;
    }
  }

  if (!solved)
  {
    printf("\nRRT. Approximate solution found. dist to goal: %.3f\n", approxDist);
    solution = approxSolution;
  }
  else
  {
    printf("\nRRT. Solution found in %.6f secs\n", time);
  }

  // printf("Number of null states: %u\n", cont_null);
  // printf("Possible rewirings: %u. Total rewirings peformed: %u\n", posible_rewirings,
  // total_rewirings);

  if (storeTree_)
  {
    std::vector<Node> nodes;
    nn.list(nodes);
    storeTree(nodes);
  }


  // Construct the solution path
  std::vector<RRT::Node> path;
  Node* current = &solution;
  path_cost_ = current->getAccCost();
  while (current != NULL)
  {
    Node node = *current;
    // copyNode(&node, current);
    path.push_back(node);
    current = current->getParent();
    path_nodes++;
  }

  // Fill statistics
  stats_.planning_time = time;
  stats_.first_sol_time = first_sol_time;
  stats_.total_samples = total_samples;
  stats_.valid_samples = valid_samples;
  stats_.goal_samples = goal_samples;
  stats_.tree_nodes = tree_nodes;
  stats_.path_nodes = path_nodes;


  delete current;
  // delete ini;
  delete goalNode;
  // delete solution;
  // delete approxSolution;

  // freeTreeMemory();

  return path;
}



void RRT::HalfRRTstar::getNearestNeighbors(KDTree<RRT::Node>& nn,
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

void RRT::HalfRRTstar::calculateParamsNearest()
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
    // printf("CalculateParamNearest. Dim: %.0f, free_volume: %.3f, unitBall: %.3f,
    // r_rrt_: %.3f\n", dim, free_volume, unitBall, r_rrt_);
  }
}


/*float RRT::RRTstar::motionCost(Node* n1, Node* n2) {
  if(space_->getDimensions() == 2) {
    //if(useKnearest_)  {
      float dist = sqrt(distanceFunction(n1, n2));
      return ((n1->getCost() + n2->getCost()) / 2.0) * exp(dist);
    //} else {
    //	return ((n1->getCost() + n2->getCost()) / 2.0);
    //}
  } else {
    return ((n1->getCost() + n2->getCost()) / 2.0) * exp(distanceFunction(n1, n2));
  }
}*/
