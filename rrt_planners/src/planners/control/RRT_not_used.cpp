#include <rrt_planners/planners/control/RRT.h>

#define PRINT_STATISTICS false


RRT::RRT::RRT() : Planner() {
	
	accompany_steer_ = false;
}

RRT::RRT::~RRT() {

}



bool RRT::RRT::steer(Node* fromNode, Node* toNode, Node* newNode)
{
	if(accompany_steer_)
		return steering_->accompany_steer(fromNode, toNode, newNode);
		
	return  steering_->rrt_steer(fromNode, toNode, newNode);
	
}




std::vector<RRT::Node> RRT::RRT::solve(float secs)
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

	//Clear datastructure and initilize it
	nn_->clear();
	//Action* act = new Action(0.0, 0.0, 0.0, 5);
	Node* ini = new Node(*start_, *init_action_state_);
	nn_->add(ini);
	
	tree_.clear();
	
	//Statistics
	unsigned int total_samples = 0;
	unsigned int valid_samples = 0;
	unsigned int goal_samples = 0;
	unsigned int tree_nodes = 1;
	unsigned int path_nodes = 0;
 	float time = 0.0;

	bool solved = false;
	bool end = false;
	Node* solution = NULL;
	Node* approxSolution = NULL;
	float approxDist = std::numeric_limits<float>::max();
	double t1, t2;
	struct timeval stop, start;
	gettimeofday(&start, NULL);
	t1=start.tv_sec+(start.tv_usec/1000000.0);
	while(!end)
	{
		//Node* randNode;
		State randState;
		
		
		if(space_->sampleUniform() < goalBias_) //sample goal according to the bias parameter
		{
			randState = *goal_;
			//State* randState = goal_;
			//randNode = new Node(*randState);
			goal_samples++;
			valid_samples++;
			total_samples++;
			//delete randState;
		} else {
			//Sample a random valid state
			unsigned int cont = 0;
			do {

				randState = *space_->sampleState();
					
				cont++;
				if(cont>1)
					total_samples++;
				else {
					valid_samples++;
					total_samples++;
				}
				//delete randState;
			} while(!space_->isState3dValid(&randState));
		}
		
		Node randNode(randState);
		
		//Find nearest node in the tree
		Node* nearNode = nn_->nearest(&randNode);
		
		Node* newNode = new Node();

		//Add the new node to the tree
		if(steer(nearNode, &randNode, newNode))
		{
			newNode->setParent(nearNode);
			//setChildren(newNode);
			nn_->add(newNode);
			tree_nodes++;
			
			float dist = 0.0;
			solved = space_->isGoalToleranceSatisfied(newNode->getState(), dist);

			if (solved)
			{
				approxDist = dist;
 				solution = newNode;
  				//break;
 			}
 			else if (dist < approxDist)
  			{
  				approxDist = dist;
  				approxSolution = newNode;
			}
		} else {
			//printf("Node NULL!!!!\n");
		}

		gettimeofday(&stop, NULL);
		t2=stop.tv_sec+(stop.tv_usec/1000000.0);
		time = t2 - t1;
		if(time >= secs /*|| solved*/) {
			end = true;
		}
	}

	if(solution == NULL)
	{
		printf("\nRRT. Approximate solution found. dist to goal: %.3f\n", approxDist);
		solution = approxSolution;
	
	} else {
		printf("\nRRT. Solution found in %.6f secs\n", time);
	}
	
	
	if(storeTree_) {
		std::vector<Node*> nodes;
		nn_->list(nodes);
		storeTree(nodes);
	}

	//Construct the solution path
	std::vector<RRT::Node> path;
	solution->getState()->setYaw(goal_->getYaw());
	Node* current = solution;
	path_cost_ = current->getAccCost();
	while (current != NULL)
	{
		Node node = *current;
		//copyNode(&node, current);
		path.push_back(node);
 		current = current->getParent();
		path_nodes++;
	}	
	/*for(unsigned int k=0; k<path.size(); k++) {
		printf("Solution. Action.size(): %u, lv:%.2f, av:%.2f\n", (unsigned int)path[k].getAction().size(), path[k].getAction().at(0)->getVx(), path[k].getAction().at(0)->getVth());
	}*/
	
	stats_.planning_time = time;
	stats_.first_sol_time = time;
	stats_.total_samples = total_samples;
	stats_.valid_samples = valid_samples;
	stats_.goal_samples = goal_samples;
	stats_.tree_nodes = tree_nodes;
	stats_.path_nodes = path_nodes;
	
	if(PRINT_STATISTICS)
	{
		printf("Planning time:   %.4f secs\n", time);
		printf("Total samples:   %u \n", total_samples);
		printf("Valid samples:   %u \n", valid_samples);
		printf("Goal samples:    %u \n",  goal_samples);
		printf("Tree nodes:      %u \n",  tree_nodes);
		printf("Path nodes:      %u \n\n",  path_nodes);
	}

	delete current;
	//delete ini;
	//delete solution;
	//delete approxSolution;
	
	freeTreeMemory();

	return path;
}


