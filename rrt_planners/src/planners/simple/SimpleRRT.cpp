#include <rrt_planners/planners/simple/SimpleRRT.h>

//#define PRINT_STATISTICS false


RRT::SimpleRRT::SimpleRRT() : Planner() {
	
	//maxRange_ = 0.5;
	//steering_ = new Steering(space_, maxRange_);
}

RRT::SimpleRRT::~SimpleRRT() {

}



RRT::State* RRT::SimpleRRT::steer(State* fromState, State* toState, std::vector<State>& istates)
{
	return  steering_->simple3dSteer(fromState, toState, istates);
}



std::vector<RRT::Node> RRT::SimpleRRT::solve(float secs)
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
	Node* ini = new Node(*start_);
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
		State randState;
		
		//sample goal according to the bias parameter
		if(space_->sampleUniform() < goalBias_ && !exploration_)
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
				
			//} while(!space_->isStateValid(&randState));
			} while(!space_->getValid3dState(&randState));
		}
		
		Node randNode(randState);
		
		//Find nearest node in the tree
		Node* nearNode = nn_->nearest(&randNode); 
		
		std::vector<State> inter_states;
		State* newState = steer(nearNode->getState(), randNode.getState(), inter_states);
		//delete randNode;
		
		//Add the new node to the tree
		if(newState != NULL)
		{
			Node* newNode = new Node(*newState);
			//delete newState;
			newNode->setIntermediateStates(inter_states);
			newNode->setParent(nearNode);
			//setChildren(newNode);
			nn_->add(newNode);
			tree_nodes++;
	
	
			if(!exploration_)
			{
				//Check if we reach the goal
				float dist = 0.0;
				solved = space_->isSimpleGoalToleranceSatisfied(newState, dist);

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
			}
		}
		

		gettimeofday(&stop, NULL);
		t2=stop.tv_sec+(stop.tv_usec/1000000.0);
		time = t2 - t1;
		if(time >= secs || solved) {
			end = true;
		}
	}

	if(solution == NULL && !exploration_)
	{
		printf("\nRRT. Approximate solution found. dist to goal: %.3f\n", approxDist);
		solution = approxSolution;
	
	} else {
		printf("\nRRT. Exlporation performed during %.6f secs\n", time);
	}
	
	
	if(storeTree_) {
		std::vector<Node*> nodes;
		nn_->list(nodes);
		storeTree(nodes);
		if(exploration_) {
			//storeLeafNodes(nodes);
			solution = space_->exploreLeafStates(storeLeafNodes(nodes));
		}
	} else if(exploration_) {
		std::vector<Node*> nodes;
		nn_->list(nodes);
		//storeLeafNodes(nodes);
		solution = space_->exploreLeafStates(storeLeafNodes(nodes));
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

	stats_.planning_time = time;
	stats_.first_sol_time = time;
	stats_.total_samples = total_samples;
	stats_.valid_samples = valid_samples;
	stats_.goal_samples = goal_samples;
	stats_.tree_nodes = tree_nodes;
	stats_.leaf_nodes = leaves_.size();
	stats_.path_nodes = path_nodes;
	
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
	//delete ini;
	//delete solution;
	//delete approxSolution;
	
	freeTreeMemory();
	
	return path;
}


/*void RRT::RRT::getTree(std::vector<RRT::Node*> &tree) const {
	//nn_->list(tree);
	nn_->getTree(tree);
	printf("Getting tree. Size: %u\n", (unsigned int)tree.size());
	for(unsigned int i=0; i<tree.size(); i++) {
		if(tree[i]->getState() == NULL)
			printf("State node %u equals to NULL\n", i);
		else
			printf("Node %u. x:%.2f, y:%.2f\n", i+1, tree[i]->getState()->getX(), tree[i]->getState()->getY());
	}
	//mytree = tree;
}*/


