#ifndef RRT_STATE_CHECKER_
#define RRT_STATE_CHECKER_

#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <math.h>

namespace RRT
{
	class StateChecker
	{

		public:
			StateChecker(){};
			virtual ~StateChecker(){}
		
			virtual bool isValid(State* s) const = 0;

			virtual bool getValid3dState(State* s) const = 0;
			
			virtual float distance(State* s1, State* s2) const {
				float dx = s1->getX() - s2->getX();
				float dy = s1->getY() - s2->getY();
				float dz = s1->getZ() - s2->getZ();
				return sqrt(dx*dx + dy*dy + dz*dz);
			}
			
			virtual float getCost(State* s) = 0;
			
			//virtual void preplanning_computations() = 0;
			
			
			//This two methods are for exploration purposes
			virtual std::vector<Node*> clusterize_leaves(std::vector<Node*>* leaves) const {
				return *leaves;
			}
			
			virtual Node* evaluate_exploration(std::vector<Node*>* leaves) const {
				return leaves->at(0);
			}


	};
}
#endif
