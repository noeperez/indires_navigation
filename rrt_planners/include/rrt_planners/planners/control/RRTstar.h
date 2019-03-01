#ifndef RRT_RRT_STAR_
#define RRT_RRT_STAR_

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Node.h>
#include <rrt_planners/StateChecker.h>
#include <rrt_planners/NearestNeighborsFLANN.h>
#include <rrt_planners/NearestNeighbors.h>
#include <rrt_planners/planners/Planner.h>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

namespace RRT
{
	class RRTstar : public RRT::Planner
	{
		public:

			RRTstar();
			~RRTstar();

			bool steer(Node* fromNode, Node* toNode, Node* newNode);
			
			bool collisionFree(Node* fromNode, Node* toNode, std::vector<Action>& acts, std::vector<State>& istates, float& motCost);

			std::vector<RRT::Node> solve(float secs);


			void setMaxRange(float range) {
				maxRange_ = range;
				steering_->setMaxRange(range);
			}

			
			void set_useKnearest(bool b) {
				useKnearest_ = b;
			}
			
			void setRewireFactor(float f) {
				rewire_factor_ = f;
			}
			/*
			void set_useFirstPathBiasing(bool b) {
				useFirstPathBiasing_ = b;
			}
			
			void setPathBias(float f) {
				pathBias_ = f;
			}
			
			void setPathBias_stddev(float f) {
				pathBias_stddev_ = f;
			}*/
			
			void getNearestNeighbors(Node* node, std::vector<Node*> &nbrs);
			
			void calculateParamsNearest();
			
			//float motionCost(Node* n1, Node* n2); 
			
			void setTimeStep(float step) {
				steering_->setTimeStep(step);
			}
			
			void setControlSteps(int min_steps, int max_steps) {
				steering_->setMinMaxSteps(min_steps, max_steps);
			}
			
			void setRobotAcc(float linear_acc, float angular_acc) {
				steering_->setAccelerations(linear_acc, angular_acc);
			}
			
			void setSteeringType(int type) {
				steering_->setSteeringType(type);
			}
			
			void setMotionCostType(int type) {
				steering_->setMotionCostType(type);
			}
			
			
			
		

		private:

			//float 				timeStep_; //should be 1/freq  with freq = freq of the controller(15~20Hz)
			//float 				minControlSteps_; //minTime = timeStep*minControlDuration
			//flaot 				maxControlSteps_;

			float 				maxRange_; //max distance to insert a new node

			bool 				useKnearest_;
			double				k_rrt_;
			double 				r_rrt_;
			float  				rewire_factor_;
			
			//bool 				useFirstPathBiasing_;
			
	};
}
#endif
