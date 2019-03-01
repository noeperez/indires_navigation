#ifndef RRT_RRT_
#define RRT_RRT_

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Action.h>
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
	class Rrt : public RRT::Planner
	{
		public:
			Rrt();
			~Rrt();

			bool steer(Node* fromNode, Node* toNode, Node* newNode);

			std::vector<RRT::Node> solve(float secs);
			

			/*void setMaxRange(float range) {
				maxRange_ = range;
			}*/
			
			void setTimeStep(float step) {
				steering_->setTimeStep(step);
			}
			
			void setControlSteps(int min_steps, int max_steps) {
				steering_->setMinMaxSteps(min_steps, max_steps);
			}
			
			void setRobotAcc(float linear_acc, float angular_acc) {
				steering_->setAccelerations(linear_acc, angular_acc);
			}
			
			/*void setAccompanySteer(bool s){
				accompany_steer_ = s;
			}*/


		private:
		
			//bool					accompany_steer_;
		
			//float 				maxRange_; //max distance to insert a new node
			
			//float 				timeStep_; //should be 1/freq  with freq = freq of the controller(15~20Hz)
			//int 				minControlSteps_; //minTime = timeStep*minControlDuration
			//int 				maxControlSteps_;

	};
}
#endif
