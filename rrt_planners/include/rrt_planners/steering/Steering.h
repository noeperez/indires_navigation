#ifndef RRT_STEERING_
#define RRT_STEERING_


#include <vector>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <rrt_planners/StateSpace.h>
#include <rrt_planners/State.h>
#include <rrt_planners/Action.h>
#include <rrt_planners/Node.h>


namespace RRT
{
	class Steering
	{
		public:
			Steering();
			Steering(StateSpace* sp);
			Steering(StateSpace* sp, float max_range);
			Steering(StateSpace* sp, float tstep, int minSteps, int maxSteps, float lAccMax, float aAccMax);
			Steering(StateSpace* sp, float max_range, float tstep, int minSteps, int maxSteps, float lAccMax, float aAccMax);
			~Steering();
			
			//Non-dynamic steering
			State* simpleSteer(State* fromState, State* toState, std::vector<State>& istates);
			//Non-dynamic steering for collision checking
			bool simpleCollisionFree(State* fromState, State* toState, std::vector<State>& istates); 

			State* simple3dSteer(State* fromState, State* toState, std::vector<State>& istates);
			bool simple3dCollisionFree(State* fromState, State* toState, std::vector<State>& istates); 

			bool rrt_3d_steer(Node* fromNode, Node* toNode, Node* newNode);
			
			//2 dimensions, only one lv and one av
			bool rrt_steer(Node* fromNode, Node* toNode, Node* newNode);
			bool rrt_collisionFree(Node* fromNode, Node* toNode, Node& out);
			//bool accompany_steer(Node* fromNode, Node* toNode, Node* newNode);
			
			//Steering for 2 dimensions (x, y)
			bool steer2(Node* fromNode, Node* toNode, Node* newNode);
			bool collisionFree2(Node* fromNode, Node* toNode, std::vector<Action>& acts, std::vector<State>& istates, float& motCost);
			
			//Steering for 3 dimensions (x, y, yaw)
			bool steer3(Node* fromNode, Node* toNode, Node* newNode);
			bool collisionFree3(Node* fromNode, Node* toNode, std::vector<Action>& acts, std::vector<State>& istates, float& motCost);
			
			//Propagate one step
			State* propagateStep(State* st, float lv, float av);
			
			//Motion cost between two intermediate points
			float motionCost(Node* n1, Node* n2); 
			
			
			float distance(State* s1, State* s2, int type);
			
			
			void setStateSpace(StateSpace* ss) {
				space_ = ss;
			}
			
			void setAccelerations(float lin_acc, float ang_acc) {
				maxLinearAcc_ = lin_acc;
				maxAngularAcc_ = ang_acc;
			}
			
			void setTimeStep(float t) {
				timeStep_ = t;
			}
			
			void setMinMaxSteps(int min, int max) {
				minControlSteps_ = min;
				maxControlSteps_ = max;
			}
			
			void setMaxRange(float r) {
				maxRange_ = r;
			}
			
			void setSteeringType(int type) {
					steeringType_ = type;
			}
			
			void setMotionCostType(int type) {
					motionCostType_ = type;
			}
			
			float normalizeAngle(float val, float min, float max);
			
			
			
			void setSteeringParams(float kp, float kv, float ka, float ko);
			
			
			
		private:
		
			StateSpace* 	space_;
		
			float 			maxRange_; //max distance to insert a new node
			
			
			float 			timeStep_; //should be 1/freq  with freq = freq of the controller(15~20Hz)
			int 			minControlSteps_; //minTime = timeStep*minControlSteps
			int 			maxControlSteps_;
		
			// Accelerations
			float 			maxLinearAcc_;
			float 			maxAngularAcc_;
			
			//maximum variation of velocities in one step
			float           max_lv_var_;
			float           max_av_var_;
			
			int 			steeringType_;
			int				motionCostType_;
			
			//Steering parameters
			float 			kp_;
			float			kv_;
			float 			ka_;
			float 			ko_;
			
	};
	
}
#endif
