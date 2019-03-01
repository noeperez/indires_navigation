#ifndef RRT_ACTION_
#define RRT_ACTION_

//#include <ros/ros.h>

namespace RRT {

	class Action
	{
		public:
			Action();
			Action(float vx, float vy, float vth, unsigned int steps); 
		    ~Action();

			void getAction(float &vx, float &vy, float &vth, unsigned int &steps);
			float getVx();
			float getVy();
			float getVth();
			unsigned int getAction();
			unsigned int getSteps();

		private:
		
			float 		vx_;

			float 		vy_;

			float 		vth_;

			// The number of steps the control is applied for
			unsigned int 	steps_;

	};
}
#endif
