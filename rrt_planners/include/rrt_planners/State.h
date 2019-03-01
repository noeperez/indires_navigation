#ifndef RRT_STATE_
#define RRT_STATE_


namespace RRT {


	/**
	*	\brief Class representing a state in 6DoF
	*
	*/
	class State
	{
		public:
			State();
			State(float x, float y, float z = 0.0, float yaw = 0.0, float roll = 0.0, float pitch = 0.0, float lv = 0.0, float av = 0.0);
		    ~State();
	
			void getState(float &x, float &y, float &z, float &yaw, float &roll, float &pitch, float &lv, float &av);
			float getX();
			float getY();
			float getZ();
			float getYaw();
			float getRoll();
			float getPitch();
			float getLinVel();
			float getAngVel();

			void setX(float x);
			void setY(float y);
			void setZ(float z);
			void setYaw(float yaw);
			void setRoll(float roll);
			void setPitch(float pitch);
			void setLv(float lv);
			void setAv(float av);

		private:
		
			float 		x_;

			float 		y_;

			float		z_;

			float 		yaw_;

			float		roll_;

			float		pitch_;

			float 		lin_vel_;

			float 		ang_vel_;

	};
}
#endif
