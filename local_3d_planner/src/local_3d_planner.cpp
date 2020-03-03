/*********************************************************************
*
* Author: Noé Pérez Higueras
*********************************************************************/

#include <local_3d_planner/local_3d_planner.h>
//#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>



#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>

using namespace std;
//using namespace costmap_2d;

namespace local_3d_planner{

  /*void UpoPlanner::reconfigure(SimpleLocalPlannerConfig &cfg)
  {
      SimpleLocalPlannerConfig config(cfg);

      boost::mutex::scoped_lock l(configuration_mutex_);
      
		acc_lim_trans_ = config.max_trans_acc;
		acc_lim_rot_ = config.max_rot_acc;
		max_vel_x_ = config.max_trans_vel;
		min_vel_x_ = config.min_trans_vel;
		max_vel_th_ = config.max_rot_vel;
		min_vel_th_ = config.min_rot_vel;
		min_in_place_vel_th_ = config.min_in_place_rot_vel;
		goal_lin_tolerance_ = config.xy_goal_tolerance;
		goal_ang_tolerance_ = config.yaw_goal_tolerance;
		wp_tolerance_ = config.wp_tolerance; 
		sim_time_ = config.sim_time;
		sim_granularity_ = config.sim_granularity;
		angular_sim_granularity_ = config.angular_sim_granularity;
		dwa_ = config.sample_angular_vels;
		//printf("\nPure Planner Reconfigure. new wp_tolerance: %.2f\n", wp_tolerance_);
  }*/



  Local3DPlanner::Local3DPlanner(std::string name, tf::TransformListener* tf,
			local_3d_planner::OdometryHelperRos* oh,
			//std::vector<geometry_msgs::Point> footprint_spec,
			double robot_radius,
			double local_area_radius,
			double controller_freq,
			double max_trans_vel, double min_trans_vel,
			double max_rot_vel, double min_rot_vel,
			double min_in_place_rot_vel,
			double max_trans_acc, double max_rot_acc,
			double yaw_goal_tolerance, double xy_goal_tolerance,
			double wp_tolerance, double sim_time,
			double sim_granularity, double angular_sim_granularity, bool dwa)
  {

		//costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);

		//printf("\n\n\n---FOOTPRINT----\n");
		//printf("inscribed_radius: %.3f, circumscribed_radius: %.3f\n", inscribed_radius_, circumscribed_radius_);
		//printf("Footprint_specs:\n");
		//for(unsigned int i = 0; i<footprint_spec_.size(); i++)
		//{
		//	printf("point %u: x=%.3f, y=%.3f\n", (i+1), footprint_spec_[i].x, footprint_spec_[i].y); 
		//}   
		//printf("\n\n"); 

		controller_freq_ = controller_freq;
		goal_reached_ = false;

    	//For pure-pursuit
    	running_ = false;
		new_plan_ = false;
		wp_index_ = -1;	
    
    	acc_lim_trans_ = max_trans_acc;
		acc_lim_rot_ = max_rot_acc;
		max_vel_x_ = max_trans_vel;
		min_vel_x_ = min_trans_vel;
		max_vel_th_ = max_rot_vel;
		min_vel_th_ = min_rot_vel;
		min_in_place_vel_th_ = min_in_place_rot_vel;
		goal_lin_tolerance_ = xy_goal_tolerance;
		goal_ang_tolerance_ = yaw_goal_tolerance;
		wp_tolerance_ = wp_tolerance; 
		sim_time_ = sim_time;
		sim_granularity_ = sim_granularity;
		angular_sim_granularity_ = angular_sim_granularity;	

		dwa_ = dwa;
		robot_radius_ = robot_radius;
		local_area_radius_ = local_area_radius;


		collision_detector_ = new CollisionDetection(name, tf, oh, max_vel_x_, max_vel_th_, acc_lim_trans_, 
				acc_lim_rot_, sim_time_, robot_radius_, local_area_radius_, sim_granularity_);
	

  }




  Local3DPlanner::~Local3DPlanner(){
	delete collision_detector_;
  }






  bool Local3DPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan)
  {
		goal_reached_ = false;

		// Copy new plan 
		global_plan_.clear();
		global_plan_.resize(new_plan.size());
		for(unsigned int i = 0; i < new_plan.size(); ++i)
		{
			global_plan_[i] = new_plan[i];
		}
		
		// Check plan size
		if(global_plan_.size() == 0)
		{
			running_ = false;
			wp_index_ = -1;
			ROS_WARN("New local plan size = 0!");
			return true;
		}
		
		// Set the way-point index to the first point of the path
		wp_index_ = 0;
		running_ = true;
		new_plan_ = true;
		
		// Set plan goal point
		geometry_msgs::PoseStamped& goal_pose = global_plan_[global_plan_.size()-1];
		goal_x_ = goal_pose.pose.position.x;
		goal_y_ = goal_pose.pose.position.y;
		goal_z_ = goal_pose.pose.position.z;
		goal_t_ = tf::getYaw(goal_pose.pose.orientation);
		
		// Set the plan starting point
		geometry_msgs::PoseStamped& start_pose = global_plan_[0];
		start_x_ = start_pose.pose.position.x;
		start_y_ = start_pose.pose.position.y;
		start_z_ = start_pose.pose.position.z;
		start_t_ = tf::getYaw(start_pose.pose.orientation);
		
		
		return true;
  }





  bool Local3DPlanner::isGoalReached()
  {
		if(goal_reached_) {
			goal_reached_ = false; //we reset the flag
			return true;
		}
		return goal_reached_;
  }

  void Local3DPlanner::resetGoal() {
		goal_reached_ = false;
  }



  //given the current state of the robot, find a good control command
  bool Local3DPlanner::findBestAction(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
      geometry_msgs::Twist& cmd_vel)
  {

	//ros::WallTime t1 = ros::WallTime::now();

	//boost::mutex::scoped_lock l(configuration_mutex_);
	
	goal_reached_ = false;
	double vx, vy = 0.0, vt;
	
	// Check we have a path and we are running
	if(!running_)
	{
		vx = 0.0;
		vt = 0.0;
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = vt;
		printf("FindBestAction. PurePursuit waiting for a path\n");
		return true;
	}
		
	// Get current robot position and velocity in X, Y and Theta (odom)
	float rx, ry, rz, rt, rvx, rvy, rvt;
	rx = global_pose.getOrigin().getX();
	ry = global_pose.getOrigin().getY();
	rz = global_pose.getOrigin().getZ();
	rt = tf::getYaw(global_pose.getRotation());
	rvx = global_vel.getOrigin().getX();
	rvy = global_vel.getOrigin().getY();
	rvt = tf::getYaw(global_vel.getRotation());
	

	// Check if we are close enough to the goal
	double dist_goal = sqrt((rx-goal_x_)*(rx-goal_x_)+(ry-goal_y_)*(ry-goal_y_)); //+(rz-goal_z_)*(rz-goal_z_));
	double dist_start = sqrt((rx-start_x_)*(rx-start_x_)+(ry-start_y_)*(ry-start_y_)); //+(rz-start_z_)*(rz-start_z_)); 

	
	if(dist_goal < goal_lin_tolerance_)
	{
		printf("FindBestAction. dist_goal: %.2f. rx:%.2f, ry:%.2f, rz:%.2f, goalx:%.2f, goaly:%.2f, goalz: %.2f\n", dist_goal, rx, ry, rz, goal_x_, goal_y_, goal_z_);
		// Stop the robot
		vx = 0.0;
		vy = 0.0;
		
		// Rotate at minumin velocity until reaching the goal angle
		if(fabs(goal_t_-rt) < goal_ang_tolerance_)
		{
			vt = 0.0;
			running_ = false;
			goal_reached_ = true; 
		}
		/*else if(goal_t_ > rt)
			vt = min_in_place_vel_th_;
		else
			vt = -min_in_place_vel_th_;
		*/
		// Modified by Noé
		else {
			float ang_diff = goal_t_ - rt;
			ang_diff = normalizeAngle(ang_diff, -M_PI, M_PI);
			if(ang_diff > 0.0)
				vt = min_in_place_vel_th_;
			else
				vt = -min_in_place_vel_th_;
		}
		
		//Added by Noé
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = vy;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = vt;
		//ros::WallTime t2 = ros::WallTime::now();
		//double secs = (t2-t1).toSec();
		//printf("\n\nFindBestPathTime: %f seconds\n\n", secs);
		printf("FindBestAction. goal reached, sending zero velocity\n");
		return true;
		
	}

	// Do we have a new plan? get the closest point into the plan
	if(new_plan_)
	{
		new_plan_ = false;
		double dist;
		wp_index_ = 0;
		for(int i=global_plan_.size()-1; i>=0; i--)
		{
			double wpx = global_plan_[i].pose.position.x;
			double wpy = global_plan_[i].pose.position.y;
			double wpz = global_plan_[i].pose.position.z;
			dist = sqrt((rx-wpx)*(rx-wpx)+(ry-wpy)*(ry-wpy)); //+(rz-wpz)*(rz-wpz));
			if(dist < wp_tolerance_)
			{
				wp_index_ = i;
				break;
			}
		}
	}
	
	// Get current way-point in the path
	double wpx = global_plan_[wp_index_].pose.position.x;
	double wpy = global_plan_[wp_index_].pose.position.y;
	double wpz = global_plan_[wp_index_].pose.position.z;
	
	// Is this way-point still valid?
	double dist_swp = sqrt((rx-wpx)*(rx-wpx)+(ry-wpy)*(ry-wpy)); //+(rz-wpz)*(rz-wpz)); 
	while(dist_swp < wp_tolerance_ && wp_index_ < (int)global_plan_.size()-1)
	{
		wp_index_++;
		wpx = global_plan_[wp_index_].pose.position.x;
		wpy = global_plan_[wp_index_].pose.position.y;
		//wpz = global_plan_[wp_index_].pose.position.z;
		dist_swp = sqrt((rx-wpx)*(rx-wpx)+(ry-wpy)*(ry-wpy)); //+(rz-wpz)*(rz-wpz));
	}

	// Transform way-point into local robot frame and get desired x,y,theta
	double dx = (wpx-rx)*cos(rt) + (wpy-ry)*sin(rt);
	double dy =-(wpx-rx)*sin(rt) + (wpy-ry)*cos(rt);
	double dt = atan2(dy, dx);
	//double dz = (wpz - rz);  //???
	//features_->transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
	

	double incr = 1/controller_freq_;

	// Check if we need rotation in place before moving the robot to reach the way-point
	if(fabs(dt) > 1.3) // 0.87 0.7~41º  0.79~45º
	{
		//vx = rvx-incr;  
		//if(vx < 0.0)
		//	vx = 0.0;
		vx = 0;
		vy = 0.0;
		vt = min_in_place_vel_th_;
		if(dt < 0.0)
			vt = -min_in_place_vel_th_;
			
		//printf("place_vt: %.2f\n", vt);
	}
	else // Select the linear and angular velocities to reach the way-point
	{
		// Compute actions depending to the distance to the goal and the starting points
		double dist_th = 1.5;
		if(dist_goal < dist_th)
		{
			vx = min_vel_x_ + (max_vel_x_ - min_vel_x_)*dist_goal/dist_th;
			vy = 0.0;
			vt = min_vel_th_ + (max_vel_th_ - min_vel_th_)*dist_goal/dist_th;
			//--added by noe
			if(dt < 0.0)
				vt *= -1;
		}
		/*else if(dist_start < dist_th)
		{
			vx = min_vel_x_ + (max_vel_x_ - min_vel_x_)*dist_start/dist_th;
			vy = 0.0;
			vt = min_vel_th_ + (max_vel_th_ - min_vel_th_)*dist_start/dist_th;
		}*/
		else
		{
			
			vx = max_vel_x_*(0.1 + exp(-fabs(dt))); // * tanh(4*dist_swp); //max_vel_x_;
			if(vx > max_vel_x_)
				vx = max_vel_x_;
			vy = 0.0;
			vt = max_vel_th_* dt; //max_vel_th_;
		}

		//if(vx > rvx+incr)
		//	vx = rvx+incr;
		
		// Set the sign of the commanded angular velocity and reset in case of small variations
		//if(dt < 0.0) //commented by Noé
		//	vt *= -1;
		if(fabs(dt) < 0.1)
			vt = 0.0;
	}
	
	// Check if the action collide with an obstacle
	double px=0.0, py=0.0, pz=0.0, pth=0.0;
	bool valid = true;
	if(vx != 0.0) {
		//valid = checkTrajectory(rx, ry, rt, rvx, rvy, rvt, vx, vy, vt, px, py, pth);
		ros::WallTime t_check1 = ros::WallTime::now();
		valid = collision_detector_->checkTraj(rvx, rvy, rvt, vx, vy, vt, px, py, pz, pth);
		ros::WallTime t_check2 = ros::WallTime::now();
		double secs = (t_check2-t_check1).toSec();
		//printf("\n\nCheckTraj time: %f seconds\n\n", secs);
	}
	if(valid /*|| fabs(vx) < 0.0001*/)	
	{
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = vy;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = vt;
		//ros::WallTime t2 = ros::WallTime::now();
		//double secs = (t2-t1).toSec();
		//printf("\n\nFindBestPathTime: %f seconds\n\n", secs);
		//printf("FindBestAction. validTraj found, vx:%.2f, vy:%.2f, vth:%.2f\n", vx, vy, vt);
		return true;
	}

	// Try to find a valid command by sampling vels
	else if(dwa_) 
	{
		//printf("Dentro de DWA!!!!!!!\n");
		float vt_orig = vt;
		float vx_orig = vx;

		float ang_vel_inc = 0.1;
		float lin_vel_dec = 0.1;

		vels_ best_vels;
		best_vels.vel_x = -1.0;
		best_vels.vel_y = vy;

		//Linear vel
		for(unsigned int l=0; l <= 3; l++) 
		{

			vx = vx_orig - (lin_vel_dec*l);
			if(vx < 0.1)
				continue;
			
			best_vels.vel_x = vx;
		
			//Angular vel
			for(unsigned int v=1; v <= 4; v++)
			{
				//To the right
				vt = vt_orig + (ang_vel_inc * v);
				if(fabs(vt) > max_vel_th_)
					vt = max_vel_th_;
				
				bool valid1 = collision_detector_->checkTraj(rvx, rvy, rvt, vx, vy, vt, px, py, pz, pth);
				double d1 = 0.0;
				if(valid1) {
					d1 = sqrt((dx-px)*(dx-px) + (dy-py)*(dy-py)); // + (dz-pz)*(dz-pz));
					best_vels.vel_th = vt;
				}

				//to the left
				vt = vt_orig - (ang_vel_inc * v);
				if(fabs(vt) > max_vel_th_)
					vt = -max_vel_th_;
				
				bool valid2 = collision_detector_->checkTraj(rvx, rvy, rvt, vx, vy, vt, px, py, pz, pth);
				if(valid2) {

					//If both commands are valid,
					//chose the closest to the path.
					if(valid1)
					{
						double d2 = sqrt((dx-px)*(dx-px) + (dy-py)*(dy-py)); // + (dz-pz)*(dz-pz));
						if(d2 < d1)
						{
							best_vels.vel_th = vt;
						}
					}
				}
				if(valid1 || valid2)
				{
					cmd_vel.linear.x = best_vels.vel_x;
					cmd_vel.linear.y = best_vels.vel_y;
					cmd_vel.linear.z = 0.0;
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = best_vels.vel_th;
					//printf("\nValid cmd found! vx:%.2f, vth:%.2f\n", vx, vt);
					//ros::WallTime t2 = ros::WallTime::now();
					//double secs = (t2-t1).toSec();
					//printf("\n\nFindBestPathTime: %f seconds\n\n", secs);
					//printf("FindBestAction. Valid Traj found with DWA. vx:%.2f, vy:%.2f, vth:%.2f\n", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
					return true;
				}

			}
		}
	} 
	
	//If still a valid command is not found, try to rotate in the spot
	if(dt > 0.09 || dt < -0.09) //0.09rad~5º
	{
		//printf("The robot should rotate on the spot\n");
		vx = 0.0;
		vy = 0.0;
		vt = min_in_place_vel_th_;
		if(dt < 0.0)
			vt = -min_in_place_vel_th_;
				
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = vy;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = vt;
		//ros::WallTime t2 = ros::WallTime::now();
		//double secs = (t2-t1).toSec();
		//printf("\n\nFindBestPathTime: %f seconds\n\n", secs);
		printf("FindBestAction. Rotating in the spot vx:%.2f, vy:%.2f, vth:%.2f\n", vx, vy, vt);
		return true;
	} else {
		// Stop the robot
		//printf("The robot should stop\n");
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;
		//ros::WallTime t2 = ros::WallTime::now();
		//double secs = (t2-t1).toSec();
		//printf("\n\nFindBestPathTime: %f seconds\n\n", secs);
		printf("FindBestAction. No valid command was found\n");
		return false;
		//return true;
	}
		
  }


  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  /*double UpoPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }*/


};

