/*********************************************************************
*
* Based on the odometry_helper_ros.h of the base_local_planner of ROS.
* Modified by: Noé Pérez Higueras
*********************************************************************/

#ifndef ODOMETRY_HELPER_ROS2_H_
#define ODOMETRY_HELPER_ROS2_H_

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
//#include <boost/thread.hpp>
#include <mutex>
#include <thread>

namespace local_3d_planner {

class OdometryHelperRos {
public:

  /** @brief Constructor.
   * @param odom_topic The topic on which to subscribe to Odometry
   *        messages.  If the empty string is given (the default), no
   *        subscription is done. */
  OdometryHelperRos(std::string odom_topic = "");
  ~OdometryHelperRos() {}

  /**
   * @brief  Callback for receiving odometry data
   * @param msg An Odometry message
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void getOdom(nav_msgs::Odometry& base_odom);

  void getRobotVel(tf::Stamped<tf::Pose>& robot_vel);

  /** @brief Set the odometry topic.  This overrides what was set in the constructor, if anything.
   *
   * This unsubscribes from the old topic (if any) and subscribes to the new one (if any).
   *
   * If odom_topic is the empty string, this just unsubscribes from the previous topic. */
  void setOdomTopic(std::string odom_topic);

  /** @brief Return the current odometry topic. */
  std::string getOdomTopic() const { return odom_topic_; }

private:

	//odom topic
	std::string odom_topic_;

	// we listen on odometry on the odom topic
	ros::Subscriber odom_sub_;
	nav_msgs::Odometry base_odom_;
	//boost::mutex odom_mutex_;
	std::mutex odom_mutex_;
	// global tf frame id
	std::string frame_id_; ///< The frame_id associated this data
};

} /* namespace local_3d_planner */
#define CHUNKY 1
#endif /* ODOMETRY_HELPER_ROS2_H_ */
