#ifndef SET_BASE_VELOCITY_H_20160220_
#define SET_BASE_VELOCITY_H_20160220_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <math.h>
#include <time.h>
#include "gecko_msgs/BaseVelocity.h"
#include "set_state/pubState.h"

class SetBaseVelocity
{
public:
  SetBaseVelocity();
  void computeBaseVelocity();
  // Callback function
  void stateCallback(const std_msgs::String::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void publishBaseVelocity();

private:
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, point_cloud_sub_, command_vel_sub_, imu_sub_;
  ros::Publisher  base_velocity_pub_;
  ros::ServiceClient state_client_;

  std::string state_;
  double front_distance_;
  double linear_velocity_, angular_velocity_;
  double roll_, pitch_, yaw_;
  double old_roll_, old_pitch_, old_yaw_;
  std::time_t now_;
  std::time_t past_;
  gecko_msgs::BaseVelocity base_velocity_;
  set_state::pubState state_srv_;

  // Constant
  double WHEEL_RADIUS_;
  double ROBOT_WIDTH_;

  // Helper function
  void stateTracking();
  void stateRotating();
  void stateGetInfo();
  void stateLeaving();
  void stateTeleop();
  inline void moveBase()
  {
    base_velocity_.linear = static_cast<int16_t>(linear_velocity_ / (WHEEL_RADIUS_) * 100);
    base_velocity_.angular = static_cast<int16_t>(angular_velocity_ / (WHEEL_RADIUS_) * 100);
  }
  inline void stopBase()
  {
    base_velocity_.linear  = 0;
    base_velocity_.angular = 0;
  }
};


#endif // SET_BASE_VELOCITY_H_20160220_
