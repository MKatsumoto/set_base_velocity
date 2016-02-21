#define DEBUG

#include "set_base_velocity/SetBaseVelocity.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_base_velocity_node");
  ros::NodeHandle nh;
  int CONTROL_FREQUENCY;
  if(!nh.getParam("/control_frequency", CONTROL_FREQUENCY))
  {
    ROS_ERROR("/control_frequency_hz is not defined!\n");
  }
  ros::Rate loop_rate_hz(CONTROL_FREQUENCY);
  SetBaseVelocity set_base_velocity;
  while(ros::ok())
  {
    set_base_velocity.computeBaseVelocity();
    set_base_velocity.publishBaseVelocity();
    ros::spinOnce();
    loop_rate_hz.sleep();
  }
  return 0;
}
