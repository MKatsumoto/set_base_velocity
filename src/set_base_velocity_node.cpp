//#define DEBUG

#include "set_base_velocity/SetBaseVelocity.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_base_velocity_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate_hz(10);
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
