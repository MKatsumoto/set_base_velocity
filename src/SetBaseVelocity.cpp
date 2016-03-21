#include "set_base_velocity/SetBaseVelocity.h"
#include <tf/transform_datatypes.h>


SetBaseVelocity::SetBaseVelocity()
  : roll_(0), pitch_(0), yaw_(0), old_roll_(0), old_pitch_(0), old_yaw_(0), dynamixel_position_(0)
{
  state_ = "Waiting";
  state_sub_ = nh_.subscribe("state", 50, &SetBaseVelocity::stateCallback, this);
  point_cloud_sub_ = nh_.subscribe("cloud", 50, &SetBaseVelocity::pointCloudCallback, this);
  command_vel_sub_ = nh_.subscribe("cmd_vel", 50, &SetBaseVelocity::commandVelocityCallback, this);
  imu_sub_ = nh_.subscribe("imu/data", 50, &SetBaseVelocity::imuCallback, this);
  dynamixel_sub_ = nh_.subscribe("swing_dyna_controller/state", 50, &SetBaseVelocity::dynamixelCallback, this);
  base_velocity_pub_ = nh_.advertise<gecko_msgs::BaseVelocity>("base_velocity", 50);
  state_client_ = nh_.serviceClient<set_state::pubState>("pubState");
}

SetBaseVelocity::~SetBaseVelocity()
{
  stopBase();
  publishBaseVelocity();
}

void SetBaseVelocity::computeBaseVelocity()
{
  if(state_ == "Normal" || state_ == "Following" || state_ == "Moving") moveBase();
  else if(state_ == "Scanning" || state_ == "Stop_temp" || state_ == "Waiting") stopBase();
  else if(state_ == "Tracking") stateTracking();
  else if(state_ == "Rotating") stateRotating();
  else if(state_ == "Scanning") stopBase();
  else if(state_ == "GetInfo")  stateGetInfo();
  else if(state_ == "Leaving")  stateLeaving();
  else stopBase();
}

/*!
 * \brief Get state from state message.
 * \param msg
 */
void SetBaseVelocity::stateCallback(const std_msgs::String::ConstPtr &msg)
{
  state_ = msg -> data;
}


/*!
 * \brief Compute minimum front distance between robot and obstacles
 *        based on point cloud data w.r.t. robot frame.
 *        This callback function may be heavy.
 * \param msg
 *
 * TODO: Improve algorithm (heavy process ...)
 */
void SetBaseVelocity::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  double dist_min = 10000;	//適当な大きい数で初期化
  for(int i = 0; i < (int)msg->points.size(); i++){
    if(msg->points[i].x >= 0.1){ //ロボットの中心より10cm以上前方
      if( (msg->points[i].y > -(ROBOT_WIDTH_/2+0.1)) && (msg->points[i].y < +(ROBOT_WIDTH_/2+0.1)) )
      {
        if(msg->points[i].x < dist_min)
        {
          dist_min = msg->points[i].x;
        }
      }
    }
  }
  front_distance_ = dist_min;
}


/*!
 * \brief Get command velocity from command velocity message.
 * \param msg
 */
void SetBaseVelocity::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  command_velocity_ = *msg;
}

/*!
 * \brief Get imu data and convert to RPY.
 * \param msg
 */
void SetBaseVelocity::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q); // Quaternion msgからtf::Quaternionに変換
  tf::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);	// tf::Quaternionからroll, pitch, yawを取得
}


void SetBaseVelocity::dynamixelCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
  dynamixel_position_ = msg->current_pos;
}


void SetBaseVelocity::publishBaseVelocity()
{
  base_velocity_pub_.publish(base_velocity_);
}





// ------------------------------------------------------------------------------

/*!
 * \brief SetBaseVelocity::stateTracking
 *
 * TODO: Modification
 */
void SetBaseVelocity::stateTracking()
{
  stopBase();
  old_roll_  = roll_;
  old_pitch_ = pitch_;
  old_yaw_   = yaw_;
  state_srv_.request.state = "Rotating";
  state_client_.call(state_srv_); // Rotatingモードへ
}


/*!
 * \brief SetBaseVelocity::stateRotating
 *
 * TODO: 左手法のテスト
 */

void SetBaseVelocity::stateRotating()
{
  static const int ROTATION_VELOCITY = -100;
  if(fabs(yaw_ - old_yaw_) < fabs(dynamixel_position_)) // [rad]
  {
    base_velocity_.linear  = 0;
    base_velocity_.angular = (dynamixel_position_ > 0) ? ROTATION_VELOCITY : - ROTATION_VELOCITY;
  }
  else
  {//ある程度回転したら
    state_srv_.request.state = "Scanning";
    state_client_.call(state_srv_); // Scanningモードへ
  }

}


void SetBaseVelocity::stateGetInfo()
{
  stopBase();
  old_roll_  = roll_;
  old_pitch_ = pitch_;
  old_yaw_   = yaw_;
  std::time(&past_);  // get current time
}


static const float LEAVING_DURATION_SEC = 20.0;
void SetBaseVelocity::stateLeaving()
{
  moveBase();
  std::time(&now_); // get current time
  double leaving_time = difftime(now_, past_);
  if (leaving_time > LEAVING_DURATION_SEC)
  {
    state_srv_.request.state = "Following";
    state_client_.call(state_srv_); // Followingモードへ
  }
}
