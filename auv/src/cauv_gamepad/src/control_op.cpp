#include <ros/ros.h>
#include <cauv_cangate/msg_motor_command.h>
#include <sensor_msgs/Joy.h>

class ControlOp
{
public:
  ControlOp();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int vert_l_, horz_l_, vert_r_, horz_r_;
  double v_scale_, h_scale_;
  ros::Publisher command_pub_;
  ros::Subscriber joy_sub_;
  
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
ControlOp::ControlOp():
  vert_l_(1),
  horz_l_(2),
  vert_r_(3),
  horz_r_(4)
{

  nh_.param("axis_prop_l", vert_l_, vert_l_);
  nh_.param("axis_strafe_l", horz_l_, horz_l_);
  nh_.param("axis_pitch_r", vert_r_, vert_r_);
  nh_.param("axis_turn_r",horz_r_, horz_r_);
  nh_.param("scale_horz", h_scale_, h_scale_);
  nh_.param("scale_vert", v_scale_, v_scale_);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  command_pub_ = nh_.advertise<cauv_cangate::msg_motor_command>("cauv_motor_command", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ControlOp::joyCallback, this);
// %EndTag(SUB)%
}
// %Tag(CALLBACK)%
void ControlOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  cauv_cangate::msg_motor_command motor_command;
  if (joy->axes[5] != 0.0 && joy->axes[5] != 1.0 ){
  	motor_command.vert_fore = v_scale_*(joy->axes[vert_l_] + joy->axes[vert_r_]);
  	motor_command.vert_aft = v_scale_*(joy->axes[vert_l_] - joy->axes[vert_r_]);
  	motor_command.horz_fore = h_scale_*(joy->axes[horz_l_] + joy->axes[horz_r_]);
  	motor_command.horz_aft = h_scale_*(joy->axes[horz_l_] - joy->axes[horz_r_]);
	command_pub_.publish(motor_command);
  }
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_translate");
  ControlOp control_translate;

  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%
  
