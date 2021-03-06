/*
Main source file for the cauv_control node.  This node manages control loops.

Topics:


Parameters:
update_frequency: The rate (in Hz) at which this node updates motor thrust.  


*/

#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <utility/string.h>
#include <utility/rounding.h>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <cauv_control/msg_floatYPR.h>
#include <cauv_cangate/msg_motor_demand.h>

#include "control.h"
#include "pid.h"

using namespace cauv;

inline cauv_cangate::msg_motor_demand& operator+=(cauv_cangate::msg_motor_demand &lhs, const cauv_cangate::msg_motor_demand &rhs)
{
    lhs.fwd_left += rhs.fwd_left;
    lhs.fwd_right += rhs.fwd_right;
    lhs.vert_fore += rhs.vert_fore;
    lhs.vert_aft += rhs.vert_aft;
    lhs.horz_fore += rhs.horz_fore;
    lhs.horz_aft += rhs.horz_aft;

    return lhs;
}

ControlLoops::ControlLoops()
{
    // Initialise ROS-related stuff

    // Subscribe from demand topics
    external_demand_sub = nh.subscribe("cauv_control/external_demand", 1, &ControlLoops::on_external_demand, this);
    attitude_demand_sub = nh.subscribe("cauv_control/attitude_demand", 1, &ControlLoops::on_attitude_demand, this);
    depth_demand_sub = nh.subscribe("cauv_control/depth_demand", 1, &ControlLoops::on_depth_demand, this);

    // Subscribe from sensor topics
    attitude_input_sub = nh.subscribe("cauv_sensors/imu_attitude", 1, &ControlLoops::on_attitude_input, this);
    depth_input_sub = nh.subscribe("cauv_sensors/pressure_depth", 1, &ControlLoops::on_depth_input, this);
    rotation_matrix_input_sub = nh.subscribe("cauv_sensors/imu_rotation_matrix", 1, &ControlLoops::on_rotation_matrix_input, this);

    // Publish to motor demand topic
    motor_demand_pub = nh.advertise<cauv_cangate::msg_motor_demand>("cauv_motor_demand", 1);
    if (!motor_demand_pub) { throw std::runtime_error("Empty Motor Publisher!"); }
    
    // Publish to status/information topics
    attitude_pub = nh.advertise<cauv_control::msg_floatYPR>("cauv_control/info_attitude_motordemand", 1);
    if (!attitude_pub) { throw std::runtime_error("Empty Attitude Publisher!"); }
    depth_pub = nh.advertise<std_msgs::Float32>("cauv_control/info_depth_motordemand", 1);
    if (!depth_pub) { throw std::runtime_error("Empty Depth Publisher!"); }


    // Instantiate and initialise PID loops
    yaw_pid = boost::make_shared<PIDControl>(true);
    depth_pid = boost::make_shared<PIDControl>(true);
    pitch_pid = boost::make_shared<PIDControl>(false);

    //TODO add error checking, reduce repetitive code
    // Kp, Ki, Kd, scale, Ap, Ai, Ad, thr, errorMAX (all doubles)

    // Get yaw pid coefficients from parameter server
    // cauv_control/PID/yaw
    double yaw_Kp, yaw_Ki, yaw_Kd, 
           yaw_Ap, yaw_Ai, yaw_Ad, 
	   yaw_thr, yaw_scale, yaw_errorMAX;

    bool got_yaw_Kp 	= ros::param::get("cauv_control/PID/yaw/Kp", yaw_Kp);
    bool got_yaw_Ki 	= ros::param::get("cauv_control/PID/yaw/Ki", yaw_Ki);
    bool got_yaw_Kd	= ros::param::get("cauv_control/PID/yaw/Kd", yaw_Kd);
    bool got_yaw_Ap 	= ros::param::get("cauv_control/PID/yaw/Ap", yaw_Ap);
    bool got_yaw_Ai 	= ros::param::get("cauv_control/PID/yaw/Ai", yaw_Ai);
    bool got_yaw_Ad 	= ros::param::get("cauv_control/PID/yaw/Ad", yaw_Ad);
    bool got_yaw_thr 	= ros::param::get("cauv_control/PID/yaw/thr", yaw_thr);
    bool got_yaw_scale 	= ros::param::get("cauv_control/PID/yaw/scale", yaw_scale);
    bool got_yaw_errorMAX = ros::param::get("cauv_control/PID/yaw/errorMAX", yaw_errorMAX);

    //Initialise yaw_pid loop from retrieved parameters
    yaw_pid->initialise(yaw_Kp, yaw_Ki, yaw_Kd, yaw_scale, yaw_Ap, yaw_Ai,
                       yaw_Ad, yaw_thr, yaw_errorMAX);
    

    // Get depth pid coefficients from parameter server
    // cauv_control/PID/depth
    double depth_Kp, depth_Ki, depth_Kd, 
           depth_Ap, depth_Ai, depth_Ad, 
	   depth_thr, depth_scale, depth_errorMAX;

    bool got_depth_Kp 	= ros::param::get("cauv_control/PID/depth/Kp", depth_Kp);
    bool got_depth_Ki 	= ros::param::get("cauv_control/PID/depth/Ki", depth_Ki);
    bool got_depth_Kd	= ros::param::get("cauv_control/PID/depth/Kd", depth_Kd);
    bool got_depth_Ap 	= ros::param::get("cauv_control/PID/depth/Ap", depth_Ap);
    bool got_depth_Ai 	= ros::param::get("cauv_control/PID/depth/Ai", depth_Ai);
    bool got_depth_Ad 	= ros::param::get("cauv_control/PID/depth/Ad", depth_Ad);
    bool got_depth_thr 	= ros::param::get("cauv_control/PID/depth/thr", depth_thr);
    bool got_depth_scale 	= ros::param::get("cauv_control/PID/depth/scale", depth_scale);
    bool got_depth_errorMAX = ros::param::get("cauv_control/PID/depth/errorMAX", depth_errorMAX);

    //Initialise depth_pid loop from retrieved parameters
    depth_pid->initialise(depth_Kp, depth_Ki, depth_Kd, depth_scale, depth_Ap, depth_Ai,
                       depth_Ad, depth_thr, depth_errorMAX);


    // Get pitch pid coefficients from parameter server
    // cauv_control/PID/pitch
    double pitch_Kp, pitch_Ki, pitch_Kd, 
           pitch_Ap, pitch_Ai, pitch_Ad, 
	   pitch_thr, pitch_scale, pitch_errorMAX;

    bool got_pitch_Kp 	= ros::param::get("cauv_control/PID/pitch/Kp", pitch_Kp);
    bool got_pitch_Ki 	= ros::param::get("cauv_control/PID/pitch/Ki", pitch_Ki);
    bool got_pitch_Kd	= ros::param::get("cauv_control/PID/pitch/Kd", pitch_Kd);
    bool got_pitch_Ap 	= ros::param::get("cauv_control/PID/pitch/Ap", pitch_Ap);
    bool got_pitch_Ai 	= ros::param::get("cauv_control/PID/pitch/Ai", pitch_Ai);
    bool got_pitch_Ad 	= ros::param::get("cauv_control/PID/pitch/Ad", pitch_Ad);
    bool got_pitch_thr 	= ros::param::get("cauv_control/PID/pitch/thr", pitch_thr);
    bool got_pitch_scale 	= ros::param::get("cauv_control/PID/pitch/scale", pitch_scale);
    bool got_pitch_errorMAX = ros::param::get("cauv_control/PID/pitch/errorMAX", pitch_errorMAX);

    //Initialise pitch_pid loop from retrieved parameters
    pitch_pid->initialise(pitch_Kp, pitch_Ki, pitch_Kd, pitch_scale, pitch_Ap, pitch_Ai,
                       pitch_Ad, pitch_thr, pitch_errorMAX);

}

void ControlLoops::update_motor_demand()
{
    if (depth_pid->enabled) total_demand += depth_demand;
    if (yaw_pid->enabled) total_demand += yaw_demand;
    if (pitch_pid->enabled) total_demand += pitch_demand;

    total_demand.fwd_left   = clamp(-127, total_demand.fwd_left,   127);
    total_demand.fwd_right  = clamp(-127, total_demand.fwd_right,  127);
    total_demand.vert_fore  = clamp(-127, total_demand.vert_fore,  127);
    total_demand.vert_aft   = clamp(-127, total_demand.vert_aft,   127);
    total_demand.horz_fore  = clamp(-127, total_demand.horz_fore,  127);
    total_demand.horz_aft   = clamp(-127, total_demand.horz_aft,   127);

    motor_demand_pub.publish(total_demand);
}

void ControlLoops::on_attitude_demand(const cauv_control::msg_floatYPR::ConstPtr attitude)
{
    target_attitude = *attitude;

    if (yaw_pid->enabled) {
        const float mv = yaw_pid->get_demand(target_attitude.yaw, current_attitude.yaw);
        yaw_demand.horz_fore = mv;
        yaw_demand.horz_aft = -mv;
    }
    if (pitch_pid->enabled) {
        const float mv = pitch_pid->get_demand(target_attitude.pitch, current_attitude.pitch);
        pitch_demand.vert_fore = -mv;
        pitch_demand.vert_aft = mv;
    }

    attitude_pub.publish(target_attitude);
}

void ControlLoops::on_depth_demand(const std_msgs::Float32::ConstPtr depth)
{
    target_depth = depth->data;
    if (depth_pid->enabled) {
        const float mv = depth_pid->get_demand(target_depth, current_depth);

	   depth_demand.fwd_left = mv*current_rotation_matrix.aZ;
        depth_demand.fwd_right = mv*current_rotation_matrix.aZ;
        depth_demand.horz_fore = mv*current_rotation_matrix.bZ;
        depth_demand.horz_aft = mv*current_rotation_matrix.bZ;
        depth_demand.vert_fore = mv*current_rotation_matrix.cZ;
        depth_demand.vert_aft = mv*current_rotation_matrix.cZ;
        
        // This code does not consider the attitude.  
        //depth_demand.vert_fore = mv;
        //depth_demand.vert_aft = mv;
    }
    depth_pub.publish(depth);
}

void ControlLoops::on_external_demand(const cauv_cangate::msg_motor_demand::ConstPtr msg) {

    external_demand.fwd_left = msg->fwd_left;
    external_demand.fwd_right = msg->fwd_right;
    external_demand.vert_fore = msg->vert_fore;
    external_demand.vert_aft = msg->vert_aft;
    external_demand.horz_fore = msg->horz_fore;
    external_demand.horz_aft = msg->horz_aft;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cauv_control");

    ros::NodeHandle nh;

    double update_frequency;
    nh.param("update_frequency", update_frequency, 10.0);
    ros::Rate loop_rate(update_frequency);
    
    boost::shared_ptr<ControlLoops> control_loops = boost::make_shared<ControlLoops>();

    while(ros::ok())
    {
        ros::spinOnce();
        control_loops->update_motor_demand();
        loop_rate.sleep();
    }

    return 0;
}
