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

    //TODO: Find the correct values to initialise the PID loops
    //yaw_pid->initialise();
    //depth_pid->initialise();
    //pitch_pid->initialise();
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

        // This code does not consider the attitude.  
        depth_demand.vert_fore = mv;
        depth_demand.vert_aft = mv;
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
