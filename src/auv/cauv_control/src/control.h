/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

#include <boost/shared_ptr.hpp>

#include "pid.h"

#include <std_msgs/Float32.h>
#include <cauv_control/msg_floatYPR.h>
#include <cauv_cangate/msg_motor_demand.h>

namespace cauv{

class ControlLoops
{
    public:
        ControlLoops();
        ~ControlLoops(){}
        void update_motor_demand();


    protected:
        boost::shared_ptr<PIDControl> yaw_pid;
        boost::shared_ptr<PIDControl> pitch_pid;
        boost::shared_ptr<PIDControl> depth_pid;

    private:
        // Callback functions for demands
        void on_external_demand(const cauv_cangate::msg_motor_demand::ConstPtr msg);
        void on_attitude_demand(const cauv_control::msg_floatYPR::ConstPtr attitude);
        void on_depth_demand(const std_msgs::Float32::ConstPtr depth);

        // Callback functions for sensor inputs
        inline void on_attitude_input(const cauv_control::msg_floatYPR::ConstPtr attitude){ current_attitude = *attitude; }
        inline void on_depth_input(const std_msgs::Float32::ConstPtr depth){ current_depth = depth->data; }

        ros::NodeHandle nh;

        // Picks up data from the sensor topics
        ros::Subscriber attitude_input_sub;
        ros::Subscriber depth_input_sub;

        // Retrieves the YPR, depth and external demands
        ros::Subscriber external_demand_sub;
        ros::Subscriber attitude_demand_sub;
        ros::Subscriber depth_demand_sub;

        // Publishes to the cauv_cangate node
        ros::Publisher motor_demand_pub;

        // Publishers, for informative purposes only
        ros::Publisher attitude_pub;
        ros::Publisher depth_pub;
        

        cauv_control::msg_floatYPR current_attitude;
        cauv_control::msg_floatYPR target_attitude;

        double current_depth;
        double target_depth;

        cauv_cangate::msg_motor_demand external_demand;
        

        cauv_cangate::msg_motor_demand yaw_demand;
        cauv_cangate::msg_motor_demand pitch_demand;
        cauv_cangate::msg_motor_demand depth_demand;

        cauv_cangate::msg_motor_demand total_demand;
};

} // namespace cauv
