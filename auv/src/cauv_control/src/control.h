/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once

namespace cauv{

class ControlLoops
{
    public:
        ControlLoops();
        ~ControlLoops();

        void start();
        void stop();

    protected:
        PIDControl yaw_pid;
        PIDControl pitch_pid;
        PIDControl depth_pid;

    private:
        boost::thread m_motorControlLoopThread;
        
        void motorControlLoop();
        void updateMotorControl();

        void onExternalMotorDemand(const cauv_cangate::msg_motor_demand& external_demand_sub);

        ros::Publisher motor_pub;
        ros::Publisher attitude_pub;
        ros::Publisher depth_pub;
        ros::Subscriber external_demand_sub;

        cauv_cangate::msg_motor_demand external_demand;
        cauv_cangate::msg_motor_demand yaw_demand;
        cauv_cangate::msg_motor_demand pitch_demand;
        cauv_cangate::msg_motor_demand depth_demand;
        cauv_cangate::msg_motor_demand total_demand;

        unsigned m_motor_updates_per_second;
};

} // namespace cauv
