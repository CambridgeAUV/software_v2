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

        void onExternalMotorDemand(const cauv_cangate::msg_motor_command& external_demand_sub);

        ros::Publisher motor_pub;
        ros::Publisher attitude_pub;
        ros::Publisher depth_pub;
        ros::Subscriber external_demand_sub;

        MotorDemand external_demand;
        MotorDemand yaw_demand;
        MotorDemand pitch_demand;
        MotorDemand depth_demand;

        unsigned m_motor_updates_per_second;
};

} // namespace cauv
