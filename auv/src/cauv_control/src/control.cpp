#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <utility/string.h>
#include <utility/rounding.h>
#include <utility/files.h>
#include <utility/options.h>
#include <utility/make_unique.h>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <std_msgs/Float32.h>
#include <cauv_control/ExternalMotorDemand.h>
#include <cauv_control/PIDParams.h>
#include <cauv_control/PIDTarget.h>

#include "control.h"

#include "pressure_imu.h"
#include "sim_imu.h"

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

using namespace cauv;

ControlLoops::ControlLoops() : 
  m_motor_updates_per_second(5)
{
    ros::NodeHandle nh;
    external_demand_sub = nh.subscribe("control/external_demand", 1, &ControlLoops::onExternalMotorDemand, this);
    motor_pub = nh.advertise<cauv_control::MotorDemand>("control/motors", 1);
    if (!motor_pub)    { throw std::runtime_error("Empty Motor Publisher!"); }
    attitude_pub = nh.advertise<cauv_control::Attitude>("control/attitude", 1);
    if (!attitude_pub) { throw std::runtime_error("Empty Attitude Publisher!"); }
    depth_pub = nh.advertise<std_msgs::Float32>("control/depth", 1);
    if (!depth_pub)    { throw std::runtime_error("Empty Depth Publisher!"); }
}

ControlLoops::~ControlLoops()
{
    stop();
}


void ControlLoops::addCanGate(const std::string &iface)
{
    m_can_gate = boost::make_shared<CANGate>(iface);
}

void ControlLoops::stop()
{
    if (m_motorControlLoopThread.get_id() != boost::thread::id()) {
        m_motorControlLoopThread.interrupt();    
        m_motorControlLoopThread.join();    
    }
}

void ControlLoops::onAttitude(const floatYPR& attitude)
{
    if (bearing_pid->enabled) {
        const float mv = bearing_pid->getMV(attitude.yaw);
        bearing_demand.hbow = mv;
        bearing_demand.hstern = -mv;
    }
    if (pitch_pid->enabled) {
        const float mv = pitch_pid->getMV(attitude.pitch);
        pitch_demand.vbow = -mv;
        pitch_demand.vstern = mv;
    }
    cauv_control::Attitude msg;
    msg.yaw = attitude.yaw;
    msg.pitch = attitude.pitch;
    msg.roll = attitude.roll;
    attitude_pub.publish(msg);
}

void ControlLoops::onDepth(float depth)
{
    if (depth_pid->enabled) {
        const float mv = depth_pid->getMV(depth);
        depth_demand.vbow = mv;
        depth_demand.vstern = mv;
    }
    std_msgs::Float32 msg;
    msg.data = depth;
    depth_pub.publish(msg);
}

void ControlLoops::onExternalMotorDemand(const cauv_control::ExternalMotorDemand::ConstPtr &msg) {
    if (msg->setProp) {external_demand.prop = msg->prop;}
    if (msg->setHbow) {external_demand.hbow = msg->hbow;}
    if (msg->setVbow) {external_demand.vbow = msg->vbow;}
    if (msg->setHstern) {external_demand.hstern = msg->hstern;}
    if (msg->setVstern) {external_demand.vstern = msg->vstern;}
}

void ControlLoops::motorControlLoop()
{
    debug() << "Control loop thread started";
    try {
        while(ros::ok())
        {
            boost::this_thread::interruption_point();
            updateMotorControl();
            if(m_motor_updates_per_second) {
                msleep(1000/m_motor_updates_per_second);
            }
            ros::spinOnce();
        }
    } catch (boost::thread_interrupted&) {
        debug() << "Control loop thread interrupted";
    }

    debug() << "Control loop thread exiting";
}

void ControlLoops::updateMotorControl()
{
    MotorDemand total_demand = external_demand;
    if (depth_pid->enabled) total_demand += depth_demand;
    if (bearing_pid->enabled) total_demand += bearing_demand;
    if (pitch_pid->enabled) total_demand += pitch_demand;

    total_demand.prop   = clamp(-127, total_demand.prop,   127);
    total_demand.hbow   = clamp(-127, total_demand.hbow,   127);
    total_demand.vbow   = clamp(-127, total_demand.vbow,   127);
    total_demand.hstern = clamp(-127, total_demand.hstern, 127);
    total_demand.vstern = clamp(-127, total_demand.vstern, 127);

    if (m_can_gate) {
        m_can_gate->setMotorState(total_demand);
    }

    motor_pub.publish(total_demand);
}

void ControlLoops::addSBG(const std::string& port, int baud_rate, int pause_time)
{
    // start up the SBG IMU
    boost::shared_ptr<sbgIMU> sbg;
    try {
        sbg = boost::make_shared<sbgIMU>(port.c_str(), baud_rate, pause_time);
        sbg->initialise();
        info() << "sbg Connected";
        m_imus.push_back(sbg);
    } catch (sbgException& e) {
        error() << "Cannot connect to sbg: " << e.what ();
    }
}

void ControlLoops::addSimIMU()
{
    boost::shared_ptr<SimIMU> sim = boost::make_shared<SimIMU>();
    sim->addObserver(shared_from_this());
    m_imus.push_back(sim);
}

void ControlLoops::start()
{

    if (m_can_gate) {
        boost::shared_ptr<PressureIMU> psb = boost::make_shared<PressureIMU>();
        m_imus.push_back(psb);
        m_can_gate->addObserver(psb);
        m_can_gate->start();
    }
    else {
        warning() << "CAN not connected. No CAN comms available, so no motor control.";
    }

    if (m_imus.size() > 0) {
        for (auto& imu : m_imus) {
            imu->start();
        }
    }
    else {
        warning() << "IMU not connected. Telemetry not available.";
    }

    depth_lock = boost::make_shared<TokenLock>();
    pitch_lock = boost::make_shared<TokenLock>();
    translate_lock = boost::make_shared<TokenLock>();

    bearing_pid = make_unique<PIDControl>("control/bearing/", translate_lock, true);
    depth_pid = make_unique<PIDControl>("control/depth/", depth_lock, true);
    pitch_pid = make_unique<PIDControl>("control/pitch/", pitch_lock, false);

    motorControlLoop();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Control");

    cauv::Options options("Control loops and IMU bridge");
    namespace po = boost::program_options;
    options.desc.add_options()
        //("xsens,x", po::value<int>()->implicit_value(0), "USB device id of the Xsens")
        ("sbg,b", po::value<std::string>()->implicit_value("/dev/ttyUSB1"), "TTY device for SBG IG500A")
        ("can,c", po::value<std::string>()->implicit_value("can0"), "CAN interface name")
        ("simulation,N", "Run in simulation mode")
      ;
    if (options.parseOptions(argc, argv)) {
        return 0;
    };

    auto vm = options.vm;

    ros::NodeHandle n;
    auto loops = boost::make_shared<cauv::ControlLoops>();

    if (vm.count("simulation")) {
        if (vm.count("xsens") || vm.count("sbg") || vm.count("can")) {
            warning() << "Running in simulation mode, ignoring hardware IMU(s)";
        }
        loops->addSimIMU();
    } else {
        if (vm.count("xsens")) {
            //addXsens(vm["xsens"].as<int>());
        }
        if (vm.count("sbg")){
            loops->addSBG(vm["sbg"].as<std::string>(), 115200, 10);
        }
        if (vm.count("can")){
            loops->addCanGate(vm["can"].as<std::string>());
        }
    }

    loops->start();

    return 0;
}
