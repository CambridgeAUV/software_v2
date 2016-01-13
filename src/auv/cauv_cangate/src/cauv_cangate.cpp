/*
cauv_cangate ROS node:

Interfaces with CAN devices (motor control board and pressure sensors)

Reads information from pressure sensors and publishes it on the
cauv_pressure_status ROS topic.

Subscribes to the cauv_motor_command topic and sends the commands
to the motor control board.  
*/

#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"

#include <cauv_cangate/msg_motor_demand.h>
#include <cauv_cangate/msg_pressure_status.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <std_msgs/Float64.h>

//added the class pressure_depth_converter
//needs to include cauv_control::DepthCalibration
class CanGate
{
    public:

    CanGate(): ifname("can0"), forward_pressure(0), back_pressure(0)
    {
	 	sub_motor_command = mynode.subscribe("cauv_motor_demand", 1, &CanGate::cauv_motor_send_command, this);
	 	pub_depth_status = mynode.advertise<std_msgs::Float64>("cauv_cangate/cauv_depth_status", 1);

	 	retrieve_calibrations(mynode);

	 	if((mysocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
		{
			ROS_ERROR("Error while opening CAN socket");
			throw std::runtime_error("Error while opening CAN socket");
		}

		strcpy(ifr.ifr_name, ifname.c_str());
		ioctl(mysocket, SIOCGIFINDEX, &ifr);

		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex; 

		ROS_INFO("%s at index %d\n", ifname.c_str(), ifr.ifr_ifindex);

		if(bind(mysocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
		{
			ROS_ERROR("Error in CAN socket bind");
			throw std::runtime_error("Error in CAN socket bind");
		}
    }

    ~CanGate()
    {
		close(mysocket);
    }

    void get_can_frame()
    {
        // TODO:
        // This read is always blocking, thus the process *never* answers to SIGTERM.  
        // It's not that annoying, but if someone is interested in solving this, please do. 
		read(mysocket, &last_frame_in_buffer, sizeof(struct can_frame));
    }

    void parse_frame()
    {
		if (last_frame_in_buffer.can_id == 11)  // This frame comes from a pressure sensor
		{
			publish_pressure();
		}
    }

    void publish_pressure()
    {
		if(last_frame_in_buffer.data[4] == 0)
		{
			memcpy(&forward_pressure, &last_frame_in_buffer.data, last_frame_in_buffer.can_dlc);

		}
		
		if(last_frame_in_buffer.data[4] == 1)
		{
			memcpy(&back_pressure, &last_frame_in_buffer.data, last_frame_in_buffer.can_dlc);
		}
		
		depth.data = calc_depth();

		pub_depth_status.publish(depth);
    }

	void retrieve_calibrations(ros::NodeHandle &mynode)
	{
		//Default values not yet set
		mynode.param("cauv_cangate/fore_offset", fore_offset, float(1.0));
		mynode.param("cauv_cangate/aft_offset", aft_offset, float(1.0));
		mynode.param("cauv_cangate/fore_multiplier", fore_multiplier, float(1.0));
		mynode.param("cauv_cangate/aft_multiplier", aft_multiplier, float(1.0));
	}

	float calc_depth()
	{
        if (forward_pressure == 0 && back_pressure == 0) return 0;

        else if (forward_pressure == 0) return depthFromAftPressure(back_pressure);

        else if (back_pressure == 0) return depthFromForePressure(forward_pressure);
        
        else return (depthFromForePressure(forward_pressure) + depthFromAftPressure(back_pressure)) / 2;
	}

	float depthFromForePressure(float const& pressure) const
    {
 		return fore_offset + fore_multiplier * pressure;
    }

    float depthFromAftPressure(float const& pressure) const
    {
		return aft_offset + aft_multiplier * pressure;
    }

    void cauv_motor_send_command(const cauv_cangate::msg_motor_demand::ConstPtr motor_control_message)
	{
		struct can_frame frame;
		
		frame.can_id  = 0x014;
		frame.can_dlc = 6;
		frame.data[0] = motor_control_message->fwd_left;
		frame.data[1] = motor_control_message->fwd_right; 
		frame.data[2] = motor_control_message->vert_fore;
		frame.data[3] = motor_control_message->vert_aft;
		frame.data[4] = motor_control_message->horz_fore;
		frame.data[5] = motor_control_message->horz_aft;

		write(mysocket, &frame, sizeof(struct can_frame)); 
	}

	private:

	ros::NodeHandle mynode;
	
	ros::Subscriber sub_motor_command;
	ros::Publisher pub_depth_status;

    //Socket declarations
	int i;
	int mysocket;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame last_frame_in_buffer;
	std::string ifname;

	int16_t forward_pressure;
	int16_t back_pressure;
	std_msgs::Float64 depth;
    
    float fore_offset;
    float aft_offset;
    float fore_multiplier;
    float aft_multiplier;    
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "cauv_cangate");

	ROS_INFO("Initialising the cauv_cangate node");

	boost::shared_ptr<CanGate> can_gate = boost::make_shared<CanGate>();

	ROS_INFO("The cauv_cangate node is up and running! \n");

    ros::Rate loopRate(100);

	while (ros::ok())
	{		
		can_gate->get_can_frame();
		can_gate->parse_frame();

		ros::spinOnce();
		loopRate.sleep();
	}

    can_gate.reset();

	return 0;
}
