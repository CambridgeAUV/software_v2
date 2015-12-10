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

#include <std_msgs/Float64.h>

int mysocket;

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

//added the class pressure_depth_converter
//needs to include cauv_control::DepthCalibration
class pressure_depth_converter
{
   private:
	float forward_pressure;
	float back_pressure;
        float fore_offset;
        float aft_offset;
        float fore_multiplier;
        float aft_multiplier;
   public:
	int16_t fwd_pressure;
	int16_t aft_pressure;


	void retrieve_calibrations(ros::NodeHandle &mynode)
	{
		//Default values not yet set
		mynode.param("cauv_cangate/fore_offset",fore_offset,float(1.0));
		mynode.param("cauv_cangate/aft_offset",aft_offset,float(1.0));
		mynode.param("cauv_cangate/fore_multiplier",fore_multiplier,float(1.0));
		mynode.param("cauv_cangate/aft_multiplier",aft_multiplier,float(1.0));
	}

	float calc_depth()
	{
		forward_pressure = fwd_pressure;
		back_pressure = aft_pressure;
		return (depthFromForePressure(forward_pressure)+depthFromAftPressure(back_pressure))/2;
	}

	float depthFromForePressure(float const& pressure) const
        {
 		return fore_offset + fore_multiplier * pressure;

        }

        float depthFromAftPressure(float const& pressure) const
        {
		return aft_offset + aft_multiplier * pressure;
        }	
};


        



int main(int argc, char **argv)
{
	//Socket declarations
	int i;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame last_frame_in_buffer;


	//ROS initialisation
	ros::init(argc, argv, "cauv_cangate");
	ros::NodeHandle mynode;
	ros::Rate loopRate(100);
	
	ros::Subscriber sub_motor_command = mynode.subscribe("cauv_motor_demand", 1, cauv_motor_send_command);

	


//publishes the depth of the midpoint of the auv
//changed from here
	pressure_depth_converter depth_track = pressure_depth_converter();

	depth_track.retrieve_calibrations(mynode);

	ros::Publisher pub_depth_status = mynode.advertise<std_msgs::Float64>("cauv_cangate/cauv_depth_status", 1);	
	std_msgs::Float64 depth;
//to here


	ROS_INFO("Initialising the cauv_cangate node");
	
	//Connect to can0 network interface
	char *ifname = "can0";

	if((mysocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
	{
		ROS_ERROR("Error while opening CAN socket");
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(mysocket, SIOCGIFINDEX, &ifr);

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	ROS_INFO("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(mysocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	{
		ROS_ERROR("Error in CAN socket bind");
	}

	ROS_INFO("The cauv_cangate node is up and running! \n");

	
	while (ros::ok())
	{		
		//Retrieve last frame in CAN buffer
		
		read(mysocket,&last_frame_in_buffer,sizeof(struct can_frame));
		
		//Parse frames by address		
		if (last_frame_in_buffer.can_id == 11)  // This frame comes from a pressure sensor
		{
			if(last_frame_in_buffer.data[4] == 0)
			{
				memcpy(&depth_track.fwd_pressure, &last_frame_in_buffer.data, 2);

			}
			
			if(last_frame_in_buffer.data[4] == 1)
			{
				memcpy(&depth_track.aft_pressure, &last_frame_in_buffer.data, 2);
			}
			
			depth.data = depth_track.calc_depth();

			pub_depth_status.publish(depth);
		}
		
		//Activate callback!
		ros::spinOnce();
		loopRate.sleep();
	}
	
	close(mysocket);
	
	return 0;
}
