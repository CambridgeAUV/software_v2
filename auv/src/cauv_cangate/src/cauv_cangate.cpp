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

int mysocket;

void cauv_motor_send_command(const cauv_cangate::msg_motor_demand& motor_control_message)
{
	struct can_frame frame;
	
	frame.can_id  = 0x014;
	frame.can_dlc = 6;
	frame.data[0] = motor_control_message.fwd_left;
	frame.data[1] = motor_control_message.fwd_right; 
	frame.data[2] = motor_control_message.vert_fore;
	frame.data[3] = motor_control_message.vert_aft;
	frame.data[4] = motor_control_message.horz_fore;
	frame.data[5] = motor_control_message.horz_aft;

	write(mysocket, &frame, sizeof(struct can_frame)); 
}


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
	
	ros::Subscriber sub_motor_command = mynode.subscribe("cauv_motor_demand", 1000, cauv_motor_send_command);
	ros::Publisher pub_pressure_status = mynode.advertise<cauv_cangate::msg_pressure_status>("cauv_pressure_status", 1000);
	
	cauv_cangate::msg_pressure_status pressure_status_message;
	
	//Connect to can0 network interface
	char *ifname = "can0";

	if((mysocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
	{
		perror("Error while opening socket");
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(mysocket, SIOCGIFINDEX, &ifr);

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	ROS_INFO("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(mysocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	{
		perror("Error in socket bind");
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
				memcpy(&pressure_status_message.fwd_pressure, &last_frame_in_buffer.data, 2);
			}
			
			if(last_frame_in_buffer.data[4] == 1)
			{
				memcpy(&pressure_status_message.aft_pressure, &last_frame_in_buffer.data, 2);
			}
			pub_pressure_status.publish(pressure_status_message);
		}
		
		//Activate callback!
		ros::spinOnce();
		loopRate.sleep();
	}
	
	close(mysocket);
	
	return 0;
}
