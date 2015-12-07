/*
cauv_sbgimu ROS node:

Reads information from the SBG IMU using functions defined in
proprietary code supplied by SBG.  

Publishes messages of type cauv_control/msg_floatYPR to the
ROS topic cauv_control_YPR, for use by the cauv_control node.  

Also, I defined SBG_PLATFORM_LITTLE_ENDIAN in protocol/protocolOutputMode.h 
to get the supplied code to compile.  

TODO: Read values for port, baud_rate and pause_time from ROS
parameter server.  
*/

#include "ros/ros.h"

#include "sbg_imu.h"

#include "cauv_control/msg_floatYPR.h"

using namespace cauv;

int main(int argc, char **argv)
{
	//ROS initialisation
	ros::init(argc, argv, "cauv_sbgimu");
	ros::NodeHandle nh;
	ros::Rate loopRate(100);

	cauv_control::msg_floatYPR imu_YPR_message;

	int error;
	double euler[3];
	
	ros::Publisher pub_sbgimu_status = nh.advertise<cauv_control::msg_floatYPR>("cauv_control_YPR", 1000);
	
	// sbgIMU constructor takes three arguments: port, baud_rate and pause_time.  
	sbgIMU* sbg_imu = new sbgIMU("/dev/usbmon1", 115200, 10);

	sbg_imu->initialise();

	ROS_INFO("The cauv_sbgimu node is up and running! \n");

	// Main loop
	while (ros::ok())
	{	
		error = sbg_imu->getYPR(euler);
		if (error == 0)
		{
			imu_YPR_message.yaw = euler[2];
			imu_YPR_message.pitch = euler[1];
			imu_YPR_message.roll = euler[0];
			pub_sbgimu_status.publish(imu_YPR_message);
		}
		
		//Activate callback!
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
