#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

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
	double* Euler;
	
	ros::Publisher pub_sbgimu_status = nh.advertise<cauv_control::msg_floatYPR>("cauv_control_YPR", 1000);
	
	sbgIMU* sbg_imu = new sbgIMU("hello", 0, 0);

	sbg_imu->initialise();

	ROS_INFO("The cauv_sbgimu node is up and running! \n");

	while (ros::ok())
	{	
		Euler = sbg_imu->getYPR();
		imu_YPR_message.yaw = Euler[2];
		imu_YPR_message.pitch = Euler[1];
		imu_YPR_message.roll = Euler[0];
		pub_sbgimu_status.publish(imu_YPR_message);
		
		//Activate callback!
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
