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
#include "cauv_control/msg_float_rotation_matrix.h"
using namespace cauv;

int main(int argc, char **argv)
{
	//ROS initialisation
	ros::init(argc, argv, "cauv_sbgimu");
	ros::NodeHandle nh;
	ros::Rate loopRate(100);

	std::string device_name;
	int baud_rate, pause_time;
	//ROS_INFO(device_name.c_str());

	// Import parameters to initialise imu device from parameter server
	// Exit with error if parameters not found.
	// device_name, baud_rate, pause_time
	bool got_device_name = ros::param::get("cauv_sbgimu/device", device_name);
	if (!got_device_name)
	{
		ROS_FATAL_STREAM("Could not get parameter: device");
		exit(1);
	}
	bool got_baud_rate = ros::param::get("cauv_sbgimu/baud_rate", baud_rate);
	if (!got_baud_rate)
	{
		ROS_FATAL_STREAM("Could not get parameter: baud_rate");
		exit(2);
	}
	bool got_pause_time = ros::param::get("cauv_sbgimu/pause_time", pause_time);
	if (!got_pause_time)
	{
		ROS_FATAL_STREAM("Could not get parameter: pause_time");
		exit(3);
	}


	cauv_control::msg_floatYPR imu_YPR_message;
	cauv_control::msg_float_rotation_matrix imu_rotation_matrix_message;

	int error;
	double euler[3];
	double rotation_matrix[9];
	
	ros::Publisher pub_sbgimu_status = nh.advertise<cauv_control::msg_floatYPR>("cauv_sensors/imu_attitude", 1);
	ros::Publisher pub_sbgimu_rotation_status = nh.advertise<cauv_control::msg_float_rotation_matrix>("cauv_sensors/imu_rotation_matrix", 1);
	
	// sbgIMU constructor takes three arguments: port, baud_rate and pause_time.  
	sbgIMU* sbg_imu = new sbgIMU(device_name.c_str(), baud_rate, pause_time);

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

		error = sbg_imu->get_rotation_matrix(rotation_matrix);
		if (error == 0)
		{
			imu_rotation_matrix_message.aX = rotation_matrix[0];
			imu_rotation_matrix_message.aY = rotation_matrix[1];
			imu_rotation_matrix_message.aZ = rotation_matrix[2];
			imu_rotation_matrix_message.bX = rotation_matrix[3];
			imu_rotation_matrix_message.bY = rotation_matrix[4];
			imu_rotation_matrix_message.bZ = rotation_matrix[5];
			imu_rotation_matrix_message.cX = rotation_matrix[6];
			imu_rotation_matrix_message.cY = rotation_matrix[7];
			imu_rotation_matrix_message.cZ = rotation_matrix[8];
			pub_sbgimu_rotation_status.publish(imu_rotation_matrix_message);
		}
		
		//Activate callback!
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
