/*
This is the priority manager node's main file.

It acts as a server, receives demands defined in the srv file, and stores them in the linked list defined in msg_requeststack.h
Then it constantly publishes the top demand in the stack to the topic which is listened to by the control node.

However, if doesn't remove a demand from the stack as soon as it is interrupted by a higher priority demand. (implementation needed)
*/

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <string>
#include <cstdlib>

#include "ros/ros.h"
#include "ros/package.h"

#include "mgr_requeststack.h"

cauv::MgrRequestList RequestStack;

bool process_request(cauv_priority_mgr::srv_control_command::Request  &req, cauv_priority_mgr::srv_control_command::Response &res)
{
	RequestStack.insert(req);
	
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cauv_priority_mgr");
	ros::NodeHandle mynode;
		
	//Read and set up as params all priority stuff from PACKAGE_PATH/config/priority.conf
	
	std::ifstream file_priority_config(ros::package::getPath("")+"/conf/priority.conf");
	
	if(file_priority_config.fail())
	{
		ROS_INFO_STREAM("Could not open config file for priorities.  Assigning standard priority of 40 to all nodes." << std::endl);
	}

	ros::param::set("/cauv/priority/default", 40);
	
	if(!ros::param::has("/cauv/priority/default"))
	{
		ROS_INFO_STREAM("Could not connect to parameter server for setting default priority. Aborting node. " << std::endl);
		return -1;
	}
	
	else
	{
		std::string cur_line, nodeid, priority_str;
		int priority;
	
		while ( ! file_priority_config.eof())
		{	
			std::getline (file_priority_config, cur_line);
		
			int delimiter = cur_line.find(" ");
		
			nodeid = cur_line.substr(0,delimiter);
			priority_str = cur_line.substr(delimiter+1);
		
			priority = atoi(priority_str.c_str());
		
			if(priority>=0 && priority<=100)
			{
				ros::param::set("/cauv/priority/"+nodeid, priority);
			}
			else ROS_INFO_STREAM("Did not set priority for node " << nodeid << " because invalid priority in file (range: 0->100)" << std::endl);
		}
	}
	
	ros::ServiceServer service = mynode.advertiseService("cauv_control_command", process_request);
	ros::Publisher pub_motorcontrol_demand = mynode.advertise<cauv_control::msg_motorcontrol_demand>("cauv_motorcontrol_demand", 100);

	ROS_INFO_STREAM("The cauv_priority_mgr node is up and running!" << std::endl);
	
	cauv_priority_mgr::srv_control_command::Request Cur_Request;
	
	while (ros::ok())
	{
		if(RequestStack.top(Cur_Request))
		
		pub_motorcontrol_demand.publish(Cur_Request.motor_demand);		
		
		ros::spinOnce();
	}
	
	return 0;
}



