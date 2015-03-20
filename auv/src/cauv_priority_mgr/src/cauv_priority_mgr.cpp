/*
Node: cauv_priority_mgr
Description: Manages accesses to the cauv_control node and distributes the commands to it.  

Publishes to the cauv_motorcontrol_demand topic (message type msg_motorcontrol_demand)
Receives service requests on cauv_control_command (service type srv_control_command)


TODO:

- Feedback system from cauv_control to indicate when command is finished -> load next command
Implement by receiving from a "cauv_motorcontrol_status" topic (0 when not finished, 1 when yes)
Probably implement by library external to node (which will be included by cauv_control and which will use multithreading),
and listen to cauv_motorcontrol_status in this node

- Encapsulation: Scripts don't have to worry about the metadata
Implement by library external to node, included by scripts.  

- Replace: a request can indicate to remove others from the queue.
Implement by assigning some sort of unique ID to each request (in Header)

- Multithreading support
Haha I might just have to start learning about multithreading first... :)

- Remove a demand from the stack as soon as interrupted by a higher priority demand (prevent dead-locks)
Manually do that when higher-priority demand comes? Or pop as soon as received?
Probably the pop-when-extracted-from-stack approach more robust, less fiddly.  

ard61
*/

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <string>

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
	
	else //default priority will be changed in here
	{
		std::string cur_line, nodeid, priority_str;
		int priority;
	
		while ( ! file_priority_config.eof())
		{	
			std::getline (file_priority_config, cur_line);
		
			int delimiter = cur_line.find(" ");
		
			nodeid = cur_line.substr(0,delimiter);
			priority_str = cur_line.substr(delimiter+1);
		
			priority = stoi(priority_str);
		
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
		if(RequestStack.top(Cur_Request)) pub_motorcontrol_demand.publish(Cur_Request.motor_demand);		
		
		ros::spinOnce();
	}
	
	return 0;
}

