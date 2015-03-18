#include <iostream>
#include <fstream>

#include <string>

#include "ros/ros.h"

#include "mgr_requeststack.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cauv_priority_mgr");
	ros::NodeHandle mynode;
	
	//Set up all priority stuff from /cauv/config/priority.conf
	
	std::ifstream file_priority_config("cauv/config/priority.conf");
	
	string cur_line, nodeid, priority_str;
	int priority;
	
	while ( ! file_priority_config.eof())
	{	
		std::getline (file_priority_config, cur_line);
		
		int delimiter = cur_line.find(" ");
		
		nodename = cur_line.substr(0,delimiter);
		priority_str = cur_line.substr(delimiter+1);
		
		priority = std::stoi(priority_str);
		
		if(priority>=0 && priority<=100)
		{
			ros::param::set("/cauv/priority/"+nodeid, priority);
		}
		else ROS_INFO("Set priority for node %s as 0 because invalid priority set (range: 0->100) \n", nodeid);
	}
	
	
	

	
	
}



