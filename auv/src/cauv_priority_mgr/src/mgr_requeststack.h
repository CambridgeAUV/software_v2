/*
Defines the request list used by the cauv_priority_mgr node to distribute access to motor control:
stores requests (def in srv_control_command.srv) by priority, and has functions to insert, remove, clean by timestamp.  
Implementation in mgr_requeststack.cpp

ard61
*/

#ifndef MGR_REQUESTSTACK_H_INCLUDED
#define MGR_REQUESTSTACK_H_INCLUDED

#include <string>

#include "ros/ros.h"

#include "include/cauv_control/msg_motorcontrol_demand.h"
#include "include/cauv_priority_mgr/srv_control_command.h"

namespace cauv
{
	int get_priority(std::string nodeid);  //get priority for a given node from ROS' param server.  

	//A link of the linked list
	class MgrRequestLink
	{
		public:

		MgrRequestLink();
		MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest);
		MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest, MgrRequestLink *next_ptr); //done
		~MgrRequestLink(); //done
	
		cauv_priority_mgr::srv_control_command::Request Request; //the access request stored by the link
		MgrRequestLink *next;
		MgrRequestLink *prev;
	};

	//The whole linked list of past requests
	class MgrRequestList
	{
		public:
		
		MgrRequestList(); //done
		~MgrRequestList(); //done
	
		int insert(cauv_priority_mgr::srv_control_command::Request InboundRequest); //done
		int remove(MgrRequestLink *RequestLink_ptr); //done
		
		bool top(cauv_priority_mgr::srv_control_command::Request &ReturnRequest); //done
		int clean(); //done
	
		private:
		
		int num_links;
		
		MgrRequestLink *TopRequestLink;
	};
}

#endif
