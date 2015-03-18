#ifndef MGR_REQUESTSTACK_H_INCLUDED
#define MGR_REQUESTSTACK_H_INCLUDED

#include <string>

#include "ros/ros.h"

#include "include/cauv_control/msg_motorcontrol_demand.h"
#include "include/cauv_priority_mgr/srv_control_command.h"

namespace cauv
{
	int get_priority(string nodeid);  //done

	//A link of the linked list
	class MgrRequestLink
	{
		public:

		MgrRequestLink(); //done
		MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest); //done
		MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest, MgrRequestLink *next_ptr);
		~MgrRequestLink(); //done
	
		cauv_priority_mgr::srv_control_command::Request Request;
		MgrRequestLink *next;
		MgrRequestLink *prev;
	}

	//The whole linked list of past requests
	class MgrRequestList
	{
		public:
		
		MgrRequestList(); //done
		~MgrRequestList(); //TODO
	
		int insert(cauv_priority_mgr::srv_control_command::Request InboundRequest); //done
		int remove(MgrRequestLink *RequestLink_ptr); //done
		
		MgrRequestLink* getTopRequestLink();  //done

		int clean(); //TODO
	
		private:
		
		int num_links;
		
		MgrRequestLink *TopRequestLink;
	}
}

#endif
