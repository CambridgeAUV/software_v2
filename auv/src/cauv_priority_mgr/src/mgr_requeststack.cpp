#include <string>
#include <ctime>

#include "ros/ros.h"

#include "include/cauv_control/msg_motorcontrol_demand.h"
#include "include/cauv_priority_mgr/srv_control_command.h"

#include "mgr_requeststack.h"

int cauv::get_priority(string nodeid)
{
	int priority;

	ros::param::get("/cauv/priority/" + nodeid, priority);
	
	return priority;
}

cauv::MgrRequestLink::MgrRequestLink()
{
	
}

cauv::MgrRequestLink::MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest)
{
	Request = InboundRequest;
	next = NULL;
	prev = NULL;
}

cauv::MgrRequestLink::MgrRequestLink(cauv_priority_mgr::srv_control_command::Request InboundRequest, cauv::MgrRequestLink* next_ptr)
{
	Request = InboundRequest;
	next = next_ptr;
	prev = NULL;

cauv::MgrRequestLink::~MgrRequestLink(){}

//Now with MgrRequestList methods

cauv::MgrRequestList::MgrRequestList()
{
	num_links = 0;
	TopRequestLink = NULL;
}

int cauv::MgrRequestList::insert(cauv_priority_mgr::srv_control_command::Request InboundRequest)
{
	if (TopRequestLink == NULL)
	{
		TopRequestLink = new RequestLink(InboundRequest);
		num_links = 1;
	}
	
	else
	{
		cauv::RequestLink* CurRequest = TopRequestLink;

		while (cauv::get_priority(CurRequest->Request.nodeid) > cauv::get_priority(InboundRequest.nodeid))
		{
			CurRequest = CurRequest->next;
		}
		
		if (CurRequest == TopRequestLink)
		{
			TopRequestLink = new RequestLink(InboundRequest, TopRequestLink);
			num_links++;
		}
		
		else
		{
			cauv::RequestLink* NewRequest = new RequestLink(InboundRequest, CurRequest);
			CurRequest->prev->next = NewRequest;
			CurRequest.prev = NewRequest;
			num_links++;
		}
	
	}

	return 0;
}

int cauv::MgrRequestList::remove(MgrRequestLink *RequestLink_ptr)
{
	if (RequestLink_ptr->prev != NULL) RequestLink_ptr->prev->next = RequestLink_ptr->next;
	if (RequestLink_ptr->next != NULL) RequestLink_ptr->next->prev = RequestLink_ptr->prev;
	delete RequestLink_ptr;
	return 0;
}

int cauv::MgrRequestList::clean()
{
	for(MgrRequestLink *Cur_Request = TopRequestLink; Cur_Request != NULL; Cur_Request = Cur_Request->next)
	{
		if(Cur_Request->Request.timeout > ros::Time::now()) remove(Cur_Request);
	}
}
MgrRequestLink* cauv::MgrRequestList::getTopRequestLink()
{
	return TopRequestLink;
}
