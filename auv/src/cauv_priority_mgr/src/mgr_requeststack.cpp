#include <iostream>
#include <string>
#include <ctime>
#include <new>

#include "ros/ros.h"

#include "include/cauv_control/msg_motorcontrol_demand.h"
#include "include/cauv_priority_mgr/srv_control_command.h"

#include "mgr_requeststack.h"

int cauv::get_priority(std::string nodeid)
{
	int priority=-1;

	if(ros::param::get("/cauv/priority/" + nodeid, priority) && priority>=0 && priority<=100) return priority;
	else
	{
		priority = ros::param::get("/cauv/priority/default", priority);
		return priority;
	}
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
}

cauv::MgrRequestLink::~MgrRequestLink(){}

//Now with MgrRequestList methods

cauv::MgrRequestList::MgrRequestList()
{
	num_links = 0;
	TopRequestLink = NULL;
}

cauv::MgrRequestList::~MgrRequestList()
{
	MgrRequestLink *Cur_Request = TopRequestLink;
	MgrRequestLink *Next_Request;
	while(Cur_Request != NULL)
	{
		Next_Request = Cur_Request->next;
		delete Cur_Request;
		Cur_Request = Next_Request;
	}
}

int cauv::MgrRequestList::insert(cauv_priority_mgr::srv_control_command::Request InboundRequest)
{
	if (TopRequestLink == NULL)
	{
		TopRequestLink = new cauv::MgrRequestLink(InboundRequest);
		num_links = 1;
	}
	
	else
	{
		cauv::MgrRequestLink* CurRequest = TopRequestLink;

		while (cauv::get_priority(CurRequest->Request.nodeid) > cauv::get_priority(InboundRequest.nodeid))
		{
			CurRequest = CurRequest->next;
		}
		
		if (CurRequest == TopRequestLink)
		{
			TopRequestLink = new MgrRequestLink(InboundRequest, TopRequestLink);
			num_links++;
		}
		
		else
		{
			cauv::MgrRequestLink* NewRequest = new MgrRequestLink(InboundRequest, CurRequest);
			CurRequest->prev->next = NewRequest;
			CurRequest->prev = NewRequest;
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
	cauv::MgrRequestLink *Cur_Request = TopRequestLink; 
	cauv::MgrRequestLink *Prev_Request;
	while (Cur_Request != NULL)
	{
		Prev_Request = Cur_Request;
		Cur_Request = Cur_Request->next;
		if(Prev_Request->Request.timeout < ros::Time::now()) this->remove(Prev_Request);
	}
	return 0;
}

bool cauv::MgrRequestList::top(cauv_priority_mgr::srv_control_command::Request &ReturnRequest)
{
	if(TopRequestLink == NULL) return false;
	
	else
	{
		cauv::MgrRequestLink *Cur_Request = TopRequestLink;
		cauv::MgrRequestLink *Prev_Request;
		cauv_priority_mgr::srv_control_command::Request ReturnRequest;
	
		while (Cur_Request->Request.timeout < ros::Time::now())
		{
			Prev_Request = Cur_Request;
			Cur_Request = Cur_Request->next;
			this->remove(Prev_Request);
		}
		ReturnRequest = Cur_Request->Request;
		//remove(Cur_Request);
		return true;
	}
}
