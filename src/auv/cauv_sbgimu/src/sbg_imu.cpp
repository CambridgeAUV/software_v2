/*
Implementation of the sbgIMU class, see sbg_imu.h
*/

#include <string>

//#define CAUV_DEBUG_COMPAT
//#include <debug/cauv_debug.h>

#include "sbg/sbgCom.h"

#include "sbg_imu.h"
#include "ros/ros.h"

using namespace std;
using namespace cauv;

sbgException::sbgException(const string& msg) : message(msg) { }

sbgException::~sbgException() throw () { }

const char* sbgException::what() const throw ()
{
    return message.c_str();
}


sbgIMU::sbgIMU(const char* port, int baud_rate, int pause_time)
    :  
        m_port(port),
        m_baud_rate(baud_rate),
        m_pause_time(pause_time)
{
    if (sbgComInit(m_port, m_baud_rate, &protocolHandle) == SBG_NO_ERROR)
    {
        //
        // Wait until the device has been initialised
        //
        sbgSleep(50);
        ROS_DEBUG("SBG IMU initialised");
    }
    else
    {
        throw sbgException("Failed to open sbg port");
    }
}


void sbgIMU::initialise()
{
    error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
    if (error != SBG_NO_ERROR) {
        ROS_WARN("Sbg not connected. ");
    }
}

sbgIMU::~sbgIMU()
{
    sbgProtocolClose(protocolHandle);
}


int sbgIMU::get_rotation_matrix(double (&rotation_matrix)[9])
{
    error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_MATRIX, &output);
    if (error == SBG_NO_ERROR)
    {
        // X - forward, z - upwards
	// Matrix in the form of <aX, aY, aZ, bX, bY, bZ, cX, cY, cZ>
	
	for(int i=0;i<9;i++)
	{
		rotation_matrix[i]=output.stateMatrix[i];
	}

        return 0;
    }
    else
    {
        ROS_WARN_STREAM("Lost connection to sbg, error code: " << error);
        return -1;
    }
    //sbgSleep(m_pause_time);
}


int sbgIMU::getYPR(double (&euler)[3])
{
    error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);
    if (error == SBG_NO_ERROR)
    {
        // X - forward, z - upwards
        // euler[0] - roll
        // euler[1] - pitch
        // euler[2] - yaw
        euler[0] = SBG_RAD_TO_DEG(output.stateEuler[0]);
        euler[1] = SBG_RAD_TO_DEG(output.stateEuler[1]);
        euler[2] = SBG_RAD_TO_DEG(output.stateEuler[2]);

        //printf("%3.2f\t%3.2f\t%3.2f\n",    euler[0],
        //                                euler[1],
        //                                euler[2]);
        return 0;
    }
    else
    {
        ROS_WARN_STREAM("Lost connection to sbg, error code: " << error);
        return -1;
    }
    //sbgSleep(m_pause_time);
}
