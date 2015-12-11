/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

/*
Defines the cauv::sbgIMU class, which allows interfacing with the SBG IMU.  
Implementation in sbg_imu.cpp
*/

#ifndef __CAUV_SBG_IMU_H__
#define __CAUV_SBG_IMU_H__

#include "sbg/sbgCom.h"

namespace cauv{

class sbgIMU
{
    public:
        sbgIMU(const char* port, int baud_rate, int pause_time);
        ~sbgIMU();

        void initialise();
        int getYPR(double (&euler)[3]);
        int get_rotation_matrix(double (&rotation_matrix)[9]);

    private:
        
        const char* m_port;
        int         m_baud_rate;
        int         m_pause_time;

        SbgProtocolHandle protocolHandle;
        SbgErrorCode error;
        SbgOutput output;
};

class sbgException : public std::exception
{
    protected:
        std::string message;
    public:
        sbgException(const std::string& msg);
        ~sbgException() throw();
        virtual const char* what() const throw();
};

} // namespace cauv

#endif // ndef __CAUV_SBG_IMU_H__
