/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
        double* getYPR();

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
