/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_PERFORMANCE_H__
#define __CAUV_PERFORMANCE_H__

#include <sys/time.h>

class Timer{
    public:
        typedef unsigned long long diff_t;

        void start(){
            gettimeofday(&m_start_time, NULL);
        }

        diff_t stop(){
            gettimeofday(&m_stop_time, NULL);
            return muSec();
        }

        diff_t muSec() const{
            return diff_t(m_stop_time.tv_sec - m_start_time.tv_sec) * 1000000 + (m_stop_time.tv_usec - m_start_time.tv_usec);
        }
    
    private:
        timeval m_start_time;
        timeval m_stop_time;
};

#endif // ndef __CAUV_PERFORMANCE_H__

