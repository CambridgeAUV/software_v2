/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_ASYNC_H__
#define __CAUV_UTILITY_ASYNC_H__

#include <set>
#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <debug/cauv_debug.h>

class AsyncService{
        typedef boost::asio::io_service io_service;
        typedef boost::asio::deadline_timer deadline_timer;
        typedef boost::shared_ptr<deadline_timer> timer_ptr;
        typedef boost::unique_lock<boost::mutex> lock_t;
        typedef boost::shared_ptr<boost::thread> thread_ptr;
    public:
        typedef boost::function<void()> function_t;

        AsyncService(int start_threads=1)
            : m_threads(),
              m_ready_mux(),
              m_ready(),
              m_have_work_to_do_mux(),
              m_have_work_count(0),
              m_have_work_to_do(),
              m_timers_lock(),
              m_inactive_timers(),
              m_ioservice(){
            while(--start_threads >= 0)
                m_threads.push_back(boost::make_shared<boost::thread>(
                    boost::bind(&AsyncService::consume, this)
                ));
            // wait until at least one thread is waiting for callbacks before
            // returning, otherwise callbacks can fall through the gap!
            lock_t l(m_ready_mux);
            m_ready.wait(l);
            // make sure the io_service always has some work to do, so it
            // doesn't stop until this object is destroyed
            keepWorking();
        }

        ~AsyncService(){
            // timers must be destroyed before the ioservice that they
            // reference
            m_inactive_timers.clear();
            m_ioservice.stop();
            for(std::vector<thread_ptr>::iterator i = m_threads.begin(); i != m_threads.end(); i++){
                (*i)->interrupt();
                (*i)->join();
            }
        }

        /* Wait for all pending handlers to complete: this thread may
         * be used to call handlers!
         *
         * After calling this, this object must be destroyed. No other member
         * functions may be called.
         */
        void waitForCompletion(){
            m_ioservice.run();
        }


        /* Call this to consume additional threads into servicing asynchronous
         * callbacks
         */
        void consume(){
            boost::system::error_code ec;
            std::size_t executed;
            bool first = true;

            while(true){
                lock_t l(m_have_work_to_do_mux);
                if(first){
                    first = false;
                    m_ready.notify_one();
                }
                // if more work was posted while we were executing the last bit
                // of work, then we'll have missed the notification on the
                // m_have_work_to_do condition variable
                if(!m_have_work_count)
                    m_have_work_to_do.wait(l);
                m_have_work_count--;
                l.unlock();
                executed = m_ioservice.run_one(ec);
                if(ec){
                    error() << "AsyncService:" << ec.message();
                    break;
                }
                if(executed == 0){
                    error() << "AsyncService: notified, but nothing available to execute";
                    // this only happens when service is stopped... so not an error
                    //break;
                }
            }
        }

        /* self explanatory */
        void callAfterMicroseconds(function_t foo, uint32_t microseconds){
            callAfterTime(foo, boost::posix_time::microseconds(microseconds));
        }
        
        /* should be called callAfterDelay... */
        template<typename Time>
        void callAfterTime(function_t foo, Time time){
            //debug() << "callback setup:" << &foo << time;
            timer_ptr timer = getTimer();

            timer->expires_from_now(time);
            timer->async_wait(boost::bind(&AsyncService::callbackThunk, this, _1, timer, foo));

            lock_t l(m_have_work_to_do_mux);
            m_have_work_count++;
            m_have_work_to_do.notify_one();
        }

    private:
        void keepWorking(){
            callAfterTime(boost::bind(&AsyncService::keepWorking, this), boost::posix_time::hours(100));
        }

        timer_ptr getTimer(){
            lock_t l(m_timers_lock);
            timer_ptr r;

            if(m_inactive_timers.size()){
                r = *m_inactive_timers.begin();
                m_inactive_timers.erase(r);
            }else{
                r = boost::make_shared<deadline_timer>(boost::ref(m_ioservice));
            }
            return r;
        }

        void releaseTimer(timer_ptr t){
            lock_t l(m_timers_lock);
            m_inactive_timers.insert(t);
        }

        void callbackThunk(boost::system::error_code const& ec, timer_ptr t, function_t foo){
            if(ec == boost::asio::error::operation_aborted){
                debug() << "timer cancelled";
                return;
            }else if(ec){
                error() << "timer error:" << ec.message();
            }else{
                //debug() << "callback:" << &foo;
                foo();
            }
            // leave releasing the timer until the end, so the callback happens
            // as soon as possible
            releaseTimer(t);
        }

        std::vector<thread_ptr> m_threads;

        boost::mutex m_ready_mux;
        boost::condition_variable m_ready;

        boost::mutex m_have_work_to_do_mux;
        uint32_t m_have_work_count;
        boost::condition_variable m_have_work_to_do;

        boost::mutex m_timers_lock;
        std::set<timer_ptr> m_inactive_timers;

        io_service m_ioservice;
};


#endif // ndef __CAUV_UTILITY_ASYNC_H__
