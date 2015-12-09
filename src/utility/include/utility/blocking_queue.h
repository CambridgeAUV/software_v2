/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __BLOCKINGQUEUEBOOST_H__
#define __BLOCKINGQUEUEBOOST_H__

#include <queue>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition.hpp>

namespace cauv {

template <typename T> 
class BlockingQueue : boost::noncopyable
{
    // Private typedefs
    typedef boost::unique_lock<boost::mutex> lock_t;
    typedef boost::mutex::scoped_try_lock try_lock_t;

    public:
        typedef size_t size_type;
        
        void push(const T& x) // Used to add a new item to the queue
        {
            lock_t lock(m_queuemutex); // Obtain the lock for this scope
            m_queue.push(x); // Push the object onto the queue
            m_itemAvailable.notify_one(); // Indicate that a new item has been pushed onto the queue
        }

        T popWait() // Returns and then deletes the next element in the queue, waiting for a new item to be pushed if neccessary. 
        {
            lock_t lock(m_queuemutex); // Obtain the lock for this scope
            while (m_queue.empty()) // Is the queue empty?
            {
                m_itemAvailable.wait(lock); // Release the lock and wait on a new push
            }
            
            T x = m_queue.front(); // Retrieve the object from the front of the queue
            m_queue.pop(); // Delete the next object onto the queue

            return x;
        } 

        bool tryPop(T& x, bool waitIfEmpty = false) // Tries to access the next element in the queue, and returns true if it succeeds with the element passed by reference, false otherwise
        {	
            try_lock_t lock(m_queuemutex); // Try to obtain the lock for this scope
            if (lock.owns_lock()) // Did we get the lock
            {
                while (m_queue.empty()) // Is the queue empty?
                {
                    if (waitIfEmpty)
                    {
                        m_itemAvailable.wait(lock); // Release the lock and wait on a new push
                    }
                    else
                    {
                        return false;
                    }
                }

                x = m_queue.front(); // Retrieve the object from the front of the queue
                m_queue.pop(); // Delete the next object onto the queue
                return true;
            }
            else 
            {
                return false;
            }
        }

        size_type size() const
        {	
            try_lock_t lock(m_queuemutex); // Try to obtain the lock for this scope
            return m_queue.size();    
        }

    private:
        std::queue <T> m_queue;

        mutable boost::mutex m_queuemutex; // The single lock for the whole queue
        boost::condition m_itemAvailable; // The condition idicating a new item has been pushed onto an empty queue
};

}

#endif
