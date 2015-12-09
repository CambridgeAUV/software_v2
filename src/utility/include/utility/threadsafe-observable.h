/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __THREADSAFE_OBSERVABLE_H__
#define __THREADSAFE_OBSERVABLE_H__

#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/recursive_mutex.hpp>

template<typename T>
class ThreadSafeObservable
{
    public:
        typedef boost::shared_ptr<T> observer_ptr_t;

        void addObserver(observer_ptr_t o)
        {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.push_back(o);
        }
        void removeObserver(observer_ptr_t o)
        {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.remove(o);
        }
        void clearObservers()
        {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.clear();
        }

    protected:
        std::list<observer_ptr_t> m_observers;
        boost::recursive_mutex m_observers_lock;
};

#endif // ndef __THREADSAFE_OBSERVABLE_H__
