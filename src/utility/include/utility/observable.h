/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __OBSERVABLE_H__
#define __OBSERVABLE_H__

#include <list>

#include <boost/shared_ptr.hpp>

template<typename T>
class Observable
{
    public:
        typedef boost::shared_ptr<T> observer_ptr_t;

        void addObserver(observer_ptr_t o)
        {
            m_observers.push_back(o);
        }
        void removeObserver(observer_ptr_t o)
        {
            m_observers.remove(o);
        }
        void clearObservers()
        {
            m_observers.clear();
        }

    protected:
        std::list<observer_ptr_t> m_observers;
};

#endif//__OBSERVABLE_H__
