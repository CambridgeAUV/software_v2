/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_LOCK_MACRO_H__
#define __CAUV_LOCK_MACRO_H__
#include <boost/typeof/typeof.hpp>

#define CAUV_LOCK(MUTEX) if (bool _lock_guard_bool = false) {} \
    else for (boost::lock_guard<BOOST_TYPEOF(MUTEX)> _lock_guard(MUTEX); !_lock_guard_bool; _lock_guard_bool = true)

#endif
