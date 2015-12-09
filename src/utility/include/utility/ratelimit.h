/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_RATELIMIT_H__
#define __CAUV_RATELIMIT_H__
#include <list>
#include <boost/date_time/posix_time/ptime.hpp>

namespace cauv{

class RateLimiter {
public:
    RateLimiter(const unsigned int max, const unsigned int period_milliseconds);
    bool click(bool blocking = false);
    unsigned int count();
    bool isSaturated();

protected:
    std::list<boost::posix_time::ptime> m_timestamps;
    const unsigned int m_maximum;
    const unsigned int m_period;
};

} // namespace cauv

#endif
