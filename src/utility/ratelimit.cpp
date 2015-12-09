/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <utility/ratelimit.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <utility/time.h>


cauv::RateLimiter::RateLimiter(const unsigned int max, const unsigned int period) : m_maximum(max), m_period(period) {
}

bool cauv::RateLimiter::click(bool blocking){

    // check the whether we've had too many requests this period
    while(isSaturated()) {
        // if blocking hasn't been requested we don't need to loop, just
        // inform the caller that the click has been denied
        if(!blocking) return false;

        // otherwise sleep until it's time to remove the element at the end of the list
        boost::posix_time::time_duration diff;
        diff = boost::posix_time::microsec_clock::universal_time() - m_timestamps.back();
        cauv::msleep(diff.total_milliseconds());
    }

    // once we're below the rate limit, allow the click to happen and
    // register a new timestamp for this click
    boost::posix_time::ptime current = boost::posix_time::microsec_clock::universal_time();
    m_timestamps.push_front(current);
    return true;
}

bool cauv::RateLimiter::isSaturated() {
    return count() >= m_maximum;
}

unsigned int cauv::RateLimiter::count(){
    // remove any timestamps older than now() - m_period
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime cutoff = now - boost::posix_time::milliseconds(m_period);
    while(m_timestamps.size() && (m_timestamps.back() < cutoff)){
        m_timestamps.pop_back();
    }
    return m_timestamps.size();
}

