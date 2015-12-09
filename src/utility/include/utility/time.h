/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_TIME_H__
#define __CAUV_UTILITY_TIME_H__

#include <boost/cstdint.hpp>
#include <string>

namespace cauv {

/* Time Stamps are seconds since the Unix Epoch, and microseconds since the
 * last whole second. Time may run backwards. Time may stand still.
 */
struct TimeStamp
{
    int32_t secs;
    int32_t musecs;
        
    TimeStamp(int32_t const& secs, int32_t const& musecs);
    TimeStamp(void);
};

/* Now, as a TimeStamp. Implemented using the operating system's posix time:
 * Time may run backwards. Time may stand still.
 */
TimeStamp now();

/* Now, as a string. The possible format flags in the string are those defined
 * by boost::date_time:
 * http://www.boost.org/doc/libs/1_49_0/doc/html/date_time/date_time_io.html#date_time.format_flags
 *
 * Here are some of the useful ones:
 *
 * %d : day of the month as decimal 01-31
 * %m : month of the year as decimal 01-12
 * %x : implementation defined date format for current locale
 * %y : two-digit year
 *
 * %H : hour 00-23
 * %M : minute 00-59
 * %s : seconds with fractional seconds: "59.999999"
 * %X : implementation defined time format for current locale
 */
std::string now(const std::string& format);

/* Equivalent to:
 * t = now();
 * return double(t.secs) + double(t.musecs)/1e6;
 */
double nowDouble();

/* NB: Can return negative values if you pass a time in the future! */
int64_t millisecondsSince(TimeStamp const& t);

void msleep(uint32_t milliseconds);

float operator-(TimeStamp const& l, TimeStamp const& r);

}

#endif // ndef __CAUV_UTILITY_TIME_H__

