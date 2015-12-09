/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/time.h>
#include <ostream>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

cauv::TimeStamp::TimeStamp(int32_t const& secs, int32_t const& musecs) :
    secs(secs), musecs(musecs) {}

cauv::TimeStamp::TimeStamp(void) : secs(0), musecs(0) {}

cauv::TimeStamp cauv::now(){
    cauv::TimeStamp r;
    boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = current_time - epoch;
    r.secs = diff.total_seconds();
    r.musecs = diff.fractional_seconds();
    return r;
}

double cauv::nowDouble(){
    const cauv::TimeStamp t = now();
    return double(t.secs) + double(t.musecs)/1e6;
}

std::string cauv::now(const std::string& format){
    using namespace boost::posix_time;
    std::ostringstream oss;
    time_facet* facet = new time_facet(format.c_str());
    oss.imbue(std::locale(oss.getloc(), facet));
    oss << microsec_clock::local_time();
    return oss.str();
}

int64_t cauv::millisecondsSince(cauv::TimeStamp const& t){
    cauv::TimeStamp n = now();
    return int64_t(n.secs - t.secs)*1000 + int64_t((n.musecs - t.musecs)/1000);
}

void cauv::msleep(uint32_t milliseconds){
    boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds));
}

float cauv::operator-(cauv::TimeStamp const& l, cauv::TimeStamp const& r)
{
    int secs_delta = l.secs - r.secs;
    float msecs_delta = (l.musecs - r.musecs) / 1000000.0f;
    return secs_delta + msecs_delta;
}
