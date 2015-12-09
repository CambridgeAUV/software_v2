/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/uid.h>
#include <utility/exec.h>

#include <string>
#include <sstream>
#include <algorithm>

cauv::UID::UID() { }
cauv::UID::UID(uint32_t const& host, int32_t const& sensor, int32_t const& seq1, int32_t const& seq2) : 
    host(host), sensor(sensor), seq1(seq1), seq2(seq2) {}

bool cauv::UID::operator==(cauv::UID const& other) const
{
    return
        host   == other.host   &&
        sensor == other.sensor &&
        seq1   == other.seq1   &&
        seq2   == other.seq2;
}

bool cauv::UID::operator<(cauv::UID const& other) const
{
    if(host < other.host)
        return true;
    else if(!(host == other.host))
        return false;
    if(sensor < other.sensor)
        return true;
    else if(!(sensor == other.sensor))
        return false;
    if(seq1 < other.seq1)
        return true;
    else if(!(seq1 == other.seq1))
        return false;
    if(seq2 < other.seq2)
        return true;
    else if(!(seq2 == other.seq2))
        return false;
    return false;
}

static uint32_t getHostID(){
    // Don't look: this gets the last 32 bits of each of the mac addresses on
    // the system, as strings...
    // (if you can do this faster and neater without the system() in a *nix
    //  compatible way, go for it...)
    std::string ids = cauv::exec(
        "(ifconfig 2>/dev/null || ip addr 2>/dev/null) | grep -o --color=never ..:..:..:..:..:.."
    );
    std::string::iterator end = std::remove(ids.begin(), ids.end(), ':');
    std::string::iterator mac_start, mac_end;
    for (mac_start = ids.begin(); mac_start != end; ) {
        mac_end = std::find(mac_start, end, '\n');
        std::string mac(mac_start, mac_end);
        if (mac != "000000000000" && mac != "ffffffffffff" && mac.size() == 12) {
            uint32_t r;
            std::stringstream s;
            s << std::hex << mac.substr(4,8);
            s >> r;
            return r;
        }
        mac_start = mac_end + 1;
    }
    return 0xdeadbeef;
}

cauv::UID cauv::mkUID(uint32_t sensor, uint64_t sequence){
    static uint32_t cached_host_id = getHostID();
    return cauv::UID(cached_host_id, sensor, uint32_t(sequence>>32), uint32_t(sequence));
}
