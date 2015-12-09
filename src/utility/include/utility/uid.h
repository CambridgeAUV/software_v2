/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_UID_H__
#define __CAUV_UTILITY_UID_H__

#include <stdint.h>

namespace cauv {

struct UID
{
    uint32_t host;
    int32_t sensor;
    int32_t seq1;
    int32_t seq2;
        
    UID();
    UID(uint32_t const& host, int32_t const& sensor, int32_t const& seq1, int32_t const& seq2);

    bool operator==(UID const& other) const;
    bool operator<(UID const& other) const;
};

UID mkUID(uint32_t sensor=0, uint64_t sequence=0);

}

#endif
