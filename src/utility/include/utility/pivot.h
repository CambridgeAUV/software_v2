/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_PIVOT_H__
#define __CAUV_PIVOT_H__
#include "rounding.h"

namespace cauv {

template <typename TMin, typename TNeutral, typename TMax, typename TVal>
inline static TVal pivot(TMin min, TNeutral neutral, TMax max, TVal value){
    TVal leftRange = neutral - min;
    TVal rightRange = max - neutral;

    // left of neutral
    if (value < neutral) {
        if (leftRange == 0) return 0;
        return clamp(-1, (value - neutral) / leftRange, 0);
    // right of neutral
    } else {
        if (rightRange == 0) return 0;
        return clamp(0, (value - neutral) / rightRange, 1);
    }
}

} //namespace cauv
#endif
