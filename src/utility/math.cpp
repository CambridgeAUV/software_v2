/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/math.h>
#include <cmath> // for floor

template<>
double cauv::mod(double const& d, double const& base)
{
    if(d > 0) {
        return d - base * std::floor(d / base);
    }else{
        return d + base * std::floor(-d / base);
    }
}

template<>
float cauv::mod(float const& d, float const& base)
{
    if(d > 0) {
        return d - base * std::floor(d / base);
    }else{
        return d + base * std::floor(-d / base);
    }
}
