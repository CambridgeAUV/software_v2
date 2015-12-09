/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/coordinates.h>

#include <cmath>

#include <utility/math.h>

namespace cc = cauv::coords;

// The reference implementation for this was python utils/coordinates.py it is
// intended to be identical.

// See
// en.wikipedia.org/wiki/Geographic_coordinate_system#Expressing_latitude_and_longitude_as_linear_units
double cc::metresPerDegreeLongitude(double const& at_latitude_degrees){
    const double tan_lat = std::tan(radians(at_latitude_degrees));
    const double beta = std::atan((Earth_B/Earth_A)*tan_lat);
    return std::fabs(Earth_A * std::cos(beta) * radians(1.0));
}

double cc::metresPerDegreeLatitude(double const& /*at_latitude_degrees*/){
    // TODO: maybe improve approx; atm 1st degree    
    return Earth_B * radians(1.0);
}

