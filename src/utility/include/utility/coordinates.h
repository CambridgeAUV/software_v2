/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_COORDINATES_H__
#define __CAUV_UTILITY_COORDINATES_H__

namespace cauv{

namespace coords{
// principal component of Earth along polar and equatorial axis (WGS84 coordinate system)
const double Earth_B = 6356752.314245; // polar
const double Earth_A = 6378137.00; // equatorial

double metresPerDegreeLongitude(double const& at_latitude_degrees);
double metresPerDegreeLatitude(double const& at_latitude_degrees);

} // namespace coords

} // namespace cauv

#endif // ndef __CAUV_UTILITY_COORDINATES_H__
