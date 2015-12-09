/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_ARRAYS_H__
#define __CAUV_UTILITY_ARRAYS_H__

#define MAX_ARRAY_SIZE 10

#include <boost/array.hpp>
#include <boost/preprocessor/repetition.hpp>

namespace cauv {

#define MAKE_ARRAY(z, N, unused) \
template<typename T> \
boost::array<T,N> make_array(BOOST_PP_ENUM_PARAMS(N,T val)) { \
    boost::array<T,N> ret = {{ BOOST_PP_ENUM_PARAMS(N,val) }}; \
    return ret; \
}

BOOST_PP_REPEAT(MAX_ARRAY_SIZE, MAKE_ARRAY, ~)

#undef MAKE_ARRAY

}

#endif
