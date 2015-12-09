/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_STREAMOPS_ARRAY_H__
#define __CAUV_STREAMOPS_ARRAY_H__

#include <ostream>
#include <boost/array.hpp>

namespace std{

template<typename T, typename char_T, typename traits, size_t Size>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, boost::array<T,Size> const& a){
    os << "array[" << a.size() << "] {";
    for(typename boost::array<T,Size>::const_iterator i = a.begin(); i != a.end();){
        os << *i;
        if(++i != a.end())
            os << ", ";
    }
    os << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_ARRAY_H__

