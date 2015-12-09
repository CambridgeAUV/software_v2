/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_STREAMOPS_PAIR_H__
#define __CAUV_STREAMOPS_PAIR_H__

#include <ostream>
#include <utility>

namespace std{

template<typename T1, typename T2, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::pair<T1, T2> const& a){
    os << "{" << a.first << "," << a.second << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_PAIR_H__

