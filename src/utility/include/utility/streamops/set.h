/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_STREAMOPS_SET_H__
#define __CAUV_STREAMOPS_SET_H__

#include <ostream>
#include <set>

namespace std{

template<typename T, typename Comp, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::set<T, Comp> const& s){
    os << "set[" << s.size() << "] {";
    for(typename std::set<T, Comp>::const_iterator i = s.begin(); i != s.end();){
        os << *i;
        if(++i != s.end())
            os << ", ";
    }
    os << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_SET_H__

