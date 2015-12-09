/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_STREAMOPS_LIST_H__
#define __CAUV_STREAMOPS_LIST_H__

#include <ostream>
#include <list>

namespace std{

template<typename T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::list<T> const& a){
    os << "list[" << a.size() << "] {";
    for(typename std::list<T>::const_iterator i = a.begin(); i != a.end();){
        os << *i;
        if(++i != a.end())
            os << ", ";
    }
    os << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_LIST_H__

