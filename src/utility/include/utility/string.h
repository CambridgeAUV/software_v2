/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_STRING_H__
#define __CAUV_UTILITY_STRING_H__

#include <sstream>
#include <string>
#include <set>

#include <boost/cstdint.hpp>

class MakeString{
    public:
        MakeString()
            : m_stream(),
              m_length_limit(0){
        }
        
        MakeString(std::locale const& withLocale)
            : m_stream(),
              m_length_limit(0){
              m_stream.imbue(withLocale);
        }

        MakeString& lengthLimit(uint32_t l){
            m_length_limit = l;
            return *this;
        }
        
        template<typename T>
        MakeString(T const& v)
            : m_stream(){
            m_stream << v;
        }

        operator std::string() const{
            std::string r = m_stream.str();
            if((m_length_limit > 3) && r.size() > m_length_limit){
                r = std::string(r.begin(), r.begin() + m_length_limit-3) + "...";
            }
            return r;
        }
        
        // abbreviation for explicit cast for contexts where implicit
        // conversion is insufficient
        std::string str() const{
            return std::string(*this);
        }

        template<class T>
        MakeString& operator<<(T const& v){
            m_stream << v;
            return *this;
        }

    protected:
        std::stringstream m_stream;
        uint32_t m_length_limit;
};

typedef MakeString mkStr;

template<typename T>
std::string toStr(T const& v){
    return mkStr() << v;
}

template<typename T>
T fromStr(const char* v){
    std::istringstream s(v);
    T t;
    s >> t;
    return t;
}

namespace cauv {
std::string implode( const std::string& glue, const std::set<std::string>& pieces );
}

#endif // ndef __CAUV_UTILITY_STRING_H__

