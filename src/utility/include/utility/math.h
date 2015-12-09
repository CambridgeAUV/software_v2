/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_MATH_H__
#define __CAUV_MATH_H__

namespace cauv {
    
    /* return smallest positive value in all cases */
    template<typename T>
    T mod(T const& d, T const& base)
    {
        T ret = d % base;
        if (ret < 0)
            ret += base;
        return ret;
    }
    
    /* return closest value to zero in all cases */
    template<typename T>
    T angleMod(T const& d, T const& base){
        T ret = mod(d, base);
        if(ret > base/2)
            ret -= base;
        return ret;
    }
   
    template<>
    double mod(double const& d, double const& base);
    
    template<>
    float mod(float const& d, float const& base);
    
    template<typename T>
    static inline T degrees(T const& radians){
        return radians * 180 / 3.14159265358979323846;
    }

    template<typename T>
    static inline T radians(T const& degrees){
        return degrees * 3.14159265358979323846 / 180;
    }

    template<typename T>
    static inline T sq(T const& val)
    {
        return val*val;
    }
}

#endif//__CAUV_MATH_H__
