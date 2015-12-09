/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_DEFER_H__
#define __CAUV_UTILITY_DEFER_H__

//TODO: think plain boost::bind can be used to achieve this...

/**
 * Deferred evaluation of functions.
 *
 * Implicit conversion of a DeferHolder object to the return type of the
 * function that it wraps is provided.
 */

#include <boost/function.hpp>

/* defer evaluation of return_T foo(); */
template<typename return_T>
class DeferHolder{
    public:
        typedef boost::function<return_T()> function_t;
        DeferHolder(function_t const& func)
            : m_func(func){
        }
        
        template<typename function_object_T>
        DeferHolder(function_object_T const& foo)
            : m_func(function_t(foo)){
        }

        operator return_T() const{
            return m_func();
        }

    private:
        function_t m_func;
};

template<typename return_T>
DeferHolder<return_T> Defer(boost::function<return_T()> const& f){
    return DeferHolder<return_T>(f);
}

/**
 * try to shoehorn anything else into a function_t: this catches
 * boost::bind<>() things, which are convertible to (but are not)
 * boost::function objects
 */
template<typename return_T, typename foo_T>
DeferHolder<return_T> Defer(foo_T const& f){
    return DeferHolder<return_T>(f);
}

#endif // __CAUV_UTILITY_DEFER_H__

