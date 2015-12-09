/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_TESTABLE_H__
#define __CAUV_TESTABLE_H__


/* Synopsis:
 * Make it easy to provide a safe conversion to bool, without requiring virtual
 * functions.
 * 
 * Usage:
 * class Derived: public TestableBase<Derived>{
 *     public:
 *         Derived()
 *             : TestableBase<Derived>(*this){
 *         }
 *         bool valid() const { .... }
 * };
 * 
 */

template<typename Derived_T_With_valid_func>
class TestableBase{
        typedef void (TestableBase::*bool_t)() const;
    public:
        operator bool_t() const {
            return static_cast<Derived_T_With_valid_func const*>(this)->valid()?
                &TestableBase::this_type_does_not_support_comparisons :
                NULL;
        }
    protected:
        ~TestableBase(){ }
    private:
        void this_type_does_not_support_comparisons() const {}
};

template <typename dT, typename T> 
bool operator!=(TestableBase<dT> const& l, T const&){
    l.comparison_not_allowed();	
    return false;	
} 
template <typename dT, typename T>
bool operator==(TestableBase<dT> const& l, T const&){
    l.comparison_not_allowed();
    return false;		
}


#endif  // ndef __CAUV_TESTABLE_H__
