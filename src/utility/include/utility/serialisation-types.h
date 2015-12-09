/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_SERIALISATION_TYPES_H__
#define __CAUV_SERIALISATION_TYPES_H__

#include <vector>
#include <boost/shared_ptr.hpp>

namespace cauv{

typedef unsigned char byte;
typedef std::vector<byte> svec_t;
typedef boost::shared_ptr<svec_t> svec_ptr;
typedef boost::shared_ptr<const svec_t> const_svec_ptr;

} // namespace cauv

#endif // ndef __CAUV_SERIALISATION_TYPES_H__

