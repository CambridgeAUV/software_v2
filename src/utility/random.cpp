/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/random.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>


namespace { boost::mt19937 static_gen; }
int cauv::random(int min, int max)
{
	boost::uniform_int<> dist(min, max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(static_gen, dist);

	return die();
}
int cauv::random(int min, int max, unsigned int seed)
{
	boost::mt19937 gen;
	gen.seed(seed);
	boost::uniform_int<> dist(min, max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(gen, dist);

	return die();
}
