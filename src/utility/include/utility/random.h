/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_RANDOM_H__
#define __CAUV_RANDOM_H__

#include <vector>
#include <set>
#include <stdexcept>

namespace cauv {

    int random(int min, int max);
    int random(int min, int max, unsigned int seed);

    template<typename T>
    std::vector<T> randomSubset(const std::vector<T>& src, typename std::vector<T>::size_type size)
    {
        if (size > src.size())
            throw std::range_error("Subset size out of range");

        std::vector<T> ret;
        std::set<int> vals;

        for (size_t j = src.size() - size; j < src.size(); ++j)
        {
            int rand = random(0, j); // generate a random integer in range [0, j]

            if (vals.find(rand) == vals.end())
                ret.push_back(src[rand]);
            else
                ret.push_back(src[j]);
        }

        return ret;
    }

    template<typename T>
    std::vector<T> randomSubset(const std::vector<T>& src, typename std::vector<T>::size_type size, unsigned int seed)
    {
        if (size > src.size())
            throw std::range_error("Subset size out of range");

        std::vector<T> ret;
        std::set<int> vals;

        for (size_t j = src.size() - size; j < src.size(); ++j)
        {
            int rand = random(0, j, seed+j); // generate a random integer in range [0, j]

            if (vals.find(rand) == vals.end())
                ret.push_back(src[rand]);
            else
                ret.push_back(src[j]);
        }

        return ret;
    }

}

#endif//__CAUV_RANDOM_H__
