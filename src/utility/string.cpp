/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <utility/string.h>

std::string cauv::implode( const std::string& glue, const std::set<std::string>& pieces )
{
    std::string a;
    int leng=pieces.size();
    
    std::set<std::string>::iterator i;
    for (i=pieces.begin(); i!=pieces.end(); i++){
        a += *i;
        if (--leng)
            a += glue;
    }
    return a;
}

