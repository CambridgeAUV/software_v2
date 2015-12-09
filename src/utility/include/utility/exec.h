/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_UTILITY_EXEC_H__
#define __CAUV_UTILITY_EXEC_H__

#include <string>
#include <cstdio>

namespace cauv{

inline static std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if(!pipe){
        return "";
    }
    char buffer[4096];
    std::string result = "";
    while(!std::feof(pipe))
        if(std::fgets(buffer, 4096, pipe))
            result += buffer;
    pclose(pipe);
    return result;
}

} // namespace cauv

#endif // ndef __CAUV_UTILITY_EXEC_H__
