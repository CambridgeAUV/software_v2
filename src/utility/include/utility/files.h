/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef UTILITY_FILES_H_
#define UTILITY_FILES_H_

#include <string>
#include <stdexcept>

class file_error : public std::runtime_error {
    public:
    explicit file_error(const std::string& what_arg) :
        std::runtime_error(what_arg) {};
};

//create a series of directories, like mkdir -p
void create_path(std::string dir_path);

bool get_lock_file(const std::string& lockfile);

#endif
