/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <utility/files.h>

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>


void create_path(std::string dir_path) {
    for(size_t ii = 1; ii <= dir_path.size(); ii++) {
        if (dir_path[ii] == '/' || ii == dir_path.size()) {
            if (ii != dir_path.size()) {
                dir_path[ii] = '\0';
            }
            int err;
            //this a little bit of an ugly hack since some nodes run as root
            err = mkdir(dir_path.c_str(), 0777);
            if (!(err == 0 || (err == -1 && errno == EEXIST))) {
                fprintf(stderr,"error creating dir %s: %d %s\n",
                        dir_path.c_str(),errno,strerror(errno));
                throw file_error("Could not create directory!");
            }
            if (ii != dir_path.size()) {
                dir_path[ii] = '/';
            }
        }
    }
}

bool get_lock_file(const std::string& lockfile) {
    int lockfile_fd;
    lockfile_fd = open(lockfile.c_str(), O_RDWR | O_CREAT, 0600);

    if (lockfile_fd < 0) {
        fprintf(stderr,"Could not open lock file %s: %d %s\n",lockfile.c_str(),errno,strerror(errno));
        throw file_error("Could not open lock file!");
    }

    int ret = lockf(lockfile_fd, F_TLOCK, 0);
    if (ret < 0) {
        if (errno == EAGAIN) {
            return false;
        } else {
            fprintf(stderr,"Could not lock file %s: %d %s\n",lockfile.c_str(),errno,strerror(errno));
            throw file_error("error getting lock on lock file (not EAGAIN)");
        }
    } else {
        return true;
    }
}
