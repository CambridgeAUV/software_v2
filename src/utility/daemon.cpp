/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <utility/daemon.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>


void daemonize(void) {
    int pid = fork();
    if(pid == 0) {
        if(fork() > 0) {
            exit(0);
        }
    } else if (pid < 0) {
        printf("fork unsuccessful!\n");
        exit(1);
    } else {
        exit(0);
    }
    chdir("/");
    umask(0000);
    freopen("/dev/null","r",stdin);
    freopen("/dev/null","w",stdout);
    freopen("/dev/null","w",stderr);
}
