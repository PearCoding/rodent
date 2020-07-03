#pragma once

#ifdef WIN32
#include <direct.h>
#define create_directory(d) _mkdir(d)
#else
#include <sys/stat.h>
#define create_directory(d) \
    {                       \
        umask(0);           \
        mkdir(d, 0777);     \
    }
#endif