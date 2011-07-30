/*
 * File:   Util.h
 * Author: afaucher
 *
 * Created on May 30, 2009, 3:03 PM
 */

#ifndef _TANKTANK_UTIL_H
#define _TANKTANK_UTIL_H

#define NORMAL "\033[0m"

#define RED "\033[0;31m"
#define YELLOW "\033[0;33m"
#define GREEN "\033[0;32m"

#define BLUE "\033[0;34m"
#define CYAN "\033[0;36m"
#define MAGENTA "\033[0;35m"
#define WHITE "\033[0;37m"
#define LIGHT_GREEN "\033[0;92m"
#define LIGHT_YELLOW "\033[0;93m"
#define LIGHT_BLUE "\033[0;94m"
#define LIGHT_RED "\033[0;91m"
#define LIGHT_PURPLE "\033[0;95m"

#define BOLD "\033[1m"
#define ITALICS "\033[3m"

#define TRACE(fmt, args...) printf( LIGHT_BLUE "%s:%s:%d " NORMAL fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##args)
#define INFO(fmt, args...) printf( BLUE "%s:%s:%d " NORMAL fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##args)
#define SUCCESS(fmt, args...) printf( GREEN "%s:%s:%d " NORMAL fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##args)
#define WARNING(fmt, args...) printf( YELLOW "WARNING %s:%s:%d " NORMAL fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##args)
#define FAILURE(fmt, args...) printf( RED "FAILURE %s:%s:%d " NORMAL fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##args)

#define CHECK(condition, ret) { \
        bool _check = (condition); \
        if (!_check) { \
            FAILURE("Failed check " #condition); \
            return ret; \
        } \
    }

#define NAMEVAL(a,b,c,d) ( (((Uint32)a)<<24) | (((Uint32)b)<<16) | (((Uint32)c)<<8) | ((Uint32)d) )
#define NAMEVAL_FORMAT "%c%c%c%c"
char printable( char val );
#define NAMEVAL_ARGS(val) printable((val>>24)&0xff), printable((val>>16)&0xff), printable((val>>8)&0xff), printable((val)&0xff)

#define MAX_PLAYERS 6

#ifdef WIN32
#define SIZE_T_FORMAT "%u"
#else
#define SIZE_T_FORMAT "%Zd"
#endif

#define TANKTANK_VERSION_STRING "0.3.4"
#define TANKTANK_VERSION_NUMBER {0,3,4,0}

#endif /* _TANKTANK_UTIL_H */

