#ifndef _MATH_SCRAPS_H_
#define _MATH_SCRAPS_H_

#include <math.h>
#include <stdint.h>
#include <cmath>

// Output
#ifdef DEBUG
    #ifdef ANDROID
        #define vinfo(...) ((void)__android_log_print(ANDROID_LOG_INFO, __FUNCTION__, __VA_ARGS__))
        #define vwarn(...) ((void)__android_log_print(ANDROID_LOG_WARN, __FUNCTION__, __VA_ARGS__))
    #else
        #define vinfo(...)  printf(__VA_ARGS__)
        #define vwarn(...)  fprintf(stderr, __VA_ARGS__);
    #endif
#else
    #define vinfo(...) ((void)0)
    #define vwarn(...) ((void)0)
#endif

#ifdef ANDROID
    #define verr(...) ((void)__android_log_print(ANDROID_LOG_ERROR, __FUNCTION__, __VA_ARGS__))
#else
    #define verr(...)   fprintf(stderr, __VA_ARGS__);
#endif

#endif
