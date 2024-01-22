#ifndef TRJPLN_TRAJPLANMATHDEFS_H
#define TRJPLN_TRAJPLANMATHDEFS_H
#include "TM_Global_Types.h"
#include "rtwtypes.h"
#include "tue_common_libs.h"

// #ifndef float32
// typedef float float32;
// #endif
// #ifndef uint8
// typedef unsigned char uint8;
// #endif
// #ifndef true
// #define true TRUE
// #endif
// #ifndef false
// #define false FALSE
// #endif
// //#include <math.h>
// #ifndef Boolean
// #define Boolean boolean_T
// #endif
#ifndef FD_ATAN
#define FD_ATAN(x) ATAN_(x)
#endif
#ifndef FD_COS
#define FD_COS(x) COS_HD_(x)
#endif
#ifndef FD_FABS
#define FD_FABS(x) TUE_CML_Abs(x)
#endif
#ifndef FD_FLOOR
#define FD_FLOOR(f_x)                         \
    (((f_x - (float32)((sint32)f_x)) >= 0.0f) \
         ? ((float32)((sint32)f_x))           \
         : ((float32)((sint32)f_x) - 1.0f))
#endif
#ifndef FD_SQR
#define FD_SQR(x) SQR(x)
#endif
#ifndef FD_THIRDPOWER
#define FD_THIRDPOWER(x) (x * x * x)
#endif
#ifndef FD_SIN
#define FD_SIN(x) SIN_HD_(x)
#endif
#ifndef FD_SQRT
#define FD_SQRT(x) SQRT(x)
#endif
#ifndef FD_TAN
#define FD_TAN(x) TAN_HD_(x)
#endif
#ifndef FD_CEIL
#define FD_CEIL(f_x)                                                           \
    (((f_x - (float32)((sint32)f_x)) > 0.0f) ? ((float32)((sint32)f_x) + 1.0f) \
                                             : ((float32)((sint32)f_x)))
#endif

#endif
