

#ifndef RTW_HEADER_ved__gye_private_h_
#define RTW_HEADER_ved__gye_private_h_
#include "ved_consts.h"

/* Includes for objects with custom storage classes. */
#include "ved_ext.h"

/*
 * Generate compile time checks that imported macros for parameters
 * with storage class "Const" are defined
 */

/*
 * Generate compile time checks that imported macros for parameters
 * with storage class "VED__Defines" are defined
 */
#ifndef CFG_VED__INT_GYRO
#error The variable for the parameter "CFG_VED__INT_GYRO" is not defined
#else
#if (CFG_VED__INT_GYRO < 0UL) || (CFG_VED__INT_GYRO > 4294967295UL)
#error The value of the variable for the parameter "CFG_VED__INT_GYRO" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_DECREASED
#error The variable for the parameter "VED_IO_STATE_DECREASED" is not defined
#else
#if (VED_IO_STATE_DECREASED < 0UL) || (VED_IO_STATE_DECREASED > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_DECREASED" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_SUBSTITUE
#error The variable for the parameter "VED_IO_STATE_SUBSTITUE" is not defined
#else
#if (VED_IO_STATE_SUBSTITUE < 0UL) || (VED_IO_STATE_SUBSTITUE > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_SUBSTITUE" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_VALID
#error The variable for the parameter "VED_IO_STATE_VALID" is not defined
#else
#if (VED_IO_STATE_VALID < 0UL) || (VED_IO_STATE_VALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_VALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_YWR
#error The variable for the parameter "VED_SIN_POS_YWR" is not defined
#else
#if (VED_SIN_POS_YWR < 0UL) || (VED_SIN_POS_YWR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_YWR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_YWRINT
#error The variable for the parameter "VED_SIN_POS_YWRINT" is not defined
#else
#if (VED_SIN_POS_YWRINT < 0UL) || (VED_SIN_POS_YWRINT > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_YWRINT" is outside of the range 0UL to 4294967295UL
#endif
#endif

#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__gye_Get_IO_State1(const uint8_T rtu_state_in[32],
                                   rtB_Get_IO_State1_ved__gye *localB,
                                   uint32_T rtp_Filter);

#endif /* RTW_HEADER_ved__gye_private_h_ */
