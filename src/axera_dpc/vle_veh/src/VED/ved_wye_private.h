

#ifndef RTW_HEADER_ved__wye_private_h_
#define RTW_HEADER_ved__wye_private_h_
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

#ifndef VED_IO_STATE_INVALID
#error The variable for the parameter "VED_IO_STATE_INVALID" is not defined
#else
#if (VED_IO_STATE_INVALID < 0UL) || (VED_IO_STATE_INVALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_INVALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_VALID
#error The variable for the parameter "VED_IO_STATE_VALID" is not defined
#else
#if (VED_IO_STATE_VALID < 0UL) || (VED_IO_STATE_VALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_VALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_NVM_POS_WLD
#error The variable for the parameter "VED_NVM_POS_WLD" is not defined
#else
#if (VED_NVM_POS_WLD < 0UL) || (VED_NVM_POS_WLD > 4294967295UL)
#error The value of the variable for the parameter "VED_NVM_POS_WLD" is outside of the range 0UL to 4294967295UL
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

#ifndef VED__USE_EST_WLD_DEP
#error The variable for the parameter "VED__USE_EST_WLD_DEP" is not defined
#else
#if (VED__USE_EST_WLD_DEP < 0UL) || (VED__USE_EST_WLD_DEP > 4294967295UL)
#error The value of the variable for the parameter "VED__USE_EST_WLD_DEP" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_CAL_WHS_LOAD
#error The variable for the parameter "VED_CAL_WHS_LOAD" is not defined
#else
#if (VED_CAL_WHS_LOAD < 0) || (VED_CAL_WHS_LOAD > 65535)
#error The value of the variable for the parameter "VED_CAL_WHS_LOAD" is outside of the range 0 to 65535
#endif
#endif

#ifndef VED_CAL_YWR_OFFS_DYN
#error The variable for the parameter "VED_CAL_YWR_OFFS_DYN" is not defined
#else
#if (VED_CAL_YWR_OFFS_DYN < 0) || (VED_CAL_YWR_OFFS_DYN > 65535)
#error The value of the variable for the parameter "VED_CAL_YWR_OFFS_DYN" is outside of the range 0 to 65535
#endif
#endif

#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__wye_calculatedeterminant(
    const real32_T rtu_u[4], rtB_calculatedeterminant_ved__wy *localB);
extern void ved__wye_Get_NVM_IO_State(uint32_T rtu_pos,
                                      uint32_T rtu_state_in,
                                      uint32_T rtu_VED_IOBitMask,
                                      rtB_Get_NVM_IO_State_ved__wye *localB);
extern void ved__wye_Time2Sec(uint16_T rtu_u, rtB_Time2Sec_ved__wye *localB);
extern void ved__wye_make_A_matrix(real32_T rtu_CycleTime,
                                   rtB_make_A_matrix_ved__wye *localB);
extern void ved__wye_get_bit(uint16_T rtu_value_in,
                             uint16_T rtu_bitmask,
                             rtB_get_bit_ved__wye *localB);
extern void ved__wye_Get_IO_State(const uint8_T rtu_state_in[32],
                                  rtB_Get_IO_State_ved__wye *localB,
                                  uint32_T rtp_Filter);

#endif /* RTW_HEADER_ved__wye_private_h_ */
