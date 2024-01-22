

#ifndef RTW_HEADER_ved__wpp_private_h_
#define RTW_HEADER_ved__wpp_private_h_
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
#ifndef VED_IO_STATE_VALID
#error The variable for the parameter "VED_IO_STATE_VALID" is not defined
#else
#if (VED_IO_STATE_VALID < 0UL) || (VED_IO_STATE_VALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_VALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WTCKS_FL
#error The variable for the parameter "VED_SIN_POS_WTCKS_FL" is not defined
#else
#if (VED_SIN_POS_WTCKS_FL < 0UL) || (VED_SIN_POS_WTCKS_FL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WTCKS_FL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WTCKS_FR
#error The variable for the parameter "VED_SIN_POS_WTCKS_FR" is not defined
#else
#if (VED_SIN_POS_WTCKS_FR < 0UL) || (VED_SIN_POS_WTCKS_FR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WTCKS_FR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WTCKS_RL
#error The variable for the parameter "VED_SIN_POS_WTCKS_RL" is not defined
#else
#if (VED_SIN_POS_WTCKS_RL < 0UL) || (VED_SIN_POS_WTCKS_RL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WTCKS_RL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WTCKS_RR
#error The variable for the parameter "VED_SIN_POS_WTCKS_RR" is not defined
#else
#if (VED_SIN_POS_WTCKS_RR < 0UL) || (VED_SIN_POS_WTCKS_RR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WTCKS_RR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WVEL_FL
#error The variable for the parameter "VED_SIN_POS_WVEL_FL" is not defined
#else
#if (VED_SIN_POS_WVEL_FL < 0UL) || (VED_SIN_POS_WVEL_FL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WVEL_FL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WVEL_FR
#error The variable for the parameter "VED_SIN_POS_WVEL_FR" is not defined
#else
#if (VED_SIN_POS_WVEL_FR < 0UL) || (VED_SIN_POS_WVEL_FR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WVEL_FR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WVEL_RL
#error The variable for the parameter "VED_SIN_POS_WVEL_RL" is not defined
#else
#if (VED_SIN_POS_WVEL_RL < 0UL) || (VED_SIN_POS_WVEL_RL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WVEL_RL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WVEL_RR
#error The variable for the parameter "VED_SIN_POS_WVEL_RR" is not defined
#else
#if (VED_SIN_POS_WVEL_RR < 0UL) || (VED_SIN_POS_WVEL_RR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WVEL_RR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__wpp_Get_IO_State10(const uint8_T rtu_state_in[32],
                                    rtB_Get_IO_State10_ved__wpp *localB,
                                    uint32_T rtp_Filter);
extern void wheel_speed_fusion_Init(rtDW_wheel_speed_fusion *localDW);
extern void wheel_speed_fusion_Start(rtDW_wheel_speed_fusion *localDW);
extern void wheel_speed_fusion(real32_T rtu_wheel_velocity,
                               real32_T rtu_CycleTime,
                               real32_T rtu_whl_Circumference,
                               uint8_T rtu_number_wheel_pulse,
                               uint8_T rtu_wheel_velocity_valid,
                               uint8_T rtu_use_pulse,
                               real32_T rtu_aqua_slip_correction,
                               uint8_T rtu_diff_wheel_pulse,
                               rtB_wheel_speed_fusion *localB,
                               rtDW_wheel_speed_fusion *localDW);

#endif /* RTW_HEADER_ved__wpp_private_h_ */
