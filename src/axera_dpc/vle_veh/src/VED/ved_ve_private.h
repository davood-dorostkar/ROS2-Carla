

#ifndef RTW_HEADER_ved__ve_private_h_
#define RTW_HEADER_ved__ve_private_h_
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

#ifndef VED_SIN_POS_VEHACL_EXT
#error The variable for the parameter "VED_SIN_POS_VEHACL_EXT" is not defined
#else
#if (VED_SIN_POS_VEHACL_EXT < 0UL) || (VED_SIN_POS_VEHACL_EXT > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_VEHACL_EXT" is outside of the range 0UL to 4294967295UL
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

#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__ve_Get_IO_State8(const uint8_T rtu_state_in[32],
                                  rtB_Get_IO_State8_ved__ve *localB,
                                  uint32_T rtp_Filter);
extern void ved__ve_check_threshold(real32_T rtu_raw_var,
                                    real32_T rtu_filt_var,
                                    real32_T *rty_out_var,
                                    rtB_check_threshold_ved__ve *localB);
extern void ved__ve_check_reset_condition(
    real32_T rtu_u_in,
    uint8_T rtu_ext_reset,
    real32_T rtu_u_filt,
    rtB_check_reset_condition_ved__v *localB);

#endif /* RTW_HEADER_ved__ve_private_h_ */
