

#ifndef RTW_HEADER_ved__sye_private_h_
#define RTW_HEADER_ved__sye_private_h_
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
#ifndef VED_IO_STATE_INIT
#error The variable for the parameter "VED_IO_STATE_INIT" is not defined
#else
#if (VED_IO_STATE_INIT < 0UL) || (VED_IO_STATE_INIT > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_INIT" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_INVALID
#error The variable for the parameter "VED_IO_STATE_INVALID" is not defined
#else
#if (VED_IO_STATE_INVALID < 0UL) || (VED_IO_STATE_INVALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_INVALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_NOTAVAIL
#error The variable for the parameter "VED_IO_STATE_NOTAVAIL" is not defined
#else
#if (VED_IO_STATE_NOTAVAIL < 0UL) || (VED_IO_STATE_NOTAVAIL > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_NOTAVAIL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_IO_STATE_VALID
#error The variable for the parameter "VED_IO_STATE_VALID" is not defined
#else
#if (VED_IO_STATE_VALID < 0UL) || (VED_IO_STATE_VALID > 4294967295UL)
#error The value of the variable for the parameter "VED_IO_STATE_VALID" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_NVM_POS_SSG
#error The variable for the parameter "VED_NVM_POS_SSG" is not defined
#else
#if (VED_NVM_POS_SSG < 0UL) || (VED_NVM_POS_SSG > 4294967295UL)
#error The value of the variable for the parameter "VED_NVM_POS_SSG" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_LATA
#error The variable for the parameter "VED_SIN_POS_LATA" is not defined
#else
#if (VED_SIN_POS_LATA < 0UL) || (VED_SIN_POS_LATA > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_LATA" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_SWA
#error The variable for the parameter "VED_SIN_POS_SWA" is not defined
#else
#if (VED_SIN_POS_SWA < 0UL) || (VED_SIN_POS_SWA > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_SWA" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED__USE_LEARNED_UNDERSTEER_GRAD
#error The variable for the parameter "VED__USE_LEARNED_UNDERSTEER_GRAD" is not defined
#else
#if (VED__USE_LEARNED_UNDERSTEER_GRAD < 0UL) || \
    (VED__USE_LEARNED_UNDERSTEER_GRAD > 4294967295UL)
#error The value of the variable for the parameter "VED__USE_LEARNED_UNDERSTEER_GRAD" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_CAL_SWA_GRAD
#error The variable for the parameter "VED_CAL_SWA_GRAD" is not defined
#else
#if (VED_CAL_SWA_GRAD < 0) || (VED_CAL_SWA_GRAD > 65535)
#error The value of the variable for the parameter "VED_CAL_SWA_GRAD" is outside of the range 0 to 65535
#endif
#endif

#ifndef VED_CTRL_STATE_INIT
#error The variable for the parameter "VED_CTRL_STATE_INIT" is not defined
#else
#if (VED_CTRL_STATE_INIT < 0) || (VED_CTRL_STATE_INIT > 65535)
#error The value of the variable for the parameter "VED_CTRL_STATE_INIT" is outside of the range 0 to 65535
#endif
#endif

#define CALL_EVENT (-1)

/* QAC Fixes */

extern void ved__sye_make_A_matrix(real32_T rtu_CycleTime,
                                   rtB_make_A_matrix_ved__sye *localB);
extern void ved__sye_Time2Sec(uint16_T rtu_u, rtB_Time2Sec_ved__sye *localB);
extern void ved__sye_get_gain_bias(const real32_T rtu_q_gain[3],
                                   rtB_get_gain_bias_ved__sye *localB);
extern void ved__sye_Get_IO_State1(const uint8_T rtu_state_in[32],
                                   rtB_Get_IO_State1_ved__sye *localB,
                                   uint32_T rtp_Filter);
extern void ved__sye_get_init_control_mode(
    uint16_T rtu_ved__ctrl_mode, rtB_get_init_control_mode_ved__s *localB);
extern void ved__sye_Get_NVM_IO_State(uint32_T rtu_pos,
                                      uint32_T rtu_state_in,
                                      uint32_T rtu_VED_IOBitMask,
                                      rtB_Get_NVM_IO_State_ved__sye *localB);
extern void ved__sye_At(const real32_T rtu_u[4], rtB_At_ved__sye *localB);
extern void ved__sye_calculatethegain(const real32_T rtu_In2[2],
                                      real32_T rtu_In1,
                                      real32_T rty_Out1[2]);
extern void ved__sye_calculatedeterminant(
    real32_T rtu_u, rtB_calculatedeterminant_ved__sy *localB);
extern void ved__sye_Ht(const real32_T rtu_u[2], rtB_Ht_ved__sye *localB);
extern void ved__sye_Reset_P_pred(const real32_T rtu_P_pred_in[4],
                                  real32_T rtu_u_det,
                                  rtB_Reset_P_pred_ved__sye *localB);
extern void ved__sye_eye(rtB_eye_ved__sye *localB);

#endif /* RTW_HEADER_ved__sye_private_h_ */
