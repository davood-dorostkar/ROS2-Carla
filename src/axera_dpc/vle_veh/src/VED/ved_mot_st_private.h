#ifndef RTW_HEADER_ved__mot_st_private_h_
#define RTW_HEADER_ved__mot_st_private_h_
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

#ifndef VED_SIN_POS_BRAKE
#error The variable for the parameter "VED_SIN_POS_BRAKE" is not defined
#else
#if (VED_SIN_POS_BRAKE < 0UL) || (VED_SIN_POS_BRAKE > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_BRAKE" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_GEAR
#error The variable for the parameter "VED_SIN_POS_GEAR" is not defined
#else
#if (VED_SIN_POS_GEAR < 0UL) || (VED_SIN_POS_GEAR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_GEAR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_PBRK
#error The variable for the parameter "VED_SIN_POS_PBRK" is not defined
#else
#if (VED_SIN_POS_PBRK < 0UL) || (VED_SIN_POS_PBRK > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_PBRK" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WDIR_FL
#error The variable for the parameter "VED_SIN_POS_WDIR_FL" is not defined
#else
#if (VED_SIN_POS_WDIR_FL < 0UL) || (VED_SIN_POS_WDIR_FL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WDIR_FL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WDIR_FR
#error The variable for the parameter "VED_SIN_POS_WDIR_FR" is not defined
#else
#if (VED_SIN_POS_WDIR_FR < 0UL) || (VED_SIN_POS_WDIR_FR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WDIR_FR" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WDIR_RL
#error The variable for the parameter "VED_SIN_POS_WDIR_RL" is not defined
#else
#if (VED_SIN_POS_WDIR_RL < 0UL) || (VED_SIN_POS_WDIR_RL > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WDIR_RL" is outside of the range 0UL to 4294967295UL
#endif
#endif

#ifndef VED_SIN_POS_WDIR_RR
#error The variable for the parameter "VED_SIN_POS_WDIR_RR" is not defined
#else
#if (VED_SIN_POS_WDIR_RR < 0UL) || (VED_SIN_POS_WDIR_RR > 4294967295UL)
#error The value of the variable for the parameter "VED_SIN_POS_WDIR_RR" is outside of the range 0UL to 4294967295UL
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

#ifndef VED_LONG_MOT_STATE_MOVE
#error The variable for the parameter "VED_LONG_MOT_STATE_MOVE" is not defined
#else
#if (VED_LONG_MOT_STATE_MOVE < 0) || (VED_LONG_MOT_STATE_MOVE > 255)
#error The value of the variable for the parameter "VED_LONG_MOT_STATE_MOVE" is outside of the range 0 to 255
#endif
#endif

#ifndef VED_LONG_MOT_STATE_MOVE_FWD
#error The variable for the parameter "VED_LONG_MOT_STATE_MOVE_FWD" is not defined
#else
#if (VED_LONG_MOT_STATE_MOVE_FWD < 0) || (VED_LONG_MOT_STATE_MOVE_FWD > 255)
#error The value of the variable for the parameter "VED_LONG_MOT_STATE_MOVE_FWD" is outside of the range 0 to 255
#endif
#endif

#ifndef VED_LONG_MOT_STATE_MOVE_RWD
#error The variable for the parameter "VED_LONG_MOT_STATE_MOVE_RWD" is not defined
#else
#if (VED_LONG_MOT_STATE_MOVE_RWD < 0) || (VED_LONG_MOT_STATE_MOVE_RWD > 255)
#error The value of the variable for the parameter "VED_LONG_MOT_STATE_MOVE_RWD" is outside of the range 0 to 255
#endif
#endif

#ifndef VED_LONG_MOT_STATE_STDST
#error The variable for the parameter "VED_LONG_MOT_STATE_STDST" is not defined
#else
#if (VED_LONG_MOT_STATE_STDST < 0) || (VED_LONG_MOT_STATE_STDST > 255)
#error The value of the variable for the parameter "VED_LONG_MOT_STATE_STDST" is outside of the range 0 to 255
#endif
#endif

#define CALL_EVENT (-1)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

// comment for EP40 project integration,20220726 heqiushu
// #if (UCHAR_MAX != (0xFFU)) || (SCHAR_MAX != (0x7F))
// #error \
//     "Code was generated for compiler with different sized uchar/char.
//     Consider adjusting Emulation Hardware word size settings on the Hardware
//     Implementation pane to match your compiler word sizes as defined in the
//     compiler's limits.h header file. Alternatively, you can select 'None' for
//     Emulation Hardware and select the 'Enable portable word sizes' option for
//     ERT based targets, which will disable the preprocessor word size checks."
// #endif

// #if (USHRT_MAX != (0xFFFFU)) || (SHRT_MAX != (0x7FFF))
// #error \
//     "Code was generated for compiler with different sized ushort/short.
//     Consider adjusting Emulation Hardware word size settings on the Hardware
//     Implementation pane to match your compiler word sizes as defined in the
//     compilers limits.h header file. Alternatively, you can select 'None' for
//     Emulation Hardware and select the 'Enable portable word sizes' option for
//     ERT based targets, this will disable the preprocessor word size checks."
// #endif

// #if (UINT_MAX != (0xFFFFFFFFU)) || (INT_MAX != (0x7FFFFFFF))
// #error \
//     "Code was generated for compiler with different sized uint/int. Consider
//     adjusting Emulation Hardware word size settings on the Hardware
//     Implementation pane to match your compiler word sizes as defined in the
//     compilers limits.h header file. Alternatively, you can select 'None' for
//     Emulation Hardware and select the 'Enable portable word sizes' option for
//     ERT based targets, this will disable the preprocessor word size checks."
// #endif

// #if (ULONG_MAX != (0xFFFFFFFFU)) || (LONG_MAX != (0x7FFFFFFF))
// #error \
//     "Code was generated for compiler with different sized ulong/long.
//     Consider adjusting Emulation Hardware word size settings on the Hardware
//     Implementation pane to match your compiler word sizes as defined in the
//     compilers limits.h header file. Alternatively, you can select 'None' for
//     Emulation Hardware and select the 'Enable portable word sizes' option for
//     ERT based targets, this will disable the preprocessor word size checks."
// #endif

/* QAC Fixes */

extern void ved__mot_st_Get_IO_State1(const uint8_T rtu_state_in[32],
                                      rtB_Get_IO_State1_ved__mot_st *localB,
                                      uint32_T rtp_Filter);
extern void ved__mot_st_whl_dir(const ved__mot_states_t *rtu_0,
                                ved__mot_states_t *rty_Out1);
extern void ved__mot_st_get_percentage(const uint8 rtu_veh_velo_percentage[15],
                                       rtB_get_percentage_ved__mot_st *localB);
extern void ved__mot_st_get_percentage_h(
    const uint8 rtu_veh_velo_percentage[15],
    rtB_get_percentage_ved__mot_st_n *localB);
extern void ved__mot_st_get_percentage_b(
    const uint8 rtu_veh_velo_percentage[15],
    rtB_get_percentage_ved__mot_st_k *localB);
extern void ved__mo_whl_fl_motion_percentage(
    uint8_T rtu_whl_direction,
    uint8_T rtu_whl_direction_valid,
    const uint8 rtu_whl_direction_percentage[12],
    rtB_whl_fl_motion_percentage_vd *localB);
extern void ved__mot_st_get_percentage_i(
    const uint8 rtu_whl_puls_percentage[15],
    rtB_get_percentage_ved__mot_st_m *localB);
extern void ved__mot_st_get_percentage_k(
    const uint8 rtu_whl_puls_percentage[15],
    rtB_get_percentage_ved__mot_st_g *localB);
extern void ved__mot_st_get_percentage_c(
    const uint8 rtu_whl_puls_percentage[15],
    rtB_get_percentage_ved__mot_st_p *localB);
extern void ved__mot__whl_puls_fl_percentage(
    uint8_T rtu_diff_whl_puls,
    uint8_T rtu_whl_puls_valid,
    const uint8 rtu_whl_puls_percentage[15],
    uint8_T rtu_cnt_ramp_in,
    uint8_T rtu_fwd_p,
    uint8_T rtu_ss_p,
    uint8_T rtu_rws_p,
    uint8_T rtu_cnt_delay_in,
    rtB_whl_puls_fl_percentage_ved__ *localB);

#endif /* RTW_HEADER_ved__mot_st_private_h_ */
