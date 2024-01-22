

#include "ved_consts.h"
#include "ved_wye_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__wye_P_correct_p[4] = {3.0F, 10.0F, 14.28F, 100.0F};

const real32_T ved__wye_P_init_p[9] = {1.0F, 0.0F, 0.0F, 0.0F,   1.0F,
                                       0.0F, 0.0F, 0.0F, 1.0E-6F};

const real32_T ved__wye_P_init_wld_p[9] = {1.0F, 0.0F, 0.0F, 0.0F,   1.0F,
                                           0.0F, 0.0F, 0.0F, 1.0E-6F};

const real32_T ved__wye_Q_gain_p[3] = {120.0F, 1.0F, 1.0E-7F};

const real32_T ved__wye_Q_gain_wld_p[3] = {120.0F, 1.0F, 1.0E-7F};

const real32_T ved__wye_Q_sigmas_dyn_off_p[5] = {14.0F, 2.5F, 1.0E-6F, 2.0F,
                                                 0.02F};

const real32_T ved__wye_Q_sigmas_dyn_off_wld_p = 1.0E-11F;
const real32_T ved__wye_Q_sigmas_p[3] = {0.04F, 0.04F, 8.0E-5F};

const real32_T ved__wye_Q_sigmas_wld_p[3] = {14.0F, 3.5F, 1.0E-6F};

const real32_T ved__wye_R_control_p = 10.0F;
const real32_T ved__wye_R_p[9] = {1.0E-6F, 0.0F, 0.0F, 0.0F,   4.0E-6F,
                                  0.0F,    0.0F, 0.0F, 0.0001F};

const real32_T ved__wye_R_wld_p[4] = {9.0E-6F, 0.0F, 0.0F, 4.0E-6F};

const real32_T ved__wye_x_init_p[3] = {0.0001F, 0.0001F, 1.0E-5F};

const real32_T ved__wye_x_init_wld_p[3] = {0.0001F, 0.0001F, 1.5F};

const real32_T ved__wye_yaw_P_correct_p[5] = {3.0F, 10.0F, 14.28F, 100.0F,
                                              0.0004F};

const real32_T ved__wye_yaw_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__wye_yaw_Q_gain_p[3] = {150.0F, 1.0F, 0.2F};

const real32_T ved__wye_yaw_Q_sigmas_p[2] = {0.15F, 0.15F};

const real32_T ved__wye_yaw_R_p[4] = {0.0001F, 0.0F, 0.0F, 0.000121F};

const real32_T ved__wye_yaw_offset_const_p[2] = {0.002F, 30.0F};

const real32_T ved__wye_yaw_x_init_p[2] = {0.0001F, 0.0001F};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
