

#include "ved_consts.h"
#include "ved_wpp_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__wpp_P_correct_p[6] = {25.0F, 10.0F, -0.3F,
                                          1.0F,  16.0F, 100000.0F};

const real32_T ved__wpp_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__wpp_Q_gain_p[8] = {10.0F, 0.2F,   0.04F, 150.0F,
                                       0.1F,  200.0F, 0.6F,  0.08F};

const real32_T ved__wpp_Q_sigmas_p[2] = {1.0F, 1.0F};

const real32_T ved__wpp_R_p[4] = {2.0F, -10.0F, 40.0F, 0.002F};

const real32_T ved__wpp_accel_correct_p = -10.0F;
const real32_T ved__wpp_aqua_slip_correct_p[2] = {0.4F, 0.4F};

const real32_T ved__wpp_puls_velocity_para_p[4] = {15.0F, 0.02F, 15.0F, 4.0F};

const real32_T ved__wpp_x_init_p = 0.0F;

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */