

#include "ved_consts.h"
#include "ved_ye_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__yaw_rate_var_tune_p = 0.1F;
const real32_T ved__ye_P_curve_init_p[4] = {0.001F, 0.0F, 0.0F, 0.01F};

const real32_T ved__ye_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__ye_Q_add_p[4] = {0.0F, 0.0F, 0.0F, 0.0F};

const real32_T ved__ye_Q_curve_gain_p = 1000.0F;
const real32_T ved__ye_Q_gain_p[4] = {150000.0F, 2.0F, 0.001F, 0.05F};

const real32_T ved__ye_Q_sigmas_curve_p[2] = {0.5F, 0.5F};

const real32_T ved__ye_Q_sigmas_p[2] = {0.1F, 0.1F};

const real32_T ved__ye_Q_sigmas_velo_gain_p[4] = {1.0F, 1.0F, 0.04F, 10.0F};

const real32_T ved__ye_mahala_para_p[9] = {2.0F,  0.8F,  0.8F,  0.8F, 0.8F,
                                           0.01F, 0.01F, 0.01F, 0.01F};

const real32_T ved__ye_x_curve_init_p[2] = {0.001F, 0.001F};

const real32_T ved__ye_x_init_p[2] = {0.001F, 0.001F};

const uint8_T ved__ye_yaw_curve_ramp_para_p[2] = {5U, 10U};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */