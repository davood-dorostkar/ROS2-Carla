

#include "ved_consts.h"
#include "ved_sye_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__sye_P_SSG_init_p[9] = {0.0001F, 0.0F, 0.0F, 0.0F,    0.001F,
                                           0.0F,    0.0F, 0.0F, 1.0E-10F};

const real32_T ved__sye_P_correct_p[4] = {100.0F, 0.5F, 5.0E-7F, 0.0002F};

const real32_T ved__sye_P_di_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__sye_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__sye_Q_SSG_range_p[5] = {0.35F, 0.5F, 3.5F, 1.0E-7F,
                                            1.0E-11F};

const real32_T ved__sye_Q_SSG_sigmas_p[2] = {0.05F, 0.05F};

const real32_T ved__sye_Q_add_p[4] = {0.0F, 0.0F, 0.0F, 0.0F};

const real32_T ved__sye_Q_di_add_p[4] = {0.0F, 0.0F, 0.0F, 0.0F};

const real32_T ved__sye_Q_di_gain_p[3] = {1000.0F, 1.0F, 0.2F};

const real32_T ved__sye_Q_di_sigmas_p[2] = {0.1F, 0.1F};

const real32_T ved__sye_Q_di_sigmas_velo_gain_p[4] = {1.0F, 0.3F, 0.01F, 13.0F};

const real32_T ved__sye_Q_gain_p[3] = {120.0F, 1.0F, 0.001F};

const real32_T ved__sye_Q_sigmas_p[2] = {0.15F, 0.15F};

const real32_T ved__sye_R_SSG_p[4] = {0.0001F, 0.0F, 0.0F, 0.000324F};

const real32_T ved__sye_R_p[2] = {0.000225F, 250000.0F};

const real32_T ved__sye_x_SSG_init_p[3] = {0.01F, 0.01F, 0.007F};

const real32_T ved__sye_x_di_init_p[2] = {0.0001F, 0.0001F};

const real32_T ved__sye_x_init_p[2] = {0.0001F, 0.0001F};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */