

#include "ved_consts.h"
#include "ved_ve_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__ve_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__ve_Q_gain_p[3] = {0.4F, 20.0F, 8.0F};

const real32_T ved__ve_Q_sigmas_p[2] = {225.0F, 12.5F};

const real32_T ved__ve_a_v_zero_p[9] = {0.0F, 0.0F, 2.0F, 0.001F, 0.001F,
                                        0.5F, 0.0F, 0.0F, 4.0F};

const real32_T ved__ve_x_init_p = 0.0F;
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */