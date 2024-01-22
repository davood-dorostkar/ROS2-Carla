

#include "ved_consts.h"
#include "ved_aye_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__aye_P_correct_p[4] = {3.0F, 10.0F, 14.28F, 100.0F};

const real32_T ved__aye_P_init_p[16] = {1.0F, 0.0F, 0.0F, 0.0F,  0.0F,   1.0F,
                                        0.0F, 0.0F, 0.0F, 0.0F,  0.001F, 0.0F,
                                        0.0F, 0.0F, 0.0F, 0.001F};

const real32_T ved__aye_Q_add_p[16] = {0.0F, 0.0F, 0.0F, 0.0F,    0.0F,    0.0F,
                                       0.0F, 0.0F, 0.0F, 0.0F,    0.0025F, 0.0F,
                                       0.0F, 0.0F, 0.0F, 1.0E-10F};

const real32_T ved__aye_Q_gain_p[2] = {120.0F, 1.0F};

const real32_T ved__aye_Q_sigmas_p[4] = {6.0F, 6.0F, 0.0F, 0.0F};

const real32_T ved__aye_R_ay_invalid_p = 2.5E+7F;
const real32_T ved__aye_R_p[4] = {0.25F, 0.0F, 0.0F, 0.04F};

const real32_T ved__aye_x_init_p[4] = {0.0001F, 0.0001F, 1.0E-5F, 1.0E-8F};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */