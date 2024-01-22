

#include "ved_consts.h"
#include "ved_sae_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__sae_Cr_p = 80000.0F;
const real32_T ved__sae_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

const real32_T ved__sae_Q_add_p[4] = {0.0F, 0.0F, 0.0F, 0.0F};

const real32_T ved__sae_Q_gain_p[3] = {1000.0F, 1.0F, 0.2F};

const real32_T ved__sae_Q_sigmas_p[2] = {0.01F, 0.01F};

const real32_T ved__sae_R_p = 0.001F;
const real32_T ved__sae_x_init_p[2] = {0.0001F, 0.001F};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
