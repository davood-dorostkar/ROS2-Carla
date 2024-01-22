#include "ved_consts.h"

#include "ved_gye_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const real32_T ved__gye_P_correct_p[2] = {100.0F, 4.0E-5F};

const real32_T ved__gye_P_init_p[4] = {1.0F, 0.0F, 0.0F, 1.0F};

// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/54 changed by guotao
// start
const real32_T ved__gye_Q_gain_p = 1.0E-10F;  // 30000.0F;
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/54 changed by guotao
// end
const real32_T ved__gye_R_p[6] = {2.5E-5F, 2.5E+7F,  5.29E-6F,
                                  0.004F,  0.00017F, 4.5E-5F};

const real32_T ved__gye_x_init_p[2] = {0.0F, 0.0F};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */