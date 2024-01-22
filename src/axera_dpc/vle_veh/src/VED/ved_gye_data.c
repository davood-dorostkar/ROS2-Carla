#include "ved_consts.h"

#include "ved_gye.h"
#include "ved_gye_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* QAC Fixes */

/* Invariant block signals (auto storage) */
const ConstBlockIO_ved__gye ved__gye_ConstB = {
    {1.0F, 0.0F}
    /* '<S2>/Ht' */
};

/* Constant parameters (auto storage) */
const ConstParam_ved__gye ved__gye_ConstP = {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S16>/Constant'
     */
    {0.0F, 0.0F},

    /* Computed Parameter: H_const1_Value
     * Referenced by: '<S2>/H_const1'
     */
    {1.0F, 0.0F, 0.0F, 1.0F},

    /* Computed Parameter: H_const_Value
     * Referenced by: '<S2>/H_const'
     */
    {1.0F, 0.0F},

    /* Expression: mgainval
     * Referenced by: '<S7>/Weighted Moving Average'
     */
    {0.125F, 0.125F, 0.125F, 0.125F, 0.125F, 0.125F, 0.125F, 0.125F}};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */