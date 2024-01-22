#include <ved_consts.h>

#include "ved_aye.h"
#include "ved_aye_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* QAC Fixes */

/* Constant parameters (auto storage) */
const ConstParam_ved__aye ved__aye_ConstP = {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S18>/Constant'
     */
    {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},

    /* Expression: single(eye(4))
     * Referenced by: '<S2>/eye'
     */
    {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
     0.0F, 0.0F, 0.0F, 1.0F}};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
