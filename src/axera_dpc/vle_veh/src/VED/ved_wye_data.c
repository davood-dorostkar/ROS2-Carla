
#include "ved_consts.h"
#include "ved_wye.h"
#include "ved_wye_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Invariant block signals (auto storage) */
const ConstBlockIO_ved__wye ved__wye_ConstB = {
    {0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F}, /* '<S2>/Ht' */

    {1.0F, 0.0F, 1.0F, 0.0F}
    /* '<S3>/Ht' */
};

/* Constant parameters (auto storage) */
const ConstParam_ved__wye ved__wye_ConstP = {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S22>/Constant'
     */
    {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},

    /* Computed Parameter: Constant_Value_d
     * Referenced by: '<S28>/Constant'
     */
    {0.0F, 0.0F, 0.0F, 0.0F},

    /* Computed Parameter: Constant_Value_a
     * Referenced by: '<S44>/Constant'
     */
    {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},

    /* Pooled Parameter (Expression: )
     * Referenced by:
     *   '<S2>/eye'
     *   '<S29>/eye'
     */
    {1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},

    /* Computed Parameter: H_const_Value
     * Referenced by: '<S2>/H_const'
     */
    {0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 1.0F},

    /* Computed Parameter: eye_Value
     * Referenced by: '<S3>/eye'
     */
    {1.0F, 0.0F, 0.0F, 1.0F},

    /* Computed Parameter: H_const_Value_b
     * Referenced by: '<S3>/H_const'
     */
    {1.0F, 1.0F, 0.0F, 0.0F}};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */