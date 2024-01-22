
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "dim_mod_feedback_par_eba.h"
#define ASW_QM_CORE1_MODULE_START_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"
/*!  @cond Doxygen_Suppress */
const DIM_FEEDBACK_PAR_struct_t DIM_FEEDBACK_PAR_data_eba = {
    {                /* s, %*/
     {0.0f, -70.0f}, /* fWeakNegCurve, Timer is running from 1.0 -> 0.0 sec */
     {1.0f, -89.0f}},
    /*fHighGasPedalPos*/ 90.0f,
    /*fHighGasPedalGrad*/ 250.0f,
    /*fHighGasPedalPosWeakNegH*/ 90.0f,
    /*fHighGasPedalPosWeakNegL*/ 0.0f,
    /*fHighGasPedalGradWeakNeg*/ 250.0f,
    /*fAutoBrakeJerkLimit*/ -4.0f,
    /*fAutoBrkTimeNoPosFdbck*/ 0.35f,
    /*fVeryHighGasPedalPos*/ 90.0f,
    /*fVeryHighGasPedalGrad*/ 250.0f,
    /*fGasPedalUsedPos*/ 1.0f,
    /*fGasPedelGradNegative*/ -5.0f,
    /*fNegFdbckTime*/ 1.0f,
    /*fPosFdbckTime*/ 3.0f};

/*! @endcond */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE1_MODULE_STOP_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"
