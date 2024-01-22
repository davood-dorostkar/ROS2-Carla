/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "dim_mod_activity_par_eba.h"
#define ASW_QM_CORE1_MODULE_START_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"
/*!  @cond Doxygen_Suppress */
const DIM_ACTIVITY_PAR_struct_t DIM_ACTIVITY_PAR_data_eba = {
    /*m/s degree/s*/
    /*BML_t_Vector2D
       fSteerAngleGradThres[DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS]*/
    {{5.0f, 200.0f}, {15.0f, 40.0f}},
    /*m/s degree/s*/
    /*BML_t_Vector2D
       fSteerAngleGradEMThres[DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS]*/
    {{0.0f, 1000.0f}, {15.0f, 400.0f}},
    /*m/s degree*/
    /*BML_t_Vector2D fFronSteerThres[DIM_ACTIVITY_PAR_fFronSteerThres_POINTS]*/
    {{0.0f, 120.0f}, {13.9f, 120.0f}},
    /*m/s degree/*/
    /*BML_t_Vector2D
       fSteeringAngleGradFilterThres[DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS]*/
    {{(5.0f / 3.6f), 45.0f}, {(40.0f / 3.6f), 23.0f}},
    /*fFronSteerThresStraight*/ 15.0f,
    /*fFronSteerGradThresStraight*/ 10.0f,
    /*fGradShutDownTime*/ 2.0f,
    /*fAngleShutDownTime*/ 1.0f,
    /*fGradHoldTime*/ 0.2f,
    /*fAngleHoldTime*/ 0.2f,
    /*fGradShutDownTimeEM*/ 1.0f,
    /*fGradPeakTime*/ 0.26f,
    /*fSteeringGradFiltHoldTime*/ 0.1f,
    /* fFilterFactorSteeringGrad*/ 0.01f};

/*! @endcond */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE1_MODULE_STOP_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"