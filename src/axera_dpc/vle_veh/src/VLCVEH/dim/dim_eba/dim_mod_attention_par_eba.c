/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "dim_mod_attention_par_eba.h"
#define ASW_QM_CORE1_MODULE_START_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"
/*!  @cond Doxygen_Suppress */
const DIM_ATTENTION_PAR_struct_t DIM_ATTENTION_PAR_data_eba = {
    /*! velocity dependent acceleration for very high attention */
    /*float32
       DIM_ATTENTION_PAR_AccelCurve[2*DIM_ATTENTION_PAR_AccelCurve_POINTS]*/
    {/*  m/s, m/s^2*/
     {8.0f, 3.8f},
     {11.1f, 2.25f},
     {20.0f, 2.25f},
     {50.0f, 1.0f}},
    /*! velocity dependent gas pedal position for high attention*/
    /*float32
       DIM_ATTENTION_PAR_GasPedalPosCurve[2*DIM_ATTENTION_PAR_GasPedalPosCurve_POINTS]*/
    {/*m/s; percent */
     {25.0f, 40.0f},
     {45.0f, 80.0f}},
    /*fGasPedalGradLowPositive*/ 150.0f,
    /*fGasPedalGradLowNegative*/ -180.0f,
    /*fGasPedalLowNegMeasured*/ -4.0f,
    /*fGasPedalLowNegMeasuredPos*/ 1.0f,
    /*fRobotControlledVelocityThresh*/ 0.6f,
    /*fNoGasPedalGradientThresh*/ 15.0f,
    /*fSteeringGradLow*/ 100.0f,
    /*fVeryHighTime*/ 1.5f,
    /*fHigherTime */ 1.5f,
    /*fHighTime*/ 1.0f,
    /*fLowTime*/ 2.5f,
    /*fRobotControlledVelocityTime*/ 2.0f,
    /*fNoGasPedalGradientTime*/ 1.0f,
    /*fLowKeepRobotTime*/ 0.0f,
    /*fTurnIndicatorMinVelocity */ 50.0f / 3.6f,
    /*fBrakePedalMaxVelocity */ 30.0f / 3.6f,
    /*DriverBrakingMaxAcceleration*/ -0.1f};

/*! @endcond */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE1_MODULE_STOP_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"