/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LCCRA
 *
 * Model Long Name     : LCCRA

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver1.0

 *

 * Model Author        : ZhuHe

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : LCCRA.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Feb  3 11:44:07 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "LCCRA.h"
#include "LCCRA_private.h"
#include "look1_iflf_binlxpw.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile real32_T LCCRA_CamExistThreshold_C_nu =
    1.0F; /* Referenced by: '<S1>/Constant15' */
const volatile real32_T LCCRA_DefaultLaneWidth_C_met =
    3.6F; /* Referenced by: '<S1>/Constant5' */
const volatile real32_T LCCRA_DelayTimeValue_C_nu[10] = {
    0.05F, 0.1F, 0.1F, 0.15F, 0.15F,
    0.2F,  0.2F, 0.2F, 0.2F,  0.2F}; /* Referenced by: '<S1>/Constant12' */

const volatile real32_T LCCRA_EgoFrontTTCThresholdBus_C_sec =
    0.38F; /* Referenced by: '<S1>/Constant35' */
const volatile real32_T LCCRA_EgoFrontTTCThreshold_C_sec =
    0.38F; /* Referenced by: '<S1>/Constant31' */
const volatile real32_T LCCRA_EgoLineOffset_C_met =
    0.2F; /* Referenced by: '<S1>/Constant29' */
const volatile real32_T LCCRA_EgoRearHangDist_C_met =
    3.5F; /* Referenced by: '<S1>/Constant32' */
const volatile real32_T LCCRA_FrontObjVelX_Bx_kph[10] = {
    0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F,
    6.0F, 7.0F, 8.0F, 10.0F}; /* Referenced by:
                               * '<S19>/1-D Lookup Table2'
                               * '<S22>/1-D Lookup Table2'
                               * '<S25>/1-D Lookup Table2'
                               * '<S26>/1-D Lookup Table2'
                               * '<S36>/1-D Lookup Table2'
                               * '<S37>/1-D Lookup Table2'
                               * '<S39>/1-D Lookup Table2'
                               * '<S42>/1-D Lookup Table2'
                               */

const volatile real32_T LCCRA_FrontTTCThresholdBus_C_sec =
    6.2F; /* Referenced by: '<S1>/Constant33' */
const volatile real32_T LCCRA_FrontTTCThreshold_C_sec =
    6.2F; /* Referenced by: '<S1>/Constant' */
const volatile real32_T LCCRA_FrontTimeGapBus_Mp_sec[12] = {
    1.1F,  1.1F,  0.92F, 0.85F, 0.8F, 0.7F, 0.7F,
    0.65F, 0.65F, 0.6F,  0.6F,  0.6F}; /* Referenced by:
                                        * '<S1>/1-D Lookup Table3'
                                        * '<S19>/1-D Lookup Table3'
                                        * '<S25>/1-D Lookup Table3'
                                        * '<S36>/1-D Lookup Table3'
                                        * '<S39>/1-D Lookup Table3'
                                        */

const volatile real32_T LCCRA_FrontTimeGap_Mp_sec[12] = {
    1.0F,  1.0F,  0.82F, 0.75F, 0.7F, 0.6F, 0.6F,
    0.55F, 0.55F, 0.5F,  0.5F,  0.5F}; /* Referenced by:
                                        * '<S1>/1-D Lookup Table1'
                                        * '<S19>/1-D Lookup Table1'
                                        * '<S25>/1-D Lookup Table1'
                                        * '<S36>/1-D Lookup Table1'
                                        * '<S39>/1-D Lookup Table1'
                                        */

const volatile real32_T LCCRA_LidarExistThreshold_C_nu =
    1.0F; /* Referenced by: '<S1>/Constant16' */
const volatile real32_T LCCRA_MaxFlagRiskTime_C_sec =
    1.0F; /* Referenced by: '<S1>/Constant14' */
const volatile real32_T LCCRA_MaxFlagTime_C_sec =
    10.0F; /* Referenced by: '<S1>/Constant13' */
const volatile real32_T LCCRA_MaxHeadingAngle_C_rad =
    45.0F; /* Referenced by: '<S1>/Constant7' */
const volatile real32_T LCCRA_MaxPredictionTime_C_sec =
    0.8F; /* Referenced by: '<S1>/Constant20' */
const volatile uint8_T LCCRA_MaxWeightThreshold_C_nu =
    10U; /* Referenced by: '<S1>/Constant11' */
const volatile real32_T LCCRA_MinHeadingAngle_C_rad =
    -45.0F; /* Referenced by: '<S1>/Constant8' */
const volatile real32_T LCCRA_MinPredictionTime_C_sec =
    0.4F; /* Referenced by: '<S1>/Constant21' */
const volatile real32_T LCCRA_NextTTCThreshold_C_sec =
    4.0F; /* Referenced by: '<S1>/Constant2' */
const volatile real32_T LCCRA_PredictionTime_K_nu =
    -0.5F; /* Referenced by: '<S1>/Constant18' */
const volatile real32_T LCCRA_PredictionTime_M_nu =
    2.0F; /* Referenced by: '<S1>/Constant19' */
const volatile real32_T LCCRA_RadarExistThreshold_C_nu =
    1.0F; /* Referenced by: '<S1>/Constant10' */
const volatile real32_T LCCRA_RearTTCThresholdBus_C_sec =
    6.2F; /* Referenced by: '<S1>/Constant34' */
const volatile real32_T LCCRA_RearTTCThreshold_C_sec =
    6.2F; /* Referenced by: '<S1>/Constant1' */
const volatile real32_T LCCRA_RearTimeGapBus_Mp_sec[12] = {
    1.1F,  1.1F,  0.95F, 0.9F, 0.85F, 0.7F, 0.7F,
    0.65F, 0.65F, 0.6F,  0.6F, 0.6F}; /* Referenced by:
                                       * '<S1>/1-D Lookup Table4'
                                       * '<S22>/1-D Lookup Table5'
                                       * '<S26>/1-D Lookup Table5'
                                       * '<S37>/1-D Lookup Table5'
                                       * '<S42>/1-D Lookup Table5'
                                       */

const volatile real32_T LCCRA_RearTimeGap_Mp_sec[12] = {
    1.0F,  1.0F,  0.85F, 0.8F, 0.75F, 0.6F, 0.6F,
    0.55F, 0.55F, 0.5F,  0.5F, 0.5F}; /* Referenced by:
                                       * '<S1>/1-D Lookup Table2'
                                       * '<S22>/1-D Lookup Table4'
                                       * '<S26>/1-D Lookup Table4'
                                       * '<S37>/1-D Lookup Table4'
                                       * '<S42>/1-D Lookup Table4'
                                       */

const volatile real32_T LCCRA_RiskDistThresholdHys_C_met =
    1.0F; /* Referenced by: '<S1>/Constant28' */
const volatile real32_T LCCRA_RiskDistanceX_C_met =
    50.0F; /* Referenced by: '<S1>/Constant23' */
const volatile real32_T LCCRA_RiskDistance_C_met =
    5.0F; /* Referenced by: '<S1>/Constant22' */
const volatile real32_T LCCRA_SelectMIOMaxDis_C_met =
    500.0F; /* Referenced by: '<S1>/Constant17' */
const volatile real32_T LCCRA_TTCSafeDistance_C_met =
    1.0F; /* Referenced by: '<S1>/Constant9' */
const volatile real32_T LCCRA_TTCThresholdHys_C_sec =
    0.5F; /* Referenced by: '<S1>/Constant26' */
const volatile real32_T LCCRA_TimeGapThresholdHys_C_sec =
    0.06F; /* Referenced by: '<S1>/Constant27' */
const volatile real32_T LCCRA_TimeGapThreshold_C_sec =
    0.4F; /* Referenced by: '<S1>/Constant3' */
const volatile real32_T LCCRA_TimeThreshold_C_sec =
    0.5F; /* Referenced by: '<S1>/Constant4' */
const volatile real32_T LCCRA_TmGpVelXFac_Mp_nu[10] = {
    1.0F,  1.0F, 0.95F, 0.9F, 0.85F, 0.8F,
    0.75F, 0.7F, 0.65F, 0.65F}; /* Referenced by:
                                 * '<S19>/1-D Lookup Table2'
                                 * '<S22>/1-D Lookup Table2'
                                 * '<S25>/1-D Lookup Table2'
                                 * '<S26>/1-D Lookup Table2'
                                 * '<S36>/1-D Lookup Table2'
                                 * '<S37>/1-D Lookup Table2'
                                 * '<S39>/1-D Lookup Table2'
                                 * '<S42>/1-D Lookup Table2'
                                 */

const volatile boolean_T LCCRA_UseObjLength_bool =
    1; /* Referenced by: '<S1>/Constant30' */
const volatile real32_T LCCRA_VehVelXTimeGap_Bx_kph[12] = {
    0.0F,   45.0F,  50.0F,  60.0F,  70.0F, 80.0F, 90.0F,
    100.0F, 110.0F, 120.0F, 130.0F, 140.0F}; /* Referenced by:
                                              * '<S1>/1-D Lookup Table1'
                                              * '<S1>/1-D Lookup Table2'
                                              * '<S1>/1-D Lookup Table3'
                                              * '<S1>/1-D Lookup Table4'
                                              * '<S19>/1-D Lookup Table1'
                                              * '<S19>/1-D Lookup Table3'
                                              * '<S22>/1-D Lookup Table4'
                                              * '<S22>/1-D Lookup Table5'
                                              * '<S25>/1-D Lookup Table1'
                                              * '<S25>/1-D Lookup Table3'
                                              * '<S26>/1-D Lookup Table4'
                                              * '<S26>/1-D Lookup Table5'
                                              * '<S36>/1-D Lookup Table1'
                                              * '<S36>/1-D Lookup Table3'
                                              * '<S37>/1-D Lookup Table4'
                                              * '<S37>/1-D Lookup Table5'
                                              * '<S39>/1-D Lookup Table1'
                                              * '<S39>/1-D Lookup Table3'
                                              * '<S42>/1-D Lookup Table4'
                                              * '<S42>/1-D Lookup Table5'
                                              */

const volatile real32_T LCCRA_VelocityThreshold_C_mps =
    1.0F; /* Referenced by: '<S1>/Constant6' */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
const BusVehicle LCCRA_rtZBusVehicle = {
    0U, /* NumTgtVeh */

    {{
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0U            /* Type */
     }}
    /* Vehicles */
}; /* BusVehicle ground */

const BusDebugMIOs LCCRA_rtZBusDebugMIOs = {
    {{
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     },
     {
         0, /* ID */

         {0.0F, 0.0F}, /* Position */

         {0.0F, 0.0F}, /* Velocity */

         {0.0F, 0.0F}, /* Acceleration */
         0.0F,         /* Length */
         0.0F,         /* Width */
         0.0F,         /* RangeMagnitude */
         0.0F,         /* VelocityMagnitude */
         0.0F,         /* AccelerationMagnitude */
         0.0F,         /* TTC */
         0.0F,         /* TimeGap */
         0U            /* Type */
     }}
    /* DebugMIOs */
}; /* BusDebugMIOs ground */

const BusWeight LCCRA_rtZBusWeight = {
    0U, /* LCCRA_egofrontweight_nu */
    0U, /* LCCRA_egorearweight_nu */
    0U, /* LCCRA_leftfrontweight_nu */
    0U, /* LCCRA_leftrearweight_nu */
    0U, /* LCCRA_rightfrontweight_nu */
    0U, /* LCCRA_rightrearweight_nu */
    0U, /* LCCRA_nextleftfrontweight_nu */
    0U, /* LCCRA_nextleftrearweight_nu */
    0U, /* LCCRA_nextrightfrontweight_nu */
    0U  /* LCCRA_nextrightrearweight_nu */
};      /* BusWeight ground */

const BusDelayTime LCCRA_rtZBusDelayTime = {
    0.0F, /* LCCRA_egofrontdelaytime_sec */
    0.0F, /* LCCRA_egoreardelaytime_sec */
    0.0F, /* LCCRA_leftfrontdelaytime_sec */
    0.0F, /* LCCRA_leftreardelaytime_sec */
    0.0F, /* LCCRA_rightfrontdelaytime_sec */
    0.0F, /* LCCRA_rightreardelaytime_sec */
    0.0F, /* LCCRA_nextleftfrontdelaytime_sec */
    0.0F, /* LCCRA_nextleftreardelaytime_sec */
    0.0F, /* LCCRA_nextrightfrontdelaytime_sec */
    0.0F  /* LCCRA_nextrightreardelaytime_sec */
};        /* BusDelayTime ground */

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
int32_T LCCRA_FrontDangerObjID_nu;       /* '<S14>/Switch1' */
int32_T LCCRA_RightDangerObjID_nu;       /* '<S16>/MATLAB Function' */
int32_T LCCRA_LeftDangerObjID_nu;        /* '<S15>/MATLAB Function' */
boolean_T LCCRA_bLeftRearTTCSafe;        /* '<S32>/Switch' */
boolean_T LCCRA_bLeftRearTGSafe;         /* '<S31>/Switch' */
boolean_T LCCRA_bLeftRearRDSafe;         /* '<S30>/Switch' */
boolean_T LCCRA_bLeftFrontTTCSafe;       /* '<S29>/Switch' */
boolean_T LCCRA_bLeftFrontTGSafe;        /* '<S28>/Switch' */
boolean_T LCCRA_bLeftFrontRDSafe;        /* '<S27>/Switch' */
boolean_T LCCRA_bNextLeftFrontSafe;      /* '<S33>/Switch' */
boolean_T LCCRA_bNextLeftRearSafe;       /* '<S34>/Switch' */
boolean_T LCCRA_LeftFrontSafeFlag_bool;  /* '<S15>/AND2' */
boolean_T LCCRA_LeftRearSafeFlag_bool;   /* '<S15>/AND3' */
boolean_T LCCRA_bRightRearTTCSafe;       /* '<S51>/Switch' */
boolean_T LCCRA_bRightRearTGSafe;        /* '<S50>/Switch' */
boolean_T LCCRA_bRightRearRDSafe;        /* '<S49>/Switch' */
boolean_T LCCRA_bRightFrontTTCSafe;      /* '<S48>/Switch' */
boolean_T LCCRA_bRightFrontTGSafe;       /* '<S47>/Switch' */
boolean_T LCCRA_bRightFrontRDSafe;       /* '<S46>/Switch' */
boolean_T LCCRA_bNextRightFrontSafe;     /* '<S44>/Switch' */
boolean_T LCCRA_bNextRightRearSafe;      /* '<S45>/Switch' */
boolean_T LCCRA_RightFrontSafeFlag_bool; /* '<S16>/AND2' */
boolean_T LCCRA_RightRearSafeFlag_bool;  /* '<S16>/AND3' */
boolean_T LCCRA_bFrontTTCSafe;           /* '<S17>/Switch' */
boolean_T LCCRA_FrontSafeFlag_bool;      /* '<S14>/Switch' */
boolean_T LCCRA_RightSafeFlag_bool;      /* '<S16>/MATLAB Function' */
boolean_T LCCRA_LeftSafeFlag_bool;       /* '<S15>/MATLAB Function' */

/* Block states (default storage) */
DW_LCCRA_T LCCRA_DW;

/* Exported data definition */

/* Definition for custom storage class: Global */
BusDelayTime LCCRA_DebugDelaytime_str;    /* '<S1>/LCCRA_FindMIOs' */
BusDebugMIOs LCCRA_DebugMIOs_str;         /* '<S1>/LCCRA_FindMIOs' */
BusWeight LCCRA_DebugWeight_str;          /* '<S1>/LCCRA_FindMIOs' */
real32_T LCCRA_LeftFrontSafeDistTmGp_met; /* '<S19>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_LeftRearSafeDistTmGp_met; /* '<S22>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_NextLeftFrontSfDistTmGp_met; /* '<S25>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_NextLeftRearSfDistTmGp_met; /* '<S26>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_NextRightFrontSfDistTmGp_met; /* '<S36>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_NextRightRearSfDistTmGp_met; /* '<S37>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_RightFrontSafeDistTmGp_met; /* '<S39>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T LCCRA_RightRearSafeDistTmGp_met; /* '<S42>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
BusVehicle LCCRA_TargetVehicle_str; /* '<S1>/LCCRA_ObjectPreProcess' */
boolean_T LCCRA_bLastFrontTTCSafe;  /* '<S17>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftFrontRDSafe; /* '<S27>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftFrontTGSafe; /* '<S28>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftFrontTTCSafe; /* '<S29>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftRearRDSafe; /* '<S30>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftRearTGSafe; /* '<S31>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastLeftRearTTCSafe; /* '<S32>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastNextLeftFrontSafe; /* '<S33>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastNextLeftRearSafe; /* '<S34>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastNextRightFrontSafe; /* '<S44>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastNextRightRearSafe; /* '<S45>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightFrontRDSafe; /* '<S46>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightFrontTGSafe; /* '<S47>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightFrontTTCSafe; /* '<S48>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightRearRDSafe; /* '<S49>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightRearTGSafe; /* '<S50>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T LCCRA_bLastRightRearTTCSafe; /* '<S51>/Unit Delay' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */

/* Forward declaration for local functions */
static void LCCRA_CalculationFrontobject(real32_T EgoVehicle_Velocity,
                                         const BusVehicle *TargetVehicle,
                                         const BusVehicle *LastTargetVehicle,
                                         BusMIO *Object,
                                         real32_T LCCRA_TTCSafeDistance_C_met_0,
                                         real32_T LCCRA_SysCycleTimeSen_sec,
                                         boolean_T LCCRA_UseObjLength_bool_0);
static void LCCRA_CalculationRearobject(real32_T EgoVehicle_Velocity,
                                        const BusVehicle *TargetVehicle,
                                        const BusVehicle *LastTargetVehicle,
                                        BusMIO *Object,
                                        real32_T LCCRA_TTCSafeDistance_C_met_1,
                                        real32_T LCCRA_SysCycleTimeSen_sec,
                                        boolean_T LCCRA_UseObjLength_bool_1,
                                        real32_T LCCRA_EgoRearHangDist_C_met_0);

/* Function for MATLAB Function: '<S1>/LCCRA_FindMIOs' */
static void LCCRA_CalculationFrontobject(real32_T EgoVehicle_Velocity,
                                         const BusVehicle *TargetVehicle,
                                         const BusVehicle *LastTargetVehicle,
                                         BusMIO *Object,
                                         real32_T LCCRA_TTCSafeDistance_C_met_0,
                                         real32_T LCCRA_SysCycleTimeSen_sec,
                                         boolean_T LCCRA_UseObjLength_bool_0) {
    int32_T i;
    real32_T LCCRA_Sizedistance_met;
    boolean_T LCCRA_MIOFind_bool;

    /*  Calculate parameters of the front MIO */
    LCCRA_MIOFind_bool = false;
    for (i = 1; (i - 1) < ((int32_T)TargetVehicle->NumTgtVeh); i++) {
        if (Object->ID ==
            TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].ID) {
            LCCRA_MIOFind_bool = true;
            Object->Position[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0];
            Object->Velocity[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Velocity[0];
            Object->Acceleration[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Acceleration[0];
            Object->Position[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[1];
            Object->Velocity[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Velocity[1];
            Object->Acceleration[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Acceleration[1];
            Object->Length =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Length;
            Object->Width =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Width;
            Object->Type =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Type;
        }
    }

    if (!LCCRA_MIOFind_bool) {
        for (i = 1; (i - 1) < ((int32_T)LastTargetVehicle->NumTgtVeh); i++) {
            if (Object->ID ==
                LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].ID) {
                LCCRA_MIOFind_bool = true;
                LCCRA_Sizedistance_met =
                    LCCRA_SysCycleTimeSen_sec * LCCRA_SysCycleTimeSen_sec;
                Object->Position[0] =
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Position[0] +
                     (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                          .Velocity[0] *
                      LCCRA_SysCycleTimeSen_sec)) +
                    ((LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                          .Acceleration[0] *
                      0.5F) *
                     LCCRA_Sizedistance_met);
                Object->Velocity[0] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Velocity[0] +
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Acceleration[0] *
                     LCCRA_SysCycleTimeSen_sec);
                Object->Acceleration[0] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Acceleration[0];
                Object->Position[1] =
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Position[1] +
                     (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                          .Velocity[1] *
                      LCCRA_SysCycleTimeSen_sec)) +
                    ((LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                          .Acceleration[1] *
                      0.5F) *
                     LCCRA_Sizedistance_met);
                Object->Velocity[1] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Velocity[1] +
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Acceleration[1] *
                     LCCRA_SysCycleTimeSen_sec);
                Object->Acceleration[1] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Acceleration[1];
                Object->Length =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Length;
                Object->Width =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Width;
            }
        }
    }

    /*     %% Calculate TTC */
    /*      RangeAngle = atan2(Object.Position(2),Object.Position(1)); */
    /*      VelocityAngle = atan2(Object.Velocity(2),Object.Velocity(1)); */
    /*      AccelerationAngle =
     * atan2(Object.Acceleration(2),Object.Acceleration(1)); */
    Object->RangeMagnitude = Object->Position[0];
    Object->VelocityMagnitude = Object->Velocity[0];
    Object->AccelerationMagnitude = 0.0F;
    if (LCCRA_UseObjLength_bool_0) {
        LCCRA_Sizedistance_met = Object->Length / 2.0F;
    } else {
        LCCRA_Sizedistance_met = 0.0F;
    }

    /*  limit speed to +/-0.1 m/s */
    if (((real_T)fabsf(Object->Velocity[0])) < 0.1) {
        if (Object->Velocity[0] >= 0.0F) {
            Object->VelocityMagnitude = 0.1F;
        } else {
            Object->VelocityMagnitude = -0.1F;
        }
    }

    if (Object->VelocityMagnitude > 0.0F) {
        Object->TTC = 100.0F;
    } else {
        Object->TTC = -(((Object->Position[0] - LCCRA_TTCSafeDistance_C_met_0) -
                         LCCRA_Sizedistance_met) /
                        Object->VelocityMagnitude);
    }

    /*      Delta = ... */
    /*          single(Object.VelocityMagnitude^2 -
     * 2*Object.AccelerationMagnitude*(Object.RangeMagnitude-LCCRA_TTCSafeDistance_C_met-LCCRA_Sizedistance_met));
     */
    /*      if Object.AccelerationMagnitude == 0 */
    /*      else */
    /*          if Delta < 0 */
    /*              Object.TTC = single(100); */
    /*          else  */
    /*              ttc1 =
     * (-Object.VelocityMagnitude+sqrt(Delta))/(Object.AccelerationMagnitude);
     */
    /*              ttc2 =
     * (-Object.VelocityMagnitude-sqrt(Delta))/(Object.AccelerationMagnitude);
     */
    /*              if (ttc1<0)&&(ttc2<0) */
    /*                  Object.TTC = single(100); */
    /*              elseif (ttc1>=0)&&(ttc2<0) */
    /*                  Object.TTC = single(ttc1); */
    /*              elseif (ttc1<0)&&(ttc2>=0) */
    /*                  Object.TTC = single(ttc2); */
    /*              else */
    /*                  Object.TTC = min(ttc1,ttc2); */
    /*              end */
    /*          end */
    /*      end */
    /*  Time-To-Collision (sec) */
    if (Object->TTC > 100.0F) {
        Object->TTC = 100.0F;
    } else {
        if (Object->TTC < 0.0F) {
            Object->TTC = 0.0F;
        }
    }

    /*     %% Calculate TimeGap */
    /*      EgoVelocityMagnitude = single(abs(EgoVehicle.Velocity *
     * cos(-RangeAngle))); */
    LCCRA_Sizedistance_met =
        (Object->Position[0] - LCCRA_Sizedistance_met) / EgoVehicle_Velocity;
    Object->TimeGap = LCCRA_Sizedistance_met;
    if (LCCRA_Sizedistance_met > 100.0F) {
        Object->TimeGap = 100.0F;
    } else {
        if (LCCRA_Sizedistance_met < 0.0F) {
            Object->TimeGap = 0.0F;
        }
    }

    if (!LCCRA_MIOFind_bool) {
        Object->TTC = 100.0F;
        Object->TimeGap = 100.0F;
    }
}

/* Function for MATLAB Function: '<S1>/LCCRA_FindMIOs' */
static void LCCRA_CalculationRearobject(
    real32_T EgoVehicle_Velocity,
    const BusVehicle *TargetVehicle,
    const BusVehicle *LastTargetVehicle,
    BusMIO *Object,
    real32_T LCCRA_TTCSafeDistance_C_met_1,
    real32_T LCCRA_SysCycleTimeSen_sec,
    boolean_T LCCRA_UseObjLength_bool_1,
    real32_T LCCRA_EgoRearHangDist_C_met_0) {
    int32_T i;
    real32_T LCCRA_Sizedistance_met;
    real32_T dis_tmp;
    boolean_T LCCRA_MIOFind_bool;

    /*  Calculate parameters of the rear MIO */
    LCCRA_MIOFind_bool = false;
    for (i = 1; (i - 1) < ((int32_T)TargetVehicle->NumTgtVeh); i++) {
        if (Object->ID ==
            TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].ID) {
            LCCRA_MIOFind_bool = true;
            Object->Position[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0];
            Object->Velocity[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Velocity[0];
            Object->Acceleration[0] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Acceleration[0];
            Object->Position[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[1];
            Object->Velocity[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Velocity[1];
            Object->Acceleration[1] =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Acceleration[1];
            Object->Length =
                LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Length;
            Object->Width =
                LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Width;
            Object->Type =
                TargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].Type;
        }
    }

    if (!LCCRA_MIOFind_bool) {
        for (i = 1; (i - 1) < ((int32_T)LastTargetVehicle->NumTgtVeh); i++) {
            if (Object->ID ==
                LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1].ID) {
                LCCRA_MIOFind_bool = true;
                Object->Position[0] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[0] +
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Velocity[0] *
                     LCCRA_SysCycleTimeSen_sec);
                Object->Velocity[0] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Velocity[0];
                Object->Acceleration[0] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Acceleration[0];
                Object->Position[1] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[1] +
                    (LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                         .Velocity[1] *
                     LCCRA_SysCycleTimeSen_sec);
                Object->Velocity[1] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Velocity[1];
                Object->Acceleration[1] =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Acceleration[1];
                Object->Length =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Length;
                Object->Width =
                    LastTargetVehicle->Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Width;
            }
        }
    }

    /*     %% Calculate TTC */
    /*      RangeAngle = atan2(Object.Position(2),Object.Position(1)); */
    dis_tmp = fabsf(Object->Position[0]);

    /*      VelocityAngle = atan2(Object.Velocity(2),Object.Velocity(1)); */
    /*      AccelerationAngle =
     * atan2(Object.Acceleration(2),Object.Acceleration(1)); */
    Object->RangeMagnitude = dis_tmp;
    Object->VelocityMagnitude = Object->Velocity[0];
    Object->AccelerationMagnitude = 0.0F;
    if (LCCRA_UseObjLength_bool_1) {
        LCCRA_Sizedistance_met = Object->Length / 2.0F;
    } else {
        LCCRA_Sizedistance_met = 0.0F;
    }

    /*  limit speed to +/-0.1 m/s */
    if (((real_T)fabsf(Object->Velocity[0])) < 0.1) {
        if (Object->Velocity[0] >= 0.0F) {
            Object->VelocityMagnitude = 0.1F;
        } else {
            Object->VelocityMagnitude = -0.1F;
        }
    }

    if (Object->VelocityMagnitude < 0.0F) {
        Object->TTC = 100.0F;
    } else {
        Object->TTC = (((dis_tmp - LCCRA_TTCSafeDistance_C_met_1) -
                        LCCRA_Sizedistance_met) -
                       LCCRA_EgoRearHangDist_C_met_0) /
                      Object->VelocityMagnitude;
    }

    /*      Delta = ... */
    /*          single(Object.VelocityMagnitude^2 -
     * 2*Object.AccelerationMagnitude*(Object.RangeMagnitude-LCCRA_TTCSafeDistance_C_met-LCCRA_Sizedistance_met));
     */
    /*      if Object.AccelerationMagnitude == 0 */
    /*      else */
    /*          if Delta < 0 */
    /*              Object.TTC = single(100); */
    /*          else  */
    /*              ttc1 =
     * (-Object.VelocityMagnitude+sqrt(Delta))/(Object.AccelerationMagnitude);
     */
    /*              ttc2 =
     * (-Object.VelocityMagnitude-sqrt(Delta))/(Object.AccelerationMagnitude);
     */
    /*              if (ttc1<0)&&(ttc2<0) */
    /*                  Object.TTC = single(100); */
    /*              elseif (ttc1>=0)&&(ttc2<0) */
    /*                  Object.TTC = single(ttc1); */
    /*              elseif (ttc1<0)&&(ttc2>=0) */
    /*                  Object.TTC = single(ttc2); */
    /*              else */
    /*                  Object.TTC = min(ttc1,ttc2); */
    /*              end */
    /*          end */
    /*      end */
    /*  Time-To-Contact (sec) */
    if (Object->TTC > 100.0F) {
        Object->TTC = 100.0F;
    } else {
        if (Object->TTC < 0.0F) {
            Object->TTC = 0.0F;
        }
    }

    /*     %% Calculate TimeGap */
    /*      RearVelocityMagnitude = single(abs(Rearvel * cos(RearVelocityAngle -
     * RangeAngle))); */
    dis_tmp =
        ((dis_tmp - LCCRA_EgoRearHangDist_C_met_0) - LCCRA_Sizedistance_met) /
        EgoVehicle_Velocity;
    Object->TimeGap = dis_tmp;
    if (dis_tmp > 100.0F) {
        Object->TimeGap = 100.0F;
    } else {
        if (dis_tmp < 0.0F) {
            Object->TimeGap = 0.0F;
        }
    }

    if (!LCCRA_MIOFind_bool) {
        Object->TTC = 100.0F;
        Object->TimeGap = 100.0F;
    }
}

/* Model step function */
void LCCRA_step(void) {
    static const BusVehPara tmp = {
        -1,           /* ID */
        {0.0F, 0.0F}, /* Position */
        {0.0F, 0.0F}, /* Velocity */
        {0.0F, 0.0F}, /* Acceleration */
        0.0F,         /* Length */
        0.0F,         /* Width */
        0U            /* Type */
    };

    BusMIO EgoFront;
    BusMIO EgoRear;
    BusMIO Ego_to_LeftFront;
    BusMIO Ego_to_LeftRear;
    BusMIO Ego_to_RightFront;
    BusMIO Ego_to_RightRear;
    BusMIO LeftFront;
    BusMIO LeftRear;
    BusMIO Left_to_EgoFront;
    BusMIO Left_to_EgoRear;
    BusMIO Left_to_NextLeftFront;
    BusMIO Left_to_NextLeftRear;
    BusMIO NextLeftFront;
    BusMIO NextLeftRear;
    BusMIO NextLeft_to_LeftFront;
    BusMIO NextLeft_to_LeftRear;
    BusMIO NextRightFront;
    BusMIO NextRightRear;
    BusMIO NextRight_to_RightFront;
    BusMIO NextRight_to_RightRear;
    BusMIO PrimeEgoFront;
    BusMIO PrimeEgoRear;
    BusMIO PrimeLeftFront;
    BusMIO PrimeLeftRear;
    BusMIO PrimeNextLeftFront;
    BusMIO PrimeNextLeftRear;
    BusMIO PrimeNextRightFront;
    BusMIO PrimeNextRightRear;
    BusMIO PrimeRightFront;
    BusMIO PrimeRightRear;
    BusMIO RightFront;
    BusMIO RightRear;
    BusMIO Right_to_EgoFront;
    BusMIO Right_to_EgoRear;
    BusMIO Right_to_NextRightFront;
    BusMIO Right_to_NextRightRear;
    int32_T LCCRA_Sensorsource_Cam;
    int32_T LCCRA_Sensorsource_Radar;
    int32_T i;
    int32_T j;
    real32_T ABPR_LaneWidth_met;
    real32_T EgoMaxX;
    real32_T EgoMinX;
    real32_T Ego_to_LeftAdjMaxX;
    real32_T Ego_to_LeftAdjMinX;
    real32_T Ego_to_RightAdjMaxX;
    real32_T Ego_to_RightAdjMinX;
    real32_T LCCRA_PredictionTime_sec;
    real32_T LeftAdjMaxX;
    real32_T LeftAdjMinX;
    real32_T LeftAdj_to_EgoMaxX;
    real32_T LeftAdj_to_EgoMinX;
    real32_T LeftAdj_to_LeftNextAdjMaxX;
    real32_T LeftAdj_to_LeftNextAdjMinX;
    real32_T LeftNextAdjMaxX;
    real32_T LeftNextAdjMinX;
    real32_T LeftNextAdj_to_LeftAdjMaxX;
    real32_T LeftNextAdj_to_LeftAdjMinX;
    real32_T Prediction_x;
    real32_T Prediction_yAdjLaneLeft;
    real32_T Prediction_yAdjLaneRight;
    real32_T Prediction_yEgoLaneLeft;
    real32_T Prediction_yEgoLaneLeft_tmp;
    real32_T Prediction_yEgoLaneLeft_tmp_0;
    real32_T Prediction_yEgoLaneRight;
    real32_T Prediction_yNextAdjLaneLeft;
    real32_T RightAdjMaxX;
    real32_T RightAdjMinX;
    real32_T RightAdj_to_EgoMaxX;
    real32_T RightAdj_to_EgoMinX;
    real32_T RightAdj_to_RightNextAdjMaxX;
    real32_T RightAdj_to_RightNextAdjMinX;
    real32_T RightNextAdjMaxX;
    real32_T RightNextAdjMinX;
    real32_T RightNextAdj_to_RightAdjMaxX;
    real32_T RightNextAdj_to_RightAdjMinX;
    real32_T halfLaneWidth;
    real32_T rtb_LCCRA_AdjLane_str_Left_Cu_0;
    real32_T rtb_LCCRA_AdjLane_str_Left_Curv;
    real32_T rtb_LCCRA_AdjLane_str_Left_Head;
    real32_T rtb_LCCRA_AdjLane_str_Left_PosY;
    real32_T rtb_LCCRA_AdjLane_str_Right_C_0;
    real32_T rtb_LCCRA_AdjLane_str_Right_Cur;
    real32_T rtb_LCCRA_AdjLane_str_Right_Hea;
    real32_T rtb_LCCRA_AdjLane_str_Right_Pos;
    real32_T rtb_LCCRA_EgoLane_str_Right_C_0;
    real32_T rtb_LCCRA_EgoLane_str_Right_Cur;
    real32_T rtb_LCCRA_NextAdjLane_str_Lef_0;
    real32_T rtb_LCCRA_NextAdjLane_str_Lef_1;
    real32_T rtb_LCCRA_NextAdjLane_str_Lef_2;
    real32_T rtb_LCCRA_NextAdjLane_str_Left_;
    real32_T rtb_LCCRA_NextAdjLane_str_Rig_0;
    real32_T rtb_LCCRA_NextAdjLane_str_Right;
    real32_T rtb_MultiportSwitch1_jddt;
    real32_T rtb_MultiportSwitch2;
    real32_T rtb_Product1;
    real32_T rtb_Product1_aqbo;
    real32_T rtb_Product1_bzrt;
    real32_T rtb_Product1_kv1k;
    real32_T yAdjLaneLeft;
    real32_T yAdjLaneRight;
    real32_T yEgoLaneLeft;
    real32_T yEgoLaneRight;
    real32_T yNextAdjLaneLeft;
    real32_T yNextAdjLaneRight;
    uint32_T LCCRA_CamIndex_nu;
    uint32_T LCCRA_LidarIndex_nu;
    uint32_T LCCRA_RadarIndex_nu;
    uint32_T LCCRA_Sensorsource_Lidar;
    uint8_T Num;
    boolean_T guard1 = false;
    boolean_T rtb_NextLeftFrontSafeFlag;
    boolean_T rtb_NextLeftRearSafeFlag;
    boolean_T rtb_bLeftFrontSafe;
    boolean_T rtb_bLeftRearSafe;

    /* MATLAB Function: '<S1>/LCCRA_ObjectPreProcess' incorporates:
     *  BusCreator generated from: '<S1>/LCCRA_ObjectPreProcess'
     *  Constant: '<S1>/Constant10'
     *  Constant: '<S1>/Constant15'
     *  Constant: '<S1>/Constant16'
     *  Constant: '<S1>/Constant7'
     *  Constant: '<S1>/Constant8'
     *  Inport: '<Root>/Inport26'
     */
    /*  TargetVehicle initialization */
    Num = 0U;
    for (i = 0; i < 40; i++) {
        LCCRA_TargetVehicle_str.Vehicles[(i)] = tmp;
    }

    /*  TargetObject preprocessing */
    for (i = 1; (i - 1) < ((int32_T)Fusion_TargetObject_str->NumTgtObj); i++) {
        if (Fusion_TargetObject_str->Objects[((int32_T)((uint8_T)i)) - 1]
                .HeadingAngle > LCCRA_MinHeadingAngle_C_rad) {
            if (Fusion_TargetObject_str->Objects[((int32_T)((uint8_T)i)) - 1]
                    .HeadingAngle < LCCRA_MaxHeadingAngle_C_rad) {
                if ((((int32_T)Fusion_TargetObject_str
                          ->Objects[((int32_T)((uint8_T)i)) - 1]
                          .Classification) == ((int32_T)Vehicle)) ||
                    (((int32_T)Fusion_TargetObject_str
                          ->Objects[((int32_T)((uint8_T)i)) - 1]
                          .Classification) == ((int32_T)Bus))) {
                    LCCRA_Sensorsource_Cam = (int32_T)((
                        uint32_T)(Fusion_TargetObject_str
                                      ->Objects[((int32_T)((uint8_T)i)) - 1]
                                      .Sensorsource &
                                  255U));
                    LCCRA_Sensorsource_Radar = (int32_T)((
                        uint32_T)(Fusion_TargetObject_str
                                      ->Objects[((int32_T)((uint8_T)i)) - 1]
                                      .Sensorsource &
                                  16128U));
                    LCCRA_Sensorsource_Lidar =
                        Fusion_TargetObject_str
                            ->Objects[((int32_T)((uint8_T)i)) - 1]
                            .Sensorsource &
                        507904U;
                    LCCRA_CamIndex_nu = 1U;
                    LCCRA_RadarIndex_nu = 1U;
                    LCCRA_LidarIndex_nu = 1U;
                    rtb_MultiportSwitch2 = 0.0F;
                    rtb_Product1 = 0.0F;
                    rtb_MultiportSwitch1_jddt = 0.0F;
                    for (j = 0; j < 32; j++) {
                        if (((int32_T)((
                                uint32_T)(((uint32_T)LCCRA_Sensorsource_Cam) &
                                          LCCRA_CamIndex_nu))) > 0) {
                            rtb_MultiportSwitch2++;
                        }

                        LCCRA_CamIndex_nu <<= 1U;
                        if (((int32_T)((
                                uint32_T)(((uint32_T)LCCRA_Sensorsource_Radar) &
                                          LCCRA_RadarIndex_nu))) > 0) {
                            rtb_Product1++;
                        }

                        LCCRA_RadarIndex_nu <<= 1U;
                        if (((int32_T)((uint32_T)(LCCRA_Sensorsource_Lidar &
                                                  LCCRA_LidarIndex_nu))) > 0) {
                            rtb_MultiportSwitch1_jddt++;
                        }

                        LCCRA_LidarIndex_nu <<= 1U;
                    }

                    rtb_Product1_aqbo = (rtb_MultiportSwitch2 + rtb_Product1) +
                                        rtb_MultiportSwitch1_jddt;
                    if (rtb_Product1_aqbo == 0.0F) {
                        rtb_Product1_aqbo = 0.01F;
                    }

                    if (Fusion_TargetObject_str
                            ->Objects[((int32_T)((uint8_T)i)) - 1]
                            .Existence >
                        ((((LCCRA_CamExistThreshold_C_nu *
                            rtb_MultiportSwitch2) +
                           (LCCRA_RadarExistThreshold_C_nu * rtb_Product1)) +
                          (LCCRA_LidarExistThreshold_C_nu *
                           rtb_MultiportSwitch1_jddt)) /
                         rtb_Product1_aqbo)) {
                        LCCRA_Sensorsource_Cam =
                            (int32_T)((uint32_T)(((uint32_T)Num) + 1U));
                        if ((((uint32_T)Num) + 1U) > 255U) {
                            LCCRA_Sensorsource_Cam = 255;
                        }

                        Num = (uint8_T)LCCRA_Sensorsource_Cam;
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .ID = Fusion_TargetObject_str
                                      ->Objects[((int32_T)((uint8_T)i)) - 1]
                                      .ID;
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Position[0] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Position[0];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Velocity[0] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Velocity[0];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Acceleration[0] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Acceleration[0];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Position[1] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Position[1];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Velocity[1] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Velocity[1];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Acceleration[1] =
                            Fusion_TargetObject_str
                                ->Objects[((int32_T)((uint8_T)i)) - 1]
                                .Acceleration[1];
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Length = Fusion_TargetObject_str
                                          ->Objects[((int32_T)((uint8_T)i)) - 1]
                                          .Length;
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Width = Fusion_TargetObject_str
                                         ->Objects[((int32_T)((uint8_T)i)) - 1]
                                         .Width;
                        LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((
                                          uint8_T)LCCRA_Sensorsource_Cam)) -
                                      1]
                            .Type = Fusion_TargetObject_str
                                        ->Objects[((int32_T)((uint8_T)i)) - 1]
                                        .Classification;
                    }
                }
            }
        }
    }

    LCCRA_TargetVehicle_str.NumTgtVeh = Num;

    /* End of MATLAB Function: '<S1>/LCCRA_ObjectPreProcess' */

    /* MATLAB Function: '<S1>/LCCRA_LanePreProcess' incorporates:
     *  BusCreator generated from: '<S1>/LCCRA_LanePreProcess'
     *  Constant: '<S1>/Constant5'
     *  Inport: '<Root>/Inport'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport11'
     *  Inport: '<Root>/Inport12'
     *  Inport: '<Root>/Inport13'
     *  Inport: '<Root>/Inport14'
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     *  Inport: '<Root>/Inport17'
     *  Inport: '<Root>/Inport18'
     *  Inport: '<Root>/Inport19'
     *  Inport: '<Root>/Inport2'
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport22'
     *  Inport: '<Root>/Inport3'
     *  Inport: '<Root>/Inport35'
     *  Inport: '<Root>/Inport36'
     *  Inport: '<Root>/Inport37'
     *  Inport: '<Root>/Inport38'
     *  Inport: '<Root>/Inport4'
     *  Inport: '<Root>/Inport5'
     *  Inport: '<Root>/Inport6'
     *  Inport: '<Root>/Inport7'
     *  Inport: '<Root>/Inport8'
     *  Inport: '<Root>/Inport9'
     */
    ABPR_LaneWidth_met = ABPR_LnWidth_met;

    /*  Determine whether the lane line is valid and calculate halfLaneWidth. */
    if (!ABPR_LnWidthValid_bool) {
        ABPR_LaneWidth_met = LCCRA_DefaultLaneWidth_C_met;
    }

    halfLaneWidth = ABPR_LaneWidth_met / 2.0F;

    /*  Judge the validity of lanes and assign values to various lane line
     * parameters. */
    if (LBP_LeLnValid_bool) {
        rtb_MultiportSwitch2 = LBP_LeLnClthPosY0_met;
        rtb_Product1 = LBP_LeLnClthHeading_rad;
        rtb_MultiportSwitch1_jddt = LBP_LeLnClthCrv_1pm;
        rtb_Product1_aqbo = LBP_LeLnClthCrvChng_1pm2;
    } else if (LBP_RiLnValid_bool) {
        rtb_MultiportSwitch2 = LBP_RiLnClthPosY0_met + ABPR_LaneWidth_met;
        rtb_Product1 = LBP_RiLnClthHeading_rad;
        rtb_MultiportSwitch1_jddt = LBP_RiLnClthCrv_1pm;
        rtb_Product1_aqbo = LBP_RiLnClthCrvChng_1pm2;
    } else if (ALP_LeLnValid_bool) {
        rtb_MultiportSwitch2 = ALP_LeLnClthPosY0_met - ABPR_LaneWidth_met;
        rtb_Product1 = ALP_LeLnClthHeading_rad;
        rtb_MultiportSwitch1_jddt = ALP_LeLnClthCrv_1pm;
        rtb_Product1_aqbo = ALP_LeLnClthCrvChng_1pm2;
    } else if (ALP_RiLnValid_bool) {
        rtb_MultiportSwitch2 =
            ALP_RiLnClthPosY0_met + (2.0F * ABPR_LaneWidth_met);
        rtb_Product1 = ALP_RiLnClthHeading_rad;
        rtb_MultiportSwitch1_jddt = ALP_RiLnClthCrv_1pm;
        rtb_Product1_aqbo = ALP_RiLnClthCrvChng_1pm2;
    } else {
        rtb_MultiportSwitch2 = halfLaneWidth;
        rtb_Product1 = VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_MultiportSwitch1_jddt = VED_EgoClthCrv_1pm;
        rtb_Product1_aqbo = VED_EgoClthCrvChng_1pm2;
    }

    if (LBP_RiLnValid_bool) {
        rtb_Product1_kv1k = LBP_RiLnClthPosY0_met;
        rtb_Product1_bzrt = LBP_RiLnClthHeading_rad;
        rtb_LCCRA_EgoLane_str_Right_Cur = LBP_RiLnClthCrv_1pm;
        rtb_LCCRA_EgoLane_str_Right_C_0 = LBP_RiLnClthCrvChng_1pm2;
    } else if (LBP_LeLnValid_bool) {
        rtb_Product1_kv1k = LBP_LeLnClthPosY0_met - ABPR_LaneWidth_met;
        rtb_Product1_bzrt = LBP_LeLnClthHeading_rad;
        rtb_LCCRA_EgoLane_str_Right_Cur = LBP_LeLnClthCrv_1pm;
        rtb_LCCRA_EgoLane_str_Right_C_0 = LBP_LeLnClthCrvChng_1pm2;
    } else if (ALP_RiLnValid_bool) {
        rtb_Product1_kv1k = ALP_RiLnClthPosY0_met + ABPR_LaneWidth_met;
        rtb_Product1_bzrt = ALP_RiLnClthHeading_rad;
        rtb_LCCRA_EgoLane_str_Right_Cur = ALP_RiLnClthCrv_1pm;
        rtb_LCCRA_EgoLane_str_Right_C_0 = ALP_RiLnClthCrvChng_1pm2;
    } else if (ALP_LeLnValid_bool) {
        rtb_Product1_kv1k = ALP_LeLnClthPosY0_met - (2.0F * ABPR_LaneWidth_met);
        rtb_Product1_bzrt = ALP_LeLnClthHeading_rad;
        rtb_LCCRA_EgoLane_str_Right_Cur = ALP_LeLnClthCrv_1pm;
        rtb_LCCRA_EgoLane_str_Right_C_0 = ALP_LeLnClthCrvChng_1pm2;
    } else {
        rtb_Product1_kv1k = -halfLaneWidth;
        rtb_Product1_bzrt = VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_LCCRA_EgoLane_str_Right_Cur = VED_EgoClthCrv_1pm;
        rtb_LCCRA_EgoLane_str_Right_C_0 = VED_EgoClthCrvChng_1pm2;
    }

    if (ALP_LeLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Left_PosY = ALP_LeLnClthPosY0_met;
        rtb_LCCRA_AdjLane_str_Left_Head = ALP_LeLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Left_Curv = ALP_LeLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Left_Cu_0 = ALP_LeLnClthCrvChng_1pm2;
    } else if (LBP_LeLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Left_PosY =
            LBP_LeLnClthPosY0_met + ABPR_LaneWidth_met;
        rtb_LCCRA_AdjLane_str_Left_Head = LBP_LeLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Left_Curv = LBP_LeLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Left_Cu_0 = LBP_LeLnClthCrvChng_1pm2;
    } else if (LBP_RiLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Left_PosY =
            LBP_RiLnClthPosY0_met + (2.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_AdjLane_str_Left_Head = LBP_RiLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Left_Curv = LBP_RiLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Left_Cu_0 = LBP_RiLnClthCrvChng_1pm2;
    } else if (ALP_RiLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Left_PosY =
            ALP_RiLnClthPosY0_met + (3.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_AdjLane_str_Left_Head = ALP_RiLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Left_Curv = ALP_RiLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Left_Cu_0 = ALP_RiLnClthCrvChng_1pm2;
    } else {
        rtb_LCCRA_AdjLane_str_Left_PosY = ABPR_LaneWidth_met + halfLaneWidth;
        rtb_LCCRA_AdjLane_str_Left_Head =
            VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_LCCRA_AdjLane_str_Left_Curv = VED_EgoClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Left_Cu_0 = VED_EgoClthCrvChng_1pm2;
    }

    if (ALP_RiLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Right_Pos = ALP_RiLnClthPosY0_met;
        rtb_LCCRA_AdjLane_str_Right_Hea = ALP_RiLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Right_Cur = ALP_RiLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Right_C_0 = ALP_RiLnClthCrvChng_1pm2;
    } else if (LBP_RiLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Right_Pos =
            LBP_RiLnClthPosY0_met - ABPR_LaneWidth_met;
        rtb_LCCRA_AdjLane_str_Right_Hea = LBP_RiLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Right_Cur = LBP_RiLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Right_C_0 = LBP_RiLnClthCrvChng_1pm2;
    } else if (LBP_LeLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Right_Pos =
            LBP_LeLnClthPosY0_met - (2.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_AdjLane_str_Right_Hea = LBP_LeLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Right_Cur = LBP_LeLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Right_C_0 = LBP_LeLnClthCrvChng_1pm2;
    } else if (ALP_LeLnValid_bool) {
        rtb_LCCRA_AdjLane_str_Right_Pos =
            ALP_LeLnClthPosY0_met - (3.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_AdjLane_str_Right_Hea = ALP_LeLnClthHeading_rad;
        rtb_LCCRA_AdjLane_str_Right_Cur = ALP_LeLnClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Right_C_0 = ALP_LeLnClthCrvChng_1pm2;
    } else {
        rtb_LCCRA_AdjLane_str_Right_Pos = (-ABPR_LaneWidth_met) - halfLaneWidth;
        rtb_LCCRA_AdjLane_str_Right_Hea =
            VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_LCCRA_AdjLane_str_Right_Cur = VED_EgoClthCrv_1pm;
        rtb_LCCRA_AdjLane_str_Right_C_0 = VED_EgoClthCrvChng_1pm2;
    }

    if (ALP_LeLnValid_bool) {
        rtb_LCCRA_NextAdjLane_str_Left_ =
            ALP_LeLnClthPosY0_met + ABPR_LaneWidth_met;
        rtb_LCCRA_NextAdjLane_str_Lef_0 = ALP_LeLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Lef_1 = ALP_LeLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Lef_2 = ALP_LeLnClthCrvChng_1pm2;
    } else if (LBP_LeLnValid_bool) {
        rtb_LCCRA_NextAdjLane_str_Left_ =
            LBP_LeLnClthPosY0_met + (2.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_NextAdjLane_str_Lef_0 = LBP_LeLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Lef_1 = LBP_LeLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Lef_2 = LBP_LeLnClthCrvChng_1pm2;
    } else if (LBP_RiLnValid_bool) {
        rtb_LCCRA_NextAdjLane_str_Left_ =
            LBP_RiLnClthPosY0_met + (3.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_NextAdjLane_str_Lef_0 = LBP_RiLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Lef_1 = LBP_RiLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Lef_2 = LBP_RiLnClthCrvChng_1pm2;
    } else if (ALP_RiLnValid_bool) {
        rtb_LCCRA_NextAdjLane_str_Left_ =
            ALP_RiLnClthPosY0_met + (4.0F * ABPR_LaneWidth_met);
        rtb_LCCRA_NextAdjLane_str_Lef_0 = ALP_RiLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Lef_1 = ALP_RiLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Lef_2 = ALP_RiLnClthCrvChng_1pm2;
    } else {
        rtb_LCCRA_NextAdjLane_str_Left_ =
            (2.0F * ABPR_LaneWidth_met) + halfLaneWidth;
        rtb_LCCRA_NextAdjLane_str_Lef_0 =
            VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_LCCRA_NextAdjLane_str_Lef_1 = VED_EgoClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Lef_2 = VED_EgoClthCrvChng_1pm2;
    }

    if (ALP_RiLnValid_bool) {
        ABPR_LaneWidth_met = ALP_RiLnClthPosY0_met - ABPR_LaneWidth_met;
        halfLaneWidth = ALP_RiLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Right = ALP_RiLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Rig_0 = ALP_RiLnClthCrvChng_1pm2;
    } else if (LBP_RiLnValid_bool) {
        ABPR_LaneWidth_met =
            LBP_RiLnClthPosY0_met - (2.0F * ABPR_LaneWidth_met);
        halfLaneWidth = LBP_RiLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Right = LBP_RiLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Rig_0 = LBP_RiLnClthCrvChng_1pm2;
    } else if (LBP_LeLnValid_bool) {
        ABPR_LaneWidth_met =
            LBP_LeLnClthPosY0_met - (3.0F * ABPR_LaneWidth_met);
        halfLaneWidth = LBP_LeLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Right = LBP_LeLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Rig_0 = LBP_LeLnClthCrvChng_1pm2;
    } else if (ALP_LeLnValid_bool) {
        ABPR_LaneWidth_met =
            ALP_LeLnClthPosY0_met - (4.0F * ABPR_LaneWidth_met);
        halfLaneWidth = ALP_LeLnClthHeading_rad;
        rtb_LCCRA_NextAdjLane_str_Right = ALP_LeLnClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Rig_0 = ALP_LeLnClthCrvChng_1pm2;
    } else {
        ABPR_LaneWidth_met = (-2.0F * ABPR_LaneWidth_met) - halfLaneWidth;
        halfLaneWidth = VED_EgoYawRate_rps * LCFRCV_SysCycleTimeSen_sec;
        rtb_LCCRA_NextAdjLane_str_Right = VED_EgoClthCrv_1pm;
        rtb_LCCRA_NextAdjLane_str_Rig_0 = VED_EgoClthCrvChng_1pm2;
    }

    /* End of MATLAB Function: '<S1>/LCCRA_LanePreProcess' */

    /* MATLAB Function: '<S1>/LCCRA_FindMIOs' incorporates:
     *  BusCreator generated from: '<S1>/LCCRA_FindMIOs'
     *  Constant: '<S1>/Constant11'
     *  Constant: '<S1>/Constant12'
     *  Constant: '<S1>/Constant17'
     *  Constant: '<S1>/Constant18'
     *  Constant: '<S1>/Constant19'
     *  Constant: '<S1>/Constant20'
     *  Constant: '<S1>/Constant21'
     *  Constant: '<S1>/Constant29'
     *  Constant: '<S1>/Constant30'
     *  Constant: '<S1>/Constant32'
     *  Constant: '<S1>/Constant9'
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport31'
     *  UnitDelay: '<S1>/Unit Delay1'
     *  UnitDelay: '<S1>/Unit Delay2'
     */
    /*  Initialize outputs and parameters */
    /*  Far enough forward so that no track is expected to exceed this distance
     */
    /*  Find front/rear MIO (Most Important Object) in the egolane, AdjLane and
     * NextAdjLane. */
    EgoMaxX = LCCRA_SelectMIOMaxDis_C_met;
    LeftAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    RightAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    LeftNextAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    RightNextAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    Ego_to_LeftAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    Ego_to_RightAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    LeftAdj_to_EgoMaxX = LCCRA_SelectMIOMaxDis_C_met;
    RightAdj_to_EgoMaxX = LCCRA_SelectMIOMaxDis_C_met;
    LeftAdj_to_LeftNextAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    LeftNextAdj_to_LeftAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    RightAdj_to_RightNextAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;
    RightNextAdj_to_RightAdjMaxX = LCCRA_SelectMIOMaxDis_C_met;

    /*  Far enough rearward so that no track is expected to exceed this distance
     */
    EgoMinX = -LCCRA_SelectMIOMaxDis_C_met;
    LeftAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    RightAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    LeftNextAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    RightNextAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    Ego_to_LeftAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    Ego_to_RightAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    LeftAdj_to_EgoMinX = -LCCRA_SelectMIOMaxDis_C_met;
    RightAdj_to_EgoMinX = -LCCRA_SelectMIOMaxDis_C_met;
    LeftAdj_to_LeftNextAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    LeftNextAdj_to_LeftAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    RightAdj_to_RightNextAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;
    RightNextAdj_to_RightAdjMinX = -LCCRA_SelectMIOMaxDis_C_met;

    /*  Initialize MIOs */
    EgoFront.ID = -1;
    EgoFront.Length = 0.0F;
    EgoFront.Width = 0.0F;
    EgoFront.RangeMagnitude = 100.0F;
    EgoFront.VelocityMagnitude = 0.0F;
    EgoFront.AccelerationMagnitude = 0.0F;
    EgoFront.TTC = 100.0F;
    EgoFront.TimeGap = 100.0F;
    EgoFront.Type = 0U;
    EgoRear.ID = -1;
    EgoRear.Length = 0.0F;
    EgoRear.Width = 0.0F;
    EgoRear.RangeMagnitude = 100.0F;
    EgoRear.VelocityMagnitude = 0.0F;
    EgoRear.AccelerationMagnitude = 0.0F;
    EgoRear.TTC = 100.0F;
    EgoRear.TimeGap = 100.0F;
    EgoRear.Type = 0U;
    LeftFront.ID = -1;
    LeftFront.Length = 0.0F;
    LeftFront.Width = 0.0F;
    LeftFront.RangeMagnitude = 100.0F;
    LeftFront.VelocityMagnitude = 0.0F;
    LeftFront.AccelerationMagnitude = 0.0F;
    LeftFront.TTC = 100.0F;
    LeftFront.TimeGap = 100.0F;
    LeftFront.Type = 0U;
    LeftRear.ID = -1;
    LeftRear.Length = 0.0F;
    LeftRear.Width = 0.0F;
    LeftRear.RangeMagnitude = 100.0F;
    LeftRear.VelocityMagnitude = 0.0F;
    LeftRear.AccelerationMagnitude = 0.0F;
    LeftRear.TTC = 100.0F;
    LeftRear.TimeGap = 100.0F;
    LeftRear.Type = 0U;
    RightFront.ID = -1;
    RightFront.Length = 0.0F;
    RightFront.Width = 0.0F;
    RightFront.RangeMagnitude = 100.0F;
    RightFront.VelocityMagnitude = 0.0F;
    RightFront.AccelerationMagnitude = 0.0F;
    RightFront.TTC = 100.0F;
    RightFront.TimeGap = 100.0F;
    RightFront.Type = 0U;
    RightRear.ID = -1;
    RightRear.Length = 0.0F;
    RightRear.Width = 0.0F;
    RightRear.RangeMagnitude = 100.0F;
    RightRear.VelocityMagnitude = 0.0F;
    RightRear.AccelerationMagnitude = 0.0F;
    RightRear.TTC = 100.0F;
    RightRear.TimeGap = 100.0F;
    RightRear.Type = 0U;
    NextLeftFront.ID = -1;
    NextLeftFront.Length = 0.0F;
    NextLeftFront.Width = 0.0F;
    NextLeftFront.RangeMagnitude = 100.0F;
    NextLeftFront.VelocityMagnitude = 0.0F;
    NextLeftFront.AccelerationMagnitude = 0.0F;
    NextLeftFront.TTC = 100.0F;
    NextLeftFront.TimeGap = 100.0F;
    NextLeftFront.Type = 0U;
    NextLeftRear.ID = -1;
    NextLeftRear.Length = 0.0F;
    NextLeftRear.Width = 0.0F;
    NextLeftRear.RangeMagnitude = 100.0F;
    NextLeftRear.VelocityMagnitude = 0.0F;
    NextLeftRear.AccelerationMagnitude = 0.0F;
    NextLeftRear.TTC = 100.0F;
    NextLeftRear.TimeGap = 100.0F;
    NextLeftRear.Type = 0U;
    NextRightFront.ID = -1;
    NextRightFront.Length = 0.0F;
    NextRightFront.Width = 0.0F;
    NextRightFront.RangeMagnitude = 100.0F;
    NextRightFront.VelocityMagnitude = 0.0F;
    NextRightFront.AccelerationMagnitude = 0.0F;
    NextRightFront.TTC = 100.0F;
    NextRightFront.TimeGap = 100.0F;
    NextRightFront.Type = 0U;
    NextRightRear.ID = -1;
    NextRightRear.Length = 0.0F;
    NextRightRear.Width = 0.0F;
    NextRightRear.RangeMagnitude = 100.0F;
    NextRightRear.VelocityMagnitude = 0.0F;
    NextRightRear.AccelerationMagnitude = 0.0F;
    NextRightRear.TTC = 100.0F;
    NextRightRear.TimeGap = 100.0F;
    NextRightRear.Type = 0U;
    PrimeEgoFront.ID = -1;
    PrimeEgoFront.Length = 0.0F;
    PrimeEgoFront.Width = 0.0F;
    PrimeEgoFront.RangeMagnitude = 100.0F;
    PrimeEgoFront.VelocityMagnitude = 0.0F;
    PrimeEgoFront.AccelerationMagnitude = 0.0F;
    PrimeEgoFront.TTC = 100.0F;
    PrimeEgoFront.TimeGap = 100.0F;
    PrimeEgoFront.Type = 0U;
    PrimeEgoRear.ID = -1;
    PrimeEgoRear.Length = 0.0F;
    PrimeEgoRear.Width = 0.0F;
    PrimeEgoRear.RangeMagnitude = 100.0F;
    PrimeEgoRear.VelocityMagnitude = 0.0F;
    PrimeEgoRear.AccelerationMagnitude = 0.0F;
    PrimeEgoRear.TTC = 100.0F;
    PrimeEgoRear.TimeGap = 100.0F;
    PrimeEgoRear.Type = 0U;
    PrimeLeftFront.ID = -1;
    PrimeLeftFront.Length = 0.0F;
    PrimeLeftFront.Width = 0.0F;
    PrimeLeftFront.RangeMagnitude = 100.0F;
    PrimeLeftFront.VelocityMagnitude = 0.0F;
    PrimeLeftFront.AccelerationMagnitude = 0.0F;
    PrimeLeftFront.TTC = 100.0F;
    PrimeLeftFront.TimeGap = 100.0F;
    PrimeLeftFront.Type = 0U;
    PrimeLeftRear.ID = -1;
    PrimeLeftRear.Length = 0.0F;
    PrimeLeftRear.Width = 0.0F;
    PrimeLeftRear.RangeMagnitude = 100.0F;
    PrimeLeftRear.VelocityMagnitude = 0.0F;
    PrimeLeftRear.AccelerationMagnitude = 0.0F;
    PrimeLeftRear.TTC = 100.0F;
    PrimeLeftRear.TimeGap = 100.0F;
    PrimeLeftRear.Type = 0U;
    PrimeRightFront.ID = -1;
    PrimeRightFront.Length = 0.0F;
    PrimeRightFront.Width = 0.0F;
    PrimeRightFront.RangeMagnitude = 100.0F;
    PrimeRightFront.VelocityMagnitude = 0.0F;
    PrimeRightFront.AccelerationMagnitude = 0.0F;
    PrimeRightFront.TTC = 100.0F;
    PrimeRightFront.TimeGap = 100.0F;
    PrimeRightFront.Type = 0U;
    PrimeRightRear.ID = -1;
    PrimeRightRear.Length = 0.0F;
    PrimeRightRear.Width = 0.0F;
    PrimeRightRear.RangeMagnitude = 100.0F;
    PrimeRightRear.VelocityMagnitude = 0.0F;
    PrimeRightRear.AccelerationMagnitude = 0.0F;
    PrimeRightRear.TTC = 100.0F;
    PrimeRightRear.TimeGap = 100.0F;
    PrimeRightRear.Type = 0U;
    PrimeNextLeftFront.ID = -1;
    PrimeNextLeftFront.Length = 0.0F;
    PrimeNextLeftFront.Width = 0.0F;
    PrimeNextLeftFront.RangeMagnitude = 100.0F;
    PrimeNextLeftFront.VelocityMagnitude = 0.0F;
    PrimeNextLeftFront.AccelerationMagnitude = 0.0F;
    PrimeNextLeftFront.TTC = 100.0F;
    PrimeNextLeftFront.TimeGap = 100.0F;
    PrimeNextLeftFront.Type = 0U;
    PrimeNextLeftRear.ID = -1;
    PrimeNextLeftRear.Length = 0.0F;
    PrimeNextLeftRear.Width = 0.0F;
    PrimeNextLeftRear.RangeMagnitude = 100.0F;
    PrimeNextLeftRear.VelocityMagnitude = 0.0F;
    PrimeNextLeftRear.AccelerationMagnitude = 0.0F;
    PrimeNextLeftRear.TTC = 100.0F;
    PrimeNextLeftRear.TimeGap = 100.0F;
    PrimeNextLeftRear.Type = 0U;
    PrimeNextRightFront.ID = -1;
    PrimeNextRightFront.Length = 0.0F;
    PrimeNextRightFront.Width = 0.0F;
    PrimeNextRightFront.RangeMagnitude = 100.0F;
    PrimeNextRightFront.VelocityMagnitude = 0.0F;
    PrimeNextRightFront.AccelerationMagnitude = 0.0F;
    PrimeNextRightFront.TTC = 100.0F;
    PrimeNextRightFront.TimeGap = 100.0F;
    PrimeNextRightFront.Type = 0U;
    PrimeNextRightRear.ID = -1;
    PrimeNextRightRear.Length = 0.0F;
    PrimeNextRightRear.Width = 0.0F;
    PrimeNextRightRear.RangeMagnitude = 100.0F;
    PrimeNextRightRear.VelocityMagnitude = 0.0F;
    PrimeNextRightRear.AccelerationMagnitude = 0.0F;
    PrimeNextRightRear.TTC = 100.0F;
    PrimeNextRightRear.TimeGap = 100.0F;
    PrimeNextRightRear.Type = 0U;
    Ego_to_LeftFront.ID = -1;
    Ego_to_LeftFront.Length = 0.0F;
    Ego_to_LeftFront.Width = 0.0F;
    Ego_to_LeftFront.RangeMagnitude = 100.0F;
    Ego_to_LeftFront.VelocityMagnitude = 0.0F;
    Ego_to_LeftFront.AccelerationMagnitude = 0.0F;
    Ego_to_LeftFront.TTC = 100.0F;
    Ego_to_LeftFront.TimeGap = 100.0F;
    Ego_to_LeftFront.Type = 0U;
    Ego_to_LeftRear.ID = -1;
    Ego_to_LeftRear.Length = 0.0F;
    Ego_to_LeftRear.Width = 0.0F;
    Ego_to_LeftRear.RangeMagnitude = 100.0F;
    Ego_to_LeftRear.VelocityMagnitude = 0.0F;
    Ego_to_LeftRear.AccelerationMagnitude = 0.0F;
    Ego_to_LeftRear.TTC = 100.0F;
    Ego_to_LeftRear.TimeGap = 100.0F;
    Ego_to_LeftRear.Type = 0U;
    Ego_to_RightFront.ID = -1;
    Ego_to_RightFront.Length = 0.0F;
    Ego_to_RightFront.Width = 0.0F;
    Ego_to_RightFront.RangeMagnitude = 100.0F;
    Ego_to_RightFront.VelocityMagnitude = 0.0F;
    Ego_to_RightFront.AccelerationMagnitude = 0.0F;
    Ego_to_RightFront.TTC = 100.0F;
    Ego_to_RightFront.TimeGap = 100.0F;
    Ego_to_RightFront.Type = 0U;
    Ego_to_RightRear.ID = -1;
    Ego_to_RightRear.Length = 0.0F;
    Ego_to_RightRear.Width = 0.0F;
    Ego_to_RightRear.RangeMagnitude = 100.0F;
    Ego_to_RightRear.VelocityMagnitude = 0.0F;
    Ego_to_RightRear.AccelerationMagnitude = 0.0F;
    Ego_to_RightRear.TTC = 100.0F;
    Ego_to_RightRear.TimeGap = 100.0F;
    Ego_to_RightRear.Type = 0U;
    Left_to_EgoFront.ID = -1;
    Left_to_EgoFront.Length = 0.0F;
    Left_to_EgoFront.Width = 0.0F;
    Left_to_EgoFront.RangeMagnitude = 100.0F;
    Left_to_EgoFront.VelocityMagnitude = 0.0F;
    Left_to_EgoFront.AccelerationMagnitude = 0.0F;
    Left_to_EgoFront.TTC = 100.0F;
    Left_to_EgoFront.TimeGap = 100.0F;
    Left_to_EgoFront.Type = 0U;
    Left_to_EgoRear.ID = -1;
    Left_to_EgoRear.Length = 0.0F;
    Left_to_EgoRear.Width = 0.0F;
    Left_to_EgoRear.RangeMagnitude = 100.0F;
    Left_to_EgoRear.VelocityMagnitude = 0.0F;
    Left_to_EgoRear.AccelerationMagnitude = 0.0F;
    Left_to_EgoRear.TTC = 100.0F;
    Left_to_EgoRear.TimeGap = 100.0F;
    Left_to_EgoRear.Type = 0U;
    Right_to_EgoFront.ID = -1;
    Right_to_EgoFront.Length = 0.0F;
    Right_to_EgoFront.Width = 0.0F;
    Right_to_EgoFront.RangeMagnitude = 100.0F;
    Right_to_EgoFront.VelocityMagnitude = 0.0F;
    Right_to_EgoFront.AccelerationMagnitude = 0.0F;
    Right_to_EgoFront.TTC = 100.0F;
    Right_to_EgoFront.TimeGap = 100.0F;
    Right_to_EgoFront.Type = 0U;
    Right_to_EgoRear.ID = -1;
    Right_to_EgoRear.Length = 0.0F;
    Right_to_EgoRear.Width = 0.0F;
    Right_to_EgoRear.RangeMagnitude = 100.0F;
    Right_to_EgoRear.VelocityMagnitude = 0.0F;
    Right_to_EgoRear.AccelerationMagnitude = 0.0F;
    Right_to_EgoRear.TTC = 100.0F;
    Right_to_EgoRear.TimeGap = 100.0F;
    Right_to_EgoRear.Type = 0U;
    NextLeft_to_LeftFront.ID = -1;
    NextLeft_to_LeftFront.Length = 0.0F;
    NextLeft_to_LeftFront.Width = 0.0F;
    NextLeft_to_LeftFront.RangeMagnitude = 100.0F;
    NextLeft_to_LeftFront.VelocityMagnitude = 0.0F;
    NextLeft_to_LeftFront.AccelerationMagnitude = 0.0F;
    NextLeft_to_LeftFront.TTC = 100.0F;
    NextLeft_to_LeftFront.TimeGap = 100.0F;
    NextLeft_to_LeftFront.Type = 0U;
    NextLeft_to_LeftRear.ID = -1;
    NextLeft_to_LeftRear.Length = 0.0F;
    NextLeft_to_LeftRear.Width = 0.0F;
    NextLeft_to_LeftRear.RangeMagnitude = 100.0F;
    NextLeft_to_LeftRear.VelocityMagnitude = 0.0F;
    NextLeft_to_LeftRear.AccelerationMagnitude = 0.0F;
    NextLeft_to_LeftRear.TTC = 100.0F;
    NextLeft_to_LeftRear.TimeGap = 100.0F;
    NextLeft_to_LeftRear.Type = 0U;
    Left_to_NextLeftFront.ID = -1;
    Left_to_NextLeftFront.Length = 0.0F;
    Left_to_NextLeftFront.Width = 0.0F;
    Left_to_NextLeftFront.RangeMagnitude = 100.0F;
    Left_to_NextLeftFront.VelocityMagnitude = 0.0F;
    Left_to_NextLeftFront.AccelerationMagnitude = 0.0F;
    Left_to_NextLeftFront.TTC = 100.0F;
    Left_to_NextLeftFront.TimeGap = 100.0F;
    Left_to_NextLeftFront.Type = 0U;
    Left_to_NextLeftRear.ID = -1;
    Left_to_NextLeftRear.Length = 0.0F;
    Left_to_NextLeftRear.Width = 0.0F;
    Left_to_NextLeftRear.RangeMagnitude = 100.0F;
    Left_to_NextLeftRear.VelocityMagnitude = 0.0F;
    Left_to_NextLeftRear.AccelerationMagnitude = 0.0F;
    Left_to_NextLeftRear.TTC = 100.0F;
    Left_to_NextLeftRear.TimeGap = 100.0F;
    Left_to_NextLeftRear.Type = 0U;
    NextRight_to_RightFront.ID = -1;
    NextRight_to_RightFront.Length = 0.0F;
    NextRight_to_RightFront.Width = 0.0F;
    NextRight_to_RightFront.RangeMagnitude = 100.0F;
    NextRight_to_RightFront.VelocityMagnitude = 0.0F;
    NextRight_to_RightFront.AccelerationMagnitude = 0.0F;
    NextRight_to_RightFront.TTC = 100.0F;
    NextRight_to_RightFront.TimeGap = 100.0F;
    NextRight_to_RightFront.Type = 0U;
    NextRight_to_RightRear.ID = -1;
    NextRight_to_RightRear.Length = 0.0F;
    NextRight_to_RightRear.Width = 0.0F;
    NextRight_to_RightRear.RangeMagnitude = 100.0F;
    NextRight_to_RightRear.VelocityMagnitude = 0.0F;
    NextRight_to_RightRear.AccelerationMagnitude = 0.0F;
    NextRight_to_RightRear.TTC = 100.0F;
    NextRight_to_RightRear.TimeGap = 100.0F;
    NextRight_to_RightRear.Type = 0U;
    Right_to_NextRightFront.ID = -1;
    Right_to_NextRightFront.Length = 0.0F;
    Right_to_NextRightFront.Width = 0.0F;
    Right_to_NextRightFront.RangeMagnitude = 100.0F;
    Right_to_NextRightFront.VelocityMagnitude = 0.0F;
    Right_to_NextRightFront.AccelerationMagnitude = 0.0F;
    Right_to_NextRightFront.TTC = 100.0F;
    Right_to_NextRightFront.TimeGap = 100.0F;
    Right_to_NextRightFront.Type = 0U;
    Right_to_NextRightRear.ID = -1;
    EgoFront.Position[0] = 0.0F;
    EgoFront.Velocity[0] = 0.0F;
    EgoFront.Acceleration[0] = 0.0F;
    EgoRear.Position[0] = 0.0F;
    EgoRear.Velocity[0] = 0.0F;
    EgoRear.Acceleration[0] = 0.0F;
    LeftFront.Position[0] = 0.0F;
    LeftFront.Velocity[0] = 0.0F;
    LeftFront.Acceleration[0] = 0.0F;
    LeftRear.Position[0] = 0.0F;
    LeftRear.Velocity[0] = 0.0F;
    LeftRear.Acceleration[0] = 0.0F;
    RightFront.Position[0] = 0.0F;
    RightFront.Velocity[0] = 0.0F;
    RightFront.Acceleration[0] = 0.0F;
    RightRear.Position[0] = 0.0F;
    RightRear.Velocity[0] = 0.0F;
    RightRear.Acceleration[0] = 0.0F;
    NextLeftFront.Position[0] = 0.0F;
    NextLeftFront.Velocity[0] = 0.0F;
    NextLeftFront.Acceleration[0] = 0.0F;
    NextLeftRear.Position[0] = 0.0F;
    NextLeftRear.Velocity[0] = 0.0F;
    NextLeftRear.Acceleration[0] = 0.0F;
    NextRightFront.Position[0] = 0.0F;
    NextRightFront.Velocity[0] = 0.0F;
    NextRightFront.Acceleration[0] = 0.0F;
    NextRightRear.Position[0] = 0.0F;
    NextRightRear.Velocity[0] = 0.0F;
    NextRightRear.Acceleration[0] = 0.0F;
    PrimeEgoFront.Position[0] = 0.0F;
    PrimeEgoFront.Velocity[0] = 0.0F;
    PrimeEgoFront.Acceleration[0] = 0.0F;
    PrimeEgoRear.Position[0] = 0.0F;
    PrimeEgoRear.Velocity[0] = 0.0F;
    PrimeEgoRear.Acceleration[0] = 0.0F;
    PrimeLeftFront.Position[0] = 0.0F;
    PrimeLeftFront.Velocity[0] = 0.0F;
    PrimeLeftFront.Acceleration[0] = 0.0F;
    PrimeLeftRear.Position[0] = 0.0F;
    PrimeLeftRear.Velocity[0] = 0.0F;
    PrimeLeftRear.Acceleration[0] = 0.0F;
    PrimeRightFront.Position[0] = 0.0F;
    PrimeRightFront.Velocity[0] = 0.0F;
    PrimeRightFront.Acceleration[0] = 0.0F;
    PrimeRightRear.Position[0] = 0.0F;
    PrimeRightRear.Velocity[0] = 0.0F;
    PrimeRightRear.Acceleration[0] = 0.0F;
    PrimeNextLeftFront.Position[0] = 0.0F;
    PrimeNextLeftFront.Velocity[0] = 0.0F;
    PrimeNextLeftFront.Acceleration[0] = 0.0F;
    PrimeNextLeftRear.Position[0] = 0.0F;
    PrimeNextLeftRear.Velocity[0] = 0.0F;
    PrimeNextLeftRear.Acceleration[0] = 0.0F;
    PrimeNextRightFront.Position[0] = 0.0F;
    PrimeNextRightFront.Velocity[0] = 0.0F;
    PrimeNextRightFront.Acceleration[0] = 0.0F;
    PrimeNextRightRear.Position[0] = 0.0F;
    PrimeNextRightRear.Velocity[0] = 0.0F;
    PrimeNextRightRear.Acceleration[0] = 0.0F;
    Ego_to_LeftFront.Position[0] = 0.0F;
    Ego_to_LeftFront.Velocity[0] = 0.0F;
    Ego_to_LeftFront.Acceleration[0] = 0.0F;
    Ego_to_LeftRear.Position[0] = 0.0F;
    Ego_to_LeftRear.Velocity[0] = 0.0F;
    Ego_to_LeftRear.Acceleration[0] = 0.0F;
    Ego_to_RightFront.Position[0] = 0.0F;
    Ego_to_RightFront.Velocity[0] = 0.0F;
    Ego_to_RightFront.Acceleration[0] = 0.0F;
    Ego_to_RightRear.Position[0] = 0.0F;
    Ego_to_RightRear.Velocity[0] = 0.0F;
    Ego_to_RightRear.Acceleration[0] = 0.0F;
    Left_to_EgoFront.Position[0] = 0.0F;
    Left_to_EgoFront.Velocity[0] = 0.0F;
    Left_to_EgoFront.Acceleration[0] = 0.0F;
    Left_to_EgoRear.Position[0] = 0.0F;
    Left_to_EgoRear.Velocity[0] = 0.0F;
    Left_to_EgoRear.Acceleration[0] = 0.0F;
    Right_to_EgoFront.Position[0] = 0.0F;
    Right_to_EgoFront.Velocity[0] = 0.0F;
    Right_to_EgoFront.Acceleration[0] = 0.0F;
    Right_to_EgoRear.Position[0] = 0.0F;
    Right_to_EgoRear.Velocity[0] = 0.0F;
    Right_to_EgoRear.Acceleration[0] = 0.0F;
    NextLeft_to_LeftFront.Position[0] = 0.0F;
    NextLeft_to_LeftFront.Velocity[0] = 0.0F;
    NextLeft_to_LeftFront.Acceleration[0] = 0.0F;
    NextLeft_to_LeftRear.Position[0] = 0.0F;
    NextLeft_to_LeftRear.Velocity[0] = 0.0F;
    NextLeft_to_LeftRear.Acceleration[0] = 0.0F;
    Left_to_NextLeftFront.Position[0] = 0.0F;
    Left_to_NextLeftFront.Velocity[0] = 0.0F;
    Left_to_NextLeftFront.Acceleration[0] = 0.0F;
    Left_to_NextLeftRear.Position[0] = 0.0F;
    Left_to_NextLeftRear.Velocity[0] = 0.0F;
    Left_to_NextLeftRear.Acceleration[0] = 0.0F;
    NextRight_to_RightFront.Position[0] = 0.0F;
    NextRight_to_RightFront.Velocity[0] = 0.0F;
    NextRight_to_RightFront.Acceleration[0] = 0.0F;
    NextRight_to_RightRear.Position[0] = 0.0F;
    NextRight_to_RightRear.Velocity[0] = 0.0F;
    NextRight_to_RightRear.Acceleration[0] = 0.0F;
    Right_to_NextRightFront.Position[0] = 0.0F;
    Right_to_NextRightFront.Velocity[0] = 0.0F;
    Right_to_NextRightFront.Acceleration[0] = 0.0F;
    Right_to_NextRightRear.Position[0] = 0.0F;
    Right_to_NextRightRear.Velocity[0] = 0.0F;
    Right_to_NextRightRear.Acceleration[0] = 0.0F;
    EgoFront.Position[1] = 0.0F;
    EgoFront.Velocity[1] = 0.0F;
    EgoFront.Acceleration[1] = 0.0F;
    EgoRear.Position[1] = 0.0F;
    EgoRear.Velocity[1] = 0.0F;
    EgoRear.Acceleration[1] = 0.0F;
    LeftFront.Position[1] = 0.0F;
    LeftFront.Velocity[1] = 0.0F;
    LeftFront.Acceleration[1] = 0.0F;
    LeftRear.Position[1] = 0.0F;
    LeftRear.Velocity[1] = 0.0F;
    LeftRear.Acceleration[1] = 0.0F;
    RightFront.Position[1] = 0.0F;
    RightFront.Velocity[1] = 0.0F;
    RightFront.Acceleration[1] = 0.0F;
    RightRear.Position[1] = 0.0F;
    RightRear.Velocity[1] = 0.0F;
    RightRear.Acceleration[1] = 0.0F;
    NextLeftFront.Position[1] = 0.0F;
    NextLeftFront.Velocity[1] = 0.0F;
    NextLeftFront.Acceleration[1] = 0.0F;
    NextLeftRear.Position[1] = 0.0F;
    NextLeftRear.Velocity[1] = 0.0F;
    NextLeftRear.Acceleration[1] = 0.0F;
    NextRightFront.Position[1] = 0.0F;
    NextRightFront.Velocity[1] = 0.0F;
    NextRightFront.Acceleration[1] = 0.0F;
    NextRightRear.Position[1] = 0.0F;
    NextRightRear.Velocity[1] = 0.0F;
    NextRightRear.Acceleration[1] = 0.0F;
    PrimeEgoFront.Position[1] = 0.0F;
    PrimeEgoFront.Velocity[1] = 0.0F;
    PrimeEgoFront.Acceleration[1] = 0.0F;
    PrimeEgoRear.Position[1] = 0.0F;
    PrimeEgoRear.Velocity[1] = 0.0F;
    PrimeEgoRear.Acceleration[1] = 0.0F;
    PrimeLeftFront.Position[1] = 0.0F;
    PrimeLeftFront.Velocity[1] = 0.0F;
    PrimeLeftFront.Acceleration[1] = 0.0F;
    PrimeLeftRear.Position[1] = 0.0F;
    PrimeLeftRear.Velocity[1] = 0.0F;
    PrimeLeftRear.Acceleration[1] = 0.0F;
    PrimeRightFront.Position[1] = 0.0F;
    PrimeRightFront.Velocity[1] = 0.0F;
    PrimeRightFront.Acceleration[1] = 0.0F;
    PrimeRightRear.Position[1] = 0.0F;
    PrimeRightRear.Velocity[1] = 0.0F;
    PrimeRightRear.Acceleration[1] = 0.0F;
    PrimeNextLeftFront.Position[1] = 0.0F;
    PrimeNextLeftFront.Velocity[1] = 0.0F;
    PrimeNextLeftFront.Acceleration[1] = 0.0F;
    PrimeNextLeftRear.Position[1] = 0.0F;
    PrimeNextLeftRear.Velocity[1] = 0.0F;
    PrimeNextLeftRear.Acceleration[1] = 0.0F;
    PrimeNextRightFront.Position[1] = 0.0F;
    PrimeNextRightFront.Velocity[1] = 0.0F;
    PrimeNextRightFront.Acceleration[1] = 0.0F;
    PrimeNextRightRear.Position[1] = 0.0F;
    PrimeNextRightRear.Velocity[1] = 0.0F;
    PrimeNextRightRear.Acceleration[1] = 0.0F;
    Ego_to_LeftFront.Position[1] = 0.0F;
    Ego_to_LeftFront.Velocity[1] = 0.0F;
    Ego_to_LeftFront.Acceleration[1] = 0.0F;
    Ego_to_LeftRear.Position[1] = 0.0F;
    Ego_to_LeftRear.Velocity[1] = 0.0F;
    Ego_to_LeftRear.Acceleration[1] = 0.0F;
    Ego_to_RightFront.Position[1] = 0.0F;
    Ego_to_RightFront.Velocity[1] = 0.0F;
    Ego_to_RightFront.Acceleration[1] = 0.0F;
    Ego_to_RightRear.Position[1] = 0.0F;
    Ego_to_RightRear.Velocity[1] = 0.0F;
    Ego_to_RightRear.Acceleration[1] = 0.0F;
    Left_to_EgoFront.Position[1] = 0.0F;
    Left_to_EgoFront.Velocity[1] = 0.0F;
    Left_to_EgoFront.Acceleration[1] = 0.0F;
    Left_to_EgoRear.Position[1] = 0.0F;
    Left_to_EgoRear.Velocity[1] = 0.0F;
    Left_to_EgoRear.Acceleration[1] = 0.0F;
    Right_to_EgoFront.Position[1] = 0.0F;
    Right_to_EgoFront.Velocity[1] = 0.0F;
    Right_to_EgoFront.Acceleration[1] = 0.0F;
    Right_to_EgoRear.Position[1] = 0.0F;
    Right_to_EgoRear.Velocity[1] = 0.0F;
    Right_to_EgoRear.Acceleration[1] = 0.0F;
    NextLeft_to_LeftFront.Position[1] = 0.0F;
    NextLeft_to_LeftFront.Velocity[1] = 0.0F;
    NextLeft_to_LeftFront.Acceleration[1] = 0.0F;
    NextLeft_to_LeftRear.Position[1] = 0.0F;
    NextLeft_to_LeftRear.Velocity[1] = 0.0F;
    NextLeft_to_LeftRear.Acceleration[1] = 0.0F;
    Left_to_NextLeftFront.Position[1] = 0.0F;
    Left_to_NextLeftFront.Velocity[1] = 0.0F;
    Left_to_NextLeftFront.Acceleration[1] = 0.0F;
    Left_to_NextLeftRear.Position[1] = 0.0F;
    Left_to_NextLeftRear.Velocity[1] = 0.0F;
    Left_to_NextLeftRear.Acceleration[1] = 0.0F;
    NextRight_to_RightFront.Position[1] = 0.0F;
    NextRight_to_RightFront.Velocity[1] = 0.0F;
    NextRight_to_RightFront.Acceleration[1] = 0.0F;
    NextRight_to_RightRear.Position[1] = 0.0F;
    NextRight_to_RightRear.Velocity[1] = 0.0F;
    NextRight_to_RightRear.Acceleration[1] = 0.0F;
    Right_to_NextRightFront.Position[1] = 0.0F;
    Right_to_NextRightFront.Velocity[1] = 0.0F;
    Right_to_NextRightFront.Acceleration[1] = 0.0F;
    Right_to_NextRightRear.Position[1] = 0.0F;
    Right_to_NextRightRear.Velocity[1] = 0.0F;
    Right_to_NextRightRear.Acceleration[1] = 0.0F;
    Right_to_NextRightRear.Length = 0.0F;
    Right_to_NextRightRear.Width = 0.0F;
    Right_to_NextRightRear.RangeMagnitude = 100.0F;
    Right_to_NextRightRear.VelocityMagnitude = 0.0F;
    Right_to_NextRightRear.AccelerationMagnitude = 0.0F;
    Right_to_NextRightRear.TTC = 100.0F;
    Right_to_NextRightRear.TimeGap = 100.0F;
    Right_to_NextRightRear.Type = 0U;

    /*  Initialize Output and Debug struct */
    /*  Define and initialize weights and DelayTime */
    /*  Lane Association */
    for (i = 1; (i - 1) < ((int32_T)LCCRA_TargetVehicle_str.NumTgtVeh); i++) {
        /*  Longitudinal position of the target vehicle relative to ego */
        /*  Lateral position of the target vehicle relative to ego */
        /*  Left EgoLane */
        yNextAdjLaneRight =
            powf(LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                     .Position[0],
                 3.0F);
        Prediction_x =
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0] *
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0];
        yEgoLaneLeft =
            (((((rtb_Product1_aqbo / 6.0F) * yNextAdjLaneRight) +
               ((rtb_MultiportSwitch1_jddt / 2.0F) * Prediction_x)) +
              (rtb_Product1 *
               LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                   .Position[0])) +
             rtb_MultiportSwitch2) -
            LCCRA_EgoLineOffset_C_met;

        /*  Right EgoLane */
        yEgoLaneRight =
            (((((rtb_LCCRA_EgoLane_str_Right_C_0 / 6.0F) * yNextAdjLaneRight) +
               ((rtb_LCCRA_EgoLane_str_Right_Cur / 2.0F) * Prediction_x)) +
              (rtb_Product1_bzrt *
               LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                   .Position[0])) +
             rtb_Product1_kv1k) +
            LCCRA_EgoLineOffset_C_met;

        /*  Left AdjLane */
        yAdjLaneLeft =
            ((((rtb_LCCRA_AdjLane_str_Left_Cu_0 / 6.0F) * yNextAdjLaneRight) +
              ((rtb_LCCRA_AdjLane_str_Left_Curv / 2.0F) * Prediction_x)) +
             (rtb_LCCRA_AdjLane_str_Left_Head *
              LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                  .Position[0])) +
            rtb_LCCRA_AdjLane_str_Left_PosY;

        /*  Right AdjLane */
        yAdjLaneRight =
            ((((rtb_LCCRA_AdjLane_str_Right_C_0 / 6.0F) * yNextAdjLaneRight) +
              ((rtb_LCCRA_AdjLane_str_Right_Cur / 2.0F) * Prediction_x)) +
             (rtb_LCCRA_AdjLane_str_Right_Hea *
              LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                  .Position[0])) +
            rtb_LCCRA_AdjLane_str_Right_Pos;

        /*  Left NextAdjLane */
        yNextAdjLaneLeft =
            ((((rtb_LCCRA_NextAdjLane_str_Lef_2 / 6.0F) * yNextAdjLaneRight) +
              ((rtb_LCCRA_NextAdjLane_str_Lef_1 / 2.0F) * Prediction_x)) +
             (rtb_LCCRA_NextAdjLane_str_Lef_0 *
              LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                  .Position[0])) +
            rtb_LCCRA_NextAdjLane_str_Left_;

        /*  Right NextAdjLane */
        yNextAdjLaneRight =
            ((((rtb_LCCRA_NextAdjLane_str_Rig_0 / 6.0F) * yNextAdjLaneRight) +
              ((rtb_LCCRA_NextAdjLane_str_Right / 2.0F) * Prediction_x)) +
             (halfLaneWidth *
              LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                  .Position[0])) +
            ABPR_LaneWidth_met;

        /*  Calculate Headway */
        /*      DistanceMagnitude = sqrt(x^2+y^2); */
        /*      DistanceAngle = atan2(y,x); */
        /*      if x > 0 */
        /*          VelocityMagnitude = single(VED_Ego_str.Velocity *
         * cos(-DistanceAngle)); */
        /*      else */
        /*          VelocityAngle =
         * atan2(LCCRA_TargetVehicle_str.Vehicles(i).Velocity(2),LCCRA_TargetVehicle_str.Vehicles(i).Velocity(1)+VED_Ego_str.Velocity);
         */
        /*          Velocity =
         * sqrt((LCCRA_TargetVehicle_str.Vehicles(i).Velocity(1) +
         * VED_Ego_str.Velocity) ^ 2 +
         * (LCCRA_TargetVehicle_str.Vehicles(i).Velocity(2)) ^ 2); */
        /*          VelocityMagnitude = single(Velocity * cos(VelocityAngle -
         * DistanceAngle)); */
        /*      end */
        /*  Calculate Prediction Time */
        LCCRA_PredictionTime_sec =
            (LCCRA_PredictionTime_K_nu *
             (fabsf(
                  LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                      .Position[0]) /
              VED_EgoVelocity_mps)) +
            LCCRA_PredictionTime_M_nu;
        if (LCCRA_PredictionTime_sec >= LCCRA_MaxPredictionTime_C_sec) {
            LCCRA_PredictionTime_sec = LCCRA_MaxPredictionTime_C_sec;
        } else {
            if (LCCRA_PredictionTime_sec <= LCCRA_MinPredictionTime_C_sec) {
                LCCRA_PredictionTime_sec = LCCRA_MinPredictionTime_C_sec;
            }
        }

        /*  Calculate Prediction Lateral Position */
        /*      PredictionPosition =
         * LCCRA_TargetVehicle_str.Vehicles(i).Position +
         * LCCRA_TargetVehicle_str.Vehicles(i).Velocity *
         * LCCRA_PredictionTime_sec + 0.5 *
         * LCCRA_TargetVehicle_str.Vehicles(i).Acceleration *
         * LCCRA_PredictionTime_sec; */
        Prediction_x =
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0] +
            (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                 .Velocity[0] *
             LCCRA_PredictionTime_sec);
        LCCRA_PredictionTime_sec =
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[1] +
            ((LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                  .Velocity[1] -
              (VED_EgoVelocity_mps * sinf(rtb_Product1))) *
             LCCRA_PredictionTime_sec);

        /*  Left EgoLane for prediction time */
        Prediction_yEgoLaneLeft_tmp = powf(Prediction_x, 3.0F);
        Prediction_yEgoLaneLeft_tmp_0 = Prediction_x * Prediction_x;
        Prediction_yEgoLaneLeft =
            ((((rtb_Product1_aqbo / 6.0F) * Prediction_yEgoLaneLeft_tmp) +
              ((rtb_MultiportSwitch1_jddt / 2.0F) *
               Prediction_yEgoLaneLeft_tmp_0)) +
             (rtb_Product1 * Prediction_x)) +
            rtb_MultiportSwitch2;

        /*  Right EgoLane for prediction time */
        Prediction_yEgoLaneRight = ((((rtb_LCCRA_EgoLane_str_Right_C_0 / 6.0F) *
                                      Prediction_yEgoLaneLeft_tmp) +
                                     ((rtb_LCCRA_EgoLane_str_Right_Cur / 2.0F) *
                                      Prediction_yEgoLaneLeft_tmp_0)) +
                                    (rtb_Product1_bzrt * Prediction_x)) +
                                   rtb_Product1_kv1k;

        /*  Left AdjLane for prediction time */
        Prediction_yAdjLaneLeft =
            ((((rtb_LCCRA_AdjLane_str_Left_Cu_0 / 6.0F) *
               Prediction_yEgoLaneLeft_tmp) +
              ((rtb_LCCRA_AdjLane_str_Left_Curv / 2.0F) *
               Prediction_yEgoLaneLeft_tmp_0)) +
             (rtb_LCCRA_AdjLane_str_Left_Head * Prediction_x)) +
            rtb_LCCRA_AdjLane_str_Left_PosY;

        /*  Right AdjLane for prediction time */
        Prediction_yAdjLaneRight =
            ((((rtb_LCCRA_AdjLane_str_Right_C_0 / 6.0F) *
               Prediction_yEgoLaneLeft_tmp) +
              ((rtb_LCCRA_AdjLane_str_Right_Cur / 2.0F) *
               Prediction_yEgoLaneLeft_tmp_0)) +
             (rtb_LCCRA_AdjLane_str_Right_Hea * Prediction_x)) +
            rtb_LCCRA_AdjLane_str_Right_Pos;

        /*  Left NextAdjLane for prediction time */
        Prediction_yNextAdjLaneLeft =
            ((((rtb_LCCRA_NextAdjLane_str_Lef_2 / 6.0F) *
               Prediction_yEgoLaneLeft_tmp) +
              ((rtb_LCCRA_NextAdjLane_str_Lef_1 / 2.0F) *
               Prediction_yEgoLaneLeft_tmp_0)) +
             (rtb_LCCRA_NextAdjLane_str_Lef_0 * Prediction_x)) +
            rtb_LCCRA_NextAdjLane_str_Left_;

        /*  Right NextAdjLane for prediction time */
        Prediction_x = ((((rtb_LCCRA_NextAdjLane_str_Rig_0 / 6.0F) *
                          Prediction_yEgoLaneLeft_tmp) +
                         ((rtb_LCCRA_NextAdjLane_str_Right / 2.0F) *
                          Prediction_yEgoLaneLeft_tmp_0)) +
                        (halfLaneWidth * Prediction_x)) +
                       ABPR_LaneWidth_met;

        /*     %% Check for Prime MIOs */
        /*  Check for ego front MIO */
        if (1.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < EgoMaxX) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if ((Prediction_yEgoLaneRight <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yEgoLaneLeft)) {
                            EgoMaxX = LCCRA_TargetVehicle_str
                                          .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                          .Position[0];

                            /*  save ID for frontEgo MIO */
                            PrimeEgoFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check of ego rear MIO */
        if (EgoMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < -1.0F) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if ((Prediction_yEgoLaneRight <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yEgoLaneLeft)) {
                            EgoMinX = LCCRA_TargetVehicle_str
                                          .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                          .Position[0];

                            /*  save ID for rearEgo MIO */
                            PrimeEgoRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for left front MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < LeftAdjMaxX) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if ((Prediction_yEgoLaneLeft <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yAdjLaneLeft)) {
                            LeftAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextFront MIO */
                            PrimeLeftFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for left rear MIO */
        if (LeftAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if ((Prediction_yEgoLaneLeft <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yAdjLaneLeft)) {
                            LeftAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextRear MIO */
                            PrimeLeftRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Ego right front MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < RightAdjMaxX) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if ((Prediction_yAdjLaneRight <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yEgoLaneRight)) {
                            RightAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextFront MIO */
                            PrimeRightFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Ego right rear MIO */
        if (RightAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if ((Prediction_yAdjLaneRight <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yEgoLaneRight)) {
                            RightAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextRear MIO */
                            PrimeRightRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for next left front MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < LeftNextAdjMaxX) {
                if (yAdjLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yNextAdjLaneLeft) {
                        if ((Prediction_yAdjLaneLeft <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yNextAdjLaneLeft)) {
                            LeftNextAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextLeftFront MIO */
                            PrimeNextLeftFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for next left rear MIO */
        if (LeftNextAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yAdjLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yNextAdjLaneLeft) {
                        if ((Prediction_yAdjLaneLeft <=
                             LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yNextAdjLaneLeft)) {
                            LeftNextAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextLeftRear MIO */
                            PrimeNextLeftRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for next right front MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < RightNextAdjMaxX) {
                if (yNextAdjLaneRight <=
                    LCCRA_TargetVehicle_str
                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneRight) {
                        if ((Prediction_x <= LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yAdjLaneRight)) {
                            RightNextAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextFront MIO */
                            PrimeNextRightFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for next right rear MIO */
        if (RightNextAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yNextAdjLaneRight <=
                    LCCRA_TargetVehicle_str
                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneRight) {
                        if ((Prediction_x <= LCCRA_PredictionTime_sec) &&
                            (LCCRA_PredictionTime_sec <=
                             Prediction_yAdjLaneRight)) {
                            RightNextAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for nextRear MIO */
                            PrimeNextRightRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*     %% Check for Cut-in and Cut-out MIOs */
        /*  Check for Ego_to_LeftFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < Ego_to_LeftAdjMaxX) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yEgoLaneLeft) {
                            Ego_to_LeftAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Ego_to_LeftFront MIO */
                            Ego_to_LeftFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Ego_to_LeftRear MIO */
        if (Ego_to_LeftAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yEgoLaneLeft) {
                            Ego_to_LeftAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Ego_to_LeftRear MIO */
                            Ego_to_LeftRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Ego_to_RightFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < Ego_to_RightAdjMaxX) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if (Prediction_yEgoLaneRight >=
                            LCCRA_PredictionTime_sec) {
                            Ego_to_RightAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Ego_to_RightFront MIO */
                            Ego_to_RightFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Ego_to_RightRear MIO */
        if (Ego_to_RightAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yEgoLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneLeft) {
                        if (Prediction_yEgoLaneRight >=
                            LCCRA_PredictionTime_sec) {
                            Ego_to_RightAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Ego_to_RightRear MIO */
                            Ego_to_RightRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Left_to_EgoFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < LeftAdj_to_EgoMaxX) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if (Prediction_yEgoLaneLeft >=
                            LCCRA_PredictionTime_sec) {
                            LeftAdj_to_EgoMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Left_to_EgoFront MIO */
                            Left_to_EgoFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Left_to_EgoRear MIO */
        if (LeftAdj_to_EgoMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if (Prediction_yEgoLaneLeft >=
                            LCCRA_PredictionTime_sec) {
                            LeftAdj_to_EgoMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Left_to_EgoRear MIO */
                            Left_to_EgoRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Right_to_EgoFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < RightAdj_to_EgoMaxX) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yEgoLaneRight) {
                            RightAdj_to_EgoMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Right_to_EgoFront MIO */
                            Right_to_EgoFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Right_to_EgoRear MIO */
        if (RightAdj_to_EgoMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yEgoLaneRight) {
                            RightAdj_to_EgoMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Right_to_EgoFront MIO */
                            Right_to_EgoRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for NextLeft_to_LeftFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < LeftNextAdj_to_LeftAdjMaxX) {
                if (yAdjLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yNextAdjLaneLeft) {
                        if (Prediction_yAdjLaneLeft >=
                            LCCRA_PredictionTime_sec) {
                            LeftNextAdj_to_LeftAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for NextLeft_to_LeftFront MIO */
                            NextLeft_to_LeftFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for NextLeft_to_LeftRear MIO */
        if (LeftNextAdj_to_LeftAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yAdjLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yNextAdjLaneLeft) {
                        if (Prediction_yAdjLaneLeft >=
                            LCCRA_PredictionTime_sec) {
                            LeftNextAdj_to_LeftAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for NextLeft_to_LeftRear MIO */
                            NextLeft_to_LeftRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Left_to_NextLeftFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < LeftAdj_to_LeftNextAdjMaxX) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yAdjLaneLeft) {
                            LeftAdj_to_LeftNextAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Left_to_NextLeftFront MIO */
                            Left_to_NextLeftFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Left_to_NextLeftRear MIO */
        if (LeftAdj_to_LeftNextAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yEgoLaneLeft <= LCCRA_TargetVehicle_str
                                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneLeft) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yAdjLaneLeft) {
                            LeftAdj_to_LeftNextAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Left_to_NextLeftRear MIO */
                            Left_to_NextLeftRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for NextRight_to_RightFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < RightNextAdj_to_RightAdjMaxX) {
                if (yNextAdjLaneRight <=
                    LCCRA_TargetVehicle_str
                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneRight) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yAdjLaneRight) {
                            RightNextAdj_to_RightAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for NextRight_to_RightFront MIO */
                            NextRight_to_RightFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for NextRight_to_RightRear MIO */
        if (RightNextAdj_to_RightAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yNextAdjLaneRight <=
                    LCCRA_TargetVehicle_str
                        .Vehicles[((int32_T)((uint8_T)i)) - 1]
                        .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yAdjLaneRight) {
                        if (LCCRA_PredictionTime_sec >=
                            Prediction_yAdjLaneRight) {
                            RightNextAdj_to_RightAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for NextRight_to_RightRear MIO */
                            NextRight_to_RightRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Right_to_NextRightFront MIO */
        if (0.0F < LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                       .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < RightAdj_to_RightNextAdjMaxX) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if (Prediction_yAdjLaneRight >=
                            LCCRA_PredictionTime_sec) {
                            RightAdj_to_RightNextAdjMaxX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Right_to_NextRightFront MIO */
                            Right_to_NextRightFront.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }

        /*  Check for Right_to_NextRightRear MIO */
        if (RightAdj_to_RightNextAdjMinX <
            LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                .Position[0]) {
            if (LCCRA_TargetVehicle_str.Vehicles[((int32_T)((uint8_T)i)) - 1]
                    .Position[0] < 0.0F) {
                if (yAdjLaneRight <= LCCRA_TargetVehicle_str
                                         .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                         .Position[1]) {
                    if (LCCRA_TargetVehicle_str
                            .Vehicles[((int32_T)((uint8_T)i)) - 1]
                            .Position[1] <= yEgoLaneRight) {
                        if (Prediction_yAdjLaneRight >=
                            LCCRA_PredictionTime_sec) {
                            RightAdj_to_RightNextAdjMinX =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .Position[0];

                            /*  save ID for Right_to_NextRightRear MIO */
                            Right_to_NextRightRear.ID =
                                LCCRA_TargetVehicle_str
                                    .Vehicles[((int32_T)((uint8_T)i)) - 1]
                                    .ID;
                        }
                    }
                }
            }
        }
    }

    /*  Filter MIOs */
    /*  Select EgoFront Object */
    if (((PrimeEgoFront.ID >= 0) || (Left_to_EgoFront.ID >= 0)) ||
        (Right_to_EgoFront.ID >= 0)) {
        if (PrimeEgoFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeEgoFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Left_to_EgoFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Left_to_EgoFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Right_to_EgoFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Right_to_EgoFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        guard1 = false;
        if (PrimeEgoFront.ID >= 0) {
            if (PrimeEgoFront.Position[0] < LCCRA_RiskDistanceX_C_met) {
                EgoFront.ID = PrimeEgoFront.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeEgoFront.TTC < Left_to_EgoFront.TTC) &&
                (PrimeEgoFront.TTC < Right_to_EgoFront.TTC)) {
                EgoFront.ID = PrimeEgoFront.ID;
            } else if ((Left_to_EgoFront.TTC < PrimeEgoFront.TTC) &&
                       (Left_to_EgoFront.TTC < Right_to_EgoFront.TTC)) {
                EgoFront.ID = Left_to_EgoFront.ID;
            } else {
                if ((Right_to_EgoFront.TTC < PrimeEgoFront.TTC) &&
                    (Right_to_EgoFront.TTC < Left_to_EgoFront.TTC)) {
                    EgoFront.ID = Right_to_EgoFront.ID;
                }
            }
        }
    }

    /*  Select EgoRear Object */
    if (((PrimeEgoRear.ID >= 0) || (Left_to_EgoRear.ID >= 0)) ||
        (Right_to_EgoRear.ID >= 0)) {
        if (PrimeEgoRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeEgoRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Left_to_EgoRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Left_to_EgoRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Right_to_EgoRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Right_to_EgoRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        guard1 = false;
        if (PrimeEgoRear.ID >= 0) {
            if (PrimeEgoRear.Position[0] < LCCRA_RiskDistanceX_C_met) {
                EgoRear.ID = PrimeEgoRear.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeEgoRear.TTC < Left_to_EgoRear.TTC) &&
                (PrimeEgoRear.TTC < Right_to_EgoRear.TTC)) {
                EgoRear.ID = PrimeEgoRear.ID;
            } else if ((Left_to_EgoRear.TTC < PrimeEgoRear.TTC) &&
                       (Left_to_EgoRear.TTC < Right_to_EgoRear.TTC)) {
                EgoRear.ID = Left_to_EgoRear.ID;
            } else {
                if ((Right_to_EgoRear.TTC < PrimeEgoRear.TTC) &&
                    (Right_to_EgoRear.TTC < Left_to_EgoRear.TTC)) {
                    EgoRear.ID = Right_to_EgoRear.ID;
                }
            }
        }
    }

    /*  Select LeftFront Object */
    if (((PrimeLeftFront.ID >= 0) || (Ego_to_LeftFront.ID >= 0)) ||
        (NextLeft_to_LeftFront.ID >= 0)) {
        if (PrimeLeftFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeLeftFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Ego_to_LeftFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Ego_to_LeftFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (NextLeft_to_LeftFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &NextLeft_to_LeftFront, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool);
        }

        guard1 = false;
        if (PrimeLeftFront.ID >= 0) {
            if (PrimeLeftFront.Position[0] < LCCRA_RiskDistanceX_C_met) {
                LeftFront.ID = PrimeLeftFront.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeLeftFront.TTC < Ego_to_LeftFront.TTC) &&
                (PrimeLeftFront.TTC < NextLeft_to_LeftFront.TTC)) {
                LeftFront.ID = PrimeLeftFront.ID;
            } else if ((Ego_to_LeftFront.TTC < PrimeLeftFront.TTC) &&
                       (Ego_to_LeftFront.TTC < NextLeft_to_LeftFront.TTC)) {
                LeftFront.ID = Ego_to_LeftFront.ID;
            } else {
                if ((NextLeft_to_LeftFront.TTC < PrimeLeftFront.TTC) &&
                    (NextLeft_to_LeftFront.TTC < Ego_to_LeftFront.TTC)) {
                    LeftFront.ID = NextLeft_to_LeftFront.ID;
                }
            }
        }
    }

    /*  Select LeftRear Object */
    if (((PrimeLeftRear.ID >= 0) || (Ego_to_LeftRear.ID >= 0)) ||
        (NextLeft_to_LeftRear.ID >= 0)) {
        if (PrimeLeftRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeLeftRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Ego_to_LeftRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Ego_to_LeftRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (NextLeft_to_LeftRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &NextLeft_to_LeftRear, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool,
                LCCRA_EgoRearHangDist_C_met);
        }

        guard1 = false;
        if (PrimeLeftRear.ID != 0) {
            if (PrimeLeftRear.Position[0] < LCCRA_RiskDistanceX_C_met) {
                LeftRear.ID = PrimeLeftRear.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeLeftRear.TTC < Ego_to_LeftRear.TTC) &&
                (PrimeLeftRear.TTC < NextLeft_to_LeftRear.TTC)) {
                LeftRear.ID = PrimeLeftRear.ID;
            } else if ((Ego_to_LeftRear.TTC < PrimeLeftRear.TTC) &&
                       (Ego_to_LeftRear.TTC < NextLeft_to_LeftRear.TTC)) {
                LeftRear.ID = Ego_to_LeftRear.ID;
            } else {
                if ((NextLeft_to_LeftRear.TTC < PrimeLeftRear.TTC) &&
                    (NextLeft_to_LeftRear.TTC < Ego_to_LeftRear.TTC)) {
                    LeftRear.ID = NextLeft_to_LeftRear.ID;
                }
            }
        }
    }

    /*  Select RightFront Object */
    if (((PrimeRightFront.ID >= 0) || (Ego_to_RightFront.ID >= 0)) ||
        (NextRight_to_RightFront.ID >= 0)) {
        if (PrimeRightFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeRightFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Ego_to_RightFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Ego_to_RightFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (NextRight_to_RightFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &NextRight_to_RightFront, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool);
        }

        guard1 = false;
        if (PrimeRightFront.ID >= 0) {
            if (PrimeRightFront.Position[0] < LCCRA_RiskDistanceX_C_met) {
                RightFront.ID = PrimeRightFront.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeRightFront.TTC < Ego_to_RightFront.TTC) &&
                (PrimeRightFront.TTC < NextRight_to_RightFront.TTC)) {
                RightFront.ID = PrimeRightFront.ID;
            } else if ((Ego_to_RightFront.TTC < PrimeRightFront.TTC) &&
                       (Ego_to_RightFront.TTC < NextRight_to_RightFront.TTC)) {
                RightFront.ID = Ego_to_RightFront.ID;
            } else {
                if ((NextRight_to_RightFront.TTC < PrimeRightFront.TTC) &&
                    (NextRight_to_RightFront.TTC < Ego_to_RightFront.TTC)) {
                    RightFront.ID = NextRight_to_RightFront.ID;
                }
            }
        }
    }

    /*  Select RightRear Object */
    if (((PrimeRightRear.ID >= 0) || (Ego_to_RightRear.ID >= 0)) ||
        (NextRight_to_RightRear.ID >= 0)) {
        if (PrimeRightRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeRightRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Ego_to_RightRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &Ego_to_RightRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (NextRight_to_RightRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &NextRight_to_RightRear, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool,
                LCCRA_EgoRearHangDist_C_met);
        }

        guard1 = false;
        if (PrimeRightRear.ID >= 0) {
            if (PrimeRightRear.Position[0] < LCCRA_RiskDistanceX_C_met) {
                RightRear.ID = PrimeRightRear.ID;
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            if ((PrimeRightRear.TTC < Ego_to_RightRear.TTC) &&
                (PrimeRightRear.TTC < NextRight_to_RightRear.TTC)) {
                RightRear.ID = PrimeRightRear.ID;
            } else if ((Ego_to_RightRear.TTC < PrimeRightRear.TTC) &&
                       (Ego_to_RightRear.TTC < NextRight_to_RightRear.TTC)) {
                RightRear.ID = Ego_to_RightRear.ID;
            } else {
                if ((NextRight_to_RightRear.TTC < PrimeRightRear.TTC) &&
                    (NextRight_to_RightRear.TTC < Ego_to_RightRear.TTC)) {
                    RightRear.ID = NextRight_to_RightRear.ID;
                }
            }
        }
    }

    /*  Select NextLeftFront Object */
    if ((PrimeNextLeftFront.ID >= 0) || (Left_to_NextLeftFront.ID >= 0)) {
        if (PrimeNextLeftFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeNextLeftFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Left_to_NextLeftFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &Left_to_NextLeftFront, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool);
        }

        if ((PrimeNextLeftFront.ID >= 0) && (Left_to_NextLeftFront.ID >= 0)) {
            if (PrimeNextLeftFront.TTC < Left_to_NextLeftFront.TTC) {
                NextLeftFront.ID = PrimeNextLeftFront.ID;
            } else {
                NextLeftFront.ID = Left_to_NextLeftFront.ID;
            }
        } else if (PrimeNextLeftFront.ID >= 0) {
            NextLeftFront.ID = PrimeNextLeftFront.ID;
        } else {
            NextLeftFront.ID = Left_to_NextLeftFront.ID;
        }
    }

    /*  Select NextLeftRear Object */
    if ((PrimeNextLeftRear.ID >= 0) || (Left_to_NextLeftRear.ID >= 0)) {
        if (PrimeNextLeftRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeNextLeftRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Left_to_NextLeftRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &Left_to_NextLeftRear, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool,
                LCCRA_EgoRearHangDist_C_met);
        }

        if ((PrimeNextLeftRear.ID >= 0) && (Left_to_NextLeftRear.ID >= 0)) {
            if (PrimeNextLeftRear.TTC < Left_to_NextLeftRear.TTC) {
                NextLeftRear.ID = PrimeNextLeftRear.ID;
            } else {
                NextLeftRear.ID = Left_to_NextLeftRear.ID;
            }
        } else if (PrimeNextLeftRear.ID >= 0) {
            NextLeftRear.ID = PrimeNextLeftRear.ID;
        } else {
            NextLeftRear.ID = Left_to_NextLeftRear.ID;
        }
    }

    /*  Select NextRightFront Object */
    if ((PrimeNextRightFront.ID >= 0) || (Right_to_NextRightFront.ID >= 0)) {
        if (PrimeNextRightFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeNextRightFront,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool);
        }

        if (Right_to_NextRightFront.ID >= 0) {
            LCCRA_CalculationFrontobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &Right_to_NextRightFront, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool);
        }

        if ((PrimeNextRightFront.ID >= 0) &&
            (Right_to_NextRightFront.ID >= 0)) {
            if (PrimeNextRightFront.TTC < Right_to_NextRightFront.TTC) {
                NextRightFront.ID = PrimeNextRightFront.ID;
            } else {
                NextRightFront.ID = Right_to_NextRightFront.ID;
            }
        } else if (PrimeNextRightFront.ID >= 0) {
            NextRightFront.ID = PrimeNextRightFront.ID;
        } else {
            NextRightFront.ID = Right_to_NextRightFront.ID;
        }
    }

    /*  Select NextRightRear Object */
    if ((PrimeNextRightRear.ID >= 0) || (Right_to_NextRightRear.ID >= 0)) {
        if (PrimeNextRightRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &PrimeNextRightRear,
                LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        }

        if (Right_to_NextRightRear.ID >= 0) {
            LCCRA_CalculationRearobject(
                VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st,
                &Right_to_NextRightRear, LCCRA_TTCSafeDistance_C_met,
                LCFRCV_SysCycleTimeSen_sec, LCCRA_UseObjLength_bool,
                LCCRA_EgoRearHangDist_C_met);
        }

        if ((PrimeNextRightRear.ID >= 0) && (Right_to_NextRightRear.ID >= 0)) {
            if (PrimeNextRightRear.TTC < Right_to_NextRightRear.TTC) {
                NextRightRear.ID = PrimeNextRightRear.ID;
            } else {
                NextRightRear.ID = Right_to_NextRightRear.ID;
            }
        } else if (PrimeNextRightRear.ID >= 0) {
            NextRightRear.ID = PrimeNextRightRear.ID;
        } else {
            NextRightRear.ID = Right_to_NextRightRear.ID;
        }
    }

    /*  Get the MIOs of the last cycle  */
    /*  egoFront MIO detected? */
    if (EgoFront.ID >= 0) {
        LCCRA_CalculationFrontobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &EgoFront,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool);
        if (EgoFront.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_egofrontweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_egofrontweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_egofrontweight_nu = (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_egofrontweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_egofrontweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_egofrontweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_egofrontweight_nu) > 0) {
            LCCRA_DW.LCCRA_egofrontdelaytime_sec += LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_egofrontdelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_egofrontweight_nu) - 1]) {
                EgoFront.ID = LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].ID;
                LCCRA_CalculationFrontobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &EgoFront,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool);
            } else {
                LCCRA_DW.LCCRA_egofrontweight_nu = 0U;
                LCCRA_DW.LCCRA_egofrontdelaytime_sec = 0.0F;
            }
        }
    }

    /*  egoRear MIO detected? */
    if (EgoRear.ID >= 0) {
        LCCRA_CalculationRearobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &EgoRear,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        if (EgoRear.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[1].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_egorearweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_egorearweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_egorearweight_nu = (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_egorearweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_egorearweight_nu = LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_egorearweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_egorearweight_nu) > 0) {
            LCCRA_DW.LCCRA_egoreardelaytime_sec += LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_egoreardelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_egorearweight_nu) - 1]) {
                EgoRear.ID = LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[1].ID;
                LCCRA_CalculationRearobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &EgoRear,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
            } else {
                LCCRA_DW.LCCRA_egorearweight_nu = 0U;
                LCCRA_DW.LCCRA_egoreardelaytime_sec = 0.0F;
            }
        }
    }

    /*  leftFront MIO detected? */
    if (LeftFront.ID >= 0) {
        LCCRA_CalculationFrontobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &LeftFront,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool);
        if (LeftFront.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_leftfrontweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_leftfrontweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_leftfrontweight_nu = (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_leftfrontweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_leftfrontweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_leftfrontweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_leftfrontweight_nu) > 0) {
            LCCRA_DW.LCCRA_leftfrontdelaytime_sec += LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_leftfrontdelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_leftfrontweight_nu) - 1]) {
                LeftFront.ID = LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].ID;
                LCCRA_CalculationFrontobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &LeftFront,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool);
            } else {
                LCCRA_DW.LCCRA_leftfrontweight_nu = 0U;
                LCCRA_DW.LCCRA_leftfrontdelaytime_sec = 0.0F;
            }
        }
    }

    /*  leftRear MIO detected?  */
    if (LeftRear.ID >= 0) {
        LCCRA_CalculationRearobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &LeftRear,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        if (LeftRear.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_leftrearweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_leftrearweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_leftrearweight_nu = (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_leftrearweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_leftrearweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_leftrearweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_leftrearweight_nu) > 0) {
            LCCRA_DW.LCCRA_leftreardelaytime_sec += LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_leftreardelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_leftrearweight_nu) - 1]) {
                LeftRear.ID = LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].ID;
                LCCRA_CalculationRearobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &LeftRear,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
            } else {
                LCCRA_DW.LCCRA_leftrearweight_nu = 0U;
                LCCRA_DW.LCCRA_leftreardelaytime_sec = 0.0F;
            }
        }
    }

    /*  rightFront MIO detected? */
    if (RightFront.ID >= 0) {
        LCCRA_CalculationFrontobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &RightFront,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool);
        if (RightFront.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_rightfrontweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_rightfrontweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_rightfrontweight_nu =
                (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_rightfrontweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_rightfrontweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_rightfrontweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_rightfrontweight_nu) > 0) {
            LCCRA_DW.LCCRA_rightfrontdelaytime_sec +=
                LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_rightfrontdelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_rightfrontweight_nu) - 1]) {
                RightFront.ID =
                    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].ID;
                LCCRA_CalculationFrontobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &RightFront,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool);
            } else {
                LCCRA_DW.LCCRA_rightfrontweight_nu = 0U;
                LCCRA_DW.LCCRA_rightfrontdelaytime_sec = 0.0F;
            }
        }
    }

    /*  rightRear MIO detected? */
    if (RightRear.ID >= 0) {
        LCCRA_CalculationRearobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &RightRear,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        if (RightRear.ID == LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_rightrearweight_nu) + 1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_rightrearweight_nu) + 1U) > 255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_rightrearweight_nu = (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_rightrearweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_rightrearweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_rightrearweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_rightrearweight_nu) > 0) {
            LCCRA_DW.LCCRA_rightreardelaytime_sec += LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_rightreardelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_rightrearweight_nu) - 1]) {
                RightRear.ID = LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].ID;
                LCCRA_CalculationRearobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &RightRear,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
            } else {
                LCCRA_DW.LCCRA_rightrearweight_nu = 0U;
                LCCRA_DW.LCCRA_rightreardelaytime_sec = 0.0F;
            }
        }
    }

    /*  NextleftFront MIO detected? */
    if (NextLeftFront.ID >= 0) {
        LCCRA_CalculationFrontobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextLeftFront,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool);
        if (NextLeftFront.ID ==
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_nextleftfrontweight_nu) +
                          1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_nextleftfrontweight_nu) + 1U) >
                255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_nextleftfrontweight_nu =
                (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_nextleftfrontweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_nextleftfrontweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_nextleftfrontweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_nextleftfrontweight_nu) > 0) {
            LCCRA_DW.LCCRA_nextleftfrontdelaytime_se +=
                LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_nextleftfrontdelaytime_se <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_nextleftfrontweight_nu) - 1]) {
                NextLeftFront.ID =
                    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].ID;
                LCCRA_CalculationFrontobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextLeftFront,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool);
            } else {
                LCCRA_DW.LCCRA_nextleftfrontweight_nu = 0U;
                LCCRA_DW.LCCRA_nextleftfrontdelaytime_se = 0.0F;
            }
        }
    }

    /*  NextleftRear MIO detected?  */
    if (NextLeftRear.ID >= 0) {
        LCCRA_CalculationRearobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextLeftRear,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        if (NextLeftRear.ID ==
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_nextleftrearweight_nu) +
                          1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_nextleftrearweight_nu) + 1U) >
                255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_nextleftrearweight_nu =
                (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_nextleftrearweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_nextleftrearweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_nextleftrearweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_nextleftrearweight_nu) > 0) {
            LCCRA_DW.LCCRA_nextleftreardelaytime_sec +=
                LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_nextleftreardelaytime_sec <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_nextleftrearweight_nu) - 1]) {
                NextLeftRear.ID =
                    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].ID;
                LCCRA_CalculationRearobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextLeftRear,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
            } else {
                LCCRA_DW.LCCRA_nextleftrearweight_nu = 0U;
                LCCRA_DW.LCCRA_nextleftreardelaytime_sec = 0.0F;
            }
        }
    }

    /*  NextrightFront MIO detected? */
    if (NextRightFront.ID >= 0) {
        LCCRA_CalculationFrontobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextRightFront,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool);
        if (NextRightFront.ID ==
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_nextrightfrontweight_nu) +
                          1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_nextrightfrontweight_nu) + 1U) >
                255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_nextrightfrontweight_nu =
                (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_nextrightfrontweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_nextrightfrontweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_nextrightfrontweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_nextrightfrontweight_nu) > 0) {
            LCCRA_DW.LCCRA_nextrightfrontdelaytime_s +=
                LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_nextrightfrontdelaytime_s <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_nextrightfrontweight_nu) - 1]) {
                NextRightFront.ID =
                    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].ID;
                LCCRA_CalculationFrontobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextRightFront,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool);
            } else {
                LCCRA_DW.LCCRA_nextrightfrontweight_nu = 0U;
                LCCRA_DW.LCCRA_nextrightfrontdelaytime_s = 0.0F;
            }
        }
    }

    /*  NextrightRear MIO detected? */
    if (NextRightRear.ID >= 0) {
        LCCRA_CalculationRearobject(
            VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
            &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextRightRear,
            LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
            LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
        if (NextRightRear.ID ==
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].ID) {
            LCCRA_Sensorsource_Cam = (int32_T)((
                uint32_T)(((uint32_T)LCCRA_DW.LCCRA_nextrightrearweight_nu) +
                          1U));
            if ((((uint32_T)LCCRA_DW.LCCRA_nextrightrearweight_nu) + 1U) >
                255U) {
                LCCRA_Sensorsource_Cam = 255;
            }

            LCCRA_DW.LCCRA_nextrightrearweight_nu =
                (uint8_T)LCCRA_Sensorsource_Cam;
            if (LCCRA_DW.LCCRA_nextrightrearweight_nu >=
                LCCRA_MaxWeightThreshold_C_nu) {
                LCCRA_DW.LCCRA_nextrightrearweight_nu =
                    LCCRA_MaxWeightThreshold_C_nu;
            }
        } else {
            LCCRA_DW.LCCRA_nextrightrearweight_nu = 0U;
        }
    } else {
        if (((int32_T)LCCRA_DW.LCCRA_nextrightrearweight_nu) > 0) {
            LCCRA_DW.LCCRA_nextrightreardelaytime_se +=
                LCFRCV_SysCycleTimeSen_sec;
            if (LCCRA_DW.LCCRA_nextrightreardelaytime_se <=
                LCCRA_DelayTimeValue_C_nu
                    [((int32_T)LCCRA_DW.LCCRA_nextrightrearweight_nu) - 1]) {
                NextRightRear.ID =
                    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].ID;
                LCCRA_CalculationRearobject(
                    VED_EgoVelocity_mps, (&(LCCRA_TargetVehicle_str)),
                    &LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st, &NextRightRear,
                    LCCRA_TTCSafeDistance_C_met, LCFRCV_SysCycleTimeSen_sec,
                    LCCRA_UseObjLength_bool, LCCRA_EgoRearHangDist_C_met);
            } else {
                LCCRA_DW.LCCRA_nextrightrearweight_nu = 0U;
                LCCRA_DW.LCCRA_nextrightreardelaytime_se = 0.0F;
            }
        }
    }

    /*  Debug */
    LCCRA_DebugWeight_str.LCCRA_egofrontweight_nu =
        LCCRA_DW.LCCRA_egofrontweight_nu;
    LCCRA_DebugWeight_str.LCCRA_egorearweight_nu =
        LCCRA_DW.LCCRA_egorearweight_nu;
    LCCRA_DebugWeight_str.LCCRA_leftfrontweight_nu =
        LCCRA_DW.LCCRA_leftfrontweight_nu;
    LCCRA_DebugWeight_str.LCCRA_leftrearweight_nu =
        LCCRA_DW.LCCRA_leftrearweight_nu;
    LCCRA_DebugWeight_str.LCCRA_rightfrontweight_nu =
        LCCRA_DW.LCCRA_rightfrontweight_nu;
    LCCRA_DebugWeight_str.LCCRA_rightrearweight_nu =
        LCCRA_DW.LCCRA_rightrearweight_nu;
    LCCRA_DebugWeight_str.LCCRA_nextleftfrontweight_nu =
        LCCRA_DW.LCCRA_nextleftfrontweight_nu;
    LCCRA_DebugWeight_str.LCCRA_nextleftrearweight_nu =
        LCCRA_DW.LCCRA_nextleftrearweight_nu;
    LCCRA_DebugWeight_str.LCCRA_nextrightfrontweight_nu =
        LCCRA_DW.LCCRA_nextrightfrontweight_nu;
    LCCRA_DebugWeight_str.LCCRA_nextrightrearweight_nu =
        LCCRA_DW.LCCRA_nextrightrearweight_nu;
    LCCRA_DebugDelaytime_str.LCCRA_egofrontdelaytime_sec =
        LCCRA_DW.LCCRA_egofrontdelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_egoreardelaytime_sec =
        LCCRA_DW.LCCRA_egoreardelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_leftfrontdelaytime_sec =
        LCCRA_DW.LCCRA_leftfrontdelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_leftreardelaytime_sec =
        LCCRA_DW.LCCRA_leftreardelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_rightfrontdelaytime_sec =
        LCCRA_DW.LCCRA_rightfrontdelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_rightreardelaytime_sec =
        LCCRA_DW.LCCRA_rightreardelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_nextleftfrontdelaytime_sec =
        LCCRA_DW.LCCRA_nextleftfrontdelaytime_se;
    LCCRA_DebugDelaytime_str.LCCRA_nextleftreardelaytime_sec =
        LCCRA_DW.LCCRA_nextleftreardelaytime_sec;
    LCCRA_DebugDelaytime_str.LCCRA_nextrightfrontdelaytime_sec =
        LCCRA_DW.LCCRA_nextrightfrontdelaytime_s;
    LCCRA_DebugDelaytime_str.LCCRA_nextrightreardelaytime_sec =
        LCCRA_DW.LCCRA_nextrightreardelaytime_se;
    LCCRA_DebugMIOs_str.DebugMIOs[0] = EgoFront;
    LCCRA_DebugMIOs_str.DebugMIOs[1] = EgoRear;
    LCCRA_DebugMIOs_str.DebugMIOs[2] = LeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[3] = LeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[4] = RightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[5] = RightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[6] = NextLeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[7] = NextLeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[8] = NextRightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[9] = NextRightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[10] = PrimeEgoFront;
    LCCRA_DebugMIOs_str.DebugMIOs[11] = PrimeEgoRear;
    LCCRA_DebugMIOs_str.DebugMIOs[12] = PrimeLeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[13] = PrimeLeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[14] = PrimeRightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[15] = PrimeRightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[16] = PrimeNextLeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[17] = PrimeNextLeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[18] = PrimeNextRightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[19] = PrimeNextRightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[20] = Ego_to_LeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[21] = Ego_to_LeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[22] = Ego_to_RightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[23] = Ego_to_RightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[24] = Left_to_EgoFront;
    LCCRA_DebugMIOs_str.DebugMIOs[25] = Left_to_EgoRear;
    LCCRA_DebugMIOs_str.DebugMIOs[26] = Right_to_EgoFront;
    LCCRA_DebugMIOs_str.DebugMIOs[27] = Right_to_EgoRear;
    LCCRA_DebugMIOs_str.DebugMIOs[28] = NextLeft_to_LeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[29] = NextLeft_to_LeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[30] = Left_to_NextLeftFront;
    LCCRA_DebugMIOs_str.DebugMIOs[31] = Left_to_NextLeftRear;
    LCCRA_DebugMIOs_str.DebugMIOs[32] = NextRight_to_RightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[33] = NextRight_to_RightRear;
    LCCRA_DebugMIOs_str.DebugMIOs[34] = Right_to_NextRightFront;
    LCCRA_DebugMIOs_str.DebugMIOs[35] = Right_to_NextRightRear;

    /*  Output */
    LCCRA_DW.LCCRA_MIOUnitDelay_str.NumMIOs = 10U;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0] = EgoFront;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[1] = EgoRear;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2] = LeftFront;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3] = LeftRear;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4] = RightFront;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5] = RightRear;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6] = NextLeftFront;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7] = NextLeftRear;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8] = NextRightFront;
    LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9] = NextRightRear;

    /* End of MATLAB Function: '<S1>/LCCRA_FindMIOs' */

    /* MultiPortSwitch: '<S23>/Multiport Switch2' incorporates:
     *  Constant: '<S1>/Constant1'
     *  Constant: '<S1>/Constant34'
     *  Selector: '<S15>/Selector3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].Type) {
        case 0:
            rtb_MultiportSwitch2 = LCCRA_RearTTCThreshold_C_sec;
            break;

        case 1:
            rtb_MultiportSwitch2 = LCCRA_RearTTCThresholdBus_C_sec;
            break;

        default:
            rtb_MultiportSwitch2 = LCCRA_RearTTCThreshold_C_sec;
            break;
    }

    /* End of MultiPortSwitch: '<S23>/Multiport Switch2' */

    /* Switch: '<S32>/Switch' incorporates:
     *  Constant: '<S1>/Constant26'
     *  RelationalOperator: '<S32>/Less Than'
     *  Selector: '<S15>/Selector3'
     *  Sum: '<S23>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_MultiportSwitch2 + LCCRA_TTCThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].TTC) {
        /* Switch: '<S32>/Switch' incorporates:
         *  Constant: '<S32>/Constant'
         */
        LCCRA_bLeftRearTTCSafe = true;
    } else {
        /* Switch: '<S32>/Switch' incorporates:
         *  RelationalOperator: '<S32>/Less Than1'
         *  UnitDelay: '<S32>/Unit Delay'
         */
        LCCRA_bLeftRearTTCSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].TTC >=
              rtb_MultiportSwitch2) &&
             (LCCRA_bLastLeftRearTTCSafe));
    }

    /* End of Switch: '<S32>/Switch' */

    /* MultiPortSwitch: '<S22>/Multiport Switch1' incorporates:
     *  Constant: '<S22>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S22>/1-D Lookup Table4'
     *  Lookup_n-D: '<S22>/1-D Lookup Table5'
     *  Product: '<S22>/Product'
     *  Selector: '<S15>/Selector3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S22>/Multiport Switch1' */

    /* Product: '<S22>/Product1' incorporates:
     *  Constant: '<S22>/Constant2'
     *  Lookup_n-D: '<S22>/1-D Lookup Table2'
     *  Selector: '<S15>/Selector3'
     *  Selector: '<S22>/Selector2'
     *  UnaryMinus: '<S22>/Unary Minus'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1 =
        look1_iflf_binlxpw(
            -LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S31>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S31>/Less Than'
     *  Selector: '<S15>/Selector3'
     *  Sum: '<S22>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1 + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].TimeGap) {
        /* Switch: '<S31>/Switch' incorporates:
         *  Constant: '<S31>/Constant'
         */
        LCCRA_bLeftRearTGSafe = true;
    } else {
        /* Switch: '<S31>/Switch' incorporates:
         *  RelationalOperator: '<S31>/Less Than1'
         *  UnitDelay: '<S31>/Unit Delay'
         */
        LCCRA_bLeftRearTGSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].TimeGap >=
              rtb_Product1) &&
             (LCCRA_bLastLeftRearTGSafe));
    }

    /* End of Switch: '<S31>/Switch' */

    /* Sum: '<S21>/Add' incorporates:
     *  Constant: '<S1>/Constant22'
     *  Constant: '<S1>/Constant28'
     *  Sum: '<S18>/Add'
     *  Sum: '<S38>/Add'
     *  Sum: '<S41>/Add'
     */
    rtb_MultiportSwitch2 =
        LCCRA_RiskDistance_C_met + LCCRA_RiskDistThresholdHys_C_met;

    /* Switch: '<S30>/Switch' incorporates:
     *  RelationalOperator: '<S30>/Less Than'
     *  Selector: '<S15>/Selector3'
     *  Sum: '<S21>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (rtb_MultiportSwitch2 <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].RangeMagnitude) {
        /* Switch: '<S30>/Switch' incorporates:
         *  Constant: '<S30>/Constant'
         */
        LCCRA_bLeftRearRDSafe = true;
    } else {
        /* Switch: '<S30>/Switch' incorporates:
         *  Constant: '<S1>/Constant22'
         *  RelationalOperator: '<S30>/Less Than1'
         *  UnitDelay: '<S30>/Unit Delay'
         */
        LCCRA_bLeftRearRDSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].RangeMagnitude >=
              LCCRA_RiskDistance_C_met) &&
             (LCCRA_bLastLeftRearRDSafe));
    }

    /* End of Switch: '<S30>/Switch' */

    /* Logic: '<S15>/AND1' */
    rtb_bLeftRearSafe = ((LCCRA_bLeftRearTTCSafe && LCCRA_bLeftRearTGSafe) &&
                         LCCRA_bLeftRearRDSafe);

    /* MultiPortSwitch: '<S20>/Multiport Switch2' incorporates:
     *  Constant: '<S1>/Constant'
     *  Constant: '<S1>/Constant33'
     *  Selector: '<S15>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThreshold_C_sec;
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThresholdBus_C_sec;
            break;

        default:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThreshold_C_sec;
            break;
    }

    /* End of MultiPortSwitch: '<S20>/Multiport Switch2' */

    /* Switch: '<S29>/Switch' incorporates:
     *  Constant: '<S1>/Constant26'
     *  RelationalOperator: '<S29>/Less Than'
     *  Selector: '<S15>/Selector1'
     *  Sum: '<S20>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_MultiportSwitch1_jddt + LCCRA_TTCThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].TTC) {
        /* Switch: '<S29>/Switch' incorporates:
         *  Constant: '<S29>/Constant'
         */
        LCCRA_bLeftFrontTTCSafe = true;
    } else {
        /* Switch: '<S29>/Switch' incorporates:
         *  RelationalOperator: '<S29>/Less Than1'
         *  UnitDelay: '<S29>/Unit Delay'
         */
        LCCRA_bLeftFrontTTCSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].TTC >=
              rtb_MultiportSwitch1_jddt) &&
             (LCCRA_bLastLeftFrontTTCSafe));
    }

    /* End of Switch: '<S29>/Switch' */

    /* MultiPortSwitch: '<S19>/Multiport Switch1' incorporates:
     *  Constant: '<S19>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S19>/1-D Lookup Table1'
     *  Lookup_n-D: '<S19>/1-D Lookup Table3'
     *  Product: '<S19>/Product'
     *  Selector: '<S15>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S19>/Multiport Switch1' */

    /* Product: '<S19>/Product1' incorporates:
     *  Constant: '<S19>/Constant2'
     *  Lookup_n-D: '<S19>/1-D Lookup Table2'
     *  Selector: '<S15>/Selector1'
     *  Selector: '<S19>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1_aqbo =
        look1_iflf_binlxpw(
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S28>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S28>/Less Than'
     *  Selector: '<S15>/Selector1'
     *  Sum: '<S19>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1_aqbo + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].TimeGap) {
        /* Switch: '<S28>/Switch' incorporates:
         *  Constant: '<S28>/Constant'
         */
        LCCRA_bLeftFrontTGSafe = true;
    } else {
        /* Switch: '<S28>/Switch' incorporates:
         *  RelationalOperator: '<S28>/Less Than1'
         *  UnitDelay: '<S28>/Unit Delay'
         */
        LCCRA_bLeftFrontTGSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].TimeGap >=
              rtb_Product1_aqbo) &&
             (LCCRA_bLastLeftFrontTGSafe));
    }

    /* End of Switch: '<S28>/Switch' */

    /* Switch: '<S27>/Switch' incorporates:
     *  RelationalOperator: '<S27>/Less Than'
     *  Selector: '<S15>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (rtb_MultiportSwitch2 <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].RangeMagnitude) {
        /* Switch: '<S27>/Switch' incorporates:
         *  Constant: '<S27>/Constant'
         */
        LCCRA_bLeftFrontRDSafe = true;
    } else {
        /* Switch: '<S27>/Switch' incorporates:
         *  Constant: '<S1>/Constant22'
         *  RelationalOperator: '<S27>/Less Than1'
         *  UnitDelay: '<S27>/Unit Delay'
         */
        LCCRA_bLeftFrontRDSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].RangeMagnitude >=
              LCCRA_RiskDistance_C_met) &&
             (LCCRA_bLastLeftFrontRDSafe));
    }

    /* End of Switch: '<S27>/Switch' */

    /* Logic: '<S15>/AND' */
    rtb_bLeftFrontSafe = ((LCCRA_bLeftFrontTTCSafe && LCCRA_bLeftFrontTGSafe) &&
                          LCCRA_bLeftFrontRDSafe);

    /* MultiPortSwitch: '<S25>/Multiport Switch1' incorporates:
     *  Constant: '<S25>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S25>/1-D Lookup Table1'
     *  Lookup_n-D: '<S25>/1-D Lookup Table3'
     *  Product: '<S25>/Product'
     *  Selector: '<S15>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S25>/Multiport Switch1' */

    /* Product: '<S25>/Product1' incorporates:
     *  Constant: '<S25>/Constant2'
     *  Lookup_n-D: '<S25>/1-D Lookup Table2'
     *  Selector: '<S15>/Selector2'
     *  Selector: '<S25>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1_kv1k =
        look1_iflf_binlxpw(
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S33>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S33>/Less Than'
     *  Selector: '<S15>/Selector2'
     *  Sum: '<S25>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1_kv1k + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].TimeGap) {
        /* Switch: '<S33>/Switch' incorporates:
         *  Constant: '<S33>/Constant'
         */
        LCCRA_bNextLeftFrontSafe = true;
    } else {
        /* Switch: '<S33>/Switch' incorporates:
         *  RelationalOperator: '<S33>/Less Than1'
         *  UnitDelay: '<S33>/Unit Delay'
         */
        LCCRA_bNextLeftFrontSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].TimeGap >=
              rtb_Product1_kv1k) &&
             (LCCRA_bLastNextLeftFrontSafe));
    }

    /* End of Switch: '<S33>/Switch' */

    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S1>/Constant6'
     *  Constant: '<S25>/Constant1'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport31'
     *  Product: '<S25>/Product3'
     *  RelationalOperator: '<S25>/GreaterThan'
     *  Selector: '<S15>/Selector2'
     *  Selector: '<S25>/Selector1'
     *  Sum: '<S25>/Subtract'
     *  Trigonometry: '<S25>/Sin'
     *  UnaryMinus: '<S25>/Unary Minus'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (LCCRA_bNextLeftFrontSafe) {
        rtb_NextLeftFrontSafeFlag = true;
    } else {
        rtb_NextLeftFrontSafeFlag =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].Velocity[1] -
              (VED_EgoVelocity_mps * sinf(LBP_LeLnClthHeading_rad))) >
             (-LCCRA_VelocityThreshold_C_mps));
    }

    /* End of Switch: '<S25>/Switch' */

    /* MultiPortSwitch: '<S26>/Multiport Switch1' incorporates:
     *  Constant: '<S26>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S26>/1-D Lookup Table4'
     *  Lookup_n-D: '<S26>/1-D Lookup Table5'
     *  Product: '<S26>/Product'
     *  Selector: '<S15>/Selector4'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S26>/Multiport Switch1' */

    /* Product: '<S26>/Product1' incorporates:
     *  Constant: '<S26>/Constant2'
     *  Lookup_n-D: '<S26>/1-D Lookup Table2'
     *  Selector: '<S15>/Selector4'
     *  Selector: '<S26>/Selector2'
     *  UnaryMinus: '<S26>/Unary Minus1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1_bzrt =
        look1_iflf_binlxpw(
            -LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S34>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S34>/Less Than'
     *  Selector: '<S15>/Selector4'
     *  Sum: '<S26>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1_bzrt + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].TimeGap) {
        /* Switch: '<S34>/Switch' incorporates:
         *  Constant: '<S34>/Constant'
         */
        LCCRA_bNextLeftRearSafe = true;
    } else {
        /* Switch: '<S34>/Switch' incorporates:
         *  RelationalOperator: '<S34>/Less Than1'
         *  UnitDelay: '<S34>/Unit Delay'
         */
        LCCRA_bNextLeftRearSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].TimeGap >=
              rtb_Product1_bzrt) &&
             (LCCRA_bLastNextLeftRearSafe));
    }

    /* End of Switch: '<S34>/Switch' */

    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S1>/Constant6'
     *  Constant: '<S26>/Constant1'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport31'
     *  Product: '<S26>/Product3'
     *  RelationalOperator: '<S26>/GreaterThan'
     *  Selector: '<S15>/Selector4'
     *  Selector: '<S26>/Selector1'
     *  Sum: '<S26>/Subtract'
     *  Trigonometry: '<S26>/Sin'
     *  UnaryMinus: '<S26>/Unary Minus'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (LCCRA_bNextLeftRearSafe) {
        rtb_NextLeftRearSafeFlag = true;
    } else {
        rtb_NextLeftRearSafeFlag =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].Velocity[1] -
              (VED_EgoVelocity_mps * sinf(LBP_LeLnClthHeading_rad))) >
             (-LCCRA_VelocityThreshold_C_mps));
    }

    /* End of Switch: '<S26>/Switch' */

    /* MATLAB Function: '<S15>/MATLAB Function' incorporates:
     *  Selector: '<S15>/Selector1'
     *  Selector: '<S15>/Selector2'
     *  Selector: '<S15>/Selector3'
     *  Selector: '<S15>/Selector4'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    LCCRA_LeftDangerObjID_nu = -1;
    rtb_MultiportSwitch1_jddt = 50000.0F;
    if (((rtb_bLeftRearSafe && rtb_bLeftFrontSafe) &&
         rtb_NextLeftFrontSafeFlag) &&
        rtb_NextLeftRearSafeFlag) {
        LCCRA_LeftSafeFlag_bool = true;
        LCCRA_LeftDangerObjID_nu = -1;
    } else {
        LCCRA_LeftSafeFlag_bool = false;
        if (!rtb_bLeftFrontSafe) {
            LCCRA_LeftDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].ID;

            /*          fMinTTC = LeftFront.TTC; */
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[2].RangeMagnitude;
        }

        if ((!rtb_bLeftRearSafe) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_LeftDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].ID;

            /*          fMinTTC = LeftRear.TTC; */
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[3].RangeMagnitude;
        }

        if ((!rtb_NextLeftFrontSafeFlag) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_LeftDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].ID;

            /*          fMinTTC = LeftRear.TTC; */
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[6].RangeMagnitude;
        }

        if ((!rtb_NextLeftRearSafeFlag) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_LeftDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[7].ID;

            /*          fMinTTC = LeftRear.TTC; */
            /*          fMinRangeMagnitude = NextLeftRear.RangeMagnitude; */
        }
    }

    /* End of MATLAB Function: '<S15>/MATLAB Function' */

    /* Logic: '<S15>/AND2' */
    LCCRA_LeftFrontSafeFlag_bool =
        (rtb_bLeftFrontSafe && rtb_NextLeftFrontSafeFlag);

    /* Logic: '<S15>/AND3' */
    LCCRA_LeftRearSafeFlag_bool =
        (rtb_bLeftRearSafe && rtb_NextLeftRearSafeFlag);

    /* Product: '<S26>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_NextLeftRearSfDistTmGp_met = rtb_Product1_bzrt * VED_EgoVelocity_mps;

    /* Product: '<S25>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_NextLeftFrontSfDistTmGp_met = rtb_Product1_kv1k * VED_EgoVelocity_mps;

    /* Product: '<S19>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_LeftFrontSafeDistTmGp_met = rtb_Product1_aqbo * VED_EgoVelocity_mps;

    /* Product: '<S22>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_LeftRearSafeDistTmGp_met = rtb_Product1 * VED_EgoVelocity_mps;

    /* MultiPortSwitch: '<S43>/Multiport Switch2' incorporates:
     *  Constant: '<S1>/Constant1'
     *  Constant: '<S1>/Constant34'
     *  Selector: '<S16>/Selector3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].Type) {
        case 0:
            rtb_Product1 = LCCRA_RearTTCThreshold_C_sec;
            break;

        case 1:
            rtb_Product1 = LCCRA_RearTTCThresholdBus_C_sec;
            break;

        default:
            rtb_Product1 = LCCRA_RearTTCThreshold_C_sec;
            break;
    }

    /* End of MultiPortSwitch: '<S43>/Multiport Switch2' */

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S1>/Constant26'
     *  RelationalOperator: '<S51>/Less Than'
     *  Selector: '<S16>/Selector3'
     *  Sum: '<S43>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1 + LCCRA_TTCThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].TTC) {
        /* Switch: '<S51>/Switch' incorporates:
         *  Constant: '<S51>/Constant'
         */
        LCCRA_bRightRearTTCSafe = true;
    } else {
        /* Switch: '<S51>/Switch' incorporates:
         *  RelationalOperator: '<S51>/Less Than1'
         *  UnitDelay: '<S51>/Unit Delay'
         */
        LCCRA_bRightRearTTCSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].TTC >=
              rtb_Product1) &&
             (LCCRA_bLastRightRearTTCSafe));
    }

    /* End of Switch: '<S51>/Switch' */

    /* MultiPortSwitch: '<S42>/Multiport Switch1' incorporates:
     *  Constant: '<S42>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S42>/1-D Lookup Table4'
     *  Lookup_n-D: '<S42>/1-D Lookup Table5'
     *  Product: '<S42>/Product'
     *  Selector: '<S16>/Selector3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S42>/Multiport Switch1' */

    /* Product: '<S42>/Product1' incorporates:
     *  Constant: '<S42>/Constant2'
     *  Lookup_n-D: '<S42>/1-D Lookup Table2'
     *  Selector: '<S16>/Selector3'
     *  Selector: '<S42>/Selector2'
     *  UnaryMinus: '<S42>/Unary Minus'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1 =
        look1_iflf_binlxpw(
            -LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S50>/Less Than'
     *  Selector: '<S16>/Selector3'
     *  Sum: '<S42>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1 + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].TimeGap) {
        /* Switch: '<S50>/Switch' incorporates:
         *  Constant: '<S50>/Constant'
         */
        LCCRA_bRightRearTGSafe = true;
    } else {
        /* Switch: '<S50>/Switch' incorporates:
         *  RelationalOperator: '<S50>/Less Than1'
         *  UnitDelay: '<S50>/Unit Delay'
         */
        LCCRA_bRightRearTGSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].TimeGap >=
              rtb_Product1) &&
             (LCCRA_bLastRightRearTGSafe));
    }

    /* End of Switch: '<S50>/Switch' */

    /* Switch: '<S49>/Switch' incorporates:
     *  RelationalOperator: '<S49>/Less Than'
     *  Selector: '<S16>/Selector3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (rtb_MultiportSwitch2 <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].RangeMagnitude) {
        /* Switch: '<S49>/Switch' incorporates:
         *  Constant: '<S49>/Constant'
         */
        LCCRA_bRightRearRDSafe = true;
    } else {
        /* Switch: '<S49>/Switch' incorporates:
         *  Constant: '<S1>/Constant22'
         *  RelationalOperator: '<S49>/Less Than1'
         *  UnitDelay: '<S49>/Unit Delay'
         */
        LCCRA_bRightRearRDSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].RangeMagnitude >=
              LCCRA_RiskDistance_C_met) &&
             (LCCRA_bLastRightRearRDSafe));
    }

    /* End of Switch: '<S49>/Switch' */

    /* Logic: '<S16>/AND1' */
    rtb_bLeftRearSafe = ((LCCRA_bRightRearTTCSafe && LCCRA_bRightRearTGSafe) &&
                         LCCRA_bRightRearRDSafe);

    /* MultiPortSwitch: '<S40>/Multiport Switch2' incorporates:
     *  Constant: '<S1>/Constant'
     *  Constant: '<S1>/Constant33'
     *  Selector: '<S16>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThreshold_C_sec;
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThresholdBus_C_sec;
            break;

        default:
            rtb_MultiportSwitch1_jddt = LCCRA_FrontTTCThreshold_C_sec;
            break;
    }

    /* End of MultiPortSwitch: '<S40>/Multiport Switch2' */

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S1>/Constant26'
     *  RelationalOperator: '<S48>/Less Than'
     *  Selector: '<S16>/Selector1'
     *  Sum: '<S40>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_MultiportSwitch1_jddt + LCCRA_TTCThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].TTC) {
        /* Switch: '<S48>/Switch' incorporates:
         *  Constant: '<S48>/Constant'
         */
        LCCRA_bRightFrontTTCSafe = true;
    } else {
        /* Switch: '<S48>/Switch' incorporates:
         *  RelationalOperator: '<S48>/Less Than1'
         *  UnitDelay: '<S48>/Unit Delay'
         */
        LCCRA_bRightFrontTTCSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].TTC >=
              rtb_MultiportSwitch1_jddt) &&
             (LCCRA_bLastRightFrontTTCSafe));
    }

    /* End of Switch: '<S48>/Switch' */

    /* MultiPortSwitch: '<S39>/Multiport Switch1' incorporates:
     *  Constant: '<S39>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S39>/1-D Lookup Table1'
     *  Lookup_n-D: '<S39>/1-D Lookup Table3'
     *  Product: '<S39>/Product'
     *  Selector: '<S16>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S39>/Multiport Switch1' */

    /* Product: '<S39>/Product1' incorporates:
     *  Constant: '<S39>/Constant2'
     *  Lookup_n-D: '<S39>/1-D Lookup Table2'
     *  Selector: '<S16>/Selector1'
     *  Selector: '<S39>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1_aqbo =
        look1_iflf_binlxpw(
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S47>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S47>/Less Than'
     *  Selector: '<S16>/Selector1'
     *  Sum: '<S39>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1_aqbo + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].TimeGap) {
        /* Switch: '<S47>/Switch' incorporates:
         *  Constant: '<S47>/Constant'
         */
        LCCRA_bRightFrontTGSafe = true;
    } else {
        /* Switch: '<S47>/Switch' incorporates:
         *  RelationalOperator: '<S47>/Less Than1'
         *  UnitDelay: '<S47>/Unit Delay'
         */
        LCCRA_bRightFrontTGSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].TimeGap >=
              rtb_Product1_aqbo) &&
             (LCCRA_bLastRightFrontTGSafe));
    }

    /* End of Switch: '<S47>/Switch' */

    /* Switch: '<S46>/Switch' incorporates:
     *  RelationalOperator: '<S46>/Less Than'
     *  Selector: '<S16>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (rtb_MultiportSwitch2 <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].RangeMagnitude) {
        /* Switch: '<S46>/Switch' incorporates:
         *  Constant: '<S46>/Constant'
         */
        LCCRA_bRightFrontRDSafe = true;
    } else {
        /* Switch: '<S46>/Switch' incorporates:
         *  Constant: '<S1>/Constant22'
         *  RelationalOperator: '<S46>/Less Than1'
         *  UnitDelay: '<S46>/Unit Delay'
         */
        LCCRA_bRightFrontRDSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].RangeMagnitude >=
              LCCRA_RiskDistance_C_met) &&
             (LCCRA_bLastRightFrontRDSafe));
    }

    /* End of Switch: '<S46>/Switch' */

    /* Logic: '<S16>/AND' */
    rtb_bLeftFrontSafe =
        ((LCCRA_bRightFrontTTCSafe && LCCRA_bRightFrontTGSafe) &&
         LCCRA_bRightFrontRDSafe);

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S36>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S36>/1-D Lookup Table1'
     *  Lookup_n-D: '<S36>/1-D Lookup Table3'
     *  Product: '<S36>/Product'
     *  Selector: '<S16>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_FrontTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S36>/Multiport Switch1' */

    /* Product: '<S36>/Product1' incorporates:
     *  Constant: '<S36>/Constant2'
     *  Lookup_n-D: '<S36>/1-D Lookup Table2'
     *  Selector: '<S16>/Selector2'
     *  Selector: '<S36>/Selector2'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_MultiportSwitch2 =
        look1_iflf_binlxpw(
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S44>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S44>/Less Than'
     *  Selector: '<S16>/Selector2'
     *  Sum: '<S36>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_MultiportSwitch2 + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].TimeGap) {
        /* Switch: '<S44>/Switch' incorporates:
         *  Constant: '<S44>/Constant'
         */
        LCCRA_bNextRightFrontSafe = true;
    } else {
        /* Switch: '<S44>/Switch' incorporates:
         *  RelationalOperator: '<S44>/Less Than1'
         *  UnitDelay: '<S44>/Unit Delay'
         */
        LCCRA_bNextRightFrontSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].TimeGap >=
              rtb_MultiportSwitch2) &&
             (LCCRA_bLastNextRightFrontSafe));
    }

    /* End of Switch: '<S44>/Switch' */

    /* Switch: '<S36>/Switch' incorporates:
     *  Constant: '<S1>/Constant6'
     *  Constant: '<S36>/Constant1'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport31'
     *  Product: '<S36>/Product3'
     *  RelationalOperator: '<S36>/GreaterThan'
     *  Selector: '<S16>/Selector2'
     *  Selector: '<S36>/Selector1'
     *  Sum: '<S36>/Subtract'
     *  Trigonometry: '<S36>/Sin'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (LCCRA_bNextRightFrontSafe) {
        rtb_NextLeftFrontSafeFlag = true;
    } else {
        rtb_NextLeftFrontSafeFlag =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].Velocity[1] -
              (VED_EgoVelocity_mps * sinf(LBP_LeLnClthHeading_rad))) <
             LCCRA_VelocityThreshold_C_mps);
    }

    /* End of Switch: '<S36>/Switch' */

    /* MultiPortSwitch: '<S37>/Multiport Switch1' incorporates:
     *  Constant: '<S37>/Constant36'
     *  Inport: '<Root>/Inport31'
     *  Lookup_n-D: '<S37>/1-D Lookup Table4'
     *  Lookup_n-D: '<S37>/1-D Lookup Table5'
     *  Product: '<S37>/Product'
     *  Selector: '<S16>/Selector4'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].Type) {
        case 0:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;

        case 1:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGapBus_Mp_sec[0])), 11U);
            break;

        default:
            rtb_MultiportSwitch1_jddt = look1_iflf_binlxpw(
                VED_EgoVelocity_mps * 3.6F,
                ((const real32_T *)&(LCCRA_VehVelXTimeGap_Bx_kph[0])),
                ((const real32_T *)&(LCCRA_RearTimeGap_Mp_sec[0])), 11U);
            break;
    }

    /* End of MultiPortSwitch: '<S37>/Multiport Switch1' */

    /* Product: '<S37>/Product1' incorporates:
     *  Constant: '<S37>/Constant2'
     *  Lookup_n-D: '<S37>/1-D Lookup Table2'
     *  Selector: '<S16>/Selector4'
     *  Selector: '<S37>/Selector2'
     *  UnaryMinus: '<S37>/Unary Minus1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    rtb_Product1_kv1k =
        look1_iflf_binlxpw(
            -LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].Velocity[0],
            ((const real32_T *)&(LCCRA_FrontObjVelX_Bx_kph[0])),
            ((const real32_T *)&(LCCRA_TmGpVelXFac_Mp_nu[0])), 9U) *
        rtb_MultiportSwitch1_jddt;

    /* Switch: '<S45>/Switch' incorporates:
     *  Constant: '<S1>/Constant27'
     *  RelationalOperator: '<S45>/Less Than'
     *  Selector: '<S16>/Selector4'
     *  Sum: '<S37>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_Product1_kv1k + LCCRA_TimeGapThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].TimeGap) {
        /* Switch: '<S45>/Switch' incorporates:
         *  Constant: '<S45>/Constant'
         */
        LCCRA_bNextRightRearSafe = true;
    } else {
        /* Switch: '<S45>/Switch' incorporates:
         *  RelationalOperator: '<S45>/Less Than1'
         *  UnitDelay: '<S45>/Unit Delay'
         */
        LCCRA_bNextRightRearSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].TimeGap >=
              rtb_Product1_kv1k) &&
             (LCCRA_bLastNextRightRearSafe));
    }

    /* End of Switch: '<S45>/Switch' */

    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S1>/Constant6'
     *  Constant: '<S37>/Constant1'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport31'
     *  Product: '<S37>/Product3'
     *  RelationalOperator: '<S37>/GreaterThan'
     *  Selector: '<S16>/Selector4'
     *  Selector: '<S37>/Selector1'
     *  Sum: '<S37>/Subtract'
     *  Trigonometry: '<S37>/Sin'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if (LCCRA_bNextRightRearSafe) {
        rtb_NextLeftRearSafeFlag = true;
    } else {
        rtb_NextLeftRearSafeFlag =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].Velocity[1] -
              (VED_EgoVelocity_mps * sinf(LBP_LeLnClthHeading_rad))) <
             LCCRA_VelocityThreshold_C_mps);
    }

    /* End of Switch: '<S37>/Switch' */

    /* MATLAB Function: '<S16>/MATLAB Function' incorporates:
     *  Selector: '<S16>/Selector1'
     *  Selector: '<S16>/Selector2'
     *  Selector: '<S16>/Selector3'
     *  Selector: '<S16>/Selector4'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    LCCRA_RightDangerObjID_nu = -1;
    rtb_MultiportSwitch1_jddt = 50000.0F;
    if (((rtb_bLeftRearSafe && rtb_bLeftFrontSafe) &&
         rtb_NextLeftFrontSafeFlag) &&
        rtb_NextLeftRearSafeFlag) {
        LCCRA_RightSafeFlag_bool = true;
        LCCRA_RightDangerObjID_nu = -1;
    } else {
        LCCRA_RightSafeFlag_bool = false;
        if (!rtb_bLeftFrontSafe) {
            LCCRA_RightDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].ID;
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[4].RangeMagnitude;
        }

        if ((!rtb_bLeftRearSafe) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_RightDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].ID;
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[5].RangeMagnitude;
        }

        if ((!rtb_NextLeftFrontSafeFlag) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_RightDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].ID;
            rtb_MultiportSwitch1_jddt =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[8].RangeMagnitude;
        }

        if ((!rtb_NextLeftRearSafeFlag) &&
            (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].RangeMagnitude <
             rtb_MultiportSwitch1_jddt)) {
            LCCRA_RightDangerObjID_nu =
                LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[9].ID;

            /*          fMinRangeMagnitude = NextRightRear.RangeMagnitude; */
        }
    }

    /* End of MATLAB Function: '<S16>/MATLAB Function' */

    /* Logic: '<S16>/AND2' */
    LCCRA_RightFrontSafeFlag_bool =
        (rtb_bLeftFrontSafe && rtb_NextLeftFrontSafeFlag);

    /* Logic: '<S16>/AND3' */
    LCCRA_RightRearSafeFlag_bool =
        (rtb_bLeftRearSafe && rtb_NextLeftRearSafeFlag);

    /* Product: '<S37>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_NextRightRearSfDistTmGp_met = rtb_Product1_kv1k * VED_EgoVelocity_mps;

    /* Product: '<S36>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_NextRightFrontSfDistTmGp_met =
        rtb_MultiportSwitch2 * VED_EgoVelocity_mps;

    /* Product: '<S39>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_RightFrontSafeDistTmGp_met = rtb_Product1_aqbo * VED_EgoVelocity_mps;

    /* Product: '<S42>/Product2' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    LCCRA_RightRearSafeDistTmGp_met = rtb_Product1 * VED_EgoVelocity_mps;

    /* MultiPortSwitch: '<S14>/Multiport Switch1' incorporates:
     *  Constant: '<S14>/Constant1'
     *  Constant: '<S1>/Constant31'
     *  Constant: '<S1>/Constant35'
     *  Selector: '<S14>/Selector1'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    switch (LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].Type) {
        case 0:
            rtb_MultiportSwitch2 = LCCRA_EgoFrontTTCThreshold_C_sec;
            break;

        case 1:
            rtb_MultiportSwitch2 = LCCRA_EgoFrontTTCThresholdBus_C_sec;
            break;

        default:
            rtb_MultiportSwitch2 = LCCRA_EgoFrontTTCThreshold_C_sec;
            break;
    }

    /* End of MultiPortSwitch: '<S14>/Multiport Switch1' */

    /* Switch: '<S17>/Switch' incorporates:
     *  Constant: '<S14>/Constant1'
     *  Constant: '<S1>/Constant26'
     *  RelationalOperator: '<S17>/Less Than'
     *  Selector: '<S14>/Selector1'
     *  Sum: '<S14>/Add'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    if ((rtb_MultiportSwitch2 + LCCRA_TTCThresholdHys_C_sec) <
        LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].TTC) {
        /* Switch: '<S17>/Switch' incorporates:
         *  Constant: '<S17>/Constant'
         */
        LCCRA_bFrontTTCSafe = true;
    } else {
        /* Switch: '<S17>/Switch' incorporates:
         *  RelationalOperator: '<S17>/Less Than1'
         *  UnitDelay: '<S17>/Unit Delay'
         */
        LCCRA_bFrontTTCSafe =
            ((LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].TTC >=
              rtb_MultiportSwitch2) &&
             (LCCRA_bLastFrontTTCSafe));
    }

    /* End of Switch: '<S17>/Switch' */

    /* Switch: '<S14>/Switch' */
    LCCRA_FrontSafeFlag_bool = LCCRA_bFrontTTCSafe;

    /* Switch: '<S14>/Switch1' */
    if (LCCRA_bFrontTTCSafe) {
        /* Switch: '<S14>/Switch1' incorporates:
         *  Constant: '<S14>/Constant4'
         */
        LCCRA_FrontDangerObjID_nu = -1;
    } else {
        /* Switch: '<S14>/Switch1' incorporates:
         *  Constant: '<S14>/Constant1'
         *  Selector: '<S14>/Selector1'
         *  UnitDelay: '<S1>/Unit Delay1'
         */
        LCCRA_FrontDangerObjID_nu =
            LCCRA_DW.LCCRA_MIOUnitDelay_str.MIOObjects[0].ID;
    }

    /* End of Switch: '<S14>/Switch1' */

    /* Update for UnitDelay: '<S1>/Unit Delay2' */
    LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st = LCCRA_TargetVehicle_str;

    /* Update for UnitDelay: '<S32>/Unit Delay' */
    LCCRA_bLastLeftRearTTCSafe = LCCRA_bLeftRearTTCSafe;

    /* Update for UnitDelay: '<S31>/Unit Delay' */
    LCCRA_bLastLeftRearTGSafe = LCCRA_bLeftRearTGSafe;

    /* Update for UnitDelay: '<S30>/Unit Delay' */
    LCCRA_bLastLeftRearRDSafe = LCCRA_bLeftRearRDSafe;

    /* Update for UnitDelay: '<S29>/Unit Delay' */
    LCCRA_bLastLeftFrontTTCSafe = LCCRA_bLeftFrontTTCSafe;

    /* Update for UnitDelay: '<S28>/Unit Delay' */
    LCCRA_bLastLeftFrontTGSafe = LCCRA_bLeftFrontTGSafe;

    /* Update for UnitDelay: '<S27>/Unit Delay' */
    LCCRA_bLastLeftFrontRDSafe = LCCRA_bLeftFrontRDSafe;

    /* Update for UnitDelay: '<S33>/Unit Delay' */
    LCCRA_bLastNextLeftFrontSafe = LCCRA_bNextLeftFrontSafe;

    /* Update for UnitDelay: '<S34>/Unit Delay' */
    LCCRA_bLastNextLeftRearSafe = LCCRA_bNextLeftRearSafe;

    /* Update for UnitDelay: '<S51>/Unit Delay' */
    LCCRA_bLastRightRearTTCSafe = LCCRA_bRightRearTTCSafe;

    /* Update for UnitDelay: '<S50>/Unit Delay' */
    LCCRA_bLastRightRearTGSafe = LCCRA_bRightRearTGSafe;

    /* Update for UnitDelay: '<S49>/Unit Delay' */
    LCCRA_bLastRightRearRDSafe = LCCRA_bRightRearRDSafe;

    /* Update for UnitDelay: '<S48>/Unit Delay' */
    LCCRA_bLastRightFrontTTCSafe = LCCRA_bRightFrontTTCSafe;

    /* Update for UnitDelay: '<S47>/Unit Delay' */
    LCCRA_bLastRightFrontTGSafe = LCCRA_bRightFrontTGSafe;

    /* Update for UnitDelay: '<S46>/Unit Delay' */
    LCCRA_bLastRightFrontRDSafe = LCCRA_bRightFrontRDSafe;

    /* Update for UnitDelay: '<S44>/Unit Delay' */
    LCCRA_bLastNextRightFrontSafe = LCCRA_bNextRightFrontSafe;

    /* Update for UnitDelay: '<S45>/Unit Delay' */
    LCCRA_bLastNextRightRearSafe = LCCRA_bNextRightRearSafe;

    /* Update for UnitDelay: '<S17>/Unit Delay' */
    LCCRA_bLastFrontTTCSafe = LCCRA_bFrontTTCSafe;
}

/* Model initialize function */
void LCCRA_initialize(void) {
    /* Registration code */

    /* block I/O */

    /* exported global signals */
    LCCRA_FrontDangerObjID_nu = 0;
    LCCRA_RightDangerObjID_nu = 0;
    LCCRA_LeftDangerObjID_nu = 0;
    LCCRA_bLeftRearTTCSafe = false;
    LCCRA_bLeftRearTGSafe = false;
    LCCRA_bLeftRearRDSafe = false;
    LCCRA_bLeftFrontTTCSafe = false;
    LCCRA_bLeftFrontTGSafe = false;
    LCCRA_bLeftFrontRDSafe = false;
    LCCRA_bNextLeftFrontSafe = false;
    LCCRA_bNextLeftRearSafe = false;
    LCCRA_LeftFrontSafeFlag_bool = false;
    LCCRA_LeftRearSafeFlag_bool = false;
    LCCRA_bRightRearTTCSafe = false;
    LCCRA_bRightRearTGSafe = false;
    LCCRA_bRightRearRDSafe = false;
    LCCRA_bRightFrontTTCSafe = false;
    LCCRA_bRightFrontTGSafe = false;
    LCCRA_bRightFrontRDSafe = false;
    LCCRA_bNextRightFrontSafe = false;
    LCCRA_bNextRightRearSafe = false;
    LCCRA_RightFrontSafeFlag_bool = false;
    LCCRA_RightRearSafeFlag_bool = false;
    LCCRA_bFrontTTCSafe = false;
    LCCRA_FrontSafeFlag_bool = false;
    LCCRA_RightSafeFlag_bool = false;
    LCCRA_LeftSafeFlag_bool = false;

    /* custom signals */
    LCCRA_DebugMIOs_str = LCCRA_rtZBusDebugMIOs;
    LCCRA_TargetVehicle_str = LCCRA_rtZBusVehicle;
    LCCRA_DebugDelaytime_str = LCCRA_rtZBusDelayTime;
    LCCRA_DebugWeight_str = LCCRA_rtZBusWeight;
    LCCRA_NextLeftRearSfDistTmGp_met = 0.0F;
    LCCRA_NextLeftFrontSfDistTmGp_met = 0.0F;
    LCCRA_LeftFrontSafeDistTmGp_met = 0.0F;
    LCCRA_LeftRearSafeDistTmGp_met = 0.0F;
    LCCRA_NextRightRearSfDistTmGp_met = 0.0F;
    LCCRA_NextRightFrontSfDistTmGp_met = 0.0F;
    LCCRA_RightFrontSafeDistTmGp_met = 0.0F;
    LCCRA_RightRearSafeDistTmGp_met = 0.0F;

    /* states (dwork) */
    (void)memset((void *)&LCCRA_DW, 0, sizeof(DW_LCCRA_T));

    /* custom states */
    LCCRA_bLastLeftRearTTCSafe = false;
    LCCRA_bLastLeftRearTGSafe = false;
    LCCRA_bLastLeftRearRDSafe = false;
    LCCRA_bLastLeftFrontTTCSafe = false;
    LCCRA_bLastLeftFrontTGSafe = false;
    LCCRA_bLastLeftFrontRDSafe = false;
    LCCRA_bLastNextLeftFrontSafe = false;
    LCCRA_bLastNextLeftRearSafe = false;
    LCCRA_bLastRightRearTTCSafe = false;
    LCCRA_bLastRightRearTGSafe = false;
    LCCRA_bLastRightRearRDSafe = false;
    LCCRA_bLastRightFrontTTCSafe = false;
    LCCRA_bLastRightFrontTGSafe = false;
    LCCRA_bLastRightFrontRDSafe = false;
    LCCRA_bLastNextRightFrontSafe = false;
    LCCRA_bLastNextRightRearSafe = false;
    LCCRA_bLastFrontTTCSafe = false;

    /* InitializeConditions for UnitDelay: '<S1>/Unit Delay2' */
    LCCRA_DW.LCCRA_TargetVehicleUnitDelay_st =
        LCCRA_ConstP.UnitDelay2_InitialCondition;

    /* InitializeConditions for UnitDelay: '<S1>/Unit Delay1' */
    LCCRA_DW.LCCRA_MIOUnitDelay_str = LCCRA_ConstP.UnitDelay1_InitialCondition;

    /* SystemInitialize for MATLAB Function: '<S1>/LCCRA_FindMIOs' */
    LCCRA_DW.LCCRA_egofrontweight_nu = 0U;
    LCCRA_DW.LCCRA_egorearweight_nu = 0U;
    LCCRA_DW.LCCRA_leftfrontweight_nu = 0U;
    LCCRA_DW.LCCRA_leftrearweight_nu = 0U;
    LCCRA_DW.LCCRA_rightfrontweight_nu = 0U;
    LCCRA_DW.LCCRA_rightrearweight_nu = 0U;
    LCCRA_DW.LCCRA_nextleftfrontweight_nu = 0U;
    LCCRA_DW.LCCRA_nextleftrearweight_nu = 0U;
    LCCRA_DW.LCCRA_nextrightfrontweight_nu = 0U;
    LCCRA_DW.LCCRA_nextrightrearweight_nu = 0U;
    LCCRA_DW.LCCRA_egofrontdelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_egoreardelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_leftfrontdelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_leftreardelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_rightfrontdelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_rightreardelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_nextleftfrontdelaytime_se = 0.0F;
    LCCRA_DW.LCCRA_nextleftreardelaytime_sec = 0.0F;
    LCCRA_DW.LCCRA_nextrightfrontdelaytime_s = 0.0F;
    LCCRA_DW.LCCRA_nextrightreardelaytime_se = 0.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
