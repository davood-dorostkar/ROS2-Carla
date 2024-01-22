/**********************************Model Property********************************
 *
 * Company             : PHIGENT
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : TJATCT
 *
 * Model Long Name     : Trajectoty control

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_01

 *

 * Model Author        :

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 20ms


 ************************************Auto Coder**********************************
 *
 * File                             : TJATCT.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Tue May  9 16:33:31 2023
 *
 * Copyright (C) by PhiGent Group Limited. All rights reserved.
 *******************************************************************************/

#include "TJATCT.h"
#include "TJATCT_private.h"
#include "look1_iflf_binlxpw.h"
#include "look1_iflf_binlxpw.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile uint8_T DTE_DelayCyclesLaDMC_P = 0U;/* Referenced by:
                                                    * '<S238>/Parameter1'
                                                    * '<S266>/Parameter1'
                                                    */

/* Number of cycle delays of the LaDMC */
const volatile real32_T DTE_KappaAngleLaDmc_M[12] = { 155.38F, 155.38F, 155.38F,
  155.38F, 155.38F, 155.38F, 155.38F, 156.25F, 187.5F, 300.0F, 325.22F, 421.0F }
;                                 /* Referenced by: '<S229>/1-D Lookup Table' */

/* Grid points of the y-Axis of the LaDMC Look Up Table "Kappa To Angle". */
const volatile real32_T DTE_KappaAngleLaDmc_X[12] = { 0.0F, 10.0F, 30.0F, 60.0F,
  80.0F, 100.0F, 110.0F, 120.0F, 130.0F, 140.0F, 150.0F, 250.0F } ;
                                  /* Referenced by: '<S229>/1-D Lookup Table' */

/* Grid points for the x-Axis of the LaDMC Look Up Table "Kappa To Angle" */
const volatile real32_T DTE_SetCrvGainLaDmc_M[13] = { 0.0F, 0.3F, 0.6F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S238>/1-D Lookup Table' */

/* Gain of the calculated curvature needed to compensate the disturbances in dependence on the vehicle's velocity */
const volatile real32_T DTE_SetCrvGainLaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S238>/1-D Lookup Table' */

/* X axis for gain of the calculated curvature needed to compensate the disturbances in dependence on the vehicle's velocity */
const volatile real32_T DTE_SetDeltaGainLaDmc_M[13] = { 0.0F, 0.2F, 0.4F, 0.6F,
  0.6F, 0.6F, 0.6F, 0.6F, 0.6F, 0.6F, 0.6F, 0.6F, 0.6F } ;
                                  /* Referenced by: '<S266>/1-D Lookup Table' */

/* Gain of the Disturbance Compensator set value */
const volatile real32_T DTE_SetDeltaGainLaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S266>/1-D Lookup Table' */

/* X axis for gain of the Disturbance Compensator set value */
const volatile real32_T DTE_Time1FltLaDmc_P = 0.2F;/* Referenced by:
                                                    * '<S224>/Parameter2'
                                                    * '<S224>/Parameter4'
                                                    * '<S253>/Parameter4'
                                                    * '<S253>/Parameter6'
                                                    * '<S253>/Parameter7'
                                                    * '<S253>/Parameter8'
                                                    */

/* First Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
const volatile real32_T DTE_Time1LaDmc_P = 0.3F;/* Referenced by:
                                                 * '<S226>/Parameter6'
                                                 * '<S226>/Parameter8'
                                                 */

/* First Time Constant of the DMC's transfer function (command steering angle -> driven curvature) */
const volatile real32_T DTE_Time2FltLaDmc_P = 0.4F;/* Referenced by:
                                                    * '<S224>/Parameter3'
                                                    * '<S224>/Parameter5'
                                                    * '<S253>/Parameter1'
                                                    * '<S253>/Parameter2'
                                                    * '<S253>/Parameter3'
                                                    * '<S253>/Parameter5'
                                                    */

/* Second Time Constant of the second order low pass needed for the inversion of the DMC's transfer function */
const volatile real32_T DTE_Time2LaDmc_P = 0.1F;/* Referenced by:
                                                 * '<S226>/Parameter7'
                                                 * '<S226>/Parameter9'
                                                 */

/* Second Time Constant of the DMC's transfer function (command steering angle -> driven curvature) */
const volatile uint8_T LGC_CswDynPT1Rst_P = 0U;/* Referenced by: '<S434>/Parameter5' */
const volatile uint8_T P_TCTLGC_ActivateDynBacGain_nu = 0U;/* Referenced by:
                                                            * '<S565>/Constant'
                                                            * '<S583>/Constant'
                                                            */
const volatile uint8_T P_TCTLGC_ActivateOverride_nu = 0U;/* Referenced by: '<S419>/Constant2' */
const volatile uint8_T P_TCTLGC_CssCrv_nu = 2U;/* Referenced by:
                                                * '<S419>/Constant'
                                                * '<S419>/Constant3'
                                                */
const volatile real32_T P_VEH_DistCogToFrontAxle_m = 0.0F;/* Referenced by: '<S564>/Constant1' */
const volatile real32_T P_VEH_DistCogToRearAxle_m = 0.0F;/* Referenced by: '<S564>/Constant2' */
const volatile real32_T P_VEH_SelfSteeringGrd_nu = 0.0F;/* Referenced by: '<S564>/Constant4' */
const volatile real32_T X_TCTLGC_CtrlErrDistY_met[6] = { -2.0F, -0.15F, -0.1F,
  -0.05F, 0.0F, 2.0F } ;               /* Referenced by:
                                        * '<S565>/Y_TCTLGC_DynBacGain_nu'
                                        * '<S583>/Y_TCTLGC_DynBacGain_nu'
                                        */

const volatile real32_T Y_TCTLGC_DynBacGain_nu[6] = { 0.0F, 0.0F, 0.8F, 1.0F,
  1.0F, 1.0F } ;                       /* Referenced by:
                                        * '<S565>/Y_TCTLGC_DynBacGain_nu'
                                        * '<S583>/Y_TCTLGC_DynBacGain_nu'
                                        */

/* Definition for custom storage class: Global */
real_T LQR_EnaButterVel = 180.0;       /* Referenced by: '<S603>/Parameter8' */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile real32_T CDC_CoeffDeltaPsiKmc_P = 1.0F;/* Referenced by: '<S92>/Constant' */

/* UNDEFINED */
const volatile real32_T CDC_CoeffDeltaPsiObsKmc_P = 0.0F;
                       /* Referenced by: '<S92>/Kmc_delta_psi_obs_gain_const' */

/* UNDEFINED */
const volatile uint8_T CDC_CswSelDeltaTheta_P = 1U;/* Referenced by: '<S91>/Parameter' */

/* Output Source Selection of DeltaTheta
   ------------------------------------------
   0: Raw unfiltered
   1: PT1 Filter
   2: Plant Observer
   3: Old Course Angle Observer */
const volatile uint8_T CDC_CswSelDistY_P = 2U;
                              /* Referenced by: '<S87>/P_TCTCDC_OssDeltaY_nu' */

/* Output Source Selection of DeltaY
   ------------------------------------------
   1: Raw unfiltered
   2: PT1 Filter
   3: Plant Observer */
const volatile uint8_T CDC_CswTrajCrv_P = 0U;/* Referenced by: '<S92>/Constant4' */

/* UNDEFINED */
const volatile uint8_T CDC_CswWatchdogAct = 0U;
                           /* Referenced by: '<S86>/P_TCTCDC_WtchdgActive_nu' */

/* Configuration switch for watchdog activation
   0: Watchdog is deactivated
   1: Watchdog is activated */
const volatile uint8_T CDC_DelayCycleNum_P = 0U;/* Referenced by: '<S92>/Constant1' */

/* Cylce numer for the delay in the heading angle signal */
const volatile uint8_T CDC_EnaFreezeByTgq_P = 3U;/* Referenced by:
                                                  * '<S85>/Parameter1'
                                                  * '<S85>/Parameter4'
                                                  */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T CDC_EnaOffByTgq_P = 0U;/* Referenced by: '<S85>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T CDC_FltDistYFc = 1.0F;
                             /* Referenced by: '<S88>/P_TCTCDC_OssDeltaY_nu2' */
const volatile real32_T CDC_FltYawFc = 1.0F;
        /* Referenced by: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec1' */
const volatile real32_T CDC_HeadingOffset = 0.0F;
        /* Referenced by: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec2' */
const volatile uint8_T CDC_StLcfOff_P = 0U;
                              /* Referenced by: '<S86>/from_data_definition1' */

/* Lcf off state parameter */
const volatile real32_T CDC_ThdErrCourseAngle_P = 0.0F;
                      /* Referenced by: '<S86>/P_TCTCDC_WtchdgLimCoAnErr_rad' */

/* Absolute value of the threshold used to classify the Course Angle Error as too high */
const volatile real32_T CDC_ThdErrDistY_P = 0.0F;
                           /* Referenced by: '<S86>/P_TCTCDC_WtchdgLimYErr_m' */

/* Absolute value of the threshold used to classify the Y-Coordinate Error as too high */
const volatile real32_T CDC_TimeSysCycle_P = 0.02F;/* Referenced by:
                                                    * '<S88>/P_TCTCDC_OssDeltaY_nu1'
                                                    * '<S92>/Parameter'
                                                    * '<S93>/Parameter'
                                                    */
const volatile uint8_T CLM_BtfQualifierCdc_P = 0U;/* Referenced by: '<S106>/Parameter' */

/* Bitmask for selecting the bits to be checked in the signal S_TCTCDC_QualifierService_nu */
const volatile uint8_T CLM_BtfQualifierClm_P = 0U;/* Referenced by: '<S106>/Parameter1' */

/* Bitmask for selecting the bits to be checked in the signal D_TCTCLM_QualifierService_nu */
const volatile uint8_T CLM_CswForceDegrReq_P = 0U;/* Referenced by: '<S106>/Parameter2' */

/* Force degradation request via signal S_TCTCLM_QuServTrajCtr
   0: Use conventional degradation request conditions
   1: Force degradation request */
const volatile uint8_T CLM_CswHldDegrReq_P = 0U;/* Referenced by: '<S106>/Parameter6' */

/* Activate/Deactivate hold functionality of degradation request
   0: Deactivate
   1: Activate */
const volatile uint8_T CLM_CswSelFbcCrv_P = 1U;/* Referenced by:
                                                * '<S134>/Parameter'
                                                * '<S134>/Parameter1'
                                                */

/* Select interface for the feedback controller
   0: Steer Angle Interface
   1: Curvature Interface */
const volatile real32_T CLM_DistCogToFrontAxle_P = 1.087F;/* Referenced by: '<S134>/Parameter3' */

/* Distance between front axle and the vehicle's center of gravity */
const volatile real32_T CLM_DistCogToRearAxle_P = 1.553F;/* Referenced by: '<S134>/Parameter4' */

/* Distance between rear axle and the vehicle's center of gravity */
const volatile real32_T CLM_MaxHldTiDegrReq_P = 0.0F;/* Referenced by: '<S106>/Parameter8' */

/* Maximum hold time of degradation request */
const volatile real32_T CLM_MinHldTiDegrReq_P = 0.0F;/* Referenced by: '<S106>/Parameter4' */

/* Minimum hold time of degradation request */
const volatile real32_T CLM_SelfSteerGrd_P = 0.0F;/* Referenced by: '<S134>/Parameter2' */

/* Self Steering Gradient of the Vehicle */
const volatile uint8_T CLM_StControlCsc_P = 4U;/* Referenced by:
                                                * '<S132>/Parameter1'
                                                * '<S106>/Parameter3'
                                                */

/* Cycle Time for VEH task */
const volatile uint8_T CLM_StDepartLeftLaKmc_P = 3U;/* Referenced by: '<S136>/Parameter' */

/* Cycle Time for VEH task */
const volatile uint8_T CLM_StDepartRightLaKmc_P = 3U;/* Referenced by: '<S136>/Parameter1' */

/* Cycle Time for VEH task */
const volatile uint8_T CLM_StRequestCsc_P = 3U;/* Referenced by: '<S132>/Parameter' */

/* Cycle Time for VEH task */
const volatile real32_T CLM_ThdDeltaFCmdGrd_LDP_M[10] = { 5.29F, 4.41F, 3.53F,
  2.9F, 2.06F, 1.47F, 1.18F, 0.88F, 0.88F, 0.88F } ;
                                 /* Referenced by: '<S115>/1-D Lookup Table1' */

/* Absolut value of the gradient limitation of the set value send to the LaDMC */
const volatile real32_T CLM_ThdDeltaFCmdGrd_M[10] = { 5.29F, 4.41F, 3.53F, 2.94F,
  1.8F, 1.18F, 0.88F, 0.76F, 0.76F, 0.76F } ;
                                  /* Referenced by: '<S115>/1-D Lookup Table' */

/* Absolut value of the gradient limitation of the set value send to the LaDMC */
const volatile real32_T CLM_ThdDeltaFCmdGrd_X[10] = { 0.0F, 20.0F, 30.0F, 40.0F,
  50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 140.0F } ;/* Referenced by:
                                                  * '<S115>/1-D Lookup Table'
                                                  * '<S115>/1-D Lookup Table1'
                                                  */

/* Velocity vector along the vehicle's longitudinal axis / kph */
const volatile real32_T CLM_ThdDeltaFCmdSat_LDP_M[10] = { 5.29F, 4.1F, 3.53F,
  2.9F, 2.35F, 1.76F, 1.18F, 0.76F, 0.76F, 0.76F } ;
                                 /* Referenced by: '<S116>/1-D Lookup Table1' */

/* Absolute value of the saturation of the set value send to the LaDMC */
const volatile real32_T CLM_ThdDeltaFCmdSat_M[10] = { 5.29F, 4.1F, 3.53F, 2.9F,
  2.35F, 1.76F, 1.18F, 0.76F, 0.76F, 0.76F } ;
                                 /* Referenced by: '<S116>/1-D Lookup Table2' */

/* Absolute value of the saturation of the set value send to the LaDMC */
const volatile real32_T CLM_ThdDeltaFCmdSat_X[10] = { 0.0F, 20.0F, 30.0F, 40.0F,
  50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 140.0F } ;/* Referenced by:
                                                  * '<S116>/1-D Lookup Table1'
                                                  * '<S116>/1-D Lookup Table2'
                                                  */

/* Velocity vector along the vehicle's longitudinal axis / kph */
const volatile real32_T CLM_ThdFbcDcSat_M[15] = { 0.01F, 0.01F, 0.01F, 0.01F,
  0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F } ;
                                 /* Referenced by: '<S121>/1-D Lookup Table2' */

/* Absolute value / (1/m) of the maximal allowed control signal "Feedback Controller + Disturbance Compensator" send to the LaDMC */
const volatile real32_T CLM_ThdFbcDcSat_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                 /* Referenced by: '<S121>/1-D Lookup Table2' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
const volatile real32_T CLM_ThdFfcCrvGrd_M[15] = { 0.5F, 0.5F, 0.5F, 0.5F, 0.5F,
  0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F } ;
                                  /* Referenced by: '<S125>/1-D Lookup Table' */

/* Absolute value / (1/(m*s)) of the maximal allowed gradient of the control signal Feedforward Controller" send to the LaDMC */
const volatile real32_T CLM_ThdFfcCrvGrd_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                  /* Referenced by: '<S125>/1-D Lookup Table' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
const volatile real32_T CLM_ThdFfcCrvSat_M[15] = { 0.01F, 0.01F, 0.01F, 0.01F,
  0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F, 0.01F } ;
                                 /* Referenced by: '<S126>/1-D Lookup Table2' */

/* Absolute value / (1/m) of the maximal allowed control signal "Feedforward Controller" send to the LaDMC */
const volatile real32_T CLM_ThdFfcCrvSat_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                 /* Referenced by: '<S126>/1-D Lookup Table2' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
const volatile real32_T CLM_ThdFfcFbcDcGrd_M[15] = { 0.5F, 0.5F, 0.5F, 0.5F,
  0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F } ;
                                  /* Referenced by: '<S120>/1-D Lookup Table' */

/* Absolute value / (1/(m*s)) of the maximal allowed gradient of the control signal "Feedback Controller + Disturbance Compensator" send to the LaDMC */
const volatile real32_T CLM_ThdFfcFbcDcGrd_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                  /* Referenced by: '<S120>/1-D Lookup Table' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
const volatile real32_T CLM_TimeSysCycle_P = 0.02F;/* Referenced by:
                                                    * '<S131>/Parameter'
                                                    * '<S131>/Parameter1'
                                                    * '<S106>/Parameter7'
                                                    * '<S106>/Parameter9'
                                                    * '<S115>/Parameter'
                                                    * '<S115>/Parameter1'
                                                    * '<S120>/Parameter'
                                                    * '<S120>/Parameter1'
                                                    * '<S125>/Parameter'
                                                    * '<S125>/Parameter1'
                                                    */

/* Cycle Time for VEH task */
const volatile real32_T DEV_CoeffCrvAmp_P = 0.0F;/* Referenced by: '<S194>/Parameter5' */

/* Amplitude (1/m) of the chirp signal used to stimulate the curvature interface of the LaKMC */
const volatile real32_T DEV_CoeffCrvFrqGain_P = 0.0F;/* Referenced by: '<S194>/Parameter3' */

/* Start frequency / (1/s) of the chirp signal used to stimulate curvature interface of LaKMC */
const volatile real32_T DEV_CoeffDeltaFAmp_P = 0.0F;/* Referenced by: '<S196>/Parameter5' */

/* Amplitude /deg of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
const volatile real32_T DEV_CoeffDeltaFFrqGain_P = 0.0F;/* Referenced by: '<S196>/Parameter3' */

/* Gain of the frequency increase of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
const volatile real32_T DEV_CoeffDeltaGainFfc_M[3] = { 0.0F, 0.0F, 0.0F } ;/* Referenced by:
                                                                      * '<S143>/1-D Lookup Table1'
                                                                      * '<S144>/Y_TCTFFC_GainFFC_nu'
                                                                      */

/* Gain of the control signal part of the feedforward controller */
const volatile real32_T DEV_CoeffDeltaGainFfc_X[3] = { 0.0F, 1.0F, 2.0F } ;/* Referenced by:
                                                                      * '<S143>/1-D Lookup Table1'
                                                                      * '<S144>/Y_TCTFFC_GainFFC_nu'
                                                                      */

/* X axis for gain of the control signal part of the feedforward controller */
const volatile uint8_T DEV_CswCrvTestSignal_P = 0U;/* Referenced by: '<S195>/Parameter' */

/* Mode of the TCTDEV Test Signal generator to stimulate curvature interface of LaKMC:
   0000 0000: Signal generator off
   0000 xxx1: Chirp active */
const volatile uint8_T DEV_CswDeltaFTestSignal_P = 0U;/* Referenced by: '<S197>/Parameter' */

/* Mode of the TCTDEV Test Signal generator to stimulate S_TCTLGC_DeltaFCmd_deg:
   0000 0000: Signal generator off
   0000 xxx1: Chirp active */
const volatile real32_T DEV_DistCogToFrontAxle_P = 0.8F;/* Referenced by:
                                                         * '<S141>/Constant1'
                                                         * '<S142>/Constant1'
                                                         */

/* Distance between front axle and the vehicle's center of gravity */
const volatile real32_T DEV_DistCogToRearAxle_P = 1.4F;/* Referenced by:
                                                        * '<S141>/Constant2'
                                                        * '<S142>/Constant2'
                                                        */

/* Distance between rear axle and the vehicle's center of gravity */
const volatile uint8_T DEV_EnaFreezeByTgq_P = 3U;/* Referenced by:
                                                  * '<S140>/Parameter1'
                                                  * '<S140>/Parameter4'
                                                  */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T DEV_EnaOffByTgq_P = 0U;/* Referenced by: '<S140>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T DEV_SelfSteeringGrd_nu = 0.0F;/* Referenced by: '<S142>/Constant3' */
const volatile real32_T DEV_StrtFrqCrvF_P = 0.0F;/* Referenced by: '<S194>/Parameter4' */

/* Gain of the frequency increase of the chirp signal used to stimulate the curvature interface of the LaKMC */
const volatile real32_T DEV_StrtFrqDeltaF_P = 0.0F;/* Referenced by: '<S196>/Parameter4' */

/* Start frequency (1/s) of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
const volatile real32_T DEV_Time1FltLaDmc_P = 0.025F;/* Referenced by:
                                                      * '<S156>/Parameter4'
                                                      * '<S156>/Parameter6'
                                                      * '<S156>/Parameter7'
                                                      * '<S156>/Parameter8'
                                                      * '<S179>/Parameter4'
                                                      * '<S179>/Parameter6'
                                                      * '<S179>/Parameter7'
                                                      * '<S179>/Parameter8'
                                                      */

/* First Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
const volatile real32_T DEV_Time2FltLaDmc_P = 0.025F;/* Referenced by:
                                                      * '<S156>/Parameter1'
                                                      * '<S156>/Parameter2'
                                                      * '<S156>/Parameter3'
                                                      * '<S156>/Parameter5'
                                                      * '<S179>/Parameter1'
                                                      * '<S179>/Parameter2'
                                                      * '<S179>/Parameter3'
                                                      * '<S179>/Parameter5'
                                                      */

/* Second Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
const volatile real32_T DEV_TimeCrvFWait_P = 0.0F;/* Referenced by: '<S194>/Parameter2' */

/* Time of the command value being set to 0 before the chirp signal output starts (for curvature interface of LaKMC) */
const volatile real32_T DEV_TimeDeltaFWait_P = 0.0F;/* Referenced by: '<S196>/Parameter2' */

/* Time of the command value being set to 0 before the chirp signal output starts */
const volatile real32_T DEV_TimeSysCycle_P = 0.01F;/* Referenced by:
                                                    * '<S195>/Parameter1'
                                                    * '<S197>/Parameter1'
                                                    * '<S151>/Parameter1'
                                                    * '<S152>/Parameter1'
                                                    * '<S153>/Parameter1'
                                                    * '<S153>/Parameter3'
                                                    * '<S157>/Parameter'
                                                    * '<S157>/Parameter1'
                                                    * '<S157>/Parameter2'
                                                    * '<S174>/Parameter1'
                                                    * '<S175>/Parameter1'
                                                    * '<S176>/Parameter1'
                                                    * '<S176>/Parameter3'
                                                    * '<S180>/Parameter'
                                                    * '<S180>/Parameter1'
                                                    * '<S180>/Parameter2'
                                                    */
const volatile real32_T DTE_CoeffDenS0LaDmc_M[13] = { 983.13F, 983.13F, 983.13F,
  29.8F, 336.08F, 298.54F, 174.2F, 151.01F, 101.08F, 102.31F, 102.31F, 102.31F,
  102.31F } ;                     /* Referenced by: '<S241>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS0LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S241>/1-D Lookup Table' */

/* x axis for s^0 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS1LaDmc_M[13] = { 284.83F, 284.83F, 284.83F,
  126.53F, 144.37F, 145.85F, 119.99F, 112.06F, 105.55F, 121.65F, 121.65F,
  121.65F, 121.65F } ;           /* Referenced by: '<S241>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS1LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table1' */

/* X axis for s^1 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS2LaDmc_M[13] = { 40.43F, 40.43F, 40.43F,
  19.14F, 11.46F, 10.67F, 7.55F, 6.27F, 6.6F, 8.34F, 8.34F, 8.34F, 8.34F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table2' */

/* s^2 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS2LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table2' */

/* X axis for s^2 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS3LaDmc_M[13] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table3' */

/* s^3 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffDenS3LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table3' */

/* X axis for s^3 coefficient of the approximated LaDMC transfer function's denominator */
const volatile real32_T DTE_CoeffNumS0LaDmc_M[13] = { 663.81F, 663.81F, 663.81F,
  17.9F, 239.76F, 208.86F, 128.37F, 118.44F, 78.8F, 76.69F, 76.69F, 76.69F,
  76.69F } ;                     /* Referenced by: '<S241>/1-D Lookup Table4' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T DTE_CoeffNumS0LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table4' */

/* X axis for s^0 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T DTE_CoeffNumS1LaDmc_M[13] = { 97.96F, 97.96F, 97.96F,
  88.83F, 52.81F, 54.24F, 47.02F, 43.82F, 42.38F, 51.27F, 51.27F, 51.27F, 51.27F
} ;                              /* Referenced by: '<S241>/1-D Lookup Table5' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T DTE_CoeffNumS1LaDmc_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S241>/1-D Lookup Table5' */

/* X axis for s^1 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T DTE_CoeffReqCrvGain_P = 1.0F;/* Referenced by: '<S207>/Parameter4' */

/* Gain Coefficient for Required vehicle curvature by road bank angle compensation */
const volatile real32_T DTE_CoeffReqDeltaGain_P = 1.0F;/* Referenced by: '<S205>/Parameter3' */

/* Gain Coefficient for Required Delta by road bank angle compensation */
const volatile real32_T DTE_CorStiffFrontAxle_P = 152820.0F;/* Referenced by:
                                                             * '<S271>/Parameter9'
                                                             * '<S272>/Parameter8'
                                                             * '<S273>/Parameter13'
                                                             * '<S273>/Parameter2'
                                                             * '<S274>/Parameter10'
                                                             */

/* Cornering stiffness coefficient of the tires at the front axle */
const volatile real32_T DTE_CorStiffRearAxle_P = 169804.0F;/* Referenced by:
                                                            * '<S271>/Parameter5'
                                                            * '<S273>/Parameter1'
                                                            * '<S273>/Parameter10'
                                                            * '<S274>/Parameter5'
                                                            */
const volatile uint8_T DTE_CswBnkAglCpmn_P = 1U;/* Referenced by:
                                                 * '<S212>/Parameter'
                                                 * '<S213>/Parameter'
                                                 */

/* Configuration switch for road bank angle compensation activation function */
const volatile uint8_T DTE_CswDstrbCmpn_P = 1U;/* Referenced by:
                                                * '<S239>/Parameter'
                                                * '<S267>/Parameter'
                                                */

/* Configuration switch for disturbance compensator activation function */
const volatile real32_T DTE_DistCogToFrontAxle_P = 1.0F;/* Referenced by:
                                                         * '<S205>/Parameter1'
                                                         * '<S271>/Parameter7'
                                                         * '<S272>/Parameter9'
                                                         * '<S273>/Parameter12'
                                                         * '<S273>/Parameter3'
                                                         * '<S274>/Parameter2'
                                                         * '<S274>/Parameter4'
                                                         */

/* Distance between front axle and the vehicle's center of gravity */
const volatile real32_T DTE_DistCogToRearAxle_P = 1.0F;/* Referenced by:
                                                        * '<S205>/Parameter2'
                                                        * '<S271>/Parameter8'
                                                        * '<S273>/Parameter11'
                                                        * '<S273>/Parameter4'
                                                        * '<S274>/Parameter3'
                                                        * '<S274>/Parameter8'
                                                        */

/* Distance between rear axle and the vehicle's center of gravity */
const volatile uint8_T DTE_EnaFreezeByTgq_P = 3U;/* Referenced by:
                                                  * '<S201>/Parameter1'
                                                  * '<S201>/Parameter4'
                                                  */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T DTE_EnaOffByTgq_P = 0U;/* Referenced by: '<S201>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T DTE_InertiaVehicle_P = 1811.2F;/* Referenced by:
                                                        * '<S274>/Parameter6'
                                                        * '<S275>/Parameter6'
                                                        */

/* Inertia of the Vehicle */
const volatile real32_T DTE_MassVehicle_P = 1415.0F;/* Referenced by:
                                                     * '<S272>/Parameter2'
                                                     * '<S273>/Parameter6'
                                                     * '<S274>/Parameter1'
                                                     * '<S275>/Parameter2'
                                                     */

/* Mass of the vehicle */
const volatile real32_T DTE_MaxCrvByBnkAgl_M[7] = { 0.0015F, 0.0015F, 0.0015F,
  0.0015F, 0.0014F, 0.0005F, 0.00025F } ;
                                  /* Referenced by: '<S212>/1-D Lookup Table' */

/* Max required vehicle curvature by road bank angle compensation */
const volatile real32_T DTE_MaxCrvByBnkAgl_X[7] = { -10.0F, 0.0F, 5.0F, 10.0F,
  22.22F, 44.44F, 55.55F } ;      /* Referenced by: '<S212>/1-D Lookup Table' */

/* X axis for max required vehicle curvature by road bank angle compensation */
const volatile real32_T DTE_MaxDeltaByBnkAgl_M[7] = { 0.0015F, 0.0015F, 0.0015F,
  0.0015F, 0.0014F, 0.0005F, 0.00025F } ;
                                  /* Referenced by: '<S213>/1-D Lookup Table' */

/* Max required Delta by road bank angle compensation */
const volatile real32_T DTE_MaxDeltaByBnkAgl_X[7] = { -10.0F, 0.0F, 5.0F, 10.0F,
  22.22F, 44.44F, 55.55F } ;      /* Referenced by: '<S213>/1-D Lookup Table' */

/* X axis for max required Delta by road bank angle compensation */
const volatile real32_T DTE_MaxReqCrvByDstrb_M[7] = { 0.0015F, 0.0015F, 0.0015F,
  0.0015F, 0.0014F, 0.0005F, 0.00025F } ;
                                  /* Referenced by: '<S239>/1-D Lookup Table' */

/* Max required curvature by disturbance compensator */
const volatile real32_T DTE_MaxReqCrvByDstrb_X[7] = { -10.0F, 0.0F, 5.0F, 10.0F,
  22.22F, 44.44F, 55.55F } ;      /* Referenced by: '<S239>/1-D Lookup Table' */

/* X axis for max required curvature by disturbance compensator */
const volatile real32_T DTE_MaxReqDeltaByDstrb_M[7] = { 0.0015F, 0.0015F,
  0.0015F, 0.0015F, 0.0014F, 0.0005F, 0.00025F } ;
                                  /* Referenced by: '<S267>/1-D Lookup Table' */

/* Max required Delta by disturbance compensator */
const volatile real32_T DTE_MaxReqDeltaByDstrb_X[7] = { -10.0F, 0.0F, 5.0F,
  10.0F, 22.22F, 44.44F, 55.55F } ;
                                  /* Referenced by: '<S267>/1-D Lookup Table' */

/* X axis for max required Delta by disturbance compensator */
const volatile real32_T DTE_SelfSteerGrdnt_P = 1.0F;/* Referenced by: '<S205>/Parameter' */

/* Self Steering Gradient of the Vehicle */
const volatile real32_T DTE_ThdLmtReqCrv_P = 0.002F;/* Referenced by: '<S207>/Parameter2' */

/* Gradient Limit threshold of the curvature request needed to compensate the bank angle */
const volatile real32_T DTE_ThdVehVelX_P = 3.0F;/* Referenced by:
                                                 * '<S201>/Parameter'
                                                 * '<S201>/Parameter3'
                                                 */

/* Threhold of the absolute value of the velocity for avoiding a potential division by zero */
const volatile real32_T DTE_TiFltSteerAngle_P = 3.0F;/* Referenced by:
                                                      * '<S284>/Parameter1'
                                                      * '<S284>/Parameter6'
                                                      */

/* Time constant of the low pass filter used to filter the steering angle */
const volatile real32_T DTE_TimeFiterReqCrv_P = 0.1F;/* Referenced by: '<S206>/Parameter1' */

/* Low pass filtered time for required curvature */
const volatile real32_T DTE_TimeFltDeltaFPar_P = 0.03F;/* Referenced by: '<S176>/Parameter2' */
const volatile real32_T DTE_TimeFltDeltaFRte_P = 0.03F;/* Referenced by: '<S153>/Parameter2' */
const volatile real32_T DTE_TimeSysCycle_P = 0.06F;/* Referenced by:
                                                    * '<S206>/Parameter'
                                                    * '<S207>/Parameter3'
                                                    * '<S277>/Parameter1'
                                                    * '<S277>/Parameter2'
                                                    * '<S225>/Parameter'
                                                    * '<S225>/Parameter1'
                                                    * '<S227>/Parameter1'
                                                    * '<S228>/Parameter'
                                                    * '<S279>/Parameter1'
                                                    * '<S280>/Parameter'
                                                    * '<S285>/Parameter'
                                                    * '<S285>/Parameter1'
                                                    * '<S247>/Parameter1'
                                                    * '<S248>/Parameter1'
                                                    * '<S249>/Parameter1'
                                                    * '<S249>/Parameter2'
                                                    * '<S249>/Parameter3'
                                                    * '<S252>/Parameter'
                                                    * '<S252>/Parameter1'
                                                    * '<S252>/Parameter2'
                                                    */

/* DTE system cycle time */
const volatile real32_T EST_AnglePObsDThetaThd0_M[13] = { 1.5F, 1.5F, 1.5F, 1.5F,
  1.5F, 1.5F, 1.5F, 1.5F, 1.5F, 1.5F, 1.5F, 1.5F, 1.5F } ;
                                  /* Referenced by: '<S355>/1-D Lookup Table' */

const volatile real32_T EST_AnglePObsDThetaThd0_X[13] = { 0.0F, 5.0F, 10.0F,
  15.0F, 20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S355>/1-D Lookup Table' */

const volatile real32_T EST_AnglePObsDThetaThd_M[13] = { 3.14F, 3.14F, 3.14F,
  3.14F, 3.14F, 3.14F, 3.14F, 3.14F, 3.14F, 3.14F, 3.14F, 3.14F, 3.14F } ;
                                  /* Referenced by: '<S354>/1-D Lookup Table' */

const volatile real32_T EST_AnglePObsDThetaThd_X[13] = { 0.0F, 5.0F, 10.0F,
  15.0F, 20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S354>/1-D Lookup Table' */

const volatile real32_T EST_AnglePobsDThetaLmt1_P = 0.02F;/* Referenced by:
                                                           * '<S356>/Parameter2'
                                                           * '<S356>/Parameter8'
                                                           * '<S357>/Parameter7'
                                                           */
const volatile real32_T EST_AnglePobsDThetaLmt2_P = 0.04F;/* Referenced by: '<S357>/Parameter5' */
const volatile real32_T EST_AnglePobsDThetaThd1_P = 0.01F;/* Referenced by: '<S356>/Parameter1' */
const volatile real32_T EST_AnglePobsDThetaThd2_P = 0.005F;/* Referenced by: '<S357>/Parameter4' */
const volatile real32_T EST_CoeffL11Pobs_M[13] = { 19.644F, 19.644F, 13.616F,
  12.751F, 12.408F, 12.267F, 12.22F, 12.221F, 12.248F, 12.289F, 12.337F, 12.387F,
  12.438F } ;                     /* Referenced by: '<S311>/1-D Lookup Table' */

/* First element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL11Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S311>/1-D Lookup Table' */

/* X axis for first element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL11Sobs_M[13] = { 1.329F, 1.329F, 0.403F,
  0.057F, -0.061F, -0.101F, -0.106F, -0.094F, -0.073F, -0.047F, -0.018F, 0.012F,
  0.041F } ;                      /* Referenced by: '<S387>/1-D Lookup Table' */

const volatile real32_T EST_CoeffL11Sobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S387>/1-D Lookup Table' */

const volatile real32_T EST_CoeffL12Pobs_M[13] = { -2.537F, -2.537F, -1.384F,
  -1.171F, -1.096F, -1.061F, -1.043F, -1.031F, -1.024F, -1.019F, -1.015F,
  -1.013F, -1.011F } ;           /* Referenced by: '<S311>/1-D Lookup Table4' */

/* First element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL12Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table4' */

/* X axis for first element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL13Pobs_M[13] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table9' */

/* First element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL13Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table9' */

/* X axis for first element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL21Pobs_M[13] = { 14.508F, 14.508F, 14.508F,
  14.508F, 14.508F, 14.508F, 14.508F, 14.508F, 14.508F, 14.508F, 14.508F,
  14.508F, 14.508F } ;           /* Referenced by: '<S311>/1-D Lookup Table1' */

/* Second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL21Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table1' */

/* X axis for second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL21Sobs_M[13] = { 3.0F, 3.0F, 3.363F, 4.976F,
  6.188F, 7.178F, 8.012F, 8.723F, 9.336F, 9.869F, 10.334F, 10.743F, 11.106F } ;
                                 /* Referenced by: '<S387>/1-D Lookup Table1' */

const volatile real32_T EST_CoeffL21Sobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S387>/1-D Lookup Table1' */

const volatile real32_T EST_CoeffL22Pobs_M[13] = { 5.356F, 5.356F, 11.886F,
  13.264F, 14.043F, 14.575F, 14.977F, 15.299F, 15.566F, 15.794F, 15.991F,
  16.164F, 16.317F } ;           /* Referenced by: '<S311>/1-D Lookup Table5' */

/* Second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL22Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table5' */

/* X axis for second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL23Pobs_M[13] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                /* Referenced by: '<S311>/1-D Lookup Table10' */

/* Second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL23Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                /* Referenced by: '<S311>/1-D Lookup Table10' */

/* X axis for second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL31Pobs_M[13] = { 27.859F, 27.859F, 13.929F,
  9.286F, 6.965F, 5.572F, 4.643F, 3.98F, 3.482F, 3.095F, 2.786F, 2.533F, 2.322F
} ;                              /* Referenced by: '<S311>/1-D Lookup Table2' */

/* Third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL31Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table2' */

/* X axis for third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL32Pobs_M[13] = { 1.537F, 1.537F, 0.384F,
  0.171F, 0.096F, 0.061F, 0.043F, 0.031F, 0.024F, 0.019F, 0.015F, 0.013F, 0.011F
} ;                              /* Referenced by: '<S311>/1-D Lookup Table6' */

/* Third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL32Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table6' */

/* X axis for third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL33Pobs_M[13] = { 10.0F, 10.0F, 5.0F, 3.333F,
  2.5F, 2.0F, 1.667F, 1.429F, 1.25F, 1.111F, 1.0F, 0.909F, 0.833F } ;
                                /* Referenced by: '<S311>/1-D Lookup Table11' */

/* Third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL33Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                /* Referenced by: '<S311>/1-D Lookup Table11' */

/* X axis for third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL41Pobs_M[13] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table3' */

/* Fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL41Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;/* Referenced by:
                                                                    * '<S311>/1-D Lookup Table12'
                                                                    * '<S311>/1-D Lookup Table3'
                                                                    */

/* X axis for fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL42Pobs_M[13] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table7' */

/* Fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL42Pobs_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S311>/1-D Lookup Table7' */

/* X axis for fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CoeffL43Pobs_M[13] = { 15.0F, 15.0F, 15.0F, 15.0F,
  15.0F, 15.0F, 15.0F, 15.0F, 15.0F, 15.0F, 15.0F, 15.0F, 15.0F } ;
                                /* Referenced by: '<S311>/1-D Lookup Table12' */

/* Fourth element of third column of gain matrix of the Luenberger observer to estimate the course angle */
const volatile real32_T EST_CorStiffFrontAxle_P = 158550.0F;/* Referenced by:
                                                             * '<S596>/Parameter5'
                                                             * '<S608>/Parameter5'
                                                             * '<S407>/Parameter4'
                                                             * '<S408>/Parameter4'
                                                             * '<S409>/Parameter4'
                                                             * '<S410>/Parameter4'
                                                             * '<S411>/Parameter4'
                                                             * '<S412>/Parameter1'
                                                             */

/* Cornering stiffness coefficient of the tires at the front axle */
const volatile real32_T EST_CorStiffRearAxle_P = 175340.0F;/* Referenced by:
                                                            * '<S596>/Parameter6'
                                                            * '<S608>/Parameter6'
                                                            * '<S407>/Parameter5'
                                                            * '<S408>/Parameter5'
                                                            * '<S409>/Parameter5'
                                                            * '<S410>/Parameter5'
                                                            */
const volatile uint8_T EST_CswEnaBetaSObs_P = 0U;/* Referenced by: '<S374>/Parameter' */

/* Configuration switch for activating beta estimation of Sobs  */
const volatile uint8_T EST_CswPObsDThetaLmt_P = 2U;/* Referenced by: '<S358>/Parameter' */
const volatile uint8_T EST_CswPObsDYSel_P = 1U;/* Referenced by: '<S366>/Parameter2' */
const volatile uint8_T EST_CswPlObsCrv_P = 1U;/* Referenced by:
                                               * '<S389>/Parameter1'
                                               * '<S389>/Parameter2'
                                               */

/* Configuration switch for curvature of plant observer */
const volatile uint8_T EST_CswPlObsDeltaY_P = 1U;/* Referenced by: '<S390>/Parameter' */

/* Configuration switch for lateral distance of plant observer */
const volatile uint8_T EST_CswSteerAngle_P = 0U;/* Referenced by: '<S391>/Parameter2' */

/* Configuration switch for steer angle input
   0: Use Steer Angle signal provided by VDY 1: Use Steer Angle signal */
const volatile uint8_T EST_CswStrAngleDly_P = 0U;/* Referenced by:
                                                  * '<S386>/Parameter'
                                                  * '<S305>/Parameter1'
                                                  */

/* Configuration switch for steer angle delay cycles */
const volatile real32_T EST_DistCogToFrontAxle_P = 1.465F;/* Referenced by:
                                                           * '<S596>/Parameter'
                                                           * '<S608>/Parameter'
                                                           * '<S407>/Parameter'
                                                           * '<S408>/Parameter'
                                                           * '<S408>/Parameter7'
                                                           * '<S409>/Parameter'
                                                           * '<S410>/Parameter'
                                                           * '<S411>/Parameter'
                                                           * '<S412>/Parameter2'
                                                           * '<S413>/Parameter'
                                                           */

/* Distance between front axle and the vehicle's center of gravity */
const volatile real32_T EST_DistCogToRearAxle_P = 1.495F;/* Referenced by:
                                                          * '<S596>/Parameter1'
                                                          * '<S608>/Parameter1'
                                                          * '<S407>/Parameter1'
                                                          * '<S408>/Parameter1'
                                                          * '<S408>/Parameter8'
                                                          * '<S409>/Parameter1'
                                                          * '<S410>/Parameter1'
                                                          */

/* Distance between rear axle and the vehicle's center of gravity */
const volatile real32_T EST_DistPObsDYGrdntThd_M[13] = { 10.0F, 10.0F, 10.0F,
  10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F } ;
                                 /* Referenced by: '<S366>/1-D Lookup Table1' */

const volatile real32_T EST_DistPObsDYGrdntThd_X[13] = { 0.0F, 5.0F, 10.0F,
  15.0F, 20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S366>/1-D Lookup Table1' */

const volatile real32_T EST_EST_DistPObsDYThd_M[13] = { 10.0F, 10.0F, 10.0F,
  10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F } ;
                                  /* Referenced by: '<S368>/1-D Lookup Table' */

const volatile real32_T EST_EST_DistPObsDYThd_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S368>/1-D Lookup Table' */

const volatile uint8_T EST_EnaFreezeByTgq_P = 3U;/* Referenced by:
                                                  * '<S403>/Parameter1'
                                                  * '<S403>/Parameter4'
                                                  */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T EST_EnaOffByTgq_P = 0U;/* Referenced by: '<S403>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T EST_FacDThetaWghtHdrOf_P = 1.0F;/* Referenced by: '<S349>/Parameter2' */
const volatile real32_T EST_FacDThetaWghtHdrSf_P = 1.0F;/* Referenced by: '<S349>/Parameter3' */
const volatile real32_T EST_FacDThetaWghtHdr_P = 1.0F;/* Referenced by: '<S349>/Parameter1' */
const volatile real32_T EST_FacDYWghtHdrOf_P = 1.0F;/* Referenced by: '<S349>/Parameter5' */
const volatile real32_T EST_FacDYWghtHdrSf_P = 1.0F;/* Referenced by: '<S349>/Parameter6' */
const volatile real32_T EST_FacDYWghtHdr_P = 1.0F;/* Referenced by: '<S349>/Parameter4' */
const volatile real32_T EST_InertiaVehicle_P = 2687.1F;/* Referenced by:
                                                        * '<S608>/Parameter4'
                                                        * '<S407>/Parameter3'
                                                        * '<S407>/Parameter6'
                                                        * '<S408>/Parameter3'
                                                        * '<S408>/Parameter6'
                                                        * '<S409>/Parameter3'
                                                        * '<S410>/Parameter3'
                                                        * '<S411>/Parameter3'
                                                        * '<S412>/Parameter3'
                                                        */

/* Inertia of the Vehicle */
const volatile real32_T EST_MassVehicle_P = 2000.0F;/* Referenced by:
                                                     * '<S596>/Parameter2'
                                                     * '<S608>/Parameter2'
                                                     * '<S407>/Parameter2'
                                                     * '<S407>/Parameter7'
                                                     * '<S408>/Parameter2'
                                                     * '<S408>/Parameter9'
                                                     * '<S411>/Parameter2'
                                                     */

/* Mass of the vehicle */
const volatile real32_T EST_MaxDThetaDotHdr_P = 0.035F;/* Referenced by: '<S340>/Parameter2' */
const volatile real32_T EST_MaxDYDotHdr_P = 0.4F;/* Referenced by: '<S335>/Parameter2' */
const volatile real32_T EST_MinDThetaDotHdr_P = 0.0175F;/* Referenced by:
                                                         * '<S340>/Parameter1'
                                                         * '<S340>/Parameter3'
                                                         */
const volatile real32_T EST_MinDYDotHdr_P = 0.2F;/* Referenced by:
                                                  * '<S335>/Parameter1'
                                                  * '<S335>/Parameter3'
                                                  */
const volatile uint8_T EST_ModeTJALatCtrlOf_P = 2U;/* Referenced by: '<S345>/Parameter2' */
const volatile real32_T EST_PlObsInDYTolBndGL1 = 0.15F;/* Referenced by:
                                                        * '<S395>/Parameter2'
                                                        * '<S395>/Parameter8'
                                                        */
const volatile real32_T EST_PlObsInDYTolBndGL2_P = 0.5F;/* Referenced by:
                                                         * '<S396>/Parameter2'
                                                         * '<S396>/Parameter8'
                                                         */
const volatile real32_T EST_PlObsInDYTolBndThr1 = 0.15F;/* Referenced by: '<S395>/Parameter1' */
const volatile real32_T EST_PlObsInDYTolBndThr2_P = 0.02F;/* Referenced by: '<S396>/Parameter1' */
const volatile real32_T EST_PreviewDistX_M[13] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S413>/1-D Lookup Table1' */

/* Preview lateral distance for single track model */
const volatile real32_T EST_PreviewDistX_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S413>/1-D Lookup Table1' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
const volatile real32_T EST_RatioSteerGear_M[6] = { 15.2F, 15.2F, 15.2F, 15.2F,
  15.2F, 15.2F } ;                /* Referenced by: '<S391>/1-D Lookup Table' */

/* Steering Gear Ratio of the vehicle */
const volatile real32_T EST_RatioSteerGear_X[6] = { 0.0F, 0.8727F, 2.094F,
  3.2289F, 5.2273F, 8.8314F } ;   /* Referenced by: '<S391>/1-D Lookup Table' */

/* X axis for steering Gear Ratio of the vehicle */
const volatile uint8_T EST_StMainLcfOff_P = 0U;/* Referenced by: '<S345>/Parameter' */
const volatile uint8_T EST_StMainLcfTJA_P = 1U;/* Referenced by: '<S345>/Parameter1' */
const volatile real32_T EST_ThdBetaSatSObs_M[7] = { 0.0F, 0.0F, 0.175F, 0.175F,
  0.175F, 0.175F, 0.175F } ;      /* Referenced by: '<S374>/1-D Lookup Table' */

/* Threshold of saturation limit for beta of Sobs */
const volatile real32_T EST_ThdBetaSatSObs_X[7] = { -10.0F, 0.0F, 5.0F, 10.0F,
  22.21F, 44.44F, 55.55F } ;      /* Referenced by: '<S374>/1-D Lookup Table' */

/* X axis for threshold of saturation limit for beta of Sobs */
const volatile real32_T EST_ThdMeanHdrOf_M[13] = { 80.0F, 80.0F, 80.0F, 80.0F,
  80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table1' */

const volatile real32_T EST_ThdMeanHdrOf_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table1' */

const volatile real32_T EST_ThdMeanHdrSf_M[13] = { 80.0F, 80.0F, 80.0F, 80.0F,
  80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table2' */

const volatile real32_T EST_ThdMeanHdrSf_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table2' */

const volatile real32_T EST_ThdMeanHdr_M[13] = { 80.0F, 80.0F, 80.0F, 80.0F,
  80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F, 80.0F } ;
                                  /* Referenced by: '<S348>/1-D Lookup Table' */

const volatile real32_T EST_ThdMeanHdr_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F, 20.0F,
  25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                  /* Referenced by: '<S348>/1-D Lookup Table' */

const volatile real32_T EST_ThdMulHdrOf_M[13] = { 60.0F, 60.0F, 60.0F, 60.0F,
  60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table4' */

const volatile real32_T EST_ThdMulHdrOf_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table4' */

const volatile real32_T EST_ThdMulHdrSf_M[13] = { 60.0F, 60.0F, 60.0F, 60.0F,
  60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table5' */

const volatile real32_T EST_ThdMulHdrSf_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table5' */

const volatile real32_T EST_ThdMulHdr_M[13] = { 60.0F, 60.0F, 60.0F, 60.0F,
  60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table3' */

const volatile real32_T EST_ThdMulHdr_X[13] = { 0.0F, 5.0F, 10.0F, 15.0F, 20.0F,
  25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F } ;
                                 /* Referenced by: '<S348>/1-D Lookup Table3' */

const volatile real32_T EST_ThdVehVel_P = 3.0F;/* Referenced by:
                                                * '<S406>/Parameter1'
                                                * '<S406>/Parameter2'
                                                */

/* Limit the absolute value of the velocity for avoiding a potential division by zero */
const volatile real32_T EST_TiHdrDThetaDotFlt_P = 0.04F;/* Referenced by: '<S341>/Parameter7' */
const volatile real32_T EST_TiHdrDYDotFlt_P = 0.04F;/* Referenced by: '<S336>/Parameter7' */
const volatile real32_T EST_TiPlObsInCrvFlt_P = 0.2F;/* Referenced by: '<S389>/Parameter3' */
const volatile real32_T EST_TiSysCycle_P = 0.02F;/* Referenced by:
                                                  * '<S389>/Parameter9'
                                                  * '<S376>/Parameter1'
                                                  * '<S376>/Parameter9'
                                                  * '<S395>/Parameter3'
                                                  * '<S395>/Parameter9'
                                                  * '<S396>/Parameter3'
                                                  * '<S396>/Parameter9'
                                                  * '<S313>/Parameter1'
                                                  * '<S313>/Parameter2'
                                                  * '<S313>/Parameter3'
                                                  * '<S313>/Parameter4'
                                                  * '<S336>/Parameter6'
                                                  * '<S341>/Parameter6'
                                                  * '<S366>/Parameter3'
                                                  * '<S366>/Parameter9'
                                                  * '<S355>/Parameter3'
                                                  * '<S355>/Parameter9'
                                                  * '<S356>/Parameter3'
                                                  * '<S356>/Parameter9'
                                                  * '<S357>/Parameter10'
                                                  * '<S357>/Parameter6'
                                                  */
const volatile uint8_T FFC_CswFfcCrv_P = 1U;/* Referenced by: '<S80>/Parameter' */

/* Configuration switch for feedforward curvature activation */
const volatile uint8_T FFC_StTgqReqFreeze_P = 0U;/* Referenced by: '<S80>/Parameter1' */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T FFC_StTgqReqOff_P = 0U;/* Referenced by: '<S80>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile uint8_T LGC_BtmDynIntRst_P = 0U;/* Referenced by:
                                                * '<S434>/Parameter12'
                                                * '<S434>/Parameter13'
                                                * '<S434>/Parameter14'
                                                * '<S434>/Parameter15'
                                                * '<S434>/Parameter16'
                                                * '<S434>/Parameter17'
                                                */

/* UNDEFINED */
const volatile uint8_T LGC_BtmDynPT1Rst_P = 0U;/* Referenced by:
                                                * '<S434>/Parameter'
                                                * '<S434>/Parameter1'
                                                * '<S434>/Parameter2'
                                                * '<S434>/Parameter3'
                                                * '<S434>/Parameter4'
                                                */

/* 0000 0000 : Dynamic PT1 reset deactivated
   0000 xxx1 : Reset based on lateral deviation active
   0000 xx1x : Reset based on driver torque active
   0000 x1xx : Reset based on standstill active */
const volatile real32_T LGC_CoeffDGainLcCac_M[15] = { 0.18F, 0.18F, 0.18F, 0.18F,
  0.18F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F } ;
                         /* Referenced by: '<S426>/Y_TCTLGC_LdcDGain_radspm1' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainLcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                         /* Referenced by: '<S426>/Y_TCTLGC_LdcDGain_radspm1' */

const volatile real32_T LGC_CoeffDGainLcLdc_M[15] = { 0.15F, 0.15F, 0.15F, 0.15F,
  0.15F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F } ;
                          /* Referenced by: '<S430>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainLcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S430>/Y_TCTLGC_LdcDGain_radspm' */

const volatile real32_T LGC_CoeffDGainOfCac_M[15] = { 0.27F, 0.27F, 0.27F, 0.27F,
  0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F, 0.27F } ;
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainOfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */

const volatile real32_T LGC_CoeffDGainOfLdc_M[15] = { 0.15F, 0.15F, 0.15F, 0.15F,
  0.15F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F } ;
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainOfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */

const volatile real32_T LGC_CoeffDGainSfCac_M[15] = { 0.232F, 0.232F, 0.232F,
  0.232F, 0.232F, 0.232F, 0.232F, 0.232F, 0.232F, 0.232F, 0.232F, 0.232F, 0.232F,
  0.232F, 0.232F } ;    /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainSfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */

const volatile real32_T LGC_CoeffDGainSfLdc_M[15] = { 0.15F, 0.15F, 0.15F, 0.15F,
  0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F } ;
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffDGainSfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */

const volatile real32_T LGC_CoeffIGainLcCac_M[15] = { 0.01F, 0.01F, 0.01F, 0.01F,
  0.01F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F } ;
                          /* Referenced by: '<S427>/Y_TCTLGC_LdcIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainLcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S427>/Y_TCTLGC_LdcIGain_radpsm' */

const volatile real32_T LGC_CoeffIGainLcLdc_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                          /* Referenced by: '<S431>/Y_TCTLGC_LdcIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainLcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S431>/Y_TCTLGC_LdcIGain_radpsm' */

const volatile real32_T LGC_CoeffIGainOfCac_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainOfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */

const volatile real32_T LGC_CoeffIGainOfLdc_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainOfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */

const volatile real32_T LGC_CoeffIGainSfCac_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainSfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */

const volatile real32_T LGC_CoeffIGainSfLdc_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffIGainSfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */

const volatile real32_T LGC_CoeffMainPGainLcCac_P = 1.0F;
                               /* Referenced by: '<S428>/P_TCTLGC_LdcP_radpm' */

/* Proportional master gain of the Course Angle Controller */
const volatile real32_T LGC_CoeffMainPGainLcLdc_P = 1.0F;
                               /* Referenced by: '<S432>/P_TCTLGC_LdcP_radpm' */

/* Proportional master gain of the Lateral Deviation Controller(Lane Centering)
 */
const volatile real32_T LGC_CoeffMainPGainOfCac_P = 1.0F;/* Referenced by: '<S428>/Constant' */

/* Proportional master gain of the Course Angle Controller */
const volatile real32_T LGC_CoeffMainPGainOfLdc_P = 1.0F;/* Referenced by: '<S432>/Constant' */

/* Proportional master gain of the Lateral Deviation Controller */
const volatile real32_T LGC_CoeffMainPGainSfCac_P = 1.0F;
                             /* Referenced by: '<S428>/P_TCTLGC_LdcSfP_radpm' */

/* Proportional master gain of the Course Angle Controller */
const volatile real32_T LGC_CoeffMainPGainSfLdc_P = 1.0F;
                             /* Referenced by: '<S432>/P_TCTLGC_LdcSfP_radpm' */

/* Proportional master gain of the Lateral Deviation Controller */
const volatile real32_T LGC_CoeffNumS0LaDmc_M[15] = { 30.14F, 30.14F, 30.14F,
  121.9F, 120.1F, 126.2F, 110.6F, 105.1F, 99.9F, 115.0F, 115.0F, 115.0F, 115.0F,
  115.0F, 115.0F } ;              /* Referenced by: '<S425>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T LGC_CoeffNumS0LaDmc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                  /* Referenced by: '<S425>/1-D Lookup Table' */

/* X axis for s^0 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T LGC_CoeffNumS1LaDmc_M[15] = { 7.807F, 7.807F, 7.807F,
  18.9F, 8.663F, 8.304F, 5.975F, 4.833F, 5.588F, 7.451F, 7.451F, 7.451F, 7.451F,
  7.451F, 7.451F } ;             /* Referenced by: '<S425>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T LGC_CoeffNumS1LaDmc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                 /* Referenced by: '<S425>/1-D Lookup Table1' */

/* X aixs for s^1 coefficient of the approximated LaDMC transfer function's numerator */
const volatile real32_T LGC_CoeffPGainByCrvCac_M[9] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                       /* Referenced by: '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */

const volatile real32_T LGC_CoeffPGainByCrvCac_X[9] = { -0.006F, -0.004F,
  -0.002F, -0.001F, 0.0F, 0.001F, 0.002F, 0.004F, 0.006F } ;
                       /* Referenced by: '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */

const volatile real32_T LGC_CoeffPGainByCrvLdc_M[9] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                       /* Referenced by: '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */

const volatile real32_T LGC_CoeffPGainByCrvLdc_X[9] = { -0.006F, -0.004F,
  -0.002F, -0.001F, 0.0F, 0.001F, 0.002F, 0.004F, 0.006F } ;
                       /* Referenced by: '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */

const volatile real32_T LGC_CoeffPGainLcCac_M[15] = { 0.45F, 0.45F, 0.4F, 0.36F,
  0.15F, 0.15F, 0.18F, 0.18F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F } ;
                              /* Referenced by: '<S428>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Course Angle Controller in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffPGainLcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                              /* Referenced by: '<S428>/Y_TCTLGC_LdcP_radpm1' */

const volatile real32_T LGC_CoeffPGainLcLdc_M[15] = { 0.08F, 0.065F, 0.06F,
  0.055F, 0.03F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F,
  0.033F, 0.033F } ;          /* Referenced by: '<S432>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffPGainLcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                              /* Referenced by: '<S432>/Y_TCTLGC_LdcP_radpm1' */

const volatile real32_T LGC_CoeffPGainOfCac_M[15] = { 0.2F, 0.2F, 0.22F, 0.22F,
  0.15F, 0.15F, 0.18F, 0.18F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F, 0.15F } ;
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcOfP_radpm1' */

/* Course Angle Controller P-Gain for object following mode */
const volatile real32_T LGC_CoeffPGainOfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcOfP_radpm1' */

const volatile real32_T LGC_CoeffPGainOfLdc_M[15] = { 0.1F, 0.1F, 0.05F, 0.05F,
  0.03F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F, 0.033F,
  0.033F } ;                /* Referenced by: '<S432>/Y_TCTLGC_LdcOfP_radpm1' */

/* Lateral Deviation Controller P-Gain for object following mode */
const volatile real32_T LGC_CoeffPGainOfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcOfP_radpm1' */

const volatile real32_T LGC_CoeffPGainSfCac_M[15] = { 0.338F, 0.338F, 0.297F,
  0.297F, 0.203F, 0.203F, 0.203F, 0.203F, 0.162F, 0.162F, 0.162F, 0.162F, 0.162F,
  0.162F, 0.162F } ;        /* Referenced by: '<S428>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Course Angle Controller in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffPGainSfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcSfP_radpm1' */

const volatile real32_T LGC_CoeffPGainSfLdc_M[15] = { 0.1F, 0.1F, 0.05F, 0.05F,
  0.03F, 0.018F, 0.014F, 0.014F, 0.014F, 0.014F, 0.014F, 0.014F, 0.014F, 0.014F,
  0.014F } ;                /* Referenced by: '<S432>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
const volatile real32_T LGC_CoeffPGainSfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcSfP_radpm1' */

const volatile real32_T LGC_CoeffPT1GainLcCac_M[15] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                          /* Referenced by: '<S429>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainLcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S429>/Y_TCTLGC_LdcDGain_radspm' */

const volatile real32_T LGC_CoeffPT1GainLcLdc_M[15] = { 2.0F, 2.0F, 2.0F, 2.0F,
  2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F } ;
                          /* Referenced by: '<S433>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainLcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S433>/Y_TCTLGC_LdcDGain_radspm' */

const volatile real32_T LGC_CoeffPT1GainOfCac_M[15] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainOfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */

const volatile real32_T LGC_CoeffPT1GainOfLdc_M[15] = { 1.2F, 1.2F, 1.2F, 1.2F,
  1.0F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F, 0.8F } ;
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainOfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */

const volatile real32_T LGC_CoeffPT1GainSfCac_M[15] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainSfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */

const volatile real32_T LGC_CoeffPT1GainSfLdc_M[15] = { 2.0F, 2.0F, 2.0F, 2.0F,
  2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F } ;
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_CoeffPT1GainSfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */

const volatile real32_T LGC_CoeffPole1LaDmc_M[15] = { -10.0F, -10.0F, -10.0F,
  -10.0F, -10.0F, -10.0F, -10.0F, -10.0F, -10.0F, -10.0F, -10.0F, -10.0F, -10.0F,
  -10.0F, -10.0F } ;             /* Referenced by: '<S425>/1-D Lookup Table2' */

/* Pole 1 of the approximated LaDMC transfer function */
const volatile real32_T LGC_CoeffPole1LaDmc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                 /* Referenced by: '<S425>/1-D Lookup Table2' */

/* X axis for pole 1 of the approximated LaDMC transfer function */
const volatile real32_T LGC_CoeffPole2LaDmc_M[15] = { -40.0F, -40.0F, -40.0F,
  -40.0F, -40.0F, -40.0F, -40.0F, -40.0F, -40.0F, -40.0F, -40.0F, -40.0F, -40.0F,
  -40.0F, -40.0F } ;             /* Referenced by: '<S425>/1-D Lookup Table3' */

/* Pole 2 of the approximated LaDMC transfer function */
const volatile real32_T LGC_CoeffPole2LaDmc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                                 /* Referenced by: '<S425>/1-D Lookup Table3' */

/* X axis for pole 2 of the approximated LaDMC transfer function */
const volatile uint8_T LGC_CswActOverride_P = 0U;/* Referenced by:
                                                  * '<S628>/Constant18'
                                                  * '<S590>/Constant18'
                                                  */

/* Configuration switch for DMC command signal
   1: Force ENABLE OUTPUT = 1
   0: Normal ENABLE OUTPUT behavior */
const volatile uint8_T LGC_CswCacMode_P = 7U;/* Referenced by:
                                              * '<S444>/P_TCTLGC_LdcMode_nu'
                                              * '<S444>/P_TCTLGC_LdcMode_nu1'
                                              * '<S444>/P_TCTLGC_LdcMode_nu2'
                                              * '<S444>/P_TCTLGC_LdcMode_nu3'
                                              * '<S484>/P_TCTLGC_LdcMode_nu'
                                              * '<S484>/P_TCTLGC_LdcMode_nu1'
                                              * '<S484>/P_TCTLGC_LdcMode_nu2'
                                              * '<S484>/P_TCTLGC_LdcMode_nu3'
                                              */

/* Y-Coordinate Controller Mode:
   xxxx xxx0	Controller is deactivated
   xxxx xxx1	P-Controller
   xxxx xx11	PDT1-Controller, DT1-Controller parallel to the P-Controller
   xxxx x1x1	PI-Controller, I-Part parallel to the P-Controller
   xxxx 1xx1	PPT1-Controller, PT1-Part parallel to the P-Controller */
const volatile uint8_T LGC_CswCssDeltaF_P = 1U;/* Referenced by:
                                                * '<S574>/Constant10'
                                                * '<S574>/Constant15'
                                                * '<S574>/Constant17'
                                                * '<S574>/Constant18'
                                                * '<S574>/Constant8'
                                                */

/* Mode of TCTLGC (Steer Angle interface):
   Source  |  Controller
   0000 0000: Off
   0000 xxx1: FBC active
   0000 xx1x: FFC active
   0000 x1xx: DC active
   0000 1xxx: BAC active
   0001 0000: Chirp active */
const volatile uint8_T LGC_CswFltErrCourse_P = 0U;/* Referenced by:
                                                   * '<S463>/Constant1'
                                                   * '<S503>/Constant1'
                                                   */

/* 1: Enable low pass first order for filtering the control error regarding the Course Angle
   0: Disable low pass first order for filtering the control error regarding the Course Angle */
const volatile uint8_T LGC_CswFltErrDistY_P = 0U;/* Referenced by: '<S545>/Constant1' */

/* 1: Enable low pass first order for filtering the control error regarding the Y-Coordinate
   0: Disable low pass first order for filtering the control error regarding the Y-Coordinate */
const volatile uint8_T LGC_CswLaDmcCmpnCac_P = 0U;/* Referenced by:
                                                   * '<S461>/Constant'
                                                   * '<S501>/Constant'
                                                   */

/* 1: Enable filter to compensate the LaDMC's transmission behavior
   0: Disable filter to compensate the LaDMC's transmission behavior */
const volatile uint8_T LGC_CswLaDmcCmpnLdc_P = 0U;/* Referenced by: '<S543>/Constant' */

/* 1: Enable filter to compensate the LaDMC's transmission behavior
   0: Disable filter to compensate the LaDMC's transmission behavior */
const volatile uint8_T LGC_CswLdcMode_P = 1U;/* Referenced by:
                                              * '<S524>/P_TCTLGC_LdcMode_nu'
                                              * '<S524>/P_TCTLGC_LdcMode_nu1'
                                              * '<S524>/P_TCTLGC_LdcMode_nu2'
                                              * '<S524>/P_TCTLGC_LdcMode_nu3'
                                              */

/* Y-Coordinate Controller Mode:
   xxxx xxx0	Controller is deactivated
   xxxx xxx1	P-Controller
   xxxx xx11	PDT1-Controller, DT1-Controller parallel to the P-Controller
   xxxx x1x1	PI-Controller, I-Part parallel to the P-Controller
   xxxx 1xx1	PPT1-Controller, PT1-Part parallel to the P-Controller */
const volatile uint8_T LGC_CswPT1DeltaFCmd_P = 0U;/* Referenced by: '<S574>/Constant13' */

/* 1: Enable low pass first order for filtering the control signal DeltaFCmd
   0: Disable low pass first order for filtering the control signal DeltaFCmd */
const volatile uint8_T LGC_EnaFreezeByTgq_P = 3U;/* Referenced by:
                                                  * '<S417>/Parameter1'
                                                  * '<S417>/Parameter4'
                                                  * '<S597>/Parameter1'
                                                  * '<S597>/Parameter3'
                                                  * '<S597>/Parameter4'
                                                  * '<S597>/Parameter5'
                                                  */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T LGC_EnaFreezeByVel_P = 10.0F;/* Referenced by: '<S597>/Parameter8' */
const volatile uint8_T LGC_EnaOffByTgq_P = 0U;/* Referenced by:
                                               * '<S417>/Parameter2'
                                               * '<S597>/Parameter2'
                                               */

/* Off parameter for Trajectory Guidance Qualifier Output */
const volatile real32_T LGC_IntResMaxCrv_P = 0.1F;/* Referenced by: '<S435>/Parameter1' */

/* UNDEFINED */
const volatile real32_T LGC_IntResMaxLatErr_P = 0.1F;/* Referenced by: '<S435>/Parameter' */

/* UNDEFINED */
const volatile real32_T LGC_IntResMaxManTrq_P = 1.0F;/* Referenced by: '<S435>/Parameter2' */

/* UNDEFINED */
const volatile real32_T LGC_MaxGrdPT1Ldc_P = 0.005F;/* Referenced by: '<S528>/Constant12' */

/* Absolute value of the maximum allowed gradient of the PT1 control signal part of the lateral deviation controller */
const volatile real32_T LGC_MaxReqDeltaFGrd_P = 10.0F;/* Referenced by:
                                                       * '<S627>/Constant'
                                                       * '<S589>/Constant'
                                                       */

/* Maximum allowed gradient of the steer angle command signal which is send to the LaDMC */
const volatile real32_T LGC_MaxReqDeltaF_P = 45.0F;/* Referenced by:
                                                    * '<S628>/Constant2'
                                                    * '<S628>/Constant3'
                                                    * '<S590>/Constant2'
                                                    * '<S590>/Constant3'
                                                    */

/* Maximum allowed steer angle command signal which is send to the LaDMC */
const volatile uint8_T LGC_OfLatCtrlMode_P = 2U;/* Referenced by: '<S422>/Parameter' */

/* Of mode for lateral control by TJASTM */
const volatile uint8_T LGC_OffLcfCtrlFcn_P = 0U;/* Referenced by: '<S422>/Parameter1' */

/* Off function for lcf control funtion by MCTLFC */
const volatile uint8_T LGC_TJALcfCtrlFcn_P = 1U;/* Referenced by: '<S422>/Parameter2' */

/* TJA function for lcf control funtion by MCTLFC */
const volatile real32_T LGC_ThdPGainGrdCac_P = 0.0F;/* Referenced by:
                                                     * '<S497>/Constant6'
                                                     * '<S497>/Constant8'
                                                     */

/* Gradient Limit for P-Gain Transition */
const volatile real32_T LGC_ThdPGainGrdCas_P = 0.05F;/* Referenced by:
                                                      * '<S457>/Constant6'
                                                      * '<S457>/Constant8'
                                                      * '<S458>/Constant1'
                                                      * '<S498>/Constant1'
                                                      */

/* Gradient Limit for P-Gain Transition */
const volatile real32_T LGC_ThdPGainGrdLdc_P = 0.05F;/* Referenced by:
                                                      * '<S539>/Constant6'
                                                      * '<S539>/Constant8'
                                                      * '<S540>/Constant1'
                                                      * '<S446>/Constant18'
                                                      * '<S486>/Constant18'
                                                      * '<S526>/Constant18'
                                                      */

/* Gradient Limit for P-Gain Transition */
const volatile real32_T LGC_ThdVelStandStill_P = 1.0F;/* Referenced by: '<S435>/Parameter3' */

/* Definition of standstill for TCTLGC */
const volatile real32_T LGC_TimeDT1LcCac_M[15] = { 0.07F, 0.07F, 0.07F, 0.07F,
  0.07F, 0.07F, 0.07F, 0.07F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F } ;
                               /* Referenced by: '<S426>/Y_TCTLGC_LdcDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
const volatile real32_T LGC_TimeDT1LcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                               /* Referenced by: '<S426>/Y_TCTLGC_LdcDT1_sec' */

const volatile real32_T LGC_TimeDT1LcLdc_M[15] = { 0.05F, 0.05F, 0.05F, 0.05F,
  0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F } ;
                               /* Referenced by: '<S430>/Y_TCTLGC_LdcDT1_sec' */

/* Time constant of the first order low pass needed to filter the Y-Coorindate Controller's D-part in dependence on the vehicle's velocity */
const volatile real32_T LGC_TimeDT1LcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                               /* Referenced by: '<S430>/Y_TCTLGC_LdcDT1_sec' */

const volatile real32_T LGC_TimeDT1OfCac_M[15] = { 0.1F, 0.1F, 0.1F, 0.1F, 0.1F,
  0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F } ;
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
const volatile real32_T LGC_TimeDT1OfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDT1_sec' */

const volatile real32_T LGC_TimeDT1OfLdc_M[15] = { 0.05F, 0.05F, 0.05F, 0.05F,
  0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F } ;
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDT1_sec' */

/* Lateral Deviation Controller P-Gain for object following mode */
const volatile real32_T LGC_TimeDT1OfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDT1_sec' */

const volatile real32_T LGC_TimeDT1SfCac_M[15] = { 0.1F, 0.1F, 0.1F, 0.1F, 0.1F,
  0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F } ;
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
const volatile real32_T LGC_TimeDT1SfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDT1_sec' */

const volatile real32_T LGC_TimeDT1SfLdc_M[15] = { 0.05F, 0.05F, 0.05F, 0.05F,
  0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F, 0.05F } ;
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDT1_sec' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
const volatile real32_T LGC_TimeDT1SfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDT1_sec' */

const volatile real32_T LGC_TimeFltErrCourse_M[15] = { 0.04F, 0.04F, 0.04F,
  0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F,
  0.04F } ;                            /* Referenced by:
                                        * '<S463>/Y_TCTLGC_PT1YErrTime_sec'
                                        * '<S503>/Y_TCTLGC_PT1YErrTime_sec'
                                        */

/* Time constant of the low pass filter used to filter the control error regarding the Course Angle */
const volatile real32_T LGC_TimeFltErrCourse_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;/* Referenced by:
                                                                      * '<S463>/Y_TCTLGC_PT1YErrTime_sec'
                                                                      * '<S503>/Y_TCTLGC_PT1YErrTime_sec'
                                                                      */

const volatile real32_T LGC_TimeFltErrDistY_M[15] = { 0.04F, 0.04F, 0.04F, 0.04F,
  0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F, 0.04F } ;
                          /* Referenced by: '<S545>/Y_TCTLGC_PT1YErrTime_sec' */

/* Time constant of the low pass filter used to filter the control error regarding the Y-Coordinate */
const volatile real32_T LGC_TimeFltErrDistY_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                          /* Referenced by: '<S545>/Y_TCTLGC_PT1YErrTime_sec' */

const volatile real32_T LGC_TimePT1DeltaFCmd_M[15] = { 0.07F, 0.07F, 0.07F,
  0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F, 0.07F,
  0.07F } ;          /* Referenced by: '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */

/* Velocity dependend value of the low pass' time constant to filter DeltaFCmd */
const volatile real32_T LGC_TimePT1DeltaFCmd_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                     /* Referenced by: '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */

const volatile real32_T LGC_TimePT1LcCac_M[15] = { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F,
  8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F } ;
                               /* Referenced by: '<S429>/Y_TCTLGC_LdcDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1LcCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                               /* Referenced by: '<S429>/Y_TCTLGC_LdcDT1_sec' */

const volatile real32_T LGC_TimePT1LcLdc_M[15] = { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F,
  8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F } ;
                               /* Referenced by: '<S433>/Y_TCTLGC_LdcDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1LcLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                               /* Referenced by: '<S433>/Y_TCTLGC_LdcDT1_sec' */

const volatile real32_T LGC_TimePT1OfCac_M[15] = { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F,
  8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F } ;
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1OfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDT1_sec' */

const volatile real32_T LGC_TimePT1OfLdc_M[15] = { 7.0F, 7.0F, 7.0F, 7.0F, 7.0F,
  7.0F, 7.0F, 7.0F, 7.0F, 7.0F, 7.0F, 7.0F, 7.0F, 7.0F, 7.0F } ;
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1OfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDT1_sec' */

const volatile real32_T LGC_TimePT1SfCac_M[15] = { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F,
  8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F } ;
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1SfCac_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDT1_sec' */

const volatile real32_T LGC_TimePT1SfLdc_M[15] = { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F,
  8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F, 8.0F } ;
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
const volatile real32_T LGC_TimePT1SfLdc_X[15] = { 0.0F, 5.0F, 10.0F, 15.0F,
  20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F, 65.0F, 70.0F } ;
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDT1_sec' */

const volatile real32_T LGC_TimeSysCycle_P = 0.02F;/* Referenced by:
                                                    * '<S82>/Parameter1'
                                                    * '<S82>/Parameter2'
                                                    * '<S82>/Parameter4'
                                                    * '<S419>/Parameter3'
                                                    * '<S601>/Parameter1'
                                                    * '<S602>/Parameter1'
                                                    * '<S574>/Parameter'
                                                    * '<S604>/Parameter3'
                                                    * '<S605>/Parameter4'
                                                    * '<S627>/Constant4'
                                                    * '<S463>/Parameter'
                                                    * '<S503>/Parameter'
                                                    * '<S545>/Parameter'
                                                    * '<S589>/Constant4'
                                                    * '<S457>/Constant7'
                                                    * '<S457>/Constant9'
                                                    * '<S458>/Constant5'
                                                    * '<S464>/Parameter'
                                                    * '<S464>/Parameter1'
                                                    * '<S465>/Parameter1'
                                                    * '<S467>/Parameter1'
                                                    * '<S467>/Parameter9'
                                                    * '<S497>/Constant7'
                                                    * '<S497>/Constant9'
                                                    * '<S498>/Constant5'
                                                    * '<S504>/Parameter'
                                                    * '<S504>/Parameter1'
                                                    * '<S505>/Parameter1'
                                                    * '<S507>/Parameter1'
                                                    * '<S507>/Parameter9'
                                                    * '<S539>/Constant7'
                                                    * '<S539>/Constant9'
                                                    * '<S540>/Constant5'
                                                    * '<S546>/Parameter'
                                                    * '<S546>/Parameter1'
                                                    * '<S547>/Parameter1'
                                                    * '<S549>/Parameter1'
                                                    * '<S549>/Parameter9'
                                                    * '<S446>/Parameter2'
                                                    * '<S446>/Parameter3'
                                                    * '<S447>/Parameter4'
                                                    * '<S448>/Parameter'
                                                    * '<S486>/Parameter2'
                                                    * '<S486>/Parameter3'
                                                    * '<S487>/Parameter4'
                                                    * '<S488>/Parameter'
                                                    * '<S526>/Parameter2'
                                                    * '<S526>/Parameter3'
                                                    * '<S527>/Parameter4'
                                                    * '<S528>/Parameter'
                                                    * '<S528>/Parameter1'
                                                    */
const volatile real32_T LQR_ButterCutFreq = 15.0F;/* Referenced by: '<S82>/Parameter3' */
const volatile real32_T LQR_ButterQ = 0.707F;/* Referenced by: '<S82>/Parameter5' */
const volatile real32_T LQR_FltDeltaDistYFc = 1.0F;/* Referenced by: '<S604>/Parameter1' */
const volatile real32_T LQR_FltKappaFc = 1.0F;/* Referenced by: '<S82>/Parameter6' */
const volatile real32_T LQR_FltYawRateFc = 1.0F;/* Referenced by: '<S602>/Parameter2' */
const volatile real32_T LQR_LeadLagCutFreq = 0.2F;/* Referenced by: '<S601>/Parameter2' */
const volatile real32_T LQR_LeadLagGain = 2.0F;/* Referenced by: '<S601>/Parameter3' */
const volatile real32_T LQR_VelX[9] = { 0.0F, 5.55555534F, 11.1111107F,
  16.666666F, 22.2222214F, 27.7777786F, 33.3333321F, 38.8888893F, 44.4444427F } ;/* Referenced by:
                                                                      * '<S598>/Constant1'
                                                                      * '<S607>/Constant1'
                                                                      */

const volatile real32_T P_TCTLGC_CacILimit_rad = 1.0F;/* Referenced by:
                                                       * '<S447>/P_TCTLGC_LdcILimit_rad'
                                                       * '<S487>/P_TCTLGC_LdcILimit_rad'
                                                       */

/* Gradient Limit for P-Gain Transition */
const volatile real32_T P_TCTLGC_LdcILimit_rad = 0.0F;
                            /* Referenced by: '<S527>/P_TCTLGC_LdcILimit_rad' */

/* Gradient Limit for P-Gain Transition */
const volatile real32_T deadZone_Vxthd = 80.0F;/* Referenced by: '<S595>/Parameter5' */
const volatile real32_T deadZone_weightedGain_e1 = 0.5F;/* Referenced by: '<S595>/Parameter1' */
const volatile real32_T deadZone_weightedGain_e2 = 0.5F;/* Referenced by: '<S595>/Parameter2' */
const volatile real32_T deadZone_width = 1.0F;/* Referenced by: '<S595>/Parameter3' */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T data_log_5;                   /* '<S439>/Gain' */
real32_T CLM_ReqFfcCrv;                /* '<S103>/Multiport Switch1'
                                        * Control signal of the Feedforward Controller after safety checks
                                        */
real32_T CLM_ReqFbcDcCrv;              /* '<S103>/Multiport Switch2'
                                        * Control signal of the Feedback Controller + Disturbance Compensator after safety checks
                                        */
real32_T CLM_LmtReqFfcCrv;             /* '<S103>/Add'
                                        * UNDEFINED
                                        */
real32_T CLM_DeltaFCmd;                /* '<S103>/Multiport Switch'
                                        * UNDEFINED
                                        */
real32_T CLM_SumCtrlCrv;               /* '<S134>/Add1'
                                        * UNDEFINED
                                        */
real32_T CLM_TimerPlauCheck;           /* '<S131>/Multiport Switch'
                                        * UNDEFINED
                                        */
real32_T CLM_HldVehCrv;                /* '<S131>/Multiport Switch1'
                                        * UNDEFINED
                                        */
real32_T CLM_ThdCrvPlauChkUp;          /* '<S136>/Multiport Switch'
                                        * UNDEFINED
                                        */
real32_T CLM_ThdCrvPlauChkLow;         /* '<S136>/Multiport Switch1'
                                        * UNDEFINED
                                        */
real32_T data_log_0;                   /* '<S81>/Gain' */
real32_T data_log_4;                   /* '<S439>/Gain1' */
real32_T data_log_3;                   /* '<S416>/Gain' */
real32_T data_log_6;                   /* '<S73>/Data Type Conversion8' */
real32_T data_log_7;                   /* '<S73>/Data Type Conversion9' */
real32_T data_log_8;                   /* '<S73>/Data Type Conversion10' */
real32_T data_log_1;                   /* '<S73>/Data Type Conversion17' */
real32_T data_log_2;                   /* '<S73>/Data Type Conversion18' */
real32_T data_log_9;                   /* '<S73>/Data Type Conversion5' */
uint8_T CLM_BtfQulifierTrajCtrl;       /* '<S105>/Add'
                                        * Trajectory controller qualifier. Includes relevant information for the Situation Assessment components.
                                        */
boolean_T CLM_EnaUnplauRequest;        /* '<S102>/Unit Delay'
                                        * UNDEFINED
                                        */
boolean_T CLM_EnaRstDegrReq;           /* '<S106>/OR'
                                        * Reset condition for Flip-Flop which holds degradation request
                                        */
boolean_T CLM_EnaSetDegrReq;           /* '<S110>/Switch'
                                        * Set condition for Flip-Flop which holds degradation request
                                        */
boolean_T CLM_EnaPlausibilityCheck;    /* '<S132>/AND1'
                                        * UNDEFINED
                                        */
boolean_T CLM_EnaHldVehCrv;            /* '<S135>/FixPt Relational Operator'
                                        * UNDEFINED
                                        */
boolean_T CLM_EnaLmtWarn;              /* '<S137>/AND'
                                        * UNDEFINED
                                        */
boolean_T CLM_EnaPlauCheck;            /* '<S131>/AND'
                                        * UNDEFINED
                                        */

/* Block states (default storage) */
DW_TJATCT_T TJATCT_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_TJATCT_T TJATCT_Y;



/* Definition for custom storage class: Global */
uint8_T CDC_BtfQualifier;              /* '<S86>/Multiport Switch3' */
real32_T CDC_CtrlErrDistY;             /* '<S87>/Multiport Switch2' */

/* UNDEFINED */
real32_T CDC_CtrlErrHeading;           /* '<S91>/Multiport Switch2' */
boolean_T CDC_EnaCntrlByTgq;           /* '<S85>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
boolean_T CDC_EnaFreezeByTgq;          /* '<S85>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
boolean_T CDC_EnaResetByTgq;           /* '<S85>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
boolean_T CDC_EnaWatchdog;             /* '<S86>/AND1' */

/* 0: Watchdog inactive
   1: Watchdog active */
real32_T CDC_EstCurHeading;            /* '<S92>/Neg2' */

/* UNDEFINED */
real32_T CDC_EstDeltaTheta;            /* '<S93>/Add1' */

/* Course Angle Deviation calculated by means of "HAF estimator" */
real32_T CDC_FltDeltaTheta;            /* '<S94>/Switch' */

/* PT1 filtered Course Angle Deviation */
real32_T CDC_FltErrDistYTpl;           /* '<S89>/Switch' */

/* Lateral Deviation, PT1 filtered */
real32_T CDC_HldCtrlErrDistY;          /* '<S87>/Multiport Switch' */

/* UNDEFINED */
real32_T CDC_HldCtrlErrHeading;        /* '<S91>/Multiport Switch1' */
real32_T CDC_PreErrCtrlHeading;        /* '<S91>/Data Type Conversion' */
uint8_T CDC_RawBtfQualifier;           /* '<S86>/Sum' */

/* UNDEFINED */
real32_T CDC_RawCtrlErrDistY;          /* '<S87>/Multiport Switch1' */

/* UNDEFINED */
real32_T CDC_RawCtrlErrHeading;        /* '<S91>/Multiport Switch' */
real32_T CDC_RawDeltaTheta;            /* '<S93>/Add' */

/* Course Angle Deviation */
real32_T CDC_RawErrDistYTpl;           /* '<S88>/Add' */

/* Lateral Deviation */
real32_T CDC_RawFltDeltaTheta;         /* '<S93>/Signal Conversion' */

/* PT1 filtered Course Angle Deviation */
real32_T CLM_CrvBySteerAngle;          /* '<S134>/Divide1' */

/* UNDEFINED */
boolean_T CLM_EnaDegrReq;              /* '<S106>/Multiport Switch1' */
boolean_T CLM_EnaGrdDeltaFCmd;         /* '<S115>/AND1' */
boolean_T CLM_EnaGrdFbcDc;             /* '<S120>/AND1' */
boolean_T CLM_EnaGrdFfcCrv;            /* '<S125>/AND1' */
boolean_T CLM_EnaSatDeltaFCmd;         /* '<S116>/GreaterThan' */
boolean_T CLM_EnaSatFbcDc;             /* '<S121>/GreaterThan' */
boolean_T CLM_EnaSatFfcCrv;            /* '<S126>/GreaterThan' */
real32_T CLM_GrdDeltaFCmd;             /* '<S115>/Multiport Switch' */

/* Steering Angle Command for DMC */
real32_T CLM_GrdFbcDc;                 /* '<S120>/Multiport Switch' */

/* UNDEFINED */
real32_T CLM_GrdFfcCrv;                /* '<S125>/Multiport Switch' */

/* UNDEFINED */
boolean_T CLM_RawEnaDegrReq;           /* '<S107>/Switch' */

/* Reset condition for Flip-Flop which holds degradation request */
real32_T CLM_RawGrdDeltaFCmd;          /* '<S117>/Multiport Switch1' */

/* Steering Angle Command for DMC */
real32_T CLM_RawGrdFbcDc;              /* '<S122>/Multiport Switch1' */

/* UNDEFINED */
real32_T CLM_RawGrdFfcCrv;             /* '<S127>/Multiport Switch1' */

/* UNDEFINED */
real32_T CLM_SatDeltaFCmd;             /* '<S119>/Switch2' */

/* Steering Angle Command for DMC */
real32_T CLM_SatFbcDc;                 /* '<S124>/Switch2' */

/* UNDEFINED */
real32_T CLM_SatFfcCrv;                /* '<S129>/Switch2' */

/* UNDEFINED */
real32_T CLM_ThdDeltaFCmdGrd;          /* '<S115>/Multiport Switch1' */
real32_T CLM_ThdDeltaFCmdSat;          /* '<S116>/Multiport Switch1' */
real32_T CLM_ThdFbcDcGrd;              /* '<S120>/1-D Lookup Table' */

/* UNDEFINED */
real32_T CLM_ThdFbcDcSat;              /* '<S121>/1-D Lookup Table2' */

/* UNDEFINED */
real32_T CLM_ThdFfcCrvGrd;             /* '<S125>/1-D Lookup Table' */

/* UNDEFINED */
real32_T CLM_ThdFfcCrvSat;             /* '<S126>/1-D Lookup Table2' */

/* UNDEFINED */
uint8_T DEV_BtfFfcQualifierPar;        /* '<S181>/Add2' */
uint8_T DEV_BtfFfcQualifierRte;        /* '<S158>/Add2' */
real32_T DEV_CoeffDeltaGainFfc;        /* '<S144>/Y_TCTFFC_GainFFC_nu' */

/* Gain of the control signal part of the feedforward controller */
real32_T DEV_CrvTestSignal;            /* '<S194>/Multiport Switch' */

/* Test signal for curvature output of lateral KMC */
real32_T DEV_DeltaFTestSignal;         /* '<S196>/Multiport Switch' */

/* Test signal for steering angle command from LaKMC to DMC */
real32_T DEV_DlySetDeltaF2DotPar;      /* '<S180>/Unit Delay1' */
real32_T DEV_DlySetDeltaF2DotRte;      /* '<S157>/Unit Delay1' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
real32_T DEV_DlySetDeltaFDotPar;       /* '<S180>/Unit Delay' */
real32_T DEV_DlySetDeltaFDotRte;       /* '<S157>/Unit Delay' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
real32_T DEV_DlySetDeltaFPar;          /* '<S180>/Unit Delay2' */
real32_T DEV_DlySetDeltaFRte;          /* '<S157>/Unit Delay2' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
boolean_T DEV_EnaCntrlByTgq;           /* '<S140>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
uint8_T DEV_EnaCrvGen;                 /* '<S195>/Bitwise Operator' */
uint8_T DEV_EnaDeltaFGen;              /* '<S197>/Bitwise Operator' */
boolean_T DEV_EnaFreezeByTgq;          /* '<S140>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
boolean_T DEV_EnaResetByTgq;           /* '<S140>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
real32_T DEV_HldReqDeltaFRte;          /* '<S143>/Multiport Switch1' */

/* Feedforward control signal part based on self steering gradient (given by VDY from RTE) for the steer angle interface */
real32_T DEV_ReqDeltaFRte;             /* '<S143>/Multiport Switch' */

/* Feedforward control signal part based on self steering gradient (given by VDY from RTE) for the steer angle interface */
boolean_T DEV_RstCrvGen;               /* '<S198>/FixPt Relational Operator' */
boolean_T DEV_RstDeltaFGen;            /* '<S199>/FixPt Relational Operator' */
real32_T DEV_SetDeltaF3DotPar;         /* '<S179>/Multiport Switch' */
real32_T DEV_SetDeltaF3DotRte;         /* '<S156>/Multiport Switch' */
real32_T DEV_SetDeltaFPar;             /* '<S191>/Switch2' */
real32_T DEV_SetDeltaFRte;             /* '<S168>/Switch2' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
real32_T DEV_TimeCrvGen;               /* '<S195>/Multiport Switch' */
real32_T DEV_TimeDeltaFGen;            /* '<S197>/Multiport Switch' */
real32_T DTE_CoeffA0TranferFcn;        /* '<S271>/Product1' */

/* s^0 coefficient of the approximated vehicle dynamic transfer function's numerator */
real32_T DTE_CoeffA1TranferFcn;        /* '<S272>/Product9' */

/* s^1 coefficient of the approximated vehicle dynamic transfer function's numerator */
real32_T DTE_CoeffB0TranferFcn;        /* '<S273>/Add' */

/* s^0 coefficient of the approximated vehicle dynamic transfer function's denominator */
real32_T DTE_CoeffB1TranferFcn;        /* '<S274>/Add1' */

/* s^1 coefficient of the approximated vehicle dynamic transfer function's denominator */
real32_T DTE_CoeffB2TranferFcn;        /* '<S275>/Product9' */

/* s^2 coefficient of the approximated vehicle dynamic transfer function's denominator */
real32_T DTE_CoeffDenS0LaDmc;          /* '<S241>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's denominator */
real32_T DTE_CoeffDenS1LaDmc;          /* '<S241>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's denominator */
real32_T DTE_CoeffDenS2LaDmc;          /* '<S241>/1-D Lookup Table2' */

/* s^2 coefficient of the approximated LaDMC transfer function's denominator */
real32_T DTE_CoeffDenS3LaDmc;          /* '<S241>/1-D Lookup Table3' */

/* s^3 coefficient of the approximated LaDMC transfer function's denominator */
real32_T DTE_CoeffNumS0LaDmc;          /* '<S241>/1-D Lookup Table4' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
real32_T DTE_CoeffNumS1LaDmc;          /* '<S241>/1-D Lookup Table5' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
real32_T DTE_Delta2DotForCrv;          /* '<S227>/Divide1' */

/* Second order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_Delta2DotLaDmc;           /* '<S247>/Divide1' */

/* Second order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_Delta2DotVdyFcn;          /* '<S284>/Multiport Switch' */

/* steer angle of the front wheels by vehicle dynamic transfer function */
real32_T DTE_Delta3DotLaDmc;           /* '<S248>/Divide1' */

/* Third order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_DeltaByVdyFcn;            /* '<S277>/Subtract' */

/* Current steer angle of the front wheels */
real32_T DTE_DeltaDotForCrv;           /* '<S228>/Divide' */

/* First order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_DeltaDotLaDmc;            /* '<S250>/Add' */

/* First order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_DeltaF2DotPar;            /* '<S174>/Divide1' */
real32_T DTE_DeltaF2DotRte;            /* '<S151>/Divide1' */
real32_T DTE_DeltaF3DotPar;            /* '<S175>/Divide1' */
real32_T DTE_DeltaF3DotRte;            /* '<S152>/Divide1' */
real32_T DTE_DeltaFDotPar;             /* '<S177>/Add' */
real32_T DTE_DeltaFDotRte;             /* '<S154>/Add' */
real32_T DTE_DeltaFPar;                /* '<S142>/Mul1' */
real32_T DTE_DeltaFRte;                /* '<S141>/Mul1' */
real32_T DTE_DeltaVdyFcn;              /* '<S288>/Switch2' */
real32_T DTE_DlyCurSteerAngle;         /* '<S277>/Multiport_Switch1' */

/* Current steer angle of the front wheels */
real32_T DTE_DlyDeltaDotVdyFcn;        /* '<S285>/Unit Delay1' */
real32_T DTE_DlyDeltaVdyFcn;           /* '<S285>/Unit Delay' */
real32_T DTE_DlySetCrvDotLaDmc;        /* '<S225>/Unit Delay1' */
real32_T DTE_DlySetCrvLaDmc;           /* '<S225>/Unit Delay' */
real32_T DTE_DlySetDelta2DotLaDmc;     /* '<S252>/Unit Delay1' */
real32_T DTE_DlySetDeltaDotLaDmc;      /* '<S252>/Unit Delay' */
real32_T DTE_DlySetDeltaLaDmc;         /* '<S252>/Unit Delay2' */
boolean_T DTE_EnaCtrlByTgq;            /* '<S201>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
boolean_T DTE_EnaFreezeByTgq;          /* '<S201>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
boolean_T DTE_EnaResetByTgq;           /* '<S201>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
real32_T DTE_EstCrvByBnkAgl;           /* '<S206>/Multiport Switch' */

/* Estimating curvature by road bank angle compensation */
real32_T DTE_FltDlyCurSteerAngle;      /* '<S282>/Add' */

/* Current steer angle of the front wheels */
real32_T DTE_HldReqCrvByBnkAgl;        /* '<S212>/Multiport Switch1' */

/* Holding value for required vehicle curvature by road bank angle compensation */
real32_T DTE_HldReqCrvByDstrb;         /* '<S239>/Multiport Switch1' */

/* Holding value for required curvature by disturbance compensator */
real32_T DTE_HldReqDeltaByBnkAgl;      /* '<S213>/Multiport Switch1' */

/* Holding value for required Delta by road bank angle compensation */
real32_T DTE_HldReqDeltaByDstrb;       /* '<S267>/Multiport Switch1' */

/* Holding value for required Delta by disturbance compensator */
real32_T DTE_KappaAngleLaDmc;          /* '<S229>/1-D Lookup Table' */

/* Grid points of the y-Axis of the LaDMC Look Up Table "Kappa To Angle". */
real32_T DTE_LmtEstCrvByBnkAgl;        /* '<S207>/Multiport Switch2' */

/* Gradient limiter value for Estimating curvature by road bank angle compensation */
real32_T DTE_LmtReqCrvByBnkAgl;        /* '<S214>/Switch2' */

/* Limiting value for required vehicle curvature by road bank angle compensation */
real32_T DTE_LmtReqCrvByDstrb;         /* '<S240>/Switch2' */

/* Limiting value for required curvature by disturbance compensator */
real32_T DTE_LmtReqDeltaByBnkAgl;      /* '<S215>/Switch2' */

/* Limiting value for required Delta by road bank angle compensation */
real32_T DTE_LmtReqDeltaByDstrb;       /* '<S268>/Switch2' */

/* Limiting value for required Delta by disturbance compensator */
real32_T DTE_LmtVehVelX;               /* '<S217>/Switch2' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
real32_T DTE_MaxCrvByBnkAgl;           /* '<S212>/1-D Lookup Table' */

/* Max required vehicle curvature by road bank angle compensation */
real32_T DTE_MaxDeltaByBnkAgl;         /* '<S213>/1-D Lookup Table' */

/* Max required Delta by road bank angle compensation */
real32_T DTE_MaxReqCrvByDstrb;         /* '<S239>/1-D Lookup Table' */

/* Max required curvature by disturbance compensator */
real32_T DTE_MaxReqDeltaByDstrb;       /* '<S267>/1-D Lookup Table' */

/* Max required Delta by disturbance compensator */
real32_T DTE_NdlySetCrvLaDmc;          /* '<S238>/Multiport_Switch1' */
real32_T DTE_NdlySetDeltaLaDmc;        /* '<S266>/Multiport_Switch1' */
real32_T DTE_Psi2DotVdyFcn;            /* '<S280>/Divide' */

/* First1 order discrete derivative of ego vehicle yaw rate for vehicle dynamic transfer function */
real32_T DTE_Psi3DotVdyFcn;            /* '<S279>/Divide1' */

/* Second order discrete derivative of ego vehicle yaw rate for vehicle dynamic transfer function */
real32_T DTE_RawCrvLaDmc;              /* '<S229>/Product1' */
real32_T DTE_RawDeltaDotLaDmc;         /* '<S249>/Divide1' */

/* Raw first order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
real32_T DTE_RawDeltaFDotPar;          /* '<S176>/Divide1' */
real32_T DTE_RawDeltaFDotRte;          /* '<S153>/Divide1' */
real32_T DTE_RawFltEstCrvByBnkAgl;     /* '<S208>/Add' */

/* Raw low pass filtered value for Estimating curvature by road bank angle compensation */
real32_T DTE_RawLmtEstCrvByBnkAgl;     /* '<S210>/Multiport Switch1' */

/* Raw gradient limiter value for Estimating curvature by road bank angle compensation */
real32_T DTE_RawReqCrvByBnkAgl;        /* '<S207>/Product1' */

/* Raw required vehicle curvature by road bank angle compensation */
real32_T DTE_RawReqCrvByDstrb;         /* '<S238>/Product' */

/* Raw required curvature by disturbance compensator */
real32_T DTE_RawReqDeltaByBnkAgl;      /* '<S205>/Product3' */
real32_T DTE_RawReqDeltaByDstrb;       /* '<S266>/Product' */

/* Raw required Delta by disturbance compensator */
real32_T DTE_ReqCrvByBnkAgl;           /* '<S212>/Multiport Switch' */

/* Required vehicle curvature by road bank angle compensation */
real32_T DTE_ReqCrvByDstrb;            /* '<S239>/Multiport Switch' */

/* Required curvature by disturbance compensator */
real32_T DTE_ReqDeltaByBnkAgl;         /* '<S213>/Multiport Switch' */

/* Required Delta by road bank angle compensation */
real32_T DTE_ReqDeltaByDstrb;          /* '<S267>/Multiport Switch' */

/* Required Delta by disturbance compensator */
real32_T DTE_ResCrvDenLaDmc;           /* '<S226>/Add3' */
real32_T DTE_ResDeltaDenLaDmc;         /* '<S246>/Add1' */

/* Vehicle dynamic transfer function's denominator result for delta */
real32_T DTE_ResDeltaDenPar;           /* '<S173>/Add1' */
real32_T DTE_ResDeltaDenRte;           /* '<S150>/Add1' */
real32_T DTE_ResDeltaDenVdyFcn;        /* '<S281>/Add1' */

/* Vehicle dynamic transfer function's denominator result for delta */
real32_T DTE_SetCrv2DotLaDmc;          /* '<S224>/Multiport Switch' */
real32_T DTE_SetCrvGainLaDmc;          /* '<S238>/1-D Lookup Table' */
real32_T DTE_SetCrvLaDmc;              /* '<S232>/Switch2' */
real32_T DTE_SetDelta3DotLaDmc;        /* '<S253>/Multiport Switch' */

/* Third order discrete derivative of ego vehicle set delta for lateral dynamic transfer function */
real32_T DTE_SetDeltaGainLaDmc;        /* '<S266>/1-D Lookup Table' */

/* Gain of the Disturbance Compensator set value */
real32_T DTE_SetDeltaLaDmc;            /* '<S263>/Switch2' */
real32_T EST_AngleCurSteer;            /* '<S391>/Multiport Switch' */

/* Current steer angle of the front wheels */
real32_T EST_AngleLaDMCSteer;          /* '<S391>/Product' */

/* Current steer angle of the front wheels from LaDMC */
real32_T EST_AnglePObsDTheta;          /* '<S354>/Multiport Switch' */
real32_T EST_AnglePObsDThetaFreeze;    /* '<S354>/Multiport Switch1' */
real32_T EST_AnglePObsDThetaLmt0;      /* '<S355>/Multiport Switch' */
real32_T EST_AnglePObsDThetaLmt0Raw;   /* '<S359>/Multiport Switch1' */
real32_T EST_AnglePObsDThetaLmt1;      /* '<S361>/Multiport Switch1' */
real32_T EST_AnglePObsDThetaLmt2;      /* '<S363>/Multiport Switch1' */
real32_T EST_AnglePObsDThetaSat;       /* '<S365>/Switch2' */
real32_T EST_AnglePObsDThetaSel;       /* '<S358>/Multiport Switch1' */
real32_T EST_AnglePObsDThetaThd;       /* '<S354>/1-D Lookup Table' */
real32_T EST_AnglePObsDThetaThd0;      /* '<S355>/1-D Lookup Table' */
real32_T EST_AngleVDYSteer;            /* '<S391>/Divide1' */

/* Current steer angle of the front wheels from VDY */
real32_T EST_BetaDotPobs;              /* '<S312>/Signal Conversion' */
real32_T EST_BetaDotSObs;              /* '<S375>/Signal Conversion' */
real32_T EST_BetaSObs;                 /* '<S374>/Multiport Switch' */
uint16_T EST_BtfQualifierByBeta;       /* '<S374>/Shift Arithmetic' */
uint16_T EST_BtfQualifierByEna;        /* '<S404>/Add2' */
uint16_T EST_BtfQualifierByHdr;        /* '<S346>/Add2' */
real32_T EST_CoeffA11StateSpace;       /* '<S407>/Subtract' */

/* Single Track State Space Coefficitents A11 */
real32_T EST_CoeffA12StateSpace;       /* '<S408>/Subtract' */

/* Single Track State Space Coefficitents A12 */
real32_T EST_CoeffA21StateSpace;       /* '<S409>/Divide' */

/* Single Track State Space Coefficitents A21 */
real32_T EST_CoeffA22StateSpace;       /* '<S410>/Divide' */

/* Single Track State Space Coefficitents A22 */
real32_T EST_CoeffAXStateSpace[2];     /* '<S385>/Mul' */
real32_T EST_CoeffB11StateSpace;       /* '<S411>/Product' */
real32_T EST_CoeffB21StateSpace;       /* '<S412>/Divide' */
real32_T EST_CoeffL11Pobs;             /* '<S311>/1-D Lookup Table' */

/* First element of first column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL11Sobs;             /* '<S387>/1-D Lookup Table' */
real32_T EST_CoeffL12Pobs;             /* '<S311>/1-D Lookup Table4' */

/* First element of second column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL13Pobs;             /* '<S311>/1-D Lookup Table9' */

/* First element of third column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL21Pobs;             /* '<S311>/1-D Lookup Table1' */

/* Second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL21Sobs;             /* '<S387>/1-D Lookup Table1' */
real32_T EST_CoeffL22Pobs;             /* '<S311>/1-D Lookup Table5' */

/* Second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL23Pobs;             /* '<S311>/1-D Lookup Table10' */

/* Second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL31Pobs;             /* '<S311>/1-D Lookup Table2' */

/* Third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL32Pobs;             /* '<S311>/1-D Lookup Table6' */

/* Third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL33Pobs;             /* '<S311>/1-D Lookup Table11' */

/* Third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL41Pobs;             /* '<S311>/1-D Lookup Table3' */

/* Fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL42Pobs;             /* '<S311>/1-D Lookup Table7' */

/* Fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffL43Pobs;             /* '<S311>/1-D Lookup Table12' */

/* Fourth element of third column of gain matrix of the Luenberger observer to estimate the course angle */
real32_T EST_CoeffLPobs[12];           /* '<S311>/FeedbackMatrixL' */
real32_T EST_CoeffLYPObs[4];           /* '<S310>/Mul' */
real32_T EST_CoeffLYStateSpace[2];     /* '<S387>/Product' */
real32_T EST_CrvPiObsCrvFlt;           /* '<S392>/Add' */

/* Target curvature after low pass filter for plant observer */
real32_T EST_CrvPlObsIn;               /* '<S389>/Multiport Switch' */

/* Curvature input for plant observer */
real32_T EST_CurSteerAngle;            /* '<S73>/Data Type Conversion' */

/* Current steer angle of the front wheels */
real32_T EST_DThetaDotPobs;            /* '<S312>/Signal Conversion2' */
real32_T EST_DYDotPobs;                /* '<S312>/Signal Conversion3' */
real32_T EST_DeltaYPlObsIn;            /* '<S390>/Multiport Switch' */
real32_T EST_DistFromCgToGud;          /* '<S413>/Add' */

/* Distance between Center of Gravity and point to be guided */
real32_T EST_DistPObsDY;               /* '<S368>/Multiport Switch' */
real32_T EST_DistPObsDYFreeze;         /* '<S368>/Multiport Switch1' */
real32_T EST_DistPObsDYGrdnt;          /* '<S366>/Multiport Switch' */
real32_T EST_DistPObsDYGrdntThd;       /* '<S366>/1-D Lookup Table1' */
real32_T EST_DistPObsDYSat;            /* '<S371>/Switch2' */
real32_T EST_DistPObsDYSel;            /* '<S366>/Multiport Switch1' */
real32_T EST_DistPObsDYThd;            /* '<S368>/1-D Lookup Table' */
real32_T EST_DistPobsDYGrdntRaw;       /* '<S369>/Multiport Switch1' */
real32_T EST_DistYDevByGrdntLmt1;      /* '<S398>/Multiport Switch1' */

/* Lateral distance deviation after first gradient limit */
real32_T EST_DistYDevStep;             /* '<S395>/Subtract3' */
real32_T EST_DistYDevTrajFromCur;      /* '<S390>/Subtract' */

/* Deviation between Trajectory distance Y and current distance Y */
real32_T EST_DlyCurSteerAngle;         /* '<S386>/Multiport_Switch' */

/* Current steering angle after delay */
boolean_T EST_EnaActvtGrdntLmt1;       /* '<S395>/GreaterThan3' */

/* Enable flag for first gradient limit activation */
boolean_T EST_EnaActvtGrdntLmt2;       /* '<S396>/AND' */

/* Enable flag for second gradient limit activation */
boolean_T EST_EnaBetaSatSObs;          /* '<S374>/GreaterThan' */
boolean_T EST_EnaByMeanHdr;            /* '<S346>/GreaterThan1' */
boolean_T EST_EnaByMulHdrPerc;         /* '<S346>/GreaterThan' */
boolean_T EST_EnaCntrlByTgq;           /* '<S403>/Equal1' */
boolean_T EST_EnaFreezeByTgq;          /* '<S403>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
boolean_T EST_EnaLmt2ByDistY;          /* '<S396>/OR' */

/* Enable flag for second gradient limt */
boolean_T EST_EnaLmtByDistY;           /* '<S395>/GreaterThan' */
boolean_T EST_EnaPObsDThetaLmt0;       /* '<S355>/AND' */
boolean_T EST_EnaPObsDThetaLmt1;       /* '<S356>/GreaterThan3' */
boolean_T EST_EnaPObsDThetaLmt2;       /* '<S357>/AND' */
boolean_T EST_EnaPObsDThetaRst1;       /* '<S356>/NOT' */
boolean_T EST_EnaPObsDThetaRst2;       /* '<S357>/OR1' */
boolean_T EST_EnaPObsDThetaSat;        /* '<S354>/GreaterThan' */
boolean_T EST_EnaPObsDYGrdnt;          /* '<S366>/AND' */
boolean_T EST_EnaPObsDYGrdntRaw;       /* '<S366>/GreaterThan3' */
boolean_T EST_EnaPObsDYSat;            /* '<S368>/GreaterThan' */
boolean_T EST_EnaResetByTgq;           /* '<S403>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
real32_T EST_ErrVehYawRate;            /* '<S387>/Subtract' */
real32_T EST_EstBetaPobs;              /* '<S318>/Switch2' */
real32_T EST_EstBetaSObs;              /* '<S379>/Switch2' */
real32_T EST_EstDThetaPobs;            /* '<S320>/Switch2' */
real32_T EST_EstDYPobs;                /* '<S321>/Switch2' */
real32_T EST_EstPsiDotPobs;            /* '<S319>/Switch2' */
real32_T EST_EstPsiDotSObs;            /* '<S382>/Switch2' */
real32_T EST_FacDThetaWghtHdrSel;      /* '<S349>/Multiport Switch3' */
real32_T EST_FacDYWghtHdrSel;          /* '<S349>/Multiport Switch4' */
real32_T EST_FltDThetaDotPObs;         /* '<S341>/Multiport Switch' */
real32_T EST_FltDYDotPObs;             /* '<S336>/Multiport Switch' */
real32_T EST_HdrPercByDY;              /* '<S335>/Multiport Switch1' */
real32_T EST_HdrPercByTheta;           /* '<S340>/Multiport Switch1' */
real32_T EST_HldBetaSObs;              /* '<S374>/Multiport Switch1' */
real32_T EST_LmtBetaSObs;              /* '<S388>/Switch2' */
real32_T EST_LmtHdrPercByDY;           /* '<S337>/Switch2' */
real32_T EST_LmtHdrPercByTheta;        /* '<S342>/Switch2' */
real32_T EST_LmtVehVelX;               /* '<S406>/Multiport Switch1' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
real32_T EST_MeanHdrPerc;              /* '<S346>/Divide1' */
uint8_T EST_ModeSelParHdr;             /* '<S345>/Multiport Switch' */
real32_T EST_MulHdrPerc;               /* '<S346>/Divide' */
real32_T EST_Psi2DotPobs;              /* '<S312>/Signal Conversion1' */
real32_T EST_Psi2DotSObs;              /* '<S375>/Signal Conversion1' */
real32_T EST_RatioSteerGear;           /* '<S391>/1-D Lookup Table' */
real32_T EST_RawBetaSObs;              /* '<S374>/Multiport Switch2' */
real32_T EST_RawEstBetaPobs;           /* '<S323>/Init' */
real32_T EST_RawEstDThetaPobs;         /* '<S327>/Init' */
real32_T EST_RawEstDYPobs;             /* '<S329>/Init' */
real32_T EST_RawEstPsiDotPobs;         /* '<S325>/Init' */
real32_T EST_RawFltDThetaDotPObs;      /* '<S343>/Add' */
real32_T EST_RawFltDYDotPObs;          /* '<S338>/Add' */
real32_T EST_RawHdrPercByDY;           /* '<S335>/Product' */
real32_T EST_RawHdrPercByTheta;        /* '<S340>/Product' */
real32_T EST_ThdBetaSatSObs;           /* '<S374>/1-D Lookup Table' */
real32_T EST_ThdMeanHdrSel;            /* '<S348>/Multiport Switch1' */
real32_T EST_ThdMulHdrSel;             /* '<S348>/Multiport Switch2' */
real32_T FFC_HldReqFfcCrv;             /* '<S80>/Unit Delay1' */

/* Curvature control signal part of the feedforward controller */
real32_T FFC_ReqFfcCrv;                /* '<S80>/Multiport Switch2' */

/* Curvature control signal part of the feedforward controller */
uint8_T LGC_ActiveLgcParamSet_nu;      /* '<S81>/Data Type Conversion12' */

/*
   Active LGC Paramter Set:
   1: Lane Centering
   2: Object Following
   3: Safety Function
 */
boolean_T LGC_CacIntReset_nu;          /* '<S81>/Data Type Conversion16' */
boolean_T LGC_CacPT1Reset_nu;          /* '<S81>/Data Type Conversion14' */
real32_T LGC_CdcCmd_rad;               /* '<S81>/Data Type Conversion4' */
real32_T LGC_Cmpn2DotLaDmcCas;         /* '<S466>/Multiport Switch' */
real32_T LGC_Cmpn2DotLaDmcCdc;         /* '<S506>/Multiport Switch' */
real32_T LGC_Cmpn2DotLaDmcLdc;         /* '<S548>/Multiport Switch' */
real32_T LGC_CmpnDotLaDmcCas;          /* '<S475>/Switch2' */
real32_T LGC_CmpnDotLaDmcCdc;          /* '<S515>/Switch2' */
real32_T LGC_CmpnDotLaDmcLdc;          /* '<S557>/Switch2' */
real32_T LGC_CmpnLaDmcCas;             /* '<S472>/Switch2' */
real32_T LGC_CmpnLaDmcCdc;             /* '<S512>/Switch2' */
real32_T LGC_CmpnLaDmcLdc;             /* '<S554>/Switch2' */
real32_T LGC_CoeffDGainCac;            /* '<S426>/Multiport Switch3' */
real32_T LGC_CoeffDGainLcCac;          /* '<S426>/Y_TCTLGC_LdcDGain_radspm1' */
real32_T LGC_CoeffDGainLcLdc;          /* '<S430>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
real32_T LGC_CoeffDGainLdc;            /* '<S430>/Multiport Switch3' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
real32_T LGC_CoeffDGainOfCac;          /* '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */
real32_T LGC_CoeffDGainOfLdc;          /* '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
real32_T LGC_CoeffDGainSfCac;          /* '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */
real32_T LGC_CoeffDGainSfLdc;          /* '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
real32_T LGC_CoeffIGainCac;            /* '<S427>/Multiport Switch1' */
real32_T LGC_CoeffIGainLcCac;          /* '<S427>/Y_TCTLGC_LdcIGain_radpsm' */
real32_T LGC_CoeffIGainLcLdc;          /* '<S431>/Y_TCTLGC_LdcIGain_radpsm' */
real32_T LGC_CoeffIGainLdc;            /* '<S431>/Multiport Switch1' */
real32_T LGC_CoeffIGainOfCac;          /* '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */
real32_T LGC_CoeffIGainOfLdc;          /* '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */
real32_T LGC_CoeffIGainSfCac;          /* '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */
real32_T LGC_CoeffIGainSfLdc;          /* '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */
real32_T LGC_CoeffMainPGainCac;        /* '<S428>/Multiport Switch7' */
real32_T LGC_CoeffMainPGainLdc;        /* '<S432>/Multiport Switch7' */
real32_T LGC_CoeffNumS0LaDmc;          /* '<S425>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
real32_T LGC_CoeffNumS1LaDmc;          /* '<S425>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
real32_T LGC_CoeffPGainByCrvCac;      /* '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */
real32_T LGC_CoeffPGainByCrvLdc;      /* '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */
real32_T LGC_CoeffPGainCac;            /* '<S428>/Multiport Switch2' */
real32_T LGC_CoeffPGainLcCac;          /* '<S428>/Y_TCTLGC_LdcP_radpm1' */
real32_T LGC_CoeffPGainLcLdc;          /* '<S432>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
real32_T LGC_CoeffPGainLdc;            /* '<S432>/Multiport Switch2' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_CoeffPGainOfCac;          /* '<S428>/Y_TCTLGC_LdcOfP_radpm1' */
real32_T LGC_CoeffPGainOfLdc;          /* '<S432>/Y_TCTLGC_LdcOfP_radpm1' */

/* Lateral Deviation Controller P-Gain for object following mode */
real32_T LGC_CoeffPGainSfCac;          /* '<S428>/Y_TCTLGC_LdcSfP_radpm1' */
real32_T LGC_CoeffPGainSfLdc;          /* '<S432>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
real32_T LGC_CoeffPT1GainCac;          /* '<S429>/Multiport Switch3' */
real32_T LGC_CoeffPT1GainLcCac;        /* '<S429>/Y_TCTLGC_LdcDGain_radspm' */
real32_T LGC_CoeffPT1GainLcLdc;        /* '<S433>/Y_TCTLGC_LdcDGain_radspm' */
real32_T LGC_CoeffPT1GainLdc;          /* '<S433>/Multiport Switch3' */
real32_T LGC_CoeffPT1GainOfCac;        /* '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */
real32_T LGC_CoeffPT1GainOfLdc;        /* '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */
real32_T LGC_CoeffPT1GainSfCac;        /* '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */
real32_T LGC_CoeffPT1GainSfLdc;        /* '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */
real32_T LGC_CoeffPole1LaDmc;          /* '<S425>/1-D Lookup Table2' */

/* Pole 1 of the approximated LaDMC transfer function */
real32_T LGC_CoeffPole2LaDmc;          /* '<S425>/1-D Lookup Table3' */

/* Pole 2 of the approximated LaDMC transfer function */
real32_T LGC_CrvReqBAC_1pm;            /* '<S565>/Switch' */
real32_T LGC_CrvReqDte_1pm;            /* '<S81>/Data Type Conversion5' */

/*
   Required curvature by disturbance compensator
 */
real32_T LGC_CrvReqFfcFrz_1pm;         /* '<S570>/Switch2' */
real32_T LGC_CrvReqFfcGrdLimT1_1pm;    /* '<S576>/Switch' */
real32_T LGC_CrvReqFfcGrdLimT2_1pm;    /* '<S578>/Add1' */
real32_T LGC_CrvReqFfcGrdLim_1pm;      /* '<S577>/Switch2' */
real32_T LGC_CtrlCrv_1pm;              /* '<S564>/Divide' */
real32_T LGC_CtrlCrv_DE_1pm;           /* '<S419>/Switch' */
real32_T LGC_CtrlErrHeadAglCrtd_rad;   /* '<S81>/Data Type Conversion' */
real32_T LGC_CtrlErrMainPGain;         /* '<S540>/Mul4' */

/* UNDEFINED */
real32_T LGC_CtrlErrMainPGainCas;      /* '<S458>/Mul4' */

/* UNDEFINED */
real32_T LGC_CtrlErrMainPGainCdc;      /* '<S498>/Mul4' */

/* UNDEFINED */
real32_T LGC_DeltaByBnkAglComp_deg;    /* '<S588>/Divide' */
real32_T LGC_DeltaFBAC_deg;            /* '<S81>/Data Type Conversion17' */
real32_T LGC_DeltaFCmdCdc;             /* '<S484>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_DeltaFCmdDC_deg;          /* '<S587>/Divide' */
real32_T LGC_DeltaFCmdFFC_deg;         /* '<S81>/Data Type Conversion10' */
real32_T LGC_DeltaFCmdUnlimited_deg;   /* '<S586>/Divide' */
real32_T LGC_DeltaFCmd_deg;            /* '<S81>/Data Type Conversion3' */

/*
   Steering Angle Command for DMC
 */
real32_T LGC_DeltaFCmd_rad;            /* '<S439>/Sum' */
real32_T LGC_DeltaFDGainCas;           /* '<S446>/Multiport Switch9' */

/* UNDEFINED */
real32_T LGC_DeltaFDGainCdc;           /* '<S486>/Multiport Switch9' */

/* UNDEFINED */
real32_T LGC_DeltaFDGainLdc;           /* '<S526>/Multiport Switch9' */

/* UNDEFINED */
real32_T LGC_DeltaFIGainCas;           /* '<S452>/Switch2' */

/* UNDEFINED */
real32_T LGC_DeltaFIGainCdc;           /* '<S492>/Switch2' */

/* UNDEFINED */
real32_T LGC_DeltaFIGainLdc;           /* '<S532>/Switch2' */

/* UNDEFINED */
real32_T LGC_DeltaFPGainCas;           /* '<S458>/Mul8' */

/* UNDEFINED */
real32_T LGC_DeltaFPGainCdc;           /* '<S498>/Mul8' */

/* UNDEFINED */
real32_T LGC_DeltaFPGainLdc;           /* '<S540>/Mul8' */

/* UNDEFINED */
real32_T LGC_DeltaFPT1GainCas;         /* '<S448>/Multiport Switch8' */

/* UNDEFINED */
real32_T LGC_DeltaFPT1GainCdc;         /* '<S488>/Multiport Switch8' */

/* UNDEFINED */
real32_T LGC_DeltaFPT1GainLdc;         /* '<S535>/Multiport Switch1' */

/* UNDEFINED */
uint8_T LGC_EnaActObjFollow;           /* '<S422>/Data Type Conversion1' */

/* Enable flag for activating object following */
uint8_T LGC_EnaActSafetyFcn;           /* '<S422>/Data Type Conversion' */

/* Enable flag for activating safety function */
boolean_T LGC_EnaCntrlByTgq;           /* '<S417>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
boolean_T LGC_EnaFreezeByTgq;          /* '<S417>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
boolean_T LGC_EnaModeChangeCas;        /* '<S457>/NotEqual' */

/* UNDEFINED */
boolean_T LGC_EnaModeChangeCdc;        /* '<S497>/NotEqual' */

/* UNDEFINED */
boolean_T LGC_EnaModeChangeDtct;       /* '<S539>/NotEqual' */
boolean_T LGC_EnaPGainGrdLmtCas;       /* '<S457>/OR3' */

/* UNDEFINED */
boolean_T LGC_EnaPGainGrdLmtCdc;       /* '<S497>/OR3' */

/* UNDEFINED */
boolean_T LGC_EnaPGainGrdLmtLdc;       /* '<S539>/OR3' */
int8_T LGC_EnaPGainGrdSignCas;         /* '<S457>/Multiport Switch7' */

/* UNDEFINED */
int8_T LGC_EnaPGainGrdSignCdc;         /* '<S497>/Multiport Switch7' */

/* UNDEFINED */
real32_T LGC_EnaPGainGrdSignLdc;       /* '<S539>/Multiport Switch7' */
boolean_T LGC_EnaPGainGrdThdCas;       /* '<S457>/GreaterThan' */

/* UNDEFINED */
boolean_T LGC_EnaPGainGrdThdCdc;       /* '<S497>/GreaterThan' */

/* UNDEFINED */
boolean_T LGC_EnaPGainGrdThdLdc;       /* '<S539>/GreaterThan' */
boolean_T LGC_EnaResetByTgq;           /* '<S417>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
boolean_T LGC_EnaRstByDistY;           /* '<S435>/AND1' */
boolean_T LGC_EnaRstByStandStill;      /* '<S435>/Less Than' */
boolean_T LGC_EnaRstByTrq;             /* '<S435>/GreaterThan2' */
boolean_T LGC_EnaRstIntCac;            /* '<S434>/OR3' */
boolean_T LGC_EnaRstIntLdc;            /* '<S434>/OR5' */
boolean_T LGC_EnaRstPT1Cac;            /* '<S434>/OR2' */
boolean_T LGC_EnaRstPT1Ldc;            /* '<S434>/OR4' */
boolean_T LGC_EnableCtrl_nu;           /* '<S81>/Data Type Conversion7' */

/*
   Enalbe flag for control by trajectory guidance qualifier output
 */
real32_T LGC_ErrCourse2DotCas;         /* '<S465>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_ErrCourse2DotCdc;         /* '<S505>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_ErrCourseDotCas;          /* '<S464>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_ErrCourseDotCdc;          /* '<S504>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_ErrCtrlCourseCas;         /* '<S461>/Multiport Switch5' */

/* UNDEFINED */
real32_T LGC_ErrCtrlCourseCdc;         /* '<S501>/Multiport Switch5' */

/* UNDEFINED */
real32_T LGC_ErrCtrlDistY;             /* '<S543>/Multiport Switch5' */

/* UNDEFINED */
real32_T LGC_ErrDistY2Dot;             /* '<S547>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_ErrDistYDot;              /* '<S546>/Multiport Switch' */

/* UNDEFINED */
real32_T LGC_FFCrv_1pm;                /* '<S568>/Switch4' */
real32_T LGC_FltErrCourseCas;          /* '<S463>/Multiport Switch4' */

/* UNDEFINED */
real32_T LGC_FltErrCourseCdc;          /* '<S503>/Multiport Switch4' */

/* UNDEFINED */
real32_T LGC_FltErrCourseDotCas;       /* '<S468>/Add' */

/* UNDEFINED */
real32_T LGC_FltErrCourseDotCdc;       /* '<S508>/Add' */

/* UNDEFINED */
real32_T LGC_FltPT1YErr_met;           /* '<S545>/Multiport Switch4' */

/* UNDEFINED */
real32_T LGC_FltRawErrDistYDot;        /* '<S550>/Add' */

/* UNDEFINED */
real32_T LGC_HldReqDeltaF;             /* '<S590>/Multiport Switch2' */

/* Steering Angle Command for DMC */
boolean_T LGC_Hold_nu;                 /* '<S81>/Data Type Conversion6' */

/*
   Enalbe flag for freeze by trajectory guidance qualifier output
 */
real32_T LGC_LdcAloneICmd_rad;         /* '<S524>/Multiport Switch' */
real32_T LGC_LdcCmd_rad;               /* '<S444>/Multiport Switch' */
boolean_T LGC_LdcIntReset_nu;          /* '<S81>/Data Type Conversion15' */
boolean_T LGC_LdcPT1Reset_nu;          /* '<S81>/Data Type Conversion13' */
real32_T LGC_LmtCoeffPGainCas;         /* '<S458>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_LmtCoeffPGainCdc;         /* '<S498>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_LmtCoeffPGainLdc;         /* '<S540>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_LmtReqDeltaF;             /* '<S592>/Switch2' */

/* Steering Angle Command for DMC */
real32_T LGC_LmtSelReqDeltaF;          /* '<S589>/Multiport Switch3' */

/* Steering Angle Command for DMC */
real32_T LGC_MaxReqDeltaF;             /* '<S589>/Product' */

/* Steering Angle Command for DMC */
real32_T LGC_RawCmpn2DotLaDmcCas;      /* '<S466>/Sum2' */
real32_T LGC_RawCmpn2DotLaDmcCdc;      /* '<S506>/Sum2' */
real32_T LGC_RawCmpn2DotLaDmcLdc;      /* '<S548>/Sum2' */
real32_T LGC_RawErrCourseDotCas;       /* '<S464>/Divide' */

/* UNDEFINED */
real32_T LGC_RawErrCourseDotCdc;       /* '<S504>/Divide' */

/* UNDEFINED */
real32_T LGC_RawErrDistYDot;           /* '<S546>/Divide' */

/* UNDEFINED */
real32_T LGC_RawFfcDeltaF;             /* '<S574>/Divide1' */

/* UNDEFINED */
real32_T LGC_RawFltErrCourseCas;       /* '<S478>/Add' */

/* UNDEFINED */
real32_T LGC_RawFltErrCourseCdc;       /* '<S518>/Add' */

/* UNDEFINED */
real32_T LGC_RawFltErrCtrlDistY;       /* '<S560>/Add' */

/* UNDEFINED */
real32_T LGC_RawLmtCoeffPGainCas;      /* '<S459>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_RawLmtCoeffPGainCdc;      /* '<S499>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_RawLmtCoeffPGainLdc;      /* '<S541>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
real32_T LGC_ReqDeltaF;                /* '<S590>/Multiport Switch' */

/* Steering Angle Command for DMC */
boolean_T LGC_Reset_nu;                /* '<S81>/Data Type Conversion9' */

/*
   Enalbe flag for reset by trajectory guidance qualifier output
 */
uint8_T LGC_SafetyFunctionActive_nu;   /* '<S81>/Data Type Conversion11' */

/*
   Enable flag for activating safety function
 */
uint8_T LGC_StActParSet;               /* '<S422>/Data Type Conversion2' */

/* Active LGC Paramter Set:
   1: Lane Centering
   2: Object Following
   3: Safety Function */
real32_T LGC_SumCrvReqFbFrz_1pm;       /* '<S569>/Switch2' */
real32_T LGC_SumCrvReqFbGrdLim_1pm;    /* '<S580>/Switch2' */
real32_T LGC_SumCrvReqFbSatLim_1pm;    /* '<S582>/Switch1' */
real32_T LGC_SumCrvReqFb_1pm;          /* '<S563>/Sum2' */
real32_T LGC_TgtCrv_DENoLatSlp_1pm;    /* '<S81>/Constant' */
real32_T LGC_TgtCrv_DE_1pm;            /* '<S567>/Switch4' */
real32_T LGC_TgtCrv_NoDE_1pm;          /* '<S81>/Constant1' */
real32_T LGC_TimeDT1Cac;               /* '<S426>/Multiport Switch4' */
real32_T LGC_TimeDT1LcCac;             /* '<S426>/Y_TCTLGC_LdcDT1_sec' */
real32_T LGC_TimeDT1LcLdc;             /* '<S430>/Y_TCTLGC_LdcDT1_sec' */
real32_T LGC_TimeDT1Ldc;               /* '<S430>/Multiport Switch4' */
real32_T LGC_TimeDT1OfCac;             /* '<S426>/Y_TCTLGC_LdcOfDT1_sec' */
real32_T LGC_TimeDT1OfLdc;             /* '<S430>/Y_TCTLGC_LdcOfDT1_sec' */
real32_T LGC_TimeDT1SfCac;             /* '<S426>/Y_TCTLGC_LdcSfDT1_sec' */
real32_T LGC_TimeDT1SfLdc;             /* '<S430>/Y_TCTLGC_LdcSfDT1_sec' */
real32_T LGC_TimeFltErrCourse;         /* '<S463>/Y_TCTLGC_PT1YErrTime_sec' */
real32_T LGC_TimeFltErrCourseCdc;      /* '<S503>/Y_TCTLGC_PT1YErrTime_sec' */
real32_T LGC_TimeFltErrDistY;          /* '<S545>/Y_TCTLGC_PT1YErrTime_sec' */
real32_T LGC_TimePT1Cac;               /* '<S429>/Multiport Switch4' */
real32_T LGC_TimePT1DeltaFCmd;      /* '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */
real32_T LGC_TimePT1LcCac;             /* '<S429>/Y_TCTLGC_LdcDT1_sec' */
real32_T LGC_TimePT1LcLdc;             /* '<S433>/Y_TCTLGC_LdcDT1_sec' */
real32_T LGC_TimePT1Ldc;               /* '<S433>/Multiport Switch4' */
real32_T LGC_TimePT1OfCac;             /* '<S429>/Y_TCTLGC_LdcOfDT1_sec' */
real32_T LGC_TimePT1OfLdc;             /* '<S433>/Y_TCTLGC_LdcOfDT1_sec' */
real32_T LGC_TimePT1SfCac;             /* '<S429>/Y_TCTLGC_LdcSfDT1_sec' */
real32_T LGC_TimePT1SfLdc;             /* '<S433>/Y_TCTLGC_LdcSfDT1_sec' */
real32_T LQR_DeltaF_Cmd_rad;           /* '<S82>/Add' */
real32_T LQR_DeltaF_Lead_Cmd_rad;      /* '<S601>/MATLAB Function' */
real32_T LQR_DeltaF_feedback_rad;      /* '<S595>/Add' */
real32_T LQR_DeltaF_feedforward_rad;   /* '<S82>/Product' */
boolean_T LQR_EnaCntrlByTgq;           /* '<S597>/Equal1' */
boolean_T LQR_EnaFreezeByTgq;          /* '<S597>/Equal' */
boolean_T LQR_EnaResetByTgq;           /* '<S597>/OR' */
real32_T LQR_FltDeltaF_Cmd_rad;        /* '<S603>/Switch' */
real32_T LQR_I_term_rad;               /* '<S614>/Switch2' */
real32_T LQR_KappaFlt;                 /* '<S599>/Switch' */
real32_T LQR_MatK_k1;                  /* '<S607>/1-D Lookup Table4' */
real32_T LQR_MatK_k2;                  /* '<S607>/1-D Lookup Table1' */
real32_T LQR_MatK_k3;                  /* '<S607>/1-D Lookup Table2' */
real32_T LQR_MatK_k4;                  /* '<S607>/1-D Lookup Table3' */
real32_T LQR_ReqDeltaF_Limit_deg;      /* '<S628>/Multiport Switch' */
real32_T LQR_YawrateFlt;               /* '<S632>/Switch' */
real32_T LQR_e1_contribution;          /* '<S595>/Product' */
real32_T LQR_e1dot_contribution;       /* '<S595>/Product1' */
real32_T LQR_e2_contribution;          /* '<S595>/Product2' */
real32_T LQR_e2dot_contribution;       /* '<S595>/Product3' */
real32_T LQR_heading_error;            /* '<S73>/Data Type Conversion12' */
real32_T LQR_heading_error_rate;       /* '<S604>/Subtract' */
real32_T LQR_lateral_error;            /* '<S73>/Data Type Conversion11' */
real32_T LQR_lateral_error_rate;       /* '<S604>/Switch' */
real32_T LQR_num_iteration;            /* '<S608>/Data Type Conversion3' */
real32_T LQR_ref_heading_rate;         /* '<S604>/Product4' */
real32_T LQR_yawrate_term;             /* '<S73>/Data Type Conversion15' */
real32_T TCTI_ActualTrqEPS;            /* '<Root>/TCTI_ActualTrqEPS' */
real32_T TCTI_NegReqTrajHeadTpl;       /* '<S92>/Neg4' */

/* negative S_TPLFBT_CurHeading_rad */
real32_T TCTI_RoadBankAngle;           /* '<Root>/TCTI_RoadBankAngle' */
boolean_T TC_EnaUnplauUnitDelay_bool;  /* '<S102>/Unit Delay' */
boolean_T TC_Freeze1RSFlipFlop_bool;   /* '<S111>/Unit Delay' */
boolean_T TC_Freeze2RSFlipFlop_bool;   /* '<S107>/Unit Delay' */
boolean_T TC_FreezeRSFlipFlop_bool;    /* '<S110>/Unit Delay' */
boolean_T TC_HoldWarnRSFlipFlop_bool;  /* '<S130>/Unit Delay' */
real32_T deadZone_gainkT;              /* '<S595>/MATLAB Function' */
real32_T deadZone_weightedError;       /* '<S595>/MATLAB Function' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/* Model step function */
void TJATCT_step(void)
{
  static const int8_T g[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  real_T rtb_Product;
  int32_T MatA_T_0;
  int32_T b_j;
  int32_T c_ix;
  int32_T c_k;
  int32_T d;
  int32_T ijA;
  int32_T ix;
  int32_T jA;
  real32_T MatA_T[16];
  real32_T P_next[16];
  real32_T P_next_tmp[16];
  real32_T P_next_tmp_2[16];
  real32_T P_next_tmp_3[16];
  real32_T rtb_y_bw[16];
  real32_T rtb_y_h2[16];
  real32_T P_next_tmp_0[4];
  real32_T P_next_tmp_1[4];
  real32_T rtb_Add1_h[4];
  real32_T rtb_Add1_m[4];
  real32_T rtb_y_dr[4];
  real32_T DTE_DeltaFRte_tmp;
  real32_T DTE_RawReqDeltaByBnkAgl_tmp;
  real32_T DTE_RawReqDeltaByBnkAgl_tmp_0;
  real32_T LGC_RawErrCourseDotCdc_tmp;
  real32_T gm;
  real32_T rtb_Add1_ge;
  real32_T rtb_Divide_bb;
  real32_T rtb_Divide_bo;
  real32_T rtb_Divide_eu;
  real32_T rtb_Divide_hw;
  real32_T rtb_Divide_l4;
  real32_T rtb_Divide_l_tmp;
  real32_T rtb_Gain;
  real32_T rtb_Gain1;
  real32_T rtb_LGC_StActParSet;
  real32_T rtb_MeasDeviationVector_idx_1;
  real32_T rtb_MultiportSwitch2_a;
  real32_T rtb_Neg_i;
  real32_T rtb_Product1_c;
  real32_T rtb_Product1_l5;
  real32_T rtb_Product1_p4;
  real32_T rtb_Product_dv;
  real32_T rtb_Product_km;
  real32_T rtb_Sum_db;
  real32_T rtb_Sum_kv;
  real32_T rtb_Sum_lu;
  real32_T rtb_Switch2_m4;
  real32_T rtb_Switch7;
  real32_T rtb_Switch_gf;
  real32_T rtb_UnitDelay2_p;
  real32_T rtb_UnitDelay3_a;
  real32_T rtb_UnitDelay5;
  real32_T rtb_UnitDelay6;
  real32_T rtb_Unit_Delay2_j;
  real32_T rtb_Unit_Delay2_o;
  real32_T rtb_Unit_Delay3;
  real32_T rtb_Unit_Delay3_ia;
  real32_T smax;
  uint32_T tmp_1;
  int8_T ipiv[4];
  int8_T ipiv_0;
  uint8_T num_iteration;
  uint8_T tmp;
  uint8_T tmp_0;
  boolean_T rtb_AND2;
  boolean_T rtb_AND_od;
  boolean_T rtb_Equal3;
  boolean_T rtb_Equal3_b;
  boolean_T rtb_Equal3_c;
  boolean_T rtb_Equal3_gh;
  boolean_T rtb_Equal3_m;
  boolean_T rtb_Equal3_n;
  boolean_T rtb_Equal4;
  boolean_T rtb_GreaterThan1_bj;
  boolean_T rtb_LGC_EnaActObjFollow;
  boolean_T rtb_LGC_EnaActSafetyFcn;
  boolean_T rtb_LQR_I_EnaResetByTgq;
  boolean_T rtb_LowerRelop1_p0;
  boolean_T rtb_OR_fh;
  boolean_T rtb_OR_km_tmp;
  boolean_T tmp_2;

  /* MATLAB Function: '<S608>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Outport39'
   *  Inport: '<Root>/Outport40'
   *  Inport: '<Root>/Outport41'
   *  Inport: '<Root>/Outport42'
   */
  memset(&rtb_y_bw[0], 0, sizeof(real32_T) << 4U);
  rtb_y_bw[0] = Test_CoeffMainPGainCac;
  rtb_y_bw[5] = Test_CoeffPGainCac;
  rtb_y_bw[10] = Test_CoeffIGainCac;
  rtb_y_bw[15] = Test_CoeffDGainCac;

  /* Gain: '<S608>/Gain' incorporates:
   *  Constant: '<S608>/Parameter5'
   */
  rtb_Gain = 0.5F * EST_CorStiffFrontAxle_P;

  /* MATLAB Function: '<S608>/MATLAB Function2' incorporates:
   *  Constant: '<S608>/Parameter'
   *  Constant: '<S608>/Parameter2'
   *  Constant: '<S608>/Parameter4'
   *  Constant: '<S82>/Parameter4'
   *  MATLAB Function: '<S608>/MATLAB Function'
   */
  rtb_y_dr[0] = 0.0F;
  rtb_y_dr[1] = 2.0F * rtb_Gain / EST_MassVehicle_P * LGC_TimeSysCycle_P;
  rtb_y_dr[2] = 0.0F;
  smax = 2.0F * rtb_Gain * EST_DistCogToFrontAxle_P;
  rtb_y_dr[3] = smax / EST_InertiaVehicle_P * LGC_TimeSysCycle_P;

  /* MinMax: '<S621>/Max' incorporates:
   *  Constant: '<S621>/Constant11'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  rtb_Product = fmax(TCTI_VehicleVelX, 3.0);

  /* Gain: '<S608>/Gain1' incorporates:
   *  Constant: '<S608>/Parameter6'
   */
  rtb_Gain1 = 0.5F * EST_CorStiffRearAxle_P;

  /* MATLAB Function: '<S608>/MATLAB Function' incorporates:
   *  Constant: '<S608>/Parameter'
   *  Constant: '<S608>/Parameter1'
   *  Constant: '<S608>/Parameter2'
   *  Constant: '<S608>/Parameter4'
   *  Constant: '<S82>/Parameter4'
   *  DataTypeConversion: '<S608>/Data Type Conversion13'
   */
  rtb_y_h2[1] = 0.0F;
  rtb_Unit_Delay3 = 2.0F * rtb_Gain + 2.0F * rtb_Gain1;
  rtb_Divide_l4 = EST_MassVehicle_P * (real32_T)rtb_Product;
  rtb_y_h2[5] = -rtb_Unit_Delay3 / rtb_Divide_l4;
  rtb_y_h2[9] = rtb_Unit_Delay3 / EST_MassVehicle_P;
  rtb_Unit_Delay3 = 2.0F * rtb_Gain1 * EST_DistCogToRearAxle_P;
  rtb_y_h2[13] = (-2.0F * rtb_Gain * EST_DistCogToFrontAxle_P + rtb_Unit_Delay3)
    / rtb_Divide_l4;
  rtb_y_h2[0] = 0.0F;
  rtb_y_h2[2] = 0.0F;
  rtb_y_h2[4] = 1.0F;
  rtb_y_h2[6] = 0.0F;
  rtb_y_h2[8] = 0.0F;
  rtb_y_h2[10] = 0.0F;
  rtb_y_h2[12] = 0.0F;
  rtb_y_h2[14] = 1.0F;
  rtb_y_h2[3] = 0.0F;
  rtb_Unit_Delay3 = smax - rtb_Unit_Delay3;
  rtb_Divide_l4 = EST_InertiaVehicle_P * (real32_T)rtb_Product;
  rtb_y_h2[7] = -rtb_Unit_Delay3 / rtb_Divide_l4;
  rtb_y_h2[11] = rtb_Unit_Delay3 / EST_InertiaVehicle_P;
  rtb_y_h2[15] = -(2.0F * rtb_Gain * (EST_DistCogToFrontAxle_P *
    EST_DistCogToFrontAxle_P) + 2.0F * rtb_Gain1 * (EST_DistCogToRearAxle_P *
    EST_DistCogToRearAxle_P)) / rtb_Divide_l4;
  rtb_Gain = LGC_TimeSysCycle_P * 0.5F;
  for (b_j = 0; b_j < 16; b_j++) {
    MatA_T_0 = g[b_j];
    P_next[b_j] = (real32_T)MatA_T_0 - rtb_Gain * rtb_y_h2[b_j];
    MatA_T[b_j] = (real32_T)MatA_T_0;
  }

  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (b_j = 0; b_j < 3; b_j++) {
    MatA_T_0 = b_j * 5;
    jA = 0;
    ix = MatA_T_0;
    smax = fabsf(P_next[MatA_T_0]);
    for (c_k = 2; c_k <= 4 - b_j; c_k++) {
      ix++;
      rtb_Gain1 = fabsf(P_next[ix]);
      if (rtb_Gain1 > smax) {
        jA = c_k - 1;
        smax = rtb_Gain1;
      }
    }

    if (P_next[MatA_T_0 + jA] != 0.0F) {
      if (jA != 0) {
        jA += b_j;
        ipiv[b_j] = (int8_T)(jA + 1);
        smax = P_next[b_j];
        P_next[b_j] = P_next[jA];
        P_next[jA] = smax;
        smax = P_next[b_j + 4];
        P_next[b_j + 4] = P_next[jA + 4];
        P_next[jA + 4] = smax;
        smax = P_next[b_j + 8];
        P_next[b_j + 8] = P_next[jA + 8];
        P_next[jA + 8] = smax;
        smax = P_next[b_j + 12];
        P_next[b_j + 12] = P_next[jA + 12];
        P_next[jA + 12] = smax;
      }

      jA = (MatA_T_0 - b_j) + 4;
      for (ix = MatA_T_0 + 1; ix < jA; ix++) {
        P_next[ix] /= P_next[MatA_T_0];
      }
    }

    jA = MatA_T_0;
    ix = MatA_T_0 + 4;
    for (c_k = 0; c_k <= 2 - b_j; c_k++) {
      if (P_next[ix] != 0.0F) {
        smax = -P_next[ix];
        c_ix = MatA_T_0 + 1;
        d = (jA - b_j) + 8;
        for (ijA = jA + 5; ijA < d; ijA++) {
          P_next[ijA] += P_next[c_ix] * smax;
          c_ix++;
        }
      }

      ix += 4;
      jA += 4;
    }
  }

  for (b_j = 0; b_j < 16; b_j++) {
    rtb_y_h2[b_j] = rtb_Gain * rtb_y_h2[b_j] + MatA_T[b_j];
  }

  for (b_j = 0; b_j < 3; b_j++) {
    ipiv_0 = ipiv[b_j];
    if (b_j + 1 != ipiv_0) {
      rtb_Gain = rtb_y_h2[b_j];
      rtb_y_h2[b_j] = rtb_y_h2[ipiv_0 - 1];
      rtb_y_h2[ipiv_0 - 1] = rtb_Gain;
      rtb_Gain = rtb_y_h2[b_j + 4];
      rtb_y_h2[b_j + 4] = rtb_y_h2[ipiv_0 + 3];
      rtb_y_h2[ipiv_0 + 3] = rtb_Gain;
      rtb_Gain = rtb_y_h2[b_j + 8];
      rtb_y_h2[b_j + 8] = rtb_y_h2[ipiv_0 + 7];
      rtb_y_h2[ipiv_0 + 7] = rtb_Gain;
      rtb_Gain = rtb_y_h2[b_j + 12];
      rtb_y_h2[b_j + 12] = rtb_y_h2[ipiv_0 + 11];
      rtb_y_h2[ipiv_0 + 11] = rtb_Gain;
    }
  }

  for (MatA_T_0 = 0; MatA_T_0 < 4; MatA_T_0++) {
    jA = MatA_T_0 << 2;
    if (rtb_y_h2[jA] != 0.0F) {
      for (ix = 2; ix < 5; ix++) {
        b_j = (ix + jA) - 1;
        rtb_y_h2[b_j] -= P_next[ix - 1] * rtb_y_h2[jA];
      }
    }

    if (rtb_y_h2[jA + 1] != 0.0F) {
      for (ix = 3; ix < 5; ix++) {
        b_j = (ix + jA) - 1;
        rtb_y_h2[b_j] -= rtb_y_h2[jA + 1] * P_next[ix + 3];
      }
    }

    LGC_RawErrCourseDotCdc_tmp = rtb_y_h2[jA + 2];
    if (LGC_RawErrCourseDotCdc_tmp != 0.0F) {
      rtb_y_h2[jA + 3] -= LGC_RawErrCourseDotCdc_tmp * P_next[11];
    }
  }

  for (MatA_T_0 = 0; MatA_T_0 < 4; MatA_T_0++) {
    jA = MatA_T_0 << 2;
    LGC_RawErrCourseDotCdc_tmp = rtb_y_h2[jA + 3];
    if (LGC_RawErrCourseDotCdc_tmp != 0.0F) {
      rtb_y_h2[jA + 3] = LGC_RawErrCourseDotCdc_tmp / P_next[15];
      for (ix = 0; ix < 3; ix++) {
        b_j = ix + jA;
        rtb_y_h2[b_j] -= rtb_y_h2[jA + 3] * P_next[ix + 12];
      }
    }

    LGC_RawErrCourseDotCdc_tmp = rtb_y_h2[jA + 2];
    if (LGC_RawErrCourseDotCdc_tmp != 0.0F) {
      rtb_y_h2[jA + 2] = LGC_RawErrCourseDotCdc_tmp / P_next[10];
      for (ix = 0; ix < 2; ix++) {
        b_j = ix + jA;
        rtb_y_h2[b_j] -= rtb_y_h2[jA + 2] * P_next[ix + 8];
      }
    }

    LGC_RawErrCourseDotCdc_tmp = rtb_y_h2[jA + 1];
    if (LGC_RawErrCourseDotCdc_tmp != 0.0F) {
      rtb_y_h2[jA + 1] = LGC_RawErrCourseDotCdc_tmp / P_next[5];
      rtb_y_h2[jA] -= rtb_y_h2[jA + 1] * P_next[4];
    }

    if (rtb_y_h2[jA] != 0.0F) {
      rtb_y_h2[jA] /= P_next[0];
    }
  }

  /* MATLAB Function: '<S608>/MATLAB Function3' incorporates:
   *  UnitDelay: '<S608>/Unit Delay'
   */
  for (b_j = 0; b_j < 4; b_j++) {
    MatA_T_0 = b_j << 2;
    MatA_T[MatA_T_0] = rtb_y_h2[b_j];
    MatA_T[MatA_T_0 + 1] = rtb_y_h2[b_j + 4];
    MatA_T[MatA_T_0 + 2] = rtb_y_h2[b_j + 8];
    MatA_T[MatA_T_0 + 3] = rtb_y_h2[b_j + 12];
  }

  num_iteration = 0U;
  rtb_Gain = 1.0E+6F;
  while ((num_iteration < 150) && (rtb_Gain > 0.01)) {
    num_iteration++;
    rtb_Gain = 0.0F;
    for (b_j = 0; b_j < 4; b_j++) {
      rtb_Add1_m[b_j] = 0.0F;
      LGC_RawErrCourseDotCdc_tmp = 0.0F;
      for (MatA_T_0 = 0; MatA_T_0 < 4; MatA_T_0++) {
        smax = rtb_y_dr[MatA_T_0];
        jA = MatA_T_0 << 2;
        ix = b_j + jA;
        P_next_tmp[ix] = 0.0F;
        P_next_tmp[ix] += TJATCT_DW.UnitDelay_DSTATE[jA] * MatA_T[b_j];
        P_next_tmp[ix] += TJATCT_DW.UnitDelay_DSTATE[jA + 1] * MatA_T[b_j + 4];
        P_next_tmp[ix] += TJATCT_DW.UnitDelay_DSTATE[jA + 2] * MatA_T[b_j + 8];
        P_next_tmp[ix] += TJATCT_DW.UnitDelay_DSTATE[jA + 3] * MatA_T[b_j + 12];
        LGC_RawErrCourseDotCdc_tmp += P_next_tmp[ix] * smax;
        rtb_Add1_m[b_j] += TJATCT_DW.UnitDelay_DSTATE[(b_j << 2) + MatA_T_0] *
          smax;
      }

      rtb_Gain += rtb_Add1_m[b_j] * rtb_y_dr[b_j];
      P_next_tmp_0[b_j] = LGC_RawErrCourseDotCdc_tmp;
    }

    rtb_Gain = 1.0F / (rtb_Gain + 1.0F);
    for (b_j = 0; b_j < 4; b_j++) {
      P_next_tmp_1[b_j] = P_next_tmp_0[b_j] * rtb_Gain;
      LGC_RawErrCourseDotCdc_tmp = 0.0F;
      for (MatA_T_0 = 0; MatA_T_0 < 4; MatA_T_0++) {
        LGC_RawErrCourseDotCdc_tmp += rtb_y_h2[(b_j << 2) + MatA_T_0] *
          rtb_Add1_m[MatA_T_0];
        jA = MatA_T_0 << 2;
        ix = b_j + jA;
        P_next_tmp_2[ix] = 0.0F;
        P_next_tmp_2[ix] += rtb_y_h2[jA] * P_next_tmp[b_j];
        P_next_tmp_2[ix] += rtb_y_h2[jA + 1] * P_next_tmp[b_j + 4];
        P_next_tmp_2[ix] += rtb_y_h2[jA + 2] * P_next_tmp[b_j + 8];
        P_next_tmp_2[ix] += rtb_y_h2[jA + 3] * P_next_tmp[b_j + 12];
      }

      rtb_Add1_h[b_j] = LGC_RawErrCourseDotCdc_tmp;
    }

    for (b_j = 0; b_j < 4; b_j++) {
      rtb_Gain = rtb_Add1_h[b_j];
      ix = b_j << 2;
      P_next_tmp_3[ix] = P_next_tmp_1[0] * rtb_Gain;
      P_next_tmp_3[ix + 1] = P_next_tmp_1[1] * rtb_Gain;
      P_next_tmp_3[ix + 2] = P_next_tmp_1[2] * rtb_Gain;
      P_next_tmp_3[ix + 3] = P_next_tmp_1[3] * rtb_Gain;
    }

    for (b_j = 0; b_j < 16; b_j++) {
      rtb_Gain = (P_next_tmp_2[b_j] - P_next_tmp_3[b_j]) + rtb_y_bw[b_j];
      TJATCT_DW.UnitDelay_DSTATE[b_j] = rtb_Gain -
        TJATCT_DW.UnitDelay_DSTATE[b_j];
      P_next_tmp[b_j] = fabsf(TJATCT_DW.UnitDelay_DSTATE[b_j]);
      P_next[b_j] = rtb_Gain;
    }

    for (b_j = 0; b_j < 4; b_j++) {
      MatA_T_0 = b_j << 2;
      rtb_Gain = P_next_tmp[MatA_T_0];
      LGC_RawErrCourseDotCdc_tmp = P_next_tmp[MatA_T_0 + 1];
      if (rtb_Gain < LGC_RawErrCourseDotCdc_tmp) {
        rtb_Gain = LGC_RawErrCourseDotCdc_tmp;
      }

      LGC_RawErrCourseDotCdc_tmp = P_next_tmp[MatA_T_0 + 2];
      if (rtb_Gain < LGC_RawErrCourseDotCdc_tmp) {
        rtb_Gain = LGC_RawErrCourseDotCdc_tmp;
      }

      LGC_RawErrCourseDotCdc_tmp = P_next_tmp[MatA_T_0 + 3];
      if (rtb_Gain < LGC_RawErrCourseDotCdc_tmp) {
        rtb_Gain = LGC_RawErrCourseDotCdc_tmp;
      }

      rtb_Add1_m[b_j] = rtb_Gain;
    }

    rtb_Gain = rtb_Add1_m[0];
    if (rtb_Add1_m[0] < rtb_Add1_m[1]) {
      rtb_Gain = rtb_Add1_m[1];
    }

    if (rtb_Gain < rtb_Add1_m[2]) {
      rtb_Gain = rtb_Add1_m[2];
    }

    if (rtb_Gain < rtb_Add1_m[3]) {
      rtb_Gain = rtb_Add1_m[3];
    }

    memcpy(&TJATCT_DW.UnitDelay_DSTATE[0], &P_next[0], sizeof(real32_T) << 4U);
  }

  /* Lookup_n-D: '<S607>/1-D Lookup Table4' incorporates:
   *  Constant: '<S607>/Constant1'
   *  Inport: '<Root>/Inport'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LQR_MatK_k1 = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LQR_VelX[0])), LQR_e1_gains, 8U);

  /* Lookup_n-D: '<S607>/1-D Lookup Table1' incorporates:
   *  Constant: '<S607>/Constant1'
   *  Inport: '<Root>/Inport1'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LQR_MatK_k2 = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LQR_VelX[0])), LQR_e1dot_gains, 8U);

  /* Lookup_n-D: '<S607>/1-D Lookup Table2' incorporates:
   *  Constant: '<S607>/Constant1'
   *  Inport: '<Root>/Inport2'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LQR_MatK_k3 = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LQR_VelX[0])), LQR_e2_gains, 8U);

  /* Lookup_n-D: '<S607>/1-D Lookup Table3' incorporates:
   *  Constant: '<S607>/Constant1'
   *  Inport: '<Root>/Inport3'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LQR_MatK_k4 = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LQR_VelX[0])), LQR_e2dot_gains, 8U);

  /* MultiPortSwitch: '<S406>/Multiport Switch1' incorporates:
   *  Constant: '<S406>/IAM_Ts_P11'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  RelationalOperator: '<S406>/GreaterThan1'
   */
  if (TCTI_VehicleVelX < 0.0F) {
    /* MultiPortSwitch: '<S406>/Multiport Switch1' incorporates:
     *  Constant: '<S406>/Parameter2'
     *  MinMax: '<S406>/Min'
     *  UnaryMinus: '<S406>/Unary Minus'
     */
    EST_LmtVehVelX = fminf(TCTI_VehicleVelX, -EST_ThdVehVel_P);
  } else {
    /* MultiPortSwitch: '<S406>/Multiport Switch1' incorporates:
     *  Constant: '<S406>/Parameter1'
     *  MinMax: '<S406>/Max'
     */
    EST_LmtVehVelX = fmaxf(TCTI_VehicleVelX, EST_ThdVehVel_P);
  }

  /* End of MultiPortSwitch: '<S406>/Multiport Switch1' */

  /* Sum: '<S413>/Add' incorporates:
   *  Constant: '<S413>/Parameter'
   *  Lookup_n-D: '<S413>/1-D Lookup Table1'
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_DistFromCgToGud = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_PreviewDistX_X[0])), ((const real32_T *)&(EST_PreviewDistX_M[0])), 12U)
    + EST_DistCogToFrontAxle_P;

  /* Product: '<S407>/Divide2' incorporates:
   *  Constant: '<S407>/Constant'
   *  Constant: '<S407>/Parameter2'
   *  Product: '<S407>/Divide5'
   */
  rtb_MeasDeviationVector_idx_1 = (real32_T)(1.0 / EST_MassVehicle_P);

  /* Sum: '<S407>/Subtract1' incorporates:
   *  Constant: '<S407>/Parameter1'
   *  Constant: '<S407>/Parameter6'
   *  Product: '<S407>/Divide3'
   *  Product: '<S407>/Product3'
   *  Sum: '<S408>/Subtract2'
   */
  rtb_Unit_Delay3 = rtb_MeasDeviationVector_idx_1 - EST_DistFromCgToGud *
    EST_DistCogToRearAxle_P / EST_InertiaVehicle_P;

  /* Sum: '<S407>/Add' incorporates:
   *  Constant: '<S407>/Parameter'
   *  Constant: '<S407>/Parameter3'
   *  Product: '<S407>/Divide1'
   *  Product: '<S407>/Divide2'
   *  Product: '<S407>/Product2'
   *  Sum: '<S408>/Add1'
   *  Sum: '<S411>/Add'
   */
  rtb_MeasDeviationVector_idx_1 += EST_DistFromCgToGud *
    EST_DistCogToFrontAxle_P / EST_InertiaVehicle_P;

  /* Sum: '<S407>/Subtract' incorporates:
   *  Constant: '<S407>/Parameter4'
   *  Constant: '<S407>/Parameter5'
   *  Product: '<S407>/Divide'
   *  Product: '<S407>/Divide4'
   *  Product: '<S407>/Product'
   *  Product: '<S407>/Product1'
   *  Product: '<S407>/Product4'
   *  Sum: '<S407>/Add'
   *  Sum: '<S407>/Subtract1'
   */
  EST_CoeffA11StateSpace = rtb_MeasDeviationVector_idx_1 * (real32_T)(-(real_T)
    EST_CorStiffFrontAxle_P / EST_LmtVehVelX) - rtb_Unit_Delay3 *
    (EST_CorStiffRearAxle_P / EST_LmtVehVelX);

  /* SignalConversion generated from: '<S309>/FirstColumnOfA' */
  rtb_y_bw[0] = EST_CoeffA11StateSpace;

  /* Product: '<S409>/Product1' incorporates:
   *  Constant: '<S409>/Parameter'
   *  Constant: '<S409>/Parameter4'
   *  Product: '<S410>/Product3'
   *  Product: '<S412>/Product'
   */
  rtb_Divide_l4 = EST_CorStiffFrontAxle_P * EST_DistCogToFrontAxle_P;

  /* Product: '<S409>/Product' incorporates:
   *  Constant: '<S409>/Parameter1'
   *  Constant: '<S409>/Parameter5'
   *  Product: '<S410>/Product4'
   */
  rtb_Gain = EST_CorStiffRearAxle_P * EST_DistCogToRearAxle_P;

  /* Product: '<S409>/Divide' incorporates:
   *  Constant: '<S409>/Parameter3'
   *  Product: '<S409>/Product'
   *  Product: '<S409>/Product1'
   *  Sum: '<S409>/Subtract'
   */
  EST_CoeffA21StateSpace = (rtb_Gain - rtb_Divide_l4) / EST_InertiaVehicle_P;

  /* SignalConversion generated from: '<S309>/FirstColumnOfA' */
  rtb_y_bw[1] = EST_CoeffA21StateSpace;

  /* UnaryMinus: '<S309>/Unary Minus' */
  rtb_y_bw[2] = -EST_CoeffA11StateSpace;

  /* SignalConversion generated from: '<S309>/FirstColumnOfA' incorporates:
   *  Constant: '<S309>/Constant'
   */
  rtb_y_bw[3] = 0.0F;

  /* Product: '<S408>/Product2' */
  rtb_Add1_ge = EST_LmtVehVelX * EST_LmtVehVelX;

  /* Sum: '<S408>/Subtract3' incorporates:
   *  Constant: '<S408>/Parameter'
   *  Sum: '<S410>/Subtract'
   */
  smax = EST_DistCogToFrontAxle_P - EST_DistFromCgToGud;

  /* Sum: '<S408>/Add2' incorporates:
   *  Constant: '<S408>/Parameter1'
   *  Sum: '<S410>/Add1'
   */
  rtb_Gain1 = EST_DistFromCgToGud + EST_DistCogToRearAxle_P;

  /* Sum: '<S408>/Subtract' incorporates:
   *  Constant: '<S408>/Constant'
   *  Constant: '<S408>/Parameter4'
   *  Constant: '<S408>/Parameter5'
   *  Product: '<S408>/Divide'
   *  Product: '<S408>/Divide4'
   *  Product: '<S408>/Product'
   *  Product: '<S408>/Product1'
   *  Product: '<S408>/Product5'
   *  Product: '<S408>/Product6'
   *  Sum: '<S408>/Add2'
   *  Sum: '<S408>/Subtract1'
   *  Sum: '<S408>/Subtract3'
   */
  EST_CoeffA12StateSpace = (rtb_Unit_Delay3 * (EST_CorStiffRearAxle_P /
    rtb_Add1_ge) * rtb_Gain1 - rtb_MeasDeviationVector_idx_1 *
    (EST_CorStiffFrontAxle_P / rtb_Add1_ge) * smax) - 1.0F;

  /* SignalConversion generated from: '<S309>/SecondColumnOfA' */
  rtb_y_bw[4] = EST_CoeffA12StateSpace;

  /* Product: '<S410>/Divide' incorporates:
   *  Constant: '<S410>/Parameter3'
   *  Product: '<S410>/Product'
   *  Product: '<S410>/Product1'
   *  Product: '<S410>/Product2'
   *  Product: '<S410>/Product5'
   *  Sum: '<S410>/Add'
   */
  EST_CoeffA22StateSpace = (real32_T)(-(real_T)(rtb_Divide_l4 * smax + rtb_Gain1
    * rtb_Gain) / (EST_LmtVehVelX * EST_InertiaVehicle_P));

  /* SignalConversion generated from: '<S309>/SecondColumnOfA' */
  rtb_y_bw[5] = EST_CoeffA22StateSpace;

  /* UnaryMinus: '<S309>/Unary Minus1' incorporates:
   *  Constant: '<S309>/Constant1'
   *  Sum: '<S309>/Add'
   */
  rtb_y_bw[6] = -(EST_CoeffA12StateSpace + 1.0F);

  /* SignalConversion generated from: '<S309>/SecondColumnOfA' incorporates:
   *  Constant: '<S309>/Constant2'
   */
  rtb_y_bw[7] = 0.0F;

  /* SignalConversion generated from: '<S309>/ThirdColumnOfA' incorporates:
   *  Constant: '<S309>/Constant3'
   */
  rtb_y_bw[8] = 0.0F;

  /* SignalConversion generated from: '<S309>/ThirdColumnOfA' incorporates:
   *  Constant: '<S309>/Constant3'
   */
  rtb_y_bw[9] = 0.0F;

  /* SignalConversion generated from: '<S309>/ThirdColumnOfA' incorporates:
   *  Constant: '<S309>/Constant3'
   */
  rtb_y_bw[10] = 0.0F;

  /* SignalConversion generated from: '<S309>/ThirdColumnOfA' */
  rtb_y_bw[11] = EST_LmtVehVelX;

  /* SignalConversion generated from: '<S309>/SystemMatrixA' */
  rtb_y_bw[12] = 0.0F;
  rtb_y_bw[13] = 0.0F;
  rtb_y_bw[14] = 0.0F;
  rtb_y_bw[15] = 0.0F;

  /* Product: '<S308>/Product2' incorporates:
   *  Concatenate: '<S309>/SystemMatrixA'
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  for (b_j = 0; b_j < 4; b_j++) {
    rtb_Gain = rtb_y_bw[b_j + 12] * TJATCT_DW.UnitDelay3_DSTATE[3] +
      (rtb_y_bw[b_j + 8] * TJATCT_DW.UnitDelay3_DSTATE[2] + (rtb_y_bw[b_j + 4] *
        TJATCT_DW.UnitDelay3_DSTATE[1] + rtb_y_bw[b_j] *
        TJATCT_DW.UnitDelay3_DSTATE[0]));
    rtb_Add1_m[b_j] = rtb_Gain;
  }

  /* End of Product: '<S308>/Product2' */

  /* Lookup_n-D: '<S391>/1-D Lookup Table' incorporates:
   *  Abs: '<S391>/Abs'
   *  Inport: '<Root>/TCTI_WhlSteerAngleVdy'
   */
  EST_RatioSteerGear = look1_iflf_binlxpw(fabsf(TCTI_WhlSteerAngleVdy), ((const
    real32_T *)&(EST_RatioSteerGear_X[0])), ((const real32_T *)
    &(EST_RatioSteerGear_M[0])), 5U);

  /* Product: '<S391>/Divide1' incorporates:
   *  Inport: '<Root>/TCTI_WhlSteerAngleVdy'
   */
  EST_AngleVDYSteer = TCTI_WhlSteerAngleVdy / EST_RatioSteerGear;

  /* Product: '<S391>/Product' incorporates:
   *  Constant: '<S391>/Constant'
   *  Constant: '<S391>/Constant1'
   *  Inport: '<Root>/TCTI_SteerAngleLaDmc'
   *  Product: '<S391>/Divide'
   */
  EST_AngleLaDMCSteer = TCTI_SteerAngleLaDmc / 180.0F * 3.14159F;

  /* MultiPortSwitch: '<S391>/Multiport Switch' incorporates:
   *  Constant: '<S391>/Parameter2'
   */
  if (EST_CswSteerAngle_P == 0) {
    /* MultiPortSwitch: '<S391>/Multiport Switch' */
    EST_AngleCurSteer = EST_AngleVDYSteer;
  } else {
    /* MultiPortSwitch: '<S391>/Multiport Switch' */
    EST_AngleCurSteer = EST_AngleLaDMCSteer;
  }

  /* End of MultiPortSwitch: '<S391>/Multiport Switch' */

  /* UnitDelay: '<S305>/Unit_Delay' */
  rtb_Gain = TJATCT_DW.Unit_Delay_DSTATE;

  /* UnitDelay: '<S305>/Unit_Delay1' */
  smax = TJATCT_DW.Unit_Delay1_DSTATE;

  /* UnitDelay: '<S305>/Unit_Delay2' */
  rtb_Gain1 = TJATCT_DW.Unit_Delay2_DSTATE;

  /* UnitDelay: '<S305>/Unit_Delay3' */
  rtb_Unit_Delay3 = TJATCT_DW.Unit_Delay3_DSTATE;

  /* MultiPortSwitch: '<S305>/Multiport_Switch' incorporates:
   *  Constant: '<S305>/Parameter1'
   *  UnitDelay: '<S305>/Unit_Delay1'
   *  UnitDelay: '<S305>/Unit_Delay2'
   *  UnitDelay: '<S305>/Unit_Delay3'
   *  UnitDelay: '<S305>/Unit_Delay4'
   */
  switch (EST_CswStrAngleDly_P) {
   case 0:
    rtb_Add1_ge = EST_AngleCurSteer;
    break;

   case 1:
    rtb_Add1_ge = rtb_Gain;
    break;

   case 2:
    rtb_Add1_ge = TJATCT_DW.Unit_Delay1_DSTATE;
    break;

   case 3:
    rtb_Add1_ge = TJATCT_DW.Unit_Delay2_DSTATE;
    break;

   case 4:
    rtb_Add1_ge = TJATCT_DW.Unit_Delay3_DSTATE;
    break;

   default:
    rtb_Add1_ge = TJATCT_DW.Unit_Delay4_DSTATE;
    break;
  }

  /* End of MultiPortSwitch: '<S305>/Multiport_Switch' */

  /* Product: '<S411>/Product' incorporates:
   *  Constant: '<S411>/Parameter4'
   *  Product: '<S411>/Divide'
   */
  EST_CoeffB11StateSpace = rtb_MeasDeviationVector_idx_1 *
    (EST_CorStiffFrontAxle_P / EST_LmtVehVelX);

  /* SignalConversion generated from: '<S305>/InputVectorB' incorporates:
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE[0] = EST_CoeffB11StateSpace;

  /* Product: '<S412>/Divide' incorporates:
   *  Constant: '<S412>/Parameter3'
   */
  EST_CoeffB21StateSpace = rtb_Divide_l4 / EST_InertiaVehicle_P;

  /* SignalConversion generated from: '<S305>/InputVectorB' incorporates:
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE[1] = EST_CoeffB21StateSpace;

  /* UnaryMinus: '<S305>/Unary Minus2' incorporates:
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE[2] = -EST_CoeffB11StateSpace;

  /* SignalConversion generated from: '<S305>/InputVectorB' incorporates:
   *  Constant: '<S305>/Constant5'
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE[3] = 0.0F;

  /* Product: '<S305>/Product1' incorporates:
   *  UnitDelay: '<S308>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE[0] *= rtb_Add1_ge;
  TJATCT_DW.UnitDelay3_DSTATE[1] *= rtb_Add1_ge;
  TJATCT_DW.UnitDelay3_DSTATE[2] *= rtb_Add1_ge;
  TJATCT_DW.UnitDelay3_DSTATE[3] *= rtb_Add1_ge;

  /* SignalConversion generated from: '<S306>/DisturbanceVectorH' incorporates:
   *  Constant: '<S306>/Constant6'
   */
  rtb_y_dr[0] = 0.0F;

  /* SignalConversion generated from: '<S306>/DisturbanceVectorH' incorporates:
   *  Constant: '<S306>/Constant6'
   */
  rtb_y_dr[1] = 0.0F;

  /* SignalConversion generated from: '<S306>/DisturbanceVectorH' */
  rtb_y_dr[2] = EST_LmtVehVelX;

  /* SignalConversion generated from: '<S306>/DisturbanceVectorH' incorporates:
   *  Constant: '<S306>/Constant7'
   */
  rtb_y_dr[3] = 0.0F;

  /* RelationalOperator: '<S403>/Equal1' incorporates:
   *  Constant: '<S403>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  EST_EnaCntrlByTgq = (TCTI_BtfTrajGuiQualifier != EST_EnaOffByTgq_P);

  /* RelationalOperator: '<S403>/Equal3' incorporates:
   *  Constant: '<S403>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3 = (TCTI_BtfTrajGuiQualifier == EST_EnaFreezeByTgq_P);

  /* Logic: '<S403>/OR' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S403>/NOT'
   *  RelationalOperator: '<S414>/FixPt Relational Operator'
   *  UnitDelay: '<S414>/Delay Input1'
   *
   * Block description for '<S414>/Delay Input1':
   *
   *  Store in Global RAM
   */
  EST_EnaResetByTgq = ((!EST_EnaCntrlByTgq) || ((int32_T)rtb_Equal3 < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_i) || TCTI_EnaReplanCurValues);

  /* Product: '<S392>/Divide' incorporates:
   *  Constant: '<S389>/Parameter3'
   *  Constant: '<S389>/Parameter9'
   *  MinMax: '<S392>/Max1'
   */
  rtb_Divide_l4 = EST_TiSysCycle_P / fmaxf(EST_TiSysCycle_P,
    EST_TiPlObsInCrvFlt_P);

  /* Switch: '<S393>/Switch2' incorporates:
   *  Constant: '<S392>/IAM_Ts_P1'
   *  Constant: '<S392>/IAM_Ts_P4'
   *  RelationalOperator: '<S393>/LowerRelop1'
   *  RelationalOperator: '<S393>/UpperRelop'
   *  Switch: '<S393>/Switch'
   */
  if (rtb_Divide_l4 > 1.0F) {
    rtb_Divide_l4 = 1.0F;
  } else {
    if (rtb_Divide_l4 < 0.0F) {
      /* Switch: '<S393>/Switch' incorporates:
       *  Constant: '<S392>/IAM_Ts_P1'
       */
      rtb_Divide_l4 = 0.0F;
    }
  }

  /* End of Switch: '<S393>/Switch2' */

  /* Sum: '<S392>/Add' incorporates:
   *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
   *  Product: '<S392>/Product'
   *  Sum: '<S392>/Subtract'
   */
  EST_CrvPiObsCrvFlt = (TCTI_ReqTrajCrvTpl - EST_CrvPiObsCrvFlt) * rtb_Divide_l4
    + EST_CrvPiObsCrvFlt;

  /* MultiPortSwitch: '<S389>/Multiport Switch' incorporates:
   *  Constant: '<S389>/Parameter1'
   *  Constant: '<S389>/Parameter2'
   *  MultiPortSwitch: '<S389>/Multiport Switch1'
   *  MultiPortSwitch: '<S389>/Multiport Switch2'
   *  RelationalOperator: '<S389>/Equal'
   *  RelationalOperator: '<S389>/Equal1'
   *  RelationalOperator: '<S389>/Equal2'
   */
  if (EST_CswPlObsCrv_P == 1) {
    /* MultiPortSwitch: '<S389>/Multiport Switch' incorporates:
     *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
     */
    EST_CrvPlObsIn = TCTI_ReqTrajCrvTpl;
  } else if (EST_CswPlObsCrv_P != 2) {
    /* MultiPortSwitch: '<S389>/Multiport Switch1' incorporates:
     *  Constant: '<S389>/Constant2'
     *  MultiPortSwitch: '<S389>/Multiport Switch'
     */
    EST_CrvPlObsIn = 0.0F;
  } else if (!EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S389>/Multiport Switch2' incorporates:
     *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
     *  MultiPortSwitch: '<S389>/Multiport Switch'
     *  MultiPortSwitch: '<S389>/Multiport Switch1'
     */
    EST_CrvPlObsIn = TCTI_ReqTrajCrvTpl;
  } else {
    /* MultiPortSwitch: '<S389>/Multiport Switch' incorporates:
     *  MultiPortSwitch: '<S389>/Multiport Switch1'
     *  MultiPortSwitch: '<S389>/Multiport Switch2'
     */
    EST_CrvPlObsIn = EST_CrvPiObsCrvFlt;
  }

  /* End of MultiPortSwitch: '<S389>/Multiport Switch' */

  /* Lookup_n-D: '<S311>/1-D Lookup Table' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL11Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL11Pobs_X[0])), ((const real32_T *)&(EST_CoeffL11Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/FirstColumnOfL' */
  EST_CoeffLPobs[0] = EST_CoeffL11Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table1' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL21Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL21Pobs_X[0])), ((const real32_T *)&(EST_CoeffL21Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/FirstColumnOfL' */
  EST_CoeffLPobs[1] = EST_CoeffL21Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table2' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL31Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL31Pobs_X[0])), ((const real32_T *)&(EST_CoeffL31Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/FirstColumnOfL' */
  EST_CoeffLPobs[2] = EST_CoeffL31Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table3' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL41Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL41Pobs_X[0])), ((const real32_T *)&(EST_CoeffL41Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/FirstColumnOfL' */
  EST_CoeffLPobs[3] = EST_CoeffL41Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table4' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL12Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL12Pobs_X[0])), ((const real32_T *)&(EST_CoeffL12Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/SecondColumnOfL' */
  EST_CoeffLPobs[4] = EST_CoeffL12Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table5' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL22Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL22Pobs_X[0])), ((const real32_T *)&(EST_CoeffL22Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/SecondColumnOfL' */
  EST_CoeffLPobs[5] = EST_CoeffL22Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table6' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL32Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL32Pobs_X[0])), ((const real32_T *)&(EST_CoeffL32Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/SecondColumnOfL' */
  EST_CoeffLPobs[6] = EST_CoeffL32Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table7' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL42Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL42Pobs_X[0])), ((const real32_T *)&(EST_CoeffL42Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/SecondColumnOfL' */
  EST_CoeffLPobs[7] = EST_CoeffL42Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table9' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL13Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL13Pobs_X[0])), ((const real32_T *)&(EST_CoeffL13Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/ThirdColumnOfL' */
  EST_CoeffLPobs[8] = EST_CoeffL13Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table10' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL23Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL23Pobs_X[0])), ((const real32_T *)&(EST_CoeffL23Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/ThirdColumnOfL' */
  EST_CoeffLPobs[9] = EST_CoeffL23Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table11' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL33Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL33Pobs_X[0])), ((const real32_T *)&(EST_CoeffL33Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/ThirdColumnOfL' */
  EST_CoeffLPobs[10] = EST_CoeffL33Pobs;

  /* Lookup_n-D: '<S311>/1-D Lookup Table12' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL43Pobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL41Pobs_X[0])), ((const real32_T *)&(EST_CoeffL43Pobs_M[0])), 12U);

  /* SignalConversion generated from: '<S311>/ThirdColumnOfL' */
  EST_CoeffLPobs[11] = EST_CoeffL43Pobs;

  /* Switch: '<S381>/Init' incorporates:
   *  Logic: '<S381>/FixPt Logical Operator'
   *  UnitDelay: '<S381>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE != 0)) {
    /* Switch: '<S381>/Init' incorporates:
     *  Constant: '<S376>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE = 0.0F;
  }

  /* End of Switch: '<S381>/Init' */

  /* Switch: '<S379>/Switch2' incorporates:
   *  Constant: '<S376>/Constant4'
   *  Constant: '<S376>/Constant5'
   *  RelationalOperator: '<S379>/LowerRelop1'
   *  RelationalOperator: '<S379>/UpperRelop'
   *  Switch: '<S379>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE > 3.1415F) {
    /* Switch: '<S379>/Switch2' */
    EST_EstBetaSObs = 3.1415F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE < -3.1415F) {
    /* Switch: '<S379>/Switch' incorporates:
     *  Constant: '<S376>/Constant5'
     *  Switch: '<S379>/Switch2'
     */
    EST_EstBetaSObs = -3.1415F;
  } else {
    /* Switch: '<S379>/Switch2' incorporates:
     *  Switch: '<S379>/Switch'
     */
    EST_EstBetaSObs = TJATCT_DW.FixPtUnitDelay1_DSTATE;
  }

  /* End of Switch: '<S379>/Switch2' */

  /* MultiPortSwitch: '<S374>/Multiport Switch2' incorporates:
   *  Constant: '<S374>/Parameter'
   */
  if (EST_CswEnaBetaSObs_P == 0) {
    /* MultiPortSwitch: '<S374>/Multiport Switch2' incorporates:
     *  Constant: '<S374>/Constant1'
     */
    EST_RawBetaSObs = 0.0F;
  } else {
    /* MultiPortSwitch: '<S374>/Multiport Switch2' */
    EST_RawBetaSObs = EST_EstBetaSObs;
  }

  /* End of MultiPortSwitch: '<S374>/Multiport Switch2' */

  /* Sum: '<S310>/Subtract' */
  rtb_Divide_l4 = EST_RawBetaSObs - EST_EstBetaPobs;

  /* Sum: '<S310>/Subtract1' incorporates:
   *  Inport: '<Root>/TCTI_VehYawRate'
   */
  rtb_MeasDeviationVector_idx_1 = TCTI_VehYawRate - EST_EstPsiDotPobs;

  /* Sum: '<S390>/Subtract' incorporates:
   *  Inport: '<Root>/TCTI_CurTrajDistYTpl'
   *  Inport: '<Root>/TCTI_ReqTrajDistYTpl'
   *  Sum: '<S88>/Add'
   */
  EST_DistYDevTrajFromCur = TCTI_ReqTrajDistYTpl - TCTI_CurTrajDistYTpl;

  /* RelationalOperator: '<S395>/GreaterThan' incorporates:
   *  Abs: '<S395>/Abs'
   *  Constant: '<S395>/Parameter1'
   *  UnitDelay: '<S395>/Unit Delay'
   */
  EST_EnaLmtByDistY = (fabsf(EST_DistYDevByGrdntLmt1) >= EST_PlObsInDYTolBndThr1);

  /* Logic: '<S396>/NOT1' incorporates:
   *  Logic: '<S355>/NOT'
   *  Logic: '<S356>/OR'
   *  Logic: '<S366>/NOT'
   */
  rtb_Equal3_m = !EST_EnaResetByTgq;

  /* Logic: '<S396>/OR' incorporates:
   *  Abs: '<S396>/Abs'
   *  Constant: '<S396>/Parameter1'
   *  Logic: '<S396>/AND1'
   *  Logic: '<S396>/NOT1'
   *  RelationalOperator: '<S396>/GreaterThan'
   *  UnitDelay: '<S396>/Unit Delay'
   */
  EST_EnaLmt2ByDistY = ((EST_EnaLmtByDistY && (fabsf
    (TJATCT_DW.UnitDelay_DSTATE_m) >= EST_PlObsInDYTolBndThr2_P)) ||
                        rtb_Equal3_m);

  /* MultiPortSwitch: '<S398>/Multiport Switch1' incorporates:
   *  Logic: '<S395>/NOT'
   *  Logic: '<S395>/OR'
   */
  if ((!EST_EnaLmtByDistY) || EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S398>/Multiport Switch1' incorporates:
     *  UnitDelay: '<S398>/Unit Delay'
     */
    EST_DistYDevByGrdntLmt1 = TJATCT_DW.UnitDelay_DSTATE_p0;
  } else {
    /* Sum: '<S398>/Sum' incorporates:
     *  UnitDelay: '<S398>/Unit Delay'
     */
    rtb_Sum_kv = EST_DistYDevTrajFromCur - TJATCT_DW.UnitDelay_DSTATE_p0;

    /* Product: '<S398>/Product' incorporates:
     *  Constant: '<S395>/Parameter2'
     *  Constant: '<S395>/Parameter3'
     */
    rtb_Product_km = EST_PlObsInDYTolBndGL1 * EST_TiSysCycle_P;

    /* Switch: '<S399>/Switch2' incorporates:
     *  RelationalOperator: '<S399>/LowerRelop1'
     */
    if (rtb_Sum_kv > rtb_Product_km) {
      rtb_Sum_kv = rtb_Product_km;
    } else {
      /* Product: '<S398>/Product1' incorporates:
       *  Constant: '<S395>/Parameter2'
       *  Constant: '<S395>/Parameter3'
       *  UnaryMinus: '<S395>/Unary Minus'
       */
      rtb_Product_km = -EST_PlObsInDYTolBndGL1 * EST_TiSysCycle_P;

      /* Switch: '<S399>/Switch' incorporates:
       *  RelationalOperator: '<S399>/UpperRelop'
       */
      if (rtb_Sum_kv < rtb_Product_km) {
        rtb_Sum_kv = rtb_Product_km;
      }

      /* End of Switch: '<S399>/Switch' */
    }

    /* End of Switch: '<S399>/Switch2' */

    /* MultiPortSwitch: '<S398>/Multiport Switch1' incorporates:
     *  Sum: '<S398>/Difference Inputs3'
     *  UnitDelay: '<S398>/Unit Delay'
     *
     * Block description for '<S398>/Difference Inputs3':
     *
     *  Add in CPU
     */
    EST_DistYDevByGrdntLmt1 = rtb_Sum_kv + TJATCT_DW.UnitDelay_DSTATE_p0;
  }

  /* End of MultiPortSwitch: '<S398>/Multiport Switch1' */

  /* MultiPortSwitch: '<S400>/Multiport Switch1' incorporates:
   *  Sum: '<S400>/Difference Inputs3'
   *  UnitDelay: '<S400>/Unit Delay'
   *
   * Block description for '<S400>/Difference Inputs3':
   *
   *  Add in CPU
   */
  switch ((int32_T)EST_EnaLmt2ByDistY) {
   case 0:
    break;

   default:
    /* Sum: '<S400>/Sum' incorporates:
     *  UnitDelay: '<S400>/Unit Delay'
     */
    rtb_Sum_kv = EST_DistYDevByGrdntLmt1 - TJATCT_DW.UnitDelay_DSTATE_p;

    /* Product: '<S400>/Product' incorporates:
     *  Constant: '<S396>/Parameter2'
     *  Constant: '<S396>/Parameter3'
     */
    rtb_Product_km = EST_PlObsInDYTolBndGL2_P * EST_TiSysCycle_P;

    /* Switch: '<S401>/Switch2' incorporates:
     *  RelationalOperator: '<S401>/LowerRelop1'
     */
    if (rtb_Sum_kv > rtb_Product_km) {
      rtb_Sum_kv = rtb_Product_km;
    } else {
      /* Product: '<S400>/Product1' incorporates:
       *  Constant: '<S396>/Parameter2'
       *  Constant: '<S396>/Parameter3'
       *  UnaryMinus: '<S396>/Unary Minus'
       */
      rtb_Product_km = -EST_PlObsInDYTolBndGL2_P * EST_TiSysCycle_P;

      /* Switch: '<S401>/Switch' incorporates:
       *  RelationalOperator: '<S401>/UpperRelop'
       */
      if (rtb_Sum_kv < rtb_Product_km) {
        rtb_Sum_kv = rtb_Product_km;
      }

      /* End of Switch: '<S401>/Switch' */
    }

    /* End of Switch: '<S401>/Switch2' */
    TJATCT_DW.UnitDelay_DSTATE_p += rtb_Sum_kv;
    break;
  }

  /* End of MultiPortSwitch: '<S400>/Multiport Switch1' */

  /* MultiPortSwitch: '<S390>/Multiport Switch' incorporates:
   *  Constant: '<S390>/Parameter'
   *  RelationalOperator: '<S390>/Equal'
   */
  if (EST_CswPlObsDeltaY_P == 1) {
    /* MultiPortSwitch: '<S390>/Multiport Switch' */
    EST_DeltaYPlObsIn = EST_DistYDevTrajFromCur;
  } else {
    /* MultiPortSwitch: '<S390>/Multiport Switch' incorporates:
     *  UnitDelay: '<S400>/Unit Delay'
     */
    EST_DeltaYPlObsIn = TJATCT_DW.UnitDelay_DSTATE_p;
  }

  /* End of MultiPortSwitch: '<S390>/Multiport Switch' */

  /* Sum: '<S310>/Subtract2' */
  rtb_Sum_kv = EST_DeltaYPlObsIn - EST_EstDYPobs;
  for (b_j = 0; b_j < 4; b_j++) {
    /* Product: '<S310>/Mul' incorporates:
     *  Concatenate: '<S311>/FeedbackMatrixL'
     */
    EST_CoeffLYPObs[b_j] = 0.0F;
    EST_CoeffLYPObs[b_j] = EST_CoeffLPobs[b_j] * rtb_Divide_l4 +
      EST_CoeffLYPObs[b_j];
    EST_CoeffLYPObs[b_j] = EST_CoeffLPobs[b_j + 4] *
      rtb_MeasDeviationVector_idx_1 + EST_CoeffLYPObs[b_j];
    EST_CoeffLYPObs[b_j] = EST_CoeffLPobs[b_j + 8] * rtb_Sum_kv +
      EST_CoeffLYPObs[b_j];

    /* Sum: '<S312>/Add1' incorporates:
     *  Product: '<S306>/Product'
     *  Sum: '<S312>/Add'
     *  Sum: '<S312>/Add3'
     */
    rtb_Add1_m[b_j] = (rtb_y_dr[b_j] * EST_CrvPlObsIn + EST_CoeffLYPObs[b_j]) +
      (rtb_Add1_m[b_j] + TJATCT_DW.UnitDelay3_DSTATE[b_j]);
  }

  /* Product: '<S385>/Mul' incorporates:
   *  SignalConversion generated from: '<S385>/Vector Concatenate1'
   *  SignalConversion generated from: '<S385>/Vector Concatenate'
   *  UnitDelay: '<S385>/Unit_Delay'
   * */
  EST_CoeffAXStateSpace[0] = 0.0F;
  EST_CoeffAXStateSpace[0] = EST_CoeffA11StateSpace *
    TJATCT_DW.Unit_Delay_DSTATE_h[0] + EST_CoeffAXStateSpace[0];
  EST_CoeffAXStateSpace[0] = EST_CoeffA12StateSpace *
    TJATCT_DW.Unit_Delay_DSTATE_h[1] + EST_CoeffAXStateSpace[0];
  EST_CoeffAXStateSpace[1] = 0.0F;
  EST_CoeffAXStateSpace[1] = EST_CoeffA21StateSpace *
    TJATCT_DW.Unit_Delay_DSTATE_h[0] + EST_CoeffAXStateSpace[1];
  EST_CoeffAXStateSpace[1] = EST_CoeffA22StateSpace *
    TJATCT_DW.Unit_Delay_DSTATE_h[1] + EST_CoeffAXStateSpace[1];

  /* UnitDelay: '<S386>/Unit_Delay' */
  rtb_Divide_l4 = TJATCT_DW.Unit_Delay_DSTATE_m;

  /* UnitDelay: '<S386>/Unit_Delay1' */
  rtb_MeasDeviationVector_idx_1 = TJATCT_DW.Unit_Delay1_DSTATE_j;

  /* UnitDelay: '<S386>/Unit_Delay2' */
  rtb_Sum_kv = TJATCT_DW.Unit_Delay2_DSTATE_g;

  /* UnitDelay: '<S386>/Unit_Delay3' */
  rtb_Product_km = TJATCT_DW.Unit_Delay3_DSTATE_j;

  /* MultiPortSwitch: '<S386>/Multiport_Switch' incorporates:
   *  Constant: '<S386>/Parameter'
   */
  switch (EST_CswStrAngleDly_P) {
   case 0:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' */
    EST_DlyCurSteerAngle = EST_AngleCurSteer;
    break;

   case 1:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' */
    EST_DlyCurSteerAngle = rtb_Divide_l4;
    break;

   case 2:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' incorporates:
     *  UnitDelay: '<S386>/Unit_Delay1'
     */
    EST_DlyCurSteerAngle = TJATCT_DW.Unit_Delay1_DSTATE_j;
    break;

   case 3:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' incorporates:
     *  UnitDelay: '<S386>/Unit_Delay2'
     */
    EST_DlyCurSteerAngle = TJATCT_DW.Unit_Delay2_DSTATE_g;
    break;

   case 4:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' incorporates:
     *  UnitDelay: '<S386>/Unit_Delay3'
     */
    EST_DlyCurSteerAngle = TJATCT_DW.Unit_Delay3_DSTATE_j;
    break;

   default:
    /* MultiPortSwitch: '<S386>/Multiport_Switch' incorporates:
     *  UnitDelay: '<S386>/Unit_Delay4'
     */
    EST_DlyCurSteerAngle = TJATCT_DW.Unit_Delay4_DSTATE_j;
    break;
  }

  /* End of MultiPortSwitch: '<S386>/Multiport_Switch' */

  /* Lookup_n-D: '<S387>/1-D Lookup Table' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL11Sobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL11Sobs_X[0])), ((const real32_T *)&(EST_CoeffL11Sobs_M[0])), 12U);

  /* Lookup_n-D: '<S387>/1-D Lookup Table1' incorporates:
   *  MultiPortSwitch: '<S406>/Multiport Switch1'
   */
  EST_CoeffL21Sobs = look1_iflf_binlxpw(EST_LmtVehVelX, ((const real32_T *)
    &(EST_CoeffL21Sobs_X[0])), ((const real32_T *)&(EST_CoeffL21Sobs_M[0])), 12U);

  /* Sum: '<S387>/Subtract' incorporates:
   *  Inport: '<Root>/TCTI_VehYawRate'
   */
  EST_ErrVehYawRate = TCTI_VehYawRate - EST_EstPsiDotSObs;

  /* Product: '<S387>/Product' incorporates:
   *  SignalConversion generated from: '<S387>/Vector Concatenate5'
   * */
  EST_CoeffLYStateSpace[0] = EST_CoeffL11Sobs * EST_ErrVehYawRate;
  EST_CoeffLYStateSpace[1] = EST_CoeffL21Sobs * EST_ErrVehYawRate;

  /* SignalConversion: '<S375>/Signal Conversion' incorporates:
   *  Product: '<S386>/Product1'
   *  SignalConversion generated from: '<S386>/Vector Concatenate3'
   *  Sum: '<S375>/Add'
   *  Sum: '<S375>/Add1'
   */
  EST_BetaDotSObs = (EST_DlyCurSteerAngle * EST_CoeffB11StateSpace +
                     EST_CoeffAXStateSpace[0]) + EST_CoeffLYStateSpace[0];

  /* Switch: '<S381>/Reset' */
  if (EST_EnaResetByTgq) {
    /* Switch: '<S381>/Init' incorporates:
     *  Constant: '<S376>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE = 0.0F;
  } else {
    /* Switch: '<S381>/Init' incorporates:
     *  Constant: '<S376>/Parameter9'
     *  Product: '<S377>/Product'
     *  Sum: '<S377>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE = EST_BetaDotSObs * EST_TiSysCycle_P +
      EST_EstBetaSObs;
  }

  /* End of Switch: '<S381>/Reset' */

  /* SignalConversion: '<S375>/Signal Conversion1' incorporates:
   *  Product: '<S386>/Product1'
   *  SignalConversion generated from: '<S386>/Vector Concatenate3'
   *  Sum: '<S375>/Add'
   *  Sum: '<S375>/Add1'
   */
  EST_Psi2DotSObs = (EST_DlyCurSteerAngle * EST_CoeffB21StateSpace +
                     EST_CoeffAXStateSpace[1]) + EST_CoeffLYStateSpace[1];

  /* Switch: '<S384>/Init' incorporates:
   *  Logic: '<S384>/FixPt Logical Operator'
   *  UnitDelay: '<S384>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_m != 0)) {
    /* Switch: '<S384>/Init' incorporates:
     *  Constant: '<S376>/Constant3'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_f = 0.0F;
  }

  /* End of Switch: '<S384>/Init' */

  /* Switch: '<S382>/Switch2' incorporates:
   *  Constant: '<S376>/Constant6'
   *  Constant: '<S376>/Constant7'
   *  RelationalOperator: '<S382>/LowerRelop1'
   *  RelationalOperator: '<S382>/UpperRelop'
   *  Switch: '<S382>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_f > 3.1415F) {
    /* Switch: '<S382>/Switch2' */
    EST_EstPsiDotSObs = 3.1415F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_f < -3.1415F) {
    /* Switch: '<S382>/Switch' incorporates:
     *  Constant: '<S376>/Constant7'
     *  Switch: '<S382>/Switch2'
     */
    EST_EstPsiDotSObs = -3.1415F;
  } else {
    /* Switch: '<S382>/Switch2' incorporates:
     *  Switch: '<S382>/Switch'
     */
    EST_EstPsiDotSObs = TJATCT_DW.FixPtUnitDelay1_DSTATE_f;
  }

  /* End of Switch: '<S382>/Switch2' */

  /* Switch: '<S384>/Reset' */
  if (EST_EnaResetByTgq) {
    /* Switch: '<S384>/Init' incorporates:
     *  Constant: '<S376>/Constant3'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_f = 0.0F;
  } else {
    /* Switch: '<S384>/Init' incorporates:
     *  Constant: '<S376>/Parameter1'
     *  Product: '<S378>/Product'
     *  Sum: '<S378>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_f = EST_Psi2DotSObs * EST_TiSysCycle_P +
      EST_EstPsiDotSObs;
  }

  /* End of Switch: '<S384>/Reset' */

  /* SignalConversion: '<S312>/Signal Conversion2' */
  EST_DThetaDotPobs = rtb_Add1_m[2];

  /* Product: '<S343>/Divide' incorporates:
   *  Constant: '<S341>/Parameter6'
   *  Constant: '<S341>/Parameter7'
   *  MinMax: '<S343>/Max1'
   */
  rtb_Add1_ge = EST_TiSysCycle_P / fmaxf(EST_TiSysCycle_P,
    EST_TiHdrDThetaDotFlt_P);

  /* Switch: '<S344>/Switch2' incorporates:
   *  Constant: '<S343>/IAM_Ts_P1'
   *  Constant: '<S343>/IAM_Ts_P4'
   *  RelationalOperator: '<S344>/LowerRelop1'
   *  RelationalOperator: '<S344>/UpperRelop'
   *  Switch: '<S344>/Switch'
   */
  if (rtb_Add1_ge > 1.0F) {
    rtb_Add1_ge = 1.0F;
  } else {
    if (rtb_Add1_ge < 0.0F) {
      /* Switch: '<S344>/Switch' incorporates:
       *  Constant: '<S343>/IAM_Ts_P1'
       */
      rtb_Add1_ge = 0.0F;
    }
  }

  /* End of Switch: '<S344>/Switch2' */

  /* Sum: '<S343>/Add' incorporates:
   *  Product: '<S343>/Product'
   *  Sum: '<S343>/Subtract'
   */
  EST_RawFltDThetaDotPObs = (EST_DThetaDotPobs - EST_RawFltDThetaDotPObs) *
    rtb_Add1_ge + EST_RawFltDThetaDotPObs;

  /* MultiPortSwitch: '<S341>/Multiport Switch' incorporates:
   *  RelationalOperator: '<S341>/Equal'
   */
  if (EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S341>/Multiport Switch' incorporates:
     *  Constant: '<S341>/Constant4'
     */
    EST_FltDThetaDotPObs = 0.0F;
  } else {
    /* MultiPortSwitch: '<S341>/Multiport Switch' */
    EST_FltDThetaDotPObs = EST_RawFltDThetaDotPObs;
  }

  /* End of MultiPortSwitch: '<S341>/Multiport Switch' */

  /* Product: '<S340>/Product' incorporates:
   *  Abs: '<S340>/Abs'
   *  Constant: '<S340>/Constant'
   *  Constant: '<S340>/Parameter1'
   *  Constant: '<S340>/Parameter2'
   *  Constant: '<S340>/Parameter3'
   *  Product: '<S340>/Divide'
   *  Sum: '<S340>/Subtract'
   *  Sum: '<S340>/Subtract1'
   */
  EST_RawHdrPercByTheta = (fabsf(EST_FltDThetaDotPObs) - EST_MinDThetaDotHdr_P) /
    (EST_MaxDThetaDotHdr_P - EST_MinDThetaDotHdr_P) * 100.0F;

  /* Switch: '<S342>/Switch2' incorporates:
   *  Constant: '<S340>/Constant1'
   *  Constant: '<S340>/Constant2'
   *  RelationalOperator: '<S342>/LowerRelop1'
   *  RelationalOperator: '<S342>/UpperRelop'
   *  Switch: '<S342>/Switch'
   */
  if (EST_RawHdrPercByTheta > 100.0F) {
    /* Switch: '<S342>/Switch2' */
    EST_LmtHdrPercByTheta = 100.0F;
  } else if (EST_RawHdrPercByTheta < 0.0F) {
    /* Switch: '<S342>/Switch' incorporates:
     *  Constant: '<S340>/Constant2'
     *  Switch: '<S342>/Switch2'
     */
    EST_LmtHdrPercByTheta = 0.0F;
  } else {
    /* Switch: '<S342>/Switch2' incorporates:
     *  Switch: '<S342>/Switch'
     */
    EST_LmtHdrPercByTheta = EST_RawHdrPercByTheta;
  }

  /* End of Switch: '<S342>/Switch2' */

  /* RelationalOperator: '<S403>/Equal' incorporates:
   *  Constant: '<S403>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  EST_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == EST_EnaFreezeByTgq_P);

  /* MultiPortSwitch: '<S340>/Multiport Switch1' incorporates:
   *  Logic: '<S340>/AND'
   *  Logic: '<S340>/NOT'
   */
  if ((!EST_EnaCntrlByTgq) || EST_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S340>/Multiport Switch1' incorporates:
     *  Constant: '<S340>/Constant5'
     */
    EST_HdrPercByTheta = 0.0F;
  } else {
    /* MultiPortSwitch: '<S340>/Multiport Switch1' */
    EST_HdrPercByTheta = EST_LmtHdrPercByTheta;
  }

  /* End of MultiPortSwitch: '<S340>/Multiport Switch1' */

  /* Product: '<S338>/Divide' incorporates:
   *  Constant: '<S336>/Parameter6'
   *  Constant: '<S336>/Parameter7'
   *  MinMax: '<S338>/Max1'
   */
  rtb_Add1_ge = EST_TiSysCycle_P / fmaxf(EST_TiSysCycle_P, EST_TiHdrDYDotFlt_P);

  /* SignalConversion: '<S312>/Signal Conversion3' */
  EST_DYDotPobs = rtb_Add1_m[3];

  /* Switch: '<S339>/Switch2' incorporates:
   *  Constant: '<S338>/IAM_Ts_P1'
   *  Constant: '<S338>/IAM_Ts_P4'
   *  RelationalOperator: '<S339>/LowerRelop1'
   *  RelationalOperator: '<S339>/UpperRelop'
   *  Switch: '<S339>/Switch'
   */
  if (rtb_Add1_ge > 1.0F) {
    rtb_Add1_ge = 1.0F;
  } else {
    if (rtb_Add1_ge < 0.0F) {
      /* Switch: '<S339>/Switch' incorporates:
       *  Constant: '<S338>/IAM_Ts_P1'
       */
      rtb_Add1_ge = 0.0F;
    }
  }

  /* End of Switch: '<S339>/Switch2' */

  /* Sum: '<S338>/Add' incorporates:
   *  Product: '<S338>/Product'
   *  Sum: '<S338>/Subtract'
   */
  EST_RawFltDYDotPObs = (EST_DYDotPobs - EST_RawFltDYDotPObs) * rtb_Add1_ge +
    EST_RawFltDYDotPObs;

  /* MultiPortSwitch: '<S336>/Multiport Switch' incorporates:
   *  RelationalOperator: '<S336>/Equal'
   */
  if (EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S336>/Multiport Switch' incorporates:
     *  Constant: '<S336>/Constant4'
     */
    EST_FltDYDotPObs = 0.0F;
  } else {
    /* MultiPortSwitch: '<S336>/Multiport Switch' */
    EST_FltDYDotPObs = EST_RawFltDYDotPObs;
  }

  /* End of MultiPortSwitch: '<S336>/Multiport Switch' */

  /* Product: '<S335>/Product' incorporates:
   *  Abs: '<S335>/Abs'
   *  Constant: '<S335>/Constant'
   *  Constant: '<S335>/Parameter1'
   *  Constant: '<S335>/Parameter2'
   *  Constant: '<S335>/Parameter3'
   *  Product: '<S335>/Divide'
   *  Sum: '<S335>/Subtract'
   *  Sum: '<S335>/Subtract1'
   */
  EST_RawHdrPercByDY = (fabsf(EST_FltDYDotPObs) - EST_MinDYDotHdr_P) /
    (EST_MaxDYDotHdr_P - EST_MinDYDotHdr_P) * 100.0F;

  /* Switch: '<S337>/Switch2' incorporates:
   *  Constant: '<S335>/Constant1'
   *  Constant: '<S335>/Constant2'
   *  RelationalOperator: '<S337>/LowerRelop1'
   *  RelationalOperator: '<S337>/UpperRelop'
   *  Switch: '<S337>/Switch'
   */
  if (EST_RawHdrPercByDY > 100.0F) {
    /* Switch: '<S337>/Switch2' */
    EST_LmtHdrPercByDY = 100.0F;
  } else if (EST_RawHdrPercByDY < 0.0F) {
    /* Switch: '<S337>/Switch' incorporates:
     *  Constant: '<S335>/Constant2'
     *  Switch: '<S337>/Switch2'
     */
    EST_LmtHdrPercByDY = 0.0F;
  } else {
    /* Switch: '<S337>/Switch2' incorporates:
     *  Switch: '<S337>/Switch'
     */
    EST_LmtHdrPercByDY = EST_RawHdrPercByDY;
  }

  /* End of Switch: '<S337>/Switch2' */

  /* MultiPortSwitch: '<S335>/Multiport Switch1' incorporates:
   *  Logic: '<S335>/AND'
   *  Logic: '<S335>/NOT'
   */
  if ((!EST_EnaCntrlByTgq) || EST_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S335>/Multiport Switch1' incorporates:
     *  Constant: '<S335>/Constant5'
     */
    EST_HdrPercByDY = 0.0F;
  } else {
    /* MultiPortSwitch: '<S335>/Multiport Switch1' */
    EST_HdrPercByDY = EST_LmtHdrPercByDY;
  }

  /* End of MultiPortSwitch: '<S335>/Multiport Switch1' */

  /* Product: '<S346>/Divide' incorporates:
   *  Constant: '<S346>/Constant'
   *  Product: '<S346>/Product'
   */
  EST_MulHdrPerc = (real32_T)(EST_HdrPercByTheta * EST_HdrPercByDY / 100.0);

  /* MultiPortSwitch: '<S345>/Multiport Switch' incorporates:
   *  Constant: '<S345>/Parameter'
   *  Constant: '<S345>/Parameter1'
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   *  Logic: '<S345>/AND'
   *  RelationalOperator: '<S345>/Equal'
   *  RelationalOperator: '<S345>/Equal1'
   */
  if ((TCTI_StCntrlFcn == EST_StMainLcfOff_P) || (TCTI_StCntrlFcn ==
       EST_StMainLcfTJA_P)) {
    /* MultiPortSwitch: '<S345>/Multiport Switch1' incorporates:
     *  Constant: '<S345>/Parameter2'
     *  Inport: '<Root>/TCTI _StLatCtrlMode'
     *  RelationalOperator: '<S345>/Equal2'
     */
    if (TCTI_StLatCtrlMode != EST_ModeTJALatCtrlOf_P) {
      /* MultiPortSwitch: '<S345>/Multiport Switch' incorporates:
       *  Constant: '<S345>/Constant'
       */
      EST_ModeSelParHdr = 2U;
    } else {
      /* MultiPortSwitch: '<S345>/Multiport Switch' incorporates:
       *  Constant: '<S345>/Constant2'
       */
      EST_ModeSelParHdr = 1U;
    }

    /* End of MultiPortSwitch: '<S345>/Multiport Switch1' */
  } else {
    /* MultiPortSwitch: '<S345>/Multiport Switch' incorporates:
     *  Constant: '<S345>/Constant1'
     */
    EST_ModeSelParHdr = 3U;
  }

  /* End of MultiPortSwitch: '<S345>/Multiport Switch' */

  /* MultiPortSwitch: '<S348>/Multiport Switch2' */
  switch (EST_ModeSelParHdr) {
   case 1:
    /* MultiPortSwitch: '<S348>/Multiport Switch2' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table3'
     */
    EST_ThdMulHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMulHdr_X[0])), ((const real32_T *)&(EST_ThdMulHdr_M[0])), 12U);
    break;

   case 2:
    /* MultiPortSwitch: '<S348>/Multiport Switch2' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table4'
     */
    EST_ThdMulHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMulHdrOf_X[0])), ((const real32_T *)&(EST_ThdMulHdrOf_M[0])), 12U);
    break;

   default:
    /* MultiPortSwitch: '<S348>/Multiport Switch2' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table5'
     */
    EST_ThdMulHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMulHdrSf_X[0])), ((const real32_T *)&(EST_ThdMulHdrSf_M[0])), 12U);
    break;
  }

  /* End of MultiPortSwitch: '<S348>/Multiport Switch2' */

  /* RelationalOperator: '<S346>/GreaterThan' */
  EST_EnaByMulHdrPerc = (EST_ThdMulHdrSel < EST_MulHdrPerc);

  /* MultiPortSwitch: '<S349>/Multiport Switch3' */
  switch (EST_ModeSelParHdr) {
   case 1:
    /* MultiPortSwitch: '<S349>/Multiport Switch3' incorporates:
     *  Constant: '<S349>/Parameter1'
     */
    EST_FacDThetaWghtHdrSel = EST_FacDThetaWghtHdr_P;
    break;

   case 2:
    /* MultiPortSwitch: '<S349>/Multiport Switch3' incorporates:
     *  Constant: '<S349>/Parameter2'
     */
    EST_FacDThetaWghtHdrSel = EST_FacDThetaWghtHdrOf_P;
    break;

   default:
    /* MultiPortSwitch: '<S349>/Multiport Switch3' incorporates:
     *  Constant: '<S349>/Parameter3'
     */
    EST_FacDThetaWghtHdrSel = EST_FacDThetaWghtHdrSf_P;
    break;
  }

  /* End of MultiPortSwitch: '<S349>/Multiport Switch3' */

  /* MultiPortSwitch: '<S349>/Multiport Switch4' */
  switch (EST_ModeSelParHdr) {
   case 1:
    /* MultiPortSwitch: '<S349>/Multiport Switch4' incorporates:
     *  Constant: '<S349>/Parameter4'
     */
    EST_FacDYWghtHdrSel = EST_FacDYWghtHdr_P;
    break;

   case 2:
    /* MultiPortSwitch: '<S349>/Multiport Switch4' incorporates:
     *  Constant: '<S349>/Parameter5'
     */
    EST_FacDYWghtHdrSel = EST_FacDYWghtHdrOf_P;
    break;

   default:
    /* MultiPortSwitch: '<S349>/Multiport Switch4' incorporates:
     *  Constant: '<S349>/Parameter6'
     */
    EST_FacDYWghtHdrSel = EST_FacDYWghtHdrSf_P;
    break;
  }

  /* End of MultiPortSwitch: '<S349>/Multiport Switch4' */

  /* Sum: '<S346>/Add1' */
  rtb_Add1_ge = EST_FacDThetaWghtHdrSel + EST_FacDYWghtHdrSel;

  /* MultiPortSwitch: '<S348>/Multiport Switch1' */
  switch (EST_ModeSelParHdr) {
   case 1:
    /* MultiPortSwitch: '<S348>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table'
     */
    EST_ThdMeanHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMeanHdr_X[0])), ((const real32_T *)&(EST_ThdMeanHdr_M[0])), 12U);
    break;

   case 2:
    /* MultiPortSwitch: '<S348>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table1'
     */
    EST_ThdMeanHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMeanHdrOf_X[0])), ((const real32_T *)&(EST_ThdMeanHdrOf_M[0])),
      12U);
    break;

   default:
    /* MultiPortSwitch: '<S348>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S348>/1-D Lookup Table2'
     */
    EST_ThdMeanHdrSel = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
      &(EST_ThdMeanHdrSf_X[0])), ((const real32_T *)&(EST_ThdMeanHdrSf_M[0])),
      12U);
    break;
  }

  /* End of MultiPortSwitch: '<S348>/Multiport Switch1' */

  /* RelationalOperator: '<S346>/GreaterThan1' */
  EST_EnaByMeanHdr = (rtb_Add1_ge >= EST_ThdMeanHdrSel);

  /* Sum: '<S346>/Add2' incorporates:
   *  ArithShift: '<S346>/Shift Arithmetic'
   *  ArithShift: '<S346>/Shift Arithmetic1'
   *  Constant: '<S346>/Constant1'
   *  Constant: '<S346>/Constant2'
   *  DataTypeConversion: '<S346>/Data Type Conversion'
   *  DataTypeConversion: '<S346>/Data Type Conversion1'
   */
  EST_BtfQualifierByHdr = (uint16_T)((uint32_T)(EST_EnaByMeanHdr << 13U) +
    (EST_EnaByMulHdrPerc << 12U));

  /* Switch: '<S329>/Init' incorporates:
   *  Logic: '<S329>/FixPt Logical Operator'
   *  UnitDelay: '<S329>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_b != 0)) {
    /* Switch: '<S329>/Init' incorporates:
     *  Constant: '<S313>/Constant13'
     */
    EST_RawEstDYPobs = 0.0F;
  } else {
    /* Switch: '<S329>/Init' incorporates:
     *  UnitDelay: '<S329>/FixPt Unit Delay1'
     */
    EST_RawEstDYPobs = TJATCT_DW.FixPtUnitDelay1_DSTATE_g;
  }

  /* End of Switch: '<S329>/Init' */

  /* Switch: '<S321>/Switch2' incorporates:
   *  Constant: '<S313>/Constant14'
   *  Constant: '<S313>/Constant15'
   *  RelationalOperator: '<S321>/LowerRelop1'
   *  RelationalOperator: '<S321>/UpperRelop'
   *  Switch: '<S321>/Switch'
   */
  if (EST_RawEstDYPobs > 10.0F) {
    /* Switch: '<S321>/Switch2' */
    EST_EstDYPobs = 10.0F;
  } else if (EST_RawEstDYPobs < -10.0F) {
    /* Switch: '<S321>/Switch' incorporates:
     *  Constant: '<S313>/Constant15'
     *  Switch: '<S321>/Switch2'
     */
    EST_EstDYPobs = -10.0F;
  } else {
    /* Switch: '<S321>/Switch2' incorporates:
     *  Switch: '<S321>/Switch'
     */
    EST_EstDYPobs = EST_RawEstDYPobs;
  }

  /* End of Switch: '<S321>/Switch2' */

  /* Sum: '<S366>/Subtract3' incorporates:
   *  UnitDelay: '<S366>/Unit Delay2'
   */
  TJATCT_DW.UnitDelay2_DSTATE_b = EST_EstDYPobs - TJATCT_DW.UnitDelay2_DSTATE_b;

  /* Lookup_n-D: '<S366>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  EST_DistPObsDYGrdntThd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const
    real32_T *)&(EST_DistPObsDYGrdntThd_X[0])), ((const real32_T *)
    &(EST_DistPObsDYGrdntThd_M[0])), 12U);

  /* Product: '<S366>/Product1' incorporates:
   *  Constant: '<S366>/Parameter9'
   *  MultiPortSwitch: '<S369>/Multiport Switch1'
   *  Product: '<S369>/Product'
   */
  rtb_Product1_p4 = EST_DistPObsDYGrdntThd * EST_TiSysCycle_P;

  /* RelationalOperator: '<S366>/GreaterThan3' incorporates:
   *  Abs: '<S366>/Abs3'
   *  Product: '<S366>/Product1'
   *  UnitDelay: '<S366>/Unit Delay2'
   */
  EST_EnaPObsDYGrdntRaw = (fabsf(TJATCT_DW.UnitDelay2_DSTATE_b) >
    rtb_Product1_p4);

  /* Logic: '<S366>/AND' */
  EST_EnaPObsDYGrdnt = (rtb_Equal3_m && EST_EnaPObsDYGrdntRaw);

  /* Lookup_n-D: '<S368>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  EST_DistPObsDYThd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(EST_EST_DistPObsDYThd_X[0])), ((const real32_T *)
    &(EST_EST_DistPObsDYThd_M[0])), 12U);

  /* Sum: '<S369>/Sum' incorporates:
   *  MultiPortSwitch: '<S369>/Multiport Switch1'
   *  UnitDelay: '<S369>/Unit Delay'
   */
  rtb_Sum_db = EST_EstDYPobs - EST_DistPobsDYGrdntRaw;

  /* Switch: '<S370>/Switch2' incorporates:
   *  MultiPortSwitch: '<S369>/Multiport Switch1'
   *  RelationalOperator: '<S370>/LowerRelop1'
   */
  if (rtb_Sum_db > rtb_Product1_p4) {
    rtb_Sum_db = rtb_Product1_p4;
  } else {
    /* Product: '<S369>/Product1' incorporates:
     *  Constant: '<S366>/Parameter3'
     *  UnaryMinus: '<S366>/Unary Minus1'
     */
    rtb_Product1_p4 = -EST_DistPObsDYGrdntThd * EST_TiSysCycle_P;

    /* Switch: '<S370>/Switch' incorporates:
     *  RelationalOperator: '<S370>/UpperRelop'
     */
    if (rtb_Sum_db < rtb_Product1_p4) {
      rtb_Sum_db = rtb_Product1_p4;
    }

    /* End of Switch: '<S370>/Switch' */
  }

  /* End of Switch: '<S370>/Switch2' */

  /* MultiPortSwitch: '<S369>/Multiport Switch1' incorporates:
   *  Sum: '<S369>/Difference Inputs3'
   *  UnitDelay: '<S369>/Unit Delay'
   *
   * Block description for '<S369>/Difference Inputs3':
   *
   *  Add in CPU
   */
  EST_DistPobsDYGrdntRaw = rtb_Sum_db + EST_DistPobsDYGrdntRaw;

  /* MultiPortSwitch: '<S366>/Multiport Switch' */
  if (!EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S366>/Multiport Switch' */
    EST_DistPObsDYGrdnt = EST_DistPobsDYGrdntRaw;
  } else {
    /* MultiPortSwitch: '<S366>/Multiport Switch' incorporates:
     *  Constant: '<S366>/Constant'
     */
    EST_DistPObsDYGrdnt = 0.0F;
  }

  /* End of MultiPortSwitch: '<S366>/Multiport Switch' */

  /* MultiPortSwitch: '<S366>/Multiport Switch1' incorporates:
   *  Constant: '<S366>/Parameter2'
   */
  if (EST_CswPObsDYSel_P == 0) {
    /* MultiPortSwitch: '<S366>/Multiport Switch1' */
    EST_DistPObsDYSel = EST_EstDYPobs;
  } else {
    /* MultiPortSwitch: '<S366>/Multiport Switch1' */
    EST_DistPObsDYSel = EST_DistPObsDYGrdnt;
  }

  /* End of MultiPortSwitch: '<S366>/Multiport Switch1' */

  /* RelationalOperator: '<S368>/GreaterThan' incorporates:
   *  Abs: '<S368>/Abs1'
   */
  EST_EnaPObsDYSat = (fabsf(EST_DistPObsDYSel) > EST_DistPObsDYThd);

  /* Sum: '<S404>/Add2' incorporates:
   *  ArithShift: '<S404>/Shift Arithmetic2'
   *  ArithShift: '<S404>/Shift Arithmetic3'
   *  Constant: '<S404>/Constant3'
   *  Constant: '<S404>/Constant4'
   *  DataTypeConversion: '<S404>/Data Type Conversion1'
   *  DataTypeConversion: '<S404>/Data Type Conversion2'
   *  DataTypeConversion: '<S404>/Data Type Conversion3'
   */
  EST_BtfQualifierByEna = (uint16_T)(((uint32_T)(EST_EnaFreezeByTgq << 1U) +
    EST_EnaCntrlByTgq) + (EST_EnaResetByTgq << 2U));

  /* Sum: '<S395>/Subtract3' */
  EST_DistYDevStep = EST_DistYDevTrajFromCur - EST_DistYDevStep;

  /* RelationalOperator: '<S395>/GreaterThan3' incorporates:
   *  Abs: '<S395>/Abs3'
   *  Constant: '<S395>/Parameter8'
   *  Constant: '<S395>/Parameter9'
   *  Product: '<S395>/Product1'
   */
  EST_EnaActvtGrdntLmt1 = (fabsf(EST_DistYDevStep) > EST_PlObsInDYTolBndGL1 *
    EST_TiSysCycle_P);

  /* Sum: '<S396>/Subtract3' incorporates:
   *  UnitDelay: '<S396>/Unit Delay2'
   */
  TJATCT_DW.UnitDelay2_DSTATE_k = EST_DistYDevTrajFromCur -
    TJATCT_DW.UnitDelay2_DSTATE_k;

  /* Logic: '<S396>/AND' incorporates:
   *  Abs: '<S396>/Abs3'
   *  Constant: '<S396>/Parameter8'
   *  Constant: '<S396>/Parameter9'
   *  Product: '<S396>/Product1'
   *  RelationalOperator: '<S396>/GreaterThan3'
   *  UnitDelay: '<S396>/Unit Delay2'
   */
  EST_EnaActvtGrdntLmt2 = ((fabsf(TJATCT_DW.UnitDelay2_DSTATE_k) >
    EST_PlObsInDYTolBndGL2_P * EST_TiSysCycle_P) && EST_EnaLmt2ByDistY);

  /* Lookup_n-D: '<S374>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  EST_ThdBetaSatSObs = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(EST_ThdBetaSatSObs_X[0])), ((const real32_T *)&(EST_ThdBetaSatSObs_M[0])),
    6U);

  /* RelationalOperator: '<S374>/GreaterThan' incorporates:
   *  Abs: '<S374>/Abs1'
   */
  EST_EnaBetaSatSObs = (fabsf(EST_RawBetaSObs) > EST_ThdBetaSatSObs);

  /* ArithShift: '<S374>/Shift Arithmetic' incorporates:
   *  Constant: '<S374>/Constant2'
   *  DataTypeConversion: '<S374>/Data Type Conversion'
   */
  EST_BtfQualifierByBeta = (uint16_T)(EST_EnaBetaSatSObs << 3U);

  /* Switch: '<S327>/Init' incorporates:
   *  Logic: '<S327>/FixPt Logical Operator'
   *  UnitDelay: '<S327>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_o != 0)) {
    /* Switch: '<S327>/Init' incorporates:
     *  Constant: '<S313>/Constant11'
     */
    EST_RawEstDThetaPobs = 0.0F;
  } else {
    /* Switch: '<S327>/Init' incorporates:
     *  UnitDelay: '<S327>/FixPt Unit Delay1'
     */
    EST_RawEstDThetaPobs = TJATCT_DW.FixPtUnitDelay1_DSTATE_e;
  }

  /* End of Switch: '<S327>/Init' */

  /* Switch: '<S320>/Switch2' incorporates:
   *  Constant: '<S313>/Constant18'
   *  Constant: '<S313>/Constant19'
   *  RelationalOperator: '<S320>/LowerRelop1'
   *  RelationalOperator: '<S320>/UpperRelop'
   *  Switch: '<S320>/Switch'
   */
  if (EST_RawEstDThetaPobs > 3.1415) {
    /* Switch: '<S320>/Switch2' */
    EST_EstDThetaPobs = 3.1415F;
  } else if (EST_RawEstDThetaPobs < -3.1415) {
    /* Switch: '<S320>/Switch' incorporates:
     *  Constant: '<S313>/Constant19'
     *  Switch: '<S320>/Switch2'
     */
    EST_EstDThetaPobs = -3.1415F;
  } else {
    /* Switch: '<S320>/Switch2' incorporates:
     *  Switch: '<S320>/Switch'
     */
    EST_EstDThetaPobs = EST_RawEstDThetaPobs;
  }

  /* End of Switch: '<S320>/Switch2' */

  /* Sum: '<S355>/Subtract3' incorporates:
   *  UnitDelay: '<S355>/Unit Delay2'
   */
  TJATCT_DW.UnitDelay2_DSTATE_c = EST_EstDThetaPobs -
    TJATCT_DW.UnitDelay2_DSTATE_c;

  /* Lookup_n-D: '<S355>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  EST_AnglePObsDThetaThd0 = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const
    real32_T *)&(EST_AnglePObsDThetaThd0_X[0])), ((const real32_T *)
    &(EST_AnglePObsDThetaThd0_M[0])), 12U);

  /* Product: '<S355>/Product1' incorporates:
   *  Constant: '<S355>/Parameter9'
   *  MultiPortSwitch: '<S359>/Multiport Switch1'
   *  Product: '<S359>/Product'
   */
  rtb_Product1_p4 = EST_AnglePObsDThetaThd0 * EST_TiSysCycle_P;

  /* Logic: '<S355>/AND' incorporates:
   *  Abs: '<S355>/Abs3'
   *  Product: '<S355>/Product1'
   *  RelationalOperator: '<S355>/GreaterThan3'
   *  UnitDelay: '<S355>/Unit Delay2'
   */
  EST_EnaPObsDThetaLmt0 = (rtb_Equal3_m && (fabsf(TJATCT_DW.UnitDelay2_DSTATE_c)
    > rtb_Product1_p4));

  /* Sum: '<S356>/Subtract3' incorporates:
   *  UnitDelay: '<S356>/Unit Delay2'
   */
  TJATCT_DW.UnitDelay2_DSTATE_a = EST_EstDThetaPobs -
    TJATCT_DW.UnitDelay2_DSTATE_a;

  /* Product: '<S356>/Product1' incorporates:
   *  Constant: '<S356>/Parameter8'
   *  Constant: '<S356>/Parameter9'
   *  MultiPortSwitch: '<S361>/Multiport Switch1'
   *  Product: '<S357>/Product2'
   *  Product: '<S361>/Product'
   */
  rtb_Sum_db = EST_AnglePobsDThetaLmt1_P * EST_TiSysCycle_P;

  /* RelationalOperator: '<S356>/GreaterThan3' incorporates:
   *  Abs: '<S356>/Abs3'
   *  Product: '<S356>/Product1'
   *  UnitDelay: '<S356>/Unit Delay2'
   */
  EST_EnaPObsDThetaLmt1 = (fabsf(TJATCT_DW.UnitDelay2_DSTATE_a) > rtb_Sum_db);

  /* Logic: '<S356>/NOT' incorporates:
   *  Abs: '<S356>/Abs'
   *  Constant: '<S356>/Parameter1'
   *  RelationalOperator: '<S356>/GreaterThan'
   *  UnitDelay: '<S356>/Unit Delay'
   */
  EST_EnaPObsDThetaRst1 = (fabsf(EST_AnglePObsDThetaLmt1) <
    EST_AnglePobsDThetaThd1_P);

  /* Logic: '<S357>/AND1' incorporates:
   *  Logic: '<S356>/OR'
   */
  rtb_Equal3_b = !EST_EnaPObsDThetaRst1;

  /* Logic: '<S357>/OR1' incorporates:
   *  Abs: '<S357>/Abs1'
   *  Constant: '<S357>/Parameter4'
   *  Logic: '<S357>/AND1'
   *  Logic: '<S357>/NOT1'
   *  RelationalOperator: '<S357>/GreaterThan1'
   *  UnitDelay: '<S357>/Unit Delay1'
   */
  EST_EnaPObsDThetaRst2 = (rtb_Equal3_b || (fabsf(EST_AnglePObsDThetaLmt2) <
    EST_AnglePobsDThetaThd2_P) || EST_EnaResetByTgq);

  /* Sum: '<S357>/Subtract1' incorporates:
   *  UnitDelay: '<S357>/Unit Delay3'
   */
  TJATCT_DW.UnitDelay3_DSTATE_p = EST_EstDThetaPobs -
    TJATCT_DW.UnitDelay3_DSTATE_p;

  /* Logic: '<S357>/AND' incorporates:
   *  Abs: '<S357>/Abs2'
   *  RelationalOperator: '<S357>/GreaterThan2'
   *  UnitDelay: '<S357>/Unit Delay3'
   */
  EST_EnaPObsDThetaLmt2 = ((fabsf(TJATCT_DW.UnitDelay3_DSTATE_p) > rtb_Sum_db) &&
    EST_EnaPObsDThetaRst2);

  /* Lookup_n-D: '<S354>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  EST_AnglePObsDThetaThd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const
    real32_T *)&(EST_AnglePObsDThetaThd_X[0])), ((const real32_T *)
    &(EST_AnglePObsDThetaThd_M[0])), 12U);

  /* Sum: '<S359>/Sum' incorporates:
   *  MultiPortSwitch: '<S359>/Multiport Switch1'
   *  UnitDelay: '<S359>/Unit Delay'
   */
  rtb_Sum_lu = EST_EstDThetaPobs - EST_AnglePObsDThetaLmt0Raw;

  /* Switch: '<S360>/Switch2' incorporates:
   *  MultiPortSwitch: '<S359>/Multiport Switch1'
   *  RelationalOperator: '<S360>/LowerRelop1'
   */
  if (rtb_Sum_lu > rtb_Product1_p4) {
    rtb_Sum_lu = rtb_Product1_p4;
  } else {
    /* Product: '<S359>/Product1' incorporates:
     *  Constant: '<S355>/Parameter3'
     *  UnaryMinus: '<S355>/Unary Minus1'
     */
    rtb_Product1_p4 = -EST_AnglePObsDThetaThd0 * EST_TiSysCycle_P;

    /* Switch: '<S360>/Switch' incorporates:
     *  RelationalOperator: '<S360>/UpperRelop'
     */
    if (rtb_Sum_lu < rtb_Product1_p4) {
      rtb_Sum_lu = rtb_Product1_p4;
    }

    /* End of Switch: '<S360>/Switch' */
  }

  /* End of Switch: '<S360>/Switch2' */

  /* MultiPortSwitch: '<S359>/Multiport Switch1' incorporates:
   *  Sum: '<S359>/Difference Inputs3'
   *  UnitDelay: '<S359>/Unit Delay'
   *
   * Block description for '<S359>/Difference Inputs3':
   *
   *  Add in CPU
   */
  EST_AnglePObsDThetaLmt0Raw = rtb_Sum_lu + EST_AnglePObsDThetaLmt0Raw;

  /* MultiPortSwitch: '<S355>/Multiport Switch' incorporates:
   *  RelationalOperator: '<S355>/Equal'
   */
  if (EST_EnaResetByTgq) {
    /* MultiPortSwitch: '<S355>/Multiport Switch' incorporates:
     *  Constant: '<S355>/Constant2'
     */
    EST_AnglePObsDThetaLmt0 = 0.0F;
  } else {
    /* MultiPortSwitch: '<S355>/Multiport Switch' */
    EST_AnglePObsDThetaLmt0 = EST_AnglePObsDThetaLmt0Raw;
  }

  /* End of MultiPortSwitch: '<S355>/Multiport Switch' */

  /* MultiPortSwitch: '<S361>/Multiport Switch1' incorporates:
   *  Logic: '<S356>/OR'
   */
  if (rtb_Equal3_b && rtb_Equal3_m) {
    /* MultiPortSwitch: '<S361>/Multiport Switch1' incorporates:
     *  UnitDelay: '<S361>/Unit Delay'
     */
    EST_AnglePObsDThetaLmt1 = TJATCT_DW.UnitDelay_DSTATE_b1;
  } else {
    /* Sum: '<S361>/Sum' incorporates:
     *  UnitDelay: '<S361>/Unit Delay'
     */
    rtb_Product1_p4 = EST_EstDThetaPobs - TJATCT_DW.UnitDelay_DSTATE_b1;

    /* Switch: '<S362>/Switch2' incorporates:
     *  RelationalOperator: '<S362>/LowerRelop1'
     */
    if (rtb_Product1_p4 > rtb_Sum_db) {
      rtb_Product1_p4 = rtb_Sum_db;
    } else {
      /* Product: '<S361>/Product1' incorporates:
       *  Constant: '<S356>/Parameter2'
       *  Constant: '<S356>/Parameter3'
       *  UnaryMinus: '<S356>/Unary Minus'
       */
      rtb_Sum_db = -EST_AnglePobsDThetaLmt1_P * EST_TiSysCycle_P;

      /* Switch: '<S362>/Switch' incorporates:
       *  RelationalOperator: '<S362>/UpperRelop'
       */
      if (rtb_Product1_p4 < rtb_Sum_db) {
        rtb_Product1_p4 = rtb_Sum_db;
      }

      /* End of Switch: '<S362>/Switch' */
    }

    /* End of Switch: '<S362>/Switch2' */

    /* MultiPortSwitch: '<S361>/Multiport Switch1' incorporates:
     *  Sum: '<S361>/Difference Inputs3'
     *  UnitDelay: '<S361>/Unit Delay'
     *
     * Block description for '<S361>/Difference Inputs3':
     *
     *  Add in CPU
     */
    EST_AnglePObsDThetaLmt1 = rtb_Product1_p4 + TJATCT_DW.UnitDelay_DSTATE_b1;
  }

  /* MultiPortSwitch: '<S363>/Multiport Switch1' */
  if (!EST_EnaPObsDThetaRst2) {
    /* MultiPortSwitch: '<S363>/Multiport Switch1' incorporates:
     *  UnitDelay: '<S363>/Unit Delay'
     */
    EST_AnglePObsDThetaLmt2 = TJATCT_DW.UnitDelay_DSTATE_f;
  } else {
    /* Sum: '<S363>/Sum' incorporates:
     *  UnitDelay: '<S363>/Unit Delay'
     */
    rtb_Sum_db = EST_AnglePObsDThetaLmt1 - TJATCT_DW.UnitDelay_DSTATE_f;

    /* Product: '<S363>/Product' incorporates:
     *  Constant: '<S357>/Parameter5'
     *  Constant: '<S357>/Parameter6'
     */
    rtb_Product1_p4 = EST_AnglePobsDThetaLmt2_P * EST_TiSysCycle_P;

    /* Switch: '<S364>/Switch2' incorporates:
     *  RelationalOperator: '<S364>/LowerRelop1'
     */
    if (rtb_Sum_db > rtb_Product1_p4) {
      rtb_Sum_db = rtb_Product1_p4;
    } else {
      /* Product: '<S363>/Product1' incorporates:
       *  Constant: '<S357>/Parameter5'
       *  Constant: '<S357>/Parameter6'
       *  UnaryMinus: '<S357>/Unary Minus1'
       */
      rtb_Product1_p4 = -EST_AnglePobsDThetaLmt2_P * EST_TiSysCycle_P;

      /* Switch: '<S364>/Switch' incorporates:
       *  RelationalOperator: '<S364>/UpperRelop'
       */
      if (rtb_Sum_db < rtb_Product1_p4) {
        rtb_Sum_db = rtb_Product1_p4;
      }

      /* End of Switch: '<S364>/Switch' */
    }

    /* End of Switch: '<S364>/Switch2' */

    /* MultiPortSwitch: '<S363>/Multiport Switch1' incorporates:
     *  Sum: '<S363>/Difference Inputs3'
     *  UnitDelay: '<S363>/Unit Delay'
     *
     * Block description for '<S363>/Difference Inputs3':
     *
     *  Add in CPU
     */
    EST_AnglePObsDThetaLmt2 = rtb_Sum_db + TJATCT_DW.UnitDelay_DSTATE_f;
  }

  /* End of MultiPortSwitch: '<S363>/Multiport Switch1' */

  /* MultiPortSwitch: '<S358>/Multiport Switch1' incorporates:
   *  Constant: '<S358>/Parameter'
   */
  switch (EST_CswPObsDThetaLmt_P) {
   case 1:
    /* MultiPortSwitch: '<S358>/Multiport Switch1' */
    EST_AnglePObsDThetaSel = EST_EstDThetaPobs;
    break;

   case 2:
    /* MultiPortSwitch: '<S358>/Multiport Switch1' */
    EST_AnglePObsDThetaSel = EST_AnglePObsDThetaLmt0;
    break;

   default:
    /* MultiPortSwitch: '<S358>/Multiport Switch1' */
    EST_AnglePObsDThetaSel = EST_AnglePObsDThetaLmt2;
    break;
  }

  /* End of MultiPortSwitch: '<S358>/Multiport Switch1' */

  /* RelationalOperator: '<S354>/GreaterThan' incorporates:
   *  Abs: '<S354>/Abs1'
   */
  EST_EnaPObsDThetaSat = (fabsf(EST_AnglePObsDThetaSel) > EST_AnglePObsDThetaThd);

  /* Sum: '<S298>/Add4' incorporates:
   *  ArithShift: '<S353>/Shift Arithmetic'
   *  ArithShift: '<S353>/Shift Arithmetic1'
   *  ArithShift: '<S353>/Shift Arithmetic2'
   *  ArithShift: '<S353>/Shift Arithmetic3'
   *  ArithShift: '<S367>/Shift Arithmetic2'
   *  ArithShift: '<S367>/Shift Arithmetic3'
   *  ArithShift: '<S397>/Shift Arithmetic'
   *  ArithShift: '<S397>/Shift Arithmetic1'
   *  Constant: '<S353>/Constant1'
   *  Constant: '<S353>/Constant2'
   *  Constant: '<S353>/Constant3'
   *  Constant: '<S353>/Constant4'
   *  Constant: '<S367>/Constant3'
   *  Constant: '<S367>/Constant4'
   *  Constant: '<S397>/Constant1'
   *  Constant: '<S397>/Constant2'
   *  DataTypeConversion: '<S353>/Data Type Conversion'
   *  DataTypeConversion: '<S353>/Data Type Conversion1'
   *  DataTypeConversion: '<S353>/Data Type Conversion2'
   *  DataTypeConversion: '<S353>/Data Type Conversion3'
   *  DataTypeConversion: '<S367>/Data Type Conversion2'
   *  DataTypeConversion: '<S367>/Data Type Conversion3'
   *  DataTypeConversion: '<S397>/Data Type Conversion'
   *  DataTypeConversion: '<S397>/Data Type Conversion1'
   *  Sum: '<S298>/Add'
   *  Sum: '<S298>/Add1'
   *  Sum: '<S298>/Add2'
   *  Sum: '<S298>/Add3'
   */
  TJATCT_Y.S_TCTEST_QualifierService_nu = (uint16_T)((uint32_T)(uint16_T)
    ((uint32_T)(uint16_T)((uint32_T)(uint16_T)((uint32_T)(uint16_T)((uint32_T)
    (uint16_T)((uint32_T)(uint16_T)((uint32_T)(uint16_T)((uint32_T)(uint16_T)
    ((uint32_T)(uint16_T)((uint32_T)(EST_EnaActvtGrdntLmt1 << 4U) +
    EST_BtfQualifierByEna) + (EST_EnaActvtGrdntLmt2 << 5U)) +
    EST_BtfQualifierByBeta) + (EST_EnaPObsDThetaLmt0 << 6U)) +
               (EST_EnaPObsDThetaLmt1 << 7U)) + (EST_EnaPObsDThetaLmt2 << 8U)) +
    (EST_EnaPObsDThetaSat << 9U)) + (EST_EnaPObsDYGrdnt << 10U)) +
     (EST_EnaPObsDYSat << 11U)) + EST_BtfQualifierByHdr);

  /* Logic: '<S86>/AND2' incorporates:
   *  Constant: '<S86>/from_data_definition1'
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   *  RelationalOperator: '<S86>/EQ'
   *  RelationalOperator: '<S86>/NEQ'
   *  UnitDelay: '<S86>/Unit_Delay'
   */
  rtb_AND2 = ((TJATCT_DW.Unit_Delay_DSTATE_c == TCTI_StCntrlFcn) &&
              (TCTI_StCntrlFcn != CDC_StLcfOff_P));

  /* RelationalOperator: '<S85>/Equal1' incorporates:
   *  Constant: '<S85>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  CDC_EnaCntrlByTgq = (TCTI_BtfTrajGuiQualifier != CDC_EnaOffByTgq_P);

  /* RelationalOperator: '<S85>/Equal' incorporates:
   *  Constant: '<S85>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  CDC_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == CDC_EnaFreezeByTgq_P);

  /* Switch: '<S371>/Switch2' incorporates:
   *  RelationalOperator: '<S371>/LowerRelop1'
   *  RelationalOperator: '<S371>/UpperRelop'
   *  Switch: '<S371>/Switch'
   *  UnaryMinus: '<S368>/Unary Minus'
   */
  if (EST_DistPObsDYSel > EST_DistPObsDYThd) {
    /* Switch: '<S371>/Switch2' */
    EST_DistPObsDYSat = EST_DistPObsDYThd;
  } else if (EST_DistPObsDYSel < -EST_DistPObsDYThd) {
    /* Switch: '<S371>/Switch' incorporates:
     *  Switch: '<S371>/Switch2'
     *  UnaryMinus: '<S368>/Unary Minus'
     */
    EST_DistPObsDYSat = -EST_DistPObsDYThd;
  } else {
    /* Switch: '<S371>/Switch2' incorporates:
     *  Switch: '<S371>/Switch'
     */
    EST_DistPObsDYSat = EST_DistPObsDYSel;
  }

  /* End of Switch: '<S371>/Switch2' */

  /* MultiPortSwitch: '<S368>/Multiport Switch1' */
  if (!EST_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S368>/Multiport Switch1' */
    EST_DistPObsDYFreeze = EST_DistPObsDYSat;
  }

  /* End of MultiPortSwitch: '<S368>/Multiport Switch1' */

  /* MultiPortSwitch: '<S368>/Multiport Switch' */
  if (!EST_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S368>/Multiport Switch' incorporates:
     *  Constant: '<S368>/Constant'
     */
    EST_DistPObsDY = 0.0F;
  } else {
    /* MultiPortSwitch: '<S368>/Multiport Switch' */
    EST_DistPObsDY = EST_DistPObsDYFreeze;
  }

  /* End of MultiPortSwitch: '<S368>/Multiport Switch' */

  /* Sum: '<S88>/Add' */
  CDC_RawErrDistYTpl = EST_DistYDevTrajFromCur;

  /* RelationalOperator: '<S85>/Equal3' incorporates:
   *  Constant: '<S85>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3_m = (TCTI_BtfTrajGuiQualifier == CDC_EnaFreezeByTgq_P);

  /* Logic: '<S85>/OR' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S85>/NOT'
   *  RelationalOperator: '<S96>/FixPt Relational Operator'
   *  UnitDelay: '<S96>/Delay Input1'
   *
   * Block description for '<S96>/Delay Input1':
   *
   *  Store in Global RAM
   */
  CDC_EnaResetByTgq = ((!CDC_EnaCntrlByTgq) || ((int32_T)rtb_Equal3_m < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_a) || TCTI_EnaReplanCurValues);

  /* Switch: '<S89>/Switch' */
  if (CDC_EnaResetByTgq) {
    /* Switch: '<S89>/Switch' */
    CDC_FltErrDistYTpl = CDC_RawErrDistYTpl;
  } else {
    /* Product: '<S89>/Divide' incorporates:
     *  Constant: '<S88>/P_TCTCDC_OssDeltaY_nu1'
     *  Constant: '<S88>/P_TCTCDC_OssDeltaY_nu2'
     *  Constant: '<S89>/Constant'
     *  MinMax: '<S89>/Max1'
     *  Product: '<S89>/Divide1'
     *  Sum: '<S89>/Subtract1'
     */
    rtb_Sum_db = CDC_TimeSysCycle_P / fmaxf(CDC_TimeSysCycle_P, (real32_T)
      (0.15915457091927682 / CDC_FltDistYFc) + CDC_TimeSysCycle_P);

    /* Switch: '<S90>/Switch2' incorporates:
     *  Constant: '<S89>/IAM_Ts_P1'
     *  Constant: '<S89>/IAM_Ts_P4'
     *  RelationalOperator: '<S90>/LowerRelop1'
     *  RelationalOperator: '<S90>/UpperRelop'
     *  Switch: '<S90>/Switch'
     */
    if (rtb_Sum_db > 1.0F) {
      rtb_Sum_db = 1.0F;
    } else {
      if (rtb_Sum_db < 0.0F) {
        /* Switch: '<S90>/Switch' incorporates:
         *  Constant: '<S89>/IAM_Ts_P1'
         */
        rtb_Sum_db = 0.0F;
      }
    }

    /* End of Switch: '<S90>/Switch2' */

    /* Switch: '<S89>/Switch' incorporates:
     *  Product: '<S89>/Product'
     *  Sum: '<S89>/Add'
     *  Sum: '<S89>/Subtract'
     *  UnitDelay: '<S89>/Unit Delay'
     */
    CDC_FltErrDistYTpl = (CDC_RawErrDistYTpl - CDC_FltErrDistYTpl) * rtb_Sum_db
      + CDC_FltErrDistYTpl;
  }

  /* End of Switch: '<S89>/Switch' */

  /* MultiPortSwitch: '<S87>/Multiport Switch1' incorporates:
   *  Constant: '<S87>/P_TCTCDC_OssDeltaY_nu'
   */
  switch (CDC_CswSelDistY_P) {
   case 1:
    /* MultiPortSwitch: '<S87>/Multiport Switch1' */
    CDC_RawCtrlErrDistY = CDC_RawErrDistYTpl;
    break;

   case 2:
    /* MultiPortSwitch: '<S87>/Multiport Switch1' */
    CDC_RawCtrlErrDistY = CDC_FltErrDistYTpl;
    break;

   default:
    /* MultiPortSwitch: '<S87>/Multiport Switch1' */
    CDC_RawCtrlErrDistY = EST_DistPObsDY;
    break;
  }

  /* End of MultiPortSwitch: '<S87>/Multiport Switch1' */

  /* MultiPortSwitch: '<S87>/Multiport Switch' */
  if (!CDC_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S87>/Multiport Switch' */
    CDC_HldCtrlErrDistY = CDC_RawCtrlErrDistY;
  }

  /* End of MultiPortSwitch: '<S87>/Multiport Switch' */

  /* MultiPortSwitch: '<S87>/Multiport Switch2' */
  if (!CDC_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S87>/Multiport Switch2' incorporates:
     *  Constant: '<S87>/Constant'
     */
    CDC_CtrlErrDistY = 0.0F;
  } else {
    /* MultiPortSwitch: '<S87>/Multiport Switch2' */
    CDC_CtrlErrDistY = CDC_HldCtrlErrDistY;
  }

  /* End of MultiPortSwitch: '<S87>/Multiport Switch2' */

  /* Switch: '<S365>/Switch2' incorporates:
   *  RelationalOperator: '<S365>/LowerRelop1'
   *  RelationalOperator: '<S365>/UpperRelop'
   *  Switch: '<S365>/Switch'
   *  UnaryMinus: '<S354>/Unary Minus'
   */
  if (EST_AnglePObsDThetaSel > EST_AnglePObsDThetaThd) {
    /* Switch: '<S365>/Switch2' */
    EST_AnglePObsDThetaSat = EST_AnglePObsDThetaThd;
  } else if (EST_AnglePObsDThetaSel < -EST_AnglePObsDThetaThd) {
    /* Switch: '<S365>/Switch' incorporates:
     *  Switch: '<S365>/Switch2'
     *  UnaryMinus: '<S354>/Unary Minus'
     */
    EST_AnglePObsDThetaSat = -EST_AnglePObsDThetaThd;
  } else {
    /* Switch: '<S365>/Switch2' incorporates:
     *  Switch: '<S365>/Switch'
     */
    EST_AnglePObsDThetaSat = EST_AnglePObsDThetaSel;
  }

  /* End of Switch: '<S365>/Switch2' */

  /* MultiPortSwitch: '<S354>/Multiport Switch1' */
  if (!EST_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S354>/Multiport Switch1' */
    EST_AnglePObsDThetaFreeze = EST_AnglePObsDThetaSat;
  }

  /* End of MultiPortSwitch: '<S354>/Multiport Switch1' */

  /* MultiPortSwitch: '<S354>/Multiport Switch' */
  if (!EST_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S354>/Multiport Switch' incorporates:
     *  Constant: '<S354>/Constant'
     */
    EST_AnglePObsDTheta = 0.0F;
  } else {
    /* MultiPortSwitch: '<S354>/Multiport Switch' */
    EST_AnglePObsDTheta = EST_AnglePObsDThetaFreeze;
  }

  /* End of MultiPortSwitch: '<S354>/Multiport Switch' */

  /* Sum: '<S93>/Add2' incorporates:
   *  Constant: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec2'
   *  Inport: '<Root>/TCTI_InclPrevTrajHeadTpl'
   */
  rtb_MultiportSwitch2_a = TCTI_InclPrevTrajHeadTpl + CDC_HeadingOffset;

  /* Switch: '<S388>/Switch2' incorporates:
   *  RelationalOperator: '<S388>/LowerRelop1'
   *  RelationalOperator: '<S388>/UpperRelop'
   *  Switch: '<S388>/Switch'
   *  UnaryMinus: '<S374>/Unary Minus'
   */
  if (EST_RawBetaSObs > EST_ThdBetaSatSObs) {
    /* Switch: '<S388>/Switch2' */
    EST_LmtBetaSObs = EST_ThdBetaSatSObs;
  } else if (EST_RawBetaSObs < -EST_ThdBetaSatSObs) {
    /* Switch: '<S388>/Switch' incorporates:
     *  Switch: '<S388>/Switch2'
     *  UnaryMinus: '<S374>/Unary Minus'
     */
    EST_LmtBetaSObs = -EST_ThdBetaSatSObs;
  } else {
    /* Switch: '<S388>/Switch2' incorporates:
     *  Switch: '<S388>/Switch'
     */
    EST_LmtBetaSObs = EST_RawBetaSObs;
  }

  /* End of Switch: '<S388>/Switch2' */

  /* MultiPortSwitch: '<S374>/Multiport Switch1' */
  if (!EST_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S374>/Multiport Switch1' */
    EST_HldBetaSObs = EST_LmtBetaSObs;
  }

  /* End of MultiPortSwitch: '<S374>/Multiport Switch1' */

  /* MultiPortSwitch: '<S374>/Multiport Switch' */
  if (!EST_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S374>/Multiport Switch' incorporates:
     *  Constant: '<S374>/Constant'
     */
    EST_BetaSObs = 0.0F;
  } else {
    /* MultiPortSwitch: '<S374>/Multiport Switch' */
    EST_BetaSObs = EST_HldBetaSObs;
  }

  /* End of MultiPortSwitch: '<S374>/Multiport Switch' */

  /* Sum: '<S93>/Add' incorporates:
   *  Inport: '<Root>/TCTI_ReqTrajHeadTpl'
   *  Sum: '<S93>/Sum2'
   */
  CDC_RawDeltaTheta = rtb_MultiportSwitch2_a - (TCTI_ReqTrajHeadTpl +
    EST_BetaSObs);

  /* Switch: '<S94>/Switch' */
  if (CDC_EnaResetByTgq) {
    /* Switch: '<S94>/Switch' */
    CDC_FltDeltaTheta = CDC_RawDeltaTheta;
  } else {
    /* Product: '<S94>/Divide' incorporates:
     *  Constant: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec1'
     *  Constant: '<S93>/Parameter'
     *  Constant: '<S94>/Constant'
     *  MinMax: '<S94>/Max1'
     *  Product: '<S94>/Divide1'
     *  Sum: '<S94>/Subtract1'
     */
    rtb_Sum_db = CDC_TimeSysCycle_P / fmaxf(CDC_TimeSysCycle_P, (real32_T)
      (0.15915457091927682 / CDC_FltYawFc) + CDC_TimeSysCycle_P);

    /* Switch: '<S95>/Switch2' incorporates:
     *  Constant: '<S94>/IAM_Ts_P1'
     *  Constant: '<S94>/IAM_Ts_P4'
     *  RelationalOperator: '<S95>/LowerRelop1'
     *  RelationalOperator: '<S95>/UpperRelop'
     *  Switch: '<S95>/Switch'
     */
    if (rtb_Sum_db > 1.0F) {
      rtb_Sum_db = 1.0F;
    } else {
      if (rtb_Sum_db < 0.0F) {
        /* Switch: '<S95>/Switch' incorporates:
         *  Constant: '<S94>/IAM_Ts_P1'
         */
        rtb_Sum_db = 0.0F;
      }
    }

    /* End of Switch: '<S95>/Switch2' */

    /* Switch: '<S94>/Switch' incorporates:
     *  Product: '<S94>/Product'
     *  Sum: '<S94>/Add'
     *  Sum: '<S94>/Subtract'
     *  UnitDelay: '<S94>/Unit Delay'
     */
    CDC_FltDeltaTheta = (CDC_RawDeltaTheta - CDC_FltDeltaTheta) * rtb_Sum_db +
      CDC_FltDeltaTheta;
  }

  /* End of Switch: '<S94>/Switch' */

  /* UnaryMinus: '<S92>/Neg4' incorporates:
   *  Inport: '<Root>/TCTI_ReqTrajHeadTpl'
   */
  TCTI_NegReqTrajHeadTpl = -TCTI_ReqTrajHeadTpl;

  /* MultiPortSwitch: '<S92>/Multiport Switch2' incorporates:
   *  Constant: '<S92>/Constant2'
   *  UnitDelay: '<S92>/Unit_Delay'
   */
  if (!CDC_EnaCntrlByTgq) {
    rtb_Sum_db = 0.0F;
  } else {
    rtb_Sum_db = TJATCT_DW.Unit_Delay_DSTATE_n;
  }

  /* End of MultiPortSwitch: '<S92>/Multiport Switch2' */

  /* UnitDelay: '<S92>/Unit Delay' */
  rtb_Product1_p4 = TJATCT_DW.UnitDelay_DSTATE_n;

  /* UnitDelay: '<S92>/Unit Delay1' */
  rtb_Sum_lu = TJATCT_DW.UnitDelay1_DSTATE_a;

  /* UnitDelay: '<S92>/Unit Delay2' */
  rtb_UnitDelay2_p = TJATCT_DW.UnitDelay2_DSTATE_ja;

  /* UnitDelay: '<S92>/Unit Delay3' */
  rtb_UnitDelay3_a = TJATCT_DW.UnitDelay3_DSTATE_k;

  /* UnitDelay: '<S92>/Unit Delay6' */
  rtb_UnitDelay6 = TJATCT_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S92>/Unit Delay5' */
  rtb_UnitDelay5 = TJATCT_DW.UnitDelay5_DSTATE;

  /* MultiPortSwitch: '<S92>/Multiport Switch1' incorporates:
   *  Constant: '<S92>/Constant'
   *  Constant: '<S92>/Kmc_delta_psi_obs_gain_const'
   *  Constant: '<S92>/Parameter'
   *  Inport: '<Root>/TCTI_VehYawRate'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Product: '<S92>/Kmc_delta_psi_obs_gain'
   *  Product: '<S92>/Mul2'
   *  Product: '<S92>/Mul3'
   *  Product: '<S92>/Mul4'
   *  Sum: '<S92>/Add1'
   *  Sum: '<S92>/Add3'
   *  Sum: '<S92>/Add4'
   *  Sum: '<S92>/Sum'
   *  UnitDelay: '<S92>/Unit_Delay'
   */
  if (!CDC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
     *  Constant: '<S92>/Constant1'
     */
    switch (CDC_DelayCycleNum_P) {
     case 0:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' */
      TJATCT_DW.UnitDelay7_DSTATE = rtb_Sum_db;
      break;

     case 1:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay_DSTATE_n;
      break;

     case 2:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay1'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay1_DSTATE_a;
      break;

     case 3:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay2'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay2_DSTATE_ja;
      break;

     case 4:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay3'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay3_DSTATE_k;
      break;

     case 5:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay6'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay6_DSTATE;
      break;

     case 6:
      /* MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
       *  UnitDelay: '<S92>/Unit Delay5'
       */
      TJATCT_DW.UnitDelay7_DSTATE = TJATCT_DW.UnitDelay5_DSTATE;
      break;
    }

    /* End of MultiPortSwitch: '<S92>/Multiport Switch3' */

    /* MultiPortSwitch: '<S92>/Multiport Switch' incorporates:
     *  Constant: '<S92>/Constant4'
     *  Inport: '<Root>/TCTI_ReqTrajCrvCsc'
     *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
     */
    if (CDC_CswTrajCrv_P == 0) {
      LGC_RawErrCourseDotCdc_tmp = TCTI_ReqTrajCrvCsc;
    } else {
      LGC_RawErrCourseDotCdc_tmp = TCTI_ReqTrajCrvTpl;
    }

    /* End of MultiPortSwitch: '<S92>/Multiport Switch' */
    TJATCT_DW.Unit_Delay_DSTATE_n = ((TCTI_VehicleVelX *
      LGC_RawErrCourseDotCdc_tmp - TCTI_VehYawRate) * CDC_CoeffDeltaPsiKmc_P -
      (TJATCT_DW.UnitDelay7_DSTATE - TCTI_NegReqTrajHeadTpl) *
      CDC_CoeffDeltaPsiObsKmc_P) * CDC_TimeSysCycle_P + rtb_Sum_db;
  } else {
    TJATCT_DW.Unit_Delay_DSTATE_n = TCTI_NegReqTrajHeadTpl;
  }

  /* End of MultiPortSwitch: '<S92>/Multiport Switch1' */

  /* UnaryMinus: '<S92>/Neg2' incorporates:
   *  UnitDelay: '<S92>/Unit_Delay'
   */
  CDC_EstCurHeading = -TJATCT_DW.Unit_Delay_DSTATE_n;

  /* Sum: '<S93>/Add1' incorporates:
   *  Sum: '<S93>/Sum'
   */
  CDC_EstDeltaTheta = rtb_MultiportSwitch2_a - (EST_BetaSObs + CDC_EstCurHeading);

  /* MultiPortSwitch: '<S91>/Multiport Switch' incorporates:
   *  Constant: '<S91>/Parameter'
   */
  switch (CDC_CswSelDeltaTheta_P) {
   case 0:
    /* MultiPortSwitch: '<S91>/Multiport Switch' */
    CDC_RawCtrlErrHeading = CDC_RawDeltaTheta;
    break;

   case 1:
    /* MultiPortSwitch: '<S91>/Multiport Switch' */
    CDC_RawCtrlErrHeading = CDC_FltDeltaTheta;
    break;

   case 2:
    /* MultiPortSwitch: '<S91>/Multiport Switch' */
    CDC_RawCtrlErrHeading = EST_AnglePObsDTheta;
    break;

   default:
    /* MultiPortSwitch: '<S91>/Multiport Switch' */
    CDC_RawCtrlErrHeading = CDC_EstDeltaTheta;
    break;
  }

  /* End of MultiPortSwitch: '<S91>/Multiport Switch' */

  /* MultiPortSwitch: '<S91>/Multiport Switch1' */
  if (!CDC_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S91>/Multiport Switch1' */
    CDC_HldCtrlErrHeading = CDC_RawCtrlErrHeading;
  }

  /* End of MultiPortSwitch: '<S91>/Multiport Switch1' */

  /* MultiPortSwitch: '<S91>/Multiport Switch2' */
  if (!CDC_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S91>/Multiport Switch2' incorporates:
     *  Constant: '<S91>/Constant'
     */
    CDC_CtrlErrHeading = 0.0F;
  } else {
    /* MultiPortSwitch: '<S91>/Multiport Switch2' */
    CDC_CtrlErrHeading = CDC_HldCtrlErrHeading;
  }

  /* End of MultiPortSwitch: '<S91>/Multiport Switch2' */

  /* Abs: '<S86>/Abs2' incorporates:
   *  Abs: '<S435>/Abs3'
   */
  rtb_Divide_hw = fabsf(CDC_CtrlErrDistY);

  /* MultiPortSwitch: '<S86>/Multiport Switch' incorporates:
   *  Abs: '<S86>/Abs2'
   *  Constant: '<S86>/Constant1'
   *  Constant: '<S86>/Constant2'
   *  Constant: '<S86>/P_TCTCDC_WtchdgLimYErr_m'
   *  RelationalOperator: '<S86>/GreaterThan'
   */
  if (rtb_Divide_hw <= CDC_ThdErrDistY_P) {
    tmp = 0U;
  } else {
    tmp = 1U;
  }

  /* End of MultiPortSwitch: '<S86>/Multiport Switch' */

  /* MultiPortSwitch: '<S86>/Multiport Switch1' incorporates:
   *  Abs: '<S86>/Abs1'
   *  Constant: '<S86>/Constant5'
   *  Constant: '<S86>/Constant7'
   *  Constant: '<S86>/P_TCTCDC_WtchdgLimCoAnErr_rad'
   *  RelationalOperator: '<S86>/GreaterThan1'
   */
  if (fabsf(CDC_CtrlErrHeading) <= CDC_ThdErrCourseAngle_P) {
    tmp_0 = 0U;
  } else {
    tmp_0 = 2U;
  }

  /* End of MultiPortSwitch: '<S86>/Multiport Switch1' */

  /* Sum: '<S86>/Sum' */
  CDC_RawBtfQualifier = (uint8_T)((uint32_T)tmp + tmp_0);

  /* MultiPortSwitch: '<S86>/Multiport Switch3' incorporates:
   *  Constant: '<S86>/P_TCTCDC_WtchdgActive_nu'
   *  MultiPortSwitch: '<S86>/Multiport Switch2'
   */
  if (CDC_CswWatchdogAct == 0) {
    /* MultiPortSwitch: '<S86>/Multiport Switch3' incorporates:
     *  Constant: '<S86>/Constant3'
     */
    CDC_BtfQualifier = 0U;
  } else if (!rtb_AND2) {
    /* MultiPortSwitch: '<S86>/Multiport Switch2' incorporates:
     *  Constant: '<S86>/Constant6'
     *  MultiPortSwitch: '<S86>/Multiport Switch3'
     */
    CDC_BtfQualifier = 0U;
  } else {
    /* MultiPortSwitch: '<S86>/Multiport Switch3' incorporates:
     *  MultiPortSwitch: '<S86>/Multiport Switch2'
     */
    CDC_BtfQualifier = CDC_RawBtfQualifier;
  }

  /* End of MultiPortSwitch: '<S86>/Multiport Switch3' */

  /* S-Function (sfix_bitop): '<S197>/Bitwise Operator' incorporates:
   *  Constant: '<S197>/Parameter'
   */
  DEV_EnaDeltaFGen = (uint8_T)(DEV_CswDeltaFTestSignal_P & 1);

  /* S-Function (sfix_bitop): '<S195>/Bitwise Operator' incorporates:
   *  Constant: '<S195>/Parameter'
   */
  DEV_EnaCrvGen = (uint8_T)(DEV_CswCrvTestSignal_P & 1);

  /* DataTypeConversion: '<S73>/Data Type Conversion11' */
  LQR_lateral_error = CDC_CtrlErrDistY;

  /* RelationalOperator: '<S597>/Equal1' incorporates:
   *  Constant: '<S597>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  LQR_EnaCntrlByTgq = (TCTI_BtfTrajGuiQualifier != LGC_EnaOffByTgq_P);

  /* RelationalOperator: '<S597>/Equal3' incorporates:
   *  Constant: '<S597>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3_b = (TCTI_BtfTrajGuiQualifier == LGC_EnaFreezeByTgq_P);

  /* Logic: '<S597>/NOT' incorporates:
   *  Logic: '<S597>/NOT1'
   */
  rtb_Equal3_gh = !LQR_EnaCntrlByTgq;

  /* Logic: '<S597>/OR' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S597>/NOT'
   *  RelationalOperator: '<S623>/FixPt Relational Operator'
   *  UnitDelay: '<S623>/Delay Input1'
   *
   * Block description for '<S623>/Delay Input1':
   *
   *  Store in Global RAM
   */
  LQR_EnaResetByTgq = (rtb_Equal3_gh || ((int32_T)rtb_Equal3_b < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_m) || TCTI_EnaReplanCurValues);

  /* Switch: '<S609>/Switch' incorporates:
   *  Constant: '<S604>/Parameter3'
   *  Constant: '<S609>/IAM_Ts_P2'
   *  Product: '<S609>/Divide'
   *  Sum: '<S609>/Sum4'
   *  UnitDelay: '<S609>/Unit_Delay1'
   */
  if (LQR_EnaResetByTgq) {
    rtb_Switch_gf = 0.0F;
  } else {
    rtb_Switch_gf = (LQR_lateral_error - TJATCT_DW.Unit_Delay1_DSTATE_d) /
      LGC_TimeSysCycle_P;
  }

  /* End of Switch: '<S609>/Switch' */

  /* RelationalOperator: '<S597>/Equal4' incorporates:
   *  Constant: '<S597>/Parameter5'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal4 = (TCTI_BtfTrajGuiQualifier == LGC_EnaFreezeByTgq_P);

  /* Logic: '<S597>/AND' incorporates:
   *  Constant: '<S597>/Parameter6'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S597>/OR1'
   *  RelationalOperator: '<S597>/NotEqual'
   *  RelationalOperator: '<S624>/FixPt Relational Operator'
   *  UnitDelay: '<S624>/Delay Input1'
   *
   * Block description for '<S624>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LQR_I_EnaResetByTgq = ((rtb_Equal3_gh || ((int32_T)rtb_Equal4 < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_k) || TCTI_EnaReplanCurValues) &&
    (TCTI_BtfTrajGuiQualifier != 5));

  /* Switch: '<S610>/Switch' incorporates:
   *  Product: '<S610>/Product'
   *  Sum: '<S610>/Add'
   *  Sum: '<S610>/Subtract'
   *  UnitDelay: '<S610>/Unit Delay'
   */
  if (rtb_LQR_I_EnaResetByTgq) {
    /* Switch: '<S611>/Switch2' incorporates:
     *  Constant: '<S609>/IAM_Ts_P1'
     *  Constant: '<S609>/IAM_Ts_P4'
     *  RelationalOperator: '<S611>/LowerRelop1'
     *  RelationalOperator: '<S611>/UpperRelop'
     *  Switch: '<S611>/Switch'
     *  UnitDelay: '<S610>/Unit Delay'
     */
    if (rtb_Switch_gf > 0.5F) {
      TJATCT_DW.UnitDelay_DSTATE_e5 = 0.5F;
    } else if (rtb_Switch_gf < -0.5F) {
      /* Switch: '<S611>/Switch' incorporates:
       *  Constant: '<S609>/IAM_Ts_P1'
       *  UnitDelay: '<S610>/Unit Delay'
       */
      TJATCT_DW.UnitDelay_DSTATE_e5 = -0.5F;
    } else {
      TJATCT_DW.UnitDelay_DSTATE_e5 = rtb_Switch_gf;
    }
  } else {
    /* Product: '<S610>/Divide' incorporates:
     *  Constant: '<S604>/Parameter1'
     *  Constant: '<S604>/Parameter3'
     *  Constant: '<S610>/Constant'
     *  MinMax: '<S610>/Max1'
     *  Product: '<S610>/Divide1'
     *  Sum: '<S610>/Subtract1'
     */
    rtb_Divide_eu = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P, (real32_T)
      (0.15915457091927682 / LQR_FltDeltaDistYFc) + LGC_TimeSysCycle_P);

    /* Switch: '<S612>/Switch2' incorporates:
     *  Constant: '<S610>/IAM_Ts_P1'
     *  Constant: '<S610>/IAM_Ts_P4'
     *  RelationalOperator: '<S612>/LowerRelop1'
     *  RelationalOperator: '<S612>/UpperRelop'
     *  Switch: '<S612>/Switch'
     */
    if (rtb_Divide_eu > 1.0F) {
      rtb_Divide_eu = 1.0F;
    } else {
      if (rtb_Divide_eu < 0.0F) {
        /* Switch: '<S612>/Switch' incorporates:
         *  Constant: '<S610>/IAM_Ts_P1'
         */
        rtb_Divide_eu = 0.0F;
      }
    }

    /* End of Switch: '<S612>/Switch2' */

    /* Switch: '<S611>/Switch2' incorporates:
     *  Constant: '<S609>/IAM_Ts_P1'
     *  Constant: '<S609>/IAM_Ts_P4'
     *  RelationalOperator: '<S611>/LowerRelop1'
     *  RelationalOperator: '<S611>/UpperRelop'
     *  Switch: '<S611>/Switch'
     */
    if (rtb_Switch_gf > 0.5F) {
      rtb_Switch_gf = 0.5F;
    } else {
      if (rtb_Switch_gf < -0.5F) {
        /* Switch: '<S611>/Switch' incorporates:
         *  Constant: '<S609>/IAM_Ts_P1'
         */
        rtb_Switch_gf = -0.5F;
      }
    }

    TJATCT_DW.UnitDelay_DSTATE_e5 += (rtb_Switch_gf -
      TJATCT_DW.UnitDelay_DSTATE_e5) * rtb_Divide_eu;
  }

  /* End of Switch: '<S610>/Switch' */

  /* Switch: '<S604>/Switch' incorporates:
   *  UnitDelay: '<S610>/Unit Delay'
   */
  LQR_lateral_error_rate = TJATCT_DW.UnitDelay_DSTATE_e5;

  /* DataTypeConversion: '<S73>/Data Type Conversion12' */
  LQR_heading_error = CDC_CtrlErrHeading;

  /* MultiPortSwitch: '<S80>/Multiport Switch1' incorporates:
   *  Constant: '<S80>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   *  RelationalOperator: '<S80>/Equal1'
   */
  if (TCTI_BtfTrajGuiQualifier != FFC_StTgqReqFreeze_P) {
    /* MultiPortSwitch: '<S80>/Multiport Switch' incorporates:
     *  Constant: '<S80>/Parameter'
     */
    if (FFC_CswFfcCrv_P == 0) {
      /* UnitDelay: '<S80>/Unit Delay1' incorporates:
       *  Constant: '<S80>/Constant'
       */
      FFC_HldReqFfcCrv = 0.0F;
    } else {
      /* UnitDelay: '<S80>/Unit Delay1' incorporates:
       *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
       */
      FFC_HldReqFfcCrv = TCTI_ReqTrajCrvTpl;
    }

    /* End of MultiPortSwitch: '<S80>/Multiport Switch' */
  }

  /* End of MultiPortSwitch: '<S80>/Multiport Switch1' */

  /* MultiPortSwitch: '<S80>/Multiport Switch2' incorporates:
   *  Constant: '<S80>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   *  RelationalOperator: '<S80>/Equal2'
   */
  if (TCTI_BtfTrajGuiQualifier == FFC_StTgqReqOff_P) {
    /* MultiPortSwitch: '<S80>/Multiport Switch2' incorporates:
     *  Constant: '<S80>/Constant2'
     */
    FFC_ReqFfcCrv = 0.0F;
  } else {
    /* MultiPortSwitch: '<S80>/Multiport Switch2' incorporates:
     *  UnitDelay: '<S80>/Unit Delay1'
     */
    FFC_ReqFfcCrv = FFC_HldReqFfcCrv;
  }

  /* End of MultiPortSwitch: '<S80>/Multiport Switch2' */

  /* Switch: '<S599>/Switch' */
  if (LQR_EnaResetByTgq) {
    /* Switch: '<S599>/Switch' */
    LQR_KappaFlt = FFC_ReqFfcCrv;
  } else {
    /* Product: '<S599>/Divide' incorporates:
     *  Constant: '<S599>/Constant'
     *  Constant: '<S82>/Parameter2'
     *  Constant: '<S82>/Parameter6'
     *  MinMax: '<S599>/Max1'
     *  Product: '<S599>/Divide1'
     *  Sum: '<S599>/Subtract1'
     */
    rtb_Switch_gf = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P, (real32_T)
      (0.15915457091927682 / LQR_FltKappaFc) + LGC_TimeSysCycle_P);

    /* Switch: '<S625>/Switch2' incorporates:
     *  Constant: '<S599>/IAM_Ts_P1'
     *  Constant: '<S599>/IAM_Ts_P4'
     *  RelationalOperator: '<S625>/LowerRelop1'
     *  RelationalOperator: '<S625>/UpperRelop'
     *  Switch: '<S625>/Switch'
     */
    if (rtb_Switch_gf > 1.0F) {
      rtb_Switch_gf = 1.0F;
    } else {
      if (rtb_Switch_gf < 0.0F) {
        /* Switch: '<S625>/Switch' incorporates:
         *  Constant: '<S599>/IAM_Ts_P1'
         */
        rtb_Switch_gf = 0.0F;
      }
    }

    /* End of Switch: '<S625>/Switch2' */

    /* Switch: '<S599>/Switch' incorporates:
     *  Product: '<S599>/Product'
     *  Sum: '<S599>/Add'
     *  Sum: '<S599>/Subtract'
     *  UnitDelay: '<S599>/Unit Delay'
     */
    LQR_KappaFlt = (FFC_ReqFfcCrv - LQR_KappaFlt) * rtb_Switch_gf + LQR_KappaFlt;
  }

  /* End of Switch: '<S599>/Switch' */

  /* Product: '<S604>/Product4' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LQR_ref_heading_rate = TCTI_VehicleVelX * LQR_KappaFlt;

  /* DataTypeConversion: '<S73>/Data Type Conversion15' incorporates:
   *  Inport: '<Root>/TCTI_VehYawRate'
   */
  LQR_yawrate_term = TCTI_VehYawRate;

  /* Switch: '<S632>/Switch' */
  if (LQR_EnaResetByTgq) {
    /* Switch: '<S632>/Switch' */
    LQR_YawrateFlt = LQR_yawrate_term;
  } else {
    /* Product: '<S632>/Divide' incorporates:
     *  Constant: '<S602>/Parameter1'
     *  Constant: '<S602>/Parameter2'
     *  Constant: '<S632>/Constant'
     *  MinMax: '<S632>/Max1'
     *  Product: '<S632>/Divide1'
     *  Sum: '<S632>/Subtract1'
     */
    rtb_Switch_gf = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P, (real32_T)
      (0.15915457091927682 / LQR_FltYawRateFc) + LGC_TimeSysCycle_P);

    /* Switch: '<S633>/Switch2' incorporates:
     *  Constant: '<S632>/IAM_Ts_P1'
     *  Constant: '<S632>/IAM_Ts_P4'
     *  RelationalOperator: '<S633>/LowerRelop1'
     *  RelationalOperator: '<S633>/UpperRelop'
     *  Switch: '<S633>/Switch'
     */
    if (rtb_Switch_gf > 1.0F) {
      rtb_Switch_gf = 1.0F;
    } else {
      if (rtb_Switch_gf < 0.0F) {
        /* Switch: '<S633>/Switch' incorporates:
         *  Constant: '<S632>/IAM_Ts_P1'
         */
        rtb_Switch_gf = 0.0F;
      }
    }

    /* End of Switch: '<S633>/Switch2' */

    /* Switch: '<S632>/Switch' incorporates:
     *  Product: '<S632>/Product'
     *  Sum: '<S632>/Add'
     *  Sum: '<S632>/Subtract'
     *  UnitDelay: '<S632>/Unit Delay'
     */
    LQR_YawrateFlt = (LQR_yawrate_term - LQR_YawrateFlt) * rtb_Switch_gf +
      LQR_YawrateFlt;
  }

  /* End of Switch: '<S632>/Switch' */

  /* Sum: '<S604>/Subtract' */
  LQR_heading_error_rate = LQR_ref_heading_rate - LQR_YawrateFlt;

  /* MATLAB Function: '<S595>/MATLAB Function' incorporates:
   *  Constant: '<S595>/Parameter1'
   *  Constant: '<S595>/Parameter2'
   *  Constant: '<S595>/Parameter3'
   *  Constant: '<S595>/Parameter5'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  SignalConversion generated from: '<S604>/Vector Concatenate'
   * */
  deadZone_weightedError = deadZone_weightedGain_e1 * fabsf(LQR_lateral_error) +
    deadZone_weightedGain_e2 * fabsf(LQR_heading_error_rate);
  deadZone_gainkT = 1.0F;
  if ((deadZone_width > 0.0F) && (TCTI_VehicleVelX > deadZone_Vxthd) &&
      (deadZone_weightedError < deadZone_width)) {
    deadZone_gainkT = deadZone_weightedError / deadZone_width;
  }

  /* End of MATLAB Function: '<S595>/MATLAB Function' */

  /* Product: '<S595>/Product' incorporates:
   *  Product: '<S595>/Product4'
   *  SignalConversion generated from: '<S604>/Vector Concatenate'
   *  Switch: '<S595>/Switch'
   */
  LQR_e1_contribution = deadZone_gainkT * LQR_MatK_k1 * LQR_lateral_error;

  /* Product: '<S595>/Product1' incorporates:
   *  SignalConversion generated from: '<S604>/Vector Concatenate'
   *  Switch: '<S595>/Switch'
   */
  LQR_e1dot_contribution = LQR_lateral_error_rate * LQR_MatK_k2;

  /* Product: '<S595>/Product2' incorporates:
   *  SignalConversion generated from: '<S604>/Vector Concatenate'
   *  Switch: '<S595>/Switch'
   */
  LQR_e2_contribution = LQR_heading_error * LQR_MatK_k3;

  /* Product: '<S595>/Product3' incorporates:
   *  SignalConversion generated from: '<S604>/Vector Concatenate'
   *  Switch: '<S595>/Switch'
   */
  LQR_e2dot_contribution = LQR_heading_error_rate * LQR_MatK_k4;

  /* Sum: '<S595>/Add' */
  LQR_DeltaF_feedback_rad = ((LQR_e1_contribution + LQR_e1dot_contribution) +
    LQR_e2_contribution) + LQR_e2dot_contribution;

  /* Switch: '<S616>/Init' incorporates:
   *  Logic: '<S616>/FixPt Logical Operator'
   *  UnitDelay: '<S616>/FixPt Unit Delay2'
   */
  if (rtb_LQR_I_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_c != 0)) {
    /* Switch: '<S616>/Init' incorporates:
     *  Constant: '<S605>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_o = 0.0F;
  }

  /* End of Switch: '<S616>/Init' */

  /* Switch: '<S614>/Switch2' incorporates:
   *  Constant: '<S605>/Constant2'
   *  RelationalOperator: '<S614>/LowerRelop1'
   *  RelationalOperator: '<S614>/UpperRelop'
   *  Switch: '<S614>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_o > 0.008F) {
    /* Switch: '<S614>/Switch2' */
    LQR_I_term_rad = 0.008F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_o < -0.008F) {
    /* Switch: '<S614>/Switch' incorporates:
     *  Switch: '<S614>/Switch2'
     */
    LQR_I_term_rad = -0.008F;
  } else {
    /* Switch: '<S614>/Switch2' incorporates:
     *  Switch: '<S614>/Switch'
     */
    LQR_I_term_rad = TJATCT_DW.FixPtUnitDelay1_DSTATE_o;
  }

  /* End of Switch: '<S614>/Switch2' */

  /* Product: '<S82>/Product' incorporates:
   *  Constant: '<S596>/Parameter'
   *  Constant: '<S596>/Parameter1'
   *  Constant: '<S598>/Constant1'
   *  Inport: '<Root>/Inport4'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Lookup_n-D: '<S598>/1-D Lookup Table4'
   *  MATLAB Function: '<S596>/MATLAB Function'
   */
  LQR_DeltaF_feedforward_rad = (EST_DistCogToFrontAxle_P +
    EST_DistCogToRearAxle_P) * LQR_KappaFlt * look1_iflf_binlxpw
    (TCTI_VehicleVelX, ((const real32_T *)&(LQR_VelX[0])), LQR_Feedforward_gains,
     8U);

  /* Sum: '<S82>/Add' incorporates:
   *  Inport: '<Root>/Outport37'
   */
  LQR_DeltaF_Cmd_rad = ((LQR_DeltaF_feedback_rad + LQR_I_term_rad) +
                        LQR_DeltaF_feedforward_rad) + Test_CoeffPT1GainLdc;

  /* Delay: '<S603>/Delay' */
  rtb_Switch_gf = TJATCT_DW.Delay_DSTATE;

  /* Delay: '<S603>/Delay2' */
  rtb_Divide_eu = LQR_FltDeltaF_Cmd_rad;

  /* MATLAB Function: '<S603>/MATLAB Function' incorporates:
   *  Constant: '<S82>/Parameter1'
   *  Constant: '<S82>/Parameter3'
   */
  rtb_MultiportSwitch2_a = 3.14159274F * LQR_ButterCutFreq / (1.0F /
    LGC_TimeSysCycle_P);
  gm = cosf(rtb_MultiportSwitch2_a) / sinf(rtb_MultiportSwitch2_a);

  /* Switch: '<S603>/Switch' incorporates:
   *  Constant: '<S603>/Parameter8'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Logic: '<S603>/OR1'
   *  Logic: '<S603>/OR2'
   *  Logic: '<S603>/OR3'
   *  Product: '<S603>/Divide'
   *  RelationalOperator: '<S603>/Equal6'
   */
  if ((TCTI_VehicleVelX < LQR_EnaButterVel / 3.5999999046325684) ||
      LQR_EnaResetByTgq) {
    /* Switch: '<S603>/Switch' */
    LQR_FltDeltaF_Cmd_rad = LQR_DeltaF_Cmd_rad;
  } else {
    /* MATLAB Function: '<S603>/MATLAB Function' incorporates:
     *  Constant: '<S82>/Parameter5'
     */
    rtb_MultiportSwitch2_a = gm * gm;
    gm *= -1.0F / LQR_ButterQ;

    /* Switch: '<S603>/Switch' incorporates:
     *  Delay: '<S603>/Delay1'
     *  Delay: '<S603>/Delay2'
     *  Delay: '<S603>/Delay3'
     *  MATLAB Function: '<S603>/MATLAB Function'
     */
    LQR_FltDeltaF_Cmd_rad = ((((2.0F * rtb_Switch_gf + LQR_DeltaF_Cmd_rad) +
      TJATCT_DW.Delay1_DSTATE) - (1.0F - rtb_MultiportSwitch2_a) * 2.0F *
      LQR_FltDeltaF_Cmd_rad) - ((rtb_MultiportSwitch2_a + gm) + 1.0F) *
      TJATCT_DW.Delay3_DSTATE) * (1.0F / ((rtb_MultiportSwitch2_a - gm) + 1.0F));
  }

  /* End of Switch: '<S603>/Switch' */

  /* MATLAB Function: '<S601>/MATLAB Function' incorporates:
   *  Constant: '<S601>/Parameter1'
   *  Constant: '<S601>/Parameter2'
   *  Constant: '<S601>/Parameter3'
   *  UnitDelay: '<S601>/Unit Delay'
   *  UnitDelay: '<S601>/Unit Delay2'
   */
  rtb_MultiportSwitch2_a = 6.283F * LQR_LeadLagCutFreq * LGC_TimeSysCycle_P;
  LQR_DeltaF_Lead_Cmd_rad = (((rtb_MultiportSwitch2_a + 2.0F) *
    LQR_FltDeltaF_Cmd_rad + (rtb_MultiportSwitch2_a - 2.0F) *
    TJATCT_DW.UnitDelay_DSTATE_g) - (rtb_MultiportSwitch2_a - 2.0F /
    LQR_LeadLagGain) * LQR_DeltaF_Lead_Cmd_rad) / (rtb_MultiportSwitch2_a + 2.0F
    / LQR_LeadLagGain);

  /* Gain: '<S82>/rad2deg' */
  LQR_ReqDeltaF_Limit_deg = 57.3F * LQR_DeltaF_Lead_Cmd_rad;

  /* RelationalOperator: '<S597>/Equal' incorporates:
   *  Constant: '<S597>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  LQR_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == LGC_EnaFreezeByTgq_P);

  /* MultiPortSwitch: '<S627>/Multiport Switch3' incorporates:
   *  MultiPortSwitch: '<S628>/Multiport Switch2'
   */
  if (!LQR_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S627>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_EnaReplanCurValues'
     */
    if (TCTI_EnaReplanCurValues) {
      /* MultiPortSwitch: '<S627>/Multiport Switch1' incorporates:
       *  Constant: '<S627>/Constant7'
       */
      TJATCT_DW.UnitDelay1_DSTATE_i = 0.0F;
    } else {
      /* Product: '<S627>/Product' incorporates:
       *  Constant: '<S627>/Constant'
       *  Constant: '<S627>/Constant4'
       */
      rtb_MultiportSwitch2_a = LGC_MaxReqDeltaFGrd_P * LGC_TimeSysCycle_P;

      /* Sum: '<S627>/Add' incorporates:
       *  UnitDelay: '<S627>/Unit Delay1'
       */
      gm = LQR_ReqDeltaF_Limit_deg - TJATCT_DW.UnitDelay1_DSTATE_i;

      /* Switch: '<S629>/Switch2' incorporates:
       *  RelationalOperator: '<S629>/LowerRelop1'
       *  RelationalOperator: '<S629>/UpperRelop'
       *  Switch: '<S629>/Switch'
       *  UnaryMinus: '<S627>/Unary Minus1'
       */
      if (gm > rtb_MultiportSwitch2_a) {
        gm = rtb_MultiportSwitch2_a;
      } else {
        if (gm < -rtb_MultiportSwitch2_a) {
          /* Switch: '<S629>/Switch' incorporates:
           *  UnaryMinus: '<S627>/Unary Minus1'
           */
          gm = -rtb_MultiportSwitch2_a;
        }
      }

      /* End of Switch: '<S629>/Switch2' */

      /* MultiPortSwitch: '<S627>/Multiport Switch1' incorporates:
       *  Sum: '<S627>/Add1'
       *  UnitDelay: '<S627>/Unit Delay1'
       */
      TJATCT_DW.UnitDelay1_DSTATE_i += gm;
    }

    /* End of MultiPortSwitch: '<S627>/Multiport Switch1' */

    /* Switch: '<S630>/Switch2' incorporates:
     *  Constant: '<S628>/Constant2'
     *  Constant: '<S628>/Constant3'
     *  RelationalOperator: '<S630>/LowerRelop1'
     *  RelationalOperator: '<S630>/UpperRelop'
     *  Switch: '<S630>/Switch'
     *  UnaryMinus: '<S628>/Unary Minus'
     *  UnitDelay: '<S627>/Unit Delay1'
     */
    if (TJATCT_DW.UnitDelay1_DSTATE_i > LGC_MaxReqDeltaF_P) {
      /* UnitDelay: '<S628>/Unit Delay' */
      TJATCT_DW.UnitDelay_DSTATE_pa = LGC_MaxReqDeltaF_P;
    } else if (TJATCT_DW.UnitDelay1_DSTATE_i < -LGC_MaxReqDeltaF_P) {
      /* Switch: '<S630>/Switch' incorporates:
       *  Constant: '<S628>/Constant3'
       *  UnaryMinus: '<S628>/Unary Minus'
       *  UnitDelay: '<S628>/Unit Delay'
       */
      TJATCT_DW.UnitDelay_DSTATE_pa = -LGC_MaxReqDeltaF_P;
    } else {
      /* UnitDelay: '<S628>/Unit Delay' incorporates:
       *  Switch: '<S630>/Switch'
       */
      TJATCT_DW.UnitDelay_DSTATE_pa = TJATCT_DW.UnitDelay1_DSTATE_i;
    }

    /* End of Switch: '<S630>/Switch2' */
  }

  /* End of MultiPortSwitch: '<S627>/Multiport Switch3' */

  /* MultiPortSwitch: '<S628>/Multiport Switch' incorporates:
   *  Constant: '<S628>/Constant1'
   *  Constant: '<S628>/Constant18'
   *  RelationalOperator: '<S628>/Equal'
   */
  if (LGC_CswActOverride_P == 0) {
    /* MultiPortSwitch: '<S628>/Multiport Switch1' */
    if (!LQR_EnaCntrlByTgq) {
      /* Gain: '<S82>/rad2deg' incorporates:
       *  Constant: '<S628>/Constant6'
       *  MultiPortSwitch: '<S628>/Multiport Switch'
       */
      LQR_ReqDeltaF_Limit_deg = 0.0F;
    } else {
      /* Gain: '<S82>/rad2deg' incorporates:
       *  MultiPortSwitch: '<S628>/Multiport Switch'
       *  UnitDelay: '<S628>/Unit Delay'
       */
      LQR_ReqDeltaF_Limit_deg = TJATCT_DW.UnitDelay_DSTATE_pa;
    }

    /* End of MultiPortSwitch: '<S628>/Multiport Switch1' */
  }

  /* End of MultiPortSwitch: '<S628>/Multiport Switch' */

  /* MultiPortSwitch: '<S115>/Multiport Switch1' incorporates:
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   */
  switch (TCTI_StCntrlFcn) {
   case 1:
    /* MultiPortSwitch: '<S115>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S115>/1-D Lookup Table'
     *  Product: '<S115>/Divide'
     */
    CLM_ThdDeltaFCmdGrd = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdGrd_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdGrd_M[0])), 9U);
    break;

   case 2:
    /* MultiPortSwitch: '<S115>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S115>/1-D Lookup Table1'
     *  Product: '<S115>/Divide'
     */
    CLM_ThdDeltaFCmdGrd = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdGrd_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdGrd_LDP_M[0])), 9U);
    break;

   default:
    /* MultiPortSwitch: '<S115>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S115>/1-D Lookup Table'
     *  Product: '<S115>/Divide'
     */
    CLM_ThdDeltaFCmdGrd = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdGrd_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdGrd_M[0])), 9U);
    break;
  }

  /* End of MultiPortSwitch: '<S115>/Multiport Switch1' */

  /* Product: '<S115>/Product' incorporates:
   *  Constant: '<S115>/Parameter1'
   *  MultiPortSwitch: '<S117>/Multiport Switch1'
   *  Product: '<S117>/Product'
   */
  rtb_MultiportSwitch2_a = CLM_ThdDeltaFCmdGrd * CLM_TimeSysCycle_P;

  /* Logic: '<S115>/NOT1' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S120>/NOT1'
   *  Logic: '<S125>/NOT1'
   *  Logic: '<S131>/OR'
   */
  rtb_GreaterThan1_bj = !TCTI_EnaReplanCurValues;

  /* Logic: '<S115>/AND1' incorporates:
   *  Abs: '<S115>/Abs'
   *  Logic: '<S115>/NOT1'
   *  Product: '<S115>/Product'
   *  RelationalOperator: '<S115>/GreaterThan'
   *  Sum: '<S115>/Subtract'
   *  UnitDelay: '<S115>/Unit Delay'
   */
  CLM_EnaGrdDeltaFCmd = ((fabsf(LQR_ReqDeltaF_Limit_deg -
    TJATCT_DW.UnitDelay_DSTATE_bb) > rtb_MultiportSwitch2_a) &&
    rtb_GreaterThan1_bj);

  /* Sum: '<S117>/Sum' incorporates:
   *  MultiPortSwitch: '<S117>/Multiport Switch1'
   *  UnitDelay: '<S117>/Unit Delay'
   */
  gm = LQR_ReqDeltaF_Limit_deg - CLM_RawGrdDeltaFCmd;

  /* Switch: '<S118>/Switch2' incorporates:
   *  MultiPortSwitch: '<S117>/Multiport Switch1'
   *  RelationalOperator: '<S118>/LowerRelop1'
   */
  if (gm > rtb_MultiportSwitch2_a) {
    gm = rtb_MultiportSwitch2_a;
  } else {
    /* Product: '<S117>/Product1' incorporates:
     *  Constant: '<S115>/Parameter'
     *  UnaryMinus: '<S115>/Unary Minus'
     */
    rtb_MultiportSwitch2_a = -CLM_ThdDeltaFCmdGrd * CLM_TimeSysCycle_P;

    /* Switch: '<S118>/Switch' incorporates:
     *  RelationalOperator: '<S118>/UpperRelop'
     */
    if (gm < rtb_MultiportSwitch2_a) {
      gm = rtb_MultiportSwitch2_a;
    }

    /* End of Switch: '<S118>/Switch' */
  }

  /* End of Switch: '<S118>/Switch2' */

  /* MultiPortSwitch: '<S117>/Multiport Switch1' incorporates:
   *  Sum: '<S117>/Difference Inputs3'
   *  UnitDelay: '<S117>/Unit Delay'
   *
   * Block description for '<S117>/Difference Inputs3':
   *
   *  Add in CPU
   */
  CLM_RawGrdDeltaFCmd = gm + CLM_RawGrdDeltaFCmd;

  /* MultiPortSwitch: '<S115>/Multiport Switch' */
  CLM_GrdDeltaFCmd = CLM_RawGrdDeltaFCmd;

  /* MultiPortSwitch: '<S116>/Multiport Switch1' incorporates:
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   */
  switch (TCTI_StCntrlFcn) {
   case 1:
    /* MultiPortSwitch: '<S116>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S116>/1-D Lookup Table2'
     *  Product: '<S116>/Divide'
     */
    CLM_ThdDeltaFCmdSat = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdSat_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdSat_M[0])), 9U);
    break;

   case 2:
    /* MultiPortSwitch: '<S116>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S116>/1-D Lookup Table1'
     *  Product: '<S116>/Divide'
     */
    CLM_ThdDeltaFCmdSat = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdSat_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdSat_LDP_M[0])), 9U);
    break;

   default:
    /* MultiPortSwitch: '<S116>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S116>/1-D Lookup Table2'
     *  Product: '<S116>/Divide'
     */
    CLM_ThdDeltaFCmdSat = look1_iflf_binlxpw(TCTI_VehicleVelX / 3.6F, ((const
      real32_T *)&(CLM_ThdDeltaFCmdSat_X[0])), ((const real32_T *)
      &(CLM_ThdDeltaFCmdSat_M[0])), 9U);
    break;
  }

  /* End of MultiPortSwitch: '<S116>/Multiport Switch1' */

  /* RelationalOperator: '<S116>/GreaterThan' incorporates:
   *  Abs: '<S116>/Abs'
   */
  CLM_EnaSatDeltaFCmd = (fabsf(CLM_GrdDeltaFCmd) > CLM_ThdDeltaFCmdSat);

  /* RelationalOperator: '<S417>/Equal1' incorporates:
   *  Constant: '<S417>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  LGC_EnaCntrlByTgq = (TCTI_BtfTrajGuiQualifier != LGC_EnaOffByTgq_P);

  /* RelationalOperator: '<S417>/Equal' incorporates:
   *  Constant: '<S417>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  LGC_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == LGC_EnaFreezeByTgq_P);

  /* Switch: '<S578>/Switch' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  UnitDelay: '<S571>/Unit Delay'
   */
  if (TCTI_EnaReplanCurValues) {
    rtb_Neg_i = FFC_ReqFfcCrv;
  } else {
    rtb_Neg_i = TJATCT_DW.UnitDelay_DSTATE_g4;
  }

  /* End of Switch: '<S578>/Switch' */

  /* Switch: '<S576>/Switch' incorporates:
   *  Constant: '<S576>/Constant2'
   *  Product: '<S576>/Product'
   *  RelationalOperator: '<S576>/GreaterThanOrEqual1'
   *  UnitDelay: '<S571>/Unit Delay'
   */
  if (FFC_ReqFfcCrv * TJATCT_DW.UnitDelay_DSTATE_g4 >= 0.0F) {
    /* Switch: '<S576>/Switch2' incorporates:
     *  Abs: '<S576>/Abs'
     *  Abs: '<S576>/Abs1'
     *  Constant: '<S576>/Constant1'
     *  RelationalOperator: '<S576>/GreaterThanOrEqual'
     *  Sum: '<S576>/Add4'
     */
    if (fabsf(FFC_ReqFfcCrv) - fabsf(TJATCT_DW.UnitDelay_DSTATE_g4) >= 0.0F) {
      /* Switch: '<S576>/Switch' incorporates:
       *  Inport: '<Root>/TCTI_MaxCrvGrdBuildup'
       */
      LGC_CrvReqFfcGrdLimT1_1pm = TCTI_MaxCrvGrdBuildup;
    } else {
      /* Switch: '<S576>/Switch' incorporates:
       *  Inport: '<Root>/TCTI_MaxCrvGrdRed'
       */
      LGC_CrvReqFfcGrdLimT1_1pm = TCTI_MaxCrvGrdRed;
    }

    /* End of Switch: '<S576>/Switch2' */
  } else {
    /* Switch: '<S576>/Switch' incorporates:
     *  Inport: '<Root>/TCTI_MaxCrvGrdRed'
     */
    LGC_CrvReqFfcGrdLimT1_1pm = TCTI_MaxCrvGrdRed;
  }

  /* End of Switch: '<S576>/Switch' */

  /* Product: '<S578>/Product' incorporates:
   *  Constant: '<S419>/Parameter3'
   */
  rtb_MultiportSwitch2_a = LGC_CrvReqFfcGrdLimT1_1pm * LGC_TimeSysCycle_P;

  /* Sum: '<S578>/Add' */
  gm = FFC_ReqFfcCrv - rtb_Neg_i;

  /* Switch: '<S579>/Switch2' incorporates:
   *  RelationalOperator: '<S579>/LowerRelop1'
   *  RelationalOperator: '<S579>/UpperRelop'
   *  Switch: '<S579>/Switch'
   *  UnaryMinus: '<S578>/Unary Minus'
   */
  if (gm > rtb_MultiportSwitch2_a) {
    gm = rtb_MultiportSwitch2_a;
  } else {
    if (gm < -rtb_MultiportSwitch2_a) {
      /* Switch: '<S579>/Switch' incorporates:
       *  UnaryMinus: '<S578>/Unary Minus'
       */
      gm = -rtb_MultiportSwitch2_a;
    }
  }

  /* End of Switch: '<S579>/Switch2' */

  /* Sum: '<S578>/Add1' */
  LGC_CrvReqFfcGrdLimT2_1pm = gm + rtb_Neg_i;

  /* Switch: '<S577>/Switch2' incorporates:
   *  Switch: '<S570>/Switch2'
   */
  if (!LGC_EnaFreezeByTgq) {
    /* Switch: '<S577>/Switch2' */
    LGC_CrvReqFfcGrdLim_1pm = LGC_CrvReqFfcGrdLimT2_1pm;

    /* Switch: '<S570>/Switch2' */
    LGC_CrvReqFfcFrz_1pm = LGC_CrvReqFfcGrdLim_1pm;
  }

  /* End of Switch: '<S577>/Switch2' */

  /* Switch: '<S419>/Switch7' incorporates:
   *  Constant: '<S419>/Constant15'
   *  Constant: '<S419>/Constant3'
   *  Constant: '<S575>/Constant1'
   *  MATLAB Function: '<S575>/MATLAB Function'
   */
  if ((P_TCTLGC_CssCrv_nu & 2U) != 0U) {
    rtb_Switch7 = LGC_CrvReqFfcFrz_1pm;
  } else {
    rtb_Switch7 = 0.0F;
  }

  /* End of Switch: '<S419>/Switch7' */

  /* Switch: '<S568>/Switch4' */
  if (LGC_EnaCntrlByTgq) {
    /* Switch: '<S568>/Switch4' */
    LGC_FFCrv_1pm = rtb_Switch7;
  } else {
    /* Switch: '<S568>/Switch4' incorporates:
     *  Constant: '<S568>/Constant6'
     */
    LGC_FFCrv_1pm = 0.0F;
  }

  /* End of Switch: '<S568>/Switch4' */

  /* Lookup_n-D: '<S125>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  CLM_ThdFfcCrvGrd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(CLM_ThdFfcCrvGrd_X[0])), ((const real32_T *)&(CLM_ThdFfcCrvGrd_M[0])), 14U);

  /* Product: '<S125>/Product' incorporates:
   *  Constant: '<S125>/Parameter1'
   *  MultiPortSwitch: '<S127>/Multiport Switch1'
   *  Product: '<S127>/Product'
   */
  rtb_MultiportSwitch2_a = CLM_ThdFfcCrvGrd * CLM_TimeSysCycle_P;

  /* Logic: '<S125>/AND1' incorporates:
   *  Abs: '<S125>/Abs'
   *  Product: '<S125>/Product'
   *  RelationalOperator: '<S125>/GreaterThan'
   *  Sum: '<S125>/Subtract'
   *  UnitDelay: '<S125>/Unit Delay'
   */
  CLM_EnaGrdFfcCrv = ((fabsf(LGC_FFCrv_1pm - TJATCT_DW.UnitDelay_DSTATE_ou) >
                       rtb_MultiportSwitch2_a) && rtb_GreaterThan1_bj);

  /* Sum: '<S127>/Sum' incorporates:
   *  MultiPortSwitch: '<S127>/Multiport Switch1'
   *  UnitDelay: '<S127>/Unit Delay'
   */
  gm = LGC_FFCrv_1pm - CLM_RawGrdFfcCrv;

  /* Switch: '<S128>/Switch2' incorporates:
   *  MultiPortSwitch: '<S127>/Multiport Switch1'
   *  RelationalOperator: '<S128>/LowerRelop1'
   */
  if (gm > rtb_MultiportSwitch2_a) {
    gm = rtb_MultiportSwitch2_a;
  } else {
    /* Product: '<S127>/Product1' incorporates:
     *  Constant: '<S125>/Parameter'
     *  UnaryMinus: '<S125>/Unary Minus'
     */
    rtb_MultiportSwitch2_a = -CLM_ThdFfcCrvGrd * CLM_TimeSysCycle_P;

    /* Switch: '<S128>/Switch' incorporates:
     *  RelationalOperator: '<S128>/UpperRelop'
     */
    if (gm < rtb_MultiportSwitch2_a) {
      gm = rtb_MultiportSwitch2_a;
    }

    /* End of Switch: '<S128>/Switch' */
  }

  /* End of Switch: '<S128>/Switch2' */

  /* MultiPortSwitch: '<S127>/Multiport Switch1' incorporates:
   *  Sum: '<S127>/Difference Inputs3'
   *  UnitDelay: '<S127>/Unit Delay'
   *
   * Block description for '<S127>/Difference Inputs3':
   *
   *  Add in CPU
   */
  CLM_RawGrdFfcCrv = gm + CLM_RawGrdFfcCrv;

  /* MultiPortSwitch: '<S125>/Multiport Switch' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   */
  if (TCTI_EnaReplanCurValues) {
    /* MultiPortSwitch: '<S125>/Multiport Switch' */
    CLM_GrdFfcCrv = LGC_FFCrv_1pm;
  } else {
    /* MultiPortSwitch: '<S125>/Multiport Switch' */
    CLM_GrdFfcCrv = CLM_RawGrdFfcCrv;
  }

  /* End of MultiPortSwitch: '<S125>/Multiport Switch' */

  /* Lookup_n-D: '<S126>/1-D Lookup Table2' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  CLM_ThdFfcCrvSat = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(CLM_ThdFfcCrvSat_X[0])), ((const real32_T *)&(CLM_ThdFfcCrvSat_M[0])), 14U);

  /* RelationalOperator: '<S126>/GreaterThan' incorporates:
   *  Abs: '<S126>/Abs'
   */
  CLM_EnaSatFfcCrv = (fabsf(CLM_GrdFfcCrv) > CLM_ThdFfcCrvSat);

  /* RelationalOperator: '<S417>/Equal3' incorporates:
   *  Constant: '<S417>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3_gh = (TCTI_BtfTrajGuiQualifier == LGC_EnaFreezeByTgq_P);

  /* Logic: '<S417>/OR' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S417>/NOT'
   *  RelationalOperator: '<S562>/FixPt Relational Operator'
   *  UnitDelay: '<S562>/Delay Input1'
   *
   * Block description for '<S562>/Delay Input1':
   *
   *  Store in Global RAM
   */
  LGC_EnaResetByTgq = ((!LGC_EnaCntrlByTgq) || ((int32_T)rtb_Equal3_gh <
    (int32_T)TJATCT_DW.DelayInput1_DSTATE_p) || TCTI_EnaReplanCurValues);

  /* Lookup_n-D: '<S545>/Y_TCTLGC_PT1YErrTime_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeFltErrDistY = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeFltErrDistY_X[0])), ((const real32_T *)&(LGC_TimeFltErrDistY_M[0])),
    14U);

  /* Product: '<S560>/Divide' incorporates:
   *  Constant: '<S545>/Parameter'
   *  MinMax: '<S560>/Max1'
   */
  rtb_MultiportSwitch2_a = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    LGC_TimeFltErrDistY);

  /* Switch: '<S561>/Switch2' incorporates:
   *  Constant: '<S560>/IAM_Ts_P1'
   *  Constant: '<S560>/IAM_Ts_P4'
   *  RelationalOperator: '<S561>/LowerRelop1'
   *  RelationalOperator: '<S561>/UpperRelop'
   *  Switch: '<S561>/Switch'
   */
  if (rtb_MultiportSwitch2_a > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else {
    if (rtb_MultiportSwitch2_a < 0.0F) {
      /* Switch: '<S561>/Switch' incorporates:
       *  Constant: '<S560>/IAM_Ts_P1'
       */
      rtb_MultiportSwitch2_a = 0.0F;
    }
  }

  /* End of Switch: '<S561>/Switch2' */

  /* Sum: '<S560>/Add' incorporates:
   *  Product: '<S560>/Product'
   *  Sum: '<S560>/Subtract'
   */
  LGC_RawFltErrCtrlDistY = (CDC_CtrlErrDistY - LGC_RawFltErrCtrlDistY) *
    rtb_MultiportSwitch2_a + LGC_RawFltErrCtrlDistY;

  /* MultiPortSwitch: '<S545>/Multiport Switch4' incorporates:
   *  Constant: '<S545>/Constant1'
   *  Logic: '<S545>/AND'
   *  Logic: '<S545>/NOT'
   */
  if (LGC_EnaResetByTgq || (LGC_CswFltErrDistY_P == 0)) {
    /* MultiPortSwitch: '<S545>/Multiport Switch4' */
    LGC_FltPT1YErr_met = CDC_CtrlErrDistY;
  } else {
    /* MultiPortSwitch: '<S545>/Multiport Switch4' */
    LGC_FltPT1YErr_met = LGC_RawFltErrCtrlDistY;
  }

  /* End of MultiPortSwitch: '<S545>/Multiport Switch4' */

  /* Switch: '<S556>/Init' incorporates:
   *  Logic: '<S556>/FixPt Logical Operator'
   *  UnitDelay: '<S556>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_f != 0)) {
    /* Switch: '<S556>/Init' incorporates:
     *  Constant: '<S549>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_c = 0.0F;
  }

  /* End of Switch: '<S556>/Init' */

  /* Switch: '<S554>/Switch2' incorporates:
   *  Constant: '<S549>/Constant4'
   *  Constant: '<S549>/Constant5'
   *  RelationalOperator: '<S554>/LowerRelop1'
   *  RelationalOperator: '<S554>/UpperRelop'
   *  Switch: '<S554>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_c > 100.0F) {
    /* Switch: '<S554>/Switch2' */
    LGC_CmpnLaDmcLdc = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_c < -100.0F) {
    /* Switch: '<S554>/Switch' incorporates:
     *  Constant: '<S549>/Constant5'
     *  Switch: '<S554>/Switch2'
     */
    LGC_CmpnLaDmcLdc = -100.0F;
  } else {
    /* Switch: '<S554>/Switch2' incorporates:
     *  Switch: '<S554>/Switch'
     */
    LGC_CmpnLaDmcLdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_c;
  }

  /* End of Switch: '<S554>/Switch2' */

  /* MultiPortSwitch: '<S543>/Multiport Switch5' incorporates:
   *  Constant: '<S543>/Constant'
   */
  if (LGC_CswLaDmcCmpnLdc_P == 0) {
    /* MultiPortSwitch: '<S543>/Multiport Switch5' */
    LGC_ErrCtrlDistY = LGC_FltPT1YErr_met;
  } else {
    /* MultiPortSwitch: '<S543>/Multiport Switch5' */
    LGC_ErrCtrlDistY = LGC_CmpnLaDmcLdc;
  }

  /* End of MultiPortSwitch: '<S543>/Multiport Switch5' */

  /* Product: '<S540>/Mul4' incorporates:
   *  Inport: '<Root>/Outport32'
   */
  LGC_CtrlErrMainPGain = LGC_ErrCtrlDistY * Test_CoeffMainPGainLdc;

  /* MultiPortSwitch: '<S539>/Multiport Switch4' incorporates:
   *  Constant: '<S539>/Constant'
   *  Logic: '<S539>/NOT'
   *  UnitDelay: '<S539>/Unit Delay'
   */
  if (LGC_EnaResetByTgq) {
    rtb_Neg_i = 0.0F;
  } else {
    rtb_Neg_i = TJATCT_DW.UnitDelay_DSTATE_hg;
  }

  /* End of MultiPortSwitch: '<S539>/Multiport Switch4' */

  /* Logic: '<S422>/AND1' incorporates:
   *  Constant: '<S422>/Parameter1'
   *  Constant: '<S422>/Parameter2'
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   *  RelationalOperator: '<S422>/Equal1'
   *  RelationalOperator: '<S422>/Equal2'
   */
  rtb_LGC_EnaActSafetyFcn = ((TCTI_StCntrlFcn != LGC_OffLcfCtrlFcn_P) &&
    (TCTI_StCntrlFcn != LGC_TJALcfCtrlFcn_P));

  /* DataTypeConversion: '<S422>/Data Type Conversion' */
  LGC_EnaActSafetyFcn = (uint8_T)rtb_LGC_EnaActSafetyFcn;

  /* RelationalOperator: '<S422>/Equal' incorporates:
   *  Constant: '<S422>/Parameter'
   *  Inport: '<Root>/TCTI _StLatCtrlMode'
   */
  rtb_LGC_EnaActObjFollow = (TCTI_StLatCtrlMode == LGC_OfLatCtrlMode_P);

  /* DataTypeConversion: '<S422>/Data Type Conversion1' */
  LGC_EnaActObjFollow = (uint8_T)rtb_LGC_EnaActObjFollow;

  /* Logic: '<S539>/OR1' incorporates:
   *  UnitDelay: '<S539>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_hg = ((LGC_EnaActSafetyFcn != 0) ||
    (LGC_EnaActObjFollow != 0));

  /* RelationalOperator: '<S539>/NotEqual' incorporates:
   *  UnitDelay: '<S539>/Unit Delay'
   */
  LGC_EnaModeChangeDtct = (rtb_Neg_i != TJATCT_DW.UnitDelay_DSTATE_hg);

  /* Sum: '<S539>/Subtract' incorporates:
   *  Inport: '<Root>/Outport33'
   *  UnitDelay: '<S539>/Unit Delay1'
   */
  rtb_Neg_i = Test_CoeffPGainLdc - TJATCT_DW.UnitDelay1_DSTATE_c;

  /* Product: '<S539>/Product' incorporates:
   *  Constant: '<S539>/Constant6'
   *  Constant: '<S539>/Constant7'
   *  MultiPortSwitch: '<S541>/Multiport Switch1'
   *  Product: '<S539>/Product1'
   *  Product: '<S541>/Product'
   */
  gm = LGC_ThdPGainGrdLdc_P * LGC_TimeSysCycle_P;

  /* RelationalOperator: '<S539>/GreaterThan' incorporates:
   *  Abs: '<S539>/Abs'
   *  Product: '<S539>/Product'
   */
  LGC_EnaPGainGrdThdLdc = (fabsf(rtb_Neg_i) > gm);

  /* Logic: '<S539>/OR3' */
  LGC_EnaPGainGrdLmtLdc = (LGC_EnaModeChangeDtct || LGC_EnaPGainGrdThdLdc);

  /* MultiPortSwitch: '<S541>/Multiport Switch1' */
  switch ((int32_T)LGC_EnaPGainGrdLmtLdc) {
   case 0:
    break;

   default:
    /* Sum: '<S541>/Sum' incorporates:
     *  Inport: '<Root>/Outport33'
     *  UnitDelay: '<S541>/Unit Delay'
     */
    rtb_MultiportSwitch2_a = Test_CoeffPGainLdc - LGC_RawLmtCoeffPGainLdc;

    /* Switch: '<S542>/Switch2' incorporates:
     *  RelationalOperator: '<S542>/LowerRelop1'
     */
    if (rtb_MultiportSwitch2_a > gm) {
      rtb_MultiportSwitch2_a = gm;
    } else {
      /* Product: '<S541>/Product1' incorporates:
       *  Constant: '<S540>/Constant1'
       *  Constant: '<S540>/Constant5'
       *  UnaryMinus: '<S540>/Unary Minus'
       */
      rtb_Product1_c = -LGC_ThdPGainGrdLdc_P * LGC_TimeSysCycle_P;

      /* Switch: '<S542>/Switch' incorporates:
       *  RelationalOperator: '<S542>/UpperRelop'
       */
      if (rtb_MultiportSwitch2_a < rtb_Product1_c) {
        rtb_MultiportSwitch2_a = rtb_Product1_c;
      }

      /* End of Switch: '<S542>/Switch' */
    }

    /* End of Switch: '<S542>/Switch2' */

    /* MultiPortSwitch: '<S541>/Multiport Switch1' incorporates:
     *  Sum: '<S541>/Difference Inputs3'
     *  UnitDelay: '<S541>/Unit Delay'
     *
     * Block description for '<S541>/Difference Inputs3':
     *
     *  Add in CPU
     */
    LGC_RawLmtCoeffPGainLdc = rtb_MultiportSwitch2_a + LGC_RawLmtCoeffPGainLdc;
    break;
  }

  /* MultiPortSwitch: '<S540>/Multiport Switch5' incorporates:
   *  Logic: '<S540>/AND'
   *  Logic: '<S540>/NOT'
   */
  if (LGC_EnaResetByTgq || (!LGC_EnaPGainGrdLmtLdc)) {
    /* MultiPortSwitch: '<S540>/Multiport Switch5' incorporates:
     *  Inport: '<Root>/Outport33'
     */
    LGC_LmtCoeffPGainLdc = Test_CoeffPGainLdc;
  } else {
    /* MultiPortSwitch: '<S540>/Multiport Switch5' */
    LGC_LmtCoeffPGainLdc = LGC_RawLmtCoeffPGainLdc;
  }

  /* End of MultiPortSwitch: '<S540>/Multiport Switch5' */

  /* Product: '<S540>/Mul8' */
  LGC_DeltaFPGainLdc = LGC_CtrlErrMainPGain * LGC_LmtCoeffPGainLdc;

  /* Signum: '<S435>/Sign2' */
  if (CDC_CtrlErrDistY < 0.0F) {
    /* Signum: '<S565>/Sign2' incorporates:
     *  Signum: '<S583>/Sign2'
     *  Switch: '<S565>/Switch'
     *  Switch: '<S583>/Switch'
     */
    rtb_Unit_Delay2_j = -1.0F;
  } else if (CDC_CtrlErrDistY > 0.0F) {
    /* Signum: '<S565>/Sign2' incorporates:
     *  Signum: '<S583>/Sign2'
     *  Switch: '<S565>/Switch'
     *  Switch: '<S583>/Switch'
     */
    rtb_Unit_Delay2_j = 1.0F;
  } else {
    /* Signum: '<S565>/Sign2' incorporates:
     *  Signum: '<S583>/Sign2'
     *  Switch: '<S565>/Switch'
     *  Switch: '<S583>/Switch'
     */
    rtb_Unit_Delay2_j = CDC_CtrlErrDistY;
  }

  /* Signum: '<S435>/Sign' incorporates:
   *  UnitDelay: '<S435>/Unit_Delay'
   */
  if (LGC_DeltaFPT1GainLdc < 0.0F) {
    LGC_DeltaFPT1GainLdc = -1.0F;
  } else {
    if (LGC_DeltaFPT1GainLdc > 0.0F) {
      LGC_DeltaFPT1GainLdc = 1.0F;
    }
  }

  /* Logic: '<S435>/AND1' incorporates:
   *  Abs: '<S435>/Abs'
   *  Constant: '<S435>/Constant3'
   *  Constant: '<S435>/Parameter'
   *  Constant: '<S435>/Parameter1'
   *  Product: '<S435>/Product'
   *  RelationalOperator: '<S435>/Equal'
   *  RelationalOperator: '<S435>/GreaterThan'
   *  RelationalOperator: '<S435>/GreaterThan1'
   *  Signum: '<S435>/Sign2'
   */
  LGC_EnaRstByDistY = ((rtb_Divide_hw >= LGC_IntResMaxLatErr_P) &&
                       (rtb_Unit_Delay2_j * LGC_DeltaFPT1GainLdc == -1.0F) &&
                       (fabsf(FFC_ReqFfcCrv) <= LGC_IntResMaxCrv_P));

  /* RelationalOperator: '<S435>/GreaterThan2' incorporates:
   *  Abs: '<S435>/Abs2'
   *  Constant: '<S435>/Parameter2'
   *  Inport: '<Root>/TCTI_ActualTrqEPS'
   */
  LGC_EnaRstByTrq = (fabsf(TCTI_ActualTrqEPS) >= LGC_IntResMaxManTrq_P);

  /* RelationalOperator: '<S435>/Less Than' incorporates:
   *  Constant: '<S435>/Parameter3'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_EnaRstByStandStill = (TCTI_VehicleVelX <= LGC_ThdVelStandStill_P);

  /* Logic: '<S434>/OR5' incorporates:
   *  Constant: '<S434>/Parameter12'
   *  Constant: '<S434>/Parameter13'
   *  Constant: '<S434>/Parameter14'
   *  Logic: '<S434>/AND10'
   *  Logic: '<S434>/AND4'
   *  Logic: '<S434>/AND6'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR13'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR14'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR15'
   */
  LGC_EnaRstIntLdc = ((((LGC_BtmDynIntRst_P & 1U) != 0U) && LGC_EnaRstByDistY) ||
                      (((LGC_BtmDynIntRst_P & 2U) != 0U) && LGC_EnaRstByTrq) ||
                      (((LGC_BtmDynIntRst_P & 4U) != 0U) &&
                       LGC_EnaRstByStandStill));

  /* Logic: '<S527>/OR' */
  rtb_OR_fh = (LGC_EnaResetByTgq || LGC_EnaRstIntLdc);

  /* Switch: '<S534>/Init' incorporates:
   *  Logic: '<S534>/FixPt Logical Operator'
   *  UnitDelay: '<S534>/FixPt Unit Delay2'
   */
  if (rtb_OR_fh || (TJATCT_DW.FixPtUnitDelay2_DSTATE_a != 0)) {
    /* Switch: '<S534>/Init' incorporates:
     *  Constant: '<S527>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_h = 0.0F;
  }

  /* End of Switch: '<S534>/Init' */

  /* Switch: '<S532>/Switch2' incorporates:
   *  Constant: '<S527>/P_TCTLGC_LdcILimit_rad'
   *  RelationalOperator: '<S532>/LowerRelop1'
   *  RelationalOperator: '<S532>/UpperRelop'
   *  Switch: '<S532>/Switch'
   *  UnaryMinus: '<S527>/Neg1'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_h > P_TCTLGC_LdcILimit_rad) {
    /* Switch: '<S532>/Switch2' */
    LGC_DeltaFIGainLdc = P_TCTLGC_LdcILimit_rad;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_h < -P_TCTLGC_LdcILimit_rad) {
    /* Switch: '<S532>/Switch' incorporates:
     *  Switch: '<S532>/Switch2'
     *  UnaryMinus: '<S527>/Neg1'
     */
    LGC_DeltaFIGainLdc = -P_TCTLGC_LdcILimit_rad;
  } else {
    /* Switch: '<S532>/Switch2' incorporates:
     *  Switch: '<S532>/Switch'
     */
    LGC_DeltaFIGainLdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_h;
  }

  /* End of Switch: '<S532>/Switch2' */

  /* MultiPortSwitch: '<S526>/Multiport Switch10' incorporates:
   *  Constant: '<S526>/Constant20'
   *  Constant: '<S526>/Parameter3'
   *  Logic: '<S526>/AND'
   *  Logic: '<S526>/NOT'
   *  Product: '<S526>/Divide'
   *  Sum: '<S526>/Sum4'
   *  UnitDelay: '<S526>/Unit_Delay1'
   */
  if (LGC_EnaResetByTgq && LGC_EnaFreezeByTgq) {
    rtb_MultiportSwitch2_a = 0.0F;
  } else {
    rtb_MultiportSwitch2_a = (LGC_DeltaFPGainLdc -
      TJATCT_DW.Unit_Delay1_DSTATE_e) / LGC_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S526>/Multiport Switch10' */

  /* MultiPortSwitch: '<S539>/Multiport Switch7' incorporates:
   *  RelationalOperator: '<S539>/GreaterThan1'
   */
  if (rtb_Neg_i <= gm) {
    /* MultiPortSwitch: '<S539>/Multiport Switch7' incorporates:
     *  Constant: '<S539>/Constant10'
     */
    LGC_EnaPGainGrdSignLdc = 1.0F;
  } else {
    /* MultiPortSwitch: '<S539>/Multiport Switch7' incorporates:
     *  Constant: '<S539>/Constant11'
     */
    LGC_EnaPGainGrdSignLdc = -1.0F;
  }

  /* End of MultiPortSwitch: '<S539>/Multiport Switch7' */

  /* MultiPortSwitch: '<S526>/Multiport Switch11' incorporates:
   *  Constant: '<S526>/Constant17'
   *  Constant: '<S526>/Constant18'
   *  Product: '<S526>/Mul1'
   *  Product: '<S526>/Mul2'
   */
  if (!LGC_EnaPGainGrdThdLdc) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = LGC_EnaPGainGrdSignLdc * LGC_ThdPGainGrdLdc_P *
      LGC_CtrlErrMainPGain;
  }

  /* End of MultiPortSwitch: '<S526>/Multiport Switch11' */

  /* Product: '<S526>/Mul3' incorporates:
   *  Inport: '<Root>/Outport35'
   *  Sum: '<S526>/Sum'
   */
  rtb_Neg_i = (rtb_MultiportSwitch2_a - LGC_RawErrCourseDotCdc_tmp) *
    Test_CoeffDGainLdc;

  /* Product: '<S529>/Divide' incorporates:
   *  Constant: '<S526>/Parameter2'
   *  Inport: '<Root>/Outport36'
   *  MinMax: '<S529>/Max1'
   */
  rtb_MultiportSwitch2_a = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    Test_TimeDT1Ldc);

  /* Switch: '<S530>/Switch2' incorporates:
   *  Constant: '<S529>/IAM_Ts_P1'
   *  Constant: '<S529>/IAM_Ts_P4'
   *  RelationalOperator: '<S530>/LowerRelop1'
   *  RelationalOperator: '<S530>/UpperRelop'
   *  Switch: '<S530>/Switch'
   */
  if (rtb_MultiportSwitch2_a > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else {
    if (rtb_MultiportSwitch2_a < 0.0F) {
      /* Switch: '<S530>/Switch' incorporates:
       *  Constant: '<S529>/IAM_Ts_P1'
       */
      rtb_MultiportSwitch2_a = 0.0F;
    }
  }

  /* End of Switch: '<S530>/Switch2' */

  /* Sum: '<S529>/Add' incorporates:
   *  Product: '<S529>/Product'
   *  Sum: '<S529>/Subtract'
   *  UnitDelay: '<S529>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_jb += (rtb_Neg_i - TJATCT_DW.UnitDelay_DSTATE_jb) *
    rtb_MultiportSwitch2_a;

  /* MultiPortSwitch: '<S526>/Multiport Switch9' incorporates:
   *  RelationalOperator: '<S526>/Equal1'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S526>/Multiport Switch9' */
    LGC_DeltaFDGainLdc = rtb_Neg_i;
  } else {
    /* MultiPortSwitch: '<S526>/Multiport Switch9' incorporates:
     *  UnitDelay: '<S529>/Unit Delay'
     */
    LGC_DeltaFDGainLdc = TJATCT_DW.UnitDelay_DSTATE_jb;
  }

  /* End of MultiPortSwitch: '<S526>/Multiport Switch9' */

  /* Logic: '<S434>/OR4' incorporates:
   *  Constant: '<S434>/Parameter'
   *  Constant: '<S434>/Parameter1'
   *  Constant: '<S434>/Parameter2'
   *  Logic: '<S434>/AND'
   *  Logic: '<S434>/AND13'
   *  Logic: '<S434>/AND14'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR1'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR2'
   */
  LGC_EnaRstPT1Ldc = ((((LGC_BtmDynPT1Rst_P & 1U) != 0U) && LGC_EnaRstByDistY) ||
                      (((LGC_BtmDynPT1Rst_P & 2U) != 0U) && LGC_EnaRstByTrq) ||
                      (((LGC_BtmDynPT1Rst_P & 4U) != 0U) &&
                       LGC_EnaRstByStandStill));

  /* Product: '<S536>/Divide' incorporates:
   *  Constant: '<S528>/Parameter'
   *  Inport: '<Root>/Outport38'
   *  MinMax: '<S536>/Max1'
   */
  rtb_MultiportSwitch2_a = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    Test_TimePT1Ldc);

  /* Switch: '<S538>/Switch2' incorporates:
   *  Constant: '<S536>/IAM_Ts_P1'
   *  Constant: '<S536>/IAM_Ts_P4'
   *  RelationalOperator: '<S538>/LowerRelop1'
   *  RelationalOperator: '<S538>/UpperRelop'
   *  Switch: '<S538>/Switch'
   */
  if (rtb_MultiportSwitch2_a > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else {
    if (rtb_MultiportSwitch2_a < 0.0F) {
      /* Switch: '<S538>/Switch' incorporates:
       *  Constant: '<S536>/IAM_Ts_P1'
       */
      rtb_MultiportSwitch2_a = 0.0F;
    }
  }

  /* End of Switch: '<S538>/Switch2' */

  /* Sum: '<S536>/Add' incorporates:
   *  Inport: '<Root>/Outport37'
   *  Product: '<S528>/Mul'
   *  Product: '<S536>/Product'
   *  Sum: '<S536>/Subtract'
   *  UnitDelay: '<S536>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_g5 += (LGC_DeltaFPGainLdc * Test_CoeffPT1GainLdc -
    TJATCT_DW.UnitDelay_DSTATE_g5) * rtb_MultiportSwitch2_a;

  /* MultiPortSwitch: '<S528>/Multiport Switch8' incorporates:
   *  Constant: '<S528>/Constant15'
   *  Logic: '<S528>/OR'
   *  MultiPortSwitch: '<S535>/Multiport Switch1'
   *  UnitDelay: '<S536>/Unit Delay'
   */
  if (LGC_EnaResetByTgq || LGC_EnaRstPT1Ldc) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = TJATCT_DW.UnitDelay_DSTATE_g5;
  }

  /* End of MultiPortSwitch: '<S528>/Multiport Switch8' */

  /* Sum: '<S535>/Sum' incorporates:
   *  MultiPortSwitch: '<S535>/Multiport Switch1'
   *  UnitDelay: '<S535>/Unit Delay'
   */
  rtb_MultiportSwitch2_a = LGC_RawErrCourseDotCdc_tmp -
    TJATCT_DW.UnitDelay_DSTATE_lex;

  /* Product: '<S535>/Product' incorporates:
   *  Constant: '<S528>/Constant12'
   *  Constant: '<S528>/Parameter1'
   *  MultiPortSwitch: '<S535>/Multiport Switch1'
   */
  gm = LGC_MaxGrdPT1Ldc_P * LGC_TimeSysCycle_P;

  /* Switch: '<S537>/Switch2' incorporates:
   *  MultiPortSwitch: '<S535>/Multiport Switch1'
   *  RelationalOperator: '<S537>/LowerRelop1'
   */
  if (rtb_MultiportSwitch2_a > gm) {
    rtb_MultiportSwitch2_a = gm;
  } else {
    /* Product: '<S535>/Product1' incorporates:
     *  Constant: '<S528>/Constant12'
     *  Constant: '<S528>/Parameter1'
     *  UnaryMinus: '<S528>/Neg'
     */
    gm = -LGC_MaxGrdPT1Ldc_P * LGC_TimeSysCycle_P;

    /* Switch: '<S537>/Switch' incorporates:
     *  RelationalOperator: '<S537>/UpperRelop'
     */
    if (rtb_MultiportSwitch2_a < gm) {
      rtb_MultiportSwitch2_a = gm;
    }

    /* End of Switch: '<S537>/Switch' */
  }

  /* End of Switch: '<S537>/Switch2' */

  /* Signum: '<S435>/Sign' incorporates:
   *  MultiPortSwitch: '<S535>/Multiport Switch1'
   *  Sum: '<S535>/Difference Inputs3'
   *  UnitDelay: '<S535>/Unit Delay'
   *
   * Block description for '<S535>/Difference Inputs3':
   *
   *  Add in CPU
   */
  LGC_DeltaFPT1GainLdc = rtb_MultiportSwitch2_a + TJATCT_DW.UnitDelay_DSTATE_lex;

  /* MultiPortSwitch: '<S524>/Multiport Switch' incorporates:
   *  Constant: '<S524>/P_TCTLGC_LdcMode_nu'
   *  Constant: '<S524>/P_TCTLGC_LdcMode_nu3'
   *  DataTypeConversion: '<S524>/Data Type Conversion'
   *  DataTypeConversion: '<S524>/Data Type Conversion1'
   *  MultiPortSwitch: '<S524>/Multiport Switch3'
   *  S-Function (sfix_bitop): '<S524>/Bitwise Operator'
   *  S-Function (sfix_bitop): '<S524>/Bitwise Operator3'
   */
  if ((LGC_CswLdcMode_P & 1U) == 0U) {
    /* MultiPortSwitch: '<S524>/Multiport Switch' incorporates:
     *  Constant: '<S524>/Constant16'
     */
    LGC_LdcAloneICmd_rad = 0.0F;
  } else {
    if ((LGC_CswLdcMode_P & 4U) == 0U) {
      /* MultiPortSwitch: '<S524>/Multiport Switch3' incorporates:
       *  Constant: '<S524>/Constant4'
       */
      LGC_RawErrCourseDotCdc_tmp = 0.0F;
    } else {
      /* MultiPortSwitch: '<S524>/Multiport Switch3' */
      LGC_RawErrCourseDotCdc_tmp = LGC_DeltaFIGainLdc;
    }

    /* MultiPortSwitch: '<S524>/Multiport Switch2' incorporates:
     *  Constant: '<S524>/Constant3'
     *  Constant: '<S524>/P_TCTLGC_LdcMode_nu2'
     *  DataTypeConversion: '<S524>/Data Type Conversion2'
     *  S-Function (sfix_bitop): '<S524>/Bitwise Operator2'
     */
    if ((LGC_CswLdcMode_P & 2U) == 0U) {
      rtb_Divide_l_tmp = 0.0F;
    } else {
      rtb_Divide_l_tmp = LGC_DeltaFDGainLdc;
    }

    /* End of MultiPortSwitch: '<S524>/Multiport Switch2' */

    /* MultiPortSwitch: '<S524>/Multiport Switch1' incorporates:
     *  Constant: '<S524>/Constant2'
     *  Constant: '<S524>/P_TCTLGC_LdcMode_nu1'
     *  DataTypeConversion: '<S524>/Data Type Conversion3'
     *  S-Function (sfix_bitop): '<S524>/Bitwise Operator1'
     */
    if ((LGC_CswLdcMode_P & 8U) == 0U) {
      rtb_Product_dv = 0.0F;
    } else {
      rtb_Product_dv = LGC_DeltaFPT1GainLdc;
    }

    /* End of MultiPortSwitch: '<S524>/Multiport Switch1' */

    /* MultiPortSwitch: '<S524>/Multiport Switch' incorporates:
     *  Sum: '<S524>/Sum1'
     *  Sum: '<S524>/Sum2'
     *  Sum: '<S524>/Sum3'
     */
    LGC_LdcAloneICmd_rad = ((LGC_DeltaFPGainLdc + LGC_RawErrCourseDotCdc_tmp) +
      rtb_Divide_l_tmp) + rtb_Product_dv;
  }

  /* End of MultiPortSwitch: '<S524>/Multiport Switch' */

  /* Lookup_n-D: '<S463>/Y_TCTLGC_PT1YErrTime_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Lookup_n-D: '<S503>/Y_TCTLGC_PT1YErrTime_sec'
   */
  LGC_TimeFltErrCourse = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeFltErrCourse_X[0])), ((const real32_T *)&(LGC_TimeFltErrCourse_M[0])),
    14U);

  /* Product: '<S478>/Divide' incorporates:
   *  Constant: '<S463>/Parameter'
   *  MinMax: '<S478>/Max1'
   */
  rtb_MultiportSwitch2_a = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    LGC_TimeFltErrCourse);

  /* Switch: '<S479>/Switch2' incorporates:
   *  Constant: '<S478>/IAM_Ts_P1'
   *  Constant: '<S478>/IAM_Ts_P4'
   *  RelationalOperator: '<S479>/LowerRelop1'
   *  RelationalOperator: '<S479>/UpperRelop'
   *  Switch: '<S479>/Switch'
   */
  if (rtb_MultiportSwitch2_a > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else {
    if (rtb_MultiportSwitch2_a < 0.0F) {
      /* Switch: '<S479>/Switch' incorporates:
       *  Constant: '<S478>/IAM_Ts_P1'
       */
      rtb_MultiportSwitch2_a = 0.0F;
    }
  }

  /* End of Switch: '<S479>/Switch2' */

  /* Sum: '<S478>/Add' incorporates:
   *  Product: '<S478>/Product'
   *  Sum: '<S478>/Subtract'
   */
  LGC_RawFltErrCourseCas = (LGC_LdcAloneICmd_rad - LGC_RawFltErrCourseCas) *
    rtb_MultiportSwitch2_a + LGC_RawFltErrCourseCas;

  /* MultiPortSwitch: '<S463>/Multiport Switch4' incorporates:
   *  Constant: '<S463>/Constant1'
   *  Logic: '<S463>/AND'
   *  Logic: '<S463>/NOT'
   */
  if (LGC_EnaResetByTgq || (LGC_CswFltErrCourse_P == 0)) {
    /* MultiPortSwitch: '<S463>/Multiport Switch4' */
    LGC_FltErrCourseCas = LGC_LdcAloneICmd_rad;
  } else {
    /* MultiPortSwitch: '<S463>/Multiport Switch4' */
    LGC_FltErrCourseCas = LGC_RawFltErrCourseCas;
  }

  /* End of MultiPortSwitch: '<S463>/Multiport Switch4' */

  /* Switch: '<S474>/Init' incorporates:
   *  Logic: '<S474>/FixPt Logical Operator'
   *  UnitDelay: '<S474>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_k != 0)) {
    /* Switch: '<S474>/Init' incorporates:
     *  Constant: '<S467>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cy = 0.0F;
  }

  /* End of Switch: '<S474>/Init' */

  /* Switch: '<S472>/Switch2' incorporates:
   *  Constant: '<S467>/Constant4'
   *  Constant: '<S467>/Constant5'
   *  RelationalOperator: '<S472>/LowerRelop1'
   *  RelationalOperator: '<S472>/UpperRelop'
   *  Switch: '<S472>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cy > 100.0F) {
    /* Switch: '<S472>/Switch2' */
    LGC_CmpnLaDmcCas = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cy < -100.0F) {
    /* Switch: '<S472>/Switch' incorporates:
     *  Constant: '<S467>/Constant5'
     *  Switch: '<S472>/Switch2'
     */
    LGC_CmpnLaDmcCas = -100.0F;
  } else {
    /* Switch: '<S472>/Switch2' incorporates:
     *  Switch: '<S472>/Switch'
     */
    LGC_CmpnLaDmcCas = TJATCT_DW.FixPtUnitDelay1_DSTATE_cy;
  }

  /* End of Switch: '<S472>/Switch2' */

  /* MultiPortSwitch: '<S461>/Multiport Switch5' incorporates:
   *  Constant: '<S461>/Constant'
   */
  if (LGC_CswLaDmcCmpnCac_P == 0) {
    /* MultiPortSwitch: '<S461>/Multiport Switch5' */
    LGC_ErrCtrlCourseCas = LGC_FltErrCourseCas;
  } else {
    /* MultiPortSwitch: '<S461>/Multiport Switch5' */
    LGC_ErrCtrlCourseCas = LGC_CmpnLaDmcCas;
  }

  /* End of MultiPortSwitch: '<S461>/Multiport Switch5' */

  /* Product: '<S458>/Mul4' incorporates:
   *  Inport: '<Root>/Outport39'
   */
  LGC_CtrlErrMainPGainCas = LGC_ErrCtrlCourseCas * Test_CoeffMainPGainCac;

  /* MultiPortSwitch: '<S457>/Multiport Switch4' incorporates:
   *  Constant: '<S457>/Constant'
   *  Logic: '<S457>/NOT'
   *  UnitDelay: '<S457>/Unit Delay'
   */
  if (LGC_EnaResetByTgq) {
    rtb_Neg_i = 0.0F;
  } else {
    rtb_Neg_i = TJATCT_DW.UnitDelay_DSTATE_jw;
  }

  /* End of MultiPortSwitch: '<S457>/Multiport Switch4' */

  /* Logic: '<S457>/OR1' incorporates:
   *  UnitDelay: '<S457>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_jw = ((LGC_EnaActSafetyFcn != 0) ||
    (LGC_EnaActObjFollow != 0));

  /* RelationalOperator: '<S457>/NotEqual' incorporates:
   *  UnitDelay: '<S457>/Unit Delay'
   */
  LGC_EnaModeChangeCas = (rtb_Neg_i != TJATCT_DW.UnitDelay_DSTATE_jw);

  /* Sum: '<S457>/Subtract' incorporates:
   *  Inport: '<Root>/Outport40'
   *  UnitDelay: '<S457>/Unit Delay1'
   */
  rtb_Neg_i = Test_CoeffPGainCac - TJATCT_DW.UnitDelay1_DSTATE_n;

  /* Product: '<S457>/Product' incorporates:
   *  Constant: '<S457>/Constant6'
   *  Constant: '<S457>/Constant7'
   *  MultiPortSwitch: '<S459>/Multiport Switch1'
   *  MultiPortSwitch: '<S499>/Multiport Switch1'
   *  Product: '<S457>/Product1'
   *  Product: '<S459>/Product'
   *  Product: '<S499>/Product'
   */
  rtb_Product1_l5 = LGC_ThdPGainGrdCas_P * LGC_TimeSysCycle_P;

  /* RelationalOperator: '<S457>/GreaterThan' incorporates:
   *  Abs: '<S457>/Abs'
   *  Product: '<S457>/Product'
   */
  LGC_EnaPGainGrdThdCas = (fabsf(rtb_Neg_i) > rtb_Product1_l5);

  /* Logic: '<S457>/OR3' */
  LGC_EnaPGainGrdLmtCas = (LGC_EnaModeChangeCas || LGC_EnaPGainGrdThdCas);

  /* MultiPortSwitch: '<S459>/Multiport Switch1' */
  switch ((int32_T)LGC_EnaPGainGrdLmtCas) {
   case 0:
    break;

   default:
    /* Sum: '<S459>/Sum' incorporates:
     *  Inport: '<Root>/Outport40'
     *  UnitDelay: '<S459>/Unit Delay'
     */
    rtb_MultiportSwitch2_a = Test_CoeffPGainCac - LGC_RawLmtCoeffPGainCas;

    /* Switch: '<S460>/Switch2' incorporates:
     *  RelationalOperator: '<S460>/LowerRelop1'
     */
    if (rtb_MultiportSwitch2_a > rtb_Product1_l5) {
      rtb_MultiportSwitch2_a = rtb_Product1_l5;
    } else {
      /* Product: '<S459>/Product1' incorporates:
       *  Constant: '<S458>/Constant1'
       *  Constant: '<S458>/Constant5'
       *  UnaryMinus: '<S458>/Unary Minus'
       */
      gm = -LGC_ThdPGainGrdCas_P * LGC_TimeSysCycle_P;

      /* Switch: '<S460>/Switch' incorporates:
       *  RelationalOperator: '<S460>/UpperRelop'
       */
      if (rtb_MultiportSwitch2_a < gm) {
        rtb_MultiportSwitch2_a = gm;
      }

      /* End of Switch: '<S460>/Switch' */
    }

    /* End of Switch: '<S460>/Switch2' */

    /* MultiPortSwitch: '<S459>/Multiport Switch1' incorporates:
     *  Sum: '<S459>/Difference Inputs3'
     *  UnitDelay: '<S459>/Unit Delay'
     *
     * Block description for '<S459>/Difference Inputs3':
     *
     *  Add in CPU
     */
    LGC_RawLmtCoeffPGainCas = rtb_MultiportSwitch2_a + LGC_RawLmtCoeffPGainCas;
    break;
  }

  /* MultiPortSwitch: '<S458>/Multiport Switch5' incorporates:
   *  Logic: '<S458>/AND'
   *  Logic: '<S458>/NOT'
   */
  if (LGC_EnaResetByTgq || (!LGC_EnaPGainGrdLmtCas)) {
    /* MultiPortSwitch: '<S458>/Multiport Switch5' incorporates:
     *  Inport: '<Root>/Outport40'
     */
    LGC_LmtCoeffPGainCas = Test_CoeffPGainCac;
  } else {
    /* MultiPortSwitch: '<S458>/Multiport Switch5' */
    LGC_LmtCoeffPGainCas = LGC_RawLmtCoeffPGainCas;
  }

  /* End of MultiPortSwitch: '<S458>/Multiport Switch5' */

  /* Product: '<S458>/Mul8' */
  LGC_DeltaFPGainCas = LGC_CtrlErrMainPGainCas * LGC_LmtCoeffPGainCas;

  /* Logic: '<S434>/OR3' incorporates:
   *  Constant: '<S434>/Parameter15'
   *  Constant: '<S434>/Parameter16'
   *  Constant: '<S434>/Parameter17'
   *  Logic: '<S434>/AND11'
   *  Logic: '<S434>/AND12'
   *  Logic: '<S434>/AND5'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR16'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR17'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR18'
   */
  LGC_EnaRstIntCac = ((((LGC_BtmDynIntRst_P & 8U) != 0U) && LGC_EnaRstByDistY) ||
                      (((LGC_BtmDynIntRst_P & 16U) != 0U) && LGC_EnaRstByTrq) ||
                      (((LGC_BtmDynIntRst_P & 32U) != 0U) &&
                       LGC_EnaRstByStandStill));

  /* Logic: '<S447>/OR' incorporates:
   *  Logic: '<S487>/OR'
   */
  rtb_OR_km_tmp = (LGC_EnaResetByTgq || LGC_EnaRstIntCac);

  /* Switch: '<S454>/Init' incorporates:
   *  Logic: '<S447>/OR'
   *  Logic: '<S454>/FixPt Logical Operator'
   *  UnitDelay: '<S454>/FixPt Unit Delay2'
   */
  if (rtb_OR_km_tmp || (TJATCT_DW.FixPtUnitDelay2_DSTATE_b1 != 0)) {
    /* Switch: '<S454>/Init' incorporates:
     *  Constant: '<S447>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_m = 0.0F;
  }

  /* End of Switch: '<S454>/Init' */

  /* Switch: '<S452>/Switch2' incorporates:
   *  Constant: '<S447>/P_TCTLGC_LdcILimit_rad'
   *  RelationalOperator: '<S452>/LowerRelop1'
   *  RelationalOperator: '<S452>/UpperRelop'
   *  Switch: '<S452>/Switch'
   *  UnaryMinus: '<S447>/Neg1'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_m > P_TCTLGC_CacILimit_rad) {
    /* Switch: '<S452>/Switch2' */
    LGC_DeltaFIGainCas = P_TCTLGC_CacILimit_rad;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_m < -P_TCTLGC_CacILimit_rad) {
    /* Switch: '<S452>/Switch' incorporates:
     *  Switch: '<S452>/Switch2'
     *  UnaryMinus: '<S447>/Neg1'
     */
    LGC_DeltaFIGainCas = -P_TCTLGC_CacILimit_rad;
  } else {
    /* Switch: '<S452>/Switch2' incorporates:
     *  Switch: '<S452>/Switch'
     */
    LGC_DeltaFIGainCas = TJATCT_DW.FixPtUnitDelay1_DSTATE_m;
  }

  /* End of Switch: '<S452>/Switch2' */

  /* MultiPortSwitch: '<S446>/Multiport Switch10' incorporates:
   *  Constant: '<S446>/Constant20'
   *  Constant: '<S446>/Parameter3'
   *  Logic: '<S446>/AND'
   *  Logic: '<S446>/NOT'
   *  Product: '<S446>/Divide'
   *  Sum: '<S446>/Sum4'
   *  UnitDelay: '<S446>/Unit_Delay1'
   */
  if (LGC_EnaResetByTgq && LGC_EnaFreezeByTgq) {
    rtb_MultiportSwitch2_a = 0.0F;
  } else {
    rtb_MultiportSwitch2_a = (LGC_DeltaFPGainCas -
      TJATCT_DW.Unit_Delay1_DSTATE_k) / LGC_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S446>/Multiport Switch10' */

  /* MultiPortSwitch: '<S457>/Multiport Switch7' incorporates:
   *  RelationalOperator: '<S457>/GreaterThan1'
   */
  if (rtb_Neg_i <= rtb_Product1_l5) {
    /* MultiPortSwitch: '<S457>/Multiport Switch7' incorporates:
     *  Constant: '<S457>/Constant10'
     */
    LGC_EnaPGainGrdSignCas = 1;
  } else {
    /* MultiPortSwitch: '<S457>/Multiport Switch7' incorporates:
     *  Constant: '<S457>/Constant11'
     */
    LGC_EnaPGainGrdSignCas = -1;
  }

  /* End of MultiPortSwitch: '<S457>/Multiport Switch7' */

  /* MultiPortSwitch: '<S446>/Multiport Switch11' incorporates:
   *  Constant: '<S446>/Constant17'
   *  Constant: '<S446>/Constant18'
   *  Product: '<S446>/Mul1'
   *  Product: '<S446>/Mul2'
   */
  if (!LGC_EnaPGainGrdThdCas) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = (real32_T)LGC_EnaPGainGrdSignCas *
      LGC_ThdPGainGrdLdc_P * LGC_CtrlErrMainPGainCas;
  }

  /* End of MultiPortSwitch: '<S446>/Multiport Switch11' */

  /* Product: '<S446>/Mul3' incorporates:
   *  Inport: '<Root>/Outport42'
   *  Sum: '<S446>/Sum'
   */
  rtb_Neg_i = (rtb_MultiportSwitch2_a - LGC_RawErrCourseDotCdc_tmp) *
    Test_CoeffDGainCac;

  /* Product: '<S449>/Divide' incorporates:
   *  Constant: '<S446>/Parameter2'
   *  Inport: '<Root>/Outport43'
   *  MinMax: '<S449>/Max1'
   *  Product: '<S489>/Divide'
   */
  gm = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P, Test_TimeDT1Cac);

  /* Switch: '<S450>/Switch2' incorporates:
   *  Constant: '<S449>/IAM_Ts_P1'
   *  Constant: '<S449>/IAM_Ts_P4'
   *  Product: '<S449>/Divide'
   *  RelationalOperator: '<S450>/LowerRelop1'
   *  RelationalOperator: '<S450>/UpperRelop'
   *  Switch: '<S450>/Switch'
   */
  if (gm > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else if (gm < 0.0F) {
    /* Switch: '<S450>/Switch' incorporates:
     *  Constant: '<S449>/IAM_Ts_P1'
     */
    rtb_MultiportSwitch2_a = 0.0F;
  } else {
    rtb_MultiportSwitch2_a = gm;
  }

  /* End of Switch: '<S450>/Switch2' */

  /* Sum: '<S449>/Add' incorporates:
   *  Product: '<S449>/Product'
   *  Sum: '<S449>/Subtract'
   *  UnitDelay: '<S449>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_o0 += (rtb_Neg_i - TJATCT_DW.UnitDelay_DSTATE_o0) *
    rtb_MultiportSwitch2_a;

  /* MultiPortSwitch: '<S446>/Multiport Switch9' incorporates:
   *  RelationalOperator: '<S446>/Equal1'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S446>/Multiport Switch9' */
    LGC_DeltaFDGainCas = rtb_Neg_i;
  } else {
    /* MultiPortSwitch: '<S446>/Multiport Switch9' incorporates:
     *  UnitDelay: '<S449>/Unit Delay'
     */
    LGC_DeltaFDGainCas = TJATCT_DW.UnitDelay_DSTATE_o0;
  }

  /* End of MultiPortSwitch: '<S446>/Multiport Switch9' */

  /* Logic: '<S434>/OR2' incorporates:
   *  Constant: '<S434>/Parameter3'
   *  Constant: '<S434>/Parameter4'
   *  Constant: '<S434>/Parameter5'
   *  Logic: '<S434>/AND1'
   *  Logic: '<S434>/AND2'
   *  Logic: '<S434>/AND3'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR3'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR4'
   *  S-Function (sfix_bitop): '<S434>/Bitwise OR5'
   */
  LGC_EnaRstPT1Cac = ((((LGC_BtmDynPT1Rst_P & 8U) != 0U) && LGC_EnaRstByDistY) ||
                      (((LGC_BtmDynPT1Rst_P & 16U) != 0U) && LGC_EnaRstByTrq) ||
                      (((LGC_CswDynPT1Rst_P & 32U) != 0U) &&
                       LGC_EnaRstByStandStill));

  /* Product: '<S455>/Divide' incorporates:
   *  Constant: '<S448>/Parameter'
   *  Inport: '<Root>/Outport45'
   *  MinMax: '<S455>/Max1'
   *  Product: '<S495>/Divide'
   */
  rtb_Product1_c = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    Test_TimePT1Cac);

  /* Switch: '<S456>/Switch2' incorporates:
   *  Constant: '<S455>/IAM_Ts_P1'
   *  Constant: '<S455>/IAM_Ts_P4'
   *  Product: '<S455>/Divide'
   *  RelationalOperator: '<S456>/LowerRelop1'
   *  RelationalOperator: '<S456>/UpperRelop'
   *  Switch: '<S456>/Switch'
   */
  if (rtb_Product1_c > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else if (rtb_Product1_c < 0.0F) {
    /* Switch: '<S456>/Switch' incorporates:
     *  Constant: '<S455>/IAM_Ts_P1'
     */
    rtb_MultiportSwitch2_a = 0.0F;
  } else {
    rtb_MultiportSwitch2_a = rtb_Product1_c;
  }

  /* End of Switch: '<S456>/Switch2' */

  /* Sum: '<S455>/Add' incorporates:
   *  Inport: '<Root>/Outport44'
   *  Product: '<S448>/Mul'
   *  Product: '<S455>/Product'
   *  Sum: '<S455>/Subtract'
   *  UnitDelay: '<S455>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_c += (LGC_DeltaFPGainCas * Test_CoeffPT1GainCac -
    TJATCT_DW.UnitDelay_DSTATE_c) * rtb_MultiportSwitch2_a;

  /* MultiPortSwitch: '<S448>/Multiport Switch8' incorporates:
   *  Logic: '<S448>/OR'
   */
  if (LGC_EnaResetByTgq || LGC_EnaRstPT1Cac) {
    /* MultiPortSwitch: '<S448>/Multiport Switch8' incorporates:
     *  Constant: '<S448>/Constant15'
     */
    LGC_DeltaFPT1GainCas = 0.0F;
  } else {
    /* MultiPortSwitch: '<S448>/Multiport Switch8' incorporates:
     *  UnitDelay: '<S455>/Unit Delay'
     */
    LGC_DeltaFPT1GainCas = TJATCT_DW.UnitDelay_DSTATE_c;
  }

  /* End of MultiPortSwitch: '<S448>/Multiport Switch8' */

  /* S-Function (sfix_bitop): '<S444>/Bitwise Operator' incorporates:
   *  Constant: '<S444>/P_TCTLGC_LdcMode_nu'
   *  S-Function (sfix_bitop): '<S484>/Bitwise Operator'
   */
  tmp_1 = LGC_CswCacMode_P & 1U;

  /* MultiPortSwitch: '<S444>/Multiport Switch' incorporates:
   *  Constant: '<S444>/P_TCTLGC_LdcMode_nu3'
   *  DataTypeConversion: '<S444>/Data Type Conversion'
   *  DataTypeConversion: '<S444>/Data Type Conversion1'
   *  MultiPortSwitch: '<S444>/Multiport Switch3'
   *  S-Function (sfix_bitop): '<S444>/Bitwise Operator'
   *  S-Function (sfix_bitop): '<S444>/Bitwise Operator3'
   */
  if (tmp_1 == 0U) {
    /* MultiPortSwitch: '<S444>/Multiport Switch' incorporates:
     *  Constant: '<S444>/Constant16'
     */
    LGC_LdcCmd_rad = 0.0F;
  } else {
    if ((LGC_CswCacMode_P & 4U) == 0U) {
      /* MultiPortSwitch: '<S444>/Multiport Switch3' incorporates:
       *  Constant: '<S444>/Constant4'
       */
      LGC_RawErrCourseDotCdc_tmp = 0.0F;
    } else {
      /* MultiPortSwitch: '<S444>/Multiport Switch3' */
      LGC_RawErrCourseDotCdc_tmp = LGC_DeltaFIGainCas;
    }

    /* MultiPortSwitch: '<S444>/Multiport Switch2' incorporates:
     *  Constant: '<S444>/Constant3'
     *  Constant: '<S444>/P_TCTLGC_LdcMode_nu2'
     *  DataTypeConversion: '<S444>/Data Type Conversion2'
     *  S-Function (sfix_bitop): '<S444>/Bitwise Operator2'
     */
    if ((LGC_CswCacMode_P & 2U) == 0U) {
      rtb_Divide_l_tmp = 0.0F;
    } else {
      rtb_Divide_l_tmp = LGC_DeltaFDGainCas;
    }

    /* End of MultiPortSwitch: '<S444>/Multiport Switch2' */

    /* MultiPortSwitch: '<S444>/Multiport Switch1' incorporates:
     *  Constant: '<S444>/Constant2'
     *  Constant: '<S444>/P_TCTLGC_LdcMode_nu1'
     *  DataTypeConversion: '<S444>/Data Type Conversion3'
     *  S-Function (sfix_bitop): '<S444>/Bitwise Operator1'
     */
    if ((LGC_CswCacMode_P & 8U) == 0U) {
      rtb_Product_dv = 0.0F;
    } else {
      rtb_Product_dv = LGC_DeltaFPT1GainCas;
    }

    /* End of MultiPortSwitch: '<S444>/Multiport Switch1' */

    /* MultiPortSwitch: '<S444>/Multiport Switch' incorporates:
     *  Sum: '<S444>/Sum1'
     *  Sum: '<S444>/Sum2'
     *  Sum: '<S444>/Sum3'
     */
    LGC_LdcCmd_rad = ((LGC_DeltaFPGainCas + LGC_RawErrCourseDotCdc_tmp) +
                      rtb_Divide_l_tmp) + rtb_Product_dv;
  }

  /* End of MultiPortSwitch: '<S444>/Multiport Switch' */

  /* Lookup_n-D: '<S503>/Y_TCTLGC_PT1YErrTime_sec' */
  LGC_TimeFltErrCourseCdc = LGC_TimeFltErrCourse;

  /* Product: '<S518>/Divide' incorporates:
   *  Constant: '<S503>/Parameter'
   *  MinMax: '<S518>/Max1'
   */
  rtb_MultiportSwitch2_a = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    LGC_TimeFltErrCourseCdc);

  /* Switch: '<S519>/Switch2' incorporates:
   *  Constant: '<S518>/IAM_Ts_P1'
   *  Constant: '<S518>/IAM_Ts_P4'
   *  RelationalOperator: '<S519>/LowerRelop1'
   *  RelationalOperator: '<S519>/UpperRelop'
   *  Switch: '<S519>/Switch'
   */
  if (rtb_MultiportSwitch2_a > 1.0F) {
    rtb_MultiportSwitch2_a = 1.0F;
  } else {
    if (rtb_MultiportSwitch2_a < 0.0F) {
      /* Switch: '<S519>/Switch' incorporates:
       *  Constant: '<S518>/IAM_Ts_P1'
       */
      rtb_MultiportSwitch2_a = 0.0F;
    }
  }

  /* End of Switch: '<S519>/Switch2' */

  /* Sum: '<S518>/Add' incorporates:
   *  Product: '<S518>/Product'
   *  Sum: '<S518>/Subtract'
   */
  LGC_RawFltErrCourseCdc = (CDC_CtrlErrHeading - LGC_RawFltErrCourseCdc) *
    rtb_MultiportSwitch2_a + LGC_RawFltErrCourseCdc;

  /* MultiPortSwitch: '<S503>/Multiport Switch4' incorporates:
   *  Constant: '<S503>/Constant1'
   *  Logic: '<S503>/AND'
   *  Logic: '<S503>/NOT'
   */
  if (LGC_EnaResetByTgq || (LGC_CswFltErrCourse_P == 0)) {
    /* MultiPortSwitch: '<S503>/Multiport Switch4' */
    LGC_FltErrCourseCdc = CDC_CtrlErrHeading;
  } else {
    /* MultiPortSwitch: '<S503>/Multiport Switch4' */
    LGC_FltErrCourseCdc = LGC_RawFltErrCourseCdc;
  }

  /* End of MultiPortSwitch: '<S503>/Multiport Switch4' */

  /* Switch: '<S514>/Init' incorporates:
   *  Logic: '<S514>/FixPt Logical Operator'
   *  UnitDelay: '<S514>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_b3 != 0)) {
    /* Switch: '<S514>/Init' incorporates:
     *  Constant: '<S507>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S514>/Init' */

  /* Switch: '<S512>/Switch2' incorporates:
   *  Constant: '<S507>/Constant4'
   *  Constant: '<S507>/Constant5'
   *  RelationalOperator: '<S512>/LowerRelop1'
   *  RelationalOperator: '<S512>/UpperRelop'
   *  Switch: '<S512>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_d > 100.0F) {
    /* Switch: '<S512>/Switch2' */
    LGC_CmpnLaDmcCdc = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_d < -100.0F) {
    /* Switch: '<S512>/Switch' incorporates:
     *  Constant: '<S507>/Constant5'
     *  Switch: '<S512>/Switch2'
     */
    LGC_CmpnLaDmcCdc = -100.0F;
  } else {
    /* Switch: '<S512>/Switch2' incorporates:
     *  Switch: '<S512>/Switch'
     */
    LGC_CmpnLaDmcCdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_d;
  }

  /* End of Switch: '<S512>/Switch2' */

  /* MultiPortSwitch: '<S501>/Multiport Switch5' incorporates:
   *  Constant: '<S501>/Constant'
   */
  if (LGC_CswLaDmcCmpnCac_P == 0) {
    /* MultiPortSwitch: '<S501>/Multiport Switch5' */
    LGC_ErrCtrlCourseCdc = LGC_FltErrCourseCdc;
  } else {
    /* MultiPortSwitch: '<S501>/Multiport Switch5' */
    LGC_ErrCtrlCourseCdc = LGC_CmpnLaDmcCdc;
  }

  /* End of MultiPortSwitch: '<S501>/Multiport Switch5' */

  /* Product: '<S498>/Mul4' incorporates:
   *  Inport: '<Root>/Outport39'
   */
  LGC_CtrlErrMainPGainCdc = LGC_ErrCtrlCourseCdc * Test_CoeffMainPGainCac;

  /* MultiPortSwitch: '<S497>/Multiport Switch4' incorporates:
   *  Constant: '<S497>/Constant'
   *  Logic: '<S497>/NOT'
   *  UnitDelay: '<S497>/Unit Delay'
   */
  if (LGC_EnaResetByTgq) {
    rtb_Neg_i = 0.0F;
  } else {
    rtb_Neg_i = TJATCT_DW.UnitDelay_DSTATE_bp;
  }

  /* End of MultiPortSwitch: '<S497>/Multiport Switch4' */

  /* Logic: '<S497>/OR1' incorporates:
   *  UnitDelay: '<S497>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_bp = ((LGC_EnaActSafetyFcn != 0) ||
    (LGC_EnaActObjFollow != 0));

  /* RelationalOperator: '<S497>/NotEqual' incorporates:
   *  UnitDelay: '<S497>/Unit Delay'
   */
  LGC_EnaModeChangeCdc = (rtb_Neg_i != TJATCT_DW.UnitDelay_DSTATE_bp);

  /* Sum: '<S497>/Subtract' incorporates:
   *  Inport: '<Root>/Outport40'
   *  UnitDelay: '<S497>/Unit Delay1'
   */
  rtb_Neg_i = Test_CoeffPGainCac - TJATCT_DW.UnitDelay1_DSTATE_o;

  /* Product: '<S497>/Product' incorporates:
   *  Constant: '<S497>/Constant6'
   *  Constant: '<S497>/Constant7'
   *  Product: '<S497>/Product1'
   */
  rtb_Unit_Delay2_o = LGC_ThdPGainGrdCac_P * LGC_TimeSysCycle_P;

  /* RelationalOperator: '<S497>/GreaterThan' incorporates:
   *  Abs: '<S497>/Abs'
   *  Product: '<S497>/Product'
   */
  LGC_EnaPGainGrdThdCdc = (fabsf(rtb_Neg_i) > rtb_Unit_Delay2_o);

  /* Logic: '<S497>/OR3' */
  LGC_EnaPGainGrdLmtCdc = (LGC_EnaModeChangeCdc || LGC_EnaPGainGrdThdCdc);

  /* MultiPortSwitch: '<S499>/Multiport Switch1' */
  switch ((int32_T)LGC_EnaPGainGrdLmtCdc) {
   case 0:
    break;

   default:
    /* Sum: '<S499>/Sum' incorporates:
     *  Inport: '<Root>/Outport40'
     *  UnitDelay: '<S499>/Unit Delay'
     */
    rtb_MultiportSwitch2_a = Test_CoeffPGainCac - LGC_RawLmtCoeffPGainCdc;

    /* Switch: '<S500>/Switch2' incorporates:
     *  RelationalOperator: '<S500>/LowerRelop1'
     */
    if (rtb_MultiportSwitch2_a > rtb_Product1_l5) {
      rtb_MultiportSwitch2_a = rtb_Product1_l5;
    } else {
      /* Product: '<S499>/Product1' incorporates:
       *  Constant: '<S498>/Constant1'
       *  Constant: '<S498>/Constant5'
       *  UnaryMinus: '<S498>/Unary Minus'
       */
      rtb_Product1_l5 = -LGC_ThdPGainGrdCas_P * LGC_TimeSysCycle_P;

      /* Switch: '<S500>/Switch' incorporates:
       *  RelationalOperator: '<S500>/UpperRelop'
       */
      if (rtb_MultiportSwitch2_a < rtb_Product1_l5) {
        rtb_MultiportSwitch2_a = rtb_Product1_l5;
      }

      /* End of Switch: '<S500>/Switch' */
    }

    /* End of Switch: '<S500>/Switch2' */

    /* MultiPortSwitch: '<S499>/Multiport Switch1' incorporates:
     *  Sum: '<S499>/Difference Inputs3'
     *  UnitDelay: '<S499>/Unit Delay'
     *
     * Block description for '<S499>/Difference Inputs3':
     *
     *  Add in CPU
     */
    LGC_RawLmtCoeffPGainCdc = rtb_MultiportSwitch2_a + LGC_RawLmtCoeffPGainCdc;
    break;
  }

  /* MultiPortSwitch: '<S498>/Multiport Switch5' incorporates:
   *  Logic: '<S498>/AND'
   *  Logic: '<S498>/NOT'
   */
  if (LGC_EnaResetByTgq || (!LGC_EnaPGainGrdLmtCdc)) {
    /* MultiPortSwitch: '<S498>/Multiport Switch5' incorporates:
     *  Inport: '<Root>/Outport40'
     */
    LGC_LmtCoeffPGainCdc = Test_CoeffPGainCac;
  } else {
    /* MultiPortSwitch: '<S498>/Multiport Switch5' */
    LGC_LmtCoeffPGainCdc = LGC_RawLmtCoeffPGainCdc;
  }

  /* End of MultiPortSwitch: '<S498>/Multiport Switch5' */

  /* Product: '<S498>/Mul8' */
  LGC_DeltaFPGainCdc = LGC_CtrlErrMainPGainCdc * LGC_LmtCoeffPGainCdc;

  /* Switch: '<S494>/Init' incorporates:
   *  Logic: '<S494>/FixPt Logical Operator'
   *  UnitDelay: '<S494>/FixPt Unit Delay2'
   */
  if (rtb_OR_km_tmp || (TJATCT_DW.FixPtUnitDelay2_DSTATE_ms != 0)) {
    /* Switch: '<S494>/Init' incorporates:
     *  Constant: '<S487>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S494>/Init' */

  /* Switch: '<S492>/Switch2' incorporates:
   *  Constant: '<S487>/P_TCTLGC_LdcILimit_rad'
   *  RelationalOperator: '<S492>/LowerRelop1'
   *  RelationalOperator: '<S492>/UpperRelop'
   *  Switch: '<S492>/Switch'
   *  UnaryMinus: '<S487>/Neg1'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_i > P_TCTLGC_CacILimit_rad) {
    /* Switch: '<S492>/Switch2' */
    LGC_DeltaFIGainCdc = P_TCTLGC_CacILimit_rad;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_i < -P_TCTLGC_CacILimit_rad) {
    /* Switch: '<S492>/Switch' incorporates:
     *  Switch: '<S492>/Switch2'
     *  UnaryMinus: '<S487>/Neg1'
     */
    LGC_DeltaFIGainCdc = -P_TCTLGC_CacILimit_rad;
  } else {
    /* Switch: '<S492>/Switch2' incorporates:
     *  Switch: '<S492>/Switch'
     */
    LGC_DeltaFIGainCdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_i;
  }

  /* End of Switch: '<S492>/Switch2' */

  /* MultiPortSwitch: '<S486>/Multiport Switch10' incorporates:
   *  Constant: '<S486>/Constant20'
   *  Constant: '<S486>/Parameter3'
   *  Logic: '<S486>/AND'
   *  Logic: '<S486>/NOT'
   *  Product: '<S486>/Divide'
   *  Sum: '<S486>/Sum4'
   *  UnitDelay: '<S486>/Unit_Delay1'
   */
  if (LGC_EnaResetByTgq && LGC_EnaFreezeByTgq) {
    rtb_MultiportSwitch2_a = 0.0F;
  } else {
    rtb_MultiportSwitch2_a = (LGC_DeltaFPGainCdc -
      TJATCT_DW.Unit_Delay1_DSTATE_m) / LGC_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S486>/Multiport Switch10' */

  /* MultiPortSwitch: '<S497>/Multiport Switch7' incorporates:
   *  RelationalOperator: '<S497>/GreaterThan1'
   */
  if (rtb_Neg_i <= rtb_Unit_Delay2_o) {
    /* MultiPortSwitch: '<S497>/Multiport Switch7' incorporates:
     *  Constant: '<S497>/Constant10'
     */
    LGC_EnaPGainGrdSignCdc = 1;
  } else {
    /* MultiPortSwitch: '<S497>/Multiport Switch7' incorporates:
     *  Constant: '<S497>/Constant11'
     */
    LGC_EnaPGainGrdSignCdc = -1;
  }

  /* End of MultiPortSwitch: '<S497>/Multiport Switch7' */

  /* MultiPortSwitch: '<S486>/Multiport Switch11' incorporates:
   *  Constant: '<S486>/Constant17'
   *  Constant: '<S486>/Constant18'
   *  Product: '<S486>/Mul1'
   *  Product: '<S486>/Mul2'
   */
  if (!LGC_EnaPGainGrdThdCdc) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = (real32_T)LGC_EnaPGainGrdSignCdc *
      LGC_ThdPGainGrdLdc_P * LGC_CtrlErrMainPGainCdc;
  }

  /* End of MultiPortSwitch: '<S486>/Multiport Switch11' */

  /* Product: '<S486>/Mul3' incorporates:
   *  Inport: '<Root>/Outport42'
   *  Sum: '<S486>/Sum'
   */
  rtb_Neg_i = (rtb_MultiportSwitch2_a - LGC_RawErrCourseDotCdc_tmp) *
    Test_CoeffDGainCac;

  /* Switch: '<S490>/Switch2' incorporates:
   *  Constant: '<S489>/IAM_Ts_P1'
   *  Constant: '<S489>/IAM_Ts_P4'
   *  RelationalOperator: '<S490>/LowerRelop1'
   *  RelationalOperator: '<S490>/UpperRelop'
   *  Switch: '<S490>/Switch'
   */
  if (gm > 1.0F) {
    gm = 1.0F;
  } else {
    if (gm < 0.0F) {
      /* Switch: '<S490>/Switch' incorporates:
       *  Constant: '<S489>/IAM_Ts_P1'
       */
      gm = 0.0F;
    }
  }

  /* End of Switch: '<S490>/Switch2' */

  /* Sum: '<S489>/Add' incorporates:
   *  Product: '<S489>/Product'
   *  Sum: '<S489>/Subtract'
   *  UnitDelay: '<S489>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_bw += (rtb_Neg_i - TJATCT_DW.UnitDelay_DSTATE_bw) *
    gm;

  /* MultiPortSwitch: '<S486>/Multiport Switch9' incorporates:
   *  RelationalOperator: '<S486>/Equal1'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S486>/Multiport Switch9' */
    LGC_DeltaFDGainCdc = rtb_Neg_i;
  } else {
    /* MultiPortSwitch: '<S486>/Multiport Switch9' incorporates:
     *  UnitDelay: '<S489>/Unit Delay'
     */
    LGC_DeltaFDGainCdc = TJATCT_DW.UnitDelay_DSTATE_bw;
  }

  /* End of MultiPortSwitch: '<S486>/Multiport Switch9' */

  /* Switch: '<S496>/Switch2' incorporates:
   *  Constant: '<S495>/IAM_Ts_P1'
   *  Constant: '<S495>/IAM_Ts_P4'
   *  RelationalOperator: '<S496>/LowerRelop1'
   *  RelationalOperator: '<S496>/UpperRelop'
   *  Switch: '<S496>/Switch'
   */
  if (rtb_Product1_c > 1.0F) {
    rtb_Product1_c = 1.0F;
  } else {
    if (rtb_Product1_c < 0.0F) {
      /* Switch: '<S496>/Switch' incorporates:
       *  Constant: '<S495>/IAM_Ts_P1'
       */
      rtb_Product1_c = 0.0F;
    }
  }

  /* End of Switch: '<S496>/Switch2' */

  /* Sum: '<S495>/Add' incorporates:
   *  Inport: '<Root>/Outport44'
   *  Product: '<S488>/Mul'
   *  Product: '<S495>/Product'
   *  Sum: '<S495>/Subtract'
   *  UnitDelay: '<S495>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_iu += (LGC_DeltaFPGainCdc * Test_CoeffPT1GainCac -
    TJATCT_DW.UnitDelay_DSTATE_iu) * rtb_Product1_c;

  /* MultiPortSwitch: '<S488>/Multiport Switch8' incorporates:
   *  Logic: '<S488>/OR'
   */
  if (LGC_EnaResetByTgq || LGC_EnaRstPT1Cac) {
    /* MultiPortSwitch: '<S488>/Multiport Switch8' incorporates:
     *  Constant: '<S488>/Constant15'
     */
    LGC_DeltaFPT1GainCdc = 0.0F;
  } else {
    /* MultiPortSwitch: '<S488>/Multiport Switch8' incorporates:
     *  UnitDelay: '<S495>/Unit Delay'
     */
    LGC_DeltaFPT1GainCdc = TJATCT_DW.UnitDelay_DSTATE_iu;
  }

  /* End of MultiPortSwitch: '<S488>/Multiport Switch8' */

  /* MultiPortSwitch: '<S484>/Multiport Switch' incorporates:
   *  Constant: '<S484>/P_TCTLGC_LdcMode_nu3'
   *  DataTypeConversion: '<S484>/Data Type Conversion'
   *  DataTypeConversion: '<S484>/Data Type Conversion1'
   *  MultiPortSwitch: '<S484>/Multiport Switch3'
   *  S-Function (sfix_bitop): '<S484>/Bitwise Operator3'
   */
  if (tmp_1 == 0U) {
    /* MultiPortSwitch: '<S484>/Multiport Switch' incorporates:
     *  Constant: '<S484>/Constant16'
     */
    LGC_DeltaFCmdCdc = 0.0F;
  } else {
    if ((LGC_CswCacMode_P & 4U) == 0U) {
      /* MultiPortSwitch: '<S484>/Multiport Switch3' incorporates:
       *  Constant: '<S484>/Constant4'
       */
      LGC_RawErrCourseDotCdc_tmp = 0.0F;
    } else {
      /* MultiPortSwitch: '<S484>/Multiport Switch3' */
      LGC_RawErrCourseDotCdc_tmp = LGC_DeltaFIGainCdc;
    }

    /* MultiPortSwitch: '<S484>/Multiport Switch2' incorporates:
     *  Constant: '<S484>/Constant3'
     *  Constant: '<S484>/P_TCTLGC_LdcMode_nu2'
     *  DataTypeConversion: '<S484>/Data Type Conversion2'
     *  S-Function (sfix_bitop): '<S484>/Bitwise Operator2'
     */
    if ((LGC_CswCacMode_P & 2U) == 0U) {
      rtb_Divide_l_tmp = 0.0F;
    } else {
      rtb_Divide_l_tmp = LGC_DeltaFDGainCdc;
    }

    /* End of MultiPortSwitch: '<S484>/Multiport Switch2' */

    /* MultiPortSwitch: '<S484>/Multiport Switch1' incorporates:
     *  Constant: '<S484>/Constant2'
     *  Constant: '<S484>/P_TCTLGC_LdcMode_nu1'
     *  DataTypeConversion: '<S484>/Data Type Conversion3'
     *  S-Function (sfix_bitop): '<S484>/Bitwise Operator1'
     */
    if ((LGC_CswCacMode_P & 8U) == 0U) {
      rtb_Product_dv = 0.0F;
    } else {
      rtb_Product_dv = LGC_DeltaFPT1GainCdc;
    }

    /* End of MultiPortSwitch: '<S484>/Multiport Switch1' */

    /* MultiPortSwitch: '<S484>/Multiport Switch' incorporates:
     *  Sum: '<S484>/Sum1'
     *  Sum: '<S484>/Sum2'
     *  Sum: '<S484>/Sum3'
     */
    LGC_DeltaFCmdCdc = ((LGC_DeltaFPGainCdc + LGC_RawErrCourseDotCdc_tmp) +
                        rtb_Divide_l_tmp) + rtb_Product_dv;
  }

  /* End of MultiPortSwitch: '<S484>/Multiport Switch' */

  /* Gain: '<S439>/Gain' */
  data_log_5 = 0.0F;

  /* Sum: '<S439>/Sum' */
  LGC_DeltaFCmd_rad = LGC_LdcCmd_rad + data_log_5;

  /* Product: '<S564>/Mul' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Product: '<S134>/Product1'
   *  Product: '<S141>/Mul'
   *  Product: '<S142>/Mul'
   */
  rtb_Divide_bo = TCTI_VehicleVelX * TCTI_VehicleVelX;

  /* Product: '<S564>/Divide' incorporates:
   *  Constant: '<S564>/Constant1'
   *  Constant: '<S564>/Constant2'
   *  Constant: '<S564>/Constant4'
   *  Product: '<S564>/Mul'
   *  Product: '<S564>/Mul1'
   *  Sum: '<S564>/Sum'
   *  Sum: '<S564>/Sum1'
   */
  LGC_CtrlCrv_1pm = LGC_DeltaFCmd_rad / ((rtb_Divide_bo *
    P_VEH_SelfSteeringGrd_nu + P_VEH_DistCogToFrontAxle_m) +
    P_VEH_DistCogToRearAxle_m);

  /* RelationalOperator: '<S201>/Equal1' incorporates:
   *  Constant: '<S201>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  DTE_EnaCtrlByTgq = (TCTI_BtfTrajGuiQualifier != DTE_EnaOffByTgq_P);

  /* RelationalOperator: '<S201>/Equal' incorporates:
   *  Constant: '<S201>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  DTE_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == DTE_EnaFreezeByTgq_P);

  /* Lookup_n-D: '<S239>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_MaxReqCrvByDstrb = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(DTE_MaxReqCrvByDstrb_X[0])), ((const real32_T *)&(DTE_MaxReqCrvByDstrb_M[0])),
    6U);

  /* Lookup_n-D: '<S238>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_SetCrvGainLaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(DTE_SetCrvGainLaDmc_X[0])), ((const real32_T *)&(DTE_SetCrvGainLaDmc_M[0])),
    12U);

  /* Product: '<S282>/Divide' incorporates:
   *  Constant: '<S277>/Parameter1'
   *  Constant: '<S277>/Parameter2'
   *  MinMax: '<S282>/Max1'
   *  Product: '<S250>/Divide'
   */
  rtb_Divide_bb = DTE_TimeSysCycle_P / fmaxf(DTE_TimeSysCycle_P,
    DTE_TimeSysCycle_P);

  /* DataTypeConversion: '<S73>/Data Type Conversion' */
  EST_CurSteerAngle = EST_AngleCurSteer;

  /* UnitDelay: '<S277>/Unit_Delay' */
  DTE_DlyCurSteerAngle = TJATCT_DW.Unit_Delay_DSTATE_l;

  /* UnitDelay: '<S277>/Unit_Delay1' */
  rtb_MultiportSwitch2_a = TJATCT_DW.Unit_Delay1_DSTATE_d4;

  /* UnitDelay: '<S277>/Unit_Delay2' */
  gm = TJATCT_DW.Unit_Delay2_DSTATE_h;

  /* Update for UnitDelay: '<S277>/Unit_Delay4' incorporates:
   *  UnitDelay: '<S277>/Unit_Delay3'
   */
  TJATCT_DW.Unit_Delay4_DSTATE_c = TJATCT_DW.Unit_Delay3_DSTATE_a;

  /* Switch: '<S283>/Switch2' incorporates:
   *  Constant: '<S282>/IAM_Ts_P1'
   *  Constant: '<S282>/IAM_Ts_P4'
   *  Product: '<S282>/Divide'
   *  RelationalOperator: '<S283>/LowerRelop1'
   *  RelationalOperator: '<S283>/UpperRelop'
   *  Switch: '<S283>/Switch'
   */
  if (rtb_Divide_bb > 1.0F) {
    rtb_Product1_c = 1.0F;
  } else if (rtb_Divide_bb < 0.0F) {
    /* Switch: '<S283>/Switch' incorporates:
     *  Constant: '<S282>/IAM_Ts_P1'
     */
    rtb_Product1_c = 0.0F;
  } else {
    rtb_Product1_c = rtb_Divide_bb;
  }

  /* End of Switch: '<S283>/Switch2' */

  /* Sum: '<S282>/Add' incorporates:
   *  Product: '<S282>/Product'
   *  Sum: '<S282>/Subtract'
   */
  DTE_FltDlyCurSteerAngle = (DTE_DlyCurSteerAngle - DTE_FltDlyCurSteerAngle) *
    rtb_Product1_c + DTE_FltDlyCurSteerAngle;

  /* RelationalOperator: '<S201>/Equal3' incorporates:
   *  Constant: '<S201>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3_n = (TCTI_BtfTrajGuiQualifier == DTE_EnaFreezeByTgq_P);

  /* Logic: '<S201>/NOT' incorporates:
   *  Logic: '<S212>/AND'
   *  Logic: '<S213>/AND'
   *  Logic: '<S239>/AND'
   *  Logic: '<S267>/AND'
   */
  rtb_Equal3_c = !DTE_EnaCtrlByTgq;

  /* Logic: '<S201>/OR' incorporates:
   *  Logic: '<S201>/NOT'
   *  RelationalOperator: '<S216>/FixPt Relational Operator'
   *  UnitDelay: '<S216>/Delay Input1'
   *
   * Block description for '<S216>/Delay Input1':
   *
   *  Store in Global RAM
   */
  DTE_EnaResetByTgq = (rtb_Equal3_c || ((int32_T)rtb_Equal3_n < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_g0));

  /* Switch: '<S290>/Init' incorporates:
   *  Logic: '<S290>/FixPt Logical Operator'
   *  UnitDelay: '<S290>/FixPt Unit Delay2'
   */
  if (DTE_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_e != 0)) {
    /* Switch: '<S290>/Init' incorporates:
     *  Constant: '<S285>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ov = 0.0F;
  }

  /* End of Switch: '<S290>/Init' */

  /* Switch: '<S288>/Switch2' incorporates:
   *  Constant: '<S285>/Constant3'
   *  Constant: '<S285>/Constant5'
   *  RelationalOperator: '<S288>/LowerRelop1'
   *  RelationalOperator: '<S288>/UpperRelop'
   *  Switch: '<S288>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ov > 10000.0F) {
    /* Switch: '<S288>/Switch2' */
    DTE_DeltaVdyFcn = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ov < -10000.0F) {
    /* Switch: '<S288>/Switch' incorporates:
     *  Constant: '<S285>/Constant5'
     *  Switch: '<S288>/Switch2'
     */
    DTE_DeltaVdyFcn = -10000.0F;
  } else {
    /* Switch: '<S288>/Switch2' incorporates:
     *  Switch: '<S288>/Switch'
     */
    DTE_DeltaVdyFcn = TJATCT_DW.FixPtUnitDelay1_DSTATE_ov;
  }

  /* End of Switch: '<S288>/Switch2' */

  /* Sum: '<S277>/Subtract' */
  DTE_DeltaByVdyFcn = DTE_FltDlyCurSteerAngle - DTE_DeltaVdyFcn;

  /* Switch: '<S234>/Init' incorporates:
   *  Logic: '<S234>/FixPt Logical Operator'
   *  UnitDelay: '<S234>/FixPt Unit Delay2'
   */
  if ((DTE_DeltaByVdyFcn != 0.0F) || (TJATCT_DW.FixPtUnitDelay2_DSTATE_j != 0))
  {
    /* Switch: '<S234>/Init' incorporates:
     *  Constant: '<S225>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cn = 0.0F;
  }

  /* End of Switch: '<S234>/Init' */

  /* Switch: '<S232>/Switch2' incorporates:
   *  Constant: '<S225>/Constant3'
   *  Constant: '<S225>/Constant5'
   *  RelationalOperator: '<S232>/LowerRelop1'
   *  RelationalOperator: '<S232>/UpperRelop'
   *  Switch: '<S232>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cn > 10000.0F) {
    /* Switch: '<S232>/Switch2' */
    DTE_SetCrvLaDmc = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cn < -10000.0F) {
    /* Switch: '<S232>/Switch' incorporates:
     *  Constant: '<S225>/Constant5'
     *  Switch: '<S232>/Switch2'
     */
    DTE_SetCrvLaDmc = -10000.0F;
  } else {
    /* Switch: '<S232>/Switch2' incorporates:
     *  Switch: '<S232>/Switch'
     */
    DTE_SetCrvLaDmc = TJATCT_DW.FixPtUnitDelay1_DSTATE_cn;
  }

  /* End of Switch: '<S232>/Switch2' */

  /* UnitDelay: '<S238>/Unit_Delay' */
  rtb_Product1_c = TJATCT_DW.Unit_Delay_DSTATE_n0;

  /* UnitDelay: '<S238>/Unit_Delay1' */
  rtb_Product1_l5 = TJATCT_DW.Unit_Delay1_DSTATE_ea;

  /* UnitDelay: '<S238>/Unit_Delay2' */
  rtb_Unit_Delay2_o = TJATCT_DW.Unit_Delay2_DSTATE_ge;

  /* UnitDelay: '<S238>/Unit_Delay3' */
  rtb_Unit_Delay3_ia = TJATCT_DW.Unit_Delay3_DSTATE_b;

  /* MultiPortSwitch: '<S238>/Multiport_Switch1' incorporates:
   *  Constant: '<S238>/Parameter1'
   */
  switch (DTE_DelayCyclesLaDMC_P) {
   case 0:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' */
    DTE_NdlySetCrvLaDmc = DTE_SetCrvLaDmc;
    break;

   case 1:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' */
    DTE_NdlySetCrvLaDmc = rtb_Product1_c;
    break;

   case 2:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S238>/Unit_Delay1'
     */
    DTE_NdlySetCrvLaDmc = TJATCT_DW.Unit_Delay1_DSTATE_ea;
    break;

   case 3:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S238>/Unit_Delay2'
     */
    DTE_NdlySetCrvLaDmc = TJATCT_DW.Unit_Delay2_DSTATE_ge;
    break;

   case 4:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S238>/Unit_Delay3'
     */
    DTE_NdlySetCrvLaDmc = TJATCT_DW.Unit_Delay3_DSTATE_b;
    break;

   default:
    /* MultiPortSwitch: '<S238>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S238>/Unit_Delay4'
     */
    DTE_NdlySetCrvLaDmc = TJATCT_DW.Unit_Delay4_DSTATE_i;
    break;
  }

  /* End of MultiPortSwitch: '<S238>/Multiport_Switch1' */

  /* Product: '<S238>/Product' */
  DTE_RawReqCrvByDstrb = DTE_SetCrvGainLaDmc * DTE_NdlySetCrvLaDmc;

  /* Switch: '<S240>/Switch2' incorporates:
   *  RelationalOperator: '<S240>/LowerRelop1'
   *  RelationalOperator: '<S240>/UpperRelop'
   *  Switch: '<S240>/Switch'
   *  UnaryMinus: '<S239>/Unary Minus'
   */
  if (DTE_RawReqCrvByDstrb > DTE_MaxReqCrvByDstrb) {
    /* Switch: '<S240>/Switch2' */
    DTE_LmtReqCrvByDstrb = DTE_MaxReqCrvByDstrb;
  } else if (DTE_RawReqCrvByDstrb < -DTE_MaxReqCrvByDstrb) {
    /* Switch: '<S240>/Switch' incorporates:
     *  Switch: '<S240>/Switch2'
     *  UnaryMinus: '<S239>/Unary Minus'
     */
    DTE_LmtReqCrvByDstrb = -DTE_MaxReqCrvByDstrb;
  } else {
    /* Switch: '<S240>/Switch2' incorporates:
     *  Switch: '<S240>/Switch'
     */
    DTE_LmtReqCrvByDstrb = DTE_RawReqCrvByDstrb;
  }

  /* End of Switch: '<S240>/Switch2' */

  /* MultiPortSwitch: '<S239>/Multiport Switch1' */
  if (!DTE_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S239>/Multiport Switch1' */
    DTE_HldReqCrvByDstrb = DTE_LmtReqCrvByDstrb;
  }

  /* End of MultiPortSwitch: '<S239>/Multiport Switch1' */

  /* MultiPortSwitch: '<S239>/Multiport Switch' incorporates:
   *  Constant: '<S239>/Parameter'
   *  Logic: '<S239>/AND'
   */
  if (rtb_Equal3_c || (DTE_CswDstrbCmpn_P == 0)) {
    /* MultiPortSwitch: '<S239>/Multiport Switch' incorporates:
     *  Constant: '<S239>/Constant'
     */
    DTE_ReqCrvByDstrb = 0.0F;
  } else {
    /* MultiPortSwitch: '<S239>/Multiport Switch' */
    DTE_ReqCrvByDstrb = DTE_HldReqCrvByDstrb;
  }

  /* End of MultiPortSwitch: '<S239>/Multiport Switch' */

  /* Lookup_n-D: '<S212>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_MaxCrvByBnkAgl = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(DTE_MaxCrvByBnkAgl_X[0])), ((const real32_T *)&(DTE_MaxCrvByBnkAgl_M[0])),
    6U);

  /* Product: '<S208>/Divide' incorporates:
   *  Constant: '<S206>/Parameter'
   *  Constant: '<S206>/Parameter1'
   *  MinMax: '<S208>/Max1'
   */
  rtb_Neg_i = DTE_TimeSysCycle_P / fmaxf(DTE_TimeSysCycle_P,
    DTE_TimeFiterReqCrv_P);

  /* Switch: '<S217>/Switch2' incorporates:
   *  Constant: '<S201>/Parameter'
   *  Constant: '<S201>/Parameter3'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  RelationalOperator: '<S217>/LowerRelop1'
   *  RelationalOperator: '<S217>/UpperRelop'
   *  Switch: '<S217>/Switch'
   *  UnaryMinus: '<S201>/Unary Minus'
   */
  if (TCTI_VehicleVelX > DTE_ThdVehVelX_P) {
    /* Switch: '<S217>/Switch2' */
    DTE_LmtVehVelX = DTE_ThdVehVelX_P;
  } else if (TCTI_VehicleVelX < -DTE_ThdVehVelX_P) {
    /* Switch: '<S217>/Switch' incorporates:
     *  Constant: '<S201>/Parameter3'
     *  Switch: '<S217>/Switch2'
     *  UnaryMinus: '<S201>/Unary Minus'
     */
    DTE_LmtVehVelX = -DTE_ThdVehVelX_P;
  } else {
    /* Switch: '<S217>/Switch2' */
    DTE_LmtVehVelX = TCTI_VehicleVelX;
  }

  /* End of Switch: '<S217>/Switch2' */

  /* MultiPortSwitch: '<S206>/Multiport Switch' incorporates:
   *  Constant: '<S206>/Constant3'
   *  Inport: '<Root>/TCTI_RoadBankAngle'
   *  RelationalOperator: '<S206>/GreaterThan'
   */
  if (TCTI_RoadBankAngle <= 0.0F) {
    /* MultiPortSwitch: '<S206>/Multiport Switch' incorporates:
     *  Abs: '<S206>/Abs1'
     *  Constant: '<S206>/Constant2'
     *  Product: '<S206>/Divide1'
     *  Product: '<S206>/Product'
     *  Product: '<S206>/Product3'
     *  Trigonometry: '<S206>/Sin'
     */
    DTE_EstCrvByBnkAgl = (real32_T)(sinf(fabsf(TCTI_RoadBankAngle)) * 9.8 /
      (DTE_LmtVehVelX * DTE_LmtVehVelX));
  } else {
    /* MultiPortSwitch: '<S206>/Multiport Switch' incorporates:
     *  Abs: '<S206>/Abs1'
     *  Constant: '<S206>/Constant2'
     *  Product: '<S206>/Divide1'
     *  Product: '<S206>/Product'
     *  Product: '<S206>/Product3'
     *  Trigonometry: '<S206>/Sin'
     *  UnaryMinus: '<S206>/Unary Minus'
     */
    DTE_EstCrvByBnkAgl = -(real32_T)(sinf(fabsf(TCTI_RoadBankAngle)) * 9.8 /
      (DTE_LmtVehVelX * DTE_LmtVehVelX));
  }

  /* End of MultiPortSwitch: '<S206>/Multiport Switch' */

  /* Switch: '<S209>/Switch2' incorporates:
   *  Constant: '<S208>/IAM_Ts_P1'
   *  Constant: '<S208>/IAM_Ts_P4'
   *  RelationalOperator: '<S209>/LowerRelop1'
   *  RelationalOperator: '<S209>/UpperRelop'
   *  Switch: '<S209>/Switch'
   */
  if (rtb_Neg_i > 1.0F) {
    rtb_Neg_i = 1.0F;
  } else {
    if (rtb_Neg_i < 0.0F) {
      /* Switch: '<S209>/Switch' incorporates:
       *  Constant: '<S208>/IAM_Ts_P1'
       */
      rtb_Neg_i = 0.0F;
    }
  }

  /* End of Switch: '<S209>/Switch2' */

  /* Sum: '<S208>/Add' incorporates:
   *  Product: '<S208>/Product'
   *  Sum: '<S208>/Subtract'
   */
  DTE_RawFltEstCrvByBnkAgl = (DTE_EstCrvByBnkAgl - DTE_RawFltEstCrvByBnkAgl) *
    rtb_Neg_i + DTE_RawFltEstCrvByBnkAgl;

  /* MultiPortSwitch: '<S206>/Multiport Switch1' incorporates:
   *  Constant: '<S206>/Constant5'
   *  MultiPortSwitch: '<S210>/Multiport Switch1'
   *  RelationalOperator: '<S206>/Equal'
   */
  if (DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_RawFltEstCrvByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S206>/Multiport Switch1' */

  /* Sum: '<S210>/Sum' incorporates:
   *  MultiPortSwitch: '<S210>/Multiport Switch1'
   *  UnitDelay: '<S210>/Unit Delay'
   */
  rtb_Neg_i = LGC_RawErrCourseDotCdc_tmp - DTE_RawLmtEstCrvByBnkAgl;

  /* Product: '<S210>/Product' incorporates:
   *  Constant: '<S207>/Parameter2'
   *  Constant: '<S207>/Parameter3'
   *  MultiPortSwitch: '<S210>/Multiport Switch1'
   */
  rtb_Product_dv = DTE_ThdLmtReqCrv_P * DTE_TimeSysCycle_P;

  /* Switch: '<S211>/Switch2' incorporates:
   *  MultiPortSwitch: '<S210>/Multiport Switch1'
   *  RelationalOperator: '<S211>/LowerRelop1'
   */
  if (rtb_Neg_i > rtb_Product_dv) {
    rtb_Neg_i = rtb_Product_dv;
  } else {
    /* Product: '<S210>/Product1' incorporates:
     *  Constant: '<S207>/Parameter2'
     *  Constant: '<S207>/Parameter3'
     *  UnaryMinus: '<S207>/Unary Minus'
     */
    rtb_Product_dv = -DTE_ThdLmtReqCrv_P * DTE_TimeSysCycle_P;

    /* Switch: '<S211>/Switch' incorporates:
     *  RelationalOperator: '<S211>/UpperRelop'
     */
    if (rtb_Neg_i < rtb_Product_dv) {
      rtb_Neg_i = rtb_Product_dv;
    }

    /* End of Switch: '<S211>/Switch' */
  }

  /* End of Switch: '<S211>/Switch2' */

  /* MultiPortSwitch: '<S210>/Multiport Switch1' incorporates:
   *  Sum: '<S210>/Difference Inputs3'
   *  UnitDelay: '<S210>/Unit Delay'
   *
   * Block description for '<S210>/Difference Inputs3':
   *
   *  Add in CPU
   */
  DTE_RawLmtEstCrvByBnkAgl = rtb_Neg_i + DTE_RawLmtEstCrvByBnkAgl;

  /* MultiPortSwitch: '<S207>/Multiport Switch2' incorporates:
   *  RelationalOperator: '<S207>/Equal'
   */
  if (DTE_EnaResetByTgq) {
    /* MultiPortSwitch: '<S207>/Multiport Switch2' incorporates:
     *  Constant: '<S207>/Constant8'
     */
    DTE_LmtEstCrvByBnkAgl = 0.0F;
  } else {
    /* MultiPortSwitch: '<S207>/Multiport Switch2' */
    DTE_LmtEstCrvByBnkAgl = DTE_RawLmtEstCrvByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S207>/Multiport Switch2' */

  /* Product: '<S207>/Product1' incorporates:
   *  Constant: '<S207>/Parameter4'
   */
  DTE_RawReqCrvByBnkAgl = DTE_LmtEstCrvByBnkAgl * DTE_CoeffReqCrvGain_P;

  /* Switch: '<S214>/Switch2' incorporates:
   *  RelationalOperator: '<S214>/LowerRelop1'
   *  RelationalOperator: '<S214>/UpperRelop'
   *  Switch: '<S214>/Switch'
   *  UnaryMinus: '<S212>/Unary Minus'
   */
  if (DTE_RawReqCrvByBnkAgl > DTE_MaxCrvByBnkAgl) {
    /* Switch: '<S214>/Switch2' */
    DTE_LmtReqCrvByBnkAgl = DTE_MaxCrvByBnkAgl;
  } else if (DTE_RawReqCrvByBnkAgl < -DTE_MaxCrvByBnkAgl) {
    /* Switch: '<S214>/Switch' incorporates:
     *  Switch: '<S214>/Switch2'
     *  UnaryMinus: '<S212>/Unary Minus'
     */
    DTE_LmtReqCrvByBnkAgl = -DTE_MaxCrvByBnkAgl;
  } else {
    /* Switch: '<S214>/Switch2' incorporates:
     *  Switch: '<S214>/Switch'
     */
    DTE_LmtReqCrvByBnkAgl = DTE_RawReqCrvByBnkAgl;
  }

  /* End of Switch: '<S214>/Switch2' */

  /* MultiPortSwitch: '<S212>/Multiport Switch1' */
  if (!DTE_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S212>/Multiport Switch1' */
    DTE_HldReqCrvByBnkAgl = DTE_LmtReqCrvByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S212>/Multiport Switch1' */

  /* MultiPortSwitch: '<S212>/Multiport Switch' incorporates:
   *  Constant: '<S212>/Parameter'
   *  Logic: '<S212>/AND'
   */
  if (rtb_Equal3_c || (DTE_CswBnkAglCpmn_P == 0)) {
    /* MultiPortSwitch: '<S212>/Multiport Switch' incorporates:
     *  Constant: '<S212>/Constant'
     */
    DTE_ReqCrvByBnkAgl = 0.0F;
  } else {
    /* MultiPortSwitch: '<S212>/Multiport Switch' */
    DTE_ReqCrvByBnkAgl = DTE_HldReqCrvByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S212>/Multiport Switch' */

  /* Switch: '<S565>/Switch' incorporates:
   *  Constant: '<S565>/Constant'
   */
  if (P_TCTLGC_ActivateDynBacGain_nu != 0) {
    /* Signum: '<S565>/Sign3' */
    if (DTE_ReqCrvByBnkAgl < 0.0F) {
      LGC_RawErrCourseDotCdc_tmp = -1.0F;
    } else if (DTE_ReqCrvByBnkAgl > 0.0F) {
      LGC_RawErrCourseDotCdc_tmp = 1.0F;
    } else {
      LGC_RawErrCourseDotCdc_tmp = DTE_ReqCrvByBnkAgl;
    }

    /* End of Signum: '<S565>/Sign3' */

    /* Switch: '<S565>/Switch' incorporates:
     *  Lookup_n-D: '<S565>/Y_TCTLGC_DynBacGain_nu'
     *  Product: '<S565>/Mul'
     *  Product: '<S565>/Mul1'
     *  Product: '<S565>/Mul2'
     */
    LGC_CrvReqBAC_1pm = look1_iflf_binlxpw(rtb_Unit_Delay2_j *
      LGC_RawErrCourseDotCdc_tmp * rtb_Divide_hw, ((const real32_T *)
      &(X_TCTLGC_CtrlErrDistY_met[0])), ((const real32_T *)
      &(Y_TCTLGC_DynBacGain_nu[0])), 5U) * DTE_ReqCrvByBnkAgl;
  } else {
    /* Switch: '<S565>/Switch' */
    LGC_CrvReqBAC_1pm = DTE_ReqCrvByBnkAgl;
  }

  /* RelationalOperator: '<S198>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S198>/Delay Input1'
   *
   * Block description for '<S198>/Delay Input1':
   *
   *  Store in Global RAM
   */
  DEV_RstCrvGen = (DEV_EnaCrvGen > TJATCT_DW.DelayInput1_DSTATE);

  /* MultiPortSwitch: '<S195>/Multiport Switch' incorporates:
   *  Constant: '<S195>/Constant1'
   *  MultiPortSwitch: '<S195>/Multiport Switch1'
   *  RelationalOperator: '<S195>/Equal'
   *  RelationalOperator: '<S195>/Equal1'
   */
  if (DEV_RstCrvGen) {
    /* MultiPortSwitch: '<S195>/Multiport Switch' incorporates:
     *  Constant: '<S195>/Constant2'
     */
    DEV_TimeCrvGen = 0.0F;
  } else {
    if (DEV_EnaCrvGen == 1U) {
      /* MultiPortSwitch: '<S195>/Multiport Switch' incorporates:
       *  Constant: '<S195>/Parameter1'
       *  MultiPortSwitch: '<S195>/Multiport Switch1'
       *  Sum: '<S195>/Add'
       *  UnitDelay: '<S195>/Unit Delay'
       */
      DEV_TimeCrvGen = DEV_TimeCrvGen + DEV_TimeSysCycle_P;
    }
  }

  /* End of MultiPortSwitch: '<S195>/Multiport Switch' */

  /* MultiPortSwitch: '<S194>/Multiport Switch' incorporates:
   *  Constant: '<S194>/Parameter2'
   *  MultiPortSwitch: '<S194>/Multiport Switch1'
   *  RelationalOperator: '<S194>/Less Than'
   */
  if (DEV_EnaCrvGen == 0) {
    /* MultiPortSwitch: '<S194>/Multiport Switch' incorporates:
     *  Constant: '<S194>/Constant1'
     */
    DEV_CrvTestSignal = 0.0F;
  } else {
    if (DEV_TimeCrvGen >= DEV_TimeCrvFWait_P) {
      /* MultiPortSwitch: '<S194>/Multiport Switch1' incorporates:
       *  Constant: '<S194>/Parameter2'
       *  Sum: '<S194>/Sum1'
       */
      rtb_Neg_i = DEV_TimeCrvFWait_P + DEV_TimeCrvGen;
    } else {
      /* MultiPortSwitch: '<S194>/Multiport Switch1' incorporates:
       *  Constant: '<S194>/Constant3'
       */
      rtb_Neg_i = 0.0F;
    }

    /* MultiPortSwitch: '<S194>/Multiport Switch' incorporates:
     *  Constant: '<S194>/Constant'
     *  Constant: '<S194>/Parameter3'
     *  Constant: '<S194>/Parameter4'
     *  Constant: '<S194>/Parameter5'
     *  Product: '<S194>/Mul'
     *  Product: '<S194>/Mul1'
     *  Product: '<S194>/Mul2'
     *  Product: '<S194>/Mul3'
     *  Sum: '<S194>/Sum'
     *  Trigonometry: '<S194>/Sin'
     */
    DEV_CrvTestSignal = (real32_T)(sin((rtb_Neg_i * DEV_CoeffCrvFrqGain_P +
      DEV_StrtFrqCrvF_P) * 6.28 * rtb_Neg_i) * DEV_CoeffCrvAmp_P);
  }

  /* End of MultiPortSwitch: '<S194>/Multiport Switch' */

  /* Switch: '<S563>/Switch2' incorporates:
   *  Constant: '<S419>/Constant'
   *  Constant: '<S563>/Constant2'
   *  S-Function (sfix_bitop): '<S563>/Bitwise AND'
   */
  if ((P_TCTLGC_CssCrv_nu & 1U) != 0U) {
    LGC_RawErrCourseDotCdc_tmp = LGC_CtrlCrv_1pm;
  } else {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  }

  /* End of Switch: '<S563>/Switch2' */

  /* Switch: '<S563>/Switch5' incorporates:
   *  Constant: '<S419>/Constant'
   *  Constant: '<S563>/Constant4'
   *  S-Function (sfix_bitop): '<S563>/Bitwise AND1'
   */
  if ((P_TCTLGC_CssCrv_nu & 4U) != 0U) {
    rtb_Divide_l_tmp = DTE_ReqCrvByDstrb;
  } else {
    rtb_Divide_l_tmp = 0.0F;
  }

  /* End of Switch: '<S563>/Switch5' */

  /* Switch: '<S563>/Switch1' incorporates:
   *  Constant: '<S419>/Constant'
   *  Constant: '<S563>/Constant1'
   *  S-Function (sfix_bitop): '<S563>/Bitwise AND2'
   */
  if ((P_TCTLGC_CssCrv_nu & 8U) != 0U) {
    rtb_Product_dv = LGC_CrvReqBAC_1pm;
  } else {
    rtb_Product_dv = 0.0F;
  }

  /* End of Switch: '<S563>/Switch1' */

  /* Switch: '<S563>/Switch3' incorporates:
   *  Constant: '<S419>/Constant'
   *  Constant: '<S563>/Constant3'
   *  S-Function (sfix_bitop): '<S563>/Bitwise AND3'
   */
  if ((P_TCTLGC_CssCrv_nu & 16U) != 0U) {
    rtb_Neg_i = DEV_CrvTestSignal;
  } else {
    rtb_Neg_i = 0.0F;
  }

  /* End of Switch: '<S563>/Switch3' */

  /* Sum: '<S563>/Sum2' incorporates:
   *  Sum: '<S563>/Sum1'
   *  Sum: '<S563>/Sum3'
   */
  LGC_SumCrvReqFb_1pm = ((LGC_RawErrCourseDotCdc_tmp + rtb_Divide_l_tmp) +
    rtb_Product_dv) + rtb_Neg_i;

  /* Switch: '<S580>/Switch2' */
  if (!LGC_EnaFreezeByTgq) {
    /* Product: '<S572>/Product' incorporates:
     *  Constant: '<S419>/Parameter3'
     *  Inport: '<Root>/TCTI_LmtReqTrajCrvGrd'
     */
    rtb_Neg_i = TCTI_LmtReqTrajCrvGrd * LGC_TimeSysCycle_P;

    /* Switch: '<S572>/Switch' incorporates:
     *  Inport: '<Root>/TCTI_EnaReplanCurValues'
     */
    if (TCTI_EnaReplanCurValues) {
      /* Switch: '<S572>/Switch' */
      TJATCT_DW.UnitDelay_DSTATE_h2 = LGC_SumCrvReqFb_1pm;
    }

    /* End of Switch: '<S572>/Switch' */

    /* Sum: '<S572>/Add' */
    rtb_Product_dv = LGC_SumCrvReqFb_1pm - TJATCT_DW.UnitDelay_DSTATE_h2;

    /* Switch: '<S581>/Switch2' incorporates:
     *  Product: '<S572>/Gain'
     *  RelationalOperator: '<S581>/LowerRelop1'
     *  RelationalOperator: '<S581>/UpperRelop'
     *  Switch: '<S581>/Switch'
     */
    if (rtb_Product_dv > rtb_Neg_i) {
      rtb_Product_dv = rtb_Neg_i;
    } else {
      if (rtb_Product_dv < -rtb_Neg_i) {
        /* Switch: '<S581>/Switch' incorporates:
         *  Product: '<S572>/Gain'
         */
        rtb_Product_dv = -rtb_Neg_i;
      }
    }

    /* End of Switch: '<S581>/Switch2' */

    /* Switch: '<S580>/Switch2' incorporates:
     *  Sum: '<S572>/Add1'
     */
    LGC_SumCrvReqFbGrdLim_1pm = rtb_Product_dv + TJATCT_DW.UnitDelay_DSTATE_h2;
  }

  /* End of Switch: '<S580>/Switch2' */

  /* Product: '<S573>/Gain1' incorporates:
   *  Inport: '<Root>/TCTI_MaxTrajCrv'
   */
  rtb_Neg_i = -TCTI_MaxTrajCrv;

  /* Switch: '<S582>/Switch' incorporates:
   *  Inport: '<Root>/TCTI_MaxTrajCrv'
   *  Product: '<S573>/Gain1'
   *  RelationalOperator: '<S582>/GreaterThanOrEqual'
   */
  if (LGC_SumCrvReqFbGrdLim_1pm >= -TCTI_MaxTrajCrv) {
    rtb_Neg_i = LGC_SumCrvReqFbGrdLim_1pm;
  }

  /* End of Switch: '<S582>/Switch' */

  /* Switch: '<S582>/Switch1' incorporates:
   *  Inport: '<Root>/TCTI_MaxTrajCrv'
   *  RelationalOperator: '<S582>/LessThanOrEqual'
   */
  if (rtb_Neg_i <= TCTI_MaxTrajCrv) {
    /* Switch: '<S582>/Switch1' */
    LGC_SumCrvReqFbSatLim_1pm = rtb_Neg_i;
  } else {
    /* Switch: '<S582>/Switch1' */
    LGC_SumCrvReqFbSatLim_1pm = TCTI_MaxTrajCrv;
  }

  /* End of Switch: '<S582>/Switch1' */

  /* Switch: '<S569>/Switch2' */
  if (!LGC_EnaFreezeByTgq) {
    /* Switch: '<S569>/Switch2' */
    LGC_SumCrvReqFbFrz_1pm = LGC_SumCrvReqFbSatLim_1pm;
  }

  /* End of Switch: '<S569>/Switch2' */

  /* Switch: '<S419>/Switch' incorporates:
   *  Constant: '<S419>/Constant2'
   *  Switch: '<S566>/Switch4'
   */
  if (P_TCTLGC_ActivateOverride_nu != 0) {
    /* Switch: '<S419>/Switch' */
    LGC_CtrlCrv_DE_1pm = LGC_SumCrvReqFb_1pm;
  } else if (LGC_EnaCntrlByTgq) {
    /* Switch: '<S566>/Switch4' incorporates:
     *  Switch: '<S419>/Switch'
     */
    LGC_CtrlCrv_DE_1pm = LGC_SumCrvReqFbFrz_1pm;
  } else {
    /* Switch: '<S419>/Switch' incorporates:
     *  Constant: '<S566>/Constant6'
     *  Switch: '<S566>/Switch4'
     */
    LGC_CtrlCrv_DE_1pm = 0.0F;
  }

  /* End of Switch: '<S419>/Switch' */

  /* Lookup_n-D: '<S120>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  CLM_ThdFbcDcGrd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(CLM_ThdFfcFbcDcGrd_X[0])), ((const real32_T *)&(CLM_ThdFfcFbcDcGrd_M[0])),
    14U);

  /* Product: '<S120>/Product' incorporates:
   *  Constant: '<S120>/Parameter1'
   *  MultiPortSwitch: '<S122>/Multiport Switch1'
   *  Product: '<S122>/Product'
   */
  rtb_Neg_i = CLM_ThdFbcDcGrd * CLM_TimeSysCycle_P;

  /* Logic: '<S120>/AND1' incorporates:
   *  Abs: '<S120>/Abs'
   *  Product: '<S120>/Product'
   *  RelationalOperator: '<S120>/GreaterThan'
   *  Sum: '<S120>/Subtract'
   *  UnitDelay: '<S120>/Unit Delay'
   */
  CLM_EnaGrdFbcDc = ((fabsf(LGC_CtrlCrv_DE_1pm - TJATCT_DW.UnitDelay_DSTATE_dt) >
                      rtb_Neg_i) && rtb_GreaterThan1_bj);

  /* Sum: '<S122>/Sum' incorporates:
   *  MultiPortSwitch: '<S122>/Multiport Switch1'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  rtb_Product_dv = LGC_CtrlCrv_DE_1pm - CLM_RawGrdFbcDc;

  /* Switch: '<S123>/Switch2' incorporates:
   *  MultiPortSwitch: '<S122>/Multiport Switch1'
   *  RelationalOperator: '<S123>/LowerRelop1'
   */
  if (rtb_Product_dv > rtb_Neg_i) {
    rtb_Product_dv = rtb_Neg_i;
  } else {
    /* Product: '<S122>/Product1' incorporates:
     *  Constant: '<S120>/Parameter'
     *  UnaryMinus: '<S120>/Unary Minus'
     */
    rtb_Neg_i = -CLM_ThdFbcDcGrd * CLM_TimeSysCycle_P;

    /* Switch: '<S123>/Switch' incorporates:
     *  RelationalOperator: '<S123>/UpperRelop'
     */
    if (rtb_Product_dv < rtb_Neg_i) {
      rtb_Product_dv = rtb_Neg_i;
    }

    /* End of Switch: '<S123>/Switch' */
  }

  /* End of Switch: '<S123>/Switch2' */

  /* MultiPortSwitch: '<S122>/Multiport Switch1' incorporates:
   *  Sum: '<S122>/Difference Inputs3'
   *  UnitDelay: '<S122>/Unit Delay'
   *
   * Block description for '<S122>/Difference Inputs3':
   *
   *  Add in CPU
   */
  CLM_RawGrdFbcDc = rtb_Product_dv + CLM_RawGrdFbcDc;

  /* MultiPortSwitch: '<S120>/Multiport Switch' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   */
  if (TCTI_EnaReplanCurValues) {
    /* MultiPortSwitch: '<S120>/Multiport Switch' */
    CLM_GrdFbcDc = LGC_CtrlCrv_DE_1pm;
  } else {
    /* MultiPortSwitch: '<S120>/Multiport Switch' */
    CLM_GrdFbcDc = CLM_RawGrdFbcDc;
  }

  /* End of MultiPortSwitch: '<S120>/Multiport Switch' */

  /* Lookup_n-D: '<S121>/1-D Lookup Table2' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  CLM_ThdFbcDcSat = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(CLM_ThdFbcDcSat_X[0])), ((const real32_T *)&(CLM_ThdFbcDcSat_M[0])), 14U);

  /* RelationalOperator: '<S121>/GreaterThan' incorporates:
   *  Abs: '<S121>/Abs'
   */
  CLM_EnaSatFbcDc = (fabsf(CLM_GrdFbcDc) > CLM_ThdFbcDcSat);

  /* UnitDelay: '<S102>/Unit Delay' */
  CLM_EnaUnplauRequest = TC_EnaUnplauUnitDelay_bool;

  /* Sum: '<S99>/Add' incorporates:
   *  ArithShift: '<S99>/Shift Arithmetic1'
   *  ArithShift: '<S99>/Shift Arithmetic2'
   *  ArithShift: '<S99>/Shift Arithmetic3'
   *  ArithShift: '<S99>/Shift Arithmetic4'
   *  ArithShift: '<S99>/Shift Arithmetic5'
   *  ArithShift: '<S99>/Shift Arithmetic6'
   *  Constant: '<S99>/Constant2'
   *  Constant: '<S99>/Constant3'
   *  Constant: '<S99>/Constant4'
   *  Constant: '<S99>/Constant5'
   *  Constant: '<S99>/Constant6'
   *  Constant: '<S99>/Constant7'
   *  DataTypeConversion: '<S99>/Data Type Conversion'
   *  DataTypeConversion: '<S99>/Data Type Conversion1'
   *  DataTypeConversion: '<S99>/Data Type Conversion2'
   *  DataTypeConversion: '<S99>/Data Type Conversion3'
   *  DataTypeConversion: '<S99>/Data Type Conversion4'
   *  DataTypeConversion: '<S99>/Data Type Conversion5'
   *  DataTypeConversion: '<S99>/Data Type Conversion6'
   */
  TJATCT_Y.CLM_BtfQualifier = (uint8_T)(((((((uint32_T)(CLM_EnaSatDeltaFCmd <<
    1U) + CLM_EnaGrdDeltaFCmd) + (CLM_EnaGrdFfcCrv << 2U)) + (CLM_EnaSatFfcCrv <<
    3U)) + (CLM_EnaGrdFbcDc << 4U)) + (CLM_EnaSatFbcDc << 5U)) +
    (CLM_EnaUnplauRequest << 6U));

  /* MultiPortSwitch: '<S106>/Multiport Switch' incorporates:
   *  Constant: '<S106>/Parameter2'
   */
  if (CLM_CswForceDegrReq_P == 0) {
    /* MultiPortSwitch: '<S106>/Multiport Switch' incorporates:
     *  Constant: '<S106>/Parameter'
     *  Constant: '<S106>/Parameter1'
     *  Logic: '<S106>/OR2'
     *  RelationalOperator: '<S106>/GreaterThan'
     *  RelationalOperator: '<S106>/GreaterThan1'
     *  S-Function (sfix_bitop): '<S106>/Bitwise Operator'
     *  S-Function (sfix_bitop): '<S106>/Bitwise Operator1'
     */
    CLM_EnaDegrReq = (((CDC_BtfQualifier & CLM_BtfQualifierCdc_P) > 0) ||
                      ((TJATCT_Y.CLM_BtfQualifier & CLM_BtfQualifierClm_P) > 0));
  } else {
    /* MultiPortSwitch: '<S106>/Multiport Switch' incorporates:
     *  Constant: '<S106>/Constant2'
     */
    CLM_EnaDegrReq = true;
  }

  /* End of MultiPortSwitch: '<S106>/Multiport Switch' */

  /* RelationalOperator: '<S106>/Equal' incorporates:
   *  Constant: '<S106>/Parameter3'
   *  Inport: '<Root>/TCTI_StLcfSys'
   */
  rtb_LowerRelop1_p0 = (TCTI_StLcfSys != CLM_StControlCsc_P);

  /* Logic: '<S106>/AND' incorporates:
   *  Logic: '<S106>/NOT2'
   */
  rtb_AND_od = (CLM_EnaDegrReq && (!rtb_LowerRelop1_p0));

  /* MultiPortSwitch: '<S109>/Multiport Switch' incorporates:
   *  Constant: '<S106>/Parameter9'
   *  Constant: '<S109>/IAM_Ts_P2'
   *  Logic: '<S109>/Logical Operator2'
   *  Product: '<S109>/Product'
   *  Sum: '<S109>/Add'
   *  UnitDelay: '<S109>/Unit Delay'
   */
  if (rtb_AND_od) {
    TJATCT_DW.UnitDelay_DSTATE_on = 0.0F;
  } else {
    TJATCT_DW.UnitDelay_DSTATE_on += CLM_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S109>/Multiport Switch' */

  /* Logic: '<S109>/Logical Operator1' incorporates:
   *  Logic: '<S108>/Logical Operator1'
   */
  tmp_2 = !rtb_AND_od;

  /* Switch: '<S111>/Switch' incorporates:
   *  Constant: '<S106>/Parameter8'
   *  Constant: '<S111>/Constant2'
   *  Logic: '<S109>/Logical Operator'
   *  Logic: '<S109>/Logical Operator1'
   *  RelationalOperator: '<S109>/GreaterThan'
   *  Switch: '<S111>/Switch1'
   *  UnitDelay: '<S109>/Unit Delay'
   *  UnitDelay: '<S111>/Unit Delay'
   */
  if (tmp_2 && (TJATCT_DW.UnitDelay_DSTATE_on >= CLM_MaxHldTiDegrReq_P)) {
    TC_Freeze1RSFlipFlop_bool = false;
  } else {
    TC_Freeze1RSFlipFlop_bool = (rtb_AND_od || TC_Freeze1RSFlipFlop_bool);
  }

  /* End of Switch: '<S111>/Switch' */

  /* Logic: '<S106>/OR' incorporates:
   *  Logic: '<S106>/NOT1'
   *  UnitDelay: '<S111>/Unit Delay'
   */
  CLM_EnaRstDegrReq = ((!TC_Freeze1RSFlipFlop_bool) || rtb_LowerRelop1_p0);

  /* MultiPortSwitch: '<S108>/Multiport Switch' incorporates:
   *  Constant: '<S106>/Parameter7'
   *  Constant: '<S108>/IAM_Ts_P2'
   *  Logic: '<S108>/Logical Operator2'
   *  Product: '<S108>/Product'
   *  Sum: '<S108>/Add'
   *  UnitDelay: '<S108>/Unit Delay'
   */
  if (rtb_AND_od) {
    TJATCT_DW.UnitDelay_DSTATE_a3 = 0.0F;
  } else {
    TJATCT_DW.UnitDelay_DSTATE_a3 += CLM_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S108>/Multiport Switch' */

  /* Switch: '<S110>/Switch' incorporates:
   *  Constant: '<S106>/Parameter4'
   *  Logic: '<S108>/Logical Operator'
   *  RelationalOperator: '<S108>/GreaterThan'
   *  UnitDelay: '<S108>/Unit Delay'
   */
  if (tmp_2 && (TJATCT_DW.UnitDelay_DSTATE_a3 >= CLM_MinHldTiDegrReq_P)) {
    /* Switch: '<S110>/Switch' incorporates:
     *  Constant: '<S110>/Constant2'
     */
    CLM_EnaSetDegrReq = false;
  } else {
    /* Switch: '<S110>/Switch' incorporates:
     *  Switch: '<S110>/Switch1'
     *  UnitDelay: '<S110>/Unit Delay'
     */
    CLM_EnaSetDegrReq = (rtb_AND_od || TC_FreezeRSFlipFlop_bool);
  }

  /* End of Switch: '<S110>/Switch' */

  /* Switch: '<S107>/Switch' */
  if (CLM_EnaRstDegrReq) {
    /* Switch: '<S107>/Switch' incorporates:
     *  Constant: '<S107>/Constant2'
     */
    CLM_RawEnaDegrReq = false;
  } else {
    /* Switch: '<S107>/Switch' incorporates:
     *  Switch: '<S107>/Switch1'
     *  UnitDelay: '<S107>/Unit Delay'
     */
    CLM_RawEnaDegrReq = (CLM_EnaSetDegrReq || TC_Freeze2RSFlipFlop_bool);
  }

  /* End of Switch: '<S107>/Switch' */

  /* MultiPortSwitch: '<S106>/Multiport Switch1' incorporates:
   *  Constant: '<S106>/Parameter6'
   */
  if (CLM_CswHldDegrReq_P != 0) {
    /* MultiPortSwitch: '<S106>/Multiport Switch' incorporates:
     *  MultiPortSwitch: '<S106>/Multiport Switch1'
     */
    CLM_EnaDegrReq = CLM_RawEnaDegrReq;
  }

  /* End of MultiPortSwitch: '<S106>/Multiport Switch1' */

  /* Sum: '<S105>/Add' incorporates:
   *  ArithShift: '<S105>/Shift Arithmetic1'
   *  ArithShift: '<S105>/Shift Arithmetic2'
   *  ArithShift: '<S105>/Shift Arithmetic3'
   *  ArithShift: '<S105>/Shift Arithmetic4'
   *  Constant: '<S105>/Constant2'
   *  Constant: '<S105>/Constant3'
   *  Constant: '<S105>/Constant4'
   *  Constant: '<S105>/Constant5'
   *  DataTypeConversion: '<S105>/Data Type Conversion'
   *  Sum: '<S298>/Add4'
   */
  CLM_BtfQulifierTrajCtrl = (uint8_T)(((uint32_T)((int32_T)((uint32_T)
    TJATCT_Y.S_TCTEST_QualifierService_nu >> 12U) << 4U) + CLM_EnaDegrReq) +
    ((int32_T)((uint32_T)TJATCT_Y.S_TCTEST_QualifierService_nu >> 13U) << 5U));

  /* Switch: '<S129>/Switch2' incorporates:
   *  RelationalOperator: '<S129>/LowerRelop1'
   *  RelationalOperator: '<S129>/UpperRelop'
   *  Switch: '<S129>/Switch'
   *  UnaryMinus: '<S126>/Unary Minus'
   */
  if (CLM_GrdFfcCrv > CLM_ThdFfcCrvSat) {
    /* Switch: '<S129>/Switch2' */
    CLM_SatFfcCrv = CLM_ThdFfcCrvSat;
  } else if (CLM_GrdFfcCrv < -CLM_ThdFfcCrvSat) {
    /* Switch: '<S129>/Switch' incorporates:
     *  Switch: '<S129>/Switch2'
     *  UnaryMinus: '<S126>/Unary Minus'
     */
    CLM_SatFfcCrv = -CLM_ThdFfcCrvSat;
  } else {
    /* Switch: '<S129>/Switch2' incorporates:
     *  Switch: '<S129>/Switch'
     */
    CLM_SatFfcCrv = CLM_GrdFfcCrv;
  }

  /* End of Switch: '<S129>/Switch2' */

  /* Switch: '<S124>/Switch2' incorporates:
   *  RelationalOperator: '<S124>/LowerRelop1'
   *  RelationalOperator: '<S124>/UpperRelop'
   *  Switch: '<S124>/Switch'
   *  UnaryMinus: '<S121>/Unary Minus'
   */
  if (CLM_GrdFbcDc > CLM_ThdFbcDcSat) {
    /* Switch: '<S124>/Switch2' */
    CLM_SatFbcDc = CLM_ThdFbcDcSat;
  } else if (CLM_GrdFbcDc < -CLM_ThdFbcDcSat) {
    /* Switch: '<S124>/Switch' incorporates:
     *  Switch: '<S124>/Switch2'
     *  UnaryMinus: '<S121>/Unary Minus'
     */
    CLM_SatFbcDc = -CLM_ThdFbcDcSat;
  } else {
    /* Switch: '<S124>/Switch2' incorporates:
     *  Switch: '<S124>/Switch'
     */
    CLM_SatFbcDc = CLM_GrdFbcDc;
  }

  /* End of Switch: '<S124>/Switch2' */

  /* MultiPortSwitch: '<S103>/Multiport Switch1' incorporates:
   *  MultiPortSwitch: '<S103>/Multiport Switch2'
   */
  if (!CLM_EnaDegrReq) {
    /* MultiPortSwitch: '<S103>/Multiport Switch1' */
    CLM_ReqFfcCrv = CLM_SatFfcCrv;

    /* MultiPortSwitch: '<S103>/Multiport Switch2' */
    CLM_ReqFbcDcCrv = CLM_SatFbcDc;
  }

  /* End of MultiPortSwitch: '<S103>/Multiport Switch1' */

  /* Sum: '<S103>/Add' */
  CLM_LmtReqFfcCrv = CLM_ReqFfcCrv + CLM_ReqFbcDcCrv;

  /* Product: '<S134>/Divide1' incorporates:
   *  Constant: '<S134>/Constant5'
   *  Constant: '<S134>/Constant6'
   *  Constant: '<S134>/Parameter2'
   *  Constant: '<S134>/Parameter3'
   *  Constant: '<S134>/Parameter4'
   *  Product: '<S134>/Divide'
   *  Product: '<S134>/Product'
   *  Product: '<S134>/Product2'
   *  Sum: '<S134>/Add'
   *  Sum: '<S134>/Add2'
   */
  CLM_CrvBySteerAngle = (real32_T)(LQR_ReqDeltaF_Limit_deg * 3.14159 / 180.0) /
    ((rtb_Divide_bo * CLM_SelfSteerGrd_P + CLM_DistCogToFrontAxle_P) +
     CLM_DistCogToRearAxle_P);

  /* Switch: '<S119>/Switch2' incorporates:
   *  RelationalOperator: '<S119>/LowerRelop1'
   *  RelationalOperator: '<S119>/UpperRelop'
   *  Switch: '<S119>/Switch'
   *  UnaryMinus: '<S116>/Unary Minus'
   */
  if (CLM_GrdDeltaFCmd > CLM_ThdDeltaFCmdSat) {
    /* Switch: '<S119>/Switch2' */
    CLM_SatDeltaFCmd = CLM_ThdDeltaFCmdSat;
  } else if (CLM_GrdDeltaFCmd < -CLM_ThdDeltaFCmdSat) {
    /* Switch: '<S119>/Switch' incorporates:
     *  Switch: '<S119>/Switch2'
     *  UnaryMinus: '<S116>/Unary Minus'
     */
    CLM_SatDeltaFCmd = -CLM_ThdDeltaFCmdSat;
  } else {
    /* Switch: '<S119>/Switch2' incorporates:
     *  Switch: '<S119>/Switch'
     */
    CLM_SatDeltaFCmd = CLM_GrdDeltaFCmd;
  }

  /* End of Switch: '<S119>/Switch2' */

  /* MultiPortSwitch: '<S103>/Multiport Switch' */
  if (!CLM_EnaDegrReq) {
    /* MultiPortSwitch: '<S103>/Multiport Switch' */
    CLM_DeltaFCmd = CLM_SatDeltaFCmd;
  }

  /* End of MultiPortSwitch: '<S103>/Multiport Switch' */

  /* MultiPortSwitch: '<S134>/Multiport Switch' incorporates:
   *  Constant: '<S134>/Constant1'
   *  Constant: '<S134>/Parameter'
   *  MultiPortSwitch: '<S134>/Multiport Switch1'
   */
  if (CLM_CswSelFbcCrv_P == 0) {
    LGC_RawErrCourseDotCdc_tmp = CLM_CrvBySteerAngle;
    rtb_Divide_l_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = CLM_DeltaFCmd;
    rtb_Divide_l_tmp = CLM_ReqFfcCrv;
  }

  /* End of MultiPortSwitch: '<S134>/Multiport Switch' */

  /* Sum: '<S134>/Add1' */
  CLM_SumCtrlCrv = LGC_RawErrCourseDotCdc_tmp + rtb_Divide_l_tmp;

  /* Logic: '<S132>/AND1' incorporates:
   *  Constant: '<S132>/Parameter'
   *  Constant: '<S132>/Parameter1'
   *  Inport: '<Root>/TCTI_EnaLmtActCsc'
   *  Inport: '<Root>/TCTI_StLcfSys'
   *  Inport: '<Root>/TCTI_StVehOdo'
   *  Logic: '<S132>/AND'
   *  Logic: '<S132>/OR1'
   *  RelationalOperator: '<S132>/Equal'
   *  RelationalOperator: '<S132>/Equal1'
   */
  CLM_EnaPlausibilityCheck = ((TCTI_StVehOdo != 0) && (TCTI_EnaLmtActCsc != 0) &&
    ((TCTI_StLcfSys == CLM_StRequestCsc_P) || (TCTI_StLcfSys ==
    CLM_StControlCsc_P)));

  /* MultiPortSwitch: '<S131>/Multiport Switch' incorporates:
   *  Logic: '<S131>/NOT1'
   *  Logic: '<S131>/OR'
   */
  if (CLM_EnaPlausibilityCheck && rtb_GreaterThan1_bj) {
    /* MultiPortSwitch: '<S131>/Multiport Switch' incorporates:
     *  Inport: '<Root>/TCTI_TimeLmtDur'
     */
    CLM_TimerPlauCheck = TCTI_TimeLmtDur;
  } else {
    /* MultiPortSwitch: '<S131>/Multiport Switch' incorporates:
     *  Constant: '<S131>/Parameter'
     *  Sum: '<S131>/Subtract'
     */
    CLM_TimerPlauCheck -= CLM_TimeSysCycle_P;
  }

  /* End of MultiPortSwitch: '<S131>/Multiport Switch' */

  /* RelationalOperator: '<S131>/GreaterThan1' incorporates:
   *  Constant: '<S131>/Parameter1'
   */
  rtb_GreaterThan1_bj = (CLM_TimerPlauCheck >= CLM_TimeSysCycle_P);

  /* RelationalOperator: '<S135>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S135>/Delay Input1'
   *
   * Block description for '<S135>/Delay Input1':
   *
   *  Store in Global RAM
   */
  CLM_EnaHldVehCrv = ((int32_T)rtb_GreaterThan1_bj < (int32_T)
                      TJATCT_DW.DelayInput1_DSTATE_l);

  /* MultiPortSwitch: '<S131>/Multiport Switch1' */
  if (!CLM_EnaHldVehCrv) {
    /* MultiPortSwitch: '<S131>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehCrv'
     */
    CLM_HldVehCrv = TCTI_VehCurvature;
  }

  /* End of MultiPortSwitch: '<S131>/Multiport Switch1' */

  /* Sum: '<S136>/Add2' */
  rtb_Neg_i = DTE_ReqCrvByBnkAgl + DTE_ReqCrvByDstrb;

  /* MultiPortSwitch: '<S136>/Multiport Switch' incorporates:
   *  Constant: '<S136>/Parameter'
   *  Inport: '<Root>/TCTI_StLaneLaKmc'
   *  RelationalOperator: '<S136>/Equal'
   */
  if (TCTI_StLaneLaKmc != CLM_StDepartLeftLaKmc_P) {
    /* MultiPortSwitch: '<S136>/Multiport Switch' incorporates:
     *  Constant: '<S136>/Constant1'
     */
    CLM_ThdCrvPlauChkUp = 1.0F;
  } else {
    /* MultiPortSwitch: '<S136>/Multiport Switch' incorporates:
     *  Sum: '<S136>/Add'
     */
    CLM_ThdCrvPlauChkUp = CLM_HldVehCrv + rtb_Neg_i;
  }

  /* End of MultiPortSwitch: '<S136>/Multiport Switch' */

  /* MultiPortSwitch: '<S136>/Multiport Switch1' incorporates:
   *  Constant: '<S136>/Parameter1'
   *  Inport: '<Root>/TCTI_StLaneLaKmc'
   *  RelationalOperator: '<S136>/Equal1'
   */
  if (TCTI_StLaneLaKmc != CLM_StDepartRightLaKmc_P) {
    /* MultiPortSwitch: '<S136>/Multiport Switch1' incorporates:
     *  Constant: '<S136>/Constant2'
     */
    CLM_ThdCrvPlauChkLow = -1.0F;
  } else {
    /* MultiPortSwitch: '<S136>/Multiport Switch1' incorporates:
     *  Sum: '<S136>/Add1'
     */
    CLM_ThdCrvPlauChkLow = CLM_HldVehCrv + rtb_Neg_i;
  }

  /* End of MultiPortSwitch: '<S136>/Multiport Switch1' */

  /* Logic: '<S137>/AND' incorporates:
   *  Logic: '<S137>/OR1'
   *  RelationalOperator: '<S137>/GreaterThan'
   *  RelationalOperator: '<S137>/GreaterThan1'
   */
  CLM_EnaLmtWarn = (((CLM_SumCtrlCrv > CLM_ThdCrvPlauChkUp) || (CLM_SumCtrlCrv <
    CLM_ThdCrvPlauChkLow)) && CLM_EnaHldVehCrv);

  /* Switch: '<S130>/Switch' incorporates:
   *  Constant: '<S130>/Constant2'
   *  Logic: '<S102>/NOT'
   *  Switch: '<S130>/Switch1'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  if (!CLM_EnaPlausibilityCheck) {
    TC_HoldWarnRSFlipFlop_bool = false;
  } else {
    TC_HoldWarnRSFlipFlop_bool = (CLM_EnaLmtWarn || TC_HoldWarnRSFlipFlop_bool);
  }

  /* End of Switch: '<S130>/Switch' */

  /* Logic: '<S131>/AND' incorporates:
   *  Constant: '<S131>/Constant1'
   *  RelationalOperator: '<S131>/GreaterThan'
   */
  CLM_EnaPlauCheck = (CLM_EnaPlausibilityCheck && (CLM_TimerPlauCheck > 0.0F));

  /* Switch: '<S567>/Switch4' */
  if (LGC_EnaCntrlByTgq) {
    /* Switch: '<S567>/Switch4' incorporates:
     *  Sum: '<S419>/Subtract2'
     */
    LGC_TgtCrv_DE_1pm = LGC_SumCrvReqFbFrz_1pm + rtb_Switch7;
  } else {
    /* Switch: '<S567>/Switch4' incorporates:
     *  Constant: '<S567>/Constant6'
     */
    LGC_TgtCrv_DE_1pm = 0.0F;
  }

  /* End of Switch: '<S567>/Switch4' */

  /* Product: '<S205>/Product' incorporates:
   *  Product: '<S272>/Product5'
   *  Product: '<S273>/Product6'
   *  Product: '<S275>/Product5'
   */
  DTE_RawReqDeltaByBnkAgl_tmp = DTE_LmtVehVelX * DTE_LmtVehVelX;

  /* Sum: '<S205>/Add1' incorporates:
   *  Constant: '<S205>/Parameter1'
   *  Constant: '<S205>/Parameter2'
   *  Sum: '<S273>/Add1'
   *  Sum: '<S274>/Add'
   */
  DTE_RawReqDeltaByBnkAgl_tmp_0 = DTE_DistCogToFrontAxle_P +
    DTE_DistCogToRearAxle_P;

  /* Product: '<S205>/Product3' incorporates:
   *  Constant: '<S205>/Parameter'
   *  Constant: '<S205>/Parameter3'
   *  Product: '<S205>/Product'
   *  Product: '<S205>/Product1'
   *  Product: '<S205>/Product2'
   *  Sum: '<S205>/Add'
   *  Sum: '<S205>/Add1'
   */
  DTE_RawReqDeltaByBnkAgl = (DTE_RawReqDeltaByBnkAgl_tmp * DTE_SelfSteerGrdnt_P
    + DTE_RawReqDeltaByBnkAgl_tmp_0) * DTE_LmtEstCrvByBnkAgl *
    DTE_CoeffReqDeltaGain_P;

  /* Lookup_n-D: '<S213>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_MaxDeltaByBnkAgl = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(DTE_MaxDeltaByBnkAgl_X[0])), ((const real32_T *)&(DTE_MaxDeltaByBnkAgl_M[0])),
    6U);

  /* Switch: '<S215>/Switch2' incorporates:
   *  RelationalOperator: '<S215>/LowerRelop1'
   *  RelationalOperator: '<S215>/UpperRelop'
   *  Switch: '<S215>/Switch'
   *  UnaryMinus: '<S213>/Unary Minus'
   */
  if (DTE_RawReqDeltaByBnkAgl > DTE_MaxDeltaByBnkAgl) {
    /* Switch: '<S215>/Switch2' */
    DTE_LmtReqDeltaByBnkAgl = DTE_MaxDeltaByBnkAgl;
  } else if (DTE_RawReqDeltaByBnkAgl < -DTE_MaxDeltaByBnkAgl) {
    /* Switch: '<S215>/Switch' incorporates:
     *  Switch: '<S215>/Switch2'
     *  UnaryMinus: '<S213>/Unary Minus'
     */
    DTE_LmtReqDeltaByBnkAgl = -DTE_MaxDeltaByBnkAgl;
  } else {
    /* Switch: '<S215>/Switch2' incorporates:
     *  Switch: '<S215>/Switch'
     */
    DTE_LmtReqDeltaByBnkAgl = DTE_RawReqDeltaByBnkAgl;
  }

  /* End of Switch: '<S215>/Switch2' */

  /* MultiPortSwitch: '<S213>/Multiport Switch1' */
  if (!DTE_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S213>/Multiport Switch1' */
    DTE_HldReqDeltaByBnkAgl = DTE_LmtReqDeltaByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S213>/Multiport Switch1' */

  /* MultiPortSwitch: '<S213>/Multiport Switch' incorporates:
   *  Constant: '<S213>/Parameter'
   *  Logic: '<S213>/AND'
   */
  if (rtb_Equal3_c || (DTE_CswBnkAglCpmn_P == 0)) {
    /* MultiPortSwitch: '<S213>/Multiport Switch' incorporates:
     *  Constant: '<S213>/Constant'
     */
    DTE_ReqDeltaByBnkAgl = 0.0F;
  } else {
    /* MultiPortSwitch: '<S213>/Multiport Switch' */
    DTE_ReqDeltaByBnkAgl = DTE_HldReqDeltaByBnkAgl;
  }

  /* End of MultiPortSwitch: '<S213>/Multiport Switch' */

  /* Switch: '<S583>/Switch' incorporates:
   *  Constant: '<S583>/Constant'
   *  Lookup_n-D: '<S583>/Y_TCTLGC_DynBacGain_nu'
   *  Product: '<S583>/Mul'
   *  Product: '<S583>/Mul1'
   *  Product: '<S583>/Mul2'
   */
  if (P_TCTLGC_ActivateDynBacGain_nu != 0) {
    /* Signum: '<S583>/Sign3' */
    if (DTE_ReqDeltaByBnkAgl < 0.0F) {
      LGC_RawErrCourseDotCdc_tmp = -1.0F;
    } else if (DTE_ReqDeltaByBnkAgl > 0.0F) {
      LGC_RawErrCourseDotCdc_tmp = 1.0F;
    } else {
      LGC_RawErrCourseDotCdc_tmp = DTE_ReqDeltaByBnkAgl;
    }

    /* End of Signum: '<S583>/Sign3' */
    rtb_Neg_i = look1_iflf_binlxpw(rtb_Unit_Delay2_j *
      LGC_RawErrCourseDotCdc_tmp * rtb_Divide_hw, ((const real32_T *)
      &(X_TCTLGC_CtrlErrDistY_met[0])), ((const real32_T *)
      &(Y_TCTLGC_DynBacGain_nu[0])), 5U) * DTE_ReqDeltaByBnkAgl;
  } else {
    rtb_Neg_i = DTE_ReqDeltaByBnkAgl;
  }

  /* Product: '<S588>/Divide' incorporates:
   *  Constant: '<S588>/Constant2'
   *  Constant: '<S588>/Constant3'
   *  Product: '<S588>/Mul1'
   */
  LGC_DeltaByBnkAglComp_deg = 180.0F * rtb_Neg_i / 3.14159274F;

  /* Product: '<S586>/Divide' incorporates:
   *  Constant: '<S586>/Constant2'
   *  Constant: '<S586>/Constant3'
   *  Product: '<S586>/Mul1'
   */
  LGC_DeltaFCmdUnlimited_deg = LGC_DeltaFCmd_rad * 180.0F / 3.14F;

  /* Lookup_n-D: '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1DeltaFCmd = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1DeltaFCmd_X[0])), ((const real32_T *)&(LGC_TimePT1DeltaFCmd_M[0])),
    14U);

  /* Product: '<S585>/Divide' incorporates:
   *  Constant: '<S574>/Parameter'
   *  MinMax: '<S585>/Max1'
   */
  rtb_Divide_hw = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P,
    LGC_TimePT1DeltaFCmd);

  /* Switch: '<S593>/Switch2' incorporates:
   *  Constant: '<S585>/IAM_Ts_P1'
   *  Constant: '<S585>/IAM_Ts_P4'
   *  RelationalOperator: '<S593>/LowerRelop1'
   *  RelationalOperator: '<S593>/UpperRelop'
   *  Switch: '<S593>/Switch'
   */
  if (rtb_Divide_hw > 1.0F) {
    rtb_Divide_hw = 1.0F;
  } else {
    if (rtb_Divide_hw < 0.0F) {
      /* Switch: '<S593>/Switch' incorporates:
       *  Constant: '<S585>/IAM_Ts_P1'
       */
      rtb_Divide_hw = 0.0F;
    }
  }

  /* End of Switch: '<S593>/Switch2' */

  /* Sum: '<S585>/Add' incorporates:
   *  Product: '<S585>/Product'
   *  Sum: '<S585>/Subtract'
   *  UnitDelay: '<S585>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_ep += (LGC_DeltaFCmdUnlimited_deg -
    TJATCT_DW.UnitDelay_DSTATE_ep) * rtb_Divide_hw;

  /* MultiPortSwitch: '<S574>/Multiport Switch2' incorporates:
   *  Constant: '<S574>/Constant16'
   */
  rtb_LGC_StActParSet = 0.0F;

  /* Lookup_n-D: '<S267>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_MaxReqDeltaByDstrb = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const
    real32_T *)&(DTE_MaxReqDeltaByDstrb_X[0])), ((const real32_T *)
    &(DTE_MaxReqDeltaByDstrb_M[0])), 6U);

  /* Lookup_n-D: '<S266>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DTE_SetDeltaGainLaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(DTE_SetDeltaGainLaDmc_X[0])), ((const real32_T *)
    &(DTE_SetDeltaGainLaDmc_M[0])), 12U);

  /* Switch: '<S265>/Init' incorporates:
   *  Logic: '<S265>/FixPt Logical Operator'
   *  UnitDelay: '<S265>/FixPt Unit Delay2'
   */
  if (DTE_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_ej != 0)) {
    /* Switch: '<S265>/Init' incorporates:
     *  Constant: '<S252>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_l = 0.0F;
  }

  /* End of Switch: '<S265>/Init' */

  /* Switch: '<S263>/Switch2' incorporates:
   *  Constant: '<S252>/Constant7'
   *  Constant: '<S252>/Constant9'
   *  RelationalOperator: '<S263>/LowerRelop1'
   *  RelationalOperator: '<S263>/UpperRelop'
   *  Switch: '<S263>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_l > 10000.0F) {
    /* Switch: '<S263>/Switch2' */
    DTE_SetDeltaLaDmc = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_l < -10000.0F) {
    /* Switch: '<S263>/Switch' incorporates:
     *  Constant: '<S252>/Constant9'
     *  Switch: '<S263>/Switch2'
     */
    DTE_SetDeltaLaDmc = -10000.0F;
  } else {
    /* Switch: '<S263>/Switch2' incorporates:
     *  Switch: '<S263>/Switch'
     */
    DTE_SetDeltaLaDmc = TJATCT_DW.FixPtUnitDelay1_DSTATE_l;
  }

  /* End of Switch: '<S263>/Switch2' */

  /* UnitDelay: '<S266>/Unit_Delay' */
  rtb_Divide_hw = TJATCT_DW.Unit_Delay_DSTATE_nr;

  /* UnitDelay: '<S266>/Unit_Delay1' */
  rtb_Switch7 = TJATCT_DW.Unit_Delay1_DSTATE_a;

  /* UnitDelay: '<S266>/Unit_Delay2' */
  rtb_Unit_Delay2_j = TJATCT_DW.Unit_Delay2_DSTATE_i;

  /* UnitDelay: '<S266>/Unit_Delay3' */
  rtb_Neg_i = TJATCT_DW.Unit_Delay3_DSTATE_h;

  /* MultiPortSwitch: '<S266>/Multiport_Switch1' incorporates:
   *  Constant: '<S266>/Parameter1'
   */
  switch (DTE_DelayCyclesLaDMC_P) {
   case 0:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' */
    DTE_NdlySetDeltaLaDmc = DTE_SetDeltaLaDmc;
    break;

   case 1:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' */
    DTE_NdlySetDeltaLaDmc = rtb_Divide_hw;
    break;

   case 2:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S266>/Unit_Delay1'
     */
    DTE_NdlySetDeltaLaDmc = TJATCT_DW.Unit_Delay1_DSTATE_a;
    break;

   case 3:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S266>/Unit_Delay2'
     */
    DTE_NdlySetDeltaLaDmc = TJATCT_DW.Unit_Delay2_DSTATE_i;
    break;

   case 4:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S266>/Unit_Delay3'
     */
    DTE_NdlySetDeltaLaDmc = TJATCT_DW.Unit_Delay3_DSTATE_h;
    break;

   default:
    /* MultiPortSwitch: '<S266>/Multiport_Switch1' incorporates:
     *  UnitDelay: '<S266>/Unit_Delay4'
     */
    DTE_NdlySetDeltaLaDmc = TJATCT_DW.Unit_Delay4_DSTATE_ib;
    break;
  }

  /* End of MultiPortSwitch: '<S266>/Multiport_Switch1' */

  /* Product: '<S266>/Product' */
  DTE_RawReqDeltaByDstrb = DTE_SetDeltaGainLaDmc * DTE_NdlySetDeltaLaDmc;

  /* Switch: '<S268>/Switch2' incorporates:
   *  RelationalOperator: '<S268>/LowerRelop1'
   *  RelationalOperator: '<S268>/UpperRelop'
   *  Switch: '<S268>/Switch'
   *  UnaryMinus: '<S267>/Unary Minus'
   */
  if (DTE_RawReqDeltaByDstrb > DTE_MaxReqDeltaByDstrb) {
    /* Switch: '<S268>/Switch2' */
    DTE_LmtReqDeltaByDstrb = DTE_MaxReqDeltaByDstrb;
  } else if (DTE_RawReqDeltaByDstrb < -DTE_MaxReqDeltaByDstrb) {
    /* Switch: '<S268>/Switch' incorporates:
     *  Switch: '<S268>/Switch2'
     *  UnaryMinus: '<S267>/Unary Minus'
     */
    DTE_LmtReqDeltaByDstrb = -DTE_MaxReqDeltaByDstrb;
  } else {
    /* Switch: '<S268>/Switch2' incorporates:
     *  Switch: '<S268>/Switch'
     */
    DTE_LmtReqDeltaByDstrb = DTE_RawReqDeltaByDstrb;
  }

  /* End of Switch: '<S268>/Switch2' */

  /* MultiPortSwitch: '<S267>/Multiport Switch1' */
  if (!DTE_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S267>/Multiport Switch1' */
    DTE_HldReqDeltaByDstrb = DTE_LmtReqDeltaByDstrb;
  }

  /* End of MultiPortSwitch: '<S267>/Multiport Switch1' */

  /* MultiPortSwitch: '<S267>/Multiport Switch' incorporates:
   *  Constant: '<S267>/Parameter'
   *  Logic: '<S267>/AND'
   */
  if (rtb_Equal3_c || (DTE_CswDstrbCmpn_P == 0)) {
    /* MultiPortSwitch: '<S267>/Multiport Switch' incorporates:
     *  Constant: '<S267>/Constant'
     */
    DTE_ReqDeltaByDstrb = 0.0F;
  } else {
    /* MultiPortSwitch: '<S267>/Multiport Switch' */
    DTE_ReqDeltaByDstrb = DTE_HldReqDeltaByDstrb;
  }

  /* End of MultiPortSwitch: '<S267>/Multiport Switch' */

  /* Product: '<S587>/Divide' incorporates:
   *  Constant: '<S587>/Constant2'
   *  Constant: '<S587>/Constant3'
   *  Product: '<S587>/Mul1'
   */
  LGC_DeltaFCmdDC_deg = 180.0F * DTE_ReqDeltaByDstrb / 3.14159274F;

  /* MultiPortSwitch: '<S574>/Multiport Switch3' incorporates:
   *  Constant: '<S574>/Constant18'
   *  S-Function (sfix_bitop): '<S574>/Bitwise Operator3'
   */
  if ((LGC_CswCssDeltaF_P & 4U) != 0U) {
    rtb_LGC_StActParSet = LGC_DeltaFCmdDC_deg;
  }

  /* End of MultiPortSwitch: '<S574>/Multiport Switch3' */

  /* RelationalOperator: '<S199>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S199>/Delay Input1'
   *
   * Block description for '<S199>/Delay Input1':
   *
   *  Store in Global RAM
   */
  DEV_RstDeltaFGen = (DEV_EnaDeltaFGen > TJATCT_DW.DelayInput1_DSTATE_g);

  /* MultiPortSwitch: '<S197>/Multiport Switch' incorporates:
   *  Constant: '<S197>/Constant1'
   *  MultiPortSwitch: '<S197>/Multiport Switch1'
   *  RelationalOperator: '<S197>/Equal'
   *  RelationalOperator: '<S197>/Equal1'
   */
  if (DEV_RstDeltaFGen) {
    /* MultiPortSwitch: '<S197>/Multiport Switch' incorporates:
     *  Constant: '<S197>/Constant2'
     */
    DEV_TimeDeltaFGen = 0.0F;
  } else {
    if (DEV_EnaDeltaFGen == 1U) {
      /* MultiPortSwitch: '<S197>/Multiport Switch' incorporates:
       *  Constant: '<S197>/Parameter1'
       *  MultiPortSwitch: '<S197>/Multiport Switch1'
       *  Sum: '<S197>/Add'
       *  UnitDelay: '<S197>/Unit Delay'
       */
      DEV_TimeDeltaFGen = DEV_TimeDeltaFGen + DEV_TimeSysCycle_P;
    }
  }

  /* End of MultiPortSwitch: '<S197>/Multiport Switch' */

  /* MultiPortSwitch: '<S196>/Multiport Switch' incorporates:
   *  Constant: '<S196>/Parameter2'
   *  MultiPortSwitch: '<S196>/Multiport Switch1'
   *  RelationalOperator: '<S196>/Less Than'
   */
  if (DEV_EnaDeltaFGen == 0) {
    /* MultiPortSwitch: '<S196>/Multiport Switch' incorporates:
     *  Constant: '<S196>/Constant1'
     */
    DEV_DeltaFTestSignal = 0.0F;
  } else {
    if (DEV_TimeDeltaFGen >= DEV_TimeDeltaFWait_P) {
      /* MultiPortSwitch: '<S196>/Multiport Switch1' incorporates:
       *  Constant: '<S196>/Parameter2'
       *  Sum: '<S196>/Sum1'
       */
      rtb_Product_dv = DEV_TimeDeltaFWait_P + DEV_TimeDeltaFGen;
    } else {
      /* MultiPortSwitch: '<S196>/Multiport Switch1' incorporates:
       *  Constant: '<S196>/Constant3'
       */
      rtb_Product_dv = 0.0F;
    }

    /* MultiPortSwitch: '<S196>/Multiport Switch' incorporates:
     *  Constant: '<S196>/Constant'
     *  Constant: '<S196>/Parameter3'
     *  Constant: '<S196>/Parameter4'
     *  Constant: '<S196>/Parameter5'
     *  Product: '<S196>/Mul'
     *  Product: '<S196>/Mul1'
     *  Product: '<S196>/Mul2'
     *  Product: '<S196>/Mul3'
     *  Sum: '<S196>/Sum'
     *  Trigonometry: '<S196>/Sin'
     */
    DEV_DeltaFTestSignal = (real32_T)(sin((rtb_Product_dv *
      DEV_CoeffDeltaFFrqGain_P + DEV_StrtFrqDeltaF_P) * 6.28 * rtb_Product_dv) *
      DEV_CoeffDeltaFAmp_P);
  }

  /* End of MultiPortSwitch: '<S196>/Multiport Switch' */

  /* MultiPortSwitch: '<S574>/Multiport Switch' incorporates:
   *  Constant: '<S574>/Constant13'
   *  Constant: '<S574>/Constant15'
   *  Constant: '<S574>/Constant9'
   *  Logic: '<S574>/AND'
   *  MultiPortSwitch: '<S574>/Multiport Switch1'
   *  S-Function (sfix_bitop): '<S574>/Bitwise Operator1'
   *  UnitDelay: '<S585>/Unit Delay'
   */
  if ((LGC_CswCssDeltaF_P & 1U) == 0U) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else if ((!LGC_EnaResetByTgq) || (LGC_CswPT1DeltaFCmd_P == 0)) {
    /* MultiPortSwitch: '<S574>/Multiport Switch1' */
    LGC_RawErrCourseDotCdc_tmp = LGC_DeltaFCmdUnlimited_deg;
  } else {
    LGC_RawErrCourseDotCdc_tmp = TJATCT_DW.UnitDelay_DSTATE_ep;
  }

  /* End of MultiPortSwitch: '<S574>/Multiport Switch' */

  /* MultiPortSwitch: '<S574>/Multiport Switch4' incorporates:
   *  Constant: '<S574>/Constant10'
   *  Constant: '<S574>/Constant14'
   *  S-Function (sfix_bitop): '<S574>/Bitwise Operator4'
   */
  if ((LGC_CswCssDeltaF_P & 8U) == 0U) {
    rtb_Divide_l_tmp = 0.0F;
  } else {
    rtb_Divide_l_tmp = LGC_DeltaByBnkAglComp_deg;
  }

  /* End of MultiPortSwitch: '<S574>/Multiport Switch4' */

  /* MultiPortSwitch: '<S574>/Multiport Switch5' incorporates:
   *  Constant: '<S574>/Constant11'
   *  Constant: '<S574>/Constant8'
   *  S-Function (sfix_bitop): '<S574>/Bitwise Operator5'
   */
  if ((LGC_CswCssDeltaF_P & 16U) == 0U) {
    rtb_Product_dv = 0.0F;
  } else {
    rtb_Product_dv = DEV_DeltaFTestSignal;
  }

  /* End of MultiPortSwitch: '<S574>/Multiport Switch5' */

  /* Sum: '<S574>/Sum2' incorporates:
   *  Sum: '<S574>/Sum1'
   *  Sum: '<S574>/Sum3'
   */
  LGC_ReqDeltaF = ((LGC_RawErrCourseDotCdc_tmp + rtb_LGC_StActParSet) +
                   rtb_Divide_l_tmp) + rtb_Product_dv;

  /* Product: '<S589>/Product' incorporates:
   *  Constant: '<S589>/Constant'
   *  Constant: '<S589>/Constant4'
   */
  LGC_MaxReqDeltaF = LGC_MaxReqDeltaFGrd_P * LGC_TimeSysCycle_P;

  /* MultiPortSwitch: '<S589>/Multiport Switch3' */
  if (!LGC_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S589>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_EnaReplanCurValues'
     */
    if (TCTI_EnaReplanCurValues) {
      /* MultiPortSwitch: '<S589>/Multiport Switch1' incorporates:
       *  Constant: '<S589>/Constant7'
       */
      LGC_LmtSelReqDeltaF = 0.0F;
    } else {
      /* Sum: '<S589>/Add' incorporates:
       *  UnitDelay: '<S589>/Unit Delay1'
       */
      rtb_Product_dv = LGC_ReqDeltaF - LGC_LmtSelReqDeltaF;

      /* Switch: '<S591>/Switch2' incorporates:
       *  RelationalOperator: '<S591>/LowerRelop1'
       *  RelationalOperator: '<S591>/UpperRelop'
       *  Switch: '<S591>/Switch'
       *  UnaryMinus: '<S589>/Unary Minus1'
       */
      if (rtb_Product_dv > LGC_MaxReqDeltaF) {
        rtb_Product_dv = LGC_MaxReqDeltaF;
      } else {
        if (rtb_Product_dv < -LGC_MaxReqDeltaF) {
          /* Switch: '<S591>/Switch' incorporates:
           *  UnaryMinus: '<S589>/Unary Minus1'
           */
          rtb_Product_dv = -LGC_MaxReqDeltaF;
        }
      }

      /* End of Switch: '<S591>/Switch2' */

      /* MultiPortSwitch: '<S589>/Multiport Switch1' incorporates:
       *  Sum: '<S589>/Add1'
       *  UnitDelay: '<S589>/Unit Delay1'
       */
      LGC_LmtSelReqDeltaF = rtb_Product_dv + LGC_LmtSelReqDeltaF;
    }

    /* End of MultiPortSwitch: '<S589>/Multiport Switch1' */
  }

  /* End of MultiPortSwitch: '<S589>/Multiport Switch3' */

  /* Switch: '<S592>/Switch2' incorporates:
   *  Constant: '<S590>/Constant2'
   *  Constant: '<S590>/Constant3'
   *  RelationalOperator: '<S592>/LowerRelop1'
   *  RelationalOperator: '<S592>/UpperRelop'
   *  Switch: '<S592>/Switch'
   *  UnaryMinus: '<S590>/Unary Minus'
   */
  if (LGC_LmtSelReqDeltaF > LGC_MaxReqDeltaF_P) {
    /* Switch: '<S592>/Switch2' */
    LGC_LmtReqDeltaF = LGC_MaxReqDeltaF_P;
  } else if (LGC_LmtSelReqDeltaF < -LGC_MaxReqDeltaF_P) {
    /* Switch: '<S592>/Switch' incorporates:
     *  Constant: '<S590>/Constant3'
     *  Switch: '<S592>/Switch2'
     *  UnaryMinus: '<S590>/Unary Minus'
     */
    LGC_LmtReqDeltaF = -LGC_MaxReqDeltaF_P;
  } else {
    /* Switch: '<S592>/Switch2' incorporates:
     *  Switch: '<S592>/Switch'
     */
    LGC_LmtReqDeltaF = LGC_LmtSelReqDeltaF;
  }

  /* End of Switch: '<S592>/Switch2' */

  /* MultiPortSwitch: '<S590>/Multiport Switch2' */
  if (!LGC_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S590>/Multiport Switch2' */
    LGC_HldReqDeltaF = LGC_LmtReqDeltaF;
  }

  /* End of MultiPortSwitch: '<S590>/Multiport Switch2' */

  /* MultiPortSwitch: '<S590>/Multiport Switch' incorporates:
   *  Constant: '<S590>/Constant1'
   *  Constant: '<S590>/Constant18'
   *  RelationalOperator: '<S590>/Equal'
   */
  if (LGC_CswActOverride_P == 0) {
    /* MultiPortSwitch: '<S590>/Multiport Switch1' */
    if (!LGC_EnaCntrlByTgq) {
      /* Sum: '<S574>/Sum2' incorporates:
       *  Constant: '<S590>/Constant6'
       *  MultiPortSwitch: '<S590>/Multiport Switch'
       */
      LGC_ReqDeltaF = 0.0F;
    } else {
      /* Sum: '<S574>/Sum2' incorporates:
       *  MultiPortSwitch: '<S590>/Multiport Switch'
       */
      LGC_ReqDeltaF = LGC_HldReqDeltaF;
    }

    /* End of MultiPortSwitch: '<S590>/Multiport Switch1' */
  }

  /* End of MultiPortSwitch: '<S590>/Multiport Switch' */

  /* DataTypeConversion: '<S81>/Data Type Conversion3' */
  LGC_DeltaFCmd_deg = LGC_ReqDeltaF;

  /* Switch: '<S259>/Init' incorporates:
   *  Logic: '<S259>/FixPt Logical Operator'
   *  UnitDelay: '<S259>/FixPt Unit Delay2'
   */
  if (DTE_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_kd != 0)) {
    /* Switch: '<S259>/Init' incorporates:
     *  Constant: '<S252>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ht = 0.0F;
  }

  /* End of Switch: '<S259>/Init' */

  /* Switch: '<S257>/Switch2' incorporates:
   *  Constant: '<S252>/Constant3'
   *  Constant: '<S252>/Constant5'
   *  RelationalOperator: '<S257>/LowerRelop1'
   *  RelationalOperator: '<S257>/UpperRelop'
   *  Switch: '<S257>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ht > 10000.0F) {
    rtb_Product_dv = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ht < -10000.0F) {
    /* Switch: '<S257>/Switch' incorporates:
     *  Constant: '<S252>/Constant5'
     */
    rtb_Product_dv = -10000.0F;
  } else {
    rtb_Product_dv = TJATCT_DW.FixPtUnitDelay1_DSTATE_ht;
  }

  /* End of Switch: '<S257>/Switch2' */

  /* Switch: '<S265>/Reset' */
  if (DTE_EnaResetByTgq) {
    /* Switch: '<S265>/Init' incorporates:
     *  Constant: '<S252>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_l = 0.0F;
  } else {
    /* Switch: '<S265>/Init' incorporates:
     *  Constant: '<S252>/Parameter2'
     *  Product: '<S256>/Product'
     *  Sum: '<S256>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_l = rtb_Product_dv * DTE_TimeSysCycle_P +
      DTE_SetDeltaLaDmc;
  }

  /* End of Switch: '<S265>/Reset' */

  /* Switch: '<S262>/Init' incorporates:
   *  Logic: '<S262>/FixPt Logical Operator'
   *  UnitDelay: '<S262>/FixPt Unit Delay2'
   */
  if (DTE_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_ck != 0)) {
    /* Switch: '<S262>/Init' incorporates:
     *  Constant: '<S252>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_dm = 0.0F;
  }

  /* End of Switch: '<S262>/Init' */

  /* Switch: '<S260>/Switch2' incorporates:
   *  Constant: '<S252>/Constant1'
   *  Constant: '<S252>/Constant2'
   *  RelationalOperator: '<S260>/LowerRelop1'
   *  RelationalOperator: '<S260>/UpperRelop'
   *  Switch: '<S260>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_dm > 10000.0F) {
    rtb_LGC_StActParSet = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_dm < -10000.0F) {
    /* Switch: '<S260>/Switch' incorporates:
     *  Constant: '<S252>/Constant2'
     */
    rtb_LGC_StActParSet = -10000.0F;
  } else {
    rtb_LGC_StActParSet = TJATCT_DW.FixPtUnitDelay1_DSTATE_dm;
  }

  /* End of Switch: '<S260>/Switch2' */

  /* Switch: '<S259>/Reset' */
  if (DTE_EnaResetByTgq) {
    /* Switch: '<S259>/Init' incorporates:
     *  Constant: '<S252>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ht = 0.0F;
  } else {
    /* Switch: '<S259>/Init' incorporates:
     *  Constant: '<S252>/Parameter'
     *  Product: '<S254>/Product'
     *  Sum: '<S254>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ht = rtb_LGC_StActParSet *
      DTE_TimeSysCycle_P + rtb_Product_dv;
  }

  /* End of Switch: '<S259>/Reset' */

  /* Lookup_n-D: '<S241>/1-D Lookup Table' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffDenS0LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffDenS0LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffDenS0LaDmc_M[0])),
    12U);

  /* Lookup_n-D: '<S241>/1-D Lookup Table1' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffDenS1LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffDenS1LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffDenS1LaDmc_M[0])),
    12U);

  /* MultiPortSwitch: '<S249>/Multiport Switch1' incorporates:
   *  Constant: '<S249>/Constant2'
   *  Sum: '<S249>/Subtract1'
   *  UnitDelay: '<S249>/Unit Delay1'
   */
  if (!DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaByVdyFcn -
      TJATCT_DW.UnitDelay1_DSTATE_f;
  }

  /* End of MultiPortSwitch: '<S249>/Multiport Switch1' */

  /* MultiPortSwitch: '<S249>/Multiport Switch2' incorporates:
   *  Constant: '<S249>/IAM_Ts_P1'
   *  Constant: '<S249>/IAM_Ts_P11'
   *  Constant: '<S249>/IAM_Ts_P2'
   *  Constant: '<S249>/Parameter3'
   *  MinMax: '<S249>/Max1'
   *  MinMax: '<S249>/Min'
   *  RelationalOperator: '<S249>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S249>/Multiport Switch2' */

  /* Product: '<S249>/Divide1' */
  DTE_RawDeltaDotLaDmc = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Switch: '<S251>/Switch2' incorporates:
   *  Constant: '<S250>/IAM_Ts_P1'
   *  Constant: '<S250>/IAM_Ts_P4'
   *  RelationalOperator: '<S251>/LowerRelop1'
   *  RelationalOperator: '<S251>/UpperRelop'
   *  Switch: '<S251>/Switch'
   */
  if (rtb_Divide_bb > 1.0F) {
    rtb_Divide_bb = 1.0F;
  } else {
    if (rtb_Divide_bb < 0.0F) {
      /* Switch: '<S251>/Switch' incorporates:
       *  Constant: '<S250>/IAM_Ts_P1'
       */
      rtb_Divide_bb = 0.0F;
    }
  }

  /* End of Switch: '<S251>/Switch2' */

  /* Sum: '<S250>/Add' incorporates:
   *  Product: '<S250>/Product'
   *  Sum: '<S250>/Subtract'
   */
  DTE_DeltaDotLaDmc = (DTE_RawDeltaDotLaDmc - DTE_DeltaDotLaDmc) * rtb_Divide_bb
    + DTE_DeltaDotLaDmc;

  /* Lookup_n-D: '<S241>/1-D Lookup Table2' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffDenS2LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffDenS2LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffDenS2LaDmc_M[0])),
    12U);

  /* MultiPortSwitch: '<S247>/Multiport Switch1' incorporates:
   *  Constant: '<S247>/Constant2'
   *  Sum: '<S247>/Subtract1'
   *  UnitDelay: '<S247>/Unit Delay1'
   */
  if (!DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaDotLaDmc -
      TJATCT_DW.UnitDelay1_DSTATE_i1;
  }

  /* End of MultiPortSwitch: '<S247>/Multiport Switch1' */

  /* MultiPortSwitch: '<S247>/Multiport Switch2' incorporates:
   *  Constant: '<S247>/IAM_Ts_P1'
   *  Constant: '<S247>/IAM_Ts_P11'
   *  Constant: '<S247>/IAM_Ts_P2'
   *  Constant: '<S247>/Parameter1'
   *  MinMax: '<S247>/Max1'
   *  MinMax: '<S247>/Min'
   *  RelationalOperator: '<S247>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S247>/Multiport Switch2' */

  /* Product: '<S247>/Divide1' */
  DTE_Delta2DotLaDmc = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Lookup_n-D: '<S241>/1-D Lookup Table3' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffDenS3LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffDenS3LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffDenS3LaDmc_M[0])),
    12U);

  /* MultiPortSwitch: '<S248>/Multiport Switch1' incorporates:
   *  Constant: '<S248>/Constant2'
   *  Sum: '<S248>/Subtract1'
   *  UnitDelay: '<S248>/Unit Delay1'
   */
  if (!DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_Delta2DotLaDmc -
      TJATCT_DW.UnitDelay1_DSTATE_g2;
  }

  /* End of MultiPortSwitch: '<S248>/Multiport Switch1' */

  /* MultiPortSwitch: '<S248>/Multiport Switch2' incorporates:
   *  Constant: '<S248>/IAM_Ts_P1'
   *  Constant: '<S248>/IAM_Ts_P11'
   *  Constant: '<S248>/IAM_Ts_P2'
   *  Constant: '<S248>/Parameter1'
   *  MinMax: '<S248>/Max1'
   *  MinMax: '<S248>/Min'
   *  RelationalOperator: '<S248>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S248>/Multiport Switch2' */

  /* Product: '<S248>/Divide1' */
  DTE_Delta3DotLaDmc = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Sum: '<S246>/Add1' incorporates:
   *  Product: '<S246>/Product1'
   *  Product: '<S246>/Product5'
   *  Product: '<S246>/Product6'
   *  Product: '<S246>/Product7'
   *  Sum: '<S246>/Add2'
   *  Sum: '<S246>/Add3'
   */
  DTE_ResDeltaDenLaDmc = (DTE_CoeffDenS0LaDmc * DTE_DeltaByVdyFcn +
    DTE_CoeffDenS1LaDmc * DTE_DeltaDotLaDmc) + (DTE_Delta2DotLaDmc *
    DTE_CoeffDenS2LaDmc + DTE_CoeffDenS3LaDmc * DTE_Delta3DotLaDmc);

  /* UnitDelay: '<S252>/Unit Delay1' */
  DTE_DlySetDelta2DotLaDmc = TJATCT_DW.UnitDelay1_DSTATE_gy;

  /* Lookup_n-D: '<S241>/1-D Lookup Table4' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffNumS0LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffNumS0LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffNumS0LaDmc_M[0])),
    12U);

  /* Lookup_n-D: '<S241>/1-D Lookup Table5' incorporates:
   *  Switch: '<S217>/Switch2'
   */
  DTE_CoeffNumS1LaDmc = look1_iflf_binlxpw(DTE_LmtVehVelX, ((const real32_T *)
    &(DTE_CoeffNumS1LaDmc_X[0])), ((const real32_T *)&(DTE_CoeffNumS1LaDmc_M[0])),
    12U);

  /* UnitDelay: '<S252>/Unit Delay2' */
  DTE_DlySetDeltaLaDmc = TJATCT_DW.UnitDelay2_DSTATE_o;

  /* UnitDelay: '<S252>/Unit Delay' */
  DTE_DlySetDeltaDotLaDmc = TJATCT_DW.UnitDelay_DSTATE_k;

  /* MultiPortSwitch: '<S253>/Multiport Switch' */
  if (!DTE_EnaCtrlByTgq) {
    /* MultiPortSwitch: '<S253>/Multiport Switch' incorporates:
     *  Constant: '<S253>/Constant17'
     */
    DTE_SetDelta3DotLaDmc = 0.0F;
  } else {
    /* Sum: '<S253>/Add2' incorporates:
     *  Constant: '<S253>/Parameter3'
     *  Constant: '<S253>/Parameter4'
     *  Sum: '<S253>/Add3'
     */
    rtb_Divide_bb = DTE_Time1FltLaDmc_P + DTE_Time2FltLaDmc_P;

    /* Product: '<S253>/Product3' incorporates:
     *  Constant: '<S253>/Parameter2'
     *  Constant: '<S253>/Parameter6'
     *  Product: '<S253>/Product6'
     */
    rtb_Switch2_m4 = DTE_Time1FltLaDmc_P * DTE_Time2FltLaDmc_P;

    /* MultiPortSwitch: '<S253>/Multiport Switch' incorporates:
     *  Product: '<S253>/Divide'
     *  Product: '<S253>/Product'
     *  Product: '<S253>/Product1'
     *  Product: '<S253>/Product2'
     *  Product: '<S253>/Product3'
     *  Product: '<S253>/Product4'
     *  Product: '<S253>/Product5'
     *  Product: '<S253>/Product7'
     *  Product: '<S253>/Product8'
     *  Sum: '<S253>/Add1'
     *  Sum: '<S253>/Add2'
     *  Sum: '<S253>/Add4'
     *  Sum: '<S253>/Subtract'
     *  Sum: '<S253>/Subtract1'
     *  Sum: '<S253>/Subtract2'
     */
    DTE_SetDelta3DotLaDmc = (((DTE_ResDeltaDenLaDmc - (rtb_Switch2_m4 *
      DTE_CoeffNumS0LaDmc + rtb_Divide_bb * DTE_CoeffNumS1LaDmc) *
      DTE_DlySetDelta2DotLaDmc) - DTE_CoeffNumS0LaDmc * DTE_DlySetDeltaLaDmc) -
      (rtb_Divide_bb * DTE_CoeffNumS0LaDmc + DTE_CoeffNumS1LaDmc) *
      DTE_DlySetDeltaDotLaDmc) / (rtb_Switch2_m4 * DTE_CoeffNumS1LaDmc);
  }

  /* End of MultiPortSwitch: '<S253>/Multiport Switch' */

  /* Switch: '<S262>/Reset' */
  if (DTE_EnaResetByTgq) {
    /* Switch: '<S262>/Init' incorporates:
     *  Constant: '<S252>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_dm = 0.0F;
  } else {
    /* Switch: '<S262>/Init' incorporates:
     *  Constant: '<S252>/Parameter1'
     *  Product: '<S255>/Product'
     *  Sum: '<S255>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_dm = DTE_SetDelta3DotLaDmc *
      DTE_TimeSysCycle_P + rtb_LGC_StActParSet;
  }

  /* End of Switch: '<S262>/Reset' */

  /* UnitDelay: '<S157>/Unit Delay1' */
  DEV_DlySetDeltaF2DotRte = TJATCT_DW.UnitDelay1_DSTATE_e;

  /* Sum: '<S141>/Sum' incorporates:
   *  Constant: '<S141>/Constant1'
   *  Constant: '<S141>/Constant2'
   *  Sum: '<S142>/Sum'
   */
  DTE_DeltaFRte_tmp = DEV_DistCogToFrontAxle_P + DEV_DistCogToRearAxle_P;

  /* Product: '<S141>/Mul1' incorporates:
   *  Inport: '<Root>/TCTI_EstSelfSteerGrdnt'
   *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
   *  Product: '<S141>/Mul2'
   *  Sum: '<S141>/Sum'
   *  Sum: '<S141>/Sum2'
   */
  DTE_DeltaFRte = (rtb_Divide_bo * TCTI_EstSelfSteerGrdnt + DTE_DeltaFRte_tmp) *
    TCTI_ReqTrajCrvTpl;

  /* Product: '<S154>/Divide' incorporates:
   *  Constant: '<S153>/Parameter1'
   *  Constant: '<S153>/Parameter2'
   *  MinMax: '<S154>/Max1'
   */
  rtb_Divide_bb = DEV_TimeSysCycle_P / fmaxf(DEV_TimeSysCycle_P,
    DTE_TimeFltDeltaFRte_P);

  /* RelationalOperator: '<S140>/Equal1' incorporates:
   *  Constant: '<S140>/Parameter2'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  DEV_EnaCntrlByTgq = (TCTI_BtfTrajGuiQualifier != DEV_EnaOffByTgq_P);

  /* RelationalOperator: '<S140>/Equal3' incorporates:
   *  Constant: '<S140>/Parameter4'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  rtb_Equal3_c = (TCTI_BtfTrajGuiQualifier == DEV_EnaFreezeByTgq_P);

  /* Logic: '<S140>/OR' incorporates:
   *  Inport: '<Root>/TCTI_EnaReplanCurValues'
   *  Logic: '<S140>/NOT'
   *  RelationalOperator: '<S147>/FixPt Relational Operator'
   *  UnitDelay: '<S147>/Delay Input1'
   *
   * Block description for '<S147>/Delay Input1':
   *
   *  Store in Global RAM
   */
  DEV_EnaResetByTgq = ((!DEV_EnaCntrlByTgq) || ((int32_T)rtb_Equal3_c < (int32_T)
    TJATCT_DW.DelayInput1_DSTATE_n) || TCTI_EnaReplanCurValues);

  /* MultiPortSwitch: '<S153>/Multiport Switch1' incorporates:
   *  Constant: '<S153>/Constant2'
   *  Sum: '<S153>/Subtract1'
   *  UnitDelay: '<S153>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaFRte - TJATCT_DW.UnitDelay1_DSTATE_if;
  }

  /* End of MultiPortSwitch: '<S153>/Multiport Switch1' */

  /* MultiPortSwitch: '<S153>/Multiport Switch2' incorporates:
   *  Constant: '<S153>/IAM_Ts_P1'
   *  Constant: '<S153>/IAM_Ts_P11'
   *  Constant: '<S153>/IAM_Ts_P2'
   *  Constant: '<S153>/Parameter3'
   *  MinMax: '<S153>/Max1'
   *  MinMax: '<S153>/Min'
   *  RelationalOperator: '<S153>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S153>/Multiport Switch2' */

  /* Product: '<S153>/Divide1' */
  DTE_RawDeltaFDotRte = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Switch: '<S155>/Switch2' incorporates:
   *  Constant: '<S154>/IAM_Ts_P1'
   *  Constant: '<S154>/IAM_Ts_P4'
   *  RelationalOperator: '<S155>/LowerRelop1'
   *  RelationalOperator: '<S155>/UpperRelop'
   *  Switch: '<S155>/Switch'
   */
  if (rtb_Divide_bb > 1.0F) {
    rtb_Divide_bb = 1.0F;
  } else {
    if (rtb_Divide_bb < 0.0F) {
      /* Switch: '<S155>/Switch' incorporates:
       *  Constant: '<S154>/IAM_Ts_P1'
       */
      rtb_Divide_bb = 0.0F;
    }
  }

  /* End of Switch: '<S155>/Switch2' */

  /* Sum: '<S154>/Add' incorporates:
   *  Product: '<S154>/Product'
   *  Sum: '<S154>/Subtract'
   */
  DTE_DeltaFDotRte = (DTE_RawDeltaFDotRte - DTE_DeltaFDotRte) * rtb_Divide_bb +
    DTE_DeltaFDotRte;

  /* MultiPortSwitch: '<S151>/Multiport Switch1' incorporates:
   *  Constant: '<S151>/Constant2'
   *  Sum: '<S151>/Subtract1'
   *  UnitDelay: '<S151>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaFDotRte -
      TJATCT_DW.UnitDelay1_DSTATE_fo;
  }

  /* End of MultiPortSwitch: '<S151>/Multiport Switch1' */

  /* MultiPortSwitch: '<S151>/Multiport Switch2' incorporates:
   *  Constant: '<S151>/IAM_Ts_P1'
   *  Constant: '<S151>/IAM_Ts_P11'
   *  Constant: '<S151>/IAM_Ts_P2'
   *  Constant: '<S151>/Parameter1'
   *  MinMax: '<S151>/Max1'
   *  MinMax: '<S151>/Min'
   *  RelationalOperator: '<S151>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S151>/Multiport Switch2' */

  /* Product: '<S151>/Divide1' */
  DTE_DeltaF2DotRte = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* MultiPortSwitch: '<S152>/Multiport Switch1' incorporates:
   *  Constant: '<S152>/Constant2'
   *  Sum: '<S152>/Subtract1'
   *  UnitDelay: '<S152>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaF2DotRte -
      TJATCT_DW.UnitDelay1_DSTATE_ip;
  }

  /* End of MultiPortSwitch: '<S152>/Multiport Switch1' */

  /* MultiPortSwitch: '<S152>/Multiport Switch2' incorporates:
   *  Constant: '<S152>/IAM_Ts_P1'
   *  Constant: '<S152>/IAM_Ts_P11'
   *  Constant: '<S152>/IAM_Ts_P2'
   *  Constant: '<S152>/Parameter1'
   *  MinMax: '<S152>/Max1'
   *  MinMax: '<S152>/Min'
   *  RelationalOperator: '<S152>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S152>/Multiport Switch2' */

  /* Product: '<S152>/Divide1' */
  DTE_DeltaF3DotRte = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Sum: '<S150>/Add1' incorporates:
   *  Product: '<S150>/Product1'
   *  Product: '<S150>/Product5'
   *  Product: '<S150>/Product6'
   *  Product: '<S150>/Product7'
   *  Sum: '<S150>/Add2'
   *  Sum: '<S150>/Add3'
   */
  DTE_ResDeltaDenRte = (DTE_CoeffDenS0LaDmc * DTE_DeltaFRte +
                        DTE_CoeffDenS1LaDmc * DTE_DeltaFDotRte) +
    (DTE_CoeffDenS2LaDmc * DTE_DeltaF2DotRte + DTE_CoeffDenS3LaDmc *
     DTE_DeltaF3DotRte);

  /* UnitDelay: '<S157>/Unit Delay2' */
  DEV_DlySetDeltaFRte = DEV_SetDeltaFRte;

  /* UnitDelay: '<S157>/Unit Delay' */
  DEV_DlySetDeltaFDotRte = TJATCT_DW.UnitDelay_DSTATE_mg;

  /* MultiPortSwitch: '<S156>/Multiport Switch' */
  if (!DEV_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S156>/Multiport Switch' incorporates:
     *  Constant: '<S156>/Constant17'
     */
    DEV_SetDeltaF3DotRte = 0.0F;
  } else {
    /* Sum: '<S156>/Add2' incorporates:
     *  Constant: '<S156>/Parameter3'
     *  Constant: '<S156>/Parameter4'
     *  Sum: '<S156>/Add3'
     */
    rtb_Divide_bb = DEV_Time1FltLaDmc_P + DEV_Time2FltLaDmc_P;

    /* Product: '<S156>/Product3' incorporates:
     *  Constant: '<S156>/Parameter2'
     *  Constant: '<S156>/Parameter6'
     *  Product: '<S156>/Product6'
     */
    rtb_Switch2_m4 = DEV_Time1FltLaDmc_P * DEV_Time2FltLaDmc_P;

    /* MultiPortSwitch: '<S156>/Multiport Switch' incorporates:
     *  Product: '<S156>/Divide'
     *  Product: '<S156>/Product'
     *  Product: '<S156>/Product1'
     *  Product: '<S156>/Product2'
     *  Product: '<S156>/Product3'
     *  Product: '<S156>/Product4'
     *  Product: '<S156>/Product5'
     *  Product: '<S156>/Product7'
     *  Product: '<S156>/Product8'
     *  Sum: '<S156>/Add1'
     *  Sum: '<S156>/Add2'
     *  Sum: '<S156>/Add4'
     *  Sum: '<S156>/Subtract'
     *  Sum: '<S156>/Subtract1'
     *  Sum: '<S156>/Subtract2'
     */
    DEV_SetDeltaF3DotRte = (((DTE_ResDeltaDenRte - (rtb_Switch2_m4 *
      DTE_CoeffNumS0LaDmc + rtb_Divide_bb * DTE_CoeffNumS1LaDmc) *
      DEV_DlySetDeltaF2DotRte) - DTE_CoeffNumS0LaDmc * DEV_DlySetDeltaFRte) -
      (rtb_Divide_bb * DTE_CoeffNumS0LaDmc + DTE_CoeffNumS1LaDmc) *
      DEV_DlySetDeltaFDotRte) / (rtb_Switch2_m4 * DTE_CoeffNumS1LaDmc);
  }

  /* End of MultiPortSwitch: '<S156>/Multiport Switch' */

  /* Switch: '<S167>/Init' incorporates:
   *  Logic: '<S167>/FixPt Logical Operator'
   *  UnitDelay: '<S167>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_el != 0)) {
    /* Switch: '<S167>/Init' incorporates:
     *  Constant: '<S157>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_de = 0.0F;
  }

  /* End of Switch: '<S167>/Init' */

  /* Switch: '<S165>/Switch2' incorporates:
   *  Constant: '<S157>/Constant1'
   *  Constant: '<S157>/Constant2'
   *  RelationalOperator: '<S165>/LowerRelop1'
   *  RelationalOperator: '<S165>/UpperRelop'
   *  Switch: '<S165>/Switch'
   *  UnitDelay: '<S157>/Unit Delay1'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_de > 100.0F) {
    TJATCT_DW.UnitDelay1_DSTATE_e = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_de < -100.0F) {
    /* Switch: '<S165>/Switch' incorporates:
     *  Constant: '<S157>/Constant2'
     *  UnitDelay: '<S157>/Unit Delay1'
     */
    TJATCT_DW.UnitDelay1_DSTATE_e = -100.0F;
  } else {
    TJATCT_DW.UnitDelay1_DSTATE_e = TJATCT_DW.FixPtUnitDelay1_DSTATE_de;
  }

  /* End of Switch: '<S165>/Switch2' */

  /* Switch: '<S167>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S167>/Init' incorporates:
     *  Constant: '<S157>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_de = 0.0F;
  } else {
    /* Switch: '<S167>/Init' incorporates:
     *  Constant: '<S157>/Parameter1'
     *  Product: '<S160>/Product'
     *  Sum: '<S160>/Subtract'
     *  UnitDelay: '<S157>/Unit Delay1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_de = DEV_SetDeltaF3DotRte *
      DEV_TimeSysCycle_P + TJATCT_DW.UnitDelay1_DSTATE_e;
  }

  /* End of Switch: '<S167>/Reset' */

  /* Switch: '<S164>/Init' incorporates:
   *  Logic: '<S164>/FixPt Logical Operator'
   *  UnitDelay: '<S164>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_kf != 0)) {
    /* Switch: '<S164>/Init' incorporates:
     *  Constant: '<S157>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_of = 0.0F;
  }

  /* End of Switch: '<S164>/Init' */

  /* Switch: '<S162>/Switch2' incorporates:
   *  Constant: '<S157>/Constant3'
   *  Constant: '<S157>/Constant5'
   *  RelationalOperator: '<S162>/LowerRelop1'
   *  RelationalOperator: '<S162>/UpperRelop'
   *  Switch: '<S162>/Switch'
   *  UnitDelay: '<S157>/Unit Delay'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_of > 100.0F) {
    TJATCT_DW.UnitDelay_DSTATE_mg = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_of < -100.0F) {
    /* Switch: '<S162>/Switch' incorporates:
     *  Constant: '<S157>/Constant5'
     *  UnitDelay: '<S157>/Unit Delay'
     */
    TJATCT_DW.UnitDelay_DSTATE_mg = -100.0F;
  } else {
    TJATCT_DW.UnitDelay_DSTATE_mg = TJATCT_DW.FixPtUnitDelay1_DSTATE_of;
  }

  /* End of Switch: '<S162>/Switch2' */

  /* Switch: '<S164>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S164>/Init' incorporates:
     *  Constant: '<S157>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_of = 0.0F;
  } else {
    /* Switch: '<S164>/Init' incorporates:
     *  Constant: '<S157>/Parameter'
     *  Product: '<S159>/Product'
     *  Sum: '<S159>/Subtract'
     *  UnitDelay: '<S157>/Unit Delay'
     *  UnitDelay: '<S157>/Unit Delay1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_of = TJATCT_DW.UnitDelay1_DSTATE_e *
      DEV_TimeSysCycle_P + TJATCT_DW.UnitDelay_DSTATE_mg;
  }

  /* End of Switch: '<S164>/Reset' */

  /* Switch: '<S170>/Init' incorporates:
   *  Logic: '<S170>/FixPt Logical Operator'
   *  UnitDelay: '<S170>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_oe != 0)) {
    /* Switch: '<S170>/Init' incorporates:
     *  Constant: '<S157>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ee = 0.0F;
  }

  /* End of Switch: '<S170>/Init' */

  /* Switch: '<S168>/Switch2' incorporates:
   *  Constant: '<S157>/Constant7'
   *  Constant: '<S157>/Constant9'
   *  RelationalOperator: '<S168>/LowerRelop1'
   *  RelationalOperator: '<S168>/UpperRelop'
   *  Switch: '<S168>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ee > 12.56F) {
    /* Switch: '<S168>/Switch2' */
    DEV_SetDeltaFRte = 12.56F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ee < -12.56F) {
    /* Switch: '<S168>/Switch' incorporates:
     *  Constant: '<S157>/Constant9'
     *  Switch: '<S168>/Switch2'
     */
    DEV_SetDeltaFRte = -12.56F;
  } else {
    /* Switch: '<S168>/Switch2' incorporates:
     *  Switch: '<S168>/Switch'
     */
    DEV_SetDeltaFRte = TJATCT_DW.FixPtUnitDelay1_DSTATE_ee;
  }

  /* End of Switch: '<S168>/Switch2' */

  /* Switch: '<S170>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S170>/Init' incorporates:
     *  Constant: '<S157>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ee = 0.0F;
  } else {
    /* Switch: '<S170>/Init' incorporates:
     *  Constant: '<S157>/Parameter2'
     *  Product: '<S161>/Product'
     *  Sum: '<S161>/Subtract'
     *  UnitDelay: '<S157>/Unit Delay'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ee = TJATCT_DW.UnitDelay_DSTATE_mg *
      DEV_TimeSysCycle_P + DEV_SetDeltaFRte;
  }

  /* End of Switch: '<S170>/Reset' */

  /* RelationalOperator: '<S140>/Equal' incorporates:
   *  Constant: '<S140>/Parameter1'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   */
  DEV_EnaFreezeByTgq = (TCTI_BtfTrajGuiQualifier == DEV_EnaFreezeByTgq_P);

  /* MultiPortSwitch: '<S143>/Multiport Switch1' */
  if (!DEV_EnaFreezeByTgq) {
    /* MultiPortSwitch: '<S143>/Multiport Switch1' incorporates:
     *  Inport: '<Root>/TCTI_VehicleVelX'
     *  Lookup_n-D: '<S143>/1-D Lookup Table1'
     *  Product: '<S143>/Mul'
     */
    DEV_HldReqDeltaFRte = DEV_SetDeltaFRte * look1_iflf_binlxpw
      (TCTI_VehicleVelX, ((const real32_T *)&(DEV_CoeffDeltaGainFfc_X[0])), ((
         const real32_T *)&(DEV_CoeffDeltaGainFfc_M[0])), 2U);
  }

  /* End of MultiPortSwitch: '<S143>/Multiport Switch1' */

  /* MultiPortSwitch: '<S143>/Multiport Switch' */
  if (!DEV_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S143>/Multiport Switch' incorporates:
     *  Constant: '<S143>/Constant2'
     */
    DEV_ReqDeltaFRte = 0.0F;
  } else {
    /* MultiPortSwitch: '<S143>/Multiport Switch' */
    DEV_ReqDeltaFRte = DEV_HldReqDeltaFRte;
  }

  /* End of MultiPortSwitch: '<S143>/Multiport Switch' */

  /* Switch: '<S193>/Init' incorporates:
   *  Logic: '<S193>/FixPt Logical Operator'
   *  UnitDelay: '<S193>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_i != 0)) {
    /* Switch: '<S193>/Init' incorporates:
     *  Constant: '<S180>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_i4 = 0.0F;
  }

  /* End of Switch: '<S193>/Init' */

  /* Switch: '<S191>/Switch2' incorporates:
   *  Constant: '<S180>/Constant7'
   *  Constant: '<S180>/Constant9'
   *  RelationalOperator: '<S191>/LowerRelop1'
   *  RelationalOperator: '<S191>/UpperRelop'
   *  Switch: '<S191>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_i4 > 12.56F) {
    /* Switch: '<S191>/Switch2' */
    DEV_SetDeltaFPar = 12.56F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_i4 < -12.56F) {
    /* Switch: '<S191>/Switch' incorporates:
     *  Constant: '<S180>/Constant9'
     *  Switch: '<S191>/Switch2'
     */
    DEV_SetDeltaFPar = -12.56F;
  } else {
    /* Switch: '<S191>/Switch2' incorporates:
     *  Switch: '<S191>/Switch'
     */
    DEV_SetDeltaFPar = TJATCT_DW.FixPtUnitDelay1_DSTATE_i4;
  }

  /* End of Switch: '<S191>/Switch2' */

  /* Lookup_n-D: '<S144>/Y_TCTFFC_GainFFC_nu' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  DEV_CoeffDeltaGainFfc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(DEV_CoeffDeltaGainFfc_X[0])), ((const real32_T *)
    &(DEV_CoeffDeltaGainFfc_M[0])), 2U);

  /* MultiPortSwitch: '<S144>/Multiport Switch1' incorporates:
   *  Product: '<S144>/Mul'
   *  UnitDelay: '<S144>/Unit Delay'
   */
  if (!DEV_EnaFreezeByTgq) {
    TJATCT_DW.UnitDelay_DSTATE_mw = DEV_SetDeltaFPar * DEV_CoeffDeltaGainFfc;
  }

  /* End of MultiPortSwitch: '<S144>/Multiport Switch1' */

  /* MultiPortSwitch: '<S144>/Multiport Switch' */
  if (!DEV_EnaCntrlByTgq) {
    /* Outport: '<Root>/DEV_ReqDeltaFPar' incorporates:
     *  Constant: '<S144>/Constant2'
     */
    TJATCT_Y.DEV_ReqDeltaFPar = 0.0F;
  } else {
    /* Outport: '<Root>/DEV_ReqDeltaFPar' incorporates:
     *  UnitDelay: '<S144>/Unit Delay'
     */
    TJATCT_Y.DEV_ReqDeltaFPar = TJATCT_DW.UnitDelay_DSTATE_mw;
  }

  /* End of MultiPortSwitch: '<S144>/Multiport Switch' */

  /* Switch: '<S187>/Init' incorporates:
   *  Logic: '<S187>/FixPt Logical Operator'
   *  UnitDelay: '<S187>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_on != 0)) {
    /* Switch: '<S187>/Init' incorporates:
     *  Constant: '<S180>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_im = 0.0F;
  }

  /* End of Switch: '<S187>/Init' */

  /* Switch: '<S185>/Switch2' incorporates:
   *  Constant: '<S180>/Constant3'
   *  Constant: '<S180>/Constant5'
   *  RelationalOperator: '<S185>/LowerRelop1'
   *  RelationalOperator: '<S185>/UpperRelop'
   *  Switch: '<S185>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_im > 100.0F) {
    rtb_Divide_bb = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_im < -100.0F) {
    /* Switch: '<S185>/Switch' incorporates:
     *  Constant: '<S180>/Constant5'
     */
    rtb_Divide_bb = -100.0F;
  } else {
    rtb_Divide_bb = TJATCT_DW.FixPtUnitDelay1_DSTATE_im;
  }

  /* End of Switch: '<S185>/Switch2' */

  /* Switch: '<S193>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S193>/Init' incorporates:
     *  Constant: '<S180>/Constant8'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_i4 = 0.0F;
  } else {
    /* Switch: '<S193>/Init' incorporates:
     *  Constant: '<S180>/Parameter2'
     *  Product: '<S184>/Product'
     *  Sum: '<S184>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_i4 = rtb_Divide_bb * DEV_TimeSysCycle_P +
      DEV_SetDeltaFPar;
  }

  /* End of Switch: '<S193>/Reset' */

  /* Switch: '<S190>/Init' incorporates:
   *  Logic: '<S190>/FixPt Logical Operator'
   *  UnitDelay: '<S190>/FixPt Unit Delay2'
   */
  if (DEV_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_ct != 0)) {
    /* Switch: '<S190>/Init' incorporates:
     *  Constant: '<S180>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_es = 0.0F;
  }

  /* End of Switch: '<S190>/Init' */

  /* Switch: '<S188>/Switch2' incorporates:
   *  Constant: '<S180>/Constant1'
   *  Constant: '<S180>/Constant2'
   *  RelationalOperator: '<S188>/LowerRelop1'
   *  RelationalOperator: '<S188>/UpperRelop'
   *  Switch: '<S188>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_es > 100.0F) {
    rtb_Switch2_m4 = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_es < -100.0F) {
    /* Switch: '<S188>/Switch' incorporates:
     *  Constant: '<S180>/Constant2'
     */
    rtb_Switch2_m4 = -100.0F;
  } else {
    rtb_Switch2_m4 = TJATCT_DW.FixPtUnitDelay1_DSTATE_es;
  }

  /* End of Switch: '<S188>/Switch2' */

  /* Switch: '<S187>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S187>/Init' incorporates:
     *  Constant: '<S180>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_im = 0.0F;
  } else {
    /* Switch: '<S187>/Init' incorporates:
     *  Constant: '<S180>/Parameter'
     *  Product: '<S182>/Product'
     *  Sum: '<S182>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_im = rtb_Switch2_m4 * DEV_TimeSysCycle_P +
      rtb_Divide_bb;
  }

  /* End of Switch: '<S187>/Reset' */

  /* Product: '<S142>/Mul1' incorporates:
   *  Constant: '<S142>/Constant3'
   *  Inport: '<Root>/TCTI_ReqTrajCrvTpl'
   *  Product: '<S142>/Mul2'
   *  Sum: '<S142>/Sum2'
   */
  DTE_DeltaFPar = (rtb_Divide_bo * DEV_SelfSteeringGrd_nu + DTE_DeltaFRte_tmp) *
    TCTI_ReqTrajCrvTpl;

  /* Product: '<S177>/Divide' incorporates:
   *  Constant: '<S176>/Parameter1'
   *  Constant: '<S176>/Parameter2'
   *  MinMax: '<S177>/Max1'
   */
  rtb_Divide_bo = DEV_TimeSysCycle_P / fmaxf(DEV_TimeSysCycle_P,
    DTE_TimeFltDeltaFPar_P);

  /* MultiPortSwitch: '<S176>/Multiport Switch1' incorporates:
   *  Constant: '<S176>/Constant2'
   *  Sum: '<S176>/Subtract1'
   *  UnitDelay: '<S176>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaFPar - TJATCT_DW.UnitDelay1_DSTATE_ei;
  }

  /* End of MultiPortSwitch: '<S176>/Multiport Switch1' */

  /* MultiPortSwitch: '<S176>/Multiport Switch2' incorporates:
   *  Constant: '<S176>/IAM_Ts_P1'
   *  Constant: '<S176>/IAM_Ts_P11'
   *  Constant: '<S176>/IAM_Ts_P2'
   *  Constant: '<S176>/Parameter3'
   *  MinMax: '<S176>/Max1'
   *  MinMax: '<S176>/Min'
   *  RelationalOperator: '<S176>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S176>/Multiport Switch2' */

  /* Product: '<S176>/Divide1' */
  DTE_RawDeltaFDotPar = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Switch: '<S178>/Switch2' incorporates:
   *  Constant: '<S177>/IAM_Ts_P1'
   *  Constant: '<S177>/IAM_Ts_P4'
   *  RelationalOperator: '<S178>/LowerRelop1'
   *  RelationalOperator: '<S178>/UpperRelop'
   *  Switch: '<S178>/Switch'
   */
  if (rtb_Divide_bo > 1.0F) {
    rtb_Divide_bo = 1.0F;
  } else {
    if (rtb_Divide_bo < 0.0F) {
      /* Switch: '<S178>/Switch' incorporates:
       *  Constant: '<S177>/IAM_Ts_P1'
       */
      rtb_Divide_bo = 0.0F;
    }
  }

  /* End of Switch: '<S178>/Switch2' */

  /* Sum: '<S177>/Add' incorporates:
   *  Product: '<S177>/Product'
   *  Sum: '<S177>/Subtract'
   */
  DTE_DeltaFDotPar = (DTE_RawDeltaFDotPar - DTE_DeltaFDotPar) * rtb_Divide_bo +
    DTE_DeltaFDotPar;

  /* MultiPortSwitch: '<S174>/Multiport Switch1' incorporates:
   *  Constant: '<S174>/Constant2'
   *  Sum: '<S174>/Subtract1'
   *  UnitDelay: '<S174>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaFDotPar -
      TJATCT_DW.UnitDelay1_DSTATE_d;
  }

  /* End of MultiPortSwitch: '<S174>/Multiport Switch1' */

  /* MultiPortSwitch: '<S174>/Multiport Switch2' incorporates:
   *  Constant: '<S174>/IAM_Ts_P1'
   *  Constant: '<S174>/IAM_Ts_P11'
   *  Constant: '<S174>/IAM_Ts_P2'
   *  Constant: '<S174>/Parameter1'
   *  MinMax: '<S174>/Max1'
   *  MinMax: '<S174>/Min'
   *  RelationalOperator: '<S174>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S174>/Multiport Switch2' */

  /* Product: '<S174>/Divide1' */
  DTE_DeltaF2DotPar = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* MultiPortSwitch: '<S175>/Multiport Switch1' incorporates:
   *  Constant: '<S175>/Constant2'
   *  Sum: '<S175>/Subtract1'
   *  UnitDelay: '<S175>/Unit Delay1'
   */
  if (!DEV_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaF2DotPar -
      TJATCT_DW.UnitDelay1_DSTATE_cz;
  }

  /* End of MultiPortSwitch: '<S175>/Multiport Switch1' */

  /* MultiPortSwitch: '<S175>/Multiport Switch2' incorporates:
   *  Constant: '<S175>/IAM_Ts_P1'
   *  Constant: '<S175>/IAM_Ts_P11'
   *  Constant: '<S175>/IAM_Ts_P2'
   *  Constant: '<S175>/Parameter1'
   *  MinMax: '<S175>/Max1'
   *  MinMax: '<S175>/Min'
   *  RelationalOperator: '<S175>/GreaterThan'
   */
  if (DEV_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DEV_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DEV_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S175>/Multiport Switch2' */

  /* Product: '<S175>/Divide1' */
  DTE_DeltaF3DotPar = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Sum: '<S173>/Add1' incorporates:
   *  Product: '<S173>/Product1'
   *  Product: '<S173>/Product5'
   *  Product: '<S173>/Product6'
   *  Product: '<S173>/Product7'
   *  Sum: '<S173>/Add2'
   *  Sum: '<S173>/Add3'
   */
  DTE_ResDeltaDenPar = (DTE_CoeffDenS0LaDmc * DTE_DeltaFPar +
                        DTE_CoeffDenS1LaDmc * DTE_DeltaFDotPar) +
    (DTE_CoeffDenS2LaDmc * DTE_DeltaF2DotPar + DTE_CoeffDenS3LaDmc *
     DTE_DeltaF3DotPar);

  /* UnitDelay: '<S180>/Unit Delay1' */
  DEV_DlySetDeltaF2DotPar = TJATCT_DW.UnitDelay1_DSTATE_c3;

  /* UnitDelay: '<S180>/Unit Delay2' */
  DEV_DlySetDeltaFPar = TJATCT_DW.UnitDelay2_DSTATE_cu;

  /* UnitDelay: '<S180>/Unit Delay' */
  DEV_DlySetDeltaFDotPar = TJATCT_DW.UnitDelay_DSTATE_pzg;

  /* MultiPortSwitch: '<S179>/Multiport Switch' */
  if (!DEV_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S179>/Multiport Switch' incorporates:
     *  Constant: '<S179>/Constant17'
     */
    DEV_SetDeltaF3DotPar = 0.0F;
  } else {
    /* MultiPortSwitch: '<S179>/Multiport Switch' incorporates:
     *  Constant: '<S179>/Parameter1'
     *  Constant: '<S179>/Parameter2'
     *  Constant: '<S179>/Parameter3'
     *  Constant: '<S179>/Parameter4'
     *  Constant: '<S179>/Parameter5'
     *  Constant: '<S179>/Parameter6'
     *  Constant: '<S179>/Parameter7'
     *  Constant: '<S179>/Parameter8'
     *  Product: '<S179>/Divide'
     *  Product: '<S179>/Product'
     *  Product: '<S179>/Product1'
     *  Product: '<S179>/Product2'
     *  Product: '<S179>/Product3'
     *  Product: '<S179>/Product4'
     *  Product: '<S179>/Product5'
     *  Product: '<S179>/Product6'
     *  Product: '<S179>/Product7'
     *  Product: '<S179>/Product8'
     *  Sum: '<S179>/Add1'
     *  Sum: '<S179>/Add2'
     *  Sum: '<S179>/Add3'
     *  Sum: '<S179>/Add4'
     *  Sum: '<S179>/Subtract'
     *  Sum: '<S179>/Subtract1'
     *  Sum: '<S179>/Subtract2'
     */
    DEV_SetDeltaF3DotPar = (((DTE_ResDeltaDenPar - (DEV_Time1FltLaDmc_P *
      DEV_Time2FltLaDmc_P * DTE_CoeffNumS0LaDmc + (DEV_Time1FltLaDmc_P +
      DEV_Time2FltLaDmc_P) * DTE_CoeffNumS1LaDmc) * DEV_DlySetDeltaF2DotPar) -
      DTE_CoeffNumS0LaDmc * DEV_DlySetDeltaFPar) - ((DEV_Time1FltLaDmc_P +
      DEV_Time2FltLaDmc_P) * DTE_CoeffNumS0LaDmc + DTE_CoeffNumS1LaDmc) *
      DEV_DlySetDeltaFDotPar) / (DEV_Time1FltLaDmc_P * DEV_Time2FltLaDmc_P *
      DTE_CoeffNumS1LaDmc);
  }

  /* End of MultiPortSwitch: '<S179>/Multiport Switch' */

  /* Switch: '<S190>/Reset' */
  if (DEV_EnaResetByTgq) {
    /* Switch: '<S190>/Init' incorporates:
     *  Constant: '<S180>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_es = 0.0F;
  } else {
    /* Switch: '<S190>/Init' incorporates:
     *  Constant: '<S180>/Parameter1'
     *  Product: '<S183>/Product'
     *  Sum: '<S183>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_es = DEV_SetDeltaF3DotPar *
      DEV_TimeSysCycle_P + rtb_Switch2_m4;
  }

  /* End of Switch: '<S190>/Reset' */

  /* Sum: '<S181>/Add2' incorporates:
   *  Abs: '<S181>/Abs1'
   *  Abs: '<S181>/Abs2'
   *  Abs: '<S181>/Abs3'
   *  Constant: '<S181>/Constant1'
   *  Constant: '<S181>/Constant2'
   *  Constant: '<S181>/Constant4'
   *  RelationalOperator: '<S181>/GreaterThan1'
   *  RelationalOperator: '<S181>/GreaterThan2'
   *  RelationalOperator: '<S181>/GreaterThan3'
   *  Sum: '<S181>/Add1'
   */
  DEV_BtfFfcQualifierPar = (uint8_T)(((uint32_T)(fabsf(DEV_DlySetDeltaFPar) >=
    99.99F) + (fabsf(DEV_DlySetDeltaFDotPar) >= 99.99F)) + (fabsf
    (DEV_DlySetDeltaF2DotPar) >= 99.99F));

  /* Sum: '<S158>/Add2' incorporates:
   *  Abs: '<S158>/Abs1'
   *  Abs: '<S158>/Abs2'
   *  Abs: '<S158>/Abs3'
   *  Constant: '<S158>/Constant1'
   *  Constant: '<S158>/Constant2'
   *  Constant: '<S158>/Constant4'
   *  RelationalOperator: '<S158>/GreaterThan1'
   *  RelationalOperator: '<S158>/GreaterThan2'
   *  RelationalOperator: '<S158>/GreaterThan3'
   *  Sum: '<S158>/Add1'
   */
  DEV_BtfFfcQualifierRte = (uint8_T)(((uint32_T)(fabsf(DEV_DlySetDeltaFRte) >=
    99.99F) + (fabsf(DEV_DlySetDeltaFDotRte) >= 99.99F)) + (fabsf
    (DEV_DlySetDeltaF2DotRte) >= 99.99F));

  /* DataTypeConversion: '<S81>/Data Type Conversion17' */
  LGC_DeltaFBAC_deg = LGC_DeltaByBnkAglComp_deg;

  /* Product: '<S271>/Product2' incorporates:
   *  Constant: '<S271>/Parameter5'
   *  Constant: '<S271>/Parameter9'
   *  Product: '<S273>/Product4'
   */
  rtb_Divide_bo = DTE_CorStiffFrontAxle_P * DTE_CorStiffRearAxle_P;

  /* Product: '<S271>/Product1' incorporates:
   *  Constant: '<S271>/Parameter7'
   *  Constant: '<S271>/Parameter8'
   *  Product: '<S271>/Product2'
   *  Product: '<S271>/Product7'
   *  Product: '<S271>/Product8'
   */
  DTE_CoeffA0TranferFcn = rtb_Divide_bo * DTE_LmtVehVelX *
    (DTE_DistCogToFrontAxle_P * DTE_DistCogToRearAxle_P);

  /* MultiPortSwitch: '<S280>/Multiport Switch' incorporates:
   *  Constant: '<S280>/Constant1'
   *  Inport: '<Root>/TCTI_VehYawRate'
   *  Sum: '<S280>/Subtract'
   *  UnitDelay: '<S280>/Unit Delay'
   */
  if (!DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = TCTI_VehYawRate - TJATCT_DW.UnitDelay_DSTATE_hr;
  }

  /* End of MultiPortSwitch: '<S280>/Multiport Switch' */

  /* MultiPortSwitch: '<S280>/Multiport Switch1' incorporates:
   *  Constant: '<S280>/IAM_Ts_P1'
   *  Constant: '<S280>/IAM_Ts_P11'
   *  Constant: '<S280>/IAM_Ts_P2'
   *  Constant: '<S280>/Parameter'
   *  MinMax: '<S280>/Max1'
   *  MinMax: '<S280>/Min'
   *  RelationalOperator: '<S280>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S280>/Multiport Switch1' */

  /* Product: '<S280>/Divide' */
  DTE_Psi2DotVdyFcn = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Product: '<S272>/Product9' incorporates:
   *  Constant: '<S272>/Parameter2'
   *  Constant: '<S272>/Parameter8'
   *  Constant: '<S272>/Parameter9'
   *  Product: '<S272>/Product6'
   *  Product: '<S272>/Product8'
   */
  DTE_CoeffA1TranferFcn = DTE_RawReqDeltaByBnkAgl_tmp * DTE_CorStiffFrontAxle_P *
    (DTE_DistCogToFrontAxle_P * DTE_MassVehicle_P);

  /* Product: '<S275>/Product9' incorporates:
   *  Constant: '<S275>/Parameter2'
   *  Constant: '<S275>/Parameter6'
   *  Product: '<S275>/Product3'
   */
  DTE_CoeffB2TranferFcn = DTE_RawReqDeltaByBnkAgl_tmp * (DTE_MassVehicle_P *
    DTE_InertiaVehicle_P);

  /* MultiPortSwitch: '<S279>/Multiport Switch1' incorporates:
   *  Constant: '<S279>/Constant2'
   *  Sum: '<S279>/Subtract1'
   *  UnitDelay: '<S279>/Unit Delay1'
   */
  if (!DTE_EnaResetByTgq) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_Psi2DotVdyFcn -
      TJATCT_DW.UnitDelay1_DSTATE_j;
  }

  /* End of MultiPortSwitch: '<S279>/Multiport Switch1' */

  /* MultiPortSwitch: '<S279>/Multiport Switch2' incorporates:
   *  Constant: '<S279>/IAM_Ts_P1'
   *  Constant: '<S279>/IAM_Ts_P11'
   *  Constant: '<S279>/IAM_Ts_P2'
   *  Constant: '<S279>/Parameter1'
   *  MinMax: '<S279>/Max1'
   *  MinMax: '<S279>/Min'
   *  RelationalOperator: '<S279>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S279>/Multiport Switch2' */

  /* Product: '<S279>/Divide1' */
  DTE_Psi3DotVdyFcn = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Sum: '<S281>/Add1' incorporates:
   *  Inport: '<Root>/TCTI_VehYawRate'
   *  Product: '<S281>/Product5'
   *  Product: '<S281>/Product6'
   *  Product: '<S281>/Product7'
   *  Sum: '<S281>/Add2'
   */
  DTE_ResDeltaDenVdyFcn = (DTE_CoeffA1TranferFcn * TCTI_VehYawRate +
    DTE_CoeffA0TranferFcn * DTE_Psi2DotVdyFcn) + DTE_Psi3DotVdyFcn *
    DTE_CoeffB2TranferFcn;

  /* Sum: '<S273>/Add' incorporates:
   *  Constant: '<S273>/Parameter10'
   *  Constant: '<S273>/Parameter11'
   *  Constant: '<S273>/Parameter12'
   *  Constant: '<S273>/Parameter13'
   *  Constant: '<S273>/Parameter6'
   *  Product: '<S273>/Product'
   *  Product: '<S273>/Product10'
   *  Product: '<S273>/Product11'
   *  Product: '<S273>/Product3'
   *  Product: '<S273>/Product5'
   *  Product: '<S273>/Product9'
   *  Sum: '<S273>/Subtract'
   */
  DTE_CoeffB0TranferFcn = DTE_RawReqDeltaByBnkAgl_tmp * DTE_MassVehicle_P *
    (DTE_CorStiffRearAxle_P * DTE_DistCogToRearAxle_P - DTE_CorStiffFrontAxle_P *
     DTE_DistCogToFrontAxle_P) + DTE_RawReqDeltaByBnkAgl_tmp_0 *
    DTE_RawReqDeltaByBnkAgl_tmp_0 * rtb_Divide_bo;

  /* Sum: '<S274>/Add1' incorporates:
   *  Constant: '<S274>/Parameter1'
   *  Constant: '<S274>/Parameter10'
   *  Constant: '<S274>/Parameter4'
   *  Constant: '<S274>/Parameter5'
   *  Constant: '<S274>/Parameter6'
   *  Constant: '<S274>/Parameter8'
   *  Product: '<S274>/Product'
   *  Product: '<S274>/Product10'
   *  Product: '<S274>/Product11'
   *  Product: '<S274>/Product3'
   *  Product: '<S274>/Product4'
   *  Product: '<S274>/Product5'
   *  Product: '<S274>/Product6'
   *  Product: '<S274>/Product9'
   *  Sum: '<S274>/Add2'
   */
  DTE_CoeffB1TranferFcn = (DTE_DistCogToFrontAxle_P * DTE_DistCogToFrontAxle_P *
    DTE_CorStiffFrontAxle_P + DTE_DistCogToRearAxle_P * DTE_DistCogToRearAxle_P *
    DTE_CorStiffRearAxle_P) * (DTE_LmtVehVelX * DTE_MassVehicle_P) +
    DTE_LmtVehVelX * DTE_InertiaVehicle_P * DTE_RawReqDeltaByBnkAgl_tmp_0;

  /* UnitDelay: '<S285>/Unit Delay1' */
  DTE_DlyDeltaDotVdyFcn = TJATCT_DW.UnitDelay1_DSTATE_fq;

  /* UnitDelay: '<S285>/Unit Delay' */
  DTE_DlyDeltaVdyFcn = TJATCT_DW.UnitDelay_DSTATE_c2;

  /* MultiPortSwitch: '<S284>/Multiport Switch' */
  if (!DTE_EnaCtrlByTgq) {
    /* MultiPortSwitch: '<S284>/Multiport Switch' incorporates:
     *  Constant: '<S284>/Constant17'
     */
    DTE_Delta2DotVdyFcn = 0.0F;
  } else {
    /* MultiPortSwitch: '<S284>/Multiport Switch' incorporates:
     *  Constant: '<S284>/Parameter1'
     *  Constant: '<S284>/Parameter6'
     *  Product: '<S284>/Divide'
     *  Product: '<S284>/Product'
     *  Product: '<S284>/Product2'
     *  Product: '<S284>/Product4'
     *  Product: '<S284>/Product5'
     *  Sum: '<S284>/Add1'
     *  Sum: '<S284>/Subtract'
     *  Sum: '<S284>/Subtract1'
     */
    DTE_Delta2DotVdyFcn = ((DTE_ResDeltaDenVdyFcn - (DTE_CoeffB1TranferFcn *
      DTE_TiFltSteerAngle_P + DTE_CoeffB0TranferFcn) * DTE_DlyDeltaDotVdyFcn) -
      DTE_CoeffB1TranferFcn * DTE_DlyDeltaVdyFcn) / (DTE_CoeffB0TranferFcn *
      DTE_TiFltSteerAngle_P);
  }

  /* End of MultiPortSwitch: '<S284>/Multiport Switch' */

  /* Switch: '<S293>/Init' incorporates:
   *  Logic: '<S293>/FixPt Logical Operator'
   *  UnitDelay: '<S293>/FixPt Unit Delay2'
   */
  if (DTE_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_n != 0)) {
    /* Switch: '<S293>/Init' incorporates:
     *  Constant: '<S285>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ox = 0.0F;
  }

  /* End of Switch: '<S293>/Init' */

  /* Switch: '<S291>/Switch2' incorporates:
   *  Constant: '<S285>/Constant1'
   *  Constant: '<S285>/Constant2'
   *  RelationalOperator: '<S291>/LowerRelop1'
   *  RelationalOperator: '<S291>/UpperRelop'
   *  Switch: '<S291>/Switch'
   *  UnitDelay: '<S285>/Unit Delay1'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ox > 10000.0F) {
    TJATCT_DW.UnitDelay1_DSTATE_fq = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_ox < -10000.0F) {
    /* Switch: '<S291>/Switch' incorporates:
     *  Constant: '<S285>/Constant2'
     *  UnitDelay: '<S285>/Unit Delay1'
     */
    TJATCT_DW.UnitDelay1_DSTATE_fq = -10000.0F;
  } else {
    TJATCT_DW.UnitDelay1_DSTATE_fq = TJATCT_DW.FixPtUnitDelay1_DSTATE_ox;
  }

  /* End of Switch: '<S291>/Switch2' */

  /* Switch: '<S293>/Reset' incorporates:
   *  Switch: '<S290>/Reset'
   */
  if (DTE_EnaResetByTgq) {
    /* Switch: '<S293>/Init' incorporates:
     *  Constant: '<S285>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ox = 0.0F;

    /* Switch: '<S290>/Init' incorporates:
     *  Constant: '<S285>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ov = 0.0F;
  } else {
    /* Switch: '<S293>/Init' incorporates:
     *  Constant: '<S285>/Parameter1'
     *  Product: '<S287>/Product'
     *  Sum: '<S287>/Subtract'
     *  UnitDelay: '<S285>/Unit Delay1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ox = DTE_Delta2DotVdyFcn *
      DTE_TimeSysCycle_P + TJATCT_DW.UnitDelay1_DSTATE_fq;

    /* Switch: '<S290>/Init' incorporates:
     *  Constant: '<S285>/Parameter'
     *  Product: '<S286>/Product'
     *  Sum: '<S286>/Subtract'
     *  UnitDelay: '<S285>/Unit Delay1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_ov = TJATCT_DW.UnitDelay1_DSTATE_fq *
      DTE_TimeSysCycle_P + DTE_DeltaVdyFcn;
  }

  /* End of Switch: '<S293>/Reset' */

  /* DataTypeConversion: '<S81>/Data Type Conversion5' */
  LGC_CrvReqDte_1pm = DTE_ReqCrvByDstrb;

  /* Switch: '<S237>/Init' incorporates:
   *  Logic: '<S237>/FixPt Logical Operator'
   *  UnitDelay: '<S237>/FixPt Unit Delay2'
   */
  if ((DTE_DeltaByVdyFcn != 0.0F) || (TJATCT_DW.FixPtUnitDelay2_DSTATE_d != 0))
  {
    /* Switch: '<S237>/Init' incorporates:
     *  Constant: '<S225>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_oi = 0.0F;
  }

  /* End of Switch: '<S237>/Init' */

  /* Switch: '<S235>/Switch2' incorporates:
   *  Constant: '<S225>/Constant1'
   *  Constant: '<S225>/Constant2'
   *  RelationalOperator: '<S235>/LowerRelop1'
   *  RelationalOperator: '<S235>/UpperRelop'
   *  Switch: '<S235>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_oi > 10000.0F) {
    rtb_Divide_bo = 10000.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_oi < -10000.0F) {
    /* Switch: '<S235>/Switch' incorporates:
     *  Constant: '<S225>/Constant2'
     */
    rtb_Divide_bo = -10000.0F;
  } else {
    rtb_Divide_bo = TJATCT_DW.FixPtUnitDelay1_DSTATE_oi;
  }

  /* End of Switch: '<S235>/Switch2' */

  /* Switch: '<S234>/Reset' */
  if (DTE_DeltaByVdyFcn != 0.0F) {
    /* Switch: '<S234>/Init' incorporates:
     *  Constant: '<S225>/Constant4'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cn = 0.0F;
  } else {
    /* Switch: '<S234>/Init' incorporates:
     *  Constant: '<S225>/Parameter'
     *  Product: '<S230>/Product'
     *  Sum: '<S230>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cn = rtb_Divide_bo * DTE_TimeSysCycle_P +
      DTE_SetCrvLaDmc;
  }

  /* End of Switch: '<S234>/Reset' */

  /* MultiPortSwitch: '<S228>/Multiport Switch' incorporates:
   *  Constant: '<S228>/Constant1'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Sum: '<S228>/Subtract'
   *  UnitDelay: '<S228>/Unit Delay'
   */
  if ((int32_T)DTE_DeltaByVdyFcn == 0) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = TCTI_VehicleVelX -
      TJATCT_DW.UnitDelay_DSTATE_hn;
  }

  /* End of MultiPortSwitch: '<S228>/Multiport Switch' */

  /* MultiPortSwitch: '<S228>/Multiport Switch1' incorporates:
   *  Constant: '<S228>/IAM_Ts_P1'
   *  Constant: '<S228>/IAM_Ts_P11'
   *  Constant: '<S228>/IAM_Ts_P2'
   *  Constant: '<S228>/Parameter'
   *  MinMax: '<S228>/Max1'
   *  MinMax: '<S228>/Min'
   *  RelationalOperator: '<S228>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S228>/Multiport Switch1' */

  /* Product: '<S228>/Divide' */
  DTE_DeltaDotForCrv = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* MultiPortSwitch: '<S227>/Multiport Switch1' incorporates:
   *  Constant: '<S227>/Constant2'
   *  Sum: '<S227>/Subtract1'
   *  UnitDelay: '<S227>/Unit Delay1'
   */
  if ((int32_T)DTE_DeltaByVdyFcn == 0) {
    LGC_RawErrCourseDotCdc_tmp = 0.0F;
  } else {
    LGC_RawErrCourseDotCdc_tmp = DTE_DeltaDotForCrv -
      TJATCT_DW.UnitDelay1_DSTATE_k;
  }

  /* End of MultiPortSwitch: '<S227>/Multiport Switch1' */

  /* MultiPortSwitch: '<S227>/Multiport Switch2' incorporates:
   *  Constant: '<S227>/IAM_Ts_P1'
   *  Constant: '<S227>/IAM_Ts_P11'
   *  Constant: '<S227>/IAM_Ts_P2'
   *  Constant: '<S227>/Parameter1'
   *  MinMax: '<S227>/Max1'
   *  MinMax: '<S227>/Min'
   *  RelationalOperator: '<S227>/GreaterThan'
   */
  if (DTE_TimeSysCycle_P < 0.0F) {
    rtb_Divide_l_tmp = fminf(DTE_TimeSysCycle_P, -0.001F);
  } else {
    rtb_Divide_l_tmp = fmaxf(DTE_TimeSysCycle_P, 0.001F);
  }

  /* End of MultiPortSwitch: '<S227>/Multiport Switch2' */

  /* Product: '<S227>/Divide1' */
  DTE_Delta2DotForCrv = LGC_RawErrCourseDotCdc_tmp / rtb_Divide_l_tmp;

  /* Sum: '<S226>/Add3' incorporates:
   *  Constant: '<S226>/Parameter6'
   *  Constant: '<S226>/Parameter7'
   *  Constant: '<S226>/Parameter8'
   *  Constant: '<S226>/Parameter9'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Product: '<S226>/Product1'
   *  Product: '<S226>/Product2'
   *  Product: '<S226>/Product3'
   *  Sum: '<S226>/Add4'
   *  Sum: '<S226>/Add5'
   */
  DTE_ResCrvDenLaDmc = ((DTE_Time1LaDmc_P + DTE_Time2LaDmc_P) *
                        DTE_DeltaDotForCrv + TCTI_VehicleVelX) +
    DTE_Time1LaDmc_P * DTE_Time2LaDmc_P * DTE_Delta2DotForCrv;

  /* UnitDelay: '<S225>/Unit Delay' */
  DTE_DlySetCrvLaDmc = TJATCT_DW.UnitDelay_DSTATE_h4;

  /* Lookup_n-D: '<S229>/1-D Lookup Table' incorporates:
   *  Constant: '<S229>/Constant1'
   *  Product: '<S229>/Product'
   */
  DTE_KappaAngleLaDmc = look1_iflf_binlxpw(DTE_EnaCtrlByTgq ? 3.6F : 0.0F, ((
    const real32_T *)&(DTE_KappaAngleLaDmc_X[0])), ((const real32_T *)
    &(DTE_KappaAngleLaDmc_M[0])), 11U);

  /* Product: '<S229>/Product1' incorporates:
   *  Constant: '<S229>/Constant'
   *  Constant: '<S229>/Constant2'
   *  Product: '<S229>/Divide'
   */
  DTE_RawCrvLaDmc = DTE_KappaAngleLaDmc / 3.14159F * 180.0F;

  /* UnitDelay: '<S225>/Unit Delay1' */
  DTE_DlySetCrvDotLaDmc = TJATCT_DW.UnitDelay1_DSTATE_ay;

  /* MultiPortSwitch: '<S224>/Multiport Switch' */
  if (!DTE_EnaResetByTgq) {
    /* MultiPortSwitch: '<S224>/Multiport Switch' incorporates:
     *  Constant: '<S224>/Constant17'
     */
    DTE_SetCrv2DotLaDmc = 0.0F;
  } else {
    /* MultiPortSwitch: '<S224>/Multiport Switch' incorporates:
     *  Constant: '<S224>/Parameter2'
     *  Constant: '<S224>/Parameter3'
     *  Constant: '<S224>/Parameter4'
     *  Constant: '<S224>/Parameter5'
     *  Product: '<S224>/Divide'
     *  Product: '<S224>/Product'
     *  Product: '<S224>/Product1'
     *  Product: '<S224>/Product2'
     *  Product: '<S224>/Product3'
     *  Product: '<S224>/Product4'
     *  Sum: '<S224>/Add'
     *  Sum: '<S224>/Subtract'
     *  Sum: '<S224>/Subtract1'
     */
    DTE_SetCrv2DotLaDmc = ((DTE_ResCrvDenLaDmc - (DTE_Time1FltLaDmc_P +
      DTE_Time2FltLaDmc_P) * DTE_DlySetCrvLaDmc * DTE_RawCrvLaDmc) -
      DTE_RawCrvLaDmc * DTE_DlySetCrvDotLaDmc) / (DTE_Time1FltLaDmc_P *
      DTE_Time2FltLaDmc_P * DTE_RawCrvLaDmc);
  }

  /* End of MultiPortSwitch: '<S224>/Multiport Switch' */

  /* Switch: '<S237>/Reset' */
  if (DTE_DeltaByVdyFcn != 0.0F) {
    /* Switch: '<S237>/Init' incorporates:
     *  Constant: '<S225>/Constant16'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_oi = 0.0F;
  } else {
    /* Switch: '<S237>/Init' incorporates:
     *  Constant: '<S225>/Parameter1'
     *  Product: '<S231>/Product'
     *  Sum: '<S231>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_oi = DTE_SetCrv2DotLaDmc *
      DTE_TimeSysCycle_P + rtb_Divide_bo;
  }

  /* End of Switch: '<S237>/Reset' */

  /* Gain: '<S81>/Gain' */
  data_log_0 = LGC_DeltaFCmd_rad;

  /* DataTypeConversion: '<S81>/Data Type Conversion4' */
  LGC_CdcCmd_rad = LGC_DeltaFCmdCdc;

  /* Switch: '<S494>/Reset' incorporates:
   *  Logic: '<S487>/NOT3'
   *  Switch: '<S493>/Enable'
   */
  if (rtb_OR_km_tmp) {
    /* Switch: '<S494>/Init' incorporates:
     *  Constant: '<S487>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_i = 0.0F;
  } else {
    if (!LGC_EnaFreezeByTgq) {
      /* Switch: '<S494>/Init' incorporates:
       *  Constant: '<S487>/Parameter4'
       *  Inport: '<Root>/Outport41'
       *  Product: '<S487>/Mul5'
       *  Product: '<S491>/Product'
       *  Sum: '<S491>/Subtract'
       *  Switch: '<S493>/Enable'
       *  UnitDelay: '<S494>/FixPt Unit Delay1'
       */
      TJATCT_DW.FixPtUnitDelay1_DSTATE_i = LGC_DeltaFPGainCdc *
        Test_CoeffIGainCac * LGC_TimeSysCycle_P + LGC_DeltaFIGainCdc;
    }
  }

  /* End of Switch: '<S494>/Reset' */

  /* Switch: '<S517>/Init' incorporates:
   *  Logic: '<S517>/FixPt Logical Operator'
   *  UnitDelay: '<S517>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_fb != 0)) {
    /* Switch: '<S517>/Init' incorporates:
     *  Constant: '<S507>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_os = 0.0F;
  }

  /* End of Switch: '<S517>/Init' */

  /* Switch: '<S515>/Switch2' incorporates:
   *  Constant: '<S507>/Constant7'
   *  Constant: '<S507>/Constant8'
   *  RelationalOperator: '<S515>/LowerRelop1'
   *  RelationalOperator: '<S515>/UpperRelop'
   *  Switch: '<S515>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_os > 100.0F) {
    /* Switch: '<S515>/Switch2' */
    LGC_CmpnDotLaDmcCdc = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_os < -100.0F) {
    /* Switch: '<S515>/Switch' incorporates:
     *  Constant: '<S507>/Constant8'
     *  Switch: '<S515>/Switch2'
     */
    LGC_CmpnDotLaDmcCdc = -100.0F;
  } else {
    /* Switch: '<S515>/Switch2' incorporates:
     *  Switch: '<S515>/Switch'
     */
    LGC_CmpnDotLaDmcCdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_os;
  }

  /* End of Switch: '<S515>/Switch2' */

  /* Switch: '<S514>/Reset' */
  if (LGC_EnaResetByTgq) {
    /* Switch: '<S514>/Init' incorporates:
     *  Constant: '<S507>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d = 0.0F;
  } else {
    /* Switch: '<S514>/Init' incorporates:
     *  Constant: '<S507>/Parameter9'
     *  Product: '<S510>/Product'
     *  Sum: '<S510>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d = LGC_CmpnDotLaDmcCdc *
      LGC_TimeSysCycle_P + LGC_CmpnLaDmcCdc;
  }

  /* End of Switch: '<S514>/Reset' */

  /* Lookup_n-D: '<S425>/1-D Lookup Table2' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPole1LaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPole1LaDmc_X[0])), ((const real32_T *)&(LGC_CoeffPole1LaDmc_M[0])),
    14U);

  /* Lookup_n-D: '<S425>/1-D Lookup Table3' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPole2LaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPole2LaDmc_X[0])), ((const real32_T *)&(LGC_CoeffPole2LaDmc_M[0])),
    14U);

  /* Product: '<S508>/Divide' incorporates:
   *  Constant: '<S504>/Constant'
   *  Constant: '<S504>/Parameter'
   *  MinMax: '<S508>/Max1'
   *  Product: '<S468>/Divide'
   *  Product: '<S550>/Divide'
   */
  rtb_Divide_l_tmp = LGC_TimeSysCycle_P / fmaxf(LGC_TimeSysCycle_P, 0.03F);

  /* MinMax: '<S504>/Min' incorporates:
   *  Constant: '<S504>/Constant2'
   *  Constant: '<S504>/Parameter1'
   *  MinMax: '<S464>/Min'
   *  MinMax: '<S465>/Min'
   *  MinMax: '<S505>/Min'
   *  MinMax: '<S546>/Min'
   *  MinMax: '<S547>/Min'
   *  MultiPortSwitch: '<S465>/Multiport Switch'
   *  MultiPortSwitch: '<S505>/Multiport Switch'
   *  MultiPortSwitch: '<S547>/Multiport Switch'
   */
  LGC_RawErrCourseDotCdc_tmp = fminf(LGC_TimeSysCycle_P, 0.001F);

  /* Product: '<S504>/Divide' incorporates:
   *  MinMax: '<S504>/Min'
   *  Sum: '<S504>/Sum4'
   *  UnitDelay: '<S504>/Unit_Delay3'
   */
  LGC_RawErrCourseDotCdc = (LGC_FltErrCourseCdc - TJATCT_DW.Unit_Delay3_DSTATE_k)
    / LGC_RawErrCourseDotCdc_tmp;

  /* Switch: '<S509>/Switch2' incorporates:
   *  Constant: '<S508>/IAM_Ts_P1'
   *  Constant: '<S508>/IAM_Ts_P4'
   *  Product: '<S508>/Divide'
   *  RelationalOperator: '<S509>/LowerRelop1'
   *  RelationalOperator: '<S509>/UpperRelop'
   *  Switch: '<S509>/Switch'
   */
  if (rtb_Divide_l_tmp > 1.0F) {
    DTE_RawReqDeltaByBnkAgl_tmp_0 = 1.0F;
  } else if (rtb_Divide_l_tmp < 0.0F) {
    /* Switch: '<S509>/Switch' incorporates:
     *  Constant: '<S508>/IAM_Ts_P1'
     */
    DTE_RawReqDeltaByBnkAgl_tmp_0 = 0.0F;
  } else {
    DTE_RawReqDeltaByBnkAgl_tmp_0 = rtb_Divide_l_tmp;
  }

  /* End of Switch: '<S509>/Switch2' */

  /* Sum: '<S508>/Add' incorporates:
   *  Product: '<S508>/Product'
   *  Sum: '<S508>/Subtract'
   */
  LGC_FltErrCourseDotCdc = (LGC_RawErrCourseDotCdc - LGC_FltErrCourseDotCdc) *
    DTE_RawReqDeltaByBnkAgl_tmp_0 + LGC_FltErrCourseDotCdc;

  /* MultiPortSwitch: '<S504>/Multiport Switch' incorporates:
   *  MultiPortSwitch: '<S505>/Multiport Switch'
   *  RelationalOperator: '<S504>/Equal'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S504>/Multiport Switch' incorporates:
     *  Constant: '<S504>/Constant3'
     */
    LGC_ErrCourseDotCdc = 0.0F;

    /* MultiPortSwitch: '<S505>/Multiport Switch' incorporates:
     *  Constant: '<S505>/Constant3'
     */
    LGC_ErrCourse2DotCdc = 0.0F;
  } else {
    /* MultiPortSwitch: '<S504>/Multiport Switch' */
    LGC_ErrCourseDotCdc = LGC_FltErrCourseDotCdc;

    /* MultiPortSwitch: '<S505>/Multiport Switch' incorporates:
     *  Product: '<S505>/Divide'
     *  Sum: '<S505>/Sum1'
     *  UnitDelay: '<S505>/Unit_Delay1'
     */
    LGC_ErrCourse2DotCdc = (LGC_ErrCourseDotCdc - TJATCT_DW.Unit_Delay1_DSTATE_f)
      / LGC_RawErrCourseDotCdc_tmp;
  }

  /* End of MultiPortSwitch: '<S504>/Multiport Switch' */

  /* Lookup_n-D: '<S425>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffNumS1LaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffNumS1LaDmc_X[0])), ((const real32_T *)&(LGC_CoeffNumS1LaDmc_M[0])),
    14U);

  /* Lookup_n-D: '<S425>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffNumS0LaDmc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffNumS0LaDmc_X[0])), ((const real32_T *)&(LGC_CoeffNumS0LaDmc_M[0])),
    14U);

  /* Sum: '<S506>/Sum5' incorporates:
   *  Sum: '<S466>/Sum5'
   *  Sum: '<S548>/Sum5'
   *  UnaryMinus: '<S506>/Neg'
   *  UnaryMinus: '<S506>/Neg1'
   */
  DTE_RawReqDeltaByBnkAgl_tmp_0 = -LGC_CoeffPole1LaDmc + -LGC_CoeffPole2LaDmc;

  /* Product: '<S506>/Mul4' incorporates:
   *  Product: '<S466>/Mul4'
   *  Product: '<S548>/Mul4'
   *  UnaryMinus: '<S506>/Neg'
   *  UnaryMinus: '<S506>/Neg1'
   */
  DTE_RawReqDeltaByBnkAgl_tmp = -LGC_CoeffPole1LaDmc * -LGC_CoeffPole2LaDmc;

  /* Sum: '<S506>/Sum2' incorporates:
   *  Product: '<S506>/Mul1'
   *  Product: '<S506>/Mul2'
   *  Product: '<S506>/Mul3'
   *  Product: '<S506>/Mul4'
   *  Product: '<S506>/Mul9'
   *  Sum: '<S506>/Sum1'
   *  Sum: '<S506>/Sum3'
   *  Sum: '<S506>/Sum4'
   *  Sum: '<S506>/Sum5'
   *  UnitDelay: '<S502>/Unit Delay'
   *  UnitDelay: '<S502>/Unit Delay1'
   */
  LGC_RawCmpn2DotLaDmcCdc = (((LGC_CoeffNumS0LaDmc * LGC_FltErrCourseCdc +
    LGC_CoeffNumS1LaDmc * LGC_ErrCourseDotCdc) + LGC_ErrCourse2DotCdc) -
    DTE_RawReqDeltaByBnkAgl_tmp_0 * TJATCT_DW.UnitDelay1_DSTATE_fc) -
    DTE_RawReqDeltaByBnkAgl_tmp * TJATCT_DW.UnitDelay_DSTATE_nu;

  /* MultiPortSwitch: '<S506>/Multiport Switch' */
  if (!LGC_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S506>/Multiport Switch' incorporates:
     *  Constant: '<S506>/Constant2'
     */
    LGC_Cmpn2DotLaDmcCdc = 0.0F;
  } else {
    /* MultiPortSwitch: '<S506>/Multiport Switch' */
    LGC_Cmpn2DotLaDmcCdc = LGC_RawCmpn2DotLaDmcCdc;
  }

  /* End of MultiPortSwitch: '<S506>/Multiport Switch' */

  /* Switch: '<S517>/Reset' */
  if (LGC_EnaResetByTgq) {
    /* Switch: '<S517>/Init' incorporates:
     *  Constant: '<S507>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_os = 0.0F;
  } else {
    /* Switch: '<S517>/Init' incorporates:
     *  Constant: '<S507>/Parameter1'
     *  Product: '<S511>/Product'
     *  Sum: '<S511>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_os = LGC_Cmpn2DotLaDmcCdc *
      LGC_TimeSysCycle_P + LGC_CmpnDotLaDmcCdc;
  }

  /* End of Switch: '<S517>/Reset' */

  /* Product: '<S546>/Divide' incorporates:
   *  Sum: '<S546>/Sum4'
   *  UnitDelay: '<S546>/Unit_Delay3'
   */
  LGC_RawErrDistYDot = (LGC_FltPT1YErr_met - TJATCT_DW.Unit_Delay3_DSTATE_kl) /
    LGC_RawErrCourseDotCdc_tmp;

  /* Switch: '<S551>/Switch2' incorporates:
   *  Constant: '<S550>/IAM_Ts_P1'
   *  Constant: '<S550>/IAM_Ts_P4'
   *  RelationalOperator: '<S551>/LowerRelop1'
   *  RelationalOperator: '<S551>/UpperRelop'
   *  Switch: '<S551>/Switch'
   */
  if (rtb_Divide_l_tmp > 1.0F) {
    DTE_DeltaFRte_tmp = 1.0F;
  } else if (rtb_Divide_l_tmp < 0.0F) {
    /* Switch: '<S551>/Switch' incorporates:
     *  Constant: '<S550>/IAM_Ts_P1'
     */
    DTE_DeltaFRte_tmp = 0.0F;
  } else {
    DTE_DeltaFRte_tmp = rtb_Divide_l_tmp;
  }

  /* End of Switch: '<S551>/Switch2' */

  /* Sum: '<S550>/Add' incorporates:
   *  Product: '<S550>/Product'
   *  Sum: '<S550>/Subtract'
   */
  LGC_FltRawErrDistYDot = (LGC_RawErrDistYDot - LGC_FltRawErrDistYDot) *
    DTE_DeltaFRte_tmp + LGC_FltRawErrDistYDot;

  /* MultiPortSwitch: '<S546>/Multiport Switch' incorporates:
   *  MultiPortSwitch: '<S547>/Multiport Switch'
   *  RelationalOperator: '<S546>/Equal'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S546>/Multiport Switch' incorporates:
     *  Constant: '<S546>/Constant3'
     */
    LGC_ErrDistYDot = 0.0F;

    /* MultiPortSwitch: '<S547>/Multiport Switch' incorporates:
     *  Constant: '<S547>/Constant3'
     */
    LGC_ErrDistY2Dot = 0.0F;
  } else {
    /* MultiPortSwitch: '<S546>/Multiport Switch' */
    LGC_ErrDistYDot = LGC_FltRawErrDistYDot;

    /* MultiPortSwitch: '<S547>/Multiport Switch' incorporates:
     *  Product: '<S547>/Divide'
     *  Sum: '<S547>/Sum1'
     *  UnitDelay: '<S547>/Unit_Delay1'
     */
    LGC_ErrDistY2Dot = (LGC_ErrDistYDot - TJATCT_DW.Unit_Delay1_DSTATE_a2) /
      LGC_RawErrCourseDotCdc_tmp;
  }

  /* End of MultiPortSwitch: '<S546>/Multiport Switch' */

  /* Switch: '<S559>/Init' incorporates:
   *  Logic: '<S559>/FixPt Logical Operator'
   *  UnitDelay: '<S559>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_h != 0)) {
    /* Switch: '<S559>/Init' incorporates:
     *  Constant: '<S549>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cj = 0.0F;
  }

  /* End of Switch: '<S559>/Init' */

  /* Switch: '<S557>/Switch2' incorporates:
   *  Constant: '<S549>/Constant7'
   *  Constant: '<S549>/Constant8'
   *  RelationalOperator: '<S557>/LowerRelop1'
   *  RelationalOperator: '<S557>/UpperRelop'
   *  Switch: '<S557>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cj > 100.0F) {
    /* Switch: '<S557>/Switch2' */
    LGC_CmpnDotLaDmcLdc = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_cj < -100.0F) {
    /* Switch: '<S557>/Switch' incorporates:
     *  Constant: '<S549>/Constant8'
     *  Switch: '<S557>/Switch2'
     */
    LGC_CmpnDotLaDmcLdc = -100.0F;
  } else {
    /* Switch: '<S557>/Switch2' incorporates:
     *  Switch: '<S557>/Switch'
     */
    LGC_CmpnDotLaDmcLdc = TJATCT_DW.FixPtUnitDelay1_DSTATE_cj;
  }

  /* End of Switch: '<S557>/Switch2' */

  /* Sum: '<S548>/Sum2' incorporates:
   *  Product: '<S548>/Mul1'
   *  Product: '<S548>/Mul2'
   *  Product: '<S548>/Mul3'
   *  Product: '<S548>/Mul9'
   *  Sum: '<S548>/Sum1'
   *  Sum: '<S548>/Sum3'
   *  Sum: '<S548>/Sum4'
   */
  LGC_RawCmpn2DotLaDmcLdc = (((LGC_CoeffNumS0LaDmc * LGC_FltPT1YErr_met +
    LGC_CoeffNumS1LaDmc * LGC_ErrDistYDot) + LGC_ErrDistY2Dot) -
    DTE_RawReqDeltaByBnkAgl_tmp_0 * LGC_CmpnDotLaDmcLdc) -
    DTE_RawReqDeltaByBnkAgl_tmp * LGC_CmpnLaDmcLdc;

  /* MultiPortSwitch: '<S548>/Multiport Switch' */
  if (!LGC_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S548>/Multiport Switch' incorporates:
     *  Constant: '<S548>/Constant2'
     */
    LGC_Cmpn2DotLaDmcLdc = 0.0F;
  } else {
    /* MultiPortSwitch: '<S548>/Multiport Switch' */
    LGC_Cmpn2DotLaDmcLdc = LGC_RawCmpn2DotLaDmcLdc;
  }

  /* End of MultiPortSwitch: '<S548>/Multiport Switch' */

  /* Switch: '<S559>/Reset' incorporates:
   *  Switch: '<S556>/Reset'
   */
  if (LGC_EnaResetByTgq) {
    /* Switch: '<S559>/Init' incorporates:
     *  Constant: '<S549>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cj = 0.0F;

    /* Switch: '<S556>/Init' incorporates:
     *  Constant: '<S549>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_c = 0.0F;
  } else {
    /* Switch: '<S559>/Init' incorporates:
     *  Constant: '<S549>/Parameter1'
     *  Product: '<S553>/Product'
     *  Sum: '<S553>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cj = LGC_Cmpn2DotLaDmcLdc *
      LGC_TimeSysCycle_P + LGC_CmpnDotLaDmcLdc;

    /* Switch: '<S556>/Init' incorporates:
     *  Constant: '<S549>/Parameter9'
     *  Product: '<S552>/Product'
     *  Sum: '<S552>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_c = LGC_CmpnDotLaDmcLdc *
      LGC_TimeSysCycle_P + LGC_CmpnLaDmcLdc;
  }

  /* End of Switch: '<S559>/Reset' */

  /* Product: '<S464>/Divide' incorporates:
   *  Sum: '<S464>/Sum4'
   *  UnitDelay: '<S464>/Unit_Delay3'
   */
  LGC_RawErrCourseDotCas = (LGC_FltErrCourseCas -
    TJATCT_DW.Unit_Delay3_DSTATE_jb) / LGC_RawErrCourseDotCdc_tmp;

  /* Switch: '<S469>/Switch2' incorporates:
   *  Constant: '<S468>/IAM_Ts_P1'
   *  Constant: '<S468>/IAM_Ts_P4'
   *  RelationalOperator: '<S469>/LowerRelop1'
   *  RelationalOperator: '<S469>/UpperRelop'
   *  Switch: '<S469>/Switch'
   */
  if (rtb_Divide_l_tmp > 1.0F) {
    rtb_Divide_l_tmp = 1.0F;
  } else {
    if (rtb_Divide_l_tmp < 0.0F) {
      /* Switch: '<S469>/Switch' incorporates:
       *  Constant: '<S468>/IAM_Ts_P1'
       */
      rtb_Divide_l_tmp = 0.0F;
    }
  }

  /* End of Switch: '<S469>/Switch2' */

  /* Sum: '<S468>/Add' incorporates:
   *  Product: '<S468>/Product'
   *  Sum: '<S468>/Subtract'
   */
  LGC_FltErrCourseDotCas = (LGC_RawErrCourseDotCas - LGC_FltErrCourseDotCas) *
    rtb_Divide_l_tmp + LGC_FltErrCourseDotCas;

  /* MultiPortSwitch: '<S464>/Multiport Switch' incorporates:
   *  MultiPortSwitch: '<S465>/Multiport Switch'
   *  RelationalOperator: '<S464>/Equal'
   */
  if (LGC_EnaResetByTgq) {
    /* MultiPortSwitch: '<S464>/Multiport Switch' incorporates:
     *  Constant: '<S464>/Constant3'
     */
    LGC_ErrCourseDotCas = 0.0F;

    /* MultiPortSwitch: '<S465>/Multiport Switch' incorporates:
     *  Constant: '<S465>/Constant3'
     */
    LGC_ErrCourse2DotCas = 0.0F;
  } else {
    /* MultiPortSwitch: '<S464>/Multiport Switch' */
    LGC_ErrCourseDotCas = LGC_FltErrCourseDotCas;

    /* MultiPortSwitch: '<S465>/Multiport Switch' incorporates:
     *  Product: '<S465>/Divide'
     *  Sum: '<S465>/Sum1'
     *  UnitDelay: '<S465>/Unit_Delay1'
     */
    LGC_ErrCourse2DotCas = (LGC_ErrCourseDotCas -
      TJATCT_DW.Unit_Delay1_DSTATE_fj) / LGC_RawErrCourseDotCdc_tmp;
  }

  /* End of MultiPortSwitch: '<S464>/Multiport Switch' */

  /* Sum: '<S466>/Sum2' incorporates:
   *  Product: '<S466>/Mul1'
   *  Product: '<S466>/Mul2'
   *  Product: '<S466>/Mul3'
   *  Product: '<S466>/Mul9'
   *  Sum: '<S466>/Sum1'
   *  Sum: '<S466>/Sum3'
   *  Sum: '<S466>/Sum4'
   *  UnitDelay: '<S462>/Unit Delay'
   */
  LGC_RawCmpn2DotLaDmcCas = (((LGC_CoeffNumS0LaDmc * LGC_FltErrCourseCas +
    LGC_CoeffNumS1LaDmc * LGC_ErrCourseDotCas) + LGC_ErrCourse2DotCas) -
    DTE_RawReqDeltaByBnkAgl_tmp_0 * LGC_CmpnDotLaDmcCas) -
    DTE_RawReqDeltaByBnkAgl_tmp * TJATCT_DW.UnitDelay_DSTATE_mz;

  /* MultiPortSwitch: '<S466>/Multiport Switch' */
  if (!LGC_EnaCntrlByTgq) {
    /* MultiPortSwitch: '<S466>/Multiport Switch' incorporates:
     *  Constant: '<S466>/Constant2'
     */
    LGC_Cmpn2DotLaDmcCas = 0.0F;
  } else {
    /* MultiPortSwitch: '<S466>/Multiport Switch' */
    LGC_Cmpn2DotLaDmcCas = LGC_RawCmpn2DotLaDmcCas;
  }

  /* End of MultiPortSwitch: '<S466>/Multiport Switch' */

  /* Switch: '<S477>/Init' incorporates:
   *  Logic: '<S477>/FixPt Logical Operator'
   *  UnitDelay: '<S477>/FixPt Unit Delay2'
   */
  if (LGC_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_of != 0)) {
    /* Switch: '<S477>/Init' incorporates:
     *  Constant: '<S467>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d2 = 0.0F;
  }

  /* End of Switch: '<S477>/Init' */

  /* Switch: '<S475>/Switch2' incorporates:
   *  Constant: '<S467>/Constant7'
   *  Constant: '<S467>/Constant8'
   *  RelationalOperator: '<S475>/LowerRelop1'
   *  RelationalOperator: '<S475>/UpperRelop'
   *  Switch: '<S475>/Switch'
   */
  if (TJATCT_DW.FixPtUnitDelay1_DSTATE_d2 > 100.0F) {
    /* Switch: '<S475>/Switch2' */
    LGC_CmpnDotLaDmcCas = 100.0F;
  } else if (TJATCT_DW.FixPtUnitDelay1_DSTATE_d2 < -100.0F) {
    /* Switch: '<S475>/Switch' incorporates:
     *  Constant: '<S467>/Constant8'
     *  Switch: '<S475>/Switch2'
     */
    LGC_CmpnDotLaDmcCas = -100.0F;
  } else {
    /* Switch: '<S475>/Switch2' incorporates:
     *  Switch: '<S475>/Switch'
     */
    LGC_CmpnDotLaDmcCas = TJATCT_DW.FixPtUnitDelay1_DSTATE_d2;
  }

  /* End of Switch: '<S475>/Switch2' */

  /* Switch: '<S477>/Reset' incorporates:
   *  Switch: '<S474>/Reset'
   */
  if (LGC_EnaResetByTgq) {
    /* Switch: '<S477>/Init' incorporates:
     *  Constant: '<S467>/Constant6'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d2 = 0.0F;

    /* Switch: '<S474>/Init' incorporates:
     *  Constant: '<S467>/Constant1'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cy = 0.0F;
  } else {
    /* Switch: '<S477>/Init' incorporates:
     *  Constant: '<S467>/Parameter1'
     *  Product: '<S471>/Product'
     *  Sum: '<S471>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_d2 = LGC_Cmpn2DotLaDmcCas *
      LGC_TimeSysCycle_P + LGC_CmpnDotLaDmcCas;

    /* Switch: '<S474>/Init' incorporates:
     *  Constant: '<S467>/Parameter9'
     *  Product: '<S470>/Product'
     *  Sum: '<S470>/Subtract'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_cy = LGC_CmpnDotLaDmcCas *
      LGC_TimeSysCycle_P + LGC_CmpnLaDmcCas;
  }

  /* End of Switch: '<S477>/Reset' */

  /* Gain: '<S439>/Gain1' */
  data_log_4 = LGC_LdcCmd_rad;

  /* DataTypeConversion: '<S81>/Data Type Conversion14' */
  LGC_CacPT1Reset_nu = LGC_EnaRstPT1Cac;

  /* Switch: '<S454>/Reset' incorporates:
   *  Logic: '<S447>/NOT3'
   *  Logic: '<S447>/OR'
   *  Switch: '<S453>/Enable'
   */
  if (rtb_OR_km_tmp) {
    /* Switch: '<S454>/Init' incorporates:
     *  Constant: '<S447>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_m = 0.0F;
  } else {
    if (!LGC_EnaFreezeByTgq) {
      /* Switch: '<S454>/Init' incorporates:
       *  Constant: '<S447>/Parameter4'
       *  Inport: '<Root>/Outport41'
       *  Product: '<S447>/Mul5'
       *  Product: '<S451>/Product'
       *  Sum: '<S451>/Subtract'
       *  Switch: '<S453>/Enable'
       *  UnitDelay: '<S454>/FixPt Unit Delay1'
       */
      TJATCT_DW.FixPtUnitDelay1_DSTATE_m = LGC_DeltaFPGainCas *
        Test_CoeffIGainCac * LGC_TimeSysCycle_P + LGC_DeltaFIGainCas;
    }
  }

  /* End of Switch: '<S454>/Reset' */

  /* DataTypeConversion: '<S81>/Data Type Conversion16' */
  LGC_CacIntReset_nu = LGC_EnaRstIntCac;

  /* Gain: '<S416>/Gain' */
  data_log_3 = LGC_LdcAloneICmd_rad;

  /* DataTypeConversion: '<S81>/Data Type Conversion13' */
  LGC_LdcPT1Reset_nu = LGC_EnaRstPT1Ldc;

  /* Switch: '<S534>/Reset' incorporates:
   *  Logic: '<S527>/NOT3'
   *  Switch: '<S533>/Enable'
   */
  if (rtb_OR_fh) {
    /* Switch: '<S534>/Init' incorporates:
     *  Constant: '<S527>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_h = 0.0F;
  } else {
    if (!LGC_EnaFreezeByTgq) {
      /* Switch: '<S534>/Init' incorporates:
       *  Constant: '<S527>/Parameter4'
       *  Inport: '<Root>/Outport34'
       *  Product: '<S527>/Mul5'
       *  Product: '<S531>/Product'
       *  Sum: '<S531>/Subtract'
       *  Switch: '<S533>/Enable'
       *  UnitDelay: '<S534>/FixPt Unit Delay1'
       */
      TJATCT_DW.FixPtUnitDelay1_DSTATE_h = LGC_DeltaFPGainLdc *
        Test_CoeffIGainLdc * LGC_TimeSysCycle_P + LGC_DeltaFIGainLdc;
    }
  }

  /* End of Switch: '<S534>/Reset' */

  /* DataTypeConversion: '<S81>/Data Type Conversion15' */
  LGC_LdcIntReset_nu = LGC_EnaRstIntLdc;

  /* MultiPortSwitch: '<S422>/Multiport Switch' incorporates:
   *  Constant: '<S422>/Constant2'
   */
  if (!rtb_LGC_EnaActSafetyFcn) {
    /* MultiPortSwitch: '<S422>/Multiport Switch1' incorporates:
     *  Constant: '<S422>/Constant1'
     *  Constant: '<S422>/Constant3'
     */
    if (!rtb_LGC_EnaActObjFollow) {
      LGC_RawErrCourseDotCdc_tmp = 1.0F;
    } else {
      LGC_RawErrCourseDotCdc_tmp = 2.0F;
    }

    /* End of MultiPortSwitch: '<S422>/Multiport Switch1' */
  } else {
    LGC_RawErrCourseDotCdc_tmp = 3.0F;
  }

  /* End of MultiPortSwitch: '<S422>/Multiport Switch' */

  /* DataTypeConversion: '<S422>/Data Type Conversion2' */
  LGC_StActParSet = (uint8_T)(int32_T)fmodf(LGC_RawErrCourseDotCdc_tmp, 256.0F);

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcDGain_radspm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainLcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainLcCac_X[0])), ((const real32_T *)&(LGC_CoeffDGainLcCac_M[0])),
    14U);

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcOfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainOfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainOfCac_X[0])), ((const real32_T *)&(LGC_CoeffDGainOfCac_M[0])),
    14U);

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcSfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainSfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainSfCac_X[0])), ((const real32_T *)&(LGC_CoeffDGainSfCac_M[0])),
    14U);

  /* MultiPortSwitch: '<S426>/Multiport Switch3' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S426>/Multiport Switch3' */
    LGC_CoeffDGainCac = LGC_CoeffDGainLcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S426>/Multiport Switch3' */
    LGC_CoeffDGainCac = LGC_CoeffDGainOfCac;
    break;

   default:
    /* MultiPortSwitch: '<S426>/Multiport Switch3' */
    LGC_CoeffDGainCac = LGC_CoeffDGainSfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S426>/Multiport Switch3' */

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1LcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1LcCac_X[0])), ((const real32_T *)&(LGC_TimeDT1LcCac_M[0])), 14U);

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcOfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1OfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1OfCac_X[0])), ((const real32_T *)&(LGC_TimeDT1OfCac_M[0])), 14U);

  /* Lookup_n-D: '<S426>/Y_TCTLGC_LdcSfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1SfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1SfCac_X[0])), ((const real32_T *)&(LGC_TimeDT1SfCac_M[0])), 14U);

  /* MultiPortSwitch: '<S426>/Multiport Switch4' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S426>/Multiport Switch4' */
    LGC_TimeDT1Cac = LGC_TimeDT1LcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S426>/Multiport Switch4' */
    LGC_TimeDT1Cac = LGC_TimeDT1OfCac;
    break;

   default:
    /* MultiPortSwitch: '<S426>/Multiport Switch4' */
    LGC_TimeDT1Cac = LGC_TimeDT1SfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S426>/Multiport Switch4' */

  /* Lookup_n-D: '<S427>/Y_TCTLGC_LdcIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainLcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainLcCac_X[0])), ((const real32_T *)&(LGC_CoeffIGainLcCac_M[0])),
    14U);

  /* Lookup_n-D: '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainOfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainOfCac_X[0])), ((const real32_T *)&(LGC_CoeffIGainOfCac_M[0])),
    14U);

  /* Lookup_n-D: '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainSfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainSfCac_X[0])), ((const real32_T *)&(LGC_CoeffIGainSfCac_M[0])),
    14U);

  /* MultiPortSwitch: '<S427>/Multiport Switch1' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S427>/Multiport Switch1' */
    LGC_CoeffIGainCac = LGC_CoeffIGainLcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S427>/Multiport Switch1' */
    LGC_CoeffIGainCac = LGC_CoeffIGainOfCac;
    break;

   default:
    /* MultiPortSwitch: '<S427>/Multiport Switch1' */
    LGC_CoeffIGainCac = LGC_CoeffIGainSfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S427>/Multiport Switch1' */

  /* MultiPortSwitch: '<S428>/Multiport Switch7' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S428>/Multiport Switch7' incorporates:
     *  Constant: '<S428>/P_TCTLGC_LdcP_radpm'
     */
    LGC_CoeffMainPGainCac = LGC_CoeffMainPGainLcCac_P;
    break;

   case 2:
    /* MultiPortSwitch: '<S428>/Multiport Switch7' incorporates:
     *  Constant: '<S428>/Constant'
     */
    LGC_CoeffMainPGainCac = LGC_CoeffMainPGainOfCac_P;
    break;

   default:
    /* MultiPortSwitch: '<S428>/Multiport Switch7' incorporates:
     *  Constant: '<S428>/P_TCTLGC_LdcSfP_radpm'
     */
    LGC_CoeffMainPGainCac = LGC_CoeffMainPGainSfCac_P;
    break;
  }

  /* End of MultiPortSwitch: '<S428>/Multiport Switch7' */

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainLcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainLcCac_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainLcCac_M[0])), 14U);

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcOfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainOfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainOfCac_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainOfCac_M[0])), 14U);

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcSfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainSfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainSfCac_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainSfCac_M[0])), 14U);

  /* MultiPortSwitch: '<S429>/Multiport Switch3' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S429>/Multiport Switch3' */
    LGC_CoeffPT1GainCac = LGC_CoeffPT1GainLcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S429>/Multiport Switch3' */
    LGC_CoeffPT1GainCac = LGC_CoeffPT1GainOfCac;
    break;

   default:
    /* MultiPortSwitch: '<S429>/Multiport Switch3' */
    LGC_CoeffPT1GainCac = LGC_CoeffPT1GainSfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S429>/Multiport Switch3' */

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1LcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1LcCac_X[0])), ((const real32_T *)&(LGC_TimePT1LcCac_M[0])), 14U);

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcOfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1OfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1OfCac_X[0])), ((const real32_T *)&(LGC_TimePT1OfCac_M[0])), 14U);

  /* Lookup_n-D: '<S429>/Y_TCTLGC_LdcSfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1SfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1SfCac_X[0])), ((const real32_T *)&(LGC_TimePT1SfCac_M[0])), 14U);

  /* MultiPortSwitch: '<S429>/Multiport Switch4' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S429>/Multiport Switch4' */
    LGC_TimePT1Cac = LGC_TimePT1LcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S429>/Multiport Switch4' */
    LGC_TimePT1Cac = LGC_TimePT1OfCac;
    break;

   default:
    /* MultiPortSwitch: '<S429>/Multiport Switch4' */
    LGC_TimePT1Cac = LGC_TimePT1SfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S429>/Multiport Switch4' */

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainLcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainLcLdc_X[0])), ((const real32_T *)&(LGC_CoeffDGainLcLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcOfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainOfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainOfLdc_X[0])), ((const real32_T *)&(LGC_CoeffDGainOfLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcSfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffDGainSfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffDGainSfLdc_X[0])), ((const real32_T *)&(LGC_CoeffDGainSfLdc_M[0])),
    14U);

  /* MultiPortSwitch: '<S430>/Multiport Switch3' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S430>/Multiport Switch3' */
    LGC_CoeffDGainLdc = LGC_CoeffDGainLcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S430>/Multiport Switch3' */
    LGC_CoeffDGainLdc = LGC_CoeffDGainOfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S430>/Multiport Switch3' */
    LGC_CoeffDGainLdc = LGC_CoeffDGainSfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S430>/Multiport Switch3' */

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1LcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1LcLdc_X[0])), ((const real32_T *)&(LGC_TimeDT1LcLdc_M[0])), 14U);

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcOfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1OfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1OfLdc_X[0])), ((const real32_T *)&(LGC_TimeDT1OfLdc_M[0])), 14U);

  /* Lookup_n-D: '<S430>/Y_TCTLGC_LdcSfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimeDT1SfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimeDT1SfLdc_X[0])), ((const real32_T *)&(LGC_TimeDT1SfLdc_M[0])), 14U);

  /* MultiPortSwitch: '<S430>/Multiport Switch4' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S430>/Multiport Switch4' */
    LGC_TimeDT1Ldc = LGC_TimeDT1LcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S430>/Multiport Switch4' */
    LGC_TimeDT1Ldc = LGC_TimeDT1OfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S430>/Multiport Switch4' */
    LGC_TimeDT1Ldc = LGC_TimeDT1SfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S430>/Multiport Switch4' */

  /* Lookup_n-D: '<S431>/Y_TCTLGC_LdcIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainLcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainLcLdc_X[0])), ((const real32_T *)&(LGC_CoeffIGainLcLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainOfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainOfLdc_X[0])), ((const real32_T *)&(LGC_CoeffIGainOfLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffIGainSfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffIGainSfLdc_X[0])), ((const real32_T *)&(LGC_CoeffIGainSfLdc_M[0])),
    14U);

  /* MultiPortSwitch: '<S431>/Multiport Switch1' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S431>/Multiport Switch1' */
    LGC_CoeffIGainLdc = LGC_CoeffIGainLcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S431>/Multiport Switch1' */
    LGC_CoeffIGainLdc = LGC_CoeffIGainOfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S431>/Multiport Switch1' */
    LGC_CoeffIGainLdc = LGC_CoeffIGainSfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S431>/Multiport Switch1' */

  /* MultiPortSwitch: '<S432>/Multiport Switch7' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S432>/Multiport Switch7' incorporates:
     *  Constant: '<S432>/P_TCTLGC_LdcP_radpm'
     */
    LGC_CoeffMainPGainLdc = LGC_CoeffMainPGainLcLdc_P;
    break;

   case 2:
    /* MultiPortSwitch: '<S432>/Multiport Switch7' incorporates:
     *  Constant: '<S432>/Constant'
     */
    LGC_CoeffMainPGainLdc = LGC_CoeffMainPGainOfLdc_P;
    break;

   default:
    /* MultiPortSwitch: '<S432>/Multiport Switch7' incorporates:
     *  Constant: '<S432>/P_TCTLGC_LdcSfP_radpm'
     */
    LGC_CoeffMainPGainLdc = LGC_CoeffMainPGainSfLdc_P;
    break;
  }

  /* End of MultiPortSwitch: '<S432>/Multiport Switch7' */

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainLcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainLcLdc_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainLcLdc_M[0])), 14U);

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcOfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainOfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainOfLdc_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainOfLdc_M[0])), 14U);

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcSfDGain_radspm' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPT1GainSfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T
    *)&(LGC_CoeffPT1GainSfLdc_X[0])), ((const real32_T *)
    &(LGC_CoeffPT1GainSfLdc_M[0])), 14U);

  /* MultiPortSwitch: '<S433>/Multiport Switch3' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S433>/Multiport Switch3' */
    LGC_CoeffPT1GainLdc = LGC_CoeffPT1GainLcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S433>/Multiport Switch3' */
    LGC_CoeffPT1GainLdc = LGC_CoeffPT1GainOfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S433>/Multiport Switch3' */
    LGC_CoeffPT1GainLdc = LGC_CoeffPT1GainSfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S433>/Multiport Switch3' */

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1LcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1LcLdc_X[0])), ((const real32_T *)&(LGC_TimePT1LcLdc_M[0])), 14U);

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcOfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1OfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1OfLdc_X[0])), ((const real32_T *)&(LGC_TimePT1OfLdc_M[0])), 14U);

  /* Lookup_n-D: '<S433>/Y_TCTLGC_LdcSfDT1_sec' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_TimePT1SfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_TimePT1SfLdc_X[0])), ((const real32_T *)&(LGC_TimePT1SfLdc_M[0])), 14U);

  /* MultiPortSwitch: '<S433>/Multiport Switch4' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S433>/Multiport Switch4' */
    LGC_TimePT1Ldc = LGC_TimePT1LcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S433>/Multiport Switch4' */
    LGC_TimePT1Ldc = LGC_TimePT1OfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S433>/Multiport Switch4' */
    LGC_TimePT1Ldc = LGC_TimePT1SfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S433>/Multiport Switch4' */

  /* Lookup_n-D: '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' incorporates:
   *  MultiPortSwitch: '<S80>/Multiport Switch2'
   */
  LGC_CoeffPGainByCrvCac = look1_iflf_binlxpw(FFC_ReqFfcCrv, ((const real32_T *)
    &(LGC_CoeffPGainByCrvCac_X[0])), ((const real32_T *)
    &(LGC_CoeffPGainByCrvCac_M[0])), 8U);

  /* Lookup_n-D: '<S428>/Y_TCTLGC_LdcP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainLcCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainLcCac_X[0])), ((const real32_T *)&(LGC_CoeffPGainLcCac_M[0])),
    14U);

  /* Lookup_n-D: '<S428>/Y_TCTLGC_LdcSfP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainSfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainSfCac_X[0])), ((const real32_T *)&(LGC_CoeffPGainSfCac_M[0])),
    14U);

  /* Lookup_n-D: '<S428>/Y_TCTLGC_LdcOfP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainOfCac = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainOfCac_X[0])), ((const real32_T *)&(LGC_CoeffPGainOfCac_M[0])),
    14U);

  /* MultiPortSwitch: '<S428>/Multiport Switch2' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S428>/Multiport Switch2' incorporates:
     *  Product: '<S428>/Mul1'
     */
    LGC_CoeffPGainCac = LGC_CoeffPGainByCrvCac * LGC_CoeffPGainLcCac;
    break;

   case 2:
    /* MultiPortSwitch: '<S428>/Multiport Switch2' */
    LGC_CoeffPGainCac = LGC_CoeffPGainOfCac;
    break;

   default:
    /* MultiPortSwitch: '<S428>/Multiport Switch2' incorporates:
     *  Product: '<S428>/Mul2'
     */
    LGC_CoeffPGainCac = LGC_CoeffPGainByCrvCac * LGC_CoeffPGainSfCac;
    break;
  }

  /* End of MultiPortSwitch: '<S428>/Multiport Switch2' */

  /* Lookup_n-D: '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' incorporates:
   *  MultiPortSwitch: '<S80>/Multiport Switch2'
   */
  LGC_CoeffPGainByCrvLdc = look1_iflf_binlxpw(FFC_ReqFfcCrv, ((const real32_T *)
    &(LGC_CoeffPGainByCrvLdc_X[0])), ((const real32_T *)
    &(LGC_CoeffPGainByCrvLdc_M[0])), 8U);

  /* Lookup_n-D: '<S432>/Y_TCTLGC_LdcP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainLcLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainLcLdc_X[0])), ((const real32_T *)&(LGC_CoeffPGainLcLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S432>/Y_TCTLGC_LdcSfP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainSfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainSfLdc_X[0])), ((const real32_T *)&(LGC_CoeffPGainSfLdc_M[0])),
    14U);

  /* Lookup_n-D: '<S432>/Y_TCTLGC_LdcOfP_radpm1' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  LGC_CoeffPGainOfLdc = look1_iflf_binlxpw(TCTI_VehicleVelX, ((const real32_T *)
    &(LGC_CoeffPGainOfLdc_X[0])), ((const real32_T *)&(LGC_CoeffPGainOfLdc_M[0])),
    14U);

  /* MultiPortSwitch: '<S432>/Multiport Switch2' */
  switch (LGC_StActParSet) {
   case 1:
    /* MultiPortSwitch: '<S432>/Multiport Switch2' incorporates:
     *  Product: '<S432>/Mul1'
     */
    LGC_CoeffPGainLdc = LGC_CoeffPGainByCrvLdc * LGC_CoeffPGainLcLdc;
    break;

   case 2:
    /* MultiPortSwitch: '<S432>/Multiport Switch2' */
    LGC_CoeffPGainLdc = LGC_CoeffPGainOfLdc;
    break;

   default:
    /* MultiPortSwitch: '<S432>/Multiport Switch2' incorporates:
     *  Product: '<S432>/Mul2'
     */
    LGC_CoeffPGainLdc = LGC_CoeffPGainByCrvLdc * LGC_CoeffPGainSfLdc;
    break;
  }

  /* End of MultiPortSwitch: '<S432>/Multiport Switch2' */

  /* DataTypeConversion: '<S81>/Data Type Conversion12' */
  LGC_ActiveLgcParamSet_nu = LGC_StActParSet;

  /* DataTypeConversion: '<S81>/Data Type Conversion11' */
  LGC_SafetyFunctionActive_nu = LGC_EnaActSafetyFcn;

  /* DataTypeConversion: '<S81>/Data Type Conversion9' */
  LGC_Reset_nu = LGC_EnaResetByTgq;

  /* DataTypeConversion: '<S81>/Data Type Conversion6' */
  LGC_Hold_nu = LGC_EnaFreezeByTgq;

  /* Switch: '<S418>/Switch' */
  if (LGC_EnaCntrlByTgq) {
    /* DataTypeConversion: '<S81>/Data Type Conversion7' incorporates:
     *  Inport: '<Root>/TCTI_StCntrlFcn'
     */
    LGC_EnableCtrl_nu = (TCTI_StCntrlFcn != 0);
  } else {
    /* DataTypeConversion: '<S81>/Data Type Conversion7' incorporates:
     *  Constant: '<S418>/Constant3'
     */
    LGC_EnableCtrl_nu = false;
  }

  /* End of Switch: '<S418>/Switch' */

  /* Switch: '<S616>/Reset' incorporates:
   *  Constant: '<S597>/Constant'
   *  Constant: '<S597>/Parameter3'
   *  Constant: '<S597>/Parameter7'
   *  Constant: '<S597>/Parameter8'
   *  Inport: '<Root>/TCTI_BtfTrajGuiQualifier'
   *  Inport: '<Root>/TCTI_VehicleVelX'
   *  Logic: '<S597>/OR2'
   *  Logic: '<S605>/NOT3'
   *  Product: '<S597>/Divide'
   *  RelationalOperator: '<S597>/Equal2'
   *  RelationalOperator: '<S597>/Equal5'
   *  RelationalOperator: '<S597>/Equal6'
   *  Switch: '<S615>/Enable'
   */
  if (rtb_LQR_I_EnaResetByTgq) {
    /* Switch: '<S616>/Init' incorporates:
     *  Constant: '<S605>/IV'
     */
    TJATCT_DW.FixPtUnitDelay1_DSTATE_o = 0.0F;
  } else {
    if ((TCTI_BtfTrajGuiQualifier != LGC_EnaFreezeByTgq_P) &&
        (TCTI_BtfTrajGuiQualifier != 5) && (TCTI_VehicleVelX >
         LGC_EnaFreezeByVel_P / 3.6F)) {
      /* Switch: '<S616>/Init' incorporates:
       *  Constant: '<S605>/Parameter4'
       *  Inport: '<Root>/Outport43'
       *  Product: '<S605>/Mul5'
       *  Product: '<S613>/Product'
       *  Sum: '<S613>/Subtract'
       *  Switch: '<S615>/Enable'
       *  UnitDelay: '<S616>/FixPt Unit Delay1'
       */
      TJATCT_DW.FixPtUnitDelay1_DSTATE_o = LQR_lateral_error * Test_TimeDT1Cac *
        LGC_TimeSysCycle_P + LQR_I_term_rad;
    }
  }

  /* End of Switch: '<S616>/Reset' */

  /* DataTypeConversion: '<S91>/Data Type Conversion' */
  CDC_PreErrCtrlHeading = CDC_CtrlErrHeading;

  /* DataTypeConversion: '<S81>/Data Type Conversion' */
  LGC_CtrlErrHeadAglCrtd_rad = CDC_CtrlErrHeading;

  /* SignalConversion: '<S93>/Signal Conversion' */
  CDC_RawFltDeltaTheta = CDC_FltDeltaTheta;

  /* Logic: '<S86>/AND1' incorporates:
   *  Constant: '<S86>/P_TCTCDC_WtchdgActive_nu'
   */
  CDC_EnaWatchdog = (rtb_AND2 && (CDC_CswWatchdogAct != 0));

  /* Switch: '<S327>/Reset' incorporates:
   *  Constant: '<S313>/Constant11'
   *  Constant: '<S313>/Constant13'
   *  Constant: '<S313>/Parameter3'
   *  Constant: '<S313>/Parameter4'
   *  Product: '<S316>/Product'
   *  Product: '<S317>/Product'
   *  Sum: '<S316>/Subtract'
   *  Sum: '<S317>/Subtract'
   *  Switch: '<S329>/Reset'
   *  UnitDelay: '<S327>/FixPt Unit Delay1'
   *  UnitDelay: '<S329>/FixPt Unit Delay1'
   */
  if (EST_EnaResetByTgq) {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_e = 0.0F;
    TJATCT_DW.FixPtUnitDelay1_DSTATE_g = 0.0F;
  } else {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_e = EST_DThetaDotPobs * EST_TiSysCycle_P +
      EST_RawEstDThetaPobs;
    TJATCT_DW.FixPtUnitDelay1_DSTATE_g = EST_DYDotPobs * EST_TiSysCycle_P +
      EST_RawEstDYPobs;
  }

  /* End of Switch: '<S327>/Reset' */

  /* Product: '<S346>/Divide1' incorporates:
   *  Product: '<S346>/Product1'
   *  Product: '<S346>/Product2'
   *  Sum: '<S346>/Add'
   */
  EST_MeanHdrPerc = (EST_FacDThetaWghtHdrSel * EST_HdrPercByTheta +
                     EST_FacDYWghtHdrSel * EST_HdrPercByDY) / rtb_Add1_ge;

  /* SignalConversion: '<S312>/Signal Conversion' */
  EST_BetaDotPobs = rtb_Add1_m[0];

  /* Switch: '<S323>/Init' incorporates:
   *  Logic: '<S323>/FixPt Logical Operator'
   *  UnitDelay: '<S323>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_dm != 0)) {
    /* Switch: '<S323>/Init' incorporates:
     *  Constant: '<S313>/Constant1'
     */
    EST_RawEstBetaPobs = 0.0F;
  } else {
    /* Switch: '<S323>/Init' incorporates:
     *  UnitDelay: '<S323>/FixPt Unit Delay1'
     */
    EST_RawEstBetaPobs = TJATCT_DW.FixPtUnitDelay1_DSTATE_h1;
  }

  /* End of Switch: '<S323>/Init' */

  /* Switch: '<S323>/Reset' incorporates:
   *  Constant: '<S313>/Constant1'
   *  Constant: '<S313>/Parameter1'
   *  Product: '<S314>/Product'
   *  Sum: '<S314>/Subtract'
   *  UnitDelay: '<S323>/FixPt Unit Delay1'
   */
  if (EST_EnaResetByTgq) {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_h1 = 0.0F;
  } else {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_h1 = EST_BetaDotPobs * EST_TiSysCycle_P +
      EST_RawEstBetaPobs;
  }

  /* End of Switch: '<S323>/Reset' */

  /* Switch: '<S318>/Switch2' incorporates:
   *  Constant: '<S313>/Constant4'
   *  Constant: '<S313>/Constant5'
   *  RelationalOperator: '<S318>/LowerRelop1'
   *  RelationalOperator: '<S318>/UpperRelop'
   *  Switch: '<S318>/Switch'
   */
  if (EST_RawEstBetaPobs > 3.1415) {
    /* Switch: '<S318>/Switch2' */
    EST_EstBetaPobs = 3.1415F;
  } else if (EST_RawEstBetaPobs < -3.1415) {
    /* Switch: '<S318>/Switch' incorporates:
     *  Constant: '<S313>/Constant5'
     *  Switch: '<S318>/Switch2'
     */
    EST_EstBetaPobs = -3.1415F;
  } else {
    /* Switch: '<S318>/Switch2' incorporates:
     *  Switch: '<S318>/Switch'
     */
    EST_EstBetaPobs = EST_RawEstBetaPobs;
  }

  /* End of Switch: '<S318>/Switch2' */

  /* SignalConversion: '<S312>/Signal Conversion1' */
  EST_Psi2DotPobs = rtb_Add1_m[1];

  /* Switch: '<S325>/Init' incorporates:
   *  Logic: '<S325>/FixPt Logical Operator'
   *  UnitDelay: '<S325>/FixPt Unit Delay2'
   */
  if (EST_EnaResetByTgq || (TJATCT_DW.FixPtUnitDelay2_DSTATE_g != 0)) {
    /* Switch: '<S325>/Init' incorporates:
     *  Constant: '<S313>/Constant17'
     */
    EST_RawEstPsiDotPobs = 0.0F;
  } else {
    /* Switch: '<S325>/Init' incorporates:
     *  UnitDelay: '<S325>/FixPt Unit Delay1'
     */
    EST_RawEstPsiDotPobs = TJATCT_DW.FixPtUnitDelay1_DSTATE_j;
  }

  /* End of Switch: '<S325>/Init' */

  /* Switch: '<S325>/Reset' incorporates:
   *  Constant: '<S313>/Constant17'
   *  Constant: '<S313>/Parameter2'
   *  Product: '<S315>/Product'
   *  Sum: '<S315>/Subtract'
   *  UnitDelay: '<S325>/FixPt Unit Delay1'
   */
  if (EST_EnaResetByTgq) {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_j = 0.0F;
  } else {
    TJATCT_DW.FixPtUnitDelay1_DSTATE_j = EST_Psi2DotPobs * EST_TiSysCycle_P +
      EST_RawEstPsiDotPobs;
  }

  /* End of Switch: '<S325>/Reset' */

  /* Switch: '<S319>/Switch2' incorporates:
   *  Constant: '<S313>/Constant8'
   *  Constant: '<S313>/Constant9'
   *  RelationalOperator: '<S319>/LowerRelop1'
   *  RelationalOperator: '<S319>/UpperRelop'
   *  Switch: '<S319>/Switch'
   */
  if (EST_RawEstPsiDotPobs > 3.1415) {
    /* Switch: '<S319>/Switch2' */
    EST_EstPsiDotPobs = 3.1415F;
  } else if (EST_RawEstPsiDotPobs < -3.1415) {
    /* Switch: '<S319>/Switch' incorporates:
     *  Constant: '<S313>/Constant9'
     *  Switch: '<S319>/Switch2'
     */
    EST_EstPsiDotPobs = -3.1415F;
  } else {
    /* Switch: '<S319>/Switch2' incorporates:
     *  Switch: '<S319>/Switch'
     */
    EST_EstPsiDotPobs = EST_RawEstPsiDotPobs;
  }

  /* End of Switch: '<S319>/Switch2' */

  /* DataTypeConversion: '<S608>/Data Type Conversion3' incorporates:
   *  MATLAB Function: '<S608>/MATLAB Function3'
   */
  LQR_num_iteration = (real32_T)num_iteration;

  /* Update for UnitDelay: '<S308>/Unit Delay3' */
  TJATCT_DW.UnitDelay3_DSTATE[0] = EST_EstBetaPobs;
  TJATCT_DW.UnitDelay3_DSTATE[1] = EST_EstPsiDotPobs;
  TJATCT_DW.UnitDelay3_DSTATE[2] = EST_EstDThetaPobs;
  TJATCT_DW.UnitDelay3_DSTATE[3] = EST_EstDYPobs;

  /* Update for UnitDelay: '<S305>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE = EST_AngleCurSteer;

  /* Update for UnitDelay: '<S305>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE = rtb_Gain;

  /* Update for UnitDelay: '<S305>/Unit_Delay2' */
  TJATCT_DW.Unit_Delay2_DSTATE = smax;

  /* Update for UnitDelay: '<S305>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE = rtb_Gain1;

  /* Update for UnitDelay: '<S305>/Unit_Delay4' */
  TJATCT_DW.Unit_Delay4_DSTATE = rtb_Unit_Delay3;

  /* Update for UnitDelay: '<S414>/Delay Input1'
   *
   * Block description for '<S414>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_i = rtb_Equal3;

  /* Update for UnitDelay: '<S381>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S381>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE = 0U;

  /* Update for UnitDelay: '<S396>/Unit Delay' incorporates:
   *  UnitDelay: '<S400>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_m = TJATCT_DW.UnitDelay_DSTATE_p;

  /* Update for UnitDelay: '<S398>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_p0 = EST_DistYDevByGrdntLmt1;

  /* Update for UnitDelay: '<S385>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE_h[0] = EST_EstBetaSObs;
  TJATCT_DW.Unit_Delay_DSTATE_h[1] = EST_EstPsiDotSObs;

  /* Update for UnitDelay: '<S386>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE_m = EST_AngleCurSteer;

  /* Update for UnitDelay: '<S386>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_j = rtb_Divide_l4;

  /* Update for UnitDelay: '<S386>/Unit_Delay2' */
  TJATCT_DW.Unit_Delay2_DSTATE_g = rtb_MeasDeviationVector_idx_1;

  /* Update for UnitDelay: '<S386>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_j = rtb_Sum_kv;

  /* Update for UnitDelay: '<S386>/Unit_Delay4' */
  TJATCT_DW.Unit_Delay4_DSTATE_j = rtb_Product_km;

  /* Update for UnitDelay: '<S384>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S384>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_m = 0U;

  /* Update for UnitDelay: '<S329>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S329>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b = 0U;

  /* Update for UnitDelay: '<S327>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S327>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_o = 0U;

  /* Update for UnitDelay: '<S363>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_f = EST_AnglePObsDThetaLmt2;

  /* Update for UnitDelay: '<S361>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_b1 = EST_AnglePObsDThetaLmt1;

  /* Update for UnitDelay: '<S86>/Unit_Delay' incorporates:
   *  Inport: '<Root>/TCTI_StCntrlFcn'
   */
  TJATCT_DW.Unit_Delay_DSTATE_c = TCTI_StCntrlFcn;

  /* Update for UnitDelay: '<S96>/Delay Input1'
   *
   * Block description for '<S96>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_a = rtb_Equal3_m;

  /* Update for UnitDelay: '<S92>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_n = rtb_Sum_db;

  /* Update for UnitDelay: '<S92>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_a = rtb_Product1_p4;

  /* Update for UnitDelay: '<S92>/Unit Delay2' */
  TJATCT_DW.UnitDelay2_DSTATE_ja = rtb_Sum_lu;

  /* Update for UnitDelay: '<S92>/Unit Delay3' */
  TJATCT_DW.UnitDelay3_DSTATE_k = rtb_UnitDelay2_p;

  /* Update for UnitDelay: '<S92>/Unit Delay6' */
  TJATCT_DW.UnitDelay6_DSTATE = rtb_UnitDelay3_a;

  /* Update for UnitDelay: '<S92>/Unit Delay5' */
  TJATCT_DW.UnitDelay5_DSTATE = rtb_UnitDelay6;

  /* Update for MultiPortSwitch: '<S92>/Multiport Switch3' incorporates:
   *  UnitDelay: '<S92>/Unit Delay7'
   */
  TJATCT_DW.UnitDelay7_DSTATE = rtb_UnitDelay5;

  /* Update for UnitDelay: '<S623>/Delay Input1'
   *
   * Block description for '<S623>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_m = rtb_Equal3_b;

  /* Update for UnitDelay: '<S609>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_d = LQR_lateral_error;

  /* Update for UnitDelay: '<S624>/Delay Input1'
   *
   * Block description for '<S624>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_k = rtb_Equal4;

  /* Update for UnitDelay: '<S616>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S616>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_c = 0U;

  /* Update for Delay: '<S603>/Delay' */
  TJATCT_DW.Delay_DSTATE = LQR_DeltaF_Cmd_rad;

  /* Update for Delay: '<S603>/Delay1' */
  TJATCT_DW.Delay1_DSTATE = rtb_Switch_gf;

  /* Update for Delay: '<S603>/Delay3' */
  TJATCT_DW.Delay3_DSTATE = rtb_Divide_eu;

  /* Update for UnitDelay: '<S601>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_g = LQR_FltDeltaF_Cmd_rad;

  /* Update for UnitDelay: '<S115>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_bb = LQR_ReqDeltaF_Limit_deg;

  /* Update for UnitDelay: '<S571>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_g4 = LGC_CrvReqFfcGrdLim_1pm;

  /* Update for UnitDelay: '<S125>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_ou = LGC_FFCrv_1pm;

  /* Update for UnitDelay: '<S562>/Delay Input1'
   *
   * Block description for '<S562>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_p = rtb_Equal3_gh;

  /* Update for UnitDelay: '<S556>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S556>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_f = 0U;

  /* Update for UnitDelay: '<S539>/Unit Delay1' incorporates:
   *  Inport: '<Root>/Outport33'
   */
  TJATCT_DW.UnitDelay1_DSTATE_c = Test_CoeffPGainLdc;

  /* Update for UnitDelay: '<S534>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S534>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_a = 0U;

  /* Update for UnitDelay: '<S526>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_e = LGC_DeltaFPGainLdc;

  /* Update for UnitDelay: '<S535>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_lex = LGC_DeltaFPT1GainLdc;

  /* Update for UnitDelay: '<S474>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S474>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_k = 0U;

  /* Update for UnitDelay: '<S457>/Unit Delay1' incorporates:
   *  Inport: '<Root>/Outport40'
   */
  TJATCT_DW.UnitDelay1_DSTATE_n = Test_CoeffPGainCac;

  /* Update for UnitDelay: '<S454>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S454>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b1 = 0U;

  /* Update for UnitDelay: '<S446>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_k = LGC_DeltaFPGainCas;

  /* Update for UnitDelay: '<S514>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S514>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b3 = 0U;

  /* Update for UnitDelay: '<S497>/Unit Delay1' incorporates:
   *  Inport: '<Root>/Outport40'
   */
  TJATCT_DW.UnitDelay1_DSTATE_o = Test_CoeffPGainCac;

  /* Update for UnitDelay: '<S494>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S494>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ms = 0U;

  /* Update for UnitDelay: '<S486>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_m = LGC_DeltaFPGainCdc;

  /* Update for UnitDelay: '<S277>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE_l = EST_CurSteerAngle;

  /* Update for UnitDelay: '<S277>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_d4 = DTE_DlyCurSteerAngle;

  /* Update for UnitDelay: '<S277>/Unit_Delay2' */
  TJATCT_DW.Unit_Delay2_DSTATE_h = rtb_MultiportSwitch2_a;

  /* Update for UnitDelay: '<S277>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_a = gm;

  /* Update for UnitDelay: '<S216>/Delay Input1'
   *
   * Block description for '<S216>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_g0 = rtb_Equal3_n;

  /* Update for UnitDelay: '<S290>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S290>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_e = 0U;

  /* Update for UnitDelay: '<S234>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S234>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_j = 0U;

  /* Update for UnitDelay: '<S238>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE_n0 = DTE_SetCrvLaDmc;

  /* Update for UnitDelay: '<S238>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_ea = rtb_Product1_c;

  /* Update for UnitDelay: '<S238>/Unit_Delay2' */
  TJATCT_DW.Unit_Delay2_DSTATE_ge = rtb_Product1_l5;

  /* Update for UnitDelay: '<S238>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_b = rtb_Unit_Delay2_o;

  /* Update for UnitDelay: '<S238>/Unit_Delay4' */
  TJATCT_DW.Unit_Delay4_DSTATE_i = rtb_Unit_Delay3_ia;

  /* Update for UnitDelay: '<S198>/Delay Input1'
   *
   * Block description for '<S198>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE = DEV_EnaCrvGen;

  /* Update for Switch: '<S572>/Switch' incorporates:
   *  UnitDelay: '<S572>/Unit Delay'
   */
  TJATCT_DW.UnitDelay_DSTATE_h2 = LGC_SumCrvReqFbGrdLim_1pm;

  /* Update for UnitDelay: '<S120>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_dt = LGC_CtrlCrv_DE_1pm;

  /* Update for UnitDelay: '<S102>/Unit Delay' incorporates:
   *  UnitDelay: '<S130>/Unit Delay'
   */
  TC_EnaUnplauUnitDelay_bool = TC_HoldWarnRSFlipFlop_bool;

  /* Update for UnitDelay: '<S110>/Unit Delay' */
  TC_FreezeRSFlipFlop_bool = CLM_EnaSetDegrReq;

  /* Update for UnitDelay: '<S107>/Unit Delay' */
  TC_Freeze2RSFlipFlop_bool = CLM_RawEnaDegrReq;

  /* Update for UnitDelay: '<S135>/Delay Input1'
   *
   * Block description for '<S135>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_l = rtb_GreaterThan1_bj;

  /* Update for UnitDelay: '<S265>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S265>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ej = 0U;

  /* Update for UnitDelay: '<S266>/Unit_Delay' */
  TJATCT_DW.Unit_Delay_DSTATE_nr = DTE_SetDeltaLaDmc;

  /* Update for UnitDelay: '<S266>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_a = rtb_Divide_hw;

  /* Update for UnitDelay: '<S266>/Unit_Delay2' */
  TJATCT_DW.Unit_Delay2_DSTATE_i = rtb_Switch7;

  /* Update for UnitDelay: '<S266>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_h = rtb_Unit_Delay2_j;

  /* Update for UnitDelay: '<S266>/Unit_Delay4' */
  TJATCT_DW.Unit_Delay4_DSTATE_ib = rtb_Neg_i;

  /* Update for UnitDelay: '<S199>/Delay Input1'
   *
   * Block description for '<S199>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_g = DEV_EnaDeltaFGen;

  /* Update for UnitDelay: '<S259>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S259>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_kd = 0U;

  /* Update for UnitDelay: '<S262>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S262>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ck = 0U;

  /* Update for UnitDelay: '<S249>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_f = DTE_DeltaByVdyFcn;

  /* Update for UnitDelay: '<S247>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_i1 = DTE_DeltaDotLaDmc;

  /* Update for UnitDelay: '<S248>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_g2 = DTE_Delta2DotLaDmc;

  /* Update for UnitDelay: '<S252>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_gy = rtb_LGC_StActParSet;

  /* Update for UnitDelay: '<S252>/Unit Delay2' */
  TJATCT_DW.UnitDelay2_DSTATE_o = DTE_SetDeltaLaDmc;

  /* Update for UnitDelay: '<S252>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_k = rtb_Product_dv;

  /* Update for UnitDelay: '<S147>/Delay Input1'
   *
   * Block description for '<S147>/Delay Input1':
   *
   *  Store in Global RAM
   */
  TJATCT_DW.DelayInput1_DSTATE_n = rtb_Equal3_c;

  /* Update for UnitDelay: '<S153>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_if = DTE_DeltaFRte;

  /* Update for UnitDelay: '<S151>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_fo = DTE_DeltaFDotRte;

  /* Update for UnitDelay: '<S152>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_ip = DTE_DeltaF2DotRte;

  /* Update for UnitDelay: '<S167>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S167>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_el = 0U;

  /* Update for UnitDelay: '<S164>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S164>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_kf = 0U;

  /* Update for UnitDelay: '<S170>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S170>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_oe = 0U;

  /* Update for UnitDelay: '<S193>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S193>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_i = 0U;

  /* Update for UnitDelay: '<S187>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S187>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_on = 0U;

  /* Update for UnitDelay: '<S190>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S190>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ct = 0U;

  /* Update for UnitDelay: '<S176>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_ei = DTE_DeltaFPar;

  /* Update for UnitDelay: '<S174>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_d = DTE_DeltaFDotPar;

  /* Update for UnitDelay: '<S175>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_cz = DTE_DeltaF2DotPar;

  /* Update for UnitDelay: '<S180>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_c3 = rtb_Switch2_m4;

  /* Update for UnitDelay: '<S180>/Unit Delay2' */
  TJATCT_DW.UnitDelay2_DSTATE_cu = DEV_SetDeltaFPar;

  /* Update for UnitDelay: '<S180>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_pzg = rtb_Divide_bb;

  /* Update for UnitDelay: '<S280>/Unit Delay' incorporates:
   *  Inport: '<Root>/TCTI_VehYawRate'
   */
  TJATCT_DW.UnitDelay_DSTATE_hr = TCTI_VehYawRate;

  /* Update for UnitDelay: '<S279>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_j = DTE_Psi2DotVdyFcn;

  /* Update for UnitDelay: '<S285>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_c2 = DTE_DeltaVdyFcn;

  /* Update for UnitDelay: '<S293>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S293>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_n = 0U;

  /* Update for UnitDelay: '<S237>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S237>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_d = 0U;

  /* Update for UnitDelay: '<S228>/Unit Delay' incorporates:
   *  Inport: '<Root>/TCTI_VehicleVelX'
   */
  TJATCT_DW.UnitDelay_DSTATE_hn = TCTI_VehicleVelX;

  /* Update for UnitDelay: '<S227>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_k = DTE_DeltaDotForCrv;

  /* Update for UnitDelay: '<S225>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_h4 = DTE_SetCrvLaDmc;

  /* Update for UnitDelay: '<S225>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_ay = rtb_Divide_bo;

  /* Update for UnitDelay: '<S517>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S517>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_fb = 0U;

  /* Update for UnitDelay: '<S502>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_nu = LGC_CmpnLaDmcCdc;

  /* Update for UnitDelay: '<S502>/Unit Delay1' */
  TJATCT_DW.UnitDelay1_DSTATE_fc = LGC_CmpnDotLaDmcCdc;

  /* Update for UnitDelay: '<S504>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_k = LGC_FltErrCourseCdc;

  /* Update for UnitDelay: '<S505>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_f = LGC_ErrCourseDotCdc;

  /* Update for UnitDelay: '<S546>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_kl = LGC_FltPT1YErr_met;

  /* Update for UnitDelay: '<S547>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_a2 = LGC_ErrDistYDot;

  /* Update for UnitDelay: '<S559>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S559>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_h = 0U;

  /* Update for UnitDelay: '<S464>/Unit_Delay3' */
  TJATCT_DW.Unit_Delay3_DSTATE_jb = LGC_FltErrCourseCas;

  /* Update for UnitDelay: '<S465>/Unit_Delay1' */
  TJATCT_DW.Unit_Delay1_DSTATE_fj = LGC_ErrCourseDotCas;

  /* Update for UnitDelay: '<S462>/Unit Delay' */
  TJATCT_DW.UnitDelay_DSTATE_mz = LGC_CmpnLaDmcCas;

  /* Update for UnitDelay: '<S477>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S477>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_of = 0U;

  /* Update for UnitDelay: '<S323>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S323>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_dm = 0U;

  /* Update for UnitDelay: '<S325>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S325>/FixPt Constant'
   */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_g = 0U;
}

/* Model initialize function */
void TJATCT_initialize(void)
{
  /* InitializeConditions for UnitDelay: '<S381>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE = 1U;

  /* InitializeConditions for UnitDelay: '<S384>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_m = 1U;

  /* InitializeConditions for UnitDelay: '<S329>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b = 1U;

  /* InitializeConditions for UnitDelay: '<S327>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_o = 1U;

  /* InitializeConditions for UnitDelay: '<S616>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_c = 1U;

  /* InitializeConditions for UnitDelay: '<S556>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_f = 1U;

  /* InitializeConditions for UnitDelay: '<S534>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_a = 1U;

  /* InitializeConditions for UnitDelay: '<S474>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_k = 1U;

  /* InitializeConditions for UnitDelay: '<S454>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b1 = 1U;

  /* InitializeConditions for UnitDelay: '<S514>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_b3 = 1U;

  /* InitializeConditions for UnitDelay: '<S494>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ms = 1U;

  /* InitializeConditions for UnitDelay: '<S290>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_e = 1U;

  /* InitializeConditions for UnitDelay: '<S234>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_j = 1U;

  /* InitializeConditions for UnitDelay: '<S265>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ej = 1U;

  /* InitializeConditions for UnitDelay: '<S259>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_kd = 1U;

  /* InitializeConditions for UnitDelay: '<S262>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ck = 1U;

  /* InitializeConditions for UnitDelay: '<S167>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_el = 1U;

  /* InitializeConditions for UnitDelay: '<S164>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_kf = 1U;

  /* InitializeConditions for UnitDelay: '<S170>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_oe = 1U;

  /* InitializeConditions for UnitDelay: '<S193>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_i = 1U;

  /* InitializeConditions for UnitDelay: '<S187>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_on = 1U;

  /* InitializeConditions for UnitDelay: '<S190>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_ct = 1U;

  /* InitializeConditions for UnitDelay: '<S293>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_n = 1U;

  /* InitializeConditions for UnitDelay: '<S237>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_d = 1U;

  /* InitializeConditions for UnitDelay: '<S517>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_fb = 1U;

  /* InitializeConditions for UnitDelay: '<S559>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_h = 1U;

  /* InitializeConditions for UnitDelay: '<S477>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_of = 1U;

  /* InitializeConditions for UnitDelay: '<S323>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_dm = 1U;

  /* InitializeConditions for UnitDelay: '<S325>/FixPt Unit Delay2' */
  TJATCT_DW.FixPtUnitDelay2_DSTATE_g = 1U;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
