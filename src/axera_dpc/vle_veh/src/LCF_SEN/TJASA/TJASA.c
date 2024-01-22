/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : TJASA
 *
 * Model Long Name     : Traffic Jam Assist

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_02

 *

 * Model Author        : WJ

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : TJASA.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Wed Feb 15 17:30:31 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "TJASA.h"
#include "TJASA_private.h"
#include "look1_iflf_binlxpw.h"
#include "plook_u32f_bincka.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile boolean_T OF_CheckRearObjects_C_bool =
    1; /* Referenced by: '<S203>/Constant3' */

/* 0: No observation of rear objects
   1: Switch on rear object check */
const volatile boolean_T OF_LcaBsdSigEnabled_C_bool = 1; /* Referenced by:
                                                          * '<S203>/Constant'
                                                          * '<S203>/Constant2'
                                                          */

/* Switch to enable check of LCA/BSD signals for rear object assessment
   (if FALSE MS flag signals will be read) */
const volatile real32_T OF_MinDurFreeAdjLane_C_sec =
    1.0F; /* Referenced by: '<S203>/Constant1' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
const volatile real32_T OF_MinDurFreeSideCollision_C_sec =
    1.0F; /* Referenced by: '<S193>/Constant2' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
const volatile boolean_T TJACMB_CMB_Enabled_C_bool =
    1; /* Referenced by: '<S15>/Constant1' */

/* Switch to enable combined mode */
const volatile boolean_T TJACMB_CombinedDataEnable_C_bool =
    0; /* Referenced by:
        * '<S15>/Constant3'
        * '<S18>/Constant1'
        * '<S665>/Constant'
        * '<S744>/Constant'
        * '<S809>/Constant'
        */

/* Switch to enable use of combined data */
const volatile boolean_T TJACMB_EnableOFO_C_bool =
    0; /* Referenced by: '<S17>/Constant4' */

/* Switch to enable object following only during combined mode */
const volatile boolean_T TJACMB_IndVelLimitsEnable_C_bool =
    0; /* Referenced by:
        * '<S39>/Constant1'
        * '<S39>/Constant6'
        */

/* TRUE if individual combined mode velocities shall be enabled
   (P_TJACMB_VelXMax_kph, P_TJACMB_VelXMin_kph) */
const volatile uint8_T TJACMB_LaneQualityHyst_C_perc =
    0U; /* Referenced by: '<S16>/Constant3' */

/* Minimum ABPR lane quality below which use of lane data in combined shall be
 * disabled (hysteresis) */
const volatile uint8_T TJACMB_LaneQualityMin_C_perc =
    30U; /* Referenced by: '<S16>/Constant4' */

/* Minimum ABPR lane quality below which use of lane data in combined shall be
 * disabled */
const volatile real32_T TJACMB_LnLengthMaxOFO_C_met =
    15.0F; /* Referenced by: '<S17>/Constant3' */

/* Maximum ABPR lane length below which OFO mode can be enabled */
const volatile boolean_T TJACMB_LnQualBothLanes_C_bool =
    0; /* Referenced by: '<S16>/Constant5' */

/*  Switch to require both lane qualities invalid for transition to object data
 * during combined mode */
const volatile boolean_T TJACMB_LnQualCheckEnable_C_bool =
    0; /* Referenced by: '<S16>/Constant2' */

/* Switch to enable check of lane qualities for transition to object data during
 * combined mode */
const volatile real32_T TJACMB_LnQualTurnOffTime_C_sec =
    0.1F; /* Referenced by: '<S16>/Constant' */

/* Turn off delay time for minimum lane quality check */
const volatile real32_T TJACMB_LnQualTurnOnTime_C_sec =
    1.0F; /* Referenced by:
           * '<S16>/Constant1'
           * '<S19>/Constant3'
           */

/* Turn on delay time for minimum lane quality check */
const volatile real32_T TJACMB_LnWeightCrvChng_C_nu =
    1.0F; /* Referenced by: '<S18>/Constant5' */

/* Combined mode curvature weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
const volatile real32_T TJACMB_LnWeightCrv_C_nu =
    1.0F; /* Referenced by: '<S18>/Constant4' */

/* Combined mode curvature weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
const volatile real32_T TJACMB_LnWeightHead_C_nu =
    1.0F; /* Referenced by: '<S18>/Constant3' */

/* Combined mode heading angle weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
const volatile real32_T TJACMB_LnWeightPosY0_C_nu =
    1.0F; /* Referenced by: '<S18>/Constant2' */

/* Combined mode lateral position weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
const volatile real32_T TJACMB_TgtObjLengthMaxOFO_C_met =
    20.0F; /* Referenced by: '<S17>/Constant1' */

/* Maximum target object length below which OFO mode can be enabled */
const volatile real32_T TJACMB_VelXMaxOFO_C_kph =
    20.0F; /* Referenced by: '<S17>/Constant2' */

/* Maximum ego vehicle velocity below which OFO mode can be enabled */
const volatile real32_T TJACMB_VelXMax_C_kph =
    130.0F; /* Referenced by: '<S39>/Constant7' */

/* Maximum allowed ego vehicle velocity for combined mode */
const volatile real32_T TJACMB_VelXMin_C_kph =
    30.0F; /* Referenced by: '<S39>/Constant3' */

/* Minimum required ego vehicle velocity for combined mode */
const volatile real32_T TJAGEN_AccelXHyst_C_mps2 = 0.294F; /* Referenced by:
                                                            * '<S88>/Constant3'
                                                            * '<S88>/Constant6'
                                                            */

/* Maximum long acceleration for TJA activation (hysteresis) */
const volatile real32_T TJAGEN_AccelYHyst_C_mps2 =
    0.98F; /* Referenced by: '<S88>/Constant1' */

/* Maximum lateral acceleration for TJA activation (hysteresis) */
const volatile real32_T TJAGEN_AclXMax_C_mps2 =
    4.8F; /* Referenced by: '<S88>/Constant4' */

/* Maximum long acceleration for TJA activation */
const volatile real32_T TJAGEN_AclXMin_C_mps2 =
    -4.8F; /* Referenced by: '<S88>/Constant5' */

/* Minimum long deceleration for TJA activation */
const volatile real32_T TJAGEN_AclYMax_C_mps2 =
    1.96F; /* Referenced by: '<S88>/Constant2' */

/* Maximum lateral acceleration for TJA activation */
const volatile uint8_T TJAGEN_ActiveStCtrlSR_C_btm =
    237U; /* Referenced by: '<S89>/Constant2' */

/* Bitmask for active state check of vehicle safety functions */
const volatile uint8_T TJAGEN_ActiveStCtrlWarn_C_btm =
    205U; /* Referenced by: '<S613>/Constant23' */

/* Bitmask for active state check of vehicle safety functions */
const volatile real32_T TJAGEN_BlockTimeTJA_C_sec =
    0.1F; /* Referenced by: '<S97>/Constant2' */

/* TJA blocking time */
const volatile boolean_T TJAGEN_CheckTJAErrorState_C_bool =
    0; /* Referenced by: '<S51>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
const volatile uint8_T TJAGEN_DrvStBrake_C_btm = 64U; /* Referenced by:
                                                       * '<S609>/Constant4'
                                                       * '<S62>/Constant4'
                                                       */

/* Bitmask value for driver brake intervention */
const volatile uint8_T TJAGEN_DrvStHOD_C_btm =
    16U; /* Referenced by: '<S64>/Constant1' */

/* Bitmask value for turn signal hazard */
const volatile uint8_T TJAGEN_DrvStHazard_C_btm =
    4U; /* Referenced by: '<S67>/Constant1' */

/* Bitmask value for turn signal hazard */
const volatile uint8_T TJAGEN_DrvStInvalidC_C_btm =
    32U; /* Referenced by: '<S46>/Constant4' */

/* Bitmask value for driver state checks of the cancel condition */
const volatile uint8_T TJAGEN_DrvStInvalidSR_C_btm =
    6U; /* Referenced by: '<S90>/Constant2' */

/*  Bitmask value for driver state checks of the strong ready condition */
const volatile uint8_T TJAGEN_DrvStInvalidWR_C_btm =
    0U; /* Referenced by: '<S54>/Constant1' */

/* Bitmask value for driver state checks of the weak ready condition */
const volatile uint8_T TJAGEN_DrvStNotBuckled_C_btm =
    2U; /* Referenced by: '<S63>/Constant1' */

/* Bitmask value for not buckled up */
const volatile real32_T TJAGEN_HODMaxTime_sec =
    5.0F; /* Referenced by: '<S64>/V_Parameter3' */

/* Max time of HOD */
const volatile real32_T TJAGEN_HazardMaxTime_sec =
    1.0F; /* Referenced by: '<S67>/V_Parameter3' */

/* Max time of turn signal hazard */
const volatile boolean_T TJAGEN_LKA_Available_C_bool = 1; /* Referenced by:
                                                           * '<S48>/Constant'
                                                           * '<S48>/Constant3'
                                                           * '<S51>/Constant'
                                                           */

/* Indicates TRUE if LKA is a coded function of the system */
const volatile real32_T TJAGEN_ManualTorqueESMaxTime_sec =
    2.0F; /* Referenced by: '<S65>/V_Parameter3' */

/* Max time of manual torque intervention */
const volatile real32_T TJAGEN_ManualTorqueHyst_nm =
    0.1F; /* Referenced by:
           * '<S65>/V_Parameter2'
           * '<S65>/V_Parameter4'
           */

/* maximum manual torque hysteresis */
const volatile real32_T TJAGEN_ManualTorqueMaxTime_sec =
    0.2F; /* Referenced by: '<S65>/V_Parameter1' */

/* Max time of manual torque intervention */
const volatile real32_T TJAGEN_ManualTorqueMax_nm = 2.0F;
/* Referenced by: '<S65>/V_Parameter11' */

/* maximum manual torque  */
const volatile real32_T TJAGEN_ManualTorqueMin_nm =
    0.8F; /* Referenced by: '<S65>/V_Parameter5' */

/* minimum manual torque  */
const volatile uint16_T TJAGEN_PrjSpecQuA_C_btm =
    0U; /* Referenced by: '<S49>/Constant7' */

/* Bitmask to check project specific abort conditions for all TJA modes */
const volatile uint16_T TJAGEN_PrjSpecQuC_C_btm =
    0U; /* Referenced by: '<S49>/Constant1' */

/* Bitmask to check project specific cancel conditions for all TJA modes */
const volatile uint16_T TJAGEN_PrjSpecQuSR_C_btm =
    0U; /* Referenced by: '<S49>/Constant2' */

/* Bitmask to check project specific strong ready conditions for all TJA modes
 */
const volatile uint16_T TJAGEN_PrjSpecQuWR_C_btm =
    0U; /* Referenced by: '<S49>/Constant4' */

/* Bitmask to check project specific weak ready conditions for all TJA modes */
const volatile uint8_T TJAGEN_QuTrajCtrCancel_C_btm =
    0U; /* Referenced by: '<S46>/Constant7' */

/* Bitmask check for S_TCTCLM_QuServTrajCtr_nu as cancel condition */
const volatile uint8_T TJAGEN_QuTrajCtrClearance_C_btm =
    0U; /* Referenced by: '<S47>/Constant4' */

/* Bitmask check for S_TCTCLM_QuServTrajCtr_nu as clearance condition */
const volatile uint16_T TJAGEN_QuTrajPlanCancel_C_btm =
    64U; /* Referenced by: '<S46>/Constant1' */

/* Bitmask check for S_TPLTJC_QuStatusTrajPlan_nu as cancel condition */
const volatile uint16_T TJAGEN_QuTrajPlanClearance_C_btm =
    0U; /* Referenced by: '<S47>/Constant2' */

/* Bitmask check for S_TPLTJC_QuStatusTrajPlan_nu as clearance condition */
const volatile uint8_T TJAGEN_QuTrajPlanMinLnQual_C_perc =
    51U; /* Referenced by: '<S105>/Constant1' */

/* Minimum required lane quality for function re-activation after specific
 * blocking time for TrajPlanCancelQualifier */
const volatile real32_T TJAGEN_RampoutTimeMax_C_sec = 0.3F; /* Referenced by:
                                                             * '<S59>/Constant'
                                                             * '<S599>/Constant'
                                                             */

/*  Maximum rampout time */
const volatile real32_T TJAGEN_SafetyFuncMaxTime_sec =
    0.0F; /* Referenced by:
           * '<S71>/V_Parameter1'
           * '<S71>/V_Parameter2'
           */

/* Max time of Safety function */
const volatile boolean_T TJAGEN_SetSysStOnLatDMC_C_bool =
    0; /* Referenced by: '<S47>/Constant1' */

/* Manual switch to set LatDMC system state ON */
const volatile real32_T TJAGEN_SteerWAngleGradHystDI_C_degps =
    20.0F; /* Referenced by: '<S68>/Constant8' */

/* steering wheel angle grad hysteresis */
const volatile real32_T TJAGEN_SteerWAngleGradHystSus_C_degps =
    2.0F; /* Referenced by: '<S68>/Constant2' */

/* steering wheel angle grad hysteresis for suspended */
const volatile real32_T TJAGEN_SteerWAngleGradMaxDI_C_degps =
    120.0F; /* Referenced by: '<S68>/Constant7' */

/* steering wheel angle grad maximum */
const volatile real32_T TJAGEN_SteerWAngleGradMaxSus_C_degps =
    120.0F; /* Referenced by: '<S68>/Constant1' */

/* steering wheel angle grad maximum for suspended */
const volatile real32_T TJAGEN_SteerWAngleHystDI_C_deg =
    5.0F; /* Referenced by: '<S69>/Constant6' */

/* steering wheel angle hysterisis */
const volatile real32_T TJAGEN_SteerWAngleHystSus_C_deg =
    2.0F; /* Referenced by: '<S69>/Constant2' */

/* steering wheel angle hysterisis for suspended */
const volatile real32_T TJAGEN_SteerWAngleMaxDI_C_deg =
    90.0F; /* Referenced by: '<S69>/Constant3' */

/* steering wheel angle maximum */
const volatile real32_T TJAGEN_SteerWAngleMaxSus_C_deg =
    90.0F; /* Referenced by: '<S69>/Constant1' */

/* steering wheel angle maximum for suspended */
const volatile real32_T TJAGEN_SteerWAngleMaxWR_C_deg =
    90.0F; /* Referenced by: '<S97>/Constant3' */

/* Maximum steering wheel angle for function activation */
const volatile uint8_T TJAGEN_SysStErrorSR_C_btm = 237U; /* Referenced by:
                                                          * '<S613>/Constant22'
                                                          * '<S89>/Constant1'
                                                          */

/* Bitmask for system error check of vehicle safety functions (ABS, ESC, etc.)
 * for SR condition */
const volatile uint8_T TJAGEN_SysStNotAvailableSR_C_btm =
    15U; /* Referenced by:
          * '<S613>/Constant26'
          * '<S89>/Constant5'
          */

/* Bitmask to check for weak ready conditions if vehicle safety functions are
 * not available */
const volatile uint8_T TJAGEN_SysStNotAvailableWR_C_btm =
    5U; /* Referenced by: '<S54>/Constant5' */

/* Bitmask to check for weak ready conditions if vehicle safety functions are
 * not available */
const volatile boolean_T TJAGEN_TJA_Available_C_bool = 1; /* Referenced by:
                                                           * '<S48>/Constant1'
                                                           * '<S48>/Constant4'
                                                           * '<S51>/Constant1'
                                                           */

/* Indicates TRUE if TJA is a coded function of the system */
const volatile boolean_T TJAGEN_TJA_ManFunctionSwitch_C_bool =
    0; /* Referenced by: '<S48>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
const volatile uint8_T TJAGEN_VehAclYInvalid_C_btm =
    32U; /* Referenced by: '<S71>/Constant7' */

/* Bitmask value for VehAclYInvalid */
const volatile real32_T TJAGEN_VehCrvHystDI_1pm =
    0.005F; /* Referenced by: '<S70>/Constant11' */

/* Vehicle curve hysteresis */
const volatile real32_T TJAGEN_VehCrvMaxDI_1pm =
    0.02F; /* Referenced by: '<S70>/Constant12' */

/* Vehicle curve maximum */
const volatile real32_T TJAGEN_VehCrvMaxWR_Cr_1pm[6] = {
    0.02F, 0.02F,   0.02F,
    0.01F, 0.0013F, 0.001F}; /* Referenced by: '<S54>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Vehicle curve maximum */
const volatile uint8_T TJAGEN_VehSafetyFuncActive_C_btm =
    4U; /* Referenced by: '<S71>/Constant1' */

/* Bitmask value for VehSafetyFuncActive */
const volatile uint8_T TJAGEN_VehSafetyFuncError_C_btm =
    8U; /* Referenced by: '<S71>/Constant5' */

/* Bitmask value for VehSafetyFuncError */
const volatile uint16_T TJAGEN_VehStInvalidC_C_btm =
    0U; /* Referenced by: '<S46>/Constant2' */

/* Bitmask value for vehicle state checks of the cancel condition */
const volatile uint16_T TJAGEN_VehStInvalidSR_C_btm =
    14589U; /* Referenced by: '<S92>/Constant2' */

/* Bitmask for vehicle state checks of the strong ready condition */
const volatile uint16_T TJAGEN_VehStInvalidWR_C_btm =
    258U; /* Referenced by: '<S54>/Constant2' */

/*  Bitmask for vehicle state checks of the weak ready condition */
const volatile uint8_T TJAGEN_VehStInvalid_C_btm =
    2U; /* Referenced by: '<S71>/Constant' */

/* Bitmask value for VehStInvalid */
const volatile real32_T TJAGEN_VehVelX_Bx_mps[6] = {
    0.0F,  5.0F,  10.0F,
    20.0F, 40.0F, 60.0F}; /* Referenced by: '<S54>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJAGEN_VehYawRateHystDI_C_rps =
    0.05F; /* Referenced by: '<S72>/Constant10' */

/* Vehicle yaw rate hysteresis */
const volatile real32_T TJAGEN_VehYawRateMaxDI_C_rps =
    0.25F; /* Referenced by: '<S72>/Constant9' */

/* Vehicle yaw rate maximum */
const volatile real32_T TJALKA_BlockTimeTJA_C_sec = 2.0F; /* Referenced by:
                                                           * '<S104>/Constant'
                                                           * '<S185>/Constant1'
                                                           */

/* TJA Blocking time */
const volatile boolean_T TJALKA_BothSideBrdgEnable_C_bool =
    0; /* Referenced by: '<S150>/Constant' */

/* TRUE if both-sided bridging/prediction is enabled */
const volatile real32_T TJALKA_CntrCrv_Bx_1pm[11] = {
    0.0F,    0.0005F, 0.001F, 0.002F, 0.003F, 0.005F,
    0.0067F, 0.008F,  0.01F,  0.015F, 0.02F};
/* Referenced by: '<S112>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJAOBF VehVelX  */
const volatile boolean_T TJALKA_ConstSiteCheckOn_C_bool =
    1; /* Referenced by: '<S106>/Constant' */

/*  Switch to enable construction site check */
const volatile real32_T TJALKA_CrvLaneWidthMaxThd_Cr_met[11] = {
    4.2F, 4.2F, 4.2F, 4.5F, 4.6F, 4.8F, 5.0F, 5.2F, 5.2F, 5.5F, 5.5F};
/* Referenced by: '<S112>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid heading angle for OF
 * mode activation (WR condition) */
const volatile real32_T TJALKA_CrvQualTurnOffTime_C_sec =
    0.5F; /* Referenced by: '<S152>/Constant3' */

/* Turn off delay time for lane curvature quality check */
const volatile real32_T TJALKA_CrvQualTurnOnTime_C_sec =
    0.5F; /* Referenced by: '<S152>/Constant2' */

/* Turn on delay time for lane curvature quality check */
const volatile uint8_T TJALKA_CrvQualityHyst_C_perc =
    0U; /* Referenced by: '<S152>/Constant' */

/* Minimum lane curvature quality hysteresis */
const volatile uint8_T TJALKA_CrvQualityMin_C_perc =
    0U; /* Referenced by: '<S152>/Constant1' */

/* Minimum lane curvature quality */
const volatile real32_T TJALKA_DistVeh2LnBndHyst_C_met =
    0.3F; /* Referenced by: '<S116>/Constant1' */

/* Minimum distance to lane boundary hysteresis */
const volatile real32_T TJALKA_DistVeh2LnBndMin_C_met =
    0.7F; /* Referenced by: '<S116>/Constant' */

/* Minimum distance to lane boundary */
const volatile uint8_T TJALKA_DrvStInvalidSR_C_btm =
    8U; /* Referenced by: '<S118>/Constant2' */

/* Bitmask to check if turn signal left/right is engaged */
const volatile boolean_T TJALKA_InjectLaneError_C_bool =
    0; /* Referenced by: '<S153>/Constant5' */

/* Setting to TRUE will inject single error, which invalidates lanes for
 * P_TJALKA_LaneInvalidTime_sec seconds */
const volatile boolean_T TJALKA_LC_Enabled_C_bool =
    1; /* Referenced by: '<S101>/Constant1' */

/* Indicates TRUE, if lane centering mode is enabled. */
const volatile real32_T TJALKA_LaneInvalidTime_C_sec =
    0.5F; /* Referenced by: '<S153>/Constant6' */

/* Lane invalidation time for lane error injection */
const volatile uint8_T TJALKA_LaneQualityHyst_C_perc =
    0U; /* Referenced by: '<S154>/Constant' */

/* Minimum lane quality hysteresis */
const volatile uint8_T TJALKA_LaneQualityMin_C_perc =
    0U; /* Referenced by: '<S154>/Constant1' */

/* Minimum lane quality */
const volatile real32_T TJALKA_LaneValidMaxTime_sec =
    0.3F; /* Referenced by:
           * '<S102>/V_Parameter1'
           * '<S102>/V_Parameter3'
           */

/* Max time of manual torque intervention */
const volatile uint16_T TJALKA_LaneValid_C_btm =
    40959U; /* Referenced by: '<S153>/Constant3' */

/* Bitmask for lane validity check (left/right) */
const volatile uint16_T TJALKA_LaneVirtOrBridged_C_btm =
    8192U; /* Referenced by: '<S153>/Constant2' */

/* Bitmask to check if lanes are bridged or virtual */
const volatile real32_T TJALKA_LaneWidthHyst_C_met =
    0.14F; /* Referenced by:
            * '<S112>/Constant1'
            * '<S112>/Constant2'
            * '<S270>/Constant2'
            * '<S270>/Constant4'
            * '<S308>/Constant2'
            * '<S308>/Constant4'
            */

/* Hysteresis for lane width check */
const volatile real32_T TJALKA_LaneWidthMax_C_met =
    4.2F; /* Referenced by: '<S308>/Constant1' */

/* Maximum allowed lane width for lane centering */
const volatile real32_T TJALKA_LaneWidthMin_C_met = 2.65F; /* Referenced by:
                                                            * '<S112>/Constant3'
                                                            * '<S308>/Constant3'
                                                            */

/* Minimum required lane width for lane centering */
const volatile real32_T TJALKA_LnIncohMaxTime_C_sec =
    7.0F; /* Referenced by:
           * '<S111>/Constant2'
           * '<S111>/Constant5'
           * '<S111>/Constant6'
           */

/* Lane incoherence max time */
const volatile real32_T TJALKA_LnIncoherenceMaxPosY0_C_met =
    1.0F; /* Referenced by:
           * '<S111>/Constant'
           * '<S111>/Constant1'
           */

/* Lane incoherence max posy0 */
const volatile real32_T TJALKA_LnPredMinValidTime_C_sec =
    5.0F; /* Referenced by: '<S150>/Constant2' */

/* Minimum required time of valid lanes and active controlling in LC or CMB mode
 * for allowing both-sided lane prediction */
const volatile real32_T TJALKA_LnPredictionCrvMax_C_1pm =
    0.0005F; /* Referenced by: '<S150>/Constant1' */

/* Maximum allowed lane curvature for lane prediction activation */
const volatile real32_T TJALKA_LnPredictionTimeMax_C_sec =
    2.0F; /* Referenced by: '<S164>/Constant1' */

/* Maximum allowed lane prediction duration */
const volatile real32_T TJALKA_LnPredictionTimeMin_C_sec =
    1.0F; /* Referenced by: '<S164>/Constant' */

/* Minimum prediction time for rampout prediction (prediction of lanes while
 * outputs torque output is set to zero) */
const volatile real32_T TJALKA_LnQualTurnOffTime_C_sec =
    0.5F; /* Referenced by: '<S154>/Constant3' */

/* Turn off delay time for lane quality check */
const volatile real32_T TJALKA_LnQualTurnOnTime_C_sec =
    0.5F; /* Referenced by: '<S154>/Constant2' */

/* Turn on delay time for lane quality check */
const volatile real32_T TJALKA_MaxHeadAngActnTJA_C_rad =
    0.1F; /* Referenced by: '<S110>/Constant1' */

/*  Maximum allowed ego vehicle heading angle for activation of TJA */
const volatile uint16_T TJALKA_PrjSpecQuC_C_btm =
    0U; /* Referenced by: '<S107>/Constant6' */

/* Bitmask to check project specific cancel conditions for lane centering */
const volatile uint16_T TJALKA_PrjSpecQuSR_C_btm =
    0U; /* Referenced by: '<S107>/Constant4' */

/* Bitmask to check project specific SR conditions for lane centering */
const volatile uint16_T TJALKA_PrjSpecQuWR_C_btm =
    0U; /* Referenced by: '<S107>/Constant3' */

/* Bitmask to check project specific WR conditions for lane centering */
const volatile real32_T TJALKA_RadiusHyst_C_met =
    10.0F; /* Referenced by: '<S114>/Constant1' */

/* Curvature radius hysteresis */
const volatile real32_T TJALKA_RadiusMin_C_met =
    40.0F; /* Referenced by: '<S114>/Constant' */

/* Minimum curvature radius for lane centering controlling */
const volatile boolean_T TJALKA_RampoutPredictOn_C_bool =
    0; /* Referenced by:
        * '<S963>/Constant1'
        * '<S164>/Constant2'
        * '<S686>/Constant1'
        */

/* Switch to enable rampout prediction (lane prediction while LatDMC output is
 * set to zero) */
const volatile boolean_T TJALKA_TransLnChecksOff_C_bool =
    1; /* Referenced by: '<S103>/Constant' */

/* TRUE if lane quality checks shall be disabled during SALC --> LC transition.
   That is, lane quality is valid by default if the Parameter is TRUE and
   TakeOverValid flag from SLC module is TRUE. */
const volatile boolean_T TJALKA_UseUncoupLaneWidth_C_bool =
    0; /* Referenced by: '<S112>/Constant4' */

/* Switch to enable use of uncoupled lane width for lane width check */
const volatile real32_T TJALKA_ValidLengthMinHyst_C_met =
    5.0F; /* Referenced by: '<S115>/Constant2' */

/* Minimum required valid length (hysteresis) */
const volatile real32_T TJALKA_ValidLengthMin_C_met =
    15.0F; /* Referenced by: '<S115>/Constant3' */

/* Minimum required valid length */
const volatile real32_T TJALKA_VehDistMax_C_met =
    180.0F; /* Referenced by: '<S126>/Constant3' */

/* Vehicle distance maximum */
const volatile real32_T TJALKA_VelXHyst_C_kph = 5.0F; /* Referenced by:
                                                       * '<S39>/Constant10'
                                                       * '<S39>/Constant2'
                                                       * '<S119>/Constant1'
                                                       * '<S119>/Constant2'
                                                       * '<S120>/Constant1'
                                                       * '<S120>/Constant2'
                                                       * '<S192>/Constant2'
                                                       * '<S192>/Constant4'
                                                       * '<S488>/Constant2'
                                                       * '<S488>/Constant4'
                                                       */

/* Longitudinal velocity hysteresis */
const volatile real32_T TJALKA_VelXMax_C_kph = 135.0F; /* Referenced by:
                                                        * '<S39>/Constant9'
                                                        * '<S119>/Constant'
                                                        * '<S120>/Constant'
                                                        */

/* Maximum longitudinal velocity allowed for lane centering mode */
const volatile real32_T TJALKA_VelXMinWR_C_kph =
    15.0F; /* Referenced by: '<S120>/Constant3' */

/* Maximum longitudinal velocity allowed for lane centering mode */
const volatile real32_T TJALKA_VelXMin_C_kph = 0.0F; /* Referenced by:
                                                      * '<S39>/Constant5'
                                                      * '<S119>/Constant3'
                                                      */

/*  Minimum longitudinal velocity required for lane centering mode */
const volatile uint16_T TJAOBF_AccObjChange_C_btm =
    2048U; /* Referenced by: '<S206>/Constant4' */

/* Bit mask to check ACC object validity for object in target lane evalution
 * (ignoring lateral position validity) */
const volatile uint16_T TJAOBF_AccObjLanesInvalid_C_btm =
    2039U; /* Referenced by: '<S206>/Constant2' */

/* Bit mask to check ACC object validity for object in target lane evalution
 * (ignoring lateral position validity) */
const volatile uint16_T TJAOBF_AccObjectInvalid_C_btm =
    2047U; /* Referenced by: '<S206>/Constant1' */

/* Bit mask to check ACC object validity */
const volatile real32_T TJAOBF_CheckLineValidMaxTime_sec =
    2.0F; /* Referenced by:
           * '<S242>/V_Parameter1'
           * '<S242>/V_Parameter2'
           */

/* Maximum lateral acceleration for TJA activation */
const volatile boolean_T TJAOBF_ConstSiteCheckOn_C_bool =
    0; /* Referenced by: '<S187>/Constant1' */

/* Switch to enable construction site check for OF SR condition */
const volatile uint8_T TJAOBF_CrvQualityHyst_C_perc =
    10U; /* Referenced by: '<S242>/Constant4' */

/* Minimum lane curvature quality hysteresis for object-in-lane-evaluation */
const volatile uint8_T TJAOBF_CrvQualityMin_C_perc =
    50U; /* Referenced by: '<S242>/Constant' */

/* Minimum lane curvature quality required for object-in-lane-evaluation */
const volatile real32_T TJAOBF_CutinObValidFreezTm_C_sec =
    3.0F; /* Referenced by: '<S210>/Constant' */

/*  Freeze time of Acc object validity status after object cut-in */
const volatile real32_T TJAOBF_DefaultLaneWidth_C_met =
    3.5F; /* Referenced by: '<S228>/Constant4' */

/* Assumption of default lane width for object-in-lane check for maximum
 * distance to lane boundary */
const volatile uint8_T TJAOBF_DrvStInvalidSR_C_btm =
    8U; /* Referenced by: '<S194>/Constant2' */

/* Bitmask for turn signal SR condition check */
const volatile real32_T TJAOBF_EgoCurveMaxSideCollision_C_1pm =
    0.01F; /* Referenced by:
            * '<S193>/Constant'
            * '<S193>/Constant1'
            */

/* Maximum allowed ego curvature for OF mode side collision check */
const volatile boolean_T TJAOBF_LaneCheckEnabled_C_bool =
    1; /* Referenced by: '<S224>/Constant1' */

/* Switch to enable object in lane evaluation */
const volatile real32_T TJAOBF_MaxDiffLnLen2ObjPosX_C_met =
    4.0F; /* Referenced by:
           * '<S241>/Constant'
           * '<S241>/Constant2'
           */

/* Maximum longitudinal difference between detected lane length and acc object
 * position x for object  in lane evaluation */
const volatile real32_T TJAOBF_MaxDurObjBrdg_C_sec =
    5.0F; /* Referenced by: '<S186>/Constant2' */

/* Maximum allowed duration of object bridging mode at high velocities */
const volatile real32_T TJAOBF_MinDurAccObjValid_C_sec =
    3.0F; /* Referenced by:
           * '<S211>/Constant'
           * '<S212>/Constant'
           */

/* Minimum duration of ACC object validity for activation of object following
 * and combined mode */
const volatile real32_T TJAOBF_MinDurLCforOB_C_sec =
    10.0F; /* Referenced by: '<S186>/Constant' */

/* Minimum duration of Lane Centering availability to allow object bridging at
 * high velocities */
const volatile real32_T TJAOBF_MinDurObjLnValidWR_C_sec =
    0.5F; /* Referenced by: '<S219>/Constant' */

/* Minimum duration of object in lane validity as WR condition */
const volatile boolean_T TJAOBF_OF_Enabled_C_bool =
    1; /* Referenced by: '<S182>/Constant1' */

/*  Switch to enable object following mode */
const volatile boolean_T TJAOBF_ObjBrdgEnabled_C_bool =
    0; /* Referenced by: '<S186>/Constant1' */

/*  Switch to enable object briding at speeds greated than TJAOBF_VelXMax_kph */
const volatile uint16_T TJAOBF_PrjSpecQuC_C_btm =
    0U; /* Referenced by: '<S188>/Constant3' */

/* Bitmask to check project specific cancel conditions for object following */
const volatile uint16_T TJAOBF_PrjSpecQuSR_C_btm =
    0U; /* Referenced by: '<S188>/Constant1' */

/* Bitmask to check project specific SR conditions for object following */
const volatile uint16_T TJAOBF_PrjSpecQuWR_C_btm = 0U; /* Referenced by:
                                                        * '<S188>/Constant2'
                                                        * '<S486>/Constant2'
                                                        */

/* Bitmask to check project specific WR conditions for object following */
const volatile real32_T TJAOBF_TgtClthCrvMaxHyst_C_1pm =
    0.004F; /* Referenced by: '<S251>/Constant' */

/* Maximum allowed target object clothoid curvature for OF mode activation
 * (hysteresis) */
const volatile real32_T TJAOBF_TgtClthCrvMaxWR_Cr_rad[6] = {
    0.008F, 0.008F, 0.006F, 0.005F, 0.003F, 0.002F};
/* Referenced by: '<S251>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid curvature for OF mode
 * activation (WR condition) */
const volatile real32_T TJAOBF_TgtClthHeadMaxHyst_C_rad =
    0.02F; /* Referenced by: '<S252>/Constant' */

/* Maximum allowed target object clothoid heading angle for OF mode activation
 */
const volatile real32_T TJAOBF_TgtClthHeadMaxWR_Cr_rad[6] = {
    0.08F, 0.08F, 0.08F,
    0.08F, 0.08F, 0.08F}; /* Referenced by: '<S252>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid heading angle for OF
 * mode activation (WR condition) */
const volatile uint16_T TJAOBF_TgtClthInvalid_C_btm =
    193U; /* Referenced by: '<S250>/Constant1' */

/* Bit mask to check validity of ACC object clothoid generation */
const volatile real32_T TJAOBF_TgtClthLengthMin_C_met =
    0.0F; /* Referenced by: '<S253>/Constant3' */

/* Minimum required object trajectory length */
const volatile real32_T TJAOBF_TgtClthMinValidTime_C_sec =
    0.3F; /* Referenced by: '<S250>/Constant' */

/* Minimum validity time of target clothoid features for OF mode activation */
const volatile real32_T TJAOBF_TgtClthPosYMaxHyst_C_met =
    0.3F; /* Referenced by: '<S254>/Constant' */

/* Maximum allowed target object clothoid pos Y0 for OF mode activation
 * (hysteresis) */
const volatile real32_T TJAOBF_TgtClthPosYMaxWR_Cr_met[6] = {
    1.5F, 1.4F, 1.3F,
    1.2F, 1.1F, 1.0F}; /* Referenced by: '<S254>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid lateral position Y0
 * for OF mode activation (WR condition) */
const volatile real32_T TJAOBF_TgtVehDist2LnBndHst_C_met =
    0.4F; /* Referenced by:
           * '<S228>/Constant1'
           * '<S228>/Constant3'
           */

/* Target vehicle minimum distance to lane boundary hysteresis
   (Distance defined as vehicle center to lane boundary) */
const volatile real32_T TJAOBF_TgtVehDist2LnBndMin_C_met =
    0.6F; /* Referenced by:
           * '<S228>/Constant'
           * '<S228>/Constant2'
           */

/* Target vehicle minimum distance to lane boundary hysteresis
   (Distance defined as vehicle center to lane boundary) */
const volatile real32_T TJAOBF_VehVelX_Bx_kph[6] = {
    0.0F,  10.0F, 20.0F, 40.0F,
    60.0F, 80.0F}; /* Referenced by:
                    * '<S251>/1-D Lookup Table1'
                    * '<S252>/1-D Lookup Table1'
                    * '<S254>/1-D Lookup Table1'
                    */

/* 1DLookupTable-Bx-TJAOBF VehVelX  */
const volatile real32_T TJAOBF_VelXMax_C_kph = 60.0F; /* Referenced by:
                                                       * '<S39>/Constant8'
                                                       * '<S192>/Constant1'
                                                       */

/* Maximum longitudinal velocity allowed for object following mode */
const volatile real32_T TJAOBF_VelXMin_C_kph = 0.0F; /* Referenced by:
                                                      * '<S39>/Constant4'
                                                      * '<S192>/Constant3'
                                                      */

/*  Minimum longitudinal velocity required for object following mode */
const volatile real32_T TJAPARAM_VEH_Wheelbase_C_met =
    2.8F; /* Referenced by:
           * '<S957>/Constant7'
           * '<S363>/Constant4'
           */

/* Vehicle wheelbase */
const volatile real32_T TJAPARAM_VEH_Width_C_met = 1.84F; /* Referenced by:
                                                           * '<S362>/Constant1'
                                                           * '<S363>/Constant1'
                                                           */

/* Vehicle width */
const volatile real32_T TJASLC_AbortYHdSignTurnOnTi_sec =
    0.5F; /* Referenced by: '<S327>/Constant' */

/* Minimum ego lane length for activation of lane change */
const volatile uint16_T TJASLC_AdjLaneBridged_C_btm =
    4U; /* Referenced by: '<S269>/Constant3' */

/* Adjacent Lane Invalid Qualifier Bitmask */
const volatile uint16_T TJASLC_AdjLaneInvalid_C_btm =
    29U; /* Referenced by: '<S269>/Constant1' */

/* Adjacent Lane Invalid Qualifier Bitmask */
const volatile real32_T TJASLC_AdjLaneWidthMax_C_met =
    4.3F; /* Referenced by: '<S270>/Constant1' */

/* Maximum adjacent lane width */
const volatile real32_T TJASLC_AdjLaneWidthMin_C_met =
    2.65F; /* Referenced by: '<S270>/Constant3' */

/* Minimum adjacent lane width */
const volatile real32_T TJASLC_BlockTimeSALC_C_sec =
    5.0F; /* Referenced by: '<S484>/Constant1' */

/* SALC specific blocking time */
const volatile real32_T TJASLC_BlockTmSALCCancle_C_sec =
    0.5F; /* Referenced by: '<S484>/Constant2' */

/* Maximum long acceleration for TJA activation (hysteresis) */
const volatile boolean_T TJASLC_CheckAdjLanes_C_bool = 1; /* Referenced by:
                                                           * '<S269>/Constant5'
                                                           * '<S270>/Constant5'
                                                           */

/* Switch to enable adjacent lane check */
const volatile boolean_T TJASLC_CheckLaneTypes_C_bool =
    1; /* Referenced by: '<S306>/Constant' */

/* Enable check of lane types for lane change */
const volatile boolean_T TJASLC_CheckRearObjects_C_bool =
    1; /* Referenced by: '<S266>/Constant3' */

/* 0: No observation of rear objects
   1: Switch on rear object check */
const volatile boolean_T TJASLC_DisableAllowGoBack_bool =
    0; /* Referenced by:
        * '<S274>/Constant'
        * '<S279>/Constant'
        */

/* Switch to enable check of lane qualities for transition to object data during
 * combined mode */
const volatile real32_T TJASLC_DistAllowAbortHd_Mp_nu[12] = {
    1.0F,  1.0F,  1.02F, 1.05F, 1.08F, 1.1F,
    1.12F, 1.15F, 1.18F, 1.2F,  1.22F, 1.25F};
/* Referenced by: '<S299>/1-D Lookup Table5' */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJASLC_DistAllowAbortVelX_Mp_met[12] = {
    0.0F,  0.0F, 0.0F,  0.5F, 0.8F, 0.900000036F,
    0.95F, 1.0F, 1.05F, 1.1F, 1.2F, 1.2F};
/* Referenced by: '<S299>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJASLC_EgoCrvMaxActivation_C_1pm =
    0.02F; /* Referenced by: '<S305>/Constant' */

/* Maximum curvature of ego lane for activation of lane change */
const volatile real32_T TJASLC_EgoLengthMinActv_C_met =
    20.0F; /* Referenced by: '<S304>/Constant' */

/* Minimum ego lane length for activation of lane change */
const volatile boolean_T TJASLC_EnableFrontObjCancle_bool =
    1; /* Referenced by: '<S279>/Constant1' */

/* Debounce time to surpress "blinking" of the signal */
const volatile boolean_T TJASLC_EnableSLCHMISwitch_nu =
    1; /* Referenced by: '<S267>/Constant' */

/* Minimum long deceleration for TJA activation */
const volatile real32_T TJASLC_GoBackTurnLtDlyTiC_sec =
    0.6F; /* Referenced by:
           * '<S262>/Constant'
           * '<S262>/Constant1'
           */

/* Indicates TRUE if TJA is a coded function of the system */
const volatile real32_T TJASLC_HeadingAllowAbort_By_rad[12] = {
    0.0F,  0.005F, 0.01F, 0.015F, 0.02F, 0.025F,
    0.03F, 0.035F, 0.04F, 0.05F,  0.06F, 0.07F};
/* Referenced by: '<S299>/1-D Lookup Table5' */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJASLC_LCPInitDurationMax_C_sec =
    10.0F; /* Referenced by: '<S527>/Constant' */

/* Maximum duration of LCP initialization after which trigger will be reset, if
 * condition didn't allow initialization of lateral movement */
const volatile real32_T TJASLC_LCPLeft2Active_sec =
    0.0F; /* Referenced by: '<S317>/Constant' */

/* LCPLeft2Active time
   origin:2s */
const volatile real32_T TJASLC_LCPLeft2Passive_sec =
    10.0F; /* Referenced by: '<S317>/Constant1' */

/* LCPLeft2Passive time */
const volatile real32_T TJASLC_LCPRight2Active_sec =
    0.0F; /* Referenced by: '<S317>/Constant3' */

/* LCPRight2Active time
   origin:2s */
const volatile real32_T TJASLC_LCPRight2Passive_sec =
    10.0F; /* Referenced by: '<S317>/Constant2' */

/* LCPRight2Passive time */
const volatile real32_T TJASLC_LCWPassiveDlyTm_C_sec =
    0.2F; /* Referenced by: '<S375>/Constant12' */

/* Maximum long acceleration for TJA activation (hysteresis) */
const volatile real32_T TJASLC_LaneChangInfoTime_sec =
    3.0F; /* Referenced by:
           * '<S379>/Constant'
           * '<S380>/Constant'
           */

/* Maximum duration of takeover time for lane centering mode after lane change
 * maneuver */
const volatile real32_T TJASLC_LaneChangeWarnTimeMax_C_sec =
    2.0F; /* Referenced by:
           * '<S375>/Constant1'
           * '<S375>/Constant2'
           * '<S375>/Constant8'
           */

/* Maximum duration of warn time for lane change abort situation */
const volatile uint16_T TJASLC_LaneInvalid_C_btm =
    49151U; /* Referenced by: '<S307>/Constant1' */

/* Lane validity check bitmask */
const volatile real32_T TJASLC_LaneTypeDebTime_C_sec =
    0.5F; /* Referenced by: '<S306>/Constant2' */

/* Lane type check debounce time */
const volatile boolean_T TJASLC_LcaBsdSigEnabled_C_bool =
    1; /* Referenced by:
        * '<S266>/Constant'
        * '<S266>/Constant2'
        */

/* Switch to enable check of LCA/BSD signals for rear object assessment
   (if FALSE MS flag signals will be read) */
const volatile real32_T TJASLC_LnChngFlagTurnOffTm_C_sec =
    0.5F; /* Referenced by: '<S364>/Constant' */

/* Turn off delay time of ABPR lane change flag */
const volatile real32_T TJASLC_ManeuverTimeMax_C_sec =
    15.0F; /* Referenced by: '<S285>/Constant1' */

/* Maximum lane change maneuver time */
const volatile boolean_T TJASLC_ManualTrigger_C_bool =
    1; /* Referenced by: '<S513>/Constant1' */

/* Switch to allow manual triggering of lane change with turn signals only */
const volatile real32_T TJASLC_MinDurFreeAdjLane_C_sec =
    0.0F; /* Referenced by: '<S266>/Constant1' */

/* Minimum duration of an empty adjacent lane to allow initilization of lane
 * change maneuver */
const volatile real32_T TJASLC_MinDurLCTrigActv_C_sec =
    1.0F; /* Referenced by: '<S485>/Constant1' */

/* Minimum duration of controlling in lane centering mode required for SALC
 * trigger activation */
const volatile real32_T TJASLC_ObjSafeTime_C_sec = 0.2F; /* Referenced by:
                                                          * '<S266>/Constant7'
                                                          * '<S266>/Constant8'
                                                          * '<S266>/Constant9'
                                                          */

/* Debounce time to surpress "blinking" of the signal */
const volatile real32_T TJASLC_ObjUnSafeTime_C_sec = 0.5F; /* Referenced by:
                                                            * '<S266>/Constant4'
                                                            * '<S266>/Constant5'
                                                            * '<S266>/Constant6'
                                                            */

/* Debounce time to surpress "blinking" of the signal */
const volatile real32_T TJASLC_PosYThdExitAbort_met =
    0.5F; /* Referenced by: '<S327>/Constant3' */

/* Minimum ego lane length for activation of lane change */
const volatile uint16_T TJASLC_PrjSpecQuC_C_btm =
    0U; /* Referenced by: '<S273>/Constant1' */

/* Bitmask to check project specific cancel conditions for lane change mode */
const volatile uint16_T TJASLC_PrjSpecQuSR_C_btm =
    0U; /* Referenced by: '<S486>/Constant1' */

/* Bitmask to check project specific SR conditions for lane change mode */
const volatile boolean_T TJASLC_SALC_Enabled_C_bool = 1; /* Referenced by:
                                                          * '<S118>/Constant'
                                                          * '<S194>/Constant1'
                                                          * '<S463>/Constant1'
                                                          */

/* Switch to enable semi-automatic lane change functionality */
const volatile real32_T TJASLC_TakeoverTiMaxAbrt_C_sec =
    4.5F; /* Referenced by: '<S378>/Constant2' */

/* Turn off delay time of ABPR lane change flag */
const volatile real32_T TJASLC_TakeoverTimeMax_C_sec =
    3.0F; /* Referenced by: '<S378>/Constant' */

/* Maximum duration of takeover time for lane centering mode after lane change
 * maneuver */
const volatile real32_T TJASLC_TimeMaxInAbortNewEgo_sec =
    4.5F; /* Referenced by: '<S327>/Constant2' */

/* Minimum ego lane length for activation of lane change */
const volatile real32_T TJASLC_TimeMaxInAbort_sec =
    4.5F; /* Referenced by: '<S327>/Constant1' */

/* Minimum ego lane length for activation of lane change */
const volatile real32_T TJASLC_TrigResetBlockTime_C_sec =
    0.5F; /* Referenced by: '<S515>/Constant' */

/* Trigger blocking time after reset */
const volatile real32_T TJASLC_TriggerTurnOnTime_C_sec =
    1.0F; /* Referenced by: '<S516>/Constant' */

/* Trigger turn on delay time */
const volatile real32_T TJASLC_TurnSignalDebTime_C_sec =
    0.0F; /* Referenced by: '<S513>/Constant' */

/* Debounce time to surpress "blinking" of the signal */
const volatile real32_T TJASLC_TurnSignalOffTime_C_sec =
    0.3F; /* Referenced by:
           * '<S536>/Constant'
           * '<S536>/Constant1'
           */

/* The time to cancle or abort ALCA, when driver turn off turn lights */
const volatile uint8_T TJASLC_ValidLaneTypeDD_C_nu = 4U; /* Referenced by:
                                                          * '<S306>/Constant12'
                                                          * '<S306>/Constant6'
                                                          */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
const volatile uint8_T TJASLC_ValidLaneTypeDS_C_nu =
    6U; /* Referenced by: '<S306>/Constant11' */

/* Valid type selector (default: 2 (SOLID_DASHED_LINE)) */
const volatile uint8_T TJASLC_ValidLaneTypeSD_C_nu =
    7U; /* Referenced by: '<S306>/Constant8' */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
const volatile uint8_T TJASLC_ValidLaneTypeTD_C_nu = 8U; /* Referenced by:
                                                          * '<S306>/Constant10'
                                                          * '<S306>/Constant7'
                                                          */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
const volatile uint8_T TJASLC_ValidLaneType_C_nu = 2U; /* Referenced by:
                                                        * '<S306>/Constant4'
                                                        * '<S306>/Constant9'
                                                        */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
const volatile real32_T TJASLC_VehVelXAllowAbort_Bx_mps[12] = {
    0.0F,  5.0F,  10.0F, 15.0F, 20.0F, 25.0F,
    30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 60.0F};
/* Referenced by: '<S299>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJASLC_VelXMax_C_kph =
    115.0F; /* Referenced by: '<S488>/Constant1' */

/* Maximum velocity threshold for SALC activation */
const volatile real32_T TJASLC_VelXMin_C_kph =
    45.0F; /* Referenced by: '<S488>/Constant3' */

/* Minimum velocity threshold for SALC activation */
const volatile real32_T TJASTM_ACCManeveurTime_C_sec =
    0.5F; /* Referenced by: '<S546>/Constant7' */

/* Maximum long acceleration for TJA activation */
const volatile real32_T TJASTM_ACCOvertime_C_sec =
    0.5F; /* Referenced by: '<S546>/Constant' */

/* ACC Active over time check: sec */
const volatile boolean_T TJASTM_EnableFS_C_bool =
    0; /* Referenced by: '<S8>/Constant' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
const volatile boolean_T TJASTM_EnableUseACCState_bool =
    1; /* Referenced by: '<S555>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
const volatile real32_T TJASTM_HansOffReleaseWarn_C_sec =
    5.0F; /* Referenced by: '<S546>/Constant1' */

/* Turn on delay time for minimum lane quality check */
const volatile real32_T TJASTM_SusTakeOverTurnOffTime_C_sec =
    3.0F; /* Referenced by: '<S9>/Constant7' */

/* Turn off delay time for suspended take over */
const volatile real32_T TJASTM_SuspendQuitTimeMax_C_sec =
    150.0F; /* Referenced by: '<S600>/Constant' */

/*  Maximum rampout time */
const volatile real32_T TJATOW_NPilotAudioTime_C_sec =
    0.05F; /* Referenced by:
            * '<S613>/Constant13'
            * '<S613>/Constant14'
            * '<S613>/Constant18'
            * '<S613>/Constant2'
            * '<S613>/Constant20'
            * '<S613>/Constant29'
            * '<S613>/Constant31'
            * '<S613>/Constant33'
            * '<S613>/Constant35'
            * '<S613>/Constant37'
            * '<S613>/Constant39'
            * '<S613>/Constant5'
            * '<S613>/Constant6'
            * '<S613>/Constant7'
            * '<S613>/Constant9'
            */

/* Turn on delay time for minimum lane quality check */
const volatile uint8_T TJATOW_NPilotSysInfoDefault_nu =
    23U; /* Referenced by: '<S613>/Constant17' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
const volatile uint16_T TJATOW_OFSpdInvalid_C_btm =
    19U; /* Referenced by: '<S648>/Constant5' */

/* Bitmask to check project specific strong ready conditions for all TJA modes
 */
const volatile boolean_T TJATTG_EnableObjDuringCMB_C_bool =
    0; /* Referenced by: '<S765>/Constant' */

/* Switch to enable use of object data during combined mode */
const volatile boolean_T TJATTG_EnableVirtAdjLane_C_bool =
    0; /* Referenced by:
        * '<S832>/Constant'
        * '<S834>/Constant'
        */

/* Enable virtual adjacent lane width for SALC */
const volatile real32_T TJATTG_LnPredMinTrajLength_C_met =
    0.5F; /* Referenced by:
           * '<S687>/Constant'
           * '<S704>/Constant'
           * '<S721>/Constant'
           */

/* Minimum trajectory length for lane prediction */
const volatile boolean_T TJATTG_NewPredEnable_C_bool =
    0; /* Referenced by: '<S686>/Constant' */

/* Switch to enable new prediction implementation */
const volatile real32_T TJATTG_ObjFolVirtLnWdth_C_met =
    3.2F; /* Referenced by:
           * '<S671>/Constant1'
           * '<S672>/Constant1'
           * '<S744>/Constant1'
           * '<S809>/Constant1'
           */

/* TJATTG_APARAM */
const volatile real32_T TJATTG_PredCrvChngPT1_C_sec =
    0.2F; /* Referenced by:
           * '<S688>/Constant1'
           * '<S705>/Constant1'
           * '<S722>/Constant1'
           */

/* Curvature change PT1 time constant during prediction (if signal is set to
 * zero) */
const volatile real32_T TJATTG_PredCrvPT1_C_sec = 0.2F; /* Referenced by:
                                                         * '<S689>/Constant1'
                                                         * '<S706>/Constant1'
                                                         * '<S723>/Constant1'
                                                         */

/* Curvature PT1 time constant during prediction (if signal is set to zero) */
const volatile boolean_T TJATTG_PredFreezeCrvChng_C_bool =
    0; /* Referenced by:
        * '<S688>/Constant'
        * '<S705>/Constant'
        * '<S722>/Constant'
        */

/* TRUE if curvature change signal shall be frozen during prediction */
const volatile boolean_T TJATTG_PredFreezeCrv_C_bool = 1; /* Referenced by:
                                                           * '<S689>/Constant'
                                                           * '<S706>/Constant'
                                                           * '<S723>/Constant'
                                                           */

/* TRUE if curvature signal shall be frozen during prediction */
const volatile real32_T TJATTG_PredResetTrajLength_C_met =
    10.0F; /* Referenced by:
            * '<S695>/Constant'
            * '<S712>/Constant'
            * '<S729>/Constant'
            */

/* Remaining saved trajectory length for prediction reset */
const volatile real32_T TJATTG_TransDurationCMB_C_sec =
    2.5F; /* Referenced by: '<S804>/Constant4' */

/* Transition duration for mode switch to combined data */
const volatile real32_T TJATTG_TransDurationLD_C_sec =
    2.5F; /* Referenced by: '<S804>/Constant2' */

/* Transition duration for mode switch to lane data */
const volatile real32_T TJATTG_TransDurationOD_C_sec =
    2.5F; /* Referenced by: '<S804>/Constant3' */

/*  Transition duration for mode switch to object data */
const volatile real32_T TJATTG_TransDurationPredct_C_sec =
    0.0F; /* Referenced by:
           * '<S963>/Constant2'
           * '<S804>/Constant'
           */

/* Transition duration to switch from predicted lane data to new detected lane
 * data */
const volatile boolean_T TJATTG_TransHandleEnable_C_bool =
    0; /* Referenced by: '<S798>/Constant' */

/* Switch to enable mode transition handling */
const volatile real32_T TJATTG_VirtAdjLaneWidth_C_met =
    0.0F; /* Referenced by:
           * '<S832>/Constant1'
           * '<S834>/Constant1'
           */

/* Virtual adjacent lane width */
const volatile real32_T TJATVG_CrvPlanHorizon_Bx_1pm[7] = {
    0.0F,   0.0001F, 0.0005F, 0.001F,
    0.002F, 0.003F,  0.005F}; /* Referenced by:
                               * '<S947>/1-D Lookup Table1'
                               * '<S949>/1-D Lookup Table1'
                               */

/* 1DLookupTable-Bx-X axis for the Planning Horizon */
const volatile real32_T TJATVG_DistYToleranceLeftTgtArea_C_met =
    0.05F; /* Referenced by: '<S891>/Constant15' */

/* TJATVG_PARAMETER- Distance Y to tolerance left target area */
const volatile real32_T TJATVG_DistYToleranceRightTgtArea_C_met =
    0.05F; /* Referenced by: '<S891>/Constant16' */

/* TJATVG_PARAMETER- Distance Y to tolerance right target area */
const volatile real32_T TJATVG_FTireAccelMax_C_mps2 =
    2.0F; /* Referenced by: '<S891>/Constant13' */

/* TJATVG_PARAMETER- FTire acceleration maximum */
const volatile real32_T TJATVG_FTireAccelMin_C_mps2 =
    0.0F; /* Referenced by: '<S891>/Constant14' */

/* TJATVG_PARAMETER- FTire acceleration minimum */
const volatile real32_T TJATVG_FactorCrvGrdBuildUp_C_fac =
    1.2F; /* Referenced by: '<S957>/Constant8' */

/* TJATVG_PARAMETER-Factor of Curve Gradient Buildup */
const volatile real32_T TJATVG_FactorCrvGrdRed_C_fac =
    1.8F; /* Referenced by: '<S957>/Constant9' */

/* TJATVG_PARAMETER-Factor of Curve Gradient Red */
const volatile real32_T TJATVG_GrdLimitTgtCrvTGC_C_1pms =
    0.1F; /* Referenced by: '<S957>/Constant11' */

/* TJATVG_PARAMETER-Gradient Limit Target Curve TGC */
const volatile boolean_T TJATVG_HighStatAccu_C_bool =
    0; /* Referenced by: '<S893>/Constant1' */

/* Indicates TRUE, if high stationary accuracy is required */
const volatile real32_T TJATVG_LWDeratLvlScalFact_Cr_nu[6] = {
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F}; /* Referenced by: '<S891>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Lookup table for lane width derating level scaling factort
 */
const volatile real32_T TJATVG_LaneWidth_Bx_met[6] = {
    0.0F, 1.0F, 2.0F,
    3.0F, 4.0F, 5.0F}; /* Referenced by: '<S891>/1-D Lookup Table' */

/* 1DLookupTable-Bx-Lane width

   The raw value is [0 0 0 0 0 0], but this BreakpointsForDimension1 of
   1DLookUp table must be strictly monotonically increasing,  so set it to [0 1
   2 3 4 5] Temporarily. */
const volatile real32_T TJATVG_LimiterMaxCrvGrd_C_1pms =
    0.1F; /* Referenced by:
           * '<S956>/Constant2'
           * '<S956>/Constant3'
           * '<S956>/Constant4'
           * '<S957>/Constant2'
           * '<S957>/Constant3'
           * '<S957>/Constant4'
           * '<S958>/Constant2'
           * '<S958>/Constant3'
           * '<S958>/Constant4'
           */

/* TJATVG_PARAMETER-Max Curvature Gradient */
const volatile real32_T TJATVG_LimiterMaxCrv_C_1pm =
    0.15F; /* Referenced by:
            * '<S956>/Constant1'
            * '<S957>/Constant1'
            * '<S958>/Constant1'
            */

/* TJATVG_PARAMETER-Max Curvature */
const volatile uint8_T TJATVG_MD1DeratingLevel_C_perc =
    0U; /* Referenced by: '<S891>/Constant22' */

/* MD1DeratingLevel */
const volatile uint8_T TJATVG_MD2DeratingLevel_C_perc =
    0U; /* Referenced by: '<S891>/Constant21' */

/* MD2DeratingLevel */
const volatile uint8_T TJATVG_MD3DeratingLevel_C_perc =
    0U; /* Referenced by: '<S891>/Constant23' */

/* MD3DeratingLevel */
const volatile real32_T TJATVG_MaxJerkAllowed_C_mps3 =
    2.0F; /* Referenced by: '<S891>/Constant12' */

/* Maximum allowed jerk */
const volatile real32_T TJATVG_MaxSteeringAngle_C_deg =
    3.0F; /* Referenced by: '<S957>/Constant6' */

/* TJATVG_PARAMETER-Max Steering Angle */
const volatile real32_T TJATVG_MaxTrqScalLimit_C_nu =
    100.0F; /* Referenced by: '<S962>/Constant1' */

/* TJATVG_APARAM-Max torque scal limit */
const volatile real32_T TJATVG_MaxTrqScalRampInGrd_C_1ps =
    100.0F; /* Referenced by: '<S961>/Constant9' */

/* Maximum torque scaling gradient during ramp-in */
const volatile real32_T TJATVG_MaxTrqScalRampOutGrd_C_1ps =
    100.0F; /* Referenced by: '<S961>/Constant10' */

/* Maximum torque scaling gradient during ramp-out */
const volatile real32_T TJATVG_MinFactorCrvGrd_C_fac =
    0.5F; /* Referenced by: '<S957>/Constant10' */

/* TJATVG_PARAMETER-Min Factor of Curve Gradient  */
const volatile boolean_T TJATVG_ModeTransTrigReplan_bool =
    1; /* Referenced by: '<S888>/Constant' */

/* Switch to enable trigger replan during mode transitions */
const volatile real32_T TJATVG_PlanHorizonLChange_Vel_sec[15] = {
    5.0F, 5.0F, 5.0F, 5.0F, 5.0F, 5.0F, 5.0F, 5.0F,
    5.0F, 5.0F, 5.0F, 5.0F, 7.0F, 8.0F, 8.0F}; /* Referenced by: '<S948>/1-D
                                                  Lookup Table' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid curvature for OF mode
 * activation (WR condition) */
const volatile real32_T TJATVG_PlanHorizonObjFolVal_Cr_sec[15] = {
    50.0F, 50.0F, 50.0F, 50.0F, 40.0F, 20.0F, 16.0F, 12.0F,
    10.0F, 8.0F,  5.0F,  4.5F,  4.5F,  4.5F,  4.5F}; /* Referenced by:
                                                        '<S949>/1-D Lookup
                                                        Table' */

/* 1DLookupTable-Cr-Lookup table for vehicle speed dependend planning horizon
 * for OF */
const volatile real32_T TJATVG_PlanHorizonScal_Cr_Fac[7] = {
    1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F}; /* Referenced by:
                        * '<S947>/1-D Lookup Table1'
                        * '<S949>/1-D Lookup Table1'
                        */

/* 1DLookupTable-Cr-Y axis of the Planning Horizon scaling factor */
const volatile real32_T TJATVG_PlanningHorizonValLC_Cr_sec[15] = {
    50.0F, 50.0F, 50.0F, 50.0F, 40.0F, 20.0F, 16.0F, 12.0F,
    10.0F, 8.0F,  5.0F,  4.5F,  4.5F,  4.5F,  4.5F}; /* Referenced by:
                                                        '<S947>/1-D Lookup
                                                        Table' */

/* 1DLookupTable-Cr-Lookup table for vehicle speed dependend planning horizon */
const volatile real32_T TJATVG_PosYPlanHorizonScal_Cr_Fac[7] = {
    1.0F, 1.1F, 1.2F, 1.2F,
    1.2F, 1.2F, 1.2F}; /* Referenced by:
                        * '<S947>/1-D Lookup Table3'
                        * '<S949>/1-D Lookup Table3'
                        */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJATVG_PosYPlanHorizon_Bx_met[7] = {
    0.0F, 0.5F, 0.7F, 0.9F,
    1.2F, 1.5F, 1.8F}; /* Referenced by:
                        * '<S947>/1-D Lookup Table3'
                        * '<S949>/1-D Lookup Table3'
                        */

/* 1DLookupTable-Bx-Vehicle velocity X */
const volatile real32_T TJATVG_PredTimeHeadAng_C_sec =
    0.2F; /* Referenced by: '<S891>/Constant10' */

/* TJATVG_PARAMETER-Prediction time of heading angle */
const volatile real32_T TJATVG_PredictionTimeCrv_C_sec =
    0.2F; /* Referenced by: '<S891>/Constant4' */

/* TJATVG_PARAMETER-Prediction time of curve */
const volatile real32_T TJATVG_RedFact_Vel_Cr_fac[6] = {
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F}; /* Referenced by: '<S893>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Y_TJATVG_RedFact_Vel_nu */
const volatile boolean_T TJATVG_SetMaxCrvAndGrdLims_C_bool =
    0; /* Referenced by: '<S957>/Constant' */

/* TJATVG_PARAMETER-Set max curve and grandient limit */
const volatile real32_T TJATVG_StrWhStifAbortGrd_C_1ps =
    10000.0F; /* Referenced by: '<S961>/Constant6' */

/* Steering Wheel Stiffness Abort Ramp Out Gradient */
const volatile real32_T TJATVG_StrWhStifLimitPredct_C_fac =
    0.1F; /* Referenced by: '<S963>/Constant5' */

/* TJA specific steering wheel stiffness limiter during rampout prediction */
const volatile real32_T TJATVG_StrWhStifLimit_C_nu =
    100.0F; /* Referenced by: '<S962>/Constant6' */

/* TJA specific steering wheel stiffness limiter */
const volatile real32_T TJATVG_StrWhStifRampInGrd_C_1ps =
    100.0F; /* Referenced by:
             * '<S961>/Constant4'
             * '<S963>/Constant6'
             */

/* Steering wheel stiffness gradient during ramp-in */
const volatile real32_T TJATVG_StrWhStifRampOutGrd_C_1ps =
    100.0F; /* Referenced by:
             * '<S961>/Constant1'
             * '<S963>/Constant7'
             */

/* Steering wheel stiffness gradient during ramp-out */
const volatile uint8_T TJATVG_TrajPlanValServQu_C_nu =
    12U; /* Referenced by: '<S891>/Constant20' */

/* Trajectory planning service qualifier */
const volatile uint8_T TJATVG_TrajPlanValSrvQuSALC_C_nu =
    12U; /* Referenced by: '<S891>/Constant19' */

/* Trajectory planning service qualifier for semi-automatic lane change
   (enables lane cross check) */
const volatile boolean_T TJATVG_TriggerReplan_C_bool =
    1; /* Referenced by: '<S891>/Constant11' */

/* Switch to trigger replanning */
const volatile real32_T TJATVG_TrqAbortGrad_C_1ps =
    10000.0F; /* Referenced by: '<S961>/Constant3' */

/* Torque Ramp Abort Ramp Out Gradient */
const volatile real32_T TJATVG_TrqRampInGrad_C_1ps =
    75.0F; /* Referenced by: '<S961>/Constant11' */

/* Torque ramp gradient during ramp-in */
const volatile real32_T TJATVG_TrqRampOutGrad_C_1ps =
    75.0F; /* Referenced by: '<S961>/Constant2' */

/* Torque ramp gradient during ramp-out */
const volatile boolean_T TJATVG_UseLtcyCompCMB_C_bool =
    0; /* Referenced by: '<S924>/Constant2' */

/* Switch to enable use of latency compensation during CMB mode */
const volatile boolean_T TJATVG_UseLtcyCompLC_C_bool =
    0; /* Referenced by: '<S924>/Constant' */

/* Switch to enable use of latency compensation during LC mode */
const volatile boolean_T TJATVG_UseLtcyCompOF_C_bool =
    0; /* Referenced by: '<S924>/Constant1' */

/* Switch to enable use of latency compensation during OF mode */
const volatile boolean_T TJATVG_UseLtcyCompSALC_C_bool =
    0; /* Referenced by: '<S924>/Constant3' */

/* Switch to enable use of latency compensation during SALC mode */
const volatile real32_T TJATVG_VehVelXLC_Bx_mps[15] = {
    0.0F, 0.5F, 1.0F, 1.5F,  2.0F,  2.5F,  3.0F, 3.5F,
    4.0F, 4.5F, 5.0F, 10.0F, 20.0F, 30.0F, 40.0F}; /* Referenced by:
                                                    * '<S947>/1-D Lookup Table'
                                                    * '<S948>/1-D Lookup Table'
                                                    */

/* 1DLookupTable-Bx-TJATVG VehVelX  */
const volatile real32_T TJATVG_VehVelXOF_Bx_mps[15] = {
    0.0F, 0.5F, 1.0F, 1.5F,  2.0F,  2.5F,  3.0F, 3.5F,
    4.0F, 4.5F, 5.0F, 10.0F, 20.0F, 30.0F, 40.0F};
/* Referenced by: '<S949>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJATVG VehVelX  */
const volatile real32_T TJATVG_VehVelX_RedFact_Bx_mps[6] = {
    0.0F, 1.0F, 2.0F,
    3.0F, 4.0F, 5.0F}; /* Referenced by: '<S893>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJATVG_APARAM for Y_TJATVG_RedFact_Vel_nu

   The raw value is [0 0 0 0 0 0], but this BreakpointsForDimension1 of
   1DLookUp table must be strictly monotonically increasing,  so set it to [0 1
   2 3 4 5] Temporarily. */
const volatile real32_T TJATVG_WeightEndTime_C_nu =
    0.001F; /* Referenced by: '<S891>/Constant18' */

/* TJATVG_PARAMETER- Weight of end time */
const volatile real32_T TJATVG_WeightTgtDistY_C_nu =
    0.1F; /* Referenced by: '<S891>/Constant17' */

/* TJATVG_PARAMETER- Weight of target distance Y  */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h"
/* Named constants for Chart: '<S314>/AbortState' */
#define TJASA_IN_Abort ((uint8_T)1U)
#define TJASA_IN_AbortNewEgo ((uint8_T)2U)
#define TJASA_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define TJASA_IN_NoActive ((uint8_T)1U)
#define TJASA_IN_OriginLaneAbort ((uint8_T)2U)

/* Named constants for Chart: '<S264>/ManeuverState' */
#define IN_LaneChangeProcedureStartLeft ((uint8_T)3U)
#define IN_LaneChangeProcedureStartRigh ((uint8_T)4U)
#define TJASA_IN_ActiveLeft ((uint8_T)1U)
#define TJASA_IN_ActiveRight ((uint8_T)2U)
#define TJASA_IN_LaneChangeManeuverEnd ((uint8_T)1U)
#define TJASA_IN_LateralMovementStart ((uint8_T)3U)
#define TJASA_IN_NewEgoLane ((uint8_T)5U)
#define TJASA_IN_NewEgoLane_grb0 ((uint8_T)2U)
#define TJASA_IN_Passive ((uint8_T)6U)
#define TJASA_IN_ReadyToTriggerLeft ((uint8_T)7U)
#define TJASA_IN_ReadyToTriggerRight ((uint8_T)8U)
#define TJAS_IN_LaneChangeManeuverStart ((uint8_T)2U)

/* Named constants for Chart: '<S8>/LatCtrlMode' */
#define TJASA_IN_Combined ((uint8_T)1U)
#define TJASA_IN_Controlling ((uint8_T)1U)
#define TJASA_IN_LaneCentering ((uint8_T)2U)
#define TJASA_IN_LaneChange ((uint8_T)3U)
#define TJASA_IN_ObjectFollowing ((uint8_T)4U)
#define TJASA_IN_Passive_nudl ((uint8_T)2U)

/* Named constants for Chart: '<S8>/StateMachineTJA' */
#define TJASA_IN_ACTIVE ((uint8_T)1U)
#define TJASA_IN_CONTROLLING ((uint8_T)1U)
#define TJASA_IN_ERROR ((uint8_T)2U)
#define TJASA_IN_NOT_ACTIVE ((uint8_T)3U)
#define TJASA_IN_NOT_PRESENT ((uint8_T)4U)
#define TJASA_IN_OFF ((uint8_T)5U)
#define TJASA_IN_PASSIVE ((uint8_T)1U)
#define TJASA_IN_STANDBY ((uint8_T)2U)
#define TJASA_IN_SUSPENDED ((uint8_T)2U)
#define TJASA_IN_WAITACC ((uint8_T)3U)

const Bus_TgtTrajAndCridrBnd_nu TJASA_rtZBus_TgtTrajAndCridrBnd_nu = {
    {
        0.0F, /* PosX0_met */
        0.0F, /* PosY0_met */
        0.0F, /* Heading_rad */
        0.0F, /* Crv_1pm */
        0.0F, /* CrvChng_1pm2 */
        0.0F  /* Length_met */
    },        /* TTG_LeftCorridorClothoid_bus */

    {
        0.0F, /* PosX0_met */
        0.0F, /* PosY0_met */
        0.0F, /* Heading_rad */
        0.0F, /* Crv_1pm */
        0.0F, /* CrvChng_1pm2 */
        0.0F  /* Length_met */
    },        /* TTG_RightCorridorClothoid_bus */

    {
        0.0F, /* PosX0_met */
        0.0F, /* PosY0_met */
        0.0F, /* Heading_rad */
        0.0F, /* Crv_1pm */
        0.0F, /* CrvChng_1pm2 */
        0.0F  /* Length_met */
    }         /* TTG_TargetTrajectory_bus */
};            /* Bus_TgtTrajAndCridrBnd_nu ground */

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T TJACMB_LaneCrvStdDev_nu;        /* '<S12>/Abs'
                                          * TJACMB lane curve standard deviation
                                            DT:float32
                                          */
real32_T TJACMB_TraceCrvStdDev_nu;       /* '<S13>/Abs'
                                          * TJACMB trace curve standard deviation
                                            DT:float32
                                          */
real32_T TJATVG_DistYTolLeTgtArea_met;   /* '<S891>/Constant15'
                                          * TJATVG DistYTolLeTgtArea
                                            DT:float32
                                          */
real32_T TJATVG_DistYTolRiTgtArea_met;   /* '<S891>/Constant16'
                                          * TJATVG DistYTolRiTgtArea
                                            DT:float32
                                          */
real32_T TJATVG_FTireAclMax_mps2;        /* '<S891>/Constant13'
                                          * TJATVG FTireAclMax
                                            DT:float32
                                          */
real32_T TJATVG_FTireAclMin_mps2;        /* '<S891>/Constant14'
                                          * TJATVG FTireAclMin
                                            DT:float32
                                          */
real32_T TJATVG_WeightTgtDistY_nu;       /* '<S891>/Constant17'
                                          * TJATVG weighted target distY
                                            DT:float32
                                          */
real32_T TJATVG_WeightEndTime_nu;        /* '<S891>/Constant18'
                                          * TJATVG weighted end time
                                            DT:float32
                                          */
real32_T TJATVG_PredTimeCrv_sec;         /* '<S891>/Constant4'
                                          * TJATVG predict time of curve
                                            DT:float32
                                          */
real32_T TJATVG_PredTimeHeadAng_sec;     /* '<S891>/Constant10'
                                          * TJATVG predict time of heading
                                            DT:float32
                                          */
real32_T TJATVG_MaxTrqScalLimit_nu;      /* '<S962>/Constant1'
                                          * TJATVG max toque scal limit
                                            DT:float32
                                          */
real32_T TJATVG_MaxJerkAllowed_mps3;     /* '<S891>/Constant12'
                                          * TJATVG max jerk allowed
                                            DT:float32
                                          */
real32_T TJACMB_CombinedPosY0_met;       /* '<S22>/Add'
                                          * TJACMB combined PosY0
                                            DT:float32
                                          */
real32_T TJACMB_CombinedPosX0_met;       /* '<S14>/Max'
                                          * TJACMB combined PosX0
                                            DT:float32
                                          */
real32_T TJACMB_CombinedHeading_rad;     /* '<S21>/Add'
                                          * TJACMB combined Heading
                                            DT:float32
                                          */
real32_T TJACMB_CombinedCrv_1pm;         /* '<S19>/Switch'
                                          * TJACMB combined Curve
                                            DT:float32
                                          */
real32_T TJACMB_CombinedCrvChng_1pm2;    /* '<S20>/Add'
                                          * TJACMB combined Curve of change
                                            DT:float32
                                          */
real32_T TJACMB_CombinedLength_met;      /* '<S14>/Min'
                                          * TJACMB combined length
                                            DT:float32
                                          */
real32_T TJATTG_LeCridrBndPosX0_met;     /* '<S10>/Signal Conversion12'
                                          * TJATTG left corridor boundary PosX0
                                            DT:float32
                                          */
real32_T TJATTG_LeCridrBndPosY0_met;     /* '<S10>/Signal Conversion13'
                                          * TJATTG left corridor boundary PosY0
                                            DT:float32
                                          */
real32_T TJATTG_LeCridrBndHeadAng_rad;   /* '<S10>/Signal Conversion14'
                                          * TJATTG left corridor boundary heading
                                            DT:float32
                                          */
real32_T TJATTG_LeCridrBndCrv_1pm;       /* '<S10>/Signal Conversion15'
                                          * TJATTG left corridor boundary curve
                                            DT:float32
                                          */
real32_T TJATTG_LeCridrBndCrvChng_1pm2;  /* '<S10>/Signal Conversion16'
                                          * TJATTG left corridor boundary curve
                                          of change  DT:float32
                                          */
real32_T TJATTG_LeCridrBndLength_met;    /* '<S10>/Signal Conversion17'
                                          * TJATTG left corridor boundary length
                                            DT:float32
                                          */
real32_T TJATTG_RiCridrBndPosX0_met;     /* '<S10>/Signal Conversion18'
                                          * TJATTG right corridor boundary PosX0
                                            DT:float32
                                          */
real32_T TJATTG_RiCridrBndPosY0_met;     /* '<S10>/Signal Conversion19'
                                          * TJATTG right corridor boundary PosY0
                                            DT:float32
                                          */
real32_T TJATTG_RiCridrBndHeadAng_rad;   /* '<S10>/Signal Conversion20'
                                          * TJATTG right corridor boundary heading
                                            DT:float32
                                          */
real32_T TJATTG_RiCridrBndCrv_1pm;       /* '<S10>/Signal Conversion21'
                                          * TJATTG right corridor boundary curve
                                            DT:float32
                                          */
real32_T TJATTG_RiCridrBndCrvChng_1pm2;  /* '<S10>/Signal Conversion22'
                                          * TJATTG right corridor boundary curve
                                          of change  DT:float32
                                          */
real32_T TJATTG_RiCridrBndLength_met;    /* '<S10>/Signal Conversion23'
                                          * TJATTG right corridor boundary length
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajPosX0_met;        /* '<S10>/Signal Conversion24'
                                          * TJATTG target trajectory PosX0
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajPosY0_met;        /* '<S10>/Signal Conversion25'
                                          * TJATTG target trajectory PosY0
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajCrv_1pm;          /* '<S10>/Signal Conversion27'
                                          * TJATTG target trajectory curve
                                            DT:float32
                                          */
real32_T TJATVG_PlanningHorizon_sec;     /* '<S890>/Multiport Switch'
                                          * TJATVG planning horizon time
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajHeadAng_rad;      /* '<S10>/Signal Conversion26'
                                          * TJATTG target trajectory heading
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajCrvChng_1pm2;     /* '<S10>/Signal Conversion28'
                                          * TJATTG target trajectory curve of change
                                            DT:float32
                                          */
real32_T TJATTG_TgtTrajLength_met;       /* '<S10>/Signal Conversion29'
                                          * TJATTG target trajectory length
                                            DT:float32
                                          */
real32_T TJATVG_StrWhStifLimit_nu;       /* '<S962>/Product'
                                          * TJATVG steer wheel limit
                                            DT:float32
                                          */
real32_T TJATVG_TrqRampGrad_1ps;         /* '<S961>/Multiport Switch2'
                                          * TJATVG torque ramp gradient
                                            DT:float32
                                          */
real32_T TJATVG_StrWhStifGrad_1ps;       /* '<S961>/Multiport Switch'
                                          * TJATVG steer wheel stif gradient
                                            DT:float32
                                          */
real32_T TJATVG_MaxTrqScalGrad_1ps;      /* '<S961>/Multiport Switch1'
                                          * TJATVG steer wheel scal gradient
                                            DT:float32
                                          */
real32_T TJATVG_MaxCrvTrajGuiCtl_1pm;    /* '<S892>/Switch'
                                          * TJATVG max curve trajectory control
                                            DT:float32
                                          */
real32_T TJATVG_MaxCrvGrdBuildup_1pms;   /* '<S892>/Switch'
                                          * TJATVG max curve gradient buildup
                                            DT:float32
                                          */
real32_T TJATVG_MaxCrvGrdRed_1pms;       /* '<S892>/Switch'
                                          * TJATVG max curve gradient red
                                            DT:float32
                                          */
real32_T TJATVG_MaxCrvGrdTGC_1pms;       /* '<S892>/Switch'
                                          * TJATVG max curve gradient TGC
                                            DT:float32
                                          */
real32_T TJATVG_SensorTStamp_sec;        /* '<S889>/Multiport Switch'
                                          * TJATVG sensor timestamp
                                            DT:float32
                                          */
real32_T TJATVG_ObstacleVelX_mps;        /* '<S891>/Constant'
                                          * TJATVG obstacle velX
                                            DT:float32
                                          */
real32_T TJATVG_ObstacleAclX_mps2;       /* '<S891>/Constant2'
                                          * TJATVG obstacle AclX
                                            DT:float32
                                          */
real32_T TJATVG_ObstacleWidth_met;       /* '<S891>/Constant3'
                                          * TJATVG obstacle width
                                            DT:float32
                                          */
real32_T TJATVG_ObstacleDistX_met;       /* '<S891>/Constant8'
                                          * TJATVG obstacle DisX
                                            DT:float32
                                          */
real32_T TJATVG_ObstacleDistY_met;       /* '<S891>/Constant9'
                                          * TJATVG obstacle DisY
                                            DT:float32
                                          */
real32_T TJATVG_LimiterTimeDuration_sec; /* '<S891>/Constant7'
                                          * TJATVG limiter time duration
                                            DT:float32
                                          */
int32_T TJASLC_SLCHighLightID_nu;        /* '<S375>/Switch3'
                                          * TJATVG obstacle DisX
                                            DT:float32
                                          */
uint16_T TJATTG_TgtCorridorInvalid_btf;  /* '<S681>/Data Type Conversion1'
                                          * TJASTM target corridor invalid
                                          bitfield  DT:uint16
                                          */
uint16_T TJACMB_CombinedInvalid_btf;     /* '<S40>/Data Type Conversion1'
                                          * TJACMB combined invalid bitfield
                                            DT:uint16
                                          */
uint16_T TJASTM_TJAInvalid_btf;          /* '<S572>/Data Type Conversion1'
                                          * TJASTM TJA invalid bitfield
                                            DT:uint16
                                          */
uint16_T TJAGEN_SuspendedAndQuit_debug;  /* '<S78>/Data Type Conversion1'
                                          * TJAGEN Suspended And Quit debug
                                          */
uint16_T TJAOBF_ObjFollowInvalid_btf;    /* '<S200>/Data Type Conversion1'
                                          * TJAOBF object  following  invalid
                                          bitfield    DT:uint16
                                          */
uint16_T TJASLC_TriggerInvalid_btf;      /* '<S525>/Data Type Conversion1'
                                          * SLC trigger invalid bitfield
                                            DT:uint16
                                          */
uint16_T TJASLC_RiLaneChangeInvalid_btf; /* '<S469>/Data Type Conversion1'
                                          * SLC right lane change invalid
                                          bitfield DT:uint16
                                          */
uint16_T TJASLC_LeLaneChangeInvalid_btf; /* '<S468>/Data Type Conversion1'
                                          * SLC left lane change invalid
                                          bitfield DT:uint16
                                          */
uint16_T TJAOBF_ObjInLaneInvalid_btf;    /* '<S218>/Data Type Conversion1'
                                          * TJAOBF object in lane invalid bitfield
                                            DT:uint16
                                          */
uint16_T TJALKA_LaneCenterInvalid_btf;   /* '<S136>/Data Type Conversion1'
                                          * TJALKA lane center invalid bitfield
                                            DT:uint16
                                          */
uint8_T TJATVG_DeratingLevel_nu;         /* '<S891>/Product'
                                          * TJATVG derating level
                                            DT:uint8
                                          */
uint8_T TJAGEN_StrongReadyInvalid_btf;   /* '<S96>/Data Type Conversion1'
                                          * TJAGEN strong ready invalid bitfield
                                            DT:uint8
                                          */
uint8_T TJALKA_LnIncoherenceStatus_nu;   /* '<S128>/Switch1'
                                          * TJALKA Lane incoherence status
                                            DT:uint8
                                          */
uint8_T TJASLC_LaneChangeInfo;           /* '<S374>/Switch1'
                                          * SLC weak ready flag
                                            DT:boolean
                                          */
uint8_T TJATVG_TrajPlanServQu_nu;        /* '<S891>/Switch'
                                          * TJATVG trajectory plan serv qulifier
                                            DT:uint8
                                          */
uint8_T TJAGEN_CancelStatus_btf;         /* '<S56>/Data Type Conversion1'
                                          * TJAGEN cancel status bitfield
                                            DT:uint8
                                          */
uint8_T TJAGEN_WeakReadyInvalid_btf;     /* '<S100>/Data Type Conversion1'
                                          * TJAGEN weak ready invalid bitfield
                                            DT:uint8
                                          */
uint8_T TJATOW_DriverTakeOverWarning_nu; /* '<S9>/Data Type Conversion1'
                                          * TJATOW takeover warning
                                            DT:uint8
                                          */
uint8_T TJASTM_NpilotSysInfo;            /* '<S613>/Switch' */
uint8_T TJALKA_LaneIncoherence_btf;      /* '<S133>/Data Type Conversion1'
                                          * TJALKA Lane incoherence debug
                                            DT:uint8
                                          */
uint8_T TJASTM_PilotAudioPlay;           /* '<S613>/Switch5' */
uint8_T
    TJASTM_LatCtrlHandsOffReleaseWarn_nu;   /* '<S546>/Data Type Conversion3' */
uint8_T TJASTM_PilotDisableACCSwitch_nu;    /* '<S546>/Data Type Conversion2' */
uint8_T TJASTM_PilotEnableACCSwitch_nu;     /* '<S546>/Data Type Conversion' */
uint8_T TJASLC_LaneChangWarning_nu;         /* '<S375>/Data Type Conversion'
                                             * lane change warning side
                                               0: no warning
                                               1: left warning
                                               2: right warning
                                             */
uint8_T TJASLC_SLCAudioPlay_nu;             /* '<S322>/Switch' */
uint8_T TJASLC_CancelAbort_btf;             /* '<S294>/Data Type Conversion1'
                                             * SLC cancel abort bitfield
                                               DT:uint8
                                             */
uint8_T TJASLC_TurnLtDirctionReq_nu;        /* '<S7>/Data Type Conversion'
                                             * Turn light diercetion reques:
                                               0: norequest 1: left; 2: Right
                                               DT:uint8
                                             */
uint8_T TJAOBF_TgtObjDataInvalid_btf;       /* '<S255>/Data Type Conversion1'
                                             * TJAOBF object data invalid bitfield
                                               DT:uint8
                                             */
uint8_T TJALKA_LnQualityInv_btf;            /* '<S181>/Data Type Conversion1'
                                             * TJALKA lane quality invalid bitfield
                                               DT:uint8
                                             */
uint8_T TJATVG_CrvAmplActivated_nu;         /* '<S891>/Constant1'
                                             * TJATVG CrvAmplActivated
                                               DT:uint8
                                             */
uint8_T TJATVG_LimiterActivated_nu;         /* '<S891>/Constant5'
                                             * TJATVG limiter Activated
                                               DT:uint8
                                             */
uint8_T TJATVG_LimiterType_nu;              /* '<S891>/Constant6'
                                             * TJATVG limiter type
                                               DT:uint8
                                             */
boolean_T TJATVG_TriggerReplan_nu;          /* '<S891>/Constant11'
                                             * TJATVG trigger replan flag
                                               DT:boolean
                                             */
boolean_T TJATVG_HighStatAccu_bool;         /* '<S893>/Constant1'
                                             * TJATVG  high stationary accuracy flag
                                               DT:boolean
                                             */
boolean_T TJALKA_LanePredictValid_bool;     /* '<S165>/Switch'
                                             * TJALKA lane predict valid flag
                                               DT:boolean
                                             */
boolean_T TJALKA_Cancel_bool;               /* '<S101>/AND1'
                                             * TJALKA cancel ready flag
                                               DT:boolean
                                             */
boolean_T TJAOBF_ObjLaneValidDuration_bool; /* '<S222>/AND'
                                             * TJAOBF object in lane valid
                                             duration flag DT:boolean
                                             */
boolean_T TJALKA_StrongReady_bool;          /* '<S101>/AND'
                                             * TJALKA strong ready flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_LKAOnlySwitch_bool;        /* '<S48>/AND3'
                                             * TJAGEN LKA only switch flag
                                               DT:boolean
                                             */
boolean_T TJAOBF_Cancel_bool;               /* '<S182>/AND3'
                                             * TJAOBF cancel flag
                                               DT:boolean
                                             */
boolean_T TJALKA_WeakReady_bool;            /* '<S101>/AND2'
                                             * TJALKA weak ready flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_Clearance_bool;            /* '<S47>/AND'
                                             * TJAGEN clearance flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_Degradation_bool;          /* '<S50>/AND'
                                             * TJAGEN degradation flag
                                               DT:boolean
                                             */
boolean_T TJACMB_ObjectCorridor_bool;       /* '<S18>/AND'
                                             * TJACMB object corridor flag
                                               DT:boolean
                                             */
boolean_T TJAOBF_StrongReady_bool;          /* '<S182>/AND1'
                                             * TJAOBF strong ready flag
                                               DT:boolean
                                             */
boolean_T TJAOBF_WeakReady_bool;            /* '<S182>/AND2'
                                             * TJAOBF weak ready flag
                                               DT:boolean
                                             */
boolean_T TJASLC_Nb_DCLCSwitchNVRAM_nu;     /* '<S267>/Switch2'
                                             * TJATVG obstacle DisX
                                               DT:float32
                                             */
boolean_T TJASLC_StrongReady_bool;          /* '<S315>/Switch'
                                             * SLC strong ready flag
                                               DT:boolean
                                             */
boolean_T TJASLC_WeakReady_bool;            /* '<S315>/Switch2'
                                             * SLC weak ready flag
                                               DT:boolean
                                             */
boolean_T TJASLC_Cancel_bool;               /* '<S315>/Signal Conversion'
                                             * SLC cancel flag
                                               DT:boolean
                                             */
boolean_T TJACMB_StrongReady_bool;          /* '<S15>/AND1'
                                             * TJACMB Strong ready flag
                                               DT:boolean
                                             */
boolean_T TJACMB_WeakReady_bool;            /* '<S15>/AND2'
                                             * TJACMB weak ready flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_FunctionSwitch_bool;       /* '<S48>/OR1'
                                             * TJAGEN function switch flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_CodeFunction_bool;         /* '<S48>/OR3'
                                             * TJAGEN code function flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_Error_bool;                /* '<S51>/Switch'
                                             * TJAGEN error flag
                                               DT:boolean
                                             */
boolean_T SLC_LCPLeft2Active_bool;          /* '<S370>/AND'
                                             * LCPLeft2Active Flag
                                             */
boolean_T SLC_LCPRight2Active_bool;         /* '<S373>/AND'
                                             * LCPRight2Active Flag
                                             */
boolean_T SLC_LCPLeft2Passive_bool;         /* '<S371>/AND'
                                             * LCPLeft2Passive Flag
                                             */
boolean_T SLC_LCPRight2Passive_bool;        /* '<S372>/AND'
                                             * LCPRight2Passive Flag
                                             */
boolean_T TJAGEN_Abort_bool;                /* '<S49>/NotEqual1'
                                             * TJAGEN abort flag
                                               DT:boolean
                                             */
boolean_T GEN_AllStateAvailable_bool;       /* '<S89>/Equal2' */
boolean_T TJAGEN_StrongReady_bool;          /* '<S53>/AND'
                                             * TJAGEN strong ready flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_Cancel_nu;                 /* '<S46>/OR'
                                             * TJAGEN cancel flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_WeakReady_bool;            /* '<S54>/AND'
                                             * TJAGEN weak ready flag
                                               DT:boolean
                                             */
boolean_T TJAGEN_FunctionQuit_bool;         /* '<S52>/OR'
                                             * Function Quit Flag
                                             */
boolean_T TJAGEN_SuspendStart_bool;         /* '<S52>/OR4'
                                             * Suspended start Flag
                                             */
boolean_T TJAGEN_SuspendEnd_bool;           /* '<S52>/OR5'
                                             * Suspended end Flag
                                             */
boolean_T STM_PrevRAMPOUT_bool;             /* '<S552>/Unit Delay'
                                             * Previous rampout Flag
                                             */
boolean_T TJATTG_PredictionEnable_bool;     /* '<S743>/Switch'
                                             * TJATTG prediction enable flag
                                               DT:boolean
                                             */
boolean_T TJATTG_TransTriggerReplan_bool;   /* '<S670>/OR2'
                                             * TJATTG transition trigger replan
                                             flag   DT:boolean
                                             */
boolean_T TJASLC_TakeOverValid_bool;        /* '<S378>/OR4'
                                             * SLC take over valid flag
                                               DT:boolean
                                             */
boolean_T TJATVG_LtcyCompActivated_nu;      /* '<S924>/Switch'
                                             * TJATVG  latency compensation flag
                                               DT:boolean
                                             */
boolean_T TJALKA_SRLaneRelateCheck_bool;    /* '<S101>/AND3'
                                             * TJALKA strong ready line related
                                             check    DT:boolean
                                             */
boolean_T TJASTM_DrvTakeOver_bool;          /* '<S553>/AND'
                                             * TJASTM driver takeover Flag
                                             */
boolean_T STM_SuspendTimeExpired_bool;      /* '<S603>/Switch'
                                             * STE Flag
                                             */
boolean_T TJAOBF_TgtObjDataValid_bool;      /* '<S5>/Signal Conversion'
                                             * TJAOBF target date valid flag
                                               DT:boolean
                                             */
boolean_T TJACMB_Cancel_bool;               /* '<S15>/Constant2'
                                             * TJACMB cancel flag
                                               DT:boolean
                                             */
E_TJATVG_TrajGuiQu_nu TJATVG_TrajGuiQu_nu;  /* '<S888>/Switch'
                                             * TJATVG trajectory qualifier
                                               DT:Enum: E_TJATVG_TrajGuiQu_nu
                                             */
E_TJASTM_SysStateTJA_nu
    TJASTM_SysStateTJAIn_nu; /* '<S8>/Signal Conversion'
                              * TJASTM system state
                                DT:Enum: E_TJASTM_SysStateTJA_nu
                              */
E_TJASTM_SysStateTJA_nu
    TJASTM_SysStateTJA_nu;                     /* '<S552>/Switch3'
                                                * TJASTM system state
                                                  DT:Enum: E_TJASTM_SysStateTJA_nu
                                                */
E_TJASTM_SysStateHWA_nu TJASTM_SysStateHWA_nu; /* '<S551>/Switch2' */
E_TJASTM_LatCtrlMode_nu
    TJASTM_LatCtrlMode_nu; /* '<S8>/Signal Conversion1'
                            * TJASTM lateral control mode
                              DT:Enum: E_TJASTM_LatCtrlMode_nu
                            */
E_TJASLC_ReadyToTrigger_nu
    TJASLC_ReadyToTrigger_nu; /* '<S479>/Switch'
                               * SLC ready to trigger state
                                 DT:Enum: E_TJASLC_ReadyToTrigger_nu
                               */
E_TJASLC_ManeuverState_nu
    TJASLC_ManeuverState_nu; /* '<S320>/Switch'
                              * SLC maneuver state
                                DT:Enum: E_TJASLC_ManeuverState_nu
                              */
E_TJASLC_LaneChangeTrig_nu
    TJASLC_LCDirection_enum; /* '<S264>/Signal Conversion'
                              * SLC cancel flag
                                DT:boolean
                              */
E_TJASLC_LaneChangeTrig_nu
    TJASLC_LaneChangeTrig_nu;                /* '<S516>/Switch2'
                                              * SLC lane change trigger state
                                                DT:Enum: E_TJASLC_LaneChangeTrig_nu
                                              */
E_TJALKA_LnBndValid_nu TJALKA_LnBndValid_nu; /* '<S151>/Switch'
                                              * TJALKA lane boundary valid state
                                                DT:Enum: E_TJALKA_LnBndValid_nu
                                              */

/* Block signals (default storage) */
B_TJASA_T TJASA_B;

/* Block states (default storage) */
DW_TJASA_T TJASA_DW;

/* Exported data definition */

/* Definition for custom storage class: Global */
uint8_T TJASA_SetBit_BS_Param_1[16] = {
    0U, 1U,  2U,  3U,  4U,  5U,  6U, 7U, 8U,
    9U, 10U, 11U, 12U, 13U, 14U, 15U}; /* Referenced by:
                                        * '<S572>/ex_sfun_set_bit'
                                        * '<S40>/ex_sfun_set_bit'
                                        * '<S78>/ex_sfun_set_bit'
                                        * '<S136>/ex_sfun_set_bit'
                                        * '<S200>/ex_sfun_set_bit'
                                        * '<S218>/ex_sfun_set_bit'
                                        * '<S468>/ex_sfun_set_bit'
                                        * '<S469>/ex_sfun_set_bit'
                                        * '<S525>/ex_sfun_set_bit'
                                        * '<S681>/ex_sfun_set_bit'
                                        */

uint8_T TJASA_SetBit_BS_Param_2[8] = {0U, 1U, 2U, 3U, 4U,
                                      5U, 6U, 7U}; /* Referenced by:
                                                    * '<S56>/ex_sfun_set_bit'
                                                    * '<S96>/ex_sfun_set_bit'
                                                    * '<S100>/ex_sfun_set_bit'
                                                    * '<S181>/ex_sfun_set_bit'
                                                    * '<S255>/ex_sfun_set_bit'
                                                    * '<S294>/ex_sfun_set_bit'
                                                    */

uint8_T TJASA_SetBit_BS_Param_3[5] = {0U, 1U, 2U, 3U, 4U};
/* Referenced by: '<S133>/ex_sfun_set_bit' */

/* Definition for custom storage class: Global */
boolean_T CMB_EnableFusion2EdgeRising_bool; /* '<S30>/Unit Delay' */

/* Enable fusion 2 edge rising--Used in TJACMB module
   DT:boolean */
boolean_T CMB_EnableFusionEdgeFalling_bool; /* '<S32>/Unit Delay' */

/* Enable fusion edge falling--Used in TJACMB module
   DT:boolean */
boolean_T CMB_EnableFusionEdgeRising_bool; /* '<S36>/Unit Delay' */

/* Enable fusion edge rising--Used in TJACMB module
   DT:boolean */
real32_T CMB_EnableFusionStopwatch_sec; /* '<S34>/Unit Delay' */

/* Enable fusion stopwatch time--Used in TJACMB module
   DT:float32 */
real32_T CMB_EnableFusionTurnOffDelay_sec; /* '<S35>/Unit Delay' */

/* Enable fusion turn off delay time--Used in TJACMB module
   DT:float32 */
boolean_T CMB_LaneQualityInvalid_bool; /* '<S16>/Switch' */

/* Debug */
boolean_T CMB_LnQualRSFF_bool[2]; /* '<S43>/Unit Delay' */

/* Lane quality RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
real32_T CMB_LnQualTurnOffDelay_sec[2]; /* '<S44>/Unit Delay' */

/* Lane quality turn off delay time--Used in TJACMB module
   DT:float32 */
real32_T CMB_LnQualTurnOnDelay_sec[2]; /* '<S45>/Unit Delay' */

/* Lane quality turn on delay time--Used in TJACMB module
   DT:float32 */
boolean_T CMB_ObjectFollowingOnly_bool; /* '<S17>/AND' */

/* Debug */
real32_T CMB_PrevCntrCrv1UnitDelay_1pm; /* '<S12>/Unit Delay' */

/* Previous control lane clothoid curve(Unit delay 1)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv2UnitDelay_1pm; /* '<S12>/Unit Delay1' */

/* Previous control lane clothoid curve(Unit delay 2)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv3UnitDelay_1pm; /* '<S12>/Unit Delay2' */

/* Previous control lane clothoid curve(Unit delay 3)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv4UnitDelay_1pm; /* '<S12>/Unit Delay3' */

/* Previous control lane clothoid curve(Unit delay 4)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv5UnitDelay_1pm; /* '<S12>/Unit Delay4' */

/* Previous control lane clothoid curve(Unit delay 5)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv6UnitDelay_1pm; /* '<S12>/Unit Delay5' */

/* Previous control lane clothoid curve(Unit delay 6)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv7UnitDelay_1pm; /* '<S12>/Unit Delay6' */

/* Previous control lane clothoid curve(Unit delay 7)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv8UnitDelay_1pm; /* '<S12>/Unit Delay7' */

/* Previous control lane clothoid curve(Unit delay 8)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCntrCrv9UnitDelay_1pm; /* '<S12>/Unit Delay8' */

/* Previous control lane clothoid curve(Unit delay 9)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevCombCrvLFUnitDelay_1pm; /* '<S25>/Unit Delay' */
real32_T CMB_PrevTgtCrv1UnitDelay_1pm;   /* '<S13>/Unit Delay' */

/* Previous target clothoid curve(Unit delay 1)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv2UnitDelay_1pm; /* '<S13>/Unit Delay1' */

/* Previous target clothoid curve(Unit delay 2)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv3UnitDelay_1pm; /* '<S13>/Unit Delay2' */

/* Previous target clothoid curve(Unit delay 3)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv4UnitDelay_1pm; /* '<S13>/Unit Delay3' */

/* Previous target clothoid curve(Unit delay 4)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv5UnitDelay_1pm; /* '<S13>/Unit Delay4' */

/* Previous target clothoid curve(Unit delay 5)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv6UnitDelay_1pm; /* '<S13>/Unit Delay5' */

/* Previous target clothoid curve(Unit delay 6)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv7UnitDelay_1pm; /* '<S13>/Unit Delay6' */

/* Previous target clothoid curve(Unit delay 7)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv8UnitDelay_1pm; /* '<S13>/Unit Delay7' */

/* Previous target clothoid curve(Unit delay 8)--Used in TJACMB module
   DT:float32 */
real32_T CMB_PrevTgtCrv9UnitDelay_1pm; /* '<S13>/Unit Delay8' */

/* Previous target clothoid curve(Unit delay 9)--Used in TJACMB module
   DT:float32 */
boolean_T CMB_VelXMaxHyst_bool; /* '<S41>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJACMB module
   DT:boolean */
boolean_T CMB_VelXMinHyst_bool; /* '<S42>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJACMB module
   DT:boolean */
real32_T CMB_WeightCrv2LowPass_1pm; /* '<S31>/Unit Delay' */

/* Weighted curve 2 low pass filter--Used in TJACMB module
   DT:float32 */
real32_T CMB_WeightCrvLowPass_1pm; /* '<S37>/Unit Delay' */

/* Weighted curve low pass filter--Used in TJACMB module
   DT:float32 */
boolean_T GEN_AclXMaxHyst_bool; /* '<S94>/Unit Delay' */

/* Acceleration X Max hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_AclXMinHyst_bool; /* '<S95>/Unit Delay' */

/* Acceleration X Min hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_AclYMaxHyst_bool; /* '<S93>/Unit Delay' */

/* Acceleration Y Max hystereis flag--Used in TJAGEN module
   DT:boolean */
real32_T GEN_BlockTimeExpiredTimerRetrigger_sec; /* '<S99>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
real32_T GEN_HODTurnOnDelay_sec; /* '<S73>/Unit Delay' */

/* HOD turn on delay time--Used in TJAGEN module
   DT:float32 */
real32_T GEN_HazardTurnOnDelay_sec; /* '<S79>/Unit Delay' */

/* turn signal hazard turn on delay time--Used in TJAGEN module
   DT:float32 */
real32_T GEN_ManualTorMaxTurnOnDelay_sec; /* '<S76>/Unit Delay' */

/* Manual torque Max turn on delay time--Used in TJAGEN module
   DT:float32 */
real32_T GEN_ManualTorMinTurnOnDelay_sec; /* '<S77>/Unit Delay' */

/* Manual torque Min turn on delay time--Used in TJAGEN module
   DT:float32 */
boolean_T GEN_ManualTorqueMaxHyst_bool; /* '<S74>/Unit Delay' */

/* Manual torque max hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_ManualTorqueMinHyst_bool; /* '<S75>/Unit Delay' */

/* Manual torque max hystereis flag--Used in TJAGEN module
   DT:boolean */
E_TJASTM_LatCtrlMode_nu GEN_PrevLatCtrlMode_Enum; /* '<S8>/Unit Delay' */

/* Previous lateal control mode--Used in TJAGEN module
   Enum: E_TJASTM_LatCtrlMode_nu */
boolean_T GEN_PrevRampoutNUnitDelay_bool; /* '<S552>/Unit Delay' */

/* Previous rampout flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_PrevRampoutUnitDelay_bool; /* '<S59>/Unit Delay' */

/* Previous rampout flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_PrevSus2UnitDelay_bool; /* '<S600>/Unit Delay' */

/* Previous suspended flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_PrevSusQuitUnitDelay_bool; /* '<S552>/Unit Delay1' */

/* Previous suspended quit flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_PrevSusUnitDelay_bool; /* '<S599>/Unit Delay' */

/* Previous suspended flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_RampoutTimeExpiredRSFF_bool; /* '<S60>/Unit Delay' */

/* Rampout Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
real32_T GEN_RampoutTimeExpiredTimerRetrigger_sec; /* '<S61>/Unit Delay' */

/* Rampout Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
real32_T GEN_SafeFuncActiveTurnOnDelay_sec; /* '<S85>/Unit Delay' */

/* Safety function active turn on delay time--Used in TJAGEN module
   DT:float32 */
real32_T GEN_SafeFuncErrorTurnOnDelay_sec; /* '<S86>/Unit Delay' */

/* Safety function error turn on delay time--Used in TJAGEN module
   DT:float32 */
boolean_T GEN_SteerWAngleGradHyst_bool; /* '<S81>/Unit Delay' */

/* Steering angle grad hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_SteerWAngleGradSusHyst_bool; /* '<S80>/Unit Delay' */

/* Steering angle grad for suspended hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_SteerWAngleHyst_bool; /* '<S82>/Unit Delay' */

/* Steering angle hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_SteerWAngleSusHyst_bool; /* '<S83>/Unit Delay' */

/* Steering angle for suspended hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_Sus2TimeExpiredRSFF_bool; /* '<S603>/Unit Delay' */

/* Susopended Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
real32_T GEN_Sus2TimeExpiredTimerRetrigger_sec; /* '<S604>/Unit Delay' */

/* Suspended Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
boolean_T GEN_SusTimeExpiredRSFF_bool; /* '<S601>/Unit Delay' */

/* Susopended Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
real32_T GEN_SusTimeExpiredTimerRetrigger_sec; /* '<S602>/Unit Delay' */

/* Suspended Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
boolean_T GEN_VehCrvDIHyst_bool; /* '<S84>/Unit Delay' */

/* vehicle curve hystereis flag--Used in TJAGEN module
   DT:boolean */
boolean_T GEN_VehYawRateDIHyst_bool; /* '<S87>/Unit Delay' */

/* vehicle yaw rate hystereis flag--Used in TJAGEN module
   DT:boolean */
real32_T LKA_BlockTimerExpiredTimerRetrigger_sec; /* '<S121>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJALKA module
   DT:float32 */
boolean_T LKA_CrvQualRSFF_bool[2]; /* '<S173>/Unit Delay' */

/* Curve quality RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
real32_T LKA_CrvQualTurnOffDelay_sec[2]; /* '<S174>/Unit Delay' */

/* Curve quality turn off delay time--Used in TJALKA module
   DT:float32 */
real32_T LKA_CrvQualTurnOnDelay_sec[2]; /* '<S175>/Unit Delay' */

/* Curve quality turn on delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_DistVeh2LnBndHyst_bool[2]; /* '<S142>/Unit Delay' */

/* Distance of Veh to Lane Boundary hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_LanePredictValidRSFF_bool; /* '<S165>/Unit Delay' */

/* Lane predict valid RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_LaneWidthMaxHyst_bool; /* '<S134>/Unit Delay' */

/* Lane width Max hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_LaneWidthMinHyst_bool; /* '<S135>/Unit Delay' */

/* Lane width Min hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_LeLnCrvQualityValid_bool; /* '<S152>/OR' */

/* Debug */
real32_T LKA_LeLnIncohTurnOffDelay_sec; /* '<S131>/Unit Delay' */

/* Left lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_LeLnQualityValid_bool; /* '<S154>/OR' */

/* Debug */
real32_T LKA_LnBndValidTurnOnDelay_sec; /* '<S166>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_LnIncohEdgeRising_bool; /* '<S125>/Unit Delay' */

/* Lane incoherence edge rising--Used in TJALKA module
   DT:boolean */
real32_T LKA_LnIncohTurnOffDelay_sec; /* '<S130>/Unit Delay' */

/* Lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_LnQualRSFF_bool[2]; /* '<S178>/Unit Delay' */

/* Lane quality RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
real32_T LKA_LnQualTurnOffDelay_sec[2]; /* '<S179>/Unit Delay' */

/* Lane quality turn off delay time--Used in TJALKA module
   DT:float32 */
real32_T LKA_LnQualTurnOnDelay_sec[2]; /* '<S180>/Unit Delay' */

/* Lane quality turn on delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_LnQualifierEdgeRising_bool; /* '<S176>/Unit Delay' */

/* Lane Qualifier edge rising--Used in TJALKA module
   DT:boolean */
real32_T LKA_LnQualifierTurnOffDelay_sec; /* '<S177>/Unit Delay' */

/* Lane Qualifier turn off delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_OBFValidEdgeRising_bool; /* '<S158>/Unit Delay' */

/* OBF valid edge rising--Used in TJALKA module
   DT:boolean */
boolean_T LKA_PredTimeExceededEdgeFalling_bool; /* '<S167>/Unit Delay' */

/* Prediction time exceeded edge falling--Used in TJALKA module
   DT:boolean */
real32_T LKA_PredTimeExceededTurnOffDelay_sec; /* '<S168>/Unit Delay' */

/* Prediction time exceeded turn off delay time--Used in TJALKA module
   DT:float32 */
real32_T LKA_PrevLeLnPosY0UnitDelay_met;        /* '<S111>/Unit Delay' */
E_TJALKA_LnBndValid_nu LKA_PrevLnBndValid_Enum; /* '<S150>/Unit Delay' */

/* Previous lane boundary valid flag--Used in TJALKA module
   DT:Enum: E_TJALKA_LnBndValid_nu */
real32_T LKA_PrevRiLnPosY0UnitDelay_met; /* '<S111>/Unit Delay1' */
real32_T LKA_PrevVehDistUnitDelay_met;   /* '<S126>/Unit Delay2' */

/* Previous vehicle distance--Used in TJALKA module
   DT:boolean */
boolean_T LKA_RadiusHyst_bool; /* '<S137>/Unit Delay' */

/* Radius hystereis flag--Used in TJALKA module
   DT:boolean */
real32_T LKA_RiLnIncohTurnOffDelay_sec; /* '<S132>/Unit Delay' */

/* Right lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
boolean_T LKA_TrajPlanCancelRSFF_bool; /* '<S124>/Unit Delay' */

/* Trajectory plan cancel RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_TurnSignalLevelHoldEdgeFalling_bool; /* '<S144>/Unit Delay' */

/* Turn signal level hold edge falling--Used in TJALKA module
   DT:boolean */
boolean_T LKA_ValidLengthHyst_bool[2]; /* '<S138>/Unit Delay' */

/* Valid length hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_VelXMaxHystWR_bool; /* '<S148>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_VelXMaxHyst_bool; /* '<S146>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_VelXMinHystWR_bool; /* '<S149>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T LKA_VelXMinHyst_bool; /* '<S147>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJALKA module
   DT:boolean */
boolean_T OBF_AccObjSwitch; /* '<S206>/NOT' */

/* Debug */
boolean_T OBF_AccObjSwitchEdgeRising_bool; /* '<S213>/Unit Delay' */

/* Acc object switch edge rising--Used in TJAOBF module
   DT:boolean */
real32_T OBF_AccObjSwitchTurnOffDelay_sec; /* '<S214>/Unit Delay' */

/* Acc object switch turn off delay time--Used in TJAOBF module
   DT:float32 */
boolean_T OBF_AccObjValid; /* '<S206>/Equal' */

/* Debug */
boolean_T OBF_AccObjValidHoldUnitDelay_bool; /* '<S220>/Unit Delay' */

/* Previous AccObjValidHold flag--Used in TJASLC module
   DT:boolean */
boolean_T OBF_AccObjValidLaneCheck; /* '<S206>/Equal1' */

/* Debug */
real32_T OBF_AccObjValidLaneTurnOnDelay_sec; /* '<S216>/Unit Delay' */

/* Acc object lane valid turn on delay time--Used in TJAOBF module
   DT:float32 */
real32_T OBF_AccObjValidTurnOnDelay_sec; /* '<S215>/Unit Delay' */

/* Acc object valid turn on delay time--Used in TJAOBF module
   DT:float32 */
boolean_T OBF_AccObjValid_bool; /* '<S215>/AND' */

/* Debug */
real32_T OBF_BitfieldValidTurnOnDelay_sec; /* '<S256>/Unit Delay' */

/* Bitfield valid turn on delay time--Used in TJAOBF module
   DT:float32 */
real32_T OBF_BlockTimeExpiredTimerRetrigger_sec; /* '<S195>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJAOBF module
   DT:float32 */
boolean_T OBF_CancelUnitDelay_bool; /* '<S1>/Unit Delay5' */

/* Cancel Unit Delay--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_Crv_SRHyst_bool; /* '<S257>/Unit Delay' */

/* Crv_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_DistOrEgoLaneInvalid_bool; /* '<S225>/OR5' */

/* Debug */
boolean_T OBF_Heading_SRHyst_bool; /* '<S258>/Unit Delay' */

/* Heading_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_LaneAttributesValidHyst_bool[2]; /* '<S246>/Unit Delay' */

/* Lane attributes valid hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_LaneCheckValid_bool; /* '<S225>/OR2' */

/* Debug */
boolean_T OBF_LeftLaneCheckValid_bool; /* '<S224>/AND' */

/* Debug */
real32_T OBF_LeftLineValidTurnOnDelay_sec; /* '<S247>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
boolean_T OBF_MaxDist2BndHyst_bool[2]; /* '<S239>/Unit Delay' */

/* Max distance to boundary hystereis flag--Used in TJAOBF module
   DT:boolean */
real32_T OBF_MaxDurObjBrdgTimerRe_sec; /* '<S198>/Unit Delay' */

/* Max duration object bridging--Used in TJAOBF module
   DT:float32 */
boolean_T OBF_MinDist2BndHyst_bool[2]; /* '<S238>/Unit Delay' */

/* Min distance to boundary hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_MinDist2LeftBndInvalid; /* '<S228>/NOT' */

/* Debug */
boolean_T OBF_MinDist2RightBndInvalid; /* '<S228>/NOT1' */

/* Debug */
real32_T OBF_MinDurLCforOBTurnOnDelay_sec; /* '<S199>/Unit Delay' */

/* Min duration LC for OBF turn on delay time--Used in TJAOBF module
   DT:float32 */
real32_T OBF_ObjLaneValidDurationTurnOnDelay_sec; /* '<S222>/Unit Delay' */

/* Obj in lane valid duration turn on delay time--Used in TJAOBF module
   DT:float32 */
boolean_T OBF_ObjLaneValidHoldUnitDelay_bool; /* '<S220>/Unit Delay1' */

/* Previous ObjLaneValidHold flag--Used in TJASLC module
   DT:boolean */
boolean_T OBF_PosY0_SRHyst_bool; /* '<S259>/Unit Delay' */

/* PosY0_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_RightLaneCheckValid_bool; /* '<S224>/AND1' */

/* Debug */
real32_T OBF_RightLineValidTurnOnDelay_sec; /* '<S248>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
boolean_T OBF_StrongReadyUnitDelay_bool; /* '<S1>/Unit Delay3' */

/* Strong ready Unit Delay--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_TargetObjDataSR_bool; /* '<S184>/AND1' */

/* Debug */
boolean_T OBF_TargetObjDataWR_bool; /* '<S184>/AND' */

/* Debug */
boolean_T OBF_TargetOutsideEgoLane_bool; /* '<S229>/Multiport Switch' */

/* Debug */
boolean_T OBF_TurnSignalHoldUnitDelay_bool; /* '<S194>/Unit Delay' */

/* Previous turn signal level hold flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_VelXMaxHyst_bool; /* '<S201>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_VelXMinHyst_bool; /* '<S202>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJAOBF module
   DT:boolean */
boolean_T OBF_WeakReadyUnitDelay_bool; /* '<S1>/Unit Delay4' */

/* Weak ready Unit Delay--Used in TJAOBF module
   DT:boolean */
boolean_T OF_NoObjectCollision_bool; /* '<S193>/NOT' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T OF_ObjectDangerLeftRear_bool; /* '<S193>/AND' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T OF_ObjectDangerRightRear_bool; /* '<S193>/AND1' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T OF_RearObjDecCrvtTurnOffDelay_sec; /* '<S204>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJAOF module
   DT:float32 */
real32_T OF_RearObjDecTurnOffDelay_sec[2]; /* '<S205>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJAOF module
   DT:float32 */
real32_T SLC_AbortNewEgoTime_sec; /* '<S342>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
E_TJASLC_AbortState_nu SLC_AbortState_enum; /* '<S314>/Signal Conversion' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
real32_T SLC_AbortTime_sec; /* '<S341>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
real32_T SLC_AbortYHdSignDelay2_sec; /* '<S343>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_Abort_bool; /* '<S261>/OR1' */

/* Debug */
boolean_T SLC_AdjLnWidthMaxHyst_bool[2]; /* '<S271>/Unit Delay' */

/* Adjacent  lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_AdjLnWidthMinHyst_bool[2]; /* '<S272>/Unit Delay' */

/* Adjacent  lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_AllowGoBack_bool; /* '<S299>/GreaterThan' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
boolean_T SLC_Cancel_bool; /* '<S261>/OR' */

/* Debug */
real32_T SLC_CenterDistToBoundary_met; /* '<S299>/Switch2' */

/* Rear obejct detected check turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_EgoVehVelMaxHyst_bool; /* '<S504>/Unit Delay' */

/* Ego vehicle velocity max hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_EgoVehVelMinHyst_bool; /* '<S505>/Unit Delay' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_ExitAbortNewEgo_bool; /* '<S342>/AND' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
boolean_T SLC_ExitAbort_bool; /* '<S327>/OR1' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
real32_T SLC_FrontSafeTrOnTime_sec; /* '<S512>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_FrontUnSafeTrOnTime_sec; /* '<S509>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_GRCOLeftRSFF_bool; /* '<S360>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_GRCORightRSFF_bool; /* '<S361>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_GoBkLfTurnLtTurnOffDelay_sec; /* '<S301>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_GoBkRiTurnLtTurnOffDelay_sec; /* '<S302>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_IntoAbort_nu; /* '<S324>/OR1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_LCActiveTurnOnDelay_sec; /* '<S503>/Unit Delay' */

/* Lane centering active turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_LCM_Cancel_bool; /* '<S321>/OR' */

/* Debug */
boolean_T SLC_LCM_End_bool; /* '<S366>/NOT' */

/* Debug */
boolean_T SLC_LCM_Start_bool; /* '<S367>/NOT' */

/* Debug */
real32_T SLC_LCPLeft2ActiveTurnOnDelay_sec; /* '<S370>/Unit Delay' */

/* LCPLeft2Active turn on delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LCPLeft2PassiveTurnOnDelay_sec; /* '<S371>/Unit Delay' */

/* LCPLeft2Passive turn on delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LCPRight2ActiveTurnOnDelay_sec; /* '<S373>/Unit Delay' */

/* LCPRight2Active turn on delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LCPRight2PassiveTurnOnDelay_sec; /* '<S372>/Unit Delay' */

/* LCPRight2Passive turn on delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LCWPassiveTurnOnDly_sec; /* '<S432>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LaneChangCancleTimeDelay_sec; /* '<S394>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LaneChangEndTimeDelay_sec; /* '<S397>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_LaneChangeBackDetc_bool; /* '<S326>/Switch2' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeCancleInfo; /* '<S394>/OR' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
E_TJASLC_LaneChangeTrig_nu
    SLC_LaneChangeDirectionAbort_enum; /* '<S314>/Signal Conversion1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
E_TJASLC_LaneChangeTrig_nu SLC_LaneChangeDirectionIn_nu; /* '<S325>/Switch' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeEdgeRising_bool; /* '<S470>/Unit Delay' */

/* Lane change edge rising--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeEndInfo; /* '<S397>/OR' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeOnGoingInfo; /* '<S381>/OR1' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangePendingInfo; /* '<S382>/OR4' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeRSFF_bool; /* '<S478>/Unit Delay' */

/* Lane change RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneChangeWarnEdgeRising_bool; /* '<S409>/Unit Delay' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
real32_T SLC_LaneChangeWarnTurnOffDelay_sec; /* '<S429>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
E_TJASLC_LaneChangeWarning_nu
    SLC_LaneChangeWarnUnitDy_Enum; /* '<S375>/Unit Delay' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
boolean_T SLC_LaneCheckValid_bool; /* '<S364>/AND' */

/* Debug */
real32_T SLC_LaneChngDetectedTurnOffDelay_sec; /* '<S368>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_LaneWidthMaxHyst_bool; /* '<S312>/Unit Delay' */

/* SLC lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LaneWidthMinHyst_bool; /* '<S313>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_LeLaneTypeTurnOffDelay_sec; /* '<S309>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LeftSafeTrOnTime_sec; /* '<S510>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LeftTurnSignalOffDelay_sec; /* '<S544>/Unit Delay' */

/* Left turn signal turn off dealy time in sec in TJASLC module
   DT:single */
real32_T SLC_LeftUnSafeTrOnTime_sec; /* '<S507>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LeverEngagedTurnOnDelay2_sec[2]; /* '<S541>/Unit Delay' */

/* Lever left or right engaged turn on delay time 2--Used in TJASLC module
   DT:float32 */
real32_T SLC_LeverEngagedTurnOnDelay_sec; /* '<S535>/Unit Delay' */

/* Lever left or right engaged turn on delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_LeverLeftEngagedRSFF_bool; /* '<S520>/Unit Delay' */

/* Lever left engaged RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LeverLeftEngaged_bool; /* '<S520>/Switch' */

/* Debug */
boolean_T SLC_LeverRightEngagedRSFF_bool; /* '<S522>/Unit Delay' */

/* Lever right engaged RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_LeverRightEngaged_bool; /* '<S522>/Switch' */

/* Debug */
real32_T SLC_LnChngBlockTimeCancleTfDelay_sec; /* '<S493>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_LnChngBlockTimeTurnOffDelay_sec; /* '<S492>/Unit Delay' */

/* Lane change block time expired turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_LnChngDetectedRSFF_bool; /* '<S495>/Unit Delay' */

/* Lane change detected RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_LnChngDetectedTurnOffDelay_sec; /* '<S496>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_ManeuverStateTurnOnDelay_sec; /* '<S448>/Unit Delay' */

/* Maneuver state turn on delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_ManvStatePassive_bool; /* '<S526>/AND' */

/* Debug */
boolean_T SLC_MaxInitDurationExceeded_bool; /* '<S527>/AND' */

/* Debug */
boolean_T SLC_MaxManeuTimeRSFF_bool; /* '<S289>/Unit Delay' */

/* max maneuver time expired RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_MaxManeuTimeRetrigger_sec; /* '<S290>/Unit Delay' */

/* Max maneuver time expired Time retrigger--Used in TJASLC module
   DT:float32 */
boolean_T SLC_NewEgoLaneRSFF_bool; /* '<S369>/Unit Delay' */

/* New ego lane RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_NewEgoLane_bool; /* '<S369>/Switch' */

/* Debug */
boolean_T SLC_OELCNewEgoLaneRSFF_bool; /* '<S303>/Unit Delay' */

/* New ego lane RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
E_TJASLC_AbortState_nu SLC_PreAbortState_enum; /* '<S314>/Unit Delay1' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
E_TJASLC_LaneChangeTrig_nu
    SLC_PreLaneChangeDirtAbort_enum; /* '<S314>/Unit Delay' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrebNewEgoOverTime_nu; /* '<S379>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrevDriverTrigResetLeftUnitDelay_bool; /* '<S268>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrevDriverTrigResetRightUnitDelay_bool; /* '<S268>/Unit Delay2' */

/* Previous driver trigger reset Right flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_PrevFrontWheelDist2BoundUnitDelay_met; /* '<S362>/Unit Delay' */

/* Previous front whell distance to boundary--Used in TJASLC module
   DT:float32 */
real32_T SLC_PrevFtWhDit2BdSignUnitDelay_met; /* '<S367>/Unit Delay' */

/* Previous front wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
E_TJASLC_LaneChangeTrig_nu
    SLC_PrevLaneChangeTrigger_nu; /* '<S516>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
E_TJASTM_LatCtrlMode_nu SLC_PrevLatCtrlMd2_Enum; /* '<S287>/Unit Delay' */

/* Previous lateral control mode 2--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
E_TJASTM_LatCtrlMode_nu SLC_PrevLatCtrlMd3_Enum; /* '<S285>/Unit Delay' */

/* Previous lateral control mode 3--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
E_TJASTM_LatCtrlMode_nu SLC_PrevLatCtrlMdSLC2LCC_Enum; /* '<S888>/Unit Delay' */

/* Previous lateral control mode--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
E_TJASTM_LatCtrlMode_nu SLC_PrevLatCtrlMd_Enum; /* '<S491>/Unit Delay' */

/* Previous lateral control mode--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
E_TJASLC_ManeuverState_nu SLC_PrevManeuverState2_Enum; /* '<S526>/Unit Delay' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
E_TJASLC_ManeuverState_nu SLC_PrevManeuverState3_Enum; /* '<S320>/Unit Delay' */

/* Previous maneuver state 3--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverStateGRCO_Enum; /* '<S264>/Unit Delay1' */

/* Previous maneuver state--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverStateSLC2LCC_Enum; /* '<S888>/Unit Delay1' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
E_TJASLC_ManeuverState_nu SLC_PrevManeuverState_Enum; /* '<S7>/Unit Delay1' */

/* Previous maneuver state--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
real32_T SLC_PrevReWhDit2BdSignUnitDelay_met; /* '<S366>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
E_TJASLC_LaneChangeWarning_nu
    SLC_PrevReaAbortWarnSide_enum; /* '<S375>/Unit Delay3' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
E_TJASLC_LaneChangeWarning_nu
    SLC_PrevRearCancleWarnSide_enum; /* '<S375>/Unit Delay2' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_PrevRearWheelDist2BoundUnitDelay_met; /* '<S363>/Unit Delay' */

/* Previous rear whell distance to boundary flag--Used in TJASLC module
   DT:float32 */
boolean_T SLC_PrevResetRSFF2_bool; /* '<S523>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrevResetRSFF_bool; /* '<S521>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrevResetUnitDelay_bool; /* '<S268>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_PrevReset_bool; /* '<S268>/Unit Delay' */

/* Debug */
boolean_T SLC_PrevTakeoverValidUnitDelay_bool; /* '<S7>/Unit Delay' */

/* Previous take over valid flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_RearAbortEdgRs_bool; /* '<S411>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RearAbortTunOffDly_sec; /* '<S431>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_RearCancleEdgRs_bool; /* '<S410>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RearCancleTunOffDly_sec; /* '<S430>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RearObjDecTurnOffDelay_sec[2]; /* '<S506>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_ResetTurnOffDelay_sec; /* '<S528>/Unit Delay' */

/* Reset turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RiLaneTypeTurnOffDelay_sec; /* '<S310>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RightSafeTrOnTime_sec; /* '<S511>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_RightTurnSignalOffDelay_sec; /* '<S545>/Unit Delay' */

/* Right turn signal turn off dealy time in sec in TJASLC module
   DT:single */
real32_T SLC_RightUnSafeTrOnTime_sec; /* '<S508>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_SameLaneChangeDetc_bool; /* '<S326>/Switch' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_StrongReadyBothSides_bool; /* '<S463>/AND1' */

/* Debug */
boolean_T SLC_TakeOverRSFF2_bool; /* '<S445>/Unit Delay' */

/* Adjacent  lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_TakeOverValidUnitDelay_bool; /* '<S1>/Unit Delay2' */

/* Take over valid Unit Delay--Used in TJASLC module
   DT:boolean */
real32_T SLC_TakeoverAbortTurnOffDelay_sec; /* '<S447>/Unit Delay' */

/* Lane centering active turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_TakeoverEdgeRising_bool; /* '<S433>/Unit Delay' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
real32_T SLC_TakeoverValidTurnOffDelay_sec; /* '<S446>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T SLC_TriggerLeft_bool; /* '<S516>/AND' */

/* Debug */
boolean_T SLC_TriggerRight_bool; /* '<S516>/AND1' */

/* Debug */
boolean_T SLC_TurnSignalOffBlckRSFF_bool; /* '<S499>/Unit Delay' */

/* SLC turn signal off RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_TurnSignalOffEF_bool; /* '<S384>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_TurnSignalTurnOffDelay_sec[2]; /* '<S519>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T SLC_UnitDelay_LePosY0_met; /* '<S326>/Unit Delay' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
real32_T SLC_UnitDelay_RiPosY0_met; /* '<S326>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_VehSpdTooLowInfo; /* '<S383>/OR2' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
boolean_T SLC_WeakReadyBothSides_bool; /* '<S463>/AND2' */

/* Debug */
boolean_T SLC_WeakReadyLeft_bool; /* '<S464>/AND' */

/* Debug */
boolean_T SLC_WeakReadyRight_bool; /* '<S465>/AND' */

/* Debug */
boolean_T STM_Cancel_bool; /* '<S547>/OR' */

/* Debug */
E_TJASTM_LatCtrlMode_nu STM_LatCtrlMdUnitDelay_bool; /* '<S1>/Unit Delay' */

/* Lateral control mode Unit Delay--Used in TJASTM module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
boolean_T STM_SuspendEdgeRising_bool; /* '<S605>/Unit Delay' */

/* Suspended state edge rising--Used in TJATTG module
   DT:boolean */
E_TJASTM_SysStateTJA_nu STM_SysStateUnitDelay_bool; /* '<S1>/Unit Delay1' */

/* System state Unit Delay--Used in TJASTM module
   DT:Enum: E_TJASTM_SysStateTJA_nu */
real32_T TJALKA_LeftLlineValidRD_Sec; /* '<S156>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T TJALKA_RightLlineValidRD_Sec; /* '<S157>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
int32_T TJASLC_PrevSLCHighLightID_nu; /* '<S375>/Unit Delay1' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T TJASLC_RSLeftLaneChangeWarn_nu; /* '<S376>/Unit Delay' */

/* Previou reset RSFlipFlop flag in lane change warning --Used in TJASLC module
   DT:boolean */
boolean_T TJASLC_RSRightLaneChangeWarn_nu; /* '<S377>/Unit Delay' */

/* Previou reset RSFlipFlop flag in lane change warning --Used in TJASLC module
   DT:boolean */
real32_T TJASTM_ACCActiveOvertime_sec; /* '<S563>/Unit Delay' */

/* Max maneuver time expired Time retrigger--Used in TJASLC module
   DT:float32 */
real32_T TJASTM_ACCOffTurnOffDelay_sec; /* '<S561>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
real32_T TJASTM_HandsOffWarnTurnOffDelay_sec; /* '<S562>/Unit Delay' */

/* Reset turn off delay time--Used in TJASLC module
   DT:float32 */
boolean_T TJASTM_PrevACCActOvrtm_bool; /* '<S8>/Unit Delay1' */

/* PrevACCActiveOvertime uint delay */
E_TJASTM_SysStateTJA_nu
    TJASTM_PrevSysStateTJAIn_envm; /* '<S546>/Unit Delay1' */

/* Previous lane boundary valid flag--Used in TJALKA module
   DT:Enum: E_TJALKA_LnBndValid_nu */
real32_T TJATOW_NPilotBrkPAudioPlay_sec; /* '<S659>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotBrkPUnitDealy_bool; /* '<S619>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotDoorAudioPlay_sec; /* '<S650>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotDoorOpenUnitDealy_bool; /* '<S622>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotEPBAudioPlay_sec; /* '<S653>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotEPBUnitDealy_bool; /* '<S625>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotErrorAudioPlay_sec; /* '<S657>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotErrorUnitDealy_bool; /* '<S616>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotHoodAudioPlay_sec; /* '<S652>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotHoodUnitDealy_bool; /* '<S624>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotLaneAudioPlay_sec; /* '<S662>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotLineUnitDealy_bool; /* '<S620>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotNoAudioPlay_sec; /* '<S655>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotNoUnitDealy_bool; /* '<S618>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotOffAudioPlay_sec; /* '<S656>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotOnAudioPlay_sec; /* '<S649>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotSafeAudioPlay_sec; /* '<S654>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotSafeUnitDealy_bool; /* '<S626>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotSeatBeltPAudioPlay_sec; /* '<S663>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotSeatBeltUnitDealy_bool; /* '<S621>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotSpdAudioPlay_sec; /* '<S658>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotSpdUnitDealy_bool; /* '<S617>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_NPilotTrunkAudioPlay_sec; /* '<S651>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_NPilotTrunkUnitDealy_bool; /* '<S623>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_PilotOverrideAudioPlay_sec; /* '<S660>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
real32_T TJATOW_PilotResumeAudioPlay_sec; /* '<S661>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
boolean_T TJATOW_RSFlipFlop_nu; /* '<S614>/Unit Delay' */

/* Previou reset RSFlipFlop flag in take over warning --Used in TJASLC module
   DT:boolean */
real32_T TJATOW_TakeOverTurnOffDelay_sec; /* '<S615>/Unit Delay' */

/* Take Over turn off delay time --Used in TJASTM module */
boolean_T TJATOW_TakeOverTurnOnDelay_nu; /* '<S610>/Unit Delay' */

/* Take Over Signal Unit Delay--Used in TJASTM module */
real32_T TJATTG_TgtTrajHeadAngUnitDy_rad; /* '<S1>/Unit Delay7' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
real32_T TJATTG_TgtTrajPosY0UnitDy_met; /* '<S1>/Unit Delay6' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
boolean_T TTG_CMBEnableUnitDelay_bool; /* '<S795>/Unit Delay' */

/* Combined date enable Unit Delay--Used in TJATTG module
   DT:boolean */
boolean_T TTG_CMBObjectCorridorUnitDelay_bool; /* '<S779>/Unit Delay2' */

/* CMB object corridor Unit Delay--Used in TJATTG module
   DT:boolean */
boolean_T TTG_CMBObjectCorridor_bool; /* '<S665>/AND' */

/* Debug */
boolean_T TTG_CMB_Enable_bool; /* '<S795>/Switch1' */

/* Debug */
real32_T TTG_CntrCrvPredictLowPass_1pm; /* '<S702>/Unit Delay' */

/* Control lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnCrvChngPredictLowPass_1pm2; /* '<S700>/Unit Delay' */

/* Control lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnCrvChngUnitDelay_1pm2; /* '<S690>/Unit Delay' */

/* Control lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnCrvUnitDelay_1pm; /* '<S691>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnHeadingUnitDelay_rad; /* '<S692>/Unit Delay' */

/* Control lane heading freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnLengthUnitDelay_met; /* '<S695>/Unit Delay' */

/* Control lane lenght freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_CntrLnPosY0UnitDelay_met; /* '<S693>/Unit Delay' */

/* Control lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
boolean_T TTG_CntrLnPredictEnable2EdgeRising_bool; /* '<S699>/Unit Delay' */

/* Control lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_CntrLnPredictEnableEdgeRising_bool; /* '<S701>/Unit Delay' */

/* Control lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_CntrLnResetEdgeRising_bool; /* '<S703>/Unit Delay' */

/* Control lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_Enable_bool; /* '<S798>/AND' */

/* Debug */
boolean_T TTG_LDEnableUnitDelay_bool; /* '<S796>/Unit Delay' */

/* Lane date enable Unit Delay--Used in TJATTG module
   DT:boolean */
boolean_T TTG_LD_Enable_bool; /* '<S796>/Switch1' */

/* Debug */
boolean_T TTG_LD_PredictFinish_bool; /* '<S766>/AND' */

/* Debug */
boolean_T TTG_LanePredictEdgeRising_bool; /* '<S742>/Unit Delay' */

/* Lane predict edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_LaneUpdate_bool; /* '<S764>/OR3' */

/* Debug */
real32_T TTG_LeCorridorCrvChngUnitDelay_1pm2; /* '<S751>/Unit Delay4' */

/* Left corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeCorridorCrvUnitDelay_1pm; /* '<S751>/Unit Delay3' */

/* Left corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeCorridorHeadingUnitDelay_rad; /* '<S751>/Unit Delay2' */

/* Left corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeCorridorLengthUnitDelay_met; /* '<S751>/Unit Delay5' */

/* Left corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeCorridorPosX0UnitDelay_met; /* '<S751>/Unit Delay' */

/* Left corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeCorridorPosY0UnitDelay_met; /* '<S751>/Unit Delay1' */

/* Left corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnCrvChngPredictLowPass_1pm2; /* '<S717>/Unit Delay' */

/* Left lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnCrvChngUnitDelay_1pm2; /* '<S707>/Unit Delay' */

/* Left lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnCrvPredictLowPass_1pm; /* '<S719>/Unit Delay' */

/* Left lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnCrvUnitDelay_1pm; /* '<S708>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnHeadingUnitDelay_rad; /* '<S709>/Unit Delay' */

/* Left lane heading freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnLengthUnitDelay_met; /* '<S712>/Unit Delay' */

/* Left lane lenght freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_LeLnPosY0UnitDelay_met; /* '<S710>/Unit Delay' */

/* Left lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
boolean_T TTG_LeLnPredictEnable2EdgeRising_bool; /* '<S716>/Unit Delay' */

/* Left lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_LeLnPredictEnableEdgeRising_bool; /* '<S718>/Unit Delay' */

/* Left lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_LeLnResetEdgeRising_bool; /* '<S720>/Unit Delay' */

/* Left lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_ODEnableUnitDelay_bool; /* '<S794>/Unit Delay' */

/* Object date enable Unit Delay--Used in TJATTG module
   DT:boolean */
boolean_T TTG_OD_Enable_bool; /* '<S794>/Switch1' */

/* Debug */
boolean_T TTG_ObjectCorridorEdgeFalling_bool; /* '<S677>/Unit Delay' */

/* Object corridor flag edge falling--Used in TJATTG module
   DT:boolean */
boolean_T TTG_ObjectCorridorEdgeRising_bool; /* '<S678>/Unit Delay' */

/* Object corridor flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_ObjectUpdate_bool; /* '<S765>/OR2' */

/* Debug */
real32_T TTG_OdoPosXDelayRe_met; /* '<S740>/Unit Delay1' */

/* Odometrie position X Delay_RE--Used in TJATTG module
   DT:float32 */
real32_T TTG_OdoPosYDelayRe_met; /* '<S739>/Unit Delay1' */

/* Odometrie position Y Delay_RE--Used in TJATTG module
   DT:float32 */
real32_T TTG_OdoYawDelayRe_rad; /* '<S738>/Unit Delay1' */

/* Odometrie yaw angle Delay_RE--Used in TJATTG module
   DT:float32 */
boolean_T TTG_PredictEnableEdgeRising_bool; /* '<S741>/Unit Delay' */

/* Lane predict enable edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_PredictEnableRSFF_bool; /* '<S743>/Unit Delay' */

/* Lane prediction enable RSFlipFlop flag--Used in TJATTG module
   DT:boolean */
boolean_T TTG_PredictEnableUnitDelay_bool; /* '<S797>/Unit Delay' */

/* Predict enable Unit Delay--Used in TJATTG module
   DT:boolean */
boolean_T TTG_Predict_Enable_bool; /* '<S797>/Switch1' */

/* Debug */
boolean_T TTG_PredictionEnableEdgeFalling_bool; /* '<S777>/Unit Delay' */

/* Prediction enable flag edge falling--Used in TJATTG module
   DT:boolean */
boolean_T TTG_PrevLnLengResetUnitDelay_bool; /* '<S667>/Unit Delay' */
boolean_T TTG_Reset_bool;                    /* '<S798>/OR1' */

/* Debug */
real32_T TTG_RiCorridorCrvChngUnitDelay_1pm2; /* '<S816>/Unit Delay4' */

/* Right corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCorridorCrvUnitDelay_1pm; /* '<S816>/Unit Delay3' */

/* Right corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCorridorHeadingUnitDelay_rad; /* '<S816>/Unit Delay2' */

/* Right corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCorridorLengthUnitDelay_met; /* '<S816>/Unit Delay5' */

/* Right corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCorridorPosX0UnitDelay_met; /* '<S816>/Unit Delay' */

/* Right corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCorridorPosY0UnitDelay_met; /* '<S816>/Unit Delay1' */

/* Right corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiCrvPredictLowPass_1pm; /* '<S736>/Unit Delay' */

/* Right lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnCrvChngPredictLowPass_1pm2; /* '<S734>/Unit Delay' */

/* Right lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnCrvChngUnitDelay_1pm2; /* '<S724>/Unit Delay' */

/* Left lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnCrvUnitDelay_1pm; /* '<S725>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnHeadingUnitDelay_rad; /* '<S726>/Unit Delay' */

/* Left lane heading freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnLengthUnitDelay_met; /* '<S729>/Unit Delay' */

/* Left lane lenght freeze output--Used in TJATTG module
   DT:float32 */
real32_T TTG_RiLnPosY0UnitDelay_met; /* '<S727>/Unit Delay' */

/* Left lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
boolean_T TTG_RiLnPredictEnable2EdgeRising_bool; /* '<S733>/Unit Delay' */

/* Right lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_RiLnPredictEnableEdgeRising_bool; /* '<S735>/Unit Delay' */

/* Right lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
boolean_T TTG_RiLnResetEdgeRising_bool; /* '<S737>/Unit Delay' */

/* Left lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
E_TJASTM_LatCtrlMode_nu TTG_STMLatCtrlMode2_Enum; /* '<S803>/Unit Delay' */

/* Previous STM lateral control mode 2--Used in TJATTG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
E_TJASTM_LatCtrlMode_nu TTG_STMLatCtrlMode_Enum; /* '<S779>/Unit Delay1' */

/* Previous STM lateral control mode--Used in TJATTG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
E_TJASTM_SysStateTJA_nu TTG_STMSystemState_Enum; /* '<S779>/Unit Delay' */

/* Previous STM system state--Used in TJATTG module
   DT:Enum: E_TJASTM_SysStateTJA_nu */
real32_T TTG_TgtCorridorCrvChngUnitDelay_1pm2; /* '<S867>/Unit Delay4' */

/* Target corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_TgtCorridorCrvUnitDelay_1pm; /* '<S867>/Unit Delay3' */

/* Target corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_TgtCorridorHeadingUnitDelay_rad; /* '<S867>/Unit Delay2' */

/* Target corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_TgtCorridorLengthUnitDelay_met; /* '<S867>/Unit Delay5' */

/* Target corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_TgtCorridorPosX0UnitDelay_met; /* '<S867>/Unit Delay' */

/* Target corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
real32_T TTG_TgtCorridorPosY0UnitDelay_met; /* '<S867>/Unit Delay1' */

/* Target corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
Bus_TgtTrajAndCridrBnd_nu
    TTG_TgtTrajAndCridrBndUnitDelay_bus; /* '<S676>/Unit Delay1' */

/* Target trajectory and corridor boundary unit delay--Used in TJATTG module
   DT:Bus: Bus_TgtTrajAndCridrBnd_nu */
real32_T TTG_TransitionFactorAStopwatch_sec; /* '<S801>/Unit Delay' */

/* Transition factor A stopwatch time--Used in TJATTG module
   DT:float32 */
real32_T TTG_TransitionTimeTurnOffDelayWithRst_sec; /* '<S808>/Unit Delay' */

/* Transition time Turn Off Delay With Rst--Used in TJATTG module
   DT:float32 */
real32_T TTG_TransitionTimeUnitDelay_sec; /* '<S804>/Unit Delay' */

/* Transition time Unit Delay--Used in TJATTG module
   DT:float32 */
E_TJASLC_AbortState_nu TVG_AbortStateUnitDy_Enum; /* '<S888>/Unit Delay2' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
E_TJASTM_LatCtrlMode_nu TVG_LatCtrlMode_Enum; /* '<S920>/Unit Delay' */

/* Rampout lateral control mode--Used in TJATVG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
boolean_T TVG_LatMovStartEdgeRising_bool; /* '<S915>/Unit Delay' */

/* move start edge rising--Used in TJATVG module
   DT:boolean */
boolean_T TVG_LatMovStartRSFF_bool; /* '<S918>/Unit Delay' */

/* Lateral moving start RSFlipFlop flag--Used in TJATVG module
   DT:boolean */
real32_T TVG_PredictionEnableTurnOffDelay_sec; /* '<S965>/Unit Delay' */

/* Prediction enable turn off delay time--Used in TJATVG module
   DT:float32 */
boolean_T TVG_PredictionEnableUnitDelay_bool; /* '<S963>/Unit Delay' */

/* Prediction enable Unit Delay--Used in TJATVG module
   DT:boolean */
E_TJASTM_LatCtrlMode_nu
    TVG_PrevLatCtlModeUnitDelay_bool; /* '<S890>/Unit Delay' */

/* Previous lateral control mode--Used in TJATVG module
   Enum: E_TJASTM_LatCtrlMode_nu */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Model step function */
void TJASA_step(void) {
    /* local block i/o variables */
    uint32_T rtb_ex_sfun_set_bit;
    uint32_T rtb_ex_sfun_set_bit_fv5m;
    uint32_T rtb_ex_sfun_set_bit_hcfw;
    uint32_T rtb_ex_sfun_set_bit_nkl2;
    uint32_T rtb_ex_sfun_set_bit_hiin;
    uint32_T rtb_ex_sfun_set_bit_m25k;
    uint32_T rtb_ex_sfun_set_bit_d0a5;
    uint32_T rtb_ex_sfun_set_bit_gaa0;
    uint32_T rtb_ex_sfun_set_bit_jofi;
    uint32_T rtb_ex_sfun_set_bit_jx5q;
    boolean_T rtb_VectorConcatenate[16];
    boolean_T rtb_VectorConcatenate_br2p[5];
    boolean_T rtb_VectorConcatenate_maf5[16];
    boolean_T rtb_VectorConcatenate_gygi[8];
    int32_T GEN_AllStateAvailable_bool_tmp;
    int32_T rtb_GEN_BrakePadelInvalid__hw14;
    int32_T rtb_GEN_WR_Custom_bool_tmp;
    real32_T rtb_ABPR_LeLnClthHeading_rad;
    real32_T rtb_ABPR_LeLnClthLength_met;
    real32_T rtb_ABPR_LeLnClthPosY0_met;
    real32_T rtb_ABPR_RiLnClthCrvChng_1pm2;
    real32_T rtb_ABPR_RiLnClthCrv_1pm;
    real32_T rtb_ABPR_RiLnClthHeading_rad;
    real32_T rtb_ABPR_RiLnClthLength_met;
    real32_T rtb_ABPR_RiLnClthPosY0_met;
    real32_T rtb_Abs_hlug;
    real32_T rtb_Abs_ob4k;
    real32_T rtb_Add_fbcu;
    real32_T rtb_CrvChng_1pm2_f0cp;
    real32_T rtb_CrvChng_1pm2_m00x;
    real32_T rtb_Crv_1pm_b0ly;
    real32_T rtb_Crv_1pm_mhaw;
    real32_T rtb_Heading_rad;
    real32_T rtb_Heading_rad_poq4;
    real32_T rtb_Length_met;
    real32_T rtb_Length_met_ffe5;
    real32_T rtb_Length_met_kx1h;
    real32_T rtb_Length_out_ltpy;
    real32_T rtb_PosX0_met_ckhb;
    real32_T rtb_PosX0_met_exq3;
    real32_T rtb_PosY0_met;
    real32_T rtb_PosY0_met_bzv0;
    real32_T rtb_PosY0_met_cmjs;
    real32_T rtb_PosY0_met_dafa;
    real32_T rtb_PosY0_met_lcpf;
    real32_T rtb_PosY0_met_o0pf;
    real32_T rtb_Subtract_if0w_idx_0;
    real32_T rtb_Subtract_if0w_idx_1;
    real32_T rtb_Switch_ithu;
    real32_T rtb_UnitDelay;
    real32_T rtb_UnitDelay1;
    real32_T rtb_UnitDelay1_ppcw;
    real32_T rtb_UnitDelay2;
    real32_T rtb_UnitDelay2_gjo3;
    real32_T rtb_UnitDelay3;
    real32_T rtb_UnitDelay3_ikos;
    real32_T rtb_UnitDelay4;
    real32_T rtb_UnitDelay4_jffl;
    real32_T rtb_UnitDelay5;
    real32_T rtb_UnitDelay5_bx0r;
    real32_T rtb_UnitDelay6;
    real32_T rtb_UnitDelay6_e5dw;
    real32_T rtb_UnitDelay7;
    real32_T rtb_UnitDelay7_gp10;
    real32_T rtb_UnitDelay_h1k2;
    real32_T rtb_out;
    real32_T rtb_uDLookupTable1_eoam;
    real32_T rtb_uDLookupTable_prmv_tmp;
    real32_T rtb_x_yaw;
    real32_T rtb_x_yaw_bzv5;
    real32_T rtb_x_yaw_hpvs_tmp;
    real32_T rtb_x_yaw_hyuw;
    real32_T rtb_y_fcnl;
    E_TJASLC_LaneChangeWarning_nu rtb_Switch28;
    E_TJASLC_ManeuverState_nu rtb_SLC_PrevManeuverState_nu;
    E_TJASTM_LatCtrlMode_nu rtb_TJASTM_PrevLatCtrlMd_nu;
    int8_T rtb_DataTypeConversion;
    int8_T rtb_DataTypeConversion1_duje;
    uint8_T rtb_DataTypeConversion1;
    uint8_T rtb_TTG_Switch_nu;
    boolean_T rtb_VectorConcatenate_aisn[16];
    boolean_T rtb_VectorConcatenate_mj2c[8];
    boolean_T guard1 = false;
    boolean_T rtb_AND18;
    boolean_T rtb_AND1_h3xh;
    boolean_T rtb_AND2;
    boolean_T rtb_AND2_emk5;
    boolean_T rtb_AND5_c5w4;
    boolean_T rtb_AND_cvae;
    boolean_T rtb_AND_eqpk;
    boolean_T rtb_AND_gpqq;
    boolean_T rtb_AND_ihx5;
    boolean_T rtb_AND_ixtq;
    boolean_T rtb_AND_jt3t;
    boolean_T rtb_AND_laqq;
    boolean_T rtb_AND_nxkg;
    boolean_T rtb_AND_ojra;
    boolean_T rtb_AND_pjo1;
    boolean_T rtb_AND_psoa;
    boolean_T rtb_CMB_OFOLnLengthInvalid_bool;
    boolean_T rtb_CMB_OFOObjLengthInvalid_boo;
    boolean_T rtb_CMB_OFOVelocityInvalid_bool;
    boolean_T rtb_CMB_VelocityValid_bool;
    boolean_T rtb_Equal1_p2mp;
    boolean_T rtb_Equal2_kbd5;
    boolean_T rtb_Equal_awdj;
    boolean_T rtb_Equal_blkb;
    boolean_T rtb_Equal_ciay;
    boolean_T rtb_Equal_i3l2_idx_0;
    boolean_T rtb_Equal_i3l2_idx_1;
    boolean_T rtb_GEN_BlockTimerExpired_bool;
    boolean_T rtb_GEN_BrakePadelInvalid__gzuo;
    boolean_T rtb_GEN_Cancel_Custom_bool;
    boolean_T rtb_GEN_DrvNotBuckledUp_bool;
    boolean_T rtb_GEN_MaxSteerAngleExceeded_b;
    boolean_T rtb_GEN_VehStInvalid_bool;
    boolean_T rtb_GEN_WR_Custom_bool;
    boolean_T rtb_GreaterThan_bpjw_idx_0;
    boolean_T rtb_GreaterThan_bpjw_idx_1;
    boolean_T rtb_LKA_Dist2BndsValid_bool;
    boolean_T rtb_LKA_LaneWidthValid_bool;
    boolean_T rtb_LKA_LengthValid_bool;
    boolean_T rtb_LKA_NoLaneChange_bool;
    boolean_T rtb_LKA_RadiusValid_bool;
    boolean_T rtb_LKA_TakeOverValid_bool;
    boolean_T rtb_LKA_VelocityValid_bool;
    boolean_T rtb_LessThan_ko2v;
    boolean_T rtb_NOT_auv4;
    boolean_T rtb_NotEqual1_cuxj;
    boolean_T rtb_NotEqual2_emxm;
    boolean_T rtb_NotEqual3_lcsh;
    boolean_T rtb_NotEqual_ozi5_tmp;
    boolean_T rtb_NotEqual_ozi5_tmp_0;
    boolean_T rtb_OBF_VelocityValid_bool;
    boolean_T rtb_OR2_pwtt;
    boolean_T rtb_OR_baue;
    boolean_T rtb_OR_bsb5;
    boolean_T rtb_OR_d1pw;
    boolean_T rtb_OR_idx_0;
    boolean_T rtb_OR_idx_1;
    boolean_T rtb_OR_imjz;
    boolean_T rtb_OR_imzg_idx_0;
    boolean_T rtb_OR_imzg_idx_1;
    boolean_T rtb_OR_lkl2;
    boolean_T rtb_OR_ly14;
    boolean_T rtb_SLC_DriverTriggerResetRight;
    boolean_T rtb_SLC_LeAdjLaneValid_bool;
    boolean_T rtb_SLC_LeAdjLaneWidthValid_boo;
    boolean_T rtb_SLC_LeTurnSignalOn_bool;
    boolean_T rtb_SLC_ObjectDetectedLeftRear_;
    boolean_T rtb_SLC_ObjectDetectedRightRear;
    boolean_T rtb_SLC_PrevDriverTriggerResetR;
    boolean_T rtb_SLC_RearAbort_bool;
    boolean_T rtb_SLC_RiAdjLaneValid_bool;
    boolean_T rtb_SLC_RiAdjLaneWidthValid_boo;
    boolean_T rtb_SLC_RiTurnSignalOn_bool;
    boolean_T rtb_SLC_VelocityValid_bool;
    boolean_T rtb_Switch1_eo0n_idx_0;
    boolean_T rtb_Switch1_eo0n_idx_1;
    boolean_T rtb_TJASTM_ACCIsOFF_bool;
    boolean_T rtb_TTG_CMB_bool;
    boolean_T rtb_TVG_Rampout_bool;

    /* UnitDelay: '<S12>/Unit Delay' */
    rtb_UnitDelay = CMB_PrevCntrCrv1UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay1' */
    rtb_UnitDelay1 = CMB_PrevCntrCrv2UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay2' */
    rtb_UnitDelay2 = CMB_PrevCntrCrv3UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay3' */
    rtb_UnitDelay3 = CMB_PrevCntrCrv4UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay4' */
    rtb_UnitDelay4 = CMB_PrevCntrCrv5UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay5' */
    rtb_UnitDelay5 = CMB_PrevCntrCrv6UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay6' */
    rtb_UnitDelay6 = CMB_PrevCntrCrv7UnitDelay_1pm;

    /* UnitDelay: '<S12>/Unit Delay7' */
    rtb_UnitDelay7 = CMB_PrevCntrCrv8UnitDelay_1pm;

    /* Abs: '<S12>/Abs' incorporates:
     *  Constant: '<S12>/Constant'
     *  Inport: '<Root>/Inport9'
     *  Product: '<S12>/Divide'
     *  Sum: '<S12>/Add'
     *  Sum: '<S12>/Add1'
     *  Sum: '<S12>/Add2'
     *  Sum: '<S12>/Add3'
     *  Sum: '<S12>/Add4'
     *  Sum: '<S12>/Add5'
     *  Sum: '<S12>/Add6'
     *  Sum: '<S12>/Add7'
     *  Sum: '<S12>/Add8'
     *  Sum: '<S12>/Subtract'
     *  UnitDelay: '<S12>/Unit Delay'
     *  UnitDelay: '<S12>/Unit Delay1'
     *  UnitDelay: '<S12>/Unit Delay2'
     *  UnitDelay: '<S12>/Unit Delay3'
     *  UnitDelay: '<S12>/Unit Delay4'
     *  UnitDelay: '<S12>/Unit Delay5'
     *  UnitDelay: '<S12>/Unit Delay6'
     *  UnitDelay: '<S12>/Unit Delay7'
     *  UnitDelay: '<S12>/Unit Delay8'
     */
    TJACMB_LaneCrvStdDev_nu = fabsf(
        ABPR_CntrLnClthCrv_1pm -
        ((((((((((ABPR_CntrLnClthCrv_1pm + CMB_PrevCntrCrv1UnitDelay_1pm) +
                 CMB_PrevCntrCrv2UnitDelay_1pm) +
                CMB_PrevCntrCrv3UnitDelay_1pm) +
               CMB_PrevCntrCrv4UnitDelay_1pm) +
              CMB_PrevCntrCrv5UnitDelay_1pm) +
             CMB_PrevCntrCrv6UnitDelay_1pm) +
            CMB_PrevCntrCrv7UnitDelay_1pm) +
           CMB_PrevCntrCrv8UnitDelay_1pm) +
          CMB_PrevCntrCrv9UnitDelay_1pm) /
         10.0F));

    /* UnitDelay: '<S13>/Unit Delay' */
    rtb_UnitDelay_h1k2 = CMB_PrevTgtCrv1UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay1' */
    rtb_UnitDelay1_ppcw = CMB_PrevTgtCrv2UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay2' */
    rtb_UnitDelay2_gjo3 = CMB_PrevTgtCrv3UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay3' */
    rtb_UnitDelay3_ikos = CMB_PrevTgtCrv4UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay4' */
    rtb_UnitDelay4_jffl = CMB_PrevTgtCrv5UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay5' */
    rtb_UnitDelay5_bx0r = CMB_PrevTgtCrv6UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay6' */
    rtb_UnitDelay6_e5dw = CMB_PrevTgtCrv7UnitDelay_1pm;

    /* UnitDelay: '<S13>/Unit Delay7' */
    rtb_UnitDelay7_gp10 = CMB_PrevTgtCrv8UnitDelay_1pm;

    /* Abs: '<S13>/Abs' incorporates:
     *  Constant: '<S13>/Constant'
     *  Inport: '<Root>/Inport38'
     *  Product: '<S13>/Divide'
     *  Sum: '<S13>/Add'
     *  Sum: '<S13>/Add1'
     *  Sum: '<S13>/Add2'
     *  Sum: '<S13>/Add3'
     *  Sum: '<S13>/Add4'
     *  Sum: '<S13>/Add5'
     *  Sum: '<S13>/Add6'
     *  Sum: '<S13>/Add7'
     *  Sum: '<S13>/Add8'
     *  Sum: '<S13>/Subtract'
     *  UnitDelay: '<S13>/Unit Delay'
     *  UnitDelay: '<S13>/Unit Delay1'
     *  UnitDelay: '<S13>/Unit Delay2'
     *  UnitDelay: '<S13>/Unit Delay3'
     *  UnitDelay: '<S13>/Unit Delay4'
     *  UnitDelay: '<S13>/Unit Delay5'
     *  UnitDelay: '<S13>/Unit Delay6'
     *  UnitDelay: '<S13>/Unit Delay7'
     *  UnitDelay: '<S13>/Unit Delay8'
     */
    TJACMB_TraceCrvStdDev_nu =
        fabsf(ODPFOH_TgtObjCrv_1pm -
              ((((((((((ODPFOH_TgtObjCrv_1pm + CMB_PrevTgtCrv1UnitDelay_1pm) +
                       CMB_PrevTgtCrv2UnitDelay_1pm) +
                      CMB_PrevTgtCrv3UnitDelay_1pm) +
                     CMB_PrevTgtCrv4UnitDelay_1pm) +
                    CMB_PrevTgtCrv5UnitDelay_1pm) +
                   CMB_PrevTgtCrv6UnitDelay_1pm) +
                  CMB_PrevTgtCrv7UnitDelay_1pm) +
                 CMB_PrevTgtCrv8UnitDelay_1pm) +
                CMB_PrevTgtCrv9UnitDelay_1pm) /
               10.0F));

    /* Switch: '<S891>/Switch2' incorporates:
     *  Constant: '<S891>/Constant22'
     *  Constant: '<S891>/Constant23'
     *  Constant: '<S891>/Constant24'
     *  Constant: '<S891>/Constant25'
     *  Inport: '<Root>/Inport93'
     *  RelationalOperator: '<S891>/Equal'
     *  RelationalOperator: '<S891>/Equal1'
     *  Switch: '<S891>/Switch1'
     */
    if (((int32_T)LCFRCV_DrivingMode_nu) == 3) {
        rtb_DataTypeConversion1 = TJATVG_MD3DeratingLevel_C_perc;
    } else if (((int32_T)LCFRCV_DrivingMode_nu) == 2) {
        /* Switch: '<S891>/Switch1' incorporates:
         *  Constant: '<S891>/Constant21'
         */
        rtb_DataTypeConversion1 = TJATVG_MD2DeratingLevel_C_perc;
    } else {
        rtb_DataTypeConversion1 = TJATVG_MD1DeratingLevel_C_perc;
    }

    /* End of Switch: '<S891>/Switch2' */

    /* Product: '<S891>/Product' incorporates:
     *  Inport: '<Root>/Inport5'
     *  Lookup_n-D: '<S891>/1-D Lookup Table'
     */
    TJATVG_DeratingLevel_nu = (uint8_T)((
        real32_T)(((real32_T)rtb_DataTypeConversion1) *
                  look1_iflf_binlxpw(
                      ABPR_LaneWidth_met,
                      ((const real32_T *)&(TJATVG_LaneWidth_Bx_met[0])),
                      ((const real32_T *)&(TJATVG_LWDeratLvlScalFact_Cr_nu[0])),
                      5U)));

    /* Constant: '<S891>/Constant15' */
    TJATVG_DistYTolLeTgtArea_met = TJATVG_DistYToleranceLeftTgtArea_C_met;

    /* Constant: '<S891>/Constant16' */
    TJATVG_DistYTolRiTgtArea_met = TJATVG_DistYToleranceRightTgtArea_C_met;

    /* Constant: '<S891>/Constant13' */
    TJATVG_FTireAclMax_mps2 = TJATVG_FTireAccelMax_C_mps2;

    /* Constant: '<S891>/Constant14' */
    TJATVG_FTireAclMin_mps2 = TJATVG_FTireAccelMin_C_mps2;

    /* Constant: '<S891>/Constant17' */
    TJATVG_WeightTgtDistY_nu = TJATVG_WeightTgtDistY_C_nu;

    /* Constant: '<S891>/Constant18' */
    TJATVG_WeightEndTime_nu = TJATVG_WeightEndTime_C_nu;

    /* Constant: '<S891>/Constant4' */
    TJATVG_PredTimeCrv_sec = TJATVG_PredictionTimeCrv_C_sec;

    /* Constant: '<S891>/Constant10' */
    TJATVG_PredTimeHeadAng_sec = TJATVG_PredTimeHeadAng_C_sec;

    /* Constant: '<S891>/Constant11' */
    TJATVG_TriggerReplan_nu = TJATVG_TriggerReplan_C_bool;

    /* Constant: '<S893>/Constant1' */
    TJATVG_HighStatAccu_bool = TJATVG_HighStatAccu_C_bool;

    /* Constant: '<S962>/Constant1' */
    TJATVG_MaxTrqScalLimit_nu = TJATVG_MaxTrqScalLimit_C_nu;

    /* Constant: '<S891>/Constant12' */
    TJATVG_MaxJerkAllowed_mps3 = TJATVG_MaxJerkAllowed_C_mps3;

    /* RelationalOperator: '<S152>/GreaterThan1' incorporates:
     *  Constant: '<S152>/Constant1'
     *  Inport: '<Root>/Inport17'
     *  Inport: '<Root>/Inport18'
     */
    rtb_Switch1_eo0n_idx_0 =
        (ABPR_LeCrvQuality_perc > TJALKA_CrvQualityMin_C_perc);
    rtb_Switch1_eo0n_idx_1 =
        (ABPR_RiCrvQuality_perc > TJALKA_CrvQualityMin_C_perc);

    /* Switch: '<S174>/Switch' incorporates:
     *  Constant: '<S152>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S174>/Max'
     *  Sum: '<S174>/Subtract'
     *  Switch: '<S174>/Switch1'
     *  UnaryMinus: '<S174>/Unary Minus'
     *  UnitDelay: '<S174>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_0) {
        LKA_CrvQualTurnOffDelay_sec[0] = TJALKA_CrvQualTurnOffTime_C_sec;
    } else {
        LKA_CrvQualTurnOffDelay_sec[0] = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               LKA_CrvQualTurnOffDelay_sec[0]) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S174>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S174>/GreaterThan'
     *  Switch: '<S174>/Switch'
     *  UnaryMinus: '<S174>/Unary Minus1'
     *  UnitDelay: '<S174>/Unit Delay'
     */
    rtb_OR_idx_0 = (rtb_Switch1_eo0n_idx_0 || (LKA_CrvQualTurnOffDelay_sec[0] >
                                               (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S174>/Switch' incorporates:
     *  Constant: '<S152>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S174>/Max'
     *  Sum: '<S174>/Subtract'
     *  Switch: '<S174>/Switch1'
     *  UnaryMinus: '<S174>/Unary Minus'
     *  UnitDelay: '<S174>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_1) {
        LKA_CrvQualTurnOffDelay_sec[1] = TJALKA_CrvQualTurnOffTime_C_sec;
    } else {
        LKA_CrvQualTurnOffDelay_sec[1] = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               LKA_CrvQualTurnOffDelay_sec[1]) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S174>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S174>/GreaterThan'
     *  Switch: '<S174>/Switch'
     *  UnaryMinus: '<S174>/Unary Minus1'
     *  UnitDelay: '<S174>/Unit Delay'
     */
    rtb_OR_idx_1 = (rtb_Switch1_eo0n_idx_1 || (LKA_CrvQualTurnOffDelay_sec[1] >
                                               (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Sum: '<S152>/Add1' incorporates:
     *  Constant: '<S152>/Constant'
     *  Constant: '<S152>/Constant1'
     */
    rtb_DataTypeConversion1 =
        (uint8_T)(((uint32_T)TJALKA_CrvQualityHyst_C_perc) +
                  ((uint32_T)TJALKA_CrvQualityMin_C_perc));

    /* RelationalOperator: '<S152>/GreaterThan' incorporates:
     *  Inport: '<Root>/Inport17'
     *  Inport: '<Root>/Inport18'
     */
    rtb_Switch1_eo0n_idx_0 = (ABPR_LeCrvQuality_perc > rtb_DataTypeConversion1);
    rtb_Switch1_eo0n_idx_1 = (ABPR_RiCrvQuality_perc > rtb_DataTypeConversion1);

    /* Switch: '<S175>/Switch' incorporates:
     *  Constant: '<S152>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S175>/Max'
     *  Sum: '<S175>/Subtract'
     *  Switch: '<S175>/Switch1'
     *  UnaryMinus: '<S175>/Unary Minus'
     *  UnitDelay: '<S175>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_0) {
        LKA_CrvQualTurnOnDelay_sec[0] =
            fmaxf(LKA_CrvQualTurnOnDelay_sec[0], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        LKA_CrvQualTurnOnDelay_sec[0] = TJALKA_CrvQualTurnOnTime_C_sec;
    }

    /* Switch: '<S173>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S152>/NOT'
     *  Logic: '<S175>/AND'
     *  RelationalOperator: '<S175>/LessThanOrEqual'
     *  Switch: '<S173>/Switch1'
     *  Switch: '<S175>/Switch'
     *  UnaryMinus: '<S175>/Unary Minus1'
     *  UnitDelay: '<S173>/Unit Delay'
     *  UnitDelay: '<S175>/Unit Delay'
     */
    LKA_CrvQualRSFF_bool[0] =
        (rtb_OR_idx_0 &&
         ((rtb_Switch1_eo0n_idx_0 &&
           (LKA_CrvQualTurnOnDelay_sec[0] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (LKA_CrvQualRSFF_bool[0])));

    /* Switch: '<S175>/Switch' incorporates:
     *  Constant: '<S152>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S175>/Max'
     *  Sum: '<S175>/Subtract'
     *  Switch: '<S175>/Switch1'
     *  UnaryMinus: '<S175>/Unary Minus'
     *  UnitDelay: '<S175>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_1) {
        LKA_CrvQualTurnOnDelay_sec[1] =
            fmaxf(LKA_CrvQualTurnOnDelay_sec[1], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        LKA_CrvQualTurnOnDelay_sec[1] = TJALKA_CrvQualTurnOnTime_C_sec;
    }

    /* Switch: '<S173>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S152>/NOT'
     *  Logic: '<S175>/AND'
     *  RelationalOperator: '<S175>/LessThanOrEqual'
     *  Switch: '<S173>/Switch1'
     *  Switch: '<S175>/Switch'
     *  UnaryMinus: '<S175>/Unary Minus1'
     *  UnitDelay: '<S173>/Unit Delay'
     *  UnitDelay: '<S175>/Unit Delay'
     */
    LKA_CrvQualRSFF_bool[1] =
        (rtb_OR_idx_1 &&
         ((rtb_Switch1_eo0n_idx_1 &&
           (LKA_CrvQualTurnOnDelay_sec[1] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (LKA_CrvQualRSFF_bool[1])));

    /* RelationalOperator: '<S154>/GreaterThan1' incorporates:
     *  Constant: '<S154>/Constant1'
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     */
    rtb_Switch1_eo0n_idx_0 =
        (ABPR_LeLnQuality_perc > TJALKA_LaneQualityMin_C_perc);
    rtb_Switch1_eo0n_idx_1 =
        (ABPR_RiLnQuality_perc > TJALKA_LaneQualityMin_C_perc);

    /* Switch: '<S179>/Switch' incorporates:
     *  Constant: '<S154>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S179>/Max'
     *  Sum: '<S179>/Subtract'
     *  Switch: '<S179>/Switch1'
     *  UnaryMinus: '<S179>/Unary Minus'
     *  UnitDelay: '<S179>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_0) {
        LKA_LnQualTurnOffDelay_sec[0] = TJALKA_LnQualTurnOffTime_C_sec;
    } else {
        LKA_LnQualTurnOffDelay_sec[0] =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, LKA_LnQualTurnOffDelay_sec[0]) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S179>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S179>/GreaterThan'
     *  Switch: '<S179>/Switch'
     *  UnaryMinus: '<S179>/Unary Minus1'
     *  UnitDelay: '<S179>/Unit Delay'
     */
    rtb_OR_idx_0 = (rtb_Switch1_eo0n_idx_0 || (LKA_LnQualTurnOffDelay_sec[0] >
                                               (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S179>/Switch' incorporates:
     *  Constant: '<S154>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S179>/Max'
     *  Sum: '<S179>/Subtract'
     *  Switch: '<S179>/Switch1'
     *  UnaryMinus: '<S179>/Unary Minus'
     *  UnitDelay: '<S179>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_1) {
        LKA_LnQualTurnOffDelay_sec[1] = TJALKA_LnQualTurnOffTime_C_sec;
    } else {
        LKA_LnQualTurnOffDelay_sec[1] =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, LKA_LnQualTurnOffDelay_sec[1]) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S179>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S179>/GreaterThan'
     *  Switch: '<S179>/Switch'
     *  UnaryMinus: '<S179>/Unary Minus1'
     *  UnitDelay: '<S179>/Unit Delay'
     */
    rtb_OR_idx_1 = (rtb_Switch1_eo0n_idx_1 || (LKA_LnQualTurnOffDelay_sec[1] >
                                               (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Sum: '<S154>/Add1' incorporates:
     *  Constant: '<S154>/Constant'
     *  Constant: '<S154>/Constant1'
     */
    rtb_DataTypeConversion1 =
        (uint8_T)(((uint32_T)TJALKA_LaneQualityHyst_C_perc) +
                  ((uint32_T)TJALKA_LaneQualityMin_C_perc));

    /* RelationalOperator: '<S154>/GreaterThan' incorporates:
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     */
    rtb_Switch1_eo0n_idx_0 = (ABPR_LeLnQuality_perc > rtb_DataTypeConversion1);
    rtb_Switch1_eo0n_idx_1 = (ABPR_RiLnQuality_perc > rtb_DataTypeConversion1);

    /* Switch: '<S180>/Switch' incorporates:
     *  Constant: '<S154>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S180>/Max'
     *  Sum: '<S180>/Subtract'
     *  Switch: '<S180>/Switch1'
     *  UnaryMinus: '<S180>/Unary Minus'
     *  UnitDelay: '<S180>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_0) {
        LKA_LnQualTurnOnDelay_sec[0] =
            fmaxf(LKA_LnQualTurnOnDelay_sec[0], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        LKA_LnQualTurnOnDelay_sec[0] = TJALKA_LnQualTurnOnTime_C_sec;
    }

    /* Switch: '<S178>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S154>/NOT'
     *  Logic: '<S180>/AND'
     *  RelationalOperator: '<S180>/LessThanOrEqual'
     *  Switch: '<S178>/Switch1'
     *  Switch: '<S180>/Switch'
     *  UnaryMinus: '<S180>/Unary Minus1'
     *  UnitDelay: '<S178>/Unit Delay'
     *  UnitDelay: '<S180>/Unit Delay'
     */
    LKA_LnQualRSFF_bool[0] =
        (rtb_OR_idx_0 &&
         ((rtb_Switch1_eo0n_idx_0 &&
           (LKA_LnQualTurnOnDelay_sec[0] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (LKA_LnQualRSFF_bool[0])));

    /* Switch: '<S180>/Switch' incorporates:
     *  Constant: '<S154>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S180>/Max'
     *  Sum: '<S180>/Subtract'
     *  Switch: '<S180>/Switch1'
     *  UnaryMinus: '<S180>/Unary Minus'
     *  UnitDelay: '<S180>/Unit Delay'
     */
    if (rtb_Switch1_eo0n_idx_1) {
        LKA_LnQualTurnOnDelay_sec[1] =
            fmaxf(LKA_LnQualTurnOnDelay_sec[1], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        LKA_LnQualTurnOnDelay_sec[1] = TJALKA_LnQualTurnOnTime_C_sec;
    }

    /* Switch: '<S178>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S154>/NOT'
     *  Logic: '<S180>/AND'
     *  RelationalOperator: '<S180>/LessThanOrEqual'
     *  Switch: '<S178>/Switch1'
     *  Switch: '<S180>/Switch'
     *  UnaryMinus: '<S180>/Unary Minus1'
     *  UnitDelay: '<S178>/Unit Delay'
     *  UnitDelay: '<S180>/Unit Delay'
     */
    LKA_LnQualRSFF_bool[1] =
        (rtb_OR_idx_1 &&
         ((rtb_Switch1_eo0n_idx_1 &&
           (LKA_LnQualTurnOnDelay_sec[1] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (LKA_LnQualRSFF_bool[1])));

    /* Abs: '<S116>/Abs' incorporates:
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport2'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(ABPR_LeLnClthPosY0_met);
    rtb_Subtract_if0w_idx_1 = fabsf(ABPR_RiLnClthPosY0_met);

    /* Sum: '<S116>/Add' incorporates:
     *  Constant: '<S116>/Constant'
     *  Constant: '<S116>/Constant1'
     */
    rtb_Add_fbcu =
        TJALKA_DistVeh2LnBndMin_C_met + TJALKA_DistVeh2LnBndHyst_C_met;

    /* Sum: '<S115>/Subtract' incorporates:
     *  Constant: '<S115>/Constant2'
     *  Constant: '<S115>/Constant3'
     */
    rtb_ABPR_LeLnClthLength_met =
        TJALKA_ValidLengthMin_C_met - TJALKA_ValidLengthMinHyst_C_met;

    /* Switch: '<S142>/Switch' incorporates:
     *  Constant: '<S116>/Constant'
     *  RelationalOperator: '<S142>/Less Than'
     *  RelationalOperator: '<S142>/Less Than1'
     *  Switch: '<S142>/Switch1'
     *  UnitDelay: '<S142>/Unit Delay'
     */
    LKA_DistVeh2LnBndHyst_bool[0] =
        ((rtb_Add_fbcu < rtb_Subtract_if0w_idx_0) ||
         ((rtb_Subtract_if0w_idx_0 >= TJALKA_DistVeh2LnBndMin_C_met) &&
          (LKA_DistVeh2LnBndHyst_bool[0])));

    /* Switch: '<S138>/Switch' incorporates:
     *  Constant: '<S115>/Constant3'
     *  Inport: '<Root>/Inport21'
     *  RelationalOperator: '<S138>/Less Than'
     *  RelationalOperator: '<S138>/Less Than1'
     *  Switch: '<S138>/Switch1'
     *  UnitDelay: '<S138>/Unit Delay'
     */
    LKA_ValidLengthHyst_bool[0] =
        ((TJALKA_ValidLengthMin_C_met < ABPR_CntrLnClthLength_met) ||
         ((ABPR_CntrLnClthLength_met >= rtb_ABPR_LeLnClthLength_met) &&
          (LKA_ValidLengthHyst_bool[0])));

    /* Switch: '<S142>/Switch' incorporates:
     *  Constant: '<S116>/Constant'
     *  RelationalOperator: '<S142>/Less Than'
     *  RelationalOperator: '<S142>/Less Than1'
     *  Switch: '<S142>/Switch1'
     *  UnitDelay: '<S142>/Unit Delay'
     */
    LKA_DistVeh2LnBndHyst_bool[1] =
        ((rtb_Add_fbcu < rtb_Subtract_if0w_idx_1) ||
         ((rtb_Subtract_if0w_idx_1 >= TJALKA_DistVeh2LnBndMin_C_met) &&
          (LKA_DistVeh2LnBndHyst_bool[1])));

    /* Switch: '<S138>/Switch' incorporates:
     *  Constant: '<S115>/Constant3'
     *  Inport: '<Root>/Inport20'
     *  RelationalOperator: '<S138>/Less Than'
     *  RelationalOperator: '<S138>/Less Than1'
     *  Switch: '<S138>/Switch1'
     *  UnitDelay: '<S138>/Unit Delay'
     */
    LKA_ValidLengthHyst_bool[1] =
        ((TJALKA_ValidLengthMin_C_met < ABPR_RiLnClthLength_met) ||
         ((ABPR_RiLnClthLength_met >= rtb_ABPR_LeLnClthLength_met) &&
          (LKA_ValidLengthHyst_bool[1])));

    /* UnitDelay: '<S1>/Unit Delay' */
    rtb_TJASTM_PrevLatCtrlMd_nu = STM_LatCtrlMdUnitDelay_bool;

    /* UnitDelay: '<S7>/Unit Delay1' */
    rtb_SLC_PrevManeuverState_nu = SLC_PrevManeuverState_Enum;

    /* Switch: '<S478>/Switch' incorporates:
     *  Constant: '<S471>/Constant'
     *  Constant: '<S475>/Constant'
     *  Constant: '<S476>/Constant'
     *  Constant: '<S478>/Constant2'
     *  Inport: '<Root>/Inport11'
     *  Logic: '<S462>/NOT2'
     *  Logic: '<S462>/OR1'
     *  Logic: '<S470>/AND'
     *  Logic: '<S470>/NOT'
     *  RelationalOperator: '<S462>/Equal7'
     *  RelationalOperator: '<S462>/Equal8'
     *  RelationalOperator: '<S462>/Equal9'
     *  UnitDelay: '<S470>/Unit Delay'
     *  UnitDelay: '<S478>/Unit Delay'
     */
    if (((((uint32_T)SLC_PrevManeuverState_Enum) !=
          E_TJASLC_ManeuverState_nu_LCPSTART) &&
         (((uint32_T)SLC_PrevManeuverState_Enum) !=
          E_TJASLC_ManeuverState_nu_LATMVSTART)) &&
        (((uint32_T)SLC_PrevManeuverState_Enum) !=
         E_TJASLC_ManeuverState_nu_LCMSTART)) {
        SLC_LaneChangeRSFF_bool = false;
    } else {
        SLC_LaneChangeRSFF_bool = ((ABPR_LaneChangeDetected_bool &&
                                    (!SLC_LaneChangeEdgeRising_bool)) ||
                                   (SLC_LaneChangeRSFF_bool));
    }

    /* End of Switch: '<S478>/Switch' */

    /* Sum: '<S270>/Add' incorporates:
     *  Constant: '<S270>/Constant1'
     *  Constant: '<S270>/Constant2'
     */
    rtb_ABPR_LeLnClthLength_met =
        TJASLC_AdjLaneWidthMax_C_met + TJALKA_LaneWidthHyst_C_met;

    /* Sum: '<S270>/Subtract1' incorporates:
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport40'
     */
    rtb_x_yaw = ABPR_LeAdjLnClthPosY0_met - ABPR_LeLnClthPosY0_met;

    /* Sum: '<S270>/Subtract2' incorporates:
     *  Inport: '<Root>/Inport2'
     *  Inport: '<Root>/Inport41'
     */
    rtb_PosY0_met_dafa = ABPR_RiLnClthPosY0_met - ABPR_RiAdjLnClthPosY0_met;

    /* RelationalOperator: '<S271>/Less Than' */
    rtb_AND_ihx5 = (rtb_ABPR_LeLnClthLength_met < rtb_x_yaw);
    rtb_AND_laqq = (rtb_ABPR_LeLnClthLength_met < rtb_PosY0_met_dafa);

    /* Sum: '<S270>/Subtract' incorporates:
     *  Constant: '<S270>/Constant3'
     *  Constant: '<S270>/Constant4'
     */
    rtb_ABPR_LeLnClthLength_met =
        TJASLC_AdjLaneWidthMin_C_met - TJALKA_LaneWidthHyst_C_met;

    /* Switch: '<S271>/Switch' incorporates:
     *  Constant: '<S270>/Constant1'
     *  RelationalOperator: '<S271>/Less Than1'
     *  Switch: '<S271>/Switch1'
     *  UnitDelay: '<S271>/Unit Delay'
     */
    SLC_AdjLnWidthMaxHyst_bool[0] =
        (rtb_AND_ihx5 || ((rtb_x_yaw >= TJASLC_AdjLaneWidthMax_C_met) &&
                          (SLC_AdjLnWidthMaxHyst_bool[0])));

    /* Switch: '<S272>/Switch' incorporates:
     *  Constant: '<S270>/Constant3'
     *  RelationalOperator: '<S272>/Less Than'
     *  RelationalOperator: '<S272>/Less Than1'
     *  Switch: '<S272>/Switch1'
     *  UnitDelay: '<S272>/Unit Delay'
     */
    SLC_AdjLnWidthMinHyst_bool[0] =
        ((TJASLC_AdjLaneWidthMin_C_met < rtb_x_yaw) ||
         ((rtb_x_yaw >= rtb_ABPR_LeLnClthLength_met) &&
          (SLC_AdjLnWidthMinHyst_bool[0])));

    /* Switch: '<S271>/Switch' incorporates:
     *  Constant: '<S270>/Constant1'
     *  RelationalOperator: '<S271>/Less Than1'
     *  Switch: '<S271>/Switch1'
     *  UnitDelay: '<S271>/Unit Delay'
     */
    SLC_AdjLnWidthMaxHyst_bool[1] =
        (rtb_AND_laqq ||
         ((rtb_PosY0_met_dafa >= TJASLC_AdjLaneWidthMax_C_met) &&
          (SLC_AdjLnWidthMaxHyst_bool[1])));

    /* Switch: '<S272>/Switch' incorporates:
     *  Constant: '<S270>/Constant3'
     *  RelationalOperator: '<S272>/Less Than'
     *  RelationalOperator: '<S272>/Less Than1'
     *  Switch: '<S272>/Switch1'
     *  UnitDelay: '<S272>/Unit Delay'
     */
    SLC_AdjLnWidthMinHyst_bool[1] =
        ((TJASLC_AdjLaneWidthMin_C_met < rtb_PosY0_met_dafa) ||
         ((rtb_PosY0_met_dafa >= rtb_ABPR_LeLnClthLength_met) &&
          (SLC_AdjLnWidthMinHyst_bool[1])));

    /* Switch: '<S270>/Switch' incorporates:
     *  Constant: '<S270>/Constant5'
     *  Logic: '<S270>/AND'
     *  Logic: '<S270>/NOT'
     *  UnitDelay: '<S271>/Unit Delay'
     *  UnitDelay: '<S272>/Unit Delay'
     */
    rtb_OR_idx_1 = !TJASLC_CheckAdjLanes_C_bool;
    rtb_SLC_LeAdjLaneWidthValid_boo =
        (rtb_OR_idx_1 ||
         ((!SLC_AdjLnWidthMaxHyst_bool[0]) && (SLC_AdjLnWidthMinHyst_bool[0])));

    /* Logic: '<S378>/OR4' incorporates:
     *  UnitDelay: '<S1>/Unit Delay2'
     */
    TJASLC_TakeOverValid_bool = SLC_TakeOverValidUnitDelay_bool;

    /* Logic: '<S103>/AND' incorporates:
     *  Constant: '<S103>/Constant'
     */
    rtb_LKA_TakeOverValid_bool =
        (TJASLC_TakeOverValid_bool && (TJALKA_TransLnChecksOff_C_bool));

    /* Logic: '<S152>/OR' incorporates:
     *  UnitDelay: '<S173>/Unit Delay'
     */
    LKA_LeLnCrvQualityValid_bool =
        ((LKA_CrvQualRSFF_bool[0]) || rtb_LKA_TakeOverValid_bool);

    /* Logic: '<S102>/NOT' */
    rtb_VectorConcatenate_gygi[0] = !LKA_LeLnCrvQualityValid_bool;

    /* Logic: '<S152>/OR1' incorporates:
     *  UnitDelay: '<S173>/Unit Delay'
     */
    rtb_LKA_Dist2BndsValid_bool =
        ((LKA_CrvQualRSFF_bool[1]) || rtb_LKA_TakeOverValid_bool);

    /* Logic: '<S102>/NOT1' */
    rtb_VectorConcatenate_gygi[1] = !rtb_LKA_Dist2BndsValid_bool;

    /* Logic: '<S154>/OR' incorporates:
     *  UnitDelay: '<S178>/Unit Delay'
     */
    LKA_LeLnQualityValid_bool =
        ((LKA_LnQualRSFF_bool[0]) || rtb_LKA_TakeOverValid_bool);

    /* Logic: '<S102>/NOT2' */
    rtb_VectorConcatenate_gygi[2] = !LKA_LeLnQualityValid_bool;

    /* Logic: '<S154>/OR1' incorporates:
     *  UnitDelay: '<S178>/Unit Delay'
     */
    rtb_AND_cvae = ((LKA_LnQualRSFF_bool[1]) || rtb_LKA_TakeOverValid_bool);

    /* Logic: '<S102>/NOT3' */
    rtb_VectorConcatenate_gygi[3] = !rtb_AND_cvae;

    /* Logic: '<S176>/AND' incorporates:
     *  Constant: '<S153>/Constant5'
     *  Logic: '<S176>/NOT'
     *  UnitDelay: '<S176>/Unit Delay'
     */
    rtb_OR_d1pw =
        ((TJALKA_InjectLaneError_C_bool) && (!LKA_LnQualifierEdgeRising_bool));

    /* Switch: '<S177>/Switch' incorporates:
     *  Constant: '<S153>/Constant6'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S177>/Max'
     *  Sum: '<S177>/Subtract'
     *  Switch: '<S177>/Switch1'
     *  UnaryMinus: '<S177>/Unary Minus'
     *  UnitDelay: '<S177>/Unit Delay'
     */
    if (rtb_OR_d1pw) {
        LKA_LnQualifierTurnOffDelay_sec = TJALKA_LaneInvalidTime_C_sec;
    } else {
        LKA_LnQualifierTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  LKA_LnQualifierTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S177>/Switch' */

    /* Switch: '<S153>/Switch' incorporates:
     *  Constant: '<S153>/Constant3'
     *  Constant: '<S153>/Constant4'
     *  Constant: '<S153>/Constant7'
     *  Constant: '<S153>/Constant8'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport6'
     *  Inport: '<Root>/Inport7'
     *  Logic: '<S153>/NOT'
     *  Logic: '<S177>/OR'
     *  RelationalOperator: '<S153>/Equal'
     *  RelationalOperator: '<S177>/GreaterThan'
     *  S-Function (sfix_bitop): '<S153>/Bitwise AND2'
     *  UnaryMinus: '<S177>/Unary Minus1'
     *  UnitDelay: '<S177>/Unit Delay'
     */
    if ((!rtb_OR_d1pw) &&
        (LKA_LnQualifierTurnOffDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec))) {
        rtb_Switch1_eo0n_idx_0 = ((((int32_T)ABPR_LeLnInvalidQu_btf) &
                                   ((int32_T)TJALKA_LaneValid_C_btm)) == 0);
        rtb_Switch1_eo0n_idx_1 = ((((int32_T)ABPR_RiLnInvalidQu_btf) &
                                   ((int32_T)TJALKA_LaneValid_C_btm)) == 0);
    } else {
        rtb_Switch1_eo0n_idx_0 = false;
        rtb_Switch1_eo0n_idx_1 = false;
    }

    /* End of Switch: '<S153>/Switch' */

    /* Logic: '<S102>/NOT4' */
    rtb_VectorConcatenate_gygi[4] = !rtb_Switch1_eo0n_idx_0;

    /* Logic: '<S102>/NOT5' */
    rtb_VectorConcatenate_gygi[5] = !rtb_Switch1_eo0n_idx_1;

    /* RelationalOperator: '<S153>/NotEqual' incorporates:
     *  Constant: '<S153>/Constant1'
     *  Constant: '<S153>/Constant2'
     *  Inport: '<Root>/Inport6'
     *  Inport: '<Root>/Inport7'
     *  S-Function (sfix_bitop): '<S153>/Bitwise AND1'
     */
    rtb_Equal_i3l2_idx_0 = (0 != (((int32_T)TJALKA_LaneVirtOrBridged_C_btm) &
                                  ((int32_T)ABPR_LeLnInvalidQu_btf)));
    rtb_Equal_i3l2_idx_1 = (0 != (((int32_T)TJALKA_LaneVirtOrBridged_C_btm) &
                                  ((int32_T)ABPR_RiLnInvalidQu_btf)));

    /* SignalConversion: '<S102>/Signal Conversion4' */
    rtb_VectorConcatenate_gygi[6] = rtb_Equal_i3l2_idx_0;

    /* SignalConversion: '<S102>/Signal Conversion1' */
    rtb_VectorConcatenate_gygi[7] = rtb_Equal_i3l2_idx_1;

    /* Logic: '<S102>/OR' incorporates:
     *  Logic: '<S102>/AND'
     */
    rtb_OR_d1pw =
        ((((LKA_LeLnCrvQualityValid_bool) && (LKA_LeLnQualityValid_bool)) &&
          rtb_Switch1_eo0n_idx_0) ||
         rtb_Equal_i3l2_idx_0);

    /* Switch: '<S156>/Switch' incorporates:
     *  Constant: '<S102>/V_Parameter3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S156>/Max'
     *  Sum: '<S156>/Subtract'
     *  Switch: '<S156>/Switch1'
     *  UnaryMinus: '<S156>/Unary Minus'
     *  UnitDelay: '<S156>/Unit Delay'
     */
    if (rtb_OR_d1pw) {
        TJALKA_LeftLlineValidRD_Sec =
            fmaxf(TJALKA_LeftLlineValidRD_Sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        TJALKA_LeftLlineValidRD_Sec = TJALKA_LaneValidMaxTime_sec;
    }

    /* End of Switch: '<S156>/Switch' */

    /* Logic: '<S156>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S156>/LessThanOrEqual'
     *  UnaryMinus: '<S156>/Unary Minus1'
     *  UnitDelay: '<S156>/Unit Delay'
     */
    rtb_AND_ihx5 = (rtb_OR_d1pw && (TJALKA_LeftLlineValidRD_Sec <=
                                    (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S102>/AND1' */
    rtb_LKA_Dist2BndsValid_bool =
        ((rtb_LKA_Dist2BndsValid_bool && rtb_AND_cvae) &&
         rtb_Switch1_eo0n_idx_1);

    /* Logic: '<S102>/OR1' */
    rtb_LKA_Dist2BndsValid_bool =
        (rtb_LKA_Dist2BndsValid_bool || rtb_Equal_i3l2_idx_1);

    /* Switch: '<S157>/Switch' incorporates:
     *  Constant: '<S102>/V_Parameter1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S157>/Max'
     *  Sum: '<S157>/Subtract'
     *  Switch: '<S157>/Switch1'
     *  UnaryMinus: '<S157>/Unary Minus'
     *  UnitDelay: '<S157>/Unit Delay'
     */
    if (rtb_LKA_Dist2BndsValid_bool) {
        TJALKA_RightLlineValidRD_Sec =
            fmaxf(TJALKA_RightLlineValidRD_Sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        TJALKA_RightLlineValidRD_Sec = TJALKA_LaneValidMaxTime_sec;
    }

    /* End of Switch: '<S157>/Switch' */

    /* Logic: '<S157>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S157>/LessThanOrEqual'
     *  UnaryMinus: '<S157>/Unary Minus1'
     *  UnitDelay: '<S157>/Unit Delay'
     */
    rtb_AND_laqq =
        (rtb_LKA_Dist2BndsValid_bool &&
         (TJALKA_RightLlineValidRD_Sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S151>/Switch' incorporates:
     *  Logic: '<S151>/AND'
     *  Switch: '<S151>/Switch1'
     *  Switch: '<S151>/Switch2'
     */
    if (rtb_AND_ihx5 && rtb_AND_laqq) {
        /* RelationalOperator: '<S150>/Equal4' incorporates:
         *  Constant: '<S169>/Constant'
         *  Switch: '<S151>/Switch'
         */
        TJALKA_LnBndValid_nu = E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE;
    } else if (rtb_AND_ihx5) {
        /* RelationalOperator: '<S150>/Equal4' incorporates:
         *  Constant: '<S170>/Constant'
         *  Switch: '<S151>/Switch'
         *  Switch: '<S151>/Switch1'
         */
        TJALKA_LnBndValid_nu = E_TJALKA_LnBndValid_nu_BND_VAL_LEFT_ONLY;
    } else if (rtb_AND_laqq) {
        /* RelationalOperator: '<S150>/Equal4' incorporates:
         *  Constant: '<S171>/Constant'
         *  Switch: '<S151>/Switch'
         *  Switch: '<S151>/Switch1'
         *  Switch: '<S151>/Switch2'
         */
        TJALKA_LnBndValid_nu = E_TJALKA_LnBndValid_nu_BND_VAL_RIGHT_ONLY;
    } else {
        /* RelationalOperator: '<S150>/Equal4' incorporates:
         *  Constant: '<S172>/Constant'
         *  Switch: '<S151>/Switch'
         *  Switch: '<S151>/Switch1'
         *  Switch: '<S151>/Switch2'
         */
        TJALKA_LnBndValid_nu = E_TJALKA_LnBndValid_nu_BND_NOT_VALID;
    }

    /* End of Switch: '<S151>/Switch' */

    /* RelationalOperator: '<S150>/Equal4' incorporates:
     *  Constant: '<S161>/Constant'
     */
    rtb_OR_d1pw = (E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE ==
                   ((uint32_T)TJALKA_LnBndValid_nu));

    /* RelationalOperator: '<S150>/Equal' incorporates:
     *  Constant: '<S159>/Constant'
     */
    rtb_AND_cvae = (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC);

    /* Logic: '<S150>/AND1' incorporates:
     *  Constant: '<S160>/Constant'
     *  Constant: '<S162>/Constant'
     *  Logic: '<S150>/OR'
     *  RelationalOperator: '<S150>/Equal1'
     *  RelationalOperator: '<S150>/Equal2'
     *  UnitDelay: '<S150>/Unit Delay'
     */
    rtb_LKA_Dist2BndsValid_bool =
        ((rtb_AND_cvae || (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                           E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) &&
         (E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE ==
          ((uint32_T)LKA_PrevLnBndValid_Enum)));

    /* Switch: '<S166>/Switch' incorporates:
     *  Constant: '<S150>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S166>/Max'
     *  Sum: '<S166>/Subtract'
     *  Switch: '<S166>/Switch1'
     *  UnaryMinus: '<S166>/Unary Minus'
     *  UnitDelay: '<S166>/Unit Delay'
     */
    if (rtb_LKA_Dist2BndsValid_bool) {
        LKA_LnBndValidTurnOnDelay_sec =
            fmaxf(LKA_LnBndValidTurnOnDelay_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        LKA_LnBndValidTurnOnDelay_sec = TJALKA_LnPredMinValidTime_C_sec;
    }

    /* End of Switch: '<S166>/Switch' */

    /* Abs: '<S150>/Abs' incorporates:
     *  Abs: '<S114>/Abs1'
     *  Inport: '<Root>/Inport9'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(ABPR_CntrLnClthCrv_1pm);

    /* Logic: '<S150>/AND2' incorporates:
     *  Abs: '<S150>/Abs'
     *  Constant: '<S150>/Constant'
     *  Constant: '<S150>/Constant1'
     *  Constant: '<S163>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S150>/AND'
     *  Logic: '<S150>/NOT'
     *  Logic: '<S150>/NOT1'
     *  Logic: '<S166>/AND'
     *  RelationalOperator: '<S150>/Equal3'
     *  RelationalOperator: '<S150>/Less Than'
     *  RelationalOperator: '<S166>/LessThanOrEqual'
     *  UnaryMinus: '<S166>/Unary Minus1'
     *  UnitDelay: '<S166>/Unit Delay'
     */
    rtb_AND2 =
        (((((((TJALKA_BothSideBrdgEnable_C_bool) && (!rtb_Equal_i3l2_idx_0)) &&
             (!rtb_Equal_i3l2_idx_1)) &&
            (rtb_Subtract_if0w_idx_0 < TJALKA_LnPredictionCrvMax_C_1pm)) &&
           rtb_AND_cvae) &&
          (rtb_LKA_Dist2BndsValid_bool && (LKA_LnBndValidTurnOnDelay_sec <=
                                           (-LCFRCV_TSysCycleTimeSen_sec)))) &&
         (E_TJALKA_LnBndValid_nu_BND_NOT_VALID ==
          ((uint32_T)TJALKA_LnBndValid_nu)));

    /* Switch: '<S168>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S168>/Max'
     *  Sum: '<S168>/Subtract'
     *  Switch: '<S168>/Switch1'
     *  UnaryMinus: '<S168>/Unary Minus'
     *  UnitDelay: '<S168>/Unit Delay'
     */
    if (rtb_AND2) {
        /* Switch: '<S164>/Switch' incorporates:
         *  Constant: '<S164>/Constant'
         *  Constant: '<S164>/Constant1'
         *  Constant: '<S164>/Constant2'
         *  UnitDelay: '<S168>/Unit Delay'
         */
        if (TJALKA_RampoutPredictOn_C_bool) {
            LKA_PredTimeExceededTurnOffDelay_sec =
                TJALKA_LnPredictionTimeMin_C_sec;
        } else {
            LKA_PredTimeExceededTurnOffDelay_sec =
                TJALKA_LnPredictionTimeMax_C_sec;
        }

        /* End of Switch: '<S164>/Switch' */
    } else {
        LKA_PredTimeExceededTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  LKA_PredTimeExceededTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S168>/Switch' */

    /* Logic: '<S168>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S168>/GreaterThan'
     *  UnaryMinus: '<S168>/Unary Minus1'
     *  UnitDelay: '<S168>/Unit Delay'
     */
    rtb_AND_ihx5 = (rtb_AND2 || (LKA_PredTimeExceededTurnOffDelay_sec >
                                 (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S182>/AND1' incorporates:
     *  UnitDelay: '<S1>/Unit Delay3'
     */
    TJAOBF_StrongReady_bool = OBF_StrongReadyUnitDelay_bool;

    /* Logic: '<S182>/AND2' incorporates:
     *  UnitDelay: '<S1>/Unit Delay4'
     */
    TJAOBF_WeakReady_bool = OBF_WeakReadyUnitDelay_bool;

    /* Logic: '<S182>/AND3' incorporates:
     *  UnitDelay: '<S1>/Unit Delay5'
     */
    TJAOBF_Cancel_bool = OBF_CancelUnitDelay_bool;

    /* Logic: '<S150>/AND3' incorporates:
     *  Logic: '<S150>/NOT3'
     */
    rtb_AND_laqq = ((TJAOBF_StrongReady_bool && TJAOBF_WeakReady_bool) &&
                    (!TJAOBF_Cancel_bool));

    /* Switch: '<S165>/Switch' incorporates:
     *  Logic: '<S150>/NOT2'
     *  Logic: '<S150>/OR1'
     *  Logic: '<S158>/AND'
     *  Logic: '<S158>/NOT'
     *  Logic: '<S167>/AND'
     *  Logic: '<S167>/NOT'
     *  UnitDelay: '<S158>/Unit Delay'
     *  UnitDelay: '<S167>/Unit Delay'
     */
    if (((((!rtb_AND_ihx5) && (LKA_PredTimeExceededEdgeFalling_bool)) ||
          (!rtb_AND_cvae)) ||
         rtb_OR_d1pw) ||
        (rtb_AND_laqq && (!LKA_OBFValidEdgeRising_bool))) {
        /* Switch: '<S165>/Switch' incorporates:
         *  Constant: '<S165>/Constant2'
         */
        TJALKA_LanePredictValid_bool = false;
    } else {
        /* Switch: '<S165>/Switch' incorporates:
         *  Switch: '<S165>/Switch1'
         *  UnitDelay: '<S165>/Unit Delay'
         */
        TJALKA_LanePredictValid_bool =
            (rtb_AND2 || (LKA_LanePredictValidRSFF_bool));
    }

    /* End of Switch: '<S165>/Switch' */

    /* Logic: '<S101>/OR' */
    rtb_OR_d1pw = (rtb_OR_d1pw || TJALKA_LanePredictValid_bool);

    /* Logic: '<S101>/NOT4' */
    rtb_VectorConcatenate_maf5[0] = !rtb_OR_d1pw;

    /* Sum: '<S119>/Add' incorporates:
     *  Constant: '<S119>/Constant'
     *  Constant: '<S119>/Constant1'
     *  Sum: '<S120>/Add'
     */
    rtb_Switch_ithu = TJALKA_VelXMax_C_kph + TJALKA_VelXHyst_C_kph;

    /* Switch: '<S146>/Switch' incorporates:
     *  Constant: '<S119>/Constant'
     *  Constant: '<S146>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S146>/Less Than'
     *  RelationalOperator: '<S146>/Less Than1'
     *  Sum: '<S119>/Add'
     *  UnitDelay: '<S146>/Unit Delay'
     */
    if (rtb_Switch_ithu < VDy_DashboardVelocity_kph) {
        LKA_VelXMaxHyst_bool = true;
    } else {
        LKA_VelXMaxHyst_bool =
            ((VDy_DashboardVelocity_kph >= TJALKA_VelXMax_C_kph) &&
             (LKA_VelXMaxHyst_bool));
    }

    /* End of Switch: '<S146>/Switch' */

    /* Switch: '<S147>/Switch' incorporates:
     *  Constant: '<S119>/Constant2'
     *  Constant: '<S119>/Constant3'
     *  Constant: '<S147>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S147>/Less Than'
     *  RelationalOperator: '<S147>/Less Than1'
     *  Sum: '<S119>/Subtract'
     *  UnitDelay: '<S147>/Unit Delay'
     */
    if (TJALKA_VelXMin_C_kph < VDy_DashboardVelocity_kph) {
        LKA_VelXMinHyst_bool = true;
    } else {
        LKA_VelXMinHyst_bool =
            ((VDy_DashboardVelocity_kph >=
              (TJALKA_VelXMin_C_kph - TJALKA_VelXHyst_C_kph)) &&
             (LKA_VelXMinHyst_bool));
    }

    /* End of Switch: '<S147>/Switch' */

    /* Logic: '<S119>/AND' incorporates:
     *  Logic: '<S119>/NOT'
     *  UnitDelay: '<S146>/Unit Delay'
     *  UnitDelay: '<S147>/Unit Delay'
     */
    rtb_AND_cvae = ((!LKA_VelXMaxHyst_bool) && (LKA_VelXMinHyst_bool));

    /* Logic: '<S101>/NOT3' */
    rtb_VectorConcatenate_maf5[1] = !rtb_AND_cvae;

    /* Logic: '<S116>/OR' incorporates:
     *  Constant: '<S116>/Constant2'
     *  Constant: '<S139>/Constant'
     *  Constant: '<S140>/Constant'
     *  Constant: '<S141>/Constant'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport2'
     *  Logic: '<S116>/AND'
     *  Logic: '<S116>/AND1'
     *  Logic: '<S116>/OR2'
     *  RelationalOperator: '<S116>/Equal4'
     *  RelationalOperator: '<S116>/Equal5'
     *  RelationalOperator: '<S116>/Equal6'
     *  RelationalOperator: '<S116>/GreaterThan'
     *  RelationalOperator: '<S116>/Less Than'
     *  UnitDelay: '<S142>/Unit Delay'
     */
    rtb_LKA_Dist2BndsValid_bool = (((((((LKA_DistVeh2LnBndHyst_bool[0]) &&
                                        (LKA_DistVeh2LnBndHyst_bool[1])) &&
                                       (ABPR_LeLnClthPosY0_met > 0.0F)) &&
                                      (ABPR_RiLnClthPosY0_met < 0.0F)) ||
                                     rtb_LKA_TakeOverValid_bool) ||
                                    TJALKA_LanePredictValid_bool) ||
                                   (((((uint32_T)SLC_PrevManeuverState_Enum) ==
                                      E_TJASLC_ManeuverState_nu_LATMVSTART) ||
                                     (((uint32_T)SLC_PrevManeuverState_Enum) ==
                                      E_TJASLC_ManeuverState_nu_LCMSTART)) ||
                                    (((uint32_T)SLC_PrevManeuverState_Enum) ==
                                     E_TJASLC_ManeuverState_nu_NEWEGO)));

    /* Logic: '<S101>/NOT6' */
    rtb_VectorConcatenate_maf5[2] = !rtb_LKA_Dist2BndsValid_bool;

    /* Lookup_n-D: '<S112>/1-D Lookup Table' incorporates:
     *  Abs: '<S150>/Abs'
     */
    rtb_Subtract_if0w_idx_1 =
        TJALKA_CrvLaneWidthMaxThd_Cr_met[plook_u32f_bincka(
            rtb_Subtract_if0w_idx_0,
            ((const real32_T *)&(TJALKA_CntrCrv_Bx_1pm[0])), 10U)];

    /* Switch: '<S134>/Switch' incorporates:
     *  Constant: '<S112>/Constant1'
     *  Constant: '<S134>/Constant'
     *  Inport: '<Root>/Inport5'
     *  RelationalOperator: '<S134>/Less Than'
     *  RelationalOperator: '<S134>/Less Than1'
     *  Sum: '<S112>/Add'
     *  UnitDelay: '<S134>/Unit Delay'
     */
    if ((rtb_Subtract_if0w_idx_1 + TJALKA_LaneWidthHyst_C_met) <
        ABPR_LaneWidth_met) {
        LKA_LaneWidthMaxHyst_bool = true;
    } else {
        LKA_LaneWidthMaxHyst_bool =
            ((ABPR_LaneWidth_met >= rtb_Subtract_if0w_idx_1) &&
             (LKA_LaneWidthMaxHyst_bool));
    }

    /* End of Switch: '<S134>/Switch' */

    /* Switch: '<S112>/Switch' incorporates:
     *  Constant: '<S112>/Constant4'
     *  Inport: '<Root>/Inport22'
     *  Inport: '<Root>/Inport5'
     */
    if (TJALKA_UseUncoupLaneWidth_C_bool) {
        rtb_Subtract_if0w_idx_1 = ABPR_UncoupledLaneWidth_met;
    } else {
        rtb_Subtract_if0w_idx_1 = ABPR_LaneWidth_met;
    }

    /* End of Switch: '<S112>/Switch' */

    /* Switch: '<S135>/Switch' incorporates:
     *  Constant: '<S112>/Constant2'
     *  Constant: '<S112>/Constant3'
     *  Constant: '<S135>/Constant'
     *  RelationalOperator: '<S135>/Less Than'
     *  RelationalOperator: '<S135>/Less Than1'
     *  Sum: '<S112>/Subtract'
     *  UnitDelay: '<S135>/Unit Delay'
     */
    if (TJALKA_LaneWidthMin_C_met < rtb_Subtract_if0w_idx_1) {
        LKA_LaneWidthMinHyst_bool = true;
    } else {
        LKA_LaneWidthMinHyst_bool =
            ((rtb_Subtract_if0w_idx_1 >=
              (TJALKA_LaneWidthMin_C_met - TJALKA_LaneWidthHyst_C_met)) &&
             (LKA_LaneWidthMinHyst_bool));
    }

    /* End of Switch: '<S135>/Switch' */

    /* Logic: '<S112>/OR' incorporates:
     *  Logic: '<S112>/AND'
     *  Logic: '<S112>/NOT'
     *  UnitDelay: '<S134>/Unit Delay'
     *  UnitDelay: '<S135>/Unit Delay'
     */
    rtb_LKA_LaneWidthValid_bool =
        (((!LKA_LaneWidthMaxHyst_bool) && (LKA_LaneWidthMinHyst_bool)) ||
         TJALKA_LanePredictValid_bool);

    /* Logic: '<S101>/NOT5' */
    rtb_VectorConcatenate_maf5[3] = !rtb_LKA_LaneWidthValid_bool;

    /* Switch: '<S114>/Switch' incorporates:
     *  Constant: '<S114>/Constant3'
     *  Inport: '<Root>/Inport9'
     *  RelationalOperator: '<S114>/GreaterThanOrEqual'
     */
    if (rtb_Subtract_if0w_idx_0 >= 1.0E-8F) {
        rtb_Subtract_if0w_idx_0 = ABPR_CntrLnClthCrv_1pm;
    } else {
        rtb_Subtract_if0w_idx_0 = 1.0E-8F;
    }

    /* End of Switch: '<S114>/Switch' */

    /* Abs: '<S114>/Abs' incorporates:
     *  Constant: '<S114>/Constant2'
     *  Product: '<S114>/Divide'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(1.0F / rtb_Subtract_if0w_idx_0);

    /* Switch: '<S137>/Switch' incorporates:
     *  Constant: '<S114>/Constant'
     *  Constant: '<S114>/Constant1'
     *  Constant: '<S137>/Constant'
     *  RelationalOperator: '<S137>/Less Than'
     *  RelationalOperator: '<S137>/Less Than1'
     *  Sum: '<S114>/Add'
     *  UnitDelay: '<S137>/Unit Delay'
     */
    if ((TJALKA_RadiusMin_C_met + TJALKA_RadiusHyst_C_met) <
        rtb_Subtract_if0w_idx_0) {
        LKA_RadiusHyst_bool = true;
    } else {
        LKA_RadiusHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >= TJALKA_RadiusMin_C_met) &&
             (LKA_RadiusHyst_bool));
    }

    /* End of Switch: '<S137>/Switch' */

    /* Logic: '<S114>/OR' incorporates:
     *  UnitDelay: '<S137>/Unit Delay'
     */
    rtb_LKA_RadiusValid_bool =
        ((LKA_RadiusHyst_bool) || TJALKA_LanePredictValid_bool);

    /* Logic: '<S101>/NOT8' */
    rtb_VectorConcatenate_maf5[4] = !rtb_LKA_RadiusValid_bool;

    /* Switch: '<S118>/Switch' incorporates:
     *  Constant: '<S118>/Constant'
     */
    if (TJASLC_SALC_Enabled_C_bool) {
        /* Logic: '<S118>/NOT2' incorporates:
         *  Constant: '<S145>/Constant'
         *  Inport: '<Root>/Inport19'
         *  Logic: '<S118>/AND1'
         *  Logic: '<S118>/NOT1'
         *  Logic: '<S118>/OR'
         *  Logic: '<S144>/AND'
         *  RelationalOperator: '<S118>/Equal'
         *  UnitDelay: '<S144>/Unit Delay'
         */
        rtb_AND_pjo1 = (((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
                          ((uint32_T)STM_LatCtrlMdUnitDelay_bool)) ||
                         rtb_LKA_TakeOverValid_bool) ||
                        (LCFRCV_TurnSignalLeverHold_bool ||
                         (!LKA_TurnSignalLevelHoldEdgeFalling_bool)));
    } else {
        /* Logic: '<S118>/NOT2' incorporates:
         *  Constant: '<S118>/Constant1'
         *  Constant: '<S118>/Constant2'
         *  Inport: '<Root>/Inport74'
         *  RelationalOperator: '<S118>/NotEqual'
         *  S-Function (sfix_bitop): '<S118>/Bitwise AND'
         */
        rtb_AND_pjo1 = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                         ((int32_T)TJALKA_DrvStInvalidSR_C_btm)) == 0);
    }

    /* End of Switch: '<S118>/Switch' */

    /* Logic: '<S101>/NOT9' incorporates:
     *  Logic: '<S101>/NOT2'
     */
    rtb_CMB_OFOObjLengthInvalid_boo = !rtb_AND_pjo1;
    rtb_VectorConcatenate_maf5[5] = rtb_CMB_OFOObjLengthInvalid_boo;

    /* Logic: '<S106>/AND' incorporates:
     *  Inport: '<Root>/Inport13'
     *  Logic: '<S187>/AND'
     */
    rtb_AND2 = !ABPR_ConstructionSite_bool;

    /* Logic: '<S106>/NOT' incorporates:
     *  Constant: '<S106>/Constant'
     *  Logic: '<S106>/AND'
     */
    rtb_AND_gpqq = (rtb_AND2 || (!TJALKA_ConstSiteCheckOn_C_bool));

    /* Logic: '<S101>/NOT7' */
    rtb_VectorConcatenate_maf5[6] = !rtb_AND_gpqq;

    /* Logic: '<S101>/NOT11' incorporates:
     *  Constant: '<S101>/Constant1'
     */
    rtb_VectorConcatenate_maf5[7] = !TJALKA_LC_Enabled_C_bool;

    /* RelationalOperator: '<S107>/Equal' incorporates:
     *  Constant: '<S107>/Constant1'
     *  Constant: '<S107>/Constant4'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S107>/Bitwise AND'
     */
    rtb_GEN_VehStInvalid_bool = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                                  ((int32_T)TJALKA_PrjSpecQuSR_C_btm)) == 0);

    /* Logic: '<S101>/NOT10' */
    rtb_VectorConcatenate_maf5[8] = !rtb_GEN_VehStInvalid_bool;

    /* SignalConversion generated from: '<S113>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[9] = false;

    /* Logic: '<S117>/AND' incorporates:
     *  Inport: '<Root>/Inport11'
     *  Logic: '<S111>/NOT'
     *  Logic: '<S368>/OR'
     */
    rtb_AND_jt3t = !ABPR_LaneChangeDetected_bool;

    /* Logic: '<S117>/AND1' incorporates:
     *  Constant: '<S143>/Constant'
     *  Logic: '<S117>/AND'
     *  Logic: '<S117>/NOT'
     *  RelationalOperator: '<S117>/NotEqual'
     */
    rtb_LKA_NoLaneChange_bool =
        ((rtb_AND_jt3t || (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
                           ((uint32_T)STM_LatCtrlMdUnitDelay_bool))) ||
         rtb_LKA_TakeOverValid_bool);

    /* Logic: '<S101>/NOT17' incorporates:
     *  Logic: '<S101>/NOT1'
     */
    rtb_CMB_OFOVelocityInvalid_bool = !rtb_LKA_NoLaneChange_bool;
    rtb_VectorConcatenate_maf5[10] = rtb_CMB_OFOVelocityInvalid_bool;

    /* Logic: '<S115>/OR' incorporates:
     *  Logic: '<S115>/AND'
     *  UnitDelay: '<S138>/Unit Delay'
     */
    rtb_LKA_LengthValid_bool =
        (((LKA_ValidLengthHyst_bool[0]) && (LKA_ValidLengthHyst_bool[1])) ||
         TJALKA_LanePredictValid_bool);

    /* Logic: '<S101>/NOT16' */
    rtb_VectorConcatenate_maf5[11] = !rtb_LKA_LengthValid_bool;

    /* Signum: '<S110>/Sign' incorporates:
     *  Inport: '<Root>/Inport3'
     */
    if (ABPR_CntrLnClthPosY0_met < 0.0F) {
        rtb_Subtract_if0w_idx_0 = -1.0F;
    } else if (ABPR_CntrLnClthPosY0_met > 0.0F) {
        rtb_Subtract_if0w_idx_0 = 1.0F;
    } else {
        rtb_Subtract_if0w_idx_0 = ABPR_CntrLnClthPosY0_met;
    }

    /* End of Signum: '<S110>/Sign' */

    /* Signum: '<S110>/Sign1' incorporates:
     *  Inport: '<Root>/Inport4'
     */
    if (ABPR_CntrLnClthHeading_rad < 0.0F) {
        rtb_Subtract_if0w_idx_1 = -1.0F;
    } else if (ABPR_CntrLnClthHeading_rad > 0.0F) {
        rtb_Subtract_if0w_idx_1 = 1.0F;
    } else {
        rtb_Subtract_if0w_idx_1 = ABPR_CntrLnClthHeading_rad;
    }

    /* End of Signum: '<S110>/Sign1' */

    /* Logic: '<S110>/OR' incorporates:
     *  Abs: '<S110>/Abs'
     *  Constant: '<S110>/Constant1'
     *  DataTypeConversion: '<S110>/Data Type Conversion'
     *  DataTypeConversion: '<S110>/Data Type Conversion1'
     *  Inport: '<Root>/Inport4'
     *  RelationalOperator: '<S110>/Less Than'
     *  RelationalOperator: '<S110>/NotEqual'
     */
    rtb_OR2_pwtt =
        ((((int8_T)rtb_Subtract_if0w_idx_0) !=
          ((int8_T)rtb_Subtract_if0w_idx_1)) ||
         (fabsf(ABPR_CntrLnClthHeading_rad) < TJALKA_MaxHeadAngActnTJA_C_rad));

    /* Logic: '<S101>/NOT14' */
    rtb_VectorConcatenate_maf5[12] = !rtb_OR2_pwtt;

    /* RelationalOperator: '<S107>/NotEqual' incorporates:
     *  Constant: '<S107>/Constant5'
     *  Constant: '<S107>/Constant6'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S107>/Bitwise AND2'
     */
    rtb_OR_lkl2 = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                    ((int32_T)TJALKA_PrjSpecQuC_C_btm)) != 0);

    /* Switch: '<S124>/Switch' incorporates:
     *  Constant: '<S105>/Constant'
     *  Constant: '<S105>/Constant1'
     *  Constant: '<S122>/Constant'
     *  Constant: '<S123>/Constant'
     *  Constant: '<S124>/Constant2'
     *  DataTypeConversion: '<S105>/Data Type Conversion'
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     *  Logic: '<S105>/AND'
     *  Logic: '<S105>/OR'
     *  Product: '<S105>/Divide'
     *  RelationalOperator: '<S105>/Equal'
     *  RelationalOperator: '<S105>/Equal1'
     *  RelationalOperator: '<S105>/GreaterThan'
     *  Sum: '<S105>/Add'
     *  UnitDelay: '<S124>/Unit Delay'
     */
    if (((((real32_T)((uint8_T)(((uint32_T)ABPR_LeLnQuality_perc) +
                                ((uint32_T)ABPR_RiLnQuality_perc)))) /
          2.0F) > ((real32_T)TJAGEN_QuTrajPlanMinLnQual_C_perc)) &&
        ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE ==
          ((uint32_T)STM_LatCtrlMdUnitDelay_bool)) ||
         (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
          E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF))) {
        LKA_TrajPlanCancelRSFF_bool = false;
    }

    /* End of Switch: '<S124>/Switch' */

    /* Logic: '<S101>/AND1' incorporates:
     *  Constant: '<S108>/Constant'
     *  Constant: '<S109>/Constant'
     *  Logic: '<S101>/OR1'
     *  Logic: '<S101>/OR2'
     *  RelationalOperator: '<S101>/Equal'
     *  RelationalOperator: '<S101>/Equal1'
     *  UnitDelay: '<S124>/Unit Delay'
     */
    TJALKA_Cancel_bool = ((rtb_OR_lkl2 || (LKA_TrajPlanCancelRSFF_bool)) &&
                          ((((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) ||
                           (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)));

    /* Switch: '<S121>/Switch2' incorporates:
     *  Constant: '<S104>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S101>/OR3'
     *  Logic: '<S104>/OR'
     *  MinMax: '<S104>/Max'
     *  RelationalOperator: '<S121>/GreaterThan'
     *  Switch: '<S121>/Switch'
     *  UnitDelay: '<S121>/Unit Delay'
     */
    if ((rtb_CMB_OFOObjLengthInvalid_boo || rtb_CMB_OFOVelocityInvalid_bool) ||
        TJALKA_Cancel_bool) {
        LKA_BlockTimerExpiredTimerRetrigger_sec =
            fmaxf(LCFRCV_TSysCycleTimeSen_sec, TJALKA_BlockTimeTJA_C_sec);
    } else if (LKA_BlockTimerExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S121>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S121>/Subtract'
         *  UnitDelay: '<S121>/Unit Delay'
         */
        LKA_BlockTimerExpiredTimerRetrigger_sec =
            LKA_BlockTimerExpiredTimerRetrigger_sec -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S121>/Unit Delay' incorporates:
         *  Constant: '<S121>/Constant1'
         *  Switch: '<S121>/Switch'
         */
        LKA_BlockTimerExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S121>/Switch2' */

    /* Logic: '<S104>/NOT' incorporates:
     *  Constant: '<S121>/Constant2'
     *  RelationalOperator: '<S121>/GreaterThan1'
     *  UnitDelay: '<S121>/Unit Delay'
     */
    rtb_GEN_DrvNotBuckledUp_bool =
        (LKA_BlockTimerExpiredTimerRetrigger_sec <= 0.0F);

    /* Logic: '<S101>/NOT13' */
    rtb_VectorConcatenate_maf5[13] = !rtb_GEN_DrvNotBuckledUp_bool;

    /* SignalConversion: '<S101>/Signal Conversion4' */
    rtb_VectorConcatenate_maf5[14] = rtb_OR_lkl2;

    /* SignalConversion: '<S101>/Signal Conversion2' */
    rtb_VectorConcatenate_maf5[15] = TJALKA_Cancel_bool;

    /* S-Function (ex_sfun_set_bit): '<S136>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S113>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_maf5[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit);

    /* S-Function (ex_sfun_set_bit): '<S181>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S155>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_gygi[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_fv5m);

    /* Logic: '<S206>/NOT' incorporates:
     *  Constant: '<S206>/Constant4'
     *  Constant: '<S206>/Constant5'
     *  Inport: '<Root>/Inport33'
     *  RelationalOperator: '<S206>/Equal2'
     *  S-Function (sfix_bitop): '<S206>/Bitwise AND2'
     */
    OBF_AccObjSwitch = ((((int32_T)ODPFOP_AccObjInvBitfield_btf) &
                         ((int32_T)TJAOBF_AccObjChange_C_btm)) != 0);

    /* Logic: '<S213>/AND' incorporates:
     *  Logic: '<S213>/NOT'
     *  UnitDelay: '<S213>/Unit Delay'
     */
    rtb_OR_lkl2 = ((OBF_AccObjSwitch) && (!OBF_AccObjSwitchEdgeRising_bool));

    /* Switch: '<S214>/Switch' incorporates:
     *  Constant: '<S210>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S214>/Max'
     *  Sum: '<S214>/Subtract'
     *  Switch: '<S214>/Switch1'
     *  UnaryMinus: '<S214>/Unary Minus'
     *  UnitDelay: '<S214>/Unit Delay'
     */
    if (rtb_OR_lkl2) {
        OBF_AccObjSwitchTurnOffDelay_sec = TJAOBF_CutinObValidFreezTm_C_sec;
    } else {
        OBF_AccObjSwitchTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  OBF_AccObjSwitchTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S214>/Switch' */

    /* Logic: '<S214>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S214>/GreaterThan'
     *  UnaryMinus: '<S214>/Unary Minus1'
     *  UnitDelay: '<S214>/Unit Delay'
     */
    rtb_OR_lkl2 = (rtb_OR_lkl2 || (OBF_AccObjSwitchTurnOffDelay_sec >
                                   (-LCFRCV_TSysCycleTimeSen_sec)));

    /* RelationalOperator: '<S206>/Equal1' incorporates:
     *  Constant: '<S206>/Constant2'
     *  Constant: '<S206>/Constant3'
     *  Inport: '<Root>/Inport33'
     *  S-Function (sfix_bitop): '<S206>/Bitwise AND1'
     */
    OBF_AccObjValidLaneCheck =
        ((((int32_T)ODPFOP_AccObjInvBitfield_btf) &
          ((int32_T)TJAOBF_AccObjLanesInvalid_C_btm)) == 0);

    /* Switch: '<S216>/Switch' incorporates:
     *  Constant: '<S212>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S216>/Max'
     *  Sum: '<S216>/Subtract'
     *  Switch: '<S216>/Switch1'
     *  UnaryMinus: '<S216>/Unary Minus'
     *  UnitDelay: '<S216>/Unit Delay'
     */
    if (OBF_AccObjValidLaneCheck) {
        OBF_AccObjValidLaneTurnOnDelay_sec =
            fmaxf(OBF_AccObjValidLaneTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_AccObjValidLaneTurnOnDelay_sec = TJAOBF_MinDurAccObjValid_C_sec;
    }

    /* End of Switch: '<S216>/Switch' */

    /* Logic: '<S216>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S216>/LessThanOrEqual'
     *  UnaryMinus: '<S216>/Unary Minus1'
     *  UnitDelay: '<S216>/Unit Delay'
     */
    rtb_AND_ojra =
        ((OBF_AccObjValidLaneCheck) && (OBF_AccObjValidLaneTurnOnDelay_sec <=
                                        (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Sum: '<S242>/Add' incorporates:
     *  Constant: '<S242>/Constant'
     *  Constant: '<S242>/Constant4'
     */
    rtb_DataTypeConversion1 =
        (uint8_T)(((uint32_T)TJAOBF_CrvQualityMin_C_perc) +
                  ((uint32_T)TJAOBF_CrvQualityHyst_C_perc));

    /* Switch: '<S246>/Switch' incorporates:
     *  Constant: '<S242>/Constant'
     *  Inport: '<Root>/Inport17'
     *  Inport: '<Root>/Inport18'
     *  RelationalOperator: '<S246>/Less Than'
     *  RelationalOperator: '<S246>/Less Than1'
     *  Switch: '<S246>/Switch1'
     *  UnitDelay: '<S246>/Unit Delay'
     */
    OBF_LaneAttributesValidHyst_bool[0] =
        ((rtb_DataTypeConversion1 < ABPR_LeCrvQuality_perc) ||
         ((ABPR_LeCrvQuality_perc >= TJAOBF_CrvQualityMin_C_perc) &&
          (OBF_LaneAttributesValidHyst_bool[0])));
    OBF_LaneAttributesValidHyst_bool[1] =
        ((rtb_DataTypeConversion1 < ABPR_RiCrvQuality_perc) ||
         ((ABPR_RiCrvQuality_perc >= TJAOBF_CrvQualityMin_C_perc) &&
          (OBF_LaneAttributesValidHyst_bool[1])));

    /* RelationalOperator: '<S242>/Equal' incorporates:
     *  Constant: '<S243>/Constant'
     */
    rtb_GEN_BrakePadelInvalid__gzuo =
        (((uint32_T)TJALKA_LnBndValid_nu) ==
         E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE);

    /* Logic: '<S242>/OR' incorporates:
     *  Constant: '<S245>/Constant'
     *  RelationalOperator: '<S242>/Equal2'
     */
    rtb_AND_nxkg = ((E_TJALKA_LnBndValid_nu_BND_VAL_LEFT_ONLY ==
                     ((uint32_T)TJALKA_LnBndValid_nu)) ||
                    rtb_GEN_BrakePadelInvalid__gzuo);

    /* Switch: '<S247>/Switch' incorporates:
     *  Constant: '<S242>/V_Parameter1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S247>/Max'
     *  Sum: '<S247>/Subtract'
     *  Switch: '<S247>/Switch1'
     *  UnaryMinus: '<S247>/Unary Minus'
     *  UnitDelay: '<S247>/Unit Delay'
     */
    if (rtb_AND_nxkg) {
        OBF_LeftLineValidTurnOnDelay_sec =
            fmaxf(OBF_LeftLineValidTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_LeftLineValidTurnOnDelay_sec = TJAOBF_CheckLineValidMaxTime_sec;
    }

    /* End of Switch: '<S247>/Switch' */

    /* Logic: '<S247>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S247>/LessThanOrEqual'
     *  UnaryMinus: '<S247>/Unary Minus1'
     *  UnitDelay: '<S247>/Unit Delay'
     */
    rtb_AND_nxkg = (rtb_AND_nxkg && (OBF_LeftLineValidTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S242>/AND2' incorporates:
     *  UnitDelay: '<S246>/Unit Delay'
     */
    rtb_AND_nxkg = (rtb_AND_nxkg && (OBF_LaneAttributesValidHyst_bool[0]));

    /* Logic: '<S224>/AND' incorporates:
     *  Constant: '<S224>/Constant1'
     *  Logic: '<S224>/AND1'
     */
    rtb_LKA_TakeOverValid_bool =
        ((TJAOBF_LaneCheckEnabled_C_bool) && rtb_AND_ojra);

    /* Logic: '<S224>/AND' incorporates:
     *  Constant: '<S241>/Constant'
     *  Constant: '<S241>/Constant1'
     *  Inport: '<Root>/Inport27'
     *  Inport: '<Root>/Inport34'
     *  Logic: '<S241>/AND'
     *  RelationalOperator: '<S241>/GreaterThan'
     *  RelationalOperator: '<S241>/Less Than'
     *  Sum: '<S241>/Subtract'
     */
    OBF_LeftLaneCheckValid_bool =
        ((rtb_LKA_TakeOverValid_bool &&
          (((ODPFOP_AccObjPosX_met - ABPR_LeLnClthLength_met) <
            TJAOBF_MaxDiffLnLen2ObjPosX_C_met) &&
           (ABPR_LeLnClthLength_met > 0.0F))) &&
         rtb_AND_nxkg);

    /* Logic: '<S242>/OR1' incorporates:
     *  Constant: '<S244>/Constant'
     *  RelationalOperator: '<S242>/Equal1'
     */
    rtb_GEN_BrakePadelInvalid__gzuo =
        ((((uint32_T)TJALKA_LnBndValid_nu) ==
          E_TJALKA_LnBndValid_nu_BND_VAL_RIGHT_ONLY) ||
         rtb_GEN_BrakePadelInvalid__gzuo);

    /* Switch: '<S248>/Switch' incorporates:
     *  Constant: '<S242>/V_Parameter2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S248>/Max'
     *  Sum: '<S248>/Subtract'
     *  Switch: '<S248>/Switch1'
     *  UnaryMinus: '<S248>/Unary Minus'
     *  UnitDelay: '<S248>/Unit Delay'
     */
    if (rtb_GEN_BrakePadelInvalid__gzuo) {
        OBF_RightLineValidTurnOnDelay_sec =
            fmaxf(OBF_RightLineValidTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_RightLineValidTurnOnDelay_sec = TJAOBF_CheckLineValidMaxTime_sec;
    }

    /* End of Switch: '<S248>/Switch' */

    /* Logic: '<S248>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S248>/LessThanOrEqual'
     *  UnaryMinus: '<S248>/Unary Minus1'
     *  UnitDelay: '<S248>/Unit Delay'
     */
    rtb_GEN_BrakePadelInvalid__gzuo =
        (rtb_GEN_BrakePadelInvalid__gzuo &&
         (OBF_RightLineValidTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S242>/AND1' incorporates:
     *  UnitDelay: '<S246>/Unit Delay'
     */
    rtb_GEN_BrakePadelInvalid__gzuo = ((OBF_LaneAttributesValidHyst_bool[1]) &&
                                       rtb_GEN_BrakePadelInvalid__gzuo);

    /* Logic: '<S224>/AND1' incorporates:
     *  Constant: '<S241>/Constant2'
     *  Constant: '<S241>/Constant3'
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport34'
     *  Logic: '<S241>/AND1'
     *  RelationalOperator: '<S241>/GreaterThan1'
     *  RelationalOperator: '<S241>/Less Than1'
     *  Sum: '<S241>/Subtract1'
     */
    OBF_RightLaneCheckValid_bool =
        ((rtb_LKA_TakeOverValid_bool &&
          (((ODPFOP_AccObjPosX_met - ABPR_RiLnClthLength_met) <
            TJAOBF_MaxDiffLnLen2ObjPosX_C_met) &&
           (ABPR_RiLnClthLength_met > 0.0F))) &&
         rtb_GEN_BrakePadelInvalid__gzuo);

    /* Logic: '<S225>/OR2' */
    OBF_LaneCheckValid_bool =
        ((OBF_LeftLaneCheckValid_bool) || (OBF_RightLaneCheckValid_bool));

    /* Trigonometry: '<S231>/Tan' incorporates:
     *  Constant: '<S233>/Constant'
     *  Constant: '<S233>/Constant1'
     *  Inport: '<Root>/Inport24'
     *  MinMax: '<S233>/Max'
     *  MinMax: '<S233>/Min'
     */
    rtb_PosY0_met_dafa = tanf(
        fminf(fmaxf(ABPR_LeLnClthHeading_rad, -0.785398185F), 0.785398185F));

    /* Sum: '<S231>/Subtract' incorporates:
     *  Inport: '<Root>/Inport23'
     *  Inport: '<Root>/Inport34'
     */
    rtb_x_yaw = ODPFOP_AccObjPosX_met - ABPR_LeLnClthPosX0_met;

    /* Product: '<S231>/Product1' */
    rtb_ABPR_LeLnClthLength_met = rtb_x_yaw * rtb_x_yaw;

    /* Sum: '<S231>/Add' incorporates:
     *  Constant: '<S231>/Constant'
     *  Inport: '<Root>/Inport25'
     *  Product: '<S231>/Product'
     *  Product: '<S231>/Product3'
     *  Product: '<S231>/Product4'
     */
    rtb_Subtract_if0w_idx_0 =
        (rtb_PosY0_met_dafa * rtb_x_yaw) +
        ((ABPR_LeLnClthCrv_1pm * rtb_ABPR_LeLnClthLength_met) * 0.5F);

    /* Product: '<S231>/Product6' incorporates:
     *  Constant: '<S231>/Constant1'
     *  Inport: '<Root>/Inport26'
     *  Product: '<S231>/Product2'
     *  Product: '<S231>/Product5'
     */
    rtb_Subtract_if0w_idx_1 = ((rtb_ABPR_LeLnClthLength_met * rtb_x_yaw) *
                               ABPR_LeLnClthCrvChng_1pm2) *
                              0.166666672F;

    /* MinMax: '<S232>/Min' incorporates:
     *  Constant: '<S231>/Constant2'
     *  Constant: '<S232>/Constant2'
     *  Constant: '<S232>/Constant3'
     *  Inport: '<Root>/Inport25'
     *  Inport: '<Root>/Inport26'
     *  MinMax: '<S232>/Max'
     *  Product: '<S231>/Product7'
     *  Product: '<S231>/Product8'
     *  Product: '<S231>/Product9'
     *  Sum: '<S231>/Add3'
     *  Sum: '<S231>/Add4'
     */
    rtb_PosY0_met_dafa = fminf(
        fmaxf((((rtb_ABPR_LeLnClthLength_met * ABPR_LeLnClthCrvChng_1pm2) *
                0.5F) +
               rtb_PosY0_met_dafa) +
                  (rtb_x_yaw * ABPR_LeLnClthCrv_1pm),
              -1.0F),
        1.0F);

    /* Abs: '<S232>/Abs' */
    rtb_x_yaw = fabsf(rtb_PosY0_met_dafa);

    /* Product: '<S230>/Product' incorporates:
     *  Constant: '<S232>/Constant1'
     *  Constant: '<S232>/Constant4'
     *  Constant: '<S232>/Constant5'
     *  Constant: '<S232>/Constant6'
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport35'
     *  Product: '<S232>/Product'
     *  Product: '<S232>/Product1'
     *  Product: '<S232>/Product2'
     *  Product: '<S232>/Product3'
     *  Sum: '<S230>/Subtract1'
     *  Sum: '<S231>/Add1'
     *  Sum: '<S231>/Add2'
     *  Sum: '<S232>/Add'
     *  Sum: '<S232>/Subtract'
     *  Sum: '<S232>/Subtract1'
     *  Trigonometry: '<S230>/Cos'
     */
    rtb_PosY0_met_dafa = (((rtb_Subtract_if0w_idx_0 + rtb_Subtract_if0w_idx_1) +
                           ABPR_LeLnClthPosY0_met) -
                          ODPFOP_AccObjPosY_met) *
                         cosf((0.785398185F * rtb_PosY0_met_dafa) -
                              ((rtb_PosY0_met_dafa * (rtb_x_yaw - 1.0F)) *
                               ((rtb_x_yaw * 0.0663F) + 0.2447F)));

    /* Trigonometry: '<S235>/Tan' incorporates:
     *  Constant: '<S237>/Constant'
     *  Constant: '<S237>/Constant1'
     *  Inport: '<Root>/Inport29'
     *  MinMax: '<S237>/Max'
     *  MinMax: '<S237>/Min'
     */
    rtb_x_yaw = tanf(
        fminf(fmaxf(ABPR_RiLnClthHeading_rad, -0.785398185F), 0.785398185F));

    /* Sum: '<S235>/Subtract' incorporates:
     *  Inport: '<Root>/Inport28'
     *  Inport: '<Root>/Inport34'
     */
    rtb_ABPR_LeLnClthLength_met =
        ODPFOP_AccObjPosX_met - ABPR_RiLnClthPosX0_met;

    /* Product: '<S235>/Product1' */
    rtb_Subtract_if0w_idx_0 =
        rtb_ABPR_LeLnClthLength_met * rtb_ABPR_LeLnClthLength_met;

    /* Sum: '<S235>/Add' incorporates:
     *  Constant: '<S235>/Constant'
     *  Inport: '<Root>/Inport30'
     *  Product: '<S235>/Product'
     *  Product: '<S235>/Product3'
     *  Product: '<S235>/Product4'
     */
    rtb_Subtract_if0w_idx_1 =
        (rtb_x_yaw * rtb_ABPR_LeLnClthLength_met) +
        ((ABPR_RiLnClthCrv_1pm * rtb_Subtract_if0w_idx_0) * 0.5F);

    /* Product: '<S235>/Product6' incorporates:
     *  Constant: '<S235>/Constant1'
     *  Inport: '<Root>/Inport31'
     *  Product: '<S235>/Product2'
     *  Product: '<S235>/Product5'
     */
    rtb_Add_fbcu = ((rtb_Subtract_if0w_idx_0 * rtb_ABPR_LeLnClthLength_met) *
                    ABPR_RiLnClthCrvChng_1pm2) *
                   0.166666672F;

    /* MinMax: '<S236>/Min' incorporates:
     *  Constant: '<S235>/Constant2'
     *  Constant: '<S236>/Constant2'
     *  Constant: '<S236>/Constant3'
     *  Inport: '<Root>/Inport30'
     *  Inport: '<Root>/Inport31'
     *  MinMax: '<S236>/Max'
     *  Product: '<S235>/Product7'
     *  Product: '<S235>/Product8'
     *  Product: '<S235>/Product9'
     *  Sum: '<S235>/Add3'
     *  Sum: '<S235>/Add4'
     */
    rtb_Subtract_if0w_idx_0 = fminf(
        fmaxf((((rtb_Subtract_if0w_idx_0 * ABPR_RiLnClthCrvChng_1pm2) * 0.5F) +
               rtb_x_yaw) +
                  (rtb_ABPR_LeLnClthLength_met * ABPR_RiLnClthCrv_1pm),
              -1.0F),
        1.0F);

    /* Abs: '<S236>/Abs' */
    rtb_x_yaw = fabsf(rtb_Subtract_if0w_idx_0);

    /* Product: '<S234>/Product' incorporates:
     *  Constant: '<S236>/Constant1'
     *  Constant: '<S236>/Constant4'
     *  Constant: '<S236>/Constant5'
     *  Constant: '<S236>/Constant6'
     *  Inport: '<Root>/Inport2'
     *  Inport: '<Root>/Inport35'
     *  Product: '<S236>/Product'
     *  Product: '<S236>/Product1'
     *  Product: '<S236>/Product2'
     *  Product: '<S236>/Product3'
     *  Sum: '<S234>/Subtract1'
     *  Sum: '<S235>/Add1'
     *  Sum: '<S235>/Add2'
     *  Sum: '<S236>/Add'
     *  Sum: '<S236>/Subtract'
     *  Sum: '<S236>/Subtract1'
     *  Trigonometry: '<S234>/Cos'
     */
    rtb_Subtract_if0w_idx_0 =
        (((rtb_Subtract_if0w_idx_1 + rtb_Add_fbcu) + ABPR_RiLnClthPosY0_met) -
         ODPFOP_AccObjPosY_met) *
        cosf((0.785398185F * rtb_Subtract_if0w_idx_0) -
             ((rtb_Subtract_if0w_idx_0 * (rtb_x_yaw - 1.0F)) *
              ((rtb_x_yaw * 0.0663F) + 0.2447F)));

    /* Switch: '<S240>/Switch' incorporates:
     *  Constant: '<S240>/Constant'
     *  Constant: '<S240>/Constant3'
     *  Logic: '<S240>/AND'
     *  Switch: '<S240>/Switch1'
     *  Switch: '<S240>/Switch2'
     */
    if ((OBF_LeftLaneCheckValid_bool) && (OBF_RightLaneCheckValid_bool)) {
        rtb_DataTypeConversion1 = 0U;
    } else if (OBF_LeftLaneCheckValid_bool) {
        /* Switch: '<S240>/Switch1' incorporates:
         *  Constant: '<S240>/Constant1'
         */
        rtb_DataTypeConversion1 = 1U;
    } else if (OBF_RightLaneCheckValid_bool) {
        /* Switch: '<S240>/Switch2' incorporates:
         *  Constant: '<S240>/Constant2'
         *  Switch: '<S240>/Switch1'
         */
        rtb_DataTypeConversion1 = 2U;
    } else {
        rtb_DataTypeConversion1 = 3U;
    }

    /* End of Switch: '<S240>/Switch' */

    /* MultiPortSwitch: '<S229>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* MultiPortSwitch: '<S229>/Multiport Switch' incorporates:
             *  Constant: '<S229>/Constant1'
             *  Logic: '<S229>/AND'
             *  Logic: '<S229>/NOT'
             *  RelationalOperator: '<S229>/GreaterThan'
             *  RelationalOperator: '<S229>/Less Than'
             */
            OBF_TargetOutsideEgoLane_bool = ((rtb_PosY0_met_dafa <= 0.0F) ||
                                             (rtb_Subtract_if0w_idx_0 >= 0.0F));
            break;

        case 1:
            /* MultiPortSwitch: '<S229>/Multiport Switch' incorporates:
             *  Constant: '<S229>/Constant1'
             *  Logic: '<S229>/NOT1'
             *  RelationalOperator: '<S229>/GreaterThan'
             */
            OBF_TargetOutsideEgoLane_bool = (rtb_PosY0_met_dafa <= 0.0F);
            break;

        case 2:
            /* MultiPortSwitch: '<S229>/Multiport Switch' incorporates:
             *  Constant: '<S229>/Constant1'
             *  Logic: '<S229>/NOT2'
             *  RelationalOperator: '<S229>/Less Than'
             */
            OBF_TargetOutsideEgoLane_bool = (rtb_Subtract_if0w_idx_0 >= 0.0F);
            break;

        default:
            /* MultiPortSwitch: '<S229>/Multiport Switch' incorporates:
             *  Constant: '<S229>/Constant2'
             */
            OBF_TargetOutsideEgoLane_bool = false;
            break;
    }

    /* End of MultiPortSwitch: '<S229>/Multiport Switch' */

    /* Sum: '<S228>/Add' incorporates:
     *  Constant: '<S228>/Constant'
     *  Constant: '<S228>/Constant1'
     */
    rtb_Add_fbcu =
        TJAOBF_TgtVehDist2LnBndMin_C_met + TJAOBF_TgtVehDist2LnBndHst_C_met;

    /* Abs: '<S228>/Abs' incorporates:
     *  Abs: '<S228>/Abs1'
     */
    rtb_Subtract_if0w_idx_1 = fabsf(rtb_PosY0_met_dafa);
    rtb_Subtract_if0w_idx_0 = fabsf(rtb_Subtract_if0w_idx_0);

    /* Switch: '<S238>/Switch' incorporates:
     *  Abs: '<S228>/Abs'
     *  Constant: '<S228>/Constant'
     *  RelationalOperator: '<S238>/Less Than'
     *  RelationalOperator: '<S238>/Less Than1'
     *  Switch: '<S238>/Switch1'
     *  UnitDelay: '<S238>/Unit Delay'
     */
    OBF_MinDist2BndHyst_bool[0] =
        ((rtb_Add_fbcu < rtb_Subtract_if0w_idx_1) ||
         ((rtb_Subtract_if0w_idx_1 >= TJAOBF_TgtVehDist2LnBndMin_C_met) &&
          (OBF_MinDist2BndHyst_bool[0])));
    OBF_MinDist2BndHyst_bool[1] =
        ((rtb_Add_fbcu < rtb_Subtract_if0w_idx_0) ||
         ((rtb_Subtract_if0w_idx_0 >= TJAOBF_TgtVehDist2LnBndMin_C_met) &&
          (OBF_MinDist2BndHyst_bool[1])));

    /* Logic: '<S228>/NOT' incorporates:
     *  UnitDelay: '<S238>/Unit Delay'
     */
    OBF_MinDist2LeftBndInvalid = !OBF_MinDist2BndHyst_bool[0];

    /* Logic: '<S228>/NOT1' incorporates:
     *  UnitDelay: '<S238>/Unit Delay'
     */
    OBF_MinDist2RightBndInvalid = !OBF_MinDist2BndHyst_bool[1];

    /* Sum: '<S228>/Subtract' incorporates:
     *  Constant: '<S228>/Constant2'
     *  Constant: '<S228>/Constant4'
     */
    rtb_x_yaw =
        TJAOBF_DefaultLaneWidth_C_met - TJAOBF_TgtVehDist2LnBndMin_C_met;

    /* Sum: '<S228>/Subtract1' incorporates:
     *  Constant: '<S228>/Constant3'
     */
    rtb_Add_fbcu = rtb_x_yaw - TJAOBF_TgtVehDist2LnBndHst_C_met;

    /* Switch: '<S239>/Switch' incorporates:
     *  RelationalOperator: '<S239>/Less Than'
     *  RelationalOperator: '<S239>/Less Than1'
     *  Switch: '<S239>/Switch1'
     *  UnitDelay: '<S239>/Unit Delay'
     */
    OBF_MaxDist2BndHyst_bool[0] = ((rtb_x_yaw < rtb_Subtract_if0w_idx_1) ||
                                   ((rtb_Subtract_if0w_idx_1 >= rtb_Add_fbcu) &&
                                    (OBF_MaxDist2BndHyst_bool[0])));
    OBF_MaxDist2BndHyst_bool[1] = ((rtb_x_yaw < rtb_Subtract_if0w_idx_0) ||
                                   ((rtb_Subtract_if0w_idx_0 >= rtb_Add_fbcu) &&
                                    (OBF_MaxDist2BndHyst_bool[1])));

    /* Switch: '<S225>/Switch' incorporates:
     *  Constant: '<S225>/Constant'
     *  Logic: '<S225>/AND1'
     *  Logic: '<S225>/OR1'
     *  Switch: '<S225>/Switch1'
     *  Switch: '<S225>/Switch2'
     */
    if ((OBF_LeftLaneCheckValid_bool) && (OBF_RightLaneCheckValid_bool)) {
        rtb_AND18 =
            ((OBF_MinDist2LeftBndInvalid) || (OBF_MinDist2RightBndInvalid));
    } else if (OBF_RightLaneCheckValid_bool) {
        /* Switch: '<S225>/Switch1' incorporates:
         *  Logic: '<S225>/OR3'
         *  UnitDelay: '<S239>/Unit Delay'
         */
        rtb_AND18 =
            ((OBF_MinDist2RightBndInvalid) || (OBF_MaxDist2BndHyst_bool[1]));
    } else if (OBF_LeftLaneCheckValid_bool) {
        /* Switch: '<S225>/Switch2' incorporates:
         *  Logic: '<S225>/OR4'
         *  Switch: '<S225>/Switch1'
         *  UnitDelay: '<S239>/Unit Delay'
         */
        rtb_AND18 =
            ((OBF_MinDist2LeftBndInvalid) || (OBF_MaxDist2BndHyst_bool[0]));
    } else {
        rtb_AND18 = false;
    }

    /* End of Switch: '<S225>/Switch' */

    /* Logic: '<S225>/OR5' */
    OBF_DistOrEgoLaneInvalid_bool =
        ((OBF_TargetOutsideEgoLane_bool) || rtb_AND18);

    /* Switch: '<S220>/Switch1' incorporates:
     *  Logic: '<S208>/AND'
     *  Logic: '<S208>/NOT1'
     *  UnitDelay: '<S220>/Unit Delay1'
     */
    if (!rtb_OR_lkl2) {
        OBF_ObjLaneValidHoldUnitDelay_bool =
            ((!OBF_LaneCheckValid_bool) || (!OBF_DistOrEgoLaneInvalid_bool));
    }

    /* End of Switch: '<S220>/Switch1' */

    /* Switch: '<S222>/Switch' incorporates:
     *  Constant: '<S219>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S222>/Max'
     *  Sum: '<S222>/Subtract'
     *  Switch: '<S222>/Switch1'
     *  UnaryMinus: '<S222>/Unary Minus'
     *  UnitDelay: '<S220>/Unit Delay1'
     *  UnitDelay: '<S222>/Unit Delay'
     */
    if (OBF_ObjLaneValidHoldUnitDelay_bool) {
        OBF_ObjLaneValidDurationTurnOnDelay_sec =
            fmaxf(OBF_ObjLaneValidDurationTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_ObjLaneValidDurationTurnOnDelay_sec =
            TJAOBF_MinDurObjLnValidWR_C_sec;
    }

    /* End of Switch: '<S222>/Switch' */

    /* Logic: '<S222>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S222>/LessThanOrEqual'
     *  UnaryMinus: '<S222>/Unary Minus1'
     *  UnitDelay: '<S220>/Unit Delay1'
     *  UnitDelay: '<S222>/Unit Delay'
     */
    TJAOBF_ObjLaneValidDuration_bool =
        ((OBF_ObjLaneValidHoldUnitDelay_bool) &&
         (OBF_ObjLaneValidDurationTurnOnDelay_sec <=
          (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S182>/NOT1' incorporates:
     *  Logic: '<S221>/NOT1'
     */
    rtb_AND5_c5w4 = !TJAOBF_ObjLaneValidDuration_bool;
    rtb_VectorConcatenate[0] = rtb_AND5_c5w4;

    /* SignalConversion: '<S207>/Signal Conversion1' incorporates:
     *  Constant: '<S224>/Constant1'
     *  Logic: '<S224>/NOT'
     */
    rtb_VectorConcatenate_maf5[0] = !TJAOBF_LaneCheckEnabled_C_bool;

    /* SignalConversion: '<S207>/Signal Conversion2' incorporates:
     *  Logic: '<S224>/NOT1'
     */
    rtb_VectorConcatenate_maf5[1] = !rtb_AND_ojra;

    /* SignalConversion: '<S207>/Signal Conversion3' incorporates:
     *  Logic: '<S224>/NOT2'
     */
    rtb_VectorConcatenate_maf5[2] = !OBF_LeftLaneCheckValid_bool;

    /* SignalConversion: '<S207>/Signal Conversion4' incorporates:
     *  Logic: '<S224>/NOT3'
     */
    rtb_VectorConcatenate_maf5[3] = !OBF_RightLaneCheckValid_bool;

    /* SignalConversion: '<S207>/Signal Conversion5' */
    rtb_VectorConcatenate_maf5[4] = OBF_TargetOutsideEgoLane_bool;

    /* Logic: '<S225>/AND6' */
    rtb_AND_nxkg =
        ((OBF_LeftLaneCheckValid_bool) && (OBF_RightLaneCheckValid_bool));

    /* SignalConversion: '<S207>/Signal Conversion6' incorporates:
     *  Logic: '<S225>/AND5'
     */
    rtb_VectorConcatenate_maf5[5] =
        (rtb_AND_nxkg && (OBF_MinDist2LeftBndInvalid));

    /* SignalConversion: '<S207>/Signal Conversion7' incorporates:
     *  Logic: '<S225>/AND4'
     */
    rtb_VectorConcatenate_maf5[6] =
        (rtb_AND_nxkg && (OBF_MinDist2RightBndInvalid));

    /* SignalConversion: '<S207>/Signal Conversion9' incorporates:
     *  Logic: '<S225>/AND3'
     *  UnitDelay: '<S239>/Unit Delay'
     */
    rtb_VectorConcatenate_maf5[7] =
        ((OBF_LeftLaneCheckValid_bool) && (OBF_MaxDist2BndHyst_bool[0]));

    /* SignalConversion: '<S207>/Signal Conversion10' incorporates:
     *  Logic: '<S225>/AND2'
     *  UnitDelay: '<S239>/Unit Delay'
     */
    rtb_VectorConcatenate_maf5[8] =
        ((OBF_RightLaneCheckValid_bool) && (OBF_MaxDist2BndHyst_bool[1]));

    /* SignalConversion: '<S207>/Signal Conversion11' */
    rtb_VectorConcatenate_maf5[9] = rtb_OR_lkl2;

    /* SignalConversion: '<S207>/Signal Conversion12' incorporates:
     *  Logic: '<S221>/NOT'
     *  UnitDelay: '<S220>/Unit Delay1'
     */
    rtb_VectorConcatenate_maf5[10] = !OBF_ObjLaneValidHoldUnitDelay_bool;

    /* SignalConversion: '<S207>/Signal Conversion13' */
    rtb_VectorConcatenate_maf5[11] = rtb_AND5_c5w4;

    /* SignalConversion generated from: '<S217>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[12] = false;

    /* SignalConversion generated from: '<S217>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[13] = false;

    /* SignalConversion generated from: '<S217>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[14] = false;

    /* SignalConversion generated from: '<S217>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[15] = false;

    /* S-Function (ex_sfun_set_bit): '<S218>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S217>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_maf5[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_hcfw);

    /* RelationalOperator: '<S250>/Equal' incorporates:
     *  Constant: '<S250>/Constant1'
     *  Constant: '<S250>/Constant2'
     *  Inport: '<Root>/Inport32'
     *  S-Function (sfix_bitop): '<S250>/Bitwise AND'
     */
    rtb_AND_nxkg = ((((int32_T)ODPFOH_TgtObjClothoidInv_btf) &
                     ((int32_T)TJAOBF_TgtClthInvalid_C_btm)) == 0);

    /* Switch: '<S256>/Switch' incorporates:
     *  Constant: '<S250>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S256>/Max'
     *  Sum: '<S256>/Subtract'
     *  Switch: '<S256>/Switch1'
     *  UnaryMinus: '<S256>/Unary Minus'
     *  UnitDelay: '<S256>/Unit Delay'
     */
    if (rtb_AND_nxkg) {
        OBF_BitfieldValidTurnOnDelay_sec =
            fmaxf(OBF_BitfieldValidTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_BitfieldValidTurnOnDelay_sec = TJAOBF_TgtClthMinValidTime_C_sec;
    }

    /* End of Switch: '<S256>/Switch' */

    /* Logic: '<S256>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S256>/LessThanOrEqual'
     *  UnaryMinus: '<S256>/Unary Minus1'
     *  UnitDelay: '<S256>/Unit Delay'
     */
    rtb_AND_nxkg = (rtb_AND_nxkg && (OBF_BitfieldValidTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Lookup_n-D: '<S254>/1-D Lookup Table1' incorporates:
     *  Inport: '<Root>/Inport8'
     */
    rtb_Subtract_if0w_idx_0 = look1_iflf_binlxpw(
        VDy_VehVelocity_kph, ((const real32_T *)&(TJAOBF_VehVelX_Bx_kph[0])),
        ((const real32_T *)&(TJAOBF_TgtClthPosYMaxWR_Cr_met[0])), 5U);

    /* Abs: '<S254>/Abs' incorporates:
     *  Inport: '<Root>/Inport36'
     */
    rtb_Subtract_if0w_idx_1 = fabsf(ODPFOH_TgtObjPosY0_met);

    /* Switch: '<S259>/Switch' incorporates:
     *  Constant: '<S254>/Constant'
     *  Constant: '<S259>/Constant'
     *  RelationalOperator: '<S259>/Less Than'
     *  RelationalOperator: '<S259>/Less Than1'
     *  Sum: '<S254>/Add'
     *  UnitDelay: '<S259>/Unit Delay'
     */
    if ((rtb_Subtract_if0w_idx_0 + TJAOBF_TgtClthPosYMaxHyst_C_met) <
        rtb_Subtract_if0w_idx_1) {
        OBF_PosY0_SRHyst_bool = true;
    } else {
        OBF_PosY0_SRHyst_bool =
            ((rtb_Subtract_if0w_idx_1 >= rtb_Subtract_if0w_idx_0) &&
             (OBF_PosY0_SRHyst_bool));
    }

    /* End of Switch: '<S259>/Switch' */

    /* Logic: '<S254>/NOT' incorporates:
     *  UnitDelay: '<S259>/Unit Delay'
     */
    rtb_GEN_BrakePadelInvalid__gzuo = !OBF_PosY0_SRHyst_bool;

    /* Lookup_n-D: '<S252>/1-D Lookup Table1' incorporates:
     *  Inport: '<Root>/Inport8'
     */
    rtb_Add_fbcu = look1_iflf_binlxpw(
        VDy_VehVelocity_kph, ((const real32_T *)&(TJAOBF_VehVelX_Bx_kph[0])),
        ((const real32_T *)&(TJAOBF_TgtClthHeadMaxWR_Cr_rad[0])), 5U);

    /* Abs: '<S252>/Abs' incorporates:
     *  Inport: '<Root>/Inport37'
     */
    rtb_Abs_hlug = fabsf(ODPFOH_TgtObjHeadAng_rad);

    /* Switch: '<S258>/Switch' incorporates:
     *  Constant: '<S252>/Constant'
     *  Constant: '<S258>/Constant'
     *  RelationalOperator: '<S258>/Less Than'
     *  RelationalOperator: '<S258>/Less Than1'
     *  Sum: '<S252>/Add'
     *  UnitDelay: '<S258>/Unit Delay'
     */
    if ((rtb_Add_fbcu + TJAOBF_TgtClthHeadMaxHyst_C_rad) < rtb_Abs_hlug) {
        OBF_Heading_SRHyst_bool = true;
    } else {
        OBF_Heading_SRHyst_bool =
            ((rtb_Abs_hlug >= rtb_Add_fbcu) && (OBF_Heading_SRHyst_bool));
    }

    /* End of Switch: '<S258>/Switch' */

    /* Logic: '<S252>/NOT' incorporates:
     *  UnitDelay: '<S258>/Unit Delay'
     */
    rtb_LessThan_ko2v = !OBF_Heading_SRHyst_bool;

    /* Lookup_n-D: '<S251>/1-D Lookup Table1' incorporates:
     *  Inport: '<Root>/Inport8'
     */
    rtb_uDLookupTable1_eoam = look1_iflf_binlxpw(
        VDy_VehVelocity_kph, ((const real32_T *)&(TJAOBF_VehVelX_Bx_kph[0])),
        ((const real32_T *)&(TJAOBF_TgtClthCrvMaxWR_Cr_rad[0])), 5U);

    /* Abs: '<S251>/Abs' incorporates:
     *  Inport: '<Root>/Inport38'
     */
    rtb_Abs_ob4k = fabsf(ODPFOH_TgtObjCrv_1pm);

    /* Switch: '<S257>/Switch' incorporates:
     *  Constant: '<S251>/Constant'
     *  Constant: '<S257>/Constant'
     *  RelationalOperator: '<S257>/Less Than'
     *  RelationalOperator: '<S257>/Less Than1'
     *  Sum: '<S251>/Add'
     *  UnitDelay: '<S257>/Unit Delay'
     */
    if ((rtb_uDLookupTable1_eoam + TJAOBF_TgtClthCrvMaxHyst_C_1pm) <
        rtb_Abs_ob4k) {
        OBF_Crv_SRHyst_bool = true;
    } else {
        OBF_Crv_SRHyst_bool = ((rtb_Abs_ob4k >= rtb_uDLookupTable1_eoam) &&
                               (OBF_Crv_SRHyst_bool));
    }

    /* End of Switch: '<S257>/Switch' */

    /* Logic: '<S251>/NOT' incorporates:
     *  UnitDelay: '<S257>/Unit Delay'
     */
    rtb_LKA_VelocityValid_bool = !OBF_Crv_SRHyst_bool;

    /* RelationalOperator: '<S253>/GreaterThan' incorporates:
     *  Constant: '<S253>/Constant3'
     *  Inport: '<Root>/Inport39'
     */
    rtb_GEN_WR_Custom_bool =
        (ODPFOH_TgtObjLength_met > TJAOBF_TgtClthLengthMin_C_met);

    /* Logic: '<S184>/AND1' */
    OBF_TargetObjDataSR_bool =
        ((((rtb_AND_nxkg && rtb_GEN_BrakePadelInvalid__gzuo) &&
           rtb_LessThan_ko2v) &&
          rtb_LKA_VelocityValid_bool) &&
         rtb_GEN_WR_Custom_bool);

    /* Logic: '<S182>/NOT2' */
    rtb_VectorConcatenate[1] = !OBF_TargetObjDataSR_bool;

    /* Switch: '<S201>/Switch' incorporates:
     *  Constant: '<S192>/Constant1'
     *  Constant: '<S192>/Constant2'
     *  Constant: '<S201>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S201>/Less Than'
     *  RelationalOperator: '<S201>/Less Than1'
     *  Sum: '<S192>/Add'
     *  UnitDelay: '<S201>/Unit Delay'
     */
    if ((TJAOBF_VelXMax_C_kph + TJALKA_VelXHyst_C_kph) <
        VDy_DashboardVelocity_kph) {
        OBF_VelXMaxHyst_bool = true;
    } else {
        OBF_VelXMaxHyst_bool =
            ((VDy_DashboardVelocity_kph >= TJAOBF_VelXMax_C_kph) &&
             (OBF_VelXMaxHyst_bool));
    }

    /* End of Switch: '<S201>/Switch' */

    /* Switch: '<S202>/Switch' incorporates:
     *  Constant: '<S192>/Constant3'
     *  Constant: '<S192>/Constant4'
     *  Constant: '<S202>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S202>/Less Than'
     *  RelationalOperator: '<S202>/Less Than1'
     *  Sum: '<S192>/Subtract'
     *  UnitDelay: '<S202>/Unit Delay'
     */
    if (TJAOBF_VelXMin_C_kph < VDy_DashboardVelocity_kph) {
        OBF_VelXMinHyst_bool = true;
    } else {
        OBF_VelXMinHyst_bool =
            ((VDy_DashboardVelocity_kph >=
              (TJAOBF_VelXMin_C_kph - TJALKA_VelXHyst_C_kph)) &&
             (OBF_VelXMinHyst_bool));
    }

    /* End of Switch: '<S202>/Switch' */

    /* Logic: '<S192>/AND' incorporates:
     *  Logic: '<S192>/NOT'
     *  UnitDelay: '<S201>/Unit Delay'
     *  UnitDelay: '<S202>/Unit Delay'
     */
    rtb_OBF_VelocityValid_bool =
        ((!OBF_VelXMaxHyst_bool) && (OBF_VelXMinHyst_bool));

    /* Logic: '<S182>/NOT3' incorporates:
     *  Logic: '<S648>/AND9'
     */
    rtb_AND5_c5w4 = !rtb_OBF_VelocityValid_bool;
    rtb_VectorConcatenate[2] = rtb_AND5_c5w4;

    /* Logic: '<S186>/OR' incorporates:
     *  Constant: '<S196>/Constant'
     *  Constant: '<S197>/Constant'
     *  RelationalOperator: '<S186>/Equal'
     *  RelationalOperator: '<S186>/Equal1'
     */
    rtb_Equal2_kbd5 = ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
                        ((uint32_T)STM_LatCtrlMdUnitDelay_bool)) ||
                       (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB));

    /* Switch: '<S199>/Switch' incorporates:
     *  Constant: '<S186>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S199>/Max'
     *  Sum: '<S199>/Subtract'
     *  Switch: '<S199>/Switch1'
     *  UnaryMinus: '<S199>/Unary Minus'
     *  UnitDelay: '<S199>/Unit Delay'
     */
    if (rtb_Equal2_kbd5) {
        OBF_MinDurLCforOBTurnOnDelay_sec =
            fmaxf(OBF_MinDurLCforOBTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_MinDurLCforOBTurnOnDelay_sec = TJAOBF_MinDurLCforOB_C_sec;
    }

    /* End of Switch: '<S199>/Switch' */

    /* Logic: '<S101>/AND' incorporates:
     *  Constant: '<S101>/Constant1'
     */
    TJALKA_StrongReady_bool = ((((((((((rtb_OR_d1pw && rtb_AND_cvae) &&
                                       rtb_LKA_Dist2BndsValid_bool) &&
                                      rtb_LKA_LaneWidthValid_bool) &&
                                     rtb_LKA_RadiusValid_bool) &&
                                    rtb_AND_pjo1) &&
                                   rtb_AND_gpqq) &&
                                  (TJALKA_LC_Enabled_C_bool)) &&
                                 rtb_GEN_VehStInvalid_bool) &&
                                rtb_LKA_NoLaneChange_bool) &&
                               rtb_LKA_LengthValid_bool);

    /* Logic: '<S186>/AND1' incorporates:
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S15>/NOT1'
     *  Logic: '<S566>/NOT1'
     *  Logic: '<S8>/NOT2'
     */
    rtb_AND18 = !TJALKA_StrongReady_bool;

    /* Logic: '<S186>/NOT1' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Logic: '<S186>/AND1'
     *  Logic: '<S186>/NOT'
     *  Logic: '<S565>/OR'
     */
    rtb_AND_ixtq = (rtb_AND18 || TJALKA_Cancel_bool);

    /* Switch: '<S198>/Switch2' incorporates:
     *  Constant: '<S186>/Constant2'
     *  Constant: '<S198>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S186>/AND'
     *  Logic: '<S186>/AND1'
     *  Logic: '<S186>/NOT1'
     *  Logic: '<S198>/AND'
     *  Logic: '<S199>/AND'
     *  RelationalOperator: '<S198>/GreaterThan'
     *  RelationalOperator: '<S198>/LessThanOrEqual'
     *  RelationalOperator: '<S199>/LessThanOrEqual'
     *  Switch: '<S198>/Switch'
     *  UnaryMinus: '<S199>/Unary Minus1'
     *  UnitDelay: '<S198>/Unit Delay'
     *  UnitDelay: '<S199>/Unit Delay'
     */
    if (((rtb_Equal2_kbd5 && (OBF_MinDurLCforOBTurnOnDelay_sec <=
                              (-LCFRCV_TSysCycleTimeSen_sec))) &&
         rtb_AND_ixtq) &&
        (OBF_MaxDurObjBrdgTimerRe_sec <= 0.0F)) {
        OBF_MaxDurObjBrdgTimerRe_sec = TJAOBF_MaxDurObjBrdg_C_sec;
    } else if (OBF_MaxDurObjBrdgTimerRe_sec > LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S198>/Switch' incorporates:
         *  Sum: '<S198>/Subtract'
         */
        OBF_MaxDurObjBrdgTimerRe_sec =
            OBF_MaxDurObjBrdgTimerRe_sec - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* Switch: '<S198>/Switch' incorporates:
         *  Constant: '<S198>/Constant1'
         */
        OBF_MaxDurObjBrdgTimerRe_sec = 0.0F;
    }

    /* End of Switch: '<S198>/Switch2' */

    /* Logic: '<S186>/AND2' incorporates:
     *  Constant: '<S186>/Constant1'
     *  Constant: '<S198>/Constant2'
     *  RelationalOperator: '<S198>/GreaterThan1'
     *  UnitDelay: '<S198>/Unit Delay'
     */
    rtb_Equal2_kbd5 = ((OBF_MaxDurObjBrdgTimerRe_sec > 0.0F) &&
                       (TJAOBF_ObjBrdgEnabled_C_bool));

    /* SignalConversion: '<S182>/Signal Conversion3' */
    rtb_VectorConcatenate[3] = rtb_Equal2_kbd5;

    /* Logic: '<S184>/NOT' */
    rtb_VectorConcatenate_gygi[0] = !rtb_AND_nxkg;

    /* Logic: '<S184>/NOT1' */
    rtb_VectorConcatenate_gygi[1] = !rtb_GEN_BrakePadelInvalid__gzuo;

    /* Logic: '<S184>/NOT2' */
    rtb_VectorConcatenate_gygi[2] = !rtb_LessThan_ko2v;

    /* Logic: '<S184>/NOT3' */
    rtb_VectorConcatenate_gygi[3] = !rtb_LKA_VelocityValid_bool;

    /* Logic: '<S184>/NOT4' */
    rtb_VectorConcatenate_gygi[4] = !rtb_GEN_WR_Custom_bool;

    /* RelationalOperator: '<S254>/Less Than' */
    rtb_GEN_WR_Custom_bool =
        (rtb_Subtract_if0w_idx_1 < rtb_Subtract_if0w_idx_0);

    /* Logic: '<S184>/NOT5' */
    rtb_VectorConcatenate_gygi[5] = !rtb_GEN_WR_Custom_bool;

    /* RelationalOperator: '<S252>/Less Than' */
    rtb_LKA_VelocityValid_bool = (rtb_Abs_hlug < rtb_Add_fbcu);

    /* Logic: '<S184>/NOT6' */
    rtb_VectorConcatenate_gygi[6] = !rtb_LKA_VelocityValid_bool;

    /* RelationalOperator: '<S251>/Less Than' */
    rtb_LessThan_ko2v = (rtb_Abs_ob4k < rtb_uDLookupTable1_eoam);

    /* Logic: '<S184>/NOT7' */
    rtb_VectorConcatenate_gygi[7] = !rtb_LessThan_ko2v;

    /* S-Function (ex_sfun_set_bit): '<S255>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S249>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_gygi[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_nkl2);

    /* Switch: '<S194>/Switch' incorporates:
     *  Constant: '<S194>/Constant1'
     */
    if (TJASLC_SALC_Enabled_C_bool) {
        /* Logic: '<S194>/NOT' incorporates:
         *  Inport: '<Root>/Inport19'
         *  Logic: '<S194>/AND'
         *  UnitDelay: '<S194>/Unit Delay'
         */
        rtb_AND_nxkg = ((!OBF_TurnSignalHoldUnitDelay_bool) ||
                        LCFRCV_TurnSignalLeverHold_bool);
    } else {
        /* Logic: '<S194>/NOT' incorporates:
         *  Constant: '<S194>/Constant'
         *  Constant: '<S194>/Constant2'
         *  Inport: '<Root>/Inport74'
         *  RelationalOperator: '<S194>/NotEqual'
         *  S-Function (sfix_bitop): '<S194>/Bitwise AND'
         */
        rtb_AND_nxkg = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                         ((int32_T)TJAOBF_DrvStInvalidSR_C_btm)) == 0);
    }

    /* End of Switch: '<S194>/Switch' */

    /* Logic: '<S182>/NOT4' */
    rtb_VectorConcatenate[4] = !rtb_AND_nxkg;

    /* Logic: '<S48>/AND' incorporates:
     *  Constant: '<S48>/Constant'
     *  Inport: '<Root>/Inport65'
     */
    rtb_GEN_BrakePadelInvalid__gzuo =
        ((TJAGEN_LKA_Available_C_bool) && LCFRCV_LKASwitch_nu);

    /* Logic: '<S48>/OR' incorporates:
     *  Constant: '<S48>/Constant1'
     *  Constant: '<S48>/Constant2'
     *  Inport: '<Root>/Inport66'
     *  Logic: '<S48>/AND1'
     *  Logic: '<S48>/AND2'
     */
    rtb_GEN_VehStInvalid_bool =
        ((LCFRCV_TJASwitch_nu && (TJAGEN_TJA_Available_C_bool)) ||
         ((TJAGEN_TJA_Available_C_bool) &&
          (TJAGEN_TJA_ManFunctionSwitch_C_bool)));

    /* Logic: '<S48>/AND3' incorporates:
     *  Logic: '<S48>/NOT'
     */
    TJAGEN_LKAOnlySwitch_bool =
        (rtb_GEN_BrakePadelInvalid__gzuo && (!rtb_GEN_VehStInvalid_bool));

    /* Logic: '<S182>/NOT' incorporates:
     *  Logic: '<S15>/NOT'
     *  Logic: '<S463>/NOT'
     */
    rtb_TJASTM_ACCIsOFF_bool = !TJAGEN_LKAOnlySwitch_bool;

    /* Logic: '<S182>/AND' incorporates:
     *  Constant: '<S182>/Constant1'
     *  Logic: '<S182>/NOT'
     */
    rtb_AND_gpqq = ((TJAOBF_OF_Enabled_C_bool) && rtb_TJASTM_ACCIsOFF_bool);

    /* Logic: '<S182>/NOT5' */
    rtb_VectorConcatenate[5] = !rtb_AND_gpqq;

    /* Logic: '<S187>/NOT' incorporates:
     *  Constant: '<S187>/Constant1'
     *  Logic: '<S187>/AND'
     */
    rtb_AND_pjo1 = (rtb_AND2 || (!TJAOBF_ConstSiteCheckOn_C_bool));

    /* Logic: '<S182>/NOT6' */
    rtb_VectorConcatenate[6] = !rtb_AND_pjo1;

    /* RelationalOperator: '<S188>/Equal' incorporates:
     *  Constant: '<S188>/Constant'
     *  Constant: '<S188>/Constant1'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S188>/Bitwise AND'
     */
    rtb_AND_cvae = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                     ((int32_T)TJAOBF_PrjSpecQuSR_C_btm)) == 0);

    /* Logic: '<S182>/NOT7' */
    rtb_VectorConcatenate[7] = !rtb_AND_cvae;

    /* Logic: '<S184>/AND' */
    OBF_TargetObjDataWR_bool =
        ((rtb_GEN_WR_Custom_bool && rtb_LKA_VelocityValid_bool) &&
         rtb_LessThan_ko2v);

    /* Logic: '<S182>/NOT8' */
    rtb_VectorConcatenate[8] = !OBF_TargetObjDataWR_bool;

    /* S-Function (sfix_bitop): '<S188>/Bitwise AND1' incorporates:
     *  Constant: '<S188>/Constant2'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S486>/Bitwise AND1'
     */
    rtb_GEN_WR_Custom_bool_tmp =
        ((int32_T)CUSTOM_PrjSpecQu_btf) & ((int32_T)TJAOBF_PrjSpecQuWR_C_btm);

    /* RelationalOperator: '<S188>/Equal1' incorporates:
     *  Constant: '<S188>/Constant4'
     *  S-Function (sfix_bitop): '<S188>/Bitwise AND1'
     */
    rtb_GEN_WR_Custom_bool = (rtb_GEN_WR_Custom_bool_tmp == 0);

    /* Logic: '<S182>/NOT9' */
    rtb_VectorConcatenate[9] = !rtb_GEN_WR_Custom_bool;

    /* RelationalOperator: '<S188>/NotEqual' incorporates:
     *  Constant: '<S188>/Constant3'
     *  Constant: '<S188>/Constant5'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S188>/Bitwise AND2'
     */
    rtb_LKA_VelocityValid_bool = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                                   ((int32_T)TJAOBF_PrjSpecQuC_C_btm)) != 0);

    /* Logic: '<S182>/AND3' incorporates:
     *  Constant: '<S189>/Constant'
     *  Constant: '<S190>/Constant'
     *  Logic: '<S182>/OR1'
     *  Logic: '<S182>/OR2'
     *  RelationalOperator: '<S182>/Equal'
     *  RelationalOperator: '<S182>/Equal1'
     */
    TJAOBF_Cancel_bool = (rtb_LKA_VelocityValid_bool &&
                          ((((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB) ||
                           (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF)));

    /* Switch: '<S195>/Switch2' incorporates:
     *  Constant: '<S185>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S185>/OR'
     *  RelationalOperator: '<S195>/GreaterThan'
     *  Switch: '<S195>/Switch'
     *  UnitDelay: '<S195>/Unit Delay'
     */
    if (TJAOBF_Cancel_bool) {
        OBF_BlockTimeExpiredTimerRetrigger_sec = TJALKA_BlockTimeTJA_C_sec;
    } else if (OBF_BlockTimeExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S195>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S195>/Subtract'
         *  UnitDelay: '<S195>/Unit Delay'
         */
        OBF_BlockTimeExpiredTimerRetrigger_sec =
            OBF_BlockTimeExpiredTimerRetrigger_sec -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S195>/Unit Delay' incorporates:
         *  Constant: '<S195>/Constant1'
         *  Switch: '<S195>/Switch'
         */
        OBF_BlockTimeExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S195>/Switch2' */

    /* Logic: '<S185>/NOT' incorporates:
     *  Constant: '<S195>/Constant2'
     *  RelationalOperator: '<S195>/GreaterThan1'
     *  UnitDelay: '<S195>/Unit Delay'
     */
    rtb_LessThan_ko2v = (OBF_BlockTimeExpiredTimerRetrigger_sec <= 0.0F);

    /* Logic: '<S182>/NOT10' */
    rtb_VectorConcatenate[10] = !rtb_LessThan_ko2v;

    /* SignalConversion: '<S182>/Signal Conversion1' */
    rtb_VectorConcatenate[11] = rtb_LKA_VelocityValid_bool;

    /* SignalConversion generated from: '<S191>/Vector Concatenate' */
    rtb_VectorConcatenate[12] = false;

    /* SignalConversion generated from: '<S191>/Vector Concatenate' */
    rtb_VectorConcatenate[13] = false;

    /* SignalConversion generated from: '<S191>/Vector Concatenate' */
    rtb_VectorConcatenate[14] = false;

    /* SignalConversion generated from: '<S191>/Vector Concatenate' */
    rtb_VectorConcatenate[15] = false;

    /* S-Function (ex_sfun_set_bit): '<S200>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S191>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_hiin);

    /* Switch: '<S269>/Switch' incorporates:
     *  Constant: '<S269>/Constant1'
     *  Constant: '<S269>/Constant2'
     *  Inport: '<Root>/Inport43'
     *  RelationalOperator: '<S269>/Equal'
     *  S-Function (sfix_bitop): '<S269>/Bitwise AND'
     */
    rtb_SLC_LeAdjLaneValid_bool =
        (rtb_OR_idx_1 || ((((int32_T)ABPR_LeAdjLnInvalidQu_btf) &
                           ((int32_T)TJASLC_AdjLaneInvalid_C_btm)) == 0));

    /* Switch: '<S148>/Switch' incorporates:
     *  Constant: '<S120>/Constant'
     *  Constant: '<S148>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S148>/Less Than'
     *  RelationalOperator: '<S148>/Less Than1'
     *  UnitDelay: '<S148>/Unit Delay'
     */
    if (rtb_Switch_ithu < VDy_DashboardVelocity_kph) {
        LKA_VelXMaxHystWR_bool = true;
    } else {
        LKA_VelXMaxHystWR_bool =
            ((VDy_DashboardVelocity_kph >= TJALKA_VelXMax_C_kph) &&
             (LKA_VelXMaxHystWR_bool));
    }

    /* End of Switch: '<S148>/Switch' */

    /* Switch: '<S149>/Switch' incorporates:
     *  Constant: '<S120>/Constant2'
     *  Constant: '<S120>/Constant3'
     *  Constant: '<S149>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S149>/Less Than'
     *  RelationalOperator: '<S149>/Less Than1'
     *  Sum: '<S120>/Subtract'
     *  UnitDelay: '<S149>/Unit Delay'
     */
    if (TJALKA_VelXMinWR_C_kph < VDy_DashboardVelocity_kph) {
        LKA_VelXMinHystWR_bool = true;
    } else {
        LKA_VelXMinHystWR_bool =
            ((VDy_DashboardVelocity_kph >=
              (TJALKA_VelXMinWR_C_kph - TJALKA_VelXHyst_C_kph)) &&
             (LKA_VelXMinHystWR_bool));
    }

    /* End of Switch: '<S149>/Switch' */

    /* Logic: '<S120>/AND' incorporates:
     *  Logic: '<S120>/NOT'
     *  UnitDelay: '<S148>/Unit Delay'
     *  UnitDelay: '<S149>/Unit Delay'
     */
    rtb_LKA_VelocityValid_bool =
        ((!LKA_VelXMaxHystWR_bool) && (LKA_VelXMinHystWR_bool));

    /* Logic: '<S101>/AND2' incorporates:
     *  Constant: '<S107>/Constant2'
     *  Constant: '<S107>/Constant3'
     *  Inport: '<Root>/Inport12'
     *  RelationalOperator: '<S107>/Equal1'
     *  S-Function (sfix_bitop): '<S107>/Bitwise AND1'
     */
    TJALKA_WeakReady_bool = (((((((int32_T)CUSTOM_PrjSpecQu_btf) &
                                 ((int32_T)TJALKA_PrjSpecQuWR_C_btm)) == 0) &&
                               rtb_OR2_pwtt) &&
                              rtb_LKA_VelocityValid_bool) &&
                             rtb_GEN_DrvNotBuckledUp_bool);

    /* Logic: '<S277>/AND' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S277>/NOT'
     */
    rtb_TTG_CMB_bool = (TJALKA_StrongReady_bool && TJALKA_WeakReady_bool);
    rtb_GEN_DrvNotBuckledUp_bool = (rtb_TTG_CMB_bool && (!TJALKA_Cancel_bool));

    /* Logic: '<S280>/AND' incorporates:
     *  UnitDelay: '<S7>/Unit Delay'
     */
    rtb_OR2_pwtt =
        (rtb_GEN_DrvNotBuckledUp_bool && (SLC_PrevTakeoverValidUnitDelay_bool));

    /* SignalConversion: '<S261>/Signal Conversion1' */
    rtb_VectorConcatenate_mj2c[0] = rtb_OR2_pwtt;

    /* RelationalOperator: '<S273>/NotEqual' incorporates:
     *  Constant: '<S273>/Constant'
     *  Constant: '<S273>/Constant1'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S273>/Bitwise AND'
     */
    rtb_Equal1_p2mp = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                        ((int32_T)TJASLC_PrjSpecQuC_C_btm)) != 0);

    /* SignalConversion: '<S261>/Signal Conversion2' */
    rtb_VectorConcatenate_mj2c[1] = rtb_Equal1_p2mp;

    /* RelationalOperator: '<S50>/Equal' incorporates:
     *  Constant: '<S58>/Constant'
     */
    rtb_Equal_ciay = (((uint32_T)STM_SysStateUnitDelay_bool) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT);

    /* Switch: '<S61>/Switch2' incorporates:
     *  Constant: '<S59>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S59>/AND'
     *  Logic: '<S59>/NOT'
     *  RelationalOperator: '<S61>/GreaterThan'
     *  Switch: '<S61>/Switch'
     *  UnitDelay: '<S59>/Unit Delay'
     *  UnitDelay: '<S61>/Unit Delay'
     */
    if (rtb_Equal_ciay && (!GEN_PrevRampoutUnitDelay_bool)) {
        GEN_RampoutTimeExpiredTimerRetrigger_sec = TJAGEN_RampoutTimeMax_C_sec;
    } else if (GEN_RampoutTimeExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S61>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S61>/Subtract'
         *  UnitDelay: '<S61>/Unit Delay'
         */
        GEN_RampoutTimeExpiredTimerRetrigger_sec =
            GEN_RampoutTimeExpiredTimerRetrigger_sec -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S61>/Unit Delay' incorporates:
         *  Constant: '<S61>/Constant1'
         *  Switch: '<S61>/Switch'
         */
        GEN_RampoutTimeExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S61>/Switch2' */

    /* Switch: '<S60>/Switch' incorporates:
     *  Constant: '<S60>/Constant2'
     *  Constant: '<S61>/Constant2'
     *  Logic: '<S59>/NOT1'
     *  Logic: '<S59>/NOT2'
     *  RelationalOperator: '<S61>/GreaterThan1'
     *  UnitDelay: '<S60>/Unit Delay'
     *  UnitDelay: '<S61>/Unit Delay'
     */
    if (!rtb_Equal_ciay) {
        GEN_RampoutTimeExpiredRSFF_bool = false;
    } else {
        GEN_RampoutTimeExpiredRSFF_bool =
            ((GEN_RampoutTimeExpiredTimerRetrigger_sec <= 0.0F) ||
             (GEN_RampoutTimeExpiredRSFF_bool));
    }

    /* End of Switch: '<S60>/Switch' */

    /* Logic: '<S47>/AND' incorporates:
     *  Constant: '<S47>/Constant1'
     *  Constant: '<S47>/Constant2'
     *  Constant: '<S47>/Constant3'
     *  Constant: '<S47>/Constant4'
     *  Constant: '<S47>/Constant5'
     *  Constant: '<S57>/Constant'
     *  Inport: '<Root>/Inport14'
     *  Inport: '<Root>/Inport62'
     *  Inport: '<Root>/Inport63'
     *  Inport: '<Root>/Inport64'
     *  Logic: '<S47>/OR'
     *  RelationalOperator: '<S47>/Equal'
     *  RelationalOperator: '<S47>/Equal1'
     *  RelationalOperator: '<S47>/Equal2'
     *  S-Function (sfix_bitop): '<S47>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S47>/Bitwise AND1'
     */
    TJAGEN_Clearance_bool =
        ((((((uint32_T)MDCTR_ControllingFunction_nu) ==
            E_MCTLCF_ControllingFunction_nu_LCF_TJA) &&
           (LCFRCV_SysStOnLatDMC_bool || (TJAGEN_SetSysStOnLatDMC_C_bool))) &&
          ((((int32_T)TRJPLN_QuStatusTrajPlan_nu) &
            ((int32_T)TJAGEN_QuTrajPlanClearance_C_btm)) == 0)) &&
         ((((int32_T)TRJCTR_QuServTrajCtr_nu) &
           ((int32_T)TJAGEN_QuTrajCtrClearance_C_btm)) == 0));

    /* Logic: '<S50>/NOT' incorporates:
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S552>/NOT1'
     *  Switch: '<S552>/Switch2'
     *  Switch: '<S552>/Switch3'
     */
    rtb_Equal_awdj = !TJAGEN_Clearance_bool;

    /* Logic: '<S50>/AND' incorporates:
     *  Logic: '<S50>/NOT'
     *  Logic: '<S50>/OR'
     *  UnitDelay: '<S60>/Unit Delay'
     */
    TJAGEN_Degradation_bool =
        (rtb_Equal_ciay &&
         ((GEN_RampoutTimeExpiredRSFF_bool) || rtb_Equal_awdj));

    /* RelationalOperator: '<S285>/Equal' incorporates:
     *  Constant: '<S288>/Constant'
     */
    rtb_NotEqual3_lcsh = (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                          E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC);

    /* Switch: '<S290>/Switch2' incorporates:
     *  Constant: '<S285>/Constant1'
     *  Constant: '<S288>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S285>/AND'
     *  RelationalOperator: '<S285>/NotEqual'
     *  RelationalOperator: '<S290>/GreaterThan'
     *  Switch: '<S290>/Switch'
     *  UnitDelay: '<S285>/Unit Delay'
     *  UnitDelay: '<S290>/Unit Delay'
     */
    if (rtb_NotEqual3_lcsh && (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC !=
                               ((uint32_T)SLC_PrevLatCtrlMd3_Enum))) {
        SLC_MaxManeuTimeRetrigger_sec = TJASLC_ManeuverTimeMax_C_sec;
    } else if (SLC_MaxManeuTimeRetrigger_sec > LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S290>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S290>/Subtract'
         *  UnitDelay: '<S290>/Unit Delay'
         */
        SLC_MaxManeuTimeRetrigger_sec =
            SLC_MaxManeuTimeRetrigger_sec - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S290>/Unit Delay' incorporates:
         *  Constant: '<S290>/Constant1'
         *  Switch: '<S290>/Switch'
         */
        SLC_MaxManeuTimeRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S290>/Switch2' */

    /* Switch: '<S289>/Switch' incorporates:
     *  Constant: '<S289>/Constant2'
     *  Constant: '<S290>/Constant2'
     *  Logic: '<S285>/NOT'
     *  Logic: '<S285>/NOT1'
     *  RelationalOperator: '<S290>/GreaterThan1'
     *  UnitDelay: '<S289>/Unit Delay'
     *  UnitDelay: '<S290>/Unit Delay'
     */
    if (!rtb_NotEqual3_lcsh) {
        SLC_MaxManeuTimeRSFF_bool = false;
    } else {
        SLC_MaxManeuTimeRSFF_bool = ((SLC_MaxManeuTimeRetrigger_sec <= 0.0F) ||
                                     (SLC_MaxManeuTimeRSFF_bool));
    }

    /* End of Switch: '<S289>/Switch' */

    /* Logic: '<S276>/OR' incorporates:
     *  Constant: '<S291>/Constant'
     *  Constant: '<S292>/Constant'
     *  Constant: '<S293>/Constant'
     *  Logic: '<S286>/AND'
     *  Logic: '<S287>/AND'
     *  RelationalOperator: '<S286>/Equal'
     *  RelationalOperator: '<S287>/Equal'
     *  RelationalOperator: '<S287>/Equal1'
     *  UnitDelay: '<S287>/Unit Delay'
     *  UnitDelay: '<S289>/Unit Delay'
     */
    rtb_NotEqual3_lcsh = ((((((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                             E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) &&
                            (((uint32_T)SLC_PrevLatCtrlMd2_Enum) ==
                             E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC)) ||
                           ((((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                             E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
                            TJAGEN_Degradation_bool)) ||
                          (SLC_MaxManeuTimeRSFF_bool));

    /* SignalConversion: '<S261>/Signal Conversion3' */
    rtb_VectorConcatenate_mj2c[2] = rtb_NotEqual3_lcsh;

    /* Logic: '<S536>/AND4' incorporates:
     *  Inport: '<Root>/Inport44'
     *  Logic: '<S494>/NOT'
     *  Logic: '<S517>/NOT3'
     *  Logic: '<S518>/NOT3'
     */
    rtb_AND1_h3xh = !LCFRCV_TurnSignalLeft_bool;

    /* Switch: '<S544>/Switch' incorporates:
     *  Constant: '<S536>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S536>/AND4'
     *  MinMax: '<S544>/Max'
     *  Sum: '<S544>/Subtract'
     *  Switch: '<S544>/Switch1'
     *  UnaryMinus: '<S544>/Unary Minus'
     *  UnitDelay: '<S544>/Unit Delay'
     */
    if (rtb_AND1_h3xh) {
        SLC_LeftTurnSignalOffDelay_sec = fmaxf(SLC_LeftTurnSignalOffDelay_sec,
                                               -LCFRCV_TSysCycleTimeSen_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeftTurnSignalOffDelay_sec = TJASLC_TurnSignalOffTime_C_sec;
    }

    /* End of Switch: '<S544>/Switch' */

    /* Logic: '<S536>/AND' incorporates:
     *  Constant: '<S536>/Constant2'
     *  Constant: '<S536>/Constant4'
     *  Constant: '<S543>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport111'
     *  Inport: '<Root>/Inport45'
     *  Logic: '<S536>/AND3'
     *  Logic: '<S536>/AND4'
     *  Logic: '<S544>/AND'
     *  RelationalOperator: '<S536>/Equal'
     *  RelationalOperator: '<S536>/Equal2'
     *  RelationalOperator: '<S536>/Equal4'
     *  RelationalOperator: '<S544>/LessThanOrEqual'
     *  UnaryMinus: '<S544>/Unary Minus1'
     *  UnitDelay: '<S544>/Unit Delay'
     */
    rtb_LKA_TakeOverValid_bool =
        ((((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
          E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG) &&
         (((LCFRCV_TurnSignalRight_bool ||
            (rtb_AND1_h3xh && (SLC_LeftTurnSignalOffDelay_sec <=
                               (-LCFRCV_TSysCycleTimeSen_sec)))) ||
           (((int32_T)LCFRCV_TurnLightReqSt_nu) == 1)) ||
          (((int32_T)LCFRCV_TurnLightReqSt_nu) == 2)));

    /* Logic: '<S536>/AND5' incorporates:
     *  Inport: '<Root>/Inport45'
     *  Logic: '<S494>/NOT1'
     *  Logic: '<S517>/NOT1'
     *  Logic: '<S518>/NOT2'
     */
    rtb_CMB_VelocityValid_bool = !LCFRCV_TurnSignalRight_bool;

    /* Switch: '<S545>/Switch' incorporates:
     *  Constant: '<S536>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S536>/AND5'
     *  MinMax: '<S545>/Max'
     *  Sum: '<S545>/Subtract'
     *  Switch: '<S545>/Switch1'
     *  UnaryMinus: '<S545>/Unary Minus'
     *  UnitDelay: '<S545>/Unit Delay'
     */
    if (rtb_CMB_VelocityValid_bool) {
        SLC_RightTurnSignalOffDelay_sec = fmaxf(SLC_RightTurnSignalOffDelay_sec,
                                                -LCFRCV_TSysCycleTimeSen_sec) -
                                          LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_RightTurnSignalOffDelay_sec = TJASLC_TurnSignalOffTime_C_sec;
    }

    /* End of Switch: '<S545>/Switch' */

    /* Logic: '<S536>/AND1' incorporates:
     *  Constant: '<S536>/Constant3'
     *  Constant: '<S536>/Constant5'
     *  Constant: '<S542>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport111'
     *  Inport: '<Root>/Inport44'
     *  Logic: '<S536>/AND2'
     *  Logic: '<S536>/AND5'
     *  Logic: '<S545>/AND'
     *  RelationalOperator: '<S536>/Equal1'
     *  RelationalOperator: '<S536>/Equal3'
     *  RelationalOperator: '<S536>/Equal5'
     *  RelationalOperator: '<S545>/LessThanOrEqual'
     *  UnaryMinus: '<S545>/Unary Minus1'
     *  UnitDelay: '<S545>/Unit Delay'
     */
    rtb_SLC_DriverTriggerResetRight =
        ((((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
          E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG) &&
         (((LCFRCV_TurnSignalLeft_bool ||
            (rtb_CMB_VelocityValid_bool && (SLC_RightTurnSignalOffDelay_sec <=
                                            (-LCFRCV_TSysCycleTimeSen_sec)))) ||
           (((int32_T)LCFRCV_TurnLightReqSt_nu) == 3)) ||
          (((int32_T)LCFRCV_TurnLightReqSt_nu) == 4)));

    /* Logic: '<S274>/OR' */
    rtb_NotEqual2_emxm =
        (rtb_LKA_TakeOverValid_bool || rtb_SLC_DriverTriggerResetRight);

    /* Logic: '<S274>/AND' incorporates:
     *  Constant: '<S283>/Constant'
     *  Constant: '<S284>/Constant'
     *  Logic: '<S274>/OR2'
     *  RelationalOperator: '<S274>/Equal2'
     *  RelationalOperator: '<S274>/Equal3'
     */
    rtb_OR_idx_0 =
        (rtb_NotEqual2_emxm && ((((uint32_T)SLC_PrevManeuverState_Enum) ==
                                 E_TJASLC_ManeuverState_nu_NEWEGO) ||
                                (((uint32_T)SLC_PrevManeuverState_Enum) ==
                                 E_TJASLC_ManeuverState_nu_LCMEND)));

    /* SignalConversion: '<S261>/Signal Conversion4' */
    rtb_VectorConcatenate_mj2c[3] = rtb_OR_idx_0;

    /* Switch: '<S512>/Switch' incorporates:
     *  Constant: '<S266>/Constant9'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport116'
     *  MinMax: '<S512>/Max'
     *  Sum: '<S512>/Subtract'
     *  Switch: '<S512>/Switch1'
     *  UnaryMinus: '<S512>/Unary Minus'
     *  UnitDelay: '<S512>/Unit Delay'
     */
    if (LCCRA_inFrontSafeFlag_bool) {
        SLC_FrontSafeTrOnTime_sec =
            fmaxf(SLC_FrontSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_FrontSafeTrOnTime_sec = TJASLC_ObjSafeTime_C_sec;
    }

    /* End of Switch: '<S512>/Switch' */

    /* Logic: '<S266>/NOT2' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport116'
     *  Logic: '<S512>/AND'
     *  RelationalOperator: '<S512>/LessThanOrEqual'
     *  UnaryMinus: '<S512>/Unary Minus1'
     *  UnitDelay: '<S512>/Unit Delay'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        ((!LCCRA_inFrontSafeFlag_bool) ||
         (SLC_FrontSafeTrOnTime_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S509>/Switch' incorporates:
     *  Constant: '<S266>/Constant6'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S509>/Max'
     *  Sum: '<S509>/Subtract'
     *  Switch: '<S509>/Switch1'
     *  UnaryMinus: '<S509>/Unary Minus'
     *  UnitDelay: '<S509>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        SLC_FrontUnSafeTrOnTime_sec =
            fmaxf(SLC_FrontUnSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_FrontUnSafeTrOnTime_sec = TJASLC_ObjUnSafeTime_C_sec;
    }

    /* End of Switch: '<S509>/Switch' */

    /* Logic: '<S509>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S509>/LessThanOrEqual'
     *  UnaryMinus: '<S509>/Unary Minus1'
     *  UnitDelay: '<S509>/Unit Delay'
     */
    rtb_AND_psoa =
        (rtb_CMB_OFOLnLengthInvalid_bool &&
         (SLC_FrontUnSafeTrOnTime_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S519>/Switch' incorporates:
     *  Constant: '<S513>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport44'
     *  Inport: '<Root>/Inport45'
     *  MinMax: '<S519>/Max'
     *  UnaryMinus: '<S519>/Unary Minus'
     *  UnitDelay: '<S519>/Unit Delay'
     */
    if (LCFRCV_TurnSignalLeft_bool) {
        rtb_Subtract_if0w_idx_0 = TJASLC_TurnSignalDebTime_C_sec;
    } else {
        rtb_Subtract_if0w_idx_0 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        SLC_TurnSignalTurnOffDelay_sec[0]);
    }

    if (LCFRCV_TurnSignalRight_bool) {
        rtb_Subtract_if0w_idx_1 = TJASLC_TurnSignalDebTime_C_sec;
    } else {
        rtb_Subtract_if0w_idx_1 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        SLC_TurnSignalTurnOffDelay_sec[1]);
    }

    /* End of Switch: '<S519>/Switch' */

    /* Switch: '<S519>/Switch1' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport44'
     *  Inport: '<Root>/Inport45'
     *  Sum: '<S519>/Subtract'
     *  UnitDelay: '<S519>/Unit Delay'
     */
    if (LCFRCV_TurnSignalLeft_bool) {
        SLC_TurnSignalTurnOffDelay_sec[0] = rtb_Subtract_if0w_idx_0;
    } else {
        SLC_TurnSignalTurnOffDelay_sec[0] =
            rtb_Subtract_if0w_idx_0 - LCFRCV_TSysCycleTimeSen_sec;
    }

    if (LCFRCV_TurnSignalRight_bool) {
        SLC_TurnSignalTurnOffDelay_sec[1] = rtb_Subtract_if0w_idx_1;
    } else {
        SLC_TurnSignalTurnOffDelay_sec[1] =
            rtb_Subtract_if0w_idx_1 - LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S519>/Switch1' */

    /* Logic: '<S519>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport44'
     *  Inport: '<Root>/Inport45'
     *  RelationalOperator: '<S519>/GreaterThan'
     *  UnaryMinus: '<S519>/Unary Minus1'
     *  UnitDelay: '<S519>/Unit Delay'
     */
    rtb_Equal_i3l2_idx_0 =
        (LCFRCV_TurnSignalLeft_bool ||
         (SLC_TurnSignalTurnOffDelay_sec[0] > (-LCFRCV_TSysCycleTimeSen_sec)));
    rtb_Equal_i3l2_idx_1 =
        (LCFRCV_TurnSignalRight_bool ||
         (SLC_TurnSignalTurnOffDelay_sec[1] > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S513>/OR' incorporates:
     *  Constant: '<S513>/Constant1'
     *  Inport: '<Root>/Inport19'
     */
    rtb_NOT_auv4 =
        (LCFRCV_TurnSignalLeverHold_bool || (TJASLC_ManualTrigger_C_bool));

    /* Logic: '<S513>/AND2' incorporates:
     *  Logic: '<S513>/AND'
     *  Logic: '<S513>/NOT1'
     */
    rtb_SLC_LeTurnSignalOn_bool =
        ((rtb_Equal_i3l2_idx_0 && (!rtb_Equal_i3l2_idx_1)) && rtb_NOT_auv4);

    /* UnitDelay: '<S268>/Unit Delay' */
    SLC_PrevReset_bool = SLC_PrevResetUnitDelay_bool;

    /* Switch: '<S521>/Switch' incorporates:
     *  Constant: '<S521>/Constant2'
     *  Constant: '<S523>/Constant2'
     *  Logic: '<S517>/NOT2'
     *  Switch: '<S521>/Switch1'
     *  Switch: '<S523>/Switch'
     *  Switch: '<S523>/Switch1'
     *  UnitDelay: '<S521>/Unit Delay'
     *  UnitDelay: '<S523>/Unit Delay'
     */
    if (rtb_AND1_h3xh && rtb_CMB_VelocityValid_bool) {
        SLC_PrevResetRSFF_bool = false;
        SLC_PrevResetRSFF2_bool = false;
    } else {
        SLC_PrevResetRSFF_bool =
            ((SLC_PrevReset_bool) || (SLC_PrevResetRSFF_bool));
        SLC_PrevResetRSFF2_bool =
            ((SLC_PrevReset_bool) || (SLC_PrevResetRSFF2_bool));
    }

    /* End of Switch: '<S521>/Switch' */

    /* Switch: '<S520>/Switch' incorporates:
     *  Logic: '<S517>/NOT'
     *  Logic: '<S517>/OR'
     *  UnitDelay: '<S521>/Unit Delay'
     */
    SLC_LeverLeftEngaged_bool =
        (rtb_SLC_LeTurnSignalOn_bool && (!SLC_PrevResetRSFF_bool));

    /* Logic: '<S513>/AND3' incorporates:
     *  Logic: '<S513>/AND1'
     *  Logic: '<S513>/NOT'
     */
    rtb_SLC_RiTurnSignalOn_bool =
        (rtb_NOT_auv4 && ((!rtb_Equal_i3l2_idx_0) && rtb_Equal_i3l2_idx_1));

    /* Switch: '<S522>/Switch' incorporates:
     *  Logic: '<S518>/NOT'
     *  Logic: '<S518>/OR'
     *  UnitDelay: '<S523>/Unit Delay'
     */
    SLC_LeverRightEngaged_bool =
        (rtb_SLC_RiTurnSignalOn_bool && (!SLC_PrevResetRSFF2_bool));

    /* Switch: '<S541>/Switch' incorporates:
     *  Constant: '<S516>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S541>/Max'
     *  UnaryMinus: '<S541>/Unary Minus'
     *  UnitDelay: '<S541>/Unit Delay'
     */
    if (SLC_LeverLeftEngaged_bool) {
        rtb_Subtract_if0w_idx_0 = fmaxf(SLC_LeverEngagedTurnOnDelay2_sec[0],
                                        -LCFRCV_TSysCycleTimeSen_sec);
    } else {
        rtb_Subtract_if0w_idx_0 = TJASLC_TriggerTurnOnTime_C_sec;
    }

    if (SLC_LeverRightEngaged_bool) {
        rtb_Subtract_if0w_idx_1 = fmaxf(SLC_LeverEngagedTurnOnDelay2_sec[1],
                                        -LCFRCV_TSysCycleTimeSen_sec);
    } else {
        rtb_Subtract_if0w_idx_1 = TJASLC_TriggerTurnOnTime_C_sec;
    }

    /* End of Switch: '<S541>/Switch' */

    /* Switch: '<S541>/Switch1' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Sum: '<S541>/Subtract'
     *  UnitDelay: '<S541>/Unit Delay'
     */
    if (SLC_LeverLeftEngaged_bool) {
        SLC_LeverEngagedTurnOnDelay2_sec[0] =
            rtb_Subtract_if0w_idx_0 - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeverEngagedTurnOnDelay2_sec[0] = rtb_Subtract_if0w_idx_0;
    }

    if (SLC_LeverRightEngaged_bool) {
        SLC_LeverEngagedTurnOnDelay2_sec[1] =
            rtb_Subtract_if0w_idx_1 - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeverEngagedTurnOnDelay2_sec[1] = rtb_Subtract_if0w_idx_1;
    }

    /* End of Switch: '<S541>/Switch1' */

    /* Logic: '<S541>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S541>/LessThanOrEqual'
     *  UnaryMinus: '<S541>/Unary Minus1'
     *  UnitDelay: '<S541>/Unit Delay'
     */
    rtb_Switch1_eo0n_idx_0 =
        ((SLC_LeverLeftEngaged_bool) && (SLC_LeverEngagedTurnOnDelay2_sec[0] <=
                                         (-LCFRCV_TSysCycleTimeSen_sec)));
    rtb_Switch1_eo0n_idx_1 =
        ((SLC_LeverRightEngaged_bool) && (SLC_LeverEngagedTurnOnDelay2_sec[1] <=
                                          (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S516>/NOT' incorporates:
     *  Logic: '<S516>/OR'
     */
    rtb_NOT_auv4 =
        ((!rtb_LKA_TakeOverValid_bool) && (!rtb_SLC_DriverTriggerResetRight));

    /* Logic: '<S516>/AND' */
    SLC_TriggerLeft_bool = (rtb_Switch1_eo0n_idx_0 && rtb_NOT_auv4);

    /* Logic: '<S516>/AND1' */
    SLC_TriggerRight_bool = (rtb_Switch1_eo0n_idx_1 && rtb_NOT_auv4);

    /* Logic: '<S279>/AND1' incorporates:
     *  Constant: '<S279>/Constant1'
     *  Constant: '<S295>/Constant'
     *  Constant: '<S296>/Constant'
     *  Logic: '<S279>/OR1'
     *  Logic: '<S279>/OR2'
     *  Logic: '<S279>/OR4'
     *  Logic: '<S279>/OR6'
     *  RelationalOperator: '<S279>/Equal'
     *  RelationalOperator: '<S279>/Equal1'
     */
    rtb_AND2 = (((((uint32_T)SLC_PrevManeuverState_Enum) ==
                  E_TJASLC_ManeuverState_nu_NEWEGO) ||
                 (((uint32_T)SLC_PrevManeuverState_Enum) ==
                  E_TJASLC_ManeuverState_nu_LCMEND)) &&
                ((rtb_AND_psoa && (TJASLC_EnableFrontObjCancle_bool)) &&
                 ((SLC_TriggerLeft_bool) || (SLC_TriggerRight_bool))));

    /* SignalConversion: '<S261>/Signal Conversion5' */
    rtb_VectorConcatenate_mj2c[4] = rtb_AND2;

    /* Logic: '<S300>/NOT' incorporates:
     *  Logic: '<S365>/NOT'
     *  Logic: '<S490>/NOT'
     */
    rtb_NotEqual_ozi5_tmp_0 = !SLC_LeverLeftEngaged_bool;

    /* Logic: '<S300>/NOT1' incorporates:
     *  Logic: '<S365>/NOT1'
     *  Logic: '<S490>/NOT1'
     */
    rtb_NotEqual_ozi5_tmp = !SLC_LeverRightEngaged_bool;

    /* Switch: '<S303>/Switch' incorporates:
     *  Constant: '<S303>/Constant2'
     *  Inport: '<Root>/Inport11'
     *  Logic: '<S300>/AND'
     *  Logic: '<S300>/NOT'
     *  Logic: '<S300>/NOT1'
     *  UnitDelay: '<S303>/Unit Delay'
     */
    if (rtb_NotEqual_ozi5_tmp_0 && rtb_NotEqual_ozi5_tmp) {
        SLC_OELCNewEgoLaneRSFF_bool = false;
    } else {
        SLC_OELCNewEgoLaneRSFF_bool =
            (ABPR_LaneChangeDetected_bool || (SLC_OELCNewEgoLaneRSFF_bool));
    }

    /* End of Switch: '<S303>/Switch' */

    /* Switch: '<S301>/Switch' incorporates:
     *  Constant: '<S262>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S301>/Max'
     *  Sum: '<S301>/Subtract'
     *  Switch: '<S301>/Switch1'
     *  UnaryMinus: '<S301>/Unary Minus'
     *  UnitDelay: '<S301>/Unit Delay'
     */
    if (SLC_LeverLeftEngaged_bool) {
        SLC_GoBkLfTurnLtTurnOffDelay_sec = TJASLC_GoBackTurnLtDlyTiC_sec;
    } else {
        SLC_GoBkLfTurnLtTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_GoBkLfTurnLtTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S301>/Switch' */

    /* Switch: '<S302>/Switch' incorporates:
     *  Constant: '<S262>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S302>/Max'
     *  Sum: '<S302>/Subtract'
     *  Switch: '<S302>/Switch1'
     *  UnaryMinus: '<S302>/Unary Minus'
     *  UnitDelay: '<S302>/Unit Delay'
     */
    if (SLC_LeverRightEngaged_bool) {
        SLC_GoBkRiTurnLtTurnOffDelay_sec = TJASLC_GoBackTurnLtDlyTiC_sec;
    } else {
        SLC_GoBkRiTurnLtTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_GoBkRiTurnLtTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S302>/Switch' */

    /* Switch: '<S299>/Switch2' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S301>/OR'
     *  Logic: '<S302>/OR'
     *  RelationalOperator: '<S301>/GreaterThan'
     *  RelationalOperator: '<S302>/GreaterThan'
     *  Switch: '<S299>/Switch3'
     *  Switch: '<S299>/Switch4'
     *  Switch: '<S299>/Switch5'
     *  UnaryMinus: '<S301>/Unary Minus1'
     *  UnaryMinus: '<S302>/Unary Minus1'
     *  UnitDelay: '<S301>/Unit Delay'
     *  UnitDelay: '<S302>/Unit Delay'
     */
    if ((SLC_LeverLeftEngaged_bool) ||
        (SLC_GoBkLfTurnLtTurnOffDelay_sec > (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S299>/Switch' incorporates:
         *  Inport: '<Root>/Inport24'
         *  Inport: '<Root>/Inport29'
         *  Switch: '<S299>/Switch6'
         *  UnitDelay: '<S303>/Unit Delay'
         */
        if (SLC_OELCNewEgoLaneRSFF_bool) {
            /* Switch: '<S299>/Switch2' incorporates:
             *  Inport: '<Root>/Inport2'
             */
            SLC_CenterDistToBoundary_met = ABPR_RiLnClthPosY0_met;
            rtb_PosY0_met_dafa = ABPR_RiLnClthHeading_rad;
        } else {
            /* Switch: '<S299>/Switch2' incorporates:
             *  Inport: '<Root>/Inport1'
             */
            SLC_CenterDistToBoundary_met = ABPR_LeLnClthPosY0_met;
            rtb_PosY0_met_dafa = ABPR_LeLnClthHeading_rad;
        }

        /* End of Switch: '<S299>/Switch' */
    } else if ((SLC_LeverRightEngaged_bool) ||
               (SLC_GoBkRiTurnLtTurnOffDelay_sec >
                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S299>/Switch1' incorporates:
         *  Inport: '<Root>/Inport1'
         *  Inport: '<Root>/Inport2'
         *  Inport: '<Root>/Inport24'
         *  Inport: '<Root>/Inport29'
         *  Switch: '<S299>/Switch3'
         *  Switch: '<S299>/Switch5'
         *  Switch: '<S299>/Switch7'
         *  UnitDelay: '<S303>/Unit Delay'
         */
        if (SLC_OELCNewEgoLaneRSFF_bool) {
            rtb_Switch_ithu = ABPR_LeLnClthPosY0_met;
            rtb_PosY0_met_dafa = ABPR_LeLnClthHeading_rad;
        } else {
            rtb_Switch_ithu = ABPR_RiLnClthPosY0_met;
            rtb_PosY0_met_dafa = ABPR_RiLnClthHeading_rad;
        }

        /* End of Switch: '<S299>/Switch1' */

        /* Switch: '<S299>/Switch2' incorporates:
         *  Constant: '<S299>/Constant1'
         *  Sum: '<S299>/Subtract'
         *  Switch: '<S299>/Switch3'
         */
        SLC_CenterDistToBoundary_met = 0.0F - rtb_Switch_ithu;
    } else {
        /* Switch: '<S299>/Switch2' incorporates:
         *  Constant: '<S299>/Constant2'
         *  Switch: '<S299>/Switch3'
         */
        SLC_CenterDistToBoundary_met = 0.0F;

        /* Switch: '<S299>/Switch5' incorporates:
         *  Constant: '<S299>/Constant3'
         */
        rtb_PosY0_met_dafa = 0.0F;
    }

    /* End of Switch: '<S299>/Switch2' */

    /* Abs: '<S299>/Abs2' */
    rtb_PosY0_met_dafa = fabsf(rtb_PosY0_met_dafa);

    /* Lookup_n-D: '<S299>/1-D Lookup Table5' incorporates:
     *  Switch: '<S866>/Switch'
     */
    rtb_PosY0_met_dafa = look1_iflf_binlxpw(
        rtb_PosY0_met_dafa,
        ((const real32_T *)&(TJASLC_HeadingAllowAbort_By_rad[0])),
        ((const real32_T *)&(TJASLC_DistAllowAbortHd_Mp_nu[0])), 11U);

    /* RelationalOperator: '<S299>/GreaterThan' incorporates:
     *  Inport: '<Root>/Inport79'
     *  Lookup_n-D: '<S299>/1-D Lookup Table1'
     *  Product: '<S299>/Product1'
     */
    SLC_AllowGoBack_bool =
        (SLC_CenterDistToBoundary_met >
         (look1_iflf_binlxpw(
              VDy_VehVelX_mps,
              ((const real32_T *)&(TJASLC_VehVelXAllowAbort_Bx_mps[0])),
              ((const real32_T *)&(TJASLC_DistAllowAbortVelX_Mp_met[0])), 11U) *
          rtb_PosY0_met_dafa));

    /* Logic: '<S274>/OR3' incorporates:
     *  Constant: '<S274>/Constant'
     *  Logic: '<S279>/OR3'
     */
    rtb_SLC_RearAbort_bool =
        ((SLC_AllowGoBack_bool) || (TJASLC_DisableAllowGoBack_bool));

    /* Logic: '<S274>/AND1' incorporates:
     *  Constant: '<S281>/Constant'
     *  Constant: '<S282>/Constant'
     *  Logic: '<S274>/OR1'
     *  Logic: '<S274>/OR3'
     *  RelationalOperator: '<S274>/Equal'
     *  RelationalOperator: '<S274>/Equal1'
     */
    rtb_NotEqual2_emxm =
        (((rtb_NotEqual2_emxm && ((((uint32_T)SLC_PrevManeuverState_Enum) ==
                                   E_TJASLC_ManeuverState_nu_LCMSTART) ||
                                  (((uint32_T)SLC_PrevManeuverState_Enum) ==
                                   E_TJASLC_ManeuverState_nu_LATMVSTART))) &&
          rtb_GEN_DrvNotBuckledUp_bool) &&
         rtb_SLC_RearAbort_bool);

    /* SignalConversion: '<S261>/Signal Conversion6' */
    rtb_VectorConcatenate_mj2c[5] = rtb_NotEqual2_emxm;

    /* Switch: '<S510>/Switch' incorporates:
     *  Constant: '<S266>/Constant7'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport99'
     *  MinMax: '<S510>/Max'
     *  Sum: '<S510>/Subtract'
     *  Switch: '<S510>/Switch1'
     *  UnaryMinus: '<S510>/Unary Minus'
     *  UnitDelay: '<S510>/Unit Delay'
     */
    if (LCCRA_InLeftSafeFlag_bool) {
        SLC_LeftSafeTrOnTime_sec =
            fmaxf(SLC_LeftSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeftSafeTrOnTime_sec = TJASLC_ObjSafeTime_C_sec;
    }

    /* End of Switch: '<S510>/Switch' */

    /* Logic: '<S266>/NOT' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport99'
     *  Logic: '<S510>/AND'
     *  RelationalOperator: '<S510>/LessThanOrEqual'
     *  UnaryMinus: '<S510>/Unary Minus1'
     *  UnitDelay: '<S510>/Unit Delay'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        ((!LCCRA_InLeftSafeFlag_bool) ||
         (SLC_LeftSafeTrOnTime_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S507>/Switch' incorporates:
     *  Constant: '<S266>/Constant4'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S507>/Max'
     *  Sum: '<S507>/Subtract'
     *  Switch: '<S507>/Switch1'
     *  UnaryMinus: '<S507>/Unary Minus'
     *  UnitDelay: '<S507>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        SLC_LeftUnSafeTrOnTime_sec =
            fmaxf(SLC_LeftUnSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeftUnSafeTrOnTime_sec = TJASLC_ObjUnSafeTime_C_sec;
    }

    /* End of Switch: '<S507>/Switch' */

    /* Switch: '<S266>/Switch' incorporates:
     *  Constant: '<S266>/Constant'
     *  Inport: '<Root>/Inport46'
     *  Inport: '<Root>/Inport48'
     *  Inport: '<Root>/Inport50'
     *  Inport: '<Root>/Inport52'
     *  Inport: '<Root>/Inport56'
     *  Logic: '<S266>/AND'
     *  Logic: '<S266>/AND1'
     *  Logic: '<S266>/OR'
     */
    if (TJASLC_LcaBsdSigEnabled_C_bool) {
        rtb_CMB_OFOVelocityInvalid_bool =
            ((LCA_ActiveLeft_bool && LCA_WarningLeft_bool) ||
             (BSD_ActiveLeft_bool && BSD_WarningLeft_bool));
    } else {
        rtb_CMB_OFOVelocityInvalid_bool = S_ODPSOP_MSFlag_RearLeft_nu;
    }

    /* End of Switch: '<S266>/Switch' */

    /* Logic: '<S266>/OR2' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S507>/AND'
     *  RelationalOperator: '<S507>/LessThanOrEqual'
     *  UnaryMinus: '<S507>/Unary Minus1'
     *  UnitDelay: '<S507>/Unit Delay'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        ((rtb_CMB_OFOLnLengthInvalid_bool &&
          (SLC_LeftUnSafeTrOnTime_sec <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
         rtb_CMB_OFOVelocityInvalid_bool);

    /* Switch: '<S511>/Switch' incorporates:
     *  Constant: '<S266>/Constant8'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport100'
     *  MinMax: '<S511>/Max'
     *  Sum: '<S511>/Subtract'
     *  Switch: '<S511>/Switch1'
     *  UnaryMinus: '<S511>/Unary Minus'
     *  UnitDelay: '<S511>/Unit Delay'
     */
    if (LCCRA_InRightSafeFlag_bool) {
        SLC_RightSafeTrOnTime_sec =
            fmaxf(SLC_RightSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_RightSafeTrOnTime_sec = TJASLC_ObjSafeTime_C_sec;
    }

    /* End of Switch: '<S511>/Switch' */

    /* Logic: '<S266>/NOT1' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport100'
     *  Logic: '<S511>/AND'
     *  RelationalOperator: '<S511>/LessThanOrEqual'
     *  UnaryMinus: '<S511>/Unary Minus1'
     *  UnitDelay: '<S511>/Unit Delay'
     */
    rtb_SLC_VelocityValid_bool =
        ((!LCCRA_InRightSafeFlag_bool) ||
         (SLC_RightSafeTrOnTime_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S508>/Switch' incorporates:
     *  Constant: '<S266>/Constant5'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S508>/Max'
     *  Sum: '<S508>/Subtract'
     *  Switch: '<S508>/Switch1'
     *  UnaryMinus: '<S508>/Unary Minus'
     *  UnitDelay: '<S508>/Unit Delay'
     */
    if (rtb_SLC_VelocityValid_bool) {
        SLC_RightUnSafeTrOnTime_sec =
            fmaxf(SLC_RightUnSafeTrOnTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_RightUnSafeTrOnTime_sec = TJASLC_ObjUnSafeTime_C_sec;
    }

    /* End of Switch: '<S508>/Switch' */

    /* Switch: '<S266>/Switch1' incorporates:
     *  Constant: '<S266>/Constant2'
     *  Inport: '<Root>/Inport47'
     *  Inport: '<Root>/Inport49'
     *  Inport: '<Root>/Inport51'
     *  Inport: '<Root>/Inport53'
     *  Inport: '<Root>/Inport57'
     *  Logic: '<S266>/AND2'
     *  Logic: '<S266>/AND3'
     *  Logic: '<S266>/OR1'
     */
    if (TJASLC_LcaBsdSigEnabled_C_bool) {
        rtb_CMB_OFOVelocityInvalid_bool =
            ((LCA_ActiveRight_bool && LCA_WarningRight_bool) ||
             (BSD_ActiveRight_bool && BSD_WarningRight_bool));
    } else {
        rtb_CMB_OFOVelocityInvalid_bool = S_ODPSOP_MSFlag_RearRight_nu;
    }

    /* End of Switch: '<S266>/Switch1' */

    /* Logic: '<S266>/OR3' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S508>/AND'
     *  RelationalOperator: '<S508>/LessThanOrEqual'
     *  UnaryMinus: '<S508>/Unary Minus1'
     *  UnitDelay: '<S508>/Unit Delay'
     */
    rtb_SLC_VelocityValid_bool =
        ((rtb_SLC_VelocityValid_bool &&
          (SLC_RightUnSafeTrOnTime_sec <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
         rtb_CMB_OFOVelocityInvalid_bool);

    /* Switch: '<S506>/Switch' incorporates:
     *  Constant: '<S266>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S506>/Max'
     *  UnaryMinus: '<S506>/Unary Minus'
     *  UnitDelay: '<S506>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        rtb_Subtract_if0w_idx_0 = TJASLC_MinDurFreeAdjLane_C_sec;
    } else {
        rtb_Subtract_if0w_idx_0 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        SLC_RearObjDecTurnOffDelay_sec[0]);
    }

    if (rtb_SLC_VelocityValid_bool) {
        rtb_Subtract_if0w_idx_1 = TJASLC_MinDurFreeAdjLane_C_sec;
    } else {
        rtb_Subtract_if0w_idx_1 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        SLC_RearObjDecTurnOffDelay_sec[1]);
    }

    /* End of Switch: '<S506>/Switch' */

    /* Switch: '<S506>/Switch1' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Sum: '<S506>/Subtract'
     *  UnitDelay: '<S506>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        SLC_RearObjDecTurnOffDelay_sec[0] = rtb_Subtract_if0w_idx_0;
    } else {
        SLC_RearObjDecTurnOffDelay_sec[0] =
            rtb_Subtract_if0w_idx_0 - LCFRCV_TSysCycleTimeSen_sec;
    }

    if (rtb_SLC_VelocityValid_bool) {
        SLC_RearObjDecTurnOffDelay_sec[1] = rtb_Subtract_if0w_idx_1;
    } else {
        SLC_RearObjDecTurnOffDelay_sec[1] =
            rtb_Subtract_if0w_idx_1 - LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S506>/Switch1' */

    /* Logic: '<S266>/AND4' incorporates:
     *  Constant: '<S266>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S506>/OR'
     *  RelationalOperator: '<S506>/GreaterThan'
     *  UnaryMinus: '<S506>/Unary Minus1'
     *  UnitDelay: '<S506>/Unit Delay'
     */
    rtb_SLC_ObjectDetectedLeftRear_ = ((rtb_CMB_OFOLnLengthInvalid_bool ||
                                        (SLC_RearObjDecTurnOffDelay_sec[0] >
                                         (-LCFRCV_TSysCycleTimeSen_sec))) &&
                                       (TJASLC_CheckRearObjects_C_bool));

    /* Logic: '<S266>/AND5' incorporates:
     *  Constant: '<S266>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S506>/OR'
     *  RelationalOperator: '<S506>/GreaterThan'
     *  UnaryMinus: '<S506>/Unary Minus1'
     *  UnitDelay: '<S506>/Unit Delay'
     */
    rtb_SLC_ObjectDetectedRightRear =
        ((TJASLC_CheckRearObjects_C_bool) &&
         (rtb_SLC_VelocityValid_bool || (SLC_RearObjDecTurnOffDelay_sec[1] >
                                         (-LCFRCV_TSysCycleTimeSen_sec))));

    /* Logic: '<S279>/AND4' incorporates:
     *  Constant: '<S297>/Constant'
     *  Constant: '<S298>/Constant'
     *  Logic: '<S279>/AND2'
     *  Logic: '<S279>/AND3'
     *  Logic: '<S279>/OR5'
     *  Logic: '<S279>/OR7'
     *  RelationalOperator: '<S279>/Equal2'
     *  RelationalOperator: '<S279>/Equal3'
     */
    rtb_SLC_RearAbort_bool =
        (((((rtb_SLC_ObjectDetectedLeftRear_ && (SLC_TriggerLeft_bool)) ||
            ((SLC_TriggerRight_bool) && rtb_SLC_ObjectDetectedRightRear)) &&
           rtb_GEN_DrvNotBuckledUp_bool) &&
          rtb_SLC_RearAbort_bool) &&
         ((((uint32_T)SLC_PrevManeuverState_Enum) ==
           E_TJASLC_ManeuverState_nu_LATMVSTART) ||
          (((uint32_T)SLC_PrevManeuverState_Enum) ==
           E_TJASLC_ManeuverState_nu_LCMSTART)));

    /* SignalConversion: '<S261>/Signal Conversion7' */
    rtb_VectorConcatenate_mj2c[6] = rtb_SLC_RearAbort_bool;

    /* SignalConversion generated from: '<S278>/Vector Concatenate' */
    rtb_VectorConcatenate_mj2c[7] = false;

    /* Switch: '<S504>/Switch' incorporates:
     *  Constant: '<S488>/Constant1'
     *  Constant: '<S488>/Constant2'
     *  Constant: '<S504>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S504>/Less Than'
     *  RelationalOperator: '<S504>/Less Than1'
     *  Sum: '<S488>/Add'
     *  UnitDelay: '<S504>/Unit Delay'
     */
    if ((TJASLC_VelXMax_C_kph + TJALKA_VelXHyst_C_kph) <
        VDy_DashboardVelocity_kph) {
        SLC_EgoVehVelMaxHyst_bool = true;
    } else {
        SLC_EgoVehVelMaxHyst_bool =
            ((VDy_DashboardVelocity_kph >= TJASLC_VelXMax_C_kph) &&
             (SLC_EgoVehVelMaxHyst_bool));
    }

    /* End of Switch: '<S504>/Switch' */

    /* Switch: '<S505>/Switch' incorporates:
     *  Constant: '<S488>/Constant3'
     *  Constant: '<S488>/Constant4'
     *  Constant: '<S505>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S505>/Less Than'
     *  RelationalOperator: '<S505>/Less Than1'
     *  Sum: '<S488>/Subtract'
     *  UnitDelay: '<S505>/Unit Delay'
     */
    if (TJASLC_VelXMin_C_kph < VDy_DashboardVelocity_kph) {
        SLC_EgoVehVelMinHyst_bool = true;
    } else {
        SLC_EgoVehVelMinHyst_bool =
            ((VDy_DashboardVelocity_kph >=
              (TJASLC_VelXMin_C_kph - TJALKA_VelXHyst_C_kph)) &&
             (SLC_EgoVehVelMinHyst_bool));
    }

    /* End of Switch: '<S505>/Switch' */

    /* Logic: '<S488>/AND' incorporates:
     *  Logic: '<S488>/NOT'
     *  UnitDelay: '<S504>/Unit Delay'
     *  UnitDelay: '<S505>/Unit Delay'
     */
    rtb_SLC_VelocityValid_bool =
        ((!SLC_EgoVehVelMaxHyst_bool) && (SLC_EgoVehVelMinHyst_bool));

    /* Logic: '<S463>/NOT1' incorporates:
     *  Logic: '<S383>/OR3'
     */
    rtb_TVG_Rampout_bool = !rtb_SLC_VelocityValid_bool;

    /* SignalConversion: '<S461>/Signal Conversion18' incorporates:
     *  Logic: '<S463>/NOT1'
     */
    rtb_VectorConcatenate_aisn[0] = rtb_TVG_Rampout_bool;

    /* Logic: '<S463>/AND' incorporates:
     *  Constant: '<S463>/Constant1'
     */
    rtb_GEN_DrvNotBuckledUp_bool =
        ((TJASLC_SALC_Enabled_C_bool) && rtb_TJASTM_ACCIsOFF_bool);

    /* Logic: '<S463>/NOT2' */
    rtb_CMB_OFOVelocityInvalid_bool = !rtb_GEN_DrvNotBuckledUp_bool;

    /* SignalConversion: '<S461>/Signal Conversion19' */
    rtb_VectorConcatenate_aisn[1] = rtb_CMB_OFOVelocityInvalid_bool;

    /* RelationalOperator: '<S486>/Equal' incorporates:
     *  Constant: '<S486>/Constant'
     *  Constant: '<S486>/Constant1'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S486>/Bitwise AND'
     */
    rtb_GEN_BlockTimerExpired_bool =
        ((((int32_T)CUSTOM_PrjSpecQu_btf) &
          ((int32_T)TJASLC_PrjSpecQuSR_C_btm)) == 0);

    /* Logic: '<S463>/NOT3' */
    rtb_CMB_OFOObjLengthInvalid_boo = !rtb_GEN_BlockTimerExpired_bool;

    /* SignalConversion: '<S461>/Signal Conversion20' */
    rtb_VectorConcatenate_aisn[2] = rtb_CMB_OFOObjLengthInvalid_boo;

    /* RelationalOperator: '<S486>/Equal1' incorporates:
     *  Constant: '<S486>/Constant4'
     */
    rtb_GEN_Cancel_Custom_bool = (rtb_GEN_WR_Custom_bool_tmp == 0);

    /* Logic: '<S463>/NOT5' */
    rtb_NotEqual1_cuxj = !rtb_GEN_Cancel_Custom_bool;

    /* SignalConversion: '<S461>/Signal Conversion21' */
    rtb_VectorConcatenate_aisn[3] = rtb_NotEqual1_cuxj;

    /* Logic: '<S485>/OR' incorporates:
     *  Constant: '<S500>/Constant'
     *  Constant: '<S501>/Constant'
     *  RelationalOperator: '<S485>/Equal'
     *  RelationalOperator: '<S485>/Equal1'
     */
    rtb_OR_baue = ((((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) ||
                   (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB));

    /* Switch: '<S503>/Switch' incorporates:
     *  Constant: '<S485>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S503>/Max'
     *  Sum: '<S503>/Subtract'
     *  Switch: '<S503>/Switch1'
     *  UnaryMinus: '<S503>/Unary Minus'
     *  UnitDelay: '<S503>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LCActiveTurnOnDelay_sec =
            fmaxf(SLC_LCActiveTurnOnDelay_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCActiveTurnOnDelay_sec = TJASLC_MinDurLCTrigActv_C_sec;
    }

    /* End of Switch: '<S503>/Switch' */

    /* Switch: '<S485>/Switch' incorporates:
     *  Constant: '<S485>/Constant'
     *  Constant: '<S502>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S503>/AND'
     *  RelationalOperator: '<S485>/NotEqual'
     *  RelationalOperator: '<S503>/LessThanOrEqual'
     *  UnaryMinus: '<S503>/Unary Minus1'
     *  UnitDelay: '<S503>/Unit Delay'
     */
    if (((uint32_T)STM_LatCtrlMdUnitDelay_bool) !=
        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) {
        rtb_GEN_MaxSteerAngleExceeded_b =
            (rtb_OR_baue &&
             (SLC_LCActiveTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));
    } else {
        rtb_GEN_MaxSteerAngleExceeded_b = true;
    }

    /* End of Switch: '<S485>/Switch' */

    /* Logic: '<S463>/NOT6' */
    rtb_OR_baue = !rtb_GEN_MaxSteerAngleExceeded_b;

    /* SignalConversion: '<S461>/Signal Conversion22' */
    rtb_VectorConcatenate_aisn[4] = rtb_OR_baue;

    /* Switch: '<S496>/Switch' incorporates:
     *  Constant: '<S490>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport11'
     *  MinMax: '<S496>/Max'
     *  Sum: '<S496>/Subtract'
     *  Switch: '<S496>/Switch1'
     *  UnaryMinus: '<S496>/Unary Minus'
     *  UnitDelay: '<S496>/Unit Delay'
     */
    if (ABPR_LaneChangeDetected_bool) {
        SLC_LnChngDetectedTurnOffDelay_sec = 2.0F;
    } else {
        SLC_LnChngDetectedTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_LnChngDetectedTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S496>/Switch' */

    /* Switch: '<S495>/Switch' incorporates:
     *  Constant: '<S495>/Constant2'
     *  Inport: '<Root>/Inport11'
     *  Logic: '<S490>/AND'
     *  UnitDelay: '<S495>/Unit Delay'
     */
    if (rtb_NotEqual_ozi5_tmp_0 && rtb_NotEqual_ozi5_tmp) {
        SLC_LnChngDetectedRSFF_bool = false;
    } else {
        SLC_LnChngDetectedRSFF_bool =
            (ABPR_LaneChangeDetected_bool || (SLC_LnChngDetectedRSFF_bool));
    }

    /* End of Switch: '<S495>/Switch' */

    /* Switch: '<S499>/Switch' incorporates:
     *  Constant: '<S498>/Constant'
     *  Constant: '<S499>/Constant2'
     *  Logic: '<S494>/AND'
     *  RelationalOperator: '<S494>/Equal'
     *  UnitDelay: '<S499>/Unit Delay'
     */
    if (rtb_AND1_h3xh && rtb_CMB_VelocityValid_bool) {
        SLC_TurnSignalOffBlckRSFF_bool = false;
    } else {
        SLC_TurnSignalOffBlckRSFF_bool =
            ((((uint32_T)SLC_PrevManeuverState_Enum) ==
              E_TJASLC_ManeuverState_nu_LATMVSTART) ||
             (SLC_TurnSignalOffBlckRSFF_bool));
    }

    /* End of Switch: '<S499>/Switch' */

    /* Logic: '<S484>/OR' incorporates:
     *  Constant: '<S497>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport11'
     *  Logic: '<S490>/OR'
     *  Logic: '<S491>/AND'
     *  Logic: '<S496>/OR'
     *  RelationalOperator: '<S491>/Equal'
     *  RelationalOperator: '<S491>/NotEqual'
     *  RelationalOperator: '<S496>/GreaterThan'
     *  UnaryMinus: '<S496>/Unary Minus1'
     *  UnitDelay: '<S491>/Unit Delay'
     *  UnitDelay: '<S495>/Unit Delay'
     *  UnitDelay: '<S496>/Unit Delay'
     *  UnitDelay: '<S499>/Unit Delay'
     */
    rtb_SLC_PrevDriverTriggerResetR =
        ((((ABPR_LaneChangeDetected_bool ||
            (SLC_LnChngDetectedTurnOffDelay_sec >
             (-LCFRCV_TSysCycleTimeSen_sec))) ||
           (SLC_LnChngDetectedRSFF_bool)) ||
          ((((uint32_T)STM_LatCtrlMdUnitDelay_bool) !=
            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
           (((uint32_T)SLC_PrevLatCtrlMd_Enum) ==
            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC))) ||
         (SLC_TurnSignalOffBlckRSFF_bool));

    /* Switch: '<S492>/Switch' incorporates:
     *  Constant: '<S484>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S492>/Max'
     *  Sum: '<S492>/Subtract'
     *  Switch: '<S492>/Switch1'
     *  UnaryMinus: '<S492>/Unary Minus'
     *  UnitDelay: '<S492>/Unit Delay'
     */
    if (rtb_SLC_PrevDriverTriggerResetR) {
        SLC_LnChngBlockTimeTurnOffDelay_sec = TJASLC_BlockTimeSALC_C_sec;
    } else {
        SLC_LnChngBlockTimeTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_LnChngBlockTimeTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S492>/Switch' */

    /* Logic: '<S261>/OR' */
    SLC_Cancel_bool =
        ((((rtb_OR2_pwtt || rtb_Equal1_p2mp) || rtb_NotEqual3_lcsh) ||
          rtb_OR_idx_0) ||
         rtb_AND2);

    /* Switch: '<S493>/Switch' incorporates:
     *  Constant: '<S484>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S493>/Max'
     *  Sum: '<S493>/Subtract'
     *  Switch: '<S493>/Switch1'
     *  UnaryMinus: '<S493>/Unary Minus'
     *  UnitDelay: '<S493>/Unit Delay'
     */
    if (SLC_Cancel_bool) {
        SLC_LnChngBlockTimeCancleTfDelay_sec = TJASLC_BlockTmSALCCancle_C_sec;
    } else {
        SLC_LnChngBlockTimeCancleTfDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_LnChngBlockTimeCancleTfDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S493>/Switch' */

    /* Logic: '<S484>/NOT2' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S484>/NOT'
     *  Logic: '<S484>/NOT1'
     *  Logic: '<S492>/OR'
     *  Logic: '<S493>/OR'
     *  RelationalOperator: '<S492>/GreaterThan'
     *  RelationalOperator: '<S493>/GreaterThan'
     *  UnaryMinus: '<S492>/Unary Minus1'
     *  UnaryMinus: '<S493>/Unary Minus1'
     *  UnitDelay: '<S492>/Unit Delay'
     *  UnitDelay: '<S493>/Unit Delay'
     */
    rtb_SLC_PrevDriverTriggerResetR =
        (((!rtb_SLC_PrevDriverTriggerResetR) &&
          (SLC_LnChngBlockTimeTurnOffDelay_sec <=
           (-LCFRCV_TSysCycleTimeSen_sec))) &&
         ((!SLC_Cancel_bool) && (SLC_LnChngBlockTimeCancleTfDelay_sec <=
                                 (-LCFRCV_TSysCycleTimeSen_sec))));

    /* Logic: '<S463>/NOT7' */
    rtb_OR_idx_0 = !rtb_SLC_PrevDriverTriggerResetR;

    /* SignalConversion: '<S461>/Signal Conversion23' */
    rtb_VectorConcatenate_aisn[5] = rtb_OR_idx_0;

    /* RelationalOperator: '<S307>/Equal' incorporates:
     *  Constant: '<S307>/Constant'
     *  Constant: '<S307>/Constant1'
     *  Inport: '<Root>/Inport6'
     *  Inport: '<Root>/Inport7'
     *  S-Function (sfix_bitop): '<S307>/Bitwise AND'
     */
    rtb_Equal_i3l2_idx_0 = ((((int32_T)ABPR_LeLnInvalidQu_btf) &
                             ((int32_T)TJASLC_LaneInvalid_C_btm)) == 0);
    rtb_Equal_i3l2_idx_1 = ((((int32_T)ABPR_RiLnInvalidQu_btf) &
                             ((int32_T)TJASLC_LaneInvalid_C_btm)) == 0);

    /* Logic: '<S307>/OR' incorporates:
     *  Constant: '<S311>/Constant'
     *  Logic: '<S307>/AND'
     *  RelationalOperator: '<S307>/Equal1'
     */
    rtb_NotEqual3_lcsh = ((rtb_Equal_i3l2_idx_0 && rtb_Equal_i3l2_idx_1) ||
                          (((uint32_T)TJALKA_LnBndValid_nu) ==
                           E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE));

    /* Logic: '<S463>/NOT4' */
    rtb_Equal1_p2mp = !rtb_NotEqual3_lcsh;

    /* SignalConversion: '<S461>/Signal Conversion24' */
    rtb_VectorConcatenate_aisn[6] = rtb_Equal1_p2mp;

    /* Switch: '<S312>/Switch' incorporates:
     *  Constant: '<S308>/Constant1'
     *  Constant: '<S308>/Constant2'
     *  Constant: '<S312>/Constant'
     *  Inport: '<Root>/Inport5'
     *  RelationalOperator: '<S312>/Less Than'
     *  RelationalOperator: '<S312>/Less Than1'
     *  Sum: '<S308>/Add'
     *  UnitDelay: '<S312>/Unit Delay'
     */
    if ((TJALKA_LaneWidthMax_C_met + TJALKA_LaneWidthHyst_C_met) <
        ABPR_LaneWidth_met) {
        SLC_LaneWidthMaxHyst_bool = true;
    } else {
        SLC_LaneWidthMaxHyst_bool =
            ((ABPR_LaneWidth_met >= TJALKA_LaneWidthMax_C_met) &&
             (SLC_LaneWidthMaxHyst_bool));
    }

    /* End of Switch: '<S312>/Switch' */

    /* Switch: '<S313>/Switch' incorporates:
     *  Constant: '<S308>/Constant3'
     *  Constant: '<S308>/Constant4'
     *  Constant: '<S313>/Constant'
     *  Inport: '<Root>/Inport5'
     *  RelationalOperator: '<S313>/Less Than'
     *  RelationalOperator: '<S313>/Less Than1'
     *  Sum: '<S308>/Subtract'
     *  UnitDelay: '<S313>/Unit Delay'
     */
    if (TJALKA_LaneWidthMin_C_met < ABPR_LaneWidth_met) {
        SLC_LaneWidthMinHyst_bool = true;
    } else {
        SLC_LaneWidthMinHyst_bool =
            ((ABPR_LaneWidth_met >=
              (TJALKA_LaneWidthMin_C_met - TJALKA_LaneWidthHyst_C_met)) &&
             (SLC_LaneWidthMinHyst_bool));
    }

    /* End of Switch: '<S313>/Switch' */

    /* Logic: '<S487>/AND' incorporates:
     *  Abs: '<S305>/Abs'
     *  Constant: '<S305>/Constant'
     *  Inport: '<Root>/Inport25'
     *  Inport: '<Root>/Inport30'
     *  Logic: '<S308>/AND'
     *  Logic: '<S308>/NOT'
     *  RelationalOperator: '<S305>/Less Than'
     *  UnitDelay: '<S312>/Unit Delay'
     *  UnitDelay: '<S313>/Unit Delay'
     */
    rtb_OR2_pwtt =
        ((((((!SLC_LaneWidthMaxHyst_bool) && (SLC_LaneWidthMinHyst_bool)) &&
            (fabsf(ABPR_LeLnClthCrv_1pm) < TJASLC_EgoCrvMaxActivation_C_1pm)) &&
           (fabsf(ABPR_RiLnClthCrv_1pm) < TJASLC_EgoCrvMaxActivation_C_1pm)) &&
          rtb_Equal_i3l2_idx_0) &&
         rtb_Equal_i3l2_idx_1);

    /* Logic: '<S463>/NOT8' */
    rtb_OR_ly14 = !rtb_OR2_pwtt;

    /* SignalConversion: '<S461>/Signal Conversion25' */
    rtb_VectorConcatenate_aisn[7] = rtb_OR_ly14;

    /* SignalConversion: '<S461>/Signal Conversion26' */
    rtb_VectorConcatenate_aisn[8] = SLC_Cancel_bool;

    /* RelationalOperator: '<S304>/GreaterThan' incorporates:
     *  Constant: '<S304>/Constant'
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport27'
     */
    rtb_GreaterThan_bpjw_idx_0 =
        (ABPR_LeLnClthLength_met > TJASLC_EgoLengthMinActv_C_met);
    rtb_GreaterThan_bpjw_idx_1 =
        (ABPR_RiLnClthLength_met > TJASLC_EgoLengthMinActv_C_met);

    /* SignalConversion: '<S461>/Signal Conversion27' incorporates:
     *  Logic: '<S465>/NOT1'
     */
    rtb_VectorConcatenate_aisn[9] = !rtb_GreaterThan_bpjw_idx_1;

    /* Logic: '<S306>/AND1' incorporates:
     *  Constant: '<S306>/Constant10'
     *  Constant: '<S306>/Constant11'
     *  Constant: '<S306>/Constant12'
     *  Constant: '<S306>/Constant9'
     *  Inport: '<Root>/Inport55'
     *  RelationalOperator: '<S306>/Equal3'
     *  RelationalOperator: '<S306>/Equal7'
     *  RelationalOperator: '<S306>/Equal8'
     *  RelationalOperator: '<S306>/Equal9'
     */
    rtb_AND1_h3xh =
        ((((ABPR_RightLaneType_enum == TJASLC_ValidLaneType_C_nu) ||
           (ABPR_RightLaneType_enum == TJASLC_ValidLaneTypeDD_C_nu)) ||
          (ABPR_RightLaneType_enum == TJASLC_ValidLaneTypeTD_C_nu)) ||
         (ABPR_RightLaneType_enum == TJASLC_ValidLaneTypeDS_C_nu));

    /* Switch: '<S310>/Switch' incorporates:
     *  Constant: '<S306>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S310>/Max'
     *  Sum: '<S310>/Subtract'
     *  Switch: '<S310>/Switch1'
     *  UnaryMinus: '<S310>/Unary Minus'
     *  UnitDelay: '<S310>/Unit Delay'
     */
    if (rtb_AND1_h3xh) {
        SLC_RiLaneTypeTurnOffDelay_sec = TJASLC_LaneTypeDebTime_C_sec;
    } else {
        SLC_RiLaneTypeTurnOffDelay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               SLC_RiLaneTypeTurnOffDelay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S310>/Switch' */

    /* Switch: '<S306>/Switch1' incorporates:
     *  Constant: '<S306>/Constant'
     *  Constant: '<S306>/Constant5'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S310>/OR'
     *  RelationalOperator: '<S310>/GreaterThan'
     *  UnaryMinus: '<S310>/Unary Minus1'
     *  UnitDelay: '<S310>/Unit Delay'
     */
    if (TJASLC_CheckLaneTypes_C_bool) {
        rtb_AND1_h3xh = (rtb_AND1_h3xh || (SLC_RiLaneTypeTurnOffDelay_sec >
                                           (-LCFRCV_TSysCycleTimeSen_sec)));
    } else {
        rtb_AND1_h3xh = true;
    }

    /* End of Switch: '<S306>/Switch1' */

    /* SignalConversion: '<S461>/Signal Conversion28' incorporates:
     *  Logic: '<S465>/NOT2'
     */
    rtb_VectorConcatenate_aisn[10] = !rtb_AND1_h3xh;

    /* Switch: '<S270>/Switch1' incorporates:
     *  Logic: '<S270>/AND'
     *  Logic: '<S270>/NOT'
     *  UnitDelay: '<S271>/Unit Delay'
     *  UnitDelay: '<S272>/Unit Delay'
     */
    rtb_SLC_RiAdjLaneWidthValid_boo =
        (rtb_OR_idx_1 ||
         ((!SLC_AdjLnWidthMaxHyst_bool[1]) && (SLC_AdjLnWidthMinHyst_bool[1])));

    /* SignalConversion: '<S461>/Signal Conversion29' incorporates:
     *  Logic: '<S465>/NOT3'
     */
    rtb_VectorConcatenate_aisn[11] = !rtb_SLC_RiAdjLaneWidthValid_boo;

    /* Switch: '<S269>/Switch1' incorporates:
     *  Constant: '<S269>/Constant1'
     *  Constant: '<S269>/Constant2'
     *  Inport: '<Root>/Inport42'
     *  RelationalOperator: '<S269>/Equal'
     *  S-Function (sfix_bitop): '<S269>/Bitwise AND'
     */
    rtb_SLC_RiAdjLaneValid_bool =
        (rtb_OR_idx_1 || ((((int32_T)ABPR_RiAdjLnInvalidQu_btf) &
                           ((int32_T)TJASLC_AdjLaneInvalid_C_btm)) == 0));

    /* SignalConversion: '<S461>/Signal Conversion30' incorporates:
     *  Logic: '<S465>/NOT4'
     */
    rtb_VectorConcatenate_aisn[12] = !rtb_SLC_RiAdjLaneValid_bool;

    /* Logic: '<S465>/NOT5' */
    rtb_CMB_VelocityValid_bool =
        (rtb_SLC_ObjectDetectedRightRear || rtb_AND_psoa);

    /* SignalConversion: '<S461>/Signal Conversion31' */
    rtb_VectorConcatenate_aisn[13] = rtb_CMB_VelocityValid_bool;

    /* SignalConversion generated from: '<S467>/Vector Concatenate' */
    rtb_VectorConcatenate_aisn[14] = false;

    /* SignalConversion generated from: '<S467>/Vector Concatenate' */
    rtb_VectorConcatenate_aisn[15] = false;

    /* SignalConversion: '<S461>/Signal Conversion' incorporates:
     *  Logic: '<S463>/NOT1'
     */
    rtb_VectorConcatenate_maf5[0] = rtb_TVG_Rampout_bool;

    /* SignalConversion: '<S461>/Signal Conversion1' */
    rtb_VectorConcatenate_maf5[1] = rtb_CMB_OFOVelocityInvalid_bool;

    /* SignalConversion: '<S461>/Signal Conversion2' */
    rtb_VectorConcatenate_maf5[2] = rtb_CMB_OFOObjLengthInvalid_boo;

    /* SignalConversion: '<S461>/Signal Conversion3' */
    rtb_VectorConcatenate_maf5[3] = rtb_NotEqual1_cuxj;

    /* SignalConversion: '<S461>/Signal Conversion4' */
    rtb_VectorConcatenate_maf5[4] = rtb_OR_baue;

    /* SignalConversion: '<S461>/Signal Conversion5' */
    rtb_VectorConcatenate_maf5[5] = rtb_OR_idx_0;

    /* SignalConversion: '<S461>/Signal Conversion6' */
    rtb_VectorConcatenate_maf5[6] = rtb_Equal1_p2mp;

    /* SignalConversion: '<S461>/Signal Conversion7' */
    rtb_VectorConcatenate_maf5[7] = rtb_OR_ly14;

    /* SignalConversion: '<S461>/Signal Conversion8' */
    rtb_VectorConcatenate_maf5[8] = SLC_Cancel_bool;

    /* SignalConversion: '<S461>/Signal Conversion9' incorporates:
     *  Logic: '<S464>/NOT1'
     */
    rtb_VectorConcatenate_maf5[9] = !rtb_GreaterThan_bpjw_idx_0;

    /* Logic: '<S306>/AND2' incorporates:
     *  Constant: '<S306>/Constant4'
     *  Constant: '<S306>/Constant6'
     *  Constant: '<S306>/Constant7'
     *  Constant: '<S306>/Constant8'
     *  Inport: '<Root>/Inport54'
     *  RelationalOperator: '<S306>/Equal2'
     *  RelationalOperator: '<S306>/Equal4'
     *  RelationalOperator: '<S306>/Equal5'
     *  RelationalOperator: '<S306>/Equal6'
     */
    rtb_OR_idx_1 =
        ((((ABPR_LeftLaneType_enum == TJASLC_ValidLaneType_C_nu) ||
           (ABPR_LeftLaneType_enum == TJASLC_ValidLaneTypeDD_C_nu)) ||
          (ABPR_LeftLaneType_enum == TJASLC_ValidLaneTypeTD_C_nu)) ||
         (ABPR_LeftLaneType_enum == TJASLC_ValidLaneTypeSD_C_nu));

    /* Switch: '<S309>/Switch' incorporates:
     *  Constant: '<S306>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S309>/Max'
     *  Sum: '<S309>/Subtract'
     *  Switch: '<S309>/Switch1'
     *  UnaryMinus: '<S309>/Unary Minus'
     *  UnitDelay: '<S309>/Unit Delay'
     */
    if (rtb_OR_idx_1) {
        SLC_LeLaneTypeTurnOffDelay_sec = TJASLC_LaneTypeDebTime_C_sec;
    } else {
        SLC_LeLaneTypeTurnOffDelay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               SLC_LeLaneTypeTurnOffDelay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S309>/Switch' */

    /* Switch: '<S306>/Switch' incorporates:
     *  Constant: '<S306>/Constant'
     *  Constant: '<S306>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S309>/OR'
     *  RelationalOperator: '<S309>/GreaterThan'
     *  UnaryMinus: '<S309>/Unary Minus1'
     *  UnitDelay: '<S309>/Unit Delay'
     */
    if (TJASLC_CheckLaneTypes_C_bool) {
        rtb_OR_ly14 = (rtb_OR_idx_1 || (SLC_LeLaneTypeTurnOffDelay_sec >
                                        (-LCFRCV_TSysCycleTimeSen_sec)));
    } else {
        rtb_OR_ly14 = true;
    }

    /* End of Switch: '<S306>/Switch' */

    /* SignalConversion: '<S461>/Signal Conversion10' incorporates:
     *  Logic: '<S464>/NOT2'
     */
    rtb_VectorConcatenate_maf5[10] = !rtb_OR_ly14;

    /* SignalConversion: '<S461>/Signal Conversion11' incorporates:
     *  Logic: '<S464>/NOT3'
     */
    rtb_VectorConcatenate_maf5[11] = !rtb_SLC_LeAdjLaneWidthValid_boo;

    /* SignalConversion: '<S461>/Signal Conversion12' incorporates:
     *  Logic: '<S464>/NOT4'
     */
    rtb_VectorConcatenate_maf5[12] = !rtb_SLC_LeAdjLaneValid_bool;

    /* Logic: '<S464>/NOT5' */
    rtb_OR_baue = (rtb_SLC_ObjectDetectedLeftRear_ || rtb_AND_psoa);

    /* SignalConversion: '<S461>/Signal Conversion13' */
    rtb_VectorConcatenate_maf5[13] = rtb_OR_baue;

    /* RelationalOperator: '<S463>/NotEqual' incorporates:
     *  Constant: '<S489>/Constant'
     */
    rtb_NotEqual1_cuxj = (((uint32_T)SLC_PreAbortState_enum) ==
                          E_TJASLC_AbortState_nu_ABORT_NOACTIVE);

    /* SignalConversion: '<S461>/Signal Conversion14' incorporates:
     *  Logic: '<S463>/NOT9'
     */
    rtb_VectorConcatenate_maf5[14] = !rtb_NotEqual1_cuxj;

    /* SignalConversion generated from: '<S466>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[15] = false;

    /* S-Function (ex_sfun_set_bit): '<S468>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S466>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_maf5[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_m25k);

    /* S-Function (ex_sfun_set_bit): '<S469>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S467>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_aisn[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_d0a5);

    /* S-Function (ex_sfun_set_bit): '<S294>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S278>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_mj2c[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_gaa0);

    /* Switch: '<S462>/Switch' incorporates:
     *  Constant: '<S472>/Constant'
     *  Constant: '<S473>/Constant'
     *  Constant: '<S474>/Constant'
     *  Constant: '<S477>/Constant'
     *  Logic: '<S462>/OR2'
     *  RelationalOperator: '<S462>/Equal1'
     *  RelationalOperator: '<S462>/Equal2'
     *  RelationalOperator: '<S462>/Equal3'
     *  RelationalOperator: '<S462>/Equal4'
     *  Switch: '<S462>/Switch1'
     *  Switch: '<S462>/Switch2'
     *  UnitDelay: '<S478>/Unit Delay'
     */
    if (((uint32_T)STM_LatCtrlMdUnitDelay_bool) !=
        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) {
        rtb_Equal_i3l2_idx_0 = true;
        rtb_Equal_i3l2_idx_1 = true;
    } else if (((((uint32_T)SLC_PrevManeuverState_Enum) ==
                 E_TJASLC_ManeuverState_nu_NEWEGO) ||
                (((uint32_T)SLC_PrevManeuverState_Enum) ==
                 E_TJASLC_ManeuverState_nu_LCMEND)) ||
               (((uint32_T)SLC_PrevManeuverState_Enum) ==
                E_TJASLC_ManeuverState_nu_ABORT)) {
        /* Switch: '<S462>/Switch1' */
        rtb_Equal_i3l2_idx_0 = true;
        rtb_Equal_i3l2_idx_1 = true;
    } else if (SLC_LaneChangeRSFF_bool) {
        /* Switch: '<S462>/Switch2' incorporates:
         *  Switch: '<S462>/Switch1'
         */
        rtb_Equal_i3l2_idx_0 = true;
        rtb_Equal_i3l2_idx_1 = true;
    } else {
        /* Switch: '<S462>/Switch2' incorporates:
         *  Constant: '<S269>/Constant3'
         *  Constant: '<S269>/Constant4'
         *  Constant: '<S269>/Constant5'
         *  Inport: '<Root>/Inport42'
         *  Inport: '<Root>/Inport43'
         *  Logic: '<S462>/AND4'
         *  Logic: '<S462>/AND5'
         *  Logic: '<S462>/AND6'
         *  Logic: '<S462>/AND7'
         *  RelationalOperator: '<S269>/Equal1'
         *  S-Function (sfix_bitop): '<S269>/Bitwise AND1'
         *  Switch: '<S269>/Switch2'
         *  Switch: '<S269>/Switch3'
         *  Switch: '<S462>/Switch1'
         */
        rtb_Equal_i3l2_idx_0 =
            (rtb_SLC_LeAdjLaneWidthValid_boo &&
             (rtb_SLC_LeAdjLaneValid_bool ||
              ((TJASLC_CheckAdjLanes_C_bool) &&
               ((((int32_T)ABPR_LeAdjLnInvalidQu_btf) &
                 ((int32_T)TJASLC_AdjLaneBridged_C_btm)) != 0))));
        rtb_Equal_i3l2_idx_1 =
            (rtb_SLC_RiAdjLaneWidthValid_boo &&
             (rtb_SLC_RiAdjLaneValid_bool ||
              ((TJASLC_CheckAdjLanes_C_bool) &&
               ((((int32_T)ABPR_RiAdjLnInvalidQu_btf) &
                 ((int32_T)TJASLC_AdjLaneBridged_C_btm)) != 0))));
    }

    /* End of Switch: '<S462>/Switch' */

    /* Switch: '<S203>/Switch' incorporates:
     *  Constant: '<S203>/Constant'
     *  Inport: '<Root>/Inport46'
     *  Inport: '<Root>/Inport48'
     *  Inport: '<Root>/Inport50'
     *  Inport: '<Root>/Inport52'
     *  Inport: '<Root>/Inport56'
     *  Logic: '<S203>/AND'
     *  Logic: '<S203>/AND1'
     *  Logic: '<S203>/OR'
     */
    if (OF_LcaBsdSigEnabled_C_bool) {
        rtb_CMB_OFOVelocityInvalid_bool =
            ((LCA_ActiveLeft_bool && LCA_WarningLeft_bool) ||
             (BSD_ActiveLeft_bool && BSD_WarningLeft_bool));
    } else {
        rtb_CMB_OFOVelocityInvalid_bool = S_ODPSOP_MSFlag_RearLeft_nu;
    }

    /* End of Switch: '<S203>/Switch' */

    /* Logic: '<S203>/OR2' incorporates:
     *  Inport: '<Root>/Inport101'
     *  Inport: '<Root>/Inport103'
     *  Logic: '<S203>/AND6'
     *  Logic: '<S203>/NOT'
     */
    rtb_CMB_OFOObjLengthInvalid_boo = (((!LCCRA_InLeftRearSafeFlag_bool) ||
                                        (!LCCRA_InLeftFrontSafeFlag_bool)) ||
                                       rtb_CMB_OFOVelocityInvalid_bool);

    /* Switch: '<S203>/Switch1' incorporates:
     *  Constant: '<S203>/Constant2'
     *  Inport: '<Root>/Inport47'
     *  Inport: '<Root>/Inport49'
     *  Inport: '<Root>/Inport51'
     *  Inport: '<Root>/Inport53'
     *  Inport: '<Root>/Inport57'
     *  Logic: '<S203>/AND2'
     *  Logic: '<S203>/AND3'
     *  Logic: '<S203>/OR1'
     */
    if (OF_LcaBsdSigEnabled_C_bool) {
        rtb_CMB_OFOVelocityInvalid_bool =
            ((LCA_ActiveRight_bool && LCA_WarningRight_bool) ||
             (BSD_ActiveRight_bool && BSD_WarningRight_bool));
    } else {
        rtb_CMB_OFOVelocityInvalid_bool = S_ODPSOP_MSFlag_RearRight_nu;
    }

    /* End of Switch: '<S203>/Switch1' */

    /* Logic: '<S203>/OR3' incorporates:
     *  Inport: '<Root>/Inport102'
     *  Inport: '<Root>/Inport104'
     *  Logic: '<S203>/AND7'
     *  Logic: '<S203>/NOT1'
     */
    rtb_CMB_OFOVelocityInvalid_bool = (((!LCCRA_InRightRearSafeFlag_bool) ||
                                        (!LCCRA_InRightFrontSafeFlag_bool)) ||
                                       rtb_CMB_OFOVelocityInvalid_bool);

    /* Switch: '<S205>/Switch' incorporates:
     *  Constant: '<S203>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S205>/Max'
     *  UnaryMinus: '<S205>/Unary Minus'
     *  UnitDelay: '<S205>/Unit Delay'
     */
    if (rtb_CMB_OFOObjLengthInvalid_boo) {
        rtb_Subtract_if0w_idx_0 = OF_MinDurFreeAdjLane_C_sec;
    } else {
        rtb_Subtract_if0w_idx_0 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        OF_RearObjDecTurnOffDelay_sec[0]);
    }

    if (rtb_CMB_OFOVelocityInvalid_bool) {
        rtb_Subtract_if0w_idx_1 = OF_MinDurFreeAdjLane_C_sec;
    } else {
        rtb_Subtract_if0w_idx_1 = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                        OF_RearObjDecTurnOffDelay_sec[1]);
    }

    /* End of Switch: '<S205>/Switch' */

    /* Switch: '<S205>/Switch1' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Sum: '<S205>/Subtract'
     *  UnitDelay: '<S205>/Unit Delay'
     */
    if (rtb_CMB_OFOObjLengthInvalid_boo) {
        OF_RearObjDecTurnOffDelay_sec[0] = rtb_Subtract_if0w_idx_0;
    } else {
        OF_RearObjDecTurnOffDelay_sec[0] =
            rtb_Subtract_if0w_idx_0 - LCFRCV_TSysCycleTimeSen_sec;
    }

    if (rtb_CMB_OFOVelocityInvalid_bool) {
        OF_RearObjDecTurnOffDelay_sec[1] = rtb_Subtract_if0w_idx_1;
    } else {
        OF_RearObjDecTurnOffDelay_sec[1] =
            rtb_Subtract_if0w_idx_1 - LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S205>/Switch1' */

    /* MinMax: '<S16>/Max' incorporates:
     *  Constant: '<S16>/Constant'
     *  Inport: '<Root>/Inport10'
     */
    rtb_Subtract_if0w_idx_0 =
        fmaxf(TJACMB_LnQualTurnOffTime_C_sec, LCFRCV_TSysCycleTimeSen_sec);

    /* RelationalOperator: '<S16>/GreaterThan1' incorporates:
     *  Constant: '<S16>/Constant4'
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        (ABPR_LeLnQuality_perc > TJACMB_LaneQualityMin_C_perc);
    rtb_Equal1_p2mp = (ABPR_RiLnQuality_perc > TJACMB_LaneQualityMin_C_perc);

    /* Switch: '<S44>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S44>/Max'
     *  Sum: '<S44>/Subtract'
     *  Switch: '<S44>/Switch1'
     *  UnaryMinus: '<S44>/Unary Minus'
     *  UnitDelay: '<S44>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        CMB_LnQualTurnOffDelay_sec[0] = rtb_Subtract_if0w_idx_0;
    } else {
        CMB_LnQualTurnOffDelay_sec[0] =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, CMB_LnQualTurnOffDelay_sec[0]) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S44>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S44>/GreaterThan'
     *  Switch: '<S44>/Switch'
     *  UnaryMinus: '<S44>/Unary Minus1'
     *  UnitDelay: '<S44>/Unit Delay'
     */
    rtb_OR_imzg_idx_0 =
        (rtb_CMB_OFOLnLengthInvalid_bool ||
         (CMB_LnQualTurnOffDelay_sec[0] > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S44>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S44>/Max'
     *  Sum: '<S44>/Subtract'
     *  Switch: '<S44>/Switch1'
     *  UnaryMinus: '<S44>/Unary Minus'
     *  UnitDelay: '<S44>/Unit Delay'
     */
    if (rtb_Equal1_p2mp) {
        CMB_LnQualTurnOffDelay_sec[1] = rtb_Subtract_if0w_idx_0;
    } else {
        CMB_LnQualTurnOffDelay_sec[1] =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, CMB_LnQualTurnOffDelay_sec[1]) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* Logic: '<S44>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S44>/GreaterThan'
     *  Switch: '<S44>/Switch'
     *  UnaryMinus: '<S44>/Unary Minus1'
     *  UnitDelay: '<S44>/Unit Delay'
     */
    rtb_OR_imzg_idx_1 = (rtb_Equal1_p2mp || (CMB_LnQualTurnOffDelay_sec[1] >
                                             (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S205>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S205>/GreaterThan'
     *  UnaryMinus: '<S205>/Unary Minus1'
     *  UnitDelay: '<S205>/Unit Delay'
     */
    rtb_OR_idx_0 =
        (rtb_CMB_OFOObjLengthInvalid_boo ||
         (OF_RearObjDecTurnOffDelay_sec[0] > (-LCFRCV_TSysCycleTimeSen_sec)));
    rtb_OR_idx_1 =
        (rtb_CMB_OFOVelocityInvalid_bool ||
         (OF_RearObjDecTurnOffDelay_sec[1] > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Sum: '<S16>/Add' incorporates:
     *  Constant: '<S16>/Constant3'
     *  Constant: '<S16>/Constant4'
     */
    rtb_DataTypeConversion1 =
        (uint8_T)(((uint32_T)TJACMB_LaneQualityHyst_C_perc) +
                  ((uint32_T)TJACMB_LaneQualityMin_C_perc));

    /* Logic: '<S16>/AND2' incorporates:
     *  Constant: '<S16>/Constant2'
     *  Inport: '<Root>/Inport15'
     *  Inport: '<Root>/Inport16'
     *  RelationalOperator: '<S16>/GreaterThan'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        ((TJACMB_LnQualCheckEnable_C_bool) &&
         (ABPR_LeLnQuality_perc > rtb_DataTypeConversion1));
    rtb_Equal1_p2mp = ((TJACMB_LnQualCheckEnable_C_bool) &&
                       (ABPR_RiLnQuality_perc > rtb_DataTypeConversion1));

    /* MinMax: '<S16>/Max1' incorporates:
     *  Constant: '<S16>/Constant1'
     *  Inport: '<Root>/Inport10'
     */
    rtb_Subtract_if0w_idx_0 =
        fmaxf(TJACMB_LnQualTurnOnTime_C_sec, LCFRCV_TSysCycleTimeSen_sec);

    /* Switch: '<S45>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S45>/Max'
     *  Sum: '<S45>/Subtract'
     *  Switch: '<S45>/Switch1'
     *  UnaryMinus: '<S45>/Unary Minus'
     *  UnitDelay: '<S45>/Unit Delay'
     */
    if (rtb_CMB_OFOLnLengthInvalid_bool) {
        CMB_LnQualTurnOnDelay_sec[0] =
            fmaxf(CMB_LnQualTurnOnDelay_sec[0], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        CMB_LnQualTurnOnDelay_sec[0] = rtb_Subtract_if0w_idx_0;
    }

    /* Switch: '<S43>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S16>/NOT2'
     *  Logic: '<S45>/AND'
     *  RelationalOperator: '<S45>/LessThanOrEqual'
     *  Switch: '<S43>/Switch1'
     *  Switch: '<S45>/Switch'
     *  UnaryMinus: '<S45>/Unary Minus1'
     *  UnitDelay: '<S43>/Unit Delay'
     *  UnitDelay: '<S45>/Unit Delay'
     */
    CMB_LnQualRSFF_bool[0] =
        (rtb_OR_imzg_idx_0 &&
         ((rtb_CMB_OFOLnLengthInvalid_bool &&
           (CMB_LnQualTurnOnDelay_sec[0] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (CMB_LnQualRSFF_bool[0])));

    /* Switch: '<S45>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S45>/Max'
     *  Sum: '<S45>/Subtract'
     *  Switch: '<S45>/Switch1'
     *  UnaryMinus: '<S45>/Unary Minus'
     *  UnitDelay: '<S45>/Unit Delay'
     */
    if (rtb_Equal1_p2mp) {
        CMB_LnQualTurnOnDelay_sec[1] =
            fmaxf(CMB_LnQualTurnOnDelay_sec[1], -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        CMB_LnQualTurnOnDelay_sec[1] = rtb_Subtract_if0w_idx_0;
    }

    /* Switch: '<S43>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S16>/NOT2'
     *  Logic: '<S45>/AND'
     *  RelationalOperator: '<S45>/LessThanOrEqual'
     *  Switch: '<S43>/Switch1'
     *  Switch: '<S45>/Switch'
     *  UnaryMinus: '<S45>/Unary Minus1'
     *  UnitDelay: '<S43>/Unit Delay'
     *  UnitDelay: '<S45>/Unit Delay'
     */
    CMB_LnQualRSFF_bool[1] =
        (rtb_OR_imzg_idx_1 &&
         ((rtb_Equal1_p2mp &&
           (CMB_LnQualTurnOnDelay_sec[1] <= (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (CMB_LnQualRSFF_bool[1])));

    /* Logic: '<S15>/NOT7' incorporates:
     *  Logic: '<S16>/NOT1'
     *  UnitDelay: '<S43>/Unit Delay'
     */
    rtb_CMB_OFOObjLengthInvalid_boo = !CMB_LnQualRSFF_bool[0];
    rtb_VectorConcatenate_maf5[7] = rtb_CMB_OFOObjLengthInvalid_boo;

    /* Logic: '<S15>/NOT8' incorporates:
     *  Logic: '<S16>/NOT'
     *  UnitDelay: '<S43>/Unit Delay'
     */
    rtb_CMB_OFOVelocityInvalid_bool = !CMB_LnQualRSFF_bool[1];
    rtb_VectorConcatenate_maf5[9] = rtb_CMB_OFOVelocityInvalid_bool;

    /* Switch: '<S16>/Switch' incorporates:
     *  Constant: '<S16>/Constant5'
     */
    if (TJACMB_LnQualBothLanes_C_bool) {
        /* Switch: '<S16>/Switch' incorporates:
         *  Logic: '<S16>/AND'
         */
        CMB_LaneQualityInvalid_bool = (rtb_CMB_OFOObjLengthInvalid_boo &&
                                       rtb_CMB_OFOVelocityInvalid_bool);
    } else {
        /* Switch: '<S16>/Switch' incorporates:
         *  Logic: '<S16>/OR'
         */
        CMB_LaneQualityInvalid_bool = (rtb_CMB_OFOObjLengthInvalid_boo ||
                                       rtb_CMB_OFOVelocityInvalid_bool);
    }

    /* End of Switch: '<S16>/Switch' */

    /* RelationalOperator: '<S17>/Less Than' incorporates:
     *  Constant: '<S17>/Constant1'
     *  Inport: '<Root>/Inport39'
     */
    rtb_CMB_OFOObjLengthInvalid_boo =
        (ODPFOH_TgtObjLength_met < TJACMB_TgtObjLengthMaxOFO_C_met);

    /* RelationalOperator: '<S17>/Less Than1' incorporates:
     *  Constant: '<S17>/Constant2'
     *  Inport: '<Root>/Inport94'
     */
    rtb_CMB_OFOVelocityInvalid_bool =
        (VDy_DashboardVelocity_kph < TJACMB_VelXMaxOFO_C_kph);

    /* RelationalOperator: '<S17>/Less Than2' incorporates:
     *  Constant: '<S17>/Constant3'
     *  Inport: '<Root>/Inport21'
     */
    rtb_CMB_OFOLnLengthInvalid_bool =
        (ABPR_CntrLnClthLength_met < TJACMB_LnLengthMaxOFO_C_met);

    /* Logic: '<S17>/AND' incorporates:
     *  Constant: '<S17>/Constant4'
     */
    CMB_ObjectFollowingOnly_bool = (((rtb_CMB_OFOObjLengthInvalid_boo &&
                                      rtb_CMB_OFOVelocityInvalid_bool) &&
                                     rtb_CMB_OFOLnLengthInvalid_bool) &&
                                    (TJACMB_EnableOFO_C_bool));

    /* Logic: '<S18>/AND' incorporates:
     *  Constant: '<S18>/Constant1'
     *  Logic: '<S18>/OR'
     */
    TJACMB_ObjectCorridor_bool =
        ((TJACMB_CombinedDataEnable_C_bool) &&
         ((CMB_ObjectFollowingOnly_bool) || (CMB_LaneQualityInvalid_bool)));

    /* Switch: '<S18>/Switch' incorporates:
     *  Constant: '<S18>/Constant'
     *  Constant: '<S18>/Constant2'
     */
    if (TJACMB_ObjectCorridor_bool) {
        rtb_Switch_ithu = 0.0F;
    } else {
        rtb_Switch_ithu = TJACMB_LnWeightPosY0_C_nu;
    }

    /* End of Switch: '<S18>/Switch' */

    /* Sum: '<S22>/Add' incorporates:
     *  Inport: '<Root>/Inport3'
     *  Inport: '<Root>/Inport36'
     *  Product: '<S22>/Product'
     *  Sum: '<S22>/Subtract'
     */
    TJACMB_CombinedPosY0_met =
        ODPFOH_TgtObjPosY0_met +
        ((ABPR_CntrLnClthPosY0_met - ODPFOH_TgtObjPosY0_met) * rtb_Switch_ithu);

    /* Logic: '<S193>/AND' incorporates:
     *  Constant: '<S193>/Constant'
     *  Constant: '<S203>/Constant3'
     *  Inport: '<Root>/Inport76'
     *  Logic: '<S203>/AND4'
     *  RelationalOperator: '<S193>/GreaterThanOrEqual'
     */
    OF_ObjectDangerLeftRear_bool =
        ((rtb_OR_idx_0 && (OF_CheckRearObjects_C_bool)) &&
         (VDy_VehCrv_1pm >= TJAOBF_EgoCurveMaxSideCollision_C_1pm));

    /* Logic: '<S193>/AND1' incorporates:
     *  Constant: '<S193>/Constant1'
     *  Constant: '<S203>/Constant3'
     *  Inport: '<Root>/Inport76'
     *  Logic: '<S203>/AND5'
     *  RelationalOperator: '<S193>/LessThanOrEqual'
     *  UnaryMinus: '<S193>/Unary Minus'
     */
    OF_ObjectDangerRightRear_bool =
        ((VDy_VehCrv_1pm <= (-TJAOBF_EgoCurveMaxSideCollision_C_1pm)) &&
         ((OF_CheckRearObjects_C_bool) && rtb_OR_idx_1));

    /* Logic: '<S193>/OR' */
    rtb_OR_idx_0 =
        ((OF_ObjectDangerLeftRear_bool) || (OF_ObjectDangerRightRear_bool));

    /* Switch: '<S204>/Switch' incorporates:
     *  Constant: '<S193>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S204>/Max'
     *  Sum: '<S204>/Subtract'
     *  Switch: '<S204>/Switch1'
     *  UnaryMinus: '<S204>/Unary Minus'
     *  UnitDelay: '<S204>/Unit Delay'
     */
    if (rtb_OR_idx_0) {
        OF_RearObjDecCrvtTurnOffDelay_sec = OF_MinDurFreeSideCollision_C_sec;
    } else {
        OF_RearObjDecCrvtTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  OF_RearObjDecCrvtTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S204>/Switch' */

    /* Logic: '<S193>/NOT' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S204>/OR'
     *  RelationalOperator: '<S204>/GreaterThan'
     *  UnaryMinus: '<S204>/Unary Minus1'
     *  UnitDelay: '<S204>/Unit Delay'
     */
    OF_NoObjectCollision_bool =
        ((!rtb_OR_idx_0) &&
         (OF_RearObjDecCrvtTurnOffDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S182>/AND1' incorporates:
     *  Logic: '<S182>/OR'
     */
    TJAOBF_StrongReady_bool =
        (((((((TJAOBF_ObjLaneValidDuration_bool &&
               (OBF_TargetObjDataSR_bool)) &&
              (rtb_OBF_VelocityValid_bool || rtb_Equal2_kbd5)) &&
             rtb_AND_nxkg) &&
            rtb_AND_gpqq) &&
           rtb_AND_pjo1) &&
          rtb_AND_cvae) &&
         (OF_NoObjectCollision_bool));

    /* Logic: '<S182>/AND2' */
    TJAOBF_WeakReady_bool =
        (((OBF_TargetObjDataWR_bool) && rtb_GEN_WR_Custom_bool) &&
         rtb_LessThan_ko2v);

    /* Switch: '<S267>/Switch2' incorporates:
     *  Constant: '<S267>/Constant'
     *  Inport: '<Root>/Inport97'
     */
    TJASLC_Nb_DCLCSwitchNVRAM_nu =
        ((!TJASLC_EnableSLCHMISwitch_nu) || LCFRCV_SLCHMISwitch_bool);

    /* Logic: '<S463>/AND1' */
    SLC_StrongReadyBothSides_bool =
        (((TJASLC_Nb_DCLCSwitchNVRAM_nu && rtb_GEN_DrvNotBuckledUp_bool) &&
          rtb_NotEqual3_lcsh) &&
         rtb_GEN_BlockTimerExpired_bool);

    /* Logic: '<S462>/AND2' */
    rtb_OR_idx_1 = ((SLC_StrongReadyBothSides_bool) && rtb_Equal_i3l2_idx_0);

    /* Switch: '<S360>/Switch' incorporates:
     *  Constant: '<S344>/Constant'
     *  Constant: '<S346>/Constant'
     *  Constant: '<S348>/Constant'
     *  Constant: '<S349>/Constant'
     *  Constant: '<S352>/Constant'
     *  Constant: '<S353>/Constant'
     *  Constant: '<S354>/Constant'
     *  Constant: '<S359>/Constant'
     *  Constant: '<S360>/Constant2'
     *  Logic: '<S315>/OR'
     *  Logic: '<S315>/OR1'
     *  Logic: '<S315>/OR2'
     *  Logic: '<S315>/OR3'
     *  Logic: '<S315>/OR4'
     *  Logic: '<S315>/OR5'
     *  RelationalOperator: '<S315>/NotEqual'
     *  RelationalOperator: '<S315>/NotEqual1'
     *  RelationalOperator: '<S315>/NotEqual10'
     *  RelationalOperator: '<S315>/NotEqual12'
     *  RelationalOperator: '<S315>/NotEqual13'
     *  RelationalOperator: '<S315>/NotEqual2'
     *  RelationalOperator: '<S315>/NotEqual3'
     *  RelationalOperator: '<S315>/NotEqual8'
     *  UnitDelay: '<S360>/Unit Delay'
     */
    if ((!SLC_TriggerLeft_bool) &&
        (((((E_TJASLC_ManeuverState_nu_LCPSTART !=
             ((uint32_T)SLC_PrevManeuverStateGRCO_Enum)) &&
            (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
             E_TJASLC_ManeuverState_nu_LATMVSTART)) &&
           (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
            E_TJASLC_ManeuverState_nu_LCMSTART)) &&
          (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
           E_TJASLC_ManeuverState_nu_NEWEGO)) &&
         (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
          E_TJASLC_ManeuverState_nu_LCMEND))) {
        SLC_GRCOLeftRSFF_bool = false;
    } else {
        SLC_GRCOLeftRSFF_bool =
            (((((E_TJASLC_ManeuverState_nu_PASSIVE ==
                 ((uint32_T)SLC_PrevManeuverStateGRCO_Enum)) ||
                (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                 E_TJASLC_ManeuverState_nu_TRIGREADY)) ||
               (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                E_TJASLC_ManeuverState_nu_LCPSTART)) &&
              (SLC_TriggerLeft_bool)) ||
             (SLC_GRCOLeftRSFF_bool));
    }

    /* End of Switch: '<S360>/Switch' */

    /* Logic: '<S462>/AND3' */
    rtb_AND_pjo1 = ((SLC_StrongReadyBothSides_bool) && rtb_Equal_i3l2_idx_1);

    /* Switch: '<S361>/Switch' incorporates:
     *  Constant: '<S345>/Constant'
     *  Constant: '<S347>/Constant'
     *  Constant: '<S350>/Constant'
     *  Constant: '<S351>/Constant'
     *  Constant: '<S355>/Constant'
     *  Constant: '<S356>/Constant'
     *  Constant: '<S357>/Constant'
     *  Constant: '<S358>/Constant'
     *  Constant: '<S361>/Constant2'
     *  Logic: '<S315>/OR10'
     *  Logic: '<S315>/OR11'
     *  Logic: '<S315>/OR6'
     *  Logic: '<S315>/OR7'
     *  Logic: '<S315>/OR8'
     *  Logic: '<S315>/OR9'
     *  RelationalOperator: '<S315>/NotEqual11'
     *  RelationalOperator: '<S315>/NotEqual14'
     *  RelationalOperator: '<S315>/NotEqual15'
     *  RelationalOperator: '<S315>/NotEqual4'
     *  RelationalOperator: '<S315>/NotEqual5'
     *  RelationalOperator: '<S315>/NotEqual6'
     *  RelationalOperator: '<S315>/NotEqual7'
     *  RelationalOperator: '<S315>/NotEqual9'
     *  UnitDelay: '<S361>/Unit Delay'
     */
    if ((!SLC_TriggerRight_bool) &&
        (((((E_TJASLC_ManeuverState_nu_LCPSTART !=
             ((uint32_T)SLC_PrevManeuverStateGRCO_Enum)) &&
            (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
             E_TJASLC_ManeuverState_nu_LATMVSTART)) &&
           (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
            E_TJASLC_ManeuverState_nu_LCMSTART)) &&
          (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
           E_TJASLC_ManeuverState_nu_NEWEGO)) &&
         (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
          E_TJASLC_ManeuverState_nu_LCMEND))) {
        SLC_GRCORightRSFF_bool = false;
    } else {
        SLC_GRCORightRSFF_bool =
            (((((E_TJASLC_ManeuverState_nu_PASSIVE ==
                 ((uint32_T)SLC_PrevManeuverStateGRCO_Enum)) ||
                (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                 E_TJASLC_ManeuverState_nu_TRIGREADY)) ||
               (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                E_TJASLC_ManeuverState_nu_LCPSTART)) &&
              (SLC_TriggerRight_bool)) ||
             (SLC_GRCORightRSFF_bool));
    }

    /* End of Switch: '<S361>/Switch' */

    /* Logic: '<S463>/AND4' incorporates:
     *  Inport: '<Root>/Inport121'
     *  Logic: '<S54>/NOT5'
     */
    rtb_AND_cvae = !EMFID_bFCA_bool;

    /* Logic: '<S463>/AND2' incorporates:
     *  Logic: '<S463>/AND3'
     *  Logic: '<S463>/AND4'
     *  UnitDelay: '<S7>/Unit Delay'
     */
    SLC_WeakReadyBothSides_bool =
        (((((((rtb_GEN_Cancel_Custom_bool && rtb_GEN_MaxSteerAngleExceeded_b) &&
              rtb_SLC_PrevDriverTriggerResetR) &&
             rtb_OR2_pwtt) &&
            rtb_SLC_VelocityValid_bool) &&
           rtb_NotEqual1_cuxj) &&
          (!SLC_PrevTakeoverValidUnitDelay_bool)) &&
         rtb_AND_cvae);

    /* Logic: '<S464>/NOT' */
    rtb_OR_baue = !rtb_OR_baue;

    /* Logic: '<S464>/AND' */
    SLC_WeakReadyLeft_bool = ((((rtb_GreaterThan_bpjw_idx_0 && rtb_OR_ly14) &&
                                rtb_SLC_LeAdjLaneWidthValid_boo) &&
                               rtb_SLC_LeAdjLaneValid_bool) &&
                              rtb_OR_baue);

    /* Logic: '<S462>/AND' */
    rtb_SLC_LeAdjLaneWidthValid_boo =
        ((SLC_WeakReadyBothSides_bool) && (SLC_WeakReadyLeft_bool));

    /* Logic: '<S465>/NOT' */
    rtb_CMB_VelocityValid_bool = !rtb_CMB_VelocityValid_bool;

    /* Logic: '<S465>/AND' */
    SLC_WeakReadyRight_bool =
        ((((rtb_GreaterThan_bpjw_idx_1 && rtb_AND1_h3xh) &&
           rtb_SLC_RiAdjLaneWidthValid_boo) &&
          rtb_SLC_RiAdjLaneValid_bool) &&
         rtb_CMB_VelocityValid_bool);

    /* Logic: '<S462>/AND1' */
    rtb_OR2_pwtt = ((SLC_WeakReadyBothSides_bool) && (SLC_WeakReadyRight_bool));

    /* Switch: '<S315>/Switch' incorporates:
     *  Switch: '<S315>/Switch2'
     *  UnitDelay: '<S360>/Unit Delay'
     */
    if (SLC_GRCOLeftRSFF_bool) {
        /* Switch: '<S315>/Switch' */
        TJASLC_StrongReady_bool = rtb_OR_idx_1;

        /* Switch: '<S315>/Switch2' */
        TJASLC_WeakReady_bool = rtb_SLC_LeAdjLaneWidthValid_boo;
    } else {
        /* Switch: '<S315>/Switch' incorporates:
         *  Switch: '<S315>/Switch1'
         *  UnitDelay: '<S361>/Unit Delay'
         */
        TJASLC_StrongReady_bool = ((SLC_GRCORightRSFF_bool) && rtb_AND_pjo1);

        /* Switch: '<S315>/Switch2' incorporates:
         *  Switch: '<S315>/Switch3'
         *  UnitDelay: '<S361>/Unit Delay'
         */
        TJASLC_WeakReady_bool = ((SLC_GRCORightRSFF_bool) && rtb_OR2_pwtt);
    }

    /* End of Switch: '<S315>/Switch' */

    /* SignalConversion: '<S315>/Signal Conversion' */
    TJASLC_Cancel_bool = SLC_Cancel_bool;

    /* Switch: '<S39>/Switch1' incorporates:
     *  Constant: '<S39>/Constant6'
     *  Constant: '<S39>/Constant7'
     *  Constant: '<S39>/Constant8'
     *  Constant: '<S39>/Constant9'
     *  MinMax: '<S39>/Min'
     */
    if (TJACMB_IndVelLimitsEnable_C_bool) {
        rtb_Subtract_if0w_idx_0 = TJACMB_VelXMax_C_kph;
    } else {
        rtb_Subtract_if0w_idx_0 =
            fminf(TJAOBF_VelXMax_C_kph, TJALKA_VelXMax_C_kph);
    }

    /* End of Switch: '<S39>/Switch1' */

    /* Switch: '<S41>/Switch' incorporates:
     *  Constant: '<S39>/Constant2'
     *  Constant: '<S41>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S41>/Less Than'
     *  RelationalOperator: '<S41>/Less Than1'
     *  Sum: '<S39>/Add'
     *  UnitDelay: '<S41>/Unit Delay'
     */
    if ((rtb_Subtract_if0w_idx_0 + TJALKA_VelXHyst_C_kph) <
        VDy_DashboardVelocity_kph) {
        CMB_VelXMaxHyst_bool = true;
    } else {
        CMB_VelXMaxHyst_bool =
            ((VDy_DashboardVelocity_kph >= rtb_Subtract_if0w_idx_0) &&
             (CMB_VelXMaxHyst_bool));
    }

    /* End of Switch: '<S41>/Switch' */

    /* Switch: '<S39>/Switch' incorporates:
     *  Constant: '<S39>/Constant1'
     *  Constant: '<S39>/Constant3'
     *  Constant: '<S39>/Constant4'
     *  Constant: '<S39>/Constant5'
     *  MinMax: '<S39>/Max'
     */
    if (TJACMB_IndVelLimitsEnable_C_bool) {
        rtb_Subtract_if0w_idx_0 = TJACMB_VelXMin_C_kph;
    } else {
        rtb_Subtract_if0w_idx_0 =
            fmaxf(TJAOBF_VelXMin_C_kph, TJALKA_VelXMin_C_kph);
    }

    /* End of Switch: '<S39>/Switch' */

    /* Switch: '<S42>/Switch' incorporates:
     *  Constant: '<S39>/Constant10'
     *  Constant: '<S42>/Constant'
     *  Inport: '<Root>/Inport94'
     *  RelationalOperator: '<S42>/Less Than'
     *  RelationalOperator: '<S42>/Less Than1'
     *  Sum: '<S39>/Subtract'
     *  UnitDelay: '<S42>/Unit Delay'
     */
    if (rtb_Subtract_if0w_idx_0 < VDy_DashboardVelocity_kph) {
        CMB_VelXMinHyst_bool = true;
    } else {
        CMB_VelXMinHyst_bool =
            ((VDy_DashboardVelocity_kph >=
              (rtb_Subtract_if0w_idx_0 - TJALKA_VelXHyst_C_kph)) &&
             (CMB_VelXMinHyst_bool));
    }

    /* End of Switch: '<S42>/Switch' */

    /* Logic: '<S39>/AND' incorporates:
     *  Logic: '<S39>/NOT'
     *  UnitDelay: '<S41>/Unit Delay'
     *  UnitDelay: '<S42>/Unit Delay'
     */
    rtb_CMB_VelocityValid_bool =
        ((!CMB_VelXMaxHyst_bool) && (CMB_VelXMinHyst_bool));

    /* Logic: '<S15>/AND' incorporates:
     *  Constant: '<S15>/Constant1'
     */
    rtb_AND1_h3xh = ((TJACMB_CMB_Enabled_C_bool) && rtb_TJASTM_ACCIsOFF_bool);

    /* Logic: '<S15>/AND1' */
    TJACMB_StrongReady_bool =
        (((TJALKA_StrongReady_bool && TJAOBF_StrongReady_bool) &&
          rtb_CMB_VelocityValid_bool) &&
         rtb_AND1_h3xh);

    /* Logic: '<S15>/AND2' */
    TJACMB_WeakReady_bool = (TJALKA_WeakReady_bool && TJAOBF_WeakReady_bool);

    /* Logic: '<S48>/OR1' incorporates:
     *  Inport: '<Root>/Inport98'
     */
    TJAGEN_FunctionSwitch_bool =
        ((rtb_GEN_BrakePadelInvalid__gzuo || rtb_GEN_VehStInvalid_bool) ||
         LCFRCV_TJAAudioSwitch_bool);

    /* Logic: '<S48>/OR3' incorporates:
     *  Constant: '<S48>/Constant3'
     *  Constant: '<S48>/Constant4'
     *  Inport: '<Root>/Inport119'
     *  Logic: '<S48>/OR2'
     */
    TJAGEN_CodeFunction_bool =
        (((TJAGEN_LKA_Available_C_bool) || (TJAGEN_TJA_Available_C_bool)) &&
         LCFRCV_bPilotOnOff_bool);

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S51>/Constant2'
     */
    if (TJAGEN_CheckTJAErrorState_C_bool) {
        /* Switch: '<S51>/Switch' incorporates:
         *  Constant: '<S51>/Constant'
         *  Constant: '<S51>/Constant1'
         *  Inport: '<Root>/Inport67'
         *  Inport: '<Root>/Inport68'
         *  Logic: '<S51>/AND'
         *  Logic: '<S51>/AND1'
         *  Logic: '<S51>/OR'
         */
        TJAGEN_Error_bool =
            (((TJAGEN_TJA_Available_C_bool) && LCFRCV_ErrorStateTJA_bool) ||
             (LCFRCV_ErrorStateLKA_bool && (TJAGEN_LKA_Available_C_bool)));
    } else {
        /* Switch: '<S51>/Switch' incorporates:
         *  Constant: '<S51>/Constant3'
         */
        TJAGEN_Error_bool = false;
    }

    /* End of Switch: '<S51>/Switch' */

    /* SignalConversion: '<S514>/Signal Conversion' */
    rtb_VectorConcatenate_aisn[0] = SLC_TriggerLeft_bool;

    /* SignalConversion: '<S514>/Signal Conversion1' */
    rtb_VectorConcatenate_aisn[1] = SLC_TriggerRight_bool;

    /* SignalConversion: '<S514>/Signal Conversion2' */
    rtb_VectorConcatenate_aisn[2] = rtb_LKA_TakeOverValid_bool;

    /* SignalConversion: '<S514>/Signal Conversion3' */
    rtb_VectorConcatenate_aisn[3] = rtb_SLC_DriverTriggerResetRight;

    /* SignalConversion: '<S514>/Signal Conversion4' */
    rtb_VectorConcatenate_aisn[4] = SLC_LeverLeftEngaged_bool;

    /* SignalConversion: '<S514>/Signal Conversion5' */
    rtb_VectorConcatenate_aisn[5] = SLC_LeverRightEngaged_bool;

    /* SignalConversion: '<S514>/Signal Conversion6' */
    rtb_VectorConcatenate_aisn[6] = rtb_SLC_LeTurnSignalOn_bool;

    /* SignalConversion: '<S514>/Signal Conversion7' */
    rtb_VectorConcatenate_aisn[7] = rtb_SLC_RiTurnSignalOn_bool;

    /* SignalConversion: '<S514>/Signal Conversion8' */
    rtb_VectorConcatenate_aisn[8] = SLC_PrevReset_bool;

    /* SignalConversion: '<S514>/Signal Conversion9' incorporates:
     *  Inport: '<Root>/Inport19'
     *  Logic: '<S513>/NOT2'
     */
    rtb_VectorConcatenate_aisn[9] = !LCFRCV_TurnSignalLeverHold_bool;

    /* SignalConversion: '<S514>/Signal Conversion10' incorporates:
     *  Constant: '<S513>/Constant1'
     *  Logic: '<S513>/NOT3'
     */
    rtb_VectorConcatenate_aisn[10] = !TJASLC_ManualTrigger_C_bool;

    /* SignalConversion: '<S514>/Signal Conversion11' incorporates:
     *  UnitDelay: '<S268>/Unit Delay1'
     */
    rtb_VectorConcatenate_aisn[11] = SLC_PrevDriverTrigResetLeftUnitDelay_bool;

    /* SignalConversion: '<S514>/Signal Conversion12' incorporates:
     *  UnitDelay: '<S268>/Unit Delay2'
     */
    rtb_VectorConcatenate_aisn[12] = SLC_PrevDriverTrigResetRightUnitDelay_bool;

    /* Logic: '<S527>/OR' */
    rtb_OR_baue = ((SLC_LeverLeftEngaged_bool) || (SLC_LeverRightEngaged_bool));

    /* Switch: '<S535>/Switch' incorporates:
     *  Constant: '<S527>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S535>/Max'
     *  Sum: '<S535>/Subtract'
     *  Switch: '<S535>/Switch1'
     *  UnaryMinus: '<S535>/Unary Minus'
     *  UnitDelay: '<S535>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LeverEngagedTurnOnDelay_sec = fmaxf(SLC_LeverEngagedTurnOnDelay_sec,
                                                -LCFRCV_TSysCycleTimeSen_sec) -
                                          LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LeverEngagedTurnOnDelay_sec = TJASLC_LCPInitDurationMax_C_sec;
    }

    /* End of Switch: '<S535>/Switch' */

    /* Logic: '<S527>/AND' incorporates:
     *  Constant: '<S532>/Constant'
     *  Constant: '<S533>/Constant'
     *  Constant: '<S534>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S527>/OR1'
     *  Logic: '<S535>/AND'
     *  RelationalOperator: '<S527>/Equal'
     *  RelationalOperator: '<S527>/Equal1'
     *  RelationalOperator: '<S527>/Equal2'
     *  RelationalOperator: '<S535>/LessThanOrEqual'
     *  UnaryMinus: '<S535>/Unary Minus1'
     *  UnitDelay: '<S535>/Unit Delay'
     */
    SLC_MaxInitDurationExceeded_bool =
        ((rtb_OR_baue && (SLC_LeverEngagedTurnOnDelay_sec <=
                          (-LCFRCV_TSysCycleTimeSen_sec))) &&
         (((((uint32_T)SLC_PrevManeuverState_Enum) ==
            E_TJASLC_ManeuverState_nu_PASSIVE) ||
           (((uint32_T)SLC_PrevManeuverState_Enum) ==
            E_TJASLC_ManeuverState_nu_TRIGREADY)) ||
          (((uint32_T)SLC_PrevManeuverState_Enum) ==
           E_TJASLC_ManeuverState_nu_LCPSTART)));

    /* SignalConversion: '<S514>/Signal Conversion14' */
    rtb_VectorConcatenate_aisn[13] = SLC_MaxInitDurationExceeded_bool;

    /* Logic: '<S526>/AND' incorporates:
     *  Constant: '<S529>/Constant'
     *  Constant: '<S530>/Constant'
     *  Constant: '<S531>/Constant'
     *  Logic: '<S526>/AND2'
     *  RelationalOperator: '<S526>/Equal'
     *  RelationalOperator: '<S526>/NotEqual'
     *  RelationalOperator: '<S526>/NotEqual1'
     */
    SLC_ManvStatePassive_bool = (((E_TJASLC_ManeuverState_nu_PASSIVE !=
                                   ((uint32_T)SLC_PrevManeuverState2_Enum)) &&
                                  (((uint32_T)SLC_PrevManeuverState2_Enum) !=
                                   E_TJASLC_ManeuverState_nu_TRIGREADY)) &&
                                 (E_TJASLC_ManeuverState_nu_PASSIVE ==
                                  ((uint32_T)SLC_PrevManeuverState_Enum)));

    /* SignalConversion: '<S514>/Signal Conversion13' */
    rtb_VectorConcatenate_aisn[14] = SLC_ManvStatePassive_bool;

    /* SignalConversion generated from: '<S524>/Vector Concatenate' */
    rtb_VectorConcatenate_aisn[15] = false;

    /* DataTypeConversion: '<S367>/Data Type Conversion' incorporates:
     *  UnitDelay: '<S367>/Unit Delay'
     */
    rtb_DataTypeConversion =
        (int8_T)floorf(SLC_PrevFtWhDit2BdSignUnitDelay_met);

    /* Switch: '<S368>/Switch' incorporates:
     *  Constant: '<S364>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport11'
     *  MinMax: '<S368>/Max'
     *  Sum: '<S368>/Subtract'
     *  Switch: '<S368>/Switch1'
     *  UnaryMinus: '<S368>/Unary Minus'
     *  UnitDelay: '<S368>/Unit Delay'
     */
    if (ABPR_LaneChangeDetected_bool) {
        SLC_LaneChngDetectedTurnOffDelay_sec = TJASLC_LnChngFlagTurnOffTm_C_sec;
    } else {
        SLC_LaneChngDetectedTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_LaneChngDetectedTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S368>/Switch' */

    /* Logic: '<S364>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S364>/NOT'
     *  Logic: '<S368>/OR'
     *  RelationalOperator: '<S368>/GreaterThan'
     *  UnaryMinus: '<S368>/Unary Minus1'
     *  UnitDelay: '<S368>/Unit Delay'
     */
    SLC_LaneCheckValid_bool =
        (rtb_NotEqual3_lcsh &&
         (rtb_AND_jt3t && (SLC_LaneChngDetectedTurnOffDelay_sec <=
                           (-LCFRCV_TSysCycleTimeSen_sec))));

    /* Switch: '<S369>/Switch' incorporates:
     *  Logic: '<S365>/AND'
     */
    if (rtb_NotEqual_ozi5_tmp_0 && rtb_NotEqual_ozi5_tmp) {
        /* Switch: '<S369>/Switch' incorporates:
         *  Constant: '<S369>/Constant2'
         */
        SLC_NewEgoLane_bool = false;
    } else {
        /* Switch: '<S369>/Switch' incorporates:
         *  Inport: '<Root>/Inport11'
         *  UnitDelay: '<S369>/Unit Delay'
         */
        SLC_NewEgoLane_bool =
            (ABPR_LaneChangeDetected_bool || (SLC_NewEgoLaneRSFF_bool));
    }

    /* End of Switch: '<S369>/Switch' */

    /* Trigonometry: '<S362>/Cos' incorporates:
     *  Inport: '<Root>/Inport4'
     *  Trigonometry: '<S363>/Cos'
     */
    rtb_Subtract_if0w_idx_0 = cosf(ABPR_CntrLnClthHeading_rad);

    /* Product: '<S362>/Product2' incorporates:
     *  Constant: '<S362>/Constant1'
     *  Constant: '<S362>/Constant3'
     *  Product: '<S362>/Product3'
     *  Product: '<S363>/Product2'
     *  Trigonometry: '<S362>/Cos'
     */
    rtb_Subtract_if0w_idx_1 =
        rtb_Subtract_if0w_idx_0 * (0.5F * TJAPARAM_VEH_Width_C_met);

    /* Switch: '<S362>/Switch4' incorporates:
     *  Logic: '<S362>/NOT'
     */
    if (SLC_LaneCheckValid_bool) {
        /* Switch: '<S362>/Switch2' incorporates:
         *  Switch: '<S362>/Switch3'
         */
        if (SLC_LeverLeftEngaged_bool) {
            /* Switch: '<S362>/Switch' incorporates:
             *  Inport: '<Root>/Inport1'
             *  Inport: '<Root>/Inport2'
             */
            if (SLC_NewEgoLane_bool) {
                rtb_Switch_ithu = ABPR_RiLnClthPosY0_met;
            } else {
                rtb_Switch_ithu = ABPR_LeLnClthPosY0_met;
            }

            /* End of Switch: '<S362>/Switch' */

            /* UnitDelay: '<S362>/Unit Delay' incorporates:
             *  Product: '<S362>/Product'
             *  Product: '<S362>/Product2'
             *  Sum: '<S362>/Subtract'
             *  Trigonometry: '<S362>/Cos'
             */
            SLC_PrevFrontWheelDist2BoundUnitDelay_met =
                (rtb_Switch_ithu * rtb_Subtract_if0w_idx_0) -
                rtb_Subtract_if0w_idx_1;
        } else if (SLC_LeverRightEngaged_bool) {
            /* Switch: '<S362>/Switch1' incorporates:
             *  Inport: '<Root>/Inport1'
             *  Inport: '<Root>/Inport2'
             *  Switch: '<S362>/Switch3'
             */
            if (SLC_NewEgoLane_bool) {
                rtb_Switch_ithu = ABPR_LeLnClthPosY0_met;
            } else {
                rtb_Switch_ithu = ABPR_RiLnClthPosY0_met;
            }

            /* End of Switch: '<S362>/Switch1' */

            /* UnitDelay: '<S362>/Unit Delay' incorporates:
             *  Product: '<S362>/Product1'
             *  Product: '<S362>/Product2'
             *  Sum: '<S362>/Add'
             *  Switch: '<S362>/Switch3'
             *  Trigonometry: '<S362>/Cos'
             */
            SLC_PrevFrontWheelDist2BoundUnitDelay_met =
                (rtb_Subtract_if0w_idx_0 * rtb_Switch_ithu) +
                rtb_Subtract_if0w_idx_1;
        } else {
            /* UnitDelay: '<S362>/Unit Delay' incorporates:
             *  Constant: '<S362>/Constant2'
             *  Switch: '<S362>/Switch3'
             */
            SLC_PrevFrontWheelDist2BoundUnitDelay_met = 0.0F;
        }

        /* End of Switch: '<S362>/Switch2' */
    }

    /* End of Switch: '<S362>/Switch4' */

    /* Signum: '<S367>/Sign' incorporates:
     *  UnitDelay: '<S362>/Unit Delay'
     */
    if (SLC_PrevFrontWheelDist2BoundUnitDelay_met < 0.0F) {
        SLC_PrevFtWhDit2BdSignUnitDelay_met = -1.0F;
    } else if (SLC_PrevFrontWheelDist2BoundUnitDelay_met > 0.0F) {
        SLC_PrevFtWhDit2BdSignUnitDelay_met = 1.0F;
    } else {
        SLC_PrevFtWhDit2BdSignUnitDelay_met =
            SLC_PrevFrontWheelDist2BoundUnitDelay_met;
    }

    /* End of Signum: '<S367>/Sign' */

    /* DataTypeConversion: '<S367>/Data Type Conversion1' incorporates:
     *  UnitDelay: '<S367>/Unit Delay'
     */
    rtb_DataTypeConversion1_duje =
        (int8_T)floorf(SLC_PrevFtWhDit2BdSignUnitDelay_met);

    /* Logic: '<S367>/NOT' incorporates:
     *  Constant: '<S367>/Constant1'
     *  Constant: '<S367>/Constant2'
     *  RelationalOperator: '<S367>/NotEqual'
     *  RelationalOperator: '<S367>/NotEqual1'
     *  RelationalOperator: '<S367>/NotEqual2'
     */
    SLC_LCM_Start_bool =
        (((0 != rtb_DataTypeConversion) &&
          (rtb_DataTypeConversion != rtb_DataTypeConversion1_duje)) &&
         (rtb_DataTypeConversion1_duje != 0));

    /* DataTypeConversion: '<S366>/Data Type Conversion' incorporates:
     *  UnitDelay: '<S366>/Unit Delay'
     */
    rtb_DataTypeConversion =
        (int8_T)floorf(SLC_PrevReWhDit2BdSignUnitDelay_met);

    /* Product: '<S363>/Product4' incorporates:
     *  Constant: '<S363>/Constant4'
     *  Inport: '<Root>/Inport4'
     *  Trigonometry: '<S363>/Sin'
     */
    rtb_Add_fbcu =
        sinf(ABPR_CntrLnClthHeading_rad) * TJAPARAM_VEH_Wheelbase_C_met;

    /* Switch: '<S363>/Switch4' incorporates:
     *  Logic: '<S363>/NOT'
     */
    if (SLC_LaneCheckValid_bool) {
        /* Switch: '<S363>/Switch2' incorporates:
         *  Switch: '<S363>/Switch3'
         */
        if (SLC_LeverLeftEngaged_bool) {
            /* Switch: '<S363>/Switch' incorporates:
             *  Inport: '<Root>/Inport1'
             *  Inport: '<Root>/Inport2'
             */
            if (SLC_NewEgoLane_bool) {
                rtb_Switch_ithu = ABPR_RiLnClthPosY0_met;
            } else {
                rtb_Switch_ithu = ABPR_LeLnClthPosY0_met;
            }

            /* End of Switch: '<S363>/Switch' */

            /* UnitDelay: '<S363>/Unit Delay' incorporates:
             *  Product: '<S363>/Product'
             *  Sum: '<S363>/Add'
             *  Sum: '<S363>/Add1'
             */
            SLC_PrevRearWheelDist2BoundUnitDelay_met =
                ((rtb_Switch_ithu * rtb_Subtract_if0w_idx_0) +
                 rtb_Subtract_if0w_idx_1) +
                rtb_Add_fbcu;
        } else if (SLC_LeverRightEngaged_bool) {
            /* Switch: '<S363>/Switch1' incorporates:
             *  Inport: '<Root>/Inport1'
             *  Inport: '<Root>/Inport2'
             *  Switch: '<S363>/Switch3'
             */
            if (SLC_NewEgoLane_bool) {
                rtb_Switch_ithu = ABPR_LeLnClthPosY0_met;
            } else {
                rtb_Switch_ithu = ABPR_RiLnClthPosY0_met;
            }

            /* End of Switch: '<S363>/Switch1' */

            /* UnitDelay: '<S363>/Unit Delay' incorporates:
             *  Product: '<S363>/Product1'
             *  Sum: '<S363>/Add2'
             *  Sum: '<S363>/Subtract'
             *  Switch: '<S363>/Switch3'
             */
            SLC_PrevRearWheelDist2BoundUnitDelay_met =
                ((rtb_Subtract_if0w_idx_0 * rtb_Switch_ithu) -
                 rtb_Subtract_if0w_idx_1) +
                rtb_Add_fbcu;
        } else {
            /* UnitDelay: '<S363>/Unit Delay' incorporates:
             *  Constant: '<S363>/Constant2'
             *  Switch: '<S363>/Switch3'
             */
            SLC_PrevRearWheelDist2BoundUnitDelay_met = 0.0F;
        }

        /* End of Switch: '<S363>/Switch2' */
    }

    /* End of Switch: '<S363>/Switch4' */

    /* Signum: '<S366>/Sign' incorporates:
     *  UnitDelay: '<S363>/Unit Delay'
     */
    if (SLC_PrevRearWheelDist2BoundUnitDelay_met < 0.0F) {
        SLC_PrevReWhDit2BdSignUnitDelay_met = -1.0F;
    } else if (SLC_PrevRearWheelDist2BoundUnitDelay_met > 0.0F) {
        SLC_PrevReWhDit2BdSignUnitDelay_met = 1.0F;
    } else {
        SLC_PrevReWhDit2BdSignUnitDelay_met =
            SLC_PrevRearWheelDist2BoundUnitDelay_met;
    }

    /* End of Signum: '<S366>/Sign' */

    /* DataTypeConversion: '<S366>/Data Type Conversion1' incorporates:
     *  UnitDelay: '<S366>/Unit Delay'
     */
    rtb_DataTypeConversion1_duje =
        (int8_T)floorf(SLC_PrevReWhDit2BdSignUnitDelay_met);

    /* Logic: '<S366>/NOT' incorporates:
     *  Constant: '<S366>/Constant1'
     *  Constant: '<S366>/Constant2'
     *  RelationalOperator: '<S366>/NotEqual'
     *  RelationalOperator: '<S366>/NotEqual1'
     *  RelationalOperator: '<S366>/NotEqual2'
     */
    SLC_LCM_End_bool =
        (((0 != rtb_DataTypeConversion) &&
          (rtb_DataTypeConversion != rtb_DataTypeConversion1_duje)) &&
         (rtb_DataTypeConversion1_duje != 0));

    /* Logic: '<S261>/OR1' */
    SLC_Abort_bool = (rtb_NotEqual2_emxm || rtb_SLC_RearAbort_bool);

    /* Logic: '<S317>/AND' */
    rtb_OR_baue = (((SLC_TriggerLeft_bool) && rtb_OR_idx_1) &&
                   rtb_SLC_LeAdjLaneWidthValid_boo);

    /* Switch: '<S370>/Switch' incorporates:
     *  Constant: '<S317>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S370>/Max'
     *  Sum: '<S370>/Subtract'
     *  Switch: '<S370>/Switch1'
     *  UnaryMinus: '<S370>/Unary Minus'
     *  UnitDelay: '<S370>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LCPLeft2ActiveTurnOnDelay_sec =
            fmaxf(SLC_LCPLeft2ActiveTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCPLeft2ActiveTurnOnDelay_sec = TJASLC_LCPLeft2Active_sec;
    }

    /* End of Switch: '<S370>/Switch' */

    /* Logic: '<S370>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S370>/LessThanOrEqual'
     *  UnaryMinus: '<S370>/Unary Minus1'
     *  UnitDelay: '<S370>/Unit Delay'
     */
    SLC_LCPLeft2Active_bool =
        (rtb_OR_baue &&
         (SLC_LCPLeft2ActiveTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S317>/AND1' incorporates:
     *  Chart: '<S264>/ManeuverState'
     *  Logic: '<S479>/AND1'
     */
    rtb_NotEqual_ozi5_tmp = (rtb_AND_pjo1 && rtb_OR2_pwtt);
    rtb_OR_baue = (rtb_NotEqual_ozi5_tmp && (SLC_TriggerRight_bool));

    /* Switch: '<S373>/Switch' incorporates:
     *  Constant: '<S317>/Constant3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S373>/Max'
     *  Sum: '<S373>/Subtract'
     *  Switch: '<S373>/Switch1'
     *  UnaryMinus: '<S373>/Unary Minus'
     *  UnitDelay: '<S373>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LCPRight2ActiveTurnOnDelay_sec =
            fmaxf(SLC_LCPRight2ActiveTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCPRight2ActiveTurnOnDelay_sec = TJASLC_LCPRight2Active_sec;
    }

    /* End of Switch: '<S373>/Switch' */

    /* Logic: '<S373>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S373>/LessThanOrEqual'
     *  UnaryMinus: '<S373>/Unary Minus1'
     *  UnitDelay: '<S373>/Unit Delay'
     */
    SLC_LCPRight2Active_bool =
        (rtb_OR_baue && (SLC_LCPRight2ActiveTurnOnDelay_sec <=
                         (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S321>/OR' incorporates:
     *  Constant: '<S450>/Constant'
     *  Constant: '<S451>/Constant'
     *  Constant: '<S452>/Constant'
     *  Constant: '<S453>/Constant'
     *  Constant: '<S454>/Constant'
     *  RelationalOperator: '<S321>/Equal'
     *  RelationalOperator: '<S321>/Equal1'
     *  RelationalOperator: '<S321>/Equal2'
     *  RelationalOperator: '<S321>/Equal3'
     *  RelationalOperator: '<S321>/Equal4'
     */
    SLC_LCM_Cancel_bool =
        ((((((SLC_Cancel_bool) || (((uint32_T)STM_SysStateUnitDelay_bool) ==
                                   E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT)) ||
            (((uint32_T)STM_SysStateUnitDelay_bool) ==
             E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE)) ||
           (((uint32_T)STM_SysStateUnitDelay_bool) ==
            E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED)) ||
          (((uint32_T)STM_SysStateUnitDelay_bool) ==
           E_TJASTM_SysStateTJA_nu_SYSST_ERROR)) ||
         (((uint32_T)STM_SysStateUnitDelay_bool) ==
          E_TJASTM_SysStateTJA_nu_SYSST_OFF));

    /* Logic: '<S317>/OR' incorporates:
     *  Logic: '<S317>/NOT'
     *  Logic: '<S317>/NOT1'
     */
    rtb_OR_baue = (((!rtb_OR_idx_1) || (!rtb_SLC_LeAdjLaneWidthValid_boo)) ||
                   (SLC_LCM_Cancel_bool));

    /* Switch: '<S371>/Switch' incorporates:
     *  Constant: '<S317>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S371>/Max'
     *  Sum: '<S371>/Subtract'
     *  Switch: '<S371>/Switch1'
     *  UnaryMinus: '<S371>/Unary Minus'
     *  UnitDelay: '<S371>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LCPLeft2PassiveTurnOnDelay_sec =
            fmaxf(SLC_LCPLeft2PassiveTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCPLeft2PassiveTurnOnDelay_sec = TJASLC_LCPLeft2Passive_sec;
    }

    /* End of Switch: '<S371>/Switch' */

    /* Logic: '<S371>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S371>/LessThanOrEqual'
     *  UnaryMinus: '<S371>/Unary Minus1'
     *  UnitDelay: '<S371>/Unit Delay'
     */
    SLC_LCPLeft2Passive_bool =
        (rtb_OR_baue && (SLC_LCPLeft2PassiveTurnOnDelay_sec <=
                         (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S317>/NOT2' incorporates:
     *  Chart: '<S264>/ManeuverState'
     */
    rtb_GEN_BrakePadelInvalid__gzuo = !rtb_AND_pjo1;

    /* Logic: '<S317>/NOT3' incorporates:
     *  Chart: '<S264>/ManeuverState'
     */
    rtb_NotEqual_ozi5_tmp_0 = !rtb_OR2_pwtt;

    /* Logic: '<S317>/OR1' incorporates:
     *  Logic: '<S317>/NOT2'
     *  Logic: '<S317>/NOT3'
     */
    rtb_OR_baue = (((SLC_LCM_Cancel_bool) || rtb_GEN_BrakePadelInvalid__gzuo) ||
                   rtb_NotEqual_ozi5_tmp_0);

    /* Switch: '<S372>/Switch' incorporates:
     *  Constant: '<S317>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S372>/Max'
     *  Sum: '<S372>/Subtract'
     *  Switch: '<S372>/Switch1'
     *  UnaryMinus: '<S372>/Unary Minus'
     *  UnitDelay: '<S372>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_LCPRight2PassiveTurnOnDelay_sec =
            fmaxf(SLC_LCPRight2PassiveTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCPRight2PassiveTurnOnDelay_sec = TJASLC_LCPRight2Passive_sec;
    }

    /* End of Switch: '<S372>/Switch' */

    /* Logic: '<S372>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S372>/LessThanOrEqual'
     *  UnaryMinus: '<S372>/Unary Minus1'
     *  UnitDelay: '<S372>/Unit Delay'
     */
    SLC_LCPRight2Passive_bool =
        (rtb_OR_baue && (SLC_LCPRight2PassiveTurnOnDelay_sec <=
                         (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Chart: '<S264>/ManeuverState' incorporates:
     *  Inport: '<Root>/Inport11'
     */
    if (((uint32_T)TJASA_DW.is_active_c6_TJASA) == 0U) {
        TJASA_DW.is_active_c6_TJASA = 1U;
        TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
        TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
        TJASA_B.LCDirection_enum = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    } else {
        switch (TJASA_DW.is_c6_TJASA) {
            case TJASA_IN_ActiveLeft:
                if ((!SLC_StrongReadyBothSides_bool) || (SLC_LCM_Cancel_bool)) {
                    TJASA_DW.is_ActiveLeft = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else if (ABPR_LaneChangeDetected_bool) {
                    TJASA_DW.is_ActiveLeft = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c6_TJASA = TJASA_IN_NewEgoLane;
                    TJASA_DW.is_NewEgoLane = TJASA_IN_NewEgoLane_grb0;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_NEWEGO;
                } else {
                    switch (TJASA_DW.is_ActiveLeft) {
                        case TJASA_IN_Abort:
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_ABORT;
                            break;

                        case TJAS_IN_LaneChangeManeuverStart:
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_LCMSTART;
                            if ((SLC_Abort_bool) || (!rtb_Equal_i3l2_idx_0)) {
                                TJASA_DW.is_ActiveLeft = TJASA_IN_Abort;
                                TJASLC_ManeuverState_nu =
                                    E_TJASLC_ManeuverState_nu_ABORT;
                            }
                            break;

                        default:
                            /* case IN_LateralMovementStart: */
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_LATMVSTART;
                            if ((SLC_Abort_bool) || (!rtb_Equal_i3l2_idx_0)) {
                                TJASA_DW.is_ActiveLeft = TJASA_IN_Abort;
                                TJASLC_ManeuverState_nu =
                                    E_TJASLC_ManeuverState_nu_ABORT;
                            } else {
                                if (SLC_LCM_Start_bool) {
                                    TJASA_DW.is_ActiveLeft =
                                        TJAS_IN_LaneChangeManeuverStart;
                                    TJASLC_ManeuverState_nu =
                                        E_TJASLC_ManeuverState_nu_LCMSTART;
                                }
                            }
                            break;
                    }
                }
                break;

            case TJASA_IN_ActiveRight:
                if (ABPR_LaneChangeDetected_bool) {
                    TJASA_DW.is_ActiveRight = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c6_TJASA = TJASA_IN_NewEgoLane;
                    TJASA_DW.is_NewEgoLane = TJASA_IN_NewEgoLane_grb0;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_NEWEGO;
                } else if ((!SLC_StrongReadyBothSides_bool) ||
                           (SLC_LCM_Cancel_bool)) {
                    TJASA_DW.is_ActiveRight = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else {
                    switch (TJASA_DW.is_ActiveRight) {
                        case TJASA_IN_Abort:
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_ABORT;
                            break;

                        case TJAS_IN_LaneChangeManeuverStart:
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_LCMSTART;
                            if ((SLC_Abort_bool) || (!rtb_Equal_i3l2_idx_1)) {
                                TJASA_DW.is_ActiveRight = TJASA_IN_Abort;
                                TJASLC_ManeuverState_nu =
                                    E_TJASLC_ManeuverState_nu_ABORT;
                            }
                            break;

                        default:
                            /* case IN_LateralMovementStart: */
                            TJASLC_ManeuverState_nu =
                                E_TJASLC_ManeuverState_nu_LATMVSTART;
                            if ((SLC_Abort_bool) || (!rtb_Equal_i3l2_idx_1)) {
                                TJASA_DW.is_ActiveRight = TJASA_IN_Abort;
                                TJASLC_ManeuverState_nu =
                                    E_TJASLC_ManeuverState_nu_ABORT;
                            } else {
                                if (SLC_LCM_Start_bool) {
                                    TJASA_DW.is_ActiveRight =
                                        TJAS_IN_LaneChangeManeuverStart;
                                    TJASLC_ManeuverState_nu =
                                        E_TJASLC_ManeuverState_nu_LCMSTART;
                                }
                            }
                            break;
                    }
                }
                break;

            case IN_LaneChangeProcedureStartLeft:
                TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_LCPSTART;
                TJASA_B.LCDirection_enum =
                    E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG;
                if (!SLC_LeverLeftEngaged_bool) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_ReadyToTriggerLeft;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_TRIGREADY;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else if (SLC_LCPLeft2Active_bool) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_ActiveLeft;
                    TJASA_DW.is_ActiveLeft = TJASA_IN_LateralMovementStart;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_LATMVSTART;
                } else {
                    if (SLC_LCPLeft2Passive_bool) {
                        TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_PASSIVE;
                        TJASA_B.LCDirection_enum =
                            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                    }
                }
                break;

            case IN_LaneChangeProcedureStartRigh:
                TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_LCPSTART;
                TJASA_B.LCDirection_enum =
                    E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG;
                if (SLC_LCPRight2Active_bool) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_ActiveRight;
                    TJASA_DW.is_ActiveRight = TJASA_IN_LateralMovementStart;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_LATMVSTART;
                } else if (!SLC_LeverRightEngaged_bool) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_ReadyToTriggerRight;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_TRIGREADY;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else {
                    if (SLC_LCPRight2Passive_bool) {
                        TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_PASSIVE;
                        TJASA_B.LCDirection_enum =
                            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                    }
                }
                break;

            case TJASA_IN_NewEgoLane:
                if ((!SLC_StrongReadyBothSides_bool) || (SLC_LCM_Cancel_bool)) {
                    TJASA_DW.is_NewEgoLane = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else if (((int32_T)TJASA_DW.is_NewEgoLane) == 1) {
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_LCMEND;
                } else {
                    /* case IN_NewEgoLane: */
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_NEWEGO;
                    if (SLC_LCM_End_bool) {
                        TJASA_DW.is_NewEgoLane = TJASA_IN_LaneChangeManeuverEnd;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_LCMEND;
                    }
                }
                break;

            case TJASA_IN_Passive:
                TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                TJASA_B.LCDirection_enum =
                    E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                if (rtb_AND_pjo1 && rtb_OR2_pwtt) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_ReadyToTriggerRight;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_TRIGREADY;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else {
                    if (rtb_OR_idx_1 && rtb_SLC_LeAdjLaneWidthValid_boo) {
                        TJASA_DW.is_c6_TJASA = TJASA_IN_ReadyToTriggerLeft;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_TRIGREADY;
                        TJASA_B.LCDirection_enum =
                            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                    }
                }
                break;

            case TJASA_IN_ReadyToTriggerLeft:
                TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_TRIGREADY;
                TJASA_B.LCDirection_enum =
                    E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                if ((rtb_OR_idx_1 && rtb_SLC_LeAdjLaneWidthValid_boo) &&
                    (SLC_LeverLeftEngaged_bool)) {
                    TJASA_DW.is_c6_TJASA = IN_LaneChangeProcedureStartLeft;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_LCPSTART;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG;
                } else if (((!rtb_OR_idx_1) ||
                            (!rtb_SLC_LeAdjLaneWidthValid_boo)) ||
                           (SLC_LCM_Cancel_bool)) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else {
                    if (rtb_NotEqual_ozi5_tmp && (SLC_LeverRightEngaged_bool)) {
                        TJASA_DW.is_c6_TJASA = IN_LaneChangeProcedureStartRigh;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_LCPSTART;
                        TJASA_B.LCDirection_enum =
                            E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG;
                    }
                }
                break;

            default:
                /* case IN_ReadyToTriggerRight: */
                TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_TRIGREADY;
                TJASA_B.LCDirection_enum =
                    E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                if (rtb_NotEqual_ozi5_tmp && (SLC_LeverRightEngaged_bool)) {
                    TJASA_DW.is_c6_TJASA = IN_LaneChangeProcedureStartRigh;
                    TJASLC_ManeuverState_nu =
                        E_TJASLC_ManeuverState_nu_LCPSTART;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG;
                } else if ((rtb_GEN_BrakePadelInvalid__gzuo ||
                            rtb_NotEqual_ozi5_tmp_0) ||
                           (SLC_LCM_Cancel_bool)) {
                    TJASA_DW.is_c6_TJASA = TJASA_IN_Passive;
                    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
                    TJASA_B.LCDirection_enum =
                        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
                } else {
                    if ((rtb_OR_idx_1 && rtb_SLC_LeAdjLaneWidthValid_boo) &&
                        (SLC_LeverLeftEngaged_bool)) {
                        TJASA_DW.is_c6_TJASA = IN_LaneChangeProcedureStartLeft;
                        TJASLC_ManeuverState_nu =
                            E_TJASLC_ManeuverState_nu_LCPSTART;
                        TJASA_B.LCDirection_enum =
                            E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG;
                    }
                }
                break;
        }
    }

    /* Switch: '<S320>/Switch' incorporates:
     *  Constant: '<S449>/Constant'
     *  RelationalOperator: '<S320>/Equal'
     */
    if (((uint32_T)STM_SysStateUnitDelay_bool) ==
        E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT) {
        /* Switch: '<S320>/Switch' incorporates:
         *  UnitDelay: '<S320>/Unit Delay'
         */
        TJASLC_ManeuverState_nu = SLC_PrevManeuverState3_Enum;
    }

    /* End of Switch: '<S320>/Switch' */

    /* RelationalOperator: '<S526>/NotEqual' incorporates:
     *  RelationalOperator: '<S322>/Equal1'
     *  RelationalOperator: '<S322>/Equal2'
     *  RelationalOperator: '<S324>/Equal'
     *  RelationalOperator: '<S375>/Equal1'
     *  RelationalOperator: '<S375>/Equal5'
     *  RelationalOperator: '<S375>/Equal7'
     *  RelationalOperator: '<S378>/Equal'
     *  RelationalOperator: '<S378>/Equal1'
     *  RelationalOperator: '<S378>/Equal2'
     *  RelationalOperator: '<S378>/Equal3'
     *  RelationalOperator: '<S378>/Equal7'
     *  RelationalOperator: '<S379>/Equal2'
     *  RelationalOperator: '<S379>/Equal3'
     *  RelationalOperator: '<S379>/Equal7'
     *  RelationalOperator: '<S380>/Equal1'
     *  RelationalOperator: '<S380>/Equal7'
     *  RelationalOperator: '<S381>/Equal1'
     *  RelationalOperator: '<S381>/Equal2'
     *  RelationalOperator: '<S381>/Equal3'
     *  RelationalOperator: '<S381>/Equal7'
     *  RelationalOperator: '<S382>/Equal1'
     *  RelationalOperator: '<S382>/Equal2'
     *  RelationalOperator: '<S382>/Equal7'
     *  RelationalOperator: '<S383>/Equal1'
     *  RelationalOperator: '<S383>/Equal7'
     *  RelationalOperator: '<S838>/Equal5'
     *  RelationalOperator: '<S838>/Equal6'
     *  RelationalOperator: '<S839>/Equal5'
     *  RelationalOperator: '<S839>/Equal6'
     *  RelationalOperator: '<S839>/Equal7'
     *  RelationalOperator: '<S839>/Equal8'
     *  RelationalOperator: '<S888>/Equal10'
     *  RelationalOperator: '<S888>/Equal9'
     *  RelationalOperator: '<S914>/Equal7'
     *  Switch: '<S320>/Switch'
     *  Switch: '<S322>/Switch'
     *  Switch: '<S322>/Switch1'
     *  Switch: '<S445>/Switch'
     *  Switch: '<S888>/Switch'
     */
    SLC_PrevManeuverState2_Enum = TJASLC_ManeuverState_nu;

    /* Logic: '<S324>/OR1' incorporates:
     *  Constant: '<S328>/Constant'
     *  Constant: '<S329>/Constant'
     *  RelationalOperator: '<S324>/Equal'
     *  RelationalOperator: '<S324>/Equal1'
     */
    SLC_IntoAbort_nu = ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
                         E_TJASLC_ManeuverState_nu_ABORT) &&
                        (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) !=
                         E_TJASLC_ManeuverState_nu_ABORT));

    /* Switch: '<S325>/Switch' incorporates:
     *  Switch: '<S325>/Switch1'
     */
    if (SLC_TriggerLeft_bool) {
        /* Switch: '<S325>/Switch' incorporates:
         *  Constant: '<S331>/Constant'
         */
        SLC_LaneChangeDirectionIn_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG;
    } else if (SLC_TriggerRight_bool) {
        /* Switch: '<S325>/Switch1' incorporates:
         *  Constant: '<S332>/Constant'
         *  Switch: '<S325>/Switch'
         */
        SLC_LaneChangeDirectionIn_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG;
    } else {
        /* Switch: '<S325>/Switch' incorporates:
         *  Constant: '<S330>/Constant'
         *  Switch: '<S325>/Switch1'
         */
        SLC_LaneChangeDirectionIn_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    }

    /* End of Switch: '<S325>/Switch' */

    /* RelationalOperator: '<S536>/Equal' incorporates:
     *  RelationalOperator: '<S326>/Equal'
     *  RelationalOperator: '<S326>/Equal1'
     *  RelationalOperator: '<S326>/Equal2'
     *  RelationalOperator: '<S326>/Equal3'
     *  Switch: '<S326>/Switch'
     *  Switch: '<S326>/Switch2'
     *  UnitDelay: '<S314>/Unit Delay'
     */
    SLC_PrevLaneChangeTrigger_nu = SLC_PreLaneChangeDirtAbort_enum;

    /* Switch: '<S326>/Switch' incorporates:
     *  Constant: '<S333>/Constant'
     *  Constant: '<S334>/Constant'
     *  RelationalOperator: '<S326>/Equal'
     *  RelationalOperator: '<S326>/Equal1'
     *  Switch: '<S326>/Switch1'
     *  Switch: '<S326>/Switch2'
     */
    if (((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
        E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG) {
        /* Switch: '<S326>/Switch' incorporates:
         *  Constant: '<S326>/Constant'
         *  Constant: '<S326>/Constant1'
         *  Inport: '<Root>/Inport1'
         *  Logic: '<S326>/AND'
         *  RelationalOperator: '<S326>/GreaterThan'
         *  RelationalOperator: '<S326>/GreaterThan1'
         *  UnitDelay: '<S326>/Unit Delay'
         */
        SLC_SameLaneChangeDetc_bool = ((SLC_UnitDelay_LePosY0_met < 0.5F) &&
                                       (ABPR_LeLnClthPosY0_met > 2.0F));

        /* Switch: '<S326>/Switch2' incorporates:
         *  Constant: '<S326>/Constant4'
         *  Constant: '<S326>/Constant5'
         *  Inport: '<Root>/Inport2'
         *  Logic: '<S326>/AND2'
         *  RelationalOperator: '<S326>/GreaterThan4'
         *  RelationalOperator: '<S326>/GreaterThan5'
         *  UnitDelay: '<S326>/Unit Delay1'
         */
        SLC_LaneChangeBackDetc_bool = ((SLC_UnitDelay_RiPosY0_met > -0.5F) &&
                                       (ABPR_RiLnClthPosY0_met < -2.0F));
    } else {
        if (((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG) {
            /* Switch: '<S326>/Switch1' incorporates:
             *  Constant: '<S326>/Constant2'
             *  Constant: '<S326>/Constant3'
             *  Inport: '<Root>/Inport2'
             *  Logic: '<S326>/AND1'
             *  RelationalOperator: '<S326>/GreaterThan2'
             *  RelationalOperator: '<S326>/GreaterThan3'
             *  Switch: '<S326>/Switch'
             *  UnitDelay: '<S326>/Unit Delay1'
             */
            SLC_SameLaneChangeDetc_bool =
                ((SLC_UnitDelay_RiPosY0_met > -0.5F) &&
                 (ABPR_RiLnClthPosY0_met < -2.0F));
        } else {
            /* Switch: '<S326>/Switch' incorporates:
             *  Inport: '<Root>/Inport11'
             *  Switch: '<S326>/Switch1'
             */
            SLC_SameLaneChangeDetc_bool = ABPR_LaneChangeDetected_bool;
        }

        /* Switch: '<S326>/Switch3' incorporates:
         *  Constant: '<S336>/Constant'
         *  RelationalOperator: '<S326>/Equal3'
         */
        if (((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG) {
            /* Switch: '<S326>/Switch2' incorporates:
             *  Constant: '<S326>/Constant6'
             *  Constant: '<S326>/Constant7'
             *  Inport: '<Root>/Inport1'
             *  Logic: '<S326>/AND3'
             *  RelationalOperator: '<S326>/GreaterThan6'
             *  RelationalOperator: '<S326>/GreaterThan7'
             *  UnitDelay: '<S326>/Unit Delay'
             */
            SLC_LaneChangeBackDetc_bool = ((SLC_UnitDelay_LePosY0_met < 0.5F) &&
                                           (ABPR_LeLnClthPosY0_met > 2.0F));
        } else {
            /* Switch: '<S326>/Switch2' incorporates:
             *  Inport: '<Root>/Inport11'
             */
            SLC_LaneChangeBackDetc_bool = ABPR_LaneChangeDetected_bool;
        }

        /* End of Switch: '<S326>/Switch3' */
    }

    /* Signum: '<S327>/Sign' incorporates:
     *  UnitDelay: '<S1>/Unit Delay6'
     */
    if (TJATTG_TgtTrajPosY0UnitDy_met < 0.0F) {
        rtb_Switch_ithu = -1.0F;
    } else if (TJATTG_TgtTrajPosY0UnitDy_met > 0.0F) {
        rtb_Switch_ithu = 1.0F;
    } else {
        rtb_Switch_ithu = TJATTG_TgtTrajPosY0UnitDy_met;
    }

    /* End of Signum: '<S327>/Sign' */

    /* Signum: '<S327>/Sign1' incorporates:
     *  UnitDelay: '<S1>/Unit Delay7'
     */
    if (TJATTG_TgtTrajHeadAngUnitDy_rad < 0.0F) {
        TJATTG_TgtTrajHeadAngUnitDy_rad = -1.0F;
    } else {
        if (TJATTG_TgtTrajHeadAngUnitDy_rad > 0.0F) {
            TJATTG_TgtTrajHeadAngUnitDy_rad = 1.0F;
        }
    }

    /* End of Signum: '<S327>/Sign1' */

    /* RelationalOperator: '<S327>/NotEqual' incorporates:
     *  DataTypeConversion: '<S327>/Data Type Conversion'
     *  DataTypeConversion: '<S327>/Data Type Conversion1'
     */
    rtb_OR_baue = (((int8_T)rtb_Switch_ithu) !=
                   ((int8_T)TJATTG_TgtTrajHeadAngUnitDy_rad));

    /* Switch: '<S343>/Switch' incorporates:
     *  Constant: '<S327>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S343>/Max'
     *  Sum: '<S343>/Subtract'
     *  Switch: '<S343>/Switch1'
     *  UnaryMinus: '<S343>/Unary Minus'
     *  UnitDelay: '<S343>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_AbortYHdSignDelay2_sec =
            fmaxf(SLC_AbortYHdSignDelay2_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_AbortYHdSignDelay2_sec = TJASLC_AbortYHdSignTurnOnTi_sec;
    }

    /* End of Switch: '<S343>/Switch' */

    /* Logic: '<S343>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S343>/LessThanOrEqual'
     *  UnaryMinus: '<S343>/Unary Minus1'
     *  UnitDelay: '<S343>/Unit Delay'
     */
    rtb_OR2_pwtt = (rtb_OR_baue && (SLC_AbortYHdSignDelay2_sec <=
                                    (-LCFRCV_TSysCycleTimeSen_sec)));

    /* RelationalOperator: '<S327>/Equal2' incorporates:
     *  Constant: '<S339>/Constant'
     */
    rtb_OR_baue = (((uint32_T)SLC_PreAbortState_enum) ==
                   E_TJASLC_AbortState_nu_ABORT_ABORT);

    /* Switch: '<S341>/Switch' incorporates:
     *  Constant: '<S327>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S341>/Max'
     *  Sum: '<S341>/Subtract'
     *  Switch: '<S341>/Switch1'
     *  UnaryMinus: '<S341>/Unary Minus'
     *  UnitDelay: '<S341>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_AbortTime_sec =
            fmaxf(SLC_AbortTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_AbortTime_sec = TJASLC_TimeMaxInAbort_sec;
    }

    /* End of Switch: '<S341>/Switch' */

    /* Logic: '<S327>/OR1' incorporates:
     *  Abs: '<S327>/Abs'
     *  Constant: '<S327>/Constant3'
     *  Constant: '<S337>/Constant'
     *  Constant: '<S338>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S327>/OR'
     *  Logic: '<S341>/AND'
     *  RelationalOperator: '<S327>/Equal'
     *  RelationalOperator: '<S327>/Equal1'
     *  RelationalOperator: '<S327>/GreaterThan3'
     *  RelationalOperator: '<S341>/LessThanOrEqual'
     *  UnaryMinus: '<S341>/Unary Minus1'
     *  UnitDelay: '<S1>/Unit Delay6'
     *  UnitDelay: '<S341>/Unit Delay'
     */
    SLC_ExitAbort_bool = ((((fabsf(TJATTG_TgtTrajPosY0UnitDy_met) <
                             TJASLC_PosYThdExitAbort_met) ||
                            rtb_OR2_pwtt) ||
                           ((((uint32_T)STM_SysStateUnitDelay_bool) ==
                             E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT) ||
                            (((uint32_T)STM_SysStateUnitDelay_bool) ==
                             E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE))) ||
                          (rtb_OR_baue && (SLC_AbortTime_sec <=
                                           (-LCFRCV_TSysCycleTimeSen_sec))));

    /* RelationalOperator: '<S327>/Equal3' incorporates:
     *  Constant: '<S340>/Constant'
     */
    rtb_OR_baue = (((uint32_T)SLC_PreAbortState_enum) ==
                   E_TJASLC_AbortState_nu_ABORT_NEWEGO);

    /* Switch: '<S342>/Switch' incorporates:
     *  Constant: '<S327>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S342>/Max'
     *  Sum: '<S342>/Subtract'
     *  Switch: '<S342>/Switch1'
     *  UnaryMinus: '<S342>/Unary Minus'
     *  UnitDelay: '<S342>/Unit Delay'
     */
    if (rtb_OR_baue) {
        SLC_AbortNewEgoTime_sec =
            fmaxf(SLC_AbortNewEgoTime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_AbortNewEgoTime_sec = TJASLC_TimeMaxInAbortNewEgo_sec;
    }

    /* End of Switch: '<S342>/Switch' */

    /* Logic: '<S342>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S342>/LessThanOrEqual'
     *  UnaryMinus: '<S342>/Unary Minus1'
     *  UnitDelay: '<S342>/Unit Delay'
     */
    SLC_ExitAbortNewEgo_bool =
        (rtb_OR_baue &&
         (SLC_AbortNewEgoTime_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Chart: '<S314>/AbortState' incorporates:
     *  Switch: '<S325>/Switch'
     */
    if (((uint32_T)TJASA_DW.is_active_c180_TJASA) == 0U) {
        TJASA_DW.is_active_c180_TJASA = 1U;
        TJASA_DW.is_c180_TJASA = TJASA_IN_NoActive;

        /* SignalConversion: '<S314>/Signal Conversion' */
        SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
        TJASA_B.SLC_LaneChangeDirection = SLC_LaneChangeDirectionIn_nu;
    } else if (((int32_T)TJASA_DW.is_c180_TJASA) == 1) {
        if (SLC_IntoAbort_nu) {
            TJASA_DW.is_c180_TJASA = TJASA_IN_OriginLaneAbort;
            TJASA_DW.is_OriginLaneAbort = TJASA_IN_Abort;

            /* SignalConversion: '<S314>/Signal Conversion' */
            SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_ABORT;
        } else {
            /* SignalConversion: '<S314>/Signal Conversion' */
            SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
            TJASA_B.SLC_LaneChangeDirection = SLC_LaneChangeDirectionIn_nu;
        }
    } else {
        /* case IN_OriginLaneAbort: */
        if (((int32_T)TJASA_DW.is_OriginLaneAbort) == 1) {
            if (SLC_SameLaneChangeDetc_bool) {
                TJASA_DW.is_OriginLaneAbort = TJASA_IN_AbortNewEgo;

                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NEWEGO;
            } else if (SLC_ExitAbort_bool) {
                TJASA_DW.is_OriginLaneAbort = TJASA_IN_NO_ACTIVE_CHILD;
                TJASA_DW.is_c180_TJASA = TJASA_IN_NoActive;

                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
                TJASA_B.SLC_LaneChangeDirection = SLC_LaneChangeDirectionIn_nu;
            } else {
                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_ABORT;
            }
        } else {
            /* case IN_AbortNewEgo: */
            if (SLC_LaneChangeBackDetc_bool) {
                TJASA_DW.is_OriginLaneAbort = TJASA_IN_Abort;

                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_ABORT;
            } else if (SLC_ExitAbortNewEgo_bool) {
                TJASA_DW.is_OriginLaneAbort = TJASA_IN_NO_ACTIVE_CHILD;
                TJASA_DW.is_c180_TJASA = TJASA_IN_NoActive;

                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
                TJASA_B.SLC_LaneChangeDirection = SLC_LaneChangeDirectionIn_nu;
            } else {
                /* SignalConversion: '<S314>/Signal Conversion' */
                SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NEWEGO;
            }
        }
    }

    /* End of Chart: '<S314>/AbortState' */

    /* S-Function (ex_sfun_set_bit): '<S525>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S524>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_aisn[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_jofi);

    /* RelationalOperator: '<S90>/Equal' incorporates:
     *  Constant: '<S90>/Constant2'
     *  Constant: '<S90>/Constant3'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S90>/Bitwise AND'
     */
    rtb_OR_baue = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                    ((int32_T)TJAGEN_DrvStInvalidSR_C_btm)) == 0);

    /* RelationalOperator: '<S92>/Equal' incorporates:
     *  Constant: '<S92>/Constant2'
     *  Constant: '<S92>/Constant3'
     *  Inport: '<Root>/Inport73'
     *  S-Function (sfix_bitop): '<S92>/Bitwise AND'
     */
    rtb_GEN_MaxSteerAngleExceeded_b =
        ((((int32_T)VDPDRV_VehStInvalid_btf) &
          ((int32_T)TJAGEN_VehStInvalidSR_C_btm)) == 0);

    /* RelationalOperator: '<S89>/Equal' incorporates:
     *  Constant: '<S89>/Constant2'
     *  Constant: '<S89>/Constant3'
     *  Inport: '<Root>/Inport71'
     *  S-Function (sfix_bitop): '<S89>/Bitwise AND'
     */
    rtb_NotEqual1_cuxj = ((((int32_T)VDPDRV_ActiveStCtrl_btf) &
                           ((int32_T)TJAGEN_ActiveStCtrlSR_C_btm)) == 0);

    /* S-Function (sfix_bitop): '<S89>/Bitwise AND1' incorporates:
     *  Constant: '<S89>/Constant1'
     *  Inport: '<Root>/Inport72'
     *  S-Function (sfix_bitop): '<S613>/Bitwise AND2'
     */
    rtb_GEN_WR_Custom_bool_tmp =
        ((int32_T)VDPDRV_SysStError_btf) & ((int32_T)TJAGEN_SysStErrorSR_C_btm);

    /* RelationalOperator: '<S89>/Equal1' incorporates:
     *  Constant: '<S89>/Constant4'
     *  S-Function (sfix_bitop): '<S89>/Bitwise AND1'
     */
    rtb_GEN_Cancel_Custom_bool = (rtb_GEN_WR_Custom_bool_tmp == 0);

    /* Abs: '<S88>/Abs1' incorporates:
     *  Abs: '<S88>/Abs2'
     *  Inport: '<Root>/Inport69'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(VDy_VehAclX_mps2);

    /* Switch: '<S94>/Switch' incorporates:
     *  Abs: '<S88>/Abs1'
     *  Constant: '<S88>/Constant3'
     *  Constant: '<S88>/Constant4'
     *  Constant: '<S94>/Constant'
     *  RelationalOperator: '<S94>/Less Than'
     *  RelationalOperator: '<S94>/Less Than1'
     *  Sum: '<S88>/Add1'
     *  UnitDelay: '<S94>/Unit Delay'
     */
    if ((TJAGEN_AclXMax_C_mps2 + TJAGEN_AccelXHyst_C_mps2) <
        rtb_Subtract_if0w_idx_0) {
        GEN_AclXMaxHyst_bool = true;
    } else {
        GEN_AclXMaxHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >= TJAGEN_AclXMax_C_mps2) &&
             (GEN_AclXMaxHyst_bool));
    }

    /* End of Switch: '<S94>/Switch' */

    /* Switch: '<S95>/Switch' incorporates:
     *  Constant: '<S88>/Constant5'
     *  Constant: '<S88>/Constant6'
     *  Constant: '<S95>/Constant'
     *  RelationalOperator: '<S95>/Less Than'
     *  RelationalOperator: '<S95>/Less Than1'
     *  Sum: '<S88>/Add2'
     *  UnitDelay: '<S95>/Unit Delay'
     */
    if (TJAGEN_AclXMin_C_mps2 < rtb_Subtract_if0w_idx_0) {
        GEN_AclXMinHyst_bool = true;
    } else {
        GEN_AclXMinHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >=
              (TJAGEN_AclXMin_C_mps2 - TJAGEN_AccelXHyst_C_mps2)) &&
             (GEN_AclXMinHyst_bool));
    }

    /* End of Switch: '<S95>/Switch' */

    /* Logic: '<S88>/AND' incorporates:
     *  Logic: '<S88>/NOT1'
     *  UnitDelay: '<S94>/Unit Delay'
     *  UnitDelay: '<S95>/Unit Delay'
     */
    rtb_GEN_BlockTimerExpired_bool =
        ((!GEN_AclXMaxHyst_bool) && (GEN_AclXMinHyst_bool));

    /* Abs: '<S88>/Abs' incorporates:
     *  Inport: '<Root>/Inport70'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(VDy_VehAclY_mps2);

    /* Switch: '<S93>/Switch' incorporates:
     *  Constant: '<S88>/Constant1'
     *  Constant: '<S88>/Constant2'
     *  Constant: '<S93>/Constant'
     *  RelationalOperator: '<S93>/Less Than'
     *  RelationalOperator: '<S93>/Less Than1'
     *  Sum: '<S88>/Add'
     *  UnitDelay: '<S93>/Unit Delay'
     */
    if ((TJAGEN_AclYMax_C_mps2 + TJAGEN_AccelYHyst_C_mps2) <
        rtb_Subtract_if0w_idx_0) {
        GEN_AclYMaxHyst_bool = true;
    } else {
        GEN_AclYMaxHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >= TJAGEN_AclYMax_C_mps2) &&
             (GEN_AclYMaxHyst_bool));
    }

    /* End of Switch: '<S93>/Switch' */

    /* Logic: '<S88>/NOT' incorporates:
     *  UnitDelay: '<S93>/Unit Delay'
     */
    rtb_OR_idx_0 = !GEN_AclYMaxHyst_bool;

    /* RelationalOperator: '<S49>/Equal' incorporates:
     *  Constant: '<S49>/Constant2'
     *  Constant: '<S49>/Constant3'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S49>/Bitwise AND'
     */
    rtb_NotEqual2_emxm = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                           ((int32_T)TJAGEN_PrjSpecQuSR_C_btm)) == 0);

    /* RelationalOperator: '<S49>/NotEqual1' incorporates:
     *  Constant: '<S49>/Constant7'
     *  Constant: '<S49>/Constant8'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S49>/Bitwise AND3'
     */
    TJAGEN_Abort_bool = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                          ((int32_T)TJAGEN_PrjSpecQuA_C_btm)) != 0);

    /* Logic: '<S53>/NOT8' */
    rtb_NotEqual3_lcsh = !TJAGEN_Abort_bool;

    /* Logic: '<S53>/NOT' */
    rtb_VectorConcatenate_gygi[0] = !rtb_OR_baue;

    /* Logic: '<S53>/NOT1' incorporates:
     *  Logic: '<S53>/NOT11'
     */
    rtb_OR2_pwtt = !rtb_GEN_MaxSteerAngleExceeded_b;
    rtb_VectorConcatenate_gygi[1] = rtb_OR2_pwtt;

    /* Logic: '<S53>/NOT2' */
    rtb_VectorConcatenate_gygi[2] = !rtb_NotEqual1_cuxj;

    /* Logic: '<S53>/NOT3' */
    rtb_VectorConcatenate_gygi[3] = !rtb_GEN_Cancel_Custom_bool;

    /* Logic: '<S53>/NOT4' incorporates:
     *  Logic: '<S53>/NOT9'
     */
    rtb_GEN_BrakePadelInvalid__gzuo = !rtb_GEN_BlockTimerExpired_bool;
    rtb_VectorConcatenate_gygi[4] = rtb_GEN_BrakePadelInvalid__gzuo;

    /* Logic: '<S53>/NOT5' incorporates:
     *  Logic: '<S53>/NOT10'
     */
    rtb_NotEqual_ozi5_tmp_0 = !rtb_OR_idx_0;
    rtb_VectorConcatenate_gygi[5] = rtb_NotEqual_ozi5_tmp_0;

    /* Logic: '<S53>/NOT6' */
    rtb_VectorConcatenate_gygi[6] = !rtb_NotEqual2_emxm;

    /* Logic: '<S53>/NOT7' */
    rtb_VectorConcatenate_gygi[7] = !rtb_NotEqual3_lcsh;

    /* S-Function (ex_sfun_set_bit): '<S96>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S91>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_gygi[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* S-Function (sfix_bitop): '<S89>/Bitwise AND2' incorporates:
     *  Constant: '<S89>/Constant5'
     *  Inport: '<Root>/Inport75'
     *  S-Function (sfix_bitop): '<S613>/Bitwise AND3'
     */
    GEN_AllStateAvailable_bool_tmp =
        ((int32_T)VDPDRV_SysStNotAvailable_btf) &
        ((int32_T)TJAGEN_SysStNotAvailableSR_C_btm);

    /* RelationalOperator: '<S89>/Equal2' incorporates:
     *  Constant: '<S89>/Constant6'
     *  S-Function (sfix_bitop): '<S89>/Bitwise AND2'
     */
    GEN_AllStateAvailable_bool = (GEN_AllStateAvailable_bool_tmp == 0);

    /* Switch: '<S82>/Switch' incorporates:
     *  Constant: '<S69>/Constant3'
     *  Constant: '<S69>/Constant6'
     *  Constant: '<S82>/Constant'
     *  Inport: '<Root>/Inport77'
     *  RelationalOperator: '<S82>/Less Than'
     *  RelationalOperator: '<S82>/Less Than1'
     *  Sum: '<S69>/Add'
     *  UnitDelay: '<S82>/Unit Delay'
     */
    if ((TJAGEN_SteerWAngleMaxDI_C_deg + TJAGEN_SteerWAngleHystDI_C_deg) <
        LCFRCV_SteerWAngle_deg) {
        GEN_SteerWAngleHyst_bool = true;
    } else {
        GEN_SteerWAngleHyst_bool =
            ((LCFRCV_SteerWAngle_deg >= TJAGEN_SteerWAngleMaxDI_C_deg) &&
             (GEN_SteerWAngleHyst_bool));
    }

    /* End of Switch: '<S82>/Switch' */

    /* Switch: '<S81>/Switch' incorporates:
     *  Constant: '<S68>/Constant7'
     *  Constant: '<S68>/Constant8'
     *  Constant: '<S81>/Constant'
     *  Inport: '<Root>/Inport96'
     *  RelationalOperator: '<S81>/Less Than'
     *  RelationalOperator: '<S81>/Less Than1'
     *  Sum: '<S68>/Add1'
     *  UnitDelay: '<S81>/Unit Delay'
     */
    if ((TJAGEN_SteerWAngleGradMaxDI_C_degps +
         TJAGEN_SteerWAngleGradHystDI_C_degps) < LCFRCV_SteerWAngleGrad_degps) {
        GEN_SteerWAngleGradHyst_bool = true;
    } else {
        GEN_SteerWAngleGradHyst_bool = ((LCFRCV_SteerWAngleGrad_degps >=
                                         TJAGEN_SteerWAngleGradMaxDI_C_degps) &&
                                        (GEN_SteerWAngleGradHyst_bool));
    }

    /* End of Switch: '<S81>/Switch' */

    /* Switch: '<S87>/Switch' incorporates:
     *  Constant: '<S72>/Constant10'
     *  Constant: '<S72>/Constant9'
     *  Constant: '<S87>/Constant'
     *  Inport: '<Root>/Inport78'
     *  RelationalOperator: '<S87>/Less Than'
     *  RelationalOperator: '<S87>/Less Than1'
     *  Sum: '<S72>/Add2'
     *  UnitDelay: '<S87>/Unit Delay'
     */
    if ((TJAGEN_VehYawRateMaxDI_C_rps + TJAGEN_VehYawRateHystDI_C_rps) <
        VDy_VehYawRate_rps) {
        GEN_VehYawRateDIHyst_bool = true;
    } else {
        GEN_VehYawRateDIHyst_bool =
            ((VDy_VehYawRate_rps >= TJAGEN_VehYawRateMaxDI_C_rps) &&
             (GEN_VehYawRateDIHyst_bool));
    }

    /* End of Switch: '<S87>/Switch' */

    /* Logic: '<S53>/AND' incorporates:
     *  Logic: '<S52>/OR2'
     *  Logic: '<S52>/OR6'
     *  Logic: '<S52>/OR7'
     *  Logic: '<S52>/OR8'
     *  UnitDelay: '<S81>/Unit Delay'
     *  UnitDelay: '<S82>/Unit Delay'
     *  UnitDelay: '<S87>/Unit Delay'
     */
    TJAGEN_StrongReady_bool =
        (((((((((rtb_OR_baue && rtb_GEN_MaxSteerAngleExceeded_b) &&
                rtb_NotEqual1_cuxj) &&
               rtb_GEN_Cancel_Custom_bool) &&
              rtb_GEN_BlockTimerExpired_bool) &&
             rtb_OR_idx_0) &&
            rtb_NotEqual2_emxm) &&
           rtb_NotEqual3_lcsh) &&
          GEN_AllStateAvailable_bool) &&
         (((!GEN_SteerWAngleHyst_bool) && (!GEN_SteerWAngleGradHyst_bool)) &&
          (!GEN_VehYawRateDIHyst_bool)));

    /* RelationalOperator: '<S46>/NotEqual' incorporates:
     *  Constant: '<S46>/Constant2'
     *  Constant: '<S46>/Constant3'
     *  Inport: '<Root>/Inport73'
     *  S-Function (sfix_bitop): '<S46>/Bitwise AND'
     */
    rtb_OR_baue = ((((int32_T)VDPDRV_VehStInvalid_btf) &
                    ((int32_T)TJAGEN_VehStInvalidC_C_btm)) != 0);

    /* RelationalOperator: '<S46>/NotEqual1' incorporates:
     *  Constant: '<S46>/Constant4'
     *  Constant: '<S46>/Constant5'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S46>/Bitwise AND1'
     */
    rtb_NotEqual1_cuxj = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                           ((int32_T)TJAGEN_DrvStInvalidC_C_btm)) != 0);

    /* RelationalOperator: '<S49>/NotEqual' incorporates:
     *  Constant: '<S49>/Constant1'
     *  Constant: '<S49>/Constant6'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S49>/Bitwise AND2'
     */
    rtb_GEN_Cancel_Custom_bool = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                                   ((int32_T)TJAGEN_PrjSpecQuC_C_btm)) != 0);

    /* RelationalOperator: '<S46>/NotEqual2' incorporates:
     *  Constant: '<S46>/Constant1'
     *  Constant: '<S46>/Constant6'
     *  Inport: '<Root>/Inport62'
     *  S-Function (sfix_bitop): '<S46>/Bitwise AND2'
     */
    rtb_NotEqual2_emxm = ((((int32_T)TRJPLN_QuStatusTrajPlan_nu) &
                           ((int32_T)TJAGEN_QuTrajPlanCancel_C_btm)) != 0);

    /* RelationalOperator: '<S46>/NotEqual3' incorporates:
     *  Constant: '<S46>/Constant7'
     *  Constant: '<S46>/Constant8'
     *  Inport: '<Root>/Inport63'
     *  S-Function (sfix_bitop): '<S46>/Bitwise AND3'
     */
    rtb_NotEqual3_lcsh = ((((int32_T)TRJCTR_QuServTrajCtr_nu) &
                           ((int32_T)TJAGEN_QuTrajCtrCancel_C_btm)) != 0);

    /* Logic: '<S46>/OR' */
    TJAGEN_Cancel_nu =
        ((((rtb_OR_baue || rtb_NotEqual1_cuxj) || rtb_GEN_Cancel_Custom_bool) ||
          rtb_NotEqual2_emxm) ||
         rtb_NotEqual3_lcsh);

    /* RelationalOperator: '<S97>/GreaterThan' incorporates:
     *  Constant: '<S97>/Constant3'
     *  Inport: '<Root>/Inport77'
     */
    rtb_GEN_MaxSteerAngleExceeded_b =
        (LCFRCV_SteerWAngle_deg > TJAGEN_SteerWAngleMaxWR_C_deg);

    /* Switch: '<S99>/Switch2' incorporates:
     *  Constant: '<S97>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S53>/OR'
     *  Logic: '<S97>/OR'
     *  MinMax: '<S97>/Max'
     *  RelationalOperator: '<S99>/GreaterThan'
     *  Switch: '<S99>/Switch'
     *  UnitDelay: '<S99>/Unit Delay'
     */
    if ((((rtb_OR2_pwtt || rtb_GEN_BrakePadelInvalid__gzuo) ||
          rtb_NotEqual_ozi5_tmp_0) ||
         TJAGEN_Cancel_nu) ||
        rtb_GEN_MaxSteerAngleExceeded_b) {
        GEN_BlockTimeExpiredTimerRetrigger_sec =
            fmaxf(LCFRCV_TSysCycleTimeSen_sec, TJAGEN_BlockTimeTJA_C_sec);
    } else if (GEN_BlockTimeExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S99>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S99>/Subtract'
         *  UnitDelay: '<S99>/Unit Delay'
         */
        GEN_BlockTimeExpiredTimerRetrigger_sec =
            GEN_BlockTimeExpiredTimerRetrigger_sec -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S99>/Unit Delay' incorporates:
         *  Constant: '<S99>/Constant1'
         *  Switch: '<S99>/Switch'
         */
        GEN_BlockTimeExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S99>/Switch2' */

    /* Logic: '<S97>/NOT' incorporates:
     *  Constant: '<S99>/Constant2'
     *  RelationalOperator: '<S99>/GreaterThan1'
     *  UnitDelay: '<S99>/Unit Delay'
     */
    rtb_GEN_BlockTimerExpired_bool =
        (GEN_BlockTimeExpiredTimerRetrigger_sec <= 0.0F);

    /* RelationalOperator: '<S54>/Equal' incorporates:
     *  Constant: '<S54>/Constant2'
     *  Constant: '<S54>/Constant3'
     *  Inport: '<Root>/Inport73'
     *  S-Function (sfix_bitop): '<S54>/Bitwise AND'
     */
    rtb_OR_idx_0 = ((((int32_T)VDPDRV_VehStInvalid_btf) &
                     ((int32_T)TJAGEN_VehStInvalidWR_C_btm)) == 0);

    /* RelationalOperator: '<S54>/Equal1' incorporates:
     *  Constant: '<S54>/Constant1'
     *  Constant: '<S54>/Constant4'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S54>/Bitwise AND1'
     */
    rtb_Equal1_p2mp = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                        ((int32_T)TJAGEN_DrvStInvalidWR_C_btm)) == 0);

    /* RelationalOperator: '<S54>/Equal2' incorporates:
     *  Constant: '<S54>/Constant5'
     *  Constant: '<S54>/Constant6'
     *  Inport: '<Root>/Inport75'
     *  S-Function (sfix_bitop): '<S54>/Bitwise AND2'
     */
    rtb_Equal2_kbd5 = ((((int32_T)VDPDRV_SysStNotAvailable_btf) &
                        ((int32_T)TJAGEN_SysStNotAvailableWR_C_btm)) == 0);

    /* RelationalOperator: '<S49>/Equal1' incorporates:
     *  Constant: '<S49>/Constant4'
     *  Constant: '<S49>/Constant5'
     *  Inport: '<Root>/Inport12'
     *  S-Function (sfix_bitop): '<S49>/Bitwise AND1'
     */
    rtb_GEN_WR_Custom_bool = ((((int32_T)CUSTOM_PrjSpecQu_btf) &
                               ((int32_T)TJAGEN_PrjSpecQuWR_C_btm)) == 0);

    /* RelationalOperator: '<S54>/Less Than' incorporates:
     *  Constant: '<S54>/Constant7'
     *  Inport: '<Root>/Inport76'
     *  Inport: '<Root>/Inport8'
     *  Lookup_n-D: '<S54>/1-D Lookup Table1'
     *  Product: '<S54>/Divide'
     */
    rtb_LessThan_ko2v =
        (VDy_VehCrv_1pm <
         look1_iflf_binlxpw(VDy_VehVelocity_kph / 3.6F,
                            ((const real32_T *)&(TJAGEN_VehVelX_Bx_mps[0])),
                            ((const real32_T *)&(TJAGEN_VehCrvMaxWR_Cr_1pm[0])),
                            5U));

    /* Logic: '<S54>/AND' incorporates:
     *  Constant: '<S54>/Constant10'
     *  Constant: '<S54>/Constant11'
     *  Constant: '<S54>/Constant12'
     *  Constant: '<S54>/Constant13'
     *  Constant: '<S54>/Constant14'
     *  Constant: '<S54>/Constant15'
     *  Constant: '<S54>/Constant16'
     *  Constant: '<S54>/Constant17'
     *  Constant: '<S54>/Constant18'
     *  Constant: '<S54>/Constant9'
     *  Inport: '<Root>/Inport122'
     *  Inport: '<Root>/Inport123'
     *  Inport: '<Root>/Inport124'
     *  Logic: '<S54>/NOT7'
     *  Logic: '<S54>/NOT8'
     *  Logic: '<S54>/NOT9'
     *  RelationalOperator: '<S54>/Less Than1'
     *  RelationalOperator: '<S54>/Less Than10'
     *  RelationalOperator: '<S54>/Less Than2'
     *  RelationalOperator: '<S54>/Less Than3'
     *  RelationalOperator: '<S54>/Less Than4'
     *  RelationalOperator: '<S54>/Less Than5'
     *  RelationalOperator: '<S54>/Less Than6'
     *  RelationalOperator: '<S54>/Less Than7'
     *  RelationalOperator: '<S54>/Less Than8'
     *  RelationalOperator: '<S54>/Less Than9'
     */
    TJAGEN_WeakReady_bool =
        (((((((((rtb_GEN_BlockTimerExpired_bool && rtb_OR_idx_0) &&
                rtb_Equal1_p2mp) &&
               rtb_Equal2_kbd5) &&
              rtb_GEN_WR_Custom_bool) &&
             rtb_LessThan_ko2v) &&
            rtb_AND_cvae) &&
           (!EMFID_bFCB_bool)) &&
          (((int32_T)ADCS4_AVM_Sts) != 3)) &&
         (((((((((((int32_T)ADCS11_Parking_WorkSts) != 1) &&
                 (((int32_T)ADCS11_Parking_WorkSts) != 3)) &&
                (((int32_T)ADCS11_Parking_WorkSts) != 4)) &&
               (((int32_T)ADCS11_Parking_WorkSts) != 5)) &&
              (((int32_T)ADCS11_Parking_WorkSts) != 6)) &&
             (((int32_T)ADCS11_Parking_WorkSts) != 7)) &&
            (((int32_T)ADCS11_Parking_WorkSts) != 8)) &&
           (((int32_T)ADCS11_Parking_WorkSts) != 9)) &&
          (((int32_T)ADCS11_Parking_WorkSts) != 11)));

    /* Logic: '<S566>/NOT2' incorporates:
     *  Logic: '<S15>/NOT5'
     *  Logic: '<S648>/AND1'
     *  Logic: '<S8>/NOT3'
     */
    rtb_OR_ly14 = !TJALKA_WeakReady_bool;

    /* Logic: '<S566>/OR1' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S564>/OR3'
     *  Logic: '<S566>/NOT2'
     *  Logic: '<S567>/OR1'
     *  Logic: '<S648>/AND10'
     *  Logic: '<S648>/AND11'
     */
    rtb_SLC_VelocityValid_bool = (rtb_AND18 || rtb_OR_ly14);

    /* Logic: '<S566>/NOT' incorporates:
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S15>/NOT2'
     *  Logic: '<S565>/NOT1'
     *  Logic: '<S648>/AND3'
     *  Logic: '<S648>/AND8'
     *  Logic: '<S8>/NOT4'
     */
    rtb_GreaterThan_bpjw_idx_1 = !TJAOBF_StrongReady_bool;

    /* Logic: '<S567>/NOT' incorporates:
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S565>/NOT3'
     *  Logic: '<S8>/NOT6'
     */
    rtb_GreaterThan_bpjw_idx_0 = !TJASLC_StrongReady_bool;

    /* Logic: '<S566>/NOT3' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S564>/NOT'
     *  Logic: '<S8>/NOT8'
     */
    rtb_SLC_RiAdjLaneValid_bool = !TJACMB_StrongReady_bool;

    /* Logic: '<S566>/NOT4' incorporates:
     *  Logic: '<S8>/NOT9'
     */
    rtb_OBF_VelocityValid_bool = !TJACMB_WeakReady_bool;

    /* Logic: '<S566>/OR2' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S565>/OR3'
     *  Logic: '<S566>/NOT3'
     *  Logic: '<S566>/NOT4'
     */
    rtb_Equal_i3l2_idx_0 =
        (rtb_SLC_RiAdjLaneValid_bool || rtb_OBF_VelocityValid_bool);

    /* Logic: '<S565>/NOT2' incorporates:
     *  Logic: '<S15>/NOT6'
     *  Logic: '<S648>/AND3'
     *  Logic: '<S8>/NOT5'
     */
    rtb_SLC_RiAdjLaneWidthValid_boo = !TJAOBF_WeakReady_bool;

    /* Logic: '<S565>/OR1' incorporates:
     *  Chart: '<S8>/StateMachineTJA'
     *  Logic: '<S564>/OR1'
     *  Logic: '<S565>/NOT2'
     */
    rtb_NotEqual_ozi5_tmp_0 =
        (rtb_GreaterThan_bpjw_idx_1 || rtb_SLC_RiAdjLaneWidthValid_boo);

    /* Logic: '<S565>/NOT4' incorporates:
     *  Logic: '<S8>/NOT7'
     */
    rtb_SLC_PrevDriverTriggerResetR = !TJASLC_WeakReady_bool;

    /* Logic: '<S565>/OR2' incorporates:
     *  Logic: '<S564>/OR2'
     *  Logic: '<S565>/NOT4'
     */
    rtb_OR2_pwtt =
        (rtb_GreaterThan_bpjw_idx_0 || rtb_SLC_PrevDriverTriggerResetR);

    /* Logic: '<S567>/OR' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Logic: '<S567>/NOT'
     */
    rtb_Equal_i3l2_idx_1 = (rtb_GreaterThan_bpjw_idx_0 || TJASLC_Cancel_bool);

    /* Logic: '<S566>/OR' incorporates:
     *  Chart: '<S8>/LatCtrlMode'
     *  Logic: '<S566>/NOT'
     *  Logic: '<S8>/OR1'
     */
    rtb_SLC_LeTurnSignalOn_bool =
        (rtb_GreaterThan_bpjw_idx_1 || TJAOBF_Cancel_bool);

    /* Logic: '<S547>/OR' incorporates:
     *  Constant: '<S568>/Constant'
     *  Constant: '<S569>/Constant'
     *  Constant: '<S570>/Constant'
     *  Constant: '<S571>/Constant'
     *  Logic: '<S564>/AND'
     *  Logic: '<S565>/AND'
     *  Logic: '<S565>/OR1'
     *  Logic: '<S565>/OR2'
     *  Logic: '<S566>/AND'
     *  Logic: '<S566>/OR'
     *  Logic: '<S566>/OR1'
     *  Logic: '<S566>/OR2'
     *  Logic: '<S567>/AND'
     *  Logic: '<S567>/OR'
     *  RelationalOperator: '<S564>/Equal'
     *  RelationalOperator: '<S565>/Equal'
     *  RelationalOperator: '<S566>/Equal'
     *  RelationalOperator: '<S567>/Equal'
     */
    STM_Cancel_bool =
        ((((TJAGEN_Cancel_nu || ((((((uint32_T)GEN_PrevLatCtrlMode_Enum) ==
                                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF) &&
                                   rtb_SLC_LeTurnSignalOn_bool) &&
                                  rtb_SLC_VelocityValid_bool) &&
                                 rtb_Equal_i3l2_idx_0)) ||
           (((((uint32_T)GEN_PrevLatCtrlMode_Enum) ==
              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
             rtb_Equal_i3l2_idx_1) &&
            rtb_SLC_VelocityValid_bool)) ||
          (((((((uint32_T)GEN_PrevLatCtrlMode_Enum) ==
               E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) &&
              rtb_AND_ixtq) &&
             rtb_NotEqual_ozi5_tmp_0) &&
            rtb_OR2_pwtt) &&
           rtb_Equal_i3l2_idx_0)) ||
         (((((((uint32_T)GEN_PrevLatCtrlMode_Enum) ==
              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB) &&
             rtb_SLC_RiAdjLaneValid_bool) &&
            rtb_NotEqual_ozi5_tmp_0) &&
           rtb_OR2_pwtt) &&
          rtb_SLC_VelocityValid_bool));

    /* RelationalOperator: '<S67>/NotEqual2' incorporates:
     *  Constant: '<S67>/Constant1'
     *  Constant: '<S67>/Constant2'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S67>/Bitwise AND2'
     */
    rtb_AND_nxkg = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                     ((int32_T)TJAGEN_DrvStHazard_C_btm)) != 0);

    /* Switch: '<S79>/Switch' incorporates:
     *  Constant: '<S67>/V_Parameter3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S79>/Max'
     *  Sum: '<S79>/Subtract'
     *  Switch: '<S79>/Switch1'
     *  UnaryMinus: '<S79>/Unary Minus'
     *  UnitDelay: '<S79>/Unit Delay'
     */
    if (rtb_AND_nxkg) {
        GEN_HazardTurnOnDelay_sec =
            fmaxf(GEN_HazardTurnOnDelay_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_HazardTurnOnDelay_sec = TJAGEN_HazardMaxTime_sec;
    }

    /* End of Switch: '<S79>/Switch' */

    /* Logic: '<S79>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S79>/LessThanOrEqual'
     *  UnaryMinus: '<S79>/Unary Minus1'
     *  UnitDelay: '<S79>/Unit Delay'
     */
    rtb_AND_nxkg = (rtb_AND_nxkg && (GEN_HazardTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* S-Function (sfix_bitop): '<S62>/Bitwise AND1' incorporates:
     *  Constant: '<S62>/Constant4'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S609>/Bitwise AND1'
     */
    rtb_GEN_BrakePadelInvalid__hw14 =
        ((int32_T)VDPDRV_DrvStInvalid_btf) & ((int32_T)TJAGEN_DrvStBrake_C_btm);

    /* RelationalOperator: '<S62>/NotEqual1' incorporates:
     *  Constant: '<S62>/Constant5'
     *  S-Function (sfix_bitop): '<S62>/Bitwise AND1'
     */
    rtb_GEN_BrakePadelInvalid__gzuo = (rtb_GEN_BrakePadelInvalid__hw14 != 0);

    /* RelationalOperator: '<S63>/NotEqual2' incorporates:
     *  Constant: '<S63>/Constant1'
     *  Constant: '<S63>/Constant2'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S63>/Bitwise AND2'
     */
    rtb_GEN_DrvNotBuckledUp_bool =
        ((((int32_T)VDPDRV_DrvStInvalid_btf) &
          ((int32_T)TJAGEN_DrvStNotBuckled_C_btm)) != 0);

    /* RelationalOperator: '<S64>/NotEqual2' incorporates:
     *  Constant: '<S64>/Constant1'
     *  Constant: '<S64>/Constant2'
     *  Inport: '<Root>/Inport74'
     *  S-Function (sfix_bitop): '<S64>/Bitwise AND2'
     */
    rtb_OR2_pwtt = ((((int32_T)VDPDRV_DrvStInvalid_btf) &
                     ((int32_T)TJAGEN_DrvStHOD_C_btm)) != 0);

    /* Switch: '<S73>/Switch' incorporates:
     *  Constant: '<S64>/V_Parameter3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S73>/Max'
     *  Sum: '<S73>/Subtract'
     *  Switch: '<S73>/Switch1'
     *  UnaryMinus: '<S73>/Unary Minus'
     *  UnitDelay: '<S73>/Unit Delay'
     */
    if (rtb_OR2_pwtt) {
        GEN_HODTurnOnDelay_sec =
            fmaxf(GEN_HODTurnOnDelay_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_HODTurnOnDelay_sec = TJAGEN_HODMaxTime_sec;
    }

    /* End of Switch: '<S73>/Switch' */

    /* Logic: '<S73>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S73>/LessThanOrEqual'
     *  UnaryMinus: '<S73>/Unary Minus1'
     *  UnitDelay: '<S73>/Unit Delay'
     */
    rtb_OR2_pwtt = (rtb_OR2_pwtt &&
                    (GEN_HODTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* DataTypeConversion: '<S96>/Data Type Conversion1' */
    TJAGEN_StrongReadyInvalid_btf = (uint8_T)rtb_ex_sfun_set_bit_jx5q;

    /* RelationalOperator: '<S71>/NotEqual3' incorporates:
     *  Constant: '<S71>/Constant'
     *  Constant: '<S71>/Constant4'
     *  S-Function (sfix_bitop): '<S71>/Bitwise AND'
     */
    rtb_GEN_VehStInvalid_bool = ((((int32_T)TJAGEN_StrongReadyInvalid_btf) &
                                  ((int32_T)TJAGEN_VehStInvalid_C_btm)) != 0);

    /* RelationalOperator: '<S71>/NotEqual1' incorporates:
     *  Constant: '<S71>/Constant1'
     *  Constant: '<S71>/Constant3'
     *  S-Function (sfix_bitop): '<S71>/Bitwise AND1'
     */
    rtb_AND_gpqq = ((((int32_T)TJAGEN_StrongReadyInvalid_btf) &
                     ((int32_T)TJAGEN_VehSafetyFuncActive_C_btm)) != 0);

    /* Switch: '<S85>/Switch' incorporates:
     *  Constant: '<S71>/V_Parameter1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S85>/Max'
     *  Sum: '<S85>/Subtract'
     *  Switch: '<S85>/Switch1'
     *  UnaryMinus: '<S85>/Unary Minus'
     *  UnitDelay: '<S85>/Unit Delay'
     */
    if (rtb_AND_gpqq) {
        GEN_SafeFuncActiveTurnOnDelay_sec =
            fmaxf(GEN_SafeFuncActiveTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_SafeFuncActiveTurnOnDelay_sec = TJAGEN_SafetyFuncMaxTime_sec;
    }

    /* End of Switch: '<S85>/Switch' */

    /* Logic: '<S85>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S85>/LessThanOrEqual'
     *  UnaryMinus: '<S85>/Unary Minus1'
     *  UnitDelay: '<S85>/Unit Delay'
     */
    rtb_AND_gpqq = (rtb_AND_gpqq && (GEN_SafeFuncActiveTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* RelationalOperator: '<S71>/NotEqual4' incorporates:
     *  Constant: '<S71>/Constant5'
     *  Constant: '<S71>/Constant6'
     *  S-Function (sfix_bitop): '<S71>/Bitwise AND2'
     */
    rtb_AND_pjo1 = ((((int32_T)TJAGEN_StrongReadyInvalid_btf) &
                     ((int32_T)TJAGEN_VehSafetyFuncError_C_btm)) != 0);

    /* Switch: '<S86>/Switch' incorporates:
     *  Constant: '<S71>/V_Parameter2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S86>/Max'
     *  Sum: '<S86>/Subtract'
     *  Switch: '<S86>/Switch1'
     *  UnaryMinus: '<S86>/Unary Minus'
     *  UnitDelay: '<S86>/Unit Delay'
     */
    if (rtb_AND_pjo1) {
        GEN_SafeFuncErrorTurnOnDelay_sec =
            fmaxf(GEN_SafeFuncErrorTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_SafeFuncErrorTurnOnDelay_sec = TJAGEN_SafetyFuncMaxTime_sec;
    }

    /* End of Switch: '<S86>/Switch' */

    /* Logic: '<S86>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S86>/LessThanOrEqual'
     *  UnaryMinus: '<S86>/Unary Minus1'
     *  UnitDelay: '<S86>/Unit Delay'
     */
    rtb_AND_pjo1 = (rtb_AND_pjo1 && (GEN_SafeFuncErrorTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S52>/OR' incorporates:
     *  Inport: '<Root>/Inport118'
     *  Inport: '<Root>/Inport120'
     *  Logic: '<S52>/OR1'
     *  UnitDelay: '<S81>/Unit Delay'
     *  UnitDelay: '<S82>/Unit Delay'
     */
    TJAGEN_FunctionQuit_bool =
        (((((((((((rtb_AND_nxkg || rtb_GEN_BrakePadelInvalid__gzuo) ||
                  rtb_GEN_DrvNotBuckledUp_bool) ||
                 rtb_OR2_pwtt) ||
                (GEN_SteerWAngleHyst_bool)) ||
               (GEN_SteerWAngleGradHyst_bool)) ||
              rtb_GEN_VehStInvalid_bool) ||
             rtb_AND_gpqq) ||
            rtb_AND_pjo1) ||
           (!GEN_AllStateAvailable_bool)) ||
          EMFID_bRCC_bool) ||
         EMFID_bRCA_bool);

    /* Abs: '<S65>/Abs' incorporates:
     *  Inport: '<Root>/Inport95'
     */
    rtb_Subtract_if0w_idx_0 = fabsf(LCFRCV_ManualTorque_nm);

    /* Switch: '<S74>/Switch' incorporates:
     *  Constant: '<S65>/V_Parameter11'
     *  Constant: '<S65>/V_Parameter2'
     *  Constant: '<S74>/Constant'
     *  RelationalOperator: '<S74>/Less Than'
     *  RelationalOperator: '<S74>/Less Than1'
     *  Sum: '<S65>/Add'
     *  UnitDelay: '<S74>/Unit Delay'
     */
    if ((TJAGEN_ManualTorqueMax_nm + TJAGEN_ManualTorqueHyst_nm) <
        rtb_Subtract_if0w_idx_0) {
        GEN_ManualTorqueMaxHyst_bool = true;
    } else {
        GEN_ManualTorqueMaxHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >= TJAGEN_ManualTorqueMax_nm) &&
             (GEN_ManualTorqueMaxHyst_bool));
    }

    /* End of Switch: '<S74>/Switch' */

    /* Switch: '<S76>/Switch' incorporates:
     *  Constant: '<S65>/V_Parameter1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S76>/Max'
     *  Sum: '<S76>/Subtract'
     *  Switch: '<S76>/Switch1'
     *  UnaryMinus: '<S76>/Unary Minus'
     *  UnitDelay: '<S74>/Unit Delay'
     *  UnitDelay: '<S76>/Unit Delay'
     */
    if (GEN_ManualTorqueMaxHyst_bool) {
        GEN_ManualTorMaxTurnOnDelay_sec = fmaxf(GEN_ManualTorMaxTurnOnDelay_sec,
                                                -LCFRCV_TSysCycleTimeSen_sec) -
                                          LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_ManualTorMaxTurnOnDelay_sec = TJAGEN_ManualTorqueMaxTime_sec;
    }

    /* End of Switch: '<S76>/Switch' */

    /* Logic: '<S76>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S76>/LessThanOrEqual'
     *  UnaryMinus: '<S76>/Unary Minus1'
     *  UnitDelay: '<S74>/Unit Delay'
     *  UnitDelay: '<S76>/Unit Delay'
     */
    rtb_AND_cvae =
        ((GEN_ManualTorqueMaxHyst_bool) &&
         (GEN_ManualTorMaxTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* RelationalOperator: '<S71>/NotEqual5' incorporates:
     *  Constant: '<S71>/Constant7'
     *  Constant: '<S71>/Constant8'
     *  S-Function (sfix_bitop): '<S71>/Bitwise AND3'
     */
    rtb_OR_imzg_idx_0 = ((((int32_T)TJAGEN_StrongReadyInvalid_btf) &
                          ((int32_T)TJAGEN_VehAclYInvalid_C_btm)) != 0);

    /* Switch: '<S84>/Switch' incorporates:
     *  Constant: '<S70>/Constant11'
     *  Constant: '<S70>/Constant12'
     *  Constant: '<S84>/Constant'
     *  Inport: '<Root>/Inport76'
     *  RelationalOperator: '<S84>/Less Than'
     *  RelationalOperator: '<S84>/Less Than1'
     *  Sum: '<S70>/Add3'
     *  UnitDelay: '<S84>/Unit Delay'
     */
    if ((TJAGEN_VehCrvMaxDI_1pm + TJAGEN_VehCrvHystDI_1pm) < VDy_VehCrv_1pm) {
        GEN_VehCrvDIHyst_bool = true;
    } else {
        GEN_VehCrvDIHyst_bool = ((VDy_VehCrv_1pm >= TJAGEN_VehCrvMaxDI_1pm) &&
                                 (GEN_VehCrvDIHyst_bool));
    }

    /* End of Switch: '<S84>/Switch' */

    /* Switch: '<S83>/Switch' incorporates:
     *  Constant: '<S69>/Constant1'
     *  Constant: '<S69>/Constant2'
     *  Constant: '<S83>/Constant'
     *  Inport: '<Root>/Inport77'
     *  RelationalOperator: '<S83>/Less Than'
     *  RelationalOperator: '<S83>/Less Than1'
     *  Sum: '<S69>/Add1'
     *  UnitDelay: '<S83>/Unit Delay'
     */
    if ((TJAGEN_SteerWAngleMaxSus_C_deg + TJAGEN_SteerWAngleHystSus_C_deg) <
        LCFRCV_SteerWAngle_deg) {
        GEN_SteerWAngleSusHyst_bool = true;
    } else {
        GEN_SteerWAngleSusHyst_bool =
            ((LCFRCV_SteerWAngle_deg >= TJAGEN_SteerWAngleMaxSus_C_deg) &&
             (GEN_SteerWAngleSusHyst_bool));
    }

    /* End of Switch: '<S83>/Switch' */

    /* Switch: '<S80>/Switch' incorporates:
     *  Constant: '<S68>/Constant1'
     *  Constant: '<S68>/Constant2'
     *  Constant: '<S80>/Constant'
     *  Inport: '<Root>/Inport96'
     *  RelationalOperator: '<S80>/Less Than'
     *  RelationalOperator: '<S80>/Less Than1'
     *  Sum: '<S68>/Add2'
     *  UnitDelay: '<S80>/Unit Delay'
     */
    if ((TJAGEN_SteerWAngleGradMaxSus_C_degps +
         TJAGEN_SteerWAngleGradHystSus_C_degps) <
        LCFRCV_SteerWAngleGrad_degps) {
        GEN_SteerWAngleGradSusHyst_bool = true;
    } else {
        GEN_SteerWAngleGradSusHyst_bool =
            ((LCFRCV_SteerWAngleGrad_degps >=
              TJAGEN_SteerWAngleGradMaxSus_C_degps) &&
             (GEN_SteerWAngleGradSusHyst_bool));
    }

    /* End of Switch: '<S80>/Switch' */

    /* Logic: '<S52>/OR3' incorporates:
     *  UnitDelay: '<S80>/Unit Delay'
     *  UnitDelay: '<S83>/Unit Delay'
     *  UnitDelay: '<S84>/Unit Delay'
     *  UnitDelay: '<S87>/Unit Delay'
     */
    rtb_OR_imzg_idx_1 = ((((rtb_OR_imzg_idx_0 || (GEN_VehYawRateDIHyst_bool)) ||
                           (GEN_VehCrvDIHyst_bool)) ||
                          (GEN_SteerWAngleSusHyst_bool)) ||
                         (GEN_SteerWAngleGradSusHyst_bool));

    /* Logic: '<S52>/OR4' */
    TJAGEN_SuspendStart_bool = (rtb_AND_cvae || rtb_OR_imzg_idx_1);

    /* Switch: '<S75>/Switch' incorporates:
     *  Constant: '<S65>/V_Parameter4'
     *  Constant: '<S65>/V_Parameter5'
     *  Constant: '<S75>/Constant'
     *  RelationalOperator: '<S75>/Less Than'
     *  RelationalOperator: '<S75>/Less Than1'
     *  Sum: '<S65>/Add1'
     *  UnitDelay: '<S75>/Unit Delay'
     */
    if (TJAGEN_ManualTorqueMin_nm < rtb_Subtract_if0w_idx_0) {
        GEN_ManualTorqueMinHyst_bool = true;
    } else {
        GEN_ManualTorqueMinHyst_bool =
            ((rtb_Subtract_if0w_idx_0 >=
              (TJAGEN_ManualTorqueMin_nm - TJAGEN_ManualTorqueHyst_nm)) &&
             (GEN_ManualTorqueMinHyst_bool));
    }

    /* End of Switch: '<S75>/Switch' */

    /* Logic: '<S65>/NOT' incorporates:
     *  UnitDelay: '<S75>/Unit Delay'
     */
    rtb_AND_eqpk = !GEN_ManualTorqueMinHyst_bool;

    /* Switch: '<S77>/Switch' incorporates:
     *  Constant: '<S65>/V_Parameter3'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S77>/Max'
     *  Sum: '<S77>/Subtract'
     *  Switch: '<S77>/Switch1'
     *  UnaryMinus: '<S77>/Unary Minus'
     *  UnitDelay: '<S77>/Unit Delay'
     */
    if (rtb_AND_eqpk) {
        GEN_ManualTorMinTurnOnDelay_sec = fmaxf(GEN_ManualTorMinTurnOnDelay_sec,
                                                -LCFRCV_TSysCycleTimeSen_sec) -
                                          LCFRCV_TSysCycleTimeSen_sec;
    } else {
        GEN_ManualTorMinTurnOnDelay_sec = TJAGEN_ManualTorqueESMaxTime_sec;
    }

    /* End of Switch: '<S77>/Switch' */

    /* Logic: '<S77>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S77>/LessThanOrEqual'
     *  UnaryMinus: '<S77>/Unary Minus1'
     *  UnitDelay: '<S77>/Unit Delay'
     */
    rtb_AND_eqpk = (rtb_AND_eqpk && (GEN_ManualTorMinTurnOnDelay_sec <=
                                     (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S52>/OR5' incorporates:
     *  Logic: '<S52>/NOT'
     */
    TJAGEN_SuspendEnd_bool = ((!rtb_OR_imzg_idx_1) && rtb_AND_eqpk);

    /* UnitDelay: '<S552>/Unit Delay' */
    STM_PrevRAMPOUT_bool = GEN_PrevRampoutNUnitDelay_bool;

    /* Switch: '<S555>/Switch1' incorporates:
     *  Constant: '<S555>/Constant1'
     *  Constant: '<S555>/Constant10'
     *  Constant: '<S555>/Constant11'
     *  Constant: '<S555>/Constant12'
     *  Constant: '<S555>/Constant13'
     *  Constant: '<S555>/Constant14'
     *  Constant: '<S555>/Constant2'
     *  Constant: '<S555>/Constant3'
     *  Constant: '<S555>/Constant4'
     *  Constant: '<S555>/Constant5'
     *  Constant: '<S555>/Constant6'
     *  Constant: '<S555>/Constant7'
     *  Constant: '<S555>/Constant8'
     *  Constant: '<S555>/Constant9'
     *  Inport: '<Root>/Inport107'
     *  Logic: '<S555>/OR'
     *  Logic: '<S555>/OR1'
     *  RelationalOperator: '<S555>/Equal'
     *  RelationalOperator: '<S555>/Equal1'
     *  RelationalOperator: '<S555>/Equal2'
     *  RelationalOperator: '<S555>/Equal3'
     *  RelationalOperator: '<S555>/Equal4'
     *  RelationalOperator: '<S555>/Equal5'
     *  RelationalOperator: '<S555>/Equal6'
     *  RelationalOperator: '<S555>/Equal7'
     *  RelationalOperator: '<S555>/Equal8'
     *  RelationalOperator: '<S555>/Equal9'
     *  Switch: '<S555>/Switch2'
     *  Switch: '<S555>/Switch3'
     */
    if (TJASTM_EnableUseACCState_bool) {
        rtb_TJASTM_ACCIsOFF_bool = (((int32_T)VLCVEH_ACCStatus_nu) == 0);
        rtb_SLC_LeAdjLaneValid_bool =
            (((((((int32_T)VLCVEH_ACCStatus_nu) == 2) ||
                (((int32_T)VLCVEH_ACCStatus_nu) == 3)) ||
               (((int32_T)VLCVEH_ACCStatus_nu) == 4)) ||
              (((int32_T)VLCVEH_ACCStatus_nu) == 5)) ||
             (((int32_T)VLCVEH_ACCStatus_nu) == 1));
        rtb_SLC_RiTurnSignalOn_bool =
            ((((((int32_T)VLCVEH_ACCStatus_nu) == 2) ||
               (((int32_T)VLCVEH_ACCStatus_nu) == 3)) ||
              (((int32_T)VLCVEH_ACCStatus_nu) == 4)) ||
             (((int32_T)VLCVEH_ACCStatus_nu) == 5));
    } else {
        rtb_TJASTM_ACCIsOFF_bool = false;
        rtb_SLC_LeAdjLaneValid_bool = true;
        rtb_SLC_RiTurnSignalOn_bool = true;
    }

    /* End of Switch: '<S555>/Switch1' */

    /* RelationalOperator: '<S8>/Equal' incorporates:
     *  Constant: '<S8>/Constant1'
     *  Inport: '<Root>/Inport123'
     */
    rtb_Equal_blkb = (((int32_T)ADCS4_AVM_Sts) == 2);

    /* Chart: '<S8>/StateMachineTJA' incorporates:
     *  Constant: '<S8>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport106'
     *  UnitDelay: '<S552>/Unit Delay1'
     *  UnitDelay: '<S8>/Unit Delay1'
     */
    if (((uint32_T)TJASA_DW.is_active_c20_TJASA) == 0U) {
        TJASA_DW.is_active_c20_TJASA = 1U;
        if (!TJAGEN_CodeFunction_bool) {
            TJASA_DW.is_c20_TJASA = TJASA_IN_NOT_PRESENT;

            /* RelationalOperator: '<S552>/Equal3' incorporates:
             *  SignalConversion: '<S8>/Signal Conversion'
             */
            TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
        } else if (TJAGEN_Error_bool) {
            TJASA_DW.is_c20_TJASA = TJASA_IN_ERROR;

            /* RelationalOperator: '<S552>/Equal3' incorporates:
             *  SignalConversion: '<S8>/Signal Conversion'
             */
            TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
        } else {
            /* default */
            TJASA_DW.is_c20_TJASA = TJASA_IN_OFF;

            /* RelationalOperator: '<S552>/Equal3' incorporates:
             *  SignalConversion: '<S8>/Signal Conversion'
             */
            TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_OFF;
        }
    } else {
        guard1 = false;
        switch (TJASA_DW.is_c20_TJASA) {
            case TJASA_IN_ACTIVE:
                if (rtb_TJASTM_ACCIsOFF_bool) {
                    TJASA_DW.is_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c20_TJASA = TJASA_IN_OFF;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_OFF;
                } else if (TJAGEN_Error_bool) {
                    TJASA_DW.is_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c20_TJASA = TJASA_IN_ERROR;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
                } else if (((((((!TJAGEN_FunctionSwitch_bool) &&
                                (TJASTM_EnableFS_C_bool)) ||
                               TJAGEN_FunctionQuit_bool) ||
                              (GEN_PrevSusQuitUnitDelay_bool)) ||
                             LCFRCV_PilotOffLeverSwitch_bool) ||
                            (!rtb_SLC_RiTurnSignalOn_bool)) ||
                           (!TJAGEN_CodeFunction_bool)) {
                    TJASA_DW.is_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c20_TJASA = TJASA_IN_NOT_ACTIVE;
                    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_PASSIVE;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
                } else if (((int32_T)TJASA_DW.is_ACTIVE) == 1) {
                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING;
                    if ((((TJAGEN_SuspendStart_bool ||
                           (((rtb_AND18 && rtb_GreaterThan_bpjw_idx_1) &&
                             rtb_GreaterThan_bpjw_idx_0) &&
                            rtb_SLC_RiAdjLaneValid_bool)) ||
                          (!TJAGEN_StrongReady_bool)) ||
                         rtb_Equal_awdj) ||
                        (STM_Cancel_bool)) {
                        TJASA_DW.is_ACTIVE = TJASA_IN_SUSPENDED;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED;
                    }
                } else {
                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    /* case IN_SUSPENDED: */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED;
                    if (((((TJAGEN_SuspendEnd_bool &&
                            (!STM_PrevRAMPOUT_bool)) &&
                           TJAGEN_Clearance_bool) &&
                          TJAGEN_StrongReady_bool) &&
                         TJAGEN_WeakReady_bool) &&
                        ((rtb_TTG_CMB_bool ||
                          (TJAOBF_StrongReady_bool && TJAOBF_WeakReady_bool)) ||
                         (TJACMB_StrongReady_bool && TJACMB_WeakReady_bool))) {
                        TJASA_DW.is_ACTIVE = TJASA_IN_CONTROLLING;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING;
                    }
                }
                break;

            case TJASA_IN_ERROR:
                /* RelationalOperator: '<S552>/Equal3' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion'
                 */
                TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
                if ((!TJAGEN_CodeFunction_bool) || (!TJAGEN_Error_bool)) {
                    if (!TJAGEN_CodeFunction_bool) {
                        TJASA_DW.is_c20_TJASA = TJASA_IN_NOT_PRESENT;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
                    } else if (TJAGEN_Error_bool) {
                        TJASA_DW.is_c20_TJASA = TJASA_IN_ERROR;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
                    } else {
                        /* default */
                        guard1 = true;
                    }
                } else {
                    if (rtb_TJASTM_ACCIsOFF_bool) {
                        guard1 = true;
                    }
                }
                break;

            case TJASA_IN_NOT_ACTIVE:
                if (rtb_TJASTM_ACCIsOFF_bool) {
                    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c20_TJASA = TJASA_IN_OFF;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_OFF;
                } else if (TJAGEN_Error_bool) {
                    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
                    TJASA_DW.is_c20_TJASA = TJASA_IN_ERROR;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
                } else {
                    switch (TJASA_DW.is_NOT_ACTIVE) {
                        case TJASA_IN_PASSIVE:
                            /* RelationalOperator: '<S552>/Equal3' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion'
                             */
                            TJASTM_SysStateTJAIn_nu =
                                E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
                            if (((((((!TJAGEN_FunctionQuit_bool) &&
                                     TJAGEN_Clearance_bool) &&
                                    TJAGEN_CodeFunction_bool) &&
                                   TJAGEN_StrongReady_bool) &&
                                  TJAGEN_WeakReady_bool) &&
                                 (((rtb_TTG_CMB_bool || rtb_Equal_blkb) ||
                                   (TJAOBF_StrongReady_bool &&
                                    TJAOBF_WeakReady_bool)) ||
                                  (TJACMB_StrongReady_bool &&
                                   TJACMB_WeakReady_bool))) &&
                                rtb_SLC_LeAdjLaneValid_bool) {
                                TJASA_DW.is_NOT_ACTIVE = TJASA_IN_STANDBY;

                                /* RelationalOperator: '<S552>/Equal3'
                                 * incorporates: SignalConversion: '<S8>/Signal
                                 * Conversion'
                                 */
                                TJASTM_SysStateTJAIn_nu =
                                    E_TJASTM_SysStateTJA_nu_SYSST_STANDBY;
                            }
                            break;

                        case TJASA_IN_STANDBY:
                            /* RelationalOperator: '<S552>/Equal3' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion'
                             */
                            TJASTM_SysStateTJAIn_nu =
                                E_TJASTM_SysStateTJA_nu_SYSST_STANDBY;
                            if ((((((TJAGEN_FunctionQuit_bool ||
                                     rtb_Equal_awdj) ||
                                    (!TJAGEN_CodeFunction_bool)) ||
                                   (!TJAGEN_StrongReady_bool)) ||
                                  (!TJAGEN_WeakReady_bool)) ||
                                 (!rtb_SLC_LeAdjLaneValid_bool)) ||
                                (((rtb_SLC_VelocityValid_bool &&
                                   (!rtb_Equal_blkb)) &&
                                  rtb_NotEqual_ozi5_tmp_0) &&
                                 rtb_Equal_i3l2_idx_0)) {
                                TJASA_DW.is_NOT_ACTIVE = TJASA_IN_PASSIVE;

                                /* RelationalOperator: '<S552>/Equal3'
                                 * incorporates: SignalConversion: '<S8>/Signal
                                 * Conversion'
                                 */
                                TJASTM_SysStateTJAIn_nu =
                                    E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
                            } else {
                                rtb_NotEqual_ozi5_tmp_0 =
                                    ((TJAGEN_FunctionSwitch_bool &&
                                      (TJASTM_EnableFS_C_bool)) ||
                                     LCFRCV_PilotOnLeverSwitch_bool);
                                if (rtb_NotEqual_ozi5_tmp_0 &&
                                    (!rtb_SLC_RiTurnSignalOn_bool)) {
                                    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_WAITACC;

                                    /* RelationalOperator: '<S552>/Equal3'
                                     * incorporates: SignalConversion:
                                     * '<S8>/Signal Conversion'
                                     */
                                    TJASTM_SysStateTJAIn_nu =
                                        E_TJASTM_SysStateTJA_nu_SYSST_WAITACC;
                                } else {
                                    if (rtb_NotEqual_ozi5_tmp_0 &&
                                        rtb_SLC_RiTurnSignalOn_bool) {
                                        TJASA_DW.is_NOT_ACTIVE =
                                            TJASA_IN_NO_ACTIVE_CHILD;
                                        TJASA_DW.is_c20_TJASA = TJASA_IN_ACTIVE;
                                        TJASA_DW.is_ACTIVE =
                                            TJASA_IN_CONTROLLING;

                                        /* RelationalOperator: '<S552>/Equal3'
                                         * incorporates: SignalConversion:
                                         * '<S8>/Signal Conversion'
                                         */
                                        TJASTM_SysStateTJAIn_nu =
                                            E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING;
                                    }
                                }
                            }
                            break;

                        default:
                            /* RelationalOperator: '<S552>/Equal3' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion'
                             */
                            /* case IN_WAITACC: */
                            TJASTM_SysStateTJAIn_nu =
                                E_TJASTM_SysStateTJA_nu_SYSST_WAITACC;
                            if ((((((((((TJAGEN_FunctionQuit_bool ||
                                         (!TJAGEN_Clearance_bool)) ||
                                        (!TJAGEN_CodeFunction_bool)) ||
                                       (!TJAGEN_StrongReady_bool)) ||
                                      (!TJAGEN_WeakReady_bool)) ||
                                     (!rtb_SLC_LeAdjLaneValid_bool)) ||
                                    (((((!TJALKA_StrongReady_bool) ||
                                        (!TJALKA_WeakReady_bool)) &&
                                       (!rtb_Equal_blkb)) &&
                                      ((!TJAOBF_StrongReady_bool) ||
                                       (!TJAOBF_WeakReady_bool))) &&
                                     ((!TJACMB_StrongReady_bool) ||
                                      (!TJACMB_WeakReady_bool)))) ||
                                   ((!TJAGEN_FunctionSwitch_bool) &&
                                    (TJASTM_EnableFS_C_bool))) ||
                                  TJAGEN_FunctionQuit_bool) ||
                                 LCFRCV_PilotOffLeverSwitch_bool) ||
                                (TJASTM_PrevACCActOvrtm_bool)) {
                                TJASA_DW.is_NOT_ACTIVE = TJASA_IN_PASSIVE;

                                /* RelationalOperator: '<S552>/Equal3'
                                 * incorporates: SignalConversion: '<S8>/Signal
                                 * Conversion'
                                 */
                                TJASTM_SysStateTJAIn_nu =
                                    E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
                            } else {
                                if (rtb_SLC_RiTurnSignalOn_bool) {
                                    TJASA_DW.is_NOT_ACTIVE =
                                        TJASA_IN_NO_ACTIVE_CHILD;
                                    TJASA_DW.is_c20_TJASA = TJASA_IN_ACTIVE;
                                    TJASA_DW.is_ACTIVE = TJASA_IN_CONTROLLING;

                                    /* RelationalOperator: '<S552>/Equal3'
                                     * incorporates: SignalConversion:
                                     * '<S8>/Signal Conversion'
                                     */
                                    TJASTM_SysStateTJAIn_nu =
                                        E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING;
                                }
                            }
                            break;
                    }
                }
                break;

            case TJASA_IN_NOT_PRESENT:
                /* RelationalOperator: '<S552>/Equal3' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion'
                 */
                TJASTM_SysStateTJAIn_nu =
                    E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
                if (TJAGEN_CodeFunction_bool) {
                    if (!TJAGEN_CodeFunction_bool) {
                        TJASA_DW.is_c20_TJASA = TJASA_IN_NOT_PRESENT;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
                    } else if (TJAGEN_Error_bool) {
                        TJASA_DW.is_c20_TJASA = TJASA_IN_ERROR;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_ERROR;
                    } else {
                        /* default */
                        TJASA_DW.is_c20_TJASA = TJASA_IN_OFF;

                        /* RelationalOperator: '<S552>/Equal3' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion'
                         */
                        TJASTM_SysStateTJAIn_nu =
                            E_TJASTM_SysStateTJA_nu_SYSST_OFF;
                    }
                }
                break;

            default:
                /* RelationalOperator: '<S552>/Equal3' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion'
                 */
                /* case IN_OFF: */
                TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_OFF;
                if (!rtb_TJASTM_ACCIsOFF_bool) {
                    TJASA_DW.is_c20_TJASA = TJASA_IN_NOT_ACTIVE;
                    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_PASSIVE;

                    /* RelationalOperator: '<S552>/Equal3' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion'
                     */
                    TJASTM_SysStateTJAIn_nu =
                        E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
                }
                break;
        }

        if (guard1) {
            TJASA_DW.is_c20_TJASA = TJASA_IN_OFF;

            /* RelationalOperator: '<S552>/Equal3' incorporates:
             *  SignalConversion: '<S8>/Signal Conversion'
             */
            TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_OFF;
        }
    }

    /* RelationalOperator: '<S8>/Equal2' incorporates:
     *  Constant: '<S548>/Constant'
     *  SignalConversion: '<S8>/Signal Conversion'
     */
    rtb_SLC_RiTurnSignalOn_bool = (((uint32_T)TJASTM_SysStateTJAIn_nu) ==
                                   E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING);

    /* RelationalOperator: '<S552>/Equal' incorporates:
     *  Constant: '<S592>/Constant'
     *  SignalConversion: '<S8>/Signal Conversion'
     */
    rtb_TJASTM_ACCIsOFF_bool = (((uint32_T)TJASTM_SysStateTJAIn_nu) !=
                                E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED);

    /* Logic: '<S552>/NOT' incorporates:
     *  Logic: '<S552>/NOT2'
     */
    rtb_NotEqual_ozi5_tmp_0 = !rtb_TJASTM_ACCIsOFF_bool;

    /* Switch: '<S602>/Switch2' incorporates:
     *  Constant: '<S599>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S552>/NOT'
     *  Logic: '<S599>/AND'
     *  Logic: '<S599>/NOT'
     *  RelationalOperator: '<S602>/GreaterThan'
     *  Switch: '<S602>/Switch'
     *  UnitDelay: '<S599>/Unit Delay'
     *  UnitDelay: '<S602>/Unit Delay'
     */
    if (rtb_NotEqual_ozi5_tmp_0 && (!GEN_PrevSusUnitDelay_bool)) {
        GEN_SusTimeExpiredTimerRetrigger_sec = TJAGEN_RampoutTimeMax_C_sec;
    } else if (GEN_SusTimeExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S602>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S602>/Subtract'
         *  UnitDelay: '<S602>/Unit Delay'
         */
        GEN_SusTimeExpiredTimerRetrigger_sec =
            GEN_SusTimeExpiredTimerRetrigger_sec - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S602>/Unit Delay' incorporates:
         *  Constant: '<S602>/Constant1'
         *  Switch: '<S602>/Switch'
         */
        GEN_SusTimeExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S602>/Switch2' */

    /* Switch: '<S601>/Switch' incorporates:
     *  Constant: '<S601>/Constant2'
     *  Constant: '<S602>/Constant2'
     *  Logic: '<S552>/NOT'
     *  Logic: '<S599>/NOT1'
     *  Logic: '<S599>/NOT2'
     *  RelationalOperator: '<S602>/GreaterThan1'
     *  UnitDelay: '<S601>/Unit Delay'
     *  UnitDelay: '<S602>/Unit Delay'
     */
    if (!rtb_NotEqual_ozi5_tmp_0) {
        GEN_SusTimeExpiredRSFF_bool = false;
    } else {
        GEN_SusTimeExpiredRSFF_bool =
            ((GEN_SusTimeExpiredTimerRetrigger_sec <= 0.0F) ||
             (GEN_SusTimeExpiredRSFF_bool));
    }

    /* End of Switch: '<S601>/Switch' */

    /* Switch: '<S552>/Switch3' incorporates:
     *  Constant: '<S594>/Constant'
     *  Constant: '<S597>/Constant'
     *  RelationalOperator: '<S552>/Equal2'
     *  RelationalOperator: '<S552>/Equal3'
     *  Switch: '<S552>/Switch2'
     */
    if (((uint32_T)TJASTM_SysStateTJAIn_nu) ==
        E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT) {
        /* Switch: '<S552>/Switch3' incorporates:
         *  Constant: '<S598>/Constant'
         */
        TJASTM_SysStateTJA_nu = E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE;
    } else if (((uint32_T)TJASTM_SysStateTJAIn_nu) !=
               E_TJASTM_SysStateTJA_nu_SYSST_WAITACC) {
        /* Switch: '<S552>/Switch1' incorporates:
         *  Logic: '<S552>/OR'
         *  Switch: '<S552>/Switch2'
         *  UnitDelay: '<S601>/Unit Delay'
         */
        if ((rtb_TJASTM_ACCIsOFF_bool || (GEN_SusTimeExpiredRSFF_bool)) ||
            rtb_Equal_awdj) {
            /* Switch: '<S552>/Switch3' */
            TJASTM_SysStateTJA_nu = TJASTM_SysStateTJAIn_nu;
        } else {
            /* Switch: '<S552>/Switch3' incorporates:
             *  Constant: '<S593>/Constant'
             */
            TJASTM_SysStateTJA_nu = E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT;
        }

        /* End of Switch: '<S552>/Switch1' */
    } else {
        /* Switch: '<S552>/Switch3' incorporates:
         *  Constant: '<S595>/Constant'
         *  Switch: '<S552>/Switch2'
         */
        TJASTM_SysStateTJA_nu = E_TJASTM_SysStateTJA_nu_SYSST_STANDBY;
    }

    /* RelationalOperator: '<S552>/Equal1' incorporates:
     *  Constant: '<S596>/Constant'
     *  Switch: '<S552>/Switch3'
     *  UnitDelay: '<S552>/Unit Delay'
     */
    GEN_PrevRampoutNUnitDelay_bool = (((uint32_T)TJASTM_SysStateTJA_nu) ==
                                      E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT);

    /* Chart: '<S8>/LatCtrlMode' incorporates:
     *  UnitDelay: '<S552>/Unit Delay'
     */
    if (((uint32_T)TJASA_DW.is_active_c19_TJASA) == 0U) {
        TJASA_DW.is_active_c19_TJASA = 1U;
        TJASA_DW.is_c19_TJASA = TJASA_IN_Passive_nudl;

        /* RelationalOperator: '<S665>/Equal' incorporates:
         *  SignalConversion: '<S8>/Signal Conversion1'
         */
        TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    } else if (((int32_T)TJASA_DW.is_c19_TJASA) == 1) {
        if ((!rtb_SLC_RiTurnSignalOn_bool) &&
            (!GEN_PrevRampoutNUnitDelay_bool)) {
            TJASA_DW.is_Controlling = TJASA_IN_NO_ACTIVE_CHILD;
            TJASA_DW.is_c19_TJASA = TJASA_IN_Passive_nudl;

            /* RelationalOperator: '<S665>/Equal' incorporates:
             *  SignalConversion: '<S8>/Signal Conversion1'
             */
            TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
        } else {
            switch (TJASA_DW.is_Controlling) {
                case TJASA_IN_Combined:
                    /* RelationalOperator: '<S665>/Equal' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion1'
                     */
                    TJASTM_LatCtrlMode_nu =
                        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB;
                    if ((rtb_SLC_RiAdjLaneValid_bool &&
                         TJAOBF_StrongReady_bool) &&
                        TJAOBF_WeakReady_bool) {
                        TJASA_DW.is_Controlling = TJASA_IN_ObjectFollowing;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF;
                    } else if (TJALKA_StrongReady_bool &&
                               rtb_SLC_RiAdjLaneValid_bool) {
                        TJASA_DW.is_Controlling = TJASA_IN_LaneCentering;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC;
                    } else {
                        if (TJASLC_StrongReady_bool && TJASLC_WeakReady_bool) {
                            TJASA_DW.is_Controlling = TJASA_IN_LaneChange;

                            /* RelationalOperator: '<S665>/Equal' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion1'
                             */
                            TJASTM_LatCtrlMode_nu =
                                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC;
                        }
                    }
                    break;

                case TJASA_IN_LaneCentering:
                    /* RelationalOperator: '<S665>/Equal' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion1'
                     */
                    TJASTM_LatCtrlMode_nu =
                        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC;
                    if (TJASLC_StrongReady_bool && TJASLC_WeakReady_bool) {
                        TJASA_DW.is_Controlling = TJASA_IN_LaneChange;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC;
                    } else if (((rtb_AND_ixtq && TJAOBF_StrongReady_bool) &&
                                TJAOBF_WeakReady_bool) &&
                               rtb_Equal_i3l2_idx_0) {
                        TJASA_DW.is_Controlling = TJASA_IN_ObjectFollowing;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF;
                    } else {
                        if (TJACMB_StrongReady_bool && TJACMB_WeakReady_bool) {
                            TJASA_DW.is_Controlling = TJASA_IN_Combined;

                            /* RelationalOperator: '<S665>/Equal' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion1'
                             */
                            TJASTM_LatCtrlMode_nu =
                                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB;
                        }
                    }
                    break;

                case TJASA_IN_LaneChange:
                    /* RelationalOperator: '<S665>/Equal' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion1'
                     */
                    TJASTM_LatCtrlMode_nu =
                        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC;
                    if ((rtb_Equal_i3l2_idx_1 && TJALKA_StrongReady_bool) &&
                        TJALKA_WeakReady_bool) {
                        TJASA_DW.is_Controlling = TJASA_IN_LaneCentering;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC;
                    }
                    break;

                default:
                    /* RelationalOperator: '<S665>/Equal' incorporates:
                     *  SignalConversion: '<S8>/Signal Conversion1'
                     */
                    /* case IN_ObjectFollowing: */
                    TJASTM_LatCtrlMode_nu =
                        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF;
                    if ((rtb_TTG_CMB_bool && rtb_SLC_LeTurnSignalOn_bool) &&
                        rtb_Equal_i3l2_idx_0) {
                        TJASA_DW.is_Controlling = TJASA_IN_LaneCentering;

                        /* RelationalOperator: '<S665>/Equal' incorporates:
                         *  SignalConversion: '<S8>/Signal Conversion1'
                         */
                        TJASTM_LatCtrlMode_nu =
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC;
                    } else {
                        if (TJACMB_StrongReady_bool && TJACMB_WeakReady_bool) {
                            TJASA_DW.is_Controlling = TJASA_IN_Combined;

                            /* RelationalOperator: '<S665>/Equal' incorporates:
                             *  SignalConversion: '<S8>/Signal Conversion1'
                             */
                            TJASTM_LatCtrlMode_nu =
                                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB;
                        }
                    }
                    break;
            }
        }
    } else {
        /* RelationalOperator: '<S665>/Equal' incorporates:
         *  SignalConversion: '<S8>/Signal Conversion1'
         */
        /* case IN_Passive: */
        TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
        if (rtb_SLC_RiTurnSignalOn_bool) {
            TJASA_DW.is_c19_TJASA = TJASA_IN_Controlling;
            if (rtb_TTG_CMB_bool && rtb_Equal_i3l2_idx_0) {
                TJASA_DW.is_Controlling = TJASA_IN_LaneCentering;

                /* RelationalOperator: '<S665>/Equal' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion1'
                 */
                TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC;
            } else if (((rtb_SLC_VelocityValid_bool && rtb_Equal_i3l2_idx_0) &&
                        TJAOBF_StrongReady_bool) &&
                       TJAOBF_WeakReady_bool) {
                TJASA_DW.is_Controlling = TJASA_IN_ObjectFollowing;

                /* RelationalOperator: '<S665>/Equal' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion1'
                 */
                TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF;
            } else {
                /* [SR_CMB && WR_CMB] */
                TJASA_DW.is_Controlling = TJASA_IN_Combined;

                /* RelationalOperator: '<S665>/Equal' incorporates:
                 *  SignalConversion: '<S8>/Signal Conversion1'
                 */
                TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB;
            }
        }
    }

    /* Logic: '<S665>/AND' incorporates:
     *  Constant: '<S665>/Constant'
     *  Constant: '<S679>/Constant'
     *  RelationalOperator: '<S665>/Equal'
     */
    TTG_CMBObjectCorridor_bool = (((TJACMB_CombinedDataEnable_C_bool) &&
                                   (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) &&
                                  TJACMB_ObjectCorridor_bool);

    /* RelationalOperator: '<S463>/NotEqual' incorporates:
     *  RelationalOperator: '<S378>/Equal17'
     *  RelationalOperator: '<S378>/Equal18'
     *  RelationalOperator: '<S838>/Equal9'
     *  RelationalOperator: '<S839>/Equal10'
     *  RelationalOperator: '<S839>/Equal9'
     *  RelationalOperator: '<S888>/Equal11'
     *  SignalConversion: '<S314>/Signal Conversion'
     *  Switch: '<S888>/Switch'
     *  Switch: '<S888>/Switch1'
     *  Switch: '<S888>/Switch3'
     */
    SLC_PreAbortState_enum = SLC_AbortState_enum;

    /* Logic: '<S839>/OR' incorporates:
     *  Constant: '<S850>/Constant'
     *  Constant: '<S851>/Constant'
     *  Constant: '<S852>/Constant'
     *  Constant: '<S853>/Constant'
     *  Constant: '<S854>/Constant'
     *  Constant: '<S855>/Constant'
     *  Constant: '<S856>/Constant'
     *  Constant: '<S857>/Constant'
     *  Constant: '<S858>/Constant'
     *  Constant: '<S859>/Constant'
     *  Constant: '<S860>/Constant'
     *  Logic: '<S839>/AND'
     *  Logic: '<S839>/OR1'
     *  Logic: '<S839>/OR2'
     *  Logic: '<S839>/OR3'
     *  Logic: '<S839>/OR4'
     *  RelationalOperator: '<S839>/Equal'
     *  RelationalOperator: '<S839>/Equal1'
     *  RelationalOperator: '<S839>/Equal10'
     *  RelationalOperator: '<S839>/Equal2'
     *  RelationalOperator: '<S839>/Equal3'
     *  RelationalOperator: '<S839>/Equal4'
     *  RelationalOperator: '<S839>/Equal5'
     *  RelationalOperator: '<S839>/Equal6'
     *  RelationalOperator: '<S839>/Equal7'
     *  RelationalOperator: '<S839>/Equal8'
     *  RelationalOperator: '<S839>/Equal9'
     */
    rtb_SLC_RiTurnSignalOn_bool =
        (((((((E_TJASLC_AbortState_nu_ABORT_NOACTIVE ==
               ((uint32_T)SLC_PreAbortState_enum)) ||
              (((uint32_T)SLC_PreAbortState_enum) ==
               E_TJASLC_AbortState_nu_ABORT_ABORT)) &&
             (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
              ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
            (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC_RQ ==
             ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
           (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB ==
            ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
          (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ ==
           ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
         ((((uint32_T)TJASTM_LatCtrlMode_nu) ==
           E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
          ((((((uint32_T)SLC_PrevManeuverState2_Enum) ==
              E_TJASLC_ManeuverState_nu_LCPSTART) ||
             (((uint32_T)SLC_PrevManeuverState2_Enum) ==
              E_TJASLC_ManeuverState_nu_NEWEGO)) ||
            (((uint32_T)SLC_PrevManeuverState2_Enum) ==
             E_TJASLC_ManeuverState_nu_LCMEND)) ||
           (((uint32_T)SLC_PrevManeuverState2_Enum) ==
            E_TJASLC_ManeuverState_nu_ABORT))));

    /* RelationalOperator: '<S838>/Equal9' incorporates:
     *  Constant: '<S849>/Constant'
     */
    rtb_Equal_blkb = (E_TJASLC_AbortState_nu_ABORT_NEWEGO ==
                      ((uint32_T)SLC_PreAbortState_enum));

    /* SignalConversion: '<S314>/Signal Conversion1' */
    SLC_LaneChangeDirectionAbort_enum = TJASA_B.SLC_LaneChangeDirection;

    /* RelationalOperator: '<S838>/Equal7' incorporates:
     *  Constant: '<S847>/Constant'
     *  SignalConversion: '<S8>/Signal Conversion1'
     */
    rtb_OR_imjz = (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
                   ((uint32_T)TJASTM_LatCtrlMode_nu));

    /* Logic: '<S838>/AND' incorporates:
     *  Constant: '<S844>/Constant'
     *  Constant: '<S845>/Constant'
     *  Constant: '<S846>/Constant'
     *  Logic: '<S838>/OR2'
     *  RelationalOperator: '<S838>/Equal4'
     *  RelationalOperator: '<S838>/Equal5'
     *  RelationalOperator: '<S838>/Equal6'
     */
    rtb_OR_bsb5 = ((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                    E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
                   ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
                     E_TJASLC_ManeuverState_nu_LATMVSTART) ||
                    (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                     E_TJASLC_ManeuverState_nu_LCMSTART)));

    /* RelationalOperator: '<S838>/Equal1' incorporates:
     *  SignalConversion: '<S264>/Signal Conversion'
     */
    TJASLC_LCDirection_enum = TJASA_B.LCDirection_enum;

    /* RelationalOperator: '<S536>/Equal' incorporates:
     *  RelationalOperator: '<S838>/Equal3'
     *  RelationalOperator: '<S838>/Equal8'
     *  SignalConversion: '<S314>/Signal Conversion1'
     */
    SLC_PrevLaneChangeTrigger_nu = SLC_LaneChangeDirectionAbort_enum;

    /* Logic: '<S838>/AND3' incorporates:
     *  Constant: '<S841>/Constant'
     *  Constant: '<S843>/Constant'
     *  Logic: '<S838>/AND1'
     *  Logic: '<S838>/AND4'
     *  RelationalOperator: '<S838>/Equal1'
     *  RelationalOperator: '<S838>/Equal3'
     */
    rtb_TJASTM_ACCIsOFF_bool =
        (((rtb_Equal_blkb && (((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
                              E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG)) &&
          rtb_OR_imjz) ||
         (rtb_OR_bsb5 && (((uint32_T)TJASLC_LCDirection_enum) ==
                          E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG)));

    /* Logic: '<S838>/AND6' incorporates:
     *  Constant: '<S842>/Constant'
     *  Constant: '<S848>/Constant'
     *  Logic: '<S838>/AND2'
     *  Logic: '<S838>/AND5'
     *  RelationalOperator: '<S838>/Equal2'
     *  RelationalOperator: '<S838>/Equal8'
     */
    rtb_SLC_LeAdjLaneValid_bool =
        ((((((uint32_T)SLC_PrevLaneChangeTrigger_nu) ==
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG) &&
           rtb_Equal_blkb) &&
          rtb_OR_imjz) ||
         (rtb_OR_bsb5 && (((uint32_T)TJASLC_LCDirection_enum) ==
                          E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG)));

    /* Switch: '<S840>/Switch' incorporates:
     *  Constant: '<S840>/Constant'
     *  Switch: '<S840>/Switch1'
     *  Switch: '<S840>/Switch2'
     */
    if (rtb_SLC_RiTurnSignalOn_bool) {
        rtb_DataTypeConversion1 = 0U;
    } else if (rtb_TJASTM_ACCIsOFF_bool) {
        /* Switch: '<S840>/Switch1' incorporates:
         *  Constant: '<S840>/Constant1'
         */
        rtb_DataTypeConversion1 = 1U;
    } else if (rtb_SLC_LeAdjLaneValid_bool) {
        /* Switch: '<S840>/Switch2' incorporates:
         *  Constant: '<S840>/Constant2'
         *  Switch: '<S840>/Switch1'
         */
        rtb_DataTypeConversion1 = 2U;
    } else {
        /* Switch: '<S840>/Switch1' incorporates:
         *  Constant: '<S840>/Constant3'
         *  Switch: '<S840>/Switch2'
         */
        rtb_DataTypeConversion1 = 3U;
    }

    /* End of Switch: '<S840>/Switch' */

    /* Logic: '<S686>/AND' incorporates:
     *  Constant: '<S686>/Constant'
     */
    rtb_AND_ixtq =
        ((TJATTG_NewPredEnable_C_bool) && TJALKA_LanePredictValid_bool);

    /* Switch: '<S743>/Switch' incorporates:
     *  Constant: '<S686>/Constant1'
     *  Logic: '<S686>/AND1'
     *  Logic: '<S686>/NOT'
     *  Logic: '<S686>/NOT1'
     *  Logic: '<S686>/OR'
     *  UnitDelay: '<S667>/Unit Delay'
     */
    if ((!TJALKA_LanePredictValid_bool) ||
        ((!TJALKA_RampoutPredictOn_C_bool) &&
         (TTG_PrevLnLengResetUnitDelay_bool))) {
        /* Switch: '<S743>/Switch' incorporates:
         *  Constant: '<S743>/Constant2'
         */
        TJATTG_PredictionEnable_bool = false;
    } else {
        /* Switch: '<S743>/Switch' incorporates:
         *  Logic: '<S742>/AND'
         *  Logic: '<S742>/NOT'
         *  UnitDelay: '<S742>/Unit Delay'
         *  UnitDelay: '<S743>/Unit Delay'
         */
        TJATTG_PredictionEnable_bool =
            ((rtb_AND_ixtq && (!TTG_LanePredictEdgeRising_bool)) ||
             (TTG_PredictEnableRSFF_bool));
    }

    /* End of Switch: '<S743>/Switch' */

    /* Product: '<S685>/Product' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport78'
     */
    rtb_Subtract_if0w_idx_1 = VDy_VehYawRate_rps * LCFRCV_TSysCycleTimeSen_sec;

    /* Logic: '<S741>/AND' incorporates:
     *  Logic: '<S741>/NOT'
     *  UnitDelay: '<S741>/Unit Delay'
     */
    rtb_OR_bsb5 =
        (TJATTG_PredictionEnable_bool && (!TTG_PredictEnableEdgeRising_bool));

    /* Switch: '<S738>/Switch' incorporates:
     *  UnaryMinus: '<S685>/Unary Minus1'
     *  UnitDelay: '<S738>/Unit Delay1'
     */
    if (rtb_OR_bsb5) {
        rtb_Add_fbcu = -rtb_Subtract_if0w_idx_1;
    } else {
        rtb_Add_fbcu = TTG_OdoYawDelayRe_rad;
    }

    /* End of Switch: '<S738>/Switch' */

    /* Product: '<S685>/Product3' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Inport: '<Root>/Inport79'
     *  Product: '<S126>/Product'
     *  Switch: '<S126>/Switch'
     */
    rtb_Subtract_if0w_idx_0 = LCFRCV_TSysCycleTimeSen_sec * VDy_VehVelX_mps;

    /* Product: '<S685>/Product1' incorporates:
     *  Product: '<S685>/Product3'
     *  Trigonometry: '<S685>/Cos'
     *  UnaryMinus: '<S685>/Unary Minus2'
     */
    rtb_Abs_hlug = cosf(-rtb_Add_fbcu) * rtb_Subtract_if0w_idx_0;

    /* Switch: '<S740>/Switch' incorporates:
     *  UnitDelay: '<S740>/Unit Delay1'
     */
    if (rtb_OR_bsb5) {
        rtb_uDLookupTable1_eoam = rtb_Abs_hlug;
    } else {
        rtb_uDLookupTable1_eoam = TTG_OdoPosXDelayRe_met;
    }

    /* End of Switch: '<S740>/Switch' */

    /* Switch: '<S725>/Switch' incorporates:
     *  Inport: '<Root>/Inport29'
     *  Inport: '<Root>/Inport30'
     *  Switch: '<S726>/Switch'
     *  UnitDelay: '<S725>/Unit Delay'
     *  UnitDelay: '<S726>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_RiLnCrvUnitDelay_1pm = ABPR_RiLnClthCrv_1pm;
        TTG_RiLnHeadingUnitDelay_rad = ABPR_RiLnClthHeading_rad;
    }

    /* End of Switch: '<S725>/Switch' */

    /* Product: '<S685>/Product2' incorporates:
     *  Product: '<S685>/Product3'
     *  Trigonometry: '<S685>/Sin'
     *  UnaryMinus: '<S685>/Unary Minus2'
     */
    rtb_Abs_ob4k = rtb_Subtract_if0w_idx_0 * sinf(-rtb_Add_fbcu);

    /* Switch: '<S739>/Switch' incorporates:
     *  UnitDelay: '<S739>/Unit Delay1'
     */
    if (rtb_OR_bsb5) {
        rtb_y_fcnl = rtb_Abs_ob4k;
    } else {
        rtb_y_fcnl = TTG_OdoPosYDelayRe_met;
    }

    /* End of Switch: '<S739>/Switch' */

    /* Switch: '<S727>/Switch' incorporates:
     *  Inport: '<Root>/Inport2'
     *  UnitDelay: '<S727>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_RiLnPosY0UnitDelay_met = ABPR_RiLnClthPosY0_met;
    }

    /* End of Switch: '<S727>/Switch' */

    /* Sum: '<S731>/Subtract' incorporates:
     *  Sum: '<S732>/Subtract'
     *  UnitDelay: '<S727>/Unit Delay'
     */
    rtb_PosY0_met_dafa = TTG_RiLnPosY0UnitDelay_met - rtb_y_fcnl;

    /* Product: '<S731>/Product' incorporates:
     *  Product: '<S730>/Product10'
     *  Switch generated from: '<S684>/Switch'
     *  UnitDelay: '<S726>/Unit Delay'
     */
    rtb_Heading_rad = rtb_uDLookupTable1_eoam * TTG_RiLnHeadingUnitDelay_rad;

    /* Product: '<S731>/Product2' incorporates:
     *  Constant: '<S731>/Constant'
     *  Product: '<S697>/Product2'
     *  Product: '<S714>/Product2'
     *  Product: '<S731>/Product1'
     */
    rtb_Length_out_ltpy =
        (rtb_uDLookupTable1_eoam * rtb_uDLookupTable1_eoam) * 0.5F;

    /* Product: '<S731>/Product6' incorporates:
     *  Product: '<S730>/Product9'
     *  Product: '<S731>/Product2'
     *  Switch generated from: '<S684>/Switch'
     *  UnitDelay: '<S725>/Unit Delay'
     */
    rtb_ABPR_RiLnClthPosY0_met = rtb_Length_out_ltpy * TTG_RiLnCrvUnitDelay_1pm;

    /* Product: '<S731>/Product7' incorporates:
     *  Product: '<S731>/Product'
     *  Product: '<S731>/Product6'
     *  Sum: '<S731>/Add'
     *  Sum: '<S731>/Add1'
     *  Sum: '<S731>/Subtract'
     */
    rtb_x_yaw =
        (rtb_ABPR_RiLnClthPosY0_met + (rtb_Heading_rad + rtb_PosY0_met_dafa)) *
        rtb_Add_fbcu;

    /* Sum: '<S732>/Subtract1' incorporates:
     *  Sum: '<S728>/Subtract1'
     *  Switch generated from: '<S684>/Switch'
     */
    rtb_Switch_ithu = rtb_uDLookupTable1_eoam - rtb_x_yaw;

    /* Sum: '<S732>/Add1' incorporates:
     *  Constant: '<S732>/Constant'
     *  Product: '<S732>/Product'
     *  Product: '<S732>/Product1'
     *  Product: '<S732>/Product2'
     *  Product: '<S732>/Product6'
     *  Sum: '<S732>/Add'
     *  Sum: '<S732>/Subtract1'
     *  UnitDelay: '<S725>/Unit Delay'
     *  UnitDelay: '<S726>/Unit Delay'
     */
    rtb_PosY0_met_dafa =
        (((rtb_Switch_ithu * rtb_Switch_ithu) * 0.5F) *
         TTG_RiLnCrvUnitDelay_1pm) +
        ((rtb_Switch_ithu * TTG_RiLnHeadingUnitDelay_rad) + rtb_PosY0_met_dafa);

    /* Sum: '<S730>/Add4' incorporates:
     *  Product: '<S730>/Product11'
     *  Product: '<S730>/Product12'
     */
    rtb_PosY0_met_dafa =
        (rtb_x_yaw * rtb_x_yaw) + (rtb_PosY0_met_dafa * rtb_PosY0_met_dafa);

    /* Switch generated from: '<S684>/Switch' incorporates:
     *  Inport: '<Root>/Inport1'
     *  Inport: '<Root>/Inport2'
     *  Inport: '<Root>/Inport24'
     *  Inport: '<Root>/Inport25'
     *  Switch: '<S708>/Switch'
     *  Switch: '<S709>/Switch'
     *  Switch: '<S710>/Switch'
     *  UnitDelay: '<S708>/Unit Delay'
     *  UnitDelay: '<S709>/Unit Delay'
     *  UnitDelay: '<S710>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S730>/Switch' incorporates:
         *  RelationalOperator: '<S730>/GreaterThan'
         *  Sqrt: '<S730>/Sqrt'
         *  Sum: '<S730>/Add2'
         *  Sum: '<S730>/Add3'
         *  UnitDelay: '<S727>/Unit Delay'
         *
         * About '<S730>/Sqrt':
         *  Operator: signedSqrt
         */
        if (rtb_y_fcnl > (rtb_ABPR_RiLnClthPosY0_met +
                          (rtb_Heading_rad + TTG_RiLnPosY0UnitDelay_met))) {
            /* Sqrt: '<S730>/Sqrt' incorporates:
             *  UnaryMinus: '<S730>/Unary Minus'
             *
             * About '<S730>/Sqrt':
             *  Operator: signedSqrt
             */
            if (rtb_PosY0_met_dafa < 0.0F) {
                rtb_ABPR_RiLnClthPosY0_met = sqrtf(fabsf(rtb_PosY0_met_dafa));
            } else {
                rtb_ABPR_RiLnClthPosY0_met = -sqrtf(rtb_PosY0_met_dafa);
            }
        } else if (rtb_PosY0_met_dafa < 0.0F) {
            /* Sqrt: '<S730>/Sqrt'
             *
             * About '<S730>/Sqrt':
             *  Operator: signedSqrt
             */
            rtb_ABPR_RiLnClthPosY0_met = -sqrtf(fabsf(rtb_PosY0_met_dafa));
        } else {
            rtb_ABPR_RiLnClthPosY0_met = sqrtf(rtb_PosY0_met_dafa);
        }

        /* End of Switch: '<S730>/Switch' */
    } else {
        rtb_ABPR_RiLnClthPosY0_met = ABPR_RiLnClthPosY0_met;
        TTG_LeLnCrvUnitDelay_1pm = ABPR_LeLnClthCrv_1pm;
        TTG_LeLnHeadingUnitDelay_rad = ABPR_LeLnClthHeading_rad;
        TTG_LeLnPosY0UnitDelay_met = ABPR_LeLnClthPosY0_met;
    }

    /* Sum: '<S714>/Subtract' incorporates:
     *  Sum: '<S715>/Subtract'
     *  UnitDelay: '<S710>/Unit Delay'
     */
    rtb_PosY0_met_dafa = TTG_LeLnPosY0UnitDelay_met - rtb_y_fcnl;

    /* Product: '<S714>/Product' incorporates:
     *  Product: '<S713>/Product10'
     *  Switch generated from: '<S683>/Switch'
     *  UnitDelay: '<S709>/Unit Delay'
     */
    rtb_x_yaw = rtb_uDLookupTable1_eoam * TTG_LeLnHeadingUnitDelay_rad;

    /* Product: '<S714>/Product6' incorporates:
     *  Product: '<S713>/Product9'
     *  Switch generated from: '<S683>/Switch'
     *  UnitDelay: '<S708>/Unit Delay'
     */
    rtb_ABPR_LeLnClthLength_met =
        rtb_Length_out_ltpy * TTG_LeLnCrvUnitDelay_1pm;

    /* Product: '<S714>/Product7' incorporates:
     *  Product: '<S714>/Product'
     *  Product: '<S714>/Product6'
     *  Sum: '<S714>/Add'
     *  Sum: '<S714>/Add1'
     *  Sum: '<S714>/Subtract'
     */
    rtb_x_yaw_hyuw =
        (rtb_ABPR_LeLnClthLength_met + (rtb_x_yaw + rtb_PosY0_met_dafa)) *
        rtb_Add_fbcu;

    /* Sum: '<S715>/Subtract1' incorporates:
     *  Sum: '<S711>/Subtract1'
     *  Switch generated from: '<S683>/Switch'
     */
    rtb_Heading_rad = rtb_uDLookupTable1_eoam - rtb_x_yaw_hyuw;

    /* Sum: '<S715>/Add1' incorporates:
     *  Constant: '<S715>/Constant'
     *  Product: '<S715>/Product'
     *  Product: '<S715>/Product1'
     *  Product: '<S715>/Product2'
     *  Product: '<S715>/Product6'
     *  Sum: '<S715>/Add'
     *  Sum: '<S715>/Subtract1'
     *  UnitDelay: '<S708>/Unit Delay'
     *  UnitDelay: '<S709>/Unit Delay'
     */
    rtb_PosY0_met_dafa =
        (((rtb_Heading_rad * rtb_Heading_rad) * 0.5F) *
         TTG_LeLnCrvUnitDelay_1pm) +
        ((rtb_Heading_rad * TTG_LeLnHeadingUnitDelay_rad) + rtb_PosY0_met_dafa);

    /* Sum: '<S713>/Add4' incorporates:
     *  Product: '<S713>/Product11'
     *  Product: '<S713>/Product12'
     */
    rtb_PosY0_met_dafa = (rtb_x_yaw_hyuw * rtb_x_yaw_hyuw) +
                         (rtb_PosY0_met_dafa * rtb_PosY0_met_dafa);

    /* Switch generated from: '<S683>/Switch' incorporates:
     *  Inport: '<Root>/Inport1'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S713>/Switch' incorporates:
         *  RelationalOperator: '<S713>/GreaterThan'
         *  Sqrt: '<S713>/Sqrt'
         *  Sum: '<S713>/Add2'
         *  Sum: '<S713>/Add3'
         *  UnitDelay: '<S710>/Unit Delay'
         *
         * About '<S713>/Sqrt':
         *  Operator: signedSqrt
         */
        if (rtb_y_fcnl > (rtb_ABPR_LeLnClthLength_met +
                          (rtb_x_yaw + TTG_LeLnPosY0UnitDelay_met))) {
            /* Sqrt: '<S713>/Sqrt' incorporates:
             *  UnaryMinus: '<S713>/Unary Minus'
             *
             * About '<S713>/Sqrt':
             *  Operator: signedSqrt
             */
            if (rtb_PosY0_met_dafa < 0.0F) {
                rtb_ABPR_LeLnClthPosY0_met = sqrtf(fabsf(rtb_PosY0_met_dafa));
            } else {
                rtb_ABPR_LeLnClthPosY0_met = -sqrtf(rtb_PosY0_met_dafa);
            }
        } else if (rtb_PosY0_met_dafa < 0.0F) {
            /* Sqrt: '<S713>/Sqrt'
             *
             * About '<S713>/Sqrt':
             *  Operator: signedSqrt
             */
            rtb_ABPR_LeLnClthPosY0_met = -sqrtf(fabsf(rtb_PosY0_met_dafa));
        } else {
            rtb_ABPR_LeLnClthPosY0_met = sqrtf(rtb_PosY0_met_dafa);
        }

        /* End of Switch: '<S713>/Switch' */
    } else {
        rtb_ABPR_LeLnClthPosY0_met = ABPR_LeLnClthPosY0_met;
    }

    /* Switch: '<S128>/Switch1' incorporates:
     *  Constant: '<S128>/Constant7'
     */
    TJALKA_LnIncoherenceStatus_nu = 0U;

    /* RelationalOperator: '<S837>/Equal' incorporates:
     *  Constant: '<S837>/Constant'
     */
    rtb_Equal_awdj = (((int32_T)TJALKA_LnIncoherenceStatus_nu) == 1);

    /* Logic: '<S832>/NOT' incorporates:
     *  Constant: '<S832>/Constant'
     *  Logic: '<S834>/NOT'
     */
    rtb_Equal_i3l2_idx_0 = !TJATTG_EnableVirtAdjLane_C_bool;

    /* Switch: '<S832>/Switch1' incorporates:
     *  Constant: '<S832>/Constant1'
     *  Inport: '<Root>/Inport40'
     *  Logic: '<S832>/NOT'
     *  Sum: '<S832>/Add'
     */
    if (rtb_Equal_i3l2_idx_0) {
        rtb_PosY0_met = ABPR_LeAdjLnClthPosY0_met;
    } else {
        rtb_PosY0_met =
            TJATTG_VirtAdjLaneWidth_C_met + rtb_ABPR_LeLnClthPosY0_met;
    }

    /* End of Switch: '<S832>/Switch1' */

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch: '<S837>/Switch' incorporates:
             *  Constant: '<S837>/Constant2'
             *  Sum: '<S837>/Add'
             */
            if (rtb_Equal_awdj) {
                rtb_PosY0_met_dafa = rtb_ABPR_RiLnClthPosY0_met + 3.0F;
            } else {
                rtb_PosY0_met_dafa = rtb_ABPR_LeLnClthPosY0_met;
            }

            /* End of Switch: '<S837>/Switch' */
            break;

        case 1:
            rtb_PosY0_met_dafa = rtb_PosY0_met;
            break;

        case 2:
            rtb_PosY0_met_dafa = rtb_ABPR_RiLnClthPosY0_met;
            break;

        default:
            rtb_PosY0_met_dafa = rtb_ABPR_LeLnClthPosY0_met;
            break;
    }

    /* Switch: '<S744>/Switch1' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        /* Switch: '<S744>/Switch6' incorporates:
         *  Constant: '<S744>/Constant1'
         *  Constant: '<S744>/Constant2'
         *  Product: '<S744>/Product'
         *  Sum: '<S744>/Add'
         */
        if (TTG_CMBObjectCorridor_bool) {
            rtb_x_yaw_hyuw = (TJATTG_ObjFolVirtLnWdth_C_met * 0.5F) +
                             TJACMB_CombinedPosY0_met;
        } else {
            rtb_x_yaw_hyuw = rtb_PosY0_met_dafa;
        }

        /* End of Switch: '<S744>/Switch6' */
    } else {
        rtb_x_yaw_hyuw = rtb_PosY0_met_dafa;
    }

    /* End of Switch: '<S744>/Switch1' */

    /* Logic: '<S764>/OR3' incorporates:
     *  Constant: '<S767>/Constant'
     *  Constant: '<S768>/Constant'
     *  Constant: '<S769>/Constant'
     *  Constant: '<S770>/Constant'
     *  Constant: '<S771>/Constant'
     *  Constant: '<S772>/Constant'
     *  Logic: '<S764>/OR'
     *  Logic: '<S764>/OR1'
     *  Logic: '<S764>/OR2'
     *  RelationalOperator: '<S764>/Equal'
     *  RelationalOperator: '<S764>/Equal1'
     *  RelationalOperator: '<S764>/Equal2'
     *  RelationalOperator: '<S764>/Equal3'
     *  RelationalOperator: '<S764>/Equal4'
     *  RelationalOperator: '<S764>/Equal5'
     */
    TTG_LaneUpdate_bool = ((((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
                              ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                             (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC_RQ ==
                              ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
                            ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
                              ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                             (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC_RQ ==
                              ((uint32_T)TJASTM_LatCtrlMode_nu)))) ||
                           ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB ==
                             ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                            (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ ==
                             ((uint32_T)TJASTM_LatCtrlMode_nu))));

    /* Logic: '<S765>/OR2' incorporates:
     *  Constant: '<S765>/Constant'
     *  Constant: '<S773>/Constant'
     *  Constant: '<S774>/Constant'
     *  Constant: '<S775>/Constant'
     *  Constant: '<S776>/Constant'
     *  Logic: '<S765>/AND'
     *  Logic: '<S765>/OR'
     *  Logic: '<S765>/OR1'
     *  RelationalOperator: '<S765>/Equal'
     *  RelationalOperator: '<S765>/Equal1'
     *  RelationalOperator: '<S765>/Equal2'
     *  RelationalOperator: '<S765>/Equal3'
     */
    TTG_ObjectUpdate_bool = ((((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB ==
                                ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                               (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ ==
                                ((uint32_T)TJASTM_LatCtrlMode_nu))) &&
                              (TJATTG_EnableObjDuringCMB_C_bool)) ||
                             ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF ==
                               ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                              (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF_RQ ==
                               ((uint32_T)TJASTM_LatCtrlMode_nu))));

    /* Switch: '<S763>/Switch' incorporates:
     *  Constant: '<S763>/Constant1'
     *  Logic: '<S763>/AND'
     *  Switch: '<S763>/Switch1'
     *  Switch: '<S763>/Switch2'
     */
    if ((TTG_LaneUpdate_bool) && (TTG_ObjectUpdate_bool)) {
        rtb_TTG_Switch_nu = 0U;
    } else if (TTG_LaneUpdate_bool) {
        /* Switch: '<S763>/Switch1' incorporates:
         *  Constant: '<S763>/Constant'
         */
        rtb_TTG_Switch_nu = 1U;
    } else if (TTG_ObjectUpdate_bool) {
        /* Switch: '<S763>/Switch2' incorporates:
         *  Constant: '<S763>/Constant2'
         *  Switch: '<S763>/Switch1'
         */
        rtb_TTG_Switch_nu = 2U;
    } else {
        /* Switch: '<S763>/Switch1' incorporates:
         *  Constant: '<S763>/Constant3'
         *  Switch: '<S763>/Switch2'
         */
        rtb_TTG_Switch_nu = 3U;
    }

    /* End of Switch: '<S763>/Switch' */

    /* Sum: '<S671>/Add' incorporates:
     *  Constant: '<S671>/Constant1'
     *  Constant: '<S671>/Constant2'
     *  Inport: '<Root>/Inport36'
     *  Product: '<S671>/Product'
     */
    rtb_x_yaw = ODPFOH_TgtObjPosY0_met + (TJATTG_ObjFolVirtLnWdth_C_met * 0.5F);

    /* Logic: '<S779>/AND' incorporates:
     *  Constant: '<S781>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S779>/Equal'
     *  RelationalOperator: '<S779>/Equal1'
     *  Switch: '<S552>/Switch3'
     *  UnitDelay: '<S779>/Unit Delay'
     */
    rtb_OR_imjz = ((E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
                    ((uint32_T)TJASTM_SysStateTJA_nu)) &&
                   (E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
                    ((uint32_T)TTG_STMSystemState_Enum)));

    /* Logic: '<S779>/OR1' incorporates:
     *  Constant: '<S789>/Constant'
     *  Constant: '<S790>/Constant'
     *  Constant: '<S791>/Constant'
     *  Constant: '<S792>/Constant'
     *  Logic: '<S779>/AND3'
     *  Logic: '<S779>/AND4'
     *  RelationalOperator: '<S779>/Equal6'
     *  RelationalOperator: '<S779>/Equal7'
     *  RelationalOperator: '<S779>/Equal8'
     *  RelationalOperator: '<S779>/Equal9'
     */
    rtb_TTG_CMB_bool = (((((TTG_CMBObjectCorridor_bool) &&
                           (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) &&
                          (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
                           E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC)) &&
                         rtb_OR_imjz) ||
                        (((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                           E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB) &&
                          (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
                           E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF)) &&
                         rtb_OR_imjz));

    /* Logic: '<S665>/AND2' incorporates:
     *  Logic: '<S677>/AND'
     *  Logic: '<S677>/NOT'
     *  UnitDelay: '<S677>/Unit Delay'
     */
    rtb_Equal_blkb = ((TTG_CMBObjectCorridor_bool) &&
                      ((!TJACMB_ObjectCorridor_bool) &&
                       (TTG_ObjectCorridorEdgeFalling_bool)));

    /* Logic: '<S766>/AND' incorporates:
     *  Constant: '<S778>/Constant'
     *  Logic: '<S777>/AND'
     *  Logic: '<S777>/NOT'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S766>/Equal1'
     *  Switch: '<S552>/Switch3'
     *  UnitDelay: '<S777>/Unit Delay'
     */
    TTG_LD_PredictFinish_bool = (((!TJATTG_PredictionEnable_bool) &&
                                  (TTG_PredictionEnableEdgeFalling_bool)) &&
                                 (E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
                                  ((uint32_T)TJASTM_SysStateTJA_nu)));

    /* Logic: '<S779>/OR2' incorporates:
     *  Constant: '<S783>/Constant'
     *  Constant: '<S784>/Constant'
     *  Constant: '<S785>/Constant'
     *  Constant: '<S793>/Constant'
     *  Logic: '<S779>/AND5'
     *  Logic: '<S779>/AND6'
     *  RelationalOperator: '<S779>/Equal10'
     *  RelationalOperator: '<S779>/Equal11'
     *  RelationalOperator: '<S779>/Equal12'
     *  RelationalOperator: '<S779>/Equal13'
     *  UnitDelay: '<S779>/Unit Delay2'
     */
    rtb_Equal_i3l2_idx_1 =
        ((((((((uint32_T)TJASTM_LatCtrlMode_nu) ==
              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) &&
             (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF)) &&
            rtb_OR_imjz) ||
           (((rtb_OR_imjz && (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC)) &&
             (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
              E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) &&
            (TTG_CMBObjectCorridorUnitDelay_bool))) ||
          rtb_Equal_blkb) ||
         (TTG_LD_PredictFinish_bool));

    /* Logic: '<S779>/AND1' incorporates:
     *  Constant: '<S782>/Constant'
     *  Constant: '<S786>/Constant'
     *  RelationalOperator: '<S779>/Equal2'
     *  RelationalOperator: '<S779>/Equal3'
     */
    rtb_SLC_LeTurnSignalOn_bool = (((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                                     E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF) &&
                                    (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
                                     E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC)) &&
                                   rtb_OR_imjz);

    /* Logic: '<S779>/AND2' incorporates:
     *  Constant: '<S787>/Constant'
     *  Constant: '<S788>/Constant'
     *  RelationalOperator: '<S779>/Equal4'
     *  RelationalOperator: '<S779>/Equal5'
     */
    rtb_AND2_emk5 = (((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                       E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF) &&
                      (((uint32_T)TTG_STMLatCtrlMode_Enum) ==
                       E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) &&
                     rtb_OR_imjz);

    /* Logic: '<S665>/AND1' incorporates:
     *  Logic: '<S678>/AND'
     *  Logic: '<S678>/NOT'
     *  UnitDelay: '<S678>/Unit Delay'
     */
    rtb_OR_imjz =
        ((TTG_CMBObjectCorridor_bool) &&
         (TJACMB_ObjectCorridor_bool && (!TTG_ObjectCorridorEdgeRising_bool)));

    /* Logic: '<S779>/OR' */
    rtb_SLC_LeTurnSignalOn_bool =
        ((rtb_SLC_LeTurnSignalOn_bool || rtb_AND2_emk5) || rtb_OR_imjz);

    /* Logic: '<S798>/OR1' */
    TTG_Reset_bool = ((rtb_Equal_i3l2_idx_1 || rtb_TTG_CMB_bool) ||
                      rtb_SLC_LeTurnSignalOn_bool);

    /* Switch: '<S808>/Switch' incorporates:
     *  Constant: '<S806>/Constant'
     *  Constant: '<S807>/Constant'
     *  Logic: '<S798>/OR'
     *  Logic: '<S803>/AND'
     *  Logic: '<S803>/NOT'
     *  RelationalOperator: '<S802>/Equal1'
     *  RelationalOperator: '<S803>/Equal'
     *  RelationalOperator: '<S803>/Equal1'
     *  UnitDelay: '<S803>/Unit Delay'
     */
    if (((((uint32_T)TTG_STMLatCtrlMode2_Enum) !=
          E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) &&
         (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
          ((uint32_T)TJASTM_LatCtrlMode_nu))) ||
        (((uint32_T)TJASTM_LatCtrlMode_nu) ==
         E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE)) {
        /* Switch: '<S808>/Switch' incorporates:
         *  Constant: '<S808>/Constant'
         */
        TTG_TransitionTimeTurnOffDelayWithRst_sec = 0.0F;
    }

    /* End of Switch: '<S808>/Switch' */

    /* Logic: '<S798>/AND' incorporates:
     *  Constant: '<S798>/Constant'
     *  Constant: '<S808>/Constant2'
     *  Logic: '<S808>/OR'
     *  RelationalOperator: '<S808>/GreaterThan1'
     */
    TTG_Enable_bool = (((TTG_Reset_bool) ||
                        (TTG_TransitionTimeTurnOffDelayWithRst_sec > 0.0F)) &&
                       (TJATTG_TransHandleEnable_C_bool));

    /* Switch: '<S795>/Switch1' incorporates:
     *  Switch: '<S794>/Switch1'
     *  Switch: '<S796>/Switch1'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S795>/Switch' incorporates:
         *  Logic: '<S795>/NOT'
         *  Switch: '<S794>/Switch'
         *  Switch: '<S796>/Switch'
         */
        if (!TTG_Reset_bool) {
            /* Switch: '<S795>/Switch1' incorporates:
             *  UnitDelay: '<S795>/Unit Delay'
             */
            TTG_CMB_Enable_bool = TTG_CMBEnableUnitDelay_bool;

            /* Switch: '<S796>/Switch1' incorporates:
             *  UnitDelay: '<S796>/Unit Delay'
             */
            TTG_LD_Enable_bool = TTG_LDEnableUnitDelay_bool;

            /* Switch: '<S794>/Switch1' incorporates:
             *  UnitDelay: '<S794>/Unit Delay'
             */
            TTG_OD_Enable_bool = TTG_ODEnableUnitDelay_bool;
        } else {
            /* Switch: '<S795>/Switch1' */
            TTG_CMB_Enable_bool = rtb_TTG_CMB_bool;

            /* Switch: '<S796>/Switch1' */
            TTG_LD_Enable_bool = rtb_Equal_i3l2_idx_1;

            /* Switch: '<S794>/Switch1' */
            TTG_OD_Enable_bool = rtb_SLC_LeTurnSignalOn_bool;
        }

        /* End of Switch: '<S795>/Switch' */
    } else {
        /* Switch: '<S795>/Switch1' */
        TTG_CMB_Enable_bool = rtb_TTG_CMB_bool;

        /* Switch: '<S796>/Switch1' */
        TTG_LD_Enable_bool = rtb_Equal_i3l2_idx_1;

        /* Switch: '<S794>/Switch1' */
        TTG_OD_Enable_bool = rtb_SLC_LeTurnSignalOn_bool;
    }

    /* End of Switch: '<S795>/Switch1' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_x_yaw_hyuw = rtb_x_yaw;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_x_yaw_hyuw = rtb_PosY0_met_dafa;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant1' Switch generated from:
             * '<S747>/Switch1' Switch generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_x_yaw_hyuw = rtb_PosY0_met_dafa;
                    break;

                case 2:
                    rtb_x_yaw_hyuw = rtb_x_yaw;
                    break;

                default:
                    rtb_x_yaw_hyuw = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S804>/Switch4' incorporates:
     *  Logic: '<S804>/OR'
     */
    if ((((TTG_LD_PredictFinish_bool) || rtb_Equal_i3l2_idx_1) ||
         rtb_SLC_LeTurnSignalOn_bool) ||
        rtb_TTG_CMB_bool) {
        /* Switch: '<S804>/Switch' incorporates:
         *  Switch: '<S804>/Switch1'
         *  Switch: '<S804>/Switch2'
         *  Switch: '<S804>/Switch3'
         */
        if (TTG_LD_PredictFinish_bool) {
            /* UnitDelay: '<S804>/Unit Delay' incorporates:
             *  Constant: '<S804>/Constant'
             */
            TTG_TransitionTimeUnitDelay_sec = TJATTG_TransDurationPredct_C_sec;
        } else if (rtb_Equal_i3l2_idx_1) {
            /* Switch: '<S804>/Switch1' incorporates:
             *  Constant: '<S804>/Constant2'
             *  UnitDelay: '<S804>/Unit Delay'
             */
            TTG_TransitionTimeUnitDelay_sec = TJATTG_TransDurationLD_C_sec;
        } else if (rtb_SLC_LeTurnSignalOn_bool) {
            /* Switch: '<S804>/Switch2' incorporates:
             *  Constant: '<S804>/Constant3'
             *  Switch: '<S804>/Switch1'
             *  UnitDelay: '<S804>/Unit Delay'
             */
            TTG_TransitionTimeUnitDelay_sec = TJATTG_TransDurationOD_C_sec;
        } else if (rtb_TTG_CMB_bool) {
            /* Switch: '<S804>/Switch3' incorporates:
             *  Constant: '<S804>/Constant4'
             *  Switch: '<S804>/Switch1'
             *  Switch: '<S804>/Switch2'
             *  UnitDelay: '<S804>/Unit Delay'
             */
            TTG_TransitionTimeUnitDelay_sec = TJATTG_TransDurationCMB_C_sec;
        } else {
            /* UnitDelay: '<S804>/Unit Delay' incorporates:
             *  Constant: '<S804>/Constant1'
             *  Switch: '<S804>/Switch1'
             *  Switch: '<S804>/Switch2'
             *  Switch: '<S804>/Switch3'
             */
            TTG_TransitionTimeUnitDelay_sec = 1.0F;
        }

        /* End of Switch: '<S804>/Switch' */
    }

    /* End of Switch: '<S804>/Switch4' */

    /* Switch: '<S801>/Switch' incorporates:
     *  Constant: '<S801>/Constant'
     *  Switch: '<S801>/Switch1'
     *  UnitDelay: '<S801>/Unit Delay'
     */
    if (TTG_Reset_bool) {
        TTG_TransitionFactorAStopwatch_sec = 0.0F;
    } else {
        if (TTG_Enable_bool) {
            /* UnitDelay: '<S801>/Unit Delay' incorporates:
             *  Inport: '<Root>/Inport10'
             *  Sum: '<S801>/Add'
             *  Switch: '<S801>/Switch1'
             */
            TTG_TransitionFactorAStopwatch_sec =
                LCFRCV_TSysCycleTimeSen_sec +
                TTG_TransitionFactorAStopwatch_sec;
        }
    }

    /* End of Switch: '<S801>/Switch' */

    /* Switch: '<S799>/Switch' incorporates:
     *  Abs: '<S799>/Abs1'
     *  Constant: '<S780>/Constant1'
     *  Constant: '<S799>/Constant1'
     *  Product: '<S799>/Divide'
     *  RelationalOperator: '<S799>/Less Than1'
     *  Switch: '<S799>/Switch1'
     *  UnitDelay: '<S801>/Unit Delay'
     *  UnitDelay: '<S804>/Unit Delay'
     */
    if (fabsf(TTG_TransitionTimeUnitDelay_sec) >= 1.0E-7F) {
        rtb_PosY0_met_dafa = TTG_TransitionFactorAStopwatch_sec /
                             TTG_TransitionTimeUnitDelay_sec;
    } else {
        rtb_PosY0_met_dafa = 1.0F;
    }

    /* End of Switch: '<S799>/Switch' */

    /* Product: '<S800>/Divide' incorporates:
     *  Constant: '<S780>/Constant'
     *  Constant: '<S800>/Constant'
     *  Constant: '<S800>/Constant1'
     *  Product: '<S780>/Product'
     *  Sum: '<S800>/Add'
     *  Trigonometry: '<S780>/Cos'
     */
    rtb_out = (cosf(rtb_PosY0_met_dafa * 3.14159274F) + 1.0F) / 2.0F;

    /* Logic: '<S751>/NOT' incorporates:
     *  Logic: '<S816>/NOT'
     *  Logic: '<S867>/NOT'
     */
    rtb_Equal_i3l2_idx_1 = !TTG_Reset_bool;

    /* Switch: '<S751>/Switch2' incorporates:
     *  Switch: '<S755>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay1'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch3' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay1'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorPosY0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .PosY0_met;
        }

        /* End of Switch: '<S751>/Switch3' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S761>/Constant'
         *  Product: '<S761>/Product'
         *  Product: '<S761>/Product1'
         *  Sum: '<S761>/Add'
         *  Sum: '<S761>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay1'
         */
        rtb_x_yaw_hyuw = (TTG_LeCorridorPosY0UnitDelay_met * rtb_out) +
                         ((1.0F - rtb_out) * rtb_x_yaw_hyuw);
    } else {
        TTG_LeCorridorPosY0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .PosY0_met;
    }

    /* End of Switch: '<S751>/Switch2' */

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' incorporates:
     *  Inport: '<Root>/Inport23'
     *  Inport: '<Root>/Inport28'
     */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_PosY0_met_dafa = ABPR_LeLnClthPosX0_met;
            break;

        case 1:
            /* Switch: '<S832>/Switch' incorporates:
             *  Inport: '<Root>/Inport23'
             *  Inport: '<Root>/Inport80'
             *  Logic: '<S832>/NOT'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_PosY0_met_dafa = ABPR_LeAdjLnClthPosX0_met;
            } else {
                rtb_PosY0_met_dafa = ABPR_LeLnClthPosX0_met;
            }

            /* End of Switch: '<S832>/Switch' */
            break;

        case 2:
            rtb_PosY0_met_dafa = ABPR_RiLnClthPosX0_met;
            break;

        default:
            rtb_PosY0_met_dafa = ABPR_LeLnClthPosX0_met;
            break;
    }

    /* MinMax: '<S14>/Max' incorporates:
     *  Inport: '<Root>/Inport58'
     *  Inport: '<Root>/Inport60'
     */
    TJACMB_CombinedPosX0_met =
        fmaxf(ABPR_CntrLnClthPosX0_met, ODPFOH_TgtObjPosX0_met);

    /* Switch: '<S744>/Switch' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_PosX0_met_ckhb = TJACMB_CombinedPosX0_met;
    } else {
        rtb_PosX0_met_ckhb = rtb_PosY0_met_dafa;
    }

    /* End of Switch: '<S744>/Switch' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Inport: '<Root>/Inport60'
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosX0_met_ckhb = ODPFOH_TgtObjPosX0_met;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_PosX0_met_ckhb = rtb_PosY0_met_dafa;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant' Inport:
             * '<Root>/Inport60' Switch generated from: '<S747>/Switch1' Switch
             * generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_PosX0_met_ckhb = rtb_PosY0_met_dafa;
                    break;

                case 2:
                    rtb_PosX0_met_ckhb = ODPFOH_TgtObjPosX0_met;
                    break;

                default:
                    rtb_PosX0_met_ckhb = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S751>/Switch' incorporates:
     *  Switch: '<S754>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch1' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorPosX0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .PosX0_met;
        }

        /* End of Switch: '<S751>/Switch1' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S760>/Constant'
         *  Product: '<S760>/Product'
         *  Product: '<S760>/Product1'
         *  Sum: '<S760>/Add'
         *  Sum: '<S760>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay'
         */
        rtb_PosX0_met_ckhb = (TTG_LeCorridorPosX0UnitDelay_met * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosX0_met_ckhb);
    } else {
        TTG_LeCorridorPosX0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .PosX0_met;
    }

    /* End of Switch: '<S751>/Switch' */

    /* Switch generated from: '<S683>/Switch' incorporates:
     *  Inport: '<Root>/Inport24'
     *  Inport: '<Root>/Inport29'
     *  Product: '<S711>/Product'
     *  Product: '<S728>/Product'
     *  Sum: '<S711>/Add'
     *  Sum: '<S711>/Add1'
     *  Sum: '<S728>/Add'
     *  Sum: '<S728>/Add1'
     *  Switch generated from: '<S684>/Switch'
     *  UnitDelay: '<S708>/Unit Delay'
     *  UnitDelay: '<S709>/Unit Delay'
     *  UnitDelay: '<S725>/Unit Delay'
     *  UnitDelay: '<S726>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool) {
        rtb_ABPR_LeLnClthHeading_rad =
            ((TTG_LeLnCrvUnitDelay_1pm * rtb_Heading_rad) +
             TTG_LeLnHeadingUnitDelay_rad) +
            rtb_Add_fbcu;
        rtb_ABPR_RiLnClthHeading_rad =
            ((TTG_RiLnCrvUnitDelay_1pm * rtb_Switch_ithu) +
             TTG_RiLnHeadingUnitDelay_rad) +
            rtb_Add_fbcu;
    } else {
        rtb_ABPR_LeLnClthHeading_rad = ABPR_LeLnClthHeading_rad;
        rtb_ABPR_RiLnClthHeading_rad = ABPR_RiLnClthHeading_rad;
    }

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Heading_rad = rtb_ABPR_LeLnClthHeading_rad;
            break;

        case 1:
            /* Switch: '<S832>/Switch2' incorporates:
             *  Inport: '<Root>/Inport81'
             *  Logic: '<S832>/NOT'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Heading_rad = ABPR_LeAdjLnClthHeading_rad;
            } else {
                rtb_Heading_rad = rtb_ABPR_LeLnClthHeading_rad;
            }

            /* End of Switch: '<S832>/Switch2' */
            break;

        case 2:
            rtb_Heading_rad = rtb_ABPR_RiLnClthHeading_rad;
            break;

        default:
            rtb_Heading_rad = rtb_ABPR_LeLnClthHeading_rad;
            break;
    }

    /* Switch: '<S18>/Switch1' incorporates:
     *  Constant: '<S18>/Constant'
     *  Constant: '<S18>/Constant3'
     */
    if (TJACMB_ObjectCorridor_bool) {
        rtb_Switch_ithu = 0.0F;
    } else {
        rtb_Switch_ithu = TJACMB_LnWeightHead_C_nu;
    }

    /* End of Switch: '<S18>/Switch1' */

    /* Sum: '<S21>/Add' incorporates:
     *  Inport: '<Root>/Inport37'
     *  Inport: '<Root>/Inport4'
     *  Product: '<S21>/Product'
     *  Sum: '<S21>/Subtract'
     */
    TJACMB_CombinedHeading_rad =
        ODPFOH_TgtObjHeadAng_rad +
        ((ABPR_CntrLnClthHeading_rad - ODPFOH_TgtObjHeadAng_rad) *
         rtb_Switch_ithu);

    /* Switch: '<S744>/Switch2' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_Heading_rad_poq4 = TJACMB_CombinedHeading_rad;
    } else {
        rtb_Heading_rad_poq4 = rtb_Heading_rad;
    }

    /* End of Switch: '<S744>/Switch2' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Inport: '<Root>/Inport37'
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Heading_rad_poq4 = ODPFOH_TgtObjHeadAng_rad;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_Heading_rad_poq4 = rtb_Heading_rad;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant2' Inport:
             * '<Root>/Inport37' Switch generated from: '<S747>/Switch1' Switch
             * generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_Heading_rad_poq4 = rtb_Heading_rad;
                    break;

                case 2:
                    rtb_Heading_rad_poq4 = ODPFOH_TgtObjHeadAng_rad;
                    break;

                default:
                    rtb_Heading_rad_poq4 = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S751>/Switch4' incorporates:
     *  Switch: '<S752>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay2'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch5' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay2'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorHeadingUnitDelay_rad =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .Heading_rad;
        }

        /* End of Switch: '<S751>/Switch5' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S758>/Constant'
         *  Product: '<S758>/Product'
         *  Product: '<S758>/Product1'
         *  Sum: '<S758>/Add'
         *  Sum: '<S758>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay2'
         */
        rtb_Heading_rad_poq4 = (TTG_LeCorridorHeadingUnitDelay_rad * rtb_out) +
                               ((1.0F - rtb_out) * rtb_Heading_rad_poq4);
    } else {
        TTG_LeCorridorHeadingUnitDelay_rad =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .Heading_rad;
    }

    /* End of Switch: '<S751>/Switch4' */

    /* Switch: '<S719>/Switch1' incorporates:
     *  Logic: '<S718>/AND'
     *  Logic: '<S718>/NOT'
     *  Switch: '<S719>/Switch'
     *  UnitDelay: '<S708>/Unit Delay'
     *  UnitDelay: '<S718>/Unit Delay'
     *  UnitDelay: '<S719>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_LeLnPredictEnableEdgeRising_bool)) {
        TTG_LeLnCrvPredictLowPass_1pm = TTG_LeLnCrvUnitDelay_1pm;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S719>/Unit Delay' incorporates:
             *  Constant: '<S706>/Constant1'
             *  Constant: '<S719>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S719>/Divide'
             *  Product: '<S719>/Product1'
             *  Sum: '<S719>/Subtract'
             *  Switch: '<S719>/Switch'
             */
            TTG_LeLnCrvPredictLowPass_1pm =
                (1.0F -
                 (LCFRCV_TSysCycleTimeSen_sec / TJATTG_PredCrvPT1_C_sec)) *
                TTG_LeLnCrvPredictLowPass_1pm;
        }
    }

    /* End of Switch: '<S719>/Switch1' */

    /* Switch generated from: '<S683>/Switch' incorporates:
     *  Inport: '<Root>/Inport25'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S706>/Switch' incorporates:
         *  Constant: '<S706>/Constant'
         *  Logic: '<S706>/NOT'
         *  UnitDelay: '<S708>/Unit Delay'
         *  UnitDelay: '<S719>/Unit Delay'
         */
        if (!TJATTG_PredFreezeCrv_C_bool) {
            rtb_PosY0_met_dafa = TTG_LeLnCrvPredictLowPass_1pm;
        } else {
            rtb_PosY0_met_dafa = TTG_LeLnCrvUnitDelay_1pm;
        }

        /* End of Switch: '<S706>/Switch' */
    } else {
        rtb_PosY0_met_dafa = ABPR_LeLnClthCrv_1pm;
    }

    /* Switch: '<S736>/Switch1' incorporates:
     *  Logic: '<S735>/AND'
     *  Logic: '<S735>/NOT'
     *  Switch: '<S736>/Switch'
     *  UnitDelay: '<S725>/Unit Delay'
     *  UnitDelay: '<S735>/Unit Delay'
     *  UnitDelay: '<S736>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_RiLnPredictEnableEdgeRising_bool)) {
        TTG_RiCrvPredictLowPass_1pm = TTG_RiLnCrvUnitDelay_1pm;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S736>/Unit Delay' incorporates:
             *  Constant: '<S723>/Constant1'
             *  Constant: '<S736>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S736>/Divide'
             *  Product: '<S736>/Product1'
             *  Sum: '<S736>/Subtract'
             *  Switch: '<S736>/Switch'
             */
            TTG_RiCrvPredictLowPass_1pm = (1.0F - (LCFRCV_TSysCycleTimeSen_sec /
                                                   TJATTG_PredCrvPT1_C_sec)) *
                                          TTG_RiCrvPredictLowPass_1pm;
        }
    }

    /* End of Switch: '<S736>/Switch1' */

    /* Switch generated from: '<S684>/Switch' incorporates:
     *  Inport: '<Root>/Inport30'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S723>/Switch' incorporates:
         *  Constant: '<S723>/Constant'
         *  Logic: '<S723>/NOT'
         *  UnitDelay: '<S725>/Unit Delay'
         *  UnitDelay: '<S736>/Unit Delay'
         */
        if (!TJATTG_PredFreezeCrv_C_bool) {
            rtb_ABPR_RiLnClthCrv_1pm = TTG_RiCrvPredictLowPass_1pm;
        } else {
            rtb_ABPR_RiLnClthCrv_1pm = TTG_RiLnCrvUnitDelay_1pm;
        }

        /* End of Switch: '<S723>/Switch' */
    } else {
        rtb_ABPR_RiLnClthCrv_1pm = ABPR_RiLnClthCrv_1pm;
    }

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Heading_rad = rtb_PosY0_met_dafa;
            break;

        case 1:
            /* Switch: '<S832>/Switch3' incorporates:
             *  Inport: '<Root>/Inport82'
             *  Logic: '<S832>/NOT'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Heading_rad = ABPR_LeAdjLnClthCrv_1pm;
            } else {
                rtb_Heading_rad = rtb_PosY0_met_dafa;
            }

            /* End of Switch: '<S832>/Switch3' */
            break;

        case 2:
            rtb_Heading_rad = rtb_ABPR_RiLnClthCrv_1pm;
            break;

        default:
            rtb_Heading_rad = rtb_PosY0_met_dafa;
            break;
    }

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S24>/Constant4'
     *  Inport: '<Root>/Inport9'
     *  Product: '<S24>/Product2'
     */
    rtb_Switch_ithu = ABPR_CntrLnClthCrv_1pm * 0.5F;

    /* Switch: '<S35>/Switch' incorporates:
     *  Constant: '<S29>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S32>/AND'
     *  MinMax: '<S35>/Max'
     *  Sum: '<S35>/Subtract'
     *  Switch: '<S35>/Switch1'
     *  UnaryMinus: '<S35>/Unary Minus'
     *  UnitDelay: '<S32>/Unit Delay'
     *  UnitDelay: '<S35>/Unit Delay'
     */
    if (CMB_EnableFusionEdgeFalling_bool) {
        CMB_EnableFusionTurnOffDelay_sec = 0.5F;
    } else {
        CMB_EnableFusionTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  CMB_EnableFusionTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S35>/Switch' */

    /* Logic: '<S35>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S32>/AND'
     *  RelationalOperator: '<S35>/GreaterThan'
     *  UnaryMinus: '<S35>/Unary Minus1'
     *  UnitDelay: '<S32>/Unit Delay'
     *  UnitDelay: '<S35>/Unit Delay'
     */
    rtb_TTG_CMB_bool =
        ((CMB_EnableFusionEdgeFalling_bool) ||
         (CMB_EnableFusionTurnOffDelay_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S34>/Switch' incorporates:
     *  Constant: '<S34>/Constant'
     *  Logic: '<S32>/AND'
     *  Switch: '<S34>/Switch1'
     *  UnitDelay: '<S32>/Unit Delay'
     *  UnitDelay: '<S34>/Unit Delay'
     */
    if (CMB_EnableFusionEdgeFalling_bool) {
        CMB_EnableFusionStopwatch_sec = 0.0F;
    } else {
        if (rtb_TTG_CMB_bool) {
            /* UnitDelay: '<S34>/Unit Delay' incorporates:
             *  Inport: '<Root>/Inport10'
             *  Sum: '<S34>/Add'
             *  Switch: '<S34>/Switch1'
             */
            CMB_EnableFusionStopwatch_sec =
                LCFRCV_TSysCycleTimeSen_sec + CMB_EnableFusionStopwatch_sec;
        }
    }

    /* End of Switch: '<S34>/Switch' */

    /* Switch: '<S37>/Switch1' incorporates:
     *  Logic: '<S36>/AND'
     *  Logic: '<S36>/NOT'
     *  UnitDelay: '<S25>/Unit Delay'
     *  UnitDelay: '<S36>/Unit Delay'
     *  UnitDelay: '<S37>/Unit Delay'
     */
    if (rtb_TTG_CMB_bool && (!CMB_EnableFusionEdgeRising_bool)) {
        CMB_WeightCrvLowPass_1pm = CMB_PrevCombCrvLFUnitDelay_1pm;

        /* Switch: '<S25>/Switch' incorporates:
         *  Switch: '<S29>/Switch'
         *  UnitDelay: '<S25>/Unit Delay'
         *  UnitDelay: '<S37>/Unit Delay'
         */
        CMB_PrevCombCrvLFUnitDelay_1pm = CMB_WeightCrvLowPass_1pm;
    } else if (rtb_TTG_CMB_bool) {
        /* Product: '<S37>/Divide' incorporates:
         *  Constant: '<S19>/Constant3'
         *  Constant: '<S29>/Constant'
         *  Inport: '<Root>/Inport10'
         *  MinMax: '<S33>/Max'
         *  Product: '<S29>/Product'
         *  Sum: '<S29>/Subtract'
         *  Switch: '<S37>/Switch'
         *  UnitDelay: '<S34>/Unit Delay'
         */
        rtb_x_yaw = LCFRCV_TSysCycleTimeSen_sec /
                    fmaxf(TJACMB_LnQualTurnOnTime_C_sec *
                              (0.5F - CMB_EnableFusionStopwatch_sec),
                          LCFRCV_TSysCycleTimeSen_sec);

        /* UnitDelay: '<S37>/Unit Delay' incorporates:
         *  Constant: '<S37>/Constant1'
         *  Product: '<S37>/Product'
         *  Product: '<S37>/Product1'
         *  Sum: '<S37>/Add'
         *  Sum: '<S37>/Subtract'
         *  Switch: '<S37>/Switch'
         */
        CMB_WeightCrvLowPass_1pm =
            (rtb_Switch_ithu * rtb_x_yaw) +
            ((1.0F - rtb_x_yaw) * CMB_WeightCrvLowPass_1pm);

        /* Switch: '<S25>/Switch' incorporates:
         *  Switch: '<S29>/Switch'
         *  UnitDelay: '<S25>/Unit Delay'
         *  UnitDelay: '<S37>/Unit Delay'
         */
        CMB_PrevCombCrvLFUnitDelay_1pm = CMB_WeightCrvLowPass_1pm;
    } else {
        /* Switch: '<S29>/Switch' incorporates:
         *  Switch: '<S25>/Switch'
         *  UnitDelay: '<S25>/Unit Delay'
         */
        CMB_PrevCombCrvLFUnitDelay_1pm = rtb_Switch_ithu;
    }

    /* End of Switch: '<S37>/Switch1' */

    /* Switch: '<S18>/Switch2' incorporates:
     *  Constant: '<S18>/Constant'
     *  Constant: '<S18>/Constant4'
     */
    if (TJACMB_ObjectCorridor_bool) {
        rtb_Switch_ithu = 0.0F;
    } else {
        rtb_Switch_ithu = TJACMB_LnWeightCrv_C_nu;
    }

    /* End of Switch: '<S18>/Switch2' */

    /* Switch: '<S19>/Switch' incorporates:
     *  Inport: '<Root>/Inport38'
     *  Inport: '<Root>/Inport9'
     *  Product: '<S26>/Product'
     *  Sum: '<S26>/Add'
     *  Sum: '<S26>/Subtract'
     */
    TJACMB_CombinedCrv_1pm =
        ODPFOH_TgtObjCrv_1pm +
        ((ABPR_CntrLnClthCrv_1pm - ODPFOH_TgtObjCrv_1pm) * rtb_Switch_ithu);

    /* Switch: '<S744>/Switch3' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_Crv_1pm_b0ly = TJACMB_CombinedCrv_1pm;
    } else {
        rtb_Crv_1pm_b0ly = rtb_Heading_rad;
    }

    /* End of Switch: '<S744>/Switch3' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Inport: '<Root>/Inport38'
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Crv_1pm_b0ly = ODPFOH_TgtObjCrv_1pm;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_Crv_1pm_b0ly = rtb_Heading_rad;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant3' Inport:
             * '<Root>/Inport38' Switch generated from: '<S747>/Switch1' Switch
             * generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_Crv_1pm_b0ly = rtb_Heading_rad;
                    break;

                case 2:
                    rtb_Crv_1pm_b0ly = ODPFOH_TgtObjCrv_1pm;
                    break;

                default:
                    rtb_Crv_1pm_b0ly = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S751>/Switch6' incorporates:
     *  Switch: '<S750>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay3'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch7' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay3'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorCrvUnitDelay_1pm =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .Crv_1pm;
        }

        /* End of Switch: '<S751>/Switch7' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S757>/Constant'
         *  Product: '<S757>/Product'
         *  Product: '<S757>/Product1'
         *  Sum: '<S757>/Add'
         *  Sum: '<S757>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay3'
         */
        rtb_Crv_1pm_b0ly = (TTG_LeCorridorCrvUnitDelay_1pm * rtb_out) +
                           ((1.0F - rtb_out) * rtb_Crv_1pm_b0ly);
    } else {
        TTG_LeCorridorCrvUnitDelay_1pm =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .Crv_1pm;
    }

    /* End of Switch: '<S751>/Switch6' */

    /* Switch: '<S707>/Switch' incorporates:
     *  Inport: '<Root>/Inport26'
     *  UnitDelay: '<S707>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_LeLnCrvChngUnitDelay_1pm2 = ABPR_LeLnClthCrvChng_1pm2;
    }

    /* End of Switch: '<S707>/Switch' */

    /* Switch: '<S717>/Switch1' incorporates:
     *  Logic: '<S716>/AND'
     *  Logic: '<S716>/NOT'
     *  Switch: '<S717>/Switch'
     *  UnitDelay: '<S707>/Unit Delay'
     *  UnitDelay: '<S716>/Unit Delay'
     *  UnitDelay: '<S717>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_LeLnPredictEnable2EdgeRising_bool)) {
        TTG_LeLnCrvChngPredictLowPass_1pm2 = TTG_LeLnCrvChngUnitDelay_1pm2;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S717>/Unit Delay' incorporates:
             *  Constant: '<S705>/Constant1'
             *  Constant: '<S717>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S717>/Divide'
             *  Product: '<S717>/Product1'
             *  Sum: '<S717>/Subtract'
             *  Switch: '<S717>/Switch'
             */
            TTG_LeLnCrvChngPredictLowPass_1pm2 =
                (1.0F -
                 (LCFRCV_TSysCycleTimeSen_sec / TJATTG_PredCrvChngPT1_C_sec)) *
                TTG_LeLnCrvChngPredictLowPass_1pm2;
        }
    }

    /* End of Switch: '<S717>/Switch1' */

    /* Switch generated from: '<S683>/Switch' incorporates:
     *  Inport: '<Root>/Inport26'
     *  Inport: '<Root>/Inport31'
     *  Switch: '<S724>/Switch'
     *  UnitDelay: '<S724>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S705>/Switch' incorporates:
         *  Constant: '<S705>/Constant'
         *  Logic: '<S705>/NOT'
         *  UnitDelay: '<S707>/Unit Delay'
         *  UnitDelay: '<S717>/Unit Delay'
         */
        if (!TJATTG_PredFreezeCrvChng_C_bool) {
            rtb_x_yaw = TTG_LeLnCrvChngPredictLowPass_1pm2;
        } else {
            rtb_x_yaw = TTG_LeLnCrvChngUnitDelay_1pm2;
        }

        /* End of Switch: '<S705>/Switch' */
    } else {
        rtb_x_yaw = ABPR_LeLnClthCrvChng_1pm2;
        TTG_RiLnCrvChngUnitDelay_1pm2 = ABPR_RiLnClthCrvChng_1pm2;
    }

    /* Switch: '<S734>/Switch1' incorporates:
     *  Logic: '<S733>/AND'
     *  Logic: '<S733>/NOT'
     *  Switch: '<S734>/Switch'
     *  UnitDelay: '<S724>/Unit Delay'
     *  UnitDelay: '<S733>/Unit Delay'
     *  UnitDelay: '<S734>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_RiLnPredictEnable2EdgeRising_bool)) {
        TTG_RiLnCrvChngPredictLowPass_1pm2 = TTG_RiLnCrvChngUnitDelay_1pm2;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S734>/Unit Delay' incorporates:
             *  Constant: '<S722>/Constant1'
             *  Constant: '<S734>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S734>/Divide'
             *  Product: '<S734>/Product1'
             *  Sum: '<S734>/Subtract'
             *  Switch: '<S734>/Switch'
             */
            TTG_RiLnCrvChngPredictLowPass_1pm2 =
                (1.0F -
                 (LCFRCV_TSysCycleTimeSen_sec / TJATTG_PredCrvChngPT1_C_sec)) *
                TTG_RiLnCrvChngPredictLowPass_1pm2;
        }
    }

    /* End of Switch: '<S734>/Switch1' */

    /* Switch generated from: '<S684>/Switch' incorporates:
     *  Inport: '<Root>/Inport31'
     */
    if (TJATTG_PredictionEnable_bool) {
        /* Switch: '<S722>/Switch' incorporates:
         *  Constant: '<S722>/Constant'
         *  Logic: '<S722>/NOT'
         *  UnitDelay: '<S724>/Unit Delay'
         *  UnitDelay: '<S734>/Unit Delay'
         */
        if (!TJATTG_PredFreezeCrvChng_C_bool) {
            rtb_ABPR_RiLnClthCrvChng_1pm2 = TTG_RiLnCrvChngPredictLowPass_1pm2;
        } else {
            rtb_ABPR_RiLnClthCrvChng_1pm2 = TTG_RiLnCrvChngUnitDelay_1pm2;
        }

        /* End of Switch: '<S722>/Switch' */
    } else {
        rtb_ABPR_RiLnClthCrvChng_1pm2 = ABPR_RiLnClthCrvChng_1pm2;
    }

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Heading_rad = rtb_x_yaw;
            break;

        case 1:
            /* Switch: '<S832>/Switch4' incorporates:
             *  Inport: '<Root>/Inport83'
             *  Logic: '<S832>/NOT'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Heading_rad = ABPR_LeAdjLnClthCrvChng_1pm2;
            } else {
                rtb_Heading_rad = rtb_x_yaw;
            }

            /* End of Switch: '<S832>/Switch4' */
            break;

        case 2:
            rtb_Heading_rad = rtb_ABPR_RiLnClthCrvChng_1pm2;
            break;

        default:
            rtb_Heading_rad = rtb_x_yaw;
            break;
    }

    /* Switch: '<S18>/Switch3' incorporates:
     *  Constant: '<S18>/Constant'
     *  Constant: '<S18>/Constant5'
     */
    if (TJACMB_ObjectCorridor_bool) {
        rtb_Switch_ithu = 0.0F;
    } else {
        rtb_Switch_ithu = TJACMB_LnWeightCrvChng_C_nu;
    }

    /* End of Switch: '<S18>/Switch3' */

    /* Sum: '<S20>/Add' incorporates:
     *  Inport: '<Root>/Inport59'
     *  Inport: '<Root>/Inport61'
     *  Product: '<S20>/Product'
     *  Sum: '<S20>/Subtract'
     */
    TJACMB_CombinedCrvChng_1pm2 =
        ODPFOH_TgtObjCrvChng_1pm2 +
        ((ABPR_CntrLnClthCrvChng_1pm2 - ODPFOH_TgtObjCrvChng_1pm2) *
         rtb_Switch_ithu);

    /* Switch: '<S744>/Switch4' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_CrvChng_1pm2_m00x = TJACMB_CombinedCrvChng_1pm2;
    } else {
        rtb_CrvChng_1pm2_m00x = rtb_Heading_rad;
    }

    /* End of Switch: '<S744>/Switch4' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Inport: '<Root>/Inport61'
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_CrvChng_1pm2_m00x = ODPFOH_TgtObjCrvChng_1pm2;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_CrvChng_1pm2_m00x = rtb_Heading_rad;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant4' Inport:
             * '<Root>/Inport61' Switch generated from: '<S747>/Switch1' Switch
             * generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_CrvChng_1pm2_m00x = rtb_Heading_rad;
                    break;

                case 2:
                    rtb_CrvChng_1pm2_m00x = ODPFOH_TgtObjCrvChng_1pm2;
                    break;

                default:
                    rtb_CrvChng_1pm2_m00x = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S751>/Switch8' incorporates:
     *  Switch: '<S749>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay4'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch9' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay4'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorCrvChngUnitDelay_1pm2 =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .CrvChng_1pm2;
        }

        /* End of Switch: '<S751>/Switch9' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S756>/Constant'
         *  Product: '<S756>/Product'
         *  Product: '<S756>/Product1'
         *  Sum: '<S756>/Add'
         *  Sum: '<S756>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay4'
         */
        rtb_CrvChng_1pm2_m00x =
            (TTG_LeCorridorCrvChngUnitDelay_1pm2 * rtb_out) +
            ((1.0F - rtb_out) * rtb_CrvChng_1pm2_m00x);
    } else {
        TTG_LeCorridorCrvChngUnitDelay_1pm2 =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .CrvChng_1pm2;
    }

    /* End of Switch: '<S751>/Switch8' */

    /* Switch: '<S712>/Switch' incorporates:
     *  Inport: '<Root>/Inport27'
     *  UnitDelay: '<S712>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_LeLnLengthUnitDelay_met = ABPR_LeLnClthLength_met;
    }

    /* End of Switch: '<S712>/Switch' */

    /* Sum: '<S712>/Subtract' incorporates:
     *  UnitDelay: '<S712>/Unit Delay'
     */
    rtb_Switch_ithu = TTG_LeLnLengthUnitDelay_met - rtb_uDLookupTable1_eoam;

    /* Switch generated from: '<S683>/Switch' incorporates:
     *  Constant: '<S704>/Constant'
     *  Inport: '<Root>/Inport20'
     *  Inport: '<Root>/Inport27'
     *  MinMax: '<S704>/Max'
     *  Switch: '<S729>/Switch'
     *  UnitDelay: '<S729>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool) {
        rtb_ABPR_LeLnClthLength_met =
            fmaxf(rtb_Switch_ithu, TJATTG_LnPredMinTrajLength_C_met);
    } else {
        rtb_ABPR_LeLnClthLength_met = ABPR_LeLnClthLength_met;
        TTG_RiLnLengthUnitDelay_met = ABPR_RiLnClthLength_met;
    }

    /* Sum: '<S729>/Subtract' incorporates:
     *  UnitDelay: '<S729>/Unit Delay'
     */
    rtb_Heading_rad = TTG_RiLnLengthUnitDelay_met - rtb_uDLookupTable1_eoam;

    /* Switch generated from: '<S684>/Switch' incorporates:
     *  Constant: '<S721>/Constant'
     *  Inport: '<Root>/Inport20'
     *  MinMax: '<S721>/Max'
     */
    if (TJATTG_PredictionEnable_bool) {
        rtb_ABPR_RiLnClthLength_met =
            fmaxf(rtb_Heading_rad, TJATTG_LnPredMinTrajLength_C_met);
    } else {
        rtb_ABPR_RiLnClthLength_met = ABPR_RiLnClthLength_met;
    }

    /* MultiPortSwitch generated from: '<S833>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Length_met = rtb_ABPR_LeLnClthLength_met;
            break;

        case 1:
            /* Switch: '<S832>/Switch5' incorporates:
             *  Inport: '<Root>/Inport84'
             *  Logic: '<S832>/NOT'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Length_met = ABPR_LeAdjLnClthLength_met;
            } else {
                rtb_Length_met = rtb_ABPR_LeLnClthLength_met;
            }

            /* End of Switch: '<S832>/Switch5' */
            break;

        case 2:
            rtb_Length_met = rtb_ABPR_RiLnClthLength_met;
            break;

        default:
            rtb_Length_met = rtb_ABPR_LeLnClthLength_met;
            break;
    }

    /* MinMax: '<S14>/Min' incorporates:
     *  Inport: '<Root>/Inport21'
     *  Inport: '<Root>/Inport39'
     */
    TJACMB_CombinedLength_met =
        fminf(ODPFOH_TgtObjLength_met, ABPR_CntrLnClthLength_met);

    /* Switch: '<S744>/Switch5' incorporates:
     *  Constant: '<S744>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_Length_met_ffe5 = TJACMB_CombinedLength_met;
    } else {
        rtb_Length_met_ffe5 = rtb_Length_met;
    }

    /* End of Switch: '<S744>/Switch5' */

    /* Switch generated from: '<S747>/Switch' incorporates:
     *  Inport: '<Root>/Inport39'
     *  Switch generated from: '<S747>/Switch1'
     *  Switch generated from: '<S747>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Length_met_ffe5 = ODPFOH_TgtObjLength_met;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S747>/Switch1' */
        rtb_Length_met_ffe5 = rtb_Length_met;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S746>/Multiport Switch'
             * incorporates: Constant: '<S762>/Constant5' Inport:
             * '<Root>/Inport39' Switch generated from: '<S747>/Switch1' Switch
             * generated from: '<S747>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_Length_met_ffe5 = rtb_Length_met;
                    break;

                case 2:
                    rtb_Length_met_ffe5 = ODPFOH_TgtObjLength_met;
                    break;

                default:
                    rtb_Length_met_ffe5 = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S751>/Switch10' incorporates:
     *  Switch: '<S753>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S751>/Unit Delay5'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S751>/Switch11' incorporates:
         *  Logic: '<S751>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S751>/Unit Delay5'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_LeCorridorLengthUnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .Length_met;
        }

        /* End of Switch: '<S751>/Switch11' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S759>/Constant'
         *  Product: '<S759>/Product'
         *  Product: '<S759>/Product1'
         *  Sum: '<S759>/Add'
         *  Sum: '<S759>/Subtract'
         *  UnitDelay: '<S751>/Unit Delay5'
         */
        rtb_Length_met_ffe5 = (TTG_LeCorridorLengthUnitDelay_met * rtb_out) +
                              ((1.0F - rtb_out) * rtb_Length_met_ffe5);
    } else {
        TTG_LeCorridorLengthUnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                .Length_met;
    }

    /* End of Switch: '<S751>/Switch10' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' incorporates:
     *  Inport: '<Root>/Inport23'
     *  Inport: '<Root>/Inport28'
     */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_PosX0_met_exq3 = ABPR_RiLnClthPosX0_met;
            break;

        case 1:
            rtb_PosX0_met_exq3 = ABPR_LeLnClthPosX0_met;
            break;

        case 2:
            /* Switch: '<S834>/Switch' incorporates:
             *  Inport: '<Root>/Inport28'
             *  Inport: '<Root>/Inport89'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_PosX0_met_exq3 = ABPR_RiAdjLnClthPosX0_met;
            } else {
                rtb_PosX0_met_exq3 = ABPR_RiLnClthPosX0_met;
            }

            /* End of Switch: '<S834>/Switch' */
            break;

        default:
            rtb_PosX0_met_exq3 = ABPR_RiLnClthPosX0_met;
            break;
    }

    /* Switch: '<S809>/Switch' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_Length_met = TJACMB_CombinedPosX0_met;
    } else {
        rtb_Length_met = rtb_PosX0_met_exq3;
    }

    /* End of Switch: '<S809>/Switch' */

    /* Switch: '<S828>/Switch' incorporates:
     *  Constant: '<S828>/Constant1'
     *  Logic: '<S828>/AND'
     *  Switch: '<S828>/Switch1'
     *  Switch: '<S828>/Switch2'
     */
    if ((TTG_LaneUpdate_bool) && (TTG_ObjectUpdate_bool)) {
        rtb_TTG_Switch_nu = 0U;
    } else if (TTG_LaneUpdate_bool) {
        /* Switch: '<S828>/Switch1' incorporates:
         *  Constant: '<S828>/Constant'
         */
        rtb_TTG_Switch_nu = 1U;
    } else if (TTG_ObjectUpdate_bool) {
        /* Switch: '<S828>/Switch2' incorporates:
         *  Constant: '<S828>/Constant2'
         *  Switch: '<S828>/Switch1'
         */
        rtb_TTG_Switch_nu = 2U;
    } else {
        /* Switch: '<S828>/Switch1' incorporates:
         *  Constant: '<S828>/Constant3'
         *  Switch: '<S828>/Switch2'
         */
        rtb_TTG_Switch_nu = 3U;
    }

    /* End of Switch: '<S828>/Switch' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Inport: '<Root>/Inport60'
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Length_met = ODPFOH_TgtObjPosX0_met;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_Length_met = rtb_PosX0_met_exq3;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant' Inport:
             * '<Root>/Inport60' Switch generated from: '<S812>/Switch1' Switch
             * generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_Length_met = rtb_PosX0_met_exq3;
                    break;

                case 2:
                    rtb_Length_met = ODPFOH_TgtObjPosX0_met;
                    break;

                default:
                    rtb_Length_met = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch' incorporates:
     *  Switch: '<S819>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch1' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorPosX0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.PosX0_met;
        }

        /* End of Switch: '<S816>/Switch1' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S825>/Constant'
         *  Product: '<S825>/Product'
         *  Product: '<S825>/Product1'
         *  Sum: '<S825>/Add'
         *  Sum: '<S825>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay'
         */
        rtb_Length_met = (TTG_RiCorridorPosX0UnitDelay_met * rtb_out) +
                         ((1.0F - rtb_out) * rtb_Length_met);
    } else {
        TTG_RiCorridorPosX0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .PosX0_met;
    }

    /* End of Switch: '<S816>/Switch' */

    /* Sum: '<S672>/Add' incorporates:
     *  Constant: '<S672>/Constant1'
     *  Constant: '<S672>/Constant2'
     *  Inport: '<Root>/Inport36'
     *  Product: '<S672>/Product'
     */
    rtb_PosY0_met_lcpf =
        ODPFOH_TgtObjPosY0_met - (TJATTG_ObjFolVirtLnWdth_C_met * 0.5F);

    /* RelationalOperator: '<S837>/Equal1' incorporates:
     *  Constant: '<S837>/Constant1'
     */
    rtb_SLC_LeTurnSignalOn_bool =
        (((int32_T)TJALKA_LnIncoherenceStatus_nu) == 2);

    /* Switch: '<S834>/Switch1' incorporates:
     *  Constant: '<S834>/Constant1'
     *  Inport: '<Root>/Inport41'
     *  Sum: '<S834>/Subtract'
     */
    if (rtb_Equal_i3l2_idx_0) {
        rtb_PosY0_met_bzv0 = ABPR_RiAdjLnClthPosY0_met;
    } else {
        rtb_PosY0_met_bzv0 =
            rtb_ABPR_RiLnClthPosY0_met - TJATTG_VirtAdjLaneWidth_C_met;
    }

    /* End of Switch: '<S834>/Switch1' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch: '<S837>/Switch1' incorporates:
             *  Constant: '<S837>/Constant2'
             *  Sum: '<S837>/Add1'
             */
            if (rtb_SLC_LeTurnSignalOn_bool) {
                rtb_PosY0_met_cmjs = rtb_ABPR_LeLnClthPosY0_met - 3.0F;
            } else {
                rtb_PosY0_met_cmjs = rtb_ABPR_RiLnClthPosY0_met;
            }

            /* End of Switch: '<S837>/Switch1' */
            break;

        case 1:
            rtb_PosY0_met_cmjs = rtb_ABPR_LeLnClthPosY0_met;
            break;

        case 2:
            rtb_PosY0_met_cmjs = rtb_PosY0_met_bzv0;
            break;

        default:
            rtb_PosY0_met_cmjs = rtb_ABPR_RiLnClthPosY0_met;
            break;
    }

    /* Switch: '<S809>/Switch1' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        /* Switch: '<S809>/Switch6' incorporates:
         *  Constant: '<S809>/Constant1'
         *  Constant: '<S809>/Constant2'
         *  Product: '<S809>/Product'
         *  Sum: '<S809>/Add'
         */
        if (TTG_CMBObjectCorridor_bool) {
            rtb_PosX0_met_exq3 = TJACMB_CombinedPosY0_met -
                                 (TJATTG_ObjFolVirtLnWdth_C_met * 0.5F);
        } else {
            rtb_PosX0_met_exq3 = rtb_PosY0_met_cmjs;
        }

        /* End of Switch: '<S809>/Switch6' */
    } else {
        rtb_PosX0_met_exq3 = rtb_PosY0_met_cmjs;
    }

    /* End of Switch: '<S809>/Switch1' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosX0_met_exq3 = rtb_PosY0_met_lcpf;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_PosX0_met_exq3 = rtb_PosY0_met_cmjs;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant1' Switch generated from:
             * '<S812>/Switch1' Switch generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_PosX0_met_exq3 = rtb_PosY0_met_cmjs;
                    break;

                case 2:
                    rtb_PosX0_met_exq3 = rtb_PosY0_met_lcpf;
                    break;

                default:
                    rtb_PosX0_met_exq3 = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch2' incorporates:
     *  Switch: '<S820>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay1'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch3' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay1'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorPosY0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.PosY0_met;
        }

        /* End of Switch: '<S816>/Switch3' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S826>/Constant'
         *  Product: '<S826>/Product'
         *  Product: '<S826>/Product1'
         *  Sum: '<S826>/Add'
         *  Sum: '<S826>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay1'
         */
        rtb_PosX0_met_exq3 = (TTG_RiCorridorPosY0UnitDelay_met * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosX0_met_exq3);
    } else {
        TTG_RiCorridorPosY0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .PosY0_met;
    }

    /* End of Switch: '<S816>/Switch2' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_PosY0_met_cmjs = rtb_ABPR_RiLnClthHeading_rad;
            break;

        case 1:
            rtb_PosY0_met_cmjs = rtb_ABPR_LeLnClthHeading_rad;
            break;

        case 2:
            /* Switch: '<S834>/Switch2' incorporates:
             *  Inport: '<Root>/Inport85'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_PosY0_met_cmjs = ABPR_RiAdjLnClthHeading_rad;
            } else {
                rtb_PosY0_met_cmjs = rtb_ABPR_RiLnClthHeading_rad;
            }

            /* End of Switch: '<S834>/Switch2' */
            break;

        default:
            rtb_PosY0_met_cmjs = rtb_ABPR_RiLnClthHeading_rad;
            break;
    }

    /* Switch: '<S809>/Switch2' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_PosY0_met_lcpf = TJACMB_CombinedHeading_rad;
    } else {
        rtb_PosY0_met_lcpf = rtb_PosY0_met_cmjs;
    }

    /* End of Switch: '<S809>/Switch2' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Inport: '<Root>/Inport37'
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosY0_met_lcpf = ODPFOH_TgtObjHeadAng_rad;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_PosY0_met_lcpf = rtb_PosY0_met_cmjs;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant2' Inport:
             * '<Root>/Inport37' Switch generated from: '<S812>/Switch1' Switch
             * generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_PosY0_met_lcpf = rtb_PosY0_met_cmjs;
                    break;

                case 2:
                    rtb_PosY0_met_lcpf = ODPFOH_TgtObjHeadAng_rad;
                    break;

                default:
                    rtb_PosY0_met_lcpf = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch4' incorporates:
     *  Switch: '<S817>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay2'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch5' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay2'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorHeadingUnitDelay_rad =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.Heading_rad;
        }

        /* End of Switch: '<S816>/Switch5' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S823>/Constant'
         *  Product: '<S823>/Product'
         *  Product: '<S823>/Product1'
         *  Sum: '<S823>/Add'
         *  Sum: '<S823>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay2'
         */
        rtb_PosY0_met_lcpf = (TTG_RiCorridorHeadingUnitDelay_rad * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosY0_met_lcpf);
    } else {
        TTG_RiCorridorHeadingUnitDelay_rad =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .Heading_rad;
    }

    /* End of Switch: '<S816>/Switch4' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Crv_1pm_mhaw = rtb_ABPR_RiLnClthCrv_1pm;
            break;

        case 1:
            rtb_Crv_1pm_mhaw = rtb_PosY0_met_dafa;
            break;

        case 2:
            /* Switch: '<S834>/Switch3' incorporates:
             *  Inport: '<Root>/Inport86'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Crv_1pm_mhaw = ABPR_RiAdjLnClthCrv_1pm;
            } else {
                rtb_Crv_1pm_mhaw = rtb_ABPR_RiLnClthCrv_1pm;
            }

            /* End of Switch: '<S834>/Switch3' */
            break;

        default:
            rtb_Crv_1pm_mhaw = rtb_ABPR_RiLnClthCrv_1pm;
            break;
    }

    /* Switch: '<S809>/Switch3' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_PosY0_met_cmjs = TJACMB_CombinedCrv_1pm;
    } else {
        rtb_PosY0_met_cmjs = rtb_Crv_1pm_mhaw;
    }

    /* End of Switch: '<S809>/Switch3' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Inport: '<Root>/Inport38'
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosY0_met_cmjs = ODPFOH_TgtObjCrv_1pm;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_PosY0_met_cmjs = rtb_Crv_1pm_mhaw;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant3' Inport:
             * '<Root>/Inport38' Switch generated from: '<S812>/Switch1' Switch
             * generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_PosY0_met_cmjs = rtb_Crv_1pm_mhaw;
                    break;

                case 2:
                    rtb_PosY0_met_cmjs = ODPFOH_TgtObjCrv_1pm;
                    break;

                default:
                    rtb_PosY0_met_cmjs = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch6' incorporates:
     *  Switch: '<S815>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay3'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch7' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay3'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorCrvUnitDelay_1pm =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.Crv_1pm;
        }

        /* End of Switch: '<S816>/Switch7' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S822>/Constant'
         *  Product: '<S822>/Product'
         *  Product: '<S822>/Product1'
         *  Sum: '<S822>/Add'
         *  Sum: '<S822>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay3'
         */
        rtb_PosY0_met_cmjs = (TTG_RiCorridorCrvUnitDelay_1pm * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosY0_met_cmjs);
    } else {
        TTG_RiCorridorCrvUnitDelay_1pm =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .Crv_1pm;
    }

    /* End of Switch: '<S816>/Switch6' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_CrvChng_1pm2_f0cp = rtb_ABPR_RiLnClthCrvChng_1pm2;
            break;

        case 1:
            rtb_CrvChng_1pm2_f0cp = rtb_x_yaw;
            break;

        case 2:
            /* Switch: '<S834>/Switch4' incorporates:
             *  Inport: '<Root>/Inport87'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_CrvChng_1pm2_f0cp = ABPR_RiAdjLnClthCrvChng_1pm2;
            } else {
                rtb_CrvChng_1pm2_f0cp = rtb_ABPR_RiLnClthCrvChng_1pm2;
            }

            /* End of Switch: '<S834>/Switch4' */
            break;

        default:
            rtb_CrvChng_1pm2_f0cp = rtb_ABPR_RiLnClthCrvChng_1pm2;
            break;
    }

    /* Switch: '<S809>/Switch4' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_Crv_1pm_mhaw = TJACMB_CombinedCrvChng_1pm2;
    } else {
        rtb_Crv_1pm_mhaw = rtb_CrvChng_1pm2_f0cp;
    }

    /* End of Switch: '<S809>/Switch4' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Inport: '<Root>/Inport61'
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Crv_1pm_mhaw = ODPFOH_TgtObjCrvChng_1pm2;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_Crv_1pm_mhaw = rtb_CrvChng_1pm2_f0cp;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant4' Inport:
             * '<Root>/Inport61' Switch generated from: '<S812>/Switch1' Switch
             * generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_Crv_1pm_mhaw = rtb_CrvChng_1pm2_f0cp;
                    break;

                case 2:
                    rtb_Crv_1pm_mhaw = ODPFOH_TgtObjCrvChng_1pm2;
                    break;

                default:
                    rtb_Crv_1pm_mhaw = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch8' incorporates:
     *  Switch: '<S814>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay4'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch9' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay4'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorCrvChngUnitDelay_1pm2 =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.CrvChng_1pm2;
        }

        /* End of Switch: '<S816>/Switch9' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S821>/Constant'
         *  Product: '<S821>/Product'
         *  Product: '<S821>/Product1'
         *  Sum: '<S821>/Add'
         *  Sum: '<S821>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay4'
         */
        rtb_Crv_1pm_mhaw = (TTG_RiCorridorCrvChngUnitDelay_1pm2 * rtb_out) +
                           ((1.0F - rtb_out) * rtb_Crv_1pm_mhaw);
    } else {
        TTG_RiCorridorCrvChngUnitDelay_1pm2 =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .CrvChng_1pm2;
    }

    /* End of Switch: '<S816>/Switch8' */

    /* MultiPortSwitch generated from: '<S835>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Length_met_kx1h = rtb_ABPR_RiLnClthLength_met;
            break;

        case 1:
            rtb_Length_met_kx1h = rtb_ABPR_LeLnClthLength_met;
            break;

        case 2:
            /* Switch: '<S834>/Switch5' incorporates:
             *  Inport: '<Root>/Inport88'
             */
            if (rtb_Equal_i3l2_idx_0) {
                rtb_Length_met_kx1h = ABPR_RiAdjLnClthLength_met;
            } else {
                rtb_Length_met_kx1h = rtb_ABPR_RiLnClthLength_met;
            }

            /* End of Switch: '<S834>/Switch5' */
            break;

        default:
            rtb_Length_met_kx1h = rtb_ABPR_RiLnClthLength_met;
            break;
    }

    /* Switch: '<S809>/Switch5' incorporates:
     *  Constant: '<S809>/Constant'
     */
    if (TJACMB_CombinedDataEnable_C_bool) {
        rtb_CrvChng_1pm2_f0cp = TJACMB_CombinedLength_met;
    } else {
        rtb_CrvChng_1pm2_f0cp = rtb_Length_met_kx1h;
    }

    /* End of Switch: '<S809>/Switch5' */

    /* Switch generated from: '<S812>/Switch' incorporates:
     *  Inport: '<Root>/Inport39'
     *  Switch generated from: '<S812>/Switch1'
     *  Switch generated from: '<S812>/Switch2'
     */
    if (TTG_OD_Enable_bool) {
        rtb_CrvChng_1pm2_f0cp = ODPFOH_TgtObjLength_met;
    } else if (TTG_LD_Enable_bool) {
        /* Switch generated from: '<S812>/Switch1' */
        rtb_CrvChng_1pm2_f0cp = rtb_Length_met_kx1h;
    } else {
        if (!TTG_CMB_Enable_bool) {
            /* MultiPortSwitch generated from: '<S811>/Multiport Switch'
             * incorporates: Constant: '<S827>/Constant5' Inport:
             * '<Root>/Inport39' Switch generated from: '<S812>/Switch1' Switch
             * generated from: '<S812>/Switch2'
             */
            switch (rtb_TTG_Switch_nu) {
                case 0:
                    break;

                case 1:
                    rtb_CrvChng_1pm2_f0cp = rtb_Length_met_kx1h;
                    break;

                case 2:
                    rtb_CrvChng_1pm2_f0cp = ODPFOH_TgtObjLength_met;
                    break;

                default:
                    rtb_CrvChng_1pm2_f0cp = 0.0F;
                    break;
            }
        }
    }

    /* Switch: '<S816>/Switch10' incorporates:
     *  Switch: '<S818>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S816>/Unit Delay5'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S816>/Switch11' incorporates:
         *  Logic: '<S816>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S816>/Unit Delay5'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_RiCorridorLengthUnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.Length_met;
        }

        /* End of Switch: '<S816>/Switch11' */

        /* BusCreator generated from: '<S884>/Bus Creator' incorporates:
         *  Constant: '<S824>/Constant'
         *  Product: '<S824>/Product'
         *  Product: '<S824>/Product1'
         *  Sum: '<S824>/Add'
         *  Sum: '<S824>/Subtract'
         *  UnitDelay: '<S816>/Unit Delay5'
         */
        rtb_CrvChng_1pm2_f0cp = (TTG_RiCorridorLengthUnitDelay_met * rtb_out) +
                                ((1.0F - rtb_out) * rtb_CrvChng_1pm2_f0cp);
    } else {
        TTG_RiCorridorLengthUnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
                .Length_met;
    }

    /* End of Switch: '<S816>/Switch10' */

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' incorporates:
     *  Inport: '<Root>/Inport23'
     *  Inport: '<Root>/Inport28'
     *  Inport: '<Root>/Inport58'
     */
    switch (rtb_DataTypeConversion1) {
        case 0:
            rtb_Length_met_kx1h = ABPR_CntrLnClthPosX0_met;
            break;

        case 1:
            rtb_Length_met_kx1h = ABPR_LeLnClthPosX0_met;
            break;

        case 2:
            rtb_Length_met_kx1h = ABPR_RiLnClthPosX0_met;
            break;

        default:
            rtb_Length_met_kx1h = ABPR_CntrLnClthPosX0_met;
            break;
    }

    /* Switch: '<S879>/Switch' incorporates:
     *  Constant: '<S879>/Constant1'
     *  Logic: '<S879>/AND'
     *  Switch: '<S879>/Switch1'
     *  Switch: '<S879>/Switch2'
     */
    if ((TTG_LaneUpdate_bool) && (TTG_ObjectUpdate_bool)) {
        rtb_TTG_Switch_nu = 0U;
    } else if (TTG_LaneUpdate_bool) {
        /* Switch: '<S879>/Switch1' incorporates:
         *  Constant: '<S879>/Constant'
         */
        rtb_TTG_Switch_nu = 1U;
    } else if (TTG_ObjectUpdate_bool) {
        /* Switch: '<S879>/Switch2' incorporates:
         *  Constant: '<S879>/Constant2'
         *  Switch: '<S879>/Switch1'
         */
        rtb_TTG_Switch_nu = 2U;
    } else {
        /* Switch: '<S879>/Switch1' incorporates:
         *  Constant: '<S879>/Constant3'
         *  Switch: '<S879>/Switch2'
         */
        rtb_TTG_Switch_nu = 3U;
    }

    /* End of Switch: '<S879>/Switch' */

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport60'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_Length_met_kx1h = ODPFOH_TgtObjPosX0_met;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_Length_met_kx1h = TJACMB_CombinedPosX0_met;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant' Inport:
                 * '<Root>/Inport60'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_Length_met_kx1h = TJACMB_CombinedPosX0_met;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_Length_met_kx1h = ODPFOH_TgtObjPosX0_met;
                        break;

                    default:
                        rtb_Length_met_kx1h = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch' incorporates:
     *  Constant: '<S876>/Constant'
     *  Product: '<S876>/Product'
     *  Product: '<S876>/Product1'
     *  Sum: '<S876>/Add'
     *  Sum: '<S876>/Subtract'
     *  Switch: '<S870>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch1' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorPosX0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .PosX0_met;
        }

        /* End of Switch: '<S867>/Switch1' */
        rtb_Length_met_kx1h = (TTG_TgtCorridorPosX0UnitDelay_met * rtb_out) +
                              ((1.0F - rtb_out) * rtb_Length_met_kx1h);
    } else {
        TTG_TgtCorridorPosX0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                .PosX0_met;
    }

    /* End of Switch: '<S867>/Switch' */

    /* Switch: '<S691>/Switch' incorporates:
     *  Inport: '<Root>/Inport3'
     *  Inport: '<Root>/Inport4'
     *  Inport: '<Root>/Inport9'
     *  Switch: '<S692>/Switch'
     *  Switch: '<S693>/Switch'
     *  UnitDelay: '<S691>/Unit Delay'
     *  UnitDelay: '<S692>/Unit Delay'
     *  UnitDelay: '<S693>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_CntrLnCrvUnitDelay_1pm = ABPR_CntrLnClthCrv_1pm;
        TTG_CntrLnHeadingUnitDelay_rad = ABPR_CntrLnClthHeading_rad;
        TTG_CntrLnPosY0UnitDelay_met = ABPR_CntrLnClthPosY0_met;
    }

    /* End of Switch: '<S691>/Switch' */

    /* Product: '<S697>/Product' incorporates:
     *  Product: '<S696>/Product10'
     *  Switch generated from: '<S682>/Switch'
     *  UnitDelay: '<S692>/Unit Delay'
     */
    rtb_x_yaw_hpvs_tmp =
        rtb_uDLookupTable1_eoam * TTG_CntrLnHeadingUnitDelay_rad;

    /* Product: '<S697>/Product6' incorporates:
     *  Product: '<S696>/Product9'
     *  Switch generated from: '<S682>/Switch'
     *  UnitDelay: '<S691>/Unit Delay'
     */
    rtb_Length_out_ltpy *= TTG_CntrLnCrvUnitDelay_1pm;

    /* Product: '<S697>/Product7' incorporates:
     *  Product: '<S697>/Product'
     *  Product: '<S697>/Product6'
     *  Sum: '<S697>/Add'
     *  Sum: '<S697>/Add1'
     *  Sum: '<S697>/Subtract'
     *  UnitDelay: '<S693>/Unit Delay'
     */
    rtb_x_yaw_bzv5 =
        (rtb_Length_out_ltpy +
         (rtb_x_yaw_hpvs_tmp + (TTG_CntrLnPosY0UnitDelay_met - rtb_y_fcnl))) *
        rtb_Add_fbcu;

    /* Sum: '<S698>/Subtract1' incorporates:
     *  Sum: '<S694>/Subtract1'
     *  Switch generated from: '<S682>/Switch'
     */
    rtb_uDLookupTable_prmv_tmp = rtb_uDLookupTable1_eoam - rtb_x_yaw_bzv5;

    /* Sum: '<S698>/Add1' incorporates:
     *  Constant: '<S698>/Constant'
     *  Product: '<S698>/Product'
     *  Product: '<S698>/Product1'
     *  Product: '<S698>/Product2'
     *  Product: '<S698>/Product6'
     *  Sum: '<S698>/Add'
     *  Sum: '<S698>/Subtract'
     *  Sum: '<S698>/Subtract1'
     *  UnitDelay: '<S691>/Unit Delay'
     *  UnitDelay: '<S692>/Unit Delay'
     *  UnitDelay: '<S693>/Unit Delay'
     */
    rtb_PosY0_met_o0pf =
        (((rtb_uDLookupTable_prmv_tmp * rtb_uDLookupTable_prmv_tmp) * 0.5F) *
         TTG_CntrLnCrvUnitDelay_1pm) +
        ((rtb_uDLookupTable_prmv_tmp * TTG_CntrLnHeadingUnitDelay_rad) +
         (TTG_CntrLnPosY0UnitDelay_met - rtb_y_fcnl));

    /* Sum: '<S696>/Add4' incorporates:
     *  Product: '<S696>/Product11'
     *  Product: '<S696>/Product12'
     */
    rtb_PosY0_met_o0pf = (rtb_x_yaw_bzv5 * rtb_x_yaw_bzv5) +
                         (rtb_PosY0_met_o0pf * rtb_PosY0_met_o0pf);

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' incorporates:
     *  Constant: '<S832>/Constant2'
     *  Constant: '<S834>/Constant2'
     *  Product: '<S832>/Product'
     *  Product: '<S834>/Product'
     *  Sum: '<S832>/Add1'
     *  Sum: '<S834>/Add1'
     */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch: '<S837>/Switch2' incorporates:
             *  Sum: '<S837>/Add2'
             *  Switch generated from: '<S682>/Switch'
             *  Switch: '<S837>/Switch3'
             */
            if (rtb_Equal_awdj) {
                rtb_PosY0_met_o0pf = rtb_ABPR_RiLnClthPosY0_met + 1.5F;
            } else if (rtb_SLC_LeTurnSignalOn_bool) {
                /* Switch: '<S837>/Switch3' incorporates:
                 *  Sum: '<S837>/Add3'
                 */
                rtb_PosY0_met_o0pf = rtb_ABPR_LeLnClthPosY0_met - 1.5F;
            } else if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S696>/Switch' incorporates:
                 *  Constant: '<S696>/Constant2'
                 *  Product: '<S696>/Product10'
                 *  Product: '<S696>/Product7'
                 *  Product: '<S696>/Product8'
                 *  Product: '<S696>/Product9'
                 *  RelationalOperator: '<S696>/GreaterThan'
                 *  Sqrt: '<S696>/Sqrt'
                 *  Sum: '<S696>/Add2'
                 *  Sum: '<S696>/Add3'
                 *  Switch generated from: '<S682>/Switch'
                 *  UnitDelay: '<S691>/Unit Delay'
                 *  UnitDelay: '<S692>/Unit Delay'
                 *  UnitDelay: '<S693>/Unit Delay'
                 *
                 * About '<S696>/Sqrt':
                 *  Operator: signedSqrt
                 */
                if (rtb_y_fcnl >
                    ((((rtb_uDLookupTable1_eoam * rtb_uDLookupTable1_eoam) *
                       0.5F) *
                      TTG_CntrLnCrvUnitDelay_1pm) +
                     ((rtb_uDLookupTable1_eoam *
                       TTG_CntrLnHeadingUnitDelay_rad) +
                      TTG_CntrLnPosY0UnitDelay_met))) {
                    /* Sqrt: '<S696>/Sqrt'
                     *
                     * About '<S696>/Sqrt':
                     *  Operator: signedSqrt
                     */
                    if (rtb_PosY0_met_o0pf < 0.0F) {
                        /* Switch: '<S837>/Switch3' */
                        rtb_PosY0_met_o0pf = sqrtf(fabsf(rtb_PosY0_met_o0pf));
                    } else {
                        /* Switch: '<S837>/Switch3' incorporates:
                         *  UnaryMinus: '<S696>/Unary Minus'
                         */
                        rtb_PosY0_met_o0pf = -sqrtf(rtb_PosY0_met_o0pf);
                    }
                } else if (rtb_PosY0_met_o0pf < 0.0F) {
                    /* Sqrt: '<S696>/Sqrt' incorporates:
                     *  Switch: '<S837>/Switch3'
                     *
                     * About '<S696>/Sqrt':
                     *  Operator: signedSqrt
                     */
                    rtb_PosY0_met_o0pf = -sqrtf(fabsf(rtb_PosY0_met_o0pf));
                } else {
                    /* Switch: '<S837>/Switch3' incorporates:
                     *  Sqrt: '<S696>/Sqrt'
                     *
                     * About '<S696>/Sqrt':
                     *  Operator: signedSqrt
                     */
                    rtb_PosY0_met_o0pf = sqrtf(rtb_PosY0_met_o0pf);
                }
            } else {
                /* Switch: '<S837>/Switch3' incorporates:
                 *  Inport: '<Root>/Inport3'
                 */
                rtb_PosY0_met_o0pf = ABPR_CntrLnClthPosY0_met;
            }

            /* End of Switch: '<S837>/Switch2' */
            break;

        case 1:
            rtb_PosY0_met_o0pf =
                (rtb_ABPR_LeLnClthPosY0_met + rtb_PosY0_met) * 0.5F;
            break;

        case 2:
            rtb_PosY0_met_o0pf =
                (rtb_ABPR_RiLnClthPosY0_met + rtb_PosY0_met_bzv0) * 0.5F;
            break;

        default:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport3'
             */
            if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S696>/Switch' incorporates:
                 *  RelationalOperator: '<S696>/GreaterThan'
                 *  Sqrt: '<S696>/Sqrt'
                 *  Sum: '<S696>/Add2'
                 *  Sum: '<S696>/Add3'
                 *  UnitDelay: '<S693>/Unit Delay'
                 *
                 * About '<S696>/Sqrt':
                 *  Operator: signedSqrt
                 */
                if (rtb_y_fcnl >
                    (rtb_Length_out_ltpy +
                     (rtb_x_yaw_hpvs_tmp + TTG_CntrLnPosY0UnitDelay_met))) {
                    /* Sqrt: '<S696>/Sqrt' incorporates:
                     *  UnaryMinus: '<S696>/Unary Minus'
                     *
                     * About '<S696>/Sqrt':
                     *  Operator: signedSqrt
                     */
                    if (rtb_PosY0_met_o0pf < 0.0F) {
                        rtb_PosY0_met_o0pf = sqrtf(fabsf(rtb_PosY0_met_o0pf));
                    } else {
                        rtb_PosY0_met_o0pf = -sqrtf(rtb_PosY0_met_o0pf);
                    }
                } else if (rtb_PosY0_met_o0pf < 0.0F) {
                    /* Sqrt: '<S696>/Sqrt'
                     *
                     * About '<S696>/Sqrt':
                     *  Operator: signedSqrt
                     */
                    rtb_PosY0_met_o0pf = -sqrtf(fabsf(rtb_PosY0_met_o0pf));
                } else {
                    rtb_PosY0_met_o0pf = sqrtf(rtb_PosY0_met_o0pf);
                }
            } else {
                rtb_PosY0_met_o0pf = ABPR_CntrLnClthPosY0_met;
            }
            break;
    }

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport36'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosY0_met_o0pf = ODPFOH_TgtObjPosY0_met;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_PosY0_met_o0pf = TJACMB_CombinedPosY0_met;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant1' Inport:
                 * '<Root>/Inport36'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_PosY0_met_o0pf = TJACMB_CombinedPosY0_met;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_PosY0_met_o0pf = ODPFOH_TgtObjPosY0_met;
                        break;

                    default:
                        rtb_PosY0_met_o0pf = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch2' incorporates:
     *  Constant: '<S877>/Constant'
     *  Product: '<S877>/Product'
     *  Product: '<S877>/Product1'
     *  Sum: '<S877>/Add'
     *  Sum: '<S877>/Subtract'
     *  Switch: '<S871>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay1'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch3' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay1'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorPosY0UnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .PosY0_met;
        }

        /* End of Switch: '<S867>/Switch3' */
        rtb_PosY0_met_o0pf = (TTG_TgtCorridorPosY0UnitDelay_met * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosY0_met_o0pf);
    } else {
        TTG_TgtCorridorPosY0UnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                .PosY0_met;
    }

    /* End of Switch: '<S867>/Switch2' */

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport4'
             *  Product: '<S694>/Product'
             *  Sum: '<S694>/Add'
             *  Sum: '<S694>/Add1'
             *  Sum: '<S694>/Subtract1'
             *  UnitDelay: '<S691>/Unit Delay'
             *  UnitDelay: '<S692>/Unit Delay'
             */
            if (TJATTG_PredictionEnable_bool) {
                rtb_ABPR_LeLnClthHeading_rad =
                    ((TTG_CntrLnCrvUnitDelay_1pm *
                      (rtb_uDLookupTable1_eoam - rtb_x_yaw_bzv5)) +
                     TTG_CntrLnHeadingUnitDelay_rad) +
                    rtb_Add_fbcu;
            } else {
                rtb_ABPR_LeLnClthHeading_rad = ABPR_CntrLnClthHeading_rad;
            }
            break;

        case 1:
            break;

        case 2:
            rtb_ABPR_LeLnClthHeading_rad = rtb_ABPR_RiLnClthHeading_rad;
            break;

        default:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport4'
             *  Product: '<S694>/Product'
             *  Sum: '<S694>/Add'
             *  Sum: '<S694>/Add1'
             *  UnitDelay: '<S691>/Unit Delay'
             *  UnitDelay: '<S692>/Unit Delay'
             */
            if (TJATTG_PredictionEnable_bool) {
                rtb_ABPR_LeLnClthHeading_rad =
                    ((TTG_CntrLnCrvUnitDelay_1pm * rtb_uDLookupTable_prmv_tmp) +
                     TTG_CntrLnHeadingUnitDelay_rad) +
                    rtb_Add_fbcu;
            } else {
                rtb_ABPR_LeLnClthHeading_rad = ABPR_CntrLnClthHeading_rad;
            }
            break;
    }

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport37'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_ABPR_LeLnClthHeading_rad = ODPFOH_TgtObjHeadAng_rad;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_ABPR_LeLnClthHeading_rad = TJACMB_CombinedHeading_rad;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant2' Inport:
                 * '<Root>/Inport37'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_ABPR_LeLnClthHeading_rad =
                            TJACMB_CombinedHeading_rad;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_ABPR_LeLnClthHeading_rad = ODPFOH_TgtObjHeadAng_rad;
                        break;

                    default:
                        rtb_ABPR_LeLnClthHeading_rad = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch4' incorporates:
     *  Constant: '<S874>/Constant'
     *  Product: '<S874>/Product'
     *  Product: '<S874>/Product1'
     *  Sum: '<S874>/Add'
     *  Sum: '<S874>/Subtract'
     *  Switch: '<S868>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay2'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch5' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay2'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorHeadingUnitDelay_rad =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .Heading_rad;
        }

        /* End of Switch: '<S867>/Switch5' */
        rtb_ABPR_LeLnClthHeading_rad =
            (TTG_TgtCorridorHeadingUnitDelay_rad * rtb_out) +
            ((1.0F - rtb_out) * rtb_ABPR_LeLnClthHeading_rad);
    } else {
        TTG_TgtCorridorHeadingUnitDelay_rad =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                .Heading_rad;
    }

    /* End of Switch: '<S867>/Switch4' */

    /* Switch: '<S702>/Switch1' incorporates:
     *  Logic: '<S701>/AND'
     *  Logic: '<S701>/NOT'
     *  Switch: '<S702>/Switch'
     *  UnitDelay: '<S691>/Unit Delay'
     *  UnitDelay: '<S701>/Unit Delay'
     *  UnitDelay: '<S702>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_CntrLnPredictEnableEdgeRising_bool)) {
        TTG_CntrCrvPredictLowPass_1pm = TTG_CntrLnCrvUnitDelay_1pm;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S702>/Unit Delay' incorporates:
             *  Constant: '<S689>/Constant1'
             *  Constant: '<S702>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S702>/Divide'
             *  Product: '<S702>/Product1'
             *  Sum: '<S702>/Subtract'
             *  Switch: '<S702>/Switch'
             */
            TTG_CntrCrvPredictLowPass_1pm =
                (1.0F -
                 (LCFRCV_TSysCycleTimeSen_sec / TJATTG_PredCrvPT1_C_sec)) *
                TTG_CntrCrvPredictLowPass_1pm;
        }
    }

    /* End of Switch: '<S702>/Switch1' */

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport9'
             */
            if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S689>/Switch' incorporates:
                 *  Constant: '<S689>/Constant'
                 *  Logic: '<S689>/NOT'
                 *  UnitDelay: '<S691>/Unit Delay'
                 *  UnitDelay: '<S702>/Unit Delay'
                 */
                if (!TJATTG_PredFreezeCrv_C_bool) {
                    rtb_PosY0_met_dafa = TTG_CntrCrvPredictLowPass_1pm;
                } else {
                    rtb_PosY0_met_dafa = TTG_CntrLnCrvUnitDelay_1pm;
                }
            } else {
                rtb_PosY0_met_dafa = ABPR_CntrLnClthCrv_1pm;
            }
            break;

        case 1:
            break;

        case 2:
            rtb_PosY0_met_dafa = rtb_ABPR_RiLnClthCrv_1pm;
            break;

        default:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport9'
             */
            if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S689>/Switch' incorporates:
                 *  Constant: '<S689>/Constant'
                 *  Logic: '<S689>/NOT'
                 *  UnitDelay: '<S691>/Unit Delay'
                 *  UnitDelay: '<S702>/Unit Delay'
                 */
                if (!TJATTG_PredFreezeCrv_C_bool) {
                    rtb_PosY0_met_dafa = TTG_CntrCrvPredictLowPass_1pm;
                } else {
                    rtb_PosY0_met_dafa = TTG_CntrLnCrvUnitDelay_1pm;
                }
            } else {
                rtb_PosY0_met_dafa = ABPR_CntrLnClthCrv_1pm;
            }
            break;
    }

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport38'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_PosY0_met_dafa = ODPFOH_TgtObjCrv_1pm;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_PosY0_met_dafa = TJACMB_CombinedCrv_1pm;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant3' Inport:
                 * '<Root>/Inport38'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_PosY0_met_dafa = TJACMB_CombinedCrv_1pm;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_PosY0_met_dafa = ODPFOH_TgtObjCrv_1pm;
                        break;

                    default:
                        rtb_PosY0_met_dafa = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch6' incorporates:
     *  Constant: '<S873>/Constant'
     *  Product: '<S873>/Product'
     *  Product: '<S873>/Product1'
     *  Sum: '<S873>/Add'
     *  Sum: '<S873>/Subtract'
     *  Switch: '<S866>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay3'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch7' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay3'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorCrvUnitDelay_1pm =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .Crv_1pm;
        }

        /* End of Switch: '<S867>/Switch7' */
        rtb_PosY0_met_dafa = (TTG_TgtCorridorCrvUnitDelay_1pm * rtb_out) +
                             ((1.0F - rtb_out) * rtb_PosY0_met_dafa);
    } else {
        TTG_TgtCorridorCrvUnitDelay_1pm = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                              .TTG_TargetTrajectory_bus.Crv_1pm;
    }

    /* End of Switch: '<S867>/Switch6' */

    /* Switch: '<S690>/Switch' incorporates:
     *  Inport: '<Root>/Inport59'
     *  UnitDelay: '<S690>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_CntrLnCrvChngUnitDelay_1pm2 = ABPR_CntrLnClthCrvChng_1pm2;
    }

    /* End of Switch: '<S690>/Switch' */

    /* Switch: '<S700>/Switch1' incorporates:
     *  Logic: '<S699>/AND'
     *  Logic: '<S699>/NOT'
     *  Switch: '<S700>/Switch'
     *  UnitDelay: '<S690>/Unit Delay'
     *  UnitDelay: '<S699>/Unit Delay'
     *  UnitDelay: '<S700>/Unit Delay'
     */
    if (TJATTG_PredictionEnable_bool &&
        (!TTG_CntrLnPredictEnable2EdgeRising_bool)) {
        TTG_CntrLnCrvChngPredictLowPass_1pm2 = TTG_CntrLnCrvChngUnitDelay_1pm2;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S700>/Unit Delay' incorporates:
             *  Constant: '<S688>/Constant1'
             *  Constant: '<S700>/Constant1'
             *  Inport: '<Root>/Inport10'
             *  Product: '<S700>/Divide'
             *  Product: '<S700>/Product1'
             *  Sum: '<S700>/Subtract'
             *  Switch: '<S700>/Switch'
             */
            TTG_CntrLnCrvChngPredictLowPass_1pm2 =
                (1.0F -
                 (LCFRCV_TSysCycleTimeSen_sec / TJATTG_PredCrvChngPT1_C_sec)) *
                TTG_CntrLnCrvChngPredictLowPass_1pm2;
        }
    }

    /* End of Switch: '<S700>/Switch1' */

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport59'
             */
            if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S688>/Switch' incorporates:
                 *  Constant: '<S688>/Constant'
                 *  Logic: '<S688>/NOT'
                 *  UnitDelay: '<S690>/Unit Delay'
                 *  UnitDelay: '<S700>/Unit Delay'
                 */
                if (!TJATTG_PredFreezeCrvChng_C_bool) {
                    rtb_x_yaw = TTG_CntrLnCrvChngPredictLowPass_1pm2;
                } else {
                    rtb_x_yaw = TTG_CntrLnCrvChngUnitDelay_1pm2;
                }
            } else {
                rtb_x_yaw = ABPR_CntrLnClthCrvChng_1pm2;
            }
            break;

        case 1:
            break;

        case 2:
            rtb_x_yaw = rtb_ABPR_RiLnClthCrvChng_1pm2;
            break;

        default:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Inport: '<Root>/Inport59'
             */
            if (TJATTG_PredictionEnable_bool) {
                /* Switch: '<S688>/Switch' incorporates:
                 *  Constant: '<S688>/Constant'
                 *  Logic: '<S688>/NOT'
                 *  UnitDelay: '<S690>/Unit Delay'
                 *  UnitDelay: '<S700>/Unit Delay'
                 */
                if (!TJATTG_PredFreezeCrvChng_C_bool) {
                    rtb_x_yaw = TTG_CntrLnCrvChngPredictLowPass_1pm2;
                } else {
                    rtb_x_yaw = TTG_CntrLnCrvChngUnitDelay_1pm2;
                }
            } else {
                rtb_x_yaw = ABPR_CntrLnClthCrvChng_1pm2;
            }
            break;
    }

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport61'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_x_yaw = ODPFOH_TgtObjCrvChng_1pm2;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_x_yaw = TJACMB_CombinedCrvChng_1pm2;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant4' Inport:
                 * '<Root>/Inport61'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_x_yaw = TJACMB_CombinedCrvChng_1pm2;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_x_yaw = ODPFOH_TgtObjCrvChng_1pm2;
                        break;

                    default:
                        rtb_x_yaw = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch8' incorporates:
     *  Constant: '<S872>/Constant'
     *  Product: '<S872>/Product'
     *  Product: '<S872>/Product1'
     *  Sum: '<S872>/Add'
     *  Sum: '<S872>/Subtract'
     *  Switch: '<S865>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay4'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch9' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay4'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorCrvChngUnitDelay_1pm2 =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .CrvChng_1pm2;
        }

        /* End of Switch: '<S867>/Switch9' */
        rtb_x_yaw = (TTG_TgtCorridorCrvChngUnitDelay_1pm2 * rtb_out) +
                    ((1.0F - rtb_out) * rtb_x_yaw);
    } else {
        TTG_TgtCorridorCrvChngUnitDelay_1pm2 =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                .CrvChng_1pm2;
    }

    /* End of Switch: '<S867>/Switch8' */

    /* Switch: '<S695>/Switch' incorporates:
     *  Inport: '<Root>/Inport21'
     *  UnitDelay: '<S695>/Unit Delay'
     */
    if (!TJATTG_PredictionEnable_bool) {
        TTG_CntrLnLengthUnitDelay_met = ABPR_CntrLnClthLength_met;
    }

    /* End of Switch: '<S695>/Switch' */

    /* Sum: '<S695>/Subtract' incorporates:
     *  UnitDelay: '<S695>/Unit Delay'
     */
    rtb_Length_out_ltpy =
        TTG_CntrLnLengthUnitDelay_met - rtb_uDLookupTable1_eoam;

    /* MultiPortSwitch generated from: '<S829>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Constant: '<S687>/Constant'
             *  Inport: '<Root>/Inport21'
             *  MinMax: '<S687>/Max'
             */
            if (TJATTG_PredictionEnable_bool) {
                rtb_ABPR_LeLnClthLength_met = fmaxf(
                    rtb_Length_out_ltpy, TJATTG_LnPredMinTrajLength_C_met);
            } else {
                rtb_ABPR_LeLnClthLength_met = ABPR_CntrLnClthLength_met;
            }
            break;

        case 1:
            break;

        case 2:
            rtb_ABPR_LeLnClthLength_met = rtb_ABPR_RiLnClthLength_met;
            break;

        default:
            /* Switch generated from: '<S682>/Switch' incorporates:
             *  Constant: '<S687>/Constant'
             *  Inport: '<Root>/Inport21'
             *  MinMax: '<S687>/Max'
             */
            if (TJATTG_PredictionEnable_bool) {
                rtb_ABPR_LeLnClthLength_met = fmaxf(
                    rtb_Length_out_ltpy, TJATTG_LnPredMinTrajLength_C_met);
            } else {
                rtb_ABPR_LeLnClthLength_met = ABPR_CntrLnClthLength_met;
            }
            break;
    }

    /* Switch generated from: '<S863>/Switch' incorporates:
     *  Inport: '<Root>/Inport39'
     *  Switch generated from: '<S863>/Switch1'
     */
    if (TTG_OD_Enable_bool) {
        rtb_ABPR_LeLnClthLength_met = ODPFOH_TgtObjLength_met;
    } else {
        if (!TTG_LD_Enable_bool) {
            /* Switch generated from: '<S863>/Switch2' incorporates:
             *  Switch generated from: '<S863>/Switch1'
             */
            if (TTG_CMB_Enable_bool) {
                rtb_ABPR_LeLnClthLength_met = TJACMB_CombinedLength_met;
            } else {
                /* MultiPortSwitch generated from: '<S862>/Multiport Switch'
                 * incorporates: Constant: '<S878>/Constant5' Inport:
                 * '<Root>/Inport39'
                 */
                switch (rtb_TTG_Switch_nu) {
                    case 0:
                        rtb_ABPR_LeLnClthLength_met = TJACMB_CombinedLength_met;
                        break;

                    case 1:
                        break;

                    case 2:
                        rtb_ABPR_LeLnClthLength_met = ODPFOH_TgtObjLength_met;
                        break;

                    default:
                        rtb_ABPR_LeLnClthLength_met = 0.0F;
                        break;
                }
            }
        }
    }

    /* Switch: '<S867>/Switch10' incorporates:
     *  Constant: '<S875>/Constant'
     *  Product: '<S875>/Product'
     *  Product: '<S875>/Product1'
     *  Sum: '<S875>/Add'
     *  Sum: '<S875>/Subtract'
     *  Switch: '<S869>/Switch'
     *  UnitDelay: '<S676>/Unit Delay1'
     *  UnitDelay: '<S867>/Unit Delay5'
     */
    if (TTG_Enable_bool) {
        /* Switch: '<S867>/Switch11' incorporates:
         *  Logic: '<S867>/NOT'
         *  UnitDelay: '<S676>/Unit Delay1'
         *  UnitDelay: '<S867>/Unit Delay5'
         */
        if (!rtb_Equal_i3l2_idx_1) {
            TTG_TgtCorridorLengthUnitDelay_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .Length_met;
        }

        /* End of Switch: '<S867>/Switch11' */
        rtb_ABPR_LeLnClthLength_met =
            (TTG_TgtCorridorLengthUnitDelay_met * rtb_out) +
            ((1.0F - rtb_out) * rtb_ABPR_LeLnClthLength_met);
    } else {
        TTG_TgtCorridorLengthUnitDelay_met =
            TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                .Length_met;
    }

    /* End of Switch: '<S867>/Switch10' */

    /* Switch: '<S676>/Switch' incorporates:
     *  BusCreator: '<S884>/Bus Creator'
     *  Constant: '<S881>/Constant'
     *  Constant: '<S882>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S676>/Equal1'
     *  RelationalOperator: '<S676>/Equal2'
     *  Switch: '<S552>/Switch3'
     *  Switch: '<S676>/Switch1'
     *  UnitDelay: '<S676>/Unit Delay1'
     * */
    if (((uint32_T)TJASTM_SysStateTJA_nu) ==
        E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) {
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .PosX0_met = rtb_PosX0_met_ckhb;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .PosY0_met = rtb_x_yaw_hyuw;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Heading_rad = rtb_Heading_rad_poq4;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Crv_1pm = rtb_Crv_1pm_b0ly;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .CrvChng_1pm2 = rtb_CrvChng_1pm2_m00x;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Length_met = rtb_Length_met_ffe5;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .PosX0_met = rtb_Length_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .PosY0_met = rtb_PosX0_met_exq3;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Heading_rad = rtb_PosY0_met_lcpf;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Crv_1pm = rtb_PosY0_met_cmjs;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .CrvChng_1pm2 = rtb_Crv_1pm_mhaw;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Length_met = rtb_CrvChng_1pm2_f0cp;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosX0_met =
            rtb_Length_met_kx1h;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosY0_met =
            rtb_PosY0_met_o0pf;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .Heading_rad = rtb_ABPR_LeLnClthHeading_rad;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.Crv_1pm =
            rtb_PosY0_met_dafa;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .CrvChng_1pm2 = rtb_x_yaw;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .Length_met = rtb_ABPR_LeLnClthLength_met;
    } else {
        if (((uint32_T)TJASTM_SysStateTJA_nu) ==
            E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT) {
            /* Switch: '<S676>/Switch1' incorporates:
             *  UnitDelay: '<S676>/Unit Delay1'
             */
            rtb_PosY0_met_dafa = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_LeftCorridorClothoid_bus.PosX0_met;
            rtb_ABPR_RiLnClthPosY0_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .PosY0_met;
            rtb_x_yaw = TTG_TgtTrajAndCridrBndUnitDelay_bus
                            .TTG_LeftCorridorClothoid_bus.Heading_rad;
            rtb_ABPR_LeLnClthLength_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .Crv_1pm;
            rtb_x_yaw_hyuw = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                 .TTG_LeftCorridorClothoid_bus.CrvChng_1pm2;
            rtb_ABPR_LeLnClthPosY0_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
                    .Length_met;
            rtb_PosY0_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                .TTG_RightCorridorClothoid_bus.PosX0_met;
            rtb_out = TTG_TgtTrajAndCridrBndUnitDelay_bus
                          .TTG_RightCorridorClothoid_bus.PosY0_met;
            rtb_PosX0_met_ckhb = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_RightCorridorClothoid_bus.Heading_rad;
            rtb_ABPR_LeLnClthHeading_rad =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.Crv_1pm;
            rtb_Heading_rad_poq4 =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.CrvChng_1pm2;
            rtb_ABPR_RiLnClthHeading_rad =
                TTG_TgtTrajAndCridrBndUnitDelay_bus
                    .TTG_RightCorridorClothoid_bus.Length_met;
            rtb_ABPR_RiLnClthCrv_1pm = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                           .TTG_TargetTrajectory_bus.PosX0_met;
            rtb_Crv_1pm_b0ly = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                   .TTG_TargetTrajectory_bus.PosY0_met;
            rtb_ABPR_RiLnClthCrvChng_1pm2 =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .Heading_rad;
            rtb_CrvChng_1pm2_m00x = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                        .TTG_TargetTrajectory_bus.Crv_1pm;
            rtb_ABPR_RiLnClthLength_met =
                TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
                    .CrvChng_1pm2;
            rtb_Length_met_ffe5 = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                      .TTG_TargetTrajectory_bus.Length_met;
        } else {
            /* Switch: '<S676>/Switch1' incorporates:
             *  BusCreator generated from: '<S880>/Bus Creator'
             *  Constant: '<S885>/Constant'
             *  Constant: '<S885>/Constant1'
             *  Constant: '<S885>/Constant2'
             *  Constant: '<S885>/Constant3'
             *  Constant: '<S885>/Constant4'
             *  Constant: '<S885>/Constant5'
             *  Constant: '<S886>/Constant'
             *  Constant: '<S886>/Constant1'
             *  Constant: '<S886>/Constant2'
             *  Constant: '<S886>/Constant3'
             *  Constant: '<S886>/Constant4'
             *  Constant: '<S886>/Constant5'
             *  Constant: '<S887>/Constant'
             *  Constant: '<S887>/Constant1'
             *  Constant: '<S887>/Constant2'
             *  Constant: '<S887>/Constant3'
             *  Constant: '<S887>/Constant4'
             *  Constant: '<S887>/Constant5'
             * */
            rtb_PosY0_met_dafa = 0.0F;
            rtb_ABPR_RiLnClthPosY0_met = 0.0F;
            rtb_x_yaw = 0.0F;
            rtb_ABPR_LeLnClthLength_met = 0.0F;
            rtb_x_yaw_hyuw = 0.0F;
            rtb_ABPR_LeLnClthPosY0_met = 0.0F;
            rtb_PosY0_met = 0.0F;
            rtb_out = 0.0F;
            rtb_PosX0_met_ckhb = 0.0F;
            rtb_ABPR_LeLnClthHeading_rad = 0.0F;
            rtb_Heading_rad_poq4 = 0.0F;
            rtb_ABPR_RiLnClthHeading_rad = 0.0F;
            rtb_ABPR_RiLnClthCrv_1pm = 0.0F;
            rtb_Crv_1pm_b0ly = 0.0F;
            rtb_ABPR_RiLnClthCrvChng_1pm2 = 0.0F;
            rtb_CrvChng_1pm2_m00x = 0.0F;
            rtb_ABPR_RiLnClthLength_met = 0.0F;
            rtb_Length_met_ffe5 = 0.0F;
        }

        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .PosX0_met = rtb_PosY0_met_dafa;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .PosY0_met = rtb_ABPR_RiLnClthPosY0_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Heading_rad = rtb_x_yaw;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Crv_1pm = rtb_ABPR_LeLnClthLength_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .CrvChng_1pm2 = rtb_x_yaw_hyuw;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Length_met = rtb_ABPR_LeLnClthPosY0_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .PosX0_met = rtb_PosY0_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .PosY0_met = rtb_out;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Heading_rad = rtb_PosX0_met_ckhb;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Crv_1pm = rtb_ABPR_LeLnClthHeading_rad;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .CrvChng_1pm2 = rtb_Heading_rad_poq4;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Length_met = rtb_ABPR_RiLnClthHeading_rad;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosX0_met =
            rtb_ABPR_RiLnClthCrv_1pm;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosY0_met =
            rtb_Crv_1pm_b0ly;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .Heading_rad = rtb_ABPR_RiLnClthCrvChng_1pm2;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.Crv_1pm =
            rtb_CrvChng_1pm2_m00x;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .CrvChng_1pm2 = rtb_ABPR_RiLnClthLength_met;
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus
            .Length_met = rtb_Length_met_ffe5;
    }

    /* End of Switch: '<S676>/Switch' */

    /* SignalConversion: '<S10>/Signal Conversion12' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndPosX0_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_LeftCorridorClothoid_bus.PosX0_met;

    /* SignalConversion: '<S10>/Signal Conversion13' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndPosY0_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_LeftCorridorClothoid_bus.PosY0_met;

    /* SignalConversion: '<S10>/Signal Conversion14' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndHeadAng_rad =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .Heading_rad;

    /* SignalConversion: '<S10>/Signal Conversion15' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndCrv_1pm = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                   .TTG_LeftCorridorClothoid_bus.Crv_1pm;

    /* SignalConversion: '<S10>/Signal Conversion16' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndCrvChng_1pm2 =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_LeftCorridorClothoid_bus
            .CrvChng_1pm2;

    /* SignalConversion: '<S10>/Signal Conversion17' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_LeCridrBndLength_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                      .TTG_LeftCorridorClothoid_bus.Length_met;

    /* SignalConversion: '<S10>/Signal Conversion18' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndPosX0_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_RightCorridorClothoid_bus.PosX0_met;

    /* SignalConversion: '<S10>/Signal Conversion19' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndPosY0_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_RightCorridorClothoid_bus.PosY0_met;

    /* SignalConversion: '<S10>/Signal Conversion20' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndHeadAng_rad =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .Heading_rad;

    /* SignalConversion: '<S10>/Signal Conversion21' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndCrv_1pm = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                   .TTG_RightCorridorClothoid_bus.Crv_1pm;

    /* SignalConversion: '<S10>/Signal Conversion22' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndCrvChng_1pm2 =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_RightCorridorClothoid_bus
            .CrvChng_1pm2;

    /* SignalConversion: '<S10>/Signal Conversion23' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_RiCridrBndLength_met = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                      .TTG_RightCorridorClothoid_bus.Length_met;

    /* SignalConversion: '<S10>/Signal Conversion24' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajPosX0_met =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosX0_met;

    /* SignalConversion: '<S10>/Signal Conversion25' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajPosY0_met =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.PosY0_met;

    /* SignalConversion: '<S10>/Signal Conversion27' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajCrv_1pm =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.Crv_1pm;

    /* Switch: '<S890>/Switch' incorporates:
     *  Constant: '<S937>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S890>/Equal'
     *  Switch: '<S552>/Switch3'
     *  UnitDelay: '<S890>/Unit Delay'
     */
    if (E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT ==
        ((uint32_T)TJASTM_SysStateTJA_nu)) {
    } else {
        TVG_PrevLatCtlModeUnitDelay_bool = TJASTM_LatCtrlMode_nu;
    }

    /* End of Switch: '<S890>/Switch' */

    /* RelationalOperator: '<S779>/Equal7' incorporates:
     *  RelationalOperator: '<S890>/Equal1'
     *  RelationalOperator: '<S890>/Equal2'
     *  RelationalOperator: '<S890>/Equal3'
     *  RelationalOperator: '<S890>/Equal4'
     *  RelationalOperator: '<S890>/Equal5'
     *  RelationalOperator: '<S890>/Equal6'
     *  RelationalOperator: '<S890>/Equal7'
     *  RelationalOperator: '<S890>/Equal8'
     *  Switch: '<S890>/Switch'
     *  Switch: '<S946>/Switch'
     *  Switch: '<S946>/Switch1'
     *  UnitDelay: '<S890>/Unit Delay'
     */
    TTG_STMLatCtrlMode_Enum = TVG_PrevLatCtlModeUnitDelay_bool;

    /* Switch: '<S946>/Switch' incorporates:
     *  Constant: '<S938>/Constant'
     *  Constant: '<S939>/Constant'
     *  Constant: '<S940>/Constant'
     *  Constant: '<S941>/Constant'
     *  Constant: '<S942>/Constant'
     *  Constant: '<S943>/Constant'
     *  Constant: '<S944>/Constant'
     *  Constant: '<S945>/Constant'
     *  Constant: '<S946>/Constant1'
     *  Constant: '<S946>/Constant3'
     *  Logic: '<S890>/OR'
     *  Logic: '<S890>/OR1'
     *  Logic: '<S890>/OR2'
     *  RelationalOperator: '<S890>/Equal1'
     *  RelationalOperator: '<S890>/Equal2'
     *  RelationalOperator: '<S890>/Equal3'
     *  RelationalOperator: '<S890>/Equal4'
     *  RelationalOperator: '<S890>/Equal5'
     *  RelationalOperator: '<S890>/Equal6'
     *  RelationalOperator: '<S890>/Equal7'
     *  RelationalOperator: '<S890>/Equal8'
     *  Switch: '<S946>/Switch1'
     *  Switch: '<S946>/Switch2'
     */
    if ((((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC_RQ ==
           ((uint32_T)TTG_STMLatCtrlMode_Enum)) ||
          (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
           ((uint32_T)TTG_STMLatCtrlMode_Enum))) ||
         (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ ==
          ((uint32_T)TTG_STMLatCtrlMode_Enum))) ||
        (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB ==
         ((uint32_T)TTG_STMLatCtrlMode_Enum))) {
        rtb_DataTypeConversion1 = 0U;
    } else if ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF ==
                ((uint32_T)TTG_STMLatCtrlMode_Enum)) ||
               (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF_RQ ==
                ((uint32_T)TTG_STMLatCtrlMode_Enum))) {
        /* Switch: '<S946>/Switch1' incorporates:
         *  Constant: '<S946>/Constant'
         */
        rtb_DataTypeConversion1 = 1U;
    } else if ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
                ((uint32_T)TTG_STMLatCtrlMode_Enum)) ||
               (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC_RQ ==
                ((uint32_T)TTG_STMLatCtrlMode_Enum))) {
        /* Switch: '<S946>/Switch2' incorporates:
         *  Constant: '<S946>/Constant2'
         *  Switch: '<S946>/Switch1'
         */
        rtb_DataTypeConversion1 = 2U;
    } else {
        rtb_DataTypeConversion1 = 3U;
    }

    /* MultiPortSwitch: '<S890>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* MultiPortSwitch: '<S890>/Multiport Switch' incorporates:
             *  Abs: '<S947>/Abs'
             *  Abs: '<S947>/Abs1'
             *  Inport: '<Root>/Inport79'
             *  Lookup_n-D: '<S947>/1-D Lookup Table'
             *  Lookup_n-D: '<S947>/1-D Lookup Table1'
             *  Lookup_n-D: '<S947>/1-D Lookup Table3'
             *  Product: '<S947>/Product'
             */
            TJATVG_PlanningHorizon_sec =
                (look1_iflf_binlxpw(
                     VDy_VehVelX_mps,
                     ((const real32_T *)&(TJATVG_VehVelXLC_Bx_mps[0])),
                     ((const real32_T *)&(
                         TJATVG_PlanningHorizonValLC_Cr_sec[0])),
                     14U) *
                 look1_iflf_binlxpw(
                     fabsf(TJATTG_TgtTrajCrv_1pm),
                     ((const real32_T *)&(TJATVG_CrvPlanHorizon_Bx_1pm[0])),
                     ((const real32_T *)&(TJATVG_PlanHorizonScal_Cr_Fac[0])),
                     6U)) *
                look1_iflf_binlxpw(
                    fabsf(TJATTG_TgtTrajPosY0_met),
                    ((const real32_T *)&(TJATVG_PosYPlanHorizon_Bx_met[0])),
                    ((const real32_T *)&(TJATVG_PosYPlanHorizonScal_Cr_Fac[0])),
                    6U);
            break;

        case 1:
            /* MultiPortSwitch: '<S890>/Multiport Switch' incorporates:
             *  Abs: '<S949>/Abs'
             *  Abs: '<S949>/Abs1'
             *  Inport: '<Root>/Inport79'
             *  Lookup_n-D: '<S949>/1-D Lookup Table'
             *  Lookup_n-D: '<S949>/1-D Lookup Table1'
             *  Lookup_n-D: '<S949>/1-D Lookup Table3'
             *  Product: '<S949>/Product'
             */
            TJATVG_PlanningHorizon_sec =
                (look1_iflf_binlxpw(
                     VDy_VehVelX_mps,
                     ((const real32_T *)&(TJATVG_VehVelXOF_Bx_mps[0])),
                     ((const real32_T *)&(
                         TJATVG_PlanHorizonObjFolVal_Cr_sec[0])),
                     14U) *
                 look1_iflf_binlxpw(
                     fabsf(TJATTG_TgtTrajCrv_1pm),
                     ((const real32_T *)&(TJATVG_CrvPlanHorizon_Bx_1pm[0])),
                     ((const real32_T *)&(TJATVG_PlanHorizonScal_Cr_Fac[0])),
                     6U)) *
                look1_iflf_binlxpw(
                    fabsf(TJATTG_TgtTrajPosY0_met),
                    ((const real32_T *)&(TJATVG_PosYPlanHorizon_Bx_met[0])),
                    ((const real32_T *)&(TJATVG_PosYPlanHorizonScal_Cr_Fac[0])),
                    6U);
            break;

        case 2:
            /* MultiPortSwitch: '<S890>/Multiport Switch' incorporates:
             *  Inport: '<Root>/Inport79'
             *  Lookup_n-D: '<S948>/1-D Lookup Table'
             */
            TJATVG_PlanningHorizon_sec = look1_iflf_binlxpw(
                VDy_VehVelX_mps,
                ((const real32_T *)&(TJATVG_VehVelXLC_Bx_mps[0])),
                ((const real32_T *)&(TJATVG_PlanHorizonLChange_Vel_sec[0])),
                14U);
            break;

        default:
            /* MultiPortSwitch: '<S890>/Multiport Switch' incorporates:
             *  Constant: '<S936>/Constant'
             */
            TJATVG_PlanningHorizon_sec = 0.0F;
            break;
    }

    /* End of MultiPortSwitch: '<S890>/Multiport Switch' */

    /* SignalConversion: '<S10>/Signal Conversion26' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajHeadAng_rad = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                    .TTG_TargetTrajectory_bus.Heading_rad;

    /* SignalConversion: '<S10>/Signal Conversion28' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajCrvChng_1pm2 = TTG_TgtTrajAndCridrBndUnitDelay_bus
                                     .TTG_TargetTrajectory_bus.CrvChng_1pm2;

    /* SignalConversion: '<S10>/Signal Conversion29' incorporates:
     *  UnitDelay: '<S676>/Unit Delay1'
     */
    TJATTG_TgtTrajLength_met =
        TTG_TgtTrajAndCridrBndUnitDelay_bus.TTG_TargetTrajectory_bus.Length_met;

    /* Logic: '<S695>/AND' incorporates:
     *  Constant: '<S695>/Constant'
     *  RelationalOperator: '<S695>/Less Than'
     */
    rtb_Equal_awdj =
        ((rtb_Length_out_ltpy < TJATTG_PredResetTrajLength_C_met) &&
         TJATTG_PredictionEnable_bool);

    /* Logic: '<S729>/AND' incorporates:
     *  Constant: '<S729>/Constant'
     *  RelationalOperator: '<S729>/Less Than'
     */
    rtb_Equal_i3l2_idx_0 =
        ((rtb_Heading_rad < TJATTG_PredResetTrajLength_C_met) &&
         TJATTG_PredictionEnable_bool);

    /* Logic: '<S667>/OR' incorporates:
     *  Logic: '<S703>/AND'
     *  Logic: '<S703>/NOT'
     *  Logic: '<S737>/AND'
     *  Logic: '<S737>/NOT'
     *  UnitDelay: '<S667>/Unit Delay'
     *  UnitDelay: '<S703>/Unit Delay'
     *  UnitDelay: '<S737>/Unit Delay'
     */
    TTG_PrevLnLengResetUnitDelay_bool =
        ((rtb_Equal_i3l2_idx_0 && (!TTG_RiLnResetEdgeRising_bool)) ||
         (rtb_Equal_awdj && (!TTG_CntrLnResetEdgeRising_bool)));

    /* Logic: '<S712>/AND' incorporates:
     *  Constant: '<S712>/Constant'
     *  RelationalOperator: '<S712>/Less Than'
     *  UnitDelay: '<S720>/Unit Delay'
     */
    TTG_LeLnResetEdgeRising_bool =
        ((rtb_Switch_ithu < TJATTG_PredResetTrajLength_C_met) &&
         TJATTG_PredictionEnable_bool);

    /* Switch: '<S808>/Switch2' incorporates:
     *  Constant: '<S808>/Constant1'
     *  RelationalOperator: '<S808>/GreaterThan'
     *  Switch: '<S808>/Switch1'
     */
    if (TTG_Reset_bool) {
        /* Switch: '<S808>/Switch' incorporates:
         *  UnitDelay: '<S804>/Unit Delay'
         */
        TTG_TransitionTimeTurnOffDelayWithRst_sec =
            TTG_TransitionTimeUnitDelay_sec;
    } else {
        if (TTG_TransitionTimeTurnOffDelayWithRst_sec > 0.0F) {
            /* Switch: '<S808>/Switch' incorporates:
             *  Inport: '<Root>/Inport10'
             *  Sum: '<S808>/Subtract'
             *  Switch: '<S808>/Switch1'
             *  UnitDelay: '<S808>/Unit Delay'
             */
            TTG_TransitionTimeTurnOffDelayWithRst_sec =
                TTG_TransitionTimeTurnOffDelayWithRst_sec -
                LCFRCV_TSysCycleTimeSen_sec;
        }
    }

    /* End of Switch: '<S808>/Switch2' */

    /* SignalConversion: '<S666>/Signal Conversion6' */
    rtb_VectorConcatenate_aisn[5] = TTG_OD_Enable_bool;

    /* Switch: '<S797>/Switch1' */
    if (TTG_Enable_bool) {
        /* Switch: '<S797>/Switch' incorporates:
         *  Logic: '<S797>/NOT'
         */
        if (!TTG_Reset_bool) {
            /* Switch: '<S797>/Switch1' incorporates:
             *  UnitDelay: '<S797>/Unit Delay'
             */
            TTG_Predict_Enable_bool = TTG_PredictEnableUnitDelay_bool;
        } else {
            /* Switch: '<S797>/Switch1' */
            TTG_Predict_Enable_bool = TTG_LD_PredictFinish_bool;
        }

        /* End of Switch: '<S797>/Switch' */
    } else {
        /* Switch: '<S797>/Switch1' */
        TTG_Predict_Enable_bool = TTG_LD_PredictFinish_bool;
    }

    /* End of Switch: '<S797>/Switch1' */

    /* Logic: '<S670>/OR2' */
    TJATTG_TransTriggerReplan_bool =
        ((((TTG_Predict_Enable_bool) || (TTG_CMB_Enable_bool)) ||
          (TTG_LD_Enable_bool)) ||
         (TTG_OD_Enable_bool));

    /* SignalConversion: '<S666>/Signal Conversion13' */
    rtb_VectorConcatenate_aisn[12] = TJATTG_TransTriggerReplan_bool;

    /* RelationalOperator: '<S914>/Equal7' incorporates:
     *  Constant: '<S916>/Constant'
     */
    rtb_Equal_i3l2_idx_1 = (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                            E_TJASLC_ManeuverState_nu_LATMVSTART);

    /* Switch: '<S918>/Switch' incorporates:
     *  Constant: '<S917>/Constant'
     *  Constant: '<S918>/Constant2'
     *  Logic: '<S914>/NOT'
     *  Logic: '<S915>/AND'
     *  Logic: '<S915>/NOT'
     *  RelationalOperator: '<S914>/Equal5'
     *  UnitDelay: '<S915>/Unit Delay'
     *  UnitDelay: '<S918>/Unit Delay'
     */
    if (((uint32_T)TJASTM_LatCtrlMode_nu) !=
        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) {
        TVG_LatMovStartRSFF_bool = false;
    } else {
        TVG_LatMovStartRSFF_bool =
            ((rtb_Equal_i3l2_idx_1 && (!TVG_LatMovStartEdgeRising_bool)) ||
             (TVG_LatMovStartRSFF_bool));
    }

    /* End of Switch: '<S918>/Switch' */

    /* Logic: '<S913>/AND' incorporates:
     *  Constant: '<S913>/Constant'
     *  Inport: '<Root>/Inport79'
     *  Inport: '<Root>/Inport90'
     *  RelationalOperator: '<S913>/Less Than'
     */
    rtb_SLC_LeTurnSignalOn_bool =
        (LCFRCV_VehStopped_nu && (VDy_VehVelX_mps < 5.0F));

    /* Switch: '<S888>/Switch' incorporates:
     *  Constant: '<S907>/Constant'
     *  Constant: '<S908>/Constant'
     *  Logic: '<S888>/AND'
     *  Logic: '<S888>/OR1'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S888>/Equal5'
     *  RelationalOperator: '<S888>/Equal6'
     *  Switch: '<S552>/Switch3'
     */
    if ((E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT ==
         ((uint32_T)TJASTM_SysStateTJA_nu)) ||
        ((((uint32_T)TJASTM_SysStateTJA_nu) ==
          E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) &&
         rtb_SLC_LeTurnSignalOn_bool)) {
        /* Switch: '<S888>/Switch' incorporates:
         *  Constant: '<S905>/Constant'
         */
        TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_FREEZE;
    } else {
        /* Logic: '<S888>/AND1' incorporates:
         *  Constant: '<S894>/Constant'
         *  Logic: '<S888>/NOT'
         *  RelationalOperator: '<S888>/Equal1'
         */
        rtb_SLC_LeTurnSignalOn_bool =
            ((!rtb_SLC_LeTurnSignalOn_bool) &&
             (E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
              ((uint32_T)TJASTM_SysStateTJA_nu)));

        /* RelationalOperator: '<S462>/Equal7' incorporates:
         *  RelationalOperator: '<S888>/Equal2'
         *  RelationalOperator: '<S888>/Equal8'
         *  UnitDelay: '<S888>/Unit Delay1'
         */
        SLC_PrevManeuverState_Enum = SLC_PrevManeuverStateSLC2LCC_Enum;

        /* Switch: '<S888>/Switch3' incorporates:
         *  Constant: '<S888>/Constant'
         *  Constant: '<S899>/Constant'
         *  Constant: '<S900>/Constant'
         *  Constant: '<S901>/Constant'
         *  Constant: '<S902>/Constant'
         *  Constant: '<S903>/Constant'
         *  Constant: '<S904>/Constant'
         *  Logic: '<S888>/AND2'
         *  Logic: '<S888>/AND3'
         *  Logic: '<S888>/AND5'
         *  Logic: '<S888>/AND6'
         *  Logic: '<S888>/AND7'
         *  Logic: '<S888>/AND8'
         *  Logic: '<S888>/AND9'
         *  Logic: '<S888>/OR3'
         *  RelationalOperator: '<S888>/Equal10'
         *  RelationalOperator: '<S888>/Equal11'
         *  RelationalOperator: '<S888>/Equal12'
         *  RelationalOperator: '<S888>/Equal2'
         *  RelationalOperator: '<S888>/Equal8'
         *  RelationalOperator: '<S888>/Equal9'
         *  Switch: '<S888>/Switch1'
         *  Switch: '<S888>/Switch2'
         *  Switch: '<S888>/Switch4'
         *  UnitDelay: '<S888>/Unit Delay2'
         *  UnitDelay: '<S918>/Unit Delay'
         */
        if (rtb_SLC_LeTurnSignalOn_bool &&
            (((((uint32_T)SLC_PrevManeuverState_Enum) ==
               E_TJASLC_ManeuverState_nu_LATMVSTART) ||
              (((uint32_T)SLC_PrevManeuverState_Enum) ==
               E_TJASLC_ManeuverState_nu_LCMSTART)) &&
             ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
               E_TJASLC_ManeuverState_nu_ABORT) ||
              (((uint32_T)SLC_PrevManeuverState2_Enum) ==
               E_TJASLC_ManeuverState_nu_PASSIVE)))) {
            /* Switch: '<S888>/Switch' incorporates:
             *  Constant: '<S896>/Constant'
             */
            TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_SLCQ2LCC;
        } else if (rtb_SLC_LeTurnSignalOn_bool &&
                   ((TVG_LatMovStartRSFF_bool) ||
                    (TJATTG_TransTriggerReplan_bool &&
                     (TJATVG_ModeTransTrigReplan_bool)))) {
            /* Switch: '<S888>/Switch1' incorporates:
             *  Constant: '<S906>/Constant'
             *  Switch: '<S888>/Switch'
             */
            TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_REFCHNG;
        } else if ((E_TJASLC_AbortState_nu_ABORT_NOACTIVE ==
                    ((uint32_T)SLC_PreAbortState_enum)) &&
                   (((uint32_T)TVG_AbortStateUnitDy_Enum) ==
                    E_TJASLC_AbortState_nu_ABORT_NEWEGO)) {
            /* Switch: '<S888>/Switch4' incorporates:
             *  Constant: '<S898>/Constant'
             *  Switch: '<S888>/Switch'
             *  Switch: '<S888>/Switch1'
             */
            TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_LANECHANG;
        } else if (rtb_SLC_LeTurnSignalOn_bool) {
            /* Switch: '<S888>/Switch2' incorporates:
             *  Constant: '<S909>/Constant'
             *  Switch: '<S888>/Switch'
             *  Switch: '<S888>/Switch1'
             *  Switch: '<S888>/Switch4'
             */
            TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ;
        } else {
            /* Switch: '<S888>/Switch' incorporates:
             *  Constant: '<S910>/Constant'
             *  Switch: '<S888>/Switch1'
             *  Switch: '<S888>/Switch2'
             *  Switch: '<S888>/Switch4'
             */
            TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_OFF;
        }
    }

    /* SignalConversion: '<S666>/Signal Conversion9' */
    rtb_VectorConcatenate_aisn[8] = TTG_Predict_Enable_bool;

    /* SignalConversion: '<S666>/Signal Conversion7' */
    rtb_VectorConcatenate_aisn[6] = TTG_LD_Enable_bool;

    /* SignalConversion: '<S666>/Signal Conversion8' */
    rtb_VectorConcatenate_aisn[7] = TTG_CMB_Enable_bool;

    /* SignalConversion: '<S666>/Signal Conversion5' */
    rtb_VectorConcatenate_aisn[4] = TTG_Enable_bool;

    /* SignalConversion: '<S666>/Signal Conversion15' */
    rtb_VectorConcatenate_aisn[14] = rtb_OR_imjz;

    /* SignalConversion: '<S666>/Signal Conversion2' */
    rtb_VectorConcatenate_aisn[2] = TTG_LD_PredictFinish_bool;

    /* SignalConversion: '<S666>/Signal Conversion16' */
    rtb_VectorConcatenate_aisn[15] = rtb_Equal_blkb;

    /* SignalConversion: '<S666>/Signal Conversion1' */
    rtb_VectorConcatenate_aisn[1] = TTG_ObjectUpdate_bool;

    /* SignalConversion: '<S666>/Signal Conversion4' */
    rtb_VectorConcatenate_aisn[0] = TTG_LaneUpdate_bool;

    /* Switch: '<S739>/Switch1' incorporates:
     *  Switch: '<S738>/Switch1'
     *  Switch: '<S738>/Switch2'
     *  Switch: '<S739>/Switch2'
     *  Switch: '<S740>/Switch1'
     *  Switch: '<S740>/Switch2'
     *  UnaryMinus: '<S685>/Unary Minus1'
     *  UnitDelay: '<S738>/Unit Delay1'
     *  UnitDelay: '<S739>/Unit Delay1'
     *  UnitDelay: '<S740>/Unit Delay1'
     */
    if (rtb_OR_bsb5) {
        TTG_OdoPosYDelayRe_met = rtb_Abs_ob4k;
        TTG_OdoPosXDelayRe_met = rtb_Abs_hlug;
        TTG_OdoYawDelayRe_rad = -rtb_Subtract_if0w_idx_1;
    } else {
        if (TJATTG_PredictionEnable_bool) {
            /* UnitDelay: '<S739>/Unit Delay1' incorporates:
             *  Sum: '<S685>/Add1'
             *  Switch: '<S739>/Switch2'
             */
            TTG_OdoPosYDelayRe_met = rtb_y_fcnl + rtb_Abs_ob4k;

            /* UnitDelay: '<S740>/Unit Delay1' incorporates:
             *  Sum: '<S685>/Add'
             *  Switch: '<S740>/Switch2'
             */
            TTG_OdoPosXDelayRe_met = rtb_uDLookupTable1_eoam + rtb_Abs_hlug;

            /* UnitDelay: '<S738>/Unit Delay1' incorporates:
             *  Sum: '<S685>/Subtract'
             *  Switch: '<S738>/Switch2'
             */
            TTG_OdoYawDelayRe_rad = rtb_Add_fbcu - rtb_Subtract_if0w_idx_1;
        }
    }

    /* End of Switch: '<S739>/Switch1' */

    /* Logic: '<S963>/AND1' incorporates:
     *  Constant: '<S963>/Constant1'
     */
    rtb_SLC_LeTurnSignalOn_bool =
        (TJATTG_PredictionEnable_bool && (TJALKA_RampoutPredictOn_C_bool));

    /* RelationalOperator: '<S893>/Equal1' incorporates:
     *  Constant: '<S959>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND2_emk5 = (((uint32_T)TJASTM_SysStateTJA_nu) ==
                     E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING);

    /* Switch: '<S962>/Switch' incorporates:
     *  Constant: '<S962>/Constant5'
     *  Constant: '<S962>/Constant6'
     */
    if (rtb_AND2_emk5) {
        rtb_Subtract_if0w_idx_1 = TJATVG_StrWhStifLimit_C_nu;
    } else {
        rtb_Subtract_if0w_idx_1 = 0.0F;
    }

    /* End of Switch: '<S962>/Switch' */

    /* Switch: '<S963>/Switch' incorporates:
     *  Constant: '<S963>/Constant3'
     *  Constant: '<S963>/Constant5'
     */
    if (rtb_SLC_LeTurnSignalOn_bool) {
        rtb_Add_fbcu = TJATVG_StrWhStifLimitPredct_C_fac;
    } else {
        rtb_Add_fbcu = 1.0F;
    }

    /* End of Switch: '<S963>/Switch' */

    /* Product: '<S962>/Product' incorporates:
     *  Inport: '<Root>/Inport79'
     *  Lookup_n-D: '<S893>/1-D Lookup Table'
     *  Product: '<S893>/Product'
     */
    TJATVG_StrWhStifLimit_nu =
        rtb_Subtract_if0w_idx_1 *
        (look1_iflf_binlxpw(
             VDy_VehVelX_mps,
             ((const real32_T *)&(TJATVG_VehVelX_RedFact_Bx_mps[0])),
             ((const real32_T *)&(TJATVG_RedFact_Vel_Cr_fac[0])), 5U) *
         rtb_Add_fbcu);

    /* Switch: '<S964>/Switch' incorporates:
     *  Constant: '<S960>/Constant'
     *  Constant: '<S964>/Constant2'
     *  Constant: '<S964>/Constant5'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S893>/Equal3'
     *  Switch: '<S552>/Switch3'
     *  Switch: '<S964>/Switch1'
     */
    if (rtb_AND2_emk5) {
        rtb_DataTypeConversion1 = 0U;
    } else if (((uint32_T)TJASTM_SysStateTJA_nu) ==
               E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT) {
        /* Switch: '<S964>/Switch1' incorporates:
         *  Constant: '<S964>/Constant1'
         */
        rtb_DataTypeConversion1 = 1U;
    } else {
        rtb_DataTypeConversion1 = 2U;
    }

    /* End of Switch: '<S964>/Switch' */

    /* MultiPortSwitch: '<S961>/Multiport Switch2' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* MultiPortSwitch: '<S961>/Multiport Switch2' incorporates:
             *  Constant: '<S961>/Constant11'
             */
            TJATVG_TrqRampGrad_1ps = TJATVG_TrqRampInGrad_C_1ps;
            break;

        case 1:
            /* Switch: '<S961>/Switch1' */
            if (TJAGEN_Abort_bool) {
                /* MultiPortSwitch: '<S961>/Multiport Switch2' incorporates:
                 *  Constant: '<S961>/Constant3'
                 */
                TJATVG_TrqRampGrad_1ps = TJATVG_TrqAbortGrad_C_1ps;
            } else {
                /* MultiPortSwitch: '<S961>/Multiport Switch2' incorporates:
                 *  Constant: '<S961>/Constant2'
                 */
                TJATVG_TrqRampGrad_1ps = TJATVG_TrqRampOutGrad_C_1ps;
            }

            /* End of Switch: '<S961>/Switch1' */
            break;

        default:
            /* MultiPortSwitch: '<S961>/Multiport Switch2' incorporates:
             *  Constant: '<S961>/Constant8'
             */
            TJATVG_TrqRampGrad_1ps = 0.0F;
            break;
    }

    /* End of MultiPortSwitch: '<S961>/Multiport Switch2' */

    /* Logic: '<S963>/AND' incorporates:
     *  Logic: '<S963>/NOT'
     *  UnitDelay: '<S963>/Unit Delay'
     */
    rtb_Equal_blkb = ((TVG_PredictionEnableUnitDelay_bool) &&
                      (!rtb_SLC_LeTurnSignalOn_bool));

    /* Switch: '<S965>/Switch' incorporates:
     *  Constant: '<S963>/Constant2'
     *  Constant: '<S963>/Constant4'
     *  MinMax: '<S965>/Max'
     *  Sum: '<S965>/Subtract'
     *  Switch: '<S965>/Switch1'
     *  UnitDelay: '<S965>/Unit Delay'
     */
    if (rtb_Equal_blkb) {
        TVG_PredictionEnableTurnOffDelay_sec = TJATTG_TransDurationPredct_C_sec;
    } else {
        TVG_PredictionEnableTurnOffDelay_sec =
            fmaxf(-0.06F, TVG_PredictionEnableTurnOffDelay_sec) - 0.06F;
    }

    /* End of Switch: '<S965>/Switch' */

    /* MultiPortSwitch: '<S961>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Logic: '<S965>/OR' incorporates:
             *  RelationalOperator: '<S965>/GreaterThan'
             *  UnitDelay: '<S965>/Unit Delay'
             */
            rtb_Equal_blkb = (rtb_Equal_blkb ||
                              (TVG_PredictionEnableTurnOffDelay_sec > -0.06F));

            /* Switch: '<S961>/Switch2' incorporates:
             *  Logic: '<S963>/OR'
             */
            if (rtb_SLC_LeTurnSignalOn_bool || rtb_Equal_blkb) {
                /* Switch: '<S963>/Switch1' */
                if (rtb_Equal_blkb) {
                    /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
                     *  Constant: '<S963>/Constant6'
                     */
                    TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifRampInGrd_C_1ps;
                } else {
                    /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
                     *  Constant: '<S963>/Constant7'
                     */
                    TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifRampOutGrd_C_1ps;
                }

                /* End of Switch: '<S963>/Switch1' */
            } else {
                /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
                 *  Constant: '<S961>/Constant4'
                 */
                TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifRampInGrd_C_1ps;
            }

            /* End of Switch: '<S961>/Switch2' */

            /* MultiPortSwitch: '<S961>/Multiport Switch1' incorporates:
             *  Constant: '<S961>/Constant9'
             */
            TJATVG_MaxTrqScalGrad_1ps = TJATVG_MaxTrqScalRampInGrd_C_1ps;
            break;

        case 1:
            /* Switch: '<S961>/Switch' */
            if (TJAGEN_Abort_bool) {
                /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
                 *  Constant: '<S961>/Constant6'
                 */
                TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifAbortGrd_C_1ps;
            } else {
                /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
                 *  Constant: '<S961>/Constant1'
                 */
                TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifRampOutGrd_C_1ps;
            }

            /* End of Switch: '<S961>/Switch' */

            /* MultiPortSwitch: '<S961>/Multiport Switch1' incorporates:
             *  Constant: '<S961>/Constant10'
             */
            TJATVG_MaxTrqScalGrad_1ps = TJATVG_MaxTrqScalRampOutGrd_C_1ps;
            break;

        default:
            /* MultiPortSwitch: '<S961>/Multiport Switch' incorporates:
             *  Constant: '<S961>/Constant5'
             */
            TJATVG_StrWhStifGrad_1ps = 0.0F;

            /* MultiPortSwitch: '<S961>/Multiport Switch1' incorporates:
             *  Constant: '<S961>/Constant7'
             */
            TJATVG_MaxTrqScalGrad_1ps = 0.0F;
            break;
    }

    /* End of MultiPortSwitch: '<S961>/Multiport Switch' */

    /* SignalConversion: '<S666>/Signal Conversion12' */
    rtb_VectorConcatenate_aisn[11] = rtb_SLC_LeAdjLaneValid_bool;

    /* SignalConversion: '<S666>/Signal Conversion11' */
    rtb_VectorConcatenate_aisn[10] = rtb_TJASTM_ACCIsOFF_bool;

    /* SignalConversion: '<S666>/Signal Conversion10' */
    rtb_VectorConcatenate_aisn[9] = rtb_SLC_RiTurnSignalOn_bool;

    /* RelationalOperator: '<S378>/Equal3' incorporates:
     *  Constant: '<S439>/Constant'
     */
    rtb_AND2_emk5 = (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                     E_TJASLC_ManeuverState_nu_PASSIVE);

    /* RelationalOperator: '<S378>/Equal' incorporates:
     *  Constant: '<S434>/Constant'
     */
    rtb_OR_bsb5 = (E_TJASLC_ManeuverState_nu_NEWEGO ==
                   ((uint32_T)SLC_PrevManeuverState2_Enum));

    /* Switch: '<S448>/Switch' incorporates:
     *  Constant: '<S378>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S448>/Max'
     *  Sum: '<S448>/Subtract'
     *  Switch: '<S448>/Switch1'
     *  UnaryMinus: '<S448>/Unary Minus'
     *  UnitDelay: '<S448>/Unit Delay'
     */
    if (rtb_OR_bsb5) {
        SLC_ManeuverStateTurnOnDelay_sec =
            fmaxf(SLC_ManeuverStateTurnOnDelay_sec,
                  -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_ManeuverStateTurnOnDelay_sec = 2.0F;
    }

    /* End of Switch: '<S448>/Switch' */

    /* Logic: '<S448>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S448>/LessThanOrEqual'
     *  UnaryMinus: '<S448>/Unary Minus1'
     *  UnitDelay: '<S448>/Unit Delay'
     */
    rtb_SLC_RiTurnSignalOn_bool =
        (rtb_OR_bsb5 &&
         (SLC_ManeuverStateTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S378>/OR8' incorporates:
     *  Constant: '<S437>/Constant'
     *  Constant: '<S438>/Constant'
     *  Constant: '<S442>/Constant'
     *  Logic: '<S378>/OR5'
     *  Logic: '<S378>/OR6'
     *  Logic: '<S378>/OR7'
     *  RelationalOperator: '<S378>/Equal17'
     *  RelationalOperator: '<S378>/Equal18'
     *  RelationalOperator: '<S378>/Equal6'
     */
    rtb_OR_bsb5 =
        (((rtb_AND2_emk5 && (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                             E_TJASLC_ManeuverState_nu_NEWEGO)) &&
          (!rtb_SLC_RiTurnSignalOn_bool)) ||
         ((((uint32_T)SLC_PreAbortState_enum) ==
           E_TJASLC_AbortState_nu_ABORT_ABORT) ||
          (((uint32_T)SLC_PreAbortState_enum) ==
           E_TJASLC_AbortState_nu_ABORT_NEWEGO)));

    /* Switch: '<S447>/Switch' incorporates:
     *  Constant: '<S378>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S447>/Max'
     *  Sum: '<S447>/Subtract'
     *  Switch: '<S447>/Switch1'
     *  UnaryMinus: '<S447>/Unary Minus'
     *  UnitDelay: '<S447>/Unit Delay'
     */
    if (rtb_OR_bsb5) {
        SLC_TakeoverAbortTurnOffDelay_sec = TJASLC_TakeoverTiMaxAbrt_C_sec;
    } else {
        SLC_TakeoverAbortTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_TakeoverAbortTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S447>/Switch' */

    /* Logic: '<S378>/OR' incorporates:
     *  Constant: '<S435>/Constant'
     *  Constant: '<S436>/Constant'
     *  Constant: '<S440>/Constant'
     *  Constant: '<S441>/Constant'
     *  Logic: '<S378>/OR1'
     *  Logic: '<S378>/OR2'
     *  RelationalOperator: '<S378>/Equal1'
     *  RelationalOperator: '<S378>/Equal2'
     *  RelationalOperator: '<S378>/Equal4'
     *  RelationalOperator: '<S378>/Equal5'
     */
    rtb_TJASTM_ACCIsOFF_bool =
        (((rtb_SLC_RiTurnSignalOn_bool ||
           (E_TJASLC_ManeuverState_nu_LCMEND ==
            ((uint32_T)SLC_PrevManeuverState2_Enum))) ||
          (((uint32_T)SLC_PrevManeuverState2_Enum) ==
           E_TJASLC_ManeuverState_nu_ABORT)) ||
         (rtb_AND2_emk5 && ((((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                             E_TJASLC_ManeuverState_nu_LATMVSTART) ||
                            (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                             E_TJASLC_ManeuverState_nu_LCMSTART))));

    /* Logic: '<S433>/AND' incorporates:
     *  Logic: '<S433>/NOT'
     *  UnitDelay: '<S433>/Unit Delay'
     */
    rtb_AND2_emk5 =
        (rtb_TJASTM_ACCIsOFF_bool && (!SLC_TakeoverEdgeRising_bool));

    /* Switch: '<S446>/Switch' incorporates:
     *  Constant: '<S378>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S446>/Max'
     *  Sum: '<S446>/Subtract'
     *  Switch: '<S446>/Switch1'
     *  UnaryMinus: '<S446>/Unary Minus'
     *  UnitDelay: '<S446>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        SLC_TakeoverValidTurnOffDelay_sec = TJASLC_TakeoverTimeMax_C_sec;
    } else {
        SLC_TakeoverValidTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_TakeoverValidTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S446>/Switch' */

    /* Switch: '<S445>/Switch' incorporates:
     *  Constant: '<S443>/Constant'
     *  Constant: '<S444>/Constant'
     *  Constant: '<S445>/Constant2'
     *  RelationalOperator: '<S378>/Equal7'
     *  RelationalOperator: '<S378>/Equal9'
     *  UnitDelay: '<S445>/Unit Delay'
     */
    if (((uint32_T)STM_SysStateUnitDelay_bool) !=
        E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) {
        SLC_TakeOverRSFF2_bool = false;
    } else {
        SLC_TakeOverRSFF2_bool = ((E_TJASLC_ManeuverState_nu_LATMVSTART ==
                                   ((uint32_T)SLC_PrevManeuverState2_Enum)) ||
                                  (SLC_TakeOverRSFF2_bool));
    }

    /* Logic: '<S378>/OR4' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S378>/OR3'
     *  Logic: '<S446>/OR'
     *  Logic: '<S447>/OR'
     *  RelationalOperator: '<S446>/GreaterThan'
     *  RelationalOperator: '<S447>/GreaterThan'
     *  UnaryMinus: '<S446>/Unary Minus1'
     *  UnaryMinus: '<S447>/Unary Minus1'
     *  UnitDelay: '<S445>/Unit Delay'
     *  UnitDelay: '<S446>/Unit Delay'
     *  UnitDelay: '<S447>/Unit Delay'
     */
    TJASLC_TakeOverValid_bool =
        (((rtb_AND2_emk5 || (SLC_TakeoverValidTurnOffDelay_sec >
                             (-LCFRCV_TSysCycleTimeSen_sec))) ||
          (rtb_OR_bsb5 || (SLC_TakeoverAbortTurnOffDelay_sec >
                           (-LCFRCV_TSysCycleTimeSen_sec)))) &&
         (SLC_TakeOverRSFF2_bool));

    /* Logic: '<S379>/OR8' incorporates:
     *  Logic: '<S382>/OR8'
     */
    rtb_SLC_LeAdjLaneValid_bool =
        ((SLC_LeverRightEngaged_bool) || (SLC_LeverLeftEngaged_bool));

    /* Logic: '<S379>/OR4' incorporates:
     *  Constant: '<S385>/Constant'
     *  Constant: '<S386>/Constant'
     *  Constant: '<S387>/Constant'
     *  Constant: '<S388>/Constant'
     *  Constant: '<S389>/Constant'
     *  Constant: '<S390>/Constant'
     *  Constant: '<S391>/Constant'
     *  Constant: '<S392>/Constant'
     *  Constant: '<S393>/Constant'
     *  Logic: '<S379>/OR1'
     *  Logic: '<S379>/OR2'
     *  Logic: '<S379>/OR3'
     *  Logic: '<S379>/OR5'
     *  Logic: '<S379>/OR6'
     *  Logic: '<S379>/OR7'
     *  Logic: '<S379>/OR8'
     *  Logic: '<S379>/OR9'
     *  Logic: '<S384>/AND'
     *  Logic: '<S384>/NOT'
     *  RelationalOperator: '<S379>/Equal1'
     *  RelationalOperator: '<S379>/Equal2'
     *  RelationalOperator: '<S379>/Equal3'
     *  RelationalOperator: '<S379>/Equal4'
     *  RelationalOperator: '<S379>/Equal5'
     *  RelationalOperator: '<S379>/Equal6'
     *  RelationalOperator: '<S379>/Equal7'
     *  RelationalOperator: '<S379>/Equal8'
     *  RelationalOperator: '<S379>/Equal9'
     *  UnitDelay: '<S379>/Unit Delay'
     *  UnitDelay: '<S384>/Unit Delay'
     */
    rtb_AND2_emk5 =
        (((((E_TJASLC_ManeuverState_nu_PASSIVE ==
             ((uint32_T)SLC_PrevManeuverState2_Enum)) &&
            ((((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
              E_TJASLC_ManeuverState_nu_NEWEGO) &&
             (!SLC_PrebNewEgoOverTime_nu))) ||
           (E_TJASLC_ManeuverState_nu_ABORT ==
            ((uint32_T)SLC_PrevManeuverState2_Enum))) ||
          ((E_TJASLC_ManeuverState_nu_PASSIVE ==
            ((uint32_T)SLC_PrevManeuverState2_Enum)) &&
           (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
            E_TJASLC_ManeuverState_nu_LCPSTART))) ||
         ((((((((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
               E_TJASLC_ManeuverState_nu_PASSIVE) ||
              (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
               E_TJASLC_ManeuverState_nu_TRIGREADY)) ||
             (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
              E_TJASLC_ManeuverState_nu_LCPSTART)) &&
            ((!rtb_SLC_LeAdjLaneValid_bool) && (SLC_TurnSignalOffEF_bool))) &&
           (((uint32_T)STM_SysStateUnitDelay_bool) ==
            E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING)) &&
          (!TJASLC_TakeOverValid_bool)));

    /* Switch: '<S394>/Switch' incorporates:
     *  Constant: '<S379>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S394>/Max'
     *  Sum: '<S394>/Subtract'
     *  Switch: '<S394>/Switch1'
     *  UnaryMinus: '<S394>/Unary Minus'
     *  UnitDelay: '<S394>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        SLC_LaneChangCancleTimeDelay_sec = TJASLC_LaneChangInfoTime_sec;
    } else {
        SLC_LaneChangCancleTimeDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  SLC_LaneChangCancleTimeDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S394>/Switch' */

    /* Logic: '<S394>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S394>/GreaterThan'
     *  UnaryMinus: '<S394>/Unary Minus1'
     *  UnitDelay: '<S394>/Unit Delay'
     */
    SLC_LaneChangeCancleInfo =
        (rtb_AND2_emk5 ||
         (SLC_LaneChangCancleTimeDelay_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S382>/OR4' incorporates:
     *  Constant: '<S402>/Constant'
     *  Constant: '<S403>/Constant'
     *  Constant: '<S404>/Constant'
     *  Constant: '<S405>/Constant'
     *  Logic: '<S382>/OR1'
     *  RelationalOperator: '<S382>/Equal1'
     *  RelationalOperator: '<S382>/Equal2'
     *  RelationalOperator: '<S382>/Equal7'
     *  RelationalOperator: '<S382>/Equal9'
     */
    SLC_LaneChangePendingInfo = (((((E_TJASLC_ManeuverState_nu_PASSIVE ==
                                     ((uint32_T)SLC_PrevManeuverState2_Enum)) ||
                                    (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                                     E_TJASLC_ManeuverState_nu_LCPSTART)) ||
                                   (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                                    E_TJASLC_ManeuverState_nu_TRIGREADY)) &&
                                  rtb_SLC_LeAdjLaneValid_bool) &&
                                 (((uint32_T)STM_SysStateUnitDelay_bool) ==
                                  E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING));

    /* Logic: '<S383>/OR2' incorporates:
     *  Constant: '<S406>/Constant'
     *  Constant: '<S407>/Constant'
     *  Constant: '<S408>/Constant'
     *  Logic: '<S383>/OR1'
     *  RelationalOperator: '<S383>/Equal1'
     *  RelationalOperator: '<S383>/Equal2'
     *  RelationalOperator: '<S383>/Equal7'
     */
    SLC_VehSpdTooLowInfo = ((((E_TJASLC_ManeuverState_nu_PASSIVE ==
                               ((uint32_T)SLC_PrevManeuverState2_Enum)) ||
                              (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                               E_TJASLC_ManeuverState_nu_LCPSTART)) &&
                             (((uint32_T)STM_SysStateUnitDelay_bool) ==
                              E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING)) &&
                            rtb_TVG_Rampout_bool);

    /* Logic: '<S380>/OR1' incorporates:
     *  Constant: '<S395>/Constant'
     *  Constant: '<S396>/Constant'
     *  Logic: '<S380>/OR2'
     *  RelationalOperator: '<S380>/Equal1'
     *  RelationalOperator: '<S380>/Equal7'
     */
    rtb_AND2_emk5 = ((E_TJASLC_ManeuverState_nu_LCMEND ==
                      ((uint32_T)SLC_PrevManeuverState2_Enum)) ||
                     ((E_TJASLC_ManeuverState_nu_NEWEGO ==
                       ((uint32_T)SLC_PrevManeuverState2_Enum)) &&
                      rtb_SLC_RiTurnSignalOn_bool));

    /* Switch: '<S397>/Switch' incorporates:
     *  Constant: '<S380>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S397>/Max'
     *  Sum: '<S397>/Subtract'
     *  Switch: '<S397>/Switch1'
     *  UnaryMinus: '<S397>/Unary Minus'
     *  UnitDelay: '<S397>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        SLC_LaneChangEndTimeDelay_sec = TJASLC_LaneChangInfoTime_sec;
    } else {
        SLC_LaneChangEndTimeDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, SLC_LaneChangEndTimeDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S397>/Switch' */

    /* Logic: '<S397>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S397>/GreaterThan'
     *  UnaryMinus: '<S397>/Unary Minus1'
     *  UnitDelay: '<S397>/Unit Delay'
     */
    SLC_LaneChangeEndInfo = (rtb_AND2_emk5 || (SLC_LaneChangEndTimeDelay_sec >
                                               (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S381>/OR1' incorporates:
     *  Constant: '<S398>/Constant'
     *  Constant: '<S399>/Constant'
     *  Constant: '<S400>/Constant'
     *  Constant: '<S401>/Constant'
     *  RelationalOperator: '<S381>/Equal1'
     *  RelationalOperator: '<S381>/Equal2'
     *  RelationalOperator: '<S381>/Equal3'
     *  RelationalOperator: '<S381>/Equal7'
     */
    SLC_LaneChangeOnGoingInfo = ((((E_TJASLC_ManeuverState_nu_LATMVSTART ==
                                    ((uint32_T)SLC_PrevManeuverState2_Enum)) ||
                                   (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                                    E_TJASLC_ManeuverState_nu_LCMSTART)) ||
                                  (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                                   E_TJASLC_ManeuverState_nu_NEWEGO)) ||
                                 (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                                  E_TJASLC_ManeuverState_nu_LCMEND));

    /* Switch: '<S374>/Switch1' incorporates:
     *  Switch: '<S374>/Switch2'
     *  Switch: '<S374>/Switch3'
     *  Switch: '<S374>/Switch4'
     */
    if (SLC_LaneChangeOnGoingInfo) {
        /* Switch: '<S374>/Switch1' incorporates:
         *  Constant: '<S374>/Constant1'
         */
        TJASLC_LaneChangeInfo = 6U;
    } else if (SLC_LaneChangeEndInfo) {
        /* Switch: '<S374>/Switch2' incorporates:
         *  Constant: '<S374>/Constant3'
         *  Switch: '<S374>/Switch1'
         */
        TJASLC_LaneChangeInfo = 3U;
    } else if (SLC_LaneChangeCancleInfo) {
        /* Switch: '<S374>/Switch3' incorporates:
         *  Constant: '<S374>/Constant4'
         *  Switch: '<S374>/Switch1'
         *  Switch: '<S374>/Switch2'
         */
        TJASLC_LaneChangeInfo = 4U;
    } else if (SLC_LaneChangePendingInfo) {
        /* Switch: '<S374>/Switch4' incorporates:
         *  Constant: '<S374>/Constant5'
         *  Switch: '<S374>/Switch1'
         *  Switch: '<S374>/Switch2'
         *  Switch: '<S374>/Switch3'
         */
        TJASLC_LaneChangeInfo = 5U;
    } else {
        /* Switch: '<S374>/Switch1' incorporates:
         *  Switch: '<S374>/Switch'
         *  Switch: '<S374>/Switch2'
         *  Switch: '<S374>/Switch3'
         *  Switch: '<S374>/Switch4'
         */
        TJASLC_LaneChangeInfo =
            SLC_VehSpdTooLowInfo ? ((uint8_T)1) : ((uint8_T)0);
    }

    /* End of Switch: '<S374>/Switch1' */

    /* SignalConversion: '<S666>/Signal Conversion14' */
    rtb_VectorConcatenate_aisn[13] = TTG_CMBObjectCorridor_bool;

    /* Switch: '<S891>/Switch' incorporates:
     *  Constant: '<S951>/Constant'
     *  Constant: '<S952>/Constant'
     *  Logic: '<S950>/OR2'
     *  RelationalOperator: '<S950>/Equal1'
     *  RelationalOperator: '<S950>/Equal2'
     */
    if ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ==
         ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
        (((uint32_T)TJASTM_LatCtrlMode_nu) ==
         E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC_RQ)) {
        /* Switch: '<S891>/Switch' incorporates:
         *  Constant: '<S891>/Constant19'
         */
        TJATVG_TrajPlanServQu_nu = TJATVG_TrajPlanValSrvQuSALC_C_nu;
    } else {
        /* Switch: '<S891>/Switch' incorporates:
         *  Constant: '<S891>/Constant20'
         */
        TJATVG_TrajPlanServQu_nu = TJATVG_TrajPlanValServQu_C_nu;
    }

    /* End of Switch: '<S891>/Switch' */

    /* Product: '<S957>/Divide' incorporates:
     *  Constant: '<S957>/Constant5'
     *  Constant: '<S957>/Constant6'
     *  Constant: '<S957>/Constant7'
     *  Product: '<S957>/Product'
     */
    rtb_Subtract_if0w_idx_1 = (0.0174532924F * TJATVG_MaxSteeringAngle_C_deg) /
                              TJAPARAM_VEH_Wheelbase_C_met;

    /* Switch generated from: '<S892>/Switch' incorporates:
     *  Constant: '<S953>/Constant'
     *  Constant: '<S954>/Constant'
     *  Logic: '<S892>/OR2'
     *  RelationalOperator: '<S892>/Equal1'
     *  RelationalOperator: '<S892>/Equal2'
     */
    if ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
         ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
        (((uint32_T)TJASTM_LatCtrlMode_nu) ==
         E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB)) {
        /* Switch generated from: '<S957>/Switch' incorporates:
         *  Constant: '<S957>/Constant'
         */
        if (TJATVG_SetMaxCrvAndGrdLims_C_bool) {
            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant1'
             */
            TJATVG_MaxCrvTrajGuiCtl_1pm = TJATVG_LimiterMaxCrv_C_1pm;

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant2'
             */
            TJATVG_MaxCrvGrdBuildup_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant3'
             */
            TJATVG_MaxCrvGrdRed_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant4'
             */
            TJATVG_MaxCrvGrdTGC_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;
        } else {
            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant10'
             *  Product: '<S957>/Product3'
             */
            TJATVG_MaxCrvTrajGuiCtl_1pm =
                TJATVG_MinFactorCrvGrd_C_fac * rtb_Subtract_if0w_idx_1;

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant10'
             *  Constant: '<S957>/Constant8'
             *  Product: '<S957>/Product1'
             *  Product: '<S957>/Product4'
             */
            TJATVG_MaxCrvGrdBuildup_1pms =
                TJATVG_MinFactorCrvGrd_C_fac *
                (rtb_Subtract_if0w_idx_1 * TJATVG_FactorCrvGrdBuildUp_C_fac);

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant10'
             *  Constant: '<S957>/Constant9'
             *  Product: '<S957>/Product2'
             *  Product: '<S957>/Product5'
             */
            TJATVG_MaxCrvGrdRed_1pms =
                (rtb_Subtract_if0w_idx_1 * TJATVG_FactorCrvGrdRed_C_fac) *
                TJATVG_MinFactorCrvGrd_C_fac;

            /* Switch generated from: '<S892>/Switch' incorporates:
             *  Constant: '<S957>/Constant11'
             */
            TJATVG_MaxCrvGrdTGC_1pms = TJATVG_GrdLimitTgtCrvTGC_C_1pms;
        }

        /* End of Switch generated from: '<S957>/Switch' */
    } else {
        /* Switch generated from: '<S892>/Switch' incorporates:
         *  Constant: '<S958>/Constant1'
         *  Switch generated from: '<S892>/Switch1'
         */
        TJATVG_MaxCrvTrajGuiCtl_1pm = TJATVG_LimiterMaxCrv_C_1pm;

        /* Switch generated from: '<S892>/Switch' incorporates:
         *  Constant: '<S958>/Constant2'
         *  Switch generated from: '<S892>/Switch1'
         */
        TJATVG_MaxCrvGrdBuildup_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;

        /* Switch generated from: '<S892>/Switch' incorporates:
         *  Constant: '<S958>/Constant3'
         *  Switch generated from: '<S892>/Switch1'
         */
        TJATVG_MaxCrvGrdRed_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;

        /* Switch generated from: '<S892>/Switch' incorporates:
         *  Constant: '<S958>/Constant4'
         *  Switch generated from: '<S892>/Switch1'
         */
        TJATVG_MaxCrvGrdTGC_1pms = TJATVG_LimiterMaxCrvGrd_C_1pms;
    }

    /* End of Switch generated from: '<S892>/Switch' */

    /* Switch: '<S924>/Switch' incorporates:
     *  Constant: '<S928>/Constant'
     *  Constant: '<S929>/Constant'
     *  Constant: '<S930>/Constant'
     *  Constant: '<S931>/Constant'
     *  Constant: '<S932>/Constant'
     *  Constant: '<S933>/Constant'
     *  Logic: '<S924>/OR1'
     *  Logic: '<S924>/OR2'
     *  Logic: '<S924>/OR3'
     *  RelationalOperator: '<S924>/Equal1'
     *  RelationalOperator: '<S924>/Equal2'
     *  RelationalOperator: '<S924>/Equal3'
     *  RelationalOperator: '<S924>/Equal4'
     *  RelationalOperator: '<S924>/Equal5'
     *  RelationalOperator: '<S924>/Equal6'
     *  Switch: '<S924>/Switch1'
     *  Switch: '<S924>/Switch2'
     */
    if ((((uint32_T)TJASTM_LatCtrlMode_nu) ==
         E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC) ||
        (((uint32_T)TJASTM_LatCtrlMode_nu) ==
         E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC_RQ)) {
        /* Switch: '<S924>/Switch' incorporates:
         *  Constant: '<S924>/Constant'
         */
        TJATVG_LtcyCompActivated_nu = TJATVG_UseLtcyCompLC_C_bool;
    } else if ((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF) ||
               (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF_RQ)) {
        /* Switch: '<S924>/Switch1' incorporates:
         *  Constant: '<S924>/Constant1'
         *  Switch: '<S924>/Switch'
         */
        TJATVG_LtcyCompActivated_nu = TJATVG_UseLtcyCompOF_C_bool;
    } else if ((((uint32_T)TJASTM_LatCtrlMode_nu) ==
                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB) ||
               (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ)) {
        /* Switch: '<S924>/Switch2' incorporates:
         *  Constant: '<S924>/Constant2'
         *  Switch: '<S924>/Switch'
         *  Switch: '<S924>/Switch1'
         */
        TJATVG_LtcyCompActivated_nu = TJATVG_UseLtcyCompCMB_C_bool;
    } else {
        /* Switch: '<S924>/Switch' incorporates:
         *  Constant: '<S924>/Constant3'
         *  Constant: '<S934>/Constant'
         *  Constant: '<S935>/Constant'
         *  Logic: '<S924>/OR4'
         *  RelationalOperator: '<S924>/Equal7'
         *  RelationalOperator: '<S924>/Equal8'
         *  Switch: '<S924>/Switch1'
         *  Switch: '<S924>/Switch2'
         *  Switch: '<S924>/Switch3'
         */
        TJATVG_LtcyCompActivated_nu =
            (((((uint32_T)TJASTM_LatCtrlMode_nu) ==
               E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC) ||
              (((uint32_T)TJASTM_LatCtrlMode_nu) ==
               E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC_RQ)) &&
             (TJATVG_UseLtcyCompSALC_C_bool));
    }

    /* End of Switch: '<S924>/Switch' */

    /* RelationalOperator: '<S889>/Equal7' incorporates:
     *  Constant: '<S923>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  Switch: '<S552>/Switch3'
     */
    rtb_TVG_Rampout_bool = (E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT ==
                            ((uint32_T)TJASTM_SysStateTJA_nu));

    /* Switch: '<S920>/Switch' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     *  UnitDelay: '<S920>/Unit Delay'
     */
    if (rtb_TVG_Rampout_bool) {
    } else {
        TVG_LatCtrlMode_Enum = TJASTM_LatCtrlMode_nu;
    }

    /* End of Switch: '<S920>/Switch' */

    /* Switch: '<S921>/Switch' incorporates:
     *  Constant: '<S921>/Constant1'
     *  Constant: '<S921>/Constant2'
     *  Constant: '<S922>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S889>/Equal6'
     *  Switch: '<S552>/Switch3'
     *  Switch: '<S921>/Switch1'
     */
    if (E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
        ((uint32_T)TJASTM_SysStateTJA_nu)) {
        rtb_DataTypeConversion1 = 0U;
    } else if (rtb_TVG_Rampout_bool) {
        /* Switch: '<S921>/Switch1' incorporates:
         *  Constant: '<S921>/Constant'
         */
        rtb_DataTypeConversion1 = 1U;
    } else {
        rtb_DataTypeConversion1 = 2U;
    }

    /* End of Switch: '<S921>/Switch' */

    /* MultiPortSwitch: '<S889>/Multiport Switch' */
    switch (rtb_DataTypeConversion1) {
        case 0:
            /* Switch: '<S919>/Switch' incorporates:
             *  Constant: '<S925>/Constant'
             *  Constant: '<S926>/Constant'
             *  Logic: '<S919>/OR1'
             *  RelationalOperator: '<S919>/Equal5'
             *  RelationalOperator: '<S919>/Equal6'
             */
            if ((E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF ==
                 ((uint32_T)TJASTM_LatCtrlMode_nu)) ||
                (((uint32_T)TJASTM_LatCtrlMode_nu) ==
                 E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF_RQ)) {
                /* MultiPortSwitch: '<S889>/Multiport Switch' incorporates:
                 *  Inport: '<Root>/Inport92'
                 */
                TJATVG_SensorTStamp_sec = ODPFOP_AccFRObjTStamp_sec;
            } else {
                /* MultiPortSwitch: '<S889>/Multiport Switch' incorporates:
                 *  Inport: '<Root>/Inport91'
                 */
                TJATVG_SensorTStamp_sec = ABPR_ABDTimeStamp_sec;
            }

            /* End of Switch: '<S919>/Switch' */
            break;

        case 1:
            /* Switch: '<S920>/Switch1' incorporates:
             *  Constant: '<S927>/Constant'
             *  RelationalOperator: '<S920>/Equal5'
             *  Switch: '<S920>/Switch'
             *  UnitDelay: '<S920>/Unit Delay'
             */
            if (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF ==
                ((uint32_T)TVG_LatCtrlMode_Enum)) {
                /* MultiPortSwitch: '<S889>/Multiport Switch' incorporates:
                 *  Inport: '<Root>/Inport92'
                 */
                TJATVG_SensorTStamp_sec = ODPFOP_AccFRObjTStamp_sec;
            } else {
                /* MultiPortSwitch: '<S889>/Multiport Switch' incorporates:
                 *  Inport: '<Root>/Inport91'
                 */
                TJATVG_SensorTStamp_sec = ABPR_ABDTimeStamp_sec;
            }

            /* End of Switch: '<S920>/Switch1' */
            break;

        default:
            /* MultiPortSwitch: '<S889>/Multiport Switch' incorporates:
             *  Constant: '<S889>/Constant'
             */
            TJATVG_SensorTStamp_sec = 0.0F;
            break;
    }

    /* End of MultiPortSwitch: '<S889>/Multiport Switch' */

    /* SignalConversion generated from: '<S680>/Vector Concatenate' */
    rtb_VectorConcatenate_aisn[3] = false;

    /* S-Function (ex_sfun_set_bit): '<S681>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S680>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_aisn[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S681>/Data Type Conversion1' */
    TJATTG_TgtCorridorInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_jx5q;

    /* Logic: '<S15>/NOT1' */
    rtb_VectorConcatenate_maf5[0] = rtb_AND18;

    /* Logic: '<S15>/NOT2' */
    rtb_VectorConcatenate_maf5[1] = rtb_GreaterThan_bpjw_idx_1;

    /* Constant: '<S15>/Constant3' */
    rtb_VectorConcatenate_maf5[2] = TJACMB_CombinedDataEnable_C_bool;

    /* SignalConversion: '<S15>/Signal Conversion1' */
    rtb_VectorConcatenate_maf5[3] = TJACMB_ObjectCorridor_bool;

    /* Logic: '<S15>/NOT3' */
    rtb_VectorConcatenate_maf5[4] = !rtb_CMB_VelocityValid_bool;

    /* Logic: '<S15>/NOT4' */
    rtb_VectorConcatenate_maf5[5] = !rtb_AND1_h3xh;

    /* Logic: '<S15>/NOT5' */
    rtb_VectorConcatenate_maf5[6] = rtb_OR_ly14;

    /* Logic: '<S15>/NOT6' */
    rtb_VectorConcatenate_maf5[8] = rtb_SLC_RiAdjLaneWidthValid_boo;

    /* SignalConversion: '<S15>/Signal Conversion2' incorporates:
     *  Logic: '<S17>/NOT'
     */
    rtb_VectorConcatenate_maf5[10] = !rtb_CMB_OFOObjLengthInvalid_boo;

    /* SignalConversion: '<S15>/Signal Conversion3' incorporates:
     *  Logic: '<S17>/NOT1'
     */
    rtb_VectorConcatenate_maf5[11] = !rtb_CMB_OFOVelocityInvalid_bool;

    /* SignalConversion: '<S15>/Signal Conversion4' incorporates:
     *  Logic: '<S17>/NOT2'
     */
    rtb_VectorConcatenate_maf5[12] = !rtb_CMB_OFOLnLengthInvalid_bool;

    /* SignalConversion: '<S15>/Signal Conversion5' incorporates:
     *  Constant: '<S17>/Constant4'
     */
    rtb_VectorConcatenate_maf5[13] = TJACMB_EnableOFO_C_bool;

    /* SignalConversion generated from: '<S38>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[14] = false;

    /* SignalConversion generated from: '<S38>/Vector Concatenate' */
    rtb_VectorConcatenate_maf5[15] = false;

    /* S-Function (ex_sfun_set_bit): '<S40>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S38>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_maf5[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S40>/Data Type Conversion1' */
    TJACMB_CombinedInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_jx5q;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[0] = rtb_OR_baue;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[1] = rtb_NotEqual1_cuxj;

    /* SignalConversion: '<S46>/Signal Conversion1' */
    rtb_VectorConcatenate_gygi[2] = rtb_GEN_Cancel_Custom_bool;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[3] = rtb_NotEqual2_emxm;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[4] = rtb_NotEqual3_lcsh;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[5] = false;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[6] = false;

    /* SignalConversion generated from: '<S55>/Vector Concatenate' */
    rtb_VectorConcatenate_gygi[7] = false;

    /* S-Function (ex_sfun_set_bit): '<S56>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S55>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_gygi[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S56>/Data Type Conversion1' */
    TJAGEN_CancelStatus_btf = (uint8_T)rtb_ex_sfun_set_bit_jx5q;

    /* Logic: '<S54>/NOT' */
    rtb_VectorConcatenate_gygi[0] = !rtb_GEN_BlockTimerExpired_bool;

    /* Logic: '<S54>/NOT1' */
    rtb_VectorConcatenate_gygi[1] = !rtb_OR_idx_0;

    /* Logic: '<S54>/NOT2' */
    rtb_VectorConcatenate_gygi[2] = !rtb_Equal1_p2mp;

    /* Logic: '<S54>/NOT3' */
    rtb_VectorConcatenate_gygi[3] = !rtb_Equal2_kbd5;

    /* Logic: '<S54>/NOT4' */
    rtb_VectorConcatenate_gygi[4] = !rtb_GEN_WR_Custom_bool;

    /* Logic: '<S54>/NOT6' */
    rtb_VectorConcatenate_gygi[5] = !rtb_LessThan_ko2v;

    /* SignalConversion: '<S54>/Signal Conversion16' */
    rtb_VectorConcatenate_gygi[6] = rtb_GEN_MaxSteerAngleExceeded_b;

    /* SignalConversion generated from: '<S98>/Vector Concatenate' incorporates:
     *  Constant: '<S54>/Constant8'
     */
    rtb_VectorConcatenate_gygi[7] = false;

    /* S-Function (ex_sfun_set_bit): '<S100>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S98>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_gygi[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_2[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S100>/Data Type Conversion1' */
    TJAGEN_WeakReadyInvalid_btf = (uint8_T)rtb_ex_sfun_set_bit_jx5q;

    /* Logic: '<S8>/NOT' */
    rtb_VectorConcatenate_aisn[0] = !TJAGEN_StrongReady_bool;

    /* Logic: '<S8>/NOT1' */
    rtb_VectorConcatenate_aisn[1] = !TJAGEN_WeakReady_bool;

    /* Logic: '<S8>/NOT2' */
    rtb_VectorConcatenate_aisn[2] = rtb_AND18;

    /* Logic: '<S8>/NOT3' */
    rtb_VectorConcatenate_aisn[3] = rtb_OR_ly14;

    /* Logic: '<S8>/NOT4' */
    rtb_VectorConcatenate_aisn[4] = rtb_GreaterThan_bpjw_idx_1;

    /* Logic: '<S8>/NOT5' */
    rtb_VectorConcatenate_aisn[5] = rtb_SLC_RiAdjLaneWidthValid_boo;

    /* Logic: '<S8>/NOT6' */
    rtb_VectorConcatenate_aisn[6] = rtb_GreaterThan_bpjw_idx_0;

    /* Logic: '<S8>/NOT7' */
    rtb_VectorConcatenate_aisn[7] = rtb_SLC_PrevDriverTriggerResetR;

    /* Logic: '<S8>/NOT8' */
    rtb_VectorConcatenate_aisn[8] = rtb_SLC_RiAdjLaneValid_bool;

    /* Logic: '<S8>/NOT9' */
    rtb_VectorConcatenate_aisn[9] = rtb_OBF_VelocityValid_bool;

    /* SignalConversion: '<S8>/Signal Conversion17' */
    rtb_VectorConcatenate_aisn[10] = STM_Cancel_bool;

    /* Logic: '<S8>/NOT11' */
    rtb_VectorConcatenate_aisn[11] = !TJAGEN_FunctionSwitch_bool;

    /* SignalConversion: '<S8>/Signal Conversion2' */
    rtb_VectorConcatenate_aisn[12] = TJAGEN_Clearance_bool;

    /* Logic: '<S8>/NOT13' */
    rtb_VectorConcatenate_aisn[13] = !TJAGEN_CodeFunction_bool;

    /* Logic: '<S8>/NOT14' */
    rtb_VectorConcatenate_aisn[14] = !TJAGEN_Error_bool;

    /* SignalConversion: '<S8>/Signal Conversion4' */
    rtb_VectorConcatenate_aisn[15] = TJAGEN_FunctionQuit_bool;

    /* S-Function (ex_sfun_set_bit): '<S572>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S550>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_aisn[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S572>/Data Type Conversion1' */
    TJASTM_TJAInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_jx5q;

    /* SignalConversion: '<S52>/Signal Conversion' */
    rtb_VectorConcatenate_aisn[0] = rtb_AND_nxkg;

    /* SignalConversion: '<S52>/Signal Conversion1' */
    rtb_VectorConcatenate_aisn[1] = rtb_GEN_BrakePadelInvalid__gzuo;

    /* SignalConversion: '<S52>/Signal Conversion2' */
    rtb_VectorConcatenate_aisn[2] = rtb_GEN_DrvNotBuckledUp_bool;

    /* SignalConversion: '<S52>/Signal Conversion3' */
    rtb_VectorConcatenate_aisn[3] = rtb_OR2_pwtt;

    /* SignalConversion: '<S52>/Signal Conversion4' incorporates:
     *  UnitDelay: '<S82>/Unit Delay'
     */
    rtb_VectorConcatenate_aisn[4] = GEN_SteerWAngleHyst_bool;

    /* SignalConversion: '<S52>/Signal Conversion5' incorporates:
     *  UnitDelay: '<S81>/Unit Delay'
     */
    rtb_VectorConcatenate_aisn[5] = GEN_SteerWAngleGradHyst_bool;

    /* SignalConversion: '<S52>/Signal Conversion6' */
    rtb_VectorConcatenate_aisn[6] = rtb_GEN_VehStInvalid_bool;

    /* SignalConversion: '<S52>/Signal Conversion7' */
    rtb_VectorConcatenate_aisn[7] = rtb_AND_gpqq;

    /* SignalConversion: '<S52>/Signal Conversion8' */
    rtb_VectorConcatenate_aisn[8] = rtb_AND_pjo1;

    /* SignalConversion: '<S52>/Signal Conversion9' */
    rtb_VectorConcatenate_aisn[9] = rtb_OR_imzg_idx_0;

    /* SignalConversion: '<S52>/Signal Conversion10' incorporates:
     *  UnitDelay: '<S84>/Unit Delay'
     */
    rtb_VectorConcatenate_aisn[10] = GEN_VehCrvDIHyst_bool;

    /* SignalConversion: '<S52>/Signal Conversion11' incorporates:
     *  UnitDelay: '<S83>/Unit Delay'
     */
    rtb_VectorConcatenate_aisn[11] = GEN_SteerWAngleSusHyst_bool;

    /* SignalConversion: '<S52>/Signal Conversion12' incorporates:
     *  UnitDelay: '<S80>/Unit Delay'
     */
    rtb_VectorConcatenate_aisn[12] = GEN_SteerWAngleGradSusHyst_bool;

    /* SignalConversion: '<S52>/Signal Conversion13' */
    rtb_VectorConcatenate_aisn[13] = rtb_AND_cvae;

    /* SignalConversion: '<S52>/Signal Conversion14' */
    rtb_VectorConcatenate_aisn[14] = rtb_AND_eqpk;

    /* SignalConversion: '<S52>/Signal Conversion15' */
    rtb_VectorConcatenate_aisn[15] = rtb_OR_imzg_idx_1;

    /* S-Function (ex_sfun_set_bit): '<S78>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S66>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_aisn[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_1[0])), ((uint8_T)16U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S78>/Data Type Conversion1' */
    TJAGEN_SuspendedAndQuit_debug = (uint16_T)rtb_ex_sfun_set_bit_jx5q;

    /* Logic: '<S101>/AND3' */
    TJALKA_SRLaneRelateCheck_bool =
        (((((rtb_OR_d1pw && rtb_LKA_Dist2BndsValid_bool) &&
            rtb_LKA_LaneWidthValid_bool) &&
           rtb_LKA_RadiusValid_bool) &&
          rtb_LKA_LengthValid_bool) &&
         rtb_LKA_NoLaneChange_bool);

    /* RelationalOperator: '<S553>/Equal' incorporates:
     *  Constant: '<S606>/Constant'
     */
    rtb_LKA_Dist2BndsValid_bool = (((uint32_T)TJASTM_SysStateTJAIn_nu) ==
                                   E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED);

    /* Logic: '<S553>/NOT' incorporates:
     *  Logic: '<S613>/AND12'
     */
    rtb_OR_d1pw = !TJALKA_SRLaneRelateCheck_bool;

    /* Logic: '<S553>/AND' incorporates:
     *  Constant: '<S607>/Constant'
     *  Constant: '<S608>/Constant'
     *  Logic: '<S553>/AND1'
     *  Logic: '<S553>/AND2'
     *  Logic: '<S553>/AND3'
     *  Logic: '<S553>/NOT'
     *  Logic: '<S553>/NOT1'
     *  Logic: '<S605>/AND'
     *  Logic: '<S605>/NOT'
     *  RelationalOperator: '<S553>/Equal1'
     *  RelationalOperator: '<S553>/Equal2'
     *  UnitDelay: '<S605>/Unit Delay'
     */
    TJASTM_DrvTakeOver_bool =
        ((rtb_LKA_Dist2BndsValid_bool && (!STM_SuspendEdgeRising_bool)) &&
         ((rtb_OR_d1pw && (E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ==
                           ((uint32_T)GEN_PrevLatCtrlMode_Enum))) ||
          ((!OF_NoObjectCollision_bool) &&
           (((uint32_T)GEN_PrevLatCtrlMode_Enum) ==
            E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF))));

    /* Logic: '<S610>/AND' incorporates:
     *  Logic: '<S610>/NOT'
     *  UnitDelay: '<S610>/Unit Delay'
     */
    rtb_AND2_emk5 =
        (TJASTM_DrvTakeOver_bool && (!TJATOW_TakeOverTurnOnDelay_nu));

    /* Switch: '<S615>/Switch' incorporates:
     *  Constant: '<S9>/Constant7'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S615>/Max'
     *  Sum: '<S615>/Subtract'
     *  Switch: '<S615>/Switch1'
     *  UnaryMinus: '<S615>/Unary Minus'
     *  UnitDelay: '<S615>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        TJATOW_TakeOverTurnOffDelay_sec = TJASTM_SusTakeOverTurnOffTime_C_sec;
    } else {
        TJATOW_TakeOverTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_TakeOverTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S615>/Switch' */

    /* Switch: '<S614>/Switch' incorporates:
     *  Constant: '<S609>/Constant5'
     *  Constant: '<S611>/Constant'
     *  Constant: '<S612>/Constant'
     *  Constant: '<S614>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S615>/OR'
     *  Logic: '<S9>/AND1'
     *  Logic: '<S9>/AND2'
     *  Logic: '<S9>/NOT1'
     *  Logic: '<S9>/OR1'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S609>/NotEqual1'
     *  RelationalOperator: '<S615>/GreaterThan'
     *  RelationalOperator: '<S9>/Equal1'
     *  RelationalOperator: '<S9>/Equal2'
     *  Switch: '<S552>/Switch3'
     *  Switch: '<S614>/Switch1'
     *  UnaryMinus: '<S615>/Unary Minus1'
     *  UnitDelay: '<S614>/Unit Delay'
     *  UnitDelay: '<S615>/Unit Delay'
     */
    if ((((((((uint32_T)TJASTM_SysStateTJA_nu) !=
             E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED) &&
            (((uint32_T)TJASTM_SysStateTJA_nu) !=
             E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT)) ||
           TJALKA_SRLaneRelateCheck_bool) ||
          rtb_AND_cvae) ||
         ((!rtb_AND2_emk5) && (TJATOW_TakeOverTurnOffDelay_sec <=
                               (-LCFRCV_TSysCycleTimeSen_sec)))) ||
        (rtb_GEN_BrakePadelInvalid__hw14 != 0)) {
        TJATOW_RSFlipFlop_nu = false;
    } else {
        TJATOW_RSFlipFlop_nu =
            (TJASTM_DrvTakeOver_bool || (TJATOW_RSFlipFlop_nu));
    }

    /* End of Switch: '<S614>/Switch' */

    /* DataTypeConversion: '<S9>/Data Type Conversion1' incorporates:
     *  UnitDelay: '<S614>/Unit Delay'
     */
    TJATOW_DriverTakeOverWarning_nu =
        TJATOW_RSFlipFlop_nu ? ((uint8_T)1) : ((uint8_T)0);

    /* Logic: '<S613>/AND11' incorporates:
     *  Constant: '<S632>/Constant'
     *  Inport: '<Root>/Inport105'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal14'
     *  Switch: '<S552>/Switch3'
     */
    rtb_OR_d1pw = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                     E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                    LCFRCV_PilotOnLeverSwitch_bool) &&
                   rtb_OR_d1pw);

    /* Logic: '<S620>/AND' incorporates:
     *  Logic: '<S620>/NOT'
     *  UnitDelay: '<S620>/Unit Delay'
     */
    rtb_NotEqual3_lcsh = (rtb_OR_d1pw && (!TJATOW_NPilotLineUnitDealy_bool));

    /* Switch: '<S662>/Switch' incorporates:
     *  Constant: '<S613>/Constant18'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S662>/Max'
     *  Sum: '<S662>/Subtract'
     *  Switch: '<S662>/Switch1'
     *  UnaryMinus: '<S662>/Unary Minus'
     *  UnitDelay: '<S662>/Unit Delay'
     */
    if (rtb_NotEqual3_lcsh) {
        TJATOW_NPilotLaneAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotLaneAudioPlay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               TJATOW_NPilotLaneAudioPlay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S662>/Switch' */

    /* DataTypeConversion: '<S200>/Data Type Conversion1' */
    TJAOBF_ObjFollowInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_hiin;

    /* Logic: '<S613>/AND13' incorporates:
     *  Constant: '<S613>/Constant21'
     *  Constant: '<S633>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport74'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal15'
     *  S-Function (sfix_bitop): '<S613>/Bitwise AND1'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND_cvae = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                     LCFRCV_PilotOnLeverSwitch_bool) &&
                    ((((uint32_T)VDPDRV_DrvStInvalid_btf) & 2U) != 0U));

    /* Logic: '<S621>/AND' incorporates:
     *  Logic: '<S621>/NOT'
     *  UnitDelay: '<S621>/Unit Delay'
     */
    rtb_NotEqual2_emxm =
        (rtb_AND_cvae && (!TJATOW_NPilotSeatBeltUnitDealy_bool));

    /* Switch: '<S663>/Switch' incorporates:
     *  Constant: '<S613>/Constant20'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S663>/Max'
     *  Sum: '<S663>/Subtract'
     *  Switch: '<S663>/Switch1'
     *  UnaryMinus: '<S663>/Unary Minus'
     *  UnitDelay: '<S663>/Unit Delay'
     */
    if (rtb_NotEqual2_emxm) {
        TJATOW_NPilotSeatBeltPAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotSeatBeltPAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_NPilotSeatBeltPAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S663>/Switch' */

    /* Logic: '<S613>/AND14' incorporates:
     *  Constant: '<S634>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport112'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal18'
     *  Switch: '<S552>/Switch3'
     */
    rtb_LKA_LaneWidthValid_bool = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                     E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                                    LCFRCV_PilotOnLeverSwitch_bool) &&
                                   LCFRCV_DoorOpen_bool);

    /* Logic: '<S622>/AND' incorporates:
     *  Logic: '<S622>/NOT'
     *  UnitDelay: '<S622>/Unit Delay'
     */
    rtb_AND_pjo1 =
        (rtb_LKA_LaneWidthValid_bool && (!TJATOW_NPilotDoorOpenUnitDealy_bool));

    /* Switch: '<S650>/Switch' incorporates:
     *  Constant: '<S613>/Constant29'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S650>/Max'
     *  Sum: '<S650>/Subtract'
     *  Switch: '<S650>/Switch1'
     *  UnaryMinus: '<S650>/Unary Minus'
     *  UnitDelay: '<S650>/Unit Delay'
     */
    if (rtb_AND_pjo1) {
        TJATOW_NPilotDoorAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotDoorAudioPlay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               TJATOW_NPilotDoorAudioPlay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S650>/Switch' */

    /* Logic: '<S613>/AND15' incorporates:
     *  Constant: '<S635>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport114'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal19'
     *  Switch: '<S552>/Switch3'
     */
    rtb_LKA_RadiusValid_bool = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                  E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                                 LCFRCV_PilotOnLeverSwitch_bool) &&
                                LCFRCV_TrunkUnLock_bool);

    /* Logic: '<S623>/AND' incorporates:
     *  Logic: '<S623>/NOT'
     *  UnitDelay: '<S623>/Unit Delay'
     */
    rtb_AND_gpqq =
        (rtb_LKA_RadiusValid_bool && (!TJATOW_NPilotTrunkUnitDealy_bool));

    /* Switch: '<S651>/Switch' incorporates:
     *  Constant: '<S613>/Constant31'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S651>/Max'
     *  Sum: '<S651>/Subtract'
     *  Switch: '<S651>/Switch1'
     *  UnaryMinus: '<S651>/Unary Minus'
     *  UnitDelay: '<S651>/Unit Delay'
     */
    if (rtb_AND_gpqq) {
        TJATOW_NPilotTrunkAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotTrunkAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_NPilotTrunkAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S651>/Switch' */

    /* Logic: '<S613>/AND16' incorporates:
     *  Constant: '<S636>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport113'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal20'
     *  Switch: '<S552>/Switch3'
     */
    rtb_LKA_NoLaneChange_bool = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                   E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                                  LCFRCV_PilotOnLeverSwitch_bool) &&
                                 LCFRCV_HoodOpen_bool);

    /* Logic: '<S624>/AND' incorporates:
     *  Logic: '<S624>/NOT'
     *  UnitDelay: '<S624>/Unit Delay'
     */
    rtb_OR_baue =
        (rtb_LKA_NoLaneChange_bool && (!TJATOW_NPilotHoodUnitDealy_bool));

    /* Switch: '<S652>/Switch' incorporates:
     *  Constant: '<S613>/Constant33'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S652>/Max'
     *  Sum: '<S652>/Subtract'
     *  Switch: '<S652>/Switch1'
     *  UnaryMinus: '<S652>/Unary Minus'
     *  UnitDelay: '<S652>/Unit Delay'
     */
    if (rtb_OR_baue) {
        TJATOW_NPilotHoodAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotHoodAudioPlay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               TJATOW_NPilotHoodAudioPlay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S652>/Switch' */

    /* Logic: '<S613>/AND17' incorporates:
     *  Constant: '<S637>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport115'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal21'
     *  Switch: '<S552>/Switch3'
     */
    rtb_LKA_LengthValid_bool = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                  E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                                 LCFRCV_PilotOnLeverSwitch_bool) &&
                                LCFRCV_bPGear_bool);

    /* Logic: '<S625>/AND' incorporates:
     *  Logic: '<S625>/NOT'
     *  UnitDelay: '<S625>/Unit Delay'
     */
    rtb_GEN_DrvNotBuckledUp_bool =
        (rtb_LKA_LengthValid_bool && (!TJATOW_NPilotEPBUnitDealy_bool));

    /* Switch: '<S653>/Switch' incorporates:
     *  Constant: '<S613>/Constant35'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S653>/Max'
     *  Sum: '<S653>/Subtract'
     *  Switch: '<S653>/Switch1'
     *  UnaryMinus: '<S653>/Unary Minus'
     *  UnitDelay: '<S653>/Unit Delay'
     */
    if (rtb_GEN_DrvNotBuckledUp_bool) {
        TJATOW_NPilotEPBAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotEPBAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJATOW_NPilotEPBAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S653>/Switch' */

    /* Logic: '<S613>/AND18' incorporates:
     *  Constant: '<S613>/Constant23'
     *  Constant: '<S613>/Constant24'
     *  Constant: '<S613>/Constant25'
     *  Constant: '<S613>/Constant27'
     *  Constant: '<S639>/Constant'
     *  Inport: '<Root>/Inport105'
     *  Inport: '<Root>/Inport71'
     *  Logic: '<S613>/AND19'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal'
     *  RelationalOperator: '<S613>/Equal16'
     *  RelationalOperator: '<S613>/Equal17'
     *  RelationalOperator: '<S613>/Equal22'
     *  S-Function (sfix_bitop): '<S613>/Bitwise AND'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND18 = (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                   E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                  LCFRCV_PilotOnLeverSwitch_bool) &&
                 ((((((int32_T)VDPDRV_ActiveStCtrl_btf) &
                     ((int32_T)TJAGEN_ActiveStCtrlWarn_C_btm)) != 0) ||
                   (rtb_GEN_WR_Custom_bool_tmp != 0)) ||
                  (GEN_AllStateAvailable_bool_tmp != 0)));

    /* Logic: '<S626>/AND' incorporates:
     *  Logic: '<S626>/NOT'
     *  UnitDelay: '<S626>/Unit Delay'
     */
    rtb_LessThan_ko2v = (rtb_AND18 && (!TJATOW_NPilotSafeUnitDealy_bool));

    /* Switch: '<S654>/Switch' incorporates:
     *  Constant: '<S613>/Constant37'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S654>/Max'
     *  Sum: '<S654>/Subtract'
     *  Switch: '<S654>/Switch1'
     *  UnaryMinus: '<S654>/Unary Minus'
     *  UnitDelay: '<S654>/Unit Delay'
     */
    if (rtb_LessThan_ko2v) {
        TJATOW_NPilotSafeAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotSafeAudioPlay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               TJATOW_NPilotSafeAudioPlay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S654>/Switch' */

    /* Logic: '<S613>/AND20' incorporates:
     *  Constant: '<S640>/Constant'
     *  Inport: '<Root>/Inport105'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal23'
     *  Switch: '<S552>/Switch3'
     */
    rtb_GEN_VehStInvalid_bool = ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                  E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                                 LCFRCV_PilotOnLeverSwitch_bool);

    /* Logic: '<S618>/AND' incorporates:
     *  Logic: '<S618>/NOT'
     *  UnitDelay: '<S618>/Unit Delay'
     */
    rtb_GEN_WR_Custom_bool =
        (rtb_GEN_VehStInvalid_bool && (!TJATOW_NPilotNoUnitDealy_bool));

    /* Switch: '<S655>/Switch' incorporates:
     *  Constant: '<S613>/Constant39'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S655>/Max'
     *  Sum: '<S655>/Subtract'
     *  Switch: '<S655>/Switch1'
     *  UnaryMinus: '<S655>/Unary Minus'
     *  UnitDelay: '<S655>/Unit Delay'
     */
    if (rtb_GEN_WR_Custom_bool) {
        TJATOW_NPilotNoAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotNoAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJATOW_NPilotNoAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S655>/Switch' */

    /* Logic: '<S613>/AND7' incorporates:
     *  Constant: '<S647>/Constant'
     *  Inport: '<Root>/Inport105'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal9'
     *  Switch: '<S552>/Switch3'
     */
    rtb_GEN_BrakePadelInvalid__gzuo =
        (((((uint32_T)TJASTM_SysStateTJA_nu) ==
           E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
          LCFRCV_PilotOnLeverSwitch_bool) &&
         rtb_GEN_BrakePadelInvalid__gzuo);

    /* Logic: '<S619>/AND' incorporates:
     *  Logic: '<S619>/NOT'
     *  UnitDelay: '<S619>/Unit Delay'
     */
    rtb_Equal2_kbd5 =
        (rtb_GEN_BrakePadelInvalid__gzuo && (!TJATOW_NPilotBrkPUnitDealy_bool));

    /* Switch: '<S659>/Switch' incorporates:
     *  Constant: '<S613>/Constant9'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S659>/Max'
     *  Sum: '<S659>/Subtract'
     *  Switch: '<S659>/Switch1'
     *  UnaryMinus: '<S659>/Unary Minus'
     *  UnitDelay: '<S659>/Unit Delay'
     */
    if (rtb_Equal2_kbd5) {
        TJATOW_NPilotBrkPAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotBrkPAudioPlay_sec = fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                                               TJATOW_NPilotBrkPAudioPlay_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S659>/Switch' */

    /* Logic: '<S613>/AND5' incorporates:
     *  Constant: '<S646>/Constant'
     *  Constant: '<S648>/Constant5'
     *  Inport: '<Root>/Inport105'
     *  Logic: '<S648>/AND2'
     *  Logic: '<S648>/AND3'
     *  Logic: '<S648>/AND4'
     *  Logic: '<S648>/AND5'
     *  Logic: '<S648>/AND6'
     *  Logic: '<S648>/AND7'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal8'
     *  RelationalOperator: '<S648>/NotEqual4'
     *  S-Function (sfix_bitop): '<S648>/Bitwise AND2'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND5_c5w4 =
        (((((uint32_T)TJASTM_SysStateTJA_nu) ==
           E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
          LCFRCV_PilotOnLeverSwitch_bool) &&
         ((((TJALKA_StrongReady_bool && rtb_OR_ly14) &&
            (!rtb_LKA_VelocityValid_bool)) &&
           (rtb_SLC_RiAdjLaneWidthValid_boo || rtb_GreaterThan_bpjw_idx_1)) ||
          ((((TJAOBF_WeakReady_bool && rtb_GreaterThan_bpjw_idx_1) &&
             rtb_AND5_c5w4) &&
            ((((int32_T)TJAOBF_ObjFollowInvalid_btf) &
              ((int32_T)TJATOW_OFSpdInvalid_C_btm)) == 0)) &&
           rtb_SLC_VelocityValid_bool)));

    /* Logic: '<S617>/AND' incorporates:
     *  Logic: '<S617>/NOT'
     *  UnitDelay: '<S617>/Unit Delay'
     */
    rtb_CMB_OFOObjLengthInvalid_boo =
        (rtb_AND5_c5w4 && (!TJATOW_NPilotSpdUnitDealy_bool));

    /* Switch: '<S658>/Switch' incorporates:
     *  Constant: '<S613>/Constant6'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S658>/Max'
     *  Sum: '<S658>/Subtract'
     *  Switch: '<S658>/Switch1'
     *  UnaryMinus: '<S658>/Unary Minus'
     *  UnitDelay: '<S658>/Unit Delay'
     */
    if (rtb_CMB_OFOObjLengthInvalid_boo) {
        TJATOW_NPilotSpdAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotSpdAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJATOW_NPilotSpdAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S658>/Switch' */

    /* Logic: '<S613>/AND4' incorporates:
     *  Constant: '<S645>/Constant'
     *  Inport: '<Root>/Inport105'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal7'
     *  Switch: '<S552>/Switch3'
     */
    rtb_LKA_VelocityValid_bool = ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                                   E_TJASTM_SysStateTJA_nu_SYSST_ERROR) &&
                                  LCFRCV_PilotOnLeverSwitch_bool);

    /* Logic: '<S616>/AND' incorporates:
     *  Logic: '<S616>/NOT'
     *  UnitDelay: '<S616>/Unit Delay'
     */
    rtb_AND_nxkg =
        (rtb_LKA_VelocityValid_bool && (!TJATOW_NPilotErrorUnitDealy_bool));

    /* Switch: '<S657>/Switch' incorporates:
     *  Constant: '<S613>/Constant5'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S657>/Max'
     *  Sum: '<S657>/Subtract'
     *  Switch: '<S657>/Switch1'
     *  UnaryMinus: '<S657>/Unary Minus'
     *  UnitDelay: '<S657>/Unit Delay'
     */
    if (rtb_AND_nxkg) {
        TJATOW_NPilotErrorAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotErrorAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_NPilotErrorAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S657>/Switch' */

    /* Logic: '<S613>/AND1' incorporates:
     *  Constant: '<S641>/Constant'
     *  Constant: '<S642>/Constant'
     *  Constant: '<S643>/Constant'
     *  Constant: '<S644>/Constant'
     *  Logic: '<S613>/AND3'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal3'
     *  RelationalOperator: '<S613>/Equal4'
     *  RelationalOperator: '<S613>/Equal5'
     *  RelationalOperator: '<S613>/Equal6'
     *  Switch: '<S552>/Switch3'
     */
    rtb_CMB_OFOVelocityInvalid_bool =
        ((((uint32_T)TJASTM_SysStateTJA_nu) ==
          E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
         (((((uint32_T)STM_SysStateUnitDelay_bool) ==
            E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) ||
           (((uint32_T)STM_SysStateUnitDelay_bool) ==
            E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED)) ||
          (((uint32_T)STM_SysStateUnitDelay_bool) ==
           E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT)));

    /* Switch: '<S656>/Switch' incorporates:
     *  Constant: '<S613>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S656>/Max'
     *  Sum: '<S656>/Subtract'
     *  Switch: '<S656>/Switch1'
     *  UnaryMinus: '<S656>/Unary Minus'
     *  UnitDelay: '<S656>/Unit Delay'
     */
    if (rtb_CMB_OFOVelocityInvalid_bool) {
        TJATOW_NPilotOffAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotOffAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJATOW_NPilotOffAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S656>/Switch' */

    /* Logic: '<S613>/AND2' incorporates:
     *  Constant: '<S627>/Constant'
     *  Constant: '<S638>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal1'
     *  RelationalOperator: '<S613>/Equal2'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND2_emk5 = ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) &&
                     (((uint32_T)STM_SysStateUnitDelay_bool) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_STANDBY));

    /* Switch: '<S649>/Switch' incorporates:
     *  Constant: '<S613>/Constant7'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S649>/Max'
     *  Sum: '<S649>/Subtract'
     *  Switch: '<S649>/Switch1'
     *  UnaryMinus: '<S649>/Unary Minus'
     *  UnitDelay: '<S649>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        TJATOW_NPilotOnAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_NPilotOnAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJATOW_NPilotOnAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S649>/Switch' */

    /* Switch: '<S613>/Switch' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S649>/OR'
     *  Logic: '<S650>/OR'
     *  Logic: '<S651>/OR'
     *  Logic: '<S652>/OR'
     *  Logic: '<S653>/OR'
     *  Logic: '<S654>/OR'
     *  Logic: '<S655>/OR'
     *  Logic: '<S656>/OR'
     *  Logic: '<S657>/OR'
     *  Logic: '<S658>/OR'
     *  Logic: '<S659>/OR'
     *  Logic: '<S662>/OR'
     *  Logic: '<S663>/OR'
     *  RelationalOperator: '<S649>/GreaterThan'
     *  RelationalOperator: '<S650>/GreaterThan'
     *  RelationalOperator: '<S651>/GreaterThan'
     *  RelationalOperator: '<S652>/GreaterThan'
     *  RelationalOperator: '<S653>/GreaterThan'
     *  RelationalOperator: '<S654>/GreaterThan'
     *  RelationalOperator: '<S655>/GreaterThan'
     *  RelationalOperator: '<S656>/GreaterThan'
     *  RelationalOperator: '<S657>/GreaterThan'
     *  RelationalOperator: '<S658>/GreaterThan'
     *  RelationalOperator: '<S659>/GreaterThan'
     *  RelationalOperator: '<S662>/GreaterThan'
     *  RelationalOperator: '<S663>/GreaterThan'
     *  Switch: '<S613>/Switch1'
     *  Switch: '<S613>/Switch10'
     *  Switch: '<S613>/Switch11'
     *  Switch: '<S613>/Switch12'
     *  Switch: '<S613>/Switch13'
     *  Switch: '<S613>/Switch14'
     *  Switch: '<S613>/Switch2'
     *  Switch: '<S613>/Switch3'
     *  Switch: '<S613>/Switch4'
     *  Switch: '<S613>/Switch7'
     *  Switch: '<S613>/Switch8'
     *  Switch: '<S613>/Switch9'
     *  UnaryMinus: '<S649>/Unary Minus1'
     *  UnaryMinus: '<S650>/Unary Minus1'
     *  UnaryMinus: '<S651>/Unary Minus1'
     *  UnaryMinus: '<S652>/Unary Minus1'
     *  UnaryMinus: '<S653>/Unary Minus1'
     *  UnaryMinus: '<S654>/Unary Minus1'
     *  UnaryMinus: '<S655>/Unary Minus1'
     *  UnaryMinus: '<S656>/Unary Minus1'
     *  UnaryMinus: '<S657>/Unary Minus1'
     *  UnaryMinus: '<S658>/Unary Minus1'
     *  UnaryMinus: '<S659>/Unary Minus1'
     *  UnaryMinus: '<S662>/Unary Minus1'
     *  UnaryMinus: '<S663>/Unary Minus1'
     *  UnitDelay: '<S649>/Unit Delay'
     *  UnitDelay: '<S650>/Unit Delay'
     *  UnitDelay: '<S651>/Unit Delay'
     *  UnitDelay: '<S652>/Unit Delay'
     *  UnitDelay: '<S653>/Unit Delay'
     *  UnitDelay: '<S654>/Unit Delay'
     *  UnitDelay: '<S655>/Unit Delay'
     *  UnitDelay: '<S656>/Unit Delay'
     *  UnitDelay: '<S657>/Unit Delay'
     *  UnitDelay: '<S658>/Unit Delay'
     *  UnitDelay: '<S659>/Unit Delay'
     *  UnitDelay: '<S662>/Unit Delay'
     *  UnitDelay: '<S663>/Unit Delay'
     */
    if (rtb_AND2_emk5 ||
        (TJATOW_NPilotOnAudioPlay_sec > (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch' incorporates:
         *  Constant: '<S613>/Constant1'
         */
        TJASTM_NpilotSysInfo = 1U;
    } else if (rtb_CMB_OFOVelocityInvalid_bool ||
               (TJATOW_NPilotOffAudioPlay_sec >
                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch1' incorporates:
         *  Constant: '<S613>/Constant3'
         *  Switch: '<S613>/Switch'
         */
        TJASTM_NpilotSysInfo = 2U;
    } else if (rtb_AND_nxkg || (TJATOW_NPilotErrorAudioPlay_sec >
                                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch2' incorporates:
         *  Constant: '<S613>/Constant4'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         */
        TJASTM_NpilotSysInfo = 3U;
    } else if (rtb_CMB_OFOObjLengthInvalid_boo ||
               (TJATOW_NPilotSpdAudioPlay_sec >
                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch3' incorporates:
         *  Constant: '<S613>/Constant8'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         */
        TJASTM_NpilotSysInfo = 4U;
    } else if (rtb_Equal2_kbd5 || (TJATOW_NPilotBrkPAudioPlay_sec >
                                   (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch4' incorporates:
         *  Constant: '<S613>/Constant10'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         */
        TJASTM_NpilotSysInfo = 5U;
    } else if (rtb_NotEqual3_lcsh || (TJATOW_NPilotLaneAudioPlay_sec >
                                      (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch7' incorporates:
         *  Constant: '<S613>/Constant11'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         */
        TJASTM_NpilotSysInfo = 6U;
    } else if (rtb_NotEqual2_emxm || (TJATOW_NPilotSeatBeltPAudioPlay_sec >
                                      (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch8' incorporates:
         *  Constant: '<S613>/Constant19'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         */
        TJASTM_NpilotSysInfo = 7U;
    } else if (rtb_AND_pjo1 || (TJATOW_NPilotDoorAudioPlay_sec >
                                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch9' incorporates:
         *  Constant: '<S613>/Constant28'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         */
        TJASTM_NpilotSysInfo = 8U;
    } else if (rtb_AND_gpqq || (TJATOW_NPilotTrunkAudioPlay_sec >
                                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch10' incorporates:
         *  Constant: '<S613>/Constant30'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = 9U;
    } else if (rtb_OR_baue || (TJATOW_NPilotHoodAudioPlay_sec >
                               (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch11' incorporates:
         *  Constant: '<S613>/Constant32'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch10'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = 10U;
    } else if (rtb_GEN_DrvNotBuckledUp_bool ||
               (TJATOW_NPilotEPBAudioPlay_sec >
                (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch12' incorporates:
         *  Constant: '<S613>/Constant34'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch10'
         *  Switch: '<S613>/Switch11'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = 11U;
    } else if (rtb_LessThan_ko2v || (TJATOW_NPilotSafeAudioPlay_sec >
                                     (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch13' incorporates:
         *  Constant: '<S613>/Constant36'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch10'
         *  Switch: '<S613>/Switch11'
         *  Switch: '<S613>/Switch12'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = 12U;
    } else if (rtb_GEN_WR_Custom_bool || (TJATOW_NPilotNoAudioPlay_sec >
                                          (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch14' incorporates:
         *  Constant: '<S613>/Constant38'
         *  Switch: '<S613>/Switch'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch10'
         *  Switch: '<S613>/Switch11'
         *  Switch: '<S613>/Switch12'
         *  Switch: '<S613>/Switch13'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = 13U;
    } else {
        /* Switch: '<S613>/Switch' incorporates:
         *  Constant: '<S613>/Constant17'
         *  Switch: '<S613>/Switch1'
         *  Switch: '<S613>/Switch10'
         *  Switch: '<S613>/Switch11'
         *  Switch: '<S613>/Switch12'
         *  Switch: '<S613>/Switch13'
         *  Switch: '<S613>/Switch14'
         *  Switch: '<S613>/Switch2'
         *  Switch: '<S613>/Switch3'
         *  Switch: '<S613>/Switch4'
         *  Switch: '<S613>/Switch7'
         *  Switch: '<S613>/Switch8'
         *  Switch: '<S613>/Switch9'
         */
        TJASTM_NpilotSysInfo = TJATOW_NPilotSysInfoDefault_nu;
    }

    /* End of Switch: '<S613>/Switch' */

    /* Logic: '<S111>/AND1' incorporates:
     *  Constant: '<S111>/Constant'
     *  Inport: '<Root>/Inport1'
     *  RelationalOperator: '<S111>/GreaterThan'
     *  RelationalOperator: '<S111>/GreaterThan3'
     *  Sum: '<S111>/Add'
     *  UnitDelay: '<S111>/Unit Delay'
     */
    rtb_AND2_emk5 =
        (((ABPR_LeLnClthPosY0_met - LKA_PrevLeLnPosY0UnitDelay_met) >
          TJALKA_LnIncoherenceMaxPosY0_C_met) &&
         (LKA_PrevLeLnPosY0UnitDelay_met < ABPR_LeLnClthPosY0_met));

    /* Logic: '<S111>/AND2' incorporates:
     *  Constant: '<S111>/Constant1'
     *  Inport: '<Root>/Inport2'
     *  RelationalOperator: '<S111>/GreaterThan1'
     *  RelationalOperator: '<S111>/GreaterThan2'
     *  Sum: '<S111>/Add1'
     *  UnitDelay: '<S111>/Unit Delay1'
     */
    rtb_OR_bsb5 = (((LKA_PrevRiLnPosY0UnitDelay_met - ABPR_RiLnClthPosY0_met) >
                    TJALKA_LnIncoherenceMaxPosY0_C_met) &&
                   (LKA_PrevRiLnPosY0UnitDelay_met > ABPR_RiLnClthPosY0_met));

    /* Logic: '<S111>/AND3' incorporates:
     *  Logic: '<S111>/NOT1'
     */
    rtb_OR_imjz = (rtb_AND2_emk5 && (!rtb_OR_bsb5));

    /* Switch: '<S131>/Switch' incorporates:
     *  Constant: '<S111>/Constant5'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S131>/Max'
     *  Sum: '<S131>/Subtract'
     *  Switch: '<S131>/Switch1'
     *  UnaryMinus: '<S131>/Unary Minus'
     *  UnitDelay: '<S131>/Unit Delay'
     */
    if (rtb_OR_imjz) {
        LKA_LeLnIncohTurnOffDelay_sec = TJALKA_LnIncohMaxTime_C_sec;
    } else {
        LKA_LeLnIncohTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, LKA_LeLnIncohTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S131>/Switch' */

    /* Logic: '<S131>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S131>/GreaterThan'
     *  UnaryMinus: '<S131>/Unary Minus1'
     *  UnitDelay: '<S131>/Unit Delay'
     */
    rtb_OR_imjz = (rtb_OR_imjz || (LKA_LeLnIncohTurnOffDelay_sec >
                                   (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S111>/AND4' incorporates:
     *  Logic: '<S111>/NOT2'
     */
    rtb_OR_bsb5 = ((!rtb_AND2_emk5) && rtb_OR_bsb5);

    /* Switch: '<S132>/Switch' incorporates:
     *  Constant: '<S111>/Constant6'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S132>/Max'
     *  Sum: '<S132>/Subtract'
     *  Switch: '<S132>/Switch1'
     *  UnaryMinus: '<S132>/Unary Minus'
     *  UnitDelay: '<S132>/Unit Delay'
     */
    if (rtb_OR_bsb5) {
        LKA_RiLnIncohTurnOffDelay_sec = TJALKA_LnIncohMaxTime_C_sec;
    } else {
        LKA_RiLnIncohTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, LKA_RiLnIncohTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S132>/Switch' */

    /* Logic: '<S132>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S132>/GreaterThan'
     *  UnaryMinus: '<S132>/Unary Minus1'
     *  UnitDelay: '<S132>/Unit Delay'
     */
    rtb_OR_bsb5 = (rtb_OR_bsb5 || (LKA_RiLnIncohTurnOffDelay_sec >
                                   (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S111>/AND' incorporates:
     *  Logic: '<S111>/OR'
     *  UnitDelay: '<S134>/Unit Delay'
     */
    rtb_AND_jt3t = ((rtb_AND_jt3t && (LKA_LaneWidthMaxHyst_bool)) &&
                    (rtb_OR_imjz || rtb_OR_bsb5));

    /* Logic: '<S125>/AND' incorporates:
     *  Logic: '<S125>/NOT'
     *  UnitDelay: '<S125>/Unit Delay'
     */
    rtb_AND2_emk5 = (rtb_AND_jt3t && (!LKA_LnIncohEdgeRising_bool));

    /* Switch: '<S130>/Switch' incorporates:
     *  Constant: '<S111>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S130>/Max'
     *  Sum: '<S130>/Subtract'
     *  Switch: '<S130>/Switch1'
     *  UnaryMinus: '<S130>/Unary Minus'
     *  UnitDelay: '<S130>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        LKA_LnIncohTurnOffDelay_sec = TJALKA_LnIncohMaxTime_C_sec;
    } else {
        LKA_LnIncohTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, LKA_LnIncohTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S130>/Switch' */

    /* Logic: '<S130>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S130>/GreaterThan'
     *  UnaryMinus: '<S130>/Unary Minus1'
     *  UnitDelay: '<S130>/Unit Delay'
     */
    rtb_AND2_emk5 = (rtb_AND2_emk5 || (LKA_LnIncohTurnOffDelay_sec >
                                       (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S126>/Switch' incorporates:
     *  Constant: '<S126>/Constant4'
     *  Logic: '<S126>/NOT3'
     *  Sum: '<S126>/Add2'
     *  UnitDelay: '<S126>/Unit Delay2'
     */
    if (!rtb_AND2_emk5) {
        LKA_PrevVehDistUnitDelay_met = 0.0F;
    } else {
        LKA_PrevVehDistUnitDelay_met =
            rtb_Subtract_if0w_idx_0 + LKA_PrevVehDistUnitDelay_met;
    }

    /* RelationalOperator: '<S126>/GreaterThan4' incorporates:
     *  Constant: '<S126>/Constant3'
     *  UnitDelay: '<S126>/Unit Delay2'
     */
    rtb_Equal_blkb = (LKA_PrevVehDistUnitDelay_met < TJALKA_VehDistMax_C_met);

    /* Logic: '<S111>/AND6' incorporates:
     *  Constant: '<S127>/Constant'
     *  Logic: '<S111>/AND5'
     *  RelationalOperator: '<S111>/Equal'
     */
    rtb_VectorConcatenate_br2p[0] = ((rtb_AND2_emk5 && rtb_Equal_blkb) &&
                                     (((uint32_T)STM_LatCtrlMdUnitDelay_bool) ==
                                      E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC));

    /* SignalConversion: '<S111>/Signal Conversion16' */
    rtb_VectorConcatenate_br2p[1] = rtb_Equal_blkb;

    /* SignalConversion: '<S111>/Signal Conversion3' */
    rtb_VectorConcatenate_br2p[2] = rtb_AND2_emk5;

    /* SignalConversion: '<S111>/Signal Conversion1' */
    rtb_VectorConcatenate_br2p[3] = rtb_OR_imjz;

    /* SignalConversion: '<S111>/Signal Conversion2' */
    rtb_VectorConcatenate_br2p[4] = rtb_OR_bsb5;

    /* S-Function (ex_sfun_set_bit): '<S133>/ex_sfun_set_bit' incorporates:
     *  Constant: '<S129>/Constant'
     */
    set_bit(0U, (boolean_T *)&rtb_VectorConcatenate_br2p[0],
            (uint8_T *)(&(TJASA_SetBit_BS_Param_3[0])), ((uint8_T)5U),
            &rtb_ex_sfun_set_bit_jx5q);

    /* DataTypeConversion: '<S133>/Data Type Conversion1' */
    TJALKA_LaneIncoherence_btf = (uint8_T)rtb_ex_sfun_set_bit_jx5q;

    /* RelationalOperator: '<S206>/Equal' incorporates:
     *  Constant: '<S206>/Constant'
     *  Constant: '<S206>/Constant1'
     *  Inport: '<Root>/Inport33'
     *  S-Function (sfix_bitop): '<S206>/Bitwise AND'
     */
    OBF_AccObjValid = ((((int32_T)ODPFOP_AccObjInvBitfield_btf) &
                        ((int32_T)TJAOBF_AccObjectInvalid_C_btm)) == 0);

    /* Switch: '<S215>/Switch' incorporates:
     *  Constant: '<S211>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S215>/Max'
     *  Sum: '<S215>/Subtract'
     *  Switch: '<S215>/Switch1'
     *  UnaryMinus: '<S215>/Unary Minus'
     *  UnitDelay: '<S215>/Unit Delay'
     */
    if (OBF_AccObjValid) {
        OBF_AccObjValidTurnOnDelay_sec = fmaxf(OBF_AccObjValidTurnOnDelay_sec,
                                               -LCFRCV_TSysCycleTimeSen_sec) -
                                         LCFRCV_TSysCycleTimeSen_sec;
    } else {
        OBF_AccObjValidTurnOnDelay_sec = TJAOBF_MinDurAccObjValid_C_sec;
    }

    /* End of Switch: '<S215>/Switch' */

    /* Logic: '<S215>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S215>/LessThanOrEqual'
     *  UnaryMinus: '<S215>/Unary Minus1'
     *  UnitDelay: '<S215>/Unit Delay'
     */
    OBF_AccObjValid_bool =
        ((OBF_AccObjValid) &&
         (OBF_AccObjValidTurnOnDelay_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S220>/Switch' */
    if (!rtb_OR_lkl2) {
        /* Switch: '<S208>/Switch' incorporates:
         *  Logic: '<S208>/NOT'
         *  UnitDelay: '<S220>/Unit Delay'
         */
        if (!OBF_LaneCheckValid_bool) {
            OBF_AccObjValidHoldUnitDelay_bool = OBF_AccObjValid_bool;
        } else {
            OBF_AccObjValidHoldUnitDelay_bool = rtb_AND_ojra;
        }

        /* End of Switch: '<S208>/Switch' */
    }

    /* End of Switch: '<S220>/Switch' */

    /* Switch: '<S551>/Switch2' incorporates:
     *  Constant: '<S573>/Constant'
     *  Constant: '<S574>/Constant'
     *  Constant: '<S584>/Constant'
     *  RelationalOperator: '<S551>/Equal'
     *  RelationalOperator: '<S551>/Equal1'
     *  RelationalOperator: '<S551>/Equal2'
     *  RelationalOperator: '<S552>/Equal1'
     *  Switch: '<S551>/Switch1'
     *  Switch: '<S551>/Switch3'
     *  Switch: '<S552>/Switch3'
     */
    if (((uint32_T)TJASTM_SysStateTJA_nu) ==
        E_TJASTM_SysStateTJA_nu_SYSST_OFF) {
        /* Switch: '<S551>/Switch2' incorporates:
         *  Constant: '<S586>/Constant'
         */
        TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_OFF;
    } else if (((uint32_T)TJASTM_SysStateTJA_nu) ==
               E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) {
        /* Switch: '<S551>/Switch2' incorporates:
         *  Constant: '<S587>/Constant'
         *  Switch: '<S551>/Switch1'
         */
        TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_Passive;
    } else if (((uint32_T)TJASTM_SysStateTJA_nu) ==
               E_TJASTM_SysStateTJA_nu_SYSST_STANDBY) {
        /* Switch: '<S551>/Switch2' incorporates:
         *  Constant: '<S588>/Constant'
         *  Switch: '<S551>/Switch1'
         *  Switch: '<S551>/Switch3'
         */
        TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_Standby;
    } else {
        /* Logic: '<S551>/OR3' incorporates:
         *  Constant: '<S551>/Constant1'
         *  Constant: '<S551>/Constant2'
         *  Constant: '<S551>/Constant3'
         *  Inport: '<Root>/Inport107'
         *  RelationalOperator: '<S551>/Equal6'
         *  RelationalOperator: '<S551>/Equal7'
         *  RelationalOperator: '<S551>/Equal8'
         *  Switch: '<S551>/Switch1'
         *  Switch: '<S551>/Switch3'
         */
        rtb_OR_lkl2 = (((((int32_T)VLCVEH_ACCStatus_nu) == 2) ||
                        (((int32_T)VLCVEH_ACCStatus_nu) == 4)) ||
                       (((int32_T)VLCVEH_ACCStatus_nu) == 5));

        /* Switch: '<S551>/Switch4' incorporates:
         *  Constant: '<S551>/Constant4'
         *  Constant: '<S551>/Constant5'
         *  Constant: '<S551>/Constant6'
         *  Constant: '<S575>/Constant'
         *  Constant: '<S577>/Constant'
         *  Constant: '<S579>/Constant'
         *  Constant: '<S582>/Constant'
         *  Constant: '<S583>/Constant'
         *  Constant: '<S585>/Constant'
         *  Constant: '<S590>/Constant'
         *  Inport: '<Root>/Inport107'
         *  Logic: '<S551>/OR1'
         *  Logic: '<S551>/OR2'
         *  Logic: '<S551>/OR4'
         *  Logic: '<S551>/OR5'
         *  Logic: '<S551>/OR6'
         *  Logic: '<S551>/OR7'
         *  Logic: '<S551>/OR8'
         *  RelationalOperator: '<S551>/Equal10'
         *  RelationalOperator: '<S551>/Equal11'
         *  RelationalOperator: '<S551>/Equal12'
         *  RelationalOperator: '<S551>/Equal13'
         *  RelationalOperator: '<S551>/Equal14'
         *  RelationalOperator: '<S551>/Equal15'
         *  RelationalOperator: '<S551>/Equal16'
         *  RelationalOperator: '<S551>/Equal3'
         *  RelationalOperator: '<S551>/Equal5'
         *  RelationalOperator: '<S551>/Equal9'
         *  Switch: '<S551>/Switch1'
         *  Switch: '<S551>/Switch3'
         *  Switch: '<S551>/Switch5'
         *  Switch: '<S551>/Switch6'
         *  Switch: '<S551>/Switch7'
         *  Switch: '<S551>/Switch8'
         */
        if (((((uint32_T)TJASTM_SysStateTJA_nu) ==
              E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) ||
             (((uint32_T)TJASTM_SysStateTJA_nu) ==
              E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT)) &&
            rtb_OR_lkl2) {
            /* Switch: '<S551>/Switch2' incorporates:
             *  Constant: '<S589>/Constant'
             *  Switch: '<S551>/Switch3'
             */
            TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_Active;
        } else if ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                    E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED) &&
                   rtb_OR_lkl2) {
            /* Switch: '<S551>/Switch5' incorporates:
             *  Constant: '<S591>/Constant'
             *  Switch: '<S551>/Switch2'
             */
            TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_LCCSuspend;
        } else if (((((uint32_T)TJASTM_SysStateTJA_nu) ==
                     E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) ||
                    (((uint32_T)TJASTM_SysStateTJA_nu) ==
                     E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT)) &&
                   (((int32_T)VLCVEH_ACCStatus_nu) == 3)) {
            /* Switch: '<S551>/Switch6' incorporates:
             *  Constant: '<S576>/Constant'
             *  Switch: '<S551>/Switch2'
             *  Switch: '<S551>/Switch5'
             */
            TJASTM_SysStateHWA_nu =
                E_TJASTM_SysStateHWA_nu_HWAState_ACCOverride;
        } else if ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                    E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED) &&
                   (((int32_T)VLCVEH_ACCStatus_nu) == 3)) {
            /* Switch: '<S551>/Switch7' incorporates:
             *  Constant: '<S578>/Constant'
             *  Switch: '<S551>/Switch2'
             *  Switch: '<S551>/Switch5'
             *  Switch: '<S551>/Switch6'
             */
            TJASTM_SysStateHWA_nu =
                E_TJASTM_SysStateHWA_nu_HWAState_BothSuspend;
        } else if ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                    E_TJASTM_SysStateTJA_nu_SYSST_ERROR) ||
                   (((int32_T)VLCVEH_ACCStatus_nu) == 6)) {
            /* Switch: '<S551>/Switch8' incorporates:
             *  Constant: '<S580>/Constant'
             *  Switch: '<S551>/Switch2'
             *  Switch: '<S551>/Switch5'
             *  Switch: '<S551>/Switch6'
             *  Switch: '<S551>/Switch7'
             */
            TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_Error;
        } else {
            /* Switch: '<S551>/Switch2' incorporates:
             *  Constant: '<S581>/Constant'
             *  Switch: '<S551>/Switch5'
             *  Switch: '<S551>/Switch6'
             *  Switch: '<S551>/Switch7'
             *  Switch: '<S551>/Switch8'
             */
            TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_Passive;
        }

        /* End of Switch: '<S551>/Switch4' */
    }

    /* End of Switch: '<S551>/Switch2' */

    /* Logic: '<S613>/AND8' incorporates:
     *  Constant: '<S628>/Constant'
     *  Constant: '<S629>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal10'
     *  RelationalOperator: '<S613>/Equal11'
     *  Switch: '<S552>/Switch3'
     */
    rtb_AND2_emk5 = ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED) &&
                     (((uint32_T)STM_SysStateUnitDelay_bool) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT));

    /* Switch: '<S660>/Switch' incorporates:
     *  Constant: '<S613>/Constant13'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S660>/Max'
     *  Sum: '<S660>/Subtract'
     *  Switch: '<S660>/Switch1'
     *  UnaryMinus: '<S660>/Unary Minus'
     *  UnitDelay: '<S660>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        TJATOW_PilotOverrideAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_PilotOverrideAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_PilotOverrideAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S660>/Switch' */

    /* Logic: '<S613>/AND9' incorporates:
     *  Constant: '<S630>/Constant'
     *  Constant: '<S631>/Constant'
     *  RelationalOperator: '<S552>/Equal1'
     *  RelationalOperator: '<S613>/Equal12'
     *  RelationalOperator: '<S613>/Equal13'
     *  Switch: '<S552>/Switch3'
     */
    rtb_OR_lkl2 = ((((uint32_T)TJASTM_SysStateTJA_nu) ==
                    E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING) &&
                   (((uint32_T)STM_SysStateUnitDelay_bool) ==
                    E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED));

    /* Switch: '<S661>/Switch' incorporates:
     *  Constant: '<S613>/Constant14'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S661>/Max'
     *  Sum: '<S661>/Subtract'
     *  Switch: '<S661>/Switch1'
     *  UnaryMinus: '<S661>/Unary Minus'
     *  UnitDelay: '<S661>/Unit Delay'
     */
    if (rtb_OR_lkl2) {
        TJATOW_PilotResumeAudioPlay_sec = TJATOW_NPilotAudioTime_C_sec;
    } else {
        TJATOW_PilotResumeAudioPlay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJATOW_PilotResumeAudioPlay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S661>/Switch' */

    /* Switch: '<S613>/Switch5' incorporates:
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S660>/OR'
     *  Logic: '<S661>/OR'
     *  RelationalOperator: '<S660>/GreaterThan'
     *  RelationalOperator: '<S661>/GreaterThan'
     *  Switch: '<S613>/Switch6'
     *  UnaryMinus: '<S660>/Unary Minus1'
     *  UnaryMinus: '<S661>/Unary Minus1'
     *  UnitDelay: '<S660>/Unit Delay'
     *  UnitDelay: '<S661>/Unit Delay'
     */
    if (rtb_AND2_emk5 ||
        (TJATOW_PilotOverrideAudioPlay_sec > (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch5' incorporates:
         *  Constant: '<S613>/Constant12'
         */
        TJASTM_PilotAudioPlay = 1U;
    } else if (rtb_OR_lkl2 || (TJATOW_PilotResumeAudioPlay_sec >
                               (-LCFRCV_TSysCycleTimeSen_sec))) {
        /* Switch: '<S613>/Switch6' incorporates:
         *  Constant: '<S613>/Constant15'
         *  Switch: '<S613>/Switch5'
         */
        TJASTM_PilotAudioPlay = 2U;
    } else {
        /* Switch: '<S613>/Switch5' incorporates:
         *  Constant: '<S613>/Constant16'
         *  Switch: '<S613>/Switch6'
         */
        TJASTM_PilotAudioPlay = 0U;
    }

    /* End of Switch: '<S613>/Switch5' */

    /* Switch: '<S604>/Switch2' incorporates:
     *  Constant: '<S600>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S600>/AND'
     *  Logic: '<S600>/NOT'
     *  RelationalOperator: '<S604>/GreaterThan'
     *  Switch: '<S604>/Switch'
     *  UnitDelay: '<S600>/Unit Delay'
     *  UnitDelay: '<S604>/Unit Delay'
     */
    if (rtb_NotEqual_ozi5_tmp_0 && (!GEN_PrevSus2UnitDelay_bool)) {
        GEN_Sus2TimeExpiredTimerRetrigger_sec = TJASTM_SuspendQuitTimeMax_C_sec;
    } else if (GEN_Sus2TimeExpiredTimerRetrigger_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S604>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S604>/Subtract'
         *  UnitDelay: '<S604>/Unit Delay'
         */
        GEN_Sus2TimeExpiredTimerRetrigger_sec =
            GEN_Sus2TimeExpiredTimerRetrigger_sec - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S604>/Unit Delay' incorporates:
         *  Constant: '<S604>/Constant1'
         *  Switch: '<S604>/Switch'
         */
        GEN_Sus2TimeExpiredTimerRetrigger_sec = 0.0F;
    }

    /* End of Switch: '<S604>/Switch2' */

    /* Switch: '<S603>/Switch' incorporates:
     *  Logic: '<S600>/NOT1'
     */
    if (!rtb_NotEqual_ozi5_tmp_0) {
        /* Switch: '<S603>/Switch' incorporates:
         *  Constant: '<S603>/Constant2'
         */
        STM_SuspendTimeExpired_bool = false;
    } else {
        /* Switch: '<S603>/Switch' incorporates:
         *  Constant: '<S604>/Constant2'
         *  Logic: '<S600>/NOT2'
         *  RelationalOperator: '<S604>/GreaterThan1'
         *  UnitDelay: '<S603>/Unit Delay'
         *  UnitDelay: '<S604>/Unit Delay'
         */
        STM_SuspendTimeExpired_bool =
            ((GEN_Sus2TimeExpiredTimerRetrigger_sec <= 0.0F) ||
             (GEN_Sus2TimeExpiredRSFF_bool));
    }

    /* End of Switch: '<S603>/Switch' */

    /* Logic: '<S546>/OR2' incorporates:
     *  Constant: '<S557>/Constant'
     *  Constant: '<S558>/Constant'
     *  Constant: '<S559>/Constant'
     *  Constant: '<S560>/Constant'
     *  Logic: '<S546>/OR'
     *  Logic: '<S546>/OR1'
     *  RelationalOperator: '<S546>/Equal1'
     *  RelationalOperator: '<S546>/Equal2'
     *  RelationalOperator: '<S546>/Equal3'
     *  RelationalOperator: '<S546>/Equal4'
     *  UnitDelay: '<S546>/Unit Delay1'
     */
    rtb_OR2_pwtt = (((((uint32_T)TJASTM_SysStateTJAIn_nu) ==
                      E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE) &&
                     (((E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ==
                        ((uint32_T)TJASTM_PrevSysStateTJAIn_envm)) ||
                       (((uint32_T)TJASTM_PrevSysStateTJAIn_envm) ==
                        E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED)) ||
                      (((uint32_T)TJASTM_PrevSysStateTJAIn_envm) ==
                       E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT))) &&
                    rtb_OR2_pwtt);

    /* Switch: '<S562>/Switch' incorporates:
     *  Constant: '<S546>/Constant1'
     *  DataTypeConversion: '<S546>/Data Type Conversion1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S562>/Max'
     *  Sum: '<S562>/Subtract'
     *  Switch: '<S562>/Switch1'
     *  UnaryMinus: '<S562>/Unary Minus'
     *  UnitDelay: '<S562>/Unit Delay'
     */
    if (rtb_OR2_pwtt) {
        TJASTM_HandsOffWarnTurnOffDelay_sec = TJASTM_HansOffReleaseWarn_C_sec;
    } else {
        TJASTM_HandsOffWarnTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec,
                  TJASTM_HandsOffWarnTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S562>/Switch' */

    /* DataTypeConversion: '<S546>/Data Type Conversion3' incorporates:
     *  DataTypeConversion: '<S546>/Data Type Conversion1'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S562>/OR'
     *  RelationalOperator: '<S562>/GreaterThan'
     *  UnaryMinus: '<S562>/Unary Minus1'
     *  UnitDelay: '<S562>/Unit Delay'
     */
    TJASTM_LatCtrlHandsOffReleaseWarn_nu =
        (uint8_T)((rtb_OR2_pwtt || (TJASTM_HandsOffWarnTurnOffDelay_sec >
                                    (-LCFRCV_TSysCycleTimeSen_sec)))
                      ? 1
                      : 0);

    /* Switch: '<S561>/Switch' incorporates:
     *  Constant: '<S546>/Constant7'
     *  DataTypeConversion: '<S546>/Data Type Conversion1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S561>/Max'
     *  Sum: '<S561>/Subtract'
     *  Switch: '<S561>/Switch1'
     *  UnaryMinus: '<S561>/Unary Minus'
     *  UnitDelay: '<S561>/Unit Delay'
     */
    if (rtb_OR2_pwtt) {
        TJASTM_ACCOffTurnOffDelay_sec = TJASTM_ACCManeveurTime_C_sec;
    } else {
        TJASTM_ACCOffTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, TJASTM_ACCOffTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S561>/Switch' */

    /* DataTypeConversion: '<S546>/Data Type Conversion2' incorporates:
     *  DataTypeConversion: '<S546>/Data Type Conversion1'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S561>/OR'
     *  RelationalOperator: '<S561>/GreaterThan'
     *  UnaryMinus: '<S561>/Unary Minus1'
     *  UnitDelay: '<S561>/Unit Delay'
     */
    TJASTM_PilotDisableACCSwitch_nu =
        (uint8_T)((rtb_OR2_pwtt || (TJASTM_ACCOffTurnOffDelay_sec >
                                    (-LCFRCV_TSysCycleTimeSen_sec)))
                      ? 1
                      : 0);

    /* RelationalOperator: '<S546>/Equal' incorporates:
     *  Constant: '<S556>/Constant'
     */
    rtb_AND2_emk5 = (((uint32_T)TJASTM_SysStateTJAIn_nu) ==
                     E_TJASTM_SysStateTJA_nu_SYSST_WAITACC);

    /* DataTypeConversion: '<S546>/Data Type Conversion' */
    TJASTM_PilotEnableACCSwitch_nu =
        rtb_AND2_emk5 ? ((uint8_T)1) : ((uint8_T)0);

    /* Switch: '<S563>/Switch' incorporates:
     *  Constant: '<S546>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S563>/Max'
     *  Sum: '<S563>/Subtract'
     *  Switch: '<S563>/Switch1'
     *  UnaryMinus: '<S563>/Unary Minus'
     *  UnitDelay: '<S563>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        TJASTM_ACCActiveOvertime_sec =
            fmaxf(TJASTM_ACCActiveOvertime_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        TJASTM_ACCActiveOvertime_sec = TJASTM_ACCOvertime_C_sec;
    }

    /* End of Switch: '<S563>/Switch' */

    /* Logic: '<S563>/AND' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S563>/LessThanOrEqual'
     *  UnaryMinus: '<S563>/Unary Minus1'
     *  UnitDelay: '<S563>/Unit Delay'
     *  UnitDelay: '<S8>/Unit Delay1'
     */
    TJASTM_PrevACCActOvrtm_bool =
        (rtb_AND2_emk5 &&
         (TJASTM_ACCActiveOvertime_sec <= (-LCFRCV_TSysCycleTimeSen_sec)));

    /* DataTypeConversion: '<S525>/Data Type Conversion1' */
    TJASLC_TriggerInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_jofi;

    /* Logic: '<S375>/OR3' incorporates:
     *  Constant: '<S412>/Constant'
     *  Constant: '<S426>/Constant'
     *  RelationalOperator: '<S375>/Equal1'
     *  RelationalOperator: '<S375>/Equal5'
     */
    rtb_AND2_emk5 = ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
                      E_TJASLC_ManeuverState_nu_PASSIVE) ||
                     (((uint32_T)SLC_PrevManeuverState2_Enum) ==
                      E_TJASLC_ManeuverState_nu_TRIGREADY));

    /* Switch: '<S432>/Switch' incorporates:
     *  Constant: '<S375>/Constant12'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S432>/Max'
     *  Sum: '<S432>/Subtract'
     *  Switch: '<S432>/Switch1'
     *  UnaryMinus: '<S432>/Unary Minus'
     *  UnitDelay: '<S432>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        SLC_LCWPassiveTurnOnDly_sec =
            fmaxf(SLC_LCWPassiveTurnOnDly_sec, -LCFRCV_TSysCycleTimeSen_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    } else {
        SLC_LCWPassiveTurnOnDly_sec = TJASLC_LCWPassiveDlyTm_C_sec;
    }

    /* End of Switch: '<S432>/Switch' */

    /* Switch: '<S375>/Switch28' incorporates:
     *  Constant: '<S413>/Constant'
     *  Constant: '<S418>/Constant'
     *  Constant: '<S425>/Constant'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S375>/OR1'
     *  Logic: '<S375>/OR2'
     *  Logic: '<S432>/AND'
     *  RelationalOperator: '<S375>/Equal2'
     *  RelationalOperator: '<S375>/Equal7'
     *  RelationalOperator: '<S432>/LessThanOrEqual'
     *  UnaryMinus: '<S432>/Unary Minus1'
     *  UnitDelay: '<S432>/Unit Delay'
     */
    if (((((uint32_T)SLC_PrevManeuverState2_Enum) ==
          E_TJASLC_ManeuverState_nu_LCPSTART) ||
         (rtb_AND2_emk5 &&
          (SLC_LCWPassiveTurnOnDly_sec <= (-LCFRCV_TSysCycleTimeSen_sec)))) &&
        (((uint32_T)STM_SysStateUnitDelay_bool) ==
         E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING)) {
        if ((rtb_AND_psoa || rtb_SLC_ObjectDetectedLeftRear_) &&
            (SLC_LeverLeftEngaged_bool)) {
            /* Switch: '<S375>/Switch28' incorporates:
             *  Constant: '<S427>/Constant'
             *  Switch: '<S375>/Switch1'
             */
            rtb_Switch28 = E_TJASLC_LaneChangeWarning_nu_LEFT_WARNING;

            /* Switch: '<S375>/Switch5' incorporates:
             *  Switch: '<S375>/Switch28'
             *  UnitDelay: '<S375>/Unit Delay'
             */
            SLC_LaneChangeWarnUnitDy_Enum = rtb_Switch28;
        } else if ((rtb_AND_psoa || rtb_SLC_ObjectDetectedRightRear) &&
                   (SLC_LeverRightEngaged_bool)) {
            /* Switch: '<S375>/Switch2' incorporates:
             *  Constant: '<S428>/Constant'
             *  Switch: '<S375>/Switch1'
             *  Switch: '<S375>/Switch28'
             */
            rtb_Switch28 = E_TJASLC_LaneChangeWarning_nu_RIGHT_WARNING;

            /* Switch: '<S375>/Switch5' incorporates:
             *  Switch: '<S375>/Switch28'
             *  UnitDelay: '<S375>/Unit Delay'
             */
            SLC_LaneChangeWarnUnitDy_Enum = rtb_Switch28;
        } else {
            /* Switch: '<S375>/Switch28' incorporates:
             *  Constant: '<S414>/Constant'
             *  Switch: '<S375>/Switch1'
             *  Switch: '<S375>/Switch2'
             */
            rtb_Switch28 = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
        }
    } else {
        rtb_Switch28 = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
    }

    /* End of Switch: '<S375>/Switch28' */

    /* DataTypeConversion: '<S375>/Data Type Conversion1' */
    rtb_OR_lkl2 = (((int32_T)rtb_Switch28) != 0);

    /* Switch: '<S429>/Switch2' incorporates:
     *  Constant: '<S375>/Constant2'
     *  Inport: '<Root>/Inport10'
     *  Logic: '<S409>/AND'
     *  Logic: '<S409>/NOT'
     *  RelationalOperator: '<S429>/GreaterThan'
     *  Switch: '<S429>/Switch'
     *  UnitDelay: '<S409>/Unit Delay'
     *  UnitDelay: '<S429>/Unit Delay'
     */
    if (rtb_OR_lkl2 && (!SLC_LaneChangeWarnEdgeRising_bool)) {
        SLC_LaneChangeWarnTurnOffDelay_sec = TJASLC_LaneChangeWarnTimeMax_C_sec;
    } else if (SLC_LaneChangeWarnTurnOffDelay_sec >
               LCFRCV_TSysCycleTimeSen_sec) {
        /* Switch: '<S429>/Switch' incorporates:
         *  Inport: '<Root>/Inport10'
         *  Sum: '<S429>/Subtract'
         *  UnitDelay: '<S429>/Unit Delay'
         */
        SLC_LaneChangeWarnTurnOffDelay_sec =
            SLC_LaneChangeWarnTurnOffDelay_sec - LCFRCV_TSysCycleTimeSen_sec;
    } else {
        /* UnitDelay: '<S429>/Unit Delay' incorporates:
         *  Constant: '<S429>/Constant1'
         *  Switch: '<S429>/Switch'
         */
        SLC_LaneChangeWarnTurnOffDelay_sec = 0.0F;
    }

    /* End of Switch: '<S429>/Switch2' */

    /* Logic: '<S411>/AND' incorporates:
     *  Logic: '<S411>/NOT'
     *  UnitDelay: '<S411>/Unit Delay'
     */
    rtb_AND2_emk5 = (rtb_SLC_RearAbort_bool && (!SLC_RearAbortEdgRs_bool));

    /* Switch: '<S375>/Switch26' incorporates:
     *  Constant: '<S375>/Constant8'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S431>/Max'
     *  Sum: '<S431>/Subtract'
     *  Switch: '<S431>/Switch'
     *  Switch: '<S431>/Switch1'
     *  UnaryMinus: '<S431>/Unary Minus'
     *  UnitDelay: '<S431>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        /* Switch: '<S375>/Switch24' incorporates:
         *  Logic: '<S375>/AND10'
         *  Logic: '<S375>/AND11'
         *  Switch: '<S375>/Switch25'
         */
        if (rtb_SLC_ObjectDetectedLeftRear_ && (SLC_LeverLeftEngaged_bool)) {
            /* Switch: '<S375>/Switch26' incorporates:
             *  Constant: '<S423>/Constant'
             *  UnitDelay: '<S375>/Unit Delay3'
             */
            SLC_PrevReaAbortWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_LEFT_WARNING;
        } else if (rtb_SLC_ObjectDetectedRightRear &&
                   (SLC_LeverRightEngaged_bool)) {
            /* Switch: '<S375>/Switch25' incorporates:
             *  Constant: '<S424>/Constant'
             *  Switch: '<S375>/Switch26'
             *  UnitDelay: '<S375>/Unit Delay3'
             */
            SLC_PrevReaAbortWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_RIGHT_WARNING;
        } else {
            /* Switch: '<S375>/Switch26' incorporates:
             *  Constant: '<S422>/Constant'
             *  Switch: '<S375>/Switch25'
             *  UnitDelay: '<S375>/Unit Delay3'
             */
            SLC_PrevReaAbortWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
        }

        /* End of Switch: '<S375>/Switch24' */
        SLC_RearAbortTunOffDly_sec = TJASLC_LaneChangeWarnTimeMax_C_sec;
    } else {
        SLC_RearAbortTunOffDly_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, SLC_RearAbortTunOffDly_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S375>/Switch26' */

    /* Logic: '<S431>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S431>/GreaterThan'
     *  UnaryMinus: '<S431>/Unary Minus1'
     *  UnitDelay: '<S431>/Unit Delay'
     */
    rtb_AND_ojra = (rtb_AND2_emk5 || (SLC_RearAbortTunOffDly_sec >
                                      (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S410>/AND' incorporates:
     *  Logic: '<S410>/NOT'
     *  UnitDelay: '<S410>/Unit Delay'
     */
    rtb_AND2_emk5 = (rtb_AND2 && (!SLC_RearCancleEdgRs_bool));

    /* Switch: '<S375>/Switch20' incorporates:
     *  Constant: '<S375>/Constant1'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S430>/Max'
     *  Sum: '<S430>/Subtract'
     *  Switch: '<S430>/Switch'
     *  Switch: '<S430>/Switch1'
     *  UnaryMinus: '<S430>/Unary Minus'
     *  UnitDelay: '<S430>/Unit Delay'
     */
    if (rtb_AND2_emk5) {
        /* Switch: '<S375>/Switch21' incorporates:
         *  Logic: '<S375>/AND8'
         *  Logic: '<S375>/AND9'
         *  Switch: '<S375>/Switch22'
         */
        if (rtb_SLC_ObjectDetectedLeftRear_ && (SLC_LeverLeftEngaged_bool)) {
            /* Switch: '<S375>/Switch20' incorporates:
             *  Constant: '<S419>/Constant'
             *  UnitDelay: '<S375>/Unit Delay2'
             */
            SLC_PrevRearCancleWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_LEFT_WARNING;
        } else if (rtb_SLC_ObjectDetectedLeftRear_ &&
                   (SLC_LeverRightEngaged_bool)) {
            /* Switch: '<S375>/Switch22' incorporates:
             *  Constant: '<S420>/Constant'
             *  Switch: '<S375>/Switch20'
             *  UnitDelay: '<S375>/Unit Delay2'
             */
            SLC_PrevRearCancleWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_RIGHT_WARNING;
        } else {
            /* Switch: '<S375>/Switch20' incorporates:
             *  Constant: '<S421>/Constant'
             *  Switch: '<S375>/Switch22'
             *  UnitDelay: '<S375>/Unit Delay2'
             */
            SLC_PrevRearCancleWarnSide_enum =
                E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
        }

        /* End of Switch: '<S375>/Switch21' */
        SLC_RearCancleTunOffDly_sec = TJASLC_LaneChangeWarnTimeMax_C_sec;
    } else {
        SLC_RearCancleTunOffDly_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, SLC_RearCancleTunOffDly_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S375>/Switch20' */

    /* Logic: '<S430>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S430>/GreaterThan'
     *  UnaryMinus: '<S430>/Unary Minus1'
     *  UnitDelay: '<S430>/Unit Delay'
     */
    rtb_OR2_pwtt = (rtb_AND2_emk5 || (SLC_RearCancleTunOffDly_sec >
                                      (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Switch: '<S375>/Switch19' incorporates:
     *  Constant: '<S416>/Constant'
     *  Constant: '<S429>/Constant2'
     *  RelationalOperator: '<S375>/Equal14'
     *  RelationalOperator: '<S429>/GreaterThan1'
     *  Switch: '<S375>/Switch27'
     *  Switch: '<S375>/Switch4'
     *  Switch: '<S375>/Switch6'
     *  UnitDelay: '<S429>/Unit Delay'
     */
    if (rtb_OR2_pwtt) {
        /* DataTypeConversion: '<S375>/Data Type Conversion' incorporates:
         *  Switch: '<S375>/Switch20'
         *  UnitDelay: '<S375>/Unit Delay2'
         */
        TJASLC_LaneChangWarning_nu = (uint8_T)SLC_PrevRearCancleWarnSide_enum;
    } else if (rtb_AND_ojra) {
        /* Switch: '<S375>/Switch27' incorporates:
         *  DataTypeConversion: '<S375>/Data Type Conversion'
         *  Switch: '<S375>/Switch26'
         *  UnitDelay: '<S375>/Unit Delay3'
         */
        TJASLC_LaneChangWarning_nu = (uint8_T)SLC_PrevReaAbortWarnSide_enum;
    } else if (SLC_LaneChangeWarnTurnOffDelay_sec > 0.0F) {
        /* Switch: '<S375>/Switch4' incorporates:
         *  DataTypeConversion: '<S375>/Data Type Conversion'
         *  Switch: '<S375>/Switch27'
         *  Switch: '<S375>/Switch5'
         *  UnitDelay: '<S375>/Unit Delay'
         */
        TJASLC_LaneChangWarning_nu = (uint8_T)SLC_LaneChangeWarnUnitDy_Enum;
    } else if (((uint32_T)rtb_Switch28) ==
               E_TJASLC_LaneChangeWarning_nu_NO_WARNING) {
        /* Switch: '<S375>/Switch6' incorporates:
         *  Constant: '<S417>/Constant'
         *  DataTypeConversion: '<S375>/Data Type Conversion'
         *  Switch: '<S375>/Switch27'
         *  Switch: '<S375>/Switch4'
         */
        TJASLC_LaneChangWarning_nu = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
    } else {
        /* DataTypeConversion: '<S375>/Data Type Conversion' */
        TJASLC_LaneChangWarning_nu = (uint8_T)rtb_Switch28;
    }

    /* End of Switch: '<S375>/Switch19' */

    /* Switch: '<S375>/Switch3' incorporates:
     *  Constant: '<S375>/Constant10'
     *  Constant: '<S375>/Constant9'
     *  RelationalOperator: '<S375>/Equal3'
     *  RelationalOperator: '<S375>/Equal4'
     *  Switch: '<S375>/Switch35'
     */
    if (1 == ((int32_T)TJASLC_LaneChangWarning_nu)) {
        /* Switch: '<S375>/Switch30' incorporates:
         *  Switch: '<S375>/Switch31'
         *  Switch: '<S375>/Switch33'
         *  Switch: '<S375>/Switch34'
         */
        if (rtb_OR2_pwtt) {
            /* Switch: '<S375>/Switch29' */
            if (rtb_AND_psoa) {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  Inport: '<Root>/Inport117'
                 */
                TJASLC_SLCHighLightID_nu = LCFRCV_inFrontDangerObjID_nu;
            } else {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  UnitDelay: '<S375>/Unit Delay1'
                 */
                TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
            }

            /* End of Switch: '<S375>/Switch29' */
        } else if (rtb_AND_ojra) {
            /* Switch: '<S375>/Switch32' incorporates:
             *  Switch: '<S375>/Switch31'
             */
            if (rtb_SLC_ObjectDetectedLeftRear_) {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  Inport: '<Root>/Inport109'
                 */
                TJASLC_SLCHighLightID_nu = LCCRA_InLeftHighLightID_nu;
            } else {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  UnitDelay: '<S375>/Unit Delay1'
                 */
                TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
            }

            /* End of Switch: '<S375>/Switch32' */
        } else if (rtb_SLC_ObjectDetectedLeftRear_) {
            /* Switch: '<S375>/Switch34' incorporates:
             *  Inport: '<Root>/Inport109'
             *  Switch: '<S375>/Switch3'
             *  Switch: '<S375>/Switch31'
             */
            TJASLC_SLCHighLightID_nu = LCCRA_InLeftHighLightID_nu;
        } else if (rtb_AND_psoa) {
            /* Switch: '<S375>/Switch33' incorporates:
             *  Inport: '<Root>/Inport117'
             *  Switch: '<S375>/Switch3'
             *  Switch: '<S375>/Switch31'
             *  Switch: '<S375>/Switch34'
             */
            TJASLC_SLCHighLightID_nu = LCFRCV_inFrontDangerObjID_nu;
        } else {
            /* Switch: '<S375>/Switch3' incorporates:
             *  Switch: '<S375>/Switch31'
             *  Switch: '<S375>/Switch34'
             *  UnitDelay: '<S375>/Unit Delay1'
             */
            TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
        }

        /* End of Switch: '<S375>/Switch30' */
    } else if (2 == ((int32_T)TJASLC_LaneChangWarning_nu)) {
        /* Switch: '<S375>/Switch37' incorporates:
         *  Switch: '<S375>/Switch35'
         *  Switch: '<S375>/Switch38'
         *  Switch: '<S375>/Switch40'
         *  Switch: '<S375>/Switch41'
         */
        if (rtb_OR2_pwtt) {
            /* Switch: '<S375>/Switch36' */
            if (rtb_AND_psoa) {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  Inport: '<Root>/Inport117'
                 */
                TJASLC_SLCHighLightID_nu = LCFRCV_inFrontDangerObjID_nu;
            } else {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  UnitDelay: '<S375>/Unit Delay1'
                 */
                TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
            }

            /* End of Switch: '<S375>/Switch36' */
        } else if (rtb_AND_ojra) {
            /* Switch: '<S375>/Switch39' incorporates:
             *  Switch: '<S375>/Switch38'
             */
            if (rtb_SLC_ObjectDetectedRightRear) {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  Inport: '<Root>/Inport110'
                 */
                TJASLC_SLCHighLightID_nu = LCCRA_InRightHighLightID_nu;
            } else {
                /* Switch: '<S375>/Switch3' incorporates:
                 *  UnitDelay: '<S375>/Unit Delay1'
                 */
                TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
            }

            /* End of Switch: '<S375>/Switch39' */
        } else if (rtb_SLC_ObjectDetectedRightRear) {
            /* Switch: '<S375>/Switch41' incorporates:
             *  Inport: '<Root>/Inport110'
             *  Switch: '<S375>/Switch3'
             *  Switch: '<S375>/Switch38'
             */
            TJASLC_SLCHighLightID_nu = LCCRA_InRightHighLightID_nu;
        } else if (rtb_AND_psoa) {
            /* Switch: '<S375>/Switch40' incorporates:
             *  Inport: '<Root>/Inport117'
             *  Switch: '<S375>/Switch3'
             *  Switch: '<S375>/Switch38'
             *  Switch: '<S375>/Switch41'
             */
            TJASLC_SLCHighLightID_nu = LCFRCV_inFrontDangerObjID_nu;
        } else {
            /* Switch: '<S375>/Switch3' incorporates:
             *  Switch: '<S375>/Switch38'
             *  Switch: '<S375>/Switch41'
             *  UnitDelay: '<S375>/Unit Delay1'
             */
            TJASLC_SLCHighLightID_nu = TJASLC_PrevSLCHighLightID_nu;
        }

        /* End of Switch: '<S375>/Switch37' */
    } else {
        /* Switch: '<S375>/Switch3' incorporates:
         *  Constant: '<S375>/Constant11'
         *  Switch: '<S375>/Switch35'
         */
        TJASLC_SLCHighLightID_nu = -1;
    }

    /* End of Switch: '<S375>/Switch3' */

    /* Logic: '<S322>/OR' incorporates:
     *  Constant: '<S455>/Constant'
     *  Constant: '<S456>/Constant'
     *  RelationalOperator: '<S322>/Equal'
     *  RelationalOperator: '<S322>/Equal1'
     */
    rtb_AND_psoa = ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
                     E_TJASLC_ManeuverState_nu_LCPSTART) &&
                    (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                     E_TJASLC_ManeuverState_nu_TRIGREADY));

    /* Switch: '<S322>/Switch' incorporates:
     *  Constant: '<S457>/Constant'
     *  Constant: '<S458>/Constant'
     *  Constant: '<S459>/Constant'
     *  Constant: '<S460>/Constant'
     *  Logic: '<S322>/OR1'
     *  Logic: '<S322>/OR2'
     *  Logic: '<S322>/OR3'
     *  Logic: '<S322>/OR4'
     *  RelationalOperator: '<S322>/Equal2'
     *  RelationalOperator: '<S322>/Equal3'
     *  RelationalOperator: '<S322>/Equal4'
     *  RelationalOperator: '<S322>/Equal5'
     *  Switch: '<S322>/Switch1'
     *  Switch: '<S322>/Switch2'
     */
    if (rtb_AND_psoa && (SLC_LeverLeftEngaged_bool)) {
        /* Switch: '<S322>/Switch' incorporates:
         *  Constant: '<S322>/Constant1'
         */
        TJASLC_SLCAudioPlay_nu = 1U;
    } else if (rtb_AND_psoa && (SLC_LeverRightEngaged_bool)) {
        /* Switch: '<S322>/Switch1' incorporates:
         *  Constant: '<S322>/Constant2'
         *  Switch: '<S322>/Switch'
         */
        TJASLC_SLCAudioPlay_nu = 2U;
    } else if ((((uint32_T)SLC_PrevManeuverState2_Enum) ==
                E_TJASLC_ManeuverState_nu_PASSIVE) &&
               (((((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                  E_TJASLC_ManeuverState_nu_LATMVSTART) ||
                 (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                  E_TJASLC_ManeuverState_nu_LCMSTART)) ||
                (((uint32_T)SLC_PrevManeuverStateGRCO_Enum) ==
                 E_TJASLC_ManeuverState_nu_ABORT))) {
        /* Switch: '<S322>/Switch2' incorporates:
         *  Constant: '<S322>/Constant3'
         *  Switch: '<S322>/Switch'
         *  Switch: '<S322>/Switch1'
         */
        TJASLC_SLCAudioPlay_nu = 3U;
    } else {
        /* Switch: '<S322>/Switch' incorporates:
         *  Constant: '<S322>/Constant4'
         *  Switch: '<S322>/Switch1'
         *  Switch: '<S322>/Switch2'
         */
        TJASLC_SLCAudioPlay_nu = 0U;
    }

    /* Logic: '<S515>/OR' incorporates:
     *  UnitDelay: '<S268>/Unit Delay1'
     *  UnitDelay: '<S268>/Unit Delay2'
     */
    rtb_OR_ly14 = ((((SLC_PrevDriverTrigResetLeftUnitDelay_bool) ||
                     (SLC_PrevDriverTrigResetRightUnitDelay_bool)) ||
                    (SLC_ManvStatePassive_bool)) ||
                   (SLC_MaxInitDurationExceeded_bool));

    /* Switch: '<S528>/Switch' incorporates:
     *  Constant: '<S515>/Constant'
     *  Inport: '<Root>/Inport10'
     *  MinMax: '<S528>/Max'
     *  Sum: '<S528>/Subtract'
     *  Switch: '<S528>/Switch1'
     *  UnaryMinus: '<S528>/Unary Minus'
     *  UnitDelay: '<S528>/Unit Delay'
     */
    if (rtb_OR_ly14) {
        SLC_ResetTurnOffDelay_sec = TJASLC_TrigResetBlockTime_C_sec;
    } else {
        SLC_ResetTurnOffDelay_sec =
            fmaxf(-LCFRCV_TSysCycleTimeSen_sec, SLC_ResetTurnOffDelay_sec) -
            LCFRCV_TSysCycleTimeSen_sec;
    }

    /* End of Switch: '<S528>/Switch' */

    /* Logic: '<S528>/OR' incorporates:
     *  Inport: '<Root>/Inport10'
     *  RelationalOperator: '<S528>/GreaterThan'
     *  UnaryMinus: '<S528>/Unary Minus1'
     *  UnitDelay: '<S268>/Unit Delay'
     *  UnitDelay: '<S528>/Unit Delay'
     */
    SLC_PrevResetUnitDelay_bool =
        (rtb_OR_ly14 ||
         (SLC_ResetTurnOffDelay_sec > (-LCFRCV_TSysCycleTimeSen_sec)));

    /* Logic: '<S479>/AND' */
    rtb_AND_psoa = (rtb_OR_idx_1 && rtb_SLC_LeAdjLaneWidthValid_boo);

    /* Switch: '<S479>/Switch' incorporates:
     *  Logic: '<S479>/AND2'
     *  Switch: '<S479>/Switch1'
     *  Switch: '<S479>/Switch2'
     */
    if (rtb_AND_psoa && rtb_NotEqual_ozi5_tmp) {
        /* Switch: '<S479>/Switch' incorporates:
         *  Constant: '<S480>/Constant'
         */
        TJASLC_ReadyToTrigger_nu = E_TJASLC_ReadyToTrigger_nu_TRIGREADY_BOTH;
    } else if (rtb_AND_psoa) {
        /* Switch: '<S479>/Switch1' incorporates:
         *  Constant: '<S481>/Constant'
         *  Switch: '<S479>/Switch'
         */
        TJASLC_ReadyToTrigger_nu = E_TJASLC_ReadyToTrigger_nu_TRIGREADY_LEFT;
    } else if (rtb_NotEqual_ozi5_tmp) {
        /* Switch: '<S479>/Switch2' incorporates:
         *  Constant: '<S482>/Constant'
         *  Switch: '<S479>/Switch'
         *  Switch: '<S479>/Switch1'
         */
        TJASLC_ReadyToTrigger_nu = E_TJASLC_ReadyToTrigger_nu_TRIGREADY_RIGHT;
    } else {
        /* Switch: '<S479>/Switch' incorporates:
         *  Constant: '<S483>/Constant'
         *  Switch: '<S479>/Switch1'
         *  Switch: '<S479>/Switch2'
         */
        TJASLC_ReadyToTrigger_nu = E_TJASLC_ReadyToTrigger_nu_TRIGREADY_NONE;
    }

    /* End of Switch: '<S479>/Switch' */

    /* DataTypeConversion: '<S294>/Data Type Conversion1' */
    TJASLC_CancelAbort_btf = (uint8_T)rtb_ex_sfun_set_bit_gaa0;

    /* DataTypeConversion: '<S469>/Data Type Conversion1' */
    TJASLC_RiLaneChangeInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_d0a5;

    /* DataTypeConversion: '<S468>/Data Type Conversion1' */
    TJASLC_LeLaneChangeInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_m25k;

    /* Switch: '<S516>/Switch' incorporates:
     *  Switch: '<S516>/Switch1'
     */
    if (rtb_Switch1_eo0n_idx_0) {
        /* RelationalOperator: '<S536>/Equal' incorporates:
         *  Constant: '<S538>/Constant'
         *  Switch: '<S516>/Switch'
         *  UnitDelay: '<S516>/Unit Delay1'
         */
        SLC_PrevLaneChangeTrigger_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG;
    } else if (rtb_Switch1_eo0n_idx_1) {
        /* RelationalOperator: '<S536>/Equal' incorporates:
         *  Constant: '<S539>/Constant'
         *  Switch: '<S516>/Switch'
         *  Switch: '<S516>/Switch1'
         *  UnitDelay: '<S516>/Unit Delay1'
         */
        SLC_PrevLaneChangeTrigger_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG;
    } else {
        /* RelationalOperator: '<S536>/Equal' incorporates:
         *  Constant: '<S537>/Constant'
         *  Switch: '<S516>/Switch'
         *  Switch: '<S516>/Switch1'
         *  UnitDelay: '<S516>/Unit Delay1'
         */
        SLC_PrevLaneChangeTrigger_nu =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    }

    /* End of Switch: '<S516>/Switch' */

    /* Switch: '<S516>/Switch2' */
    if (rtb_NOT_auv4) {
        /* Switch: '<S516>/Switch2' incorporates:
         *  Switch: '<S516>/Switch'
         *  UnitDelay: '<S516>/Unit Delay1'
         */
        TJASLC_LaneChangeTrig_nu = SLC_PrevLaneChangeTrigger_nu;
    } else {
        /* Switch: '<S516>/Switch2' incorporates:
         *  Constant: '<S540>/Constant'
         */
        TJASLC_LaneChangeTrig_nu = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    }

    /* End of Switch: '<S516>/Switch2' */

    /* DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
     *  Switch: '<S516>/Switch2'
     */
    TJASLC_TurnLtDirctionReq_nu = (uint8_T)TJASLC_LaneChangeTrig_nu;

    /* Switch: '<S377>/Switch' incorporates:
     *  Constant: '<S377>/Constant2'
     *  Switch: '<S377>/Switch1'
     *  UnitDelay: '<S377>/Unit Delay'
     */
    if (SLC_LeverLeftEngaged_bool) {
        TJASLC_RSRightLaneChangeWarn_nu = false;
    } else {
        TJASLC_RSRightLaneChangeWarn_nu =
            ((SLC_LeverRightEngaged_bool) || (TJASLC_RSRightLaneChangeWarn_nu));
    }

    /* End of Switch: '<S377>/Switch' */

    /* Switch: '<S376>/Switch' incorporates:
     *  Constant: '<S376>/Constant2'
     *  Switch: '<S376>/Switch1'
     *  UnitDelay: '<S376>/Unit Delay'
     */
    if (SLC_LeverRightEngaged_bool) {
        TJASLC_RSLeftLaneChangeWarn_nu = false;
    } else {
        TJASLC_RSLeftLaneChangeWarn_nu =
            ((SLC_LeverLeftEngaged_bool) || (TJASLC_RSLeftLaneChangeWarn_nu));
    }

    /* End of Switch: '<S376>/Switch' */

    /* DataTypeConversion: '<S255>/Data Type Conversion1' */
    TJAOBF_TgtObjDataInvalid_btf = (uint8_T)rtb_ex_sfun_set_bit_nkl2;

    /* SignalConversion: '<S5>/Signal Conversion' */
    TJAOBF_TgtObjDataValid_bool = OBF_TargetObjDataSR_bool;

    /* DataTypeConversion: '<S218>/Data Type Conversion1' */
    TJAOBF_ObjInLaneInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit_hcfw;

    /* DataTypeConversion: '<S181>/Data Type Conversion1' */
    TJALKA_LnQualityInv_btf = (uint8_T)rtb_ex_sfun_set_bit_fv5m;

    /* DataTypeConversion: '<S136>/Data Type Conversion1' */
    TJALKA_LaneCenterInvalid_btf = (uint16_T)rtb_ex_sfun_set_bit;

    /* Update for UnitDelay: '<S12>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport9'
     */
    CMB_PrevCntrCrv1UnitDelay_1pm = ABPR_CntrLnClthCrv_1pm;

    /* Update for UnitDelay: '<S12>/Unit Delay1' */
    CMB_PrevCntrCrv2UnitDelay_1pm = rtb_UnitDelay;

    /* Update for UnitDelay: '<S12>/Unit Delay2' */
    CMB_PrevCntrCrv3UnitDelay_1pm = rtb_UnitDelay1;

    /* Update for UnitDelay: '<S12>/Unit Delay3' */
    CMB_PrevCntrCrv4UnitDelay_1pm = rtb_UnitDelay2;

    /* Update for UnitDelay: '<S12>/Unit Delay4' */
    CMB_PrevCntrCrv5UnitDelay_1pm = rtb_UnitDelay3;

    /* Update for UnitDelay: '<S12>/Unit Delay5' */
    CMB_PrevCntrCrv6UnitDelay_1pm = rtb_UnitDelay4;

    /* Update for UnitDelay: '<S12>/Unit Delay6' */
    CMB_PrevCntrCrv7UnitDelay_1pm = rtb_UnitDelay5;

    /* Update for UnitDelay: '<S12>/Unit Delay7' */
    CMB_PrevCntrCrv8UnitDelay_1pm = rtb_UnitDelay6;

    /* Update for UnitDelay: '<S12>/Unit Delay8' */
    CMB_PrevCntrCrv9UnitDelay_1pm = rtb_UnitDelay7;

    /* Update for UnitDelay: '<S13>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport38'
     */
    CMB_PrevTgtCrv1UnitDelay_1pm = ODPFOH_TgtObjCrv_1pm;

    /* Update for UnitDelay: '<S13>/Unit Delay1' */
    CMB_PrevTgtCrv2UnitDelay_1pm = rtb_UnitDelay_h1k2;

    /* Update for UnitDelay: '<S13>/Unit Delay2' */
    CMB_PrevTgtCrv3UnitDelay_1pm = rtb_UnitDelay1_ppcw;

    /* Update for UnitDelay: '<S13>/Unit Delay3' */
    CMB_PrevTgtCrv4UnitDelay_1pm = rtb_UnitDelay2_gjo3;

    /* Update for UnitDelay: '<S13>/Unit Delay4' */
    CMB_PrevTgtCrv5UnitDelay_1pm = rtb_UnitDelay3_ikos;

    /* Update for UnitDelay: '<S13>/Unit Delay5' */
    CMB_PrevTgtCrv6UnitDelay_1pm = rtb_UnitDelay4_jffl;

    /* Update for UnitDelay: '<S13>/Unit Delay6' */
    CMB_PrevTgtCrv7UnitDelay_1pm = rtb_UnitDelay5_bx0r;

    /* Update for UnitDelay: '<S13>/Unit Delay7' */
    CMB_PrevTgtCrv8UnitDelay_1pm = rtb_UnitDelay6_e5dw;

    /* Update for UnitDelay: '<S13>/Unit Delay8' */
    CMB_PrevTgtCrv9UnitDelay_1pm = rtb_UnitDelay7_gp10;

    /* Update for UnitDelay: '<S888>/Unit Delay' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     */
    SLC_PrevLatCtrlMdSLC2LCC_Enum = TJASTM_LatCtrlMode_nu;

    /* Update for RelationalOperator: '<S150>/Equal' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     *  UnitDelay: '<S1>/Unit Delay'
     */
    STM_LatCtrlMdUnitDelay_bool = TJASTM_LatCtrlMode_nu;

    /* Update for RelationalOperator: '<S462>/Equal7' incorporates:
     *  Switch: '<S320>/Switch'
     *  UnitDelay: '<S7>/Unit Delay1'
     */
    SLC_PrevManeuverState_Enum = TJASLC_ManeuverState_nu;

    /* Update for UnitDelay: '<S470>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport11'
     */
    SLC_LaneChangeEdgeRising_bool = ABPR_LaneChangeDetected_bool;

    /* Update for UnitDelay: '<S1>/Unit Delay2' */
    SLC_TakeOverValidUnitDelay_bool = TJASLC_TakeOverValid_bool;

    /* Update for UnitDelay: '<S176>/Unit Delay' incorporates:
     *  Constant: '<S153>/Constant5'
     */
    LKA_LnQualifierEdgeRising_bool = TJALKA_InjectLaneError_C_bool;

    /* Update for UnitDelay: '<S150>/Unit Delay' incorporates:
     *  Switch: '<S151>/Switch'
     */
    LKA_PrevLnBndValid_Enum = TJALKA_LnBndValid_nu;

    /* Update for UnitDelay: '<S167>/Unit Delay' */
    LKA_PredTimeExceededEdgeFalling_bool = rtb_AND_ihx5;

    /* Update for UnitDelay: '<S1>/Unit Delay3' */
    OBF_StrongReadyUnitDelay_bool = TJAOBF_StrongReady_bool;

    /* Update for UnitDelay: '<S1>/Unit Delay4' */
    OBF_WeakReadyUnitDelay_bool = TJAOBF_WeakReady_bool;

    /* Update for UnitDelay: '<S1>/Unit Delay5' */
    OBF_CancelUnitDelay_bool = TJAOBF_Cancel_bool;

    /* Update for UnitDelay: '<S158>/Unit Delay' */
    LKA_OBFValidEdgeRising_bool = rtb_AND_laqq;

    /* Update for UnitDelay: '<S165>/Unit Delay' */
    LKA_LanePredictValidRSFF_bool = TJALKA_LanePredictValid_bool;

    /* Update for UnitDelay: '<S144>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport19'
     */
    LKA_TurnSignalLevelHoldEdgeFalling_bool = LCFRCV_TurnSignalLeverHold_bool;

    /* Update for UnitDelay: '<S213>/Unit Delay' */
    OBF_AccObjSwitchEdgeRising_bool = OBF_AccObjSwitch;

    /* Update for UnitDelay: '<S194>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport19'
     */
    OBF_TurnSignalHoldUnitDelay_bool = LCFRCV_TurnSignalLeverHold_bool;

    /* Update for UnitDelay: '<S7>/Unit Delay' */
    SLC_PrevTakeoverValidUnitDelay_bool = TJASLC_TakeOverValid_bool;

    /* Update for UnitDelay: '<S287>/Unit Delay' incorporates:
     *  UnitDelay: '<S1>/Unit Delay'
     */
    SLC_PrevLatCtrlMd2_Enum = rtb_TJASTM_PrevLatCtrlMd_nu;

    /* Update for RelationalOperator: '<S50>/Equal' incorporates:
     *  Switch: '<S552>/Switch3'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    STM_SysStateUnitDelay_bool = TJASTM_SysStateTJA_nu;

    /* Update for UnitDelay: '<S59>/Unit Delay' */
    GEN_PrevRampoutUnitDelay_bool = rtb_Equal_ciay;

    /* Update for UnitDelay: '<S285>/Unit Delay' incorporates:
     *  UnitDelay: '<S1>/Unit Delay'
     */
    SLC_PrevLatCtrlMd3_Enum = rtb_TJASTM_PrevLatCtrlMd_nu;

    /* Update for UnitDelay: '<S520>/Unit Delay' */
    SLC_LeverLeftEngagedRSFF_bool = SLC_LeverLeftEngaged_bool;

    /* Update for UnitDelay: '<S522>/Unit Delay' */
    SLC_LeverRightEngagedRSFF_bool = SLC_LeverRightEngaged_bool;

    /* Update for UnitDelay: '<S491>/Unit Delay' incorporates:
     *  UnitDelay: '<S1>/Unit Delay'
     */
    SLC_PrevLatCtrlMd_Enum = rtb_TJASTM_PrevLatCtrlMd_nu;

    /* Update for RelationalOperator: '<S463>/NotEqual' incorporates:
     *  SignalConversion: '<S314>/Signal Conversion'
     *  UnitDelay: '<S314>/Unit Delay1'
     */
    SLC_PreAbortState_enum = SLC_AbortState_enum;

    /* Update for RelationalOperator: '<S315>/NotEqual2' incorporates:
     *  Switch: '<S320>/Switch'
     *  UnitDelay: '<S264>/Unit Delay1'
     */
    SLC_PrevManeuverStateGRCO_Enum = TJASLC_ManeuverState_nu;

    /* Update for UnitDelay: '<S268>/Unit Delay1' */
    SLC_PrevDriverTrigResetLeftUnitDelay_bool = rtb_LKA_TakeOverValid_bool;

    /* Update for UnitDelay: '<S268>/Unit Delay2' */
    SLC_PrevDriverTrigResetRightUnitDelay_bool =
        rtb_SLC_DriverTriggerResetRight;

    /* Update for RelationalOperator: '<S526>/NotEqual' incorporates:
     *  UnitDelay: '<S526>/Unit Delay'
     *  UnitDelay: '<S7>/Unit Delay1'
     */
    SLC_PrevManeuverState2_Enum = rtb_SLC_PrevManeuverState_nu;

    /* Update for UnitDelay: '<S369>/Unit Delay' */
    SLC_NewEgoLaneRSFF_bool = SLC_NewEgoLane_bool;

    /* Update for UnitDelay: '<S320>/Unit Delay' incorporates:
     *  Switch: '<S320>/Switch'
     */
    SLC_PrevManeuverState3_Enum = TJASLC_ManeuverState_nu;

    /* Update for UnitDelay: '<S326>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport1'
     */
    SLC_UnitDelay_LePosY0_met = ABPR_LeLnClthPosY0_met;

    /* Update for UnitDelay: '<S314>/Unit Delay' incorporates:
     *  SignalConversion: '<S314>/Signal Conversion1'
     */
    SLC_PreLaneChangeDirtAbort_enum = SLC_LaneChangeDirectionAbort_enum;

    /* Update for UnitDelay: '<S326>/Unit Delay1' incorporates:
     *  Inport: '<Root>/Inport2'
     */
    SLC_UnitDelay_RiPosY0_met = ABPR_RiLnClthPosY0_met;

    /* Update for UnitDelay: '<S1>/Unit Delay6' */
    TJATTG_TgtTrajPosY0UnitDy_met = TJATTG_TgtTrajPosY0_met;

    /* Update for Signum: '<S327>/Sign1' incorporates:
     *  UnitDelay: '<S1>/Unit Delay7'
     */
    TJATTG_TgtTrajHeadAngUnitDy_rad = TJATTG_TgtTrajHeadAng_rad;

    /* Update for RelationalOperator: '<S566>/Equal' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    GEN_PrevLatCtrlMode_Enum = TJASTM_LatCtrlMode_nu;

    /* Update for UnitDelay: '<S552>/Unit Delay1' */
    GEN_PrevSusQuitUnitDelay_bool = STM_SuspendTimeExpired_bool;

    /* Update for UnitDelay: '<S599>/Unit Delay' incorporates:
     *  Logic: '<S552>/NOT'
     */
    GEN_PrevSusUnitDelay_bool = rtb_NotEqual_ozi5_tmp_0;

    /* Update for UnitDelay: '<S742>/Unit Delay' */
    TTG_LanePredictEdgeRising_bool = rtb_AND_ixtq;

    /* Update for UnitDelay: '<S743>/Unit Delay' */
    TTG_PredictEnableRSFF_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S741>/Unit Delay' */
    TTG_PredictEnableEdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S779>/Unit Delay' incorporates:
     *  Switch: '<S552>/Switch3'
     */
    TTG_STMSystemState_Enum = TJASTM_SysStateTJA_nu;

    /* Update for RelationalOperator: '<S779>/Equal7' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     *  UnitDelay: '<S779>/Unit Delay1'
     */
    TTG_STMLatCtrlMode_Enum = TJASTM_LatCtrlMode_nu;

    /* Update for UnitDelay: '<S779>/Unit Delay2' */
    TTG_CMBObjectCorridorUnitDelay_bool = TTG_CMBObjectCorridor_bool;

    /* Update for UnitDelay: '<S677>/Unit Delay' */
    TTG_ObjectCorridorEdgeFalling_bool = TJACMB_ObjectCorridor_bool;

    /* Update for UnitDelay: '<S777>/Unit Delay' */
    TTG_PredictionEnableEdgeFalling_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S678>/Unit Delay' */
    TTG_ObjectCorridorEdgeRising_bool = TJACMB_ObjectCorridor_bool;

    /* Update for UnitDelay: '<S803>/Unit Delay' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion1'
     */
    TTG_STMLatCtrlMode2_Enum = TJASTM_LatCtrlMode_nu;

    /* Update for UnitDelay: '<S795>/Unit Delay' */
    TTG_CMBEnableUnitDelay_bool = TTG_CMB_Enable_bool;

    /* Update for UnitDelay: '<S796>/Unit Delay' */
    TTG_LDEnableUnitDelay_bool = TTG_LD_Enable_bool;

    /* Update for UnitDelay: '<S794>/Unit Delay' */
    TTG_ODEnableUnitDelay_bool = TTG_OD_Enable_bool;

    /* Update for UnitDelay: '<S718>/Unit Delay' */
    TTG_LeLnPredictEnableEdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S735>/Unit Delay' */
    TTG_RiLnPredictEnableEdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S32>/Unit Delay' incorporates:
     *  Constant: '<S23>/Constant3'
     */
    CMB_EnableFusionEdgeFalling_bool = false;

    /* Update for UnitDelay: '<S36>/Unit Delay' */
    CMB_EnableFusionEdgeRising_bool = rtb_TTG_CMB_bool;

    /* Update for UnitDelay: '<S30>/Unit Delay' incorporates:
     *  Constant: '<S23>/Constant3'
     */
    CMB_EnableFusion2EdgeRising_bool = false;

    /* Update for UnitDelay: '<S716>/Unit Delay' */
    TTG_LeLnPredictEnable2EdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S733>/Unit Delay' */
    TTG_RiLnPredictEnable2EdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S701>/Unit Delay' */
    TTG_CntrLnPredictEnableEdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S699>/Unit Delay' */
    TTG_CntrLnPredictEnable2EdgeRising_bool = TJATTG_PredictionEnable_bool;

    /* Update for UnitDelay: '<S703>/Unit Delay' */
    TTG_CntrLnResetEdgeRising_bool = rtb_Equal_awdj;

    /* Update for UnitDelay: '<S737>/Unit Delay' */
    TTG_RiLnResetEdgeRising_bool = rtb_Equal_i3l2_idx_0;

    /* Update for UnitDelay: '<S797>/Unit Delay' */
    TTG_PredictEnableUnitDelay_bool = TTG_Predict_Enable_bool;

    /* Update for UnitDelay: '<S915>/Unit Delay' */
    TVG_LatMovStartEdgeRising_bool = rtb_Equal_i3l2_idx_1;

    /* Update for UnitDelay: '<S888>/Unit Delay2' incorporates:
     *  SignalConversion: '<S314>/Signal Conversion'
     */
    TVG_AbortStateUnitDy_Enum = SLC_AbortState_enum;

    /* Update for UnitDelay: '<S888>/Unit Delay1' incorporates:
     *  Switch: '<S320>/Switch'
     */
    SLC_PrevManeuverStateSLC2LCC_Enum = TJASLC_ManeuverState_nu;

    /* Update for UnitDelay: '<S963>/Unit Delay' */
    TVG_PredictionEnableUnitDelay_bool = rtb_SLC_LeTurnSignalOn_bool;

    /* Update for UnitDelay: '<S433>/Unit Delay' */
    SLC_TakeoverEdgeRising_bool = rtb_TJASTM_ACCIsOFF_bool;

    /* Update for UnitDelay: '<S384>/Unit Delay' incorporates:
     *  Logic: '<S379>/OR8'
     */
    SLC_TurnSignalOffEF_bool = rtb_SLC_LeAdjLaneValid_bool;

    /* Update for UnitDelay: '<S379>/Unit Delay' */
    SLC_PrebNewEgoOverTime_nu = rtb_SLC_RiTurnSignalOn_bool;

    /* Update for UnitDelay: '<S605>/Unit Delay' */
    STM_SuspendEdgeRising_bool = rtb_LKA_Dist2BndsValid_bool;

    /* Update for UnitDelay: '<S610>/Unit Delay' */
    TJATOW_TakeOverTurnOnDelay_nu = TJASTM_DrvTakeOver_bool;

    /* Update for UnitDelay: '<S620>/Unit Delay' */
    TJATOW_NPilotLineUnitDealy_bool = rtb_OR_d1pw;

    /* Update for UnitDelay: '<S621>/Unit Delay' */
    TJATOW_NPilotSeatBeltUnitDealy_bool = rtb_AND_cvae;

    /* Update for UnitDelay: '<S622>/Unit Delay' */
    TJATOW_NPilotDoorOpenUnitDealy_bool = rtb_LKA_LaneWidthValid_bool;

    /* Update for UnitDelay: '<S623>/Unit Delay' */
    TJATOW_NPilotTrunkUnitDealy_bool = rtb_LKA_RadiusValid_bool;

    /* Update for UnitDelay: '<S624>/Unit Delay' */
    TJATOW_NPilotHoodUnitDealy_bool = rtb_LKA_NoLaneChange_bool;

    /* Update for UnitDelay: '<S625>/Unit Delay' */
    TJATOW_NPilotEPBUnitDealy_bool = rtb_LKA_LengthValid_bool;

    /* Update for UnitDelay: '<S626>/Unit Delay' */
    TJATOW_NPilotSafeUnitDealy_bool = rtb_AND18;

    /* Update for UnitDelay: '<S618>/Unit Delay' */
    TJATOW_NPilotNoUnitDealy_bool = rtb_GEN_VehStInvalid_bool;

    /* Update for UnitDelay: '<S619>/Unit Delay' */
    TJATOW_NPilotBrkPUnitDealy_bool = rtb_GEN_BrakePadelInvalid__gzuo;

    /* Update for UnitDelay: '<S617>/Unit Delay' */
    TJATOW_NPilotSpdUnitDealy_bool = rtb_AND5_c5w4;

    /* Update for UnitDelay: '<S616>/Unit Delay' */
    TJATOW_NPilotErrorUnitDealy_bool = rtb_LKA_VelocityValid_bool;

    /* Update for UnitDelay: '<S111>/Unit Delay' incorporates:
     *  Inport: '<Root>/Inport1'
     */
    LKA_PrevLeLnPosY0UnitDelay_met = ABPR_LeLnClthPosY0_met;

    /* Update for UnitDelay: '<S111>/Unit Delay1' incorporates:
     *  Inport: '<Root>/Inport2'
     */
    LKA_PrevRiLnPosY0UnitDelay_met = ABPR_RiLnClthPosY0_met;

    /* Update for UnitDelay: '<S125>/Unit Delay' */
    LKA_LnIncohEdgeRising_bool = rtb_AND_jt3t;

    /* Update for UnitDelay: '<S600>/Unit Delay' */
    GEN_PrevSus2UnitDelay_bool = rtb_NotEqual_ozi5_tmp_0;

    /* Update for UnitDelay: '<S603>/Unit Delay' */
    GEN_Sus2TimeExpiredRSFF_bool = STM_SuspendTimeExpired_bool;

    /* Update for UnitDelay: '<S546>/Unit Delay1' incorporates:
     *  SignalConversion: '<S8>/Signal Conversion'
     */
    TJASTM_PrevSysStateTJAIn_envm = TJASTM_SysStateTJAIn_nu;

    /* Update for UnitDelay: '<S409>/Unit Delay' */
    SLC_LaneChangeWarnEdgeRising_bool = rtb_OR_lkl2;

    /* Update for UnitDelay: '<S411>/Unit Delay' */
    SLC_RearAbortEdgRs_bool = rtb_SLC_RearAbort_bool;

    /* Update for UnitDelay: '<S410>/Unit Delay' */
    SLC_RearCancleEdgRs_bool = rtb_AND2;

    /* Update for UnitDelay: '<S375>/Unit Delay1' */
    TJASLC_PrevSLCHighLightID_nu = TJASLC_SLCHighLightID_nu;
}

/* Model initialize function */
void TJASA_initialize(void) {
    /* Registration code */

    /* block I/O */
    {
        TJASA_B.LCDirection_enum = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
        TJASA_B.SLC_LaneChangeDirection =
            E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    }

    /* exported global signals */
    TJACMB_LaneCrvStdDev_nu = 0.0F;
    TJACMB_TraceCrvStdDev_nu = 0.0F;
    TJATVG_DistYTolLeTgtArea_met = 0.0F;
    TJATVG_DistYTolRiTgtArea_met = 0.0F;
    TJATVG_FTireAclMax_mps2 = 0.0F;
    TJATVG_FTireAclMin_mps2 = 0.0F;
    TJATVG_WeightTgtDistY_nu = 0.0F;
    TJATVG_WeightEndTime_nu = 0.0F;
    TJATVG_PredTimeCrv_sec = 0.0F;
    TJATVG_PredTimeHeadAng_sec = 0.0F;
    TJATVG_MaxTrqScalLimit_nu = 0.0F;
    TJATVG_MaxJerkAllowed_mps3 = 0.0F;
    TJACMB_CombinedPosY0_met = 0.0F;
    TJACMB_CombinedPosX0_met = 0.0F;
    TJACMB_CombinedHeading_rad = 0.0F;
    TJACMB_CombinedCrv_1pm = 0.0F;
    TJACMB_CombinedCrvChng_1pm2 = 0.0F;
    TJACMB_CombinedLength_met = 0.0F;
    TJATTG_LeCridrBndPosX0_met = 0.0F;
    TJATTG_LeCridrBndPosY0_met = 0.0F;
    TJATTG_LeCridrBndHeadAng_rad = 0.0F;
    TJATTG_LeCridrBndCrv_1pm = 0.0F;
    TJATTG_LeCridrBndCrvChng_1pm2 = 0.0F;
    TJATTG_LeCridrBndLength_met = 0.0F;
    TJATTG_RiCridrBndPosX0_met = 0.0F;
    TJATTG_RiCridrBndPosY0_met = 0.0F;
    TJATTG_RiCridrBndHeadAng_rad = 0.0F;
    TJATTG_RiCridrBndCrv_1pm = 0.0F;
    TJATTG_RiCridrBndCrvChng_1pm2 = 0.0F;
    TJATTG_RiCridrBndLength_met = 0.0F;
    TJATTG_TgtTrajPosX0_met = 0.0F;
    TJATTG_TgtTrajPosY0_met = 0.0F;
    TJATTG_TgtTrajCrv_1pm = 0.0F;
    TJATVG_PlanningHorizon_sec = 0.0F;
    TJATTG_TgtTrajHeadAng_rad = 0.0F;
    TJATTG_TgtTrajCrvChng_1pm2 = 0.0F;
    TJATTG_TgtTrajLength_met = 0.0F;
    TJATVG_StrWhStifLimit_nu = 0.0F;
    TJATVG_TrqRampGrad_1ps = 0.0F;
    TJATVG_StrWhStifGrad_1ps = 0.0F;
    TJATVG_MaxTrqScalGrad_1ps = 0.0F;
    TJATVG_MaxCrvTrajGuiCtl_1pm = 0.0F;
    TJATVG_MaxCrvGrdBuildup_1pms = 0.0F;
    TJATVG_MaxCrvGrdRed_1pms = 0.0F;
    TJATVG_MaxCrvGrdTGC_1pms = 0.0F;
    TJATVG_SensorTStamp_sec = 0.0F;
    TJASLC_SLCHighLightID_nu = 0;
    TJATTG_TgtCorridorInvalid_btf = 0U;
    TJACMB_CombinedInvalid_btf = 0U;
    TJASTM_TJAInvalid_btf = 0U;
    TJAGEN_SuspendedAndQuit_debug = 0U;
    TJAOBF_ObjFollowInvalid_btf = 0U;
    TJASLC_TriggerInvalid_btf = 0U;
    TJASLC_RiLaneChangeInvalid_btf = 0U;
    TJASLC_LeLaneChangeInvalid_btf = 0U;
    TJAOBF_ObjInLaneInvalid_btf = 0U;
    TJALKA_LaneCenterInvalid_btf = 0U;
    TJATVG_DeratingLevel_nu = 0U;
    TJAGEN_StrongReadyInvalid_btf = 0U;
    TJALKA_LnIncoherenceStatus_nu = 0U;
    TJASLC_LaneChangeInfo = 0U;
    TJATVG_TrajPlanServQu_nu = 0U;
    TJAGEN_CancelStatus_btf = 0U;
    TJAGEN_WeakReadyInvalid_btf = 0U;
    TJATOW_DriverTakeOverWarning_nu = 0U;
    TJASTM_NpilotSysInfo = 0U;
    TJALKA_LaneIncoherence_btf = 0U;
    TJASTM_PilotAudioPlay = 0U;
    TJASTM_LatCtrlHandsOffReleaseWarn_nu = 0U;
    TJASTM_PilotDisableACCSwitch_nu = 0U;
    TJASTM_PilotEnableACCSwitch_nu = 0U;
    TJASLC_LaneChangWarning_nu = 0U;
    TJASLC_SLCAudioPlay_nu = 0U;
    TJASLC_CancelAbort_btf = 0U;
    TJASLC_TurnLtDirctionReq_nu = 0U;
    TJAOBF_TgtObjDataInvalid_btf = 0U;
    TJALKA_LnQualityInv_btf = 0U;
    TJATVG_TriggerReplan_nu = false;
    TJATVG_HighStatAccu_bool = false;
    TJALKA_LanePredictValid_bool = false;
    TJALKA_Cancel_bool = false;
    TJAOBF_ObjLaneValidDuration_bool = false;
    TJALKA_StrongReady_bool = false;
    TJAGEN_LKAOnlySwitch_bool = false;
    TJAOBF_Cancel_bool = false;
    TJALKA_WeakReady_bool = false;
    TJAGEN_Clearance_bool = false;
    TJAGEN_Degradation_bool = false;
    TJACMB_ObjectCorridor_bool = false;
    TJAOBF_StrongReady_bool = false;
    TJAOBF_WeakReady_bool = false;
    TJASLC_Nb_DCLCSwitchNVRAM_nu = false;
    TJASLC_StrongReady_bool = false;
    TJASLC_WeakReady_bool = false;
    TJASLC_Cancel_bool = false;
    TJACMB_StrongReady_bool = false;
    TJACMB_WeakReady_bool = false;
    TJAGEN_FunctionSwitch_bool = false;
    TJAGEN_CodeFunction_bool = false;
    TJAGEN_Error_bool = false;
    SLC_LCPLeft2Active_bool = false;
    SLC_LCPRight2Active_bool = false;
    SLC_LCPLeft2Passive_bool = false;
    SLC_LCPRight2Passive_bool = false;
    TJAGEN_Abort_bool = false;
    GEN_AllStateAvailable_bool = false;
    TJAGEN_StrongReady_bool = false;
    TJAGEN_Cancel_nu = false;
    TJAGEN_WeakReady_bool = false;
    TJAGEN_FunctionQuit_bool = false;
    TJAGEN_SuspendStart_bool = false;
    TJAGEN_SuspendEnd_bool = false;
    STM_PrevRAMPOUT_bool = false;
    TJATTG_PredictionEnable_bool = false;
    TJATTG_TransTriggerReplan_bool = false;
    TJASLC_TakeOverValid_bool = false;
    TJATVG_LtcyCompActivated_nu = false;
    TJALKA_SRLaneRelateCheck_bool = false;
    TJASTM_DrvTakeOver_bool = false;
    STM_SuspendTimeExpired_bool = false;
    TJAOBF_TgtObjDataValid_bool = false;
    TJATVG_TrajGuiQu_nu = E_TJATVG_TrajGuiQu_nu_TGQ_REQ_OFF;
    TJASTM_SysStateTJAIn_nu = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
    TJASTM_SysStateTJA_nu = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
    TJASTM_SysStateHWA_nu = E_TJASTM_SysStateHWA_nu_HWAState_OFF;
    TJASTM_LatCtrlMode_nu = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    TJASLC_ReadyToTrigger_nu = E_TJASLC_ReadyToTrigger_nu_TRIGREADY_NONE;
    TJASLC_ManeuverState_nu = E_TJASLC_ManeuverState_nu_PASSIVE;
    TJASLC_LCDirection_enum = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    TJASLC_LaneChangeTrig_nu = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    TJALKA_LnBndValid_nu = E_TJALKA_LnBndValid_nu_BND_NOT_VALID;

    /* custom signals */
    SLC_CenterDistToBoundary_met = 0.0F;
    LKA_LeLnCrvQualityValid_bool = false;
    LKA_LeLnQualityValid_bool = false;
    OBF_AccObjSwitch = false;
    OBF_AccObjValidLaneCheck = false;
    OBF_LeftLaneCheckValid_bool = false;
    OBF_RightLaneCheckValid_bool = false;
    OBF_LaneCheckValid_bool = false;
    OBF_TargetOutsideEgoLane_bool = false;
    OBF_MinDist2LeftBndInvalid = false;
    OBF_MinDist2RightBndInvalid = false;
    OBF_DistOrEgoLaneInvalid_bool = false;
    OBF_TargetObjDataSR_bool = false;
    OBF_TargetObjDataWR_bool = false;
    SLC_PrevReset_bool = false;
    SLC_LeverLeftEngaged_bool = false;
    SLC_LeverRightEngaged_bool = false;
    SLC_TriggerLeft_bool = false;
    SLC_TriggerRight_bool = false;
    SLC_AllowGoBack_bool = false;
    SLC_Cancel_bool = false;
    CMB_LaneQualityInvalid_bool = false;
    CMB_ObjectFollowingOnly_bool = false;
    OF_ObjectDangerLeftRear_bool = false;
    OF_ObjectDangerRightRear_bool = false;
    OF_NoObjectCollision_bool = false;
    SLC_StrongReadyBothSides_bool = false;
    SLC_WeakReadyBothSides_bool = false;
    SLC_WeakReadyLeft_bool = false;
    SLC_WeakReadyRight_bool = false;
    SLC_MaxInitDurationExceeded_bool = false;
    SLC_ManvStatePassive_bool = false;
    SLC_LaneCheckValid_bool = false;
    SLC_NewEgoLane_bool = false;
    SLC_LCM_Start_bool = false;
    SLC_LCM_End_bool = false;
    SLC_Abort_bool = false;
    SLC_LCM_Cancel_bool = false;
    SLC_IntoAbort_nu = false;
    SLC_SameLaneChangeDetc_bool = false;
    SLC_LaneChangeBackDetc_bool = false;
    SLC_ExitAbort_bool = false;
    SLC_ExitAbortNewEgo_bool = false;
    STM_Cancel_bool = false;
    TTG_CMBObjectCorridor_bool = false;
    TTG_LaneUpdate_bool = false;
    TTG_ObjectUpdate_bool = false;
    TTG_LD_PredictFinish_bool = false;
    TTG_Reset_bool = false;
    TTG_Enable_bool = false;
    TTG_CMB_Enable_bool = false;
    TTG_LD_Enable_bool = false;
    TTG_OD_Enable_bool = false;
    TTG_Predict_Enable_bool = false;
    SLC_LaneChangeCancleInfo = false;
    SLC_LaneChangePendingInfo = false;
    SLC_VehSpdTooLowInfo = false;
    SLC_LaneChangeEndInfo = false;
    SLC_LaneChangeOnGoingInfo = false;
    OBF_AccObjValid = false;
    OBF_AccObjValid_bool = false;
    SLC_LaneChangeDirectionIn_nu = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    SLC_LaneChangeDirectionAbort_enum =
        E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    SLC_AbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;

    /* states (dwork) */
    (void)memset((void *)&TJASA_DW, 0, sizeof(DW_TJASA_T));

    /* custom states */
    TTG_TgtTrajAndCridrBndUnitDelay_bus = TJASA_rtZBus_TgtTrajAndCridrBnd_nu;
    CMB_PrevCntrCrv1UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv2UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv3UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv4UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv5UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv6UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv7UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv8UnitDelay_1pm = 0.0F;
    CMB_PrevCntrCrv9UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv1UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv2UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv3UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv4UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv5UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv6UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv7UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv8UnitDelay_1pm = 0.0F;
    CMB_PrevTgtCrv9UnitDelay_1pm = 0.0F;
    LKA_CrvQualTurnOffDelay_sec[0] = 0.0F;
    LKA_CrvQualTurnOffDelay_sec[1] = 0.0F;
    LKA_CrvQualTurnOnDelay_sec[0] = 0.0F;
    LKA_CrvQualTurnOnDelay_sec[1] = 0.0F;
    LKA_LnQualTurnOffDelay_sec[0] = 0.0F;
    LKA_LnQualTurnOffDelay_sec[1] = 0.0F;
    LKA_LnQualTurnOnDelay_sec[0] = 0.0F;
    LKA_LnQualTurnOnDelay_sec[1] = 0.0F;
    LKA_LnQualifierTurnOffDelay_sec = 0.0F;
    TJALKA_LeftLlineValidRD_Sec = 0.0F;
    TJALKA_RightLlineValidRD_Sec = 0.0F;
    LKA_LnBndValidTurnOnDelay_sec = 0.0F;
    LKA_PredTimeExceededTurnOffDelay_sec = 0.0F;
    LKA_BlockTimerExpiredTimerRetrigger_sec = 0.0F;
    OBF_AccObjSwitchTurnOffDelay_sec = 0.0F;
    OBF_AccObjValidLaneTurnOnDelay_sec = 0.0F;
    OBF_LeftLineValidTurnOnDelay_sec = 0.0F;
    OBF_RightLineValidTurnOnDelay_sec = 0.0F;
    OBF_ObjLaneValidDurationTurnOnDelay_sec = 0.0F;
    OBF_BitfieldValidTurnOnDelay_sec = 0.0F;
    OBF_MinDurLCforOBTurnOnDelay_sec = 0.0F;
    OBF_MaxDurObjBrdgTimerRe_sec = 0.0F;
    OBF_BlockTimeExpiredTimerRetrigger_sec = 0.0F;
    GEN_RampoutTimeExpiredTimerRetrigger_sec = 0.0F;
    SLC_MaxManeuTimeRetrigger_sec = 0.0F;
    SLC_LeftTurnSignalOffDelay_sec = 0.0F;
    SLC_RightTurnSignalOffDelay_sec = 0.0F;
    SLC_FrontSafeTrOnTime_sec = 0.0F;
    SLC_FrontUnSafeTrOnTime_sec = 0.0F;
    SLC_TurnSignalTurnOffDelay_sec[0] = 0.0F;
    SLC_TurnSignalTurnOffDelay_sec[1] = 0.0F;
    SLC_LeverEngagedTurnOnDelay2_sec[0] = 0.0F;
    SLC_LeverEngagedTurnOnDelay2_sec[1] = 0.0F;
    SLC_GoBkLfTurnLtTurnOffDelay_sec = 0.0F;
    SLC_GoBkRiTurnLtTurnOffDelay_sec = 0.0F;
    SLC_LeftSafeTrOnTime_sec = 0.0F;
    SLC_LeftUnSafeTrOnTime_sec = 0.0F;
    SLC_RightSafeTrOnTime_sec = 0.0F;
    SLC_RightUnSafeTrOnTime_sec = 0.0F;
    SLC_RearObjDecTurnOffDelay_sec[0] = 0.0F;
    SLC_RearObjDecTurnOffDelay_sec[1] = 0.0F;
    SLC_LCActiveTurnOnDelay_sec = 0.0F;
    SLC_LnChngDetectedTurnOffDelay_sec = 0.0F;
    SLC_LnChngBlockTimeTurnOffDelay_sec = 0.0F;
    SLC_LnChngBlockTimeCancleTfDelay_sec = 0.0F;
    SLC_RiLaneTypeTurnOffDelay_sec = 0.0F;
    SLC_LeLaneTypeTurnOffDelay_sec = 0.0F;
    OF_RearObjDecTurnOffDelay_sec[0] = 0.0F;
    OF_RearObjDecTurnOffDelay_sec[1] = 0.0F;
    CMB_LnQualTurnOffDelay_sec[0] = 0.0F;
    CMB_LnQualTurnOffDelay_sec[1] = 0.0F;
    CMB_LnQualTurnOnDelay_sec[0] = 0.0F;
    CMB_LnQualTurnOnDelay_sec[1] = 0.0F;
    OF_RearObjDecCrvtTurnOffDelay_sec = 0.0F;
    SLC_LeverEngagedTurnOnDelay_sec = 0.0F;
    SLC_PrevFtWhDit2BdSignUnitDelay_met = 0.0F;
    SLC_PrevFrontWheelDist2BoundUnitDelay_met = 0.0F;
    SLC_LaneChngDetectedTurnOffDelay_sec = 0.0F;
    SLC_PrevReWhDit2BdSignUnitDelay_met = 0.0F;
    SLC_PrevRearWheelDist2BoundUnitDelay_met = 0.0F;
    SLC_LCPLeft2ActiveTurnOnDelay_sec = 0.0F;
    SLC_LCPRight2ActiveTurnOnDelay_sec = 0.0F;
    SLC_LCPLeft2PassiveTurnOnDelay_sec = 0.0F;
    SLC_LCPRight2PassiveTurnOnDelay_sec = 0.0F;
    SLC_UnitDelay_LePosY0_met = 0.0F;
    SLC_UnitDelay_RiPosY0_met = 0.0F;
    TJATTG_TgtTrajPosY0UnitDy_met = 0.0F;
    TJATTG_TgtTrajHeadAngUnitDy_rad = 0.0F;
    SLC_AbortYHdSignDelay2_sec = 0.0F;
    SLC_AbortTime_sec = 0.0F;
    SLC_AbortNewEgoTime_sec = 0.0F;
    GEN_BlockTimeExpiredTimerRetrigger_sec = 0.0F;
    GEN_HazardTurnOnDelay_sec = 0.0F;
    GEN_HODTurnOnDelay_sec = 0.0F;
    GEN_SafeFuncActiveTurnOnDelay_sec = 0.0F;
    GEN_SafeFuncErrorTurnOnDelay_sec = 0.0F;
    GEN_ManualTorMaxTurnOnDelay_sec = 0.0F;
    GEN_ManualTorMinTurnOnDelay_sec = 0.0F;
    GEN_SusTimeExpiredTimerRetrigger_sec = 0.0F;
    TTG_OdoYawDelayRe_rad = 0.0F;
    TTG_OdoPosXDelayRe_met = 0.0F;
    TTG_RiLnCrvUnitDelay_1pm = 0.0F;
    TTG_RiLnHeadingUnitDelay_rad = 0.0F;
    TTG_OdoPosYDelayRe_met = 0.0F;
    TTG_RiLnPosY0UnitDelay_met = 0.0F;
    TTG_LeLnCrvUnitDelay_1pm = 0.0F;
    TTG_LeLnHeadingUnitDelay_rad = 0.0F;
    TTG_LeLnPosY0UnitDelay_met = 0.0F;
    TTG_TransitionTimeTurnOffDelayWithRst_sec = 0.0F;
    TTG_TransitionTimeUnitDelay_sec = 0.0F;
    TTG_TransitionFactorAStopwatch_sec = 0.0F;
    TTG_LeCorridorPosY0UnitDelay_met = 0.0F;
    TTG_LeCorridorPosX0UnitDelay_met = 0.0F;
    TTG_LeCorridorHeadingUnitDelay_rad = 0.0F;
    TTG_LeLnCrvPredictLowPass_1pm = 0.0F;
    TTG_RiCrvPredictLowPass_1pm = 0.0F;
    CMB_EnableFusionTurnOffDelay_sec = 0.0F;
    CMB_PrevCombCrvLFUnitDelay_1pm = 0.0F;
    CMB_WeightCrvLowPass_1pm = 0.0F;
    CMB_EnableFusionStopwatch_sec = 0.0F;
    CMB_WeightCrv2LowPass_1pm = 0.0F;
    TTG_LeCorridorCrvUnitDelay_1pm = 0.0F;
    TTG_LeLnCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_LeLnCrvChngPredictLowPass_1pm2 = 0.0F;
    TTG_RiLnCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_RiLnCrvChngPredictLowPass_1pm2 = 0.0F;
    TTG_LeCorridorCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_LeLnLengthUnitDelay_met = 0.0F;
    TTG_RiLnLengthUnitDelay_met = 0.0F;
    TTG_LeCorridorLengthUnitDelay_met = 0.0F;
    TTG_RiCorridorPosX0UnitDelay_met = 0.0F;
    TTG_RiCorridorPosY0UnitDelay_met = 0.0F;
    TTG_RiCorridorHeadingUnitDelay_rad = 0.0F;
    TTG_RiCorridorCrvUnitDelay_1pm = 0.0F;
    TTG_RiCorridorCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_RiCorridorLengthUnitDelay_met = 0.0F;
    TTG_TgtCorridorPosX0UnitDelay_met = 0.0F;
    TTG_CntrLnCrvUnitDelay_1pm = 0.0F;
    TTG_CntrLnHeadingUnitDelay_rad = 0.0F;
    TTG_CntrLnPosY0UnitDelay_met = 0.0F;
    TTG_TgtCorridorPosY0UnitDelay_met = 0.0F;
    TTG_TgtCorridorHeadingUnitDelay_rad = 0.0F;
    TTG_CntrCrvPredictLowPass_1pm = 0.0F;
    TTG_TgtCorridorCrvUnitDelay_1pm = 0.0F;
    TTG_CntrLnCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_CntrLnCrvChngPredictLowPass_1pm2 = 0.0F;
    TTG_TgtCorridorCrvChngUnitDelay_1pm2 = 0.0F;
    TTG_CntrLnLengthUnitDelay_met = 0.0F;
    TTG_TgtCorridorLengthUnitDelay_met = 0.0F;
    TVG_PredictionEnableTurnOffDelay_sec = 0.0F;
    SLC_ManeuverStateTurnOnDelay_sec = 0.0F;
    SLC_TakeoverAbortTurnOffDelay_sec = 0.0F;
    SLC_TakeoverValidTurnOffDelay_sec = 0.0F;
    SLC_LaneChangCancleTimeDelay_sec = 0.0F;
    SLC_LaneChangEndTimeDelay_sec = 0.0F;
    TJATOW_TakeOverTurnOffDelay_sec = 0.0F;
    TJATOW_NPilotLaneAudioPlay_sec = 0.0F;
    TJATOW_NPilotSeatBeltPAudioPlay_sec = 0.0F;
    TJATOW_NPilotDoorAudioPlay_sec = 0.0F;
    TJATOW_NPilotTrunkAudioPlay_sec = 0.0F;
    TJATOW_NPilotHoodAudioPlay_sec = 0.0F;
    TJATOW_NPilotEPBAudioPlay_sec = 0.0F;
    TJATOW_NPilotSafeAudioPlay_sec = 0.0F;
    TJATOW_NPilotNoAudioPlay_sec = 0.0F;
    TJATOW_NPilotBrkPAudioPlay_sec = 0.0F;
    TJATOW_NPilotSpdAudioPlay_sec = 0.0F;
    TJATOW_NPilotErrorAudioPlay_sec = 0.0F;
    TJATOW_NPilotOffAudioPlay_sec = 0.0F;
    TJATOW_NPilotOnAudioPlay_sec = 0.0F;
    LKA_PrevLeLnPosY0UnitDelay_met = 0.0F;
    LKA_PrevRiLnPosY0UnitDelay_met = 0.0F;
    LKA_LeLnIncohTurnOffDelay_sec = 0.0F;
    LKA_RiLnIncohTurnOffDelay_sec = 0.0F;
    LKA_LnIncohTurnOffDelay_sec = 0.0F;
    LKA_PrevVehDistUnitDelay_met = 0.0F;
    OBF_AccObjValidTurnOnDelay_sec = 0.0F;
    TJATOW_PilotOverrideAudioPlay_sec = 0.0F;
    TJATOW_PilotResumeAudioPlay_sec = 0.0F;
    GEN_Sus2TimeExpiredTimerRetrigger_sec = 0.0F;
    TJASTM_HandsOffWarnTurnOffDelay_sec = 0.0F;
    TJASTM_ACCOffTurnOffDelay_sec = 0.0F;
    TJASTM_ACCActiveOvertime_sec = 0.0F;
    SLC_LCWPassiveTurnOnDly_sec = 0.0F;
    SLC_LaneChangeWarnTurnOffDelay_sec = 0.0F;
    SLC_RearAbortTunOffDly_sec = 0.0F;
    SLC_RearCancleTunOffDly_sec = 0.0F;
    SLC_ResetTurnOffDelay_sec = 0.0F;
    TJASLC_PrevSLCHighLightID_nu = 0;
    TTG_LeLnResetEdgeRising_bool = false;
    LKA_CrvQualRSFF_bool[0] = false;
    LKA_CrvQualRSFF_bool[1] = false;
    LKA_LnQualRSFF_bool[0] = false;
    LKA_LnQualRSFF_bool[1] = false;
    LKA_DistVeh2LnBndHyst_bool[0] = false;
    LKA_DistVeh2LnBndHyst_bool[1] = false;
    LKA_ValidLengthHyst_bool[0] = false;
    LKA_ValidLengthHyst_bool[1] = false;
    SLC_LaneChangeEdgeRising_bool = false;
    SLC_LaneChangeRSFF_bool = false;
    SLC_AdjLnWidthMaxHyst_bool[0] = false;
    SLC_AdjLnWidthMaxHyst_bool[1] = false;
    SLC_AdjLnWidthMinHyst_bool[0] = false;
    SLC_AdjLnWidthMinHyst_bool[1] = false;
    SLC_TakeOverValidUnitDelay_bool = false;
    LKA_LnQualifierEdgeRising_bool = false;
    LKA_PredTimeExceededEdgeFalling_bool = false;
    OBF_StrongReadyUnitDelay_bool = false;
    OBF_WeakReadyUnitDelay_bool = false;
    OBF_CancelUnitDelay_bool = false;
    LKA_OBFValidEdgeRising_bool = false;
    LKA_LanePredictValidRSFF_bool = false;
    LKA_VelXMaxHyst_bool = false;
    LKA_VelXMinHyst_bool = false;
    LKA_LaneWidthMaxHyst_bool = false;
    LKA_LaneWidthMinHyst_bool = false;
    LKA_RadiusHyst_bool = false;
    LKA_TurnSignalLevelHoldEdgeFalling_bool = false;
    LKA_TrajPlanCancelRSFF_bool = false;
    OBF_AccObjSwitchEdgeRising_bool = false;
    OBF_ObjLaneValidHoldUnitDelay_bool = false;
    OBF_LaneAttributesValidHyst_bool[0] = false;
    OBF_LaneAttributesValidHyst_bool[1] = false;
    OBF_MinDist2BndHyst_bool[0] = false;
    OBF_MinDist2BndHyst_bool[1] = false;
    OBF_MaxDist2BndHyst_bool[0] = false;
    OBF_MaxDist2BndHyst_bool[1] = false;
    OBF_PosY0_SRHyst_bool = false;
    OBF_Heading_SRHyst_bool = false;
    OBF_Crv_SRHyst_bool = false;
    OBF_VelXMaxHyst_bool = false;
    OBF_VelXMinHyst_bool = false;
    OBF_TurnSignalHoldUnitDelay_bool = false;
    LKA_VelXMaxHystWR_bool = false;
    LKA_VelXMinHystWR_bool = false;
    SLC_PrevTakeoverValidUnitDelay_bool = false;
    GEN_PrevRampoutUnitDelay_bool = false;
    GEN_RampoutTimeExpiredRSFF_bool = false;
    SLC_MaxManeuTimeRSFF_bool = false;
    SLC_PrevResetUnitDelay_bool = false;
    SLC_PrevResetRSFF_bool = false;
    SLC_LeverLeftEngagedRSFF_bool = false;
    SLC_PrevResetRSFF2_bool = false;
    SLC_LeverRightEngagedRSFF_bool = false;
    SLC_OELCNewEgoLaneRSFF_bool = false;
    SLC_EgoVehVelMaxHyst_bool = false;
    SLC_EgoVehVelMinHyst_bool = false;
    SLC_LnChngDetectedRSFF_bool = false;
    SLC_TurnSignalOffBlckRSFF_bool = false;
    SLC_LaneWidthMaxHyst_bool = false;
    SLC_LaneWidthMinHyst_bool = false;
    CMB_LnQualRSFF_bool[0] = false;
    CMB_LnQualRSFF_bool[1] = false;
    SLC_GRCOLeftRSFF_bool = false;
    SLC_GRCORightRSFF_bool = false;
    CMB_VelXMaxHyst_bool = false;
    CMB_VelXMinHyst_bool = false;
    SLC_PrevDriverTrigResetLeftUnitDelay_bool = false;
    SLC_PrevDriverTrigResetRightUnitDelay_bool = false;
    SLC_NewEgoLaneRSFF_bool = false;
    GEN_AclXMaxHyst_bool = false;
    GEN_AclXMinHyst_bool = false;
    GEN_AclYMaxHyst_bool = false;
    GEN_SteerWAngleHyst_bool = false;
    GEN_SteerWAngleGradHyst_bool = false;
    GEN_VehYawRateDIHyst_bool = false;
    GEN_ManualTorqueMaxHyst_bool = false;
    GEN_VehCrvDIHyst_bool = false;
    GEN_SteerWAngleSusHyst_bool = false;
    GEN_SteerWAngleGradSusHyst_bool = false;
    GEN_ManualTorqueMinHyst_bool = false;
    GEN_PrevRampoutNUnitDelay_bool = false;
    GEN_PrevSusQuitUnitDelay_bool = false;
    TJASTM_PrevACCActOvrtm_bool = false;
    GEN_PrevSusUnitDelay_bool = false;
    GEN_SusTimeExpiredRSFF_bool = false;
    TTG_PrevLnLengResetUnitDelay_bool = false;
    TTG_LanePredictEdgeRising_bool = false;
    TTG_PredictEnableRSFF_bool = false;
    TTG_PredictEnableEdgeRising_bool = false;
    TTG_CMBObjectCorridorUnitDelay_bool = false;
    TTG_ObjectCorridorEdgeFalling_bool = false;
    TTG_PredictionEnableEdgeFalling_bool = false;
    TTG_ObjectCorridorEdgeRising_bool = false;
    TTG_CMBEnableUnitDelay_bool = false;
    TTG_LDEnableUnitDelay_bool = false;
    TTG_ODEnableUnitDelay_bool = false;
    TTG_LeLnPredictEnableEdgeRising_bool = false;
    TTG_RiLnPredictEnableEdgeRising_bool = false;
    CMB_EnableFusionEdgeFalling_bool = false;
    CMB_EnableFusionEdgeRising_bool = false;
    CMB_EnableFusion2EdgeRising_bool = false;
    TTG_LeLnPredictEnable2EdgeRising_bool = false;
    TTG_RiLnPredictEnable2EdgeRising_bool = false;
    TTG_CntrLnPredictEnableEdgeRising_bool = false;
    TTG_CntrLnPredictEnable2EdgeRising_bool = false;
    TTG_CntrLnResetEdgeRising_bool = false;
    TTG_RiLnResetEdgeRising_bool = false;
    TTG_PredictEnableUnitDelay_bool = false;
    TVG_LatMovStartEdgeRising_bool = false;
    TVG_LatMovStartRSFF_bool = false;
    TVG_PredictionEnableUnitDelay_bool = false;
    SLC_TakeoverEdgeRising_bool = false;
    SLC_TakeOverRSFF2_bool = false;
    SLC_TurnSignalOffEF_bool = false;
    SLC_PrebNewEgoOverTime_nu = false;
    STM_SuspendEdgeRising_bool = false;
    TJATOW_TakeOverTurnOnDelay_nu = false;
    TJATOW_RSFlipFlop_nu = false;
    TJATOW_NPilotLineUnitDealy_bool = false;
    TJATOW_NPilotSeatBeltUnitDealy_bool = false;
    TJATOW_NPilotDoorOpenUnitDealy_bool = false;
    TJATOW_NPilotTrunkUnitDealy_bool = false;
    TJATOW_NPilotHoodUnitDealy_bool = false;
    TJATOW_NPilotEPBUnitDealy_bool = false;
    TJATOW_NPilotSafeUnitDealy_bool = false;
    TJATOW_NPilotNoUnitDealy_bool = false;
    TJATOW_NPilotBrkPUnitDealy_bool = false;
    TJATOW_NPilotSpdUnitDealy_bool = false;
    TJATOW_NPilotErrorUnitDealy_bool = false;
    LKA_LnIncohEdgeRising_bool = false;
    OBF_AccObjValidHoldUnitDelay_bool = false;
    GEN_PrevSus2UnitDelay_bool = false;
    GEN_Sus2TimeExpiredRSFF_bool = false;
    SLC_LaneChangeWarnEdgeRising_bool = false;
    SLC_RearAbortEdgRs_bool = false;
    SLC_RearCancleEdgRs_bool = false;
    TJASLC_RSRightLaneChangeWarn_nu = false;
    TJASLC_RSLeftLaneChangeWarn_nu = false;
    STM_SysStateUnitDelay_bool = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
    TTG_STMSystemState_Enum = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
    TJASTM_PrevSysStateTJAIn_envm = E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT;
    SLC_PrevLatCtrlMdSLC2LCC_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    STM_LatCtrlMdUnitDelay_bool = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    SLC_PrevLatCtrlMd2_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    SLC_PrevLatCtrlMd3_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    SLC_PrevLatCtrlMd_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    GEN_PrevLatCtrlMode_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    TTG_STMLatCtrlMode_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    TTG_STMLatCtrlMode2_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    TVG_PrevLatCtlModeUnitDelay_bool =
        E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    TVG_LatCtrlMode_Enum = E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE;
    SLC_PrevManeuverState_Enum = E_TJASLC_ManeuverState_nu_PASSIVE;
    SLC_PrevManeuverStateGRCO_Enum = E_TJASLC_ManeuverState_nu_PASSIVE;
    SLC_PrevManeuverState2_Enum = E_TJASLC_ManeuverState_nu_PASSIVE;
    SLC_PrevManeuverState3_Enum = E_TJASLC_ManeuverState_nu_PASSIVE;
    SLC_PrevManeuverStateSLC2LCC_Enum = E_TJASLC_ManeuverState_nu_PASSIVE;
    LKA_PrevLnBndValid_Enum = E_TJALKA_LnBndValid_nu_BND_NOT_VALID;
    SLC_PrevLaneChangeTrigger_nu = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    SLC_PreLaneChangeDirtAbort_enum = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;
    SLC_PreAbortState_enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
    TVG_AbortStateUnitDy_Enum = E_TJASLC_AbortState_nu_ABORT_NOACTIVE;
    SLC_LaneChangeWarnUnitDy_Enum = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
    SLC_PrevReaAbortWarnSide_enum = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;
    SLC_PrevRearCancleWarnSide_enum = E_TJASLC_LaneChangeWarning_nu_NO_WARNING;

    /* InitializeConditions for UnitDelay: '<S720>/Unit Delay' */
    TTG_LeLnResetEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S470>/Unit Delay' */
    SLC_LaneChangeEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S176>/Unit Delay' */
    LKA_LnQualifierEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S158>/Unit Delay' */
    LKA_OBFValidEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S213>/Unit Delay' */
    OBF_AccObjSwitchEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S742>/Unit Delay' */
    TTG_LanePredictEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S741>/Unit Delay' */
    TTG_PredictEnableEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S678>/Unit Delay' */
    TTG_ObjectCorridorEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S718>/Unit Delay' */
    TTG_LeLnPredictEnableEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S735>/Unit Delay' */
    TTG_RiLnPredictEnableEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S36>/Unit Delay' */
    CMB_EnableFusionEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S30>/Unit Delay' */
    CMB_EnableFusion2EdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S716>/Unit Delay' */
    TTG_LeLnPredictEnable2EdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S733>/Unit Delay' */
    TTG_RiLnPredictEnable2EdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S701>/Unit Delay' */
    TTG_CntrLnPredictEnableEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S699>/Unit Delay' */
    TTG_CntrLnPredictEnable2EdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S703>/Unit Delay' */
    TTG_CntrLnResetEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S737>/Unit Delay' */
    TTG_RiLnResetEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S915>/Unit Delay' */
    TVG_LatMovStartEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S433>/Unit Delay' */
    SLC_TakeoverEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S605>/Unit Delay' */
    STM_SuspendEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S610>/Unit Delay' */
    TJATOW_TakeOverTurnOnDelay_nu = true;

    /* InitializeConditions for UnitDelay: '<S620>/Unit Delay' */
    TJATOW_NPilotLineUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S621>/Unit Delay' */
    TJATOW_NPilotSeatBeltUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S622>/Unit Delay' */
    TJATOW_NPilotDoorOpenUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S623>/Unit Delay' */
    TJATOW_NPilotTrunkUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S624>/Unit Delay' */
    TJATOW_NPilotHoodUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S625>/Unit Delay' */
    TJATOW_NPilotEPBUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S626>/Unit Delay' */
    TJATOW_NPilotSafeUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S618>/Unit Delay' */
    TJATOW_NPilotNoUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S619>/Unit Delay' */
    TJATOW_NPilotBrkPUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S617>/Unit Delay' */
    TJATOW_NPilotSpdUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S616>/Unit Delay' */
    TJATOW_NPilotErrorUnitDealy_bool = true;

    /* InitializeConditions for UnitDelay: '<S125>/Unit Delay' */
    LKA_LnIncohEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S546>/Unit Delay1' */
    TJASTM_PrevSysStateTJAIn_envm = E_TJASTM_SysStateTJA_nu_SYSST_OFF;

    /* InitializeConditions for UnitDelay: '<S409>/Unit Delay' */
    SLC_LaneChangeWarnEdgeRising_bool = true;

    /* InitializeConditions for UnitDelay: '<S411>/Unit Delay' */
    SLC_RearAbortEdgRs_bool = true;

    /* InitializeConditions for UnitDelay: '<S410>/Unit Delay' */
    SLC_RearCancleEdgRs_bool = true;

    /* InitializeConditions for UnitDelay: '<S375>/Unit Delay1' */
    TJASLC_PrevSLCHighLightID_nu = -1;

    /* SystemInitialize for Chart: '<S264>/ManeuverState' */
    TJASA_DW.is_ActiveLeft = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_ActiveRight = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_NewEgoLane = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_active_c6_TJASA = 0U;
    TJASA_DW.is_c6_TJASA = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_B.LCDirection_enum = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;

    /* SystemInitialize for Chart: '<S314>/AbortState' */
    TJASA_DW.is_OriginLaneAbort = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_active_c180_TJASA = 0U;
    TJASA_DW.is_c180_TJASA = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_B.SLC_LaneChangeDirection = E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG;

    /* SystemInitialize for Chart: '<S8>/StateMachineTJA' */
    TJASA_DW.is_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_NOT_ACTIVE = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_active_c20_TJASA = 0U;
    TJASA_DW.is_c20_TJASA = TJASA_IN_NO_ACTIVE_CHILD;

    /* SystemInitialize for Chart: '<S8>/LatCtrlMode' */
    TJASA_DW.is_Controlling = TJASA_IN_NO_ACTIVE_CHILD;
    TJASA_DW.is_active_c19_TJASA = 0U;
    TJASA_DW.is_c19_TJASA = TJASA_IN_NO_ACTIVE_CHILD;

    /* ConstCode for Constant: '<S15>/Constant2' */
    TJACMB_Cancel_bool = false;

    /* ConstCode for Constant: '<S891>/Constant' */
    TJATVG_ObstacleVelX_mps = 0.0F;

    /* ConstCode for Constant: '<S891>/Constant2' */
    TJATVG_ObstacleAclX_mps2 = 0.0F;

    /* ConstCode for Constant: '<S891>/Constant3' */
    TJATVG_ObstacleWidth_met = 0.0F;

    /* ConstCode for Constant: '<S891>/Constant8' */
    TJATVG_ObstacleDistX_met = 999.0F;

    /* ConstCode for Constant: '<S891>/Constant9' */
    TJATVG_ObstacleDistY_met = 100.0F;

    /* ConstCode for Constant: '<S891>/Constant1' */
    TJATVG_CrvAmplActivated_nu = 0U;

    /* ConstCode for Constant: '<S891>/Constant5' */
    TJATVG_LimiterActivated_nu = 0U;

    /* ConstCode for Constant: '<S891>/Constant6' */
    TJATVG_LimiterType_nu = 0U;

    /* ConstCode for Constant: '<S891>/Constant7' */
    TJATVG_LimiterTimeDuration_sec = 0.0F;
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