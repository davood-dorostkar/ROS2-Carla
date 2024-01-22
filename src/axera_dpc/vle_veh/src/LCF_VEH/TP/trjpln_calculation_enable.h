/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef TRJPLN_CALCULATION_ENABLE_H
#define TRJPLN_CALCULATION_ENABLE_H

#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "trajectory_plan_ext.h"
#include "trjpln_latency_compensation.h"
#include "trjpln_consts.h"
#include "tue_common_libs.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef struct {
    uint8 uiControllingFunction_nu;  // function type of lateral controlling,
                                     // [0,7], E_LCF_OFF_nu= 0,E_LCF_TJA_nu=
                                     // 1,E_LCF_LDP_nu= 2,E_LCF_LDPOC_nu=
                                     // 3,E_LCF_RDP_nu= 4,E_LCF_ALCA_nu=
                                     // 5,E_LCF_AOLC_nu= 6, E_LCF_ESA_nu= 7
    float32 fCycleTimeVeh_sec;       // cycle time, range[0.001,0.03]
    float32 fEgoVelX_mps;            // ego vehicle VelX, [-20, 150]
    float32 fEPSManualTrqActVal_Nm;  // the acture manual torque value from EPS,
                                     // [-100,100]
    uint8 uiTrajGuiQualifier_nu;  // qualifier value of trajectory guidence, The
                                  // qualifier indicates if/how the target
                                  // curvature should be considered, [0,5]
                                  // .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ	= 1,
                                  // E_LCF_TGQ_REQ_FREEZE= 3, E_LCF_TGQ_REQ_FFC
                                  // = 4, E_LCF_TGQ_REQ_REFCHNG= 5
    uint8 uiSysStateLCF_nu;       // lateral control function system state enum
                                  // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                                  // E_LCF_SYSSTATE_DISABLED = 1;
                                  // E_LCF_SYSSTATE_PASSIVE = 2;
                                  // E_LCF_SYSSTATE_REQUEST = 3;
                                  // E_LCF_SYSSTATE_CONTROLLING = 4;
    // E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
    // = 6; \n\n
    boolean bLCOTriggerReplan;  // trigger for replanning, [0,1]
    boolean bTrajecotryEnd;  // the check result of whether planned trajectory
                             // have already finished since last replan. [0 ...
                             // 1]
    float32 fCtrlErrDistY_met;        // todo ,[-100-100]
    float32 fCtrlErrHeadAglPrev_rad;  // todo,[-6-6]
    boolean bSysStOffLatDMC;          // system state of lateral DMC module
    // ,[0-1],Indicates TRUE if DMC_LAT_STATUS = 0
    // (LCF_LADMC_OFF)
    float32 fPredTimeCurve_sec;    // todo,[0-60]
    float32 fPredTimeHeadAng_sec;  // todo, [0-60]
    boolean
        bLTATriggerReplan;   // trigger replan signal from CSCLTA module,[0-1]
    uint8 uiLatCtrlMode_nu;  // lateral control mode, [0,8]
                             // ,E_TJASTM_LATCTRLMD_PASSIVE=
                             // 0,E_TJASTM_LATCTRLMD_LC=
                             // 1,E_TJASTM_LATCTRLMD_OF=
                             // 2,E_TJASTM_LATCTRLMD_CMB=
                             // 3,E_TJASTM_LATCTRLMD_SALC=
                             // 4,E_TJASTM_LATCTRLMD_LC_RQ=
                             // 5,E_TJASTM_LATCTRLMD_OF_RQ=
                             // 6,E_TJASTM_LATCTRLMD_CMB_RQ= 7,
                             // E_TJASTM_LATCTRLMD_SALC_RQ= 8
    float32 fDevDistY_met;   // vehicle ego lateral distance in right corridor
                             // coordinate system,[-15-15]
    float32 fDevHeadingAngle_rad;  // vehicle heading angle in right corridor
                                   // coordinate system, [-0.78539816 ...
                                   // 0.78539816]
    float32 fTargetPathY0_met;  // lateral position of the target path (target
                                // corridor with X0 = 0),[-10 ... 10]
    float32 fTargetPathHeading_rad;  // heading angle of the target path (target
                                     // corridor with X0 = 0),[-0.78539816 ...
                                     // 0.78539816]
    boolean bSysStReqLatDMC;         // Indicates TRUE if DMC_LAT_STATUS = 6
                                     // (LCF_LADMC_REQ),[0-1]
} TRJPLN_CalculationEnableInReq_t;

typedef struct {
    boolean bTrajPlanEnble;        // trajectory plan enable signal ,[0-1]
    boolean bTrigTrajReplan;       // trajectory replan enable signal ,[0-1]
    boolean bReplanModeArcLength;  // replan mode with arc length switch while
                                   // ego velocity less than 25kmp ,[0-1]
    boolean bReplanCurValues;      // replan mode with curvature switch ,[0-1]
    boolean bReplanTgtValues;    // we will replan target values while plan mode
                                 // changed or input replan signal is TRUE from
                                 // other module ,[0-1]
    boolean bTrigCustFctChange;  // set bTrigCustFctChange TRUE while last cycle
                                 // and current cycle both are CtrlActived, and
                                 // are actived by different
                                 // ControllingFunction, [0-1]
    boolean bTrajGuiQuChange;    // Signal indicates the trajectory guidance
                                 // qualifier change,[0-1]
    boolean bTrigCustFctActn;    // same value with bReplanCurValues, [0-1]
    boolean bTrigReplanTgtTraj;  // whether target trajectory replaned is
                                 // triggered, set true if bReplanTgtValues ||
                                 // bReplanCurValues, [0-1]
    boolean bEnblSpecPlanStrategy;  // we need a special plan strategy for TJA
                                    // while TJA function is controlling, [0-1]
    float32 fDelayVehGui_sec;       // vehichle delay time from the lookup talbe
    // result of ego velocity float32-todo ,[0-60]
    boolean bTrigLargeDeviation;  // Trigger indicates if the deviation is too
                                  // large,[0-1]
    float32 fPredictionTimeCrv_sec;   // Prediction time of the curvature,
                                      // saterated, ramped up,[0-1]
    float32 fPredictionTimeHead_sec;  // Prediction time of the heading,
                                      // saturated, ramped up,[0-1]
    boolean bCorridorJumpDetected;    // Boolean indicates if there is jump in
                                      // corridor info,[0-1]
    boolean bLatDMCReqFinished;       // Indicates if DMC and EPS handshake is
                                      // finished, [0-1]
} TRJPLN_CalculationEnableOutPro_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TRJPLN_CalculationEnable_Exec(
    const TRJPLN_CalculationEnableInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_CalculationEnableOutPro_t* proPorts,
    TRJPLN_CalcEnableDebug_t* debug);
void TRJPLN_CEN_DebugOutput(TRJPLN_CalculationEnableOutPro_t proPorts,
                            TRJPLN_CalcEnableDebug_t* debug);
void LCF_TRJPLN_CalculationReset_Exec(void);

void TRJPLN_CEN_CalcTriggerTrajPlan(
    const TRJPLN_CalculationEnableInReq_t sCalEnableInput,
    TRJPLN_CalculationEnableOutPro_t* proPorts);

void TRJPLN_CEN_TrajGuiQuChange(uint8 uiLatCtrlMode,
                                boolean bSysStOffLatDMC,
                                uint8 uiTrajGuiQualifier,
                                boolean* bTrigReferChange,
                                boolean* bTrigOBF,
                                boolean* bTrajGuiQuChange);

void TRJPLN_CEN_CalcTrigDrvImplusSteer(uint8 uiControllingFunction,
                                       float32 fCtrlErrDistY,
                                       float32 fCtrlErrHeadAglPrev,
                                       float32 fManuTrqActualVal_Nm,
                                       float32 fSysCycleTimeVeh_sec,
                                       boolean* bTrigLargeDeviation);

void TRJPLN_CEN_SysStateVldCheck(uint8 uiSysStateLCF_nu,
                                 boolean bSysStReqLatDMC,
                                 uint8 uiControllingFunction_nu,
                                 boolean* bEnblSpecPlanStrategy,
                                 boolean* bTrigCustFctChange,
                                 boolean* bActiveCtrl,
                                 boolean* bLatDMCReqFinished);

void TRJPLN_CEN_CorridorJumpCheck(float32 fDevDistY_met,
                                  float32 fDevHeading_rad,
                                  float32 fTargetPathY0_met,
                                  float32 fTargetPathHeading_rad,
                                  uint8 uiControllingFunction,
                                  uint8 uiLatCtrlMode,
                                  boolean bTrigCustFctChange,
                                  boolean* bCorridorJumpDetected);

void TRJPLN_CEN_CalcDelayPredVehCtrl(float32 fSysCycleTimeVeh_sec,
                                     float32 fVehVelX_mps,
                                     boolean bReplanCurValues,
                                     float32 fPredictTimeCrv_sec,
                                     float32 fPredTimeHeading_sec,
                                     float32* fDelayVehGui_sec,
                                     float32* fPredictionTimeCrv_sec,
                                     float32* fPredictionTimeHead_sec);

void TRJPLN_CEN_CalcTimeAndTrigger(float32 fTrajPlanInvokeTime_sec,
                                   boolean bTrajPlanEnable,
                                   boolean bTrigReplanTgtTraj,
                                   float32 fSysCycleTimeVeh_sec,
                                   boolean bTrajEnd,
                                   boolean* bTrigTrajReplan);

#ifdef __cplusplus
}
#endif
#endif
