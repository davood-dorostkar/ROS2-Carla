/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trjpln_calculation_enable.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean TRJPLN_Sb_ReplanModeArcLength =
    FALSE;  // used in function TRJPLN_CEN_CalcTriggerTrajPlan
static boolean TRJPLN_Sb_ActiveCtrlHistory =
    FALSE;  // used in function TRJPLN_CEN_CalcTriggerTrajPlan
static uint8 TRJPLN_Se_SysStateVldCtrlFunc_nu =
    0u;  // used in function TRJPLN_CEN_SysStateVldCheck
static boolean TRJPLN_Sb_SysStateVldActCtrl_nu =
    FALSE;  // used in function TRJPLN_CEN_SysStateVldCheck
static boolean TRJPLN_Sb_SysStateVldReqDMC_nu =
    FALSE;  // used in function TRJPLN_CEN_SysStateVldCheck
static float32 TRJPLN_Sf_CorriJumpDevDistY_met =
    0.f;  // used in function TRJPLN_CEN_CorridorJumpCheck
static float32 TRJPLN_Sf_CorriJumpDTgtPathY0_met =
    0.f;  // used in function TRJPLN_CEN_CorridorJumpCheck
static float32 TRJPLN_Sf_CorriJumpDevHeading_rad =
    0.f;  // used in function TRJPLN_CEN_CorridorJumpCheck
static float32 TRJPLN_Sf_CorriJumpTgtPathHead_rad =
    0.f;  // used in function TRJPLN_CEN_CorridorJumpCheck
static uint8 TRJPLN_Se_CorriJumpLatCtrlMode_nu =
    0u;  // used in function TRJPLN_CEN_CorridorJumpCheck
static uint8 TRJPLN_Se_CorriJumpControllingFunc_nu =
    0u;  // used in function TRJPLN_CEN_CorridorJumpCheck
static boolean TRJPLN_Sb_TrajGuiEdgRisCheck_nu =
    FALSE;  // used in function TRJPLN_CEN_TrajGuiQuChange
static boolean TRJPLN_Sb_TrajGuiEdgeBICheck_nu =
    FALSE;  // used in function TRJPLN_CEN_TrajGuiQuChange
static uint8 TRJPLN_Se_TrajGuiLatCtrlMode_nu =
    0u;  // used in function TRJPLN_CEN_TrajGuiQuChange
static uint8 TRJPLN_Se_TrajGuiQualiferVal_nu =
    0u;  // used in function TRJPLN_CEN_TrajGuiQuChange
static float32 TRJPLN_Sf_DrvImplusSteerMaxTimer_sec =
    0.f;  // used in function TRJPLN_CEN_CalcTrigDrvImplusSteer
static float32 TRJPLN_Sf_DrvImplusSteerMinTimer_sec =
    0.f;  // used in function TRJPLN_CEN_CalcTrigDrvImplusSteer
static boolean TRJPLN_Sb_DrvImplusSteerMaxTorEdg_nu =
    FALSE;  // used in function TRJPLN_CEN_CalcTrigDrvImplusSteer
static boolean TRJPLN_Sb_DrvImplusSteerErrCheckEdg_nu =
    FALSE;  // used in function TRJPLN_CEN_CalcTrigDrvImplusSteer
static boolean TRJPLN_Sb_DrvImplusSteerMinTorEdg_nu =
    FALSE;  // used in function TRJPLN_CEN_CalcTrigDrvImplusSteer
static float32 TRJPLN_Sf_CalcDelayPredDeadTime_sec =
    0.f;  // used in function TRJPLN_CEN_CalcDelayPredVehCtrl
static float32 TRJPLN_Sf_CalcDelayPredTimeCrv_sec =
    0.f;  // used in function TRJPLN_CEN_CalcDelayPredVehCtrl
static float32 TRJPLN_Sf_CalcDelayPredTimeHead_sec =
    0.f;  // used in function TRJPLN_CEN_CalcDelayPredVehCtrl
static boolean TRJPLN_Sb_CalcTimeTrigTrajRep_nu =
    FALSE;  // used in function TRJPLN_CEN_CalcTimeAndTrigger
static boolean TRJPLN_Sb_CalcTimeTrigTrajEnd_nu =
    FALSE;  // used in function TRJPLN_CEN_CalcTimeAndTrigger
static float32 TRJPLN_Sf_CalcTimeTrigLastTime_sec =
    0.f;  // used in function TRJPLN_CEN_CalcTimeAndTrigger
static float32 TRJPLN_Sf_CalcTimeTrigCurrTime_sec =
    0.f;  // used in function TRJPLN_CEN_CalcTimeAndTrigger
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname: LCF_TRJPLN_CalculationEnable_Exec */ /*!

                          @brief: trajectory plan enable, replan check

                          @description: trajectory plan is enabled while system
                          state check passed, and
                          replan
                          would be triggered while situation changed from
                          history

                          @param[in]
                          const TRJPLN_CalculationEnableInReq_t* reqPorts:
                          calculation enable
                          check module input
                          const TRJPLN_TrajectoryPlanParam_t* paras: system
                          parameters
                          @param[out]
                          TRJPLN_CalculationEnableOutPro_t* proPorts:
                          calculation enable check
                          result
                          @return
                          @startuml
                          start
                          :TRJPLN_CEN_CalcTriggerTrajPlan;
                          note:check trajectory plan trigger relate logic
                          :TRJPLN_CEN_CalcDelayPredVehCtrl;
                          note:make a gradient limit to every input \nprediction
                          time.
                          :TRJPLN_CEN_CalcTimeAndTrigger;
                          note:whether trajectory need a replan trigger
                          :TRJPLN_CEN_DebugOutput;
                          note:set calculation enable module's output to \ndebug
                          structure for datalogger
                          needed
                          end
                          @enduml
                          *****************************************************************************/
void LCF_TRJPLN_CalculationEnable_Exec(
    const TRJPLN_CalculationEnableInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_CalculationEnableOutPro_t* proPorts,
    TRJPLN_CalcEnableDebug_t* debug) {
    // check trajectory plan trigger relate logic
    TRJPLN_CEN_CalcTriggerTrajPlan(*reqPorts, proPorts);
    // calculate predicted delay time with gradient limit
    TRJPLN_CEN_CalcDelayPredVehCtrl(
        reqPorts->fCycleTimeVeh_sec, reqPorts->fEgoVelX_mps,
        proPorts->bReplanCurValues, reqPorts->fPredTimeCurve_sec,
        reqPorts->fPredTimeHeadAng_sec, &proPorts->fDelayVehGui_sec,
        &proPorts->fPredictionTimeCrv_sec, &proPorts->fPredictionTimeHead_sec);
    // trajectory replan check
    TRJPLN_CEN_CalcTimeAndTrigger(
        paras->fCycleTime_sec, proPorts->bTrajPlanEnble,
        proPorts->bTrigReplanTgtTraj, reqPorts->fCycleTimeVeh_sec,
        reqPorts->bTrajecotryEnd, &proPorts->bTrigTrajReplan);

    // debug output
    TRJPLN_CEN_DebugOutput(*proPorts, debug);
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_DebugOutput */ /*!

                                     @brief: output calculation enable module
                                     output to debug structure

                                     @description:

                                     @param[in]
                                     @param[out]

                                     @return
                                     *****************************************************************************/
void TRJPLN_CEN_DebugOutput(TRJPLN_CalculationEnableOutPro_t proPorts,
                            TRJPLN_CalcEnableDebug_t* debug) {
    debug->bCorridorJumpDetected = proPorts.bCorridorJumpDetected;
    debug->bEnblSpecPlanStrategy = proPorts.bEnblSpecPlanStrategy;
    debug->bLatDMCReqFinished = proPorts.bLatDMCReqFinished;
    debug->bReplanCurValues = proPorts.bReplanCurValues;
    debug->bReplanModeArcLength = proPorts.bReplanModeArcLength;
    debug->bReplanTgtValues = proPorts.bReplanTgtValues;
    debug->bTrajGuiQuChange = proPorts.bTrajGuiQuChange;
    debug->bTrajPlanEnble = proPorts.bTrajPlanEnble;
    debug->bTrigCustFctActn = proPorts.bTrigCustFctActn;
    debug->bTrigCustFctChange = proPorts.bTrigCustFctChange;
    debug->bTrigLargeDeviation = proPorts.bTrigLargeDeviation;
    debug->bTrigReplanTgtTraj = proPorts.bTrigReplanTgtTraj;
    debug->bTrigTrajReplan = proPorts.bTrigTrajReplan;
    debug->fDelayVehGui_sec = proPorts.fDelayVehGui_sec;
    debug->fPredictionTimeCrv_sec = proPorts.fPredictionTimeCrv_sec;
    debug->fPredictionTimeHead_sec = proPorts.fPredictionTimeHead_sec;
}

/*****************************************************************************
  Functionname: LCF_TRJPLN_CalculationReset_Exec */ /*!

                           @brief: global value reset function of calculation
                           enable module

                           @description: global value reset function of
                           calculation enable module

                           @param[in]
                           @param[out]

                           @return
                           *****************************************************************************/
void LCF_TRJPLN_CalculationReset_Exec(void) {
    TRJPLN_Sb_ReplanModeArcLength = FALSE;
    TRJPLN_Sb_ActiveCtrlHistory = FALSE;
    TRJPLN_Se_SysStateVldCtrlFunc_nu = 0u;
    TRJPLN_Sb_SysStateVldActCtrl_nu = FALSE;
    TRJPLN_Sb_SysStateVldReqDMC_nu = FALSE;
    TRJPLN_Sf_CorriJumpDevDistY_met = 0.f;
    TRJPLN_Sf_CorriJumpDTgtPathY0_met = 0.f;
    TRJPLN_Sf_CorriJumpDevHeading_rad = 0.f;
    TRJPLN_Sf_CorriJumpTgtPathHead_rad = 0.f;
    TRJPLN_Se_CorriJumpLatCtrlMode_nu = 0u;
    TRJPLN_Se_CorriJumpControllingFunc_nu = 0u;
    TRJPLN_Sb_TrajGuiEdgRisCheck_nu = FALSE;
    TRJPLN_Sb_TrajGuiEdgeBICheck_nu = FALSE;
    TRJPLN_Se_TrajGuiLatCtrlMode_nu = 0u;
    TRJPLN_Se_TrajGuiQualiferVal_nu = 0u;
    TRJPLN_Sf_DrvImplusSteerMaxTimer_sec = 0.f;
    TRJPLN_Sf_DrvImplusSteerMinTimer_sec = 0.f;
    TRJPLN_Sb_DrvImplusSteerMaxTorEdg_nu = FALSE;
    TRJPLN_Sb_DrvImplusSteerErrCheckEdg_nu = FALSE;
    TRJPLN_Sb_DrvImplusSteerMinTorEdg_nu = FALSE;
    TRJPLN_Sf_CalcDelayPredDeadTime_sec = 0.f;
    TRJPLN_Sf_CalcDelayPredTimeCrv_sec = 0.f;
    TRJPLN_Sf_CalcDelayPredTimeHead_sec = 0.f;
    TRJPLN_Sb_CalcTimeTrigTrajRep_nu = FALSE;
    TRJPLN_Sb_CalcTimeTrigTrajEnd_nu = FALSE;
    TRJPLN_Sf_CalcTimeTrigLastTime_sec = 0.f;
    TRJPLN_Sf_CalcTimeTrigCurrTime_sec = 0.f;
}

/*****************************************************************************
  Functionname:TRJPLN_CEN_CalcTriggerTrajPlan */ /*!

                              @brief: trajectory plan/replan/mode calculate

                              @description: we will check trajectory planning is
                              enabled, replan, mode change,
                              disabled
                              by the system state change

                              @param[in]
                              const TRJPLN_CalculationEnableInReq_t
                              sCalEnableInput: lateral
                              control mode, function, system state...
                              @param[out]
                              TRJPLN_CalculationEnableOutPro_t* proPorts:
                              trajectory plan trigger
                              check result
                              @return
                              @uml
                              @startuml
                              start
                              :lateral system state check;
                              :the controling requesting data jump check
                              compares with last cycle;
                              :trajecotry qualifier status change check;
                              :we will check if the driver is trying to control
                              steer wheel;
                              :trajectory mode change to ArcLength based check;
                              :trajectory plan enable check;
                              :trajectory value replan trigger check;
                              end
                              @enduml
                              *****************************************************************************/
void TRJPLN_CEN_CalcTriggerTrajPlan(
    const TRJPLN_CalculationEnableInReq_t sCalEnableInput,
    TRJPLN_CalculationEnableOutPro_t* proPorts) {
    boolean bActiveCtrl = FALSE;
    boolean bTrigReferChange = FALSE;
    boolean bTrigOBF = FALSE;
    // lateral system state check
    TRJPLN_CEN_SysStateVldCheck(
        sCalEnableInput.uiSysStateLCF_nu, sCalEnableInput.bSysStReqLatDMC,
        sCalEnableInput.uiControllingFunction_nu,
        &proPorts->bEnblSpecPlanStrategy, &proPorts->bTrigCustFctChange,
        &bActiveCtrl, &proPorts->bLatDMCReqFinished);
    // the controling requesting data jump check compares with last cycle
    TRJPLN_CEN_CorridorJumpCheck(
        sCalEnableInput.fDevDistY_met, sCalEnableInput.fDevHeadingAngle_rad,
        sCalEnableInput.fTargetPathY0_met,
        sCalEnableInput.fTargetPathHeading_rad,
        sCalEnableInput.uiControllingFunction_nu,
        sCalEnableInput.uiLatCtrlMode_nu, proPorts->bTrigCustFctChange,
        &proPorts->bCorridorJumpDetected);
    // trajecotry qualifier status change check
    TRJPLN_CEN_TrajGuiQuChange(
        sCalEnableInput.uiLatCtrlMode_nu, sCalEnableInput.bSysStOffLatDMC,
        sCalEnableInput.uiTrajGuiQualifier_nu, &bTrigReferChange, &bTrigOBF,
        &proPorts->bTrajGuiQuChange);
    // we will check if the driver is trying to control steer wheel
    TRJPLN_CEN_CalcTrigDrvImplusSteer(sCalEnableInput.uiControllingFunction_nu,
                                      sCalEnableInput.fCtrlErrDistY_met,
                                      sCalEnableInput.fCtrlErrHeadAglPrev_rad,
                                      sCalEnableInput.fEPSManualTrqActVal_Nm,
                                      sCalEnableInput.fCycleTimeVeh_sec,
                                      &proPorts->bTrigLargeDeviation);

    // calculate trigger and enable check

    // the optimal trajectory calculate mode is switched to ArcLength based mode
    // while in low speed situation
    proPorts->bReplanModeArcLength = sCalEnableInput.fEgoVelX_mps <
                                     (0.277778f * TPLCEN_VMINTIMEBASEDTRAJ_KPH);
    // the trajectory plan is enable while the control is actived by lateral
    // control function
    proPorts->bTrajPlanEnble = bActiveCtrl && TPLCEN_TRAJPLANENABLE_NU;
    // we will replan target values while (condition 1 && (condition 2.1 ||
    // condition 2.2)):
    // 1. bTrajPlanEnble is TRUE;
    // 2.1 bReplanModeArcLength changed with last cycle
    // 2.2  replan trigger is TRUE in lateral control module and lantency
    // compensation module
    proPorts->bReplanTgtValues =
        (proPorts->bTrajPlanEnble &&
         ((proPorts->bReplanModeArcLength != TRJPLN_Sb_ReplanModeArcLength) ||
          (sCalEnableInput.bLCOTriggerReplan &&
           sCalEnableInput.bLTATriggerReplan && TPLCEN_LCOREPLANENABLE_NU)));

    boolean bActveCtrlEdgRising =
        TUE_CML_RisingEdgeSwitch(bActiveCtrl, &TRJPLN_Sb_ActiveCtrlHistory);
    proPorts->bReplanCurValues =
        (proPorts->bTrajPlanEnble &&
         (bActveCtrlEdgRising || bTrigReferChange ||
          proPorts->bTrajGuiQuChange || proPorts->bTrigLargeDeviation ||
          bTrigOBF || proPorts->bCorridorJumpDetected ||
          proPorts->bLatDMCReqFinished));
    proPorts->bTrigCustFctActn = proPorts->bReplanCurValues;
    // whether target trajectory replaned is triggered
    proPorts->bTrigReplanTgtTraj =
        proPorts->bReplanTgtValues || proPorts->bReplanCurValues;

    TRJPLN_Sb_ReplanModeArcLength = proPorts->bReplanModeArcLength;
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_CalcDelayPredVehCtrl */ /*!

                            @brief: calculate prediction time based on input

                            @description: we will calculate gradient limit
                            value, and make a gradient limit
                            to every
                            input prediction time.

                            @param[in]
                            loat32 fSysCycleTimeVeh_sec: cycle time
                            float32 fVehVelX_mps: ego velocity
                            boolean bReplanCurValues: replan triggered
                            float32 fPredictTimeCrv_sec: predict time of
                            curvature from input of
                            trajectory plan moudle
                            float32 fPredTimeHeading_sec: predict time of
                            heading angle from input
                            of trajectory plan moudle
                            @param[out]
                            float32* fDelayVehGui_sec: vehichle delay time from
                            the lookup talbe
                            result of ego velocity
                            float32* fPredictionTimeCrv_sec: gradient limit
                            result of curvature
                            prediction time
                            float32* fPredictionTimeHead_sec: gradient limit
                            result of heading
                            angle prediction time
                            @return
                            @uml
                            @startuml
                            start
                            :delay time calculate based on ego velocity lookup
                            result;
                            :delay time gradientlimit calculate;
                            :prediction time threshold check and gradient limit
                            calculate;
                            if(replan happend) then (yes)
                            :output prediction time directly without gradient
                            limit;
                            else(no)
                            :output prediction time with gradient limit;
                            endif
                            end
                            @enduml
                            *****************************************************************************/
void TRJPLN_CEN_CalcDelayPredVehCtrl(float32 fSysCycleTimeVeh_sec,
                                     float32 fVehVelX_mps,
                                     boolean bReplanCurValues,
                                     float32 fPredictTimeCrv_sec,
                                     float32 fPredTimeHeading_sec,
                                     float32* fDelayVehGui_sec,
                                     float32* fPredictionTimeCrv_sec,
                                     float32* fPredictionTimeHead_sec) {
    // calculate gradient limit function input and limit value
    float32 afTableInputX[TPLCEN_DEADTIMEVEHCTRL_TABLENUM_NU] =
        TPLCEN_POTVECDYNVELX_TABLEX_MPS;
    float32 afTableInputY[TPLCEN_DEADTIMEVEHCTRL_TABLENUM_NU] =
        TPLCEN_DEADTIMEVEHCTRL_TABLEY_SEC;
    float32 fDeadTimeInput_sec =
        TUE_CML_LookUpTable2D(fVehVelX_mps, afTableInputX, afTableInputY,
                              TPLCEN_DEADTIMEVEHCTRL_TABLENUM_NU);
    float32 fDeadTimeLimit_nu = TUE_CML_MinMax(0.f, 1.f, fDeadTimeInput_sec) /
                                TPLCEN_DTGRADLIMITDURATION_SEC;

    float32 fTimeCrvInput_sec =
        TUE_CML_MinMax(0.f, TPLCEN_MAXPREDICTIONTIME_SEC, fPredictTimeCrv_sec);
    float32 fTimeCrvLimit_nu = fTimeCrvInput_sec / TPLCEN_PTCRVGRADLIMITDUR_SEC;

    float32 fTimeHeadInput_sec =
        TUE_CML_MinMax(0.f, TPLCEN_MAXPREDICTIONTIME_SEC, fPredTimeHeading_sec);
    float32 fTimeHeadLimit_nu =
        fTimeHeadInput_sec / TPLCEN_PTHEADGRADLIMITDUR_SEC;

    // reset check
    if (bReplanCurValues) {
        *fDelayVehGui_sec = fDeadTimeInput_sec;
        *fPredictionTimeCrv_sec = fTimeCrvInput_sec;
        *fPredictionTimeHead_sec = fTimeHeadInput_sec;
    } else {
        // gradient limit function for time value with calculated input and
        // limit
        *fDelayVehGui_sec = TUE_CML_GradLimit_M(
            fDeadTimeInput_sec, fDeadTimeLimit_nu, -fDeadTimeLimit_nu,
            fSysCycleTimeVeh_sec, TRJPLN_Sf_CalcDelayPredDeadTime_sec);
        *fPredictionTimeCrv_sec = TUE_CML_GradLimit_M(
            fTimeCrvInput_sec, fTimeCrvLimit_nu, -fTimeCrvLimit_nu,
            fSysCycleTimeVeh_sec, TRJPLN_Sf_CalcDelayPredTimeCrv_sec);
        *fPredictionTimeHead_sec = TUE_CML_GradLimit_M(
            fTimeHeadInput_sec, fTimeHeadLimit_nu, -fTimeHeadLimit_nu,
            fSysCycleTimeVeh_sec, TRJPLN_Sf_CalcDelayPredTimeHead_sec);
    }

    TRJPLN_Sf_CalcDelayPredDeadTime_sec = *fDelayVehGui_sec;
    TRJPLN_Sf_CalcDelayPredTimeCrv_sec = *fPredictionTimeCrv_sec;
    TRJPLN_Sf_CalcDelayPredTimeHead_sec = *fPredictionTimeHead_sec;
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_CalcTimeAndTrigger */ /*!

                              @brief: trajectory replan trigger signal check

                              @description:  we will make trajectory replan
                              while (condition 1 && (condition
                              2.1 || condition 2.2 || condition 2.3))
                              condition 1: trajectory replan enabled
                              condition 2.1: trajectory end signal edge rising
                              passed
                              condition 2.2: replan target trajectory triggered
                              condition 2.3: trajectory replan timer check

                              @param[in]
                              float32 fTrajPlanInvokeTime_sec: the system invoke
                              cycle time of
                              trajectory plan module
                              boolean bTrajPlanEnable: trajectory plan enable
                              check result
                              boolean bTrigReplanTgtTraj: target trajectory
                              replan triggered
                              float32 fSysCycleTimeVeh_sec: cycle time
                              boolean bTrajEnd: trajectory plan end signal, the
                              trajectory should
                              be replaned while trajectory plan end
                              @param[out]
                              boolean* bTrigTrajReplan: trajectory replan check
                              by calculation
                              enable module
                              @return
                              *****************************************************************************/
void TRJPLN_CEN_CalcTimeAndTrigger(float32 fTrajPlanInvokeTime_sec,
                                   boolean bTrajPlanEnable,
                                   boolean bTrigReplanTgtTraj,
                                   float32 fSysCycleTimeVeh_sec,
                                   boolean bTrajEnd,
                                   boolean* bTrigTrajReplan) {
    // timer logic, bTimerValue is TRUE while coutner expired, otherwise output
    // FALSE
    boolean bTimerValue = TRJPLN_Sf_CalcTimeTrigLastTime_sec >
                          (fTrajPlanInvokeTime_sec - fSysCycleTimeVeh_sec);
    TRJPLN_Sf_CalcTimeTrigLastTime_sec =
        (TRJPLN_Sb_CalcTimeTrigTrajRep_nu || bTimerValue)
            ? fSysCycleTimeVeh_sec
            : TRJPLN_Sf_CalcTimeTrigCurrTime_sec;
    TRJPLN_Sf_CalcTimeTrigCurrTime_sec =
        TRJPLN_Sf_CalcTimeTrigLastTime_sec + fSysCycleTimeVeh_sec;

    *bTrigTrajReplan =
        bTrajPlanEnable &&
        (TUE_CML_RisingEdgeSwitch(bTrajEnd,
                                  &TRJPLN_Sb_CalcTimeTrigTrajEnd_nu) ||
         bTrigReplanTgtTraj || (bTimerValue && TPLCEN_CYCENABLE_NU));
    TRJPLN_Sb_CalcTimeTrigTrajRep_nu = *bTrigTrajReplan;
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_SysStateVldCheck */ /*!

                                @brief: check our system enable state

                                @description: 1. the specPlanStrategy is on
                                while TJA is controlling
                                    2. check whether control is actived,
                                    3. whther controlling function changed,
                                    4. did Lat DMC control request is finished

                                @param[in]
                                    uint8 uiSysStateLCF_nu: LCF system state,
                                contrlling, rrequest,
                                disabled ...
                                    boolean bSysStReqLatDMC: did our system is
                                requesting lateral DMC
                                control
                                    uint8 uiControllingFunction_nu: which
                                lateral function is enabled,
                                TJA? LDP?...
                                @param[out]
                                    boolean* bEnblSpecPlanStrategy: did we need
                                a special plan
                                strategy for TJA
                                    boolean* bTrigCustFctChange: did the
                                controlling function changed
                                in current cycle
                                    boolean* bActiveCtrl: is lateral controlling
                                is actived
                                    boolean* bLatDMCReqFinished: whther Lateral
                                DMC request finished
                                in current cycles
                                @return
                                *****************************************************************************/
void TRJPLN_CEN_SysStateVldCheck(uint8 uiSysStateLCF_nu,
                                 boolean bSysStReqLatDMC,
                                 uint8 uiControllingFunction_nu,
                                 boolean* bEnblSpecPlanStrategy,
                                 boolean* bTrigCustFctChange,
                                 boolean* bActiveCtrl,
                                 boolean* bLatDMCReqFinished) {
    boolean bSysStateLCFCheck =
        ((uiSysStateLCF_nu == E_LCF_SYSSTATE_CONTROLLING) ||
         (uiSysStateLCF_nu == E_LCF_SYSSTATE_REQUEST) ||
         (TPLCEN_ACTIVEBYRAMPOUT_NU ? uiSysStateLCF_nu == E_LCF_SYSSTATE_RAMPOUT
                                    : FALSE));
    // set bEnblSpecPlanStrategy TRUE while we are in TJA controlling and LCF is
    // requesting or controlling state
    *bEnblSpecPlanStrategy =
        TPLCEN_ALLOWSPEPLANSTRATEGY_NU
            ? ((uiControllingFunction_nu == E_LCF_TJA_nu) && bSysStateLCFCheck)
            : FALSE;
    // set bActiveCtrl TRUE while the LCF state is controlling state and
    // controlling function is not OFF state
    *bActiveCtrl =
        bSysStateLCFCheck && (uiControllingFunction_nu != E_LCF_OFF_nu);
    // set bTrigCustFctChange TRUE while last cycle and current cycle both are
    // CtrlActived, and are actived by different ControllingFunction
    *bTrigCustFctChange =
        TPLCEN_ENABLEFCNCHNGDETECT_NU
            ? (TRJPLN_Sb_SysStateVldActCtrl_nu && *bActiveCtrl &&
               (uiControllingFunction_nu != TRJPLN_Se_SysStateVldCtrlFunc_nu))
            : TRUE;
    // set bLatDMCReqFinished TRUE while ReqDMC is actived in last cycle, and
    // not actived in current cycle, and our controlling function is not TJA
    *bLatDMCReqFinished = (uiControllingFunction_nu != E_LCF_TJA_nu) &&
                          TRJPLN_Sb_SysStateVldReqDMC_nu && !bSysStReqLatDMC &&
                          TPLCEN_REINIWRTDMCREQ_NU;

    TRJPLN_Sb_SysStateVldReqDMC_nu = bSysStReqLatDMC;
    TRJPLN_Se_SysStateVldCtrlFunc_nu = uiControllingFunction_nu;
    TRJPLN_Sb_SysStateVldActCtrl_nu = *bActiveCtrl;
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_CorridorJumpCheck */ /*!

                               @brief: controlling request data jump detect
                               logic

                               @description:
                                 1. TPLCEN_USEJUMPDETECTION_NU is TURE:
                               bCorridorJumpDetected is
                               TRUE while there is a big
                                 value change compares with last cycle
                                 2. TPLCEN_USEJUMPDETECTION_NU is FALSE:
                               bCorridorJumpDetected is
                               TRUE while controlling
                                 function changed in current cycle, and is still
                               controlling
                               @param[in]
                                 float32 fDevDistY_met: vehicle ego lateral
                               distance in right
                               corridor coordinate system
                                 float32 fDevHeading_rad:vehicle heading angle
                               in right corridor
                               coordinate system
                                 float32 fTargetPathY0_met: controlling target
                               path PosY0 value
                                 float32 fTargetPathHeading_rad:controlling
                               target path heading
                               angle
                                 uint8 uiControllingFunction: controlling
                               function enum value
                                 uint8 uiLatCtrlMode: lateral control mode enum
                               value
                                 boolean bTrigCustFctChange: did controlling
                               function changed
                               compare with last cycle
                               @param[out]
                                 boolean* bCorridorJumpDetected: controlling
                               corridor value jump
                               detect result
                               @return
                               *****************************************************************************/
void TRJPLN_CEN_CorridorJumpCheck(float32 fDevDistY_met,
                                  float32 fDevHeading_rad,
                                  float32 fTargetPathY0_met,
                                  float32 fTargetPathHeading_rad,
                                  uint8 uiControllingFunction,
                                  uint8 uiLatCtrlMode,
                                  boolean bTrigCustFctChange,
                                  boolean* bCorridorJumpDetected) {
    if (TPLCEN_USEJUMPDETECTION_NU) {
        boolean bJumpDetectOBFCheck =
            TPLCEN_ENABLE_JUMPDETECTOBF_NU
                ? TRUE
                : !((uiControllingFunction == E_LCF_TJA_nu) &&
                        ((uiLatCtrlMode == E_TJASTM_LATCTRLMD_OF_RQ) &&
                         !TPLCEN_ENABLEJUMPDETECTOFRQ_NU) ||
                    (uiLatCtrlMode == E_TJASTM_LATCTRLMD_OF));
        // big jump check of controlling related data
        boolean bValueJumpCheck =
            (fABS(fDevDistY_met - TRJPLN_Sf_CorriJumpDevDistY_met) >=
             TPLCEN_REINIDELTADISTY_MET) ||
            (fABS(fTargetPathY0_met - TRJPLN_Sf_CorriJumpDTgtPathY0_met) >=
             TPLCEN_REINIDELTADISTY_MET) ||
            (fABS(fDevHeading_rad - TRJPLN_Sf_CorriJumpDevHeading_rad) >=
             TPLCEN_REINIDELTAHEAD_RAD) ||
            (fABS(fTargetPathHeading_rad -
                  TRJPLN_Sf_CorriJumpTgtPathHead_rad) >=
             TPLCEN_REINIDELTAHEAD_RAD);
        *bCorridorJumpDetected =
            (bTrigCustFctChange && (bJumpDetectOBFCheck && bValueJumpCheck));
    } else {
        // output bCorridorJumpDetected TRUE while controlling function changed
        // in current cycle, and is still controlling
        boolean bReplanByTJACheck =
            TPLCEN_REPLANBYTJA2X_NU
                ? TRUE
                : !(((TRJPLN_Se_CorriJumpLatCtrlMode_nu ==
                      E_TJASTM_LATCTRLMD_LC) ||
                     (TRJPLN_Se_CorriJumpLatCtrlMode_nu ==
                      E_TJASTM_LATCTRLMD_LC_RQ) ||
                     (TRJPLN_Se_CorriJumpLatCtrlMode_nu ==
                      E_TJASTM_LATCTRLMD_CMB) ||
                     (TRJPLN_Se_CorriJumpLatCtrlMode_nu ==
                      E_TJASTM_LATCTRLMD_CMB_RQ)) &&
                    (TRJPLN_Se_CorriJumpControllingFunc_nu == E_LCF_TJA_nu));
        *bCorridorJumpDetected = (bTrigCustFctChange && bReplanByTJACheck);
        // *bCorridorJumpDetected = bTrigCustFctChange &&
        // (TPLCEN_USEJUMPDETECTION_NU ? (bJumpDetectOBFCheck &&
        // bValueJumpCheck) : bReplanByTJACheck);
    }

    TRJPLN_Se_CorriJumpLatCtrlMode_nu = uiLatCtrlMode;
    TRJPLN_Se_CorriJumpControllingFunc_nu = uiControllingFunction;
    TRJPLN_Sf_CorriJumpDevDistY_met = fDevDistY_met;
    TRJPLN_Sf_CorriJumpDTgtPathY0_met = fTargetPathY0_met;
    TRJPLN_Sf_CorriJumpDevHeading_rad = fDevHeading_rad;
    TRJPLN_Sf_CorriJumpTgtPathHead_rad = fTargetPathHeading_rad;
}

/*****************************************************************************
  Functionname:TRJPLN_CEN_TrajGuiQuChange */ /*!

                                  @brief:check trajectory guidance qualifier
                                  change state

                                  @description: is TrajGuiQualifier state
                                  changed in current cycle compares with
                                  last cycle value

                                  @param[in]
                                        uint8 uiLatCtrlMode: latera control mode
                                        boolean bSysStOffLatDMC:system state of
                                  lateral DMC module
                                  ,[0��1],Indicates TRUE if DMC_LAT_STATUS = 0
                                  (LCF_LADMC_OFF)
                                        uint8 uiTrajGuiQualifier:qualifier value
                                  of trajectory guidence
                                  @param[out]
                                        boolean* bTrigReferChange: output true
                                  while uiTrajGuiQualifier
                                  changed from other state to
                                  E_LCF_TGQ_REQ_REFCHNG mode
                                        boolean* bTrigOBF: output true while
                                  ObjectFollowing mode is
                                  triggered in current cycle
                                        boolean* bTrajGuiQuChange: output true
                                  while current cycle's
                                  GuiQualifier changed to Request while laste
                                  cycle is frezz
                                  @return
                                  *****************************************************************************/
void TRJPLN_CEN_TrajGuiQuChange(uint8 uiLatCtrlMode,
                                boolean bSysStOffLatDMC,
                                uint8 uiTrajGuiQualifier,
                                boolean* bTrigReferChange,
                                boolean* bTrigOBF,
                                boolean* bTrajGuiQuChange) {
    // detect lane change fail senario, return to lane center mode
    boolean bLaneChangeFaild =
        (uiTrajGuiQualifier == E_LCF_TGQ_REQ_CHNGFAIL) &&
        (TRJPLN_Se_TrajGuiQualiferVal_nu == E_LCF_TGQ_REQ_REFCHNG);
    boolean bLaneChangedSLCCancle =
        (uiTrajGuiQualifier == E_LCF_TGQ_REQ_LANECHANGED) &&
        (TRJPLN_Se_TrajGuiQualiferVal_nu != E_LCF_TGQ_REQ_LANECHANGED);
    // refer change trigger check, from other state to E_LCF_TGQ_REQ_REFCHNG
    // mode
    *bTrigReferChange =
        (TUE_CML_RisingEdgeSwitch(uiTrajGuiQualifier == E_LCF_TGQ_REQ_REFCHNG,
                                  &TRJPLN_Sb_TrajGuiEdgRisCheck_nu)) ||
        bLaneChangeFaild || bLaneChangedSLCCancle;

    // check whther ObjectFollowing mode is triggered in current cycle
    boolean bLatCtrlModeCheck =
        (TRJPLN_Se_TrajGuiLatCtrlMode_nu != E_TJASTM_LATCTRLMD_PASSIVE) &&
        ((uiLatCtrlMode == E_TJASTM_LATCTRLMD_OF) ||
         (uiLatCtrlMode == E_TJASTM_LATCTRLMD_OF_RQ)) &&
        (TPLCEN_ALLOWREPLANOBF_NU);
    *bTrigOBF = (TRJPLN_Sb_TrajGuiEdgeBICheck_nu != bLatCtrlModeCheck);

    // TrajGuiQualifier changed in current cycle. current cycle's GuiQualifier
    // changed to Request while laste cycle is frezz
    *bTrajGuiQuChange =
        (uiTrajGuiQualifier == E_LCF_TGQ_REQ) &&
        (TRJPLN_Se_TrajGuiQualiferVal_nu == E_LCF_TGQ_REQ_FREEZE) &&
        (TPLCEN_ACTIVEBYRAMPOUT_NU ? bSysStOffLatDMC : TRUE);

    TRJPLN_Se_TrajGuiQualiferVal_nu = uiTrajGuiQualifier;
    TRJPLN_Sb_TrajGuiEdgeBICheck_nu = bLatCtrlModeCheck;
    TRJPLN_Se_TrajGuiLatCtrlMode_nu = uiLatCtrlMode;
}

/*****************************************************************************
  Functionname: TRJPLN_CEN_CalcTrigDrvImplusSteer */ /*!

                          @brief: we check the manual torque value of driver

                          @description: we will check the manual torque value is
                          too large or too small
                          than we thought.
                          the driver is trying to control steer wheel while the
                          manual torque is
                          too large. the
                          driver is hands off while the manual torque is too
                          small. and, we will
                          check control error
                          deviation value,

                          @param[in]
                          uint8 uiControllingFunction: controlling function,
                          TJA, LDP ...
                          float32 fCtrlErrDistY: control error distance Y in the
                          last cycle
                          float32 fCtrlErrHeadAglPrev:  control error heading
                          angle in the last
                          cycle
                          float32 fManuTrqActualVal_Nm: manual torque value by
                          driver
                          float32 fSysCycleTimeVeh_sec: cycle time
                          @param[out]
                          boolean* bTrigLargeDeviation: whether the large
                          deviation trigger is
                          detected
                          @return
                          *****************************************************************************/
void TRJPLN_CEN_CalcTrigDrvImplusSteer(uint8 uiControllingFunction,
                                       float32 fCtrlErrDistY,
                                       float32 fCtrlErrHeadAglPrev,
                                       float32 fManuTrqActualVal_Nm,
                                       float32 fSysCycleTimeVeh_sec,
                                       boolean* bTrigLargeDeviation) {
    if (TPLCEN_REPLANLARGEERRORMODE_NU != 0u) {
        uint8 uiCheckResult_nu = 0u;
        // the manual torque should no more than TPLCEN_MANUALTRQMIN_NM, and no
        // less than TPLCEN_MANUALTRQMAX_NM
        // which means the driver should not control the steer wheel angle, and
        // he also should not hands off
        boolean bManuTrqMaxCheck = !TUE_CML_TimerRetrigger(
            fSysCycleTimeVeh_sec,
            fManuTrqActualVal_Nm >= TPLCEN_MANUALTRQMAX_NM,
            TPLCEN_MANUALTRQMAXTIME_SEC, &TRJPLN_Sf_DrvImplusSteerMaxTimer_sec);

        boolean bManuTrqMinCheck = !TUE_CML_TimerRetrigger(
            fSysCycleTimeVeh_sec,
            fManuTrqActualVal_Nm <= TPLCEN_MANUALTRQMIN_NM,
            TPLCEN_MANUALTRQMINTIME_SEC, &TRJPLN_Sf_DrvImplusSteerMinTimer_sec);
        // the control error value should not higher than threshold
        boolean bCtrlErrorCheck =
            (fABS(fCtrlErrDistY) >= TPLCEN_ERRDISTY_MET) ||
            (fABS(fCtrlErrHeadAglPrev) >= TPLCEN_ERRHEADAGLPREV_RAD);

        TPLLCO_SetBit(
            0u,
            TUE_CML_RisingEdgeSwitch(bManuTrqMaxCheck && bCtrlErrorCheck,
                                     &TRJPLN_Sb_DrvImplusSteerMaxTorEdg_nu),
            &uiCheckResult_nu);
        TPLLCO_SetBit(
            1u,
            TUE_CML_RisingEdgeSwitch(bCtrlErrorCheck,
                                     &TRJPLN_Sb_DrvImplusSteerErrCheckEdg_nu),
            &uiCheckResult_nu);
        TPLLCO_SetBit(2u, bManuTrqMinCheck && bCtrlErrorCheck,
                      &uiCheckResult_nu);
        TPLLCO_SetBit(
            3u,
            TUE_CML_RisingEdgeSwitch(bManuTrqMinCheck && bCtrlErrorCheck,
                                     &TRJPLN_Sb_DrvImplusSteerMinTorEdg_nu),
            &uiCheckResult_nu);

        *bTrigLargeDeviation =
            (TPLCEN_REPLANLARGEERRORMODE_NU & uiCheckResult_nu) &&
            (uiControllingFunction == E_LCF_TJA_nu);
    } else {
        *bTrigLargeDeviation = FALSE;
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
