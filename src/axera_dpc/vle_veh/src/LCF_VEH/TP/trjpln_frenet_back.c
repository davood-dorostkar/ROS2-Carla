/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trjpln_frenet_back.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static float32 TRJPLN_Sf_CalcTgtCrvGrdPredTime_sec =
    0.f;  // used in function TRJPLN_FBT_CalcTargetCurveGrad
static float32 TRJPLN_Sf_CalcTgtCrvGrdLowpassFilter_1pm =
    0.f;  // used in function TRJPLN_FBT_CalcTargetCurveGrad
static boolean TRJPLN_Sb_TargetValueSelectSRTrigger =
    FALSE;  // used in function TRJPLN_FBT_TargetValueSelector
static boolean TRJPLN_Sb_TargetValueSelectEdgeRise =
    FALSE;  // used in function TRJPLN_FBT_TargetValueSelector
static float32 TRJPLN_Sf_TargetValueSelectTimmerRetrig_sec =
    0.f;  // used in function TRJPLN_FBT_TargetValueSelector
static float32 TRJPLN_Sf_TargetValueSelectCrvGradLimit_1pm =
    0.f;  // used in function TRJPLN_FBT_TargetValueSelector
static float32 TRJPLN_Sf_TargetValueSelectDistYGradLimit_met =
    0.f;  // used in function TRJPLN_FBT_TargetValueSelector
static float32 TRJPLN_Sf_TargetValueSelectHeadIncGradLimit_rad =
    0.f;  // used in function TRJPLN_FBT_TargetValueSelector
static float32 TRJPLN_Sf_TargetValueSelectHeadGradLimit_rad =
    0.f;  // used in function TRJPLN_FBT_TargetValueSelector
static uint16 TRJPLN_Sc_CheckTrajPlanQuStatus =
    0u;  // used in function TPJPLN_FBT_CheckTrajPlanStatus
static boolean TRJPLN_Sb_CheckTrajPlanEdgeRising =
    FALSE;  // used in function TPJPLN_FBT_CheckTrajPlanStatus
static float32 TRJPLN_Sf_CheckTrajPlanTimmerMax =
    0.f;  // used in function TPJPLN_FBT_CheckTrajPlanStatus
static float32 TRJPLN_Sf_CheckTrajPlanTimmerMin =
    0.f;  // used in function TPJPLN_FBT_CheckTrajPlanStatus
static float32 TRJPLN_Sf_TimerInLaneChange_sec =
    0.f;  // used in function TRJPLN_FBT_TgtDistYPlausiCheck
static float32 TRJPLN_Sb_LastinLaneChangeDly =
    FALSE;  // used in function TRJPLN_FBT_TgtDistYPlausiCheck
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:LCF_TRJPLN_FrenetBack_Exec */ /*!

@brief: frenet back transformation module main process

@description: This module executes the transformation from coordinates that are
relative to the road back to world coordinates

@param[in]
const TRJPLN_FrenetBackInReq_t* reqPorts: input data of frenet
back transformation module
const TRJPLN_TrajectoryPlanParam_t* paras: system parameters
@param[out]
TRJPLN_FrenetBackOutPro_t* proPorts: output data of frenet back
transformation module
@return
@uml
@startuml
start
:TRJPLN_calFrenetBackTransformation;
note:frenet back transformation, from frenet coordinate\n system to cartesian
coordinate system
:TRJPLN_FBT_CalcTargetCurveGrad;
note:calculate target curvature and curvature \ngradient at end of the planned
trajectory
if (trajectory is enabled) then (yes)
:TRJPLN_FBT_TargetValueSelector;
note:select output path parameters, and \ncalculate the difference between
target \ncorridor path and planned trajectory path
endif
:TRJPLN_FBT_TgtDistYPlausiCheck;
note:plausibility check of DistY, the trajectory\n distY should between left
corridor and right corridor
:TPJPLN_FBT_CheckTrajPlanStatus;
note:trajectory planner status check, value \nassignment of bitfiled
:TPJPLN_FBT_SetupTrajGuiFromQualifier;
note:set trajectory guidance qualification \nvalue based on lateral control mode
and\n trajectory plan enabled
end
@enduml
*****************************************************************************/
void LCF_TRJPLN_FrenetBack_Exec(const TRJPLN_FrenetBackInReq_t* reqPorts,
                                const TRJPLN_TrajectoryPlanParam_t* paras,
                                TRJPLN_FrenetBackOutPro_t* proPorts,
                                TRJPLN_FrenetBackDebug_t* debug) {
    float32 fTargetCorridorCrv_1pm = 0.f;
    float32 fTgtCrvInclPreviewAndDeadTime_1pm = 0.f;
    boolean bTrajPlanInvalid = FALSE;

    // step 1, frenet back transformation, from frenet coordinate system to
    // cartesian coordinate system
    TRJPLN_calFBTInType_t sFrenetBackTransInput = {
        reqPorts->uiQuStatusTrajPlan_nu != 0u,  // uiNoTrajFound_nu;
        reqPorts->bTrajPlanEnble,               // uiTPLFBT_TrajGuiEnable_nu;
        reqPorts->fRiCorridorCurve_1pm,     // fTPLFBT_CridrRightSeg1_Crv_1pm;
        reqPorts->fRiCorridorCrvChng_1pm2,  //
        reqPorts->fTrajDistY_met,           // fTPLFBT_TrajDistY_met;
        reqPorts->fTrajDistY1stDeriv_mps,   // fTPLFBT_TrajDistY1stDeriv_mps;
        reqPorts->fTrajDistY2ndDeriv_mps2,  // fTPLFBT_TrajDistY2ndDeriv_nu;
        reqPorts->fTrajDistY3rdDeriv_mps3,  // fTPLFBT_TrajDistY3rdDeriv_nu;
        reqPorts->fTrajVelRefCurve_mps,     // fTPLFBT_TrajVelRefCurve_mps;
        reqPorts->fTrajAclRefCurve_mps2,    // fTPLFBT_TrajAclRefCurve_mps2;
        reqPorts->bReplanModeArcLength,     // uiReplanModeArcLength_nu;
        reqPorts->fCurDistYPreview_met,     // fDistYCurValuePreview_met;
        reqPorts->fCurDistY1stToPrev_mps,  // fDistY1stDerivCurValuePreview_mps;
        reqPorts->fDevHeadingAngle_rad,    // fDevHeading_rad;
        reqPorts->fDevDistY_met,           // fDevDistY_met;
        reqPorts->fYDtTrjFmHeadPrev_mps,   // fYDotTrajFromHeadingPreview_mps;
        reqPorts
            ->fYDt2TrjFmKpPrevDT_mps2,  // fYDot2TrajFromKappaPrevAndDT_mps2;
        reqPorts->fYDt3TrjFmKpPrevDT_mps3,  // fYDot3TrajFromKappaPrevAndDT_nu;
        reqPorts->fYD2TrjFmKpPrev_mps2  // fYDotDotTrajFromKappaPreview_mps2;
    };
    TRJPLN_calFBTOutType_t sFrenetBackResult =
        TRJPLN_calFrenetBackTransformation(&sFrenetBackTransInput);
    proPorts->fTrajDistYPrev_met = sFrenetBackResult.fTrajDistY_met;
    proPorts->fTrajHeadingPrev_rad = sFrenetBackResult.fTrajHeading_rad;
    proPorts->fTrajTgtCrvPrev_1pm = sFrenetBackResult.fTrajTgtCrv_1pm;
    proPorts->fTrajTgtCrvGrdPrev_1pms = sFrenetBackResult.fTrajTgtCrvGrd_1pms;
    proPorts->fCurHeading_rad = sFrenetBackResult.fHeadingCurValuePreview_rad;
    // proPorts->fCurHeading_rad = reqPorts->fDevHeadingAngle_rad;
    proPorts->fCurDistY_met = reqPorts->fDevDistY_met;

    // step 2, calculate target curvature and curvature gradient at end of the
    // planned trajectory
    TRJPLN_CalcTargetCurveGradIn_st sCalcTargetCurveInput = {
        &reqPorts->bReplanCurValues,             //* pbReplanCurValues;
        &reqPorts->fCycleTimeVeh_sec,            //* pfSysCycleTimeVeh_sec;
        &reqPorts->fTrajVelRefCurve_mps,         //* pfTrajVelRefCurve_mps;
        &reqPorts->fTargetCorridorCurve_1pm,     //* pfTargetCorridorCrv_1pm;
        &reqPorts->fTargetCorridorCrvChng_1pm2,  //* pfTgtCridrChngOfCrv_1pm2;
        &reqPorts->fTargetCorridorLength_met,    //* pfTargetCridrLength_met;
        &reqPorts->fTargetCorridorPosX0_met,     //* pfTargetCorridorPosX0_met;
        &reqPorts->fPredictionTimeCrv_sec,       //* fPredictionTimeCrve_sec;
        &reqPorts->fDelayVehGui_sec,             //* pfDelayVehGui_sec;
        &reqPorts->bTrigTrajReplan,              //* pbTrigTrajReplan;
        &sFrenetBackResult.fTgtCrvInclPreviewAndDeadTime_1pm,  //*
        // pfTgtCrvTrajInclPrevAndDeadTime_1pm;
        &sFrenetBackResult.fTgtCrvGrdTrajInclPrevAndDT_1pms,  //*
        // pfTgtCrvGrdTrajInclPrevAndDeadTime_1pm2;
        &reqPorts->bEnblSpecPlanStrategy,  //* pbEnblSpecPlanStrategy;
        &reqPorts->uiTrajGuiQualifier_nu   //* puiTrajGuiQualifier_nu;
    };
    TRJPLN_FBT_CalcTargetCurveGrad(
        sCalcTargetCurveInput, &fTargetCorridorCrv_1pm,
        &fTgtCrvInclPreviewAndDeadTime_1pm, &proPorts->fTrajTgtCrvGrd_1pms);

    // step 3, select output path parameters, and calculate the difference
    // between target corridor path and planned trajectory path
    TRJPLN_TargeValueSelectionIn_st sTargetValueSelectionIn = {
        &reqPorts->fCycleTimeVeh_sec,         // pfSysCycleTimeVeh_sec;
        &sFrenetBackResult.fTrajDistY_met,    // pfTrajDistYPrev_met;
        &sFrenetBackResult.fTrajHeading_rad,  // pfTrajHeading_rad;
        &sFrenetBackResult
             .fTrajHeadingInclPreview_rad,  // pfTrajHeadingInclPreview_rad;
        &reqPorts->bReplanCurValues,        // pbReplanCurValues;
        &fTargetCorridorCrv_1pm,            // pfTargetCorridorCrv_1pm;
        &reqPorts->uiLatCtrlMode_nu,        // puiLatCtrlMode_nu;
        &reqPorts->fTargetPathY0_met,       // pfTargetPathY0_met;
        &reqPorts->fTargetPathHeading_rad,  // pfTargetPathHeading_rad;
        &reqPorts->fEgoVelX_mps,            // pfVehVelX_mps;
        &fTgtCrvInclPreviewAndDeadTime_1pm  // pfTgtCrvInclPrevAndDeadTime_1pm;
    };

    TRJPLN_TargeValueSelectionOut_st sTargetValueSelectionOut = {
        &proPorts->fTrajDistY_met,           // pfTrajDistY_met;
        &proPorts->fTrajHeadInclPrev_rad,    // pfTrajHeadInclPrev_rad;
        &proPorts->fTrajTgtCurve_1pm,        // pfTrajTgtCrv_1pm;
        &proPorts->fDeltaTargetCrv_1pm,      // pfDeltaTargetCrv_1pm;
        &proPorts->fDeltaTargetPosY0_met,    // pfDeltaTargetPosY0_met;
        &proPorts->fDeltaTargetHeading_rad,  // pfDeltaTargetHeading_rad;
        &proPorts->bUseTargetCorridor,       // pbUseTargetCorridor;
        &proPorts->bTargetSwitch,            // pbTargetSwitch;
        &proPorts->bGradLimitActive,         // pbGradLimitActive;
        &proPorts->fTrajHeading_rad          // pfTrajHeading_rad;
    };
    if (reqPorts->bTrajPlanEnble || TPLFBT_TRAJPLANENBL_NU) {
        TRJPLN_FBT_TargetValueSelector(sTargetValueSelectionIn,
                                       sTargetValueSelectionOut);
    } else {
        // reset output if trajectory plan is not enabled
        *sTargetValueSelectionOut.pbGradLimitActive = FALSE;
        *sTargetValueSelectionOut.pbTargetSwitch = FALSE;
        *sTargetValueSelectionOut.pbUseTargetCorridor = FALSE;
        *sTargetValueSelectionOut.pfDeltaTargetCrv_1pm = 0.f;
        *sTargetValueSelectionOut.pfDeltaTargetHeading_rad = 0.f;
        *sTargetValueSelectionOut.pfDeltaTargetPosY0_met = 0.f;
        *sTargetValueSelectionOut.pfTrajDistY_met = 0.f;
        *sTargetValueSelectionOut.pfTrajHeadInclPrev_rad = 0.f;
        *sTargetValueSelectionOut.pfTrajHeading_rad = 0.f;
        *sTargetValueSelectionOut.pfTrajTgtCrv_1pm = 0.f;
    }
    proPorts->fCtrlErrDistY_met =
        proPorts->fTrajDistY_met - reqPorts->fDevDistY_met;
    proPorts->fCtrlErrHeadAglPrev_rad =
        proPorts->fTrajHeadInclPrev_rad - proPorts->fCurHeading_rad;
    proPorts->fCtrlErrHeadingAngle_rad =
        proPorts->fTrajHeading_rad - reqPorts->fDevHeadingAngle_rad;

    // step 4, plausibility check of DistY, the trajectory distY should between
    // left corridor and right corridor
    TRJPLN_FBT_TgtDistYPlausiCheck(
        reqPorts->fDevDistY_met, reqPorts->fRiCorridorPosY0_met,
        reqPorts->fLeCorridorPosY0_met, proPorts->fTrajDistY_met,
        reqPorts->uiTrajGuiQualifier_nu, &proPorts->bPlausiCheckStatus);

    // step 5, trajectory planner status check, value assignment of bitfiled
    TPJPLN_FBT_CheckTrajPlanStatus(
        reqPorts->uiSysStateLCF_nu, reqPorts->uiSPlausiCheckStatus_nu,
        reqPorts->uiQuStatusTrajPlan_nu, reqPorts->fCycleTimeVeh_sec,
        proPorts->bPlausiCheckStatus, &proPorts->uiD_QuStatusTrajPlan_nu,
        &proPorts->uiS_QuStatusTrajPlan_nu, &bTrajPlanInvalid);

    // step 6, set trajectory guidance qualification value based on lateral
    // control mode and trajectory plan enabled
    TPJPLN_FBT_SetupTrajGuiFromQualifier(
        reqPorts->uiSysStateLCF_nu, bTrajPlanInvalid, reqPorts->bTrajPlanEnble,
        reqPorts->uiTrajGuiQualifier_nu, &proPorts->uiTrajGuiQualifier_nu);

    debug->fTrajHeading_rad = proPorts->fTrajHeading_rad;
    debug->fTrajDistY_met = proPorts->fTrajDistY_met;
    debug->fTrajTgtCurve_1pm = proPorts->fTrajTgtCurve_1pm;
    debug->fTrajTgtCrvGrd_1pms = proPorts->fTrajTgtCrvGrd_1pms;
    debug->uiTrajGuiQualifier_nu = proPorts->uiTrajGuiQualifier_nu;
    debug->fTrajHeadingPrev_rad = proPorts->fTrajHeadingPrev_rad;
    debug->fTrajTgtCrvPrev_1pm = proPorts->fTrajTgtCrvPrev_1pm;
    debug->fCurHeading_rad = proPorts->fCurHeading_rad;
    debug->fCurDistY_met = proPorts->fCurDistY_met;
    debug->fTrajHeadInclPrev_rad = proPorts->fTrajHeadInclPrev_rad;
    debug->fCtrlErrDistY_met = proPorts->fCtrlErrDistY_met;
    debug->fCtrlErrHeadingAngle_rad = proPorts->fCtrlErrHeadingAngle_rad;
    debug->fCtrlErrHeadAglPrev_rad = proPorts->fCtrlErrHeadAglPrev_rad;
    debug->fTrajDistYPrev_met = proPorts->fTrajDistYPrev_met;
    debug->fDeltaTargetCrv_1pm = proPorts->fDeltaTargetCrv_1pm;
    debug->fDeltaTargetPosY0_met = proPorts->fDeltaTargetPosY0_met;
    debug->fDeltaTargetHeading_rad = proPorts->fDeltaTargetHeading_rad;
    debug->bUseTargetCorridor = proPorts->bUseTargetCorridor;
    debug->bTargetSwitch = proPorts->bTargetSwitch;
    debug->bGradLimitActive = proPorts->bGradLimitActive;
    debug->bPlausiCheckStatus = proPorts->bPlausiCheckStatus;
    debug->uiS_QuStatusTrajPlan_nu = proPorts->uiS_QuStatusTrajPlan_nu;
    debug->fTrajTgtCrvGrdPrev_1pms = proPorts->fTrajTgtCrvGrdPrev_1pms;
    debug->uiD_QuStatusTrajPlan_nu = proPorts->uiD_QuStatusTrajPlan_nu;
}

/*****************************************************************************
  Functionname: TPJPLN_FBT_SetupTrajGuiFromQualifier */ /*!

@brief: output trajectory guidance qualifier result while plan enabled

@description: output trajectory guidance qualifier result while plan enabled.
the intput uiTrajGuiQualifier_nu would be output directly if control mode
valid

@param[in]
uint8 uiSysStateLCF_nu:  LCF control mode from SEN function
boolean bTrajPlanInvalid: trajectory plan invalid check result from last
function
boolean bTrajPlanEnable: trajectory plan enabled flag
uint8 uiTrajGuiQualifier_nu: qualifier value of trajectory guidence from
lateral control function
@param[out]
uint8* puiTrajGuiQualifier_nu: trajectory plan module's trajectory guidance
qualifier result
@return
*****************************************************************************/
void TPJPLN_FBT_SetupTrajGuiFromQualifier(uint8 uiSysStateLCF_nu,
                                          boolean bTrajPlanInvalid,
                                          boolean bTrajPlanEnable,
                                          uint8 uiTrajGuiQualifier_nu,
                                          uint8* puiTrajGuiQualifier_nu) {
    boolean bTrajPlanInvalidCheck = bTrajPlanInvalid || !bTrajPlanEnable;
    boolean bTrajGuiQualSwitch =
        bTrajPlanInvalidCheck &&
        (uiSysStateLCF_nu == E_LCF_SYSSTATE_CONTROLLING) &&
        TPLFBT_MANIPTRAJGUIQU_NU;
    if (bTrajGuiQualSwitch) {
        *puiTrajGuiQualifier_nu = E_LCF_TGQ_REQ_FREEZE;
    } else {
        boolean bTempCheck = bTrajPlanInvalidCheck &&
                             (uiSysStateLCF_nu == E_LCF_SYSSTATE_REQUEST) &&
                             TPLFBT_MANIPTRAJGUIQU_NU;
        *puiTrajGuiQualifier_nu =
            bTempCheck ? E_LCF_TGQ_REQ_OFF : uiTrajGuiQualifier_nu;
    }
}

/*****************************************************************************
  Functionname: TPJPLN_FBT_CheckTrajPlanStatus */ /*!

@brief: qualification bit field value assignment and trajectory plan invalid
bool result check

@description: qualification bit field value assignment and trajectory plan
invalid bool result check

@param[in]
uint8 uiSysStateLCF_nu: LCF control mode from SEN function
uint8 uiPlausiCheckStatus_nu: latency compensation module
plausibility check result
uint16 uiQuStatusTrajPlan_nu: trajectory plan quality status
float32 fSysCycleTimeVeh_sec: cycle time
boolean bPlausiCheckStatus: plausibility check result of
TRJPLN_FBT_TgtDistYPlausiCheck function

@param[out]
uint16* uiDQuStatusTrajPlan: trajectory plan qualification result bit
field
uint16* uiSQuStatusTrajPlan: trajectory plan qualification result bit
field
boolean* bTrajPlanInvalid: trajectory plan invalid check bool
@return
*****************************************************************************/
void TPJPLN_FBT_CheckTrajPlanStatus(uint8 uiSysStateLCF_nu,
                                    uint8 uiPlausiCheckStatus_nu,
                                    uint16 uiQuStatusTrajPlan_nu,
                                    float32 fSysCycleTimeVeh_sec,
                                    boolean bPlausiCheckStatus,
                                    uint16* uiDQuStatusTrajPlan,
                                    uint16* uiSQuStatusTrajPlan,
                                    boolean* bTrajPlanInvalid) {
    /*Notes for S_TPLTJC_QuStatusTrajPlan_nu:
    Bitfield indicates the status of trajectory planner : 1 Not OK, 0 OK
            0 bit : min acceleration check
            1 bit : max acceleration check
            2 bit : right corridor boundary collision check
            3 bit : left corridor boundary collision check
            4 bit : object collision check
            5 bit : matrix invertible
            6 bit : trajectory length
            7 bit : max jerk check
            8 bit : lane cross check
            9 bit : target lateral distance
            10 bit : vehicle lateral distance
            11 bit : right corridor transformation
            12 bit : target corridor transformation
            13 bit : left corridor transformation
            14 bit : input target corridor heading*/
    // TRJPLN_GetBit
    // TPLTJC_SetBit
    boolean bSysStateLCFEnable =
        (uiSysStateLCF_nu == E_LCF_SYSSTATE_REQUEST) ||
        (uiSysStateLCF_nu == E_LCF_SYSSTATE_CONTROLLING);

    if (bSysStateLCFEnable) {
        uint16 uiResultBit = 0u;
        if (TPLFBT_USELONGQUSTATUS_NU) {
            TPLTJC_SetBit(9u, bPlausiCheckStatus, &uiResultBit);
            TPLTJC_SetBit(10u, TRJPLN_GetBit(0u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
            TPLTJC_SetBit(11u, TRJPLN_GetBit(1u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
            TPLTJC_SetBit(12u, TRJPLN_GetBit(2u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
            TPLTJC_SetBit(13u, TRJPLN_GetBit(3u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
            TPLTJC_SetBit(14u, TRJPLN_GetBit(4u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
            TPLTJC_SetBit(15u, TRJPLN_GetBit(5u, uiPlausiCheckStatus_nu),
                          &uiResultBit);
        } else {
            uiResultBit = uiPlausiCheckStatus_nu;
            TPLTJC_SetBit(6u, bPlausiCheckStatus, &uiResultBit);
            TPLTJC_SetBit(7u, uiQuStatusTrajPlan_nu != 0u, &uiResultBit);
        }
        *uiDQuStatusTrajPlan = uiResultBit;
    } else {
        *uiDQuStatusTrajPlan = 0u;
    }

    // TRJPLN_Sc_CheckTrajPlanQuStatus
    boolean bEdgeRisingInput = (!TPLFBT_HOLDALLBITS_NU)
                                   ? (TRJPLN_Sc_CheckTrajPlanQuStatus != 0u)
                                   : (TRJPLN_Sc_CheckTrajPlanQuStatus);
    boolean bEdgeRisingCheck = TUE_CML_RisingEdgeSwitch(
        bEdgeRisingInput, &TRJPLN_Sb_CheckTrajPlanEdgeRising);

    // TRJPLN_Sf_CheckTrajPlanTimmerMin
    boolean bMaxTimmerTrigger = TUE_CML_TimerRetrigger(
        fSysCycleTimeVeh_sec, bEdgeRisingCheck, TPLFBT_MAXQUSTATUSHOLD_SEC,
        &TRJPLN_Sf_CheckTrajPlanTimmerMax);
    boolean bMinTimmerTrigger = TUE_CML_TimerRetrigger(
        fSysCycleTimeVeh_sec, bEdgeRisingCheck, TPLFBT_MINQUSTATUSHOLD_SEC,
        &TRJPLN_Sf_CheckTrajPlanTimmerMin);

    if ((TPLFBT_QUSTATUSHOLD_NU) && (TRJPLN_Sc_CheckTrajPlanQuStatus != 0u) &&
        bMaxTimmerTrigger && (bMinTimmerTrigger || bSysStateLCFEnable)) {
        if (TPLFBT_HOLDALLBITS_NU) {
            TRJPLN_Sc_CheckTrajPlanQuStatus =
                (*uiDQuStatusTrajPlan) | (TRJPLN_Sc_CheckTrajPlanQuStatus);
        }
    } else {
        TRJPLN_Sc_CheckTrajPlanQuStatus = *uiDQuStatusTrajPlan;
    }
    *bTrajPlanInvalid =
        (TRJPLN_Sc_CheckTrajPlanQuStatus & TPLFBT_CHECKTRAJPLANSTATUS_NU) != 0u;
    *uiSQuStatusTrajPlan = TPLFBT_QUSTATUSTRAJPLAN_NU
                               ? TPLFBT_QUSTATUSTRAJPLAN_NU
                               : TRJPLN_Sc_CheckTrajPlanQuStatus;
}

/*****************************************************************************
  Functionname: TRJPLN_FBT_TgtDistYPlausiCheck */ /*!

@brief: check the trajectory distance Y plausibility

@description: threshold check of trajectory distance Y

@param[in]
float32 fDevDistY_met: ego distance Y from right corridor at current
time
float32 fRiCorridorPosY0_met: right corridor PosY0
float32 fLeCorridorPosY0_met: left corridor PosY0
float32 fTrajDistY_met: planned trajectory DistY
@param[out]
boolean* pbPlausiCheckStatus: plausibility check result bool
@return
*****************************************************************************/
void TRJPLN_FBT_TgtDistYPlausiCheck(float32 fDevDistY_met,
                                    float32 fRiCorridorPosY0_met,
                                    float32 fLeCorridorPosY0_met,
                                    float32 fTrajDistY_met,
                                    uint8 uiTrajGuiQualifier_nu,
                                    boolean* pbPlausiCheckStatus) {
    boolean bResult =
        (MAX(fLeCorridorPosY0_met, fDevDistY_met) >= fTrajDistY_met) &&
        (MIN(fRiCorridorPosY0_met, fDevDistY_met) <= fTrajDistY_met);
    boolean binLaneChange;
    if(uiTrajGuiQualifier_nu == E_LCF_TGQ_REQ_REFCHNG){
        binLaneChange = TRUE;
    }else{
        binLaneChange = FALSE;
    }
    boolean binLaneChangeDly;
    binLaneChangeDly = TUE_CML_TurnOffDelay_M(
        binLaneChange, 4.5F, 0.02F, &TRJPLN_Sf_TimerInLaneChange_sec,
        TRJPLN_Sb_LastinLaneChangeDly);
    
    if (binLaneChangeDly) {
        bResult = (MAX(fLeCorridorPosY0_met, fDevDistY_met) +
                       TPLFBT_LANCHGPLAUSICHECK_MET >=
                   fTrajDistY_met) &&
                  (MIN(fRiCorridorPosY0_met, fDevDistY_met) -
                       TPLFBT_LANCHGPLAUSICHECK_MET <=
                   fTrajDistY_met);
    }

    *pbPlausiCheckStatus = !bResult;
    TRJPLN_Sb_LastinLaneChangeDly = binLaneChangeDly;
}

/*****************************************************************************
  Functionname:TRJPLN_FBT_TargetValueSelector */ /*!

@brief: select and output final target trajectory parameter

@description: calculate the difference between current ego position and target
trajectory position .
we will choose the original target corridor data to output while
exception happened, but
most of time, the planned trajectory would be output directly

@param[in]
TRJPLN_TargeValueSelectionIn_st sInput: input of function
@param[out]
TRJPLN_TargeValueSelectionOut_st sOutput: output of function
@return
*****************************************************************************/
void TRJPLN_FBT_TargetValueSelector(TRJPLN_TargeValueSelectionIn_st sInput,
                                    TRJPLN_TargeValueSelectionOut_st sOutput) {
    *sOutput.pfDeltaTargetCrv_1pm = *sInput.pfTgtCrvInclPrevAndDeadTime_1pm -
                                    (*sInput.pfTargetCorridorCrv_1pm);
    *sOutput.pfDeltaTargetPosY0_met =
        *sInput.pfTrajDistYPrev_met - (*sInput.pfTargetPathY0_met);
    *sOutput.pfDeltaTargetHeading_rad =
        *sInput.pfTrajHeadingInclPreview_rad -
        (TPLFBT_USETGTCRIDRHEADING_NU ? (*sInput.pfTargetPathHeading_rad)
                                      : 0.f);

    // check whether we need to change output from planned trajectory to target
    // corridor
    boolean bTempLatCtrlModeCheck =
        TPLFBT_ENABLEOBFMODE_NU
            ? (!(*sInput.puiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_OF ||
                 *sInput.puiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_OF_RQ))
            : (TRUE);
    if (bTempLatCtrlModeCheck &&
        !(!TPLFBT_ENABLEDIRECTSWITCH_NU && *sInput.pbReplanCurValues) &&
        fABS(*sOutput.pfDeltaTargetCrv_1pm) <= TPLFBT_DELTATARGETCRV_1PM &&
        fABS(*sOutput.pfDeltaTargetPosY0_met) <= TPLFBT_DELTATARGETPOSY0_MET &&
        fABS(*sOutput.pfDeltaTargetHeading_rad) <=
            TPLFBT_DELTATARGETHEADING_RAD) {
        *sOutput.pbTargetSwitch = TRUE;
    } else {
        *sOutput.pbTargetSwitch = FALSE;
    }
    TRJPLN_Sb_TargetValueSelectSRTrigger =
        TUE_CML_SRTrigger_M(*sOutput.pbTargetSwitch, *sInput.pbReplanCurValues,
                            TRJPLN_Sb_TargetValueSelectSRTrigger);
    *sOutput.pbUseTargetCorridor = TRJPLN_Sb_TargetValueSelectSRTrigger;

    boolean bTempEdgeRiseResult = TUE_CML_RisingEdgeSwitch(
        *sOutput.pbUseTargetCorridor, &TRJPLN_Sb_TargetValueSelectEdgeRise);
    *sOutput.pbGradLimitActive = TUE_CML_TimerRetrigger(
        *sInput.pfSysCycleTimeVeh_sec, bTempEdgeRiseResult,
        TPLFBT_DURATIONGRADLIMIT_SEC,
        &TRJPLN_Sf_TargetValueSelectTimmerRetrig_sec);

    if (!(*sOutput.pbUseTargetCorridor)) {
        // if we did not need to use target corridor data, the planned
        // trajectory would be output directly
        *sOutput.pfTrajTgtCrv_1pm = (*sInput.pfTgtCrvInclPrevAndDeadTime_1pm);
        *sOutput.pfTrajDistY_met = (*sInput.pfTrajDistYPrev_met);
        *sOutput.pfTrajHeadInclPrev_rad =
            (*sInput.pfTrajHeadingInclPreview_rad);
        *sOutput.pfTrajHeading_rad = (*sInput.pfTrajHeading_rad);
    } else {
        // if we need to use target corridor data because some exception found,
        // we will output filtered data of target corridor
        float32 afTableInputX[TPLFBT_GRADLIMITCRVFACTOR_TABLENUM_NU] =
            TPLFBT_VEHVELX_MPS_TABLEX_MPS;
        float32 afTableInputY[TPLFBT_GRADLIMITCRVFACTOR_TABLENUM_NU] =
            TPLFBT_GRADLIMITCRVFACTOR_TABLEY_NU;
        float32 fGradLimitCrvFactor_nu = TUE_CML_LookUpTable2D(
            *sInput.pfVehVelX_mps, afTableInputX, afTableInputY,
            TPLFBT_GRADLIMITCRVFACTOR_TABLENUM_NU);
        float32 fLimit = fGradLimitCrvFactor_nu * (*sInput.pfVehVelX_mps) *
                         TPLFBT_GRADLIMITCRV_1PM2;
        *sOutput.pfTrajTgtCrv_1pm =
            TUE_CML_GradLimit_M(*sInput.pfTargetCorridorCrv_1pm, fLimit,
                                -fLimit, *sInput.pfSysCycleTimeVeh_sec,
                                TRJPLN_Sf_TargetValueSelectCrvGradLimit_1pm);
        // TUE_CML_GradLimit_M,

        fLimit = (*sOutput.pbGradLimitActive) ? TPLFBT_GRADLIMITPOSY_MPS
                                              : TPLFBT_GRADLIMITPOSYHIGH_MPS;
        *sOutput.pfTrajDistY_met =
            TUE_CML_GradLimit_M(*sInput.pfTargetPathY0_met, fLimit, -fLimit,
                                *sInput.pfSysCycleTimeVeh_sec,
                                TRJPLN_Sf_TargetValueSelectDistYGradLimit_met);

        fLimit = (*sOutput.pbGradLimitActive) ? TPLFBT_GRADLIMITHEAD_RPS
                                              : TPLFBT_GRADLIMITHEADHIGH_RPS;
        *sOutput.pfTrajHeadInclPrev_rad = TUE_CML_GradLimit_M(
            (TPLFBT_USETGTCRIDRHEADING_NU ? (*sInput.pfTargetPathHeading_rad)
                                          : 0.f),
            fLimit, -fLimit, *sInput.pfSysCycleTimeVeh_sec,
            TRJPLN_Sf_TargetValueSelectHeadIncGradLimit_rad);

        fLimit = (*sOutput.pbGradLimitActive) ? TPLFBT_GRADLIMITHEAD_RPS
                                              : TPLFBT_GRADLIMITHEADHIGH_RPS;
        *sOutput.pfTrajHeading_rad = TUE_CML_GradLimit_M(
            (TPLFBT_USETGTCRIDRHEADING_NU ? (*sInput.pfTargetPathHeading_rad)
                                          : 0.f),
            fLimit, -fLimit, *sInput.pfSysCycleTimeVeh_sec,
            TRJPLN_Sf_TargetValueSelectHeadGradLimit_rad);
    }
    TRJPLN_Sf_TargetValueSelectCrvGradLimit_1pm = *sOutput.pfTrajTgtCrv_1pm;
    TRJPLN_Sf_TargetValueSelectDistYGradLimit_met = *sOutput.pfTrajDistY_met;
    TRJPLN_Sf_TargetValueSelectHeadIncGradLimit_rad =
        *sOutput.pfTrajHeadInclPrev_rad;
    TRJPLN_Sf_TargetValueSelectHeadGradLimit_rad = *sOutput.pfTrajHeading_rad;
}

/*****************************************************************************
  Functionname:TRJPLN_FBT_CalcTargetCurveGrad */ /*!

@brief: calculate target curvature and curvature gradient at end of the planned
trajectory

@description: calculate target curvature and curvature gradient at end of the
planned trajectory,
consider preview time and vehicle delay time (DeadTime)

@param[in]
const TRJPLN_CalcTargetCurveGradIn_st sInput: frenet back transformed
ego and target lateral parameters
@param[out]
float32* fMeasKappaTgtInclPrevAndDeadTime_1pm: target curvature (with
lowpass filter) at planned trajectory end point
float32* fTgtCrvInclPrevAndDeadTime_1pm: target curvature (with extra
vehicle delay time and preview time)
float32* fTrajTgtCrvGrd_1pm2: target curvature gradient with extra
vehicle delay time and preview time
@return
*****************************************************************************/
void TRJPLN_FBT_CalcTargetCurveGrad(
    const TRJPLN_CalcTargetCurveGradIn_st sInput,
    float32* fMeasKappaTgtInclPrevAndDeadTime_1pm,
    float32* fTgtCrvInclPrevAndDeadTime_1pm,
    float32* fTrajTgtCrvGrd_1pm2) {
    float32 fMeasCrvGradTgt_1pm2 =
        (*sInput.pfTrajVelRefCurve_mps) * (*sInput.pfTgtCridrChngOfCrv_1pm2);

    // calculate conv target kappa from clothoid model
    float32 fTgtKappa_1pm = (-(*sInput.pfTargetCorridorPosX0_met)) *
                                (*sInput.pfTgtCridrChngOfCrv_1pm2) +
                            (*sInput.pfTargetCorridorCrv_1pm);
    if ((*sInput.pbTrigTrajReplan) || TPLFBT_CRVPREDICTIONENBL_NU) {
        TRJPLN_Sf_CalcTgtCrvGrdPredTime_sec =
            (*sInput.pfPredictionTimeCrve_sec + *sInput.pfDelayVehGui_sec);
    }

    *fMeasKappaTgtInclPrevAndDeadTime_1pm =
        fTgtKappa_1pm + (MIN(*sInput.pfTargetCridrLength_met,
                             *sInput.pfTrajVelRefCurve_mps *
                                 TRJPLN_Sf_CalcTgtCrvGrdPredTime_sec) *
                         (*sInput.pfTgtCridrChngOfCrv_1pm2));
    if (TPLFBT_CRVPT1FILTERENBL_NU) {
        if (*sInput.pbReplanCurValues) {
            TRJPLN_Sf_CalcTgtCrvGrdLowpassFilter_1pm = 0.f;
        } else {
            // lowpass filter
            TUE_CML_LowPassFilter(
                &TRJPLN_Sf_CalcTgtCrvGrdLowpassFilter_1pm,
                *fMeasKappaTgtInclPrevAndDeadTime_1pm,
                (*sInput.pfSysCycleTimeVeh_sec / TPLFBT_CRVPT1TIMECONST_SEC));
            *fMeasKappaTgtInclPrevAndDeadTime_1pm =
                TRJPLN_Sf_CalcTgtCrvGrdLowpassFilter_1pm;
        }
    }

    if (TPLFBT_TGTCRVSWITCH_NU && (*sInput.pbEnblSpecPlanStrategy) &&
        (*sInput.puiTrajGuiQualifier_nu != E_LCF_TGQ_REQ_REFCHNG)) {
        *fTgtCrvInclPrevAndDeadTime_1pm = *fMeasKappaTgtInclPrevAndDeadTime_1pm;
        *fTrajTgtCrvGrd_1pm2 = fMeasCrvGradTgt_1pm2;
    } else {
        *fTgtCrvInclPrevAndDeadTime_1pm =
            *sInput.pfTgtCrvTrajInclPrevAndDeadTime_1pm;
        *fTrajTgtCrvGrd_1pm2 = *sInput.pfTgtCrvGrdTrajInclPrevAndDeadTime_1pm2;
    }
}

/*****************************************************************************
  Functionname: LCF_TRJPLN_FrenetBack_Reset */ /*!

@brief: reset function of frenet back module

@description: reset function of frenet back module

@param[in]
@param[out]

@return
*****************************************************************************/
void LCF_TRJPLN_FrenetBack_Reset(void) {
    TRJPLN_Sf_CalcTgtCrvGrdPredTime_sec = 0.f;
    TRJPLN_Sf_CalcTgtCrvGrdLowpassFilter_1pm = 0.f;
    TRJPLN_Sb_TargetValueSelectSRTrigger = FALSE;
    TRJPLN_Sb_TargetValueSelectEdgeRise = FALSE;
    TRJPLN_Sf_TargetValueSelectTimmerRetrig_sec = 0.f;
    TRJPLN_Sf_TargetValueSelectCrvGradLimit_1pm = 0.f;
    TRJPLN_Sf_TargetValueSelectDistYGradLimit_met = 0.f;
    TRJPLN_Sf_TargetValueSelectHeadIncGradLimit_rad = 0.f;
    TRJPLN_Sf_TargetValueSelectHeadGradLimit_rad = 0.f;
    TRJPLN_Sc_CheckTrajPlanQuStatus = 0u;
    TRJPLN_Sb_CheckTrajPlanEdgeRising = FALSE;
    TRJPLN_Sf_CheckTrajPlanTimmerMax = 0.f;
    TRJPLN_Sf_CheckTrajPlanTimmerMin = 0.f;
    TRJPLN_Sf_TimerInLaneChange_sec = 0.f;
    TRJPLN_Sb_LastinLaneChangeDly = FALSE;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
