/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trjpln_latency_compensation.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"
const volatile float32 TP_VEHVELX2PREVIEWDISTXOF_TABLEY_MET[13] = {
    0.f,   3.5f,  4.f,   8.f,   8.f,   10.5f, 12.0f,
    13.5f, 13.5f, 13.5f, 13.5f, 13.5f, 13.5f};
const volatile boolean LCF_TPLLCO_REPLANDEVENABLE_NU = FALSE;
const volatile boolean LCF_TPLLCO_USEODOREPLAN_NU = TRUE;
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h"

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
static TRJPLN_LatencyCompensationEgoMotion_t TRJPLN_Ss_EgoHistoryMovement_nu = {
    0};
static boolean TRJPLN_Sb_CCCModeChange_nu = FALSE;
static float32 TRJPLN_Sa_CtrlPtDev_met[2] = {0.f};
static TRJPLN_LatencyCompensationEgoMotion_t TRJPLN_Ss_AccumuEgoMotion_nu = {0};
static TRJPLN_LatencyCompensationAllCorridor_t
    TRJPLN_Ss_CorriFrozChckHisCorr_nu = {0};
static boolean TRJPLN_Sb_CorriFrozChckTrajLengthVald_b = FALSE;
static float32 TRJPLN_Sf_CorriFrozChckHistoryCCCLength_met = 0.f;
static uint8 TRJPLN_Sc_CCCResetAccumuLaneQual_nu = 0;
static boolean TRJPLN_Sb_CCCPassCheck_nu = FALSE;
static float32 TRJPLN_Sf_CalcCurVehPosLastTimeStamp_sec = 0.f;
static boolean TRJPLN_Sb_TrajGuiQuaREQEdgeRise_nu = FALSE;
static boolean TRJPLN_Sb_TrajGuiQuaTGQEdgeRise_nu = FALSE;
static boolean TRJPLN_Sb_LastCycleTriggerReplan_nu = FALSE;
static float32 TRJPLN_Sa_CalcCurVehDeltaDist_met[TPLLCO_ODOHISTLENGTH_NU] = {
    0.f};
static float32 TRJPLN_Sa_CalcCurVehDelOri_rad[TPLLCO_ODOHISTLENGTH_NU] = {0.f};
static uint8 TRJPLN_Sc_CalcCurVeArrayId_nu = 0u;
static float32 TRJPLN_Sf_SetDevReplanHeadingInit_rad = 0.f;
static TRJPLN_TrajectoryParams_t TRJPLN_Ss_LastCycleRightCorr_nu = {0.f};
static float32 TRJPLN_Sf_SetDevDistYInit_met = 0.f;
static boolean TRJPLN_Sb_SetDevLastTrigReplan_nu = FALSE;
static float32 TRJPLN_Sf_LastCylceHeadingVeh_rad = 0.f;
static float32 TRJPLN_Sf_LastCyclePosDevX_met = 0.f;
static float32 TRJPLN_Sf_LastCyclePosDevY_met = 0.f;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname: LCF_TRJPLN_LatencyCompensation_Exec */ /*!

@brief: lentency compensation module

@description: we did lantency compensation and coordinate system transformation
in this module

@param[in]
const TRJPLN_LatencyCompensationInReq_t* reqPorts: lantency compensation
input
const TRJPLN_TrajectoryPlanParam_t* paras: lantency compensation static
parameter
@param[out]
TRJPLN_LatencyCompensationOutPro_t* proPorts: latency compensation output
@return
@uml
@startuml
start
:CourseConsistenceCheck;
note:target corridor consistency check
:LatencyCompensation;
note:calculate ego and corridor's \nlatency compensate result
:CorridorTrjecotryCalc;
note:calculate target corridor value \nwith latency compansated result
if (CCCWarning != TRUE && replan is triggered) then (yes)
:PlausibilityCheck;
note:we did threshold and deviation check \nfor the latency compensated data
else (no)
:set plausibility check result to defalu value 0;
endif
end
@enduml
*****************************************************************************/
void LCF_TRJPLN_LatencyCompensation_Exec(
    const TRJPLN_LatencyCompensationInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_LatencyCompensationOutPro_t* proPorts,
    TRJPLN_LatenCompDebug_t* debug) {
    TRJPLN_LatencyCompensationAllCorridor_t sAllCorridorWithCCC = {0};

    CourseConsistenceCheck(&reqPorts->sConsisCheckIn, &reqPorts->sAllCorridor,
                           &reqPorts->sCommonInput, &proPorts->sConsisCheckOut,
                           &TRJPLN_Ss_EgoHistoryMovement_nu,
                           &sAllCorridorWithCCC);

    LatencyCompensation(&reqPorts->sLatCompIn, &reqPorts->sCommonInput,
                        &TRJPLN_Ss_EgoHistoryMovement_nu, &sAllCorridorWithCCC,
                        &reqPorts->sConsisCheckIn, &proPorts->sCompenCorriParam,
                        &proPorts->sCompCorridor);

    CorridorTrjecotryCalc(proPorts->sCompCorridor.fTargetCorridorPosX0_met,
                          proPorts->sCompCorridor.fTargetCorridorPosY0_met,
                          proPorts->sCompCorridor.fTargetCorridorHeading_rad,
                          proPorts->sCompCorridor.fTargetCorridorCurve_1pm,
                          proPorts->sCompCorridor.fTargetCorridorCrvChng_1pm2,
                          &proPorts->sTargetPath);

    if (!proPorts->sConsisCheckOut.bCCCWarning &&
        proPorts->sCompCorridor.bTriggerReplan) {
        PlausibilityCheck(&proPorts->sCompCorridor,
                          &proPorts->sCompenCorriParam, &sAllCorridorWithCCC,
                          reqPorts->sCommonInput.fEgoVelX_mps,
                          &proPorts->sPlasiCheck);
    } else {
        proPorts->sPlasiCheck.uiDPlausiCheckStatus_nu =
            0u;  // reset this check result whle PlausibilityCheck function is
                 // not enable
    }

    TPLLCO_SetBit(5u, proPorts->sConsisCheckOut.bCCCInvalid,
                  &proPorts->sPlasiCheck.uiSPlausiCheckStatus_nu);
}

/*****************************************************************************
  Functionname:  CourseConsistenceCheck */ /*!

@brief: target corridor consistency check

@description: the output corridor would be replaced by history reference
corridor values if
         CCC check false

@param[in]
         pConsisCheckIn: the lane quality values
         pAllCorridor: current corridor values
         pCommonInput: cycle time, ego velocity and ego yawrate
@param[out]
         pCCCOut: course consistency check result
         pEgoMotion: ego moved distance since the reference corridor
is reset
         pAllCorridorWithCCC: the output corridor values after
consistency check, the corridor
         values would be replaced by reference values if CCC Check
false
@return void

@uml
@startuml
start
if (current control mode is TJA && CCC module is enable )
:CCC module should reset if the control mode is chenged before;
:ego moved distance calculate since last CCC reset triggered;
:sample test point chosen for the comparison check with reference ;
:sample test point position calculate with the considering of ego moved
distance ;
:check whether reference corridor should be replaced by current new corridor;
:CCC module reset check with the considering of lane quality and passed
percent of reference trajectory;
:CCC warning and invalid check based on the deviation of sample point
distance Y between reference corridor and current corridor ;
:CCC warning only output valid value while the replace switch is on;
if (CCC warning on)
 :memset ego motion values;
 :replace corridor values to CCC history reference corridor values;
endif
else
:memset all the values;
:output same corridor values same with the input current measured corridor
values;
endif
end
@enduml
*****************************************************************************/
void CourseConsistenceCheck(
    const TRJPLN_LatencyCompensationConsisCheck_t* pConsisCheckIn,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridor,
    const TRJPLN_LatencyCompensationCommon_t* pCommonInput,
    TRJPLN_LatencyCompensationCCCOut_t* pCCCOut,
    TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion,
    TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC) {
    if (pConsisCheckIn->uiControllingFunction_nu == E_LCF_TJA_nu &&
        (pConsisCheckIn->uiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_LC ||
         pConsisCheckIn->uiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_CMB ||
         pConsisCheckIn->uiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_LC_RQ ||
         pConsisCheckIn->uiLatCtrlMode_nu == E_TJASTM_LATCTRLMD_CMB_RQ) &&
        pCommonInput->uiTrajGuiQualifier_nu != E_LCF_TGQ_REQ_FREEZE &&
        P_TPLLCO_CCCENABLE_NU) {
        float32 fTestPointPosX_met = 0.f;
        float32 fTestPointPosY_met = 0.f;
        float32 fCCCTestPointPosY_met = 0.f;
        float32 fValidLength_met = 0.f;
        if (TRJPLN_Sb_CCCModeChange_nu) {
            pCCCOut->bCCCReset = TRUE;
            TRJPLN_Sb_CCCModeChange_nu = FALSE;
        }
        // ego moved distance calculate since last CCC reset triggered
        EgoMotionCalculation(
            pCommonInput->fEgoYawRate_rps, pCommonInput->fCycleTimeVeh_sec,
            pCommonInput->fEgoVelX_mps, pCCCOut->bCCCReset, pEgoMotion);

        // sample test point chosen for the comparison check with reference
        TestPointPositionCal(
            pAllCorridor->fTgtTrajLength_met, pAllCorridor->fTgtTrajPosX0_met,
            pAllCorridor->fTgtTrajPosY0_met,
            pAllCorridor->fTgtTrajHeadingAng_rad,
            pAllCorridor->fTgtTrajCurve_1pm, pAllCorridor->fTgtTrajCrvChng_1pm2,
            &fTestPointPosX_met, &fTestPointPosY_met);

        // sample test point position calculate with the considering of ego
        // moved distance
        TestPointNewPositionCal(fTestPointPosX_met, fTestPointPosY_met,
                                *pEgoMotion, &pCCCOut->fCCCTestPointDistX_met,
                                &fCCCTestPointPosY_met);

        // check whether reference corridor should be replaced by current new
        // corridor
        CorridorFrozenCheck(pCCCOut->bCCCReset, *pAllCorridor,
                            pEgoMotion->fEgoDistX_met, &pCCCOut->fCCCLength_met,
                            &fValidLength_met, pAllCorridorWithCCC);

        // CCC module reset check with the considering of lane quality and
        // passed percent of reference trajectory
        CCCResetCheck(pCCCOut->fCCCLength_met, pCCCOut->fCCCTestPointDistX_met,
                      *pConsisCheckIn, &pCCCOut->fCCCPassedPerc_nu,
                      &pCCCOut->bCCCReset);

        // CCC warning and invalid check based on the deviation of sample point
        // distance Y between reference corridor and current corridor
        CCCWarningCheck(*pAllCorridorWithCCC, pCCCOut->fCCCTestPointDistX_met,
                        fCCCTestPointPosY_met, pCCCOut->fCCCPassedPerc_nu,
                        fValidLength_met, &pCCCOut->fCCCDeviation_met,
                        &pCCCOut->bCCCWarning, &pCCCOut->bCCCInvalid);

        pCCCOut->bCCCWarning =
            pCCCOut->bCCCWarning && TPLLCO_CCCALLOWOVERWRITE_NU;
        if (!pCCCOut->bCCCWarning) {
            memset(pEgoMotion, 0,
                   sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
            memcpy(pAllCorridorWithCCC, pAllCorridor,
                   sizeof(TRJPLN_LatencyCompensationAllCorridor_t));
        }
    } else {
        TRJPLN_Sb_CCCModeChange_nu = TRUE;
        memset(pCCCOut, 0, sizeof(TRJPLN_LatencyCompensationCCCOut_t));
        memset(pEgoMotion, 0, sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
        memcpy(pAllCorridorWithCCC, pAllCorridor,
               sizeof(TRJPLN_LatencyCompensationAllCorridor_t));
    }
}

/*****************************************************************************
  Functionname:  LatencyCompensation                                          */ /*!

@brief: calculate ego and corridor's latency compensate result

@description: the time delay would be compensated by ego movement data, and all
the relate
                  data's coordinate system would be transformed to right
lane corridor coordinate
                  system, including ego position, left corridor, target
corridor and right corridor

@param[in]
                  const TRJPLN_LatencyCompensationLatComIn_t*
pLatCompIn:timestamp and cycle time input
                  const TRJPLN_LatencyCompensationCommon_t* pCommonInput:ego
movement and timestamp input
                  const TRJPLN_LatencyCompensationEgoMotion_t*
pEgoMotion:ego history record movement data since last reset
                  const TRJPLN_LatencyCompensationAllCorridor_t*
pAllCorridorWithCCC:left, target and right corridor data

@param[out]
                  TRJPLN_LatencyCompensationCorriParamOut_t*
pCompenCorriParam:the new coordinate system related data, the
                  origin point of right corridor coordinate system is The
vertical point of the orthogonal projection from
                  the car to the right lane line
                  TRJPLN_LatencyCompensationCorridor_t* pCompCorridor:time
delay compensated and coordinate system transformed result
                  of ego and three corridors
@return
@uml
@startuml
start
  :CalcCurrentVehPosition;
  note:calculate ego car distance and heading \nangle delta value between
SEN timestamp \nand VEH timestamp
  if (replan is triggered) then (yes)
          :TranslateRotateCoriCoord;
          note: calculate rectangular projection of \nthe ego vehicle to the
right corridor
  endif
  :LatencyCompensationOutput;
  note: left, right and target corridor time \ndelay compensation
end
@enduml
*****************************************************************************/
void LatencyCompensation(
    const TRJPLN_LatencyCompensationLatComIn_t* pLatCompIn,
    const TRJPLN_LatencyCompensationCommon_t* pCommonInput,
    const TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC,
    const TRJPLN_LatencyCompensationConsisCheck_t* pConsisCheckIn,
    TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    TRJPLN_LatencyCompensationCorridor_t* pCompCorridor) {
    TPLLCO_EgoDeviationSum_t sDevEgo = {0};
    TPLLCO_EgoOdoVeh_t sDevOdoVeh = {0};
    boolean bTriggerReplan = FALSE;

    // ego movement time delay calculate
    CalcCurrentVehPosition(*pLatCompIn, *pCommonInput,
                           &(pCompenCorriParam->fMeasDeltaTime_sec),
                           &sDevOdoVeh, &sDevEgo, &bTriggerReplan);

    if (bTriggerReplan) {
        // we will need a new coordinate system if replan triggered,the origin
        // point of new right corridor coordinate
        // system is The vertical point of the orthogonal projection from
        // the car to the right lane line
        TranslateRotateCoriCoord(sDevEgo, *pEgoMotion, *pAllCorridorWithCCC,
                                 pCommonInput->fEgoVelX_mps,
                                 pConsisCheckIn->uiLatCtrlMode_nu,
                                 pCompenCorriParam, TRJPLN_Sa_CtrlPtDev_met);
    }
    // calculate the compensate and coordinate transformed result
    LatencyCompensationOutput(
        sDevOdoVeh, sDevEgo.fDevHeadingEgo_rad, bTriggerReplan,
        pLatCompIn->uiSysStateLCF_nu, pConsisCheckIn->uiLatCtrlMode_nu,
        *pCompenCorriParam, TRJPLN_Sa_CtrlPtDev_met, pCommonInput->fEgoVelX_mps,
        *pAllCorridorWithCCC, pCompCorridor);
}

/*****************************************************************************
  Functionname:  CorridorTrjecotryCalc */ /*!

@brief: calculate target corridor parameter

@description: we will calculate PosY0, heading angle and curvature of target
corridor at X = 0m point

@param[in]
            const float32 fPosX0_met: target corridor PosX0
            const float32 fPosY0_met: target corridor PosY0
            const float32 fHeadingAgl_rad: target corridor HeadingAngle
            const float32 fCurve_1pm: target corridor Curvature
            const float32 fCurveChng_1pm2: target corridor curvature
change
@param[out]
            TRJPLN_LatencyCompensationTargetPath_t* pTargetPath:
calculate output data, including PosY, heading and curvature
            at position X = 0 point
@return
*****************************************************************************/
void CorridorTrjecotryCalc(
    const float32 fPosX0_met,
    const float32 fPosY0_met,
    const float32 fHeadingAgl_rad,
    const float32 fCurve_1pm,
    const float32 fCurveChng_1pm2,
    TRJPLN_LatencyCompensationTargetPath_t* pTargetPath) {
    float32 fDeltaX_met = 0.f - fPosX0_met;
    float32 fTanHead = TAN_HD_(fHeadingAgl_rad);

    // Y = Y0+ X*tan(HeadingAgl) + 0.5*X*X*Curve + 1/6 * X*X*X*CurveChange
    pTargetPath->fTargetPathY0_met =
        fPosY0_met + fDeltaX_met * fTanHead +
        0.5f * TUE_CML_Sqr(fDeltaX_met) * fCurve_1pm +
        fDeltaX_met * fDeltaX_met * fDeltaX_met * fCurveChng_1pm2 / 6.f;
    pTargetPath->fTargetPathHeading_rad =
        ATAN_(fCurveChng_1pm2 * fDeltaX_met * fDeltaX_met * 0.5f +
              fDeltaX_met * fCurve_1pm + fTanHead);
    pTargetPath->fTargetPathCurve_rad =
        fCurve_1pm + fDeltaX_met * fCurveChng_1pm2;
}

/*****************************************************************************
  Functionname:  PlausibilityCheck                                          */ /*!

  @brief: we did threshold and deviation check for the latency compensated data

  @description: right corridor parameter check, and corridor deviation check
between compensated
                        corridor parameter and original corridor parameter

  @param[in]
                        const TRJPLN_LatencyCompensationCorridor_t*
pCompCorridor: latency compensated corridor data
                        const TRJPLN_LatencyCompensationCorriParamOut_t*
pCompenCorriParam: coordinate system transform related parameter
                        const TRJPLN_LatencyCompensationAllCorridor_t*
pAllCorridorWithCCC: original corridor data before latency compensated
                        const float32 fEgoVelX_mps: ego velocity
  @param[out]
                        TRJPLN_LatencyCompensationPlasiCheck_t* pPlasiCheck:
plausibility check output

@uml
@startuml
start
:vehicle lateral position check;
:right corridor distY check;
:right corridor heading angle check;
:right corridor curve delta check;
:right corridor distY deviation check;
:target corridor distY deviation check;
:left corridor distY deviation check;
:target trajectory heading threashold check;
:output value assignment;
end
@enduml
  @return
*****************************************************************************/
void PlausibilityCheck(
    const TRJPLN_LatencyCompensationCorridor_t* pCompCorridor,
    const TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC,
    const float32 fEgoVelX_mps,
    TRJPLN_LatencyCompensationPlasiCheck_t* pPlasiCheck) {
    boolean bLateralPosCheck = FALSE;
    boolean bRightCorriDistYCheck = FALSE;
    boolean bRightCorriHeadingCheck = FALSE;
    boolean bRightCorriCurveCheck = FALSE;
    boolean bRightOutRangeCheck = FALSE;
    boolean bTargetOutRangeCheck = FALSE;
    boolean bLeftOutRangeCheck = FALSE;
    boolean bTgtCorridorHeadingCheck;

    // vehicle lateral position check
    bLateralPosCheck = fABS(pCompCorridor->fDevDistY_met +
                            pAllCorridorWithCCC->fRiCridBndPosY0_met) >
                       TPLLCO_DISTYCHECKTOLERANCE_MET;
    // right corridor distY check
    bRightCorriDistYCheck = fABS(pCompCorridor->fRiCorridorPosY0_met) > 1e-3f;
    // right corridor heading angle check
    bRightCorriHeadingCheck =
        fABS(pCompCorridor->fRiCorridorHeadingAgl_rad) > 1e-3f;
    // right corridor curve delta check
    bRightCorriCurveCheck =
        fABS(pCompCorridor->fRiCorridorCurve_1pm -
             pAllCorridorWithCCC->fRiCridBndCrv_1pm) >
        (TPLLCO_CRVCHECKTOLERANCE_MET * 2.f /
         TUE_CML_Sqr(pCompCorridor->fTargetCorridorLength_met));

    // right corridor distY deviation check
    TRJPLN_TrajectoryParams_t sRiOriginalCorridorData = {
        pAllCorridorWithCCC->fRiCridBndPosX0_met,
        pAllCorridorWithCCC->fRiCridBndPosY0_met,
        pAllCorridorWithCCC->fRiCridBndHeadAng_rad,
        pAllCorridorWithCCC->fRiCridBndCrv_1pm,
        pAllCorridorWithCCC->fRiCridBndCrvChng_1pm2,
        pAllCorridorWithCCC->fRiCridBndLength_met};
    TRJPLN_TrajectoryParams_t sRiLatenCompenCorridorData = {
        pCompCorridor->fRiCorridorPosX0_met,
        pCompCorridor->fRiCorridorPosY0_met,
        pCompCorridor->fRiCorridorHeadingAgl_rad,
        pCompCorridor->fRiCorridorCurve_1pm,
        pCompCorridor->fRiCorridorCrvChng_1pm2,
        pCompCorridor->fRiCorridorLength_met};
    DistYDeviationPlausibilityCheck(
        sRiOriginalCorridorData, sRiLatenCompenCorridorData,
        pCompenCorriParam->fRightOrientation_rad,
        pCompenCorriParam->fPosRightPosX_met,
        pCompenCorriParam->fPosRightPosY_met, 4u,
        pPlasiCheck->fLeCorridorTransDev_met, &bRightOutRangeCheck);

    // target corridor distY deviation check
    TRJPLN_TrajectoryParams_t sTgtOriginalCorridorData = {
        pAllCorridorWithCCC->fTgtTrajPosX0_met,
        pAllCorridorWithCCC->fTgtTrajPosY0_met,
        pAllCorridorWithCCC->fTgtTrajHeadingAng_rad,
        pAllCorridorWithCCC->fTgtTrajCurve_1pm,
        pAllCorridorWithCCC->fTgtTrajCrvChng_1pm2,
        pAllCorridorWithCCC->fTgtTrajLength_met};
    TRJPLN_TrajectoryParams_t sTgtLatenCompenCorridorData = {
        pCompCorridor->fTargetCorridorPosX0_met,
        pCompCorridor->fTargetCorridorPosY0_met,
        pCompCorridor->fTargetCorridorHeading_rad,
        pCompCorridor->fTargetCorridorCurve_1pm,
        pCompCorridor->fTargetCorridorCrvChng_1pm2,
        pCompCorridor->fTargetCorridorLength_met};
    DistYDeviationPlausibilityCheck(
        sTgtOriginalCorridorData, sTgtLatenCompenCorridorData,
        pCompenCorriParam->fRightOrientation_rad,
        pCompenCorriParam->fPosRightPosX_met,
        pCompenCorriParam->fPosRightPosY_met, 4u,
        pPlasiCheck->fTargetCorridorTransDev_met, &bTargetOutRangeCheck);

    // left corridor distY deviation check
    TRJPLN_TrajectoryParams_t sLeOriginalCorridorData = {
        pAllCorridorWithCCC->fLeCridBndPosX0_met,
        pAllCorridorWithCCC->fLeCridBndPosY0_met,
        pAllCorridorWithCCC->fLeCridBndHeadAng_rad,
        pAllCorridorWithCCC->fLeCridBndCrv_1pm,
        pAllCorridorWithCCC->fLeCridBndCrvChng_1pm2,
        pAllCorridorWithCCC->fLeCridBndLength_met};
    TRJPLN_TrajectoryParams_t sLeLatenCompenCorridorData = {
        pCompCorridor->fLeCorridorPosX0_met,
        pCompCorridor->fLeCorridorPosY0_met,
        pCompCorridor->fLeCorridorHeadingAgl_rad,
        pCompCorridor->fLeCorridorCurve_1pm,
        pCompCorridor->fLeCorridorCrvChng_1pm2,
        pCompCorridor->fLeCorridorLength_met};
    DistYDeviationPlausibilityCheck(
        sLeOriginalCorridorData, sLeLatenCompenCorridorData,
        pCompenCorriParam->fRightOrientation_rad,
        pCompenCorriParam->fPosRightPosX_met,
        pCompenCorriParam->fPosRightPosY_met, 4u,
        pPlasiCheck->fLeCorridorTransDev_met, &bLeftOutRangeCheck);

    // target trajectory heading threashold check
    float32 afTableInputX1[TPLLCO_TGTCRIDRHEADTOLCRV_TABLENUM_NU] =
        TPLLCO_CORRIDORCURVATURE_TABLEX_1PM;
    float32 afTableInputY1[TPLLCO_TGTCRIDRHEADTOLCRV_TABLENUM_NU] =
        TPLLCO_TGTCRIDRHEADTOLCRV_TABLEY_RAD;
    float32 fCurvatureLookupValue = TUE_CML_LookUpTable2D(
        pAllCorridorWithCCC->fTgtTrajCurve_1pm, afTableInputX1, afTableInputY1,
        TPLLCO_TGTCRIDRHEADTOLCRV_TABLENUM_NU);

    float32 afTableInputX2[TPLLCO_TGTCRIDRHEADTOLVELX_TABLENUM_NU] =
        TPLLCO_VEHVELX_TABLEX_MPS;
    float32 afTableInputY2[TPLLCO_TGTCRIDRHEADTOLVELX_TABLENUM_NU] =
        TPLLCO_TGTCRIDRHEADTOLVELX_TABLEY_RAD;
    float32 fEgoVelXLookupValue =
        TUE_CML_LookUpTable2D(fEgoVelX_mps, afTableInputX2, afTableInputY2,
                              TPLLCO_TGTCRIDRHEADTOLVELX_TABLENUM_NU);

    bTgtCorridorHeadingCheck =
        fABS(pAllCorridorWithCCC->fTgtTrajHeadingAng_rad) >
            fCurvatureLookupValue ||
        fABS(pAllCorridorWithCCC->fTgtTrajHeadingAng_rad) > fEgoVelXLookupValue;

    // plausibility check output logic
    TPLLCO_SetBit(0u, bLateralPosCheck, &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(1u, bRightCorriDistYCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(2u, bRightCorriHeadingCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(3u, bRightCorriCurveCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(4u, bRightOutRangeCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(5u, bTargetOutRangeCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);
    TPLLCO_SetBit(6u, bLeftOutRangeCheck,
                  &pPlasiCheck->uiDPlausiCheckStatus_nu);

    TPLLCO_SetBit(0u, bLateralPosCheck, &pPlasiCheck->uiSPlausiCheckStatus_nu);
    TPLLCO_SetBit(1u,
                  bRightCorriDistYCheck || bRightCorriHeadingCheck ||
                      bRightCorriCurveCheck || bRightOutRangeCheck,
                  &pPlasiCheck->uiSPlausiCheckStatus_nu);
    TPLLCO_SetBit(2u, bTargetOutRangeCheck,
                  &pPlasiCheck->uiSPlausiCheckStatus_nu);
    TPLLCO_SetBit(3u, bLeftOutRangeCheck,
                  &pPlasiCheck->uiSPlausiCheckStatus_nu);
    TPLLCO_SetBit(4u, bTgtCorridorHeadingCheck,
                  &pPlasiCheck->uiSPlausiCheckStatus_nu);
}

/*****************************************************************************
  Functionname:LCF_TRJPLN_LatencyCompensation_Reset */ /*!

@brief: global value reset function of latency compensation module

@description: global value reset function of latency compensation module

@param[in]
@param[out]

@return
*****************************************************************************/
void LCF_TRJPLN_LatencyCompensation_Reset(void) {
    memset(&TRJPLN_Ss_EgoHistoryMovement_nu, 0,
           sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
    TRJPLN_Sb_CCCModeChange_nu = FALSE;
    memset(TRJPLN_Sa_CtrlPtDev_met, 0, sizeof(float32) * 2);
    // TRJPLN_Sa_CtrlPtDev_met[2] = { 0.f };
    memset(&TRJPLN_Ss_AccumuEgoMotion_nu, 0,
           sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
    memset(&TRJPLN_Ss_CorriFrozChckHisCorr_nu, 0,
           sizeof(TRJPLN_LatencyCompensationAllCorridor_t));
    TRJPLN_Sb_CorriFrozChckTrajLengthVald_b = FALSE;
    TRJPLN_Sf_CorriFrozChckHistoryCCCLength_met = 0.f;
    TRJPLN_Sc_CCCResetAccumuLaneQual_nu = 0;
    TRJPLN_Sb_CCCPassCheck_nu = FALSE;
    TRJPLN_Sf_CalcCurVehPosLastTimeStamp_sec = 0.f;
    TRJPLN_Sb_TrajGuiQuaREQEdgeRise_nu = FALSE;
    TRJPLN_Sb_TrajGuiQuaTGQEdgeRise_nu = FALSE;
    TRJPLN_Sb_LastCycleTriggerReplan_nu = FALSE;
    memset(TRJPLN_Sa_CalcCurVehDeltaDist_met, 0,
           sizeof(float32) * TPLLCO_ODOHISTLENGTH_NU);
    // TRJPLN_Sa_CalcCurVehDeltaDist_met[TPLLCO_ODOHISTLENGTH_NU] = { 0.f };
    memset(TRJPLN_Sa_CalcCurVehDelOri_rad, 0,
           sizeof(float32) * TPLLCO_ODOHISTLENGTH_NU);
    // TRJPLN_Sa_CalcCurVehDelOri_rad[TPLLCO_ODOHISTLENGTH_NU] = { 0.f };
    TRJPLN_Sc_CalcCurVeArrayId_nu = 0u;
    TRJPLN_Sf_SetDevReplanHeadingInit_rad = 0.f;
    memset(&TRJPLN_Ss_LastCycleRightCorr_nu, 0,
           sizeof(TRJPLN_TrajectoryParams_t));
    TRJPLN_Sf_SetDevDistYInit_met = 0.f;
    TRJPLN_Sb_SetDevLastTrigReplan_nu = FALSE;
    TRJPLN_Sf_LastCylceHeadingVeh_rad = 0.f;
    TRJPLN_Sf_LastCyclePosDevX_met = 0.f;
    TRJPLN_Sf_LastCyclePosDevY_met = 0.f;
}

/*****************************************************************************
  Functionname:   EgoMotionCalculation */ /*!

@brief: calcualte history accumulate movement data before reset

@description: output zero data whie CCCReset if true, or accumulate YawAngle,
                       DistanceX and DistanceY before reset

@param[in]
               fYawrate: current yawrate of ego car
               fCycletime: cycle time
               fVelX: ego velocity
               bCCCReset: flag to show whether CCC(course consistency
check) module is reset or not

@param[out]
               pEgoMotion: distance movement X and Y, YawAngle changed
@return
*****************************************************************************/
void EgoMotionCalculation(const float32 fYawrate,
                          const float32 fCycletime,
                          const float32 fVelX,
                          const boolean bCCCReset,
                          TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion) {
    if (!bCCCReset) {
        TRJPLN_Ss_AccumuEgoMotion_nu.fEgoYawAngle_rad += fYawrate * fCycletime;
        TRJPLN_Ss_AccumuEgoMotion_nu.fEgoDistX_met +=
            COS_HD_(TRJPLN_Ss_AccumuEgoMotion_nu.fEgoYawAngle_rad) * fVelX *
            fCycletime;
        TRJPLN_Ss_AccumuEgoMotion_nu.fEgoDistY_met +=
            SIN_HD_(TRJPLN_Ss_AccumuEgoMotion_nu.fEgoYawAngle_rad) * fVelX *
            fCycletime;
    } else {
        memset(&TRJPLN_Ss_AccumuEgoMotion_nu, 0,
               sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
    }
    memcpy(pEgoMotion, &TRJPLN_Ss_AccumuEgoMotion_nu,
           sizeof(TRJPLN_LatencyCompensationEgoMotion_t));
}
/*****************************************************************************
  Functionname: TestPointPositionCal */ /*!

@brief: calculate a test point on the trajectory

@description: choose sample point on trajectory on at least 5 meters away

@param[in]
               equation parameters of trajectory curve
               fTgtTrajLength:
               fTgtTrajPosX0:
               fTgtTrajPosY0:
               fTgtTrajHeading:
               fTgtTrajCurve:
               fTgtTrajCurveChng:
@param[out]
               sample point DistanceX and DistanceY, which is calculated
based on input equation parameters
               fTestPointDistX:
               fTestPointDistY:
@return
*****************************************************************************/
void TestPointPositionCal(float32 fTgtTrajLength,
                          float32 fTgtTrajPosX0,
                          float32 fTgtTrajPosY0,
                          float32 fTgtTrajHeading,
                          float32 fTgtTrajCurve,
                          float32 fTgtTrajCurveChng,
                          float32* fTestPointDistX,
                          float32* fTestPointDistY) {
    *fTestPointDistX = MAX(fTgtTrajLength * TPLLCO_CCCTESTDISXPERC_NU,
                           TPLLCO_CCCMINTESTDISTX_MET);
    *fTestPointDistY = TUE_CML_CalcDistYOfClothoidsCurve(
        fTgtTrajHeading, fTgtTrajCurve, fTgtTrajCurveChng,
        *fTestPointDistX - fTgtTrajPosX0, fTgtTrajPosY0);
}
/*****************************************************************************
  Functionname:TestPointNewPositionCal */ /*!

@brief: calculate ego motion compensated point position

@description: new point position  = old point position  + motion compensated
value

@param[in]
      fTestPointDistX: point position distanceX without ego motion
compensated
      fTestPointDistY: point position distanceY without ego motion
compensated
      sEgoMotion: ego history moved DistanceX, DistanceY and heading
angle change
@param[out]
      fCCCTestPointDistX: point positionX with ego motion
compensated
      fCCCTestPointDistY: point positionY with ego motion
compensated
@return
*****************************************************************************/
void TestPointNewPositionCal(float32 fTestPointDistX,
                             float32 fTestPointDistY,
                             TRJPLN_LatencyCompensationEgoMotion_t sEgoMotion,
                             float32* fCCCTestPointDistX,
                             float32* fCCCTestPointDistY) {
    *fCCCTestPointDistX =
        (-SIN_HD_(sEgoMotion.fEgoYawAngle_rad) * fTestPointDistY) +
        (COS_HD_(sEgoMotion.fEgoYawAngle_rad) * fTestPointDistX) +
        sEgoMotion.fEgoDistX_met;
    *fCCCTestPointDistY =
        (COS_HD_(sEgoMotion.fEgoYawAngle_rad) * fTestPointDistY) +
        (SIN_HD_(sEgoMotion.fEgoYawAngle_rad) * fTestPointDistX) +
        sEgoMotion.fEgoDistY_met;
}
/*****************************************************************************
  Functionname: CorridorFrozenCheck                                           */ /*!

@brief: calculate reference corridor value and valid length

@description: the length and related corridor value would use history corridor
value,
                          unless CCCReset is TRUE or edge rising check
passed for target trajectory length.
                          the valid length of corridor is history trajectory
's length minus ego moved distance

@param[in]
                  bCCCReset:CCC module is reseted or not
                  pAllCorridor: current measured corridor values
                  fEgoMovedDistX: ego car moved history distance since last
reset
@param[out]
                  fCCCLength: the length of reference trajectory
                  fValidLength: the valid length of trajectory, exclude the
moving distance from ego car
                  pAllCorrdorsReference: the reference corridor value, which
will be used to compare with current corridor

@uml
@startuml
start
:target trajectory valid length edge rising check;
if (CCCReset || edge rising is TRUE)
  :reset the CCC length and reference corridor value;
endif
:valid length  = CCCLength - ego moved;
end
@enduml
@return
*****************************************************************************/
void CorridorFrozenCheck(
    boolean bCCCReset,
    TRJPLN_LatencyCompensationAllCorridor_t pAllCorridor,
    float32 fEgoMovedDistX,
    float32* fCCCLength,
    float32* fValidLength,
    TRJPLN_LatencyCompensationAllCorridor_t* pAllCorrdorsReference) {
    boolean bTrajLengthValidRising =
        TUE_CML_RisingEdgeSwitch(pAllCorridor.fTgtTrajLength_met > 0,
                                 &TRJPLN_Sb_CorriFrozChckTrajLengthVald_b);
    if (bTrajLengthValidRising || bCCCReset) {
        TRJPLN_Sf_CorriFrozChckHistoryCCCLength_met =
            pAllCorridor.fTgtTrajLength_met;
        memcpy(&TRJPLN_Ss_CorriFrozChckHisCorr_nu, &pAllCorridor,
               sizeof(TRJPLN_LatencyCompensationAllCorridor_t));
    }
    *fCCCLength = TRJPLN_Sf_CorriFrozChckHistoryCCCLength_met;
    memcpy(pAllCorrdorsReference, &TRJPLN_Ss_CorriFrozChckHisCorr_nu,
           sizeof(TRJPLN_LatencyCompensationAllCorridor_t));
    *fValidLength =
        TRJPLN_Sf_CorriFrozChckHistoryCCCLength_met - fEgoMovedDistX;
}
/*****************************************************************************
  Functionname:CCCResetCheck                                            */ /*!

              @brief: CCC module reset signal check

              @description: the CCC module should be reset while (lane quality
            too
            low
            && (test point distance too far||
                                            test point distance is too far for
            the
            valid
            distance))

              @param[in]:
                                    fCCCLength: the reference trajectory length
                                    fTestNewPointX: the sample point which is
            used
            for
            CCC check
                                    sLaneData: ego lane quality data
              @param[out]
                                    fCCCPassedPerc: The proportion of the sample
            point
            distance to the total reference trajectory length
                                    bCCCReset: CCC reset check result
              @return
            *****************************************************************************/
void CCCResetCheck(float32 fCCCLength,
                   float32 fTestNewPointX,
                   TRJPLN_LatencyCompensationConsisCheck_t sLaneData,
                   float32* fCCCPassedPerc,
                   boolean* bCCCReset) {
    // average lane quality is calculated
    float32 fLaneQuality =
        (sLaneData.uiLeLnQuality_per + sLaneData.uiRiLnQuality_per) / 2.0f;
    float32 fLaneCurveQuality =
        (sLaneData.uiLeCrvQuality_per + sLaneData.uiRiCrvQuality_per) / 2.0f;

    // accumulate number while lane quality is too low
    if (fLaneQuality < TPLLCO_CCCMINLNQUALITY_PERC ||
        fLaneCurveQuality < TPLLCO_CCCMINCRVQUALITY_PERC ||
        TRJPLN_Sc_CCCResetAccumuLaneQual_nu >= 250u) {
        TRJPLN_Sc_CCCResetAccumuLaneQual_nu = 0u;
    } else {
        TRJPLN_Sc_CCCResetAccumuLaneQual_nu++;
    }

    *fCCCPassedPerc = fTestNewPointX / MAX(0.0001f, fCCCLength);

    // CCC reset check
    if ((TPLLCO_CCCRESETMAXDISTX_MET < fTestNewPointX ||
         *fCCCPassedPerc > TPLLCO_CCCRESETMAXPERC_NU) &&
        TRJPLN_Sc_CCCResetAccumuLaneQual_nu >= TPLLCO_CCCMINQUCYCLE_NU) {
        *bCCCReset = TRUE;
    } else {
        *bCCCReset = FALSE;
    }
}
/*****************************************************************************
  Functionname: CCCWarningCheck                                           */ /*!

        @brief: output CCC warning, invalid, devation value

        @description: calculate deviation Y value between current corridor and
      reference corridor
                                      with same distance X

        @param[in]
                              sAllCorrdorsReference: history reference corridor
                              fCCCTestPointDistX: sample point X as a reference
      point for comparison
                              fCCCTestPointDistY:sample point Y as a reference
      point
      for comparison
                              fCCCPassedPerc: The proportion of the sample point
      distance to the total reference trajectory length
                              fValidLength: remaining valid distance for
      reference
      corridor, which minus ego moved distance

        @param[out]
                              fCCCDeviation: the deviation value between current
      corridor DistY and reference corridor DistY, with same
                              input sample point DistX
                              bCCCWarning: CCC check output warning/failed
      signal
      while deviation is too large or length percent check failed
                              bCCCInvalid: CCC check is invalid while reference
      valid length is less than zero meter
        @return

         @uml
        @startuml
        start
      :calculate reference distY;
      :distY deviation check between reference and current measured;
      :CCC deviation check and passed percent of trajectory check;
      :RS check with above result;
      if (RSCheck passed && (target trajectory curvature less than threshold ||
      switch on))
              :CCC Warning is true while reference trajectory remained length
      valid;
              :CCC Invalid is true while valid length less than zero;
      else
              :CCC warning and invalid check is FALSE;
      endif
        end
        @enduml
      *****************************************************************************/
void CCCWarningCheck(
    TRJPLN_LatencyCompensationAllCorridor_t sAllCorrdorsReference,
    float32 fCCCTestPointDistX,
    float32 fCCCTestPointDistY,
    float32 fCCCPassedPerc,
    float32 fValidLength,
    float32* fCCCDeviation,
    boolean* bCCCWarning,
    boolean* bCCCInvalid) {
    // calculate test point position Y with reference corridor
    float32 fCorridPosY = TUE_CML_CalcDistYOfClothoidsCurve(
        sAllCorrdorsReference.fTgtTrajHeadingAng_rad,
        sAllCorrdorsReference.fTgtTrajCurve_1pm,
        sAllCorrdorsReference.fTgtTrajCrvChng_1pm2,
        fCCCTestPointDistX - sAllCorrdorsReference.fTgtTrajPosX0_met,
        sAllCorrdorsReference.fTgtTrajPosY0_met);
    // calculate deviation distance between measured distance Y and calculated
    // reference distance Y, based on sample point distance X
    *fCCCDeviation = fCorridPosY - fCCCTestPointDistY;

    boolean bCCCDeviationCheck =
        TUE_CML_Abs(*fCCCDeviation) > TPLLCO_CCCTHRESHOLDMAXDEX_MET;
    boolean bCCCPercCheckFail = fCCCPassedPerc <= TPLLCO_CCCVALIDMAXPERC_NU;

    // reset RS check when distance Y deviation check failed
    boolean bRSCheck =
        TUE_CML_RSFlipFlop((bCCCDeviationCheck && bCCCPercCheckFail),
                           !bCCCDeviationCheck, &TRJPLN_Sb_CCCPassCheck_nu);

    // check CCC warning and invalid while RS cehck passed and the ego car is
    // running on a straight road
    if (bRSCheck && (TUE_CML_Abs(sAllCorrdorsReference.fTgtTrajCurve_1pm) <=
                         TPLLCO_CCCMAXCURVATURE_1PM ||
                     TPLLCO_CCCENABLECURVE_NU)) {
        *bCCCWarning = (fValidLength > 0.f);
        *bCCCInvalid = (fValidLength < TPLLCO_CCCMINVALIDLENGTH_MET);
    } else {
        *bCCCWarning = FALSE;
        *bCCCInvalid = FALSE;
    }
}

/*****************************************************************************
  Functionname:CalcCurrentVehPosition */ /*!

@brief: calculate ego car distance and heading angle delta value between SEN
timestamp and VEH timestamp

@description:
         we will store ego car moved distance and heading angle based
on velocity and yawrate, to calculate the latency
         distance and heading angle value. if the latency cycle is 5
cycles, we will output delta distance and heading
         angle based on the summary result of last 5 cycles' delta
distance and delta heading angle

@param[in]:
         TRJPLN_LatencyCompensationLatComIn_t sLatCompIn: the
trajectory input system state, time stamp and function switch
         TRJPLN_LatencyCompensationCommon_t sCommonInput: cycle time
input, ego velocity and yawrate input, cycle time input,
         trajectory plan control mode input from lateral control
function

@param[out]:
         float32* fMeasDeltaTime_sec: the delta time between lateral
control requirement timestamp and current velocity and
         yawrate timestamp
         TPLLCO_EgoOdoVeh_t* pDevOdoVeh: distance delta and heading
angle delta value in current cycle
         TPLLCO_EgoDeviationSum_t* pDevEgo: distance delta and heading
angle delta value between lateral control requirement
         timestamp and current VED timestamp
         boolean* bTriggerReplan: whether we need a trajectory replan
flag, based on delta time and trajectory plan
         requirement mode change

@uml
@startuml
start
if (time diff more than 0.001 second or the trajectory control requirement mode
edge rising check pass) then (yes)
:trigger replan check pass;
else (no)
:trigger replan check fail;
endif
if (latency compensate switch is activated) then (yes)
:recored moved distance and angle in current cycle;
endif
if (trigger replan passed) then (yes)
:calculate delta cycle number between SEN and VEH timestamp;
:output compensate distance and angle value from history records and delta
cycle;
else (no)
:compensate value is zero;
endif
end
@enduml
@return
*****************************************************************************/
void CalcCurrentVehPosition(
    const TRJPLN_LatencyCompensationLatComIn_t sLatCompIn,
    const TRJPLN_LatencyCompensationCommon_t sCommonInput,
    float32* fMeasDeltaTime_sec,
    TPLLCO_EgoOdoVeh_t* pDevOdoVeh,
    TPLLCO_EgoDeviationSum_t* pDevEgo,
    boolean* bTriggerReplan) {
    // step 1, replan trigger calculate
    float32 fCurrentCycelTime_sec =
        TPLLCO_USESENSORTSTAMP_NU ? sLatCompIn.fSensorTimeStamp_sec
                                  : (sLatCompIn.uiSenToVehTStamp_us * 1e-6f);
    boolean fCycleTimeCheck =
        ((fABS(fCurrentCycelTime_sec -
               TRJPLN_Sf_CalcCurVehPosLastTimeStamp_sec) > 0.001f) &&
         !(TRJPLN_Sb_LastCycleTriggerReplan_nu &&
           TPLLCO_SUPRESSDOUBLETRIGRPL_NU));
    TRJPLN_Sf_CalcCurVehPosLastTimeStamp_sec = fCurrentCycelTime_sec;
    *bTriggerReplan =
        TUE_CML_RisingEdgeSwitch(
            sCommonInput.uiTrajGuiQualifier_nu == E_LCF_TGQ_REQ_REFCHNG,
            &TRJPLN_Sb_TrajGuiQuaREQEdgeRise_nu) ||
        TUE_CML_RisingEdgeSwitch(
            sCommonInput.uiTrajGuiQualifier_nu == E_LCF_TGQ_REQ,
            &TRJPLN_Sb_TrajGuiQuaTGQEdgeRise_nu) ||
        fCycleTimeCheck;
    TRJPLN_Sb_LastCycleTriggerReplan_nu = *bTriggerReplan;

    // step 2, calculate history ego movement
    if (sLatCompIn.bLatencyCompActivated) {
        TRJPLN_Sa_CalcCurVehDeltaDist_met[TRJPLN_Sc_CalcCurVeArrayId_nu] =
            sCommonInput.fEgoVelX_mps * sCommonInput.fCycleTimeVeh_sec;
        TRJPLN_Sa_CalcCurVehDelOri_rad[TRJPLN_Sc_CalcCurVeArrayId_nu] =
            sCommonInput.fEgoYawRate_rps * sCommonInput.fCycleTimeVeh_sec;
    } else {
        TRJPLN_Sa_CalcCurVehDeltaDist_met[TRJPLN_Sc_CalcCurVeArrayId_nu] = 0.f;
        TRJPLN_Sa_CalcCurVehDelOri_rad[TRJPLN_Sc_CalcCurVeArrayId_nu] = 0.f;
    }
    pDevOdoVeh->fDevOdoEgoHead_rad =
        TRJPLN_Sa_CalcCurVehDelOri_rad[TRJPLN_Sc_CalcCurVeArrayId_nu];
    pDevOdoVeh->fDevOdoEgoDist_met =
        TRJPLN_Sa_CalcCurVehDeltaDist_met[TRJPLN_Sc_CalcCurVeArrayId_nu];

    // step 3, interpolation ego veh position
    if (*bTriggerReplan) {
        // step 3.1, calculate latency time
        uint8 fLatencyCycleNum_nu = 0u;
        if (sLatCompIn.bLatencyCompActivated) {
            float32 fTimeDiff_sec = 0.f;
            if (!TPLLCO_TIMEDIFFSWITCH_NU) {
                fTimeDiff_sec = sLatCompIn.uiVehSync4LCO_us * (float32)1e-6 -
                                sLatCompIn.fSensorTimeStamp_sec;
            } else {
                fTimeDiff_sec = TPLLCO_TIMEDIFF_SEC;
            }
            if (fTimeDiff_sec >= 0) {
                fLatencyCycleNum_nu = (uint8)ROUND_TO_INT(
                    fTimeDiff_sec / sCommonInput.fCycleTimeVeh_sec);
            } else {
                fLatencyCycleNum_nu = (uint8)ROUND_TO_INT(
                    (TPLLCO_ODOMAXTIME_US * (float32)1e-6 + fTimeDiff_sec) /
                    sCommonInput.fCycleTimeVeh_sec);
            }
        }
        uint8 uiNumHistOdo_nu =
            MIN(TPLLCO_ODOHISTLENGTH_NU, fLatencyCycleNum_nu);
        uint8 uiIdxOld_nu = (TRJPLN_Sc_CalcCurVeArrayId_nu - uiNumHistOdo_nu) %
                            TPLLCO_ODOHISTLENGTH_NU;
        *fMeasDeltaTime_sec =
            (float32)uiNumHistOdo_nu * sCommonInput.fCycleTimeVeh_sec;
        // step 3.2, interpolation calculate
        float32 fSumDeltaHeading_rad = 0.f;
        float32 fSumDeltaDistX_met = 0.f;
        float32 fSumDeltaDistY_met = 0.f;
        for (sint8 i = uiNumHistOdo_nu; i >= 0; i--) {
            uint8 uiCurrentId_nu = (uiIdxOld_nu + i) % TPLLCO_ODOHISTLENGTH_NU;
            fSumDeltaHeading_rad +=
                TRJPLN_Sa_CalcCurVehDelOri_rad[uiCurrentId_nu];
            fSumDeltaDistX_met +=
                TRJPLN_Sa_CalcCurVehDeltaDist_met[uiCurrentId_nu] *
                COS_HD_(fSumDeltaHeading_rad);
            fSumDeltaDistY_met +=
                TRJPLN_Sa_CalcCurVehDeltaDist_met[uiCurrentId_nu] *
                SIN_HD_(fSumDeltaHeading_rad);
        }
        pDevEgo->fDevHeadingEgo_rad = fSumDeltaHeading_rad;
        pDevEgo->fEgoDevPosXSum_met = fSumDeltaDistX_met;
        pDevEgo->fEgoDevPosYSum_met = fSumDeltaDistY_met;
    }

    TRJPLN_Sc_CalcCurVeArrayId_nu =
        TRJPLN_Sc_CalcCurVeArrayId_nu + 1 < TPLLCO_ODOHISTLENGTH_NU
            ? TRJPLN_Sc_CalcCurVeArrayId_nu + 1
            : 0;
}

/*****************************************************************************
  Functionname:TranslateRotateCoriCoord */ /*!

@brief: calculate rectangular projection of the ego vehicle to the right
corridor

@description:
   calculate rectangular projected point from ego car to reference
right corridor,
   calculate delta distance from previewed ego distance and
projected point on right corridor.

@param[in]:
   const TPLLCO_EgoDeviationSum_t sDevEgo: ego moved value in
current cycle
   const TRJPLN_LatencyCompensationEgoMotion_t sEgoMotion: ego
moved distance since the reference corridor is reset
   const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor:
lane corridor data
   float32 fEgoVelX_mps: ego velocity
@param[out]
   TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam:
calculate the rectangular projection of the vehicle from
   the right corridor
   float32 fCtrlPtDev_met[2]: previewed control point delta
distance, which is interpolated by ego velocity
@uml
@startuml
start
:calculate heading angle from last reference reset time to current time;
if(PREVIEWD DISTANCE SWITCH on) then (yes)
:interpolated distance value is calculated based ego velocity;
:previewed distance add to control point deviation value;
endif
:delta distance calculated from last reset time to current time;
while (i < 10)
:i++;
note:Cyclic calculation of the value of the rectangular projection from the car
to the right corridor line
:calcualte projected point value;
:calculate delta distance from ego point to projected point on right corridor;
:consider the heading angle changed of right corridor, which is caused by ego
movement;
endwhile
:output value;
end
@enduml
@return
*****************************************************************************/
void TranslateRotateCoriCoord(
    const TPLLCO_EgoDeviationSum_t sDevEgo,
    const TRJPLN_LatencyCompensationEgoMotion_t sEgoMotion,
    const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor,
    float32 fEgoVelX_mps,
    uint8 uiLatCtrlMode_nu,
    TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    float32 fCtrlPtDev_met[2]) {
    // the delta heading angle = current cycle heading angle changed value +
    // history accumulate delta angle since reference corridor changed
    float32 fEgoHeadAgl_rad =
        sDevEgo.fDevHeadingEgo_rad + sEgoMotion.fEgoYawAngle_rad;

    if (TPLLCO_PREVIEWDISTXENABLE_NU) {
        // velocity interpolated to calculate the preview distance
        float32 afTableInputX[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
            TPLLCO_VEHVELX2PREVIEWDISTX_TABLEX_MPS;

        float32 afTableInputYOF[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
            TPLLCO_VEHVELX2PREVIEWDISTXOF_TABLEY_MET;
        memcpy(afTableInputYOF, TP_VEHVELX2PREVIEWDISTXOF_TABLEY_MET,
               13 * sizeof(real32_T));
        float32 afTableInputY[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
            TPLLCO_VEHVELX2PREVIEWDISTX_TABLEY_MET;
        float32 fPreviewDistX_met = 0.f;
        if (uiLatCtrlMode_nu == 2U) {
            fPreviewDistX_met = TUE_CML_LookUpTable2D(
                fEgoVelX_mps, afTableInputX, afTableInputYOF,
                TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU);
        } else {
            fPreviewDistX_met = TUE_CML_LookUpTable2D(
                fEgoVelX_mps, afTableInputX, afTableInputY,
                TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU);
        }
        fCtrlPtDev_met[0] = fPreviewDistX_met * COS_HD_(fEgoHeadAgl_rad);
        fCtrlPtDev_met[1] = fPreviewDistX_met * SIN_HD_(fEgoHeadAgl_rad);
    }
    // the ego point to be projected on the right corridor
    pCompenCorriParam->fDeltaCoordPosX_met = sEgoMotion.fEgoDistX_met +
                                             sDevEgo.fEgoDevPosXSum_met +
                                             fCtrlPtDev_met[0];
    pCompenCorriParam->fDeltaCoordPosY_met = sEgoMotion.fEgoDistY_met +
                                             sDevEgo.fEgoDevPosYSum_met +
                                             fCtrlPtDev_met[1];

    // calculate the orthogonal projection of ego car point to right corridor
    float32 afPosRight_met[2] = {0.f};   // the projected reference point in
                                         // right corridor at previewed ego
                                         // distance X
    float32 afDeltaProj_met[2] = {0.f};  // the delta distance X/Y value between
                                         // projected right corridor reference
                                         // point and preview car position
    float32 fRightOri_rad =
        0.f;  // the right corridor heading angle at previewed ego distance X
    float32 afPosRightLastCycle_met[2] = {0.f};  // the extra right corridor
                                                 // reference point value change
                                                 // caused by ego movement
    for (uint8 i = 0; i < TPLLCO_NUMITER_NU; i++) {
        afPosRight_met[0] =
            pCompenCorriParam->fDeltaCoordPosX_met - afPosRightLastCycle_met[0];

        float32 fTanHead = TAN_HD_(sAllCorridor.fRiCridBndHeadAng_rad);

        float32 fXDelta = afPosRight_met[0] - sAllCorridor.fRiCridBndPosX0_met;
        // Y = Y0+ X*tan(HeadingAgl) + 0.5*X*X*Curve + 1/6 * X*X*X*CurveChange
        afPosRight_met[1] =
            sAllCorridor.fRiCridBndPosY0_met + fXDelta * fTanHead +
            0.5f * TUE_CML_Sqr(fXDelta) * sAllCorridor.fRiCridBndCrv_1pm +
            fXDelta * fXDelta * fXDelta * sAllCorridor.fRiCridBndCrvChng_1pm2 /
                6.f;
        // angle = ARCTAN(CurveChange*X*X*0.5 + X*Curve + tan(HeadingAgl))
        fRightOri_rad = ATAN_(
            sAllCorridor.fRiCridBndCrvChng_1pm2 * fXDelta * fXDelta * 0.5f +
            fXDelta * sAllCorridor.fRiCridBndCrv_1pm + fTanHead);

        afDeltaProj_met[0] =
            pCompenCorriParam->fDeltaCoordPosX_met - afPosRight_met[0];
        afDeltaProj_met[1] =
            pCompenCorriParam->fDeltaCoordPosY_met - afPosRight_met[1];

        // counter clockwise rotation by fPhi_rad angle
        float32 fPhi_rad = fRightOri_rad - ATAN_(-afDeltaProj_met[0] /
                                                 SafeDiv(afDeltaProj_met[1]));
        afPosRightLastCycle_met[0] = afDeltaProj_met[0] * COS_HD_(fPhi_rad) -
                                     afDeltaProj_met[1] * SIN_HD_(fPhi_rad);
        afPosRightLastCycle_met[1] = afDeltaProj_met[0] * SIN_HD_(fPhi_rad) +
                                     afDeltaProj_met[1] * COS_HD_(fPhi_rad);
    }
    pCompenCorriParam->fDeltaProjPosX_met = afDeltaProj_met[0];
    pCompenCorriParam->fDeltaProjPosY_met = afDeltaProj_met[1];
    pCompenCorriParam->fPosRightPosX_met = afPosRight_met[0];
    pCompenCorriParam->fPosRightPosY_met = afPosRight_met[1];
    pCompenCorriParam->fRightOrientation_rad = fRightOri_rad;
}

/*****************************************************************************
  Functionname:LatencyCompensationOutput */ /*!

@brief: left, right and target corridor time delay compensation

@description: calculate compensated corridor data, based on the rectangular
projected value
from ego movement to right corridor coordinate. and the new
output corridor data's origin
of the coordinate is changed to the foot point of the orthogonal
projection from
the car to the right lane corridor

@param[in]:
const TPLLCO_EgoOdoVeh_t sDevOdoVeh: ego movement data in
current cycle
float32 fDevHeadingEgo_rad: ego movement heading angle in
current cycle
boolean bTriggerReplan: replan trigger check flag
uint8 uiSysStateLCF_nu: LCF control mode from SEN function
const TRJPLN_LatencyCompensationCorriParamOut_t
sCompenCorriParam: rectangular projected related value from ego car
to right corridor coordinate
float32 fCtrlPtDev_met[2]: ego deviation data based on previewed
current control point of ego
float32 fEgoVelX_mps: ego velocity
const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor:
left, right and target corridor data
@param[out]
TRJPLN_LatencyCompensationCorridor_t* pCompCorridor: delay time
compensated and coordinate transformed corridor data
based on ego movement

@return
*****************************************************************************/
void LatencyCompensationOutput(
    const TPLLCO_EgoOdoVeh_t sDevOdoVeh,
    float32 fDevHeadingEgo_rad,
    boolean bTriggerReplan,
    uint8 uiSysStateLCF_nu,
    uint8 uiLatCtrlMode_nu,
    const TRJPLN_LatencyCompensationCorriParamOut_t sCompenCorriParam,
    float32 fCtrlPtDev_met[2],
    float32 fEgoVelX_mps,
    const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor,
    TRJPLN_LatencyCompensationCorridor_t* pCompCorridor) {
    boolean bTriggerReplanDeviaCal = FALSE;
    if ((TPLLCO_CALCULATIONBYRAMPOUT_NU
             ? uiSysStateLCF_nu == E_LCF_SYSSTATE_RAMPOUT
             : FALSE) ||
        (uiSysStateLCF_nu == E_LCF_SYSSTATE_CONTROLLING) ||
        (TPLLCO_CALCULATIONBYREQUEST_NU
             ? uiSysStateLCF_nu == E_LCF_SYSSTATE_REQUEST
             : FALSE)) {
        bTriggerReplanDeviaCal = TRUE;
    }
    pCompCorridor->bTriggerReplan = bTriggerReplan;

    // set all corridor data, time delay compensated and coordinate transform to
    // the new origin point
    if (bTriggerReplan) {
        // calculate compensated right corridor data based on ego projected
        // movement on right corridor coordinate
        TRJPLN_TrajectoryParams_t sInputCorridor = {
            sAllCorridor.fRiCridBndPosX0_met,
            sAllCorridor.fRiCridBndPosY0_met,
            sAllCorridor.fRiCridBndHeadAng_rad,
            sAllCorridor.fRiCridBndCrv_1pm,
            sAllCorridor.fRiCridBndCrvChng_1pm2,
            sAllCorridor.fRiCridBndLength_met};
        TRJPLN_TrajectoryParams_t sTrajTransOutputCorridor = {0.f};
        TRJPLN_TrajectoryParams_t sCoordTransOutputCorridor = {0.f};
        TPLLCO_TrajecotryTranslation(sCompenCorriParam.fPosRightPosX_met,
                                     sInputCorridor, &sTrajTransOutputCorridor);
        TPLLCO_CoordinateTranformation(
            sCompenCorriParam.fPosRightPosX_met,
            sCompenCorriParam.fPosRightPosY_met,
            sCompenCorriParam.fRightOrientation_rad, sTrajTransOutputCorridor,
            TPLLCO_COORDTRAFOMINLENGTH_MET, &sCoordTransOutputCorridor);
        TPLLCO_TranjParamsCopyValue(sCoordTransOutputCorridor,
                                    &pCompCorridor->fRiCorridorPosX0_met,
                                    &pCompCorridor->fRiCorridorPosY0_met,
                                    &pCompCorridor->fRiCorridorHeadingAgl_rad,
                                    &pCompCorridor->fRiCorridorCurve_1pm,
                                    &pCompCorridor->fRiCorridorCrvChng_1pm2,
                                    &pCompCorridor->fRiCorridorLength_met);

        // set left corridor data
        sInputCorridor.fX0_met = sAllCorridor.fLeCridBndPosX0_met;
        sInputCorridor.fY0_met = sAllCorridor.fLeCridBndPosY0_met;
        sInputCorridor.fHeading_rad = sAllCorridor.fLeCridBndHeadAng_rad;
        sInputCorridor.fCurve_1pm = sAllCorridor.fLeCridBndCrv_1pm;
        sInputCorridor.fCurveChng_1pm2 = sAllCorridor.fLeCridBndCrvChng_1pm2;
        sInputCorridor.fLength_met = sAllCorridor.fLeCridBndLength_met;
        memset(&sTrajTransOutputCorridor, 0, sizeof(TRJPLN_TrajectoryParams_t));
        memset(&sCoordTransOutputCorridor, 0,
               sizeof(TRJPLN_TrajectoryParams_t));
        TPLLCO_CoordinateTranformation(
            sCompenCorriParam.fPosRightPosX_met,
            sCompenCorriParam.fPosRightPosY_met,
            sCompenCorriParam.fRightOrientation_rad, sInputCorridor,
            TPLLCO_COORDTRAFOMINLENGTH_MET, &sCoordTransOutputCorridor);
        TPLLCO_TrajecotryTranslation(0.f, sCoordTransOutputCorridor,
                                     &sTrajTransOutputCorridor);
        TPLLCO_TranjParamsCopyValue(sTrajTransOutputCorridor,
                                    &pCompCorridor->fLeCorridorPosX0_met,
                                    &pCompCorridor->fLeCorridorPosY0_met,
                                    &pCompCorridor->fLeCorridorHeadingAgl_rad,
                                    &pCompCorridor->fLeCorridorCurve_1pm,
                                    &pCompCorridor->fLeCorridorCrvChng_1pm2,
                                    &pCompCorridor->fLeCorridorLength_met);

        // set target corridor data
        sInputCorridor.fX0_met = sAllCorridor.fTgtTrajPosX0_met;
        sInputCorridor.fY0_met = sAllCorridor.fTgtTrajPosY0_met;
        sInputCorridor.fHeading_rad = sAllCorridor.fTgtTrajHeadingAng_rad;
        sInputCorridor.fCurve_1pm = sAllCorridor.fTgtTrajCurve_1pm;
        sInputCorridor.fCurveChng_1pm2 = sAllCorridor.fTgtTrajCrvChng_1pm2;
        sInputCorridor.fLength_met = sAllCorridor.fTgtTrajLength_met;
        memset(&sCoordTransOutputCorridor, 0,
               sizeof(TRJPLN_TrajectoryParams_t));
        TPLLCO_CoordinateTranformation(
            sCompenCorriParam.fPosRightPosX_met,
            sCompenCorriParam.fPosRightPosY_met,
            sCompenCorriParam.fRightOrientation_rad, sInputCorridor,
            TPLLCO_COORDTRAFOMINLENGTH_MET, &sCoordTransOutputCorridor);
        TPLLCO_TranjParamsCopyValue(sCoordTransOutputCorridor,
                                    &pCompCorridor->fTargetCorridorPosX0_met,
                                    &pCompCorridor->fTargetCorridorPosY0_met,
                                    &pCompCorridor->fTargetCorridorHeading_rad,
                                    &pCompCorridor->fTargetCorridorCurve_1pm,
                                    &pCompCorridor->fTargetCorridorCrvChng_1pm2,
                                    &pCompCorridor->fTargetCorridorLength_met);
    }

    // set replan deviation data
    const TRJPLN_TrajectoryParams_t sInputRightCorridor = {
        pCompCorridor->fRiCorridorPosX0_met,
        pCompCorridor->fRiCorridorPosY0_met,
        pCompCorridor->fRiCorridorHeadingAgl_rad,
        pCompCorridor->fRiCorridorCurve_1pm,
        pCompCorridor->fRiCorridorCrvChng_1pm2,
        pCompCorridor->fRiCorridorLength_met};
    const TRJPLN_SetDevReplanDeltaIn_t sInputParams = {
        &bTriggerReplan,    &fDevHeadingEgo_rad, &fCtrlPtDev_met[0],
        &fCtrlPtDev_met[1], &fEgoVelX_mps,       &sDevOdoVeh,
        &sCompenCorriParam};
    TPLLCO_SetDevReplanDelta(bTriggerReplanDeviaCal, sInputParams,
                             sInputRightCorridor, uiLatCtrlMode_nu,
                             &pCompCorridor->fDevDistY_met,
                             &pCompCorridor->fDevHeadingAngle_rad,
                             &pCompCorridor->fReplanDevDistY_met,
                             &pCompCorridor->fReplanDevHeading_rad);
}

/*****************************************************************************
  Functionname:TPLLCO_TrajecotryTranslation */ /*!

@brief: calculate the new point status at fInputX_met based on the original
corridor parameters (sInputCorri)

@description: we will calculate the length, curvature, heading angle, curvature
change of the corridor at the new point fInputX_met

@param[in]:
const float32 fInputX_met: new start point DistanceX value
const TRJPLN_TrajectoryParams_t sInputCorri: current input corridor
parameters

@param[out]
TRJPLN_TrajectoryParams_t* pOutputCorri: the calculated new
corridor parameters at the input DistanceX
@return
*****************************************************************************/
void TPLLCO_TrajecotryTranslation(const float32 fInputX_met,
                                  const TRJPLN_TrajectoryParams_t sInputCorri,
                                  TRJPLN_TrajectoryParams_t* pOutputCorri) {
    // calculate the X0�� Y0 and heading angle of the new corridor
    TPLLCO_Corridor_TrajectoryEquation(
        fInputX_met, sInputCorri, &pOutputCorri->fX0_met,
        &pOutputCorri->fY0_met, &pOutputCorri->fHeading_rad);
    // calculate the curvature, curvature change and length of the new corridor
    float32 fDeltaX_met = fInputX_met - sInputCorri.fX0_met;
    pOutputCorri->fCurve_1pm =
        sInputCorri.fCurve_1pm + sInputCorri.fCurveChng_1pm2 * fDeltaX_met;
    pOutputCorri->fCurveChng_1pm2 = sInputCorri.fCurveChng_1pm2;
    pOutputCorri->fLength_met = sInputCorri.fLength_met - fDeltaX_met;
}

/*****************************************************************************
  Functionname:TPLLCO_Corridor_TrajectoryEquation */ /*!

@brief: calculate the Y and heading angle value at the input fX_met point based
on corridor parameters

@description: calculate the Y and heading angle value at the input fX_met point
based on corridor parameters

@param[in]:
const float32 fX_met: the point fX_met value that we need to calculate
based on the corridor parameters
const TRJPLN_TrajectoryParams_t sInputCorri: input corridor parameter
@param[out]
float32* fOutX0_met: the new corridor parameter X0 at fX_met
float32* fOutY0_met: the new corridor parameter Y0 at fX_met
float32* fOutHeading_rad: the new corridor parameter heading angle at
fX_met
@return
*****************************************************************************/
void TPLLCO_Corridor_TrajectoryEquation(
    const float32 fX_met,
    const TRJPLN_TrajectoryParams_t sInputCorri,
    float32* fOutX0_met,
    float32* fOutY0_met,
    float32* fOutHeading_rad) {
    *fOutX0_met = fX_met;
    float32 fTanHead = TAN_HD_(sInputCorri.fHeading_rad);

    float32 fXDelta = fX_met - sInputCorri.fX0_met;
    // Y = Y0+ X*tan(HeadingAgl) + 0.5*X*X*Curve + 1/6 * X*X*X*CurveChange
    *fOutY0_met =
        sInputCorri.fY0_met + fXDelta * fTanHead +
        0.5f * TUE_CML_Sqr(fXDelta) * sInputCorri.fCurve_1pm +
        fXDelta * fXDelta * fXDelta * sInputCorri.fCurveChng_1pm2 / 6.f;
    *fOutHeading_rad =
        ATAN_(sInputCorri.fCurveChng_1pm2 * fXDelta * fXDelta * 0.5f +
              fXDelta * sInputCorri.fCurve_1pm + fTanHead);
}

/*****************************************************************************
  Functionname:TPLLCO_CoordinateTranformation */ /*!

@brief: the input curve's coordinate would be transformed to the new origin
point

@description: we will calculate the position value at half length and full
length of curve, make a
clock rotate for the orientation change, and calculate curvature
relate data based on the calculated
half length and full length points

@param[in]:
const float32 fCridCoordSysPosX_met:the new origin point coordinate X
const float32 fCridCoordSysPosY_met:the new origin point coordinate Y
const float32 fCridCoordSysOrien_rad:the heading angle of right
corridor at new origin point
const TRJPLN_TrajectoryParams_t sInputCorri:the curve which need the
coordinate transformation
const float32 fCorridTrafoLengthMin_met:minimum length of corridor
@param[out]
TRJPLN_TrajectoryParams_t* pOutCorri:coordinate transformed curve
parameters
@uml
@startuml
start
:we will calculate point location on curve at half and full length;
:the new (X,Y) value is calculated based on the new input origin point ;
note:(Xnew,Ynew) = (Xorigin,Yorigin) - (X0, Y)
:we get the 0, half length, full length point at new coordinate;
:renew the heading angle and valid length of curve at new coordinate;
:calculate curvature and curvature change at new coordinate;
end
@enduml
@return
*****************************************************************************/
void TPLLCO_CoordinateTranformation(const float32 fCridCoordSysPosX_met,
                                    const float32 fCridCoordSysPosY_met,
                                    const float32 fCridCoordSysOrien_rad,
                                    const TRJPLN_TrajectoryParams_t sInputCorri,
                                    const float32 fCorridTrafoLengthMin_met,
                                    TRJPLN_TrajectoryParams_t* pOutCorri) {
    float32 fPosX0AtLength_met = 0.f;
    float32 fPosY0AtLength_met = 0.f;
    float32 fPosX0AtHalfLength_met = 0.f;
    float32 fPosY0AtHalfLength_met = 0.f;
    float32 fTempHeading_rad = 0.f;

    TRJPLN_TrajectoryParams_t sLengthChangeTraj = {0.f};
    memcpy(&sLengthChangeTraj, &sInputCorri, sizeof(TRJPLN_TrajectoryParams_t));
    sLengthChangeTraj.fLength_met =
        MAX(sInputCorri.fLength_met, fCorridTrafoLengthMin_met);

    // calculate the middle and end points data at half length and length
    // distance
    TPLLCO_Corridor_TrajectoryEquation(
        sLengthChangeTraj.fLength_met + sLengthChangeTraj.fX0_met,
        sLengthChangeTraj, &fPosX0AtLength_met, &fPosY0AtLength_met,
        &fTempHeading_rad);
    TPLLCO_Corridor_TrajectoryEquation(
        0.5f * sLengthChangeTraj.fLength_met + sLengthChangeTraj.fX0_met,
        sLengthChangeTraj, &fPosX0AtHalfLength_met, &fPosY0AtHalfLength_met,
        &fTempHeading_rad);

    // anti clock rotate
    float32 fRotatePosX0AtLength_met =
        (fPosX0AtLength_met - fCridCoordSysPosX_met) *
            COS_HD_(-fCridCoordSysOrien_rad) -
        (fPosY0AtLength_met - fCridCoordSysPosY_met) *
            SIN_HD_(-fCridCoordSysOrien_rad);
    float32 fRotatePosY0AtLength_met =
        (fPosX0AtLength_met - fCridCoordSysPosX_met) *
            SIN_HD_(-fCridCoordSysOrien_rad) +
        (fPosY0AtLength_met - fCridCoordSysPosY_met) *
            COS_HD_(-fCridCoordSysOrien_rad);

    float32 fRotatePosX0AtHalfLength_met =
        (fPosX0AtHalfLength_met - fCridCoordSysPosX_met) *
            COS_HD_(-fCridCoordSysOrien_rad) -
        (fPosY0AtHalfLength_met - fCridCoordSysPosY_met) *
            SIN_HD_(-fCridCoordSysOrien_rad);
    float32 fRotatePosY0AtHalfLength_met =
        (fPosX0AtHalfLength_met - fCridCoordSysPosX_met) *
            SIN_HD_(-fCridCoordSysOrien_rad) +
        (fPosY0AtHalfLength_met - fCridCoordSysPosY_met) *
            COS_HD_(-fCridCoordSysOrien_rad);

    pOutCorri->fX0_met = (sInputCorri.fX0_met - fCridCoordSysPosX_met) *
                             COS_HD_(-fCridCoordSysOrien_rad) -
                         (sInputCorri.fY0_met - fCridCoordSysPosY_met) *
                             SIN_HD_(-fCridCoordSysOrien_rad);
    pOutCorri->fY0_met = (sInputCorri.fX0_met - fCridCoordSysPosX_met) *
                             SIN_HD_(-fCridCoordSysOrien_rad) +
                         (sInputCorri.fY0_met - fCridCoordSysPosY_met) *
                             COS_HD_(-fCridCoordSysOrien_rad);

    // calculate deviation distance to start point
    float32 fEndPosDevDistX_met = fRotatePosX0AtLength_met - pOutCorri->fX0_met;
    float32 fEndPosDevDistY_met = fRotatePosY0AtLength_met - pOutCorri->fY0_met;

    float32 fMidPosDevDistX_met =
        fRotatePosX0AtHalfLength_met - pOutCorri->fX0_met;
    float32 fMidPosDevDistY_met =
        fRotatePosY0AtHalfLength_met - pOutCorri->fY0_met;

    pOutCorri->fHeading_rad = sInputCorri.fHeading_rad - fCridCoordSysOrien_rad;
    pOutCorri->fLength_met = sInputCorri.fLength_met < fCorridTrafoLengthMin_met
                                 ? sInputCorri.fLength_met
                                 : fEndPosDevDistX_met;

    float32 fTanHead_nu = TAN_HD_(pOutCorri->fHeading_rad);

    // DevFac = (EndX - MidX) * (EndX*MidX)^2 / 12
    float32 fDevFactor_nu =
        SafeDiv((fEndPosDevDistX_met - fMidPosDevDistX_met) *
                TUE_CML_Sqr(fEndPosDevDistX_met * fMidPosDevDistX_met) / 12.f);
    // curve = ((EndX)^3 * (MidY - MidX*Tan) - (MidX)^3 * (EndY - EndX*Tan)) /
    // (6 * DevFac)
    pOutCorri->fCurve_1pm =
        (fEndPosDevDistX_met * fEndPosDevDistX_met * fEndPosDevDistX_met *
             (fMidPosDevDistY_met - fMidPosDevDistX_met * fTanHead_nu) -
         fMidPosDevDistX_met * fMidPosDevDistX_met * fMidPosDevDistX_met *
             (fEndPosDevDistY_met - fEndPosDevDistX_met * fTanHead_nu)) /
        (6.0f * fDevFactor_nu);
    // curve change = ((MidX)^2 * (EndY - EndX*Tan) - (EndX)^2 * (MidY -
    // MidX*Tan)) / (2 * DevFac)
    pOutCorri->fCurveChng_1pm2 =
        (TUE_CML_Sqr(fMidPosDevDistX_met) *
             (fEndPosDevDistY_met - fEndPosDevDistX_met * fTanHead_nu) -
         TUE_CML_Sqr(fEndPosDevDistX_met) *
             (fMidPosDevDistY_met - fMidPosDevDistX_met * fTanHead_nu)) /
        (2.0f * fDevFactor_nu);
}

/*****************************************************************************
  Functionname:TPLLCO_TranjParamsCopyValue */ /*!

@brief: value assignment from input to individual output pointer

@description:value assignment from input to individual output pointer

@param[in]:
RJPLN_TrajectoryParams_t sInputCorri:input curve parameters
structure
@param[out]
float32* fX0_met:output pointer of X0
float32* fY0_met:output pointer of Y0
float32* fHeading_rad:output pointer of heading angle
float32* fCurve_1pm:output pointer of curve
float32* fCurveChng_1pm2:output pointer of curve change
float32* fLength_met:output pointer of valid length
@return
*****************************************************************************/
void TPLLCO_TranjParamsCopyValue(TRJPLN_TrajectoryParams_t sInputCorri,
                                 float32* fX0_met,
                                 float32* fY0_met,
                                 float32* fHeading_rad,
                                 float32* fCurve_1pm,
                                 float32* fCurveChng_1pm2,
                                 float32* fLength_met) {
    *fX0_met = sInputCorri.fX0_met;
    *fY0_met = sInputCorri.fY0_met;
    *fHeading_rad = sInputCorri.fHeading_rad;
    *fCurve_1pm = sInputCorri.fCurve_1pm;
    *fCurveChng_1pm2 = sInputCorri.fCurveChng_1pm2;
    *fLength_met = sInputCorri.fLength_met;
}

/*****************************************************************************
  Functionname:TPLLCO_SetDevReplanDelta */ /*!

@brief: calculate ego movement value at new coordinate, which is right corridor
coordinate system

@description: we will calculate ego vehicle's heading and PosY value at right
corridor coordinate system,
   the new right corridor coordinate system's origin point is the
The vertical point of the orthogonal
   projection from ego car to the right lane corridor

@param[in]:
   boolean bTriggerReplanDeviaCal:whether we will trigger a replan
deviation calculate
   const TRJPLN_SetDevReplanDeltaIn_t sInputParams:ego car state
for coordinate transformed needed
   const TRJPLN_TrajectoryParams_t sRightClothoid:input right lane
corridor parameters
@param[out]
   float32* fDevDistY_met:the vehicle ego lateral distance in
right corridor coordinate system
   float32* fDevHeading_rad:vehicle heading angle in right
corridor coordinate system
   float32* fReplanDistY_met:deviation in lateral position by
replanning
   float32* fReplanDevHeading_rad:heading angle deviation by
replanning

@uml
@startuml
start
if(replan deviation calculate triggered) then (yes)
:calculate ego heading angle at current time;
:calculate ego position at current time;
if(replan trigger is TRUE) then (yes)
:anti clock rotate current ego position based on right lane orientation
;
:renew the LastCyclePosDev data, which would be used in replan trigger
FALSE situation;
else(no)
:renew the LastCyclePosDev data;
endif
:calculate the rectangular projection data of ego car on right lane;
:calculate ego DistY and Heading angle at right lane coordinate system;
if(replan deviation calculate is ON) then (yes)
:calculate replan deviation value;
else(no)
:replan deviation outputs are set to zero;
endif
else(no)
:all the outputs would be set to zero;
endif
end
@enduml
@return
*****************************************************************************/
void TPLLCO_SetDevReplanDelta(boolean bTriggerReplanDeviaCal,
                              const TRJPLN_SetDevReplanDeltaIn_t sInputParams,
                              const TRJPLN_TrajectoryParams_t sRightClothoid,
                              uint8 uiLatCtrlMode_nu,
                              float32* fDevDistY_met,
                              float32* fDevHeading_rad,
                              float32* fReplanDistY_met,
                              float32* fReplanDevHeading_rad) {
    if (bTriggerReplanDeviaCal) {
        // calculate ego vehicle heading angle
        float32 fHeadingVeh_rad = 0.f;
        if (*sInputParams.bTriggerReplan_nu) {
            fHeadingVeh_rad = *sInputParams.fDevHeadingEgo_rad -
                              sInputParams.sCoordParams->fRightOrientation_rad;
        } else {
            fHeadingVeh_rad = TRJPLN_Sf_LastCylceHeadingVeh_rad +
                              sInputParams.sDevOdoVeh->fDevOdoEgoHead_rad;
        }
        TRJPLN_Sf_LastCylceHeadingVeh_rad = fHeadingVeh_rad;

        // claculate ego vehicle position
        float32 fPosVehX_met = TRJPLN_Sf_LastCyclePosDevX_met +
                               COS_HD_(fHeadingVeh_rad) *
                                   sInputParams.sDevOdoVeh->fDevOdoEgoDist_met;
        float32 fPosVehY_met = TRJPLN_Sf_LastCyclePosDevY_met +
                               SIN_HD_(fHeadingVeh_rad) *
                                   sInputParams.sDevOdoVeh->fDevOdoEgoDist_met;

        if (TPLLCO_PREVIEWDISTXENABLE_NU) {
            float32 afTableInputX[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
                TPLLCO_VEHVELX2PREVIEWDISTX_TABLEX_MPS;

            float32 afTableInputYOF[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
                TPLLCO_VEHVELX2PREVIEWDISTXOF_TABLEY_MET;
            memcpy(afTableInputYOF, TP_VEHVELX2PREVIEWDISTXOF_TABLEY_MET,
                   13 * sizeof(real32_T));
            float32 afTableInputY[TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU] =
                TPLLCO_VEHVELX2PREVIEWDISTX_TABLEY_MET;
            float32 fPreviewDistX_met = 0.f;
            if (uiLatCtrlMode_nu == 2U) {
                fPreviewDistX_met = TUE_CML_LookUpTable2D(
                    *sInputParams.fVehVelX_mps, afTableInputX, afTableInputYOF,
                    TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU);
            } else {
                fPreviewDistX_met = TUE_CML_LookUpTable2D(
                    *sInputParams.fVehVelX_mps, afTableInputX, afTableInputY,
                    TPLLCO_VEHVELX2PREVIEWDISTX_TABLENUM_NU);
            }

            fPosVehX_met += fPreviewDistX_met * COS_HD_(fHeadingVeh_rad);
            fPosVehY_met += fPreviewDistX_met * SIN_HD_(fHeadingVeh_rad);
        }

        if (*sInputParams.bTriggerReplan_nu) {
            // anti clock rotate
            float32 fRotatePosX_met =
                (*sInputParams.fCtrlPtDevX_met) *
                    COS_HD_(-sInputParams.sCoordParams->fRightOrientation_rad) -
                (*sInputParams.fCtrlPtDevY_met) *
                    SIN_HD_(-sInputParams.sCoordParams->fRightOrientation_rad);
            float32 fRotatePosY_met =
                (*sInputParams.fCtrlPtDevX_met) *
                    SIN_HD_(-sInputParams.sCoordParams->fRightOrientation_rad) +
                (*sInputParams.fCtrlPtDevY_met) *
                    COS_HD_(-sInputParams.sCoordParams->fRightOrientation_rad);

            TRJPLN_Sf_LastCyclePosDevX_met = -fRotatePosX_met;
            TRJPLN_Sf_LastCyclePosDevY_met =
                -fRotatePosY_met +
                TUE_CML_Sign(sInputParams.sCoordParams->fDeltaProjPosY_met) *
                    TUE_CML_SqrtApprox(
                        TUE_CML_Sqr(
                            sInputParams.sCoordParams->fDeltaProjPosX_met) +
                        TUE_CML_Sqr(
                            sInputParams.sCoordParams->fDeltaProjPosY_met));
        } else {
            TRJPLN_Sf_LastCyclePosDevX_met +=
                COS_HD_(fHeadingVeh_rad) *
                sInputParams.sDevOdoVeh->fDevOdoEgoDist_met;
            TRJPLN_Sf_LastCyclePosDevY_met +=
                SIN_HD_(fHeadingVeh_rad) *
                sInputParams.sDevOdoVeh->fDevOdoEgoDist_met;
        }

        // calcualte ego projection data
        float32 afPosRight_met[2] = {0.f};
        float32 afDeltaProj_met[2] = {0.f};
        float32 fRightOri_rad = 0.f;
        float32 afPosRightLastCycle_met[2] = {0.f};
        for (uint8 i = 0; i < TPLLCO_NUMITER_NU; i++) {
            afPosRight_met[0] = fPosVehX_met - afPosRightLastCycle_met[0];

            float32 fTanHead =
                TAN_HD_(TRJPLN_Ss_LastCycleRightCorr_nu.fHeading_rad);
            float32 fXDelta =
                afPosRight_met[0] - TRJPLN_Ss_LastCycleRightCorr_nu.fX0_met;
            // Y = Y0+ X*tan(HeadingAgl) + 0.5*X*X*Curve + 1/6 *
            // X*X*X*CurveChange
            afPosRight_met[1] =
                TRJPLN_Ss_LastCycleRightCorr_nu.fY0_met + fXDelta * fTanHead +
                0.5f * TUE_CML_Sqr(fXDelta) *
                    TRJPLN_Ss_LastCycleRightCorr_nu.fCurve_1pm +
                fXDelta * fXDelta * fXDelta *
                    TRJPLN_Ss_LastCycleRightCorr_nu.fCurveChng_1pm2 / 6.f;
            fRightOri_rad =
                ATAN_(TRJPLN_Ss_LastCycleRightCorr_nu.fCurveChng_1pm2 *
                          fXDelta * fXDelta * 0.5f +
                      fXDelta * TRJPLN_Ss_LastCycleRightCorr_nu.fCurve_1pm +
                      fTanHead);

            afDeltaProj_met[0] = fPosVehX_met - afPosRight_met[0];
            afDeltaProj_met[1] = fPosVehY_met - afPosRight_met[1];

            float32 fPhi_rad =
                fRightOri_rad -
                ATAN_(-afDeltaProj_met[0] / SafeDiv(afDeltaProj_met[1]));
            afPosRightLastCycle_met[0] =
                afDeltaProj_met[0] * COS_HD_(fPhi_rad) -
                afDeltaProj_met[1] * SIN_HD_(fPhi_rad);
            afPosRightLastCycle_met[1] =
                afDeltaProj_met[0] * SIN_HD_(fPhi_rad) +
                afDeltaProj_met[1] * COS_HD_(fPhi_rad);
        }
        if (*sInputParams.bTriggerReplan_nu) {
            *fDevDistY_met =
                TUE_CML_Sign(sInputParams.sCoordParams->fDeltaProjPosY_met) *
                TUE_CML_SqrtApprox(
                    TUE_CML_Sqr(sInputParams.sCoordParams->fDeltaProjPosX_met) +
                    TUE_CML_Sqr(sInputParams.sCoordParams->fDeltaProjPosY_met));
            *fDevHeading_rad = *sInputParams.fDevHeadingEgo_rad -
                               sInputParams.sCoordParams->fRightOrientation_rad;
        } else {
            *fDevDistY_met =
                TUE_CML_Sign(afDeltaProj_met[1]) *
                TUE_CML_SqrtApprox(TUE_CML_Sqr(afDeltaProj_met[0]) +
                                   TUE_CML_Sqr(afDeltaProj_met[1]));
            *fDevHeading_rad = fHeadingVeh_rad - fRightOri_rad;
        }

        if (TRJPLN_Sb_SetDevLastTrigReplan_nu &&
            *sInputParams.bTriggerReplan_nu && LCF_TPLLCO_REPLANDEVENABLE_NU) {
            if (LCF_TPLLCO_USEODOREPLAN_NU) {
                *fReplanDevHeading_rad =
                    fHeadingVeh_rad - fRightOri_rad - *fDevHeading_rad;
                *fReplanDistY_met =
                    COS_HD_(*fReplanDevHeading_rad) *
                        (TUE_CML_Sign(afDeltaProj_met[1]) *
                         TUE_CML_SqrtApprox(TUE_CML_Sqr(afDeltaProj_met[0]) +
                                            TUE_CML_Sqr(afDeltaProj_met[1]))) -
                    *fDevDistY_met;
            } else {
                *fReplanDevHeading_rad =
                    TRJPLN_Sf_SetDevReplanHeadingInit_rad - *fDevHeading_rad;
                *fReplanDistY_met = COS_HD_(*fReplanDevHeading_rad) *
                                        TRJPLN_Sf_SetDevDistYInit_met -
                                    *fDevDistY_met;
            }
        } else {
            *fReplanDistY_met = 0.f;
            *fReplanDevHeading_rad = 0.f;
        }

        TRJPLN_Sf_SetDevReplanHeadingInit_rad =
            *sInputParams.fDevHeadingEgo_rad -
            sInputParams.sCoordParams->fRightOrientation_rad;
        memcpy(&TRJPLN_Ss_LastCycleRightCorr_nu, &sRightClothoid,
               sizeof(TRJPLN_TrajectoryParams_t));
        TRJPLN_Sf_SetDevDistYInit_met =
            TUE_CML_Sign(sInputParams.sCoordParams->fDeltaProjPosY_met) *
            TUE_CML_SqrtApprox(
                TUE_CML_Sqr(sInputParams.sCoordParams->fDeltaProjPosX_met) +
                TUE_CML_Sqr(sInputParams.sCoordParams->fDeltaProjPosY_met));
        TRJPLN_Sb_SetDevLastTrigReplan_nu = bTriggerReplanDeviaCal;
    } else {
        *fDevDistY_met = 0.f;
        *fDevHeading_rad = 0.f;
        *fReplanDistY_met = 0.f;
        *fReplanDevHeading_rad = 0.f;
        TRJPLN_Sb_SetDevLastTrigReplan_nu = FALSE;
    }
}

/*****************************************************************************
  Functionname:TPLLCO_SetBit                                            */ /*!

              @brief: set bit value based on input bit index and value

              @description:set bit value based on input bit index and value

              @param[in]:
                                    uint8 uiBitIndex: bit index of insert place
                                    boolean bBitSetVal: bit set TRUE or FALSE
              @param[in\out]
                                     uint8* pTargetBit: inserted bit
              @return
            *****************************************************************************/
void TPLLCO_SetBit(uint8 uiBitIndex, boolean bBitSetVal, uint8* pTargetBit) {
    if (bBitSetVal) {
        TUE_CML_Setbit_M(*pTargetBit, uiBitIndex);
    } else {
        TUE_CML_Clrbit_M(*pTargetBit, uiBitIndex);
    }
}

/*****************************************************************************
  Functionname:DistYDeviationPlausibilityCheck */ /*!

@brief: we will calculate DistY deviation between original corridor and latency
compensated corridor

@description: we select 4 points on the corridor based on the corridor length,
calculation every
DistY deviation at every selected point. the DistY deviation check
would fail if the
DistY deviation more than threshold

@param[in]:
const TRJPLN_TrajectoryParams_t sOriginCorridor: original corridor
parameter without latency compensation
const TRJPLN_TrajectoryParams_t sLatenCompenCorridor: latency
compensated corridor parameter
const float32 fRightOrientation_rad: right orientation of the right
corridor coordinate system
const float32 fPosRightX_met: origin point X of the right corridor
coordinate system
const float32 fPosRightY_met: origin point Y of the right corridor
coordinate system
const uint8 uiNumForIterator: the size of iterator loop
@param[out]
float32 fDeviationDistY_met[4]: DistY deviation at every sample point
boolean* bOutOfRange: is DistY deviation more than threshold
@return

@uml
@startuml
start
:uiIterator=0;
while (uiIterator < 4)
:original X = original corridor length * uiIterator / 4;
:distance Y calculate in original corridor;
:anti clock rotation of the original X and Y;
:distance Y calculate in compensated corridor;
:PosY deviation threshold calculation by the lookup result of distance;
:output DistY deviation array result;
:uiIterator++;
endwhile
:out of range boolean output;
end
@enduml
*****************************************************************************/
void DistYDeviationPlausibilityCheck(
    const TRJPLN_TrajectoryParams_t sOriginCorridor,
    const TRJPLN_TrajectoryParams_t sLatenCompenCorridor,
    const float32 fRightOrientation_rad,
    const float32 fPosRightX_met,
    const float32 fPosRightY_met,
    const uint8 uiNumForIterator,
    float32 fDeviationDistY_met[4],
    boolean* bOutOfRange) {
    boolean bTempOutRangeCheck = FALSE;
    for (uint8 uiIterator = 0u; uiIterator < MIN(uiNumForIterator, 4u);
         uiIterator++) {
        // distance Y calculate in original corridor
        float32 fOriginPosX_met = sOriginCorridor.fLength_met *
                                  (float32)(uiIterator + 1) / uiNumForIterator;
        float32 fOriginPosY_met = TUE_CML_CalcDistYOfClothoidsCurve(
            sOriginCorridor.fHeading_rad, sOriginCorridor.fCurve_1pm,
            sOriginCorridor.fCurveChng_1pm2,
            fOriginPosX_met - sOriginCorridor.fX0_met, sOriginCorridor.fY0_met);

        // anti clock rotation
        float32 fRotateOriginPosY_met =
            (fOriginPosX_met - fPosRightX_met) *
                SIN_HD_(-fRightOrientation_rad) +
            (fOriginPosY_met - fPosRightY_met) * COS_HD_(fRightOrientation_rad);
        float32 fRotateOriginPosX_met =
            (fOriginPosX_met - fPosRightX_met) *
                COS_HD_(fRightOrientation_rad) +
            (fOriginPosY_met - fPosRightY_met) * SIN_HD_(fRightOrientation_rad);

        // distance Y calculate in compensated corridor
        float32 fLatenCompPosY_met = TUE_CML_CalcDistYOfClothoidsCurve(
            sLatenCompenCorridor.fHeading_rad, sLatenCompenCorridor.fCurve_1pm,
            sLatenCompenCorridor.fCurveChng_1pm2,
            fRotateOriginPosX_met - sLatenCompenCorridor.fX0_met,
            sLatenCompenCorridor.fY0_met);

        // PosY deviation threshold calculation by the lookup result of distance
        float32 afTableInputX[TPLLCO_ALLOWEDDEVPOSY_TABLENUM_NU] =
            TPLLCO_TESTPOINTPOSX_TABLEX_MET;
        float32 afTableInputY[TPLLCO_ALLOWEDDEVPOSY_TABLENUM_NU] =
            TPLLCO_ALLOWEDDEVPOSY_TABLEY_MET;
        float32 fPosYDeviationLimit_met =
            TUE_CML_LookUpTable2D(fOriginPosX_met, afTableInputX, afTableInputY,
                                  TPLLCO_ALLOWEDDEVPOSY_TABLENUM_NU);

        // output result
        fDeviationDistY_met[uiIterator] =
            fABS(fRotateOriginPosY_met - fLatenCompPosY_met);
        bTempOutRangeCheck =
            bTempOutRangeCheck ||
            (fDeviationDistY_met[uiIterator] > fPosYDeviationLimit_met);
    }
    *bOutOfRange = bTempOutRangeCheck;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */