/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trjpln_frenet_transform.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static float32 TRJPLN_Sf_DevDistY_met = 0.f;
static float32 TRJPLN_Sf_DevHeading_rad = 0.f;
static float32 TRJPLN_Sa_TgtDistY_met[TARGET_POINTS_ARRAY_SIZE] = {0.f};
static float32 TRJPLN_Sa_TgtDistY1stDeriv_nu[TARGET_POINTS_ARRAY_SIZE] = {0.f};
static float32 TRJPLN_Sa_TgtDistY2ndDeriv_nu[TARGET_POINTS_ARRAY_SIZE] = {0.f};
static uint8 TRJPLN_Sc_NumTgtPoints_nu = 0u;
static float32 TRJPLN_Sa_TgtPoints_nu[TARGET_POINTS_ARRAY_SIZE] = {0};
static float32 TRJPLN_Sf_TrajPlanningHorizon_sec = 0.f;
static float32 TRJPLN_Sf_PlanHorizonVisRange_sec = 0.f;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:LCF_TRJPLN_FrenetTransform_Exec */ /*!

@brief: frenet transformation process

@description: frenet transformation process.
1. Transformation of the current values in Frenet coordinate system
2. Calculation of the predicted current values in Frenet coordinate
system
3. Transformation of the target curve in Frenet coordinate system
4. Transformation of the left corridor boundary  in Frenet coordinate
system

@param[in]
const TRJPLN_FrenetTransformInReq_t* reqPorts: frenet transform module
input
const TRJPLN_TrajectoryPlanParam_t* paras: system parameters
@param[out]
TRJPLN_FrenetTransformOutPro_t* proPorts: frenet transform result

@return
@uml
@startuml
start
:determine the start values of the trajectory;
note:The start values of the trajectory correspond to the values\n of the
trajectory of the previous time step while \nconsidering steps and rotation in
the right corridor boundary.
:TRJPLN_calFrenetTransformation;
note:frenet transformation core function
:output ego car related data in frenet coordinate system;
if(replan happend) then (yes)
:output new transformed target trajectory;
else(no)
:output history transformed target trajectory;
endif
:output other related data;
end
@enduml
*****************************************************************************/
void LCF_TRJPLN_FrenetTransform_Exec(
    const TRJPLN_FrenetTransformInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_FrenetTransformOutPro_t* proPorts,
    TRJPLN_FrenetTransfDebug_t* debug) {
    float32 fDistYTrajPrev_met = 0.f;
    float32 fHeadingTrajPrev_rad = 0.f;
    float32 fCurHdngAndDeltaHdng_rad = 0.f;
    float32 fStartCrv_1pm = TPLFRT_USEDMCSUMCRV_NU
                                ? reqPorts->fKappaSumCommand_1pm
                                : reqPorts->fEgoCurve_1pm;

    // determine the start values of the trajectory
    // The start values of the trajectory correspond to the values of the
    // trajectory of the previous time step while
    // considering steps and rotation in the right corridor boundary.
    float32 fTempDistYTrajPrev1_met = reqPorts->fTrajDistYPrev_met -
                                      TRJPLN_Sf_DevDistY_met +
                                      reqPorts->fDevDistY_met;
    float32 fTempHeadingTrajPrev1_rad = reqPorts->fTrajHeadingPrev_rad -
                                        TRJPLN_Sf_DevHeading_rad +
                                        reqPorts->fDevHeadingAngle_rad;

    float32 fTempDistYTrajPrev2_met =
        reqPorts->fTrajDistYPrev_met *
            COS_HD_(reqPorts->fReplanDevHeading_rad) -
        reqPorts->fReplanDevDistY_met;
    float32 fTempHeadingTrajPrev2_rad =
        reqPorts->fTrajHeadingPrev_rad - reqPorts->fReplanDevHeading_rad;

    TRJPLN_Sf_DevDistY_met = reqPorts->fDevDistY_met;
    TRJPLN_Sf_DevHeading_rad = reqPorts->fDevHeadingAngle_rad;
    if (TPLFRT_CRIDRSWITCHDETECTION_NU) {
        fDistYTrajPrev_met = fTempDistYTrajPrev1_met;
        fHeadingTrajPrev_rad = fTempHeadingTrajPrev1_rad;
    } else {
        fDistYTrajPrev_met = fTempDistYTrajPrev2_met;
        fHeadingTrajPrev_rad = fTempHeadingTrajPrev2_rad;
    }
    fCurHdngAndDeltaHdng_rad = fTempHeadingTrajPrev1_rad;

    // frenet transformation process input
    TRJPLN_calFTInTypeV3_t sFrenetTransformInput = {
        reqPorts->fDevDistY_met,                // fDevDistY_met
        reqPorts->fDevHeadingAngle_rad,         // fDevHeading_rad
        reqPorts->fRiCorridorCurve_1pm,         // fCridrRightSg1_Crv_1pm
        reqPorts->fRiCorridorCrvChng_1pm2,      // fCridrRightSg1_ChOfCrv_1pm2
        reqPorts->fRiCorridorLength_met,        // fCridrRightSg1_Length_met
        reqPorts->fLeCorridorPosY0_met,         // fCridrLeftSg1_DistY_met
        reqPorts->fLeCorridorHeadingAgl_rad,    // fCridrLeftSg1_Heading_rad
        reqPorts->fLeCorridorCurve_1pm,         // fCridrLeftSg1_Crv_1pm
        reqPorts->fLeCorridorCrvChng_1pm2,      // fCridrLeftSg1_ChOfCrv_1pm2
        reqPorts->fLeCorridorLength_met,        // fCridrLeftSg1_Length_met
        reqPorts->fTargetCorridorPosY0_met,     // fTgtTrajSg1_DistY_met
        reqPorts->fTargetCorridorHeading_rad,   // fTgtTrajSg1_Heading_rad
        reqPorts->fTargetCorridorCurve_1pm,     // fTgtTrajSg1_Crv_1pm
        reqPorts->fTargetCorridorCrvChng_1pm2,  // fTgtTrajSg1_ChngOfCrv_1pm2
        reqPorts->fTargetCorridorLength_met,    // fTgtTrajSg1_Length_met
        reqPorts->bTrajPlanEnble,               // uiTrajPlanEnbl_nu
        reqPorts->bTrigTrajReplan,              // uiTrigRecalc_nu
        reqPorts->fPlanningHorizon_sec,         // fPlanningHorizon_sec
        reqPorts->fEgoCurve_1pm,                // fCurCrvVehMotion_1pm
        fStartCrv_1pm,                          // fStartCrv_1pm
        reqPorts->fEgoVelX_mps,                 // fEgoVelX_mps
        reqPorts->fEgoAccelX_mps2,              // fEgoAclX_mps2
        reqPorts->fTrajTgtCrvPrev_1pm,          // fTgtCrvTrajPrev_1pm
        reqPorts->fTargetCorridorPosX0_met,     // fTgtTrajSg1_DistX_met
        reqPorts->bReplanModeArcLength,         // uiReplanModeArcLength_nu
        TPLFRT_CALCLEFTCRIDRENABLE_NU,          // uiLeftCridrActive_nu
        fDistYTrajPrev_met,                     // fDistYTrajPrev_met
        fHeadingTrajPrev_rad,                   // fHeadingTrajPrev_rad
        fCurHdngAndDeltaHdng_rad,               // fCurHdngAndDeltaHdng_rad
        reqPorts->fPredictionTimeHead_sec,      // fPreviewTimeHeading_sec
        reqPorts->fDelayVehGui_sec,             // fDeadTime_sec
        reqPorts->bTrigReplanTgtTraj,           // uiTrigTgtTrajCalc_nu
        reqPorts->fRiCorridorPosX0_met,         // fCridrRightSg1_DistX_met
        reqPorts->fLeCorridorPosX0_met,         // fCridrLeftSg1_DistX_met
        TPLFRT_USETGTRAJX0_NU,                  // uiUseTgtTrajX0_nu
        FALSE,                                  // uiUseCridrRightX0_nu
        reqPorts->fRiCorridorHeadingAgl_rad,    // fCridrRightSg1_Heading_rad
        reqPorts->fRiCorridorPosY0_met,         // fCridrRightSg1_DistY_met
        reqPorts->bReplanCurValues              // uiReplanCurValues_nu
    };

    // frenet transformation core function
    TRJPLN_calFTOutType_t sFrenetTransformOutnput =
        TRJPLN_calFrenetTransformation(&sFrenetTransformInput);

    // output ego car related data in frenet coordinate system
    proPorts->fCurDistY_met = sFrenetTransformOutnput.fDistYCurValue_met;
    proPorts->fCurDistY1stDeriv_mps =
        sFrenetTransformOutnput.fDistY1stDerivCurValue_nu;
    proPorts->fCurDistY2ndDeriv_mps2 =
        sFrenetTransformOutnput.fDistY2ndDerivCurValue_nu;
    proPorts->fTrajVelRefCurve_mps = sFrenetTransformOutnput.fTrajVel_mps;
    proPorts->fTrajAclRefCurve_mps2 = sFrenetTransformOutnput.fTrajAcl_mps2;

    // output new calculated target corridor sample points data if replan is
    // riggered
    if (reqPorts->bTrigReplanTgtTraj) {
        memcpy(proPorts->afTargetDistY_met,
               &sFrenetTransformOutnput.afTgtDistY_met,
               sizeof(sFrenetTransformOutnput.afTgtDistY_met));
        memcpy(proPorts->fTargetDistY1stDeriv_mps,
               &sFrenetTransformOutnput.afTgtDistY1stDeriv_nu,
               sizeof(sFrenetTransformOutnput.afTgtDistY1stDeriv_nu));
        memcpy(proPorts->fTargetDistY2ndDeriv_mps2,
               &sFrenetTransformOutnput.afTgtDistY2ndDeriv_nu,
               sizeof(sFrenetTransformOutnput.afTgtDistY2ndDeriv_nu));

        memcpy(&TRJPLN_Sa_TgtPoints_nu, &sFrenetTransformOutnput.afTgtPoints_nu,
               sizeof(sFrenetTransformOutnput.afTgtPoints_nu));
        memcpy(proPorts->afTargetPoints_nu,
               &sFrenetTransformOutnput.afTgtPoints_nu,
               sizeof(sFrenetTransformOutnput.afTgtPoints_nu));
        if (TUE_CML_IsZero(sFrenetTransformOutnput.afTgtPoints_nu[0])) {
            proPorts->afTargetPoints_nu[0] = TPLFRT_TGTPOINTMIN_SEC;
        }

        proPorts->uiNumOfTgtPoints_nu =
            sFrenetTransformOutnput.uiNumTgtPoints_nu;
        proPorts->fTrajPlanningHorizon_sec =
            sFrenetTransformOutnput.fTrajPlanningHorizon_sec;
        proPorts->fPlanHorizonVisRange_sec =
            sFrenetTransformOutnput.fPlanHorizonVisRange_sec;
    } else {
        memcpy(proPorts->afTargetDistY_met, &TRJPLN_Sa_TgtDistY_met,
               sizeof(TRJPLN_Sa_TgtDistY_met));
        memcpy(proPorts->fTargetDistY1stDeriv_mps,
               &TRJPLN_Sa_TgtDistY1stDeriv_nu,
               sizeof(TRJPLN_Sa_TgtDistY1stDeriv_nu));
        memcpy(proPorts->fTargetDistY2ndDeriv_mps2,
               &TRJPLN_Sa_TgtDistY2ndDeriv_nu,
               sizeof(TRJPLN_Sa_TgtDistY2ndDeriv_nu));

        memcpy(proPorts->afTargetPoints_nu, &TRJPLN_Sa_TgtPoints_nu,
               sizeof(TRJPLN_Sa_TgtPoints_nu));
        if (TUE_CML_IsZero(TRJPLN_Sa_TgtPoints_nu[0])) {
            proPorts->afTargetPoints_nu[0] = TPLFRT_TGTPOINTMIN_SEC;
        }

        proPorts->uiNumOfTgtPoints_nu = TRJPLN_Sc_NumTgtPoints_nu;
        proPorts->fTrajPlanningHorizon_sec = TRJPLN_Sf_TrajPlanningHorizon_sec;
        proPorts->fPlanHorizonVisRange_sec = TRJPLN_Sf_PlanHorizonVisRange_sec;
    }
    memcpy(&TRJPLN_Sa_TgtDistY_met, proPorts->afTargetDistY_met,
           sizeof(proPorts->afTargetDistY_met));
    memcpy(&TRJPLN_Sa_TgtDistY1stDeriv_nu, proPorts->fTargetDistY1stDeriv_mps,
           sizeof(proPorts->fTargetDistY1stDeriv_mps));
    memcpy(&TRJPLN_Sa_TgtDistY2ndDeriv_nu, proPorts->fTargetDistY2ndDeriv_mps2,
           sizeof(proPorts->fTargetDistY2ndDeriv_mps2));

    TRJPLN_Sc_NumTgtPoints_nu = proPorts->uiNumOfTgtPoints_nu;
    TRJPLN_Sf_TrajPlanningHorizon_sec = proPorts->fTrajPlanningHorizon_sec;
    TRJPLN_Sf_PlanHorizonVisRange_sec = proPorts->fPlanHorizonVisRange_sec;

    memcpy(proPorts->afLeDistY_met, &sFrenetTransformOutnput.afDistYLeft_met,
           sizeof(sFrenetTransformOutnput.afDistYLeft_met));
    proPorts->fTrajDistYPrev_met =
        sFrenetTransformOutnput.fTrajDistYPrevious_met;
    proPorts->fTrajDistY1stToPrev_mps =
        sFrenetTransformOutnput.fTrajDistYPrev1stDeriv_nu;
    proPorts->fTrajDistY2ndToPrev_mps2 =
        sFrenetTransformOutnput.fTrajDistYPrev2ndDeriv_nu;
    proPorts->fDistY1stToDevHead_mps =
        sFrenetTransformOutnput.f1stDerivDevHeading_nu;
    proPorts->fDistY2ndToDevHead_mps2 =
        sFrenetTransformOutnput.f2ndDerivDevHeading_nu;
    proPorts->fCurDistYPreview_met =
        sFrenetTransformOutnput.fDstYCurValuePreview_met;
    proPorts->fCurDistY1stToPrev_mps =
        sFrenetTransformOutnput.fDstY1stDrvCurValPrvw_mps;
    proPorts->fPreviewTimeHeading_sec =
        sFrenetTransformOutnput.fFrenTrafHeadingPrvw_sec;
    proPorts->uiNumOfPointsCridrLeft_nu =
        sFrenetTransformOutnput.uiNumPointsCridrLeft_nu;

    debug->fCurDistY_met = proPorts->fCurDistY_met;
    debug->fCurDistY1stDeriv_mps = proPorts->fCurDistY1stDeriv_mps;
    debug->fCurDistY2ndDeriv_mps2 = proPorts->fCurDistY2ndDeriv_mps2;
    debug->fTrajVelRefCurve_mps = proPorts->fTrajVelRefCurve_mps;
    debug->fTrajAclRefCurve_mps2 = proPorts->fTrajAclRefCurve_mps2;
    for (int j = 0; j < 15; j++) {
        debug->afTargetDistY_met[j] = proPorts->afTargetDistY_met[j];
        debug->fTargetDistY1stDeriv_mps[j] =
            proPorts->fTargetDistY1stDeriv_mps[j];
        debug->fTargetDistY2ndDeriv_mps2[j] =
            proPorts->fTargetDistY2ndDeriv_mps2[j];
        debug->afTargetPoints_nu[j] = proPorts->afTargetPoints_nu[j];
    }
    for (int j = 0; j < 100; j++) {
        debug->afLeDistY_met[j] = proPorts->afLeDistY_met[j];
    }
    debug->fTrajDistYPrev_met = proPorts->fTrajDistYPrev_met;
    debug->fTrajDistY1stToPrev_mps = proPorts->fTrajDistY1stToPrev_mps;
    debug->fTrajDistY2ndToPrev_mps2 = proPorts->fTrajDistY2ndToPrev_mps2;
    debug->uiNumOfTgtPoints_nu = proPorts->uiNumOfTgtPoints_nu;
    debug->fTrajPlanningHorizon_sec = proPorts->fTrajPlanningHorizon_sec;
    debug->fDistY1stToDevHead_mps = proPorts->fDistY1stToDevHead_mps;
    debug->fDistY2ndToDevHead_mps2 = proPorts->fDistY2ndToDevHead_mps2;
    debug->fCurDistYPreview_met = proPorts->fCurDistYPreview_met;
    debug->fCurDistY1stToPrev_mps = proPorts->fCurDistY1stToPrev_mps;
    debug->fPreviewTimeHeading_sec = proPorts->fPreviewTimeHeading_sec;
    debug->fPlanHorizonVisRange_sec = proPorts->fPlanHorizonVisRange_sec;
    debug->uiNumOfPointsCridrLeft_nu = proPorts->uiNumOfPointsCridrLeft_nu;
}

/*****************************************************************************
  Functionname: LCF_TRJPLN_FrenetTransform_Reset */ /*!

@brief: function reset for frenet transformation module

@description:function reset for frenet transformation module

@param[in]:void
@param[out]:void

@return
*****************************************************************************/
void LCF_TRJPLN_FrenetTransform_Reset(void) {
    TRJPLN_Sf_DevDistY_met = 0.f;
    TRJPLN_Sf_DevHeading_rad = 0.f;

    memset(TRJPLN_Sa_TgtDistY_met, 0, sizeof(TRJPLN_Sa_TgtDistY_met));
    memset(TRJPLN_Sa_TgtDistY1stDeriv_nu, 0,
           sizeof(TRJPLN_Sa_TgtDistY1stDeriv_nu));
    memset(TRJPLN_Sa_TgtDistY2ndDeriv_nu, 0,
           sizeof(TRJPLN_Sa_TgtDistY2ndDeriv_nu));
    TRJPLN_Sc_NumTgtPoints_nu = 0u;
    memset(TRJPLN_Sa_TgtPoints_nu, 0, sizeof(TRJPLN_Sa_TgtPoints_nu));
    TRJPLN_Sf_TrajPlanningHorizon_sec = 0.f;
    TRJPLN_Sf_PlanHorizonVisRange_sec = 0.f;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
