/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp.h"
#include "cp_kalman.h"
#include "cp_par.h"
#include "fip_ext.h"
#include "stddef.h"
#include "TM_Global_Types.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

#ifndef CP_MEAS_ID_TRACE_TRAJECTORY
#define CP_MEAS_ID_TRACE_TRAJECTORY 539369472u
#endif
/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Trace Trajectory information @vaddr: CP_MEAS_ID_TRACE_TRAJECTORY
 * @cycleid:VLC_ENV */
CPTraceTrajectory_t CPTraceTrajectory;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static const MEASInfo_t CPTraceTrajectoryMeasInfo = {

    CP_MEAS_ID_TRACE_TRAJECTORY, /* .VirtualAddress */
    sizeof(CPTraceTrajectory),   /* .Length */
    VLC_MEAS_FUNC_ID,            /* .FuncID */
    VLC_MEAS_FUNC_CHAN_ID        /* .FuncChannelID */
};

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/
/*! @brief RE_FUS_SMOOTHING_MAX */
#define RE_FUS_SMOOTHING_MAX (0.45f)
/*! @brief RE_FUS_SMOOTHING_INC */
#define RE_FUS_SMOOTHING_INC (0.03f)
/*! @brief CP_TRACEWEIGHT_CURVE_THRESH */
#define CP_TRACEWEIGHT_CURVE_THRESH (0.0001F)
/*! @brief NUM_TRAJ_SAMPLES_AT_START */
#define NUM_TRAJ_SAMPLES_AT_START (6u)

#define CP_MAX_ANGLE_NEARRANGE_FOV (25.f)
#define CP_MAX_DIST_NEAR_RANGE (70.f)
#define CP_MIN_NUM_SAMPLES_VDY (5)
/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
boolean SITrajectoryData_EgoCourseOnly;
boolean SITrajectoryData_FusionTraces;
boolean SITrajectoryData_CamLaneFusion;
boolean pTrajData_EgoCourseOnly;
boolean pTrajData_FusionTraces;
boolean pTrajData_CamLaneFusion;
float32 pTrajData_fTrajC0;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/*! Structure describing a moving object's parallelism to our course */
typedef struct {
    boolean relevantTrace;
    float32 fEGODistMean;
    float32 fEGODistStdDev;
    float32 fREDistMean;
    float32 fREDistStdDev;
} CPRoadAndEgo2MOTraceParallelism_t;

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! @brief CP_INIT_TRAJ_NOISE_C0 */
#define CP_INIT_TRAJ_NOISE_C0 1.0e-10f
/*! @brief CP_INIT_TRAJ_NOISE_C1 */
#define CP_INIT_TRAJ_NOISE_C1 1.0e-11f

/*! first ramp for vehicle velocity / RE distance criterion */

SET_MEMSEC_CONST(CPREPlausibilityHighway)
static const GDBLFunction_t CPREPlausibilityHighway = {

    PLAUSIBILITY_HIGHWAY_LOWSPEED_A,  /* Ausgangswert A1 */
    PLAUSIBILITY_HIGHWAY_HIGHSPEED_A, /* Ausgangswert A2 */
    /* Steigung der Anpassungsgerade:        (A2-A1)/(E2-E1) */
    (PLAUSIBILITY_HIGHWAY_HIGHSPEED_A - PLAUSIBILITY_HIGHWAY_LOWSPEED_A) /
        (PLAUSIBILITY_HIGHWAY_HIGHSPEED - PLAUSIBILITY_HIGHWAY_LOWSPEED),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    PLAUSIBILITY_HIGHWAY_LOWSPEED_A -
        (((PLAUSIBILITY_HIGHWAY_HIGHSPEED_A - PLAUSIBILITY_HIGHWAY_LOWSPEED_A) /
          (PLAUSIBILITY_HIGHWAY_HIGHSPEED - PLAUSIBILITY_HIGHWAY_LOWSPEED)) *
         PLAUSIBILITY_HIGHWAY_LOWSPEED)};

/*! CPCourseGradUpdateSTD */
SET_MEMSEC_CONST(CPCourseGradUpdateSTD)
static const GDBLFunction_t CPCourseGradUpdateSTD = {

    COURSE_GRADUPDATE_STD_LOWSPEED_A,  /* Ausgangswert A1 */
    COURSE_GRADUPDATE_STD_HIGHSPEED_A, /* Ausgangswert A2 */
    /* Steigung der Anpassungsgerade:        (A2-A1)/(E2-E1) */
    (COURSE_GRADUPDATE_STD_HIGHSPEED_A - COURSE_GRADUPDATE_STD_LOWSPEED_A) /
        (COURSE_GRADUPDATE_STD_HIGHSPEED - COURSE_GRADUPDATE_STD_LOWSPEED),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    COURSE_GRADUPDATE_STD_LOWSPEED_A -
        (((COURSE_GRADUPDATE_STD_HIGHSPEED_A -
           COURSE_GRADUPDATE_STD_LOWSPEED_A) /
          (COURSE_GRADUPDATE_STD_HIGHSPEED - COURSE_GRADUPDATE_STD_LOWSPEED)) *
         COURSE_GRADUPDATE_STD_LOWSPEED)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/* function prototype declarations */
static void CPTrajectoryReset(const CPCourseData_t *pCourseData,
                              CPTrajectoryData_t *pTrajData);
static void CPTrajKalmanSetInitValues(const CPCourseData_t *pCourseData,
                                      const CPTrajectoryData_t *pTrajData);

static void CPApproximateRefpoint(const float32 fX,
                                  const float32 fY,
                                  const float32 fC0,
                                  const float32 fC1,
                                  float32 *pfRefX);
static Vector2_f32_t CPClothApproxByPolynom(const float32 fC0,
                                            const float32 fC1,
                                            const float32 fYawAngle,
                                            const float32 fDistOnCloth);
static Vector2_f32_t CPClothApproxByCircle(const float32 fC0,
                                           const float32 fDistOnCloth);
static void CPFusePosSamples(CPPosSamples_t *const Samples,
                             const CPPosSamples_t *const MeasSamples);
static void CPCalculateVDYSamples(const CPCourseData_t *pCourseData,
                                  CPPosSamples_t *pEGO_PosSamples);

/*************************************************************************************************************************
  Functionname:    CPInitPosSamples */
void CPInitPosSamples(CPPosSamples_t *pSamples) {
    uint32 i;

    for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
        pSamples->fx[i] = 0.0F;
        pSamples->fy[i] = 0.0F;
        pSamples->fVar[i] = 0.0F;
    }
    pSamples->nb_samples = 0u;
}

/*************************************************************************************************************************
  Functionname:    CPInitGradSamples */
void CPInitGradSamples(CPGradSamples_t *pSamples) {
    uint32 i;

    for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
        pSamples->fx[i] = 0.0F;
        pSamples->fdydx[i] = 0.0F;
        pSamples->fdydxMinStdDev[i] = 0.0F;
        pSamples->valid[i] = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    CPTrajectoryInit */
void CPTrajectoryInit(const CPCourseData_t *pCourseData,
                      boolean bUseRoadEstim,
                      boolean bUseObjTraces,
                      boolean bUseCamLaneMarker,
                      boolean bUseNaviPath,
                      CPTrajectoryData_t *pTrajData) {
    /* Set fusion configuration of Trajectory*/

    /*ROAD_ESTIMATION can be turned of*/
    if (bUseRoadEstim != FALSE) {
        pTrajData->Config.UseRoadEstim = TRUE;
    } else {
        pTrajData->Config.UseRoadEstim = FALSE;
    }

    /*OBJECT_TRACES can be turned of*/
    if (bUseObjTraces != FALSE) {
        pTrajData->Config.UseObjTraces = TRUE;
    } else {
        pTrajData->Config.UseObjTraces = FALSE;
    }

    pTrajData->Config.UseCamLane = bUseCamLaneMarker;

    /* reset State */
    /*EGO_COURSE is mandatory
      if only ego course is used fusion is bypassed
      and distance is calculated in a different manner */
    if ((bUseRoadEstim == FALSE) && (bUseObjTraces == FALSE) &&
        (bUseNaviPath == FALSE)) {
        pTrajData->State.EgoCourseOnly = TRUE;
    } else {
        pTrajData->State.EgoCourseOnly = FALSE;
    }

    /* Set the number of states */
    pTrajData->KafiEst.uiNbOfStates = NB_TRAJ_STATE;

    /* Kalman Matrices */
    GDBKalmanCreateMat(&pTrajData->KafiEst.AMat, pTrajData->KafiData.AMatrix,
                       MATTYPE_FULL, NB_TRAJ_STATE, NB_TRAJ_STATE);
    GDBKalmanCreateMat(&pTrajData->KafiEst.QMat, pTrajData->KafiData.QMatrix,
                       MATTYPE_DIAGONAL, NB_TRAJ_STATE, NB_TRAJ_STATE);
    GDBKalmanCreateMat(&pTrajData->KafiEst.XVec, pTrajData->KafiData.XVector,
                       MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);

    /* Internal Matrixes */
    GDBKalmanCreateMat(&pTrajData->KafiEst.UDMat, pTrajData->KafiData.UDMatrix,
                       MATTYPE_UPPERTRIANGULAR, NB_TRAJ_STATE, NB_TRAJ_STATE);
    GDBKalmanCreateMat(&pTrajData->KafiEst.XsVec, pTrajData->KafiData.XsVector,
                       MATTYPE_VECTOR, NB_TRAJ_STATE, (uint8)1);
    GDBKalmanCreateMat(&pTrajData->KafiEst.PDiagMat,
                       pTrajData->KafiData.PDiagMatrix, MATTYPE_DIAGONAL,
                       NB_TRAJ_STATE, NB_TRAJ_STATE);

    /* Reset with Course C0 and TrajC1 with 0*/
    CPTrajectoryReset(pCourseData, pTrajData);

    _PARAM_UNUSED(bUseNaviPath);
}

/*************************************************************************************************************************
  Functionname:    CPTrajectoryReset */
static void CPTrajectoryReset(const CPCourseData_t *pCourseData,
                              CPTrajectoryData_t *pTrajData) {
    /* Init TrajAngle with Course Slip Angle */
    pTrajData->Current.fTrajAngle = pCourseData->SideSlipAngle;
    /* Init Old State with actual VDY State */
    pTrajData->Current.fTrajC0 = pCourseData->fCurve;
    pTrajData->Current.fTrajC1 = 0.0F;
    pTrajData->LastCycle.fTrajC0 = pCourseData->fCurve;
    pTrajData->LastCycle.fTrajC1 = 0.0F;
    pTrajData->LastCycle.fTrajAngle = pCourseData->SideSlipAngle;
    /* Reset state bits */
    pTrajData->State.EgoCourseOnly = TRUE;
    pTrajData->State.FusionTraces = FALSE;
    pTrajData->State.FusionRoadstimation = FALSE;
    pTrajData->State.FusionHystLRoadRangeConf = FALSE;
    pTrajData->State.FusionHystRRoadRangeConf = FALSE;
    pTrajData->State.FusionTraceHystEgoSpeed = FALSE;
    pTrajData->State.FusionHystEgoSpeed = FALSE;
    pTrajData->State.FusionHystEgoRadius = FALSE;
    pTrajData->State.FusTraceCrit = FALSE;
    pTrajData->State.FusionPlaus = FALSE;
    pTrajData->State.CamLaneQualityHigh = FALSE;
    pTrajData->State.CamLaneFusion = FALSE;
    pTrajData->State.NaviPathFusion = FALSE;

    /* Init UnplausibleSCurve Dist with 0 */
    pTrajData->fDistXUnplausibleSCurve = 0.0F;
    pTrajData->fRoadEstFusRange = 0.0F;
    pTrajData->fMovObjFusRange = 0.0F;
    pTrajData->fMSETrace = 0.0F;
    pTrajData->bTraceFusionActiveLastCycle = FALSE;

    pTrajData->fCamLaneFusionTimer = 0.0F;
    pTrajData->fEgo2CamDistX = 0.0F;
    pTrajData->fBaseY = 0.0F;
    pTrajData->fFilteredDIC = 0.0F;

    /* Init TrajC0 with Course C0 and TrajC1 with 0*/
    CPTrajKalmanSetInitValues(pCourseData, pTrajData);
}
/*************************************************************************************************************************
  Functionname:    CPTrajKalmanSetInitValues */
static void CPTrajKalmanSetInitValues(const CPCourseData_t *pCourseData,
                                      const CPTrajectoryData_t *pTrajData) {
    GDBKalmanSetMat(&pTrajData->KafiEst.XVec, sTRAJ_C0, (uint8)0,
                    pCourseData->fCurve);
    GDBKalmanSetMat(&pTrajData->KafiEst.XsVec, sTRAJ_C0, (uint8)0,
                    pCourseData->fCurve);
    GDBKalmanSetMat(&pTrajData->KafiEst.XVec, sTRAJ_C1, (uint8)0, 0.0f);
    GDBKalmanSetMat(&pTrajData->KafiEst.XsVec, sTRAJ_C1, (uint8)0, 0.0f);

    /* Set Initial P Matrix (P = U*D*U' ==> UD = P0) */

    GDBKalmanSetMat(&pTrajData->KafiEst.UDMat, sTRAJ_C0, sTRAJ_C0,
                    CP_INIT_TRAJ_NOISE_C0);
    GDBKalmanSetMat(&pTrajData->KafiEst.UDMat, sTRAJ_C1, sTRAJ_C1,
                    CP_INIT_TRAJ_NOISE_C1);

    GDBKalmanUpdatePDiag(&pTrajData->KafiEst);
}

/*************************************************************************************************************************
  Functionname:    CPPrepareTrajectoryData */
void CPPrepareTrajectoryData(CPTrajectoryData_t *pTrajData) {
    const GDBTrafoMatrix2D_t *pM =
        VLCGetTrafoMatrix2DCOFForward(); /* Note was:
                                            VLCGetTrafoMatrix2DCOFForwardTgtSync();
                                          */
    float32 fDistXUnplausible;
    const SIScaleBracketState_t LC_State = SIReturnStateScaleBracket();
    const float32 f_DistXUnplSCurveLastCycle =
        pTrajData->fDistXUnplausibleSCurve;
    /* remember last state*/
    pTrajData->LastCycle.fTrajAngle = pTrajData->Current.fTrajAngle;
    pTrajData->LastCycle.fTrajC0 = CPKalmanGetTrajEstState(pTrajData, sTRAJ_C0);
    pTrajData->LastCycle.fTrajC1 = CPKalmanGetTrajEstState(pTrajData, sTRAJ_C1);
    /*@todo invent a TrajAngle estimation*/
    pTrajData->Current.fTrajAngle = 0.0f;

    /*compensate UnplausibleSCurve distance*/
    fDistXUnplausible =
        GDBmathTrafoXPos2D(pM, pTrajData->fDistXUnplausibleSCurve, 0.0f);
    pTrajData->fDistXUnplausibleSCurve = MAX_FLOAT(fDistXUnplausible, 0.0f);

    /*! In case of a lane change, reset the detected S-Curve */
    if ((f_DistXUnplSCurveLastCycle <
         (pTrajData->fDistXUnplausibleSCurve - C_F32_DELTA)) &&
        ((LC_State == PRE_LANE_CHANGE_LEFT) ||
         (LC_State == PRE_LANE_CHANGE_RIGHT))) {
        pTrajData->fDistXUnplausibleSCurve = f_DistXUnplSCurveLastCycle;
    }

    /*reset to Trajectory default state*/
    pTrajData->State.EgoCourseOnly = TRUE;
    pTrajData->State.FusionRoadstimation = FALSE;
    pTrajData->State.FusionTraces = FALSE;

    pTrajData->fMovObjFusRange = 0.0f;
    pTrajData->fMSETrace = 0.0f;
    pTrajData->fRoadEstFusRange = 0.0f;
}

/* ***********************************************************************
 @fn           CPCalculateTrajectory                           */
void CPCalculateTrajectory(const CPCourseData_t *pCourseData,
                           const CPTrajectoryInputParameter_t *pTrajInputPara,
                           CPTrajectoryData_t *pTrajData) {
    CPPrepareTrajectoryData(pTrajData);

    // Check if right way!

    if ((pTrajData->Config.UseRoadEstim != FALSE) ||
        (pTrajData->Config.UseObjTraces != FALSE) ||
        (pTrajData->Config.UseCamLane != FALSE)) {
        CP_t_TrajectoryPosSamples TrajectoryPosSamples;

        CPTraceTrajectory_t TraceTrajectory;
        CP_t_InputSourceParams Trace_Params;

        /*predict fused trajectory based on model*/
        CPKalmanPredictTrajectory(pTrajData);

        /***************** Prepare VDY Trajectory for Fusion
         * ********************************/
        CPCalculateVDYSamples(pCourseData,
                              &(TrajectoryPosSamples.EGO_PosSamples));

        /***************** Prepare Trace Trajectory for Fusion
         * *******************************/
        if (pTrajData->Config.UseObjTraces) {
            /* Evaluate trace information */
            CPFusionTracesEvalTraces(pCourseData, &Trace_Params);

            /* Calculate a combined trace trajectory based on all traces */
            CPCalculateCombinedTraceTrajectory(&TraceTrajectory, pCourseData,
                                               pTrajData);

            /* Calculate trace samples based on trace trajectory */
            CPCalculateTraceSamples(&TraceTrajectory,
                                    &(TrajectoryPosSamples.TRACE_PosSamples));

            /* Evaluate trace samples */
            CPFusionEvalTraceSamples(&TraceTrajectory,
                                     &(TrajectoryPosSamples.TRACE_PosSamples),
                                     pTrajData);

            /* freeze the trace trajectory data for visualization in BirdEyeView
             * V4 MO  */
            CPTraceTrajectory = TraceTrajectory;
            CPCallVLCFreezeforTraceTraject(&CPTraceTrajectory);
        }

        /***************** Sample-Based ACC Trajectory Fusion
         * ********************************/
        CPTrajSampleFusionMain(pTrajData, &TrajectoryPosSamples);

        /* Check if merging with lane detected by camera necessary */
        if (pTrajData->Config.UseCamLane) {
            CPFusionCamMain(pTrajData);
        } else {
            pTrajData->State.CamLaneFusion = FALSE;
        }

        if (pTrajData->State.EgoCourseOnly == FALSE) {
            GDBKalmanUpdatePDiag(&pTrajData->KafiEst);
        }
    }

    /*! Suppress Fusion if Blinker-Feature is active or in the first phase of
       the lane change; in the second phase of the lane change fuse the
       trajectory (in case of valid fusion data) */
    if (pTrajInputPara->bSuppressFusion != FALSE) {
        pTrajData->State.EgoCourseOnly = TRUE;
        /*! Reset all other fusion bits */
        pTrajData->State.CamLaneFusion = FALSE;
        pTrajData->State.FusionRoadstimation = FALSE;
        pTrajData->State.NaviPathFusion = FALSE;
        pTrajData->State.FusionTraces = FALSE;
    }

    /* bypassing fusion filter if fusion disabled */
    if (pTrajData->State.EgoCourseOnly != FALSE) {
        GDBKalmanSetMat(&pTrajData->KafiEst.XVec, sTRAJ_C0, (uint8)0,
                        pCourseData->fCurve);
        GDBKalmanSetMat(&pTrajData->KafiEst.XsVec, sTRAJ_C0, (uint8)0,
                        pCourseData->fCurve);
        GDBKalmanSetMat(&pTrajData->KafiEst.XVec, sTRAJ_C1, (uint8)0, 0.0f);
        GDBKalmanSetMat(&pTrajData->KafiEst.UDMat, sTRAJ_C0, sTRAJ_C0,
                        pCourseData->fCurveVar);
        GDBKalmanSetMat(&pTrajData->KafiEst.XsVec, sTRAJ_C1, (uint8)0, 0.0f);
        GDBKalmanSetMat(&pTrajData->KafiEst.UDMat, sTRAJ_C1, sTRAJ_C1, 0.0f);
    }

    /*set C0 and C1 in interface struct*/
    pTrajData->Current.fTrajC0 = CPKalmanGetTrajEstState(pTrajData, sTRAJ_C0);
    pTrajData->Current.fTrajC1 = CPKalmanGetTrajEstState(pTrajData, sTRAJ_C1);

    extern CPTrajectoryData_t SITrajectoryData;
    SITrajectoryData_EgoCourseOnly = SITrajectoryData.State.EgoCourseOnly;
    SITrajectoryData_FusionTraces = SITrajectoryData.State.FusionTraces;
    SITrajectoryData_CamLaneFusion = SITrajectoryData.State.CamLaneFusion;
    pTrajData_EgoCourseOnly = pTrajData->State.EgoCourseOnly;
    pTrajData_FusionTraces = pTrajData->State.FusionTraces;
    pTrajData_CamLaneFusion = pTrajData->State.CamLaneFusion;
    pTrajData_fTrajC0 = pTrajData->Current.fTrajC0;
}

/*************************************************************************************************************************
  Functionname:   CPTrajSampleFusionMain */
void CPTrajSampleFusionMain(CPTrajectoryData_t *pTrajData,
                            CP_t_TrajectoryPosSamples *pTrajectoryPosSamples) {
    uint8 iSample, minFusionDist, maxFusionDist;
    uint8 iSampleStart = 0;
    float32 fDeltaTrajDuringLC = 0.f;
    float32 fDeltaCombinedTrajectories = 0.f;

    CPPosSamples_t FusedPosSamples;
    /* long range fusion */
    CPPosSamples_t LRFusedPosSamples;
    /* short range fusion */
    CPPosSamples_t SRFusedPosSamples;

    CPNoiseModelLinear_t ConstantNoise;

    CPPosSamples_t TracePosSamples;
    float32 fWeightingTraces = 0.0f;

    /*************** Generate Samples for VDY Trajectory ******************/
    for (iSample = 0; iSample < MAX_NB_TRAJ_SAMPLES; iSample++) {
        LRFusedPosSamples.fx[iSample] =
            pTrajectoryPosSamples->EGO_PosSamples.fx[iSample];
        LRFusedPosSamples.fy[iSample] =
            pTrajectoryPosSamples->EGO_PosSamples.fy[iSample];
        LRFusedPosSamples.fVar[iSample] = CPGetCourseVariance(
            pTrajectoryPosSamples->EGO_PosSamples.fx[iSample],
            EGO_SPEED_X_OBJ_SYNC);

        /* Copy fused position samples to auxiliary vector */
        SRFusedPosSamples.fx[iSample] = LRFusedPosSamples.fx[iSample];
        SRFusedPosSamples.fy[iSample] = LRFusedPosSamples.fy[iSample];
        SRFusedPosSamples.fVar[iSample] = LRFusedPosSamples.fVar[iSample];
    }
    LRFusedPosSamples.nb_samples = MAX_NB_TRAJ_SAMPLES;
    SRFusedPosSamples.nb_samples = MAX_NB_TRAJ_SAMPLES;

    /*************** Generate Samples for Trace Trajectory ****************/
    for (iSample = 0;
         iSample < MIN(MAX_NB_TRAJ_SAMPLES,
                       pTrajectoryPosSamples->TRACE_PosSamples.nb_samples);
         iSample++) {
        TracePosSamples.fx[iSample] =
            pTrajectoryPosSamples->TRACE_PosSamples.fx[iSample];
        TracePosSamples.fy[iSample] =
            pTrajectoryPosSamples->TRACE_PosSamples.fy[iSample];
        if (fABS(EGO_CURVE_OBJ_SYNC) > CP_TRACEWEIGHT_CURVE_THRESH) {
            fWeightingTraces =
                MINMAX_FLOAT(0.5f, 1.0f, 0.5f + pTrajData->fMSETrace);
        } else {
            fWeightingTraces =
                MINMAX_FLOAT(1.3f, 1.5f, 1.3f + pTrajData->fMSETrace);
        }

        TracePosSamples.fVar[iSample] =
            fWeightingTraces *
            CPGetTraceTrajVariance(
                pTrajectoryPosSamples->TRACE_PosSamples.fx[iSample],
                EGO_SPEED_X_OBJ_SYNC);
    }
    TracePosSamples.nb_samples =
        pTrajectoryPosSamples->TRACE_PosSamples.nb_samples;

    /******* Fusion of the trajectory with the longest valid distance
     * ************************/

    /**************** Fusion of Trace, Road, and VDY Samples ***************/
    /******************** for The Short Range Trajectory ******************/

    /********************** Fusion of Road Trajectory ********************/

    /********************** Fusion of Trace Trajectory ********************/

    if (pTrajData->State.FusionTraces != FALSE) {
        CPFusePosSamples(&SRFusedPosSamples, &TracePosSamples);
    }

    maxFusionDist = MIN(pTrajectoryPosSamples->TRACE_PosSamples.nb_samples,
                        MAX_NB_TRAJ_SAMPLES);
    minFusionDist = 0u;

    /************* Combine SR-Trajectory with LR-Trajectory ***************/

    for (iSample = iSampleStart; iSample < minFusionDist; iSample++) {
        FusedPosSamples.fx[iSample] = SRFusedPosSamples.fx[iSample];
        FusedPosSamples.fy[iSample] =
            SRFusedPosSamples.fy[iSample] + fDeltaTrajDuringLC;
        FusedPosSamples.fVar[iSample] = SRFusedPosSamples.fVar[iSample];
    }

    for (iSample = minFusionDist; iSample < maxFusionDist; iSample++) {
        FusedPosSamples.fx[iSample] = LRFusedPosSamples.fx[iSample];

        FusedPosSamples.fy[iSample] =
            LRFusedPosSamples.fy[iSample] + fDeltaCombinedTrajectories;

        FusedPosSamples.fVar[iSample] = LRFusedPosSamples.fVar[iSample];
    }

    /* Limit maximum fusion distance as a function of VDY. In case the VDY
       trajectory indicates a narrow curve, do not use the complete amount of
       samples. Instead, the maximum fusion distance is set to the field-of-view
       calculated by the number of samples of the VDY samples */
    FusedPosSamples.nb_samples =
        (pTrajectoryPosSamples->EGO_PosSamples.nb_samples < maxFusionDist)
            ? pTrajectoryPosSamples->EGO_PosSamples.nb_samples
            : maxFusionDist;

    /* Update Kalman filter with fused samples */
    ConstantNoise.fNoiseOffset = CP_KALMAN_POSAMPLES_NOISE_OFFSET;
    ConstantNoise.fNoiseGradient = 0.f;

    CPKalmanUpdatePos(&FusedPosSamples, ConstantNoise, pTrajData);
}

/*************************************************************************************************************************
  Functionname:    CPCalculateVDYSamples */
static void CPCalculateVDYSamples(const CPCourseData_t *pCourseData,
                                  CPPosSamples_t *pEGO_PosSamples) {
    CPCourseData_t CourseDataLocal = *pCourseData;

    uint8 i;
    const float32 f_yDistAtX =
        (CP_MAX_DIST_NEAR_RANGE * (TAN_(DEG2RAD(CP_MAX_ANGLE_NEARRANGE_FOV))) +
         SENSOR_Y_POSITION);

    /* Set local curve gradient to 0.0 for sampling */
    CourseDataLocal.fCurveGradient = 0.0f;
    CourseDataLocal.SideSlipAngle = 0.0f;

    CPSamplePosClothApprox(&CourseDataLocal, CPClothApproxType_Automatic,
                           CP_SAMPLEDIST_MAX, pEGO_PosSamples);
    CPMoveSamplesFromCoGToSensor(pEGO_PosSamples);

    /* Determine maximum number of samples based on curvature. Although VDY
       provides always 20 samples, not all of them are always suitable for the
       Kalman fitting. TYpically, in narrow curves the first
       samples points will be fitted worse compared to the last samples points,
       which are beyond the field of view. */
    /* Ensure that a minimum number of samples for VDY is always used */
    pEGO_PosSamples->nb_samples = CP_MIN_NUM_SAMPLES_VDY;
    for (i = CP_MIN_NUM_SAMPLES_VDY; i < MAX_NB_TRAJ_SAMPLES; i++) {
        if (fABS(pEGO_PosSamples->fy[i]) < f_yDistAtX) {
            pEGO_PosSamples->nb_samples = i + 1;
        } else {
            /* Lateral position of VDY samples exceeds field-of-view */
            break;
        }
    }
}

/* @todo: Is this function still needed? Currently no references to it found */

/*************************************************************************************************************************
  Functionname:    CPTraj2MOTraceParallelismCheck */
CPTraj2MOTraceParallelism_t CPTraj2MOTraceParallelismCheck(
    TraceID_t iTr, const CPTrajectoryData_t *pTrajectoryData) {
    CPTraj2MOTraceParallelism_t ret;
    sint32 iS;
    float32 fLeftThreshold, fRightThreshold;
    float32 fSAmin = 0.0F;
    float32 fSAmax = 0.0F;
    float32 fX, fY;
    float32 fStdDevTemp = 0.0F;
    CPTrajRefPoint_t TrajRefPoint;
    GDBTrafoMatrix2D_t COF2CEGO;
    /*! Get number of lanes */
    const sint8 s_NumberOfLeftLanes = FIP_s_GetLMLeftNumLane();
    const sint8 s_NumberOfRightLanes = FIP_s_GetLMRightNumLane();

    /*init*/
    ret.relevantTrace = FALSE;
    ret.fSADistMean = 0.0f;
    ret.fSADistStdDev = 0.0f;

    /* Use road's estimation of number of lanes */
    fLeftThreshold =
        (MAX(0.0f, ((float32)s_NumberOfLeftLanes)) + 1.0f) * STRASSENBREITE;
    fRightThreshold = (MAX(0.0f, ((float32)s_NumberOfRightLanes)) + 1.0f) *
                      STRASSENBREITE * -1.0F;

    COF2CEGO = GDBGetTrafoMatrixByDisplacement(VLC_fBumperToCoG, 0.0f);

    if ((FIP_STATIC_TRACE_GET_VLC_ID(iTr) <= TRACE_VALID_NO_OBJ_ID) &&
        (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr) > 0)) {
        for (iS = 0; iS < FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr); iS++) {
            fX = FIP_STATIC_TRACE_GET_X(iTr)[iS];
            fY = FIP_STATIC_TRACE_GET_Y(iTr)[iS];
            GDBmathTrafoPos2D(&COF2CEGO, &fX, &fY);
            TrajRefPoint = CPCalculateDistancePoint2Clothoid(
                fX, fY, pTrajectoryData->Current.fTrajC0,
                pTrajectoryData->Current.fTrajC1);

            ret.fSADistMean += TrajRefPoint.fDistToTraj;
            ret.fSADistStdDev += SQR(TrajRefPoint.fDistToTraj);
            if (fSAmin > TrajRefPoint.fDistToTraj) {
                fSAmin = TrajRefPoint.fDistToTraj;
            }
            if (fSAmax < TrajRefPoint.fDistToTraj) {
                fSAmax = TrajRefPoint.fDistToTraj;
            }
        }
        ret.fSADistMean /= (float32)FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr);
        /* Prevent using invalid or negative result under sqrt */
        fStdDevTemp = (ret.fSADistStdDev /
                       (float32)FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr)) -
                      SQR(ret.fSADistMean);
        if (fStdDevTemp > CP_INFINITE_DIST) {
            fStdDevTemp = CP_INFINITE_DIST;
        } else {
            /* do nothing */
        }
        if (fStdDevTemp >= C_F32_DELTA) {
            ret.fSADistStdDev = SQRT_(fStdDevTemp);
        } else {
            ret.fSADistStdDev = C_F32_DELTA;
        }
        /*check if trace is within lanes in ego-vehicles direction*/
        ret.relevantTrace =
            (boolean)(((FIP_STATIC_TRACE_GET_VLC_ID(iTr) <
                        TRACE_VALID_NO_OBJ_ID) && /*Object alive*/
                       (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr) >
                        EXTOBJAPPROX_PARALLELISM_MIN_LENGTH_TRACE) /*minimal
                                                                      Length of
                                                                      Trace*/
                       ) &&
                      ((ret.fSADistMean <
                        fLeftThreshold) && /*within carriageway */
                       (ret.fSADistMean >
                        fRightThreshold) && /*within carriageway */
                       (ret.fSADistStdDev < PARALLELISM_THRESH) /*parallelism*/
                       ));                                      /*parallelism*/
    }
    return ret;
}

/*************************************************************************************************************************
  Functionname:    CPFusePosSamples */
static void CPFusePosSamples(CPPosSamples_t *const Samples,
                             const CPPosSamples_t *const MeasSamples) {
    uint32 i;
    uint32 fMaxFusionDist;

    /* Determine maximum fusion distance */
    fMaxFusionDist = MIN(Samples->nb_samples, MeasSamples->nb_samples);

    for (i = 0; i < fMaxFusionDist; i++) {
        /* Calculate weighting factor as a function of variance */
        const float32 fWeight =
            Samples->fVar[i] / (Samples->fVar[i] + MeasSamples->fVar[i]);

        /* Calculate mean of combined random sample */
        Samples->fx[i] = Samples->fx[i] +
                         ((fWeight) * (MeasSamples->fx[i] - Samples->fx[i]));
        Samples->fy[i] = Samples->fy[i] +
                         ((fWeight) * (MeasSamples->fy[i] - Samples->fy[i]));

        /* Calculate variance of combined random sample */
        Samples->fVar[i] = (Samples->fVar[i] * MeasSamples->fVar[i]) /
                           (Samples->fVar[i] + MeasSamples->fVar[i]);
    }
}

/*************************************************************************************************************************
  Functionname:    CPSampleGradFromCourse */
void CPSampleGradFromCourse(const CPCourseData_t *pCourseData,
                            CPGradSamples_t *pSamples) {
    uint32 i;
    float32 x_s = 0.0F;
    const float32 x_max = MINMAX(CP_SAMPLEDIST_MIN, CP_SAMPLEDIST_MAX,
                                 CP_SAMPLETIMEDIST * EGO_SPEED_X_OBJ_SYNC);
    const float32 x_inc = x_max / (float32)MAX_NB_TRAJ_SAMPLES;
    const float32 fPlausibleC1 =
        CPGetCourseGradUpdateSTDC1AtSpeed(EGO_SPEED_X_OBJ_SYNC);

    for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
        /*generate MAX_NB_TRAJ_SAMPLES 200m along Course*/
        x_s += x_inc;
        pSamples->fx[i] = x_s - VLC_fBumperToCoG;
        pSamples->fdydx[i] = pCourseData->fCurve * x_s;
        pSamples->fdydxMinStdDev[i] =
            (0.5f * SQR(x_s) * fPlausibleC1) + CP_GRADUPDATE_STD_MIN;
        /*pSamples->fdydxMaxStdDev[i] = pSamples->fdydxMinStdDev[i];*/
        pSamples->valid[i] = 2u;
    }
}

/*************************************************************************************************************************
  Functionname:    CPClothApproxByPolynom */
static Vector2_f32_t CPClothApproxByPolynom(const float32 fC0,
                                            const float32 fC1,
                                            const float32 fYawAngle,
                                            const float32 fDistOnCloth) {
    float32 fX, fXX, fXXX, fY;
    Vector2_f32_t sRes;

    fX = fDistOnCloth; /* Approximation of X Value */
    fXX = fX * fX;
    fXXX = fX * fXX;

    fY = ((1.f / 6.f) * fC1 * fXXX) + ((1.f / 2.f) * fC0 * fXX) +
         (fYawAngle * fX);

    sRes.fXDist = fX;
    sRes.fYDist = fY;

    return sRes;
}

/*************************************************************************************************************************
  Functionname:    CPClothApproxByCircle */
static Vector2_f32_t CPClothApproxByCircle(const float32 fC0,
                                           const float32 fDistOnCloth) {
    float32 fX, fY, fRadius;
    Vector2_f32_t sRes;

    fRadius = 1.0f / fC0;

    fX = fRadius * GDBcos_52(fDistOnCloth);
    fY = (fRadius * GDBsin_52(fDistOnCloth)) + fRadius;

    sRes.fXDist = fX;
    sRes.fYDist = fY;

    return sRes;
}

/*************************************************************************************************************************
  Functionname:    CPSamplePosClothApprox */
void CPSamplePosClothApprox(const CPCourseData_t *pCPCourseData,
                            const eCPClothApproxType_t CPClothApproxType,
                            const float32 fSampleDistMax,
                            CPPosSamples_t *pCPPosSamples) {
    uint32 i;
    float32 x_inc = 0.0f;
    float32 x_s = 0.0f;
    float32 t_inc = 0.0f;
    float32 t_s = 0.0f;
    float32 fRadius = 0.0f;
    float32 fMaxArclength;

    eCPClothApproxType_t selectedApproxType;

    /* Figure out which Approximation (polynomial or circle) to use */
    if (CPClothApproxType == CPClothApproxType_Automatic) {
        if (fABS(pCPCourseData->fCurve) > CURVATURE_USE_CIRCLE_EQUATION) {
            selectedApproxType = CPClothApproxType_CircleOnly;
        } else {
            selectedApproxType = CPClothApproxType_PolynomialOnly;
        }
    } else {
        selectedApproxType = CPClothApproxType;
    }

    /* Start sampling */
    switch (selectedApproxType) {
        case CPClothApproxType_PolynomialOnly:
            /* do sampling with polynomial approximation */
            x_inc = (fSampleDistMax / (float32)MAX_NB_TRAJ_SAMPLES);

            for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
                Vector2_f32_t vCoordsTemp;

                x_s += x_inc;
                vCoordsTemp = CPClothApproxByPolynom(
                    pCPCourseData->fCurve, pCPCourseData->fCurveGradient,
                    pCPCourseData->SideSlipAngle, x_s);

                pCPPosSamples->fx[i] = vCoordsTemp.fXDist;
                pCPPosSamples->fy[i] = vCoordsTemp.fYDist;
            }
            break;
        case CPClothApproxType_CircleOnly:
            /* do sampling with circle approximation */
            t_s = -C_PI / 2.F;
            fRadius = 1.F / pCPCourseData->fCurve;
            fMaxArclength = (2.F * fABS(fRadius)) * (C_PI / 5.F);
            fMaxArclength = MIN_FLOAT(fSampleDistMax, fMaxArclength);
            t_inc = (fMaxArclength * (1.0f / (float32)MAX_NB_TRAJ_SAMPLES)) *
                    pCPCourseData->fCurve;

            for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
                Vector2_f32_t vCoordsTemp;

                t_s += t_inc;
                vCoordsTemp = CPClothApproxByCircle(pCPCourseData->fCurve, t_s);

                pCPPosSamples->fx[i] = vCoordsTemp.fXDist;
                pCPPosSamples->fy[i] = vCoordsTemp.fYDist;
            }
            break;
        default:
            /* selectedApproxType has unsupported Content */
            break;
    }

    pCPPosSamples->nb_samples = MAX_NB_TRAJ_SAMPLES;
}

/*************************************************************************************************************************
  Functionname:    CPLimitSamplesXDist */
void CPLimitSamplesXDist(const float32 fXDistMax,
                         CPPosSamples_t *pCPPosSamples) {
    uint32 i;
    uint8 uiNbSamplesNew = 0U;

    for (i = 0u; i < pCPPosSamples->nb_samples; i++) {
        if (pCPPosSamples->fx[i] < fXDistMax) {
            uiNbSamplesNew++;
        }
    }

    pCPPosSamples->nb_samples = uiNbSamplesNew;
}

/*************************************************************************************************************************
  Functionname:    CPMoveSamplesFromCoGToSensor */
void CPMoveSamplesFromCoGToSensor(CPPosSamples_t *pCPPosSamples) {
    uint32 i;

    for (i = 0u; i < pCPPosSamples->nb_samples; i++) {
        pCPPosSamples->fx[i] = pCPPosSamples->fx[i] - VLC_fBumperToCoG;
    }
}

/*************************************************************************************************************************
  Functionname:    CPGetTraceEstimationAsCourseData */
CPCourseData_t CPGetTraceEstimationAsCourseData(
    const CPTraceTrajectory_t *pTraceTrajectory) {
    CPCourseData_t CourseDataResult;

    CourseDataResult.fCurve = pTraceTrajectory->fCurve;
    CourseDataResult.fCurveGradient = 0.f;
    CourseDataResult.fCurveVar = 0.f;
    CourseDataResult.SideSlipAngle = 0.f;
    CourseDataResult.SideSlipAngleVariance = 0.f;

    return CourseDataResult;
}

/*************************************************************************************************************************
  Functionname:    CPCalculateDistance2Traj */
void CPCalculateDistance2Traj(const float32 fX,
                              const float32 fY,
                              const boolean bUseEgoCourseOnly,
                              const GDBTrajectoryData_t *pTrajData,
                              CPTrajRefPoint_t *pDist2Traj) {
    if (bUseEgoCourseOnly != FALSE) {
        *pDist2Traj =
            CPCalculateDistancePoint2Circle(fX, fY, pTrajData->fTrajC0);
    } else {
        *pDist2Traj = CPCalculateDistancePoint2Clothoid(
            fX, fY, pTrajData->fTrajC0, pTrajData->fTrajC1);
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalculateDistancePoint2Circle */
CPTrajRefPoint_t CPCalculateDistancePoint2Circle(float32 fX,
                                                 float32 fY,
                                                 float32 fC0) {
    float32 fRadius = 999999.f;
    float32 fR = 0.f;
    float32 fDistToCourse = 0.f;
    float32 fDistOnCourse = 0.f;
    float32 fNormVecX = 0.f;
    float32 fNormVecY = 0.f;
    float32 fRefCourseDistX = 0.f;
    float32 fRefCourseDistY = 0.f;
    CPTrajRefPoint_t ReferencePoint;

    if (fABS(fC0) > CURVATURE_USE_CIRCLE_EQUATION) {
        fRadius = 1.0f / (fC0);
        /* Object Transform to Moment Pole Coordinates */
        fY -= fRadius;
        fR = SQRT_(SQR(fX) + SQR(fY));
        /* NormVec to Course always pointing to the left side of course */
        if (fR >= C_F32_DELTA) {
            if (fC0 > 0.0f) {
                fNormVecX = -fX / fR;
                fNormVecY = -fY / fR;
                fDistToCourse = (fABS(fRadius) - fR);
                fDistOnCourse = fRadius * (C_HALFPI + ATAN2_(fY, fX));
            } else {
                fNormVecX = fX / fR;
                fNormVecY = fY / fR;
                fDistToCourse = -(fABS(fRadius) - fR);
                fDistOnCourse = fRadius * (ATAN2_(fY, fX) - C_HALFPI);
            }
        }
        /* DistCourse (fRadius-fR) positive when object left of course; negative
         * when object right of course*/

        fRefCourseDistX = fX - (fNormVecX * fDistToCourse);
        fRefCourseDistY = (fY - (fNormVecY * fDistToCourse)) + fRadius;

    } else {
        /* use old parabolic approximation for wide curves and distance in Y-
         * Direction*/
        fRefCourseDistX = fX;
        fRefCourseDistY = SQR(fX) * (fC0) * (0.5f);
        fDistToCourse = (fY - fRefCourseDistY);
        /*instead of integral 0 to fX of function SQRT(1+(fC0*x)^2) dx*/
        fDistOnCourse = fX;
    }
    ReferencePoint.fX = fRefCourseDistX;
    ReferencePoint.fY = fRefCourseDistY;
    ReferencePoint.fDistToTraj = fDistToCourse;
    ReferencePoint.fDistOnTraj = fDistOnCourse;

    return ReferencePoint;
}

/*************************************************************************************************************************
  Functionname:    CPApproximateRefpoint */
static void CPApproximateRefpoint(const float32 fX,
                                  const float32 fY,
                                  const float32 fC0,
                                  const float32 fC1,
                                  float32 *pfRefX) {
    float32 fXc = *pfRefX;
    float32 fXXc = SQR(fXc);
    float32 fC1XXc = fC1 * fXXc;
    float32 fC0Xc = fC0 * fXc;
    float32 fYc = (C_SIXTH * fC1XXc * fXc) + (0.5f * fC0Xc * fXc);
    float32 fm = ((0.5f * fC1XXc) + fC0Xc);
    float32 fm_inv;

    if (fABS(fm) < C_F32_DELTA) {
        *pfRefX = fX;
    } else {
        fm_inv = 1.0f / fm;
        *pfRefX = (((fY - fYc) + (fm * fXc)) + (fm_inv * fX)) / (fm + fm_inv);
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalculateDistancePoint2Clothoid */
CPTrajRefPoint_t CPCalculateDistancePoint2Clothoid(const float32 fX,
                                                   const float32 fY,
                                                   const float32 fC0,
                                                   const float32 fC1) {
    float32 fTemp;
    float32 fYDiff;
    CPTrajRefPoint_t ReferencePoint;
    ReferencePoint.fX = fX;
    CPApproximateRefpoint(fX, fY, fC0, fC1, &ReferencePoint.fX);

    fTemp = SQR(ReferencePoint.fX);
    ReferencePoint.fY =
        (0.5f * fC0 * fTemp) + ((fC1 * fTemp * ReferencePoint.fX) * C_SIXTH);
    fYDiff = fY - ReferencePoint.fY;
    ReferencePoint.fDistToTraj =
        SQRT_(SQR(fX - ReferencePoint.fX) + SQR(fYDiff));
    if (fYDiff < 0.0f) {
        ReferencePoint.fDistToTraj *= -1.0f;
    }

    /*@todo implement arclength on clothoid approximation*/
    /*@hack use difference in x instead*/
    ReferencePoint.fDistOnTraj = ReferencePoint.fX;

    return ReferencePoint;
}

/*************************************************************************************************************************
  Functionname:    CPGetCourseVariance */
float32 CPGetCourseVariance(const float32 f_DistX,
                            const float32 f_EgoSpeedCorrected) {
    float32 f_TimeGap, f_SQR_TimeGap;
    float32 f_Var;

    if (f_EgoSpeedCorrected > C_F32_DELTA) /* Prevent division by zero */
    {
        /* Compute time-gap */
        f_TimeGap = fABS(f_DistX) / f_EgoSpeedCorrected;
    } else {
        /* Get maximum time-gap
           If speed is almost zero, then a large time will result in a large
           variance */
        f_TimeGap = CP_TRAJ_VAR_TIMEGAP_MAX;
    }

    /* Calculate the square of the time */
    f_SQR_TimeGap = SQR(f_TimeGap);

    /* Two-segmented variance approximation.
       For the time-gap 0s to 1s and above 1s different approximations are used.
     */
    if (f_TimeGap < CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_LOW) {
        /* For low time-gap values approximation by power function with exponent
         * 3 */
        f_Var = (CP_VDY_COURSE_VAR_COEFF_LOW * f_SQR_TimeGap * f_TimeGap) +
                CP_TRAJ_VAR_OFFSET;
    } else {
        /* For high time-gap values approximation by power function with
         * exponent 4.5 */
        f_Var = (CP_VDY_COURSE_VAR_COEFF_LOW * f_SQR_TimeGap * f_SQR_TimeGap *
                 SQRT(f_TimeGap)) +
                CP_TRAJ_VAR_OFFSET;
    }

    return f_Var;
}

/*************************************************************************************************************************
  Functionname:    CPGetTraceTrajVariance */
float32 CPGetTraceTrajVariance(const float32 f_DistX,
                               const float32 f_EgoSpeedCorrected) {
    float32 f_TimeGap, f_SQR_TimeGap;
    float32 f_Var;
    const FIP_t_FusedRoadType t_RoadTypeLevel_1 = FIP_ROAD_TYPE_CITY;

    if (f_EgoSpeedCorrected > C_F32_DELTA) /* Prevent division by zero */
    {
        /* Compute time-gap */
        f_TimeGap = fABS(f_DistX) / f_EgoSpeedCorrected;
    } else {
        /* Get maximum time-gap.
           If speed is almost zero, then a large time will result in a large
           variance */
        f_TimeGap = CP_TRAJ_VAR_TIMEGAP_MAX;
    }

    /* Calculate the square of the time */
    f_SQR_TimeGap = SQR(f_TimeGap);

    /* Use different approximation functions of the variance for highways and
     * country road */
    if (((f_EgoSpeedCorrected < CP_TRACE_TRAJ_VEGO_MAX_COUNTRY) &&
         (t_RoadTypeLevel_1 != FIP_ROAD_TYPE_HIGHWAY)) ||
        (fABS(EGO_CURVE_OBJ_SYNC) > CP_TRACE_TRAJ_CURVE_MAX_HIGHWAY)) {
        /* Three-segmented variance approximation for country road and the rest.
         For the time-gap 0s-1s, 1s-4s and above 4s different approximations are
         used. */
        if (f_TimeGap < CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_LOW) {
            /* For low time-gap values approximation by power function with
             * exponent 3 */
            f_Var = (CP_TRACE_TRAJ_COURSE_VAR_COEFF_COUNTRY_LOW *
                     f_SQR_TimeGap * f_TimeGap) +
                    CP_TRAJ_VAR_OFFSET;
        } else if (f_TimeGap < CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_MID) {
            /* For mid time-gap values approximation by power function with
             * exponent 3.5 */
            f_Var = (CP_TRACE_TRAJ_COURSE_VAR_COEFF_COUNTRY_LOW *
                     f_SQR_TimeGap * f_TimeGap * SQRT(f_TimeGap)) +
                    CP_TRAJ_VAR_OFFSET;
        } else {
            /* For high time-gap values approximation by power function with
             * exponent 5 */
            f_Var = (CP_TRACE_TRAJ_COURSE_VAR_COEFF_COUNTRY_HIGH *
                     f_SQR_TimeGap * f_SQR_TimeGap * f_TimeGap) +
                    CP_TRAJ_VAR_OFFSET;
        }
    } else {
        /* One-segmented variance approximation for highways.
         For the whole time-gap one single function approximation is used. */
        /* Approximation by power function with exponent 3.5 */
        f_Var = (CP_TRACE_TRAJ_COURSE_VAR_COEFF_HIGHWAY * f_SQR_TimeGap *
                 f_TimeGap * SQRT(f_TimeGap)) +
                CP_TRAJ_VAR_OFFSET;
    }
    return f_Var;
}

/*************************************************************************************************************************
  Functionname:    CPGetRoadVariance */
float32 CPGetRoadVariance(const float32 f_DistX,
                          const float32 f_EgoSpeedCorrected) {
    float32 f_TimeGap, f_SQR_TimeGap;
    float32 f_Var;

    if (f_EgoSpeedCorrected > C_F32_DELTA) /* Prevent division by zero */
    {
        /* Compute time-gap */
        f_TimeGap = fABS(f_DistX) / f_EgoSpeedCorrected;
    } else {
        /* Get maximum time-gap.
           If speed is almost zero, then a large time will result in a large
           variance */
        f_TimeGap = CP_TRAJ_VAR_TIMEGAP_MAX;
    }

    /* Calculate the square of the time */
    f_SQR_TimeGap = SQR(f_TimeGap);

    /* Three-segmented variance approximation.
       For the time-gap 0s-1s, 1s-4s and above 4s different approximations are
       used. */
    if (f_TimeGap < CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_LOW) {
        /* For low time-gap values approximation by power function with exponent
         * 2 */
        f_Var =
            (CP_ROAD_COURSE_VAR_COEFF_LOW * f_SQR_TimeGap) + CP_TRAJ_VAR_OFFSET;
    } else if (f_TimeGap < CP_TRAJ_VAR_APPROX_SEGMENT_TIMEGAP_MID) {
        /* For mid time-gap values approximation by power function with
         * exponent 2.5 */
        f_Var =
            (CP_ROAD_COURSE_VAR_COEFF_LOW * f_SQR_TimeGap * SQRT(f_TimeGap)) +
            CP_TRAJ_VAR_OFFSET;
    } else {
        /* For high time-gap values approximation by power function with
         * exponent 3.5 */
        f_Var = (CP_ROAD_COURSE_VAR_COEFF_HIGH * f_SQR_TimeGap * f_TimeGap *
                 SQRT(f_TimeGap)) +
                CP_TRAJ_VAR_OFFSET;
    }

    return f_Var;
}

/*************************************************************************************************************************
  Functionname:    CPGetPlausibleC1AtSpeed */
float32 CPGetPlausibleC1AtSpeed(const fVelocity_t EgoSpeedCorrected) {
    float32 fA;

    fA = dGDBmathLineareFunktion(&CPREPlausibilityHighway, EgoSpeedCorrected);

    return (1.0f / SQR(fA));
}

/*************************************************************************************************************************
  Functionname:    CPGetCourseGradUpdateSTDC1AtSpeed */
float32 CPGetCourseGradUpdateSTDC1AtSpeed(const fVelocity_t EgoSpeedCorrected) {
    float32 fA;

    fA = dGDBmathLineareFunktion(&CPCourseGradUpdateSTD, EgoSpeedCorrected);

    return (1.0f / SQR(fA));
}

/*************************************************************************************************************************
  Functionname:    CPCopyTraj2Meas */
void CPCopyTraj2Meas(const CPTrajectoryData_t *pTrajectoryData,
                     CPTrajectoryMeas_t *pMeasTrajectory) {
    pMeasTrajectory->fc0 = pTrajectoryData->Current.fTrajC0;
    pMeasTrajectory->fc1 = pTrajectoryData->Current.fTrajC1;
    pMeasTrajectory->fphi = pTrajectoryData->Current.fTrajAngle;
    /*center of gravity in car front coordinate system*/
    pMeasTrajectory->fx = -1.0f * VLC_fBumperToCoG;
    pMeasTrajectory->fy = 0.0f;
    pMeasTrajectory->Config = pTrajectoryData->Config;
    pMeasTrajectory->State = pTrajectoryData->State;
}

/*************************************************************************************************************************
  Functionname:    CPCopyCourse2Meas */
void CPCopyCourse2Meas(const CPCourseData_t *pCourseData,
                       CPTrajectoryMeas_t *pMeasTrajectory) {
    pMeasTrajectory->fc0 = pCourseData->fCurve;
    pMeasTrajectory->fc1 = 0.0f;
    pMeasTrajectory->fphi = pCourseData->SideSlipAngle;
    /*center of gravity in car front coordinate system*/
    pMeasTrajectory->fx = -1.0f * VLC_fBumperToCoG;
    pMeasTrajectory->fy = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    CPGetCourseDataEgo */
void CPGetCourseDataEgo(
    CPCourseData_t *pCourseData,
    const boolean bUseSlipAngle /*, const VED_VehDyn_t * pInputSignals*/) {
#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "CPGetCourseData explicit ego dynamic parameter passing currently commented out! Check if functionality called for, or not needed!")

#endif
    pCourseData->fCurve =
        EGO_CURVE_OBJ_SYNC; /* pInputSignals->Lateral.Curve.Curve */
    pCourseData->fCurveGradient =
        EGO_CURVE_GRAD_OBJ_SYNC; /* pInputSignals->Lateral.Curve.Gradient */
    pCourseData->fCurveVar =
        EGO_CURVE_VAR_OBJ_SYNC; /* pInputSignals->Lateral.Curve.varC0 */

    if (bUseSlipAngle != FALSE) {
        pCourseData->SideSlipAngle =
            EGO_SIDE_SLIP_OBJ_SYNC; /* pInputSignals->Lateral.SlipAngle.SideSlipAngle
                                     */
        pCourseData->SideSlipAngleVariance =
            EGO_SIDE_SLIP_VAR_OBJ_SYNC; /* pInputSignals->Lateral.SlipAngle.Variance
                                         */
    } else {
        pCourseData->SideSlipAngle = 0.0f;
        pCourseData->SideSlipAngleVariance = 0.0f;
    }
}

/*************************************************************************************************************************
  Functionname:    CPGetTracePoly */
void CPGetTracePoly(CPTracePolyL2_t *pTracePoly, const TraceID_t iTr) {
    CPTraceAddData_t *const pCurTraceAddData = &CPTraceAdd[iTr];

    pTracePoly->fC0 = pCurTraceAddData->ApproxPoly.fC0;
    pTracePoly->fC1 = pCurTraceAddData->ApproxPoly.fC1;
    pTracePoly->fC2 = pCurTraceAddData->ApproxPoly.fC2;
    pTracePoly->isValid = pCurTraceAddData->ApproxPoly.isValid;
}

/*************************************************************************************************************************
  Functionname:    CPGetCurvature */
float32 CPGetCurvature(CPCourseData_t const *const pTrajectory,
                       const float32 fXPosition) {
    float32 fCurvatureAtXPosition;

    fCurvatureAtXPosition =
        pTrajectory->fCurve + (pTrajectory->fCurveGradient * fXPosition);

    return fCurvatureAtXPosition;
}

/*************************************************************************************************************************
  Functionname:    CPCallVLCFreezeforTraceTraject */
void CPCallVLCFreezeforTraceTraject(
    CPTraceTrajectory_t *p_CPTraceTrajectoryInfo) {
    //(void)VLC_FREEZE_DATA(&CPTraceTrajectoryMeasInfo, p_CPTraceTrajectoryInfo,
    // NULL);
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */