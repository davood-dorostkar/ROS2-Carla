/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * guotao <guotao1@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "./cp.h"
#include "./cp_par.h"
#include "./cp_kalman.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
#define CP_MIN_TRACE_LENGTH 20.f
#define CP_MAX_TRACE_MSE 5.0f
#define CP_MAX_TRACE_DIST_HIGH_VAR 130.f
#define CP_MIN_AVLC_QUALITY 75u

#define CP_MAX_DEVIATION_SAMPLE_TO_HYPO (0.6f)

#define RADIUS_MIN (20u)
#define LAT_ACCEL_MIN_RADIUS (3u)
#define EXTRAPOLATION_THRES (40.f)

/* Number of parallel tested hypotheses */
#define CP_PSO_NUM_HYPOTHESIS (3u)

/* PSO internal variables */
#define CP_TRACE_PSO_XI (0.7298f)
#define CP_TRACE_PSO_C1 (2.8f)
#define CP_TRACE_PSO_C2 (1.3f)
#define CP_TRACE_PSO_INIT_FIT (1000.f)
#define CP_TRACE_PSO_CHANGE_RADIUS (500.f)

/* Maximum number of PSO iterations.
  This has an huge effect on complexity, do not use values
  larger than 10 */
#define CP_TRACE_PSO_ITER_MAX (10u)

#define CP_TRACE_RADIUS_MAX (99999.f)
#define CP_TRACE_CURVATURE_MIN (1e-5f)

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

static void CPSetValidityOfTraceForFusion(TraceID_t iTrace,
                                          uint8 ui_TraceQualityFlag);
static float32 CPCalculateAvgDevTrace(float32 fRadius);

/*************************************************************************************************************************
  Functionname:    CPFusionTraceCalcTraceCircleParallelism */
CPTrace2CurveParallelism_t CPFusionTraceCalcTraceCircleParallelism(
    uint8 iTr, float32 fC0, const GDBTrafoMatrix2D_t *MOT2CIR) {
    CPTrace2CurveParallelism_t ret;
    CPTrajRefPoint_t TrajRefPoint;
    sint8 iS;

    ret.fDistMean = 0.0f;
    ret.fDistStdDev = 0.0f;

    if ((FIP_STATIC_TRACE_GET_VLC_ID(iTr) < TRACE_VALID_NO_OBJ_ID) &&
        (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr) >
         PARALLELISM_MIN_LENGTH_TRACE)) {
        f32_t fVarianceVal;
        const float32 InvNoPts =
            (1.f / (float32)FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr));
        for (iS = 0; iS < FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr); iS++) {
            float32 fX = FIP_STATIC_TRACE_GET_X(iTr)[iS];
            float32 fY = FIP_STATIC_TRACE_GET_Y(iTr)[iS];
            GDBmathTrafoPos2D(MOT2CIR, &fX, &fY);
            TrajRefPoint = CPCalculateDistancePoint2Circle(fX, fY, fC0);
            ret.fDistMean += TrajRefPoint.fDistToTraj;
            ret.fDistStdDev += SQR(TrajRefPoint.fDistToTraj);
        }
        ret.fDistMean *= InvNoPts;

        fVarianceVal = (ret.fDistStdDev * InvNoPts) - SQR(ret.fDistMean);
        if (fVarianceVal > 0.f) {
            ret.fDistStdDev = SQRT_(fVarianceVal);
        } else {
            ret.fDistStdDev = 0.f;
        }
    } else {
        ret.fDistMean = CP_TRAJ_INVALID_VALUE; /*EOMOT_VALID_OBJ_ID*/
        ret.fDistStdDev = CP_TRAJ_INVALID_VALUE;
    }
    return ret;
}

/*************************************************************************************************************************
  Functionname:    CPFusionTraceCalcTraceClothoidParallelism */
CPTrace2CurveParallelism_t CPFusionTraceCalcTraceClothoidParallelism(
    uint8 iTr, float32 fC0, float32 fC1, const GDBTrafoMatrix2D_t *MOT2CIR) {
    CPTrace2CurveParallelism_t ret;
    CPTrajRefPoint_t TrajRefPoint;
    sint8 iS;

    ret.fDistMean = 0.0f;
    ret.fDistStdDev = 0.0f;

    if ((FIP_STATIC_TRACE_GET_VLC_ID(iTr) < TRACE_VALID_NO_OBJ_ID) &&
        (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr) >
         PARALLELISM_MIN_LENGTH_TRACE)) {
        f32_t fVarianceVal;
        const float32 InvNoPts =
            (1.f / (float32)FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr));
        for (iS = 0; iS < FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr); iS++) {
            float32 fX = FIP_STATIC_TRACE_GET_X(iTr)[iS];
            float32 fY = FIP_STATIC_TRACE_GET_Y(iTr)[iS];
            GDBmathTrafoPos2D(MOT2CIR, &fX, &fY);
            TrajRefPoint = CPCalculateDistancePoint2Clothoid(fX, fY, fC0, fC1);
            ret.fDistMean += TrajRefPoint.fDistToTraj;
            ret.fDistStdDev += SQR(TrajRefPoint.fDistToTraj);
        }
        ret.fDistMean *= InvNoPts;

        fVarianceVal = (ret.fDistStdDev * InvNoPts) - SQR(ret.fDistMean);
        if (fVarianceVal > 0.f) {
            ret.fDistStdDev = SQRT_(fVarianceVal);
        } else {
            ret.fDistStdDev = 0.f;
        }
    } else {
        ret.fDistMean = CP_TRAJ_INVALID_VALUE; /*EOMOT_VALID_OBJ_ID*/
        ret.fDistStdDev = CP_TRAJ_INVALID_VALUE;
    }
    return ret;
}

/*************************************************************************************************************************
  Functionname:    CPFusionTraceIsFusionSituation */

boolean CPFusionTraceIsFusionSituation(CPTrajectoryState_t *pTrajState) {
    boolean bRet = FALSE;

    if (pTrajState->FusionTraceHystEgoSpeed) {
        if (EGO_SPEED_X_OBJ_SYNC <
            (FUSIONTRACESPEEDTHRESH - FUSIONTRACESPEEDHYSTOFFSET)) {
            pTrajState->FusionTraceHystEgoSpeed = FALSE;
        }
    } else {
        if (EGO_SPEED_X_OBJ_SYNC > FUSIONTRACESPEEDTHRESH) {
            pTrajState->FusionTraceHystEgoSpeed = TRUE;
        }
    }
    /* if speed is over 60(-Hyst) and there is no Tunnel --> TRUE */
    if ((pTrajState->FusionTraceHystEgoSpeed)

            ) {
        bRet = TRUE;
    } else {
        bRet = FALSE;
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    CPFusionTracesEvalTraces */
void CPFusionTracesEvalTraces(const CPCourseData_t *pCourseData,
                              CP_t_InputSourceParams *pTrace_Params) {
    uint8 iTrace;
    boolean Trace_EgoValid[FIP_STATIC_TRACE_NO_OF_TRACES];
    // boolean Trace_RoadValid[FIP_STATIC_TRACE_NO_OF_TRACES];
    /*trace transformation to center of gravity without slipangle*/
    GDBTrafoMatrix2D_t MOT2EGO =
        GDBGetTrafoMatrixByDisplacement(VLC_fBumperToCoG, SENSOR_Y_POSITION);

    const float32 fLeftThreshold = STRASSENBREITE;
    const float32 fRightThreshold = STRASSENBREITE * -1.0F;

    /*assess Traces */
    for (iTrace = 0; iTrace < FIP_STATIC_TRACE_NO_OF_TRACES; iTrace++) {
        pTrace_Params->Trace_QualityValid[iTrace] =
            ((FIP_STATIC_TRACE_GET_VLC_ID(iTrace) <
              TRACE_VALID_NO_OBJ_ID) /*Object alive*/
             // delete since we do not have shadow evaluate
             /*&& (!OBJ_IS_SHADOW(FIP_STATIC_TRACE_GET_VLC_ID(iTrace)))*/
             /* Reject traces of objects with high cluster variance and large
                longitudinal distance */
             && ((OBJ_GET_AVLC_FUN_PRESEL_QUALITY(FIP_STATIC_TRACE_GET_VLC_ID(
                      iTrace)) > CP_MIN_AVLC_QUALITY) ||
                 (OBJ_LONG_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTrace)) <
                  CP_MAX_TRACE_DIST_HIGH_VAR)) &&
             (OBJ_PROBABILITY_OF_EXIST(FIP_STATIC_TRACE_GET_VLC_ID(iTrace)) >
              CP_MIN_PEX_FOR_VALID_TRACE) &&
             ((FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTrace) >
               PARALLELISM_MIN_LENGTH_TRACE) /*minimal Length of Trace*/
              || (FIP_STATIC_TRACE_REACHED_EGO_VEH(iTrace))))
                ? SI_TRACE_QUAL_MASK_TRACE_VALID
                : 0u;

        pTrace_Params->Trace_EGOPara[iTrace] =
            CPFusionTraceCalcTraceCircleParallelism(iTrace, pCourseData->fCurve,
                                                    &MOT2EGO);
        Trace_EgoValid[iTrace] =
            ((pTrace_Params->Trace_EGOPara[iTrace].fDistMean <
              fLeftThreshold) && /*within carriageway */
             (pTrace_Params->Trace_EGOPara[iTrace].fDistMean >
              fRightThreshold) && /*within carriageway */
             (pTrace_Params->Trace_EGOPara[iTrace].fDistStdDev <
              PARALLELISM_THRESH) /*parallelism*/
             )
                ? TRUE
                : FALSE;
        if ((pTrace_Params->Trace_QualityValid[iTrace] > 0u) &&
            (Trace_EgoValid[iTrace])) {
            SET_BIT(pTrace_Params->Trace_QualityValid[iTrace],
                    SI_TRACE_QUAL_MASK_EGO_PARALLEL);
        }

        /* Evaluate each trace if it is suitable for computation of a trace
         * trajectory and store result */
        CPSetValidityOfTraceForFusion(
            iTrace, pTrace_Params->Trace_QualityValid[iTrace]);
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalculateTraceSamples */
void CPCalculateTraceSamples(const CPTraceTrajectory_t *pTraceTrajectory,
                             CPPosSamples_t *pTRACE_PosSamples) {
    CPCourseData_t TraceCourseData;

    /* Initialize trace samples */
    CPInitPosSamples(pTRACE_PosSamples);

    /* Get trace estimation in correct format */
    TraceCourseData = CPGetTraceEstimationAsCourseData(pTraceTrajectory);

    /* Sample the trace trajectory */
    CPSamplePosClothApprox(&TraceCourseData, CPClothApproxType_PolynomialOnly,
                           CP_SAMPLEDIST_MAX, pTRACE_PosSamples);

    CPMoveSamplesFromCoGToSensor(pTRACE_PosSamples);

    CPLimitSamplesXDist(pTraceTrajectory->fMaxDist, pTRACE_PosSamples);

    /* Increase the number of samples to a minimum length of
       MIN_FUSION_TRACE_POS_SAMPLES
       if the samples were reasonable good so far. */
    /* if(pTraceTrajectory->fMeanSquaredError < 0.1f)
    {
    pTRACE_PosSamples->nb_samples = MAX(pTRACE_PosSamples->nb_samples,10);
    }*/
    pTRACE_PosSamples->nb_samples =
        MIN(pTRACE_PosSamples->nb_samples, MAX_NB_TRAJ_SAMPLES);
}

/*************************************************************************************************************************
  Functionname:    CPFusionEvalTraceSamples */
void CPFusionEvalTraceSamples(const CPTraceTrajectory_t *pTraceTrajectory,
                              CPPosSamples_t *pTRACE_PosSamples,
                              CPTrajectoryData_t *pTrajData) {
    boolean bIsGoodTraceTrajectory, bFusTraceSituationCriterion;

    if ((pTraceTrajectory->fMaxDist > CP_MIN_TRACE_LENGTH) &&
        (pTraceTrajectory->fMeanSquaredError < CP_MAX_TRACE_MSE)) {
        bIsGoodTraceTrajectory = TRUE;
    } else {
        bIsGoodTraceTrajectory = FALSE;
    }

    bFusTraceSituationCriterion =
        CPFusionTraceIsFusionSituation(&pTrajData->State);

    if ((bIsGoodTraceTrajectory != FALSE) &&
        (bFusTraceSituationCriterion != FALSE) &&
        (pTraceTrajectory->fWeight < CP_MAX_DEVIATION_SAMPLE_TO_HYPO)) {
        pTrajData->State.FusionTraces = TRUE;
        pTrajData->State.EgoCourseOnly = FALSE;

        /* Set moving objects fusion */
        pTrajData->fMovObjFusRange = pTraceTrajectory->fMaxDist;

        /* Set MSE of trace trajectory */
        pTrajData->fMSETrace = pTraceTrajectory->fMeanSquaredError;

        /* Remember for the next cycle, that trace fusion was active */
        pTrajData->bTraceFusionActiveLastCycle = TRUE;

    } else {
        /* We are not going to use traces. Therefore, reset the trace trajectory
         * again */
        CPInitPosSamples(pTRACE_PosSamples);

        /* Remember for the next cycle, that trace fusion was inactive */
        pTrajData->bTraceFusionActiveLastCycle = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalculateCombinedTraceTrajectory */
void CPCalculateCombinedTraceTrajectory(CPTraceTrajectory_t *pTraceTrajectory,
                                        const CPCourseData_t *pCourseData,
                                        const CPTrajectoryData_t *pTrajData) {
    /*low-pass filter variables*/
    static float32 fTraceCurve = 0.0f;
    const float32 fAlpha = 0.3f;

    /*trace trajectory variables*/
    uint32 iTr, iIter;
    uint32 uNumUsedTraces = 0u;

    float32 fRadiusMin, f_aDist;
    float32 fMaxDistTrace = 0.f;
    float32 xPos = 0.f;

    /* local variables required for PSO hypothesis testing */
    float32 f_radius_hyp[CP_PSO_NUM_HYPOTHESIS];
    float32 f_local_best_rad[CP_PSO_NUM_HYPOTHESIS];
    float32 f_local_best_fit[CP_PSO_NUM_HYPOTHESIS];
    float32 f_vel_hyp[CP_PSO_NUM_HYPOTHESIS];

    float32 f_global_best_rad;
    float32 f_global_best_fit;

    /* Init all PSO related variables */
    /* Calculate the min radius for the trace trajectory. The minimum radius is
       calculated for a given ego-velocity and an assumed maximum lateral
       acceleration of 3 m/s^2. For more narrow curve radii the lateral
       acceleration would be beyond ACC specs, thus we can ignore them. */
    fRadiusMin =
        MAX_FLOAT(SQR(EGO_SPEED_X_OBJ_SYNC) / LAT_ACCEL_MIN_RADIUS, RADIUS_MIN);

    /* First hypothesis based on minimum radius based on lateral acceleration */
    f_radius_hyp[0u] = BML_f_Sign(pTrajData->LastCycle.fTrajC0) * fRadiusMin;

    /* Second hypothesis based on VDY trajectory */
    f_radius_hyp[1u] = (fABS(pCourseData->fCurve) > CP_TRACE_CURVATURE_MIN)
                           ? (1.f / pCourseData->fCurve)
                           : CP_TRACE_RADIUS_MAX;

    /* Third hypothesis based on ACC trajectory */
    f_radius_hyp[2u] =
        (fABS(pTrajData->LastCycle.fTrajC0) > CP_TRACE_CURVATURE_MIN)
            ? (1.f / pTrajData->LastCycle.fTrajC0)
            : CP_TRACE_RADIUS_MAX;

    /* Initialize local best hypotheses with the given hypotheses */
    for (iTr = 0; iTr < CP_PSO_NUM_HYPOTHESIS; iTr++) {
        f_local_best_rad[iTr] = f_radius_hyp[iTr];
        f_local_best_fit[iTr] = CP_TRACE_PSO_INIT_FIT;
        f_vel_hyp[iTr] = 10.f;
    }

    /* Use as a global best starting position, radius of previously calculated
     * trace trajectory  */
    if ((pTrajData->bTraceFusionActiveLastCycle != FALSE) &&
        (fABS(fTraceCurve) > CP_TRACE_CURVATURE_MIN)) {
        f_global_best_rad = 1.f / fTraceCurve;
    } else if (fABS(pTrajData->LastCycle.fTrajC0) > CP_TRACE_CURVATURE_MIN) {
        f_global_best_rad = 1.f / pTrajData->LastCycle.fTrajC0;
    } else {
        f_global_best_rad = CP_TRACE_RADIUS_MAX;
    }

    /*go through all traces*/
    for (iTr = 0; iTr < TRACE_NO_OF_TRACES; iTr++) {
        if (CPTraceAdd[iTr].bUseTraceForFusion != FALSE) {
            /* save the max distance from valid trace */
            xPos = OBJ_LONG_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTr));
            if (xPos > fMaxDistTrace) {
                fMaxDistTrace = xPos;
            }

            uNumUsedTraces++;
        }
    }

    /* calculate fitness value for global best radius hypothesis */
    f_global_best_fit = CPCalculateAvgDevTrace(f_global_best_rad);

    if (uNumUsedTraces > 0u) {
        /* find best initial hypothesis */
        for (iTr = 0u; iTr < CP_PSO_NUM_HYPOTHESIS; iTr++) {
            f_local_best_fit[iTr] = CPCalculateAvgDevTrace(f_radius_hyp[iTr]);

            /* if the local best hypothesis is better compared to the global
               best hypothesis, then replace global best with local best
               hypothesis */
            if (f_local_best_fit[iTr] < f_global_best_fit) {
                f_global_best_fit = f_local_best_fit[iTr];
                f_global_best_rad = f_radius_hyp[iTr];
            }

            f_radius_hyp[iTr] += (-1.f * BML_f_Mod((float32)iTr + 1.f, 2.f)) *
                                 CP_TRACE_PSO_CHANGE_RADIUS;
        }

        for (iIter = 0; iIter < CP_TRACE_PSO_ITER_MAX; iIter++) {
            for (iTr = 0; iTr < CP_PSO_NUM_HYPOTHESIS; iTr++) {
                /* Compared to conventional PSO no random numbers are used.
                   Although this is not optimum, the good initial hypothesis
                   can compensate the disadvantage */
                /* For details, refer to van den Bergh, F. (2001). An Analysis
                   of Particle Swarm Optimizers (PhD thesis). University of
                   Pretoria, Faculty of Natural and Agricultural Science. */
                f_vel_hyp[iTr] = CP_TRACE_PSO_XI *
                                 (f_vel_hyp[iTr] +
                                  (CP_TRACE_PSO_C1 * (f_local_best_rad[iTr] -
                                                      f_radius_hyp[iTr])) +
                                  (CP_TRACE_PSO_C2 *
                                   (f_global_best_rad - f_radius_hyp[iTr])));
                f_radius_hyp[iTr] += f_vel_hyp[iTr];

                f_aDist = CPCalculateAvgDevTrace(f_radius_hyp[iTr]);
                /* In case the current hypothesis is better than any other
                   hypothesis of this particle, we replace the local best
                   hypothesis */
                if (f_aDist < f_local_best_fit[iTr]) {
                    f_local_best_fit[iTr] = f_aDist;
                    f_local_best_rad[iTr] = f_radius_hyp[iTr];
                }

                /* In case the current hypothesis is better than any other
                   hypothesis of the overall swarm, we replace the global best
                   hypothesis */
                if (f_aDist < f_global_best_fit) {
                    f_global_best_fit = f_aDist;
                    f_global_best_rad = f_radius_hyp[iTr];
                }
            }
        }
        pTraceTrajectory->fCurve = (fABS(f_global_best_rad) > BML_f_Delta)
                                       ? (1.f / f_global_best_rad)
                                       : (0.f);

        /* If the max distance of the traces is over the threshold ->
         * extrapolate to the maximum distance / if not -> no extrapolation*/
        pTraceTrajectory->fMaxDist =
            (fMaxDistTrace < EXTRAPOLATION_THRES) ? fMaxDistTrace : 200.f;

        /*set the weight for the trace trajectory*/
        pTraceTrajectory->fMeanSquaredError = 1.f;
        pTraceTrajectory->fWeight = f_global_best_fit;
    } else {
        pTraceTrajectory->fCurve = 0.f;
        pTraceTrajectory->fMaxDist = 0.f;
        pTraceTrajectory->fMeanSquaredError = CP_TRAJ_INVALID_VALUE;
        pTraceTrajectory->fWeight = f_global_best_fit;
    }

    /* Apply low-pass filter for trace trajectory */
    if (pTrajData->bTraceFusionActiveLastCycle != FALSE) {
        fTraceCurve = ((1.0f - fAlpha) * fTraceCurve) +
                      (fAlpha * pTraceTrajectory->fCurve);
        pTraceTrajectory->fCurve = fTraceCurve;
    } else {
        fTraceCurve = pTraceTrajectory->fCurve;
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalculateAvgDevTrace */
static float32 CPCalculateAvgDevTrace(float32 fRadius) {
    uint8 iTr;
    uint32 iSamplesAll;
    sint16 iS;

    float32 f_aDist, f_aDistPrev;
    float32 f_Curve2TraceDev = 0.f;
    float32 f_xPos, f_yPos;

    f_aDist = 0.f;
    f_aDistPrev = 0.f;
    iSamplesAll = 0;

    for (iTr = 0; iTr < FIP_STATIC_TRACE_NO_OF_TRACES; iTr++) {
        if (CPTraceAdd[iTr].bUseTraceForFusion != FALSE) {
            for (iS = -1; iS < FIP_STATIC_TRACE_GET_NO_OF_POINTS(iTr); iS++) {
                if (iS == -1) {
                    /*take the object position as first trace sample*/
                    f_xPos =
                        OBJ_LONG_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTr));
                    f_yPos =
                        OBJ_LAT_DISPLACEMENT(FIP_STATIC_TRACE_GET_VLC_ID(iTr));
                } else {
                    /*copy the trace samples into the sample structure*/
                    f_xPos = FIP_STATIC_TRACE_GET_X(iTr)[iS];
                    f_yPos = FIP_STATIC_TRACE_GET_Y(iTr)[iS];
                }

                /*calculate the orthogonal distance between trajectory and
                 * sample point*/
                if (SQRT(SQR(f_xPos) + SQR(fRadius - f_yPos)) >
                    BML_f_AlmostZero) {
                    f_aDist =
                        fRadius - (BML_f_Sign(fRadius) *
                                   (SQRT(SQR(f_xPos) + SQR(fRadius - f_yPos))));
                } else {
                    f_aDist = 0.f;
                }

                if (iS == -1) {
                    /* save current orthogonal distance to calculate deviation
                     * with next sample */
                    f_aDistPrev = f_aDist;
                } else {
                    /* Calculate the orthogonal distance deviation between
                     * current and previous sample */
                    f_Curve2TraceDev += fABS(f_aDist - f_aDistPrev);
                    /* Save current sample as previous sample for the next
                     * iteration */
                    f_aDistPrev = f_aDist;

                    iSamplesAll++;
                }
            }
        }
    }

    if (iSamplesAll > 0) {
        /*calculate the average of the distance deviation*/
        f_Curve2TraceDev = f_Curve2TraceDev / (float32)iSamplesAll;
    }

    return f_Curve2TraceDev;
}

/*************************************************************************************************************************
  Functionname:    CPSetValidityOfTraceForFusion */
static void CPSetValidityOfTraceForFusion(TraceID_t iTrace,
                                          uint8 ui_TraceQualityFlag) {
    boolean b_UseTraceForFusion = FALSE;

    if ((CPTraceAdd[iTrace].iObjNr != -1) && (ui_TraceQualityFlag > 0u) &&
        ((2.0f * CPTraceAdd[iTrace].ApproxPoly.fC2 *
          SQR(EGO_SPEED_X_OBJ_SYNC)) < CP_TRACE_MAX_PLAUSIBLE_LAT_ACCEL)) {
        /* check if we can follow the trace with ay */
        b_UseTraceForFusion = TRUE;
    }

    CPTraceAdd[iTrace].bUseTraceForFusion = b_UseTraceForFusion;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */