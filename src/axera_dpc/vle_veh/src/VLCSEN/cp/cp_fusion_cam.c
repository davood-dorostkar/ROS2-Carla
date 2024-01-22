/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp.h"
#include "cp_par.h"
#include "cp_kalman.h"
#include "fip_ext.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/* Internal constant for minimal C0 delta, where second order equation for
lane middle intersection is solved. (Note: has to be non-zero to prevent
division by zero */
#define CP_MIN_C0_DELTA_LANE_MID 1e-8f

#define CP_DRV_INT_CURVE_TIME 30.0f

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

void CPFusionCamMain(CPTrajectoryData_t *pTrajData);

static boolean CPFusionCamSituationVote(CPTrajectoryData_t *pTrajData);
static void CPPosSampleLaneMarkers(const float32 fEgo2CamX,
                                   const float32 fCamBaseY,
                                   const float32 fCamLaneFusionTimer,
                                   CPPosSamples_t *pSamples);
static float32 CPGetCamLaneMarker(const t_CamLaneInputData *pCamLane,
                                  const float32 fMinYDist,
                                  const float32 fMaxYDist,
                                  const float32 fDefaultRetY);
/* CPRESTRUCTED: Rename: CPTrajCamFusion -> CPFusionCamMain */
/*************************************************************************************************************************
  Functionname:    CPFusionCamMain */
void CPFusionCamMain(CPTrajectoryData_t *pTrajData) {
    CPNoiseModelLinear_t ConstantNoise;

    /* Do deep pass filtering of driver intended curvature */
    pTrajData->fFilteredDIC = GDB_FILTER(
        EGO_DRV_INT_CURVE_RAW, pTrajData->fFilteredDIC, CP_DRV_INT_CURVE_TIME);
    /* Check if situation is useful for camera lane fusion */
    if (CPFusionCamSituationVote(pTrajData)) {
        CPPosSamples_t LaneMarkerSamples;

        CPPosSampleLaneMarkers(pTrajData->fEgo2CamDistX, pTrajData->fBaseY,
                               pTrajData->fCamLaneFusionTimer,
                               &LaneMarkerSamples);
        /* CPRECONSTRUCTED: Extended Function Call of CPPosUpdateTrajectory by
         * Noise Model */
        /* CPPosUpdateTrajectory(&LaneMarkerSamples, pTrajData); */

        ConstantNoise.fNoiseOffset = CP_KALMAN_POSAMPLES_NOISE_OFFSET;
        ConstantNoise.fNoiseGradient = 0.f;

        CPKalmanUpdatePos(&LaneMarkerSamples, ConstantNoise, pTrajData);

        /* Update state settings: ego only off/camera fusion on */
        pTrajData->State.CamLaneFusion = TRUE;
        pTrajData->State.EgoCourseOnly = FALSE;

    } else {
        /* Not a cam lane fusion situation: set cam lane fusion state to off */
        pTrajData->State.CamLaneFusion = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    CPGetCamLaneMarker */
static float32 CPGetCamLaneMarker(const t_CamLaneInputData *pCamLane,
                                  const float32 fMinYDist,
                                  const float32 fMaxYDist,
                                  const float32 fDefaultRetY) {
    t_CamLaneMarkerEnum i;
    float32 fRet = fDefaultRetY;
    /* Go through the up to four lane markers and find matching lane markers */
    for (i = CL_CAM_LANE_MK_ADJ_LEFT; (i <= CL_CAM_LANE_MK_ADJ_RIGHT); i++) {
        /* First condition: lane marker state has to be valid */
        if ((pCamLane->LaneMarkerInfo[i].u_ExistanceProbability >=
             FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
            /* Second condition: the marker's distance has to be within
             * [fMinYDist .. fMaxYDist] */
            const float32 fCurDist = pCamLane->LaneMarkerInfo[i].f_MarkerDist;
            if ((fCurDist >= fMinYDist) && (fCurDist <= fMaxYDist)) {
                /* Set return value to current distance */
                fRet = fCurDist;
            }
        }
    }
    return fRet;
}

static void CPSecondOrderEquationSolve(float32 fTargetYDispl,
                                       float32 f_CamCurve,
                                       float32 f_CamAngle,
                                       float32 *p_fEgo2CamX,
                                       boolean *p_bRet) {
    float32 fTmp, x_fact, fC0Delta;
    x_fact = SIN_(f_CamAngle);
    fC0Delta = (f_CamCurve - EGO_CURVE_OBJ_SYNC);
    /* Second order equation: fC0Delta/2 * x^2+ x_fact * x + y_offs = 0
        Verify that really a second order equation */
    if ((fC0Delta > CP_MIN_C0_DELTA_LANE_MID) ||
        (fC0Delta < (-CP_MIN_C0_DELTA_LANE_MID))) {
        /* Based on equation the sub-part (b^2-4ac): correct with a tolerance */
        if (fC0Delta > 0.f) {
            fTmp = (SQR(x_fact) -
                    (2.0f * fC0Delta *
                     (fTargetYDispl - CP_CAM_LANE_FUSION_EGO_LANE_TOLERANCE)));
        } else {
            fTmp = (SQR(x_fact) -
                    (2.0f * fC0Delta *
                     (fTargetYDispl + CP_CAM_LANE_FUSION_EGO_LANE_TOLERANCE)));
        }

        if (fTmp >= 0.f) {
            float32 fX1, fX2;
            /* Take square root of b^2-4ac */
            fTmp = SQRT_(fTmp);
            /* Solution #1 : (-b - sqrt(b^2-4ac))/(2a) */
            fX1 = (-x_fact - fTmp) / fC0Delta;
            /* Solution #2 : (-b + sqrt(b^2-4ac))/(2a) */
            fX2 = (-x_fact + fTmp) / fC0Delta;
            /* Choose smallest positive solution */
            if (fX1 >= 0.f) {
                if (fX2 >= 0.f) {
                    /* Both positive: take smaller of two */
                    *p_fEgo2CamX = MIN(fX1, fX2);
                } else {
                    /* Use X1 as that is positive */
                    *p_fEgo2CamX = fX1;
                }
            } else {
                /* Note: fX2 may be negative as well, but that is caught
                 * afterwards */
                *p_fEgo2CamX = fX2;
            }
        } else {
            /* 2nd degree equation not solveable. This can happen when the yaw
            rate already swings back for stabilization in the target lane.
            Verify that this is the case (i.e.: target Y displacement small),
            then fuse samples from X=0, otherwise having ego samples take over
          */
            if (fABS(fTargetYDispl) < CP_PAR_LC_MIN_MARKER_DIST) {
                *p_fEgo2CamX = EGO_SPEED_X_OBJ_SYNC;
            } else {
                *p_fEgo2CamX = CP_SAMPLEDIST_MAX;
                *p_bRet = FALSE;
            }
        }
    } else {
        /* First order equation, since fC0Delta is practically zero */
        if ((x_fact > C_F32_DELTA) || (x_fact < (-C_F32_DELTA))) {
            *p_fEgo2CamX = (-fTargetYDispl / x_fact);
        } else {
            *p_fEgo2CamX = CP_SAMPLEDIST_MAX;
            *p_bRet = FALSE;
        }
    }

    /* Verify that positive definite result found */
    if (*p_fEgo2CamX < 0.f) {
        /* Bad result: the two parabolas meet in negative X only */
        *p_fEgo2CamX = CP_SAMPLEDIST_MAX;
        *p_bRet = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    CPFusionCamSituationVote */
static boolean CPFusionCamSituationVote(CPTrajectoryData_t *pTrajData) {
    boolean bRet = FALSE;
    float32 fMinCamDistX;
    float32 fMaxCamAngle;
    float32 fTargetYDispl = 0.f;

    float32 f_CamLaneVisibilityDist, f_CamLaneC0, f_CamLaneAngle;

    /* Determine minumum necessary camera lane foresight for camera fusion */
    fMinCamDistX = SIGetMovingObjPickupRange();
    fMinCamDistX *= CP_PAR_MIN_CAM_LANE_FUS_DIST_RATIO;
    /* Set maximum camera lane angle */
    fMaxCamAngle = CP_PAR_MAX_CAM_LANE_ANGLE;
    /* If camera lane quality already high, then do hysteriesis */
    if (pTrajData->State.CamLaneQualityHigh) {
        fMinCamDistX *= CP_PAR_CAM_LANE_FUSION_HYST;
        fMaxCamAngle *= (1.f / CP_PAR_CAM_LANE_FUSION_HYST);
    }
    fMinCamDistX =
        MAX_FLOAT(fMinCamDistX, C_F32_DELTA); /*!< To ensure that minimal camera
                                                 lane distance is above zeros */

    /* Verify that camera lane minimal visibility distance
    criteria satisfied and seen camera curvature not too large (in large
    curvature situations
    the error of the camera seems to increase) and the seen camera lane angle
    shall not exceed
    a given threshold (when high angles are reported, camera lane seems to be
    completely bananas) */

    f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane();  /*!< Visibility distance of camera
                                              lane */
    f_CamLaneC0 = FIP_f_GetCurveCamLane(); /*!< Curvature of camera lane */
    f_CamLaneAngle =
        FIP_f_GetHeadingAngleCamLane(); /*!< Heading angle of camera lane */
    if ((f_CamLaneVisibilityDist > fMinCamDistX) &&
        (fABS(f_CamLaneC0) < CP_PAR_MAX_CAM_CURVATURE) &&
        (fABS(f_CamLaneAngle) < fMaxCamAngle)) {
        sint16 SILaneChangeProb = SILCGetLaneChangeProbability();

        /* Set camera quality bit */
        pTrajData->State.CamLaneQualityHigh = TRUE;

        /* Use camera lane markers in special lane change situations only */
        if (ABS(SILaneChangeProb) > CP_PAR_DIM_LANE_CHANGE_PROB_MIN) {
            /* Get center lane marker : it's the one thats
             * [-CP_PAR_LC_MIN_MARKER_DIST .. CP_PAR_LC_MIN_MARKER_DIST] far
             * away */
            const float32 fCenterMarker = CPGetCamLaneMarker(
                VLCSEN_pCamLaneData, -CP_PAR_LC_MIN_MARKER_DIST,
                CP_PAR_LC_MIN_MARKER_DIST, 0.f);
            /* Determine target Y displacement of lane change maneuver */
            if (SILaneChangeProb > 0) {
                /* Lane change to the left, evaluate lane marker(s) to our left
                 */
                /* Get far left lane marker : it's the one thats
                 * [CP_PAR_LC_MIN_MARKER_DIST  .. CP_PAR_LC_MAX_MARKER_DIST] far
                 * away */
                const float32 fFarLeftMarker = CPGetCamLaneMarker(
                    VLCSEN_pCamLaneData, CP_PAR_LC_MIN_MARKER_DIST,
                    CP_PAR_LC_MAX_MARKER_DIST,
                    (fCenterMarker + (2.f * CP_PAR_DEFAULT_LC_MARKER_OFFSET)));
                /* Create combined target Y displacement */
                fTargetYDispl = 0.5f * (fCenterMarker + fFarLeftMarker);
                /* Second activation criteria: high driver activity on steering
                wheel (as when high steering wheel
                dynamics are involved, then the current vehicle yaw rate is a
                bad predictor of the future path) */
                if (OBJ_GET_RELEVANT_OBJ_NR == OBJ_INDEX_NO_OBJECT) {
                    pTrajData->fCamLaneFusionTimer =
                        CP_PAR_CAM_LANE_CHANGE_ACTIVATION_TIME;
                }
            } else {
                /* Lane change to the right, evaluate lane marker(s) to our
                 * right */
                /* Get far right lane marker : it's the one thats
                 * [-CP_PAR_LC_MAX_MARKER_DIST .. - CP_PAR_LC_MIN_MARKER_DIST]
                 * far away */
                const float32 fFarRightMarker = CPGetCamLaneMarker(
                    VLCSEN_pCamLaneData, -CP_PAR_LC_MAX_MARKER_DIST,
                    -CP_PAR_LC_MIN_MARKER_DIST,
                    (fCenterMarker + (-2.f * CP_PAR_DEFAULT_LC_MARKER_OFFSET)));
                /* Create combined target Y displacement */
                fTargetYDispl = 0.5f * (fCenterMarker + fFarRightMarker);
                /* Second activation criteria: high driver activity on steering
                wheel (as when high steering wheel
                dynamics are involved, then the current vehicle yaw rate is a
                bad predictor of the future path) */
                if (OBJ_GET_RELEVANT_OBJ_NR == OBJ_INDEX_NO_OBJECT) {
                    pTrajData->fCamLaneFusionTimer =
                        CP_PAR_CAM_LANE_CHANGE_ACTIVATION_TIME;
                }
            }
        } else {
            /* Get far left lane marker : it's the one thats
             * [CP_PAR_NO_LC_MIN_MARKER_DIST .. CP_PAR_NO_LC_MAX_MARKER_DIST]
             * far away */
            const float32 fFarLeftMarker = CPGetCamLaneMarker(
                VLCSEN_pCamLaneData, CP_PAR_NO_LC_MIN_MARKER_DIST,
                CP_PAR_NO_LC_MAX_MARKER_DIST,
                (CP_PAR_DEFAULT_LC_MARKER_OFFSET));
            /* Get far right lane marker : it's the one thats
             * [-CP_PAR_NO_LC_MAX_MARKER_DIST .. -CP_PAR_NO_LC_MIN_MARKER_DIST]
             * far away */
            const float32 fFarRightMarker = CPGetCamLaneMarker(
                VLCSEN_pCamLaneData, -CP_PAR_NO_LC_MAX_MARKER_DIST,
                -CP_PAR_NO_LC_MIN_MARKER_DIST,
                (-CP_PAR_DEFAULT_LC_MARKER_OFFSET));
            /* Verify that the thus gotten lane not too wide */
            if ((fFarLeftMarker - fFarRightMarker) <
                CP_PAR_NO_LC_MAX_LANEWIDTH) {
                /* Target Y displacement is middle of the two above */
                fTargetYDispl = 0.5f * (fFarLeftMarker + fFarRightMarker);
            } else {
                /* Unplausible lane markers for lane change model */
                fTargetYDispl = 0.f;
                /* Reset fusion timer */
                pTrajData->fCamLaneFusionTimer = 0.f;
            }
        }
        /* Store the target Y displacment (goal lane middle) */
        pTrajData->fBaseY = fTargetYDispl;

        bRet = TRUE;

    } else {
        /* Camera lane data not available or visibility distance too small */
        bRet = FALSE;

        /* Clear camera quality bit */
        pTrajData->State.CamLaneQualityHigh = FALSE;
    }

    /* Update camera lane fusion timer active */
    if (pTrajData->fCamLaneFusionTimer > 0.f) {
        pTrajData->fCamLaneFusionTimer -= CP_CYCLE_TIME;
    }

    /* Assuming the ego course can be approximated as 0.5 * (1/R) * x^2 then the
    x coordinate
    where the camera target lane and the ego course Y meet is
    SQRT(2*fTargetYDispl*R) */
    if ((fABS(EGO_CURVE_OBJ_SYNC) < 1e-2f) &&
        (EGO_SPEED_X_OBJ_SYNC > CP_PAR_MIN_CAM_LANE_FUS_SPEED) &&
        (EGO_SPEED_X_OBJ_SYNC < CP_PAR_MAX_CAM_LANE_FUS_SPEED) &&
        (bRet == TRUE)) {
        float32 f_CamCurve, f_CamAngle;
        float32 fEgo2CamX;

        /* Get the camera lane information */
        f_CamCurve = FIP_f_GetCurveCamLane();
        f_CamAngle = FIP_f_GetHeadingAngleCamLane();

        CPSecondOrderEquationSolve(fTargetYDispl, f_CamCurve, f_CamAngle,
                                   &fEgo2CamX, &bRet);

        /* Store ego 2 camera lane blending distance */
        pTrajData->fEgo2CamDistX = fEgo2CamX;
    } else {
        /* Curvature too large or ego speed too large : do not use camera lane
         */
        pTrajData->fEgo2CamDistX = CP_SAMPLEDIST_MAX;
        bRet = FALSE;
    }

    return bRet;
}

/*************************************************************************************************************************
  Functionname:    CPPosSampleLaneMarkers */
static void CPPosSampleLaneMarkers(const float32 fEgo2CamX,
                                   const float32 fCamBaseY,
                                   const float32 fCamLaneFusionTimer,
                                   CPPosSamples_t *pSamples) {
    uint32 i;
    float32 y_offs;
    float32 x_ego_cam; /*!< X coordinate of point, where ego samples blend over
                     to camera
                     samples */
    float32 f_CamCurve;
    float32 f_CamAngle;
    float32 f_CamLength;
    float32 f_CamCurvatureChange;

    /* Get the camera lane information */
    f_CamAngle = FIP_f_GetHeadingAngleCamLane();
    f_CamLength = FIP_f_GetVisibilityDistCamLane();
    f_CamCurve = FIP_f_GetCurveCamLane();
    f_CamCurvatureChange = FIP_f_GetCurvatureChangeCamLane();

    /* Set X where blending over between ego to camera lane shall take place */
    x_ego_cam = fEgo2CamX;

    /* Set as Y offset the middle of the lane (if available) */
    y_offs = fCamBaseY;

    /* Assuming the ego course can be approximated as 0.5 * (1/R) * x^2 then the
    x coordinate
    where the camera target lane and the ego course Y meet is
    SQRT(2*fTargetYDispl*R) */
    if (fABS(EGO_CURVE_OBJ_SYNC) < 1e-2f) {
        float32 x_s = 0.0f;
        float32 x_inc;
        float32 y_s;

        /* Use as long as the markers are visible */
        x_inc = ((f_CamLength) * (1.f / (float32)MAX_NB_TRAJ_SAMPLES));

        for (i = 0u; i < MAX_NB_TRAJ_SAMPLES; i++) {
            /*generate MAX_NB_TRAJ_SAMPLES for RW_VLC_MAX m along Course*/
            x_s += x_inc;
            pSamples->fx[i] = x_s;
            if (fCamLaneFusionTimer > C_F32_DELTA) {
                if (x_s < x_ego_cam) {
                    /* Use ego data */
                    y_s = 0.5f * EGO_CURVE_OBJ_SYNC * SQR(x_s);
                } else {
                    /* Use camera lane */
                    y_s = y_offs + (TAN_HD_(f_CamAngle)) * x_s +
                          (0.5f * x_s * x_s * f_CamCurve) +
                          (C_SIXTH * x_s * x_s * x_s * f_CamCurvatureChange);
                }
            } else {
                y_s = (TAN_HD_(f_CamAngle)) * x_s +
                      (0.5f * x_s * x_s * f_CamCurve) +
                      (C_SIXTH * x_s * x_s * x_s * f_CamCurvatureChange);
            }
            pSamples->fy[i] = y_s;
        }
        pSamples->nb_samples = MAX_NB_TRAJ_SAMPLES;
    } else {
        CPCourseData_t CourseData_Temp;

        CourseData_Temp.fCurve = EGO_CURVE_OBJ_SYNC;
        CourseData_Temp.fCurveGradient = 0.f;
        CourseData_Temp.fCurveVar = 0.f;
        CourseData_Temp.SideSlipAngle = 0.f;
        CourseData_Temp.SideSlipAngleVariance = 0.f;

        CPSamplePosClothApprox(&CourseData_Temp, CPClothApproxType_CircleOnly,
                               CP_SAMPLEDIST_MAX, pSamples);
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */