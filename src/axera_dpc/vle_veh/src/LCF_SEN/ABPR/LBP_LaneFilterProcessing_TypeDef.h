/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_TYPEDEF_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_TYPEDEF_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "tue_common_libs.h"

/* Ego lane detection state */
#define LANE_NO_VALID 0      /* Both lane lines are invalid */
#define LANE_RIGHT_VALID 1   /* The right lane is valid */
#define LANE_LEFT_VALID 2    /* The left lane line is valid */
#define LANE_RIGHT_VIRTUAL 3 /* The right lane is virtual */
#define LANE_LEFT_VIRTUAL 4  /* The left lane is virtual */
#define LANE_BOTH_VALID 5    /* Both lane lines are valid */

/* Kalman filter status*/
#define KF_STATE_INVALID 0     /* Invalid state */
#define KF_STATE_FULL_UPDATE 1 /* Valid, full update */
#define KF_STATE_DEGR_UPDATE 2 /* Valid, degraded update */
#define KF_STATE_PREDICTION 3  /* Valid, prediction */
#define KF_STATE_INIT 4        /* Valid, init, */
#define KF_STATE_RESET 5       /* Invalid, reset */

/* Camera type */
#define CFG_CAMERA_MINIEYE 0   /* MiniEye */
#define CFG_CAMERA_SENSETIME 1 /* SenseTime */

/*****************************3.Lane filtering and
 * plausibilization***********************/
#ifndef Rte_TypeDef_sLWVInput_t
#define Rte_TypeDef_sLWVInput_t
typedef struct {
    REAL32_T
    fVehVelX; /* Vehicle speed based on the wheel speeds , (unit, km/h) */
    UINT8_T bNewCorridorValid;
    UINT8_T bUpDownHillDegrade;
    UINT8_T bBridgePossible;

    REAL32_T fPosY0Lf;
    REAL32_T fHeadingLf;
    UINT8_T uQualityLf;
    UINT16_T uRangeCheckQualifierLf;
    UINT8_T bNotAvailableLf;
    UINT8_T bDistYStepDtctLf;
    UINT8_T bLengthInvalidLf;
    UINT8_T bBridgeUnCplLf;
    UINT8_T bQualityNotValidLf;
    REAL32_T fPosY0Ri;
    REAL32_T fHeadingRi;
    UINT8_T uQualityRi;
    UINT16_T uRangeCheckQualifierRi;
    UINT8_T bNotAvailableRi;
    UINT8_T bDistYStepDtctRi;
    UINT8_T bLengthInvalidRi;
    UINT8_T bBridgeUnCplRi;
    UINT8_T bQualityNotValidRi;
    UINT8_T bLaneChangeDtct;
} sLWVInput_t;
#endif

#ifndef Rte_TypeDef_sLWVParam_t
#define Rte_TypeDef_sLWVParam_t
typedef struct {
    UINT16_T uBitMaskRangeCheckLf; /* 341  '101010101' */
    UINT16_T uBitMaskRangeCheckRi; /* 170  '010101010' */
    REAL32_T fTiBridgeByVirtual; /* Defines the turn on delay time a LD virtual
                                    line is used for bridging */
    REAL32_T fMaxBridgeDistance;
    REAL32_T fMaxBridgeTime;
    REAL32_T fDefaultLaneWidth;
    REAL32_T fLowPassTimeConst;
    REAL32_T fMaxLaneWidth;
    REAL32_T fMaxLaneWidthHyst;
    REAL32_T fMinLaneWidth;
    REAL32_T fMinLaneWidthHyst;
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(unit, s) */
} sLWVParam_t;
#endif

#ifndef Rte_TypeDef_sLWVOutput_t
#define Rte_TypeDef_sLWVOutput_t
typedef struct {
    UINT8_T uLaneValidQualifer;
    REAL32_T fLaneWidth;
    UINT8_T bLaneWidthValid;
} sLWVOutput_t;
#endif

#ifndef Rte_TypeDef_sLWVDebug_t
#define Rte_TypeDef_sLWVDebug_t
typedef struct {
    UINT8_T bValidByRangeLf;
    UINT8_T bLaneValidLf;
    UINT8_T bValidByRangeRi;
    UINT8_T bLaneValidRi;
    UINT8_T bLaneVirtualLf;
    UINT8_T bLaneVirtualRi;
    UINT8_T bBridgeByVirtualLf;
    UINT8_T bBridgeByVirtualRi;
    UINT8_T bRawLaneBridgeLf;
    UINT8_T bRawLaneBridgeRi;
    REAL32_T fBridgeDistanceLf;
    REAL32_T fBridgeDistanceRi;
    UINT8_T bEnableByDistanceLf;
    UINT8_T bEnableByDistanceRi;
    REAL32_T fBridgeTimeLf;
    REAL32_T fBridgeTimeRi;
    UINT8_T bEnableByTimeLf;
    UINT8_T bEnableByTimeRi;
    UINT8_T bLaneBridgeLf;
    UINT8_T bLaneBridgeRi;
    UINT8_T bLaneWidthReset;
    REAL32_T fRawLaneWidth;
    UINT8_T bLowPassReset;
    REAL32_T fTimeLowPass;
    REAL32_T fRawTimeLowPass;
    REAL32_T fMaxLaneWidth;
    REAL32_T fMinLaneWidth;
    REAL32_T fCoeffLowPass;
} sLWVDebug_t;
#endif

#ifndef Rte_TypeDef_sKLMInput_t
#define Rte_TypeDef_sKLMInput_t
typedef struct {
    UINT8_T uLaneValidQualifer;
    UINT8_T bValidNewCorr;
    UINT8_T bUpDownHillDegrade;
    UINT8_T bValidKlmFltCntr;

    REAL32_T fOverallQualityLf;
    REAL32_T fOverallQualityRi;

    REAL32_T fPosY0UlpLf;   /* Left lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingUlpLf; /* Left lane heading angle by Ulp, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvUlpLf;     /* Left lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUlpLf; /* Left lane curvature rate by Ulp, (0,
                               -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUlpLf; /* Left lane length by Ulp, (0, 0~300, m) */

    REAL32_T fPosY0UlpRi;   /* Left lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingUlpRi; /* Left lane heading angle by Ulp, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvUlpRi;     /* Left lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUlpRi; /* Left lane curvature rate by Ulp, (0,
                               -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUlpRi; /* Left lane length by Ulp, (0, 0~300, m) */

    REAL32_T fLaneWidth;

    REAL32_T fVehYawRateStd; /* Ego vehicle yaw rate standard deviation, (0,
                                0~1, rad/s) */
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (0, -20~150,
                          m/s) */
    REAL32_T fVehYawRate;    /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */
    REAL32_T fStraightDtct;
} sKLMInput_t;
#endif

#ifndef Rte_TypeDef_sKLMParam_t
#define Rte_TypeDef_sKLMParam_t
typedef struct {
    UINT8_T bUseDegrUpdateForFlt; /* Use degraded update in center lane kalman
                                     filter, (0, 0~1, -) */
    REAL32_T
    fThdQualityForDegrUpdate; /* Quality threshold for degraded update in
                                 center lane kalman filter, (50, 0~1, %) */
    REAL32_T fStdDevPosY0;   /* Center lane lateral distance standard deviation,
                                (0.1, 0~5, m) */
    REAL32_T fStdDevHeading; /* Center lane yaw angle standard deviation, (0.01,
                                0~1, rad) */
    REAL32_T fStdDevCrv; /* Center lane curvature standard deviation, (0.001,
                            0~0.5, 1/m) */
    REAL32_T fStdDevCrvRate; /* Center lane curvature rate standard deviation,
                                (1e-4, 0~0.1, 1/m^2) */

    UINT8_T
    bUseVdyYawRateStdDev;       /* TRUE:Use VDY yaw rate standard deviation.
                          FALSE:Use constant standard deviation, (0, 0~1, -) */
    REAL32_T fStdDevVehYawRate; /* Center lane yaw rate standard deviation,
                                   (0.01, 0~1, rad/s) */

    REAL32_T
    fMinVelForKlmFilt; /* Minimum kalman filter input velocity to avoid
                          internal matrix inversion failures, (0.01, 0~100,
                          m/s) */

    REAL32_T fErrCoeff1; /* Geometric model error (kappa2diff) calculation
                            factor 1 - [minimum radius european highway, (650,
                            0~3.4E38, m) */

    REAL32_T fErrCoeff2; /* Geometric model error (kappa2diff) calculation
                            factor 2 - [average ego vehicle velocity highway],
                            (35, 0~100, m/s) */

    REAL32_T
    fInitRFactor; /* Initialization factor for R_laneKF matrix -> achieve
                     better transient oscillation, (3, 1~100， -) */

    REAL32_T
    fDegradeWeight; /* Weight for degraded update_laneKF, (0.2, 1E-4~1, -)
                     */
    REAL32_T
    fMnUpdateQual;        /* Minimum measurement quality to apply kalman filter
                             update step. If the measurement quality is below only
                             the prediction step is applied., (30, 0~100, %) */
    REAL32_T fMnInitQual; /* Minimum measurement quality for the kalman filter
                             initialization, (35, 0~100, %) */
    REAL32_T
    fIncQual; /* Quality increase for the lane kalman filter after the
                 update step has been applied, (33, 1~1E4, 1/s) */
    REAL32_T fDecQualDeg;  /* Quality decrease for the lane kalman filter after
                              the degraded update step has been applied, (33,
                              1~1E4, 1/s) */
    REAL32_T fDecQualPred; /* Quality decrease for the lane kalman filter after
                              only a prediction step has been applied, (300,
                              1~1E4, 1/s) */
    REAL32_T fKGainFac;    /* Factor needed to set the dynamic of the lateral
                              position within degraded update, (1, 0~1, -) */
    REAL32_T
    fDynYawFactor;          /* Factor needed to set the dynamic of the yaw angle
                               within the prediction step, (2, 0~1E5, -) */
    REAL32_T fDynDistYFact; /* Factor needed to set the dynamic of the lateral
                               distance within the prediction step;↵100000.0f
                               High Dynamic, 10000.0f Middle Dynamic, <1000.0f
                               Low Dynamic', (2500, 0~1E5, -) */
    REAL32_T fDynCrvFact; /* Factor to control the curvature signal dynamics in
                             lane center kalman filter, (100, 0~1E5, -) */
    REAL32_T
    fDynCrvRateFact; /* Factor to control the curvature signal dynamics in
                        lane center kalman filter, (2, 0~1E5, -) */

    REAL32_T fThdLatDistDev; /* Lateral deviation threshold for estimated and
                                measured lateral position, (0.2, 0~5, m) */

    REAL32_T fDiffFadingFactor; /*Fading factor between left and right lane.
                                   Fading will be applied dependent on the
                                   internal lane quality, (60, 0~1E5, -)*/

    UINT8_T bUseGradientLimit; /* Use gradient limitation, (1, 0~1, -) */
    REAL32_T fThdPosYGrd; /* Lateral distance gradient threshold, (0.66, 0~1E5,
                             m/s) */
    REAL32_T
    fThdHeadingGrd;          /* Heading angle gradient threshold, (0.067, 0~1E5,
                                rad/s) */
    REAL32_T fThdCrvGrd;     /* Curvature gradient threshold, (0.067, -0.1~0.1,
                                1/(m*s)) */
    REAL32_T fThdCrvRateGrd; /* Curvature rate gradient threshold, (2.5E-5,
                                0~0.1, 1/(m^2*s)) */
    UINT8_T bUseDegrUpdate; /* Use degraded update in center lane kalman filter,
                               (0, 0~1, -) */
    REAL32_T
    bThdDegrUpdateQuality; /* Quality threshold for degraded update in
                              center lane kalman filter, (50, 0~100, %)*/
    UINT8_T bUseStraightDtct;
    REAL32_T fFacStraightDtct;
    UINT8_T bUseCrvEstimation;
    REAL32_T fStartLengthForCrvEst; /* Define start point for 2nd order
                                       polynomial curvature estimation, (5,
                                       0~150, m) */
    REAL32_T fEndLengthForCrvEst;   /* Define end point for 2nd order polynomial
                                       curvature estimation, (35, 0~200, m) */
    REAL32_T fMinDistForCrvEst;     /* Minimum distance between TLB
                                       (TrackerLaneBoundary) valid length and the
                                       desired start point for the curvature
                                       estimation., (5, 0~150, m) */
    REAL32_T fStartFltCrv; /* Start curvature value for maximum PT1 curvature
                              filter time constant, (5.0E-4, 0~0.1, 1/m) */
    REAL32_T fEndFltCrv;   /* Final curvature value for maximum PT1 curvature
                              filter time constant, (0.002, 0~0.1, 1/m) */
    REAL32_T fTiFltForStraight; /* PT1 time constant for curvature signal
                                   filtering while driving straight, (0.06,
                                   0~10, s) */
    REAL32_T
    fTiFltForCurve;     /* PT1 time constant for curvature signal filtering
                           while driving in a curve, (0.08, 0~10, s) */
    UINT8_T bUseCrvKlm; /* Activate curvature kalman filter, (0, 0~1, -) */

    REAL32_T
    fStdDevCrvCntr; /* Center lane curvature standard deviation, (9E-4,
                       0~0.1, 1/m) */
    REAL32_T fStdDevCrvRateCntr; /* Center lane curvature rate standard
                                    deviation, (7.0E-5, 0~0.1, 1/m^2) */
    REAL32_T fErrCoeff1Cntr;  /* Geometric model error (kappa2diff) calculation
                                 factor 1 - [minimum radius european highway,
                                 (650, 0~3.4E38, m) */
    REAL32_T fErrCoeff2Cntr;  /* Geometric model error (kappa2diff) calculation
                                 factor 2 - [average ego vehicle velocity
                                 highway], (35, 0~100, m/s) */
    REAL32_T fThdCrvCntr;     /* Defines the threshold for a curve (different
                                 filtering is applied in kalman filter),
                                 (1.0E-4, 0~0.1, 1/m) */
    REAL32_T fFacMatrixQCntr; /* Q11 matrix factor for curved road, (1.0005,
                                 0~1E5, -) */
    REAL32_T fFacMatrixQStraightCntr; /* Q11 matrix factor for straight road,
                                         (1.0002, 0~1E5, -) */

    REAL32_T
    fFacInitRCntr; /* Initialization factor for R_laneKF matrix -> achieve
                      better transient oscillation, (3, 1~100， -) */
    REAL32_T
    fMinInitQual; /* Minimum measurement quality for the kalman filter
                     initialization, (35, 0~100, %) */
    REAL32_T fMinUpdateQualCntr; /* Minimum measurement quality to apply kalman
                                    filter update step. If the measurement
                                    quality is below only the prediction step is
                                    applied., (30, 0~100, %) */
    REAL32_T fWeightDegradeCntr; /* Weight for degraded update_laneKF, (0.2,
                                    1E-4~1, -) */
    REAL32_T fQualityIncCntr;    /* Quality increase for the lane kalman filter
                                    after the update step has been applied, (33,
                                    1~1E4, 1/s) */
    REAL32_T
    fQualityDecCntr; /* Quality decrease for the lane kalman filter after
                        the update step has been applied, (16.66, 1~1E4,
                        1/s) */
    REAL32_T
    fQualityDecPredCntr; /* Quality decrease for the lane kalman filter
                            after only a prediction step has been applied,
                            (100, 1~1E4, 1/s) */

    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components ,(0, 0.001~0.1, s)
                    */
} sKLMParam_t;
#endif

#ifndef Rte_TypeDef_sKLMOutput_t
#define Rte_TypeDef_sKLMOutput_t
typedef struct {
    REAL32_T
    fPosY0KlmLf; /* Left lane Y0 position by Kalman filter, (0, -15~15, m)
                  */
    REAL32_T fHeadingKlmLf; /* Left lane heading angle by Kalman filter, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvKlmLf; /* Left lane curvature by Kalman filter, (0, -0.1~0.1,
                           1/m) */
    REAL32_T fCrvRateKlmLf;  /* Left lane curvature rate by Kalman filter, (0,
                                -0.001~0.001, 1/m^2) */
    UINT8_T uBtfStatusKlmLf; /* Left lane Kalman filter status, (0, 0~255, -) */
    REAL32_T fQualityMeasureLf; /* Left measure quality by Kalman filter, (0,
                                   0~100, -) */
    UINT8_T bLatDistDevLf;      /* Left lane lateral distance deviation flag */

    REAL32_T fPosY0KlmCntr; /* Center lane Y0 position by Kalman filter, (0,
                               -15~15, m) */
    REAL32_T
    fHeadingKlmCntr;          /* Center lane heading angle by Kalman filter, (0,
                                 -0.7854~0.7854, rad) */
    REAL32_T fCrvKlmCntr;     /* Center lane curvature by Kalman filter, (0,
                                 -0.1~0.1, 1/m) */
    REAL32_T fCrvRateKlmCntr; /* Center lane curvature rate by Kalman filter,
                                 (0, -0.001~0.001, 1/m^2) */
    UINT8_T
    uBtfStatusKlmCntr; /* Center lane Kalman filter status, (0, 0~255, -) */
    REAL32_T fQualityMeasureCntr; /* Center measure quality by Kalman filter,
                                     (0, 0~100, -) */

    REAL32_T fPosY0KlmRi;   /* Right lane Y0 position by Kalman filter, (0,
                               -15~15, m) */
    REAL32_T fHeadingKlmRi; /* Right lane heading angle by Kalman filter, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvKlmRi; /* Right lane curvature by Kalman filter, (0, -0.1~0.1,
                           1/m) */
    REAL32_T fCrvRateKlmRi; /* Right lane curvature rate by Kalman filter, (0,
                               -0.001~0.001, 1/m^2) */
    UINT8_T
    uBtfStatusKlmRi; /* Right lane Kalman filter status, (0, 0~255, -) */
    REAL32_T fQualityMeasureRi; /* Right measure quality by Kalman filter, (0,
                                   0~100, -) */
    UINT8_T bLatDistDevRi;      /* Right lane lateral distance deviation flag */
} sKLMOutput_t;
#endif

#ifndef Rte_TypeDef_sKLMDebug_t
#define Rte_TypeDef_sKLMDebug_t
typedef struct {
    UINT8_T bValidLaneLf; /* Left ego lane validity, (0, 0~1, -) */
    UINT8_T
    bEnaDegrUpdateLf;      /* Enable flag for degraded update in center lane
                              kalman filter, (0, 0~1, -) */
    REAL32_T fRawCrvKlmLf; /* Left lane curvature by Kalman filter, (0,
                              -0.1~0.1, 1/m) */
    REAL32_T fStartPosXForCrvLf;
    REAL32_T fEndPosXForCrvLf;
    REAL32_T fRawEstCrvKlmLf;
    REAL32_T fRawTiFltCrvEstLf;
    REAL32_T fTiFltCrvEstLf;
    REAL32_T fEstCrvKlmLf;

    UINT8_T bValidLaneCntr; /* Center ego lane validity, (0, 0~1, -) */
    REAL32_T fDelatYaw;
    REAL32_T fDeltaPosX;
    REAL32_T fDeltaPosY;
    REAL32_T fRawPredPosY;
    REAL32_T fPredPosY;
    REAL32_T fPredLatDistLf;
    REAL32_T fPredLatDistRi;
    UINT8_T bEnaByLfKlmDiffCntr; /* Enable flag for center lane lateral distance
                                    deviation by left lane Kalman filter Y
                                    predicted difference, (0, 0~1, -) */
    UINT8_T bEnaByRiKlmDiffCntr; /* Enable flag for center lateral distance
                                    deviation by right lane Kalman filter Y
                                    predicted difference, (0, 0~1, -) */
    UINT8_T bLatDistDevLf;
    UINT8_T bLatDistDevRi;
    REAL32_T fFacFadingCntr;
    REAL32_T fRawPosY0Cntr; /* Center lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fRawHeadingCntr; /* Center lane heading angle by Ulp, (0,
                                 -0.7854~0.7854, rad) */
    REAL32_T fRawCrvCntr; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fRawCrvRateCntr; /* Center lane curvature rate by Ulp, (0,
                                 -0.001~0.001, 1/m^2) */
    UINT8_T bResetRateLimitCntr;
    REAL32_T fPosY0Cntr;   /* Center lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingCntr; /* Center lane heading angle by Ulp, (0,
                              -0.7854~0.7854, rad) */
    REAL32_T fCrvCntr; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCntr; /* Center lane curvature rate by Ulp, (0,
                              -0.001~0.001, 1/m^2) */
    REAL32_T fOverallQualityCntr;
    UINT8_T bRawEnaDegrUpdate;
    UINT8_T bEnaByEdgeFail;
    UINT8_T bEnaBySRTrig;
    UINT8_T bEnaDegrUpdateCntr;
    REAL32_T fCoeffByStraightDtct;
    REAL32_T fRawDegrWeightCntr;
    REAL32_T fLengthValidCntr;
    REAL32_T fStartPosXForCrvCntr;
    REAL32_T fEndPosXForCrvCntr;
    REAL32_T fRawEstCrvKlmCntr;
    REAL32_T fRawTiFltCrvEstCntr;
    REAL32_T fTiFltCrvEstCntr;
    REAL32_T fEstCrvKlmCntr;
    REAL32_T fCrvByCrvFltCntr;
    REAL32_T fCrvRateByCrvFltCntr;
    UINT8_T uBtfStatusByCrvFltLf;
    UINT8_T fQualityByCrvFltLf;

    UINT8_T bValidLaneRi; /* Right ego lane validity, (0, 0~1, -) */
    UINT8_T
    bEnaDegrUpdateRi;      /* Enable flag for degraded update in center lane
                              kalman filter, (0, 0~1, -) */
    REAL32_T fRawCrvKlmRi; /* Right lane curvature by Kalman filter, (0,
                              -0.1~0.1, 1/m) */
    REAL32_T fStartPosXForCrvRi;
    REAL32_T fEndPosXForCrvRi;
    REAL32_T fRawEstCrvKlmRi;
    REAL32_T fRawTiFltCrvEstRi;
    REAL32_T fTiFltCrvEstRi;
    REAL32_T fEstCrvKlmRi;
} sKLMDebug_t;
#endif

typedef struct {
    REAL32_T fPosY0;
    REAL32_T fHeading;
    REAL32_T fCrv;
    REAL32_T fCrvRate;
    REAL32_T fValidLength;

    REAL32_T fStdDevPosY0;
    REAL32_T sStdDevHeading;
    REAL32_T fStdDevCrv;
    REAL32_T fStdDevCrvRate;
    REAL32_T fStdDevVehYawRate;

    REAL32_T fVehVelX;
    REAL32_T fVehYawRate;

    REAL32_T fDeltaTime;
    UINT8_T bLaneDataValid;
    UINT8_T bDegradedUpdate;
    REAL32_T fOverallQuality;

    UINT8_T bLaneChange;

    REAL32_T fErrCoeff1;
    REAL32_T fErrCoeff2;
    REAL32_T fInitRFactor;
    REAL32_T fDegradeWeight;
    UINT8_T fMnUpdateQual;
    UINT8_T fMnInitQual;
    REAL32_T fIncQual;
    REAL32_T fDecQualDeg;
    REAL32_T fDecQualPred;
    REAL32_T fKGainFac;
    REAL32_T fDynYawFactor;
    REAL32_T fDynDistYFact;
    REAL32_T fDynCrvFact;
    REAL32_T fDynCrvRateFact;
} sLKFInput_t;

typedef struct {
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;

    REAL32_T sf_CrvStdDev_1pm;
    REAL32_T sf_CrvChngStdDev_1pm2;

    REAL32_T sf_VehVelX_mps;
    REAL32_T sf_DeltaT_sec;

    UINT8_T sf_crvDataValid_bool;
    UINT8_T sf_DegradedUpdate_bool;
    UINT8_T sf_OverallMeasurementQuality_perc;

    REAL32_T sf_CrvKFErrCoeff1_nu;
    REAL32_T sf_CrvKFErrCoeff2_nu;

    REAL32_T sf_CrvKFDefCurve_1pm;
    REAL32_T sf_CrvKFQ11Fac_nu;
    REAL32_T sf_CrvKFQ11FacStraight_nu;

    REAL32_T sf_CrvKFInitRFactor_nu;
    UINT8_T sf_CrvKFMnInitQual_perc;

    UINT8_T sf_CrvKFMnUpdateQual_perc;
    REAL32_T sf_CrvKFDegradeWeight_nu;

    REAL32_T sf_CrvKFIncQual_1ps;
    REAL32_T sf_CrvKFDecQualDeg_1ps;
    REAL32_T sf_CrvKFDecQualPred_1ps;
} sCrvFltInput_t;

typedef struct {
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;
    UINT8_T sf_KFStatus_btf;
    UINT8_T sf_QualityMeasure_perc;
} sCrvFltOutput_t;

#ifndef Rte_TypeDef_sCFVInput_t
#define Rte_TypeDef_sCFVInput_t
typedef struct {
    UINT8_T
    bLaneChangeDtct; /* The flag for lane change detection, (0, 0~1, -) */
    UINT8_T bValidLaneWidth; /* Valid flag for lane width by ULP, (0, 0~1, -) */

    UINT8_T
    uBtfKalmanFilterLf; /* Left lane Kalman filter status, (0, 0~255, -) */
    REAL32_T fQualityMeasureLf; /* Left lane measure Quality by Kalman filter,
                                   (0, 0~100, %) */
    REAL32_T
    fPosY0FltLf; /* Left lane Y0 position by Kalman filter (0, -15~15, m) */
    REAL32_T fHeadingFltLf; /* Left lane heading angle by Kalman filter, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvFltLf; /* Left lane curvature by Kalman filter, (0, -0.1~0.1,
                           1/m) */
    REAL32_T
    fCrvRateFltLf; /* Left lane curvature change by Kalman filter, (0,
                      -0.001~0.001, 1/m^2) */

    UINT8_T uBtfKalmanFilterCntr; /* Center lane Kalman filter status, (0,
                                     0~255, -) */
    REAL32_T fQualityMeasureCntr; /* Center lane measure Quality by Kalman
                                     filter, (0, 0~100, %) */
    REAL32_T fPosY0FltCntr; /* Center lane Y0 position by Kalman filter (0,
                               -15~15, m) */
    REAL32_T
    fHeadingFltCntr;      /* Center lane heading angle by Kalman filter, (0,
                             -0.7854~0.7854, rad) */
    REAL32_T fCrvFltCntr; /* Center lane curvature by Kalman filter, (0,
                             -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateFltCntr; /* Center lane curvature change by Kalman filter, (0,
                        -0.001~0.001, 1/m^2) */

    UINT8_T
    uBtfKalmanFilterRi; /* Right lane Kalman filter status, (0, 0~255, -) */
    REAL32_T
    fQualityMeasureRi; /* Right lane measure Quality by Kalman filter, (0,
                          0~100, %) */
    REAL32_T
    fPosY0FltRi; /* Right lane Y0 position by Kalman filter (0, -15~15, m)
                  */
    REAL32_T fHeadingFltRi; /* Right lane heading angle by Kalman filter, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvFltRi; /* Right lane curvature by Kalman filter, (0, -0.1~0.1,
                           1/m) */
    REAL32_T
    fCrvRateFltRi; /* Right lane curvature change by Kalman filter, (0,
                      -0.001~0.001, 1/m^2) */
} sCFVInput_t;
#endif

#ifndef Rte_TypeDef_sCFVParam_t
#define Rte_TypeDef_sCFVParam_t
typedef struct {
    REAL32_T
    fMinKalmanQuality; /* Minimum kalman filter quality, (3, 0~100, %) */
    REAL32_T
    fMaxDistYStep; /* Maximum for step detection in lateral positions,
                      (0.17, 0~5, m) */
    REAL32_T fMaxHeadingStep; /* Maximum for step detection in heading angle,
                                 (0, -0.7854~0.7854, rad) */
    REAL32_T fMaxCrvStep;     /* Maximum for step detection in curvature, (0.17,
                                 0~5, m) */
    REAL32_T fMaxCrvRateStep; /* Maximum for step detection in curvature rate,
                                 (0.17, 0~5, m) */

    REAL32_T fTdDistYStepValid;   /* Turn on delay time for lateral positions
                                     validity, (1, 0~60, s) */
    REAL32_T fTdHeadingStepValid; /* Turn on delay time for heading angle
                                     validity, (1, 0~60, s) */
    REAL32_T fTdCrvStepValid; /* Turn on delay time for curvature validity, (1,
                                 0~60, s) */
    REAL32_T fTdCrvRateStepValid;  /* Turn on delay time for curvature rate
                                      validity, (1, 0~60, s) */
    REAL32_T fMinQualityForBridge; /* Minimum center lane kalman filter to
                                      enable lane bridging, (30, 0~100, %) */

    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06,
                               0.001~0.1, s) */
} sCFVParam_t;
#endif

#ifndef Rte_TypeDef_sCFVOutput_t
#define Rte_TypeDef_sCFVOutput_t
typedef struct {
    UINT8_T bValidKalmanFilterLf; /* Left lane Kalman filter valid status, (0,
                                     0~1, -) */
    UINT8_T
    bInValidDistYStepLf; /* Left lateral distance step invalid flag, (0,
                            0~1, -) */
    UINT8_T bInValidHeadingStepLf; /* Left lateral heading angle invalid flag,
                                      (0, 0~1, -) */
    UINT8_T bInValidCrvStepLf; /* Left lateral curvature rate invalid flag, (0,
                                  0~1, -) */
    UINT8_T bInValidCrvRateStepLf; /* Left lateral curvature rate step invalid
                                      flag, (0, 0~1, -) */

    UINT8_T bValidKalmanFilterCntr; /* Center lane Kalman filter valid status,
                                       (0, 0~1, -) */
    UINT8_T bInValidDistYStepCntr; /* Center lateral distance step invalid flag,
                                      (0, 0~1, -) */
    UINT8_T bInValidHeadingStepCntr; /* Center lateral heading angle invalid
                                        flag, (0, 0~1, -) */
    UINT8_T bInValidCrvStepCntr; /* Center lateral curvature rate invalid flag,
                                    (0, 0~1, -) */
    UINT8_T bInValidCrvRateStepCntr; /* Center lateral curvature rate step
                                        invalid flag, (0, 0~1, -) */
    UINT8_T bPossibleLaneBridgeCntr; /* Lane bridge possible by center lane ,
                                        (0, 0~1, -)*/

    UINT8_T bValidKalmanFilterRi;  /* Right lane Kalman filter valid status, (0,
                                      0~1, -) */
    UINT8_T bInValidDistYStepRi;   /* Right lateral distance step invalid flag,
                                      (0, 0~1, -) */
    UINT8_T bInValidHeadingStepRi; /* Right lateral heading angle invalid flag,
                                      (0, 0~1, -) */
    UINT8_T
    bInValidCrvStepRi; /* Right lateral curvature rate invalid flag, (0,
                          0~1, -) */
    UINT8_T
    bInValidCrvRateStepRi; /* Right lateral curvature rate step invalid
                              flag, (0, 0~1, -) */
} sCFVOutput_t;
#endif

#ifndef Rte_TypeDef_sCFVDebug_t
#define Rte_TypeDef_sCFVDebug_t
typedef struct {
    UINT8_T bEnaStepDtctLf; /* Enable flag for left lane step detection, (0,
                               0~1, -) */
    REAL32_T fAbsDistYStepLf;
    UINT8_T
    bRawInValidDistYStepLf; /* Raw last Left lane lateral distance step
                               invalidity, (0, 0~1, -)*/
    UINT8_T
    bDlyValidDistYStepLf; /* Left lane lateral distance step invalidity
                             after turn on delay, (0, 0~1, -)*/
    REAL32_T fTimerDistYStepValidLf;
    UINT8_T bResetDistYStepLf;
    REAL32_T fAbsHeadingStepLf;
    UINT8_T bRawInValidHeadingStepLf; /* Raw last left lane heading angle step
                                         invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidHeadingStepLf;   /* Left lane heading angle step invalidity
                                         after turn on delay, (0, 0~1, -)*/
    REAL32_T fTimerHeadingStepValidLf;
    UINT8_T bResetHeadingStepLf;
    REAL32_T fAbsCrvStepLf;
    UINT8_T bRawInValidCrvStepLf;
    UINT8_T bDlyValidCrvStepLf;
    REAL32_T fTimerCrvStepValidLf;
    UINT8_T bResetCrvStepLf;
    REAL32_T fAbsCrvRateStepLf;
    UINT8_T bRawInValidCrvRateStepLf;
    UINT8_T bDlyValidCrvRateStepLf;
    REAL32_T fTimerCrvRateStepValidLf;
    UINT8_T bResetCrvRateStepLf;

    UINT8_T bEnaStepDtctCntr;
    REAL32_T fAbsDistYStepCntr;
    UINT8_T bRawInValidDistYStepCntr;
    UINT8_T bDlyValidDistYStepCntr;
    REAL32_T fTimerDistYStepValidCntr;
    UINT8_T bResetDistYStepCntr;
    REAL32_T fAbsHeadingStepCntr;
    UINT8_T bRawInValidHeadingStepCntr;
    UINT8_T bDlyValidHeadingStepCntr;
    REAL32_T fTimerHeadingStepValidCntr;
    UINT8_T bResetHeadingStepCntr;
    REAL32_T fAbsCrvStepCntr;
    UINT8_T bRawInValidCrvStepCntr;
    UINT8_T bDlyValidCrvStepCntr;
    REAL32_T fTimerCrvStepValidCntr;
    UINT8_T bResetCrvStepCntr;
    REAL32_T fAbsCrvRateStepCntr;
    UINT8_T bRawInValidCrvRateStepCntr;
    UINT8_T bDlyValidCrvRateStepCntr;
    REAL32_T fTimerCrvRateStepValidCntr;
    UINT8_T bResetCrvRateStepCntr;

    UINT8_T bEnaStepDtctRi;
    REAL32_T fAbsDistYStepRi;
    UINT8_T bRawInValidDistYStepRi;
    UINT8_T bDlyValidDistYStepRi;
    REAL32_T fTimerDistYStepValidRi;
    UINT8_T bResetDistYStepRi;
    REAL32_T fAbsHeadingStepRi;
    UINT8_T bRawInValidHeadingStepRi;
    UINT8_T bDlyValidHeadingStepRi;
    REAL32_T fTimerHeadingStepValidRi;
    UINT8_T bResetHeadingStepRi;
    REAL32_T fAbsCrvStepRi;
    UINT8_T bRawInValidCrvStepRi;
    UINT8_T bDlyValidCrvStepRi;
    REAL32_T fTimerCrvStepValidRi;
    UINT8_T bResetCrvStepRi;
    REAL32_T fAbsCrvRateStepRi;
    UINT8_T bRawInValidCrvRateStepRi;
    UINT8_T bDlyValidCrvRateStepRi;
    REAL32_T fTimerCrvRateStepValidRi;
    UINT8_T bResetCrvRateStepRi;
} sCFVDebug_t;
#endif

#ifndef Rte_TypeDef_sLFPInput_t
#define Rte_TypeDef_sLFPInput_t
typedef struct {
    /* LWV */
    REAL32_T
    fVehVelX; /* Vehicle speed based on the wheel speeds , (unit, km/h) */
    UINT8_T bNewCorridorValid;
    UINT8_T bUpDownHillDegrade;

    REAL32_T fPosY0Lf;
    REAL32_T fHeadingLf;
    UINT8_T uQualityLf;
    UINT16_T uRangeCheckQualifierLf;
    UINT8_T bNotAvailableLf;
    UINT8_T bDistYStepDtctLf;
    UINT8_T bLengthInvalidLf;
    UINT8_T bBridgeUnCplLf;
    UINT8_T bQualityNotValidLf;
    REAL32_T fPosY0Ri;
    REAL32_T fHeadingRi;
    UINT8_T uQualityRi;
    UINT16_T uRangeCheckQualifierRi;
    UINT8_T bNotAvailableRi;
    UINT8_T bDistYStepDtctRi;
    UINT8_T bLengthInvalidRi;
    UINT8_T bBridgeUnCplRi;
    UINT8_T bQualityNotValidRi;

    /* KLM */
    // UINT8_T  uLaneValidQualifer;
    // UINT8_T  bValidNewCorr;
    // UINT8_T  bUpDownHillDegrade;
    UINT8_T bValidKlmFltCntr;

    REAL32_T fOverallQualityLf;
    REAL32_T fOverallQualityRi;

    REAL32_T fPosY0UlpLf;   /* Left lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingUlpLf; /* Left lane heading angle by Ulp, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvUlpLf;     /* Left lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUlpLf; /* Left lane curvature rate by Ulp, (0,
                               -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUlpLf; /* Left lane length by Ulp, (0, 0~300, m) */

    REAL32_T fPosY0UlpRi;   /* Left lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingUlpRi; /* Left lane heading angle by Ulp, (0,
                               -0.7854~0.7854, rad) */
    REAL32_T fCrvUlpRi;     /* Left lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUlpRi; /* Left lane curvature rate by Ulp, (0,
                               -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUlpRi; /* Left lane length by Ulp, (0, 0~300, m) */

    REAL32_T fLaneWidth;

    REAL32_T fVehYawRateStd; /* Ego vehicle yaw rate standard deviation, (0,
                                0~1, rad/s) */
    // REAL32_T fVehVelX;                      /* Vehicle speed based on the
    // wheel speeds , (0, -20~150, m/s) */
    REAL32_T fVehYawRate;    /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */
    REAL32_T fStraightDtct;
} sLFPInput_t;
#endif

#ifndef Rte_TypeDef_sLFPParam_t
#define Rte_TypeDef_sLFPParam_t
typedef struct {
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06,
                               0.001~0.1, s) */
} sLFPParam_t;
#endif

#ifndef Rte_TypeDef_sLFPOutput_t
#define Rte_TypeDef_sLFPOutput_t
typedef struct {
    UINT8_T uLaneValidQualifier;     /* xx, (0, 0~1, LFP) */
    UINT8_T uBridgePossible;         /* xx, (0, 0~1, LFP) */
    UINT8_T bKalmanValidLf;          /* xx, (0, 0~1, LFP) */
    UINT8_T bDistYStepDebouncedLf;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedLf; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedLf;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedLf; /* xx, (0, 0~1, LFP) */

    UINT8_T bKalmanValidCntr;          /* xx, (0, 0~1, LFP) */
    UINT8_T bDistYStepDebouncedCntr;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedCntr; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedCntr;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedCntr; /* xx, (0, 0~1, LFP) */

    UINT8_T bKalmanValidRi;          /* xx, (0, 0~1, LFP) */
    UINT8_T bDistYStepDebouncedRi;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedRi; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedRi;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedRi; /* xx, (0, 0~1, LFP) */

    REAL32_T
    fFltQualityCntr; /* Center lane kalman quality, (0, 0~100, %, LFP) */
    UINT8_T
    uFltStatusCntr; /* Center lane kalman filter status, (0, 0~5, -, LFP) */
                    /* Left boundary */
    REAL32_T fPosX0FltLf; /* Left lane X0 position after kalman filter, (0,
                             -300~300, m, LFP) */
    REAL32_T fPosY0FltLf; /* Left lane Y0 position after kalman filter, (0,
                             -15~15, m，LFP)  */
    REAL32_T
    fHeadingFltLf;          /* Left lane heading after kalman filter angle, (0,
                               -0.7854~0.7854, rad, LFP) */
    REAL32_T fCrvFltLf;     /* Left lane curvature after kalman filter, (0,
                               -0.1~0.1, 1/m, LFP) */
    REAL32_T fCrvRateFltLf; /* Left lane change of curvature after kalman
                               filter, (0, -0.001~0.001, 1/m^2, LFP) */
    UINT8_T bLatDistDevLf;  /* Left lane lateral distance deviation flag */

    /* Center boundary */
    REAL32_T
    fPosX0FltCntr; /* Center lane X0 position after kalman filter, (0,
                      -300~300, m, LFP) */
    REAL32_T
    fPosY0FltCntr; /* Center lane Y0 position after kalman filter, (0,
                      -15~15, m, LFP) */
    REAL32_T
    fHeadingFltCntr;      /* Center lane heading angle after kalman filter, (0,
                             -0.7854~0.7854, rad, LFP) */
    REAL32_T fCrvFltCntr; /* Center lane curvature after kalman filter, (0,
                             -0.1~0.1, 1/m, LFP) */
    REAL32_T fCrvRateFltCntr; /* Center lane change of curvature after kalman
                                 filter, (0, -0.001~0.001, 1/m^2, LFP) */

    /* Right boundary */
    REAL32_T fPosX0FltRi; /* Right lane X0 position after kalman filter, (0,
                             -300~300, m, LFP) */
    REAL32_T fPosY0FltRi; /* Right lane Y0 position after kalman filter, (0,
                             -15~15, m, LFP)  */
    REAL32_T
    fHeadingFltRi;          /* Right lane heading angle after kalman filter, (0,
                               -0.7854~0.7854, rad, LFP) */
    REAL32_T fCrvFltRi;     /* Right lane curvature after kalman filter, (0,
                               -0.1~0.1, 1/m, LFP) */
    REAL32_T fCrvRateFltRi; /* Right lane change of curvature after kalman
                               filter, (0, -0.001~0.001, 1/m^2, LFP) */
    UINT8_T bLatDistDevRi;  /* Right lane lateral distance deviation flag */

    REAL32_T fLaneWidth;
} sLFPOutput_t;
#endif

#ifndef Rte_TypeDef_sLFPDebug_t
#define Rte_TypeDef_sLFPDebug_t
typedef struct {
    sLWVInput_t sLWVInput;
    sLWVParam_t sLWVParam;
    sLWVOutput_t sLWVOutput;
    sLWVDebug_t sLWVDebug;

    sKLMInput_t sKLMInput;
    sKLMParam_t sKLMParam;
    sKLMOutput_t sKLMOutput;
    sKLMDebug_t sKLMDebug;

    sCFVInput_t sCFVInput;
    sCFVParam_t sCFVParam;
    sCFVOutput_t sCFVOutput;
    sCFVDebug_t sCFVDebug;
} sLFPDebug_t;
#endif

/**********************************************************Local
 * Variable****************************************************************************/

typedef struct {
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;

    REAL32_T sf_CrvStdDev_1pm;
    REAL32_T sf_CrvChngStdDev_1pm2;

    REAL32_T sf_VehVelX_mps;
    REAL32_T sf_DeltaT_sec;

    UINT8_T sf_crvDataValid_bool;
    UINT8_T sf_DegradedUpdate_bool;
    UINT8_T sf_OverallMeasurementQuality_perc;

    REAL32_T sf_CrvKFErrCoeff1_nu;
    REAL32_T sf_CrvKFErrCoeff2_nu;

    REAL32_T sf_CrvKFDefCurve_1pm;
    REAL32_T sf_CrvKFQ11Fac_nu;
    REAL32_T sf_CrvKFQ11FacStraight_nu;

    REAL32_T sf_CrvKFInitRFactor_nu;
    UINT8_T sf_CrvKFMnInitQual_perc;

    UINT8_T sf_CrvKFMnUpdateQual_perc;
    REAL32_T sf_CrvKFDegradeWeight_nu;

    REAL32_T sf_CrvKFIncQual_1ps;
    REAL32_T sf_CrvKFDecQualDeg_1ps;
    REAL32_T sf_CrvKFDecQualPred_1ps;
} crvKFInTypeV2;

typedef struct {
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;
    UINT8_T sf_KFStatus_btf;
    UINT8_T sf_QualityMeasure_perc;
} crvKFOutType;

typedef struct {
    REAL32_T sf_PosY0_met;
    REAL32_T sf_HeadingAngle_rad;
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;
    REAL32_T sf_Length_met;

    REAL32_T sf_PosY0StdDev_met;
    REAL32_T sf_HeadingAngleStdDev_rad;
    REAL32_T sf_CrvStdDev_1pm;
    REAL32_T sf_CrvChngStdDev_1pm2;
    REAL32_T sf_VehYawRateStdDev_radps;

    REAL32_T sf_VehVelX_mps;
    REAL32_T sf_VehYawRate_radps;

    REAL32_T sf_DeltaT_sec;
    UINT8_T sf_LaneDataValid_bool;
    UINT8_T sf_DegradedUpdate_bool;
    UINT8_T sf_OverallMeasurementQuality_perc;

    UINT8_T sf_LaneChange_bool;

    REAL32_T sf_LaneKFErrCoeff1_met;
    REAL32_T sf_LaneKFErrCoeff2_mps;
    REAL32_T sf_LaneKFInitRFactor_nu;
    REAL32_T sf_LaneKFDegradeWeight_nu;
    UINT8_T sf_LaneKFMnUpdateQual_perc;
    UINT8_T sf_LaneKFMnInitQual_perc;
    REAL32_T sf_LaneKFIncQual_1ps;
    REAL32_T sf_LaneKFDecQualDeg_1ps;
    REAL32_T sf_LaneKFDecQualPred_1ps;
    REAL32_T sf_LaneKFKGainFac_nu;
    REAL32_T sf_LaneKFDynYawFactor_nu;
    REAL32_T sf_LaneKFDynDistYFact_nu;
    REAL32_T sf_LaneKFDynCrvFact_nu;
    REAL32_T sf_LaneKFDynCrvRateFact_nu;
} laneKFInTypeV3;

//  Output
typedef struct {
    REAL32_T sf_PosY0_met;
    REAL32_T sf_HeadingAngle_rad;
    REAL32_T sf_Crv_1pm;
    REAL32_T sf_CrvChng_1pm2;
    UINT8_T sf_KFStatus_btf;
    UINT8_T sf_QualityMeasure_perc;
} laneKFOutType;

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_TYPEDEF_H_
