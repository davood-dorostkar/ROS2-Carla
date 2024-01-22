/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_TYPEDEF_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_TYPEDEF_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "tue_common_libs.h"

/***********************************4.Ego lane output
 * generation*************************/
#ifndef Rte_TypeDef_sESIInput_t
#define Rte_TypeDef_sESIInput_t
typedef struct {
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (-20~150,
                          m/s, In3) */

    UINT8_T bCamStatusQualifierValid; /* Camera status qualifier valid , (0~1,
                                         -, CLP) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, CLP) */

    UINT8_T bUpDownHillDegrade; /* Enable flag for downhill/uphill degrade,
                                   (0~1, DSC) */

    REAL32_T fYawLf;     /* left yaw , (x~x, rad, LFP) */
    UINT8_T bYawValidLf; /* left yaw valid, (x~x, rad, LFP) */
    REAL32_T fYawRi;     /* left yaw , (0~1, rad, LFP) */
    UINT8_T bYawValidRi; /* left yaw valid, (0~1, rad, LFP) */

    REAL32_T fPosY0Lf;   /* Left lane clothoid Y0 position , (-15~15, m, ULP) */
    REAL32_T fHeadingLf; /* Left lane clothoid heading angle , (-0.7854~0.7854,
                            rad, ULP) */
    REAL32_T fCrvLf; /* Left lane clothoid curvature  , (-0.1~0.1, 1/m, ULP) */
    UINT8_T bAvailableLf; /* Defines whether left lane is available or not,
                             (0~1, -, ULP) */
    UINT8_T uQualityLf; /* Left lane quality percentage, (0~x锟斤拷-, ULP) */
    REAL32_T fOverallQualityLf; /* Left lane quality percentage by all
                                   properties, (0~100锟斤拷%, ULP) */

    REAL32_T fPosY0Ri; /* Right lane clothoid Y0 position , (-15~15, m, ULP) */
    REAL32_T fHeadingRi; /* Right lane clothoid heading angle , (-0.7854~0.7854,
                            rad, ULP) */
    REAL32_T fCrvRi; /* Right lane clothoid curvature  , (-0.1~0.1, 1/m, ULP) */
    UINT8_T bAvailableRi; /* Defines whether right lane is available or not,
                             (0~1, -, ULP) */
    UINT8_T uQualityRi;   /* Right lane quality, (0~x锟斤拷-, ULP) */
    REAL32_T fOverallQualityRi; /* Right lane quality percentage by all
                                   properties, (0~100锟斤拷%, ULP) */
    UINT8_T bVituralLeft;
    UINT8_T bVituralRight;
} sESIInput_t;
#endif

#ifndef Rte_TypeDef_sESIParam_t
#define Rte_TypeDef_sESIParam_t
typedef struct {
    UINT8_T bUseFilteredHeadingSafe; /* flag for using filtered heading angle
                                        for safe interface, (1, 0~1, -) */
    UINT8_T bUseLatencyCompSafe; /* Activates latency compensation implemented
                                    in the safety interface, (1, 0~1, -) */
    UINT8_T
    bUseLowPassFilterSafe; /* TRUE: Activate lateral velocity PT1 filter for
                              the latency compensation, (0, 0~1, -) */
    REAL32_T fTimeLowPassFilterSafe; /* TRUE: Activate lateral velocity PT1
                                        filter for the latency compensation,
                                        (0.2, 0~10, s) */
    REAL32_T fLatencyTimeSafe; /* Latency time for lateral distance latency
                                  compensation, (0.2,0~5, s) */
    REAL32_T
    fDistYLimitStepDtct;       /* Threshold for step detection in lane marker
                                  positions, (0.17,0~5, m) */
    REAL32_T fHeadLimStepDtct; /* Threshold for step detection in lane marker
                                  positions, (0.015,0~7854, rad) */
    REAL32_T fCrvLimStepDtct;  /* Threshold for step detection in lane marker
                                  positions, (7.5e-4,0~0.1, 1/m) */
    REAL32_T
    fJumpDebounceTimeSafe; /* Cycle time for LCF_SEN components ,(0.06, s)
                            */
    REAL32_T
    fLDVirtualDelayTime; /* Cycle time for LCF_SEN components ,(0.06, s) */
    UINT8_T bUpDownDeactivatSafe; /* Deactivate SIF if up downhill scenario has
                                     been detected, (0, 0~1, -) */
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06, s) */

} sESIParam_t;
#endif

#ifndef Rte_TypeDef_sESIOutput_t
#define Rte_TypeDef_sESIOutput_t
typedef struct {
    REAL32_T
    fPosY0SafeLf; /* Left lane clothoid Y0 position (safety interface),
                     (-15~15, m) */
    REAL32_T fHeadingSafeLf; /* Left lane clothoid heading angle (safety
                                interface), (-0.7854~0.7854, rad) */
    REAL32_T fCrvSafeLf;     /* Left lane clothoid curvature (safety interface),
                                (-0.1~0.1, 1/m) */
    REAL32_T fPosY0SafeRi;   /* Right lane clothoid Y0 position (safety
                                interface), (-15~15, m) */
    REAL32_T fHeadingSafeRi; /* Right lane clothoid heading angle (safety
                                interface), (-0.7854~0.7854, rad) */
    REAL32_T fCrvSafeRi; /* Right lane clothoid curvature (safety interface),
                            (-0.1~0.1, 1/m) */
    UINT8_T uInvalidQualifierSafeLf; /* Left lane invalid qualifier for the
                                        safety interface, (0~255, -) */
    UINT8_T uInvalidQualifierSafeRi; /* Right lane invalid qualifier for the
                                        safety interface, (0~255, -) */
} sESIOutput_t;
#endif

#ifndef Rte_TypeDef_sESIDebug_t
#define Rte_TypeDef_sESIDebug_t
typedef struct {
    REAL32_T fRawLateraVelLf;
    REAL32_T fRawLateraVelRi;
    REAL32_T fCoeffFltSafe;
    REAL32_T fLateraVelLf;
    REAL32_T fLateraVelRi;
    UINT8_T bSuppressLaneStepDtctLf;
    REAL32_T fAbsDistYStepLf;
    REAL32_T fAbsHeadingStepLf;
    REAL32_T fAbsCrvStepLf;
    UINT8_T bDistYStepDtctLf;
    UINT8_T bHeadingStepDtctLf;
    UINT8_T bCrvStepDtctLf;
    UINT8_T bRawDistYStepFlipLf;
    REAL32_T fTimerDistYStepFlipLf;
    UINT8_T bRstDistYStepFlipLf;
    UINT8_T bDistYStepFlipLf;
    UINT8_T bRawHeadingStepFlipLf;
    REAL32_T fTimerHeadingStepFlipLf;
    UINT8_T bRstHeadingStepFlipLf;
    UINT8_T bHeadingStepFlipLf;
    UINT8_T bRawCrvStepFlipLf;
    REAL32_T fTimerCrvStepFlipLf;
    UINT8_T bRstCrvStepFlipLf;
    UINT8_T bCrvStepFlipLf;
    UINT8_T bSetByQualityLf;
    REAL32_T fTimerInValidByQltyLf;
    UINT8_T bInValidByQltyLf;
    UINT8_T bInValidByAllQltyLf;

    UINT8_T bSuppressLaneStepDtctRi;
    REAL32_T fAbsDistYStepRi;
    REAL32_T fAbsHeadingStepRi;
    REAL32_T fAbsCrvStepRi;
    UINT8_T bDistYStepDtctRi;
    UINT8_T bHeadingStepDtctRi;
    UINT8_T bCrvStepDtctRi;
    UINT8_T bRawDistYStepFlipRi;
    REAL32_T fTimerDistYStepFlipRi;
    UINT8_T bRstDistYStepFlipRi;
    UINT8_T bDistYStepFlipRi;
    UINT8_T bRawHeadingStepFlipRi;
    REAL32_T fTimerHeadingStepFlipRi;
    UINT8_T bRstHeadingStepFlipRi;
    UINT8_T bHeadingStepFlipRi;
    UINT8_T bRawCrvStepFlipRi;
    REAL32_T fTimerCrvStepFlipRi;
    UINT8_T bRstCrvStepFlipRi;
    UINT8_T bCrvStepFlipRi;
    UINT8_T bSetByQualityRi;
    REAL32_T fTimerInValidByQltyRi;
    UINT8_T bInValidByQltyRi;
    UINT8_T bInValidByAllQltyRi;
} sESIDebug_t;
#endif

#ifndef Rte_TypeDef_sECIInput_t
#define Rte_TypeDef_sECIInput_t
typedef struct {
    UINT8_T bCamStatusQualifierValid; /* Camera status quality valid, (0, 0~1,
                                         -, CLP) */
    UINT8_T bNotAvailableLf; /* Defines whether a lane boundary track is
                                available or not, (unit, -, CLP) */
    UINT8_T bDistYStepDtctLf;
    UINT8_T bLengthInvalidLf;
    UINT8_T bNotAvailableRi; /* Defines whether a lane boundary track is
                                available or not, (unit, -, CLP) */
    UINT8_T bDistYStepDtctRi;
    UINT8_T bLengthInvalidRi;
    UINT16_T uRangeCheckQualifier; /*  */
    UINT8_T bLaneTypeInvalidLf;
    UINT8_T bLaneColorInvalidLf;
    UINT8_T bLaneQualityInvalidLf;
    UINT8_T bLaneTypeInvalidRi;
    UINT8_T bLaneColorInvalidRi;
    UINT8_T bLaneQualityInvalidRi;

    REAL32_T fValidLengthLf; /* XX, (0, X, m, ULP) */
    REAL32_T fValidLengthRi; /* XX, (0, X, m, ULP) */
    UINT8_T bLaneVirtualCplLf;
    UINT8_T bLaneVirtualCplRi;

    UINT8_T uLaneValidQualifier; /* xx, (0, 0~1, LFP) */
    UINT8_T uBridgePossible;     /* xx, (0, 0~1, LFP) */

    UINT8_T bKalmanValidLf;          /* xx, (0, 0~1, LFP) */
    UINT8_T bDistYStepDebouncedLf;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedLf; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedLf;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedLf; /* xx, (0, 0~1, LFP) */

    UINT8_T bKalmanValidCntr; /* xx, (0, 0~1, LFP) */

    UINT8_T bKalmanValidRi;          /* xx, (0, 0~1, LFP) */
    UINT8_T bDistYStepDebouncedRi;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedRi; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedRi;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedRi; /* xx, (0, 0~1, LFP) */

    REAL32_T
    fFltQualityCntr; /* Center lane kalman quality, (0, 0~100, %, LFP) */
    UINT8_T
    uFltStatusCntr; /* Center lane kalman filter status, (0, 0~5, -, LFP) */

    UINT8_T bDistYStepDebouncedCntr;   /* xx, (0, 0~1, LFP) */
    UINT8_T bHeadingStepDebouncedCntr; /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvStepDebouncedCntr;     /* xx, (0, 0~1, LFP) */
    UINT8_T bCrvRateStepDebouncedCntr; /* xx, (0, 0~1, LFP) */

    /* Left boundary */
    REAL32_T fPosX0FltLf;   /* Left lane X0 position after kalman filter, (0,
                               -300~300, m, LFP) */
    REAL32_T fPosY0FltLf;   /* Left lane Y0 position after kalman filter, (0,
                               -15~15, m��LFP)  */
    REAL32_T fHeadingFltLf; /* Left lane heading after kalman filterangle, (0,
                               -0.7854~0.7854, rad, LFP) */
    REAL32_T fCrvFltLf;     /* Left lane curvature after kalman filter, (0,
                               -0.1~0.1, 1/m, LFP) */
    REAL32_T fCrvRateFltLf; /* Left lane change of curvature after kalman
                               filter, (0, -0.001~0.001, 1/m^2, LFP) */
    // REAL32_T fValidLengthFltLf;        /* Left lane length after kalman
    // filter, (0, 0~300, m, LFP) */

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
    REAL32_T
    fValidLengthFltCntr; /* Center lane length after kalman filter, (0,
                            0~300, m, LFP) */

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
    // REAL32_T fValidLengthFltRi;        /* Right lane length after kalman
    // filter, (0, 0~300, m, LFP) */

    REAL32_T fLaneWidth;

    UINT8_T bLineMergeDtcRi; /* Right line is merged (0, 0~1 ) */
    UINT8_T bLineMergeDtcLf; /* Left line is merged (0, 0~1 ) */
} sECIInput_t;
#endif

#ifndef Rte_TypeDef_sECIParam_t
#define Rte_TypeDef_sECIParam_t
typedef struct {
    REAL32_T fDefaultLaneWidth; /* Default Lane Width, (2.5, 0~10, m) */
    REAL32_T
    fMaxDistYRange; /* Maximum range check value for lateral position, (6,
                       0~10, m) */
    REAL32_T
    fMaxHeadingRange;      /* Maximum range check value for yaw angle, (0.5,
                              0~0.7854, rad) */
    REAL32_T fMaxCrvRange; /* Maximum range check value for curvature, (0.01,
                              0~0.1, 1/m) */
    REAL32_T
    fMaxCrvRateRange; /* Maximum range check value for curvature rate,
                         (0.001, 0~0.1, 1/m^2) */
    REAL32_T fTimeDelayVirtulLane; /* Determines the cycles for the samle turn
                                      on delay for the LD virtual information,
                                      (0.06, 0~6, s) */

    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06, s) */

} sECIParam_t;
#endif

#ifndef Rte_TypeDef_sECIOutput_t
#define Rte_TypeDef_sECIOutput_t
typedef struct {
    REAL32_T
    fLaneWidth; /* Lane width by uncoupled lane processing (0, 0~10, m) */
    UINT16_T uLaneInvalidQualifierLf; /* Qualifier for left lane invalid, (0,
                                         0~65535, -) */
    UINT16_T uLaneInvalidQualifierRi; /* Qualifier for right lane invalid, (0,
                                         0~65535, -)*/
    UINT8_T uVisualValidQualifier;    /* Qualifier for the visualization, (0,
                                         0~255, -) */
    REAL32_T fFltQualityCntr; /* Center lane kalman quality, (0, 0~100, %) */
    UINT8_T uFltStatusCntr; /* Center lane kalman filter status, (0, 0~5, -) */
    UINT8_T
    uLaneValidQualDMC; /* Lane valid qualifier for LatDMC, (0, 0~255, -) */
    UINT16_T uOutRangeCheckQualifier; /* Output data range qualifier, (0,
                                         0~65535, -) */

    /* Left boundary */
    REAL32_T
    fPosX0CtrlLf; /* Left lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlLf;   /* Left lane clothoid Y0 position  (init +10m), (0,
                                -15~15, m) */
    REAL32_T fHeadingCtrlLf; /* Left lane clothoid heading angle, (0,
                                -0.7854~0.7854, rad) */
    REAL32_T fCrvCtrlLf; /* Left lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCtrlLf;     /* Left lane clothoid change of curvature, (0,
                                    -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCtrlLf; /* Left lane clothoid length, (0, 0~300, m) */

    /* Center boundary */
    REAL32_T
    fPosX0CtrlCntr; /* Center lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlCntr;   /* Center lane clothoid Y0 position (init +10m),
                                  (0, -15~15, m) */
    REAL32_T fHeadingCtrlCntr; /* Center lane clothoid heading angle, (0,
                                  -0.7854~0.7854, rad) */
    REAL32_T
    fCrvCtrlCntr; /* Center lane clothoid curvature, (0, -0.1~0.1, 1/m*/
    REAL32_T
    fCrvRateCtrlCntr; /* Center lane clothoid change of curvature, (0,
                         -0.001~0.001, 1/m^2) */
    REAL32_T
    fValidLengthCtrlCntr; /* Center lane clothoid length, (0, 0~300, m) */

    /* Right boundary */
    REAL32_T
    fPosX0CtrlRi; /* Right lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlRi;   /* Right lane clothoid Y0 position (init +10m), (0,
                                -15~15, m) */
    REAL32_T fHeadingCtrlRi; /* Right lane clothoid heading angle, (0,
                                -0.7854~0.7854, rad) */
    REAL32_T fCrvCtrlRi; /* Right lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCtrlRi;     /* Right lane clothoid change of curvature, (0,
                                    -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCtrlRi; /* Right lane clothoid length, (0, 0~300, m) */
} sECIOutput_t;
#endif

#ifndef Rte_TypeDef_sECIDebug_t
#define Rte_TypeDef_sECIDebug_t
typedef struct {
    UINT8_T bValidBoth;
    UINT8_T bValidOnlyLf;
    UINT8_T bValidOnlyRi;

} sECIDebug_t;
#endif

typedef struct {
    UINT16_T uOutRangeCheckQualifier; /* Confidence for straight detection -
                                         value from 0 to 100, (0, 0~100, %) */
} sLSDOutput;

#ifndef Rte_TypeDef_sELGInput_t
#define Rte_TypeDef_sELGInput_t
typedef struct {
    /* EGI input */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */
    UINT8_T uLaneTypeLf; /* Left lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uLaneTypeRi; /* Right lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T bConstructionSiteDtct; /* 1: Construction site detected; 0: No
                                      construction site (0~1, -) */
    UINT8_T uOverallQualityLf; /* Quality of the left  lane information based on
                                  all properties, (0~255, %)*/
    UINT8_T uOverallQualityRi; /* Quality of the right lane information based on
                                  all properties, (0~255,%) */
    REAL32_T
    fOverallCrvQualityLf;          /* Quality of the left curvature information,
                                      (0~255, %) */
    REAL32_T fOverallCrvQualityRi; /* Quality of the right curvature
                                      information, (0~255, %) */
    REAL32_T fTstamp; /* ABD data timestamp in seconds , (0~4295, s) */
    UINT16_T uRangeCheckQualifier; /* Input range check qualifier bitfield ,
                                      (0~65535, s) */
    /* ESI input */
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (-20~150,
                          m/s, In3) */

    UINT8_T bCamStatusQualifierValid; /* Camera status qualifier valid , (0~1,
                                         -, CLP) */
    // UINT8_T  bLaneChangeDtct;          /* Flag that indicates a detected lane
    // change, (0~1, CLP) */

    UINT8_T bUpDownHillDegrade; /* Enable flag for downhill/uphill degrade,
                                   (0~1, DSC) */

    REAL32_T fYawLf;     /* left yaw , (x~x, rad, LFP) */
    UINT8_T bYawValidLf; /* left yaw valid, (x~x, rad, LFP) */
    REAL32_T fYawRi;     /* left yaw , (0~1, rad, LFP) */
    UINT8_T bYawValidRi; /* left yaw valid, (0~1, rad, LFP) */

    REAL32_T fPosY0Lf;   /* Left lane clothoid Y0 position , (-15~15, m, ULP) */
    REAL32_T fHeadingLf; /* Left lane clothoid heading angle , (-0.7854~0.7854,
                            rad, ULP) */
    REAL32_T fCrvLf; /* Left lane clothoid curvature  , (-0.1~0.1, 1/m, ULP) */
    UINT8_T bAvailableLf; /* Defines whether left lane is available or not,
                             (0~1, -, ULP) */
    UINT8_T uQualityLf;   /* Left lane quality percentage, (0~x，-, ULP) */
    REAL32_T fOverallQualityLf; /* Left lane quality percentage by all
                                   properties, (0~100，%, ULP) */
    // UINT8_T  bAvailableLf;             /* Defines whether left lane is
    // available or not, (0~1, -, ULP) */

    REAL32_T fPosY0Ri; /* Right lane clothoid Y0 position , (-15~15, m, ULP) */
    REAL32_T fHeadingRi; /* Right lane clothoid heading angle , (-0.7854~0.7854,
                            rad, ULP) */
    REAL32_T fCrvRi; /* Right lane clothoid curvature  , (-0.1~0.1, 1/m, ULP) */
    UINT8_T bAvailableRi; /* Defines whether right lane is available or not,
                             (0~1, -, ULP) */
    UINT8_T uQualityRi;   /* Right lane quality, (0~x，-, ULP) */
    REAL32_T fOverallQualityRi; /* Right lane quality percentage by all
                                   properties, (0~100，%, ULP) */
    // UINT8_T  bAvailableRi;             /* Defines whether right lane is
    // available or not, (0~1, -, ULP) */

    /* ECI input */
    UINT8_T bCamStatusQualityValid; /* Camera status quality valid, (0, 0~1, -,
                                       CLP) */
    UINT8_T bNotAvailableLf;        /* Defines whether a lane boundary track is
                                       available or not, (unit, -, CLP) */
    UINT8_T bDistYStepDtctLf;
    UINT8_T bLengthInvalidLf;
    UINT8_T bNotAvailableRi; /* Defines whether a lane boundary track is
                                available or not, (unit, -, CLP) */
    UINT8_T bDistYStepDtctRi;
    UINT8_T bLengthInvalidRi;
    // UINT16_T uRangeCheckQualifier;     /*  */
    UINT8_T bLaneTypeInvalidLf;
    UINT8_T bLaneColorInvalidLf;
    UINT8_T bLaneQualityInvalidLf;
    UINT8_T bLaneTypeInvalidRi;
    UINT8_T bLaneColorInvalidRi;
    UINT8_T bLaneQualityInvalidRi;

    REAL32_T fValidLengthLf; /* XX, (0, X, m, ULP) */
    REAL32_T fValidLengthRi; /* XX, (0, X, m, ULP) */
    UINT8_T bLaneVirtualCplLf;
    UINT8_T bLaneVirtualCplRi;
    // REAL32_T fValidLengthRi;             /* XX, (0, X, m, ULP) */

    UINT8_T uLaneValidQualifier; /* xx, (0, 0~1, LFP) */
    UINT8_T uBridgePossible;     /* xx, (0, 0~1, LFP) */

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
    REAL32_T fPosX0FltLf;   /* Left lane X0 position after kalman filter, (0,
                               -300~300, m, LFP) */
    REAL32_T fPosY0FltLf;   /* Left lane Y0 position after kalman filter, (0,
                               -15~15, m，LFP)  */
    REAL32_T fHeadingFltLf; /* Left lane heading after kalman filterangle, (0,
                               -0.7854~0.7854, rad, LFP) */
    REAL32_T fCrvFltLf;     /* Left lane curvature after kalman filter, (0,
                               -0.1~0.1, 1/m, LFP) */
    REAL32_T fCrvRateFltLf; /* Left lane change of curvature after kalman
                               filter, (0, -0.001~0.001, 1/m^2, LFP) */
    // REAL32_T fValidLengthFltLf;        /* Left lane length after kalman
    // filter, (0, 0~300, m, LFP) */

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
    REAL32_T
    fValidLengthFltCntr; /* Center lane length after kalman filter, (0,
                            0~300, m, LFP) */

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
    // REAL32_T fValidLengthFltRi;        /* Right lane length after kalman
    // filter, (0, 0~300, m, LFP) */

    REAL32_T fLaneWidth;
    UINT8_T bLineMergeDtcRi; /* Right line is merged (0, 0~1 ) */
    UINT8_T bLineMergeDtcLf; /* Left line is merged (0, 0~1 ) */
    UINT8_T bVituralLeft;
    UINT8_T bVituralRight;
} sELGInput_t;
#endif

#ifndef Rte_TypeDef_sELGParam_t
#define Rte_TypeDef_sELGParam_t
typedef struct {
    /******************************************1.ESI
     * parameter***********************************/
    UINT8_T bUseFilteredHeadingSafe; /* flag for using filtered heading angle
                                        for safe interface, (1, 0~1, -) */
    UINT8_T bUseLatencyCompSafe; /* Activates latency compensation implemented
                                    in the safety interface, (1, 0~1, -) */
    UINT8_T
    bUseLowPassFilterSafe; /* TRUE: Activate lateral velocity PT1 filter for
                              the latency compensation, (0, 0~1, -) */
    REAL32_T fTimeLowPassFilterSafe; /* TRUE: Activate lateral velocity PT1
                                        filter for the latency compensation,
                                        (0.2, 0~10, s) */
    REAL32_T fLatencyTimeSafe; /* Latency time for lateral distance latency
                                  compensation, (0.2,0~5, s) */
    REAL32_T
    fDistYLimitStepDtct;       /* Threshold for step detection in lane marker
                                  positions, (0.17,0~5, m) */
    REAL32_T fHeadLimStepDtct; /* Threshold for step detection in lane marker
                                  positions, (0.015,0~7854, rad) */
    REAL32_T fCrvLimStepDtct;  /* Threshold for step detection in lane marker
                                  positions, (7.5e-4,0~0.1, 1/m) */
    REAL32_T
    fJumpDebounceTimeSafe; /* Cycle time for LCF_SEN components ,(0.06, s)
                            */
    REAL32_T
    fLDVirtualDelayTime; /* Cycle time for LCF_SEN components ,(0.06, s) */
    UINT8_T bUpDownDeactivatSafe; /* Deactivate SIF if up downhill scenario has
                                     been detected, (0, 0~1, -) */

    /******************************************2.ECI parameter
     * **********************************/
    REAL32_T fDefaultLaneWidth; /* Default Lane Width, (2.5, 0~10, m) */
    REAL32_T
    fMaxDistYRange; /* Maximum range check value for lateral position, (6,
                       0~10, m) */
    REAL32_T
    fMaxHeadingRange;      /* Maximum range check value for yaw angle, (0.5,
                              0~0.7854, rad) */
    REAL32_T fMaxCrvRange; /* Maximum range check value for curvature, (0.01,
                              0~0.1, 1/m) */
    REAL32_T
    fMaxCrvRateRange; /* Maximum range check value for curvature rate,
                         (0.001, 0~0.1, 1/m^2) */
    REAL32_T fTimeDelayVirtulLane; /* Turn on delay for the LD virtual
                                      information, (0.06, 0~6, s) */

    /****************************************Common
     * paramter**************************************/
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06,
                               0.001~0.1, s) */
} sELGParam_t;
#endif

#ifndef Rte_TypeDef_sELGOutput_t
#define Rte_TypeDef_sELGOutput_t
typedef struct {
    /* GLO output*/
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */
    UINT8_T uLaneTypeLf; /* Left lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uLaneTypeRi; /* Right lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T bConstructionSiteDtct; /* 1: Construction site detected; 0: No
                                      construction site (0~1, -) */
    UINT8_T uOverallQualityLf; /* Quality of the left  lane information based on
                                  all properties, (0~255, %)*/
    UINT8_T uOverallQualityRi; /* Quality of the right lane information based on
                                  all properties, (0~255,%) */
    UINT8_T
    uCrvQualityLf; /* Quality of the left curvature information, (0~255, %)
                    */
    UINT8_T uCrvQualityRi;  /* Quality of the right curvature information,
                               (0~255, %) */
    REAL32_T fABDTimeStamp; /* ABD data timestamp in seconds , (0~4295, s) */
    UINT16_T uRangeCheckQualifier; /* Input range check qualifier bitfield ,
                                      (0~65535, s) */
    /* ESI output*/
    REAL32_T
    fPosY0SafeLf; /* Left lane clothoid Y0 position (safety interface),
                     (-15~15, m) */
    REAL32_T fHeadingSafeLf; /* Left lane clothoid heading angle (safety
                                interface), (-0.7854~0.7854, rad) */
    REAL32_T fCrvSafeLf;     /* Left lane clothoid curvature (safety interface),
                                (-0.1~0.1, 1/m) */
    REAL32_T fPosY0SafeRi;   /* Right lane clothoid Y0 position (safety
                                interface), (-15~15, m) */
    REAL32_T fHeadingSafeRi; /* Right lane clothoid heading angle (safety
                                interface), (-0.7854~0.7854, rad) */
    REAL32_T fCrvSafeRi; /* Right lane clothoid curvature (safety interface),
                            (-0.1~0.1, 1/m) */
    UINT8_T uInvalidQualifierSafeLf; /* Left lane invalid qualifier for the
                                        safety interface, (0~255, -) */
    UINT8_T uInvalidQualifierSafeRi; /* Right lane invalid qualifier for the
                                        safety interface, (0~255, -) */
    /* ECI output*/
    REAL32_T
    fLaneWidth; /* Lane width by uncoupled lane processing (0, 0~10, m) */
    UINT16_T uLaneInvalidQualifierLf; /* Qualifier for left lane invalid, (0,
                                         0~65535, -) */
    UINT16_T uLaneInvalidQualifierRi; /* Qualifier for right lane invalid, (0,
                                         0~65535, -)*/
    UINT8_T uVisualValidQualifier;    /* Qualifier for the visualization, (0,
                                         0~255, -) */
    REAL32_T fFltQualityCntr; /* Center lane kalman quality, (0, 0~100, %) */
    UINT8_T uFltStatusCntr; /* Center lane kalman filter status, (0, 0~5, -) */
    UINT8_T
    uLaneValidQualDMC; /* Lane valid qualifier for LatDMC, (0, 0~255, -) */
    UINT16_T uOutRangeCheckQualifier; /* Output data range qualifier, (0,
                                         0~65535, -) */

    /* Left boundary */
    REAL32_T
    fPosX0CtrlLf; /* Left lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlLf;   /* Left lane clothoid Y0 position  (init +10m), (0,
                                -15~15, m) */
    REAL32_T fHeadingCtrlLf; /* Left lane clothoid heading angle, (0,
                                -0.7854~0.7854, rad) */
    REAL32_T fCrvCtrlLf; /* Left lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCtrlLf;     /* Left lane clothoid change of curvature, (0,
                                    -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCtrlLf; /* Left lane clothoid length, (0, 0~300, m) */

    /* Center boundary */
    REAL32_T
    fPosX0CtrlCntr; /* Center lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlCntr;   /* Center lane clothoid Y0 position (init +10m),
                                  (0, -15~15, m) */
    REAL32_T fHeadingCtrlCntr; /* Center lane clothoid heading angle, (0,
                                  -0.7854~0.7854, rad) */
    REAL32_T
    fCrvCtrlCntr; /* Center lane clothoid curvature, (0, -0.1~0.1, 1/m*/
    REAL32_T
    fCrvRateCtrlCntr; /* Center lane clothoid change of curvature, (0,
                         -0.001~0.001, 1/m^2) */
    REAL32_T
    fValidLengthCtrlCntr; /* Center lane clothoid length, (0, 0~300, m) */

    /* Right boundary */
    REAL32_T
    fPosX0CtrlRi; /* Right lane clothoid X0 position, (0, -300~300, m) */
    REAL32_T fPosY0CtrlRi;   /* Right lane clothoid Y0 position (init +10m), (0,
                                -15~15, m) */
    REAL32_T fHeadingCtrlRi; /* Right lane clothoid heading angle, (0,
                                -0.7854~0.7854, rad) */
    REAL32_T fCrvCtrlRi; /* Right lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCtrlRi;     /* Right lane clothoid change of curvature, (0,
                                    -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCtrlRi; /* Right lane clothoid length, (0, 0~300, m) */
} sELGOutput_t;
#endif

#ifndef Rte_TypeDef_sELGDebug_t
#define Rte_TypeDef_sELGDebug_t
typedef struct {
    sESIInput_t sESIInput;
    sESIParam_t sESIParam;
    sESIOutput_t sESIOutput;
    sESIDebug_t sESIDebug;
    sECIInput_t sECIInput;
    sECIParam_t sECIParam;
    sECIOutput_t sECIOutput;
    sECIDebug_t sECIDebug;
} sELGDebug_t;
#endif

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EGOLANEGENERATION_TYPEDEF_H_
