/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "LBP_UncoupledLaneProcessing_TypeDef.h"
#include "LBP_CheckLaneProperity_TypeDef.h"
#include "LBP_LaneFilterProcessing_TypeDef.h"
#include "LBP_EgoLaneGeneration_TypeDef.h"
/* LBP input */
typedef struct {
    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components ,(0, 0~0.001, s) */
    UINT8_T uCamStatusQualifier; /* Camera status qualifier, not used in
                                    algorithm now, (x,min~max, -) */
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (0, -20~150,
                          m/s) */
    REAL32_T
    fPosY0[LBP_NUM]; /* Lateral distance at X = 0.0 m, (0, -15~15, m) */
    REAL32_T fHeadingAngle[LBP_NUM];  /* Heading angle (Yaw angle) of a lane
                                         boundary track, (0,
                                         -0.7854~0.7854, rad) */
    REAL32_T fCurvature[LBP_NUM];     /* Curvature of a lane boundary track, (0,
                                         -0.1~0.1, 1/m) */
    REAL32_T fCurvatureRate[LBP_NUM]; /* Curvature rate of a lane boundary
                                         track, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLength[LBP_NUM];   /* Visibility range of a lane boundary
                                         track, (0, 0~300, m) */
    UINT8_T
    uQuality[LBP_NUM]; /* Quality of a lane boundary track, (0, 0~100, -) */
    UINT8_T uMarkerType[LBP_NUM]; /* Describes the type of a lane marker
                                     (e.g.road edge,…) ,(0, 0~100, -) */
    UINT8_T
    uEventType[LBP_NUM]; /* Describes the event (e.g. construction site)
                            ,(0, 0~8, -) */
    REAL32_T
    fEventDistance[LBP_NUM]; /* Describes the distance to a lane event in
                                meters, (0, 0~200, m) */
    UINT8_T
    uEventQuality[LBP_NUM];         /* Describes the quality of a detected lane
                                       event ,(0, 0~100, %) */
    UINT8_T bAvailable[LBP_NUM];    /* Defines whether a lane boundary track is
                                       available or not, (0, 0~1, -) */
    REAL32_T fStdDevPosY0[LBP_NUM]; /* Standard deviation of the lateral
                                       distance of lane marker tracks, (0,0~5,
                                       m) */
    REAL32_T fStdDevHeadingAngle[LBP_NUM]; /* Standard deviation of the yaw
                                              angle of lane marker tracks, (0,
                                              0~1, rad) */
    REAL32_T fStdDevCurvature[LBP_NUM]; /* Standard deviation of the curvature,
                                           (0, 0~0.5, 1/m) */
    REAL32_T fStdDevCurvatureRate[LBP_NUM]; /* Standard deviation of the
                                               curvature rate, (0, 0~1, 1/m^2)
                                               */
    REAL32_T fVehYawRateStd; /* Ego vehicle yaw rate standard deviation, (0,
                                0~1, rad/s) */
    REAL32_T fVehYawRate;    /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    UINT8_T bParallelModelActiv; /* Parallel mode active flag, not used in
                                    algorithm now, (0, 0~1, -) */
    UINT8_T uLaneChange; /* Defines whether a lane change has been detected or
                            not, (0, 0~3, -) */
    REAL32_T fAgeMeter;  /* Any boundary age meter, not used in algorithm , (0,
                            min~max, m) */
    UINT8_T
    uColor[LBP_NUM];     /* Defines the color of a lane boundary,(0, 0~5, -) */
    UINT8_T uLeftIndex;  /* Index for left-hand any boundaries,(0, -1~10, -) */
    UINT8_T uRightIndex; /* Index for right-hand any boundaries,(0, -1~10, -) */
    REAL32_T fTstamp;    /* Camera data timestamp,(0, 0~4.295e9, s) */
    UINT8_T uRoadWorks;  /* ABD road works bitfield,(0, 0~255, -) */
    REAL32_T fFeatureCoordsXYLe; /* Undefined, not used in algorithm , (0,
                                    min~max, -) */
    REAL32_T fFeatureCoordsXYRi; /* Undefined, not used in algorithm , (0,
                                    min~max, -) */
    UINT8_T
    bCompState; /* Lane detection component state enumeration , (0, 0~6, -)
                 */
    REAL32_T fVertCurvature; /* ABD height profile estimation; Coefficient to
                                second-order monomia，(0, -0.1~0.1, 1/m)  */
    REAL32_T
    fVertCurvatureRate;      /* ABD height profile estimation; Coefficient to
                                third-order monomial，(0, -0.1~0.1, 1/m)  */
    REAL32_T fVertAvailable; /* Defines height profile available state，(0,
                                -10~10,  -)  */
    REAL32_T
    fVertSlopeChange; /* Defines slope change change state，(0, -10~10,  -)
                       */
    REAL32_T
    fVertValidLength;     /* Lookahead distance (longitudinal coordinate of
                             furthest feature pair), (0, 0~300, m) */
    UINT8_T uWeatherCond; /* Camera blockage (CB) weather condition enum ，(0,
                             0~255,  -) */
    REAL32_T fSineWaveDtct; /* Flag from ABD interface that indicates sine wave
                               road condition, (0, -10~10, -) */
    UINT8_T bVitural[LBP_NUM];
} sLBPInput_t;

/* Lane Boundary Data */
typedef struct {
    /* System parameter */
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(unit, s) */

} sLBPParam_t;

/* LBP Output */
typedef struct {
    UINT16_T uLaneInvalidQualifierLf; /* Qualifier for left lane invalid, (0,
                                         0~65535, -) */
    UINT16_T uLaneInvalidQualifierRi; /* Qualifier for right lane invalid, (0,
                                         0~65535, -)*/
    REAL32_T
    fLaneWidth; /* Lane width by uncoupled lane processing (0, 0~10, m) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */
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
    REAL32_T
    fPosY0SafeLf; /* Left lane clothoid Y0 position (safety interface),
                     (-15~15, m)*/
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
    UINT8_T
    uLaneValidQualDMC; /* Lane valid qualifier for LatDMC, (0, 0~255, -) */
    UINT8_T uVisualValidQualifier; /* Qualifier for the visualization, (0,
                                      0~255, -) */
    UINT8_T uLaneTypeLf; /* Left lane type(eg,9 means road edge) , (0~255, -) */
    UINT8_T
    uLaneTypeRi; /* Right lane type(eg,9 means road edge) , (0~255, -) */
    UINT8_T bConstructionSiteDtct; /* 1: Construction site detected; 0: No
                                      construction site (0~1, -) */
    UINT8_T uOverallQualityLf; /* Quality of the left  lane information based on
                                  all properties, (0~255, %)*/
    UINT8_T uOverallQualityRi; /* Quality of the right lane information based on
                                  all properties, (0~255, %) */
    UINT8_T
    uCrvQualityLf; /* Quality of the left curvature information, (0~255, %)
                    */
    UINT8_T uCrvQualityRi;  /* Quality of the right curvature information,
                               (0~255, %) */
    REAL32_T fABDTimeStamp; /* ABD data timestamp in seconds , (0~4295, s) */
    UINT16_T uRangeCheckQualifier; /* Input range check qualifier bitfield ,
                                      (0~65535, s) */
    REAL32_T fFltQualityCntr; /* Center lane Kalman quality, (0, 0~100, %) */
    UINT8_T uFltStatusCntr; /* Center lane Kalman filter status, (0, 0~5, -) */
    UINT16_T uOutRangeCheckQualifier; /* Output data range qualifier, (0,
                                         0~65535, -) */
    UINT8_T
    uPercStraightDtct;   /* Confidence for straight detection - value from 0
                            to 100, (0, 0~100, %) */
    UINT8_T uPercExitLf; /* Exit percent for left side, (0, 0~100, %) */
    UINT8_T uPercExitRi; /* Exit percent for right side, (0, 0~100, %) */
    UINT8_T bBridgePossibleUnCplLf; /* Left lane uncoupled lane bridging
                                       possible, (0, 0~1, -) */
    UINT8_T bBridgePossibleUnCplRi; /* Right lane uncoupled lane bridging
                                       possible, (0, 0~1, -) */
    UINT8_T uPercUpDownHillDtct;    /* Indicates downhill / uphill scenario
                                       detection confidence, (0, 0~100, %) */
    REAL32_T fLaneWidthUnCpl; /* Raw lane width of ABD interface uncoupled lane
                                 data, (0, 0~10, m) */
    REAL32_T fLaneWidthCpl;   /* Raw lane width of ABD interface coupled lane
                                 data, (0, 0~10, m) */
    UINT8_T
    uOverallQualityUnCplLf; /* Left uncoupled lane quality, (0, 0~100, %) */
    UINT8_T
    uOverallQualityUnCplRi;       /* Right uncoupled lane quality, (0, 0~100, %)
                                   */
    UINT8_T uOverallQualityCplLf; /* Left coupled lane quality, (0, 0~100, %) */
    UINT8_T
    uOverallQualityCplRi;    /* Right coupled lane quality, (0, 0~100, %) */
    UINT8_T uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0, 0~100, %) */
    UINT8_T bLineMergeDtcRi; /* Right line is merged (0, 0~1 ) */
    UINT8_T bLineMergeDtcLf; /* Left line is merged (0, 0~1 ) */
} sLBPOutput_t;

#ifndef Rte_TypeDef_sLBPDebug_t
#define Rte_TypeDef_sLBPDebug_t
typedef struct {
    /* Define ULP module variables */
    sULPInput_t sULPInput;
    sULPParam_t sULPParam;
    sULPOutput_t sULPOutput;
    sULPDebug_t sULPDebug;

    /* Define CLP module variables */
    sCLPInput_t sCLPInput;
    sCLPParam_t sCLPParam;
    sCLPOutput_t sCLPOutput;
    sCLPDebug_t sCLPDebug;

    /* Define LFP module variables */
    sLFPInput_t sLFPInput;
    sLFPParam_t sLFPParam;
    sLFPOutput_t sLFPOutput;
    sLFPDebug_t sLFPDebug;

    /* Define ELG module variables */
    sELGInput_t sELGInput;
    sELGParam_t sELGParam;
    sELGOutput_t sELGOutput;
    sELGDebug_t sELGDebug;

} sLBPDebug_t;
#endif

extern void LCF_LBP_Reset(void);

void LCF_LBP_Exec(const sLBPInput_t *pLBPInput,
                  const sLBPParam_t *pLBPParam,
                  sLBPOutput_t *pLBPOutput,
                  sLBPDebug_t *pLBPDebug);
#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_EXT_H_
