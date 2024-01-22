/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_TYPEDEF_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_TYPEDEF_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "tue_common_libs.h"
/* Lane configuration*/
#define LBP_NUM 4   /* Total lane lines */
#define UN_CPL_LF 0 /* Uncoupled left lane index */
#define UN_CPL_RI 1 /* Uncoupled right lane line index */
#define CPL_LF 2    /* coupled left lane line index */
#define CPL_RI 3    /* coupled right lane line index */

/***********************************************1.Uncoupled lane
 * processing*************************************************/
#ifndef Rte_TypeDef_sDMQInput_t
#define Rte_TypeDef_sDMQInput_t
typedef struct {
    REAL32_T fPosY0;   /* Lateral distance at X = 0.0 m, (0, -15~15, m) */
    REAL32_T fHeading; /* Heading angle (Yaw angle) of a lane boundary track,
                          (0, -0.7854~0.7854, rad) */
    REAL32_T fCrv; /* Curvature of a lane boundary track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRate;     /* Curvature rate of a lane boundary track, (0,
                              -0.001~0.001, 1/m^2) */
    REAL32_T fValidLength; /* Visibility range of a lane boundary track, (0,
                              0~300, m) */
    UINT8_T
    bAvailable; /* Defines whether a lane boundary track is available or
                   not, (0, 0~1, -) */
    REAL32_T
    fStdDevPosY0; /* Standard deviation of the lateral distance of lane
                     marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeading; /* Standard deviation of the yaw angle of lane
                                marker tracks, (0, 0~1, rad) */
    REAL32_T
    fStdDevCrv; /* Standard deviation of the curvature, (0, 0~0.5, 1/m) */
    REAL32_T fStdDevCrvRate; /* Standard deviation of the curvature rate, (0,
                                0~1, 1/m^2) */
    REAL32_T fQuality; /* Quality of a lane boundary track, (0, 0~100, -) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -) */
    UINT8_T bUpDnHillDgrd;   /* Flag that indicates downhill/uphill degrade, (0,
                                0~1, -) */
} sDMQInput_t;
#endif

#ifndef Rte_TypeDef_sDMQParam_t
#define Rte_TypeDef_sDMQParam_t
typedef struct {
    REAL32_T
    fMinPosY0Step; /* The min lateral distance step , (0.06, 0~15, m) */
    REAL32_T
    fMaxPosY0Step; /* The max lateral distance step , (0.12, 0~15, m) */
    REAL32_T fFactorPosY0Step; /* The quality penalty factor for lateral
                                  distance step, (50, 0~100, -) */

    REAL32_T fMinHeadingStep; /* The min heading angle step, (0.004, 0~0.7854,
                                 rad) */
    REAL32_T fMaxHeadingStep; /* The max heading angle step, (0.008, 0~0.7854,
                                 rad) */
    REAL32_T
    fFactorHeadingStep; /* The quality penalty factor for heading angle
                           step, (40, 0~100, -) */

    REAL32_T fMinCrvStep;    /* The min curvature step , (3e-4, 0~0.1, 1/m) */
    REAL32_T fMaxCrvStep;    /* The max curvature step , (6e-4, 0~0.1, 1/m) */
    REAL32_T fFactorCrvStep; /* The quality penalty factor for curvature step,
                                (50, 0~100, -) */

    REAL32_T fMinCrvRateStep;    /* The min curvature rate step, (3e-6, 0~0.001,
                                    1/m^2) */
    REAL32_T fMaxCrvRateStep;    /* The max curvature rate step, (6e-6, 0~0.001,
                                    1/m^2) */
    REAL32_T fFactorCrvRateStep; /* The quality penalty factor for curvature
                                    rate step, (20, 0~100, -) */

    REAL32_T
    fMaxRawQualityPenalty; /* The max quality penalty by step detection
                              before rate limit, (100, 0~100, %)*/
    REAL32_T fMaxQualityPenaltyRate; /* The Max change rate of lane quality
                                        penalty, (10000, 0~100000, %/s)*/
    REAL32_T fMinQualityPenaltyRate; /* The Min change rate of lane quality
                                        penalty, (-100, -10000000~100000, %/s)*/
    REAL32_T fValidLengthQuality;    /* Reference valid length for quality, (30,
                                        0~300, m) */
    REAL32_T
    fTdEnaLaneVirtualCpl;        /* Turn on delay time for lane virtual enable
                                    flag, (0.06, 0~60, s) */
    REAL32_T fDefQlyVirtualLane; /* Default internal quality for LD virtual
                                    line, (31, 0~100, %) */

    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06,
                               0.001~0.1, s) */
} sDMQParam_t;
#endif

#ifndef Rte_TypeDef_sDMQOutput_t
#define Rte_TypeDef_sDMQOutput_t
typedef struct {
    REAL32_T
    fOverallQuality; /* Lane quality percentage by all properties, (0,
                        0~100%) */
    REAL32_T
    fCrvQuality; /* Curvature quality percentage by all properties, (0,
                    0~100%) */
    UINT8_T bEnaLaneVirtualCpl; /* Enable flag for coupled lane virtual, (0,
                                   0~1, -) */
} sDMQOutput_t;
#endif

#ifndef Rte_TypeDef_sDMQDebug_t
#define Rte_TypeDef_sDMQDebug_t
typedef struct {
    REAL32_T fQualityPenaltyByPosY0; /* The quality penalty by lateral distance
                                        step, (0~10, -)*/
    REAL32_T fQualityPenaltyByHeading; /* The quality penalty by heading angle
                                          step, (0~10, -)*/
    REAL32_T fQualityPenaltyByCrv;     /* The quality penalty by curvature step,
                                          (0~10, -)*/
    REAL32_T fQualityPenaltyByCrvRate; /* The quality penalty by curvature rate
                                          step, (0~10, -)*/
    REAL32_T fRawQualityPenalty;       /* Raw quality penalty, (0, 0~100, %) */
    REAL32_T fQualityPenalty; /* The quality penalty  by step detection after
                                 rate limit, (0~100, %) */
    REAL32_T fLengthQuality;  /* The quality penalty  by lane valid length ,
                                 (0~100, %) */
    REAL32_T fMeasureQuality; /* The quality penalty  by measurement quality ,
                                 (0~100, %) */
    REAL32_T fRawOverallQuality; /* Lane quality percentage by all properties,
                                    (0, 0~100%) */
    UINT8_T
    bDlyEnaLaneVirtualCpl; /* Enable flag for coupled lane virtual after
                              turn on delay, (0, 0~1, -) */
    REAL32_T fAbsStepPosY0;
    REAL32_T fAbsStepHeading;
    REAL32_T fAbsStepCrv;
    REAL32_T fAbsStepCrvRate;
    REAL32_T fLastPosY0;
    REAL32_T fLastHeading;
    REAL32_T fLastCrv;
    REAL32_T fLastCrvRate;
    UINT8_T bLastAvailable;
    REAL32_T fLastQualityPenalty;
    REAL32_T fLastOverallQuality;
    UINT8_T bLastDlyEnaLaneVirtualCpl;
    REAL32_T fMeasureCrvQuality; /* Curvature quality by measure standard
                                    deviation, (0,0~100, %) */
    REAL32_T
    fLengthCrvQuality; /* Curvature quality by valid length, (0, 0~100, %)
                        */
    REAL32_T fTimerEnaLaneVirtualCpl; /* Timer for coupled virtual lane enable
                                         flag turn off delay, (0, 0~60, s) */
    REAL32_T fRawCrvQuality;          /* Raw curvature quality percentage by all
                                         properties, (0, 0~100%) */

} sDMQDebug_t;
#endif

#ifndef Rte_TypeDef_sLMCInput_t
#define Rte_TypeDef_sLMCInput_t
typedef struct {
    REAL32_T fPosY0;   /* Lateral distance at X = 0.0 m, (0, -15~15, m) */
    REAL32_T fHeading; /* Heading angle (Yaw angle) of a lane boundary track,
                          (0, -0.7854~0.7854, rad) */
    REAL32_T fCrv; /* Curvature of a lane boundary track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRate;     /* Curvature rate of a lane boundary track, (0,
                              -0.001~0.001, 1/m^2) */
    REAL32_T fValidLength; /* Visibility range of a lane boundary track, (0,
                              0~300, m) */
    UINT8_T
    bAvailable;        /* Defines whether a lane boundary track is available or
                          not, (0, 0~1, -) */
    REAL32_T fQuality; /* Quality of a lane boundary track, (0, 0~100, -) */
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (0, -20~150,
                          m/s) */
    REAL32_T fVehYawRate; /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    REAL32_T
    fOverallQuality;     /* Lane quality percentage by all properties, (0,
                            0~100%) */
    UINT8_T uMarkerType; /* Describes the type of a lane marker (e.g.road
                            edge,) ,(0, 0~100, -) */
    UINT8_T uColor;      /* Defines the color of a lane boundary,(0, 0~5, -) */

    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -) */
} sLMCInput_t;
#endif

#ifndef Rte_TypeDef_sLMCParam_t
#define Rte_TypeDef_sLMCParam_t
typedef struct {
    UINT8_T bUseMotionComp;    /* Switch for using lane motion compensation,(1,
                                  0~1, -) */
    REAL32_T fThdLengthSetLmc; /* Valid length threshold for set lane motion
                                  compensation, (25, 0~300, m) */
    REAL32_T
    fThdQualitySetLmc; /* Overall quality threshold for set lane motion
                          compensation, (70, 0~100, %) */
    REAL32_T
    fThdQlyChngSetLmc; /* Overall quality change threshold for set lane
                          motion compensation, (25, 0~100, %) */

    REAL32_T fThdQualityRstLmc; /* Overall quality threshold for reset lane
                                   motion compensation, (75, 0~100, %) */
    REAL32_T fThdCrvRstLmc; /* Overall quality threshold for reset lane motion
                               compensation, (75, -0.1~0.1, 1/m) */
    REAL32_T fThdLengthRstLmc; /* Max length threshold for reset lane motion
                                  compensation, (15, 0~300, m) */

    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components , (0, 0~0.001, s) */
} sLMCParam_t;
#endif

#ifndef Rte_TypeDef_sLMCOutput_t
#define Rte_TypeDef_sLMCOutput_t
typedef struct {
    REAL32_T fPosX0Cpmn; /* PosX0 after motion compensation, (0, -300~300, m) */
    REAL32_T fPosY0Cpmn; /* PosY0 after motion compensation,  (0, -15~15, m) */
    REAL32_T fHeadingCpmn; /* Heading angle after motion compensation, (0,
                              -0.7854~0.7854, rad) */
    REAL32_T
    fCrvCpmn; /* Curvature after motion compensation, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCpmn; /* Curvature rate after motion compensation, (unit,
                              1/m^2) */
    REAL32_T fValidLengthCpmn; /* Valid length after motion compensation, (0,
                                  -0.001~0.001, 1/m^2) */
    UINT8_T uMarkerTypeCpmn;   /* Marker type after motion compensation  ,(0,
                                  0~300, m) */
    UINT8_T bAvailableCpmn;    /* Lane available flag after motion compensation,
                                  (0, 0~1, -) */
    UINT8_T uColorCpmn; /* Lane color after motion compensation,(0, 0~5, -) */
    REAL32_T
    fOverallQuality; /* Lane quality percentage by all properties, (0,
                        0~100%) */
} sLMCOutput_t;
#endif

#ifndef Rte_TypeDef_sLMCDebug_t
#define Rte_TypeDef_sLMCDebug_t
typedef struct {
    UINT8_T bSetLmcByLength; /* Set flag for motion compensation by valid
                                length, (0, 0~1, -) */
    UINT8_T bSetLmcByOverallQuality; /* Set flag for motion compensation by
                                        overall quality, (0, 0~1, -) */
    UINT8_T bSetLmcByAvlChng;        /* Set flag for motion compensation by lane
                                        available change, (0, 0~1, -) */
    UINT8_T bSetLmcByQlyChng;        /* Set flag for motion compensation by lane
                                        overall quality change, (0, 0~1, -) */
    UINT8_T
    bSetLmcEnable; /* Set flag for motion compensation enable, (0, 0~1, -)
                    */
    UINT8_T bRstLmcByCrv;       /* Reset flag for motion compensation enable by
                                   curvature, (0, 0~1, -) */
    UINT8_T bRstLmcByAvlQly;    /* Reset flag for motion compensation enable by
                                   lane available and high quality, (0, 0~1, -) */
    UINT8_T bRstLmcByMaxLength; /* Reset flag for motion compensation enable by
                                   max valid length, (0, 0~1, -) */
    UINT8_T bRstLmcByPolyFit;   /* Reset flag for motion compensation enable by
                                   polynomial fitting, (0, 0~1, -) */
    UINT8_T bRstLmcByLaneChng;  /* Reset flag for motion compensation enable by
                                   lane change, (0, 0~1, -) */
    UINT8_T bRstLmcByLowQly;    /* Reset flag for motion compensation enable by
                                   poor quality, (0, 0~1, -) */
    UINT8_T bRstLmcEnable; /* Reset flag for motion compensation enable, (0,
                              0~1, -) */
    UINT8_T
    bEnaMotionCmpn; /* Lane motion compensation enable flag, (0, 0~1, -) */

    REAL32_T fDeltaPosX;    /* Increased value of position X in one task cycle,
                               (unit, rad) */
    REAL32_T fDeltaPosY;    /* Increased value of position Y in one task cycle,
                               (unit, rad) */
    REAL32_T fDeltaYaw;     /* Increased value of yaw angle in one task cycle,
                               (unit, rad) */
    UINT8_T bEnaPolyFit3rd; /* The enable flag for third order polynomial
                               fitting, (0, 0~1, -) */
    REAL32_T
    fMaxValidLengthCpmn; /* Max valid length after motion compensation, (0,
                            0~300, m) */
    REAL32_T
    fMinValidLengthCpmn;  /* Min valid length after motion compensation, (0,
                             0~300, m) */
    REAL32_T fPosY03rd;   /* Last position Y0 of Third-order polynomial fit for
                             lane motion compensation, (unit, m) */
    REAL32_T fHeading3rd; /* Last heading angle of Third-order polynomial fit
                             for lane motion compensation, (unit, m) */
    REAL32_T fCrv3rd; /* Curvature Third-order polynomial fit for lane motion
                         compensation, (unit, m) */
    REAL32_T fCrvRate3rd; /* Curvature rate of Third-order polynomial fit for
                             lane motion compensation, (unit, m) */
    REAL32_T fDevTraj3rd; /* Deviation of Third-order polynomial fit for lane
                             motion compensation, (unit, m) */
    REAL32_T fPosXRaw[POLYFIT_SAMPLE_POINTS]; /* Raw valid length sample points,
                                                 (unit, m) */
    REAL32_T fPosYRaw[POLYFIT_SAMPLE_POINTS]; /* Lateral distance by raw valid
                                                 length through third-order
                                                 polynomial fitting, (unit, m)
                                                 */
    REAL32_T
    fPosXPre[POLYFIT_SAMPLE_POINTS];          /* Valid length(After polynomial
                                                 fitting) sample points, (unit, m) */
    REAL32_T fPosYPre[POLYFIT_SAMPLE_POINTS]; /* Lateral distance by valid
                                                 length(After polynomial
                                                 fitting) through third-order
                                                 polynomial fitting, (unit, m)
                                                 */
    REAL32_T
    fPosXRot[POLYFIT_SAMPLE_POINTS]; /* Position X sample points by motion
                                        compensation, (unit, m) */
    REAL32_T
    fPosYRot[POLYFIT_SAMPLE_POINTS]; /* Position Y sample points by motion
                                        compensation, (unit, m) */
    REAL32_T fPosXArray[POLYFIT_SAMPLE_POINTS]; /* Position X sample points by
                                                   motion compensation, (unit,
                                                   m) */
    REAL32_T fPosYArray[POLYFIT_SAMPLE_POINTS]; /* Position Y sample points by
                                                   motion compensation, (unit,
                                                   m) */

    UINT8_T bEnaApplySmthByCmpn; /* Enable flag for apply smooth by motion
                                    compensation flag,(0, 0~1, -) */
    UINT8_T bEnaApplySmthByPosY; /* Enable flag for apply smooth by position Y
                                    change,(0, 0~1, -) */
    UINT8_T bEnaApplySmthByHeading; /* Enable flag for apply smooth by heading
                                       angle change,(0, 0~1, -) */
    UINT8_T bRawEnaApplySmth; /* Raw set flag of S-R Flip-Flop, (0,0~1, -) */
    UINT8_T bEnaApplySmth;    /* Set flag of S-R Flip-Flop, (0,0~1, -) */

    UINT8_T
    bRstPosY0Smth; /* Apply smooth data transition reset flag, (0, 0~1, -)
                    */
    UINT8_T bRstHeadingSmth; /* Apply smooth data transition reset flag, (0,
                                0~1, -) */
    UINT8_T
    bRstCrvSmth; /* Apply smooth data transition reset flag, (0, 0~1, -) */
    UINT8_T bRstCrvRateSmth; /* Apply smooth data transition reset flag, (0,
                                0~1, -) */
    UINT8_T
    bEnaPosY0Smth; /* Apply smooth data transition enable flag, (0, 0~1, -)
                    */
    UINT8_T bEnaHeadingSmth; /* Apply smooth data transition enable flag, (0,
                                0~1, -) */
    UINT8_T
    bEnaCrvSmth; /* Apply smooth data transition enable flag, (0, 0~1, -) */
    UINT8_T bEnaCrvRateSmth; /* Apply smooth data transition enable flag, (0,
                                0~1, -) */
} sLMCDebug_t;
#endif

#ifndef Rte_TypeDef_sULBInput_t
#define Rte_TypeDef_sULBInput_t
typedef struct {
    REAL32_T
    fPosY0UnCplLf; /* UnUncoupled left lane lateral distance at X = 0.0 m,
                      (0, -15~15, m) */
    REAL32_T fHeadingUnCplLf; /* Uncoupled left lane heading angle (Yaw angle)
                                 of a lane boundary track, (0, -0.7854~0.7854,
                                 rad) */
    REAL32_T fCrvUnCplLf; /* Uncoupled left lane curvature of a lane boundary
                             track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUnCplLf; /* Uncoupled left lane curvature rate of a lane
                                 boundary track, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUnCplLf; /* Uncoupled left lane visibility range of a
                                     lane boundary track, (0, 0~300, m) */
    UINT8_T
    bAvailableUnCplLf; /* Uncoupled left lane available flag, (0, 0~1, -) */

    REAL32_T
    fPosY0UnCplRi; /* Uncoupled Right lane lateral distance at X = 0.0 m,
                      (0, -15~15, m) */
    REAL32_T fHeadingUnCplRi; /* Uncoupled Right lane heading angle (Yaw angle)
                                 of a lane boundary track, (0, -0.7854~0.7854,
                                 rad) */
    REAL32_T fCrvUnCplRi; /* Uncoupled Right lane curvature of a lane boundary
                             track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateUnCplRi; /* Uncoupled Right lane curvature rate of a lane
                                 boundary track, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthUnCplRi; /* Uncoupled Right lane visibility range of a
                                     lane boundary track, (0, 0~300, m) */
    UINT8_T
    bAvailableUnCplRi; /* Uncoupled Right lane available flag, (0, 0~1, -)
                        */

    REAL32_T fVehYawRate; /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (0, -20~150,
                          m/s) */
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */

    UINT8_T bDistYStepDtctLf;
    UINT8_T bDistYStepDtctRi;
    UINT8_T bUpDownHillDegrade;

    UINT8_T uPercExitLf; /* Exit percent for left side, (0, 0~100, %) */
    UINT8_T uPercExitRi; /* Exit percent for right side, (0, 0~100, %) */

    UINT8_T bLineMergeDtcRi; /* Right line is merged (0, 0~1 ) */
    UINT8_T bLineMergeDtcLf; /* Left line is merged (0, 0~1 ) */
} sULBInput_t;
#endif

#ifndef Rte_TypeDef_sULBParam_t
#define Rte_TypeDef_sULBParam_t
typedef struct {
    REAL32_T fThdLaneWidthDiff; /* Lane width difference threshold for exit
                                   detection, (1, 0~10, m) */
    REAL32_T
    fTdRawNotParallel;         /* Turn on delay time for raw not parallel flag,
                                  (0.42, 0~60, s) */
    REAL32_T fThdLatDistDev;   /* Lateral distance deviation threshold for
                                  uncoupled lane bridge, (0.3, 0~5, m) */
    UINT8_T bUseStepDtctUnCpl; /* Determines uncoupled lane step detection
                                  usage, (1, 0~1, -) */
    REAL32_T fThdHeadingDev; /* Heading angle deviation threshold for uncoupled
                                lane bridge, (0.15, 0~1, rad) */
    REAL32_T fThdCrvDev;     /* Curvature deviation threshold for uncoupled lane
                                bridge, (0.001, 0~0.1, 1/m) */
    REAL32_T
    fMaxCrvBridgeUnCpl; /* Uncoupled lane bridging curvature reset RSP
                           (upper hysteresis) value, (0.001, 0~0.1, 1/m) */
    REAL32_T
    fMinCrvBridgeUnCpl; /* Uncoupled lane bridging curvature reset LSP
                           (Lower hysteresis) value, (5e-4, 0~0.1, 1/m) */

    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components ,(0, 0.001~0.1, s)
                    */
} sULBParam_t;
#endif

#ifndef Rte_TypeDef_sULBOutput_t
#define Rte_TypeDef_sULBOutput_t
typedef struct {
    UINT8_T bBridgeUnCplLf;  /* Left lane uncoupled lane bridging possible, (0,
                                0~1, -) */
    UINT8_T bBridgeUnCplRi;  /* Left lane uncoupled lane bridging possible, (0,
                                0~1, -) */
    UINT8_T uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0, 0~100, %) */
} sULBOutput_t;
#endif

#ifndef Rte_TypeDef_sULBDebug_t
#define Rte_TypeDef_sULBDebug_t
typedef struct {
    REAL32_T fLaneWidthUnCpl;   /* Lane width at X = 0.0m, (0, 0~10, m) */
    REAL32_T fPosYAtMaxUnCplLf; /* Left lane lateral distance at max raw valid
                                   length (0, -15~15, m) */
    REAL32_T
    fPosYAtMaxUnCplRi; /* Right lane lateral distance at max raw valid
                          length (0, -15~15, m) */
    REAL32_T
    fRotPosYUnCplLf; /* Rotate uncoupled left lane PosY at max length, (0,
                        -15~15, m) */
    REAL32_T
    fRotPosYUnCplRi; /* Rotate uncoupled right lane PosY at max length, (0,
                        -15~15, m) */
    REAL32_T fLaneWidthAtMaxUnCpl; /* Lane width at max raw valid length, (0,
                                      0~10, m) */
    REAL32_T
    fDiffLaneWidthUnCpl; /* uncoupled lane width difference, (0, 0~10, m) */
    UINT8_T bRawNotParallelUnCpl; /* Raw flag that uncoupled lanes are not
                                     parallel, (0, 0~1, -) */
    UINT8_T bDlyNotParallelUnCpl; /* Flag after turn on delay that uncoupled
                                     lanes are not parallel, (0, 0~1, -) */
    UINT8_T bNotParallelUnCpl;    /* Flag that uncoupled lanes are not parallel,
                                     (0, 0~1, -) */
    REAL32_T fTimerRawNotParallel; /* Timer for Raw not parallel flag turn on
                                      delay, (0, 0~60, s) */
    REAL32_T fDeltaYawCplLf;
    REAL32_T fDeltaXUnCplLf;
    REAL32_T fDeltaYUnCplLf;
    REAL32_T fDeltaYawCplRi;
    REAL32_T fDeltaXUnCplRi;
    REAL32_T fDeltaYUnCplRi;
    REAL32_T fUnitDeltaXUnCplLf;
    REAL32_T fUnitDeltaYUnCplLf;
    REAL32_T fUnitDeltaYawCplRi;
    REAL32_T fUnitDeltaYawCplLf;
    REAL32_T fUnitDeltaXUnCplRi;
    REAL32_T fUnitDeltaYUnCplRi;
    REAL32_T
    fDlyPosXUnCplLf; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T
    fDlyPosYUnCplLf; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T
    fDlyPosXUnCplRi; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T
    fDlyPosYUnCplRi; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T
    fCpmnPosXUnCplLf; /* Position X after delay 8 cycles, (0, -300~300, m)
                       */
    REAL32_T
    fCpmnPosYUnCplLf; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T
    fCpmnPosXUnCplRi; /* Position X after delay 8 cycles, (0, -300~300, m)
                       */
    REAL32_T
    fCpmnPosYUnCplRi; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fRotCmpnPosYUnCplLf; /* Position X after delay 8 cycles, (0,
                                     -15~15, m) */
    REAL32_T fRotCmpnPosYUnCplRi; /* Position X after delay 8 cycles, (0,
                                     -15~15, m) */
    REAL32_T fDevLatDistCplLf;
    REAL32_T fDevLatDistCplRi;
    REAL32_T fDevLatDistDev;
    UINT8_T bTrigLatDistDev;
    REAL32_T fPenaltyPosY0UnCplLf;
    REAL32_T fPenaltyHeadingUnCplLf;
    REAL32_T fPenaltyCrvUnCplLf;
    REAL32_T fPenaltyCrvRateUnCplLf;
    REAL32_T fRawPenaltyUnCplLf;
    REAL32_T fPenaltyUnCplLf;

    REAL32_T fPenaltyPosY0UnCplRi;
    REAL32_T fPenaltyHeadingUnCplRi;
    REAL32_T fPenaltyCrvUnCplRi;
    REAL32_T fPenaltyCrvRateUnCplRi;
    REAL32_T fRawPenaltyUnCplRi;
    REAL32_T fPenaltyUnCplRi;
    UINT8_T bRawStepDtctUnCplLf;
    UINT8_T bRawStepDtctUnCplRi;
    UINT8_T bStepDtctUnCplLf;
    UINT8_T bStepDtctUnCplRi;

    REAL32_T fHeadingPolyUnCplLf;
    REAL32_T fHeadingPolyUnCplRi;
    REAL32_T fDevHeadingUnCplLf;
    REAL32_T fDevHeadingUnCplRi;
    REAL32_T fDevHeadingDevUnCpl;
    UINT8_T bTrigHeadingDevUnCpl;
    REAL32_T fCrvRefUnCplLf;
    REAL32_T fCrvRefUnCplRi;
    REAL32_T fDevCrvUnCplLf;
    REAL32_T fDevCrvUnCplRi;
    REAL32_T fDevCrvDevUnCpl;
    UINT8_T bTrigCrvDevUnCpl;

    UINT8_T bNotParallelBridgeUnCplLf;
    UINT8_T bSetBridgeUnCplLf;
    UINT8_T bResetBridgeUnCplLf;
    UINT8_T bDlySetBridgeUnCplLf;
    REAL32_T fTimerSetUnCplLf;
    UINT8_T bResetByCrvUnCplLf;
    UINT8_T bNotParallelBridgeUnCplRi;
    UINT8_T bSetBridgeUnCplRi;
    UINT8_T bResetBridgeUnCplRi;
    UINT8_T bDlySetBridgeUnCplRi;
    REAL32_T fTimerSetUnCplRi;
    UINT8_T bResetByCrvUnCplRi;
    REAL32_T fTempULB;
    UINT8_T bTempULB;

} sULBDebug_t;
#endif

#ifndef Rte_TypeDef_sDLBInput_t
#define Rte_TypeDef_sDLBInput_t
typedef struct {
    UINT8_T bUpDownHillDegr;

    UINT8_T bBridgeUnCplLf;
    REAL32_T fOverallQualityUnCplLf;

    UINT8_T bBridgeUnCplRi;
    REAL32_T fOverallQualityUnCplRi;

    REAL32_T fPosX0CplLf; /* Left lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0CplLf; /* Left lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingCplLf;      /* Left lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvCplLf; /* Left lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateCplLf; /* Left lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCplLf; /* Left lane valid length, (0, 0~300, m) */
    REAL32_T fQualityCplLf;     /* Left lane quality, (0, 0~300, m) */
    UINT8_T
    uLaneTypeCplLf; /* Left lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeCplLf; /* Left lane event type(e.g. construction site)
                                ,(0, 0~8, -) */
    UINT8_T bAvailableCplLf; /* Defines whether left lane is available or not,
                                (0, 0~1, -) */
    UINT8_T
    uEventQualityCplLf; /* The quality of left lane event ,(0, 0~100, %) */
    REAL32_T
    fEventDistanceCplLf; /* The distance to left lane event in meters, (0,
                            0~200, m) */
    REAL32_T fStdDevPosY0CplLf; /* Standard deviation of the lateral distance of
                                   lane marker tracks, (0,0~5, m) */
    REAL32_T
    fStdDevHeadingCplLf;          /* Standard deviation of the yaw angle of left
                                     lane, (0, 0~1, rad) */
    REAL32_T fStdDevCrvCplLf;     /* Standard deviation of the curvature of left
                                     lane, (0, 0~0.5, 1/m) */
    REAL32_T fStdDevCrvRateCplLf; /* Standard deviation of the curvature rate of
                                     left lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorCplLf;          /* Left lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityCplLf; /* Left lane quality percentage by all
                                      properties, (0~100%, ULP) */

    REAL32_T fPosX0CplRi; /* Right lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0CplRi; /* Right lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingCplRi; /* Right lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvCplRi; /* Right lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateCplRi; /* Right lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCplRi; /* Right lane valid length, (0, 0~300, m) */
    REAL32_T fQualityCplRi;     /* Right lane quality, (0, 0~300, m) */
    UINT8_T
    uLaneTypeCplRi; /* Right lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeCplRi; /* Right lane event type(e.g. construction site)
                                ,(0, 0~8, -) */
    UINT8_T bAvailableCplRi; /* Defines whether right lane is available or not,
                                (0, 0~1, -) */
    UINT8_T
    uEventQualityCplRi; /* The quality of right lane event ,(0, 0~100, %) */
    REAL32_T
    fEventDistanceCplRi; /* The distance to right lane event in meters, (0,
                            0~200, m) */
    REAL32_T fStdDevPosY0CplRi; /* Standard deviation of the lateral distance of
                                   lane marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeadingCplRi; /* Standard deviation of the yaw angle of
                                     right lane, (0, 0~1, rad) */
    REAL32_T fStdDevCrvCplRi; /* Standard deviation of the curvature of right
                                 lane, (0, 0~0.5, 1/m) */
    REAL32_T
    fStdDevCrvRateCplRi; /* Standard deviation of the curvature rate of
                            right lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorCplRi; /* Right lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityCplRi; /* Right lane quality percentage by all
                                      properties, (0~100%, ULP) */
} sDLBInput_t;
#endif

#ifndef Rte_TypeDef_sDLBParam_t
#define Rte_TypeDef_sDLBParam_t
typedef struct {
    UINT8_T bUseLaneDynWeight;   /* Use dynamic lane weighting of
                                    coupled/uncoupled lane, (1, 0~1, -) */
    REAL32_T fRefQualityFadeFac; /* Fading factor for dynamic coupled/uncoupled
                                    lane weighting, (25, 0~100, -) */

    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components ,(0, 0.001~0.1, s)
                    */

} sDLBParam_t;
#endif

#ifndef Rte_TypeDef_sDLBOutput_t
#define Rte_TypeDef_sDLBOutput_t
typedef struct {
    REAL32_T fPosX0Lf; /* Left lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0Lf; /* Left lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingLf;      /* Left lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvLf; /* Left lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateLf; /* Left lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthLf; /* Left lane valid length, (0, 0~300, m) */
    REAL32_T fQualityLf;     /* Left lane quality, (0, 0~300, m) */
    UINT8_T uLaneTypeLf;  /* Left lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeLf; /* Left lane event type(e.g. construction site) ,(0,
                             0~8, -) */
    UINT8_T bAvailableLf; /* Defines whether left lane is available or not, (0,
                             0~1, -) */
    UINT8_T uEventQualityLf; /* The quality of left lane event ,(0, 0~100, %) */
    REAL32_T
    fEventDistanceLf;          /* The distance to left lane event in meters, (0,
                                  0~200, m) */
    REAL32_T fStdDevPosY0Lf;   /* Standard deviation of the lateral distance of
                                  lane marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeadingLf; /* Standard deviation of the yaw angle of left
                                  lane, (0, 0~1, rad) */
    REAL32_T
    fStdDevCrvLf; /* Standard deviation of the curvature of left lane, (0,
                     0~0.5, 1/m) */
    REAL32_T fStdDevCrvRateLf;  /* Standard deviation of the curvature rate of
                                   left lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorLf;           /* Left lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityLf; /* Left lane quality percentage by all
                                   properties, (0~100%, ULP) */

    REAL32_T fPosX0Ri; /* Right lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0Ri; /* Right lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingRi;      /* Right lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvRi; /* Right lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateRi; /* Right lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthRi; /* Right lane valid length, (0, 0~300, m) */
    REAL32_T fQualityRi;     /* Right lane quality, (0, 0~300, m) */
    UINT8_T uLaneTypeRi;  /* Right lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeRi; /* Right lane event type(e.g. construction site) ,(0,
                             0~8, -) */
    UINT8_T
    bAvailableRi; /* Defines whether right lane is available or not, (0,
                     0~1, -) */
    UINT8_T
    uEventQualityRi; /* The quality of right lane event ,(0, 0~100, %) */
    REAL32_T fEventDistanceRi; /* The distance to right lane event in meters,
                                  (0, 0~200, m) */
    REAL32_T fStdDevPosY0Ri;   /* Standard deviation of the lateral distance of
                                  lane marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeadingRi; /* Standard deviation of the yaw angle of right
                                  lane, (0, 0~1, rad) */
    REAL32_T
    fStdDevCrvRi; /* Standard deviation of the curvature of right lane, (0,
                     0~0.5, 1/m) */
    REAL32_T fStdDevCrvRateRi;  /* Standard deviation of the curvature rate of
                                   right lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorRi;           /* Right lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityRi; /* Right lane quality percentage by all
                                   properties, (0~100%, ULP) */

} sDLBOutput_t;
#endif

#ifndef Rte_TypeDef_sDLBDebug_t
#define Rte_TypeDef_sDLBDebug_t
typedef struct {
    REAL32_T
    fSysCycleTime; /* Cycle time for LCF_SEN components ,(0, 0.001~0.1, s)
                    */

} sDLBDebug_t;
#endif

#ifndef Rte_TypeDef_sULPInput_t
#define Rte_TypeDef_sULPInput_t
typedef struct {
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -) */
    UINT8_T bUpDnHillDgrd;   /* Flag that indicates downhill/uphill degrade, (0,
                                0~1, -) */

    REAL32_T fPosY0CplLf;   /* Lateral distance at X = 0.0 m, (0, -15~15, m) */
    REAL32_T fHeadingCplLf; /* Heading angle (Yaw angle) of a lane boundary
                               track, (0, -0.7854~0.7854, rad) */
    REAL32_T
    fCrvCplLf; /* Curvature of a lane boundary track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCplLf;     /* Curvature rate of a lane boundary track, (0,
                                   -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCplLf; /* Visibility range of a lane boundary track,
                                   (0, 0~300, m) */
    UINT8_T bAvailableCplLf;    /* Defines whether a lane boundary track is
                                   available or not, (0, 0~1, -) */
    REAL32_T fStdDevPosY0CplLf; /* Standard deviation of the lateral distance of
                                   lane marker tracks, (0,0~5, m) */
    REAL32_T
    fStdDevHeadingCplLf;      /* Standard deviation of the yaw angle of lane
                                 marker tracks, (0, 0~1, rad) */
    REAL32_T fStdDevCrvCplLf; /* Standard deviation of the curvature, (0, 0~0.5,
                                 1/m) */
    REAL32_T fStdDevCrvRateCplLf; /* Standard deviation of the curvature rate,
                                     (0, 0~1, 1/m^2) */

    REAL32_T fPosY0CplRi;   /* Lateral distance at X = 0.0 m, (0, -15~15, m) */
    REAL32_T fHeadingCplRi; /* Heading angle (Yaw angle) of a lane boundary
                               track, (0, -0.7854~0.7854, rad) */
    REAL32_T
    fCrvCplRi; /* Curvature of a lane boundary track, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCplRi;     /* Curvature rate of a lane boundary track, (0,
                                   -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthCplRi; /* Visibility range of a lane boundary track,
                                   (0, 0~300, m) */
    UINT8_T bAvailableCplRi;    /* Defines whether a lane boundary track is
                                   available or not, (0, 0~1, -) */
    REAL32_T fStdDevPosY0CplRi; /* Standard deviation of the lateral distance of
                                   lane marker tracks, (0,0~5, m) */
    REAL32_T
    fStdDevHeadingCplRi;      /* Standard deviation of the yaw angle of lane
                                 marker tracks, (0, 0~1, rad) */
    REAL32_T fStdDevCrvCplRi; /* Standard deviation of the curvature, (0, 0~0.5,
                                 1/m) */
    REAL32_T fStdDevCrvRateCplRi; /* Standard deviation of the curvature rate,
                                     (0, 0~1, 1/m^2) */

    //                                      /* LMC */
    REAL32_T
    fQualityCplLf; /* Quality of a lane boundary track, (0, 0~100, -) */
    REAL32_T
    fQualityCplRi; /* Quality of a lane boundary track, (0, 0~100, -) */

    REAL32_T fVehVelX; /* Vehicle speed based on the wheel speeds , (0, -20~150,
                          m/s) */
    REAL32_T fVehYawRate; /* 'Ego Vehicle yaw rate (VED), (0, -1~1, rad/s) */
    UINT8_T uMarkerTypeCplLf; /* Describes the type of a lane marker (e.g.road
                                 edge,) ,(0, 0~100, -) */
    UINT8_T uMarkerTypeCplRi; /* Describes the type of a lane marker (e.g.road
                                 edge,) ,(0, 0~100, -) */
    UINT8_T uColorCplLf; /* Defines the color of a lane boundary,(0, 0~5, -) */
    UINT8_T uColorCplRi; /* Defines the color of a lane boundary,(0, 0~5, -) */

    /* ULB */
    UINT8_T uPercExitLf; /* Exit percent for left side, (0, 0~100, %) */
    UINT8_T uPercExitRi; /* Exit percent for right side, (0, 0~100, %) */

    /* DLB */
    UINT8_T uEventTypeCplLf; /* Left lane event type(e.g. construction site)
                                ,(0, 0~8, -) */
    UINT8_T uEventTypeCplRi; /* Right lane event type(e.g. construction site)
                                ,(0, 0~8, -) */
    UINT8_T
    uEventQualityCplLf; /* The quality of left lane event ,(0, 0~100, %) */
    UINT8_T
    uEventQualityCplRi; /* The quality of right lane event ,(0, 0~100, %) */
    REAL32_T
    fEventDistanceCplLf; /* The distance to left lane event in meters, (0,
                            0~200, m) */
    REAL32_T
    fEventDistanceCplRi; /* The distance to right lane event in meters, (0,
                            0~200, m) */

    UINT8_T bDistYStepDtctLf; /* Left lane DistY step detected */
    UINT8_T bDistYStepDtctRi; /* Right lane DistY step detected */

    UINT8_T bLineMergeDtcRi; /* Right line is merged (0, 0~1 ) */
    UINT8_T bLineMergeDtcLf; /* Left line is merged (0, 0~1 ) */
} sULPInput_t;
#endif

#ifndef Rte_TypeDef_sULPParam_t
#define Rte_TypeDef_sULPParam_t
typedef struct {
    UINT8_T uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0, 0~100, %) */

    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(0.06,
                               0.001~0.1, s) */

} sULPParam_t;
#endif

#ifndef Rte_TypeDef_sULPOutput_t
#define Rte_TypeDef_sULPOutput_t
typedef struct {
    REAL32_T fPosX0Lf; /* Left lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0Lf; /* Left lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingLf;      /* Left lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvLf; /* Left lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateLf; /* Left lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthLf; /* Left lane valid length, (0, 0~300, m) */
    REAL32_T fQualityLf;     /* Left lane quality, (0, 0~300, m) */
    UINT8_T uLaneTypeLf;  /* Left lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeLf; /* Left lane event type(e.g. construction site) ,(0,
                             0~8, -) */
    UINT8_T bAvailableLf; /* Defines whether left lane is available or not, (0,
                             0~1, -) */
    UINT8_T uEventQualityLf; /* The quality of left lane event ,(0, 0~100, %) */
    REAL32_T
    fEventDistanceLf;          /* The distance to left lane event in meters, (0,
                                  0~200, m) */
    REAL32_T fStdDevPosY0Lf;   /* Standard deviation of the lateral distance of
                                  lane marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeadingLf; /* Standard deviation of the yaw angle of left
                                  lane, (0, 0~1, rad) */
    REAL32_T
    fStdDevCrvLf; /* Standard deviation of the curvature of left lane, (0,
                     0~0.5, 1/m) */
    REAL32_T fStdDevCrvRateLf;  /* Standard deviation of the curvature rate of
                                   left lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorLf;           /* Left lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityLf; /* Left lane quality percentage by all
                                   properties, (0~100%, ULP) */

    REAL32_T fPosX0Ri; /* Right lane X0 position , (0, 0~300, m) */
    REAL32_T fPosY0Ri; /* Right lane Y0 position , (0, -15~15, m) */
    REAL32_T
    fHeadingRi;      /* Right lane heading angle , (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvRi; /* Right lane curvature, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateRi; /* Right lane curvature rate, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fValidLengthRi; /* Right lane valid length, (0, 0~300, m) */
    REAL32_T fQualityRi;     /* Right lane quality, (0, 0~300, m) */
    UINT8_T uLaneTypeRi;  /* Right lane type(eg,9 means roadedge), (0~255, -) */
    UINT8_T uEventTypeRi; /* Right lane event type(e.g. construction site) ,(0,
                             0~8, -) */
    UINT8_T
    bAvailableRi; /* Defines whether right lane is available or not, (0,
                     0~1, -) */
    UINT8_T
    uEventQualityRi; /* The quality of right lane event ,(0, 0~100, %) */
    REAL32_T fEventDistanceRi; /* The distance to right lane event in meters,
                                  (0, 0~200, m) */
    REAL32_T fStdDevPosY0Ri;   /* Standard deviation of the lateral distance of
                                  lane marker tracks, (0,0~5, m) */
    REAL32_T fStdDevHeadingRi; /* Standard deviation of the yaw angle of right
                                  lane, (0, 0~1, rad) */
    REAL32_T
    fStdDevCrvRi; /* Standard deviation of the curvature of right lane, (0,
                     0~0.5, 1/m) */
    REAL32_T fStdDevCrvRateRi;  /* Standard deviation of the curvature rate of
                                   right lane, (0, 0~1, 1/m^2) */
    UINT8_T uColorRi;           /* Right lane color, (0, 0~5, -) */
    REAL32_T fOverallQualityRi; /* Right lane quality percentage by all
                                   properties, (0~100%, ULP) */

    REAL32_T
    fOverallCrvQualityLf;          /* Quality of the left curvature information,
                                      (0~255, %, ELG) */
    REAL32_T fOverallCrvQualityRi; /* Quality of the right curvature
                                      information, (0~255, % ELG) */

    UINT8_T bBridgePossibleUnCplLf; /* left lane uncoupled lane bridging
                                       possible, (0, 0~1, -) */
    UINT8_T bBridgePossibleUnCplRi; /* Right lane uncouplded bridge possible,
                                       (0, 0~1, -) */
    REAL32_T
    fOverallQualityUnCplLf; /* Left uncoupled lane quality, (0, 0~100, %) */
    REAL32_T
    fOverallQualityUnCplRi; /* Right uncoupled lane quality, (0, 0~100, %)
                             */
    REAL32_T
    fOverallQualityCplLf; /* Left coupled lane quality, (0, 0~100, %) */
    REAL32_T
    fOverallQualityCplRi;    /* Right coupled lane quality, (0, 0~100, %) */
    UINT8_T uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0, 0~255, -) */

} sULPOutput_t;
#endif

#ifndef Rte_TypeDef_sULPDebug_t
#define Rte_TypeDef_sULPDebug_t
typedef struct {
    /* */

    /* For data collection */
    sDMQInput_t sDMQInputCplLf;
    sDMQParam_t sDMQParamCplLf;
    sDMQOutput_t sDMQOutputCplLf;
    sDMQDebug_t sDMQDebugCplLf;

    sDMQInput_t sDMQInputCplRi;
    sDMQParam_t sDMQParamCplRi;
    sDMQOutput_t sDMQOutputCplRi;
    sDMQDebug_t sDMQDebugCplRi;

    sLMCInput_t sLMCInputCplLf;
    sLMCParam_t sLMCParamCplLf;
    sLMCOutput_t sLMCOutputCplLf;
    sLMCDebug_t sLMCDebugCplLf;

    sLMCInput_t sLMCInputCplRi;
    sLMCParam_t sLMCParamCplRi;
    sLMCOutput_t sLMCOutputCplRi;
    sLMCDebug_t sLMCDebugCplRi;

    sULBInput_t sULBInput;
    sULBParam_t sULBParam;
    sULBOutput_t sULBOutput;
    sULBDebug_t sULBDebug;

    sDLBInput_t sDLBInput;
    sDLBParam_t sDLBParam;
    sDLBOutput_t sDLBOutput;
    sDLBDebug_t sDLBDebug;
} sULPDebug_t;
#endif

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_TYPEDEF_H_
