/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_TYPEDEF_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_TYPEDEF_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "tue_common_libs.h"

/****************************2.Check basic lane boundary
 * properites**********************/

/***************************************************************************************************************************************/

/****************************2.Check basic lane boundary
 * properites**********************/
#ifndef Rte_TypeDef_sCLPInput_t
#define Rte_TypeDef_sCLPInput_t
typedef struct {
    // Raw lane width input
    UINT8_T bAvailableUnCplLf;  // Available flag of uncoupled left lane, (0,
                                //    0~1, -, )
    UINT8_T bAvailableUnCplRi;  // Available flag of uncoupled right lane, (0,
                                //    0~1,-, )
    UINT8_T
    bAvailableCplLf;  // Available flag of coupled left lane, (0, 0~1, -, )
    UINT8_T
    bAvailableCplRi;  // Available flag of coupled right lane, (0, 0~1,-, )
    REAL32_T fPosY0UnCplLf;  // Lateral distance of uncoupled left lane, (0,
                             //  -15~15, m)
    REAL32_T fPosY0UnCplRi;  // Lateral distance of uncoupled right lane, (0,
                             //     -15~15, m)
    REAL32_T
    fPosY0CplLf;  // Lateral distance of coupled left lane, (0, -15~15, m)
    REAL32_T
    fPosY0CplRi;  // Lateral distance of coupled right lane, (0, -15~15, m)
    REAL32_T fHeadingUnCplLf;  // Heading angle of uncoupled left lane, (0,
                               //   -0.7854~0.7854, rad)
    REAL32_T fHeadingUnCplRi;  // Heading angle of uncoupled right lane, (0,
                               //   -0.7854~0.7854, rad)
    REAL32_T fHeadingCplLf;    // Heading angle of coupled left lane, (0,
                               //     -0.7854~0.7854, rad)
    REAL32_T fHeadingCplRi;    // Heading angle of coupled right lane, (0,
                               //     -0.7854~0.7854, rad)
    REAL32_T fValidLengthUnCpLf;
    REAL32_T fValidLengthUnCpRi;
    REAL32_T fCrvUnCpLf;
    REAL32_T fCrvUnCpRi;
    REAL32_T fCrvRateUnCpLf;
    REAL32_T fCrvRateUnCpRi;

    // DSC input
    REAL32_T
    fVertSlopeChange;  // Defines slope change change state��(0, -10~10,  -)

    REAL32_T fSineWaveDtct;  // Flag from ABD interface that indicates sine wave
                             //     road condition, (0, -10~10, -)
    REAL32_T fUpDownHillDtct;  // Indicates downhill/uphill scenario detection
                               //   confidence, (0, -10~10, -)
    // Not define
    REAL32_T
    fVehVelX;  // Vehicle speed based on the wheel speeds , (0, -20~150,
               //    m/s) */

    // LSD input */
    //  REAL32_T fVehVelX;            /* Vehicle speed based on the wheel speeds
    //  , (0, -20~150, m/s) */
    UINT8_T uQualityUlpLf;    // Left lane quality by ULP, (0, 0~100, -) */
    UINT8_T bAvailableUlpLf;  // Left lane available flag by ULP, (0, 0~1, -) */
    REAL32_T
    fValidLengthUlpLf;       // Left lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fPosY0UlpLf;    // Left lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fHeadingUlpLf;  // Left lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fCrvUlpLf;  // Left lane curvature by ULP, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateUlpLf;  // Left lane curvature by ULP, (0, -0.1~0.1, 1/m) */
    REAL32_T fOverallQualityLf;  // Left lane Quality based on all properties,
                                 //    (0, 0~100, %)*/

    UINT8_T uQualityUlpRi;  // Right lane quality by ULP, (0, 0~100, -) */
    UINT8_T
    bAvailableUlpRi;  // Right lane available flag by ULP, (0, 0~1, -) */
    REAL32_T
    fValidLengthUlpRi;       // Right lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fPosY0UlpRi;    // Right lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fHeadingUlpRi;  // Right lane valid length by ULP, (0, 0~1, -) */
    REAL32_T fCrvUlpRi;  // Right lane curvature by ULP, (0, -0.1~0.1, 1/m) */
    REAL32_T
    fCrvRateUlpRi;  // Right lane curvature by ULP, (0, -0.1~0.1, 1/m) */
    REAL32_T fOverallQualityRi;  // Right lane Quality based on all properties,
                                 //    (0, 0~100, %)*/

    UINT8_T uMarkerTypeLf;  // Left lane marker type by ULP (e.g.road edge,��)
                            //   ,(0, 0~100, -) */
    UINT8_T uMarkerTypeRi;  // Right lane marker type by ULP (e.g.road edge,��)
                            //   ,(0, 0~100, -) */
    UINT8_T uLaneColorLf;  // Left lane color by ULP (e.g.road edge,��) ,(0,
                           //  0~100, -) */
    UINT8_T uLaneColorRi;  // Right lane color by ULP (e.g.road edge,��) ,(0,
                           //  0~100, -) */

    REAL32_T
    fCurvature;  // The Curvature of a lane boundary track, (unit, 1/m) */
    REAL32_T fCurvatureRate;  // The curvature rate of a lane boundary track,
                              // (unit, 1/m^2) */

    UINT8_T
    bAvailableLf;  // Available flag of a lane boundary track, (unit, -, ) */
    UINT8_T
    bAvailableRi;  // Available flag of a lane boundary track, (unit, -, ) */

    REAL32_T fLastPosY0;  // The last distance given is the value of the
                          // polynomial evaluated at X = 0.0 m, (unit, m) */
    REAL32_T
    fLastHeadingAngle;  // The last Heading Angle��Yaw angle�� of a lane
                        //   boundary track, (unit, radians) */
    REAL32_T fLastCurvature;  // The last Curvature of a lane boundary track,
                              // (unit, 1/m) */
    REAL32_T fLastCurvatureRate;  // The last curvature rate of a lane boundary
                                  // track, (unit, 1 / m ^ 2) * /
    UINT8_T bLastAvailable;       // Last Available flag of
                                  // a lane boundary track,
                                  //    (unit, -, ) */

    REAL32_T fLastCoeffQualityPenalty;  // The last lane line quality penalty
                                        //   coefficient by step detection after
                                        //   rate limit, (0~10, -, )*/

    REAL32_T
    fStdDevPosY0; /* Standard deviation of the lateral distance of lane
                     marker tracks, (unit, m, In12, ) */
    REAL32_T
    fStdDevHeadingAngle;       /* Standard deviation of the yaw angle of lane
                                  marker tracks, (unit, radians, In13, ) */
    REAL32_T fStdDevCurvature; /* Standard deviation of the curvature, (unit,
                                  1/m, In14, ) */
    REAL32_T
    fStdDevCurvatureRate; /* Standard deviation of the curvature rate,
                             (unit, 1/m^2, In15, ) */
    REAL32_T
    fValidLength; /* Visibility range of a lane boundary track, (unit, m) */

    UINT8_T uRoadWorks;   /* ABD road works bitfield,(unit, -) */
    UINT8_T uWeatherCond; /* Camera blockage (CB) weather condition enum
                             ��(0~255, unit, -)  */
    UINT8_T
    uCompState; /* Lane detection component state enumeration , (unit, -) */
    UINT8_T bUpDownHillCritical; /* XXX , (unit, -)  */
    UINT8_T uLaneChange; /* Defines whether a lane change has been detected or
                            not, (1,0~3, -) */
    /* (0:unknown,1:left change, 2:right change, 3:unchanged).*/
    UINT8_T bLaneBridgeLf; /*xxx,(unit, -) */
    UINT8_T bLaneBridgeRi; /*xxx,(unit, -) */
    UINT8_T bEnaByBridgePossible;
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -) */

    /* CLV */
    REAL32_T fLaneWidth; /* Lane width from LFP, (0, 0~10, m, LFP) */

    UINT8_T bLatDistDevLf; /* Left lane lateral distance deviation flag */
    UINT8_T bLatDistDevRi; /* Right lane lateral distance deviation flag */

    } sCLPInput_t;
#endif

#ifndef Rte_TypeDef_sCLPParam_t
#define Rte_TypeDef_sCLPParam_t
typedef struct {
    REAL32_T fDefaultLaneWidth;       /* Default Lane Width, (2.5, 0~10, m) */
    UINT8_T uConstructionSiteBitmask; /* Bit mask determines the ABD
                                         construction site information
                                         usage,(0,0~255, -) */
    UINT8_T uConstLaneChangeLf;       /* Constant for left lane change has been
                                         detected, (1,0~3, -) */
    UINT8_T uConstLaneChangeRi;       /* Constant for right lane change has been
                                         detected, (2,0~3, -) */

    UINT8_T uWeatherCondBitmask; /* Bit mask selects weather condition
                                    information for evaluation, (0,0~255, -) */
    UINT8_T
    bUseWeatherCond; /* Use CB weather condition information, (0,0~1, -) */

    UINT8_T uCompStateEnum; /* Lane detection component state enumeration,
                               (0,0~6, -) */
    UINT8_T
    bUseCompState; /* Decide to use LD algo component state information,
                      (0,0~1, -) */

    /* CLV */
    REAL32_T
    fDistYLimitStepDtct; /* Threshold for step detection in lane marker
                            positions, (0.17, 0~5, m) */
    REAL32_T fTurnOnTimeDistYStep; /* Debounce time for lateral distance step
                                      detection, (1, 0~60s, s) */
    REAL32_T
    fMaxLaneWidth; /* Maximum valid lane width for LCF ,(4.8, 0~10, m) */
    REAL32_T
    fMaxLaneWidthHyst; /* Lane width hysteresis for maximum lane width
                          ,(0.2, 0~10, m) */

    REAL32_T
    fMinLaneWidth; /* Minimum valid lane width for LCF ,(2.5, 0~10, m) */
    REAL32_T
    fMinLaneWidthHyst;        /* Lane width hysteresis for maximum lane width
                                 ,(0.2, 0~10, m) */
    REAL32_T TolRangeNewCorr; /* The corridor is considered new if the width
                                 changes more than this value,(0.25, 0~5, m) */
    REAL32_T TolRangeDistY;   /* Tolerance for lane width after step in lateral
                                 position (0.1, ~, m)*/
    REAL32_T
    fTdNewCorridorOff; /* The time it takes to consider a new corridor
                          valid, (3, 0~60, s) */
    REAL32_T
    fTdAgainNormalOff; /* The time it takes to consider a new corridor
                          valid, (2, 0~60, s) */
    REAL32_T fMinValidLength_X[6]; /* X axis for min valid lane length, (-,
                                      -20~150, m/s) */
    /*  = 0 5  10 20  40 60  */
    REAL32_T
    fMinValidLength_M[6]; /* Min valid lane length map, (-, -15~15, m) */
                          /* = 15 18 20 20 20 20 */
    REAL32_T fHystMinValidLength_X[6]; /* X axis for min valid lane length, (-,
                                          -20~150, m/s) */
    /*  =  0 5 10 20 40 60 */
    REAL32_T fHystMinValidLength_M[6]; /* Min valid lane length map, (-, -15~15,
                                          m) */
    /*  = 0.3 5.0 5.0 5.0 5.0 5.0 */
    REAL32_T fTdlengthValid; /* Turn off delay time for length validity, (0,
                                0~60, s) */
    UINT16_T
    uBtfValidMarkerType;         /* Bitmask for valid lane marker type, (1078,
                                    0~65535, -) */
    UINT16_T uBtfValidLaneColor; /* Bitmask for valid lane marker colors, (63,
                                    0~65535, -) */
    REAL32_T fCoeffHystMaxRange; /* Maximum range check hysteresis factor, (0,
                                    0~1, -) */
    REAL32_T
    fMaxRangePosY0; /* Maximum range check value for lateral position, (6,
                       0~10, m) */
    REAL32_T fMaxRangeHeading; /* Maximum range check value for heading angle,
                                  (0.5, 0~0.7854, rad) */
    REAL32_T fMaxRangeCrv; /* Maximum range check value for curvature, (0.01,
                              0~10, 1/m) */
    REAL32_T
    fMaxRangeCrvRate;         /* Maximum range check value for curvature rate,
                                 (0.001, 0~10, 1/m^2) */
    REAL32_T fMaxRangeLength; /* Maximum range check value for valid length,
                                 (300, 0~10, m) */

    /* DSC parameter */
    REAL32_T fMinVelUpDownDtct;    /* Minimum velocity for UpDownhill detection,
                                      (1, 0~100, m/s) */
    REAL32_T fUpDownHillPT1TConst; /* Up/Downhill detection PT1 time constant,
                                      (10, 0.001~100, s) */
    UINT8_T
    bUseABDSineWave; /*Determines usage of ABD interface sine wave road
                        detection flag, (1, 0~1, -) */
    REAL32_T fTurnOffTiSineWave; /* ABD sine wave road indicator hold off time
                                    (3, 0~60, s) */
    UINT8_T
    bUseABDSlopeChange; /*Determines usage of ABD interface significant
                           slope detection flag, (1, 0~1, -) */
    REAL32_T fTurnOffTiSlopeChange; /* Hold off delay time for LD slope
                                       detection information, (0.5, 0~60, s) */
    UINT8_T bUseUpDownHill; /*Determines usage Up/downHill flag, (1, 0~1, -) */

    REAL32_T fUpDownHillRSP; /* Upper hysteresis threshold for up/downhill
                                detetion evaluation, (75, 0~100, -) */
    REAL32_T fUpDownHillLSP; /* Lower hysteresis threshold for up/downhill
                                detetion evaluation, (10, 0~100, -) */
    REAL32_T
    fUpDownHillShutOffRSP; /* Upper threshold for up/downhill detection
                              (hysteresis RSP), (100, 0~100, -) */
    REAL32_T
    fUpDownHillShutOffLSP;  /* Lower threshold for up/downhill detection
                               (hysteresis LSP), (75, 0~100, -) */
    REAL32_T fSysCycleTime; /* Cycle time for LCF_SEN components ,(unit, s) */

} sCLPParam_t;
#endif

#ifndef Rte_TypeDef_sCLPOutput_t
#define Rte_TypeDef_sCLPOutput_t
    typedef struct
    {
        UINT8_T bNotAvailableLf; /* Defines whether a lane boundary track is
                                    available or not, (unit, -, CLP) */
        UINT8_T bDistYStepDtctLf;
        UINT8_T bLengthInvalidLf;
        UINT8_T bNotAvailableRi; /* Defines whether a lane boundary track is
                                    available or not, (unit, -, CLP) */
        UINT8_T bValidnewCorridorLf;
        UINT8_T bValidnewCorridorRi;
        UINT8_T bDistYStepDtctRi;
        UINT8_T bLengthInvalidRi;
        UINT8_T bLnQualityInvalidLf;
        UINT8_T bLnQualityInvalidRi;

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

    UINT8_T bCamStatusQualifierValid; /* Camera status qualifier valid , (0,
                                         0~1, -) */

    UINT8_T bUpDownHillDegrade; /* Enable flag for downhill/uphill degrade, (0,
                                   0~1, -) */

    /* CLV */
    UINT8_T bInValidLengthLf; /* The invalidity of left lane valid length, (0,
                                 0~1, -) */
    UINT8_T bInValidLengthRi; /* The invalidity of right lane valid length, (0,
                                 0~1, -) */
    UINT8_T
    bInValidMakerTypeLf;         /* The invalidity of left lane marker type, (0,
                                    0~1, -) */
    UINT8_T bInValidMakerTypeRi; /* The invalidity of right lane marker type,
                                    (0, 0~1, -) */
    UINT8_T
    bInValidLaneColorLf;         /* The invalidity of left lane marker type, (0,
                                    0~1, -) */
    UINT8_T bInValidLaneColorRi; /* The invalidity of right lane marker type,
                                    (0, 0~1, -) */

    // UINT8_T  bCamStatusQualifierValid; /* Camera status qualifier valid , (0,
    // 0~1, -, CLP) */

    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change, (0,
                                0~1, -, ELG) */

    REAL32_T fPercStraightDtct; /* Confidence for straight detection - value
                                   from 0 to 100, (0, 0~100, %)*/
    REAL32_T fPercExitLf;       /* Exit percent for left side, (0, 0~100, %) */

    REAL32_T fPercExitRi; /* Exit percent for right side, (0, 0~100, %) */

    REAL32_T fPercUpDownHillDtct; /* Indicates downhill / uphill scenario
                                     detection confidence, (0, 0~100, %) */

    REAL32_T fRawLaneWidthUnCpl; /* Raw lane width of ABD interface uncoupled
                                    lane data, (0, 0~10, m) */

    REAL32_T fRawLaneWidthCpl; /* Raw lane width of ABD interface coupled lane
                                  data, (0, 0~10, m) */

    UINT16_T uRangeCheckQualifier; /* Input range check qualifier bitfield ,
                                      (0~65535, s, ELG) */

    UINT8_T bConstructionSiteDtct; /* 1: Construction site detected; 0: No
                                      construction site (0~1, -, ELG) */

    UINT8_T bExitUnCplLf;
    UINT8_T bExitUnCplRi;
    UINT8_T bSineWaveUnCpl;
    UINT8_T bNewCorridorValid;
    UINT8_T bLineMergeDtcRi;
    UINT8_T bLineMergeDtcLf;
    } sCLPOutput_t;
#endif

#ifndef Rte_TypeDef_sCLPDebug_t
#define Rte_TypeDef_sCLPDebug_t
typedef struct {
    UINT8_T bLaneChangeDtct; /* Flag that indicates a detected lane change,
                                (0~1, -, ) */

    UINT8_T
    bLastAvailableUlpLf;     /* Last left lane available flag, (unit, -, ) */
    UINT8_T bDisaStepDtctLf; /* Disable flag of step detection, (0, 0~1, -) */
    UINT8_T bEnaByPosY0UlpLf;
    REAL32_T fAbsDevPosY0UlpLf;
    UINT8_T bEnaDistYStepDtctLf; /* Enable flag of lateral distance step
                                    detection, (0, 0~1, -)*/
    UINT8_T bLatDistStepTrigLf; /* Lateral distance trigger flag, (0, 0~1, -) */
    REAL32_T fLaneWidthUlpLf;   /* Lane width , (0, 0~10, m) */
    UINT8_T bValidLaneWidthUlpLf; /* lane width valid flag, (0, 0~10, m) */
    REAL32_T fHistoryLaneWidthLf; /* History lane width, (0,0~10, m)  */
    REAL32_T fAbsDevLaneWidthUlpLf;
    UINT8_T bInVldLaneWidthDevLf; /* Enable flag by lane width for reset
                                     calculation (0, 0~1, -) */
    UINT8_T bRawValidNewCorrLf;   /* Raw valid flag for the corridor is new, (0,
                                     0~1, -) */
    REAL32_T fTimerRawValidNewCorrLf; /* Timer for new corridor turn off delay
                                         (0, 0~60, s) */
    REAL32_T fAbsDevLaneWidthFltLf;
    UINT8_T bEnaByLaneWidthFltLf; /* Enable flag by lane width(from LFP) for
                                     reset calculation (0, 0~1, -) */
    UINT8_T
    bLaneWidthAgainNormalLf; /* Lane width again normal flag, (0, 0~1, -) */
    REAL32_T fTimerAgainNormalLf; /* Timer for Lane width again normal turn off
                                     delay (0, 0~60, s) */
    UINT8_T
    bRawDistYStepDtctLf;      /* Raw flag(by S-R trigger) for distance Y step
                                 detection, (0, 0~1, -) */
    UINT8_T bStepDebouncedLf; /* Step debounced result for distance Y step
                                 detection, (0, 0~1, -) */
    UINT8_T
    bRawStepDebouncedLf; /* The flag of raw step debounced, (0, 0~1, -) */
    REAL32_T fTimerStepDebouncedLf;   /* Timer for step debounced turn on delay
                                         (0, 0~60, s) */
    UINT8_T bResetRawDistYStepDtctLf; /* Reset flag for raw lateral distance
                                         step detection, (0, 0~1, -) */

    UINT8_T
    bLastAvailableUlpRi;     /* Last left lane available flag, (unit, -, ) */
    UINT8_T bDisaStepDtctRi; /* Disable flag of step detection, (0, 0~1, -) */
    UINT8_T bEnaByPosY0UlpRi;
    REAL32_T fAbsDevPosY0UlpRi;
    UINT8_T bEnaDistYStepDtctRi; /* Enable flag of lateral distance step
                                    detection, (0, 0~1, -)*/
    UINT8_T bLatDistStepTrigRi; /* Lateral distance trigger flag, (0, 0~1, -) */
    REAL32_T fLaneWidthUlpRi;   /* Lane width , (0, 0~10, m) */
    UINT8_T bValidLaneWidthUlpRi; /* lane width valid flag, (0, 0~10, m) */
    REAL32_T fHistoryLaneWidthRi; /* History lane width, (0,0~10, m)  */
    REAL32_T fAbsDevLaneWidthUlpRi;
    UINT8_T bInVldLaneWidthDevRi; /* Enable flag by lane width for reset
                                     calculation (0, 0~1, -) */
    UINT8_T bRawValidNewCorrRi;   /* Raw valid flag for the corridor is new, (0,
                                     0~1, -) */
    REAL32_T fTimerRawValidNewCorrRi; /* Timer for new corridor turn off delay
                                         (0, 0~60, s) */
    REAL32_T fAbsDevLaneWidthFltRi;
    UINT8_T bEnaByLaneWidthFltRi; /* Enable flag by lane width(from LFP) for
                                     reset calculation (0, 0~1, -) */
    UINT8_T
    bLaneWidthAgainNormalRi; /* Lane width again normal flag, (0, 0~1, -) */
    REAL32_T fTimerAgainNormalRi; /* Timer for Lane width again normal turn off
                                     delay (0, 0~60, s) */
    UINT8_T
    bRawDistYStepDtctRi;      /* Raw flag(by S-R trigger) for distance Y step
                                 detection, (0, 0~1, -) */
    UINT8_T bStepDebouncedRi; /* Step debounced result for distance Y step
                                 detection, (0, 0~1, -) */
    UINT8_T
    bRawStepDebouncedRi; /* The flag of raw step debounced, (0, 0~1, -) */
    REAL32_T fTimerStepDebouncedRi;   /* Timer for step debounced turn on delay
                                         (0, 0~60, s) */
    UINT8_T bResetRawDistYStepDtctRi; /* Reset flag for raw lateral distance
                                         step detection, (0, 0~1, -) */

    REAL32_T fMinValidlength; /* Maximum valid lane length, (0, -15~15, m) */
    REAL32_T
    fHystMinValidlength; /* Maximum valid lane length, (0, -15~15, m) */
    UINT8_T
    bRawValidLengthLf;      /* Raw validity of left lane length, (0, 0~1, -) */
    UINT8_T bValidLengthLf; /* Raw validity of left lane length, (0, 0~1, -) */
    REAL32_T
    fTimerLengthValidLf; /* Timer for left length validity turn off delay*/
    UINT8_T
    bRawValidLengthRi;      /* Raw validity of right lane length, (0, 0~1, -) */
    UINT8_T bValidLengthRi; /* Raw validity of right lane length, (0, 0~1, -) */
    REAL32_T
    fTimerLengthValidRi; /* Timer for right length validity turn off delay*/
    UINT16_T uStMarerTypeLf;
    UINT16_T uStMarerTypeRi;
    UINT16_T uStLaneColorLf;
    UINT16_T uStLaneColorRi;

    UINT8_T bOutRangePosY0Lf; /* Flag that left position Y0 is out of range (0,
                                 0~1, -) */
    UINT8_T bOutRangeHeadingLf; /* Flag that left the heading angle is out of
                                   range (0, 0~1, -) */
    UINT8_T
    bOutRangeCrvLf; /* Flag that left curvature is out of range (0, 0~1, -)
                     */
    UINT8_T bOutRangeCrvRateLf; /* Flag that left curvature rate is out of range
                                   (0, 0~1, -) */
    UINT8_T bOutRangeLengthLf;  /* Flag that left valid length is out of range
                                   (0, 0~1, -) */
    UINT8_T
    bOutRangePosY0Ri; /* Flag that right position Y0 is out of range (0,
                         0~1, -) */
    UINT8_T bOutRangeHeadingRi; /* Flag that right the heading angle is out of
                                   range (0, 0~1, -) */
    UINT8_T bOutRangeCrvRi;     /* Flag that right curvature is out of range (0,
                                   0~1, -) */
    UINT8_T bOutRangeCrvRateRi; /* Flag that right curvature rate is out of
                                   range (0, 0~1, -) */
    UINT8_T bOutRangeLengthRi;  /* Flag that right valid length is out of range
                                   (0, 0~1, -) */
    REAL32_T
    fMinRangePosY0; /* Minimum range check value for lateral position, (6,
                       0~10, m) */
    REAL32_T fMinRangeHeading; /* Minimum range check value for heading angle,
                                  (0.5, 0~0.7854, rad) */
    REAL32_T fMinRangeCrv; /* Minimum range check value for curvature, (0.01,
                              0~10, 1/m) */
    REAL32_T
    fMinRangeCrvRate;         /* Minimum range check value for curvature rate,
                                 (0.001, 0~10, 1/m^2) */
    REAL32_T fMinRangeLength; /* Minimum range check value for valid length,
                                 (300, 0~10, m) */

    UINT8_T bEnaByLineWidthMergeRi;
    UINT8_T bEnaByCurvatureMergeRi;
    UINT8_T bEnaByAtMaxCurvatureMergeRi;
    UINT8_T bTempHdAtMaxUnCplMergeRi;
    UINT8_T bEnaByHdAtMaxUnCplMergeRi;
    UINT8_T bRawEnaByHdUnCplMergeRi;
    UINT8_T bEnaByHdUnCplMergeRi;
    UINT8_T bRawMergeUnCplRi;

    UINT8_T bEnaByLineWidthMergeLf;
    UINT8_T bEnaByCurvatureMergeLf;
    UINT8_T bEnaByAtMaxCurvatureMergeLf;
    UINT8_T bTempHdAtMaxUnCplMergeLf;
    UINT8_T bEnaByHdAtMaxUnCplMergeLf;
    UINT8_T bRawEnaByHdUnCplMergeLf;
    UINT8_T bEnaByHdUnCplMergeLf;
    UINT8_T bRawMergeUnCplLf;

    REAL32_T fHeadingAtMaxUnCplLf;
    REAL32_T fHeadingAtMaxUnCplRi;
    REAL32_T fPosYAtMaxUnCplLf;
    REAL32_T fPosYAtMaxUnCplRi;
    REAL32_T fCurvatureAtMaxUnCplLf;
    REAL32_T fCurvatureAtMaxUnCplRi;
    } sCLPDebug_t;
#endif

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_TYPEDEF_H_
