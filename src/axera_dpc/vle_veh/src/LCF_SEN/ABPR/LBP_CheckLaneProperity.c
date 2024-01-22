/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */

#include "LBP_CheckLaneProperity.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"

const volatile REAL32_T LBP_fDistYLimitStepDtct = 0.8f;

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static UINT8_T bLastTempHeadingAtMaxUnCplLf = 0U;
static REAL32_T fTimerHeadingAtMaxUnCplLf = 0.0F;
static UINT8_T bLastEnaByHeadingAtMaxUnCplLf = 0U;
static REAL32_T fTimerHeadingUnCplLf = 0.0F;
static UINT8_T bLastEnaByHeadingUnCplLf = 0U;
static UINT8_T bLastRawExitUnCplLf = 0U;
static REAL32_T fTimerExitUnCplLf = 0.0F;
static UINT8_T bLastExitUnCplLf = 0U;
// ExitRampDetection
static UINT8_T bLastEnaByHeadingUnCplRi = 0U;
static UINT8_T bLastExitUnCplRi = 0U;
static UINT8_T bLastRawExitUnCplRi = 0U;
// ExitRampDetection
static REAL32_T fTimerHeadingUnCplRi = 0.0F;
static REAL32_T fTimerExitUnCplRi = 0.0F;
static UINT8_T bLastTempHeadingAtMaxUnCplRi = 0U;
static REAL32_T fTimerHeadingAtMaxUnCplRi = 0.0F;
static UINT8_T bLastEnaByHeadingAtMaxUnCplRi = 0U;

// DetectSlopeChange
static REAL32_T fLastPercUpDownHillDtct =
    0.0F; /* Last downhill/uphill detection confidence, (0, 0~100, %) */
static UINT8_T bLastUpDownHillDegrade =
    0U; /* Last enable flag for downhill/uphill degrade, (0, 0~1, -) */
static UINT8_T bLastEnaBySineWave =
    0U; /* Enable flag by sine wave road for
           downhill/uphill detection, (0, 0~1, -) */
static UINT8_T bLastEnaBySlopeChange =
    0U; /* Enable flag by slope change for downhill/uphill detection, (0,
           0~1, -) */
static REAL32_T fTimerSineWave =
    0.0F; /* Timer for sine wave enable flag turn off delay, (0, 0~60, s) */
static REAL32_T fTimerSlopeChange = 0.0F; /* Timer for slope change enable flag
                                             turn off delay, (0, 0~60, s) */
// LaneStraightDetection
static REAL32_T fLastLengthIntLf; /* Last left lane length by integration,
                                     (0, 0~300, m) */
static UINT8_T
    bLastOutMaxCrvLf; /* Last left lane curvature out of maximum, (0 0~1,
                         -) */
static REAL32_T fLastLengthIntRi; /* Last right lane length by integration,
                                     (0, 0~300, m) */
static UINT8_T bLastOutMaxCrvRi;  /* Last right lane curvature out of
                                     maximum, (0 0~1, -) */
// CLP_LeftValidCorriAfterLatDisJump
static UINT8_T bLastInVldLaneWidthDevLf =
    0U; /* Last Enable flag by lane width for
           reset calculation (0, 0~1, -) */
static REAL32_T fHistoryLaneWidthLf =
    0.0F; /* History lane width, (0,0~10, m)  */
static REAL32_T fTimerRawValidNewCorrLf =
    0.0F; /* Timer for new corridor turn off delay (0, 0~60, s) */
static REAL32_T fHistoryLaneWidthFltLf =
    0.0F; /* History lane width from LFP, (0,0~10, m)  */
static REAL32_T fTimerAgainNormalLf = 0.0F; /* Timer for Lane width again
                                           normal turn off delay (0,
                                           0~60, s) */

// LeftLatDistStepDtct
static UINT8_T bLastAvailableUlpLf =
    0U; /* Last left lane available flag, (unit, -, ) */
static REAL32_T fLastPosY0UlpLf =
    0.0F; /* The last distance given is the value of the polynomial
             evaluated at X = 0.0 m, (unit, m) */
// static UINT8_T bLastLaneWidthAgainNormalLf =
//     0U; /* The corridor is new or not, (0, 0~1, -) */
static UINT8_T bLastRawDistYStepDtctLf =
    0U; /* Last raw flag(by S-R trigger) for distance Y step detection, (0,
           0~1, -) */
static UINT8_T bLastRawStepDebouncedLf =
    0U; /* last flag of raw step debounced, (0, 0~1, -) */
static UINT8_T bLastStepDebouncedLf =
    0U; /* last flag of step debounced, (0, 0~1, -) */
static UINT8_T bLastRawValidNewCorrLf =
    0U; /* last Raw valid flag for the corridor is new, (0, 0~1, -) */
static REAL32_T fTimerStepDebouncedLf =
    0.0F; /* Timer for step debounced turn on delay (0, 0~60, s) */

// CLP_RightValidCorriAfterLatDisJump
static UINT8_T bLastInVldLaneWidthDevRi =
    0U; /* Last Enable flag by lane width for
           reset calculation (0, 0~1, -) */
static REAL32_T fHistoryLaneWidthRi =
    0.0F; /* History lane width, (0,0~10, m)  */
static REAL32_T fTimerRawValidNewCorrRi =
    0.0F; /* Timer for new corridor turn off delay (0, 0~60, s) */
static REAL32_T fHistoryLaneWidthFltRi =
    0.0F; /* History lane width from LFP, (0,0~10, m)  */
static REAL32_T fTimerAgainNormalRi = 0.0F; /* Timer for Lane width again
                                           normal turn off delay (0,
                                           0~60, s) */
// RightLatDistStepDtct
static UINT8_T bLastAvailableUlpRi =
    0U; /* Last left lane available flag, (unit, -, ) */
static REAL32_T fLastPosY0UlpRi =
    0.0F; /* The last distance given is the value of the polynomial
             evaluated at X = 0.0 m, (unit, m) */
// static UINT8_T bLastLaneWidthAgainNormalRi =
//     0U; /* The corridor is new or not, (0, 0~1, -) */
static UINT8_T bLastRawDistYStepDtctRi =
    0U; /* Last raw flag(by S-R trigger) for distance Y step detection, (0,
           0~1, -) */
static UINT8_T bLastRawStepDebouncedRi =
    0U; /* last flag of raw step debounced, (0, 0~1, -) */
static UINT8_T bLastStepDebouncedRi =
    0U; /* last flag of step debounced, (0, 0~1, -) */
static UINT8_T bLastRawValidNewCorrRi =
    0U; /* last Raw valid flag for the corridor is new, (0, 0~1, -) */
static REAL32_T fTimerStepDebouncedRi =
    0.0F; /* Timer for step debounced turn on delay (0, 0~60, s) */

// OutRangeCheck
static UINT8_T bLastOutRangePosY0Lf =
    0U; /* Flag that left position Y0 is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeHeadingLf =
    0U; /* Flag that left the heading angle is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeCrvLf =
    0U; /* Flag that left curvature is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeCrvRateLf =
    0U; /* Flag that left curvature rate is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeLengthLf =
    0U; /* Flag that left valid length is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangePosY0Ri =
    0U; /* Flag that right position Y0 is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeHeadingRi =
    0U; /* Flag that right the heading angle is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeCrvRi =
    0U; /* Flag that right curvature is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeCrvRateRi =
    0U; /* Flag that right curvature rate is out of range (0, 0~1, -) */
static UINT8_T bLastOutRangeLengthRi =
    0U; /* Flag that right valid length is out of range (0, 0~1, -) */
// CheckLaneValidity
// static UINT8_T bLastAvailableLf = 0U; /* Defines whether a lane boundary
//                                          track is available or not,
//                                          (unit,
//                                          -, ) */
// static UINT8_T bLastAvailableRi = 0U; /* Defines whether a lane boundary
//                                          track is available or not,
//                                          (unit,
//                                          -, ) */
static UINT8_T bLastRawValidLengthLf =
    0U; /* Last raw validity of left lane length, (0, 0~1, -) */
static UINT8_T bLastValidLengthLf =
    0U; /* Raw validity of left lane length, (0, 0~1, -) */
static UINT8_T bLastRawValidLengthRi =
    0U; /* Last raw validity of right lane length, (0, 0~1, -) */
static UINT8_T bLastValidLengthRi =
    0U; /* Raw validity of right lane length, (0, 0~1, -) */
static UINT8_T bLastLnQualityInvalidLf = 0U;
static UINT8_T bLastLnQualityInvalidRi = 0U;
static REAL32_T fTimerLengthValidLf =
    0.0F; /* Timer for left length validity turn off delay*/
static REAL32_T fTimerLengthValidRi =
    0.0F; /* Timer for right length validity turn off delay*/
static UINT8_T bLastTempHdAtMaxUnCplRiMerge = 0U;
static UINT8_T bLastEnaByHdAtMaxUnCplRiMerge = 0U;
static REAL32_T fTimerHdAtMaxUnCplRiMerge = 0.F;
static UINT8_T bLastEnaByHdUnCplRiMerge = 0U;
static REAL32_T fTimerHdUnCplRiMerge = 0.F;
static UINT8_T bLastRawMergeUnCplRi = 0U;
static UINT8_T bLastMergeUnCplRi = 0U;
static REAL32_T fTimerMergeUnCplRi = 0.F;
static UINT8_T bLastTempHdAtMaxUnCplLfMerge = 0U;
static UINT8_T bLastEnaByHdAtMaxUnCplLfMerge = 0U;
static REAL32_T fTimerHdAtMaxUnCplLfMerge = 0.F;
static UINT8_T bLastEnaByHdUnCplLfMerge = 0U;
static REAL32_T fTimerHdUnCplLfMerge = 0.F;
static UINT8_T bLastRawMergeUnCplLf = 0U;
static UINT8_T bLastMergeUnCplLf = 0U;
static REAL32_T fTimerMergeUnCplLf = 0.F;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void CheckLaneProperityReset(void) {
    bLastTempHeadingAtMaxUnCplLf = 0U;
    fTimerHeadingAtMaxUnCplLf = 0.0F;
    bLastEnaByHeadingAtMaxUnCplLf = 0U;
    fTimerHeadingUnCplLf = 0.0F;
    bLastEnaByHeadingUnCplLf = 0U;
    bLastRawExitUnCplLf = 0U;
    fTimerExitUnCplLf = 0.0F;
    bLastExitUnCplLf = 0U;
    // ExitRampDetection
    bLastEnaByHeadingUnCplRi = 0U;
    bLastExitUnCplRi = 0U;
    bLastRawExitUnCplRi = 0U;
    // ExitRampDetection
    fTimerHeadingUnCplRi = 0.0F;
    fTimerExitUnCplRi = 0.0F;
    bLastTempHeadingAtMaxUnCplRi = 0U;
    fTimerHeadingAtMaxUnCplRi = 0.0F;
    bLastEnaByHeadingAtMaxUnCplRi = 0U;

    // DetectSlopeChange
    fLastPercUpDownHillDtct =
        0.0F;  // Last downhill/uphill detection confidence, (0, 0~100, %) */
    bLastUpDownHillDegrade =
        0U;  // Last enable flag for downhill/uphill degrade, (0, 0~1, -) */
    bLastEnaBySineWave = 0U;     // Enable flag by sine wave road for
                                 //    downhill/uphill detection, (0, 0~1, -) */
    bLastEnaBySlopeChange = 0U;  // Enable flag by slope change for
                                 //    downhill/uphill detection, (0, 0~1, -) */
    fTimerSineWave = 0.0F;  // Timer for sine wave enable flag turn off delay,
                            // (0, 0~60, s) */
    fTimerSlopeChange = 0.0F;  // Timer for slope change enable flag turn off
                               //  delay, (0, 0~60, s) */
    // LaneStraightDetection
    fLastLengthIntLf = 0.0F;  // Last left lane length by integration,
                              //    (0, 0~300, m) */

    bLastOutMaxCrvLf = 0U;  // Last left lane curvature out of maximum, (0 0~1,
                            // -) */
    fLastLengthIntRi = 0.0F;        // Last right lane length by integration,
                                    // (0, 0~300, m) */
    bLastOutMaxCrvRi = 0U;          // Last right lane curvature out of
                                    //  maximum, (0 0~1, -) */
                                    // CLP_LeftValidCorriAfterLatDisJump
    bLastInVldLaneWidthDevLf = 0U;  // Last Enable flag by lane width for
                                    //   reset calculation (0, 0~1, -) */
    fHistoryLaneWidthLf = 0.0F;     // History lane width, (0,0~10, m)  */
    fTimerRawValidNewCorrLf =
        0.0F;  // Timer for new corridor turn off delay (0, 0~60, s) */
    fHistoryLaneWidthFltLf =
        0.0F;                   // History lane width from LFP, (0,0~10, m)  */
    fTimerAgainNormalLf = 0.0F; /* Timer for Lane width again
                                             normal turn off delay (0,
                                             0~60, s) */

    // LeftLatDistStepDtct
    bLastAvailableUlpLf = 0U;  // Last left lane available flag, (unit, -, ) */
    fLastPosY0UlpLf = 0.0F;    /* The last distance given is the value of the
                    polynomial   evaluated at X = 0.0 m, (unit, m) */
    //   bLastLaneWidthAgainNormalLf =
    //     0U; /* The corridor is new or not, (0, 0~1, -) */
    bLastRawDistYStepDtctLf = 0U; /* Last raw flag(by S-R trigger) for distance
                                     Y step detection, (0, 0~1, -) */
    bLastRawStepDebouncedLf =
        0U; /* last flag of raw step debounced, (0, 0~1, -) */
    bLastStepDebouncedLf = 0U; /* last flag of step debounced, (0, 0~1, -) */
    bLastRawValidNewCorrLf =
        0U; /* last Raw valid flag for the corridor is new, (0, 0~1, -) */
    fTimerStepDebouncedLf =
        0.0F; /* Timer for step debounced turn on delay (0, 0~60, s) */

    // CLP_RightValidCorriAfterLatDisJump
    bLastInVldLaneWidthDevRi = 0U; /* Last Enable flag by lane width for
                                      reset calculation (0, 0~1, -) */
    fHistoryLaneWidthRi = 0.0F;    /* History lane width, (0,0~10, m)  */
    fTimerRawValidNewCorrRi =
        0.0F; /* Timer for new corridor turn off delay (0, 0~60, s) */
    fHistoryLaneWidthFltRi =
        0.0F;                   /* History lane width from LFP, (0,0~10, m)  */
    fTimerAgainNormalRi = 0.0F; /* Timer for Lane width again
                                             normal turn off delay (0,
                                             0~60, s) */
                                // RightLatDistStepDtct
    bLastAvailableUlpRi = 0U;   /* Last left lane available flag, (unit, -, ) */
    fLastPosY0UlpRi =
        0.0F; /* The last distance given is the value of the
                 polynomial     evaluated at X = 0.0 m, (unit, m) */
    //   bLastLaneWidthAgainNormalRi =
    //     0U; /* The corridor is new or not, (0, 0~1, -) */
    bLastRawDistYStepDtctRi = 0U; /* Last raw flag(by S-R trigger) for distance
                                     Y step detection, (0, 0~1, -) */
    bLastRawStepDebouncedRi =
        0U; /* last flag of raw step debounced, (0, 0~1, -) */
    bLastStepDebouncedRi = 0U; /* last flag of step debounced, (0, 0~1, -) */
    bLastRawValidNewCorrRi =
        0U; /* last Raw valid flag for the corridor is new, (0, 0~1, -) */
    fTimerStepDebouncedRi =
        0.0F; /* Timer for step debounced turn on delay (0, 0~60, s) */

    // OutRangeCheck
    bLastOutRangePosY0Lf =
        0U; /* Flag that left position Y0 is out of range (0, 0~1, -) */
    bLastOutRangeHeadingLf =
        0U; /* Flag that left the heading angle is out of range (0, 0~1, -) */
    bLastOutRangeCrvLf =
        0U; /* Flag that left curvature is out of range (0, 0~1, -) */
    bLastOutRangeCrvRateLf =
        0U; /* Flag that left curvature rate is out of range (0, 0~1, -) */
    bLastOutRangeLengthLf =
        0U; /* Flag that left valid length is out of range (0, 0~1, -) */
    bLastOutRangePosY0Ri =
        0U; /* Flag that right position Y0 is out of range (0, 0~1, -) */
    bLastOutRangeHeadingRi =
        0U; /* Flag that right the heading angle is out of range (0, 0~1, -) */
    bLastOutRangeCrvRi =
        0U; /* Flag that right curvature is out of range (0, 0~1, -) */
    bLastOutRangeCrvRateRi =
        0U; /* Flag that right curvature rate is out of range (0, 0~1, -) */
    bLastOutRangeLengthRi =
        0U; /* Flag that right valid length is out of range (0, 0~1, -) */
            // CheckLaneValidity
            //   bLastAvailableLf = 0U; /* Defines whether a lane boundary
    //                                          track is available or not,
    //                                          (unit,
    //                                          -, ) */
    //   bLastAvailableRi = 0U; /* Defines whether a lane boundary
    //                                          track is available or not,
    //                                          (unit,
    //                                          -, ) */
    bLastRawValidLengthLf =
        0U; /* Last raw validity of left lane length, (0, 0~1, -) */
    bLastValidLengthLf = 0U; /* Raw validity of left lane length, (0, 0~1, -) */
    bLastRawValidLengthRi =
        0U; /* Last raw validity of right lane length, (0, 0~1, -) */
    bLastValidLengthRi =
        0U; /* Raw validity of right lane length, (0, 0~1, -) */
    bLastLnQualityInvalidLf = 0U;
    bLastLnQualityInvalidRi = 0U;
    fTimerLengthValidLf =
        0.0F; /* Timer for left length validity turn off delay*/
    fTimerLengthValidRi =
        0.0F; /* Timer for right length validity turn off delay*/
    bLastTempHdAtMaxUnCplRiMerge = 0U;
    bLastEnaByHdAtMaxUnCplRiMerge = 0U;
    bLastRawMergeUnCplRi = 0U;
    bLastMergeUnCplRi = 0U;
    fTimerMergeUnCplRi = 0.F;
    fTimerHdAtMaxUnCplRiMerge = 0.F;
    bLastEnaByHdUnCplRiMerge = 0U;
    fTimerHdUnCplRiMerge = 0.F;
    bLastTempHdAtMaxUnCplLfMerge = 0U;
    bLastEnaByHdAtMaxUnCplLfMerge = 0U;
    bLastRawMergeUnCplLf = 0U;
    bLastMergeUnCplLf = 0U;
    fTimerMergeUnCplLf = 0.F;
    fTimerHdAtMaxUnCplLfMerge = 0.F;
    bLastEnaByHdUnCplLfMerge = 0U;
    fTimerHdUnCplLfMerge = 0.F;
}
/**************************2.Check lane
 * properity******************************************/
/****************************************************************************************
        @fn           ExitRampDetection
        @brief        Exit ramp detection
        @description  Exit ramp detection:
                                          1.Position Y and heading angle at
maximum length;
                                          2.Left lane exit flag;
                                          3.Right lane exit flag;
                                          4.Left and right lane sine wave flag.
        @param[in]    pCLPInput : Input for ExitRampDetection
        @param[in]    pCLPParam : Parameter for ExitRampDetection
        @param[out]   pCLPOutput: Output for ExitRampDetection
        @param[out]   pCLPDebug : Debug(measurement) for ExitRampDetection
        @return       void
        @startuml
        title ExitRampDetection
        (*)--> 1.PositionYAndHeadingAngleAtMaximumLength
           --> 2.LeftLaneExitFlag
           --> (*)
         1.PositionYAndHeadingAngleAtMaximumLength --> 3.RightLaneExitFlag
           --> (*)
         1.PositionYAndHeadingAngleAtMaximumLength -->
4.LeftAndRightLaneSineWaveFlag
           --> (*)
        @enduml
******************************************************************************************/
void CLP_LeftLaneExitFlag(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          UINT8_T bEnaByLineWidthUnCpl,
                          sCLPOutput_t *pCLPOutput) {
    REAL32_T fTempERD = 0.0F;
    UINT8_T bEnaByCrvUnCplLf = 0U;
    UINT8_T bTempHeadingAtMaxUnCplLf = 0U;
    UINT8_T bTempERD = 0U;
    UINT8_T bEnaByHeadingAtMaxUnCplLf = 0U;
    UINT8_T bRawEnaByHeadingUnCplLf = 0U;
    UINT8_T bEnaByHeadingUnCplLf = 0U;
    UINT8_T bRawExitUnCplLf = 0U;
    UINT8_T bExitUnCplLf = 0U;

    /***************************2.Left lane exit
     * flag**************************************/
    /* Curve comparison */
    fTempERD = pCLPInput->fCrvUnCpLf - pCLPInput->fCrvUnCpRi;
    if ((TUE_CML_Abs_M(pCLPInput->fCrvUnCpLf) >
         TUE_CML_Abs_M(pCLPInput->fCrvUnCpRi)) &&
        (fTempERD > 0.002F)) {
        bEnaByCrvUnCplLf = 1U;
    } else {
        bEnaByCrvUnCplLf = 0U;
    }

    /* Heading at max length comparison*/
    fTempERD = fHeadingAtMaxUnCplLf - fHeadingAtMaxUnCplRi;
    if ((fTempERD > 0.04F) && (fHeadingAtMaxUnCplLf > 0.04F)) {
        bTempHeadingAtMaxUnCplLf = 1U;
    } else {
        bTempHeadingAtMaxUnCplLf = 0U;
    }

    if ((bLastTempHeadingAtMaxUnCplLf == 0U) &&
        (bTempHeadingAtMaxUnCplLf == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }

    bEnaByHeadingAtMaxUnCplLf = TUE_CML_TurnOffDelay_M(
        bTempERD, 1.0F, pCLPParam->fSysCycleTime, &fTimerHeadingAtMaxUnCplLf,
        bLastEnaByHeadingAtMaxUnCplLf);

    /* Heading comparison */
    fTempERD = fHeadingAtMaxUnCplLf - pCLPInput->fHeadingUnCplLf;
    if (fTempERD > 0.04F) {
        bRawEnaByHeadingUnCplLf = 1U;
    } else {
        bRawEnaByHeadingUnCplLf = 0U;
    }
    bEnaByHeadingUnCplLf = TUE_CML_TurnOffDelay_M(
        bRawEnaByHeadingUnCplLf, 1.0F, pCLPParam->fSysCycleTime,
        &fTimerHeadingUnCplLf, bLastEnaByHeadingUnCplLf);
    if ((pCLPInput->bAvailableUnCplLf == 1U) &&
        (pCLPInput->bAvailableUnCplRi == 1U) && (bEnaByCrvUnCplLf == 1U) &&
        (bEnaByHeadingAtMaxUnCplLf == 1U) && (bEnaByHeadingUnCplLf == 1U)) {
        bRawExitUnCplLf = 1U;
    } else {
        bRawExitUnCplLf = 0U;
    }

    if ((bLastRawExitUnCplLf == 0U) && (bRawExitUnCplLf == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    bExitUnCplLf =
        TUE_CML_TurnOffDelay_M(bTempERD, 2.3F, pCLPParam->fSysCycleTime,
                               &fTimerExitUnCplLf, bLastExitUnCplLf);

    if ((bEnaByLineWidthUnCpl == 1U) && (bExitUnCplLf == 1U)) {
        pCLPOutput->bExitUnCplLf = 1U;
    } else {
        pCLPOutput->bExitUnCplLf = 0U;
    }
    bLastEnaByHeadingUnCplLf = bEnaByHeadingUnCplLf;
    bLastTempHeadingAtMaxUnCplLf = bTempHeadingAtMaxUnCplLf;
    bLastEnaByHeadingAtMaxUnCplLf = bEnaByHeadingAtMaxUnCplLf;
    bLastRawExitUnCplLf = bRawExitUnCplLf;
    bLastExitUnCplLf = bExitUnCplLf;
}
void LineMergeDetectionRi(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          REAL32_T fPosYAtMaxUnCplLf,
                          REAL32_T fPosYAtMaxUnCplRi,
                          REAL32_T fCurvatureAtMaxUnCplLf,
                          REAL32_T fCurvatureAtMaxUnCplRi,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug) {
    REAL32_T fTempERD = 0.0F;
    UINT8_T bTempERD = 0U;
    UINT8_T bEnaByLineWidthMerge = 0U;

    // Width comparision
    fTempERD = (pCLPInput->fPosY0UnCplLf - pCLPInput->fPosY0UnCplRi) -
               (fPosYAtMaxUnCplLf - fPosYAtMaxUnCplRi);
    if (fTempERD > 1.5F) {
        bEnaByLineWidthMerge = 1U;
    } else {
        bEnaByLineWidthMerge = 0U;
    }

    // Curve comparision
    UINT8_T bEnaByCurvatureMerge = 0U;
    UINT8_T bEnaByAtMaxCurvatureMerge = 0U;

    fTempERD = pCLPInput->fCrvUnCpRi - pCLPInput->fCrvUnCpLf;
    if (fTempERD > 0.002F && pCLPInput->fCrvUnCpRi > 0.002F &&
        (TUE_CML_Abs_M(pCLPInput->fCrvUnCpRi) >
         TUE_CML_Abs_M(pCLPInput->fCrvUnCpLf))) {
        bEnaByCurvatureMerge = 1U;
    } else {
        bEnaByCurvatureMerge = 0U;
    }

    fTempERD = fCurvatureAtMaxUnCplRi - fCurvatureAtMaxUnCplLf;
    if (fTempERD > 0.002F && fCurvatureAtMaxUnCplRi > 0.002F &&
        (TUE_CML_Abs_M(fCurvatureAtMaxUnCplRi) >
         TUE_CML_Abs_M(fCurvatureAtMaxUnCplLf))) {
        bEnaByAtMaxCurvatureMerge = 1U;
    } else {
        bEnaByAtMaxCurvatureMerge = 0U;
    }

    // Heading comparision
    /* Heading at max length comparison*/
    UINT8_T bTempHdAtMaxUnCplRiMerge = 0U;
    fTempERD = fHeadingAtMaxUnCplRi - fHeadingAtMaxUnCplLf;
    if ((fTempERD > 0.04F) && (fHeadingAtMaxUnCplRi > 0.04F)) {
        bTempHdAtMaxUnCplRiMerge = 1U;
    } else {
        bTempHdAtMaxUnCplRiMerge = 0U;
    }

    if ((bLastTempHdAtMaxUnCplRiMerge == 0U) &&
        (bTempHdAtMaxUnCplRiMerge == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    UINT8_T bEnaByHdAtMaxUnCplRiMerge = 0U;
    bEnaByHdAtMaxUnCplRiMerge = TUE_CML_TurnOffDelay_M(
        bTempERD, 1.0F, pCLPParam->fSysCycleTime, &fTimerHdAtMaxUnCplRiMerge,
        bLastEnaByHdAtMaxUnCplRiMerge);
    /* Heading comparison */
    UINT8_T bRawEnaByHdUnCplRiMerge = 0U;
    fTempERD = fHeadingAtMaxUnCplRi - pCLPInput->fHeadingUnCplRi;
    if (fTempERD > 0.04F) {
        bRawEnaByHdUnCplRiMerge = 1U;
    } else {
        bRawEnaByHdUnCplRiMerge = 0U;
    }
    UINT8_T bEnaByHdUnCplRiMerge = 0U;

    bEnaByHdUnCplRiMerge = TUE_CML_TurnOffDelay_M(
        bRawEnaByHdUnCplRiMerge, 1.0F, pCLPParam->fSysCycleTime,
        &fTimerHdUnCplRiMerge, bLastEnaByHdUnCplRiMerge);

    UINT8_T bRawMergeUnCplRi = 0U;
    if ((pCLPInput->bAvailableUnCplLf == 1U) &&
        (pCLPInput->bAvailableUnCplRi == 1U) &&
        ((bEnaByCurvatureMerge == 1U) || (bEnaByAtMaxCurvatureMerge == 1)) &&
        (bEnaByHdAtMaxUnCplRiMerge == 1U) && (bEnaByHdUnCplRiMerge == 1U) &&
        (bEnaByLineWidthMerge == 1)) {
        bRawMergeUnCplRi = 1U;
    } else {
        bRawMergeUnCplRi = 0U;
    }

    if ((bLastRawMergeUnCplRi == 0U) && (bRawMergeUnCplRi == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    UINT8_T bMergeUnCplRi = 0U;
    bMergeUnCplRi =
        TUE_CML_TurnOffDelay_M(bTempERD, 2.3F, pCLPParam->fSysCycleTime,
                               &fTimerMergeUnCplRi, bLastMergeUnCplRi);
    // Merge dectetion
    // pCLPOutput->bLineMergeDtcRi = bMergeUnCplRi;
    pCLPOutput->bLineMergeDtcRi = 0u;
    bLastTempHdAtMaxUnCplRiMerge = bTempHdAtMaxUnCplRiMerge;
    bLastEnaByHdAtMaxUnCplRiMerge = bEnaByHdAtMaxUnCplRiMerge;
    bLastEnaByHdUnCplRiMerge = bEnaByHdUnCplRiMerge;
    bLastRawMergeUnCplRi = bRawMergeUnCplRi;
    bLastMergeUnCplRi = bMergeUnCplRi;

    pCLPDebug->bEnaByLineWidthMergeRi = bEnaByLineWidthMerge;
    pCLPDebug->bEnaByCurvatureMergeRi = bEnaByCurvatureMerge;
    pCLPDebug->bEnaByAtMaxCurvatureMergeRi = bEnaByAtMaxCurvatureMerge;
    pCLPDebug->bTempHdAtMaxUnCplMergeRi = bTempHdAtMaxUnCplRiMerge;
    pCLPDebug->bEnaByHdAtMaxUnCplMergeRi = bEnaByHdAtMaxUnCplRiMerge;
    pCLPDebug->bRawEnaByHdUnCplMergeRi = bRawEnaByHdUnCplRiMerge;
    pCLPDebug->bEnaByHdUnCplMergeRi = bEnaByHdUnCplRiMerge;
    pCLPDebug->bRawMergeUnCplRi = bRawMergeUnCplRi;

    pCLPDebug->fHeadingAtMaxUnCplLf = fHeadingAtMaxUnCplLf;
    pCLPDebug->fHeadingAtMaxUnCplRi = fHeadingAtMaxUnCplRi;
    pCLPDebug->fPosYAtMaxUnCplLf = fPosYAtMaxUnCplLf;
    pCLPDebug->fPosYAtMaxUnCplRi = fPosYAtMaxUnCplRi;
    pCLPDebug->fCurvatureAtMaxUnCplLf = fCurvatureAtMaxUnCplLf;
    pCLPDebug->fCurvatureAtMaxUnCplRi = fCurvatureAtMaxUnCplRi;
}

void LineMergeDetectionLf(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          REAL32_T fPosYAtMaxUnCplLf,
                          REAL32_T fPosYAtMaxUnCplRi,
                          REAL32_T fCurvatureAtMaxUnCplLf,
                          REAL32_T fCurvatureAtMaxUnCplRi,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug) {
    REAL32_T fTempERD = 0.0F;
    UINT8_T bTempERD = 0U;
    UINT8_T bEnaByLineWidthMerge = 0U;

    // Width comparision
    fTempERD = (pCLPInput->fPosY0UnCplLf - pCLPInput->fPosY0UnCplRi) -
               (fPosYAtMaxUnCplLf - fPosYAtMaxUnCplRi);
    if (fTempERD > 1.5F) {
        bEnaByLineWidthMerge = 1U;
    } else {
        bEnaByLineWidthMerge = 0U;
    }

    // Curve comparision
    UINT8_T bEnaByCurvatureMerge = 0U;
    UINT8_T bEnaByAtMaxCurvatureMerge = 0U;

    fTempERD = pCLPInput->fCrvUnCpRi - pCLPInput->fCrvUnCpLf;
    if (fTempERD > 0.002F && pCLPInput->fCrvUnCpLf < -0.002F &&
        (TUE_CML_Abs_M(pCLPInput->fCrvUnCpLf) >
         TUE_CML_Abs_M(pCLPInput->fCrvUnCpRi))) {
        bEnaByCurvatureMerge = 1U;
    } else {
        bEnaByCurvatureMerge = 0U;
    }

    fTempERD = fCurvatureAtMaxUnCplRi - fCurvatureAtMaxUnCplLf;
    if (fTempERD > 0.002F && fCurvatureAtMaxUnCplLf < -0.002F &&
        (TUE_CML_Abs_M(fCurvatureAtMaxUnCplLf) >
         TUE_CML_Abs_M(fCurvatureAtMaxUnCplRi))) {
        bEnaByAtMaxCurvatureMerge = 1U;
    } else {
        bEnaByAtMaxCurvatureMerge = 0U;
    }

    // Heading comparision
    /* Heading at max length comparison*/
    UINT8_T bTempHdAtMaxUnCplLfMerge = 0U;
    fTempERD = fHeadingAtMaxUnCplRi - fHeadingAtMaxUnCplLf;
    if ((fTempERD > 0.04F) && (fHeadingAtMaxUnCplLf < -0.04F)) {
        bTempHdAtMaxUnCplLfMerge = 1U;
    } else {
        bTempHdAtMaxUnCplLfMerge = 0U;
    }

    if ((bLastTempHdAtMaxUnCplLfMerge == 0U) &&
        (bTempHdAtMaxUnCplLfMerge == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    UINT8_T bEnaByHdAtMaxUnCplLfMerge = 0U;
    bEnaByHdAtMaxUnCplLfMerge = TUE_CML_TurnOffDelay_M(
        bTempERD, 1.0F, pCLPParam->fSysCycleTime, &fTimerHdAtMaxUnCplLfMerge,
        bLastEnaByHdAtMaxUnCplLfMerge);
    /* Heading comparison */
    UINT8_T bRawEnaByHdUnCplLfMerge = 0U;
    fTempERD = fHeadingAtMaxUnCplLf - pCLPInput->fHeadingUnCplLf;
    if (fTempERD < -0.04F) {
        bRawEnaByHdUnCplLfMerge = 1U;
    } else {
        bRawEnaByHdUnCplLfMerge = 0U;
    }
    UINT8_T bEnaByHdUnCplLfMerge = 0U;

    bEnaByHdUnCplLfMerge = TUE_CML_TurnOffDelay_M(
        bRawEnaByHdUnCplLfMerge, 1.0F, pCLPParam->fSysCycleTime,
        &fTimerHdUnCplLfMerge, bLastEnaByHdUnCplLfMerge);

    UINT8_T bRawMergeUnCplLf = 0U;
    if ((pCLPInput->bAvailableUnCplLf == 1U) &&
        (pCLPInput->bAvailableUnCplRi == 1U) &&
        ((bEnaByCurvatureMerge == 1U) || (bEnaByAtMaxCurvatureMerge == 1)) &&
        (bEnaByHdAtMaxUnCplLfMerge == 1U) && (bEnaByHdUnCplLfMerge == 1U) &&
        (bEnaByLineWidthMerge == 1)) {
        bRawMergeUnCplLf = 1U;
    } else {
        bRawMergeUnCplLf = 0U;
    }

    if ((bLastRawMergeUnCplLf == 0U) && (bRawMergeUnCplLf == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    UINT8_T bMergeUnCplLf = 0U;
    bMergeUnCplLf =
        TUE_CML_TurnOffDelay_M(bTempERD, 2.3F, pCLPParam->fSysCycleTime,
                               &fTimerMergeUnCplLf, bLastMergeUnCplLf);
    // Merge dectetion
    // pCLPOutput->bLineMergeDtcLf = bMergeUnCplLf;
    pCLPOutput->bLineMergeDtcLf = 0u;
    bLastTempHdAtMaxUnCplLfMerge = bTempHdAtMaxUnCplLfMerge;
    bLastEnaByHdAtMaxUnCplLfMerge = bEnaByHdAtMaxUnCplLfMerge;
    bLastEnaByHdUnCplLfMerge = bEnaByHdUnCplLfMerge;
    bLastRawMergeUnCplLf = bRawMergeUnCplLf;
    bLastMergeUnCplLf = bMergeUnCplLf;

    pCLPDebug->bEnaByLineWidthMergeLf = bEnaByLineWidthMerge;
    pCLPDebug->bEnaByCurvatureMergeLf = bEnaByCurvatureMerge;
    pCLPDebug->bEnaByAtMaxCurvatureMergeLf = bEnaByAtMaxCurvatureMerge;
    pCLPDebug->bTempHdAtMaxUnCplMergeLf = bTempHdAtMaxUnCplLfMerge;
    pCLPDebug->bEnaByHdAtMaxUnCplMergeLf = bEnaByHdAtMaxUnCplLfMerge;
    pCLPDebug->bRawEnaByHdUnCplMergeLf = bRawEnaByHdUnCplLfMerge;
    pCLPDebug->bEnaByHdUnCplMergeLf = bEnaByHdUnCplLfMerge;
    pCLPDebug->bRawMergeUnCplLf = bRawMergeUnCplLf;
}

void LineMergeDetection(const sCLPInput_t *pCLPInput,
                        const sCLPParam_t *pCLPParam,
                        sCLPOutput_t *pCLPOutput,
                        sCLPDebug_t *pCLPDebug) {
    REAL32_T fMaxLengthUnCpl = 0.0F;

    REAL32_T fPosYAtMaxUnCplLf = 0.0F;
    REAL32_T fHeadingAtMaxUnCplLf = 0.0F;
    REAL32_T fCurvatureAtMaxUnCplLf = 0.0F;

    REAL32_T fPosYAtMaxUnCplRi = 0.0F;
    REAL32_T fHeadingAtMaxUnCplRi = 0.0F;
    REAL32_T fCurvatureAtMaxUnCplRi = 0.0F;

    /***************************1.Position Y and heading angle at maximum
     * length***********/
    fMaxLengthUnCpl = TUE_CML_Max_M(pCLPInput->fValidLengthUnCpLf,
                                    pCLPInput->fValidLengthUnCpRi);
    fPosYAtMaxUnCplLf = TUE_CML_PosY3rd_M(
        fMaxLengthUnCpl, pCLPInput->fPosY0UnCplLf, pCLPInput->fHeadingUlpLf,
        pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fHeadingAtMaxUnCplLf =
        TUE_CML_Yaw3rd_M(fMaxLengthUnCpl, pCLPInput->fHeadingUlpLf,
                         pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fCurvatureAtMaxUnCplLf = TUE_CML_Crv3rd_M(
        fMaxLengthUnCpl, pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fPosYAtMaxUnCplRi = TUE_CML_PosY3rd_M(
        fMaxLengthUnCpl, pCLPInput->fPosY0UnCplRi, pCLPInput->fHeadingUlpRi,
        pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    fHeadingAtMaxUnCplRi =
        TUE_CML_Yaw3rd_M(fMaxLengthUnCpl, pCLPInput->fHeadingUlpRi,
                         pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    fCurvatureAtMaxUnCplRi = TUE_CML_Crv3rd_M(
        fMaxLengthUnCpl, pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    /***************************2. Right Line merge detection
     * flag***********************/
    LineMergeDetectionRi(pCLPInput, pCLPParam, fHeadingAtMaxUnCplLf,
                         fHeadingAtMaxUnCplRi, fPosYAtMaxUnCplLf,
                         fPosYAtMaxUnCplRi, fCurvatureAtMaxUnCplLf,
                         fCurvatureAtMaxUnCplRi, pCLPOutput, pCLPDebug);

    /***************************3. Left Line merge detection
     * flag***********************/
    LineMergeDetectionLf(pCLPInput, pCLPParam, fHeadingAtMaxUnCplLf,
                         fHeadingAtMaxUnCplRi, fPosYAtMaxUnCplLf,
                         fPosYAtMaxUnCplRi, fCurvatureAtMaxUnCplLf,
                         fCurvatureAtMaxUnCplRi, pCLPOutput, pCLPDebug);
}

void ExitRampDetection(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug) {
    REAL32_T fMaxLengthUnCpl = 0.0F;
    UINT8_T bEnaByLineWidthUnCpl = 0U;

    REAL32_T fPosYAtMaxUnCplLf = 0.0F;
    REAL32_T fHeadingAtMaxUnCplLf = 0.0F;
    REAL32_T fCurvatureAtMaxUnCplLf = 0.0F;

    REAL32_T fPosYAtMaxUnCplRi = 0.0F;
    REAL32_T fHeadingAtMaxUnCplRi = 0.0F;
    REAL32_T fCurvatureAtMaxUnCplRi = 0.0F;

    UINT8_T bEnaByCrvUnCplRi = 0U;
    UINT8_T bEnaByHeadingAtMaxUnCplRi = 0U;
    UINT8_T bEnaByHeadingUnCplRi = 0U;
    UINT8_T bRawEnaByHeadingUnCplRi = 0U;
    UINT8_T bRawExitUnCplRi = 0U;
    UINT8_T bExitUnCplRi = 0U;

    // UINT8_T bSineWaveUnCpl = 0U;
    // REAL32_T fPosYAtX1UnCplLf = 0.0F;
    // REAL32_T fPosYAtX1UnCplRi = 0.0F;
    // UINT8_T bEnaByCrvUnCpl = 0U;
    // UINT8_T bEnaByLaneWidth = 0U;
    // UINT8_T bRawEnaByX1UnCpl = 0U;
    // UINT8_T bRawEnaByX2UnCpl = 0U;
    // UINT8_T bDlyEnaByX1UnCpl = 0U;

    // REAL32_T fPosYAtX2UnCplLf = 0.0F;
    // REAL32_T fPosYAtX2UnCplRi = 0.0F;

    REAL32_T fTempERD = 0.0F;
    UINT8_T bTempERD = 0U;

    UINT8_T bTempHeadingAtMaxUnCplRi = 0U;

    /***************************1.Position Y and heading angle at maximum
     * length***********/
    fMaxLengthUnCpl = TUE_CML_Max_M(pCLPInput->fValidLengthUnCpLf,
                                    pCLPInput->fValidLengthUnCpRi);
    fPosYAtMaxUnCplLf = TUE_CML_PosY3rd_M(
        fMaxLengthUnCpl, pCLPInput->fPosY0UnCplLf, pCLPInput->fHeadingUlpLf,
        pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fHeadingAtMaxUnCplLf =
        TUE_CML_Yaw3rd_M(fMaxLengthUnCpl, pCLPInput->fHeadingUlpLf,
                         pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fCurvatureAtMaxUnCplLf = TUE_CML_Crv3rd_M(
        fMaxLengthUnCpl, pCLPInput->fCrvUnCpLf, pCLPInput->fCrvRateUnCpLf);

    fPosYAtMaxUnCplRi = TUE_CML_PosY3rd_M(
        fMaxLengthUnCpl, pCLPInput->fPosY0UnCplRi, pCLPInput->fHeadingUlpRi,
        pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    fHeadingAtMaxUnCplRi =
        TUE_CML_Yaw3rd_M(fMaxLengthUnCpl, pCLPInput->fHeadingUlpRi,
                         pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    fCurvatureAtMaxUnCplRi = TUE_CML_Crv3rd_M(
        fMaxLengthUnCpl, pCLPInput->fCrvUnCpRi, pCLPInput->fCrvRateUnCpRi);

    /* Width Comparison */
    fTempERD = (fPosYAtMaxUnCplLf - fPosYAtMaxUnCplRi) -
               (pCLPInput->fPosY0UnCplLf - pCLPInput->fPosY0UnCplRi);
    if (fTempERD > 1.5F) {
        bEnaByLineWidthUnCpl = 1U;
    } else {
        bEnaByLineWidthUnCpl = 0U;
    }

    CLP_LeftLaneExitFlag(pCLPInput, pCLPParam, fHeadingAtMaxUnCplLf,
                         fHeadingAtMaxUnCplRi, bEnaByLineWidthUnCpl,
                         pCLPOutput);

    /***************************3.Right lane exit
     * flag*************************************/
    /* Curve comparison */
    fTempERD = TUE_CML_Abs_M(pCLPInput->fCrvUnCpLf - pCLPInput->fCrvUnCpRi);
    if ((TUE_CML_Abs_M(pCLPInput->fCrvUnCpLf) <
         TUE_CML_Abs_M(pCLPInput->fCrvUnCpRi)) &&
        (fTempERD > 0.002F)) {
        bEnaByCrvUnCplRi = 1U;
    } else {
        bEnaByCrvUnCplRi = 0U;
    }

    /* Heading at max length comparison*/
    fTempERD = fHeadingAtMaxUnCplLf - fHeadingAtMaxUnCplRi;
    if ((fTempERD > 0.04F) && (fHeadingAtMaxUnCplRi < -0.04F)) {
        bTempHeadingAtMaxUnCplRi = 1U;
    } else {
        bTempHeadingAtMaxUnCplRi = 0U;
    }

    if ((bLastTempHeadingAtMaxUnCplRi == 0U) &&
        (bTempHeadingAtMaxUnCplRi == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }

    bEnaByHeadingAtMaxUnCplRi = TUE_CML_TurnOffDelay_M(
        bTempERD, 1.0F, pCLPParam->fSysCycleTime, &fTimerHeadingAtMaxUnCplRi,
        bLastEnaByHeadingAtMaxUnCplRi);

    /* Heading comparison */
    fTempERD = fHeadingAtMaxUnCplRi - pCLPInput->fHeadingUnCplRi;
    if (fTempERD < -0.04F) {
        bRawEnaByHeadingUnCplRi = 1U;
    } else {
        bRawEnaByHeadingUnCplRi = 0U;
    }
    bEnaByHeadingUnCplRi = TUE_CML_TurnOffDelay_M(
        bRawEnaByHeadingUnCplRi, 1.0F, pCLPParam->fSysCycleTime,
        &fTimerHeadingUnCplRi, bLastEnaByHeadingUnCplRi);
    if ((pCLPInput->bAvailableUnCplLf == 1U) &&
        (pCLPInput->bAvailableUnCplRi == 1U) && (bEnaByCrvUnCplRi == 1U) &&
        (bEnaByHeadingAtMaxUnCplRi == 1U) && (bEnaByHeadingUnCplRi == 1U)) {
        bRawExitUnCplRi = 1U;
    } else {
        bRawExitUnCplRi = 0U;
    }

    if ((bLastRawExitUnCplRi == 0U) && (bRawExitUnCplRi == 1U)) {
        bTempERD = 1U;
    } else {
        bTempERD = 0U;
    }
    bExitUnCplRi =
        TUE_CML_TurnOffDelay_M(bTempERD, 2.3F, pCLPParam->fSysCycleTime,
                               &fTimerExitUnCplRi, bLastExitUnCplRi);

    if ((bEnaByLineWidthUnCpl == 1U) && (bExitUnCplRi == 1U)) {
        pCLPOutput->bExitUnCplRi = 1U;
    } else {
        pCLPOutput->bExitUnCplRi = 0U;
    }

    /***************************4.Left and right lane sine wave
     * flag***********************/
    // fPosYAtX1UnCplLf = TUE_CML_PosY3rd_M(50.0F,
    //                               pCLPInput->fPosY0UnCplLf,
    //                               pCLPInput->fHeadingUlpLf,
    //                               pCLPInput->fCrvUnCpLf,
    //                               pCLPInput->fCrvRateUnCpLf);
    // fPosYAtX1UnCplRi = TUE_CML_PosY3rd_M(50.F,
    //                               pCLPInput->fPosY0UnCplRi,
    //                               pCLPInput->fHeadingUlpRi,
    //                               pCLPInput->fCrvUnCpRi,
    //                               pCLPInput->fCrvRateUnCpRi);

    // fPosYAtX2UnCplLf = TUE_CML_PosY3rd_M(60.0F,
    //                               pCLPInput->fPosY0UnCplLf,
    //                               pCLPInput->fHeadingUlpLf,
    //                               pCLPInput->fCrvUnCpLf,
    //                               pCLPInput->fCrvRateUnCpLf);
    // fPosYAtX2UnCplRi = TUE_CML_PosY3rd_M(60.F,
    //                               pCLPInput->fPosY0UnCplRi,
    //                               pCLPInput->fHeadingUlpRi,
    //                               pCLPInput->fCrvUnCpRi,
    //                               pCLPInput->fCrvRateUnCpRi);

    // fTempERD = (fPosYAtX1UnCplLf - fPosYAtX1UnCplRi) -
    //           (pCLPInput->fPosY0UnCplLf - pCLPInput->fPosY0UnCplRi)/2.0F;
    // if(fTempERD > 0.0F)
    //{
    //    bEnaByLaneWidth = 1U;
    //}
    // else
    //{
    //    bEnaByLaneWidth = 0U;
    //}

    // if(((pCLPInput->fCrvUnCpLf >= 0.0F) && (pCLPInput->fCrvUnCpRi >= 0.0F))
    // ||
    //   ((pCLPInput->fCrvUnCpLf < 0.0F) && (pCLPInput->fCrvUnCpRi < 0.0F)))
    //{
    //    bEnaByCrvUnCpl = 0U;
    //}
    // else
    //{
    //    bEnaByCrvUnCpl = 1U;
    //}

    // if((bEnaByLaneWidth == 1U) &&
    //    (bEnaByCrvUnCpl == 1U) &&
    //    (pCLPInput->bAvailableUnCplLf == 1U) &&
    //    (pCLPInput->bAvailableUnCplRi == 1U) &&
    //    (pCLPOutput->bExitUnCplLf == 0U) &&
    //    (pCLPOutput->bExitUnCplRi == 0U))
    //{
    //    bRawEnaByX1UnCpl = 1U;
    //}
    // else
    //{
    //    bRawEnaByX1UnCpl = 0U;
    //}

    ///* wait and delay */

    ///* */
    // if((fPosYAtX2UnCplLf < fPosYAtX2UnCplRi) &&
    //   (pCLPInput->bAvailableUnCplLf == 1U) &&
    //   (pCLPInput->bAvailableUnCplRi == 1U))
    //{
    //    bRawEnaByX2UnCpl = 1U;
    //}
    // else
    //{
    //    bRawEnaByX2UnCpl = 0U;
    //}

    /***************************Save last
     * value********************************************/

    bLastEnaByHeadingUnCplRi = bEnaByHeadingUnCplRi;
    bLastExitUnCplRi = bExitUnCplRi;
    bLastRawExitUnCplRi = bRawExitUnCplRi;

    bLastTempHeadingAtMaxUnCplRi = bTempHeadingAtMaxUnCplRi;
    bLastEnaByHeadingAtMaxUnCplRi = bEnaByHeadingAtMaxUnCplRi;

    /***************************Output and
     * debug*******************************************/
}

/******************************DetectSlopeChange********************************************
        @fn           DetectSlopeChange
        @brief        Calculate downhill/uphill scenario detection confidence
        @description  Detect slope change:
                                        1.Raw downhill/uphill confidence;
                                        2.Low pass filter;
        @param[in]    pCLPInput : Input for CLP
        @param[in]    pCLPParam : Parameter for CLP
        @param[out]   pCLPOutput: Output for CLP
        @param[out]   pCLPDebug : Debug(measurement) for CLP
        @return       void
        @startuml
        title ExitRampDetection
        (*)--> 1.Raw downhill/uphill confidence
           --> 2.Low pass filter
           --> (*)
        @enduml
 ******************************************************************************************/
void DetectSlopeChange(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug) {
    UINT8_T bEnaBySineWave = 0U; /* Enable flag by sine wave road for
                                    downhill/uphill detection, (0, 0~1, -) */
    UINT8_T
    bEnaBySlopeChange = 0U; /* Enable flag by slope change for
                               downhill/uphill detection, (0, 0, 0~1, -) */
    REAL32_T fRawUpDownHillDtct =
        0.0F; /* Downhill/uphill confidence before low pass filter, (0, 0~100,
                 %) */

    REAL32_T fLowPassCoeff = 0.0F; /* Low pass filter coefficient,(0, 0~1, -) */

    UINT8_T bTempCLP = 0U;
    REAL32_T fTempCLP = 0.0F;

    /**********************1.Raw downhill / uphill
     * confidence*****************************/
    /* Sine wave road condition */
    if ((pCLPParam->bUseABDSineWave == 1U) &&
        (pCLPInput->fSineWaveDtct > 0.001F)) {
        bTempCLP = 1U;
    } else {
        bTempCLP = 0U;
    }
    bEnaBySineWave = TUE_CML_TurnOffDelay_M(
        bTempCLP, pCLPParam->fTurnOffTiSineWave, pCLPParam->fSysCycleTime,
        &fTimerSineWave, bLastEnaBySineWave);

    /* Slope change condition */
    if ((pCLPParam->bUseABDSlopeChange == 1U) &&
        (pCLPInput->fVertSlopeChange > 0.001F)) {
        bTempCLP = 1U;
    } else {
        bTempCLP = 0U;
    }
    bEnaBySlopeChange = TUE_CML_TurnOffDelay_M(
        bTempCLP, pCLPParam->fTurnOffTiSlopeChange, pCLPParam->fSysCycleTime,
        &fTimerSlopeChange, bLastEnaBySlopeChange);

    /* Calculate raw downhill/uphill scenario detection confidence */
    if ((bEnaBySineWave == 1U) && (bEnaBySlopeChange == 1U)) {
        fTempCLP = 100.0F;
    } else {
        fTempCLP = 0.0F;
    }

    if (pCLPParam->bUseUpDownHill == 1U) {
        fRawUpDownHillDtct =
            TUE_CML_Max_M(fTempCLP, (pCLPInput->fUpDownHillDtct));
    } else {
        fRawUpDownHillDtct = fTempCLP;
    }

    /**********************2.Low pass
     * filter**********************************************/
    if (pCLPInput->fVehVelX > pCLPParam->fMinVelUpDownDtct) {
        fLowPassCoeff = TUE_CML_DivProtection_M(
            pCLPParam->fSysCycleTime, pCLPParam->fUpDownHillPT1TConst, 1e-6f);
        // (pCLPParam->fSysCycleTime) /
        // (pCLPParam->fUpDownHillPT1TConst);
        fLowPassCoeff = TUE_CML_Limit_M(fLowPassCoeff, 0.0F, 1.0F);
        pCLPOutput->fPercUpDownHillDtct =
            fRawUpDownHillDtct * fLowPassCoeff +
            fLastPercUpDownHillDtct * (1 - fLowPassCoeff);
    } else {
        pCLPOutput->fPercUpDownHillDtct = fLastPercUpDownHillDtct;
    }

    /* Uphill/downHill degrade*/
    pCLPOutput->bUpDownHillDegrade = TUE_CML_Hysteresis_M(
        pCLPOutput->fPercUpDownHillDtct, pCLPParam->fUpDownHillRSP,
        pCLPParam->fUpDownHillLSP, bLastUpDownHillDegrade);

    /**********************Save last
     * value**********************************************/
    bLastEnaBySineWave = bEnaBySineWave;
    bLastEnaBySlopeChange = bEnaBySlopeChange;
    bLastUpDownHillDegrade = pCLPOutput->bUpDownHillDegrade;
    fLastPercUpDownHillDtct = pCLPOutput->fPercUpDownHillDtct;

    /**********************Output and
     * debug**********************************************/
}

/****************************************************************************************
        @fn           LaneStraightDetection(StraightDetection)
        @brief        Calculate the lane straight confidence
        @description  Lane straight detection:
                                          1.Left lane straight detection:
                                                  1.1 Calculate left lane
integration reset flag;
                                                  1.2 Calculate left lane
straight confidence;
                                          2.right lane straight detection:
                                                  1.1 Calculate right
laneintegration reset flag;
                                                  1.2 Calculate right lane
straight confidence;
        @param[in]    pCLPInput : Input for LSD
        @param[in]    pCLPParam : Parameter for LSD
        @param[out]   pCLPOutput: Output for LSD
        @param[out]   pCLPDebug : Debug(measurement) for LSD
        @return       void
        @startuml
        title LaneStraightDetection
        (*)--> 1.LeftLaneStraightDetection
           --> 1.1 CalculateLeftLaneIntegrationResetFlag
           --> 1.2 CalculateLeftLaneStraightConfidence
           --> (*)
        (*)--> 2.RightLaneStraightDetection
           --> 2.1 CalculateRightLaneIntegrationResetFlag
           --> 2.2 CalculateRightLaneStraightConfidence
           --> (*)
        @enduml
******************************************************************************************/
void LaneStraightDetection(const sCLPInput_t *pCLPInput,
                           const sCLPParam_t *pCLPParam,
                           sCLPOutput_t *pCLPOutput,
                           sCLPDebug_t *pCLPDebug) {
    UINT8_T bResetByOverallQualityLf =
        0U;  // Reset flag by left lane overall quality, (0, 0~1, -)
    UINT8_T bResetByLengthLf =
        0U;  // Reset flag by left lane valid length, (0, 0~1, -)
    UINT8_T bOutMaxCrvLf =
        0U;  // Left lane curvature out of maximum, (0 0~1, -)
    UINT8_T bResetByLnVldLf =
        0U;  // Reset flag by left lane parameter validity, (0 0~1, -)
    // UINT8_T bResetByMeanDevLf =
    //     0U;  // Reset flag by left lane mean deviation, (0 0~1, -)
    REAL32_T fPosXUlpLf[POLYFIT_SAMPLE_POINTS] = {
        0.0F};  // Left lane X position sample points , (0, -300~300, m)
    REAL32_T fPosYUlpLf[POLYFIT_SAMPLE_POINTS] = {
        0.0F};  // Left lane Y position sample points, (0, -15~15, m)
    UINT8_T bResetByDevToTrajLf =
        0U;  // Reset flag by left lane parameter validity, (0 0~1, -)
    UINT8_T bResetForIntLf =
        0U;  // left lane length integration reset flag, (0 0~1, -)
    REAL32_T fLengthIntLf =
        0.0F;  // Left lane length by integration, (0, 0~300, m)
    REAL32_T
    fPercStraightLf;  // Left lane straight confidence, (0, 0~100, %)

    UINT8_T bResetByOverallQualityRi =
        0U;  // Reset flag by right lane overall quality, (0, 0~1, -)
    UINT8_T bResetByLengthRi =
        0U;  // Reset flag by right lane valid length, (0, 0~1, -)
    UINT8_T bOutMaxCrvRi =
        0U;  // Right lane curvature out of maximum, (0 0~1, -)
    UINT8_T bResetByLnVldRi =
        0U;  // Reset flag by right lane parameter validity, (0 0~1, -)
    // UINT8_T bResetByMeanDevRi =
    //     0U;  // Reset flag by right lane mean deviation, (0 0~1, -)
    REAL32_T fPosXUlpRi[POLYFIT_SAMPLE_POINTS] = {
        0.0F};  // Right lane X position sample points , (0, -300~300, m)
    REAL32_T fPosYUlpRi[POLYFIT_SAMPLE_POINTS] = {
        0.0F};  // Right lane Y position sample points, (0, -15~15, m)
    UINT8_T bResetByDevToTrajRi =
        0U;  // Reset flag by right lane parameter validity, (0 0~1, -)
    UINT8_T bResetForIntRi =
        0U;  // Right lane length integration reset flag, (0 0~1, -)
    REAL32_T fLengthIntRi =
        0.0F;  // Right lane length by integration, (0, 0~300, m)
    REAL32_T
    fPercStraightRi;  // Right lane straight confidence, (0, 0~100, %)
    // REAL32_T fTempLSD = 0.0F;
    // UINT8_T bTempLSD = 0U;

    REAL32_T fDevTraj1stLf = 0.0F;
    REAL32_T fDevTraj1stRi = 0.0F;
    sPFTInput_t sPFTInputLf = {0};    // Third order polynomial fitting input
    sPFTOutput_t sPFTOutputLf = {0};  // Third order polynomial fitting Output
    sPFTInput_t sPFTInputRi = {0};    // Third order polynomial fitting input
    sPFTOutput_t sPFTOutputRi = {0};  // Third order polynomial fitting Output

    /****************************1.Left lane straight
     * detection*****************************/
    /****************************1.1 Calculate left lane integration reset
     * flag*************/
    /* Reset flag by overall quality for left lane */
    if (pCLPInput->fOverallQualityLf > 40.0F) {
        bResetByOverallQualityLf = 1U;
    } else {
        bResetByOverallQualityLf = 0U;
    }

    /* Reset flag by lane length for left lane */
    if (pCLPInput->fValidLengthUlpLf > fLastLengthIntLf) {
        bResetByLengthLf = 1U;
    } else {
        bResetByLengthLf = 0U;
    }

    /* Reset flag by lane parameter validity for left lane */
    /* The reset flag is 1 must meet the following conditions:
                     1.Lane line quality is not low,  not equal to 1 ;
                     2.Lane line is available;
                     3.Curvature does not exceed the limit.*/
    /* Check whether the curvature out of the maximum */
    bOutMaxCrvLf = TUE_CML_Hysteresis_M(TUE_CML_Abs_M(pCLPInput->fCrvUlpLf),
                                        0.00035F, 0.00025F, bLastOutMaxCrvLf);
    if ((bOutMaxCrvLf == 0U) && (pCLPInput->uQualityUlpLf != 1U) &&
        (pCLPInput->bAvailableUlpLf == 1U)) {
        bResetByLnVldLf = 1U;
    } else {
        bResetByLnVldLf = 0U;
    }

    // Reset flag by mean deviation for left lane
    // Sample points from ULP
    for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        fPosXUlpLf[ii] =
            pCLPInput->fValidLengthUlpLf * ii / (POLYFIT_SAMPLE_POINTS);
        fPosYUlpLf[ii] = TUE_CML_PosY3rd_M(
            fPosXUlpLf[ii], pCLPInput->fPosY0UlpLf, pCLPInput->fHeadingUlpLf,
            pCLPInput->fCrvUlpLf, pCLPInput->fCrvRateUlpLf);
        sPFTInputLf.fFctWeight[ii] = 1.0f;
        sPFTInputLf.fPosXArray[ii] = fPosXUlpLf[ii];
        sPFTInputLf.fPosYArray[ii] = fPosYUlpLf[ii];
    }
    /* First-order polynomial fitting */
    sPFTInputLf.bEnable1st = 1U;
    sPFTInputLf.bEnable2nd = 0U;
    sPFTInputLf.bEnable3rd = 0U;
    sPFTInputLf.fFctCrvDecay = 1.0F;
    sPFTInputLf.fFctCrvChngDecay = 1.0F;

    TUE_CML_PolyFit_M(&sPFTInputLf, &sPFTOutputLf);

    fDevTraj1stLf = sPFTOutputLf.fDevToTraj1st;
    bResetByDevToTrajLf = (fDevTraj1stLf / POLYFIT_SAMPLE_POINTS) < 0.1f;

    /* left lane integration reset flag */
    if ((bResetByOverallQualityLf == 1U) && (bResetByLengthLf == 1U) &&
        (bResetByLnVldLf == 1U) && (bResetByDevToTrajLf == 1U)) {
        bResetForIntLf = 1U;
    } else {
        bResetForIntLf = 0U;
    }

    /****************************1.2 Calculate left lane straight
     * confidence****************/
    if (bResetForIntLf == 1U) {
        fLengthIntLf = pCLPInput->fValidLengthUlpLf;
    } else {
        fLengthIntLf =
            fLastLengthIntLf - pCLPInput->fVehVelX * pCLPParam->fSysCycleTime;
    }
    fLengthIntLf = TUE_CML_Limit_M(fLengthIntLf, 0.0F, 100.0F);

    if (bOutMaxCrvLf == 1U) {
        fPercStraightLf = 0.0F;
    } else {
        fPercStraightLf = (fLengthIntLf - 0.8F * pCLPInput->fVehVelX) * 0.3F;
        fPercStraightLf = TUE_CML_Limit_M(fPercStraightLf, 0.0F, 100.0F);
    }

    /****************************2.Right lane straight
     * detection*****************************/
    /****************************2.1 Calculate Right lane integration reset
     * flag*************/
    /* Reset flag by overall quality for Right lane */
    if (pCLPInput->fOverallQualityRi > 40.0F) {
        bResetByOverallQualityRi = 1U;
    } else {
        bResetByOverallQualityRi = 0U;
    }

    /* Reset flag by lane length for Right lane */
    if (pCLPInput->fValidLengthUlpRi > fLastLengthIntRi) {
        bResetByLengthRi = 1U;
    } else {
        bResetByLengthRi = 0U;
    }

    /* Reset flag by lane parameter validity for Right lane */
    /* The reset flag is 1 must meet the following conditions:
                     1.Lane line quality is not low,  not equal to 1 ;
                     2.Lane line is available;
                     3.Curvature does not exceed the limit.*/
    /* Check whether the curvature out of the maximum */
    bOutMaxCrvRi = TUE_CML_Hysteresis_M(TUE_CML_Abs_M(pCLPInput->fCrvUlpRi),
                                        0.00035F, 0.00025F, bLastOutMaxCrvRi);
    if ((bOutMaxCrvRi == 0U) && (pCLPInput->uQualityUlpRi != 1U) &&
        (pCLPInput->bAvailableUlpRi == 1U)) {
        bResetByLnVldRi = 1U;
    } else {
        bResetByLnVldRi = 0U;
    }

    /* Reset flag by mean deviation for Right lane */
    /* Sample points from ULP */
    for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        fPosXUlpRi[ii] =
            pCLPInput->fValidLengthUlpRi * ii / (POLYFIT_SAMPLE_POINTS);
        fPosYUlpRi[ii] = TUE_CML_PosY3rd_M(
            fPosXUlpRi[ii], pCLPInput->fPosY0UlpRi, pCLPInput->fHeadingUlpRi,
            pCLPInput->fCrvUlpRi, pCLPInput->fCrvRateUlpRi);
        sPFTInputRi.fFctWeight[ii] = 1.0f;
        sPFTInputRi.fPosXArray[ii] = fPosXUlpRi[ii];
        sPFTInputRi.fPosYArray[ii] = fPosYUlpRi[ii];
    }
    /* First-order polynomial fitting deviation*/
    sPFTInputRi.bEnable1st = 1U;
    sPFTInputRi.bEnable2nd = 0U;
    sPFTInputRi.bEnable3rd = 0U;
    sPFTInputRi.fFctCrvDecay = 1.0F;
    sPFTInputRi.fFctCrvChngDecay = 1.0F;

    TUE_CML_PolyFit_M(&sPFTInputRi, &sPFTOutputRi);

    fDevTraj1stRi = sPFTOutputRi.fDevToTraj1st;
    bResetByDevToTrajRi = (fDevTraj1stRi / POLYFIT_SAMPLE_POINTS) < 0.1f;

    /* Right lane integration reset flag */
    if ((bResetByOverallQualityRi == 1U) && (bResetByLengthRi == 1U) &&
        (bResetByLnVldRi == 1U) && (bResetByDevToTrajRi == 1U)) {
        bResetForIntRi = 1U;
    } else {
        bResetForIntRi = 0U;
    }

    /****************************2.2 Calculate Right lane straight
     * confidence****************/
    if (bResetForIntRi == 1U) {
        fLengthIntRi = pCLPInput->fValidLengthUlpRi;
    } else {
        fLengthIntRi =
            fLastLengthIntRi - pCLPInput->fVehVelX * pCLPParam->fSysCycleTime;
    }
    fLengthIntRi = TUE_CML_Limit_M(fLengthIntRi, 0.0F, 100.0F);

    if (bOutMaxCrvRi == 1U) {
        fPercStraightRi = 0.0F;
    } else {
        fPercStraightRi = (fLengthIntRi - 0.8F * pCLPInput->fVehVelX) * 0.3F;
        fPercStraightRi = TUE_CML_Limit_M(fPercStraightRi, 0.0F, 100.0F);
    }

    /**************************************Output and
     * debug********************************/
    pCLPOutput->fPercStraightDtct = (fPercStraightLf + fPercStraightRi) / 2;

    /**************************************Save last
     * value********************************/
    bLastOutMaxCrvLf = bOutMaxCrvLf;
    fLastLengthIntLf = fLengthIntLf;
    bLastOutMaxCrvRi = bOutMaxCrvRi;
    fLastLengthIntRi = fLengthIntRi;
}

/****************************************************************************************
        @fn           CalculateOtherEvent
        @brief        Calculate raw lane width,construction site��Lane
changes,camera status
        @description  Calculate other event:
                                          1.Calculate raw lane width;
                                          2.Construction site detection;
                                          3.Lane change evaluation;
                                          4.Check camera status qualifier.
        @param[in]    pCLPInput : Input for COE
        @param[in]    pCLPParam : Parameter for COE
        @param[out]   pCLPOutput: Output for COE
        @param[out]   pCLPDebug : Debug(measurement) for COE
        @return       void
        @startuml
        title CalculateOtherEvent
        (*)--> 1.Calculate raw lane width
           --> (*)
        (*)--> 2.Construction site detection
           --> (*)
        (*)--> 3.Lane change evaluation
           --> (*)
        (*)--> 4.Check camera status qualifier
           --> (*)
        @enduml
******************************************************************************************/
void CalculateOtherEvent(const sCLPInput_t *pCLPInput,
                         const sCLPParam_t *pCLPParam,
                         sCLPOutput_t *pCLPOutput,
                         sCLPDebug_t *pCLPDebug) {
    UINT8_T bValidCamByWeather = 0;   /* Temp variable, (unit, -)*/
    UINT8_T bValidCamByCompState = 0; /* Temp variable, (unit, -)*/

    /****************************1.Calculate raw lane
     * width*********************************/
    /* Calculate raw lane width of coupled lane */
    if ((pCLPInput->bAvailableCplLf == 1U) &&
        (pCLPInput->bAvailableCplRi == 1U)) {
        pCLPOutput->fRawLaneWidthCpl =
            pCLPInput->fPosY0CplLf * TUE_CML_Cos_M(pCLPInput->fHeadingCplLf) -
            pCLPInput->fPosY0CplRi * TUE_CML_Cos_M(pCLPInput->fHeadingCplRi);
        pCLPOutput->fRawLaneWidthCpl =
            TUE_CML_Abs_M(pCLPOutput->fRawLaneWidthCpl);
    } else {
        pCLPOutput->fRawLaneWidthCpl = pCLPParam->fDefaultLaneWidth;
    }
    /* Calculate raw lane width of uncoupled lane */
    if ((pCLPInput->bAvailableUnCplLf == 1U) &&
        (pCLPInput->bAvailableUnCplRi == 1U)) {
        pCLPOutput->fRawLaneWidthUnCpl =
            pCLPInput->fPosY0UnCplLf *
                TUE_CML_Cos_M(pCLPInput->fHeadingUnCplLf) -
            pCLPInput->fPosY0UnCplRi *
                TUE_CML_Cos_M(pCLPInput->fHeadingUnCplRi);
        pCLPOutput->fRawLaneWidthUnCpl =
            TUE_CML_Abs_M(pCLPOutput->fRawLaneWidthUnCpl);
    } else {
        pCLPOutput->fRawLaneWidthUnCpl = pCLPParam->fDefaultLaneWidth;
    }

    /****************************2.Construction site
     * detection******************************/
    if ((pCLPInput->uRoadWorks & pCLPParam->uConstructionSiteBitmask) != 0U) {
        pCLPOutput->bConstructionSiteDtct = 1U;
    } else {
        pCLPOutput->bConstructionSiteDtct = 0U;
    }

    /****************************3.Lane change
     * evaluation***********************************/
    pCLPOutput->bLaneChangeDtct =
        ((pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeLf) ||
         (pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeRi));

    /****************************4.Check camera status
     * qualifier****************************/
    if (pCLPParam->bUseWeatherCond == 1U) {
        bValidCamByWeather =
            (pCLPInput->uWeatherCond) >> (pCLPParam->uWeatherCondBitmask);
    } else {
        bValidCamByWeather = 1U;
    }
    if (pCLPParam->bUseCompState == 1U) {
        bValidCamByCompState =
            ((pCLPInput->uCompState) == (pCLPParam->uCompStateEnum));
    } else {
        bValidCamByCompState = 1U;
    }
    pCLPOutput->bCamStatusQualifierValid =
        (pCLPInput->bUpDownHillCritical == 1) ||
        ((bValidCamByWeather == 1) && (bValidCamByCompState == 1));
}

void CLP_LeftValidCorriAfterLatDisJump(const sCLPInput_t *pCLPInput,
                                       const sCLPParam_t *pCLPParam,
                                       sCLPDebug_t *pCLPDebug,
                                       UINT8_T bLatDistStepTrigLf,
                                       UINT8_T *bRawValidNewCorrLf,
                                       UINT8_T *bLaneWidthAgainNormalLf) {
    REAL32_T fLaneWidthUlpLf = 0.0F;   /* Lane width , (0, 0~10, m) */
    UINT8_T bValidLaneWidthUlpLf = 0U; /* lane width valid flag, (0, 0~10, m) */
    UINT8_T bInVldLaneWidthDevLf =
        0U; /* Enable flag by lane width for reset calculation (0, 0~1, -) */
    REAL32_T fAbsDevLaneWidthUlpLf = 0.0F;
    REAL32_T fAbsDevLaneWidthFltLf = 0.0F;
    UINT8_T
    bEnaByLaneWidthFltLf = 0; /* Enable flag by lane width(from LFP) for
                                 reset calculation (0, 0~1, -) */

    /*************2.1 Lane Width valid
     * flag*************************************************/
    /* Ulp lane width*/
    fLaneWidthUlpLf =
        pCLPInput->fPosY0UlpLf * TUE_CML_Cos_M(pCLPInput->fHeadingUlpLf) -
        pCLPInput->fPosY0UlpRi * TUE_CML_Cos_M(pCLPInput->fHeadingUlpRi);
    fLaneWidthUlpLf = TUE_CML_Abs_M(fLaneWidthUlpLf);
    /* Determine current lane width validity for new corridor validation*/
    if (fLaneWidthUlpLf <=
            (pCLPParam->fMaxLaneWidth + pCLPParam->fMaxLaneWidthHyst) &&
        fLaneWidthUlpLf >=
            (pCLPParam->fMinLaneWidth + pCLPParam->fMinLaneWidthHyst)) {
        bValidLaneWidthUlpLf = 1U;
    } else {
        bValidLaneWidthUlpLf = 0U;
    }

    /*************2.2 Raw new lane corridor valid
     * flag**************************************/
    if ((bLastInVldLaneWidthDevLf == 1U) || (bLatDistStepTrigLf == 1U)) {
        fHistoryLaneWidthLf = fLaneWidthUlpLf;
    } else {
        fHistoryLaneWidthLf =
            fHistoryLaneWidthLf; /* Value store, remain unchanged */
    }

    fAbsDevLaneWidthUlpLf =
        TUE_CML_Abs_M(fLaneWidthUlpLf - fHistoryLaneWidthLf);
    if (fAbsDevLaneWidthUlpLf > pCLPParam->TolRangeNewCorr) {
        bInVldLaneWidthDevLf = 1U;
    } else {
        bInVldLaneWidthDevLf = 0U;
    }
    /* A new corridor is valid if:
               -a lateral step has been triggered (while bridging is allowed)
               -the current measured lane width is valid
               -a new stable corridor is determined (lane width remains in a
       certain range
                    for at certain time span) */
    if ((bLatDistStepTrigLf == 1U) || (bInVldLaneWidthDevLf == 1U) ||
        (bValidLaneWidthUlpLf == 0U)) {
        *bRawValidNewCorrLf = 0U;
        fTimerRawValidNewCorrLf = pCLPParam->fTdNewCorridorOff;
    } else {
        if (fTimerRawValidNewCorrLf > pCLPParam->fSysCycleTime) {
            fTimerRawValidNewCorrLf =
                fTimerRawValidNewCorrLf - pCLPParam->fSysCycleTime;
            *bRawValidNewCorrLf = 0U;
        } else {
            fTimerRawValidNewCorrLf = 0.0F;
            *bRawValidNewCorrLf = 1U;
        }
    }

    /*************2.3 Lane width again normal
     * flag******************************************/
    /* The lane width is valid again if:
    -a lateral step has been triggered(while bridging is allowed)
    -the current lane width is in range width the lane width before
     the step detection for at certain time span */
    if (bLatDistStepTrigLf == 1U) {
        fHistoryLaneWidthFltLf = pCLPInput->fLaneWidth;
    } else {
        fHistoryLaneWidthFltLf =
            fHistoryLaneWidthFltLf; /* Value store, remain unchanged */
    }
    /* The corridor is considered new if the width changes more than this value
     */
    fAbsDevLaneWidthFltLf =
        TUE_CML_Abs_M(fLaneWidthUlpLf - fHistoryLaneWidthFltLf);
    if (fAbsDevLaneWidthFltLf > pCLPParam->TolRangeDistY) {
        bEnaByLaneWidthFltLf = 1U;
    } else {
        bEnaByLaneWidthFltLf = 0U;
    }
    if ((bEnaByLaneWidthFltLf == 1U) || (bLatDistStepTrigLf == 1U)) {
        *bLaneWidthAgainNormalLf = 0U;
        fTimerAgainNormalLf = pCLPParam->fTdAgainNormalOff;
    } else {
        if (fTimerAgainNormalLf > pCLPParam->fSysCycleTime) {
            fTimerAgainNormalLf =
                fTimerAgainNormalLf - pCLPParam->fSysCycleTime;
            *bLaneWidthAgainNormalLf = 0U;
        } else {
            fTimerRawValidNewCorrLf = 0.0F;
            *bLaneWidthAgainNormalLf = 1U;
        }
    }

    pCLPDebug->fLaneWidthUlpLf = fLaneWidthUlpLf;
    pCLPDebug->bValidLaneWidthUlpLf = bValidLaneWidthUlpLf;
    pCLPDebug->fHistoryLaneWidthLf = fHistoryLaneWidthLf;
    pCLPDebug->bInVldLaneWidthDevLf = bInVldLaneWidthDevLf;
    pCLPDebug->fTimerRawValidNewCorrLf = fTimerRawValidNewCorrLf;
    pCLPDebug->fAbsDevLaneWidthFltLf = fAbsDevLaneWidthFltLf;
    pCLPDebug->bEnaByLaneWidthFltLf = bEnaByLaneWidthFltLf;
    pCLPDebug->fTimerAgainNormalLf = fTimerAgainNormalLf;

    bLastInVldLaneWidthDevLf = bInVldLaneWidthDevLf;
}

/****************************************************************************************
        @fn           LeftLatDistStepDtct
        @brief        Detect whether the left lateral distance step is too large
        @description  Left lateral distance step detection:
                                          1.Set lane lateral distance step
detection;
                                          2.Lane lateral distance step detection
debounced:
                                                  2.1 Lane Width valid flag;
                                                  2.2 Raw new lane corridor
valid flag;
                                                  2.3 Lane width again normal
flag;
                                                  2.4 Debounce lateral step
detection.
        @param[in]    pCLPInput : Input for LeftLatDistStepDtct
        @param[in]    pCLPParam : Parameter for LeftLatDistStepDtct
        @param[out]   pCLPOutput: Output for LeftLatDistStepDtct
        @param[out]   pCLPDebug : Debug(measurement) for LeftLatDistStepDtct
        @return       void
        @startuml
        title LeftLatDistStepDtct
        (*)--> 1.SetLaneLateralDistanceStepDetection
           --> 2.1 LaneWidthValidFlag
           --> 2.2 RawNewLaneCorridorValidFlag
           --> 2.3 LaneWidthAgainNormalFlag
           --> 2.4 DebounceLateralStepDetection
           --> (*)
        @enduml
******************************************************************************************/
void LeftLatDistStepDtct(const sCLPInput_t *pCLPInput,
                         const sCLPParam_t *pCLPParam,
                         sCLPOutput_t *pCLPOutput,
                         sCLPDebug_t *pCLPDebug) {
    UINT8_T bEnaByLaneChange = 0U; /* Enable flag of lane change, (0, 0~1, -) */
    UINT8_T bDisaStepDtctLf =
        0U; /* Disable flag of step detection, (0, 0~1, -) */
    UINT8_T bEnaDistYStepDtctLf =
        0U; /* Enable flag of lateral distance step detection, (0, 0~1, -)*/
    UINT8_T bLatDistStepTrigLf =
        0U; /* Lateral distance trigger flag, (0, 0~1, -) */
    UINT8_T bRawValidNewCorrLf =
        0U; /* Raw valid flag for the corridor is new, (0, 0~1, -) */
    UINT8_T bLaneWidthAgainNormalLf =
        0U; /* Lane width again normal flag, (0, 0~1, -) */
    UINT8_T
    bRawDistYStepDtctLf = 0U; /* Raw flag(by S-R trigger) for distance Y
                                 step detection, (0, 0~1, -) */
    UINT8_T
    bStepDebouncedLf = 0U; /* Step debounced result for distance Y step
                              detection, (0, 0~1, -) */
    UINT8_T bRawStepDebouncedLf =
        0U; /* The flag of raw step debounced, (0, 0~1, -) */

    UINT8_T bResetRawDistYStepDtctLf =
        0U; /* Reset flag for raw lateral distance step detection, (0, 0~1, -)
             */
    UINT8_T bEnaByPosY0UlpLf = 0U;
    REAL32_T fAbsDevPosY0UlpLf = 0.0F;

    UINT8_T bTempCLP = 0; /* Temp variable, (0, 0~1, -) */
    // REAL32_T fTempCLP = 0.0F; /* Temp variable, (0, X~X, -) */
    UINT8_T bResetCLP = 0; /* Temp variable, (0, 0~1, -) */

    /* Lane change flag */
    if ((pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeLf) ||
        (pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeRi)) {
        bEnaByLaneChange = 1U;
    } else {
        bEnaByLaneChange = 0U;
    }

    /*************1.Set lane lateral distance step
     * detection********************************/
    /* Suppress step detection if
    -a lane boundary is not available
    -edge rising detection of the lane boundary availability
    -a lane change has been detected */
    if (((bLastAvailableUlpLf == 0U) && (pCLPInput->bAvailableUlpLf == 1U)) ||
        (pCLPInput->bAvailableUlpLf == 0U) || (bEnaByLaneChange == 1U)) {
        bDisaStepDtctLf = 1U;
    } else {
        bDisaStepDtctLf = 0U;
    }

    /* Set lateral distance step if
             - step detection is not suppressed
             - a lateral distance step has been detected
             - a too high lateral deviation has been measured in the last cycle
             - uncoupled lane bridging is triggered (exit ramp detected,
       uncoupled lane jump detected,uncoupled lane parallelism violated)*/
    fAbsDevPosY0UlpLf = TUE_CML_Abs_M(pCLPInput->fPosY0UlpLf - fLastPosY0UlpLf);
    if (fAbsDevPosY0UlpLf > pCLPParam->fDistYLimitStepDtct) {
        bEnaByPosY0UlpLf = 1U;
    } else {
        bEnaByPosY0UlpLf = 0U;
    }

    if (bDisaStepDtctLf == 0U) {
        if ((bEnaByPosY0UlpLf == 1U) || (pCLPInput->bLatDistDevLf == 1U) ||
            (pCLPInput->bLaneBridgeRi == 1U)) {
            bEnaDistYStepDtctLf = 1U;
        } else {
            bEnaDistYStepDtctLf = 0U;
        }
    } else {
        bEnaDistYStepDtctLf = 0;
    }

    /*************2.Lane lateral distance step detection
     * debounced**************************/
    if ((bEnaDistYStepDtctLf == 1U) &&
        (pCLPInput->bEnaByBridgePossible == 1U)) {
        bLatDistStepTrigLf = 1U;
    } else {
        bLatDistStepTrigLf = 0U;
    }

    CLP_LeftValidCorriAfterLatDisJump(pCLPInput, pCLPParam, pCLPDebug,
                                      bLatDistStepTrigLf, &bRawValidNewCorrLf,
                                      &bLaneWidthAgainNormalLf);

    /*************2.4 Debounce lateral step
     * detection***************************************/
    if (bEnaDistYStepDtctLf == 1U) {
        bTempCLP = 0U;
    } else {
        bTempCLP = 1U;
    }
    bRawStepDebouncedLf = TUE_CML_TurnOnDelay_M(
        bTempCLP, pCLPParam->fTurnOnTimeDistYStep, pCLPParam->fSysCycleTime,
        &fTimerStepDebouncedLf, bLastRawStepDebouncedLf);
    if ((bRawStepDebouncedLf == 1U) || (bEnaByLaneChange == 1U)) {
        bResetCLP = 1U;
    } else {
        bResetCLP = 0U;
    }
    bStepDebouncedLf = TUE_CML_SRTrigger_M(bEnaDistYStepDtctLf, bResetCLP,
                                           bLastStepDebouncedLf);

    /****************2.5 Left lateral distance step
     * detection***************************************/
    if ((bRawValidNewCorrLf == 1U) || (bLaneWidthAgainNormalLf == 1U) ||
        (pCLPInput->bEnaByBridgePossible == 0U) || (bEnaByLaneChange == 1U)) {
        bResetRawDistYStepDtctLf = 1U;
    } else {
        bResetRawDistYStepDtctLf = 0U;
    }

    bRawDistYStepDtctLf = TUE_CML_SRTrigger_M(
        bLatDistStepTrigLf, bResetRawDistYStepDtctLf, bLastRawDistYStepDtctLf);
    if ((bRawDistYStepDtctLf == 1U) || (bStepDebouncedLf == 1U)) {
        pCLPOutput->bDistYStepDtctLf = 1U;
    } else {
        pCLPOutput->bDistYStepDtctLf = 0U;
    }

    if ((bLastRawDistYStepDtctLf == 1U) && (bRawDistYStepDtctLf == 0U) &&
        (bLastRawValidNewCorrLf == 0U) && (bRawValidNewCorrLf == 1U)) {
        pCLPOutput->bValidnewCorridorLf = 1U;
    } else {
        pCLPOutput->bValidnewCorridorLf = 0U;
    }

    /*************************Out and debug**************************/
    pCLPDebug->bLastAvailableUlpLf = bLastAvailableUlpLf;
    pCLPDebug->bDisaStepDtctLf = bDisaStepDtctLf;
    pCLPDebug->bEnaByPosY0UlpLf = bEnaByPosY0UlpLf;
    pCLPDebug->fAbsDevPosY0UlpLf = fAbsDevPosY0UlpLf;
    pCLPDebug->bEnaDistYStepDtctLf = bEnaDistYStepDtctLf;
    pCLPDebug->bLatDistStepTrigLf = bLatDistStepTrigLf;
    pCLPDebug->bRawValidNewCorrLf = bRawValidNewCorrLf;
    pCLPDebug->bLaneWidthAgainNormalLf = bLaneWidthAgainNormalLf;
    pCLPDebug->bRawDistYStepDtctLf = bRawDistYStepDtctLf;
    pCLPDebug->bStepDebouncedLf = bStepDebouncedLf;
    pCLPDebug->bRawStepDebouncedLf = bRawStepDebouncedLf;
    pCLPDebug->fTimerStepDebouncedLf = fTimerStepDebouncedLf;
    pCLPDebug->bResetRawDistYStepDtctLf = bResetRawDistYStepDtctLf;

    /*************************Save last value************************/
    bLastRawValidNewCorrLf = bRawValidNewCorrLf;
    bLastAvailableUlpLf = pCLPInput->bAvailableUlpLf;
    fLastPosY0UlpLf = pCLPInput->fPosY0UlpLf;
    bLastRawStepDebouncedLf = bRawStepDebouncedLf;
    // bLastLaneWidthAgainNormalLf = bLaneWidthAgainNormalLf;
    bLastRawDistYStepDtctLf = bRawDistYStepDtctLf;
    bLastStepDebouncedLf = bStepDebouncedLf;
}

void CLP_RightValidCorriAfterLatDisJump(const sCLPInput_t *pCLPInput,
                                        const sCLPParam_t *pCLPParam,
                                        sCLPDebug_t *pCLPDebug,
                                        UINT8_T bLatDistStepTrigRi,
                                        UINT8_T *bRawValidNewCorrRi,
                                        UINT8_T *bLaneWidthAgainNormalRi) {
    REAL32_T fLaneWidthUlpRi = 0.0F;   /* Lane width , (0, 0~10, m) */
    UINT8_T bValidLaneWidthUlpRi = 0U; /* lane width valid flag, (0, 0~10, m) */
    REAL32_T fAbsDevLaneWidthUlpRi = 0.0F;
    REAL32_T fAbsDevLaneWidthFltRi = 0.0F;
    UINT8_T bInVldLaneWidthDevRi =
        0U; /* Enable flag by lane width for reset calculation (0, 0~1, -) */
    UINT8_T
    bEnaByLaneWidthFltRi = 0; /* Enable flag by lane width(from LFP) for
                                 reset calculation (0, 0~1, -) */
    /*************2.1 Lane Width valid
     * flag*************************************************/
    /* Ulp lane width*/
    fLaneWidthUlpRi =
        pCLPInput->fPosY0UlpLf * TUE_CML_Cos_M(pCLPInput->fHeadingUlpLf) -
        pCLPInput->fPosY0UlpRi * TUE_CML_Cos_M(pCLPInput->fHeadingUlpRi);
    fLaneWidthUlpRi = TUE_CML_Abs_M(fLaneWidthUlpRi);
    /* Determine current lane width validity for new corridor validation*/
    if (fLaneWidthUlpRi <=
            (pCLPParam->fMaxLaneWidth + pCLPParam->fMaxLaneWidthHyst) &&
        fLaneWidthUlpRi >=
            (pCLPParam->fMinLaneWidth + pCLPParam->fMinLaneWidthHyst)) {
        bValidLaneWidthUlpRi = 1U;
    } else {
        bValidLaneWidthUlpRi = 0U;
    }

    /*************2.2 Raw new lane corridor valid
     * flag**************************************/
    if ((bLastInVldLaneWidthDevRi == 1U) || (bLatDistStepTrigRi == 1U)) {
        fHistoryLaneWidthRi = fLaneWidthUlpRi;
    } else {
        fHistoryLaneWidthRi =
            fHistoryLaneWidthRi; /* Value store, remain unchanged */
    }

    fAbsDevLaneWidthUlpRi =
        TUE_CML_Abs_M(fLaneWidthUlpRi - fHistoryLaneWidthRi);
    if (fAbsDevLaneWidthUlpRi > pCLPParam->TolRangeNewCorr) {
        bInVldLaneWidthDevRi = 1U;
    } else {
        bInVldLaneWidthDevRi = 0U;
    }
    /* A new corridor is valid if:
               -a lateral step has been triggered (while bridging is allowed)
               -the current measured lane width is valid
               -a new stable corridor is determined (lane width remains in a
       certain range
                    for at certain time span) */
    if ((bLatDistStepTrigRi == 1U) || (bInVldLaneWidthDevRi == 1U) ||
        (bValidLaneWidthUlpRi == 0U)) {
        *bRawValidNewCorrRi = 0U;
        fTimerRawValidNewCorrRi = pCLPParam->fTdNewCorridorOff;
    } else {
        if (fTimerRawValidNewCorrRi > pCLPParam->fSysCycleTime) {
            fTimerRawValidNewCorrRi =
                fTimerRawValidNewCorrRi - pCLPParam->fSysCycleTime;
            *bRawValidNewCorrRi = 0U;
        } else {
            fTimerRawValidNewCorrRi = 0.0F;
            *bRawValidNewCorrRi = 1U;
        }
    }

    /*************2.3 Lane width again normal
     * flag******************************************/
    /* The lane width is valid again if:
    -a lateral step has been triggered(while bridging is allowed)
    -the current lane width is in range width the lane width before
     the step detection for at certain time span */
    if (bLatDistStepTrigRi == 1U) {
        fHistoryLaneWidthFltRi = pCLPInput->fLaneWidth;
    } else {
        fHistoryLaneWidthFltRi =
            fHistoryLaneWidthFltRi; /* Value store, remain unchanged */
    }
    /* The corridor is considered new if the width changes more than this value
     */
    fAbsDevLaneWidthFltRi =
        TUE_CML_Abs_M(fLaneWidthUlpRi - fHistoryLaneWidthFltRi);
    if (fAbsDevLaneWidthFltRi > pCLPParam->TolRangeDistY) {
        bEnaByLaneWidthFltRi = 1U;
    } else {
        bEnaByLaneWidthFltRi = 0U;
    }
    if ((bEnaByLaneWidthFltRi == 1U) || (bLatDistStepTrigRi == 1U)) {
        *bLaneWidthAgainNormalRi = 0U;
        fTimerAgainNormalRi = pCLPParam->fTdAgainNormalOff;
    } else {
        if (fTimerAgainNormalRi > pCLPParam->fSysCycleTime) {
            fTimerAgainNormalRi =
                fTimerAgainNormalRi - pCLPParam->fSysCycleTime;
            *bLaneWidthAgainNormalRi = 0U;
        } else {
            fTimerRawValidNewCorrRi = 0.0F;
            *bLaneWidthAgainNormalRi = 1U;
        }
    }
    pCLPDebug->fLaneWidthUlpRi = fLaneWidthUlpRi;
    pCLPDebug->bValidLaneWidthUlpRi = bValidLaneWidthUlpRi;
    pCLPDebug->fHistoryLaneWidthRi = fHistoryLaneWidthRi;
    pCLPDebug->bInVldLaneWidthDevRi = bInVldLaneWidthDevRi;
    pCLPDebug->fTimerRawValidNewCorrRi = fTimerRawValidNewCorrRi;
    pCLPDebug->fAbsDevLaneWidthFltRi = fAbsDevLaneWidthFltRi;
    pCLPDebug->bEnaByLaneWidthFltRi = bEnaByLaneWidthFltRi;
    pCLPDebug->fTimerAgainNormalRi = fTimerAgainNormalRi;

    bLastInVldLaneWidthDevRi = bInVldLaneWidthDevRi;
}

/****************************************************************************************
        @fn           RightLatDistStepDtct
        @brief        Detect whether the right lateral distance step is too
large
        @description  Right lateral distance step detection:
                                          1.Set lane lateral distance step
detection;
                                          2.Lane lateral distance step detection
debounced:
                                                  2.1 Lane Width valid flag;
                                                  2.2 Raw new lane corridor
valid flag;
                                                  2.3 Lane width again normal
flag;
                                                  2.4 Debounce lateral step
detection.
        @param[in]    pCLPInput : Input for RightLatDistStepDtct
        @param[in]    pCLPParam : Parameter for rightLatDistStepDtct
        @param[out]   pCLPOutput: Output for RightLatDistStepDtct
        @param[out]   pCLPDebug : Debug(measurement) for RightLatDistStepDtct
        @return       void
        @startuml
        title RightLatDistStepDtct
        (*)--> 1.SetLaneLateralDistanceStepDetection
           --> 2.1 LaneWidthValidFlag
           --> 2.2 RawNewLaneCorridorValidFlag
           --> 2.3 LaneWidthAgainNormalFlag
           --> 2.4 DebounceLateralStepDetection
           --> (*)
        @enduml
******************************************************************************************/
void RightLatDistStepDtct(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug) {
    UINT8_T bEnaByLaneChange = 0U; /* Enable flag of lane change, (0, 0~1, -) */
    UINT8_T bDisaStepDtctRi =
        0U; /* Disable flag of step detection, (0, 0~1, -) */
    UINT8_T bEnaDistYStepDtctRi =
        0U; /* Enable flag of lateral distance step detection, (0, 0~1, -)*/
    UINT8_T bLatDistStepTrigRi =
        0U; /* Lateral distance trigger flag, (0, 0~1, -) */
    UINT8_T bRawValidNewCorrRi =
        0U; /* Raw valid flag for the corridor is new, (0, 0~1, -) */
    UINT8_T bLaneWidthAgainNormalRi =
        0U; /* Lane width again normal flag, (0, 0~1, -) */
    UINT8_T
    bRawDistYStepDtctRi = 0U; /* Raw flag(by S-R trigger) for distance Y
                                 step detection, (0, 0~1, -) */
    UINT8_T
    bStepDebouncedRi = 0U; /* Step debounced result for distance Y step
                              detection, (0, 0~1, -) */
    UINT8_T bRawStepDebouncedRi =
        0U; /* The flag of raw step debounced, (0, 0~1, -) */

    UINT8_T bResetRawDistYStepDtctRi =
        0U; /* Reset flag for raw lateral distance step detection, (0, 0~1, -)
             */
    UINT8_T bEnaByPosY0UlpRi = 0U;
    REAL32_T fAbsDevPosY0UlpRi = 0.0F;

    UINT8_T bTempCLP = 0; /* Temp variable, (0, 0~1, -) */
    // REAL32_T fTempCLP = 0.0F; /* Temp variable, (0, X~X, -) */
    UINT8_T bResetCLP = 0; /* Temp variable, (0, 0~1, -) */

    /* Lane change flag */
    if ((pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeLf) ||
        (pCLPInput->uLaneChange == pCLPParam->uConstLaneChangeRi)) {
        bEnaByLaneChange = 1U;
    } else {
        bEnaByLaneChange = 0U;
    }

    /*************1.Set lane lateral distance step
     * detection********************************/
    /* Suppress step detection if
    -a lane boundary is not available
    -edge rising detection of the lane boundary availability
    -a lane change has been detected */
    if (((bLastAvailableUlpRi == 0U) && (pCLPInput->bAvailableUlpRi == 1U)) ||
        (pCLPInput->bAvailableUlpRi == 0U) || (bEnaByLaneChange == 1U)) {
        bDisaStepDtctRi = 1U;
    } else {
        bDisaStepDtctRi = 0U;
    }

    /* Set lateral distance step if
             - step detection is not suppressed
             - a lateral distance step has been detected
             - a too high lateral deviation has been measured in the last cycle
             - uncoupled lane bridging is triggered (exit ramp detected,
       uncoupled lane jump detected,uncoupled lane parallelism violated)*/
    fAbsDevPosY0UlpRi = TUE_CML_Abs_M(pCLPInput->fPosY0UlpRi - fLastPosY0UlpRi);
    if (fAbsDevPosY0UlpRi > pCLPParam->fDistYLimitStepDtct) {
        bEnaByPosY0UlpRi = 1U;
    } else {
        bEnaByPosY0UlpRi = 0U;
    }

    if (bDisaStepDtctRi == 0U) {
        if ((bEnaByPosY0UlpRi == 1U) || (pCLPInput->bLatDistDevRi == 1U) ||
            (pCLPInput->bLaneBridgeLf == 1U)) {
            bEnaDistYStepDtctRi = 1U;
        } else {
            bEnaDistYStepDtctRi = 0U;
        }
    } else {
        bEnaDistYStepDtctRi = 0;
    }

    /*************2.Lane lateral distance step detection
     * debounced**************************/
    if ((bEnaDistYStepDtctRi == 1U) &&
        (pCLPInput->bEnaByBridgePossible == 1U)) {
        bLatDistStepTrigRi = 1U;
    } else {
        bLatDistStepTrigRi = 0U;
    }
    CLP_RightValidCorriAfterLatDisJump(pCLPInput, pCLPParam, pCLPDebug,
                                       bLatDistStepTrigRi, &bRawValidNewCorrRi,
                                       &bLaneWidthAgainNormalRi);

    /*************2.4 Debounce lateral step
     * detection***************************************/
    if (bEnaDistYStepDtctRi == 1U) {
        bTempCLP = 0U;
    } else {
        bTempCLP = 1U;
    }
    bRawStepDebouncedRi = TUE_CML_TurnOnDelay_M(
        bTempCLP, pCLPParam->fTurnOnTimeDistYStep, pCLPParam->fSysCycleTime,
        &fTimerStepDebouncedRi, bLastRawStepDebouncedRi);
    if ((bRawStepDebouncedRi == 1U) || (bEnaByLaneChange == 1U)) {
        bResetCLP = 1U;
    } else {
        bResetCLP = 0U;
    }
    bStepDebouncedRi = TUE_CML_SRTrigger_M(bEnaDistYStepDtctRi, bResetCLP,
                                           bLastStepDebouncedRi);

    /****************2.5 Left lateral distance step
     * detection***************************************/
    if ((bRawValidNewCorrRi == 1U) || (bLaneWidthAgainNormalRi == 1U) ||
        (pCLPInput->bEnaByBridgePossible == 0U) || (bEnaByLaneChange == 1U)) {
        bResetRawDistYStepDtctRi = 1U;
    } else {
        bResetRawDistYStepDtctRi = 0U;
    }

    bRawDistYStepDtctRi = TUE_CML_SRTrigger_M(
        bLatDistStepTrigRi, bResetRawDistYStepDtctRi, bLastRawDistYStepDtctRi);
    if ((bRawDistYStepDtctRi == 1U) || (bStepDebouncedRi == 1U)) {
        pCLPOutput->bDistYStepDtctRi = 1U;
    } else {
        pCLPOutput->bDistYStepDtctRi = 0U;
    }

    if ((bLastRawDistYStepDtctRi == 1U) && (bRawDistYStepDtctRi == 0U) &&
        (bLastRawValidNewCorrRi == 0U) && (bRawValidNewCorrRi == 1U)) {
        pCLPOutput->bValidnewCorridorRi = 1U;
    } else {
        pCLPOutput->bValidnewCorridorRi = 0U;
    }

    /*************************Out and debug**************************/
    pCLPDebug->bLastAvailableUlpRi = bLastAvailableUlpRi;
    pCLPDebug->bDisaStepDtctRi = bDisaStepDtctRi;
    pCLPDebug->bEnaByPosY0UlpRi = bEnaByPosY0UlpRi;
    pCLPDebug->fAbsDevPosY0UlpRi = fAbsDevPosY0UlpRi;
    pCLPDebug->bEnaDistYStepDtctRi = bEnaDistYStepDtctRi;
    pCLPDebug->bLatDistStepTrigRi = bLatDistStepTrigRi;
    pCLPDebug->bRawValidNewCorrRi = bRawValidNewCorrRi;
    pCLPDebug->bLaneWidthAgainNormalRi = bLaneWidthAgainNormalRi;
    pCLPDebug->bRawDistYStepDtctRi = bRawDistYStepDtctRi;
    pCLPDebug->bStepDebouncedRi = bStepDebouncedRi;
    pCLPDebug->bRawStepDebouncedRi = bRawStepDebouncedRi;
    pCLPDebug->fTimerStepDebouncedRi = fTimerStepDebouncedRi;
    pCLPDebug->bResetRawDistYStepDtctRi = bResetRawDistYStepDtctRi;

    /*************************Save last value************************/
    bLastAvailableUlpRi = pCLPInput->bAvailableUlpRi;
    fLastPosY0UlpRi = pCLPInput->fPosY0UlpRi;
    bLastRawStepDebouncedRi = bRawStepDebouncedRi;
    // bLastLaneWidthAgainNormalRi = bLaneWidthAgainNormalRi;
    bLastRawDistYStepDtctRi = bRawDistYStepDtctRi;
    bLastStepDebouncedRi = bStepDebouncedRi;
    bLastRawValidNewCorrRi = bRawValidNewCorrRi;
}

/****************************************************************************************
        @fn           OutRangeCheck
        @brief        Input data range check
        @description  Out range check:
                                          1.Bit 0: Left lateral distance out of
range
                                          2.Bit 1: Right lateral distance out of
range
                                          3.Bit 2: Left heading angle out of
range
                                          4.Bit 3: Right heading angle out of
range
                                          5.Bit 4: Left curvature out of range
                                          6.Bit 5: Right curvature out of range
                                          7.Bit 6: Left curvature rate out of
range
                                          8.Bit 7: Right curvature rate out of
range
                                          9.Bit 8: Left valid length out of
range
                                          10.Bit 9: Right valid length out of
range
        @param[in]    pCLPInput : Input for OutRangeCheck
        @param[in]    pCLPParam : Parameter for OutRangeCheck
        @param[out]   pCLPOutput: Output for OutRangeCheck
        @param[out]   pCLPDebug : Debug(measurement) for OutRangeCheck
        @return       void
        @startuml
        title OutRangeCheck
        (*)--> 1.Bit 0: Left lateral distance out of range
                --> (*)
        (*)--> 2.Bit 1: Right lateral distance out of range
                --> (*)
        (*)--> 3.Bit 2: Left heading angle out of range
           --> (*)
        (*)--> 4.Bit 3: Right heading angle out of range
                --> (*)
        (*)--> 5.Bit 4: Left curvature out of range
                --> (*)
        (*)--> 6.Bit 5: Right curvature out of range
                --> (*)
        (*)--> 7.Bit 6: Left curvature rate out of range
                --> (*)
        (*)--> 8.Bit 7: Right curvature rate out of range
                --> (*)
        (*)--> 9.Bit 8: Left valid length out of range
                --> (*)
        (*)--> 10.Bit 9: Right valid length out of range
           --> (*)
        @enduml
******************************************************************************************/
void OutRangeCheck(const sCLPInput_t *pCLPInput,
                   const sCLPParam_t *pCLPParam,
                   sCLPOutput_t *pCLPOutput,
                   sCLPDebug_t *pCLPDebug) {
    UINT8_T bOutRangePosY0Lf =
        0U; /* Flag that left position Y0 is out of range (0, 0~1, -) */
    UINT8_T bOutRangeHeadingLf =
        0U; /* Flag that left the heading angle is out of range (0, 0~1, -) */
    UINT8_T bOutRangeCrvLf =
        0U; /* Flag that left curvature is out of range (0, 0~1, -) */
    UINT8_T bOutRangeCrvRateLf =
        0U; /* Flag that left curvature rate is out of range (0, 0~1, -) */
    UINT8_T bOutRangeLengthLf =
        0U; /* Flag that left valid length is out of range (0, 0~1, -) */
    UINT8_T bOutRangePosY0Ri =
        0U; /* Flag that right position Y0 is out of range (0, 0~1, -) */
    UINT8_T bOutRangeHeadingRi =
        0U; /* Flag that right the heading angle is out of range (0, 0~1, -) */
    UINT8_T bOutRangeCrvRi =
        0U; /* Flag that right curvature is out of range (0, 0~1, -) */
    UINT8_T bOutRangeCrvRateRi =
        0U; /* Flag that right curvature rate is out of range (0, 0~1, -) */
    UINT8_T bOutRangeLengthRi =
        0U; /* Flag that right valid length is out of range (0, 0~1, -) */

    REAL32_T fMinRangePosY0 =
        0.0F; /* Minimum range check value for lateral position, (6, 0~10, m) */
    REAL32_T fMinRangeHeading = 0.0F; /* Minimum range check value for heading
                                         angle, (0.5, 0~0.7854, rad) */
    REAL32_T fMinRangeCrv =
        0.0F; /* Minimum range check value for curvature, (0.01, 0~10, 1/m) */
    REAL32_T fMinRangeCrvRate = 0.0F; /* Minimum range check value for curvature
                                         rate, (0.001, 0~10, 1/m^2) */
    REAL32_T fMinRangeLength =
        0.0F; /* Minimum range check value for valid length, (300, 0~10, m) */

    /* Calculate minimum range check value */
    fMinRangePosY0 = pCLPParam->fMaxRangePosY0 * pCLPParam->fCoeffHystMaxRange;
    fMinRangeHeading =
        pCLPParam->fMaxRangeHeading * pCLPParam->fCoeffHystMaxRange;
    fMinRangeCrv = pCLPParam->fMaxRangeCrv * pCLPParam->fCoeffHystMaxRange;
    fMinRangeCrvRate =
        pCLPParam->fMaxRangeCrvRate * pCLPParam->fCoeffHystMaxRange;
    fMinRangeLength =
        pCLPParam->fMaxRangeLength * pCLPParam->fCoeffHystMaxRange;

    /* Bit 0: Left lateral distance out of range */
    bOutRangePosY0Lf =
        TUE_CML_Hysteresis_M(pCLPInput->fPosY0UlpLf, pCLPParam->fMaxRangePosY0,
                             fMinRangePosY0, bLastOutRangePosY0Lf);
    if (bOutRangePosY0Lf == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 0U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 0U);
    }

    /* Bit 1: Right lateral distance out of range */
    bOutRangePosY0Ri = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pCLPInput->fPosY0UlpRi), pCLPParam->fMaxRangePosY0,
        fMinRangePosY0, bLastOutRangePosY0Ri);
    if (bOutRangePosY0Ri == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 1U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 1U);
    }

    /* Bit 2: Left heading angle out of range */
    bOutRangeHeadingLf = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pCLPInput->fHeadingUlpLf), pCLPParam->fMaxRangeHeading,
        fMinRangeHeading, bLastOutRangeHeadingLf);
    if (bOutRangeHeadingLf == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 2U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 2U);
    }

    /* Bit 3: Right heading angle out of range */
    bOutRangeHeadingRi = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pCLPInput->fHeadingUlpRi), pCLPParam->fMaxRangeHeading,
        fMinRangeHeading, bLastOutRangeHeadingRi);
    if (bOutRangeHeadingRi == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 3U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 3U);
    }

    /* Bit 4: Left curvature out of range */
    bOutRangeCrvLf = TUE_CML_Hysteresis_M(TUE_CML_Abs_M(pCLPInput->fCrvUlpLf),
                                          pCLPParam->fMaxRangeCrv, fMinRangeCrv,
                                          bLastOutRangeCrvLf);
    if (bOutRangeCrvLf == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 4U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 4U);
    }

    /* Bit 5: Right curvature out of range */
    bOutRangeCrvRi = TUE_CML_Hysteresis_M(TUE_CML_Abs_M(pCLPInput->fCrvUlpRi),
                                          pCLPParam->fMaxRangeCrv, fMinRangeCrv,
                                          bLastOutRangeCrvRi);
    if (bOutRangeCrvRi == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 5U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 5U);
    }

    /* Bit 6: Left curvature rate out of range */
    bOutRangeCrvRateLf = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pCLPInput->fCrvRateUlpLf), pCLPParam->fMaxRangeCrvRate,
        fMinRangeCrvRate, bLastOutRangeCrvRateLf);
    if (bOutRangeCrvRateLf == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 6U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 6U);
    }

    /* Bit 7: Right curvature rate out of range */
    bOutRangeCrvRateRi = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pCLPInput->fCrvRateUlpRi), pCLPParam->fMaxRangeCrvRate,
        fMinRangeCrvRate, bLastOutRangeCrvRateRi);
    if (bOutRangeCrvRateRi == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 7U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 7U);
    }

    /* Bit 8: Left valid length out of range */
    bOutRangeLengthLf = TUE_CML_Hysteresis_M(
        pCLPInput->fValidLengthUlpLf, pCLPParam->fMaxRangeLength,
        fMinRangeLength, bLastOutRangeLengthLf);
    if (bOutRangeLengthLf == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 8U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 8U);
    }

    /* Bit 9: Right valid length out of range */
    bOutRangeLengthRi = TUE_CML_Hysteresis_M(
        pCLPInput->fValidLengthUlpRi, pCLPParam->fMaxRangeLength,
        fMinRangeLength, bLastOutRangeLengthRi);
    if (bOutRangeLengthRi == 1U) {
        TUE_CML_Setbit_M(pCLPOutput->uRangeCheckQualifier, 9U);
    } else {
        TUE_CML_Clrbit_M(pCLPOutput->uRangeCheckQualifier, 9U);
    }

    /****************************Ouput and
     * debug***************************************/
    pCLPDebug->bOutRangePosY0Lf = bOutRangePosY0Lf;
    pCLPDebug->bOutRangeHeadingLf = bOutRangeHeadingLf;
    pCLPDebug->bOutRangeCrvLf = bOutRangeCrvLf;
    pCLPDebug->bOutRangeCrvRateLf = bOutRangeCrvRateLf;
    pCLPDebug->bOutRangeLengthLf = bOutRangeLengthLf;
    pCLPDebug->bOutRangePosY0Ri = bOutRangePosY0Ri;
    pCLPDebug->bOutRangeHeadingRi = bOutRangeHeadingRi;
    pCLPDebug->bOutRangeCrvRi = bOutRangeCrvRi;
    pCLPDebug->bOutRangeCrvRateRi = bOutRangeCrvRateRi;
    pCLPDebug->bOutRangeLengthRi = bOutRangeLengthRi;
    pCLPDebug->fMinRangePosY0 = fMinRangePosY0;
    pCLPDebug->fMinRangeHeading = fMinRangeHeading;
    pCLPDebug->fMinRangeCrv = fMinRangeCrv;
    pCLPDebug->fMinRangeCrvRate = fMinRangeCrvRate;
    pCLPDebug->fMinRangeLength = fMinRangeLength;

    /****************************Save last
     * value***************************************/
    bLastOutRangePosY0Lf = bOutRangePosY0Lf;
    bLastOutRangeHeadingLf = bOutRangeHeadingLf;
    bLastOutRangeCrvLf = bOutRangeCrvLf;
    bLastOutRangeCrvRateLf = bOutRangeCrvRateLf;
    bLastOutRangeLengthLf = bOutRangeLengthLf;
    bLastOutRangePosY0Ri = bOutRangePosY0Ri;
    bLastOutRangeHeadingRi = bOutRangeHeadingRi;
    bLastOutRangeCrvRi = bOutRangeCrvRi;
    bLastOutRangeCrvRateRi = bOutRangeCrvRateRi;
    bLastOutRangeLengthRi = bOutRangeLengthRi;
}

/****************************************************************************************
        @fn           CheckLaneValidity
        @brief        Check the validity of the lane after ULP
        @description  Check lane(left/right) validity:
                                          1.Validate ego lane availability;
                                          2.Lateral distance step detection;
                                          3.Validate length;
                                          4.Check lane type;
                                          5.Check lane color;
                                          6.Range checks;
                                          7.Uncoupled lane bridge;
                                          8.Lane quality evaluation.
        @param[in]    pCLPInput : Input for CLV
        @param[in]    pCLPParam : Parameter for CLV
        @param[out]   pCLPOutput: Output for CLV
        @param[out]   pCLPDebug : Debug(measurement) for CLV
        @return       void
        @startuml
        title CheckLaneValidity
        (*)--> 1.Validate ego lane availability
           --> (*)
        (*)--> 2.Lateral distance step detection
           --> (*)
        (*)--> 3.Validate length
           --> (*)
        (*)--> 4.Check lane type
           --> (*)
        (*)--> 5.Check lane color
           --> (*)
        (*)--> 6.Range checks
           --> (*)
        (*)--> 7.Uncoupled lane bridge
           --> (*)
        (*)--> 8.Lane quality evaluation
           --> (*)
        @enduml
******************************************************************************************/
void CheckLaneValidity(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug) {
    REAL32_T fMinValidlength =
        0.0F; /* Maximum valid lane length, (0, -15~15, m) */
    REAL32_T fHystMinValidlength =
        0.0F; /* Maximum valid lane length, (0, -15~15, m) */
    UINT8_T bRawValidLengthLf =
        0U; /* Raw validity of left lane length, (0, 0~1, -) */
    UINT8_T bValidLengthLf =
        0U; /* Raw validity of left lane length, (0, 0~1, -) */

    UINT8_T bRawValidLengthRi =
        0U; /* Raw validity of right lane length, (0, 0~1, -) */

    UINT8_T bValidLengthRi =
        0U; /* Raw validity of right lane length, (0, 0~1, -) */

    // UINT8_T bTempCLP = 0U;
    // UINT16_T uTempCLP = 0U;
    REAL32_T fTempCLP = 0.0F;
    UINT16_T uStMarerTypeLf = 0U;
    UINT16_T uStMarerTypeRi = 0U;
    UINT16_T uStLaneColorLf = 0U;
    UINT16_T uStLaneColorRi = 0U;

    UINT8_T bLengthValidSetLf = 0U;
    UINT8_T bLengthValidSetRi = 0U;
    UINT8_T bLengthValidResetLf = 0U;
    UINT8_T bLengthValidResetRi = 0U;
    UINT8_T bLnQualityInvalidLf = 0U;
    UINT8_T bLnQualityInvalidRi = 0U;

    /***********************************************CheckLeftLaneProperites************************************************************/
    /****************************1.Validate ego lane
     * availability********************************/
    /****************For left lane***********************/
    if (pCLPInput->bAvailableUlpLf == 1U) {
        pCLPOutput->bNotAvailableLf = 0U;
    } else {
        pCLPOutput->bNotAvailableLf = 1U;
    }

    /****************For right lane***********************/
    if (pCLPInput->bAvailableUlpRi == 1U) {
        pCLPOutput->bNotAvailableRi = 0U;
    } else {
        pCLPOutput->bNotAvailableRi = 1U;
    }

    /****************************2.Lateral distance step
     * detection********************************/
    LeftLatDistStepDtct(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);
    RightLatDistStepDtct(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    pCLPOutput->bNewCorridorValid =
        pCLPOutput->bValidnewCorridorLf || pCLPOutput->bValidnewCorridorRi;

    /****************************3.Validate
     * length********************************/
    // TUE_CML_CalculatePolygonValue2D(6U, pCLPParam->bUseABDSineWave)
    fMinValidlength = TUE_CML_LookUpTable2D(
        (pCLPInput->fVehVelX),
        ((const REAL32_T *)&(pCLPParam->fMinValidLength_X[0])),
        ((const REAL32_T *)&(pCLPParam->fMinValidLength_M[0])), 6U);
    fHystMinValidlength = TUE_CML_LookUpTable2D(
        (pCLPInput->fVehVelX),
        ((const REAL32_T *)&(pCLPParam->fHystMinValidLength_X[0])),
        ((const REAL32_T *)&(pCLPParam->fHystMinValidLength_M[0])), 6U);

    /* RSFlipFlop and turn off delay for left lane length invalidity */
    fTempCLP = fMinValidlength - fHystMinValidlength;

    bLengthValidSetLf = pCLPInput->fValidLengthUlpLf > fMinValidlength;
    bLengthValidResetLf = fTempCLP > pCLPInput->fValidLengthUlpLf;
    bRawValidLengthLf = TUE_CML_SRTrigger_M(
        bLengthValidSetLf, bLengthValidResetLf, bLastRawValidLengthLf);

    bValidLengthLf = TUE_CML_TurnOffDelay_M(
        bRawValidLengthLf, pCLPParam->fTdlengthValid, pCLPParam->fSysCycleTime,
        &fTimerLengthValidLf, bLastValidLengthLf);

    if (bValidLengthLf == 1U) {
        pCLPOutput->bInValidLengthLf = 0U;
    } else {
        pCLPOutput->bInValidLengthLf = 1U;
    }

    /* RSFlipFlop and turn off delay for right lane length invalidity */
    fTempCLP = fMinValidlength - fHystMinValidlength;

    bLengthValidSetRi = pCLPInput->fValidLengthUlpRi > fMinValidlength;
    bLengthValidResetRi = fTempCLP > pCLPInput->fValidLengthUlpRi;
    bRawValidLengthRi = TUE_CML_SRTrigger_M(
        bLengthValidSetRi, bLengthValidResetRi, bLastRawValidLengthRi);

    bValidLengthRi = TUE_CML_TurnOffDelay_M(
        bRawValidLengthRi, pCLPParam->fTdlengthValid, pCLPParam->fSysCycleTime,
        &fTimerLengthValidRi, bLastValidLengthRi);
    if (bValidLengthRi == 0U) {
        pCLPOutput->bInValidLengthRi = 1U;
    } else {
        pCLPOutput->bInValidLengthRi = 0U;
    }

    /****************************4.Check lane
     * type*************************************/
    uStMarerTypeLf =
        (pCLPParam->uBtfValidMarkerType >> pCLPInput->uMarkerTypeLf) & 0x01;
    if (uStMarerTypeLf == 0U) {
        pCLPOutput->bInValidMakerTypeLf = 1U;
    } else {
        pCLPOutput->bInValidMakerTypeLf = 0U;
    }

    uStMarerTypeRi =
        (pCLPParam->uBtfValidMarkerType >> pCLPInput->uMarkerTypeRi) & 0x01;
    if (uStMarerTypeRi == 0U) {
        pCLPOutput->bInValidMakerTypeRi = 1U;
    } else {
        pCLPOutput->bInValidMakerTypeRi = 0U;
    }
    /****************************5.Check lane
     * color*************************************/
    uStLaneColorLf =
        (pCLPParam->uBtfValidLaneColor >> pCLPInput->uLaneColorLf) & 0x01;
    if (uStLaneColorLf == 0U) {
        pCLPOutput->bInValidLaneColorLf = 1U;
    } else {
        pCLPOutput->bInValidLaneColorLf = 0U;
    }

    uStLaneColorRi =
        (pCLPParam->uBtfValidLaneColor >> pCLPInput->uLaneColorRi) & 0x01;
    if (uStLaneColorRi == 0U) {
        pCLPOutput->bInValidLaneColorRi = 1U;
    } else {
        pCLPOutput->bInValidLaneColorRi = 0U;
    }

    /****************************6.Range
     * checks*****************************************/
    OutRangeCheck(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /****************************7.Uncoupled lane
     * bridge********************************/
    pCLPOutput->bLaneVirtualCplLf = pCLPInput->bLaneBridgeLf;
    pCLPOutput->bLaneVirtualCplRi = pCLPInput->bLaneBridgeRi;

    /****************************8.Lane quality
     * evaluation******************************/
    bLnQualityInvalidLf = TUE_CML_Hysteresis_M(
        pCLPInput->fOverallQualityLf, 40.f, 29.f, bLastLnQualityInvalidLf);
    bLnQualityInvalidRi = TUE_CML_Hysteresis_M(
        pCLPInput->fOverallQualityRi, 40.f, 29.f, bLastLnQualityInvalidRi);

    pCLPOutput->bLnQualityInvalidLf = !bLnQualityInvalidLf;
    pCLPOutput->bLnQualityInvalidRi = !bLnQualityInvalidRi;

    /****************************Ouput and
     * debug***************************************/
    pCLPDebug->fMinValidlength = fMinValidlength;
    pCLPDebug->fHystMinValidlength = fHystMinValidlength;
    pCLPDebug->bRawValidLengthLf = bRawValidLengthLf;
    pCLPDebug->bValidLengthLf = bValidLengthLf;
    pCLPDebug->fTimerLengthValidLf = fTimerLengthValidLf;
    pCLPDebug->bRawValidLengthRi = bRawValidLengthRi;
    pCLPDebug->bValidLengthRi = bValidLengthRi;
    pCLPDebug->fTimerLengthValidRi = fTimerLengthValidRi;
    pCLPDebug->uStMarerTypeLf = uStMarerTypeLf;
    pCLPDebug->uStMarerTypeRi = uStMarerTypeRi;
    pCLPDebug->uStLaneColorLf = uStLaneColorLf;
    pCLPDebug->uStLaneColorRi = uStLaneColorRi;

    /****************************Save last
     * value***************************************/
    bLastRawValidLengthLf = bRawValidLengthLf;
    bLastRawValidLengthRi = bRawValidLengthRi;
    bLastValidLengthLf = bValidLengthLf;
    bLastValidLengthRi = bValidLengthRi;
    bLastLnQualityInvalidLf = bLnQualityInvalidLf;
    bLastLnQualityInvalidRi = bLnQualityInvalidRi;
}

/****************************************************************************************
        @fn           INPUT_CheckLaneProperity
        @brief        Input for CheckLaneProperity
        @description  Input for CheckLaneProperity:
                                        The input signal is processed uniformly
        @param[in]    pLBPInput : Input from external input
        @param[in]    pLBPParam : Input for external parameter
        @param[in]    pULPOutput : Input from CLP output
        @param[in]    pLFPOutput : Input from CLP output
        @param[out]   pCLPInput:  Input for INPUT_CheckLaneProperity
        @param[out]   pCLPParam : Parameter for INPUT_CheckLaneProperity
        @return       void
        @startuml
        title INPUT_CheckLaneProperity
        (*)--> 1.pLBPInput
           --> (*)
        (*)--> 2.pLBPParam
           --> (*)
        (*)--> 3.pULPOutput
           --> (*)
        (*)--> 4.pLFPOutput
           --> (*)
        @enduml
 ******************************************************************************************/
void INPUT_CheckLaneProperity(const sLBPInput_t *pLBPInput,
                              const sLBPParam_t *pLBPParam,
                              const sULPOutput_t *pULPOutput,
                              const sLFPOutput_t *pLFPOutput,
                              sCLPInput_t *pCLPInput,
                              sCLPParam_t *pCLPParam) {
    REAL32_T fMinValidLength_X[6] = {0.0F, 5.0F, 10.0F, 20.0F, 40.0F, 60.0F};
    REAL32_T fMinValidLength_M[6] = {15.0F, 18.0F, 20.0F, 20.0F, 20.0F, 20.0F};
    REAL32_T fHystMinValidLength_X[6] = {0.0F,  5.0F,  10.0F,
                                         20.0F, 40.0F, 60.0F};
    REAL32_T fHystMinValidLength_M[6] = {0.3F, 5.0F, 5.0F, 5.0F, 5.0F, 5.0F};

    pCLPInput->bAvailableUnCplLf = pLBPInput->bAvailable[UN_CPL_LF];
    pCLPInput->bAvailableUnCplRi = pLBPInput->bAvailable[UN_CPL_RI];
    pCLPInput->bAvailableCplLf = pLBPInput->bAvailable[CPL_LF];
    pCLPInput->bAvailableCplRi = pLBPInput->bAvailable[CPL_RI];
    pCLPInput->fPosY0UnCplLf = pLBPInput->fPosY0[UN_CPL_LF];
    pCLPInput->fPosY0UnCplRi = pLBPInput->fPosY0[UN_CPL_RI];
    pCLPInput->fPosY0CplLf = pLBPInput->fPosY0[CPL_LF];
    pCLPInput->fPosY0CplRi = pLBPInput->fPosY0[CPL_RI];
    pCLPInput->fHeadingUnCplLf = pLBPInput->fHeadingAngle[UN_CPL_LF];
    pCLPInput->fHeadingUnCplRi = pLBPInput->fHeadingAngle[UN_CPL_RI];
    pCLPInput->fHeadingCplLf = pLBPInput->fHeadingAngle[CPL_LF];
    pCLPInput->fHeadingCplRi = pLBPInput->fHeadingAngle[CPL_RI];
    pCLPInput->fValidLengthUnCpLf = pLBPInput->fValidLength[UN_CPL_LF];
    pCLPInput->fValidLengthUnCpRi = pLBPInput->fValidLength[UN_CPL_RI];
    pCLPInput->fCrvUnCpLf = pLBPInput->fCurvature[UN_CPL_LF];
    pCLPInput->fCrvUnCpRi = pLBPInput->fCurvature[UN_CPL_RI];
    pCLPInput->fCrvRateUnCpLf = pLBPInput->fCurvatureRate[UN_CPL_LF];
    pCLPInput->fCrvRateUnCpRi = pLBPInput->fCurvatureRate[UN_CPL_RI];
    pCLPInput->uWeatherCond = 1U;
    pCLPInput->uCompState = 1U;
    pCLPInput->bUpDownHillCritical = 1U;

    pCLPInput->fVertSlopeChange = pLBPInput->fVertSlopeChange;
    pCLPInput->fSineWaveDtct = pLBPInput->fSineWaveDtct;
    pCLPInput->fUpDownHillDtct = 0U;
    pCLPInput->fVehVelX = pLBPInput->fVehVelX;

    pCLPInput->uRoadWorks = pLBPInput->uRoadWorks;
    pCLPInput->uLaneChange = pLBPInput->uLaneChange;

    /* ULP */
    pCLPInput->uQualityUlpLf = (UINT8_T)pULPOutput->fQualityLf;
    pCLPInput->bAvailableUlpLf = pULPOutput->bAvailableLf;
    pCLPInput->fValidLengthUlpLf = pULPOutput->fValidLengthLf;
    pCLPInput->fPosY0UlpLf = pULPOutput->fPosY0Lf;
    pCLPInput->fHeadingUlpLf = pULPOutput->fHeadingLf;
    pCLPInput->fCrvUlpLf = pULPOutput->fCrvLf;
    pCLPInput->fCrvRateUlpLf = pULPOutput->fCrvRateLf;
    pCLPInput->fOverallQualityLf = pULPOutput->fOverallQualityLf;
    pCLPInput->uMarkerTypeLf = pULPOutput->uLaneTypeLf;
    pCLPInput->uLaneColorLf = pULPOutput->uColorLf;
    pCLPInput->bLaneBridgeLf = pULPOutput->bBridgePossibleUnCplLf;

    pCLPInput->uQualityUlpRi = (UINT8_T)pULPOutput->fQualityRi;
    pCLPInput->bAvailableUlpRi = pULPOutput->bAvailableRi;
    pCLPInput->fValidLengthUlpRi = pULPOutput->fValidLengthRi;
    pCLPInput->fPosY0UlpRi = pULPOutput->fPosY0Ri;
    pCLPInput->fHeadingUlpRi = pULPOutput->fHeadingRi;
    pCLPInput->fCrvUlpRi = pULPOutput->fCrvRi;
    pCLPInput->fCrvRateUlpRi = pULPOutput->fCrvRateRi;
    pCLPInput->fOverallQualityRi = pULPOutput->fOverallQualityRi;
    pCLPInput->uMarkerTypeRi = pULPOutput->uLaneTypeRi;
    pCLPInput->uLaneColorRi = pULPOutput->uColorRi;
    pCLPInput->bLaneBridgeRi = pULPOutput->bBridgePossibleUnCplRi;

    /* Form LFP */
    pCLPInput->fLaneWidth = pLFPOutput->fLaneWidth;
    pCLPInput->bLatDistDevLf = pLFPOutput->bLatDistDevLf;
    pCLPInput->bLatDistDevRi = pLFPOutput->bLatDistDevRi;
    pCLPInput->bEnaByBridgePossible = pLFPOutput->uBridgePossible;

    /* */
    pCLPParam->fDefaultLaneWidth = 3.0F;
    pCLPParam->uConstructionSiteBitmask = 0U;
    pCLPParam->uConstLaneChangeLf = 1U;
    pCLPParam->uConstLaneChangeRi = 2U;
    pCLPParam->uWeatherCondBitmask = 0U;
    pCLPParam->bUseWeatherCond = 0U;
    pCLPParam->uCompStateEnum = 0U;
    pCLPParam->bUseCompState = 0U;
    pCLPParam->fDistYLimitStepDtct = LBP_fDistYLimitStepDtct;
    pCLPParam->fTurnOnTimeDistYStep = 1.0F;
    pCLPParam->fMaxLaneWidth = 4.8F;
    pCLPParam->fMaxLaneWidthHyst = 0.2F;
    pCLPParam->fMinLaneWidth = 2.5F;
    pCLPParam->fMinLaneWidthHyst = 0.2F;
    pCLPParam->TolRangeNewCorr = 0.25F;
    pCLPParam->TolRangeDistY = 0.1F;

    pCLPParam->fTdNewCorridorOff = 3.0F;
    pCLPParam->fTdAgainNormalOff = 2.0F;
    TUE_CML_MemoryCopy_M((void *)&fMinValidLength_X[0],
                         (void *)&(pCLPParam->fMinValidLength_X),
                         sizeof(fMinValidLength_X));
    TUE_CML_MemoryCopy_M((void *)&fMinValidLength_M[0],
                         (void *)&(pCLPParam->fMinValidLength_M),
                         sizeof(fMinValidLength_M));
    TUE_CML_MemoryCopy_M((void *)&fHystMinValidLength_X[0],
                         (void *)&(pCLPParam->fHystMinValidLength_X),
                         sizeof(fHystMinValidLength_X));
    TUE_CML_MemoryCopy_M((void *)&fHystMinValidLength_M[0],
                         (void *)&(pCLPParam->fHystMinValidLength_M),
                         sizeof(fHystMinValidLength_M));
    pCLPParam->fTdlengthValid = 0.0F;
    pCLPParam->uBtfValidMarkerType = 1078U;
    pCLPParam->uBtfValidLaneColor = 63U;
    pCLPParam->fCoeffHystMaxRange = 0.98F;
    pCLPParam->fMaxRangePosY0 = 6.0F;
    pCLPParam->fMaxRangeHeading = 0.5F;
    pCLPParam->fMaxRangeCrv = 0.02F;
    pCLPParam->fMaxRangeCrvRate = 0.002F;
    pCLPParam->fMaxRangeLength = 300.0F;
    pCLPParam->fMinVelUpDownDtct = 1.0F;
    pCLPParam->fUpDownHillPT1TConst = 10.0F;
    pCLPParam->bUseABDSineWave = 1U;
    pCLPParam->fTurnOffTiSineWave = 3.0F;
    pCLPParam->bUseABDSlopeChange = 1U;
    pCLPParam->fTurnOffTiSlopeChange = 0.5F;
    pCLPParam->bUseUpDownHill = 1U;
    pCLPParam->fUpDownHillRSP = 75.0F;
    pCLPParam->fUpDownHillLSP = 10.0F;
    pCLPParam->fUpDownHillShutOffRSP = 100.0F;
    pCLPParam->fUpDownHillShutOffLSP = 75.0F;
    pCLPParam->fSysCycleTime = pLBPParam->fSysCycleTime;
}

/****************************************************************************************
        @fn           CheckLaneProperity
        @brief        Check the event and lane validity after uncoupled
 processing
        @description  Check lane property:
                                        1.Event information;
                                                1.1 Lane exit detection
                                                1.2 Detect slope changes;
                                                1.3 Lane straight detection;
                                                1.4 Other(Construction
 site��Lane changes);
                                        2.Check the validity of the lane after
 ULP;
        @param[in]    pCLPInput : Input for CLP
        @param[in]    pCLPParam : Parameter for CLP
        @param[out]   pCLPOutput: Output for CLP
        @param[out]   pCLPDebug : Debug(measurement) for CLP
        @return       void
        @startuml
        title INPUT_CheckLaneProperity
        (*)--> 1.EventInformation
           --> 1.1 LaneExitDetection
           --> (*)
        1.EventInformation--> 1.2 DetectSlopeChanges
        1.EventInformation--> 1.3 Lane straight detection
        1.EventInformation--> 1.4 Other(Construction site��Lane changes)
           --> (*)
        (*)--> 2.CheckTheValidityOfTheLaneAfterULP
           --> (*)
        @enduml
 ******************************************************************************************/
void CheckLaneProperity(const sCLPInput_t *pCLPInput,
                        const sCLPParam_t *pCLPParam,
                        sCLPOutput_t *pCLPOutput,
                        sCLPDebug_t *pCLPDebug) {
    // static UINT8_T bLastUpDownHillCritical =
    //     0U; /* Last enable flag for downhill/uphill critical, (0, 0~1, -) */

    // UINT8_T  bUpDownHillCritical = 0U;                /* Enable flag for
    // downhill/uphill critical, (0, 0~1, -) */
    // UINT8_T bTempCLP = 0U;

    /*******************************1.Event
     * information**********************************/
    /*******************************1.1 Lane exit
     * detection******************************/
    ExitRampDetection(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /*******************************1.2 Detect slope
     * changes*****************************/
    DetectSlopeChange(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);
    /* Uphill/downHill critical*/
    // bUpDownHillCritical =
    // TUE_CML_Hysteresis_M(pCLPOutput->fPercUpDownHillDtct,
    //                                              pCLPParam->fUpDownHillShutOffRSP,
    //                                              pCLPParam->fUpDownHillShutOffLSP,
    //                                              bLastUpDownHillCritical);

    /*******************************1.3 Lane straight
     * detection**************************/
    LaneStraightDetection(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /*******************************1.4 Other(Construction site��Lane
     * changes)***********/
    CalculateOtherEvent(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /*******************************1.5 Line Merge Detection***********/
    LineMergeDetection(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /*******************************2.Check the validity of the lane after
     * ULP***********/
    CheckLaneValidity(pCLPInput, pCLPParam, pCLPOutput, pCLPDebug);

    /*******************************Save last
     * value**************************************/
    // bLastUpDownHillCritical = bUpDownHillCritical;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */