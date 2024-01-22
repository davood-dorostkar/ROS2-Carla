/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "LBP_UncoupledLaneProcessing.h"
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static REAL32_T fLastPosY0[LBP_NUM] = {
    0.0F}; /* Last lateral distance at X = 0.0 m, (0, -15~15, m) */
static REAL32_T fLastHeading[LBP_NUM] = {
    0.0F}; /* Last heading angle (Yaw angle) of a lane boundary track, (0,
             -0.7854~0.7854, rad) */
static REAL32_T fLastCrv[LBP_NUM] = {
    0.0F}; /* Curvature of a lane boundary track, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRate[LBP_NUM] = {
    0.0F}; /* Curvature rate of a lane boundary track, (0, -0.001~0.001,
             1/m^2) */
static UINT8_T bLastAvailable[LBP_NUM] = {
    0U}; /* Defines whether a lane boundary track is available or not, (0,
           0~1, -) */
static REAL32_T fLastQualityPenalty[LBP_NUM] = {
    0.0F}; /* The last lane line quality penalty by step detection after
             rate limit, (0~10, - ) */
static REAL32_T fLastOverallQuality[LBP_NUM] = {
    0.0F}; /* Last lane quality percentage
             by all properties, (0,
             0~100��%) */
static UINT8_T bLastDlyEnaLaneVirtualCpl[LBP_NUM] = {
    0U}; /* Last Enable flag for coupled lane virtual after turn on delay,
           (0, 0~1, -) */
static REAL32_T fTimerEnaLaneVirtualCpl =
    0.0F; /* Timer for coupled virtual lane enable flag turn off delay, (0,
             0~60, s) */

static REAL32_T fLastValidLength[LBP_NUM] = {
    0.0F}; /* Last valid length(camera input), (0, 0~300, m) */
static UINT8_T bLastAvailableLMC[LBP_NUM] = {
    0U}; /* Last lane boundary available flag, (0, 0~1, -) */
static REAL32_T fLastOverallQualityLMC[LBP_NUM] = {
    0.0F}; /* Lane quality percentage by all properties, (0, 0~100��%) */
static REAL32_T fLastMaxValidLengthCpmn[LBP_NUM] = {
    0.0F}; /* Max valid length after motion compensation, (0, 0~300, m) */
static REAL32_T fLastMinValidLengthCpmn[LBP_NUM] = {
    0.0F}; /* Min valid length after motion compensation, (0, 0~300, m) */
static UINT8_T bLastEnaMotionCmpn[LBP_NUM] = {
    0U}; /* Lane motion compensation enable flag, (0, 0~1, -) */
static UINT8_T bLastInVldTraj3rd[LBP_NUM] = {0U}; /* Invalid flag of third
                                                   order polynomial
                                                   trajectory fit, (0,
                                                   0~1, -) */

static REAL32_T fLastPosY03rd[LBP_NUM] = {
    0.0F}; /* Last position Y0 of Third-order polynomial fit, (unit, m) */
static REAL32_T fLastHeading3rd[LBP_NUM] = {
    0.0F}; /* Last heading angle of Third-order polynomial fit, (unit, m) */
static REAL32_T fLastCrv3rd[LBP_NUM] = {
    0.0F}; /* Curvature Third-order polynomial fit, (unit, m) */
static REAL32_T fLastCrvRate3rd[LBP_NUM] = {
    0.0F}; /* Curvature rate of Third-order polynomial fit , (unit, m) */
static UINT8_T bLastRawEnaApplySmth[LBP_NUM] = {0U};

static UINT8_T bLastEnaPosY0Smth[LBP_NUM] = {
    0U}; /* Apply smooth data transition disable flag, (unit, -) */
static UINT8_T bLastEnaHeadingSmth[LBP_NUM] = {
    0U}; /* Apply smooth data transition disable flag, (unit, -) */
static UINT8_T bLastEnaCrvSmth[LBP_NUM] = {
    0U}; /* Apply smooth data transition disable flag, (unit, -) */
static UINT8_T bLastEnaCrvRateSmth[LBP_NUM] = {
    0U}; /* Apply smooth data transition disable flag, (unit, -) */
static REAL32_T fLastPosY0Cpmn[LBP_NUM] = {
    0.0F}; /* Last position Y0 of Third-order polynomial fit , (unit, m) */
static REAL32_T fLastHeadingCpmn[LBP_NUM] = {
    0.0F}; /* Last heading angle of Third-order polynomial fit , (unit, m)
            */
static REAL32_T fLastCrvCpmn[LBP_NUM] = {
    0.0F}; /* Curvature Third-order polynomial fit , (unit, m) */
static REAL32_T fLastCrvRateCpmn[LBP_NUM] = {
    0.0F}; /* Curvature rate of Third-order polynomial fit , (unit, m) */
static UINT8_T bLastRstPosY0Smth[LBP_NUM] = {
    0U}; /* Last PosY0 reset flag , (unit, -) */
static UINT8_T bLastRstHeadingSmth[LBP_NUM] = {
    0U}; /* Last Heading reset flag , (unit, -) */
static UINT8_T bLastRstCrvSmth[LBP_NUM] = {
    0U}; /* Last Curve reset flag , (unit, -) */
static UINT8_T bLastRstCrvRateSmth[LBP_NUM] = {
    0U}; /* Last CurveRate reset flag , (unit, -) */
static REAL32_T fTimerbRstLmcByTime[LBP_NUM] = {0.0F};
static UINT8_T bLastRstLmcByTime[LBP_NUM] = {0U};
static REAL32_T fTimerRawNotParallel =
    0.0F; /* Timer for Raw not parallel flag turn on delay, (0, 0~60, s) */
static UINT8_T bLastDlyNotParallelUnCpl =
    0U; /* Last flag after turn on delay that uncoupled lanes are not
           parallel, (0, 0~1, -) */
static REAL32_T fLastDeltaYawCplLf = 0.0F;
static REAL32_T fLastDeltaYawCplRi = 0.0F;
static REAL32_T fLastDeltaXUnCplLf = 0.0F;
static REAL32_T fLastDeltaYUnCplLf = 0.0F;
static REAL32_T fLastDeltaXUnCplRi = 0.0F;
static REAL32_T fLastDeltaYUnCplRi = 0.0F;
static REAL32_T fClothoidUnCplLf[4][8] = {0}; /* Uncoupled left lane clothoid */
static REAL32_T fClothoidUnCplRi[4][8] = {
    0}; /* Uncoupled right lane clothoid */
static REAL32_T fLastPosY0UnCplLf =
    0.0F; /* Last uncoupled left lane lateral distance at X = 0.0 m, (0,
             -15~15, m) */
static REAL32_T fLastHeadingUnCplLf =
    0.0F; /* Last uncoupled left lane heading angle (Yaw angle) of a lane
             boundary track, (0, -0.7854~0.7854, rad) */
static REAL32_T fLastCrvUnCplLf = 0.0F; /* Last uncoupled left lane
                                           curvature of a lane boundary
                                           track, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRateUnCplLf =
    0.0F; /* Last uncoupled left lane curvature rate of a lane boundary
             track, (0, -0.001~0.001, 1/m^2) */
static REAL32_T fLastPosY0UnCplRi =
    0.0F; /* Last uncoupled Right lane lateral distance at X = 0.0 m, (0,
             -15~15, m) */
static REAL32_T fLastHeadingUnCplRi =
    0.0F; /* Last uncoupled Right lane heading angle (Yaw angle) of a lane
             boundary track, (0, -0.7854~0.7854, rad) */
static REAL32_T fLastCrvUnCplRi = 0.0F; /* Last uncoupled Right lane
                                           curvature of a lane boundary
                                           track, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRateUnCplRi =
    0.0F; /* Last uncoupled Right lane curvature rate of a lane boundary
             track, (0, -0.001~0.001, 1/m^2) */
static UINT8_T bLastAvailableUnCplLf =
    0U; /* Uncoupled left lane available flag, (0, 0~1, -) */
static UINT8_T bLastAvailableUnCplRi =
    0U; /* Uncoupled right lane available flag, (0, 0~1, -) */
static REAL32_T fLastPenaltyUnCplLf = 0.0F;
static REAL32_T fLastPenaltyUnCplRi = 0.0F;
static UINT8_T bLastRawStepDtctUnCplLf = 0U;
static UINT8_T bLastRawStepDtctUnCplRi = 0U;
static UINT8_T bLastRawNotParallelUnCpl =
    0U; /* Last Flag that uncoupled lanes are not parallel, (0, 0~1, -) */
static REAL32_T fLastCrvRefUnCplLf = 0.0F;
static REAL32_T fLastCrvRefUnCplRi = 0.0F;
static UINT8_T bLastBridgeUnCplLf =
    0U; /* Last left lane uncoupled lane bridging possible, (0, 0~1, -) */
static UINT8_T bLastBridgeUnCplRi =
    0U; /* Last right lane uncoupled lane bridging possible, (0, 0~1, -) */
static UINT8_T bLastDlySetBridgeUnCplLf = 0U;
static UINT8_T bLastDlySetBridgeUnCplRi = 0U;
static REAL32_T fLastPercExitLf =
    0.0F; /* Exit percent for left side, (0, 0~100, %) */
static REAL32_T fLastPercExitRi =
    0.0F; /* Exit percent for right side, (0, 0~100, %) */
static UINT8_T bLastResetByCrvUnCplLf = 0U;
static UINT8_T bLastResetByCrvUnCplRi = 0U;
static REAL32_T fTimerSetUnCplLf = 0.0F;
static REAL32_T fTimerSetUnCplRi = 0.0F;

static UINT8_T uLastMarkerTypeCpmn[LBP_NUM] = {
    0U}; /* Last marker type after motion compensation  ,(0, 0~300, m) */
static UINT8_T uLastColorCpmn[LBP_NUM] = {
    0U}; /* Last lane color after motion compensation,(0, 0~5, -) */
static UINT8_T uLastOverallQuality[LBP_NUM] = {0U};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void UncoupledLaneReset(void) {
    for (int i = 0; i < 4; i++) {
        fLastPosY0[i] = 0;
        fLastHeading[i] = 0;
        fLastCrv[i] = 0;
        fLastCrvRate[i] = 0;
        bLastAvailable[i] = 0;
        fLastQualityPenalty[i] = 0;
        fLastOverallQuality[i] = 0;
        bLastDlyEnaLaneVirtualCpl[i] = 0;
        fLastValidLength[i] = 0;
        bLastAvailableLMC[i] = 0;
        fLastOverallQualityLMC[i] = 0;
        fLastMaxValidLengthCpmn[i] = 0;
        fLastMinValidLengthCpmn[i] = 0;
        bLastEnaMotionCmpn[i] = 0;
        bLastInVldTraj3rd[i] = 0;
    }
    fTimerEnaLaneVirtualCpl = 0.0F; /* Timer for coupled virtual lane enable
                                       flag turn off delay, (0, 0~60, s) */
    for (int i = 0; i < 4; i++) {
        uLastOverallQuality[i] = 0U;
    }
    for (int i = 0; i < 4; i++) {
        fLastPosY03rd[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        fLastHeading3rd[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        fLastCrv3rd[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        fLastCrvRate3rd[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        bLastRawEnaApplySmth[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        bLastEnaPosY0Smth[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        bLastEnaHeadingSmth[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        bLastEnaCrvSmth[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        bLastEnaCrvRateSmth[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        fLastPosY0Cpmn[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        fLastHeadingCpmn[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        fLastCrvCpmn[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        fLastCrvRateCpmn[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        bLastRstPosY0Smth[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        bLastRstHeadingSmth[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        bLastRstCrvSmth[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
        bLastRstCrvRateSmth[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        fTimerbRstLmcByTime[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        bLastRstLmcByTime[i] = 0;
    }
    fTimerRawNotParallel =
        0.0F; /* Timer for Raw not parallel flag turn on delay, (0, 0~60, s) */
    bLastDlyNotParallelUnCpl =
        0U; /* Last flag after turn on delay that uncoupled lanes are not
               parallel, (0, 0~1, -) */
    fLastDeltaYawCplLf = 0.0F;
    fLastDeltaYawCplRi = 0.0F;
    fLastDeltaXUnCplLf = 0.0F;
    fLastDeltaYUnCplLf = 0.0F;
    fLastDeltaXUnCplRi = 0.0F;
    fLastDeltaYUnCplRi = 0.0F;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            fClothoidUnCplLf[i][j] = 0;
            fClothoidUnCplRi[i][j] = 0;
        }
    }
    // fClothoidUnCplLf[4][8] = {
    //     0}; /* Uncoupled left lane clothoid */
    // fClothoidUnCplRi[4][8] = {
    //     0}; /* Uncoupled right lane clothoid */
    fLastPosY0UnCplLf = 0.0F; /* Last uncoupled left lane lateral distance at X
                                 = 0.0 m, (0, -15~15, m) */
    fLastHeadingUnCplLf =
        0.0F; /* Last uncoupled left lane heading angle (Yaw angle) of a lane
                 boundary track, (0, -0.7854~0.7854, rad) */
    fLastCrvUnCplLf = 0.0F; /* Last uncoupled left lane
                                              curvature of a lane boundary
                                              track, (0, -0.1~0.1, 1/m) */
    fLastCrvRateUnCplLf =
        0.0F; /* Last uncoupled left lane curvature rate of a lane boundary
                 track, (0, -0.001~0.001, 1/m^2) */
    fLastPosY0UnCplRi = 0.0F; /* Last uncoupled Right lane lateral distance at X
                                 = 0.0 m, (0, -15~15, m) */
    fLastHeadingUnCplRi =
        0.0F; /* Last uncoupled Right lane heading angle (Yaw angle) of a lane
                 boundary track, (0, -0.7854~0.7854, rad) */
    fLastCrvUnCplRi = 0.0F; /* Last uncoupled Right lane
                                               curvature of a lane boundary
                                               track, (0, -0.1~0.1, 1/m) */
    fLastCrvRateUnCplRi =
        0.0F; /* Last uncoupled Right lane curvature rate of a lane boundary
                 track, (0, -0.001~0.001, 1/m^2) */
    bLastAvailableUnCplLf =
        0U; /* Uncoupled left lane available flag, (0, 0~1, -) */
    bLastAvailableUnCplRi =
        0U; /* Uncoupled right lane available flag, (0, 0~1, -) */
    fLastPenaltyUnCplLf = 0.0F;
    fLastPenaltyUnCplRi = 0.0F;
    bLastRawStepDtctUnCplLf = 0U;
    bLastRawStepDtctUnCplRi = 0U;
    bLastRawNotParallelUnCpl =
        0U; /* Last Flag that uncoupled lanes are not parallel, (0, 0~1, -) */
    fLastCrvRefUnCplLf = 0.0F;
    fLastCrvRefUnCplRi = 0.0F;
    bLastBridgeUnCplLf =
        0U; /* Last left lane uncoupled lane bridging possible, (0, 0~1, -) */
    bLastBridgeUnCplRi =
        0U; /* Last right lane uncoupled lane bridging possible, (0, 0~1, -) */
    bLastDlySetBridgeUnCplLf = 0U;
    bLastDlySetBridgeUnCplRi = 0U;
    fLastPercExitLf = 0.0F; /* Exit percent for left side, (0, 0~100, %) */
    fLastPercExitRi = 0.0F; /* Exit percent for right side, (0, 0~100, %) */
    bLastResetByCrvUnCplLf = 0U;
    bLastResetByCrvUnCplRi = 0U;
    fTimerSetUnCplLf = 0.0F;
    fTimerSetUnCplRi = 0.0F;

    for (int i = 0; i < 4; i++) {
        uLastMarkerTypeCpmn[i] = 0u;
    }
    for (int i = 0; i < 4; i++) {
        uLastColorCpmn[i] = 0;
    }
}

/****************************************************************************************
        @fn           INPUT_UncoupledLaneProcessing
        @brief        Lane Boundary Processing
        @description  Uncoupled lane pProcessing:
                                        The input signal is processed uniformly
        @param[in]    pLBPInput : Input from external input
        @param[in]    pLBPParam : Input for external parameter
        @param[in]    pCLPOutput : Input from CLP output
        @param[in]    pLFPOutput : Input from CLP output
        @param[out]   pLBPOutput: Output for INPUT_UncoupledLaneProcessing
        @param[out]   pULPParam : Parameter for INPUT_UncoupledLaneProcessing
        @return       void
        @startuml
        title INPUT_UncoupledLaneProcessing
        (*)--> 1.pLBPInput
           --> (*)
        (*)--> 2.pLBPParam
           --> (*)
        (*)--> 3.pCLPOutput
           --> (*)
        (*)--> 4.pLFPOutput
           --> (*)
        @enduml
 ******************************************************************************************/
void INPUT_UncoupledLaneProcessing(const sLBPInput_t *pLBPInput,
                                   const sLBPParam_t *pLBPParam,
                                   const sCLPOutput_t *pCLPOutput,
                                   const sLFPOutput_t *pLFPOutput,
                                   sULPInput_t *pULPInput,
                                   sULPParam_t *pULPParam) {
    pULPInput->bLaneChangeDtct = pLBPInput->uLaneChange;
    pULPInput->bUpDnHillDgrd = pCLPOutput->bUpDownHillDegrade;
    pULPInput->fPosY0CplLf = pLBPInput->fPosY0[CPL_LF];
    pULPInput->fHeadingCplLf = pLBPInput->fHeadingAngle[CPL_LF];
    pULPInput->fCrvCplLf = pLBPInput->fCurvature[CPL_LF];
    pULPInput->fCrvRateCplLf = pLBPInput->fCurvatureRate[CPL_LF];
    pULPInput->fValidLengthCplLf = pLBPInput->fValidLength[CPL_LF];
    pULPInput->bAvailableCplLf = pLBPInput->bAvailable[CPL_LF];
    pULPInput->fStdDevPosY0CplLf = pLBPInput->fStdDevPosY0[CPL_LF];
    pULPInput->fStdDevHeadingCplLf = pLBPInput->fStdDevHeadingAngle[CPL_LF];
    pULPInput->fStdDevCrvCplLf = pLBPInput->fStdDevCurvature[CPL_LF];
    pULPInput->fStdDevCrvRateCplLf = pLBPInput->fStdDevCurvatureRate[CPL_LF];
    pULPInput->fPosY0CplRi = pLBPInput->fPosY0[CPL_RI];
    pULPInput->fHeadingCplRi = pLBPInput->fHeadingAngle[CPL_RI];
    pULPInput->fCrvCplRi = pLBPInput->fCurvature[CPL_RI];
    pULPInput->fCrvRateCplRi = pLBPInput->fCurvatureRate[CPL_RI];
    pULPInput->fValidLengthCplRi = pLBPInput->fValidLength[CPL_RI];
    pULPInput->bAvailableCplRi = pLBPInput->bAvailable[CPL_RI];
    pULPInput->fStdDevPosY0CplRi = pLBPInput->fStdDevPosY0[CPL_RI];
    pULPInput->fStdDevHeadingCplRi = pLBPInput->fStdDevHeadingAngle[CPL_RI];
    pULPInput->fStdDevCrvCplRi = pLBPInput->fStdDevCurvature[CPL_RI];
    pULPInput->fStdDevCrvRateCplRi = pLBPInput->fStdDevCurvatureRate[CPL_RI];
    pULPInput->fQualityCplLf = (REAL32_T)pLBPInput->uQuality[CPL_LF];
    pULPInput->fQualityCplRi = (REAL32_T)pLBPInput->uQuality[CPL_RI];
    pULPInput->fVehVelX = pLBPInput->fVehVelX;
    pULPInput->fVehYawRate = pLBPInput->fVehYawRate;

    pULPInput->uMarkerTypeCplLf = pLBPInput->uMarkerType[CPL_LF];
    pULPInput->uMarkerTypeCplRi = pLBPInput->uMarkerType[CPL_RI];
    pULPInput->uColorCplLf = pLBPInput->uColor[CPL_LF];
    pULPInput->uColorCplRi = pLBPInput->uColor[CPL_RI];
    pULPInput->uPercExitLf = pCLPOutput->bExitUnCplLf;
    pULPInput->uPercExitRi = pCLPOutput->bExitUnCplRi;
    pULPInput->uEventTypeCplLf = pLBPInput->uEventType[CPL_LF];
    pULPInput->uEventTypeCplRi = pLBPInput->uEventType[CPL_RI];
    pULPInput->uEventQualityCplLf = pLBPInput->uEventQuality[CPL_LF];
    pULPInput->uEventQualityCplRi = pLBPInput->uEventQuality[CPL_RI];
    pULPInput->fEventDistanceCplLf = pLBPInput->fEventDistance[CPL_LF];
    pULPInput->fEventDistanceCplRi = pLBPInput->fEventDistance[CPL_RI];

    pULPInput->bDistYStepDtctLf = pCLPOutput->bDistYStepDtctLf;
    pULPInput->bDistYStepDtctRi = pCLPOutput->bDistYStepDtctRi;

    pULPInput->bLineMergeDtcRi = pCLPOutput->bLineMergeDtcRi;
    pULPInput->bLineMergeDtcLf = pCLPOutput->bLineMergeDtcLf;

    /* */
    pULPParam->fSysCycleTime = pLBPParam->fSysCycleTime;
}

/**************************************1.Uncoupled lane
 * processing*******************************/
/****************************************************************************************
        @fn           INPUT_DetermineMeasureQuality
        @brief        Input interface for DetermineMeasureQuality
        @description  Input interface for DetermineMeasureQuality:
                                          1.Input from external;
                                          2.Input from CLP output;
        @param[in]    pULPInput : Input from ULP input
        @param[in]    pULPParam : Parameter from ULP parameter
        @param[out]   pDMQInputCplLf: Input for coupled left lane
        @param[out]   pDMQParamCplLf: Parameter for coupled left lane
        @param[out]   pDMQInputCplRi: Input for coupled right lane
        @param[out]   pDMQParamCplRi: Parameter for coupled right lane
        @return       void
        @startuml
        title INPUT_DetermineMeasureQuality
        (*)--> 1.Input from external
           --> (*)
        (*)--> 2.Input from CLP output
           --> (*)
        @enduml
        *****************************************************************************************/
void INPUT_DetermineMeasureQuality(const sULPInput_t *pULPInput,
                                   const sULPParam_t *pULPParam,
                                   sDMQInput_t *pDMQInputCplLf,
                                   sDMQParam_t *pDMQParamCplLf,
                                   sDMQInput_t *pDMQInputCplRi,
                                   sDMQParam_t *pDMQParamCplRi) {
    /* Input */
    pDMQInputCplLf->fPosY0 = pULPInput->fPosY0CplLf;
    pDMQInputCplLf->fHeading = pULPInput->fHeadingCplLf;
    pDMQInputCplLf->fCrv = pULPInput->fCrvCplLf;
    pDMQInputCplLf->fCrvRate = pULPInput->fCrvRateCplLf;
    pDMQInputCplLf->fValidLength = pULPInput->fValidLengthCplLf;
    pDMQInputCplLf->bAvailable = pULPInput->bAvailableCplLf;
    pDMQInputCplLf->fStdDevPosY0 = pULPInput->fStdDevPosY0CplLf;
    pDMQInputCplLf->fStdDevHeading = pULPInput->fStdDevHeadingCplLf;
    pDMQInputCplLf->fStdDevCrv = pULPInput->fStdDevCrvCplLf;
    pDMQInputCplLf->fStdDevCrvRate = pULPInput->fStdDevCrvRateCplLf;
    pDMQInputCplLf->fQuality = pULPInput->fQualityCplLf;
    pDMQInputCplLf->bLaneChangeDtct = pULPInput->bLaneChangeDtct;
    pDMQInputCplLf->bUpDnHillDgrd = pULPInput->bUpDnHillDgrd;

    pDMQInputCplRi->fPosY0 = pULPInput->fPosY0CplRi;
    pDMQInputCplRi->fHeading = pULPInput->fHeadingCplRi;
    pDMQInputCplRi->fCrv = pULPInput->fCrvCplRi;
    pDMQInputCplRi->fCrvRate = pULPInput->fCrvRateCplRi;
    pDMQInputCplRi->fValidLength = pULPInput->fValidLengthCplRi;
    pDMQInputCplRi->bAvailable = pULPInput->bAvailableCplRi;
    pDMQInputCplRi->fStdDevPosY0 = pULPInput->fStdDevPosY0CplRi;
    pDMQInputCplRi->fStdDevHeading = pULPInput->fStdDevHeadingCplRi;
    pDMQInputCplRi->fStdDevCrv = pULPInput->fStdDevCrvCplRi;
    pDMQInputCplRi->fStdDevCrvRate = pULPInput->fStdDevCrvRateCplRi;
    pDMQInputCplRi->fQuality = pULPInput->fQualityCplRi;
    pDMQInputCplRi->bLaneChangeDtct = pULPInput->bLaneChangeDtct;
    pDMQInputCplRi->bUpDnHillDgrd = pULPInput->bUpDnHillDgrd;

    /* Parameter */
    pDMQParamCplLf->fMinPosY0Step = 0.24F;
    pDMQParamCplLf->fMaxPosY0Step = 0.60F;
    pDMQParamCplLf->fFactorPosY0Step = 40.0F;
    pDMQParamCplLf->fMinHeadingStep = 0.02F;
    pDMQParamCplLf->fMaxHeadingStep = 0.04F;
    pDMQParamCplLf->fFactorHeadingStep = 30.0F;
    // pDMQParamCplLf->fMinCrvStep = 4E-4F;
    // pDMQParamCplLf->fMaxCrvStep = 10E-4F;
    pDMQParamCplLf->fMinCrvStep = 10E-4F;
    pDMQParamCplLf->fMaxCrvStep = 20E-4F;
    pDMQParamCplLf->fFactorCrvStep = 20.0F;
    // pDMQParamCplLf->fMinCrvRateStep = 6E-6F;
    // pDMQParamCplLf->fMaxCrvRateStep = 12E-6F;
    pDMQParamCplLf->fMinCrvRateStep = 20E-6F;
    pDMQParamCplLf->fMaxCrvRateStep = 50E-6F;
    pDMQParamCplLf->fFactorCrvRateStep = 10.0F;
    pDMQParamCplLf->fMaxRawQualityPenalty = 100.0F;
    pDMQParamCplLf->fMaxQualityPenaltyRate = 1E2F;
    pDMQParamCplLf->fMinQualityPenaltyRate = -33.0F;
    pDMQParamCplLf->fValidLengthQuality = 30.0F;
    pDMQParamCplLf->fTdEnaLaneVirtualCpl = 0.06F;
    pDMQParamCplLf->fDefQlyVirtualLane = 31.0F;
    pDMQParamCplLf->fSysCycleTime = pULPParam->fSysCycleTime;

    ////*pDMQParamCplRi = *pDMQParamCplLf;
    TUE_CML_MemoryCopy_M((void *)pDMQParamCplLf, (void *)pDMQParamCplRi,
                         sizeof(sDMQParam_t));
}

void DMQ_OverallStandardDevQuality(const sDMQInput_t *pDMQInput,
                                   REAL32_T *fMeasureQuality) {
    REAL32_T fMeasureTemp[4] = {0}; /* Temp variable, (0, 0~100, %) */
    REAL32_T fTempDMQ = 0.0F;       /* Temp variable, (unit, -) */

    /****************************************1.2 Standard deviation
     * quality*****************************************/
    /* According to the ratio of standard deviation and reference standard
       deviation of lane measurement value,
       the quality of lane line is obtained */
    /*    StdDev/RefDev <  0.5  Quality = 25;
                  StdDev/RefDev <  0.7  Quality = 20;
                  StdDev/RefDev <  0.9  Quality = 10;
                  StdDev/RefDev >= 0.9  Quality = 0; */
    fMeasureTemp[0] = pDMQInput->fStdDevPosY0 / 0.07F;
    fMeasureTemp[1] = pDMQInput->fStdDevHeading / 0.009F;
    fMeasureTemp[2] = pDMQInput->fStdDevCrv / 7e-4F;
    fMeasureTemp[3] = pDMQInput->fStdDevCrvRate / 2E-6F;
    *fMeasureQuality = 0.0F;
    for (UINT8_T ii = 0U; ii < 4U; ii++) {
        if (fMeasureTemp[ii] < 0.5F) {
            fTempDMQ = 25.0F;
        } else if (fMeasureTemp[ii] < 0.7F) {
            fTempDMQ = 20.0F;
        } else if (fMeasureTemp[ii] < 0.9F) {
            fTempDMQ = 10.0F;
        } else {
            fTempDMQ = 0.0F;
        }
        *fMeasureQuality = *fMeasureQuality + fTempDMQ;
    }
}

/********************************1.1 Determine measure
quality********************************
        @fn            DetermineMeasureQuality
        @brief         Determine Quality Lane by raw any boundary(camera input)
        @description   Determine measurement quality lane:
                                          1.Lane quality calculation:
                                                  1.1 Penalty for step
detections;
                                                  1.2 Standard deviation
quality;
                                                  1.3 Lookahead distance
quality;
                                                  1.4 Update quality for virtual
lane.
                                          2.Curvature quality calculation(Move
to ELG):
                                                  2.1 Standard deviation
quality;
                                                  2.2 Lookahead distance
quality;
                                                  2.3 Update quality for virtual
lane.
        @param[in]     pDMQInput  : Input for DetermineMeasureQuality
        @param[in]     pDMQParam  : Parameter for DetermineMeasureQuality
        @param[out]    pDMQOutput : Output for DetermineMeasureQuality
        @param[out]    pDMQDebug  : Debug for DetermineMeasureQuality
        @return        void
        @startuml
        title DetermineMeasureQuality
        (*)--> 1.1 Penalty for step detections
           --> CalculateLaneQuality
        (*)--> 1.2 Standard deviation quality
           --> CalculateLaneQuality
        (*)--> 1.3 Lookahead distance quality
           --> CalculateLaneQuality
        (*)--> Update quality for virtual lane
           --> CalculateLaneQuality
           -->1.4 Update quality for virtual lane
           -->(*)
        (*)--> 2.1 Standard deviation quality
           --> CalculateCrvQuality
        (*)--> 2.2 Lookahead distance quality
           --> CalculateCrvQuality
           -->2.3 Update quality for virtual lane
           -->(*)
        @enduml
**************************************************************************************************/
void DetermineMeasureQuality(const UINT8_T LaneIndex,
                             const sDMQInput_t *pDMQInput,
                             const sDMQParam_t *pDMQParam,
                             sDMQOutput_t *pDMQOutput,
                             sDMQDebug_t *pDMQDebug) {
    REAL32_T fTempDMQ = 0.0F; /* Temp variable, (unit, -) */
    UINT8_T bTempDMQ = 0U;    /* Temp variable, (unit, -) */
    REAL32_T fAbsStepPosY0 = 0.0F;
    REAL32_T fAbsStepHeading = 0.0F;
    REAL32_T fAbsStepCrv = 0.0F;
    REAL32_T fAbsStepCrvRate = 0.0F;
    REAL32_T fQualityPenaltyByPosY0 =
        0.0F; /* The quality penalty by lateral distance step, (0, 0~100, %) */
    REAL32_T fQualityPenaltyByHeading =
        0.0F; /* The quality penalty by heading angle step, (0, 0~100, %) */
    REAL32_T fQualityPenaltyByCrv =
        0.0F; /* The quality penalty by curvature step, (0, 0~100, %) */
    REAL32_T fQualityPenaltyByCrvRate =
        0.0F; /* The quality penalty by curvature rate step, (0, 0~100, %) */
    REAL32_T fRawQualityPenalty = 0.0F; /* Raw quality penalty, (0, 0~100, %) */
    REAL32_T fQualityPenalty = 0.0F;
    ; /* Quality penalty, (0, 0~100, %) */

    REAL32_T fMeasureTemp[4] = {0}; /* Temp variable, (0, 0~100, %) */
    REAL32_T fMeasureQuality =
        0.0F; /* Lane quality by measure standard deviation, (0, 0~100, %) */
    REAL32_T fLengthQuality =
        0.0F; /* Lane quality by valid length, (0, 0~100, %) */
    REAL32_T fRawOverallQuality =
        0.0F; /* Lane quality percentage by all properties, (0, 0~100��%) */
    REAL32_T fMeasureCrvQuality =
        0.0F; /* Curvature quality by measure standard deviation, (0,0~100, %)
               */
    REAL32_T fLengthCrvQuality =
        0.0F; /* Curvature quality by valid length, (0, 0~100, %) */
    UINT8_T bDlyEnaLaneVirtualCpl = 0U; /* Enable flag for coupled lane virtual
                                           after turn on delay, (0, 0~1, -) */

    REAL32_T fRawCrvQuality = 0.0F; /* Raw curvature quality percentage by all
                                       properties, (0, 0~100��%) */

    /******************************1.Lane quality
     * calculation******************************************/
    /******************************1.1 Penalty for step
     * detections*************************************/
    /* (step - min)/(max - min) *factor */
    /* Step detection by PosY0 */
    fAbsStepPosY0 = pDMQInput->fPosY0 - fLastPosY0[LaneIndex];
    fAbsStepPosY0 = TUE_CML_Abs_M(fAbsStepPosY0);
    fTempDMQ = (fAbsStepPosY0 - pDMQParam->fMinPosY0Step) /
               (pDMQParam->fMaxPosY0Step - pDMQParam->fMinPosY0Step);
    fQualityPenaltyByPosY0 =
        TUE_CML_Limit_M(fTempDMQ, 0.0F, 1.0F) * pDMQParam->fFactorPosY0Step;

    /* Step detection by Heading Angle */
    fAbsStepHeading = pDMQInput->fHeading - fLastHeading[LaneIndex];
    fAbsStepHeading = TUE_CML_Abs_M(fAbsStepHeading);
    fTempDMQ = (fAbsStepHeading - pDMQParam->fMinHeadingStep) /
               (pDMQParam->fMaxHeadingStep - pDMQParam->fMinHeadingStep);
    fQualityPenaltyByHeading =
        TUE_CML_Limit_M(fTempDMQ, 0.0F, 1.0F) * pDMQParam->fFactorHeadingStep;

    /* Step detection by Curvature */
    fAbsStepCrv = pDMQInput->fCrv - fLastCrv[LaneIndex];
    fAbsStepCrv = TUE_CML_Abs_M(fAbsStepCrv);
    fTempDMQ = (fAbsStepCrv - pDMQParam->fMinCrvStep) /
               (pDMQParam->fMaxCrvStep - pDMQParam->fMinCrvStep);
    fQualityPenaltyByCrv =
        TUE_CML_Limit_M(fTempDMQ, 0.0F, 1.0F) * pDMQParam->fFactorCrvStep;

    /* Step detection by Curvature */
    fAbsStepCrvRate = pDMQInput->fCrvRate - fLastCrvRate[LaneIndex];
    fAbsStepCrvRate = TUE_CML_Abs_M(fAbsStepCrvRate);
    fTempDMQ = (fAbsStepCrvRate - pDMQParam->fMinCrvRateStep) /
               (pDMQParam->fMaxCrvRateStep - pDMQParam->fMinCrvRateStep);
    fQualityPenaltyByCrvRate =
        TUE_CML_Limit_M(fTempDMQ, 0.0F, 1.0F) * pDMQParam->fFactorCrvRateStep;

    /* Quality penalty coefficient by step detection */
    if ((pDMQInput->bAvailable == bLastAvailable[LaneIndex]) &&
        (pDMQInput->bLaneChangeDtct == 0U)) {
        fRawQualityPenalty = fQualityPenaltyByPosY0 + fQualityPenaltyByHeading +
                             fQualityPenaltyByCrv + fQualityPenaltyByCrvRate;
        if (fRawQualityPenalty > 0.0F) {
            fRawQualityPenalty =
                fRawQualityPenalty + fLastQualityPenalty[LaneIndex];
        } else {
            fRawQualityPenalty = 0.0F;
        }
    } else {
        fRawQualityPenalty = 0.0F;
    }
    fRawQualityPenalty =
        TUE_CML_Min_M(fRawQualityPenalty, pDMQParam->fMaxRawQualityPenalty);
    fQualityPenalty = TUE_CML_GradLimit_M(
        fRawQualityPenalty, pDMQParam->fMaxQualityPenaltyRate,
        pDMQParam->fMinQualityPenaltyRate, pDMQParam->fSysCycleTime,
        fLastQualityPenalty[LaneIndex]);
    fQualityPenalty = TUE_CML_Min_M(fQualityPenalty, 100.0F);

    /****************************************1.2 Standard deviation
     * quality*****************************************/
    DMQ_OverallStandardDevQuality(pDMQInput, &fMeasureQuality);

    /****************************************1.3 Lookahead distance
     * quality*****************************************/
    if (pDMQInput->fValidLength > pDMQParam->fValidLengthQuality) {
        fLengthQuality = 100.0F;
    } else {
        fLengthQuality = TUE_CML_DivProtection_M(
            pDMQInput->fValidLength, pDMQParam->fValidLengthQuality, 0.0F);
        fLengthQuality = fLengthQuality * 100.0F;
    }

    /****************************************1.4 Update quality for virtual
     * lane************************************/
    /* Calculate raw lane overall quality */
    // bEnableForQuality = TUE_CML_Hysteresis_M(pDMQInput->uQuality, 20.0F,
    // 10.0F, bLastEnableForQuality);
    if (pDMQInput->bAvailable == 1U) {
        fRawOverallQuality =
            (fMeasureQuality + fLengthQuality) / 2.0F - fQualityPenalty;
        fRawOverallQuality = TUE_CML_Max_M(fRawOverallQuality, 0.0F);
    } else {
        fRawOverallQuality = 0.0F;
    }

    /* Update quality for virtual lane */
    if (((UINT8_T)pDMQInput->fQuality) == 1U) {
        pDMQOutput->bEnaLaneVirtualCpl = 1U;
    } else {
        pDMQOutput->bEnaLaneVirtualCpl = 0U;
    }

    bDlyEnaLaneVirtualCpl = TUE_CML_TurnOnDelay_M(
        pDMQOutput->bEnaLaneVirtualCpl, pDMQParam->fTdEnaLaneVirtualCpl,
        pDMQParam->fSysCycleTime, &fTimerEnaLaneVirtualCpl,
        bLastDlyEnaLaneVirtualCpl[LaneIndex]);
    if (bDlyEnaLaneVirtualCpl == 0U) {
        pDMQOutput->fOverallQuality = fRawOverallQuality;
    } else if (pDMQInput->bUpDnHillDgrd == 0U) {
        pDMQOutput->fOverallQuality = pDMQParam->fDefQlyVirtualLane;
    } else {
        pDMQOutput->fOverallQuality = 0.0F;
    }
    /* (100.0F, -1E4F) */
    pDMQOutput->fOverallQuality = TUE_CML_GradLimit_M(
        pDMQOutput->fOverallQuality, 100.0F, -500.0F, pDMQParam->fSysCycleTime,
        fLastOverallQuality[LaneIndex]);
    if (LaneIndex == CPL_LF) {
        if (pDMQInput->bLaneChangeDtct == 2) {
            pDMQOutput->fOverallQuality = uLastOverallQuality[CPL_RI];
        }
    } else if (LaneIndex == CPL_RI) {
        if (pDMQInput->bLaneChangeDtct == 1) {
            pDMQOutput->fOverallQuality = uLastOverallQuality[CPL_LF];
        }
    }

    pDMQOutput->fOverallQuality =
        TUE_CML_Limit_M(pDMQOutput->fOverallQuality, 0.0F, 100.0F);
    uLastOverallQuality[LaneIndex] = pDMQOutput->fOverallQuality;

    /******************************2.Curvature quality
     * calculation*************************************/
    /******************************2.1 Standard deviation
     * quality**************************************/
    /* According to the ratio of standard deviation and reference standard
       deviation of lane measurement value,
       the quality of lane line is obtained */
    /*    StdDev/RefDev <  0.5  Quality = 25;
                  StdDev/RefDev <  0.7  Quality = 20;
                  StdDev/RefDev <  0.9  Quality = 10;
                  StdDev/RefDev >= 0.9  Quality = 0; */
    fMeasureTemp[0] = pDMQInput->fStdDevCrv / 8E-4F;
    fMeasureTemp[1] = pDMQInput->fStdDevCrvRate / 4E-6F;
    fMeasureCrvQuality = 0.0F;
    for (UINT8_T ii = 0U; ii < 2U; ii++) {
        if (fMeasureTemp[ii] < 0.5F) {
            fTempDMQ = 100.0F;
        } else if (fMeasureTemp[ii] < 0.7F) {
            fTempDMQ = 80.0F;
        } else if (fMeasureTemp[ii] < 0.9F) {
            fTempDMQ = 40.0F;
        } else {
            fTempDMQ = 0.0F;
        }
        fMeasureCrvQuality = fMeasureCrvQuality + fTempDMQ;
    }
    fMeasureCrvQuality = fMeasureCrvQuality / 2.0F;

    /****************************************2.2 Lookahead distance
     * quality*****************************************/
    if (pDMQInput->fValidLength > 40.0F) {
        fLengthCrvQuality = 100.0F;
    } else {
        fLengthCrvQuality = pDMQInput->fValidLength / 40.0F * 100.0F;
    }

    /*****************************************2.3 Update quality for virtual
     * lane***********************************/
    if (pDMQInput->bAvailable == 1U) {
        fRawCrvQuality = (fMeasureCrvQuality + fLengthCrvQuality) / 2.0F;
    } else {
        fRawCrvQuality = 0.0F;
    }

    /* pDMQInput->uQuality == 1U */
    if (pDMQOutput->bEnaLaneVirtualCpl == 0U) {
        pDMQOutput->fCrvQuality = fRawCrvQuality;
    } else {
        pDMQOutput->fCrvQuality = 1.0F;
    }

    /****************************************Output and debug
     * *****************************************/
    pDMQDebug->fQualityPenaltyByPosY0 = fQualityPenaltyByPosY0;
    pDMQDebug->fQualityPenaltyByHeading = fQualityPenaltyByHeading;
    pDMQDebug->fQualityPenaltyByCrv = fQualityPenaltyByCrv;
    pDMQDebug->fQualityPenaltyByCrvRate = fQualityPenaltyByCrvRate;
    pDMQDebug->fRawQualityPenalty = fRawQualityPenalty;
    pDMQDebug->fQualityPenalty = fQualityPenalty;
    pDMQDebug->fLengthQuality = fLengthQuality;
    pDMQDebug->fMeasureQuality = fMeasureQuality;
    pDMQDebug->fRawOverallQuality = fRawOverallQuality;
    pDMQDebug->bDlyEnaLaneVirtualCpl = bDlyEnaLaneVirtualCpl;
    pDMQDebug->fAbsStepPosY0 = fAbsStepPosY0;
    pDMQDebug->fAbsStepHeading = fAbsStepHeading;
    pDMQDebug->fAbsStepCrv = fAbsStepCrv;
    pDMQDebug->fAbsStepCrvRate = fAbsStepCrvRate;
    pDMQDebug->fLastPosY0 = fLastPosY0[LaneIndex];
    pDMQDebug->fLastHeading = fLastHeading[LaneIndex];
    pDMQDebug->fLastCrv = fLastCrv[LaneIndex];
    pDMQDebug->fLastCrvRate = fLastCrvRate[LaneIndex];
    pDMQDebug->bLastAvailable = bLastAvailable[LaneIndex];
    pDMQDebug->fLastQualityPenalty = fLastQualityPenalty[LaneIndex];
    pDMQDebug->fLastOverallQuality = fLastOverallQuality[LaneIndex];
    pDMQDebug->bLastDlyEnaLaneVirtualCpl = bLastDlyEnaLaneVirtualCpl[LaneIndex];
    pDMQDebug->fMeasureCrvQuality = fMeasureCrvQuality;
    pDMQDebug->fLengthCrvQuality = fLengthCrvQuality;
    pDMQDebug->fTimerEnaLaneVirtualCpl = fTimerEnaLaneVirtualCpl;
    pDMQDebug->fRawCrvQuality = fRawCrvQuality;

    /****************************************Save last value
     * *****************************************/
    fLastPosY0[LaneIndex] = pDMQInput->fPosY0;
    fLastHeading[LaneIndex] = pDMQInput->fHeading;
    fLastCrv[LaneIndex] = pDMQInput->fCrv;
    fLastCrvRate[LaneIndex] = pDMQInput->fCrvRate;
    bLastAvailable[LaneIndex] = pDMQInput->bAvailable;
    fLastQualityPenalty[LaneIndex] = fQualityPenalty;
    fLastOverallQuality[LaneIndex] = pDMQOutput->fOverallQuality;
    bLastDlyEnaLaneVirtualCpl[LaneIndex] = bDlyEnaLaneVirtualCpl;
}

/****************************************************************************************
        @fn           INPUT_LaneMotionCompensation
        @brief        Input interface for DetermineMeasureQuality
        @description  Input interface for DetermineMeasureQuality:
                                          1.Input from external;
                                          2.Input from DMQ output;
        @param[in]    pULPInput : Input from ULP input
        @param[in]    pULPParam : Parameter from ULP parameter
        @param[in]    pDMQOutputCplLf : Input from coupled left lane measure
   quanlity
        @param[in]    pDMQOutputCplRi : Input from coupled right lane measure
   quanlity
        @param[out]   pLMCInputCplLf: Input for coupled left lane
        @param[out]   pLMCParamCplLf: Parameter for coupled left lane
        @param[out]   pLMCInputCplRi: Input for coupled right lane
        @param[out]   pLMCParamCplRi: Parameter for coupled right lane
        @return       void
        @startuml
        title INPUT_LaneMotionCompensation
        (*)--> 1.Input from external
           --> (*)
        (*)--> 2.Input from DMQ output
           --> (*)
        @enduml
        *****************************************************************************************/
void INPUT_LaneMotionCompensation(const sULPInput_t *pULPInput,
                                  const sULPParam_t *pULPParam,
                                  const sDMQOutput_t *pDMQOutputCplLf,
                                  const sDMQOutput_t *pDMQOutputCplRi,
                                  sLMCInput_t *pLMCInputCplLf,
                                  sLMCParam_t *pLMCParamCplLf,
                                  sLMCInput_t *pLMCInputCplRi,
                                  sLMCParam_t *pLMCParamCplRi) {
    /* Input */
    pLMCInputCplLf->fPosY0 = pULPInput->fPosY0CplLf;
    pLMCInputCplLf->fHeading = pULPInput->fHeadingCplLf;
    pLMCInputCplLf->fCrv = pULPInput->fCrvCplLf;
    pLMCInputCplLf->fCrvRate = pULPInput->fCrvRateCplLf;
    pLMCInputCplLf->fValidLength = pULPInput->fValidLengthCplLf;
    pLMCInputCplLf->bAvailable = pULPInput->bAvailableCplLf;
    pLMCInputCplLf->fQuality = pULPInput->fQualityCplLf;
    pLMCInputCplLf->fVehVelX = pULPInput->fVehVelX;
    pLMCInputCplLf->fVehYawRate = pULPInput->fVehYawRate;
    pLMCInputCplLf->uMarkerType = pULPInput->uMarkerTypeCplLf;
    pLMCInputCplLf->uColor = pULPInput->uColorCplLf;
    pLMCInputCplLf->fOverallQuality = pDMQOutputCplLf->fOverallQuality;
    pLMCInputCplLf->bLaneChangeDtct = pULPInput->bLaneChangeDtct;

    pLMCInputCplRi->fPosY0 = pULPInput->fPosY0CplRi;
    pLMCInputCplRi->fHeading = pULPInput->fHeadingCplRi;
    pLMCInputCplRi->fCrv = pULPInput->fCrvCplRi;
    pLMCInputCplRi->fCrvRate = pULPInput->fCrvRateCplRi;
    pLMCInputCplRi->fValidLength = pULPInput->fValidLengthCplRi;
    pLMCInputCplRi->bAvailable = pULPInput->bAvailableCplRi;
    pLMCInputCplRi->fQuality = pULPInput->fQualityCplRi;
    pLMCInputCplRi->fVehVelX = pULPInput->fVehVelX;
    pLMCInputCplRi->fVehYawRate = pULPInput->fVehYawRate;
    pLMCInputCplRi->uMarkerType = pULPInput->uMarkerTypeCplRi;
    pLMCInputCplRi->uColor = pULPInput->uColorCplRi;
    pLMCInputCplRi->fOverallQuality = pDMQOutputCplRi->fOverallQuality;
    pLMCInputCplRi->bLaneChangeDtct = pULPInput->bLaneChangeDtct;

    /* Parameter */
    pLMCParamCplLf->bUseMotionComp = 1U;
    pLMCParamCplLf->fThdLengthSetLmc = 25.0F;
    pLMCParamCplLf->fThdQualitySetLmc = 70.0F;
    pLMCParamCplLf->fThdQlyChngSetLmc = 25.0F;
    pLMCParamCplLf->fThdQualityRstLmc = 75.0F;
    pLMCParamCplLf->fThdCrvRstLmc = 2e-3f;
    pLMCParamCplLf->fThdLengthRstLmc = 15.0F;

    pLMCParamCplLf->fSysCycleTime = pULPParam->fSysCycleTime;

    TUE_CML_MemoryCopy_M((void *)pLMCParamCplLf, (void *)pLMCParamCplRi,
                         sizeof(sLMCParam_t));
}

void LMC_DetrmCompensationSet(REAL32_T fLastValidLength,
                              REAL32_T fLastOverallQualityIn,
                              UINT8_T bLastAvailableIn,
                              sLMCInput_t const *pLMCInput,
                              sLMCParam_t const *pLMCParam,
                              UINT8_T *bSetLmcEnable,
                              sLMCDebug_t *pLMCDebug) {
    UINT8_T
    bSetLmcByLength = 0U; /* Set flag for motion compensation enable by
                             valid length, (0, 0~1, -) */
    UINT8_T bSetLmcByOverallQuality =
        0U; /* Set flag for motion compensation enable by overall quality, (0,
               0~1, -) */
    UINT8_T
    bSetLmcByAvlChng = 0U; /* Set flag for motion compensation enable by
                              lane available change, (0, 0~1, -) */
    UINT8_T bSetLmcByQlyChng =
        0U; /* Set flag for motion compensation enable by lane overall quality
               change, (0, 0~1, -) */

    if (fLastValidLength > pLMCParam->fThdLengthSetLmc) {
        bSetLmcByLength = 1U;
    } else {
        bSetLmcByLength = 0U;
    }

    if (fLastOverallQualityIn > pLMCParam->fThdQualitySetLmc) {
        bSetLmcByOverallQuality = 1U;
    } else {
        bSetLmcByOverallQuality = 0U;
    }

    if ((pLMCInput->bAvailable == 0U) && (bLastAvailableIn == 1U)) {
        bSetLmcByAvlChng = 1U;
    } else {
        bSetLmcByAvlChng = 0U;
    }

    if ((fLastOverallQualityIn - pLMCInput->fOverallQuality) >
        pLMCParam->fThdQlyChngSetLmc) {
        bSetLmcByQlyChng = 1U;
    } else {
        bSetLmcByQlyChng = 0U;
    }

    if ((pLMCParam->bUseMotionComp == 1U) && (bSetLmcByLength == 1U) &&
        (bSetLmcByOverallQuality == 1U) &&
        ((bSetLmcByAvlChng == 1U) || (bSetLmcByQlyChng == 1U))) {
        *bSetLmcEnable = 1U;
    } else {
        *bSetLmcEnable = 0U;
    }
    pLMCDebug->bSetLmcByLength = bSetLmcByLength;
    pLMCDebug->bSetLmcByOverallQuality = bSetLmcByOverallQuality;
    pLMCDebug->bSetLmcByAvlChng = bSetLmcByAvlChng;
    pLMCDebug->bSetLmcByQlyChng = bSetLmcByQlyChng;
}

void LMC_DetrmCompensationReset(REAL32_T fLastMaxValidLengthCpmn,
                                UINT8_T bLastInVldTraj3rd,
                                UINT8_T bRstLmcByTime,
                                sLMCInput_t const *pLMCInput,
                                sLMCParam_t const *pLMCParam,
                                UINT8_T *bRstLmcEnable,
                                sLMCDebug_t *pLMCDebug) {
    UINT8_T bRstLmcByCrv = 0U; /* Reset flag for motion compensation enable by
  curvature, (0, 0~1, -) */
    UINT8_T bRstLmcByAvlQly = 0U; /* Reset flag for motion compensation enable
                                     by lane available and high quality, (0,
                                     0~1, -) */
    UINT8_T bRstLmcByMaxLength =
        0U; /* Reset flag for motion compensation enable by max valid length,
               (0, 0~1, -) */
    UINT8_T
    bRstLmcByPolyFit = 0U; /* Reset flag for motion compensation enable by
                              polynomial fitting, (0, 0~1, -) */
    UINT8_T bRstLmcByLaneChng = 0U; /* Reset flag for motion compensation enable
                                       by lane change, (0, 0~1, -) */
    UINT8_T bRstLmcByLowQly = 0U;   /* Reset flag for motion compensation enable
                                       by poor quality, (0, 0~1, -) */
    if (TUE_CML_Abs_M(pLMCInput->fCrv) > pLMCParam->fThdCrvRstLmc) {
        bRstLmcByCrv = 1U;
    } else {
        bRstLmcByCrv = 0U;
    }

    if ((pLMCInput->bAvailable == 1U) &&
        (pLMCInput->fOverallQuality > pLMCParam->fThdQualityRstLmc)) {
        bRstLmcByAvlQly = 1U;
    } else {
        bRstLmcByAvlQly = 0U;
    }

    if (fLastMaxValidLengthCpmn < pLMCParam->fThdLengthRstLmc) {
        bRstLmcByMaxLength = 1U;
    } else {
        bRstLmcByMaxLength = 0U;
    }

    if (bLastInVldTraj3rd == 1U) {
        bRstLmcByPolyFit = 1U;
    } else {
        bRstLmcByPolyFit = 0U;
    }

    if (pLMCInput->bLaneChangeDtct == 1U) {
        bRstLmcByLaneChng = 1U;
    } else {
        bRstLmcByLaneChng = 0U;
    }

    if (((UINT8_T)pLMCInput->fQuality) == 1U) {
        bRstLmcByLowQly = 1U;
    } else {
        bRstLmcByLowQly = 0U;
    }

    if ((bRstLmcByCrv == 1U) || (bRstLmcByAvlQly == 1U) ||
        (bRstLmcByMaxLength == 1U) || (bRstLmcByPolyFit == 1U) ||
        (bRstLmcByLaneChng == 1U) || (bRstLmcByLowQly == 1U) || bRstLmcByTime)

    {
        *bRstLmcEnable = 1U;
    } else {
        *bRstLmcEnable = 0U;
    }

    pLMCDebug->bRstLmcByAvlQly = bRstLmcByAvlQly;
    pLMCDebug->bRstLmcByMaxLength = bRstLmcByMaxLength;
    pLMCDebug->bRstLmcByPolyFit = bRstLmcByPolyFit;
    pLMCDebug->bRstLmcByLaneChng = bRstLmcByLaneChng;
    pLMCDebug->bRstLmcByLowQly = bRstLmcByLowQly;
}

void LMC_EnableFlagSmooth(UINT8_T bLastEnaMotionCmpn,
                          UINT8_T bEnaMotionCmpn,
                          REAL32_T fPosY03rd,
                          REAL32_T fLastPosY03rd,
                          REAL32_T fHeading3rd,
                          REAL32_T fLastHeading3rd,
                          UINT8_T bLastRawEnaApplySmth,
                          UINT8_T *bRawEnaApplySmth,
                          UINT8_T *bEnaApplySmth,
                          sLMCDebug_t *pLMCDebug) {
    UINT8_T
    bEnaApplySmthByCmpn = 0U; /* Enable flag for apply smooth by motion
                                 compensation flag,(0, 0~1, -) */
    UINT8_T bEnaApplySmthByPosY =
        0U; /* Enable flag for apply smooth by position Y change,(0, 0~1, -) */
    UINT8_T bEnaApplySmthByHeading = 0U; /* Enable flag for apply smooth by
                                            heading angle change,(0, 0~1, -) */
    if ((bLastEnaMotionCmpn == 1U) && (bEnaMotionCmpn == 0U)) {
        bEnaApplySmthByCmpn = 1U;
    } else {
        bEnaApplySmthByCmpn = 0U;
    }

    if (TUE_CML_Abs_M(fPosY03rd - fLastPosY03rd) < 0.05f) {
        bEnaApplySmthByPosY = 1U;
    } else {
        bEnaApplySmthByPosY = 0U;
    }

    if (TUE_CML_Abs_M(fHeading3rd - fLastHeading3rd) < 0.005f) {
        bEnaApplySmthByHeading = 1U;
    } else {
        bEnaApplySmthByHeading = 0U;
    }

    if ((bEnaApplySmthByCmpn == 1U) && (bEnaApplySmthByPosY == 1U) &&
        (bEnaApplySmthByHeading == 1U)) {
        *bRawEnaApplySmth = 1U;
    } else {
        *bRawEnaApplySmth = 0U;
    }

    if ((*bRawEnaApplySmth == 0U) && (bLastRawEnaApplySmth == 1U)) {
        *bEnaApplySmth = 1U;
    } else {
        *bEnaApplySmth = 0U;
    }
    pLMCDebug->bEnaApplySmthByCmpn = bEnaApplySmthByCmpn;
    pLMCDebug->bEnaApplySmthByPosY = bEnaApplySmthByPosY;
    pLMCDebug->bEnaApplySmthByHeading = bEnaApplySmthByHeading;
}

UINT8_T LMC_Comparation(REAL32_T input, REAL32_T threshold) {
    if (input <= threshold) {
        return 1U;
    } else {
        return 0U;
    }
}
/****************************************************************************************
        @fn            LaneMotionCompensation
        @brief         Motion compensation for any boundary(camera input)
        @description   Motion compensation for any boundary:
                                        1.Motion compensation enable flag;
                                        2.Calculation of sampling points;
                                        3.Motion compensation calculation;
                                        4.Apply smooth data transition;
                                        5.Rearrange lane boundary
        @param[in]     pLMCInput  : input for LaneMotionCompensation
        @param[in]     pLMCParam  : parameter for LaneMotionCompensation
        @param[out]    pLMCOutput : output for LaneMotionCompensation
        @param[out]    pLMCDebug  : debug for LaneMotionCompensation
        @return        void
        @startuml
        title INPUT_LaneMotionCompensation
        (*)--> 1.MotionCompensationEnableFlag
           --> SelectSamplePoint
        1.MotionCompensationEnableFlag-->3.MotionCompensationCalculation
        1.MotionCompensationEnableFlag-->4.ApplySmoothDataTransition
        1.MotionCompensationEnableFlag-->5.RearrangeLaneBoundary
        (*)--> 2.CalculationOfSamplingPoints
           --> 2.1 SamplePointsFromCameraInput
           --> SelectSamplePoint
        2.CalculationOfSamplingPoints--> 2.2
SamplePointsFromLastLaneMotionCompensation
           --> SelectSamplePoint
           --> 3.MotionCompensationCalculation
           --> 4.ApplySmoothDataTransition
           --> 5.RearrangeLaneBoundary
           --> (*)
        @enduml
******************************************************************************************/
void LaneMotionCompensation(const UINT8_T LaneIndex,
                            sLMCInput_t const *pLMCInput,
                            sLMCParam_t const *pLMCParam,
                            sLMCOutput_t *pLMCOutput,
                            sLMCDebug_t *pLMCDebug) {
    REAL32_T fPosY0Cpmn = 0.0F; /* Position Y0 of Third-order polynomial fit for
                                   lane motion compensation, (unit, m) */
    REAL32_T fHeadingCpmn = 0.0F; /* Heading angle of Third-order polynomial fit
                                     for lane motion compensation, (unit, m) */
    REAL32_T fCrvCpmn = 0.0F; /*  Curvature Third-order polynomial fit for lane
                                 motion compensation, (unit, m) */
    REAL32_T fCrvRateCpmn =
        0.0F; /* Curvature rate of Third-order polynomial fit for lane motion
                 compensation, (unit, m) */

    REAL32_T
    fPosY03rd = 0.0F; /* Last position Y0 of Third-order polynomial fit for lane
                         motion compensation, (unit, m) */
    REAL32_T fHeading3rd =
        0.0F; /* Last heading angle of Third-order polynomial fit for lane
                 motion compensation, (unit, m) */
    REAL32_T fCrv3rd = 0.0F; /* Curvature Third-order polynomial fit for lane
                                motion compensation, (unit, m) */
    REAL32_T fCrvRate3rd = 0.0F; /* Curvature rate of Third-order polynomial fit
                                    for lane motion compensation, (unit, m) */
    REAL32_T fDevTraj3rd = 0.0F; /* Deviation of Third-order polynomial fit for
                                    lane motion compensation, (unit, m) */

    UINT8_T bRstPosY0Smth =
        0U; /* Apply smooth data transition reset flag, (0, 0~1, -) */
    UINT8_T bRstHeadingSmth =
        0U; /* Apply smooth data transition reset flag, (0, 0~1, -) */
    UINT8_T bRstCrvSmth =
        0U; /* Apply smooth data transition reset flag, (0, 0~1, -) */
    UINT8_T bRstCrvRateSmth =
        0U; /* Apply smooth data transition reset flag, (0, 0~1, -) */
    UINT8_T bEnaPosY0Smth =
        0U; /* Apply smooth data transition enable flag, (0, 0~1, -) */
    UINT8_T bEnaHeadingSmth =
        0U; /* Apply smooth data transition enable flag, (0, 0~1, -) */
    UINT8_T bEnaCrvSmth =
        0U; /* Apply smooth data transition enable flag, (0, 0~1, -) */
    UINT8_T bEnaCrvRateSmth =
        0U; /* Apply smooth data transition enable flag, (0, 0~1, -) */
    UINT8_T bInVldTraj3rd = 0U;

    UINT8_T bRawEnaApplySmth = 0U; /* Set flag of S-R Flip-Flop, (unit, -) */
    UINT8_T bEnaApplySmth = 0U;    /* Set flag of S-R Flip-Flop, (0,0~1, -) */
    UINT8_T bTempLMC = 0U;         /* Temp variable, (unit, -)*/
    UINT8_T bResetFlag = 0U;       /* Reset flag of S-R Flip-Flop, (unit, -) */
    UINT8_T bEnaMotionCmpn =
        0U; /* Lane motion compensation enable flag, (0, 0~1, -) */
    REAL32_T fPosXRaw[POLYFIT_SAMPLE_POINTS] = {
        0.0F}; /* Raw valid length sample points, (unit, m) */
    REAL32_T fPosYRaw[POLYFIT_SAMPLE_POINTS] = {0.0F}; /* Lateral distance by
                                                        raw valid length
                                                        through third-order
                                                        polynomial fitting,
                                                        (unit, m) */
    REAL32_T fPosXPre[POLYFIT_SAMPLE_POINTS] = {
        0.0F}; /* Valid length(After polynomial fitting) sample points, (unit,
                 m) */
    REAL32_T fPosYPre[POLYFIT_SAMPLE_POINTS] = {
        0.0F}; /* Lateral distance by valid length(After polynomial fitting)
                 through third-order polynomial fitting, (unit, m) */
    REAL32_T fDeltaPosX =
        0.0F; /* Increased value of position X in one task cycle, (unit, rad) */
    REAL32_T fDeltaPosY =
        0.0F; /* Increased value of position Y in one task cycle, (unit, rad) */
    REAL32_T fDeltaYaw =
        0.0F; /* Increased value of yaw angle in one task cycle, (unit, rad) */

    REAL32_T fTempLMC = 0.0F; /*  Temp variable, (unit, -)*/
    REAL32_T fPosXRot[POLYFIT_SAMPLE_POINTS] = {
        0}; /* Position X sample points by motion compensation, (unit, m) */
    REAL32_T fPosYRot[POLYFIT_SAMPLE_POINTS] = {
        0}; /* Position Y sample points by motion compensation, (unit, m) */
    UINT8_T bEnaSmth = 0U;
    // UINT8_T uLastMarkerTypeCpmn =
    //     0U; /* Last marker type after motion compensation  ,(0, 0~300, m) */
    // UINT8_T uLastColorCpmn =
    //     0U; /* Last lane color after motion compensation,(0, 0~5, -) */
    sPFTInput_t sPFTInput = {0};   /* Third order polynomial fitting input */
    sPFTOutput_t sPFTOutput = {0}; /* Third order polynomial fitting Output */
    REAL32_T fMaxValidLengthCpmn =
        0.0F; /* Max valid length after motion compensation, (0, 0~300, m) */
    REAL32_T fMinValidLengthCpmn =
        0.0F; /* Min valid length after motion compensation, (0, 0~300, m) */

    UINT8_T bSetLmcEnable =
        0U; /* Set flag for motion compensation enable, (0, 0~1, -) */

    UINT8_T bRstLmcEnable =
        0U; /* Reset flag for motion compensation enable, (0, 0~1, -) */
    UINT8_T bRstLmcByTime = 0U;

    /***********************************1.Motion Compensation Enable
     * flag*****************************************/
    /* Set flag of S-R Flip-Flop for motion compensation must meet following
       conditions:
            1.Motion compensation calibration switch is on;
            2.The valid lane length is greater than the threshold;
            3.Lane line quality is greater than the threshold;
            4.Lane from available to unavailable or Lane line quality decline is
       greater than the threshold. */
    LMC_DetrmCompensationSet(fLastValidLength[LaneIndex],
                             fLastOverallQualityLMC[LaneIndex],
                             bLastAvailableLMC[LaneIndex], pLMCInput, pLMCParam,
                             &bSetLmcEnable, pLMCDebug);

    /* Reset flag of S-R Flip-Flop for motion compensation must meet any of
       following conditions:
            1.The curvature of the lane line exceeds the threshold;
            2.Lane lines are available and of high quality;
            3.Valid length of lane line is too short;
            4.Lane line polynomial fitting is invalid;
            5.Lane line detected change;
            6.The lane line detected by the camera is of poor quality. */
    bRstLmcByTime = TUE_CML_TurnOnDelay_M(
        bLastEnaMotionCmpn[LaneIndex], 3.0f, pLMCParam->fSysCycleTime,
        &fTimerbRstLmcByTime[LaneIndex], bLastRstLmcByTime[LaneIndex]);
    LMC_DetrmCompensationReset(fLastMaxValidLengthCpmn[LaneIndex],
                               bLastInVldTraj3rd[LaneIndex], bRstLmcByTime,
                               pLMCInput, pLMCParam, &bRstLmcEnable, pLMCDebug);

    /* Lane motion compensation enable flag, S-R Flip-Flop */
    bEnaMotionCmpn = TUE_CML_SRTrigger_M(bSetLmcEnable, bRstLmcEnable,
                                         bLastEnaMotionCmpn[LaneIndex]);

    /******************************************2.Calculation of sampling
     * points*************************************/
    /* Sample points from camera input */
    for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        fPosXRaw[ii] =
            pLMCInput->fValidLength * ii / (POLYFIT_SAMPLE_POINTS - 1);
        fPosYRaw[ii] = TUE_CML_PosY3rd_M(fPosXRaw[ii], pLMCInput->fPosY0,
                                         pLMCInput->fHeading, pLMCInput->fCrv,
                                         pLMCInput->fCrvRate);
    }

    /* Sample points from last lane motion compensation */
    for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        fPosXPre[ii] = (fLastMaxValidLengthCpmn[LaneIndex] -
                        fLastMinValidLengthCpmn[LaneIndex]) *
                           ii / (POLYFIT_SAMPLE_POINTS - 1.0F) +
                       fLastMinValidLengthCpmn[LaneIndex];
        fPosYPre[ii] = TUE_CML_PosY3rd_M(
            fPosXPre[ii], fLastPosY03rd[LaneIndex], fLastHeading3rd[LaneIndex],
            fLastCrv3rd[LaneIndex], fLastCrvRate3rd[LaneIndex]);
    }

    /* Delta motion */
    fDeltaYaw = pLMCInput->fVehYawRate * pLMCParam->fSysCycleTime;
    fDeltaPosX = TUE_CML_Cos_M(fDeltaYaw) * pLMCInput->fVehVelX *
                 pLMCParam->fSysCycleTime;
    fDeltaPosY = TUE_CML_Sin_M(fDeltaYaw) * pLMCInput->fVehVelX *
                 pLMCParam->fSysCycleTime;

    /* Motion compensate  Points */
    /* Coordinate rotation and translation */
    /* x' = x*cos(theta) + y * sin(theta) - Dx */
    /* y' = y*cos(theta) - y * sin(theta) - Dy */
    for (UINT8_T ii = 0U; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        fTempLMC = fPosXPre[ii] * TUE_CML_Cos_M(fDeltaYaw) +
                   fPosYPre[ii] * TUE_CML_Sin_M(fDeltaYaw);
        fPosXRot[ii] = fTempLMC - fDeltaPosX;
        fTempLMC = fPosYPre[ii] * TUE_CML_Cos_M(fDeltaYaw) -
                   fPosXPre[ii] * TUE_CML_Sin_M(fDeltaYaw);
        fPosYRot[ii] = fTempLMC - fDeltaPosY;
    }

    /****************************************3.Motion compensation
     * calculation*******************************************/
    sPFTInput.bEnable1st = 0U;
    sPFTInput.bEnable2nd = 0U;

    if ((bEnaMotionCmpn == 1U) ||
        ((pLMCInput->fValidLength > 5.0F) && (pLMCInput->bAvailable == 1U))) {
        sPFTInput.bEnable3rd = 1U;
    } else {
        sPFTInput.bEnable3rd = 0U;
    }

    sPFTInput.fFctCrvDecay = 1.0F;
    sPFTInput.fFctCrvChngDecay = 1.0F;

    for (UINT8_T ii = 0U; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        sPFTInput.fFctWeight[ii] = 1.0F;

        if (bEnaMotionCmpn == 1U) {
            sPFTInput.fPosXArray[ii] = fPosXRot[ii];
            sPFTInput.fPosYArray[ii] = fPosYRot[ii];
        } else {
            sPFTInput.fPosXArray[ii] = fPosXRaw[ii];
            sPFTInput.fPosYArray[ii] = fPosYRaw[ii];
        }

        if (fMaxValidLengthCpmn < sPFTInput.fPosXArray[ii]) {
            fMaxValidLengthCpmn = sPFTInput.fPosXArray[ii];
        } else {
            ;
        }

        if (fMinValidLengthCpmn > sPFTInput.fPosXArray[ii]) {
            fMinValidLengthCpmn = sPFTInput.fPosXArray[ii];
        } else {
            ;
        }
    }
    pLMCOutput->fValidLengthCpmn = fMaxValidLengthCpmn;

    TUE_CML_PolyFit_M(&sPFTInput, &sPFTOutput);
    fPosY03rd = sPFTOutput.fPosY03rd;
    fHeading3rd = sPFTOutput.fHeading3rd;
    fCrv3rd = sPFTOutput.fCrv3rd;
    fCrvRate3rd = sPFTOutput.fChngOfCrv3rd;
    bInVldTraj3rd = sPFTOutput.bTrajInvalid3rd;
    fDevTraj3rd = sPFTOutput.fDevToTraj3rd;

    /*************************************************4.Apply smooth data
     * transition*****************************************/
    /*Enable flag for apply smooth data transition */
    LMC_EnableFlagSmooth(bLastEnaMotionCmpn[LaneIndex], bEnaMotionCmpn,
                         fPosY03rd, fLastPosY03rd[LaneIndex], fHeading3rd,
                         fLastHeading3rd[LaneIndex],
                         bLastRawEnaApplySmth[LaneIndex], &bRawEnaApplySmth,
                         &bEnaApplySmth, pLMCDebug);

    /* Rate limiter for Position Y0 */
    fTempLMC = TUE_CML_Abs_M(fPosY03rd - fLastPosY03rd[LaneIndex]) /
               pLMCParam->fSysCycleTime;
    bRstPosY0Smth = LMC_Comparation(fTempLMC, 0.33f);
    bEnaPosY0Smth =
        TUE_CML_SRTrigger_M(bEnaApplySmth, bLastRstPosY0Smth[LaneIndex],
                            bLastEnaPosY0Smth[LaneIndex]);
    if (bEnaPosY0Smth == 1U) {
        pLMCOutput->fPosY0Cpmn = TUE_CML_GradLimit_M(fPosY03rd, 0.33F, -0.33F,
                                                     pLMCParam->fSysCycleTime,
                                                     fLastPosY0Cpmn[LaneIndex]);
    } else {
        pLMCOutput->fPosY0Cpmn = fPosY03rd;
    }

    /*  Rate Limiter for Heading angle */
    fTempLMC = TUE_CML_Abs_M(fHeading3rd - fLastHeading3rd[LaneIndex]) /
               pLMCParam->fSysCycleTime;
    bRstHeadingSmth = LMC_Comparation(fTempLMC, 0.05f);
    bEnaHeadingSmth =
        TUE_CML_SRTrigger_M(bEnaApplySmth, bLastRstHeadingSmth[LaneIndex],
                            bLastEnaHeadingSmth[LaneIndex]);
    if (bEnaHeadingSmth == 1U) {
        pLMCOutput->fHeadingCpmn = TUE_CML_GradLimit_M(
            fHeading3rd, 0.05F, -0.05F, pLMCParam->fSysCycleTime,
            fLastHeadingCpmn[LaneIndex]);
    } else {
        pLMCOutput->fHeadingCpmn = fHeading3rd;
    }

    /*  Rate Limiter for Curvature */
    fTempLMC = TUE_CML_Abs_M(fCrv3rd - fLastCrv3rd[LaneIndex]) /
               pLMCParam->fSysCycleTime;
    bRstCrvSmth = LMC_Comparation(fTempLMC, 0.0025f);
    bEnaCrvSmth = TUE_CML_SRTrigger_M(bEnaApplySmth, bLastRstCrvSmth[LaneIndex],
                                      bLastEnaCrvSmth[LaneIndex]);
    if (bEnaCrvSmth == 1U) {
        pLMCOutput->fCrvCpmn = TUE_CML_GradLimit_M(fCrv3rd, 0.0025F, -0.0025F,
                                                   pLMCParam->fSysCycleTime,
                                                   fLastCrvCpmn[LaneIndex]);
    } else {
        pLMCOutput->fCrvCpmn = fCrv3rd;
    }

    /*  Rate Limiter for curvature rate */
    fTempLMC = TUE_CML_Abs_M(fCrvRate3rd - fLastCrvRate3rd[LaneIndex]) /
               pLMCParam->fSysCycleTime;
    bRstCrvRateSmth = LMC_Comparation(fTempLMC, 2.5e-5f);
    bEnaCrvRateSmth =
        TUE_CML_SRTrigger_M(bEnaApplySmth, bLastRstCrvRateSmth[LaneIndex],
                            bLastEnaCrvRateSmth[LaneIndex]);
    if (bEnaCrvRateSmth == 1U) {
        pLMCOutput->fCrvRateCpmn = TUE_CML_GradLimit_M(
            fCrvRate3rd, 2.5E-5F, -2.5E-5F, pLMCParam->fSysCycleTime,
            fLastCrvRateCpmn[LaneIndex]);
    } else {
        pLMCOutput->fCrvRateCpmn = fCrvRate3rd;
    }

    /*************************************************5.Rearrange lane
     * boundary*****************************************/
    if (bEnaMotionCmpn == 1U) {
        pLMCOutput->uMarkerTypeCpmn = uLastMarkerTypeCpmn[LaneIndex];
        pLMCOutput->bAvailableCpmn = 1U;
        pLMCOutput->uColorCpmn = uLastColorCpmn[LaneIndex];
        pLMCOutput->fOverallQuality = 30.0F;
    } else {
        pLMCOutput->uMarkerTypeCpmn = pLMCInput->uMarkerType;
        pLMCOutput->bAvailableCpmn = pLMCInput->bAvailable;
        pLMCOutput->uColorCpmn = pLMCInput->uColor;
        pLMCOutput->fOverallQuality = pLMCInput->fOverallQuality;
    }

    /***************************************************Save last
     * value*******************************************/
    fLastValidLength[LaneIndex] = pLMCInput->fValidLength;
    bLastAvailableLMC[LaneIndex] = pLMCInput->bAvailable;
    fLastOverallQualityLMC[LaneIndex] = pLMCInput->fOverallQuality;
    fLastMaxValidLengthCpmn[LaneIndex] = fMaxValidLengthCpmn;
    fLastMinValidLengthCpmn[LaneIndex] = fMinValidLengthCpmn;
    bLastEnaMotionCmpn[LaneIndex] = bEnaMotionCmpn;
    bLastInVldTraj3rd[LaneIndex] = bInVldTraj3rd;
    fLastPosY03rd[LaneIndex] = fPosY03rd;
    fLastHeading3rd[LaneIndex] = fHeading3rd;
    fLastCrv3rd[LaneIndex] = fCrv3rd;
    fLastCrvRate3rd[LaneIndex] = fCrvRate3rd;
    bLastRawEnaApplySmth[LaneIndex] = bRawEnaApplySmth;
    bLastEnaPosY0Smth[LaneIndex] = bEnaPosY0Smth;
    bLastEnaHeadingSmth[LaneIndex] = bEnaHeadingSmth;
    bLastEnaCrvSmth[LaneIndex] = bEnaCrvSmth;
    bLastEnaCrvRateSmth[LaneIndex] = bEnaCrvRateSmth;
    fLastPosY0Cpmn[LaneIndex] = pLMCOutput->fPosY0Cpmn;
    fLastHeadingCpmn[LaneIndex] = pLMCOutput->fHeadingCpmn;
    fLastCrvCpmn[LaneIndex] = pLMCOutput->fCrvCpmn;
    fLastCrvRateCpmn[LaneIndex] = pLMCOutput->fCrvRateCpmn;
    uLastMarkerTypeCpmn[LaneIndex] = pLMCOutput->uMarkerTypeCpmn;
    uLastColorCpmn[LaneIndex] = pLMCOutput->uColorCpmn;
    bLastRstPosY0Smth[LaneIndex] = bRstPosY0Smth;
    bLastRstHeadingSmth[LaneIndex] = bRstHeadingSmth;
    bLastRstCrvSmth[LaneIndex] = bRstCrvSmth;
    bLastRstCrvRateSmth[LaneIndex] = bRstCrvRateSmth;
    bLastRstLmcByTime[LaneIndex] = bRstLmcByTime;

    /***************************************************Output and
     * debug*******************************************/
    pLMCDebug->bSetLmcEnable = bSetLmcEnable;
    pLMCDebug->bRstLmcEnable = bRstLmcEnable;
    pLMCDebug->bEnaMotionCmpn = bEnaMotionCmpn;
    pLMCDebug->fDeltaYaw = fDeltaYaw;
    pLMCDebug->fDeltaPosX = fDeltaPosX;
    pLMCDebug->fDeltaPosY = fDeltaPosY;
    pLMCDebug->bEnaPolyFit3rd = sPFTInput.bEnable3rd;
    pLMCDebug->fMaxValidLengthCpmn = fMaxValidLengthCpmn;
    pLMCDebug->fMinValidLengthCpmn = fMinValidLengthCpmn;
    pLMCDebug->fPosY03rd = fPosY03rd;
    pLMCDebug->fHeading3rd = fHeading3rd;
    pLMCDebug->fCrv3rd = fCrv3rd;
    pLMCDebug->fCrvRate3rd = fCrvRate3rd;
    pLMCDebug->fDevTraj3rd = fDevTraj3rd;

    for (UINT8_T ii = 0U; ii < POLYFIT_SAMPLE_POINTS; ii++) {
        pLMCDebug->fPosXRaw[ii] = fPosXRaw[ii];
        pLMCDebug->fPosYRaw[ii] = fPosYRaw[ii];
        pLMCDebug->fPosXPre[ii] = fPosXPre[ii];
        pLMCDebug->fPosYPre[ii] = fPosYPre[ii];
        pLMCDebug->fPosXRot[ii] = fPosXRot[ii];
        pLMCDebug->fPosXRot[ii] = fPosXRot[ii];
        pLMCDebug->fPosXArray[ii] = sPFTInput.fPosXArray[ii];
        pLMCDebug->fPosYArray[ii] = sPFTInput.fPosYArray[ii];
    }

    pLMCDebug->bRawEnaApplySmth = bRawEnaApplySmth;
    pLMCDebug->bEnaApplySmth = bEnaApplySmth;

    pLMCDebug->bRstPosY0Smth = bRstPosY0Smth;
    pLMCDebug->bRstHeadingSmth = bRstHeadingSmth;
    pLMCDebug->bRstCrvSmth = bRstCrvSmth;
    pLMCDebug->bRstCrvRateSmth = bRstCrvRateSmth;
    pLMCDebug->bEnaPosY0Smth = bEnaPosY0Smth;
    pLMCDebug->bEnaHeadingSmth = bEnaHeadingSmth;
    pLMCDebug->bEnaCrvSmth = bEnaCrvSmth;
    pLMCDebug->bEnaCrvRateSmth = bEnaCrvRateSmth;

} /* LaneMotionCompensation end */

/****************************************************************************************
        @fn           INPUT_UncoupledLaneBridge
        @brief        Input interface for UncoupledLaneBridge
        @description  Input interface for UncoupledLaneBridge:
                                          1.Input from external;
        @param[in]    pULPInput : Input from ULP input
        @param[in]    pULPParam : Parameter from ULP parameter
        @param[out]   pULBInput: Input for INPUT_UncoupledLaneBridge
        @param[out]   pULBParam: Parameter for INPUT_UncoupledLaneBridge
        @return       void
        @startuml
        title INPUT_UncoupledLaneBridge
        (*)--> 1.Input from external
           --> (*)
        @enduml
        *****************************************************************************************/
void INPUT_UncoupledLaneBridge(const sULPInput_t *pULPInput,
                               const sULPParam_t *pULPParam,
                               sULBInput_t *pULBInput,
                               sULBParam_t *pULBParam) {
    /* Input */
    pULBInput->fPosY0UnCplLf = pULPInput->fPosY0CplLf;
    pULBInput->fHeadingUnCplLf = pULPInput->fHeadingCplLf;
    pULBInput->fCrvUnCplLf = pULPInput->fCrvCplLf;
    pULBInput->fCrvRateUnCplLf = pULPInput->fCrvRateCplLf;
    pULBInput->fValidLengthUnCplLf = pULPInput->fValidLengthCplLf;
    pULBInput->bAvailableUnCplLf = pULPInput->bAvailableCplLf;

    pULBInput->fPosY0UnCplRi = pULPInput->fPosY0CplRi;
    pULBInput->fHeadingUnCplRi = pULPInput->fHeadingCplRi;
    pULBInput->fCrvUnCplRi = pULPInput->fCrvCplRi;
    pULBInput->fCrvRateUnCplRi = pULPInput->fCrvRateCplRi;
    pULBInput->fValidLengthUnCplRi = pULPInput->fValidLengthCplRi;
    pULBInput->bAvailableUnCplRi = pULPInput->bAvailableCplRi;

    pULBInput->fVehYawRate = pULPInput->fVehYawRate;
    pULBInput->fVehVelX = pULPInput->fVehVelX;
    pULBInput->bLaneChangeDtct = pULPInput->bLaneChangeDtct;

    pULBInput->uPercExitLf = pULPInput->uPercExitLf;
    pULBInput->uPercExitRi = pULPInput->uPercExitRi;

    pULBInput->bDistYStepDtctLf = pULPInput->bDistYStepDtctLf;
    pULBInput->bDistYStepDtctRi = pULPInput->bDistYStepDtctRi;
    pULBInput->bLineMergeDtcRi = pULPInput->bLineMergeDtcRi;
    pULBInput->bLineMergeDtcLf = pULPInput->bLineMergeDtcLf;

    /* Parameter */
    pULBParam->fThdLaneWidthDiff = 1.0F;
    pULBParam->fTdRawNotParallel = 0.42F;
    pULBParam->fThdLatDistDev = 0.3F;
    pULBParam->bUseStepDtctUnCpl = 1U;
    pULBParam->fThdHeadingDev = 0.15F;
    pULBParam->fThdCrvDev = 0.001F;
    pULBParam->fMaxCrvBridgeUnCpl = 0.003F;
    pULBParam->fMinCrvBridgeUnCpl = 0.002F;

    pULBParam->fSysCycleTime = pULPParam->fSysCycleTime;
}

void ULB_LaneParallelism(const sULBInput_t *pULBInput,
                         const sULBParam_t *pULBParam,
                         sULBDebug_t *pULBDebug,
                         REAL32_T *fMaxLengthUnCpl,
                         REAL32_T *fRotPosYUnCplLf,
                         REAL32_T *fRotPosYUnCplRi,
                         UINT8_T *bRawNotParallelUnCpl,
                         UINT8_T *bNotParallelUnCpl) {
    REAL32_T fLaneWidthUnCpl = 0.0F; /* Lane width at X = 0.0m, (0, 0~10, m) */
    REAL32_T fDiffLaneWidthUnCpl =
        0.0F; /* uncoupled lane width difference, (0, 0~10, m) */
    REAL32_T
    fPosYAtMaxUnCplLf = 0.0F; /* Left lane lateral distance at max raw valid
                                 length (0, -15~15, m) */
    REAL32_T
    fPosYAtMaxUnCplRi = 0.0F; /* Right lane lateral distance at max raw
                                 valid length (0, -15~15, m) */
    REAL32_T fLaneWidthAtMaxUnCpl =
        0.0F; /* Lane width at max raw valid length, (0, 0~10, m) */
    UINT8_T bDlyNotParallelUnCpl =
        0U; /* Flag after turn on delay that uncoupled lanes are not parallel,
               (0, 0~1, -) */

    /***************************1.Determine uncoupled
     * lane parallelism*********************/
    /***************************1.1 Check uncoupled lane width
     * difference******************/
    if ((pULBInput->bAvailableUnCplLf == 1U) &&
        (pULBInput->bAvailableUnCplRi == 1U)) {
        /* Raw uncoupled lane width */
        fLaneWidthUnCpl = pULBInput->fPosY0UnCplLf *
                              TUE_CML_Cos_M(pULBInput->fHeadingUnCplLf) -
                          pULBInput->fPosY0UnCplRi *
                              TUE_CML_Cos_M(pULBInput->fHeadingUnCplRi);
        fLaneWidthUnCpl = TUE_CML_Abs_M(fLaneWidthUnCpl);

        /* uncoupled lane width at max valid length */
        *fMaxLengthUnCpl = TUE_CML_Max_M(pULBInput->fValidLengthUnCplLf,
                                         pULBInput->fValidLengthUnCplRi);
        fPosYAtMaxUnCplLf = TUE_CML_PosY3rd_M(
            *fMaxLengthUnCpl, pULBInput->fPosY0UnCplLf,
            pULBInput->fHeadingUnCplLf, pULBInput->fCrvUnCplLf,
            pULBInput->fCrvRateUnCplLf);
        fPosYAtMaxUnCplRi = TUE_CML_PosY3rd_M(
            *fMaxLengthUnCpl, pULBInput->fPosY0UnCplRi,
            pULBInput->fHeadingUnCplRi, pULBInput->fCrvUnCplRi,
            pULBInput->fCrvRateUnCplRi);

        *fRotPosYUnCplLf =
            *fMaxLengthUnCpl * TUE_CML_Sin_M(pULBInput->fHeadingUnCplLf) -
            fPosYAtMaxUnCplLf * TUE_CML_Cos_M(pULBInput->fHeadingUnCplLf);
        *fRotPosYUnCplRi =
            *fMaxLengthUnCpl * TUE_CML_Sin_M(pULBInput->fHeadingUnCplRi) -
            fPosYAtMaxUnCplRi * TUE_CML_Cos_M(pULBInput->fHeadingUnCplRi);
        fLaneWidthAtMaxUnCpl = *fRotPosYUnCplLf - *fRotPosYUnCplRi;
        fLaneWidthAtMaxUnCpl = TUE_CML_Abs_M(fLaneWidthAtMaxUnCpl);

        /* uncoupled lane width difference */
        fDiffLaneWidthUnCpl = fLaneWidthAtMaxUnCpl - fLaneWidthUnCpl;
    } else {
        fDiffLaneWidthUnCpl = 0.0F;
    }

    /***************************1.2 Determine uncoupled lane
     * parallelism*******************/
    if (fDiffLaneWidthUnCpl > pULBParam->fThdLaneWidthDiff) {
        *bRawNotParallelUnCpl = 1U;
    } else {
        *bRawNotParallelUnCpl = 0U;
    }

    bDlyNotParallelUnCpl = TUE_CML_TurnOnDelay_M(
        *bRawNotParallelUnCpl, pULBParam->fTdRawNotParallel,
        pULBParam->fSysCycleTime, &fTimerRawNotParallel,
        bLastDlyNotParallelUnCpl);
    if ((bLastDlyNotParallelUnCpl == 0U) && (bDlyNotParallelUnCpl == 1U)) {
        *bNotParallelUnCpl = 1U;
    } else {
        *bNotParallelUnCpl = 0U;
    }

    pULBDebug->fLaneWidthUnCpl = fLaneWidthUnCpl;
    pULBDebug->fPosYAtMaxUnCplLf = fPosYAtMaxUnCplLf;
    pULBDebug->fPosYAtMaxUnCplRi = fPosYAtMaxUnCplRi;
    pULBDebug->fLaneWidthAtMaxUnCpl = fLaneWidthAtMaxUnCpl;
    pULBDebug->fDiffLaneWidthUnCpl = fDiffLaneWidthUnCpl;
    pULBDebug->fTimerRawNotParallel = fTimerRawNotParallel;
    pULBDebug->fLaneWidthAtMaxUnCpl = fLaneWidthAtMaxUnCpl;
    pULBDebug->fLaneWidthAtMaxUnCpl = fLaneWidthAtMaxUnCpl;
    pULBDebug->fLaneWidthAtMaxUnCpl = fLaneWidthAtMaxUnCpl;
    pULBDebug->fLaneWidthAtMaxUnCpl = fLaneWidthAtMaxUnCpl;
    pULBDebug->bDlyNotParallelUnCpl = bDlyNotParallelUnCpl;

    bLastDlyNotParallelUnCpl = bDlyNotParallelUnCpl;
}

void ULB_LateralDisDiffOverTine(const sULBInput_t *pULBInput,
                                const sULBParam_t *pULBParam,
                                UINT8_T bLastRawNotParallelUnCpl,
                                UINT8_T bRawNotParallelUnCpl,
                                REAL32_T fMaxLengthUnCpl,
                                REAL32_T fRotPosYUnCplLf,
                                REAL32_T fRotPosYUnCplRi,
                                sULBDebug_t *pULBDebug,
                                REAL32_T *fDevLatDistCplLf,
                                REAL32_T *fDevLatDistCplRi,
                                UINT8_T *bTrigLatDistDev) {
    REAL32_T fUnitDeltaYawCplRi = 0.0F;
    REAL32_T fUnitDeltaYawCplLf = 0.0F;
    REAL32_T fDeltaYawCplLf = 0.0F;
    REAL32_T fDeltaYawCplRi = 0.0F;
    REAL32_T fUnitDeltaXUnCplLf = 0.0F;
    REAL32_T fUnitDeltaYUnCplLf = 0.0F;
    REAL32_T fUnitDeltaXUnCplRi = 0.0F;
    REAL32_T fUnitDeltaYUnCplRi = 0.0F;
    REAL32_T fDeltaXUnCplLf = 0.0F;
    REAL32_T fDeltaYUnCplLf = 0.0F;
    REAL32_T fDeltaXUnCplRi = 0.0F;
    REAL32_T fDeltaYUnCplRi = 0.0F;
    REAL32_T fDlyPosXUnCplLf =
        0.0F; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T fDlyPosYUnCplLf =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fDlyPosXUnCplRi =
        0.0F; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T fDlyPosYUnCplRi =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fCpmnPosXUnCplLf =
        0.0F; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T fCpmnPosYUnCplLf =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fCpmnPosXUnCplRi =
        0.0F; /* Position X after delay 8 cycles, (0, -300~300, m) */
    REAL32_T fCpmnPosYUnCplRi =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fRotCmpnPosYUnCplLf =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fRotCmpnPosYUnCplRi =
        0.0F; /* Position X after delay 8 cycles, (0, -15~15, m) */
    REAL32_T fDevLatDistDev = 0.0F;

    /***************************2.Evaluate lateral distance difference over
     * time***********/
    /***************************2.1 Rotate X and Y point at max
     * length*********************/
    /* Delta motion accumulation */
    fUnitDeltaYawCplLf = pULBInput->fVehYawRate * pULBParam->fSysCycleTime;
    fUnitDeltaYawCplRi = pULBInput->fVehYawRate * pULBParam->fSysCycleTime;

    if ((bLastRawNotParallelUnCpl == 0U) && (bRawNotParallelUnCpl == 1U)) {
        fDeltaYawCplLf = fUnitDeltaYawCplLf;
        fDeltaYawCplRi = fUnitDeltaYawCplRi;
    } else {
        fDeltaYawCplLf = fLastDeltaYawCplLf + fUnitDeltaYawCplLf;
        fDeltaYawCplRi = fLastDeltaYawCplRi + fUnitDeltaYawCplRi;
    }

    fUnitDeltaXUnCplLf = pULBInput->fVehVelX * pULBParam->fSysCycleTime *
                         TUE_CML_Cos_M(fDeltaYawCplLf);
    fUnitDeltaYUnCplLf = pULBInput->fVehVelX * pULBParam->fSysCycleTime *
                         TUE_CML_Sin_M(fDeltaYawCplLf);

    fUnitDeltaXUnCplRi = pULBInput->fVehVelX * pULBParam->fSysCycleTime *
                         TUE_CML_Cos_M(fDeltaYawCplRi);
    fUnitDeltaYUnCplRi = pULBInput->fVehVelX * pULBParam->fSysCycleTime *
                         TUE_CML_Sin_M(fDeltaYawCplRi);

    if ((bLastRawNotParallelUnCpl == 0U) && (bRawNotParallelUnCpl == 1U)) {
        fDeltaXUnCplLf = fUnitDeltaXUnCplLf;
        fDeltaYUnCplLf = fUnitDeltaYUnCplLf;

        fDeltaXUnCplRi = fUnitDeltaXUnCplRi;
        fDeltaYUnCplRi = fUnitDeltaYUnCplRi;
    } else {
        fDeltaXUnCplLf = fLastDeltaXUnCplLf + fUnitDeltaXUnCplLf;
        fDeltaYUnCplLf = fLastDeltaYUnCplLf + fUnitDeltaYUnCplLf;

        fDeltaXUnCplRi = fLastDeltaXUnCplRi + fUnitDeltaXUnCplRi;
        fDeltaYUnCplRi = fLastDeltaYUnCplRi + fUnitDeltaYUnCplRi;
    }

    /* Calculate Position Y */
    fDlyPosXUnCplLf = fMaxLengthUnCpl + fDeltaXUnCplLf;
    fDlyPosYUnCplLf = TUE_CML_PosY3rd_M(
        fDlyPosXUnCplLf, fClothoidUnCplLf[0][0], fClothoidUnCplLf[1][0],
        fClothoidUnCplLf[2][0], fClothoidUnCplLf[3][0]);

    fDlyPosXUnCplRi = fMaxLengthUnCpl + fDeltaXUnCplRi;
    fDlyPosYUnCplRi = TUE_CML_PosY3rd_M(
        fDlyPosXUnCplRi, fClothoidUnCplRi[0][0], fClothoidUnCplRi[1][0],
        fClothoidUnCplRi[2][0], fClothoidUnCplRi[3][0]);

    /* Motion compensateXY Points */
    fCpmnPosXUnCplLf = fDlyPosXUnCplLf * TUE_CML_Cos_M(fDeltaYawCplLf) +
                       fDlyPosYUnCplLf * TUE_CML_Sin_M(fDeltaYawCplLf) -
                       fDeltaXUnCplLf;
    fCpmnPosYUnCplLf = fDlyPosYUnCplLf * TUE_CML_Cos_M(fDeltaYawCplLf) -
                       fDlyPosXUnCplLf * TUE_CML_Sin_M(fDeltaYawCplLf) -
                       fDeltaYUnCplLf;

    fCpmnPosXUnCplRi = fDlyPosXUnCplRi * TUE_CML_Cos_M(fDeltaYawCplRi) +
                       fDlyPosYUnCplRi * TUE_CML_Sin_M(fDeltaYawCplRi) -
                       fDeltaXUnCplRi;
    fCpmnPosYUnCplRi = fDlyPosYUnCplRi * TUE_CML_Cos_M(fDeltaYawCplRi) -
                       fDlyPosXUnCplRi * TUE_CML_Sin_M(fDeltaYawCplRi) -
                       fDeltaYUnCplRi;

    /* Rotate XY point by yaw */
    fRotCmpnPosYUnCplLf =
        -fCpmnPosXUnCplLf * TUE_CML_Sin_M(pULBInput->fHeadingUnCplLf) +
        fCpmnPosYUnCplLf * TUE_CML_Cos_M(pULBInput->fHeadingUnCplLf);
    fRotCmpnPosYUnCplRi =
        -fCpmnPosXUnCplRi * TUE_CML_Sin_M(pULBInput->fHeadingUnCplRi) +
        fCpmnPosYUnCplRi * TUE_CML_Cos_M(pULBInput->fHeadingUnCplRi);

    /* Lateral distance deviation */
    *fDevLatDistCplLf = TUE_CML_Abs_M(fRotCmpnPosYUnCplLf + fRotPosYUnCplLf);
    *fDevLatDistCplRi = TUE_CML_Abs_M(fRotCmpnPosYUnCplRi + fRotPosYUnCplRi);
    if (fClothoidUnCplLf[0][0] == 0 && fClothoidUnCplLf[1][0] == 0 &&
        fClothoidUnCplLf[2][0] == 0 && fClothoidUnCplLf[3][0] == 0) {
        *fDevLatDistCplLf = 0.f;
        *fDevLatDistCplRi = 0.f;
    }
    fDevLatDistDev = TUE_CML_Abs_M(*fDevLatDistCplLf - *fDevLatDistCplRi);
    if (fDevLatDistDev > pULBParam->fThdLatDistDev) {
        *bTrigLatDistDev = 1U;
    } else {
        *bTrigLatDistDev = 0U;
    }

    /* Lane clothoid delay 8 cycles */
    for (UINT8_T ii = 0U; ii < 4U; ii++) {
        for (UINT8_T jj = 0U; jj < 7U; jj++) {
            fClothoidUnCplLf[ii][jj] = fClothoidUnCplLf[ii][jj + 1];
            fClothoidUnCplRi[ii][jj] = fClothoidUnCplRi[ii][jj + 1];
        }
    }
    fClothoidUnCplLf[0][7] = pULBInput->fPosY0UnCplLf;
    fClothoidUnCplLf[1][7] = pULBInput->fHeadingUnCplLf;
    fClothoidUnCplLf[2][7] = pULBInput->fCrvUnCplLf;
    fClothoidUnCplLf[3][7] = pULBInput->fCrvRateUnCplLf;

    fClothoidUnCplRi[0][7] = pULBInput->fPosY0UnCplRi;
    fClothoidUnCplRi[1][7] = pULBInput->fHeadingUnCplRi;
    fClothoidUnCplRi[2][7] = pULBInput->fCrvUnCplRi;
    fClothoidUnCplRi[3][7] = pULBInput->fCrvRateUnCplRi;

    pULBDebug->fDeltaYawCplLf = fDeltaYawCplLf;
    pULBDebug->fDeltaXUnCplLf = fDeltaXUnCplLf;
    pULBDebug->fDeltaYUnCplLf = fDeltaYUnCplLf;
    pULBDebug->fDeltaYawCplRi = fDeltaYawCplRi;
    pULBDebug->fDeltaXUnCplRi = fDeltaXUnCplRi;
    pULBDebug->fDeltaYUnCplRi = fDeltaYUnCplRi;
    pULBDebug->fUnitDeltaXUnCplLf = fUnitDeltaXUnCplLf;
    pULBDebug->fUnitDeltaYUnCplLf = fUnitDeltaYUnCplLf;
    pULBDebug->fUnitDeltaYawCplRi = fUnitDeltaYawCplRi;
    pULBDebug->fUnitDeltaYawCplLf = fUnitDeltaYawCplLf;
    pULBDebug->fUnitDeltaXUnCplRi = fUnitDeltaXUnCplRi;
    pULBDebug->fUnitDeltaYUnCplRi = fUnitDeltaYUnCplRi;
    pULBDebug->fDlyPosXUnCplLf = fDlyPosXUnCplLf;
    pULBDebug->fDlyPosYUnCplLf = fDlyPosYUnCplLf;
    pULBDebug->fDlyPosXUnCplRi = fDlyPosXUnCplRi;
    pULBDebug->fDlyPosYUnCplRi = fDlyPosYUnCplRi;
    pULBDebug->fCpmnPosXUnCplLf = fCpmnPosXUnCplLf;
    pULBDebug->fCpmnPosYUnCplLf = fCpmnPosYUnCplLf;
    pULBDebug->fCpmnPosXUnCplRi = fCpmnPosXUnCplRi;
    pULBDebug->fCpmnPosYUnCplRi = fCpmnPosYUnCplRi;
    pULBDebug->fRotCmpnPosYUnCplLf = fRotCmpnPosYUnCplLf;
    pULBDebug->fRotCmpnPosYUnCplRi = fRotCmpnPosYUnCplRi;

    pULBDebug->fDevLatDistDev = fDevLatDistDev;

    fLastDeltaYawCplLf = fDeltaYawCplLf;
    fLastDeltaXUnCplLf = fDeltaXUnCplLf;
    fLastDeltaYUnCplLf = fDeltaYUnCplLf;
    fLastDeltaYawCplRi = fDeltaYawCplRi;
    fLastDeltaXUnCplRi = fDeltaXUnCplRi;
    fLastDeltaYUnCplRi = fDeltaYUnCplRi;
}

void ULB_PenaltyStepDetection(const sULBInput_t *pULBInput,
                              const sULBParam_t *pULBParam,
                              sULBDebug_t *pULBDebug,
                              UINT8_T *bStepDtctUnCplLf,
                              UINT8_T *bStepDtctUnCplRi) {
    REAL32_T fTempULB = 0.0F;
    REAL32_T fPenaltyPosY0UnCplLf = 0.0F;
    REAL32_T fPenaltyHeadingUnCplLf = 0.0F;
    REAL32_T fPenaltyCrvUnCplLf = 0.0F;
    REAL32_T fPenaltyCrvRateUnCplLf = 0.0F;
    REAL32_T fRawPenaltyUnCplLf = 0.0F;
    REAL32_T fPenaltyUnCplLf = 0.0F;

    REAL32_T fPenaltyPosY0UnCplRi = 0.0F;
    REAL32_T fPenaltyHeadingUnCplRi = 0.0F;
    REAL32_T fPenaltyCrvUnCplRi = 0.0F;
    REAL32_T fPenaltyCrvRateUnCplRi = 0.0F;
    REAL32_T fRawPenaltyUnCplRi = 0.0F;
    REAL32_T fPenaltyUnCplRi = 0.0F;
    UINT8_T bRawStepDtctUnCplLf = 0U;
    UINT8_T bRawStepDtctUnCplRi = 0U;

    /***************************3.Penalty for step
     * detections******************************/
    /******************************1.1 Penalty for step
     * detections*************************************/
    /* (step - min)/(max - min) *factor */
    /* Step detection by PosY0 */
    fTempULB = TUE_CML_Abs_M(pULBInput->fPosY0UnCplLf - fLastPosY0UnCplLf);
    fTempULB = (fTempULB - 0.05F) / (0.15F - 0.05F);
    fPenaltyPosY0UnCplLf = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 5.0F;

    fTempULB = TUE_CML_Abs_M(pULBInput->fPosY0UnCplRi - fLastPosY0UnCplRi);
    fTempULB = (fTempULB - 0.05F) / (0.15F - 0.05F);
    fPenaltyPosY0UnCplRi = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 5.0F;

    /* Step detection by Heading Angle */
    fTempULB = TUE_CML_Abs_M(pULBInput->fHeadingUnCplLf - fLastHeadingUnCplLf);
    fTempULB = (fTempULB - 0.004F) / (0.008f - 0.004F);
    fPenaltyHeadingUnCplLf = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 4.0F;

    fTempULB = TUE_CML_Abs_M(pULBInput->fHeadingUnCplRi - fLastHeadingUnCplRi);
    fTempULB = (fTempULB - 0.004F) / (0.008f - 0.004F);
    fPenaltyHeadingUnCplRi = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 4.0F;

    /* Step detection by Curvature */
    fTempULB = TUE_CML_Abs_M(pULBInput->fCrvUnCplLf - fLastCrvUnCplLf);
    fTempULB = (fTempULB - 3E-4F) / (6E-4f - 3E-4f);
    fPenaltyCrvUnCplLf = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 5.0F;

    fTempULB = TUE_CML_Abs_M(pULBInput->fCrvUnCplRi - fLastCrvUnCplRi);
    fTempULB = (fTempULB - 3E-4F) / (6E-4f - 3E-4f);
    fPenaltyCrvUnCplRi = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 5.0F;

    /* Step detection by Curvature rate */
    fTempULB = TUE_CML_Abs_M(pULBInput->fCrvRateUnCplLf - fLastCrvRateUnCplLf);
    fTempULB = (fTempULB - 3E-6F) / (6E-6F - 3E-6F);
    fPenaltyCrvRateUnCplLf = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 2.0F;

    fTempULB = TUE_CML_Abs_M(pULBInput->fCrvRateUnCplRi - fLastCrvRateUnCplRi);
    fTempULB = (fTempULB - 3E-6F) / (6E-6F - 3E-6F);
    fPenaltyCrvRateUnCplRi = TUE_CML_Limit_M(0.0F, 1.0F, fTempULB) * 2.0F;

    /* Quality penalty coefficient by step detection */
    if ((pULBInput->bAvailableUnCplLf == bLastAvailableUnCplLf) &&
        (pULBInput->bAvailableUnCplRi == bLastAvailableUnCplRi) &&
        (pULBInput->bLaneChangeDtct == 0U)) {
        fRawPenaltyUnCplLf = fPenaltyPosY0UnCplLf + fPenaltyHeadingUnCplLf +
                             fPenaltyCrvUnCplLf + fPenaltyCrvRateUnCplLf;
        if ((UINT8_T)fRawPenaltyUnCplLf == 0U) {
            fRawPenaltyUnCplLf = 0.0F;
        } else {
            fRawPenaltyUnCplLf = fRawPenaltyUnCplLf + fLastPenaltyUnCplLf;
        }

        fRawPenaltyUnCplRi = fPenaltyPosY0UnCplRi + fPenaltyHeadingUnCplRi +
                             fPenaltyCrvUnCplRi + fPenaltyCrvRateUnCplRi;
        if ((UINT8_T)fRawPenaltyUnCplRi == 0U) {
            fRawPenaltyUnCplRi = 0.0F;
        } else {
            fRawPenaltyUnCplRi = fRawPenaltyUnCplRi + fLastPenaltyUnCplRi;
        }
    } else {
        fRawPenaltyUnCplLf = 0.0F;
        fRawPenaltyUnCplRi = 0.0F;
    }

    fPenaltyUnCplLf =
        TUE_CML_GradLimit_M(fRawPenaltyUnCplLf, 1000.0F, -1000.0F,
                            pULBParam->fSysCycleTime, fLastPenaltyUnCplLf);
    fPenaltyUnCplRi =
        TUE_CML_GradLimit_M(fRawPenaltyUnCplRi, 1000.0F, -1000.0F,
                            pULBParam->fSysCycleTime, fLastPenaltyUnCplRi);
    if (fPenaltyUnCplLf > 40.0F) {
        bRawStepDtctUnCplLf = 1U;
    } else {
        bRawStepDtctUnCplLf = 0U;
    }

    if (fPenaltyUnCplRi > 40.0F) {
        bRawStepDtctUnCplRi = 1U;
    } else {
        bRawStepDtctUnCplRi = 0U;
    }

    if (pULBParam->bUseStepDtctUnCpl == 1U) {
        if ((bLastRawStepDtctUnCplLf == 0U) && (bRawStepDtctUnCplLf == 1U)) {
            *bStepDtctUnCplLf = 1U;
        } else {
            *bStepDtctUnCplLf = 0U;
        }

        if ((bLastRawStepDtctUnCplRi == 0U) && (bRawStepDtctUnCplRi == 1U)) {
            *bStepDtctUnCplRi = 1U;
        } else {
            *bStepDtctUnCplRi = 0U;
        }
    } else {
        *bStepDtctUnCplLf = 0U;
        *bStepDtctUnCplRi = 0U;
    }

    pULBDebug->fPenaltyPosY0UnCplLf = fPenaltyPosY0UnCplLf;
    pULBDebug->fPenaltyHeadingUnCplLf = fPenaltyHeadingUnCplLf;
    pULBDebug->fPenaltyCrvUnCplLf = fPenaltyCrvUnCplLf;
    pULBDebug->fPenaltyCrvRateUnCplLf = fPenaltyCrvRateUnCplLf;
    pULBDebug->fRawPenaltyUnCplLf = fRawPenaltyUnCplLf;
    pULBDebug->fPenaltyUnCplLf = fPenaltyUnCplLf;
    pULBDebug->fPenaltyPosY0UnCplRi = fPenaltyPosY0UnCplRi;
    pULBDebug->fPenaltyHeadingUnCplRi = fPenaltyHeadingUnCplRi;
    pULBDebug->fPenaltyCrvUnCplRi = fPenaltyCrvUnCplRi;
    pULBDebug->fPenaltyCrvRateUnCplRi = fPenaltyCrvRateUnCplRi;
    pULBDebug->fRawPenaltyUnCplRi = fRawPenaltyUnCplRi;
    pULBDebug->fPenaltyUnCplRi = fPenaltyUnCplRi;
    pULBDebug->bRawStepDtctUnCplLf = bRawStepDtctUnCplLf;
    pULBDebug->bRawStepDtctUnCplRi = bRawStepDtctUnCplRi;

    fLastPosY0UnCplLf = pULBInput->fPosY0UnCplLf;
    fLastHeadingUnCplLf = pULBInput->fHeadingUnCplLf;
    fLastCrvUnCplLf = pULBInput->fCrvUnCplLf;
    fLastCrvRateUnCplLf = pULBInput->fCrvRateUnCplLf;
    fLastPosY0UnCplRi = pULBInput->fPosY0UnCplRi;
    fLastHeadingUnCplRi = pULBInput->fHeadingUnCplRi;
    fLastCrvUnCplRi = pULBInput->fCrvUnCplRi;
    fLastCrvRateUnCplRi = pULBInput->fCrvRateUnCplRi;
    bLastAvailableUnCplLf = pULBInput->bAvailableUnCplLf;
    bLastAvailableUnCplRi = pULBInput->bAvailableUnCplRi;
    fLastPenaltyUnCplLf = fPenaltyUnCplLf;
    fLastPenaltyUnCplRi = fPenaltyUnCplRi;
    bLastRawStepDtctUnCplLf = bRawStepDtctUnCplLf;
    bLastRawStepDtctUnCplRi = bRawStepDtctUnCplRi;
}

void ULB_OutputBitfiled(UINT8_T bStepDtctUnCplLf,
                        UINT8_T bStepDtctUnCplRi,
                        UINT8_T bNotParallelBridgeUnCplLf,
                        UINT8_T bNotParallelBridgeUnCplRi,
                        UINT8_T bTrigHeadingDevUnCpl,
                        UINT8_T bTrigCrvDevUnCpl,
                        sULBOutput_t *pULBOutput) {
    /***************************Output and
     * debug*******************************************/
    /* Uncoupled lane bridge bitfield */
    if (bStepDtctUnCplLf == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 0U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 0U);
    }

    if (bStepDtctUnCplRi == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 1U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 1U);
    }

    if (bNotParallelBridgeUnCplLf == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 2U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 2U);
    }

    if (bNotParallelBridgeUnCplRi == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 3U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 3U);
    }

    if (bTrigHeadingDevUnCpl == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 4U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 4U);
    }

    if (bTrigCrvDevUnCpl == 1U) {
        TUE_CML_Setbit_M(pULBOutput->uBtfBridgeUnCpl, 5U);
    } else {
        TUE_CML_Clrbit_M(pULBOutput->uBtfBridgeUnCpl, 5U);
    }
}

/****************************************************************************************
        @fn           UncoupledLaneBridge
        @brief        Determine whether the uncoupled lane can be bridged
        @description  Uncoupled lane bridge:
                                          1.Determine uncoupled lane
 parallelism;
                                          2.Evaluate lateral distance difference
 over time;
                                          3.Penalty for step detections;
                                          4.Evaluate curvature and heading over
 time;
                                          5.Evaluate uncoupled lane bridge.
        @param[in]    pULBInput : Input for UncoupledLaneBridge
        @param[in]    pULBParam : Parameter for UncoupledLaneBridge
        @param[out]   pULBOutput: Output for UncoupledLaneBridge
        @param[out]   pULBDebug : Debug(measurement) for UncoupledLaneBridge
        @return       void
        @startuml
        title UncoupledLaneBridge
        (*)--> 1.DetermineUncoupledLaneParallelism
           --> 2.EvaluateLateralDistanceDifferenceOverTime
        1.DetermineUncoupledLaneParallelism -->
 4.EvaluateCurvatureAndHeadingOverTime
        1.DetermineUncoupledLaneParallelism --> 5.EvaluateUncoupledLaneBridge
        (*)--> 3.PenaltyForStepDetections
           --> 5.EvaluateUncoupledLaneBridge
        2.EvaluateLateralDistanceDifferenceOverTime-->
 5.EvaluateUncoupledLaneBridge
        4.EvaluateCurvatureAndHeadingOverTime--> 5.EvaluateUncoupledLaneBridge
           --> (*)
        @enduml
 ******************************************************************************************/
void UncoupledLaneBridge(const sULBInput_t *pULBInput,
                         const sULBParam_t *pULBParam,
                         sULBOutput_t *pULBOutput,
                         sULBDebug_t *pULBDebug) {
    REAL32_T fMaxLengthUnCpl =
        0.0F; /* Max valid length by raw uncoupled lane, (0, 0~300, m)*/
    UINT8_T bRawNotParallelUnCpl =
        0U; /* Raw flag that uncoupled lanes are not parallel, (0, 0~1, -) */
    UINT8_T bNotParallelUnCpl =
        0U; /* Flag that uncoupled lanes are not parallel, (0, 0~1, -) */
    REAL32_T fRotPosYUnCplLf = 0.0F; /* Rotate uncoupled left lane PosY at
    max
                                        length, (0, -15~15, m) */
    REAL32_T
    fRotPosYUnCplRi = 0.0F; /* Rotate uncoupled right lane PosY at
max
                               length, (0, -15~15, m) */
    // REAL32_T fDlyRotYUnCplLf = 0.0F; /* Rotate uncoupled left lane PosY at
    // max
    //                                     length after delay, (0, -15~15, m) */
    // REAL32_T
    // fDlyRotYUnCplRi = 0.0F; /* Rotate uncoupled right lane PosY at max
    //                            length after delay, (0, -15~15, m) */
    REAL32_T fDevLatDistCplLf = 0.0F;
    REAL32_T fDevLatDistCplRi = 0.0F;
    UINT8_T bTrigLatDistDev = 0U;

    UINT8_T bStepDtctUnCplLf = 0U;
    UINT8_T bStepDtctUnCplRi = 0U;

    REAL32_T fHeadingPolyUnCplLf = 0.0F;
    REAL32_T fHeadingPolyUnCplRi = 0.0F;
    REAL32_T fDevHeadingUnCplLf = 0.0F;
    REAL32_T fDevHeadingUnCplRi = 0.0F;
    REAL32_T fDevHeadingDevUnCpl = 0.0F;
    UINT8_T bTrigHeadingDevUnCpl = 0U;
    REAL32_T fCrvRefUnCplLf = 0.0F;
    REAL32_T fCrvRefUnCplRi = 0.0F;
    REAL32_T fDevCrvUnCplLf = 0.0F;
    REAL32_T fDevCrvUnCplRi = 0.0F;
    REAL32_T fDevCrvDevUnCpl = 0.0F;
    UINT8_T bTrigCrvDevUnCpl = 0U;

    UINT8_T bNotParallelBridgeUnCplLf = 0U;
    UINT8_T bSetBridgeUnCplLf = 0U;
    UINT8_T bResetBridgeUnCplLf = 0U;
    UINT8_T bDlySetBridgeUnCplLf = 0U;

    UINT8_T bResetByCrvUnCplLf = 0U;
    UINT8_T bNotParallelBridgeUnCplRi = 0U;
    UINT8_T bSetBridgeUnCplRi = 0U;
    UINT8_T bResetBridgeUnCplRi = 0U;
    UINT8_T bDlySetBridgeUnCplRi = 0U;

    UINT8_T bResetByCrvUnCplRi = 0U;

    /***************************1.Determine uncoupled lane
     * parallelism*********************/
    ULB_LaneParallelism(pULBInput, pULBParam, pULBDebug, &fMaxLengthUnCpl,
                        &fRotPosYUnCplLf, &fRotPosYUnCplRi,
                        &bRawNotParallelUnCpl, &bNotParallelUnCpl);

    /***************************2.Evaluate lateral distance difference over
     * time***********/
    ULB_LateralDisDiffOverTine(
        pULBInput, pULBParam, bLastRawNotParallelUnCpl, bRawNotParallelUnCpl,
        fMaxLengthUnCpl, fRotPosYUnCplLf, fRotPosYUnCplRi, pULBDebug,
        &fDevLatDistCplLf, &fDevLatDistCplRi, &bTrigLatDistDev);

    /***************************3.Penalty for step
     * detections******************************/

    ULB_PenaltyStepDetection(pULBInput, pULBParam, pULBDebug, &bStepDtctUnCplLf,
                             &bStepDtctUnCplRi);

    /***************************4.Evaluate curvature and heading over
     * time*****************/
    fHeadingPolyUnCplLf = TUE_CML_Yaw3rd_M(
        pULBInput->fValidLengthUnCplLf, pULBInput->fHeadingUnCplLf,
        pULBInput->fCrvUnCplLf, pULBInput->fCrvRateUnCplLf);

    fHeadingPolyUnCplRi = TUE_CML_Yaw3rd_M(
        pULBInput->fValidLengthUnCplRi, pULBInput->fHeadingUnCplRi,
        pULBInput->fCrvUnCplRi, pULBInput->fCrvRateUnCplRi);

    fDevHeadingUnCplLf =
        TUE_CML_Abs_M(fHeadingPolyUnCplLf - pULBInput->fHeadingUnCplLf);
    fDevHeadingUnCplRi =
        TUE_CML_Abs_M(fHeadingPolyUnCplRi - pULBInput->fHeadingUnCplRi);
    fDevHeadingDevUnCpl =
        TUE_CML_Abs_M(fDevHeadingUnCplLf - fDevHeadingUnCplRi);
    if ((fDevHeadingDevUnCpl > pULBParam->fThdHeadingDev) &&
        (bNotParallelUnCpl == 1U)) {
        bTrigHeadingDevUnCpl = 1U;
    } else {
        bTrigHeadingDevUnCpl = 0U;
    }

    if (bRawNotParallelUnCpl == 0U) {
        fCrvRefUnCplLf = pULBInput->fCrvUnCplLf;
        fCrvRefUnCplRi = pULBInput->fCrvUnCplRi;
    } else {
        fCrvRefUnCplLf = fLastCrvRefUnCplLf;
        fCrvRefUnCplRi = fLastCrvRefUnCplRi;
    }
    fDevCrvUnCplLf = TUE_CML_Abs_M(pULBInput->fCrvUnCplLf - fCrvRefUnCplLf);
    fDevCrvUnCplRi = TUE_CML_Abs_M(pULBInput->fCrvUnCplRi - fCrvRefUnCplRi);
    fDevCrvDevUnCpl = TUE_CML_Abs_M(fDevCrvUnCplLf - fDevCrvUnCplRi);

    if ((fDevCrvDevUnCpl > pULBParam->fThdCrvDev) &&
        (bNotParallelUnCpl == 1U)) {
        bTrigCrvDevUnCpl = 1U;
    } else {
        bTrigCrvDevUnCpl = 0U;
    }

    /***************************5.Evaluate uncoupled lane
     * bridge***************************/
    if ((bNotParallelUnCpl == 1U) && (bTrigLatDistDev == 1U) &&
        (fDevLatDistCplLf <= fDevLatDistCplRi) && (bLastBridgeUnCplLf == 0U) &&
        (bLastBridgeUnCplRi == 0U) && (pULBInput->bDistYStepDtctLf == 0U) &&
        (pULBInput->bDistYStepDtctRi == 0U)) {
        bNotParallelBridgeUnCplLf = 1U;
    } else {
        bNotParallelBridgeUnCplLf = 0U;
    }

    /* Set flag for uncoupled left lane bridge */
    if ((bNotParallelBridgeUnCplLf == 1U) ||
        (pULBInput->uPercExitRi > fLastPercExitRi) ||
        (pULBInput->bLineMergeDtcLf) || (bStepDtctUnCplRi == 1U)) {
        bSetBridgeUnCplLf = 1U;
    } else {
        bSetBridgeUnCplLf = 0U;
    }

    /* Reset flag for uncoupled left lane bridge */
    bDlySetBridgeUnCplLf = TUE_CML_TurnOffDelay_M(
        bSetBridgeUnCplLf, 0.5F, pULBParam->fSysCycleTime, &fTimerSetUnCplLf,
        bLastDlySetBridgeUnCplLf);
    bResetByCrvUnCplLf = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pULBInput->fCrvUnCplLf), pULBParam->fMaxCrvBridgeUnCpl,
        pULBParam->fMinCrvBridgeUnCpl, bLastResetByCrvUnCplLf);
    if ((bDlySetBridgeUnCplLf == 0U) || (pULBInput->bLaneChangeDtct == 1U) ||
        (pULBInput->bUpDownHillDegrade == 1U) || (bResetByCrvUnCplLf == 1U)) {
        bResetBridgeUnCplLf = 1U;
    } else {
        bResetBridgeUnCplLf = 0U;
    }

    pULBOutput->bBridgeUnCplLf = TUE_CML_SRTrigger_M(
        bSetBridgeUnCplLf, bResetBridgeUnCplLf, bLastBridgeUnCplLf);

    if ((bNotParallelUnCpl == 1U) && (bTrigLatDistDev == 1U) &&
        (fDevLatDistCplLf > fDevLatDistCplRi) && (bLastBridgeUnCplLf == 0U) &&
        (bLastBridgeUnCplRi == 0U) && (pULBInput->bDistYStepDtctLf == 0U) &&
        (pULBInput->bDistYStepDtctRi == 0U)) {
        bNotParallelBridgeUnCplRi = 1U;
    } else {
        bNotParallelBridgeUnCplRi = 0U;
    }

    /* Set flag for uncoupled right lane bridge */
    if ((bNotParallelBridgeUnCplRi == 1U) ||
        (pULBInput->uPercExitLf > fLastPercExitLf) ||
        (pULBInput->bLineMergeDtcRi) || (bStepDtctUnCplLf == 1U)) {
        bSetBridgeUnCplRi = 1U;
    } else {
        bSetBridgeUnCplRi = 0U;
    }

    /* Reset flag for uncoupled right lane bridge */
    bDlySetBridgeUnCplRi = TUE_CML_TurnOffDelay_M(
        bSetBridgeUnCplRi, 0.5F, pULBParam->fSysCycleTime, &fTimerSetUnCplRi,
        bLastDlySetBridgeUnCplRi);
    bResetByCrvUnCplRi = TUE_CML_Hysteresis_M(
        TUE_CML_Abs_M(pULBInput->fCrvUnCplRi), pULBParam->fMaxCrvBridgeUnCpl,
        pULBParam->fMinCrvBridgeUnCpl, bLastResetByCrvUnCplRi);
    if ((bDlySetBridgeUnCplRi == 0U) || (pULBInput->bLaneChangeDtct == 1U) ||
        (pULBInput->bUpDownHillDegrade == 1U) || (bResetByCrvUnCplRi == 1U)) {
        bResetBridgeUnCplRi = 1U;
    } else {
        bResetBridgeUnCplRi = 0U;
    }

    pULBOutput->bBridgeUnCplRi = TUE_CML_SRTrigger_M(
        bSetBridgeUnCplRi, bResetBridgeUnCplRi, bLastBridgeUnCplRi);

    /***************************Output and
     * debug*******************************************/
    /* Uncoupled lane bridge bitfield */
    ULB_OutputBitfiled(bStepDtctUnCplLf, bStepDtctUnCplRi,
                       bNotParallelBridgeUnCplLf, bNotParallelBridgeUnCplRi,
                       bTrigHeadingDevUnCpl, bTrigCrvDevUnCpl, pULBOutput);
    pULBDebug->fRotPosYUnCplLf = fRotPosYUnCplLf;
    pULBDebug->fRotPosYUnCplRi = fRotPosYUnCplRi;
    pULBDebug->bRawNotParallelUnCpl = bRawNotParallelUnCpl;
    pULBDebug->bNotParallelUnCpl = bNotParallelUnCpl;

    pULBDebug->fDevLatDistCplLf = fDevLatDistCplLf;
    pULBDebug->fDevLatDistCplRi = fDevLatDistCplRi;
    pULBDebug->bTrigLatDistDev = bTrigLatDistDev;

    pULBDebug->bStepDtctUnCplLf = bStepDtctUnCplLf;
    pULBDebug->bStepDtctUnCplRi = bStepDtctUnCplRi;

    pULBDebug->fHeadingPolyUnCplLf = fHeadingPolyUnCplLf;
    pULBDebug->fHeadingPolyUnCplRi = fHeadingPolyUnCplRi;
    pULBDebug->fDevHeadingUnCplLf = fDevHeadingUnCplLf;
    pULBDebug->fDevHeadingUnCplRi = fDevHeadingUnCplRi;
    pULBDebug->fDevHeadingDevUnCpl = fDevHeadingDevUnCpl;
    pULBDebug->bTrigHeadingDevUnCpl = bTrigHeadingDevUnCpl;
    pULBDebug->fCrvRefUnCplLf = fCrvRefUnCplLf;
    pULBDebug->fCrvRefUnCplRi = fCrvRefUnCplRi;
    pULBDebug->fDevCrvUnCplLf = fDevCrvUnCplLf;
    pULBDebug->fDevCrvUnCplRi = fDevCrvUnCplRi;
    pULBDebug->fDevCrvDevUnCpl = fDevCrvDevUnCpl;
    pULBDebug->bTrigCrvDevUnCpl = bTrigCrvDevUnCpl;

    pULBDebug->bNotParallelBridgeUnCplLf = bNotParallelBridgeUnCplLf;
    pULBDebug->bSetBridgeUnCplLf = bSetBridgeUnCplLf;
    pULBDebug->bResetBridgeUnCplLf = bResetBridgeUnCplLf;
    pULBDebug->bDlySetBridgeUnCplLf = bDlySetBridgeUnCplLf;
    pULBDebug->fTimerSetUnCplLf = fTimerSetUnCplLf;
    pULBDebug->bResetByCrvUnCplLf = bResetByCrvUnCplLf;
    pULBDebug->bNotParallelBridgeUnCplRi = bNotParallelBridgeUnCplRi;
    pULBDebug->bSetBridgeUnCplRi = bSetBridgeUnCplRi;
    pULBDebug->bResetBridgeUnCplRi = bResetBridgeUnCplRi;
    pULBDebug->bDlySetBridgeUnCplRi = bDlySetBridgeUnCplRi;
    pULBDebug->fTimerSetUnCplRi = fTimerSetUnCplRi;
    pULBDebug->bResetByCrvUnCplRi = bResetByCrvUnCplRi;

    /***************************Save last
     * value********************************************/
    fLastCrvRefUnCplLf = fCrvRefUnCplLf;
    fLastCrvRefUnCplRi = fCrvRefUnCplRi;
    bLastBridgeUnCplLf = pULBOutput->bBridgeUnCplLf;
    bLastBridgeUnCplRi = pULBOutput->bBridgeUnCplRi;
    bLastDlySetBridgeUnCplLf = bDlySetBridgeUnCplLf;
    bLastDlySetBridgeUnCplRi = bDlySetBridgeUnCplRi;
    fLastPercExitLf = (REAL32_T)pULBInput->uPercExitLf;
    fLastPercExitRi = (REAL32_T)pULBInput->uPercExitRi;
    bLastResetByCrvUnCplLf = bResetByCrvUnCplLf;
    bLastResetByCrvUnCplRi = bResetByCrvUnCplRi;
    bLastRawNotParallelUnCpl = bRawNotParallelUnCpl;
}

/****************************************************************************************
        @fn           INPUT_DetermineLaneBoundary
        @brief        Input interface for DetermineLaneBoundary
        @description  Input interface for DetermineLaneBoundary:
                                          1.Input from external;
                                          2.Input from DMQ output;
                                          3.Input from LMC output;
                                          4.Input from ULB output.
        @param[in]    pULPInput : Input from ULP input
        @param[in]    pULPParam : Parameter from ULP parameter
        @param[in]    pDMQOutputCplLf: Output for coupled left lane
   DetermineMeasureQuality
        @param[in]    pDMQOutputCplRi: Output for coupled right lane
   DetermineMeasureQuality
        @param[in]    pLMCOutputCplLf: Output for coupled left lane
   LaneMotionCompensation
        @param[in]    pLMCOutputCplRi: Output for coupled right lane
   LaneMotionCompensation
        @param[in]    pULBOutput: Output from UncoupledLaneBridge
        @param[out]   pDMQInputCplRi: Input for coupled right lane
        @param[out]   pDMQParamCplRi: Parameter for coupled right lane
        @return       void
        @startuml
        title INPUT_DetermineLaneBoundary
        (*)--> 1.Input from external
           --> (*)
        (*)--> 2.Input from DMQ output
           --> (*)
        (*)--> 3.Input from LMC output
           --> (*)
        (*)--> 4.Input from ULB output
           --> (*)
        @enduml
        *****************************************************************************************/
void INPUT_DetermineLaneBoundary(const sULPInput_t *pULPInput,
                                 const sULPParam_t *pULPParam,
                                 const sDMQOutput_t *pDMQOutputCplLf,
                                 const sDMQOutput_t *pDMQOutputCplRi,
                                 const sLMCOutput_t *pLMCOutputCplLf,
                                 const sLMCOutput_t *pLMCOutputCplRi,
                                 const sULBOutput_t *pULBOutput,
                                 sDLBInput_t *pDLBInput,
                                 sDLBParam_t *pDLBParam) {
    // pDLBInput->fPosX0CplLf       = fPosX0CplLf;
    pDLBInput->fPosY0CplLf = pLMCOutputCplLf->fPosY0Cpmn;
    pDLBInput->fHeadingCplLf = pLMCOutputCplLf->fHeadingCpmn;
    pDLBInput->fCrvCplLf = pLMCOutputCplLf->fCrvCpmn;
    pDLBInput->fCrvRateCplLf = pLMCOutputCplLf->fCrvRateCpmn;
    pDLBInput->fValidLengthCplLf = pLMCOutputCplLf->fValidLengthCpmn;
    pDLBInput->fQualityCplLf = pULPInput->fQualityCplLf;
    pDLBInput->uLaneTypeCplLf = pLMCOutputCplLf->uMarkerTypeCpmn;

    pDLBInput->uEventTypeCplLf = pULPInput->uEventTypeCplLf;
    pDLBInput->bAvailableCplLf = pULPInput->bAvailableCplLf;
    pDLBInput->uEventQualityCplLf = pULPInput->uEventQualityCplLf;
    pDLBInput->fEventDistanceCplLf = pULPInput->fEventDistanceCplLf;
    pDLBInput->fStdDevPosY0CplLf = pULPInput->fStdDevPosY0CplLf;
    pDLBInput->fStdDevHeadingCplLf = pULPInput->fStdDevHeadingCplLf;
    pDLBInput->fStdDevCrvCplLf = pULPInput->fStdDevCrvCplLf;
    pDLBInput->fStdDevCrvRateCplLf = pULPInput->fStdDevCrvRateCplLf;
    pDLBInput->uColorCplLf = pLMCOutputCplLf->uColorCpmn;
    pDLBInput->fOverallQualityCplLf = pLMCOutputCplLf->fOverallQuality;

    pDLBInput->fPosY0CplRi = pLMCOutputCplRi->fPosY0Cpmn;
    pDLBInput->fHeadingCplRi = pLMCOutputCplRi->fHeadingCpmn;
    pDLBInput->fCrvCplRi = pLMCOutputCplRi->fCrvCpmn;
    pDLBInput->fCrvRateCplRi = pLMCOutputCplRi->fCrvRateCpmn;
    pDLBInput->fValidLengthCplRi = pLMCOutputCplRi->fValidLengthCpmn;
    pDLBInput->fQualityCplRi = pULPInput->fQualityCplRi;
    pDLBInput->uLaneTypeCplRi = pLMCOutputCplRi->uMarkerTypeCpmn;

    pDLBInput->uEventTypeCplRi = pULPInput->uEventTypeCplRi;
    pDLBInput->bAvailableCplRi = pULPInput->bAvailableCplRi;
    pDLBInput->uEventQualityCplRi = pULPInput->uEventQualityCplRi;
    pDLBInput->fEventDistanceCplRi = pULPInput->fEventDistanceCplRi;
    pDLBInput->fStdDevPosY0CplRi = pULPInput->fStdDevPosY0CplRi;
    pDLBInput->fStdDevHeadingCplRi = pULPInput->fStdDevHeadingCplRi;
    pDLBInput->fStdDevCrvCplRi = pULPInput->fStdDevCrvCplRi;
    pDLBInput->fStdDevCrvRateCplRi = pULPInput->fStdDevCrvRateCplRi;
    pDLBInput->uColorCplRi = pLMCOutputCplRi->uColorCpmn;
    pDLBInput->fOverallQualityCplRi = pLMCOutputCplRi->fOverallQuality;
}

/****************************************************************************************
        @fn           DetermineLaneBoundary
        @brief        Generate uncoupled lane boundary
        @description  Determine lane boundary:
                                          1.Determine left lane fading factor;
                                          2.Left lane weighting;
                                          3.Determine right lane fading factor;
                                          4.Right lane weighting.
        @param[in]    pDLBInput : Input for DetermineLaneBoundary
        @param[in]    pDLBParam : Parameter for DetermineLaneBoundary
        @param[out]   pDLBOutput: Output for DetermineLaneBoundary
        @param[out]   pDLBDebug : Debug(measurement) for DetermineLaneBoundary
        @return       void
        @startuml
        title INPUT_DetermineLaneBoundary
        (*)--> 1.Determine left lane fading factor
           --> (*)
        (*)--> 2.Left lane weighting
           --> (*)
        (*)--> 3.Determine right lane fading factor
           --> (*)
        (*)--> 4.Right lane weighting
           --> (*)
        @enduml
 ******************************************************************************************/
void DetermineLaneBoundary(const sDLBInput_t *pDLBInput,
                           const sDLBParam_t *pDLBParam,
                           sDLBOutput_t *pDLBOutput,
                           sDLBDebug_t *pDLBDebug) {
    REAL32_T fFacFadeLf = 0.0F;
    // REAL32_T fCoeffFadeFaceLf = 0.0F;

    REAL32_T fTempDLB = 0.0F;
    // REAL32_T fCoeffFadeFaceRi = 0.0F;

    /***************************1.Determine left lane fading
     * factor************************/
    if (pDLBInput->bBridgeUnCplLf == 1U) {
        pDLBOutput->fOverallQualityLf = pDLBInput->fOverallQualityUnCplLf;
        fFacFadeLf = 0.0F;
    } else if ((pDLBParam->bUseLaneDynWeight == 0U) ||
               (pDLBInput->bUpDownHillDegr == 1U)) {
        pDLBOutput->fOverallQualityLf = pDLBInput->fOverallQualityCplLf;
        fFacFadeLf = 1.0F;
    } else {
        if (pDLBInput->bBridgeUnCplRi == 1U) {
            pDLBOutput->fOverallQualityLf = 0.0F;
            fFacFadeLf = 0.5F;
        } else {
            fTempDLB = (pDLBInput->fOverallQualityCplLf -
                        pDLBInput->fOverallQualityUnCplLf);
            fTempDLB = fTempDLB / pDLBParam->fRefQualityFadeFac;
            fTempDLB = TUE_CML_Limit_M(fTempDLB, 1.0F, -1.0F);
            fFacFadeLf = fTempDLB * 0.5F + 0.5F;

            pDLBOutput->fOverallQualityLf =
                pDLBInput->fOverallQualityCplLf * (1.0F - fFacFadeLf) +
                pDLBInput->fOverallQualityUnCplLf * fFacFadeLf;
        }
    }
    /***************************2.Left lane
     * weighting**************************************/

    /***************************3.Determine right lane fading
     * factor***********************/

    /***************************4.Right lane
     * weighting*************************************/

    /***************************Output and
     * debug*******************************************/

    pDLBOutput->fPosX0Lf = 0.0F;
    pDLBOutput->fPosY0Lf = pDLBInput->fPosY0CplLf;
    pDLBOutput->fHeadingLf = pDLBInput->fHeadingCplLf;
    pDLBOutput->fCrvLf = pDLBInput->fCrvCplLf;
    pDLBOutput->fCrvRateLf = pDLBInput->fCrvRateCplLf;
    pDLBOutput->fValidLengthLf = pDLBInput->fValidLengthCplLf;
    pDLBOutput->fQualityLf = pDLBInput->fQualityCplLf;
    pDLBOutput->uLaneTypeLf = pDLBInput->uLaneTypeCplLf;

    pDLBOutput->uEventTypeLf = pDLBInput->uEventTypeCplLf;
    pDLBOutput->bAvailableLf = pDLBInput->bAvailableCplLf;
    pDLBOutput->uEventQualityLf = pDLBInput->uEventQualityCplLf;
    pDLBOutput->fEventDistanceLf = pDLBInput->fEventDistanceCplLf;
    pDLBOutput->fStdDevPosY0Lf = pDLBInput->fStdDevPosY0CplLf;
    pDLBOutput->fStdDevHeadingLf = pDLBInput->fStdDevHeadingCplLf;
    pDLBOutput->fStdDevCrvLf = pDLBInput->fStdDevCrvCplLf;
    pDLBOutput->fStdDevCrvRateLf = pDLBInput->fStdDevCrvRateCplLf;
    pDLBOutput->uColorLf = pDLBInput->uColorCplLf;
    pDLBOutput->fOverallQualityLf = pDLBInput->fOverallQualityCplLf;

    pDLBOutput->fPosX0Ri = 0.0F;
    pDLBOutput->fPosY0Ri = pDLBInput->fPosY0CplRi;
    pDLBOutput->fHeadingRi = pDLBInput->fHeadingCplRi;
    pDLBOutput->fCrvRi = pDLBInput->fCrvCplRi;
    pDLBOutput->fCrvRateRi = pDLBInput->fCrvRateCplRi;
    pDLBOutput->fValidLengthRi = pDLBInput->fValidLengthCplRi;
    pDLBOutput->fQualityRi = pDLBInput->fQualityCplRi;
    pDLBOutput->uLaneTypeRi = pDLBInput->uLaneTypeCplRi;

    pDLBOutput->uEventTypeRi = pDLBInput->uEventTypeCplRi;
    pDLBOutput->bAvailableRi = pDLBInput->bAvailableCplRi;
    pDLBOutput->uEventQualityRi = pDLBInput->uEventQualityCplRi;
    pDLBOutput->fEventDistanceRi = pDLBInput->fEventDistanceCplRi;
    pDLBOutput->fStdDevPosY0Ri = pDLBInput->fStdDevPosY0CplRi;
    pDLBOutput->fStdDevHeadingRi = pDLBInput->fStdDevHeadingCplRi;
    pDLBOutput->fStdDevCrvRi = pDLBInput->fStdDevCrvCplRi;
    pDLBOutput->fStdDevCrvRateRi = pDLBInput->fStdDevCrvRateCplRi;
    pDLBOutput->uColorRi = pDLBInput->uColorCplRi;
    pDLBOutput->fOverallQualityRi = pDLBInput->fOverallQualityCplRi;

    /****************************Save last
     * value************************************************/
}

/****************************************************************************************
        @fn           OUTPUT_UncoupledLaneProcess
        @brief        Output for UncoupledLaneProcess
        @description  Output for UncoupledLaneProcess:
                                          1.Output from DMQ;
                                          2.Output form LMC;
                                          3.Output from ULB;
                                          4.Output from DLB.
        @param[in]    pDMQOutputCplLf : Output from
 DetermineLaneBoundary(uncoupled left lane)
        @param[in]    pDMQDebugCplLf :  Debug from
 DetermineLaneBoundary(uncoupled left lane)
        @param[in]    pDMQOutputCplRi : Output from
 DetermineLaneBoundary(uncoupled right lane)
        @param[in]    pDMQDebugCplRi:   Debug from
 DetermineLaneBoundary(uncoupled right lane)
        @param[in]    pLMCOutputCplLf : Output from
 LaneMotionCompensation(uncoupled left lane)
        @param[in]    pLMCDebugCplLf :  Debug from
 LaneMotionCompensation(uncoupled left lane)
        @param[in]    pLMCOutputCplLf : Output from
 LaneMotionCompensation(uncoupled right lane)
        @param[in]    pLMCDebugCplLf :  Debug from
 LaneMotionCompensation(uncoupled right lane)
        @param[in]    pULBOutput : Input from UncoupledLaneBridge
        @param[in]    pULBDebug : Debug for UncoupledLaneBridge
        @param[in]    pDLBOutput : Input from DetermineLaneBoundary
        @param[in]    pDLBDebug : Debug for DetermineLaneBoundary
        @param[out]   pULPOutput: Output for OUTPUT_UncoupledLaneProcess
        @param[out]   pULPDebug : Debug(measurement) for
 OUTPUT_UncoupledLaneProcess
        @return       void
        @startuml
        title OUTPUT_UncoupledLaneProcess
        (*)--> 1.Output from DMQ
           --> (*)
        (*)--> 2.Output form LMC
           --> (*)
        (*)--> 3.Output from ULB
           --> (*)
        (*)--> 4.Output from DLB
           --> (*)
        @enduml
 ******************************************************************************************/
void OUTPUT_UncoupledLaneProcess(const sDMQOutput_t *pDMQOutputCplLf,
                                 const sDMQDebug_t *pDMQDebugCplLf,
                                 const sDMQOutput_t *pDMQOutputCplRi,
                                 const sDMQDebug_t *pDMQDebugCplRi,
                                 const sLMCOutput_t *pLMCOutputCplLf,
                                 const sLMCDebug_t *pLMCDebugCplLf,
                                 const sLMCOutput_t *pLMCOutputCplRi,
                                 const sLMCDebug_t *pLMCDebugCplRi,
                                 const sULBOutput_t *pULBOutput,
                                 const sULBDebug_t *pULBDebug,
                                 const sDLBOutput_t *pDLBOutput,
                                 const sDLBDebug_t *pDLBDebug,
                                 sULPOutput_t *pULPOutput,
                                 sULPDebug_t *pULPDebug) {
    pULPOutput->fPosX0Lf = pDLBOutput->fPosX0Lf;
    pULPOutput->fPosY0Lf = pDLBOutput->fPosY0Lf;
    pULPOutput->fHeadingLf = pDLBOutput->fHeadingLf;
    pULPOutput->fCrvLf = pDLBOutput->fCrvLf;
    pULPOutput->fCrvRateLf = pDLBOutput->fCrvRateLf;
    pULPOutput->fValidLengthLf = pDLBOutput->fValidLengthLf;
    pULPOutput->fQualityLf = pDLBOutput->fQualityLf;
    pULPOutput->uLaneTypeLf = pDLBOutput->uLaneTypeLf;

    pULPOutput->uEventTypeLf = pDLBOutput->uEventTypeLf;
    pULPOutput->bAvailableLf = pDLBOutput->bAvailableLf;
    pULPOutput->uEventQualityLf = pDLBOutput->uEventQualityLf;
    pULPOutput->fEventDistanceLf = pDLBOutput->fEventDistanceLf;
    pULPOutput->fStdDevPosY0Lf = pDLBOutput->fStdDevPosY0Lf;
    pULPOutput->fStdDevHeadingLf = pDLBOutput->fStdDevHeadingLf;
    pULPOutput->fStdDevCrvLf = pDLBOutput->fStdDevCrvLf;
    pULPOutput->fStdDevCrvRateLf = pDLBOutput->fStdDevCrvRateLf;
    pULPOutput->uColorLf = pDLBOutput->uColorLf;
    pULPOutput->fOverallQualityLf = pDLBOutput->fOverallQualityLf;
    pULPOutput->fOverallCrvQualityLf = pDMQOutputCplLf->fCrvQuality;

    pULPOutput->fPosX0Ri = pDLBOutput->fPosX0Ri;
    pULPOutput->fPosY0Ri = pDLBOutput->fPosY0Ri;
    pULPOutput->fHeadingRi = pDLBOutput->fHeadingRi;
    pULPOutput->fCrvRi = pDLBOutput->fCrvRi;
    pULPOutput->fCrvRateRi = pDLBOutput->fCrvRateRi;
    pULPOutput->fValidLengthRi = pDLBOutput->fValidLengthRi;
    pULPOutput->fQualityRi = pDLBOutput->fQualityRi;
    pULPOutput->uLaneTypeRi = pDLBOutput->uLaneTypeRi;

    pULPOutput->uEventTypeRi = pDLBOutput->uEventTypeRi;
    pULPOutput->bAvailableRi = pDLBOutput->bAvailableRi;
    pULPOutput->uEventQualityRi = pDLBOutput->uEventQualityRi;
    pULPOutput->fEventDistanceRi = pDLBOutput->fEventDistanceRi;
    pULPOutput->fStdDevPosY0Ri = pDLBOutput->fStdDevPosY0Ri;
    pULPOutput->fStdDevHeadingRi = pDLBOutput->fStdDevHeadingRi;
    pULPOutput->fStdDevCrvRi = pDLBOutput->fStdDevCrvRi;
    pULPOutput->fStdDevCrvRateRi = pDLBOutput->fStdDevCrvRateRi;
    pULPOutput->uColorRi = pDLBOutput->uColorRi;
    pULPOutput->fOverallQualityRi = pDLBOutput->fOverallQualityRi;
    pULPOutput->fOverallCrvQualityRi = pDMQOutputCplRi->fCrvQuality;

    /*  */
    pULPOutput->bBridgePossibleUnCplLf = pULBOutput->bBridgeUnCplLf;
    pULPOutput->bBridgePossibleUnCplRi = pULBOutput->bBridgeUnCplRi;
    pULPOutput->uBtfBridgeUnCpl = pULBOutput->uBtfBridgeUnCpl;
}

/****************************************************************************************
        @fn           UncoupledLaneProcessing
        @brief        The camera input lane line is processed to generate the
 raw lane
        @description  Uncoupled lane processing:
                                          1.Determine lane measurement quality;
                                          2.Lane motion compensation;
                                          3.Determine uncoupled lane bridge;
                                          4.Determine lane Boundary.
        @param[in]    pULPInput : Input for UncoupledLaneProcessing
        @param[in]    pULPParam : Parameter for UncoupledLaneProcessing
        @param[out]   pULPOutput: Output for UncoupledLaneProcessing
        @param[out]   pULPDebug : Debug(measurement) for UncoupledLaneProcessing
        @return       void
        @startuml
        title UncoupledLaneProcessing
        (*)--> 1.DetermineLaneMeasurementQuality
           --> 2.LaneMotionCompensation
        (*)--> 3.DetermineUncoupledLaneBridge
           --> 4.DetermineLaneBoundary
           --> OUTPUT_UncoupledLaneProcess
           --> (*)
           1.DetermineLaneMeasurementQuality --> 4.DetermineLaneBoundary
           2.LaneMotionCompensation --> 4.DetermineLaneBoundary
        @enduml
 ******************************************************************************************/
void UncoupledLaneProcessing(const sULPInput_t *pULPInput,
                             const sULPParam_t *pULPParam,
                             sULPOutput_t *pULPOutput,
                             sULPDebug_t *pULPDebug) {
    sDMQInput_t sDMQInputCplLf = {0};
    sDMQParam_t sDMQParamCplLf = {0};
    sDMQOutput_t sDMQOutputCplLf = {0};
    sDMQDebug_t sDMQDebugCplLf = {0};

    sDMQInput_t sDMQInputCplRi = {0};
    sDMQParam_t sDMQParamCplRi = {0};
    sDMQOutput_t sDMQOutputCplRi = {0};
    sDMQDebug_t sDMQDebugCplRi = {0};

    sLMCInput_t sLMCInputCplLf = {0};
    sLMCParam_t sLMCParamCplLf = {0};
    sLMCOutput_t sLMCOutputCplLf = {0};
    sLMCDebug_t sLMCDebugCplLf = {0};

    sLMCInput_t sLMCInputCplRi = {0};
    sLMCParam_t sLMCParamCplRi = {0};
    sLMCOutput_t sLMCOutputCplRi = {0};
    sLMCDebug_t sLMCDebugCplRi = {0};

    sULBInput_t sULBInput = {0};
    sULBParam_t sULBParam = {0};
    sULBOutput_t sULBOutput = {0};
    sULBDebug_t sULBDebug = {0};

    sDLBInput_t sDLBInput = {0};
    sDLBParam_t sDLBParam = {0};
    sDLBOutput_t sDLBOutput = {0};
    sDLBDebug_t sDLBDebug = {0};

    /**************************1.Determine lane measurement
     * quality**************************/
    /* Input for Determine lane measurement quality */
    INPUT_DetermineMeasureQuality(pULPInput, pULPParam, &sDMQInputCplLf,
                                  &sDMQParamCplLf, &sDMQInputCplRi,
                                  &sDMQParamCplRi);

    /* Determine left lane measurement quality */
    DetermineMeasureQuality(CPL_LF, &sDMQInputCplLf, &sDMQParamCplLf,
                            &sDMQOutputCplLf, &sDMQDebugCplLf);

    /* Determine right lane measurement quality */
    DetermineMeasureQuality(CPL_RI, &sDMQInputCplRi, &sDMQParamCplRi,
                            &sDMQOutputCplRi, &sDMQDebugCplRi);

    /***********************************2.Lane motion
     * compensation***************************/
    /* Input for lane motion compensation */
    INPUT_LaneMotionCompensation(
        pULPInput, pULPParam, &sDMQOutputCplLf, &sDMQOutputCplRi,
        &sLMCInputCplLf, &sLMCParamCplLf, &sLMCInputCplRi, &sLMCParamCplRi);

    /* Left lane motion compensation */
    LaneMotionCompensation(CPL_LF, &sLMCInputCplLf, &sLMCParamCplLf,
                           &sLMCOutputCplLf, &sLMCDebugCplLf);

    /* Right lane motion compensation */
    LaneMotionCompensation(CPL_RI, &sLMCInputCplRi, &sLMCParamCplRi,
                           &sLMCOutputCplRi, &sLMCDebugCplRi);

    /**********************3.Determine uncoupled lane
     * bridge********************************/
    INPUT_UncoupledLaneBridge(pULPInput, pULPParam, &sULBInput, &sULBParam);

    UncoupledLaneBridge(&sULBInput, &sULBParam, &sULBOutput, &sULBDebug);

    /**********************4.termine lane
     * Boundary******************************************/
    INPUT_DetermineLaneBoundary(pULPInput, pULPParam, &sDMQOutputCplLf,
                                &sDMQOutputCplRi, &sLMCOutputCplLf,
                                &sLMCOutputCplRi, &sULBOutput, &sDLBInput,
                                &sDLBParam);

    DetermineLaneBoundary(&sDLBInput, &sDLBParam, &sDLBOutput, &sDLBDebug);

    /**********************Output and
     * debug*************************************************/
    OUTPUT_UncoupledLaneProcess(
        &sDMQOutputCplLf, &sDMQDebugCplLf, &sDMQOutputCplRi, &sDMQDebugCplRi,
        &sLMCOutputCplLf, &sLMCDebugCplLf, &sLMCOutputCplRi, &sLMCDebugCplRi,
        &sULBOutput, &sULBDebug, &sDLBOutput, &sDLBDebug, pULPOutput,
        pULPDebug);

    TUE_CML_MemoryCopy_M((void *)&sDMQInputCplLf,
                         (void *)&(pULPDebug->sDMQInputCplLf),
                         sizeof(sDMQInput_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQParamCplLf,
                         (void *)&(pULPDebug->sDMQParamCplLf),
                         sizeof(sDMQParam_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQOutputCplLf,
                         (void *)&(pULPDebug->sDMQOutputCplLf),
                         sizeof(sDMQOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQDebugCplLf,
                         (void *)&(pULPDebug->sDMQDebugCplLf),
                         sizeof(sDMQDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sDMQInputCplRi,
                         (void *)&(pULPDebug->sDMQInputCplRi),
                         sizeof(sDMQInput_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQParamCplRi,
                         (void *)&(pULPDebug->sDMQParamCplRi),
                         sizeof(sDMQParam_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQOutputCplRi,
                         (void *)&(pULPDebug->sDMQOutputCplRi),
                         sizeof(sDMQOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sDMQDebugCplRi,
                         (void *)&(pULPDebug->sDMQDebugCplRi),
                         sizeof(sDMQDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sLMCInputCplLf,
                         (void *)&(pULPDebug->sLMCInputCplLf),
                         sizeof(sLMCInput_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCParamCplLf,
                         (void *)&(pULPDebug->sLMCParamCplLf),
                         sizeof(sLMCParam_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCOutputCplLf,
                         (void *)&(pULPDebug->sLMCOutputCplLf),
                         sizeof(sLMCOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCDebugCplLf,
                         (void *)&(pULPDebug->sLMCDebugCplLf),
                         sizeof(sLMCDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sLMCInputCplRi,
                         (void *)&(pULPDebug->sLMCInputCplRi),
                         sizeof(sLMCInput_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCParamCplRi,
                         (void *)&(pULPDebug->sLMCParamCplRi),
                         sizeof(sLMCParam_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCOutputCplRi,
                         (void *)&(pULPDebug->sLMCOutputCplRi),
                         sizeof(sLMCOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sLMCDebugCplRi,
                         (void *)&(pULPDebug->sLMCDebugCplRi),
                         sizeof(sLMCDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sULBInput, (void *)&(pULPDebug->sULBInput),
                         sizeof(sULBInput_t));
    TUE_CML_MemoryCopy_M((void *)&sULBParam, (void *)&(pULPDebug->sULBParam),
                         sizeof(sULBParam_t));
    TUE_CML_MemoryCopy_M((void *)&sULBOutput, (void *)&(pULPDebug->sULBOutput),
                         sizeof(sULBOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sULBDebug, (void *)&(pULPDebug->sULBDebug),
                         sizeof(sULBDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sDLBInput, (void *)&(pULPDebug->sDLBInput),
                         sizeof(sDLBInput_t));
    TUE_CML_MemoryCopy_M((void *)&sDLBParam, (void *)&(pULPDebug->sDLBParam),
                         sizeof(sDLBParam_t));
    TUE_CML_MemoryCopy_M((void *)&sDLBOutput, (void *)&(pULPDebug->sDLBOutput),
                         sizeof(sDLBOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sDLBDebug, (void *)&(pULPDebug->sDLBDebug),
                         sizeof(sDLBDebug_t));
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */