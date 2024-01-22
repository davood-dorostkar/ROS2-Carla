/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "LBP_EgoLaneGeneration.h"
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
// ESI_CalPosY
static REAL32_T fLastLateraVelLf = 0.0F;
static REAL32_T fLastLateraVelRi = 0.0F;
// ESI_CheckLeftSafetyLaneData
static UINT8_T bLastAvailableLf = 0U;
static REAL32_T fLastPosY0Lf = 0.0F;
static REAL32_T fLastHeadingLf = 0.0F;
static REAL32_T fLastCrvLf = 0.0F;
static UINT8_T bLastRawDistYStepFlipLf = 0U;
static UINT8_T bLastRawHeadingStepFlipLf = 0U;
static UINT8_T bLastRawCrvStepFlipLf = 0U;
static UINT8_T bLastDistYStepFlipLf = 0U;
static UINT8_T bLastHeadingStepFlipLf = 0U;
static UINT8_T bLastCrvStepFlipLf = 0U;
static UINT8_T bLastInValidByQltyLf = 0;
static UINT8_T bLastInValidByAllQltyLf = 0;
static REAL32_T fTimerDistYStepFlipLf = 0.0F;
static REAL32_T fTimerHeadingStepFlipLf = 0.0F;
static REAL32_T fTimerCrvStepFlipLf = 0.0F;
static REAL32_T fTimerInValidByQltyLf = 0.0F;

// EgoSafetyInterface
static UINT8_T bLastAvailableRi = 0U;
static REAL32_T fLastPosY0Ri = 0.0F;
static REAL32_T fLastHeadingRi = 0.0F;
static REAL32_T fLastCrvRi = 0.0F;
static UINT8_T bLastRawDistYStepFlipRi = 0U;
static UINT8_T bLastRawHeadingStepFlipRi = 0U;
static UINT8_T bLastRawCrvStepFlipRi = 0U;
static UINT8_T bLastDistYStepFlipRi = 0U;
static UINT8_T bLastHeadingStepFlipRi = 0U;
static UINT8_T bLastCrvStepFlipRi = 0U;
static UINT8_T bLastInValidByQltyRi = 0;
static UINT8_T bLastInValidByAllQltyRi = 0;
static REAL32_T fTimerDistYStepFlipRi = 0.0F;
static REAL32_T fTimerHeadingStepFlipRi = 0.0F;
static REAL32_T fTimerCrvStepFlipRi = 0.0F;
static REAL32_T fTimerInValidByQltyRi = 0.0F;

// ECI_DetermineLeftLaneQualifier
static REAL32_T fTimerVirtualLaneLf = 0.0F;
static UINT8_T bLastVirtualLaneLf = 0U;

// ECI_DetermineRightLaneQualifier
static UINT8_T bLastVirtualLaneRi = 0U;
static REAL32_T fTimerVirtualLaneRi = 0.0F;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void EgoLaneGenerationReset(void) {
    // ESI_CalPosY
    fLastLateraVelLf = 0.0F;
    fLastLateraVelRi = 0.0F;
    // ESI_CheckLeftSafetyLaneData
    bLastAvailableLf = 0U;
    fLastPosY0Lf = 0.0F;
    fLastHeadingLf = 0.0F;
    fLastCrvLf = 0.0F;
    bLastRawDistYStepFlipLf = 0U;
    bLastRawHeadingStepFlipLf = 0U;
    bLastRawCrvStepFlipLf = 0U;
    bLastDistYStepFlipLf = 0U;
    bLastHeadingStepFlipLf = 0U;
    bLastCrvStepFlipLf = 0U;
    bLastInValidByQltyLf = 0;
    bLastInValidByAllQltyLf = 0;
    fTimerDistYStepFlipLf = 0.0F;
    fTimerHeadingStepFlipLf = 0.0F;
    fTimerCrvStepFlipLf = 0.0F;
    fTimerInValidByQltyLf = 0.0F;

    // EgoSafetyInterface
    bLastAvailableRi = 0U;
    fLastPosY0Ri = 0.0F;
    fLastHeadingRi = 0.0F;
    fLastCrvRi = 0.0F;
    bLastRawDistYStepFlipRi = 0U;
    bLastRawHeadingStepFlipRi = 0U;
    bLastRawCrvStepFlipRi = 0U;
    bLastDistYStepFlipRi = 0U;
    bLastHeadingStepFlipRi = 0U;
    bLastCrvStepFlipRi = 0U;
    bLastInValidByQltyRi = 0;
    bLastInValidByAllQltyRi = 0;
    fTimerDistYStepFlipRi = 0.0F;
    fTimerHeadingStepFlipRi = 0.0F;
    fTimerCrvStepFlipRi = 0.0F;
    fTimerInValidByQltyRi = 0.0F;

    // ECI_DetermineLeftLaneQualifier
    fTimerVirtualLaneLf = 0.0F;
    bLastVirtualLaneLf = 0U;

    // ECI_DetermineRightLaneQualifier
    bLastVirtualLaneRi = 0U;
    fTimerVirtualLaneRi = 0.0F;
}

/***********************************4.Ego lane
 * generation************************************/
void INPUT_EgoLaneGeneration(const sLBPInput_t *pLBPInput,
                             const sLBPParam_t *pLBPParam,
                             const sULPOutput_t *pULPOutput,
                             const sCLPOutput_t *pCLPOutput,
                             const sLFPOutput_t *pLFPOutput,
                             sELGInput_t *pELGInput,
                             sELGParam_t *pELGParam) {
    /***********************************1.input***********************************/
    /***********************************1.1 EGI
     * input*****************************/
    pELGInput->bLaneChangeDtct = pCLPOutput->bLaneChangeDtct;
    pELGInput->uLaneTypeLf = pULPOutput->uLaneTypeLf;
    pELGInput->uLaneTypeRi = pULPOutput->uLaneTypeRi;
    pELGInput->bConstructionSiteDtct = pCLPOutput->bConstructionSiteDtct;
    pELGInput->fOverallQualityLf = pULPOutput->fOverallQualityLf;
    pELGInput->fOverallQualityRi = pULPOutput->fOverallQualityRi;
    pELGInput->fOverallCrvQualityLf = pULPOutput->fOverallCrvQualityLf;
    pELGInput->fOverallCrvQualityRi = pULPOutput->fOverallCrvQualityRi;
    pELGInput->fTstamp = pLBPInput->fTstamp;
    pELGInput->uRangeCheckQualifier = pCLPOutput->uRangeCheckQualifier;

    /***********************************1.2 ESI
     * input*****************************/
    pELGInput->fVehVelX = pLBPInput->fVehVelX;
    pELGInput->bCamStatusQualifierValid = pCLPOutput->bCamStatusQualifierValid;
    // pELGInput->bLaneChangeDtct          = pCLPInput->bLaneChangeDtct;
    pELGInput->bUpDownHillDegrade = pCLPOutput->bUpDownHillDegrade;

    pELGInput->fYawLf = pLFPOutput->fHeadingFltLf;
    pELGInput->bYawValidLf = pLFPOutput->bKalmanValidLf;
    pELGInput->fPosY0Lf = pULPOutput->fPosY0Lf;
    pELGInput->fHeadingLf = pULPOutput->fHeadingLf;
    pELGInput->fCrvLf = pULPOutput->fCrvLf;
    pELGInput->bAvailableLf = pULPOutput->bAvailableLf;
    pELGInput->uQualityLf = (UINT8_T)pULPOutput->fQualityLf;
    pELGInput->fOverallQualityLf = pULPOutput->fOverallQualityLf;

    pELGInput->fYawRi = pLFPOutput->fHeadingFltRi;
    pELGInput->bYawValidRi = pLFPOutput->bKalmanValidRi;
    pELGInput->fPosY0Ri = pULPOutput->fPosY0Ri;
    pELGInput->fHeadingRi = pULPOutput->fHeadingRi;
    pELGInput->fCrvRi = pULPOutput->fCrvRi;
    pELGInput->bAvailableRi = pULPOutput->bAvailableRi;
    pELGInput->uQualityRi = (UINT8_T)pULPOutput->fQualityRi;
    pELGInput->fOverallQualityRi = pULPOutput->fOverallQualityRi;
    pELGInput->bVituralLeft = pLBPInput->bVitural[0];
    pELGInput->bVituralRight = pLBPInput->bVitural[1];

    /***********************************1.3 ECI
     * input*****************************/
    // pELGInput->bCamStatusQualifierValid =
    // pCLPOutput->bCamStatusQualifierValid;
    pELGInput->bNotAvailableLf = pCLPOutput->bNotAvailableLf;
    pELGInput->bDistYStepDtctLf = pCLPOutput->bDistYStepDtctLf;
    pELGInput->bLengthInvalidLf = pCLPOutput->bLengthInvalidLf;

    pELGInput->bNotAvailableRi = pCLPOutput->bNotAvailableRi;
    pELGInput->bDistYStepDtctRi = pCLPOutput->bDistYStepDtctRi;
    pELGInput->bLengthInvalidRi = pCLPOutput->bLengthInvalidRi;

    pELGInput->uRangeCheckQualifier = pCLPOutput->uRangeCheckQualifier;
    pELGInput->bLaneTypeInvalidLf = pCLPOutput->bInValidMakerTypeLf;
    pELGInput->bLaneColorInvalidLf = pCLPOutput->bInValidLaneColorLf;

    pELGInput->bLaneQualityInvalidLf = pCLPOutput->bLnQualityInvalidLf;
    pELGInput->bLaneTypeInvalidRi = pCLPOutput->bInValidMakerTypeRi;
    pELGInput->bLaneColorInvalidRi = pCLPOutput->bInValidLaneColorRi;
    pELGInput->bLaneQualityInvalidRi = pCLPOutput->bLnQualityInvalidRi;

    pELGInput->fValidLengthLf = pULPOutput->fValidLengthLf;
    pELGInput->fValidLengthRi = pULPOutput->fValidLengthRi;
    // pELGInput->bLaneVirtualCplLf       = pCLPOutput->bLaneVirtualCplLf;
    // pELGInput->bLaneVirtualCplRi       = pCLPOutput->bLaneVirtualCplRi;
    if (pLBPInput->uQuality[CPL_LF] == 1U) {
        pELGInput->bLaneVirtualCplLf = 1U;
    } else {
        pELGInput->bLaneVirtualCplLf = 0U;
    }

    if (pLBPInput->uQuality[CPL_RI] == 1U) {
        pELGInput->bLaneVirtualCplRi = 1U;
    } else {
        pELGInput->bLaneVirtualCplRi = 0U;
    }

    pELGInput->uLaneValidQualifier = pLFPOutput->uLaneValidQualifier;
    pELGInput->uBridgePossible = pLFPOutput->uBridgePossible;
    pELGInput->bKalmanValidLf = pLFPOutput->bKalmanValidLf;
    pELGInput->bDistYStepDebouncedLf = pLFPOutput->bDistYStepDebouncedLf;
    pELGInput->bHeadingStepDebouncedLf = pLFPOutput->bHeadingStepDebouncedLf;
    pELGInput->bCrvStepDebouncedLf = pLFPOutput->bCrvStepDebouncedLf;

    pELGInput->bCrvRateStepDebouncedLf = pLFPOutput->bCrvRateStepDebouncedLf;
    pELGInput->bKalmanValidCntr = pLFPOutput->bKalmanValidCntr;
    pELGInput->bKalmanValidRi = pLFPOutput->bKalmanValidRi;
    pELGInput->bDistYStepDebouncedRi = pLFPOutput->bDistYStepDebouncedRi;
    pELGInput->bHeadingStepDebouncedRi = pLFPOutput->bHeadingStepDebouncedRi;

    pELGInput->bCrvStepDebouncedRi = pLFPOutput->bCrvStepDebouncedRi;
    pELGInput->bCrvRateStepDebouncedRi = pLFPOutput->bCrvRateStepDebouncedRi;

    pELGInput->bDistYStepDebouncedCntr = pLFPOutput->bDistYStepDebouncedCntr;
    pELGInput->bHeadingStepDebouncedCntr =
        pLFPOutput->bHeadingStepDebouncedCntr;
    pELGInput->bCrvStepDebouncedCntr = pLFPOutput->bCrvStepDebouncedCntr;
    pELGInput->bCrvRateStepDebouncedCntr =
        pLFPOutput->bCrvRateStepDebouncedCntr;

    pELGInput->fFltQualityCntr = pLFPOutput->fFltQualityCntr;
    pELGInput->uFltStatusCntr = pLFPOutput->uFltStatusCntr;

    pELGInput->fPosX0FltLf = pLFPOutput->fPosX0FltLf;
    pELGInput->fPosY0FltLf = pLFPOutput->fPosY0FltLf;
    pELGInput->fHeadingFltLf = pLFPOutput->fHeadingFltLf;
    pELGInput->fCrvFltLf = pLFPOutput->fCrvFltLf;
    pELGInput->fCrvRateFltLf = pLFPOutput->fCrvRateFltLf;
    pELGInput->fPosX0FltCntr = pLFPOutput->fPosX0FltCntr;
    pELGInput->fPosY0FltCntr = pLFPOutput->fPosY0FltCntr;
    pELGInput->fHeadingFltCntr = pLFPOutput->fHeadingFltCntr;
    pELGInput->fCrvFltCntr = pLFPOutput->fCrvFltCntr;
    pELGInput->fCrvRateFltCntr = pLFPOutput->fCrvRateFltCntr;
    pELGInput->fPosX0FltRi = pLFPOutput->fPosX0FltRi;
    pELGInput->fPosY0FltRi = pLFPOutput->fPosY0FltRi;
    pELGInput->fHeadingFltRi = pLFPOutput->fHeadingFltRi;
    pELGInput->fCrvFltRi = pLFPOutput->fCrvFltRi;
    pELGInput->fCrvRateFltRi = pLFPOutput->fCrvRateFltRi;

    pELGInput->fLaneWidth = pLFPOutput->fLaneWidth;

    pELGInput->bLineMergeDtcRi = pCLPOutput->bLineMergeDtcRi;
    pELGInput->bLineMergeDtcLf = pCLPOutput->bLineMergeDtcLf;

    // ***********************************2.parameter*******************************/
    pELGParam->fSysCycleTime = pLBPParam->fSysCycleTime;
}

/****************************************************************************************
        @fn           INPUT_EgoSafetyInterface
        @brief        Ego safety interface input
        @description  Ego safety interface input:
                                          1.ULP(UncoupledLaneProcessing);
                                          2.CLP(CheckBasicLaneBoundaryProperites);
                                          3.LFP(AnyBoundaryFilteringAndPlausibilization).
        @param[in]    pELGInput : Input for ELG
        @param[in]    pELGParam : Parameter for ELGe
        @param[out]   pELGOutput: Output for ELG
        @param[out]   pELGDebug : Debug(measurement) for ELG
        @return       void
        @startuml
        title EgoLaneGeneration
        (*)-->1.ULP(UncoupledLaneProcessing)
           --> (*)
        (*)-->2.CLP(CheckBasicLaneBoundaryProperites)
           -->(*)
        (*)-->3.LFP(AnyBoundaryFilteringAndPlausibilization)
           -->(*)
        @enduml
 ******************************************************************************************/
void INPUT_EgoSafetyInterface(const sELGInput_t *pELGInput,
                              const sELGParam_t *pELGParam,
                              sESIInput_t *pESIInput,
                              sESIParam_t *pESIParam) {
    /* ESI input */
    pESIInput->fVehVelX = pELGInput->fVehVelX;
    pESIInput->bCamStatusQualifierValid = pELGInput->bCamStatusQualifierValid;
    pESIInput->bLaneChangeDtct = pELGInput->bLaneChangeDtct;
    pESIInput->bUpDownHillDegrade = pELGInput->bUpDownHillDegrade;

    pESIInput->fYawLf = pELGInput->fYawLf;
    pESIInput->bYawValidLf = pELGInput->bYawValidLf;
    pESIInput->fPosY0Lf = pELGInput->fPosY0Lf;
    pESIInput->fHeadingLf = pELGInput->fHeadingLf;
    pESIInput->fCrvLf = pELGInput->fCrvLf;
    pESIInput->bAvailableLf = pELGInput->bAvailableLf;
    pESIInput->uQualityLf = pELGInput->uQualityLf;
    pESIInput->fOverallQualityLf = pELGInput->fOverallQualityLf;

    pESIInput->fYawRi = pELGInput->fYawRi;
    pESIInput->bYawValidRi = pELGInput->bYawValidRi;
    pESIInput->fPosY0Ri = pELGInput->fPosY0Ri;
    pESIInput->fHeadingRi = pELGInput->fHeadingRi;
    pESIInput->fCrvRi = pELGInput->fCrvRi;
    pESIInput->bAvailableRi = pELGInput->bAvailableRi;
    pESIInput->uQualityRi = pELGInput->uQualityRi;
    pESIInput->fOverallQualityRi = pELGInput->fOverallQualityRi;
    pESIInput->bVituralLeft = pELGInput->bVituralLeft;
    pESIInput->bVituralRight = pELGInput->bVituralRight;

    /* ESI parameter */
    pESIParam->bUseFilteredHeadingSafe = 1U;
    pESIParam->bUseLatencyCompSafe = 1U;
    pESIParam->bUseLowPassFilterSafe = 1U;
    pESIParam->fTimeLowPassFilterSafe = 0.2F;
    pESIParam->fLatencyTimeSafe = 0.2F;

    pESIParam->fDistYLimitStepDtct = 0.17F;  // 0.17
    pESIParam->fHeadLimStepDtct = 0.015F;
    pESIParam->fCrvLimStepDtct = 7.5E-4F;
    pESIParam->fJumpDebounceTimeSafe = 0.06F;
    pESIParam->fLDVirtualDelayTime = 0.06F;

    pESIParam->bUpDownDeactivatSafe = 0U;
    pESIParam->fSysCycleTime = pELGParam->fSysCycleTime;
}

void ESI_CalHeading(const sESIInput_t *pESIInput,
                    const sESIParam_t *pESIParam,
                    sESIOutput_t *pESIOutput) {
    if ((pESIInput->bYawValidLf == 1U) &&
        (pESIParam->bUseFilteredHeadingSafe == 1U)) {
        pESIOutput->fHeadingSafeLf = pESIInput->fYawLf;
    } else {
        pESIOutput->fHeadingSafeLf = pESIInput->fHeadingLf;
    }

    if ((pESIInput->bYawValidRi == 1U) &&
        (pESIParam->bUseFilteredHeadingSafe == 1U)) {
        pESIOutput->fHeadingSafeRi = pESIInput->fYawRi;
    } else {
        pESIOutput->fHeadingSafeRi = pESIInput->fHeadingRi;
    }

    pESIOutput->fHeadingSafeLf = pESIInput->fHeadingLf;
    pESIOutput->fHeadingSafeRi = pESIInput->fHeadingRi;
}

void ESI_CalPosY(const sESIInput_t *pESIInput,
                 const sESIParam_t *pESIParam,
                 sESIOutput_t *pESIOutput,
                 sESIDebug_t *pESIDebug) {
    REAL32_T fRawLateraVelLf = 0.0F;
    REAL32_T fLateraVelLf = 0.0F;
    REAL32_T fRawLateraVelRi = 0.0F;
    REAL32_T fLateraVelRi = 0.0F;
    REAL32_T fCoeffFltSafe =
        0.0F; /* TRUE: Activate lateral velocity PT1 filter for the latency
                 compensation, (0.2, 0~10, s) */
    if (pESIParam->bUseLatencyCompSafe == 0U) {
        pESIOutput->fPosY0SafeLf = pESIInput->fPosY0Lf;
        pESIOutput->fPosY0SafeRi = pESIInput->fPosY0Ri;
    } else {
        fRawLateraVelLf = pESIInput->fVehVelX * pESIOutput->fHeadingSafeLf;
        fRawLateraVelRi = pESIInput->fVehVelX * pESIOutput->fHeadingSafeRi;
        if (pESIParam->bUseLowPassFilterSafe == 1U) {
            fCoeffFltSafe = (pESIParam->fSysCycleTime) /
                            TUE_CML_Max_M(pESIParam->fTimeLowPassFilterSafe,
                                          pESIParam->fSysCycleTime);
            fLateraVelLf = fRawLateraVelLf * fCoeffFltSafe +
                           fLastLateraVelLf * (1.0F - fCoeffFltSafe);
            fLateraVelRi = fRawLateraVelRi * fCoeffFltSafe +
                           fLastLateraVelRi * (1.0F - fCoeffFltSafe);
        } else {
            fLateraVelLf = fRawLateraVelLf;
            fLateraVelRi = fRawLateraVelRi;
        }
        pESIOutput->fPosY0SafeLf =
            pESIInput->fPosY0Lf + fLateraVelLf * pESIParam->fLatencyTimeSafe;
        pESIOutput->fPosY0SafeRi =
            pESIInput->fPosY0Ri + fLateraVelRi * pESIParam->fLatencyTimeSafe;
    }
    pESIDebug->fRawLateraVelLf = fRawLateraVelLf;
    pESIDebug->fRawLateraVelRi = fRawLateraVelRi;
    pESIDebug->fCoeffFltSafe = fCoeffFltSafe;
    pESIDebug->fLateraVelLf = fLateraVelLf;
    pESIDebug->fLateraVelRi = fLateraVelRi;

    fLastLateraVelLf = fLateraVelLf;
    fLastLateraVelRi = fLateraVelRi;
}

void ESI_CheckLeftSafetyLaneData(const sESIInput_t *pESIInput,
                                 const sESIParam_t *pESIParam,
                                 sESIOutput_t *pESIOutput,
                                 sESIDebug_t *pESIDebug) {
    UINT8_T bSuppressLaneStepDtctLf = 0U;
    UINT8_T bDistYStepDtctLf = 0U;
    UINT8_T bHeadingStepDtctLf = 0U;
    UINT8_T bCrvStepDtctLf = 0U;
    UINT8_T bRawDistYStepFlipLf = 0U;
    UINT8_T bRawHeadingStepFlipLf = 0U;
    UINT8_T bRawCrvStepFlipLf = 0U;
    UINT8_T bDistYStepFlipLf = 0U;
    UINT8_T bHeadingStepFlipLf = 0U;
    UINT8_T bCrvStepFlipLf = 0U;
    UINT8_T bInValidByQltyLf = 0U;
    UINT8_T bInValidByAllQltyLf = 0U;
    UINT8_T uInvalidQualifierSafeLf = 0U;
    REAL32_T fAbsDistYStepLf = 0.0F;
    REAL32_T fAbsHeadingStepLf = 0.0F;
    REAL32_T fAbsCrvStepLf = 0.0F;
    UINT8_T bRstDistYStepFlipLf = 0U;
    UINT8_T bRstHeadingStepFlipLf = 0U;
    UINT8_T bRstCrvStepFlipLf = 0U;
    UINT8_T bSetByQualityLf = 0U;

    /* CheckLeftLaneStepDtctSuppression */
    if (((pESIInput->bAvailableLf == 1U) && (bLastAvailableLf == 0U)) ||
        (pESIInput->bAvailableLf == 0U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bSuppressLaneStepDtctLf = 1U;
    } else {
        bSuppressLaneStepDtctLf = 0U;
    }

    /* LeftLaneStepDetection */
    fAbsDistYStepLf = TUE_CML_Abs_M(pESIInput->fPosY0Lf - fLastPosY0Lf);
    fAbsHeadingStepLf = TUE_CML_Abs_M(pESIInput->fHeadingLf - fLastHeadingLf);
    fAbsCrvStepLf = TUE_CML_Abs_M(pESIInput->fCrvLf - fLastCrvLf);
    if ((bSuppressLaneStepDtctLf == 0U) &&
        (fAbsDistYStepLf > pESIParam->fDistYLimitStepDtct)) {
        bDistYStepDtctLf = 1U;
    } else {
        bDistYStepDtctLf = 0U;
    }

    if ((bSuppressLaneStepDtctLf == 0U) &&
        (fAbsHeadingStepLf > pESIParam->fHeadLimStepDtct)) {
        bHeadingStepDtctLf = 1U;
    } else {
        bHeadingStepDtctLf = 0U;
    }
    if ((bSuppressLaneStepDtctLf == 0U) &&
        (fAbsCrvStepLf > pESIParam->fCrvLimStepDtct)) {
        bCrvStepDtctLf = 1U;
    } else {
        bCrvStepDtctLf = 0U;
    }

    /* LeftDebounceStepDetections */
    bRawDistYStepFlipLf = TUE_CML_TurnOnDelay_M(
        (!bDistYStepDtctLf), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerDistYStepFlipLf,
        bLastRawDistYStepFlipLf);
    if ((bRawDistYStepFlipLf == 1U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bRstDistYStepFlipLf = 1U;
    } else {
        bRstDistYStepFlipLf = 0U;
    }
    bDistYStepFlipLf = TUE_CML_SRTrigger_M(
        bDistYStepDtctLf, bRstDistYStepFlipLf, bLastDistYStepFlipLf);
    if (bDistYStepFlipLf == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeLf, 0U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeLf, 0U);
    }

    bRawHeadingStepFlipLf = TUE_CML_TurnOnDelay_M(
        (!bHeadingStepDtctLf), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerHeadingStepFlipLf,
        bLastRawHeadingStepFlipLf);
    if ((bRawHeadingStepFlipLf == 1U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bRstHeadingStepFlipLf = 1U;
    } else {
        bRstHeadingStepFlipLf = 0U;
    }
    bHeadingStepFlipLf = TUE_CML_SRTrigger_M(
        bHeadingStepDtctLf, bRstHeadingStepFlipLf, bLastHeadingStepFlipLf);
    if (bHeadingStepFlipLf == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeLf, 1U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeLf, 1U);
    }

    bRawCrvStepFlipLf = TUE_CML_TurnOnDelay_M(
        (!bCrvStepDtctLf), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerCrvStepFlipLf, bLastRawCrvStepFlipLf);
    if ((bRawCrvStepFlipLf == 1U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bRstCrvStepFlipLf = 1U;
    } else {
        bRstCrvStepFlipLf = 0U;
    }
    bCrvStepFlipLf = TUE_CML_SRTrigger_M(bCrvStepDtctLf, bRstCrvStepFlipLf,
                                         bLastCrvStepFlipLf);
    if (bCrvStepFlipLf == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeLf, 2U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeLf, 2U);
    }

    /* CheckLaneBoundQuality */
    if ((pESIInput->uQualityLf == 1U || pESIInput->bVituralLeft == 1U)) {
        bSetByQualityLf = 1U;
    } else {
        bSetByQualityLf = 0U;
    }

    bInValidByQltyLf = TUE_CML_TurnOnDelay_M(
        bSetByQualityLf, pESIParam->fLDVirtualDelayTime,
        pESIParam->fSysCycleTime, &fTimerInValidByQltyLf, bLastInValidByQltyLf);
    bInValidByAllQltyLf = !(TUE_CML_Hysteresis_M(
        pESIInput->fOverallQualityLf, 40.0F, 29.0F, bLastInValidByAllQltyLf));
    if ((bInValidByQltyLf == 1U) || (bInValidByAllQltyLf == 1U)) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeLf, 3U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeLf, 3U);
    }
    // uInvalidQualifierSafeLf = 0u;

    /* SafetyInterfaceValidation */
    if ((pESIInput->bCamStatusQualifierValid == 1) &&
        ((pESIInput->bUpDownHillDegrade == 0) ||
         (pESIParam->bUpDownDeactivatSafe == 0))) {
        pESIOutput->uInvalidQualifierSafeLf = uInvalidQualifierSafeLf;
    } else {
        pESIOutput->uInvalidQualifierSafeLf = 0U;
    }

    pESIDebug->bSuppressLaneStepDtctLf = bSuppressLaneStepDtctLf;
    pESIDebug->fAbsDistYStepLf = fAbsDistYStepLf;
    pESIDebug->fAbsHeadingStepLf = fAbsHeadingStepLf;
    pESIDebug->fAbsCrvStepLf = fAbsCrvStepLf;
    pESIDebug->bDistYStepDtctLf = bDistYStepDtctLf;
    pESIDebug->bHeadingStepDtctLf = bHeadingStepDtctLf;
    pESIDebug->bRawDistYStepFlipLf = bRawDistYStepFlipLf;
    pESIDebug->fTimerDistYStepFlipLf = fTimerDistYStepFlipLf;
    pESIDebug->bRstDistYStepFlipLf = bRstDistYStepFlipLf;
    pESIDebug->bDistYStepFlipLf = bDistYStepFlipLf;
    pESIDebug->bRawHeadingStepFlipLf = bRawHeadingStepFlipLf;
    pESIDebug->fTimerHeadingStepFlipLf = fTimerHeadingStepFlipLf;
    pESIDebug->bRstHeadingStepFlipLf = bRstHeadingStepFlipLf;
    pESIDebug->bHeadingStepFlipLf = bHeadingStepFlipLf;
    pESIDebug->bRawCrvStepFlipLf = bRawCrvStepFlipLf;
    pESIDebug->fTimerCrvStepFlipLf = fTimerCrvStepFlipLf;
    pESIDebug->bRstCrvStepFlipLf = bRstCrvStepFlipLf;
    pESIDebug->bCrvStepFlipLf = bCrvStepFlipLf;
    pESIDebug->bSetByQualityLf = bSetByQualityLf;
    pESIDebug->fTimerInValidByQltyLf = fTimerInValidByQltyLf;
    pESIDebug->bInValidByQltyLf = bInValidByQltyLf;
    pESIDebug->bInValidByAllQltyLf = bInValidByAllQltyLf;

    bLastAvailableLf = pESIInput->bAvailableLf;
    fLastPosY0Lf = pESIInput->fPosY0Lf;
    fLastHeadingLf = pESIInput->fHeadingLf;
    fLastCrvLf = pESIInput->fCrvLf;
    bLastRawDistYStepFlipLf = bRawDistYStepFlipLf;
    bLastRawHeadingStepFlipLf = bRawHeadingStepFlipLf;
    bLastRawCrvStepFlipLf = bRawCrvStepFlipLf;
    bLastDistYStepFlipLf = bDistYStepFlipLf;
    bLastHeadingStepFlipLf = bHeadingStepFlipLf;
    bLastCrvStepFlipLf = bCrvStepFlipLf;
    bLastInValidByQltyLf = bInValidByQltyLf;
    bLastInValidByAllQltyLf = bInValidByAllQltyLf;
}

/****************************************************************************************
        @fn           EgoSafetyInterface
        @brief        Generate ego lane after uncoupled lane processing
        @description  Ego Lane Safety Interface:
                                        1.Calculate curvature;
                                        2.Calculate heading angle;
                                        3.Calculate PosY0;
                                        4.Safety Interface Validation:(1)
 Check Left Safety Lane data; (2) Check Right Safety Lane data.
        @param[in]    pESIInput : Input for ESI
        @param[in]    pESIParam : Parameter for ESI
        @param[in]    pESIOutput: Output for ESI
        @param[out]   pESIDebug : Debug(measurement) for ESI
        @return       void
        @startuml
        title EgoSafetyInterface
        (*)-->1.CalculateCurvature
          -->(*)
        (*)-->2.CalculateHeadingAngle
           -->3.CalculatePosY0
           --> (*)
        (*)-->4.SafetyInterfaceValidation
        2.CalculateHeadingAngle->4.SafetyInterfaceValidation
        -->(*)
        @enduml
 ******************************************************************************************/
void EgoSafetyInterface(const sESIInput_t *pESIInput,
                        const sESIParam_t *pESIParam,
                        sESIOutput_t *pESIOutput,
                        sESIDebug_t *pESIDebug) {
    UINT8_T bSuppressLaneStepDtctRi = 0U;
    UINT8_T bDistYStepDtctRi = 0U;
    UINT8_T bHeadingStepDtctRi = 0U;
    UINT8_T bCrvStepDtctRi = 0U;
    UINT8_T bRawDistYStepFlipRi = 0U;
    UINT8_T bRawHeadingStepFlipRi = 0U;
    UINT8_T bRawCrvStepFlipRi = 0U;
    UINT8_T bDistYStepFlipRi = 0U;
    UINT8_T bHeadingStepFlipRi = 0U;
    UINT8_T bCrvStepFlipRi = 0U;

    UINT8_T bInValidByQltyRi = 0U;
    UINT8_T bInValidByAllQltyRi = 0U;
    UINT8_T uInvalidQualifierSafeRi = 0U;
    REAL32_T fAbsDistYStepRi = 0.0F;
    REAL32_T fAbsHeadingStepRi = 0.0F;
    REAL32_T fAbsCrvStepRi = 0.0F;
    UINT8_T bRstDistYStepFlipRi = 0U;
    UINT8_T bRstHeadingStepFlipRi = 0U;
    UINT8_T bRstCrvStepFlipRi = 0U;
    UINT8_T bSetByQualityRi = 0U;

    /****************************************1.Calculate
     * curvature************************************/
    pESIOutput->fCrvSafeLf = pESIInput->fCrvLf;
    pESIOutput->fCrvSafeRi = pESIInput->fCrvRi;

    /*****************************************2.Calculate heading
     * angle********************************/
    ESI_CalHeading(pESIInput, pESIParam, pESIOutput);

    /******************************************3.Calculate
     * PosY0***************************************/
    ESI_CalPosY(pESIInput, pESIParam, pESIOutput, pESIDebug);

    /***********************************************4.Safety Interface
     * Validation*******************************/
    /* (1) Check Left Safety Lane data */

    ESI_CheckLeftSafetyLaneData(pESIInput, pESIParam, pESIOutput, pESIDebug);

    /* (2) Check right Safety Lane data */
    if (((pESIInput->bAvailableRi == 1U) && (bLastAvailableRi == 0U)) ||
        (pESIInput->bAvailableRi == 0U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bSuppressLaneStepDtctRi = 1U;
    } else {
        bSuppressLaneStepDtctRi = 0U;
    }

    /* LeftLaneStepDetection */
    fAbsDistYStepRi = TUE_CML_Abs_M(pESIInput->fPosY0Ri - fLastPosY0Ri);
    fAbsHeadingStepRi = TUE_CML_Abs_M(pESIInput->fHeadingRi - fLastHeadingRi);
    fAbsCrvStepRi = TUE_CML_Abs_M(pESIInput->fCrvRi - fLastCrvRi);
    if ((bSuppressLaneStepDtctRi == 0U) &&
        (fAbsDistYStepRi > pESIParam->fDistYLimitStepDtct)) {
        bDistYStepDtctRi = 1U;
    } else {
        bDistYStepDtctRi = 0U;
    }
    if ((bSuppressLaneStepDtctRi == 0U) &&
        (fAbsHeadingStepRi > pESIParam->fHeadLimStepDtct)) {
        bHeadingStepDtctRi = 1U;
    } else {
        bHeadingStepDtctRi = 0U;
    }
    if ((bSuppressLaneStepDtctRi == 0U) &&
        (fAbsCrvStepRi > pESIParam->fCrvLimStepDtct)) {
        bCrvStepDtctRi = 1U;
    } else {
        bCrvStepDtctRi = 0U;
    }

    /* LeftDebounceStepDetections */
    bRawDistYStepFlipRi = TUE_CML_TurnOnDelay_M(
        (!bDistYStepDtctRi), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerDistYStepFlipRi,
        bLastRawDistYStepFlipRi);
    if ((bRawDistYStepFlipRi == 1) || (pESIInput->bLaneChangeDtct == 1)) {
        bRstDistYStepFlipRi = 1U;
    } else {
        bRstDistYStepFlipRi = 0U;
    }
    bDistYStepFlipRi = TUE_CML_SRTrigger_M(
        bDistYStepDtctRi, bRstDistYStepFlipRi, bLastDistYStepFlipRi);
    if (bDistYStepFlipRi == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeRi, 0U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeRi, 0U);
        ;
    }

    bRawHeadingStepFlipRi = TUE_CML_TurnOnDelay_M(
        (!bHeadingStepDtctRi), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerHeadingStepFlipRi,
        bLastRawHeadingStepFlipRi);
    if ((bRawHeadingStepFlipRi == 1U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bRstHeadingStepFlipRi = 1U;
    } else {
        bRstHeadingStepFlipRi = 0U;
    }
    bHeadingStepFlipRi = TUE_CML_SRTrigger_M(
        bHeadingStepDtctRi, bRstHeadingStepFlipRi, bLastHeadingStepFlipRi);
    if (bHeadingStepFlipRi == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeRi, 1U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeRi, 1U);
    }

    bRawCrvStepFlipRi = TUE_CML_TurnOnDelay_M(
        (!bCrvStepDtctRi), pESIParam->fJumpDebounceTimeSafe,
        pESIParam->fSysCycleTime, &fTimerCrvStepFlipRi, bLastRawCrvStepFlipRi);
    if ((bRawHeadingStepFlipRi == 1U) || (pESIInput->bLaneChangeDtct == 1U)) {
        bRstCrvStepFlipRi = 1U;
    } else {
        bRstCrvStepFlipRi = 0U;
    }
    bCrvStepFlipRi = TUE_CML_SRTrigger_M(bCrvStepDtctRi, bRstCrvStepFlipRi,
                                         bLastCrvStepFlipRi);
    if (bCrvStepFlipRi == 1U) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeRi, 2U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeRi, 2U);
    }

    /* CheckLaneBoundQuality */
    if (pESIInput->uQualityRi == 1U || pESIInput->bVituralRight == 1U) {
        bSetByQualityRi = 1U;
    } else {
        bSetByQualityRi = 0U;
    }
    bInValidByQltyRi = TUE_CML_TurnOnDelay_M(
        bSetByQualityRi, pESIParam->fLDVirtualDelayTime,
        pESIParam->fSysCycleTime, &fTimerInValidByQltyRi, bLastInValidByQltyRi);
    bInValidByAllQltyRi = !(TUE_CML_Hysteresis_M(
        pESIInput->fOverallQualityRi, 40.0F, 29.0F, bLastInValidByAllQltyRi));
    if ((bInValidByQltyRi == 1U) || (bInValidByAllQltyRi == 1U)) {
        TUE_CML_Setbit_M(uInvalidQualifierSafeRi, 3U);
    } else {
        TUE_CML_Clrbit_M(uInvalidQualifierSafeRi, 3U);
    }
    // uInvalidQualifierSafeRi = 0u;

    /* SafetyInterfaceValidation */
    if ((pESIInput->bCamStatusQualifierValid == 1) &&
        ((pESIInput->bUpDownHillDegrade == 0) ||
         (pESIParam->bUpDownDeactivatSafe == 0))) {
        pESIOutput->uInvalidQualifierSafeRi = uInvalidQualifierSafeRi;
    } else {
        pESIOutput->uInvalidQualifierSafeRi = 0U;
    }

    /******************************************************Output and
     * debug**************************************/

    pESIDebug->bSuppressLaneStepDtctRi = bSuppressLaneStepDtctRi;
    pESIDebug->fAbsDistYStepRi = fAbsDistYStepRi;
    pESIDebug->fAbsHeadingStepRi = fAbsHeadingStepRi;
    pESIDebug->fAbsCrvStepRi = fAbsCrvStepRi;
    pESIDebug->bDistYStepDtctRi = bDistYStepDtctRi;
    pESIDebug->bHeadingStepDtctRi = bHeadingStepDtctRi;
    pESIDebug->bRawDistYStepFlipRi = bRawDistYStepFlipRi;
    pESIDebug->fTimerDistYStepFlipRi = fTimerDistYStepFlipRi;
    pESIDebug->bRstDistYStepFlipRi = bRstDistYStepFlipRi;
    pESIDebug->bDistYStepFlipRi = bDistYStepFlipRi;
    pESIDebug->bRawHeadingStepFlipRi = bRawHeadingStepFlipRi;
    pESIDebug->fTimerHeadingStepFlipRi = fTimerHeadingStepFlipRi;
    pESIDebug->bRstHeadingStepFlipRi = bRstHeadingStepFlipRi;
    pESIDebug->bHeadingStepFlipRi = bHeadingStepFlipRi;
    pESIDebug->bRawCrvStepFlipRi = bRawCrvStepFlipRi;
    pESIDebug->fTimerCrvStepFlipRi = fTimerCrvStepFlipRi;
    pESIDebug->bRstCrvStepFlipRi = bRstCrvStepFlipRi;
    pESIDebug->bCrvStepFlipRi = bCrvStepFlipRi;
    pESIDebug->bSetByQualityRi = bSetByQualityRi;
    pESIDebug->fTimerInValidByQltyRi = fTimerInValidByQltyRi;
    pESIDebug->bInValidByQltyRi = bInValidByQltyRi;
    pESIDebug->bInValidByAllQltyRi = bInValidByAllQltyRi;

    /******************************************************Save last
     * value****************************************/
    bLastAvailableRi = pESIInput->bAvailableRi;
    fLastPosY0Ri = pESIInput->fPosY0Ri;
    fLastHeadingRi = pESIInput->fHeadingRi;
    fLastCrvRi = pESIInput->fCrvRi;
    bLastRawDistYStepFlipRi = bRawDistYStepFlipRi;
    bLastRawHeadingStepFlipRi = bRawHeadingStepFlipRi;
    bLastRawCrvStepFlipRi = bRawCrvStepFlipRi;
    bLastDistYStepFlipRi = bDistYStepFlipRi;
    bLastHeadingStepFlipRi = bHeadingStepFlipRi;
    bLastCrvStepFlipRi = bCrvStepFlipRi;
    bLastInValidByQltyRi = bInValidByQltyRi;
    bLastInValidByAllQltyRi = bInValidByAllQltyRi;
} /* EgoLaneSafetyInterface */

/****************************************************************************************
        @fn           INPUT_EgoControlInterface
        @brief        EgoControlInterface input interface
        @description  EgoControlInterface input interface:
        @param[in]    pELGInput : Input from ELG input
        @param[in]    pELGParam : Parameter from ELG parameter
        @param[in]    pECIInput: Output for EgoControlInterface
        @param[out]   pECIParam : Debug(measurement) for EgoControlInterface
        @return       void
        @startuml
        title EgoSafetyInterface
        (*)-->1.pELGInput
          -->(*)
        (*)-->2.pELGParam
           --> (*)
        @enduml
 ******************************************************************************************/
void INPUT_EgoControlInterface(const sELGInput_t *pELGInput,
                               const sELGParam_t *pELGParam,
                               sECIInput_t *pECIInput,
                               sECIParam_t *pECIParam) {
    /* ECI input */
    pECIInput->bCamStatusQualifierValid = pELGInput->bCamStatusQualifierValid;
    pECIInput->bNotAvailableLf = pELGInput->bNotAvailableLf;
    pECIInput->bDistYStepDtctLf = pELGInput->bDistYStepDtctLf;
    pECIInput->bLengthInvalidLf = pELGInput->bLengthInvalidLf;
    pECIInput->bNotAvailableRi = pELGInput->bNotAvailableRi;

    pECIInput->bDistYStepDtctRi = pELGInput->bDistYStepDtctRi;
    pECIInput->bLengthInvalidRi = pELGInput->bLengthInvalidRi;
    pECIInput->uRangeCheckQualifier = pELGInput->uRangeCheckQualifier;
    pECIInput->bLaneTypeInvalidLf = pELGInput->bLaneTypeInvalidLf;
    pECIInput->bLaneColorInvalidLf = pELGInput->bLaneColorInvalidLf;

    pECIInput->bLaneQualityInvalidLf = pELGInput->bLaneQualityInvalidLf;
    pECIInput->bLaneTypeInvalidRi = pELGInput->bLaneTypeInvalidRi;
    pECIInput->bLaneColorInvalidRi = pELGInput->bLaneColorInvalidRi;
    pECIInput->bLaneQualityInvalidRi = pELGInput->bLaneQualityInvalidRi;
    pECIInput->fValidLengthLf = pELGInput->fValidLengthLf;

    pECIInput->fValidLengthRi = pELGInput->fValidLengthRi;
    pECIInput->bLaneVirtualCplLf = pELGInput->bLaneVirtualCplLf;
    pECIInput->bLaneVirtualCplRi = pELGInput->bLaneVirtualCplRi;
    pECIInput->uLaneValidQualifier = pELGInput->uLaneValidQualifier;

    pECIInput->uBridgePossible = pELGInput->uBridgePossible;
    pECIInput->bKalmanValidLf = pELGInput->bKalmanValidLf;
    pECIInput->bDistYStepDebouncedLf = pELGInput->bDistYStepDebouncedLf;
    pECIInput->bHeadingStepDebouncedLf = pELGInput->bHeadingStepDebouncedLf;
    pECIInput->bCrvStepDebouncedLf = pELGInput->bCrvStepDebouncedLf;

    pECIInput->bCrvRateStepDebouncedLf = pELGInput->bCrvRateStepDebouncedLf;
    pECIInput->bKalmanValidCntr = pELGInput->bKalmanValidCntr;
    pECIInput->bKalmanValidRi = pELGInput->bKalmanValidRi;
    pECIInput->bDistYStepDebouncedRi = pELGInput->bDistYStepDebouncedRi;
    pECIInput->bHeadingStepDebouncedRi = pELGInput->bHeadingStepDebouncedRi;

    pECIInput->bCrvStepDebouncedRi = pELGInput->bCrvStepDebouncedRi;
    pECIInput->bCrvRateStepDebouncedRi = pELGInput->bCrvRateStepDebouncedRi;
    pECIInput->fFltQualityCntr = pELGInput->fFltQualityCntr;
    pECIInput->uFltStatusCntr = pELGInput->uFltStatusCntr;

    pECIInput->bDistYStepDebouncedCntr = pELGInput->bDistYStepDebouncedCntr;
    pECIInput->bHeadingStepDebouncedCntr = pELGInput->bHeadingStepDebouncedCntr;
    pECIInput->bCrvStepDebouncedCntr = pELGInput->bCrvStepDebouncedCntr;
    pECIInput->bCrvRateStepDebouncedCntr = pELGInput->bCrvRateStepDebouncedCntr;

    pECIInput->fPosX0FltLf = pELGInput->fPosX0FltLf;
    pECIInput->fPosY0FltLf = pELGInput->fPosY0FltLf;
    pECIInput->fHeadingFltLf = pELGInput->fHeadingFltLf;
    pECIInput->fCrvFltLf = pELGInput->fCrvFltLf;
    pECIInput->fCrvRateFltLf = pELGInput->fCrvRateFltLf;

    pECIInput->fPosX0FltCntr = pELGInput->fPosX0FltCntr;
    pECIInput->fPosY0FltCntr = pELGInput->fPosY0FltCntr;
    pECIInput->fHeadingFltCntr = pELGInput->fHeadingFltCntr;
    pECIInput->fCrvFltCntr = pELGInput->fCrvFltCntr;
    pECIInput->fCrvRateFltCntr = pELGInput->fCrvRateFltCntr;

    pECIInput->fPosX0FltRi = pELGInput->fPosX0FltRi;
    pECIInput->fPosY0FltRi = pELGInput->fPosY0FltRi;
    pECIInput->fHeadingFltRi = pELGInput->fHeadingFltRi;
    pECIInput->fCrvFltRi = pELGInput->fCrvFltRi;
    pECIInput->fCrvRateFltRi = pELGInput->fCrvRateFltRi;

    pECIInput->fLaneWidth = pELGInput->fLaneWidth;

    pECIInput->bLineMergeDtcRi = pELGInput->bLineMergeDtcRi;
    pECIInput->bLineMergeDtcLf = pELGInput->bLineMergeDtcLf;

    /* ECI parameter */
    pECIParam->fDefaultLaneWidth = 2.5F;
    pECIParam->fMaxDistYRange = 6.0F;
    pECIParam->fMaxHeadingRange = 0.5F;
    pECIParam->fMaxCrvRange = 0.02F;
    pECIParam->fMaxCrvRateRange = 0.001F;
    pECIParam->fTimeDelayVirtulLane = 0.06F;
    pECIParam->fSysCycleTime = pELGParam->fSysCycleTime;
}

UINT8_T ECI_bValidBoth(UINT8_T uLaneValidQualifier, UINT8_T uBridgePossible) {
    if (uLaneValidQualifier == LANE_BOTH_VALID) {
        return 1U;
    } else if ((uLaneValidQualifier == LANE_LEFT_VIRTUAL) &&
               (uBridgePossible == 1U)) {
        return 1U;
    } else if ((uLaneValidQualifier == LANE_RIGHT_VIRTUAL) &&
               (uBridgePossible == 1U)) {
        return 1U;
    } else {
        return 0U;
    }
}

void ECI_EgoLaneBoundariesOut(UINT8_T bValidBoth,
                              UINT8_T bValidOnlyLf,
                              UINT8_T bValidOnlyRi,
                              const sECIInput_t *pECIInput,
                              const sECIParam_t *pECIParam,
                              sECIOutput_t *pECIOutput) {
    if (bValidBoth == 1U) {
        /* Determine left boundary */
        pECIOutput->fPosX0CtrlLf = 0.0F;
        pECIOutput->fPosY0CtrlLf =
            pECIInput->fPosY0FltCntr + pECIInput->fLaneWidth / 2.0F;
        pECIOutput->fHeadingCtrlLf = pECIInput->fHeadingFltCntr;
        pECIOutput->fCrvCtrlLf = pECIInput->fCrvFltCntr;
        pECIOutput->fCrvRateCtrlLf = pECIInput->fCrvRateFltCntr;
        if (pECIInput->uLaneValidQualifier == LANE_LEFT_VIRTUAL) {
            pECIOutput->fValidLengthCtrlLf = pECIInput->fValidLengthRi;
        } else {
            pECIOutput->fValidLengthCtrlLf = pECIInput->fValidLengthLf;
        }

        /* Determine center boundary */
        pECIOutput->fPosX0CtrlCntr = 0.0F;
        pECIOutput->fPosY0CtrlCntr = pECIInput->fPosY0FltCntr;
        pECIOutput->fHeadingCtrlCntr = pECIInput->fHeadingFltCntr;
        pECIOutput->fCrvCtrlCntr = pECIInput->fCrvFltCntr;
        pECIOutput->fCrvRateCtrlCntr = pECIInput->fCrvRateFltCntr;
        if (pECIInput->uLaneValidQualifier == LANE_BOTH_VALID) {
            pECIOutput->fValidLengthCtrlCntr = TUE_CML_Max_M(
                pECIInput->fValidLengthLf, pECIInput->fValidLengthRi);
        } else if (pECIInput->uLaneValidQualifier == LANE_LEFT_VIRTUAL) {
            pECIOutput->fValidLengthCtrlCntr = pECIInput->fValidLengthRi;
        } else {
            pECIOutput->fValidLengthCtrlCntr = pECIInput->fValidLengthLf;
        }

        /* Determine right boundary */
        pECIOutput->fPosX0CtrlRi = 0.0F;
        pECIOutput->fPosY0CtrlRi =
            pECIInput->fPosY0FltCntr - pECIInput->fLaneWidth / 2.0F;
        pECIOutput->fHeadingCtrlRi = pECIInput->fHeadingFltCntr;
        pECIOutput->fCrvCtrlRi = pECIInput->fCrvFltCntr;
        pECIOutput->fCrvRateCtrlRi = pECIInput->fCrvRateFltCntr;
        if (pECIInput->uLaneValidQualifier == LANE_RIGHT_VIRTUAL) {
            pECIOutput->fValidLengthCtrlRi = pECIInput->fValidLengthLf;
        } else {
            pECIOutput->fValidLengthCtrlRi = pECIInput->fValidLengthRi;
        }
    } else if (bValidOnlyLf == 1U) {
        /* Determine left boundary length */
        pECIOutput->fPosX0CtrlLf = 0.0F;
        pECIOutput->fPosY0CtrlLf = pECIInput->fPosY0FltLf;
        pECIOutput->fHeadingCtrlLf = pECIInput->fHeadingFltLf;
        pECIOutput->fCrvCtrlLf = pECIInput->fCrvFltLf;
        pECIOutput->fCrvRateCtrlLf = pECIInput->fCrvRateFltLf;
        pECIOutput->fValidLengthCtrlLf = pECIInput->fValidLengthLf;

        /* Determine center boundary */
        pECIOutput->fPosX0CtrlCntr = 0.0F;
        pECIOutput->fPosY0CtrlCntr =
            pECIInput->fPosY0FltLf - pECIParam->fDefaultLaneWidth / 2.0F;
        pECIOutput->fHeadingCtrlCntr = pECIInput->fHeadingFltLf;
        pECIOutput->fCrvCtrlCntr = pECIInput->fCrvFltLf;
        pECIOutput->fCrvRateCtrlCntr = pECIInput->fCrvRateFltLf;
        pECIOutput->fValidLengthCtrlCntr = pECIInput->fValidLengthLf;

        /* Determine right boundary */
        pECIOutput->fPosX0CtrlRi = 0.0F;
        pECIOutput->fPosY0CtrlRi =
            pECIInput->fPosY0FltLf - pECIParam->fDefaultLaneWidth;
        pECIOutput->fHeadingCtrlRi = pECIInput->fHeadingFltLf;
        pECIOutput->fCrvCtrlRi = pECIInput->fCrvFltLf;
        pECIOutput->fCrvRateCtrlRi = pECIInput->fCrvRateFltLf;
        pECIOutput->fValidLengthCtrlRi = pECIInput->fValidLengthLf;
    } else if (bValidOnlyRi == 1U) {
        /* Determine left boundary length */
        pECIOutput->fPosX0CtrlLf = 0.0F;
        pECIOutput->fPosY0CtrlLf =
            pECIInput->fPosY0FltRi + pECIParam->fDefaultLaneWidth;
        pECIOutput->fHeadingCtrlLf = pECIInput->fHeadingFltRi;
        pECIOutput->fCrvCtrlLf = pECIInput->fCrvFltRi;
        pECIOutput->fCrvRateCtrlLf = pECIInput->fCrvRateFltRi;
        pECIOutput->fValidLengthCtrlLf = pECIInput->fValidLengthRi;

        /* Determine center boundary */
        pECIOutput->fPosX0CtrlCntr = 0.0F;
        pECIOutput->fPosY0CtrlCntr =
            pECIInput->fPosY0FltRi + pECIParam->fDefaultLaneWidth / 2.0F;
        pECIOutput->fHeadingCtrlCntr = pECIInput->fHeadingFltRi;
        pECIOutput->fCrvCtrlCntr = pECIInput->fCrvFltRi;
        pECIOutput->fCrvRateCtrlCntr = pECIInput->fCrvRateFltRi;
        pECIOutput->fValidLengthCtrlCntr = pECIInput->fValidLengthRi;

        /* Determine right boundary */
        pECIOutput->fPosX0CtrlRi = 0.0F;
        pECIOutput->fPosY0CtrlRi = pECIInput->fPosY0FltRi;
        pECIOutput->fHeadingCtrlRi = pECIInput->fHeadingFltRi;
        pECIOutput->fCrvCtrlRi = pECIInput->fCrvFltRi;
        pECIOutput->fCrvRateCtrlRi = pECIInput->fCrvRateFltRi;
        pECIOutput->fValidLengthCtrlRi = pECIInput->fValidLengthRi;
    } else /* Left lane and right lane are both invalid */
    {
        /* Determine left boundary length */
        pECIOutput->fPosX0CtrlLf = 0.0F;
        pECIOutput->fPosY0CtrlLf = 5.0F;
        pECIOutput->fHeadingCtrlLf = 0.0F;
        pECIOutput->fCrvCtrlLf = 0.0F;
        pECIOutput->fCrvRateCtrlLf = 0.0F;
        pECIOutput->fValidLengthCtrlLf = 0.0F;

        /* Determine center boundary */
        pECIOutput->fPosX0CtrlCntr = 0.0F;
        pECIOutput->fPosY0CtrlCntr = 0.0F;
        pECIOutput->fHeadingCtrlCntr = 0.0F;
        pECIOutput->fCrvCtrlCntr = 0.0F;
        pECIOutput->fCrvRateCtrlCntr = 0.0F;
        pECIOutput->fValidLengthCtrlCntr = 0.0F;

        /* Determine right boundary */
        pECIOutput->fPosX0CtrlRi = 0.0F;
        pECIOutput->fPosY0CtrlRi = -5.0F;
        pECIOutput->fHeadingCtrlRi = 0.0F;
        pECIOutput->fCrvCtrlRi = 0.0F;
        pECIOutput->fCrvRateCtrlRi = 0.0F;
        pECIOutput->fValidLengthCtrlRi = 0.0F;
    }
}

void ECI_RangeChecks(const sECIParam_t *pECIParam, sECIOutput_t *pECIOutput) {
    if (TUE_CML_Abs_M(pECIOutput->fPosY0CtrlLf) > pECIParam->fMaxDistYRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 0U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 0U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fPosY0CtrlRi) > pECIParam->fMaxDistYRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 1U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 1U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fPosY0CtrlCntr) > pECIParam->fMaxDistYRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 2U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 2U);
    }

    /* Check heading angle out of range */
    if (TUE_CML_Abs_M(pECIOutput->fHeadingCtrlLf) >
        pECIParam->fMaxHeadingRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 3U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 3U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fHeadingCtrlRi) >
        pECIParam->fMaxHeadingRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 4U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 4U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fHeadingCtrlCntr) >
        pECIParam->fMaxHeadingRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 5U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 5U);
    }

    /* Check curvature out of range */
    if (TUE_CML_Abs_M(pECIOutput->fCrvCtrlLf) > pECIParam->fMaxCrvRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 6U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 6U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fCrvCtrlRi) > pECIParam->fMaxCrvRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 7U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 7U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fCrvCtrlCntr) > pECIParam->fMaxCrvRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 8U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 8U);
    }

    /* Check curvature rate out of range */
    if (TUE_CML_Abs_M(pECIOutput->fCrvRateCtrlLf) >
        pECIParam->fMaxCrvRateRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 9U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 9U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fCrvRateCtrlRi) >
        pECIParam->fMaxCrvRateRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 10U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 10U);
    }

    if (TUE_CML_Abs_M(pECIOutput->fCrvRateCtrlCntr) >
        pECIParam->fMaxCrvRateRange) {
        TUE_CML_Setbit_M(pECIOutput->uOutRangeCheckQualifier, 11U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uOutRangeCheckQualifier, 11U);
    }
}

void ECI_DetermineDMCQualifier(const sECIInput_t *pECIInput,
                               sECIOutput_t *pECIOutput) {
    if (pECIInput->bCamStatusQualifierValid == 0U) {
        pECIOutput->uLaneValidQualDMC = 0U; /* 0-no ego lanes */
    } else {
        if (pECIInput->uLaneValidQualifier == LANE_NO_VALID) {
            pECIOutput->uLaneValidQualDMC = 0U; /* 0-no ego lanes */
        } else if (pECIInput->uLaneValidQualifier == LANE_BOTH_VALID) {
            pECIOutput->uLaneValidQualDMC =
                2u; /* 2-both sided ego lane detection */
        } else {
            pECIOutput->uLaneValidQualDMC =
                1U; /* 1-one sided ego lane detection */
        }
    }
}

void ECI_DetermineVisualQualifier(UINT8_T bValidOnlyLf,
                                  UINT8_T bValidOnlyRi,
                                  const sECIInput_t *pECIInput,
                                  sECIOutput_t *pECIOutput) {
    if (pECIInput->bCamStatusQualifierValid == 0U) {
        pECIOutput->uVisualValidQualifier = 0U;
    } else {
        if (bValidOnlyLf == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uVisualValidQualifier, 0U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uVisualValidQualifier, 0U);
        }

        if (bValidOnlyRi == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uVisualValidQualifier, 1U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uVisualValidQualifier, 1U);
        }

        if (pECIInput->uLaneValidQualifier == LANE_LEFT_VIRTUAL) {
            TUE_CML_Setbit_M(pECIOutput->uVisualValidQualifier, 2U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uVisualValidQualifier, 2U);
        }

        if (pECIInput->uLaneValidQualifier == LANE_RIGHT_VIRTUAL) {
            TUE_CML_Setbit_M(pECIOutput->uVisualValidQualifier, 3U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uVisualValidQualifier, 3U);
        }

        if ((pECIInput->uLaneValidQualifier == LANE_BOTH_VALID) &&
            (pECIInput->bKalmanValidCntr == 1U)) {
            TUE_CML_Setbit_M(pECIOutput->uVisualValidQualifier, 6U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uVisualValidQualifier, 6U);
        }
    }
}

void ECI_DetermineLeftLaneQualifierBit(const sECIInput_t *pECIInput,
                                       sECIOutput_t *pECIOutput) {
    /* Input data range check:
    Bit 0: Left lateral distance out of range
    Bit 2: Left heading angle out of range
    Bit 4: Left curvature out of range
    Bit 6: Left curvature rate out of range
    Bit 8: Left valid length out of range */
    if ((pECIInput->uRangeCheckQualifier & 0x155) != 0U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 8U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 8U);
    }

    /* Output data range qualifier:
                    Bit 0: Left lateral position out of range
                    Bit 3: Left heading out of range
                    Bit 6: Left curvature out of range
                    Bit 9: Left curvature rate out of range */
    if ((pECIOutput->uOutRangeCheckQualifier & 0x249) != 0U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 9U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 9U);
    }

    // if (pECIInput->bLaneTypeInvalidLf == 1U) {
    //     TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 10U);
    // } else {
    //     TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 10U);
    // }

    if (pECIInput->bLaneColorInvalidLf == 1U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 11U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 11U);
    }

    if (pECIInput->bLaneQualityInvalidLf == 1U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 12U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 12U);
    }
}

void ECI_DetermineLeftLaneQualifier(UINT8_T bValidBoth,
                                    const sECIParam_t *pECIParam,
                                    const sECIInput_t *pECIInput,
                                    sECIOutput_t *pECIOutput) {
    UINT8_T bVirtualLaneLf = 0U;
    UINT8_T bDistYStepDebounced = 0U;
    UINT8_T bHeadingStepDebounced = 0U;
    UINT8_T bCrvStepDebounced = 0U;
    UINT8_T bCrvRateStepDebounced = 0U;
    UINT8_T bKalmanValid = 0U;
    if (pECIInput->bCamStatusQualifierValid == 0U) {
        pECIOutput->uLaneInvalidQualifierLf = 0U;
    } else {
        if (pECIInput->bNotAvailableLf == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 0U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 0U);
        }

        /*if (pECIInput->bDistYStepDtctLf == 1U)
        {
                TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 1U);
        }
        else
        {
                TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 1U);
        }*/

        if (pECIInput->bLengthInvalidLf == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 2U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 2U);
        }

        bDistYStepDebounced = bValidBoth ? pECIInput->bDistYStepDebouncedCntr
                                         : pECIInput->bDistYStepDebouncedLf;
        bHeadingStepDebounced = bValidBoth
                                    ? pECIInput->bHeadingStepDebouncedCntr
                                    : pECIInput->bHeadingStepDebouncedLf;
        bCrvStepDebounced = bValidBoth ? pECIInput->bCrvStepDebouncedCntr
                                       : pECIInput->bCrvStepDebouncedLf;
        bCrvRateStepDebounced = bValidBoth
                                    ? pECIInput->bCrvRateStepDebouncedCntr
                                    : pECIInput->bCrvRateStepDebouncedLf;

        if (bDistYStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 3U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 3U);
        }

        if (bHeadingStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 4U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 4U);
        }

        if (bCrvStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 5U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 5U);
        }

        if (bCrvRateStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 6U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 6U);
        }

        bKalmanValid = bValidBoth ? pECIInput->bKalmanValidCntr
                                  : pECIInput->bKalmanValidLf;
        if (bKalmanValid == 0U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 7U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 7U);
        }
        ECI_DetermineLeftLaneQualifierBit(pECIInput, pECIOutput);

        if ((pECIInput->uLaneValidQualifier == LANE_LEFT_VIRTUAL) &&
            (bValidBoth == 1U)) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 13U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 13U);
        }

        bVirtualLaneLf = TUE_CML_TurnOnDelay_M(
            pECIInput->bLaneVirtualCplLf, pECIParam->fTimeDelayVirtulLane,
            pECIParam->fSysCycleTime, &fTimerVirtualLaneLf, bLastVirtualLaneLf);
        if (bVirtualLaneLf == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 14U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 14U);
        }

        if (pECIInput->bLineMergeDtcLf) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierLf, 15U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierLf, 15U);
        }
    }
    bLastVirtualLaneLf = bVirtualLaneLf;
}

void ECI_DetermineRightLaneQualifierBit(const sECIInput_t *pECIInput,
                                        sECIOutput_t *pECIOutput) {
    /* Bit 8: Right lane input data out of range (By ULP) */
    /* Input data range check:
                    Bit 0: Left lateral distance out of range
                    Bit 2: Left heading angle out of range
                    Bit 4: Left curvature out of range
                    Bit 6: Left curvature rate out of range
                    Bit 8: Left valid length out of range */
    if ((pECIInput->uRangeCheckQualifier & 0x2aa) != 0U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 8U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 8U);
    }

    /* Bit 9: Right lane Output data out of range(By ULP) */
    /* Output data range qualifier:
                    Bit 0: Left lateral position out of range
                    Bit 3: Left heading out of range
                    Bit 6: Left curvature out of range
                    Bit 9: Left curvature rate out of range */
    if ((pECIOutput->uOutRangeCheckQualifier & 0x249) != 0U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 9U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 9U);
    }

    /* Bit10: Right Lane type invalid(By ULP) */
    // if (pECIInput->bLaneTypeInvalidRi == 1U) {
    //     TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 10U);
    // } else {
    //     TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 10U);
    // }

    /* Bit11: Right lane color invalid(By ULP) */
    if (pECIInput->bLaneColorInvalidRi == 1U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 11U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 11U);
    }

    /* Bit12: Right lane color invalid(By ULP) */
    if (pECIInput->bLaneQualityInvalidRi == 1U) {
        TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 12U);
    } else {
        TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 12U);
    }
}

void ECI_DetermineRightLaneQualifier(UINT8_T bValidBoth,
                                     const sECIParam_t *pECIParam,
                                     const sECIInput_t *pECIInput,
                                     sECIOutput_t *pECIOutput) {
    UINT8_T bVirtualLaneRi = 0U;
    UINT8_T bDistYStepDebounced = 0U;
    UINT8_T bHeadingStepDebounced = 0U;
    UINT8_T bCrvStepDebounced = 0U;
    UINT8_T bCrvRateStepDebounced = 0U;
    UINT8_T bKalmanValid = 0U;
    if (pECIInput->bCamStatusQualifierValid == 0U) {
        pECIOutput->uLaneInvalidQualifierRi = 0U;
    } else {
        /* Bit 0: Right lane not available */
        if (pECIInput->bNotAvailableRi == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 0U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 0U);
        }

        /* Bit 1: Right lane lateral distance step (By ULP) */
        // if (pECIInput->bDistYStepDtctRi == 1U)
        // {
        //    TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 1U);
        // }
        // else
        // {
        //    TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 1U);
        // }

        /* Bit 2: Right lane length invalid (By ULP) */
        if (pECIInput->bLengthInvalidRi == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 2U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 2U);
        }

        bDistYStepDebounced = bValidBoth ? pECIInput->bDistYStepDebouncedCntr
                                         : pECIInput->bDistYStepDebouncedRi;
        bHeadingStepDebounced = bValidBoth
                                    ? pECIInput->bHeadingStepDebouncedCntr
                                    : pECIInput->bHeadingStepDebouncedRi;
        bCrvStepDebounced = bValidBoth ? pECIInput->bCrvStepDebouncedCntr
                                       : pECIInput->bCrvStepDebouncedRi;
        bCrvRateStepDebounced = bValidBoth
                                    ? pECIInput->bCrvRateStepDebouncedCntr
                                    : pECIInput->bCrvRateStepDebouncedRi;

        /* Bit 3: Right lane lateral distance step (output data) */
        if (bDistYStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 3U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 3U);
        }

        /* Bit 4: Right lane lateral distance step (output data) */
        if (bHeadingStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 4U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 4U);
        }

        /* Bit 5: Right lane curvature step (output data) */
        if (bCrvStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 5U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 5U);
        }

        /* Bit 6: Right lane curvature rate step (output data) */
        if (bCrvRateStepDebounced == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 6U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 6U);
        }

        /* Bit 7: Right lane Kalman filter status not valid */
        bKalmanValid = bValidBoth ? pECIInput->bKalmanValidCntr
                                  : pECIInput->bKalmanValidRi;
        if (bKalmanValid == 0U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 7U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 7U);
        }
        ECI_DetermineRightLaneQualifierBit(pECIInput, pECIOutput);

        if ((pECIInput->uLaneValidQualifier == LANE_RIGHT_VIRTUAL) &&
            (bValidBoth == 1U)) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 13U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 13U);
        }

        bVirtualLaneRi = TUE_CML_TurnOnDelay_M(
            pECIInput->bLaneVirtualCplRi, pECIParam->fTimeDelayVirtulLane,
            pECIParam->fSysCycleTime, &fTimerVirtualLaneRi, bLastVirtualLaneRi);
        if (bVirtualLaneRi == 1U) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 14U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 14U);
        }

        if (pECIInput->bLineMergeDtcRi) {
            TUE_CML_Setbit_M(pECIOutput->uLaneInvalidQualifierRi, 15U);
        } else {
            TUE_CML_Clrbit_M(pECIOutput->uLaneInvalidQualifierRi, 15U);
        }
    }
    bLastVirtualLaneRi = bVirtualLaneRi;
}
/****************************************************************************************
        @fn           EgoControlInterface(EgoLaneOutputGeneration)
        @brief        Ego lane control interface output data
        @description  Ego lane control interface:
                                          1.Determine ego lane Boundarie
 validity;
                                          2.Determine ego lane boundaries
 outputData;
                                          3.Range checks;
                                          4.Generate lane bitfields.
        @param[in]    pECIInput : Input for ECI
        @param[in]    pECIParam : Parameter for ECI
        @param[out]   pECIOutput: Output for ECI
        @param[out]   pECIDebug : Debug(measurement) for ECI
        @return       void
        @startuml
        title EgoLaneGeneration
        (*)--> 1.DetermineEgoLaneBoundarieValidity
           --> 2.DetermineEgoLaneBoundariesOutputData
           --> 3.RangeChecks
           --> (*)
           1.DetermineEgoLaneBoundarieValidity --> 4.GenerateLaneBitfields
           3.RangeChecks --> 4.GenerateLaneBitfields
           --> (*)
        (*)--> (*)
        @enduml
 ******************************************************************************************/
void EgoControlInterface(const sECIInput_t *pECIInput,
                         const sECIParam_t *pECIParam,
                         sECIOutput_t *pECIOutput,
                         sECIDebug_t *pECIDebug) {
    UINT8_T bValidBoth = 0U;
    UINT8_T bValidOnlyLf = 0U;
    UINT8_T bValidOnlyRi = 0U;

    /**************************1.Determine ego lane Boundarie
     * validity********************/
    /* Determine both lane lines are valid */
    bValidBoth = ECI_bValidBoth(pECIInput->uLaneValidQualifier,
                                pECIInput->uBridgePossible);

    /* Determine only the left lane is valid */
    if ((pECIInput->bKalmanValidLf == 1U) &&
        (pECIInput->bDistYStepDebouncedLf == 0U) &&
        (pECIInput->bHeadingStepDebouncedLf == 0U) &&
        (pECIInput->bCrvStepDebouncedLf == 0U) &&
        (pECIInput->bCrvRateStepDebouncedLf == 0U) &&
        (pECIInput->uLaneValidQualifier == LANE_LEFT_VALID)) {
        bValidOnlyLf = 1U;
    } else {
        bValidOnlyLf = 0U;
    }

    /* Determine only the right lane is valid */
    if ((pECIInput->bKalmanValidRi == 1U) &&
        (pECIInput->bDistYStepDebouncedRi == 0U) &&
        (pECIInput->bHeadingStepDebouncedRi == 0U) &&
        (pECIInput->bCrvStepDebouncedRi == 0U) &&
        (pECIInput->bCrvRateStepDebouncedRi == 0U) &&
        (pECIInput->uLaneValidQualifier == LANE_RIGHT_VALID)) {
        bValidOnlyRi = 1U;
    } else {
        bValidOnlyRi = 0U;
    }

    /************************2.Determine ego lane boundaries
     * outputData********************/
    ECI_EgoLaneBoundariesOut(bValidBoth, bValidOnlyLf, bValidOnlyRi, pECIInput,
                             pECIParam, pECIOutput);

    /************************3.Range
     * checks*************************************************/
    /* Check lateral position out of range */
    ECI_RangeChecks(pECIParam, pECIOutput);

    /************************4.Generate lane
     * bitfields*************************************/
    /* 4.1 Determine LatDMC qualifier */
    ECI_DetermineDMCQualifier(pECIInput, pECIOutput);

    /* 4.2 Determine valid visualization qualifier */
    ECI_DetermineVisualQualifier(bValidOnlyLf, bValidOnlyRi, pECIInput,
                                 pECIOutput);
    /* 4.2 Determine valid visualization qualifier */

    /* 4.3 Determine left ego lane qualifier */
    ECI_DetermineLeftLaneQualifier(bValidBoth, pECIParam, pECIInput,
                                   pECIOutput);

    /* 4.4 Determine Right ego lane qualifier */
    ECI_DetermineRightLaneQualifier(bValidBoth, pECIParam, pECIInput,
                                    pECIOutput);
    // pECIOutput->uLaneInvalidQualifierLf = 0U;
    // pECIOutput->uLaneInvalidQualifierRi = 0U;

    /**********************************Output and
     * debug*****************************************/
    pECIOutput->fLaneWidth = pECIInput->fLaneWidth;
    pECIOutput->uFltStatusCntr = pECIInput->uFltStatusCntr;
    pECIOutput->fFltQualityCntr = pECIInput->fFltQualityCntr;

    pECIDebug->bValidBoth = bValidBoth;
    pECIDebug->bValidOnlyLf = bValidOnlyLf;
    pECIDebug->bValidOnlyRi = bValidOnlyRi;
} /* EgoLaneSafetyInterface */

/****************************************************************************************
        @fn           OUTPUT_EgoLaneGeneration
        @brief        Ego lane generation output data
        @description  Ego lane generation output data:
                                          1.Ego lane generic interface output
 data;
                                          2.Ego lane safety interface output
 data;
                                          3.Ego lane control interface output
 data;
        @param[in]    pESIOutput : Output from EgoSafetyInterface
        @param[in]    pESIDebug_t : Debug from EgoSafetyInterface
        @param[in]    pECIOutput: Output from EgoControlInterface
        @param[in]    pECIDebug_t : Debug from EgoControlInterface
        @param[out]   pELGOutput: Output for OUTPUT_EgoLaneGeneration
        @param[out]   pELGDebug : Debug(measurement) for
 OUTPUT_EgoLaneGeneration
        @return       void
        @startuml
        title OUTPUT_EgoLaneGeneration
        (*)--> 1.GenericEgoLaneOutputData
           --> (*)
        (*)-->2.EgoLaneSafetyInterface
           -->(*)
        (*)-->3.EgoLaneOutputGeneration
           -->(*)
        @enduml
 ******************************************************************************************/
void OUTPUT_EgoLaneGeneration(const sESIOutput_t *pESIOutput,
                              const sESIDebug_t *pESIDebug_t,
                              const sECIOutput_t *pECIOutput,
                              const sECIDebug_t *pECIDebug_t,
                              sELGOutput_t *pELGOutput,
                              sELGDebug_t *pELGDebug) {
    /***********************************1.output*******************************/
    /* 1.1 ESI output*/
    pELGOutput->fPosY0SafeLf = pESIOutput->fPosY0SafeLf;
    pELGOutput->fHeadingSafeLf = pESIOutput->fHeadingSafeLf;
    pELGOutput->fCrvSafeLf = pESIOutput->fCrvSafeLf;
    pELGOutput->uInvalidQualifierSafeLf = pESIOutput->uInvalidQualifierSafeLf;
    pELGOutput->fPosY0SafeRi = pESIOutput->fPosY0SafeRi;
    pELGOutput->fHeadingSafeRi = pESIOutput->fHeadingSafeRi;
    pELGOutput->fCrvSafeRi = pESIOutput->fCrvSafeRi;
    pELGOutput->uInvalidQualifierSafeRi = pESIOutput->uInvalidQualifierSafeRi;

    /* 1.2 ECI output */
    pELGOutput->fLaneWidth = pECIOutput->fLaneWidth;
    pELGOutput->uLaneInvalidQualifierLf = pECIOutput->uLaneInvalidQualifierLf;
    pELGOutput->uLaneInvalidQualifierRi = pECIOutput->uLaneInvalidQualifierRi;
    pELGOutput->uVisualValidQualifier = pECIOutput->uVisualValidQualifier;
    pELGOutput->fFltQualityCntr = pECIOutput->fFltQualityCntr;
    pELGOutput->uFltStatusCntr = pECIOutput->uFltStatusCntr;
    pELGOutput->uLaneValidQualDMC = pECIOutput->uLaneValidQualDMC;
    pELGOutput->uOutRangeCheckQualifier = pECIOutput->uOutRangeCheckQualifier;

    pELGOutput->fPosX0CtrlLf = pECIOutput->fPosX0CtrlLf;
    pELGOutput->fPosY0CtrlLf = pECIOutput->fPosY0CtrlLf;
    pELGOutput->fHeadingCtrlLf = pECIOutput->fHeadingCtrlLf;
    pELGOutput->fCrvCtrlLf = pECIOutput->fCrvCtrlLf;
    pELGOutput->fCrvRateCtrlLf = pECIOutput->fCrvRateCtrlLf;
    pELGOutput->fValidLengthCtrlLf = pECIOutput->fValidLengthCtrlLf;

    pELGOutput->fPosX0CtrlCntr = pECIOutput->fPosX0CtrlCntr;
    pELGOutput->fPosY0CtrlCntr = pECIOutput->fPosY0CtrlCntr;
    pELGOutput->fHeadingCtrlCntr = pECIOutput->fHeadingCtrlCntr;
    pELGOutput->fCrvCtrlCntr = pECIOutput->fCrvCtrlCntr;
    pELGOutput->fCrvRateCtrlCntr = pECIOutput->fCrvRateCtrlCntr;
    pELGOutput->fValidLengthCtrlCntr = pECIOutput->fValidLengthCtrlCntr;

    pELGOutput->fPosX0CtrlRi = pECIOutput->fPosX0CtrlRi;
    pELGOutput->fPosY0CtrlRi = pECIOutput->fPosY0CtrlRi;
    pELGOutput->fHeadingCtrlRi = pECIOutput->fHeadingCtrlRi;
    pELGOutput->fCrvCtrlRi = pECIOutput->fCrvCtrlRi;
    pELGOutput->fCrvRateCtrlRi = pECIOutput->fCrvRateCtrlRi;
    pELGOutput->fValidLengthCtrlRi = pECIOutput->fValidLengthCtrlRi;

    /***********************************2.Debug*******************************/
    /* 2.1 ESI debug*/

    /* 2.2 ECI debug*/
}

/****************************************************************************************
        @fn           EgoLaneGeneration
        @brief        Generate ego lane from camera input
        @description  Ego lane generation:
                                          1.Ego lane generic interface output
 data;
                                          2.Ego lane safety interface output
 data;
                                          3.Ego lane control interface output
 data;
        @param[in]    pELGInput : Input for ELG
        @param[in]    pELGParam : Parameter for ELGe
        @param[out]   pELGOutput: Output for ELG
        @param[out]   pELGDebug : Debug(measurement) for ELG
        @return       void
        @startuml
        title EgoLaneGeneration
        (*)--> 1.Ego lane generic interface output data
           --> (*)
        (*)-->2.Ego lane safety interface output data
           -->(*)
        (*)-->3.Ego lane control interface output data
           -->(*)
        @enduml
 ******************************************************************************************/
void EgoLaneGeneration(const sELGInput_t *pELGInput,
                       const sELGParam_t *pELGParam,
                       sELGOutput_t *pELGOutput,
                       sELGDebug_t *pELGDebug) {
    sESIInput_t sESIInput = {0};
    sESIParam_t sESIParam = {0};
    sESIOutput_t sESIOutput = {0};
    sESIDebug_t sESIDebug = {0};

    sECIInput_t sECIInput = {0};
    sECIParam_t sECIParam = {0};
    sECIOutput_t sECIOutput = {0};
    sECIDebug_t sECIDebug = {0};

    /***********************************1.Ego lane generic interface output
     * data****************************/
    pELGOutput->bLaneChangeDtct = pELGInput->bLaneChangeDtct;
    pELGOutput->uLaneTypeLf = pELGInput->uLaneTypeLf;
    pELGOutput->uLaneTypeRi = pELGInput->uLaneTypeRi;
    pELGOutput->bConstructionSiteDtct = pELGInput->bConstructionSiteDtct;
    pELGOutput->uOverallQualityLf =
        (UINT8_T)(TUE_CML_Limit_M(pELGInput->fOverallQualityLf, 0.0F, 100.0F));
    pELGOutput->uOverallQualityRi =
        (UINT8_T)(TUE_CML_Limit_M(pELGInput->fOverallQualityRi, 0.0F, 100.0F));
    pELGOutput->uCrvQualityLf = (UINT8_T)(TUE_CML_Limit_M(
        pELGInput->fOverallCrvQualityLf, 0.0F, 100.0F));
    pELGOutput->uCrvQualityRi = (UINT8_T)(TUE_CML_Limit_M(
        pELGInput->fOverallCrvQualityRi, 0.0F, 100.0F));
    pELGOutput->fABDTimeStamp =
        TUE_CML_Limit_M((pELGInput->fTstamp * 1E-6f), 0, 4295);
    pELGOutput->uRangeCheckQualifier = pELGInput->uRangeCheckQualifier;

    /***********************************2.Ego lane safety interface output
     * data*******************************/
    INPUT_EgoSafetyInterface(pELGInput, pELGParam, &sESIInput, &sESIParam);
    EgoSafetyInterface(&sESIInput, &sESIParam, &sESIOutput, &sESIDebug);

    /***********************************3.Ego lane control interface output
     * data*******************************/
    INPUT_EgoControlInterface(pELGInput, pELGParam, &sECIInput, &sECIParam);
    EgoControlInterface(&sECIInput, &sECIParam, &sECIOutput, &sECIDebug);

    /**************************************output and
     * debug*********************************/
    OUTPUT_EgoLaneGeneration(&sESIOutput, &sESIDebug, &sECIOutput, &sECIDebug,
                             pELGOutput, pELGDebug);

    TUE_CML_MemoryCopy_M((void *)&sESIInput, (void *)&(pELGDebug->sESIInput),
                         sizeof(sESIInput_t));
    TUE_CML_MemoryCopy_M((void *)&sESIParam, (void *)&(pELGDebug->sESIParam),
                         sizeof(sESIParam_t));
    TUE_CML_MemoryCopy_M((void *)&sESIOutput, (void *)&(pELGDebug->sESIOutput),
                         sizeof(sESIOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sESIDebug, (void *)&(pELGDebug->sESIDebug),
                         sizeof(sESIDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sECIInput, (void *)&(pELGDebug->sECIInput),
                         sizeof(sECIInput_t));
    TUE_CML_MemoryCopy_M((void *)&sECIParam, (void *)&(pELGDebug->sECIParam),
                         sizeof(sECIParam_t));
    TUE_CML_MemoryCopy_M((void *)&sECIOutput, (void *)&(pELGDebug->sECIOutput),
                         sizeof(sECIOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sECIDebug, (void *)&(pELGDebug->sECIDebug),
                         sizeof(sECIDebug_t));
}

/****************************************************************************************
        @fn           EgoControlInterface(EgoLaneOutputGeneration)
        @brief        Ego lane control interface output data
        @description  Ego lane control interface:
                                          1.Uncoupled lane processing;
                                          2.Check lane properity;
                                          3.Lane filtering and plausibilization;
                                          4.Ego lane generation.
        @param[in]    pECIInput : Input for EgoLaneControlInterface
        @param[in]    pECIParam : Parameter for EgoLaneControlInterface
        @param[out]   pECIOutput: Output for EgoLaneControlInterface
        @param[out]   pECIDebug : Debug(measurement) for EgoLaneControlInterface
        @return       void
        @startuml
        title LBPout
        (*)--> UncoupledLaneProcessing
           --> CheckLaneProperity
           --> LaneFilterProcessing
           --> EgoLaneGeneration
           --> LBPout
           --> (*)
           UncoupledLaneProcessing --> LaneFilterProcessing
           UncoupledLaneProcessing --> EgoLaneGeneration
           CheckLaneProperity --> EgoLaneGeneration
        (*) --> CheckLaneProperity
        @enduml
 ******************************************************************************************/
void LBPOutput(const sULPOutput_t *pULPOutput,
               const sULPDebug_t *pULPDebug,
               const sCLPOutput_t *pCLPOutput,
               const sCLPDebug_t *pCLPDebug,
               const sLFPOutput_t *pLFPOutput,
               const sLFPDebug_t *pLFPDebug,
               const sELGOutput_t *pELGOutput,
               const sELGDebug_t *pELGDebug,
               sLBPOutput_t *pLBPOutput,
               sLBPDebug_t *pLBPDebug) {
    /*******************************1.LBP output**********************/
    /* ULP(7) */
    pLBPOutput->bBridgePossibleUnCplLf = pULPOutput->bBridgePossibleUnCplLf;
    pLBPOutput->bBridgePossibleUnCplRi = pULPOutput->bBridgePossibleUnCplRi;
    pLBPOutput->uOverallQualityUnCplLf =
        (UINT8_T)(pULPOutput->fOverallQualityUnCplLf);
    pLBPOutput->uOverallQualityUnCplRi =
        (UINT8_T)(pULPOutput->fOverallQualityUnCplRi);
    pLBPOutput->uOverallQualityCplLf =
        (UINT8_T)(pULPOutput->fOverallQualityCplLf);
    pLBPOutput->uOverallQualityCplRi =
        (UINT8_T)(pULPOutput->fOverallQualityCplRi);
    pLBPOutput->uBtfBridgeUnCpl = pULPOutput->uBtfBridgeUnCpl;

    /* CLP(6)*/
    pLBPOutput->uPercStraightDtct = (UINT8_T)(pCLPOutput->fPercStraightDtct);
    pLBPOutput->uPercExitLf = (UINT8_T)(pCLPOutput->fPercExitLf);
    pLBPOutput->uPercExitRi = (UINT8_T)(pCLPOutput->fPercExitRi);
    pLBPOutput->uPercUpDownHillDtct =
        (UINT8_T)(pCLPOutput->fPercUpDownHillDtct);
    pLBPOutput->fLaneWidthUnCpl = pCLPOutput->fRawLaneWidthUnCpl;
    pLBPOutput->fLaneWidthCpl = pCLPOutput->fRawLaneWidthCpl;
    pLBPOutput->bLineMergeDtcRi = pCLPOutput->bLineMergeDtcRi;
    pLBPOutput->bLineMergeDtcLf = pCLPOutput->bLineMergeDtcLf;

    /* ELG/GLO(10) */
    pLBPOutput->bLaneChangeDtct = pELGOutput->bLaneChangeDtct;
    pLBPOutput->uLaneTypeLf = pELGOutput->uLaneTypeLf;
    pLBPOutput->uLaneTypeRi = pELGOutput->uLaneTypeRi;
    pLBPOutput->bConstructionSiteDtct = pELGOutput->bConstructionSiteDtct;
    pLBPOutput->uOverallQualityLf = pELGOutput->uOverallQualityLf;
    pLBPOutput->uOverallQualityRi = pELGOutput->uOverallQualityRi;
    pLBPOutput->uCrvQualityLf = pELGOutput->uCrvQualityLf;
    pLBPOutput->uCrvQualityRi = pELGOutput->uCrvQualityRi;
    pLBPOutput->fABDTimeStamp = pELGOutput->fABDTimeStamp;
    pLBPOutput->uRangeCheckQualifier = pELGOutput->uRangeCheckQualifier;

    /* ELG/ESI(8) */
    pLBPOutput->fPosY0SafeLf = pELGOutput->fPosY0SafeLf;
    pLBPOutput->fHeadingSafeLf = pELGOutput->fHeadingSafeLf;
    pLBPOutput->fCrvSafeLf = pELGOutput->fCrvSafeLf;
    pLBPOutput->uInvalidQualifierSafeLf = pELGOutput->uInvalidQualifierSafeLf;
    pLBPOutput->fPosY0SafeRi = pELGOutput->fPosY0SafeRi;
    pLBPOutput->fHeadingSafeRi = pELGOutput->fHeadingSafeRi;
    pLBPOutput->fCrvSafeRi = pELGOutput->fCrvSafeRi;
    pLBPOutput->uInvalidQualifierSafeRi = pELGOutput->uInvalidQualifierSafeRi;

    /* ELG/ECI(26) */
    pLBPOutput->fLaneWidth = pELGOutput->fLaneWidth;
    pLBPOutput->uLaneInvalidQualifierLf = pELGOutput->uLaneInvalidQualifierLf;
    pLBPOutput->uLaneInvalidQualifierRi = pELGOutput->uLaneInvalidQualifierRi;
    pLBPOutput->uVisualValidQualifier = pELGOutput->uVisualValidQualifier;
    pLBPOutput->fFltQualityCntr = pELGOutput->fFltQualityCntr;
    pLBPOutput->uFltStatusCntr = pELGOutput->uFltStatusCntr;
    pLBPOutput->uLaneValidQualDMC = pELGOutput->uLaneValidQualDMC;
    pLBPOutput->uOutRangeCheckQualifier = pELGOutput->uOutRangeCheckQualifier;

    pLBPOutput->fPosX0CtrlLf = pELGOutput->fPosX0CtrlLf;
    pLBPOutput->fPosY0CtrlLf = pELGOutput->fPosY0CtrlLf;
    pLBPOutput->fHeadingCtrlLf = pELGOutput->fHeadingCtrlLf;
    pLBPOutput->fCrvCtrlLf = pELGOutput->fCrvCtrlLf;
    pLBPOutput->fCrvRateCtrlLf = pELGOutput->fCrvRateCtrlLf;
    pLBPOutput->fValidLengthCtrlLf = pELGOutput->fValidLengthCtrlLf;

    pLBPOutput->fPosX0CtrlCntr = pELGOutput->fPosX0CtrlCntr;
    pLBPOutput->fPosY0CtrlCntr = pELGOutput->fPosY0CtrlCntr;
    pLBPOutput->fHeadingCtrlCntr = pELGOutput->fHeadingCtrlCntr;
    pLBPOutput->fCrvCtrlCntr = pELGOutput->fCrvCtrlCntr;
    pLBPOutput->fCrvRateCtrlCntr = pELGOutput->fCrvRateCtrlCntr;
    pLBPOutput->fValidLengthCtrlCntr = pELGOutput->fValidLengthCtrlCntr;

    pLBPOutput->fPosX0CtrlRi = pELGOutput->fPosX0CtrlRi;
    pLBPOutput->fPosY0CtrlRi = pELGOutput->fPosY0CtrlRi;
    pLBPOutput->fHeadingCtrlRi = pELGOutput->fHeadingCtrlRi;
    pLBPOutput->fCrvCtrlRi = pELGOutput->fCrvCtrlRi;
    pLBPOutput->fCrvRateCtrlRi = pELGOutput->fCrvRateCtrlRi;
    pLBPOutput->fValidLengthCtrlRi = pELGOutput->fValidLengthCtrlRi;

    /*******************************1.LBP Debug**********************/
    /* ELG/GLO */

    /* ELG/ESI */

    /* ELG/ECI */
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */