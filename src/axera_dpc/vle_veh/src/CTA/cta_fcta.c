

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#include "./cta_fcta.h"  // NOLINT
#include <string.h>
// #include "../../decision/src/CTA/cta_fcta.h"

#include "./cta.h"  // NOLINT
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
FCTAGlobal_t FCTAGlobal;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define CAL_START_CODE
#include "Mem_Map.h"
const volatile unsigned char FCTA_HIT_CONFI_THRESH = 90u;
#define CAL_STOP_CODE
#include "Mem_Map.h"

/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void FCTAExec(const FCTAInReq_t* FCTAreqPorts,
              const FCTAParam_t* FCTAparams,
              FCTAOutPro_t* FCTAproPorts,
              FCTADebug_t* FCTAdebugInfo) {
    if (FCTAreqPorts->bFCTAFunctionActive) {
        FCTAPreProcess(FCTAreqPorts, FCTAparams, &FCTAGlobal);
        FCTAMainProcess(FCTAreqPorts, FCTAparams, &FCTAGlobal);
        FCTAProProcess(&FCTAGlobal, FCTAproPorts, FCTAdebugInfo);
    } else {
        if (FCTAreqPorts->LastCycleStates.bFCTAFunctionActive) {
            FCTAReset();
        }
        // FCTAdebugInfo->FCTAStateMachine = FCTADebugStateOFF;
    }
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void FCTAReset() {
    FCTAGlobal.fCriticalTTC_s = TUE_C_F32_VALUE_INVALID;
    FCTAGlobal.fCriticalObjDistY_met = TUE_C_F32_VALUE_INVALID;
    FCTAGlobal.fCriticalObjDistYLastCycle_met = TUE_C_F32_VALUE_INVALID;
    FCTAGlobal.iCriticalObjID_nu = -1;
    FCTAGlobal.iCriticalObjIDLastCycle_nu = -1;
    FCTAGlobal.bWarningInterrupt = FALSE;
    FCTAGlobal.uInterruptCycleCount_nu = 0u;
    FCTAGlobal.FCTAStateMachine = FCTAStateOFF;

    // FCTAGlobal.fMaxLatSensorRange = TUE_C_F32_VALUE_INVALID;

    for (uint8 uObj = 0u; uObj < FCTA_MAX_NUM_OBJECTS; uObj++) {
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bMirror = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bObjectAtEdgeFoV = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bObjectFromSide = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bQuality = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bRelevant = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bShortWarning = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bUpdatedRecently = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bValidApproachAngle = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bRearMirrorObject = FALSE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bObjMovementValid = TRUE;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].fRearTrackProb = 0.f;
        FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].uRearMirrorCnt = 0u;
        // Initialization of FCTA object level attributes
        for (uint8 uWarnLevel = 0u;
             uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS; uWarnLevel++) {
            // Init warning level values
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .fBTHitHystTimer_s[uWarnLevel] = 0.f;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .uBreakthroughHitConfi[uWarnLevel] = 0u;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj].bWarning[uWarnLevel] =
                FALSE;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .bWarningLastCycle[uWarnLevel] = FALSE;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .bObjectInRange[uWarnLevel] = FALSE;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .bTTCBelowThresh[uWarnLevel] = FALSE;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .bBTHitHystActive[uWarnLevel] = FALSE;
            FCTAGlobal.FCTA_Va_ObjectListGlobal[uObj]
                .bBreakthroughHit[uWarnLevel] = FALSE;
        }
    }
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void FCTAPreProcess(const FCTAInReq_t* FCTAreqPorts,
                    const FCTAParam_t* FCTAparams,
                    FCTAGlobal_t* pFCTAGlobal) {
    // set all parameters for RCTA function depending on ego speed and BSW
    // parameters
    CTAFCTASetParameters(FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps,
                         FCTAreqPorts->FCTARoadInformation.fCurveRadius_met,
                         FCTAreqPorts->LastCycleStates.FCTAState,
                         FCTAreqPorts->CTAGlobleInput.fSensorOffsetToRear_met,
                         FCTAparams, pFCTAGlobal);
    // Init all cyclic variables
    CTAFCTAInitCyclic(pFCTAGlobal->bFCTAWarnActive);
}
/*****************************************************************************
  Functionname:CTAFCTASetParameters                                     */ /*!

       @brief set necessary algorithm parameters for FCTA decision

       @description set necessary algorithm parameters for FCTA decision. Use
     BSW algorithm parameters by default,
                    if not available use hard coded defines, so keep
     defines/values up to date

       @param[in] fegoVelocity  the ego vehicle longitudinal velocity ,unit:m/s
                              fCurveRadius  The curvature radius of the current
     driver road ,unit:m
                              FCTALastCycleState  FCTA function State in last
     cycle
                              fSensorOffsetToRear_met  Distance between mounting
     position of the sensor and vehicle front edge
                              FCTAparams  FCTA parameter structure

       @param[out] pFCTAGlobal  FCTA global structure

       @return
     *****************************************************************************/
void CTAFCTASetParameters(const float32 fegoVelocity,
                          const float32 fCurveRadius,
                          FCTAStateInReq_t FCTALastCycleState,
                          float32 fSensorOffsetToRear_met,
                          const FCTAParam_t* FCTAparams,
                          FCTAGlobal_t* pFCTAGlobal) {
    float32 fXMinBreakthrough = 0.0f;
    float32 fXMaxBreakthrough = 0.0f;
    float32 fTTCThreshold = 0.0f;
    float32 fTargetRangeMax = 0.0f;
    // float32 fBreakthroughReduction = 0.0f;

    // Set level dependent parameters
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        switch (uWarnLevel) {
            case 0:
                fTargetRangeMax = FCTAparams->fTargetRangeMax;  // 71.f;
                fTTCThreshold =
                    FCTAparams->fTTCThreshold +
                    FCTAparams->fTTCThresholdMargin;  // 2.5f + 1.0f;
                fXMaxBreakthrough = FCTAparams->fXMaxBreakthrough;  // 7.0f;
                fXMinBreakthrough = FCTAparams->fXMinBreakthrough;  // 0.f;
                // For level 0 FCTA warning function adapt the breakthrough
                // parameters based on curve radius
                // if (fABS(fegoVelocity) >
                //         CTA_FCTA_ADAPT_BREAKTHROUGH_MIN_SPEED &&  // 0.5
                //     FCTALastCycleState != FCTA_INIT) {
                //     fBreakthroughReduction = TUE_CML_BoundedLinInterpol2(
                //         fABS(fCurveRadius), CTA_FCTA_LI_ADAPT_BRK_MIN_CURVE,
                //         CTA_FCTA_LI_ADAPT_BRK_MAX_CURVE,
                //         CTA_FCTA_LI_ADAPT_BRK_MIN,
                //         CTA_FCTA_LI_ADAPT_BRK_MAX);  // 10  50  1  0
                //     fXMinBreakthrough += fBreakthroughReduction;
                //     fXMaxBreakthrough -= fBreakthroughReduction;
                // }
                break;
            case 1:
                fTargetRangeMax = FCTAparams->fTargetRangeMaxL2;  // 71.f;
                fTTCThreshold =
                    FCTAparams->fTTCThresholdL2 +
                    FCTAparams->fTTCThresholdMargin;  // 2.5f + 1.0f;
                fXMaxBreakthrough = FCTAparams->fXMaxBreakthroughL2;  // 7.0f;
                fXMinBreakthrough = FCTAparams->fXMinBreakthrough;    // 0.f;
                // For level 0 FCTA warning function adapt the breakthrough
                // parameters based on curve radius
                // if (fABS(fegoVelocity) >
                //         CTA_FCTA_ADAPT_BREAKTHROUGH_MIN_SPEED &&  // 0.5
                //     FCTALastCycleState != FCTA_INIT) {
                //     // fBreakthroughReduction = TUE_CML_BoundedLinInterpol2(
                //     //     fABS(fCurveRadius),
                //     CTA_FCTA_LI_ADAPT_BRK_MIN_CURVE,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MAX_CURVE,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MIN,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MAX);  // 10  50  1  0
                //     fXMinBreakthrough += fBreakthroughReduction;
                //     fXMaxBreakthrough -= fBreakthroughReduction;
                // }
                break;
            case 2:
                fTargetRangeMax = FCTAparams->fTargetRangeMaxL3;  // 71.f;
                fTTCThreshold =
                    FCTAparams->fTTCThresholdL3 +
                    FCTAparams->fTTCThresholdMargin;  // 2.5f + 1.0f;
                fXMaxBreakthrough = FCTAparams->fXMaxBreakthroughL3;  // 7.0f;
                fXMinBreakthrough = FCTAparams->fXMinBreakthrough;    // 0.f;
                // For level 0 FCTA warning function adapt the breakthrough
                // parameters based on curve radius
                // if (fABS(fegoVelocity) >
                //         CTA_FCTA_ADAPT_BREAKTHROUGH_MIN_SPEED &&  // 0.5
                //     FCTALastCycleState != FCTA_INIT) {
                //     // fBreakthroughReduction = TUE_CML_BoundedLinInterpol2(
                //     //     fABS(fCurveRadius),
                //     CTA_FCTA_LI_ADAPT_BRK_MIN_CURVE,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MAX_CURVE,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MIN,
                //     //     CTA_FCTA_LI_ADAPT_BRK_MAX);  // 10  50  1  0
                //     fXMinBreakthrough += fBreakthroughReduction;
                //     fXMaxBreakthrough -= fBreakthroughReduction;
                // }
                break;
            default:
                break;
        }

        // Filter min and max breakthrough values for smooth transition between
        // reduced and normal values
        if (FCTALastCycleState == FCTA_INIT) {
            FCTAGlobal.fXMinBreakthrough_met[uWarnLevel] =
                fXMinBreakthrough - fSensorOffsetToRear_met;
            FCTAGlobal.fXMaxBreakthrough_met[uWarnLevel] =
                fXMaxBreakthrough - fSensorOffsetToRear_met;
        } else {
            TUE_CML_LowPassFilter(
                &pFCTAGlobal->fXMinBreakthrough_met[uWarnLevel],
                (fXMinBreakthrough - fSensorOffsetToRear_met),
                CTA_FCTA_X_BREAKTHROUGH_FILTER);
            TUE_CML_LowPassFilter(
                &pFCTAGlobal->fXMaxBreakthrough_met[uWarnLevel],
                (fXMaxBreakthrough - fSensorOffsetToRear_met),
                CTA_FCTA_X_BREAKTHROUGH_FILTER);
        }
        // set TTC threshold
        pFCTAGlobal->fTTCThreshold_s[uWarnLevel] = fTTCThreshold;
        // set max object range
        pFCTAGlobal->fMaxObjRange_met[uWarnLevel] = fTargetRangeMax;
    }
    // set common parameter
    pFCTAGlobal->fMaxHeadingAngle_deg =
        FCTAparams->fMaxHeadingAngle + CTA_FCTA_APPROACH_ANGLE_MARGIN;  // 10
    pFCTAGlobal->fMinHeadingAngle_deg =
        FCTAparams->fMinHeadingAngle - CTA_FCTA_APPROACH_ANGLE_MARGIN;
    // pFCTAGlobal->fVegoMin_mps = -2.78f;
    // pFCTAGlobal->fVegoMax_mps = 8.33f;
    pFCTAGlobal->fVTargetMin = FCTAparams->fVTargetMin;  // 0.83f;
    // pFCTAGlobal->fVTargetMax = FCTAparams->fVTargetMax;//100.f;
}
/*****************************************************************************
  Functionname: CTAFCTAInitCyclic                                    */ /*!

          @brief  CTA cyclic initialization. Initialize variables which are not
        stored across cycles.

          @description  CTA cyclic initialization. Initialize variables which
        are not stored across cycles.

          @param[in]

          @param[out] pbFCTAWarnActive  FCTA warn active pointer

          @return
        *****************************************************************************/
void CTAFCTAInitCyclic(boolean* pbFCTAWarnActive) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        pbFCTAWarnActive[uWarnLevel] = FALSE;
    }
    FCTAGlobal.iCriticalObjIDLastCycle_nu = FCTAGlobal.iCriticalObjID_nu;
    FCTAGlobal.iCriticalObjID_nu = -1;
    FCTAGlobal.fCriticalObjDistYLastCycle_met =
        FCTAGlobal.fCriticalObjDistY_met;
    FCTAGlobal.fCriticalObjDistY_met = TUE_C_F32_VALUE_INVALID;
    FCTAGlobal.fCriticalTTC_s = TUE_C_F32_VALUE_INVALID;
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void FCTAMainProcess(const FCTAInReq_t* FCTAreqPorts,
                     const FCTAParam_t* FCTAparams,
                     FCTAGlobal_t* pFCTAGlobal) {
    for (uint8 uObj = 0u; uObj < FCTA_MAX_NUM_OBJECTS; uObj++) {
        const FCTAEMFusionObjInReq_t* pEMFusionObjInput =
            &FCTAreqPorts->EMFusionObjListInput[uObj];
        FCTAObjGlobal_t* pFCTAObjGlobal =
            &pFCTAGlobal->FCTA_Va_ObjectListGlobal[uObj];
        const FCTACTObjInReq_t* pFCTACTObjInput =
            &FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj];
        const FCTACTAObjListInReq_t* pFCTACTAObjInput =
            &FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj];

        if (!pEMFusionObjInput->uiMaintenanceState_nu ==
            CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
            // Check if object is in range
            CTAFCTACheckObjectInRange(
                pEMFusionObjInput->fDistX_met, pEMFusionObjInput->fDistY_met,
                pFCTAGlobal->fMaxObjRange_met, pFCTAObjGlobal->bObjectInRange);
            // Check whether the object was updated in the last cycles
            CTAFCTACheckObjectUpdateRecently(
                pEMFusionObjInput->uiMeasuredTargetFrequency_nu,
                pFCTACTAObjInput->fUpdateRate_nu,
                &pFCTAObjGlobal->bUpdatedRecently);
            // Check angle of approach is in range
            CTAFCTACheckApproachAngle(pEMFusionObjInput->bRightSensor,
                                      pEMFusionObjInput->fRelHeading_rad,
                                      pFCTAGlobal->fMaxHeadingAngle_deg,
                                      pFCTAGlobal->fMinHeadingAngle_deg,
                                      pFCTAObjGlobal->bObjectFromSide,
                                      &pFCTAObjGlobal->bValidApproachAngle);
            // Check if an object which enables a warning is a possible mirror
            CTAFCTACheckForMirror(pEMFusionObjInput->fMirrorProb_per,
                                  &pFCTAObjGlobal->bMirror);
            // Check the object movement
            CTAFCTAObjectMovement(pEMFusionObjInput, pFCTACTAObjInput->fVabs,
                                  pFCTACTAObjInput->fVxPosBased,
                                  pFCTACTAObjInput->fVyPosBased,
                                  &pFCTAObjGlobal->bObjMovementValid);
            // Check whether object is relevant
            CTAFCTACheckObjectRelevance(
                uObj, pEMFusionObjInput,
                FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps, pFCTACTAObjInput,
                pFCTACTObjInput, pFCTAGlobal, pFCTAObjGlobal->bObjMovementValid,
                FCTAreqPorts->CTGlobalInput.fMaxLatSensorRange,
                pFCTAObjGlobal->bWarning, &pFCTAObjGlobal->bRelevant);
            // Calculate whether the object has the necessary quality
            CTAFCTACheckObjectQuality(
                pEMFusionObjInput,
                FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps,
                pFCTACTAObjInput->fUpdateRate_nu,
                pFCTACTAObjInput->fAssocProbFiltered_nu,
                &pFCTAObjGlobal->bQuality);
            // Check whether the object hits the breakthrough line
            CTAFCTACheckBreakthroughHit(
                pFCTAGlobal->fXMaxBreakthrough_met,
                pFCTAGlobal->fXMinBreakthrough_met,
                pFCTACTObjInput->fDistToCrossingLine_met,
                pFCTACTObjInput->fXBreakthrough_met, pFCTAObjGlobal->bWarning,
                pFCTAObjGlobal->bBreakthroughHit);
            // Update the confidence of the breakthrough hit
            CTAFCTACalculateBreakthroughHitConfidence(
                pFCTACTObjInput->fXBreakthrough_met,
                pFCTACTObjInput->fXBreakthroughStd_met, pFCTAGlobal,
                FCTAreqPorts, pFCTAObjGlobal->bObjectAtEdgeFoV,
                pFCTAObjGlobal->bBreakthroughHit,
                pFCTAObjGlobal->uBreakthroughHitConfi);
            // Check the hysteresis timer(smoothing of warning ON/OFF)
            CTAFCTAUpdateBTHitHysteresisTimer(
                FCTAreqPorts->fCycleTime_s, pFCTACTObjInput->fTTC_s,
                pFCTACTObjInput->fTTCFiltered_s, pFCTAObjGlobal->bWarning,
                pFCTAObjGlobal->bBreakthroughHit,
                pFCTAObjGlobal->bWarningLastCycle,
                pFCTAObjGlobal->fBTHitHystTimer_s);
            // Check the hysteresis timer condition (unstable object
            // trajectories do not result in unstable warnings)
            CTAFCTACheckBTHitHysteresisTimer(pFCTAObjGlobal->fBTHitHystTimer_s,
                                             pFCTAObjGlobal->bBTHitHystActive);
            // Check the TTC condition
            CTAFCTACheckTTC(
                pFCTACTObjInput->fTTC_s, pFCTACTObjInput->fTTCFiltered_s,
                pFCTAGlobal->fTTCThreshold_s, pEMFusionObjInput->bRightSensor,
                pFCTAObjGlobal->bObjectAtEdgeFoV, pEMFusionObjInput->fDistY_met,
                pFCTAObjGlobal->bTTCBelowThresh);
            // Check the short warning condition
            CTAFCTACheckShortWarning(
                pFCTACTObjInput->fTTC_s, pFCTACTObjInput->fTTCFiltered_s,
                pFCTACTObjInput->fDistToCrossingLine_met,
                pFCTAObjGlobal->bWarning, &pFCTAObjGlobal->bShortWarning);
            // Decide whether the object shall warn
            CTAFCTAWarningDecision(pFCTAObjGlobal, pEMFusionObjInput->fRCS,
                                   pFCTACTObjInput->bRearTrack_nu,
                                   pFCTAObjGlobal->bWarningLastCycle,
                                   pFCTAObjGlobal->bWarning);
            // Set the global warning
            CTAFCTASetGlobalWarning(
                pEMFusionObjInput->iFusionID, pEMFusionObjInput->fDistY_met,
                pEMFusionObjInput->fWidthLeft_met, pFCTAObjGlobal->bWarning,
                pFCTACTObjInput->fTTCFiltered_s, pFCTAGlobal);
        }
    }

    // FCTA state machine
    CTAFCTAStateMachine(FCTAreqPorts, FCTAparams, pFCTAGlobal);
}
/*****************************************************************************
  Functionname:CTAFCTACheckObjectInRange                                     */ /*!

  @brief  Check for valid FCTA range

  @description  The range check prevents targets which may turn away from
warning

  @param[in] fDistX_met  Object's longitudinal relative distance
             fDistY_met  Object's lateral relative distance
                         fMaxObjRange_met  the max threshold of object range

  @param[out] bObjectInRange  Flag whether the object is in the range

  @return
*****************************************************************************/
void CTAFCTACheckObjectInRange(float32 fDistX_met,
                               float32 fDistY_met,
                               float32* fMaxObjRange_met,
                               boolean* pbObjectInRange) {
    // calculate the distance of the object
    float32 fRangeSqr = SQR(fDistX_met) + SQR(fDistY_met);

    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        boolean bInRange = FALSE;
        // check whether the object is in range
        if (fRangeSqr < SQR(fMaxObjRange_met[uWarnLevel])) {
            bInRange = TRUE;
        }
        pbObjectInRange[uWarnLevel] = bInRange;
    }
}
/*****************************************************************************
  Functionname: CTAFCTACheckObjectUpdateRecently */ /*!

                              @brief  Check the current update status of the
                            object

                              @description  Check the current update status of
                            the object

                              @param[in] uiMeasuredTargetFrequency_nu  Bitfield
                            to indicate if the object was measured in the last 8
                            cycles
                                         fUpdateRate_nu  The object measurement
                            update rate,unit:NULL

                              @param[out] pbUpdatedRecently  Flag whether the
                            object is updated recently

                              @return
                            *****************************************************************************/
void CTAFCTACheckObjectUpdateRecently(uint8 uiMeasuredTargetFrequency_nu,
                                      float32 fUpdateRate_nu,
                                      boolean* pbUpdatedRecently) {
    boolean bUpdateRecently = FALSE;
    if ((uiMeasuredTargetFrequency_nu & 192u) == 192) {
        bUpdateRecently = TRUE;
    } else {
        if (fUpdateRate_nu > CTA_FCTA_UPDRATE_MIN &&  // 0.8
            ((uiMeasuredTargetFrequency_nu & 128u) == 128u ||
             (uiMeasuredTargetFrequency_nu & 64u) == 64u)) {
            bUpdateRecently = TRUE;
        }
    }
    *pbUpdatedRecently = bUpdateRecently;
}
/*****************************************************************************
  Functionname:CTAFCTACheckApproachAngle                                     */ /*!

  @brief Check for valid RCTA approach angle

  @description Check for valid RCTA approach angle

  @param[in]  bRightSensor:Flag whether the object is detected by the right
sensor
                          fAbsOrientation_rad:Heading Angle
                          fMaxHeadingAngle_deg:Calculate the max value of
heading angle
                          fMinHeadingAngle_deg:Calculate the min value of
heading angle
                          bObjectFromSide: Flag whether the object approach from
side
                          bValidApproachAngle:Flag whether the object heading
angle is valid in the last cycle
  @param[out] bValidApproachAngle:Flag whether the object heading angle is valid
in the current cycle

  @return
*****************************************************************************/
void CTAFCTACheckApproachAngle(const boolean bRightSensor,
                               const float32 fAbsOrientation_rad,
                               float32 fMaxHeadingAngle_deg,
                               float32 fMinHeadingAngle_deg,
                               boolean bObjectFromSide,
                               boolean* bValidApproachAngle) {
    float32 fMaxHeadingAngle = fMaxHeadingAngle_deg;
    float32 fMinHeadingAngle = fMinHeadingAngle_deg;
    boolean bValidAngle = FALSE;
    float32 fHeading = fAbsOrientation_rad;

    // check if on right sensor, revert the sign of the heading for using the
    // old coordinate system
    if (bRightSensor) {
        fHeading = fHeading * -1.f;
    }
    // Check whether hysteresis shall be applied
    if (*bValidApproachAngle) {
        // add hysteresis
        fMaxHeadingAngle += FCTA_APPROACH_ANGLE_HYST;  // 10
        fMinHeadingAngle -= FCTA_APPROACH_ANGLE_HYST;
    }

    // side targets use the customer defined parameters for max rear crossing
    // angles
    if (bObjectFromSide) {
        if (fHeading < DEG2RAD(fMaxHeadingAngle) &&
            fHeading > DEG2RAD(fMinHeadingAngle)) {
            bValidAngle = TRUE;
        }
    } else {
        // to inhibit false alerts, targets which appear/approach from behind
        // use
        // tighter angle requirements
        if (fHeading < DEG2RAD(fMaxHeadingAngle) &&
            fHeading > DEG2RAD(fMinHeadingAngle)) {
            bValidAngle = TRUE;
        }
    }
    *bValidApproachAngle = bValidAngle;
}
/*****************************************************************************
  Functionname:CTAFCTACheckForMirror                                     */ /*!

      @brief Check if object has sufficient mirror probability

      @description Check if object has sufficient mirror probability

      @param[in]  fMirrorProb_per: The probability that the object is mirror
                              pbMirror: Flag whether the object is mirror in the
    last cycle
      @param[out] pbMirror: Flag whether the object is mirror in the current
    cycle

      @return
    *****************************************************************************/
void CTAFCTACheckForMirror(float32 fMirrorProb_per, boolean* pbMirror) {
    boolean bMirror = FALSE;
    if (!(*pbMirror)) {
        if (fMirrorProb_per > FCTA_MIRROR_ACTIVATION_THRESH) {
            // Object has sufficient mirror probability, set mirror flag
            bMirror = TRUE;
        }
    } else {
        if (fMirrorProb_per > FCTA_MIRROR_DEACTIVATION_THRESH) {
            // Object has sufficient mirror probability, keep mirror flag
            bMirror = TRUE;
        }
    }
    *pbMirror = bMirror;
}
/*****************************************************************************
  Functionname:CTAFCTAObjectMovement                                     */ /*!

      @brief  Check if the object movement is valid based on the difference
    between position and tracking based velocity

      @description  Check if the object movement is valid based on the
    difference between position and tracking based velocity

      @param[in]  pEMFusionObjInput
                  fVabs
                              fVxPosBased
                              fVyPosBased
                              pbObjMovementValid
      @param[out] pbObjMovementValid

      @return
    *****************************************************************************/
void CTAFCTAObjectMovement(const FCTAEMFusionObjInReq_t* pEMFusionObjInput,
                           float32 fVabs,
                           float32 fVxPosBased,
                           float32 fVyPosBased,
                           boolean* pbObjMovementValid) {
    float32 fDistY = pEMFusionObjInput->fDistY_met;
    float32 fVrelY = pEMFusionObjInput->fVrelY_mps;
    const float32 fVxDiff = fABS(pEMFusionObjInput->fVrelX_mps - fVxPosBased);
    const float32 fVyDiff = fABS(pEMFusionObjInput->fVrelY_mps - fVyPosBased);
    boolean bObjMovementValid = *pbObjMovementValid;
    // check if on right sensor, revert the sign of the Y distance and Vy for
    // using the old coordinate system
    if (pEMFusionObjInput->bRightSensor) {
        fDistY = fDistY * -1.f;
        fVrelY = fVrelY * -1.f;
    }
    if (*pbObjMovementValid) {
        if (fDistY <
                FCTA_OBJ_VALID_DISTY_MAX &&  // only check the object movement
                                             // at close distances    //5
            fABS(pEMFusionObjInput->fArelX_mpss) <
                FCTA_OBJ_VALID_ARELX_MAX &&  // restrict comparison between
                                             // tracked kinematics and position
                                             // based kinematics to small
                                             // acceleration  //3.5
            fABS(pEMFusionObjInput->fArelY_mpss) <
                FCTA_OBJ_VALID_ARELY_MAX &&       // 3.5
            fVrelY > FCTA_OBJ_VALID_VRELY_MIN &&  // only check the movement for
                                                  // small velocities in y
                                                  // direction   //-2
            pEMFusionObjInput->fVrelY_mps < FCTA_OBJ_VALID_VRELY_MAX &&
            fVabs < FCTA_OBJ_VALID_VABS_MAX) {  // restrict to small absolute
                                                // object velocities
            if (fVxDiff > FCTA_VX_DIFF_MOV_INVALID ||  // 1
                fVyDiff > FCTA_VX_DIFF_MOV_INVALID) {
                bObjMovementValid = FALSE;
            }
        }
    } else {
        // if the object movement is detected as invalid check for smaller
        // differences between position
        if (fDistY > FCTA_OBJ_VALID_DISTY_TRSH &&
            fVxDiff < FCTA_VX_DIFF_MOV_VALID &&
            fVyDiff < FCTA_VX_DIFF_MOV_VALID) {
            bObjMovementValid = TRUE;
        }
    }
    *pbObjMovementValid = bObjMovementValid;
}
/*****************************************************************************
  Functionname: CTAFCTACheckObjectRelevance */ /*!

                                   @brief  Check if the tracked object is
                                 relevant(exists) for a warning

                                   @description  Check if the tracked object is
                                 relevant(exists) for a warning

                                   @param[in]

                                   @param[out]

                                   @return
                                 *****************************************************************************/
void CTAFCTACheckObjectRelevance(
    uint8 uObj,
    const FCTAEMFusionObjInReq_t* pEMFusionObjInput,
    float32 fegoVelocity_mps,
    const FCTACTAObjListInReq_t* pFCTACTAObjInput,
    const FCTACTObjInReq_t* pFCTACTObjInput,
    FCTAGlobal_t* pFCTAGlobal,
    boolean bObjMovementValid,
    float32 fMaxLatSensorRange,
    boolean* pbWarning,
    boolean* pbRelevant) {
    float32 fMovementAbsSqrThresh;
    float32 fSpeedOverGround;
    float32 fMovementAbsSqr;
    float32 fSpeedOverGroundThresh;
    float32 fDistY = pEMFusionObjInput->fDistY_met;
    boolean bMovementCloseObjValid = TRUE;
    boolean bRelevant = FALSE;
    // ghost objects created at the side of the subject may cause issues
    if (pEMFusionObjInput->fFirstDetectX_met < FCTA_REL_MAX_FIRSTX &&  // 5
        pEMFusionObjInput->fFirstDetectX_met > FCTA_REL_MIN_FIRSTX &&  // -10
        fMaxLatSensorRange > FCTA_REL_MAX_LAT_SEN_RANGE &&
        fegoVelocity_mps > FCTA_VEGO_THRESH_STAT2MOV) {
        fMovementAbsSqrThresh = TUE_CML_BoundedLinInterpol2(
            pEMFusionObjInput->fFirstDetectY_met, FCTA_LI_MOVTRSH_MIN_DISTY,
            FCTA_LI_MOVTRSH_MAX_DISTY, FCTA_LI_MOVTRSH_MIN_THRESH,
            FCTA_LI_MOVTRSH_MAX_THRESH);  // 5  20  8  4
    } else {
        fMovementAbsSqrThresh = TUE_CML_BoundedLinInterpol2(
            fegoVelocity_mps, FCTA_LI_MOVTRSH_MIN_SPD, FCTA_LI_MOVTRSH_MAX_SPD,
            FCTA_LI_MOVTRSH_MIN_MOVTRSH,
            FCTA_LI_MOVTRSH_MAX_MOVTRSH);  // 0  3  2  4
    }
    // compute track's overall velocity over ground
    fSpeedOverGround = pFCTACTAObjInput->fVabs;
    // compute track's overall movement over ground
    fMovementAbsSqr = SQR(pFCTACTAObjInput->fXMovement_met) +
                      SQR(pFCTACTAObjInput->fYMovement_met);
    if (*pbRelevant) {
        fSpeedOverGroundThresh =
            pFCTAGlobal->fVTargetMin * FCTA_SPDOVERGND_RELEVANT_FACTOR;  // 0.5
    } else {
        fSpeedOverGroundThresh = pFCTAGlobal->fVTargetMin;
    }
    // check if on right sensor, revert the sign of the Y distance and Vy for
    // using the old coordinate system
    if (pEMFusionObjInput->bRightSensor) {
        fDistY = fDistY * -1.f;
    }
    // For close objects check if some relevant movement is detected to avoid FA
    // on close objects with high TTC fluctuation
    if (fDistY < FCTA_MAX_Y_GEN_RELEVANCE &&                            // 5
        pEMFusionObjInput->uiLifeCycles_nu > FCTA_REL_LIFECYCLE_MIN &&  // 100
        fABS(pEMFusionObjInput->fVrelX_mps) < FCTA_REL_VRELX_MAX &&
        !pbWarning[0]) {
        float32 fDist2CrossingLineDiff;
        fDist2CrossingLineDiff =
            fABS(pFCTACTObjInput->fDistToCrossingLine_met -
                 pFCTACTObjInput->fDistToCrossingLineFiltered_met);

        if (fDist2CrossingLineDiff < (0.5f * FCTA_MAX_Y_GEN_RELEVANCE)) {
            bMovementCloseObjValid = FALSE;
        }
    }

    if (fSpeedOverGround > fSpeedOverGroundThresh &&
        pEMFusionObjInput->uiLifeCycles_nu >= FCTA_MIN_LIFECYCLE_RELEVANT &&
        fMovementAbsSqr > fMovementAbsSqrThresh && bMovementCloseObjValid &&
        bObjMovementValid) {
        bRelevant = TRUE;
    }
    *pbRelevant = bRelevant;
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTAFCTACheckObjectQuality(const FCTAEMFusionObjInReq_t* pEMFusionObjInput,
                               float32 fegoVelocity,
                               float32 fUpdateRate_nu,
                               float32 fAssocProbFiltered_nu,
                               boolean* pbQuality) {
    boolean bObjQuality = FALSE;
    float32 fPoEThresh;
    float32 fPoEThreshLifetime;
    float32 fUpdateRateThresh;
    float32 fUpdateRateThreshLifetime;
    float32 fAssocProbThresh;
    float32 fAssocProbThreshRange;
    float32 fRangeSqr;
    float32 fSpeedOverGroundSqr;
    float32 fEgoSpeedRedFactor;

    // Calculate object's speed over ground square
    fSpeedOverGroundSqr =
        SQR(pEMFusionObjInput->fVabsX_mps) + SQR(pEMFusionObjInput->fVabsY_mps);
    // Calculate PoE threshold
    fPoEThresh = TUE_CML_BoundedLinInterpol2(
        fSpeedOverGroundSqr, SQR(FCTA_LI_POE_MIN_SPEED),
        SQR(FCTA_LI_POE_MAX_SPEED), FCTA_LI_POE_MIN_POE,
        FCTA_LI_POE_MAX_POE);  // 30/3.6  60/3.6  0.99  0.93
    fPoEThreshLifetime = TUE_CML_BoundedLinInterpol2(
        (float32)pEMFusionObjInput->uiLifeCycles_nu,
        FCTA_LI_POE_LFT_MIN_LIFETIME, FCTA_LI_POE_LFT_MAX_LIFETIME,
        FCTA_LI_POE_LFT_MIN_POE,
        FCTA_LI_POE_LFT_MAX_POE);  // 15  25  0.93  0.99
    fPoEThresh = MAX(fPoEThresh, fPoEThreshLifetime);
    // Calculate update rate threshold
    fUpdateRateThresh = TUE_CML_BoundedLinInterpol2(
        fSpeedOverGroundSqr, SQR(FCTA_LI_UPDATE_MIN_SPEED),
        SQR(FCTA_LI_UPDATE_MAX_SPEED), FCTA_LI_UPDATE_MIN_POE,
        FCTA_LI_UPDATE_MAX_POE);  // 30/3.6  60/3.6  0.85  0.75
    fUpdateRateThreshLifetime = TUE_CML_BoundedLinInterpol2(
        (float32)pEMFusionObjInput->uiLifeCycles_nu,
        FCTA_LI_UPDATE_LFT_MIN_LIFETIME, FCTA_LI_UPDATE_LFT_MAX_LIFETIME,
        FCTA_LI_UPDATE_LFT_MIN_UPDATE,
        FCTA_LI_UPDATE_LFT_MAX_UPDATE);  // 15  25  0.75  0.85
    fUpdateRateThresh = MAX(fUpdateRateThresh, fUpdateRateThreshLifetime);
    // Calculate the association probability threshold
    // The association probability threshold should be speed and distance
    // dependent This makes sure that fast objects are allowed to warn early
    // enough, but mostly close ghost objects do not reach the threshold
    fAssocProbThresh = TUE_CML_BoundedLinInterpol2(
        fSpeedOverGroundSqr, SQR(FCTA_LI_ASSOCP_MIN_SPEED),
        SQR(FCTA_LI_ASSOCP_MAX_SPEED), FCTA_LI_ASSOCP_MIN_ASSOCP,
        FCTA_LI_ASSOCP_MAX_ASSOCP);  // 10/3.6  50/3.6  0.8  0.3
    fRangeSqr =
        SQR(pEMFusionObjInput->fDistX_met) + SQR(pEMFusionObjInput->fDistY_met);
    fAssocProbThreshRange = TUE_CML_BoundedLinInterpol2(
        fRangeSqr, SQR(FCTA_LI_ASSOCP_RNG_MIN_RANGE),
        SQR(FCTA_LI_ASSOCP_RNG_MAX_RANGE), FCTA_LI_ASSOCP_RNG_MIN_ASSOCP,
        FCTA_LI_ASSOCP_RNG_MAX_ASSOCP);  // 40 30 0.4 1
    fAssocProbThresh = MIN(fAssocProbThresh, fAssocProbThreshRange);

    fEgoSpeedRedFactor = TUE_CML_BoundedLinInterpol2(
        fABS(fegoVelocity), CTA_FCTA_LI_REDEGO_MIN_EGOSPEED,
        CTA_FCTA_LI_REDEGO_MAX_EGOSPEED, CTA_FCTA_LI_REDEGO_MIN_FACTOR,
        CTA_FCTA_LI_REDEGO_MAX_FACTOR);  // 0  3  0.9  1

    if (fABS(fegoVelocity) < TUE_C_F32_DELTA) {
        fPoEThresh *= fEgoSpeedRedFactor;
        fUpdateRateThresh *= fEgoSpeedRedFactor;
        fAssocProbThresh *= fEgoSpeedRedFactor;
    }

    // Apply hysteresis
    if (*pbQuality) {
        fPoEThresh -= FCTA_POE_HYST;            // 0.02
        fUpdateRateThresh -= FCTA_UPDRTE_HYST;  // 0.1
        fAssocProbThresh -= FCTA_ASSOCP_HYST;   // 0.1
    }
    // Check all quality criteria
    if (pEMFusionObjInput->fProbabilityOfExistence_per > fPoEThresh &&
        fUpdateRate_nu > fUpdateRateThresh - 0.4f &&
        fAssocProbFiltered_nu > fAssocProbThresh - 0.5f &&
        pEMFusionObjInput->bObjStable) {
        bObjQuality = TRUE;
    }

    *pbQuality = bObjQuality;
}
/*****************************************************************************
  Functionname:CTAFCTACheckBreakthroughHit                                    */ /*!

 @brief Check for object breakthrough hit

 @description Check for object trajectory intersection with rear crossing line

 @param[in]  fXMaxBreakthrough_met:x-axis min edge of breakthrough
                         fXMinBreakthrough_met:x-axis max edge of breakthrough
                         fDistToCrossingLine_met
                         fXBreakthrough_met:The x-axis breakthrough of the
object
                         pbWarning:Flag whether the RCTA warning is activated
 @param[out] pbBreakthroughHit:Flag whether the object hits breakthrough

 @return
*****************************************************************************/
void CTAFCTACheckBreakthroughHit(float32* fXMaxBreakthrough_met,
                                 float32* fXMinBreakthrough_met,
                                 float32 fDistToCrossingLine_met,
                                 float32 fXBreakthrough_met,
                                 boolean* pbWarning,
                                 boolean* pbBreakthroughHit) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        boolean bBreakthroughHit = FALSE;
        float32 fMaxBreakthrough = fXMaxBreakthrough_met[uWarnLevel];
        float32 fMinBreakthrough = fXMinBreakthrough_met[uWarnLevel];
        // The object already passed the crossing line
        if (fDistToCrossingLine_met > 0.0f) {
            // Switch on hysteresis
            if (pbWarning[uWarnLevel]) {
                fMaxBreakthrough += FCTA_BREAKTHROUGH_HYSTERESIS;  // 2
                fMinBreakthrough -= FCTA_BREAKTHROUGH_HYSTERESIS;
            }
            // Test breakthrough in range
            if (fXBreakthrough_met < fMaxBreakthrough &&
                fXBreakthrough_met > fMinBreakthrough) {
                bBreakthroughHit = TRUE;
            }
        }
        pbBreakthroughHit[uWarnLevel] = bBreakthroughHit;
    }
}
/*****************************************************************************
Functionname:CTAFCTACalculateBreakthroughHitConfidence* / /*!

@brief Calculate breakthrough hit confidence for an object

@description Calculate breakthrough hit confidence for an object

@param[in]  fXBreakthrough_met:The y-axis breakthrough of the object
                        fXBreakthroughStd_met:The y-axis breakthrough standard
deviation of the object fXMinBreakthrough_met:x-axis min edge of breakthrough
                        fXMaxBreakthrough_met:x-axis max edge of breakthrough
                        pbBreakthroughHit:Flag whether the object hits
breakthrough
@param[out] uBreakthroughHitConfi:Breakthrough hit confidence

@return
*****************************************************************************/
void CTAFCTACalculateBreakthroughHitConfidence(float32 fXBreakthrough_met,
                                               float32 fXBreakthroughStd_met,
                                               FCTAGlobal_t* pFCTAGlobal,
                                               const FCTAInReq_t* FCTAreqPorts,
                                               boolean bObjectAtEdgeFoV,
                                               boolean* pbBreakthroughHit,
                                               uint8* puBreakthroughHitConfi) {
    float32 fMaxConfUpdate = FCTA_CONFI_UPDATE_MAX;  // 20
    float32 fMinConfUpdate = FCTA_CONFI_UPDATE_MIN;  // 5

    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        float32 fHitConfidence;
        float32 fHitConfidenceUpdate;
        float32 fBreakthroughHitRatio;
        float32 fMaxHitLimit;
        float32 fMinHitLimit;
        float32 fMaxBreakthroughLimit;
        float32 fMinBreakthroughLimit;
        fHitConfidence = puBreakthroughHitConfi[uWarnLevel];
        // Get the min and max point where the object hits the breakthrough line
        fMaxBreakthroughLimit = fXBreakthrough_met + fXBreakthroughStd_met;
        fMinBreakthroughLimit = fXBreakthrough_met - fXBreakthroughStd_met;
        fMaxHitLimit = CTAFCTACalculateHitLimit(
            fMaxBreakthroughLimit,
            pFCTAGlobal->fXMaxBreakthrough_met[uWarnLevel],
            pFCTAGlobal->fXMinBreakthrough_met[uWarnLevel],
            pbBreakthroughHit[uWarnLevel]);
        fMinHitLimit = CTAFCTACalculateHitLimit(
            fMinBreakthroughLimit,
            pFCTAGlobal->fXMaxBreakthrough_met[uWarnLevel],
            pFCTAGlobal->fXMinBreakthrough_met[uWarnLevel],
            pbBreakthroughHit[uWarnLevel]);
        // Calculate the ratio of the length where the object will probably hit
        // the breakthrough and the uncertainty of the hit estimation
        fBreakthroughHitRatio = (fMaxHitLimit - fMinHitLimit) /
                                SafeDiv(2.0f * fXBreakthroughStd_met);
        // Calculate the ratio of length where the object will probably hit the
        // breakthrough and the uncertainty of the hit estimation
        if (bObjectAtEdgeFoV ||
            (fABS(FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps) >
             FCTA_BREAKTHROUGH_MIN_SPEED) &&
                (fABS(FCTAreqPorts->FCTARoadInformation.fCurveRadius_met) <
                 FCTA_BREAKTHROUGH_MAX_CURVE)) {
            // The confidence should be increased or decreased slower if the
            // object is at the edge of the FoV
            fMaxConfUpdate = FCTA_CONFI_UPDATE_MAX_EFOV;
            fMinConfUpdate = FCTA_CONFI_UPDATE_MIN_EFOV;
        }
        // Breakthrough was not hit
        if (!pbBreakthroughHit[uWarnLevel]) {
            // The confidence should be decreased. The less the object hits the
            // breakthrough the faster the confidence shall decrease
            fHitConfidenceUpdate = TUE_CML_BoundedLinInterpol2(
                fBreakthroughHitRatio, 0.0f, 1.0f, -fMaxConfUpdate,
                -fMinConfUpdate);  // 5   20
        } else {
            // The confidence should be increased. The more the object hits the
            // breakthrough the faster the confidence shall increase
            fHitConfidenceUpdate =
                TUE_CML_BoundedLinInterpol2(fBreakthroughHitRatio, 0.0f, 1.0f,
                                            fMinConfUpdate, fMaxConfUpdate);
        }
        fHitConfidence += fHitConfidenceUpdate;
        fHitConfidence = TUE_CML_MinMax(0.0f, 100.0f, fHitConfidence);

        puBreakthroughHitConfi[uWarnLevel] = (uint8)fHitConfidence;
    }
}
/*****************************************************************************
  Functionname: CTAFCTACalculateHitLimit */ /*!

                                      @brief: Limit the expected breakthrough
                                    between the upper and the lower

                                      @description: Limit the expected
                                    breakthrough between the upper and the lower

                                      @param[in]  fBreakthroughLimit
                                    pfYBreakthrough[uLine] +
                                    pfYBreakthroughStd[uLine]
                                                              fYMaxBreakthrough
                                    y-axis max edge of breakthrough
                                                              fYMinBreakthrough
                                    y-axis min edge of breakthrough
                                                              bBreakthroughHit
                                    Flag whether the object hits breakthrough in
                                    the current cycle
                                      @param[out] void

                                      @return     fRet
                                    Limited value of breakthrough
                                    *****************************************************************************/
float32 CTAFCTACalculateHitLimit(float32 fBreakthroughLimit,
                                 float32 fXMaxBreakthrough,
                                 float32 fXMinBreakthrough,
                                 boolean bBreakthroughHit) {
    float32 fBreakthroughHyst = 0.0f;
    float32 fRet;

    // Add hysteresis of breakthrough
    if (bBreakthroughHit) {
        fBreakthroughHyst = FCTA_BREAKTHROUGH_HYSTERESIS;  // 2
    }

    if (fBreakthroughLimit > (fXMaxBreakthrough + fBreakthroughHyst)) {
        // The limit of the expected breakthrough is above the upper
        // breakthrough limit
        fRet = fXMaxBreakthrough;
    } else if (fBreakthroughLimit < (fXMinBreakthrough - fBreakthroughHyst)) {
        // The limit of the expected breakthrough is below the lower
        // breakthrough limit
        fRet = fXMinBreakthrough;
    } else {
        // The limit of the expected breakthrough is within the defined
        // breakthrough
        fRet = fBreakthroughLimit;
    }
    return fRet;
}
/*****************************************************************************
  Functionname: CTAFCTAUpdateBTHitHysteresisTimer */ /*!

                             @brief Update the warning hysteresis timer

                             @description Update the warning hysteresis timer to
                           prevent unstable per object warnings
                                                      and to dead reckon to
                           exact warning termination time

                             @param[in]  fCycleTime_s:Current task cycle time
                           from EMGlobalOutput
                                                     fTTC_s:TTC of the object
                                                     fTTCFiltered_s:Filtered TTC
                           of the object
                                                     bWarning:Flag whether the
                           RCTA warning is activated
                                                     bBreakthroughHit:Flag
                           whether the object hits breakthrough in the current
                           cycle
                                                     bWarningLastCycle:Flag
                           whether the RCTA warning is activated in the last
                           cycle
                             @param[out] fBTHitHystTimer_s:the warning
                           hysteresis timer

                             @return
                           *****************************************************************************/
void CTAFCTAUpdateBTHitHysteresisTimer(float32 fCycleTime_s,
                                       float32 fTTC_s,
                                       float32 fTTCFiltered_s,
                                       boolean* bWarning,
                                       boolean* bBreakthroughHit,
                                       boolean* bWarningLastCycle,
                                       float32* pfBTHitHystTimer_s) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        // When a warning starts or when the breakthrough was hit during a
        // warning
        if (bWarning[uWarnLevel] &&
            (bBreakthroughHit[uWarnLevel] || !bWarningLastCycle[uWarnLevel])) {
            // Set the dead reckon timer to the calculated TTC
            pfBTHitHystTimer_s[uWarnLevel] = MIN(fTTC_s, fTTCFiltered_s);
        } else {
            // Run the timer
            pfBTHitHystTimer_s[uWarnLevel] -= fCycleTime_s;
        }
        // Limit it to 1s
        pfBTHitHystTimer_s[uWarnLevel] =
            TUE_CML_MinMax(0.0f, 1.0f, pfBTHitHystTimer_s[uWarnLevel]);
    }
}
/*****************************************************************************
  Functionname:CTAFCTACheckBTHitHysteresisTimer */ /*!

                               @brief Check warning against hysteresis timer

                               @description Check warning against hysteresis
                             timer

                               @param[in]  fBTHitHystTimer_s:the warning
                             hysteresis timer

                               @param[out] bBTHitHystActive:Flag whether the
                             warning hysteresis

                               @return
                             *****************************************************************************/
void CTAFCTACheckBTHitHysteresisTimer(float32* fBTHitHystTimer_s,
                                      boolean* pbBTHitHystActive) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        boolean bBTHitHystActive = FALSE;
        // As long as the time is not elapsed set hysteresis to active
        if (fBTHitHystTimer_s[uWarnLevel] > TUE_C_F32_DELTA) {
            bBTHitHystActive = TRUE;
        }
        pbBTHitHystActive[uWarnLevel] = bBTHitHystActive;
    }
}
/*****************************************************************************
  Functionname:CTAFCTACheckTTC                                    */ /*!

             @brief Check the object's TTC

             @description Check the object's TTC for possible OSE warning start

             @param[in]  fTTC_s:TTC of the object
                                     fTTCFiltered_s:Filtered TTC of the object
                                     fTTCThreshold_s:TTC thresholds of three
           levels
             @param[out] bTTCBelowThresh:Flag whether the object's TTC is below
           TTC Threshold

             @return
           *****************************************************************************/
void CTAFCTACheckTTC(float32 fTTC_s,
                     float32 fTTCFiltered_s,
                     float32* pfTTCThreshold_s,
                     boolean bRightSensor,
                     boolean bObjectAtEdgeFoV,
                     float32 fDistY_met,
                     boolean* pbTTCBelowThresh) {
    float32 fDistY = fDistY_met;

    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        boolean bTTCBelowThresh = FALSE;
        float32 fTTCThreshold = pfTTCThreshold_s[uWarnLevel];
        // The TTC was already below the threshold. Use hysteresis
        if (pbTTCBelowThresh[uWarnLevel]) {
            if (fTTC_s < (2.0f * fTTCThreshold) ||
                fTTCFiltered_s < (2.0 * fTTCThreshold)) {
                bTTCBelowThresh = TRUE;
            }
        } else {
            // Limit the threshold if the object is at the edge of FoV
            if (bObjectAtEdgeFoV) {
                fTTCThreshold = MIN(fTTCThreshold, FCTA_TTC_EFOV_THRESH);
            }
            // check if on right sensor, revert the sign of the Y distance and
            // Vy for using the old coordinate system
            if (bRightSensor) {
                fDistY = fDistY * -1.f;
            }
            // for close objects the general and the filtered TTC have to be
            // below the TTC threshold
            if (fDistY > FCTA_MAX_Y_GEN_RELEVENCE) {
                if (fTTC_s < fTTCThreshold || fTTCFiltered_s < fTTCThreshold) {
                    bTTCBelowThresh = TRUE;
                }
            } else {
                if (fTTC_s < fTTCThreshold && fTTCFiltered_s < fTTCThreshold) {
                    bTTCBelowThresh = TRUE;
                }
            }
        }
        pbTTCBelowThresh[uWarnLevel] = bTTCBelowThresh;
    }
}
/*****************************************************************************
  Functionname:CTAFCTACheckShortWarning                                     */ /*!

   @brief Check the object's TTC

   @description Check the object's TTC for possible OSE warning start

   @param[in]  fTTC_s:TTC of the object
                           fTTCFiltered_s:Filtered TTC of the object
   @param[out] bShortTTC:Flag whether object's TTC is below 0

   @return
 *****************************************************************************/
void CTAFCTACheckShortWarning(float32 fTTC_s,
                              float32 fTTCFiltered_s,
                              float32 fDistToCrossingLine_met,
                              boolean* bWarning,
                              boolean* pbShortWarning) {
    boolean bShortWarning = FALSE;

    // if no warning is enabled yet, check for potentially very short warnings
    if (!bWarning[0]) {
        if (fTTC_s < FCTA_SHORT_WARNING_THRESH ||
            fTTCFiltered_s < FCTA_SHORT_WARNING_THRESH) {
            bShortWarning = TRUE;
        } else {
            if (fDistToCrossingLine_met < FCTA_SHORT_WARN_DIST_THRESH &&  // 1
                fTTC_s < FCTA_SHORTWARN_CLOSEOBJ_TTC_THR) {
                bShortWarning = TRUE;
            }
        }
    }
    *pbShortWarning = bShortWarning;
}
/*****************************************************************************
  Functionname:CTAFCTAWarningDecision                                     */ /*!

     @brief Decision if this object causes a RCTA warning

     @description Decision if this track causes a FCTA warning by checking
   breakthrough and TTC,
                              consider hysteresis and detect cluster hopper here
   and save ID of track

     @param[in]  pFCTAObjGlobal  FCTA object information global structure
                 fRCS  RCS
                             bRearTrack_nu
                             pbWarning  Flag whether the RCTA warning was
   activated in the last cycle
     @param[out] pbWarning  Flag whether the RCTA warning was activated in the
   current cycle
                 bWarningLastCycle:Flag whether the RCTA warning was activated
   in the last cycle

     @return
   *****************************************************************************/
void CTAFCTAWarningDecision(FCTAObjGlobal_t* pFCTAObjGlobal,
                            float32 fRCS,
                            boolean bRearTrack_nu,
                            boolean* pbWarningLastCycle,
                            boolean* pbWarning) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        boolean bWarning = FALSE;
        // Check warning start conditions
        if (!pbWarning[uWarnLevel]) {
            if (pFCTAObjGlobal->uBreakthroughHitConfi[uWarnLevel] >=
                    FCTA_HIT_CONFI_THRESH &&
                pFCTAObjGlobal->bObjectInRange[uWarnLevel] &&
                pFCTAObjGlobal->bRelevant &&
                pFCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
                !pFCTAObjGlobal->bMirror && !pFCTAObjGlobal->bShortWarning &&
                !bRearTrack_nu && pFCTAObjGlobal->bValidApproachAngle &&
                pFCTAObjGlobal->bUpdatedRecently && pFCTAObjGlobal->bQuality &&
                fRCS > FCTA_RCS_THRESH) {
                bWarning = TRUE;
            }
        } else {  // Check continued warning condition
            if (pFCTAObjGlobal->bRelevant &&
                pFCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
                pFCTAObjGlobal->bBTHitHystActive[uWarnLevel]) {
                bWarning = TRUE;
            }
        }
        // Store current warning info to detect position edge
        pFCTAObjGlobal->bWarningLastCycle[uWarnLevel] = pbWarning[uWarnLevel];
        pbWarning[uWarnLevel] = bWarning;
    }
}
/*****************************************************************************
  Functionname:CTAFCTASetGlobalWarning                                     */ /*!

    @brief Global warning decision

    @description Decision if this track causes a FCTA warning by checking
  breakthrough and TTC,
                             consider hysteresis and detect cluster hopper here
  and save ID of track.

    @param[in]  uObj:The serial number of number
                            fDistY_met:Object's lateral relative distance
                            fWidthLeft_met:Object's width left of the track
  position(left sensor view)
                            pbWarning:Flag whether the FCTA warning was
  activated in the current cycle
                            fTTCFiltered_s:Filtered TTC of the object
    @param[out] RCTAGlobal:RCTA global structure

    @return
  *****************************************************************************/
void CTAFCTASetGlobalWarning(sint32 iFusionID,
                             const float32 fDistY_met,
                             const float32 fWidthLeft_met,
                             boolean* pbWarning,
                             float32 fTTCFiltered_s,
                             FCTAGlobal_t* pFCTAGlobal) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        // The current object is warning
        if (pbWarning[uWarnLevel]) {
            // Set the global warning
            pFCTAGlobal->bFCTAWarnActive[uWarnLevel] = TRUE;
            // Set additional info according to the warning state
            switch (uWarnLevel) {
                case (uint8)FCTA_WARN_LEVEL_ONE:
                    // If the TTC of the current object is below another warning
                    // object
                    if (fTTCFiltered_s < pFCTAGlobal->fCriticalTTC_s) {
                        pFCTAGlobal->fCriticalTTC_s = fTTCFiltered_s;
                        pFCTAGlobal->iCriticalObjID_nu = iFusionID;
                        // Save rear edge position
                        pFCTAGlobal->fCriticalObjDistY_met =
                            fDistY_met + fWidthLeft_met;
                    }
                    break;
                case (uint8)FCTA_WARN_LEVEL_TWO:
                    break;
                case (uint8)FCTA_WARN_LEVEL_THREE:
                    break;
                default:
                    break;
            }
        }
    }
}
/*****************************************************************************
  Functionname:CTAFCTAStateMachine                                     */ /*!

    @brief  FCTA state machine process

    @description FCTA state machine process. off/standby/active/failure

    @param[in]  FCTAGlobal: FCTA global structure
    @param[out] FCTAGlobal: FCTA global structure

    @return
  *****************************************************************************/
void CTAFCTAStateMachine(const FCTAInReq_t* FCTAreqPorts,
                         const FCTAParam_t* FCTAparams,
                         FCTAGlobal_t* pFCTAGlobal) {
    switch (pFCTAGlobal->FCTAStateMachine) {
        case FCTAStateOFF:
            if (FCTAreqPorts->bFCTAFunctionActive) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
                if (FCTAreqPorts->bFCTAFailure) {
                    pFCTAGlobal->FCTAStateMachine = FCTAStateFailure;
                }
            } else {
                pFCTAGlobal->FCTAStateMachine = FCTAStateOFF;
            }
            break;
        case FCTAStateFailure:
            if (FCTAreqPorts->bFCTAFailure) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateFailure;
            } else {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
            }
            break;
        case FCTAStateStandby:
            if (!FCTAreqPorts->bFCTAFunctionActive) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateOFF;
            } else if (FCTAreqPorts->bFCTAFailure) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateFailure;
            } else if (FCTAreqPorts->b_FCTA_RCA || FCTAreqPorts->b_FCTA_FCB) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
            } else if ((pFCTAGlobal->bFCTAWarnActive[0] ||
                        pFCTAGlobal->bFCTAWarnActive[1] ||
                        pFCTAGlobal->bFCTAWarnActive[2]) &&
                       (FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps <
                            FCTAparams->fVEgoMax &&
                        FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps >=
                            FCTAparams->fVEgoMin) &&
                       FCTAreqPorts->FCTAVehicleSig.uGear_nu ==
                           EGO_VEHICLE_GEAR_DRIVE) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateActive;
            } else {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
            }
            break;
        case FCTAStateActive:
            if (!FCTAreqPorts->bFCTAFunctionActive) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateOFF;
            } else if (FCTAreqPorts->bFCTAFailure) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateFailure;
            } else if (FCTAreqPorts->b_FCTA_RCA || FCTAreqPorts->b_FCTA_FCB) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
            } else if ((pFCTAGlobal->bFCTAWarnActive[0] ||
                        pFCTAGlobal->bFCTAWarnActive[1] ||
                        pFCTAGlobal->bFCTAWarnActive[2]) &&
                       (FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps <=
                            FCTAparams->fVEgoMax + 1.f &&
                        FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps >=
                            FCTAparams->fVEgoMin) &&
                       FCTAreqPorts->FCTAVehicleSig.uGear_nu ==
                           EGO_VEHICLE_GEAR_DRIVE) {
                pFCTAGlobal->FCTAStateMachine = FCTAStateActive;
            } else {
                pFCTAGlobal->FCTAStateMachine = FCTAStateStandby;
            }
            break;
        default:
            break;
    }
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void FCTAProProcess(FCTAGlobal_t* FCTAGlobal,
                    FCTAOutPro_t* FCTAproPorts,
                    FCTADebug_t* FCTAdebugInfo) {
    // memcpy(FCTAdebugInfo, FCTAGlobal, sizeof(FCTAGlobal_t));
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        FCTAproPorts->bFCTAWarnActive[uWarnLevel] =
            FCTAGlobal->bFCTAWarnActive[uWarnLevel];
    }
    FCTAproPorts->fCriticalTTC_s = FCTAGlobal->fCriticalTTC_s;
    FCTAproPorts->iCriticalObjID_nu = FCTAGlobal->iCriticalObjID_nu;
    FCTAproPorts->FCTAStateMachine = FCTAGlobal->FCTAStateMachine;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */