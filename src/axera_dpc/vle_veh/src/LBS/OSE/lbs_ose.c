/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "lbs_ose.h"

/*****************************************************************************
        VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
OSEGlobal_t OSEGlobal;
OSEInReq_t OSEReqPorts;
OSEParam_t OSEParams;
OSEOutPro_t OSEProPorts;
OSEDebug_t OSEDebugInfo;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:                                          */ /*!

                     @brief:

                     @description:

                     @param[in] void
                     @param[out] void

                     @return
                   *****************************************************************************/
void LBS_OSE_Exec(const OSEInReq_t* reqPorts,
                  const OSEParam_t* params,
                  OSEOutPro_t* proPorts,
                  OSEDebug_t* debug) {
    OSERunStateGlobal_t eOSEState = OSEGlobal.eOSEState;

    switch (eOSEState) {
        case OSE_OK:
            if ((reqPorts->OSEFunctionSwitch.bOSEFunctionActive) ||
                (reqPorts->OSEFunctionSwitch.bOSEPowermode3min)) {
                OSE_PreProcess(reqPorts, params, &OSEGlobal);
                OSE_MainProcess(reqPorts, params, &OSEGlobal, debug);
                OSE_PostProcess(reqPorts, proPorts, debug, &OSEGlobal);
            } else {
                LBS_OSE_Reset();
            }
            break;
        case OSE_Init:
            LBS_OSE_Reset();
            OSEGlobal.eOSEState = OSE_OK;
            break;
        default:
            LBS_OSE_Reset();
            OSEGlobal.eOSEState = OSE_OK;
            break;
    }
}

/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

        @brief: Init of OSE signals

        @description: Init of OSE signals

        @param[in] void
        @param[out] void

        @return
      *****************************************************************************/
void LBS_OSE_Reset() {
    OSEInitGlobals();
    OSEInitObjects();
    OSEGlobal.eOSEState = OSE_Init;
}
/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

        @brief: Init of OSE global data

        @description: Init of OSE global data

        @param[in] void
        @param[out] void

        @return
      *****************************************************************************/
void OSEInitGlobals() {
    OSEGlobal.MultiObj.fCriticalTTC = TUE_C_F32_VALUE_INVALID;
    OSEGlobal.MultiObj.fCriticalObjDistX = TUE_C_F32_VALUE_INVALID;
    OSEGlobal.MultiObj.fCriticalObjDistXLastCycle = TUE_C_F32_VALUE_INVALID;
    OSEGlobal.MultiObj.uCriticalObjID = TUE_C_UI8_VALUE_INVALID;
    OSEGlobal.MultiObj.uCriticalObjIDLastCycle = TUE_C_UI8_VALUE_INVALID;
    OSEGlobal.MultiObj.bWarningInterrupt = FALSE;
    OSEGlobal.MultiObj.uInterruptCycleCount = 0;
    // OSEGlobal.SelDrvHyp = LBS_OSE_HYP_UNKNOWN;

    for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        OSEGlobal.bOSEWarnActive[uWarnLevel] = FALSE;
        OSEGlobal.ParameterLevel[uWarnLevel].fYMaxBreakthrough[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.ParameterLevel[uWarnLevel].fYMaxBreakthrough[1u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.ParameterLevel[uWarnLevel].fYMinBreakthrough[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.ParameterLevel[uWarnLevel].fYMinBreakthrough[1u] =
            TUE_C_F32_VALUE_INVALID;
        // OSEGlobal.WarningTimer[uWarnLevel] = 0.0f;
    }
}
/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

        @brief: Init of object data

        @description: Init of object data

        @param[in] void
        @param[out] void

        @return
      *****************************************************************************/
void OSEInitObjects() {
    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[1u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[1u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[0u] = TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[1u] = TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[1u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[0u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[1u] =
            TUE_C_F32_VALUE_INVALID;
        OSEGlobal.OSEObjInfoArray[uObj].fSideTrackProb = OSE_FRONTOBJ_INIT_PROB;
        OSEGlobal.OSEObjInfoArray[uObj].bRelevant = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bMirror = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bSideTrack = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bObjectFromRear = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bValidApproachAngle = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bObjectAtEdgeFoV = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].bShortTTC = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].fQuality = 0.0f;
        OSEGlobal.OSEObjInfoArray[uObj].bUpdatedRecently = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[0] = 0u;
        OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[1] = 0u;
        OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[2] = 0u;
        OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.fValue_met = 0.0f;

        for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
             uWarnLevel++) {
            // Init warning level values
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .fBTHitHystTimer = 0.0f;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .uBreakthroughHitConfi[0u] = 0u;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .uBreakthroughHitConfi[1u] = 0u;
            OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bWarning =
                FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bWarningLastCycle = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bObjectInRange = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bTTCBelowThresh[0u] = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bTTCBelowThresh[1u] = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bBTHitHystActive = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bBreakthroughHit[0u] = FALSE;
            OSEGlobal.OSEObjInfoArray[uObj]
                .InfoLevel[uWarnLevel]
                .bBreakthroughHit[1u] = FALSE;
        }
    }
}
/*****************************************************************************
  Functionname: OSE_PreProcess                                          */ /*!

      @brief: The PreProcess of OSE function

      @description: The PreProcess of OSE function

      @param[in ] reqPorts       the inputs structure of OSE function
                  params         the parameter structure of OSE function
                              pOSEGlobal     OSE global variable structure
      @param[out] void

      @return
    *****************************************************************************/
void OSE_PreProcess(const OSEInReq_t* reqPorts,
                    const OSEParam_t* params,
                    OSEGlobal_t* pOSEGlobal) {
    // Calculate left and right sensor offset to side
    LBSOSESetParameters(params, &pOSEGlobal->fLeftSensorOffsetToSide_met,
                        &pOSEGlobal->fRightSensorOffsetToSide_met,
                        &pOSEGlobal->fLeftSensorOffsetToRear_met,
                        &pOSEGlobal->fRightSensorOffsetToRear_met);
    // Init all cyclic variables
    LBSOSEInitCyclic(pOSEGlobal);
    // Check current driving condition: unknown   parking   driving
    // LBSOSECheckDrivingConditions();
}
/*****************************************************************************
  Functionname: OSE_MainProcess                                        */ /*!

       @brief: OSE main function

       @description: OSE condition check and warning decision

       @param[in]  reqPorts        Input structure of OSE function
                   params          Parameter structure of OSE function
       @param[out] pOSEGlobal      OSE global variable structure

       @return
     *****************************************************************************/
static uint8 LBS_DOW_ObjCounter = 0u;
void OSE_MainProcess(const OSEInReq_t* reqPorts,
                     const OSEParam_t* params,
                     OSEGlobal_t* pOSEGlobal,
                     OSEDebug_t* debug) {
    // Perform same operations on all existing objects
    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        const OSEGenObjectInReq_t* pEMGenObjInReq =
            &reqPorts->EMGenObjList.aObject[uObj];
        OSEObjInfoArrayGlobal_t* pOSEObjGlobal =
            &pOSEGlobal->OSEObjInfoArray[uObj];
        const OSELBSObjInfoArrayInReq_t* pOSELBSObjInReq =
            &reqPorts->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj];

        if (!bObjIsDeleted(
                uObj, reqPorts->EMGenObjList.aObject[uObj].eMaintenanceState)) {
            LBS_DOW_ObjCounter = uObj;
            // Calculate the limit value of YBreakthrough for OSE function
            LBSOSECalculateYBreakthroughLimit(
                reqPorts->EMGenObjList.aObject[uObj].bRightSensor,
                pOSEGlobal->fLeftSensorOffsetToSide_met,
                pOSEGlobal->fRightSensorOffsetToSide_met, params,
                pOSEGlobal->ParameterLevel);
            // Calculate distance to crossing line
            LBSOSECalculateDistToCrossingLine(
                pEMGenObjInReq->bRightSensor, pEMGenObjInReq->fDistX_met,
                pOSEGlobal->fLeftSensorOffsetToRear_met,
                pOSEGlobal->fRightSensorOffsetToRear_met,
                params->fXBreakthroughLine_met[0u],
                pEMGenObjInReq->fLengthFront_met,
                &pOSEObjGlobal->fDistToCrossingLine_met[0u]);
            LBSOSECalculateDistToCrossingLine(
                pEMGenObjInReq->bRightSensor, pEMGenObjInReq->fDistX_met,
                pOSEGlobal->fLeftSensorOffsetToRear_met,
                pOSEGlobal->fRightSensorOffsetToRear_met,
                params->fXBreakthroughLine_met[1u],
                pEMGenObjInReq->fLengthFront_met,
                &pOSEObjGlobal->fDistToCrossingLine_met[1u]);
            // Calculate the additional margin depending on the object heading
            // angle LBSOSECalculateObjBreakthroughMargin();
            // LBSOSECalculateObjBreakthroughMargin();
            // Estimate the width of the object based in RCS
            LBSOSEEstimate_Width(pEMGenObjInReq->fWidth_met,
                                 pEMGenObjInReq->fRCS,
                                 pOSEObjGlobal->fEstWidth.uCounters,
                                 &pOSEObjGlobal->fEstWidth.fValue_met);
            // Calculate x-axis breakthrough
            LBSOSECalculateYBreakthrough(pEMGenObjInReq, pOSEObjGlobal, 0u);
            LBSOSECalculateYBreakthrough(pEMGenObjInReq, pOSEObjGlobal, 1u);
            // Calculate time to crossing
            LBSOSECalculateTTC(
                reqPorts->fCycletime_s, pEMGenObjInReq->fVrelX_mps,
                pOSEObjGlobal->fDistToCrossingLine_met[0u],
                &pOSEObjGlobal->fTTC_s[0u], &pOSEObjGlobal->fTTCFiltered_s[0u]);
            LBSOSECalculateTTC(
                reqPorts->fCycletime_s, pEMGenObjInReq->fVrelX_mps,
                pOSEObjGlobal->fDistToCrossingLine_met[1u],
                &pOSEObjGlobal->fTTC_s[1u], &pOSEObjGlobal->fTTCFiltered_s[1u]);
            // Calculate the side object probability
            LBSOSECalculateFrontObjectProbability(
                uObj, reqPorts, pOSEGlobal, &pOSEObjGlobal->fSideTrackProb);
            // Check whether this object is a side object
            LBSOSECheckSideObject(pOSEObjGlobal->fSideTrackProb,
                                  &pOSEObjGlobal->bSideTrack);
            // Check whether the object was updated in the last cycles
            LBSOSECheckObjectUpdateRecently(
                pEMGenObjInReq->uiMeasuredTargetFrequency_nu,
                &pOSEObjGlobal->bUpdatedRecently);
            // Check if object is in range
            LBSOSECheckObjectInRange(
                pEMGenObjInReq->fDistX_met, pEMGenObjInReq->fDistY_met,
                pEMGenObjInReq->fLengthFront_met, params->fTargetRangeMax_met,
                pOSEObjGlobal->InfoLevel);
            // Check if target angle of approach is in range
            LBSOSECheckRearApproach(pEMGenObjInReq->fDistX_met,
                                    pEMGenObjInReq->fFirstDetectX_met,
                                    &pOSEObjGlobal->bObjectFromRear);
            // Check angle of approach is in range
            LBSOSECheckApproachAngle(
                pEMGenObjInReq->fAbsOrientation_rad,
                pOSEObjGlobal->bObjectFromRear, params->fMaxHeadingAngle,
                params->fMinHeadingAngle, &pOSEObjGlobal->bValidApproachAngle);
            // Check if an object which enables a warning is a possible mirror
            LBSOSECheckForMirror(pEMGenObjInReq->fMirrorProb_per,
                                 &pOSEObjGlobal->bMirror);
            // Check whether object is relevant
            LBSOSECheckObjectRelevance(
                pOSELBSObjInReq->fXMovement_met,
                pOSELBSObjInReq->fYMovement_met, pEMGenObjInReq->fVabsX_mps,
                pEMGenObjInReq->fVabsY_mps, pEMGenObjInReq->uiLifeCycles_nu,
                params->fVTargetMin_mps, &pOSEObjGlobal->bRelevant);
            // printf("%d_OBJ fSpeedOverGroundSqr %f\t, fMovementAbsSqr %f\t,
            // uiLifeCycles_nu %d\n",
            //     uObj,
            //     (SQR(pEMGenObjInReq->fVabsY_mps) +
            //     SQR(pEMGenObjInReq->fVabsX_mps)),
            //     (SQR(pOSELBSObjInReq->fXMovement_met) +
            //     SQR(pOSELBSObjInReq->fYMovement_met)),
            //     pEMGenObjInReq->uiLifeCycles_nu);
            // Calculate whether the object has the necessary quality
            LBSOSECheckObjectQuality(
                pEMGenObjInReq, pOSELBSObjInReq->fUpdateRate_nu,
                pOSELBSObjInReq->fAssocProbFiltered, &pOSEObjGlobal->fQuality);
            // Check whether the object hits the breakthrough line
            LBSOSECheckBreakthroughHit(
                0u, pOSEObjGlobal->fDistToCrossingLine_met,
                pOSEObjGlobal->fYBreakthrough, pOSEGlobal->ParameterLevel,
                pOSEObjGlobal->InfoLevel);
            LBSOSECheckBreakthroughHit(
                1u, pOSEObjGlobal->fDistToCrossingLine_met,
                pOSEObjGlobal->fYBreakthrough, pOSEGlobal->ParameterLevel,
                pOSEObjGlobal->InfoLevel);
            // Update the confidence of the breakthrough hit
            LBSOSECalculateBreakthroughHitConfidence(
                0u, pOSEObjGlobal->fYBreakthrough,
                pOSEObjGlobal->fYBreakthroughStd,
                pOSEObjGlobal->bObjectAtEdgeFoV, pOSEGlobal->ParameterLevel,
                pOSEObjGlobal->InfoLevel);
            LBSOSECalculateBreakthroughHitConfidence(
                1u, pOSEObjGlobal->fYBreakthrough,
                pOSEObjGlobal->fYBreakthroughStd,
                pOSEObjGlobal->bObjectAtEdgeFoV, pOSEGlobal->ParameterLevel,
                pOSEObjGlobal->InfoLevel);
            // Check the hysteresis timer(smoothing of warning ON/OFF)
            LBSOSEUpdateBTHitHysteresisTimer(
                reqPorts->fCycletime_s, pOSEObjGlobal->fTTC_s[0u],
                pOSEObjGlobal->fTTCFiltered_s[0u], pOSEObjGlobal->InfoLevel);
            // Check the hysteresis timer condition (unstable object
            // trajectories do not result in unstable warnings)
            LBSOSECheckBTHitHysteresis(pOSEObjGlobal->InfoLevel);
            // Check the TTC condition
            LBSOSECheckTTC(0u, pOSEObjGlobal->fTTC_s[0u],
                           pOSEObjGlobal->fTTCFiltered_s[0u],
                           pOSEObjGlobal->bObjectAtEdgeFoV,
                           params->fTTCThreshold_s, pOSEObjGlobal->InfoLevel);
            LBSOSECheckTTC(1u, pOSEObjGlobal->fTTC_s[1u],
                           pOSEObjGlobal->fTTCFiltered_s[1u],
                           pOSEObjGlobal->bObjectAtEdgeFoV,
                           params->fTTCThreshold_s, pOSEObjGlobal->InfoLevel);
            // Check the short warning condition
            LBSOSECheckShortWarning(pOSEObjGlobal->fTTC_s[0u],
                                    pOSEObjGlobal->fTTCFiltered_s[0u],
                                    &pOSEObjGlobal->bShortTTC);
            // Decide whether the object shall warn
            LBSOSEWarningDecision(pEMGenObjInReq->fRCS, pOSEObjGlobal,
                                  pOSEObjGlobal->InfoLevel);
            // Set the global warning
            LBSOSESetGlobalWarning(uObj, pEMGenObjInReq->fDistX_met,
                                   pEMGenObjInReq->fLengthRear_met,
                                   pOSEObjGlobal->fTTCFiltered_s[0u],
                                   pOSEObjGlobal, &pOSEGlobal->MultiObj,
                                   pOSEGlobal->bOSEWarnActive);
            debug->OSEWarnDecideList[uObj].OBJID = uObj;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_bRelevant =
                pOSEObjGlobal->bRelevant;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_Non_bMirror =
                !pOSEObjGlobal->bMirror;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_Non_bShortTTC =
                !pOSEObjGlobal->bShortTTC;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_Non_bSideTrack =
                !pOSEObjGlobal->bSideTrack;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_bValidApproachAngle =
                pOSEObjGlobal->bValidApproachAngle;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_bUpdatedRecently =
                pOSEObjGlobal->bUpdatedRecently;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fQuality =
                pOSEObjGlobal->fQuality;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fRCS =
                pEMGenObjInReq->fRCS;
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fTTC_s[0u] =
                pOSEObjGlobal->fTTC_s[0u];
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fTTC_s[1u] =
                pOSEObjGlobal->fTTC_s[1u];
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fTTCFiltered_s[0u] =
                pOSEObjGlobal->fTTCFiltered_s[0u];
            debug->OSEWarnDecideList[uObj].LCA_WarnDecide_fTTCFiltered_s[1u] =
                pOSEObjGlobal->fTTCFiltered_s[1u];

            for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
                 uWarnLevel++) {
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_bWarning =
                    pOSEObjGlobal->InfoLevel[uWarnLevel].bWarning;
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_uBreakthroughHitConfi[0u] =
                    pOSEObjGlobal->InfoLevel[uWarnLevel]
                        .uBreakthroughHitConfi[0u];
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_uBreakthroughHitConfi[1u] =
                    pOSEObjGlobal->InfoLevel[uWarnLevel]
                        .uBreakthroughHitConfi[1u];
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_bObjectInRange =
                    pOSEObjGlobal->InfoLevel[uWarnLevel].bObjectInRange;
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_bTTCBelowThresh[0u] =
                    pOSEObjGlobal->InfoLevel[uWarnLevel].bTTCBelowThresh[0u];
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_bTTCBelowThresh[1u] =
                    pOSEObjGlobal->InfoLevel[uWarnLevel].bTTCBelowThresh[1u];
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_bBTHitHystActive =
                    pOSEObjGlobal->InfoLevel[uWarnLevel].bBTHitHystActive;
                debug->OSEWarnDecideList[uObj]
                    .OSE_WarnDecide_InfoLevel[uWarnLevel]
                    .LCA_WarnDecide_fTTCThreshold_s =
                    params->fTTCThreshold_s[uWarnLevel];
            }
        }
    }
    // Handle multi targets
    LBSOSECheckMultiObjectInterrupt(&reqPorts->EMGenObjList,
                                    &pOSEGlobal->MultiObj);
}

/*****************************************************************************
  Functionname: OSE_PostProcess                                        */ /*!

       @brief: Process the warnings for OSE function

       @description: Process the warnings for OSE function

       @param[in]
       @param[out]

       @return
     *****************************************************************************/
void OSE_PostProcess(const OSEInReq_t* reqPorts,
                     OSEOutPro_t* proPorts,
                     OSEDebug_t* debug,
                     OSEGlobal_t* pOSEGlobal) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        proPorts->bOSEWarnActive[uWarnLevel] =
            pOSEGlobal->bOSEWarnActive[uWarnLevel];
    }
    proPorts->bWarningInterrupt = pOSEGlobal->MultiObj.bWarningInterrupt;
    proPorts->uCriticalObjID = pOSEGlobal->MultiObj.uCriticalObjID;
    proPorts->fCriticalObjDistX = pOSEGlobal->MultiObj.fCriticalObjDistX;
    proPorts->fCriticalTTC = pOSEGlobal->MultiObj.fCriticalTTC;

    // memcpy(proPorts->OSEObjInfoArray, pOSEGlobal->OSEObjInfoArray,
    //        (sizeof(OSEObjInfoArrayOutPro_t) * LBS_INPUT_OBJECT_NUMBER));

    // static float32 fDowPowerModetimer = 0.f;

    // customer feature for DOW power Mode shut down
    // proPorts->bDOWPowerModeDone = TUE_CML_TimerRetrigger(
    //     reqPorts->fCycletime_s,
    //     reqPorts->OSEFunctionSwitch.bOSEPowermode3min, 3000 /*ms*/,
    //     &fDowPowerModetimer);
    proPorts->bDOWPowerModeDone = reqPorts->OSEFunctionSwitch.bOSEPowermode3min;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
