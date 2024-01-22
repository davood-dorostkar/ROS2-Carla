/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#include "./cta.h"  // NOLINT
#include <string.h>
// #include "../../decision/src/CTA/cta.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
CTAGlobal_t CTAGlobal;
static FCTAInReq_t FCTAreqPorts;
static FCTAParam_t FCTAparams;
static FCTAOutPro_t FCTAproPorts;
static FCTADebug_t FCTAdebugInfo;
static RCTAInReq_t RCTAreqPorts;
static RCTAParam_t RCTAparams;
static RCTAOutPro_t RCTAproPorts;
static RCTADebug_t RCTAdebugInfo;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:                                       */ /*!

                         @brief

                         @description

                         @param[in]

                         @param[out]

                         @return
                       *****************************************************************************/
void CTA_Reset() {
    CTAGlobal.eCTAState = CTA_INIT;
    // Init all CTA globals
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        CTAGlobal.bFCTAFunctionOutput = FALSE;
    }
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        CTAGlobal.bRCTAFunctionOutput = FALSE;
    }
    // CTAGlobal.fSensorOffsetToRear_met
    // CTAGlobal.fSensorOffsetToSide_met
    CTAGlobal.fMaxSpeedOverGround = 0.f;
    CTAGlobal.LastCycleStates.eRoadType = CTA_ROAD_TYPE_UNKNOWN;
    CTAGlobal.LastCycleStates.bFCTAFunctionActive = FALSE;
    CTAGlobal.LastCycleStates.bRCTAFunctionActive = FALSE;
    CTAGlobal.LastCycleStates.bEgoSpeedConditionFCTA = FALSE;
    CTAGlobal.LastCycleStates.CTAState = CTA_INIT;
    // Initialize CTA related information
    CTAInitObjects();

    RCTAReset();
    FCTAReset();

    // memset(&FCTAreqPorts, 0.0f, sizeof(FCTAInReq_t));
    // memset(&FCTAparams, 0.0f, sizeof(FCTAParam_t));
    // memset(&FCTAproPorts, 0.0f, sizeof(FCTAOutPro_t));
    // memset(&FCTAdebugInfo, 0.0f, sizeof(FCTADebug_t));
    // memset(&RCTAreqPorts, 0.0f, sizeof(RCTAInReq_t));
    // memset(&RCTAparams, 0.0f, sizeof(RCTAParam_t));
    // memset(&RCTAproPorts, 0.0f, sizeof(RCTAOutPro_t));
    // memset(&RCTAdebugInfo, 0.0f, sizeof(RCTADebug_t));
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTAInitObjects() {
    for (uint8 uObj = 0u; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
        // Clear all CTA general object related information
        CTAGlobal.CTAObjectList[uObj].fTTCAccel_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fTTC_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fTTCFiltered_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fTTCRadial_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fCycleTimeSum_s = 0.f;
        CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.5f;
        CTAGlobal.CTAObjectList[uObj].fAssocProbFiltered_nu = 0.5f;
        CTAGlobal.CTAObjectList[uObj].fAngle_deg = 0.f;
        CTAGlobal.CTAObjectList[uObj].fObjLengthMax_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].fObjWidthMax_met = 0.f;
        // CTAGlobal.CTAObjectList[uObj].fRangeRadial_met =
        // TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fFCTAVabs = 0.f;
        CTAGlobal.CTAObjectList[uObj].fRCTAVabs = 0.f;
        CTAGlobal.CTAObjectList[uObj].ObjectBorder.fXMin_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].ObjectBorder.fXMax_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].ObjectBorder.fYMin_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].ObjectBorder.fYMax_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].fFCTAXLastCycle_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fFCTAYLastCycle_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fFCTAVxPosBased_mps =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fFCTAVyPosBased_mps =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fRCTAXLastCycle_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fRCTAYLastCycle_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fRCTAVxPosBased_mps =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fRCTAVyPosBased_mps =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fXMin_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fXMax_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fYMin_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fYMax_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectRotated.fDistX =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectRotated.fDistY =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectRotated.fLength =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].ObjectRotated.fWidth =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTAObjectList[uObj].fFCTAXMovement_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].fFCTAYMovement_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].fRCTAXMovement_met = 0.f;
        CTAGlobal.CTAObjectList[uObj].fRCTAYMovement_met = 0.f;
        // CTAGlobal.CTAObjectList[uObj].uUniqueID = TUE_C_UI16_VALUE_INVALID;
        // CTAGlobal.CTAObjectList[uObj].uLastMergedObjID =
        // TUE_C_UI8_VALUE_INVALID; CTAGlobal.CTAObjectList[uObj].bLowTTCAtState
        // = FALSE; CTAGlobal.CTAObjectList[uObj].bCreatedAdjStableObj = FALSE;
        // CTAGlobal.CTAObjectList[uObj].bObjValidForSelection = FALSE;
        // CTAGlobal.CTAObjectList[uObj].bPriolObject = FALSE;
        // CTAGlobal.CTAObjectList[uObj].fSpeedFiltered = 0.f;

        // Initialize CTA CT object
        CTAGlobal.CTObjectList[uObj].fFCTAXBreakthrough_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTAXBreakthroughFiltered_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTAXBreakthroughStd_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTATTC_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTATTCFiltered_s =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTAXBreakthrough_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTAXBreakthroughFiltered_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTAXBreakthroughStd_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTATTC_s = TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTATTCFiltered_s =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTADistToCrossingLine_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fRCTADistToCrossingLine_met =
            TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fDistToCrossingLineFiltered_met =
            TUE_C_F32_VALUE_INVALID;
        // CTAGlobal.CTObjectList[uObj].fObjBreakthroughMargin_met =
        // TUE_C_F32_VALUE_INVALID;
        CTAGlobal.CTObjectList[uObj].fFCTARearTrackProb_per =
            CTA_FCTA_REARTRACK_INIT_PROB;  // 0.3
        CTAGlobal.CTObjectList[uObj].fRCTARearTrackProb_per =
            CTA_FCTA_REARTRACK_INIT_PROB;  // 0.3
        CTAGlobal.CTObjectList[uObj].bFCTARearTrack_nu = FALSE;
        CTAGlobal.CTObjectList[uObj].bRCTARearTrack_nu = FALSE;
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
void CTA_Exec(const CTAInReq_t* reqPorts,
              const CTAParam_t* params,
              CTAOutPro_t* proPorts,
              CTADebug_t* debugInfo) {
    if (CTAGlobal.eCTAState == CTA_OK) {
        // PreProcess
        CTAPreProcess(reqPorts, params, &CTAGlobal);

        // FCTA: Front Cross Traffic Alert
        CTAToFCTAInputWrapper(reqPorts, params, &CTAGlobal, &FCTAreqPorts,
                              &FCTAparams);
        FCTAExec(&FCTAreqPorts, &FCTAparams, &FCTAproPorts, &FCTAdebugInfo);
        FCTAToCTAOutputWrapper(&FCTAproPorts, &FCTAdebugInfo, &CTAGlobal,
                               debugInfo);

        // RCTA: Rear Cross Traffic Alert
        CTAToRCTAInputWrapper(reqPorts, params, &CTAGlobal, &RCTAreqPorts,
                              &RCTAparams);
        RCTAExec(&RCTAreqPorts, &RCTAparams, &RCTAproPorts, &RCTAdebugInfo);
        RCTAToCTAOutputWrapper(&RCTAproPorts, &RCTAdebugInfo, &CTAGlobal,
                               debugInfo);

        // ProProcess
        CTAProProcess(reqPorts, params, &CTAGlobal, proPorts);

    } else {
        CTA_Reset();
        CTAGlobal.eCTAState = CTA_OK;
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
void CTAPreProcess(const CTAInReq_t* reqPorts,
                   const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal) {
    // gather global properties needed for CTA
    CTACalculateGlobalProperties(&params->CTA_Ks_VehicleParameter_nu,
                                 &pCTAGlobal->fSensorOffsetToSide_met,
                                 &pCTAGlobal->fSensorOffsetToRear_met);
    // gather general object properties needed for CTA
    CTACalculateObjectProperties(reqPorts, params, pCTAGlobal);
    // gather general object properties needed for CTA CT
    CTACalculateCTObjectProperties(reqPorts, params, pCTAGlobal);
}
/*****************************************************************************
  Functionname:CTACalculateGlobalProperties */ /*!

                                   @brief Calculate global CTA properties

                                   @description Calculate global CTA properties

                                   @param[in]  pVehicleParameter: Vehicle
                                 parameter structure

                                   @param[out] pfSensorOffsetToSide_met: the
                                 offset from sensor mounting position to the
                                 side edge of vehicle
                                                           pfSensorOffsetToRear_met:
                                 the offset from sensor mounting position to the
                                 front or rear edge of vehicle

                                   @return
                                 *****************************************************************************/
void CTACalculateGlobalProperties(
    const CTAVehicleParam_t* pVehicleParameter,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    SensorMountingPosGlobal_t* pfSensorOffsetToRear_met) {
    // Process vehicle parameters from RTE. This is additional to init
    // processing, to make sure that SW signal delays do not cause issues with
    // setting the values correctly calculate the Offset between sensor and side
    pfSensorOffsetToSide_met->fLeftFrontPos_met =
        0.5f * pVehicleParameter->CTA_Kf_VehicleWidth_met -
        pVehicleParameter->CTA_Ks_LeftFrontSensorMounting.CTA_Kf_LatPos_met;
    pfSensorOffsetToSide_met->fRightFrontPos_met =
        -0.5f * pVehicleParameter->CTA_Kf_VehicleWidth_met -
        pVehicleParameter->CTA_Ks_RightFrontSensorMounting.CTA_Kf_LatPos_met;
    pfSensorOffsetToSide_met->fLeftRearPos_met =
        0.5f * pVehicleParameter->CTA_Kf_VehicleWidth_met -
        pVehicleParameter->CTA_Ks_LeftRearSensorMounting_nu.CTA_Kf_LatPos_met;
    pfSensorOffsetToSide_met->fRightRearPos_met =
        -0.5f * pVehicleParameter->CTA_Kf_VehicleWidth_met -
        pVehicleParameter->CTA_Ks_RightRearSensorMounting_nu.CTA_Kf_LatPos_met;
    // calculate the Offset between sensor and front/rear side
    pfSensorOffsetToRear_met->fLeftFrontPos_met =
        pVehicleParameter->CTA_Ks_LeftFrontSensorMounting.CTA_Kf_LongPos_met -
        pVehicleParameter->CTA_Kf_OverhangFront_met;
    // pfSensorOffsetToRear_met->fLeftFrontPos_met = TUE_CML_MinMax(
    //     -0.5f, -0.1f, pfSensorOffsetToRear_met->fLeftFrontPos_met);
    pfSensorOffsetToRear_met->fRightFrontPos_met =
        pVehicleParameter->CTA_Ks_RightFrontSensorMounting.CTA_Kf_LongPos_met -
        pVehicleParameter->CTA_Kf_OverhangFront_met;
    // pfSensorOffsetToRear_met->fRightFrontPos_met = TUE_CML_MinMax(
    //     -0.5f, -0.1f, pfSensorOffsetToRear_met->fRightFrontPos_met);
    pfSensorOffsetToRear_met->fLeftRearPos_met =
        pVehicleParameter->CTA_Kf_VehicleLength_met -
        pVehicleParameter->CTA_Kf_OverhangFront_met +
        pVehicleParameter->CTA_Ks_LeftRearSensorMounting_nu.CTA_Kf_LongPos_met;
    // pfSensorOffsetToRear_met->fLeftRearPos_met = TUE_CML_MinMax(
    //     0.05f, 0.35f, pfSensorOffsetToRear_met->fLeftRearPos_met);
    pfSensorOffsetToRear_met->fRightRearPos_met =
        pVehicleParameter->CTA_Kf_VehicleLength_met -
        pVehicleParameter->CTA_Kf_OverhangFront_met +
        pVehicleParameter->CTA_Ks_RightRearSensorMounting_nu.CTA_Kf_LongPos_met;
    // pfSensorOffsetToRear_met->fRightRearPos_met = TUE_CML_MinMax(
    //     0.05f, 0.35f, pfSensorOffsetToRear_met->fRightRearPos_met);
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTACalculateObjectProperties(const CTAInReq_t* reqPorts,
                                  const CTAParam_t* params,
                                  CTAGlobal_t* pCTAGlobal) {
    for (uint8 uObj = 0u; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
        const CTA_FusionObject_t* pEMFusionObjInput =
            &reqPorts->CTAEMFusionObjList.aObjects[uObj];
        CTAObjectInfoGlobal_t* pCTAObjGlobal = &pCTAGlobal->CTAObjectList[uObj];
        const EgoVehicleInReq_t* pEgoVehicleInput = &reqPorts->EgoVehicleInfo;

        if (!pEMFusionObjInput->uiMaintenanceState_nu ==
            CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
            // calculate the borders in which an object has moved during its
            // lifetime

            CTACalculateObjectFCTAMovementBorders(
                params, pEMFusionObjInput->bRightSide,
                pEMFusionObjInput->fDistX_met, pEMFusionObjInput->fDistY_met,
                &pCTAObjGlobal->ObjectMovementBorder,
                &pCTAObjGlobal->fFCTAXMovement_met,
                &pCTAObjGlobal->fFCTAYMovement_met);
            CTACalculateObjectRCTAMovementBorders(
                params, pEMFusionObjInput->bRightSide,
                pEMFusionObjInput->fDistX_met, pEMFusionObjInput->fDistY_met,
                &pCTAObjGlobal->ObjectMovementBorder,
                &pCTAObjGlobal->fRCTAXMovement_met,
                &pCTAObjGlobal->fRCTAYMovement_met);
            // calculate object quality related qualifies
            CTACalculateObjectQualifiers(
                pEMFusionObjInput->uiHighestAssocProb_per,
                pEMFusionObjInput->uiMaintenanceState_nu,
                &pCTAObjGlobal->fUpdateRate_nu,
                &pCTAObjGlobal->fAssocProbFiltered_nu);
            // calculate vx and vy based on the change in position of the
            // current object
            CTACalculateFCTAPosBasedVxVy(params, reqPorts->fCycleTime_s,
                                         pEMFusionObjInput,
                                         pCTAObjGlobal->fFCTAXLastCycle_met,
                                         pCTAObjGlobal->fFCTAYLastCycle_met,
                                         &pCTAObjGlobal->fFCTAVxPosBased_mps,
                                         &pCTAObjGlobal->fFCTAVyPosBased_mps);
            CTACalculateRCTAPosBasedVxVy(params, reqPorts->fCycleTime_s,
                                         pEMFusionObjInput,
                                         pCTAObjGlobal->fRCTAXLastCycle_met,
                                         pCTAObjGlobal->fRCTAYLastCycle_met,
                                         &pCTAObjGlobal->fRCTAVxPosBased_mps,
                                         &pCTAObjGlobal->fRCTAVyPosBased_mps);
            // calculate the absolute object velocity
            CTACalculateFCTAAbsoluteObjectVelocity(pEMFusionObjInput, params,
                                                   pEgoVehicleInput,
                                                   &pCTAObjGlobal->fFCTAVabs);
            CTACalculateRCTAAbsoluteObjectVelocity(pEMFusionObjInput, params,
                                                   pEgoVehicleInput,
                                                   &pCTAObjGlobal->fRCTAVabs);
        }
    }
}
/*****************************************************************************
  Functionname:CTACalculateObjectMovementBorders */ /*!

                              @brief:Calculates the x and y borders in which an
                            object has moved during its lifetime

                              @description:Calculates the x and y borders in
                            which an object has moved during its lifetime

                              @param[in]  fDistX_met:Object's longitudinal
                            relative distance
                                          fDistY_met:Object's lateral relative
                            distance

                              @param[out] pObjectMovementBorder:The object
                            movement border information
                                          fXMovement_met:The object total moving
                            distance in the x direction,unit:m
                                                      fYMovement_met:The object
                            total moving distance in the y direction,unit:m
                              @return
                            *****************************************************************************/
void CTACalculateObjectFCTAMovementBorders(
    const CTAParam_t* params,
    boolean bRightSide,
    float32 fDistX_met,
    float32 fDistY_met,
    CTAObjectBorder_t* pObjectMovementBorder,
    float32* fXMovement_met,
    float32* fYMovement_met) {
    float32 fDistX;
    float32 fDistY;
    fDistX = fDistX_met -
             params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;
    if (bRightSide) {
        fDistY =
            fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }

    if (pObjectMovementBorder->fXMin_met > TUE_C_F32_VALUE_INVALID - 1.0f) {
        pObjectMovementBorder->fXMin_met = fDistX;
        pObjectMovementBorder->fXMax_met = fDistX;
        pObjectMovementBorder->fYMin_met = fDistY;
        pObjectMovementBorder->fYMax_met = fDistY;
    } else {
        // Calculate the bounds of the longitudinal motion
        if (fDistX > pObjectMovementBorder->fXMax_met) {
            pObjectMovementBorder->fXMax_met = fDistX;
            *fXMovement_met = pObjectMovementBorder->fXMax_met -
                              pObjectMovementBorder->fXMin_met;
        } else {
            if (fDistX < pObjectMovementBorder->fXMin_met) {
                pObjectMovementBorder->fXMin_met = fDistX;
                *fXMovement_met = pObjectMovementBorder->fXMax_met -
                                  pObjectMovementBorder->fXMin_met;
            }
        }

        // Calculate the bounds of the lateral motion
        if (fDistY > pObjectMovementBorder->fYMax_met) {
            pObjectMovementBorder->fYMax_met = fDistY;
            *fYMovement_met = pObjectMovementBorder->fYMax_met -
                              pObjectMovementBorder->fYMin_met;
        } else {
            if (fDistY < pObjectMovementBorder->fYMin_met) {
                pObjectMovementBorder->fYMin_met = fDistY;
                *fYMovement_met = pObjectMovementBorder->fYMax_met -
                                  pObjectMovementBorder->fYMin_met;
            }
        }
    }
}
/*****************************************************************************
  Functionname:CTACalculateObjectMovementBorders */ /*!

                              @brief:Calculates the x and y borders in which an
                            object has moved during its lifetime

                              @description:Calculates the x and y borders in
                            which an object has moved during its lifetime

                              @param[in]  fDistX_met:Object's longitudinal
                            relative distance
                                          fDistY_met:Object's lateral relative
                            distance

                              @param[out] pObjectMovementBorder:The object
                            movement border information
                                          fXMovement_met:The object total moving
                            distance in the x direction,unit:m
                                                      fYMovement_met:The object
                            total moving distance in the y direction,unit:m
                              @return
                            *****************************************************************************/
void CTACalculateObjectRCTAMovementBorders(
    const CTAParam_t* params,
    boolean bRightSide,
    float32 fDistX_met,
    float32 fDistY_met,
    CTAObjectBorder_t* pObjectMovementBorder,
    float32* fXMovement_met,
    float32* fYMovement_met) {
    float32 fDistX;
    float32 fDistY;
    fDistX = fDistX_met +
             (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
              params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met);
    if (bRightSide) {
        fDistY =
            fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }

    if (pObjectMovementBorder->fXMin_met > TUE_C_F32_VALUE_INVALID - 1.0f) {
        pObjectMovementBorder->fXMin_met = fDistX;
        pObjectMovementBorder->fXMax_met = fDistX;
        pObjectMovementBorder->fYMin_met = fDistY;
        pObjectMovementBorder->fYMax_met = fDistY;
    } else {
        // Calculate the bounds of the longitudinal motion
        if (fDistX > pObjectMovementBorder->fXMax_met) {
            pObjectMovementBorder->fXMax_met = fDistX;
            *fXMovement_met = pObjectMovementBorder->fXMax_met -
                              pObjectMovementBorder->fXMin_met;
        } else {
            if (fDistX < pObjectMovementBorder->fXMin_met) {
                pObjectMovementBorder->fXMin_met = fDistX;
                *fXMovement_met = pObjectMovementBorder->fXMax_met -
                                  pObjectMovementBorder->fXMin_met;
            }
        }

        // Calculate the bounds of the lateral motion
        if (fDistY > pObjectMovementBorder->fYMax_met) {
            pObjectMovementBorder->fYMax_met = fDistY;
            *fYMovement_met = pObjectMovementBorder->fYMax_met -
                              pObjectMovementBorder->fYMin_met;
        } else {
            if (fDistY < pObjectMovementBorder->fYMin_met) {
                pObjectMovementBorder->fYMin_met = fDistY;
                *fYMovement_met = pObjectMovementBorder->fYMax_met -
                                  pObjectMovementBorder->fYMin_met;
            }
        }
    }
}

/*****************************************************************************
  Functionname:CTACalculateObjectQualifiers */ /*!

                                   @brief:Calculates object quality related
                                 qualifiers

                                   @description:Calculates object quality
                                 related qualifiers,update rate,association
                                 probability filtered

                                   @param[in]  uiHighestAssocProb_per:Highest
                                 cluster association probability of the object
                                 filter result
                                               uiMaintenanceState_nu:the
                                 maintenance(measured,predicted) state whether
                                 the object is deleted
                                   @param[out] pfUpdateRate_nu:The object
                                 measurement update rate,unit:NULL
                                               pfAssocProbFiltered_nu:Filtered
                                 highest cluster association probability of the
                                 object filter result
                                   @return
                                 *****************************************************************************/
void CTACalculateObjectQualifiers(uint8 uiHighestAssocProb_per,
                                  uint8 uiMaintenanceState_nu,
                                  float32* pfUpdateRate_nu,
                                  float32* pfAssocProbFiltered_nu) {
    float32 fObjMeadured;
    float32 fFilterConst;
    const float32 fHighestAssocProb = (float32)uiHighestAssocProb_per * 0.01f;
    /**********************************************************************
     *Filter update rate
     **********************************************************************/
    if (uiMaintenanceState_nu == CTA_EM_GEN_OBJECT_MT_STATE_MEASURED) {
        fObjMeadured = 1.0f;
        fFilterConst = CTA_UPDATERATE_FILTER_UP;  // 0.05
    } else {
        fObjMeadured = 0.0f;
        fFilterConst = CTA_UPDATERATE_FILTER_DOWN;  // 0.025
    }

    GDB_Math_LowPassFilter(pfUpdateRate_nu, fObjMeadured, fFilterConst);

    /**********************************************************************
     *Filter highest association probability
     **********************************************************************/
    // Now not to use uiHighestAssocProb_per because miss the AssocProb data
    if (*pfAssocProbFiltered_nu < fHighestAssocProb) {
        fFilterConst = CTA_ASSOCPROB_FILTER_UP;  // 0.05
    } else {
        fFilterConst = CTA_ASSOCPROB_FILTER_DOWN;  // 0.05
    }
    GDB_Math_LowPassFilter(pfAssocProbFiltered_nu, fHighestAssocProb,
                           fFilterConst);
}
/*****************************************************************************
  Functionname:CTACalculatePosBasedVxVy                                     */ /*!

   @brief Calculate a filtered VrelX and VrelY based on the change in position
 of the object

   @description Calculate a filtered VrelX and VrelY based on the change in
 position of the object

   @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
               pEMFusionObjInput:EM fusion radar objects information
                           fXLastCycle_met:DistX of last cycle
                           fYLastCycle_met:DistY of last cycle
   @param[out] pfVxPosBased_mps:the VrelX based on the position of current and
 last cycle
               pfVyPosBased_mps:the VrelY based on the position of current and
 last cycle
   @return
 *****************************************************************************/
void CTACalculateFCTAPosBasedVxVy(const CTAParam_t* params,
                                  float32 fCycleTime_s,
                                  const CTA_FusionObject_t* pEMFusionObjInput,
                                  float32 fXLastCycle_met,
                                  float32 fYLastCycle_met,
                                  float32* pfVxPosBased_mps,
                                  float32* pfVyPosBased_mps) {
    float32 fDistX;
    float32 fDistY;
    fDistX = pEMFusionObjInput->fDistX_met -
             params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;
    if (pEMFusionObjInput->bRightSide) {
        fDistY =
            pEMFusionObjInput->fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            pEMFusionObjInput->fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }
    // Check if the current cycle time is valid and if a value for the position
    // of the last cycle has already been set
    if (fXLastCycle_met > (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA) ||
        fCycleTime_s < TUE_C_F32_DELTA) {
        *pfVxPosBased_mps = pEMFusionObjInput->fVrelX_mps;
        *pfVyPosBased_mps = pEMFusionObjInput->fVrelY_mps;
    } else {
        // Calculate a filtered vrelx and vrely based on the change in position
        // of the object
        GDB_Math_LowPassFilter(pfVxPosBased_mps,
                               (fDistX - fXLastCycle_met) / fCycleTime_s,
                               CTA_LPF_VRELXY_ALPHA);
        GDB_Math_LowPassFilter(pfVyPosBased_mps,
                               (fDistY - fYLastCycle_met) / fCycleTime_s,
                               CTA_LPF_VRELXY_ALPHA);
    }
}
/*****************************************************************************
  Functionname:CTACalculatePosBasedVxVy                                     */ /*!

   @brief Calculate a filtered VrelX and VrelY based on the change in position
 of the object

   @description Calculate a filtered VrelX and VrelY based on the change in
 position of the object

   @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
               pEMFusionObjInput:EM fusion radar objects information
                           fXLastCycle_met:DistX of last cycle
                           fYLastCycle_met:DistY of last cycle
   @param[out] pfVxPosBased_mps:the VrelX based on the position of current and
 last cycle
               pfVyPosBased_mps:the VrelY based on the position of current and
 last cycle
   @return
 *****************************************************************************/
void CTACalculateRCTAPosBasedVxVy(const CTAParam_t* params,
                                  float32 fCycleTime_s,
                                  const CTA_FusionObject_t* pEMFusionObjInput,
                                  float32 fXLastCycle_met,
                                  float32 fYLastCycle_met,
                                  float32* pfVxPosBased_mps,
                                  float32* pfVyPosBased_mps) {
    float32 fDistX;
    float32 fDistY;
    fDistX = pEMFusionObjInput->fDistX_met +
             (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
              params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met);
    if (pEMFusionObjInput->bRightSide) {
        fDistY =
            pEMFusionObjInput->fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            pEMFusionObjInput->fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }
    // Check if the current cycle time is valid and if a value for the position
    // of the last cycle has already been set
    if (fXLastCycle_met > (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA) ||
        fCycleTime_s < TUE_C_F32_DELTA) {
        *pfVxPosBased_mps = pEMFusionObjInput->fVrelX_mps;
        *pfVyPosBased_mps = pEMFusionObjInput->fVrelY_mps;
    } else {
        // Calculate a filtered vrelx and vrely based on the change in position
        // of the object
        GDB_Math_LowPassFilter(pfVxPosBased_mps,
                               (fDistX - fXLastCycle_met) / fCycleTime_s,
                               CTA_LPF_VRELXY_ALPHA);
        GDB_Math_LowPassFilter(pfVyPosBased_mps,
                               (fDistY - fYLastCycle_met) / fCycleTime_s,
                               CTA_LPF_VRELXY_ALPHA);
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
void CTACalculateFCTAAbsoluteObjectVelocity(
    const CTA_FusionObject_t* pEMFusionObjInput,
    const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput,
    float32* fVabs) {
    float32 fSensorLatPos;
    float32 fSensorLongOffset;
    const float32 fOverhangFront = 1.f;  // Todo: platform dependent, to be
    // provided by VehPar or algorithm param
    const float32 fOverhangRear = 0.9f;  // Todo: platform dependent, to be
    // provided by VehPar or algorithm param
    float32 fVxAbs;
    float32 fVyAbs;
    float32 fVabsObj;

    // calculate Vabs
    fVxAbs = pEgoVehicleInput->fegoVelocity_mps +
             pEMFusionObjInput->fVrelX_mps -
             pEgoVehicleInput->fYawRate_radps * (pEMFusionObjInput->fDistY_met);
    fVyAbs = pEMFusionObjInput->fVrelY_mps +
             pEgoVehicleInput->fYawRate_radps *
                 (pEMFusionObjInput->fDistX_met -
                  params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met +
                  params->CTA_Ks_VehicleParameter_nu.CTA_Kf_WheelBase_met);

    fVabsObj = SQRT(SQR(fVxAbs) + SQR(fVyAbs));
    *fVabs = fVabsObj;
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTACalculateRCTAAbsoluteObjectVelocity(
    const CTA_FusionObject_t* pEMFusionObjInput,
    const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput,
    float32* fVabs) {
    float32 fSensorLatPos;
    float32 fSensorLongOffset;
    const float32 fOverhangFront = 1.f;  // Todo: platform dependent, to be
    // provided by VehPar or algorithm param
    const float32 fOverhangRear = 0.9f;  // Todo: platform dependent, to be
    // provided by VehPar or algorithm param
    float32 fVxAbs;
    float32 fVyAbs;
    float32 fVabsObj;

    // calculate Vabs
    fVxAbs = pEgoVehicleInput->fegoVelocity_mps +
             pEMFusionObjInput->fVrelX_mps -
             pEgoVehicleInput->fYawRate_radps * (pEMFusionObjInput->fDistY_met);
    fVyAbs =
        pEMFusionObjInput->fVrelY_mps +
        pEgoVehicleInput->fYawRate_radps *
            (pEMFusionObjInput->fDistX_met +
             (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
              params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met) +
             params->CTA_Ks_VehicleParameter_nu.CTA_Kf_WheelBase_met);

    fVabsObj = SQRT(SQR(fVxAbs) + SQR(fVyAbs));
    *fVabs = fVabsObj;
}
/*****************************************************************************
  Functionname:CTACalculateCTObjectProperties */ /*!

                                 @brief

                                 @description

                                 @param[in]

                                 @param[out]

                                 @return
                               *****************************************************************************/
void CTACalculateCTObjectProperties(const CTAInReq_t* reqPorts,
                                    const CTAParam_t* params,
                                    CTAGlobal_t* pCTAGlobal) {
    // init all cyclic variables
    CTA_CTInitCyclic(pCTAGlobal);
    // calculate the maximum lateral sensor range
    CTA_CTCalculateMaxLatSensorRange(reqPorts->CTAEMFusionObjList.aObjects,
                                     reqPorts->EgoVehicleInfo.fegoVelocity_mps,
                                     &pCTAGlobal->CTGlobals.fMaxLatSensorRange);

    for (uint8 uObj = 0u; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
        CTObjectInfoGlobal_t* pCTObjGlobal = &pCTAGlobal->CTObjectList[uObj];
        const CTA_FusionObject_t* pEMFusionObjInput =
            &reqPorts->CTAEMFusionObjList.aObjects[uObj];

        if (!reqPorts->CTAEMFusionObjList.aObjects[uObj]
                 .uiMaintenanceState_nu == CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
            // Calculate distance to crossing line
            CTA_CTCalculateFCTADistToCrossingLine(
                params, pEMFusionObjInput, &pCTAGlobal->fSensorOffsetToSide_met,
                &pCTObjGlobal->fFCTADistToCrossingLine_met,
                &pCTObjGlobal->fDistToCrossingLineFiltered_met);
            CTA_CTCalculateRCTADistToCrossingLine(
                params, pEMFusionObjInput, &pCTAGlobal->fSensorOffsetToSide_met,
                &pCTObjGlobal->fRCTADistToCrossingLine_met);
            // Calculate x-axis breakthrough
            CTA_CTCalculateFCTAXBreakthrough(
                params, pEMFusionObjInput,
                pCTObjGlobal->fFCTADistToCrossingLine_met,
                &pCTObjGlobal->fFCTAXBreakthrough_met,
                &pCTObjGlobal->fFCTAXBreakthroughStd_met,
                &pCTObjGlobal->fFCTAXBreakthroughFiltered_met);
            CTA_CTCalculateRCTAXBreakthrough(
                params, pEMFusionObjInput,
                pCTObjGlobal->fRCTADistToCrossingLine_met,
                &pCTObjGlobal->fRCTAXBreakthrough_met,
                &pCTObjGlobal->fRCTAXBreakthroughStd_met,
                &pCTObjGlobal->fRCTAXBreakthroughFiltered_met);
            // Calculate Time to crossing
            CTA_CTCalculateTTC(
                reqPorts->fCycleTime_s, pEMFusionObjInput->bRightSide,
                pEMFusionObjInput->fVrelY_mps,
                pCTObjGlobal->fFCTADistToCrossingLine_met,
                &pCTObjGlobal->fFCTATTC_s, &pCTObjGlobal->fFCTATTCFiltered_s);
            CTA_CTCalculateTTC(
                reqPorts->fCycleTime_s, pEMFusionObjInput->bRightSide,
                pEMFusionObjInput->fVrelY_mps,
                pCTObjGlobal->fRCTADistToCrossingLine_met,
                &pCTObjGlobal->fRCTATTC_s, &pCTObjGlobal->fRCTATTCFiltered_s);
            // Calculate the rear object probability
            CTA_CTCalculateFCTARearObjectProbability(
                uObj, pEMFusionObjInput, reqPorts, pCTAGlobal,
                &pCTObjGlobal->fFCTARearTrackProb_per);
            CTA_CTCalculateRCTARearObjectProbability(
                uObj, pEMFusionObjInput, reqPorts, pCTAGlobal,
                &pCTObjGlobal->fRCTARearTrackProb_per);
            // Check the rear object
            CTA_CTCheckRearObject(pEMFusionObjInput->uClassification_nu,
                                  pCTObjGlobal->fFCTARearTrackProb_per,
                                  &pCTObjGlobal->bFCTARearTrack_nu);
            CTA_CTCheckRearObject(pEMFusionObjInput->uClassification_nu,
                                  pCTObjGlobal->fRCTARearTrackProb_per,
                                  &pCTObjGlobal->bRCTARearTrack_nu);
        }
    }
}
/*****************************************************************************
  Functionname:CTA_CTInitCyclic                                     */ /*!

           @brief:Initialize variables which are not stored across cycles

           @description:Initialize variables which are not stored across cycles

           @param[in]

           @param[out]

           @return
         *****************************************************************************/
void CTA_CTInitCyclic(CTAGlobal_t* pCTAGlobal) {
    // use same properties for RCTA and FCTA
    // init multi object handling
    pCTAGlobal->CTGlobals.iCriticalObjIDLastCycle =
        pCTAGlobal->CTGlobals.iCriticalObjID;
    pCTAGlobal->CTGlobals.iCriticalObjID = -1;
    pCTAGlobal->CTGlobals.fCriticalObjDistYLastCycle =
        pCTAGlobal->CTGlobals.fCriticalObjDistY;
    pCTAGlobal->CTGlobals.fCriticalObjDistY = TUE_C_F32_VALUE_INVALID;
    pCTAGlobal->CTGlobals.fCriticalTTC = TUE_C_F32_VALUE_INVALID;
}
/*****************************************************************************
  Functionname:CTA_CTCalculateMaxLatSensorRange */ /*!

                               @brief: calculate the current maximum lateral
                             sensor range based on crossing object information

                               @description: calculate the current maximum
                             lateral sensor range based on crossing object
                             information

                               @param[in]

                               @param[out]

                               @return
                             *****************************************************************************/
void CTA_CTCalculateMaxLatSensorRange(
    const CTA_FusionObject_t* pCTAEMFusionObjList,
    float32 fegoVelocity_mps,
    float32* pfMaxLatSensorRange) {
    float32 fMaxLatSensorRange = *pfMaxLatSensorRange;
    if (fegoVelocity_mps < CTA_CT_MAXLATERALRANGE_MIN_EGOSPEED) {
        if (*pfMaxLatSensorRange >
            (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA)) {
            *pfMaxLatSensorRange = 0.f;
        }
        // Loop over all objects
        for (uint8 uObj = 0u; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
            if (!pCTAEMFusionObjList[uObj].uiMaintenanceState_nu ==
                CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
                float32 fDistY = pCTAEMFusionObjList[uObj].fDistY_met;
                float32 fVrelY = pCTAEMFusionObjList[uObj].fVrelY_mps;
                // check if on right sensor, revert the sign of the Y distance
                // for using the old coordinate system
                if (pCTAEMFusionObjList[uObj].bRightSide) {
                    fDistY = fDistY * -1.f;
                    fVrelY = fVrelY * -1.f;
                }
                if (fVrelY < CTA_CT_MAXLATRANGE_MAX_VRELX &&
                    pCTAEMFusionObjList[uObj].fDistX_met <
                        CTA_CT_MAXLATRANGE_MAX_DISTX &&
                    fDistY < CTA_CT_MAXLATRANGE_MAX_YDIST &&
                    fDistY > CTA_CT_MAXLATRANGE_MIN_YDIST &&
                    pCTAEMFusionObjList[uObj].fProbabilityOfExistence_per >
                        CTA_CT_MAXLATRANGE_MIN_POE) {
                    if (pCTAEMFusionObjList[uObj].fFirstDetectY_met >
                        *pfMaxLatSensorRange) {
                        fMaxLatSensorRange =
                            pCTAEMFusionObjList[uObj].fFirstDetectY_met;
                    }
                }
            }
        }
    } else {
        fMaxLatSensorRange = TUE_C_F32_VALUE_INVALID;
    }

    *pfMaxLatSensorRange = fMaxLatSensorRange;
}
/*****************************************************************************
  Functionname:CTA_CTCalculateDistToCrossingLine */ /*!

                              @brief:Calculate the distance of the object to the
                            crossing line

                              @description:Calculate the distance of the object
                            to the crossing line

                              @param[in]  pEMFusionObjInput:EM fusion radar object
                            information structure

                              @param[out]

                              @return
                            *****************************************************************************/
void CTA_CTCalculateFCTADistToCrossingLine(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met,
    float32* pfDistToCrossingLineFiltered_met) {
    float32 fDistToCrossingLine;
    float32 fClosestDistY;
    float32 fDistYFLCorner;
    float32 fDistYFRCorner;
    float32 fSensorOffsetToSide = 0.f;
    float32 fDistY;

    // Reference point of object to calculate distance to crossing line is the
    // closest of the two front corners to crossing
    // line because objects are now rotated
    if (pEMFusionObjInput->bRightSide) {
        fDistY =
            pEMFusionObjInput->fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            pEMFusionObjInput->fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }
    fDistYFLCorner = fDistY +
                     pEMFusionObjInput->fLengthFront_met *
                         SIN_(pEMFusionObjInput->fAbsOrientation_rad) +
                     pEMFusionObjInput->fWidthLeft_met *
                         COS_(pEMFusionObjInput->fAbsOrientation_rad);
    fDistYFRCorner = fDistY +
                     pEMFusionObjInput->fLengthFront_met *
                         SIN_(pEMFusionObjInput->fAbsOrientation_rad) -
                     pEMFusionObjInput->fWidthRight_met *
                         COS_(pEMFusionObjInput->fAbsOrientation_rad);

    if (!pEMFusionObjInput->bRightSide) {
        // On the left sensor the closest corner always has smallest Y distance
        // between the two front corners
        fClosestDistY = MIN(fDistYFLCorner, fDistYFRCorner);
    } else {
        // On the left sensor the closest corner always has smallest Y distance
        // between the two front corners
        fClosestDistY = MAX(fDistYFLCorner, fDistYFRCorner);
        // Revert the sign of the Y distance for using the old coordinate system
        fClosestDistY = fClosestDistY * -1.f;
    }

    fDistToCrossingLine = fClosestDistY;
    *pfDistToCrossingLine_met = fDistToCrossingLine;

    if (*pfDistToCrossingLineFiltered_met <
        TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA) {
        TUE_CML_LowPassFilter(pfDistToCrossingLineFiltered_met,
                              *pfDistToCrossingLine_met,
                              CTA_CT_LPF_DIST2CROSS_FILTER);
    } else {
        *pfDistToCrossingLineFiltered_met = *pfDistToCrossingLine_met;
    }
}
/*****************************************************************************
  Functionname:CTA_CTCalculateDistToCrossingLine */ /*!

                              @brief:Calculate the distance of the object to the
                            crossing line

                              @description:Calculate the distance of the object
                            to the crossing line

                              @param[in]  pEMFusionObjInput:EM fusion radar object
                            information structure

                              @param[out]

                              @return
                            *****************************************************************************/
void CTA_CTCalculateRCTADistToCrossingLine(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met) {
    float32 fDistToCrossingLine;
    float32 fClosestDistY;
    float32 fDistYFLCorner;
    float32 fDistYFRCorner;
    float32 fSensorOffsetToSide = 0.f;
    float32 fDistY;

    // Reference point of object to calculate distance to crossing line is the
    // closest of the two front corners to crossing
    // line because objects are now rotated

    if (pEMFusionObjInput->bRightSide) {
        fDistY =
            pEMFusionObjInput->fDistY_met +
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    } else {
        fDistY =
            pEMFusionObjInput->fDistY_met -
            0.5f * params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
    }
    fDistYFLCorner = fDistY +
                     pEMFusionObjInput->fLengthFront_met *
                         SIN_(pEMFusionObjInput->fAbsOrientation_rad) +
                     pEMFusionObjInput->fWidthLeft_met *
                         COS_(pEMFusionObjInput->fAbsOrientation_rad);
    fDistYFRCorner = fDistY +
                     pEMFusionObjInput->fLengthFront_met *
                         SIN_(pEMFusionObjInput->fAbsOrientation_rad) -
                     pEMFusionObjInput->fWidthRight_met *
                         COS_(pEMFusionObjInput->fAbsOrientation_rad);

    if (!pEMFusionObjInput->bRightSide) {
        // On the left sensor the closest corner always has smallest Y distance
        // between the two front corners
        fClosestDistY = MIN(fDistYFLCorner, fDistYFRCorner);
    } else {
        // On the left sensor the closest corner always has smallest Y distance
        // between the two front corners
        fClosestDistY = MAX(fDistYFLCorner, fDistYFRCorner);
        // Revert the sign of the Y distance for using the old coordinate system
        fClosestDistY = fClosestDistY * -1.f;
    }

    fDistToCrossingLine = fClosestDistY;
    *pfDistToCrossingLine_met = fDistToCrossingLine;
}
/*****************************************************************************
  Functionname:CTA_CTCalculateXBreakthrough */ /*!

                                   @brief:Calculate y-axis breakthrough of this
                                 object

                                   @description:Calculate y-axis breakthrough of
                                 this object, also check its start position

                                   @param[in]  pEMFusionObjInput:EM fusion radar
                                 object information structure
                                                           fDistToCrossingLine_met:Distance
                                 to crossing line

                                   @param[out] pfXBreakthrough_met
                                               pfXBreakthroughStd_met
                                               pfXBreakthroughFiltered_met
                                   @return
                                 *****************************************************************************/
void CTA_CTCalculateFCTAXBreakthrough(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    float32 fDistToCrossingLine_met,
    float32* pfXBreakthrough_met,
    float32* pfXBreakthroughStd_met,
    float32* pfXBreakthroughFiltered_met) {
    float32 fHeading = pEMFusionObjInput->fRelHeading_rad;
    float32 fHeadingStd = pEMFusionObjInput->fRelHeadingStd_rad;
    float32 fXBreakthrough = TUE_C_F32_VALUE_INVALID;
    float32 fXBreakthroughStd = TUE_C_F32_VALUE_INVALID;

    // Check if on right sensor, revert the sign of the heading for using the
    // old coordinate system
    if (pEMFusionObjInput->bRightSide) {
        fHeading = fHeading * -1.f;
    }
    // For object which move towards the X axis calculate the breakthrough
    if (fHeading < -TUE_C_F32_DELTA &&
        fHeading > -(TUE_CML_Pi - TUE_C_F32_DELTA)) {
        float32 fTanHeading = TAN_(fHeading + TUE_CML_Pi / 2.f);
        float32 fXBreakthrough_fDistToCrossingLine;
        float32 fXBreakthrough_fDistX;
        float32 fXBreakthrough_fHeading;
        // Take the object position

        fXBreakthrough =
            pEMFusionObjInput->fDistX_met -
            params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;

        if (!pEMFusionObjInput->bRightSide) {
            fXBreakthrough += 0.5f * (pEMFusionObjInput->fWidthLeft_met -
                                      pEMFusionObjInput->fWidthRight_met);
        } else {
            fXBreakthrough += 0.5f * (pEMFusionObjInput->fWidthRight_met -
                                      pEMFusionObjInput->fWidthLeft_met);
        }
        // Add the heading angle dependent part
        fXBreakthrough += fTanHeading * (fDistToCrossingLine_met);

        // Calculate the Std
        /*The error propagation is calculated using following formula:
          q = f(x1,...,xn)
          the uncertainty is calculated:
          q_std = SQRT((dq/dx1 * x1_std)^2 + .... + (dq/dxn * xn_std)^2)*/
        // Calculate the partial derivatives of the breakthrough calculation
        // formula
        fXBreakthrough_fDistX = 1.0f;
        fXBreakthrough_fHeading =
            (2.0f * fDistToCrossingLine_met) /
            SafeDiv(COS_(2.0f * (fHeading + TUE_CML_Pi)) + 1.0f);
        fXBreakthrough_fDistToCrossingLine = fTanHeading;
        // Calculate the square sum of the partial derivatives times the
        // corresponding standard deviation
        fXBreakthroughStd =
            SQR(fXBreakthrough_fDistX * pEMFusionObjInput->fDistXStd_met);
        fXBreakthroughStd += SQR(fXBreakthrough_fHeading * fHeadingStd);
        fXBreakthroughStd += SQR(fXBreakthrough_fDistToCrossingLine *
                                 pEMFusionObjInput->fDistYStd_met);
        // Calculate the root
        fXBreakthroughStd = SQRT(fXBreakthroughStd);
        // Limit the value to reasonable limits
        fXBreakthroughStd = MIN(fXBreakthroughStd, TUE_C_F32_VALUE_INVALID);
    }
    *pfXBreakthrough_met = fXBreakthrough;
    *pfXBreakthroughStd_met = fXBreakthroughStd;

    // if the calculated breakthrough is not invalid
    if (*pfXBreakthrough_met < (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA)) {
        // In the first cycle the filtered breakthrough is set to the
        // current breakthrough
        if (*pfXBreakthroughFiltered_met > TUE_C_F32_VALUE_INVALID - 1.f) {
            *pfXBreakthroughFiltered_met = *pfXBreakthrough_met;
        } else {
            // Afterwards filter the value. The filter speed depends on the
            // lifetime of the object. => Young objects -> fast filter
            *pfXBreakthroughFiltered_met = TUE_CML_MacroLowPassFilter(
                *pfXBreakthrough_met, *pfXBreakthroughFiltered_met,
                TUE_CML_Min(pEMFusionObjInput->uiLifeCycles_nu,
                            CTA_CT_XBREAKTHROUGH_MIN_FILTER));  // 10
        }
    }
}
/*****************************************************************************
  Functionname:CTA_CTCalculateXBreakthrough */ /*!

                                   @brief:Calculate y-axis breakthrough of this
                                 object

                                   @description:Calculate y-axis breakthrough of
                                 this object, also check its start position

                                   @param[in]  pEMFusionObjInput:EM fusion radar
                                 object information structure
                                                           fDistToCrossingLine_met:Distance
                                 to crossing line

                                   @param[out] pfXBreakthrough_met
                                               pfXBreakthroughStd_met
                                               pfXBreakthroughFiltered_met
                                   @return
                                 *****************************************************************************/
void CTA_CTCalculateRCTAXBreakthrough(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    float32 fDistToCrossingLine_met,
    float32* pfXBreakthrough_met,
    float32* pfXBreakthroughStd_met,
    float32* pfXBreakthroughFiltered_met) {
    float32 fHeading = pEMFusionObjInput->fRelHeading_rad;
    float32 fHeadingStd = pEMFusionObjInput->fRelHeadingStd_rad;
    float32 fXBreakthrough = TUE_C_F32_VALUE_INVALID;
    float32 fXBreakthroughStd = TUE_C_F32_VALUE_INVALID;

    // Check if on right sensor, revert the sign of the heading for using the
    // old coordinate system
    if (pEMFusionObjInput->bRightSide) {
        fHeading = fHeading * -1.f;
    }
    // For object which move towards the X axis calculate the breakthrough
    if (fHeading < -TUE_C_F32_DELTA &&
        fHeading > -(TUE_CML_Pi - TUE_C_F32_DELTA)) {
        float32 fTanHeading = TAN_(fHeading + TUE_CML_Pi / 2.f);
        float32 fXBreakthrough_fDistToCrossingLine;
        float32 fXBreakthrough_fDistX;
        float32 fXBreakthrough_fHeading;
        // Take the object position
        fXBreakthrough =
            pEMFusionObjInput->fDistX_met +
            (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
             params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met);

        if (!pEMFusionObjInput->bRightSide) {
            fXBreakthrough += 0.5f * (pEMFusionObjInput->fWidthLeft_met -
                                      pEMFusionObjInput->fWidthRight_met);
        } else {
            fXBreakthrough += 0.5f * (pEMFusionObjInput->fWidthRight_met -
                                      pEMFusionObjInput->fWidthLeft_met);
        }
        // Add the heading angle dependent part
        fXBreakthrough += fTanHeading * (fDistToCrossingLine_met);

        // Calculate the Std
        /*The error propagation is calculated using following formula:
          q = f(x1,...,xn)
          the uncertainty is calculated:
          q_std = SQRT((dq/dx1 * x1_std)^2 + .... + (dq/dxn * xn_std)^2)*/
        // Calculate the partial derivatives of the breakthrough calculation
        // formula
        fXBreakthrough_fDistX = 1.0f;
        fXBreakthrough_fHeading =
            (2.0f * fDistToCrossingLine_met) /
            SafeDiv(COS_(2.0f * (fHeading + TUE_CML_Pi)) + 1.0f);
        fXBreakthrough_fDistToCrossingLine = fTanHeading;
        // Calculate the square sum of the partial derivatives times the
        // corresponding standard deviation
        fXBreakthroughStd =
            SQR(fXBreakthrough_fDistX * pEMFusionObjInput->fDistXStd_met);
        fXBreakthroughStd += SQR(fXBreakthrough_fHeading * fHeadingStd);
        fXBreakthroughStd += SQR(fXBreakthrough_fDistToCrossingLine *
                                 pEMFusionObjInput->fDistYStd_met);
        // Calculate the root
        fXBreakthroughStd = SQRT(fXBreakthroughStd);
        // Limit the value to reasonable limits
        fXBreakthroughStd = MIN(fXBreakthroughStd, TUE_C_F32_VALUE_INVALID);
    }
    *pfXBreakthrough_met = fXBreakthrough;
    *pfXBreakthroughStd_met = fXBreakthroughStd;

    // if the calculated breakthrough is not invalid
    if (*pfXBreakthrough_met < (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA)) {
        // In the first cycle the filtered breakthrough is set to the
        // current breakthrough
        if (*pfXBreakthroughFiltered_met > TUE_C_F32_VALUE_INVALID - 1.f) {
            *pfXBreakthroughFiltered_met = *pfXBreakthrough_met;
        } else {
            // Afterwards filter the value. The filter speed depends on the
            // lifetime of the object. => Young objects -> fast filter
            *pfXBreakthroughFiltered_met = TUE_CML_MacroLowPassFilter(
                *pfXBreakthrough_met, *pfXBreakthroughFiltered_met,
                TUE_CML_Min(pEMFusionObjInput->uiLifeCycles_nu,
                            CTA_CT_XBREAKTHROUGH_MIN_FILTER));  // 10
        }
    }
}
/*****************************************************************************
  Functionname:CTA_CTCalculateTTC                                     */ /*!

         @brief:Calculate TTC to computed breakthrough

         @description:Calculate TTC to computed breakthrough using constant
       speed assumption

         @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
                     eSensorMountingPos
                                 fVrelY_mps
                                 fDistToCrossingLine_met
         @param[out] pfTTC_s
                     pfTTCFiltered_s

         @return
       *****************************************************************************/
void CTA_CTCalculateTTC(float32 fCycleTime_s,
                        const boolean bRightSide,
                        const float32 fVrelY_mps,
                        float32 fDistToCrossingLine_met,
                        float32* pfTTC_s,
                        float32* pfTTCFiltered_s) {
    float32 fVrelY = fVrelY_mps;
    float32 fTTC;
    float32 fTTCFiltered = *pfTTCFiltered_s;

    // Check if on right sensor, revert the sign of the velocity for using the
    // old coordinate system
    if (bRightSide) {
        fVrelY = fVrelY * -1.f;
    }
    // Calculate the TTC
    fTTC = fDistToCrossingLine_met / SafeDiv(-fVrelY);
    // if the TTC is negative or when the object already passed the crossing
    // line
    if (fTTC < 0.f || fDistToCrossingLine_met < 0.f) {
        fTTC = TUE_C_F32_VALUE_INVALID;
    }
    // For large TTCs set TTC filtered to TTC
    if (fTTC > CTA_CT_LARGE_TTC || fTTCFiltered > CTA_CT_LARGE_TTC) {
        fTTCFiltered = fTTC;
    } else {
        float32 fFilterSpeed;
        // Predicted the TTC
        fTTCFiltered -= fCycleTime_s;
        // calculate the filter speed for increasing an for decreasing TTC
        if (fTTCFiltered > fTTC) {
            // For close objects set slow filter constant
            fFilterSpeed = GDBmathLinFuncLimBounded(
                fDistToCrossingLine_met, 0.f, CTA_CT_TTCFILTER_MAX_DIST2CROS,
                CTA_CT_TTCFILTER_DOWN_MIN_ALPHA,
                CTA_CT_TTCFILTER_DOWN_MAX_ALPHA);  // 0  15  0.1  0.3
        } else {
            // For close objects set slow filter constant
            fFilterSpeed = GDBmathLinFuncLimBounded(
                fDistToCrossingLine_met, 0.f, CTA_CT_TTCFILTER_MAX_DIST2CROS,
                CTA_CT_TTCFILTER_UP_MIN_ALPHA,
                CTA_CT_TTCFILTER_UP_MAX_ALPHA);  // 0  15  0.05  0.2
        }
        // Filter TTC
        TUE_CML_LowPassFilter(&fTTCFiltered, fTTC, fFilterSpeed);
    }
    *pfTTCFiltered_s = fTTCFiltered;
    *pfTTC_s = fTTC;
}
/*****************************************************************************
  Functionname:CTA_CTCalculateRearObjectProbability */ /*!

                           @brief:Calculate a probability value for this object
                         to be a rear object, by looking for objects in front of
                         it

                           @description:Calculate a probability value for this
                         object to be a rear object, by looking for objects in
                         front of it

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTA_CTCalculateFCTARearObjectProbability(
    uint8 uObj,
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts,
    CTAGlobal_t* pCTAGlobal,
    float32* fFCTARearTrackProb_per) {
    float32 fDistYCandObj = 0.f;
    float32 bAbortLoop = FALSE;
    CTARearObjState_t eRearObjState = CTA_REARTRACK_NO;
    float32 fFCTAProbChange = 0.f;

    float32 fDistYCurrObj = pCurrObjectEMInput->fDistY_met;
    // Check if on right sensor, revert the sign of the Y distance for using the
    // old coordinate system
    if (pCurrObjectEMInput->bRightSide) {
        fDistYCurrObj = fDistYCurrObj * -1.f;
    }
    // loop over all object
    for (uint8 uIndex = 0u; (uIndex < CTA_MAX_NUM_OBJECTS) && (!bAbortLoop) &&
                            (eRearObjState == CTA_REARTRACK_NO);
         uIndex++) {
        const CTA_FusionObject_t* pObjCandEMInfo =
            &reqPorts->CTAEMFusionObjList.aObjects[uIndex];
        CTObjectInfoGlobal_t* pCTObjCandGlobal =
            &pCTAGlobal->CTObjectList[uIndex];
        CTAObjectInfoGlobal_t* pCTAObjCandGlobal =
            &pCTAGlobal->CTAObjectList[uIndex];
        CTAObjectInfoGlobal_t* pCTAObjCurrGlobal =
            &pCTAGlobal->CTAObjectList[uObj];
        CTAFCTAObjectInfoGlobal_t* pCTAFCTAObjGlobal =
            &pCTAGlobal->CTAFCTAObjectList[uIndex];

        if (!pObjCandEMInfo->uiMaintenanceState_nu ==
                CTA_EM_GEN_OBJECT_MT_STATE_DELETED &&
            uObj != uIndex) {
            fDistYCandObj = pObjCandEMInfo->fDistY_met;
            if (pObjCandEMInfo->bRightSide) {
                fDistYCandObj = fDistYCandObj * -1.f;
            }
            // If the object is in the area of interest or the object is behind
            // the search object
            if ((pCurrObjectEMInput->fDistX_met - pObjCandEMInfo->fDistX_met) >
                    CTA_CT_REAROBJPROB_DISTX_THRESH ||  // 10
                (fDistYCurrObj - fDistYCandObj) < 0.f) {
                // This object is not in the area of interest check the next one
            } else if ((pCurrObjectEMInput->fDistX_met -
                        pObjCandEMInfo->fDistX_met) <
                       -CTA_CT_REAROBJPROB_DISTX_THRESH) {
                // This and all following objects are in front of the area of
                // interest we can abort the loop
                bAbortLoop = TRUE;
            } else if (pCTAFCTAObjGlobal->bRelevant &&
                       !pCTObjCandGlobal->bFCTARearTrack_nu) {
                eRearObjState = CTA_CTCalculateFCTASearchFrontObject(
                    pCurrObjectEMInput, pObjCandEMInfo, pCTAObjCurrGlobal,
                    pCTAObjCandGlobal,
                    pCTAGlobal->CTObjectList[uObj].fFCTARearTrackProb_per);
            } else {
                // MISRA happiness
            }
        }
    }

    // Adapt the rear object probability
    switch (eRearObjState) {
        case CTA_REARTRACK_POS_SPEED:

            // The condition is completely fulfilled: increase the probability
            if (*fFCTARearTrackProb_per >
                CTA_FCTA_REARTRACK_IMPLAUSIBLE_THRESH) {
                float32 fFCTALifeCycleThresh = 0.f;
                float32 fFCTATemp;
                fFCTALifeCycleThresh = TUE_CML_BoundedLinInterpol2(
                    pCurrObjectEMInput->fFirstDetectY_met,
                    CTA_CT_LI_REARPROB_MIN_FIRSTY,
                    CTA_CT_LI_REARPROB_MAX_FIRSTY,
                    CTA_CT_LI_REARPROB_MAX_LIFECYCLE,
                    CTA_CT_LI_REARPROB_MIN_LIFECYCLE);  // 20  40  75  50
                // Reduce the max probability change for old objects
                fFCTATemp = TUE_CML_BoundedLinInterpol2(
                    (float32)pCurrObjectEMInput->uiLifeCycles_nu,
                    (float32)fFCTALifeCycleThresh, 0.f, 0.f,
                    CTA_FCTA_REARTRACK_INC_PROB);

                // Reduce the max probability change for objects near the y-axes
                fFCTAProbChange = TUE_CML_BoundedLinInterpol2(
                    (float32)fDistYCurrObj, 0.f,
                    CTA_CT_LI_MAXREARPROB_MAX_DISTY, 0.f,
                    CTA_FCTA_REARTRACK_INC_PROB);
                // Use the minimum of all limitations
                fFCTAProbChange = MIN(fFCTAProbChange, fFCTATemp);
            }
            break;
        case CTA_REARTRACK_POS:
            // The condition is partly fulfilled: keep the probability
            break;
        case CTA_REARTRACK_NO:
            // The condition is not fulfilled: decrease the probability
            fFCTAProbChange = -CTA_FCTA_REARTRACK_DEC_PROB;  // 0.02
            break;
        default:
            // Do nothing
            break;
    }
    // Update the probability
    *fFCTARearTrackProb_per += fFCTAProbChange;
    // Limit the rear object probability
    *fFCTARearTrackProb_per = TUE_CML_MinMax(0.f, 1.f, *fFCTARearTrackProb_per);
}
/*****************************************************************************
  Functionname:CTA_CTCalculateRearObjectProbability */ /*!

                           @brief:Calculate a probability value for this object
                         to be a rear object, by looking for objects in front of
                         it

                           @description:Calculate a probability value for this
                         object to be a rear object, by looking for objects in
                         front of it

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTA_CTCalculateRCTARearObjectProbability(
    uint8 uObj,
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts,
    CTAGlobal_t* pCTAGlobal,
    float32* fRCTARearTrackProb_per) {
    float32 fDistYCandObj = 0.f;
    float32 bAbortLoop = FALSE;
    CTARearObjState_t eRearObjState = CTA_REARTRACK_NO;
    float32 fRCTAProbChange = 0.f;

    float32 fDistYCurrObj = pCurrObjectEMInput->fDistY_met;
    // Check if on right sensor, revert the sign of the Y distance for using the
    // old coordinate system
    if (pCurrObjectEMInput->bRightSide) {
        fDistYCurrObj = fDistYCurrObj * -1.f;
    }
    // loop over all object
    for (uint8 uIndex = 0u; (uIndex < CTA_MAX_NUM_OBJECTS) && (!bAbortLoop) &&
                            (eRearObjState == CTA_REARTRACK_NO);
         uIndex++) {
        const CTA_FusionObject_t* pObjCandEMInfo =
            &reqPorts->CTAEMFusionObjList.aObjects[uIndex];
        CTObjectInfoGlobal_t* pCTObjCandGlobal =
            &pCTAGlobal->CTObjectList[uIndex];
        CTAObjectInfoGlobal_t* pCTAObjCandGlobal =
            &pCTAGlobal->CTAObjectList[uIndex];
        CTAObjectInfoGlobal_t* pCTAObjCurrGlobal =
            &pCTAGlobal->CTAObjectList[uObj];
        CTAFCTAObjectInfoGlobal_t* pCTAFCTAObjGlobal =
            &pCTAGlobal->CTAFCTAObjectList[uIndex];

        if (!pObjCandEMInfo->uiMaintenanceState_nu ==
                CTA_EM_GEN_OBJECT_MT_STATE_DELETED &&
            uObj != uIndex) {
            fDistYCandObj = pObjCandEMInfo->fDistY_met;
            if (pObjCandEMInfo->bRightSide) {
                fDistYCandObj = fDistYCandObj * -1.f;
            }
            // If the object is in the area of interest or the object is behind
            // the search object
            if ((pCurrObjectEMInput->fDistX_met - pObjCandEMInfo->fDistX_met) >
                    CTA_CT_REAROBJPROB_DISTX_THRESH ||  // 10
                (fDistYCurrObj - fDistYCandObj) < 0.f) {
                // This object is not in the area of interest check the next one
            } else if ((pCurrObjectEMInput->fDistX_met -
                        pObjCandEMInfo->fDistX_met) <
                       -CTA_CT_REAROBJPROB_DISTX_THRESH) {
                // This and all following objects are in front of the area of
                // interest we can abort the loop
                bAbortLoop = TRUE;
            } else if (pCTAFCTAObjGlobal->bRelevant &&
                       !pCTObjCandGlobal->bRCTARearTrack_nu) {
                eRearObjState = CTA_CTCalculateRCTASearchFrontObject(
                    pCurrObjectEMInput, pObjCandEMInfo, pCTAObjCurrGlobal,
                    pCTAObjCandGlobal,
                    pCTAGlobal->CTObjectList[uObj].fRCTARearTrackProb_per);
            } else {
                // MISRA happiness
            }
        }
    }

    // Adapt the rear object probability
    switch (eRearObjState) {
        case CTA_REARTRACK_POS_SPEED:

            // The condition is completely fulfilled: increase the probability
            if (*fRCTARearTrackProb_per >
                CTA_FCTA_REARTRACK_IMPLAUSIBLE_THRESH) {
                float32 fFCTALifeCycleThresh = 0.f;
                float32 fRCTALifeCycleThresh = 0.f;
                float32 fFCTATemp;
                float32 fRCTATemp;

                fRCTALifeCycleThresh = CTA_CT_LI_REARPROB_MIN_LIFECYCLE;
                // Reduce the max probability change for old objects
                fRCTATemp = TUE_CML_BoundedLinInterpol2(
                    (float32)pCurrObjectEMInput->uiLifeCycles_nu,
                    (float32)fRCTALifeCycleThresh, 0.f, 0.f,
                    CTA_FCTA_REARTRACK_INC_PROB);
                // Reduce the max probability change for objects near the y-axes
                fRCTAProbChange = TUE_CML_BoundedLinInterpol2(
                    (float32)fDistYCurrObj, 0.f,
                    CTA_CT_LI_MAXREARPROB_MAX_DISTY, 0.f,
                    CTA_FCTA_REARTRACK_INC_PROB);
                // Use the minimum of all limitations
                fRCTAProbChange = MIN(fRCTAProbChange, fFCTATemp);
            }
            break;
        case CTA_REARTRACK_POS:
            // The condition is partly fulfilled: keep the probability
            break;
        case CTA_REARTRACK_NO:
            // The condition is not fulfilled: decrease the probability
            fRCTAProbChange = -CTA_FCTA_REARTRACK_DEC_PROB;  // 0.02
            break;
        default:
            // Do nothing
            break;
    }
    // Update the probability
    *fRCTARearTrackProb_per += fRCTAProbChange;
    // Limit the rear object probability
    *fRCTARearTrackProb_per = TUE_CML_MinMax(0.f, 1.f, *fRCTARearTrackProb_per);
}
/*****************************************************************************
  Functionname:CTA_CTCalculateSearchFrontObject */ /*!

                               @brief Search for objects in front of the current
                             object

                               @description Search for objects in front of the
                             current object

                               @param[in]

                               @param[out]

                               @return
                             *****************************************************************************/
CTARearObjState_t CTA_CTCalculateFCTASearchFrontObject(
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTA_FusionObject_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal,
    float32 fFCTARearTrackProb_per) {
    float32 fXDiff;
    float32 fYDiff;
    float32 fVxDiff;
    float32 fVyDiff;
    float32 fVyDiffPosBased;
    float32 fMaxRangeDiff;
    float32 fMaxDistXDiff;
    float32 fMaxDistYDiff;

    CTARearObjState_t eObjRearTrackState = CTA_REARTRACK_NO;
    // Use the heading values because it represents the relative heading of the
    // target
    float32 fHeading = pObjCandEMInfo->fRelHeading_rad;

    // check if on right sensor, revert the sign of the heading for using the
    // old coordinate system
    if (pObjCandEMInfo->bRightSide) {
        fHeading = fHeading * -1.f;
    }
    // Calculate object position difference
    fXDiff = CTA_CTCalculateObjectDistance(
        pCTAObjCurrGlobal->ObjectBorder.fXMin_met,
        pCTAObjCurrGlobal->ObjectBorder.fXMax_met,
        pCTAObjCandGlobal->ObjectBorder.fXMin_met,
        pCTAObjCandGlobal->ObjectBorder.fXMax_met);
    fYDiff = CTA_CTCalculateObjectDistance(
        pCTAObjCurrGlobal->ObjectBorder.fYMin_met,
        pCTAObjCurrGlobal->ObjectBorder.fYMax_met,
        pCTAObjCandGlobal->ObjectBorder.fYMin_met,
        pCTAObjCandGlobal->ObjectBorder.fYMax_met);
    fVxDiff = fABS(pObjCandEMInfo->fVrelX_mps - pCurrObjectEMInput->fVrelX_mps);
    fVyDiff = fABS(pObjCandEMInfo->fVrelY_mps - pCurrObjectEMInput->fVrelY_mps);
    fVyDiffPosBased = fABS(pCTAObjCandGlobal->fFCTAVyPosBased_mps -
                           pCTAObjCurrGlobal->fFCTAVyPosBased_mps);
    // Calculate max differences. Assume that two following vehicles have at
    // least CTA_CT_MULTIOBJ_SAFE_MARGIN security distance Multiply with the
    // assumed security distance S = V*t
    fMaxRangeDiff = SQRT(SQR(pCurrObjectEMInput->fVabsX_mps) +
                         SQR(pCurrObjectEMInput->fVabsY_mps)) *
                    CTA_CT_MULTIOBJ_SAFE_MARGIN;
    // Calculate the limits depending on the heading angle
    fMaxDistXDiff = fABS(COS_(fHeading) * fMaxRangeDiff);
    fMaxDistYDiff = fABS(SIN_(fHeading) * fMaxRangeDiff);
    // Limit the thresholds
    fMaxDistXDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                   CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistXDiff);
    fMaxDistYDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                   CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistYDiff);
    // Check rear object conditions
    if (fXDiff < fMaxDistXDiff && fYDiff < fMaxDistYDiff) {
        float32 fMaxVxDiff;
        float32 fMaxVyDiff;
        // Position criteria is fulfilled
        eObjRearTrackState = CTA_REARTRACK_POS;
        // Set velocity thresholds
        fMaxVxDiff = TUE_CML_BoundedLinInterpol2(
            fFCTARearTrackProb_per, 0.f, 1.f, CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
            CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
        fMaxVyDiff = TUE_CML_BoundedLinInterpol2(
            fFCTARearTrackProb_per, 0.f, 1.f, CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
            CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
        // Check speed criteria
        if (fVxDiff < fMaxVxDiff &&
            (fVyDiff < fMaxVyDiff || fVyDiffPosBased < fMaxVyDiff)) {
            eObjRearTrackState = CTA_REARTRACK_POS_SPEED;
        }
    }
    // return rear track bool
    return eObjRearTrackState;
}
/*****************************************************************************
  Functionname:CTA_CTCalculateSearchFrontObject */ /*!

                               @brief Search for objects in front of the current
                             object

                               @description Search for objects in front of the
                             current object

                               @param[in]

                               @param[out]

                               @return
                             *****************************************************************************/
CTARearObjState_t CTA_CTCalculateRCTASearchFrontObject(
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTA_FusionObject_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal,
    float32 fRCTARearTrackProb_per) {
    float32 fXDiff;
    float32 fYDiff;
    float32 fVxDiff;
    float32 fVyDiff;
    float32 fVyDiffPosBased;
    float32 fMaxRangeDiff;
    float32 fMaxDistXDiff;
    float32 fMaxDistYDiff;

    CTARearObjState_t eObjRearTrackState = CTA_REARTRACK_NO;
    // Use the heading values because it represents the relative heading of the
    // target
    float32 fHeading = pObjCandEMInfo->fRelHeading_rad;

    // check if on right sensor, revert the sign of the heading for using the
    // old coordinate system
    if (pObjCandEMInfo->bRightSide) {
        fHeading = fHeading * -1.f;
    }
    // Calculate object position difference
    fXDiff = CTA_CTCalculateObjectDistance(
        pCTAObjCurrGlobal->ObjectBorder.fXMin_met,
        pCTAObjCurrGlobal->ObjectBorder.fXMax_met,
        pCTAObjCandGlobal->ObjectBorder.fXMin_met,
        pCTAObjCandGlobal->ObjectBorder.fXMax_met);
    fYDiff = CTA_CTCalculateObjectDistance(
        pCTAObjCurrGlobal->ObjectBorder.fYMin_met,
        pCTAObjCurrGlobal->ObjectBorder.fYMax_met,
        pCTAObjCandGlobal->ObjectBorder.fYMin_met,
        pCTAObjCandGlobal->ObjectBorder.fYMax_met);
    fVxDiff = fABS(pObjCandEMInfo->fVrelX_mps - pCurrObjectEMInput->fVrelX_mps);
    fVyDiff = fABS(pObjCandEMInfo->fVrelY_mps - pCurrObjectEMInput->fVrelY_mps);
    fVyDiffPosBased = fABS(pCTAObjCandGlobal->fRCTAVyPosBased_mps -
                           pCTAObjCurrGlobal->fRCTAVyPosBased_mps);
    // Calculate max differences. Assume that two following vehicles have at
    // least CTA_CT_MULTIOBJ_SAFE_MARGIN security distance Multiply with the
    // assumed security distance S = V*t
    fMaxRangeDiff = SQRT(SQR(pCurrObjectEMInput->fVabsX_mps) +
                         SQR(pCurrObjectEMInput->fVabsY_mps)) *
                    CTA_CT_MULTIOBJ_SAFE_MARGIN;
    // Calculate the limits depending on the heading angle
    fMaxDistXDiff = fABS(COS_(fHeading) * fMaxRangeDiff);
    fMaxDistYDiff = fABS(SIN_(fHeading) * fMaxRangeDiff);
    // Limit the thresholds
    fMaxDistXDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                   CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistXDiff);
    fMaxDistYDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                   CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistYDiff);
    // Check rear object conditions
    if (fXDiff < fMaxDistXDiff && fYDiff < fMaxDistYDiff) {
        float32 fMaxVxDiff;
        float32 fMaxVyDiff;
        // Position criteria is fulfilled
        eObjRearTrackState = CTA_REARTRACK_POS;
        // Set velocity thresholds
        fMaxVxDiff = TUE_CML_BoundedLinInterpol2(
            fRCTARearTrackProb_per, 0.f, 1.f, CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
            CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
        fMaxVyDiff = TUE_CML_BoundedLinInterpol2(
            fRCTARearTrackProb_per, 0.f, 1.f, CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
            CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
        // Check speed criteria
        if (fVxDiff < fMaxVxDiff &&
            (fVyDiff < fMaxVyDiff || fVyDiffPosBased < fMaxVyDiff)) {
            eObjRearTrackState = CTA_REARTRACK_POS_SPEED;
        }
    }
    // return rear track bool
    return eObjRearTrackState;
}
/*****************************************************************************
  Functionname: CTA_CTCalculateObjectDistance */ /*!

                                 @brief Calculate the distance of two objects in
                               X or Y direction, using their dimensions

                                 @description Calculate the distance of two
                               objects in X or Y direction, using their
                               dimensions

                                 @param[in]  fObjCurrMin_met
                                             fObjCurrMax_met
                                                         fObjCandMin_met
                                                         fObjCandMax_met
                                 @param[out]

                                 @return fDistance
                               *****************************************************************************/
float32 CTA_CTCalculateObjectDistance(float32 fObjCurrMin_met,
                                      float32 fObjCurrMax_met,
                                      float32 fObjCandMin_met,
                                      float32 fObjCandMax_met) {
    float32 fDistance = 0.f;

    const float32 fDiffBA = fABS(fObjCandMax_met - fObjCurrMin_met);
    const float32 fDiffAB = fABS(fObjCurrMax_met - fObjCandMin_met);

    // Check the condition for no overlap
    if (fObjCandMax_met < fObjCurrMin_met ||
        fObjCurrMax_met < fObjCandMin_met) {
        fDistance = MIN(fDiffBA, fDiffAB);
    }
    return fDistance;
}
/*****************************************************************************
  Functionname:CTA_CTCheckRearObject                                     */ /*!

      @brief check the rear object

      @description Search for rear objects which are the end of a moving vehicle
    such objects would also warn and keep the warning active too long or
                           trigger new warnings on the same object

      @param[in]  uClassification_nu
                  fRearTrackProb_per
      @param[out] bRearTrack_nu

      @return
    *****************************************************************************/
void CTA_CTCheckRearObject(uint32 uClassification_nu,
                           float32 fRearTrackProb_per,
                           boolean* pbRearTrack_nu) {
    boolean bRearTrack = FALSE;
    // Use hysteresis
    if (*pbRearTrack_nu) {
        if (fRearTrackProb_per >
                CTA_FCTA_REARTRACK_DEACTIVATION_THRESH ||  // 0.7
            uClassification_nu == CTA_EM_GEN_OBJECT_CLASS_MULTIPLE) {
            bRearTrack = TRUE;
        }
    } else {
        if (fRearTrackProb_per > CTA_FCTA_REARTRACK_ACTIVATION_THRESH ||  // 0.9
            uClassification_nu == CTA_EM_GEN_OBJECT_CLASS_MULTIPLE) {
            bRearTrack = TRUE;
        }
    }
    *pbRearTrack_nu = bRearTrack;
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTAToFCTAInputWrapper(const CTAInReq_t* reqPorts,
                           const CTAParam_t* params,
                           CTAGlobal_t* pCTAGlobal,
                           FCTAInReq_t* FCTAreqPorts,
                           FCTAParam_t* FCTAparams) {
    FCTAreqPorts->bFCTAFunctionActive = reqPorts->bFCTAFunctionActive;
    FCTAreqPorts->bFCTAFunctionOutputActive =
        reqPorts->bFCTAFunctionOutputActive;
    FCTAreqPorts->bFCTAFailure = reqPorts->bFCTAFailure;
    FCTAreqPorts->b_FCTA_FCB = reqPorts->b_FCTA_FCB;
    FCTAreqPorts->b_FCTA_RCA = reqPorts->b_FCTA_RCA;
    FCTAreqPorts->fCycleTime_s = reqPorts->fCycleTime_s;
    FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps =
        reqPorts->EgoVehicleInfo.fegoVelocity_mps;
    FCTAreqPorts->FCTAVehicleSig.uGear_nu = reqPorts->EgoVehicleInfo.uGear_nu;
    FCTAreqPorts->FCTARoadInformation.fCurveRadius_met =
        reqPorts->CTARoadInformation.fCurveRadius_met;
    FCTAreqPorts->LastCycleStates.FCTAState =
        pCTAGlobal->LastCycleStates.CTAState;
    FCTAreqPorts->LastCycleStates.bFCTAFunctionActive =
        pCTAGlobal->LastCycleStates.bFCTAFunctionActive;

    FCTAparams->bActive = params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kb_Active_nu;
    FCTAparams->fBreakthroughMargin =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_BreakthroughMargin_met;
    FCTAparams->fMaxHeadingAngle =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg;
    FCTAparams->fMinHeadingAngle =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg;
    FCTAparams->fTargetRangeMax =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met;
    FCTAparams->fTargetRangeMaxL2 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met;
    FCTAparams->fTargetRangeMaxL3 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL3_met;
    FCTAparams->fTTCThreshold =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s;
    FCTAparams->fTTCThresholdL2 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s;
    FCTAparams->fTTCThresholdL3 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL3_s;
    FCTAparams->fTTCThresholdPed_s =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPed_s;
    FCTAparams->fTTCThresholdPedL2_s =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPedL2_s;
    FCTAparams->fTTCThresholdPedL3_s =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPedL3_s;
    FCTAparams->fTTCThresholdMargin =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s;
    FCTAparams->fVEgoMax =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps;
    FCTAparams->fVEgoMin =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps;
    FCTAparams->fVTargetMax =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMax_mps;
    FCTAparams->fVTargetMin =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps;
    FCTAparams->fXMaxBreakthrough =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met;
    FCTAparams->fXMinBreakthrough =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met;
    FCTAparams->fXMaxBreakthroughL2 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met;
    FCTAparams->fXMaxBreakthroughL3 =
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL3_met;

    FCTAreqPorts->CTAGlobleInput.fSensorOffsetToRear_met = 0.0f;
    // FCTAreqPorts->CTAGlobleInput.fSensorOffsetToRear_met =
    //     (pCTAGlobal->fSensorOffsetToRear_met.fLeftFrontPos_met +
    //      pCTAGlobal->fSensorOffsetToRear_met.fRightFrontPos_met) *
    //     0.5f;
    FCTAreqPorts->CTGlobalInput.fMaxLatSensorRange =
        pCTAGlobal->CTGlobals.fMaxLatSensorRange;

    for (uint8 uObj = 0; uObj < FCTA_MAX_NUM_OBJECTS; uObj++) {
        FCTAreqPorts->EMFusionObjListInput[uObj].iFusionID =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].iRawFusionID_nu;
        FCTAreqPorts->EMFusionObjListInput[uObj].bObjStable =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].bObjStable;
        FCTAreqPorts->EMFusionObjListInput[uObj].fRelHeading_rad =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fRelHeading_rad;
        FCTAreqPorts->EMFusionObjListInput[uObj].fArelX_mpss =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fArelX_mpss;
        FCTAreqPorts->EMFusionObjListInput[uObj].fArelY_mpss =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fArelY_mpss;
        FCTAreqPorts->EMFusionObjListInput[uObj].fDistX_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistX_met -
            params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;
        FCTAreqPorts->EMFusionObjListInput[uObj].fFirstDetectX_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fFirstDetectX_met -
            params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;
        if (reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide) {
            FCTAreqPorts->EMFusionObjListInput[uObj].fDistY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met +
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
            FCTAreqPorts->EMFusionObjListInput[uObj].fFirstDetectY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fFirstDetectY_met +
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
        } else {
            FCTAreqPorts->EMFusionObjListInput[uObj].fDistY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met -
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
            FCTAreqPorts->EMFusionObjListInput[uObj].fFirstDetectY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fFirstDetectY_met -
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
        }

        FCTAreqPorts->EMFusionObjListInput[uObj].fMirrorProb_per =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fMirrorProb_per;
        FCTAreqPorts->EMFusionObjListInput[uObj].fProbabilityOfExistence_per =
            reqPorts->CTAEMFusionObjList.aObjects[uObj]
                .fProbabilityOfExistence_per;
        FCTAreqPorts->EMFusionObjListInput[uObj].fRCS =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fRCS;
        FCTAreqPorts->EMFusionObjListInput[uObj].fVabsX_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVabsX_mps;
        FCTAreqPorts->EMFusionObjListInput[uObj].fVabsY_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVabsY_mps;
        FCTAreqPorts->EMFusionObjListInput[uObj].fVrelX_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVrelX_mps;
        FCTAreqPorts->EMFusionObjListInput[uObj].fVrelY_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVrelY_mps;
        FCTAreqPorts->EMFusionObjListInput[uObj].fWidthLeft_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fWidthLeft_met;
        FCTAreqPorts->EMFusionObjListInput[uObj].uiLifeCycles_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].uiLifeCycles_nu;
        FCTAreqPorts->EMFusionObjListInput[uObj].uiMaintenanceState_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].uiMaintenanceState_nu;
        FCTAreqPorts->EMFusionObjListInput[uObj].uiMeasuredTargetFrequency_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj]
                .uiMeasuredTargetFrequency_nu;
        FCTAreqPorts->EMFusionObjListInput[uObj].bRightSensor =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide;

        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj]
            .fAssocProbFiltered_nu =
            pCTAGlobal->CTAObjectList[uObj].fAssocProbFiltered_nu;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fUpdateRate_nu =
            pCTAGlobal->CTAObjectList[uObj].fUpdateRate_nu;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVabs =
            pCTAGlobal->CTAObjectList[uObj].fFCTAVabs;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVxPosBased =
            pCTAGlobal->CTAObjectList[uObj].fFCTAVxPosBased_mps;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVyPosBased =
            pCTAGlobal->CTAObjectList[uObj].fFCTAVyPosBased_mps;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fXMovement_met =
            pCTAGlobal->CTAObjectList[uObj].fFCTAXMovement_met;
        FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fYMovement_met =
            pCTAGlobal->CTAObjectList[uObj].fFCTAYMovement_met;

        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .bRearTrack_nu = pCTAGlobal->CTObjectList[uObj].bFCTARearTrack_nu;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .fDistToCrossingLine_met =
            pCTAGlobal->CTObjectList[uObj].fFCTADistToCrossingLine_met;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .fDistToCrossingLineFiltered_met =
            pCTAGlobal->CTObjectList[uObj].fDistToCrossingLineFiltered_met;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .fTTCFiltered_s = pCTAGlobal->CTObjectList[uObj].fFCTATTCFiltered_s;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj].fTTC_s =
            pCTAGlobal->CTObjectList[uObj].fFCTATTC_s;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .fXBreakthrough_met =
            pCTAGlobal->CTObjectList[uObj].fFCTAXBreakthrough_met;
        FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
            .fXBreakthroughStd_met =
            pCTAGlobal->CTObjectList[uObj].fFCTAXBreakthroughStd_met;
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
void FCTAToCTAOutputWrapper(FCTAOutPro_t* FCTAproPorts,
                            FCTADebug_t* FCTAdebugInfo,
                            CTAGlobal_t* pCTAGlobal,
                            CTADebug_t* debugInfo) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel] =
            FCTAproPorts->bFCTAWarnActive[uWarnLevel];
    }
    pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s = FCTAproPorts->fCriticalTTC_s;
    pCTAGlobal->CTAFCTAOutput.iCriticalObjID_nu =
        FCTAproPorts->iCriticalObjID_nu;
    pCTAGlobal->CTAFCTAOutput.FCTAStateMachine = FCTAproPorts->FCTAStateMachine;
    // memcpy(&debugInfo->FCTADebugInfo, FCTAdebugInfo,
    // sizeof(FCTADebugInfo_t));
}
/*****************************************************************************
   RCTA Input Wrapper
*****************************************************************************/
void CTAToRCTAInputWrapper(const CTAInReq_t* reqPorts,
                           const CTAParam_t* params,
                           CTAGlobal_t* pCTAGlobal,
                           RCTAInReq_t* RCTAreqPorts,
                           RCTAParam_t* RCTAparams) {
    RCTAreqPorts->bRCTAFunctionActive = reqPorts->bRCTAFunctionActive;
    RCTAreqPorts->bRCTAFunctionOutputActive =
        reqPorts->bRCTAFunctionOutputActive;
    RCTAreqPorts->bRCTAFailure = reqPorts->bRCTAFailure;
    RCTAreqPorts->b_RCTA_FCB = reqPorts->b_RCTA_FCB;
    RCTAreqPorts->b_RCTA_RCA = reqPorts->b_RCTA_RCA;
    RCTAreqPorts->fCycleTime_s = reqPorts->fCycleTime_s;
    // RCTAreqPorts->CTAGlobleInput.fSensorOffsetToRear_met =
    RCTAreqPorts->RCTAVehicleSig.StWheelAngle_rad =
        reqPorts->EgoVehicleInfo.fSelfSteering_rad;
    RCTAreqPorts->RCTAVehicleSig.fegoVelocity_mps =
        reqPorts->EgoVehicleInfo.fegoVelocity_mps;
    RCTAreqPorts->RCTAVehicleSig.uGear_nu = reqPorts->EgoVehicleInfo.uGear_nu;
    RCTAparams->RCTAAlgoParam.fMaxHeadingAngle =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg;
    RCTAparams->RCTAAlgoParam.fMinHeadingAngle =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg;
    RCTAparams->RCTAAlgoParam.fTargetRangeMax_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met;
    RCTAparams->RCTAAlgoParam.fTargetRangeMaxL2_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met;
    RCTAparams->RCTAAlgoParam.fTargetRangeMaxL3_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL3_met;
    RCTAparams->RCTAAlgoParam.fTTCThreshold_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdL2_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdL3_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL3_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdMargin_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdPed_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPed_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdPedL2_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPedL2_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdPedL3_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdPedL3_s;
    RCTAparams->RCTAAlgoParam.fTTCThresholdMarginPed_s =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMarginPed_s;
    RCTAparams->RCTAAlgoParam.fVEgoMax =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps;
    RCTAparams->RCTAAlgoParam.fVEgoMin =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps;
    RCTAparams->RCTAAlgoParam.fVTargetMin =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps;
    RCTAparams->RCTAAlgoParam.fXMaxBreakthrough_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met;
    RCTAparams->RCTAAlgoParam.fXMinBreakthrough_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met;
    RCTAparams->RCTAAlgoParam.fXMinBreakthroughL2_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met;
    RCTAparams->RCTAAlgoParam.fXMinBreakthroughL3_met =
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL3_met;
    memcpy(&RCTAparams->RCTAAlgoParam.SteeringAngleCutOff,
           &params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu,
           sizeof(RCTASteeringAngleCutOffParam_t));

    for (uint8 uObj = 0; uObj < RCTA_MAX_NUM_OBJECTS; uObj++) {
        RCTAreqPorts->EMFusionObjListInput[uObj].iFusionID =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].iRawFusionID_nu;
        RCTAreqPorts->EMFusionObjListInput[uObj].bObjStable =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].bObjStable;
        RCTAreqPorts->EMFusionObjListInput[uObj].fDistX_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistX_met +
            (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
             params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met);

        if (reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide) {
            RCTAreqPorts->EMFusionObjListInput[uObj].fDistY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met +
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
            RCTAreqPorts->EMFusionObjListInput[uObj].fFirstDetectY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fFirstDetectY_met +
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;

        } else {
            RCTAreqPorts->EMFusionObjListInput[uObj].fDistY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met -
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
            RCTAreqPorts->EMFusionObjListInput[uObj].fFirstDetectY_met =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fFirstDetectY_met -
                0.5f *
                    params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met;
        }
        RCTAreqPorts->EMFusionObjListInput[uObj].fLengthFront_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fLengthFront_met;
        RCTAreqPorts->EMFusionObjListInput[uObj].fMirrorProb_per =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fMirrorProb_per;
        RCTAreqPorts->EMFusionObjListInput[uObj].fProbabilityOfExistence_per =
            reqPorts->CTAEMFusionObjList.aObjects[uObj]
                .fProbabilityOfExistence_per;
        RCTAreqPorts->EMFusionObjListInput[uObj].fRCS =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fRCS;
        RCTAreqPorts->EMFusionObjListInput[uObj].fRelHeading_rad =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fRelHeading_rad;
        RCTAreqPorts->EMFusionObjListInput[uObj].fRelHeadingStd_rad =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fRelHeadingStd_rad;
        RCTAreqPorts->EMFusionObjListInput[uObj].fVabsX_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVabsX_mps;
        RCTAreqPorts->EMFusionObjListInput[uObj].fVabsY_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVabsY_mps;
        RCTAreqPorts->EMFusionObjListInput[uObj].fVrelY_mps =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fVrelY_mps;
        RCTAreqPorts->EMFusionObjListInput[uObj].fWidthLeft_met =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].fWidthLeft_met;
        RCTAreqPorts->EMFusionObjListInput[uObj].uiLifeCycles_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].uiLifeCycles_nu;
        RCTAreqPorts->EMFusionObjListInput[uObj].uiMaintenanceState_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].uiMaintenanceState_nu;
        RCTAreqPorts->EMFusionObjListInput[uObj].uiMeasuredTargetFrequency_nu =
            reqPorts->CTAEMFusionObjList.aObjects[uObj]
                .uiMeasuredTargetFrequency_nu;
        RCTAreqPorts->EMFusionObjListInput[uObj].bRightSensor =
            reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide;

        RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj]
            .fAssocProbFiltered_nu =
            pCTAGlobal->CTAObjectList[uObj].fAssocProbFiltered_nu;
        RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fUpdateRate_nu =
            pCTAGlobal->CTAObjectList[uObj].fUpdateRate_nu;
        RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fXMovement_met =
            pCTAGlobal->CTAObjectList[uObj].fRCTAXMovement_met;
        RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fYMovement_met =
            pCTAGlobal->CTAObjectList[uObj].fRCTAYMovement_met;

        // CT object
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].bRearTrack_nu =
            pCTAGlobal->CTObjectList[uObj].bRCTARearTrack_nu;
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj]
            .fDistToCrossingLine_met =
            pCTAGlobal->CTObjectList[uObj].fRCTADistToCrossingLine_met;
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].fTTC_s =
            pCTAGlobal->CTObjectList[uObj].fRCTATTC_s;
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].fTTCFiltered_s =
            pCTAGlobal->CTObjectList[uObj].fRCTATTCFiltered_s;
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj]
            .fXBreakthrough_met =
            pCTAGlobal->CTObjectList[uObj].fRCTAXBreakthrough_met;
        RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj]
            .fXBreakthroughStd_met =
            pCTAGlobal->CTObjectList[uObj].fRCTAXBreakthroughStd_met;
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
void RCTAToCTAOutputWrapper(RCTAOutPro_t* RCTAproPorts,
                            RCTADebug_t* RCTAdebugInfo,
                            CTAGlobal_t* pCTAGlobal,
                            CTADebug_t* debugInfo) {
    for (uint8 uWarnLevel = 0u; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel] =
            RCTAproPorts->bRCTAWarnActive[uWarnLevel];
    }
    pCTAGlobal->CTARCTAOutput.bWarningInterrupt =
        RCTAproPorts->bWarningInterrupt;
    pCTAGlobal->CTARCTAOutput.fCriticalObjDistYLastCycle_met =
        RCTAproPorts->fCriticalObjDistYLastCycle_met;
    pCTAGlobal->CTARCTAOutput.fCriticalObjDistY_met =
        RCTAproPorts->fCriticalObjDistY_met;
    pCTAGlobal->CTARCTAOutput.fCriticalTTC_s = RCTAproPorts->fCriticalTTC_s;
    pCTAGlobal->CTARCTAOutput.iCriticalObjIDLastCycle_nu =
        RCTAproPorts->iCriticalObjIDLastCycle_nu;
    pCTAGlobal->CTARCTAOutput.iCriticalObjID_nu =
        RCTAproPorts->iCriticalObjID_nu;
    pCTAGlobal->CTARCTAOutput.uInterruptCycleCount_nu =
        RCTAproPorts->uInterruptCycleCount_nu;
    pCTAGlobal->CTARCTAOutput.RCTAStateMachine = RCTAproPorts->RCTAStateMachine;

    // memcpy(&debugInfo->RCTADebugInfo, RCTAdebugInfo,
    // sizeof(RCTADebugInfo_t)); memcpy(&debugInfo->CTAObjectList,
    // pCTAGlobal->CTAObjectList,
    //        sizeof(CTAObjectInfoGlobal_t) *
    //        CTA_EXTERN_MAX_NUM_FUSION_OBJECTS);
    // memcpy(&debugInfo->CTObjectList, pCTAGlobal->CTObjectList,
    //        sizeof(CTObjectInfoGlobal_t) * CTA_EXTERN_MAX_NUM_FUSION_OBJECTS);
}
/*****************************************************************************
  Functionname:                                     */ /*!

                           @brief

                           @description

                           @param[in]

                           @param[out]

                           @return
                         *****************************************************************************/
void CTAProProcess(const CTAInReq_t* reqPorts,
                   const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal,
                   CTAOutPro_t* proPorts) {
    boolean bFCTAFunctionOutputActive = FALSE;
    boolean bRCTAFunctionOutputActive = FALSE;
    boolean bEgoSpeedCondition = TRUE;  // ego vehicle check is not here
    boolean bTempWarning = FALSE;
    boolean bRCTARightSide, bFCTARightSide;
    // RCTA
    if (reqPorts->bRCTAFunctionActive && reqPorts->bRCTAFunctionOutputActive) {
        bRCTAFunctionOutputActive = TRUE;
    }
    // bEgoSpeedCondition = CTAProcessCheckEgoSpeedRange(
    //     reqPorts->EgoVehicleInfo.fegoVelocity_mps,
    //     params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps,
    //     params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps);
    pCTAGlobal->bRCTAFunctionOutput = CTAProcessSetFuncionOutput(
        params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kb_Active_nu,
        bRCTAFunctionOutputActive, bEgoSpeedCondition);
    for (uint8 uWarnLevel = 0; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        switch (uWarnLevel) {
            case (uint8)CTA_RCTA_WARN_LEVEL_ONE:
                // check whether the warning is active and the whether it's not
                // interrupted
                if (pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel] &&
                    !pCTAGlobal->CTARCTAOutput.bWarningInterrupt) {
                    bTempWarning = TRUE;
                } else {
                    bTempWarning = FALSE;
                }
                bTempWarning = CTAProcessSetWarningOutput(
                    pCTAGlobal->bRCTAFunctionOutput, bTempWarning,
                    pCTAGlobal->CTARCTAOutput.RCTAStateMachine);
                // Set CTAState values for RCTA warning
                if (bTempWarning) {
                    proPorts->fRCTAfTTC_s =
                        pCTAGlobal->CTARCTAOutput.fCriticalTTC_s;
                    proPorts->iRCTACriticalObjID_nu =
                        pCTAGlobal->CTARCTAOutput.iCriticalObjID_nu;
                    for (uint8 i = 0u; i < CTA_EXTERN_MAX_NUM_FUSION_OBJECTS;
                         i++) {
                        if (reqPorts->CTAEMFusionObjList.aObjects[i]
                                .iRawFusionID_nu ==
                            pCTAGlobal->CTARCTAOutput.iCriticalObjID_nu) {
                            bRCTARightSide =
                                reqPorts->CTAEMFusionObjList.aObjects[i]
                                    .bRightSide;
                        }
                    }

                    if (!bRCTARightSide) {
                        proPorts->bRCTAWarningLeftL1 = TRUE;
                    } else {
                        proPorts->bRCTAWarningRightL1 = TRUE;
                    }

                } else {
                    proPorts->bRCTAWarningLeftL1 = FALSE;
                    proPorts->bRCTAWarningRightL1 = FALSE;
                    proPorts->fRCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
                    proPorts->iRCTACriticalObjID_nu = -1;
                }
                break;
            case (uint8)CTA_RCTA_WARN_LEVEL_TWO:
                // set CTAState value for L2 RCTA warning
                bTempWarning = CTAProcessSetWarningOutput(
                    pCTAGlobal->bRCTAFunctionOutput,
                    pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel],
                    pCTAGlobal->CTARCTAOutput.RCTAStateMachine);
                if (bTempWarning) {
                    // proPorts->fRCTAfTTC_s =
                    //     pCTAGlobal->CTARCTAOutput.fCriticalTTC_s;
                    // proPorts->uRCTACriticalObjID_nu =
                    //     pCTAGlobal->CTARCTAOutput.iCriticalObjID_nu;
                    if (!bRCTARightSide) {
                        proPorts->bRCTAWarningLeftL2 = TRUE;
                    } else {
                        proPorts->bRCTAWarningRightL2 = TRUE;
                    }
                } else {
                    proPorts->bRCTAWarningLeftL2 = FALSE;
                    proPorts->bRCTAWarningRightL2 = FALSE;
                    // proPorts->fRCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
                    // proPorts->uRCTACriticalObjID_nu =
                    // TUE_C_UI8_VALUE_INVALID;
                }
                break;
            case (uint8)CTA_RCTA_WARN_LEVEL_THREE:
                // proPorts->bRCTAWarningL3 = CTAProcessSetWarningOutput(
                //     pCTAGlobal->bRCTAFunctionOutput,
                //     pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel]);
                break;
            default:
                break;
        }
    }

    // FCTA
    if (reqPorts->bFCTAFunctionActive && reqPorts->bFCTAFunctionOutputActive) {
        bFCTAFunctionOutputActive = TRUE;
    }
    // bEgoSpeedCondition = CTAProcessCheckEgoSpeedRange(
    //     reqPorts->EgoVehicleInfo.fegoVelocity_mps,
    //     params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps,
    //     params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps);
    pCTAGlobal->bFCTAFunctionOutput = CTAProcessSetFuncionOutput(
        params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kb_Active_nu,
        bFCTAFunctionOutputActive, bEgoSpeedCondition);
    for (uint8 uWarnLevel = 0; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        switch (uWarnLevel) {
            case 0:
                // check whether the warning is active and the whether it's not
                // interrupted
                if (pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel]) {
                    bTempWarning = TRUE;
                } else {
                    bTempWarning = FALSE;
                }
                bTempWarning = CTAProcessSetWarningOutput(
                    pCTAGlobal->bFCTAFunctionOutput, bTempWarning,
                    pCTAGlobal->CTAFCTAOutput.FCTAStateMachine);
                // Set CTAState values for RCTA warning
                if (bTempWarning) {
                    proPorts->fFCTAfTTC_s =
                        pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s;
                    proPorts->iFCTACriticalObjID_nu =
                        pCTAGlobal->CTAFCTAOutput.iCriticalObjID_nu;
                    for (uint8 i = 0u; i < CTA_EXTERN_MAX_NUM_FUSION_OBJECTS;
                         i++) {
                        if (reqPorts->CTAEMFusionObjList.aObjects[i]
                                .iRawFusionID_nu ==
                            proPorts->iFCTACriticalObjID_nu) {
                            bFCTARightSide =
                                reqPorts->CTAEMFusionObjList.aObjects[i]
                                    .bRightSide;
                        }
                    }
                    if (!bFCTARightSide) {
                        proPorts->bFCTAWarningLeftL1 = TRUE;
                    } else {
                        proPorts->bFCTAWarningRightL1 = TRUE;
                    }

                } else {
                    proPorts->bFCTAWarningLeftL1 = FALSE;
                    proPorts->bFCTAWarningRightL1 = FALSE;
                    proPorts->fFCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
                    proPorts->iFCTACriticalObjID_nu = -1;
                }
                break;
            case 1:
                // set CTAState value for L2 RCTA warning
                bTempWarning = CTAProcessSetWarningOutput(
                    pCTAGlobal->bFCTAFunctionOutput,
                    pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel],
                    pCTAGlobal->CTAFCTAOutput.FCTAStateMachine);
                if (bTempWarning) {
                    // proPorts->fFCTAfTTC_s =
                    //     pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s;
                    // proPorts->iFCTACriticalObjID_nu =
                    //     pCTAGlobal->CTAFCTAOutput.iCriticalObjID_nu;
                    if (!bFCTARightSide) {
                        proPorts->bFCTAWarningLeftL2 = TRUE;

                    } else {
                        proPorts->bFCTAWarningRightL2 = TRUE;
                    }
                } else {
                    proPorts->bFCTAWarningLeftL2 = FALSE;
                    proPorts->bFCTAWarningRightL2 = FALSE;
                    // proPorts->fFCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
                    // proPorts->iFCTACriticalObjID_nu = -1;
                }
                break;
            case 2:
                break;
            default:
                break;
        }
    }
    float32 fFCTADistX = 0;
    float32 fFCTADistY = 0;
    float32 fRCTADistX = 0;
    float32 fRCTADistY = 0;
    // CTA
    for (uint8 uObj = 0u; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
        if (!reqPorts->CTAEMFusionObjList.aObjects[uObj]
                 .uiMaintenanceState_nu == CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
            fFCTADistX =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistX_met -
                params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met;
            if (reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide) {
                fFCTADistY =
                    reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met +
                    0.5f * params->CTA_Ks_VehicleParameter_nu
                               .CTA_Kf_VehicleWidth_met;
            } else {
                fFCTADistY =
                    reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met -
                    0.5f * params->CTA_Ks_VehicleParameter_nu
                               .CTA_Kf_VehicleWidth_met;
            }
            pCTAGlobal->CTAObjectList[uObj].fFCTAXLastCycle_met = fFCTADistX;
            pCTAGlobal->CTAObjectList[uObj].fFCTAYLastCycle_met = fFCTADistY;

            fRCTADistX =
                reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistX_met +
                (params->CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met -
                 params->CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met);
            if (reqPorts->CTAEMFusionObjList.aObjects[uObj].bRightSide) {
                fRCTADistY =
                    reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met +
                    0.5f * params->CTA_Ks_VehicleParameter_nu
                               .CTA_Kf_VehicleWidth_met;
            } else {
                fRCTADistY =
                    reqPorts->CTAEMFusionObjList.aObjects[uObj].fDistY_met -
                    0.5f * params->CTA_Ks_VehicleParameter_nu
                               .CTA_Kf_VehicleWidth_met;
            }
            pCTAGlobal->CTAObjectList[uObj].fRCTAXLastCycle_met = fRCTADistX;
            pCTAGlobal->CTAObjectList[uObj].fRCTAYLastCycle_met = fRCTADistY;
        }
    }
    CTAGlobal.LastCycleStates.CTAState = CTAGlobal.eCTAState;
    CTAGlobal.LastCycleStates.bFCTAFunctionActive =
        reqPorts->bFCTAFunctionActive;
    CTAGlobal.LastCycleStates.bRCTAFunctionActive =
        reqPorts->bRCTAFunctionActive;
    if (pCTAGlobal->CTAFCTAOutput.FCTAStateMachine == CTAStateFailure) {
        proPorts->uFCTASystemFaultState_nu = 0x3;
    } else {
        proPorts->uFCTASystemFaultState_nu = 0x0;
    }
    if (pCTAGlobal->CTARCTAOutput.RCTAStateMachine == CTAStateFailure) {
        proPorts->uRCTASystemFaultState_nu = 0x3;
    } else {
        proPorts->uRCTASystemFaultState_nu = 0x0;
    }
}
/*****************************************************************************
  Functionname: CTAProcessCheckEgoSpeedRange */ /*!

                                  @brief Check whether ego vehicle speed is
                                sufficient

                                  @description Check whether ego vehicle speed
                                is sufficient

                                  @param[in]  fEgoSpeed
                                              fMinEgoSpeed
                                                          fMaxEgoSpeed
                                  @param[out]

                                  @return bEgoSpeedCondition
                                *****************************************************************************/
boolean CTAProcessCheckEgoSpeedRange(float32 fEgoSpeed,
                                     float32 fMaxEgoSpeed,
                                     float32 fMinEgoSpeed) {
    boolean bEgoSpeedCondition = FALSE;
    if (fEgoSpeed > fMinEgoSpeed && fEgoSpeed < fMaxEgoSpeed) {
        bEgoSpeedCondition = TRUE;
    }
    return bEgoSpeedCondition;
}
/*****************************************************************************
  Functionname: CTAProcessSetFuncionOutput                                    */ /*!

 @brief Check whether ego vehicle speed is sufficient

 @description Check whether ego vehicle speed is sufficient

 @param[in]  bFunctionEnabled: flag of algorithm parameters function output
             bFunctionActive: flag of function output switch
                         bEgoSpeedCondition: flag of speed enable
 @param[out]

 @return bFunctionOutput: flag of function output
*****************************************************************************/
boolean CTAProcessSetFuncionOutput(boolean bFunctionEnabled,
                                   boolean bFunctionActive,
                                   boolean bEgoSpeedCondition) {
    boolean bFunctionOutput = FALSE;
    // All conditions have to be fulfilled to set the function output
    if (bFunctionActive && bFunctionEnabled && bEgoSpeedCondition) {
        bFunctionOutput = TRUE;
    }
    return bFunctionOutput;
}
/*****************************************************************************
  Functionname: CTAProcessSetWarningOutput                                    */ /*!

 @brief Check whether ego vehicle speed is sufficient

 @description Check whether ego vehicle speed is sufficient

 @param[in]  bFunctionOutputEnabled: flag of function output
             bInternalWarning: RCTA algorithm warning
 @param[out]

 @return bWarningoutput: CTA process warning
*****************************************************************************/
boolean CTAProcessSetWarningOutput(boolean bFunctionOutputEnabled,
                                   boolean bInternalWarning,
                                   CTADebugStateMachine_t CTAStateMachine) {
    boolean bWarningoutput = FALSE;
    if (bFunctionOutputEnabled && bInternalWarning &&
        CTAStateMachine == CTAStateActive) {
        bWarningoutput = TRUE;
    }
    return bWarningoutput;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */