/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "lcf_alp.h"
#include "lcf_alp_main.h"
#include "tue_common_libs.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static sALPCalculate_t ALPCalculate = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: ALPExec                                  */ /*!

                                                           @brief: ALP Exec main
                                                         function

                                                           @description:
                                                         Adjacent lane process
                                                         main
                                                         function

                                                           @param[in]:reqPorts
                                                         ALP data input
                                                         pointer
                                                           @param[in]:params
                                                         ALP params input
                                                         pointer
                                                           @param[in]:proPorts
                                                         ALP output
                                                         pointer
                                                           @param[in]:debugInfo
                                                         ALP debug output
                                                         pointer

                                                           @return:void

                                                           @uml:
                                                           @startuml
                                                           start
                                                           :ALP_PreProcess;
                                                           note left:set
                                                         Adjacent lane
                                                         information
                                                         to internal data
                                                           :ALP_MainProcess;
                                                           note left:Adjacent
                                                         lane main process
                                                         function
                                                           :ALP_PostProcess;
                                                           note left:post the
                                                         Adjacent lane
                                                         function to ALP output
                                                         data
                                                           stop
                                                           @enduml
                                                         *****************************************************************************/
void LCF_ALP_Exec(const sALPInReq_st* reqPorts,
                  const sALPParam_st* params,
                  sALPOutput_st* proPorts,
                  sALPDebug_st* debugInfo) {
    // printf("--------------------ALP--------------------\n");

    switch (ALPCalculate.eALPState) {
        case ALP_OK:
            /*Function  has been init*/
            ALP_PreProecss(reqPorts, params, proPorts, debugInfo);
            ALP_MainProcess(reqPorts, params, proPorts, debugInfo);
            ALP_PostProcess(reqPorts, params, proPorts, debugInfo);
            break;
        case ALP_INIT:
            /*First enter reset*/
            LCF_ALP_Init_Reset();
            InitALPDefaultOutPut(proPorts, debugInfo);
            ALPCalculate.eALPState = ALP_OK;
            break;
        default:
            LCF_ALP_Init_Reset();
            InitALPDefaultOutPut(proPorts, debugInfo);
            ALPCalculate.eALPState = ALP_OK;
            break;
    }
}

/*****************************************************************************
  Functionname: LCF_ALP_Init_Reset                                  */ /*!

                          @brief: ALP init set function

                          @description: ALP Init set function

                          @param[in]:void

                          @return:void
                        *****************************************************************************/
void LCF_ALP_Init_Reset(void) {
    sALPCalculate_t* pALPCal = &ALPCalculate;
    sALPLaneInfo_t* pALPLaneCal;
    // Check Lane available flag and map data to ALP internal data
    for (uint8 uiIndex = 0; uiIndex < eLaneNumber; uiIndex++) {
        pALPLaneCal = &pALPCal->aAdjLaneCal[uiIndex];

        // pALPLaneCal->ud_bPrevAdjAvailable_bool      = FALSE;
        // pALPLaneCal->ud_bPrevAdjLaneLatPosValid_met = FALSE;
        // pALPLaneCal->ud_bPrevAdjStepDebounced_bool  = FALSE;
        // pALPLaneCal->ud_bPrevAdjStepDetected_bool   = FALSE;
        // pALPLaneCal->ud_fPrevAdjLanePosX_met        = 0.0f;
        // pALPLaneCal->ud_fPrevAdjPosX_met            = 0.0f;
        // pALPLaneCal->ud_fPrevEgoLanePosX_met        = 0.0f;
        // pALPLaneCal->ud_uiPrevCounterValue_per      = 0u;
        memset(pALPLaneCal, 0u, sizeof(sALPCalculate_t));
    }
    pALPCal->eALPState = ALP_INIT;
}

/*****************************************************************************
  Functionname: ALP_PreProecss                                  */ /*!

                                      @brief: ALP pre process function

                                      @description: set Adjacent lane
                                    information to
                                    internal data

                                      @param[in]:reqPorts    ALP data input
                                    pointer
                                      @param[in]:params      ALP params input
                                    pointer
                                      @param[in]:proPorts    ALP output pointer
                                      @param[in]:debugInfo   ALP debug output
                                    pointer

                                      @return:void
                                    *****************************************************************************/
void ALP_PreProecss(const sALPInReq_st* reqPorts,
                    const sALPParam_st* params,
                    sALPOutput_st* proPorts,
                    sALPDebug_st* debugInfo) {
    // Check Lane available flag and map data to ALP internal data
    for (uint8 uiIndex = 0; uiIndex < eLaneNumber; uiIndex++) {
        // InitAdjLaneInput(&reqPorts->aALPAdjLanes[uiIndex],
        // &ALPCalculate.aAdjLanes[uiIndex]);
        // InitAdjLaneInput
        memcpy(&ALPCalculate.aAdjLanes[uiIndex],
               &reqPorts->aALPAdjLanes[uiIndex], sizeof(sALPAdjLaneInfo_st));
        // InitEgoLaneInput
        memcpy(&ALPCalculate.aEgoLanes[uiIndex],
               &reqPorts->aALPEgoLanes[uiIndex], sizeof(sALPEgoLaneInfo_st));
    }
}

/*****************************************************************************
  Functionname: InitAdjLaneInput                                  */ /*!

                                @brief: set adjacent lane data to internal data

                                @description: set adjacent lane data to internal
                              data

                                @param[in]:pAdjLane        Adjacent lane struct
                              pointer
                                @param[out]:pCalAdjLane    Internal Adjacent
                              lane struct
                              pointer

                                @return:void
                              *****************************************************************************/
void InitAdjLaneInput(sALPAdjLaneInfo_st* pAdjLane,
                      sALPAdjLaneInfo_st* pCalAdjLane) {
    pCalAdjLane->bAvailable_bool = pAdjLane->bAvailable_bool;
    pCalAdjLane->fPosY0_met = pAdjLane->fPosY0_met;
    pCalAdjLane->fHeadingAngle_rad = pAdjLane->fHeadingAngle_rad;
    pCalAdjLane->fCurvature_1pm = pAdjLane->fCurvature_1pm;
    pCalAdjLane->fCurvatureRate_1pm2 = pAdjLane->fCurvatureRate_1pm2;
    pCalAdjLane->fValidLength_met = pAdjLane->fValidLength_met;
    pCalAdjLane->uQuality_nu = pAdjLane->uQuality_nu;
    pCalAdjLane->uMarkerType_nu = pAdjLane->uMarkerType_nu;
}

/*****************************************************************************
  Functionname: InitAdjLaneDefault                                  */ /*!

                          @brief:Init the internal lane data

                          @description: Init the internal adjacent lane data to
                        default
                        value

                          @param[in]:pAdjLane    Internal Adjacent lane struct
                        pointer
                          @param[in]:elaneIndex  Process lane array index

                          @param[out]:pAdjLane    Internal Adjacent lane struct
                        pointer

                          @return:void
                        *****************************************************************************/
void InitAdjLaneDefault(sALPAdjLaneInfo_st* pAdjLane,
                        eALPLaneIndex elaneIndex) {
    switch (elaneIndex) {
        case eLeftLaneIndex:
            pAdjLane->fPosY0_met = ALP_ADJLANE_LATDISTANCE_C0_DEFALUT;
            break;
        case eRightLaneIndex:
            pAdjLane->fPosY0_met = -ALP_ADJLANE_LATDISTANCE_C0_DEFALUT;
            break;
        default:
            pAdjLane->fPosY0_met = ALP_ADJLANE_LATDISTANCE_C0_DEFALUT;
            break;
    }
    pAdjLane->bAvailable_bool = ALP_ADJLANE_AVAILABLE_VALUE_DEFAULT;
    pAdjLane->fHeadingAngle_rad = ALP_ADJLANE_YAWANGLE_C1_DEFAULT;
    pAdjLane->fCurvature_1pm = ALP_ADJLANE_CURVATURE_C2_DEFAULT;
    pAdjLane->fCurvatureRate_1pm2 = ALP_ADJLANE_CURVATURE_RATE_C3_DEFAULT;
    pAdjLane->fValidLength_met = ALP_ADJLANE_VALIDLENGTH_VALUE_DEFAULT;
    pAdjLane->uQuality_nu = ALP_ADJLANE_QUALITY_VALUE_DEFAULT;
    pAdjLane->uMarkerType_nu = ALP_ADJLANE_MARKERTYPE_VALUE_DEFAULT;
}

/*****************************************************************************
  Functionname: InitALPDefaultOutPut                                  */ /*!

                    @brief:Init the ALP module output when the first init

                    @description: Init the ALP module output when the first init

                    @param[in]:proPorts    ALP output pointer
                    @param[in]:debugInfo   ALP debug output pointer

                    @param[out]:proPorts    ALP output pointer
                    @param[out]:debugInfo   ALP debug output pointer

                    @return:void
                  *****************************************************************************/
void InitALPDefaultOutPut(sALPOutput_st* proPorts, sALPDebug_st* debugInfo) {
    proPorts->bLeftAdjLnBridging_bool = FALSE;
    proPorts->bLeftAdjLnStepDeounced_bool = FALSE;
    proPorts->bLeftAdjLnStepOnBridging_bool = FALSE;
    proPorts->bRightAdjLnBridging_bool = FALSE;
    proPorts->bRightAdjLnStepDeounced_bool = FALSE;
    proPorts->bRightAdjLnStepOnBridging_bool = FALSE;

    proPorts->fLeftAdjLnCrvRate_1pm2 = 0.0f;
    proPorts->fLeftAdjLnCurvature_1pm = 0.0f;
    proPorts->fLeftAdjLnDistance_met = 0.0f;
    proPorts->fLeftAdjLnValidLength_met = 0.0f;
    proPorts->fLeftAdjLnYawAngle_rad = 0.0f;
    proPorts->fRightAdjLnCrvRate_1pm2 = 0.0f;
    proPorts->fRightAdjLnCurvature_1pm = 0.0f;
    proPorts->fRightAdjLnDistance_met = 0.0f;
    proPorts->fRightAdjLnValidLength_met = 0.0f;
    proPorts->fRightAdjLnYawAngle_rad = 0.0f;

    proPorts->uiLeftAdjLnAliveCount_per =
        ALP_PROPORT_ALIVECOUNTER_VALUE_DEFALUT;
    proPorts->uiLeftAdjLnInvalidQu_btf = ALP_PROPORT_QUALITY_BITFIED_DEFAULT;
    proPorts->uiRightAdjLnAliveCount_per =
        ALP_PROPORT_ALIVECOUNTER_VALUE_DEFALUT;
    proPorts->uiRightAdjLnInvalidQu_btf = ALP_PROPORT_QUALITY_BITFIED_DEFAULT;
}

/*****************************************************************************
  Functionname: ALP_MainProcess                                  */ /*!

                                   @brief: ALP main process function

                                   @description: Adjacent lane main process
                                 function

                                   @param[in]:reqPorts    ALP data input pointer
                                   @param[in]:params      ALP params input
                                 pointer
                                   @param[in]:proPorts    ALP output pointer
                                   @param[in]:debugInfo   ALP debug output
                                 pointer

                                   @return:void
                                 *****************************************************************************/
void ALP_MainProcess(const sALPInReq_st* reqPorts,
                     const sALPParam_st* params,
                     sALPOutput_st* proPorts,
                     sALPDebug_st* debugInfo) {
    // Adjacent left lane stabilization with ego lane data
    AdjacentLaneStabilization(reqPorts, params, debugInfo, eLeftLaneIndex);
    // Adjacent right lane stabilization with ego lane data
    AdjacentLaneStabilization(reqPorts, params, debugInfo, eRightLaneIndex);
}

/*****************************************************************************
  Functionname: ALP_PostProcess                                  */ /*!

                                   @brief: ALP post process function

                                   @description: post the Adjacent lane function
                                 to ALP
                                 output data

                                   @param[in]:reqPorts    ALP data input pointer
                                   @param[in]:params      ALP params input
                                 pointer
                                   @param[in]:proPorts    ALP output pointer
                                   @param[in]:debugInfo   ALP debug output
                                 pointer

                                   @return:void
                                 *****************************************************************************/
void ALP_PostProcess(const sALPInReq_st* reqPorts,
                     const sALPParam_st* params,
                     sALPOutput_st* proPorts,
                     sALPDebug_st* debugInfo) {
    const sALPLaneInfo_t* pLaneInfo;
    const sALPAdjLaneInfo_st* pAdjLaneInput;

    // post left side adjacent lane information
    pLaneInfo = &ALPCalculate.aAdjLaneCal[eLeftLaneIndex];
    pAdjLaneInput = &ALPCalculate.aAdjLanes[eLeftLaneIndex];
    proPorts->uiLeftAdjLnInvalidQu_btf = ALP_ADJLANE_QUALITY_BITFIED_DEFAULT;

    // A && B ->means if A == TRUE then will run B condition else will not run B
    // condition,Short circuit principle of && symbol
    !pAdjLaneInput->bAvailable_bool && (proPorts->uiLeftAdjLnInvalidQu_btf |=
                                        ALP_ADJLANE_AVAILABLE_BITFIED_MASK);
    pLaneInfo->bStepDebounced && (proPorts->uiLeftAdjLnInvalidQu_btf |=
                                  ALP_ADJLANE_STEPONBRIDGING_BITFIED_MASK);
    pLaneInfo->bAdjLaneBridging && (proPorts->uiLeftAdjLnInvalidQu_btf |=
                                    ALP_ADJLANE_BRIDGING_BITFIED_MASK);
    pLaneInfo->bPrevStepDtctOnBridging_bool &&
        (proPorts->uiLeftAdjLnInvalidQu_btf |=
         ALP_ADJLANE_STEPONBRIDGING_BITFIED_MASK);

    proPorts->bLeftAdjLnBridging_bool = pLaneInfo->bAdjLaneBridging;
    proPorts->bLeftAdjLnStepDeounced_bool = pLaneInfo->bStepDebounced;
    proPorts->bLeftAdjLnStepOnBridging_bool =
        pLaneInfo->bPrevStepDtctOnBridging_bool;
    proPorts->fLeftAdjLnCrvRate_1pm2 = pAdjLaneInput->fCurvatureRate_1pm2;
    proPorts->fLeftAdjLnCurvature_1pm = pAdjLaneInput->fCurvature_1pm;
    proPorts->fLeftAdjLnDistance_met = pAdjLaneInput->fPosY0_met;
    proPorts->fLeftAdjLnValidLength_met = pAdjLaneInput->fValidLength_met;
    proPorts->fLeftAdjLnYawAngle_rad = pAdjLaneInput->fHeadingAngle_rad;
    proPorts->uiLeftAdjLnAliveCount_per = pLaneInfo->uiCounterValue;

    // post right side adjacent lane information
    pLaneInfo = &ALPCalculate.aAdjLaneCal[eRightLaneIndex];
    pAdjLaneInput = &ALPCalculate.aAdjLanes[eRightLaneIndex];
    proPorts->uiRightAdjLnInvalidQu_btf = ALP_ADJLANE_QUALITY_BITFIED_DEFAULT;

    !pAdjLaneInput->bAvailable_bool && (proPorts->uiRightAdjLnInvalidQu_btf |=
                                        ALP_ADJLANE_AVAILABLE_BITFIED_MASK);
    pLaneInfo->bStepDebounced && (proPorts->uiRightAdjLnInvalidQu_btf |=
                                  ALP_ADJLANE_STEPONBRIDGING_BITFIED_MASK);
    pLaneInfo->bAdjLaneBridging && (proPorts->uiRightAdjLnInvalidQu_btf |=
                                    ALP_ADJLANE_BRIDGING_BITFIED_MASK);
    pLaneInfo->bPrevStepDtctOnBridging_bool &&
        (proPorts->uiRightAdjLnInvalidQu_btf |=
         ALP_ADJLANE_STEPONBRIDGING_BITFIED_MASK);

    proPorts->bRightAdjLnBridging_bool = pLaneInfo->bAdjLaneBridging;
    proPorts->bRightAdjLnStepDeounced_bool = pLaneInfo->bStepDebounced;
    proPorts->bRightAdjLnStepOnBridging_bool =
        pLaneInfo->bPrevStepDtctOnBridging_bool;
    proPorts->fRightAdjLnCrvRate_1pm2 = pAdjLaneInput->fCurvatureRate_1pm2;
    proPorts->fRightAdjLnCurvature_1pm = pAdjLaneInput->fCurvature_1pm;
    proPorts->fRightAdjLnDistance_met = pAdjLaneInput->fPosY0_met;
    proPorts->fRightAdjLnValidLength_met = pAdjLaneInput->fValidLength_met;
    proPorts->fRightAdjLnYawAngle_rad = pAdjLaneInput->fHeadingAngle_rad;
    proPorts->uiRightAdjLnAliveCount_per = pLaneInfo->uiCounterValue;
}

/*****************************************************************************
  Functionname: AdjacentLaneStabilization                                  */ /*!

     @brief:Adjacent left lane stabilization with ego lane data

     @description: Adjacent left lane stabilization with ego lane data

     @param[in]:reqPorts    ALP data input pointer
     @param[in]:params      ALP params input pointer
     @param[in]:debugInfo   ALP debug output pointer
     @param[in]:elaneIndex  Process lane array index

     @return:void

     @uml
     @startuml
     start
     note left: Adjacent left lane stabilization with ego lane data
     :LaterPositionStepDetection;
     note left:Adjacent lane lateral position step detection
     :AdjacentLaneAliveCounter;
     note left:Lateral position alive counter(existence provability)
     :DetermineLateralPositionAndValidity;
     note left:Determine lateral position and valid value
     :StepDetectionOnBridging;
     note
     Check step detected after bridging and
     determine lateral position validity(include bridging)
     end note
     :LateralPositionFilter;
     note left:Lateral position low pass filter
     :AdjacentLaneStabilizationWithEgoLaneData;
     note left:Adjacent lane stabilization with ego lane data
     :DetermineAdjacentLaneBridgingState;
     note left:Determine adjacent lane bridging state
     :UpdatePreviousData;
     note left:Store current cycle lane calculate value and flag
     stop
     @enduml
   *****************************************************************************/
void AdjacentLaneStabilization(const sALPInReq_st* reqPorts,
                               const sALPParam_st* params,
                               sALPDebug_st* debugInfo,
                               eALPLaneIndex elaneIndex) {
    sALPLaneInfo_t* pCalLaneInfo = &ALPCalculate.aAdjLaneCal[elaneIndex];
    const sALPEgoLaneInfo_st* pEgoLane = &ALPCalculate.aEgoLanes[elaneIndex];
    const sALPAdjLaneInfo_st* pAdjLane = &ALPCalculate.aAdjLanes[elaneIndex];
    sALPAdjLaneInfo_st* pAdjLaneStabilization =
        &ALPCalculate.aAdjLanes[elaneIndex];
    const boolean bLaneChangeDetected = reqPorts->bLaneChangeDetected_nu;
    const float32 fCycleTime_sec = reqPorts->fCycleTime_sec;

    boolean bStepDebounced = FALSE;
    boolean bLateralPositionValid = FALSE;
    float32 fLateralPosition = 0.0f;
    boolean bAdjLaneBridging = FALSE;
    float32 uiCounterValue = 0u;
    boolean bCounterValueValid = FALSE;
    boolean bStepDetectedOnBridging = FALSE;

    // Lateral position step detection
    LaterPositionStepDetection(pEgoLane, pAdjLane, params, pCalLaneInfo,
                               fCycleTime_sec, bLaneChangeDetected,
                               &bStepDebounced);
    // Lateral position alive counter(existence probability)
    AdjacentLaneAliveCounter(pAdjLane, params, pCalLaneInfo, fCycleTime_sec,
                             bLaneChangeDetected, bStepDebounced,
                             &uiCounterValue, &bCounterValueValid);

    // Lateral position and validity determination
    DetermineLateralPositionAndValidity(
        pEgoLane, pAdjLane, pCalLaneInfo, bStepDebounced, bCounterValueValid,
        &fLateralPosition, &bLateralPositionValid);
    StepDetectionOnBridging(pEgoLane, pAdjLane, params, pCalLaneInfo,
                            fLateralPosition, bLaneChangeDetected,
                            &bLateralPositionValid, &bStepDetectedOnBridging);
    LateralPositionFilter(params, pCalLaneInfo, fCycleTime_sec, uiCounterValue,
                          bLaneChangeDetected, &fLateralPosition);

    // Adjacent lane stabilization with ego lane data
    AdjacentLaneStabilizationWithEgoLaneData(pEgoLane, pAdjLaneStabilization,
                                             params, fLateralPosition,
                                             bLateralPositionValid, elaneIndex);
    // Determine adjacent lane bridging state
    DetermineAdjacentLaneBridgingState(pAdjLane, bLateralPositionValid,
                                       &bAdjLaneBridging);

    // Store current cycle lane calculate value and flag
    pCalLaneInfo->bAdjLaneBridging = bAdjLaneBridging;
    pCalLaneInfo->bCounterValueValid = bCounterValueValid;
    pCalLaneInfo->bLateralPositionValid = bLateralPositionValid;
    pCalLaneInfo->bStepDebounced = bStepDebounced;
    pCalLaneInfo->bStepDebouncedOnBridging = bStepDetectedOnBridging;
    pCalLaneInfo->fLateralPosition = fLateralPosition;
    pCalLaneInfo->uiCounterValue = uiCounterValue;
}

/*****************************************************************************
  Functionname: LaterPositionStepDetection                                  */ /*!

  @brief:Lateral position step detection

  @description: Adjacent lane lateral position step detection

  @param[in]:pEgoLane                         ego lane struct data pointer
  @param[in]:pAdjLane                         adjacent lane struct data pointer
  @param[in]:params->fLatPosStepDtcLimit_met  lateral position step threshold
  @param[in]:fCycleTime_sec                   current cycle exec time
  @param[in]:bLaneChangeDetected              lane changed flag

  @param[out]:pbStepDebounced                 whether the lateral position step
detection flag

  @return:void

  @uml
  @startuml
  start
  note left:Adjacent lane lateral position step detection
  :set bAvailableChange;
  note
  Step1,check the adjacent lane available signal change
  end note
  :calculate bLatPosStepDetected;
  note
  Step2,check the adjacent lane lateral distance exceed
  the limit threshold(means the lane quality no enough)
  end note
  :calculate fNotLatPosStepDeteced;
  note
  Step3,Turn on delay bLatPosStepDetected
  end note
  :update bPrevStepDebounced by RS trigger;
  note
  Step4,Debounced processing of
  the bLatPosStepDetected signal
  end note
  stop
  @enduml
*****************************************************************************/
void LaterPositionStepDetection(const sALPEgoLaneInfo_st* pEgoLane,
                                const sALPAdjLaneInfo_st* pAdjLane,
                                const sALPParam_st* params,
                                sALPLaneInfo_t* pCalLaneInfo,
                                const float32 fCycleTime_sec,
                                const boolean bLaneChangeDetected,
                                boolean* pbStepDebounced) {
    boolean bPrevAvailable = pCalLaneInfo->ud_bPrevAdjAvailable_bool;
    float32 fPrevPosY = pCalLaneInfo->ud_fPrevAdjPosX_met;
    boolean bPrevStepDeteced = pCalLaneInfo->ud_bPrevAdjStepDetected_bool;
    boolean bPrevStepDebounced = pCalLaneInfo->ud_bPrevAdjStepDebounced_bool;

    boolean bAvailableChange;
    boolean bStepSuppressionNotAcitve;
    boolean bLatPosStepDetected;
    boolean bRSResetParamter;
    float32 fPosYDelta;

    // Step1,check the adjacent lane available signal change
    if (pAdjLane->bAvailable_bool != bPrevAvailable) {
        bAvailableChange = TRUE;
    } else {
        bAvailableChange = FALSE;
    }
    pCalLaneInfo->ud_bPrevAdjAvailable_bool = pAdjLane->bAvailable_bool;

    // if the available signal change ,suppression the lateral distance step
    // detection
    bStepSuppressionNotAcitve = !(bAvailableChange || bLaneChangeDetected);

    // Step2,check the adjacent lane lateral distance exceed the limit
    // threshold(means the lane quality no enough)
    fPosYDelta = fABS(pAdjLane->fPosY0_met - fPrevPosY);
    bLatPosStepDetected = (fPosYDelta > params->fLatPosStepDtcLimit_met) &&
                          (bStepSuppressionNotAcitve);
    pCalLaneInfo->ud_fPrevAdjPosX_met = pAdjLane->fPosY0_met;

    // Step3,Turn on delay bLatPosStepDetected
    boolean fNotLatPosStepDeteced;
    fNotLatPosStepDeteced = TUE_CML_TurnOnDelay_M(
        !bLatPosStepDetected, params->fDebounceTLatPosStep_sec, fCycleTime_sec,
        &pCalLaneInfo->fStepTimer, bPrevStepDeteced);
    pCalLaneInfo->ud_bPrevAdjStepDetected_bool = fNotLatPosStepDeteced;

    // Step4,Debounced processing of the bLatPosStepDetected signal
    bRSResetParamter = (fNotLatPosStepDeteced) || (bLaneChangeDetected);
    TUE_CML_RSFlipFlop(bLatPosStepDetected, bRSResetParamter,
                       &bPrevStepDebounced);

    pCalLaneInfo->ud_bPrevAdjStepDebounced_bool = bPrevStepDebounced;

    // set step debounced flag output
    *pbStepDebounced = bPrevStepDebounced;
}

/*****************************************************************************
  Functionname: AdjacentLaneAliveCounter                                  */ /*!

        @brief:Lateral position alive counter

        @description: Lateral position alive counter(existence provability)

        @param[in]:pEgoLane                         ego lane struct data pointer
        @param[in]:params->fAliveCounterTime_sec    todo
        @param[in]:fCycleTime_sec                   current cycle exec time
        @param[in]:params->uiMinAliveCounter_perc   alive counter minimum valid
        threshold
        @param[in]:pCalLaneInfo                     lane data calculate result
        @param[in]:bLaneChangeDetected              lane change detected flag
        @param[in]:bStepDebounced                   whether the lateral position
        step detection flag

        @param[out]:puiCounterValue                 adjacent lane alive counter
        @param[out]:pbCounterValueValid             adjacent lane alive counter
        available flag

        @return:void

        @uml
        @startuml
        start
        note left:Lateral position alive counter(existence provability)
        if(Reset counter condition) then (yes)
        :Reset Alive Counter;
        note
        Reset counter if
        - a lane change has been detected
        - a lateral position step has been detected
        - in the previous cycle a step was
          detected while transition
        -back from bridging
        end note
        else (no)
        :Update Alive Counter;
        note
        Calculate value for
        increase/decrease in each cycle
        then Decrease or increase counter
        end note
        endif
        :Check Counter Valid;
        note
        Step2,Check whether the
        AliveCounter exceed the threshold
        end note
        stop
        @enduml
        *****************************************************************************/
void AdjacentLaneAliveCounter(const sALPAdjLaneInfo_st* pAdjLane,
                              const sALPParam_st* params,
                              sALPLaneInfo_t* pCalLaneInfo,
                              const float32 fCycleTime_sec,
                              boolean bLaneChangeDetected,
                              boolean bStepDebounced,
                              float32* puiCounterValue,
                              boolean* pbCounterValueValid) {
    const boolean bPrevStepDtctOnBridging =
        pCalLaneInfo->bPrevStepDtctOnBridging_bool;
    const float32 uiPrevLaneAliveCounter =
        pCalLaneInfo->uiPrevCounterValue_perc;
    float32 uiAliveCounter;
    // Step1,Calculate alive counter for adjacent lane current cycle
    if (bStepDebounced || bPrevStepDtctOnBridging || bLaneChangeDetected) {
        // Reset counter if
        //- a lane change has been detected
        //- a lateral position step has been detected
        //- in the previous cycle a step was detected while transition
        // back from bridging
        uiAliveCounter = 0;
    } else {
        // Calculate value for increase/decrease in each cycle
        float32 fCounterStep =
            100.0f /
            SafeDiv((params->fAliveCounterTime_sec / SafeDiv(fCycleTime_sec)));

        // Decrease or increase counter
        if (pAdjLane->bAvailable_bool == FALSE) {
            fCounterStep *= -1;
        }

        uiAliveCounter = TUE_CML_Max(uiPrevLaneAliveCounter + fCounterStep,
                                     ALP_ADJLANE_ALIVECOUNTER_VALUE_MIN);
        uiAliveCounter =
            TUE_CML_MinMax(ALP_ADJLANE_ALIVECOUNTER_VALUE_MIN,
                           ALP_ADJLANE_ALIVECOUNTER_VALUE_MAX, uiAliveCounter);
    }

    // Step2,Check whether the AliveCounter exceed the threshold
    // Defines a threshold at which value the adjacent lane can be bridged
    if (uiAliveCounter >= params->uiMinAliveCounter_perc) {
        *pbCounterValueValid = TRUE;
        *puiCounterValue = uiAliveCounter;
    } else {
        *pbCounterValueValid = FALSE;
        *puiCounterValue = uiAliveCounter;
    }

    // update previous counter value
    pCalLaneInfo->uiPrevCounterValue_perc = uiAliveCounter;
}

/*****************************************************************************
  Functionname: AdjacentLaneStabilizationWithEgoLaneData */ /*!

                   @brief:Adjacent lane stabilization with ego lane data

                   @description: Adjacent lane stabilization with ego lane data

                   @param[in]:pEgoLane                         ego lane struct
                   data pointer
                   @param[in]:params->bAllowBridging_bool      allow use
                   bridging mode flag
                   @param[in]:pCalLaneInfo                     lane data
                   calculate result
                   @param[in]:bStepDebounced                   whether the
                   lateral position step
                   detection flag
                   @param[in]:fLateralPosition                 lateral position
                   filter output
                   @param[in]:bLateralPositionValid            lateral position
                   filter result
                   available flag
                   @param[in]:elaneIndex                       Process lane
                   array index

                   @param[out]:pAdjLane                        adjacent lane
                   internal struct data
                   pointer

                   @return:void

                   @uml
                   @startuml
                   start
                   note left:Adjacent lane stabilization with ego lane data
                   if(bAllowBridging_bool) then (yes)
                   :use bridging result;
                   note
                   use
                   fLateralPosition
                   bLateralPositionValid
                   end note
                   else (no)
                   :use source result;
                   note
                   use
                   pAdjLane->fPosY0_met
                   pAdjLane->bAvailable_bool
                   end note
                   endif
                   :update Adjacent lane data;
                   note
                   Step2,use ego lane and bridging value output
                   or init value output
                   end note
                   stop
                   @enduml
                   *****************************************************************************/
void AdjacentLaneStabilizationWithEgoLaneData(
    const sALPEgoLaneInfo_st* pEgoLane,
    sALPAdjLaneInfo_st* pAdjLane,
    const sALPParam_st* params,
    float32 fLateralPosition,
    boolean bLateralPositionValid,
    eALPLaneIndex elaneIndex) {
    float32 fAdjLaneData;
    boolean bAdjLaneDataValid;

    // Step1,Check whether use bridging lateral position result
    if (params->bAllowBridging_bool) {
        fAdjLaneData = fLateralPosition;
        bAdjLaneDataValid = bLateralPositionValid;
    } else {
        fAdjLaneData = pAdjLane->fPosY0_met;
        bAdjLaneDataValid = pAdjLane->bAvailable_bool;
    }

    // Step2,use ego lane and bridging value output or init value output
    if ((pEgoLane->uEgoQuality_btf == ALP_EGOLANE_VALID_BITFIED_DEFAULT ||
         pEgoLane->uEgoQuality_btf & ALP_EGOLANE_VALID_BITFIED_BRIDGE) &&
        (bAdjLaneDataValid)) {
        pAdjLane->fPosY0_met = fAdjLaneData;
        pAdjLane->fHeadingAngle_rad = pEgoLane->fHeadingAngle_rad;
        pAdjLane->fCurvature_1pm = pEgoLane->fCurvature_1pm;
        pAdjLane->fCurvatureRate_1pm2 = pEgoLane->fCurvatureRate_1pm2;
        pAdjLane->fValidLength_met = pEgoLane->fValidLength_met;
    } else {
        InitAdjLaneDefault(pAdjLane, elaneIndex);
    }
}

/*****************************************************************************
  Functionname: DetermineAdjacentLaneBridgingState */ /*!

                         @brief:Determine adjacent lane bridging state

                         @description: Determine adjacent lane bridging state

                         @param[in]:pAdjLane->bAvailable_bool        adjacent
                         lane available flag
                         @param[in]:bLateralPositionValid            lateral
                         position filter result
                         available flag

                         @param[out]:pbAdjLaneBridging               whether
                         adjacent lane lateral
                         position bridging flag

                         @return:void

                         @uml
                         @startuml
                         start
                         note left:Determine adjacent lane bridging state
                         :Update pbAdjLaneBridging flag;
                         note
                         bridging if
                         adjacent lane not available
                         and
                         bLateralPositionValid
                         ( Adjacent lane invalid and ego lane valid)
                         end note
                         stop
                         @enduml
                         *****************************************************************************/
void DetermineAdjacentLaneBridgingState(const sALPAdjLaneInfo_st* pAdjLane,
                                        boolean bLateralPositionValid,
                                        boolean* pbAdjLaneBridging) {
    // Adjacent lane not available and lateral position valid => Adjacent lane
    // invalid and ego lane valid
    *pbAdjLaneBridging =
        (pAdjLane->bAvailable_bool == FALSE) && bLateralPositionValid;
}

/*****************************************************************************
  Functionname: DetermineLateralPositionAndValidity */ /*!

                        @brief:Determine lateral position and valid value

                        @description:Determine lateral position and valid value

                        @param[in]:pEgoLane                         ego lane
                        struct data pointer
                        @param[in]:pAdjLane                         adjacent
                        lane struct data pointer
                        @param[in]:pCalLaneInfo                     lane data
                        calculate result
                        @param[in]:bStepDebounced                   whether the
                        lateral position step
                        detection flag
                        @param[in]:bCounterValueValid               adjacent
                        lane alive counter
                        available flag

                        @param[out]:pfLateralPosition               lateral
                        position filter output
                        @param[out]:pbLateralPositionValid          lateral
                        position filter result
                        available flag

                        @return:void

                        @uml
                        @startuml
                        start
                        note left:Determine lateral position and valid value
                        :Calculate Lateral Position;
                        note
                        Step1,Calculate current cycle
                        adjacent lane lateral position
                        end note
                        :Check Lateral position valid;
                        note
                        Step2,Check adjacent lane is available
                        or counter valid and ego lane quality enough
                        end note
                        stop
                        @enduml
                        *****************************************************************************/
void DetermineLateralPositionAndValidity(const sALPEgoLaneInfo_st* pEgoLane,
                                         const sALPAdjLaneInfo_st* pAdjLane,
                                         sALPLaneInfo_t* pCalLaneInfo,
                                         boolean bStepDebounced,
                                         boolean bCounterValueValid,
                                         float32* pfLateralPosition,
                                         boolean* pbLateralPositionValid) {
    float32 fPrevEgoLanePosX = pCalLaneInfo->ud_fPrevEgoLanePosX_met;

    const float32 fPrevLatPos = pCalLaneInfo->fPrevLatPos_met;
    float32 fAdjLaneLatPos;
    boolean bAdjLaneLatPosValid;

    // Step1,Calculate current cycle adjacent lane lateral position
    if (pAdjLane->bAvailable_bool) {
        fAdjLaneLatPos = pAdjLane->fPosY0_met;
    } else {
        if (pEgoLane->uEgoQuality_btf == ALP_EGOLANE_VALID_BITFIED_DEFAULT) {
            fAdjLaneLatPos =
                fPrevLatPos + (pEgoLane->fPosY0_met - fPrevEgoLanePosX);
        } else {
            fAdjLaneLatPos = fPrevLatPos;
        }
    }
    pCalLaneInfo->ud_fPrevEgoLanePosX_met = pEgoLane->fPosY0_met;
    // Step2,Check adjacent lane is available or counter valid and ego lane
    // quality enough
    boolean bCheckLaneState =
        (pAdjLane->bAvailable_bool) ||
        (bCounterValueValid &&
         pEgoLane->uEgoQuality_btf == ALP_EGOLANE_VALID_BITFIED_DEFAULT);

    if ((bStepDebounced == FALSE) && bCheckLaneState) {
        bAdjLaneLatPosValid = TRUE;
    } else {
        bAdjLaneLatPosValid = FALSE;
    }

    // set Lateral position valid flag output
    *pfLateralPosition = fAdjLaneLatPos;
    // set Lateral position valid flag output
    *pbLateralPositionValid = bAdjLaneLatPosValid;
}

/*****************************************************************************
  Functionname: StepDetectionOnBridging                                  */ /*!

           @brief:Check step detected after bridging

           @description:Check step detected after bridging and determine lateral
         position
                        validity(include bridging)

           @param[in]:pEgoLane                         ego lane struct data
         pointer
           @param[in]:pAdjLane                         adjacent lane struct data
         pointer
           @param[in]:params->fLatPosStepDtcLimit_met  lateral position step
         threshold
           @param[in]:pCalLaneInfo                     lane data calculate
         result
           @param[in]:fAdjLaneLatPos                   adjacent lane lateral
         position

           @param[out]:pbLateralPositionValid          lateral position filter
         result
         available flag(include step detection)
           @param[out]:pbStepDetectedOnBridging        whether the bridging
         lateral
         position step detection flag

           @return:void

           @uml
           @startuml
           start
           note
           Check step detected after bridging and
           determine lateral position validity
           (include bridging)
           end note
           :Calculate fAdjLanePosXDelta;
           note
           Step1,Calculate current lateral
           position delta with previous cycle
           end note
           :set bStepDetectedOnBridging;
           note
           Step2,Check step detection
           whether exceed threshold after bridging
           end note
           stop
           @enduml
         *****************************************************************************/
void StepDetectionOnBridging(const sALPEgoLaneInfo_st* pEgoLane,
                             const sALPAdjLaneInfo_st* pAdjLane,
                             const sALPParam_st* params,
                             sALPLaneInfo_t* pCalLaneInfo,
                             float32 fAdjLaneLatPos,
                             boolean bLaneChangeDetected,
                             boolean* pbAdjLaneLatPosValid,
                             boolean* pbStepDetectedOnBridging) {
    float32 fPrevAdjLanePosX = pCalLaneInfo->ud_fPrevAdjLanePosX_met;
    boolean bPrevAdjLaneLatPosValid =
        pCalLaneInfo->ud_bPrevAdjLaneLatPosValid_met;

    boolean bStepDetectedOnBridging;

    // Step1,Calculate current lateral position delta with previous cycle
    float32 fAdjLanePosXDelta;
    fAdjLanePosXDelta = fABS(fAdjLaneLatPos - fPrevAdjLanePosX);
    pCalLaneInfo->ud_fPrevAdjLanePosX_met =
        fAdjLaneLatPos;  // Update previous lateral distance

    // Step2,Check step detection whether exceed threshold after bridging
    if ((fAdjLanePosXDelta > params->fLatPosStepDtcLimit_met) &&
        (bPrevAdjLaneLatPosValid == *pbAdjLaneLatPosValid) &&
        (!bLaneChangeDetected)) {
        bStepDetectedOnBridging = TRUE;
    } else {
        bStepDetectedOnBridging = FALSE;
    }
    pCalLaneInfo->ud_bPrevAdjLaneLatPosValid_met =
        *pbAdjLaneLatPosValid;  // Update previous lateral distance valid

    *pbAdjLaneLatPosValid =
        (bStepDetectedOnBridging == FALSE) && (*pbAdjLaneLatPosValid);
    *pbStepDetectedOnBridging = bStepDetectedOnBridging;

    pCalLaneInfo->bPrevStepDtctOnBridging_bool = bStepDetectedOnBridging;
}

/*****************************************************************************
  Functionname: LateralPositionFilter                                  */ /*!

                 @brief:Lateral position low pass filter

                 @description:Lateral position low pass filter

                 @param[in]:params->LatPosPT1TConst_sec      todo
                 @param[in]:fCycleTime_sec                   current cycle exec
               time
                 @param[in]:pCalLaneInfo                     lane data calculate
               result
                 @param[in]:uiCounterValue                   adjacent lane alive
               counter
                 @param[in]:bCounterValueValid               adjacent lane alive
               counter
               available flag

                 @param[out]:pfAdjLaneLatPos                 lateral position
               filter
               output

                 @return:void

                 @uml
                 @startuml
                 start
                 note left:Lateral position low pass filter
                 if (Reset condition) then (yes)
                 :Reset filter input;
                 note
                 Reset filter if
                 counter value > 0 && previous counter == 0
                 (raise trigger)
                 end note
                 else (no)
                 :filter fPrevFilterOldIn;
                 note
                 fAdjLaneLatPos Low pass filter
                 end note
                 endif
                 stop
                 @enduml
               *****************************************************************************/
void LateralPositionFilter(const sALPParam_st* params,
                           sALPLaneInfo_t* pCalLaneInfo,
                           const float32 fCycleTime_sec,
                           float32 uiCounterValue,
                           boolean bLaneChangeDetected,
                           float32* pfAdjLaneLatPos) {
    float32 fPrevFilterOldIn = pCalLaneInfo->fPrevLatPos_met;
    // Step1,check reset condition
    if (((uiCounterValue > 0) &&
         (pCalLaneInfo->ud_uiPrevCounterValue_per == 0)) ||
        bLaneChangeDetected) {
        // Reset filter input
        fPrevFilterOldIn = *pfAdjLaneLatPos;
    } else {
        // Step2,fAdjLaneLatPos Low pass filter
        float32 fAlpha = params->LatPosPT1TConst_sec;
        TUE_CML_LowPassFilter(&fPrevFilterOldIn, *pfAdjLaneLatPos, fAlpha);
        *pfAdjLaneLatPos = fPrevFilterOldIn;
    }
    //

    pCalLaneInfo->fPrevLatPos_met = fPrevFilterOldIn;
    pCalLaneInfo->ud_uiPrevCounterValue_per = uiCounterValue;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */