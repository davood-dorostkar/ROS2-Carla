/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wen mingshu <wen mingshu@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_lca_main.h"
#include "lbs_lca_calculation.h"
#include "lbs_lca_par.h"

#include "tue_common_libs.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// define global variables
static LCACalculate_st LCACalculate;
LCAInReq_st LCAReqPorts;
LCAOutPro_st LCAProPorts;
LCAParam_st LCAParams;
LCADebug_t LCADebugInfo;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/
LCACalculate_st* pGetLCACalculatePointer() { return &LCACalculate; }

LCARunState_t* pGetLCACalculatePointer_LCARunState() {
    return &(LCACalculate.LCARunState);
}

LCAGlobals_st* pGetLCACalculatePointer_LCAGlobals() {
    return &(LCACalculate.LCAGlobals);
}

LCAObjInfo_t* pGetLCACalculatePointer_LCAObjInfo(uint8 uObj) {
    // Check Object index range //return id
    // LCA_ASSERT(/*(uObj >= 0) &&*/ (uObj < LBS_INPUT_OBJECT_NUMBER));

    return &(LCACalculate.LCAObjInfoList[uObj]);
}

const LCAGenObject_st* pGetLCAGenObject(uint8 uObj,
                                        const LCAGenObjList_st* pGenObjList) {
    // Check Object index range
    // LCA_ASSERT(/*(uObj >= 0) &&*/ (uObj < LBS_INPUT_OBJECT_NUMBER));

    return &(pGenObjList->aObject[uObj]);
}
// LCALBSObjInfo_t* pGetLCAGenObject_LBSObj(uint8 uObj, LCAGenObjList_st*
// pGenObjList)
// {
// //Check Object index range
// LCA_ASSERT((uObj >= 0) && (uObj < LBS_INPUT_OBJECT_NUMBER));

// return &(pGenObjList->aObject[uObj].LBSObjInfo);
// }

// LCASIObjInfo_t* pGetLCAGenObject_SIObj(uint8 uObj, LCAGenObjList_st*
// pGenObjList)
// {
// //Check Object index range
// LCA_ASSERT((uObj >= 0) && (uObj < LBS_INPUT_OBJECT_NUMBER));

// return &(pGenObjList->aObject[uObj].SIObjInfo);
// }

const LCAGenObjInfo_t* pGetLCAGenObject_GenObj(
    uint8 uObj, const LCAGenObjList_st* pGenObjList) {
    // Check Object index range
    // LCA_ASSERT(/*(uObj >= 0) &&*/ (uObj < LBS_INPUT_OBJECT_NUMBER));

    return &(pGenObjList->aObject[uObj].GenObjInfo);
}

boolean bGetLCAGenObject_GenObjIsDeleted(uint8 uObj,
                                         const LCAGenObjList_st* pGenObjList) {
    // Check Object index range
    // LCA_ASSERT(/*(uObj >= 0) &&*/ (uObj < LBS_INPUT_OBJECT_NUMBER));

    if (pGenObjList->aObject[uObj].GenObjInfo.uiMaintenanceState_nu ==
        LCA_EM_GEN_OBJECT_MT_STATE_DELETED) {
        return TRUE;
    } else {
        return FALSE;
    }
}

LCAStateMachine_t* pGetLCACalculatePointer_LCAstatemachine() {
    return &(LCACalculate.LCAStateMachine);
}

LCAStatusCondition_t* pGetLCACalculatePointer_LCAStatusCondition() {
    return &(LCACalculate.LCAStatusCondition);
}
/*****************************************************************************
  Functionname: LBS_LCA_Exec                                  */ /*!

                @brief:LCA main function to run

                @description:Main module for the LBS LCA functionality

                @param[in]:reqPorts   LCA input
                                       params     LCA parameter input
                                       proPorts   LCA output
                                       debugInfo  LCA debug information
                @return:void
              *****************************************************************************/
void LBS_LCA_Exec(const LCAInReq_st* reqPorts,
                  const LCAParam_st* params,
                  LCAOutPro_st* proPorts,
                  LCADebug_t* debugInfo) {
    boolean bLCAFunctionActive = reqPorts->LCASystemParam.bLCAFunctionActive;
    boolean bLCAFunctionActiveLast =
        pGetLCACalculatePointer_LCARunState()->bLCAFunctionActionLastCycle;
    LCAState_t eLCAState = pGetLCACalculatePointer_LCARunState()->eLCAState;

    switch (eLCAState) {
        case LCA_OK:
            /*Function switch enable and function has been init*/
            if (bLCAFunctionActive == TRUE) {
                LCA_PreProcess(reqPorts, params, proPorts, debugInfo);
                LCA_MainProcess(reqPorts, params, proPorts, debugInfo);
                LCA_PostProcess(proPorts, debugInfo);
            } else {
                /*Function exit reset*/
                if (bLCAFunctionActiveLast == TRUE) {
                    LBS_LCA_Init_Reset();
                }
            }
            break;
        case LCA_INIT:
            /*First enter reset*/
            LBS_LCA_Init_Reset(); /*Global static init*/
            pGetLCACalculatePointer_LCARunState()->eLCAState = LCA_OK;
            break;
        default:
            LBS_LCA_Init_Reset();
            pGetLCACalculatePointer_LCARunState()->eLCAState = LCA_OK;
            break;
    }
}

/*****************************************************************************
  Functionname: LCA_PreProecss                                  */ /*!

              @brief:LCA Pre-processing function

              @description:Preparation of LCA function-related calculation
            parameters and thresholds

              @param[in]:reqPorts   LCA input
                                     params     LCA parameter input
                                     proPorts   LCA output
                                     debugInfo  LCA debug information
              @return:void
            *****************************************************************************/
void LCA_PreProcess(const LCAInReq_st* reqPorts,
                    const LCAParam_st* params,
                    LCAOutPro_st* proPorts,
                    LCADebug_t* debugInfo) {
    const LCALBSInputInfo_st* pLBSInfo = &reqPorts->LBSInputInfo;
    const LCARoad_t* pRoad = &reqPorts->Road;
    const LCAVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
    const LCAGenObjList_st* pGenObjList = &reqPorts->GenObjList;
    LCACalculate_st* pLCACal = pGetLCACalculatePointer();
    LCAGlobals_st* pLCAGlobals = pGetLCACalculatePointer_LCAGlobals();
    const LCALBSObjInfo_Array* pLBSObj = &reqPorts->LBSInputInfo.LBSObjInfoList;
    const LCASIObjInfo_Array* pSIObjList =
        &reqPorts->LBSInputInfo.SIObjInfoList;

    /*Store and reset global parameter*/
    LCASetGlobals(pLCAGlobals);

    /*Set LCA and parameters based on the input parameter*/
    LCASetParamter(params, pLCAGlobals);

    /*Set LCA TTC parameters*/
    LCASetTTCParamter(params, pLBSInfo, pLCAGlobals);

    /*Calculate the range limitation that shall be used for LCA*/
    LCACalculateRangeLimitation(pRoad, pEgoInfo, pLCAGlobals);

    /*Check if a stable object is behind us that might serve
    as a mirror for targets(large traffic signs) for targets
    in front of us*/
    LCACheckStableMirringObject(pGenObjList, pLBSObj, pSIObjList, pLBSInfo,
                                pRoad, pEgoInfo, pLCACal);

    /*Check if the current road conditions based on lane matrix
    and guardrail information provided by EM allow a target to
    overtake us on the adjacent lane*/
    LCACheckLCAPathBlocked(pRoad, pEgoInfo, params->fVehicleWidth_met,
                           &pLCACal->LCAGlobals);
}

/*****************************************************************************
  Functionname: LCA_MainProcess                                  */ /*!

             @brief:LCA main calculate function

             @description:Main calculation functions of the LCA module

             @param[in]:reqPorts   LCA input
                                    params     LCA parameter input
                                    proPorts   LCA output
                                    debugInfo  LCA debug information
             @return:void
           *****************************************************************************/
void LCA_MainProcess(const LCAInReq_st* reqPorts,
                     const LCAParam_st* params,
                     LCAOutPro_st* proPorts,
                     LCADebug_t* debugInfo) {
    const LCAGenObjList_st* pGenObjList = &reqPorts->GenObjList;
    const LCARoad_t* pRoad = &reqPorts->Road;
    const LCAVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
    const LCABSDObjInfo_t* pBSDInfoInput = NULL;
    const LCAGenObject_st* pObjInfo = NULL;
    const LCAGenObjInfo_t* pGenObj = NULL;
    const LCALBSObjInfo_t* pLBSObj = NULL;
    const LCASIObjInfo_t* pSIObj = NULL;
    LCAObjInfo_t* pLCAObj = NULL;
    LCACalculate_st* pLCACalc = &LCACalculate;
    const LCAPreProcessInput_t* pLCAPreProcessInput =
        &reqPorts->LCAPreProcessInput;

    uint8 uObjIndex;
    for (uObjIndex = 0u; uObjIndex < LBS_INPUT_OBJECT_NUMBER; uObjIndex++) {
        if (!bGetLCAGenObject_GenObjIsDeleted(uObjIndex, pGenObjList)) {
            pObjInfo = pGetLCAGenObject(uObjIndex, pGenObjList);
            pGenObj = &(pObjInfo->GenObjInfo);
            pLBSObj = &reqPorts->LBSInputInfo.LBSObjInfoList[uObjIndex];
            pSIObj = &reqPorts->LBSInputInfo.SIObjInfoList[uObjIndex];
            pLCAObj = &pLCACalc->LCAObjInfoList[uObjIndex];
            pBSDInfoInput = &reqPorts->LBSInputInfo.BSDObjInfoList[uObjIndex];

            /*Set Object depend TTC Thresh*/
            LCASetObjDependTTCThreshold(pLCAObj->bLCAWarning, pLCACalc, pGenObj,
                                        &pLCAObj->fTTCThreshold);

            /*Check Object properties from the start of the object*/
            LCACheckObjectStartProperties(
                uObjIndex, pGenObjList, pLBSObj, pLCAObj->fTTCThreshold,
                /*&pLBSObj->bLowTTCAtStart*/ &pLCAObj->bLowTTCAtStart,
                /*&pLBSObj->bCreateAdjStableObj*/ &pLCAObj
                    ->bCreateAdjStableObj);

            /*Check if the object is a mirror object*/
            pLCAObj->bLCAMirrorObject = LCACheckObjectMirrorStatus(
                pGenObj, pLBSObj->fUpdateRate_nu, pLCAObj->bLCAMirrorObject);

            /*Check if the object is a front mirror object*/
            pLCAObj->bLCAMirrorFrontObject = LCACheckObjectFrontMirror(
                uObjIndex, pGenObjList, pLBSObj, pEgoInfo, pLCAObj, pLCACalc,
                reqPorts->LCALBSGlobalInput.fMaxSpeedOverGround_mps);

            /*Check if the object quality is sufficient*/
            pLCAObj->bLCAQuality = LCACheckObjectQuality(
                pGenObj, pLBSObj,
                /*pLBSObj->bLowTTCAtStart*/ pLCAObj->bLowTTCAtStart,
                /*pLBSObj->bCreateAdjStableObj*/ pLCAObj->bCreateAdjStableObj,
                pLCAObj->bLCAQuality, pLCAObj->bLCAWarning);

            /*Check the update status of the object*/
            pLCAObj->bUpdateRecently = LCACheckUpdateStatus(
                pGenObj->uiMeasuredTargetFrequency_nu, pLBSObj->fUpdateRate_nu);

            /*Check if the object is relevant for LCA*/
            pLCAObj->bLCARelevant = LCACheckObjectRelevance(
                pGenObj, pSIObj, pLBSObj->fXMovement_met, pLCAObj->bLCARelevant,
                pLCAObj->bLCAWarning);

            /*Check if the object is within the current LCA range*/
            pLCAObj->bInLCARange = LCACheckObjectInLCARange(
                pGenObj, pLCACalc->LCAGlobals.fLCARange);

            /*Check if the object is behind the guardrail*/
            pLCAObj->fBehindGrdProb_per = LCACheckObjectBehindGRD(
                pGenObj, pRoad, pEgoInfo, pLCAObj->bLCAMirrorFrontObject,
                pLCAObj->fBehindGrdProb_per);

            /*Check if the object fulfills the LCA lane conditions*/
            pLCAObj->bLCALaneConditions = LCACheckLaneConditions(
                pGenObj, pSIObj, pLCACalc->LCAGlobals.bLCAPathBlockedLeft,
                pLCACalc->LCAGlobals.bLCAPathBlockedRight);

            /*Check the object path*/
            pLCAObj->bLCAObjPathInvalid =
                LCACheckObjPath(pGenObj, pRoad, pLCAObj->bLCAObjPathInvalid,
                                params->fVehicleWidth_met);

            /*Check if the object fulfills the LCA warning conditions*/
            pLCAObj->bLCAWarningConditions =
                LCACheckWarningConditions(pGenObj, pLCAObj);

            /*Perform the final warning decision*/
            pLCAObj->bLCAWarning = LCAFinalWarningDecision(
                pGenObj, pLBSObj, pLCAObj, pLCAObj->bInLCARange,
                pLCAObj->bLCAWarningConditions, pLCAObj->bLCAWarning,
                pBSDInfoInput->fBSDZoneXMin_met,
                &pLCACalc->LCAGlobals.LCAWarnInfo);

            /*Store warning information of the closest warning object*/
            LCAStoreWarningObjInfo(uObjIndex, pGenObj, pLCAObj, pLBSObj,
                                   &pLCACalc->LCAGlobals.LCAWarnInfo);
        }
    }
    /*Calculate the low pass filtered rate of front mirror objects*/
    if ((float32)pLCACalc->LCAGlobals.LCAFrontMirror.uNofFMObjects >
        pLCACalc->LCAGlobals.LCAFrontMirror.fFMObjRate) {
        GDB_Math_LowPassFilter(
            &pLCACalc->LCAGlobals.LCAFrontMirror.fFMObjRate,
            (float32)pLCACalc->LCAGlobals.LCAFrontMirror.uNofFMObjects,
            LCA_FMOBJRATE_FILTER_UP);
    } else {
        GDB_Math_LowPassFilter(
            &pLCACalc->LCAGlobals.LCAFrontMirror.fFMObjRate,
            (float32)pLCACalc->LCAGlobals.LCAFrontMirror.uNofFMObjects,
            LCA_FMOBJRATE_FILTER_DOWN);
    }

    // LCA state precondition process
    LBSLCAStateConditionProcess(reqPorts, pLCAPreProcessInput, pEgoInfo);

    // LCA state machine
    LBSLCAStateMachineProcess();

    // can signal output
    LBSLCAHmiOutput(reqPorts, pLCAPreProcessInput, proPorts);
}

/*****************************************************************************
  Functionname: LCA_PostProcess                                  */ /*!

             @brief:Post processing of the LCA warnings

             @description:Post processing of the LCA warnings

             @param[in]:reqPorts   LCA input
                                    params     LCA parameter input
                                    proPorts   LCA output
                                    debugInfo  LCA debug information
             @return:void
           *****************************************************************************/
void LCA_PostProcess(LCAOutPro_st* proPorts, LCADebug_t* debugInfo) {
    proPorts->bLCAWarnActive =
        LCACalculate.LCAGlobals.LCAWarnInfo.bLCAWarnActive;
    proPorts->uLCAWarningID_nu =
        LCACalculate.LCAGlobals.LCAWarnInfo.uLCAWarningID_nu;
    proPorts->fXObjectWarning_met =
        LCACalculate.LCAGlobals.LCAWarnInfo.fXObjectWarning_met;
    proPorts->LCAStateMachineOutput = LCACalculate.LCAStateMachine;

    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        proPorts->LCAObjOutputList[uObj].bLCAMirrorObject =
            LCACalculate.LCAObjInfoList[uObj].bLCAMirrorObject;
        proPorts->LCAObjOutputList[uObj].bLCAMirrorFrontObject =
            LCACalculate.LCAObjInfoList[uObj].bLCAMirrorFrontObject;
        proPorts->LCAObjOutputList[uObj].bLCAWarning =
            LCACalculate.LCAObjInfoList[uObj].bLCAWarning;

        // proPorts->LCAObjOutputList[uObj].bInLCARange =
        //     LCACalculate.LCAObjInfoList[uObj].bInLCARange;
        // proPorts->LCAObjOutputList[uObj].bLCALaneConditions =
        //     LCACalculate.LCAObjInfoList[uObj].bLCALaneConditions;
        // proPorts->LCAObjOutputList[uObj].bLCAMirrorFrontObject =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAMirrorFrontObject;
        // proPorts->LCAObjOutputList[uObj].bLCAMirrorObject =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAMirrorObject;
        // proPorts->LCAObjOutputList[uObj].bLCAObjPathInvalid =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAObjPathInvalid;
        // proPorts->LCAObjOutputList[uObj].bLCAQuality =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAQuality;
        // proPorts->LCAObjOutputList[uObj].bLCARelevant =
        //     LCACalculate.LCAObjInfoList[uObj].bLCARelevant;
        // proPorts->LCAObjOutputList[uObj].bLCAWarning =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAWarning;
        // proPorts->LCAObjOutputList[uObj].bLCAWarningConditions =
        //     LCACalculate.LCAObjInfoList[uObj].bLCAWarningConditions;
        // proPorts->LCAObjOutputList[uObj].bUpdateRecently =
        //     LCACalculate.LCAObjInfoList[uObj].bUpdateRecently;
        // proPorts->LCAObjOutputList[uObj].fBehindGrdProb_per =
        //     LCACalculate.LCAObjInfoList[uObj].fBehindGrdProb_per;

        proPorts->LCAObjOutputList[uObj].bCreateAdjStableObj =
            LCACalculate.LCAObjInfoList[uObj].bCreateAdjStableObj;
        proPorts->LCAObjOutputList[uObj].bLowTTCAtStart =
            LCACalculate.LCAObjInfoList[uObj].bLowTTCAtStart;
    }

    debugInfo->bLCAPathBlockedLeft =
        LCACalculate.LCAGlobals.bLCAPathBlockedLeft;
    debugInfo->bLCAPathBlockedRight =
        LCACalculate.LCAGlobals.bLCAPathBlockedRight;
    debugInfo->fLCARange = LCACalculate.LCAGlobals.fLCARange;
    debugInfo->uCntLCAPathBlockedLeft =
        LCACalculate.LCAGlobals.uCntLCAPathBlockedLeft;
    debugInfo->uCntLCAPathBlockedRight =
        LCACalculate.LCAGlobals.uCntLCAPathBlockedRight;
    // memcpy(&debugInfo->LCAWarnInfo, &LCACalculate.LCAGlobals.LCAWarnInfo,
    //        sizeof(LCAWarnInfo_t));
    // memcpy(&debugInfo->LCAConfig, &LCACalculate.LCAGlobals.LCAConfig,
    //        sizeof(LCAConfig_t));
    // memcpy(&debugInfo->LCAFrontMirror, &LCACalculate.LCAGlobals.LCAFrontMirror,
    //        sizeof(LCAFrontMirror_t));
    // memcpy(&debugInfo->LCAWarnDecideList_Debug, &LCACalculate.LCAWarnDecideList,
    //        sizeof(LCA_Warn_Decide_Debug_Array));
}

/*****************************************************************************
  Functionname: LBS_LCA_Init_Reset                                  */ /*!

          @brief:Initialization all of the LCA internal data

          @description:Initialization all of the LCA internal data,for external
        calls

          @param[in]:reqPorts   LCA input
                                 params     LCA parameter input
                                 proPorts   LCA output
                                 debugInfo  LCA debug information
          @return:void
        *****************************************************************************/
void LBS_LCA_Init_Reset() {
    /*Init Object calculate data*/
    LCA_InitObjects();
    /*Init LCA Global calculation data*/
    LCA_InitGlobal();

    pGetLCACalculatePointer_LCARunState()->eLCAState = LCA_INIT;
}

/*****************************************************************************
  Functionname: LCA_InitObjects                                         */ /*!

      @brief: Initialization of the LCA internal object data

      @description:Initialization of the LCA internal object data

      @param[in]:-
      @return:   void
    *****************************************************************************/
void LCA_InitObjects() {
    uint8 uObj;

    for (uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        LCA_InitObject(uObj);
    }
}

/*****************************************************************************
  Functionname: LCA_InitObject                                  */ /*!

              @brief:Initialization of the internal object data

              @description:Initialization of the internal object data

              @param[in] :uObjNumber -> Array position of the object

              @param[out]:pLCAObj    -> Pointer to LCA related data of the
            object

              @return:void
            *****************************************************************************/
void LCA_InitObject(uint8 uObjNumber) {
    LCAObjInfo_t* pLCAObj = pGetLCACalculatePointer_LCAObjInfo(uObjNumber);
    /*Initialize LCA attributes*/
    pLCAObj->bUpdateRecently = FALSE;
    pLCAObj->bInLCARange = FALSE;
    pLCAObj->bLCAMirrorObject = FALSE;
    pLCAObj->bLCAMirrorFrontObject = FALSE;
    pLCAObj->bLCAObjPathInvalid = FALSE;
    pLCAObj->bLCAQuality = FALSE;
    pLCAObj->bLCALaneConditions = FALSE;
    pLCAObj->bLCARelevant = FALSE;
    pLCAObj->bLCAWarningConditions = FALSE;
    pLCAObj->bLCAWarning = FALSE;
    pLCAObj->uFrontMirrorCnt = 0u;
    pLCAObj->fBehindGrdProb_per = 0.0f;
    pLCAObj->fTTCThreshold = LCA_TTC_THRESH_DEFAULT;
    pLCAObj->bLowTTCAtStart = FALSE;
    pLCAObj->bCreateAdjStableObj = FALSE;
}

/*****************************************************************************
  Functionname: LCA_InitGlobal                                  */ /*!

              @brief:Initialization of the LCA internal global data

              @description:Initialization of the LCA internal global data

              @param[in]:pLCAGlobal   pointer of global structure containing LCA
            data

              @param[out]:pLCAGlobal   pointer of global structure containing
            LCA data

              @return:void
            *****************************************************************************/
void LCA_InitGlobal() {
    LCAGlobals_st* pLCAGlobal = pGetLCACalculatePointer_LCAGlobals();

    /*Initialize LCA global attributes*/
    pLCAGlobal->fLCARange = LCA_RANGE_MAX_DEFAULT;  // 60
    pLCAGlobal->LCAFrontMirror.fFMObjRate = 0.0f;
    pLCAGlobal->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps =
        LCA_FRONT_MIRROR_VX_THRESH_OFFSET;  // 0.5
    pLCAGlobal->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMin_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMax_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMin_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMax_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.fRCSStableObjOwnLane =
        LCA_FRONT_MIRROR_RCS_INVALID;  // -20
    pLCAGlobal->LCAFrontMirror.fRCSStableObjAdjLane =
        LCA_FRONT_MIRROR_RCS_INVALID;
    pLCAGlobal->LCAFrontMirror.uClosetStableObjIDOwnLane =
        TUE_C_UI8_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.uClosetStableObjIDAdjLane =
        TUE_C_UI8_VALUE_INVALID;
    pLCAGlobal->LCAFrontMirror.uNofFMObjects = 0u;

    /*Initialization of warning info variables */
    pLCAGlobal->LCAWarnInfo.fCriticalTTC_s = -TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAWarnInfo.fXObjectWarning_met = -TUE_C_F32_VALUE_INVALID;
    pLCAGlobal->LCAWarnInfo.uLCAWarningID_nu = TUE_C_UI8_VALUE_INVALID;
    pLCAGlobal->LCAWarnInfo.bLCAWarnActive = FALSE;
    pLCAGlobal->LCAWarnInfo.bLCAWarningLastCycle = FALSE;

    /*Initialization of LCA configuration parameters*/
    pLCAGlobal->LCAConfig.fTTCThreshVrelLow_s = LCA_TTC_THRESH_DEFAULT;   // 4
    pLCAGlobal->LCAConfig.fTTCThreshVrelMid_s = LCA_TTC_THRESH_DEFAULT;   // 4
    pLCAGlobal->LCAConfig.fTTCThreshVrelHigh_s = LCA_TTC_THRESH_DEFAULT;  // 4
    pLCAGlobal->LCAConfig.fTTCHysteresis_s = 1.0f;
    pLCAGlobal->LCAConfig.fLCARangeMax_met = LCA_RANGE_MAX_DEFAULT;  // 60
    pLCAGlobal->LCAConfig.fLCACurveRadMax_met =
        LCA_CURVE_RAD_MAX_DEFAULT;  // 250
    pLCAGlobal->LCAConfig.uLCAWarningDurationCfg = 0u;

    pLCAGlobal->uCntLCAPathBlockedLeft = 0u;
    pLCAGlobal->bLCAPathBlockedRight = 0u;
    pLCAGlobal->bLCAPathBlockedLeft = FALSE;
    pLCAGlobal->bLCAPathBlockedRight = FALSE;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */