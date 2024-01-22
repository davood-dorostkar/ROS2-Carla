/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2022 by SenseTime Group Limited. All rights reserved.
 */
#include "envm_consts.h"
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! sub-module state */
StateEM_t StateEM;

/*! frame (cycle time, cycle counter, operation mode ...) */
EnvmFrame_t EnvmFrame;

/*! EM time measurement info */
Envm_t_TimeArray EM_a_TimeArray;

/*! Memory for the internal EM object list (access via EnvmData.pPubEmObj) */
ObjectList_t EnvmInternalObj;
SRRIDManegeObjList_t SRRIDManageObjectList;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
CEnvmData_t CEnvmData = {NULL, /*!< Camera lane related input data*/
                         NULL,
                         /*!< Camera lane related output data*/};
// static variables
/* em private glob data*/
EMGlob_t EMGlob;

EnvmData_t EnvmData = {
    /* .pPrivGlob = */
    &EMGlob,
    /* .pFrame = */
    &EnvmFrame,
    /* .pPubFctObj = */
    NULL,
    /* .pEgoDynRaw = */
    NULL,
    /* .pGlobEgoStatic = */
    NULL,
    /* .pNvmIn = */
    NULL,
    /* .pAlgoParameters = */
    NULL,
    /* .p_HWSample */
    NULL,

    /* .pObjListPointers = */
    NULL,
    /* .pPrivObjList = */
    &(EnvmInternalObj),
    /* .pObjPreSeletList = */
    &(EMPreSeletObj),

    /* .EnvmExternalObj = */
    NULL,  // &(EnvmExternalObj),

    /* .pCamObject_t = */
    // NULL,  // &(CamObj_Sense),//memory for sense time camera object list
           // liuyang
           // 20190513

    /* .pExtFusionObjList = */
    NULL,
    /* .pTSRObject = */
    NULL,
    /* .pSRRIDManageObjectList = */
    &(SRRIDManageObjectList),
    /* .pFusionIDManageObjectList = */
    &(FusionIDManageObjectList),
    /* .pEgoDynObjSync = */
    NULL,
    /* .pEgoDynTgtSync = */
    NULL,
    /* .pECAMtCyclEnvmode = */
    NULL,
    /* .pNvmOut = */
    NULL,
    /* .pCEnvmData = */
    &CEnvmData,
    /* .pDemEvents = */
    NULL,
};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
PreSeletObjectList_t EMPreSeletObj;

FusionIDManegeObjList_t FusionIDManageObjectList;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// void ENVM_Exec_Wrapper(const baseReqEMPrtList_t* reqPorts,
//                        baseProEnvmPrtList_t* proPorts,
//                        ExtObjectList_t* pRadarInput,
//                        CamObjectList* pCamerInput,
//                        uint32 uTimestamp) {
//     reqEMPrtList_t req = {0};
//     proEnvmPrtList_t proport = {0};

//     BSW_s_EnvmCtrlData_t bsw = {0};
//     BSW_s_EnvmCtrlData_t* pBSW = &bsw;

//     pBSW->EnvmCycleViolation = reqPorts->pCtrl->EnvmCycleViolation;
//     pBSW->EnvmOpMode = reqPorts->pCtrl->EnvmOpMode;
//     pBSW->HWSample = reqPorts->pCtrl->HWSample;
//     pBSW->RSPCycleViolation = reqPorts->pCtrl->RSPCycleViolation;
//     pBSW->u_Dummy81 = reqPorts->pCtrl->u_Dummy81;
//     pBSW->u_Dummy82 = reqPorts->pCtrl->u_Dummy82;
//     pBSW->u_Dummy83 = reqPorts->pCtrl->u_Dummy83;
//     pBSW->u_VersionNumber = reqPorts->pCtrl->u_VersionNumber;
//     pBSW->sSigHeader.eSigStatus = reqPorts->pCtrl->sSigHeader.eSigStatus;
//     pBSW->sSigHeader.uiCycleCounter =
//         reqPorts->pCtrl->sSigHeader.uiCycleCounter;
//     pBSW->sSigHeader.uiMeasurementCounter =
//         reqPorts->pCtrl->sSigHeader.uiMeasurementCounter;
//     pBSW->sSigHeader.uiTimeStamp = reqPorts->pCtrl->sSigHeader.uiTimeStamp;
//     pBSW->uiTimeStamp_us = uTimestamp;

//     req.pCtrl = &bsw;
//     req.p_CamObjInput = reqPorts->p_CamObjInput;
//     req.pAlgoParameters = reqPorts->pAlgoParameters;

//     req.pCustIn = reqPorts->pCustIn;
//     req.pEnvmCallBackHdlr = reqPorts->pEnvmCallBackHdlr;
//     req.pFctObjectList = reqPorts->pFctObjectList;
//     req.pVehicleDynData = reqPorts->pVehicleDynData;
//     req.pVehicleStatData = reqPorts->pVehicleStatData;
//     req.pVehSig = reqPorts->pVehSig;
//     req.pNvmIn = reqPorts->pNvmIn;

//     req.pExtObjList = pRadarInput;
//     req.pCamObject = pCamerInput;
//     req.pExtSRRObjList = NULL;

//     proport.p_FusionStatusOutput = proPorts->p_FusionStatusOutput;
//     proport.pCameraLaneData = proPorts->pCameraLaneData;
//     proport.pDem = proPorts->pDem;
//     proport.pECAMtCyclEnvmode = proPorts->pECAMtCyclEnvmode;
//     proport.pEnvmGenObjectList = proPorts->pEnvmGenObjectList;
//     proport.pEnvmTechObjectList = proPorts->pEnvmTechObjectList;
//     proport.pNvmOut = proPorts->pNvmOut;
//     proport.pObjSyncEgoDynamic = proPorts->pObjSyncEgoDynamic;
//     proport.pTgtSyncEgoDynamic = proPorts->pTgtSyncEgoDynamic;
//     proport.pSRRGenObjList = NULL;

//     // const reqEMPrtList_t* input = &req;
//     ENVM_Exec(&req, &proport);
// }
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */