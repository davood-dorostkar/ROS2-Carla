/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_main.h"
#include "lbs_wrapper.h"
// #include "lbs_bsd_ext.h"
// #include "lbs_si_ext.h"
// #include "lbs_lca_ext.h"
// #include "lbs_ose_ext.h"
#include "lbs_bsd_ext.h"
#include "lbs_lca_ext.h"
#include "lbs_ose_ext.h"
#include "lbs_si_ext.h"
#include "lbs_rcw_ext.h"

#ifdef UBUNTU_SYSTEM
#include "BSW_DataLog_Server.h"
#endif

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
LBSState_t LBSState;
LBSCalculate_st LBSCalculate;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#if LBS_DEVELOPMENT_DEBUG
/*****************************************************************************
  Functionname: LBSInputProcess                                  */ /*!

             @brief:

             @description:

             @param[in]:reqPorts

             @return:void
           *****************************************************************************/
void LBSInputProcess(LBSInReq_st* reqPorts, const LBSParam_st* params) {
    uint16 i;
    // uint8* pObjDynProp;
    // uint8 uObjDynProp;
    LBS_GenObject_st* pGenObj = NULL;

    for (i = 0; i < LBS_INPUT_OBJECT_NUMBER; i++) {
        pGenObj = &(reqPorts->GenObjList.aObject[i]);
        // pObjDynProp =
        //     &(reqPorts->GenObjList.aObject[i].Attributes.eDynamicProperty_nu);

        // switch (*pObjDynProp)
        //{
        // case EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN:
        //	if (pGenObj->Kinemactic.fVrelX_mps > 1.0f &&
        // pGenObj->Attributes.eClassification_nu ==
        // LBS_EM_GEN_OBJECT_CLASS_CAR)
        //	{
        //		*pObjDynProp = EM_GEN_OBJECT_DYN_PROPERTY_MOVING;
        //	}
        //	break;
        // case EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING:
        //	if (   (pGenObj->Attributes.eClassification_nu ==
        // LBS_EM_GEN_OBJECT_CLASS_TRUCK)
        //		|| (pGenObj->Attributes.eClassification_nu ==
        // LBS_EM_GEN_OBJECT_CLASS_CAR))
        //	{
        //		*pObjDynProp = EM_GEN_OBJECT_DYN_PROPERTY_MOVING;
        //	}
        //	break;
        // default:
        //	//nothing to do,keep the original dynamic property
        //	break;
        //}

        /*****************************************************************************
         *Updata Object width and length by object classify                   */ /*!
                      ****************************************************************************/

        switch (pGenObj->Attributes.eClassification_nu) {
            case LBS_EM_GEN_OBJECT_CLASS_CAR:
                pGenObj->Geometry.fWidth_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH;
                pGenObj->Geometry.fLength_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH;

                pGenObj->Geometry.fWidthLeft_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH * 0.5f;
                pGenObj->Geometry.fWidthRight_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH * 0.5f;
                pGenObj->Geometry.fLengthFront_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH * 0.5f;
                pGenObj->Geometry.fLengthRear_met =
                    LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH * 0.5f;

                break;
            case LBS_EM_GEN_OBJECT_CLASS_TRUCK:
                pGenObj->Geometry.fWidth_met =
                    TUE_CML_MinMax(pGenObj->Geometry.fWidth_met,
                                   LBS_EM_GEN_CLASS_TRUCK_MIN_WIDTH,
                                   LBS_EM_GEN_CLASS_TRUCK_MAX_WIDTH);
                pGenObj->Geometry.fLength_met =
                    TUE_CML_MinMax(pGenObj->Geometry.fLength_met,
                                   LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH,
                                   LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH);

                pGenObj->Geometry.fWidthLeft_met =
                    pGenObj->Geometry.fWidth_met * 0.5f;
                pGenObj->Geometry.fWidthRight_met =
                    pGenObj->Geometry.fWidth_met * 0.5f;

                pGenObj->Geometry.fLengthFront_met =
                    pGenObj->Geometry.fLength_met * 0.5f;
                pGenObj->Geometry.fLengthRear_met =
                    pGenObj->Geometry.fLength_met * 0.5f;

                break;
            default:
                // nothing to do
                break;
        }
    }
}
#endif

/*****************************************************************************
  Functionname: LBS_Exec                                  */ /*!

                    @brief:LBS Main Exec function

                    @description:the LBS main logical function

                    @param[in]:reqPorts,params,proPorts,debugInfo

                    @return:void
                  *****************************************************************************/
void LBS_Exec(const LBSInReq_st* reqPorts,
              const LBSParam_st* params,
              LBSOutPro_t* proPorts,
              LBSDebug_t* debugInfo) {
    LBSCalculate_st* pLBSCalc = &LBSCalculate;

    if (LBSState == LBS_OK) {
#if LBS_DEVELOPMENT_DEBUG

        LBSInputProcess(
            reqPorts,
            params); /************************************************** delete
                        the function , implemented in the
                        interface***************************************************/
#endif

        // static uint32 LBS_SI_UseTime = 0;
        // static uint32 LBS_BSD_UseTime = 0;
        // static uint32 LBS_LCA_UseTime = 0;
        // static uint32 LBS_DOW_UseTime = 0;
        // static uint32 LBS_RCW_UseTime = 0;
        // static uint32 LBS_Inter_StepTime = 0;
        // static uint64 LBS_Inter_RecordTime = 0;
        // StbM_TimeStampType timeStamp;
        // StbM_UserDataType userData;
        // static uint64 tempTime = 0;
        // static uint64 tempTime_SI = 0;
        // static uint64 tempTime_LCA = 0;
        // static uint64 tempTime_BSD = 0;
        // static uint64 tempTime_RCW = 0;
        // static uint64 tempTime_DOW = 0;
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_Inter_StepTime = tempTime - LBS_Inter_RecordTime;
        // LBS_Inter_RecordTime = tempTime;

        // SI MAIN FUNCTION
        /**********************************************/
        /* Run the SI function module                */
        /**********************************************/
        // get SI input from LBS layer
        LBSSIInputWrapper(reqPorts, params, &SIReqPorts, &SIParams, pLBSCalc);
        // SI main function
        LBS_SIExec(&SIReqPorts, &SIParams, &SIProPorts, &SIDebugInfo);
        // set LCA output to LBS layer
        LBSSIOutputWrapper(&SIProPorts, &SIDebugInfo, pLBSCalc, debugInfo);
        // LBS process...
        LBSProcess(reqPorts, params, proPorts, debugInfo);
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime_SI = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_SI_UseTime = tempTime_SI - tempTime;
        // LBS_Inter_RecordTime = tempTime_SI;

        // main process...
        // BSD MAIN FUNCTION outer wrapper
        // get input from extern data
        BSDInputWrapper(reqPorts, &BsdReqPorts, params, &BsdParams);
        // BSD MAIN FUNCTION
        LBS_BSDExec(&BsdReqPorts, &BsdParams, &BsdProPorts, &BsdDebugInfo);
        // set output to extern data
        BSDOutputWrapper(proPorts, debugInfo, &BsdProPorts, &BsdDebugInfo,
                         pLBSCalc);
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime_BSD = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_BSD_UseTime = tempTime_BSD - tempTime_SI;
        // LBS_Inter_RecordTime = tempTime_BSD;

        /**********************************************/
        /* Run the LCA function module                */
        /**********************************************/
        // get LCA input from LBS layer
        LBSLCAInputWrapper(reqPorts, params, &LCAReqPorts, &LCAParams,
                           pLBSCalc);
        // LCA main function
        LBS_LCA_Exec(&LCAReqPorts, &LCAParams, &LCAProPorts, &LCADebugInfo);
        // set LCA output to LBS layer
        LBSLCAOutputWrapper(&LCAProPorts, &LCADebugInfo, pLBSCalc, debugInfo);
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime_LCA = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_LCA_UseTime = tempTime_LCA - tempTime_BSD;
        // LBS_Inter_RecordTime = tempTime_LCA;
        /**********************************************/
        /* Run the OSE function module                */
        /**********************************************/
        // get OSE input from LBS layer
        LBSOSEInputWrapper(reqPorts, params, &OSEReqPorts, &OSEParams,
                           pLBSCalc);
        // OSE main function
        LBS_OSE_Exec(&OSEReqPorts, &OSEParams, &OSEProPorts, &OSEDebugInfo);
        // set OSE output to LBS layer
        LBSOSEOutputWrapper(&OSEProPorts, &OSEDebugInfo, pLBSCalc, debugInfo);
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime_DOW = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_DOW_UseTime = tempTime_DOW - tempTime_LCA;
        // LBS_Inter_RecordTime = tempTime_DOW;
        /**********************************************/
        /* Run the RCW function module                */
        /**********************************************/
        // get input from extern data
        RCWInputWrapper(reqPorts, &RcwReqPorts, params, &RcwParams, pLBSCalc);
        // // RCW MAIN FUNCTION
        LBS_RCWExec(&RcwReqPorts, &RcwParams, &RcwProPorts, &RcwDebugInfo);
        // set output to extern data
        RCWOutputWrapper(proPorts, debugInfo, &RcwProPorts, &RcwDebugInfo,
                         pLBSCalc);
        // Hirain_StbM_GetCurrentTime(0, &timeStamp, &userData);
        // tempTime_RCW = ((uint64)timeStamp.seconds * 1000u) +
        //            ((uint64)timeStamp.nanoseconds * 1e-6);
        // LBS_RCW_UseTime = tempTime_RCW - tempTime_DOW;
        // LBS_Inter_RecordTime = tempTime_RCW;
        LBSPostProcess(reqPorts, params, proPorts, debugInfo, pLBSCalc);

    } else {
        LBS_Reset();
        LBSState = LBS_OK;
    }

#ifdef UBUNTU_SYSTEM
    // datalogger logic
    DATALOGInfo_t Record1;
    static LCADatalog_st LCADatalog;
    memcpy(&LCADatalog.LCAReqPorts, &LCAReqPorts, sizeof(LCAInReq_st));
    memcpy(&LCADatalog.LCAParams, &LCAParams, sizeof(LCAParam_st));
    memcpy(&LCADatalog.LCAProPorts, &LCAProPorts, sizeof(LCAOutPro_st));
    memcpy(&LCADatalog.LCADebugInfo, &LCADebugInfo, sizeof(LCADebug_t));

    Record1.StructID = Data_LCADatalog_t_type;
    Record1.Length = sizeof(LCADatalog_st);
    Record1.SocBuf = (uint8*)&LCADatalog;

    BSW_DataLog_FreezeData(Record1);
#endif
}

/*****************************************************************************
  Functionname: LBSProcess                                  */ /*!

                  @brief:The LBS General property calculate function

                  @description:The LBS General property calculate function

                  @param[in]:reqPorts,params,proPorts,debugInfo

                  @return:void
                *****************************************************************************/
void LBSProcess(const LBSInReq_st* reqPorts,
                const LBSParam_st* params,
                LBSOutPro_t* proPorts,
                LBSDebug_t* debugInfo) {
    // LBS Init...
    const EMGenObjList_st* pGenObjList = &reqPorts->GenObjList;

    // Clear object LBS calculate array if the object id changed this cycle
    LBSInitObjCalArray(pGenObjList);

    // Calculated global LBS properties
    LBSCalculateGlobalProperties(reqPorts, params, proPorts, debugInfo);

    // Calculates object properties used in LBS
    LBSCalculateObjectProperties(reqPorts, params, proPorts, debugInfo);
}

/*****************************************************************************
  Functionname: LBSInitObjCalArray                                  */ /*!

          @brief: Clear LBS calculate array

          @description: Clear LBS calculate array if the object id changed
                                        and save the new object for next cycle
        to judge if the object id change

          @param[in]:pGenObjList: the LBS object list input pointer

          @return:void
        *****************************************************************************/
void LBSInitObjCalArray(const EMGenObjList_st* pGenObjList) {
    uint8 uObj;
    const LBS_GenObject_st* pGenObjInfo = NULL;
    LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();

    for (uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        pGenObjInfo = pGetGenObjListPointer_Object(uObj, pGenObjList);

        if (pLBSCalculate->LastObjIDList[uObj] !=
            pGenObjInfo->General.uiID_nu) {
            // object id have been change,init the calculate array
            LBSInitObjInfo(uObj);
            // save current object id
            pLBSCalculate->LastObjIDList[uObj] = pGenObjInfo->General.uiID_nu;
        }
    }
}
/*****************************************************************************
  Functionname: LBSPostProcess                                  */ /*!

              @brief:the LBS post output function

              @description:output the LBS result to proPorts

              @param[in]:reqPorts,params,proPorts,debugInfo

              @return:void
            *****************************************************************************/
void LBSPostProcess(const LBSInReq_st* reqPorts,
                    const LBSParam_st* params,
                    LBSOutPro_t* proPorts,
                    LBSDebug_t* debugInfo,
                    LBSCalculate_st* pLBSCalc) {
    // const LBSSystemParam_t* pLBSParam = &reqPorts->LBSSystemParam;

    LBSProcessBSDWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
    LBSProcessLCAWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
    LBSProcessOSEWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
    LBSProcessRCWWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);

    // memcpy(&debugInfo->LBSCalcuDebug.RoadRelation, &pLBSCalc->RoadRelation,
    //        sizeof(LBS_SRRObjRoadRelation_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LastObjIDList, &pLBSCalc->LastObjIDList,
    //        sizeof(LBS_LastCycleObjID_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSObjHistoryList,
    //        &pLBSCalc->LBSObjHistoryList, sizeof(LBS_ObjHistory_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSRunState, &pLBSCalc->LBSRunState,
    //        sizeof(LBS_RunState_t));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSObjInfoList,
    // &pLBSCalc->LBSObjInfoList,
    //        sizeof(LBS_ObjInfo_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSSICalc, &pLBSCalc->LBSSICalc,
    //        sizeof(LBS_SICalculate_t));
    // memcpy(&debugInfo->LBSCalcuDebug.SIObjInfoList, &pLBSCalc->SIObjInfoList,
    //        sizeof(LBS_SIObjInfo_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSBSDCalc, &pLBSCalc->LBSBSDCalc,
    //        sizeof(LBS_BSDCalculate_t));
    // memcpy(&debugInfo->LBSCalcuDebug.BSDObjInfoList,
    // &pLBSCalc->BSDObjInfoList,
    //        sizeof(LBS_BSDObjInfo_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSLCACalc, &pLBSCalc->LBSLCACalc,
    //        sizeof(LBS_LCACalculate_t));
    // memcpy(&debugInfo->LBSCalcuDebug.LCAObjInfoList,
    // &pLBSCalc->LCAObjInfoList,
    //        sizeof(LBS_LCAObjInfo_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.OSEObjInfoList,
    // &pLBSCalc->OSEObjInfoList,
    //        sizeof(LBS_OSEObjInfo_Array));
    // memcpy(&debugInfo->LBSCalcuDebug.LBSWarnLastCycle,
    //        &pLBSCalc->LBSWarnLastCycle, sizeof(LBS_WarningLastCycle_t));
}

/*****************************************************************************
  Functionname: LBSProcessBSDWarnings                                  */ /*!

       @brief:check if the BSD warning in enable conditions

       @description:check if the BSD warning in enable conditions,ego speed or
     switch

       @param[in]:reqPorts,params,proPorts,debugInfo

       @return:void
     *****************************************************************************/
void LBSProcessBSDWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc) {
    const LBSSystemParam_t* pLBSParam = &reqPorts->LBSSystemParam;
    const LBSBSDWarningParameter_t* pBSDParam =
        &params->LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu;
    const LBSBSDCalculate_t* pBSDCal = &pLBSCalc->LBSBSDCalc;
    const float32 fEgoSpeedX = reqPorts->EgoVehInfo.fegoVelocity_mps;
    LBSFunState_t* pFunState = &proPorts->LBSFunState;
    LBS_Globals_t* pLBSGlobal = &proPorts->LBSGlobals;

    boolean bBSDFunctionOutputActive = FALSE;
    boolean bEgoSpeedCondition = FALSE;

    if ((pLBSParam->bBSDFunctionActive == TRUE) &&
        (pLBSParam->bBSDFunctionOutputActive == TRUE)) {
        // set output active if function and function output both active
        bBSDFunctionOutputActive = TRUE;
    }

    /*****************************************************************************/
    /* BSD */
    /*****************************************************************************/
    // bEgoSpeedCondition = LBSCheckEgoSpeedActThreshold(
    //     fEgoSpeedX, pBSDParam->LBS_Kf_BSDVelMinWarnDisable_mps,
    //     pBSDParam->LBS_Kf_BSDVelMinWarnEnable_mps,
    //     &pLBSGlobal->LastCycleStates.bEgoSpeedConditionBSD);
    bEgoSpeedCondition =
        (fEgoSpeedX >= pBSDParam->LBS_Kf_BSDVelMinWarnEnable_mps);
    pLBSGlobal->bBSDFunctionOutput =
        LBSSetFunctionOutput(bBSDFunctionOutputActive, bEgoSpeedCondition);

    // Check warning flag if function active and output
    if ((pLBSGlobal->bBSDFunctionOutput == TRUE) &&
        (reqPorts->LBSSupportInfo.LBSBSDFailure == FALSE) &&
        (((pBSDCal->bBSDWarnActiveLeft)) || ((pBSDCal->bBSDWarnActiveRight)))) {
        pFunState->bBSDWarning = TRUE;
        pFunState->bBSDWarningLeft = pBSDCal->bBSDWarnActiveLeft;
        pFunState->bBSDWarningRight = pBSDCal->bBSDWarnActiveRight;
    } else {
        pFunState->bBSDWarning = FALSE;
        pFunState->bBSDWarningLeft = FALSE;
        pFunState->bBSDWarningRight = FALSE;
    }
    // printf("bBSDWarning is %d\t, bBSDWarningLeft is %d\t, bBSDWarningRight is
    // %d\n", pFunState->bBSDWarning,
    //         pFunState->bBSDWarningLeft, pFunState->bBSDWarningRight);
}
/*****************************************************************************
  Functionname: LBSProcessLCAWarnings                                  */ /*!

       @brief:check if the LCA warning in enable conditions

       @description:check if the LCA warning in enable conditions,ego speed or
     switch

       @param[in]:reqPorts,params,proPorts,debugInfo

       @return:void
     *****************************************************************************/
void LBSProcessLCAWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc) {
    boolean bEgoSpeedCondition = FALSE;
    boolean bLCAFunctionOutputActive = FALSE;
    uint8 uLCAWaningID = pLBSCalc->LBSLCACalc.uLCAWarningID_nu;
    static boolean bLCABSDLeftLastCycleActive = FALSE;
    static boolean bLCABSDRightLastCycleActive = FALSE;

    if (reqPorts->LBSSystemParam.bLCAFunctionActive &&
        reqPorts->LBSSystemParam.bLCAFunctionOutputActive) {
        // set output active if function and function output both active
        bLCAFunctionOutputActive = TRUE;
    }
    // Check if the ego speed in the range of function activation
    // bEgoSpeedCondition = LBSCheckEgoSpeedActThreshold(
    //     reqPorts->EgoVehInfo.fegoVelocity_mps,
    //     params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVownMinWarnDisable_mps,
    //     params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVoWnMinWarnEnable_mps,
    //     &pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionLCA);

    // Check if all conditions are met to activate function output
    boolean bLCABSDVelOupt =
        (reqPorts->EgoVehInfo.fegoVelocity_mps >=
         params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVoWnMinWarnEnable_mps);
    pLBSCalc->LBS_Globals.bLCAFunctionOutput =
        LBSSetFunctionOutput(bLCAFunctionOutputActive, bLCABSDVelOupt);

    pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle =
        pLBSCalc->LBSLCACalc.bLCAWarnActive;
    // Check warning flag if function active and output
    if (pLBSCalc->LBS_Globals.bLCAFunctionOutput &&
        pLBSCalc->LBSLCACalc.bLCAWarnActive &&
        uLCAWaningID != TUE_C_UI8_VALUE_INVALID &&
        (reqPorts->LBSSupportInfo.LCAFailure == FALSE)) {
        proPorts->LBSFunState.bLCAWarning = TRUE;
        proPorts->LBSFunState.fTTC_s = pLBSCalc->LBSLCACalc.fCriticalTTC_s;
        proPorts->LBSFunState.bLCAWarningLeft =
            pLBSCalc->LBSLCACalc.bLCAWarnActiveLeft;
        proPorts->LBSFunState.bLCAWarningRight =
            pLBSCalc->LBSLCACalc.bLCAWarnActiveRight;
    } else {
        proPorts->LBSFunState.bLCAWarning = FALSE;
        proPorts->LBSFunState.bLCAWarningLeft = FALSE;
        proPorts->LBSFunState.bLCAWarningRight = FALSE;
    }

    // EP40
    proPorts->LBSCANOutputs.uLCABSDOnOffSet =
        reqPorts->LBSSystemParam.bLCAFunctionActive;

    if (((reqPorts->LBSSupportInfo.LCAFailure == TRUE) ||
         (reqPorts->LBSSupportInfo.LBSBSDFailure == TRUE)) &&
        (reqPorts->LBSSystemParam.bLCAFunctionActive == TRUE)) {
        proPorts->LBSCANOutputs.uLCASystemFaultStatus = 2u;
    } else {
        proPorts->LBSCANOutputs.uLCASystemFaultStatus = 0u;
    }

    // keep output left Active until 1min
    boolean bLCABSDLefttActiveFlag = FALSE;
    boolean bLCABSDLefttStarActive = FALSE;
    if (proPorts->LBSFunState.bBSDWarningLeft == TRUE ||
        proPorts->LBSFunState.bLCAWarningLeft == TRUE) {
        bLCABSDLefttActiveFlag = TRUE;
    }

    if (bLCABSDLeftLastCycleActive == FALSE && bLCABSDLefttActiveFlag == TRUE) {
        bLCABSDLefttStarActive = TRUE;
    }
    bLCABSDLeftLastCycleActive = bLCABSDLefttActiveFlag;

    static float32 fLCABSDLeftActiveRemainValue = 0.f;
    boolean bLCABSDLeftActiveKeep = TUE_CML_TimerRetrigger(
        params->LBS_Ks_RCWParameter_nu.fCycletime_s, bLCABSDLefttStarActive,
        1.f, &fLCABSDLeftActiveRemainValue);

    if (bLCABSDLeftActiveKeep || bLCABSDLefttActiveFlag) {
        if (reqPorts->LBSSupportInfo.uTurnLightReqSt == 1u) {
            proPorts->LBSCANOutputs.uLCALeftWarnSt = 2u;
        } else {
            proPorts->LBSCANOutputs.uLCALeftWarnSt = 1u;
        }
    } else {
        proPorts->LBSCANOutputs.uLCALeftWarnSt = 0u;
    }

    // keep output right Active until 1min
    boolean bLCABSDRightActiveFlag = FALSE;
    boolean bLCABSDRighttStarActive = FALSE;
    if (proPorts->LBSFunState.bBSDWarningRight == TRUE ||
        proPorts->LBSFunState.bLCAWarningRight == TRUE) {
        bLCABSDRightActiveFlag = TRUE;
    }

    if ((bLCABSDRightLastCycleActive == FALSE) &&
        (bLCABSDRightActiveFlag == TRUE)) {
        bLCABSDRighttStarActive = TRUE;
    }
    bLCABSDRightLastCycleActive = bLCABSDRightActiveFlag;

    static float32 fLCABSDRightActiveRemainValue = 0.f;
    boolean bLCABSDRightActiveKeep = TUE_CML_TimerRetrigger(
        params->LBS_Ks_RCWParameter_nu.fCycletime_s, bLCABSDRighttStarActive,
        1.f, &fLCABSDRightActiveRemainValue);

    if (bLCABSDRightActiveKeep || bLCABSDRightActiveFlag) {
        if (reqPorts->LBSSupportInfo.uTurnLightReqSt == 2u) {
            proPorts->LBSCANOutputs.uLCARightWarnSt = 2u;
        } else {
            proPorts->LBSCANOutputs.uLCARightWarnSt = 1u;
        }
    } else {
        proPorts->LBSCANOutputs.uLCARightWarnSt = 0u;
    }

    if (proPorts->LBSCANOutputs.uLCALeftWarnSt != 0u) {
        if (proPorts->LBSFunState.bLCAWarningLeft == TRUE) {
            proPorts->LBSCANOutputs.uLCABSDObjIndex = uLCAWaningID;
        } else {
            proPorts->LBSCANOutputs.uLCABSDObjIndex =
                pLBSCalc->LBSBSDCalc.uBSDWarnActiveLeftID;
        }

        if (proPorts->LBSCANOutputs.uLCABSDObjIndex < LBS_INPUT_OBJECT_NUMBER) {
            uint8 uLCABSDObjIndex_t = proPorts->LBSCANOutputs.uLCABSDObjIndex;
            proPorts->LBSCANOutputs.siLCALeftHighlightID_nu =
                reqPorts->GenObjList.aObject[uLCABSDObjIndex_t]
                    .General.iRawFusionID_nu;
        } else {
            proPorts->LBSCANOutputs.siLCALeftHighlightID_nu = -1;
        }
    } else if (proPorts->LBSCANOutputs.uLCARightWarnSt != 0u) {
        if (proPorts->LBSFunState.bLCAWarningRight == TRUE) {
            proPorts->LBSCANOutputs.uLCABSDObjIndex = uLCAWaningID;
        } else {
            proPorts->LBSCANOutputs.uLCABSDObjIndex =
                pLBSCalc->LBSBSDCalc.uBSDWarnActiveRightID;
        }

        if (proPorts->LBSCANOutputs.uLCABSDObjIndex < LBS_INPUT_OBJECT_NUMBER) {
            uint8 uLCABSDObjIndex_t = proPorts->LBSCANOutputs.uLCABSDObjIndex;
            proPorts->LBSCANOutputs.siLCARightHighlightID_nu =
                reqPorts->GenObjList.aObject[uLCABSDObjIndex_t]
                    .General.iRawFusionID_nu;
        } else {
            proPorts->LBSCANOutputs.siLCARightHighlightID_nu = -1;
        }
    } else {
        proPorts->LBSCANOutputs.uLCABSDObjIndex = 255u;
        proPorts->LBSCANOutputs.siLCALeftHighlightID_nu = -1;
        proPorts->LBSCANOutputs.siLCARightHighlightID_nu = -1;
    }
}
/*****************************************************************************
  Functionname: LBSProcessOSEWarnings                                  */ /*!

       @brief: check if the OSE warning in enable conditions

       @description: check if the OSE warning in enable conditions,ego speed or
     switch

       @param[in]: reqPorts,params,proPorts,debugInfo

       @return:void
     *****************************************************************************/
void LBSProcessOSEWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc) {
    boolean bFunctionEnabled = TRUE;
    boolean bOSEFunctionOutputActive = FALSE;
    boolean bWarnThisCycle[OSE_LBS_NUM_OF_WARN_LEVELS];
    uint8 uOSEWaningID = pLBSCalc->LBSOSECalc.uCriticalObjID;
    // minimum activation speed condition is common for all warnings
    // if (reqPorts->EgoVehInfo.fegoVelocity_mps <
    //         params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps &&
    //     reqPorts->EgoVehInfo.fegoVelocity_mps >
    //         params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps) {
    //     bFunctionEnabled = TRUE;
    // } else {
    //     bFunctionEnabled = FALSE;
    // }
    pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionOSE =
        bFunctionEnabled;

    if (reqPorts->LBSSystemParam.bOSEFunctionActive &&
        reqPorts->LBSSystemParam.bOSEFunctionOutputActive) {
        // set output active if function and function output both active
        bOSEFunctionOutputActive = TRUE;
    }
    // Checks OSE activation via BSW parameters(driver), gear shift position and
    // ego speed condition
    bFunctionEnabled =
        LBSSetFunctionOutput(bOSEFunctionOutputActive, bFunctionEnabled);

    for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_LBS_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        pLBSCalc->LBS_Globals.bOSEFunctionOutput[uWarnLevel] = bFunctionEnabled;
        bWarnThisCycle[uWarnLevel] =
            pLBSCalc->LBS_Globals.bOSEFunctionOutput[uWarnLevel] &&
            pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel];

        if (uWarnLevel < 1u && pLBSCalc->LBSOSECalc.bWarningInterrupt) {
            // First level of warning can be interrupted
            bWarnThisCycle[uWarnLevel] = FALSE;
        }  // else: rest of levels can't be interrupted or first level is not
           // interrupted
        bWarnThisCycle[uWarnLevel] = LBSOSEWarnCalcNextState(
            uWarnLevel, bWarnThisCycle[uWarnLevel],
            reqPorts->LBSSystemParam.fCycletime_s, params, pLBSCalc);

        pLBSCalc->LBSWarnLastCycle.bOSEWarningLastCycle[uWarnLevel] =
            pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel];
    }

    proPorts->LBSFunState.bOSEWarning.bPreWarnActive = bWarnThisCycle[0];
    proPorts->LBSFunState.bOSEWarning.bAcuteWarnActive = bWarnThisCycle[1];
    proPorts->LBSFunState.bOSEWarning.bDoorLockingActive = bWarnThisCycle[2];
    // Copy into outputs
    boolean bDOWVelOupt;
    if (reqPorts->EgoVehInfo.fegoVelocity_mps <
            params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps &&
        reqPorts->EgoVehInfo.fegoVelocity_mps >
            params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps) {
        bDOWVelOupt = TRUE;
    } else {
        bDOWVelOupt = FALSE;
    }
    boolean bDOWHMIOpen;
    bDOWHMIOpen = (reqPorts->LBSSystemParam.bOSEFunctionActive ||
                   pLBSCalc->LBSOSECalc.bDOWPowerModeDone);
    if( bDOWHMIOpen && bDOWVelOupt
        && (uOSEWaningID != TUE_C_UI8_VALUE_INVALID)
        && (reqPorts->LBSSupportInfo.DOWFailure == FALSE)
        && (!reqPorts->GenObjList.aObject[uOSEWaningID].bRightSensor)
        /*&& (reqPorts->LBSSupportInfo.bDoor_FL || reqPorts->LBSSupportInfo.bDoor_RL)*/) {
        proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive =
            bWarnThisCycle[0];
        proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive =
            bWarnThisCycle[1];
        proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive =
            bWarnThisCycle[2];
        proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
    } else if (bDOWHMIOpen && bDOWVelOupt
        && (uOSEWaningID != TUE_C_UI8_VALUE_INVALID)
        && (reqPorts->LBSSupportInfo.DOWFailure == FALSE)
        && (reqPorts->GenObjList.aObject[uOSEWaningID].bRightSensor)
        /*&& (reqPorts->LBSSupportInfo.bDoor_FR || reqPorts->LBSSupportInfo.bDoor_RR)*/) {
        proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive =
            bWarnThisCycle[0];
        proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive =
            bWarnThisCycle[1];
        proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive =
            bWarnThisCycle[2];
        proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
    } else {
        proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
        proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
        proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
    }

    if ((proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive == TRUE) &&
        (reqPorts->LBSSupportInfo.bDoor_FL == FALSE) &&
        (reqPorts->LBSSupportInfo.bDoor_RL == FALSE)) {
        proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
    }

    if ((proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive == TRUE) &&
        (reqPorts->LBSSupportInfo.bDoor_FR == FALSE) &&
        (reqPorts->LBSSupportInfo.bDoor_RR == FALSE)) {
        proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
    }

    if (proPorts->LBSFunState.bOSEWarning.bPreWarnActive) {
        proPorts->LBSFunState.fTTC_s = pLBSCalc->LBSOSECalc.fCriticalTTC;
    } else {
        proPorts->LBSFunState.fTTC_s = TUE_C_F32_VALUE_INVALID;
    }

    // EP40 output
    proPorts->LBSCANOutputs.bDOWPowerModeDone =
        pLBSCalc->LBSOSECalc.bDOWPowerModeDone;
    proPorts->LBSCANOutputs.uDOWState =
        reqPorts->LBSSystemParam.bOSEFunctionActive;
    proPorts->LBSCANOutputs.uDOWObjIndex = uOSEWaningID;

    if (proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive) {
        proPorts->LBSCANOutputs.uDOWLeftActivation = 3u;
    } else if (proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive) {
        proPorts->LBSCANOutputs.uDOWLeftActivation = 2u;
    } else if (proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive) {
        proPorts->LBSCANOutputs.uDOWLeftActivation = 1u;
    } else {
        proPorts->LBSCANOutputs.uDOWLeftActivation = 0u;
    }

    if (proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive) {
        proPorts->LBSCANOutputs.uDOWRightActivation = 3u;
    } else if (proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive) {
        proPorts->LBSCANOutputs.uDOWRightActivation = 2u;
    } else if (proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive) {
        proPorts->LBSCANOutputs.uDOWRightActivation = 1u;
    } else {
        proPorts->LBSCANOutputs.uDOWRightActivation = 0u;
    }

    if ((uOSEWaningID < LBS_INPUT_OBJECT_NUMBER) &&
        (proPorts->LBSCANOutputs.uDOWLeftActivation != 0u)) {
        proPorts->LBSCANOutputs.siDOWLeftHighlightID_nu =
            reqPorts->GenObjList.aObject[uOSEWaningID].General.iRawFusionID_nu;
    } else {
        proPorts->LBSCANOutputs.siDOWLeftHighlightID_nu = -1;
    }

    if ((uOSEWaningID < LBS_INPUT_OBJECT_NUMBER) &&
        (proPorts->LBSCANOutputs.uDOWRightActivation != 0u)) {
        proPorts->LBSCANOutputs.siDOWRightHighlightID_nu =
            reqPorts->GenObjList.aObject[uOSEWaningID].General.iRawFusionID_nu;
    } else {
        proPorts->LBSCANOutputs.siDOWRightHighlightID_nu = -1;
    }

    if (reqPorts->LBSSupportInfo.DOWFailure &&
        reqPorts->LBSSystemParam.bOSEFunctionActive) {
        proPorts->LBSCANOutputs.uDOWSystemFaultStatus = 2u;
    } else {
        proPorts->LBSCANOutputs.uDOWSystemFaultStatus = 0u;
    }
}
/*****************************************************************************
  Functionname: LBSProcessRCWWarnings                                  */ /*!

    @brief: check if the RCW warning in enable conditions

    @description: check if the RCW warning in enable conditions,ego speed or
    switch

    @param[in]: reqPorts,params,proPorts,debugInfo

    @return:void
*****************************************************************************/
void LBSProcessRCWWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc) {
    // if (proPorts->LBSFunState.uRCWWarning == 1u) {
    //     proPorts->LBSCANOutputs.uRCWwarningReq = 1u;
    //     proPorts->LBSCANOutputs.uRCWWarnState = 1u;
    // } else if (proPorts->LBSFunState.uRCWWarning == 2u) {
    //     proPorts->LBSCANOutputs.uRCWwarningReq = 1u;
    //     proPorts->LBSCANOutputs.uRCWWarnState = 2u;
    // } else {
    //     proPorts->LBSCANOutputs.uRCWwarningReq = 0u;
    //     proPorts->LBSCANOutputs.uRCWWarnState = 0u;
    // }
    static boolean bRCWLastCycleActive = FALSE;
    boolean bRCWActiveFlag = FALSE;
    boolean bRCWStarActive = FALSE;
    uint8 uRCWActiveObjIndex_t = proPorts->LBSCANOutputs.uRCWObjIndex;
    if (proPorts->LBSFunState.uRCWWarning != 0u) {
        bRCWActiveFlag = TRUE;
    }

    if ((bRCWLastCycleActive == FALSE) && (bRCWActiveFlag == TRUE)) {
        bRCWStarActive = TRUE;
    }
    bRCWLastCycleActive = bRCWActiveFlag;

    static float32 fActiveRemainValue = 0.f;
    boolean bRCWActiveKeep =
        TUE_CML_TimerRetrigger(params->LBS_Ks_RCWParameter_nu.fCycletime_s,
                               bRCWStarActive, 1.f, &fActiveRemainValue);

    if (bRCWActiveKeep || bRCWActiveFlag) {
        if (proPorts->LBSFunState.uRCWWarning == 2u) {
            proPorts->LBSCANOutputs.uRCWwarningReq = 1u;
            proPorts->LBSCANOutputs.uRCWWarnState = 2u;
        } else {
            proPorts->LBSCANOutputs.uRCWwarningReq = 1u;
            proPorts->LBSCANOutputs.uRCWWarnState = 1u;
        }
    } else {
        proPorts->LBSCANOutputs.uRCWwarningReq = 0u;
        proPorts->LBSCANOutputs.uRCWWarnState = 0u;
    }

    if ((proPorts->LBSCANOutputs.uRCWwarningReq != 0u) &&
        (uRCWActiveObjIndex_t < LBS_INPUT_OBJECT_NUMBER)) {
        proPorts->LBSCANOutputs.siRCWHighlightID_nu =
            reqPorts->GenObjList.aObject[uRCWActiveObjIndex_t]
                .General.iRawFusionID_nu;
    } else {
        proPorts->LBSCANOutputs.siRCWHighlightID_nu = -1;
    }

    if (debugInfo->RCWDebug.LBSDebug_RCWstatemachine == 4u) {
        proPorts->LBSCANOutputs.uRCWSystemFaultStatus = 2u;
    } else {
        proPorts->LBSCANOutputs.uRCWSystemFaultStatus = 0u;
    }

    // if (debugInfo->RCWDebug.LBSDebug_RCWstatemachine == 5u) {
    if (reqPorts->LBSSystemParam.bRCWFunctionActive == FALSE) {
        proPorts->LBSCANOutputs.bRCWState = FALSE;
    } else {
        proPorts->LBSCANOutputs.bRCWState = TRUE;
    }
}

/*****************************************************************************
  Functionname: LBSOSEWarnCalcNextState                                  */ /*!

     @brief: Process the warning state for the given warning

     @description: Process the warning state for the given warning

     @param[in]: uWarnLevel     The warning

     @return: retValue          Flag whether OSE warning is output by the state
   machine
   *****************************************************************************/
boolean LBSOSEWarnCalcNextState(uint8 uWarnLevel,
                                boolean bWarnThisCycle,
                                const float32 fCycletime,
                                const LBSParam_st* params,
                                LBSCalculate_st* pLBSCalc) {
    boolean retValue;

    switch (pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel]) {
        case OSE_STATE_PASSIVE: {
            if (bWarnThisCycle) {
                // currently in passive state -> transition to active state
                pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] =
                    OSE_STATE_ACTIVE;
            }  // else remain in passive state
            // in passive state always reset timer and set output to false
            pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0f;
            retValue = FALSE;
        } break;
        case OSE_STATE_ACTIVE: {
            if (bWarnThisCycle) {
                // currently in active state -> check maximum timer
                if (pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] >
                    params->LBS_Ks_OSEParameter_nu
                        .LBS_Ka_OSEMaxTime_s[uWarnLevel]) {
                    // maximum warning duration has expired -> go into locked
                    // state and reset the timer
                    pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] =
                        OSE_STATE_LOCKED;
                    pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0f;
                    // set output location to FALSE
                    retValue = FALSE;
                } else {
                    // maximum warning duration has not yet expired -> keep
                    // state and set output to true
                    retValue = TRUE;
                }
            } else {
                // warning is not set this cycle -> check minimum time
                if (pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] <
                    params->LBS_Ks_OSEParameter_nu
                        .LBS_Ka_OSEMinTime_s[uWarnLevel]) {
                    // minimum time has not yet elapsed -> set output to true
                    retValue = TRUE;
                } else {
                    // minimum timer has expired and warning conditions are not
                    // met -> go into locked and reset timer
                    pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] =
                        OSE_STATE_LOCKED;
                    pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0f;
                    // set output to false
                    retValue = FALSE;
                }
            }
        } break;
        case OSE_STATE_LOCKED: {
            if (pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] >
                OSE_LBS_COMMON_WARN_LOCK_TIME)  // 5
            {
                // Lock time expired -> allowed to go inti passive state and
                // reset time
                pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] =
                    OSE_STATE_PASSIVE;
                pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0f;
            }  // else keep locked state
            // in locked state always set output to false
            retValue = FALSE;
        } break;
        default: {
            // always set output to false and reset timer
            retValue = FALSE;
            pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0f;
        } break;
    }
    // incream timer for current state
    pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] += fCycletime;
    return retValue;
}
/*****************************************************************************
  Functionname: LBSCheckEgoSpeedActThreshold                                  */ /*!

@brief:Check if the ego speed in the range of function activation

@description:Check if the ego speed in the range of function activation

@param[in]:fEgoSpeed,fDisableThresh,fEnableThresh,bEgoSpeedConLastCycle

@return:void
*****************************************************************************/
boolean LBSCheckEgoSpeedActThreshold(float32 fEgoSpeed,
                                     float32 fDisableThresh,
                                     float32 fEnableThresh,
                                     boolean* bEgoSpeedConLastCycle) {
    boolean bEgoSpeedCondition = FALSE;
    if ((fEgoSpeed > fEnableThresh) ||
        ((fEgoSpeed > fDisableThresh) && (*bEgoSpeedConLastCycle == TRUE))) {
        bEgoSpeedCondition = TRUE;
    }

    // Update speed condition last cycle
    *bEgoSpeedConLastCycle = bEgoSpeedCondition;

    return bEgoSpeedCondition;
}
/*****************************************************************************
  Functionname: LBSSetFunctionOutput                                  */ /*!

        @brief: Check if all parameters and conditions are met to activate
      function output

        @description: Check if all parameters and conditions are met to activate
      function output

        @param[in]:
                                bFunctionActive               Flag whether
      function has been activated by algorithm
                                bEgoSpeedCondition            Flag whether ego
      speed is sufficient to activate the function

        @return:    bFunctionOutput               Flag whether all conditions
      are met
      *****************************************************************************/
boolean LBSSetFunctionOutput(boolean bFunctionActive,
                             boolean bEgoSpeedCondition) {
    boolean bFunctionOutput = FALSE;

    // All conditions have to be fulfilled to set the function output
    if ((bFunctionActive == TRUE) && (bEgoSpeedCondition == TRUE)) {
        // function switch enable and in the enable speed range
        bFunctionOutput = TRUE;
    } else {
        bFunctionOutput = FALSE;
    }
    return bFunctionOutput;
}
/*****************************************************************************
  Functionname: LBS_Reset                                  */ /*!

                   @brief: LBS function init reset

                   @description:the function first exec reset process

                   @param[in]:void

                   @return:void
                 *****************************************************************************/
void LBS_Reset() {
    // Reset LBS Global
    LBSState = LBS_INIT;
    // init LBSGlobal struct
    uint8 uObjectIndex;
    LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();

    // init LBS Global parameter
    memset(pLBSCalculate, 0u, sizeof(LBSCalculate_st));

    // clear all LBS related information
    for (uObjectIndex = 0u; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
         uObjectIndex++) {
        LBSInitObjInfo(uObjectIndex);
    }

    // Init last cycle id to invalid
    for (uObjectIndex = 0u; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
         uObjectIndex++) {
        pLBSCalculate->LastObjIDList[uObjectIndex] = TUE_C_UI16_VALUE_INVALID;
        pLBSCalculate->LBSObjHistoryList[uObjectIndex].fFirstDetectX_met =
            TUE_C_F32_VALUE_INVALID;
        pLBSCalculate->LBSObjHistoryList[uObjectIndex].fFirstDetectY_met =
            TUE_C_F32_VALUE_INVALID;
        pLBSCalculate->LBSObjHistoryList[uObjectIndex].fMaxRange_met =
            TUE_C_F32_VALUE_INVALID;
    }

    // Set object selection algo parameters
    LBSFCTObjSelSetParameter();
}

/*****************************************************************************
  Functionname: LBSInitObjInfo                                  */ /*!

              @brief: LBS function for init calculate object array

              @description:the function first exec reset process for object

              @param[in]:uObjectIndex,the object array index

              @return:void
            *****************************************************************************/
void LBSInitObjInfo(uint8 uObjectIndex) {
    LBSObjInfo_st* pLBSObj = pGetLBSObjInfoPointer(uObjectIndex);

    // clear all LBS general object related information
    pLBSObj->fTTCAccel_mps2 = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fTTC_s = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fTTCFiltered_s = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fTTCRadial_s = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fCycletimeSum_s = 0.0f;
    pLBSObj->fUpdateRate_nu = 0.5f;
    pLBSObj->fAssocProbFiltered = 0.8f;
    pLBSObj->fAngle_deg = 0.0f;
    pLBSObj->fObjLengthMax = 0.0f;
    pLBSObj->fObjWidthMax = 0.0f;
    pLBSObj->fRangeRadial = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fVabs_mpss = 0.0f;
    pLBSObj->ObjBorders.fXmin_met = 0.0f;
    pLBSObj->ObjBorders.fXmax_met = 0.0f;
    pLBSObj->ObjBorders.fYmin_met = 0.0f;
    pLBSObj->ObjBorders.fYmax_met = 0.0f;
    pLBSObj->fXLastCycle_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fYLastCycle_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fVxPosBased = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fVyPosBased = TUE_C_F32_VALUE_INVALID;
    pLBSObj->ObjMovementBorders.fXmin_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->ObjMovementBorders.fXmax_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->ObjMovementBorders.fYmin_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->ObjMovementBorders.fYmax_met = TUE_C_F32_VALUE_INVALID;
    pLBSObj->fXMovement_met = 0.0f;
    pLBSObj->fYMovement_met = 0.0f;
    pLBSObj->uUniqueID = TUE_C_UI16_VALUE_INVALID;
    pLBSObj->uLastMergedObjID = TUE_C_UI8_VALUE_INVALID;
    pLBSObj->bLowTTCAtStart = FALSE;
    pLBSObj->bCreateAdjStableObj = FALSE;
    pLBSObj->fSpeedFiltered_mpss = 0.0f;
    pLBSObj->ObjSel.bBreakthroughHit = FALSE;
    pLBSObj->ObjSel.bObjectFastEnough = FALSE;
    pLBSObj->ObjSel.bObjInRange = FALSE;
    pLBSObj->firstStableDynProp = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */