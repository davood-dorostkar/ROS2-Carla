/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_bsd_main.h"
#include "lbs_bsd_calculation.h"
#include "lbs_bsd_par.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
BSDInReq_st BsdReqPorts;
BSDOutPro_st BsdProPorts;
BSDParam_st BsdParams;
BSDDebug_t BsdDebugInfo;
/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: BSDExec                                  */ /*!

                     @brief: BSD Exec main function

                     @description: BSD main function

                     @param[in]:LBS input,params

                     @return:void
                   *****************************************************************************/
void LBS_BSDExec(const BSDInReq_st* reqPorts,
                 const BSDParam_st* params,
                 BSDOutPro_st* proPorts,
                 BSDDebug_t* debugInfo) {
    boolean bBSDFunctionActive = reqPorts->BSDSystemParam.bBSDFunctionActive;
    boolean bBSDFunctionActiveLast =
        pGetBSDCalculatePointer()->BSDRunState.bBSDFunctionActionLastCycle;
    // BSDState_t eBSDState = pGetBSDCalculatePointer()->BSDRunState.eBSDState;
    BSDState_t eBSDState = BSDCalculate.BSDRunState.eBSDState;

    switch (eBSDState) {
        case BSD_OK:
            /*Function switch enable and function has been init*/
            if (bBSDFunctionActive == TRUE) {
                BSD_PreProecss(reqPorts, params, proPorts, debugInfo);
                BSD_MainProcess(reqPorts, params, proPorts, debugInfo);
                BSD_PostProcess(reqPorts, params, proPorts, debugInfo);
            } else {
                /*Function exit reset*/
                if (bBSDFunctionActiveLast == TRUE) {
                    LBS_BSD_Init_Reset();
                    pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
                }
            }
            break;
        case BSD_INIT:
            /*First enter reset*/
            LBS_BSD_Init_Reset();
            pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
            break;
        default:
            LBS_BSD_Init_Reset();
            pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
            break;
    }
    pGetBSDCalculatePointer()->BSDRunState.bBSDFunctionActionLastCycle =
        bBSDFunctionActive;
}

/*****************************************************************************
  Functionname: LBS_BSD_Init_Reset                                  */ /*!

          @brief: BSD function init reset

          @description:the function first exec reset process

          @param[in]:void

          @return: void
        *****************************************************************************/
void LBS_BSD_Init_Reset() {
    // Init internal object data
    BSD_InitObject();
    // Init internal global data
    BSD_InitGlobal();
    // Init BSD run flag
    pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_INIT;
}

/*****************************************************************************
  Functionname: BSD_InitObject                                  */ /*!

              @brief:this function initializes all LBS internal objects related
            BSD-data

              @description: include zone parameter and object property init

              @param[in]:void

              @return:void
            *****************************************************************************/
void BSD_InitObject() {
    // init pro port object array
    // proPorts->BSDObjInfo
    uint8 uObjNumber;

    for (uObjNumber = 0u; uObjNumber < LBS_INPUT_OBJECT_NUMBER; uObjNumber++) {
        BSD_InitOneObject(uObjNumber);
    }
}

/*****************************************************************************
  Functionname: BSD_InitOneObject                                  */ /*!

           @brief:this function initializes one LBS internal objects related
         BSD-data

           @description: include zone parameter and object property init

           @param[in]:uObjNumber:the object array index number

           @return:void
         *****************************************************************************/
void BSD_InitOneObject(uint8 uObjNumber) {
    BSD_Info_t* pBSDObjInfo;
    BSDZone_ObjPar* pBSDObjZonePar;

    // Get pointer of Calculate data
    pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObjNumber);
    pBSDObjZonePar = pGetBSDCalculatePointer_BSDZoneObjPar(uObjNumber);

    // init check data like delay time,appearance
    pBSDObjInfo->fSoTDelayTime_s = 0.0f;
    pBSDObjInfo->fRearConf_nu = 0.0f;
    pBSDObjInfo->fBSDZoneObjXmin_met = 0.0f;
    pBSDObjInfo->ubAppearance_nu = BSD_APPEAR_INVALID;
    pBSDObjInfo->ubHitsInFront_nu = 0u;
    pBSDObjInfo->ubHitsInSide_nu = 0u;
    pBSDObjInfo->ubHitsInRear_nu = 0u;
    pBSDObjInfo->ubGrdHitCounter_nu = 0u;
    pBSDObjInfo->ubBehindGrdCounter_nu = 0u;
    pBSDObjInfo->ubClass_nu = BSD_CLASS_UNDEFINED;
    pBSDObjInfo->ubOwnLaneCounter_nu = 0u;

    // init check boolean result
    pBSDObjInfo->bBSDRelevant = FALSE;
    pBSDObjInfo->bBSDWarning = FALSE;
    pBSDObjInfo->bCreateBehindGRD = FALSE;
    pBSDObjInfo->bFastSoT = FALSE;
    pBSDObjInfo->bInBSDZone = FALSE;
    pBSDObjInfo->bInSOTZone = FALSE;
    pBSDObjInfo->bInSOTZonePrevious = FALSE;
    pBSDObjInfo->bIsSoT = FALSE;
    pBSDObjInfo->bLivedLongEnough = FALSE;
    pBSDObjInfo->bObjectAndZoneOverlap = FALSE;
    pBSDObjInfo->bObjectBehindGRD = FALSE;
    pBSDObjInfo->bObjectOnOwnlane = FALSE;
    pBSDObjInfo->bPlausibility = FALSE;
    pBSDObjInfo->bPossibleWrappedObj = FALSE;
    pBSDObjInfo->bQualityEnough = FALSE;
    pBSDObjInfo->bShortWarn = FALSE;
    pBSDObjInfo->bSoTDelayActive = FALSE;
    pBSDObjInfo->bUpdatedRecently = FALSE;
    pBSDObjInfo->bUpdatedRecentlyWeak = FALSE;

    // init each object warning zone parameter
    pBSDObjZonePar->fZoneXmax_met = 0.0f;
    pBSDObjZonePar->fZoneXmin_met = 0.0f;
    pBSDObjZonePar->fZoneYmax_met = 0.0f;
    pBSDObjZonePar->fZoneYmin_met = 0.0f;
}

/*****************************************************************************
  Functionname: BSD_InitGlobal                                  */ /*!

              @brief:this function initializes the internal global BSD data

              @description:initializes the BSD global data and Zone parameter

              @param[in]:void

              @return:void
            *****************************************************************************/
void BSD_InitGlobal() {
    // init BsdGlobal struct
    BSDCalculate_st* pBSDCal = pGetBSDCalculatePointer();
    BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
    BSDGlobal_BSDZone_st* pBSDGlobalZonePar =
        pGetBSDCalculatePointer_BSDGlobaZonePar();

    // Init BSD Global parameter
    memset(pBSDGlobal, 0u, sizeof(BSD_Globals_t));
    pBSDGlobal->fMinAssocProbFront_nu = BSD_ASSOCPROB_INIT;
    pBSDGlobal->fMinAssocProbSideRear_nu = BSD_ASSOCPROB_INIT;
    pBSDGlobal->fMinXmoved_met = 0.0f;
    pBSDGlobal->fBSDZoneXmin_met = TUE_C_F32_VALUE_INVALID;

    // Init left and right side warning zone parameter
    memset(pBSDGlobalZonePar, 0u, sizeof(BSDGlobal_BSDZone_st));

    // Init the first cycle object id array
    for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        pBSDCal->LastObjIDList[uObj] = TUE_C_UI16_VALUE_INVALID;
    }

    // BSDMultiObjCalcuInfoReset();
}

/*****************************************************************************
  Functionname: BSD_PrePorecss                                  */ /*!

              @brief: BSD PreProcess function

              @description: BSD PreProcess function to update BSDZone parameter
            and threshold

              @param[in]:LBS Global input,Road,Params

              @return:void
            *****************************************************************************/
void BSD_PreProecss(const BSDInReq_st* reqPorts,
                    const BSDParam_st* params,
                    BSDOutPro_st* proPorts,
                    BSDDebug_t* debugInfo) {
    const BSDVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
    const BSD_LBSGlobalInfo_t* pLBSGlobalInput =
        &reqPorts->LBSInputInfo.LBSGlobalInfo;
    const BSDRoad_t* pRoad = &reqPorts->Road;
    const BSDSensorMounting_t* pSensorMounting =
        &params->SensorMounting.SensorLeft;  // Temp to use left side
    // const BSDVehParameter_t* pBSDVehParameter = &params->BSDVehParameter;

    /*update left and right side basic BSD Zone parameter at the same time*/
    BSDGetBSDZoneParameters(pLBSGlobalInput, pEgoInfo, pRoad, params);

    /*Calculate object association  threshold*/
    BSDCycleGlobalUpdate(pSensorMounting);

    /*Calculate check object moving or stationary threshold of velocity */
    BSDCalculateVxThreshMovStatClassification(pEgoInfo);

    /*Clear object BSD calculate array if the object id changed this cycle*/
    BSDInitObjCalArray(&reqPorts->GenObjList);

    /*Store last cycle warning flag*/
    pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveLeftLastCycle =
        pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveLeft;
    pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveRightLastCycle =
        pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveRight;
    pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveLeft = FALSE;
    pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveRight = FALSE;
}

/*****************************************************************************
  Functionname: BSDInitObjCalArray                                  */ /*!

          @brief: Clear BSD calculate array

          @description: Clear BSD calculate array if the object id changed
                        and save the new object for next cycle to judge if the
        object id change

          @param[in]:pGenObjList: the LBS object list input pointer

          @return:void
        *****************************************************************************/
void BSDInitObjCalArray(const BSDGenObjList_st* pGenObjList) {
    uint8 uObj;

    const BSD_GenObject_st* pGenObjInfo = NULL;
    BSDCalculate_st* pBSDCalculate = pGetBSDCalculatePointer();

    for (uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        pGenObjInfo = pGetBSDGenObjListPointer_Object(uObj, pGenObjList);

        if (pBSDCalculate->LastObjIDList[uObj] !=
            pGenObjInfo->General.uiID_nu) {
            // object id have been change,init the calculate array
            BSD_InitOneObject(uObj);
            // save current object id
            pBSDCalculate->LastObjIDList[uObj] = pGenObjInfo->General.uiID_nu;
        }
    }
}

/*****************************************************************************
  Functionname: BSDGetBSDZoneParameters                                  */ /*!

  @brief:Update current both side BSDZone Parameter 

  @description:update both side BSDZone Parameter

  @param[in]:LBSGlobalsInfo,EgoInfo,RoadInfo,params

  @return:void
*****************************************************************************/

void BSDGetBSDZoneParameters(const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
                             const BSDVehicleInfo_t* pEgoInfo,
                             const BSDRoad_t* pRoad,
                             const BSDParam_st* params) {
    BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
    BSDGlobal_BSDZone_st* pBSDGlobalZonePar =
        pGetBSDCalculatePointer_BSDGlobaZonePar();
    BSDZoneParameter_t* pLeftZonePar = &pBSDGlobalZonePar->BSDZoneParameterLeft;
    BSDZoneParameter_t* pRightZonePar =
        &pBSDGlobalZonePar->BSDZoneParameterRight;
    const BsdZone_t* pBSDZoneParams = &params->BsdZone;
    // const BSDSensorMounting_t* pSenserMountLeft =
    // &params->SensorMounting.SensorLeft;
    const BSDSensorMounting_t* pSenserMountRight =
        &params->SensorMounting.SensorRight;

    const float32 fVehicleWidth = params->BSDVehParameter.fVehicleWidth_met;
    const float32 fCenter2Axis =
        params->BSDVehParameter.fVehCenter2FrontAxis_met;
    const float32 fXmin = pBSDZoneParams->fXmin_met;
    const float32 fXmax = pBSDZoneParams->fXmax_met;
    const float32 fYmin = pBSDZoneParams->fYmin_met;
    const float32 fYmax = pBSDZoneParams->fYmax_met;
    const float32 fX_hys = pBSDZoneParams->fHysteresisX_met;
    const float32 fYmin_hys = pBSDZoneParams->fHysteresisYmin_met;
    const float32 fYmax_hys = pBSDZoneParams->fHysteresisYmax_met;
    float32 fXminAdapt;

    if (pBSDGlobal->fBSDZoneXmin_met == TUE_C_F32_VALUE_INVALID) {
        // Init BSD Zone X Min value
        pBSDGlobal->fBSDZoneXmin_met =
            pBSDZoneParams->fXmin_met -
            params->BSDVehParameter.fVehCenter2FrontAxis_met;
    }
    const float32 fZoneXminLastCycle = pBSDGlobal->fBSDZoneXmin_met;

    /**************************************************************
     *update X min adapter range
     **************************************************************/
    pBSDGlobal->fBSDZoneXminStatic_met = fXmin - fCenter2Axis;
    fXminAdapt = BSDCalculateAdaptedBSDZoneLength(
        pLBSGlobalInput, pEgoInfo, pRoad, pSenserMountRight, fZoneXminLastCycle,
        fCenter2Axis);

    /**************************************************************
     *update left side BSD Zone parameter
     **************************************************************/
    // Object coordinate has been translate to AUTOSAR,so not need to
    // compensation sensor mount distance
    pLeftZonePar->fBSDZoneXmin_met = fXminAdapt;
    pLeftZonePar->fBSDZoneXminWithHyst_met = fXminAdapt - fX_hys;
    pLeftZonePar->fBSDZoneXmax_met = fXmax - fCenter2Axis;
    pLeftZonePar->fBSDZoneXmaxWithHyst_met =
        pLeftZonePar->fBSDZoneXmax_met + fX_hys;

    // add vehicle width (consider sensor mounting) to the Zone for transform
    // coordinator from zone to ego
    pLeftZonePar->fBSDZoneYmin_met = fYmin + (fVehicleWidth * 0.5f);
    pLeftZonePar->fBSDZoneYmax_met = fYmax + (fVehicleWidth * 0.5f);
    pLeftZonePar->fBSDZoneYminWithHyst_met =
        pLeftZonePar->fBSDZoneYmin_met - fYmin_hys;
    pLeftZonePar->fBSDZoneYmaxWithHyst_met =
        pLeftZonePar->fBSDZoneYmax_met + fYmax_hys;

    /**************************************************************
     *update right side BSD Zone parameter                         *
     **************************************************************/
    pRightZonePar->fBSDZoneXmin_met = fXminAdapt;
    pRightZonePar->fBSDZoneXminWithHyst_met = fXminAdapt - fX_hys;
    pRightZonePar->fBSDZoneXmax_met = fXmax - fCenter2Axis;
    pRightZonePar->fBSDZoneXmaxWithHyst_met =
        pRightZonePar->fBSDZoneXmax_met + fX_hys;

    pRightZonePar->fBSDZoneYmin_met = -fYmax - (fVehicleWidth * 0.5f);
    pRightZonePar->fBSDZoneYmax_met = -fYmin - (fVehicleWidth * 0.5f);
    pRightZonePar->fBSDZoneYminWithHyst_met =
        pRightZonePar->fBSDZoneYmin_met - fYmin_hys;
    pRightZonePar->fBSDZoneYmaxWithHyst_met =
        pRightZonePar->fBSDZoneYmax_met + fYmax_hys;
    // printf("%f\t%f\t%f\n", fXminAdapt, fXmax,
    // pRightZonePar->fBSDZoneXmin_met);
}

/*****************************************************************************
  Functionname: BSDCycleGlobalUpdate                                  */ /*!

        @brief:Calculate BSD Global threshold and store

        @description:according to sensor mounting angle to get the BSD threshold

        @param[in]:sensorMountingInfo

        @return:void
      *****************************************************************************/
void BSDCycleGlobalUpdate(const BSDSensorMounting_t* pSensorMounting) {
    // update global information
    BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
    const float32 fMountingAngleDeg =
        RAD2DEG(fABS(pSensorMounting->fRollAngle_rad)) - 90.0f;

    // Calculate front association  threshold
    pBSDGlobal->fMinAssocProbFront_nu = GDBmathLinFuncLimBounded(
        fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
        BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINPROB_FRONT,
        BSD_LI_ASSOC_MAX_MINPROB_FRONT);

    // Calculate side association  threshold
    pBSDGlobal->fMinAssocProbSideRear_nu = GDBmathLinFuncLimBounded(
        fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
        BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINPROB_SIDEREAR,
        BSD_LI_ASSOC_MAX_MINPROB_SIDEREAR);

    // Calculate x moved association  threshold
    pBSDGlobal->fMinXmoved_met = GDBmathLinFuncLimBounded(
        fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
        BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINXMOVED,
        BSD_LI_ASSOC_MAX_MINXMOVED);

    // init SOT object number counter
    pBSDGlobal->ScenarioObserver.uNumberSoTObjsLastCycle_nu =
        pBSDGlobal->ScenarioObserver.uNumberSoTObjs_nu;
    pBSDGlobal->ScenarioObserver.uNumberSoTObjs_nu = 0u;
}

/*****************************************************************************
  Functionname: BSDCalculateVxThreshMovStatClassification */ /*!

@brief:Update object moving state velocity threshold

@description:according ego speed to get the move state velocity threshold

@param[in]:EgoInfo;

@return:void
*****************************************************************************/
void BSDCalculateVxThreshMovStatClassification(
    const BSDVehicleInfo_t* pEgoInfo) {
    BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();

    // base ego speed to interpolation fVxThreshMovStat_mps value
    pBSDGlobal->fVxThreshMovStat_mps = GDBmathLinFuncLimBounded(
        pEgoInfo->fegoVelocity_mps, BSD_LI_MOVSTAT_EGOSPEED_MIN,
        BSD_LI_MOVSTAT_EGOSPEED_MAX, BSD_LI_MOVSTAT_VXTHRESH_MIN,
        BSD_LI_MOVSTAT_VXTHRESH_MAX);
}

/*****************************************************************************
  Functionname: BSD_MainProcess                                  */ /*!

  @brief:the BSD Warning main logical function

  @description:the BSD Warning main logical function

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
static uint8 LBS_BSD_ObjCounter = 0u;
void BSD_MainProcess(const BSDInReq_st* reqPorts,
                     const BSDParam_st* params,
                     BSDOutPro_st* proPorts,
                     BSDDebug_t* debugInfo) {
    uint16 uObj;
    const BSDVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
    const BSDGenObjList_st* pGenObjList = &reqPorts->GenObjList;
    const BSDSRRObjList_st* pSRRObjList = &reqPorts->SRRObjList;
    const BSDRoad_t* pRoad = &reqPorts->Road;
    const BSD_LBSInputInfo_st* pLBSInputInfo = &reqPorts->LBSInputInfo;
    const BSDSystemParam_t* pBSDSystemParam = &reqPorts->BSDSystemParam;
    const BSDPreProcessInput_t* pBSDPreProcessInput =
        &reqPorts->BSDPreProcessInput;

    const BsdZone_t* pBsdZonePar = &params->BsdZone;
    const BSDSensorMounting_t* pSensorMounting =
        &params->SensorMounting.SensorLeft;  // Temp to use left side parameter
    const BSDWarningParameter_t* pBSDWarnParameter =
        &params->BSDWarningParameter;
    const BSDVehParameter_t* pVehParameter = &params->BSDVehParameter;

    float32 fBSDGlobalWarningDistX;

    BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();

    // Rest Warning flag from last cycle
    pBSDGlobal->bBSDWarnActive = FALSE;
    pBSDGlobal->bBSDWarnActiveLeft = FALSE;
    pBSDGlobal->bBSDWarnActiveRight = FALSE;
    pBSDGlobal->fBSDWarnActiveLeftDistX_met = -F32_VALUE_INVALID;
    pBSDGlobal->fBSDWarnActiveRightDistX_met = -F32_VALUE_INVALID;

    for (uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        if (!bGetObjIsDeleted(uObj, pGenObjList)) {
            LBS_BSD_ObjCounter = uObj;
            BSD_Info_t* pBSDObjInfo = NULL;
            const BSD_GenObject_st* pGenObjInfo = NULL;
            const BSD_SRRObject_st* pSRRObjInfo = NULL;

            pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObj);
            pGenObjInfo = pGetBSDGenObjListPointer_Object(uObj, pGenObjList);
            pSRRObjInfo = pGetBSDSRRObjListPointer_Object(uObj, pSRRObjList);

            // update BSD Warning active flag by current object direction flag
            BSDUpdateBSDWarningActiveFlag(uObj, pGenObjInfo);

            // update BSD zone with hysteresis by current object direction flag
            BSDUpdateBSDZoneWithHyst(uObj, pGenObjInfo, pBSDGlobal);

            // judge sensor side to update current global BSD Zone parameter
            BSDUpdateGlobalZoneParameters(uObj, pGenObjInfo);

            // base on current object warning state to adjust object BSD zone
            // area
            BSDGetZoneParametersForCurrentObject(uObj, pGenObjInfo, pRoad,
                                                 pBsdZonePar);

            // Calculate object compensation angle for SectorCuts
            BSDCalculateSectorCuts(uObj, pGenObjInfo,
                                   &pLBSInputInfo->LBSGlobalInfo, pEgoInfo,
                                   pRoad, pVehParameter);

            // check object appearance to the BSD zone from front,side or rear
            pBSDObjInfo->ubAppearance_nu = BSDClassifyAppreance(
                uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pVehParameter);

            // check whether the object is in the BSD Zone area
            pBSDObjInfo->bInBSDZone =
                BSDCheckObjectInBSDZone(uObj, pLBSInputInfo, pEgoInfo, pRoad);

            // check whether the object is in the SOT Zone area
            pBSDObjInfo->bInSOTZone = BSDCheckObjectInSOTZone(
                uObj, pGenObjInfo, pLBSInputInfo, pRoad);

            // check whether the object has enough overlap with the zero to warn
            pBSDObjInfo->bObjectAndZoneOverlap = BSDCheckObjectZoneOverlap(
                uObj, pGenObjInfo, pLBSInputInfo, pRoad);

            // update possibly guardrail object counter
            BSDCalculateUpdateGrdCounter(uObj, pGenObjInfo, pSRRObjInfo,
                                         pLBSInputInfo, pEgoInfo, pRoad,
                                         pSensorMounting);
            pBSDObjInfo->bObjectBehindGRD =
                BSDCheckObjectBehindGRD(uObj, pGenObjInfo, pLBSInputInfo);

            // check whether the object is behind guardrail
            BSDCalculateUpdateOwnlaneCounter(uObj, pGenObjInfo, pSRRObjInfo,
                                             pLBSInputInfo, pRoad);

            // count up if object is on the own lane
            pBSDObjInfo->bObjectOnOwnlane =
                BSDCheckObjectOnOwnlane(uObj, pGenObjInfo);

            // check whether the object quality is high enough
            pBSDObjInfo->bQualityEnough = BSDCheckObjectQuality(
                uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pEgoInfo);

            // check whether the object lived enough
            pBSDObjInfo->bLivedLongEnough =
                BSDCheckObjectLivedLongEnough(uObj, pLBSInputInfo);

            // check whether the object was updated in last few cycle
            BSDCheckObjectUpdatedRecently(uObj, pSRRObjInfo);

            // classify object
            BSDClassifyObject(uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo,
                              pEgoInfo, pRoad, pVehParameter);

            // classify objects which have a high enough guardrail hit counter
            BSDClassifyGRD(uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo,
                           pEgoInfo, pRoad);

            // check object is BSD relevant
            pBSDObjInfo->bBSDRelevant =
                BSDCheckObjectIsRelevant(uObj, pGenObjInfo, pVehParameter);

            // check whether the object is SOT target
            pBSDObjInfo->bIsSoT =
                BSDCheckObjectSoT(uObj, pGenObjInfo, pLBSInputInfo,
                                  pBSDWarnParameter, pVehParameter);

            // check object for Fast SoT condition
            pBSDObjInfo->bFastSoT = BSDCheckObjectFastSoT(
                uObj, pGenObjInfo, pBSDWarnParameter, pBSDSystemParam);

            // check SOT delay
            pBSDObjInfo->bSoTDelayActive =
                BSDCheckObjectSoTDelay(uObj, pBSDWarnParameter);

            // calculate the new SOT object delay time
            BSDCalculateUpdateObjectSoTDelay(uObj, pGenObjInfo, pSRRObjInfo,
                                             pLBSInputInfo, pBSDSystemParam,
                                             pVehParameter);

            // update the scenario observer
            BSDUpdateScenarioObserver(uObj, pGenObjInfo);

            // check whether the object movement is plausible
            pBSDObjInfo->bPlausibility = BSDCheckObjectPlausibility(
                uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pEgoInfo,
                pVehParameter);

            // check for short warning,to inhibition short warn
            pBSDObjInfo->bShortWarn = BSDCheckObjectShortWarning(
                uObj, pGenObjInfo, pLBSInputInfo, pRoad, pBSDWarnParameter,
                pBSDSystemParam);

            // check for VRU velocity out of range
            // pBSDObjInfo->bVRUvelocitysuppression =
            // BSDCheckVRUObjectVelocity(uObj, pGenObjInfo);

            // check whether all warning conditions are fulfilled
            pBSDObjInfo->bBSDWarning = BSDCheckObjectWarningConditions(
                uObj, pSRRObjInfo, pLBSInputInfo);

            // Calculate real distance if object in warning state
            fBSDGlobalWarningDistX = BSDCalculateObjectRelDist(
                uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo);

            // check the object warning state,and switch on the
            // globapSRRObjInfol warning
            BSDSetGlobalWarning(uObj, pBSDObjInfo, fBSDGlobalWarningDistX);
        }
    }
    // check for the multi object activation scenario*/
    // BSDCheckMultiObjectActivation(pBSDSystemParam);

    // BSD state precondition process
    LBSBSDStateConditionProcess(reqPorts, pBSDPreProcessInput, pEgoInfo);

    // BSD state machine
    LBSBSDStateMachineProcess();
}

/*****************************************************************************
  Functionname: BSD_PostProcess                                  */ /*!

  @brief:the BSD post output function

  @description:output the BSD calculate result to proPorts 

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void BSD_PostProcess(const BSDInReq_st* reqPorts,
                     const BSDParam_st* params,
                     BSDOutPro_st* proPorts,
                     BSDDebug_t* debugInfo) {
    // set output to BSD global
    BSD_Globals_t* pBSDOutputGlobals = &proPorts->BSD_Globals;

    const BSD_Globals_t* const pBSDGlobals =
        pGetBSDCalculatePointer_BSDGlobals();
    // const BSD_Info_Array* const pBSDObjList =
    //     &pGetBSDCalculatePointer()->BSDObjInfoList;
    // const BSDZone_ObjPar_Array* const pBSDObjZoneArr =
        // &pGetBSDCalculatePointer()->BSDZoneObjParList;
    // const BSDGlobal_BSDZone_st* pBSDGlobalZonePar =
    //     pGetBSDCalculatePointer_BSDGlobaZonePar();
    // const BSD_Warn_Decide_Debug_Array* pBSDWarnDecide =
    //     &pGetBSDCalculatePointer()->BSDWarnDecideLsit;

    // set global data and obj data to output
    memcpy(pBSDOutputGlobals, pBSDGlobals, sizeof(BSD_Globals_t));
    // memcpy(proPorts->BSDZoneObjParList, pBSDObjZoneArr,
    //        sizeof(BSDZone_ObjPar_Array));
    // set Debug output
    // memcpy(&debugInfo->BSDGlobalsZones, pBSDGlobalZonePar,
    //        sizeof(BSDGlobal_BSDZone_st));
    // memcpy(&debugInfo->BSD_Globals, pBSDGlobals, sizeof(BSD_Globals_t));
    // memcpy(&debugInfo->BSDZoneObjParList, pBSDObjZoneArr,
    //        sizeof(BSDZone_ObjPar_Array));
    // memcpy(&debugInfo->BSDObjInfo, pBSDObjList, sizeof(BSD_Info_Array));
    // memcpy(&debugInfo->BSDWarnDecideList_Debug, pBSDWarnDecide,
    //        sizeof(BSD_Warn_Decide_Debug_Array));
    for (uint8 uindex = 0u; uindex < LBS_INPUT_OBJECT_NUMBER; uindex++) {
        proPorts->BSDOutputInfoList[uindex].fZoneXmin_met =
            pGetBSDCalculatePointer_BSDZoneObjPar(uindex)->fZoneXmin_met;
        proPorts->BSDOutputInfoList[uindex].bBSDWarning =
            pGetBSDCalculatePointer_ObjInfo(uindex)->bBSDWarning;
    }
    debugInfo->uiVersionNumber = LBS_BSD_VERSION_NUMBER;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */