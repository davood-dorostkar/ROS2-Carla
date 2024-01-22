/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_si_main.h"
#include "lbs_si_calculation.h"
#include "lbs_si_par.h"

#include "tue_common_libs.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SIInPut_st SIReqPorts;
SIOutPut_st SIProPorts;
SIParam_st SIParams;
SIDebug_st SIDebugInfo;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: LBS_SIExec                                  */ /*!

                  @brief:SI main function to run

                  @description:situation interpret main function

                  @param[in]:reqPorts   SI input pointer
                                         params     SI parameter input pointer
                                         proPorts   SI output pointer
                                         debugInfo  SI debug information pointer
                  @return:void
                *****************************************************************************/
void LBS_SIExec(const SIInPut_st* reqPorts,
                const SIParam_st* params,
                SIOutPut_st* proPorts,
                SIDebug_st* debugInfo) {
    static SIState_t SIState;

    // printf("SI PROCESS\n");
    if ((SIState == SI_OK) /*&&
        (reqPorts->GenObjList.sSigHeader.eSigStatus_nu == 1)*/) {
        /*Update SI global information*/
        SIUpdateGlobal(reqPorts, params);

        /*Do seek and track width calculate once and store results in global
         * variables*/
        SICalculateBaseLaneWidths(reqPorts, params);

        /*Calculate the trace brackets*/
        SICalculateLaneCorridor(reqPorts, params);

        /*Lane association */
        SI_LaneAssociation(reqPorts, params, proPorts, debugInfo);

    } else {
        LBS_SI_Init_Reset();
        SIState = SI_OK;
    }
}

void LBS_SI_Init_Reset() {}

/*****************************************************************************
  Functionname: SIUpdateGlobal                                  */ /*!

              @brief:Update SI global data

              @description:Update SI global data

              @param[in]:pRoad->fCurveRadius_met                    The
            curvature radius of the current driver road
                                     pRoad->fDrivenCurveRadius_met
            The curvature radius of the ego driver curve
                                     pLCAZone->fLCAZoneYMaxNear_met
            LCA near zone Ymax
                                     pLCAZone->fLCAZoneYMinNear_met
            LCA near zone Ymin
                                     pSIGlobal->fCurveRadiusMinFiltered_met
            The minimum filter curve radius

              @return:void
            *****************************************************************************/
void SIUpdateGlobal(const SIInPut_st* reqPorts, const SIParam_st* params) {
    // update the current lane curve radius(minimum of ego or lane curve radius)

    const SIRoad_t* pRoad = &reqPorts->Road;
    SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
    const float32 fCurveRadius = fABS(pRoad->fCurveRadius_met);
    const float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
    const float32 fCurveRadiusMin = MIN(fCurveRadius, fDrivenCurveRadius);

// Copy current lane width setting to SIGlobals struct
#if SI_DYNAMIC_LANE_BRACKET_EXTENSION
    pSIGlobal->fLaneWidth_met = pRoad->fLaneWidth_met;
#else
    const SI_LCAZone_t* pLCAZone = &params->LCAParamter.LCAZone;
    const float32 fLCAZoneYMaxNear_met = pLCAZone->fLCAZoneYMaxNear_met;
    const float32 fLCAZoneYMinNear_met = pLCAZone->fLCAZoneYMinNear_met;
    pSIGlobal->fLaneWidth_met = (fLCAZoneYMaxNear_met - fLCAZoneYMinNear_met);
#endif
    // Update filtered curve radius
    if (fCurveRadiusMin < pSIGlobal->fCurveRadiusMinFiltered_met) {
        // if CurveRadius down use low pass filter down alpha
        TUE_CML_LowPassFilter(&pSIGlobal->fCurveRadiusMinFiltered_met,
                              fCurveRadiusMin, 0.2f);
    } else {
        // if CurveRadius up use low pass filter up alpha
        TUE_CML_LowPassFilter(&pSIGlobal->fCurveRadiusMinFiltered_met,
                              fCurveRadiusMin, 0.02f);
    }
}

/*****************************************************************************
  Functionname: SICalculateBaseLaneWidths                                  */ /*!

   @brief:Update Calculate the seek and track lane widths

   @description:Calculate the seek and track mode lane widths

   @param[in]:reqPorts   SI input pointer

   @return:void
 *****************************************************************************/
void SICalculateBaseLaneWidths(const SIInPut_st* reqPorts,
                               const SIParam_st* params) {
    SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
    const SIVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;

    /*Seek mode lane Width*/
    pSIGlobal->fSIseekLaneWidth_met =
        SICalcRelTraTrackWidthSeek(pEgoInfo, params);

    /*Track mode lane width*/
    pSIGlobal->fSITrackLaneWidth_met =
        SICalcRelTraTrackWidthTrack(pEgoInfo, params);
}

/*****************************************************************************
  Functionname: SICalculateLaneCorridor                                  */ /*!

     @brief:Calculates the corridor width for the actual object,include criteria
            for corridor expansion and restriction

     @description:This is the "main" function of the corridor evaluation
   functions.
                  It loops through all objects first updating their relevancy
   information
                              (function SIAdvantageRelevantObject),and
   calculating the corresponding
                              trace bracket

     @param[in]: reqPorts : SIInPut_st pointer
                 params   : SIParam_st parameter pointer
                             LaneWidth: Estimated lane width
     @param[out]: fTraceBracketLeft_met :Left trace bracket for each
   object,AUTOSAR coordinates
                  fTraceBracketRight_met :Right trace bracket for each
   object,AUTOSAR coordinates
     @return:void
   *****************************************************************************/
void SICalculateLaneCorridor(const SIInPut_st* reqPorts,
                             const SIParam_st* params) {
    // uint8 uObj;
    // const SIGenObject_st* pGenObj = NULL;
    // SISRRObject_st* pSRRObj = NULL;
    // SI_Info_st* pSIObj = NULL;
    // const SILCAObjInfo_t* pLCAObj = NULL;
    // const SILCAObjInfo_Array* pLCAObjList = reqPorts->LCAObjInfoList;

    for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        const SIGenObject_st* pGenObj = &reqPorts->GenObjList.aObject[uObj];
        // pSRRObj = &reqPorts->SRRObjList.aObject[uObj];
        // pLCAObj = pLCAObjList[uObj];
        const SILCAObjInfo_t* pLCAObj = &reqPorts->LCAObjInfoList[uObj];
        SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);

        if (pGenObj->GenObjInfo.uiMaintenanceState_nu !=
            SI_EM_GEN_OBJ_MT_STATE_DELETED) {
            /*Update Object corridor information for given object*/
            SIAdvantageRelevantObject(
                uObj, pGenObj, pLCAObj,
                reqPorts->SISysParam.fCycletime_s);  // pSRRObj,

            /*Calculate the lane assignment(trace bracket)matrix*/
            SICalcCriteriaMatrix(uObj, reqPorts, params);

            /*Output the Obj trace bracket left and right side position*/
            SISetTraceBracketOutput(uObj, pGenObj, pSIObj, params);
        }
    }
}

/*****************************************************************************
  Functionname: SI_LaneAssociation                                  */ /*!

          @brief: Binding each target to the corresponding lane

          @description: Binding each target to the corresponding lane

          @param[in]: reqPorts  params  proPorts  debugInfo

          @return:void
        *****************************************************************************/
void SI_LaneAssociation(const SIInPut_st* reqPorts,
                        const SIParam_st* params,
                        SIOutPut_st* proPorts,
                        SIDebug_st* debugInfo) {
    uint8 uObj;
    // SI_Info_st* pSIObj = NULL;
    // const SIGenObject_st* pGenObj = NULL;
    // const SILBSObjInfo_st* pLBSObjInfo = NULL;
    SITrajOccupancy_t Occupancy;
    // boolean bStateFlow;
    const SLACInReq_t reqPorts1 = {
        &reqPorts->GenObjList, &reqPorts->EgoVehInfo, &reqPorts->Road,
        reqPorts->SISysParam.fCycletime_s};  //&reqPorts->SRRObjList,

    for (uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
      const SIGenObject_st* pGenObj = &reqPorts->GenObjList.aObject[uObj];
        if (pGenObj->GenObjInfo.uiMaintenanceState_nu !=
            SI_EM_GEN_OBJ_MT_STATE_DELETED) {
            SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);
            const SIGenObject_st* pGenObj = &(reqPorts->GenObjList.aObject[uObj]);
            const SILBSObjInfo_st* pLBSObjInfo = &reqPorts->LBSObjInfoList[uObj];

            /*Calculate the overlap with the trace brackets*/
            SICalculateInlaneOverlap(uObj, pGenObj, pSIObj, SI_MIN_OBJECT_WIDTH,
                                    SI_MAX_OBJECT_WIDTH, &Occupancy);

            /*Lane Association State flow*/
            SILaneAsscoiationChange(uObj, pGenObj, pSIObj, pLBSObjInfo, &Occupancy,
                                    reqPorts1, params);

            /*Associates an object with one of the road lanes*/
            SIGetAssociateLane(uObj, pGenObj, pSIObj);

            /*Output the lane association information*/
            SIPostProcess(uObj, proPorts, debugInfo);
        }
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */