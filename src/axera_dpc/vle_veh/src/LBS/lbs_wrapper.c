/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "lbs_wrapper.h"

/*****************************************************************************
  SI FUNCTION PROTOTYPES
*****************************************************************************/
void LBSSIInputWrapper(const LBSInReq_st* extReqPorts,
                       const LBSParam_st* extReqParam,
                       SIInPut_st* input,
                       SIParam_st* paramInput,
                       LBSCalculate_st* pLBSCalc) {
    // memcpy LBSParam to SIParam
    // memcpy(&paramInput->pSensorMounting, &extReqParam->SensorMounting,
    // sizeof(LBS_SensorMounting_st)); memcpy(&paramInput->pVehParameter,
    // &extReqParam->VehParameter, sizeof(LBS_VehParameter_t));
    input->GenObjList.sSigHeader.eSigStatus_nu = 1u;
    // Ego vehicle information
    input->EgoVehInfo.fegoVelocity_mps =
        extReqPorts->EgoVehInfo.fegoVelocity_mps;
    input->EgoVehInfo.fegoAcceleration_mps2 =
        extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
    // Road information
    input->Road.fC0Fused_1pm = extReqPorts->Road.fC0Fused_1pm;
    input->Road.fC1Fused_1pm2 = extReqPorts->Road.fC1Fused_1pm2;
    input->Road.fConfAdjacentLanes_per =
        extReqPorts->Road.fConfAdjacentLanes_per;
    input->Road.fConfOppositeLanes_per =
        extReqPorts->Road.fConfOppositeLanes_per;
    input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
    input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
    input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
    input->Road.fDrivenCurveRadius_met =
        extReqPorts->Road.fDrivenCurveRadius_met;
    input->Road.fLaneWidth_met = extReqPorts->Road.fLaneWidth_met;
    input->Road.fYawAngleFused_rad = extReqPorts->Road.fYawAngleFused_rad;
    input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
    input->Road.fYOffsetFusedOppBorder_met =
        extReqPorts->Road.fYOffsetFusedOppBorder_met;
    input->Road.iNumOfAdjacentLanes_nu =
        extReqPorts->Road.iNumOfAdjacentLanes_nu;
    input->Road.iNumOfOppositeLanes_nu =
        extReqPorts->Road.iNumOfOppositeLanes_nu;
    // LBS system parameters
    // input->SISysParam.bSIFunctionActive =
    // extReqPorts->LBSSystemParam.bSIFunctionActive;
    // input->SISysParam.bSIFunctionOutputActive =
    // extReqPorts->LBSSystemParam.bBSDFunctionOutputActive;
    input->SISysParam.fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;
    memset(&(input->GenObjList), 0, sizeof(SIGenObjList_st));
    memset(&(input->LBSObjInfoList), 0, sizeof(SILBSObjInfo_Array));
    memset(&(input->LCAObjInfoList), 0, sizeof(SILCAObjInfo_Array));

    // input->GenObjList.sSigHeader.eSigStatus_nu = 1u;
    for (uint8 uObjIndex = 0u; uObjIndex < LBS_INPUT_OBJECT_NUMBER;
         uObjIndex++) {
        if (extReqPorts->GenObjList.aObject[uObjIndex]
                .General.uiMaintenanceState_nu != EM_GEN_OBJ_MT_STATE_DELETED) {
            // EM general object list information
            input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor =
                extReqPorts->GenObjList.aObject[uObjIndex].bRightSensor;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.eClassification_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Attributes.eClassification_nu;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.eDynamicProperty_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Attributes.eDynamicProperty_nu;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistX_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Kinemactic.fDistX_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistY_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Kinemactic.fDistY_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Kinemactic.fVrelX_mps;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthLeft_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Geometry.fWidthLeft_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthRight_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Geometry.fWidthRight_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.uiLifeCycles_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .General.uiLifeCycles_nu;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.uiMaintenanceState_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .General.uiMaintenanceState_nu;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.fProbabilityOfExistence_per =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .Qualifiers.fProbabilityOfExistence_per;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDist2Course_met =
                pLBSCalc->RoadRelation[uObjIndex].fDist2Course_met;
            // extReqPorts->SRRObjList.aObject[uObjIndex]
            //     .RoadRelation.fDist2Course_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fFirstDetectX_met =
                pLBSCalc->LBSObjHistoryList[uObjIndex].fFirstDetectX_met;
            // extReqPorts->SRRObjList.aObject[uObjIndex]
            //     .History.fFirstDetectX_met;

            // LBS object information
            input->LBSObjInfoList[uObjIndex].fUpdateRate_nu =
                pLBSCalc->LBSObjInfoList[uObjIndex].fUpdateRate_nu;
            // LCA object information
            input->LCAObjInfoList[uObjIndex].bLCAWarning =
                pLBSCalc->LCAObjInfoList[uObjIndex].bLCAWarning;
        }
    }

    paramInput->VehParameter.fVehicleWidth_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
    // paramInput->LCAParamter.fDefaultLaneWidth =
    // extReqParam->LBSLCAParameter.fDefaultLaneWidth_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneXMid_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneXMid_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneXMin_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneXMin_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneYMaxFar_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMaxFar_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneYMaxNear_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMaxNear_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneYMinFar_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMinFar_met;
    paramInput->LCAParamter.LCAZone.fLCAZoneYMinNear_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMinNear_met;
    // Sensor mounting parameter
    paramInput->SensorMounting.SensorLeft.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorLeft.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorLeft.fRollAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_RollAngle_rad;
    paramInput->SensorMounting.SensorLeft.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_YawAngle_rad;

    paramInput->SensorMounting.SensorRight.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorRight.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorRight.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorRight.fRollAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_RollAngle_rad;
    paramInput->SensorMounting.SensorRight.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorRight.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_YawAngle_rad;
}

void LBSSIOutputWrapper(SIOutPut_st* pOutput,
                        SIDebug_st* pDebug,
                        LBSCalculate_st* pLBSCalc,
                        LBSDebug_t* extDebugPorts) {
    for (uint8 uObjIndex = 0u; uObjIndex < LBS_INPUT_OBJECT_NUMBER;
         uObjIndex++) {
        pLBSCalc->SIObjInfoList[uObjIndex].eAssociatedLane =
            pOutput->SIObjInfoList[uObjIndex].eAssociatedLane;
        pLBSCalc->SIObjInfoList[uObjIndex].fDistToTraj_met =
            pOutput->SIObjInfoList[uObjIndex].fDistToTraj_met;
        pLBSCalc->SIObjInfoList[uObjIndex].fObjBracketOverlap_met =
            pOutput->SIObjInfoList[uObjIndex].fObjBracketOverlap_met;
        pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketLeft_met =
            pOutput->SIObjInfoList[uObjIndex].fTraceBracketLeft_met;
        pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketRight_met =
            pOutput->SIObjInfoList[uObjIndex].fTraceBracketRight_met;
        pLBSCalc->SIObjInfoList[uObjIndex].fVrelToTraj_mps =
            pOutput->SIObjInfoList[uObjIndex].fVrelToTraj_mps;
    }
}

/*****************************************************************************
  BSD FUNCTION PROTOTYPES
*****************************************************************************/
void BSDInputWrapper(const LBSInReq_st* extReqPorts,
                     BSDInReq_st* input,
                     const LBSParam_st* extReqParam,
                     BSDParam_st* paramInput) {
    const LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();
    const LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
    const LBSBSDParameter_t* pBSDParam = &extReqParam->LBS_Ks_BSDParameter_nu;
    uint8 uObj;
    memset(&(input->GenObjList), 0, sizeof(BSDGenObjList_st));
    memset(&(input->SRRObjList), 0, sizeof(BSDSRRObjList_st));
    memset(&(input->LBSInputInfo), 0, sizeof(BSD_LBSInputInfo_st));
    // set Object List input
    for (uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        if (extReqPorts->GenObjList.aObject[uObj]
                .General.uiMaintenanceState_nu != EM_GEN_OBJ_MT_STATE_DELETED) {
            // set General Object List input
            input->GenObjList.aObject[uObj].Attributes.eClassification_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.eClassification_nu;
            input->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.eDynamicProperty_nu;
            input->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.uiClassConfidence_per;
            input->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.uiDynConfidence_per;

            input->GenObjList.aObject[uObj].General.fLifeTime_s =
                extReqPorts->GenObjList.aObject[uObj].General.fLifeTime_s;
            input->GenObjList.aObject[uObj].General.uiID_nu =
                extReqPorts->GenObjList.aObject[uObj].General.uiID_nu;
            input->GenObjList.aObject[uObj].General.uiLifeCycles_nu =
                extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
            input->GenObjList.aObject[uObj].General.uiMaintenanceState_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .General.uiMaintenanceState_nu;

            input->GenObjList.aObject[uObj].Geometry.fLength_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLength_met;
            input->GenObjList.aObject[uObj].Geometry.fLengthFront_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
            input->GenObjList.aObject[uObj].Geometry.fLengthRear_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
            input->GenObjList.aObject[uObj].Geometry.fWidth_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
            input->GenObjList.aObject[uObj].Geometry.fWidthLeft_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
            input->GenObjList.aObject[uObj].Geometry.fWidthRight_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;

            input->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss;
            input->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss;
            input->GenObjList.aObject[uObj].Kinemactic.fDistX_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
            input->GenObjList.aObject[uObj].Kinemactic.fDistY_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
            input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
            input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;

            input->GenObjList.aObject[uObj].bRightSensor =
                extReqPorts->GenObjList.aObject[uObj].bRightSensor;

            // set SRR Object List input
            input->SRRObjList.aObject[uObj].History.fFirstDetectX_met =
                pLBSCalculate->LBSObjHistoryList[uObj].fFirstDetectX_met;
            // extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectX_met;
            input->SRRObjList.aObject[uObj].History.fFirstDetectY_met =
                pLBSCalculate->LBSObjHistoryList[uObj].fFirstDetectY_met;
            // extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectY_met;

            input->SRRObjList.aObject[uObj].Qualifiers.bObjStable =
                extReqPorts->SRRObjList.aObject[uObj].Qualifiers.bObjStable;
            input->SRRObjList.aObject[uObj]
                .Qualifiers.fProbabilityOfExistence_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.fProbabilityOfExistence_per;
            input->SRRObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.uiHighestAssocProb_per;
            input->SRRObjList.aObject[uObj]
                .Qualifiers.uiMeasuredTargetFrequency_nu =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.uiMeasuredTargetFrequency_nu;

            input->SRRObjList.aObject[uObj].RoadRelation.bDist2BorderValid =
                pLBSCalculate->RoadRelation[uObj].bDist2BorderValid;
            // extReqPorts->SRRObjList.aObject[uObj]
            //     .RoadRelation.bDist2BorderValid;
            input->SRRObjList.aObject[uObj].RoadRelation.fDist2Border_met =
                pLBSCalculate->RoadRelation[uObj].fDist2Border_met;
            // extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Border_met;
            input->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met =
                pLBSCalculate->RoadRelation[uObj].fDist2Course_met;
            // extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met;

            input->LBSInputInfo.LBSObjInfoList[uObj].fAngle_deg =
                pLBSCalculate->LBSObjInfoList[uObj].fAngle_deg;
            input->LBSInputInfo.LBSObjInfoList[uObj].fAssocProbFiltered =
                pLBSCalculate->LBSObjInfoList[uObj].fAssocProbFiltered;
            input->LBSInputInfo.LBSObjInfoList[uObj].fCycletimeSum_s =
                pLBSCalculate->LBSObjInfoList[uObj].fCycletimeSum_s;
            input->LBSInputInfo.LBSObjInfoList[uObj].fTTC_s =
                pLBSCalculate->LBSObjInfoList[uObj].fTTC_s;
            input->LBSInputInfo.LBSObjInfoList[uObj].fTTCFiltered_s =
                pLBSCalculate->LBSObjInfoList[uObj].fTTCFiltered_s;
            input->LBSInputInfo.LBSObjInfoList[uObj].fUpdateRate_nu =
                pLBSCalculate->LBSObjInfoList[uObj].fUpdateRate_nu;
            input->LBSInputInfo.LBSObjInfoList[uObj].fXMovement_met =
                pLBSCalculate->LBSObjInfoList[uObj].fXMovement_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].fYMovement_met =
                pLBSCalculate->LBSObjInfoList[uObj].fYMovement_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmax_met =
                pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmin_met =
                pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmax_met =
                pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmin_met =
                pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmin_met;

            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorFrontObject =
                pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorFrontObject;
            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorObject =
                pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorObject;
            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAWarning =
                pLBSCalculate->LCAObjInfoList[uObj].bLCAWarning;
        }
    }

    // set Vehicle information input
    input->EgoVehInfo.fegoVelocity_mps =
        extReqPorts->EgoVehInfo.fegoVelocity_mps;

    // set Road information input
    input->Road.BorderEstmGridData_fConf_per =
        extReqPorts->Road.BorderEstmGridData_fConf_per;
    input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
    input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
    input->Road.fDrivenCurveRadius_met =
        extReqPorts->Road.fDrivenCurveRadius_met;
    input->Road.fLaneWidth_met = extReqPorts->Road.fLaneWidth_met;
    input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
    input->Road.iNumOfAdjacentLanes_nu =
        extReqPorts->Road.iNumOfAdjacentLanes_nu;
    input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
    input->Road.fYOffsetFusedOppBorder_met =
        extReqPorts->Road.fYOffsetFusedOppBorder_met;

    // set LBS and LCA object information input
    // for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fAngle_deg =
    //         pLBSCalculate->LBSObjInfoList[uObj].fAngle_deg;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fAssocProbFiltered =
    //         pLBSCalculate->LBSObjInfoList[uObj].fAssocProbFiltered;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fCycletimeSum_s =
    //         pLBSCalculate->LBSObjInfoList[uObj].fCycletimeSum_s;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fTTC_s =
    //         pLBSCalculate->LBSObjInfoList[uObj].fTTC_s;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fTTCFiltered_s =
    //         pLBSCalculate->LBSObjInfoList[uObj].fTTCFiltered_s;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fUpdateRate_nu =
    //         pLBSCalculate->LBSObjInfoList[uObj].fUpdateRate_nu;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fXMovement_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].fXMovement_met;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].fYMovement_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].fYMovement_met;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmax_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmin_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmax_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
    //     input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmin_met =
    //         pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmin_met;

    //     input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorFrontObject =
    //         pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorFrontObject;
    //     input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorObject =
    //         pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorObject;
    //     input->LBSInputInfo.LCAObjInfoList[uObj].bLCAWarning =
    //         pLBSCalculate->LCAObjInfoList[uObj].bLCAWarning;
    // }

    // set LBS Global input
    input->LBSInputInfo.LBSGlobalInfo.bInnerSensorDriven =
        pLBSGlobals->bInnerSensorDriven;
    input->LBSInputInfo.LBSGlobalInfo.bInnerSensorSteering =
        pLBSGlobals->bInnerSensorSteering;
    input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToRear_met =
        pLBSGlobals->fSensorOffsetToRear_met;
    // input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToSide_met =
    // pLBSGlobals->fSensorOffetToSide_met;

    // set function enable switch input
    input->BSDSystemParam.bBSDFunctionActive =
        extReqPorts->LBSSystemParam.bBSDFunctionActive;
    input->BSDSystemParam.bBSDFunctionOutputActive =
        extReqPorts->LBSSystemParam.bBSDFunctionOutputActive;
    input->BSDSystemParam.fCycletime_s =
        extReqPorts->LBSSystemParam.fCycletime_s;

    // input->BSDPreProcessInput.BSDHmiOpen =
    // extReqPorts->LBS_Ns_NVRAM_nu.LBS_Nb_BSDPowerOffSwitchState_nu;
    input->BSDPreProcessInput.BSDFailure =
        extReqPorts->LBSSupportInfo.LBSBSDFailure;

    // set parameter input
    // vehicle parameter
    paramInput->BSDVehParameter.fVehicleLength_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
    paramInput->BSDVehParameter.fVehicleWidth_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
    paramInput->BSDVehParameter.fWheelBase_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_WheelBase_met;
    paramInput->BSDVehParameter.fVehCenter2FrontAxis_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met;
    paramInput->BSDVehParameter.fVehRear2FrontAxis_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met +
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met * 0.5f;

    // BSD warning parameter and zone area parameter
    memcpy(&paramInput->BSDWarningParameter,
           &pBSDParam->LBS_Ks_BSDWarnParameter_nu,
           sizeof(BSDWarningParameter_t));

    memcpy(&paramInput->BsdZone, &pBSDParam->LBS_Ks_BSDZoneParameter_nu,
           sizeof(BsdZone_t));

    // Sensor mounting parameter
    paramInput->SensorMounting.SensorLeft.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorLeft.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorLeft.fRollAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_RollAngle_rad;
    paramInput->SensorMounting.SensorLeft.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_YawAngle_rad;

    paramInput->SensorMounting.SensorRight.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorRight.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorRight.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorRight.fRollAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_RollAngle_rad;
    paramInput->SensorMounting.SensorRight.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorRight.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_YawAngle_rad;
}

void BSDOutputWrapper(LBSOutPro_t* extProPorts,
                      LBSDebug_t* extDebugPorts,
                      BSDOutPro_st* pOutput,
                      BSDDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc) {
    // BSD debug output
    // memcpy(&extDebugPorts->BSDOutput, pOutput, sizeof(BSDOutPro_st));
    // memcpy(&extDebugPorts->BSDDebug.BSDObjInfo, pDebug->BSDObjInfo,
    //        sizeof(LBSBSDInfoArrayDebug));
    // memcpy(&extDebugPorts->BSDDebug.LBS_BSDWarnDecideList_Debug,
    //        pDebug->BSDWarnDecideList_Debug,
    //        sizeof(LBS_BSD_Warn_Decide_Debug_Array));
    // printf("LBSDEBUG IS %d\t, BSDDEBUG IS %d\n",
    // extDebugPorts->BSDDebug.BSDObjInfo[0].bInBSDZone,
    // pDebug->BSDObjInfo[0].bInBSDZone);

    // set BSD output for LBS interface
    pLBSCalc->LBSWarnLastCycle.bBSDWarningLeftLastCycle =
        pOutput->BSD_Globals.bBSDWarnActiveLeftLastCycle;
    pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle =
        pOutput->BSD_Globals.bBSDWarnActiveRightLastCycle;
    pLBSCalc->LBSBSDCalc.bBSDWarnActiveLeft =
        pOutput->BSD_Globals.bBSDWarnActiveLeft;
    pLBSCalc->LBSBSDCalc.bBSDWarnActiveRight =
        pOutput->BSD_Globals.bBSDWarnActiveRight;
    pLBSCalc->LBSBSDCalc.uBSDWarnActiveLeftID =
        pOutput->BSD_Globals.bBSDWarnActiveLeftID_nu;
    pLBSCalc->LBSBSDCalc.uBSDWarnActiveRightID =
        pOutput->BSD_Globals.bBSDWarnActiveRightID_nu;

    for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        pLBSCalc->BSDObjInfoList[uObj].fBSDZoneXMin_met =
            pOutput->BSDOutputInfoList[uObj].fZoneXmin_met;
        pLBSCalc->BSDObjInfoList[uObj].bBSDInterruptRCW =
            pOutput->BSDOutputInfoList[uObj].bBSDWarning;
    }
}
/*****************************************************************************
  LCA FUNCTION PROTOTYPES
*****************************************************************************/
void LBSLCAInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam,
                        LCAInReq_st* input,
                        LCAParam_st* paramInput,
                        LBSCalculate_st* pLBSCalc) {
    float32 fDistX_met;
    float32 fDistY_met;
    // Ego vehicle information
    input->EgoVehInfo.fegoVelocity_mps =
        extReqPorts->EgoVehInfo.fegoVelocity_mps;
    input->EgoVehInfo.fegoAcceleration_mps2 =
        extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
    // Road information
    input->Road.fConfAdjacentLanes_per =
        extReqPorts->Road.fConfAdjacentLanes_per;
    input->Road.fConfOppositeLanes_per =
        extReqPorts->Road.fConfOppositeLanes_per;
    input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
    input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
    input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
    input->Road.fDrivenCurveRadius_met =
        extReqPorts->Road.fDrivenCurveRadius_met;
    input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
    input->Road.fYOffsetFusedOppBorder_met =
        extReqPorts->Road.fYOffsetFusedOppBorder_met;
    input->Road.iNumOfAdjacentLanes_nu =
        extReqPorts->Road.iNumOfAdjacentLanes_nu;
    input->Road.iNumOfOppositeLanes_nu =
        extReqPorts->Road.iNumOfOppositeLanes_nu;
    // LBS system parameters
    input->LCASystemParam.bLCAFunctionActive =
        extReqPorts->LBSSystemParam.bLCAFunctionActive;
    input->LCASystemParam.bLCAFunctionOutputActive =
        extReqPorts->LBSSystemParam.bLCAFunctionOutputActive;
    input->LCASystemParam.fCycletime_s =
        extReqPorts->LBSSystemParam.fCycletime_s;

    input->LCAPreProcessInput.ActGearValid =
        extReqPorts->LBSSupportInfo.ActGearValid;
    input->LCAPreProcessInput.uTurnLightReqSt =
        extReqPorts->LBSSupportInfo.uTurnLightReqSt;
    input->LCAPreProcessInput.GearInReverseAndParking =
        extReqPorts->LBSSupportInfo.GearInReverseAndParking;
    input->LCAPreProcessInput.VehicleSpdDisplayValid =
        extReqPorts->LBSSupportInfo.VehicleSpdDisplayValid;

    // Calculate information from LBS layer
    if (pLBSCalc->LBSWarnLastCycle.bBSDWarningLeftLastCycle ||
        pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle) {
        input->LBSInputInfo.LBSWarningLastCycle.bBSDWarningLastCycle = TRUE;
    }
    input->LBSInputInfo.LBSWarningLastCycle.bLCAWarningLastCycle =
        pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle;
    input->LCALBSGlobalInput.fMaxSpeedOverGround_mps =
        pLBSCalc->LBS_Globals.fMaxSpeedOverGround_mps;

    memset(&(input->GenObjList), 0, sizeof(LCAGenObjList_st));
    memset(&(input->LBSInputInfo), 0, sizeof(LCALBSInputInfo_st));

    for (uint8 uObjIndex = 0u; uObjIndex < LBS_INPUT_OBJECT_NUMBER;
         uObjIndex++) {
        if (extReqPorts->GenObjList.aObject[uObjIndex]
                .General.uiMaintenanceState_nu != EM_GEN_OBJ_MT_STATE_DELETED) {
            // EM general object list information
            input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor =
                extReqPorts->GenObjList.aObject[uObjIndex].bRightSensor;
            if (!input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor) {
                fDistX_met = extReqPorts->GenObjList.aObject[uObjIndex]
                                 .Kinemactic.fDistX_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorLeft_nu.LBS_Kf_LongPos_met;
                fDistY_met = extReqPorts->GenObjList.aObject[uObjIndex]
                                 .Kinemactic.fDistY_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorLeft_nu.LBS_Kf_LatPos_met;
            } else {
                fDistX_met = extReqPorts->GenObjList.aObject[uObjIndex]
                                 .Kinemactic.fDistX_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorRight_nu.LBS_Kf_LongPos_met;
                fDistY_met = extReqPorts->GenObjList.aObject[uObjIndex]
                                 .Kinemactic.fDistY_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorRight_nu.LBS_Kf_LatPos_met;
            }
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistX_met =
                fDistX_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistY_met =
                fDistY_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Kinemactic.fVrelX_mps;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fLengthFront_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Geometry.fLengthFront_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthLeft_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Geometry.fWidthLeft_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthRight_met =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .Geometry.fWidthRight_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.uiLifeCycles_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .General.uiLifeCycles_nu;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.uiMaintenanceState_nu =
                extReqPorts->GenObjList.aObject[uObjIndex]
                    .General.uiMaintenanceState_nu;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fDist2Border_met =
                pLBSCalc->RoadRelation[uObjIndex].fDist2Border_met;
            // extReqPorts->SRRObjList.aObject[uObjIndex]
            //     .RoadRelation.fDist2Border_met;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.bDist2BorderValid =
                pLBSCalc->RoadRelation[uObjIndex].bDist2BorderValid;
            // extReqPorts->SRRObjList.aObject[uObjIndex]
            //     .RoadRelation.bDist2BorderValid;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fMirrorProb_per =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .SensorSpecific.fMirrorProb_per;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.fRCS =
                extReqPorts->SRRObjList.aObject[uObjIndex].SensorSpecific.fRCS;
            input->GenObjList.aObject[uObjIndex].GenObjInfo.bObjStable =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .Qualifiers.bObjStable;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.fProbabilityOfExistence_per =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .Qualifiers.fProbabilityOfExistence_per;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.uiHighestAssocProb_per =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .Qualifiers.uiHighestAssocProb_per;
            input->GenObjList.aObject[uObjIndex]
                .GenObjInfo.uiMeasuredTargetFrequency_nu =
                extReqPorts->SRRObjList.aObject[uObjIndex]
                    .Qualifiers.uiMeasuredTargetFrequency_nu;
            // LBS SI object information
            input->LBSInputInfo.SIObjInfoList[uObjIndex].eAssociatedLane =
                pLBSCalc->SIObjInfoList[uObjIndex].eAssociatedLane;
            input->LBSInputInfo.SIObjInfoList[uObjIndex].fDistToTraj_met =
                pLBSCalc->SIObjInfoList[uObjIndex].fDistToTraj_met;
            input->LBSInputInfo.SIObjInfoList[uObjIndex].fVrelToTraj_mps =
                pLBSCalc->SIObjInfoList[uObjIndex].fVrelToTraj_mps;
            input->LBSInputInfo.SIObjInfoList[uObjIndex]
                .fObjBracketOverlap_met =
                pLBSCalc->SIObjInfoList[uObjIndex].fObjBracketOverlap_met;
            input->LBSInputInfo.SIObjInfoList[uObjIndex].fTraceBracketLeft_met =
                pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketLeft_met;
            input->LBSInputInfo.SIObjInfoList[uObjIndex]
                .fTraceBracketRight_met =
                pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketRight_met;
            // LBS BSD object information
            input->LBSInputInfo.BSDObjInfoList[uObjIndex].fBSDZoneXMin_met =
                pLBSCalc->BSDObjInfoList[uObjIndex].fBSDZoneXMin_met;
            input->LBSInputInfo.LBSObjInfoList[uObjIndex].fUpdateRate_nu =
                pLBSCalc->LBSObjInfoList[uObjIndex].fUpdateRate_nu;
            input->LBSInputInfo.LBSObjInfoList[uObjIndex].fXMovement_met =
                pLBSCalc->LBSObjInfoList[uObjIndex].fXMovement_met;
            // input->LBSInputInfo.LBSObjInfoList[uObjIndex].bCreateAdjStableObj
            // =
            //     pLBSCalc->LBSObjInfoList[uObjIndex].bCreateAdjStableObj;
            // input->LBSInputInfo.LBSObjInfoList[uObjIndex].bLowTTCAtStart =
            //     pLBSCalc->LBSObjInfoList[uObjIndex].bLowTTCAtStart;
            input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTC_s =
                pLBSCalc->LBSObjInfoList[uObjIndex].fTTC_s;
            input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTCAccel_mps2 =
                pLBSCalc->LBSObjInfoList[uObjIndex].fTTCAccel_mps2;
            input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTCFiltered_s =
                pLBSCalc->LBSObjInfoList[uObjIndex].fTTCFiltered_s;
        }
    }

    paramInput->fBridgeWarningTime_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCABridgeWarningTime_s;
    paramInput->fMaxLCACurveRadius_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCACurveRadius_met;
    paramInput->fMaxLCARange_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCARange_met;
    paramInput->fMinTTCHysteresis_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMinTTCHysteresis_s;
    paramInput->fTTCThreshHighRelSpeed_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshHighRelSpeed_s;
    paramInput->fTTCThreshLowRelSpeed_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshLowRelSpeed_s;
    paramInput->fTTCThreshMidRelSpeed_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshMidRelSpeed_s;
    paramInput->fTTCThreshold_s =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshold_s;
    paramInput->fVehicleWidth_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVehicleWidth_met;
    paramInput->uLCAWarningDuration =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ku_LCAWarningDuration_nu;
    paramInput->LCAZone.fLCAZoneXMid_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneXMid_met;
    paramInput->LCAZone.fLCAZoneXMin_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneXMin_met;
    paramInput->LCAZone.fLCAZoneYMaxFar_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMaxFar_met;
    paramInput->LCAZone.fLCAZoneYMaxNear_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMaxNear_met;
    paramInput->LCAZone.fLCAZoneYMinFar_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMinFar_met;
    paramInput->LCAZone.fLCAZoneYMinNear_met =
        extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
            .LBS_Kf_LCAZoneYMinNear_met;
}

void LBSLCAOutputWrapper(LCAOutPro_st* pOutput,
                         LCADebug_t* pDebug,
                         LBSCalculate_st* pLBSCalc,
                         LBSDebug_t* extDebugPorts) {
    // pLBSCalc->LBSLCACalc.bLCAWarnActive = pOutput->bLCAWarnActive;
    pLBSCalc->LBSLCACalc.bLCAWarnActive =
        (pOutput->LCAStateMachineOutput == LCAState_Active);
    pLBSCalc->LBSLCACalc.bLCAWarnActiveLeft =
        pOutput->LCACanSignalOutput.ADCS8_LCALeftWarnSt;
    pLBSCalc->LBSLCACalc.bLCAWarnActiveRight =
        pOutput->LCACanSignalOutput.ADCS8_LCARightWarnSt;
    pLBSCalc->LBSLCACalc.uLCAWarningID_nu = pOutput->uLCAWarningID_nu;
    pLBSCalc->LBSLCACalc.fXObjectWarning_met = pOutput->fXObjectWarning_met;
    pLBSCalc->LBSLCACalc.fCriticalTTC_s = pOutput->fCriticalTTC_s;
    pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle =
        pOutput->bLCAWarningLastCycle;

    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorFrontObject =
            pOutput->LCAObjOutputList[uObj].bLCAMirrorFrontObject;
        pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorObject =
            pOutput->LCAObjOutputList[uObj].bLCAMirrorObject;
        pLBSCalc->LCAObjInfoList[uObj].bLCAWarning =
            pOutput->LCAObjOutputList[uObj].bLCAWarning;
        pLBSCalc->LBSObjInfoList[uObj].bLowTTCAtStart =
            pOutput->LCAObjOutputList[uObj].bLowTTCAtStart;
        pLBSCalc->LBSObjInfoList[uObj].bCreateAdjStableObj =
            pOutput->LCAObjOutputList[uObj].bCreateAdjStableObj;

        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bInLCARange =
        // pOutput->LCAObjOutputList[uObj].bInLCARange;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCALaneConditions =
        // pOutput->LCAObjOutputList[uObj].bLCALaneConditions;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAMirrorFrontObject
        // = pOutput->LCAObjOutputList[uObj].bLCAMirrorFrontObject;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAMirrorObject =
        // pOutput->LCAObjOutputList[uObj].bLCAMirrorObject;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAObjPathInvalid =
        // pOutput->LCAObjOutputList[uObj].bLCAObjPathInvalid;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAQuality =
        // pOutput->LCAObjOutputList[uObj].bLCAQuality;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCARelevant =
        // pOutput->LCAObjOutputList[uObj].bLCARelevant;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAWarning =
        // pOutput->LCAObjOutputList[uObj].bLCAWarning;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bLCAWarningConditions
        // = pOutput->LCAObjOutputList[uObj].bLCAWarningConditions;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].bUpdateRecently =
        // pOutput->LCAObjOutputList[uObj].bUpdateRecently;
        // extDebugPorts->LCADebug.LCAObjOutputList[uObj].fBehindGrdProb_per =
        // pOutput->LCAObjOutputList[uObj].fBehindGrdProb_per;
    }
    // memcpy(&extDebugPorts->LCADebug.LCAObjOutputList,
    //        &pOutput->LCAObjOutputList, sizeof(LBSLCAObjInfoArrayDebug));
    // memcpy(&extDebugPorts->LCADebug.LBS_LCAWarnDecideList_Debug,
    //        &pDebug->LCAWarnDecideList_Debug,
    //        sizeof(LBS_LCA_Warn_Decide_Debug_Array));
    // memcpy(&extDebugPorts->LCADebug.LCAWarnInfo, &pDebug->LCAWarnInfo,
    //        sizeof(LBS_LCAWarnInfo_t));
    // memcpy(&extDebugPorts->LCADebug.LCAConfig, &pDebug->LCAConfig,
    //        sizeof(LBS_LCAConfig_t));
    // memcpy(&extDebugPorts->LCADebug.LCAFrontMirror, &pDebug->LCAFrontMirror,
    //        sizeof(LBS_LCAFrontMirror_t));
    extDebugPorts->LCADebug.bLCAPathBlockedLeft = pDebug->bLCAPathBlockedLeft;
    extDebugPorts->LCADebug.bLCAPathBlockedRight = pDebug->bLCAPathBlockedRight;
    extDebugPorts->LCADebug.uCntLCAPathBlockedLeft =
        pDebug->uCntLCAPathBlockedLeft;
    extDebugPorts->LCADebug.uCntLCAPathBlockedRight =
        pDebug->uCntLCAPathBlockedRight;
    extDebugPorts->LCADebug.fLCARange = pDebug->fLCARange;
    extDebugPorts->LCADebug.uLCAWarningID_nu = pDebug->uLCAWarningID_nu;
}

/*****************************************************************************
  OSE FUNCTION PROTOTYPES
*****************************************************************************/
void LBSOSEInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam,
                        OSEInReq_t* input,
                        OSEParam_t* paramInput,
                        LBSCalculate_st* pLBSCalc) {
    // extReqPorts->input
    memset(&(input->EMGenObjList), 0, sizeof(OSEEMGenObjListInReq_t));
    memset(&(input->OSELBSGlobalInReq), 0, sizeof(OSELBSGlobalInReq_t));

    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        if (extReqPorts->GenObjList.aObject[uObj]
                .General.uiMaintenanceState_nu != EM_GEN_OBJ_MT_STATE_DELETED) {
            // EM input information
            input->EMGenObjList.aObject[uObj].bRightSensor =
                extReqPorts->GenObjList.aObject[uObj].bRightSensor;
            // EM front asix to SRR location
            float32 fDistX_met, fDistY_met;
            if (!input->EMGenObjList.aObject[uObj].bRightSensor) {
                fDistX_met = extReqPorts->GenObjList.aObject[uObj]
                                 .Kinemactic.fDistX_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorLeft_nu.LBS_Kf_LongPos_met;
                fDistY_met = extReqPorts->GenObjList.aObject[uObj]
                                 .Kinemactic.fDistY_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorLeft_nu.LBS_Kf_LatPos_met;
            } else {
                fDistX_met = extReqPorts->GenObjList.aObject[uObj]
                                 .Kinemactic.fDistX_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorRight_nu.LBS_Kf_LongPos_met;
                fDistY_met = extReqPorts->GenObjList.aObject[uObj]
                                 .Kinemactic.fDistY_met -
                             extReqParam->LBS_Ks_SensorMounting_nu
                                 .LBS_Kf_SensorRight_nu.LBS_Kf_LatPos_met;
            }
            input->EMGenObjList.aObject[uObj].eMaintenanceState =
                extReqPorts->GenObjList.aObject[uObj]
                    .General.uiMaintenanceState_nu;
            input->EMGenObjList.aObject[uObj].fAbsOrientationStd_rad =
                extReqPorts->GenObjList.aObject[uObj]
                    .Geometry.fAbsOrientationStd_rad;
            input->EMGenObjList.aObject[uObj].fAbsOrientation_rad =
                extReqPorts->GenObjList.aObject[uObj]
                    .Geometry.fAbsOrientation_rad;
            input->EMGenObjList.aObject[uObj].fDistXStd_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistXStd_met;
            input->EMGenObjList.aObject[uObj].fDistX_met = fDistX_met;
            // extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
            input->EMGenObjList.aObject[uObj].fDistYStd_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistYStd_met;
            input->EMGenObjList.aObject[uObj].fDistY_met = fDistY_met;
            // extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
            input->EMGenObjList.aObject[uObj].fFirstDetectX_met =
                pLBSCalc->LBSObjHistoryList[uObj].fFirstDetectX_met;
            // extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectX_met;
            input->EMGenObjList.aObject[uObj].fLengthFront_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
            input->EMGenObjList.aObject[uObj].fLengthRear_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
            input->EMGenObjList.aObject[uObj].fMirrorProb_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .SensorSpecific.fMirrorProb_per;
            input->EMGenObjList.aObject[uObj].fProbabilityOfExistence_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.fProbabilityOfExistence_per;
            input->EMGenObjList.aObject[uObj].fRCS =
                extReqPorts->SRRObjList.aObject[uObj].SensorSpecific.fRCS;
            input->EMGenObjList.aObject[uObj].fVabsX_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsX_mps;
            input->EMGenObjList.aObject[uObj].fVabsY_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsY_mps;
            input->EMGenObjList.aObject[uObj].fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
            input->EMGenObjList.aObject[uObj].fVrelY_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps;
            input->EMGenObjList.aObject[uObj].fWidthLeft_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
            input->EMGenObjList.aObject[uObj].fWidthRight_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;
            input->EMGenObjList.aObject[uObj].fWidth_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
            input->EMGenObjList.aObject[uObj].uiLifeCycles_nu =
                extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
            input->EMGenObjList.aObject[uObj].uiMeasuredTargetFrequency_nu =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.uiMeasuredTargetFrequency_nu;
            // LBS object information
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                .fAssocProbFiltered =
                pLBSCalc->LBSObjInfoList[uObj].fAssocProbFiltered;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fUpdateRate_nu =
                pLBSCalc->LBSObjInfoList[uObj].fUpdateRate_nu;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fXMovement_met =
                pLBSCalc->LBSObjInfoList[uObj].fXMovement_met;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fYMovement_met =
                pLBSCalc->LBSObjInfoList[uObj].fYMovement_met;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                .ObjBorders.fXmax_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                .ObjBorders.fXmin_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                .ObjBorders.fYmax_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
            input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                .ObjBorders.fYmin_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmin_met;
        }
    }
    input->fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;
    input->OSEFunctionSwitch.bOSEFunctionActive =
        extReqPorts->LBSSystemParam.bOSEFunctionActive;
    input->OSEFunctionSwitch.bOSEPowermode3min =
        extReqPorts->LBSSupportInfo.bDOW_PowerMode_3Mins;

    // extReqParam->paramInput
    paramInput->bActive =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kb_OSEActive_nu;
    // paramInput->bEnableObjAdaptiveBreakthrough =
    // extReqParam->LBSOSEParameter.bEnableObjAdaptiveBreakthrough;
    paramInput->fMaxHeadingAngle =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMaxHeadingAngle_deg;
    paramInput->fMaxTime_s[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[0u];
    paramInput->fMaxTime_s[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[1u];
    paramInput->fMaxTime_s[2u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[2u];
    paramInput->fMinHeadingAngle =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMinHeadingAngle_deg;
    paramInput->fMinTime_s[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[0u];
    paramInput->fMinTime_s[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[1u];
    paramInput->fMinTime_s[2u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[2u];
    paramInput->fTargetRangeMax_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[0u];
    paramInput->fTargetRangeMax_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[1u];
    paramInput->fTargetRangeMax_met[2u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[2u];
    paramInput->fTTCThreshold_s[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[0u];
    paramInput->fTTCThreshold_s[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[1u];
    paramInput->fTTCThreshold_s[2u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[2u];
    paramInput->fVEgoMax_mps =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps;
    paramInput->fVEgoMin_mps =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps;
    paramInput->fVTargetMax_mps =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMax_mps;
    paramInput->fVTargetMin_mps =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMin_mps;
    paramInput->fXBreakthroughLine_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[0u];
    paramInput->fXBreakthroughLine_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[1u];
    paramInput->fYMaxBreakthroughMargin_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu
            .LBS_Ka_OSEYMaxBreakthroughMargin_met[0u];
    paramInput->fYMaxBreakthroughMargin_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu
            .LBS_Ka_OSEYMaxBreakthroughMargin_met[1u];
    paramInput->fYMaxBreakthrough_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[0u];
    paramInput->fYMaxBreakthrough_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[1u];
    paramInput->fYMinBreakthroughMargin_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu
            .LBS_Ka_OSEYMinBreakthroughMargin_met[0u];
    paramInput->fYMinBreakthroughMargin_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu
            .LBS_Ka_OSEYMinBreakthroughMargin_met[1u];
    paramInput->fYMinBreakthrough_met[0u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[0u];
    paramInput->fYMinBreakthrough_met[1u] =
        extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[1u];
    paramInput->SensorMounting.SensorLeft.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorRight.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorRight.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPos_met;
    paramInput->VehParAdd.fOverhangFront_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_OverhangFront_met;
    paramInput->VehParAdd.fVehicleLength_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
    paramInput->VehParAdd.fVehicleWidth_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
}

void LBSOSEOutputWrapper(OSEOutPro_t* pOutput,
                         OSEDebug_t* OSEDebugInfo,
                         LBSCalculate_st* pLBSCalc,
                         LBSDebug_t* debugInfo) {
    // pOutput -> pLBSCalc
    for (uint8 uWarnLevel = 0u; uWarnLevel < LBS_OSE_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
        pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel] =
            pOutput->bOSEWarnActive[uWarnLevel];
    }
    pLBSCalc->LBSOSECalc.bWarningInterrupt = pOutput->bWarningInterrupt;
    pLBSCalc->LBSOSECalc.fCriticalTTC = pOutput->fCriticalTTC;
    pLBSCalc->LBSOSECalc.uCriticalObjID = pOutput->uCriticalObjID;
    pLBSCalc->LBSOSECalc.bDOWPowerModeDone = pOutput->bDOWPowerModeDone;
    // memcpy(&pLBSCalc->OSEObjInfoList, &pOutput->OSEObjInfoArray,
    //        sizeof(LBSOSEObjInfo_Array));

    // pOutput->OSEDebug
    // memcpy(&debugInfo->OSEDebug.OSEObjInfoArray, &pOutput->OSEObjInfoArray,
    //        sizeof(LBSOSEObjInfoArrayDebug_Array));
    // memcpy(&debugInfo->OSEDebug.OSEWarnDecideDebug,
    //        &OSEDebugInfo->OSEWarnDecideList,
    //        sizeof(LBS_OSE_Warn_Decide_Debug_Array));
}

/*****************************************************************************
  RCW FUNCTION PROTOTYPES
*****************************************************************************/
void RCWInputWrapper(const LBSInReq_st* extReqPorts,
                     RCWInReq_st* input,
                     const LBSParam_st* extReqParam,
                     RCWParam_st* paramInput,
                     LBSCalculate_st* pLBSCalc) {
    // RCWInReq_st
    static uint32 LBSRCW_InputObj_Counter = 0;
    const LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
    memset(&(input->GenObjList), 0, sizeof(RCWGenObjList_st));
    memset(&(input->LBSInputInfo), 0, sizeof(RCW_LBSInputInfo_st));
    for (uint8 uObj = 0u; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
        if (extReqPorts->GenObjList.aObject[uObj]
                .General.uiMaintenanceState_nu != EM_GEN_OBJ_MT_STATE_DELETED) {
            LBSRCW_InputObj_Counter = uObj;
            // input->GenObjList.aObject[uObj].ObjID =
            //     extReqPorts->GenObjList.aObject[uObj].ObjID;
            input->GenObjList.aObject[uObj].bRightSensor =
                extReqPorts->GenObjList.aObject[uObj].bRightSensor;
            input->GenObjList.aObject[uObj].Kinemactic.fDistX_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
            input->GenObjList.aObject[uObj].Kinemactic.fDistY_met =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
            input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
            input->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps;
            input->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss;
            input->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss =
                extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss;
            input->GenObjList.aObject[uObj].Geometry.fWidth_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
            input->GenObjList.aObject[uObj].Geometry.fWidthLeft_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
            input->GenObjList.aObject[uObj].Geometry.fWidthRight_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;
            input->GenObjList.aObject[uObj].Geometry.fLength_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLength_met;
            input->GenObjList.aObject[uObj].Geometry.fLengthFront_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
            input->GenObjList.aObject[uObj].Geometry.fLengthRear_met =
                extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
            input->GenObjList.aObject[uObj].Geometry.fAbsOrientation_rad =
                extReqPorts->GenObjList.aObject[uObj]
                    .Geometry.fAbsOrientation_rad;
            input->GenObjList.aObject[uObj].General.fLifeTime_s =
                extReqPorts->GenObjList.aObject[uObj].General.fLifeTime_s;
            input->GenObjList.aObject[uObj].General.uiLifeCycles_nu =
                extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
            input->GenObjList.aObject[uObj].General.uiMaintenanceState_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .General.uiMaintenanceState_nu;
            input->GenObjList.aObject[uObj].General.uiID_nu =
                extReqPorts->GenObjList.aObject[uObj].General.uiID_nu;
            input->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.eDynamicProperty_nu;
            input->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.uiDynConfidence_per;
            input->GenObjList.aObject[uObj].Attributes.eClassification_nu =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.eClassification_nu;
            input->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per =
                extReqPorts->GenObjList.aObject[uObj]
                    .Attributes.uiClassConfidence_per;
            input->GenObjList.aObject[uObj]
                .Qualifiers.fProbabilityOfExistence_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.fProbabilityOfExistence_per;
            input->GenObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.uiHighestAssocProb_per;
            input->GenObjList.aObject[uObj]
                .Qualifiers.uiMeasuredTargetFrequency_nu =
                extReqPorts->SRRObjList.aObject[uObj]
                    .Qualifiers.uiMeasuredTargetFrequency_nu;
            // printf("\nuiMeasuredTargetFrequency_nu %f\n",
            // extReqPorts->SRRObjList.aObject[uObj].Qualifiers.uiMeasuredTargetFrequency_nu);
            input->GenObjList.aObject[uObj].Qualifiers.bObjStable =
                extReqPorts->SRRObjList.aObject[uObj].Qualifiers.bObjStable;
            input->GenObjList.aObject[uObj].RoadRelation.fDist2Course_met =
                pLBSCalc->RoadRelation[uObj].fDist2Course_met;
            // extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met;

            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmin_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmax_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmin_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmin_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmax_met =
                pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].fTTC_s =
                pLBSCalc->LBSObjInfoList[uObj].fTTC_s;
            input->LBSInputInfo.LBSObjInfoList[uObj].fTTCAccel_mps2 =
                pLBSCalc->LBSObjInfoList[uObj].fTTCAccel_mps2;
            input->LBSInputInfo.LBSObjInfoList[uObj].fCycletimeSum_s =
                pLBSCalc->LBSObjInfoList[uObj].fCycletimeSum_s;
            input->LBSInputInfo.LBSObjInfoList[uObj].fUpdateRate_nu =
                pLBSCalc->LBSObjInfoList[uObj].fUpdateRate_nu;
            // printf("\n fUpdateRate_nu %f\n",
            // pLBSCalc->LBSObjInfoList[uObj].fUpdateRate_nu);
            input->LBSInputInfo.LBSObjInfoList[uObj].fXMovement_met =
                pLBSCalc->LBSObjInfoList[uObj].fXMovement_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].fYMovement_met =
                pLBSCalc->LBSObjInfoList[uObj].fYMovement_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].fAngle_deg =
                pLBSCalc->LBSObjInfoList[uObj].fAngle_deg;
            input->LBSInputInfo.LBSObjInfoList[uObj].fAssocProbFiltered =
                pLBSCalc->LBSObjInfoList[uObj].fAssocProbFiltered;
            // printf("\nfAssocProbFiltered %f\n",
            // pLBSCalc->LBSObjInfoList[uObj].fAssocProbFiltered);
            input->LBSInputInfo.LBSObjInfoList[uObj].bLowTTCAtStart =
                pLBSCalc->LBSObjInfoList[uObj].bLowTTCAtStart;
            input->LBSInputInfo.LBSObjInfoList[uObj].bCreateAdjStableObj =
                pLBSCalc->LBSObjInfoList[uObj].bCreateAdjStableObj;
            input->LBSInputInfo.LBSObjInfoList[uObj].eAssociatedLane =
                pLBSCalc->SIObjInfoList[uObj].eAssociatedLane;
            input->LBSInputInfo.LBSObjInfoList[uObj].fDistToTraj_met =
                pLBSCalc->SIObjInfoList[uObj].fDistToTraj_met;
            input->LBSInputInfo.LBSObjInfoList[uObj].fVrelToTraj_mps =
                pLBSCalc->SIObjInfoList[uObj].fVrelToTraj_mps;
            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorObject =
                pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorObject;
            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorFrontObject =
                pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorFrontObject;
            input->LBSInputInfo.LCAObjInfoList[uObj].bLCAWarning =
                pLBSCalc->LCAObjInfoList[uObj].bLCAWarning;
            input->LBSInputInfo.BSDObjInfoList[uObj].bBSDWarning =
                pLBSCalc->BSDObjInfoList[uObj].bBSDInterruptRCW;
        }
    }
    input->EgoVehInfo.fegoAcceleration_mps2 =
        extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
    input->EgoVehInfo.fegoVelocity_mps =
        extReqPorts->EgoVehInfo.fegoVelocity_mps;
    input->EgoVehInfo.fLatAccel_mps2 = extReqPorts->EgoVehInfo.fLatAccel_mps2;
    input->EgoVehInfo.fLatVelocity_mps = 0.f;

    input->RCWSystemSwitch.bRCWFunctionActive =
        extReqPorts->LBSSystemParam.bRCWFunctionActive;
    input->RCWSystemSwitch.bRCWFunctionOutputActive =
        extReqPorts->LBSSystemParam.bRCWFunctionOutputActive;
    input->RCWSystemSwitch.bRCWFunctionNVMActive =
        extReqPorts->LBS_Ns_NVRAM_nu.LBS_Nb_RCWPowerOffSwitchState_nu;

    input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToRear_met =
        pLBSGlobals->fSensorOffsetToRear_met;
    input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToSide_met =
        pLBSGlobals->fSensorOffetToSide_met;
    // input->RCWPreProcessInput.RCWHmiOpen =
    //     extReqPorts->LBSSystemParam.bRCWFunctionActive;
    input->RCWPreProcessInput.RCWFailure =
        extReqPorts->LBSSupportInfo.LBSRCWFailure;
    input->RCWPreProcessInput.LeftTurnLightOpen =
        extReqPorts->LBSSupportInfo.LBSLeftTurnLightOpen;
    input->RCWPreProcessInput.RightTurnLightOpen =
        extReqPorts->LBSSupportInfo.LBSRightTurnLightOpen;
    input->RCWPreProcessInput.GearInReverse =
        extReqPorts->LBSSupportInfo.LBSGearInReverse;
    // input->RCWPreProcessInput.BlockingTimeActive = FALSE; inplement in the
    // postproces

    // RCWParam_st
    paramInput->RCWVehParameter.fVehicleWidth_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
    paramInput->RCWVehParameter.fVehicleLength_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
    paramInput->RCWVehParameter.fVehCenter2FrontAxis_met =
        extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met;

    paramInput->SensorMounting.SensorLeft.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorLeft.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorLeft.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_YawAngle_rad;
    paramInput->SensorMounting.SensorRight.fLatPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LatPos_met;
    paramInput->SensorMounting.SensorRight.fLongPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPos_met;
    paramInput->SensorMounting.SensorRight.fVertPos_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_VertPos_met;
    paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPosToCoG_met;
    paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_PitchAngle_rad;
    paramInput->SensorMounting.SensorRight.fOrientation_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_Orientation_rad;
    paramInput->SensorMounting.SensorRight.fYawAngle_rad =
        extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_YawAngle_rad;

    SenseTime_Memcpy(&extReqParam->LBS_Ks_RCWParameter_nu,
                     &paramInput->RCWWarningParameter,
                     sizeof(RCWWarningParameter_t));
}

void RCWOutputWrapper(LBSOutPro_t* extProPorts,
                      LBSDebug_t* extDebugPorts,
                      RCWOutPro_st* pOutput,
                      RCWDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc) {
    extProPorts->LBSFunState.uRCWWarning = pOutput->uHmiRCWWarningActive;
    extProPorts->LBS_Ns_NVRAM_nu.LBS_Nb_RCWPowerOffSwitchState_nu =
        pOutput->bHmiRCWHmiOn;
    extProPorts->LBSCANOutputs.uRCWObjIndex = pOutput->uHmiRCWWarningID;
    extDebugPorts->RCWDebug.LBSDebug_RCWstatemachine =
        pDebug->Debug_RCWstatemachine;
    // memcpy(&extDebugPorts->RCWDebug, pDebug, sizeof(RCWDebug_t));
    // printf("RCW prob %f\t, LBS prob %f\n",
    //         pDebug->RCWWarnDecideDebug[40].RCW_WarnDecide_fAssocProbFiltered,
    //         extDebugPorts->RCWDebug.LBSRCWWarnDecideDebug[40].RCW_WarnDecide_fAssocProbFiltered);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */