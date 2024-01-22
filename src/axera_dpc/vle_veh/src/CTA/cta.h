/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef CTA_H
#define CTA_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
*****************************************************************************/
#include "./cta_extern.h"
#include "./cta_fcta_extern.h"
#include "./cta_rcta_extern.h"
#include "./cta_par.h"
#include "tue_common_libs.h"

#define TUE_CML_MacroLowPassFilter(neu, alt, zeit_k) \
    ((neu + alt * zeit_k) / (zeit_k + 1.f))  // Macro for a low filter
/*****************************************************************************
 TYPEDEFS GLOBAL VARIABLE
*****************************************************************************/
typedef enum {
    CTA_RCTA_WARN_LEVEL_ONE,
    CTA_RCTA_WARN_LEVEL_TWO,
    CTA_RCTA_WARN_LEVEL_THREE
} CTARCTAWarningLevel_t;

typedef enum {
    CTA_REARTRACK_NO,
    CTA_REARTRACK_POS,
    CTA_REARTRACK_POS_SPEED
} CTARearObjState_t;

typedef enum {
    CTA_INIT,  // Initialize all
    CTA_OK,    // normal processing
} CTAState_t;

typedef struct {
    float32 fXMin_met;
    float32 fXMax_met;
    float32 fYMin_met;
    float32 fYMax_met;
} CTAObjectBorder_t;

typedef struct {
    float32 fDistX;
    float32 fDistY;
    float32 fWidth;
    float32 fLength;
} CTAObjectRotated_t;

typedef struct {
    CTAObjectBorder_t ObjectBorder;  // The object border information
    CTAObjectBorder_t
        ObjectMovementBorder;  // The object movement border information
    CTAObjectRotated_t ObjectRotated;
    float32 fTTC_s;
    float32 fTTCAccel_s;
    float32 fTTCFiltered_s;
    float32 fTTCRadial_s;
    float32 fFCTAVabs;
    float32 fRCTAVabs;
    // float32 fRangeRadial_met;
    float32 fFCTAXLastCycle_met;
    float32 fFCTAYLastCycle_met;
    float32 fRCTAXLastCycle_met;
    float32 fRCTAYLastCycle_met;
    float32 fFCTAVxPosBased_mps;
    float32 fFCTAVyPosBased_mps;
    float32 fRCTAVxPosBased_mps;
    float32 fRCTAVyPosBased_mps;
    // float32 fSpeedFiltered;
    float32 fCycleTimeSum_s;  // The object exist life time,unit:s
    float32 fUpdateRate_nu;   // The object measurement update rate,unit:NULL
    float32 fAssocProbFiltered_nu;  // Filtered highest cluster association
                                    // probability of the object filter result
    float32 fFCTAXMovement_met;     // The object total moving distance in the x
                                    // direction,unit:m
    float32 fFCTAYMovement_met;     // The object total moving distance in the y
    // direction,unit:m
    float32 fRCTAXMovement_met;  // The object total moving distance in the x
                                 // direction,unit:m
    float32 fRCTAYMovement_met;  // The object total moving distance in the y
                                 // direction,unit:m
    float32 fAngle_deg;
    float32 fObjWidthMax_met;
    float32 fObjLengthMax_met;
    // uint16 uUniqueID;
    // uint8 uLastMergedObjID;
    // boolean bLowTTCAtState;
    // boolean bCreatedAdjStableObj;
    // boolean bObjValidForSelection;
    // boolean bPriolObject;
} CTAObjectInfoGlobal_t;

typedef struct {
    float32 fLeftFrontPos_met;
    float32 fRightFrontPos_met;
    float32 fLeftRearPos_met;
    float32 fRightRearPos_met;
} SensorMountingPosGlobal_t;

typedef struct {
    float32 fFCTAXBreakthrough_met;
    float32 fRCTAXBreakthrough_met;
    float32 fFCTAXBreakthroughFiltered_met;
    float32 fRCTAXBreakthroughFiltered_met;
    float32 fFCTAXBreakthroughStd_met;
    float32 fRCTAXBreakthroughStd_met;
    float32 fFCTATTC_s;                   // TTC of the object
    float32 fRCTATTC_s;                   // TTC of the object
    float32 fFCTATTCFiltered_s;           // Filtered TTC of the object
    float32 fRCTATTCFiltered_s;           // Filtered TTC of the object
    float32 fFCTADistToCrossingLine_met;  // Distance to crossing line
    float32 fRCTADistToCrossingLine_met;  // Distance to crossing line
    float32
        fDistToCrossingLineFiltered_met;  // Filtered distance to crossing line
    float32 fFCTARearTrackProb_per;
    float32 fRCTARearTrackProb_per;
    // float32 fObjBreakthroughMargin_met;
    boolean bFCTARearTrack_nu;
    boolean bRCTARearTrack_nu;
} CTObjectInfoGlobal_t;

typedef struct {
    boolean bRelevant;
} CTAFCTAObjectInfoGlobal_t;

typedef struct {
    float32 fCriticalTTC;
    float32 fCriticalObjDistY;
    float32 fCriticalObjDistYLastCycle;
    sint32 iCriticalObjID;
    sint32 iCriticalObjIDLastCycle;
    uint8 uInterruptCycleCount;
    boolean bWarningInterrupt;
    float32 fMaxLatSensorRange;
} CTACTGlobal_t;

typedef struct {
    uint8 eRoadType;
    CTAState_t CTAState;
    boolean bFCTAFunctionActive;
    boolean bRCTAFunctionActive;
    boolean bEgoSpeedConditionFCTA;
} CTALastCycleState_t;

typedef struct {
    boolean bRCTAWarnActive[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
    float32 fCriticalTTC_s;
    float32 fCriticalObjDistY_met;
    float32 fCriticalObjDistYLastCycle_met;
    sint32 iCriticalObjID_nu;
    sint32 iCriticalObjIDLastCycle_nu;
    uint8 uInterruptCycleCount_nu;
    boolean bWarningInterrupt;
    CTADebugStateMachine_t RCTAStateMachine;
} CTARCTAOutput_t;

typedef struct {
    boolean bFCTAWarnActive[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
    float32 fCriticalTTC_s;
    sint32 iCriticalObjID_nu;
    CTADebugStateMachine_t FCTAStateMachine;
} CTAFCTAOutput_t;

typedef struct {
    CTAState_t eCTAState;  // CTA state: init or ok
    SensorMountingPosGlobal_t
        fSensorOffsetToSide_met;  // the offset from sensor mounting position to
                                  // the side edge of vehicle
    SensorMountingPosGlobal_t
        fSensorOffsetToRear_met;  // the offset from sensor mounting position to
                                  // the front or rear edge of vehicle
    float32 fMaxSpeedOverGround;
    CTALastCycleState_t LastCycleStates;
    CTACTGlobal_t CTGlobals;
    CTAObjectInfoGlobal_t CTAObjectList[CTA_MAX_NUM_OBJECTS];
    CTObjectInfoGlobal_t CTObjectList[CTA_MAX_NUM_OBJECTS];
    CTAFCTAObjectInfoGlobal_t CTAFCTAObjectList[CTA_MAX_NUM_OBJECTS];
    boolean bFCTAFunctionOutput;  // [CTA_FCTA_CFG_NUM_OF_WARN_LEVELS] ;
    boolean bRCTAFunctionOutput;  // [CTA_RCTA_CFG_NUM_OF_WARN_LEVELS] ;
    CTARCTAOutput_t CTARCTAOutput;
    CTAFCTAOutput_t CTAFCTAOutput;
} CTAGlobal_t;

void CTAInitObjects();

void CTAPreProcess(const CTAInReq_t* reqPorts,
                   const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal);
void CTACalculateGlobalProperties(
    const CTAVehicleParam_t* pVehicleParameter,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    SensorMountingPosGlobal_t* pfSensorOffsetToRear_met);
void CTACalculateObjectProperties(const CTAInReq_t* reqPorts,
                                  const CTAParam_t* params,
                                  CTAGlobal_t* pCTAGlobal);
void CTACalculateObjectFCTAMovementBorders(
    const CTAParam_t* params,
    boolean bRightSide,
    float32 fDistX_met,
    float32 fDistY_met,
    CTAObjectBorder_t* pObjectMovementBorder,
    float32* fXMovement_met,
    float32* fYMovement_met);
void CTACalculateObjectRCTAMovementBorders(
    const CTAParam_t* params,
    boolean bRightSide,
    float32 fDistX_met,
    float32 fDistY_met,
    CTAObjectBorder_t* pObjectMovementBorder,
    float32* fXMovement_met,
    float32* fYMovement_met);
void CTACalculateObjectQualifiers(uint8 uiHighestAssocProb_per,
                                  uint8 uiMaintenanceState_nu,
                                  float32* fUpdateRate_nu,
                                  float32* fAssocProbFiltered_nu);
void CTACalculateFCTAPosBasedVxVy(const CTAParam_t* params,
                                  float32 fCycleTime_s,
                                  const CTA_FusionObject_t* pEMFusionObjInput,
                                  float32 fXLastCycle_met,
                                  float32 fYLastCycle_met,
                                  float32* pfVxPosBased_mps,
                                  float32* pfVyPosBased_mps);
void CTACalculateRCTAPosBasedVxVy(const CTAParam_t* params,
                                  float32 fCycleTime_s,
                                  const CTA_FusionObject_t* pEMFusionObjInput,
                                  float32 fXLastCycle_met,
                                  float32 fYLastCycle_met,
                                  float32* pfVxPosBased_mps,
                                  float32* pfVyPosBased_mps);
void CTACalculateFCTAAbsoluteObjectVelocity(
    const CTA_FusionObject_t* pEMFusionObjInput,
    const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput,
    float32* fVabs);
void CTACalculateRCTAAbsoluteObjectVelocity(
    const CTA_FusionObject_t* pEMFusionObjInput,
    const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput,
    float32* fVabs);
void CTACalculateCTObjectProperties(const CTAInReq_t* reqPorts,
                                    const CTAParam_t* params,
                                    CTAGlobal_t* pCTAGlobal);
void CTA_CTInitCyclic(CTAGlobal_t* pCTAGlobal);
void CTA_CTCalculateMaxLatSensorRange(
    const CTA_FusionObject_t* pCTAEMFusionObjList,
    float32 fegoVelocity_mps,
    float32* pfMaxLatSensorRange);
void CTA_CTCalculateFCTADistToCrossingLine(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met,
    float32* pfDistToCrossingLineFiltered_met);
void CTA_CTCalculateRCTADistToCrossingLine(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met);
void CTA_CTCalculateFCTAXBreakthrough(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    float32 fDistToCrossingLine_met,
    float32* pfXBreakthrough_met,
    float32* pfXBreakthroughStd_met,
    float32* pfXBreakthroughFiltered_met);
void CTA_CTCalculateRCTAXBreakthrough(
    const CTAParam_t* params,
    const CTA_FusionObject_t* pEMFusionObjInput,
    float32 fDistToCrossingLine_met,
    float32* pfXBreakthrough_met,
    float32* pfXBreakthroughStd_met,
    float32* pfXBreakthroughFiltered_met);
void CTA_CTCalculateTTC(float32 fCycleTime_s,
                        const boolean bRightSide,
                        const float32 fVrelY_mps,
                        float32 fDistToCrossingLine_met,
                        float32* pfTTC_s,
                        float32* pfTTCFiltered_s);
void CTA_CTCalculateFCTARearObjectProbability(
    uint8 uObj,
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts,
    CTAGlobal_t* pCTAGlobal,
    float32* fFCTARearTrackProb_per);
void CTA_CTCalculateRCTARearObjectProbability(
    uint8 uObj,
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts,
    CTAGlobal_t* pCTAGlobal,
    float32* fRCTARearTrackProb_per);
CTARearObjState_t CTA_CTCalculateFCTASearchFrontObject(
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTA_FusionObject_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal,
    float32 fFCTARearTrackProb_per);
CTARearObjState_t CTA_CTCalculateRCTASearchFrontObject(
    const CTA_FusionObject_t* pCurrObjectEMInput,
    const CTA_FusionObject_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal,
    float32 fRCTARearTrackProb_per);
float32 CTA_CTCalculateObjectDistance(float32 fObjCurrMin_met,
                                      float32 fObjCurrMax_met,
                                      float32 fObjCandMin_met,
                                      float32 fObjCandMax_met);
void CTA_CTCheckRearObject(uint32 uClassification_nu,
                           float32 fRearTrackProb_per,
                           boolean* bRearTrack_nu);

void CTAToFCTAInputWrapper(const CTAInReq_t* reqPorts,
                           const CTAParam_t* params,
                           CTAGlobal_t* CTAGlobal,
                           FCTAInReq_t* FCTAreqPorts,
                           FCTAParam_t* FCTAparams);
void FCTAToCTAOutputWrapper(FCTAOutPro_t* FCTAproPorts,
                            FCTADebug_t* FCTAdebugInfo,
                            CTAGlobal_t* CTAGlobal,
                            CTADebug_t* debugInfo);
void CTAToRCTAInputWrapper(const CTAInReq_t* reqPorts,
                           const CTAParam_t* params,
                           CTAGlobal_t* CTAGlobal,
                           RCTAInReq_t* RCTAreqPorts,
                           RCTAParam_t* RCTAparams);
void RCTAToCTAOutputWrapper(RCTAOutPro_t* RCTAproPorts,
                            RCTADebug_t* RCTAdebugInfo,
                            CTAGlobal_t* CTAGlobal,
                            CTADebug_t* debugInfo);
void CTAProProcess(const CTAInReq_t* reqPorts,
                   const CTAParam_t* params,
                   CTAGlobal_t* CTAGlobal,
                   CTAOutPro_t* proPorts);
boolean CTAProcessCheckEgoSpeedRange(float32 fEgoSpeed,
                                     float32 fMinEgoSpeed,
                                     float32 fMaxEgoSpeed);
boolean CTAProcessSetFuncionOutput(boolean bFunctionEnabled,
                                   boolean bFunctionActive,
                                   boolean bEgoSpeedCondition);
boolean CTAProcessSetWarningOutput(boolean bFunctionOutputEnabled,
                                   boolean bInternalWarning,
                                   CTADebugStateMachine_t RCTAStateMachine);
#ifdef __cplusplus
}
#endif
#endif
