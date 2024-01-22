/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef CTA_EXTERN_H
#define CTA_EXTERN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
 *****************************************************************************/
#include "TM_Global_Types.h"
/*****************************************************************************
 MACRO DEFINITION
*****************************************************************************/
// #define CTA_EXTERN_MAX_NUM_OBJECTS \
//     160u  // the object's number of side radar output
#define CTA_EXTERN_MAX_NUM_FUSION_OBJECTS \
    100u  // the object's number of side radar output
#define CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS 3u
#define CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS 3u

/*****************************************************************************
 TYPEDEFS : INPUT   PARAMETER    OUTPUT    DEBUG
*****************************************************************************/
#ifndef Rte_TypeDef_ST_CTASEN_NVRAMData_t
#define Rte_TypeDef_ST_CTASEN_NVRAMData_t
typedef struct {
    boolean CTA_Nb_FCTAPowerOffSwitchState_nu;  // the BSD function switch state
                                                // when power is off
    boolean CTA_Nb_RCTAPowerOffSwitchState_nu;  // the LCA function switch state
    // when power is off
    boolean CTA_Nb_BoolReserved1_nu;
    boolean CTA_Nb_BoolReserved2_nu;
    boolean CTA_Nb_BoolReserved3_nu;
    boolean CTA_Nb_BoolReserved4_nu;
    boolean CTA_Nb_BoolReserved5_nu;
    uint8 CTA_Nu_UintReserved1_nu;
    uint8 CTA_Nu_UintReserved2_nu;
    uint8 CTA_Nu_UintReserved3_nu;
    uint8 CTA_Nu_UintReserved4_nu;
    uint8 CTA_Nu_UintReserved5_nu;
    float32 CTA_Nf_FloatReserved1_nu;
    float32 CTA_Nf_FloatReserved2_nu;
    float32 CTA_Nf_FloatReserved3_nu;
    float32 CTA_Nf_FloatReserved4_nu;
    float32 CTA_Nf_FloatReserved5_nu;
} ST_CTASEN_NVRAMData_t;
#endif

#ifndef Rte_TypeDef_eSensorMountingPos_t
#define Rte_TypeDef_eSensorMountingPos_t
typedef enum {
    LeftFrontPos,
    RightFrontPos,
    LeftRearPos,
    RighRearPos
} eSensorMountingPos_t;
#endif

#ifndef Rte_TypeDef_EMSRRObjectInReq_t
#define Rte_TypeDef_EMSRRObjectInReq_t
typedef struct {
    eSensorMountingPos_t eSensorMountingPos;
    // boolean bRightSensor;
    // GenObjKinematics
    float32 fDistX_met;  // Object's longitudinal relative distance
    float32 fDistXStd_met;
    float32 fDistY_met;  // Object's lateral relative distance
    float32 fDistYStd_met;
    float32 fVrelX_mps;  // Object's longitudinal relative velocity
    // float32 fVrelXStd_mps;
    float32 fVrelY_mps;  // Object's lateral relative velocity
    // float32 fVrelYStd_mps;
    float32 fArelX_mpss;
    // float32 fArelXStd_mpss;
    float32 fArelY_mpss;
    // float32 fArelYStd_mpss;
    float32 fVabsX_mps;  // Object's longitudinal velocity over ground
    // float32 fVabsXStd_mps;
    float32 fVabsY_mps;  // Object's lateral velocity over ground
    // float32 fVabsYStd_mps;
    // float32 fAabsX_mpss;
    // float32 fAabsXStd_mpss;
    // float32 fAabsY_mpss;
    // float32 fAabsYStd_mpss;
    // GenObjGeometry
    // float32 fWidth_met; /*Object's overall width*/ float32 fWidthStd_met;
    // /**/
    float32 fWidthLeft_met;   // Object's width left of the track position(left
                              // sensor view)
    float32 fWidthRight_met;  // Object's width right of the track position(left
                              // sensor view)
    // float32 fLength_met; /*Object's overall length*/ float32 fLengthStd_met;
    // /**/
    float32 fLengthFront_met;  // Object's length ahead of the track
                               // position(left sensor view)
    float32 fLengthRear_met;   // Object's length behind the track position(left
                               // sensor view)
    float32 fAbsOrientation_rad;  // Object moving direction,based on VX and VY
                                  // in AUTOSAR(left sensor view)
    // float32 fAbsOrientationStd_rad;
    float32 fRelHeading_rad;     // Heading Angle
    float32 fRelHeadingStd_rad;  // standard deviation heading Angle
    // float32 fClosestPointX_met;                                     /*X
    // position of closet point*/ float32 fClosestPointY_met; /*Y position of
    // closet point*/
    // GenObjGeneral
    // float32 fLifeTime_s;                                            /*Object
    // lifetime in second,unit: s*/
    uint16 uiLifeCycles_nu;  // Object lifetime in cycles,unit: null
    // uint32 uiLastMeasuredTimeStamp_ms;
    // uint16 uiLastMeasuredCycle_nu;
    uint8 uiMaintenanceState_nu;  // the maintenance(measured,predicted) state
                                  // whether the object is deleted
    // uint16 uiID_nu;
    // GenObjAttribute
    // uint8 eDynamicProperty_nu;                                      /*Object
    // dynamic property,stationary,moving or oncoming*/ uint8
    // uiDynConfidence_per;                                      /*General
    // confidence of dynamic property(moving,crossing,oncoming)*/
    uint32 uClassification_nu;  // Object classification
    // uint8 uiClassConfidence_per; /*Confidences for all classification*/ uint8
    // eObjctOcclusion_nu;
    // SRRObjHistory
    float32
        fFirstDetectX_met;  // X position where the object was created, unit:m
    float32
        fFirstDetectY_met;  // Y position where the object was created, unit:m
    // float32 fMaxRange_met;                                          /* Max
    // range over lifetime of this object, unit:m*/
    // SRRObjQualifier
    float32
        fProbabilityOfExistence_per;  // Probability that the object represents
                                      // a real object,unit:0.0-1.0f
    uint8 uiHighestAssocProb_per;     // Highest association probability of all
                                      // associated clusters in this cycle
    uint8 uiMeasuredTargetFrequency_nu;
    // Bitfield to indicate if the object was
    // measured in the last 8 cycles
    // uint8 eFusionStatus_nu; /*Enumeration of fused objects status*/
    boolean bObjStable;  // Flag that object is stable
    // SRRObjRoadRelation
    // float32 fProbInEgoLane_per;
    //  float32 fDist2Course_met; /* Object distance to
    // course, unit:m*/ float32 fDist2Border_met; /* Object distance to border
    // unit:m*/ float32 fGRDTrkProbability_per;
    // boolean bDist2BorderValid;                                      /* Valid
    // flag if the value in Dist2Border can be used */
    // SRRObjSensorSpecific
    float32 fRCS;             // RCS
    float32 fMirrorProb_per;  // The probability that the object is mirror
} EMSRRObjectInReq_t;
#endif

#ifndef Rte_TypeDef_EgoVehicleInReq_t
#define Rte_TypeDef_EgoVehicleInReq_t
typedef struct {
    float32 fegoVelocity_mps;  // the ego vehicle longitudinal velocity
                               // unit:m/s
    // float32 fVaregoVelocity_mps;                                    /*the ego
    // vehicle longitudinal velocity variance,unit:m/s*/ float32
    // fegoAcceleration_mps2;                                  /*the ego vehicle
    // longitudinal acceleration ,unit:m/s^2*/ float32 fVaregoAcceleration_mps2;
    // the ego vehicle longitudinal acceleration variance,unit:m/s^2
    float32 fYawRate_radps;  // the ego vehicle yaw rate ,unit:rad/s
    // float32 fCurve_1pm;                                             /*the ego
    // vehicle driver curve ,unit:1/m*/ float32 DrvIntCurve_1pm; /*the ego
    // vehicle driver curve ,unit:1/m*/ float32 fLatAccel_mps2; /*the ego
    // vehicle Lateral Acceleration,unit:m/s^2*/ float32 fSlipAngle_rad; /*the
    // ego vehicle slip angle ,unit:rad*/
    float32 fSelfSteering_rad;  // the ego vehicle steering angle ,unit:rad
    uint8 uGear_nu;  // ego vehicle gear. 0x0 Default 0x1 Park 0x2 Reverse 0x3
                     // Neutral 0x4 Drive 0x5 S 0x7 Invalid
} EgoVehicleInReq_t;
#endif

#ifndef Rte_TypeDef_CTARoadInReq_t
#define Rte_TypeDef_CTARoadInReq_t
typedef struct {
    float32 fCurveRadius_met;  // The curvature radius of the current driver
                               // road ,unit:m
} CTARoadInReq_t;
#endif

#ifndef Rte_TypeDef_CTASignalHeader_t
#define Rte_TypeDef_CTASignalHeader_t
typedef struct {
    uint32 uiTimeStamp;
    uint16 uiMeasurementCounter;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
} CTASignalHeader_t;
#endif

#ifndef Rte_TypeDef_CTA_FusionObject_t
#define Rte_TypeDef_CTA_FusionObject_t
typedef struct {
    float32 fDistX_met;  // Object's longitudinal relative distance
    float32 fDistXStd_met;
    float32 fDistY_met;  // Object's lateral relative distance
    float32 fDistYStd_met;
    float32 fVrelX_mps;  // Object's longitudinal relative velocity
    float32 fVrelY_mps;  // Object's lateral relative velocity
    float32 fArelX_mpss;
    float32 fArelY_mpss;
    float32 fVabsX_mps;       // Object's longitudinal velocity over ground
    float32 fVabsY_mps;       // Object's lateral velocity over ground
    float32 fWidthLeft_met;   // Object's width left of the track position(left
                              // sensor view)
    float32 fWidthRight_met;  // Object's width right of the track position(left
                              // sensor view)
    float32 fLengthFront_met;  // Object's length ahead of the track
                               // position(left sensor view)
    float32 fLengthRear_met;   // Object's length behind the track position(left
                               // sensor view)
    float32 fAbsOrientation_rad;  // Object moving direction,based on VX and VY
                                  // in AUTOSAR(left sensor view)
    float32 fRelHeading_rad;      // Heading Angle
    float32 fRelHeadingStd_rad;   // standard deviation heading Angle
    uint16 uiLifeCycles_nu;       // Object lifetime in cycles,unit: null
    uint8 uiMaintenanceState_nu;  // the maintenance(measured,predicted) state
                                  // whether the object is deleted
    uint32 uClassification_nu;    // Object classification
    float32
        fFirstDetectX_met;  // X position where the object was created, unit:m
    float32
        fFirstDetectY_met;  // Y position where the object was created, unit:m
    float32
        fProbabilityOfExistence_per;  // Probability that the object represents
                                      // a real object,unit:0.0-1.0f
    uint8 uiHighestAssocProb_per;     // Highest association probability of all
                                      // associated clusters in this cycle
    uint8 uiMeasuredTargetFrequency_nu;
    boolean bObjStable;       // Flag that object is stable
    float32 fRCS;             // RCS
    float32 fMirrorProb_per;  // The probability that the object is mirror
    uint8 uNumOfUsedObjects;
    uint16 uiID_nu;
    sint32 iRawFusionID_nu;
    boolean bRightSide;
} CTA_FusionObject_t;
#endif

#ifndef Rte_TypeDef_CTAFusionObjectInReq_t
#define Rte_TypeDef_CTAFusionObjectInReq_t
typedef struct {
    uint32 uiVersionNumber;
    CTASignalHeader_t sSigHeader;
    uint8 uNumOfUsedObjects;
    CTA_FusionObject_t aObjects[CTA_EXTERN_MAX_NUM_FUSION_OBJECTS];
} CTAFusionObjectInReq_t;
#endif

#ifndef Rte_TypeDef_CTAInReq_t
#define Rte_TypeDef_CTAInReq_t
typedef struct {
    float32 fCycleTime_s;         // Current task cycle time from EMGlobalOutput
    boolean bFCTAFunctionActive;  // FCTA function active flag
    boolean bFCTAFunctionOutputActive;  // FCTA function output flag
    boolean bRCTAFunctionActive;        // RCTA function active flag
    boolean bRCTAFunctionOutputActive;  // RCTA function output flag
    boolean bFCTAFailure;               // boolean bFCA;
    boolean b_FCTA_RCA;
    boolean b_FCTA_FCB;
    boolean bRCTAFailure;  // boolean bRCA;
    boolean b_RCTA_RCA;
    boolean b_RCTA_FCB;
    CTAFusionObjectInReq_t CTAEMFusionObjList;
    EgoVehicleInReq_t EgoVehicleInfo;
    CTARoadInReq_t CTARoadInformation;
    ST_CTASEN_NVRAMData_t CTA_Ns_NVRAM_nu;
} CTAInReq_t;
#endif

#ifndef Rte_TypeDef_CTASensorMounting_t
#define Rte_TypeDef_CTASensorMounting_t
typedef struct {
    float32 CTA_Kf_LatPos_met;  /*the radar sensor mounting Y position ,unit:m*/
    float32 CTA_Kf_LongPos_met; /*the radar sensor mounting X position ,unit:m*/
    // float32 fVertPos_met;                                           /*the
    // radar sensor mounting Z position ,unit:m*/ float32 fLongPosToCoG_met;
    // /*the radar sensor mounting position to COG longitudinal distance
    // ,unit:m*/ float32 fPitchAngle_rad; /*the radar sensor mounting Pitch
    // angle ,unit:rad*/ float32 fOrientation_rad; /*TODO*/ float32
    // fRollAngle_rad;
    // /*the radar sensor mounting Roll angle ,unit:rad*/ float32 fYawAngle_rad;
    // /*the radar sensor mounting Yaw angle ,unit:rad,range[-pi,+pi]*/
} CTASensorMounting_t;
#endif

#ifndef Rte_TypeDef_CTAVehicleParam_t
#define Rte_TypeDef_CTAVehicleParam_t
typedef struct {
    // vehicle body
    float32 CTA_Kf_WheelBase_met;     // The distance between the center of the
                                      // front wheel and the center of the rear
                                      // wheel,unit:m*/
    float32 CTA_Kf_VehicleWidth_met;  /*the vehicle body width,unit:m*/
    float32 CTA_Kf_VehicleLength_met; /*the vehicle body length,unit:m*/
    // float32 fVehCenter2FrontAxis_met; /*the vehicle center to front axis
    // center distance*/
    float32 CTA_Kf_OverhangFront_met;  // the length of vehicle front overhang
    CTASensorMounting_t CTA_Ks_LeftFrontSensorMounting;
    CTASensorMounting_t CTA_Ks_RightFrontSensorMounting;
    CTASensorMounting_t CTA_Ks_LeftRearSensorMounting_nu;
    CTASensorMounting_t CTA_Ks_RightRearSensorMounting_nu;
} CTAVehicleParam_t;
#endif

#ifndef Rte_TypeDef_CTARCTASteeringAngleCutOffParam_t
#define Rte_TypeDef_CTARCTASteeringAngleCutOffParam_t
typedef struct {
    float32 fSteerAngleCutOffMin_deg;       // 300
    float32 fSteerAngleCutOffMid_deg;       // 400
    float32 fSteerAngleCutOffMax_deg;       // 500
    float32 fXMinBreakthroughSWAMid_met;    // -5
    float32 fXMinBreakthroughSWAMax_met;    // -3
    boolean bEnableSteeringAngleCutOff_nu;  // TRUE
} CTARCTASteeringAngleCutOffParam_t;
#endif

#ifndef Rte_TypeDef_CTARCTAAlgoParam_t
#define Rte_TypeDef_CTARCTAAlgoParam_t
typedef struct {
    float32 CTA_Kf_TTCThreshold_s;        // 2.5
    float32 CTA_Kf_TTCThresholdL2_s;      // 1.5
    float32 CTA_Kf_TTCThresholdL3_s;      // 1.5
    float32 CTA_Kf_TTCThresholdMargin_s;  // 0
    float32 CTA_Kf_TTCThresholdPed_s;
    float32 CTA_Kf_TTCThresholdPedL2_s;
    float32 CTA_Kf_TTCThresholdPedL3_s;
    float32 CTA_Kf_TTCThresholdMarginPed_s;
    float32 CTA_Kf_XMinBreakthrough_met;    // -6
    float32 CTA_Kf_XMinBreakthroughL2_met;  // -4.199
    float32 CTA_Kf_XMinBreakthroughL3_met;  // -4.199
    float32 CTA_Kf_XMaxBreakthrough_met;    // 1
    float32 CTA_Kf_VEgoMax_mps;             // -3.611m/s = 13km/h
    float32 CTA_Kf_VEgoMin_mps;             // 0
    float32 CTA_Kf_VTargetMin_mps;  // The min target vehicle speed of RCTA is
                                    // actived; Value: 1.399f m/s
    // float32 fVTargetMax; // 100
    // float32 fKeepTime; // 0
    // float32 fNoAlertZoneWidth; // 0
    float32 CTA_Kf_MaxHeadingAngle_deg;   // -30
    float32 CTA_Kf_MinHeadingAngle_deg;   // -160
    float32 CTA_Kf_TargetRangeMax_met;    // 42
    float32 CTA_Kf_TargetRangeMaxL2_met;  // 42
    float32 CTA_Kf_TargetRangeMaxL3_met;  // 42
    // float32 fBreakthroughMargin;// 1
    // boolean bEnableObjAdaptiveBreakthrough;// True
    boolean CTA_Kb_Active_nu;  // TRUE
    CTARCTASteeringAngleCutOffParam_t CTA_Ks_SteeringAngleCutOff_nu;
    // float32 fVTargetMinVRU;
    // boolean bEnableSensorLeftRightFusion;
} CTARCTAAlgoParam_t;
#endif

#ifndef Rte_TypeDef_CTAFCTAAlgoParam_t
#define Rte_TypeDef_CTAFCTAAlgoParam_t
typedef struct {
    float32 CTA_Kf_TTCThreshold_s;    // 2.5f
    float32 CTA_Kf_TTCThresholdL2_s;  // 1.5f
    float32 CTA_Kf_TTCThresholdL3_s;  // 1.5f
    float32 CTA_Kf_TTCThresholdPed_s;
    float32 CTA_Kf_TTCThresholdPedL2_s;
    float32 CTA_Kf_TTCThresholdPedL3_s;
    float32 CTA_Kf_XMinBreakthrough_met;    // 0.f
    float32 CTA_Kf_XMinBreakthroughL2_met;  // 0.f
    float32 CTA_Kf_XMinBreakthroughL3_met;  // 0.f
    float32 CTA_Kf_XMaxBreakthrough_met;    // 7.f
    float32 CTA_Kf_VEgoMax_mps;             // 300.f// 8.333m/s = 20km/h
    float32 CTA_Kf_VEgoMin_mps;             // -300.f// 0
    float32 CTA_Kf_VTargetMin_mps;          // 0.82999998f
    float32 CTA_Kf_VTargetMax_mps;          // 300.f// 100
    float32 CTA_Kf_MaxHeadingAngle_deg;     // 120.f
    float32 CTA_Kf_MinHeadingAngle_deg;     // 60.f
    float32 CTA_Kf_TargetRangeMax_met;      // 71.f
    float32 CTA_Kf_TargetRangeMaxL2_met;    // 71.f
    float32 CTA_Kf_TargetRangeMaxL3_met;    // 71.f
    float32 CTA_Kf_BreakthroughMargin_met;  // 0.f
    float32 CTA_Kf_TTCThresholdMargin_s;    // 1.f
    boolean CTA_Kb_Active_nu;               // True
} CTAFCTAAlgoParam_t;
#endif

#ifndef Rte_TypeDef_CTAParam_t
#define Rte_TypeDef_CTAParam_t
typedef struct {
    CTAVehicleParam_t CTA_Ks_VehicleParameter_nu;
    CTARCTAAlgoParam_t CTA_Ks_RCTAAlgoParameter_nu;
    CTAFCTAAlgoParam_t CTA_Ks_FCTAAlgoParameter_nu;
} CTAParam_t;
#endif

#ifndef Rte_TypeDef_CTAOutPro_t
#define Rte_TypeDef_CTAOutPro_t
typedef struct {
    uint32 uiVersionNumber;
    ST_CTASEN_NVRAMData_t CTA_Ns_NVRAM_nu;
    // RCTA
    float32 fRCTAfTTC_s;
    sint32 iRCTACriticalObjID_nu;
    uint8 uRCTACriticalObjType_nu;  // 0x0 No object 0x1 Pedestrain 0x2 Vehicle
                                    // 0x3 Cyelist 0x4 Unknown
    uint8 uRCTASystemFaultState_nu;
    boolean bRCTAWarningLeftL1;
    boolean bRCTAWarningLeftL2;
    boolean bRCTAWarningRightL1;
    boolean bRCTAWarningRightL2;
    // FCTA
    boolean bFCTAWarningLeftL1;
    boolean bFCTAWarningLeftL2;
    boolean bFCTAWarningRightL1;
    boolean bFCTAWarningRightL2;
    float32 fFCTAfTTC_s;
    sint32 iFCTACriticalObjID_nu;
    uint8 uFCTACriticalObjType_nu;
    uint8 uFCTASystemFaultState_nu;

} CTAOutPro_t;
#endif

#ifndef Rte_TypeDef_CTADebugStateMachine_t
#define Rte_TypeDef_CTADebugStateMachine_t
typedef enum {
    CTAStateOFF = 0u,
    CTAStateStandby,
    CTAStateActive,
    CTAStateFailure
} CTADebugStateMachine_t;
#endif

// #ifndef Rte_TypeDef_CTAFCTADebugObjGlobal_t
// #define Rte_TypeDef_CTAFCTADebugObjGlobal_t
// typedef struct {
//     float32 fBTHitHystTimer_s[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     uint8 uBreakthroughHitConfi[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bBreakthroughHit[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bWarning[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bWarningLastCycle[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bObjectInRange[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Flag
//     whether
//                                                                 // the object
//                                                                 is
//                                                                 // in the
//                                                                 range
//     boolean bTTCBelowThresh[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bBTHitHystActive[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     float32 fRearTrackProb;
//     uint8 uRearMirrorCnt;
//     boolean bRelevant;
//     boolean bMirror;
//     boolean bObjectFromSide;
//     boolean bValidApproachAngle;
//     boolean bObjectAtEdgeFoV;
//     boolean bShortWarning;
//     boolean bUpdatedRecently;  // Flag whether the object is updated recently
//     boolean bQuality;
//     boolean bRearMirrorObject;
//     boolean bObjMovementValid;
// } CTAFCTADebugObjGlobal_t;
// #endif

#ifndef Rte_TypeDef_FCTADebugInfo_t
#define Rte_TypeDef_FCTADebugInfo_t
typedef struct {
    // float32 fTTCThreshold_s[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // float32 fXMinBreakthrough_met[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // float32 fXMaxBreakthrough_met[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // float32
    //     fMaxObjRange_met[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];  // the max
    //                                                           // threshold of
    //                                                           // object range
    // float32 fMaxHeadingAngle_deg;
    // float32 fMinHeadingAngle_deg;
    // float32 fVegoMin_mps;
    // float32 fVegoMax_mps;
    // float32 fVTargetMin;
    // float32 fVTargetMax;
    // float32 fCriticalTTC_s;
    // float32 fCriticalObjDistY_met;
    // float32 fCriticalObjDistYLastCycle_met;
    // sint32 iCriticalObjID_nu;
    // sint32 iCriticalObjIDLastCycle_nu;
    // uint8 uInterruptCycleCount_nu;
    // boolean bWarningInterrupt;
    // boolean bFCTAWarnActive[CTA_FCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // CTAFCTADebugObjGlobal_t
    //     FCTA_Va_ObjectListGlobal[CTA_EXTERN_MAX_NUM_FUSION_OBJECTS];
    CTADebugStateMachine_t FCTAStateMachine;
} FCTADebugInfo_t;
#endif

// #ifndef Rte_TypeDef_CTARCTADebugObjGlobal_t
// #define Rte_TypeDef_CTARCTADebugObjGlobal_t
// typedef struct {
//     float32
//         fBTHitHystTimer_s[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // the warning
//     // hysteresis timer
//     uint8 uBreakthroughHitConfi
//         [CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Breakthrough
//                                               // hit
//                                               // confidence
//     boolean
//         bBreakthroughHit[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Flag whether
//     // the object hits
//     // breakthrough
//     boolean
//         bWarning[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Flag whether the
//         RCTA
//                                                       // warning is activated
//     boolean
//         bWarningLastCycle[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Flag
//         whether
//                                                                // the RCTA
//                                                                // warning is
//     // activated in the last cycle
//     boolean bObjectInRange[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean bTTCBelowThresh[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];
//     boolean
//         bBTHitHystActive[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // Flag whether
//                                                               // the warning
//                                                               // hysteresis
//     boolean bRelevant;
//     boolean bMirror;
//     boolean bObjectFromSide;      // Flag whether the object approach from
//     side boolean bValidApproachAngle;  // Flag whether the object heading
//     angle is
//                                   // valid in the current cycle
//     boolean bShortTTC;
//     boolean bUpdatedRecently;
//     boolean bQuality;
// } CTARCTADebugObjGlobal_t;
// #endif

#ifndef Rte_TypeDef_RCTADebugInfo_t
#define Rte_TypeDef_RCTADebugInfo_t
typedef struct {
    // float32 fTTCThreshold_s[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // float32 fXMinBreakthrough_met
    //     [CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // x-axis min
    //                                           // edge of
    //                                           // breakthrough
    // float32 fXMaxBreakthrough_met
    //     [CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];  // x-axis max
    //                                           // edge of
    //                                           // breakthrough
    // float32 fMaxObjRange_met[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // float32 fMaxHeadingAngle_deg;  // Calculate the max value of heading
    // angle float32 fMinHeadingAngle_deg;  // Calculate the min value of
    // heading angle float32 fCriticalTTC_s; float32 fCriticalObjDistY_met;
    // float32 fCriticalObjDistYLastCycle_met;
    // sint32 iCriticalObjID_nu;
    // sint32 iCriticalObjIDLastCycle_nu;
    // uint8 uInterruptCycleCount_nu;
    // boolean bWarningInterrupt;
    // boolean bRCTAWarnActive[CTA_RCTA_DEBUG_NUM_OF_WARN_LEVELS];
    // CTARCTADebugObjGlobal_t
    //     RCTA_Va_ObjectListGlobal[CTA_EXTERN_MAX_NUM_FUSION_OBJECTS];
    CTADebugStateMachine_t RCTAStateMachine;
} RCTADebugInfo_t;
#endif

// #ifndef Rte_TypeDef_CTAObjectBorderDebug_t
// #define Rte_TypeDef_CTAObjectBorderDebug_t
// typedef struct {
//     float32 fXMin_met;
//     float32 fXMax_met;
//     float32 fYMin_met;
//     float32 fYMax_met;
// } CTAObjectBorderDebug_t;
// #endif

// #ifndef Rte_TypeDef_CTAObjectRotatedDebug_t
// #define Rte_TypeDef_CTAObjectRotatedDebug_t
// typedef struct {
//     float32 fDistX;
//     float32 fDistY;
//     float32 fWidth;
//     float32 fLength;
// } CTAObjectRotatedDebug_t;
// #endif

// #ifndef Rte_TypeDef_CTAObjectInfoGlobalDebug_t
// #define Rte_TypeDef_CTAObjectInfoGlobalDebug_t
// typedef struct {
//     CTAObjectBorderDebug_t ObjectBorder;  // The object border information
//     CTAObjectBorderDebug_t
//         ObjectMovementBorder;  // The object movement border information
//     CTAObjectRotatedDebug_t ObjectRotated;
//     float32 fTTC_s;
//     float32 fTTCAccel_s;
//     float32 fTTCFiltered_s;
//     float32 fTTCRadial_s;
//     float32 fFCTAVabs;
//     float32 fRCTAVabs;
//     float32 fFCTAXLastCycle_met;
//     float32 fFCTAYLastCycle_met;
//     float32 fRCTAXLastCycle_met;
//     float32 fRCTAYLastCycle_met;
//     float32 fFCTAVxPosBased_mps;
//     float32 fFCTAVyPosBased_mps;
//     float32 fRCTAVxPosBased_mps;
//     float32 fRCTAVyPosBased_mps;
//     float32 fCycleTimeSum_s;  // The object exist life time,unit:s
//     float32 fUpdateRate_nu;   // The object measurement update rate,unit:NULL
//     float32 fAssocProbFiltered_nu;  // Filtered highest cluster association
//                                     // probability of the object filter
//                                     result
//     float32 fFCTAXMovement_met;     // The object total moving distance in
//     the x
//                                     // direction,unit:m
//     float32 fFCTAYMovement_met;     // The object total moving distance in
//     the y
//     // direction,unit:m
//     float32 fRCTAXMovement_met;  // The object total moving distance in the x
//                                  // direction,unit:m
//     float32 fRCTAYMovement_met;  // The object total moving distance in the y
//                                  // direction,unit:m
//     float32 fAngle_deg;
//     float32 fObjWidthMax_met;
//     float32 fObjLengthMax_met;
// } CTAObjectInfoGlobalDebug_t;
// #endif

// #ifndef Rte_TypeDef_CTObjectInfoGlobalDebug_t
// #define Rte_TypeDef_CTObjectInfoGlobalDebug_t
// typedef struct {
//     float32 fFCTAXBreakthrough_met;
//     float32 fRCTAXBreakthrough_met;
//     float32 fFCTAXBreakthroughFiltered_met;
//     float32 fRCTAXBreakthroughFiltered_met;
//     float32 fFCTAXBreakthroughStd_met;
//     float32 fRCTAXBreakthroughStd_met;
//     float32 fFCTATTC_s;                   // TTC of the object
//     float32 fRCTATTC_s;                   // TTC of the object
//     float32 fFCTATTCFiltered_s;           // Filtered TTC of the object
//     float32 fRCTATTCFiltered_s;           // Filtered TTC of the object
//     float32 fFCTADistToCrossingLine_met;  // Distance to crossing line
//     float32 fRCTADistToCrossingLine_met;  // Distance to crossing line
//     float32
//         fDistToCrossingLineFiltered_met;  // Filtered distance to crossing
//         line
//     float32 fFCTARearTrackProb_per;
//     float32 fRCTARearTrackProb_per;
//     boolean bFCTARearTrack_nu;
//     boolean bRCTARearTrack_nu;
// } CTObjectInfoGlobalDebug_t;
// #endif

#ifndef Rte_TypeDef_CTADebug_t
#define Rte_TypeDef_CTADebug_t
typedef struct {
    // uint32 uiVersionNumber;
    // CTAObjectInfoGlobalDebug_t
    // CTAObjectList[CTA_EXTERN_MAX_NUM_FUSION_OBJECTS];
    // CTObjectInfoGlobalDebug_t
    // CTObjectList[CTA_EXTERN_MAX_NUM_FUSION_OBJECTS];
    FCTADebugInfo_t FCTADebugInfo;
    RCTADebugInfo_t RCTADebugInfo;
} CTADebug_t;
#endif

/*****************************************************************************
 FUNCTION
*****************************************************************************/
void CTA_Exec(const CTAInReq_t* reqPorts,
              const CTAParam_t* params,
              CTAOutPro_t* proPorts,
              CTADebug_t* debugInfo);
void CTA_Reset();

#ifdef __cplusplus
}
#endif
#endif
