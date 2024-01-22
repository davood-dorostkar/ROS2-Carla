/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wen mingshu <wenmingshu@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_CONVERTERS_TYPES_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_CONVERTERS_TYPES_H_
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "FusionCompiler.h"
/* Inclusions ================================================================*/

#define CamDetectionType_GENERAL_OBJECT_PEDESTRIAN (1U)
#define CamDetectionType_GENERAL_OBJECT_VEHICLE (2U)
#define CamDetectionType_PRECISE_OBJECT_VEHICLE (3U)

#define CamMotionMode_UNKOWN (0U)
#define CamMotionMode_STATIC (1U)
#define CamMotionMode_PRECEDING (2U)
#define CamMotionMode_ONCOMING (3U)
#define CamMotionMode_CROSSING (4U)

#define CamObjClass_UNKOWN (0U)
#define CamObjClass_CAR (1U)
#define CamObjClass_TRUCK (2U)
#define CamObjClass_PEDESTRIAN (3U)
#define CamObjClass_TWO_WHEELER (4U)
#define CamObjClass_OTHER_OBJECT (5U)

#define CamObjPos_UNKOWN (0U)
#define CamObjPos_LEFT_IS_FRONT (1U)
#define CamObjPos_RIGHT_IS_FRONT (2U)

#define CamTrackingStatus_PRECONFIRMED (0U)
#define CamTrackingStatus_CONFIRMED (1U)

#define RaMeasEnabled_DISABLED (0U)
#define RaMeasEnabled_ENABLED (1U)

#define RaMeasFlag_MEASURED (0U)
#define RaMeasFlag_EXTRAPOLATED (1U)

#define RaObjClass_UNKNOWN (0U)
#define RaObjClass_4_WHEELER (1U)
#define RaObjClass_2_WHEELER (2U)
#define RaObjClass_PEDESTRIAN (3U)

#define RaObjMotionPattern_UNKOWN (0U)
#define RaObjMotionPattern_STATIONARY (1U)
#define RaObjMotionPattern_STOPPED (2U)
#define RaObjMotionPattern_MOVING (3U)
#define RaObjMotionPattern_CROSSING (4U)

#define RaSGU_NOT_FAILED (0U)
#define RaSGU_FAILED (1U)

#define RaStatusBlkProg_NOT_BLOCKED (0U)
#define RaStatusBlkProg_BLOCKED (1U)

#define RaStatusHW_NOT_FAILED (0U)
#define RaStatusHW_FAILED (1U)

#define RaStatusMisalign_UNKNOWN (0U)
#define RaStatusMisalign_CALIBRATED (1U)
#define RaStatusMisalign_MISALIGNED (2U)
#define RaStatusMisalign_CALIB_IN_PROGRESS (3U)
#define RaStatusMisalign_UNCALIBRATED (4U)

#define RaUpdateFlag_NEW_OBJECT (0U)
#define RaUpdateFlag_OBJECT_EXISTED (1U)

#define RaValidFlag_OBJECT_INVALID (0U)
#define RaValidFlag_OBJECT_VALID (1U)

/* ImplementationDataTypes ===================================================*/
/*
typedef struct {
float32 ALgtStdFromWhlSpd;
uint8 ALgtStdFromWhlSpdQf;
uint8 ALgtStdFromWhlSpdSafeChks;
uint8 ALgtStdFromWhlSpdSafeCntr;
} ALgtStdFromWhlSpdSafe1;

typedef struct {
float32 YawRateWithCmp;
uint8 YawRate1Qf2;
float32 ALatWithCmp;
uint8 ALat1Qf2;
float32 GrdtOfALgt;
uint8 ALgt1Qf2;
uint8 AsyDataWithCmpSafeChks;
uint8 AsyDataWithCmpSafeCntr;
} AsyDataWithCmpSafe1; */

typedef struct {
    uint8 detectionType;
    uint8 objectClass;
    uint8 motionMode;
    uint8 objPose;
    uint8 trackingStatus;
    uint8 id;
    float32 xPos;
    float32 yPos;
    float32 xRelVel;
    float32 yRelVel;
    float32 xPosStd;
    float32 xRelVelStd;
    float32 objWidth;
    float32 objYawRate;
    uint8 ageConsecutive;
    float32 vehicleClassConfidence;
    float32 pedestrianClassConfidence;
    uint8 formApproxError;
    boolean isOccludedLeft;
    boolean isOccludedRight;
    boolean isObservedLeft;
    boolean isObservedRight;
    boolean isAsilVerified;
    boolean isPredicted;
    boolean isAtCrosswalkRiskPoint;
} CamObjType;

typedef struct {
    uint8 numObjects;
    CamObjType objects[21];
} CamObjVecType;

typedef struct {
    boolean isAvailable;
    boolean isBlocked;
    uint32 timeStamp;
    float32 latency;
} CamStatusFlagsType;

typedef struct {
    CamObjVecType objectsVec;
    CamStatusFlagsType statusFlags;
} CamObjListType;

typedef struct {
    float32 longPosition;
    float32 latPosition;
    float32 headingAngle;
    float32 speed;
    float32 longVelocity;
    float32 latVelocity;
    float32 acceleration;
    float32 longAcceleration;
    float32 latAcceleration;
    float32 curvature;
} typeTrackEstimateVcc;

typedef struct {
    uint8 type;
    float32 width;
    float32 height;
    uint8 nearestSide;
    uint8 turnIndicator;
    uint8 brakeLightIndicator;
    uint8 hazardLightIndicator;
    uint8 motionPatternCurrent;
    uint8 motionPatternHistory;
    float32 distanceToLeftNearLaneMarking;
    float32 distanceToRightNearLaneMarking;
    float32 distanceToLeftFarLaneMarking;
    float32 distanceToRightFarLaneMarking;
} typeTrackInformationVcc;

typedef struct {
    float32 accelerationStdDev;
    uint8 cmbbPrimaryConfidence;
    uint8 cmbbSecondaryConfidence;
    uint8 fcwConfidence;
    uint8 fusionSource;
    float32 headingAngleStdDev;
    uint8 id;
    float32 latPositionStdDev;
    uint8 leftNearLaneMarkingConfidence;
    uint8 leftFarLaneMarkingConfidence;
    float32 longPositionStdDev;
    uint8 motionModel;
    uint8 positionConfidence;
    uint8 rightNearLaneMarkingConfidence;
    uint8 rightFarLaneMarkingConfidence;
    float32 speedStdDev;
    uint8 tjaConfidence;
    uint8 trackStatus;
    uint8 trafficScenario;
    uint8 visionId;
} typeTrackPropertiesVcc;

typedef struct {
    typeTrackEstimateVcc Estimate;
    typeTrackInformationVcc Information;
    typeTrackPropertiesVcc Properties;
} typeFrontFusionObjectVcc;
typedef typeFrontFusionObjectVcc rt_Array_typeFrontFusionObjectVcc_32[32];
typedef struct {
    float32 latDistanceZeroOrderCoeff;
    float32 latDistanceFirstOrderCoeff;
    float32 latDistanceSecondOrderCoeff;
    float32 latDistanceThirdOrderCoeff;
    float32 longDistanceToEnd;
} typeBarrierEstimateVcc;

typedef struct {
    uint8 informationSource;
} typeBarrierObjectVccInformation0;

typedef struct {
    uint8 trackStatus;
    uint8 cmbbConfidence;
    uint8 lkaConfidence;
    uint8 tjaConfidencePrimary;
    float32 tuneConfidence;
} typeBarrierPropertiesVcc;

typedef struct {
    typeBarrierEstimateVcc Estimate;
    typeBarrierObjectVccInformation0 Information;
    typeBarrierPropertiesVcc Properties;
} typeBarrierObjectVcc;

typedef struct {
    typeBarrierObjectVcc Left;
    typeBarrierObjectVcc Right;
} typeBarriersVcc;

typedef struct {
    uint32 timeStamp;
    uint16 lookIndex;
    uint8 visionOnlyBrake;
    uint8 visionOnlyWarning;
    uint8 visionOnlyVruBrake;
    rt_Array_typeFrontFusionObjectVcc_32 Objects;
    typeBarriersVcc Barriers;
} typeRaCamObjectVcc;

typedef struct {
    uint8 objtype;
    uint8 objMotionPattern;
    uint8 updateFlag;
    uint8 validFlag;
    uint8 measFlag;
    uint8 id;
    float32 xPos;
    float32 yPos;
    float32 xVelRel;
    float32 yVelRel;
    float32 xAccRel;
    float32 xPosStd;
    float32 yPosStd;
    float32 xVelRelStd;
    float32 obstacleProb;
    float32 objExstProb;
} RaObjType;

typedef struct {
    uint8 numObjects;
    RaObjType objects[40];
} RaObjVecType;

typedef struct {
    uint8 measEnabled;
    uint8 SGUFailed;
    uint8 statusMisalign;
    uint8 statusBlkProg;
    uint8 statusHWErr;
    float32 latency;
    uint32 timeStamp;
    float32 hostSpeed;
    float32 hostYaw;
} RaStatusFlagsType;
typedef struct {
    RaObjVecType objectsVec;
    RaStatusFlagsType statusFlags;
} RaObjListType;

/*
typedef struct {
   float32 VehSpdLgt;
   uint8 VehSpdLgtQf;
   uint8 VehSpdLgtCntr;
   uint8 VehSpdLgtChks;
} VehSpdLgtSafe1;*/

/* PerInstanceMemory type definition Traceability: [rte_sws_7133] ============*/

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_CONVERTERS_TYPES_H_

/* End of File
 * Converters_types.h=====================================================*/
