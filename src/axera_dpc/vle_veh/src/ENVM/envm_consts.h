/*
 * Copyright (C) 2018-2022 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#ifndef ENVM_CONSTS_H
#define ENVM_CONSTS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "envm_ext.h"
#include "TM_Global_Types.h"
// typedef unsigned char boolean;
// typedef signed char sint8;
// typedef unsigned char uint8;
// typedef signed short sint16;
// typedef unsigned short uint16;
// typedef signed long sint32;
// typedef unsigned long uint32;
// typedef signed long long sint64;
// // typedef unsigned long long uint64;
// typedef float float32;
// typedef double float64;

// typedef uint8 ui8_t;
// typedef uint16 ui16_t;
// typedef uint32 ui32_t;

// typedef sint8 i8_t;
// typedef sint16 i16_t;
// typedef sint32 i32_t;

// typedef float32 f32_t;
// typedef boolean bool_t;

// typedef signed int sbit32_t;
// typedef signed short sbit16_t;
// typedef signed char sbit8_t;
// typedef unsigned int ubit32_t;
// typedef unsigned short ubit16_t;
// typedef unsigned char ubit8_t;

#ifndef NULL
#define NULL ((void*)0)
#endif

typedef enum { EM_INIT, Envm_OK } StateEM_t;

typedef enum {
    EM_TIME_CP_START,
    Envm_TIME_CP_PROC,
    Envm_TIME_CP_DAP,
    EM_TIME_CP_OD_MAINTENANCE,
    EM_TIME_CP_OD_PRED_NEAR,
    EM_TIME_CP_OD_JPDA1_NEAR,
    EM_TIME_CP_OD_JPDA2_NEAR,
    EM_TIME_CP_OD_SC_NEAR,
    EM_TIME_CP_OD_SR_NEAR,
    EM_TIME_CP_OD_INTERSCAN,
    EM_TIME_CP_OD_PRED_FAR,
    EM_TIME_CP_OD_JPDA1_FAR,
    EM_TIME_CP_OD_JPDA2_FAR,
    EM_TIME_CP_OD_SC_FAR,
    EM_TIME_CP_OD_SR_FAR,
    EM_TIME_CP_OD_NEW_OBJ,
    EM_TIME_CP_OD_GRID_INPUT,
    EM_TIME_CP_OD_GRID_ASSOC,
    EM_TIME_CP_OD_CAM_ASSOC,
    EM_TIME_CP_OD_FILT,
    EM_TIME_CP_OD_ATTRIB,
    EM_TIME_CP_OD_OBJREL,
    EM_TIME_CP_OD,
    EM_TIME_CP_RD,
    EM_TIME_CP_PD,
    EM_TIME_CP_FPS,
    Envm_TIME_CP_PP,
    Envm_TIME_CP_PROC_OUT,
    Envm_TIME_CP_END,
    EM_TIME_CP_NUMER_OF_CHECKPOINTS
} Envm_t_TimeCheckpoint;

typedef uint32 Envm_t_TimeArray[EM_TIME_CP_NUMER_OF_CHECKPOINTS];

typedef enum {
    OPS_CLASS_PRIO_CUSTOM,
    OPS_CLASS_PRIO_DYNAMIC_POE_AND_CLOSE,
    OPS_CLASS_PRIO_STATIC_POE_AND_CLOSE,
    OPS_CLASS_PRIO_DYNAMIC_POE_OR_CLOSE,
    OPS_CLASS_PRIO_ONCOMING_POE_OR_CLOSE,
    OPS_CLASS_PRIO_STATIC_POE_OR_CLOSE,
    OPS_CLASS_PRIO_OTHER,
    OPS_CLASS_NUM_PRIO_CLASSES
} t_FPSPrioMajorClass2;

typedef enum { WRP_EM_INIT, WRP_Envm_OK } StateWRP_EM_t;

typedef enum {
    EM_SIGCHECK_VALID = 0,
    EM_SIGCHECK_WARNING = 1,
    EM_SIGCHECK_ERROR = 2
} EMSigCheckStatusInfo_t;
typedef enum StateFPSTag { FPS_INIT, FPS_OK, FPS_RED_QUAL } StateFPS_t;

#if (!defined(_MSC_VER))
#ifdef ALGO_INLINE
#undef ALGO_INLINE
#endif
#define ALGO_INLINE static inline
#else
#define ALGO_INLINE static inline

#endif

typedef struct {
    float32 dAmin;

    float32 dAmax;

    float32 dM;

    float32 dB;
} BML_t_LinFunctionArgs;

typedef struct {
    boolean b_valid;
    float32 f_value;
    uint32 u_TimeStamp;
} SignalHistory_t;
typedef struct {
    uint16 u_currentStatus;
    uint8 u_warningCnt;
    uint8 u_errorCnt;
} EMSigCheckStatusData_t;

typedef struct {
    uint32 u_version;
    uint32 u_cycleID;

    EMSigCheckStatusData_t VehicleDynData_Longitudinal_VeloCorr_corrFact;
    EMSigCheckStatusData_t VehicleDynData_Longitudinal_VeloCorr_corrVelo;
    EMSigCheckStatusData_t VehicleDynData_Longitudinal_MotVar_Accel;
    EMSigCheckStatusData_t VehicleDynData_Lateral_Curve_Curve;
    EMSigCheckStatusData_t VehicleDynData_Lateral_YawRate_YawRate;
    EMSigCheckStatusData_t VehicleDynData_Long_Vel_Corrected_Status;
    EMSigCheckStatusData_t VehicleDynData_Long_Vel_Status;
    EMSigCheckStatusData_t VehicleDynData_Long_Accel_Status;
    EMSigCheckStatusData_t VehicleDynData_Lat_Curve_Status;
    EMSigCheckStatusData_t VehicleDynData_Lat_YawRate_Status;
    EMSigCheckStatusData_t VehicleDynData_MotionState_Status;

    EMSigCheckStatusData_t VehicleStatData_SensorMounting_VertPos;
    EMSigCheckStatusData_t VehicleStatData_SensorMounting_LongPos;
    EMSigCheckStatusData_t VehicleStatData_SensorMounting_LongPosToCoG;
    EMSigCheckStatusData_t VehicleStatData_SensorMounting_VertPos_Status;
    EMSigCheckStatusData_t VehicleStatData_SensorMounting_LongPos_Status;
    EMSigCheckStatusData_t VehicleStatData_SensorMounting_LongPosToCoG_Status;

    EMSigCheckStatusData_t VehicleStatData_VehicleTrackWidthFront;
    EMSigCheckStatusData_t VehicleStatData_VehicleWheelBase;
    EMSigCheckStatusData_t VehicleStatData_VehicleAxleLoadDistribution;
    EMSigCheckStatusData_t VehicleStatData_VehicleTrackWidthFront_Status;
    EMSigCheckStatusData_t VehicleStatData_VehicleWheelBase_Status;
    EMSigCheckStatusData_t VehicleStatData_VehicleAxleLoadDistribution_status;

    EMSigCheckStatusData_t RSPCluListNear_ClustListHead_f_RangegateLength;
    EMSigCheckStatusData_t RSPCluListNear_ClustListHead_f_AmbFreeDopplerRange;
    EMSigCheckStatusData_t RSPCluListNear_ClustListHead_Status;

    EMSigCheckStatusData_t RSPCluListFar_ClustListHead_f_RangegateLength;
    EMSigCheckStatusData_t RSPCluListFar_ClustListHead_f_AmbFreeDopplerRange;
    EMSigCheckStatusData_t RSPCluListFar_ClustListHead_Status;

    EMSigCheckStatusData_t ALNAzimuthCorrection_a_Azimuth_Far;
    EMSigCheckStatusData_t ALNAzimuthCorrection_a_Azimuth_Near;
    EMSigCheckStatusData_t ALNAzimuth_Status;

    EMSigCheckStatusData_t Ctrl_sSigHeader_uiMeasurementCounter;
    EMSigCheckStatusData_t Ctrl_sSigHeader_uiCycleCounter;
    EMSigCheckStatusData_t Ctrl_sSigHeader_uiTimeStamp;

    EMSigCheckStatusData_t CamObjInput_sSigHeader_uiMeasurementCounter;
    EMSigCheckStatusData_t CamObjInput_sSigHeader_uiCycleCounter;
    EMSigCheckStatusData_t CamObjInput_sSigHeader_uiTimeStamp;
    EMSigCheckStatusData_t CamObjInput_u_CurrentExternalTime;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_u_Counter;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_u_Timestamp;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_dataComplete;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_commonCRC_OK;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_eventDataQualifier;
    EMSigCheckStatusData_t CamObjInput_objectListHeader_extendedQualifier;
} EMSigCheckMeasFreezeMem_t;

#define Envm_N_OBJECTS 40

typedef uint8 Envm_t_ObjectPrioIndexArray[Envm_N_OBJECTS];
#define RSP_NUM_OFCLUSTERS_NEAR 768
#define TUE_OPS_PRIO_NUM_QUOTA (OPS_CLASS_NUM_PRIO_CLASSES)
#define OPS_CLASS_PRIO_QUOTA_CUSTOM \
    (0u)  // Quota for customer
          // requirement object : 0/20
#define OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_AND_CLOSE \
    (19u)  // Quota for dynamic object
           // with high POE and close to ego trajectory : 10/20
#define OPS_CLASS_PRIO_QUOTA_STATIC_POE_AND_CLOSE \
    (13u)  // Quota for static object
           // with high POE and close to ego trajectory : 7/20
#define OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_OR_CLOSE \
    (3u)  // Quota for dynamic object
          // with high POE or close to ego trajectory : 0/20
#define OPS_CLASS_PRIO_QUOTA_ONCOMING_POE_OR_CLOSE \
    (5u)  // Quota for oncoming object
          // with high POE or close to ego trajectoryy : 3/20
#define OPS_CLASS_PRIO_QUOTA_STATIC_POE_OR_CLOSE \
    (0u)  // Quota for static object
          // with high POE or close to ego trajectory : 0/20
#define OPS_CLASS_PRIO_QUOTA_OTHER (0u)  // Quota for other object : 0/20
#define SYS_NUM_OF_SCANS 1
#define SYS_SCAN_NEAR 0U
#define SYS_SCAN_FAR 1U

typedef unsigned char a_AzAngleValidFar_array_t[1];

typedef unsigned char a_AzAngleValidNear_array_t[RSP_NUM_OFCLUSTERS_NEAR];

static const uint8 tue_OpsPrioQuota[TUE_OPS_PRIO_NUM_QUOTA] = {
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_CUSTOM */
    OPS_CLASS_PRIO_QUOTA_CUSTOM,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_AND_CLOSE */
    OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_AND_CLOSE,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_STATIC_POE_AND_CLOSE */
    OPS_CLASS_PRIO_QUOTA_STATIC_POE_AND_CLOSE,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_OR_CLOSE */
    OPS_CLASS_PRIO_QUOTA_DYNAMIC_POE_OR_CLOSE,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_ONCOMING_POE_OR_CLOSE */
    OPS_CLASS_PRIO_QUOTA_ONCOMING_POE_OR_CLOSE,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_STATIC_POE_OR_CLOSE */
    OPS_CLASS_PRIO_QUOTA_STATIC_POE_OR_CLOSE,
    /* Quota for prio class OPS_CLASS_PRIO_QUOTA_OTHER */
    OPS_CLASS_PRIO_QUOTA_OTHER};
typedef struct {
    float32 fAcellXLast;
    float32 fYawRateLast;
    uint32 u_LastRSPTimeStamp;

    uint32 u_StartEMTimestamp;
    float32 f_RCSThresholdReduction[SYS_NUM_OF_SCANS];
    float32 f_CPedSituation_ActiveTime;
    float32 f_TimeNCAPVehicleConditionsFullfiled;

    uint8 u_i_KFCoupledObjNr;
    uint8 u_RSPListInvalidCounter;
    uint8 u_MeasFreezeCounter;
    uint8 u_MeasFreezeCounterCallback;
    uint8 u_MeasFreezeCounter_MeasBuffer;
    uint8 u_MeasFreezeCounterCallback_MeasBuffer;
    uint8 u_MeasFreezeBufferedFailureCounter;
} EMGlob_t;

typedef struct {
    ui32_t uiEM_ALL;
    ui32_t uiDAP;
    ui32_t uiOD;
    ui32_t uiRD;
    ui32_t uiEM_LIB;
    ui32_t uiPD;
    ui32_t uiEM_APPL_ALL;
    ui32_t EM_VERSION_AS_TEXT;
    ui32_t uiALGO;
} EMVersions_t;

typedef struct {
    float32 fCycleTimeAverage;
    EMVersions_t Versions;
    uint16 uiCycleCounter;
    bool_t bFirstCycleDone;
    bool_t bStartupDone;
    uint8 uiStartupCycles;
    sint8 siRemainingRSPStartupCycles;
    sint8 siRemainingVDYStartupCycles;
    bool_t bExtEnvmCycleViolation;
    bool_t bExtRSPCycleViolation;
    bool_t bValidInputPorts;
    uint8 eEnvmOpModeRequest;
    uint8 eEnvmOpMode;
} EnvmFrame_t;

typedef struct {
    Envm_t_GenObjectList* pEnvmGenObjectList;
    Envm_t_CRObjectList* pEnvmTechObjectList;
} EnvmObjListDataPointer_t;

typedef struct {
    uint8 uiTunnelProb;
    uint8 uiFilteredTimeIndicator;  // filtered value of u8TimeIndicator from
                                    // mini-eye
} SceneRecognize_t;

typedef struct {
    uint8 ucAccObjQuality;
    uint8 ucAccSelBits;
} ACCPresel_t;

typedef struct {
    uint8 ObjectId;
    sint32 RawObjectID;
    Kinematic_t Kinematic;
    Geometry_t Geometry;
    Attributes_t Attributes;
    General_t General;
    Qualifiers_t Qualifiers;
    SensorSpecific_t SensorSpecific;
    ACCPresel_t ACCPresel;
    EBAPresel_t EBAPresel;
    LegacyObj_t Legacy;
} Objects_t;

typedef Objects_t Objects_array_t[Envm_NR_PRIVOBJECTS];

typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint32 uiTimeStamp;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
    HeaderObjList_t HeaderObjList;
    // Ticket:http://119.3.139.124:10101/tickets/BSW_100.git/10 changed by
    // guotao 20200417 start
    SceneRecognize_t
        sSceneDescription;  // add a new structure for scene recognize: tunnel
                            // recognize, time indicator...
    // Ticket:http://119.3.139.124:10101/tickets/BSW_100.git/10 changed by
    // guotao 20200417 end
    Objects_array_t Objects;
} ObjectList_t;

typedef struct {
    uint32 u_VersionNumber;
    ENVMSignalHeader_t sSigHeader;
} EM_t_CameraLaneInputData;

typedef struct {
    const EM_t_CameraLaneInputData* p_CamLaneInput;
    Envm_t_CameraLaneOutputFctData* pCameraLaneData;
} CEnvmData_t;

typedef Objects_t SRRIDManegeObjects_array_t[TUE_SRR_RADAR_RAW_OBJECT_NUM];
typedef Objects_t FusionIDManegeObjects_array_t[EM_Fusion_GEN_OBJECT_NUM];

typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint32 uiTimeStamp;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
    HeaderObjList_t HeaderObjList;
    SRRIDManegeObjects_array_t Objects;
} SRRIDManegeObjList_t;
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint32 uiTimeStamp;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
    HeaderObjList_t HeaderObjList;
    FusionIDManegeObjects_array_t Objects;
} FusionIDManegeObjList_t;

typedef struct {
    EMGlob_t* pPrivGlob;
    EnvmFrame_t* pFrame;
    const AssessedObjList_t* pPubFctObj;
    const VEDVehDyn_t* pEgoDynRaw;
    const VEDVehPar_t* pGlobEgoStatic;
    const EnvmNvmIn_t* pNvmIn;
    const BSW_s_AlgoParameters_t* pAlgoParameters;
    const uint16* p_HWSample;
    EnvmObjListDataPointer_t* p_ObjListPointers;
    ObjectList_t* pPrivObjList;
    PreSeletObjectList_t* pObjPreSeletList;
    const ExtObjectList_t* pExtObjList;
    // const CamObjectList* pCamObject;
    const ExtObjectList_t* pExtFusionObjList;
    const TSRObjectList_t* pTSRObject;
    SRRIDManegeObjList_t* pSRRIDManageObjectList;
    FusionIDManegeObjList_t* pFusionIDManageObjectList;
    VEDVehDyn_t* pEgoDynObjSync;
    VEDVehDyn_t* pEgoDynTgtSync;
    ECAMtCyclEnvmode_t* pECAMtCyclEnvmode;
    EnvmNvmOut_t* pNvmOut;
    CEnvmData_t* pCEnvmData;
    EnvmDEnvmOut_t* pDemEvents;
} EnvmData_t;

typedef struct {
    float32 f_X;
    float32 f_Y;
    float32 f_DistToTraj;
    float32 f_DistOnTraj;
} BML_t_TrajRefPoint;

extern CEnvmData_t CEnvmData;

/*! sub-module state */
extern StateEM_t StateEM;

/*! frame (cycle time, cycle counter, operation mode ...) */
extern EnvmFrame_t EnvmFrame;

/*! EM time measurement info */
extern Envm_t_TimeArray EM_a_TimeArray;

/*! Memory for the internal EM object list (access via EnvmData.pPubEmObj) */
extern ObjectList_t EnvmInternalObj;

extern PreSeletObjectList_t EMPreSeletObj;

extern SRRIDManegeObjList_t SRRIDManageObjectList;
extern FusionIDManegeObjList_t FusionIDManageObjectList;

// extern ExtObjectList_t  EnvmExternalObj;
// extern ExtSRRObjList_array_t ExtSRRObjects;
//
// extern CamObjectList CamObj_Sense;

/*! Array to describe the transformation between the EM-internal object ID's and
 * the FCT object ID's, i.e. a_EnvmObjIDToFCTObjIDConvert[i_EMObjID] =
 * i_FCTObjID */
extern sint8 a_EnvmObjIDToFCTObjIDConvert[(uint32)Envm_NR_PRIVOBJECTS];

/* em private glob data*/
extern EMGlob_t EMGlob;

/* storage for the new object list */
extern EnvmData_t EnvmData;

#define Envm_ERRORTRAP_MAX_FILELENGTH (32u)
#define Envm_ERRORTRAP_MAXERRORS (5u)
#define GET_EGO_RAW_DATA_PTR EnvmData.pEgoDynRaw

#define Envm_MOD_INVALID_INPUT_SIGNALS 3U
#ifndef AL_SIG_STATE_INIT
#define AL_SIG_STATE_INIT 0U
#endif
#ifndef AL_SIG_STATE_OK
#define AL_SIG_STATE_OK 1U
#endif
#ifndef AL_SIG_STATE_INVALID
#define AL_SIG_STATE_INVALID 2U
#endif

#ifndef VED_LONG_MOT_STATE_MOVE
#define VED_LONG_MOT_STATE_MOVE 0U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_FWD
#define VED_LONG_MOT_STATE_MOVE_FWD 1U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_RWD
#define VED_LONG_MOT_STATE_MOVE_RWD 2U
#endif
#ifndef VED_LONG_MOT_STATE_STDST
#define VED_LONG_MOT_STATE_STDST 3U
#endif
#define EM_ERRORTRAP_TYPE_UNKNOWN (0u)
#define EM_ERRORTRAP_TYPE_WARNING (1u)
#define EM_ERRORTRAP_TYPE_ERROR (2u)
#define Envm_ERRORTRAP_TYPE_EXCEPTION (3u)
#define ODDELAY_f_drvIntCurveObjLatency (0.0f)
#define ODDELAY_f_yawRateObjLatency (20.0f)
#define ODDELAY_f_yawRateTgtLatency (0.0f)
#define ODDELAY_f_yawRateLatencyJitter (80.0f)
#define ODDELAY_f_slipAngleTgtLatency (0.0f)
#define ODDELAY_f_slipAngleObjLatency (0.0f)
#define ODDELAY_f_speedXObjLatency (0.0f)
#define ODDELAY_f_accelXObjLatency (60.0f)
#define BSW_ALGOPARAMETERS_INTFVER 32U
#define CFG_Envm_SWITCH_ON 1
#define CFG_Envm_EGO_YAWRATE_COMPENSATION (CFG_Envm_SWITCH_ON)
#ifndef Envm_MOD_STARTUP
#define Envm_MOD_STARTUP 0U
#endif
#ifndef Envm_MOD_RUNNING
#define Envm_MOD_RUNNING 1U
#endif
#ifndef Envm_MOD_PAUSE
#define Envm_MOD_PAUSE 2U
#endif
#define CFG_Envm_GENERIC_OBJECT_INTERFACE_ACTIVE 1
#define DELAY_BUFFER_SIZE (3UL)
#define Envm_OBJECT_LIST_INTFVER (9U)
#define Envm_TUE_CAM_OBJECT_LIST_INTFVER (1U)
#define GET_FRAME_DATA EnvmData.pFrame
#define Envm_CYCLE_COUNTER GET_FRAME_DATA->uiCycleCounter
#define TASK_CYCLE_TIME_50 0.050F
#define GET_Envm_EXT_OBJ_DATA_PTR EnvmData.pExtObjList
#define RADIUS_INIT_VALUE (999999.F)                 /* in m */
#define CURVATURE_USE_CIRCLE_EQUATION (1.F / 1000.F) /* in 1/m */
#define GET_EGO_OBJ_SYNC_DATA_PTR EnvmData.pEgoDynObjSync
#define EGO_CURVE_OBJ_SYNC GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.Curve
#define TU_OBJECT_DYNPROP_MOVING (0)
#define TU_OBJECT_DYNPROP_STATIONARY (1)
#define TU_OBJECT_DYNPROP_ONCOMING (2)
#define TU_OBJECT_DYNPROP_STATIONARY_CANDITATE (3)
#define TU_OBJECT_DYNPROP_UNKNOWN (4)
#define TU_OBJECT_DYNPROP_CROSSING_STATIONARY (5)
#define TU_OBJECT_DYNPROP_CROSSING_MOVING (6)
#define TU_OBJECT_DYNPROP_STOPPED (7)
#define TUE_OPS_DISTANCE_TO_TRAJECTORY_THRESHOLD \
    4  // Distance to ego trajectory threshold set for object pre-selection
       // check in tue_OpsPrioQuota

#define EM_FRAME_DATA EnvmData.pFrame
#define GET_Envm_INT_OBJ_DATA_PTR EnvmData.pPrivObjList
#define EM_GET_BSW_ALGO_PARAMS_PTR EnvmData.pAlgoParameters
#define Envm_GET_NVM_IN_PTR EnvmData.pNvmIn
#define GET_FCT_PUB_OBJ_DATA_PTR EnvmData.pPubFctObj
#define GET_EGO_STATIC_DATA_PTR EnvmData.pGlobEgoStatic
// EnvmData.pCustomData->pCustomIn
#define Envm_FCT_CYCLE_INTFVER 1U
#define Envm_GEN_OBJECT_LIST_INTFVER 196608U
#define Envm_CR_OBJECT_LIST_INTFVER 6U
#define Envm_GEN_OBJECT_INTFVER 12U
#define VED_VEH_DYN_INTFVER 8U
#define EM_GET_DEM_EVENTS_PTR EnvmData.pDemEvents
#define EM_GET_NVM_OUT_PTR EnvmData.pNvmOut
#define UNREFERENCED_FORMAL_PARAMETER(x) (void)(x)

#ifndef MT_STATE_DELETED
#define MT_STATE_DELETED (0U)
#endif
#ifndef MT_STATE_NEW
#define MT_STATE_NEW (1U)
#endif
#ifndef MT_STATE_MEASURED
#define MT_STATE_MEASURED (2U)
#endif
#ifndef MT_STATE_PREDICTED
#define MT_STATE_PREDICTED (3U)
#endif
#ifndef MT_STATE_MERGE_DELETED
#define MT_STATE_MERGE_DELETED (4U)
#endif
#ifndef MT_STATE_MERGE_NEW
#define MT_STATE_MERGE_NEW (5U)
#endif
#define TUE_OPS_POE_THRESHOLD \
    0.9  // POE threshold set for object pre-selection check in tue_OpsPrioQuota
#define EGO_SPEED_X_CORRECTED \
    GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVelo
#define GET_Envm_GEN_OBJ_DATA_PTR EnvmData.p_ObjListPointers->pEnvmGenObjectList
#define GET_Envm_TECH_OBJ_DATA_PTR \
    EnvmData.p_ObjListPointers->pEnvmTechObjectList
#define Envm_INVALID_ID_INDEX ((ui8_t)254)
#define Envm_GEN_OBJECT_SM_ID_NONE 255

#ifndef OBJCLASS_POINT
#define OBJCLASS_POINT (0U)
#endif
#ifndef OBJCLASS_CAR
#define OBJCLASS_CAR (1U)
#endif
#ifndef OBJCLASS_TRUCK
#define OBJCLASS_TRUCK (2U)
#endif
#ifndef OBJCLASS_PEDESTRIAN
#define OBJCLASS_PEDESTRIAN (3U)
#endif
#ifndef OBJCLASS_MOTORCYCLE
#define OBJCLASS_MOTORCYCLE (4U)
#endif
#ifndef OBJCLASS_BICYCLE
#define OBJCLASS_BICYCLE (5U)
#endif
#ifndef OBJCLASS_WIDE
#define OBJCLASS_WIDE (6U)
#endif
#ifndef OBJCLASS_UNCLASSIFIED
#define OBJCLASS_UNCLASSIFIED (7U)
#endif
#ifndef OBJCLASS_CAM_CROSSING
#define OBJCLASS_CAM_CROSSING (8U)
#endif
#ifndef OBJCLASS_CAM_GEN
#define OBJCLASS_CAM_GEN (9U)
#endif

#ifndef Envm_GEN_OBJECT_CLASS_POINT
#define Envm_GEN_OBJECT_CLASS_POINT 0U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_CAR
#define Envm_GEN_OBJECT_CLASS_CAR 1U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_TRUCK
#define Envm_GEN_OBJECT_CLASS_TRUCK 2U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_PEDESTRIAN
#define Envm_GEN_OBJECT_CLASS_PEDESTRIAN 3U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_MOTORCYCLE
#define Envm_GEN_OBJECT_CLASS_MOTORCYCLE 4U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_BICYCLE
#define Envm_GEN_OBJECT_CLASS_BICYCLE 5U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_WIDE
#define Envm_GEN_OBJECT_CLASS_WIDE 6U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_UNCLASSIFIED
#define Envm_GEN_OBJECT_CLASS_UNCLASSIFIED 7U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_TL
#define Envm_GEN_OBJECT_CLASS_TL 8U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_CLASS_MAX_DIFF_TYPES 9U
#endif

#ifndef OBJECT_MOVSTATE_STATIONARY
#define OBJECT_MOVSTATE_STATIONARY (0U)
#endif
#ifndef OBJECT_MOVSTATE_STOPPED
#define OBJECT_MOVSTATE_STOPPED (1U)
#endif
#ifndef OBJECT_MOVSTATE_MOVING
#define OBJECT_MOVSTATE_MOVING (2U)
#endif

#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_MOVING
#define Envm_GEN_OBJECT_DYN_PROPERTY_MOVING 0U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY
#define Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY 1U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING
#define Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING 2U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT
#define Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT 3U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT
#define Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT 4U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN
#define Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN 5U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED
#define Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED 6U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES 7U
#endif

#ifndef Envm_GEN_OBJECT_OCCL_NONE
#define Envm_GEN_OBJECT_OCCL_NONE 0U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_UNKNOWN
#define Envm_GEN_OBJECT_OCCL_UNKNOWN 1U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_PARTLY
#define Envm_GEN_OBJECT_OCCL_PARTLY 2U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_FULL
#define Envm_GEN_OBJECT_OCCL_FULL 3U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_OCCL_MAX_DIFF_TYPES 4U
#endif

#define OBJ_INDEX_NO_OBJECT ((sint8)-1)
#ifndef Envm_GEN_OBJECT_SM_ID_NONE
#define Envm_GEN_OBJECT_SM_ID_NONE 255
#endif
#ifndef Envm_GEN_OBJECT_SM_ID_UNKNOWN
#define Envm_GEN_OBJECT_SM_ID_UNKNOWN 254
#endif

#ifndef Envm_GEN_OBJECT_MT_STATE_DELETED
#define Envm_GEN_OBJECT_MT_STATE_DELETED 0U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_NEW
#define Envm_GEN_OBJECT_MT_STATE_NEW 1U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_MEASURED
#define Envm_GEN_OBJECT_MT_STATE_MEASURED 2U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_PREDICTED
#define Envm_GEN_OBJECT_MT_STATE_PREDICTED 3U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_INACTIVE
#define Envm_GEN_OBJECT_MT_STATE_INACTIVE 4U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_MT_STATE_MAX_DIFF_TYPES 5U
#endif
#define VED_GET_IO_STATE(pos_, val_) ((val_)[(pos_)])

#ifndef VED_SOUT_POS_CURVE
#define VED_SOUT_POS_CURVE 0U
#endif
#ifndef VED_SOUT_POS_YWR
#define VED_SOUT_POS_YWR 1U
#endif
#ifndef VED_SOUT_POS_DRCRV
#define VED_SOUT_POS_DRCRV 2U
#endif
#ifndef VED_SOUT_POS_VEL
#define VED_SOUT_POS_VEL 3U
#endif
#ifndef VED_SOUT_POS_ACCEL
#define VED_SOUT_POS_ACCEL 4U
#endif
#ifndef VED_SOUT_POS_MSTAT
#define VED_SOUT_POS_MSTAT 5U
#endif
#ifndef VED_SOUT_POS_VCORR
#define VED_SOUT_POS_VCORR 6U
#endif
#ifndef VED_SOUT_POS_SSA
#define VED_SOUT_POS_SSA 7U
#endif
#ifndef VED_SOUT_POS_LATACC
#define VED_SOUT_POS_LATACC 8U
#endif
#ifndef VED_SOUT_POS_MAX
#define VED_SOUT_POS_MAX 12U
#endif
#define EM_SIGCHECK_VERSION (0x010000uL)
#define EGO_SPEED_X_CORRECTED_STATE \
    VED_GET_IO_STATE(VED_SOUT_POS_VCORR, GET_EGO_RAW_DATA_PTR->State)
#define EM_SIGCHECK_VELOCORR_CORRFACT_MIN (0.88f)
#define Envm_SIGCHECK_VELOCORR_CORRFACT_MAX (1.12f)
#define Envm_SIGCHECK_STATUS_OK 0x0000
#define EGO_SPEED_X_STATE \
    VED_GET_IO_STATE(VED_SOUT_POS_VEL, GET_EGO_RAW_DATA_PTR->State)
#define EM_SIGCHECK_STATUS_FLAG_INVALID 0x0040
#ifndef float32_LowerLimit
#define float32_LowerLimit -1.175494351E38f
#endif
#ifndef float32_UpperLimit
#define float32_UpperLimit 3.402823466E38f
#endif
#define EM_SIGCHECK_VELOCORR_CORRVELO_MIN (float32_LowerLimit)
#define EM_SIGCHECK_VELOCORR_CORRVELO_MAX (115.0f)
#define EGO_SPEED_X_CORRECTED_VAR \
    GET_EGO_RAW_DATA_PTR->Longitudinal.VeloCorr.corrVeloVar
#define EM_SIGCHECK_STD_DEV_MAX_FACTOR (0.05f)
#define EM_SIGCHECK_STD_DEV_MAX_BASE_EGO_SPEED_X_CORRECTED (0.5f)
#define EM_SIGCHECK_VELOCORR_CORRVELO_DERIVATION_MIN (-30.0f)
#define EM_SIGCHECK_VELOCORR_CORRVELO_DERIVATION_MAX (30.0f)
#define EGO_ACCEL_X_STATE \
    VED_GET_IO_STATE(VED_SOUT_POS_ACCEL, GET_EGO_RAW_DATA_PTR->State)
#define EGO_ACCEL_X_RAW GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.Accel
#define EM_SIGCHECK_MOTVAR_ACCEL_MIN (-15.0f)
#define EM_SIGCHECK_MOTVAR_ACCEL_MAX (15.0f)
#define EGO_ACCEL_X_VAR_RAW GET_EGO_RAW_DATA_PTR->Longitudinal.MotVar.varAccel
#define Envm_SIGCHECK_STD_DEV_MAX_BASE_EGO_ACCEL_X_RAW (0.5f)
#define EGO_CURVE_RAW GET_EGO_RAW_DATA_PTR->Lateral.Curve.Curve
#define Envm_SIGCHECK_CURVE_MIN (-0.2f)
#define Envm_SIGCHECK_CURVE_MAX (0.2f)
#define EGO_YAW_RATE_STATE \
    VED_GET_IO_STATE(VED_SOUT_POS_YWR, GET_EGO_RAW_DATA_PTR->State)
#define EGO_YAW_RATE_RAW GET_EGO_RAW_DATA_PTR->Lateral.YawRate.YawRate
#define EM_SIGCHECK_YAWRATE_MIN (-150.0f)
#define EM_SIGCHECK_YAWRATE_MAX (150.0f)
#define EGO_YAW_RATE_VAR_RAW GET_EGO_RAW_DATA_PTR->Lateral.YawRate.Variance
#define EM_SIGCHECK_STD_DEV_MAX_BASE_EGO_YAW_RATE_RAW (0.02f)
#define EM_SIGCHECK_YAWRATE_DERIVATION_MIN (float32_LowerLimit)
#define Envm_SIGCHECK_YAWRATE_DERIVATION_MAX (float32_UpperLimit)
#define EGO_MOTION_STATE_STATE \
    VED_GET_IO_STATE(VED_SOUT_POS_MSTAT, GET_EGO_RAW_DATA_PTR->State)

#ifndef VEH_PAR_SEN_MOUNT_LAT_POS
#define VEH_PAR_SEN_MOUNT_LAT_POS 0U
#endif
#ifndef VEH_PAR_SEN_MOUNT_LONG_POS
#define VEH_PAR_SEN_MOUNT_LONG_POS 1U
#endif
#ifndef VEH_PAR_SEN_MOUNT_VERT_POS
#define VEH_PAR_SEN_MOUNT_VERT_POS 2U
#endif
#ifndef VEH_PAR_SEN_MOUNT_LONGPOS_TO_COG
#define VEH_PAR_SEN_MOUNT_LONGPOS_TO_COG 3U
#endif
#ifndef VEH_PAR_SEN_MOUNT_PITCH_ANGLE
#define VEH_PAR_SEN_MOUNT_PITCH_ANGLE 4U
#endif
#ifndef VEH_PAR_SEN_MOUNT_ORIENTATION
#define VEH_PAR_SEN_MOUNT_ORIENTATION 5U
#endif
#ifndef VEH_PAR_SEN_MOUNT_ROLL_ANGLE
#define VEH_PAR_SEN_MOUNT_ROLL_ANGLE 6U
#endif
#ifndef VEH_PAR_SEN_MOUNT_YAW_ANGLE
#define VEH_PAR_SEN_MOUNT_YAW_ANGLE 7U
#endif
#ifndef VEH_PAR_SEN_MOUNT_MAX
#define VEH_PAR_SEN_MOUNT_MAX 8U
#endif
#define SENSOR_MOUNTING GET_EGO_STATIC_DATA_PTR->SensorMounting
#define SENSOR_Z_POSITION SENSOR_MOUNTING.VertPos
#define Envm_SIGCHECK_VERTPOS_MIN (0.0f)
#define Envm_SIGCHECK_VERTPOS_MAX (3.0f)
#define SENSOR_X_POSITION_CoG SENSOR_MOUNTING.LongPosToCoG
#define EM_SIGCHECK_INVALID_SENSOR_X_POSITION_CoG (0.0f)
#define SENSOR_X_POSITION SENSOR_MOUNTING.LongPos
#define EM_SIGCHECK_INVALID_SENSOR_X_POSITION (0.0f)

#ifndef VED_PAR_POS_SSG
#define VED_PAR_POS_SSG 0U
#endif
#ifndef VED_PAR_POS_SWARAT
#define VED_PAR_POS_SWARAT 1U
#endif
#ifndef VED_PAR_POS_WBASE
#define VED_PAR_POS_WBASE 2U
#endif
#ifndef VED_PAR_POS_TWDFR
#define VED_PAR_POS_TWDFR 3U
#endif
#ifndef VED_PAR_POS_TWDRE
#define VED_PAR_POS_TWDRE 4U
#endif
#ifndef VED_PAR_POS_VEHWGT
#define VED_PAR_POS_VEHWGT 5U
#endif
#ifndef VED_PAR_POS_COGH
#define VED_PAR_POS_COGH 6U
#endif
#ifndef VED_PAR_POS_AXLD
#define VED_PAR_POS_AXLD 7U
#endif
#ifndef VED_PAR_POS_WHLDFR
#define VED_PAR_POS_WHLDFR 8U
#endif
#ifndef VED_PAR_POS_WHLDRE
#define VED_PAR_POS_WHLDRE 9U
#endif
#ifndef VED_PAR_POS_WHLCIR
#define VED_PAR_POS_WHLCIR 10U
#endif
#ifndef VED_PAR_POS_DRVAXL
#define VED_PAR_POS_DRVAXL 11U
#endif
#ifndef VED_PAR_POS_WTCKSREV
#define VED_PAR_POS_WTCKSREV 12U
#endif
#ifndef VED_PAR_POS_CSFR
#define VED_PAR_POS_CSFR 13U
#endif
#ifndef VED_PAR_POS_CSRE
#define VED_PAR_POS_CSRE 14U
#endif
#ifndef VED_PAR_POS_MAX
#define VED_PAR_POS_MAX 16U
#endif
#define EGO_VEHICLE_WHEEL_BASE GET_EGO_STATIC_DATA_PTR->VehParMain.WheelBase
#define EM_SIGCHECK_INVALID_WHEEL_BASE (0.0f)
#define EGO_VEHICLE_TRACK_WIDTH_FRONT \
    GET_EGO_STATIC_DATA_PTR->VehParMain.TrackWidthFront
#define Envm_SIGCHECK_INVALID_TRACK_WIDTH_FRONT (0.0f)
#define EGO_VEHICLE_AXLE_LOAD_DISTR \
    GET_EGO_STATIC_DATA_PTR->VehParMain.AxisLoadDistr
#define Envm_SIGCHECK_INVALID_AXLE_LOAD_DISTR (0.0f)
#define Envm_SIGCHECK_STATUS_FLAG_ERROR 0x0002
#ifndef UInt8_LowerLimit
#define UInt8_LowerLimit 0
#endif
#ifndef UInt8_UpperLimit
#define UInt8_UpperLimit 255
#endif
#define EM_SIGCHECK_STATUS_FLAG_WARNING 0x0001
#define EM_SIGCHECK_STATUS_FLAG_RANGE 0x0004
#define EM_SIGCHECK_STATUS_FLAG_VARIANCE 0x0010
#define EM_SIGCHECK_STATUS_FLAG_DERIVATION 0x0020

#ifndef OBJECT_PROPERTY_MOVING
#define OBJECT_PROPERTY_MOVING (0U)
#endif
#ifndef OBJECT_PROPERTY_STATIONARY
#define OBJECT_PROPERTY_STATIONARY (1U)
#endif
#ifndef OBJECT_PROPERTY_ONCOMING
#define OBJECT_PROPERTY_ONCOMING (2U)
#endif

#define FUN_PRESEL_ACC_DROP_QUAL (0u)
#define FUN_PRESEL_ACC_STAT_OBSTACLE (30u)
#define FUN_PRESEL_ACC_MIN_KEEP_OBJ_QUAL (40u)
#define FUN_PRESEL_ACC_MIN_OBJ_QUAL (70u)
#define FUN_PRESEL_ACC_HIGH_CLUST_VAR_OBJ_QUAL (75u)
#define FUN_PRESEL_ACC_LOW_CLUST_VAR_OBJ_QUAL (76u)

#define FUN_PRESEL_ACC_OCCLUSION_OBJ_QUAL (80u)
#define FUN_PRESEL_ACC_MIN_INLANE_OBJ_QUAL (85u)

ALGO_INLINE Objects_t* Envm_p_GetPrivObject(sint8 iObj) {
    return &(EnvmData.pPrivObjList->Objects[iObj]);
}

#ifndef FPS_ACC_BIT_POE
#define FPS_ACC_BIT_POE (1U)
#endif
#ifndef FPS_ACC_BIT_RCS
#define FPS_ACC_BIT_RCS (2U)
#endif
#ifndef FPS_ACC_BIT_TGT
#define FPS_ACC_BIT_TGT (4U)
#endif
#ifndef FPS_ACC_BIT_LATV
#define FPS_ACC_BIT_LATV (8U)
#endif
#ifndef FPS_ACC_BIT_LV
#define FPS_ACC_BIT_LV (16U)
#endif
#ifndef FPS_ACC_BIT_NR
#define FPS_ACC_BIT_NR (32U)
#endif
#ifndef FPS_ACC_BIT_PED
#define FPS_ACC_BIT_PED (64U)
#endif
#ifndef FPS_ACC_BIT_POBS
#define FPS_ACC_BIT_POBS (128U)
#endif
#define Envm_INT_GET_Envm_OBJ(iObj) GET_Envm_INT_OBJ_DATA_PTR->Objects[iObj]
#define Envm_INT_OBJ_ATTRIBUTES(iObj) Envm_INT_GET_Envm_OBJ(iObj).Attributes
#define EM_INT_OBJ_DYNAMIC_PROPERTY(iObj) \
    Envm_INT_OBJ_ATTRIBUTES(iObj).eDynamicProperty
#define EGO_SPEED_X_OBJ_SYNC \
    GET_EGO_OBJ_SYNC_DATA_PTR->Longitudinal.MotVar.Velocity
#ifndef OBJ_NOT_OOI
#define OBJ_NOT_OOI -1
#endif
#ifndef OBJ_NEXT_OOI
#define OBJ_NEXT_OOI 0
#endif
#define EGO_CURVE_GRAD_OBJ_SYNC \
    GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.Gradient
#define EGO_YAW_RATE_OBJ_SYNC GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.YawRate.YawRate
#define EM_INT_OBJ_NUMBER_OF_OBJ_USED \
    GET_Envm_INT_OBJ_DATA_PTR->HeaderObjList.iNumOfUsedObjects
#define Envm_INT_OBJ_INDEX_DISTX_SORTED \
    GET_Envm_INT_OBJ_DATA_PTR->HeaderObjList.iDistXSortedObjectList
// #define C_F32_DELTA ((float32)0.0001f)
static const sint8 FPSLookupMTF2MTD[256] = {
    /* Lookup values for 0 to 15 */
    -36,
    -34,
    -32,
    -30,
    -30,
    -28,
    -26,
    -24,
    -28,
    -26,
    -24,
    -22,
    -22,
    -20,
    -18,
    -16,
    /* Lookup values for 16 to 31 */
    -26,
    -24,
    -22,
    -20,
    -20,
    -18,
    -16,
    -14,
    -18,
    -16,
    -14,
    -12,
    -12,
    -10,
    -8,
    -6,
    /* Lookup values for 32 to 47 */
    -24,
    -22,
    -20,
    -18,
    -18,
    -16,
    -14,
    -12,
    -16,
    -14,
    -12,
    -10,
    -10,
    -8,
    -6,
    -4,
    /* Lookup values for 48 to 63 */
    -14,
    -12,
    -10,
    -8,
    -8,
    -6,
    -4,
    -2,
    -6,
    -4,
    -2,
    0,
    0,
    2,
    4,
    6,
    /* Lookup values for 64 to 79 */
    -22,
    -20,
    -18,
    -16,
    -16,
    -14,
    -12,
    -10,
    -14,
    -12,
    -10,
    -8,
    -8,
    -6,
    -4,
    -2,
    /* Lookup values for 80 to 95 */
    -12,
    -10,
    -8,
    -6,
    -6,
    -4,
    -2,
    0,
    -4,
    -2,
    0,
    2,
    2,
    4,
    6,
    8,
    /* Lookup values for 96 to 111 */
    -10,
    -8,
    -6,
    -4,
    -4,
    -2,
    0,
    2,
    -2,
    0,
    2,
    4,
    4,
    6,
    8,
    10,
    /* Lookup values for 112 to 127 */
    0,
    2,
    4,
    6,
    6,
    8,
    10,
    12,
    8,
    10,
    12,
    14,
    14,
    16,
    18,
    20,
    /* Lookup values for 128 to 143 */
    -20,
    -18,
    -16,
    -14,
    -14,
    -12,
    -10,
    -8,
    -12,
    -10,
    -8,
    -6,
    -6,
    -4,
    -2,
    0,
    /* Lookup values for 144 to 159 */
    -10,
    -8,
    -6,
    -4,
    -4,
    -2,
    0,
    2,
    -2,
    0,
    2,
    4,
    4,
    6,
    8,
    10,
    /* Lookup values for 160 to 175 */
    -8,
    -6,
    -4,
    -2,
    -2,
    0,
    2,
    4,
    0,
    2,
    4,
    6,
    6,
    8,
    10,
    12,
    /* Lookup values for 176 to 191 */
    2,
    4,
    6,
    8,
    8,
    10,
    12,
    14,
    10,
    12,
    14,
    16,
    16,
    18,
    20,
    22,
    /* Lookup values for 192 to 207 */
    -6,
    -4,
    -2,
    0,
    0,
    2,
    4,
    6,
    2,
    4,
    6,
    8,
    8,
    10,
    12,
    14,
    /* Lookup values for 208 to 223 */
    4,
    6,
    8,
    10,
    10,
    12,
    14,
    16,
    12,
    14,
    16,
    18,
    18,
    20,
    22,
    24,
    /* Lookup values for 224 to 239 */
    6,
    8,
    10,
    12,
    12,
    14,
    16,
    18,
    14,
    16,
    18,
    20,
    20,
    22,
    24,
    26,
    /* Lookup values for 240 to 255 */
    16,
    18,
    20,
    22,
    22,
    24,
    26,
    28,
    24,
    26,
    28,
    30,
    30,
    32,
    34,
    36,
};
#define C_KMH_MS (3.6F)
#define EGO_MOTION_STATE_RAW GET_EGO_RAW_DATA_PTR->MotionState.MotState
#define C_MILLISEC_SEC (1000.0F)
#define Envm_INT_OBJ_GET_EBA_MOV_PRESEL_QUALITY(iObj) \
    Envm_INT_GET_Envm_OBJ(iObj).EBAPresel.ucEbaMovingObjQuality
#define EM_INT_OBJ_GET_EBA_CROSSING_PED_PRESEL(iObj) \
    Envm_INT_GET_Envm_OBJ(iObj).EBAPresel.bCrossingPedEbaPresel
#define EM_INT_OBJ_GET_EBA_INHIBITIONMASK(iObj) \
    Envm_INT_GET_Envm_OBJ(iObj).EBAPresel.eEbaInhibitionMask
#define Envm_INT_OBJ_GET_EBA_HYP_CAT(iObj) \
    Envm_INT_GET_Envm_OBJ(iObj).EBAPresel.eEbaHypCat
#define OPS_EBA_OBJ_MAX_SAFETY (255u)
#define Envm_INT_OBJ_GENERAL(iObj) Envm_INT_GET_Envm_OBJ(iObj).General
#define EM_INT_OBJ_IS_DELETED(iObj)                                           \
    ((Envm_INT_OBJ_GENERAL(iObj).eObjMaintenanceState == MT_STATE_DELETED) || \
     (Envm_INT_OBJ_GENERAL(iObj).eObjMaintenanceState ==                      \
      MT_STATE_MERGE_DELETED))
#define OPS_EBA_OBJ_SAFETY_OBS_BIT (32u)
#define OPS_EBA_OBJ_SAFETY_RCS_BIT (64u)
#define OPS_EBA_OBJ_SAFETY_POE_BIT (128u)
#define OPS_EBA_OBJ_SAFETY_LFT_BIT (16u)
#define Percentage_max (100U)

#ifndef FPS_EBA_INH_NONE
#define FPS_EBA_INH_NONE 0U
#endif
#ifndef FPS_EBA_INH_LAT_WARN
#define FPS_EBA_INH_LAT_WARN 1U
#endif
#ifndef FPS_EBA_INH_PRE_WARN
#define FPS_EBA_INH_PRE_WARN 2U
#endif
#ifndef FPS_EBA_INH_ACU_WARN
#define FPS_EBA_INH_ACU_WARN 4U
#endif
#ifndef FPS_EBA_INH_PRE_FILL
#define FPS_EBA_INH_PRE_FILL 8U
#endif
#ifndef FPS_EBA_INH_HBA_THRD
#define FPS_EBA_INH_HBA_THRD 16U
#endif
#ifndef FPS_EBA_INH_HBA_TBRK
#define FPS_EBA_INH_HBA_TBRK 32U
#endif
#ifndef FPS_EBA_INH_PRECRASH
#define FPS_EBA_INH_PRECRASH 64U
#endif
#ifndef FPS_EBA_INH_BRAKE_L1
#define FPS_EBA_INH_BRAKE_L1 128U
#endif
#ifndef FPS_EBA_INH_BRAKE_L2
#define FPS_EBA_INH_BRAKE_L2 256U
#endif
#ifndef FPS_EBA_INH_BRAKE_L3
#define FPS_EBA_INH_BRAKE_L3 512U
#endif
#ifndef FPS_EBA_INH_ALL
#define FPS_EBA_INH_ALL 65535U
#endif

#ifndef GDB_OBJECT_SUBPROP_NORMAL
#define GDB_OBJECT_SUBPROP_NORMAL (0U)
#endif
#ifndef GDB_OBJECT_SUBPROP_CROSSING
#define GDB_OBJECT_SUBPROP_CROSSING (1U)
#endif
#define Envm_INT_OBJ_LEGACY(iObj) Envm_INT_GET_Envm_OBJ(iObj).Legacy

#define Envm_INT_OBJ_RCS_TGT_TRESHOLD_UNCOMP(iObj) \
    Envm_INT_OBJ_LEGACY(iObj).fRCSTargetThresholdUncomp
#define RSP_NUM_OF_RCS_TABLE_VAL 41
#define RCS_THR_LOW_VELOCITY (10.f)
#define RCS_THR_MID_VELOCITY (21.f)

#define RCS_THR_TARGET_TRACED_NEAR_ARRAY                                      \
    (-29.0f), (-29.0f), (-29.0f), (-29.0f), (-27.0f), (-27.0f), (-27.0f),     \
        (-27.0f), (-26.8f), (-26.4f), (-26.0f), (-25.8f), (-25.6f), (-25.4f), \
        (-25.2f), (-25.0f), (-24.8f), (-24.6f), (-24.4f), (-24.2f), (-24.0f), \
        (-23.8f), (-23.6f), (-23.4f), (-23.2f), (-23.0f), (-22.8f), (-22.6f), \
        (-22.4f), (-22.2f), (-22.0f), (-21.8f), (-21.6f), (-21.4f), (-21.2f), \
        (-21.0f), (-20.8f), (-20.6f), (-20.4f), (-20.2f), (-20.0f)

#define RCS_THR_TARGET_TRACED_NEAR_MID_VEL_ARRAY                              \
    (-29.0f), (-29.0f), (-29.0f), (-29.0f), (-27.0f), (-27.0f), (-27.0f),     \
        (-27.0f), (-26.8f), (-26.4f), (-26.0f), (-25.8f), (-25.6f), (-25.4f), \
        (-25.2f), (-25.0f), (-24.8f), (-24.6f), (-24.4f), (-24.2f), (-24.0f), \
        (-23.8f), (-23.6f), (-23.4f), (-23.2f), (-23.0f), (-22.8f), (-22.6f), \
        (-22.4f), (-22.2f), (-22.0f), (-21.8f), (-21.6f), (-21.4f), (-21.2f), \
        (-21.0f), (-20.8f), (-20.6f), (-20.4f), (-20.2f), (-20.0f)

#define RCS_THR_TARGET_TRACED_NEAR_LOW_VEL_ARRAY                              \
    (-30.6f), (-30.6f), (-30.6f), (-28.4f), (-27.0f), (-27.0f), (-27.0f),     \
        (-27.0f), (-26.8f), (-26.4f), (-26.0f), (-25.8f), (-25.6f), (-25.4f), \
        (-25.2f), (-25.0f), (-24.8f), (-24.6f), (-24.4f), (-24.2f), (-24.0f), \
        (-23.8f), (-23.6f), (-23.4f), (-23.2f), (-23.0f), (-22.8f), (-22.6f), \
        (-22.4f), (-22.2f), (-22.0f), (-21.8f), (-21.6f), (-21.4f), (-21.2f), \
        (-21.0f), (-20.8f), (-20.6f), (-20.4f), (-20.2f), (-20.0f)

#define ODDELAY_f_curveTgtLatency (0.0f)
#define ODDELAY_f_curveObjLatency (20.0f)
#define ODDELAY_f_drvIntCurveTgtLatency (0.0f)
#define ODDELAY_f_drvIntCurveObjLatency (0.0f)
#define ODDELAY_f_speedXTgtLatency (0.0f)
#define ODDELAY_f_speedXObjLatency (0.0f)
#define ODDELAY_f_accelXTgtLatency (0.0f)
#define ODDELAY_f_accelXObjLatency (60.0f)

ALGO_INLINE uint32 EM_f_GetRSPTimeStamp(const uint8 u_ScanMode) {
    uint32 u_Res = 0;

    u_Res = EnvmData.pPrivObjList->sSigHeader.uiTimeStamp;

    return u_Res;
}
#ifndef CR_OBJECT_PROPERTY_MOVING
#define CR_OBJECT_PROPERTY_MOVING 0U
#endif
#ifndef CR_OBJECT_PROPERTY_STATIONARY
#define CR_OBJECT_PROPERTY_STATIONARY 1U
#endif
#ifndef CR_OBJECT_PROPERTY_ONCOMING
#define CR_OBJECT_PROPERTY_ONCOMING 2U
#endif

#ifndef CR_OBJECT_SUBPROP_UNIFAL
#define CR_OBJECT_SUBPROP_UNIFAL 0U
#endif
#ifndef CR_OBJECT_SUBPROP_CROSSING
#define CR_OBJECT_SUBPROP_CROSSING 1U
#endif

#ifndef CR_OBJECT_MOVSTATE_STATIONARY
#define CR_OBJECT_MOVSTATE_STATIONARY 0U
#endif
#ifndef CR_OBJECT_MOVSTATE_STOPPED
#define CR_OBJECT_MOVSTATE_STOPPED 1U
#endif
#ifndef CR_OBJECT_MOVSTATE_MOVING
#define CR_OBJECT_MOVSTATE_MOVING 2U
#endif

#ifndef CR_MEAS_SEN_NONE
#define CR_MEAS_SEN_NONE 0U
#endif
#ifndef CR_MEAS_SEN_FCRCAN
#define CR_MEAS_SEN_FCRCAN 1U
#endif
#ifndef CR_MEAS_SEN_NECRCAN
#define CR_MEAS_SEN_NECRCAN 2U
#endif
#ifndef CR_MEAS_SEN_GRID
#define CR_MEAS_SEN_GRID 4U
#endif
#ifndef CR_MEAS_SEN_CAM
#define CR_MEAS_SEN_CAM 8U
#endif
#ifndef CR_MEAS_SEN_1
#define CR_MEAS_SEN_1 16U
#endif
#ifndef CR_MEAS_SEN_2
#define CR_MEAS_SEN_2 32U
#endif
#ifndef CR_MEAS_SEN_3
#define CR_MEAS_SEN_3 64U
#endif
#ifndef CR_MEAS_SEN_4
#define CR_MEAS_SEN_4 128U
#endif

#ifndef CR_LONGVEHICLE_TYPE_UNIFAL
#define CR_LONGVEHICLE_TYPE_UNIFAL 0U
#endif
#ifndef CR_LONGVEHICLE_TYPE_REAL
#define CR_LONGVEHICLE_TYPE_REAL 1U
#endif
#ifndef CR_LONGVEHICLE_TYPE_MIRROR
#define CR_LONGVEHICLE_TYPE_MIRROR 2U
#endif
#ifndef CR_LONGVEHICLE_TYPE_SHADOW
#define CR_LONGVEHICLE_TYPE_SHADOW 3U
#endif
#ifndef CR_LONGVEHICLE_TYPE_MIDDLE
#define CR_LONGVEHICLE_TYPE_MIDDLE 4U
#endif

#ifndef CR_OBJCLASS_POINT
#define CR_OBJCLASS_POINT 0U
#endif
#ifndef CR_OBJCLASS_CAR
#define CR_OBJCLASS_CAR 1U
#endif
#ifndef CR_OBJCLASS_TRUCK
#define CR_OBJCLASS_TRUCK 2U
#endif
#ifndef CR_OBJCLASS_PEDESTRIAN
#define CR_OBJCLASS_PEDESTRIAN 3U
#endif
#ifndef CR_OBJCLASS_MOTORCYCLE
#define CR_OBJCLASS_MOTORCYCLE 4U
#endif
#ifndef CR_OBJCLASS_BICYCLE
#define CR_OBJCLASS_BICYCLE 5U
#endif
#ifndef CR_OBJCLASS_WIDE
#define CR_OBJCLASS_WIDE 6U
#endif
#ifndef CR_OBJCLASS_UNCLASSIFIED
#define CR_OBJCLASS_UNCLASSIFIED 7U
#endif
extern float32 f_GlobalLongPosToRot;
extern float32 f_GlobalCluSyncCosFloatAng;
extern float32 f_GlobalObjSyncCosFloatAng;
ALGO_INLINE float32 Envm_f_GetEgoObjSyncCosFloatAng(void) {
    return f_GlobalObjSyncCosFloatAng;
}
ALGO_INLINE float32 Envm_f_GetSensorLatOffset(void) {
    return EnvmData.pGlobEgoStatic->SensorMounting.LatPos;
}
ALGO_INLINE float32 EM_f_GetEgoObjSyncYawRate(void) {
    return EnvmData.pEgoDynObjSync->Lateral.YawRate.YawRate;
}
ALGO_INLINE float32
EMTRAFO_f_GetObjSyncEgoMotionYawrateFactor(float32 f_DistY) {
    return (EM_f_GetEgoObjSyncYawRate() *
            (f_DistY - Envm_f_GetSensorLatOffset()));
}
ALGO_INLINE float32 EM_f_GetEgoObjSyncVelX(void) {
    return EnvmData.pEgoDynObjSync->Longitudinal.MotVar.Velocity;
}
ALGO_INLINE float32 EMTRAFO_f_GetObjSyncEgoMotionVx(float32 f_DistY) {
    return ((EM_f_GetEgoObjSyncVelX() * Envm_f_GetEgoObjSyncCosFloatAng()) -
            EMTRAFO_f_GetObjSyncEgoMotionYawrateFactor(f_DistY));
}
ALGO_INLINE float32 EM_f_GetEgoObjSyncVelXVar(void) {
    return EnvmData.pEgoDynObjSync->Longitudinal.MotVar.varVelocity;
}
ALGO_INLINE float32 Envm_f_GetSensorLongPosToRot(void) {
    return f_GlobalLongPosToRot;
}
ALGO_INLINE float32 EMTRAFO_f_GetObjSyncEgoMotionVy(float32 f_DistX) {
    return (EM_f_GetEgoObjSyncYawRate() *
            (f_DistX + Envm_f_GetSensorLongPosToRot()));
}
ALGO_INLINE float32 EM_f_GetEgoObjSyncAccelX(void) {
    return EnvmData.pEgoDynObjSync->Longitudinal.MotVar.Accel;
}
ALGO_INLINE float32 EM_f_GetEgoObjSyncAccelXVar(void) {
    return EnvmData.pEgoDynObjSync->Longitudinal.MotVar.varAccel;
}
ALGO_INLINE float32 EMSIZE_f_GetDeltaOrienation2RefPoint(float32 fOrientation,
                                                         uint8 eRefPointPos) {
    return (fOrientation - ((float32)eRefPointPos * (3.14159265359f * 0.5f)));
}
#define GET_FCT_OBJ_ID(iObj) a_EnvmObjIDToFCTObjIDConvert[iObj]

ALGO_INLINE uint32 BML_u_Round2Uint(float32 x) { return (uint32)(x + 0.5f); }
#define ROUND_TO_UINT(x) BML_u_Round2Uint(x)
#define EM_INT_OBJ_MAINTENANCE_STATE(iObj) \
    (Envm_INT_OBJ_GENERAL(iObj).eObjMaintenanceState)

#ifndef VED_IO_STATE_VALID
#define VED_IO_STATE_VALID 0U
#endif
#ifndef VED_IO_STATE_INVALID
#define VED_IO_STATE_INVALID 1U
#endif
#ifndef VED_IO_STATE_NOTAVAIL
#define VED_IO_STATE_NOTAVAIL 2U
#endif
#ifndef VED_IO_STATE_DECREASED
#define VED_IO_STATE_DECREASED 3U
#endif
#ifndef VED_IO_STATE_SUBSTITUE
#define VED_IO_STATE_SUBSTITUE 4U
#endif
#ifndef VED_IO_STATE_INPLAUSIBLE
#define VED_IO_STATE_INPLAUSIBLE 5U
#endif
#ifndef VED_IO_STATE_INIT
#define VED_IO_STATE_INIT 15U
#endif
#ifndef VED_IO_STATE_MAX
#define VED_IO_STATE_MAX 255U
#endif

#define IS_SIGNAL_STATUS_OK(status) (status == VED_IO_STATE_VALID)

#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) (void)(x)
#endif
#define Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(iObj) \
    (Envm_INT_OBJ_ATTRIBUTES(iObj).uiStoppedConfidence > 80U)
#define Envm_INT_OBJ_QUALIFIERS(iObj) Envm_INT_GET_Envm_OBJ(iObj).Qualifiers
#define EM_INT_OBJ_OBSTACLE_PROBABILITY(iObj) \
    Envm_INT_OBJ_QUALIFIERS(iObj).ucObstacleProbability
#define GET_FCT_OBJ_PUB(iObj) GET_FCT_PUB_OBJ_DATA_PTR->ObjList[iObj]
#define OBJ_FCT_GET_OOI_POS(iObj) \
    ((GET_FCT_OBJ_ID(iObj) < 0)   \
         ? (OBJ_NOT_OOI)          \
         : (GET_FCT_OBJ_PUB(GET_FCT_OBJ_ID(iObj)).ObjOfInterest.eObjOOI))
#define OBJ_FCT_GET_RELEVANT(iObj)        \
    ((GET_FCT_OBJ_ID(iObj) < 0) ? (FALSE) \
                                : (OBJ_FCT_GET_OOI_POS(iObj) == OBJ_NEXT_OOI))
ALGO_INLINE float32 EM_f_GetCycleTime(void) {
    return EnvmData.pECAMtCyclEnvmode->fECAMtCycleTime;
}
ALGO_INLINE boolean Envm_b_IsTunnelDetected(void) {
    return EnvmData.pPrivObjList->sSceneDescription.uiTunnelProb >= 60u;
}
#define Envm_INT_OBJ_KINEnvmATIC(iObj) Envm_INT_GET_Envm_OBJ(iObj).Kinematic

#define EM_INT_OBJ_LONG_DISPLACEMENT(iObj) Envm_INT_OBJ_KINEnvmATIC(iObj).fDistX
#define Envm_INT_OBJ_LAT_DISPLACEnvmENT(iObj) \
    Envm_INT_OBJ_KINEnvmATIC(iObj).fDistY
#define Envm_INT_OBJ_CLASSIFICATION(iObj) \
    Envm_INT_OBJ_ATTRIBUTES(iObj).eClassification
#define EM_INT_OBJ_LIFETIME(iObj) Envm_INT_OBJ_LEGACY(iObj).uiLifeTime
#define Envm_INT_OBJ_GEOMETRY(iObj) Envm_INT_GET_Envm_OBJ(iObj).Geometry
#define Envm_INT_OBJ_ORIENTATION_VALID(iObj) \
    Envm_INT_OBJ_GEOMETRY(iObj).fOrientationValid
#define Envm_INT_OBJ_LAT_VREL(iObj) Envm_INT_OBJ_KINEnvmATIC(iObj).fVrelY
#define Envm_INT_OBJ_PROBABILITY_OF_EXIST(iObj) \
    Envm_INT_OBJ_QUALIFIERS(iObj).fProbabilityOfExistence
ALGO_INLINE uint8
Envm_u_GetTgtConfirmDensitySingleScan(sint8 s_Obj, const uint8 u_ScanMode) {
    uint8 u_TgtDensity = 0u;

    if (SYS_SCAN_NEAR == u_ScanMode) {
        u_TgtDensity = EnvmData.pPrivObjList->Objects[s_Obj]
                           .Qualifiers.uMeasuredTargetFrequencyNear;
    } else {
        u_TgtDensity = EnvmData.pPrivObjList->Objects[s_Obj]
                           .Qualifiers.uMeasuredTargetFrequencyFar;
    }

    return u_TgtDensity;
}
ALGO_INLINE uint8 Envm_u_GetTgtConfirmDensity(sint8 s_Obj) {
    return (Envm_u_GetTgtConfirmDensitySingleScan(s_Obj, SYS_SCAN_NEAR) |
            Envm_u_GetTgtConfirmDensitySingleScan(s_Obj, SYS_SCAN_FAR));
}
#define Envm_INT_OBJ_RCS(iObj) Envm_INT_GET_Envm_OBJ(iObj).SensorSpecific.fRCS
ALGO_INLINE float32 Envm_f_GetRCSThresholdReduction(uint8 uScanMode) {
    return EnvmData.pPrivGlob->f_RCSThresholdReduction[uScanMode];
}
#define EM_INT_OBJ_LONG_VREL(iObj) Envm_INT_OBJ_KINEnvmATIC(iObj).fVrelX
ALGO_INLINE float32 EM_f_GetEgoObjSyncCurve(void) {
    return EnvmData.pEgoDynObjSync->Lateral.Curve.Curve;
}
#define CML_GetBit(source, bitmask) (((source) & (bitmask)) == (bitmask))
#define BML_SetBit(source, bitmask) ((source) |= (bitmask))
#define SET_BIT(source, bitmask) BML_SetBit(source, bitmask)
#define Envm_INT_OBJ_LAT_DISPLACEnvmENT_VAR(iObj) \
    ((Envm_INT_OBJ_KINEnvmATIC(iObj).fDistYStd) * \
     (Envm_INT_OBJ_KINEnvmATIC(iObj).fDistYStd))
#define EM_INT_OBJ_MOVING_TO_STATIONARY(iObj) \
    Envm_INT_OBJ_ATTRIBUTES(iObj).uiStoppedConfidence
ALGO_INLINE float32 EM_f_GetEgoCluSyncVelX(void) {
    return EnvmData.pEgoDynTgtSync->Longitudinal.MotVar.Velocity;
}

ALGO_INLINE void EM_v_ShiftCurrentTgtConfirmDensity(sint8 s_Obj) {
    EnvmData.pPrivObjList->Objects[s_Obj]
        .Qualifiers.uMeasuredTargetFrequencyNear >>= 1u;
    EnvmData.pPrivObjList->Objects[s_Obj]
        .Qualifiers.uMeasuredTargetFrequencyFar >>= 1u;
}
ALGO_INLINE void Envm_v_SetTgtConfirmDensitySingleScan(sint8 s_Obj,
                                                       uint8 u_DensityValue,
                                                       const uint8 u_ScanMode) {
    if (SYS_SCAN_NEAR == u_ScanMode) {
        EnvmData.pPrivObjList->Objects[s_Obj]
            .Qualifiers.uMeasuredTargetFrequencyNear = u_DensityValue;
    } else {
        EnvmData.pPrivObjList->Objects[s_Obj]
            .Qualifiers.uMeasuredTargetFrequencyFar = u_DensityValue;
    }
}

#ifndef TRAFFICSIGN_SPEED_5
#define TRAFFICSIGN_SPEED_5 18U
#endif
#ifndef TRAFFICSIGN_SPEED_10
#define TRAFFICSIGN_SPEED_10 19U
#endif
// #ifndef TRAFFICSIGN_SPEED_15
// #define TRAFFICSIGN_SPEED_15 20U
// #endif
#ifndef TRAFFICSIGN_SPEED_20
#define TRAFFICSIGN_SPEED_20 20U
#endif
#ifndef TRAFFICSIGN_SPEED_30
#define TRAFFICSIGN_SPEED_30 21U
#endif
// #ifndef TRAFFICSIGN_SPEED_35
// #define TRAFFICSIGN_SPEED_35 162U
// #endif
#ifndef TRAFFICSIGN_SPEED_40
#define TRAFFICSIGN_SPEED_40 22U
#endif
#ifndef TRAFFICSIGN_SPEED_50
#define TRAFFICSIGN_SPEED_50 23U
#endif
#ifndef TRAFFICSIGN_SPEED_60
#define TRAFFICSIGN_SPEED_60 24U
#endif
#ifndef TRAFFICSIGN_SPEED_70
#define TRAFFICSIGN_SPEED_70 25U
#endif
#ifndef TRAFFICSIGN_SPEED_80
#define TRAFFICSIGN_SPEED_80 26U
#endif
#ifndef TRAFFICSIGN_SPEED_90
#define TRAFFICSIGN_SPEED_90 27U
#endif
#ifndef TRAFFICSIGN_SPEED_100
#define TRAFFICSIGN_SPEED_100 28U
#endif
#ifndef TRAFFICSIGN_SPEED_110
#define TRAFFICSIGN_SPEED_110 29U
#endif
#ifndef TRAFFICSIGN_SPEED_120
#define TRAFFICSIGN_SPEED_120 30U
#endif
#ifndef TRAFFICSIGN_LIFT_5
#define TRAFFICSIGN_LIFT_5 31U
#endif
#ifndef TRAFFICSIGN_LIFT_10
#define TRAFFICSIGN_LIFT_10 32U
#endif
#ifndef TRAFFICSIGN_LIFT_20
#define TRAFFICSIGN_LIFT_20 33U
#endif
#ifndef TRAFFICSIGN_LIFT_30
#define TRAFFICSIGN_LIFT_30 34U
#endif
// #ifndef TRAFFICSIGN_LIFT_35
// #define TRAFFICSIGN_LIFT_35 176U
// #endif
#ifndef TRAFFICSIGN_LIFT_40
#define TRAFFICSIGN_LIFT_40 35U
#endif
#ifndef TRAFFICSIGN_LIFT_50
#define TRAFFICSIGN_LIFT_50 36U
#endif
#ifndef TRAFFICSIGN_LIFT_60
#define TRAFFICSIGN_LIFT_60 37U
#endif
#ifndef TRAFFICSIGN_LIFT_70
#define TRAFFICSIGN_LIFT_70 38U
#endif
#ifndef TRAFFICSIGN_LIFT_80
#define TRAFFICSIGN_LIFT_80 39U
#endif
#ifndef TRAFFICSIGN_LIFT_90
#define TRAFFICSIGN_LIFT_90 40U
#endif
#ifndef TRAFFICSIGN_LIFT_100
#define TRAFFICSIGN_LIFT_100 41U
#endif
#ifndef TRAFFICSIGN_LIFT_110
#define TRAFFICSIGN_LIFT_110 42U
#endif
#ifndef TRAFFICSIGN_LIFT_120
#define TRAFFICSIGN_LIFT_120 43U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_50
#define TRAFFICSIGN_SPEEDLIMIT_50 44U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_60
#define TRAFFICSIGN_SPEEDLIMIT_60 45U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_70
#define TRAFFICSIGN_SPEEDLIMIT_70 46U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_80
#define TRAFFICSIGN_SPEEDLIMIT_80 47U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_90
#define TRAFFICSIGN_SPEEDLIMIT_90 48U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_100
#define TRAFFICSIGN_SPEEDLIMIT_100 49U
#endif
#ifndef TRAFFICSIGN_SPEEDLIMIT_110
#define TRAFFICSIGN_SPEEDLIMIT_110 50U
#endif
#ifndef TRAFFICSIGN_X_LEFT
#define TRAFFICSIGN_X_LEFT 72U
#endif
#ifndef TRAFFICSIGN_X_RIGHT
#define TRAFFICSIGN_X_RIGHT 73U
#endif
#ifndef TRAFFICSIGN_X_STRAIGHT
#define TRAFFICSIGN_X_STRAIGHT 106U
#endif
#ifndef TRAFFICSIGN_X_TURNING
#define TRAFFICSIGN_X_TURNING 100U
#endif
#ifndef TRAFFICSIGN_X_PARKING
#define TRAFFICSIGN_X_PARKING 101U
#endif
#ifndef TRAFFICSIGN_X_ENTER
#define TRAFFICSIGN_X_ENTER 102U
#endif
#ifndef TRAFFICSIGN_X_TURNNINGBACK
#define TRAFFICSIGN_X_TURNNINGBACK 74U
#endif
#ifndef TRAFFICSIGN_X_PASSING
#define TRAFFICSIGN_X_PASSING 105U
#endif
#ifndef TRAFFICSIGN_X_AUDIBLEWARNING
#define TRAFFICSIGN_X_AUDIBLEWARNING 75U
#endif
// #ifndef TRAFFICSIGN_X_LANDR
// #define TRAFFICSIGN_X_LANDR 185U
// #endif
// #ifndef TRAFFICSIGN_X_ROUND
// #define TRAFFICSIGN_X_ROUND 186U
// #endif
// #ifndef TRAFFICSIGN_X_WHISTLE
// #define TRAFFICSIGN_X_WHISTLE 278U
// #endif

#ifndef VARIABLE_TRFCSGN_SPEED_10
#define VARIABLE_TRFCSGN_SPEED_10 53U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_20
#define VARIABLE_TRFCSGN_SPEED_20 54U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_30
#define VARIABLE_TRFCSGN_SPEED_30 55U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_40
#define VARIABLE_TRFCSGN_SPEED_40 56U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_50
#define VARIABLE_TRFCSGN_SPEED_50 57U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_60
#define VARIABLE_TRFCSGN_SPEED_60 58U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_70
#define VARIABLE_TRFCSGN_SPEED_70 59U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_80
#define VARIABLE_TRFCSGN_SPEED_80 60U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_90
#define VARIABLE_TRFCSGN_SPEED_90 61U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_100
#define VARIABLE_TRFCSGN_SPEED_100 62U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_110
#define VARIABLE_TRFCSGN_SPEED_110 63U
#endif
#ifndef VARIABLE_TRFCSGN_SPEED_120
#define VARIABLE_TRFCSGN_SPEED_120 64U
#endif

#ifdef __cplusplus
}
#endif
#endif
