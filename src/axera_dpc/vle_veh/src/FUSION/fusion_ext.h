/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * zhang guanglin <zhang guanglin@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_FUSION_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_FUSION_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "Platform_Types.h"
// #ifndef boolean
// typedef unsigned char boolean;
// #endif
// #ifndef sint8
// typedef signed char sint8;
// #endif
// #ifndef uint8
// typedef unsigned char uint8;
// #endif
// #ifndef sint16
// typedef signed short sint16;
// #endif
// #ifndef uint16
// typedef unsigned short uint16;
// #endif
// #ifndef sint32
// typedef signed long sint32;
// #endif
// #ifndef uint32
// typedef unsigned long uint32;
// #endif
// #ifndef sint64
// typedef signed long long sint64;
// #endif
// #ifndef uint64
// typedef unsigned long long uint64;
// #endif
// #ifndef  float32
// typedef float float32;
// #endif
// #ifndef  float64
// typedef double float64;
// #endif

#ifndef false
#define false (0)
#endif

#ifndef true
#define true (!false)
#endif

#ifndef FALSE
#define FALSE false
#endif

#ifndef TRUE
#define TRUE true
#endif

// typedef uint32 AlgoInterfaceVersionNumber_t;

#define SRR_RAM_OBJ_NUM 40
#define FUSION_NR_PRIVOBJECTS (40)
#define TUE_RADAR_RAW_OBJECT_NUM 128  // radar object number get from sensor
#define TUE_FRONT_RADAR_RAW_OBJECT_NUM 128
#define TUE_FRONT_LEFT_RADAR_RAW_OBJECT_NUM 12
#define TUE_FRONT_RIGHT_RADAR_RAW_OBJECT_NUM 12
#define TUE_REAR_RIGHT_RADAR_RAW_OBJECT_NUM 16
#define TUE_REAR_LEFT_RADAR_RAW_OBJECT_NUM 16
#define TUE_FUS_OUTPUT_OBJ_NUMBER 100
#define EM_SINGAL_SRR_GEN_OBJECT_NUM \
    SRR_RAM_OBJ_NUM  // number of object output for one single SRR object
#define MULTI_SENSOR_RADAR_FUSION_FrontRadar (0U)
#define MULTI_SENSOR_RADAR_FUSION_FrontLeftRadar (1U)
#define MULTI_SENSOR_RADAR_FUSION_FrontRightRadar (2U)
#define MULTI_SENSOR_RADAR_FUSION_RearLeftRadar (3U)
#define MULTI_SENSOR_RADAR_FUSION_RearRightRadar (4U)
#define MULTI_SENSOR_RADAR_FUSION_QUANTITY (5u)
#define CAMERA_OBJ_INPUT_NUM (20u)

#define TU_OBJECT_DYNPROP_MOVING (0)
#define TU_OBJECT_DYNPROP_STATIONARY (1)
#define TU_OBJECT_DYNPROP_ONCOMING (2)
#define TU_OBJECT_DYNPROP_STATIONARY_CANDITATE (3)
#define TU_OBJECT_DYNPROP_UNKNOWN (4)
#define TU_OBJECT_DYNPROP_CROSSING_STATIONARY (5)
#define TU_OBJECT_DYNPROP_CROSSING_MOVING (6)
#define TU_OBJECT_DYNPROP_STOPPED (7)

#ifndef MT_STATE_MEASURED
#define MT_STATE_MEASURED (2U)
#endif

#define TASK_CYCLE_TIME_50 0.050F

#define POSX (0u)
#define POSY (1u)
#define VELX (2u)
#define VELY (3u)
#define ACCX (4u)
#define ACCY (5u)

#define VARIANCE_POSX (0u)
#define VARIANCE_POSY (2u)
#define VARIANCE_VELX (5u)
#define VARIANCE_VELY (9u)
#define VARIANCE_ACCX (14u)
#define VARIANCE_ACCY (20u)

typedef struct {
    float32 fDistX;
    float32 fDistXStd;
    float32 fDistY;
    float32 fDistYStd;
    float32 fVx;
    float32 fVxStd; 
    float32 fVy;
    float32 fVyStd; 
    float32 fAx; 
    float32 fAxStd;
    float32 fAy;
    float32 fAyStd;
    float32 fOrientation; 
    uint8 eRefpointLocation;
} CamObjKinematics;

typedef struct {
    float32 fHeight;
    float32 fOffsetToGround;
    float32 fLength;
    float32 fWidth;
} CamObjGeometry;

typedef struct {
    uint8
        probabilityOfClassification;
    uint8 ObjectType;
    uint8 CIPVFlag;  // key car flag
    uint8 MCPFlag;   // key ped flag
} CamObjClassification;

typedef struct {
    uint8 eStatusBrakeLight;
    uint8
        uiBrakeLightConfidence;
    uint8 eStatusTurnIndicator;
    uint8
        uiTurnIndicatorConfidence;
    uint8 eAssociatedLane;
    uint8 uiAssociatedLaneConfidence;
    uint8 eLaneChangeManeuver;
    uint8 percentageOwnDrivingLane;
    uint8 percentageSideDrivingLaneLeft;
    uint8 percentageSideDrivingLaneRight;
    uint8 eMainteanceState;
    uint8 percentageProbOfExist;
    uint8 eMotionState;
    uint8 eObjectOcclusion;
    uint32 ObjectLifeCycles;
} CamObjAttributes;

typedef struct {
    uint8 ObjectID;
    boolean bCamObjValid;
    CamObjKinematics Kinematic;
    CamObjGeometry Geometry;
    CamObjClassification Classification;
    CamObjAttributes Attributes;
} CamObject;

typedef CamObject CamObjectArray[CAMERA_OBJ_INPUT_NUM];

typedef struct {
    uint32  uiTimeStamp;
    uint8   eSigStatus;
    CamObjectArray aObject;
} CamObjectList;

typedef struct {
    float32 fDistX;
    float32 fDistXStd;
    float32 fDistY;
    float32 fDistYStd;
    float32 fVrelX;
    float32 fVrelXStd;
    float32 fVrelY;
    float32 fVrelYStd;
    float32 fArelX;
    float32 fArelXStd;
    float32 fArelY;
    float32 fArelYStd;
} fus_Kinematic_t;

typedef struct {
    float32 fWidth;
    float32 fLength;
    float32 fOrientation;
    float32 fOrientationStd;
    float32 fOrientationValid;
} fus_Geometry_t;

typedef struct {
    uint8 eDynamicProperty;
    uint8 uiStoppedConfidence;
    uint8 eAbsMovingState;
    uint8 eClassification;
    uint8 uiClassConfidence;
} fus_Attributes_t;

typedef struct {
    float32 fLifeTime;
    uint32 fTimeStamp;
    uint8 eObjMaintenanceState;
    uint8 cObjMergingID;
} fus_General_t;

typedef struct {
    float32 fProbabilityOfExistence;
    uint8 ucObstacleProbability;  // add Obstacle Probability for ars410 radar
    uint8 uMeasuredTargetFrequencyNear;
    uint8 uMeasuredTargetFrequencyFar;
} fus_Qualifiers_t;

typedef struct {
    float32 fRCS;
    boolean bCamConfirmed;  // true: object has been observed by camera in last
                            // 100 cycle
} fus_SensorSpecific_t;

typedef struct {
    uint16 eEbaInhibitionMask;
    uint8 ucEbaMovingObjQuality;
    uint8 eEbaHypCat;
    boolean bCrossingPedEbaPresel;
} fus_EBAPresel_t;

typedef struct {
    float32 fMaxAccelY;
    float32 fRCSTargetThresholdUncomp;
    uint16 uiLifeTime;
    uint8 eDynamicSubProperty;
} fus_LegacyObj_t;

typedef struct {
    sint32 ObjectId;  // changed by 20210915    //uint8
    fus_Kinematic_t Kinematic;
    fus_Geometry_t Geometry;
    fus_Attributes_t Attributes;
    fus_General_t General;
    fus_Qualifiers_t Qualifiers;
    fus_SensorSpecific_t SensorSpecific;
    fus_EBAPresel_t EBAPresel;
    fus_LegacyObj_t Legacy;
} FusObjects_t;

typedef FusObjects_t FusObjects_array_t[TUE_RADAR_RAW_OBJECT_NUM];

typedef struct {
    uint32  uiTimeStamp;
    uint16  uiCycleCounter;
    uint8   eSigStatus;
    uint16  uNumOfUsedObjects;
    FusObjects_array_t Objects;
} FusObjectList_t;

typedef struct {
    boolean bIsValid; /**< valid flag to check whether valid data is read on RTE */
    uint8   u8Reserved;   /**< reserved value to ensure 4 byte alignment */
    uint16  u16Reserved; /**< reserved value to ensure 4 byte alignment */
    float32 f32Speed;   /**< Speed of ego vehicle [m/s] */
    float32 f32Acc;     /**< Acceleration of ego vehicle [m/s^2] */
    float32 f32YawRate; /**< Yaw rate of ego vehicle [rad/s] */
    float32 f32Age;     /**< age of ego motion, including latency [s]*/
} VedInfo_t;

typedef FusObjectList_t
    MultiRadar_ObjectList_t[MULTI_SENSOR_RADAR_FUSION_QUANTITY];

typedef struct {
    MultiRadar_ObjectList_t Radarobjlists;
} Fusion_Radar_input_lists;

typedef struct {
    // uint32 index;
    VedInfo_t vedInput;
    CamObjectList camObject;
    Fusion_Radar_input_lists radarList;
} reqFusionPrtList_t;

/**
 * Object list for external Fusion interface
 */
typedef struct {
    float32 data[6]; /**< data */
    uint16 nRows;    /**< number of rows */
    uint16 u16Reserved;
} Vec_t;

typedef struct {
    float32 data[21];
    uint16 u16Size;
    uint16 u16Reserved;
} SymMatrix_t;

typedef struct {
    /* Object Information */
    boolean bUpdated;         /** Updated in current cycle */
    uint8 u8CoordSystem;      // Indicates in which coordinate system the ego
                                //  position lives
    uint8 u8TrkblBinPosition; // Indicate the objects bin position for object
                                //  selection
    uint8 u8VisionIdx; /** vision idx for vision ID mapping in output converter */

    uint16 u16ID;  // u16ID is an identifier number that allows for tracking of
                    //   persistent objects over time. 0 is a valid ID
    uint16 u16Age; // Time [ms] that has passed since initialization of
                    //   underlying track: 0...65000 is 0...65sec; 65001 is more
                    //   than 65sec(saturation value)
    uint16 u16Lifespan; // Lifespan of trackable to handle coasting and to see
                        //    if trackable is valid or not
    uint16 u16RefPoint; // Indicates which point of the target object is given
                        //    by the coordinates
    uint8 u8CyclesNoVision; // number of consequitve vision updates without
                            //    confirmation of this object -> reset to 0 if
                            //    object is updated
    uint8 u8CyclesNoRadar;  // number of consequitve radar updates without
                            //    confirmation of this object -> reset to 0 if
                            //    object is updated

    uint16 au16SensorID[6]; // u16ID of the sensor input object. 0 is a valid
                            //    ID. The order must always map to the sensor
                            //    pattern array in trackabListUtils
    uint16 au16SensorIDLast[6]; // u16ID of the sensor nput object from the
                                //    last update. 0 is a valid ID. The order must
                                //    always map to the sensor pattern array in
                                //    trackabListUtils

    uint32 u32SensorsCurr; // Bit pattern that indicates which sensors have
                            //   seen this object during their last measurements.
                            //   Each of the first 31 bits has a specific meaning
                            //   (see defines)
    uint32
        u32SensorsHist; // Bit pattern that indicates which sensors have seen
                        //    this object at some point in time. Each of the 31 LSB
                        //    Bits has a specific meaning (see defines)

    /* Kinematics */
    Vec_t vecX; // state vector from Kalman filter including position,
                //    velocity, acceleration

    SymMatrix_t matP; // P matrix from Kalman filter including variances and
                        //  covariances of all states

    float32 f32Gain;    /** camera gain for this object */
    float32 f32GainVar; /** Variance of camera gain for this object */

    /* Coordinated Turn Model */
    float32 f32YawRate;             /** Yaw rate of target vehicle */
    float32 f32YawRateVar;          /** Variance of yaw rate for this object */
    float32 f32Heading;             /** Heading for this object */
    float32 f32HeadingVar;          /** Variance of heading for this object */
    float32 f32CovarHeadingYawRate; // Covariance of heading and yaw rate for
                                    //    this object

    /* Qualities */
    float32 f32ObstacleProbability; /** Track Quality **/
    float32 f32ExistenceQuality;    /** Existence Quality **/
    float32 fRCS;  // set RCS value for radar, new value for the tuerme
    uint8 eObjMaintenanceState;  // set eObjMaintenanceState value for radar,
                                 // new value for the tuerme

    /* Geometry */
    float32 f32Width;  /** [m] */
    float32 f32Length; /** [m] */
    float32 f32Height; /** [m] */

    /* Object Definition */
    uint16
        u16MotionType; // stationary, moving, stopped, ... (find the definition
                        //   in: TueObjFusn_ObjListInputConstants.h)
    uint16
        u16Class; // Object Class: vehicle, two-wheeler, pedestrian, ... (find
                    //  the definition in: TueObjFusn_ObjListInputConstants.h)
    uint16 u16ClassProb; // classification probability(0 is default; 1...99
                            // is "probability value"; 101 is "probability
                            // undecided/unavailable"

    uint8 u8RadarMotionTypeInput;  // get motion type from radar input, new
                                   // value for the tuerme. set value from enum
                                   // TueFusion_SensorInputMotion

    /*　customer requirement */
    boolean bCIPVFlag; /** if the object is the key car */
    boolean bMCPFlag;  /** if the object is the key ped */
    uint16 uFusionSts; /** object fusion status : fusion /camera / radar */
    uint16 uIDrecord;  /** the record the id of fisrt detected the object*/
    uint16 uLifeCycle; /** the life cycle of the object */
} Fusn_TrackTypeOut;

typedef struct {
    uint16 u16ListUpdateCounter; // Counter that is incremented whenever a
                                    // module outputs an updated object(starting
                                    // from 0, maximum value is 60.000, counter
                                    // will overflow to 0 afterwards)
    uint16 u16NumObjects;        // Number of trackables which have a non-dying
                                 // lifespan
    uint32 u32SensorPattern;     // Sensor pattern
    float32
        f32MeasurementLatency; // Common age [s] of all object estimations
                                // within the data array: positive latencies
                                // indicate outdated information; negative
                                // latencies indicate predicted object data
    Fusn_TrackTypeOut aTrackable[128]; /** object array with limited capacity */
} Fusn_ObjectListOut;

typedef struct {
    uint32 u32ErrorCode;
    uint8 u8AAU_Code;
    uint8 u8FunctionCode;
    uint16 u16Age;
} Fusn_ErrorTypeOut;

typedef struct {
    Fusn_ErrorTypeOut
        aErrorBuffer[5];    // ringbuffer items for an amount of time
    uint16 u16NextWriteIdx; // index of next free item
    uint16 u16NumOfItems;   // Note that u16NumOfItems does not necessarily
                            //    point to the last object in the buffer
} Fusn_ErrorBufferOut;

typedef struct {
    Fusn_ObjectListOut objListOutput;
    // Fusn_ObjectListOut objListOutputIDManager;
    Fusn_ErrorBufferOut errTypeOutput;
} proFusionPrtList_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
// __declspec(dllexport) void ENVM_Exec(const reqEMPrtList_t* reqPorts,
// proEnvmPrtList_t* proPorts);
// void ENVM_Exec(const reqEMPrtList_t* reqPorts, proEnvmPrtList_t* proPorts);
// void Fusion_Init();
// void Fusion_Main(void);
// __declspec(dllexport) void ENVM_Exec_Wrapper(const baseReqEMPrtList_t*
// reqPorts, baseProEnvmPrtList_t* proPorts, ExtObjectList_t* pRadarInput,
// CamObjectList* pCamerInput, uint32 uTimestamp); extern void
// ENVM_Exec_Wrapper(const baseReqEMPrtList_t* reqPorts, baseProEnvmPrtList_t*
// proPorts,ExtObjectList_t* pRadarInput , CamObjectList* pCamerInput, uint32
// uTimestamp);

#ifdef __cplusplus
}
#endif
#endif // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_FUSION_EXT_H_
