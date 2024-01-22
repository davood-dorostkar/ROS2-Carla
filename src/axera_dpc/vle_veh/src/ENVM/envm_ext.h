/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef ENVM_EXT_H
#define ENVM_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
#include "TM_Global_TypeDefs.h"
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

// typedef uint32 AlgoInterfaceVersionNumber_t;

#define SRR_RAM_OBJ_NUM 40
#define Envm_NR_PRIVOBJECTS (40)
#define TUE_RADAR_RAW_OBJECT_NUM 60  // radar object number get from sensor
#define TUE_SRR_RADAR_RAW_OBJECT_NUM \
    60  // SRR radar object number get from sensor
#define EM_SINGAL_SRR_GEN_OBJECT_NUM \
    SRR_RAM_OBJ_NUM  // number of object output for one single SRR object
#define EM_Fusion_GEN_OBJECT_NUM 100u
// number of object output for one single SRR object
#define CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX (40u)
#define TUE_PRV_FUSION_MATH_PI (3.141592653589793f)
#define TUE_PRV_FUSION_MATH_PI_HALF (1.570796326794896f)

#define ADCU_FID_NUM 36u
#define PDCU_FID_NUM 26u

#ifndef FLT_ZERO
#define FLT_ZERO (float32)(0.0f)
#endif

#ifndef Rte_TypeDef_ENVMSignalHeader_t
typedef struct {
    uint32 uiTimeStamp;
    uint16 uiMeasurementCounter;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
} ENVMSignalHeader_t;
#define Rte_TypeDef_ENVMSignalHeader_t
#endif

#ifndef Rte_TypeDef_BSW_s_EnvmCtrlData_t
typedef struct {
    AlgoInterfaceVersionNumber_t u_VersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint16 HWSample;
    uint8 EnvmOpMode;
    boolean RSPCycleViolation;
    boolean EnvmCycleViolation;
    uint32 uiTimeStamp_us;  // system time stamp us
    unsigned char u_Dummy81;
    unsigned char u_Dummy82;
    unsigned char u_Dummy83;
} BSW_s_EnvmCtrlData_t;
#define Rte_TypeDef_BSW_s_EnvmCtrlData_t
#endif

// typedef sint8 aiOOIList_array_t[6];

#ifndef Rte_TypeDef_HeaderAssessedObjList_t
typedef struct {
    sint8 iNumOfUsedObjects;
    unsigned char iPadding;
    sint8 aiOOIList[6];
    uint8 eRelObjLossReason;
} HeaderAssessedObjList_t;
#define Rte_TypeDef_HeaderAssessedObjList_t
#endif

#ifndef Rte_TypeDef_LaneInformation_t
typedef struct {
    uint8 eAssociatedLane;
    uint8 eFuncAssociatedLane;
    unsigned char uiCutInProbability;
    unsigned char uiCutOutProbability;
} LaneInformation_t;
#define Rte_TypeDef_LaneInformation_t
#endif

#ifndef Rte_TypeDef_ObjOfInterest_t
typedef struct {
    unsigned char cExternalID;
    sint8 eObjOOI;
} ObjOfInterest_t;
#define Rte_TypeDef_ObjOfInterest_t
#endif

#ifndef Rte_TypeDef_LegacyAOL_t
typedef struct {
    float fDistToRef;
} LegacyAOL_t;
#define Rte_TypeDef_LegacyAOL_t
#endif

#ifndef Rte_TypeDef_FCTPubObject_t
typedef struct {
    LaneInformation_t LaneInformation;
    ObjOfInterest_t ObjOfInterest;
    LegacyAOL_t Legacy;
} FCTPubObject_t;
#define Rte_TypeDef_FCTPubObject_t
#endif

#ifndef ObjList_array_t_define
typedef FCTPubObject_t ObjList_array_t[40];
#define ObjList_array_t_define
#endif

#ifndef Rte_TypeDef_AssessedObjList_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    HeaderAssessedObjList_t HeaderAssessedObjList;
    ObjList_array_t ObjList;
} AssessedObjList_t;
#define Rte_TypeDef_AssessedObjList_t
#endif

#ifndef Rte_TypeDef_VEDMotVarVehDyn_t
typedef struct {
    float Velocity;
    float Accel;
    float varVelocity;
    float varAccel;
} VEDMotVarVehDyn_t;
#define Rte_TypeDef_VEDMotVarVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDVeloCorrVehDyn_t
typedef struct {
    float corrFact;
    float corrVar;
    float corrVelo;
    float corrVeloVar;
    float minVelo;
    float maxVelo;
    uint8 corrQual;
    boolean bRollerTestBench;
} VEDVeloCorrVehDyn_t;
#define Rte_TypeDef_VEDVeloCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDAccelCorrVehDyn_t
typedef struct {
    float corrAccel;
    float corrAccelVar;
} VEDAccelCorrVehDyn_t;
#define Rte_TypeDef_VEDAccelCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLongitudinal_t
typedef struct {
    VEDMotVarVehDyn_t MotVar;
    VEDVeloCorrVehDyn_t VeloCorr;
    VEDAccelCorrVehDyn_t AccelCorr;
    float fBrakePedalPosGrad_nu;  // Time derivative of brake pedal position
    float fGasPedalPosGrad_nu;    // Time derivative of gas pedal position
} VEDLongitudinal_t;
#define Rte_TypeDef_VEDLongitudinal_t
#endif

#ifndef Rte_TypeDef_VEDYawRateVehDyn_t
typedef struct {
    float YawRate;
    float Variance;
    float Quality;
} VEDYawRateVehDyn_t;
#define Rte_TypeDef_VEDYawRateVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDCurveVehDyn_t
typedef struct {
    float Curve;
    float C1;
    float Gradient;
    float varC0;
    float varC1;
    float Quality;
    float CrvError;
    unsigned char CrvConf;
} VEDCurveVehDyn_t;
#define Rte_TypeDef_VEDCurveVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDDrvIntCurveVehDyn_t
typedef struct {
    float Curve;
    float Variance;
    float Gradient;
} VEDDrvIntCurveVehDyn_t;
#define Rte_TypeDef_VEDDrvIntCurveVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLatAccelVehDyn_t
typedef struct {
    float LatAccel;
    float Variance;
} VEDLatAccelVehDyn_t;
#define Rte_TypeDef_VEDLatAccelVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDSideSlipVehDyn_t
typedef struct {
    float SideSlipAngle;
    float Variance;
} VEDSideSlipVehDyn_t;
#define Rte_TypeDef_VEDSideSlipVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLateral_t
typedef struct {
    VEDYawRateVehDyn_t YawRate;
    float OffCompStWheelAngle;
    VEDCurveVehDyn_t Curve;
    VEDDrvIntCurveVehDyn_t DrvIntCurve;
    VEDLatAccelVehDyn_t Accel;
    VEDSideSlipVehDyn_t SlipAngle;
    float32 fStWheelAngleGradient_rps;  // the gradient of steer wheel angle.
                                        // unit: rad per second
} VEDLateral_t;
#define Rte_TypeDef_VEDLateral_t
#endif

#ifndef Rte_TypeDef_VEDMotionStateVehDyn_t
typedef struct {
    uint8 MotState;
    float Confidence;
} VEDMotionStateVehDyn_t;
#define Rte_TypeDef_VEDMotionStateVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLegacyVehDyn_t
typedef struct {
    float YawRateMaxJitter;
    boolean bStandStill;
} VEDLegacyVehDyn_t;
#define Rte_TypeDef_VEDLegacyVehDyn_t
#endif

// typedef uint8 State_array_t[12];

#ifndef Rte_TypeDef_VEDAddCalculateSign_t
typedef struct {
    float SpeedOdometerCalculate_mps;
} VEDAddCalculateSign_t;
#define Rte_TypeDef_VEDAddCalculateSign_t
#endif

#ifndef Rte_TypeDef_VEDVehDyn_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    VEDLongitudinal_t Longitudinal;
    VEDLateral_t Lateral;
    VEDMotionStateVehDyn_t MotionState;
    VEDLegacyVehDyn_t Legacy;
    uint8 State[12];
    VEDAddCalculateSign_t AddCalculateSign;
} VEDVehDyn_t;
#define Rte_TypeDef_VEDVehDyn_t
#endif

// typedef uint8 State_array_t_4[16];

// typedef float ang_array_t[2];

// typedef float rat_array_t[2];

#ifndef Rte_TypeDef_VEDSwa_t
typedef struct {
    ang_array_t ang;
    rat_array_t rat;
} VEDswa_t;
#define Rte_TypeDef_VEDSwa_t
#endif

// typedef float rat_array_t_0[2];

// typedef float vel_array_t[2];

#ifndef Rte_TypeDef_VEDvel_t
typedef struct {
    vel_array_t vel;
    rat_array_t_0 rat;
} VEDvel_t;
#define Rte_TypeDef_VEDvel_t
#endif

#ifndef Rte_TypeDef_VEDStRatio_t
typedef struct {
    VEDswa_t swa;
    VEDvel_t vel;
} VEDStRatio_t;
#define Rte_TypeDef_VEDStRatio_t
#endif

#ifndef Rte_TypeDef_VEDVehParMain_t
typedef struct {
    uint8 State[16];
    float SelfSteerGrad;
    VEDStRatio_t SteeringRatio;
    float DIST_FrontAxle_to_FontBumper;
    float WheelBase;
    float TrackWidthFront;
    float TrackWidthRear;
    float VehWeight;
    float CntrOfGravHeight;
    float AxisLoadDistr;
    float WhlLoadDepFrontAxle;
    float WhlLoadDepRearAxle;
    float WhlCircumference;
    uint8 DrvAxle;
    unsigned char WhlTcksPerRev;
    float FrCrnrStiff;
    float ReCrnrStiff;
} VEDVehParMain_t;
#define Rte_TypeDef_VEDVehParMain_t
#endif

// typedef uint8 State_array_t_5[12];

#ifndef Rte_TypeDef_VehParAdd_t
typedef struct {
    State_array_t_5 State;
    float VehicleWidth;
    float VehicleLength;
    float CurbWeight;
    float OverhangFront;
    float FrontAxleRoadDist;
    float WheelWidth;
    float PassableHeight;
    float DistCameraToHoodX;
    float DistCameraToHoodY;
    uint8 SteeringVariant;
} VehParAdd_t;
#define Rte_TypeDef_VehParAdd_t
#endif

// typedef uint8 State_array_t_6[8];

#ifndef Rte_TypeDef_VEDSensorMounting_t
typedef struct {
    uint8 State[8];
    float LatPos;
    float LongPos;
    float VertPos;
    float LongPosToCoG;
    float PitchAngle;
    uint8 Orientation;
    float RollAngle;
    float YawAngle;
} VEDSensorMounting_t;
#define Rte_TypeDef_VEDSensorMounting_t
#endif

// typedef uint8 State_array_t_7[8];

#ifndef Rte_TypeDef_VEDSensor_t
typedef struct {
    State_array_t_7 State;
    float CoverDamping;
    float fCoverageAngle;
    float fLobeAngle;
    float fCycleTime;
    unsigned char uNoOfScans;
} VEDSensor_t;
#define Rte_TypeDef_VEDSensor_t
#endif

#ifndef Rte_TypeDef_VEDVehPar_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    VEDVehParMain_t VehParMain;
    VehParAdd_t VehParAdd;
    VEDSensorMounting_t SensorMounting;
    VEDSensor_t Sensor;
} VEDVehPar_t;
#define Rte_TypeDef_VEDVehPar_t
#endif

#ifndef Rte_TypeDef_EnvmNvmState_t
typedef struct {
    uint32 StateTrafficOrientation;
} EnvmNvmState_t;
#define Rte_TypeDef_EnvmNvmState_t
#endif

#ifndef Rte_TypeDef_EnvmNvmIn_t
typedef struct {
    uint8 TrafficOrientation;
    EnvmNvmState_t NVMInState;
} EnvmNvmIn_t;
#define Rte_TypeDef_EnvmNvmIn_t
#endif

#ifndef Rte_TypeDef_EBAPreBrkAccelTabNv_t
typedef struct {
    float Velo;
    float Accel;
} EBAPreBrkAccelTabNv_t;
#define Rte_TypeDef_EBAPreBrkAccelTabNv_t
#endif

#ifndef Rte_TypeDef_EBAPreBrkAccelParNv_t
typedef EBAPreBrkAccelTabNv_t AccelL1_array_t[4];
typedef EBAPreBrkAccelTabNv_t AccelL2_array_t[4];
typedef struct {
    uint8 Valid;
    AccelL1_array_t AccelL1;
    AccelL2_array_t AccelL2;
} EBAPreBrkAccelParNv_t;
#define Rte_TypeDef_EBAPreBrkAccelParNv_t
#endif

#ifndef Rte_TypeDef_Fct_eba_algo_parameters_t
typedef struct {
    boolean CodingValid;
    uint32 CodingBits;
    EBAPreBrkAccelParNv_t PreBrkParAccelTab;
} Fct_eba_algo_parameters_t;
#define Rte_TypeDef_Fct_eba_algo_parameters_t
#endif

#ifndef Rte_TypeDef_Fct_acc_algo_parameters_t
typedef struct {
    uint8 CameraFusionPreselBits;
    uint8 NaviFusionPreselBits;
} Fct_acc_algo_parameters_t;
#define Rte_TypeDef_Fct_acc_algo_parameters_t
#endif

#ifndef Rte_TypeDef_Fct_general_algo_par_t
typedef struct {
    uint8 FnSwitchBits;
} Fct_general_algo_par_t;
#define Rte_TypeDef_Fct_general_algo_par_t
#endif

#ifndef Rte_TypeDef_Fct_algo_parameters_t
typedef struct {
    Fct_eba_algo_parameters_t Eba;
    Fct_acc_algo_parameters_t Acc;
    Fct_general_algo_par_t General;
} Fct_algo_parameters_t;
#define Rte_TypeDef_Fct_algo_parameters_t
#endif

#ifndef Rte_TypeDef_BSW_s_AlgoParameters_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    Fct_algo_parameters_t Fct;
} BSW_s_AlgoParameters_t;
#define Rte_TypeDef_BSW_s_AlgoParameters_t
#endif

typedef enum {
    SRR_LOC_FRONT_LEFT,   // = 0
    SRR_LOC_FRONT_RIGHT,  // = 1
    SRR_LOC_REAR_LEFT,    // = 2
    SRR_LOC_REAR_RIGHT,   // = 3
    SRR_SENSOR_NUM = 6    // = 4
} SRR_Location_en;

#ifndef Rte_TypeDef_SRRWarning_St
typedef struct {
    uint8 u8WarningDOW;
    uint8 u8WarningFCW;
    uint8 u8WarningLCA;
    uint8 u8WarningRCTA;
} SRRWarning_St;
#define Rte_TypeDef_SRRWarning_St
#endif

#ifndef Rte_TypeDef_SRRObj_st
typedef struct {
    uint8 ObjectId;
    float32 fDistX;          /*m*/
    float32 fDistY;          /*m*/
    float32 fVrelX;          /*m/s*/
    float32 fVrelY;          /*m/s*/
    float32 fArelX;          /*m/s2*/
    float32 fArelY;          /*m/s2*/
    float32 fWidth;          /*m*/
    float32 fLength;         /*m*/
    uint8 iDynamicProperty;  // 0x0: moving,0x1: stationary,0x2: oncoming,0x3:
                             // stationary candidate,0x4: unknown,0x5: crossing
                             // stationary,0x6: crossing moving,0x7: stopped
    float32 fRCS;
    float32 fDistXStd;
    float32 fDistYStd;
    float32 fVrelXStd;
    float32 fVrelYStd;
    float32 fArelXStd;
    float32 fArelYStd;
    float32 fOrientationStd;
    float32 fProbabilityOfExistence;
    uint8 iObjMaintenanceState;  // 0x0: deleted,0x1: new,0x2: measured,0x3:
                                 // predicted,0x4: deleted for merge,0x5: new
                                 // from merge
    uint8
        iClassification;  // 0x0: point,0x1: car,0x2: truck,0x3: pedestrian,0x4:
                          // motorcycle,0x5: bicycle,0x6: wide,0x7: reserved
    float32 fOrientation;
} SRRObj_st;
#define Rte_TypeDef_SRRObj_st
#endif

typedef SRRObj_st ExtSRRObj_array_t[SRR_RAM_OBJ_NUM];

#ifndef Rte_TypeDef_SRRPos_st
typedef struct {
    uint32 uiSRRLoc;    // set value based on SRR_Location_en enum value
    float32 fPosX;      /* m */
    float32 fPosY;      /* m */
    float32 fPosZ;      /* m */
    float32 fHeadAngle; /* rad */
} SRRPos_st;
#define Rte_TypeDef_SRRPos_st
#endif

#ifndef Rte_TypeDef_ExtSRRObjList_t
typedef struct {
    ENVMSignalHeader_t sSigHeader;
    SRRPos_st sSRRPos;
    uint8 uiNumOfObjects;
    ExtSRRObj_array_t sRRObject;
    SRRWarning_St sWarning;
} ExtSRRObjList_t;
#define Rte_TypeDef_ExtSRRObjList_t
#endif

typedef ExtSRRObjList_t ExtSRRObj[6];

#ifndef Rte_TypeDef_ExtSRRObjList_array_t
typedef struct {
    uint32 uiVersionNumber;
    uint32 iNumofSRRs;
    ExtSRRObj sRRObjects;
} ExtSRRObjList_array_t;
#define Rte_TypeDef_ExtSRRObjList_array_t
#endif

// #ifndef Rte_TypeDef_CamObjKinematics
// typedef struct {
//     float32 fDistX;
//     float32 fDistXStd;
//     float32 fDistY;
//     float32 fDistYStd;
//     float32 fVx;
//     float32 fVxStd;
//     float32 fVy;
//     float32 fVyStd;
//     float32 fAx;
//     float32 fAxStd;
//     float32 fAy;
//     float32 fAyStd;
//     float32 fOrientation;
//     uint8 eRefpointLocation;
// } CamObjKinematics;
// #define Rte_TypeDef_CamObjKinematics
// #endif

// #ifndef Rte_TypeDef_CamObjGeometry
// typedef struct {
//     float32 fHeight;
//     float32 fOffsetToGround;
//     float32 fLength;
//     float32 fWidth;
// } CamObjGeometry;
// #define Rte_TypeDef_CamObjGeometry
// #endif

// #ifndef Rte_TypeDef_CamObjClassification
// typedef struct {
//     uint8 probabilityOfClassification;
//     uint8 ObjectType;
// } CamObjClassification;
// #define Rte_TypeDef_CamObjClassification
// #endif

// #ifndef Rte_TypeDef_CamObjAttributes
// typedef struct {
//     uint8 eStatusBrakeLight;
//     uint8 uiBrakeLightConfidence;
//     uint8 eStatusTurnIndicator;
//     uint8 uiTurnIndicatorConfidence;
//     uint8 eAssociatedLane;
//     uint8 uiAssociatedLaneConfidence;
//     uint8 eLaneChangeManeuver;
//     uint8 percentageOwnDrivingLane;
//     uint8 percentageSideDrivingLaneLeft;
//     uint8 percentageSideDrivingLaneRight;
//     uint8 eMainteanceState;
//     uint8 percentageProbOfExist;
//     uint8 eMotionState;
//     uint8 eObjectOcclusion;
//     uint32 ObjectLifeCycles;
// } CamObjAttributes;
// #define Rte_TypeDef_CamObjAttributes
// #endif

// #ifndef Rte_TypeDef_CamObject
// typedef struct {
//     uint8 ObjectID;
//     CamObjKinematics Kinematic;
//     CamObjGeometry Geometry;
//     CamObjClassification Classification;
//     CamObjAttributes Attributes;
// } CamObject;
// #define Rte_TypeDef_CamObject
// #endif

// typedef CamObject CamObjectArray[24];

#ifndef Rte_TypeDef_CamLane_t
typedef struct {
    uint8 Type;
    uint8 Quality;
    float Position;
    float Curvature;
    float CurvatureDer;
    float WidthMarking;
    float HeadingAngle;
    float ViewRangeStart;
    float ViewRangeEnd;
    uint8 LineCrossing;
    uint8 LineMarkColor;
    uint8 CanMsgEnd;
} CamLane_t;
#define Rte_TypeDef_CamLane_t
#endif

typedef CamLane_t CamLane_array_t[4];

// #ifndef Rte_TypeDef_CamObjectList
// typedef struct {
//     AlgoInterfaceVersionNumber_t uiVersionNumber;
//     // ENVMSignalHeader_t sSigHeader;
//     // CamObjectArray aObject;
//     // CamLane_array_t sLaneLines;
// } CamObjectList;
// #define Rte_TypeDef_CamObjectList
// #endif

#ifndef Rte_TypeDef_HeaderObjList_t
typedef sint8 iSortedObjectList_array_t[Envm_NR_PRIVOBJECTS];
typedef struct {
    sint8 iNumOfUsedObjects;
    iSortedObjectList_array_t iDistXSortedObjectList;
} HeaderObjList_t;
#define Rte_TypeDef_HeaderObjList_t
#endif

#ifndef Rte_TypeDef_Kinematic_t
typedef struct {
    float32 fDistX;
    float32 fDistXStd;
    float32 fDistY;
    float32 fDistYStd;
    float32 fVrelX;
    float32 fVrelXStd;
    float32 fVrelY;
    float32 fVrelYStd;
    float32 fVabsX;
    float32 fVabsXStd;
    float32 fVabsY;
    float32 fVabsYStd;
    float32 fArelX;
    float32 fArelXStd;
    float32 fArelY;
    float32 fArelYStd;
    float32 fAabsX;
    float32 fAabsXStd;
    float32 fAabsY;
    float32 fAabsYStd;
} Kinematic_t;
#define Rte_TypeDef_Kinematic_t
#endif

#ifndef Rte_TypeDef_Geometry_t
typedef struct {
    float32 fWidth;
    float32 fLength;
    float32 fOrientation;
    float32 fOrientationStd;
    float32 fOrientationValid;
} Geometry_t;
#define Rte_TypeDef_Geometry_t
#endif

#ifndef Rte_TypeDef_Attributes_t
typedef struct {
    uint8 eDynamicProperty;
    uint8 uiStoppedConfidence;
    uint8 eAbsMovingState;
    uint8 eClassification;
    uint8 uiClassConfidence;
} Attributes_t;
#define Rte_TypeDef_Attributes_t
#endif

#ifndef Rte_TypeDef_General_t
typedef struct {
    float32 fLifeTime;
    uint32 fTimeStamp;
    uint8 eObjMaintenanceState;
    uint8 cObjMergingID;
} General_t;
#define Rte_TypeDef_General_t
#endif

#ifndef Rte_TypeDef_Qualifiers_t
typedef struct {
    float32 fProbabilityOfExistence;
    uint8 ucObstacleProbability;  // add Obstacle Probability for ars410 radar
    uint8 uMeasuredTargetFrequencyNear;
    uint8 uMeasuredTargetFrequencyFar;
} Qualifiers_t;
#define Rte_TypeDef_Qualifiers_t
#endif

#ifndef Rte_TypeDef_SensorSpecific_t
typedef struct {
    float32 fRCS;
    boolean bCamConfirmed;  // true: object has been observed by camera in last
                            // 100 cycle
} SensorSpecific_t;
#define Rte_TypeDef_SensorSpecific_t
#endif

#ifndef Rte_TypeDef_EBAPresel_t
typedef struct {
    uint16 eEbaInhibitionMask;
    uint8 ucEbaMovingObjQuality;
    uint8 eEbaHypCat;
    boolean bCrossingPedEbaPresel;
    boolean bCrossingBicycleEbaPresel;  // wulin todo 20220316, add
                                        // bCrossingBicycleEbaPresel
} EBAPresel_t;
#define Rte_TypeDef_EBAPresel_t
#endif

#ifndef Rte_TypeDef_LegacyObj_t
typedef struct {
    float32 fMaxAccelY;
    float32 fRCSTargetThresholdUncomp;
    uint16 uiLifeTime;
    uint8 eDynamicSubProperty;
} LegacyObj_t;
#define Rte_TypeDef_LegacyObj_t
#endif

#ifndef Rte_TypeDef_ExtObjects_t
typedef struct {
    sint32 ObjectId;  // changed by 20210915 // uint8
    Kinematic_t Kinematic;
    Geometry_t Geometry;
    Attributes_t Attributes;
    General_t General;
    Qualifiers_t Qualifiers;
    SensorSpecific_t SensorSpecific;
    EBAPresel_t EBAPresel;
    LegacyObj_t Legacy;
} ExtObjects_t;
#define Rte_TypeDef_ExtObjects_t
#endif

typedef ExtObjects_t Ext_Objects_array_t[EM_Fusion_GEN_OBJECT_NUM];
typedef ExtObjects_t PreSelet_Objects_array_t[Envm_NR_PRIVOBJECTS];

#ifndef Rte_TypeDef_PreSeletObjectList_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint32 uiTimeStamp;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
    HeaderObjList_t HeaderObjList;
    PreSelet_Objects_array_t Objects;
} PreSeletObjectList_t;
#define Rte_TypeDef_PreSeletObjectList_t
#endif

#ifndef Rte_TypeDef_ExtObjectList_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint32 uiTimeStamp;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
    HeaderObjList_t HeaderObjList;
    Ext_Objects_array_t Objects;
} ExtObjectList_t;
#define Rte_TypeDef_ExtObjectList_t
#endif

#ifndef Rte_TypeDef_Bbox2D
typedef struct {
    boolean initialized;               // Flag of initialization
    float32 top_left_x;                // X coordinate of top left point
    float32 top_left_y;                // Y coordinate of top left point
    float32 bottom_right_x;            // X coordinate of bottom right point
    float32 bottom_right_y;            // Y coordinate of bottom right point
    float32 confidence;                // confidence
    boolean is_left_side_truncated;    // Whether the left side is truncated
    boolean is_right_side_truncated;   // Whether the right side is truncated
    boolean is_top_side_truncated;     // Whether the top side is truncated
    boolean is_bottom_side_truncated;  // Whether the bottom side is truncated
} Bbox2D;
#define Rte_TypeDef_Bbox2D
#endif

#ifndef Rte_TypeDef_CameraBBox2DInfo
typedef struct {
    Bbox2D raw_detection_box;  // raw detection box
    Bbox2D tracked_box;        // tracked box
} CameraBBox2DInfo;
#define Rte_TypeDef_CameraBBox2DInfo
#endif

#ifndef Rte_TypeDef_CamTrafficLight_t
typedef struct {
    // int label;              // enum ObjectLabel
    // uint16 status_label;
    // /* status definition:
    // # 0 BLACK,
    // # 1 GREEN,
    // # 2 YELLOW,
    // # 3 RED,
    // */
    // uint16 color_label;
    // /* color definition:
    // # NONE = 0x00,
    // # GREEN = 0x02,
    // # YELLOW = 0x04,
    // # RED = 0x08,
    // # GREEN_FLASH = 0x10,
    // # YELLOW_FLASH = 0x20,
    // # BLACK = 0x40,
    // # GREEN_NUMBER = 0x80
    // */
    // uint16 type_label;
    // /* type definition
    // # 0 UNDEFINE,
    // # 1 NONE,
    // # 2 CIRCLE,
    // # 3 PEDESTRIAN ,
    // # 4 TURN_LEFT,
    // # 5 STRAIGHT,
    // # 6 TURN_RIGHT,
    // # 7 TURN_AROUND,
    // # 8 TURN_LEFT_STRAIGHT,
    // # 9 TURN_LEFT_AROUND,
    // # 10 TURN_RIGHT_STRAIGHT,
    // # 11 BIKE,
    // # 12 X_ENTER,
    // # 13 NUMBER,
    // # 14 ENTER,
    // */
    // float32 type_confidence;            // Type of confidence
    // float32 existence_confidence;       // Existence of confidence
    // sint32 track_id;                    // Id
    // CameraBBox2DInfo camera_bbox_info;  // Bounding box

    uint32 id;

    float32 position_x;
    float32 position_y;
    float32 position_z;

    float32 quaternion_x;
    float32 quaternion_y;
    float32 quaternion_z;
    float32 quaternion_w;

    float32 length;
    float32 width;
    float32 height;
    uint8 detection_status;
    uint8 confidence;

    uint8 status_label;
    // Unknown = 0 Invalid = 1 Off = 2 Green = 3 Yellow = 4 Red = 5 GreenFlash =
    //     6 YellowFlash = 7

    uint8 type_label;
    // None = 0 未定义 等同于Unknown White = 1 Yellow = 2
    //     Orange = 3
    //     Blue = 4 Green = 5 Gray = 6 LeftGrayRightYellow =
    //         7 LeftYellowRightWhite = 8 Other = 255

    sint32 lane_ids[8];
} CamTrafficLight_t;
#define Rte_TypeDef_CamTrafficLight_t
#endif

typedef CamTrafficLight_t CamTrafficLight_array_t[10];

#ifndef Rte_TypeDef_CamTrafficSign_t
typedef struct {
    // sint16 label;
    // float type_confidence;       // type of confidence
    // float existence_confidence;  // existence of confidence
    // sint32 track_id;             // id
    // CameraBBox2DInfo camera_bbox_info;

    uint32 id;

    float32 position_x;
    float32 position_y;
    float32 position_z;

    float32 quaternion_x;
    float32 quaternion_y;
    float32 quaternion_z;
    float32 quaternion_w;

    float32 length;
    float32 width;
    float32 height;
    uint8 detection_status;
    uint8 confidence;

    uint32 label;
    // Unknown = 0,                           //未知
    //     RoadWorks = 1,                     //道路施工
    //     Stop = 2,                          //停止标示
    //     OvertakingProhibited = 3,          //禁止超车
    //     EndOfProhibitionOnOvertaking = 4,  //解除禁止超车
    //     ChildrenAndSchoolZone = 5,         //学校区域
    //     MinSpeedLimit = 6,                 //最小限速
    //     MaxSpeedLimit = 7,                 //最大限速
    //     EndOfSpeedLimit = 8,               //限速结束
    //     NoEntrance = 9,                    //禁止驶入
    //     AllSpeedLimitCancel = 10,          //取消所有限速
    //     NoParkingSign = 11,                //禁止停车
    //     StartOfHighway = 12,               //高速公路起点
    //     EndOfHighway = 13,                 //高速公路终点
    //     LeftCurve = 14,                    //向左急转弯路
    //     RightCurve = 15,                   //向右急转弯路
    //     SeriesCurves = 16,                 //连续弯路
    //     Others = 17,                       //其他
    //     Speed_limit_5 = 18, Speed_limit_10 = 19, Speed_limit_20 = 20,
    // Speed_limit_30 = 21, Speed_limit_40 = 22, Speed_limit_50 = 23,
    // Speed_limit_60 = 24, Speed_limit_70 = 25, Speed_limit_80 = 26,
    // Speed_limit_90 = 27, Speed_limit_100 = 28, Speed_limit_110 = 29,
    // Speed_limit_120 = 30,
    // End_speed_limit_5 = 31, End_speed_limit_10 = 32, End_speed_limit_20 = 33,
    // End_speed_limit_30 = 34, End_speed_limit_40 = 35, End_speed_limit_50 =
    // 36, End_speed_limit_60 = 37, End_speed_limit_70 = 38, End_speed_limit_80
    // = 39, End_speed_limit_90 = 40, End_speed_limit_100 = 41,
    // End_speed_limit_110 = 42, End_speed_limit_120 = 43,
    // Minimum_speed_limit_50 = 44, Minimum_speed_limit_60 = 45,
    // Minimum_speed_limit_70 = 46, Minimum_speed_limit_80 = 47,
    // Minimum_speed_limit_90 = 48, Minimum_speed_limit_100 = 49,
    // Minimum_speed_limit_110 = 50, Overtake_restriction = 51,
    // Ending_of_overtake_restriction = 52, Variable_speed_limit_10 = 53,
    // Variable_speed_limit_20 = 54, Variable_speed_limit_30 = 55,
    // Variable_speed_limit_40 = 56, Variable_speed_limit_50 = 57,
    // Variable_speed_limit_60 = 58, Variable_speed_limit_70 = 59,
    // Variable_speed_limit_80 = 60, Variable_speed_limit_90 = 61,
    // Variable_speed_limit_100 = 62, Variable_speed_limit_110 = 63,
    // Variable_speed_limit_120 = 64, Variable_sign_others = 65, Merge_Left =
    // 66, Merge_right = 67, Attention_to_Pedestrians = 68,
    // Attention_to_children = 69, Stop_sign = 70, Slow_down_and_give_way = 71,
    // No_left_turn = 72, No_right_turn = 73, No_U_turn = 74,
    // No_Audible_Warning = 75, Left_turn_sign = 76, Right_turn_sign = 77,
    // pedestrian_crossing = 78, U_turn_Lane = 79,
    // Number_of_lanes_becoming_less = 80, Lane_reducing = 81, Work_zone_sign =
    // 82, Lane_changed = 83, Left_turn_lane = 84, Right_turn_lane = 85,
    // Toll_gate = 86, Left_turn_and_forward_Lane = 87,
    // Right_turn_and_forward_Lane = 88, Left_turn_and_U_turn_Lane = 89,
    // Bus_lane = 90, Close_to_toll_gate = 91, School_ahead_low_down = 92,
    // Ramp = 93, Military_control_zone = 94,
    // Radio_observatory = 95

    sint32 lane_ids[8];
} CamTrafficSign_t;
#define Rte_TypeDef_CamTrafficSign_t
#endif

typedef CamTrafficSign_t CamTrafficSign_array_t[10];

#ifndef Rte_TypeDef_TSRObjectList_t
typedef struct {
    uint8 uTrafficLightsize;
    CamTrafficLight_array_t sTrafficLights;
    uint8 uTrafficSignsize;
    CamTrafficSign_array_t sTrafficSigns;
} TSRObjectList_t;
#define Rte_TypeDef_TSRObjectList_t
#endif

#ifndef Rte_TypeDef_ST_reqVLCVEH_SOA_HMI_t
typedef struct {
    sint32 siFCWHighlightID_nu;
    uint8 uiFCWWarningLevel_enum;
    sint32 siAEBHighlightID_nu;
    uint8 uiAEBWarningLevel_enum;
    sint32 siACCHighlightID_nu;
    uint8 uiACCWarningLevel_enum;
} ST_reqVLCVEH_SOA_HMI_t;
#define Rte_TypeDef_ST_reqVLCVEH_SOA_HMI_t
#endif

#ifndef Rte_TypeDef_ST_reqCTASEN_SOA_HMI_t
typedef struct {
    sint32 siFCTALeftHighlightID_nu;
    uint8 uiFCTALeftWarningLevel_enum;
    sint32 siFCTARightHighlightID_nu;
    uint8 uiFCTARightWarningLevel_enum;
    sint32 siRCTALeftHighlightID_nu;
    uint8 uiRCTALeftWarningLevel_enum;
    sint32 siRCTARightHighlightID_nu;
    uint8 uiRCTARightWarningLevel_enum;
} ST_reqCTASEN_SOA_HMI_t;
#define Rte_TypeDef_ST_reqCTASEN_SOA_HMI_t
#endif

#ifndef Rte_TypeDef_ST_reqLBSSEN_SOA_HMI_t
typedef struct {
    sint32 siLCALeftHighlightID_nu;
    uint8 uiLCALeftWarningLevel_enum;
    sint32 siLCARightHighlightID_nu;
    uint8 uiLCARightWarningLevel_enum;
    sint32 siDOWLeftHighlightID_nu;
    uint8 uiDOWLeftWarningLevel_enum;
    sint32 siDOWRightHighlightID_nu;
    uint8 uiDOWRightWarningLevel_enum;
    sint32 siRCWHighlightID_nu;
    uint8 uiRCWWarningLevel_enum;
} ST_reqLBSSEN_SOA_HMI_t;
#define Rte_TypeDef_ST_reqLBSSEN_SOA_HMI_t
#endif

#ifndef Rte_TypeDef_ST_reqLCFSEN_SOA_HMI_t
typedef struct {
    sint32 siALCHighlightID_nu;
    uint8 uiALCWarningLevel_enum;
} ST_reqLCFSEN_SOA_HMI_t;
#define Rte_TypeDef_ST_reqLCFSEN_SOA_HMI_t
#endif

#ifndef ST_SOA_HMI_t_define
typedef struct {
    ST_reqVLCVEH_SOA_HMI_t reqVLCVEH_SOA_HMI;
    ST_reqCTASEN_SOA_HMI_t reqCTASEN_SOA_HMI;
    ST_reqLBSSEN_SOA_HMI_t reqLBSSEN_SOA_HMI;
    ST_reqLCFSEN_SOA_HMI_t reqLCFSEN_SOA_HMI;
} ST_SOA_HMI_t;
#define ST_SOA_HMI_t_define
#endif

typedef uint8 ST_ADCU_FID_Array_t[ADCU_FID_NUM];
typedef uint8 ST_PDCU_FID_Array_t[PDCU_FID_NUM];

#ifndef Rte_TypeDef_Envm_FIDInport_t
typedef struct {
    ST_ADCU_FID_Array_t ADCU_FID;
    ST_PDCU_FID_Array_t PDCU_FID;
} Envm_FIDInport_t;
#define Rte_TypeDef_Envm_FIDInport_t
#endif

#ifndef Rte_TypeDef_reqEMPrtList_t
typedef struct {
    const BSW_s_EnvmCtrlData_t *pCtrl;
    const AssessedObjList_t *pFctObjectList;
    const VEDVehDyn_t *pVehicleDynData;
    const VEDVehPar_t *pVehicleStatData;
    const EnvmNvmIn_t *pNvmIn;
    const BSW_s_AlgoParameters_t *pAlgoParameters;
    // const CamObjectList *pCamObject;
    const ExtObjectList_t *pExtObjList;
    const TSRObjectList_t *pTSRObject;
    const ST_SOA_HMI_t *pSOA_HMI;
    const Envm_FIDInport_t *pFIDInport;
} reqEMPrtList_t;
#define Rte_TypeDef_reqEMPrtList_t
#endif

// typedef sint8 Envm_t_GenObjSortedIdxArray[40];

#ifndef Rte_TypeDef_EM_t_GenObjListHeader
typedef struct {
    sint8 iNumOfUsedObjects;
    Envm_t_GenObjSortedIdxArray iSortedObjectList;
    uint16 eObjListSource;
} EM_t_GenObjListHeader;
#define Rte_TypeDef_EM_t_GenObjListHeader
#endif

#ifndef Rte_TypeDef_Envm_t_GenObjKinEnvmatics
typedef struct {
    float fDistX;
    float fDistXStd;
    float fDistY;
    float fDistYStd;
    float fVrelX;
    float fVrelXStd;
    float fVrelY;
    float fVrelYStd;
    float fArelX;
    float fArelXStd;
    float fArelY;
    float fArelYStd;
    float fVabsX;
    float fVabsXStd;
    float fVabsY;
    float fVabsYStd;
    float fAabsX;
    float fAabsXStd;
    float fAabsY;
    float fAabsYStd;
} Envm_t_GenObjKinEnvmatics;
#define Rte_TypeDef_Envm_t_GenObjKinEnvmatics
#endif

#ifndef Rte_TypeDef_Envm_t_GenObjShapePointCoord
typedef struct {
    float fPosX;
    float fPosY;
    unsigned short uiPosXStd;
    unsigned short uiPosYStd;
} Envm_t_GenObjShapePointCoord;
#define Rte_TypeDef_Envm_t_GenObjShapePointCoord
#endif

#ifndef Rte_TypeDef_EM_t_GenObjGeometry
typedef Envm_t_GenObjShapePointCoord EM_t_GenObjSPCArray_t[4];
typedef struct {
    EM_t_GenObjSPCArray_t aShapePointCoordinates;
} EM_t_GenObjGeometry;
#define Rte_TypeDef_EM_t_GenObjGeometry
#endif

#ifndef Rte_TypeDef_Envm_t_GenObjAttributes
typedef struct {
    uint8 eDynamicProperty;
    unsigned char uiDynConfidence;
    uint8 eClassification;
    unsigned char uiClassConfidence;
    uint8 eObjectOcclusion;
} Envm_t_GenObjAttributes;
#define Rte_TypeDef_Envm_t_GenObjAttributes
#endif

#ifndef Rte_TypeDef_EM_t_GenObjGenerals
typedef struct {
    float fLifeTime;
    uint16 uiLifeCycles;
    uint8 eMaintenanceState;
    unsigned char uiID;
    uint8 eSplitMergeState;
    unsigned char uiMergeID;
    unsigned char uiSplitID;
} EM_t_GenObjGenerals;
#define Rte_TypeDef_EM_t_GenObjGenerals
#endif

#ifndef Rte_TypeDef_Envm_t_GenObjQualifiers
typedef struct {
    unsigned char uiProbabilityOfExistence;
    unsigned char uiAccObjQuality;
    unsigned char uiEbaObjQuality;
    uint8 eEbaHypCat;
    uint16 eEbaInhibitionMask;
} Envm_t_GenObjQualifiers;
#define Rte_TypeDef_Envm_t_GenObjQualifiers
#endif

#ifndef Rte_TypeDef_Envm_t_GenObject
typedef struct {
    Envm_t_GenObjKinEnvmatics Kinematic;
    EM_t_GenObjGeometry Geometry;
    Envm_t_GenObjAttributes Attributes;
    EM_t_GenObjGenerals General;
    Envm_t_GenObjQualifiers Qualifiers;
} Envm_t_GenObject;
#define Rte_TypeDef_Envm_t_GenObject
#endif

#ifndef Rte_TypeDef_Envm_t_GenObjectList
typedef Envm_t_GenObject Envm_t_GenObjArray[40];
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    EM_t_GenObjListHeader HeaderObjList;
    Envm_t_GenObjArray aObject;
} Envm_t_GenObjectList;
#define Rte_TypeDef_Envm_t_GenObjectList
#endif

#ifndef Rte_TypeDef_Envm_t_CR_KinEnvmatic
typedef struct {
    float fMaxAccelY;
} Envm_t_CR_KinEnvmatic;
#define Rte_TypeDef_Envm_t_CR_KinEnvmatic
#endif

#ifndef Rte_TypeDef_Envm_t_CR_Geometry
typedef struct {
    float fWidth;
    float fLength;
    float fOrientation;
    float fOrientationStd;
} Envm_t_CR_Geometry;
#define Rte_TypeDef_Envm_t_CR_Geometry
#endif

#ifndef Rte_TypeDef_Envm_t_CR_MotionAttributes
typedef struct {
    uint8 eDynamicProperty;
    uint8 uiStoppedConfidence;
    uint8 eAbsMovingState;
    uint8 eDynamicSubProperty;
} Envm_t_CR_MotionAttributes;
#define Rte_TypeDef_Envm_t_CR_MotionAttributes
#endif

#ifndef Rte_TypeDef_Envm_t_CR_Attributes
typedef struct {
    uint8 eClassification;
    uint8 uiClassConfidence;
    unsigned char ui8Dummy;
    uint8 uiReferenceToTrace;
} Envm_t_CR_Attributes;
#define Rte_TypeDef_Envm_t_CR_Attributes
#endif

#ifndef Rte_TypeDef_Envm_t_CR_SensorSpecific
typedef struct {
    float fRCS;
    uint8 ucMeasuredSources;
    uint8 eObjRelationsClass;
    uint8 eFOVOverlapFar;
    boolean bNearObjInBeam;
} Envm_t_CR_SensorSpecific;
#define Rte_TypeDef_Envm_t_CR_SensorSpecific
#endif

#ifndef Rte_TypeDef_Envm_t_CR_Legacy
typedef struct {
    float fAngle;
    float fLastTargetDistY;
    float fLastTargetDistX;
} Envm_t_CR_Legacy;
#define Rte_TypeDef_Envm_t_CR_Legacy
#endif

#ifndef Rte_TypeDef_EM_t_ARSObject
typedef struct {
    Envm_t_CR_KinEnvmatic Kinematic;
    Envm_t_CR_Geometry Geometry;
    Envm_t_CR_MotionAttributes MotionAttributes;
    Envm_t_CR_Attributes Attributes;
    Envm_t_CR_SensorSpecific SensorSpecific;
    Envm_t_CR_Legacy Legacy;
} EM_t_ARSObject;
#define Rte_TypeDef_EM_t_ARSObject
#endif

#ifndef Rte_TypeDef_Envm_t_CRObjectList
typedef EM_t_ARSObject Envm_t_CRObjectArray[40];
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    Envm_t_CRObjectArray aObject;
} Envm_t_CRObjectList;
#define Rte_TypeDef_Envm_t_CRObjectList
#endif

#ifndef Rte_TypeDef_ECAMtCyclEnvmode_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    uint8 eCycleMode;
    float32 fECAMtCycleTime;
} ECAMtCyclEnvmode_t;
#define Rte_TypeDef_ECAMtCyclEnvmode_t
#endif

#ifndef Rte_TypeDef_EnvmDEnvmOut_t
typedef struct {
    uint8 DEMEvent;
} EnvmDEnvmOut_t;
#define Rte_TypeDef_EnvmDEnvmOut_t
#endif

#ifndef Rte_TypeDef_EnvmNvmOut_t
typedef struct {
    uint8 TrafficOrientation;
    EnvmNvmState_t NvmOutState;
} EnvmNvmOut_t;
#define Rte_TypeDef_EnvmNvmOut_t
#endif

#ifndef Rte_TypeDef_Envm_t_CameraLaneOutputFctData
typedef struct {
    uint32 u_VersionNumber;
    ENVMSignalHeader_t sSigHeader;
    CamLane_array_t sLaneLines;
} Envm_t_CameraLaneOutputFctData;
#define Rte_TypeDef_Envm_t_CameraLaneOutputFctData
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenKinematics_t
typedef struct {
    float32 fDistX_met;
    float32 fDistXStd_met;
    float32 fDistY_met;
    float32 fDistYStd_met;
    float32 fVrelX_mps;
    float32 fVrelXStd_mps;
    float32 fVrelY_mps;
    float32 fVrelYStd_mps;
    float32 fArelX_mpss;
    float32 fArelXStd_mpss;
    float32 fArelY_mpss;
    float32 fArelYStd_mpss;
    float32 fVabsX_mps;
    float32 fVabsXStd_mps;
    float32 fVabsY_mps;
    float32 fVabsYStd_mps;
    float32 fAabsX_mpss;
    float32 fAabsXStd_mpss;
    float32 fAabsY_mpss;
    float32 fAabsYStd_mpss;
} EM_Fusion_GenKinematics_t;
#define Rte_TypeDef_EM_Fusion_GenKinematics_t
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenGeometry_t
typedef struct {
    float32 fWidth_met;
    float32 fWidthStd_met;
    float32 fWidthLeft_met;   // the distance from left side to detect point
    float32 fWidthRight_met;  // the distance from right side to detect point
    float32 fLength_met;
    float32 fLengthStd_met;
    float32 fLengthFront_met;  // the distance from front side to detect point
    float32 fLengthRear_met;   // the distance from rear side to detect point
    float32 fOrientation_rad;
    float32 fOrientationStd_rad;
} EM_Fusion_GenGeometry_t;
#define Rte_TypeDef_EM_Fusion_GenGeometry_t
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenGenerals_t
typedef struct {
    float32 fLifeTime_s;
    uint16 uiLifeCycles_nu;
    uint8 uiMaintenanceState_nu;  // 0x0: deleted,0x1: new,0x2: measured,0x3:
                                  // predicted,0x4: deleted for merge,0x5: new
                                  // from merge
    uint16 uiID_nu;
    sint32 iRawFusionID_nu;
} EM_Fusion_GenGenerals_t;
#define Rte_TypeDef_EM_Fusion_GenGenerals_t
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenAttributes_t
typedef struct {
    uint8
        eDynamicProperty_nu;  // 0x0: moving,0x1: stationary,0x2: oncoming,0x3:
                              // stationary candidate,0x4: unknown,0x5: crossing
                              // stationary,0x6: crossing moving,0x7: stopped
    uint8 uiDynConfidence_per;
    uint32 eClassification_nu;  // 0x0: point,0x1: car,0x2: truck,0x3:
                                // pedestrian,0x4: motorcycle,0x5: bicycle,0x6:
                                // wide,0x7: reserved
    uint8 uiClassConfidence_per;
} EM_Fusion_GenAttributes_t;
#define Rte_TypeDef_EM_Fusion_GenAttributes_t
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenQualifiers_t
typedef struct {
    float32 fProbabilityOfExistence_per;
    uint8 uiMeasuredTargetFrequency_nu;
    boolean bObjStable;
} EM_Fusion_GenQualifiers_t;
#define Rte_TypeDef_EM_Fusion_GenQualifiers_t
#endif

#ifndef Rte_TypeDef_EM_Fusion_GenObject_t
typedef struct {
    EM_Fusion_GenKinematics_t sKinematic;
    EM_Fusion_GenGeometry_t sGeometry;
    EM_Fusion_GenGenerals_t sGeneral;
    EM_Fusion_GenAttributes_t sAttribute;
    EM_Fusion_GenQualifiers_t sQualifier;
} EM_Fusion_GenObject_t;
#define Rte_TypeDef_EM_Fusion_GenObject_t
#endif

#ifndef Rte_TypeDef_Envm_FusionObjectList_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint8 uNumOfUsedObjects;
    EM_Fusion_GenObject_t aObjects[EM_Fusion_GEN_OBJECT_NUM];
} Envm_FusionObjectList_t;
#define Rte_TypeDef_Envm_FusionObjectList_t
#endif

#ifndef Rte_TypeDef_Envm_TrafficLight_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint8 uTrafficLightsize;                /* size of traffic lights */
    CamTrafficLight_array_t sTrafficLights; /* traffic light array */
} Envm_TrafficLight_t;
#define Rte_TypeDef_Envm_TrafficLight_t
#endif

#ifndef Rte_TypeDef_Envm_TrafficSign_t
typedef struct {
    uint32 uiVersionNumber;
    ENVMSignalHeader_t sSigHeader;
    uint16 CAM_SpeedUpperLimit_btf;    /* Upper speed Limitsign */
    uint16 CAM_SpeedLowerLimit_btf;    /* Lower speed Limitsign */
    uint16 CAM_SpeedRelUpperLimit_btf; /* Relieve upper speed Limitsign */
    uint16 CAM_SpeedRelLowerLimit_btf; /* Relieve lower speed Limitsign */
    uint32 CAM_ForbiddenSign_btf[2];   /* Forbidden sign */
    uint8 uSpeedUpperLimitSize;
    CamTrafficSign_t sSpeedUpperLimit[10];
    uint8 uSpeedLowerLimitSize;
    CamTrafficSign_t sSpeedLowerLimit[10];
    uint8 uForbiddenSignSize;
    CamTrafficSign_t sForbiddenSign[10];
} Envm_TrafficSign_t;
#define Rte_TypeDef_Envm_TrafficSign_t
#endif

#ifndef Rte_TypeDef_Envm_ADASHighlightID_t
typedef struct {
    sint32 siFCWHighlightID_nu;
    uint8 uiFCWWarningLevel_enum;
    sint32 siAEBHighlightID_nu;
    uint8 uiAEBWarningLevel_enum;
    sint32 siACCHighlightID_nu;
    uint8 uiACCWarningLevel_enum;
    sint32 siFCTALeftHighlightID_nu;
    uint8 uiFCTALeftWarningLevel_enum;
    sint32 siFCTARightHighlightID_nu;
    uint8 uiFCTARightWarningLevel_enum;
    sint32 siRCTALeftHighlightID_nu;
    uint8 uiRCTALeftWarningLevel_enum;
    sint32 siRCTARightHighlightID_nu;
    uint8 uiRCTARightWarningLevel_enum;
    sint32 siLCALeftHighlightID_nu;
    uint8 uiLCALeftWarningLevel_enum;
    sint32 siLCARightHighlightID_nu;
    uint8 uiLCARightWarningLevel_enum;
    sint32 siDOWLeftHighlightID_nu;
    uint8 uiDOWLeftWarningLevel_enum;
    sint32 siDOWRightHighlightID_nu;
    uint8 uiDOWRightWarningLevel_enum;
    sint32 siRCWHighlightID_nu;
    uint8 uiRCWWarningLevel_enum;
    sint32 siALCHighlightID_nu;
    uint8 uiALCWarningLevel_enum;
} Envm_ADASHighlightID_t;
#define Rte_TypeDef_Envm_ADASHighlightID_t
#endif

#ifndef Rte_TypeDef_FIDOutCondition_t
typedef struct {
    boolean bRCA;
    boolean bFCA;
    boolean bFCB;
    boolean bRCC;
} FIDOutCondition_t;
#define Rte_TypeDef_FIDOutCondition_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_ACC_t
typedef struct {
    FIDOutCondition_t FIDOut_ACC_cdt;
} Envm_FIDMOut_ACC_t;
#define Rte_TypeDef_Envm_FIDMOut_ACC_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_AEB_t
typedef struct {
    FIDOutCondition_t FIDOut_AEB_cdt;
} Envm_FIDMOut_AEB_t;
#define Rte_TypeDef_Envm_FIDMOut_AEB_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_DOW_t
typedef struct {
    FIDOutCondition_t FIDOut_DOW_cdt;
} Envm_FIDMOut_DOW_t;
#define Rte_TypeDef_Envm_FIDMOut_DOW_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_FCTA_t
typedef struct {
    FIDOutCondition_t FIDOut_FCTA_cdt;
} Envm_FIDMOut_FCTA_t;
#define Rte_TypeDef_Envm_FIDMOut_FCTA_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_FCW_t
typedef struct {
    FIDOutCondition_t FIDOut_FCW_cdt;
} Envm_FIDMOut_FCW_t;
#define Rte_TypeDef_Envm_FIDMOut_FCW_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_IHBC_t
typedef struct {
    FIDOutCondition_t FIDOut_IHBC_cdt;
} Envm_FIDMOut_IHBC_t;
#define Rte_TypeDef_Envm_FIDMOut_IHBC_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_LDP_t
typedef struct {
    FIDOutCondition_t FIDOut_LDP_cdt;
} Envm_FIDMOut_LDP_t;
#define Rte_TypeDef_Envm_FIDMOut_LDP_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_LDW_t
typedef struct {
    FIDOutCondition_t FIDOut_LDW_cdt;
} Envm_FIDMOut_LDW_t;
#define Rte_TypeDef_Envm_FIDMOut_LDW_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_Pilot_t
typedef struct {
    FIDOutCondition_t FIDOut_Pilot_cdt;
} Envm_FIDMOut_Pilot_t;
#define Rte_TypeDef_Envm_FIDMOut_Pilot_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_RCTA_t
typedef struct {
    FIDOutCondition_t FIDOut_RCTA_cdt;
} Envm_FIDMOut_RCTA_t;
#define Rte_TypeDef_Envm_FIDMOut_RCTA_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_RCW_t
typedef struct {
    FIDOutCondition_t FIDOut_RCW_cdt;
} Envm_FIDMOut_RCW_t;
#define Rte_TypeDef_Envm_FIDMOut_RCW_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_TSR_t
typedef struct {
    FIDOutCondition_t FIDOut_TSR_cdt;
} Envm_FIDMOut_TSR_t;
#define Rte_TypeDef_Envm_FIDMOut_TSR_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_BSD_t
typedef struct {
    FIDOutCondition_t FIDOut_BSD_cdt;
} Envm_FIDMOut_BSD_t;
#define Rte_TypeDef_Envm_FIDMOut_BSD_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_LCA_t
typedef struct {
    FIDOutCondition_t FIDOut_LCA_cdt;
} Envm_FIDMOut_LCA_t;
#define Rte_TypeDef_Envm_FIDMOut_LCA_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOut_NOP_t
typedef struct {
    FIDOutCondition_t FIDOut_NOP_cdt;
} Envm_FIDMOut_NOP_t;
#define Rte_TypeDef_Envm_FIDMOut_NOP_t
#endif

#ifndef Rte_TypeDef_Envm_FIDMOutPro_t
typedef struct {
    Envm_FIDMOut_ACC_t FIDMOut_ACC;
    Envm_FIDMOut_AEB_t FIDMOut_AEB;
    Envm_FIDMOut_FCW_t FIDMOut_FCW;
    Envm_FIDMOut_RCTA_t FIDMOut_RCTA;
    Envm_FIDMOut_FCTA_t FIDMOut_FCTA;
    Envm_FIDMOut_TSR_t FIDMOut_TSR;
    Envm_FIDMOut_LCA_t FIDMOut_LCA;
    Envm_FIDMOut_BSD_t FIDMOut_BSD;
    Envm_FIDMOut_LDW_t FIDMOut_LDW;
    Envm_FIDMOut_LDP_t FIDMOut_LDP;
    Envm_FIDMOut_DOW_t FIDMOut_DOW;
    Envm_FIDMOut_RCW_t FIDMOut_RCW;
    Envm_FIDMOut_IHBC_t FIDMOut_IHBC;
    Envm_FIDMOut_Pilot_t FIDMOut_Pilot;
    Envm_FIDMOut_NOP_t FIDMOut_NOP;
} Envm_FIDMOutPro_t;
#define Rte_TypeDef_Envm_FIDMOutPro_t
#endif

// #ifndef Rte_TypeDef_proEnvmPrtList_t
typedef struct {
    Envm_t_GenObjectList *pEnvmGenObjectList;
    Envm_t_CRObjectList *pEnvmTechObjectList;
    ECAMtCyclEnvmode_t *pECAMtCyclEnvmode;
    VEDVehDyn_t *pObjSyncEgoDynamic;
    VEDVehDyn_t *pTgtSyncEgoDynamic;
    EnvmDEnvmOut_t *pDem;
    EnvmNvmOut_t *pNvmOut;
    Envm_t_CameraLaneOutputFctData *pCameraLaneData;
    Envm_FusionObjectList_t *pFusionObjectList;
    Envm_TrafficLight_t *pTrafficLight;
    Envm_TrafficSign_t *pTrafficSign;
    Envm_ADASHighlightID_t *pADASHighlightID;
    Envm_FIDMOutPro_t *pFIDMOutPro;
} proEnvmPrtList_t;
// #define Rte_TypeDef_proEnvmPrtList_t
// #endif

// #ifndef baseReqEMPrtList_t_define
// typedef struct {
//     const BSW_s_EnvmCtrlData_t* pCtrl;
//     const AssessedObjList_t* pFctObjectList;
//     const VEDVehDyn_t* pVehicleDynData;
//     const VEDVehPar_t* pVehicleStatData;
//     const VEDVehSig_t* pVehSig;
//     const EnvmNvmIn_t* pNvmIn;
//     const CustEMInData_t* pCustIn;
//     const Envm_t_CameraObjInputData* p_CamObjInput;
//     const BSW_s_AlgoParameters_t* pAlgoParameters;
//     const EMCallBackHdlr_t* pEnvmCallBackHdlr;
// } baseReqEMPrtList_t;
// #define baseReqEMPrtList_t_define
// #endif

// #ifndef baseProEnvmPrtList_t_define
// typedef struct {
//     Envm_t_GenObjectList* pEnvmGenObjectList;
//     Envm_t_CRObjectList* pEnvmTechObjectList;
//     ECAMtCyclEnvmode_t* pECAMtCyclEnvmode;
//     VEDVehDyn_t* pObjSyncEgoDynamic;
//     VEDVehDyn_t* pTgtSyncEgoDynamic;
//     EnvmDEnvmOut_t* pDem;
//     EnvmNvmOut_t* pNvmOut;
//     Envm_t_FusionOutputData* p_FusionStatusOutput;
//     Envm_t_CameraLaneOutputFctData* pCameraLaneData;
// } baseProEnvmPrtList_t;
// #define baseProEnvmPrtList_t_define
// #endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
// __declspec(dllexport) void ENVM_Exec(const reqEMPrtList_t* reqPorts,
// proEnvmPrtList_t* proPorts);
void ENVM_Exec(const reqEMPrtList_t *reqPorts, proEnvmPrtList_t *proPorts);

// __declspec(dllexport) void ENVM_Exec_Wrapper(const baseReqEMPrtList_t*
// reqPorts, baseProEnvmPrtList_t* proPorts, ExtObjectList_t* pRadarInput,
// CamObjectList* pCamerInput, uint32 uTimestamp); extern void
// ENVM_Exec_Wrapper(const baseReqEMPrtList_t* reqPorts, baseProEnvmPrtList_t*
// proPorts,ExtObjectList_t* pRadarInput , CamObjectList* pCamerInput, uint32
// uTimestamp);

#ifdef __cplusplus
}
#endif
#endif
