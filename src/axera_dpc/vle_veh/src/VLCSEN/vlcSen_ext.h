/*
 * Copyright (C) 2022 by SenseAuto Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

#pragma once
#ifndef VLCSEN_EXT_H
#define VLCSEN_EXT_H
#include "TM_Global_TypeDefs.h"
#ifdef __cplusplus
extern "C" {
#endif

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

// typedef boolean Boolean;
// typedef float32 Float32;
// typedef float64 Float64;
// typedef sint16 Sint16;
// typedef sint32 Sint32;
// typedef sint8 Sint8;
// typedef uint16 Uint16;
// typedef uint32 Uint32;
// typedef uint8 Uint8;
// typedef void Void;

// typedef sint8 acc_obj_lane_t;
// typedef uint8 obj_class_t;
// typedef uint8 INHIBITION_REASON_t;
// typedef uint8 acc_situation_class_t;
// typedef uint8 eRelObjLossReason_t;
// typedef uint32 EBACodingParamFmod_t;
// typedef uint32 EBACodingParamGen_t;
// typedef uint8 t_MarkerColor;
// typedef uint8 t_MarkerType;
// typedef uint16 Envm_t_GenEbaInhibit;
// typedef uint8 Envm_t_GenObjDynamicProperty;
// typedef uint8 Envm_t_GenObjMaintenanceState;
// typedef uint16 EM_t_GenObjSensorSource;
// typedef uint8 eLongControlStatus_t;
// typedef uint8 DDR_Aln_System_Setting_t;
// typedef uint8 DDR_Target_Fusion_Status_t;
// typedef uint8 DDR_Target_Recog_Status_t;
// typedef uint8 ucPredictState_t;
// typedef uint8 eTurnIndicator_t;
// typedef uint8 VLCObjUsageState_t;
// typedef uint8 VLC_OP_MODE_t;
// typedef sint16 acceleration_t;
// typedef uint8 confidence_t;
// typedef sint16 distance_t;
// typedef uint8 eGDBPDStates_t;
// typedef sint16 factor_t;
// typedef uint16 times_t;
// typedef uint32 times_long_t;
// typedef sint16 velocity_t;
// typedef uint8 eCDHypothesisType_t;
// typedef uint8 eEBADynProp_t;
// typedef uint8 eEBAObjectClass_t;
// typedef uint16 AlgoCycleCounter_t;
// typedef uint32 AlgoDataTimeStamp_t;
// typedef uint8 AlgoErrorState_t;
// typedef uint32 AlgoInterfaceVersionNumber_t;
// typedef uint8 AlgoSignalState_t;
// typedef sint8 ObjNumber_t;
// // typedef float32 fDistance_t;
// typedef uint8 percentage_t;
// typedef uint8 quality_t;
// typedef uint8 ucConfidence_t;
// typedef uint8 VEDDrvAxle_t;
// typedef uint8 VEDIoStateTypes_t;
// typedef uint8 MotState_t;
// typedef uint8 corrQual_t;
// typedef uint8 Orientation_t;
// typedef uint8 SteeringVariant_t;
// typedef uint8 SYSDampState_t;
// typedef uint8 DC_status_information_t;
// typedef uint8 ldm_ctrl_state_t;
// typedef uint8 door_state_t;
// typedef uint8 SpeedUnitEnum_t;
// typedef uint8 seatbelt_state_t;
// typedef uint8 country_code_t;
// typedef sint32 signed_fuzzy_t;

#ifndef VLC_ACC_OBJ_HIST_NUM
#define VLC_ACC_OBJ_HIST_NUM 10
#endif

#ifndef VLCVEH_EXT_H

#ifndef Rte_TypeDef_SignalHeader_t
typedef struct {
    AlgoDataTimeStamp_t uiTimeStamp;
    AlgoCycleCounter_t uiMeasurementCounter;
    AlgoCycleCounter_t uiCycleCounter;
    AlgoSignalState_t eSigStatus;
} SignalHeader_t;
#define Rte_TypeDef_SignalHeader_t
#endif

#ifndef Rte_TypeDef_TcuSysInfo_t
typedef struct {
    uint32 uiCycleCounter;
    uint32 uiCycleStart;
    uint32 uiCycleEnd;
} TcuSysInfo_t;
#define Rte_TypeDef_TcuSysInfo_t
#endif

#endif

#define SRR_RAM_OBJ_NUM 40
#define Envm_NR_PRIVOBJECTS (40)
#define Acc_max_number_ooi (6u)
// #define TUE_RADAR_RAW_OBJECT_NUM 60  // radar object number get from sensor

#ifndef VLCVEH_EXT_H
// VLCCtrlData_t
#ifndef Rte_TypeDef_VLCCtrlData_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    VLC_OP_MODE_t OpMode;
    float32 fCycleTime;
} VLCCtrlData_t;
#define Rte_TypeDef_VLCCtrlData_t
#endif

#endif

#ifndef VLCVEH_EXT_H

#ifndef Rte_TypeDef_VeloCorrVehDyn_t
// VED_VehDyn_t
typedef struct {
    float corrFact;
    float corrVar;
    float corrVelo;
    float corrVeloVar;
    float minVelo;
    float maxVelo;
    corrQual_t corrQual;
    boolean bRollerTestBench;
} VeloCorrVehDyn_t;
#define Rte_TypeDef_VeloCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_MotVarVehDyn_t
typedef struct {
    float Velocity;
    float Accel;
    float varVelocity;
    float varAccel;
} MotVarVehDyn_t;
#define Rte_TypeDef_MotVarVehDyn_t
#endif

#ifndef Rte_TypeDef_AccelCorrVehDyn_t
typedef struct {
    float corrAccel;
    float corrAccelVar;
} AccelCorrVehDyn_t;
#define Rte_TypeDef_AccelCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_Longitudinal_t
typedef struct {
    MotVarVehDyn_t MotVar;
    VeloCorrVehDyn_t VeloCorr;
    AccelCorrVehDyn_t AccelCorr;
} Longitudinal_t;
#define Rte_TypeDef_Longitudinal_t
#endif

#ifndef Rte_TypeDef_YawRateVehDyn_t
typedef struct {
    float YawRate;
    float Variance;
    float Quality;
} YawRateVehDyn_t;
#define Rte_TypeDef_YawRateVehDyn_t
#endif

#ifndef Rte_TypeDef_CurveVehDyn_t
typedef struct {
    float Curve;
    float C1;
    float Gradient;
    float varC0;
    float varC1;
    float Quality;
    float CrvError;
    unsigned char CrvConf;
} CurveVehDyn_t;
#define Rte_TypeDef_CurveVehDyn_t
#endif

#ifndef Rte_TypeDef_DrvIntCurveVehDyn_t
typedef struct {
    float Curve;
    float Variance;
    float Gradient;
} DrvIntCurveVehDyn_t;
#define Rte_TypeDef_DrvIntCurveVehDyn_t
#endif

#ifndef Rte_TypeDef_LatAccelVehDyn_t
typedef struct {
    float LatAccel;
    float Variance;
} LatAccelVehDyn_t;
#define Rte_TypeDef_LatAccelVehDyn_t
#endif

#ifndef Rte_TypeDef_SideSlipVehDyn_t
typedef struct {
    float SideSlipAngle;
    float Variance;
} SideSlipVehDyn_t;
#define Rte_TypeDef_SideSlipVehDyn_t
#endif

#ifndef Rte_TypeDef_Lateral_t
typedef struct {
    YawRateVehDyn_t YawRate;
    float OffCompStWheelAngle;
    CurveVehDyn_t Curve;
    DrvIntCurveVehDyn_t DrvIntCurve;
    LatAccelVehDyn_t Accel;
    SideSlipVehDyn_t SlipAngle;
} Lateral_t;
#define Rte_TypeDef_Lateral_t
#endif

#ifndef Rte_TypeDef_MotionStateVehDyn_t
typedef struct {
    MotState_t MotState;
    float Confidence;
} MotionStateVehDyn_t;
#define Rte_TypeDef_MotionStateVehDyn_t
#endif

#ifndef Rte_TypeDef_LegacyVehDyn_t
typedef struct {
    float YawRateMaxJitter;
    boolean bStandStill;
} LegacyVehDyn_t;
#define Rte_TypeDef_LegacyVehDyn_t
#endif

// typedef VEDIoStateTypes_t State_array_t[12];

#ifndef Rte_TypeDef_VED_VehDyn_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    Longitudinal_t Longitudinal;
    Lateral_t Lateral;
    MotionStateVehDyn_t MotionState;
    LegacyVehDyn_t Legacy;
    State_array_t State;
} VED_VehDyn_t;
#define Rte_TypeDef_VED_VehDyn_t
#endif

#endif

#ifndef VLCVEH_EXT_H
// VED_VehPar_t

// typedef VEDIoStateTypes_t State_array_t_4[16];
// typedef VEDIoStateTypes_t State_array_t_5[12];
// typedef VEDIoStateTypes_t State_array_t_6[8];
// typedef VEDIoStateTypes_t State_array_t_7[8];

// typedef float ang_array_t[2];
// typedef float rat_array_t[2];
// typedef float rat_array_t_0[2];
// typedef float vel_array_t[2];

#ifndef Rte_TypeDef_swa_t
typedef struct {
    ang_array_t ang;
    rat_array_t rat;
} swa_t;
#define Rte_TypeDef_swa_t
#endif

#ifndef Rte_TypeDef_vel_t
typedef struct {
    vel_array_t vel;
    rat_array_t_0 rat;
} vel_t;
#define Rte_TypeDef_vel_t
#endif

#ifndef Rte_TypeDef_StRatio_t
typedef struct {
    swa_t swa;
    vel_t vel;
} StRatio_t;
#define Rte_TypeDef_StRatio_t
#endif

#ifndef Rte_TypeDef_VehParMain_t
typedef struct {
    State_array_t_4 State;
    float SelfSteerGrad;
    StRatio_t SteeringRatio;
    float WheelBase;
    float TrackWidthFront;
    float TrackWidthRear;
    float VehWeight;
    float CntrOfGravHeight;
    float AxisLoadDistr;
    float WhlLoadDepFrontAxle;
    float WhlLoadDepRearAxle;
    float WhlCircumference;
    VEDDrvAxle_t DrvAxle;
    unsigned char WhlTcksPerRev;
    float FrCrnrStiff;
    float ReCrnrStiff;
} VehParMain_t;
#define Rte_TypeDef_VehParMain_t
#endif

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
    SteeringVariant_t SteeringVariant;
} VehParAdd_t;
#define Rte_TypeDef_VehParAdd_t
#endif

#ifndef Rte_TypeDef_SensorMounting_t
typedef struct {
    State_array_t_6 State;
    float LatPos;
    float LongPos;
    float VertPos;
    float LongPosToCoG;
    float PitchAngle;
    Orientation_t Orientation;
    float RollAngle;
    float YawAngle;
} SensorMounting_t;
#define Rte_TypeDef_SensorMounting_t
#endif

#ifndef Rte_TypeDef_Sensor_t
typedef struct {
    State_array_t_7 State;
    float CoverDamping;
    float fCoverageAngle;
    float fLobeAngle;
    float fCycleTime;
    unsigned char uNoOfScans;
} Sensor_t;
#define Rte_TypeDef_Sensor_t
#endif

#ifndef Rte_TypeDef_VED_VehPar_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    VehParMain_t VehParMain;
    VehParAdd_t VehParAdd;
    SensorMounting_t SensorMounting;
    Sensor_t Sensor;
} VED_VehPar_t;
#define Rte_TypeDef_VED_VehPar_t
#endif

#endif

#ifndef VLCVEH_EXT_H
// VLC_DFV2SenInfo_t

#ifndef Rte_TypeDef_VLC_DFV2SenInfo_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    boolean StandStill;
    boolean OverrideAccel;
    velocity_t CurLongCtrlVelocity;
    acceleration_t CurLongCtrlAccel;
    acceleration_t MaxAccelLimit;
    acceleration_t MinAccelLimit;
    percentage_t HeadwaySetting;
    percentage_t ProbLaneChgLeft;
    percentage_t ProbLaneChgRight;
    boolean AccOn;
    boolean AccNotOff;
    boolean DecelLimOverride;
    boolean CtrlToRelevObj;
    boolean ObjectEffective;
    ObjNumber_t EBAActivationObjID;
    times_t MovingTime;
} VLC_DFV2SenInfo_t;
#define Rte_TypeDef_VLC_DFV2SenInfo_t
#endif

#endif

#ifndef Rte_TypeDef_sLongControlStatus_t
// VLCCustomInput_t
typedef struct {
    eLongControlStatus_t eLongControlStatus;
} sLongControlStatus_t;
#define Rte_TypeDef_sLongControlStatus_t
#endif

#ifndef Rte_TypeDef_VLCCustomInput_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    sLongControlStatus_t sLongControlStatus;
    eTurnIndicator_t eTurnIndicator;
    uint8 speedometer_speed;
    uint8 TJASLC_ManeuverState_nu;
    uint8 TJASLC_LaneChangeTrig_nu;
} VLCCustomInput_t;
#define Rte_TypeDef_VLCCustomInput_t
#endif

#ifndef VLCVEH_EXT_H

#ifndef Rte_TypeDef_CPAR_EBA_Parameters_t
// VLC_Parameters_t
typedef struct {
    EBACodingParamGen_t EBACodingParamGen;
    EBACodingParamFmod_t EBACodingParamFmod;
    boolean EBACodingParamValid;
} CPAR_EBA_Parameters_t;
#define Rte_TypeDef_CPAR_EBA_Parameters_t
#endif

#ifndef Rte_TypeDef_VLC_Parameters_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    CPAR_EBA_Parameters_t EBA;
} VLC_Parameters_t;
#define Rte_TypeDef_VLC_Parameters_t
#endif

#endif
// t_CamLaneInputData

#ifndef Rte_TypeDef_t_CourseInfoSeg
typedef struct {
    float f_C0;
    float f_C1;
    float f_Length;
} t_CourseInfoSeg;
#define Rte_TypeDef_t_CourseInfoSeg
#endif

#ifndef Rte_TypeDef_t_CourseInfo
typedef struct {
    float f_Angle;
    t_CourseInfoSeg CourseInfoSegNear;
} t_CourseInfo;
#define Rte_TypeDef_t_CourseInfo
#endif

#ifndef Rte_TypeDef_t_LaneMarkerInfo
typedef struct {
    float f_MarkerDist;
    t_MarkerColor MarkerColor;
    t_MarkerType MarkerType;
    uint8 u_ExistanceProbability;
} t_LaneMarkerInfo;
#define Rte_TypeDef_t_LaneMarkerInfo
#endif

#ifndef Rte_TypeDef_t_ConstructionSite
typedef struct {
    boolean b_MultipleMarker;
    boolean b_LeftBarrier;
    boolean b_RightBarrier;
    boolean b_CrossingMarker;
    boolean b_InhibitSingleLane;
    boolean b_Hold;
} t_ConstructionSite;
#define Rte_TypeDef_t_ConstructionSite
#endif

#ifndef Rte_TypeDef_t_CamLaneInputData
typedef t_LaneMarkerInfo LaneMarkerInfo_array_t[4];
typedef t_CourseInfo CourseInfo_array_t[4];
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    CourseInfo_array_t CourseInfo;
    LaneMarkerInfo_array_t LaneMarkerInfo;
    t_ConstructionSite ConstructionSite;
} t_CamLaneInputData;
#define Rte_TypeDef_t_CamLaneInputData
#endif

#ifndef VLCVEH_EXT_H
// VLC_acc_object_t

#ifndef Rte_TypeDef_obj_status_t
typedef struct {
    boolean MEASURED;
    boolean TRACKED;
    boolean NEW;
    boolean STANDING;
    boolean STOPPED;
    boolean MOVING;
    boolean DETECTED;
} obj_status_t;
#define Rte_TypeDef_obj_status_t
#endif

#ifndef Rte_TypeDef_object_t
typedef struct {
    velocity_t REL_LONG_SPEED;
    velocity_t REL_LAT_SPEED;
    distance_t LONG_DISPLACEMENT;
    distance_t LAT_DISPLACEMENT;
    acceleration_t REL_LONG_ACCEL;
    acceleration_t REL_LAT_ACCEL;
    confidence_t QUALITY;
    obj_class_t CLASS;
    obj_status_t OBJECT_STATUS;
    distance_t WIDTH;
    ObjNumber_t OBJECT_ID;
} object_t;
#define Rte_TypeDef_object_t
#endif

#ifndef Rte_TypeDef_acc_object_usage_status_t
typedef struct {
    boolean INTEREST;
    boolean USE_FOR_CONTROL;
    boolean USE_FOR_ALERT;
    boolean LOST_REASON;
} acc_object_usage_status_t;
#define Rte_TypeDef_acc_object_usage_status_t
#endif

#ifndef Rte_TypeDef_Fuzzy_Signal_Input_t
typedef struct {
    signed_fuzzy_t fuzzy_rel_distance;
    signed_fuzzy_t fuzzy_a_obj;
    signed_fuzzy_t fuzzy_distance_set_error;
    signed_fuzzy_t fuzzy_distance_min_error;
    signed_fuzzy_t fuzzy_softness;
    signed_fuzzy_t fuzzy_v_own;
    signed_fuzzy_t fuzzy_v_obj;
    signed_fuzzy_t fuzzy_distance;
    signed_fuzzy_t fuzzy_speed_rel;
    signed_fuzzy_t fuzzy_time_gap;
} Fuzzy_Signal_Input_t;
#define Rte_TypeDef_Fuzzy_Signal_Input_t
#endif

#ifndef Rte_TypeDef_Fuzzy_Rule_Input_t
typedef struct {
    signed_fuzzy_t fv_rel_distance_close;
    signed_fuzzy_t fv_speed_rel_slower;
    signed_fuzzy_t fv_speed_rel_faster;
    signed_fuzzy_t fv_v_own_stillstand;
    signed_fuzzy_t fv_a_obj_braking;
    signed_fuzzy_t fv_speed_rel_same_speed;
    signed_fuzzy_t fv_a_obj_lowaccel;
    signed_fuzzy_t fv_distance_min_error_to_far;
    signed_fuzzy_t fv_v_own_urban;
    signed_fuzzy_t fv_speed_rel_much_slower;
    signed_fuzzy_t fv_distance_set_error_near;
    signed_fuzzy_t fv_softness_soft;
    signed_fuzzy_t fv_speed_rel_littleslower;
    signed_fuzzy_t fv_a_obj_accelerating;
    signed_fuzzy_t fv_a_obj_rolling;
    signed_fuzzy_t fv_v_obj_standstill;
    signed_fuzzy_t fv_distance_min_error_veryshort;
    signed_fuzzy_t fv_rel_distance_still_OK;
    signed_fuzzy_t fv_distance_min_error_short;
    signed_fuzzy_t fv_v_own_fast;
    signed_fuzzy_t fv_speed_rel_littlefaster;
    signed_fuzzy_t fv_distance_min_error_below;
    signed_fuzzy_t fv_v_own_slow;
    signed_fuzzy_t fv_rel_distance_far;
    signed_fuzzy_t fv_rel_distance_OK;
    signed_fuzzy_t fv_a_obj_hard_braking;
    signed_fuzzy_t fv_distance_min_error_near;
    signed_fuzzy_t fv_distance_set_error_close;
    signed_fuzzy_t fv_distance_set_error_not_close_enough;
    signed_fuzzy_t fv_softness_very_dynamic;
    signed_fuzzy_t fv_distance_to_close;
    signed_fuzzy_t fv_v_obj_fast;
    signed_fuzzy_t fv_distance_set_error_to_close;
    signed_fuzzy_t fv_a_obj_soft_braking;
    signed_fuzzy_t fv_distance_set_error_to_far;
    signed_fuzzy_t fv_v_obj_urban;
    signed_fuzzy_t fv_speed_rel_really_slower;
    signed_fuzzy_t fv_distance_set_error_more_than_requested;
    signed_fuzzy_t fv_a_obj_mid_braking;
    signed_fuzzy_t fv_speed_rel_close_to_faster;
    signed_fuzzy_t fv_rel_distance_close_to_far;
    signed_fuzzy_t fv_speed_rel_middleslower;
    signed_fuzzy_t fv_a_obj_fast_braking;
    signed_fuzzy_t fv_v_own_low_speed;
    signed_fuzzy_t fv_v_own_mid;
    signed_fuzzy_t fv_v_own_full_range;
    signed_fuzzy_t fv_time_gap_small;
    signed_fuzzy_t fv_time_gap_mid;
    signed_fuzzy_t fv_time_gap_large;
} Fuzzy_Rule_Input_t;
#define Rte_TypeDef_Fuzzy_Rule_Input_t
#endif

#ifndef Rte_TypeDef_VLC_acc_object_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    object_t AUTOSAR;
    velocity_t LONG_SPEED;
    velocity_t LONG_SPEED_HISTORY[VLC_ACC_OBJ_HIST_NUM];
    acceleration_t LONG_ACCEL;
    distance_t LAT_DISPL_FROM_LANE;
    acc_obj_lane_t LANE_INFORMATION;
    acceleration_t MAX_ALLOWED_DECEL;
    acceleration_t MAX_ALLOWED_ACCEL;
    acceleration_t CONTROL_ACCEL;
    acceleration_t NEEDED_DECEL;
    times_t TTC;
    quality_t AVLC_CUT_IN_OUT_POTENTIAL;
    acc_object_usage_status_t USAGE_STATUS;
    distance_t REQUESTED_DISTANCE_MODIFIED_ACT;
    distance_t REQUESTED_DISTANCE_MODIFIED_PRED;
    percentage_t CONTROL_SMOOTHNESS;
    factor_t ALERT_MODIFICATION_FACTOR;
    ObjNumber_t LAST_OBJECT_ID;
    acceleration_t LONG_ACCEL_MODIFIED;
    times_t TTS;
    acceleration_t ACCEL_REQUEST_FUZZY;
    acceleration_t ACCEL_REQUEST_TTS;
    acceleration_t ACCEL_REQUEST_DMIN;
    signed_fuzzy_t FuzzyAreaArray[47];
    signed_fuzzy_t FuzzyAreaPosArray[47];
    signed_fuzzy_t FuzzyMidArray[47];
    signed_fuzzy_t FuzzyValArray[47];
    Fuzzy_Signal_Input_t Fuzzy_Signal_Input;
    Fuzzy_Rule_Input_t Fuzzy_Rule_Input;
} VLC_acc_object_t;
#define Rte_TypeDef_VLC_acc_object_t
#endif

#endif

#ifndef VLCVEH_EXT_H
// VLC_acc_output_data_t

#ifndef Rte_TypeDef_acc_output_status_t
typedef struct {
    boolean ALERT;
    boolean INHIBITED;
    INHIBITION_REASON_t INHIBITION_REASON;
    boolean ALLOW_INIT;
} acc_output_status_t;
#define Rte_TypeDef_acc_output_status_t
#endif

#ifndef Rte_TypeDef_acc_situation_classifier_t
typedef struct {
    acc_situation_class_t SITUATION;
    confidence_t CRITICALITY;
} acc_situation_classifier_t;
#define Rte_TypeDef_acc_situation_classifier_t
#endif

#ifndef Rte_TypeDef_VLC_acc_output_data_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    acceleration_t DISTANCE_CTRL_ACCEL_MAX;
    acceleration_t DISTANCE_CTRL_ACCEL_MIN;
    acceleration_t MAX_AVLC_ACCELERATION;
    acceleration_t MAX_AVLC_DECELERATION;
    times_t REQUESTED_TIMEGAP;
    distance_t REQUESTED_DISTANCE;
    distance_t REQUESTED_MAX_INTRUSION;
    velocity_t RECOMMENDED_VELOCITY;
    percentage_t HEADWAY_SETTING;
    acc_output_status_t AVLC_OUTPUT_STATUS;
    acc_situation_classifier_t SITUATION_CLASS;
} VLC_acc_output_data_t;
#define Rte_TypeDef_VLC_acc_output_data_t
#endif

#endif
// VLCCustomOutput_t

#ifndef Rte_TypeDef_SICustSelectInfo_t
typedef struct {
    float fLongControlDistance;
    float fSiAccPreselDist;
    float fVisibilityRestriction;
    float fRangeFactor;
    float fSeekLaneWidth;
} SICustSelectInfo_t;
#define Rte_TypeDef_SICustSelectInfo_t
#endif

#ifndef Rte_TypeDef_CPCustOutPredTraj_t
typedef struct {
    float fCurveC0;
    float fCurveC1;
    float fAngle;
    ucPredictState_t ucPredictState;
} CPCustOutPredTraj_t;
#define Rte_TypeDef_CPCustOutPredTraj_t
#endif

#ifndef Rte_TypeDef_VLCBlockageDebugInfo_t
typedef struct {
    SYSDampState_t eSPMBlockageState;
    eGDBPDStates_t eSPMSelfTestState;
    unsigned char ucEstiRange;
    ucConfidence_t ucEstiRangeConf;
    unsigned char ucEstiRangeProb;
    unsigned char ucObjLossProb;
    ucConfidence_t ucObjLossConf;
    unsigned char ucOverallProb;
    ucConfidence_t ucOverallConf;
    unsigned short usRoadbeamTimeCnt;
    unsigned short usTimeoutWayCnt;
    unsigned char ucTimeoutTimeCnt;
    unsigned char ucTimeoutBlkProb;
    ucConfidence_t ucTimeoutBlkConf;
    unsigned short usFullBlkTimer;
} VLCBlockageDebugInfo_t;
#define Rte_TypeDef_VLCBlockageDebugInfo_t
#endif

#ifndef Rte_TypeDef_DDR_FFD_OutputDFS_t
typedef struct {
    DDR_Aln_System_Setting_t Sys_setting_elevation;
    DDR_Aln_System_Setting_t Sys_setting_azimuth;
    ObjNumber_t Internal_Object_Number;
    unsigned char DDR_Target_Object_Number;
    float DDR_Target_ETTC;
    float DDR_Target_TTC;
    float Target_Distance_Value;
    float Target_Lateral_Position;
    float Target_RelVelX;
    float Target_RelVelY;
    float Target_RelAccelX;
    DDR_Target_Fusion_Status_t DDR_Target_Fusion_Status;
    DDR_Target_Recog_Status_t DDR_Target_Recog_Status;
    boolean Target_switch_flag_for_veh;
    boolean Target_switch_flag_for_ped;
    boolean Crossing_Pedestrian_flag;
    boolean First_detection_flag;
    unsigned char Prob_obj_exist_in_ego_lane;
    unsigned char Overlap_ratio;
    boolean Extrapolation_flag;
    boolean Normal_threshold_flag;
    boolean Low_threshold_flag;
    boolean Stationary_vehicle_flag;
    boolean Metal_on_road_flag;
    boolean Overhead_structure_flag;
    boolean ARS_Update_flag;
    boolean Camera_Update_flag;
} DDR_FFD_OutputDFS_t;
#define Rte_TypeDef_DDR_FFD_OutputDFS_t
#endif

#ifndef Rte_TypeDef_AbsolutKinematics_t
typedef struct {
    float fAbsVelocityX;
    float fAbsAccelerationX;
} AbsolutKinematics_t;
#define Rte_TypeDef_AbsolutKinematics_t
#endif

#ifndef Rte_TypeDef_VLCCustObjData_t
typedef struct {
    AbsolutKinematics_t AbsolutKinematics;
} VLCCustObjData_t;
#define Rte_TypeDef_VLCCustObjData_t
#endif

typedef VLCCustObjData_t CustObjData_array_t[40];

#ifndef Rte_TypeDef_VLCCustomOutput_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    SICustSelectInfo_t AccSelectionInfo;
    CPCustOutPredTraj_t PredictedTrajectory;
    VLCBlockageDebugInfo_t BlockageInfo;
    DDR_FFD_OutputDFS_t DDR_FFD_Output;
    CustObjData_array_t CustObjData;
} VLCCustomOutput_t;
#define Rte_TypeDef_VLCCustomOutput_t
#endif

#ifndef VLCVEH_EXT_H
// VLCCDOutputCustom_t
typedef float AnecLong_array_t[12];

#ifndef Rte_TypeDef_iBrkOutput_t
typedef struct {
    AnecLong_array_t AnecLong;
    float AllowedDistance;
} iBrkOutput_t;
#define Rte_TypeDef_iBrkOutput_t
#endif

#ifndef Rte_TypeDef_VLCCDOutputCustom_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    iBrkOutput_t iBrkOutput;
    TcuSysInfo_t sTcuSysInfo;
} VLCCDOutputCustom_t;
#define Rte_TypeDef_VLCCDOutputCustom_t
#endif

#endif

#ifndef VLCVEH_EXT_H
// VLC_HypothesisIntf_t

#ifndef Rte_TypeDef_HypoIntfHeader_t
typedef struct {
    unsigned char uiNumOfHypotheses;
} HypoIntfHeader_t;
#define Rte_TypeDef_HypoIntfHeader_t
#endif

#ifndef Rte_TypeDef_HypoIntfDegrSfty_t
typedef struct {
    float fMaxDistALN;
    float fMaxDistHRZ;
    float fMaxDistVIS;
    float fMaxDist;
} HypoIntfDegrSfty_t;
#define Rte_TypeDef_HypoIntfDegrSfty_t
#endif

#ifndef Rte_TypeDef_HypoIntfDegrPerf_t
typedef struct {
    float fMaxDistALN;
    float fMaxDistHRZ;
    float fMaxDistVIS;
    float fMaxDist;
} HypoIntfDegrPerf_t;
#define Rte_TypeDef_HypoIntfDegrPerf_t
#endif

#ifndef Rte_TypeDef_HypoIntfDegr_t
typedef struct {
    HypoIntfDegrSfty_t Safety;
    HypoIntfDegrPerf_t Performance;
} HypoIntfDegr_t;
#define Rte_TypeDef_HypoIntfDegr_t
#endif

#ifndef Rte_TypeDef_HypoIntfCustom_t
typedef struct {
    float fTTSEmergency;
    float fTTSEmergencyStd;
} HypoIntfCustom_t;
#define Rte_TypeDef_HypoIntfCustom_t
#endif

#ifndef Rte_TypeDef_Hypothesis_t
typedef struct {
    ObjNumber_t uiObjectRef;
    eCDHypothesisType_t eType;
    unsigned char uiObjectProbability;
    eEBAObjectClass_t eEBAObjectClass;
    eEBADynProp_t eEBADynProp;
    Envm_t_GenEbaInhibit eEBAInhibitionMask;
    unsigned char uiHypothesisProbability;
    float fHypothesisLifetime;
    float fTTC;
    float fTTCStd;
    float fTTC2;
    float fTTC3;
    float fTTC4;
    float fTTBPre;
    float fTTBPreStd;
    float fTTBAcute;
    float fTTBAcuteStd;
    float fTTSPre;
    float fTTSPreStd;
    float fTTSAcute;
    float fTTSAcuteStd;
    float fLongNecAccel;
    float fLatNecAccel;
    float fDistX;
    float fDistXStd;
    float fVrelX;
    float fVrelXStd;
    float fArelX;
    float fArelXStd;
    float fDistY;
    float fDistYStd;
    float fVrelY;
    float fVrelYStd;
    float fArelY;
    float fArelYStd;
    float fClosingVelocity;
    float fClosingVelocityStd;
    HypoIntfCustom_t Custom;
} Hypothesis_t;
#define Rte_TypeDef_Hypothesis_t
#endif

typedef Hypothesis_t Hypothesis_array_t[6];

#ifndef Rte_TypeDef_VLC_HypothesisIntf_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    HypoIntfHeader_t Header;
    HypoIntfDegr_t Degradation;
    Hypothesis_array_t Hypothesis;
    TcuSysInfo_t sTcuSysInfo;
} VLC_HypothesisIntf_t;
#define Rte_TypeDef_VLC_HypothesisIntf_t
#endif

#endif

// DFSErrorOut_t
#ifndef Rte_TypeDef_DFSErrorOut_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    AlgoErrorState_t ObjectNotMeasured;
    AlgoErrorState_t ObjectNotMeasTmp;
    AlgoErrorState_t Blockage;
    AlgoErrorState_t IncreaseBlockage;
    AlgoErrorState_t DecreaseBlockage;
    unsigned char uiNoOfObjLosses;
    AlgoErrorState_t BelowMinPerfDist_AZ;
    AlgoErrorState_t BelowMinPerfDist_VED;
    AlgoErrorState_t BelowMinPerfDist_EL;
    AlgoErrorState_t InterferenceRange;
    AlgoErrorState_t SRDDampingState;
    boolean bObjSelectionActive;
    boolean bCollisionAvoidActive;
    boolean bRequestRoadBeam;
    fDistance_t tSafetyDistanceEBA;
} DFSErrorOut_t;
#define Rte_TypeDef_DFSErrorOut_t
#endif

#ifndef Rte_TypeDef_AccOOIGenKinematics_t
typedef struct {
    float fDistX;
    float fDistY;
    float fVrelX;
    float fVrelY;
    float fArelX;
    float fArelY;
    float fVabsX;
    float fVabsY;
    float fAabsX;
    float fAabsY;
} AccOOIGenKinematics_t;
#define Rte_TypeDef_AccOOIGenKinematics_t
#endif

#ifndef Rte_TypeDef_AccOOIGenAttributes_t
typedef struct {
    unsigned char uiCutInOutProbability;
    Envm_t_GenObjDynamicProperty eDynamicProperty;
    float fLifeTime;
    AlgoCycleCounter_t uiLifeCycles;
    Envm_t_GenObjMaintenanceState eMaintenanceState;
    ObjNumber_t uiObjectID;
    VLCObjUsageState_t eUsageState;
} AccOOIGenAttributes_t;
#define Rte_TypeDef_AccOOIGenAttributes_t
#endif

#ifndef Rte_TypeDef_AccOOINextLong_t
typedef struct {
    AccOOIGenKinematics_t Kinematic;
    AccOOIGenAttributes_t Attributes;
    eRelObjLossReason_t eRelObjLossReason;
} AccOOINextLong_t;
#define Rte_TypeDef_AccOOINextLong_t
#endif

#ifndef Rte_TypeDef_AccOOI_t
typedef struct {
    AccOOIGenKinematics_t Kinematic;
    AccOOIGenAttributes_t Attributes;
} AccOOI_t;
#define Rte_TypeDef_AccOOI_t
#endif

#ifndef Rte_TypeDef_VLCSenAccOOI_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    AccOOINextLong_t AccOOINextLong;
    AccOOI_t AccOOIHiddenNextLong;
    AccOOI_t AccOOINextLeft;
    AccOOI_t AccOOINextRight;
    AccOOI_t AccOOIHiddenNextLeft;
    AccOOI_t AccOOIHiddenNextRight;
} VLCSenAccOOI_t;
#define Rte_TypeDef_VLCSenAccOOI_t
#endif

// ECAMtCyclEnvmode_t======

#ifndef Rte_TypeDef_ECAMtCyclEnvmode_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    uint8 eCycleMode;
    float32 fECAMtCycleTime;
} ECAMtCyclEnvmode_t;
#define Rte_TypeDef_ECAMtCyclEnvmode_t
#endif

// Envm_t_CRObjectList======
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
    SignalHeader_t sSigHeader;
    Envm_t_CRObjectArray aObject;
} Envm_t_CRObjectList;
#define Rte_TypeDef_Envm_t_CRObjectList
#endif

// Envm_t_GenObjectList======

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
    uint8 ObjectId;
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

// typedef ExtObjects_t Ext_Objects_array_t[TUE_RADAR_RAW_OBJECT_NUM];

// #ifndef ExtObjectList_t_define
// typedef struct {
//     uint32 uiVersionNumber;
//     SignalHeader_t sSigHeader;
//     uint32 uiTimeStamp;
//     uint16 uiCycleCounter;
//     uint8 eSigStatus;
//     HeaderObjList_t HeaderObjList;
//     Ext_Objects_array_t Objects;
// } ExtObjectList_t;
// #define ExtObjectList_t_define
// #endif

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
    SignalHeader_t sSigHeader;
    EM_t_GenObjListHeader HeaderObjList;
    Envm_t_GenObjArray aObject;
} Envm_t_GenObjectList;
#define Rte_TypeDef_Envm_t_GenObjectList
#endif

#ifndef VLCVEH_EXT_H
// Com_AlgoParameters_t
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

#ifndef Rte_TypeDef_Com_AlgoParameters_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    Fct_algo_parameters_t Fct;
} Com_AlgoParameters_t;
#define Rte_TypeDef_Com_AlgoParameters_t
#endif

#endif

// AssessedObjList_t
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

#ifndef Rte_TypeDef_VLCPubObject_t
typedef struct {
    LaneInformation_t LaneInformation;
    ObjOfInterest_t ObjOfInterest;
    LegacyAOL_t Legacy;
} VLCPubObject_t;
#define Rte_TypeDef_VLCPubObject_t
#endif

#ifndef ObjList_array_t_define
typedef VLCPubObject_t ObjList_array_t[40];
#define ObjList_array_t_define
#endif

#ifndef Rte_TypeDef_AssessedObjList_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    HeaderAssessedObjList_t HeaderAssessedObjList;
    ObjList_array_t ObjList;
} AssessedObjList_t;
#define Rte_TypeDef_AssessedObjList_t
#endif

typedef struct {
    float32 VLCSEN_Kf_SteeringRatio;     // Overall steering ratio
    float32 VLCSEN_Kf_WheelBase;         // Distance between front and rear axle
    float32 VLCSEN_Kf_TrackWidthFront;   // Track width front axle
    float32 VLCSEN_Kf_TrackWidthRear;    // Track width rear axle
    float32 VLCSEN_Kf_VehWidth;          // Vehicle width
    float32 VLCSEN_Kf_VehLength;         // Vehicle length
    float32 VLCSEN_Kf_CntrOfGravHeight;  // Center of gravity height
    float32
        VLCSEN_Kf_AxisLoadDistr;  // Ratio between rear axle load and total load
    float32 VLCSEN_Kf_FrCrnrStiff;  // Front axle cornering stiffness
    float32 VLCSEN_Kf_ReCrnrStiff;  // Rear axle cornering stiffness
    float32 VLCSEN_Kf_OverhangFront;

    float32 VLCSEN_Kf_SensorLatPos;   // Lateral displacment related to vehicle
                                      // reference point(+left)
    float32 VLCSEN_Kf_SensorLongPos;  // Longitudinal displacment related to
                                      // vehicle reference point(+fwd)
    float32
        VLCSEN_Kf_SensorVertPos;  // Vertical displacemnt (height) over ground
    float32 VLCSEN_Kf_SensorLongPosToCoG;  // Longitudinal displacment related
                                           // to vehicle center of gravity(+fwd)
    float32 VLCSEN_Kf_SensorPitchAngle;    // Angular displacment about sensor
                                           // lateral axis
    uint8 VLCSEN_Ku_SensorOrientation;
    float32 VLCSEN_Kf_SensorRollAngle;  // Angular displacement about sensor
                                        // longitudinal axis
    float32 VLCSEN_Kf_SensorYawAngle;   // Angular displacemnt about vehicle
                                        // vertical axis

    uint8 VLCSEN_Ku_AEBTriggerSensitivity;  // Sensitivity of decision
                                            // triggering deceleration
    uint8 VLCSEN_Ku_SensitivityReserved1;   // Sensitivity Reserved
    uint8 VLCSEN_Ku_SensitivityReserved2;   // Sensitivity Reserved
    uint8 VLCSEN_Ku_SensitivityReserved3;   // Sensitivity Reserved
    uint8 VLCSEN_Ku_SensitivityReserved4;   // Sensitivity Reserved
    uint8 VLCSEN_Ku_SensitivityReserved5;   // Sensitivity Reserved
    float32 VLCSEN_Kf_Reserved1;            // Reserved
    float32 VLCSEN_Kf_Reserved2;            // Reserved
    float32 VLCSEN_Kf_Reserved3;            // Reserved
    float32 VLCSEN_Kf_Reserved4;            // Reserved
    float32 VLCSEN_Kf_Reserved5;            // Reserved
    uint8 VLCSEN_Ku_Reserved1;              // Reserved
    uint8 VLCSEN_Ku_Reserved2;              // Reserved
    uint8 VLCSEN_Ku_Reserved3;              // Reserved
    uint8 VLCSEN_Ku_Reserved4;              // Reserved
    uint8 VLCSEN_Ku_Reserved5;              // Reserved
    boolean VLCSEN_Kb_Reserved1;            // Reserved
    boolean VLCSEN_Kb_Reserved2;            // Reserved
    boolean VLCSEN_Kb_Reserved3;            // Reserved
    boolean VLCSEN_Kb_Reserved4;            // Reserved
    boolean VLCSEN_Kb_Reserved5;            // Reserved
} VLCSen_Parameters_t;  // vlc sen function calibration parameters

#ifndef Rte_TypeDef_VLCSen_NVRAMData_t
typedef struct {
    float32 VLCSEN_Nf_Reserved1;  // Reserved
    float32 VLCSEN_Nf_Reserved2;  // Reserved
    float32 VLCSEN_Nf_Reserved3;  // Reserved
    float32 VLCSEN_Nf_Reserved4;  // Reserved
    float32 VLCSEN_Nf_Reserved5;  // Reserved
    uint8 VLCSEN_Nu_Reserved1;    // Reserved
    uint8 VLCSEN_Nu_Reserved2;    // Reserved
    uint8 VLCSEN_Nu_Reserved3;    // Reserved
    uint8 VLCSEN_Nu_Reserved4;    // Reserved
    uint8 VLCSEN_Nu_Reserved5;    // Reserved
    boolean VLCSEN_Nb_Reserved1;  // Reserved
    boolean VLCSEN_Nb_Reserved2;  // Reserved
    boolean VLCSEN_Nb_Reserved3;  // Reserved
    boolean VLCSEN_Nb_Reserved4;  // Reserved
    boolean VLCSEN_Nb_Reserved5;  // Reserved
} VLCSen_NVRAMData_t;
#define Rte_TypeDef_VLCSen_NVRAMData_t
#endif

#ifndef Rte_TypeDef_KinCtrlDynInput_t
typedef struct {
    signed short longi_initialization_accel;
    float steer_angle;
    float steer_speed;
    float predict_curvature;
    float sensor_long_accel;
    boolean stand_still_detected;
    DC_status_information_t DC_status_information;
    boolean driver_override_accel_pedal;
    boolean driver_override_decel_pedal;
    boolean driver_braking;
    boolean DAS_accel_request_limited;
    boolean DAS_decel_request_limited;
    boolean longi_shutoff_acknowledged;
    boolean acc_enable;
    boolean acc_inhibit;
    boolean acc_reset;
    boolean park_brk_eng;
    boolean brk_sw;
    boolean door_state_fr;
    boolean door_state_fl;
    boolean door_state_rr;
    boolean door_state_rl;
    boolean hood_state;
    boolean trunk_state;
    seatbelt_state_t seatbelt_state;
    country_code_t country_code;
} KinCtrlDynInput_t;
#define Rte_TypeDef_KinCtrlDynInput_t
#endif

#ifndef Rte_TypeDef_KinDisplayInput_t
typedef struct {
    SpeedUnitEnum_t speed_unit;
    unsigned short speedometer_speed;
} KinDisplayInput_t;
#define Rte_TypeDef_KinDisplayInput_t
#endif

#ifndef Rte_TypeDef_TSRInfo
typedef struct {
    uint8 tsr_speed_limit;
    uint8 tsr_flag;  // 0:No flag detected, 1:Speed limit flag detected, 2:Speed
                     // limit release flag detected
    boolean tsr_active;
} TSRInfo;
#define Rte_TypeDef_TSRInfo
#endif

#ifndef Rte_TypeDef_NavInfo
typedef struct {
    uint8 nav_speed_limit;
    boolean nav_active;
} NavInfo;
#define Rte_TypeDef_NavInfo
#endif

#ifndef Rte_TypeDef_LongCtrlInputCustom_In_t
typedef struct {
    boolean AVLC_ReversibleFail;
    boolean AVLC_IrreversibleFail;
    boolean Camera_Availability;
    boolean ESC_Active;  // add for Project EP40, He Qiushu 20211109
    boolean ABS_Active;
    boolean AEB_Active;
    boolean APA_Active;
    boolean AYC_Active;
    boolean TCS_Active;
    boolean EPB_Active;
    boolean HDC_Active;  // add for Project EP40, He Qiushu 20211109
    boolean USR_Active;  // Ultra sonic radar, add for Project EP40, He Qiushu
                         // 20211109
    boolean NNP_Active;  // NETA Navigation Pilot active flag
    boolean ISA_Active;  // Intelligent Speed Assit active flag
    boolean Door_Open;
    TSRInfo tsr_info;  // Information from Traffic Sign Recognition
    NavInfo nav_info;  // Information from navigation
} LongCtrlInputCustom_In_t;
#define Rte_TypeDef_LongCtrlInputCustom_In_t
#endif

#ifndef Rte_TypeDef_LongCtrlInputCustom_t
typedef struct {
    LongCtrlInputCustom_In_t LongCtrlInputCustom;
} LongCtrlInputCustom_t;
#define Rte_TypeDef_LongCtrlInputCustom_t
#endif

#ifndef Rte_TypeDef_VLC_LongCtrlInput_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    KinCtrlDynInput_t KinCtrlDynInput;
    KinDisplayInput_t DisplayOutput;
    LongCtrlInputCustom_t Custom;
} VLC_LongCtrlInput_t;
#define Rte_TypeDef_VLC_LongCtrlInput_t
#endif

#ifndef Rte_TypeDef_OvertakeAssistInfo
typedef struct {
    boolean overtake_assist_flag;
    boolean overtake_assist_flag_last_cycle;
    float32 target_time_gap;
    float32 release_flag;
    sint16 headway_setting;
} OvertakeAssistInfo;
#define Rte_TypeDef_OvertakeAssistInfo
#endif

// Inputs required by VLC component for sensor cycle processing
typedef struct {
    const VLCCtrlData_t* pSenCtrlData;  //!< VLC control data
    const VED_VehDyn_t*
        pEgoDynObjSync;  //!< The dynamic vehicle ego data object sync
    const VED_VehDyn_t* pEgoDynRaw;      //!< The dynamic vehicle ego data raw
    const VED_VehPar_t* pEgoStaticData;  //!< the static vehicle ego data
    const ECAMtCyclEnvmode_t* pECAMtCyclEnvmode;  //!< The global sensor state
    const Envm_t_GenObjectList* pEmGenObjList;    //!< EM generic object list
    const Envm_t_CRObjectList*
        pEmARSObjList;  //!< EM ARS-technology-specific object list
    // Longitudinal control input ports
    const VLC_DFV2SenInfo_t*
        pDFVLongOut;  //!< Internal communication struct from VLC_VEH to VLC_SEN
    // customer specific input/output
    const VLCCustomInput_t*
        pVLCCustomInput;  // Custom input algo parameters from BSW
    const Com_AlgoParameters_t*
        pBswAlgoParameters;  //!< Input algo parameters from BSW
    const VLC_Parameters_t* pCPAR_VLC_Parameters;  // VLC Coding Parameters
    const t_CamLaneInputData* pCamLaneData;        // Camera lane input data
    const VLCSen_NVRAMData_t* pVLCSenNvRams;   // vlc sen function nvram data
    const VLC_LongCtrlInput_t* pLongCtrlResp;  // Dynamic controller response
} reqVLCSenPrtList_t;

typedef struct {
    AssessedObjList_t* pPubFctObj;  // The public VLC object data
    VLC_acc_object_t*
        pAccDisplayObj;  // AVLC_DISPLAY_OBJECT output from VLC_SEN
    VLC_acc_output_data_t* pAccOutput;    // AVLC_OUTPUT_DATA from VLC_SEN
                                          // customer specific input/output
    VLCCustomOutput_t* pVLCCustomOutput;  // Custom output custom output for CD
    VLCCDOutputCustom_t* pCollDetOutput;  // Collision detection custom output
                                          // Hypothesis interface
    VLC_HypothesisIntf_t* pVLCCDHypotheses;      // CD hypotheses
    DFSErrorOut_t* pErrorOut;                    // VLC error output
    VLCSenAccOOI_t* pVLCAccOOIData;              // VLC OOI Objects
    VLCSen_NVRAMData_t* pVLCSenNvRams;           // vlc sen function nvram data
    OvertakeAssistInfo* p_overtake_assist_info;  // overtake assist data
} proVLCSenPrtList_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
// __declspec(dllexport) void VLCSen_Exec(const reqVLCSenPrtList_t
// pRequirePorts, const VLCSen_Parameters_t* pVLCParameters, const
// proVLCSenPrtList_t* pProvidePorts);
void VLCSen_Exec(const reqVLCSenPrtList_t* pRequirePorts,
                 const VLCSen_Parameters_t* pVLCParameters,
                 const proVLCSenPrtList_t* pProvidePorts);

#ifdef __cplusplus
}
#endif
#endif
