#pragma once
#ifndef VLCVEH_EXT_H
#define VLCVEH_EXT_H
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
// typedef uint8 e_AlnOpMode_t;
// typedef uint8 u_CycleState_t;
// typedef uint8 e_AlnState_t;
// typedef uint8 e_AlignState_t;
// typedef uint8 e_Direction_t;
// typedef uint8 e_GainReduction_t;
// typedef uint8 e_PowerReduction_t;
// typedef uint8 eAssociatedLane_t;
// typedef sint8 eObjOOI_t;
// typedef uint8 eRelObjLossReason_t;
// typedef uint8 CameraFusionPreselBits_t;
// typedef uint32 EBACodingBitEnum_t;
// typedef uint8 EBAPreBrkAccelParNvState_t;
// typedef uint8 Envm_Ped_ArtDummy_t;
// typedef uint8 Em_cem_algo_mode_t;
// typedef uint8 Envm_cEnvm_algo_sensor_state_t;
// typedef uint8 FnSwitchBits_t;
// typedef uint8 NaviFusionPreselBits_t;
// typedef uint8 ui8_UseFarNearScanForBlck_t;
// typedef uint8 EnvmOpMode_t;
// typedef uint32 EBACodingParamFmod_t;
// typedef uint32 EBACodingParamGen_t;
// typedef uint8 t_CamLaneMarkerEnum;
// typedef uint8 t_MarkerColor;
// typedef uint8 t_MarkerType;
// typedef uint8 Envm_t_CustomerProjectDef;
// typedef uint8 EMB0_t_SensorSource;
// typedef uint8 EMB0_t_Service;
// typedef uint8 EM_t_Cust_CamAvailStatus;
// typedef uint8 Envm_t_Cust_FusionStatus;
// typedef uint8 EM_t_Cust_MatchType;
// typedef uint16 EMT0_t_StatusFusion;
// typedef uint8 Envm_t_CamBrakeLight;
// typedef uint8 Envm_t_CamObjManeuver;
// typedef uint8 EM_t_CamTurnLightsState;
// typedef uint8 EMT0_t_CamDetectionMode;
// typedef uint8 EMT0_t_CamObjAssociatedLane;
// typedef uint8 EMT0_t_CamSensorState;
// typedef uint8 EMT0_t_MCamObjClass;
// typedef uint8 EMT0_t_MCamObjDynProp;
// typedef uint8 EMT0_t_ObjFlashLightStat;
// typedef uint8 ui_Accuracy_t;
// typedef uint8 Envm_t_AmbigState;
// typedef uint8 Envm_t_ClusterListValid;
// typedef uint8 Envm_t_DynamicProperty;
// typedef uint8 Envm_t_InvalidReason;
// typedef uint8 Envm_t_PositionInBeam;
// typedef uint8 Envm_t_CR_AbsMovingState;
// typedef uint8 Envm_t_CR_Classification;
// typedef uint8 Envm_t_CR_DynamicProperty;
// typedef uint8 EM_t_ARS_DynamicSubProperty;
// typedef uint8 Envm_t_CR_MeasuredSources;
// typedef uint8 Envm_t_FOVOverlapFar;
// typedef uint8 Envm_t_ObjRelationsClass;
// typedef uint8 u_RoadSide_t;
// typedef uint8 Envm_t_GenEbaHypCat;
// typedef uint16 Envm_t_GenEbaInhibit;
// typedef uint8 Envm_t_GenEbaObjCat;
// typedef uint8 Envm_t_GenFusionLevel;
// typedef uint8 Envm_t_GenObjClassification;
// typedef uint8 Envm_t_GenObjDynamicProperty;
// typedef uint8 Envm_t_GenObjMaintenanceState;
// typedef uint8 Envm_t_GenObjOcclusionState;
// typedef uint16 EM_t_GenObjSensorSource;
// typedef uint8 Envm_t_GenObjShapePointState;
// typedef uint8 EM_t_GenObjSplitMergeState;
// typedef uint8 eCycleMode_t;
// typedef uint8 eLongControlStatus_t;
// typedef uint8 DDR_Aln_System_Setting_t;
// typedef uint8 DDR_Target_Fusion_Status_t;
// typedef uint8 DDR_Target_Recog_Status_t;
// typedef uint8 ucPredictState_t;
// typedef uint8 DIMInputSignalState_t;
// typedef uint8 eDriverSetting_t;
// typedef uint8 eTurnIndicator_t;
// typedef uint8 DIMOutMonState_t;
// typedef uint8 DriverAction_t;
// typedef uint8 eDriverActivityState_t;
// typedef uint8 eDriverAttentionState_t;
// typedef uint8 eDriverFeedbackState_t;
// typedef uint8 eBuzzerState_t;
// typedef uint8 ucCameraObjectStatus_t;
// typedef uint8 ucCameraStatus_t;
// typedef uint32 eFunctionSwitch_t;
// typedef uint8 eMainSwitch_t;
// typedef uint32 eObjectSwitch_t;
// typedef uint8 eEBAFctChan_t;
// typedef uint8 eEBASignalState_t;
// typedef uint8 eFunctionQualifier_t;
// typedef uint8 eGeneratorControl_t;
// typedef uint8 eThreatLvl_t;
// typedef uint8 eWarnSens_t;
// typedef uint8 VLCObjUsageState_t;
// typedef uint8 EngRun_Stat_t;
// typedef uint8 VLCTransmissionGear_t;
// typedef uint8 DFVPTSignalState_t;
// typedef uint8 PwrFreeD_Stat_TCM_t;
// typedef uint8 DFVInspMode_t;
// typedef uint8 PCS_ALM_Disp_Req_Status_t;
// typedef uint8 PCS_DW_Disp_Req_Status_t;
// typedef uint8 PCS_Guard_Status_t;
// typedef uint8 PCS_PedW_Disp_Req_Status_t;
// typedef uint8 eBuzzerPattern_t;
// typedef uint8 eVLCVehLongReqSource_t;
// typedef uint8 VLCStateSig_t;
// typedef uint8 VLC_OP_MODE_t;
// typedef uint8 SpeedUnitEnum_t;
// typedef sint16 acceleration_t;
// typedef uint8 confidence_t;
// typedef sint16 distance_t;
// typedef uint8 eEBAObjQuality_t;
// typedef uint8 eGDBPDStates_t;
// typedef uint16 eTraceBracketBitMasks_t;
// typedef sint16 factor_t;
// typedef uint16 t_u_DistanceLong;
// typedef uint16 times_t;
// typedef uint32 times_long_t;
// typedef sint16 velocity_t;
// typedef uint8 eCDHypothesisType_t;
// typedef uint8 eEBADynProp_t;
// typedef uint8 eEBAObjectClass_t;
// typedef uint32 InterfaceVersion;
// typedef uint8 DC_status_information_t;
// typedef uint8 country_code_t;
// typedef uint8 door_state_t;
// typedef uint8 ldm_ctrl_state_t;
// typedef uint8 seatbelt_state_t;
// typedef uint8 ACCHeadwayLeverSetting_t;
// typedef uint8 ACCInternalState_t;
// typedef uint8 DAS_failure_information_t;
// typedef uint8 DAS_status_t;
// typedef uint8 DM_alert_level_t;
// typedef uint8 DM_status_t;
// typedef uint8 Ext_AccInhibitionRq_Out_t;
// typedef uint8 FCA_status_t;
// typedef uint8 cc_reported_error_t;
// typedef uint8 display_op_status_t;
// typedef uint8 ldm_drive_mode_t;
// typedef uint32 DLG_VirtAddrs;
// typedef uint32 Quality_t;
// typedef uint8 eTraceType_t;
// typedef uint8 eTraceUpdateState_t;
// typedef uint8 u_OpMode_t;
// typedef uint8 u_GainRxChipNear_t;
// typedef uint8 RSP_t_OpMode;
// typedef sint8 eLaneWidthClass_t;
// typedef uint8 eLaneWidthSource_t;
// typedef uint8 eRoadTypeClass_t;
// typedef uint8 eRoadWorksClass_t;
// typedef uint8 eTrafficOrientation_t;
// typedef uint8 e_BorderType_t;
// typedef uint32 AlgoApplicationNumber_t;
// typedef uint32 AlgoComponentVersionNumber_t;
// typedef uint16 AlgoCycleCounter_t;
// typedef uint32 AlgoDataTimeStamp_t;
// typedef uint8 AlgoErrorState_t;
// typedef uint32 AlgoInterfaceVersionNumber_t;
// typedef uint8 AlgoSignalState_t;
// typedef uint32 BaseReturnCode_t;
// typedef uint8 FusiErrorExtDirection_t;
// typedef uint8 FusiErrorExtType_t;
// typedef uint32 GenAlgoQualifiers_t;
// typedef sint8 ObjNumber_t;
// typedef uint16 ParameterID_t;
// typedef uint8 SigState_t;
// typedef uint8 TraceID_t;
// typedef float32 fAccelAbs_t;
// // typedef float32 fAccel_t;
// typedef float32 fAngleDeg_t;
// // typedef float32 fAngle_t;
// typedef float32 fConfidence_t;
// typedef float32 fCurve_t;
// // typedef float32 fDistance_t;
// typedef float32 fGradient_t;
// typedef float32 fProbability_t;
// typedef float32 fQuality_t;
// typedef float32 fRadius_t;
// typedef float32 fRatio_t;
// typedef float32 fStiffness_t;
// typedef float32 fTemperature_t;
// // typedef float32 fTime_t;
// typedef float32 fUncertainty_t;
// // typedef float32 fVariance_t;
// typedef float32 fVelocityAbs_t;
// // typedef float32 fVelocity_t;
// typedef float32 fWeight_t;
// // typedef float32 fYawRate_t;
// // typedef uint8 percentage_t;
// typedef uint8 quality_t;
// typedef uint8 ucConfidence_t;
// typedef uint16 uiTime_t;
// typedef sint32 SymbolicConstants;
// typedef uint8 LongDirState_t;
// typedef uint8 VEDCaliState_t;
// typedef uint8 VEDCtrlState_t;
// typedef uint8 VEDDrvAxle_t;
// typedef uint8 VEDIoStateTypes_t;
// typedef uint32 VEDIoState_t;
// typedef uint32 VDYNvmState_t;
// typedef uint8 VEDTrailerConnection_t;
// typedef uint8 VehDynStatePos_t;
// typedef uint8 VehParAddStatePos_t;
// typedef uint8 VehParMainStatePos_t;
// typedef uint8 VehParSenMountStatePos_t;
// typedef uint8 VehParSensorStatePos_t;
// typedef uint8 VehSigAddStatePos_t;
// typedef uint8 VehSigBrakeStatePos_t;
// typedef uint8 VehSigMainStatePos_t;
// typedef uint8 VehSigPowertrainStatePos_t;
// typedef uint8 VEDErrState_t;
// typedef uint8 MotState_t;
// typedef uint8 corrQual_t;
// typedef uint8 Orientation_t;
// typedef uint8 SteeringVariant_t;
// typedef uint8 ActGearPos_t;
// typedef uint8 IgnitionSwitch_t;
// typedef uint8 ParkBrakeState_t;
// typedef uint8 SpeedUnit_t;
// typedef uint8 TransmissionGear_t;
// typedef uint8 TurnSignal_t;
// typedef uint8 VehLongMotStateExt_t;
// typedef uint8 WiperStage_t;
// typedef uint8 WiperState_t;
// typedef uint8 WiperWasherFrontState_t;
// typedef uint8 eHeightLevel_t;
// typedef uint8 eSuspensionSystem_t;
// typedef uint8 Percentage_s_t;
// typedef uint8 True_False_s_t;
// typedef uint8 SYSDampState_t;
// typedef uint8 iRSPFlag_t;
// typedef uint16 SYS_t_HWSample;
// typedef uint8 SYS_t_Scans;
// typedef sint16 s16q3_t;
// typedef sint16 s16q4_t;
// typedef sint16 s16q5_t;
// typedef sint16 s16q6_t;
// typedef sint16 s16q7_t;
// typedef sint16 s16q8_t;
// typedef sint16 s16q9_t;
// typedef sint32 s32q18_t;
// typedef sint32 s32q24_t;
// typedef sint32 s32q3_t;
// typedef sint32 s32q4_t;
// typedef sint32 s32q7_t;
// typedef uint16 u16q3_t;
// typedef uint16 u16q4_t;
// typedef uint16 u16q5_t;
// typedef uint16 u16q6_t;
// typedef uint16 u16q7_t;
// typedef uint16 u16q8_t;
// typedef uint16 u16q9_t;
// typedef uint32 u32q18_t;
// typedef uint32 u32q24_t;
// typedef uint32 u32q3_t;
// typedef uint32 u32q4_t;
// typedef uint32 u32q7_t;
// typedef uint8 u8q1_t;
// typedef uint8 u8q2_t;
// typedef uint8 u8q3_t;
// typedef uint8 u8q4_t;
// typedef uint8 u8q5_t;
// typedef uint8 u8q6_t;
// typedef uint8 u8q7_t;
// typedef uint8 u8q8_t;
// typedef uint8 t_MarkerColor;
// typedef uint8 t_MarkerType;
// typedef sint32 signed_fuzzy_t;

#ifndef VLC_ACC_OBJ_HIST_NUM
#define VLC_ACC_OBJ_HIST_NUM 10
#endif

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

#define SRR_RAM_OBJ_NUM 40
#define Envm_NR_PRIVOBJECTS (40)
// #define TUE_RADAR_RAW_OBJECT_NUM 60  // radar object number get from sensor

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

#ifndef Rte_TypeDef_VLCCtrlData_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    VLC_OP_MODE_t OpMode;  // VLC requested operation mode
    float32 fCycleTime;    // The VLC cycle time ,unit:s
} VLCCtrlData_t;
#define Rte_TypeDef_VLCCtrlData_t
#endif

#ifndef Rte_TypeDef_VeloCorrVehDyn_t
typedef struct {
    float corrFact;
    float corrVar;
    float corrVelo;  // Ego velocity ,unit m/s
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
    float corrAccel;  // ego Acceleration ,unit m/s^2
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
    object_t AUTOSAR;       // according to AUTOSAR
    velocity_t LONG_SPEED;  // absolute longitudinal speed of object
    velocity_t LONG_SPEED_HISTORY[VLC_ACC_OBJ_HIST_NUM];
    acceleration_t
        LONG_ACCEL;  // absolute longitudinal acceleration of the object
    distance_t LAT_DISPL_FROM_LANE;  // lateral displacement from predicted path
    acc_obj_lane_t LANE_INFORMATION;   // lane the object is associated to
    acceleration_t MAX_ALLOWED_DECEL;  // max allowed decel on this object
    acceleration_t MAX_ALLOWED_ACCEL;  // max allowed accel on thin object
    acceleration_t CONTROL_ACCEL;      // raw acceleration out of the controller
                                       // (not limited between min/max)
    acceleration_t NEEDED_DECEL;       // deceleration needed to avoid a crash
    times_t TTC;                       // time to collision
    quality_t AVLC_CUT_IN_OUT_POTENTIAL;  // potential of this object to cut in
                                          // or out for ACC function
    acc_object_usage_status_t USAGE_STATUS;
    distance_t REQUESTED_DISTANCE_MODIFIED_ACT;  // current control distance
                                                 // modified by driver intention
                                                 // and cutout potential
    distance_t
        REQUESTED_DISTANCE_MODIFIED_PRED;  // predicted (vown) control distance
                                           // modified by driver intention and
                                           // cutout potential
    percentage_t
        CONTROL_SMOOTHNESS;  // smoothness value for longitudinal control
    factor_t
        ALERT_MODIFICATION_FACTOR;  // factor for modifying the alert distance
    ObjNumber_t LAST_OBJECT_ID;  // object ID from the object of the last cycle
    acceleration_t
        LONG_ACCEL_MODIFIED;  // absolute filtered/modified longitudinal
                              // acceleration of the object
    times_t TTS;  // time to stop for objects stopping within the next seconds
    acceleration_t
        ACCEL_REQUEST_FUZZY;  // acceleration request from fuzzy controller
    acceleration_t
        ACCEL_REQUEST_TTS;  // acceleration request from physics to standstill
    acceleration_t ACCEL_REQUEST_DMIN;  // acceleration request from physics to
                                        // approach to set distance
    signed_fuzzy_t FuzzyAreaArray[47];
    signed_fuzzy_t FuzzyAreaPosArray[47];
    signed_fuzzy_t FuzzyMidArray[47];
    signed_fuzzy_t FuzzyValArray[47];
    Fuzzy_Signal_Input_t Fuzzy_Signal_Input;
    Fuzzy_Rule_Input_t Fuzzy_Rule_Input;
} VLC_acc_object_t;
#define Rte_TypeDef_VLC_acc_object_t
#endif

#ifndef Rte_TypeDef_acc_output_status_t
typedef struct {
    boolean ALERT;
    boolean INHIBITED;                      // ACC inhibit status
    INHIBITION_REASON_t INHIBITION_REASON;  // alignment inhibition reason
    boolean ALLOW_INIT;  // Indicates the possibility, to initialize the control
                         // acceleration to the current ego vehicle acceleration
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
    acceleration_t
        DISTANCE_CTRL_ACCEL_MAX;  // Minimum acceleration for intrusion distance
    acceleration_t
        DISTANCE_CTRL_ACCEL_MIN;  // Maximum acceleration for intrusion distance
    acceleration_t MAX_AVLC_ACCELERATION;  // Maximum allowed ACC acceleration
    acceleration_t MAX_AVLC_DECELERATION;  // Maximum allowed ACC deceleration
    times_t REQUESTED_TIMEGAP;             // max time gap
    distance_t REQUESTED_DISTANCE;         // max distance
    distance_t REQUESTED_MAX_INTRUSION;    // neutral distance
    velocity_t RECOMMENDED_VELOCITY;
    percentage_t HEADWAY_SETTING;  // head way setting
    acc_output_status_t AVLC_OUTPUT_STATUS;
    acc_situation_classifier_t SITUATION_CLASS;  // Criticality of the situation
} VLC_acc_output_data_t;
#define Rte_TypeDef_VLC_acc_output_data_t
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

#ifndef Rte_TypeDef_CPAR_EBA_Parameters_t
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

#ifndef Rte_TypeDef_VLC_DFV2SenInfo_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    boolean StandStill;
    boolean OverrideAccel;  // Driver override by gas pedal
    velocity_t CurLongCtrlVelocity;
    acceleration_t CurLongCtrlAccel;
    acceleration_t MaxAccelLimit;
    acceleration_t MinAccelLimit;
    percentage_t HeadwaySetting;
    percentage_t ProbLaneChgLeft;   // probability of lane change to left lane
    percentage_t ProbLaneChgRight;  // probability of lane change to right lane
    boolean AccOn;                  // ACC is in active state
    boolean AccNotOff;              // ACC is active, engaged or override
    boolean DecelLimOverride;  // deceleration limitation after override active
    boolean CtrlToRelevObj;
    boolean ObjectEffective;  // target object is effective for control
    ObjNumber_t EBAActivationObjID;
    times_t MovingTime;
} VLC_DFV2SenInfo_t;
#define Rte_TypeDef_VLC_DFV2SenInfo_t
#endif

#ifndef Rte_TypeDef_VLC_AccLeverInput_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    boolean SetSwitch;  // The ACC Set button signal = CS1_GearPositionReqSt for
                        // EP40 Heqiushu 20211108
    uint8 DecelSwitch;  // The ACC decelearting button. 0: not pressed, 1:
                        // pressed once, 2:press hold
    uint8 ResumeAccelSwitch;  // The ACC resume button and accelerating button
                              // signal. 0: not pressed, 1: pressed once,
                              // 2:press hold
    boolean Cancel;           // The ACC cancel button signal
    uint8 ACCMode;            // The ACC select mode button signal
    boolean MainSwitch;       // The ACC main switch button signal
    boolean HeadwayInc;       // The ACC headway increase button signal
    boolean HeadwayDec;       // The ACC main decrease button signal
    boolean HeadwaySwitch;    // Headway cycle setting switch button
    uint8 Headway;
    boolean PilotSwitch;
    boolean FCA_Setting;
} VLC_AccLeverInput_t;
#define Rte_TypeDef_VLC_AccLeverInput_t
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
    boolean ESC_Active;
    boolean ABS_Active;
    boolean AEB_Active;
    boolean ARP_Active;
    boolean AYC_Active;
    boolean TCS_Active;
    boolean EPB_Active;
    boolean HDC_Active;
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

#ifndef Rte_TypeDef_VLC_DIMInputGeneric_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    boolean bACCActive;
    eDriverSetting_t eDriverSetting;
    float fSteeringWheelAngle;
    float fSteeringWheelAngleGrad;
    DIMInputSignalState_t eSteeringWheelAngleStat;
    DIMInputSignalState_t eSteeringWheelAngleGradStat;
    float fAccelPedalPos;
    float fAccelPedalGrad;
    DIMInputSignalState_t eAccelPadelStat;
    DIMInputSignalState_t eAccelPadelGradStat;
    eTurnIndicator_t eTurnIndicator;
    VLCStateSig_t eDriverBraking;
} VLC_DIMInputGeneric_t;
#define Rte_TypeDef_VLC_DIMInputGeneric_t
#endif

#ifndef Rte_TypeDef_sDriverInput_t
typedef struct {
    boolean bDriverOverride;
} sDriverInput_t;
#define Rte_TypeDef_sDriverInput_t
#endif

#ifndef Rte_TypeDef_VLC_DIMInputCustom_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    VLCStateSig_t eSpeedLimitActive;
    sDriverInput_t sDriverInput;
} VLC_DIMInputCustom_t;
#define Rte_TypeDef_VLC_DIMInputCustom_t
#endif

#ifndef Rte_TypeDef_VLC_SADInputGeneric_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    eMainSwitch_t eMainSwitch;
    eFunctionSwitch_t eFunctionSwitch;
    eObjectSwitch_t eObjectSwitch;
} VLC_SADInputGeneric_t;
#define Rte_TypeDef_VLC_SADInputGeneric_t
#endif

#ifndef Rte_TypeDef_VLC_SADInputCustom_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    ucCameraStatus_t ucCameraStatus;
    ucCameraObjectStatus_t ucCameraObjectStatus;
    eBuzzerState_t eBuzzerState;
} VLC_SADInputCustom_t;
#define Rte_TypeDef_VLC_SADInputCustom_t
#endif

typedef VEDIoStateTypes_t State_array_t_0[32];

typedef VEDIoStateTypes_t State_array_t_1[28];

typedef VEDIoStateTypes_t State_array_t_2[4];

typedef VEDIoStateTypes_t State_array_t_3[2];

#ifndef Rte_TypeDef_VehSigMain_t
typedef struct {
    State_array_t_0 State;
    float YawRate; /*	rad/s	*/
    float YawRateTemp;
    float StWheelAngle;  /* rad 	*/
    float LatAccel;      /* m/s2 	*/
    float WhlVelFrLeft;  /* m/s 		*/
    float WhlVelFrRight; /* m/s 		*/
    float WhlVelReLeft;  /* m/s 		*/
    float WhlVelReRight; /* m/s 		*/
    float YawRateInt;
    float YawRateIntTemp;
    float LongAccel; /* m/s2 	*/
    float RearWhlAngle;
    float VehVelocityExt;         /* unit : m/s */
    float VehLongAccelExt;        /* m/s2 	*/
    LongDirState_t WhlDirFrLeft;  /* m/s 		*/
    LongDirState_t WhlDirFrRight; /* m/s 		*/
    LongDirState_t WhlDirReLeft;  /* m/s 		*/
    LongDirState_t WhlDirReRight; /* m/s 		*/
    unsigned char WhlTicksDevFrLeft;
    unsigned char WhlTicksDevFrRight;
    unsigned char WhlTicksDevReLeft;
    unsigned char WhlTicksDevReRight;
    ActGearPos_t ActGearPos;
    unsigned short BrakeActLevel;
    ParkBrakeState_t ParkBrakeState;
    LongDirState_t VehLongDirExt;
    VehLongMotStateExt_t VehLongMotStateExt;
    float CurveC0Ext;
    float CurveC1Ext;
    float SideSlipAngleExt;
} VehSigMain_t;
#define Rte_TypeDef_VehSigMain_t
#endif

#ifndef Rte_TypeDef_WheelHeightLevel_t
typedef struct {
    signed short FrontLeft;
    signed short FrontRight;
    signed short RearLeft;
    signed short RearRight;
} WheelHeightLevel_t;
#define Rte_TypeDef_WheelHeightLevel_t
#endif

#ifndef Rte_TypeDef_VehSigAdd_t
typedef struct {
    State_array_t_1 State;
    float EnvTemp;
    WiperState_t WiperState;
    WiperStage_t WiperStage;
    boolean WiperOutParkPos;
    WiperWasherFrontState_t WiperWasherFrontState;
    boolean RainSensor;
    TurnSignal_t TurnSignal;
    boolean FogLampFront;
    boolean FogLampRear;
    float RoadWhlAngFr;
    float RoadWhlAngRe;
    float Odometer;
    float GasPedalPos;
    boolean KickDown;
    boolean BrakePedalPressed;
    boolean DriverTired;
    SpeedUnit_t SpeedUnit;
    float SpeedoSpeed;
    VEDTrailerConnection_t TrailerConnection;
    VEDTrailerConnection_t TrailerConnBeforeShutDown;
    unsigned char TrailerLengthInput;
    unsigned char ParkAidDet_L;
    unsigned char ParkAidDet_CL;
    unsigned char ParkAidDet_CR;
    unsigned char ParkAidDet_R;
    IgnitionSwitch_t IgnitionSwitch;
    eSuspensionSystem_t eSuspensionSystem;
    eHeightLevel_t eHeightLevel;
    WheelHeightLevel_t WheelHeightLevel;
    signed short AlignmentByte1;
    signed short AlignmentByte2;
    float MotorTorque;
} VehSigAdd_t;
#define Rte_TypeDef_VehSigAdd_t
#endif

#ifndef Rte_TypeDef_PowerTrain_t
typedef struct {
    TransmissionGear_t ActualGear;
    TransmissionGear_t TargetGear;
    boolean EngineRunning;
    unsigned char FillByte;
    State_array_t_2 State;
} PowerTrain_t;
#define Rte_TypeDef_PowerTrain_t
#endif

#ifndef Rte_TypeDef_Brake_t
typedef struct {
    boolean ABSCtrl;
    boolean TCSCtrl;
    State_array_t_3 State;
} Brake_t;
#define Rte_TypeDef_Brake_t
#endif

#ifndef Rte_TypeDef_VED_VehSig_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    VehSigMain_t VehSigMain;
    VehSigAdd_t VehSigAdd;
    PowerTrain_t PowerTrain;
    Brake_t Brake;
} VED_VehSig_t;
#define Rte_TypeDef_VED_VehSig_t
#endif

#ifndef Rte_TypeDef_AEB_VehSig_t
typedef struct {
    unsigned char CDCS_FCW_OnOffSet;
    unsigned char CDCS_AEB_OnOffSet;
    boolean AEB_Inhibit_flag;
    boolean AEB_Active_flag;
    boolean AEB_Error_flag;
    boolean FCW_Inhibit_flag;
    boolean FCW_Active_flag;
    boolean FCW_Error_flag;
    unsigned char FCW_SensitiveLevel;
} AEB_VehSig_t;
#define Rte_TypeDef_AEB_VehSig_t
#endif

#ifndef Rte_TypeDef_KinOutput_t
typedef struct {
    signed short MinRequestedLongAcceleration;
    signed short MaxRequestedLongAcceleration;
    DAS_status_t DAS_status;
    boolean DAS_accel_limitation_active;
    DAS_failure_information_t DAS_failure_information;
    boolean brake_pre_fill;
    boolean stand_still_request;
} KinOutput_t;
#define Rte_TypeDef_KinOutput_t
#endif

#ifndef Rte_TypeDef_KinDriverOutput_t
typedef struct {
    boolean drive_off_request;
    boolean drive_off_possible;
    boolean drive_off_inhibit;
    boolean drive_off_confirm;
    cc_reported_error_t failure_state;
    display_op_status_t operational_mode;
    ldm_drive_mode_t ldm_drive_mode;
} KinDriverOutput_t;
#define Rte_TypeDef_KinDriverOutput_t
#endif

#ifndef Rte_TypeDef_KinFctInfo_t
typedef struct {
    boolean object_detected;
    boolean FCA_alert;
    FCA_status_t FCA_status;
    DM_alert_level_t DM_alert_level;
    DM_status_t DM_status;
    boolean headway_control_alert;
    float requested_distance;
    unsigned char obj_interest_distance;
    unsigned char headway_setting;
    unsigned char desired_speed;
    unsigned char recommended_speed;
    unsigned char speed_target;
} KinFctInfo_t;
#define Rte_TypeDef_KinFctInfo_t
#endif

#ifndef Rte_TypeDef_CustomOutput_t
typedef struct {
    boolean AVLC_Object_Effective;
    boolean Ext_CcCancel_Rq;
    boolean Ext_CcCancelWarn_Rq;
    Ext_AccInhibitionRq_Out_t Ext_AccInhibitionRq_Out;
    acceleration_t RequestedLongAccelRaw;
    signed short RequestedLongPosAccelGrad;
    signed short RequestedLongNegAccelGrad;
    acceleration_t RequestedLongAccelHyst;
    boolean SoftStopRequest;
    velocity_t MaxOperatingSpeed;
    velocity_t MinOperatingSpeed;
    ACCHeadwayLeverSetting_t ACCHeadwayLeverSetting;
    boolean ACCAccelEnable;
    unsigned short ACCDisplayOut;
    unsigned char CruiseCtrlMode;
    ACCInternalState_t ACCInternalState;
} CustomOutput_t;
#define Rte_TypeDef_CustomOutput_t
#endif

#ifndef Rte_TypeDef_LongCtrlOutputCustom_t
typedef struct {
    CustomOutput_t CustomOutput;
} LongCtrlOutputCustom_t;
#define Rte_TypeDef_LongCtrlOutputCustom_t
#endif

#ifndef Rte_TypeDef_VLC_LongCtrlOutput_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    KinOutput_t KinOutput;
    KinDriverOutput_t DriverData;
    KinFctInfo_t KinFctInfo;
    LongCtrlOutputCustom_t Custom;
} VLC_LongCtrlOutput_t;
#define Rte_TypeDef_VLC_LongCtrlOutput_t
#endif

#ifndef Rte_TypeDef_VLC_DIMOutputCustom_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    DIMOutMonState_t eDriverMonitoringState;
    eDriverAttentionState_t eDriverAttentionState;
    signed char iLaneChangeProbability;
    eDriverFeedbackState_t eDriverFeedbackState;
    eDriverActivityState_t eDriverActivityState;
    DriverAction_t DriverAction;
} VLC_DIMOutputCustom_t;
#define Rte_TypeDef_VLC_DIMOutputCustom_t
#endif

#ifndef Rte_TypeDef_VLC_SADOutputGeneric_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
} VLC_SADOutputGeneric_t;
#define Rte_TypeDef_VLC_SADOutputGeneric_t
#endif

#ifndef Rte_TypeDef_SADOutCustDegadationState_t
typedef struct {
    boolean bWarningsNotPossible;
    boolean bBrakePreCondNotPossible;
    boolean bPreBrakeNotPossible;
    boolean bPreCrashNotPossible;
} SADOutCustDegadationState_t;
#define Rte_TypeDef_SADOutCustDegadationState_t
#endif

#ifndef Rte_TypeDef_HEADOutCustHBA_t
typedef struct {
    float fHBADecel;
    unsigned char uiHBALevel;
    float uiHBAFactor;
    unsigned char uActiveHyp;
    eEBAFctChan_t eFctChan;
} HEADOutCustHBA_t;
#define Rte_TypeDef_HEADOutCustHBA_t
#endif

#ifndef Rte_TypeDef_SADOutCustPrefill_t
typedef struct {
    boolean bPrefillActive;
    boolean bPreRunActive;
    eGeneratorControl_t eGeneratorControl;
    unsigned char uActiveHyp;
    eEBAFctChan_t eFctChan;
} SADOutCustPrefill_t;
#define Rte_TypeDef_SADOutCustPrefill_t
#endif

#ifndef Rte_TypeDef_SADOutCustPrebrake_t
typedef struct {
    float fPreBrakeDecel;
    boolean bPreBrakeDecelEnabled;
    boolean bPreBrakeStdstillRequest;
    boolean bPreBrakeCamConfirmed;
    unsigned char uPreBrakeLevel;
    boolean bPreBrakeEmergency;
    unsigned char uActiveHyp;
    eEBAFctChan_t eFctChan;
} SADOutCustPrebrake_t;
#define Rte_TypeDef_SADOutCustPrebrake_t
#endif

#ifndef Rte_TypeDef_SADOutCustPreCrash_t
typedef struct {
    float fPreCrashTTC;
    float fPreCrashCV;
    boolean bEMAActive;
    eEBAFctChan_t eEBAFctChan;
    unsigned char uActiveHyp;
} SADOutCustPreCrash_t;
#define Rte_TypeDef_SADOutCustPreCrash_t
#endif

#ifndef Rte_TypeDef_SADOutCustHypReactions_t
typedef struct {
    boolean bLatentPreWarningActive;
    boolean bLatentAcuteWarningActive;
    boolean bDynamicPreWarningActive;
    boolean bDynamicAcuteWarningActive;
    boolean bDynamicHapticAcuteWarningActive;
    boolean bHBAActive;
    boolean bPreFillActive;
    boolean bPreBrakeActive;
} SADOutCustHypReactions_t;
#define Rte_TypeDef_SADOutCustHypReactions_t
#endif

typedef SADOutCustHypReactions_t rgHypReactions_array_t[12];

#ifndef Rte_TypeDef_SADOutCustWarningType_t
typedef struct {
    eEBASignalState_t eSignal;
    eEBAFctChan_t eFctChan;
    unsigned char uActiveHyp;
} SADOutCustWarningType_t;
#define Rte_TypeDef_SADOutCustWarningType_t
#endif

#ifndef Rte_TypeDef_SADOutCustHaptWaningType_t
typedef struct {
    eEBASignalState_t eSignal;
    eEBAFctChan_t eFctChan;
    unsigned char uActiveHyp;
    eWarnSens_t eWarnSens;
} SADOutCustHaptWaningType_t;
#define Rte_TypeDef_SADOutCustHaptWaningType_t
#endif

#ifndef Rte_TypeDef_SADOutCustWarnings_t
typedef struct {
    SADOutCustWarningType_t sPreStaticWarning;
    SADOutCustWarningType_t sAcuteStaticWarning;
    SADOutCustWarningType_t sPreDynamicWarning;
    SADOutCustWarningType_t sAcuteDynamicWarning;
    SADOutCustHaptWaningType_t sAcuteDynamicHaptWarning;
} SADOutCustWarnings_t;
#define Rte_TypeDef_SADOutCustWarnings_t
#endif

#ifndef Rte_TypeDef_SADOutCustVLCData_t
typedef struct {
    float fVelocityFactor;
    ObjNumber_t iObjRef;
    float fSafeObjDistance;
} SADOutCustVLCData_t;
#define Rte_TypeDef_SADOutCustVLCData_t
#endif

#ifndef Rte_TypeDef_SADOutCustVLCQualifier_t
typedef struct {
    eFunctionQualifier_t ePedFunctionQualifier;
    eFunctionQualifier_t eVehFunctionQualifier;
    eFunctionQualifier_t eUnclassifiedFunctionQualifier;
    eFunctionQualifier_t ePreCrashFunctionQualifier;
} SADOutCustVLCQualifier_t;
#define Rte_TypeDef_SADOutCustVLCQualifier_t
#endif

#ifndef Rte_TypeDef_HEADOutCustThreatLvl_t
typedef struct {
    eThreatLvl_t threatLvl;
} HEADOutCustThreatLvl_t;
#define Rte_TypeDef_HEADOutCustThreatLvl_t
#endif

#ifndef VLC_LODMCOutput_t_define
typedef struct VLC_LODMCOutput_t{
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    float32 EngineTRQReq;              // Requested torque as input for Engine Control module
    boolean TorqueActiveFlag;        // torque activation flag as input for Engine Control module
    unsigned char BrakeActiveFlag;              // Brake activation flag as input for Brake Control module
    float32 DecelReq;                  // Requested declaration as input for Brake Control module
}VLC_LODMCOutput_t;
#define VLC_LODMCOutput_t_define
#endif

#ifndef Rte_TypeDef_VLC_SADOutputCustom_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    SADOutCustDegadationState_t sDegradationState;
    HEADOutCustHBA_t sHBA;
    SADOutCustPrefill_t sPrefill;
    SADOutCustPrebrake_t sPreBrake;
    SADOutCustWarnings_t sWarnings;
    SADOutCustPreCrash_t sPreCrash;
    rgHypReactions_array_t rgHypReactions;
    SADOutCustVLCData_t sVLCData;
    SADOutCustVLCQualifier_t sVLCQualifiers;
    HEADOutCustThreatLvl_t sThreatlvl;
    TcuSysInfo_t sTcuSysInfo;
} VLC_SADOutputCustom_t;
#define Rte_TypeDef_VLC_SADOutputCustom_t
#endif

#ifndef Rte_TypeDef_VLC_DFVErrorOut_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    AlgoErrorState_t YawRateUnplausible;
    AlgoErrorState_t VehicleSpeedUnplausible;
} VLC_DFVErrorOut_t;
#define Rte_TypeDef_VLC_DFVErrorOut_t
#endif

#ifndef Rte_TypeDef_VLCVehOutLongArb_t
typedef struct {
    float fLongReqAccel;
    VLCStateSig_t LongReqAccelState;
    eVLCVehLongReqSource_t eLongReqAccelSource;
} VLCVehOutLongArb_t;
#define Rte_TypeDef_VLCVehOutLongArb_t
#endif

#ifndef Rte_TypeDef_FFDOutputDFV_t
typedef struct {
    signed short Estimated_Curve_R_Value;
    PCS_Guard_Status_t DDR_PCS_Guard_Status;
    boolean FFD_PCS_DW_Buzzer_Request_Flag;
    boolean FFD_PCS_PedW_Buzzer_Request_Flag;
    boolean FFD_PCS_ALM_Buzzer_Request_Flag;
    boolean FFD_Override_ALM_Request_Flag;
    boolean FFD_Override_AEB_Request_Flag;
    boolean FFD_Override_Brake_Request_Flag;
    boolean FFD_Override_Steer_Request_Flag;
    boolean FFD_Override_Accel_Request_Flag;
} FFDOutputDFV_t;
#define Rte_TypeDef_FFDOutputDFV_t
#endif

#ifndef Rte_TypeDef_DFVOutArbCustTMC_t
typedef struct {
    PCS_ALM_Disp_Req_Status_t PCS_ALM_Disp_Req_Status;
    DFVInspMode_t eACCInspMode;
    PCS_DW_Disp_Req_Status_t PCS_DW_Disp_Req_Status;
    PCS_PedW_Disp_Req_Status_t PCS_PedW_Disp_Req_Status;
    eBuzzerPattern_t eBuzzerPattern;
    boolean PCS_Disp_Status_Req_Flag;
    FFDOutputDFV_t FFDOutput;
} DFVOutArbCustTMC_t;
#define Rte_TypeDef_DFVOutArbCustTMC_t
#endif

#ifndef Rte_TypeDef_VLC_DFVOutArbitrated_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNr;
    SignalHeader_t sSigHeader;
    VLCVehOutLongArb_t Long;
    DFVOutArbCustTMC_t GSP_C;
} VLC_DFVOutArbitrated_t;
#define Rte_TypeDef_VLC_DFVOutArbitrated_t
#endif

typedef struct {
    float32 VLCVEH_Kf_SteeringRatio; /* Overall steering ratio */
    float32 VLCVEH_Kf_WheelBase; /* Distance between front and rear axle     */
    float32 VLCVEH_Kf_TrackWidthFront;  /* Track width front axle  */
    float32 VLCVEH_Kf_TrackWidthRear;   /* Track width rear axle   */
    float32 VLCVEH_Kf_VehWidth;         /* Vehicle width         */
    float32 VLCVEH_Kf_VehLength;        /* Vehicle length          */
    float32 VLCVEH_Kf_CntrOfGravHeight; /* Center of gravity height */
    float32 VLCVEH_Kf_AxisLoadDistr; /* Ratio between rear axle load and total
                                        load  */
    float32 VLCVEH_Kf_FrCrnrStiff;   /* Front axle cornering stiffness  */
    float32 VLCVEH_Kf_ReCrnrStiff;   /* Rear axle cornering stiffness */
    float32 VLCVEH_Kf_OverhangFront;

    float32 VLCVEH_Kf_SensorLatPos;  /* Lateral displacment related to vehicle
                                        reference point (+ left) */
    float32 VLCVEH_Kf_SensorLongPos; /* Longitudinal displacment related to
                                        vehicle reference point (+ fwd) */
    float32
        VLCVEH_Kf_SensorVertPos; /* Vertical displacemnt (height) over ground */
    float32
        VLCVEH_Kf_SensorLongPosToCoG;   /* Longitudinal displacment related to
                                           vehicle center of gravity (+ fwd) */
    float32 VLCVEH_Kf_SensorPitchAngle; /* Angular displacment about sensor
                                           lateral axis */
    uint8 VLCVEH_Ku_SensorOrientation;
    float32 VLCVEH_Kf_SensorRollAngle; /* Angular displacement about sensor
                                          longitudinal axis */
    float32 VLCVEH_Kf_SensorYawAngle;  /* Angular displacemnt about vehicle
                                          vertical axis*/

    uint8 VLCVEH_Ku_AEBTriggerSensitivity; /* Sensitivity of decision triggering
                                              deceleration*/
    uint8 VLCVEH_Ku_SensitivityReserved1;  /* Sensitivity Reserved*/
    uint8 VLCVEH_Ku_SensitivityReserved2;  /* Sensitivity Reserved*/
    uint8 VLCVEH_Ku_SensitivityReserved3;  /* Sensitivity Reserved*/
    uint8 VLCVEH_Ku_SensitivityReserved4;  /* Sensitivity Reserved*/
    uint8 VLCVEH_Ku_SensitivityReserved5;  /* Sensitivity Reserved*/

    float32 VLCVEH_Kf_Reserved1; /*Reserved*/
    float32 VLCVEH_Kf_Reserved2; /*Reserved*/
    float32 VLCVEH_Kf_Reserved3; /*Reserved*/
    float32 VLCVEH_Kf_Reserved4; /*Reserved*/
    float32 VLCVEH_Kf_Reserved5; /*Reserved*/
    uint8 VLCVEH_Ku_Reserved1;   /*Reserved*/
    uint8 VLCVEH_Ku_Reserved2;   /*Reserved*/
    uint8 VLCVEH_Ku_Reserved3;   /*Reserved*/
    uint8 VLCVEH_Ku_Reserved4;   /*Reserved*/
    uint8 VLCVEH_Ku_Reserved5;   /*Reserved*/
    boolean VLCVEH_Kb_Reserved1; /*Reserved*/
    boolean VLCVEH_Kb_Reserved2; /*Reserved*/
    boolean VLCVEH_Kb_Reserved3; /*Reserved*/
    boolean VLCVEH_Kb_Reserved4; /*Reserved*/
    boolean VLCVEH_Kb_Reserved5; /*Reserved*/
} VLCVeh_Parameters_t;           /* vlc veh function calibration parameters*/

#ifndef Rte_TypeDef_VLCVeh_NvramData_t
typedef struct {
    float32 VLCVEH_Nf_Reserved1; /*Reserved*/
    float32 VLCVEH_Nf_Reserved2; /*Reserved*/
    float32 VLCVEH_Nf_Reserved3; /*Reserved*/
    float32 VLCVEH_Nf_Reserved4; /*Reserved*/
    float32 VLCVEH_Nf_Reserved5; /*Reserved*/
    uint8 VLCVEH_Nu_FCWSensitivity;
    uint8 VLCVEH_Nu_Reserved2; /*Reserved*/
    uint8 VLCVEH_Nu_Reserved3; /*Reserved*/
    uint8 VLCVEH_Nu_Reserved4; /*Reserved*/
    uint8 VLCVEH_Nu_Reserved5; /*Reserved*/
    boolean VLCVEH_Nb_ACCOnOffSwitch;
    boolean VLCVEH_Nb_SLFOnOffSwitch;
    boolean VLCVEH_Nb_AEBOnOffSwitch;
    boolean VLCVEH_Nb_FCWOnOffSwitch;
    boolean VLCVEH_Nb_Reserved5; /*Reserved*/
} VLCVeh_NvramData_t;            /* vlc veh function nvram data*/
#define Rte_TypeDef_VLCVeh_NvramData_t
#endif

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
typedef t_CourseInfo CourseInfo_array_t[4];
typedef t_LaneMarkerInfo LaneMarkerInfo_array_t[4];
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    SignalHeader_t sSigHeader;
    CourseInfo_array_t CourseInfo;
    LaneMarkerInfo_array_t LaneMarkerInfo;
    t_ConstructionSite ConstructionSite;
} t_CamLaneInputData;
#define Rte_TypeDef_t_CamLaneInputData
#endif

#ifndef Rte_TypeDef_ISAInfo
typedef struct {
    boolean isa_state;
    // boolean set_speed_change_flag;  // If driver changes set speed after isa
    //                                 // speed is active and before it is
    //                                 // released, this flag will be 1.
    uint8 isa_speed_validity;  // 0: isa speed is invalid, 1:isa speed is valid,
                               // 2: isa speed is accept as set speed, 3: isa
                               // speed is released, restore the previous set
                               // speed.
    uint8 isa_speed;
    uint8 previous_set_speed;  // Store set speed before isa speed is accepted.
    uint8 previous_isa_speed;  // Store the last ISA speed.
} ISAInfo;
#define Rte_TypeDef_ISAInfo
#endif

#ifndef Rte_TypeDef_PathGeometry
typedef struct {
    float32 lateral_deviation;
    float32 heading_angle;
    float32 curvature;
    float32 curvature_rate;
    float32 view_range;
} PathGeometry;
#define Rte_TypeDef_PathGeometry
#endif

#ifndef PathKinematic_define
typedef struct {
    float32 lateral_acceleration_limit;
    float32 longitudinal_acceleration_limit;
    float32 longitudinal_velocity_limit;
    float32 longitudinal_velocity_actual;
} PathKinematic;
#define PathKinematic_define
#endif

#ifndef PathPointPosition_define
typedef struct {
    float32 pos_x;
    float32 pos_y;
    float32 distance_interval;
    float32 distance_global;
} PathPointPosition;
#define PathPointPosition_define
#endif

#ifndef PathPointInfo_define
typedef struct {
    PathGeometry geometry;
    PathKinematic kinematic;
    PathPointPosition position;
} PathPointInfo;
#define PathPointInfo_define
#endif

#ifndef LaneProperty_define
typedef struct {
    uint8 confidence;
    boolean validity;
} LaneProperty;
#define LaneProperty_define
#endif

#ifndef LaneMarkInfo_define
typedef struct {
    PathGeometry geometry;
    PathGeometry filter_geometry;
    LaneProperty property;
} LaneMarkInfo;
#define LaneMarkInfo_define
#endif

#ifndef Rte_TypeDef_PACCInfo
typedef struct {
    boolean pacc_state;
    float32 pacc_acceleration;
    float32 pacc_raw_acceleration;
    PathGeometry control_path;  // path used for PACC control
} PACCInfo;
#define Rte_TypeDef_PACCInfo
#endif

#ifndef cc_control_data_t_define
typedef struct cc_control_data_t {
    sint16 LIM_ACCELERATION;
    sint16 VLC_ACCELERATION;
    sint16 VLC_ACCELERATION_P_PART;
    sint16 VLC_ACCELERATION_I_PART;
    sint16 MAXIMUM_ACCELERATION_LIMIT;
    sint16 MINIMUM_ACCELERATION_LIMIT;
    sint32 ACCELERATION_REQUEST_GRAD;
    uint8 DECEL_LIM_OVERRIDE_ACTIVE;
    uint8 VLC_AVLC_TO_VLC_TRANSITION;
    uint16 DECEL_LIM_ACTIVE_COUNTER;
    uint16 VLC_ACCEL_FILTER_TIME;
} cc_control_data_t;
#define cc_control_data_t_define
#endif

#ifndef cc_accel_gradient_limits_t_define
typedef struct cc_accel_gradient_limits_t {
    sint32 MAX_NEG_GRAD;
    sint32 MAX_POS_GRAD;
} cc_accel_gradient_limits_t;
#define cc_accel_gradient_limits_t_define
#endif

#ifndef cc_accel_status_t_define
typedef struct cc_accel_status_t {
    unsigned char LAT_ACCEL_LIM_ACTIVE : 1;
    unsigned char DECEL_LIM_ENGAGE : 1;
    unsigned char OBJECT_EFFECTIVE : 1;
    unsigned char ACCEL_RAMP_ACTIVE : 1;
    unsigned char ALLOW_INIT : 1;
    unsigned char CANCEL_RAMP : 1;
    unsigned char RAPID_RAMP : 1;
    unsigned char NO_RAMP : 1;
} cc_accel_status_t;
#define cc_accel_status_t_define
#endif

#ifndef cc_acceleration_control_data_t_define
typedef struct cc_acceleration_control_data_t {
    sint16 MAXIMUM_ACCELERATION_LATERAL_LIMITED;
    sint16 MAXIMUM_COMMANDED_ACCELERATION;
    sint16 MINIMUM_COMMANDED_ACCELERATION;
    sint16 MAXIMUM_REQUESTED_ACCELERATION;
    sint16 MINIMUM_REQUESTED_ACCELERATION;
    sint16 MAXIMUM_REQUESTED_ACCELERATION_FILTERED;
    sint16 MINIMUM_REQUESTED_ACCELERATION_FILTERED;
    sint16 MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
    sint16 MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
    sint16 MIN_ALLOWED_ACCEL;
    sint16 MAX_ALLOWED_ACCEL;
    sint16 MAX_CUSTOM_ALLOWED_ACCEL;
    sint16 MIN_CUSTOM_ALLOWED_ACCEL;
    cc_accel_status_t ACCEL_STATUS;
    cc_accel_gradient_limits_t ACCEL_GRADIENT_LIMITS;
} cc_acceleration_control_data_t;
#define cc_acceleration_control_data_t_define
#endif

/*! Inputs required by VLC component for vehicle cycle processing */
typedef struct {
    const VLCCtrlData_t*
        pVehCtrlData;               /*!< VLC_VEH operation mode control data */
    const VED_VehDyn_t* pEgoDynRaw; /*!< The dynamic vehicle ego data raw */
    const VED_VehPar_t* pEgoStaticData; /*!< the static vehicle ego data */
    /* Longitudinal control input ports */
    const VLC_AccLeverInput_t*
        pAccLever; /*!< ACC lever information (input from driver) */
    VLC_LongCtrlInput_t* pLongCtrlResp; /*!< Dynamic controller response */
    const VLC_acc_object_t*
        pAccDisplayObj; /*!< Display object data output by VLC_SEN */
    const VLC_acc_output_data_t* pAccOutput; /*!< ACC output data by VLC_SEN */
    const VLCSenAccOOI_t* pVLCAccOOIData;
    const t_CamLaneInputData* pCamLaneData; /*!< Camera lane input data */

    const VLC_DIMInputGeneric_t* pDIMInputGeneric; /*!< Generic DIM input */
    const VLC_DIMInputCustom_t* pDIMInputCustom;   /*!< Custom DIM input */
    const VLC_HypothesisIntf_t* pVLCCDHypotheses;  /*!< CD hypotheses */
    VLC_SADInputGeneric_t* pSADInputGeneric;       /*!< Generic HEAD input */
    const VLC_SADInputCustom_t* pSADInputCustom;   /*!< Custom HEAD input */
    /* algo parameters from BSW */
    const Com_AlgoParameters_t*
        pBswAlgoParameters; /*!< Input algo parameters from BSW */
    const VLC_Parameters_t* pCPAR_VLC_Parameters; /*!< VLC Coding Parameters */
    const VED_VehSig_t* pVehSig;       /*!< Direct access to vehicle signals */
    ST_VLCVeh_NVRAMData_t* pVLCVehNvRams; /* vlc veh function nvram data*/
    AEB_VehSig_t* pAebVehSig;
} reqVLCVehPrtList_t;

typedef struct {
    VLC_DFV2SenInfo_t*
        pDFVLongOut; /*!< Internal info passed from VLC_VEH to VLC_SEN */
    VLC_LongCtrlOutput_t*
        pLongCtrlOutput; /*!< Longitudinal controller output data */

    VLC_DIMOutputCustom_t* pDIMOutputCustom;    /*!< Custom DIM output */
    VLC_SADOutputGeneric_t* pHEADOutputGeneric; /*!< Generic HEAD output */
    VLC_SADOutputCustom_t* pHEADOutputCustom;   /*!< Custom HEAD output */

    VLC_DFVErrorOut_t* pErrorOut; /*!< VLC error output */
    VLC_DFVOutArbitrated_t*
        pVLCVehOutArbitrated; /*!< Aribrated output for vehicle functions */
    ST_VLCVeh_NVRAMData_t* pVLCVehNvRams; /* vlc veh function nvram data*/
    ISAInfo* p_isa_info;
    PACCInfo* p_pacc_info;
    VLC_LODMCOutput_t* pLODMCOutput;
} proVLCVehPrtList_t;


typedef struct {
    ubit8_t VLC_TAKE_ACTUAL_SPEED : 1;
    ubit8_t LIM_TAKE_ACTUAL_SPEED : 1;
    ubit8_t VLC_RESUME_SET_SPEED : 1;
    ubit8_t LIM_RESUME_SET_SPEED : 1;
    ubit8_t ACCEL_MODE : 1;
    ubit8_t VLC_INCREASE_SET_SPEED : 1;
    ubit8_t VLC_DECREASE_SET_SPEED : 1;
    ubit8_t DECEL_MODE : 1;

    ubit8_t LIM_INCREASE_SET_SPEED : 1;
    ubit8_t LIM_DECREASE_SET_SPEED : 1;
    ubit8_t SPEED_STEP_1 : 1;
    ubit8_t CANCEL_FUNKTION : 1;
    ubit8_t DISENGAGE_DRIVER_INTERVENED : 1;
    ubit8_t RESET_SETSPEED : 1;
    ubit8_t SWITCH_SPEED_UNIT : 1;
    ubit8_t DRIVE_OFF_CONFIRM : 1;
} cc_driver_operations_t_debug;

typedef struct {
    ubit8_t ACCEPT_VLC_ENGAGEMENT : 1;
    ubit8_t ACCEPT_LIM_ENGAGEMENT : 1;
    ubit8_t END_VLC_ENGAGEMENT : 1;
    ubit8_t DRIVER_OVERRIDE : 1;
    ubit8_t VLC_DISENGAGEMENT : 1;
    ubit8_t VLC_DISENGAGEMENT_RAMP : 1;
    ubit8_t VLC_DECEL_ONLY : 1;
    ubit8_t VLC_END_DECEL_ONLY : 1;

    ubit8_t END_VLC_DISENGAGEMENT : 1;
    ubit8_t LIM_DISENGAGEMENT : 1;
    ubit8_t END_LIM_DISENGAGEMENT : 1;
    ubit8_t GOTO_PERM_LIM : 1;
    ubit8_t END_PERM_LIM : 1;
    ubit8_t DRIVE_OFF_POSSIBLE : 1;
    ubit8_t VLC_ENGAGEMENT_STAT : 1;
    ubit8_t CONTROL_TO_RELEVANT_OBJECT : 1;
} cc_engagement_conditions_t_debug;

typedef struct
{
    cc_driver_operations_t_debug DRIVER_OPERATIONS;
    cc_driver_operations_t_debug DRIVER_OPERATIONS_LAST_CYCLE;
    cc_engagement_conditions_t_debug ENGAGEMENT_CONDITIONS;
    Uint8 CONTROL_STATE;
    Uint8 CONTROL_STATE_LAST_CYCLE;
} vlcVeh_ext_sm_debug_info;


typedef struct {
    cc_acceleration_control_data_t* pVlcACCCtrlData;
    cc_control_data_t* pVlcCCCtrlData;
    vlcVeh_ext_sm_debug_info* pVlcVehSmDebugInfo;
} reqVLCVehDebugList_t;

/*****************************************************************************
  VARIABLEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*! Main VLC_VEH Process function */
//__declspec(dllexport) void VLCVeh_Exec(const reqVLCVehPrtList_t*
// pRequirePorts, const proVLCVehPrtList_t* pProvidePorts);

void LODMCSimplePD();

void VLCVeh_Exec(const reqVLCVehPrtList_t* pRequirePorts,
                 const VLCVeh_Parameters_t* pVLCParameters,
                 const proVLCVehPrtList_t* pProvidePorts,
                 const reqVLCVehDebugList_t* pVLCVehDebugPorts);

#ifdef __cplusplus
}
#endif
#endif
