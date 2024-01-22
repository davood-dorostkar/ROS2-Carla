/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

#pragma once
#ifndef glob_typedefs_H
#define glob_typedefs_H
#include "Rte_Type.h"   //RENDL
#include "TM_Global_Types.h"

// typedef boolean Boolean;
typedef float32 Float32;
typedef float64 Float64;
typedef sint16 Sint16;
typedef sint32 Sint32;
typedef sint8 Sint8;
typedef uint16 Uint16;
typedef uint32 Uint32;
typedef uint8 Uint8;
typedef void Void;
typedef sint8 acc_obj_lane_t;
typedef uint8 obj_class_t;
typedef uint8 INHIBITION_REASON_t;
typedef uint8 acc_situation_class_t;
typedef uint8 e_AlnOpMode_t;
typedef uint8 u_CycleState_t;
typedef uint8 e_AlnState_t;
typedef uint8 e_AlignState_t;
typedef uint8 e_Direction_t;
typedef uint8 e_GainReduction_t;
typedef uint8 e_PowerReduction_t;
typedef uint8 eAssociatedLane_t;
typedef sint8 eObjOOI_t;
typedef uint8 eRelObjLossReason_t;
typedef uint8 CameraFusionPreselBits_t;
typedef uint32 EBACodingBitEnum_t;
typedef uint8 EBAPreBrkAccelParNvState_t;
typedef uint8 Envm_Ped_ArtDummy_t;
typedef uint8 Em_cem_algo_mode_t;
typedef uint8 Envm_cEnvm_algo_sensor_state_t;
typedef uint8 FnSwitchBits_t;
typedef uint8 NaviFusionPreselBits_t;
typedef uint8 ui8_UseFarNearScanForBlck_t;
typedef uint8 EnvmOpMode_t;
typedef uint32 EBACodingParamFmod_t;
typedef uint32 EBACodingParamGen_t;
typedef uint8 t_CamLaneMarkerEnum;
typedef uint8 t_MarkerColor;
typedef uint8 t_MarkerType;
typedef uint8 Envm_t_CustomerProjectDef;
typedef uint8 EMB0_t_SensorSource;
typedef uint8 EMB0_t_Service;
typedef uint8 EM_t_Cust_CamAvailStatus;
typedef uint8 Envm_t_Cust_FusionStatus;
typedef uint8 EM_t_Cust_MatchType;
typedef uint16 EMT0_t_StatusFusion;
typedef uint8 Envm_t_CamBrakeLight;
typedef uint8 Envm_t_CamObjManeuver;
typedef uint8 EM_t_CamTurnLightsState;
typedef uint8 EMT0_t_CamDetectionMode;
typedef uint8 EMT0_t_CamObjAssociatedLane;
typedef uint8 EMT0_t_CamSensorState;
typedef uint8 EMT0_t_MCamObjClass;
typedef uint8 EMT0_t_MCamObjDynProp;
typedef uint8 EMT0_t_ObjFlashLightStat;
typedef uint8 ui_Accuracy_t;
typedef uint8 Envm_t_AmbigState;
typedef uint8 Envm_t_ClusterListValid;
typedef uint8 Envm_t_DynamicProperty;
typedef uint8 Envm_t_InvalidReason;
typedef uint8 Envm_t_PositionInBeam;
typedef uint8 Envm_t_CR_AbsMovingState;
typedef uint8 Envm_t_CR_Classification;
typedef uint8 Envm_t_CR_DynamicProperty;
typedef uint8 EM_t_ARS_DynamicSubProperty;
typedef uint8 Envm_t_CR_MeasuredSources;
typedef uint8 Envm_t_FOVOverlapFar;
typedef uint8 Envm_t_ObjRelationsClass;
typedef uint8 u_RoadSide_t;
typedef uint8 Envm_t_GenEbaHypCat;
typedef uint16 Envm_t_GenEbaInhibit;
typedef uint8 Envm_t_GenEbaObjCat;
typedef uint8 Envm_t_GenFusionLevel;
typedef uint8 Envm_t_GenObjClassification;
typedef uint8 Envm_t_GenObjDynamicProperty;
typedef uint8 Envm_t_GenObjMaintenanceState;
typedef uint8 Envm_t_GenObjOcclusionState;
typedef uint16 EM_t_GenObjSensorSource;
typedef uint8 Envm_t_GenObjShapePointState;
typedef uint8 EM_t_GenObjSplitMergeState;
typedef uint8 eCycleMode_t;
typedef uint8 eLongControlStatus_t;
typedef uint8 DDR_Aln_System_Setting_t;
typedef uint8 DDR_Target_Fusion_Status_t;
typedef uint8 DDR_Target_Recog_Status_t;
typedef uint8 ucPredictState_t;
typedef uint8 DIMInputSignalState_t;
typedef uint8 eDriverSetting_t;
typedef uint8 eTurnIndicator_t;
typedef uint8 DIMOutMonState_t;
typedef uint8 DriverAction_t;
typedef uint8 eDriverActivityState_t;
typedef uint8 eDriverAttentionState_t;
typedef uint8 eDriverFeedbackState_t;
typedef uint8 eBuzzerState_t;
typedef uint8 ucCameraObjectStatus_t;
typedef uint8 ucCameraStatus_t;
typedef uint32 eFunctionSwitch_t;
typedef uint8 eMainSwitch_t;
typedef uint32 eObjectSwitch_t;
typedef uint8 eEBAFctChan_t;
typedef uint8 eEBASignalState_t;
typedef uint8 eFunctionQualifier_t;
typedef uint8 eGeneratorControl_t;
typedef uint8 eThreatLvl_t;
typedef uint8 eWarnSens_t;
typedef uint8 VLCObjUsageState_t;
typedef uint8 EngRun_Stat_t;
typedef uint8 VLCTransmissionGear_t;
typedef uint8 DFVPTSignalState_t;
typedef uint8 PwrFreeD_Stat_TCM_t;
typedef uint8 DFVInspMode_t;
typedef uint8 PCS_ALM_Disp_Req_Status_t;
typedef uint8 PCS_DW_Disp_Req_Status_t;
typedef uint8 PCS_Guard_Status_t;
typedef uint8 PCS_PedW_Disp_Req_Status_t;
typedef uint8 eBuzzerPattern_t;
typedef uint8 eVLCVehLongReqSource_t;
typedef uint8 VLCStateSig_t;
typedef uint8 VLC_OP_MODE_t;
typedef uint8 SpeedUnitEnum_t;
typedef sint16 acceleration_t;
typedef uint8 confidence_t;
typedef sint16 distance_t;
typedef uint8 eEBAObjQuality_t;
typedef uint8 eGDBPDStates_t;
typedef uint16 eTraceBracketBitMasks_t;
typedef sint16 factor_t;
typedef uint16 t_u_DistanceLong;
typedef uint16 times_t;
typedef uint32 times_long_t;
typedef sint16 velocity_t;
typedef uint8 eCDHypothesisType_t;
typedef uint8 eEBADynProp_t;
typedef uint8 eEBAObjectClass_t;
typedef uint32 InterfaceVersion;
typedef uint8 DC_status_information_t;
typedef uint8 country_code_t;
typedef uint8 door_state_t;
typedef uint8 ldm_ctrl_state_t;
typedef uint8 seatbelt_state_t;
typedef uint8 ACCHeadwayLeverSetting_t;
typedef uint8 ACCInternalState_t;
typedef uint8 DAS_failure_information_t;
typedef uint8 DAS_status_t;
typedef uint8 DM_alert_level_t;
typedef uint8 DM_status_t;
typedef uint8 Ext_AccInhibitionRq_Out_t;
typedef uint8 FCA_status_t;
typedef uint8 cc_reported_error_t;
typedef uint8 display_op_status_t;
typedef uint8 ldm_drive_mode_t;
typedef uint32 DLG_VirtAddrs;
typedef uint32 Quality_t;
typedef uint8 eTraceType_t;
typedef uint8 eTraceUpdateState_t;
typedef uint8 u_OpMode_t;
typedef uint8 u_GainRxChipNear_t;
typedef uint8 RSP_t_OpMode;
typedef sint8 eLaneWidthClass_t;
typedef uint8 eLaneWidthSource_t;
typedef uint8 eRoadTypeClass_t;
typedef uint8 eRoadWorksClass_t;
typedef uint8 eTrafficOrientation_t;
typedef uint8 e_BorderType_t;
typedef uint32 AlgoApplicationNumber_t;
typedef uint32 AlgoComponentVersionNumber_t;
//typedef uint16 AlgoCycleCounter_t;
//typedef uint32 AlgoDataTimeStamp_t;
typedef uint8 AlgoErrorState_t;
//typedef uint32 AlgoInterfaceVersionNumber_t;
//typedef uint8 AlgoSignalState_t;
typedef uint32 BaseReturnCode_t;
typedef uint8 FusiErrorExtDirection_t;
typedef uint8 FusiErrorExtType_t;
typedef uint32 GenAlgoQualifiers_t;
typedef sint8 ObjNumber_t;
typedef uint16 ParameterID_t;
typedef uint8 SigState_t;
typedef uint8 TraceID_t;
typedef float32 fAccelAbs_t;
typedef float32 fAccel_t;
typedef float32 fAngleDeg_t;
typedef float32 fAngle_t;
typedef float32 fConfidence_t;
typedef float32 fCurve_t;
typedef float32 fDistance_t;
typedef float32 fGradient_t;
typedef float32 fProbability_t;
typedef float32 fQuality_t;
typedef float32 fRadius_t;
typedef float32 fRatio_t;
typedef float32 fStiffness_t;
typedef float32 fTemperature_t;
typedef float32 fTime_t;
typedef float32 fUncertainty_t;
typedef float32 fVariance_t;
typedef float32 fVelocityAbs_t;
typedef float32 fVelocity_t;
typedef float32 fWeight_t;
typedef float32 fYawRate_t;
typedef uint8 percentage_t;
typedef uint8 quality_t;
typedef uint8 ucConfidence_t;
typedef uint16 uiTime_t;
typedef sint32 SymbolicConstants;
typedef uint8 LongDirState_t;
typedef uint8 VEDCaliState_t;
typedef uint8 VEDCtrlState_t;
typedef uint8 VEDDrvAxle_t;
typedef uint8 VEDIoStateTypes_t;
typedef uint32 VEDIoState_t;
typedef uint32 VDYNvmState_t;
typedef uint8 VEDTrailerConnection_t;
typedef uint8 VehDynStatePos_t;
typedef uint8 VehParAddStatePos_t;
typedef uint8 VehParMainStatePos_t;
typedef uint8 VehParSenMountStatePos_t;
typedef uint8 VehParSensorStatePos_t;
typedef uint8 VehSigAddStatePos_t;
typedef uint8 VehSigBrakeStatePos_t;
typedef uint8 VehSigMainStatePos_t;
typedef uint8 VehSigPowertrainStatePos_t;
typedef uint8 VEDErrState_t;
typedef uint8 MotState_t;
typedef uint8 corrQual_t;
typedef uint8 Orientation_t;
typedef uint8 SteeringVariant_t;
typedef uint8 ActGearPos_t;
typedef uint8 IgnitionSwitch_t;
typedef uint8 ParkBrakeState_t;
typedef uint8 SpeedUnit_t;
typedef uint8 TransmissionGear_t;
typedef uint8 TurnSignal_t;
typedef uint8 VehLongMotStateExt_t;
typedef uint8 WiperStage_t;
typedef uint8 WiperState_t;
typedef uint8 WiperWasherFrontState_t;
typedef uint8 eHeightLevel_t;
typedef uint8 eSuspensionSystem_t;
typedef uint8 Percentage_s_t;
typedef uint8 True_False_s_t;
typedef uint8 SYSDampState_t;
typedef uint8 iRSPFlag_t;
typedef uint16 SYS_t_HWSample;
typedef uint8 SYS_t_Scans;
typedef sint16 s16q3_t;
typedef sint16 s16q4_t;
typedef sint16 s16q5_t;
typedef sint16 s16q6_t;
typedef sint16 s16q7_t;
typedef sint16 s16q8_t;
typedef sint16 s16q9_t;
typedef sint32 s32q18_t;
typedef sint32 s32q24_t;
typedef sint32 s32q3_t;
typedef sint32 s32q4_t;
typedef sint32 s32q7_t;
typedef uint16 u16q3_t;
typedef uint16 u16q4_t;
typedef uint16 u16q5_t;
typedef uint16 u16q6_t;
typedef uint16 u16q7_t;
typedef uint16 u16q8_t;
typedef uint16 u16q9_t;
typedef uint32 u32q18_t;
typedef uint32 u32q24_t;
typedef uint32 u32q3_t;
typedef uint32 u32q4_t;
typedef uint32 u32q7_t;
typedef uint8 u8q1_t;
typedef uint8 u8q2_t;
typedef uint8 u8q3_t;
typedef uint8 u8q4_t;
typedef uint8 u8q5_t;
typedef uint8 u8q6_t;
typedef uint8 u8q7_t;
typedef uint8 u8q8_t;
typedef ucConfidence_t ACCObjectQuality_t;
typedef sint32 signed_fuzzy_t;

/************************* Add by Qdl - for pass build *************************/
#define Rte_TypeDef_RadarContiObject
typedef struct
{
  sint8 sensor_id;
  Timestamp recv_time_ns;
  uint8 track_id;
  float32 distance_long;
  float32 distance_lat;
  float32 velocity_long;
  float32 velocity_lat;
  float32 RCS;
  uint8 dynamic_prop_enum;
  boolean quality_vaild;
  float32 distance_long_rms;
  float32 distance_lat_rms;
  float32 velocity_long_rms;
  float32 velocity_lat_rms;
  float32 accel_long_rms;
  float32 accel_lat_rms;
  float32 orientation_rms;
  float32 prob_of_exist;
  uint8 measurement_state_enum;
  boolean extend_vaild;
  float32 accel_long;
  float32 accel_lat;
  float32 orientation;
  float32 length;
  float32 width;
  uint8 object_type_enum;
} RadarContiObject;

/* Add by Qdl - for pass build */
#define Rte_TypeDef_Rte_DT_ST_RadarInput2SOC_t_3
typedef RadarContiObject Rte_DT_ST_RadarInput2SOC_t_3[168];

#define Rte_TypeDef_ADCU_FID
typedef uint8 ADCU_FID[36];

#define Rte_TypeDef_LaneInformation_t
typedef struct
{
  uint8 eAssociatedLane;
  uint8 eFuncAssociatedLane;
  uint8 uiCutInProbability;
  uint8 uiCutOutProbability;
} LaneInformation_t;

#define Rte_TypeDef_ObjOfInterest_t
typedef struct
{
  uint8 cExternalID;
  sint8 eObjOOI;
} ObjOfInterest_t;

#define Rte_TypeDef_LegacyAOL_t
typedef struct
{
  float32 fDistToRef;
} LegacyAOL_t;

#define Rte_TypeDef_FCTPubObject_t
typedef struct
{
  LaneInformation_t LaneInformation;
  ObjOfInterest_t ObjOfInterest;
  LegacyAOL_t Legacy;
} FCTPubObject_t;

#define Rte_TypeDef_Rte_DT_EMAssessedObjList_t_3
typedef FCTPubObject_t Rte_DT_EMAssessedObjList_t_3[40];

#define Rte_TypeDef_Rte_DT_EMHeaderAssessedObjList_t_2
typedef sint8 Rte_DT_EMHeaderAssessedObjList_t_2[6];

#define Rte_TypeDef_EMHeaderAssessedObjList_t
typedef struct
{
  sint8 iNumOfUsedObjects;
  uint8 iPadding;
  Rte_DT_EMHeaderAssessedObjList_t_2 aiOOIList;
  uint8 eRelObjLossReason;
} EMHeaderAssessedObjList_t;

#define Rte_TypeDef_EMAssessedObjList_t
typedef struct
{
  uint32 uiVersionNumber;
  ENVMSignalHeader_t sSigHeader;
  EMHeaderAssessedObjList_t HeaderAssessedObjList;
  Rte_DT_EMAssessedObjList_t_3 ObjList;
} EMAssessedObjList_t;

#define Rte_TypeDef_ST_CarConfigParamOut
typedef struct
{
  uint32 uiFuncConfig;
  uint8 uCarParameter;
} ST_CarConfigParamOut;

#define Rte_TypeDef_FIDOutCondition_t
typedef struct
{
  boolean bRCA;
  boolean bFCA;
  boolean bFCB;
  boolean bRCC;
} FIDOutCondition_t;

#define Rte_TypeDef_FIDMOut_ACC_t
typedef struct
{
  FIDOutCondition_t FIDOut_ACC_cdt;
} FIDMOut_ACC_t;

#define Rte_TypeDef_FIDMOut_AEB_t
typedef struct
{
  FIDOutCondition_t FIDOut_AEB_cdt;
} FIDMOut_AEB_t;

#define Rte_TypeDef_FIDMOut_DOW_t
typedef struct
{
  FIDOutCondition_t FIDOut_DOW_cdt;
} FIDMOut_DOW_t;

#define Rte_TypeDef_FIDMOut_FCTA_t
typedef struct
{
  FIDOutCondition_t FIDOut_FCTA_cdt;
} FIDMOut_FCTA_t;

#define Rte_TypeDef_FIDMOut_FCW_t
typedef struct
{
  FIDOutCondition_t FIDOut_FCW_cdt;
} FIDMOut_FCW_t;

#define Rte_TypeDef_FIDMOut_IHBC_t
typedef struct
{
  FIDOutCondition_t FIDOut_IHBC_cdt;
} FIDMOut_IHBC_t;

#define Rte_TypeDef_FIDMOut_LDP_t
typedef struct
{
  FIDOutCondition_t FIDOut_LDP_cdt;
} FIDMOut_LDP_t;

#define Rte_TypeDef_FIDMOut_LDW_t
typedef struct
{
  FIDOutCondition_t FIDOut_LDW_cdt;
} FIDMOut_LDW_t;

#define Rte_TypeDef_FIDMOut_Pilot_t
typedef struct
{
  FIDOutCondition_t FIDOut_Pilot_cdt;
} FIDMOut_Pilot_t;

#define Rte_TypeDef_FIDMOut_RCTA_t
typedef struct
{
  FIDOutCondition_t FIDOut_RCTA_cdt;
} FIDMOut_RCTA_t;

#  define Rte_TypeDef_FIDMOut_RCW_t
typedef struct
{
  FIDOutCondition_t FIDOut_RCW_cdt;
} FIDMOut_RCW_t;

#  define Rte_TypeDef_FIDMOut_TSR_t
typedef struct
{
  FIDOutCondition_t FIDOut_TSR_cdt;
} FIDMOut_TSR_t;

#  define Rte_TypeDef_FIDMout_BSD_t
typedef struct
{
  FIDOutCondition_t FIDOut_BSD_cdt;
} FIDMout_BSD_t;

#  define Rte_TypeDef_FIDMout_LCA_t
typedef struct
{
  FIDOutCondition_t FIDOut_LCA_cdt;
} FIDMout_LCA_t;

#  define Rte_TypeDef_FIDMout_NOP_t
typedef struct
{
  FIDOutCondition_t FIDOut_NOP_cdt;
} FIDMout_NOP_t;

#define Rte_TypeDef_ST_FIDMOutPro_t
typedef struct
{
  FIDMOut_ACC_t FIDMOut_ACC;
  FIDMOut_AEB_t FIDMOut_AEB;
  FIDMOut_FCW_t FIDMOut_FCW;
  FIDMOut_RCTA_t FIDMOut_RCTA;
  FIDMOut_FCTA_t FIDMOut_FCTA;
  FIDMOut_TSR_t FIDMOut_TSR;
  FIDMout_LCA_t FIDMOut_LCA;
  FIDMout_BSD_t FIDMOut_BSD;
  FIDMOut_LDW_t FIDMOut_LDW;
  FIDMOut_LDP_t FIDMOut_LDP;
  FIDMOut_DOW_t FIDMOut_DOW;
  FIDMOut_RCW_t FIDMOut_RCW;
  FIDMOut_IHBC_t FIDMOut_IHBC;
  FIDMOut_Pilot_t FIDMOut_Pilot;
  FIDMout_NOP_t FIDMOut_NOP;
} ST_FIDMOutPro_t;

#  define Rte_TypeDef_EPS_TimeStamp
typedef struct
{
  uint8 timeBaseStatus;
  uint32 nanoseconds;
  uint32 seconds;
  uint16 secondsHi;
} EPS_TimeStamp;

#  define Rte_TypeDef_EM_SRR_GenAttributes_t
typedef struct
{
  uint8 eDynamicProperty_nu;
  uint8 uiDynConfidence_per;
  uint32 eClassification_nu;
  uint8 uiClassConfidence_per;
} EM_SRR_GenAttributes_t;

#  define Rte_TypeDef_EM_SRR_GenGenerals_t
typedef struct
{
  float32 fLifeTime_s;
  uint16 uiLifeCycles_nu;
  uint8 uiMaintenanceState_nu;
  uint16 uiID_nu;
} EM_SRR_GenGenerals_t;

#  define Rte_TypeDef_EM_SRR_GenGeometry_t
typedef struct
{
  float32 fWidth_met;
  float32 fWidthStd_met;
  float32 fWidthLeft_met;
  float32 fWidthRight_met;
  float32 fLength_met;
  float32 fLengthStd_met;
  float32 fLengthFront_met;
  float32 fLengthRear_met;
  float32 fOrientation_rad;
  float32 fOrientationStd_rad;
} EM_SRR_GenGeometry_t;

#  define Rte_TypeDef_EM_SRR_GenKinematics_t
typedef struct
{
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
} EM_SRR_GenKinematics_t;

#  define Rte_TypeDef_EM_SRR_GenQualifiers_t
typedef struct
{
  float32 fProbabilityOfExistence_per;
  uint8 uiMeasuredTargetFrequency_nu;
  boolean bObjStable;
} EM_SRR_GenQualifiers_t;

#  define Rte_TypeDef_EM_SRR_GenObject_t
typedef struct
{
  EM_SRR_GenKinematics_t sKinematic;
  EM_SRR_GenGeometry_t sGeometry;
  EM_SRR_GenGenerals_t sGeneral;
  EM_SRR_GenAttributes_t sAttribute;
  EM_SRR_GenQualifiers_t sQualifier;
} EM_SRR_GenObject_t;

#  define Rte_TypeDef_SRRPos_st
typedef struct
{
  uint32 uiSRRLoc;
  float32 fPosX;
  float32 fPosY;
  float32 fPosZ;
  float32 fHeadAngle;
} SRRPos_st;

#  define Rte_TypeDef_rt_Array_EM_SRR_GenObject_t_40
typedef EM_SRR_GenObject_t rt_Array_EM_SRR_GenObject_t_40[40];

#  define Rte_TypeDef_EMSRRGenObjectArray_t
typedef struct
{
  ENVMSignalHeader_t sSigHeader;
  uint8 uiNumOfObjects;
  rt_Array_EM_SRR_GenObject_t_40 aObjects;
  SRRPos_st sSensorPosition;
} EMSRRGenObjectArray_t;

#  define Rte_TypeDef_EMSRRGenObjList_t
typedef struct
{
  uint32 uiVersionNumber;
  ENVMSignalHeader_t sSigHeader;
  EMSRRGenObjectArray_t aFrontLeftObjects;
  EMSRRGenObjectArray_t aFrontRightObjects;
  EMSRRGenObjectArray_t aRearLeftObjects;
  EMSRRGenObjectArray_t aRearRightObjects;
} EMSRRGenObjList_t;

#  define Rte_TypeDef_VEDVehSig_t
typedef struct
{
  uint32 uiVersionNumber;
  VEDSignalHeader_t sSigHeader;
  VEDVehSigMain_t VehSigMain;
  VEDVehSigAdd_t VehSigAdd;
  VEDPowerTrain_t PowerTrain;
  VEDBrake_t Brake;
} VEDVehSig_t;

#  define Rte_TypeDef_EMVEDVehPar_t
typedef struct
{
  uint32 uiVersionNumber;
  ENVMSignalHeader_t sSigHeader;
  EMVEDVehParMain_t VehParMain;
  EMVehParAdd_t VehParAdd;
  EMVEDSensorMounting_t SensorMounting;
  EMVEDSensor_t Sensor;
} EMVEDVehPar_t;

#  define Rte_TypeDef_rt_Array_UInt8_16
typedef uint8 rt_Array_UInt8_16[16];

#  define Rte_TypeDef_rt_Array_UInt8_8
typedef uint8 rt_Array_UInt8_8[8];

#  define Rte_TypeDef_DtcInfo
typedef struct
{
  uint32 dtc;
  uint32 event_id;
  uint8 status;
} DtcInfo;

#  define Rte_TypeDef_adcu_ved_dtc_3
typedef DtcInfo adcu_ved_dtc_3[3];

#  define Rte_TypeDef_ST_adcu_ved_dtc
typedef struct
{
  adcu_ved_dtc_3 adcu_ved_dtc;
} ST_adcu_ved_dtc;

#  define Rte_TypeDef_VLCVeh_NVRAMData_t
typedef struct
{
  float32 VLCVEH_Nf_Reserved1;
  float32 VLCVEH_Nf_Reserved2;
  float32 VLCVEH_Nf_Reserved3;
  float32 VLCVEH_Nf_Reserved4;
  float32 VLCVEH_Nf_Reserved5;
  uint8 VLCVEH_Nu_Reserved1;
  uint8 VLCVEH_Nu_Reserved2;
  uint8 VLCVEH_Nu_Reserved3;
  uint8 VLCVEH_Nu_Reserved4;
  uint8 VLCVEH_Nu_Reserved5;
  boolean VLCVEH_Nb_Reserved1;
  boolean VLCVEH_Nb_Reserved2;
  boolean VLCVEH_Nb_Reserved3;
  boolean VLCVEH_Nb_Reserved4;
  boolean VLCVEH_Nb_Reserved5;
} VLCVeh_NVRAMData_t;
/************************* Add by Qdl - for pass build *************************/
// deal for wrapper ---------------------
typedef uint8 State_array_t[12];
typedef uint8 State_array_t_4[16];
typedef float ang_array_t[2];
typedef float rat_array_t_0[2];
typedef float vel_array_t[2];
typedef uint8 State_array_t_5[12];
typedef uint8 State_array_t_6[8];
typedef uint8 State_array_t_7[8];
typedef float rat_array_t[2];
typedef sint8 Envm_t_GenObjSortedIdxArray[40];
// end ----------------------------------

//====st define
typedef enum {
    e_RTA_EVT_Marker = 0,
    e_RTA_EVT_AlgoStart = 1,
    e_RTA_EVT_AlgoEnd = 2,
    e_RTA_EVT_TSK_Switch = 3,
    e_RTA_EVT_TSK_Rdy = 4,
    e_RTA_EVT_MAX_TYPES = 5,

    e_RTA_EVT_LAST_ENTRY = 65536
} AS_t_RtaEventType;

typedef uint8 MEASFuncID_t;
typedef uint8 MEASFuncChannelID_t;
typedef struct MEASInfo_t {
    uint32 VirtualAddress;
    uint32 Length;
    MEASFuncID_t FuncID;
    MEASFuncChannelID_t FuncChannelID;
} MEASInfo_t;

typedef enum {
    MEAS_OK = 0,
    MEAS_JOB_LIMIT = 1,
    MEAS_FULL_BUFFER = 2,
    MEAS_BAD_GROUP_ID = 3,
    MEAS_BAD_DATA_ALIGNMENT = 4,
    MEAS_CALL_FROM_IRQ_LEVEL = 5,
    MEAS_JOB_REJECTED = 6,
    MEAS_BUFFER_SIZE_TOO_BIG = 7,
    MEAS_INFOBLOCK_WRONG = 8
} MEASReturn_t;

typedef void (*MEAS_Callback_t)(void);
typedef MEASReturn_t (*AS_t_MeasHdlr)(const MEASInfo_t*,
                                      void*,
                                      MEAS_Callback_t);
typedef uint32 (*AS_t_GetTimestampuS32)(void);

// #if (!defined(_MSC_VER))
//  typedef void (*AS_t_RTAAlgoServiceAddEvent)(const RTA_t_EventList
//  RTA_t_Event, const uint8 u_GlobalId, const uint8 u_LocalId, const uint8
//  u_OptData); #else typedef
//  sint32(*AS_t_RTAAlgoServiceAddEvent)(AS_t_RtaEventType RtaEvtType, uint8
//  u_IdGlobal, uint8 u_IdLocal, uint8 u_OptData); #endif
typedef struct {
    uint32 u_Version;
    AS_t_MeasHdlr pfMeasFreeze;
    AS_t_GetTimestampuS32 pfGetTimestampuS32;
    // AS_t_RTAAlgoServiceAddEvent 		pfRTAAlgoServiceAddEvent;
} AS_t_ServiceFuncts;

typedef struct {
    signed char iNumOfLaneRight;
    signed char iNumOfLaneLeft;
    float NoLaneProbLeft;
    float NoLaneProbRight;
} LaneMatrix_t;

#ifndef VLCSEN_EXT_H
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

/************************* Add by Qdl - for pass build *************************/
#define Rte_TypeDef_ST_RadarInput2SOC_t
typedef struct
{
  MsgHeader header;
  Timestamp actual_time_ns;
  uint8 bus_fault;
  Rte_DT_ST_RadarInput2SOC_t_3 object_list;
} ST_RadarInput2SOC_t;

/************************* Add by Qdl - for pass build *************************/
#endif
#endif

#endif
