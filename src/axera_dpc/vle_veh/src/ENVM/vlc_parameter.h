/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * huangchangchang <huangchangchang@senseauto.com>
 * wangsifan <wangsifan@senseauto.com>
 */
#pragma once

#ifndef PARAMETER_INPUT_H
#define PARAMETER_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "envm_ext.h"

#define VED_SET_IO_STATE(pos_, state_, val_) ((val_)[(pos_)] = (state_))
#define VED_GET_IO_STATE(pos_, val_) ((val_)[(pos_)])

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

typedef enum {
    DataProcCycle_BSW_s_AlgoParameters_Aln_b_AzimMonEnable = 0,
    DataProcCycle_BSW_s_AlgoParameters_Aln_b_ElevMonEnable,
    DataProcCycle_BSW_s_AlgoParameters_Aln_b_EolChannelCalibEnable,
    DataProcCycle_BSW_s_AlgoParameters_Aln_b_OnlineAngleDevCalibEnable,
    DataProcCycle_BSW_s_AlgoParameters_Aln_b_OnlineChannelCalibEnable,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimEolMax,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimMonMax,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimMonMaxDiff,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevEolMax,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevMonMax,
    DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevMonMaxDiff,
    DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_0,
    DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_1,
    DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_2,
    DataProcCycle_BSW_s_AlgoParameters_Envm_CEnvm_eCEnvmParCldState,
    DataProcCycle_BSW_s_AlgoParameters_Envm_CEnvm_eCEnvmParCodState,
    DataProcCycle_BSW_s_AlgoParameters_Envm_CEnvm_eCEnvmParModeSelection,
    DataProcCycle_BSW_s_AlgoParameters_Envm_Lat_b_UseExternalAxLat,
    DataProcCycle_BSW_s_AlgoParameters_Envm_Lat_b_UseExternalYawRateLat,
    DataProcCycle_BSW_s_AlgoParameters_Envm_Lat_u_AccelXObjLat,
    DataProcCycle_BSW_s_AlgoParameters_Envm_Lat_u_YawRateObjLat,
    DataProcCycle_BSW_s_AlgoParameters_Envm_Ped_ePedArtDummy,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Acc_CameraFusionPreselBits,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Acc_NaviFusionPreselBits,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageActive,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageUseTemperature,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageUseWiper,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_UseRoadbeam,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_BlockageHiTempThreshFactor,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffRange,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffTime_Timeout,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffWay_Timeout,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_i8_BlockageHiTempThresh,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_i8_BlockageLoTempThresh,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_BlockageSpeedThreshold,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_BlockageWiperThresh,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadbeamSpeedMin,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadQualityMin,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadVisibilityMin,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_UseFarNearScanForBlck,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui_ShutoffNoOfObjLosses,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingBits,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingValid,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Accel,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Velo,
    DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_Valid,
    DataProcCycle_BSW_s_AlgoParameters_Fct_General_FnSwitchBits,
    DataProcCycle_BSW_s_AlgoParameters_uiVersionNumber,
    AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamFmod,
    AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamGen,
    AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamValid,
    AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_eSigStatus,
    AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiCycleCounter,
    AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiMeasurementCounter,
    AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiTimeStamp,
    AlgoVehCycle_pCPAR_FCT_Parameters_uiVersionNumber,
    AlgoVehCycle_VehPar_Sensor_CoverDamping,
    AlgoVehCycle_VehPar_SensorMounting_LatPos,
    AlgoVehCycle_VehPar_SensorMounting_LongPos,
    AlgoVehCycle_VehPar_SensorMounting_Orientation,
    AlgoVehCycle_VehPar_SensorMounting_PitchAngle,
    AlgoVehCycle_VehPar_SensorMounting_VertPos,
    AlgoVehCycle_VehPar_SensorMounting_YawAngle,
    AlgoVehCycle_VehPar_SensorMounting_State_0,
    AlgoVehCycle_VehPar_SensorMounting_State_1,
    AlgoVehCycle_VehPar_SensorMounting_LongPosToCoG,
    AlgoVehCycle_VehPar_VehParMain_SelfSteerGrad,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_0,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_1,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_0,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_1,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_0,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_1,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_0,
    AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_1,
    // 20220106NANNAN new Front axle to front bumper
    AlgoVehCycle_VehPar_VehParMain_FrontAxle_to_FontBumper,
    AlgoVehCycle_VehPar_VehParMain_WheelBase,
    AlgoVehCycle_VehPar_VehParMain_TrackWidthFront,
    AlgoVehCycle_VehPar_VehParMain_TrackWidthRear,
    AlgoVehCycle_VehPar_VehParMain_CntrOfGravHeight,
    AlgoVehCycle_VehPar_VehParMain_WhlLoadDepFrontAxle,
    AlgoVehCycle_VehPar_VehParMain_WhlLoadDepRearAxle,
    AlgoVehCycle_VehPar_VehParMain_WhlCircumference,
    AlgoVehCycle_VehPar_VehParMain_WhlTcksPerRev,
    AlgoVehCycle_VehPar_VehParMain_FrCrnrStiff,
    AlgoVehCycle_VehPar_VehParMain_ReCrnrStiff,
    AlgoVehCycle_VehPar_VehParAdd_VehicleWidth,
    AlgoVehCycle_VehPar_VehParAdd_VehicleLength,
    AlgoVehCycle_VehPar_VehParAdd_OverhangFront,
    AlgoVehCycle_VehPar_VehParAdd_CurbWeight,
    AlgoVehCycle_VehPar_VehParAdd_FrontAxleRoadDist,
    AlgoVehCycle_VehPar_Sensor_fCycleTime,
    AlgoVehCycle_VehPar_Sensor_fCoverageAngle,
    AlgoVehCycle_VehPar_Sensor_fLobeAngle,
    AlgoVehCycle_VehPar_Sensor_uNoOfScans,
    AlgoVehCycle_DIMInputGeneric_bACCActive,
    AlgoVehCycle_DIMInputGeneric_eAccelPadelGradStat,
    AlgoVehCycle_DIMInputGeneric_eAccelPadelStat,
    AlgoVehCycle_DIMInputGeneric_eDriverBraking,
    AlgoVehCycle_DIMInputGeneric_eDriverSetting,
    AlgoVehCycle_DIMInputGeneric_eSteeringWheelAngleGradStat,
    AlgoVehCycle_DIMInputGeneric_eSteeringWheelAngleStat,
    AlgoVehCycle_DIMInputGeneric_eTurnIndicator,
    AlgoVehCycle_DIMInputGeneric_fAccelPedalGrad,
    AlgoVehCycle_DIMInputGeneric_fAccelPedalPos,
    AlgoVehCycle_DIMInputGeneric_fSteeringWheelAngle,
    AlgoVehCycle_DIMInputGeneric_fSteeringWheelAngleGrad,
    AlgoVehCycle_DIMInputGeneric_sSigHeader_eSigStatus,

    AlgoVehCycle_DIMInputGeneric_uiVersionNumber,
    AlgoVehCycle_DIMInputCustom_eSpeedLimitActive,
    AlgoVehCycle_DIMInputCustom_sDriverInput_bDriverOverride,
    AlgoVehCycle_DIMInputCustom_sSigHeader_eSigStatus,

    AlgoVehCycle_DIMInputCustom_sSigHeader_uiMeasurementCounter,
    AlgoVehCycle_DIMInputCustom_sSigHeader_uiTimeStamp,
    AlgoVehCycle_DIMInputCustom_uiVersionNumber,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ABS_Active,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ACC_IrreversibleFail,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ACC_ReversibleFail,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_AEB_Active,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_APA_Active,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_AYC_Active,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_Camera_Availability,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_Door_Open,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_EPB_Active,
    AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_TSC_Active,

    AlgoVehCycle_LongCtrlInput_DisplayOutput_speed_unit,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_acc_enable,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_brk_sw,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_country_code,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DAS_accel_request_limited,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DAS_decel_request_limited,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DC_status_information,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_door_state,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_driver_braking,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_driver_override_accel_pedal,

    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_ldm_ctrl_state,

    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_longi_shutoff_acknowledged,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_park_brk_eng,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_perm_lim_setspeed,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_seatbelt_state,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_stand_still_detected,
    AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_var_lim_eng,
    AlgoVehCycle_LongCtrlInput_sSigHeader_eSigStatus,
    AlgoVehCycle_LongCtrlInput_sSigHeader_uiCycleCounter,

    AlgoVehCycle_LongCtrlInput_uiVersionNumber,
    AlgoVehCycle_SADInputGeneric_eFunctionSwitch,
    AlgoVehCycle_SADInputGeneric_EnvmainSwitch,
    AlgoVehCycle_SADInputGeneric_eObjectSwitch,
    AlgoVehCycle_SADInputGeneric_sSigHeader_eSigStatus,

    AlgoVehCycle_SADInputGeneric_uiVersionNumber,
    AlgoVehCycle_SADInputCustom_eBuzzerState,
    AlgoVehCycle_SADInputCustom_sSigHeader_eSigStatus,

    AlgoVehCycle_SADInputCustom_sSigHeader_uiMeasurEnvmentCounter,
    AlgoVehCycle_SADInputCustom_sSigHeader_uiTimeStamp,
    AlgoVehCycle_SADInputCustom_ucCameraObjectStatus,
    AlgoVehCycle_SADInputCustom_ucCameraStatus,
    AlgoVehCycle_SADInputCustom_uiVersionNumber,
    Calibration_BUFFSIZE,
} Calibration_index;

extern double VehCalibration[Calibration_BUFFSIZE];
void Init_BSW_AlgoParameters(BSW_s_AlgoParameters_t *output);
void UNIF_InitVehicleParameter1(VEDVehPar_t *const pDestVehPar);
void Init_BSW_EnvmCtrlData(BSW_s_EnvmCtrlData_t *output);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
