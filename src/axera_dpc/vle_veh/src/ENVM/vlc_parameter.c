/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * huangchangchang <huangchangchang@senseauto.com>
 * wangsifan <wangsifan@senseauto.com>
 */
#include "vlc_parameter.h"
#include "string.h"
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
double VehCalibration[Calibration_BUFFSIZE] = {
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Aln_b_AzimMonEnable */
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Aln_b_ElevMonEnable */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Aln_b_EolChannelCalibEnable */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Aln_b_OnlineAngleDevCalibEnable
          */
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Aln_b_OnlineChannelCalibEnable */
    0.10471976, /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimEolMax */
    0.10471976, /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimMonMax */
    0.2617994,  /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_AzimMonMaxDiff */
    0.06981317, /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevEolMax */
    0.10471976, /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevMonMax */
    0.2617994,  /* DataProcCycle_BSW_s_AlgoParameters_Aln_f_ElevMonMaxDiff */
    0.0,        /* DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_0 */
    0.0,        /* DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_1 */
    0.0,        /* DataProcCycle_BSW_s_AlgoParameters_Aln_u_Dummy_2 */
    1.0,        /* DataProcCycle_BSW_s_AlgoParameters_Em_Cem_eCEMParCldState */
    1.0,        /* DataProcCycle_BSW_s_AlgoParameters_Em_Cem_eCEMParCodState */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Cem_eCEMParModeSelection */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Lat_b_UseExternalAxLat */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Lat_b_UseExternalYawRateLat */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Lat_u_AccelXObjLat */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Lat_u_YawRateObjLat */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Em_Ped_ePedArtDummy */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Acc_CameraFusionPreselBits */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Acc_NaviFusionPreselBits */
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageActive */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageUseTemperature
          */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_BlockageUseWiper
          */
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_b_UseRoadbeam */
    5.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_BlockageHiTempThreshFactor
          */
    80.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffRange */
    90.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffTime_Timeout
           */
    2000.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_f32_ShutoffWay_Timeout
             */
    -10.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_i8_BlockageHiTempThresh
            */
    15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_i8_BlockageLoTempThresh
           */
    43.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_BlockageSpeedThreshold
           */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_BlockageWiperThresh
          */
    10.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadbeamSpeedMin
           */
    40.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadQualityMin
           */
    80.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_RoadVisibilityMin
           */
    1.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui8_UseFarNearScanForBlck
          */
    3.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Blockage_ui_ShutoffNoOfObjLosses
          */
    39.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingBits */
    0.0,  /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingValid */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Velo
          */
    -15.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Accel
            */
    0.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Velo
          */
    0.0,  /* DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_Valid
           */
    32.0, /* DataProcCycle_BSW_s_AlgoParameters_Fct_General_FnSwitchBits */
    32.0, /* DataProcCycle_BSW_s_AlgoParameters_uiVersionNumber */
    3661499.0, /* AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamFmod */
    11.0,      /* AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamGen */
    1.0,       /* AlgoVehCycle_pCPAR_FCT_Parameters_EBA_EBACodingParamValid */
    1.0,       /* AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_eSigStatus */
    25350.0,   /* AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiCycleCounter */
    15725.0, /* AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiMeasurementCounter
              */
    4215008906.0, /* AlgoVehCycle_pCPAR_FCT_Parameters_sSigHeader_uiTimeStamp */
    12.0,         /* AlgoVehCycle_pCPAR_FCT_Parameters_uiVersionNumber */
    2.1099999,    /* AlgoVehCycle_VehPar_Sensor_CoverDamping */
    0.0055,       /* AlgoVehCycle_VehPar_SensorMounting_LatPos */
    0.82599998,   /* AlgoVehCycle_VehPar_SensorMounting_LongPos */
    1.0,          /* AlgoVehCycle_VehPar_SensorMounting_Orientation */
    0.52399999,   /* AlgoVehCycle_VehPar_SensorMounting_PitchAngle */
    0.412,        /* AlgoVehCycle_VehPar_SensorMounting_VertPos */
    0.0,          /* AlgoVehCycle_VehPar_SensorMounting_YawAngle */
    1.0,          /* AlgoVehCycle_VehPar_SensorMounting_State_0 */
    1.0,          /* AlgoVehCycle_VehPar_SensorMounting_State_1 */
    2.03399992,   /* AlgoVehCycle_VehPar_SensorMounting_LongPosToCoG */
    0.003734,     /* AlgoVehCycle_VehPar_VehParMain_SelfSteerGrad */
    0.0,          /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_0 */
    1000.0,       /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_1 */
    14.52000046,  /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_0 */
    14.52000046,  /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_1 */
    0.0,          /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_0 */
    0.0,          /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_1 */
    14.52000046,  /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_0 */
    14.52000046,  /* AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_1 */
    // 20220106NANNAN new Front axle to front bumper
    0.5948,       /* AlgoVehCycle_VehPar_VehParMain_FrontAxle_to_FontBumper */
    2.96,         /* AlgoVehCycle_VehPar_VehParMain_WheelBase */
    1.696,        /* AlgoVehCycle_VehPar_VehParMain_TrackWidthFront */
    1.695,        /* AlgoVehCycle_VehPar_VehParMain_TrackWidthRear */
    0.63422,      /* AlgoVehCycle_VehPar_VehParMain_CntrOfGravHeight */
    3.6329999,    /* AlgoVehCycle_VehPar_VehParMain_WhlLoadDepFrontAxle */
    2.92000008,   /* AlgoVehCycle_VehPar_VehParMain_WhlLoadDepRearAxle */
    2.16770005,   /* AlgoVehCycle_VehPar_VehParMain_WhlCircumference */
    96.0,         /* AlgoVehCycle_VehPar_VehParMain_WhlTcksPerRev */
    192227.34375, /* AlgoVehCycle_VehPar_VehParMain_FrCrnrStiff */
    139744.40625, /* AlgoVehCycle_VehPar_VehParMain_ReCrnrStiff */
    1.98,         /* AlgoVehCycle_VehPar_VehParAdd_VehicleWidth */
    4.95,         /* AlgoVehCycle_VehPar_VehParAdd_VehicleLength */
    0.5948,       /* AlgoVehCycle_VehPar_VehParAdd_OverhangFront */
    1817.0,       /* AlgoVehCycle_VehPar_VehParAdd_CurbWeight */
    0.33700001,   /* AlgoVehCycle_VehPar_VehParAdd_FrontAxleRoadDist */
    90.0,         /* AlgoVehCycle_VehPar_Sensor_fCycleTime */
    20.0,         /* AlgoVehCycle_VehPar_Sensor_fCoverageAngle */
    8.0,          /* AlgoVehCycle_VehPar_Sensor_fLobeAngle */
    2.0,          /* AlgoVehCycle_VehPar_Sensor_uNoOfScans */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_bACCActive */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eAccelPadelGradStat */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eAccelPadelStat */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eDriverBraking */
    3.0,          /* AlgoVehCycle_DIMInputGeneric_eDriverSetting */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eSteeringWheelAngleGradStat */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eSteeringWheelAngleStat */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_eTurnIndicator */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_fAccelPedalGrad */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_fAccelPedalPos */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_fSteeringWheelAngle */
    0.0,          /* AlgoVehCycle_DIMInputGeneric_fSteeringWheelAngleGrad */
    1.0,          /* AlgoVehCycle_DIMInputGeneric_sSigHeader_eSigStatus */

    4.0, /* AlgoVehCycle_DIMInputGeneric_uiVersionNumber */
    0.0, /* AlgoVehCycle_DIMInputCustom_eSpeedLimitActive */
    0.0, /* AlgoVehCycle_DIMInputCustom_sDriverInput_bDriverOverride */
    0.0, /* AlgoVehCycle_DIMInputCustom_sSigHeader_eSigStatus */

    0.0, /* AlgoVehCycle_DIMInputCustom_sSigHeader_uiMeasurementCounter */
    0.0, /* AlgoVehCycle_DIMInputCustom_sSigHeader_uiTimeStamp */
    8.0, /* AlgoVehCycle_DIMInputCustom_uiVersionNumber */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ABS_Active
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ACC_IrreversibleFail
          */
    1.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_ACC_ReversibleFail
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_AEB_Active
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_APA_Active
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_AYC_Active
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_Camera_Availability
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_Door_Open
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_EPB_Active
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_Custom_LongCtrlInputCustomSGMWIn_TSC_Active
          */

    0.0, /* AlgoVehCycle_LongCtrlInput_DisplayOutput_speed_unit */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_acc_enable */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_brk_sw */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_country_code */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DAS_accel_request_limited
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DAS_decel_request_limited
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_DC_status_information */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_door_state */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_driver_braking */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_driver_override_accel_pedal
          */
    // 0.0,			 /*
    // AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_driver_override_decel_pedal */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_ldm_ctrl_state */
    // 930.0,			 /*
    // AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_longi_initialization_accel */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_longi_shutoff_acknowledged
          */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_park_brk_eng */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_perm_lim_setspeed */
    1.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_seatbelt_state */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_stand_still_detected */
    0.0, /* AlgoVehCycle_LongCtrlInput_KinCtrlDynInput_var_lim_eng */
    1.0, /* AlgoVehCycle_LongCtrlInput_sSigHeader_eSigStatus */
    0.0, /* AlgoVehCycle_LongCtrlInput_sSigHeader_uiCycleCounter */

    21.0,      /* AlgoVehCycle_LongCtrlInput_uiVersionNumber */
    4112063.0, /* AlgoVehCycle_HEADInputGeneric_eFunctionSwitch */
    0.0,       /* AlgoVehCycle_HEADInputGeneric_eMainSwitch */
    7.0,       /* AlgoVehCycle_HEADInputGeneric_eObjectSwitch */
    1.0,       /* AlgoVehCycle_HEADInputGeneric_sSigHeader_eSigStatus */

    5.0, /* AlgoVehCycle_HEADInputGeneric_uiVersionNumber */
    0.0, /* AlgoVehCycle_HEADInputCustom_eBuzzerState */
    0.0, /* AlgoVehCycle_HEADInputCustom_sSigHeader_eSigStatus */

    0.0, /* AlgoVehCycle_HEADInputCustom_sSigHeader_uiMeasurementCounter */
    0.0, /* AlgoVehCycle_HEADInputCustom_sSigHeader_uiTimeStamp */
    0.0, /* AlgoVehCycle_HEADInputCustom_ucCameraObjectStatus */
    0.0, /* AlgoVehCycle_HEADInputCustom_ucCameraStatus */
    9.0, /* AlgoVehCycle_HEADInputCustom_uiVersionNumber */
};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void Init_BSW_AlgoParameters(BSW_s_AlgoParameters_t *output) {
    output->Fct.Eba.CodingBits =
        VehCalibration[DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingBits];
    output->Fct.Eba.CodingValid =
        VehCalibration[DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_CodingValid];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[0].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[0].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_0_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[1].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[1].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_1_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[2].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[2].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_2_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[3].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL1[3].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL1_3_Velo];

    output->Fct.Eba.PreBrkParAccelTab.AccelL2[0].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[0].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_0_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[1].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[1].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_1_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[2].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[2].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_2_Velo];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[3].Accel = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Accel];
    output->Fct.Eba.PreBrkParAccelTab.AccelL2[3].Velo = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_AccelL2_3_Velo];
    output->Fct.Eba.PreBrkParAccelTab.Valid = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_Eba_PreBrkParAccelTab_Valid];
    output->Fct.General.FnSwitchBits = VehCalibration
        [DataProcCycle_BSW_s_AlgoParameters_Fct_General_FnSwitchBits];

    output->uiVersionNumber =
        VehCalibration[DataProcCycle_BSW_s_AlgoParameters_uiVersionNumber];
}

void UNIF_InitVehicleParameter1(VEDVehPar_t *const pDestVehPar) {
    static uint32 VehPar_uiCycleCounter = 0;
    static uint32 VehPar_uiMeasurementCounter = 0;

    // NORM_ui8_EquippedWithFLC = RADAR_EQ_WITH_FLC;

    if (pDestVehPar != NULL) {
        VEDVehParMain_t *p_VehParMain = &(pDestVehPar->VehParMain);
        pDestVehPar->Sensor.CoverDamping =
            VehCalibration[AlgoVehCycle_VehPar_Sensor_CoverDamping];

        pDestVehPar->SensorMounting.LatPos =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_LatPos];
        pDestVehPar->SensorMounting.LongPos =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_LongPos];

        pDestVehPar->SensorMounting.Orientation =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_Orientation];

        pDestVehPar->SensorMounting.PitchAngle =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_PitchAngle];
        pDestVehPar->SensorMounting.VertPos =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_VertPos];
        pDestVehPar->SensorMounting.YawAngle =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_YawAngle];
        pDestVehPar->SensorMounting.State[0] =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_State_0];
        pDestVehPar->SensorMounting.State[1] =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_State_1];
        pDestVehPar->SensorMounting.LongPosToCoG =
            VehCalibration[AlgoVehCycle_VehPar_SensorMounting_LongPosToCoG];

        //(void)memset(&p_VehParMain->State, 0xFF,
        //(uint16)sizeof(p_VehParMain->State));

        VED_SET_IO_STATE(VED_PAR_POS_SSG, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_SWARAT, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_WBASE, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_TWDFR, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_TWDRE, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_VEHWGT, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_COGH, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_AXLD, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_WHLDFR, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_WHLDRE, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_WHLCIR, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_WTCKSREV, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_CSFR, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        VED_SET_IO_STATE(VED_PAR_POS_CSRE, VED_IO_STATE_VALID,
                         p_VehParMain->State);
        p_VehParMain->SelfSteerGrad =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_SelfSteerGrad];
        p_VehParMain->SteeringRatio.swa.ang[0] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_0];
        p_VehParMain->SteeringRatio.swa.ang[1] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_ang_1];
        p_VehParMain->SteeringRatio.swa.rat[0] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_0];
        p_VehParMain->SteeringRatio.swa.rat[1] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_swa_rat_1];
        p_VehParMain->SteeringRatio.vel.vel[0] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_0];
        p_VehParMain->SteeringRatio.vel.vel[1] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_vel_1];
        p_VehParMain->SteeringRatio.vel.rat[0] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_0];
        p_VehParMain->SteeringRatio.vel.rat[1] = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_SteeringRatio_vel_rat_1];
        // 20220106NANNAN new Front axle to front bumper
        p_VehParMain->DIST_FrontAxle_to_FontBumper = VehCalibration
            [AlgoVehCycle_VehPar_VehParMain_FrontAxle_to_FontBumper];
        p_VehParMain->WheelBase =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_WheelBase];
        p_VehParMain->TrackWidthFront =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_TrackWidthFront];
        p_VehParMain->TrackWidthRear =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_TrackWidthRear];
        p_VehParMain->CntrOfGravHeight =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_CntrOfGravHeight];
        p_VehParMain->WhlLoadDepFrontAxle =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_WhlLoadDepFrontAxle];
        p_VehParMain->WhlLoadDepRearAxle =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_WhlLoadDepRearAxle];
        p_VehParMain->WhlCircumference =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_WhlCircumference];
        p_VehParMain->WhlTcksPerRev =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_WhlTcksPerRev];
        p_VehParMain->FrCrnrStiff =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_FrCrnrStiff];
        p_VehParMain->ReCrnrStiff =
            VehCalibration[AlgoVehCycle_VehPar_VehParMain_ReCrnrStiff];
        pDestVehPar->VehParAdd.VehicleWidth =
            VehCalibration[AlgoVehCycle_VehPar_VehParAdd_VehicleWidth];
        pDestVehPar->VehParAdd.VehicleLength =
            VehCalibration[AlgoVehCycle_VehPar_VehParAdd_VehicleLength];
        pDestVehPar->VehParAdd.OverhangFront =
            VehCalibration[AlgoVehCycle_VehPar_VehParAdd_OverhangFront];
        pDestVehPar->VehParAdd.CurbWeight =
            VehCalibration[AlgoVehCycle_VehPar_VehParAdd_CurbWeight];
        pDestVehPar->VehParAdd.FrontAxleRoadDist =
            VehCalibration[AlgoVehCycle_VehPar_VehParAdd_FrontAxleRoadDist];
        pDestVehPar->Sensor.fCycleTime =
            VehCalibration[AlgoVehCycle_VehPar_Sensor_fCycleTime];
        pDestVehPar->Sensor.fCoverageAngle =
            VehCalibration[AlgoVehCycle_VehPar_Sensor_fCoverageAngle];
        pDestVehPar->Sensor.fLobeAngle =
            VehCalibration[AlgoVehCycle_VehPar_Sensor_fLobeAngle];
        pDestVehPar->Sensor.uNoOfScans =
            VehCalibration[AlgoVehCycle_VehPar_Sensor_uNoOfScans];

        pDestVehPar->sSigHeader.eSigStatus = 1;
        pDestVehPar->sSigHeader.uiCycleCounter = VehPar_uiCycleCounter;
        pDestVehPar->sSigHeader.uiMeasurementCounter =
            VehPar_uiMeasurementCounter;

        // pDestVehPar->sSigHeader.uiTimeStamp =
        // Rte_SWCVEDAdapt_RPortVehSig_DEPVehSig_Buf.sSigHeader.uiTimeStamp;
        VehPar_uiCycleCounter++;
        VehPar_uiMeasurementCounter++;
    }
}

void Init_BSW_EnvmCtrlData(BSW_s_EnvmCtrlData_t *output) {
    output->u_VersionNumber = 0;

    output->sSigHeader.eSigStatus = 1;
    output->sSigHeader.uiCycleCounter = 0;
    output->sSigHeader.uiMeasurementCounter = 0;
    output->sSigHeader.uiTimeStamp = 0;

    output->HWSample = 0;
    output->EnvmOpMode = 0;
    output->RSPCycleViolation = FALSE;
    output->EnvmCycleViolation = FALSE;
    output->uiTimeStamp_us = 0;
    output->u_Dummy81 = 0;
    output->u_Dummy82 = 0;
    output->u_Dummy83 = 0;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */