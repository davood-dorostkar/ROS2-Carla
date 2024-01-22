// "Copyright [2022] <Copyright Senseauto>" [legal/copyright]
#pragma once
#ifndef SRC_DMC_MODULE_LCF_DMC_LCF_DMC_LOCAL_EXT_H_
#define SRC_DMC_MODULE_LCF_DMC_LCF_DMC_LOCAL_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
// #include "TM_Global_Types.h"
#include "./LaDMC/TJADMC_ext.h"
#include "tue_common_libs.h"
#include "Rte_Type.h"  // RENDL
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#ifndef Rte_TypeDef_LCF_DmcSignalHeader_t
typedef struct {
    uint32_T uiTimeStamp;
    uint16_T uiMeasurementCounter;
    uint16_T uiCycleCounter;
    uint8_T eSigStatus;
} LCF_DmcSignalHeader_t; /* Common header for all structured data types */
#define Rte_TypeDef_LCF_DmcSignalHeader_t
#endif

#ifndef Rte_TypeDef_LCF_DmcAlgoCompState_t
typedef struct {
    uint32_T uiVersionNumber;
    LCF_DmcSignalHeader_t sSigHeader;
    uint32_T uiAlgoVersionNumber;
    uint8_T eCompState;
    uint32_T eGenAlgoQualifier;
} LCF_DmcAlgoCompState_t;
#define Rte_TypeDef_LCF_DmcAlgoCompState_t
#endif

#ifndef Rte_TypeDef_LCF_Dmc_BaseCtrlData_t
typedef struct {
    uint32_T uiVersionNumber;
    LCF_DmcSignalHeader_t sSigHeader;
    uint16_T eOpMode;
    // uint8_T eSchedulingMode;
    // uint8_T eSchedulingSubMode;
} LCF_Dmc_BaseCtrlData_t;
#define Rte_TypeDef_LCF_Dmc_BaseCtrlData_t
#endif

#ifndef Rte_TypeDef_LCF_Dmc_VehDyn_t
typedef struct {
    LCF_DmcSignalHeader_t sSigHeader;
    real32_T fEgoVelX_mps;
    real32_T fEgoCurve_1pm;
    real32_T fEgoAccelX_mps2;
    real32_T fEgoYawRate_rps;
    real32_T fWhlSteerAngleVdy_rad;
    real32_T fEstSelfSteerGrdnt_rads2pm;
    real32_T fSteerWheelAgl_rad;
    real32_T fManuActuTrqEPS_nm;    /* Actual manual torque of EPS */
    real32_T fSteerWhlAglspd_rads;  // Steer wheel angle speed
} LCF_Dmc_VehDyn_t;
#define Rte_TypeDef_LCF_Dmc_VehDyn_t
#endif

#ifndef Rte_TypeDef_LCFDMC_CSCLTAData_t
typedef struct {
    LCF_DmcSignalHeader_t sSigHeader;
    uint8_T uiControllingFunction_nu;  // function type of lateral controlling,
                                       // [0,7], E_LCF_OFF_nu= 0,E_LCF_TJA_nu=
                                       // 1,E_LCF_LDP_nu= 2,E_LCF_LDPOC_nu=
                                       // 3,E_LCF_RDP_nu= 4,E_LCF_ALCA_nu=
                                       // 5,E_LCF_AOLC_nu= 6, E_LCF_ESA_nu= 7
    real32_T fPlanningHorizon_sec;     // max Planning horizon(time) of the
                                       // trajectory, [0，60]
    uint8_T uiSysStateLCF_nu;  // lateral control function system state enum
                               // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                               // E_LCF_SYSSTATE_DISABLED = 1;
                               // E_LCF_SYSSTATE_PASSIVE = 2;
                               // E_LCF_SYSSTATE_REQUEST = 3;
                               // E_LCF_SYSSTATE_CONTROLLING = 4;
                               // E_LCF_SYSSTATE_RAMPOUT = 5;
                               // E_LCF_SYSSTATE_ERROR = 6; \n\n
    boolean_T
        bTriggerReplan;  // trigger replan signal from CSCLTA module,[0，1]
    real32_T fTgtTrajPosX0_met;  // the PosX0 value of target corridor bound,
                                 // [-300, 300]
    real32_T fTgtTrajPosY0_met;  // the PosY0 value of target corridor bound,
                                 // [-15, 15]
    real32_T fTgtTrajHeadingAng_rad;  // the heading angle value of target
                                      // corridor bound, [-0.78539816,
                                      // 0.78539816]
    real32_T fTgtTrajCurve_1pm;     // the curve value of target corridor bound,
                                    // [-0.1, 0.1]
    real32_T fTgtTrajCrvChng_1pm2;  // the curve deviation value of target
                                    // corridor bound, [-0.001, 0.001]
    real32_T fTgtTrajLength_met;  // the length value of target corridor bound,
                                  // [0, 150]
    boolean_T
        bLatencyCompActivated;      // the trigger flag for latency compensation
                                    // function, [0,1] 1: latency compensation
                                    // enable, 0: latency compensation disable
    real32_T fSensorTimeStamp_sec;  // time stamp of the camera signal from
                                    // camera sensor,[0,4295]
    uint8_T uiTrajPlanServQu_nu;    // todo
} LCFDMC_CSCLTAData_t;
#define Rte_TypeDef_LCFDMC_CSCLTAData_t
#endif

#ifndef Rte_TypeDef_LCFDMC_LBPData_t
typedef struct {
    LCF_DmcSignalHeader_t sSigHeader;
    uint8_T uiLeLnQuality_per;   // quality of left lane, [0, 100]
    uint8_T uiRiLnQuality_per;   // quality of right lane, [0, 100]
    uint8_T uiLeCrvQuality_per;  // quality of left lane curve, [0, 100]
    uint8_T uiRiCrvQuality_per;  // quality of right lane curve, [0, 100]
    real32_T
        fPosX0CtrlCntr;  // Center lane clothoid X0 position, (0, -300~300, m)
    real32_T fPosY0CtrlCntr;    // Center lane clothoid Y0 position (init +10m),
                                // (0, -15~15, m)
    real32_T fHeadingCtrlCntr;  // Center lane clothoid heading angle, (0,
                                // -0.7854~0.7854, rad)
    real32_T fCrvCtrlCntr;  // Center lane clothoid curvature, (0, -0.1~0.1, 1/m
    real32_T fCrvRateCtrlCntr;  // Center lane clothoid change of curvature, (0,
                                // -0.001~0.001, 1/m^2)
} LCFDMC_LBPData_t;
#define Rte_TypeDef_LCFDMC_LBPData_t
#endif

#ifndef Rte_TypeDef_LCFDMC_TJASAState_t
typedef struct {
    LCF_DmcSignalHeader_t sSigHeader;
    uint8_T uiLatCtrlMode_nu;  // lateral control mode, [0,8]
                               // ,E_TJASTM_LATCTRLMD_PASSIVE=
                               // 0,E_TJASTM_LATCTRLMD_LC=
                               // 1,E_TJASTM_LATCTRLMD_OF=
                               // 2,E_TJASTM_LATCTRLMD_CMB=
                               // 3,E_TJASTM_LATCTRLMD_SALC=
                               // 4,E_TJASTM_LATCTRLMD_LC_RQ=
                               // 5,E_TJASTM_LATCTRLMD_OF_RQ=
                               // 6,E_TJASTM_LATCTRLMD_CMB_RQ= 7,
                               // E_TJASTM_LATCTRLMD_SALC_RQ= 8
} LCFDMC_TJASAState_t;
#define Rte_TypeDef_LCFDMC_TJASAState_t
#endif

#ifndef Rte_TypeDef_LCF_TJADMCOutPro_t
typedef struct {
    real32_T fSteerAngle_deg;
    real32_T fSteerWhlAngle_deg;
    real32_T fLonAccCmd;
    uint8_T uiEPSRequest_nu;
    uint8_T uiLonAccRequest_nu;
    real32_T fSteerWhlTqAdd_nm;    // Reserved for DMC2
    real32_T fDampingLevelRmp_nu;  // Reserved for DMC2
} LCF_TJADMCOutPro_t;
#define Rte_TypeDef_LCF_TJADMCOutPro_t
#endif

#ifndef Rte_TypeDef_LCFDMC_LDPSAData_t
typedef struct {
    uint8_T LDWC_DgrSide_St;
    uint8_T LDWC_RdyToTrig_B;
    uint8_T LDWC_SysOut_St;
    uint8_T LDWC_StateLf_St;
    uint8_T LDWC_StateRi_St;
    uint8_T LDPSC_DgrSide_St;
    uint8_T LDPSC_RdyToTrig_B;
    uint8_T LDPSC_SysOut_St;
    uint8_T LDPSC_StateLf_St;
    uint8_T LDPSC_StateRi_St;
    real32_T LDPDT_LnPstnLf_Mi;
    real32_T LDPDT_LnPstnRi_Mi;
    real32_T LDPDT_LnHeadLf_Rad;
    real32_T LDPDT_LnHeadRi_Rad;
    real32_T LDPDT_LnCltdCurvLf_ReMi;
    real32_T LDPDT_LnCltdCurvRi_ReMi;
} LCFDMC_LDPSAData_t;
#define Rte_TypeDef_LCFDMC_LDPSAData_t
#endif

#ifndef Rte_TypeDef_LCFDMC_TPData_t
typedef struct {
    boolean_T bReplanCurValues;     // replan mode with curvature switch
    uint8_T uiTrajGuiQualifier_nu;  // qualifier value of trajectory guidence,
                                    // [0,5] .E_LCF_TGQ_REQ_OFF= 0,
                                    // E_LCF_TGQ_REQ = 1, E_LCF_TGQ_REQ_FREEZE=
                                    // E_LCF_TGQ_REQ_REFCHNG= 5
    real32_T
        fTrajDistY_met;  // Lateral deviation of planned trajectory, [-100,100]
    real32_T fTrajTgtCurve_1pm;  // Target Curvature for kinematic controller,
                                 // [-0.1,0.1]
    real32_T fTrajHeadInclPrev_rad;  // Target heading angle for kinematic
                                     // controller under consideration of
                                     // preview time, [-6,6]
    real32_T
        fTrajHeading_rad;  // Target heading angle for kinematic controller,
                           // [-6,6]
    real32_T fTrajTgtCrvGrd_1pms;  // Target Curvature gradient for kinematic
                                   // controller, [-0.1,0.1]
} LCFDMC_TPData_t;
#define Rte_TypeDef_LCFDMC_TPData_t
#endif

#ifndef Rte_TypeDef_LCFDMC_TCData_t
typedef struct {
    real32_T TJATCT_DeltaFCmd;
    real32_T TJATCT_ReqFfcCrv;
    real32_T TJATCT_SumCtrlCrv;
    real32_T TJATCT_ReqFbcDcCrv;
    real32_T CDC_CtrlErrDistY;
    real32_T CDC_CtrlErrHeading;
    boolean_T LGC_EnableCtrl_nu;
} LCFDMC_TCData_t;
#define Rte_TypeDef_LCFDMC_TCData_t
#endif

#ifndef Rte_TypeDef_LCF_SenVehToDmc_t
typedef struct {
    LCFDMC_LDPSAData_t sLDPSAData;
    LCFDMC_LBPData_t sLBPData;  // the input data of lcf dmc from LBP(sen)
    LCFDMC_TJASAState_t
        sTJASAState;  // the input data of lcf dmc from TJASA(sen)
    LCFDMC_TPData_t sTPData;
    LCFDMC_TCData_t sTCData;
} LCF_SenVehToDmc_t;
#define Rte_TypeDef_LCF_SenVehToDmc_t
#endif

#ifndef Rte_TypeDef_LCF_DmcNVRAMData_t
typedef struct {
    boolean_T LCFVeh_Nb_SwitchReserved1_nu;   // Power off switch
    boolean_T LCFVeh_Nb_SwitchReserved2_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved3_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved4_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved5_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved6_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved7_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved8_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved9_nu;   // Reserved
    boolean_T LCFVeh_Nb_SwitchReserved10_nu;  // Reserved

    uint8_T LCFVeh_Nu_SensitivityReserved1_nu;  // sensitivity
    uint8_T LCFVeh_Nu_SensitivityReserved2_nu;  // Reserved
    uint8_T LCFVeh_Nu_SensitivityReserved3_nu;  // Reserved
    uint8_T LCFVeh_Nu_SensitivityReserved4_nu;  // Reserved
    uint8_T LCFVeh_Nu_SensitivityReserved5_nu;  // Reserved

    float32 LCFVeh_Nf_SensitivityReserved1_nu;
    float32 LCFVeh_Nf_SensitivityReserved2_nu;
    float32 LCFVeh_Nf_SensitivityReserved3_nu;
    float32 LCFVeh_Nf_SensitivityReserved4_nu;
    float32 LCFVeh_Nf_SensitivityReserved5_nu;
    float32 LCFVeh_Nf_SensitivityReserved6_nu;
    float32 LCFVeh_Nf_SensitivityReserved7_nu;
    float32 LCFVeh_Nf_SensitivityReserved8_nu;
    float32 LCFVeh_Nf_SensitivityReserved9_nu;
    float32 LCFVeh_Nf_SensitivityReserved10_nu;
} LCF_DmcNVRAMData_t;
#define Rte_TypeDef_LCF_DmcNVRAMData_t
#endif

#ifndef Rte_TypeDef_LCF_DmcCalibration
typedef struct {
    uint16_T Dmc_configuration_mode;
    uint8_T Dmc_offset_calibration_mode_par;
    real32_T Dmc_DeltaF_offset_par;
    real32_T Lat_oc_kappa_cmd_filter_coeff_par;
    real32_T Lat_oc_max_kappa_dys_par;
    real32_T Lat_oc_ki_par;
    real32_T Lat_oc_delta_off_flt_initial_omega_par;
    real32_T Lat_oc_delta_offset_filter_omega_par;
    real32_T Lat_oc_max_driver_torque_par;
    real32_T Dyc_state_filter_time_constant;
    uint16_T Dyc_compensation_mode;
    real32_T Dyc_kappa_a2_pos_corr_factor;
    real32_T Dyc_kappa_a2_x_scheduling[12];
    real32_T Dyc_kappa_a2_y_scheduling[12];
    real32_T Dyc_kappa_a2_factor;
    real32_T Dyc_kappa_angle_gen_corr_factor;
    real32_T Dyc_kappa_angle_t_x_schedul_gen[12];
    real32_T Dyc_kappa_angle_t_y_schedul_gen[12];
    real32_T Dyc_kappa_angle_gen_cor_fct_neg;
    real32_T Dyc_kappa_angle_t_y_sch_gen_neg[12];
    real32_T Lat_max_ay;
    uint32_T Sac_controller_mode_1;
    real32_T Sat_max_cmd_factor_x_scheduling[13];
    real32_T Sat_max_cmd_factor_y_scheduling[13];
    real32_T Sat_max_delta_f_cmd;
    real32_T VEH_Vehicle_Selfsteering_Factor;
    real32_T Sat_max_angle_offset_x[4];
    real32_T Sat_max_angle_offset_y[4];
    real32_T Sat_max_angle_low_bound_x_sched[3];
    real32_T Sat_max_angle_low_bound_y_sched[3];
    real32_T Tdf_min_steer_torque_class;
    uint16_T Tdf_derating_mode;
    real32_T Tdf_torque_request_sign_slope;
    real32_T Tdf_steer_torque_sign_slope;
    real32_T Tdf_torque_derating_threshold;
    real32_T Tdf_trq_derating_threshold_dp;
    real32_T Tdf_trq_der_thrs_dp_hi_sens;
    real32_T Tdf_torque_derating_slope_ldp;
    real32_T Tdf_torque_derating_slope;
    real32_T TDF_Derating_Switch_Thresh2;
    real32_T TDF_Derating_Switch_Thresh1;
    real32_T Sac_delta_f_cmd_grad_barrier;
    real32_T Sac_delta_f_cmd_min_grad;
} LCF_DmcCalibration;
#define Rte_TypeDef_LCF_DmcCalibration
#endif

#ifndef Rte_TypeDef_LCF_NOPToDmc_t
typedef struct {
    real32_T NOP_Lon_Acc_Cmd;
    real32_T NOP_Delta_F_Cmd;
    boolean_T NOP_Lon_Acc_Enable_flag;
    boolean_T NOP_Delta_F_Enable_flag;
} LCF_NOPToDmc_t;
#define Rte_TypeDef_LCF_NOPToDmc_t
#endif

#ifndef Rte_TypeDef_LCF_VLCToDmc_t
typedef struct {
    real32_T ADAS_Lon_Acc_Cmd;
    real32_T ADAS_Lon_Acc_Enable_flag;
} LCF_VLCToDmc_t;
#define Rte_TypeDef_LCF_VLCToDmc_t
#endif

#ifndef Rte_TypeDef_reqLcfDmcPrtList_t
typedef struct {
    LCF_Dmc_BaseCtrlData_t sBaseCtrlData;  // Control data giving information
                                           // about the current mode
    LCF_Dmc_VehDyn_t sLCFVehDyn;           // Vehicle dynamic data (VDY output)
    LCF_SenVehToDmc_t
        sLcfInputFromSenVehData;  // the input data of lcf veh from lcf sen
    LCF_NOPToDmc_t sLCFInputFromNOPData;
    LCF_VLCToDmc_t sLCFInputFromVLCData;
    LCF_DmcNVRAMData_t sNVRAMData;  // NVRAM Data
    LCF_DmcCalibration sLCF_DmcCalibrations;
    boolean DMC_NVRAMReset;
} reqLcfDmcPrtList_t;
#define Rte_TypeDef_reqLcfDmcPrtList_t
#endif

#ifndef Rte_TypeDef_reqLcfDmcParams
typedef struct {
    real32_T LCFVeh_Kf_fSysCycleTime_sec;
    real32_T LCFVeh_Kf_fEgoVehWidth_met;
    real32_T LCFVeh_Kf_fEgoVehLength_met;
    real32_T LCFVeh_Kf_CrnStiffFr;
    real32_T LCFVeh_Kf_CrnStiffRe;
    real32_T LCFVeh_Kf_DistToFrontAxle_met;
    real32_T LCFVeh_Kf_DistToRearAxle_met;
    real32_T LCFVeh_Kf_Mass_kg;
    real32_T LCFVeh_Kf_SelfStrGrad;
    real32_T LCFVeh_Kf_SteeringRatio_nu;
} reqLcfDmcParams;
#define Rte_TypeDef_reqLcfDmcParams
#endif

#ifndef Rte_TypeDef_LCF_DmcGenericOutputs_t
typedef struct { /* [Satisfies_rte sws 1191] */
    uint32_T uiVersionNumber;
    LCF_DmcSignalHeader_t sSigHeader;
    LCF_TJADMCOutPro_t sDMCOutput;
    // sTJADMCDebug_t sDMCDebug;
} LCF_DmcGenericOutputs_t;
#define Rte_TypeDef_LCF_DmcGenericOutputs_t
#endif

#ifndef Rte_TypeDef_proLcfDmcPrtList_t
typedef struct {
    LCF_DmcAlgoCompState_t
        pAlgoCompState;  //!< State return values of the algo component
    LCF_DmcGenericOutputs_t pLcfDmcOutputData;  //!<
    LCF_DmcNVRAMData_t sNVRAMData;              // NVRAM Data
} proLcfDmcPrtList_t;
#define Rte_TypeDef_proLcfDmcPrtList_t
#endif

#ifndef Rte_TypeDef_proLcfDmcDebug_t
typedef struct {
    real32_T ADP_Dyc_Corr_Factor;
    real32_T CAM_Lateral_Error_Sign;
    real32_T DYC_Filter_Kappa_Command;
    real32_T DYC_Steer_Angle_Feedforward;
    real32_T HEC_Yaw_Rate_Filter;
    boolean_T Initialisation_Flag;
    real32_T LAT_Kappa_Linz_Filter_Output;
    real32_T LAT_Sat_Dynamic_Threshold_int;
    real32_T Lateral_Error_Delta;
    real32_T Lateral_Error_Mean;
    real32_T Mean_Sample_Update_sum;
    real32_T Mean_Vehicle_Velocity;
    real32_T Mean_Vehicle_Velocity_sum;
    real32_T Mean_kappa_command;
    real32_T Mean_kappa_command_sum;
    boolean_T New_Update_Aval;
    real32_T New_Update_Aval_sum;
    real32_T SAC_Angle_Command_Corr;
    real32_T SAC_Angle_Command_Yawrate_Fback;
    real32_T SAC_Arbitrated_Angle_Cmd;
    real32_T SAC_Arbitrated_Angle_Cmd_Raw;
    real32_T SAC_Compensation_Angle_Command;
    real32_T SAC_Control_Error;
    real32_T SAC_Derated_Angle_Command;
    real32_T SAC_Integrator_Sat_Out;
    real32_T SAC_Rate_Lim_Angle_Command;
    real32_T SAC_Trq_Derating_Factor;
    real32_T SAC_Yrc_Angle_Command;
    real32_T SAC_Yrc_Control_Error;
    real32_T SAT_Req_Dyn_Steer_Angle_Max;
    real32_T SAT_Req_Steer_Angle_Max;
    real32_T SAT_Saturated_Angle_Command;
    real32_T TDF_Composite_Derating_Factor;
    real32_T TDF_Idle_Derating_Factor;
    real32_T TDF_Selected_State_Derating_Factor;
    real32_T VEH_Delta_F_Oc;
    real32_T VEH_Delta_F_Offset;
    boolean_T TDF_Driver_Counter_Steering;
    real32_T TDF_Max_Steer_Torque;
    real32_T TDF_Selected_Torque_Source;
    real32_T TDF_Torque_Der_Factor_HF_Path;
    real32_T TDF_Torque_Derating_Factor;
    real32_T TDF_Torque_Derating_Slop_Arb;
    real32_T TDF_Trq_Derating_Threshold_Arb;
    real32_T TDF_Vehicle_Steer_Torque_Factor;
    real32_T TDF_Torque_Request_Factor;
    real32_T VEH_Steer_Torque_Comp;
    real32_T LAT_OC_State_Preload;
    boolean_T LAT_Oc_Cal_Hold_Flag;
    boolean_T LAT_Oc_Cal_Hold_Flag_Shrt;
    boolean_T LAT_Oc_Disable_Flag;
    boolean_T LAT_Oc_Dys_Active;
    real32_T LAT_Oc_Filtered_Kappa_Cam;
    boolean_T LAT_Oc_High_Driver_Torque;
    boolean_T LAT_Oc_Implaus_Lateral_Error;
    real32_T LAT_Oc_Integrator_Input;
    real32_T LAT_Oc_Integrator_Input_Kappa;
    real32_T LAT_Oc_Integrator_Input_Kappa_dbg;
    real32_T LAT_Oc_Integrator_Output;
    real32_T LAT_Oc_Integrator_Sat_Out;
    boolean_T LAT_Oc_Kappa_Active;
    boolean_T LAT_Oc_Kappa_Con_Enb_Flag;
    real32_T LAT_Oc_Kappa_Latency_state;
    real32_T LAT_Oc_Kappa_Status_dbg;
    boolean_T LAT_Oc_Max_Delta_F_Dot_Flag;
    boolean_T LAT_Oc_Max_Drv_Trq_Flag;
    boolean_T LAT_Oc_Max_Hea_Err_Flag;
    boolean_T LAT_Oc_Max_Kappa_Cmd_Flag;
    boolean_T LAT_Oc_Max_Lat_Acc_Flag;
    real32_T LAT_Oc_Sat_Integrator_state;
    boolean_T LAT_Oc_Trigger_Flag_Kappa;
    real32_T LAT_Oc_Trigger_Flag_Kappa_state;
    real32_T LAT_Oc_offset_filter_omega;
    boolean_T LAT_Oc_Max_Flt_Kappa_Cmd_Flag;
    boolean_T LAT_Oc_Min_Veh_Vel_Flag;

    // Reserved for DMC2
    real32_T LAT_Arb_Eps_Torque_Request;
    real32_T LAT_Direct_Feedthrough_Trq_Par;
    real32_T LAT_Eps_Torque_Request_Interface;
    real32_T LAT_Eps_Torque_Request_Output_Limted;
    real32_T LAT_Torque_Gradient_Par;
    real32_T LAT_Torque_Request_No_Fric;
    real32_T Rate_Limiter_Output;
    real32_T SAC_Active_Damping_Torque;
    real32_T SAC_Control_Error_DMC2;
    real32_T SAC_Delta_F_Dot_Filtered;
    boolean_T SAC_Disable;
    real32_T SAC_Dynamic_Feedforward_Torque;
    real32_T SAC_Filtered_Angle_Command;
    real32_T SAC_Integrator_Torque;
    real32_T SAC_Proportional_Torque;
    real32_T SAC_Torque_Sat_Out;
    real32_T SAT_Saturated_Angle_Command_DMC2;
} proLcfDmcDebug_t;
#define Rte_TypeDef_proLcfDmcDebug_t
#endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

void LcfDMCExec(const reqLcfDmcPrtList_t* const reqPorts,
                const reqLcfDmcParams* reqParams,
                proLcfDmcPrtList_t* const proPorts,
                proLcfDmcDebug_t* proDebugs);
void LcfDMCReset(const reqLcfDmcParams* reqParams);

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_
        // SDK_DECISION_SRC_LCF_DMC_LCF_DMC_LOCAL_EXT_H_
