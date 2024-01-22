/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
// Copyright [2022] <Copyright senseauto>" [legal/copyright]
#include "TJADMC_ext.h"
#include "LaDMC.h"
#include <string.h>
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Imported (extern) block signals */
// real32_T fSteerAngleRequest_deg; /* '<Root>/fSteerAngleRequest_deg' */
// uint8_T uiModeRequest_nu;        /* '<Root>/uiModeRequest_nu' */
// real32_T fSteerAngle_deg;        /* '<Root>/fSteerAngle_deg' */
// real32_T fDriverTorque_nm;       /* '<Root>/fDriverTorque_nm' */
// real32_T fVehicleVelX_ms;        /* '<Root>/fVehicleVelX_ms' */
// uint8_T uiDriverMode_nu;         /* '<Root>/uiDriverMode_nu' */
real32_T VEH_Vehicle_Speed;                       /* '<Root>/Inport' */
real32_T LKC_Kappa_Command;                       /* '<Root>/Inport2' */
real32_T VEH_Yaw_Rate;                            /* '<Root>/Inport5' */
real32_T Veh_Lat_Acc;                             /* '<Root>/Inport7' */
real32_T VEH_Steer_Torque;                        /* '<Root>/Inport11' */
real32_T LAT_Stiffness_Request_Factor;            /* '<Root>/Inport16' */
real32_T VEH_SteerAngle;                          /* '<Root>/Inport17' */
real32_T CAM_Latral_Error_Qf;                     /* '<Root>/Inport18' */
real32_T CAM_Lateral_Error;                       /* '<Root>/Inport19' */
real32_T VEH_cycletime;                           /* '<Root>/Inport13' */
real32_T VEH_Delta_F_Dot;                         /* '<Root>/VEH_Delta_F_Dot' */
real32_T LKC_Delta_Ys;                            /* '<Root>/LKC_Delta_Ys' */
real32_T LKC_Delta_Psi;                           /* '<Root>/LKC_Delta_Psi' */
uint8_T LDP_Status;                               /* '<Root>/Inport24' */
real32_T CAM_Kappa_Cmd;                           /* '<Root>/Inport1' */
real32_T CAM_Kappa_Cmd_Qf;                        /* '<Root>/Inport9' */
real32_T EPS_Gear_Ratio;                          /* '<Root>/Inport15' */
real32_T LaKMC_kappaP_cmd;                        /* '<Root>/Inport3' */
real32_T LaKMC_angle_req_max_limit_scale;         /* '<Root>/Inport4' */
real32_T LaKMC_angle_req_max_grad_scale;          /* '<Root>/Inport6' */
real32_T CAM_Heading_Error;                       /* '<Root>/Inport8' */
real32_T CAM_Heading_Error_Qf;                    /* '<Root>/Inport10' */
uint8_T LKC_DgrSide;                              /* '<Root>/Inport12' */
uint8_T TJALatCtrlMode_nu;                        /* '<Root>/Inport20' */
real32_T NOP_Lon_Acc_Cmd;                         /* '<Root>/Inport14' */
real32_T NOP_Delta_F_Cmd;                         /* '<Root>/Inport49' */
real32_T ADAS_Lon_Acc_Cmd;                        /* '<Root>/Inport54' */
real32_T ADAS_Delta_F_Cmd;                        /* '<Root>/Inport59' */
boolean_T NOP_Lon_Acc_Enable_flag;                /* '<Root>/Inport60' */
boolean_T NOP_Delta_F_Enable_flag;                /* '<Root>/Inport61' */
boolean_T ADAS_Lon_Acc_Enable_flag;               /* '<Root>/Inport62' */
boolean_T ADAS_Delta_F_Enable_flag;               /* '<Root>/Inport63' */
uint16_T Dmc_configuration_mode_par;              /* '<Root>/Inport21' */
uint8_T Dmc_offset_calibration_mode_par;          /* '<Root>/Inport64' */
real32_T Dmc_DeltaF_offset_par;                   /* '<Root>/Inport65' */
real32_T Lat_oc_kappa_cmd_filter_coeff_par;       /* '<Root>/Inport66' */
real32_T Lat_oc_max_kappa_dys_par;                /* '<Root>/Inport67' */
real32_T Lat_oc_ki_par;                           /* '<Root>/Inport68' */
real32_T Lat_oc_delta_off_flt_initial_omega_par;  /* '<Root>/Inport69' */
real32_T Lat_oc_delta_offset_filter_omega_par;    /* '<Root>/Inport70' */
real32_T Lat_oc_max_driver_torque_par;            /* '<Root>/Inport71' */
real32_T Dyc_state_filter_time_constant_par;      /* '<Root>/Inport25' */
uint16_T Dyc_compensation_mode_par;               /* '<Root>/Inport26' */
real32_T Dyc_kappa_a2_pos_corr_factor_par;        /* '<Root>/Inport27' */
real32_T Dyc_kappa_a2_x_scheduling_par[12];       /* '<Root>/Inport22' */
real32_T Dyc_kappa_a2_y_scheduling_par[12];       /* '<Root>/Inport23' */
real32_T Dyc_kappa_a2_factor_par;                 /* '<Root>/Inport28' */
real32_T Dyc_kappa_angle_gen_corr_factor_par;     /* '<Root>/Inport29' */
real32_T Dyc_kappa_angle_t_x_schedul_gen_par[12]; /* '<Root>/Inport30' */
real32_T Dyc_kappa_angle_t_y_schedul_gen_par[12]; /* '<Root>/Inport31' */
real32_T Dyc_kappa_angle_gen_cor_fct_neg_par;     /* '<Root>/Inport32' */
real32_T Dyc_kappa_angle_t_y_sch_gen_neg_par[12]; /* '<Root>/Inport33' */
real32_T Lat_max_ay_par;                          /* '<Root>/Inport34' */
uint32_T Sac_controller_mode_1_par;               /* '<Root>/Inport35' */
real32_T Sat_max_cmd_factor_x_scheduling_par[13]; /* '<Root>/Inport36' */
real32_T Sat_max_cmd_factor_y_scheduling_par[13]; /* '<Root>/Inport37' */
real32_T Sat_max_delta_f_cmd_par;                 /* '<Root>/Inport38' */
real32_T VEH_Vehicle_Selfsteering_Factor_par;     /* '<Root>/Inport39' */
real32_T Sat_max_angle_offset_x_par[4];           /* '<Root>/Inport40' */
real32_T Sat_max_angle_offset_y_par[4];           /* '<Root>/Inport41' */
real32_T Sat_max_angle_low_bound_x_sched_par[3];  /* '<Root>/Inport42' */
real32_T Sat_max_angle_low_bound_y_sched_par[3];  /* '<Root>/Inport43' */
real32_T Tdf_min_steer_torque_class_par;          /* '<Root>/Inport44' */
uint16_T Tdf_derating_mode_par;                   /* '<Root>/Inport45' */
real32_T Tdf_torque_request_sign_slope_par;       /* '<Root>/Inport46' */
real32_T Tdf_steer_torque_sign_slope_par;         /* '<Root>/Inport47' */
real32_T Tdf_torque_derating_threshold_par;       /* '<Root>/Inport48' */
real32_T Tdf_trq_derating_threshold_dp_par;       /* '<Root>/Inport50' */
real32_T Tdf_trq_der_thrs_dp_hi_sens_par;         /* '<Root>/Inport51' */
real32_T Tdf_torque_derating_slope_ldp_par;       /* '<Root>/Inport52' */
real32_T Tdf_torque_derating_slope_par;           /* '<Root>/Inport53' */
real32_T TDF_Derating_Switch_Thresh2_par;         /* '<Root>/Inport55' */
real32_T TDF_Derating_Switch_Thresh1_par;         /* '<Root>/Inport56' */
real32_T Sac_delta_f_cmd_grad_barrier_par;        /* '<Root>/Inport57' */
real32_T Sac_delta_f_cmd_min_grad_par;            /* '<Root>/Inport58' */
boolean_T DMC_NVRAMReset_par;                     /* '<Root>/Inport72' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:     LCF_TJADMC_Reset                                  */ /*!

  @brief:           TJADMC function reset

  @description:     All global variables related to TJADMC are 
                    reset in this function when TJADMC executes
                    for the first time, or system exception needs
                    to be reset

  @param[in]:       void

  @return:          void
*****************************************************************************/
void LCF_TJADMC_Reset(void) { LaDMC_initialize(); }

/*****************************************************************************
  Functionname:     LCF_TJADMC_Exec                                  */ /*!

  @brief:           Execution entry of TJADMC function

  @description:     MCTLFC main function

  @param[in]:       reqPorts   MCTLFC input
                    params     MCTLFC parameter input
                    proPorts   MCTLFC output
                    debugInfo  MCTLFC debug information

  @return:void
*****************************************************************************/
void LCF_TJADMC_Exec(const sTJADMCInReq_t* reqPorts,
                     const sTJADMCParam_t* param,
                     sTJADMCOutPro_t* proPorts,
                     sTJADMCDebug_t* debug) {
    /* TJADMC input wrapper */
    // fSteerAngleRequest_deg = reqPorts->fSteerAngleRequest_deg;
    // uiModeRequest_nu = reqPorts->uiModeRequest_nu;
    // fSteerAngle_deg = reqPorts->fSteerAngle_deg;
    // fDriverTorque_nm = reqPorts->fDriverTorque_nm;
    // fVehicleVelX_ms = reqPorts->fVehicleVelX_ms;
    // uiDriverMode_nu = reqPorts->uiDriverMode_nu;
    VEH_Vehicle_Speed = reqPorts->VEH_Vehicle_Speed;
    LKC_Kappa_Command = reqPorts->LKC_Kappa_Command;
    VEH_Yaw_Rate = reqPorts->VEH_Yaw_Rate;
    Veh_Lat_Acc = reqPorts->Veh_Lat_Acc;
    VEH_Steer_Torque = reqPorts->VEH_Steer_Torque;
    LAT_Stiffness_Request_Factor = reqPorts->LAT_Stiffness_Request_Factor;
    VEH_SteerAngle = reqPorts->VEH_SteerAngle;
    CAM_Latral_Error_Qf = reqPorts->CAM_Latral_Error_Qf;
    CAM_Lateral_Error = reqPorts->CAM_Lateral_Error;
    VEH_Delta_F_Dot = reqPorts->VEH_Delta_F_Dot;
    LKC_Delta_Ys = reqPorts->LKC_Delta_Ys;
    LKC_Delta_Psi = reqPorts->LKC_Delta_Psi;
    LDP_Status = reqPorts->LDP_Status;
    CAM_Kappa_Cmd = reqPorts->CAM_Kappa_Cmd;
    CAM_Kappa_Cmd_Qf = reqPorts->CAM_Kappa_Cmd_Qf;
    EPS_Gear_Ratio = reqPorts->EPS_Gear_Ratio;
    LaKMC_kappaP_cmd = reqPorts->LaKMC_kappaP_cmd;
    LaKMC_angle_req_max_limit_scale = reqPorts->LaKMC_angle_req_max_limit_scale;
    LaKMC_angle_req_max_grad_scale = reqPorts->LaKMC_angle_req_max_grad_scale;
    CAM_Heading_Error = reqPorts->CAM_Heading_Error;
    CAM_Heading_Error_Qf = reqPorts->CAM_Heading_Error_Qf;
    LKC_DgrSide = reqPorts->LKC_DgrSide;
    NOP_Lon_Acc_Cmd = reqPorts->NOP_Lon_Acc_Cmd;
    NOP_Delta_F_Cmd = reqPorts->NOP_Delta_F_Cmd;
    ADAS_Lon_Acc_Cmd = reqPorts->ADAS_Lon_Acc_Cmd;
    ADAS_Delta_F_Cmd = reqPorts->ADAS_Delta_F_Cmd;
    NOP_Lon_Acc_Enable_flag = reqPorts->NOP_Lon_Acc_Enable_flag;
    NOP_Delta_F_Enable_flag = reqPorts->NOP_Delta_F_Enable_flag;
    ADAS_Lon_Acc_Enable_flag = reqPorts->ADAS_Lon_Acc_Enable_flag;
    ADAS_Delta_F_Enable_flag = reqPorts->ADAS_Delta_F_Enable_flag;
    VEH_cycletime = reqPorts->VEH_cycletime;
    TJALatCtrlMode_nu = reqPorts->TJALatCtrlMode_nu;
    Dmc_configuration_mode_par = reqPorts->Dmc_configuration_mode_par;
    Dmc_offset_calibration_mode_par = reqPorts->Dmc_offset_calibration_mode_par;
    Dmc_DeltaF_offset_par = reqPorts->Dmc_DeltaF_offset_par;
    Dmc_DeltaF_offset_par = reqPorts->Dmc_DeltaF_offset_par;
    Lat_oc_kappa_cmd_filter_coeff_par =
        reqPorts->Lat_oc_kappa_cmd_filter_coeff_par;
    Lat_oc_max_kappa_dys_par = reqPorts->Lat_oc_max_kappa_dys_par;
    Lat_oc_ki_par = reqPorts->Lat_oc_ki_par;
    Lat_oc_delta_off_flt_initial_omega_par =
        reqPorts->Lat_oc_delta_off_flt_initial_omega_par;
    Lat_oc_delta_offset_filter_omega_par =
        reqPorts->Lat_oc_delta_offset_filter_omega_par;
    Lat_oc_max_driver_torque_par = reqPorts->Lat_oc_max_driver_torque_par;
    Dyc_state_filter_time_constant_par =
        reqPorts->Dyc_state_filter_time_constant_par;
    Dyc_compensation_mode_par = reqPorts->Dyc_compensation_mode_par;
    Dyc_kappa_a2_pos_corr_factor_par =
        reqPorts->Dyc_kappa_a2_pos_corr_factor_par;
    memcpy(Dyc_kappa_a2_x_scheduling_par,
           reqPorts->Dyc_kappa_a2_x_scheduling_par, 12 * sizeof(real32_T));
    memcpy(Dyc_kappa_a2_y_scheduling_par,
           reqPorts->Dyc_kappa_a2_y_scheduling_par, 12 * sizeof(real32_T));
    Dyc_kappa_a2_factor_par = reqPorts->Dyc_kappa_a2_factor_par;
    Dyc_kappa_angle_gen_corr_factor_par =
        reqPorts->Dyc_kappa_angle_gen_corr_factor_par;
    memcpy(Dyc_kappa_angle_t_x_schedul_gen_par,
           reqPorts->Dyc_kappa_angle_t_x_schedul_gen_par,
           12 * sizeof(real32_T));
    memcpy(Dyc_kappa_angle_t_y_schedul_gen_par,
           reqPorts->Dyc_kappa_angle_t_y_schedul_gen_par,
           12 * sizeof(real32_T));
    Dyc_kappa_angle_gen_cor_fct_neg_par =
        reqPorts->Dyc_kappa_angle_gen_cor_fct_neg_par;
    memcpy(Dyc_kappa_angle_t_y_sch_gen_neg_par,
           reqPorts->Dyc_kappa_angle_t_y_sch_gen_neg_par,
           12 * sizeof(real32_T));
    Lat_max_ay_par = reqPorts->Lat_max_ay_par;
    Sac_controller_mode_1_par = reqPorts->Sac_controller_mode_1_par;
    memcpy(Sat_max_cmd_factor_x_scheduling_par,
           reqPorts->Sat_max_cmd_factor_x_scheduling_par,
           13 * sizeof(real32_T));
    memcpy(Sat_max_cmd_factor_y_scheduling_par,
           reqPorts->Sat_max_cmd_factor_y_scheduling_par,
           13 * sizeof(real32_T));
    Sat_max_delta_f_cmd_par = reqPorts->Sat_max_delta_f_cmd_par;
    VEH_Vehicle_Selfsteering_Factor_par =
        reqPorts->VEH_Vehicle_Selfsteering_Factor_par;
    memcpy(Sat_max_angle_offset_x_par, reqPorts->Sat_max_angle_offset_x_par,
           4 * sizeof(real32_T));
    memcpy(Sat_max_angle_offset_y_par, reqPorts->Sat_max_angle_offset_y_par,
           4 * sizeof(real32_T));
    memcpy(Sat_max_angle_low_bound_x_sched_par,
           reqPorts->Sat_max_angle_low_bound_x_sched_par, 3 * sizeof(real32_T));
    memcpy(Sat_max_angle_low_bound_y_sched_par,
           reqPorts->Sat_max_angle_low_bound_y_sched_par, 3 * sizeof(real32_T));
    Tdf_min_steer_torque_class_par = reqPorts->Tdf_min_steer_torque_class_par;
    Tdf_derating_mode_par = reqPorts->Tdf_derating_mode_par;
    Tdf_torque_request_sign_slope_par =
        reqPorts->Tdf_torque_request_sign_slope_par;
    Tdf_steer_torque_sign_slope_par = reqPorts->Tdf_steer_torque_sign_slope_par;
    Tdf_torque_derating_threshold_par =
        reqPorts->Tdf_torque_derating_threshold_par;
    Tdf_trq_derating_threshold_dp_par =
        reqPorts->Tdf_trq_derating_threshold_dp_par;
    Tdf_trq_der_thrs_dp_hi_sens_par = reqPorts->Tdf_trq_der_thrs_dp_hi_sens_par;
    Tdf_torque_derating_slope_ldp_par =
        reqPorts->Tdf_torque_derating_slope_ldp_par;
    Tdf_torque_derating_slope_par = reqPorts->Tdf_torque_derating_slope_par;
    TDF_Derating_Switch_Thresh2_par = reqPorts->TDF_Derating_Switch_Thresh2_par;
    TDF_Derating_Switch_Thresh1_par = reqPorts->TDF_Derating_Switch_Thresh1_par;
    Sac_delta_f_cmd_grad_barrier_par =
        reqPorts->Sac_delta_f_cmd_grad_barrier_par;
    Sac_delta_f_cmd_min_grad_par = reqPorts->Sac_delta_f_cmd_min_grad_par;
    DMC_NVRAMReset_par = reqPorts->DMC_NVRAMReset_par;
    memcpy(Dyc_kappa_a2_x_scheduling_par,
           reqPorts->Dyc_kappa_a2_x_scheduling_par, 12 * sizeof(real32_T));
    memcpy(Dyc_kappa_a2_y_scheduling_par,
           reqPorts->Dyc_kappa_a2_y_scheduling_par, 12 * sizeof(real32_T));

    /* TJADMC function */
    LaDMC_step();

    /* TJADMC output wrapper */
    // proPorts->fSteerWhlTqAddl_nm = fSteerWhlTqAddl_nm;
    // proPorts->uiTorqueReqAct_nu = uiTorqueReqAct_nu;q
    // proPorts->fSteerWhlAngleAddl_deg = fSteerWhlAngleAddl_deg;
    proPorts->fSteerAngle_deg = fSteerAngle_deg;
    proPorts->fSteerWhlAngle_deg = fSteerWhlAngle_deg;
    proPorts->uiEPSRequest_nu = uiEPSRequest_nu;
    proPorts->uiLonAccRequest_nu = uiLonAccRequest_nu;
    proPorts->fLonAccCmd = fLonAccCmd;

    /* TJASA debug wrapper */
    debug->ADP_Dyc_Corr_Factor = ADP_Dyc_Corr_Factor;
    debug->CAM_Lateral_Error_Sign = CAM_Lateral_Error_Sign;
    debug->DYC_Filter_Kappa_Command = DYC_Filter_Kappa_Command;
    debug->DYC_Steer_Angle_Feedforward = DYC_Steer_Angle_Feedforward;
    debug->HEC_Yaw_Rate_Filter = HEC_Yaw_Rate_Filter;
    debug->Initialisation_Flag = Initialisation_Flag;
    debug->LAT_Kappa_Linz_Filter_Output = LAT_Kappa_Linz_Filter_Output;
    debug->LAT_Oc_Integrator_Output = LAT_Oc_Integrator_Output;
    debug->LAT_Sat_Dynamic_Threshold_int = LAT_Sat_Dynamic_Threshold_int;
    debug->Lateral_Error_Delta = Lateral_Error_Delta;
    debug->Lateral_Error_Mean = Lateral_Error_Mean;
    debug->Mean_Sample_Update_sum = Mean_Sample_Update_sum;
    debug->Mean_Vehicle_Velocity = Mean_Vehicle_Velocity;
    debug->Mean_Vehicle_Velocity_sum = Mean_Vehicle_Velocity_sum;
    debug->Mean_kappa_command = Mean_kappa_command;
    debug->Mean_kappa_command_sum = Mean_kappa_command_sum;
    debug->New_Update_Aval = New_Update_Aval;
    debug->New_Update_Aval_sum = New_Update_Aval_sum;
    debug->SAC_Angle_Command_Corr = SAC_Angle_Command_Corr;
    debug->SAC_Angle_Command_Yawrate_Fback = SAC_Angle_Command_Yawrate_Fback;
    debug->SAC_Arbitrated_Angle_Cmd = SAC_Arbitrated_Angle_Cmd;
    debug->SAC_Arbitrated_Angle_Cmd_Raw = SAC_Arbitrated_Angle_Cmd_Raw;
    debug->SAC_Compensation_Angle_Command = SAC_Compensation_Angle_Command;
    debug->SAC_Control_Error = SAC_Control_Error;
    debug->SAC_Derated_Angle_Command = SAC_Derated_Angle_Command;
    debug->SAC_Integrator_Sat_Out = SAC_Integrator_Sat_Out;
    debug->SAC_Rate_Lim_Angle_Command = SAC_Rate_Lim_Angle_Command;
    debug->SAC_Trq_Derating_Factor = SAC_Trq_Derating_Factor;
    debug->SAC_Yrc_Angle_Command = SAC_Yrc_Angle_Command;
    debug->SAC_Yrc_Control_Error = SAC_Yrc_Control_Error;
    debug->SAT_Req_Dyn_Steer_Angle_Max = SAT_Req_Dyn_Steer_Angle_Max;
    debug->SAT_Req_Steer_Angle_Max = SAT_Req_Steer_Angle_Max;
    debug->SAT_Saturated_Angle_Command = SAT_Saturated_Angle_Command;
    debug->TDF_Composite_Derating_Factor = TDF_Composite_Derating_Factor;
    debug->TDF_Idle_Derating_Factor = TDF_Idle_Derating_Factor;
    debug->TDF_Selected_State_Derating_Factor =
        TDF_Selected_State_Derating_Factor;
    debug->VEH_Delta_F_Oc = VEH_Delta_F_Oc;
    debug->VEH_Delta_F_Offset = VEH_Delta_F_Offset;
    debug->TDF_Driver_Counter_Steering = TDF_Driver_Counter_Steering;
    debug->TDF_Max_Steer_Torque = TDF_Max_Steer_Torque;
    debug->TDF_Selected_Torque_Source = TDF_Selected_Torque_Source;
    debug->TDF_Torque_Der_Factor_HF_Path = TDF_Torque_Der_Factor_HF_Path;
    debug->TDF_Torque_Derating_Factor = TDF_Torque_Derating_Factor;
    debug->TDF_Torque_Derating_Slop_Arb = TDF_Torque_Derating_Slop_Arb;
    debug->TDF_Trq_Derating_Threshold_Arb = TDF_Trq_Derating_Threshold_Arb;
    debug->TDF_Vehicle_Steer_Torque_Factor = TDF_Vehicle_Steer_Torque_Factor;
    debug->TDF_Torque_Request_Factor = TDF_Torque_Request_Factor;
    debug->VEH_Steer_Torque_Comp = VEH_Steer_Torque_Comp;
    debug->LAT_OC_State_Preload = LAT_OC_State_Preload;
    debug->LAT_Oc_Cal_Hold_Flag = LAT_Oc_Cal_Hold_Flag;
    debug->LAT_Oc_Cal_Hold_Flag_Shrt = LAT_Oc_Cal_Hold_Flag_Shrt;
    debug->LAT_Oc_Disable_Flag = LAT_Oc_Disable_Flag;
    debug->LAT_Oc_Dys_Active = LAT_Oc_Dys_Active;
    debug->LAT_Oc_Filtered_Kappa_Cam = LAT_Oc_Filtered_Kappa_Cam;
    debug->LAT_Oc_High_Driver_Torque = LAT_Oc_High_Driver_Torque;
    debug->LAT_Oc_Implaus_Lateral_Error = LAT_Oc_Implaus_Lateral_Error;
    debug->LAT_Oc_Integrator_Input = LAT_Oc_Integrator_Input;
    debug->LAT_Oc_Integrator_Input_Kappa = LAT_Oc_Integrator_Input_Kappa;
    debug->LAT_Oc_Integrator_Input_Kappa_dbg =
        LAT_Oc_Integrator_Input_Kappa_dbg;
    debug->LAT_Oc_Integrator_Output = LAT_Oc_Integrator_Output;
    debug->LAT_Oc_Integrator_Sat_Out = LAT_Oc_Integrator_Sat_Out;
    debug->LAT_Oc_Kappa_Active = LAT_Oc_Kappa_Active;
    debug->LAT_Oc_Kappa_Con_Enb_Flag = LAT_Oc_Kappa_Con_Enb_Flag;
    debug->LAT_Oc_Kappa_Latency_state = LAT_Oc_Kappa_Latency_state;
    debug->LAT_Oc_Kappa_Status_dbg = LAT_Oc_Kappa_Status_dbg;
    debug->LAT_Oc_Max_Delta_F_Dot_Flag = LAT_Oc_Max_Delta_F_Dot_Flag;
    debug->LAT_Oc_Max_Drv_Trq_Flag = LAT_Oc_Max_Drv_Trq_Flag;
    debug->LAT_Oc_Max_Hea_Err_Flag = LAT_Oc_Max_Hea_Err_Flag;
    debug->LAT_Oc_Max_Kappa_Cmd_Flag = LAT_Oc_Max_Kappa_Cmd_Flag;
    debug->LAT_Oc_Max_Lat_Acc_Flag = LAT_Oc_Max_Lat_Acc_Flag;
    debug->LAT_Oc_Sat_Integrator_state = LAT_Oc_Sat_Integrator_state;
    debug->LAT_Oc_Trigger_Flag_Kappa = LAT_Oc_Trigger_Flag_Kappa;
    debug->LAT_Oc_Trigger_Flag_Kappa_state = LAT_Oc_Trigger_Flag_Kappa_state;
    debug->LAT_Oc_offset_filter_omega = LAT_Oc_offset_filter_omega;
    debug->LAT_Oc_Max_Flt_Kappa_Cmd_Flag = LAT_Oc_Max_Flt_Kappa_Cmd_Flag;
    debug->LAT_Oc_Min_Veh_Vel_Flag = LAT_Oc_Min_Veh_Vel_Flag;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */