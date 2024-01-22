// Copyright [2022] <Copyright Senseauto>" [legal/copyright]
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LADMC_TJADMC_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LADMC_TJADMC_EXT_H_

// #include "TJADMC.h"
#include "rtwtypes.h"

typedef struct {
    real32_T VEH_Vehicle_Speed;                 /* '<Root>/Inport' */
    real32_T LKC_Kappa_Command;                 /* '<Root>/Inport2' */
    real32_T VEH_Yaw_Rate;                      /* '<Root>/Inport5' */
    real32_T Veh_Lat_Acc;                       /* '<Root>/Inport7' */
    real32_T VEH_Steer_Torque;                  /* '<Root>/Inport11' */
    real32_T LAT_Stiffness_Request_Factor;      /* '<Root>/Inport16' */
    real32_T VEH_SteerAngle;                    /* '<Root>/Inport17' */
    real32_T CAM_Latral_Error_Qf;               /* '<Root>/Inport18' */
    real32_T CAM_Lateral_Error;                 /* '<Root>/Inport19' */
    real32_T VEH_cycletime;                     /* '<Root>/Inport13' */
    real32_T VEH_Delta_F_Dot;                   /* '<Root>/VEH_Delta_F_Dot' */
    real32_T LKC_Delta_Ys;                      /* '<Root>/LKC_Delta_Ys' */
    real32_T LKC_Delta_Psi;                     /* '<Root>/LKC_Delta_Psi' */
    uint8_T LDP_Status;                         /* '<Root>/Inport24' */
    real32_T CAM_Kappa_Cmd;                     /* '<Root>/Inport1' */
    real32_T CAM_Kappa_Cmd_Qf;                  /* '<Root>/Inport9' */
    real32_T EPS_Gear_Ratio;                    /* '<Root>/Inport15' */
    real32_T LaKMC_kappaP_cmd;                  /* '<Root>/Inport3' */
    real32_T LaKMC_angle_req_max_limit_scale;   /* '<Root>/Inport4' */
    real32_T LaKMC_angle_req_max_grad_scale;    /* '<Root>/Inport6' */
    real32_T CAM_Heading_Error;                 /* '<Root>/Inport8' */
    real32_T CAM_Heading_Error_Qf;              /* '<Root>/Inport10' */
    uint8_T LKC_DgrSide;                        /* '<Root>/Inport12' */
    uint8_T TJALatCtrlMode_nu;                  /* '<Root>/Inport20' */
    real32_T NOP_Lon_Acc_Cmd;                   /* '<Root>/Inport14' */
    real32_T NOP_Delta_F_Cmd;                   /* '<Root>/Inport49' */
    real32_T ADAS_Lon_Acc_Cmd;                  /* '<Root>/Inport54' */
    real32_T ADAS_Delta_F_Cmd;                  /* '<Root>/Inport59' */
    boolean_T NOP_Lon_Acc_Enable_flag;          /* '<Root>/Inport60' */
    boolean_T NOP_Delta_F_Enable_flag;          /* '<Root>/Inport61' */
    boolean_T ADAS_Lon_Acc_Enable_flag;         /* '<Root>/Inport62' */
    boolean_T ADAS_Delta_F_Enable_flag;         /* '<Root>/Inport63' */
    uint16_T Dmc_configuration_mode_par;        /* '<Root>/Inport21' */
    uint8_T Dmc_offset_calibration_mode_par;    /* '<Root>/Inport64' */
    real32_T Dmc_DeltaF_offset_par;             /* '<Root>/Inport65' */
    real32_T Lat_oc_kappa_cmd_filter_coeff_par; /* '<Root>/Inport66' */
    real32_T Lat_oc_max_kappa_dys_par;          /* '<Root>/Inport67' */
    real32_T Lat_oc_ki_par;                     /* '<Root>/Inport68' */
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
    boolean_T DMC_NVRAMReset_par;
} sTJADMCInReq_t;

typedef struct {
    uint8_T uTemp_nu; /* Not used, just for the unified interface */
} sTJADMCParam_t;

typedef struct {
    real32_T fSteerAngle_deg;
    real32_T fSteerWhlAngle_deg;
    real32_T fLonAccCmd;
    uint8_T uiEPSRequest_nu;
    uint8_T uiLonAccRequest_nu;
} sTJADMCOutPro_t;

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
} sTJADMCDebug_t;

extern void LCF_TJADMC_Reset(void);
extern void LCF_TJADMC_Exec(const sTJADMCInReq_t* reqPorts,
                            const sTJADMCParam_t* param,
                            sTJADMCOutPro_t* proPorts,
                            sTJADMCDebug_t* debug);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LADMC_TJADMC_EXT_H_
