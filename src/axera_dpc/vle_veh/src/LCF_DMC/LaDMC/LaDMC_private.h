/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LaDMC
 *
 * Model Long Name     :

 *

 * Model Advisor       : Not Check

 *

 * Model Version       :

 *

 * Model Author        :

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    :


 ************************************Auto Coder**********************************
 *
 * File                             : LaDMC_private.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Jan  6 09:31:14 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LaDMC_private_h_
#define RTW_HEADER_LaDMC_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T VEH_Vehicle_Speed;     /* '<Root>/Inport' */
extern real32_T LKC_Kappa_Command;     /* '<Root>/Inport2' */
extern real32_T VEH_Yaw_Rate;          /* '<Root>/Inport5' */
extern real32_T Veh_Lat_Acc;           /* '<Root>/Inport7' */
extern real32_T VEH_Steer_Torque;      /* '<Root>/Inport11' */
extern real32_T LAT_Stiffness_Request_Factor;/* '<Root>/Inport16' */
extern real32_T VEH_SteerAngle;        /* '<Root>/Inport17' */
extern real32_T CAM_Latral_Error_Qf;   /* '<Root>/Inport18' */
extern real32_T CAM_Lateral_Error;     /* '<Root>/Inport19' */
extern real32_T VEH_cycletime;         /* '<Root>/Inport13' */
extern real32_T VEH_Delta_F_Dot;       /* '<Root>/VEH_Delta_F_Dot' */
extern real32_T LKC_Delta_Ys;          /* '<Root>/LKC_Delta_Ys' */
extern real32_T LKC_Delta_Psi;         /* '<Root>/LKC_Delta_Psi' */
extern uint8_T LDP_Status;             /* '<Root>/Inport24' */
extern real32_T CAM_Kappa_Cmd;         /* '<Root>/Inport1' */
extern real32_T CAM_Kappa_Cmd_Qf;      /* '<Root>/Inport9' */
extern real32_T EPS_Gear_Ratio;        /* '<Root>/Inport15' */
extern real32_T LaKMC_kappaP_cmd;      /* '<Root>/Inport3' */
extern real32_T LaKMC_angle_req_max_limit_scale;/* '<Root>/Inport4' */
extern real32_T LaKMC_angle_req_max_grad_scale;/* '<Root>/Inport6' */
extern real32_T CAM_Heading_Error;     /* '<Root>/Inport8' */
extern real32_T CAM_Heading_Error_Qf;  /* '<Root>/Inport10' */
extern uint8_T LKC_DgrSide;            /* '<Root>/Inport12' */
extern uint8_T TJALatCtrlMode_nu;      /* '<Root>/Inport20' */
extern real32_T NOP_Lon_Acc_Cmd;       /* '<Root>/Inport14' */
extern real32_T NOP_Delta_F_Cmd;       /* '<Root>/Inport49' */
extern real32_T ADAS_Lon_Acc_Cmd;      /* '<Root>/Inport54' */
extern real32_T ADAS_Delta_F_Cmd;      /* '<Root>/Inport59' */
extern boolean_T NOP_Lon_Acc_Enable_flag;/* '<Root>/Inport60' */
extern boolean_T NOP_Delta_F_Enable_flag;/* '<Root>/Inport61' */
extern boolean_T ADAS_Lon_Acc_Enable_flag;/* '<Root>/Inport62' */
extern boolean_T ADAS_Delta_F_Enable_flag;/* '<Root>/Inport63' */
extern uint16_T Dmc_configuration_mode_par;/* '<Root>/Inport21' */
extern uint8_T Dmc_offset_calibration_mode_par;/* '<Root>/Inport64' */
extern real32_T Dmc_DeltaF_offset_par; /* '<Root>/Inport65' */
extern real32_T Lat_oc_kappa_cmd_filter_coeff_par;/* '<Root>/Inport66' */
extern real32_T Lat_oc_max_kappa_dys_par;/* '<Root>/Inport67' */
extern real32_T Lat_oc_ki_par;         /* '<Root>/Inport68' */
extern real32_T Lat_oc_delta_off_flt_initial_omega_par;/* '<Root>/Inport69' */
extern real32_T Lat_oc_delta_offset_filter_omega_par;/* '<Root>/Inport70' */
extern real32_T Lat_oc_max_driver_torque_par;/* '<Root>/Inport71' */
extern real32_T Dyc_state_filter_time_constant_par;/* '<Root>/Inport25' */
extern uint16_T Dyc_compensation_mode_par;/* '<Root>/Inport26' */
extern real32_T Dyc_kappa_a2_pos_corr_factor_par;/* '<Root>/Inport27' */
extern real32_T Dyc_kappa_a2_x_scheduling_par[12];/* '<Root>/Inport22' */
extern real32_T Dyc_kappa_a2_y_scheduling_par[12];/* '<Root>/Inport23' */
extern real32_T Dyc_kappa_a2_factor_par;/* '<Root>/Inport28' */
extern real32_T Dyc_kappa_angle_gen_corr_factor_par;/* '<Root>/Inport29' */
extern real32_T Dyc_kappa_angle_t_x_schedul_gen_par[12];/* '<Root>/Inport30' */
extern real32_T Dyc_kappa_angle_t_y_schedul_gen_par[12];/* '<Root>/Inport31' */
extern real32_T Dyc_kappa_angle_gen_cor_fct_neg_par;/* '<Root>/Inport32' */
extern real32_T Dyc_kappa_angle_t_y_sch_gen_neg_par[12];/* '<Root>/Inport33' */
extern real32_T Lat_max_ay_par;        /* '<Root>/Inport34' */
extern uint32_T Sac_controller_mode_1_par;/* '<Root>/Inport35' */
extern real32_T Sat_max_cmd_factor_x_scheduling_par[13];/* '<Root>/Inport36' */
extern real32_T Sat_max_cmd_factor_y_scheduling_par[13];/* '<Root>/Inport37' */
extern real32_T Sat_max_delta_f_cmd_par;/* '<Root>/Inport38' */
extern real32_T VEH_Vehicle_Selfsteering_Factor_par;/* '<Root>/Inport39' */
extern real32_T Sat_max_angle_offset_x_par[4];/* '<Root>/Inport40' */
extern real32_T Sat_max_angle_offset_y_par[4];/* '<Root>/Inport41' */
extern real32_T Sat_max_angle_low_bound_x_sched_par[3];/* '<Root>/Inport42' */
extern real32_T Sat_max_angle_low_bound_y_sched_par[3];/* '<Root>/Inport43' */
extern real32_T Tdf_min_steer_torque_class_par;/* '<Root>/Inport44' */
extern uint16_T Tdf_derating_mode_par; /* '<Root>/Inport45' */
extern real32_T Tdf_torque_request_sign_slope_par;/* '<Root>/Inport46' */
extern real32_T Tdf_steer_torque_sign_slope_par;/* '<Root>/Inport47' */
extern real32_T Tdf_torque_derating_threshold_par;/* '<Root>/Inport48' */
extern real32_T Tdf_trq_derating_threshold_dp_par;/* '<Root>/Inport50' */
extern real32_T Tdf_trq_der_thrs_dp_hi_sens_par;/* '<Root>/Inport51' */
extern real32_T Tdf_torque_derating_slope_ldp_par;/* '<Root>/Inport52' */
extern real32_T Tdf_torque_derating_slope_par;/* '<Root>/Inport53' */
extern real32_T TDF_Derating_Switch_Thresh2_par;/* '<Root>/Inport55' */
extern real32_T TDF_Derating_Switch_Thresh1_par;/* '<Root>/Inport56' */
extern real32_T Sac_delta_f_cmd_grad_barrier_par;/* '<Root>/Inport57' */
extern real32_T Sac_delta_f_cmd_min_grad_par;/* '<Root>/Inport58' */
extern boolean_T DMC_NVRAMReset_par;   /* '<Root>/Inport72' */

#endif                                 /* RTW_HEADER_LaDMC_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
