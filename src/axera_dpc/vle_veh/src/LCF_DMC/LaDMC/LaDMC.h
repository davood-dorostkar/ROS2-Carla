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
 * File                             : LaDMC.h
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

#ifndef RTW_HEADER_LaDMC_h_
#define RTW_HEADER_LaDMC_h_
#include <math.h>
#ifndef LaDMC_COMMON_INCLUDES_
#define LaDMC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LaDMC_COMMON_INCLUDES_ */

#include "LaDMC_types.h"

/* Macros for accessing real-time model data structure */

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T fSteerAngle_deg;       /* '<S11>/Switch5' */
extern real32_T fSteerWhlAngle_deg;    /* '<S11>/Product1' */
extern real32_T fLonAccCmd;            /* '<S287>/Merge' */
extern uint8_T uiEPSRequest_nu;        /* '<S251>/Switch' */
extern uint8_T uiLonAccRequest_nu;     /* '<S286>/Switch' */

/* Model entry point functions */
extern void LaDMC_initialize(void);
extern void LaDMC_step(void);

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile real32_T Acf_no_of_items_in_sample;/* Referenced by: '<S168>/Constant' */
extern const volatile real32_T Adp_dyc_corr_fact_dec_x_sched[12];
                                 /* Referenced by: '<S152>/1-D Lookup Table1' */
extern const volatile real32_T Adp_dyc_corr_fact_dec_y_sched[12];
                                 /* Referenced by: '<S152>/1-D Lookup Table1' */
extern const volatile real32_T Adp_dyc_corr_fact_inc_x_sched[12];
                                  /* Referenced by: '<S152>/1-D Lookup Table' */
extern const volatile real32_T Adp_dyc_corr_fact_inc_y_sched[12];
                                  /* Referenced by: '<S152>/1-D Lookup Table' */
extern const volatile real32_T Adp_dyc_corr_factor_lower_limit;/* Referenced by: '<S159>/Constant' */
extern const volatile real32_T Adp_dyc_corr_factor_upper_limit;/* Referenced by: '<S159>/Constant1' */
extern const volatile real32_T DYC_Ay_Linear_Fading_Constant;/* Referenced by: '<S108>/Constant2' */
extern const volatile real32_T DYC_Ay_Linear_Treshold;/* Referenced by: '<S108>/Constant1' */
extern const volatile real32_T DYC_Filter_Omega;/* Referenced by: '<S126>/Constant' */
extern const volatile real32_T DYC_Filter_Tau;/* Referenced by: '<S126>/Constant1' */
extern const volatile real32_T DYC_Kappa_Angle_Hi_Ay_Corr_Fact;/* Referenced by: '<S108>/Constant' */
extern const volatile real32_T DYC_Kappa_Angle_T_X_Sch_Hi_Ay[12];
                                  /* Referenced by: '<S108>/1-D Lookup Table' */
extern const volatile real32_T DYC_Kappa_Angle_T_Y_Sch_Hi_Ay[12];
                                  /* Referenced by: '<S108>/1-D Lookup Table' */
extern const volatile real32_T DYC_Kappa_Coeff[4];/* Referenced by: '<S85>/Constant1' */
extern const volatile real32_T DYC_Long_Force_Comp_Cor_Factor;/* Referenced by: '<S111>/Constant' */
extern const volatile real32_T DYC_Max_Delta_F_Correction;/* Referenced by: '<S111>/Constant4' */
extern const volatile real32_T DYC_Max_Lateral_Acceleration;/* Referenced by: '<S111>/Constant3' */
extern const volatile real32_T Delta_F_Direct_Feedthrough_Par;/* Referenced by: '<S253>/Constant1' */
extern const volatile real32_T Delta_F_Gradient_Par;/* Referenced by: '<S253>/Constant' */
extern const volatile real32_T Dyc_boost_signal_reduction;/* Referenced by: '<S91>/Constant4' */
extern const volatile real32_T Dyc_kappa_A4_X_scheduling[11];
                                  /* Referenced by: '<S85>/1-D Lookup Table1' */
extern const volatile real32_T Dyc_kappa_A4_Y_scheduling[11];
                                  /* Referenced by: '<S85>/1-D Lookup Table1' */
extern const volatile real32_T Dyc_kappa_A4_factor;/* Referenced by: '<S85>/Constant4' */
extern const volatile real32_T Dyc_kappa_a2_boost_factor;/* Referenced by: '<S91>/Constant' */
extern const volatile real32_T Dyc_kappa_a2_x_scheduling_Sc[12];
                                   /* Referenced by: '<S92>/1-D Lookup Table' */
extern const volatile real32_T Dyc_kappa_a2_y_scheduling_Sc[12];
                                   /* Referenced by: '<S92>/1-D Lookup Table' */
extern const volatile real32_T Dyc_kappa_a3_factor;/* Referenced by: '<S85>/Constant3' */
extern const volatile real32_T Dyc_kappa_a3_x_scheduling[11];
                                   /* Referenced by: '<S85>/1-D Lookup Table' */
extern const volatile real32_T Dyc_kappa_a3_y_scheduling[11];
                                   /* Referenced by: '<S85>/1-D Lookup Table' */
extern const volatile real32_T Dyc_kappa_angle_ldp_corr_y_sch[12];
                                  /* Referenced by: '<S114>/1-D Lookup Table' */
extern const volatile real32_T Dyc_kappa_angle_lpf_corr_factor;/* Referenced by: '<S87>/Constant' */
extern const volatile real32_T Dyc_kappa_angle_t_x_schedul_gen[12];/* Referenced by:
                                                                    * '<S114>/1-D Lookup Table'
                                                                    * '<S114>/1-D Lookup Table3'
                                                                    */
extern const volatile real32_T Dyc_kappa_angle_t_y_schedul_nom[12];
                                 /* Referenced by: '<S114>/1-D Lookup Table3' */
extern const volatile real32_T Dyc_kappa_dot_boost_thrs;/* Referenced by: '<S91>/Constant1' */
extern const volatile real32_T Dyc_kappa_dot_filter_coeff;/* Referenced by: '<S93>/Constant' */
extern const volatile real32_T Dyc_time_constant_x_scheduling[12];
                                   /* Referenced by: '<S86>/1-D Lookup Table' */
extern const volatile real32_T Dyc_time_constant_y_scheduling[12];
                                   /* Referenced by: '<S86>/1-D Lookup Table' */
extern const volatile real32_T HEC_Kd_X_Scheduling[11];
                                 /* Referenced by: '<S127>/HEC_Kd_Scheduling' */
extern const volatile real32_T HEC_Kd_Y_Scheduling[11];
                                 /* Referenced by: '<S127>/HEC_Kd_Scheduling' */
extern const volatile real32_T Hec_r_dot_factor;/* Referenced by: '<S138>/Constant' */
extern const volatile real32_T Hec_r_dot_factor_Sc;/* Referenced by: '<S138>/Constant1' */
extern const volatile real32_T Hec_r_dot_factor_x_scheduling[11];
                                  /* Referenced by: '<S138>/1-D Lookup Table' */
extern const volatile real32_T Hec_r_dot_factor_x_scheduling_Sc[11];
                                 /* Referenced by: '<S138>/1-D Lookup Table1' */
extern const volatile real32_T Hec_r_dot_factor_y_scheduling[11];
                                  /* Referenced by: '<S138>/1-D Lookup Table' */
extern const volatile real32_T Hec_r_dot_factor_y_scheduling_Sc[11];
                                 /* Referenced by: '<S138>/1-D Lookup Table1' */
extern const volatile real32_T Hec_r_factor;/* Referenced by: '<S139>/Constant' */
extern const volatile real32_T Hec_r_factor_Sc;/* Referenced by: '<S139>/Constant1' */
extern const volatile real32_T Hec_r_pt1_factor;/* Referenced by: '<S81>/Constant' */
extern const volatile real32_T Hec_r_x_scheduling[11];
                                  /* Referenced by: '<S139>/1-D Lookup Table' */
extern const volatile real32_T Hec_r_x_scheduling_Sc[11];
                                 /* Referenced by: '<S139>/1-D Lookup Table1' */
extern const volatile real32_T Hec_r_y_scheduling[11];
                                  /* Referenced by: '<S139>/1-D Lookup Table' */
extern const volatile real32_T Hec_r_y_scheduling_Sc[11];
                                 /* Referenced by: '<S139>/1-D Lookup Table1' */
extern const volatile real32_T Hec_yaw_rate_filter_coeff;/* Referenced by: '<S137>/Constant' */
extern const volatile real32_T LAT_Kappa_Dot_Filter_Coeff;/* Referenced by: '<S184>/Constant1' */
extern const volatile real32_T LAT_Yaw_Rate_Filter_Coeff;/* Referenced by: '<S190>/Constant1' */
extern const volatile real32_T Lat_delta_f_dot_filter_coeff;/* Referenced by: '<S182>/Constant1' */
extern const volatile real32_T Lat_delta_f_offset;
                                /* Referenced by: '<S196>/LAT_Delta_F_Offset' */
extern const volatile real32_T Lat_delta_off_flt_initial_loops;
                   /* Referenced by: '<S201>/Lat_delta_off_flt_initial_loops' */
extern const volatile real32_T Lat_direct_feedthrough_kappa;/* Referenced by: '<S235>/Constant1' */
extern const volatile real32_T Lat_kappa_discharge_gradient;/* Referenced by: '<S250>/Constant1' */
extern const volatile real32_T Lat_kappa_discharge_min_error;/* Referenced by: '<S249>/Constant4' */
extern const volatile real32_T Lat_kappa_discharge_slope;/* Referenced by: '<S249>/Constant6' */
extern const volatile real32_T Lat_kappa_filter_coeff;/* Referenced by: '<S234>/Constant1' */
extern const volatile real32_T Lat_kappa_gradient_ldp;/* Referenced by: '<S248>/Constant1' */
extern const volatile real32_T Lat_kappa_lateral_error_slope;/* Referenced by: '<S249>/Constant2' */
extern const volatile real32_T Lat_kappa_linz_default_memshp;/* Referenced by: '<S243>/Constant' */
extern const volatile real32_T Lat_kappa_linz_error_slope;/* Referenced by: '<S236>/Constant4' */
extern const volatile real32_T Lat_kappa_linz_filt_err_hi_curv;/* Referenced by: '<S236>/Constant' */
extern const volatile real32_T Lat_kappa_linz_filt_max_error;/* Referenced by: '<S236>/Constant2' */
extern const volatile real32_T Lat_kappa_linz_filter_max_omega;/* Referenced by: '<S241>/Constant1' */
extern const volatile real32_T Lat_kappa_linz_filter_min_omega;/* Referenced by: '<S241>/Constant3' */
extern const volatile real32_T Lat_kappa_linz_head_mx_memshp;/* Referenced by: '<S239>/Constant5' */
extern const volatile real32_T Lat_kappa_linz_height_factor;/* Referenced by: '<S237>/Constant' */
extern const volatile real32_T Lat_kappa_linz_height_mx_memshp;/* Referenced by: '<S237>/Constant2' */
extern const volatile real32_T Lat_kappa_linz_lat_err_coeff;/* Referenced by:
                                                             * '<S244>/Constant1'
                                                             * '<S245>/Constant1'
                                                             */
extern const volatile real32_T Lat_kappa_linz_lat_error_slope;/* Referenced by:
                                                               * '<S238>/Constant3'
                                                               * '<S239>/Constant3'
                                                               */
extern const volatile real32_T Lat_kappa_linz_max_head_error;/* Referenced by: '<S239>/Constant1' */
extern const volatile real32_T Lat_kappa_linz_max_lat_error;/* Referenced by: '<S238>/Constant1' */
extern const volatile real32_T Lat_kappa_linz_min_crv_progress;/* Referenced by: '<S237>/Constant3' */
extern const volatile real32_T Lat_kappa_linz_staight_mx_memshp;/* Referenced by: '<S240>/Constant1' */
extern const volatile real32_T Lat_kappa_linz_straight_weight;/* Referenced by: '<S240>/Constant' */
extern const volatile real32_T Lat_kappa_linz_wt_fact_progress;/* Referenced by: '<S237>/Constant4' */
extern const volatile real32_T Lat_kappa_max_lateral_error;/* Referenced by: '<S249>/Constant' */
extern const volatile real32_T Lat_kppa_min_omega_x_scheduling[12];
                                  /* Referenced by: '<S241>/1-D Lookup Table' */
extern const volatile real32_T Lat_kppa_min_omega_y_scheduling[12];
                                  /* Referenced by: '<S241>/1-D Lookup Table' */
extern const volatile real32_T Lat_ldp_startup_time;/* Referenced by: '<S187>/Constant9' */
extern const volatile real32_T Lat_max_kappa_grad_x_scheduling[12];
                                  /* Referenced by: '<S250>/1-D Lookup Table' */
extern const volatile real32_T Lat_max_kappa_grad_y_scheduling[12];
                                  /* Referenced by: '<S250>/1-D Lookup Table' */
extern const volatile real32_T Lat_max_kappa_gradient;/* Referenced by: '<S250>/Constant' */
extern const volatile real32_T Lat_oc_fast_ki;
                                    /* Referenced by: '<S206>/Lat_oc_fast_ki' */
extern const volatile real32_T Lat_oc_implaus_lateral_error;
                      /* Referenced by: '<S210>/Lat_oc_implaus_lateral_error' */
extern const volatile real32_T Lat_oc_kappa_cmd_filter_coeff;
                     /* Referenced by: '<S232>/Lat_oc_kappa_cmd_filter_coeff' */
extern const volatile real32_T Lat_oc_kappa_ffwd_filter_coeff;
                    /* Referenced by: '<S213>/Lat_oc_kappa_ffwd_filter_coeff' */
extern const volatile real32_T Lat_oc_kappa_max_driver_torque;
                    /* Referenced by: '<S223>/Lat_oc_kappa_max_driver_torque' */
extern const volatile real32_T Lat_oc_kappa_max_heading_error;/* Referenced by:
                                                               * '<S220>/Lat_oc_kappa_max_heading_error'
                                                               * '<S224>/Lat_oc_kappa_max_heading_error'
                                                               */
extern const volatile real32_T Lat_oc_kappa_min_head_err_qual;/* Referenced by:
                                                               * '<S220>/Lat_oc_kappa_min_head_err_qual'
                                                               * '<S227>/Lat_oc_kappa_min_head_err_qual'
                                                               */
extern const volatile real32_T Lat_oc_kappa_min_latency;
                          /* Referenced by: '<S220>/Lat_oc_kappa_min_latency' */
extern const volatile real32_T Lat_oc_max_delta_f_dot;
                            /* Referenced by: '<S222>/Lat_oc_max_delta_f_dot' */
extern const volatile real32_T Lat_oc_max_delta_offset;
                           /* Referenced by: '<S200>/Lat_oc_max_delta_offset' */
extern const volatile real32_T Lat_oc_max_delta_offset_kappa;
                     /* Referenced by: '<S200>/Lat_oc_max_delta_offset_kappa' */
extern const volatile real32_T Lat_oc_max_kappa;
                                  /* Referenced by: '<S221>/Lat_oc_max_kappa' */
extern const volatile real32_T Lat_oc_max_lateral_accel;
                          /* Referenced by: '<S225>/Lat_oc_max_lateral_accel' */
extern const volatile real32_T Lat_oc_max_lateral_error;
                          /* Referenced by: '<S206>/Lat_oc_max_lateral_error' */
extern const volatile real32_T Lat_oc_max_offset_rate;
                            /* Referenced by: '<S215>/Lat_oc_max_offset_rate' */
extern const volatile real32_T Lat_oc_max_velocity;
                               /* Referenced by: '<S226>/Lat_oc_max_velocity' */
extern const volatile real32_T Lat_oc_min_kappa_quality;
                          /* Referenced by: '<S228>/Lat_oc_min_kappa_quality' */
extern const volatile real32_T Lat_oc_min_velocity;
                               /* Referenced by: '<S229>/Lat_oc_min_velocity' */
extern const volatile real32_T Lat_oc_min_velocity_dys;
                           /* Referenced by: '<S212>/Lat_oc_min_velocity_dys' */
extern const volatile real32_T Lat_oc_minimum_latency;
                            /* Referenced by: '<S204>/Lat_oc_minimum_latency' */
extern const volatile real32_T Lat_oc_minimum_latency_shrt;
                       /* Referenced by: '<S205>/Lat_oc_minimum_latency_shrt' */
extern const volatile real32_T Lat_status_first_run_time;
                         /* Referenced by: '<S188>/Lat_status_first_run_time' */
extern const volatile real32_T Lat_yaw_rate_dot_filter_coeff;/* Referenced by: '<S140>/Constant' */
extern const volatile real32_T Lco_curv_dot_filter_fall_coeff;/* Referenced by: '<S277>/Constant2' */
extern const volatile real32_T Lco_curv_dot_filter_risng_coeff;/* Referenced by: '<S277>/Constant1' */
extern const volatile real32_T Lco_curv_dot_preload_enh_factor;/* Referenced by: '<S276>/Constant' */
extern const volatile real32_T Lco_filter_falling_coeff;/* Referenced by: '<S268>/Constant2' */
extern const volatile real32_T Lco_filter_rising_coeff;/* Referenced by: '<S268>/Constant1' */
extern const volatile real32_T Lco_min_curvature_command;/* Referenced by: '<S267>/Constant' */
extern const volatile real32_T Lco_min_curvature_dot;/* Referenced by: '<S275>/Constant' */
extern const volatile real32_T Lco_min_curvature_dot_slop;/* Referenced by: '<S275>/Constant2' */
extern const volatile real32_T Lco_min_curvature_slope;/* Referenced by: '<S267>/Constant2' */
extern const volatile real32_T Lco_preload_enhancement_factor;/* Referenced by: '<S266>/Constant' */
extern const volatile real32_T Lon_Acc_Gradient_Par;/* Referenced by: '<S288>/Constant' */
extern const volatile real32_T Max_Wait_Counter_x[9];
                                  /* Referenced by: '<S153>/1-D Lookup Table' */
extern const volatile real32_T Max_Wait_Counter_y[9];
                                  /* Referenced by: '<S153>/1-D Lookup Table' */
extern const volatile uint32_T Sac_controller_mode_2;/* Referenced by: '<S6>/Constant6' */
extern const volatile real32_T Sac_delta_f_counter_steer_grad;/* Referenced by: '<S280>/Constant' */
extern const volatile real32_T Sac_delta_psi_dot_factor;
                          /* Referenced by: '<S127>/Sac_delta_psi_dot_factor' */
extern const volatile real32_T Sac_delta_psi_dot_maf_length;
                      /* Referenced by: '<S128>/Sac_delta_psi_dot_maf_length' */
extern const volatile real32_T Sac_delta_psi_pt1_filter_coeff;
                    /* Referenced by: '<S128>/Sac_delta_psi_pt1_filter_coeff' */
extern const volatile real32_T Sac_one_by_ts;/* Referenced by:
                                              * '<S79>/Sac_one_by_ts1'
                                              * '<S128>/Sac_one_by_ts'
                                              */
extern const volatile real32_T Sac_osd_delta_ys_dot_coeff;
                         /* Referenced by: '<S79>/Sac_osd_delta_ys_dot_coeff' */
extern const volatile real32_T Sac_osd_delta_ys_dot_feedback;
                      /* Referenced by: '<S79>/Sac_osd_delta_ys_dot_feedback' */
extern const volatile real32_T Sac_parity_time_const_barrier;/* Referenced by: '<S10>/Constant1' */
extern const volatile real32_T Sac_ts; /* Referenced by:
                                        * '<S79>/Constant1'
                                        * '<S80>/Sac_ts'
                                        * '<S188>/Sac_ts'
                                        * '<S254>/Sac_ts2'
                                        * '<S128>/Constant2'
                                        * '<S132>/Constant1'
                                        * '<S200>/Sac_ts'
                                        * '<S289>/Sac_ts2'
                                        */
extern const volatile real32_T Sac_yrc_ctrl_cmd_fading_x_sched[11];
                     /* Referenced by: '<S134>/SAC_Yrc_Ctrl_Cmd_Fading_Sched' */
extern const volatile real32_T Sac_yrc_ctrl_cmd_fading_y_sched[11];
                     /* Referenced by: '<S134>/SAC_Yrc_Ctrl_Cmd_Fading_Sched' */
extern const volatile real32_T Sac_yrc_ki_gain;
                                    /* Referenced by: '<S80>/Sac_yrc_ki_gain' */
extern const volatile real32_T Sac_yrc_kp_gain;/* Referenced by: '<S80>/Constant1' */
extern const volatile real32_T Sac_yrc_kp_pt1_gain;/* Referenced by: '<S80>/Constant2' */
extern const volatile real32_T Sac_yrc_kp_pt1_x_scheduling[11];
                          /* Referenced by: '<S80>/SAC_Yrc_Kp_Pt1_Scheduling' */
extern const volatile real32_T Sac_yrc_kp_pt1_y_scheduling[11];
                          /* Referenced by: '<S80>/SAC_Yrc_Kp_Pt1_Scheduling' */
extern const volatile real32_T Sac_yrc_kp_x_scheduling[11];
                              /* Referenced by: '<S80>/SAC_Yrc_Kp_Scheduling' */
extern const volatile real32_T Sac_yrc_kp_y_scheduling[11];
                              /* Referenced by: '<S80>/SAC_Yrc_Kp_Scheduling' */
extern const volatile real32_T Sac_yrc_loop_gain_corr;
                             /* Referenced by: '<S80>/Sac_yrc_loop_gain_corr' */
extern const volatile real32_T Sac_yrc_pt1_filter_coeff;
                          /* Referenced by: '<S132>/Sac_yrc_pt1_filter_coeff' */
extern const volatile real32_T Sat_dynamic_enhancement_factor;/* Referenced by: '<S74>/Constant' */
extern const volatile real32_T Sat_thrs_control_kp;/* Referenced by: '<S62>/Constant' */
extern const volatile real32_T SpeedSegmentLookup1[10];/* Referenced by: '<S151>/Constant' */
extern const volatile real32_T SpeedSegmentLookup2[9];/* Referenced by: '<S151>/Constant1' */
extern const volatile real32_T TDF_Derating_Switch_Min_Time;/* Referenced by: '<S57>/Constant' */
extern const volatile real32_T TDF_Max_Derating_Factor;/* Referenced by: '<S56>/Constant1' */
extern const volatile real32_T TDF_Switch_Falling_Rate;/* Referenced by: '<S54>/Constant1' */
extern const volatile real32_T TDF_Switch_Rising_Rate;/* Referenced by: '<S54>/Constant' */
extern const volatile real32_T Tdf_comp_factor_filter_coeff;/* Referenced by: '<S50>/Constant1' */
extern const volatile real32_T Tdf_comp_filter_min_residual;/* Referenced by: '<S50>/Constant6' */
extern const volatile real32_T Tdf_control_err_derating_slope;/* Referenced by: '<S46>/Constant2' */
extern const volatile real32_T Tdf_control_error_threshold;/* Referenced by: '<S46>/Constant' */
extern const volatile real32_T Tdf_der_thrs_curv_x_scheduling[5];
                                   /* Referenced by: '<S44>/1-D Lookup Table' */
extern const volatile real32_T Tdf_der_thrs_curv_y_scheduling[5];
                                   /* Referenced by: '<S44>/1-D Lookup Table' */
extern const volatile real32_T Tdf_derating_end;/* Referenced by: '<S12>/Constant1' */
extern const volatile real32_T Tdf_derating_slope_x_scheduling[11];
                                   /* Referenced by: '<S42>/1-D Lookup Table' */
extern const volatile real32_T Tdf_derating_slope_y_scheduling[11];
                                   /* Referenced by: '<S42>/1-D Lookup Table' */
extern const volatile real32_T Tdf_derating_start;/* Referenced by: '<S12>/Constant' */
extern const volatile real32_T Tdf_derating_thrs_x_scheduling[11];
                                  /* Referenced by: '<S44>/1-D Lookup Table1' */
extern const volatile real32_T Tdf_derating_thrs_y_scheduling[11];
                                  /* Referenced by: '<S44>/1-D Lookup Table1' */
extern const volatile real32_T Tdf_idle_enable_min_latency;/* Referenced by: '<S19>/Constant10' */
extern const volatile real32_T Tdf_idle_max_curvature_error;/* Referenced by: '<S19>/Constant6' */
extern const volatile real32_T Tdf_idle_max_delta_f_dot;/* Referenced by: '<S19>/Constant4' */
extern const volatile real32_T Tdf_idle_max_heading_error;/* Referenced by: '<S19>/Constant8' */
extern const volatile real32_T Tdf_idle_max_kappa;/* Referenced by: '<S19>/Constant3' */
extern const volatile real32_T Tdf_idle_max_lateral_accel;/* Referenced by: '<S19>/Constant2' */
extern const volatile real32_T Tdf_idle_max_lateral_error;/* Referenced by: '<S19>/Constant5' */
extern const volatile real32_T Tdf_idle_max_steer_angle;/* Referenced by: '<S19>/Constant9' */
extern const volatile real32_T Tdf_idle_max_torque_request;/* Referenced by:
                                                            * '<S19>/Constant'
                                                            * '<S19>/Constant1'
                                                            */
extern const volatile real32_T Tdf_idle_max_yaw_rate;/* Referenced by: '<S19>/Constant7' */
extern const volatile real32_T Tdf_maximum_delta_f_dot;/* Referenced by: '<S47>/Constant1' */
extern const volatile real32_T Tdf_min_steer_trq_cls_x_schedul[11];
                                  /* Referenced by: '<S17>/1-D Lookup Table2' */
extern const volatile real32_T Tdf_min_steer_trq_cls_y_schedul[11];
                                  /* Referenced by: '<S17>/1-D Lookup Table2' */
extern const volatile real32_T Tdf_out_scale_filt_min_residual;/* Referenced by: '<S13>/Constant6' */
extern const volatile real32_T Tdf_sac_pritaty_omega;/* Referenced by: '<S47>/Constant' */
extern const volatile real32_T Tdf_st_wheel_unbalance_factor;/* Referenced by: '<S25>/Constant' */
extern const volatile real32_T Tdf_steer_torque_comp_slope;/* Referenced by: '<S31>/Constant1' */
extern const volatile real32_T Tdf_steer_torque_comp_slope_ldp;/* Referenced by: '<S31>/Constant2' */
extern const volatile real32_T Tdf_steer_torque_comp_thrs_ldp;/* Referenced by: '<S17>/Constant' */
extern const volatile real32_T Tdf_steer_trq_cmp_ldp_x_schedul[11];
                                   /* Referenced by: '<S17>/1-D Lookup Table' */
extern const volatile real32_T Tdf_steer_trq_cmp_ldp_y_schedul[11];
                                   /* Referenced by: '<S17>/1-D Lookup Table' */
extern const volatile real32_T Tdf_steer_trq_cmp_slp_x_schedul[12];
                                   /* Referenced by: '<S31>/1-D Lookup Table' */
extern const volatile real32_T Tdf_steer_trq_cmp_slp_y_schedul[12];
                                   /* Referenced by: '<S31>/1-D Lookup Table' */
extern const volatile real32_T Tdf_steer_trq_comp_reduced_thrs;/* Referenced by: '<S17>/Constant7' */
extern const volatile real32_T Tdf_torque_der_filt_coeff_hf;/* Referenced by: '<S14>/Constant3' */
extern const volatile real32_T Tdf_torque_derating_filt_coeff;/* Referenced by: '<S13>/Constant1' */
extern const volatile real32_T Tdf_trq_der_init_corr_x_schedul[11];
                                   /* Referenced by: '<S36>/1-D Lookup Table' */
extern const volatile real32_T Tdf_trq_der_init_corr_y_schedul[11];
                                   /* Referenced by: '<S36>/1-D Lookup Table' */
extern const volatile real32_T Tdf_trq_derating_max_curv;/* Referenced by: '<S44>/Constant' */
extern const volatile real32_T Tdf_trq_derating_max_init_trq;/* Referenced by: '<S36>/Constant' */
extern const volatile real32_T Tdf_trq_derating_slope_hf_path;/* Referenced by: '<S41>/Constant' */
extern const volatile real32_T Tdf_trq_derating_slope_hi_sens;/* Referenced by: '<S42>/Constant2' */
extern const volatile real32_T Tdf_trq_derating_thrs_hi_sens;/* Referenced by: '<S21>/Constant3' */
extern const volatile real32_T Tdf_trq_derating_thrs_x_schedul[11];
                                   /* Referenced by: '<S21>/1-D Lookup Table' */
extern const volatile real32_T Tdf_trq_derating_thrs_y_schedul[11];
                                   /* Referenced by: '<S21>/1-D Lookup Table' */
extern const volatile real32_T Tdf_trq_filter_coeff;/* Referenced by:
                                                     * '<S33>/Constant'
                                                     * '<S49>/Constant1'
                                                     * '<S49>/Constant3'
                                                     */
extern const volatile real32_T Tdf_trq_min_stiffness_inc;/* Referenced by: '<S20>/Constant' */
extern const volatile real32_T Tdf_vel_derating_filt_coeff;/* Referenced by:
                                                            * '<S45>/Constant3'
                                                            * '<S59>/Constant3'
                                                            */
extern const volatile real32_T Tdf_velocity_derating_slope;/* Referenced by: '<S48>/Constant2' */
extern const volatile real32_T Tdf_velocity_derating_threshold;/* Referenced by: '<S48>/Constant' */
extern const volatile real32_T VEH_Force_Long_Tire_Factor;/* Referenced by: '<S111>/Constant2' */
extern const volatile real32_T VEH_Vehicle_Reduced_Mass;/* Referenced by: '<S111>/Constant1' */
extern const volatile real32_T VEH_Vehicle_Selfsteering_Factor;/* Referenced by: '<S115>/Constant1' */
extern const volatile real32_T VEH_Vehicle_Wheel_Base;/* Referenced by:
                                                       * '<S64>/Constant'
                                                       * '<S65>/Constant1'
                                                       * '<S115>/Constant'
                                                       */

/* Declaration for custom storage class: Global */
extern real32_T ADP_Dyc_Corr_Factor;   /* '<S181>/Add' */
extern real32_T ADP_Dyc_Corr_Factor_state;/* '<S181>/Unit Delay' */
extern real32_T CAM_Lateral_Error_Sign;/* '<S167>/Sign' */
extern real32_T Corretion_Factor_Left_Vect[9];/* '<S82>/Unit Delay1' */
extern real32_T Corretion_Factor_Right_Vect[9];/* '<S82>/Unit Delay' */
extern real32_T DMC_Integtrator_Output;/* '<S104>/Unit Delay' */
extern real32_T DMC_Integtrator_Output1;/* '<S103>/Unit Delay' */
extern real32_T DMC_Integtrator_Output2;/* '<S107>/Unit Delay' */
extern real32_T DMC_Integtrator_Output3;/* '<S106>/Unit Delay' */
extern real32_T DMC_Integtrator_Output4;/* '<S105>/Unit Delay' */
extern real32_T DYC_Boost_Filter_Output_state;/* '<S126>/Discrete-Time Integrator' */
extern real32_T DYC_Filter_Kappa_Command;/* '<S85>/Switch2' */
extern real32_T DYC_Kappa_Dot_Filter_state;/* '<S93>/Unit Delay' */
extern real32_T DYC_Steer_Angle_Feedforward;/* '<S87>/Product5' */
extern real32_T Delta_F_Rate_Limitation_state;/* '<S254>/Unit Delay' */
extern real32_T HEC_Yaw_Rate_Filter;   /* '<S112>/Add' */
extern real32_T HEC_Yaw_Rate_Filter_state;/* '<S137>/Unit Delay' */
extern real32_T HEC_Yaw_Rate_Filter_state2;/* '<S112>/Unit Delay' */
extern boolean_T Initialisation_Flag;  /* '<S179>/Constant' */
extern boolean_T Initialisation_Flag_state;/* '<S149>/Unit Delay' */
extern real32_T LAT_Delta_F_Dot_Filter_state;/* '<S182>/Unit Delay' */
extern real32_T LAT_Delta_F_Vdy_Offset_pre;/* '<S192>/Unit Delay' */
extern real32_T LAT_Eps_Torque_Filter_state;/* '<S132>/Unit Delay' */
extern real32_T LAT_Filtered_Kappa_Cmd_pre;/* '<S246>/Unit Delay' */
extern real32_T LAT_Filtered_Kappa_Cmd_pre_pre;/* '<S246>/Unit Delay1' */
extern real32_T LAT_Filtered_Kappa_Cmd_pre_pre_pre;/* '<S246>/Unit Delay2' */
extern real32_T LAT_Filtered_Kappa_state;/* '<S234>/Unit Delay' */
extern real32_T LAT_Grad_Limited_Kappa_state;/* '<S235>/Unit Delay' */
extern real32_T LAT_Kappa_Command_filt_state;/* '<S232>/Unit Delay' */
extern real32_T LAT_Kappa_Command_filt_state1;/* '<S213>/Unit Delay' */
extern real32_T LAT_Kappa_Command_pre; /* '<S194>/Unit Delay' */
extern real32_T LAT_Kappa_Command_state;/* '<S231>/Unit Delay' */
extern real32_T LAT_Kappa_Dot_Filter_state;/* '<S184>/Unit Delay' */
extern real32_T LAT_Kappa_Linz_Filter_Output;/* '<S242>/Unit Delay' */
extern real32_T LAT_Kappa_Linz_Heading_Err_Mem_state;/* '<S245>/Unit Delay' */
extern real32_T LAT_Kappa_Linz_Lat_Error_Mem_state;/* '<S244>/Unit Delay' */
extern real32_T LAT_OC_State_Preload;  /* '<S214>/Switch3' */
extern boolean_T LAT_Oc_Cal_Hold_Flag; /* '<S204>/Relational Operator' */
extern boolean_T LAT_Oc_Cal_Hold_Flag_Shrt;/* '<S205>/Relational Operator' */
extern boolean_T LAT_Oc_Disable_Flag;  /* '<S207>/Or Operator' */
extern boolean_T LAT_Oc_Dys_Active;    /* '<S208>/Not Operator' */
extern real32_T LAT_Oc_Filtered_Kappa_Cam;/* '<S231>/Add' */
extern boolean_T LAT_Oc_High_Driver_Torque;/* '<S198>/Logical Operator3' */
extern boolean_T LAT_Oc_Implaus_Lateral_Error;/* '<S210>/Relational Operator' */
extern real32_T LAT_Oc_Integrator_Input;/* '<S206>/Product' */
extern real32_T LAT_Oc_Integrator_Input_Kappa;/* '<S201>/Product2' */
extern real32_T LAT_Oc_Integrator_Input_Kappa_dbg;/* '<S201>/Switch1' */
extern real32_T LAT_Oc_Integrator_Output;/* '<S6>/Constant1' */
extern real32_T LAT_Oc_Integrator_Sat_Out;/* '<S216>/MinMax2' */
extern boolean_T LAT_Oc_Kappa_Active;  /* '<S198>/Logical Operator4' */
extern boolean_T LAT_Oc_Kappa_Con_Enb_Flag;/* '<S230>/Relational Operator2' */
extern real32_T LAT_Oc_Kappa_Latency_state;/* '<S230>/Unit Delay' */
extern real32_T LAT_Oc_Kappa_Status_dbg;/* '<S219>/Add' */
extern boolean_T LAT_Oc_Max_Delta_F_Dot_Flag;/* '<S222>/Relational Operator' */
extern boolean_T LAT_Oc_Max_Drv_Trq_Flag;/* '<S223>/Relational Operator' */
extern boolean_T LAT_Oc_Max_Flt_Kappa_Cmd_Flag;/* '<S211>/Logical Operator' */
extern boolean_T LAT_Oc_Max_Hea_Err_Flag;/* '<S224>/Relational Operator' */
extern boolean_T LAT_Oc_Max_Kappa_Cmd_Flag;/* '<S221>/Relational Operator' */
extern boolean_T LAT_Oc_Max_Lat_Acc_Flag;/* '<S225>/Relational Operator' */
extern boolean_T LAT_Oc_Min_Veh_Vel_Flag;/* '<S212>/Relational Operator' */
extern real32_T LAT_Oc_Sat_Integrator_state;/* '<S200>/Unit Delay2' */
extern boolean_T LAT_Oc_Trigger_Flag_Kappa;/* '<S202>/Logical Operator' */
extern real32_T LAT_Oc_Trigger_Flag_Kappa_state;/* '<S201>/Unit Delay' */
extern real32_T LAT_Oc_offset_filter_omega;/* '<S201>/Switch3' */
extern real32_T LAT_Pt1_Output_Undelayed_pre;/* '<S268>/Unit Delay' */
extern real32_T LAT_Pt1_Output_Undelayed_pre2;/* '<S277>/Unit Delay' */
extern real32_T LAT_Sat_Dynamic_Threshold_int;/* '<S69>/Unit Delay' */
extern real32_T LAT_Sat_Dynamic_Threshold_state;/* '<S62>/Unit Delay' */
extern boolean_T LAT_Status_Firsst_Run_pre;/* '<S203>/Unit Delay1' */
extern boolean_T LAT_Status_First_Run_state;/* '<S214>/Unit Delay3' */
extern real32_T LAT_Stiffness_Request_Factor_pre;/* '<S20>/Unit Delay' */
extern boolean_T LAT_Vdy_Offset_Used_pre;/* '<S200>/Unit Delay1' */
extern real32_T LAT_Velocity_Derating_Factor_state;/* '<S45>/Unit Delay' */
extern real32_T LAT_Yaw_Rate_Filter_state;/* '<S190>/Unit Delay' */
extern real32_T LDC_Sac_Parity_Filter_state;/* '<S70>/Unit Delay' */
extern boolean_T LDP_Active_state1;    /* '<S187>/Unit Delay' */
extern boolean_T LDP_Active_state2;    /* '<S187>/Unit Delay1' */
extern real32_T LKC_Delta_Psi_pre1;    /* '<S128>/Unit Delay1' */
extern real32_T LKC_Delta_Psi_pre2;    /* '<S128>/Unit Delay2' */
extern real32_T LKC_Delta_Psi_pre3;    /* '<S128>/Unit Delay3' */
extern real32_T LKC_Delta_Psi_pre4;    /* '<S128>/Unit Delay4' */
extern real32_T LKC_Delta_Psi_pre5;    /* '<S128>/Unit Delay5' */
extern real32_T LKC_Delta_Psi_pre6;    /* '<S128>/Unit Delay6' */
extern real32_T LKC_Delta_Psi_pre7;    /* '<S128>/Unit Delay7' */
extern real32_T LKC_Delta_Psi_pre8;    /* '<S128>/Unit Delay8' */
extern real32_T LKC_Delta_Psi_pre9;    /* '<S128>/Unit Delay9' */
extern real32_T LKC_Delta_Ys_pre;      /* '<S79>/Unit Delay1' */
extern uint16_T LTLE_Waiting_Counter_state;/* '<S144>/Unit Delay3' */
extern uint16_T LTRE_Waiting_Counter_state;/* '<S144>/Unit Delay2' */
extern real32_T Last_Seg_Correction_Factor_Left_Scal_state;/* '<S151>/Unit Delay1' */
extern real32_T Last_Seg_Correction_Factor_Right_Scal_state;/* '<S151>/Unit Delay2' */
extern real32_T Lat_ldp_startup_time_state;/* '<S187>/Unit Delay2' */
extern real32_T Lat_oc_minimum_latency_shrt_state;/* '<S205>/Unit Delay' */
extern real32_T Lat_oc_minimum_latency_state;/* '<S204>/Unit Delay' */
extern real32_T Lat_status_first_run_timer_state;/* '<S188>/Unit Delay2' */
extern real32_T Lateral_Error_Delta;   /* '<S167>/Subtract' */
extern real32_T Lateral_Error_Invalid_state;/* '<S167>/Resettable Delay4' */
extern real32_T Lateral_Error_Mean;    /* '<S167>/Divide' */
extern real32_T Lateral_Error_Mean_state;/* '<S167>/Resettable Delay5' */
extern real32_T Lon_Acc_Rate_Limitation_state;/* '<S289>/Unit Delay' */
extern real32_T Mean_Sample_Update;    /* '<S168>/Unit Delay1' */
extern real32_T Mean_Sample_Update_pre1;/* '<S167>/Resettable Delay' */
extern real32_T Mean_Sample_Update_pre2;/* '<S167>/Resettable Delay1' */
extern real32_T Mean_Sample_Update_pre3;/* '<S167>/Resettable Delay2' */
extern real32_T Mean_Sample_Update_pre4;/* '<S167>/Resettable Delay3' */
extern real32_T Mean_Sample_Update_sum;/* '<S168>/Resettable Delay1' */
extern real32_T Mean_Vehicle_Velocity; /* '<S168>/Unit Delay3' */
extern real32_T Mean_Vehicle_Velocity_sum;/* '<S168>/Resettable Delay3' */
extern real32_T Mean_kappa_command;    /* '<S168>/Unit Delay2' */
extern real32_T Mean_kappa_command_sum;/* '<S168>/Resettable Delay2' */
extern boolean_T New_Update_Aval;      /* '<S168>/Unit Delay' */
extern real32_T New_Update_Aval_sum;   /* '<S168>/Resettable Delay' */
extern uint16_T RTLE_Waiting_Counter_state;/* '<S144>/Unit Delay1' */
extern uint16_T RTRE_Waiting_Counter_state;/* '<S144>/Unit Delay' */
extern real32_T SAC_Angle_Command_Corr;/* '<S6>/Constant10' */
extern real32_T SAC_Angle_Command_Rate_Limiter_state;/* '<S10>/Unit Delay' */
extern real32_T SAC_Angle_Command_Yawrate_Fback;/* '<S81>/Switch' */
extern real32_T SAC_Arbitrated_Angle_Cmd;/* '<S77>/Switch' */
extern real32_T SAC_Arbitrated_Angle_Cmd_Raw;/* '<S84>/Gain1' */
extern real32_T SAC_Compensation_Angle_Command;/* '<S79>/Add1' */
extern real32_T SAC_Control_Error;     /* '<S4>/Subtract' */
extern real32_T SAC_Control_Error_Eps; /* '<S11>/Subtract1' */
extern real32_T SAC_Control_Error_pre; /* '<S5>/Unit Delay' */
extern real32_T SAC_Control_Error_state;/* '<S1>/Unit Delay2' */
extern real32_T SAC_Delta_Psi_Dot_state;/* '<S130>/Unit Delay' */
extern real32_T SAC_Delta_Psi_state;   /* '<S128>/Unit Delay' */
extern real32_T SAC_Derated_Angle_Command;/* '<S9>/Add' */
extern boolean_T SAC_Disable_pre;      /* '<S132>/Unit Delay1' */
extern boolean_T SAC_Disable_pre_pre;  /* '<S132>/Unit Delay2' */
extern real32_T SAC_Integrator_Sat_Out;/* '<S80>/Unit Delay1' */
extern real32_T SAC_Pt1_Filter1_state; /* '<S129>/Unit Delay' */
extern real32_T SAC_Rate_Lim_Angle_Command;/* '<S10>/Add1' */
extern real32_T SAC_Trq_Derating_Factor;/* '<S12>/Switch3' */
extern real32_T SAC_Yrc_Angle_Command; /* '<S80>/Switch4' */
extern real32_T SAC_Yrc_Control_Error; /* '<S80>/Add' */
extern real32_T SAT_Req_Dyn_Steer_Angle_Max;/* '<S74>/Min' */
extern real32_T SAT_Req_Steer_Angle_Max;/* '<S75>/Min' */
extern real32_T SAT_Saturated_Angle_Command;/* '<S67>/Switch' */
extern real32_T TDF_Composite_Derating_Factor;/* '<S16>/Switch1' */
extern real32_T TDF_Control_Error_Factor_state;/* '<S49>/Unit Delay' */
extern real32_T TDF_Derating_Time_state;/* '<S57>/Unit Delay' */
extern boolean_T TDF_Driver_Counter_Steering;/* '<S17>/LessThanOrEqual' */
extern real32_T TDF_Flt_Drv_Comp_Factor_state;/* '<S50>/Unit Delay' */
extern real32_T TDF_Idle_Derating_Factor;/* '<S19>/Switch1' */
extern real32_T TDF_Idle_Enable_Condition_state;/* '<S35>/Unit Delay' */
extern real32_T TDF_Ldp_Override_Factor_state;/* '<S12>/Unit Delay' */
extern real32_T TDF_Max_Steer_Torque;  /* '<S38>/Max1' */
extern real32_T TDF_Saturated_Angle_Command_Error_state;/* '<S47>/Unit Delay' */
extern real32_T TDF_Selected_State_Derating_Factor;/* '<S24>/Subtract' */
extern real32_T TDF_Selected_Torque_Source;/* '<S29>/Multiport Switch' */
extern real32_T TDF_State_Derating_Factor_Comp_state;/* '<S54>/Unit Delay' */
extern real32_T TDF_Steer_Torque_Sample_pre;/* '<S39>/Unit Delay' */
extern real32_T TDF_Torque_Der_Factor_HF_Path;/* '<S41>/Max' */
extern real32_T TDF_Torque_Der_Factor_HF_Path_state;/* '<S14>/Unit Delay' */
extern real32_T TDF_Torque_Derating_Factor;/* '<S40>/Max' */
extern real32_T TDF_Torque_Derating_Factor_state;/* '<S13>/Unit Delay' */
extern real32_T TDF_Torque_Derating_Slop_Arb;/* '<S42>/Add' */
extern real32_T TDF_Torque_Request_Factor;/* '<S30>/Max' */
extern real32_T TDF_Torque_Request_Factor_state;/* '<S33>/Unit Delay' */
extern real32_T TDF_Trq_Derating_Threshold_Arb;/* '<S43>/Add' */
extern real32_T TDF_Vehicle_Steer_Torque_Factor;/* '<S32>/Max1' */
extern real32_T VEH_Abs_Steer_Torque_Comp_pre;/* '<S38>/Unit Delay1' */
extern real32_T VEH_Delta_F_Oc;        /* '<S195>/Gain' */
extern real32_T VEH_Delta_F_Offset;    /* '<S196>/Add' */
extern real32_T VEH_Delta_F_Pre;       /* '<S1>/Unit Delay' */
extern real32_T VEH_Delta_F_pre1;      /* '<S282>/Unit Delay' */
extern real32_T VEH_Delta_F_pre2;      /* '<S282>/Unit Delay1' */
extern real32_T VEH_Delta_F_pre3;      /* '<S282>/Unit Delay2' */
extern real32_T VEH_Steer_Torque_Comp; /* '<S59>/Add' */
extern real32_T VEH_Steer_Torque_Comp_state;/* '<S59>/Unit Delay' */
extern real32_T VEH_Yaw_Rate_Dot_sate; /* '<S140>/Unit Delay' */
extern real32_T VEH_Yaw_Rate_state;    /* '<S136>/Unit Delay' */
extern real32_T Veh_Lat_Acc_Filt;      /* '<S147>/Unit Delay' */
extern uint32_T Vehicle_Speed_Segment_state;/* '<S151>/Unit Delay' */
extern real32_T fDeltaFCmd_State;      /* '<S11>/Unit Delay1' */
extern uint8_T uiDeltaF_Request_nu_pre;/* '<S251>/Unit Delay' */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/Display' : Unused code path elimination
 * Block '<S16>/Display' : Unused code path elimination
 * Block '<S16>/Display1' : Unused code path elimination
 * Block '<S16>/Signal Conversion' : Unused code path elimination
 * Block '<S17>/Abs' : Unused code path elimination
 * Block '<S17>/Constant3' : Unused code path elimination
 * Block '<S17>/Constant5' : Unused code path elimination
 * Block '<S17>/Gain2' : Unused code path elimination
 * Block '<S17>/LessThanOrEqual1' : Unused code path elimination
 * Block '<S17>/LessThanOrEqual2' : Unused code path elimination
 * Block '<S17>/Min' : Unused code path elimination
 * Block '<S29>/Display' : Unused code path elimination
 * Block '<S29>/Display1' : Unused code path elimination
 * Block '<S29>/Display2' : Unused code path elimination
 * Block '<S29>/Display3' : Unused code path elimination
 * Block '<S18>/Product' : Unused code path elimination
 * Block '<S19>/Display' : Unused code path elimination
 * Block '<S39>/Constant' : Unused code path elimination
 * Block '<S51>/Constant' : Unused code path elimination
 * Block '<S51>/Constant1' : Unused code path elimination
 * Block '<S51>/Constant2' : Unused code path elimination
 * Block '<S51>/Constant3' : Unused code path elimination
 * Block '<S51>/Constant4' : Unused code path elimination
 * Block '<S51>/Constant5' : Unused code path elimination
 * Block '<S51>/Divide' : Unused code path elimination
 * Block '<S51>/Max' : Unused code path elimination
 * Block '<S51>/Max1' : Unused code path elimination
 * Block '<S51>/Min' : Unused code path elimination
 * Block '<S51>/Product' : Unused code path elimination
 * Block '<S51>/Subtract' : Unused code path elimination
 * Block '<S51>/Subtract1' : Unused code path elimination
 * Block '<S52>/1-D Lookup Table' : Unused code path elimination
 * Block '<S52>/Add' : Unused code path elimination
 * Block '<S52>/Constant' : Unused code path elimination
 * Block '<S52>/Constant1' : Unused code path elimination
 * Block '<S52>/Constant2' : Unused code path elimination
 * Block '<S52>/Gain' : Unused code path elimination
 * Block '<S52>/Product' : Unused code path elimination
 * Block '<S52>/Product1' : Unused code path elimination
 * Block '<S52>/Product2' : Unused code path elimination
 * Block '<S52>/Subtract' : Unused code path elimination
 * Block '<S57>/Display' : Unused code path elimination
 * Block '<S58>/Display' : Unused code path elimination
 * Block '<S58>/Display1' : Unused code path elimination
 * Block '<S58>/Display2' : Unused code path elimination
 * Block '<S1>/Constant1' : Unused code path elimination
 * Block '<S63>/Scope' : Unused code path elimination
 * Block '<S67>/Display' : Unused code path elimination
 * Block '<S77>/Constant' : Unused code path elimination
 * Block '<S77>/Display' : Unused code path elimination
 * Block '<S77>/Gain' : Unused code path elimination
 * Block '<S91>/Constant3' : Unused code path elimination
 * Block '<S91>/Constant6' : Unused code path elimination
 * Block '<S91>/Product2' : Unused code path elimination
 * Block '<S91>/Switch1' : Unused code path elimination
 * Block '<S109>/Display' : Unused code path elimination
 * Block '<S87>/Display' : Unused code path elimination
 * Block '<S79>/Display1' : Unused code path elimination
 * Block '<S79>/Display7' : Unused code path elimination
 * Block '<S128>/Display1' : Unused code path elimination
 * Block '<S128>/Display2' : Unused code path elimination
 * Block '<S79>/Scope2' : Unused code path elimination
 * Block '<S80>/Data Type Conversion' : Unused code path elimination
 * Block '<S80>/Data Type Conversion11' : Unused code path elimination
 * Block '<S80>/Data Type Conversion4' : Unused code path elimination
 * Block '<S80>/Display' : Unused code path elimination
 * Block '<S80>/Display1' : Unused code path elimination
 * Block '<S80>/Display2' : Unused code path elimination
 * Block '<S132>/Display1' : Unused code path elimination
 * Block '<S134>/Display' : Unused code path elimination
 * Block '<S134>/Display1' : Unused code path elimination
 * Block '<S81>/Display' : Unused code path elimination
 * Block '<S183>/Constant1' : Unused code path elimination
 * Block '<S197>/Add4' : Unused code path elimination
 * Block '<S197>/Add5' : Unused code path elimination
 * Block '<S197>/Constant3' : Unused code path elimination
 * Block '<S197>/Constant7' : Unused code path elimination
 * Block '<S197>/Data Type Conversion' : Unused code path elimination
 * Block '<S197>/Data Type Conversion1' : Unused code path elimination
 * Block '<S197>/Data Type Conversion3' : Unused code path elimination
 * Block '<S203>/Constant1' : Unused code path elimination
 * Block '<S203>/Constant3' : Unused code path elimination
 * Block '<S203>/Logical Operator' : Unused code path elimination
 * Block '<S203>/Logical Operator1' : Unused code path elimination
 * Block '<S203>/Switch1' : Unused code path elimination
 * Block '<S203>/Switch3' : Unused code path elimination
 * Block '<S197>/Lat_oc_calib_cnt_grad' : Unused code path elimination
 * Block '<S197>/Logical Operator' : Unused code path elimination
 * Block '<S197>/MinMax' : Unused code path elimination
 * Block '<S197>/Sac_ts' : Unused code path elimination
 * Block '<S197>/Switch5' : Unused code path elimination
 * Block '<S197>/Unit Delay5' : Unused code path elimination
 * Block '<S198>/Display1' : Unused code path elimination
 * Block '<S198>/Display2' : Unused code path elimination
 * Block '<S198>/Display3' : Unused code path elimination
 * Block '<S198>/Lat_oc_max_driver_torque' : Unused code path elimination
 * Block '<S199>/Display1' : Unused code path elimination
 * Block '<S199>/Display2' : Unused code path elimination
 * Block '<S199>/Display3' : Unused code path elimination
 * Block '<S199>/Display4' : Unused code path elimination
 * Block '<S199>/Display5' : Unused code path elimination
 * Block '<S208>/Data Type Conversion' : Unused code path elimination
 * Block '<S206>/Lat_oc_ki' : Unused code path elimination
 * Block '<S207>/Data Type Conversion' : Unused code path elimination
 * Block '<S207>/Display1' : Unused code path elimination
 * Block '<S207>/Display2' : Unused code path elimination
 * Block '<S207>/Display3' : Unused code path elimination
 * Block '<S207>/Display4' : Unused code path elimination
 * Block '<S207>/Display5' : Unused code path elimination
 * Block '<S207>/Display6' : Unused code path elimination
 * Block '<S209>/Add' : Unused code path elimination
 * Block '<S209>/Constant' : Unused code path elimination
 * Block '<S209>/Constant1' : Unused code path elimination
 * Block '<S209>/Gain1' : Unused code path elimination
 * Block '<S209>/Gain2' : Unused code path elimination
 * Block '<S209>/Gain3' : Unused code path elimination
 * Block '<S209>/Gain4' : Unused code path elimination
 * Block '<S209>/Switch1' : Unused code path elimination
 * Block '<S209>/Switch2' : Unused code path elimination
 * Block '<S209>/Switch3' : Unused code path elimination
 * Block '<S209>/Switch4' : Unused code path elimination
 * Block '<S211>/Display' : Unused code path elimination
 * Block '<S211>/Lat_oc_max_kappa_dys' : Unused code path elimination
 * Block '<S211>/tst1' : Unused code path elimination
 * Block '<S211>/tst2' : Unused code path elimination
 * Block '<S212>/Display' : Unused code path elimination
 * Block '<S200>/Bitwise Operator' : Unused code path elimination
 * Block '<S200>/Display' : Unused code path elimination
 * Block '<S200>/Display1' : Unused code path elimination
 * Block '<S200>/Display2' : Unused code path elimination
 * Block '<S200>/Display3' : Unused code path elimination
 * Block '<S200>/Logical Operator' : Unused code path elimination
 * Block '<S200>/Relational Operator' : Unused code path elimination
 * Block '<S200>/Scope' : Unused code path elimination
 * Block '<S201>/Lat_delta_off_flt_initial_omega' : Unused code path elimination
 * Block '<S201>/Lat_delta_offset_filter_omega' : Unused code path elimination
 * Block '<S202>/Display' : Unused code path elimination
 * Block '<S202>/Display1' : Unused code path elimination
 * Block '<S202>/Display2' : Unused code path elimination
 * Block '<S220>/Display' : Unused code path elimination
 * Block '<S221>/Display' : Unused code path elimination
 * Block '<S231>/Lat_oc_kappa_cmd_filter_coeff' : Unused code path elimination
 * Block '<S222>/Display' : Unused code path elimination
 * Block '<S223>/Display' : Unused code path elimination
 * Block '<S226>/Display' : Unused code path elimination
 * Block '<S227>/Display' : Unused code path elimination
 * Block '<S228>/Display' : Unused code path elimination
 * Block '<S229>/Display' : Unused code path elimination
 * Block '<S183>/Scope' : Unused code path elimination
 * Block '<S196>/Sub2' : Unused code path elimination
 * Block '<S187>/AND4' : Unused code path elimination
 * Block '<S187>/AND5' : Unused code path elimination
 * Block '<S187>/Constant15' : Unused code path elimination
 * Block '<S187>/Display' : Unused code path elimination
 * Block '<S187>/Display1' : Unused code path elimination
 * Block '<S187>/Display2' : Unused code path elimination
 * Block '<S187>/Display3' : Unused code path elimination
 * Block '<S187>/Display4' : Unused code path elimination
 * Block '<S187>/Display5' : Unused code path elimination
 * Block '<S187>/Display6' : Unused code path elimination
 * Block '<S187>/Equal8' : Unused code path elimination
 * Block '<S187>/NOT4' : Unused code path elimination
 * Block '<S188>/Lat_status_first_run_timer' : Unused code path elimination
 * Block '<S6>/Scope' : Unused code path elimination
 * Block '<S251>/Scope' : Unused code path elimination
 * Block '<S254>/Scope' : Unused code path elimination
 * Block '<S254>/Scope1' : Unused code path elimination
 * Block '<S7>/Scope' : Unused code path elimination
 * Block '<S8>/Gain' : Unused code path elimination
 * Block '<S258>/Constant' : Unused code path elimination
 * Block '<S258>/Constant1' : Unused code path elimination
 * Block '<S258>/Max' : Unused code path elimination
 * Block '<S258>/Max1' : Unused code path elimination
 * Block '<S258>/Switch' : Unused code path elimination
 * Block '<S259>/Add' : Unused code path elimination
 * Block '<S259>/Constant' : Unused code path elimination
 * Block '<S259>/Constant1' : Unused code path elimination
 * Block '<S259>/Min' : Unused code path elimination
 * Block '<S259>/Product' : Unused code path elimination
 * Block '<S259>/Product1' : Unused code path elimination
 * Block '<S259>/Subtract' : Unused code path elimination
 * Block '<S260>/Constant' : Unused code path elimination
 * Block '<S260>/Constant1' : Unused code path elimination
 * Block '<S260>/Max' : Unused code path elimination
 * Block '<S260>/Min' : Unused code path elimination
 * Block '<S261>/1-D Lookup Table' : Unused code path elimination
 * Block '<S261>/Constant' : Unused code path elimination
 * Block '<S261>/Gain' : Unused code path elimination
 * Block '<S261>/Product' : Unused code path elimination
 * Block '<S262>/Constant' : Unused code path elimination
 * Block '<S265>/AND' : Unused code path elimination
 * Block '<S265>/Constant' : Unused code path elimination
 * Block '<S265>/Constant1' : Unused code path elimination
 * Block '<S265>/GreaterThanOrEqual' : Unused code path elimination
 * Block '<S265>/NOT' : Unused code path elimination
 * Block '<S265>/Switch' : Unused code path elimination
 * Block '<S262>/Min' : Unused code path elimination
 * Block '<S263>/Constant' : Unused code path elimination
 * Block '<S269>/Constant' : Unused code path elimination
 * Block '<S269>/Switch' : Unused code path elimination
 * Block '<S270>/Abs' : Unused code path elimination
 * Block '<S270>/Constant' : Unused code path elimination
 * Block '<S270>/Constant1' : Unused code path elimination
 * Block '<S270>/Constant2' : Unused code path elimination
 * Block '<S270>/Constant3' : Unused code path elimination
 * Block '<S270>/Max' : Unused code path elimination
 * Block '<S270>/Min' : Unused code path elimination
 * Block '<S270>/Product' : Unused code path elimination
 * Block '<S270>/Subtract' : Unused code path elimination
 * Block '<S271>/Abs' : Unused code path elimination
 * Block '<S271>/Constant' : Unused code path elimination
 * Block '<S271>/Constant1' : Unused code path elimination
 * Block '<S271>/Constant2' : Unused code path elimination
 * Block '<S271>/Constant3' : Unused code path elimination
 * Block '<S271>/Max' : Unused code path elimination
 * Block '<S271>/Min' : Unused code path elimination
 * Block '<S271>/Product' : Unused code path elimination
 * Block '<S271>/Subtract' : Unused code path elimination
 * Block '<S272>/Constant' : Unused code path elimination
 * Block '<S273>/Add' : Unused code path elimination
 * Block '<S273>/Add1' : Unused code path elimination
 * Block '<S273>/Add2' : Unused code path elimination
 * Block '<S273>/Constant1' : Unused code path elimination
 * Block '<S273>/Constant2' : Unused code path elimination
 * Block '<S273>/Constant6' : Unused code path elimination
 * Block '<S273>/Divide' : Unused code path elimination
 * Block '<S273>/Divide1' : Unused code path elimination
 * Block '<S273>/GreaterThan' : Unused code path elimination
 * Block '<S273>/Product' : Unused code path elimination
 * Block '<S273>/Subtract' : Unused code path elimination
 * Block '<S273>/Switch' : Unused code path elimination
 * Block '<S273>/Unit Delay' : Unused code path elimination
 * Block '<S272>/Product' : Unused code path elimination
 * Block '<S272>/Product1' : Unused code path elimination
 * Block '<S263>/Min' : Unused code path elimination
 * Block '<S264>/Constant' : Unused code path elimination
 * Block '<S274>/Constant' : Unused code path elimination
 * Block '<S274>/Switch' : Unused code path elimination
 * Block '<S264>/Min' : Unused code path elimination
 * Block '<S8>/Product' : Unused code path elimination
 * Block '<S281>/Abs' : Unused code path elimination
 * Block '<S281>/Constant' : Unused code path elimination
 * Block '<S281>/Constant1' : Unused code path elimination
 * Block '<S281>/Max' : Unused code path elimination
 * Block '<S281>/Product' : Unused code path elimination
 * Block '<S281>/Sign' : Unused code path elimination
 * Block '<S281>/Subtract' : Unused code path elimination
 * Block '<S282>/Add' : Unused code path elimination
 * Block '<S282>/Gain' : Unused code path elimination
 * Block '<S283>/Constant' : Unused code path elimination
 * Block '<S283>/Gain' : Unused code path elimination
 * Block '<S283>/Max' : Unused code path elimination
 * Block '<S283>/Min' : Unused code path elimination
 * Block '<S10>/Subtract2' : Unused code path elimination
 * Block '<S10>/Subtract3' : Unused code path elimination
 * Block '<S288>/Constant1' : Unused code path elimination
 * Block '<S289>/Add1' : Unused code path elimination
 * Block '<S289>/Constant' : Unused code path elimination
 * Block '<S289>/Max' : Unused code path elimination
 * Block '<S289>/Min1' : Unused code path elimination
 * Block '<S289>/Product5' : Unused code path elimination
 * Block '<S289>/Subtract1' : Unused code path elimination
 * Block '<S285>/Scope' : Unused code path elimination
 * Block '<S11>/Add' : Unused code path elimination
 * Block '<S11>/Bitwise AND' : Unused code path elimination
 * Block '<S11>/Bitwise AND1' : Unused code path elimination
 * Block '<S11>/Bitwise AND2' : Unused code path elimination
 * Block '<S11>/Constant' : Unused code path elimination
 * Block '<S11>/Constant1' : Unused code path elimination
 * Block '<S11>/Constant2' : Unused code path elimination
 * Block '<S11>/Constant3' : Unused code path elimination
 * Block '<S11>/Display' : Unused code path elimination
 * Block '<S11>/Display1' : Unused code path elimination
 * Block '<S11>/Display2' : Unused code path elimination
 * Block '<S11>/Display3' : Unused code path elimination
 * Block '<S11>/Display4' : Unused code path elimination
 * Block '<S11>/Display5' : Unused code path elimination
 * Block '<S11>/Switch' : Unused code path elimination
 * Block '<S11>/Switch1' : Unused code path elimination
 * Block '<S11>/Switch3' : Unused code path elimination
 * Block '<S11>/Switch4' : Unused code path elimination
 * Block '<S1>/Scope4' : Unused code path elimination
 * Block '<S17>/Gain3' : Eliminated nontunable gain of 1
 * Block '<S132>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S133>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S134>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S200>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S215>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S201>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S202>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S202>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S219>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S194>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S254>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S289>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S183>/Constant2' : Unused code path elimination
 * Block '<S214>/Constant3' : Unused code path elimination
 * Block '<S185>/Constant' : Unused code path elimination
 * Block '<S241>/Constant' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('LaDMC_model/LaDMC')    - opens subsystem LaDMC_model/LaDMC
 * hilite_system('LaDMC_model/LaDMC/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LaDMC_model'
 * '<S1>'   : 'LaDMC_model/LaDMC'
 * '<S3>'   : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function'
 * '<S4>'   : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering'
 * '<S5>'   : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation'
 * '<S6>'   : 'LaDMC_model/LaDMC/LaDMCInputProcess'
 * '<S7>'   : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration'
 * '<S8>'   : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation'
 * '<S9>'   : 'LaDMC_model/LaDMC/SAC_Angle_Command_Derating'
 * '<S10>'  : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter'
 * '<S11>'  : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd'
 * '<S12>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function'
 * '<S13>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/FilterLAT_Pt1_Filter_Default_1_Dual_Slop'
 * '<S14>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/LAT_Pt1_Filter_Default_1_Singale_Slop'
 * '<S15>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/SAC_Variable_Stiffness_Definition'
 * '<S16>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Composite_Derating_Factor'
 * '<S17>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification'
 * '<S18>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Dos_Limit_Scale'
 * '<S19>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Idle_Control_Derating'
 * '<S20>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Integrator_Reset_Condition'
 * '<S21>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating'
 * '<S22>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating'
 * '<S23>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Load_Compensation_Derating'
 * '<S24>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating'
 * '<S25>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Unbalance_Compensation'
 * '<S26>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/LAT_Torque_Saturation_Scaling'
 * '<S27>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/LDP_Left_Lane_Mode_Selection'
 * '<S28>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Ldp_Threshold_Fading'
 * '<S29>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_State_Derating_Factor_Selection'
 * '<S30>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Torque_Request_Factor'
 * '<S31>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Vehicle_Steer_Torque_Compensation_Factor'
 * '<S32>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Vehicle_Steer_Torque_Factor'
 * '<S33>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Torque_Request_Factor/LAT_Pt1_Filter'
 * '<S34>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Diver_Steer_Classification/TDF_Vehicle_Steer_Torque_Compensation_Factor/saturation'
 * '<S35>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Idle_Control_Derating/TDF_Idle_Mono_Flop'
 * '<S36>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Initial_Driver_Torque'
 * '<S37>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Ldp_Threshold_Fading'
 * '<S38>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Max_Steering_Torque'
 * '<S39>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Steer_Torque_Sample'
 * '<S40>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Torque_Derating_Factor'
 * '<S41>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Torque_Derating_Factor_Higher_Freqency_Path'
 * '<S42>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Torque_Derating_Slop'
 * '<S43>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Torque_Derating_Threshold'
 * '<S44>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Torque_Derating/TDF_Torque_Derating_Threshold/TDF_Torque_Derating_Threshold_Scheduling'
 * '<S45>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating/Filter'
 * '<S46>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating/TDF_Control_Error_Factor'
 * '<S47>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating/TDF_Saturated_Angle_Command_Error'
 * '<S48>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating/TDF_Velocity_Factor'
 * '<S49>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Linear_Velocity_Derating/TDF_Control_Error_Factor/Filter'
 * '<S50>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Load_Compensation_Derating/LAT_Pt1_Filter_Dual_Slop'
 * '<S51>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Load_Compensation_Derating/SAC_Variable_Stiffness_Definition'
 * '<S52>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Load_Compensation_Derating/TDF_Compensation_Factor_Fading'
 * '<S53>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/Compare To Constant2'
 * '<S54>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/LAT_Rate_Limited_Switch'
 * '<S55>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/TDF_Driver_Torque_Detection'
 * '<S56>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/TDF_Saturation'
 * '<S57>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/TDF_Driver_Torque_Detection/TDF_State_Der_Active'
 * '<S58>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_State_Derating/TDF_Driver_Torque_Detection/TDF_State_Derating'
 * '<S59>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Unbalance_Compensation/Filter'
 * '<S60>'  : 'LaDMC_model/LaDMC/ TDF_Torque_Derating_Function/DMC_Lat_TDF_Torque_Derating_Function/TDF_Unbalance_Compensation/TDF_Steering_Angle_Saturation'
 * '<S61>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation'
 * '<S62>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/LAT_Sat_Threshold_Control'
 * '<S63>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Angle_Saturation'
 * '<S64>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Calc_Max_Angle'
 * '<S65>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Calc_Req_Angle'
 * '<S66>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Max_Delta_F_Cmd'
 * '<S67>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Saturated_Angle_Cmd'
 * '<S68>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Threshold_Definition'
 * '<S69>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/LAT_Sat_Threshold_Control/DMC_Integrator'
 * '<S70>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/LAT_Sat_Threshold_Control/LDC_Sac_Parity_Filter'
 * '<S71>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Calc_Max_Angle/Radians to Degrees'
 * '<S72>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Calc_Req_Angle/Radians to Degrees'
 * '<S73>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Saturated_Angle_Cmd/Compare To Constant'
 * '<S74>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Threshold_Definition/SAT_Max_Requested_Dynamic_Steering_Angle'
 * '<S75>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Threshold_Definition/SAT_Max_Requested_Steering_Angle'
 * '<S76>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Filtering/SAT_Angle_Command_Saturation/SAT_Threshold_Definition/SAT_Max_Steering_Angle_Command_Selection'
 * '<S77>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/ARB_Angle_Cmd_Arbitration'
 * '<S78>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter'
 * '<S79>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Osillation_Damping'
 * '<S80>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Control'
 * '<S81>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback'
 * '<S82>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor'
 * '<S83>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/ARB_Angle_Cmd_Arbitration/Compare To Constant'
 * '<S84>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/ARB_Angle_Cmd_Arbitration/Degrees to Radians'
 * '<S85>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter'
 * '<S86>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter'
 * '<S87>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function'
 * '<S88>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/Compare To Constant'
 * '<S89>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/Compare To Constant1'
 * '<S90>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/Compare To Constant2'
 * '<S91>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/DYC_Boost_Filter'
 * '<S92>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/DYC_Kappa_A2_Factor_Scheduling'
 * '<S93>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/DYC_Boost_Filter/DYC_Kappa_Dot_Filter'
 * '<S94>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/DYC_Kappa_A2_Factor_Scheduling/Compare To Constant1'
 * '<S95>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_Dynamic_Comp_Filter/DYC_Kappa_A2_Factor_Scheduling/Compare To Constant2'
 * '<S96>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/Compare To Constant'
 * '<S97>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_Dynamic_Compensation_Enabling'
 * '<S98>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter'
 * '<S99>'  : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter1'
 * '<S100>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/TRJ_Timebased_Kappa_Cmd'
 * '<S101>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_Dynamic_Compensation_Enabling/Compare To Constant'
 * '<S102>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_Dynamic_Compensation_Enabling/Compare To Constant1'
 * '<S103>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter/DMC_Integrator'
 * '<S104>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter/DMC_Integrator1'
 * '<S105>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter1/DMC_Integrator'
 * '<S106>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter1/DMC_Integrator1'
 * '<S107>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/DYC_state_Filter/DYC_State_FIlter1/DMC_Integrator2'
 * '<S108>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara'
 * '<S109>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Compensation_Angle_Selection'
 * '<S110>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Correction_Factor_Selection'
 * '<S111>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Longitudinal_Force_Comp'
 * '<S112>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/HEC_Yaw_Rate_Filter'
 * '<S113>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/Compare To Constant'
 * '<S114>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Angle_Per_Kappa_Gen_Selection'
 * '<S115>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Feedforward_Control_Selection'
 * '<S116>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Saturation'
 * '<S117>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/SAC_Kp_Fading'
 * '<S118>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Angle_Per_Kappa_Gen_Selection/Compare To Constant'
 * '<S119>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Angle_Per_Kappa_Gen_Selection/Compare To Constant1'
 * '<S120>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Feedforward_Control_Selection/Compare To Constant'
 * '<S121>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Angle_Per_Kappa_Chara/DYC_Feedforward_Control_Selection/Radians to Degrees'
 * '<S122>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Compensation_Angle_Selection/Compare To Constant2'
 * '<S123>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Correction_Factor_Selection/Compare To Constant2'
 * '<S124>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Longitudinal_Force_Comp/DYC_Angle_Saturation'
 * '<S125>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Longitudinal_Force_Comp/DYC_Lat_Accel_Saturation'
 * '<S126>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/DYC_Dynamic_Compensation_Filter/SAC_Kappa_Angle_Transfer_Function/DYC_Longitudinal_Force_Comp/LDC_Phase_Correction'
 * '<S127>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Osillation_Damping/SAC_Delta_Psi_Diff_Factor'
 * '<S128>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Osillation_Damping/SAC_Delta_Psi_Dot_Estimation'
 * '<S129>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Osillation_Damping/SAC_Pt1_Filter1'
 * '<S130>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Osillation_Damping/SAC_Delta_Psi_Dot_Estimation/HEC_Pt1_Filter'
 * '<S131>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Control/Compare To Constant2'
 * '<S132>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Control/LAT_Eps_Torque_Filter'
 * '<S133>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Control/SAC_Integrator_Saturation'
 * '<S134>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Control/SAC_Kappa_Command_Arbitration'
 * '<S135>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/Compare To Constant'
 * '<S136>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/HEC_Yaw_Rate_Dot_Estimation'
 * '<S137>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/HEC_Yaw_Rate_Filter'
 * '<S138>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/Hec_r_dot_factor'
 * '<S139>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/Hec_r_factor'
 * '<S140>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/HEC_Yaw_Rate_Dot_Estimation/Subsystem'
 * '<S141>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/Hec_r_dot_factor/Compare To Constant'
 * '<S142>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/SAC_Yaw_Rate_Feedback/Hec_r_factor/Compare To Constant'
 * '<S143>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Algorithm_Parameter'
 * '<S144>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation'
 * '<S145>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection'
 * '<S146>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Lateral_Error_Validity_Check'
 * '<S147>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/PT1'
 * '<S148>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Possible_Adaptation_Check'
 * '<S149>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Read_EEPROM_Correction_Factor'
 * '<S150>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Save_Correction_Factor_In_Vector'
 * '<S151>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Speed_Segment_Identify'
 * '<S152>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Algorithm_Parameter/Increase_Reduction_Factor_Computation'
 * '<S153>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Algorithm_Parameter/Maximum_Waiting_Counter_Computation'
 * '<S154>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Compare To Constant'
 * '<S155>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Compare To Constant1'
 * '<S156>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Compare To Constant2'
 * '<S157>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Compare To Constant3'
 * '<S158>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Compare To Constant4'
 * '<S159>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Correction_Factor_Saturation'
 * '<S160>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Default'
 * '<S161>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Left_Turn_Left_Error'
 * '<S162>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Left_Turn_Right_Error'
 * '<S163>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Right_Turn_Left_Error'
 * '<S164>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Correction_Factor_Estimation/Right_Turn_Right_Error'
 * '<S165>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Mean_Calculator'
 * '<S166>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction'
 * '<S167>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Mean_Calculator/Mean_Of_All_Segments'
 * '<S168>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Mean_Calculator/Mean_Of_Segment'
 * '<S169>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant'
 * '<S170>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant1'
 * '<S171>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant2'
 * '<S172>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant3'
 * '<S173>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant4'
 * '<S174>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant5'
 * '<S175>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant6'
 * '<S176>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant7'
 * '<S177>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Error_Side_And_Desired_Driving_Curve_Detection/Turn_And_Lateral_Error_Sisw_Direction/Compare To Constant8'
 * '<S178>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Read_EEPROM_Correction_Factor/Compare To Constant'
 * '<S179>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Read_EEPROM_Correction_Factor/Initial'
 * '<S180>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Save_Correction_Factor_In_Vector/Compare To Constant'
 * '<S181>' : 'LaDMC_model/LaDMC/LAT_Angle_Command_Generation/Steering_Characteristic_Corrction_Factor/Save_Correction_Factor_In_Vector/Correction_Factor_Gradient_Limitation'
 * '<S182>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Dot_Filter'
 * '<S183>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch'
 * '<S184>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Dot_Filter'
 * '<S185>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter'
 * '<S186>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kinematic_Signals_Arbiter'
 * '<S187>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Ldp_States'
 * '<S188>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Status_First_Run'
 * '<S189>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Stiffness_Request_Limitation'
 * '<S190>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Yaw_Rate_Filter'
 * '<S191>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration'
 * '<S192>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Vdy_Offset'
 * '<S193>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Enable_Lateral_Control'
 * '<S194>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Kappa_Command_Selection'
 * '<S195>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/Radians to Degrees'
 * '<S196>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/VEH_Delta_F_Offset_Calibration_Selection'
 * '<S197>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Calibration_Counter'
 * '<S198>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_High_Driver_Torque_Class'
 * '<S199>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration'
 * '<S200>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Integrator'
 * '<S201>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Kappa'
 * '<S202>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition'
 * '<S203>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Calibration_Counter/LAT_OC_State_Preload'
 * '<S204>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Oc_Calibration_Hold_Flag'
 * '<S205>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Oc_Calibration_Hold_Flag_Short'
 * '<S206>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Oc_Learning_Rate'
 * '<S207>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition'
 * '<S208>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Oc_Learning_Rate/LAT_Oc_Integrator_Input_Raw'
 * '<S209>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/Encoder'
 * '<S210>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_OC_Implaus_Lateral_Error'
 * '<S211>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Maximum_Filtered_Curvature_Command_Flag'
 * '<S212>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Minimum_Vehicle_Velocity_Flag'
 * '<S213>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Maximum_Filtered_Curvature_Command_Flag/LAT_Pt1_Filter'
 * '<S214>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Integrator/LAT_OC_State_Preload1'
 * '<S215>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Integrator/LAT_Oc_Integ_Inp_Saturation'
 * '<S216>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Integrator/LAT_Oc_Integrator_Saturation'
 * '<S217>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/Compare To Zero 1'
 * '<S218>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/Compare To Zero 2'
 * '<S219>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Encoder'
 * '<S220>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Kappa_Confirmed_Enable_Flag'
 * '<S221>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Curvature_Command_Flag'
 * '<S222>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Delta_F_Dot_Flag'
 * '<S223>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Driver_Torque_Flag'
 * '<S224>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Heading_Error_Flag'
 * '<S225>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Lateral_Acceleration_Flag'
 * '<S226>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Velocity_Threshold_Flag'
 * '<S227>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Min_Heading_Error_Qualifier_Flag'
 * '<S228>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Min_Kappa_Command_Qualifier_Flag'
 * '<S229>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Min_Velocity_Threshold_Flag'
 * '<S230>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Kappa_Confirmed_Enable_Flag/LAT_Oc_Kappa_Mono_Flop'
 * '<S231>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Curvature_Command_Flag/LAT_Pt1_Filter'
 * '<S232>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Delta_F_Offset_Switch/LAT_Delta_F_Offset_Calibration/LAT_Offset_Calibration_Trigger_Condition/LAT_Oc_Max_Lateral_Acceleration_Flag/LAT_Pt1_Filter'
 * '<S233>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter'
 * '<S234>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Pre_Filter'
 * '<S235>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter'
 * '<S236>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Error_Class'
 * '<S237>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Height_Class'
 * '<S238>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Lateral_Class'
 * '<S239>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Lateral_Class1'
 * '<S240>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Strght_Fwd_Class'
 * '<S241>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Linz_Filt_Omega_Fading'
 * '<S242>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Linz_Pt1_Filter'
 * '<S243>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/Subsystem'
 * '<S244>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Lateral_Class/LAT_Pt1_Filter1'
 * '<S245>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Linz_Filter/LAT_Kappa_Linz_Filt_Lateral_Class1/LAT_Pt1_Filter1'
 * '<S246>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter/LAT_Kappa_Filter'
 * '<S247>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter/LAT_Kappa_Gradient'
 * '<S248>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter/LAT_Kappa_Gradient/LAT_Kappa_Gradient_Fader'
 * '<S249>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter/LAT_Kappa_Gradient/LAT_Kappa_Gradient_Fading_Signal'
 * '<S250>' : 'LaDMC_model/LaDMC/LaDMCInputProcess/LAT_Kappa_Filter/LAT_Kappa_Rate_Limiter/LAT_Kappa_Gradient/LAT_Max_Kappa_Gradient'
 * '<S251>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Cmd_Arbitration'
 * '<S252>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Cmd_Select'
 * '<S253>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Rate_Gradient_Arbitration'
 * '<S254>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Rate_Limitation'
 * '<S255>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Cmd_Select/Else'
 * '<S256>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Cmd_Select/If'
 * '<S257>' : 'LaDMC_model/LaDMC/Lateral_Cmd_Source_Arbitration/Delta_F_Cmd_Select/If1'
 * '<S258>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/LCO_Comp_Factor_Arbitration'
 * '<S259>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/LCO_Compensation_Factor_Fader'
 * '<S260>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/LCO_Dos_Limit_Scale_Saturation'
 * '<S261>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/LCO_Load_Compensation_Factor'
 * '<S262>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic'
 * '<S263>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1'
 * '<S264>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic2'
 * '<S265>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic/LCO_Fading_Signal_Selection'
 * '<S266>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic/LCO_Min_Curvature_Command_Filtering'
 * '<S267>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic/LC_Min_Curvature_Command'
 * '<S268>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic/LCO_Min_Curvature_Command_Filtering/LAT_Pt1_Filter_Dual_Slop'
 * '<S269>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1/LCO_Fading_Signal_Selection'
 * '<S270>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1/LCO_Min_Control_Error'
 * '<S271>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1/LCO_Min_Lateral_Error'
 * '<S272>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1/LCO_Minimum_Error_Enhancement'
 * '<S273>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic1/LCO_Minimum_Error_Enhancement/LAT_Pt1_Filter_Dual_Slop'
 * '<S274>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic2/LCO_Fading_Signal_Selection'
 * '<S275>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic2/LCO_Min_Kappa_Dot'
 * '<S276>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic2/LCO_Minimum_Error_Enhancement'
 * '<S277>' : 'LaDMC_model/LaDMC/SAC_Accuracy_Calculation/Lco_comp_act_characteristic2/LCO_Minimum_Error_Enhancement/LAT_Pt1_Filter_Dual_Slop'
 * '<S278>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Cmd_Grad'
 * '<S279>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Filter1'
 * '<S280>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Grad_Arbitration'
 * '<S281>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Filter1/LAT_Cut_Delta_Sample'
 * '<S282>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Filter1/LAT_Delta_F_Sample'
 * '<S283>' : 'LaDMC_model/LaDMC/SAC_Angle_Command_Rate_Limiter/SAC_Delta_F_Filter1/LAT_Saturate_Delta_Sample'
 * '<S284>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration'
 * '<S285>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration'
 * '<S286>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Cmd_Arbitration'
 * '<S287>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Cmd_Select'
 * '<S288>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Rate_Gradient_Arbitration'
 * '<S289>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Rate_Limitation'
 * '<S290>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Cmd_Select/Else'
 * '<S291>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Cmd_Select/If'
 * '<S292>' : 'LaDMC_model/LaDMC/SAC_Steering_Wheel_Angle_Cmd/ADAS_NOP_Cmd_Source_Arbitration/Longitudinal_Cmd_Source_Arbitration/Lon_Acc_Cmd_Select/If1'
 */
#endif                                 /* RTW_HEADER_LaDMC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
