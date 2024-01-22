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
 * File                             : LaDMC.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Jan  6 09:31:14 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "LaDMC.h"
#include "LaDMC_private.h"
#include "intrp1d_fu32fl_pw.h"
#include "look1_iflf_binlxpw.h"
#include "look1_iflf_binlxpw.h"
#include "plook_u32ff_binc.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile real32_T Acf_no_of_items_in_sample = 100.0F;/* Referenced by: '<S168>/Constant' */
const volatile real32_T Adp_dyc_corr_fact_dec_x_sched[12] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                 /* Referenced by: '<S152>/1-D Lookup Table1' */

const volatile real32_T Adp_dyc_corr_fact_dec_y_sched[12] = { 0.002F, 0.002F,
  0.002F, 0.002F, 0.003F, 0.005F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F
} ;                              /* Referenced by: '<S152>/1-D Lookup Table1' */

const volatile real32_T Adp_dyc_corr_fact_inc_x_sched[12] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S152>/1-D Lookup Table' */

const volatile real32_T Adp_dyc_corr_fact_inc_y_sched[12] = { 0.002F, 0.002F,
  0.002F, 0.002F, 0.003F, 0.005F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F
} ;                               /* Referenced by: '<S152>/1-D Lookup Table' */

const volatile real32_T Adp_dyc_corr_factor_lower_limit = 0.7F;/* Referenced by: '<S159>/Constant' */
const volatile real32_T Adp_dyc_corr_factor_upper_limit = 1.3F;/* Referenced by: '<S159>/Constant1' */
const volatile real32_T DYC_Ay_Linear_Fading_Constant = 0.5F;/* Referenced by: '<S108>/Constant2' */
const volatile real32_T DYC_Ay_Linear_Treshold = 4.0F;/* Referenced by: '<S108>/Constant1' */
const volatile real32_T DYC_Filter_Omega = 20.0F;/* Referenced by: '<S126>/Constant' */
const volatile real32_T DYC_Filter_Tau = 0.1F;/* Referenced by: '<S126>/Constant1' */
const volatile real32_T DYC_Kappa_Angle_Hi_Ay_Corr_Fact = 1.0F;/* Referenced by: '<S108>/Constant' */
const volatile real32_T DYC_Kappa_Angle_T_X_Sch_Hi_Ay[12] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S108>/1-D Lookup Table' */

const volatile real32_T DYC_Kappa_Angle_T_Y_Sch_Hi_Ay[12] = { 174.2F, 177.0F,
  185.3F, 199.3F, 218.8F, 243.8F, 274.5F, 352.5F, 452.7F, 575.3F, 1076.6F,
  1915.0F } ;                     /* Referenced by: '<S108>/1-D Lookup Table' */

const volatile real32_T DYC_Kappa_Coeff[4] = { 1.0F, 0.3F, 0.0F, 0.0F } ;/* Referenced by: '<S85>/Constant1' */

const volatile real32_T DYC_Long_Force_Comp_Cor_Factor = 1.0F;/* Referenced by: '<S111>/Constant' */
const volatile real32_T DYC_Max_Delta_F_Correction = 2.0F;/* Referenced by: '<S111>/Constant4' */
const volatile real32_T DYC_Max_Lateral_Acceleration = 6.0F;/* Referenced by: '<S111>/Constant3' */
const volatile real32_T Delta_F_Direct_Feedthrough_Par = 0.0F;/* Referenced by: '<S253>/Constant1' */
const volatile real32_T Delta_F_Gradient_Par = 40.0F;/* Referenced by: '<S253>/Constant' */
const volatile real32_T Dyc_boost_signal_reduction = 0.003F;/* Referenced by: '<S91>/Constant4' */
const volatile real32_T Dyc_kappa_A4_X_scheduling[11] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S85>/1-D Lookup Table1' */

const volatile real32_T Dyc_kappa_A4_Y_scheduling[11] = { 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                  /* Referenced by: '<S85>/1-D Lookup Table1' */

const volatile real32_T Dyc_kappa_A4_factor = 0.0F;/* Referenced by: '<S85>/Constant4' */
const volatile real32_T Dyc_kappa_a2_boost_factor = 0.0F;/* Referenced by: '<S91>/Constant' */
const volatile real32_T Dyc_kappa_a2_x_scheduling_Sc[12] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S92>/1-D Lookup Table' */

const volatile real32_T Dyc_kappa_a2_y_scheduling_Sc[12] = { 0.14315F, 0.14315F,
  0.14315F, 0.14315F, 0.14315F, 0.17178F, 0.20041F, 0.283155F, 0.394005F,
  0.483285F, 0.483285F, 0.483285F } ;
                                   /* Referenced by: '<S92>/1-D Lookup Table' */

const volatile real32_T Dyc_kappa_a3_factor = 0.0F;/* Referenced by: '<S85>/Constant3' */
const volatile real32_T Dyc_kappa_a3_x_scheduling[11] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 50.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S85>/1-D Lookup Table' */

const volatile real32_T Dyc_kappa_a3_y_scheduling[11] = { 0.018F, 0.018F, 0.018F,
  0.018F, 0.018F, 0.018F, 0.018F, 0.018F, 0.018F, 0.018F, 0.018F } ;
                                   /* Referenced by: '<S85>/1-D Lookup Table' */

const volatile real32_T Dyc_kappa_angle_ldp_corr_y_sch[12] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S114>/1-D Lookup Table' */

const volatile real32_T Dyc_kappa_angle_lpf_corr_factor = 0.0F;/* Referenced by: '<S87>/Constant' */
const volatile real32_T Dyc_kappa_angle_t_x_schedul_gen[12] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;/* Referenced by:
                                                                      * '<S114>/1-D Lookup Table'
                                                                      * '<S114>/1-D Lookup Table3'
                                                                      */

const volatile real32_T Dyc_kappa_angle_t_y_schedul_nom[12] = { 155.38F, 155.38F,
  155.38F, 155.38F, 155.38F, 155.38F, 161.024307F, 188.389496F, 215.754807F,
  243.12F, 325.215698F, 420.994F } ;
                                 /* Referenced by: '<S114>/1-D Lookup Table3' */

const volatile real32_T Dyc_kappa_dot_boost_thrs = 0.003F;/* Referenced by: '<S91>/Constant1' */
const volatile real32_T Dyc_kappa_dot_filter_coeff = 1.0F;/* Referenced by: '<S93>/Constant' */
const volatile real32_T Dyc_time_constant_x_scheduling[12] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S86>/1-D Lookup Table' */

const volatile real32_T Dyc_time_constant_y_scheduling[12] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                   /* Referenced by: '<S86>/1-D Lookup Table' */

const volatile real32_T HEC_Kd_X_Scheduling[11] = { 0.0F, 10.0F, 20.0F, 30.0F,
  40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                 /* Referenced by: '<S127>/HEC_Kd_Scheduling' */

const volatile real32_T HEC_Kd_Y_Scheduling[11] = { 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                 /* Referenced by: '<S127>/HEC_Kd_Scheduling' */

const volatile real32_T Hec_r_dot_factor = 0.01F;/* Referenced by: '<S138>/Constant' */
const volatile real32_T Hec_r_dot_factor_Sc = 0.0F;/* Referenced by: '<S138>/Constant1' */
const volatile real32_T Hec_r_dot_factor_x_scheduling[11] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S138>/1-D Lookup Table' */

const volatile real32_T Hec_r_dot_factor_x_scheduling_Sc[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                 /* Referenced by: '<S138>/1-D Lookup Table1' */

const volatile real32_T Hec_r_dot_factor_y_scheduling[11] = { 3.0F, 3.0F, 3.0F,
  3.0F, 3.0F, 3.0F, 3.0F, 3.0F, 3.0F, 3.0F, 3.0F } ;
                                  /* Referenced by: '<S138>/1-D Lookup Table' */

const volatile real32_T Hec_r_dot_factor_y_scheduling_Sc[11] = { 0.5F, 0.5F,
  0.5F, 0.2F, 0.08F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                 /* Referenced by: '<S138>/1-D Lookup Table1' */

const volatile real32_T Hec_r_factor = 0.001F;/* Referenced by: '<S139>/Constant' */
const volatile real32_T Hec_r_factor_Sc = 0.0F;/* Referenced by: '<S139>/Constant1' */
const volatile real32_T Hec_r_pt1_factor = 0.001F;/* Referenced by: '<S81>/Constant' */
const volatile real32_T Hec_r_x_scheduling[11] = { 0.0F, 10.0F, 20.0F, 30.0F,
  40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 220.0F } ;
                                  /* Referenced by: '<S139>/1-D Lookup Table' */

const volatile real32_T Hec_r_x_scheduling_Sc[11] = { 0.0F, 10.0F, 20.0F, 30.0F,
  40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 220.0F } ;
                                 /* Referenced by: '<S139>/1-D Lookup Table1' */

const volatile real32_T Hec_r_y_scheduling[11] = { 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 2.5F, 4.0F, 5.8F, 14.0F } ;
                                  /* Referenced by: '<S139>/1-D Lookup Table' */

const volatile real32_T Hec_r_y_scheduling_Sc[11] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                 /* Referenced by: '<S139>/1-D Lookup Table1' */

const volatile real32_T Hec_yaw_rate_filter_coeff = 0.04F;/* Referenced by: '<S137>/Constant' */
const volatile real32_T LAT_Kappa_Dot_Filter_Coeff = 1.0F;/* Referenced by: '<S184>/Constant1' */
const volatile real32_T LAT_Yaw_Rate_Filter_Coeff = 1.0F;/* Referenced by: '<S190>/Constant1' */
const volatile real32_T Lat_delta_f_dot_filter_coeff = 0.4F;/* Referenced by: '<S182>/Constant1' */
const volatile real32_T Lat_delta_f_offset = 0.001F;
                                /* Referenced by: '<S196>/LAT_Delta_F_Offset' */
const volatile real32_T Lat_delta_off_flt_initial_loops = 1000.0F;
                   /* Referenced by: '<S201>/Lat_delta_off_flt_initial_loops' */
const volatile real32_T Lat_direct_feedthrough_kappa = 1.0F;/* Referenced by: '<S235>/Constant1' */
const volatile real32_T Lat_kappa_discharge_gradient = 0.0F;/* Referenced by: '<S250>/Constant1' */
const volatile real32_T Lat_kappa_discharge_min_error = 0.002F;/* Referenced by: '<S249>/Constant4' */
const volatile real32_T Lat_kappa_discharge_slope = 50.0F;/* Referenced by: '<S249>/Constant6' */
const volatile real32_T Lat_kappa_filter_coeff = 0.131F;/* Referenced by: '<S234>/Constant1' */
const volatile real32_T Lat_kappa_gradient_ldp = 1.0F;/* Referenced by: '<S248>/Constant1' */
const volatile real32_T Lat_kappa_lateral_error_slope = 10.0F;/* Referenced by: '<S249>/Constant2' */
const volatile real32_T Lat_kappa_linz_default_memshp = 0.5F;/* Referenced by: '<S243>/Constant' */
const volatile real32_T Lat_kappa_linz_error_slope = 10000.0F;/* Referenced by: '<S236>/Constant4' */
const volatile real32_T Lat_kappa_linz_filt_err_hi_curv = 0.0F;/* Referenced by: '<S236>/Constant' */
const volatile real32_T Lat_kappa_linz_filt_max_error = 0.0002F;/* Referenced by: '<S236>/Constant2' */
const volatile real32_T Lat_kappa_linz_filter_max_omega = 20.0F;/* Referenced by: '<S241>/Constant1' */
const volatile real32_T Lat_kappa_linz_filter_min_omega = 0.5F;/* Referenced by: '<S241>/Constant3' */
const volatile real32_T Lat_kappa_linz_head_mx_memshp = 0.0F;/* Referenced by: '<S239>/Constant5' */
const volatile real32_T Lat_kappa_linz_height_factor = 500.0F;/* Referenced by: '<S237>/Constant' */
const volatile real32_T Lat_kappa_linz_height_mx_memshp = 0.0F;/* Referenced by: '<S237>/Constant2' */
const volatile real32_T Lat_kappa_linz_lat_err_coeff = 0.999F;/* Referenced by:
                                                               * '<S244>/Constant1'
                                                               * '<S245>/Constant1'
                                                               */
const volatile real32_T Lat_kappa_linz_lat_error_slope = 0.0F;/* Referenced by:
                                                               * '<S238>/Constant3'
                                                               * '<S239>/Constant3'
                                                               */
const volatile real32_T Lat_kappa_linz_max_head_error = 0.03F;/* Referenced by: '<S239>/Constant1' */
const volatile real32_T Lat_kappa_linz_max_lat_error = 0.3F;/* Referenced by: '<S238>/Constant1' */
const volatile real32_T Lat_kappa_linz_min_crv_progress = 0.0005F;/* Referenced by: '<S237>/Constant3' */
const volatile real32_T Lat_kappa_linz_staight_mx_memshp = 0.0F;/* Referenced by: '<S240>/Constant1' */
const volatile real32_T Lat_kappa_linz_straight_weight = 2000.0F;/* Referenced by: '<S240>/Constant' */
const volatile real32_T Lat_kappa_linz_wt_fact_progress = 0.0F;/* Referenced by: '<S237>/Constant4' */
const volatile real32_T Lat_kappa_max_lateral_error = 0.2F;/* Referenced by: '<S249>/Constant' */
const volatile real32_T Lat_kppa_min_omega_x_scheduling[12] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S241>/1-D Lookup Table' */

const volatile real32_T Lat_kppa_min_omega_y_scheduling[12] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S241>/1-D Lookup Table' */

const volatile real32_T Lat_ldp_startup_time = 0.5F;/* Referenced by: '<S187>/Constant9' */
const volatile real32_T Lat_max_kappa_grad_x_scheduling[12] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S250>/1-D Lookup Table' */

const volatile real32_T Lat_max_kappa_grad_y_scheduling[12] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S250>/1-D Lookup Table' */

const volatile real32_T Lat_max_kappa_gradient = 0.001F;/* Referenced by: '<S250>/Constant' */
const volatile real32_T Lat_oc_fast_ki = 0.01F;
                                    /* Referenced by: '<S206>/Lat_oc_fast_ki' */
const volatile real32_T Lat_oc_implaus_lateral_error = 0.6F;
                      /* Referenced by: '<S210>/Lat_oc_implaus_lateral_error' */
const volatile real32_T Lat_oc_kappa_cmd_filter_coeff = 0.1F;
                     /* Referenced by: '<S232>/Lat_oc_kappa_cmd_filter_coeff' */
const volatile real32_T Lat_oc_kappa_ffwd_filter_coeff = 0.1F;
                    /* Referenced by: '<S213>/Lat_oc_kappa_ffwd_filter_coeff' */
const volatile real32_T Lat_oc_kappa_max_driver_torque = 1.0F;
                    /* Referenced by: '<S223>/Lat_oc_kappa_max_driver_torque' */
const volatile real32_T Lat_oc_kappa_max_heading_error = 1.0F;/* Referenced by:
                                                               * '<S220>/Lat_oc_kappa_max_heading_error'
                                                               * '<S224>/Lat_oc_kappa_max_heading_error'
                                                               */
const volatile real32_T Lat_oc_kappa_min_head_err_qual = 1.0F;/* Referenced by:
                                                               * '<S220>/Lat_oc_kappa_min_head_err_qual'
                                                               * '<S227>/Lat_oc_kappa_min_head_err_qual'
                                                               */
const volatile real32_T Lat_oc_kappa_min_latency = 100.0F;
                          /* Referenced by: '<S220>/Lat_oc_kappa_min_latency' */
const volatile real32_T Lat_oc_max_delta_f_dot = 0.2F;
                            /* Referenced by: '<S222>/Lat_oc_max_delta_f_dot' */
const volatile real32_T Lat_oc_max_delta_offset = 0.72F;
                           /* Referenced by: '<S200>/Lat_oc_max_delta_offset' */
const volatile real32_T Lat_oc_max_delta_offset_kappa = 0.72F;
                     /* Referenced by: '<S200>/Lat_oc_max_delta_offset_kappa' */
const volatile real32_T Lat_oc_max_kappa = 0.002F;
                                  /* Referenced by: '<S221>/Lat_oc_max_kappa' */
const volatile real32_T Lat_oc_max_lateral_accel = 2.0F;
                          /* Referenced by: '<S225>/Lat_oc_max_lateral_accel' */
const volatile real32_T Lat_oc_max_lateral_error = 3.0F;
                          /* Referenced by: '<S206>/Lat_oc_max_lateral_error' */
const volatile real32_T Lat_oc_max_offset_rate = 0.1F;
                            /* Referenced by: '<S215>/Lat_oc_max_offset_rate' */
const volatile real32_T Lat_oc_max_velocity = 140.0F;
                               /* Referenced by: '<S226>/Lat_oc_max_velocity' */
const volatile real32_T Lat_oc_min_kappa_quality = 1.0F;
                          /* Referenced by: '<S228>/Lat_oc_min_kappa_quality' */
const volatile real32_T Lat_oc_min_velocity = 30.0F;
                               /* Referenced by: '<S229>/Lat_oc_min_velocity' */
const volatile real32_T Lat_oc_min_velocity_dys = 5.0F;
                           /* Referenced by: '<S212>/Lat_oc_min_velocity_dys' */
const volatile real32_T Lat_oc_minimum_latency = 250.0F;
                            /* Referenced by: '<S204>/Lat_oc_minimum_latency' */
const volatile real32_T Lat_oc_minimum_latency_shrt = 50.0F;
                       /* Referenced by: '<S205>/Lat_oc_minimum_latency_shrt' */
const volatile real32_T Lat_status_first_run_time = 5.0F;
                         /* Referenced by: '<S188>/Lat_status_first_run_time' */
const volatile real32_T Lat_yaw_rate_dot_filter_coeff = 0.62F;/* Referenced by: '<S140>/Constant' */
const volatile real32_T Lco_curv_dot_filter_fall_coeff = 0.015F;/* Referenced by: '<S277>/Constant2' */
const volatile real32_T Lco_curv_dot_filter_risng_coeff = 1.0F;/* Referenced by: '<S277>/Constant1' */
const volatile real32_T Lco_curv_dot_preload_enh_factor = 0.002F;/* Referenced by: '<S276>/Constant' */
const volatile real32_T Lco_filter_falling_coeff = 0.015F;/* Referenced by: '<S268>/Constant2' */
const volatile real32_T Lco_filter_rising_coeff = 1.0F;/* Referenced by: '<S268>/Constant1' */
const volatile real32_T Lco_min_curvature_command = 0.0002F;/* Referenced by: '<S267>/Constant' */
const volatile real32_T Lco_min_curvature_dot = 0.01F;/* Referenced by: '<S275>/Constant' */
const volatile real32_T Lco_min_curvature_dot_slop = 500.0F;/* Referenced by: '<S275>/Constant2' */
const volatile real32_T Lco_min_curvature_slope = 500.0F;/* Referenced by: '<S267>/Constant2' */
const volatile real32_T Lco_preload_enhancement_factor = 2.0F;/* Referenced by: '<S266>/Constant' */
const volatile real32_T Lon_Acc_Gradient_Par = 5.0F;/* Referenced by: '<S288>/Constant' */
const volatile real32_T Max_Wait_Counter_x[9] = { 10.0F, 30.0F, 50.0F, 70.0F,
  90.0F, 110.0F, 130.0F, 150.0F, 170.0F } ;
                                  /* Referenced by: '<S153>/1-D Lookup Table' */

const volatile real32_T Max_Wait_Counter_y[9] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F } ;      /* Referenced by: '<S153>/1-D Lookup Table' */

const volatile uint32_T Sac_controller_mode_2 = 0U;/* Referenced by: '<S6>/Constant6' */
const volatile real32_T Sac_delta_f_counter_steer_grad = 0.0F;/* Referenced by: '<S280>/Constant' */
const volatile real32_T Sac_delta_psi_dot_factor = 0.001F;
                          /* Referenced by: '<S127>/Sac_delta_psi_dot_factor' */
const volatile real32_T Sac_delta_psi_dot_maf_length = 0.001F;
                      /* Referenced by: '<S128>/Sac_delta_psi_dot_maf_length' */
const volatile real32_T Sac_delta_psi_pt1_filter_coeff = 0.1F;
                    /* Referenced by: '<S128>/Sac_delta_psi_pt1_filter_coeff' */
const volatile real32_T Sac_one_by_ts = 50.0F;/* Referenced by:
                                               * '<S79>/Sac_one_by_ts1'
                                               * '<S128>/Sac_one_by_ts'
                                               */
const volatile real32_T Sac_osd_delta_ys_dot_coeff = 0.1F;
                         /* Referenced by: '<S79>/Sac_osd_delta_ys_dot_coeff' */
const volatile real32_T Sac_osd_delta_ys_dot_feedback = 0.0F;
                      /* Referenced by: '<S79>/Sac_osd_delta_ys_dot_feedback' */
const volatile real32_T Sac_parity_time_const_barrier = 0.0F;/* Referenced by: '<S10>/Constant1' */
const volatile real32_T Sac_ts = 0.02F;/* Referenced by:
                                        * '<S79>/Constant1'
                                        * '<S80>/Sac_ts'
                                        * '<S188>/Sac_ts'
                                        * '<S254>/Sac_ts2'
                                        * '<S128>/Constant2'
                                        * '<S132>/Constant1'
                                        * '<S200>/Sac_ts'
                                        * '<S289>/Sac_ts2'
                                        */
const volatile real32_T Sac_yrc_ctrl_cmd_fading_x_sched[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                     /* Referenced by: '<S134>/SAC_Yrc_Ctrl_Cmd_Fading_Sched' */

const volatile real32_T Sac_yrc_ctrl_cmd_fading_y_sched[11] = { 1.0F, 1.0F, 0.5F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                     /* Referenced by: '<S134>/SAC_Yrc_Ctrl_Cmd_Fading_Sched' */

const volatile real32_T Sac_yrc_ki_gain = 0.001F;
                                    /* Referenced by: '<S80>/Sac_yrc_ki_gain' */
const volatile real32_T Sac_yrc_kp_gain = 0.001F;/* Referenced by: '<S80>/Constant1' */
const volatile real32_T Sac_yrc_kp_pt1_gain = 0.001F;/* Referenced by: '<S80>/Constant2' */
const volatile real32_T Sac_yrc_kp_pt1_x_scheduling[11] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                          /* Referenced by: '<S80>/SAC_Yrc_Kp_Pt1_Scheduling' */

const volatile real32_T Sac_yrc_kp_pt1_y_scheduling[11] = { 40.0F, 40.0F, 10.0F,
  10.0F, 10.0F, 5.0F, 3.0F, 3.0F, 1.0F, 1.0F, 1.0F } ;
                          /* Referenced by: '<S80>/SAC_Yrc_Kp_Pt1_Scheduling' */

const volatile real32_T Sac_yrc_kp_x_scheduling[11] = { 0.0F, 10.0F, 20.0F,
  30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                              /* Referenced by: '<S80>/SAC_Yrc_Kp_Scheduling' */

const volatile real32_T Sac_yrc_kp_y_scheduling[11] = { 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                              /* Referenced by: '<S80>/SAC_Yrc_Kp_Scheduling' */

const volatile real32_T Sac_yrc_loop_gain_corr = 1.0F;
                             /* Referenced by: '<S80>/Sac_yrc_loop_gain_corr' */
const volatile real32_T Sac_yrc_pt1_filter_coeff = 0.04F;
                          /* Referenced by: '<S132>/Sac_yrc_pt1_filter_coeff' */
const volatile real32_T Sat_dynamic_enhancement_factor = 1.0F;/* Referenced by: '<S74>/Constant' */
const volatile real32_T Sat_thrs_control_kp = 0.0F;/* Referenced by: '<S62>/Constant' */
const volatile real32_T SpeedSegmentLookup1[10] = { 0.0F, 20.0F, 40.0F, 60.0F,
  80.0F, 100.0F, 120.0F, 140.0F, 160.0F, 180.0F } ;/* Referenced by: '<S151>/Constant' */

const volatile real32_T SpeedSegmentLookup2[9] = { 10.0F, 30.0F, 50.0F, 70.0F,
  90.0F, 110.0F, 130.0F, 150.0F, 170.0F } ;/* Referenced by: '<S151>/Constant1' */

const volatile real32_T TDF_Derating_Switch_Min_Time = 0.3F;/* Referenced by: '<S57>/Constant' */
const volatile real32_T TDF_Max_Derating_Factor = 1.0F;/* Referenced by: '<S56>/Constant1' */
const volatile real32_T TDF_Switch_Falling_Rate = 1.0F;/* Referenced by: '<S54>/Constant1' */
const volatile real32_T TDF_Switch_Rising_Rate = 1.0F;/* Referenced by: '<S54>/Constant' */
const volatile real32_T Tdf_comp_factor_filter_coeff = 0.999F;/* Referenced by: '<S50>/Constant1' */
const volatile real32_T Tdf_comp_filter_min_residual = 0.1F;/* Referenced by: '<S50>/Constant6' */
const volatile real32_T Tdf_control_err_derating_slope = 0.0F;/* Referenced by: '<S46>/Constant2' */
const volatile real32_T Tdf_control_error_threshold = 1.0F;/* Referenced by: '<S46>/Constant' */
const volatile real32_T Tdf_der_thrs_curv_x_scheduling[5] = { 0.0F, 0.001F,
  0.002F, 0.003F, 0.004F } ;       /* Referenced by: '<S44>/1-D Lookup Table' */

const volatile real32_T Tdf_der_thrs_curv_y_scheduling[5] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F } ;                   /* Referenced by: '<S44>/1-D Lookup Table' */

const volatile real32_T Tdf_derating_end = 1.0F;/* Referenced by: '<S12>/Constant1' */
const volatile real32_T Tdf_derating_slope_x_scheduling[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S42>/1-D Lookup Table' */

const volatile real32_T Tdf_derating_slope_y_scheduling[11] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                   /* Referenced by: '<S42>/1-D Lookup Table' */

const volatile real32_T Tdf_derating_start = 0.375F;/* Referenced by: '<S12>/Constant' */
const volatile real32_T Tdf_derating_thrs_x_scheduling[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S44>/1-D Lookup Table1' */

const volatile real32_T Tdf_derating_thrs_y_scheduling[11] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S44>/1-D Lookup Table1' */

const volatile real32_T Tdf_idle_enable_min_latency = 100.0F;/* Referenced by: '<S19>/Constant10' */
const volatile real32_T Tdf_idle_max_curvature_error = 0.0004F;/* Referenced by: '<S19>/Constant6' */
const volatile real32_T Tdf_idle_max_delta_f_dot = 0.06F;/* Referenced by: '<S19>/Constant4' */
const volatile real32_T Tdf_idle_max_heading_error = 0.1F;/* Referenced by: '<S19>/Constant8' */
const volatile real32_T Tdf_idle_max_kappa = 0.0008F;/* Referenced by: '<S19>/Constant3' */
const volatile real32_T Tdf_idle_max_lateral_accel = 0.3F;/* Referenced by: '<S19>/Constant2' */
const volatile real32_T Tdf_idle_max_lateral_error = 0.05F;/* Referenced by: '<S19>/Constant5' */
const volatile real32_T Tdf_idle_max_steer_angle = 0.1F;/* Referenced by: '<S19>/Constant9' */
const volatile real32_T Tdf_idle_max_torque_request = 0.05F;/* Referenced by:
                                                             * '<S19>/Constant'
                                                             * '<S19>/Constant1'
                                                             */
const volatile real32_T Tdf_idle_max_yaw_rate = 0.01F;/* Referenced by: '<S19>/Constant7' */
const volatile real32_T Tdf_maximum_delta_f_dot = 2.0F;/* Referenced by: '<S47>/Constant1' */
const volatile real32_T Tdf_min_steer_trq_cls_x_schedul[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                  /* Referenced by: '<S17>/1-D Lookup Table2' */

const volatile real32_T Tdf_min_steer_trq_cls_y_schedul[11] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                  /* Referenced by: '<S17>/1-D Lookup Table2' */

const volatile real32_T Tdf_out_scale_filt_min_residual = 0.1F;/* Referenced by: '<S13>/Constant6' */
const volatile real32_T Tdf_sac_pritaty_omega = 7.0F;/* Referenced by: '<S47>/Constant' */
const volatile real32_T Tdf_st_wheel_unbalance_factor = 0.0F;/* Referenced by: '<S25>/Constant' */
const volatile real32_T Tdf_steer_torque_comp_slope = 5.0F;/* Referenced by: '<S31>/Constant1' */
const volatile real32_T Tdf_steer_torque_comp_slope_ldp = 1.9F;/* Referenced by: '<S31>/Constant2' */
const volatile real32_T Tdf_steer_torque_comp_thrs_ldp = 2.5F;/* Referenced by: '<S17>/Constant' */
const volatile real32_T Tdf_steer_trq_cmp_ldp_x_schedul[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S17>/1-D Lookup Table' */

const volatile real32_T Tdf_steer_trq_cmp_ldp_y_schedul[11] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                   /* Referenced by: '<S17>/1-D Lookup Table' */

const volatile real32_T Tdf_steer_trq_cmp_slp_x_schedul[12] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 140.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S31>/1-D Lookup Table' */

const volatile real32_T Tdf_steer_trq_cmp_slp_y_schedul[12] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                   /* Referenced by: '<S31>/1-D Lookup Table' */

const volatile real32_T Tdf_steer_trq_comp_reduced_thrs = 0.1F;/* Referenced by: '<S17>/Constant7' */
const volatile real32_T Tdf_torque_der_filt_coeff_hf = 0.1F;/* Referenced by: '<S14>/Constant3' */
const volatile real32_T Tdf_torque_derating_filt_coeff = 0.01537F;/* Referenced by: '<S13>/Constant1' */
const volatile real32_T Tdf_trq_der_init_corr_x_schedul[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S36>/1-D Lookup Table' */

const volatile real32_T Tdf_trq_der_init_corr_y_schedul[11] = { 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;
                                   /* Referenced by: '<S36>/1-D Lookup Table' */

const volatile real32_T Tdf_trq_derating_max_curv = 0.004F;/* Referenced by: '<S44>/Constant' */
const volatile real32_T Tdf_trq_derating_max_init_trq = 5.0F;/* Referenced by: '<S36>/Constant' */
const volatile real32_T Tdf_trq_derating_slope_hf_path = 0.002F;/* Referenced by: '<S41>/Constant' */
const volatile real32_T Tdf_trq_derating_slope_hi_sens = 1.46F;/* Referenced by: '<S42>/Constant2' */
const volatile real32_T Tdf_trq_derating_thrs_hi_sens = 0.3F;/* Referenced by: '<S21>/Constant3' */
const volatile real32_T Tdf_trq_derating_thrs_x_schedul[11] = { 0.0F, 10.0F,
  20.0F, 30.0F, 40.0F, 60.0F, 80.0F, 100.0F, 120.0F, 180.0F, 250.0F } ;
                                   /* Referenced by: '<S21>/1-D Lookup Table' */

const volatile real32_T Tdf_trq_derating_thrs_y_schedul[11] = { 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F } ;
                                   /* Referenced by: '<S21>/1-D Lookup Table' */

const volatile real32_T Tdf_trq_filter_coeff = 0.1F;/* Referenced by:
                                                     * '<S33>/Constant'
                                                     * '<S49>/Constant1'
                                                     * '<S49>/Constant3'
                                                     */
const volatile real32_T Tdf_trq_min_stiffness_inc = 0.1F;/* Referenced by: '<S20>/Constant' */
const volatile real32_T Tdf_vel_derating_filt_coeff = 0.02F;/* Referenced by:
                                                             * '<S45>/Constant3'
                                                             * '<S59>/Constant3'
                                                             */
const volatile real32_T Tdf_velocity_derating_slope = 0.001F;/* Referenced by: '<S48>/Constant2' */
const volatile real32_T Tdf_velocity_derating_threshold = 20.0F;/* Referenced by: '<S48>/Constant' */
const volatile real32_T VEH_Force_Long_Tire_Factor = 2.0463E-7F;/* Referenced by: '<S111>/Constant2' */
const volatile real32_T VEH_Vehicle_Reduced_Mass = 971.863586F;/* Referenced by: '<S111>/Constant1' */
const volatile real32_T VEH_Vehicle_Selfsteering_Factor = 0.00506F;/* Referenced by: '<S115>/Constant1' */
const volatile real32_T VEH_Vehicle_Wheel_Base = 2.73F;/* Referenced by:
                                                        * '<S64>/Constant'
                                                        * '<S65>/Constant1'
                                                        * '<S115>/Constant'
                                                        */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */


/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T fSteerAngle_deg;              /* '<S11>/Switch2' */
real32_T fSteerWhlAngle_deg;           /* '<S11>/Product1' */
real32_T fLonAccCmd;                   /* '<S287>/Merge' */
uint8_T uiEPSRequest_nu;               /* '<S251>/Switch' */
uint8_T uiLonAccRequest_nu;            /* '<S286>/Switch' */

/* Exported data definition */



/* Definition for custom storage class: Global */
real32_T ADP_Dyc_Corr_Factor;          /* '<S181>/Add' */
real32_T ADP_Dyc_Corr_Factor_state;    /* '<S181>/Unit Delay' */
real32_T CAM_Lateral_Error_Sign;       /* '<S167>/Sign' */
real32_T Corretion_Factor_Left_Vect[9];/* '<S82>/Unit Delay1' */
real32_T Corretion_Factor_Right_Vect[9];/* '<S82>/Unit Delay' */
real32_T DMC_Integtrator_Output;       /* '<S104>/Unit Delay' */
real32_T DMC_Integtrator_Output1;      /* '<S103>/Unit Delay' */
real32_T DMC_Integtrator_Output2;      /* '<S107>/Unit Delay' */
real32_T DMC_Integtrator_Output3;      /* '<S106>/Unit Delay' */
real32_T DMC_Integtrator_Output4;      /* '<S105>/Unit Delay' */
real32_T DYC_Boost_Filter_Output_state;/* '<S126>/Discrete-Time Integrator' */
real32_T DYC_Filter_Kappa_Command;     /* '<S85>/Switch2' */
real32_T DYC_Kappa_Dot_Filter_state;   /* '<S93>/Unit Delay' */
real32_T DYC_Steer_Angle_Feedforward;  /* '<S87>/Product5' */
real32_T Delta_F_Rate_Limitation_state;/* '<S254>/Unit Delay' */
real32_T HEC_Yaw_Rate_Filter;          /* '<S112>/Add' */
real32_T HEC_Yaw_Rate_Filter_state;    /* '<S137>/Unit Delay' */
real32_T HEC_Yaw_Rate_Filter_state2;   /* '<S112>/Unit Delay' */
boolean_T Initialisation_Flag;         /* '<S179>/Constant' */
boolean_T Initialisation_Flag_state;   /* '<S149>/Unit Delay' */
real32_T LAT_Delta_F_Dot_Filter_state; /* '<S182>/Unit Delay' */
real32_T LAT_Delta_F_Vdy_Offset_pre;   /* '<S192>/Unit Delay' */
real32_T LAT_Eps_Torque_Filter_state;  /* '<S132>/Unit Delay' */
real32_T LAT_Filtered_Kappa_Cmd_pre;   /* '<S246>/Unit Delay' */
real32_T LAT_Filtered_Kappa_Cmd_pre_pre;/* '<S246>/Unit Delay1' */
real32_T LAT_Filtered_Kappa_Cmd_pre_pre_pre;/* '<S246>/Unit Delay2' */
real32_T LAT_Filtered_Kappa_state;     /* '<S234>/Unit Delay' */
real32_T LAT_Grad_Limited_Kappa_state; /* '<S235>/Unit Delay' */
real32_T LAT_Kappa_Command_filt_state; /* '<S232>/Unit Delay' */
real32_T LAT_Kappa_Command_filt_state1;/* '<S213>/Unit Delay' */
real32_T LAT_Kappa_Command_pre;        /* '<S194>/Unit Delay' */
real32_T LAT_Kappa_Command_state;      /* '<S231>/Unit Delay' */
real32_T LAT_Kappa_Dot_Filter_state;   /* '<S184>/Unit Delay' */
real32_T LAT_Kappa_Linz_Filter_Output; /* '<S242>/Unit Delay' */
real32_T LAT_Kappa_Linz_Heading_Err_Mem_state;/* '<S245>/Unit Delay' */
real32_T LAT_Kappa_Linz_Lat_Error_Mem_state;/* '<S244>/Unit Delay' */
real32_T LAT_OC_State_Preload;         /* '<S214>/Switch3' */
boolean_T LAT_Oc_Cal_Hold_Flag;        /* '<S204>/Relational Operator' */
boolean_T LAT_Oc_Cal_Hold_Flag_Shrt;   /* '<S205>/Relational Operator' */
boolean_T LAT_Oc_Disable_Flag;         /* '<S207>/Or Operator' */
boolean_T LAT_Oc_Dys_Active;           /* '<S208>/Not Operator' */
real32_T LAT_Oc_Filtered_Kappa_Cam;    /* '<S231>/Add' */
boolean_T LAT_Oc_High_Driver_Torque;   /* '<S198>/Logical Operator3' */
boolean_T LAT_Oc_Implaus_Lateral_Error;/* '<S210>/Relational Operator' */
real32_T LAT_Oc_Integrator_Input;      /* '<S206>/Product' */
real32_T LAT_Oc_Integrator_Input_Kappa;/* '<S201>/Product2' */
real32_T LAT_Oc_Integrator_Input_Kappa_dbg;/* '<S201>/Switch1' */
real32_T LAT_Oc_Integrator_Output;     /* '<S6>/Constant1' */
real32_T LAT_Oc_Integrator_Sat_Out;    /* '<S216>/MinMax2' */
boolean_T LAT_Oc_Kappa_Active;         /* '<S198>/Logical Operator4' */
boolean_T LAT_Oc_Kappa_Con_Enb_Flag;   /* '<S230>/Relational Operator2' */
real32_T LAT_Oc_Kappa_Latency_state;   /* '<S230>/Unit Delay' */
real32_T LAT_Oc_Kappa_Status_dbg;      /* '<S219>/Add' */
boolean_T LAT_Oc_Max_Delta_F_Dot_Flag; /* '<S222>/Relational Operator' */
boolean_T LAT_Oc_Max_Drv_Trq_Flag;     /* '<S223>/Relational Operator' */
boolean_T LAT_Oc_Max_Flt_Kappa_Cmd_Flag;/* '<S211>/Logical Operator' */
boolean_T LAT_Oc_Max_Hea_Err_Flag;     /* '<S224>/Relational Operator' */
boolean_T LAT_Oc_Max_Kappa_Cmd_Flag;   /* '<S221>/Relational Operator' */
boolean_T LAT_Oc_Max_Lat_Acc_Flag;     /* '<S225>/Relational Operator' */
boolean_T LAT_Oc_Min_Veh_Vel_Flag;     /* '<S212>/Relational Operator' */
real32_T LAT_Oc_Sat_Integrator_state;  /* '<S200>/Unit Delay2' */
boolean_T LAT_Oc_Trigger_Flag_Kappa;   /* '<S202>/Logical Operator' */
real32_T LAT_Oc_Trigger_Flag_Kappa_state;/* '<S201>/Unit Delay' */
real32_T LAT_Oc_offset_filter_omega;   /* '<S201>/Switch3' */
real32_T LAT_Pt1_Output_Undelayed_pre; /* '<S268>/Unit Delay' */
real32_T LAT_Pt1_Output_Undelayed_pre2;/* '<S277>/Unit Delay' */
real32_T LAT_Sat_Dynamic_Threshold_int;/* '<S69>/Unit Delay' */
real32_T LAT_Sat_Dynamic_Threshold_state;/* '<S62>/Unit Delay' */
boolean_T LAT_Status_Firsst_Run_pre;   /* '<S203>/Unit Delay1' */
boolean_T LAT_Status_First_Run_state;  /* '<S214>/Unit Delay3' */
real32_T LAT_Stiffness_Request_Factor_pre;/* '<S20>/Unit Delay' */
boolean_T LAT_Vdy_Offset_Used_pre;     /* '<S200>/Unit Delay1' */
real32_T LAT_Velocity_Derating_Factor_state;/* '<S45>/Unit Delay' */
real32_T LAT_Yaw_Rate_Filter_state;    /* '<S190>/Unit Delay' */
real32_T LDC_Sac_Parity_Filter_state;  /* '<S70>/Unit Delay' */
boolean_T LDP_Active_state1;           /* '<S187>/Unit Delay' */
boolean_T LDP_Active_state2;           /* '<S187>/Unit Delay1' */
real32_T LKC_Delta_Psi_pre1;           /* '<S128>/Unit Delay1' */
real32_T LKC_Delta_Psi_pre2;           /* '<S128>/Unit Delay2' */
real32_T LKC_Delta_Psi_pre3;           /* '<S128>/Unit Delay3' */
real32_T LKC_Delta_Psi_pre4;           /* '<S128>/Unit Delay4' */
real32_T LKC_Delta_Psi_pre5;           /* '<S128>/Unit Delay5' */
real32_T LKC_Delta_Psi_pre6;           /* '<S128>/Unit Delay6' */
real32_T LKC_Delta_Psi_pre7;           /* '<S128>/Unit Delay7' */
real32_T LKC_Delta_Psi_pre8;           /* '<S128>/Unit Delay8' */
real32_T LKC_Delta_Psi_pre9;           /* '<S128>/Unit Delay9' */
real32_T LKC_Delta_Ys_pre;             /* '<S79>/Unit Delay1' */
uint16_T LTLE_Waiting_Counter_state;   /* '<S144>/Unit Delay3' */
uint16_T LTRE_Waiting_Counter_state;   /* '<S144>/Unit Delay2' */
real32_T Last_Seg_Correction_Factor_Left_Scal_state;/* '<S151>/Unit Delay1' */
real32_T Last_Seg_Correction_Factor_Right_Scal_state;/* '<S151>/Unit Delay2' */
real32_T Lat_ldp_startup_time_state;   /* '<S187>/Unit Delay2' */
real32_T Lat_oc_minimum_latency_shrt_state;/* '<S205>/Unit Delay' */
real32_T Lat_oc_minimum_latency_state; /* '<S204>/Unit Delay' */
real32_T Lat_status_first_run_timer_state;/* '<S188>/Unit Delay2' */
real32_T Lateral_Error_Delta;          /* '<S167>/Subtract' */
real32_T Lateral_Error_Invalid_state;  /* '<S167>/Resettable Delay4' */
real32_T Lateral_Error_Mean;           /* '<S167>/Divide' */
real32_T Lateral_Error_Mean_state;     /* '<S167>/Resettable Delay5' */
real32_T Lon_Acc_Rate_Limitation_state;/* '<S289>/Unit Delay' */
real32_T Mean_Sample_Update;           /* '<S168>/Unit Delay1' */
real32_T Mean_Sample_Update_pre1;      /* '<S167>/Resettable Delay' */
real32_T Mean_Sample_Update_pre2;      /* '<S167>/Resettable Delay1' */
real32_T Mean_Sample_Update_pre3;      /* '<S167>/Resettable Delay2' */
real32_T Mean_Sample_Update_pre4;      /* '<S167>/Resettable Delay3' */
real32_T Mean_Sample_Update_sum;       /* '<S168>/Resettable Delay1' */
real32_T Mean_Vehicle_Velocity;        /* '<S168>/Unit Delay3' */
real32_T Mean_Vehicle_Velocity_sum;    /* '<S168>/Resettable Delay3' */
real32_T Mean_kappa_command;           /* '<S168>/Unit Delay2' */
real32_T Mean_kappa_command_sum;       /* '<S168>/Resettable Delay2' */
boolean_T New_Update_Aval;             /* '<S168>/Unit Delay' */
real32_T New_Update_Aval_sum;          /* '<S168>/Resettable Delay' */
uint16_T RTLE_Waiting_Counter_state;   /* '<S144>/Unit Delay1' */
uint16_T RTRE_Waiting_Counter_state;   /* '<S144>/Unit Delay' */
real32_T SAC_Angle_Command_Corr;       /* '<S6>/Constant10' */
real32_T SAC_Angle_Command_Rate_Limiter_state;/* '<S10>/Unit Delay' */
real32_T SAC_Angle_Command_Yawrate_Fback;/* '<S81>/Switch' */
real32_T SAC_Arbitrated_Angle_Cmd;     /* '<S77>/Switch' */
real32_T SAC_Arbitrated_Angle_Cmd_Raw; /* '<S84>/Gain1' */
real32_T SAC_Compensation_Angle_Command;/* '<S79>/Add1' */
real32_T SAC_Control_Error;            /* '<S4>/Subtract' */
real32_T SAC_Control_Error_Eps;        /* '<S11>/Subtract1' */
real32_T SAC_Control_Error_pre;        /* '<S5>/Unit Delay' */
real32_T SAC_Control_Error_state;      /* '<S1>/Unit Delay2' */
real32_T SAC_Delta_Psi_Dot_state;      /* '<S130>/Unit Delay' */
real32_T SAC_Delta_Psi_state;          /* '<S128>/Unit Delay' */
real32_T SAC_Derated_Angle_Command;    /* '<S9>/Add' */
boolean_T SAC_Disable_pre;             /* '<S132>/Unit Delay1' */
boolean_T SAC_Disable_pre_pre;         /* '<S132>/Unit Delay2' */
real32_T SAC_Integrator_Sat_Out;       /* '<S80>/Unit Delay1' */
real32_T SAC_Pt1_Filter1_state;        /* '<S129>/Unit Delay' */
real32_T SAC_Rate_Lim_Angle_Command;   /* '<S10>/Add1' */
real32_T SAC_Trq_Derating_Factor;      /* '<S12>/Switch3' */
real32_T SAC_Yrc_Angle_Command;        /* '<S80>/Switch4' */
real32_T SAC_Yrc_Control_Error;        /* '<S80>/Add' */
real32_T SAT_Req_Dyn_Steer_Angle_Max;  /* '<S74>/Min' */
real32_T SAT_Req_Steer_Angle_Max;      /* '<S75>/Min' */
real32_T SAT_Saturated_Angle_Command;  /* '<S67>/Switch' */
real32_T TDF_Composite_Derating_Factor;/* '<S16>/Switch1' */
real32_T TDF_Control_Error_Factor_state;/* '<S49>/Unit Delay' */
real32_T TDF_Derating_Time_state;      /* '<S57>/Unit Delay' */
boolean_T TDF_Driver_Counter_Steering; /* '<S17>/LessThanOrEqual' */
real32_T TDF_Flt_Drv_Comp_Factor_state;/* '<S50>/Unit Delay' */
real32_T TDF_Idle_Derating_Factor;     /* '<S19>/Switch1' */
real32_T TDF_Idle_Enable_Condition_state;/* '<S35>/Unit Delay' */
real32_T TDF_Ldp_Override_Factor_state;/* '<S12>/Unit Delay' */
real32_T TDF_Max_Steer_Torque;         /* '<S38>/Max1' */
real32_T TDF_Saturated_Angle_Command_Error_state;/* '<S47>/Unit Delay' */
real32_T TDF_Selected_State_Derating_Factor;/* '<S24>/Subtract' */
real32_T TDF_Selected_Torque_Source;   /* '<S29>/Multiport Switch' */
real32_T TDF_State_Derating_Factor_Comp_state;/* '<S54>/Unit Delay' */
real32_T TDF_Steer_Torque_Sample_pre;  /* '<S39>/Unit Delay' */
real32_T TDF_Torque_Der_Factor_HF_Path;/* '<S41>/Max' */
real32_T TDF_Torque_Der_Factor_HF_Path_state;/* '<S14>/Unit Delay' */
real32_T TDF_Torque_Derating_Factor;   /* '<S40>/Max' */
real32_T TDF_Torque_Derating_Factor_state;/* '<S13>/Unit Delay' */
real32_T TDF_Torque_Derating_Slop_Arb; /* '<S42>/Add' */
real32_T TDF_Torque_Request_Factor;    /* '<S30>/Max' */
real32_T TDF_Torque_Request_Factor_state;/* '<S33>/Unit Delay' */
real32_T TDF_Trq_Derating_Threshold_Arb;/* '<S43>/Add' */
real32_T TDF_Vehicle_Steer_Torque_Factor;/* '<S32>/Max1' */
real32_T VEH_Abs_Steer_Torque_Comp_pre;/* '<S38>/Unit Delay1' */
real32_T VEH_Delta_F_Oc;               /* '<S195>/Gain' */
real32_T VEH_Delta_F_Offset;           /* '<S196>/Add' */
real32_T VEH_Delta_F_Pre;              /* '<S1>/Unit Delay' */
real32_T VEH_Delta_F_pre1;             /* '<S282>/Unit Delay' */
real32_T VEH_Delta_F_pre2;             /* '<S282>/Unit Delay1' */
real32_T VEH_Delta_F_pre3;             /* '<S282>/Unit Delay2' */
real32_T VEH_Steer_Torque_Comp;        /* '<S59>/Add' */
real32_T VEH_Steer_Torque_Comp_state;  /* '<S59>/Unit Delay' */
real32_T VEH_Yaw_Rate_Dot_sate;        /* '<S140>/Unit Delay' */
real32_T VEH_Yaw_Rate_state;           /* '<S136>/Unit Delay' */
real32_T Veh_Lat_Acc_Filt;             /* '<S147>/Unit Delay' */
uint32_T Vehicle_Speed_Segment_state;  /* '<S151>/Unit Delay' */
real32_T fDeltaFCmd_State;             /* '<S11>/Unit Delay1' */
uint8_T uiDeltaF_Request_nu_pre;       /* '<S251>/Unit Delay' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Model step function */
void LaDMC_step(void)
{
  int32_T i;
  real32_T rtb_Switch_iu[18];
  real32_T TDF_Torque_Request_Factor_sta_0;
  real32_T rtb_Abs;
  real32_T rtb_Abs1_j;
  real32_T rtb_Add_hp5;
  real32_T rtb_Add_nx;
  real32_T rtb_Add_p2;
  real32_T rtb_DMC_Integtrator_Output_b;
  real32_T rtb_DMC_Integtrator_Output_ks;
  real32_T rtb_Gain_d;
  real32_T rtb_Hec_r_factor;
  real32_T rtb_LAT_Delta_F_cmd;
  real32_T rtb_LAT_Kappa_Gradient_Fading_S;
  real32_T rtb_LAT_Saturated_Delta_F_Cmd;
  real32_T rtb_Max_a;
  real32_T rtb_MinMax;
  real32_T rtb_MinMax_k;
  real32_T rtb_Product4_b;
  real32_T rtb_Product_lg;
  real32_T rtb_SAC_Arbitrated_Angle_Cmd;
  real32_T rtb_SAC_Trq_Derating_Factor;
  real32_T rtb_SAT_Max_Delta_F_Cmd;
  real32_T rtb_SAT_Max_Steer_Angle_Cmd_Sel;
  real32_T rtb_Sac_ts2;
  real32_T rtb_Subtract_jc;
  real32_T rtb_Switch_iy_idx_0;
  real32_T rtb_TDF_State_Derating_Factor_C;
  real32_T rtb_TRJ_Timebased_cmd_idx_0;
  real32_T rtb_TRJ_Timebased_cmd_idx_1;
  real32_T rtb_TRJ_Timebased_cmd_idx_2;
  real32_T rtb_TRJ_Timebased_cmd_idx_3;
  real32_T rtb_Tdf_min_steer_torque_class;
  real32_T rtb_UnitDelay5;
  real32_T rtb_UnitDelay6;
  real32_T rtb_VEH_Vehicle_Speed;
  uint32_T LAT_Oc_Kappa_Active_tmp;
  uint32_T rtb_SAC_Controller_Mode;
  uint32_T rtb_Vehicle_Speed_Index;
  uint16_T rtb_Maximum_Waitiing_Counter;
  uint8_T rtb_Turn_And_Error_Side_f;
  boolean_T rtb_Compare_hu;
  boolean_T rtb_LAT_Enable_Lateral_Control;
  boolean_T rtb_LDP_Active;
  boolean_T rtb_NOT_n;
  boolean_T rtb_NotOperator2_0;
  boolean_T rtb_OR;
  boolean_T rtb_OrOperator;
  boolean_T rtb_RelationalOperator_b;
  boolean_T rtb_SAC_Disable;

  /* Switch: '<S251>/Switch' incorporates:
   *  Inport: '<Root>/Inport61'
   *  Inport: '<Root>/Inport63'
   *  Switch: '<S251>/Switch1'
   */
  if (ADAS_Delta_F_Enable_flag) {
    /* Switch: '<S251>/Switch' incorporates:
     *  Constant: '<S251>/Constant'
     */
    uiEPSRequest_nu = 1U;
  } else if (NOP_Delta_F_Enable_flag) {
    /* Switch: '<S251>/Switch1' incorporates:
     *  Constant: '<S251>/Constant1'
     *  Switch: '<S251>/Switch'
     */
    uiEPSRequest_nu = 2U;
  } else {
    /* Switch: '<S251>/Switch' incorporates:
     *  Constant: '<S251>/Constant3'
     *  Switch: '<S251>/Switch1'
     */
    uiEPSRequest_nu = 0U;
  }

  /* End of Switch: '<S251>/Switch' */

  /* Sum: '<S188>/Add' incorporates:
   *  Constant: '<S188>/Sac_ts'
   *  UnitDelay: '<S188>/Unit Delay2'
   */
  rtb_Add_p2 = Lat_status_first_run_timer_state + Sac_ts;

  /* RelationalOperator: '<S188>/Relational Operator4' incorporates:
   *  Constant: '<S188>/Lat_status_first_run_time'
   */
  LAT_Status_Firsst_Run_pre = (rtb_Add_p2 > Lat_status_first_run_time);

  /* Switch: '<S214>/Switch3' incorporates:
   *  Logic: '<S214>/Logical Operator'
   *  Logic: '<S214>/Logical Operator1'
   *  UnitDelay: '<S214>/Unit Delay3'
   */
  if (LAT_Status_Firsst_Run_pre && (!LAT_Status_First_Run_state)) {
    /* Switch: '<S214>/Switch3' incorporates:
     *  Inport: '<Root>/Inport65'
     */
    LAT_OC_State_Preload = Dmc_DeltaF_offset_par;
  } else {
    /* Switch: '<S214>/Switch3' incorporates:
     *  Constant: '<S214>/Constant1'
     */
    LAT_OC_State_Preload = 0.0F;
  }

  /* End of Switch: '<S214>/Switch3' */

  /* MinMax: '<S204>/MinMax' incorporates:
   *  Constant: '<S204>/Lat_oc_minimum_latency'
   *  UnitDelay: '<S204>/Unit Delay'
   */
  rtb_MinMax = fminf(Lat_oc_minimum_latency, Lat_oc_minimum_latency_state);

  /* RelationalOperator: '<S204>/Relational Operator' incorporates:
   *  Constant: '<S204>/Lat_oc_minimum_latency'
   */
  LAT_Oc_Cal_Hold_Flag = (Lat_oc_minimum_latency != rtb_MinMax);

  /* MinMax: '<S205>/MinMax' incorporates:
   *  Constant: '<S205>/Lat_oc_minimum_latency_shrt'
   *  UnitDelay: '<S205>/Unit Delay'
   */
  rtb_MinMax_k = fminf(Lat_oc_minimum_latency_shrt,
                       Lat_oc_minimum_latency_shrt_state);

  /* RelationalOperator: '<S205>/Relational Operator' incorporates:
   *  Constant: '<S205>/Lat_oc_minimum_latency_shrt'
   */
  LAT_Oc_Cal_Hold_Flag_Shrt = (Lat_oc_minimum_latency_shrt != rtb_MinMax_k);

  /* Logic: '<S208>/Or Operator' */
  rtb_OrOperator = (LAT_Oc_Cal_Hold_Flag || LAT_Oc_Cal_Hold_Flag_Shrt);

  /* Logic: '<S208>/Not Operator' */
  LAT_Oc_Dys_Active = !rtb_OrOperator;

  /* Switch: '<S200>/Switch1' incorporates:
   *  Constant: '<S200>/Lat_oc_max_delta_offset'
   *  Constant: '<S200>/Lat_oc_max_delta_offset_kappa'
   */
  if (LAT_Oc_Dys_Active) {
    rtb_Product4_b = Lat_oc_max_delta_offset;
  } else {
    rtb_Product4_b = Lat_oc_max_delta_offset_kappa;
  }

  /* End of Switch: '<S200>/Switch1' */

  /* Switch: '<S200>/Switch3' incorporates:
   *  Constant: '<S200>/Constant2'
   *  Inport: '<Root>/Inport72'
   */
  if (DMC_NVRAMReset_par) {
    LAT_Oc_Sat_Integrator_state = 0.0F;
  }

  /* MinMax: '<S216>/MinMax1' incorporates:
   *  Sum: '<S200>/Add3'
   */
  rtb_SAC_Trq_Derating_Factor = fminf(LAT_Oc_Sat_Integrator_state +
    LAT_OC_State_Preload, rtb_Product4_b);

  /* Gain: '<S200>/Gain' */
  rtb_Product4_b = -rtb_Product4_b;

  /* MinMax: '<S216>/MinMax2' */
  LAT_Oc_Integrator_Sat_Out = fmaxf(rtb_SAC_Trq_Derating_Factor, rtb_Product4_b);

  /* Sum: '<S196>/Add' incorporates:
   *  Constant: '<S196>/LAT_Delta_F_Offset'
   */
  VEH_Delta_F_Offset = LAT_Oc_Integrator_Sat_Out + Lat_delta_f_offset;

  /* Gain: '<S195>/Gain' incorporates:
   *  Constant: '<S183>/Constant'
   *  Inport: '<Root>/Inport15'
   *  Inport: '<Root>/Inport17'
   *  MinMax: '<S183>/Max'
   *  Product: '<S183>/Divide'
   */
  VEH_Delta_F_Oc = VEH_SteerAngle / fmaxf(EPS_Gear_Ratio, 1.0F) * 57.2957802F;

  /* If: '<S252>/judgement_If&Else' incorporates:
   *  Inport: '<Root>/Inport49'
   *  Inport: '<S255>/Inport'
   *  Inport: '<S257>/Inport'
   */
  if (uiEPSRequest_nu == 1) {
    /* Switch: '<S252>/Switch3' incorporates:
     *  Constant: '<S252>/Constant2'
     *  Inport: '<Root>/Inport64'
     *  S-Function (sfix_bitop): '<S252>/Bitwise AND2'
     */
    if ((Dmc_offset_calibration_mode_par & 1U) != 0U) {
      rtb_SAC_Trq_Derating_Factor = VEH_Delta_F_Offset;
    } else {
      rtb_SAC_Trq_Derating_Factor = 0.0F;
    }

    /* End of Switch: '<S252>/Switch3' */

    /* Outputs for IfAction SubSystem: '<S252>/If' incorporates:
     *  ActionPort: '<S256>/Action Port'
     */
    /* DataTypeConversion: '<S256>/Data Type Conversion' incorporates:
     *  Inport: '<Root>/Inport59'
     *  Sum: '<S252>/Add'
     */
    rtb_Product4_b = ADAS_Delta_F_Cmd + rtb_SAC_Trq_Derating_Factor;

    /* End of Outputs for SubSystem: '<S252>/If' */
  } else if (uiEPSRequest_nu == 2) {
    /* Outputs for IfAction SubSystem: '<S252>/If1' incorporates:
     *  ActionPort: '<S257>/Action Port'
     */
    rtb_Product4_b = NOP_Delta_F_Cmd;

    /* End of Outputs for SubSystem: '<S252>/If1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S252>/Else' incorporates:
     *  ActionPort: '<S255>/Action Port'
     */
    rtb_Product4_b = VEH_Delta_F_Oc;

    /* End of Outputs for SubSystem: '<S252>/Else' */
  }

  /* End of If: '<S252>/judgement_If&Else' */

  /* Logic: '<S187>/OR4' incorporates:
   *  Constant: '<S187>/Constant12'
   *  Constant: '<S187>/Constant13'
   *  Inport: '<Root>/Inport24'
   *  RelationalOperator: '<S187>/Equal5'
   *  RelationalOperator: '<S187>/Equal6'
   */
  rtb_LDP_Active = ((LDP_Status == 3) || (LDP_Status == 5));

  /* Sum: '<S254>/Subtract' incorporates:
   *  UnitDelay: '<S254>/Unit Delay'
   */
  rtb_SAC_Trq_Derating_Factor = rtb_Product4_b - Delta_F_Rate_Limitation_state;

  /* Abs: '<S254>/Abs' */
  rtb_Abs = fabsf(rtb_SAC_Trq_Derating_Factor);

  /* Gain: '<S254>/Sac_ts2' incorporates:
   *  Constant: '<S253>/Constant'
   *  Inport: '<Root>/Inport15'
   *  Product: '<S253>/Divide'
   */
  rtb_Sac_ts2 = Delta_F_Gradient_Par / EPS_Gear_Ratio * Sac_ts;

  /* Signum: '<S254>/Sign' */
  if (rtb_SAC_Trq_Derating_Factor < 0.0F) {
    rtb_SAC_Trq_Derating_Factor = -1.0F;
  } else {
    if (rtb_SAC_Trq_Derating_Factor > 0.0F) {
      rtb_SAC_Trq_Derating_Factor = 1.0F;
    }
  }

  /* End of Signum: '<S254>/Sign' */

  /* Sum: '<S254>/Add' incorporates:
   *  Abs: '<S254>/Abs1'
   *  MinMax: '<S254>/Min'
   *  Product: '<S254>/Product1'
   *  Product: '<S254>/Product4'
   *  UnitDelay: '<S254>/Unit Delay'
   */
  Delta_F_Rate_Limitation_state = fminf(fabsf(rtb_Sac_ts2 *
    rtb_SAC_Trq_Derating_Factor), rtb_Abs) * rtb_SAC_Trq_Derating_Factor +
    Delta_F_Rate_Limitation_state;

  /* Switch: '<S254>/Switch1' incorporates:
   *  Constant: '<S253>/Constant1'
   *  Constant: '<S254>/Constant'
   *  MinMax: '<S254>/Max'
   *  MinMax: '<S254>/Min1'
   *  Product: '<S254>/Product5'
   *  Sum: '<S254>/Add1'
   *  Sum: '<S254>/Subtract1'
   */
  if (rtb_LDP_Active) {
    rtb_LAT_Delta_F_cmd = rtb_Product4_b;
  } else {
    rtb_LAT_Delta_F_cmd = fminf(Delta_F_Direct_Feedthrough_Par, fmaxf(0.0F,
      rtb_Abs - rtb_Sac_ts2)) * rtb_SAC_Trq_Derating_Factor +
      Delta_F_Rate_Limitation_state;
  }

  /* End of Switch: '<S254>/Switch1' */

  /* Gain: '<S278>/Gain' incorporates:
   *  Inport: '<Root>/Inport6'
   */
  rtb_Product4_b = 0.01F * LaKMC_angle_req_max_grad_scale;

  /* Sum: '<S278>/Add' incorporates:
   *  Constant: '<S278>/Constant2'
   *  Inport: '<Root>/Inport57'
   *  Inport: '<Root>/Inport58'
   *  MinMax: '<S278>/Min'
   *  Product: '<S278>/Product'
   *  Product: '<S278>/Product1'
   *  Sum: '<S278>/Subtract'
   */
  rtb_Product4_b = (1.0F - rtb_Product4_b) * fminf
    (Sac_delta_f_cmd_grad_barrier_par, Sac_delta_f_cmd_min_grad_par) +
    rtb_Product4_b * Sac_delta_f_cmd_grad_barrier_par;

  /* MinMax: '<S189>/Max' incorporates:
   *  Constant: '<S189>/Constant'
   *  Constant: '<S189>/Constant1'
   *  Gain: '<S189>/Gain'
   *  Inport: '<Root>/Inport16'
   *  MinMax: '<S189>/Min'
   */
  LAT_Stiffness_Request_Factor_pre = fmaxf(fminf(0.01F *
    LAT_Stiffness_Request_Factor, 1.0F), 0.0F);

  /* Switch: '<S7>/Switch2' */
  rtb_LAT_Enable_Lateral_Control = (uiEPSRequest_nu != 0);

  /* MinMax: '<S6>/Max' incorporates:
   *  Abs: '<S6>/Abs'
   *  Constant: '<S6>/Constant3'
   *  Inport: '<Root>/Inport'
   */
  rtb_VEH_Vehicle_Speed = fmaxf(fabsf(VEH_Vehicle_Speed), 0.01F);

  /* UnitDelay: '<S234>/Unit Delay' */
  rtb_Abs = LAT_Filtered_Kappa_state;

  /* Sum: '<S234>/Add1' incorporates:
   *  Constant: '<S234>/Constant1'
   *  Inport: '<Root>/Inport13'
   *  Inport: '<Root>/Inport2'
   *  Product: '<S234>/Divide'
   *  Product: '<S234>/Product'
   *  Sum: '<S234>/Add'
   *  Sum: '<S234>/Subtract'
   *  UnitDelay: '<S234>/Unit Delay'
   */
  LAT_Filtered_Kappa_state = VEH_cycletime / (VEH_cycletime +
    Lat_kappa_filter_coeff) * (LKC_Kappa_Command - LAT_Filtered_Kappa_state) +
    LAT_Filtered_Kappa_state;

  /* Switch: '<S234>/Switch' incorporates:
   *  Constant: '<S234>/Constant'
   *  Gain: '<S234>/Gain'
   *  Inport: '<Root>/Inport13'
   *  RelationalOperator: '<S234>/GreaterThan'
   *  Sum: '<S234>/Add2'
   *  UnitDelay: '<S234>/Unit Delay'
   */
  if (VEH_cycletime > 0.01) {
    rtb_Abs = LAT_Filtered_Kappa_state;
  } else {
    rtb_Abs = (LAT_Filtered_Kappa_state + rtb_Abs) * 0.5F;
  }

  /* End of Switch: '<S234>/Switch' */

  /* Sum: '<S242>/Subtract' incorporates:
   *  UnitDelay: '<S242>/Unit Delay'
   */
  rtb_SAT_Max_Delta_F_Cmd = rtb_Abs - LAT_Kappa_Linz_Filter_Output;

  /* Abs: '<S237>/Abs' incorporates:
   *  Abs: '<S240>/Abs'
   */
  rtb_Sac_ts2 = fabsf(rtb_Abs);

  /* MinMax: '<S237>/Max' incorporates:
   *  Abs: '<S237>/Abs'
   *  Constant: '<S237>/Constant3'
   *  Constant: '<S237>/Constant4'
   *  Constant: '<S237>/Constant5'
   *  Constant: '<S237>/Constant6'
   *  MinMax: '<S237>/Min2'
   *  Product: '<S237>/Product1'
   *  Sum: '<S237>/Subtract'
   */
  rtb_SAC_Arbitrated_Angle_Cmd = fmaxf(fminf((rtb_Sac_ts2 -
    Lat_kappa_linz_min_crv_progress) * Lat_kappa_linz_wt_fact_progress, 1.0F),
    0.0F);

  /* Switch: '<S238>/Switch' incorporates:
   *  Constant: '<S238>/Constant'
   *  Inport: '<Root>/Inport18'
   *  Inport: '<Root>/Inport19'
   */
  if (CAM_Latral_Error_Qf != 0.0F) {
    rtb_SAC_Trq_Derating_Factor = CAM_Lateral_Error;
  } else {
    rtb_SAC_Trq_Derating_Factor = 0.0F;
  }

  /* End of Switch: '<S238>/Switch' */

  /* Product: '<S244>/Divide' incorporates:
   *  Constant: '<S244>/Constant1'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S245>/Divide'
   *  Sum: '<S244>/Add'
   */
  rtb_LAT_Kappa_Gradient_Fading_S = VEH_cycletime / (VEH_cycletime +
    Lat_kappa_linz_lat_err_coeff);

  /* Sum: '<S244>/Add1' incorporates:
   *  Abs: '<S238>/Abs'
   *  Constant: '<S238>/Constant1'
   *  Constant: '<S238>/Constant2'
   *  Constant: '<S238>/Constant3'
   *  Constant: '<S238>/Constant4'
   *  MinMax: '<S238>/Max'
   *  MinMax: '<S238>/Max1'
   *  MinMax: '<S238>/Min'
   *  Product: '<S238>/Product'
   *  Product: '<S244>/Divide'
   *  Product: '<S244>/Product'
   *  Sum: '<S238>/Subtract'
   *  Sum: '<S244>/Subtract'
   *  UnitDelay: '<S244>/Unit Delay'
   */
  LAT_Kappa_Linz_Lat_Error_Mem_state = (fmaxf(fminf(fmaxf(fabsf
    (rtb_SAC_Trq_Derating_Factor) - Lat_kappa_linz_max_lat_error, 0.0F) *
    Lat_kappa_linz_lat_error_slope, 1.0F), -1.0F) -
    LAT_Kappa_Linz_Lat_Error_Mem_state) * rtb_LAT_Kappa_Gradient_Fading_S +
    LAT_Kappa_Linz_Lat_Error_Mem_state;

  /* Switch: '<S239>/Switch' incorporates:
   *  Constant: '<S239>/Constant'
   *  Inport: '<Root>/Inport10'
   *  Inport: '<Root>/Inport8'
   */
  if (CAM_Heading_Error_Qf != 0.0F) {
    rtb_SAC_Trq_Derating_Factor = CAM_Heading_Error;
  } else {
    rtb_SAC_Trq_Derating_Factor = 0.0F;
  }

  /* End of Switch: '<S239>/Switch' */

  /* Sum: '<S245>/Add1' incorporates:
   *  Abs: '<S239>/Abs'
   *  Constant: '<S239>/Constant1'
   *  Constant: '<S239>/Constant2'
   *  Constant: '<S239>/Constant3'
   *  Constant: '<S239>/Constant4'
   *  Constant: '<S239>/Constant5'
   *  MinMax: '<S239>/Max'
   *  MinMax: '<S239>/Max1'
   *  MinMax: '<S239>/Min'
   *  MinMax: '<S239>/Min1'
   *  Product: '<S239>/Product'
   *  Product: '<S245>/Product'
   *  Sum: '<S239>/Subtract'
   *  Sum: '<S245>/Subtract'
   *  UnitDelay: '<S245>/Unit Delay'
   */
  LAT_Kappa_Linz_Heading_Err_Mem_state = (fminf(fmaxf(fminf(fmaxf(fabsf
    (rtb_SAC_Trq_Derating_Factor) - Lat_kappa_linz_max_head_error, 0.0F) *
    Lat_kappa_linz_lat_error_slope, 1.0F), -1.0F), Lat_kappa_linz_head_mx_memshp)
    - LAT_Kappa_Linz_Heading_Err_Mem_state) * rtb_LAT_Kappa_Gradient_Fading_S +
    LAT_Kappa_Linz_Heading_Err_Mem_state;

  /* MinMax: '<S243>/Min' incorporates:
   *  Abs: '<S236>/Abs'
   *  Abs: '<S237>/Abs'
   *  Constant: '<S236>/Constant'
   *  Constant: '<S236>/Constant1'
   *  Constant: '<S236>/Constant2'
   *  Constant: '<S236>/Constant3'
   *  Constant: '<S236>/Constant4'
   *  Constant: '<S236>/Constant5'
   *  Constant: '<S237>/Constant'
   *  Constant: '<S237>/Constant1'
   *  Constant: '<S237>/Constant2'
   *  Constant: '<S240>/Constant'
   *  Constant: '<S240>/Constant1'
   *  Constant: '<S240>/Constant2'
   *  Constant: '<S240>/Constant3'
   *  Constant: '<S243>/Constant'
   *  Constant: '<S243>/Constant1'
   *  Constant: '<S243>/Constant2'
   *  MinMax: '<S236>/Max'
   *  MinMax: '<S236>/Max1'
   *  MinMax: '<S236>/Min'
   *  MinMax: '<S237>/Min'
   *  MinMax: '<S237>/Min1'
   *  MinMax: '<S240>/Max'
   *  MinMax: '<S240>/Min'
   *  MinMax: '<S243>/Max'
   *  Product: '<S236>/Product'
   *  Product: '<S236>/Product1'
   *  Product: '<S236>/Product2'
   *  Product: '<S237>/Product'
   *  Product: '<S240>/Product'
   *  Sum: '<S236>/Add'
   *  Sum: '<S236>/Subtract'
   *  Sum: '<S236>/Subtract1'
   *  Sum: '<S240>/Subtract'
   *  Sum: '<S243>/Add'
   *  UnitDelay: '<S244>/Unit Delay'
   *  UnitDelay: '<S245>/Unit Delay'
   */
  rtb_Sac_ts2 = fminf(1.0F, fmaxf(0.0F, ((((fmaxf(-1.0F, fminf(1.0F, fmaxf(0.0F,
    fabsf(rtb_SAT_Max_Delta_F_Cmd) - ((1.0F - rtb_SAC_Arbitrated_Angle_Cmd) *
    Lat_kappa_linz_filt_max_error + Lat_kappa_linz_filt_err_hi_curv *
    rtb_SAC_Arbitrated_Angle_Cmd)) * Lat_kappa_linz_error_slope)) - fminf
    (Lat_kappa_linz_height_mx_memshp, fminf(1.0F, Lat_kappa_linz_height_factor *
    rtb_Sac_ts2))) - fminf(fmaxf(Lat_kappa_linz_staight_mx_memshp -
    Lat_kappa_linz_straight_weight * rtb_Sac_ts2, 0.0F), 1.0F)) +
    LAT_Kappa_Linz_Lat_Error_Mem_state) - LAT_Kappa_Linz_Heading_Err_Mem_state)
    + Lat_kappa_linz_default_memshp));

  /* Sum: '<S242>/Add1' incorporates:
   *  Constant: '<S241>/Constant1'
   *  Constant: '<S241>/Constant2'
   *  Constant: '<S241>/Constant3'
   *  Gain: '<S241>/Gain'
   *  Inport: '<Root>/Inport13'
   *  Lookup_n-D: '<S241>/1-D Lookup Table'
   *  Product: '<S241>/Product'
   *  Product: '<S241>/Product1'
   *  Product: '<S241>/Product2'
   *  Product: '<S242>/Product'
   *  Product: '<S242>/Product1'
   *  Sum: '<S241>/Add'
   *  Sum: '<S241>/Subtract'
   *  Switch: '<S241>/Switch'
   *  UnitDelay: '<S242>/Unit Delay'
   */
  LAT_Kappa_Linz_Filter_Output = (look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(Lat_kppa_min_omega_x_scheduling
    [0])), ((const real32_T *)&(Lat_kppa_min_omega_y_scheduling[0])), 11U) *
    Lat_kappa_linz_filter_min_omega * (1.0F - rtb_Sac_ts2) +
    Lat_kappa_linz_filter_max_omega * rtb_Sac_ts2) * VEH_cycletime *
    rtb_SAT_Max_Delta_F_Cmd + LAT_Kappa_Linz_Filter_Output;

  /* Sum: '<S235>/Subtract' incorporates:
   *  UnitDelay: '<S235>/Unit Delay'
   */
  rtb_Abs1_j = rtb_Abs - LAT_Grad_Limited_Kappa_state;

  /* Abs: '<S235>/Abs' */
  rtb_Sac_ts2 = fabsf(rtb_Abs1_j);

  /* Switch: '<S248>/Switch' incorporates:
   *  Constant: '<S248>/Constant'
   *  Constant: '<S248>/Constant1'
   *  Constant: '<S250>/Constant'
   *  Constant: '<S250>/Constant1'
   *  Gain: '<S250>/Gain'
   *  Lookup_n-D: '<S250>/1-D Lookup Table'
   *  MinMax: '<S250>/Max'
   *  Product: '<S248>/Product'
   *  Product: '<S248>/Product1'
   *  Product: '<S250>/Product'
   *  Sum: '<S248>/Add'
   *  Sum: '<S248>/Subtract'
   */
  if (rtb_LDP_Active) {
    rtb_SAC_Arbitrated_Angle_Cmd = Lat_kappa_gradient_ldp;
  } else {
    /* Product: '<S249>/Product2' incorporates:
     *  Abs: '<S249>/Abs'
     *  Abs: '<S249>/Abs1'
     *  Constant: '<S249>/Constant'
     *  Constant: '<S249>/Constant1'
     *  Constant: '<S249>/Constant2'
     *  Constant: '<S249>/Constant3'
     *  Constant: '<S249>/Constant4'
     *  Constant: '<S249>/Constant5'
     *  Constant: '<S249>/Constant6'
     *  Constant: '<S249>/Constant7'
     *  Inport: '<Root>/LKC_Delta_Ys'
     *  MinMax: '<S249>/Max'
     *  MinMax: '<S249>/Max1'
     *  MinMax: '<S249>/Min'
     *  MinMax: '<S249>/Min1'
     *  Product: '<S249>/Product'
     *  Product: '<S249>/Product1'
     *  Sum: '<S249>/Subtract'
     *  Sum: '<S249>/Subtract1'
     */
    rtb_LAT_Kappa_Gradient_Fading_S = fminf(1.0F, fmaxf(0.0F, rtb_Sac_ts2 -
      Lat_kappa_max_lateral_error) * Lat_kappa_lateral_error_slope) * fminf(1.0F,
      fmaxf(0.0F, fabsf(LKC_Delta_Ys) - Lat_kappa_discharge_min_error) *
      Lat_kappa_discharge_slope);
    rtb_SAC_Arbitrated_Angle_Cmd = look1_iflf_binlxpw(3.6F *
      rtb_VEH_Vehicle_Speed, ((const real32_T *)
      &(Lat_max_kappa_grad_x_scheduling[0])), ((const real32_T *)
      &(Lat_max_kappa_grad_y_scheduling[0])), 11U) * Lat_max_kappa_gradient *
      (1.0F - rtb_LAT_Kappa_Gradient_Fading_S) + fmaxf(Lat_max_kappa_gradient,
      Lat_kappa_discharge_gradient) * rtb_LAT_Kappa_Gradient_Fading_S;
  }

  /* End of Switch: '<S248>/Switch' */

  /* Product: '<S235>/Product' incorporates:
   *  Inport: '<Root>/Inport13'
   */
  rtb_LAT_Kappa_Gradient_Fading_S = rtb_SAC_Arbitrated_Angle_Cmd * VEH_cycletime;

  /* Signum: '<S235>/Sign' */
  if (rtb_Abs1_j < 0.0F) {
    rtb_Abs1_j = -1.0F;
  } else {
    if (rtb_Abs1_j > 0.0F) {
      rtb_Abs1_j = 1.0F;
    }
  }

  /* End of Signum: '<S235>/Sign' */

  /* Sum: '<S235>/Add' incorporates:
   *  Abs: '<S235>/Abs1'
   *  MinMax: '<S235>/Min'
   *  Product: '<S235>/Product1'
   *  Product: '<S235>/Product2'
   *  UnitDelay: '<S235>/Unit Delay'
   */
  LAT_Grad_Limited_Kappa_state = fminf(rtb_Sac_ts2, fabsf(rtb_Abs1_j *
    rtb_LAT_Kappa_Gradient_Fading_S)) * rtb_Abs1_j +
    LAT_Grad_Limited_Kappa_state;

  /* Switch: '<S185>/Switch' incorporates:
   *  Constant: '<S235>/Constant'
   *  Constant: '<S235>/Constant1'
   *  Inport: '<Root>/Inport21'
   *  MinMax: '<S235>/Max'
   *  MinMax: '<S235>/Min1'
   *  Product: '<S235>/Product3'
   *  S-Function (sfix_bitop): '<S185>/Bitwise AND'
   *  Sum: '<S235>/Add1'
   *  Sum: '<S235>/Subtract1'
   *  UnitDelay: '<S242>/Unit Delay'
   */
  if ((Dmc_configuration_mode_par & 4U) != 0U) {
    rtb_LAT_Kappa_Gradient_Fading_S = LAT_Kappa_Linz_Filter_Output;
  } else {
    rtb_LAT_Kappa_Gradient_Fading_S = fminf(Lat_direct_feedthrough_kappa, fmaxf
      (0.0F, rtb_Sac_ts2 - rtb_LAT_Kappa_Gradient_Fading_S)) * rtb_Abs1_j +
      LAT_Grad_Limited_Kappa_state;
  }

  /* End of Switch: '<S185>/Switch' */

  /* Product: '<S21>/Product' incorporates:
   *  Gain: '<S21>/Gain'
   *  Inport: '<Root>/Inport48'
   *  Lookup_n-D: '<S21>/1-D Lookup Table'
   */
  rtb_SAC_Trq_Derating_Factor = look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed,
    ((const real32_T *)&(Tdf_trq_derating_thrs_x_schedul[0])), ((const real32_T *)
    &(Tdf_trq_derating_thrs_y_schedul[0])), 10U) *
    Tdf_torque_derating_threshold_par;

  /* Sum: '<S59>/Add' incorporates:
   *  UnitDelay: '<S59>/Unit Delay'
   */
  VEH_Steer_Torque_Comp = VEH_Steer_Torque_Comp_state;

  /* Product: '<S59>/Divide' incorporates:
   *  Constant: '<S59>/Constant3'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S45>/Divide'
   *  Sum: '<S59>/Add1'
   */
  rtb_Product_lg = 1.0F / (Tdf_vel_derating_filt_coeff + VEH_cycletime) *
    VEH_cycletime;

  /* Sum: '<S59>/Add' incorporates:
   *  Constant: '<S25>/Constant'
   *  Constant: '<S60>/Constant'
   *  Inport: '<Root>/Inport11'
   *  MinMax: '<S60>/Max'
   *  MinMax: '<S60>/Min'
   *  Product: '<S25>/Product'
   *  Product: '<S59>/Divide'
   *  Product: '<S59>/Product'
   *  Sum: '<S25>/Add'
   *  Sum: '<S59>/Subtract'
   */
  VEH_Steer_Torque_Comp = ((Tdf_st_wheel_unbalance_factor * fmaxf(-5.0F, fminf
    (5.0F, VEH_Delta_F_Oc)) + VEH_Steer_Torque) - VEH_Steer_Torque_Comp) *
    rtb_Product_lg + VEH_Steer_Torque_Comp;

  /* Product: '<S17>/Product4' incorporates:
   *  Gain: '<S17>/Gain4'
   *  Inport: '<Root>/Inport44'
   *  Lookup_n-D: '<S17>/1-D Lookup Table2'
   */
  rtb_Tdf_min_steer_torque_class = look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(Tdf_min_steer_trq_cls_x_schedul
    [0])), ((const real32_T *)&(Tdf_min_steer_trq_cls_y_schedul[0])), 10U) *
    Tdf_min_steer_torque_class_par;

  /* Abs: '<S32>/Abs' incorporates:
   *  Abs: '<S25>/Abs'
   */
  rtb_UnitDelay5 = fabsf(VEH_Steer_Torque_Comp);

  /* Signum: '<S32>/Sign' */
  if (VEH_Steer_Torque_Comp < 0.0F) {
    /* Signum: '<S31>/Sign' */
    rtb_UnitDelay6 = -1.0F;
  } else if (VEH_Steer_Torque_Comp > 0.0F) {
    /* Signum: '<S31>/Sign' */
    rtb_UnitDelay6 = 1.0F;
  } else {
    /* Signum: '<S31>/Sign' */
    rtb_UnitDelay6 = VEH_Steer_Torque_Comp;
  }

  /* MinMax: '<S32>/Max1' incorporates:
   *  Abs: '<S32>/Abs'
   *  Constant: '<S32>/Constant'
   *  Constant: '<S32>/Constant2'
   *  Inport: '<Root>/Inport47'
   *  MinMax: '<S32>/Max'
   *  MinMax: '<S32>/Min'
   *  Product: '<S32>/Product'
   *  Product: '<S32>/Product1'
   *  Signum: '<S32>/Sign'
   *  Sum: '<S32>/Subtract'
   */
  TDF_Vehicle_Steer_Torque_Factor = fmaxf(fminf(fmaxf(rtb_UnitDelay5 -
    rtb_Tdf_min_steer_torque_class, 0.0F) * rtb_UnitDelay6 *
    Tdf_steer_torque_sign_slope_par, 1.0F), -1.0F);

  /* Switch: '<S27>/Switch' incorporates:
   *  Constant: '<S187>/Constant11'
   *  Constant: '<S27>/Constant'
   *  Constant: '<S27>/Constant1'
   *  Inport: '<Root>/Inport12'
   *  RelationalOperator: '<S187>/Equal4'
   */
  if (LKC_DgrSide == 1) {
    i = -1;
  } else {
    i = 1;
  }

  /* End of Switch: '<S27>/Switch' */

  /* MinMax: '<S17>/Max' incorporates:
   *  Constant: '<S17>/Constant6'
   *  Product: '<S17>/Product3'
   */
  rtb_Sac_ts2 = fmaxf(TDF_Vehicle_Steer_Torque_Factor * (real32_T)i, 0.0F);

  /* Switch: '<S43>/Switch' incorporates:
   *  Constant: '<S37>/Constant'
   *  Inport: '<Root>/Inport51'
   *  MinMax: '<S21>/Max1'
   *  Product: '<S37>/Product'
   *  Product: '<S37>/Product1'
   *  Sum: '<S37>/Add'
   *  Sum: '<S37>/Subtract'
   */
  if (rtb_LDP_Active) {
    /* MinMax: '<S21>/Max' incorporates:
     *  Inport: '<Root>/Inport50'
     */
    rtb_Max_a = fmaxf(rtb_SAC_Trq_Derating_Factor,
                      Tdf_trq_derating_threshold_dp_par);
    rtb_Abs1_j = (1.0F - rtb_Sac_ts2) * rtb_Max_a + fmaxf(rtb_Max_a,
      Tdf_trq_der_thrs_dp_hi_sens_par) * rtb_Sac_ts2;
  } else {
    rtb_Abs1_j = rtb_SAC_Trq_Derating_Factor;
  }

  /* End of Switch: '<S43>/Switch' */

  /* Abs: '<S44>/Abs' incorporates:
   *  Abs: '<S148>/Abs2'
   *  Abs: '<S153>/Abs'
   *  Abs: '<S19>/Abs2'
   *  Abs: '<S267>/Abs1'
   */
  rtb_Max_a = fabsf(rtb_LAT_Kappa_Gradient_Fading_S);

  /* Product: '<S43>/Product' incorporates:
   *  Abs: '<S44>/Abs'
   *  Constant: '<S44>/Constant'
   *  Gain: '<S44>/Gain'
   *  Lookup_n-D: '<S44>/1-D Lookup Table'
   *  Lookup_n-D: '<S44>/1-D Lookup Table1'
   *  MinMax: '<S44>/Min'
   *  Product: '<S44>/Product'
   */
  rtb_SAC_Arbitrated_Angle_Cmd = look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(Tdf_derating_thrs_x_scheduling[0])),
    ((const real32_T *)&(Tdf_derating_thrs_y_scheduling[0])), 10U) *
    look1_iflf_binlxpw(fminf(Tdf_trq_derating_max_curv, rtb_Max_a), ((const
    real32_T *)&(Tdf_der_thrs_curv_x_scheduling[0])), ((const real32_T *)
    &(Tdf_der_thrs_curv_y_scheduling[0])), 4U) * rtb_Abs1_j;

  /* MinMax: '<S15>/Min' incorporates:
   *  Constant: '<S12>/Constant'
   *  Constant: '<S12>/Constant1'
   *  Constant: '<S15>/Constant'
   *  Constant: '<S15>/Constant1'
   *  Constant: '<S15>/Constant2'
   *  MinMax: '<S15>/Max'
   *  MinMax: '<S15>/Max1'
   *  Product: '<S15>/Divide'
   *  Sum: '<S15>/Add'
   *  Sum: '<S15>/Add1'
   */
  rtb_Abs1_j = fminf(fmaxf(LAT_Stiffness_Request_Factor_pre - Tdf_derating_start,
    0.0F) / fmaxf(Tdf_derating_end - Tdf_derating_start, 0.1F), 1.0F);

  /* Sum: '<S43>/Add' incorporates:
   *  Constant: '<S21>/Constant3'
   *  Constant: '<S43>/Constant'
   *  MinMax: '<S21>/Min'
   *  Product: '<S43>/Product1'
   *  Product: '<S43>/Product2'
   *  Sum: '<S43>/Subtract'
   */
  TDF_Trq_Derating_Threshold_Arb = (1.0F - rtb_Abs1_j) * fminf
    (Tdf_trq_derating_thrs_hi_sens, rtb_SAC_Trq_Derating_Factor) +
    rtb_SAC_Arbitrated_Angle_Cmd * rtb_Abs1_j;

  /* MinMax: '<S38>/Max1' incorporates:
   *  Constant: '<S36>/Constant'
   *  Constant: '<S38>/Constant'
   *  Gain: '<S36>/Gain'
   *  Lookup_n-D: '<S36>/1-D Lookup Table'
   *  MinMax: '<S36>/Min'
   *  MinMax: '<S38>/Max'
   *  Sum: '<S36>/Add'
   *  Sum: '<S38>/Subtract'
   *  UnitDelay: '<S38>/Unit Delay1'
   *  UnitDelay: '<S39>/Unit Delay'
   */
  TDF_Max_Steer_Torque = fmaxf(VEH_Abs_Steer_Torque_Comp_pre - fmaxf
    (look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed, ((const real32_T *)
    &(Tdf_trq_der_init_corr_x_schedul[0])), ((const real32_T *)
    &(Tdf_trq_der_init_corr_y_schedul[0])), 10U) + fminf
     (TDF_Steer_Torque_Sample_pre, Tdf_trq_derating_max_init_trq),
     TDF_Trq_Derating_Threshold_Arb), 0.0F);

  /* MinMax: '<S41>/Max' incorporates:
   *  Constant: '<S41>/Constant'
   *  Constant: '<S41>/Constant1'
   *  Constant: '<S41>/Constant2'
   *  Product: '<S41>/Product'
   *  Sum: '<S41>/Subtract'
   */
  TDF_Torque_Der_Factor_HF_Path = fmaxf(0.0F, 1.0F - TDF_Max_Steer_Torque *
    Tdf_trq_derating_slope_hf_path);

  /* Switch: '<S11>/Switch5' incorporates:
   *  UnitDelay: '<S11>/Unit Delay1'
   */
  fSteerAngle_deg = fDeltaFCmd_State;

  /* Sum: '<S11>/Subtract1' */
  SAC_Control_Error_Eps = fSteerAngle_deg - VEH_Delta_F_Oc;

  /* MultiPortSwitch: '<S29>/Multiport Switch' incorporates:
   *  ArithShift: '<S29>/Shift Arithmetic'
   *  Constant: '<S29>/Constant'
   *  Inport: '<Root>/Inport45'
   *  S-Function (sfix_bitop): '<S29>/Bitwise AND2'
   *  Sum: '<S29>/Add'
   */
  switch ((int32_T)(((uint32_T)(Tdf_derating_mode_par & 24) >> 3) + 1U)) {
   case 1:
    /* MultiPortSwitch: '<S29>/Multiport Switch' */
    TDF_Selected_Torque_Source = 0.0F;
    break;

   case 2:
    /* MultiPortSwitch: '<S29>/Multiport Switch' */
    TDF_Selected_Torque_Source = 0.0F;
    break;

   case 3:
    /* MultiPortSwitch: '<S29>/Multiport Switch' incorporates:
     *  Gain: '<S29>/Gain'
     */
    TDF_Selected_Torque_Source = -VEH_Steer_Torque_Comp;
    break;

   default:
    /* MultiPortSwitch: '<S29>/Multiport Switch' */
    TDF_Selected_Torque_Source = SAC_Control_Error_Eps;
    break;
  }

  /* End of MultiPortSwitch: '<S29>/Multiport Switch' */

  /* Sum: '<S33>/Add' incorporates:
   *  Constant: '<S33>/Constant'
   *  Inport: '<Root>/Inport13'
   *  Sum: '<S49>/Add1'
   */
  TDF_Torque_Request_Factor_sta_0 = VEH_cycletime + Tdf_trq_filter_coeff;

  /* Sum: '<S33>/Add1' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S33>/Divide'
   *  Product: '<S33>/Product'
   *  Sum: '<S33>/Add'
   *  Sum: '<S33>/Subtract'
   *  UnitDelay: '<S33>/Unit Delay'
   */
  TDF_Torque_Request_Factor_state = VEH_cycletime /
    TDF_Torque_Request_Factor_sta_0 * (TDF_Selected_Torque_Source -
    TDF_Torque_Request_Factor_state) + TDF_Torque_Request_Factor_state;

  /* MinMax: '<S30>/Max' incorporates:
   *  Constant: '<S30>/Constant1'
   *  Inport: '<Root>/Inport46'
   *  MinMax: '<S30>/Min'
   *  Product: '<S30>/Product'
   *  UnitDelay: '<S33>/Unit Delay'
   */
  TDF_Torque_Request_Factor = fmaxf(fminf(Tdf_torque_request_sign_slope_par *
    TDF_Torque_Request_Factor_state, 1.0F), -1.0F);

  /* RelationalOperator: '<S17>/LessThanOrEqual' incorporates:
   *  Constant: '<S17>/Constant4'
   *  Product: '<S17>/Product2'
   */
  TDF_Driver_Counter_Steering = (TDF_Torque_Request_Factor *
    TDF_Vehicle_Steer_Torque_Factor <= -0.5F);

  /* Switch: '<S12>/Switch' incorporates:
   *  Constant: '<S12>/Constant2'
   */
  if (TDF_Driver_Counter_Steering) {
    rtb_SAC_Trq_Derating_Factor = TDF_Torque_Der_Factor_HF_Path;
  } else {
    rtb_SAC_Trq_Derating_Factor = 1.0F;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Sum: '<S14>/Add' incorporates:
   *  Constant: '<S14>/Constant2'
   *  Constant: '<S14>/Constant3'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S14>/Divide'
   *  Product: '<S14>/Product'
   *  Sum: '<S14>/Add1'
   *  Sum: '<S14>/Subtract'
   *  Sum: '<S14>/Subtract1'
   *  Switch: '<S14>/Switch1'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  TDF_Torque_Der_Factor_HF_Path_state = 1.0F / (Tdf_torque_der_filt_coeff_hf +
    VEH_cycletime) * VEH_cycletime * ((rtb_SAC_Trq_Derating_Factor - 1.0F) -
    TDF_Torque_Der_Factor_HF_Path_state) + TDF_Torque_Der_Factor_HF_Path_state;

  /* Switch: '<S42>/Switch' incorporates:
   *  Inport: '<Root>/Inport52'
   *  Inport: '<Root>/Inport53'
   */
  if (rtb_LDP_Active) {
    rtb_Switch_iy_idx_0 = Tdf_torque_derating_slope_ldp_par;
  } else {
    rtb_Switch_iy_idx_0 = Tdf_torque_derating_slope_par;
  }

  /* End of Switch: '<S42>/Switch' */

  /* Sum: '<S42>/Add' incorporates:
   *  Constant: '<S42>/Constant2'
   *  Constant: '<S42>/Constant3'
   *  Gain: '<S42>/Gain'
   *  Inport: '<Root>/Inport53'
   *  Lookup_n-D: '<S42>/1-D Lookup Table'
   *  MinMax: '<S42>/Min'
   *  Product: '<S42>/Product'
   *  Product: '<S42>/Product1'
   *  Product: '<S42>/Product2'
   *  Sum: '<S42>/Subtract'
   */
  TDF_Torque_Derating_Slop_Arb = look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(Tdf_derating_slope_x_scheduling
    [0])), ((const real32_T *)&(Tdf_derating_slope_y_scheduling[0])), 10U) *
    rtb_Switch_iy_idx_0 * rtb_Abs1_j + (1.0F - rtb_Abs1_j) * fminf
    (Tdf_torque_derating_slope_par, Tdf_trq_derating_slope_hi_sens);

  /* MinMax: '<S40>/Max' incorporates:
   *  Constant: '<S40>/Constant1'
   *  Constant: '<S40>/Constant2'
   *  Product: '<S40>/Product'
   *  Sum: '<S40>/Subtract'
   */
  TDF_Torque_Derating_Factor = fmaxf(0.0F, 1.0F - TDF_Max_Steer_Torque *
    TDF_Torque_Derating_Slop_Arb);

  /* Switch: '<S12>/Switch1' incorporates:
   *  Constant: '<S12>/Constant3'
   */
  if (TDF_Driver_Counter_Steering) {
    rtb_SAC_Trq_Derating_Factor = TDF_Torque_Derating_Factor;
  } else {
    rtb_SAC_Trq_Derating_Factor = 1.0F;
  }

  /* End of Switch: '<S12>/Switch1' */

  /* Sum: '<S13>/Subtract' incorporates:
   *  Constant: '<S13>/Constant2'
   *  Sum: '<S13>/Subtract1'
   *  Switch: '<S13>/Switch1'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  rtb_Abs1_j = (rtb_SAC_Trq_Derating_Factor - 1.0F) -
    TDF_Torque_Derating_Factor_state;

  /* Switch: '<S13>/Switch' incorporates:
   *  Constant: '<S13>/Constant1'
   *  Constant: '<S13>/Constant6'
   *  Constant: '<S13>/Constant7'
   *  Gain: '<S13>/Gain'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S13>/Divide'
   *  RelationalOperator: '<S13>/GreaterThan'
   *  Sum: '<S13>/Add1'
   */
  if (rtb_Abs1_j <= -Tdf_out_scale_filt_min_residual) {
    rtb_SAT_Max_Delta_F_Cmd = 1.0F;
  } else {
    rtb_SAT_Max_Delta_F_Cmd = 1.0F / (Tdf_torque_derating_filt_coeff +
      VEH_cycletime) * VEH_cycletime;
  }

  /* End of Switch: '<S13>/Switch' */

  /* Sum: '<S13>/Add' incorporates:
   *  Product: '<S13>/Product'
   *  Switch: '<S13>/Switch1'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  TDF_Torque_Derating_Factor_state = rtb_Abs1_j * rtb_SAT_Max_Delta_F_Cmd +
    TDF_Torque_Derating_Factor_state;

  /* S-Function (sfix_bitop): '<S6>/Bitwise OR' incorporates:
   *  ArithShift: '<S6>/Shift Arithmetic'
   *  Constant: '<S6>/Constant11'
   *  Constant: '<S6>/Constant6'
   *  Inport: '<Root>/Inport35'
   */
  rtb_SAC_Controller_Mode = Sac_controller_mode_2 << 16U |
    Sac_controller_mode_1_par;

  /* Sum: '<S190>/Add1' incorporates:
   *  Constant: '<S190>/Constant1'
   *  Inport: '<Root>/Inport5'
   *  Product: '<S190>/Product'
   *  Sum: '<S190>/Subtract'
   *  UnitDelay: '<S190>/Unit Delay'
   */
  LAT_Yaw_Rate_Filter_state = (VEH_Yaw_Rate - LAT_Yaw_Rate_Filter_state) *
    LAT_Yaw_Rate_Filter_Coeff + LAT_Yaw_Rate_Filter_state;

  /* Product: '<S137>/Divide' incorporates:
   *  Constant: '<S137>/Constant'
   *  Inport: '<Root>/Inport13'
   *  Sum: '<S137>/Add'
   */
  rtb_Abs1_j = VEH_cycletime / (VEH_cycletime + Hec_yaw_rate_filter_coeff);

  /* Sum: '<S137>/Add1' incorporates:
   *  Product: '<S137>/Product'
   *  Sum: '<S137>/Subtract'
   *  UnitDelay: '<S137>/Unit Delay'
   *  UnitDelay: '<S190>/Unit Delay'
   */
  HEC_Yaw_Rate_Filter_state = (LAT_Yaw_Rate_Filter_state -
    HEC_Yaw_Rate_Filter_state) * rtb_Abs1_j + HEC_Yaw_Rate_Filter_state;

  /* S-Function (sfix_bitop): '<S139>/Bitwise AND' incorporates:
   *  S-Function (sfix_bitop): '<S138>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S92>/Bitwise AND1'
   *  Switch: '<S81>/Switch'
   */
  LAT_Oc_Kappa_Active_tmp = rtb_SAC_Controller_Mode & 7U;

  /* Switch: '<S139>/Switch' incorporates:
   *  Constant: '<S139>/Constant'
   *  Constant: '<S139>/Constant1'
   *  Constant: '<S142>/Constant'
   *  Gain: '<S139>/Gain'
   *  Lookup_n-D: '<S139>/1-D Lookup Table'
   *  Lookup_n-D: '<S139>/1-D Lookup Table1'
   *  Product: '<S139>/Product'
   *  Product: '<S139>/Product1'
   *  RelationalOperator: '<S142>/Compare'
   *  S-Function (sfix_bitop): '<S139>/Bitwise AND'
   */
  if (LAT_Oc_Kappa_Active_tmp == 4U) {
    rtb_Hec_r_factor = look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed, ((const
      real32_T *)&(Hec_r_x_scheduling_Sc[0])), ((const real32_T *)
      &(Hec_r_y_scheduling_Sc[0])), 10U) * Hec_r_factor_Sc;
  } else {
    rtb_Hec_r_factor = look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed, ((const
      real32_T *)&(Hec_r_x_scheduling[0])), ((const real32_T *)
      &(Hec_r_y_scheduling[0])), 10U) * Hec_r_factor;
  }

  /* End of Switch: '<S139>/Switch' */

  /* Sum: '<S140>/Add1' incorporates:
   *  Constant: '<S140>/Constant'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S136>/Divide'
   *  Product: '<S140>/Divide'
   *  Product: '<S140>/Product'
   *  Sum: '<S136>/Subtract'
   *  Sum: '<S140>/Add'
   *  Sum: '<S140>/Subtract'
   *  UnitDelay: '<S136>/Unit Delay'
   *  UnitDelay: '<S140>/Unit Delay'
   *  UnitDelay: '<S190>/Unit Delay'
   */
  VEH_Yaw_Rate_Dot_sate = ((LAT_Yaw_Rate_Filter_state - VEH_Yaw_Rate_state) /
    VEH_cycletime - VEH_Yaw_Rate_Dot_sate) * (VEH_cycletime / (VEH_cycletime +
    Lat_yaw_rate_dot_filter_coeff)) + VEH_Yaw_Rate_Dot_sate;

  /* S-Function (sfix_bitop): '<S81>/Bitwise AND' incorporates:
   *  S-Function (sfix_bitop): '<S80>/Bitwise Operator'
   */
  rtb_Vehicle_Speed_Index = rtb_SAC_Controller_Mode & 2048U;

  /* Switch: '<S81>/Switch' incorporates:
   *  Constant: '<S135>/Constant'
   *  Constant: '<S141>/Constant'
   *  RelationalOperator: '<S135>/Compare'
   *  RelationalOperator: '<S141>/Compare'
   *  S-Function (sfix_bitop): '<S81>/Bitwise AND'
   *  Switch: '<S138>/Switch'
   */
  if (rtb_Vehicle_Speed_Index == 2048U) {
    /* Switch: '<S81>/Switch' incorporates:
     *  Constant: '<S81>/Constant1'
     */
    SAC_Angle_Command_Yawrate_Fback = 0.0F;
  } else {
    if (LAT_Oc_Kappa_Active_tmp == 4U) {
      /* Switch: '<S138>/Switch' incorporates:
       *  Constant: '<S138>/Constant1'
       *  Gain: '<S138>/Gain'
       *  Lookup_n-D: '<S138>/1-D Lookup Table1'
       *  Product: '<S138>/Product1'
       */
      rtb_SAC_Trq_Derating_Factor = look1_iflf_binlxpw(3.6F *
        rtb_VEH_Vehicle_Speed, ((const real32_T *)
        &(Hec_r_dot_factor_x_scheduling_Sc[0])), ((const real32_T *)
        &(Hec_r_dot_factor_y_scheduling_Sc[0])), 10U) * Hec_r_dot_factor_Sc;
    } else {
      /* Switch: '<S138>/Switch' incorporates:
       *  Constant: '<S138>/Constant'
       *  Gain: '<S138>/Gain'
       *  Lookup_n-D: '<S138>/1-D Lookup Table'
       *  Product: '<S138>/Product'
       */
      rtb_SAC_Trq_Derating_Factor = look1_iflf_binlxpw(3.6F *
        rtb_VEH_Vehicle_Speed, ((const real32_T *)
        &(Hec_r_dot_factor_x_scheduling[0])), ((const real32_T *)
        &(Hec_r_dot_factor_y_scheduling[0])), 10U) * Hec_r_dot_factor;
    }

    /* Switch: '<S81>/Switch' incorporates:
     *  Constant: '<S81>/Constant'
     *  Product: '<S81>/Product'
     *  Product: '<S81>/Product1'
     *  Product: '<S81>/Product2'
     *  Sum: '<S81>/Add1'
     *  Sum: '<S81>/Add2'
     *  UnitDelay: '<S137>/Unit Delay'
     *  UnitDelay: '<S140>/Unit Delay'
     *  UnitDelay: '<S190>/Unit Delay'
     */
    SAC_Angle_Command_Yawrate_Fback = (HEC_Yaw_Rate_Filter_state *
      Hec_r_pt1_factor + rtb_Hec_r_factor * LAT_Yaw_Rate_Filter_state) +
      VEH_Yaw_Rate_Dot_sate * rtb_SAC_Trq_Derating_Factor;
  }

  /* Gain: '<S108>/Gain' */
  rtb_Gain_d = 3.6F * rtb_VEH_Vehicle_Speed;

  /* Switch: '<S134>/Switch2' incorporates:
   *  Constant: '<S134>/Constant1'
   *  Constant: '<S134>/Constant2'
   *  Gain: '<S108>/Gain'
   *  Gain: '<S134>/Gain1'
   *  Inport: '<Root>/Inport21'
   *  Lookup_n-D: '<S114>/1-D Lookup Table3'
   *  Lookup_n-D: '<S134>/SAC_Yrc_Ctrl_Cmd_Fading_Sched'
   *  MinMax: '<S134>/MinMax'
   *  Product: '<S134>/Divide'
   *  Product: '<S134>/Product'
   *  S-Function (sfix_bitop): '<S134>/Bitwise Operator2'
   */
  if ((Dmc_configuration_mode_par & 256U) != 0U) {
    rtb_SAC_Arbitrated_Angle_Cmd = rtb_LAT_Delta_F_cmd / fmaxf
      (look1_iflf_binlxpw(rtb_Gain_d, ((const real32_T *)
         &(Dyc_kappa_angle_t_x_schedul_gen[0])), ((const real32_T *)
         &(Dyc_kappa_angle_t_y_schedul_nom[0])), 11U), 0.1F) *
      look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed, ((const real32_T *)
      &(Sac_yrc_ctrl_cmd_fading_x_sched[0])), ((const real32_T *)
      &(Sac_yrc_ctrl_cmd_fading_y_sched[0])), 10U);
  } else {
    rtb_SAC_Arbitrated_Angle_Cmd = 0.0F;
  }

  /* End of Switch: '<S134>/Switch2' */

  /* Sum: '<S80>/Add' incorporates:
   *  Product: '<S80>/Product'
   *  Sum: '<S134>/Add3'
   *  UnitDelay: '<S190>/Unit Delay'
   */
  SAC_Yrc_Control_Error = (rtb_SAC_Arbitrated_Angle_Cmd +
    rtb_LAT_Kappa_Gradient_Fading_S) * rtb_VEH_Vehicle_Speed -
    LAT_Yaw_Rate_Filter_state;

  /* Switch: '<S132>/Switch1' incorporates:
   *  Constant: '<S132>/Constant'
   *  Constant: '<S132>/Constant1'
   *  Constant: '<S132>/Sac_yrc_pt1_filter_coeff'
   *  Product: '<S132>/Divide'
   *  Sum: '<S132>/Add2'
   *  UnitDelay: '<S132>/Unit Delay2'
   */
  if (SAC_Disable_pre_pre) {
    rtb_SAC_Arbitrated_Angle_Cmd = 1.0F;
  } else {
    rtb_SAC_Arbitrated_Angle_Cmd = Sac_ts / (Sac_ts + Sac_yrc_pt1_filter_coeff);
  }

  /* End of Switch: '<S132>/Switch1' */

  /* Sum: '<S132>/Add' incorporates:
   *  Constant: '<S80>/Constant2'
   *  Gain: '<S80>/Gain3'
   *  Lookup_n-D: '<S80>/SAC_Yrc_Kp_Pt1_Scheduling'
   *  Product: '<S132>/Product'
   *  Product: '<S80>/Product1'
   *  Product: '<S80>/Product4'
   *  Sum: '<S132>/Add1'
   *  UnitDelay: '<S132>/Unit Delay'
   */
  LAT_Eps_Torque_Filter_state = (look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed,
    ((const real32_T *)&(Sac_yrc_kp_pt1_x_scheduling[0])), ((const real32_T *)
    &(Sac_yrc_kp_pt1_y_scheduling[0])), 10U) * Sac_yrc_kp_pt1_gain *
    SAC_Yrc_Control_Error - LAT_Eps_Torque_Filter_state) *
    rtb_SAC_Arbitrated_Angle_Cmd + LAT_Eps_Torque_Filter_state;

  /* Switch: '<S80>/Switch4' incorporates:
   *  Constant: '<S131>/Constant'
   *  RelationalOperator: '<S131>/Compare'
   */
  if (rtb_Vehicle_Speed_Index == 2048U) {
    /* Switch: '<S80>/Switch4' incorporates:
     *  Constant: '<S80>/Constant'
     */
    SAC_Yrc_Angle_Command = 0.0F;
  } else {
    /* Switch: '<S80>/Switch4' incorporates:
     *  Constant: '<S80>/Constant1'
     *  Gain: '<S80>/Gain1'
     *  Gain: '<S80>/Sac_yrc_loop_gain_corr'
     *  Lookup_n-D: '<S80>/SAC_Yrc_Kp_Scheduling'
     *  Product: '<S80>/Product2'
     *  Product: '<S80>/Product3'
     *  Sum: '<S80>/Add2'
     *  UnitDelay: '<S132>/Unit Delay'
     *  UnitDelay: '<S80>/Unit Delay1'
     */
    SAC_Yrc_Angle_Command = ((look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed,
      ((const real32_T *)&(Sac_yrc_kp_x_scheduling[0])), ((const real32_T *)
      &(Sac_yrc_kp_y_scheduling[0])), 10U) * Sac_yrc_kp_gain *
      SAC_Yrc_Control_Error + LAT_Eps_Torque_Filter_state) +
      SAC_Integrator_Sat_Out) * Sac_yrc_loop_gain_corr;
  }

  /* End of Switch: '<S80>/Switch4' */

  /* Switch: '<S77>/Switch' incorporates:
   *  Constant: '<S83>/Constant'
   *  RelationalOperator: '<S83>/Compare'
   *  S-Function (sfix_bitop): '<S77>/Bitwise AND'
   */
  if ((rtb_SAC_Controller_Mode & 128U) == 128U) {
    /* Switch: '<S77>/Switch' incorporates:
     *  Sum: '<S77>/Add'
     */
    SAC_Arbitrated_Angle_Cmd = SAC_Yrc_Angle_Command -
      SAC_Angle_Command_Yawrate_Fback;
  } else {
    /* Switch: '<S77>/Switch' incorporates:
     *  Sum: '<S77>/Add1'
     */
    SAC_Arbitrated_Angle_Cmd = rtb_LAT_Delta_F_cmd;
  }

  /* End of Switch: '<S77>/Switch' */

  /* Product: '<S66>/Product' incorporates:
   *  Gain: '<S66>/Gain'
   *  Inport: '<Root>/Inport36'
   *  Inport: '<Root>/Inport37'
   *  Inport: '<Root>/Inport38'
   *  Lookup_n-D: '<S66>/1-D Lookup Table'
   */
  rtb_SAT_Max_Delta_F_Cmd = look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed,
    Sat_max_cmd_factor_x_scheduling_par, Sat_max_cmd_factor_y_scheduling_par,
    12U) * Sat_max_delta_f_cmd_par;

  /* Sum: '<S187>/Add' incorporates:
   *  Constant: '<S187>/Constant7'
   *  MinMax: '<S187>/Min'
   *  Switch: '<S187>/Switch2'
   *  UnitDelay: '<S187>/Unit Delay2'
   */
  Lat_ldp_startup_time_state = fminf(Lat_ldp_startup_time_state, 5.0F);

  /* Gain: '<S64>/Gain' incorporates:
   *  Switch: '<S76>/Switch'
   */
  rtb_SAC_Trq_Derating_Factor = 3.6F * rtb_VEH_Vehicle_Speed;

  /* Lookup_n-D: '<S64>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/Inport42'
   *  Inport: '<Root>/Inport43'
   *  MinMax: '<S64>/Max1'
   *  Switch: '<S76>/Switch'
   */
  rtb_SAC_Arbitrated_Angle_Cmd = look1_iflf_binlxpw(rtb_SAC_Trq_Derating_Factor,
    Sat_max_angle_low_bound_x_sched_par, Sat_max_angle_low_bound_y_sched_par, 2U);

  /* Lookup_n-D: '<S64>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Inport40'
   *  Inport: '<Root>/Inport41'
   *  MinMax: '<S64>/Max1'
   *  Switch: '<S76>/Switch'
   */
  rtb_SAC_Trq_Derating_Factor = look1_iflf_binlxpw(rtb_SAC_Trq_Derating_Factor,
    Sat_max_angle_offset_x_par, Sat_max_angle_offset_y_par, 3U);

  /* Sum: '<S64>/Add1' incorporates:
   *  Abs: '<S64>/Abs'
   *  Constant: '<S64>/Constant'
   *  Constant: '<S64>/Constant1'
   *  Gain: '<S71>/Gain'
   *  Inport: '<Root>/Inport39'
   *  MinMax: '<S64>/Max'
   *  Product: '<S64>/Divide'
   *  Product: '<S64>/Product'
   *  Product: '<S64>/Product1'
   *  Sum: '<S64>/Add'
   *  Switch: '<S76>/Switch'
   *  UnitDelay: '<S190>/Unit Delay'
   */
  rtb_SAC_Trq_Derating_Factor += fabsf(LAT_Yaw_Rate_Filter_state *
    VEH_Vehicle_Wheel_Base / fmaxf(rtb_VEH_Vehicle_Speed, 1.38888884F) +
    LAT_Yaw_Rate_Filter_state * rtb_VEH_Vehicle_Speed *
    VEH_Vehicle_Selfsteering_Factor_par) * 57.2957802F;

  /* MinMax: '<S64>/Max1' incorporates:
   *  Switch: '<S76>/Switch'
   */
  rtb_SAC_Trq_Derating_Factor = fmaxf(rtb_SAC_Trq_Derating_Factor,
    rtb_SAC_Arbitrated_Angle_Cmd);

  /* Product: '<S65>/Product' incorporates:
   *  Product: '<S108>/Product2'
   *  Product: '<S115>/Product'
   *  Product: '<S19>/Product'
   *  Product: '<S225>/Product1'
   *  Switch: '<S108>/Switch'
   *  Switch: '<S115>/Switch'
   *  Switch: '<S76>/Switch'
   */
  rtb_SAC_Arbitrated_Angle_Cmd = rtb_VEH_Vehicle_Speed * rtb_VEH_Vehicle_Speed;

  /* Switch: '<S76>/Switch' incorporates:
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S65>/Constant'
   *  Constant: '<S65>/Constant1'
   *  Gain: '<S72>/Gain'
   *  Inport: '<Root>/Inport34'
   *  Inport: '<Root>/Inport39'
   *  MinMax: '<S65>/Max'
   *  MinMax: '<S76>/Min'
   *  MinMax: '<S76>/Min1'
   *  Product: '<S65>/Divide'
   *  Product: '<S65>/Product'
   *  Product: '<S65>/Product1'
   *  Product: '<S65>/Product2'
   *  Sum: '<S65>/Add'
   */
  rtb_SAT_Max_Steer_Angle_Cmd_Sel = fminf(fminf((rtb_SAC_Arbitrated_Angle_Cmd *
    VEH_Vehicle_Selfsteering_Factor_par + VEH_Vehicle_Wheel_Base) * 57.2957802F *
    (Lat_max_ay_par / fmaxf(rtb_SAC_Arbitrated_Angle_Cmd, 0.1F)),
    rtb_SAC_Trq_Derating_Factor), 100.0F);

  /* MinMax: '<S74>/Min' incorporates:
   *  Constant: '<S74>/Constant'
   *  Product: '<S74>/Product'
   *  Sum: '<S62>/Add'
   *  Sum: '<S74>/Subtract'
   *  UnitDelay: '<S62>/Unit Delay'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  SAT_Req_Dyn_Steer_Angle_Max = fminf(rtb_SAT_Max_Delta_F_Cmd,
    Sat_dynamic_enhancement_factor * rtb_SAT_Max_Steer_Angle_Cmd_Sel -
    (LAT_Sat_Dynamic_Threshold_state + LAT_Sat_Dynamic_Threshold_int));

  /* MinMax: '<S63>/Max' incorporates:
   *  Gain: '<S63>/Gain'
   *  MinMax: '<S63>/Min'
   */
  rtb_LAT_Saturated_Delta_F_Cmd = fmaxf(fminf(SAC_Arbitrated_Angle_Cmd,
    SAT_Req_Dyn_Steer_Angle_Max), -SAT_Req_Dyn_Steer_Angle_Max);

  /* Switch: '<S67>/Switch' incorporates:
   *  Constant: '<S73>/Constant'
   *  RelationalOperator: '<S73>/Compare'
   *  S-Function (sfix_bitop): '<S67>/Bitwise AND'
   */
  if ((rtb_SAC_Controller_Mode & 512U) == 512U) {
    /* Switch: '<S67>/Switch' */
    SAT_Saturated_Angle_Command = SAC_Arbitrated_Angle_Cmd;
  } else {
    /* Switch: '<S67>/Switch' */
    SAT_Saturated_Angle_Command = rtb_LAT_Saturated_Delta_F_Cmd;
  }

  /* End of Switch: '<S67>/Switch' */

  /* Sum: '<S4>/Subtract' */
  SAC_Control_Error = SAT_Saturated_Angle_Command - VEH_Delta_F_Oc;

  /* Sum: '<S47>/Subtract' incorporates:
   *  UnitDelay: '<S47>/Unit Delay'
   */
  rtb_Subtract_jc = SAT_Saturated_Angle_Command -
    TDF_Saturated_Angle_Command_Error_state;

  /* Sum: '<S49>/Add' incorporates:
   *  Abs: '<S46>/Abs'
   *  Abs: '<S46>/Abs1'
   *  Constant: '<S46>/Constant'
   *  Constant: '<S46>/Constant1'
   *  Constant: '<S46>/Constant2'
   *  Constant: '<S46>/Constant3'
   *  Inport: '<Root>/Inport13'
   *  MinMax: '<S46>/Max'
   *  MinMax: '<S46>/Min'
   *  Product: '<S46>/Product'
   *  Product: '<S49>/Divide'
   *  Product: '<S49>/Product'
   *  Sum: '<S46>/Subtract'
   *  Sum: '<S46>/Subtract1'
   *  Sum: '<S49>/Subtract'
   *  UnitDelay: '<S49>/Unit Delay'
   */
  TDF_Control_Error_Factor_state = (fminf(1.0F, fmaxf(0.0F, (fabsf
    (SAC_Control_Error) - fabsf(rtb_Subtract_jc)) - Tdf_control_error_threshold)
    * Tdf_control_err_derating_slope) - TDF_Control_Error_Factor_state) * (1.0F /
    TDF_Torque_Request_Factor_sta_0 * VEH_cycletime) +
    TDF_Control_Error_Factor_state;

  /* Sum: '<S182>/Add1' incorporates:
   *  Constant: '<S182>/Constant1'
   *  Inport: '<Root>/Inport13'
   *  Inport: '<Root>/VEH_Delta_F_Dot'
   *  Product: '<S182>/Divide'
   *  Product: '<S182>/Product'
   *  Sum: '<S182>/Add'
   *  Sum: '<S182>/Subtract'
   *  UnitDelay: '<S182>/Unit Delay'
   */
  LAT_Delta_F_Dot_Filter_state = VEH_cycletime / (VEH_cycletime +
    Lat_delta_f_dot_filter_coeff) * (VEH_Delta_F_Dot -
    LAT_Delta_F_Dot_Filter_state) + LAT_Delta_F_Dot_Filter_state;

  /* Abs: '<S48>/Abs' incorporates:
   *  Abs: '<S19>/Abs3'
   *  Abs: '<S222>/Abs'
   *  UnitDelay: '<S182>/Unit Delay'
   */
  TDF_Torque_Request_Factor_sta_0 = fabsf(LAT_Delta_F_Dot_Filter_state);

  /* Sum: '<S45>/Add' incorporates:
   *  Abs: '<S48>/Abs'
   *  Constant: '<S22>/Constant'
   *  Constant: '<S46>/Constant4'
   *  Constant: '<S48>/Constant'
   *  Constant: '<S48>/Constant1'
   *  Constant: '<S48>/Constant2'
   *  Constant: '<S48>/Constant3'
   *  Constant: '<S48>/Constant4'
   *  MinMax: '<S22>/Max'
   *  MinMax: '<S48>/Max'
   *  MinMax: '<S48>/Max1'
   *  Product: '<S45>/Product'
   *  Product: '<S48>/Product'
   *  Sum: '<S22>/Subtract'
   *  Sum: '<S45>/Subtract'
   *  Sum: '<S46>/Subtract2'
   *  Sum: '<S48>/Subtract'
   *  Sum: '<S48>/Subtract1'
   *  UnitDelay: '<S45>/Unit Delay'
   *  UnitDelay: '<S49>/Unit Delay'
   */
  LAT_Velocity_Derating_Factor_state = ((fmaxf(1.0F -
    TDF_Control_Error_Factor_state, fmaxf(0.0F, 1.0F - fmaxf
    (TDF_Torque_Request_Factor_sta_0 - Tdf_velocity_derating_threshold, 0.0F) *
    Tdf_velocity_derating_slope)) - 1.0F) - LAT_Velocity_Derating_Factor_state) *
    rtb_Product_lg + LAT_Velocity_Derating_Factor_state;

  /* Switch: '<S16>/Switch1' incorporates:
   *  Inport: '<Root>/Inport45'
   *  S-Function (sfix_bitop): '<S16>/Bitwise AND'
   */
  if ((Tdf_derating_mode_par & 4U) != 0U) {
    /* Switch: '<S16>/Switch1' incorporates:
     *  Constant: '<S13>/Constant5'
     *  Constant: '<S14>/Constant5'
     *  Constant: '<S22>/Constant'
     *  Product: '<S16>/Product'
     *  Product: '<S16>/Product1'
     *  Sum: '<S13>/Add2'
     *  Sum: '<S14>/Add2'
     *  Sum: '<S22>/Add'
     *  UnitDelay: '<S13>/Unit Delay'
     *  UnitDelay: '<S14>/Unit Delay'
     *  UnitDelay: '<S45>/Unit Delay'
     */
    TDF_Composite_Derating_Factor = (TDF_Torque_Der_Factor_HF_Path_state + 1.0F)
      * (TDF_Torque_Derating_Factor_state + 1.0F) *
      (LAT_Velocity_Derating_Factor_state + 1.0F);
  } else {
    /* Switch: '<S16>/Switch1' incorporates:
     *  Constant: '<S16>/Constant13'
     */
    TDF_Composite_Derating_Factor = 1.0F;
  }

  /* End of Switch: '<S16>/Switch1' */

  /* MinMax: '<S54>/Max' incorporates:
   *  Constant: '<S54>/Constant3'
   *  Constant: '<S54>/Constant4'
   *  MinMax: '<S54>/Min'
   *  UnitDelay: '<S54>/Unit Delay'
   */
  rtb_TDF_State_Derating_Factor_C = fmaxf(0.0F, fminf(1.0F,
    TDF_State_Derating_Factor_Comp_state));

  /* Switch: '<S24>/Switch' incorporates:
   *  Constant: '<S24>/Constant'
   *  Constant: '<S53>/Constant'
   *  Constant: '<S56>/Constant'
   *  Constant: '<S56>/Constant1'
   *  Inport: '<Root>/Inport45'
   *  MinMax: '<S56>/Max'
   *  MinMax: '<S56>/Min'
   *  RelationalOperator: '<S53>/Compare'
   *  S-Function (sfix_bitop): '<S24>/Bitwise AND2'
   */
  if ((Tdf_derating_mode_par & 1) == 1) {
    rtb_SAC_Trq_Derating_Factor = fmaxf(fminf(TDF_Max_Derating_Factor,
      rtb_TDF_State_Derating_Factor_C), 0.0F);
  } else {
    rtb_SAC_Trq_Derating_Factor = 0.0F;
  }

  /* End of Switch: '<S24>/Switch' */

  /* Sum: '<S24>/Subtract' incorporates:
   *  Constant: '<S24>/Constant1'
   */
  TDF_Selected_State_Derating_Factor = 1.0F - rtb_SAC_Trq_Derating_Factor;

  /* Switch: '<S19>/Switch1' incorporates:
   *  Inport: '<Root>/Inport45'
   *  S-Function (sfix_bitop): '<S19>/Bitwise AND'
   */
  if ((Tdf_derating_mode_par & 2U) != 0U) {
    /* Switch: '<S19>/Switch1' incorporates:
     *  Constant: '<S19>/Constant10'
     *  MinMax: '<S35>/Min'
     *  RelationalOperator: '<S35>/LessThanOrEqual'
     *  UnitDelay: '<S35>/Unit Delay'
     */
    TDF_Idle_Derating_Factor = (real32_T)(fminf(TDF_Idle_Enable_Condition_state,
      Tdf_idle_enable_min_latency) < Tdf_idle_enable_min_latency);
  } else {
    /* Switch: '<S19>/Switch1' incorporates:
     *  Constant: '<S19>/Constant13'
     */
    TDF_Idle_Derating_Factor = 1.0F;
  }

  /* End of Switch: '<S19>/Switch1' */

  /* Switch: '<S12>/Switch3' incorporates:
   *  Inport: '<Root>/Inport45'
   *  S-Function (sfix_bitop): '<S12>/Bitwise AND'
   */
  if ((Tdf_derating_mode_par & 32U) != 0U) {
    /* Switch: '<S12>/Switch3' incorporates:
     *  Constant: '<S12>/Constant13'
     */
    SAC_Trq_Derating_Factor = 0.0F;
  } else {
    /* Switch: '<S12>/Switch3' incorporates:
     *  Product: '<S12>/Product'
     */
    SAC_Trq_Derating_Factor = TDF_Composite_Derating_Factor *
      TDF_Selected_State_Derating_Factor * TDF_Idle_Derating_Factor;
  }

  /* End of Switch: '<S12>/Switch3' */

  /* Switch: '<S9>/Switch' incorporates:
   *  Constant: '<S1>/Constant4'
   *  Constant: '<S9>/Constant'
   *  Inport: '<Root>/Inport21'
   *  MinMax: '<S9>/Min'
   *  S-Function (sfix_bitop): '<S9>/Bitwise AND'
   */
  if ((Dmc_configuration_mode_par & 1024U) != 0U) {
    rtb_SAC_Trq_Derating_Factor = fminf(SAC_Trq_Derating_Factor, 1.0F);
  } else {
    rtb_SAC_Trq_Derating_Factor = 1.0F;
  }

  /* End of Switch: '<S9>/Switch' */

  /* MinMax: '<S9>/Min1' incorporates:
   *  Gain: '<S9>/Gain'
   *  Inport: '<Root>/Inport4'
   */
  rtb_SAC_Trq_Derating_Factor = fminf(0.01F * LaKMC_angle_req_max_limit_scale,
    rtb_SAC_Trq_Derating_Factor);

  /* Sum: '<S280>/Add' incorporates:
   *  Constant: '<S280>/Constant'
   *  Constant: '<S280>/Constant1'
   *  MinMax: '<S280>/Max'
   *  Product: '<S280>/Product'
   *  Product: '<S280>/Product1'
   *  Sum: '<S280>/Subtract'
   */
  rtb_Switch_iy_idx_0 = (1.0F - rtb_SAC_Trq_Derating_Factor) * fmaxf
    (Sac_delta_f_counter_steer_grad, rtb_Product4_b) + rtb_Product4_b *
    rtb_SAC_Trq_Derating_Factor;

  /* Sum: '<S9>/Add' incorporates:
   *  Constant: '<S9>/Constant1'
   *  Product: '<S9>/Product'
   *  Product: '<S9>/Product1'
   *  Sum: '<S9>/Subtract'
   */
  SAC_Derated_Angle_Command = (1.0F - rtb_SAC_Trq_Derating_Factor) *
    VEH_Delta_F_Oc + SAT_Saturated_Angle_Command * rtb_SAC_Trq_Derating_Factor;

  /* Sum: '<S10>/Subtract' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  rtb_Product4_b = SAC_Derated_Angle_Command -
    SAC_Angle_Command_Rate_Limiter_state;

  /* Abs: '<S10>/Abs' */
  rtb_SAC_Trq_Derating_Factor = fabsf(rtb_Product4_b);

  /* Product: '<S10>/Product' incorporates:
   *  Inport: '<Root>/Inport13'
   */
  rtb_Product_lg = rtb_Switch_iy_idx_0 * VEH_cycletime;

  /* Signum: '<S10>/Sign' */
  if (rtb_Product4_b < 0.0F) {
    rtb_Product4_b = -1.0F;
  } else {
    if (rtb_Product4_b > 0.0F) {
      rtb_Product4_b = 1.0F;
    }
  }

  /* End of Signum: '<S10>/Sign' */

  /* Logic: '<S187>/NOT5' */
  rtb_SAC_Disable = !rtb_LAT_Enable_Lateral_Control;

  /* Switch: '<S10>/Switch' incorporates:
   *  Abs: '<S10>/Abs1'
   *  MinMax: '<S10>/Min1'
   *  Product: '<S10>/Product1'
   *  Product: '<S10>/Product2'
   *  Sum: '<S10>/Add'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  if (rtb_SAC_Disable) {
    SAC_Angle_Command_Rate_Limiter_state = VEH_Delta_F_Oc;
  } else {
    SAC_Angle_Command_Rate_Limiter_state = fminf(fabsf(rtb_Product_lg *
      rtb_Product4_b), rtb_SAC_Trq_Derating_Factor) * rtb_Product4_b +
      SAC_Angle_Command_Rate_Limiter_state;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Sum: '<S10>/Add1' incorporates:
   *  Constant: '<S10>/Constant'
   *  Constant: '<S10>/Constant1'
   *  MinMax: '<S10>/Max'
   *  MinMax: '<S10>/Min'
   *  Product: '<S10>/Product3'
   *  Product: '<S10>/Product4'
   *  Sum: '<S10>/Subtract1'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  SAC_Rate_Lim_Angle_Command = fminf(Sac_parity_time_const_barrier *
    rtb_Switch_iy_idx_0, fmaxf(0.0F, rtb_SAC_Trq_Derating_Factor -
    rtb_Product_lg)) * rtb_Product4_b + SAC_Angle_Command_Rate_Limiter_state;

  /* Switch: '<S11>/Switch5' incorporates:
   *  Inport: '<Root>/Inport21'
   *  S-Function (sfix_bitop): '<S11>/Bitwise AND3'
   *  Switch: '<S11>/Switch2'
   */
  if (rtb_LDP_Active) {
    /* Switch: '<S11>/Switch5' */
    fSteerAngle_deg = rtb_LAT_Delta_F_cmd;
  } else if ((Dmc_configuration_mode_par & 64U) != 0U) {
    /* Switch: '<S11>/Switch2' incorporates:
     *  Switch: '<S11>/Switch5'
     */
    fSteerAngle_deg = SAC_Rate_Lim_Angle_Command;
  } else {
    /* Switch: '<S11>/Switch5' incorporates:
     *  Switch: '<S11>/Switch2'
     */
    fSteerAngle_deg = rtb_LAT_Delta_F_cmd;
  }

  /* End of Switch: '<S11>/Switch5' */

  /* Product: '<S11>/Product1' incorporates:
   *  Inport: '<Root>/Inport15'
   */
  fSteerWhlAngle_deg = fSteerAngle_deg * EPS_Gear_Ratio;

  /* Switch: '<S286>/Switch' incorporates:
   *  Inport: '<Root>/Inport60'
   *  Inport: '<Root>/Inport62'
   *  Switch: '<S286>/Switch1'
   */
  if (ADAS_Lon_Acc_Enable_flag) {
    /* Switch: '<S286>/Switch' incorporates:
     *  Constant: '<S286>/Constant'
     */
    uiLonAccRequest_nu = 1U;
  } else if (NOP_Lon_Acc_Enable_flag) {
    /* Switch: '<S286>/Switch1' incorporates:
     *  Constant: '<S286>/Constant1'
     *  Switch: '<S286>/Switch'
     */
    uiLonAccRequest_nu = 2U;
  } else {
    /* Switch: '<S286>/Switch' incorporates:
     *  Constant: '<S286>/Constant3'
     *  Switch: '<S286>/Switch1'
     */
    uiLonAccRequest_nu = 0U;
  }

  /* End of Switch: '<S286>/Switch' */

  /* If: '<S287>/judgement_If&Else' */
  if (uiLonAccRequest_nu == 1) {
    /* Outputs for IfAction SubSystem: '<S287>/If' incorporates:
     *  ActionPort: '<S291>/Action Port'
     */
    /* Merge: '<S287>/Merge' incorporates:
     *  Inport: '<Root>/Inport54'
     *  Inport: '<S291>/Inport'
     */
    fLonAccCmd = ADAS_Lon_Acc_Cmd;

    /* End of Outputs for SubSystem: '<S287>/If' */
  } else if (uiLonAccRequest_nu == 2) {
    /* Outputs for IfAction SubSystem: '<S287>/If1' incorporates:
     *  ActionPort: '<S292>/Action Port'
     */
    /* Merge: '<S287>/Merge' incorporates:
     *  Inport: '<Root>/Inport14'
     *  Inport: '<S292>/Inport'
     */
    fLonAccCmd = NOP_Lon_Acc_Cmd;

    /* End of Outputs for SubSystem: '<S287>/If1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S287>/Else' incorporates:
     *  ActionPort: '<S290>/Action Port'
     */
    /* Merge: '<S287>/Merge' incorporates:
     *  Constant: '<S287>/Constant'
     *  Inport: '<S290>/Inport'
     */
    fLonAccCmd = 0.0F;

    /* End of Outputs for SubSystem: '<S287>/Else' */
  }

  /* End of If: '<S287>/judgement_If&Else' */

  /* UnitDelay: '<S103>/Unit Delay' */
  rtb_Product_lg = DMC_Integtrator_Output1;

  /* Product: '<S86>/Product' incorporates:
   *  Gain: '<S86>/Gain'
   *  Inport: '<Root>/Inport25'
   *  Lookup_n-D: '<S86>/1-D Lookup Table'
   */
  rtb_Switch_iy_idx_0 = look1_iflf_binlxpw(3.6F * rtb_VEH_Vehicle_Speed, ((
    const real32_T *)&(Dyc_time_constant_x_scheduling[0])), ((const real32_T *)
    &(Dyc_time_constant_y_scheduling[0])), 11U) *
    Dyc_state_filter_time_constant_par;

  /* MinMax: '<S98>/Max' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_SAC_Trq_Derating_Factor = fmaxf(rtb_Switch_iy_idx_0, 0.006F);

  /* Product: '<S98>/Product' */
  rtb_Product4_b = rtb_SAC_Trq_Derating_Factor * rtb_SAC_Trq_Derating_Factor;

  /* Sum: '<S98>/Add' incorporates:
   *  Constant: '<S98>/Constant1'
   *  Constant: '<S98>/Constant2'
   *  Constant: '<S98>/Constant3'
   *  Product: '<S98>/Divide'
   *  Product: '<S98>/Divide1'
   *  Product: '<S98>/Divide2'
   *  Product: '<S98>/Product1'
   *  Product: '<S98>/Product2'
   *  Product: '<S98>/Product3'
   *  UnitDelay: '<S103>/Unit Delay'
   *  UnitDelay: '<S104>/Unit Delay'
   */
  rtb_Add_hp5 = (1.0F / rtb_Product4_b * rtb_LAT_Kappa_Gradient_Fading_S - 1.0F /
                 rtb_Product4_b * DMC_Integtrator_Output) - 2.0F /
    rtb_SAC_Trq_Derating_Factor * DMC_Integtrator_Output1;

  /* UnitDelay: '<S106>/Unit Delay' */
  rtb_DMC_Integtrator_Output_ks = DMC_Integtrator_Output3;

  /* UnitDelay: '<S105>/Unit Delay' */
  rtb_DMC_Integtrator_Output_b = DMC_Integtrator_Output4;

  /* MinMax: '<S99>/Max' incorporates:
   *  Constant: '<S99>/Constant'
   */
  rtb_Switch_iy_idx_0 = fmaxf(rtb_Switch_iy_idx_0, 0.02F);

  /* Product: '<S99>/Product' */
  rtb_SAC_Trq_Derating_Factor = rtb_Switch_iy_idx_0 * rtb_Switch_iy_idx_0;

  /* Product: '<S99>/Product4' */
  rtb_Product4_b = rtb_SAC_Trq_Derating_Factor * rtb_Switch_iy_idx_0;

  /* Sum: '<S99>/Add' incorporates:
   *  Constant: '<S99>/Constant1'
   *  Constant: '<S99>/Constant2'
   *  Constant: '<S99>/Constant3'
   *  Constant: '<S99>/Constant4'
   *  Product: '<S99>/Divide'
   *  Product: '<S99>/Divide1'
   *  Product: '<S99>/Divide2'
   *  Product: '<S99>/Divide3'
   *  Product: '<S99>/Product1'
   *  Product: '<S99>/Product2'
   *  Product: '<S99>/Product3'
   *  Product: '<S99>/Product5'
   *  UnitDelay: '<S105>/Unit Delay'
   *  UnitDelay: '<S106>/Unit Delay'
   *  UnitDelay: '<S107>/Unit Delay'
   */
  rtb_Add_nx = ((1.0F / rtb_Product4_b * rtb_LAT_Kappa_Gradient_Fading_S - 1.0F /
                 rtb_Product4_b * DMC_Integtrator_Output2) - 3.0F /
                rtb_SAC_Trq_Derating_Factor * DMC_Integtrator_Output3) - 3.0F /
    rtb_Switch_iy_idx_0 * DMC_Integtrator_Output4;

  /* Sum: '<S184>/Add1' incorporates:
   *  Constant: '<S184>/Constant1'
   *  Inport: '<Root>/Inport3'
   *  Product: '<S184>/Product'
   *  Sum: '<S184>/Subtract'
   *  UnitDelay: '<S184>/Unit Delay'
   */
  LAT_Kappa_Dot_Filter_state = (LaKMC_kappaP_cmd - LAT_Kappa_Dot_Filter_state) *
    LAT_Kappa_Dot_Filter_Coeff + LAT_Kappa_Dot_Filter_state;

  /* Switch: '<S97>/Switch1' incorporates:
   *  Constant: '<S101>/Constant'
   *  Constant: '<S102>/Constant'
   *  Constant: '<S97>/Constant2'
   *  Inport: '<Root>/Inport26'
   *  Logic: '<S97>/AND'
   *  RelationalOperator: '<S101>/Compare'
   *  RelationalOperator: '<S102>/Compare'
   *  S-Function (sfix_bitop): '<S97>/Bitwise AND'
   *  S-Function (sfix_bitop): '<S97>/Bitwise AND1'
   */
  if (((Dyc_compensation_mode_par & 2048) == 2048) && rtb_LDP_Active) {
    rtb_NotOperator2_0 = false;
  } else {
    rtb_NotOperator2_0 = ((Dyc_compensation_mode_par & 4) == 4);
  }

  /* End of Switch: '<S97>/Switch1' */

  /* Switch: '<S86>/Switch1' incorporates:
   *  Constant: '<S100>/Constant'
   *  Constant: '<S100>/Constant1'
   *  UnitDelay: '<S184>/Unit Delay'
   */
  if (rtb_NotOperator2_0) {
    /* Switch: '<S86>/Switch' incorporates:
     *  Constant: '<S96>/Constant'
     *  Constant: '<S98>/Constant5'
     *  Inport: '<Root>/Inport26'
     *  RelationalOperator: '<S96>/Compare'
     *  S-Function (sfix_bitop): '<S86>/Bitwise AND'
     *  UnitDelay: '<S103>/Unit Delay'
     *  UnitDelay: '<S104>/Unit Delay'
     *  UnitDelay: '<S105>/Unit Delay'
     *  UnitDelay: '<S106>/Unit Delay'
     *  UnitDelay: '<S107>/Unit Delay'
     */
    if ((Dyc_compensation_mode_par & 32) == 32) {
      rtb_TRJ_Timebased_cmd_idx_0 = DMC_Integtrator_Output;
      rtb_TRJ_Timebased_cmd_idx_1 = DMC_Integtrator_Output1;
      rtb_TRJ_Timebased_cmd_idx_2 = rtb_Add_hp5;
      rtb_TRJ_Timebased_cmd_idx_3 = 0.0F;
    } else {
      rtb_TRJ_Timebased_cmd_idx_0 = DMC_Integtrator_Output2;
      rtb_TRJ_Timebased_cmd_idx_1 = DMC_Integtrator_Output3;
      rtb_TRJ_Timebased_cmd_idx_2 = DMC_Integtrator_Output4;
      rtb_TRJ_Timebased_cmd_idx_3 = rtb_Add_nx;
    }

    /* End of Switch: '<S86>/Switch' */
  } else {
    rtb_TRJ_Timebased_cmd_idx_0 = rtb_LAT_Kappa_Gradient_Fading_S;
    rtb_TRJ_Timebased_cmd_idx_1 = LAT_Kappa_Dot_Filter_state;
    rtb_TRJ_Timebased_cmd_idx_2 = 0.0F;
    rtb_TRJ_Timebased_cmd_idx_3 = 0.0F;
  }

  /* End of Switch: '<S86>/Switch1' */

  /* Sum: '<S93>/Add' incorporates:
   *  Constant: '<S93>/Constant'
   *  Product: '<S93>/Product'
   *  Sum: '<S93>/Subtract'
   *  UnitDelay: '<S93>/Unit Delay'
   */
  DYC_Kappa_Dot_Filter_state = (rtb_TRJ_Timebased_cmd_idx_1 -
    DYC_Kappa_Dot_Filter_state) * Dyc_kappa_dot_filter_coeff +
    DYC_Kappa_Dot_Filter_state;

  /* Abs: '<S19>/Abs1' incorporates:
   *  Abs: '<S275>/Abs'
   *  Abs: '<S91>/Abs'
   *  Switch: '<S85>/Switch2'
   *  UnitDelay: '<S93>/Unit Delay'
   */
  rtb_SAC_Trq_Derating_Factor = fabsf(DYC_Kappa_Dot_Filter_state);

  /* Logic: '<S19>/NOT' incorporates:
   *  Logic: '<S148>/NOT2'
   *  Logic: '<S202>/Not Operator 2'
   */
  rtb_NotOperator2_0 = !rtb_LDP_Active;

  /* Abs: '<S19>/Abs4' incorporates:
   *  Abs: '<S206>/Abs'
   *  Inport: '<Root>/LKC_Delta_Ys'
   */
  rtb_Product4_b = fabsf(LKC_Delta_Ys);

  /* Switch: '<S35>/Switch' incorporates:
   *  Abs: '<S19>/Abs1'
   *  Abs: '<S19>/Abs4'
   *  Abs: '<S19>/Abs5'
   *  Abs: '<S19>/Abs6'
   *  Abs: '<S19>/Abs7'
   *  Abs: '<S19>/Abs8'
   *  Constant: '<S19>/Constant'
   *  Constant: '<S19>/Constant1'
   *  Constant: '<S19>/Constant2'
   *  Constant: '<S19>/Constant3'
   *  Constant: '<S19>/Constant4'
   *  Constant: '<S19>/Constant5'
   *  Constant: '<S19>/Constant6'
   *  Constant: '<S19>/Constant7'
   *  Constant: '<S19>/Constant8'
   *  Constant: '<S19>/Constant9'
   *  Constant: '<S35>/Constant'
   *  Constant: '<S35>/Constant1'
   *  Gain: '<S19>/Gain'
   *  Inport: '<Root>/LKC_Delta_Psi'
   *  Logic: '<S19>/AND'
   *  Logic: '<S19>/NOT'
   *  Product: '<S19>/Product1'
   *  RelationalOperator: '<S19>/Less Than'
   *  RelationalOperator: '<S19>/Less Than1'
   *  RelationalOperator: '<S19>/Less Than2'
   *  RelationalOperator: '<S19>/Less Than3'
   *  RelationalOperator: '<S19>/Less Than4'
   *  RelationalOperator: '<S19>/Less Than5'
   *  RelationalOperator: '<S19>/Less Than6'
   *  RelationalOperator: '<S19>/Less Than7'
   *  RelationalOperator: '<S19>/Less Than8'
   *  RelationalOperator: '<S19>/Less Than9'
   *  Sum: '<S35>/Add'
   *  UnitDelay: '<S190>/Unit Delay'
   *  UnitDelay: '<S35>/Unit Delay'
   */
  if ((0.0F < Tdf_idle_max_torque_request) && (rtb_SAC_Trq_Derating_Factor <
       Tdf_idle_max_torque_request) && (rtb_SAC_Arbitrated_Angle_Cmd * rtb_Max_a
       < Tdf_idle_max_lateral_accel) && (rtb_Max_a < Tdf_idle_max_kappa) &&
      (TDF_Torque_Request_Factor_sta_0 < Tdf_idle_max_delta_f_dot) &&
      (rtb_Product4_b < Tdf_idle_max_lateral_error) && rtb_NotOperator2_0 &&
      (fabsf(SAC_Yrc_Control_Error) < Tdf_idle_max_curvature_error) && (fabsf
       (LAT_Yaw_Rate_Filter_state) < Tdf_idle_max_yaw_rate) && (fabsf(57.3F *
        LKC_Delta_Psi) < Tdf_idle_max_heading_error) && (fabsf(VEH_Delta_F_Oc) <
       Tdf_idle_max_steer_angle)) {
    TDF_Idle_Enable_Condition_state = TDF_Idle_Enable_Condition_state + 1.0F;
  } else {
    TDF_Idle_Enable_Condition_state = 0.0F;
  }

  /* End of Switch: '<S35>/Switch' */

  /* Abs: '<S25>/Abs' incorporates:
   *  UnitDelay: '<S38>/Unit Delay1'
   */
  VEH_Abs_Steer_Torque_Comp_pre = rtb_UnitDelay5;

  /* Switch: '<S39>/Switch' incorporates:
   *  UnitDelay: '<S38>/Unit Delay1'
   *  UnitDelay: '<S39>/Unit Delay'
   */
  if (!TDF_Driver_Counter_Steering) {
    TDF_Steer_Torque_Sample_pre = VEH_Abs_Steer_Torque_Comp_pre;
  }

  /* End of Switch: '<S39>/Switch' */

  /* Product: '<S47>/Product' incorporates:
   *  Constant: '<S47>/Constant'
   */
  rtb_Subtract_jc *= Tdf_sac_pritaty_omega;

  /* MinMax: '<S47>/Min' incorporates:
   *  Constant: '<S47>/Constant1'
   */
  rtb_Subtract_jc = fminf(Tdf_maximum_delta_f_dot, rtb_Subtract_jc);

  /* Sum: '<S47>/Add' incorporates:
   *  Constant: '<S47>/Constant1'
   *  Gain: '<S47>/Gain'
   *  Inport: '<Root>/Inport13'
   *  MinMax: '<S47>/Max'
   *  Product: '<S47>/Product1'
   *  UnitDelay: '<S47>/Unit Delay'
   */
  TDF_Saturated_Angle_Command_Error_state = VEH_cycletime * fmaxf
    (-Tdf_maximum_delta_f_dot, rtb_Subtract_jc) +
    TDF_Saturated_Angle_Command_Error_state;

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S17>/Constant'
   *  Constant: '<S28>/Constant'
   *  Constant: '<S31>/Constant1'
   *  Constant: '<S31>/Constant2'
   *  Gain: '<S17>/Gain'
   *  Lookup_n-D: '<S17>/1-D Lookup Table'
   *  Product: '<S17>/Product'
   *  Product: '<S28>/Product'
   *  Product: '<S28>/Product1'
   *  Sum: '<S28>/Add'
   *  Sum: '<S28>/Subtract'
   *  Switch: '<S31>/Switch'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  if (rtb_LDP_Active) {
    rtb_Tdf_min_steer_torque_class = look1_iflf_binlxpw(3.6F *
      rtb_VEH_Vehicle_Speed, ((const real32_T *)
      &(Tdf_steer_trq_cmp_ldp_x_schedul[0])), ((const real32_T *)
      &(Tdf_steer_trq_cmp_ldp_y_schedul[0])), 10U) *
      Tdf_steer_torque_comp_thrs_ldp * (1.0F - TDF_Ldp_Override_Factor_state) +
      rtb_Tdf_min_steer_torque_class * TDF_Ldp_Override_Factor_state;
    rtb_Switch_iy_idx_0 = Tdf_steer_torque_comp_slope_ldp;
  } else {
    rtb_Switch_iy_idx_0 = Tdf_steer_torque_comp_slope;
  }

  /* End of Switch: '<S17>/Switch' */

  /* Sum: '<S50>/Subtract' incorporates:
   *  Abs: '<S32>/Abs'
   *  Constant: '<S17>/Constant1'
   *  Constant: '<S17>/Constant7'
   *  Constant: '<S26>/Constant'
   *  Constant: '<S31>/Constant'
   *  Constant: '<S34>/Constant'
   *  Gain: '<S17>/Gain1'
   *  Gain: '<S31>/Gain'
   *  Lookup_n-D: '<S31>/1-D Lookup Table'
   *  MinMax: '<S17>/Min1'
   *  MinMax: '<S31>/Max'
   *  MinMax: '<S34>/Max'
   *  MinMax: '<S34>/Min'
   *  Product: '<S17>/Product1'
   *  Product: '<S26>/Product'
   *  Product: '<S26>/Product1'
   *  Product: '<S31>/Product'
   *  Product: '<S31>/Product1'
   *  Product: '<S31>/Product2'
   *  Sum: '<S26>/Add'
   *  Sum: '<S26>/Subtract'
   *  Sum: '<S31>/Subtract'
   *  UnitDelay: '<S50>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = -fminf(fmaxf(fminf(fmaxf(rtb_UnitDelay5 - ((1.0F -
    LAT_Stiffness_Request_Factor_pre) * Tdf_steer_trq_comp_reduced_thrs +
    rtb_Tdf_min_steer_torque_class * LAT_Stiffness_Request_Factor_pre), 0.0F) *
    rtb_UnitDelay6 * rtb_Switch_iy_idx_0 * look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(Tdf_steer_trq_cmp_slp_x_schedul
    [0])), ((const real32_T *)&(Tdf_steer_trq_cmp_slp_y_schedul[0])), 11U), 1.0F),
    -1.0F) * TDF_Torque_Request_Factor, 0.0F) - TDF_Flt_Drv_Comp_Factor_state;

  /* Switch: '<S50>/Switch' incorporates:
   *  Constant: '<S50>/Constant1'
   *  Constant: '<S50>/Constant6'
   *  Constant: '<S50>/Constant7'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S50>/Divide'
   *  RelationalOperator: '<S50>/GreaterThan'
   *  Sum: '<S50>/Add1'
   */
  if (rtb_Switch_iy_idx_0 >= Tdf_comp_filter_min_residual) {
    rtb_Subtract_jc = 1.0F;
  } else {
    rtb_Subtract_jc = 1.0F / (Tdf_comp_factor_filter_coeff + VEH_cycletime) *
      VEH_cycletime;
  }

  /* End of Switch: '<S50>/Switch' */

  /* Sum: '<S50>/Add' incorporates:
   *  Product: '<S50>/Product'
   *  UnitDelay: '<S50>/Unit Delay'
   */
  TDF_Flt_Drv_Comp_Factor_state = rtb_Switch_iy_idx_0 * rtb_Subtract_jc +
    TDF_Flt_Drv_Comp_Factor_state;

  /* MinMax: '<S57>/Max' incorporates:
   *  Constant: '<S57>/Constant'
   *  Constant: '<S57>/Constant1'
   *  MinMax: '<S57>/Min'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = fmaxf(fminf(TDF_Derating_Switch_Min_Time,
    TDF_Derating_Time_state), 0.0F);

  /* Sum: '<S54>/Subtract' incorporates:
   *  Constant: '<S57>/Constant'
   *  RelationalOperator: '<S57>/GreaterThanOrEqual'
   */
  rtb_Subtract_jc = (real32_T)(TDF_Derating_Switch_Min_Time <=
    rtb_Switch_iy_idx_0) - rtb_TDF_State_Derating_Factor_C;

  /* Switch: '<S54>/Switch1' incorporates:
   *  Constant: '<S54>/Constant2'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S54>/Product2'
   *  Sum: '<S54>/Add'
   *  Switch: '<S54>/Switch'
   *  UnitDelay: '<S54>/Unit Delay'
   */
  if (rtb_SAC_Disable) {
    TDF_State_Derating_Factor_Comp_state = 0.0F;
  } else {
    if (rtb_Subtract_jc != 0.0F) {
      /* Signum: '<S54>/Sign' */
      if (rtb_Subtract_jc < 0.0F) {
        rtb_Subtract_jc = -1.0F;
      } else {
        if (rtb_Subtract_jc > 0.0F) {
          rtb_Subtract_jc = 1.0F;
        }
      }

      /* Switch: '<S54>/Switch' incorporates:
       *  Constant: '<S54>/Constant'
       *  Product: '<S54>/Product'
       */
      rtb_Subtract_jc *= TDF_Switch_Rising_Rate;
    } else {
      /* Switch: '<S54>/Switch' incorporates:
       *  Signum: '<S54>/Sign'
       */
      rtb_Subtract_jc = 0.0F;
    }

    TDF_State_Derating_Factor_Comp_state = rtb_Subtract_jc * VEH_cycletime +
      rtb_TDF_State_Derating_Factor_C;
  }

  /* End of Switch: '<S54>/Switch1' */

  /* Switch: '<S58>/Switch' incorporates:
   *  Constant: '<S58>/Constant1'
   *  Constant: '<S58>/Constant3'
   *  Inport: '<Root>/Inport55'
   *  Inport: '<Root>/Inport56'
   *  RelationalOperator: '<S58>/GreaterThan'
   *  RelationalOperator: '<S58>/GreaterThan1'
   *  Sum: '<S58>/Add'
   *  Switch: '<S58>/Switch1'
   *  UnitDelay: '<S38>/Unit Delay1'
   */
  if (VEH_Abs_Steer_Torque_Comp_pre > TDF_Derating_Switch_Thresh2_par) {
    rtb_Subtract_jc = (VEH_Abs_Steer_Torque_Comp_pre -
                       TDF_Derating_Switch_Thresh2_par) + 1.0F;
  } else if (VEH_Abs_Steer_Torque_Comp_pre > TDF_Derating_Switch_Thresh1_par) {
    /* Switch: '<S58>/Switch1' incorporates:
     *  Constant: '<S58>/Constant1'
     */
    rtb_Subtract_jc = 1.0F;
  } else {
    rtb_Subtract_jc = -1.0F;
  }

  /* End of Switch: '<S58>/Switch' */

  /* Sum: '<S57>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S57>/Product'
   *  UnitDelay: '<S57>/Unit Delay'
   */
  TDF_Derating_Time_state = rtb_Subtract_jc * VEH_cycletime +
    rtb_Switch_iy_idx_0;

  /* Sum: '<S70>/Add1' incorporates:
   *  Constant: '<S70>/Constant'
   *  Product: '<S70>/Product'
   *  Sum: '<S70>/Subtract'
   *  UnitDelay: '<S70>/Unit Delay'
   */
  LDC_Sac_Parity_Filter_state = (rtb_LAT_Saturated_Delta_F_Cmd -
    LDC_Sac_Parity_Filter_state) * 0.02F + LDC_Sac_Parity_Filter_state;

  /* MinMax: '<S75>/Min' */
  SAT_Req_Steer_Angle_Max = fminf(rtb_SAT_Max_Delta_F_Cmd,
    rtb_SAT_Max_Steer_Angle_Cmd_Sel);

  /* Product: '<S62>/Product' incorporates:
   *  Abs: '<S62>/Abs'
   *  Constant: '<S62>/Constant'
   *  Sum: '<S62>/Subtract'
   *  UnitDelay: '<S70>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = (fabsf(LDC_Sac_Parity_Filter_state) -
    SAT_Req_Steer_Angle_Max) * Sat_thrs_control_kp;

  /* Sum: '<S69>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S69>/Product'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  LAT_Sat_Dynamic_Threshold_int = rtb_Switch_iy_idx_0 * VEH_cycletime +
    LAT_Sat_Dynamic_Threshold_int;

  /* MinMax: '<S62>/Max' incorporates:
   *  Constant: '<S62>/Constant1'
   *  UnitDelay: '<S62>/Unit Delay'
   */
  LAT_Sat_Dynamic_Threshold_state = fmaxf(0.0F, rtb_Switch_iy_idx_0);

  /* Gain: '<S84>/Gain1' incorporates:
   *  Sum: '<S77>/Add1'
   */
  SAC_Arbitrated_Angle_Cmd_Raw = 0.0174532924F * rtb_LAT_Delta_F_cmd;

  /* Gain: '<S85>/Gain' */
  rtb_SAT_Max_Delta_F_Cmd = 3.6F * rtb_VEH_Vehicle_Speed;

  /* Switch: '<S92>/Switch' incorporates:
   *  Constant: '<S94>/Constant'
   *  Gain: '<S85>/Gain'
   *  Lookup_n-D: '<S92>/1-D Lookup Table'
   *  RelationalOperator: '<S94>/Compare'
   */
  if (LAT_Oc_Kappa_Active_tmp == 4U) {
    rtb_Subtract_jc = look1_iflf_binlxpw(rtb_SAT_Max_Delta_F_Cmd, ((const
      real32_T *)&(Dyc_kappa_a2_x_scheduling_Sc[0])), ((const real32_T *)
      &(Dyc_kappa_a2_y_scheduling_Sc[0])), 11U);
  } else {
    /* Lookup_n-D: '<S92>/1-D Lookup Table1' incorporates:
     *  Gain: '<S85>/Gain'
     *  Inport: '<Root>/Inport22'
     *  Inport: '<Root>/Inport23'
     */
    rtb_Subtract_jc = look1_iflf_binlxpw(rtb_SAT_Max_Delta_F_Cmd,
      Dyc_kappa_a2_x_scheduling_par, Dyc_kappa_a2_y_scheduling_par, 11U);
  }

  /* End of Switch: '<S92>/Switch' */

  /* Switch: '<S85>/Switch2' incorporates:
   *  Constant: '<S90>/Constant'
   *  Inport: '<Root>/Inport26'
   *  RelationalOperator: '<S90>/Compare'
   *  S-Function (sfix_bitop): '<S85>/Bitwise AND2'
   */
  if ((Dyc_compensation_mode_par & 3) == 2) {
    /* Switch: '<S85>/Switch' incorporates:
     *  Constant: '<S88>/Constant'
     *  Constant: '<S89>/Constant'
     *  Logic: '<S85>/AND'
     *  RelationalOperator: '<S88>/Compare'
     *  RelationalOperator: '<S89>/Compare'
     *  S-Function (sfix_bitop): '<S85>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S85>/Bitwise AND1'
     *  Switch: '<S85>/Switch1'
     */
    if (((Dyc_compensation_mode_par & 1024) == 1024) && rtb_LDP_Active) {
      rtb_Switch_iy_idx_0 = 1.0F;
      rtb_Subtract_jc = 0.0F;
      rtb_LAT_Delta_F_cmd = 0.0F;
      rtb_SAT_Max_Delta_F_Cmd = 0.0F;
    } else if ((Dyc_compensation_mode_par & 8) == 8) {
      /* Switch: '<S85>/Switch1' incorporates:
       *  Constant: '<S85>/Constant1'
       */
      rtb_Switch_iy_idx_0 = DYC_Kappa_Coeff[0];
      rtb_Subtract_jc = DYC_Kappa_Coeff[1];
      rtb_LAT_Delta_F_cmd = DYC_Kappa_Coeff[2];
      rtb_SAT_Max_Delta_F_Cmd = DYC_Kappa_Coeff[3];
    } else {
      /* Switch: '<S85>/Switch1' incorporates:
       *  Constant: '<S85>/Constant2'
       */
      rtb_Switch_iy_idx_0 = 1.0F;

      /* Switch: '<S92>/Switch1' incorporates:
       *  Constant: '<S95>/Constant'
       *  Inport: '<Root>/Inport27'
       *  Inport: '<Root>/Inport28'
       *  Product: '<S92>/Product'
       *  Product: '<S92>/Product1'
       *  RelationalOperator: '<S95>/Compare'
       *  Switch: '<S85>/Switch1'
       */
      if (rtb_TRJ_Timebased_cmd_idx_0 > 0.0F) {
        rtb_Subtract_jc *= Dyc_kappa_a2_pos_corr_factor_par;
      } else {
        rtb_Subtract_jc *= Dyc_kappa_a2_factor_par;
      }

      /* End of Switch: '<S92>/Switch1' */

      /* Switch: '<S85>/Switch1' incorporates:
       *  Constant: '<S85>/Constant3'
       *  Constant: '<S85>/Constant4'
       *  Gain: '<S85>/Gain'
       *  Lookup_n-D: '<S85>/1-D Lookup Table'
       *  Lookup_n-D: '<S85>/1-D Lookup Table1'
       *  Product: '<S85>/Product'
       *  Product: '<S85>/Product1'
       */
      rtb_LAT_Delta_F_cmd = look1_iflf_binlxpw(rtb_SAT_Max_Delta_F_Cmd, ((const
        real32_T *)&(Dyc_kappa_a3_x_scheduling[0])), ((const real32_T *)
        &(Dyc_kappa_a3_y_scheduling[0])), 10U) * Dyc_kappa_a3_factor;
      rtb_SAT_Max_Delta_F_Cmd = look1_iflf_binlxpw(rtb_SAT_Max_Delta_F_Cmd, ((
        const real32_T *)&(Dyc_kappa_A4_X_scheduling[0])), ((const real32_T *)
        &(Dyc_kappa_A4_Y_scheduling[0])), 10U) * Dyc_kappa_A4_factor;
    }

    /* End of Switch: '<S85>/Switch' */

    /* Switch: '<S91>/Switch' incorporates:
     *  Constant: '<S91>/Constant'
     *  Constant: '<S91>/Constant1'
     *  Constant: '<S91>/Constant2'
     *  Product: '<S91>/Product'
     *  RelationalOperator: '<S91>/GreaterThan'
     *  UnitDelay: '<S93>/Unit Delay'
     */
    if (rtb_SAC_Trq_Derating_Factor > Dyc_kappa_dot_boost_thrs) {
      rtb_Tdf_min_steer_torque_class = Dyc_kappa_a2_boost_factor *
        DYC_Kappa_Dot_Filter_state;
    } else {
      rtb_Tdf_min_steer_torque_class = 0.0F;
    }

    /* End of Switch: '<S91>/Switch' */

    /* Signum: '<S91>/Sign' */
    if (rtb_Tdf_min_steer_torque_class < 0.0F) {
      rtb_UnitDelay5 = -1.0F;
    } else if (rtb_Tdf_min_steer_torque_class > 0.0F) {
      rtb_UnitDelay5 = 1.0F;
    } else {
      rtb_UnitDelay5 = rtb_Tdf_min_steer_torque_class;
    }

    /* End of Signum: '<S91>/Sign' */

    /* Switch: '<S85>/Switch2' incorporates:
     *  Abs: '<S91>/Abs1'
     *  Constant: '<S91>/Constant4'
     *  Constant: '<S91>/Constant5'
     *  DotProduct: '<S85>/Dot Product'
     *  MinMax: '<S91>/Max'
     *  Product: '<S91>/Product1'
     *  Sum: '<S85>/Add'
     *  Sum: '<S91>/Subtract'
     */
    DYC_Filter_Kappa_Command = (((rtb_TRJ_Timebased_cmd_idx_0 *
      rtb_Switch_iy_idx_0 + rtb_TRJ_Timebased_cmd_idx_1 * rtb_Subtract_jc) +
      rtb_TRJ_Timebased_cmd_idx_2 * rtb_LAT_Delta_F_cmd) +
      rtb_TRJ_Timebased_cmd_idx_3 * rtb_SAT_Max_Delta_F_Cmd) + fmaxf(0.0F, fabsf
      (rtb_Tdf_min_steer_torque_class) - Dyc_boost_signal_reduction) *
      rtb_UnitDelay5;
  } else {
    /* Switch: '<S85>/Switch2' incorporates:
     *  Constant: '<S85>/Constant5'
     */
    DYC_Filter_Kappa_Command = 0.0F;
  }

  /* Sum: '<S103>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S103>/Product'
   *  UnitDelay: '<S103>/Unit Delay'
   */
  DMC_Integtrator_Output1 = rtb_Add_hp5 * VEH_cycletime +
    DMC_Integtrator_Output1;

  /* Sum: '<S104>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S104>/Product'
   *  UnitDelay: '<S104>/Unit Delay'
   */
  DMC_Integtrator_Output = rtb_Product_lg * VEH_cycletime +
    DMC_Integtrator_Output;

  /* Sum: '<S105>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S105>/Product'
   *  UnitDelay: '<S105>/Unit Delay'
   */
  DMC_Integtrator_Output4 = rtb_Add_nx * VEH_cycletime + DMC_Integtrator_Output4;

  /* Sum: '<S106>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S106>/Product'
   *  UnitDelay: '<S106>/Unit Delay'
   */
  DMC_Integtrator_Output3 = rtb_DMC_Integtrator_Output_b * VEH_cycletime +
    DMC_Integtrator_Output3;

  /* Sum: '<S107>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Product: '<S107>/Product'
   *  UnitDelay: '<S107>/Unit Delay'
   */
  DMC_Integtrator_Output2 = rtb_DMC_Integtrator_Output_ks * VEH_cycletime +
    DMC_Integtrator_Output2;

  /* Switch: '<S115>/Switch' incorporates:
   *  Constant: '<S115>/Constant'
   *  Constant: '<S115>/Constant1'
   *  Constant: '<S120>/Constant'
   *  Gain: '<S121>/Gain'
   *  Inport: '<Root>/Inport26'
   *  Product: '<S115>/Product1'
   *  RelationalOperator: '<S120>/Compare'
   *  S-Function (sfix_bitop): '<S115>/Bitwise AND'
   *  Sum: '<S115>/Add'
   */
  if ((Dyc_compensation_mode_par & 16) == 16) {
    /* Switch: '<S114>/Switch1' incorporates:
     *  Constant: '<S119>/Constant'
     *  Gain: '<S108>/Gain'
     *  Inport: '<Root>/Inport29'
     *  Inport: '<Root>/Inport30'
     *  Inport: '<Root>/Inport31'
     *  Inport: '<Root>/Inport32'
     *  Inport: '<Root>/Inport33'
     *  Lookup_n-D: '<S114>/1-D Lookup Table1'
     *  Lookup_n-D: '<S114>/1-D Lookup Table2'
     *  Product: '<S114>/Product'
     *  Product: '<S114>/Product1'
     *  Product: '<S114>/Product2'
     *  RelationalOperator: '<S119>/Compare'
     */
    if (rtb_LAT_Kappa_Gradient_Fading_S > 0.0F) {
      /* Switch: '<S114>/Switch' incorporates:
       *  Constant: '<S114>/Constant2'
       *  Gain: '<S108>/Gain'
       *  Lookup_n-D: '<S114>/1-D Lookup Table'
       *  RelationalOperator: '<S118>/Compare'
       */
      if (rtb_LDP_Active) {
        rtb_Switch_iy_idx_0 = look1_iflf_binlxpw(rtb_Gain_d, ((const real32_T *)
          &(Dyc_kappa_angle_t_x_schedul_gen[0])), ((const real32_T *)
          &(Dyc_kappa_angle_ldp_corr_y_sch[0])), 11U);
      } else {
        rtb_Switch_iy_idx_0 = 1.0F;
      }

      /* End of Switch: '<S114>/Switch' */
      rtb_Switch_iy_idx_0 *= Dyc_kappa_angle_gen_corr_factor_par *
        look1_iflf_binlxpw(rtb_Gain_d, Dyc_kappa_angle_t_x_schedul_gen_par,
                           Dyc_kappa_angle_t_y_schedul_gen_par, 11U);
    } else {
      rtb_Switch_iy_idx_0 = Dyc_kappa_angle_gen_cor_fct_neg_par *
        look1_iflf_binlxpw(rtb_Gain_d, Dyc_kappa_angle_t_x_schedul_gen_par,
                           Dyc_kappa_angle_t_y_sch_gen_neg_par, 11U);
    }

    /* End of Switch: '<S114>/Switch1' */
  } else {
    rtb_Switch_iy_idx_0 = (rtb_SAC_Arbitrated_Angle_Cmd *
      VEH_Vehicle_Selfsteering_Factor + VEH_Vehicle_Wheel_Base) * 57.2957802F;
  }

  /* Switch: '<S108>/Switch' incorporates:
   *  Abs: '<S108>/Abs'
   *  Constant: '<S108>/Constant1'
   *  Constant: '<S108>/Constant2'
   *  Constant: '<S108>/Constant3'
   *  Constant: '<S113>/Constant'
   *  Constant: '<S116>/Constant'
   *  Constant: '<S116>/Constant1'
   *  Inport: '<Root>/Inport26'
   *  MinMax: '<S116>/Max'
   *  MinMax: '<S116>/Min'
   *  Product: '<S108>/Product1'
   *  Product: '<S108>/Product3'
   *  RelationalOperator: '<S113>/Compare'
   *  S-Function (sfix_bitop): '<S108>/Bitwise AND'
   *  Sum: '<S108>/Subtract'
   */
  if ((Dyc_compensation_mode_par & 512) == 512) {
    rtb_Subtract_jc = fmaxf(0.0F, fminf(1.0F, (fabsf
      (rtb_SAC_Arbitrated_Angle_Cmd * rtb_LAT_Kappa_Gradient_Fading_S) -
      DYC_Ay_Linear_Treshold) * DYC_Ay_Linear_Fading_Constant));
  } else {
    rtb_Subtract_jc = 0.0F;
  }

  /* Sum: '<S108>/Add' incorporates:
   *  Constant: '<S108>/Constant'
   *  Constant: '<S117>/Constant'
   *  Gain: '<S108>/Gain'
   *  Lookup_n-D: '<S108>/1-D Lookup Table'
   *  Product: '<S108>/Product'
   *  Product: '<S108>/Product4'
   *  Product: '<S108>/Product5'
   *  Sum: '<S117>/Subtract'
   */
  rtb_LAT_Delta_F_cmd = (1.0F - rtb_Subtract_jc) * rtb_Switch_iy_idx_0 +
    DYC_Kappa_Angle_Hi_Ay_Corr_Fact * look1_iflf_binlxpw(rtb_Gain_d, ((const
    real32_T *)&(DYC_Kappa_Angle_T_X_Sch_Hi_Ay[0])), ((const real32_T *)
    &(DYC_Kappa_Angle_T_Y_Sch_Hi_Ay[0])), 11U) * rtb_Subtract_jc;

  /* Sum: '<S112>/Add' incorporates:
   *  Constant: '<S81>/Constant'
   *  Product: '<S112>/Product'
   *  Product: '<S87>/Product'
   *  Product: '<S87>/Product1'
   *  Sum: '<S112>/Subtract'
   *  Sum: '<S81>/Add'
   *  UnitDelay: '<S112>/Unit Delay'
   */
  HEC_Yaw_Rate_Filter = HEC_Yaw_Rate_Filter_state2;
  HEC_Yaw_Rate_Filter = ((Hec_r_pt1_factor + rtb_Hec_r_factor) *
    rtb_VEH_Vehicle_Speed * DYC_Filter_Kappa_Command - HEC_Yaw_Rate_Filter) *
    rtb_Abs1_j + HEC_Yaw_Rate_Filter;

  /* Product: '<S126>/Product' incorporates:
   *  Constant: '<S111>/Constant'
   *  Constant: '<S111>/Constant1'
   *  Constant: '<S111>/Constant2'
   *  Constant: '<S111>/Constant3'
   *  Constant: '<S111>/Constant4'
   *  Constant: '<S126>/Constant'
   *  Constant: '<S6>/Constant7'
   *  DiscreteIntegrator: '<S126>/Discrete-Time Integrator'
   *  Gain: '<S124>/Gain'
   *  Gain: '<S125>/Gain'
   *  MinMax: '<S124>/Max'
   *  MinMax: '<S124>/Min'
   *  MinMax: '<S125>/Max'
   *  MinMax: '<S125>/Min'
   *  Product: '<S111>/Product'
   *  Product: '<S111>/Product1'
   *  Product: '<S111>/Product2'
   *  Product: '<S111>/Product3'
   *  Product: '<S111>/Product4'
   *  Product: '<S111>/Product5'
   *  Sum: '<S126>/Subtract'
   */
  rtb_Abs1_j = (fmaxf(fminf(VEH_Vehicle_Reduced_Mass *
    VEH_Force_Long_Tire_Factor * DYC_Long_Force_Comp_Cor_Factor * fmaxf(fminf
    (rtb_VEH_Vehicle_Speed * rtb_VEH_Vehicle_Speed *
     rtb_LAT_Kappa_Gradient_Fading_S, DYC_Max_Lateral_Acceleration),
    -DYC_Max_Lateral_Acceleration) * 0.0F, DYC_Max_Delta_F_Correction),
                      -DYC_Max_Delta_F_Correction) -
                DYC_Boost_Filter_Output_state) * DYC_Filter_Omega;

  /* PreLookup: '<S151>/Prelookup1' incorporates:
   *  Constant: '<S151>/Constant1'
   *  Gain: '<S151>/Gain1'
   */
  rtb_Vehicle_Speed_Index = plook_u32ff_binc(3.6F * rtb_VEH_Vehicle_Speed, ((
    const real32_T *)&(SpeedSegmentLookup2[0])), 8U, &rtb_Hec_r_factor);

  /* Switch: '<S149>/Switch' incorporates:
   *  UnitDelay: '<S149>/Unit Delay'
   *  UnitDelay: '<S82>/Unit Delay'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  if (Initialisation_Flag_state) {
    for (i = 0; i < 9; i++) {
      rtb_Switch_iu[i] = Corretion_Factor_Left_Vect[i];
      rtb_Switch_iu[i + 9] = Corretion_Factor_Right_Vect[i];
    }
  } else {
    for (i = 0; i < 9; i++) {
      /* Switch: '<S149>/Switch1' */
      rtb_Switch_iu[i] = 1.0F;
      rtb_Switch_iu[i + 9] = 1.0F;
    }
  }

  /* End of Switch: '<S149>/Switch' */

  /* SignalConversion generated from: '<S150>/Assignment' incorporates:
   *  UnitDelay: '<S82>/Unit Delay'
   */
  for (i = 0; i < 9; i++) {
    Corretion_Factor_Right_Vect[i] = rtb_Switch_iu[i];
  }

  /* End of SignalConversion generated from: '<S150>/Assignment' */

  /* Sum: '<S147>/Add' incorporates:
   *  Constant: '<S147>/Constant'
   *  Inport: '<Root>/Inport7'
   *  Product: '<S147>/Product'
   *  Sum: '<S147>/Subtract'
   *  UnitDelay: '<S147>/Unit Delay'
   */
  Veh_Lat_Acc_Filt = (Veh_Lat_Acc - Veh_Lat_Acc_Filt) * 0.95F + Veh_Lat_Acc_Filt;

  /* RelationalOperator: '<S146>/GreaterThanOrEqual' incorporates:
   *  Constant: '<S146>/Constant'
   *  Inport: '<Root>/Inport18'
   */
  rtb_LDP_Active = (CAM_Latral_Error_Qf >= 1.0F);

  /* Logic: '<S165>/NOT' */
  rtb_NOT_n = !rtb_LDP_Active;

  /* Logic: '<S168>/OR' incorporates:
   *  UnitDelay: '<S168>/Unit Delay'
   */
  rtb_OR = (New_Update_Aval || rtb_NOT_n);

  /* Delay: '<S168>/Resettable Delay1' */
  if (rtb_OR) {
    Mean_Sample_Update_sum = 0.0F;
  }

  /* Switch: '<S168>/Switch1' incorporates:
   *  Constant: '<S168>/Constant4'
   *  Inport: '<Root>/Inport19'
   */
  if (rtb_NOT_n) {
    rtb_Switch_iy_idx_0 = 0.0F;
  } else {
    rtb_Switch_iy_idx_0 = CAM_Lateral_Error;
  }

  /* End of Switch: '<S168>/Switch1' */

  /* Sum: '<S168>/Add1' incorporates:
   *  Delay: '<S168>/Resettable Delay1'
   */
  Mean_Sample_Update_sum = Mean_Sample_Update_sum + rtb_Switch_iy_idx_0;

  /* Delay: '<S168>/Resettable Delay' */
  if (rtb_OR) {
    New_Update_Aval_sum = 0.0F;
  }

  /* Sum: '<S168>/Add' incorporates:
   *  Delay: '<S168>/Resettable Delay'
   *  Switch: '<S168>/Switch'
   */
  New_Update_Aval_sum = New_Update_Aval_sum + (real32_T)!rtb_NOT_n;

  /* MinMax: '<S168>/Max' incorporates:
   *  Constant: '<S168>/Constant3'
   *  Delay: '<S168>/Resettable Delay'
   */
  rtb_Gain_d = fmaxf(New_Update_Aval_sum, 1.0F);

  /* RelationalOperator: '<S168>/GreaterThanOrEqual' incorporates:
   *  Constant: '<S168>/Constant'
   *  Delay: '<S168>/Resettable Delay'
   *  UnitDelay: '<S168>/Unit Delay'
   */
  New_Update_Aval = (New_Update_Aval_sum >= Acf_no_of_items_in_sample);

  /* Switch: '<S168>/Switch2' incorporates:
   *  Delay: '<S168>/Resettable Delay1'
   *  Product: '<S168>/Divide'
   *  UnitDelay: '<S168>/Unit Delay'
   *  UnitDelay: '<S168>/Unit Delay1'
   */
  if (New_Update_Aval) {
    Mean_Sample_Update = Mean_Sample_Update_sum / rtb_Gain_d;
  }

  /* End of Switch: '<S168>/Switch2' */

  /* Outputs for Enabled SubSystem: '<S165>/Mean_Of_All_Segments' incorporates:
   *  EnablePort: '<S167>/Enable'
   */
  /* Logic: '<S165>/OR' incorporates:
   *  UnitDelay: '<S168>/Unit Delay'
   */
  if (New_Update_Aval || rtb_NOT_n) {
    /* Switch: '<S167>/Switch' incorporates:
     *  Constant: '<S167>/Constant'
     *  Delay: '<S167>/Resettable Delay1'
     *  UnitDelay: '<S168>/Unit Delay1'
     */
    if (rtb_NOT_n) {
      rtb_Switch_iy_idx_0 = 0.0F;

      /* Delay: '<S167>/Resettable Delay' incorporates:
       *  Constant: '<S167>/Constant'
       */
      Mean_Sample_Update_pre1 = 0.0F;
      Mean_Sample_Update_pre2 = 0.0F;
    } else {
      rtb_Switch_iy_idx_0 = Mean_Sample_Update;
    }

    /* End of Switch: '<S167>/Switch' */

    /* Delay: '<S167>/Resettable Delay' */
    rtb_Subtract_jc = Mean_Sample_Update_pre1;

    /* Delay: '<S167>/Resettable Delay1' */
    rtb_SAT_Max_Delta_F_Cmd = Mean_Sample_Update_pre2;

    /* Delay: '<S167>/Resettable Delay2' incorporates:
     *  Delay: '<S167>/Resettable Delay3'
     *  Delay: '<S167>/Resettable Delay4'
     */
    if (rtb_NOT_n) {
      Mean_Sample_Update_pre3 = 0.0F;
      Mean_Sample_Update_pre4 = 0.0F;
      Lateral_Error_Invalid_state = 1.0F;
    }

    rtb_Tdf_min_steer_torque_class = Mean_Sample_Update_pre3;

    /* Product: '<S167>/Divide' incorporates:
     *  Delay: '<S167>/Resettable Delay'
     *  Delay: '<S167>/Resettable Delay1'
     *  Delay: '<S167>/Resettable Delay2'
     *  Delay: '<S167>/Resettable Delay3'
     *  Delay: '<S167>/Resettable Delay4'
     *  Sum: '<S167>/Add'
     */
    Lateral_Error_Mean = ((((rtb_Switch_iy_idx_0 + Mean_Sample_Update_pre1) +
      Mean_Sample_Update_pre2) + Mean_Sample_Update_pre3) +
                          Mean_Sample_Update_pre4) / Lateral_Error_Invalid_state;

    /* Signum: '<S167>/Sign' */
    if (Lateral_Error_Mean < 0.0F) {
      /* Signum: '<S167>/Sign' */
      CAM_Lateral_Error_Sign = -1.0F;
    } else if (Lateral_Error_Mean > 0.0F) {
      /* Signum: '<S167>/Sign' */
      CAM_Lateral_Error_Sign = 1.0F;
    } else {
      /* Signum: '<S167>/Sign' */
      CAM_Lateral_Error_Sign = Lateral_Error_Mean;
    }

    /* End of Signum: '<S167>/Sign' */

    /* Delay: '<S167>/Resettable Delay5' */
    if (rtb_NOT_n) {
      Lateral_Error_Mean_state = 0.0F;
    }

    /* Sum: '<S167>/Subtract' incorporates:
     *  Delay: '<S167>/Resettable Delay5'
     */
    Lateral_Error_Delta = Lateral_Error_Mean - Lateral_Error_Mean_state;

    /* Switch: '<S167>/Switch1' incorporates:
     *  Constant: '<S167>/Constant2'
     *  Constant: '<S167>/Constant3'
     *  Delay: '<S167>/Resettable Delay4'
     *  RelationalOperator: '<S167>/Less Than'
     *  Sum: '<S167>/Add1'
     */
    if (Lateral_Error_Invalid_state < 5.0F) {
      Lateral_Error_Invalid_state = Lateral_Error_Invalid_state + 1.0F;
    }

    /* End of Switch: '<S167>/Switch1' */

    /* Update for Delay: '<S167>/Resettable Delay' */
    Mean_Sample_Update_pre1 = rtb_Switch_iy_idx_0;

    /* Update for Delay: '<S167>/Resettable Delay1' */
    Mean_Sample_Update_pre2 = rtb_Subtract_jc;

    /* Update for Delay: '<S167>/Resettable Delay2' */
    Mean_Sample_Update_pre3 = rtb_SAT_Max_Delta_F_Cmd;

    /* Update for Delay: '<S167>/Resettable Delay3' */
    Mean_Sample_Update_pre4 = rtb_Tdf_min_steer_torque_class;

    /* Update for Delay: '<S167>/Resettable Delay5' */
    Lateral_Error_Mean_state = Lateral_Error_Mean;
  }

  /* End of Logic: '<S165>/OR' */
  /* End of Outputs for SubSystem: '<S165>/Mean_Of_All_Segments' */

  /* Abs: '<S148>/Abs3' */
  rtb_Switch_iy_idx_0 = fabsf(Lateral_Error_Mean);

  /* Abs: '<S148>/Abs1' incorporates:
   *  Abs: '<S198>/Absolute'
   *  Abs: '<S223>/Abs'
   *  Inport: '<Root>/Inport11'
   */
  rtb_DMC_Integtrator_Output_b = fabsf(VEH_Steer_Torque);

  /* Logic: '<S148>/NOT' */
  rtb_NOT_n = !rtb_SAC_Disable;

  /* RelationalOperator: '<S154>/Compare' incorporates:
   *  Abs: '<S148>/Abs'
   *  Abs: '<S148>/Abs1'
   *  Abs: '<S148>/Abs4'
   *  Constant: '<S148>/Constant'
   *  Constant: '<S148>/Constant1'
   *  Constant: '<S148>/Constant2'
   *  Constant: '<S148>/Constant3'
   *  Constant: '<S148>/Constant4'
   *  Constant: '<S148>/Constant5'
   *  Constant: '<S148>/Constant6'
   *  Constant: '<S148>/Constant7'
   *  Constant: '<S82>/Constant'
   *  Logic: '<S148>/AND'
   *  Logic: '<S148>/NOT'
   *  RelationalOperator: '<S148>/Less Than'
   *  RelationalOperator: '<S148>/Less Than1'
   *  RelationalOperator: '<S148>/Less Than2'
   *  RelationalOperator: '<S148>/Less Than3'
   *  RelationalOperator: '<S148>/Less Than4'
   *  RelationalOperator: '<S148>/Less Than5'
   *  RelationalOperator: '<S148>/Less Than6'
   *  RelationalOperator: '<S148>/Less Than7'
   *  RelationalOperator: '<S148>/Less Than8'
   *  UnitDelay: '<S147>/Unit Delay'
   *  UnitDelay: '<S149>/Unit Delay'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  rtb_LDP_Active = ((fabsf(Veh_Lat_Acc_Filt) < 3.5F) && rtb_NOT_n &&
                    (rtb_DMC_Integtrator_Output_b < 3.0F) && rtb_NotOperator2_0 &&
                    (rtb_Max_a < 0.01) && (rtb_VEH_Vehicle_Speed >=
    2.7777777777777777) && (rtb_VEH_Vehicle_Speed <= 50.0F) &&
                    Initialisation_Flag_state &&
                    (LAT_Stiffness_Request_Factor_pre >= 0.7) &&
                    (rtb_Switch_iy_idx_0 > 0.05F) && (rtb_Switch_iy_idx_0 <=
    3.0F) && (fabsf(VEH_Delta_F_Pre) <= 15.0F) && rtb_LDP_Active);

  /* RelationalOperator: '<S169>/Compare' incorporates:
   *  Constant: '<S169>/Constant'
   */
  rtb_Compare_hu = (Lateral_Error_Mean != 0.0F);

  /* Delay: '<S168>/Resettable Delay2' */
  if (rtb_OR) {
    Mean_kappa_command_sum = 0.0F;
  }

  /* Sum: '<S168>/Add2' incorporates:
   *  Delay: '<S168>/Resettable Delay2'
   */
  Mean_kappa_command_sum = Mean_kappa_command_sum +
    rtb_LAT_Kappa_Gradient_Fading_S;

  /* Switch: '<S168>/Switch3' incorporates:
   *  Delay: '<S168>/Resettable Delay2'
   *  Product: '<S168>/Divide1'
   *  UnitDelay: '<S168>/Unit Delay'
   *  UnitDelay: '<S168>/Unit Delay2'
   */
  if (New_Update_Aval) {
    Mean_kappa_command = Mean_kappa_command_sum / rtb_Gain_d;
  }

  /* End of Switch: '<S168>/Switch3' */

  /* Switch: '<S166>/Switch' incorporates:
   *  Constant: '<S166>/LeftTurnLeftError'
   *  Constant: '<S170>/Constant'
   *  Constant: '<S171>/Constant'
   *  Constant: '<S172>/Constant'
   *  Constant: '<S173>/Constant'
   *  Constant: '<S174>/Constant'
   *  Constant: '<S175>/Constant'
   *  Constant: '<S176>/Constant'
   *  Constant: '<S177>/Constant'
   *  Logic: '<S166>/AND'
   *  Logic: '<S166>/AND1'
   *  Logic: '<S166>/AND2'
   *  Logic: '<S166>/AND3'
   *  RelationalOperator: '<S170>/Compare'
   *  RelationalOperator: '<S171>/Compare'
   *  RelationalOperator: '<S172>/Compare'
   *  RelationalOperator: '<S173>/Compare'
   *  RelationalOperator: '<S174>/Compare'
   *  RelationalOperator: '<S175>/Compare'
   *  RelationalOperator: '<S176>/Compare'
   *  RelationalOperator: '<S177>/Compare'
   *  Switch: '<S166>/Switch1'
   *  Switch: '<S166>/Switch2'
   *  Switch: '<S166>/Switch3'
   *  UnitDelay: '<S168>/Unit Delay2'
   */
  if (rtb_Compare_hu && (CAM_Lateral_Error_Sign == -1.0F) && (Mean_kappa_command
       > 0.0F)) {
    rtb_Turn_And_Error_Side_f = 1U;
  } else if (rtb_Compare_hu && (CAM_Lateral_Error_Sign == 1.0F) &&
             (Mean_kappa_command > 0.0F)) {
    /* Switch: '<S166>/Switch1' incorporates:
     *  Constant: '<S166>/LeftTurnRightError'
     */
    rtb_Turn_And_Error_Side_f = 2U;
  } else if (rtb_Compare_hu && (CAM_Lateral_Error_Sign == -1.0F) &&
             (Mean_kappa_command < 0.0F)) {
    /* Switch: '<S166>/Switch2' incorporates:
     *  Constant: '<S166>/RightTurnLeftError'
     *  Switch: '<S166>/Switch1'
     */
    rtb_Turn_And_Error_Side_f = 3U;
  } else if (rtb_Compare_hu && (CAM_Lateral_Error_Sign == 1.0F) &&
             (Mean_kappa_command < 0.0F)) {
    /* Switch: '<S166>/Switch3' incorporates:
     *  Constant: '<S166>/RightTurnRightError'
     *  Switch: '<S166>/Switch1'
     *  Switch: '<S166>/Switch2'
     */
    rtb_Turn_And_Error_Side_f = 4U;
  } else {
    /* Switch: '<S166>/Switch2' incorporates:
     *  Constant: '<S166>/Constant4'
     *  Switch: '<S166>/Switch1'
     *  Switch: '<S166>/Switch3'
     */
    rtb_Turn_And_Error_Side_f = 0U;
  }

  /* End of Switch: '<S166>/Switch' */

  /* MinMax: '<S153>/Min' incorporates:
   *  Constant: '<S153>/Constant'
   *  Constant: '<S153>/Constant1'
   *  Constant: '<S153>/Constant2'
   *  MinMax: '<S153>/Max'
   *  Product: '<S153>/Product'
   */
  rtb_Switch_iy_idx_0 = fminf(fmaxf(rtb_Max_a * 120.0F, 0.0F), 1.0F);

  /* Product: '<S153>/Product1' */
  rtb_Subtract_jc = rtb_Switch_iy_idx_0;

  /* Product: '<S153>/Product2' incorporates:
   *  Constant: '<S153>/Constant4'
   *  Constant: '<S153>/Constant5'
   *  Sum: '<S153>/Subtract'
   */
  rtb_SAT_Max_Delta_F_Cmd = (1.0F - rtb_Switch_iy_idx_0) * 1.2F;

  /* Gain: '<S143>/Gain' */
  rtb_Switch_iy_idx_0 = 3.6F * rtb_VEH_Vehicle_Speed;

  /* DataTypeConversion: '<S153>/Data Type Conversion' incorporates:
   *  Lookup_n-D: '<S153>/1-D Lookup Table'
   *  Product: '<S153>/Product3'
   *  Sum: '<S153>/Add'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_Subtract_jc = fmodf(floorf((rtb_Subtract_jc + rtb_SAT_Max_Delta_F_Cmd) *
    look1_iflf_binlxpw(rtb_Switch_iy_idx_0, ((const real32_T *)
    &(Max_Wait_Counter_x[0])), ((const real32_T *)&(Max_Wait_Counter_y[0])), 8U)),
    65536.0F);
  rtb_Maximum_Waitiing_Counter = (uint16_T)(rtb_Subtract_jc < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-rtb_Subtract_jc : (int32_T)(uint16_T)
    rtb_Subtract_jc);

  /* Delay: '<S168>/Resettable Delay3' */
  if (rtb_OR) {
    Mean_Vehicle_Velocity_sum = 0.0F;
  }

  /* Sum: '<S168>/Add3' incorporates:
   *  Delay: '<S168>/Resettable Delay3'
   */
  Mean_Vehicle_Velocity_sum = Mean_Vehicle_Velocity_sum + rtb_VEH_Vehicle_Speed;

  /* Switch: '<S168>/Switch4' incorporates:
   *  Delay: '<S168>/Resettable Delay3'
   *  Product: '<S168>/Divide2'
   *  UnitDelay: '<S168>/Unit Delay'
   *  UnitDelay: '<S168>/Unit Delay3'
   */
  if (New_Update_Aval) {
    Mean_Vehicle_Velocity = Mean_Vehicle_Velocity_sum / rtb_Gain_d;
  }

  /* End of Switch: '<S168>/Switch4' */

  /* PreLookup: '<S151>/Prelookup' incorporates:
   *  Constant: '<S151>/Constant'
   *  Gain: '<S151>/Gain'
   *  UnitDelay: '<S168>/Unit Delay3'
   */
  rtb_SAC_Controller_Mode = plook_u32ff_binc(3.6F * Mean_Vehicle_Velocity, ((
    const real32_T *)&(SpeedSegmentLookup1[0])), 9U, &rtb_Subtract_jc);

  /* UnitDelay: '<S151>/Unit Delay2' */
  rtb_Gain_d = Last_Seg_Correction_Factor_Right_Scal_state;

  /* MultiPortSwitch: '<S151>/Index Vector1' incorporates:
   *  UnitDelay: '<S151>/Unit Delay2'
   */
  Last_Seg_Correction_Factor_Right_Scal_state =
    rtb_Switch_iu[rtb_SAC_Controller_Mode + 9U];

  /* RelationalOperator: '<S151>/Equal' incorporates:
   *  UnitDelay: '<S151>/Unit Delay'
   */
  rtb_OR = (rtb_SAC_Controller_Mode == Vehicle_Speed_Segment_state);

  /* MinMax: '<S152>/Max' incorporates:
   *  Abs: '<S152>/Abs'
   *  Constant: '<S82>/Constant'
   *  Sum: '<S152>/Subtract'
   */
  rtb_Subtract_jc = fmaxf(fabsf(Lateral_Error_Mean) - 0.05F, 0.05F);

  /* Product: '<S152>/Product7' incorporates:
   *  Constant: '<S152>/Constant6'
   *  Constant: '<S152>/Constant7'
   *  Lookup_n-D: '<S152>/1-D Lookup Table1'
   *  MinMax: '<S152>/Min1'
   *  Product: '<S152>/Product4'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_SAT_Max_Delta_F_Cmd = fminf(1.2F * rtb_Subtract_jc, 1.0F) *
    look1_iflf_binlxpw(rtb_Switch_iy_idx_0, ((const real32_T *)
    &(Adp_dyc_corr_fact_dec_x_sched[0])), ((const real32_T *)
    &(Adp_dyc_corr_fact_dec_y_sched[0])), 11U);

  /* Lookup_n-D: '<S152>/1-D Lookup Table' incorporates:
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = look1_iflf_binlxpw(rtb_Switch_iy_idx_0, ((const
    real32_T *)&(Adp_dyc_corr_fact_inc_x_sched[0])), ((const real32_T *)
    &(Adp_dyc_corr_fact_inc_y_sched[0])), 11U);

  /* Product: '<S152>/Product' incorporates:
   *  Constant: '<S152>/Constant1'
   */
  rtb_Subtract_jc *= 1.2F;

  /* MinMax: '<S152>/Min' incorporates:
   *  Constant: '<S152>/Constant2'
   */
  rtb_Subtract_jc = fminf(rtb_Subtract_jc, 1.0F);

  /* Product: '<S152>/Product3' incorporates:
   *  Product: '<S152>/Product1'
   */
  rtb_Tdf_min_steer_torque_class = rtb_Switch_iy_idx_0 * rtb_Subtract_jc;

  /* If: '<S144>/judgement_If&Else' incorporates:
   *  Constant: '<S144>/Max_Driver_Steering_Torque_Inner_lane'
   *  Constant: '<S144>/Max_Driver_Steering_Torque_Inner_lane1'
   *  Constant: '<S144>/Max_Driver_Steering_Torque_Outer_lane'
   *  Constant: '<S144>/Max_Driver_Steering_Torque_Outer_lane2'
   *  Constant: '<S155>/Constant'
   *  Constant: '<S156>/Constant'
   *  Constant: '<S157>/Constant'
   *  Constant: '<S158>/Constant'
   *  Inport: '<Root>/Inport11'
   *  Logic: '<S144>/AND'
   *  Logic: '<S144>/AND1'
   *  Logic: '<S144>/AND2'
   *  Logic: '<S144>/AND3'
   *  RelationalOperator: '<S144>/Less Than'
   *  RelationalOperator: '<S144>/Less Than1'
   *  RelationalOperator: '<S144>/Less Than2'
   *  RelationalOperator: '<S144>/Less Than3'
   *  RelationalOperator: '<S155>/Compare'
   *  RelationalOperator: '<S156>/Compare'
   *  RelationalOperator: '<S157>/Compare'
   *  RelationalOperator: '<S158>/Compare'
   */
  if (rtb_LDP_Active && (rtb_Turn_And_Error_Side_f == 1) && (VEH_Steer_Torque <
       2.5F)) {
    /* Outputs for IfAction SubSystem: '<S144>/Left_Turn_Left_Error' incorporates:
     *  ActionPort: '<S161>/Action Port'
     */
    /* Sum: '<S161>/Add' incorporates:
     *  Abs: '<S161>/Abs'
     *  Constant: '<S161>/Constant'
     *  Constant: '<S161>/Constant1'
     *  Logic: '<S161>/AND'
     *  RelationalOperator: '<S161>/LessThanOrEqual'
     *  RelationalOperator: '<S161>/LessThanOrEqual1'
     *  UnitDelay: '<S144>/Unit Delay3'
     *  UnitDelay: '<S168>/Unit Delay'
     *  UnitDelay: '<S5>/Unit Delay'
     */
    LTLE_Waiting_Counter_state = (uint16_T)((uint32_T)((Lateral_Error_Delta <=
      0.0F) && (fabsf(SAC_Control_Error_pre) < 0.04F) && New_Update_Aval) +
      LTLE_Waiting_Counter_state);

    /* Product: '<S161>/Product' incorporates:
     *  Constant: '<S161>/Constant5'
     *  UnitDelay: '<S151>/Unit Delay1'
     */
    rtb_Switch_iy_idx_0 = Last_Seg_Correction_Factor_Left_Scal_state * 1.05F;

    /* Switch: '<S161>/Switch1' incorporates:
     *  Logic: '<S161>/OR'
     *  MultiPortSwitch: '<S151>/Index Vector'
     *  RelationalOperator: '<S161>/GreaterThan'
     *  RelationalOperator: '<S161>/LessThanOrEqual2'
     *  Sum: '<S161>/Subtract'
     *  Switch: '<S161>/Switch2'
     *  Switch: '<S161>/Switch3'
     */
    if (LTLE_Waiting_Counter_state > rtb_Maximum_Waitiing_Counter) {
      /* Switch: '<S161>/Switch3' incorporates:
       *  Logic: '<S161>/OR'
       *  MultiPortSwitch: '<S151>/Index Vector'
       *  RelationalOperator: '<S161>/LessThanOrEqual2'
       */
      if (rtb_OR || (rtb_Switch_iu[rtb_SAC_Controller_Mode] <=
                     rtb_Switch_iy_idx_0)) {
        rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];
      }

      rtb_Switch_iy_idx_0 -= rtb_SAT_Max_Delta_F_Cmd;

      /* Sum: '<S161>/Add' incorporates:
       *  Constant: '<S161>/Constant4'
       *  Sum: '<S161>/Subtract'
       */
      LTLE_Waiting_Counter_state = 0U;
    } else {
      if (rtb_OR || (rtb_Switch_iu[rtb_SAC_Controller_Mode] <=
                     rtb_Switch_iy_idx_0)) {
        /* Switch: '<S161>/Switch3' incorporates:
         *  MultiPortSwitch: '<S151>/Index Vector'
         */
        rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];
      }

      /* MinMax: '<S161>/Min' */
      if (LTLE_Waiting_Counter_state >= rtb_Maximum_Waitiing_Counter) {
        /* Sum: '<S161>/Add' */
        LTLE_Waiting_Counter_state = rtb_Maximum_Waitiing_Counter;
      }

      /* End of MinMax: '<S161>/Min' */
    }

    /* End of Switch: '<S161>/Switch1' */

    /* Sum: '<S162>/Add' incorporates:
     *  Constant: '<S161>/Constant6'
     *  SignalConversion generated from: '<S161>/Outport3'
     */
    LTRE_Waiting_Counter_state = 0U;

    /* Sum: '<S163>/Add' incorporates:
     *  Constant: '<S161>/Constant7'
     *  SignalConversion generated from: '<S161>/Outport4'
     */
    RTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S164>/Add' incorporates:
     *  Constant: '<S161>/Constant8'
     *  SignalConversion generated from: '<S161>/Outport5'
     */
    RTRE_Waiting_Counter_state = 0U;

    /* SignalConversion: '<S161>/Signal Conversion' incorporates:
     *  UnitDelay: '<S151>/Unit Delay2'
     */
    rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;

    /* End of Outputs for SubSystem: '<S144>/Left_Turn_Left_Error' */
  } else if (rtb_LDP_Active && (rtb_Turn_And_Error_Side_f == 2) &&
             (VEH_Steer_Torque > -1.5F)) {
    /* Outputs for IfAction SubSystem: '<S144>/Left_Turn_Right_Error' incorporates:
     *  ActionPort: '<S162>/Action Port'
     */
    /* Sum: '<S162>/Add' incorporates:
     *  Abs: '<S162>/Abs'
     *  Constant: '<S162>/Constant'
     *  Constant: '<S162>/Constant1'
     *  Logic: '<S162>/AND'
     *  RelationalOperator: '<S162>/LessThanOrEqual'
     *  RelationalOperator: '<S162>/LessThanOrEqual1'
     *  UnitDelay: '<S144>/Unit Delay2'
     *  UnitDelay: '<S168>/Unit Delay'
     *  UnitDelay: '<S5>/Unit Delay'
     */
    LTRE_Waiting_Counter_state = (uint16_T)((uint32_T)((Lateral_Error_Delta >=
      0.0F) && (fabsf(SAC_Control_Error_pre) < 0.04F) && New_Update_Aval) +
      LTRE_Waiting_Counter_state);

    /* Product: '<S162>/Product' incorporates:
     *  Constant: '<S162>/Constant5'
     *  UnitDelay: '<S151>/Unit Delay1'
     */
    rtb_Switch_iy_idx_0 = Last_Seg_Correction_Factor_Left_Scal_state * 0.95F;

    /* Switch: '<S162>/Switch1' incorporates:
     *  Logic: '<S162>/OR'
     *  MultiPortSwitch: '<S151>/Index Vector'
     *  RelationalOperator: '<S162>/GreaterThan'
     *  RelationalOperator: '<S162>/LessThanOrEqual2'
     *  Sum: '<S162>/Subtract'
     *  Switch: '<S162>/Switch2'
     *  Switch: '<S162>/Switch3'
     */
    if (LTRE_Waiting_Counter_state > rtb_Maximum_Waitiing_Counter) {
      /* Switch: '<S162>/Switch3' incorporates:
       *  Logic: '<S162>/OR'
       *  MultiPortSwitch: '<S151>/Index Vector'
       *  RelationalOperator: '<S162>/LessThanOrEqual2'
       */
      if (rtb_OR || (rtb_Switch_iu[rtb_SAC_Controller_Mode] >=
                     rtb_Switch_iy_idx_0)) {
        rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];
      }

      rtb_Switch_iy_idx_0 += rtb_Tdf_min_steer_torque_class;

      /* Sum: '<S162>/Add' incorporates:
       *  Constant: '<S162>/Constant4'
       *  Sum: '<S162>/Subtract'
       */
      LTRE_Waiting_Counter_state = 0U;
    } else {
      if (rtb_OR || (rtb_Switch_iu[rtb_SAC_Controller_Mode] >=
                     rtb_Switch_iy_idx_0)) {
        /* Switch: '<S162>/Switch3' incorporates:
         *  MultiPortSwitch: '<S151>/Index Vector'
         */
        rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];
      }

      /* MinMax: '<S162>/Min' */
      if (LTRE_Waiting_Counter_state >= rtb_Maximum_Waitiing_Counter) {
        /* Sum: '<S162>/Add' */
        LTRE_Waiting_Counter_state = rtb_Maximum_Waitiing_Counter;
      }

      /* End of MinMax: '<S162>/Min' */
    }

    /* End of Switch: '<S162>/Switch1' */

    /* Sum: '<S161>/Add' incorporates:
     *  Constant: '<S162>/Constant6'
     *  SignalConversion generated from: '<S162>/Outport3'
     */
    LTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S163>/Add' incorporates:
     *  Constant: '<S162>/Constant7'
     *  SignalConversion generated from: '<S162>/Outport4'
     */
    RTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S164>/Add' incorporates:
     *  Constant: '<S162>/Constant8'
     *  SignalConversion generated from: '<S162>/Outport5'
     */
    RTRE_Waiting_Counter_state = 0U;

    /* SignalConversion: '<S162>/Signal Conversion' incorporates:
     *  UnitDelay: '<S151>/Unit Delay2'
     */
    rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;

    /* End of Outputs for SubSystem: '<S144>/Left_Turn_Right_Error' */
  } else if (rtb_LDP_Active && (rtb_Turn_And_Error_Side_f == 3) &&
             (VEH_Steer_Torque < 1.5F)) {
    /* Outputs for IfAction SubSystem: '<S144>/Right_Turn_Left_Error' incorporates:
     *  ActionPort: '<S163>/Action Port'
     */
    /* Sum: '<S163>/Add' incorporates:
     *  Abs: '<S163>/Abs'
     *  Constant: '<S163>/Constant'
     *  Constant: '<S163>/Constant1'
     *  Logic: '<S163>/AND'
     *  RelationalOperator: '<S163>/LessThanOrEqual'
     *  RelationalOperator: '<S163>/LessThanOrEqual1'
     *  UnitDelay: '<S144>/Unit Delay1'
     *  UnitDelay: '<S168>/Unit Delay'
     *  UnitDelay: '<S5>/Unit Delay'
     */
    RTLE_Waiting_Counter_state = (uint16_T)((uint32_T)((Lateral_Error_Delta <=
      0.0F) && (fabsf(SAC_Control_Error_pre) < 0.04F) && New_Update_Aval) +
      RTLE_Waiting_Counter_state);

    /* Product: '<S163>/Product' incorporates:
     *  Constant: '<S163>/Constant5'
     */
    rtb_Subtract_jc = rtb_Gain_d * 0.95F;

    /* Switch: '<S163>/Switch1' incorporates:
     *  Logic: '<S163>/OR'
     *  RelationalOperator: '<S163>/GreaterThan'
     *  RelationalOperator: '<S163>/LessThanOrEqual2'
     *  Sum: '<S163>/Subtract'
     *  Switch: '<S163>/Switch2'
     *  Switch: '<S163>/Switch3'
     *  UnitDelay: '<S151>/Unit Delay2'
     */
    if (RTLE_Waiting_Counter_state > rtb_Maximum_Waitiing_Counter) {
      /* Switch: '<S163>/Switch3' incorporates:
       *  Logic: '<S163>/OR'
       *  RelationalOperator: '<S163>/LessThanOrEqual2'
       *  UnitDelay: '<S151>/Unit Delay2'
       */
      if (rtb_OR || (Last_Seg_Correction_Factor_Right_Scal_state >=
                     rtb_Subtract_jc)) {
        rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;
      }

      rtb_Subtract_jc += rtb_Tdf_min_steer_torque_class;

      /* Sum: '<S163>/Add' incorporates:
       *  Constant: '<S163>/Constant4'
       *  Sum: '<S163>/Subtract'
       */
      RTLE_Waiting_Counter_state = 0U;
    } else {
      if (rtb_OR || (Last_Seg_Correction_Factor_Right_Scal_state >=
                     rtb_Subtract_jc)) {
        /* Switch: '<S163>/Switch3' incorporates:
         *  UnitDelay: '<S151>/Unit Delay2'
         */
        rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;
      }

      /* MinMax: '<S163>/Min' */
      if (RTLE_Waiting_Counter_state >= rtb_Maximum_Waitiing_Counter) {
        /* Sum: '<S163>/Add' */
        RTLE_Waiting_Counter_state = rtb_Maximum_Waitiing_Counter;
      }

      /* End of MinMax: '<S163>/Min' */
    }

    /* End of Switch: '<S163>/Switch1' */

    /* Sum: '<S161>/Add' incorporates:
     *  Constant: '<S163>/Constant6'
     *  SignalConversion generated from: '<S163>/Outport3'
     */
    LTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S162>/Add' incorporates:
     *  Constant: '<S163>/Constant7'
     *  SignalConversion generated from: '<S163>/Outport4'
     */
    LTRE_Waiting_Counter_state = 0U;

    /* Sum: '<S164>/Add' incorporates:
     *  Constant: '<S163>/Constant8'
     *  SignalConversion generated from: '<S163>/Outport5'
     */
    RTRE_Waiting_Counter_state = 0U;

    /* SignalConversion: '<S163>/Signal Conversion' incorporates:
     *  MultiPortSwitch: '<S151>/Index Vector'
     */
    rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];

    /* End of Outputs for SubSystem: '<S144>/Right_Turn_Left_Error' */
  } else if (rtb_LDP_Active && (rtb_Turn_And_Error_Side_f == 3) &&
             (VEH_Steer_Torque < -2.5F)) {
    /* Outputs for IfAction SubSystem: '<S144>/Right_Turn_Right_Error' incorporates:
     *  ActionPort: '<S164>/Action Port'
     */
    /* Sum: '<S164>/Add' incorporates:
     *  Abs: '<S164>/Abs'
     *  Constant: '<S164>/Constant'
     *  Constant: '<S164>/Constant1'
     *  Logic: '<S164>/AND'
     *  RelationalOperator: '<S164>/LessThanOrEqual'
     *  RelationalOperator: '<S164>/LessThanOrEqual1'
     *  UnitDelay: '<S144>/Unit Delay'
     *  UnitDelay: '<S168>/Unit Delay'
     *  UnitDelay: '<S5>/Unit Delay'
     */
    RTRE_Waiting_Counter_state = (uint16_T)((uint32_T)((Lateral_Error_Delta >=
      0.0F) && (fabsf(SAC_Control_Error_pre) < 0.04F) && New_Update_Aval) +
      RTRE_Waiting_Counter_state);

    /* Product: '<S164>/Product' incorporates:
     *  Constant: '<S164>/Constant5'
     */
    rtb_Subtract_jc = rtb_Gain_d * 1.05F;

    /* Switch: '<S164>/Switch1' incorporates:
     *  Logic: '<S164>/OR'
     *  RelationalOperator: '<S164>/GreaterThan'
     *  RelationalOperator: '<S164>/LessThanOrEqual2'
     *  Sum: '<S164>/Subtract'
     *  Switch: '<S164>/Switch2'
     *  Switch: '<S164>/Switch3'
     *  UnitDelay: '<S151>/Unit Delay2'
     */
    if (RTRE_Waiting_Counter_state > rtb_Maximum_Waitiing_Counter) {
      /* Switch: '<S164>/Switch3' incorporates:
       *  Logic: '<S164>/OR'
       *  RelationalOperator: '<S164>/LessThanOrEqual2'
       *  UnitDelay: '<S151>/Unit Delay2'
       */
      if (rtb_OR || (Last_Seg_Correction_Factor_Right_Scal_state <=
                     rtb_Subtract_jc)) {
        rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;
      }

      rtb_Subtract_jc -= rtb_SAT_Max_Delta_F_Cmd;

      /* Sum: '<S164>/Add' incorporates:
       *  Constant: '<S164>/Constant4'
       *  Sum: '<S164>/Subtract'
       */
      RTRE_Waiting_Counter_state = 0U;
    } else {
      if (rtb_OR || (Last_Seg_Correction_Factor_Right_Scal_state <=
                     rtb_Subtract_jc)) {
        /* Switch: '<S164>/Switch3' incorporates:
         *  UnitDelay: '<S151>/Unit Delay2'
         */
        rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;
      }

      /* MinMax: '<S164>/Min' */
      if (RTRE_Waiting_Counter_state >= rtb_Maximum_Waitiing_Counter) {
        /* Sum: '<S164>/Add' */
        RTRE_Waiting_Counter_state = rtb_Maximum_Waitiing_Counter;
      }

      /* End of MinMax: '<S164>/Min' */
    }

    /* End of Switch: '<S164>/Switch1' */

    /* Sum: '<S161>/Add' incorporates:
     *  Constant: '<S164>/Constant6'
     *  SignalConversion generated from: '<S164>/Outport3'
     */
    LTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S162>/Add' incorporates:
     *  Constant: '<S164>/Constant7'
     *  SignalConversion generated from: '<S164>/Outport4'
     */
    LTRE_Waiting_Counter_state = 0U;

    /* Sum: '<S163>/Add' incorporates:
     *  Constant: '<S164>/Constant8'
     *  SignalConversion generated from: '<S164>/Outport5'
     */
    RTLE_Waiting_Counter_state = 0U;

    /* SignalConversion: '<S164>/Signal Conversion' incorporates:
     *  MultiPortSwitch: '<S151>/Index Vector'
     */
    rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];

    /* End of Outputs for SubSystem: '<S144>/Right_Turn_Right_Error' */
  } else {
    /* Outputs for IfAction SubSystem: '<S144>/Default' incorporates:
     *  ActionPort: '<S160>/Action Port'
     */
    /* Sum: '<S161>/Add' incorporates:
     *  Constant: '<S160>/Constant2'
     *  SignalConversion generated from: '<S160>/Outport3'
     */
    LTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S162>/Add' incorporates:
     *  Constant: '<S160>/Constant1'
     *  SignalConversion generated from: '<S160>/Outport4'
     */
    LTRE_Waiting_Counter_state = 0U;

    /* Sum: '<S163>/Add' incorporates:
     *  Constant: '<S160>/Constant7'
     *  SignalConversion generated from: '<S160>/Outport5'
     */
    RTLE_Waiting_Counter_state = 0U;

    /* Sum: '<S164>/Add' incorporates:
     *  Constant: '<S160>/Constant8'
     *  SignalConversion generated from: '<S160>/Outport6'
     */
    RTRE_Waiting_Counter_state = 0U;

    /* SignalConversion: '<S160>/Signal Conversion1' incorporates:
     *  MultiPortSwitch: '<S151>/Index Vector'
     */
    rtb_Switch_iy_idx_0 = rtb_Switch_iu[rtb_SAC_Controller_Mode];

    /* SignalConversion: '<S160>/Signal Conversion' incorporates:
     *  UnitDelay: '<S151>/Unit Delay2'
     */
    rtb_Subtract_jc = Last_Seg_Correction_Factor_Right_Scal_state;

    /* End of Outputs for SubSystem: '<S144>/Default' */
  }

  /* End of If: '<S144>/judgement_If&Else' */

  /* MinMax: '<S159>/Max' incorporates:
   *  Constant: '<S159>/Constant'
   */
  rtb_Switch_iy_idx_0 = fmaxf(rtb_Switch_iy_idx_0,
    Adp_dyc_corr_factor_lower_limit);
  for (i = 0; i < 9; i++) {
    /* Assignment: '<S150>/Assignment' incorporates:
     *  UnitDelay: '<S82>/Unit Delay'
     *  UnitDelay: '<S82>/Unit Delay1'
     */
    Corretion_Factor_Left_Vect[i] = Corretion_Factor_Right_Vect[i];

    /* SignalConversion generated from: '<S150>/Assignment1' incorporates:
     *  UnitDelay: '<S82>/Unit Delay'
     */
    Corretion_Factor_Right_Vect[i] = rtb_Switch_iu[i + 9];
  }

  /* Assignment: '<S150>/Assignment' incorporates:
   *  Constant: '<S159>/Constant1'
   *  MinMax: '<S159>/Max2'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  Corretion_Factor_Left_Vect[(int32_T)rtb_SAC_Controller_Mode] = fminf
    (rtb_Switch_iy_idx_0, Adp_dyc_corr_factor_upper_limit);

  /* MinMax: '<S159>/Max1' incorporates:
   *  Constant: '<S159>/Constant'
   */
  rtb_Subtract_jc = fmaxf(Adp_dyc_corr_factor_lower_limit, rtb_Subtract_jc);

  /* MinMax: '<S159>/Max3' incorporates:
   *  Constant: '<S159>/Constant1'
   */
  LAT_Oc_Kappa_Active_tmp = rtb_SAC_Controller_Mode;
  if (rtb_SAC_Controller_Mode > 2147483647U) {
    LAT_Oc_Kappa_Active_tmp = 2147483647U;
  }

  Corretion_Factor_Right_Vect[(int32_T)LAT_Oc_Kappa_Active_tmp] = fminf
    (Adp_dyc_corr_factor_upper_limit, rtb_Subtract_jc);

  /* End of MinMax: '<S159>/Max3' */

  /* Sum: '<S181>/Add' incorporates:
   *  UnitDelay: '<S181>/Unit Delay'
   */
  ADP_Dyc_Corr_Factor = ADP_Dyc_Corr_Factor_state;

  /* Switch: '<S150>/Switch' incorporates:
   *  Assignment: '<S150>/Assignment'
   *  Assignment: '<S150>/Assignment1'
   *  Constant: '<S180>/Constant'
   *  Interpolation_n-D: '<S150>/Interpolation Using Prelookup'
   *  Interpolation_n-D: '<S150>/Interpolation Using Prelookup1'
   *  RelationalOperator: '<S180>/Compare'
   *  UnitDelay: '<S82>/Unit Delay'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  if (rtb_LAT_Kappa_Gradient_Fading_S > 0.0F) {
    rtb_Switch_iy_idx_0 = intrp1d_fu32fl_pw(rtb_Vehicle_Speed_Index,
      rtb_Hec_r_factor, (&(Corretion_Factor_Left_Vect[0])));
  } else {
    rtb_Switch_iy_idx_0 = intrp1d_fu32fl_pw(rtb_Vehicle_Speed_Index,
      rtb_Hec_r_factor, (&(Corretion_Factor_Right_Vect[0])));
  }

  /* End of Switch: '<S150>/Switch' */

  /* Sum: '<S181>/Subtract' */
  rtb_Switch_iy_idx_0 -= ADP_Dyc_Corr_Factor;

  /* Signum: '<S181>/Sign' */
  if (rtb_Switch_iy_idx_0 < 0.0F) {
    rtb_Subtract_jc = -1.0F;
  } else if (rtb_Switch_iy_idx_0 > 0.0F) {
    rtb_Subtract_jc = 1.0F;
  } else {
    rtb_Subtract_jc = rtb_Switch_iy_idx_0;
  }

  /* End of Signum: '<S181>/Sign' */

  /* Abs: '<S181>/Abs' */
  rtb_Switch_iy_idx_0 = fabsf(rtb_Switch_iy_idx_0);

  /* Sum: '<S181>/Add' incorporates:
   *  Constant: '<S181>/Constant'
   *  Inport: '<Root>/Inport13'
   *  MinMax: '<S181>/Min'
   *  Product: '<S181>/Product'
   *  Product: '<S181>/Product1'
   */
  ADP_Dyc_Corr_Factor = fminf(rtb_Switch_iy_idx_0, 10.0F * VEH_cycletime) *
    rtb_Subtract_jc + ADP_Dyc_Corr_Factor;

  /* Switch: '<S109>/Switch' incorporates:
   *  Constant: '<S109>/Constant'
   *  Constant: '<S122>/Constant'
   *  Constant: '<S126>/Constant1'
   *  DiscreteIntegrator: '<S126>/Discrete-Time Integrator'
   *  Inport: '<Root>/Inport26'
   *  Product: '<S126>/Product1'
   *  RelationalOperator: '<S122>/Compare'
   *  S-Function (sfix_bitop): '<S109>/Bitwise AND2'
   *  Sum: '<S126>/Add'
   */
  if ((Dyc_compensation_mode_par & 256) == 256) {
    rtb_Subtract_jc = 0.0F;
  } else {
    rtb_Subtract_jc = DYC_Filter_Tau * rtb_Abs1_j +
      DYC_Boost_Filter_Output_state;
  }

  /* End of Switch: '<S109>/Switch' */

  /* Switch: '<S110>/Switch' incorporates:
   *  Constant: '<S110>/Constant'
   *  Constant: '<S123>/Constant'
   *  Inport: '<Root>/Inport26'
   *  RelationalOperator: '<S123>/Compare'
   *  S-Function (sfix_bitop): '<S110>/Bitwise AND2'
   */
  if ((Dyc_compensation_mode_par & 64) == 64) {
    rtb_Switch_iy_idx_0 = ADP_Dyc_Corr_Factor;
  } else {
    rtb_Switch_iy_idx_0 = 1.0F;
  }

  /* End of Switch: '<S110>/Switch' */

  /* Product: '<S87>/Product5' incorporates:
   *  Constant: '<S87>/Constant'
   *  Product: '<S87>/Product3'
   *  Product: '<S87>/Product4'
   *  Sum: '<S87>/Add'
   *  Sum: '<S87>/Add1'
   */
  DYC_Steer_Angle_Feedforward = ((rtb_LAT_Delta_F_cmd * DYC_Filter_Kappa_Command
    + HEC_Yaw_Rate_Filter * Dyc_kappa_angle_lpf_corr_factor) + rtb_Subtract_jc) *
    rtb_Switch_iy_idx_0;

  /* UnitDelay: '<S128>/Unit Delay1' */
  rtb_Hec_r_factor = LKC_Delta_Psi_pre1;

  /* UnitDelay: '<S128>/Unit Delay2' */
  rtb_Gain_d = LKC_Delta_Psi_pre2;

  /* UnitDelay: '<S128>/Unit Delay3' */
  rtb_SAT_Max_Delta_F_Cmd = LKC_Delta_Psi_pre3;

  /* UnitDelay: '<S128>/Unit Delay4' */
  rtb_Tdf_min_steer_torque_class = LKC_Delta_Psi_pre4;

  /* UnitDelay: '<S128>/Unit Delay5' */
  rtb_UnitDelay5 = LKC_Delta_Psi_pre5;

  /* UnitDelay: '<S128>/Unit Delay6' */
  rtb_UnitDelay6 = LKC_Delta_Psi_pre6;

  /* UnitDelay: '<S128>/Unit Delay7' */
  rtb_Product_lg = LKC_Delta_Psi_pre7;

  /* UnitDelay: '<S128>/Unit Delay8' */
  rtb_Add_hp5 = LKC_Delta_Psi_pre8;

  /* MultiPortSwitch: '<S128>/Multiport Switch' incorporates:
   *  Constant: '<S128>/Constant'
   *  Constant: '<S128>/Constant1'
   *  Constant: '<S128>/Sac_delta_psi_dot_maf_length'
   *  Gain: '<S128>/Gain1'
   *  Gain: '<S128>/Gain2'
   *  Gain: '<S128>/Gain3'
   *  Gain: '<S128>/Gain4'
   *  Gain: '<S128>/Gain5'
   *  Gain: '<S128>/Gain6'
   *  Gain: '<S128>/Gain7'
   *  Gain: '<S128>/Gain8'
   *  Gain: '<S128>/Gain9'
   *  Inport: '<Root>/LKC_Delta_Psi'
   *  MinMax: '<S128>/MinMax'
   *  Sum: '<S128>/Add11'
   *  Sum: '<S128>/Add12'
   *  Sum: '<S128>/Add2'
   *  Sum: '<S128>/Add3'
   *  Sum: '<S128>/Add4'
   *  Sum: '<S128>/Add5'
   *  Sum: '<S128>/Add6'
   *  Sum: '<S128>/Add7'
   *  Sum: '<S128>/Add8'
   *  Sum: '<S128>/Add9'
   *  UnitDelay: '<S128>/Unit Delay1'
   *  UnitDelay: '<S128>/Unit Delay2'
   *  UnitDelay: '<S128>/Unit Delay3'
   *  UnitDelay: '<S128>/Unit Delay4'
   *  UnitDelay: '<S128>/Unit Delay5'
   *  UnitDelay: '<S128>/Unit Delay6'
   *  UnitDelay: '<S128>/Unit Delay7'
   *  UnitDelay: '<S128>/Unit Delay8'
   *  UnitDelay: '<S128>/Unit Delay9'
   */
  switch ((int32_T)fminf(Sac_delta_psi_dot_maf_length + 1.0F, 10.0F)) {
   case 1:
    rtb_DMC_Integtrator_Output_ks = LKC_Delta_Psi;
    break;

   case 2:
    rtb_DMC_Integtrator_Output_ks = (LKC_Delta_Psi + LKC_Delta_Psi_pre1) * 0.5F;
    break;

   case 3:
    rtb_DMC_Integtrator_Output_ks = ((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) * 0.33F;
    break;

   case 4:
    rtb_DMC_Integtrator_Output_ks = (((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) * 0.25F;
    break;

   case 5:
    rtb_DMC_Integtrator_Output_ks = ((((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) * 0.2F;
    break;

   case 6:
    rtb_DMC_Integtrator_Output_ks = (((((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) +
      LKC_Delta_Psi_pre5) * 0.167F;
    break;

   case 7:
    rtb_DMC_Integtrator_Output_ks = ((((((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) +
      LKC_Delta_Psi_pre5) + LKC_Delta_Psi_pre6) * 0.142F;
    break;

   case 8:
    rtb_DMC_Integtrator_Output_ks = (((((((LKC_Delta_Psi + LKC_Delta_Psi_pre1) +
      LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) +
      LKC_Delta_Psi_pre5) + LKC_Delta_Psi_pre6) + LKC_Delta_Psi_pre7) * 0.125F;
    break;

   case 9:
    rtb_DMC_Integtrator_Output_ks = ((((((((LKC_Delta_Psi + LKC_Delta_Psi_pre1)
      + LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) +
      LKC_Delta_Psi_pre5) + LKC_Delta_Psi_pre6) + LKC_Delta_Psi_pre7) +
      LKC_Delta_Psi_pre8) * 0.111F;
    break;

   default:
    rtb_DMC_Integtrator_Output_ks = (((((((((LKC_Delta_Psi + LKC_Delta_Psi_pre1)
      + LKC_Delta_Psi_pre2) + LKC_Delta_Psi_pre3) + LKC_Delta_Psi_pre4) +
      LKC_Delta_Psi_pre5) + LKC_Delta_Psi_pre6) + LKC_Delta_Psi_pre7) +
      LKC_Delta_Psi_pre8) + LKC_Delta_Psi_pre9) * 0.1F;
    break;
  }

  /* End of MultiPortSwitch: '<S128>/Multiport Switch' */

  /* Sum: '<S130>/Add' incorporates:
   *  Constant: '<S128>/Constant2'
   *  Constant: '<S128>/Sac_delta_psi_pt1_filter_coeff'
   *  Gain: '<S128>/Sac_one_by_ts'
   *  Product: '<S128>/Divide'
   *  Product: '<S130>/Product'
   *  Sum: '<S128>/Add1'
   *  Sum: '<S128>/Add10'
   *  Sum: '<S130>/Add1'
   *  UnitDelay: '<S128>/Unit Delay'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  SAC_Delta_Psi_Dot_state = ((rtb_DMC_Integtrator_Output_ks -
    SAC_Delta_Psi_state) * Sac_one_by_ts - SAC_Delta_Psi_Dot_state) * (Sac_ts /
    (Sac_ts + Sac_delta_psi_pt1_filter_coeff)) + SAC_Delta_Psi_Dot_state;

  /* Sum: '<S129>/Add' incorporates:
   *  Constant: '<S79>/Constant1'
   *  Constant: '<S79>/Sac_osd_delta_ys_dot_coeff'
   *  Gain: '<S79>/Sac_one_by_ts1'
   *  Inport: '<Root>/LKC_Delta_Ys'
   *  Product: '<S129>/Product'
   *  Product: '<S79>/Divide'
   *  Sum: '<S129>/Add1'
   *  Sum: '<S79>/Add10'
   *  Sum: '<S79>/Add2'
   *  UnitDelay: '<S129>/Unit Delay'
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  SAC_Pt1_Filter1_state = ((LKC_Delta_Ys - LKC_Delta_Ys_pre) * Sac_one_by_ts -
    SAC_Pt1_Filter1_state) * (Sac_ts / (Sac_ts + Sac_osd_delta_ys_dot_coeff)) +
    SAC_Pt1_Filter1_state;

  /* Sum: '<S79>/Add1' incorporates:
   *  Gain: '<S127>/Gain4'
   *  Gain: '<S127>/Sac_delta_psi_dot_factor'
   *  Gain: '<S79>/Sac_osd_delta_ys_dot_feedback'
   *  Lookup_n-D: '<S127>/HEC_Kd_Scheduling'
   *  Product: '<S79>/Product4'
   *  UnitDelay: '<S129>/Unit Delay'
   *  UnitDelay: '<S130>/Unit Delay'
   */
  SAC_Compensation_Angle_Command = look1_iflf_binlxpw(3.6F *
    rtb_VEH_Vehicle_Speed, ((const real32_T *)&(HEC_Kd_X_Scheduling[0])), ((
    const real32_T *)&(HEC_Kd_Y_Scheduling[0])), 10U) * Sac_delta_psi_dot_factor
    * SAC_Delta_Psi_Dot_state + Sac_osd_delta_ys_dot_feedback *
    SAC_Pt1_Filter1_state;

  /* Switch: '<S133>/Switch4' incorporates:
   *  Constant: '<S133>/Constant'
   *  Constant: '<S80>/SAC_Yrc_Integrator_Limit'
   */
  if (rtb_SAC_Disable) {
    rtb_Subtract_jc = 0.0F;
  } else {
    rtb_Subtract_jc = 2.0F;
  }

  /* End of Switch: '<S133>/Switch4' */

  /* MinMax: '<S133>/MinMax2' incorporates:
   *  Gain: '<S133>/Gain'
   *  Gain: '<S80>/Sac_ts'
   *  Gain: '<S80>/Sac_yrc_ki_gain'
   *  MinMax: '<S133>/MinMax'
   *  Sum: '<S80>/Add1'
   *  UnitDelay: '<S80>/Unit Delay1'
   */
  SAC_Integrator_Sat_Out = fmaxf(fminf(Sac_yrc_ki_gain * SAC_Yrc_Control_Error *
    Sac_ts + SAC_Integrator_Sat_Out, rtb_Subtract_jc), -rtb_Subtract_jc);

  /* Outputs for Enabled SubSystem: '<S149>/Initial' incorporates:
   *  EnablePort: '<S179>/Enable'
   */
  /* SignalConversion generated from: '<S179>/Outport' incorporates:
   *  Logic: '<S149>/NOT'
   *  UnitDelay: '<S149>/Unit Delay'
   */
  Initialisation_Flag = ((!Initialisation_Flag_state) || Initialisation_Flag);

  /* End of Outputs for SubSystem: '<S149>/Initial' */

  /* Logic: '<S198>/Logical Operator3' incorporates:
   *  Inport: '<Root>/Inport71'
   *  RelationalOperator: '<S198>/Relational Operator'
   */
  LAT_Oc_High_Driver_Torque = (rtb_DMC_Integtrator_Output_b >=
    Lat_oc_max_driver_torque_par);

  /* S-Function (sfix_bitop): '<S198>/Bitwise Operator' incorporates:
   *  Inport: '<Root>/Inport21'
   *  S-Function (sfix_bitop): '<S207>/Bitwise Operator'
   */
  LAT_Oc_Kappa_Active_tmp = Dmc_configuration_mode_par & 128U;

  /* Logic: '<S198>/Logical Operator4' incorporates:
   *  Logic: '<S198>/Logical Operator2'
   *  S-Function (sfix_bitop): '<S198>/Bitwise Operator'
   */
  LAT_Oc_Kappa_Active = ((!rtb_NOT_n) || (LAT_Oc_Kappa_Active_tmp != 0U) ||
    LAT_Oc_High_Driver_Torque);

  /* RelationalOperator: '<S212>/Relational Operator' incorporates:
   *  Constant: '<S212>/Lat_oc_min_velocity_dys'
   *  Gain: '<S212>/Gain'
   */
  LAT_Oc_Min_Veh_Vel_Flag = (3.6F * rtb_VEH_Vehicle_Speed <
    Lat_oc_min_velocity_dys);

  /* Sum: '<S231>/Add' incorporates:
   *  Inport: '<Root>/Inport13'
   *  Inport: '<Root>/Inport66'
   *  Product: '<S231>/Divide'
   *  Product: '<S231>/Product'
   *  Sum: '<S231>/Add1'
   *  Sum: '<S231>/Sub'
   *  UnitDelay: '<S194>/Unit Delay'
   *  UnitDelay: '<S231>/Unit Delay'
   */
  LAT_Oc_Filtered_Kappa_Cam = LAT_Kappa_Command_state;
  LAT_Oc_Filtered_Kappa_Cam = VEH_cycletime / (VEH_cycletime +
    Lat_oc_kappa_cmd_filter_coeff_par) * (LAT_Kappa_Command_pre -
    LAT_Oc_Filtered_Kappa_Cam) + LAT_Oc_Filtered_Kappa_Cam;

  /* Sum: '<S213>/Add' incorporates:
   *  Constant: '<S213>/Lat_oc_kappa_ffwd_filter_coeff'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S213>/Divide'
   *  Product: '<S213>/Product'
   *  Sum: '<S213>/Add1'
   *  Sum: '<S213>/Sub'
   *  UnitDelay: '<S194>/Unit Delay'
   *  UnitDelay: '<S213>/Unit Delay'
   */
  LAT_Kappa_Command_filt_state1 = VEH_cycletime / (VEH_cycletime +
    Lat_oc_kappa_ffwd_filter_coeff) * (LAT_Kappa_Command_pre -
    LAT_Kappa_Command_filt_state1) + LAT_Kappa_Command_filt_state1;

  /* Abs: '<S211>/Abs' incorporates:
   *  Abs: '<S221>/Abs'
   */
  rtb_Switch_iy_idx_0 = fabsf(LAT_Oc_Filtered_Kappa_Cam);

  /* Logic: '<S211>/Logical Operator' incorporates:
   *  Abs: '<S211>/Abs'
   *  Abs: '<S211>/Abs1'
   *  Inport: '<Root>/Inport67'
   *  RelationalOperator: '<S211>/Relational Operator'
   *  RelationalOperator: '<S211>/Relational Operator1'
   *  UnitDelay: '<S213>/Unit Delay'
   */
  LAT_Oc_Max_Flt_Kappa_Cmd_Flag = ((rtb_Switch_iy_idx_0 >
    Lat_oc_max_kappa_dys_par) || (Lat_oc_max_kappa_dys_par < fabsf
    (LAT_Kappa_Command_filt_state1)));

  /* RelationalOperator: '<S210>/Relational Operator' incorporates:
   *  Constant: '<S210>/Lat_oc_implaus_lateral_error'
   *  Inport: '<Root>/LKC_Delta_Ys'
   */
  LAT_Oc_Implaus_Lateral_Error = (LKC_Delta_Ys > Lat_oc_implaus_lateral_error);

  /* Logic: '<S207>/Or Operator' */
  LAT_Oc_Disable_Flag = ((LAT_Oc_Kappa_Active_tmp != 0U) ||
    LAT_Oc_Min_Veh_Vel_Flag || LAT_Oc_Max_Flt_Kappa_Cmd_Flag || (!rtb_NOT_n) ||
    LAT_Oc_Implaus_Lateral_Error);

  /* Switch: '<S204>/Switch' incorporates:
   *  Constant: '<S204>/Constant1'
   *  Constant: '<S204>/Constant2'
   *  Sum: '<S204>/Add'
   *  UnitDelay: '<S204>/Unit Delay'
   */
  if (LAT_Oc_Disable_Flag) {
    Lat_oc_minimum_latency_state = 0.0F;
  } else {
    Lat_oc_minimum_latency_state = rtb_MinMax + 1.0F;
  }

  /* End of Switch: '<S204>/Switch' */

  /* Switch: '<S205>/Switch' incorporates:
   *  Constant: '<S205>/Constant1'
   *  Constant: '<S205>/Constant2'
   *  Sum: '<S205>/Add'
   *  UnitDelay: '<S205>/Unit Delay'
   */
  if (LAT_Oc_High_Driver_Torque) {
    Lat_oc_minimum_latency_shrt_state = 0.0F;
  } else {
    Lat_oc_minimum_latency_shrt_state = rtb_MinMax_k + 1.0F;
  }

  /* End of Switch: '<S205>/Switch' */

  /* Switch: '<S206>/Switch' incorporates:
   *  Constant: '<S206>/Lat_oc_fast_ki'
   *  Constant: '<S206>/Lat_oc_max_lateral_error'
   *  Inport: '<Root>/Inport68'
   *  RelationalOperator: '<S206>/Relational Operator'
   */
  if (rtb_Product4_b > Lat_oc_max_lateral_error) {
    rtb_Product4_b = Lat_oc_fast_ki;
  } else {
    rtb_Product4_b = Lat_oc_ki_par;
  }

  /* End of Switch: '<S206>/Switch' */

  /* Switch: '<S208>/Switch' incorporates:
   *  Constant: '<S208>/Constant'
   *  Inport: '<Root>/LKC_Delta_Ys'
   */
  if (rtb_OrOperator) {
    rtb_MinMax = 0.0F;
  } else {
    rtb_MinMax = LKC_Delta_Ys;
  }

  /* End of Switch: '<S208>/Switch' */

  /* Product: '<S206>/Product' */
  LAT_Oc_Integrator_Input = rtb_Product4_b * rtb_MinMax;

  /* RelationalOperator: '<S229>/Relational Operator' incorporates:
   *  Constant: '<S229>/Lat_oc_min_velocity'
   *  Gain: '<S229>/Gain'
   */
  rtb_LDP_Active = (3.6F * rtb_VEH_Vehicle_Speed > Lat_oc_min_velocity);

  /* RelationalOperator: '<S226>/Relational Operator' incorporates:
   *  Constant: '<S226>/Lat_oc_max_velocity'
   *  Gain: '<S226>/Gain'
   */
  rtb_OrOperator = (3.6F * rtb_VEH_Vehicle_Speed < Lat_oc_max_velocity);

  /* Sum: '<S232>/Add' incorporates:
   *  Constant: '<S232>/Lat_oc_kappa_cmd_filter_coeff'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S232>/Divide'
   *  Product: '<S232>/Product'
   *  Sum: '<S232>/Add1'
   *  Sum: '<S232>/Sub'
   *  UnitDelay: '<S194>/Unit Delay'
   *  UnitDelay: '<S232>/Unit Delay'
   */
  LAT_Kappa_Command_filt_state = VEH_cycletime / (VEH_cycletime +
    Lat_oc_kappa_cmd_filter_coeff) * (LAT_Kappa_Command_pre -
    LAT_Kappa_Command_filt_state) + LAT_Kappa_Command_filt_state;

  /* RelationalOperator: '<S225>/Relational Operator' incorporates:
   *  Abs: '<S225>/Abs'
   *  Constant: '<S225>/Lat_oc_max_lateral_accel'
   *  Product: '<S225>/Product2'
   *  UnitDelay: '<S232>/Unit Delay'
   */
  LAT_Oc_Max_Lat_Acc_Flag = (rtb_SAC_Arbitrated_Angle_Cmd * fabsf
    (LAT_Kappa_Command_filt_state) < Lat_oc_max_lateral_accel);

  /* RelationalOperator: '<S221>/Relational Operator' incorporates:
   *  Constant: '<S221>/Lat_oc_max_kappa'
   */
  LAT_Oc_Max_Kappa_Cmd_Flag = (rtb_Switch_iy_idx_0 < Lat_oc_max_kappa);

  /* RelationalOperator: '<S222>/Relational Operator' incorporates:
   *  Constant: '<S222>/Lat_oc_max_delta_f_dot'
   */
  LAT_Oc_Max_Delta_F_Dot_Flag = (TDF_Torque_Request_Factor_sta_0 <
    Lat_oc_max_delta_f_dot);

  /* RelationalOperator: '<S228>/Relational Operator' incorporates:
   *  Constant: '<S228>/Lat_oc_min_kappa_quality'
   *  Inport: '<Root>/Inport9'
   */
  rtb_OR = (CAM_Kappa_Cmd_Qf >= Lat_oc_min_kappa_quality);

  /* RelationalOperator: '<S218>/Compare' incorporates:
   *  Constant: '<S218>/Constant'
   *  UnitDelay: '<S194>/Unit Delay'
   */
  rtb_Compare_hu = (LAT_Kappa_Command_pre != 0.0F);

  /* RelationalOperator: '<S227>/Relational Operator' incorporates:
   *  Constant: '<S227>/Lat_oc_kappa_min_head_err_qual'
   *  Inport: '<Root>/Inport10'
   */
  rtb_RelationalOperator_b = (CAM_Heading_Error_Qf >=
    Lat_oc_kappa_min_head_err_qual);

  /* Abs: '<S224>/Abs' incorporates:
   *  Abs: '<S220>/Abs'
   *  Gain: '<S224>/Gain'
   *  Inport: '<Root>/Inport8'
   */
  rtb_MinMax = fabsf(57.3F * CAM_Heading_Error);

  /* RelationalOperator: '<S224>/Relational Operator' incorporates:
   *  Abs: '<S224>/Abs'
   *  Constant: '<S224>/Lat_oc_kappa_max_heading_error'
   */
  LAT_Oc_Max_Hea_Err_Flag = (rtb_MinMax < Lat_oc_kappa_max_heading_error);

  /* MinMax: '<S230>/MinMax' incorporates:
   *  Constant: '<S220>/Lat_oc_kappa_min_latency'
   *  UnitDelay: '<S230>/Unit Delay'
   */
  rtb_MinMax_k = fminf(Lat_oc_kappa_min_latency, LAT_Oc_Kappa_Latency_state);

  /* RelationalOperator: '<S230>/Relational Operator2' incorporates:
   *  Constant: '<S220>/Lat_oc_kappa_min_latency'
   */
  LAT_Oc_Kappa_Con_Enb_Flag = (Lat_oc_kappa_min_latency <= rtb_MinMax_k);

  /* RelationalOperator: '<S223>/Relational Operator' incorporates:
   *  Constant: '<S223>/Lat_oc_kappa_max_driver_torque'
   */
  LAT_Oc_Max_Drv_Trq_Flag = (rtb_DMC_Integtrator_Output_b <
    Lat_oc_kappa_max_driver_torque);

  /* Logic: '<S202>/Logical Operator' */
  LAT_Oc_Trigger_Flag_Kappa = (rtb_LDP_Active && rtb_OrOperator &&
    LAT_Oc_Max_Lat_Acc_Flag && LAT_Oc_Max_Kappa_Cmd_Flag && LAT_Oc_Kappa_Active &&
    LAT_Oc_Max_Delta_F_Dot_Flag && rtb_OR && rtb_Compare_hu &&
    rtb_RelationalOperator_b && LAT_Oc_Max_Hea_Err_Flag &&
    LAT_Oc_Kappa_Con_Enb_Flag && LAT_Oc_Max_Drv_Trq_Flag && rtb_NotOperator2_0);

  /* Switch: '<S201>/Switch1' */
  if (LAT_Oc_Trigger_Flag_Kappa) {
    /* Switch: '<S201>/Switch1' incorporates:
     *  Product: '<S201>/Product1'
     *  Sum: '<S201>/Sub1'
     *  Sum: '<S201>/Sub2'
     *  UnitDelay: '<S192>/Unit Delay'
     *  UnitDelay: '<S194>/Unit Delay'
     */
    LAT_Oc_Integrator_Input_Kappa_dbg = (LAT_Delta_F_Vdy_Offset_pre -
      LAT_Kappa_Command_pre * rtb_LAT_Delta_F_cmd) - LAT_Oc_Integrator_Sat_Out;
  } else {
    /* Switch: '<S201>/Switch1' incorporates:
     *  Constant: '<S201>/Constant1'
     */
    LAT_Oc_Integrator_Input_Kappa_dbg = 0.0F;
  }

  /* End of Switch: '<S201>/Switch1' */

  /* MinMax: '<S201>/MinMax' incorporates:
   *  Constant: '<S201>/Lat_delta_off_flt_initial_loops'
   *  Sum: '<S201>/Add2'
   *  Switch: '<S201>/Switch2'
   *  UnitDelay: '<S201>/Unit Delay'
   */
  LAT_Oc_Trigger_Flag_Kappa_state = fminf(Lat_delta_off_flt_initial_loops,
    (real32_T)LAT_Oc_Trigger_Flag_Kappa + LAT_Oc_Trigger_Flag_Kappa_state);

  /* Switch: '<S201>/Switch3' incorporates:
   *  Constant: '<S201>/Lat_delta_off_flt_initial_loops'
   *  RelationalOperator: '<S201>/Relational Operator'
   *  UnitDelay: '<S201>/Unit Delay'
   */
  if (Lat_delta_off_flt_initial_loops > LAT_Oc_Trigger_Flag_Kappa_state) {
    /* Switch: '<S201>/Switch3' incorporates:
     *  Inport: '<Root>/Inport69'
     */
    LAT_Oc_offset_filter_omega = Lat_oc_delta_off_flt_initial_omega_par;
  } else {
    /* Switch: '<S201>/Switch3' incorporates:
     *  Inport: '<Root>/Inport70'
     */
    LAT_Oc_offset_filter_omega = Lat_oc_delta_offset_filter_omega_par;
  }

  /* End of Switch: '<S201>/Switch3' */

  /* Product: '<S201>/Product2' */
  LAT_Oc_Integrator_Input_Kappa = LAT_Oc_Integrator_Input_Kappa_dbg *
    LAT_Oc_offset_filter_omega;

  /* Switch: '<S200>/Switch' incorporates:
   *  Constant: '<S200>/Constant'
   *  Inport: '<Root>/Inport64'
   *  S-Function (sfix_bitop): '<S200>/Bitwise AND'
   */
  if ((Dmc_offset_calibration_mode_par & 2U) != 0U) {
    rtb_Product4_b = LAT_Oc_Integrator_Input;
  } else {
    rtb_Product4_b = 0.0F;
  }

  /* End of Switch: '<S200>/Switch' */

  /* Switch: '<S200>/Switch2' incorporates:
   *  Constant: '<S200>/Constant1'
   *  Inport: '<Root>/Inport64'
   *  S-Function (sfix_bitop): '<S200>/Bitwise AND1'
   */
  if ((Dmc_offset_calibration_mode_par & 4U) != 0U) {
    rtb_Subtract_jc = LAT_Oc_Integrator_Input_Kappa;
  } else {
    rtb_Subtract_jc = 0.0F;
  }

  /* End of Switch: '<S200>/Switch2' */

  /* Switch: '<S200>/Switch3' incorporates:
   *  Constant: '<S215>/Lat_oc_max_offset_rate'
   *  Gain: '<S200>/Sac_ts'
   *  Gain: '<S215>/Gain1'
   *  MinMax: '<S215>/MinMax1'
   *  MinMax: '<S215>/MinMax3'
   *  Sum: '<S200>/Add1'
   *  Sum: '<S200>/Add2'
   */
  LAT_Oc_Sat_Integrator_state = fmaxf(fminf(rtb_Product4_b + rtb_Subtract_jc,
    Lat_oc_max_offset_rate), -Lat_oc_max_offset_rate) * Sac_ts +
    LAT_Oc_Integrator_Sat_Out;

  /* Sum: '<S219>/Add' incorporates:
   *  Gain: '<S219>/Gain10'
   *  Gain: '<S219>/Gain11'
   *  Gain: '<S219>/Gain12'
   *  Gain: '<S219>/Gain13'
   *  Gain: '<S219>/Gain14'
   *  Gain: '<S219>/Gain15'
   *  Gain: '<S219>/Gain16'
   *  Gain: '<S219>/Gain2'
   *  Gain: '<S219>/Gain3'
   *  Gain: '<S219>/Gain4'
   *  Gain: '<S219>/Gain5'
   *  Gain: '<S219>/Gain6'
   *  Gain: '<S219>/Gain7'
   *  Switch: '<S219>/Switch1'
   *  Switch: '<S219>/Switch10'
   *  Switch: '<S219>/Switch11'
   *  Switch: '<S219>/Switch12'
   *  Switch: '<S219>/Switch13'
   *  Switch: '<S219>/Switch15'
   *  Switch: '<S219>/Switch16'
   *  Switch: '<S219>/Switch2'
   *  Switch: '<S219>/Switch3'
   *  Switch: '<S219>/Switch4'
   *  Switch: '<S219>/Switch5'
   *  Switch: '<S219>/Switch6'
   *  Switch: '<S219>/Switch7'
   */
  LAT_Oc_Kappa_Status_dbg = ((((((((((((((2.0F * (real32_T)rtb_OrOperator +
    (real32_T)rtb_LDP_Active) + 4.0F * (real32_T)LAT_Oc_Max_Lat_Acc_Flag) + 8.0F
    * (real32_T)LAT_Oc_Max_Kappa_Cmd_Flag) + 16.0F * (real32_T)
    LAT_Oc_Kappa_Active) + 32.0F * (real32_T)LAT_Oc_Max_Delta_F_Dot_Flag) +
    64.0F * (real32_T)rtb_OR) + 128.0F) + 256.0F) + 512.0F * (real32_T)
    rtb_Compare_hu) + 1024.0F * (real32_T)rtb_RelationalOperator_b) + 2048.0F *
    (real32_T)LAT_Oc_Max_Hea_Err_Flag) + 4096.0F * (real32_T)
    LAT_Oc_Kappa_Con_Enb_Flag) + 8192.0F * (real32_T)LAT_Oc_Max_Drv_Trq_Flag) +
    16384.0F * (real32_T)rtb_NotOperator2_0) + 32768.0F * (real32_T)
    LAT_Oc_Trigger_Flag_Kappa;

  /* Switch: '<S230>/Switch' incorporates:
   *  Constant: '<S220>/Lat_oc_kappa_max_heading_error'
   *  Constant: '<S220>/Lat_oc_kappa_min_head_err_qual'
   *  Constant: '<S230>/Constant1'
   *  Constant: '<S230>/Constant2'
   *  Inport: '<Root>/Inport10'
   *  Logic: '<S220>/Logical Operator'
   *  RelationalOperator: '<S220>/Relational Operator1'
   *  RelationalOperator: '<S220>/Relational Operator2'
   *  Sum: '<S230>/Add'
   *  UnitDelay: '<S230>/Unit Delay'
   */
  if ((rtb_MinMax < Lat_oc_kappa_max_heading_error) && (CAM_Heading_Error_Qf >=
       Lat_oc_kappa_min_head_err_qual)) {
    LAT_Oc_Kappa_Latency_state = rtb_MinMax_k + 1.0F;
  } else {
    LAT_Oc_Kappa_Latency_state = 0.0F;
  }

  /* End of Switch: '<S230>/Switch' */

  /* Sum: '<S192>/Sub' incorporates:
   *  UnitDelay: '<S192>/Unit Delay'
   */
  LAT_Delta_F_Vdy_Offset_pre = VEH_Delta_F_Oc;

  /* Switch: '<S194>/Switch' incorporates:
   *  Inport: '<Root>/Inport1'
   *  UnitDelay: '<S194>/Unit Delay'
   */
  if (rtb_NOT_n) {
    LAT_Kappa_Command_pre = rtb_LAT_Kappa_Gradient_Fading_S;
  } else {
    LAT_Kappa_Command_pre = CAM_Kappa_Cmd;
  }

  /* End of Switch: '<S194>/Switch' */

  /* Switch: '<S235>/Switch' */
  if (!rtb_LAT_Enable_Lateral_Control) {
    /* Sum: '<S235>/Add' incorporates:
     *  Gain: '<S246>/Gain'
     *  Sum: '<S246>/Add'
     *  UnitDelay: '<S246>/Unit Delay'
     *  UnitDelay: '<S246>/Unit Delay1'
     *  UnitDelay: '<S246>/Unit Delay2'
     */
    LAT_Grad_Limited_Kappa_state = (((rtb_Abs + LAT_Filtered_Kappa_Cmd_pre) +
      LAT_Filtered_Kappa_Cmd_pre_pre) + LAT_Filtered_Kappa_Cmd_pre_pre_pre) *
      0.25F;
  }

  /* End of Switch: '<S235>/Switch' */

  /* MinMax: '<S188>/MinMax' incorporates:
   *  Constant: '<S188>/Constant2'
   *  UnitDelay: '<S188>/Unit Delay2'
   */
  Lat_status_first_run_timer_state = fminf(rtb_Add_p2, 20.0F);

  /* Switch: '<S254>/Switch' incorporates:
   *  Logic: '<S254>/NOT'
   *  Logic: '<S254>/OR'
   *  RelationalOperator: '<S251>/Equal'
   *  UnitDelay: '<S251>/Unit Delay'
   */
  if ((uiDeltaF_Request_nu_pre != uiEPSRequest_nu) || (uiEPSRequest_nu == 0)) {
    /* Sum: '<S254>/Add' */
    Delta_F_Rate_Limitation_state = VEH_Delta_F_Oc;
  }

  /* End of Switch: '<S254>/Switch' */

  /* Sum: '<S268>/Subtract' incorporates:
   *  Abs: '<S267>/Abs'
   *  Constant: '<S266>/Constant'
   *  Constant: '<S267>/Constant'
   *  Constant: '<S267>/Constant1'
   *  Constant: '<S267>/Constant2'
   *  Constant: '<S267>/Constant3'
   *  Inport: '<Root>/Inport1'
   *  MinMax: '<S267>/Max'
   *  MinMax: '<S267>/Max1'
   *  MinMax: '<S267>/Min'
   *  Product: '<S266>/Product'
   *  Product: '<S267>/Product'
   *  Sum: '<S267>/Subtract'
   *  Sum: '<S267>/Subtract1'
   *  UnitDelay: '<S268>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = fminf(fmaxf(0.0F, fmaxf(fabsf(CAM_Kappa_Cmd) -
    Lco_min_curvature_command, rtb_Max_a - Lco_min_curvature_command)) *
    Lco_min_curvature_slope, 1.0F) * Lco_preload_enhancement_factor -
    LAT_Pt1_Output_Undelayed_pre;

  /* Switch: '<S268>/Switch' incorporates:
   *  Constant: '<S268>/Constant1'
   *  Constant: '<S268>/Constant2'
   *  Constant: '<S268>/Constant6'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S268>/Divide'
   *  Product: '<S268>/Divide1'
   *  RelationalOperator: '<S268>/GreaterThan'
   *  Sum: '<S268>/Add1'
   *  Sum: '<S268>/Add2'
   */
  if (rtb_Switch_iy_idx_0 >= 0.0F) {
    rtb_Subtract_jc = 1.0F / (Lco_filter_rising_coeff + VEH_cycletime) *
      VEH_cycletime;
  } else {
    rtb_Subtract_jc = 1.0F / (Lco_filter_falling_coeff + VEH_cycletime) *
      VEH_cycletime;
  }

  /* End of Switch: '<S268>/Switch' */

  /* Sum: '<S268>/Add' incorporates:
   *  Product: '<S268>/Product'
   *  UnitDelay: '<S268>/Unit Delay'
   */
  LAT_Pt1_Output_Undelayed_pre = rtb_Switch_iy_idx_0 * rtb_Subtract_jc +
    LAT_Pt1_Output_Undelayed_pre;

  /* Sum: '<S277>/Subtract' incorporates:
   *  Constant: '<S275>/Constant'
   *  Constant: '<S275>/Constant1'
   *  Constant: '<S275>/Constant2'
   *  Constant: '<S275>/Constant3'
   *  Constant: '<S276>/Constant'
   *  MinMax: '<S275>/Max'
   *  MinMax: '<S275>/Min'
   *  Product: '<S275>/Product'
   *  Product: '<S276>/Product'
   *  Sum: '<S275>/Subtract'
   *  UnitDelay: '<S277>/Unit Delay'
   */
  rtb_Switch_iy_idx_0 = fminf(1.0F, fmaxf(0.0F, rtb_SAC_Trq_Derating_Factor -
    Lco_min_curvature_dot) * Lco_min_curvature_dot_slop) *
    Lco_curv_dot_preload_enh_factor - LAT_Pt1_Output_Undelayed_pre2;

  /* Switch: '<S277>/Switch' incorporates:
   *  Constant: '<S277>/Constant1'
   *  Constant: '<S277>/Constant2'
   *  Constant: '<S277>/Constant6'
   *  Inport: '<Root>/Inport13'
   *  Product: '<S277>/Divide'
   *  Product: '<S277>/Divide1'
   *  RelationalOperator: '<S277>/GreaterThan'
   *  Sum: '<S277>/Add1'
   *  Sum: '<S277>/Add2'
   */
  if (rtb_Switch_iy_idx_0 >= 0.0F) {
    rtb_Subtract_jc = 1.0F / (Lco_curv_dot_filter_risng_coeff + VEH_cycletime) *
      VEH_cycletime;
  } else {
    rtb_Subtract_jc = 1.0F / (Lco_curv_dot_filter_fall_coeff + VEH_cycletime) *
      VEH_cycletime;
  }

  /* End of Switch: '<S277>/Switch' */

  /* Sum: '<S277>/Add' incorporates:
   *  Product: '<S277>/Product'
   *  UnitDelay: '<S277>/Unit Delay'
   */
  LAT_Pt1_Output_Undelayed_pre2 = rtb_Switch_iy_idx_0 * rtb_Subtract_jc +
    LAT_Pt1_Output_Undelayed_pre2;

  /* UnitDelay: '<S282>/Unit Delay' */
  rtb_Add_p2 = VEH_Delta_F_pre1;

  /* Update for UnitDelay: '<S282>/Unit Delay2' incorporates:
   *  UnitDelay: '<S282>/Unit Delay1'
   */
  VEH_Delta_F_pre3 = VEH_Delta_F_pre2;

  /* Signum: '<S289>/Sign' incorporates:
   *  Sum: '<S289>/Subtract'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  if (0.0F - Lon_Acc_Rate_Limitation_state < 0.0F) {
    rtb_Subtract_jc = -1.0F;
  } else if (0.0F - Lon_Acc_Rate_Limitation_state > 0.0F) {
    rtb_Subtract_jc = 1.0F;
  } else {
    rtb_Subtract_jc = 0.0F - Lon_Acc_Rate_Limitation_state;
  }

  /* End of Signum: '<S289>/Sign' */

  /* Sum: '<S289>/Add' incorporates:
   *  Abs: '<S289>/Abs'
   *  Abs: '<S289>/Abs1'
   *  Constant: '<S288>/Constant'
   *  Gain: '<S289>/Sac_ts2'
   *  MinMax: '<S289>/Min'
   *  Product: '<S289>/Product1'
   *  Product: '<S289>/Product4'
   *  Sum: '<S289>/Subtract'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  Lon_Acc_Rate_Limitation_state = fminf(fabsf(Sac_ts * Lon_Acc_Gradient_Par *
    rtb_Subtract_jc), fabsf(0.0F - Lon_Acc_Rate_Limitation_state)) *
    rtb_Subtract_jc + Lon_Acc_Rate_Limitation_state;

  /* Update for UnitDelay: '<S214>/Unit Delay3' */
  LAT_Status_First_Run_state = LAT_Status_Firsst_Run_pre;

  /* Update for UnitDelay: '<S187>/Unit Delay' */
  LDP_Active_state1 = false;

  /* Update for UnitDelay: '<S187>/Unit Delay1' */
  LDP_Active_state2 = false;

  /* Update for UnitDelay: '<S59>/Unit Delay' */
  VEH_Steer_Torque_Comp_state = VEH_Steer_Torque_Comp;

  /* Update for UnitDelay: '<S11>/Unit Delay1' */
  fDeltaFCmd_State = fSteerAngle_deg;

  /* Update for UnitDelay: '<S136>/Unit Delay' incorporates:
   *  UnitDelay: '<S190>/Unit Delay'
   */
  VEH_Yaw_Rate_state = LAT_Yaw_Rate_Filter_state;

  /* Update for UnitDelay: '<S132>/Unit Delay2' incorporates:
   *  UnitDelay: '<S132>/Unit Delay1'
   */
  SAC_Disable_pre_pre = SAC_Disable_pre;

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  TDF_Ldp_Override_Factor_state = rtb_Sac_ts2;

  /* Update for UnitDelay: '<S112>/Unit Delay' */
  HEC_Yaw_Rate_Filter_state2 = HEC_Yaw_Rate_Filter;

  /* Update for DiscreteIntegrator: '<S126>/Discrete-Time Integrator' */
  DYC_Boost_Filter_Output_state = 0.01F * rtb_Abs1_j +
    DYC_Boost_Filter_Output_state;

  /* Update for UnitDelay: '<S149>/Unit Delay' */
  Initialisation_Flag_state = Initialisation_Flag;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  VEH_Delta_F_Pre = VEH_Delta_F_Oc;

  /* Update for UnitDelay: '<S5>/Unit Delay' incorporates:
   *  UnitDelay: '<S1>/Unit Delay2'
   */
  SAC_Control_Error_pre = SAC_Control_Error_state;

  /* Update for UnitDelay: '<S151>/Unit Delay' */
  Vehicle_Speed_Segment_state = rtb_SAC_Controller_Mode;

  /* Update for UnitDelay: '<S151>/Unit Delay1' incorporates:
   *  MultiPortSwitch: '<S151>/Index Vector'
   */
  Last_Seg_Correction_Factor_Left_Scal_state =
    rtb_Switch_iu[rtb_SAC_Controller_Mode];

  /* Update for UnitDelay: '<S181>/Unit Delay' */
  ADP_Dyc_Corr_Factor_state = ADP_Dyc_Corr_Factor;

  /* Update for UnitDelay: '<S128>/Unit Delay1' incorporates:
   *  Inport: '<Root>/LKC_Delta_Psi'
   */
  LKC_Delta_Psi_pre1 = LKC_Delta_Psi;

  /* Update for UnitDelay: '<S128>/Unit Delay2' */
  LKC_Delta_Psi_pre2 = rtb_Hec_r_factor;

  /* Update for UnitDelay: '<S128>/Unit Delay3' */
  LKC_Delta_Psi_pre3 = rtb_Gain_d;

  /* Update for UnitDelay: '<S128>/Unit Delay4' */
  LKC_Delta_Psi_pre4 = rtb_SAT_Max_Delta_F_Cmd;

  /* Update for UnitDelay: '<S128>/Unit Delay5' */
  LKC_Delta_Psi_pre5 = rtb_Tdf_min_steer_torque_class;

  /* Update for UnitDelay: '<S128>/Unit Delay6' */
  LKC_Delta_Psi_pre6 = rtb_UnitDelay5;

  /* Update for UnitDelay: '<S128>/Unit Delay7' */
  LKC_Delta_Psi_pre7 = rtb_UnitDelay6;

  /* Update for UnitDelay: '<S128>/Unit Delay8' */
  LKC_Delta_Psi_pre8 = rtb_Product_lg;

  /* Update for UnitDelay: '<S128>/Unit Delay9' */
  LKC_Delta_Psi_pre9 = rtb_Add_hp5;

  /* Update for UnitDelay: '<S128>/Unit Delay' */
  SAC_Delta_Psi_state = rtb_DMC_Integtrator_Output_ks;

  /* Update for UnitDelay: '<S79>/Unit Delay1' incorporates:
   *  Inport: '<Root>/LKC_Delta_Ys'
   */
  LKC_Delta_Ys_pre = LKC_Delta_Ys;

  /* Update for UnitDelay: '<S132>/Unit Delay1' */
  SAC_Disable_pre = rtb_SAC_Disable;

  /* Update for UnitDelay: '<S231>/Unit Delay' */
  LAT_Kappa_Command_state = LAT_Oc_Filtered_Kappa_Cam;

  /* Update for UnitDelay: '<S200>/Unit Delay1' incorporates:
   *  Constant: '<S191>/Constant1'
   */
  LAT_Vdy_Offset_Used_pre = false;

  /* Update for UnitDelay: '<S246>/Unit Delay' */
  LAT_Filtered_Kappa_Cmd_pre = rtb_Abs;

  /* Update for UnitDelay: '<S246>/Unit Delay1' */
  LAT_Filtered_Kappa_Cmd_pre_pre = rtb_Abs;

  /* Update for UnitDelay: '<S246>/Unit Delay2' */
  LAT_Filtered_Kappa_Cmd_pre_pre_pre = rtb_Abs;

  /* Update for UnitDelay: '<S251>/Unit Delay' */
  uiDeltaF_Request_nu_pre = uiEPSRequest_nu;

  /* Update for UnitDelay: '<S282>/Unit Delay' */
  VEH_Delta_F_pre1 = 0.0F;

  /* Update for UnitDelay: '<S282>/Unit Delay1' */
  VEH_Delta_F_pre2 = rtb_Add_p2;

  /* Update for UnitDelay: '<S1>/Unit Delay2' */
  SAC_Control_Error_state = SAC_Control_Error;
}

/* Model initialize function */
void LaDMC_initialize(void)
{
  /* SystemInitialize for Enabled SubSystem: '<S165>/Mean_Of_All_Segments' */
  /* InitializeConditions for Delay: '<S167>/Resettable Delay4' */
  Lateral_Error_Invalid_state = 1.0F;

  /* End of SystemInitialize for SubSystem: '<S165>/Mean_Of_All_Segments' */

  /* SystemInitialize for Enabled SubSystem: '<S149>/Initial' */
  /* SystemInitialize for SignalConversion generated from: '<S179>/Outport' */
  Initialisation_Flag = true;

  /* End of SystemInitialize for SubSystem: '<S149>/Initial' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
