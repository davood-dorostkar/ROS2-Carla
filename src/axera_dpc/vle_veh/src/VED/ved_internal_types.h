/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */

#ifndef _RTW_HEADER_ved__internal_types_h_
#define _RTW_HEADER_ved__internal_types_h_

#include "ved_ext.h"

#ifndef _DEFINED_TYPEDEF_FOR_ved__wpp_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__wpp_out_t_
typedef struct {
    float32 wheel_velo_front_left;
    float32 wheel_velo_front_left_var;
    float32 wheel_velo_front_right;
    float32 wheel_velo_front_right_var;
    float32 wheel_velo_rear_left;
    float32 wheel_velo_rear_left_var;
    float32 wheel_velo_rear_right;
    float32 wheel_velo_rear_right_var;
    uint8 aqua_slip_state_front_left;
    uint8 aqua_slip_state_front_right;
    uint8 aqua_slip_state_rear_left;
    uint8 aqua_slip_state_rear_right;
} ved__wpp_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__ve_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__ve_out_t_
typedef struct {
    float32 veh_velo;
    float32 veh_velo_var;
    float32 veh_accel;
    float32 veh_accel_var;
} ved__ve_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__wye_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__wye_out_t_
typedef struct {
    float32 whl_yaw_rate;
    float32 whl_yaw_rate_var;
    float32 gier_yaw_rate_offset;
    float32 gier_yaw_rate_offset_var;
    float32 front_whl_yaw_rate_filt;
    float32 front_whl_yaw_rate_filt_wld;
    float32 rear_whl_yaw_rate_filt;
    float32 rear_whl_yaw_rate_filt_wld;
    float32 diff_whl_yaw_front_rear;
    float32 est_whl_load_dep_front;
    float32 raw_est_yaw_offset;
    uint8 dyn_yaw_off_control;
    uint8 wld_control;
    uint8 r_On_Off_control;
} ved__wye_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__gye_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__gye_out_t_
typedef struct {
    float32 gier_yaw_rate;
    float32 gier_yaw_rate_var;
    boolean r_On_Off_control;
} ved__gye_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__gye_out_filt_t_
#define _DEFINED_TYPEDEF_FOR_ved__gye_out_filt_t_
typedef struct {
    float32 gier_yaw_rate;
    float32 gier_yaw_rate_var;
    float32 raw_est_yaw_offset_filt;
} ved__gye_out_filt_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__aye_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__aye_out_t_
typedef struct {
    float32 ay_yaw_rate;
    float32 ay_yaw_rate_var;
    float32 track_bent;
    float32 track_bent_var;
    uint8 r_On_Off_control;
} ved__aye_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__sye_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__sye_out_t_
typedef struct {
    float32 stw_yaw_rate;
    float32 stw_yaw_rate_var;
    float32 stw_yaw_rate_offset;
    float32 stw_yaw_rate_offset_var;
    float32 stw_curve;
    float32 stw_curve_var;
    float32 stw_curve_grad;
    float32 stw_curve_grad_var;
    float32 stw_understeer_grad;
    float32 stw_understeer_grad_disc;
    float32 stw_understeer_grad_var;
    float32 stw_understeer_grad_max;
    float32 stw_understeer_grad_min;
    uint8 stw_understeer_control;
    uint8 r_On_Off_control;
} ved__sye_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__ye_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__ye_out_t_
typedef struct {
    float32 veh_yaw_rate;
    float32 veh_yaw_rate_var;
    float32 veh_wye_rate_usage;
    float32 veh_gye_rate_usage;
    float32 veh_aye_rate_usage;
    float32 veh_sye_rate_usage;
    float32 veh_merge_curve;
    float32 veh_merge_curve_var;
    float32 veh_merge_curve_grad;
    float32 veh_merge_curve_grad_var;
    float32 veh_merge_curve_Q11;
    float32 veh_lat_accel;
    float32 veh_yaw_rate_var_org;
} ved__ye_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__sae_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__sae_out_t_
typedef struct {
    float32 est_slip_angle;
    float32 est_slip_angle_var;
    float32 raw_slip_angle;
} ved__sae_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__mot_st_out_t_
#define _DEFINED_TYPEDEF_FOR_ved__mot_st_out_t_
typedef struct {
    uint8 fwd;
    uint8 ss;
    uint8 rvs;
    uint8 mot_state;
    uint8 mot_quality;
    sint8 mot_counter;
} ved__mot_st_out_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__yaw_offset_t_
#define _DEFINED_TYPEDEF_FOR_ved__yaw_offset_t_
typedef struct {
    float32 offset;
    float32 var;
    float32 quality;
    uint8 state;
} ved__yaw_offset_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__swa_offset_t_
#define _DEFINED_TYPEDEF_FOR_ved__swa_offset_t_
typedef struct {
    float32 offset;
    float32 var;
    uint8 state;
} ved__swa_offset_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__whs_offset_t_
#define _DEFINED_TYPEDEF_FOR_ved__whs_offset_t_
typedef struct {
    float32 offset_ratio_front;
    float32 offset_ratio_front_dev;
    float32 offset_ratio_rear;
    float32 offset_ratio_rear_dev;
    sint32 SpeedRange;
} ved__whs_offset_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__offsets_in_t_
#define _DEFINED_TYPEDEF_FOR_ved__offsets_in_t_
typedef struct {
    ved__yaw_offset_t ved__yaw_offset;
    ved__swa_offset_t ved__swa_offset;
    ved__whs_offset_t ved__whs_offset;
    ved__swa_offset_t ved__ay_offset;
} ved__offsets_in_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_VED_InternalData_t_
#define _DEFINED_TYPEDEF_FOR_VED_InternalData_t_
typedef struct {
    ved__wpp_out_t ved__wpp_out;
    ved__ve_out_t ved__ve_out;
    ved__wye_out_t ved__wye_out;
    ved__gye_out_t ved__gye_out;
    ved__gye_out_filt_t ved__gye_out_filt;
    ved__aye_out_t ved__aye_out;
    ved__sye_out_t ved__sye_out;
    ved__ye_out_t ved__ye_out;
    ved__sae_out_t ved__sae_out;
    ved__mot_st_out_t ved__mot_st_out;
    ved__offsets_in_t ved__offsets_in;
} VED_InternalData_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__mot_states_t_
#define _DEFINED_TYPEDEF_FOR_ved__mot_states_t_
typedef struct {
    uint8 fwd;
    uint8 ss;
    uint8 rvs;
} ved__mot_states_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__whl_puls_mot_st_t_
#define _DEFINED_TYPEDEF_FOR_ved__whl_puls_mot_st_t_
typedef struct {
    ved__mot_states_t front_left;
    ved__mot_states_t front_right;
    ved__mot_states_t rear_left;
    ved__mot_states_t rear_right;
} ved__whl_puls_mot_st_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__whl_dir_mot_st_t_
#define _DEFINED_TYPEDEF_FOR_ved__whl_dir_mot_st_t_
typedef struct {
    ved__mot_states_t front_left;
    ved__mot_states_t front_right;
    ved__mot_states_t rear_left;
    ved__mot_states_t rear_right;
} ved__whl_dir_mot_st_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__input_mot_st_t_
#define _DEFINED_TYPEDEF_FOR_ved__input_mot_st_t_
typedef struct {
    ved__whl_puls_mot_st_t whl_puls_states;
    ved__mot_states_t veh_velocity_state;
    ved__whl_dir_mot_st_t whl_dir_states;
    ved__mot_states_t ALN_dir_states;
    ved__mot_states_t brake_torque_state;
    ved__mot_states_t gear_shift_state;
    ved__mot_states_t park_brake_state;
    ved__mot_states_t yaw_rate_state;
} ved__input_mot_st_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__bayes_out_mot_states_t_
#define _DEFINED_TYPEDEF_FOR_ved__bayes_out_mot_states_t_
typedef struct {
    ved__mot_states_t stage_1;
    ved__mot_states_t stage_2;
    ved__mot_states_t stage_3;
    ved__mot_states_t stage_4;
    ved__mot_states_t stage_5;
} ved__bayes_out_mot_states_t;
#endif

#ifndef _DEFINED_TYPEDEF_FOR_ved__bayes_mot_states_t_
#define _DEFINED_TYPEDEF_FOR_ved__bayes_mot_states_t_
typedef struct {
    ved__input_mot_st_t mot_st_bayes_in;
    ved__bayes_out_mot_states_t mot_st_bayes_out;
    ved__mot_st_out_t mot_st_out;
} ved__bayes_mot_states_t;
#endif

#endif
