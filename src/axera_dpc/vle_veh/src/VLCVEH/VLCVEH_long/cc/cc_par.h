/*! \file **********************************************************************

  COMPONENT:              CC (Cruise Control)

  MODULENAME:             cc_par.h

  @brief                  Controller Parameter definition


*****************************************************************************/

/* conditional include */
#ifndef CCPAR_INCLUDED
#define CCPAR_INCLUDED

//#include "vlc_long_veh.h"
#include "cc_cfg.h"

#if (defined(_MSC_VER) && defined(_WIN32))
#define Cc_const DLLEXPORT
#else
#define Cc_const
#endif

#define max_decel_gradient_critical MAX_DECEL_GRADIENT_CRITICAL
#define max_decel_gradient_critical_points ((uint16)2)
extern Cc_const volatile acceleration_t
    MAX_DECEL_GRADIENT_CRITICAL[2 * max_decel_gradient_critical_points];

/* Factor to increase the acceleration request gradient in critical traffic
 * situations */
#define criticality_accel_gradient_gain CRITICALITY_ACCEL_GRADIENT_GAIN
#define accel_gradient_gain_points ((uint16)2)
extern Cc_const volatile factor_t
    CRITICALITY_ACCEL_GRADIENT_GAIN[2 * accel_gradient_gain_points];

/* Factor to reduce filter time in critical traffic situations */
#define criticality_accel_filter_time CRITICALITY_ACCEL_FILTER_TIME
#define accel_filter_time_time_points ((uint16)2)
extern Cc_const volatile times_t
    CRITICALITY_ACCEL_FILTER_TIME[2 * accel_filter_time_time_points];

/*! set mode depended parameters */
extern void VLC_SELECT_PARAM_SET(void);

/*![m/s�] acceleration delta that is used to determine if the acceleration will
 * reach a specific value (keep it at 0.05m/(s^2)*/
#define Cc_accel_band ((acceleration_t)50)

/*! [m/s�] cruise acceleration is a function of the difference vehicle speed and
 * set speed */
#define Cc_accel_curve VLC_ACCEL_CURVE
#define Cc_curve_points ((uint16)18)
extern Cc_const volatile acceleration_t VLC_ACCEL_CURVE[2 * Cc_curve_points];

/*! [m/s�] cruise acceleration is a function of the difference integration of
 * delta speed and set speed */
#define Cc_accel_curve_delta_speed_integ VLC_ACCEL_CURVE_DELTA_SPEED_INT
#define Cc_accel_curve_delta_speed_integ_points ((uint16)6)
extern Cc_const volatile acceleration_t
    VLC_ACCEL_CURVE_DELTA_SPEED_INT[2 *
                                    Cc_accel_curve_delta_speed_integ_points];

/*! [m/s�] max longitudinal acceleration is a function of the difference vehicle
 * speed and max speed for a lateral acceleration*/
#define Cc_long_accel_curve VLC_LONG_ACCEL_CURVE
#define Cc_long_accel_curve_points ((uint16)8)
extern Cc_const volatile acceleration_t
    VLC_LONG_ACCEL_CURVE[2 * Cc_long_accel_curve_points];

/*! [m/s] max speed is a function of the lateral acceleration*/
#define Cc_alat_speed_curve_points ((uint16)6)
#define Cc_alat_speed_curve pVLC_ALatSpeedCurve
extern Cc_const volatile velocity_t* pVLC_ALatSpeedCurve;

/*! [m/s�] max allowed vehicle(ESP) acceleration is a function of the vehicle
 * speed*/
#define Cc_max_vehicle_accel_points ((uint16)4)
#define Cc_max_vehicle_accel VLC_MAX_VEHICLE_ACCEL
extern Cc_const volatile acceleration_t
    VLC_MAX_VEHICLE_ACCEL[2 * Cc_max_vehicle_accel_points];

/*! [m/s�] max allowed vehicle(ESP) deceleration is a function of the vehicle
 * speed*/
#define Cc_max_vehicle_decel_points ((uint16)3)
#define Cc_max_vehicle_decel VLC_MAX_VEHICLE_DECEL
extern Cc_const volatile acceleration_t
    VLC_MAX_VEHICLE_DECEL[2 * Cc_max_vehicle_decel_points];

/*! [m/s�] max cruise acceleration is a function of the vehicle speed*/
#define Cc_max_accel_curve_points ((uint16)4)
#define Cc_max_accel_curve VLC_MAX_ACCEL_CURVE
extern Cc_const volatile acceleration_t
    VLC_MAX_ACCEL_CURVE[2 * Cc_max_accel_curve_points];

// Minimum allowed acceleration for ACC
#define Cc_min_accel_active_curve VLC_ACTIVE_MIN_ACCEL_CURVE
extern Cc_const volatile acceleration_t VLC_ACTIVE_MIN_ACCEL_CURVE;

// Maximum allowed acceleration for ACC
#define Cc_max_accel_active_curve_points ((uint16)4)
#define Cc_max_accel_active_curve VLC_ACTIVE_MAX_ACCEL_CURVE
extern Cc_const volatile acceleration_t
    VLC_ACTIVE_MAX_ACCEL_CURVE[2 * Cc_max_accel_active_curve_points];

/*! [m/s�] accel gain is a function of the vehicle speed*/
#define Cc_accel_gain_curve VLC_ACCEL_GAIN_CURVE
#define Cc_accel_gain_curve_points ((uint16)4)
extern Cc_const volatile acceleration_t
    VLC_ACCEL_GAIN_CURVE[2 * Cc_accel_gain_curve_points];

/*! [m/s�] limiter acceleration is a function of the difference vehicle speed
 * and set speed */
#define Lim_accel_curve LIM_ACCEL_CURVE
#define Lim_curve_points ((uint16)14)
extern Cc_const volatile acceleration_t LIM_ACCEL_CURVE[2 * Lim_curve_points];

/*! [m/s] setspeed offset for cruise control*/
#define Cc_offset_setspeed VLC_V_OFFSET_SETSPEED
extern Cc_const volatile velocity_t VLC_V_OFFSET_SETSPEED;

/*! [m/s] delta speed integration upper limit for cruise control*/
#define Cc_delta_speed_integ_upper_limit VLC_V_DELTA_INT_UP_LIM
extern Cc_const volatile velocity_t VLC_V_DELTA_INT_UP_LIM;

/*! [m/s] delta speed integration upper limit for cruise control*/
#define Cc_delta_speed_integ_lower_limit VLC_V_DELTA_INT_LOW_LIM
extern Cc_const volatile velocity_t VLC_V_DELTA_INT_LOW_LIM;

/*! [m/s] acceleration request prior to "acceleration mode" */
#define Cc_accelmode_init VLC_V_ACCELMODE_INIT
extern Cc_const volatile velocity_t VLC_V_ACCELMODE_INIT;

/*! [m/s] acceleration request during "acceleration mode" */
#define Cc_accelmode_delta VLC_V_ACCELMODE
extern Cc_const volatile velocity_t VLC_V_ACCELMODE;

/*! [m/s�] maximum acceleration request during cruise control */
#define Cc_accel_limit VLC_ACCEL_LIMIT
extern Cc_const volatile acceleration_t VLC_ACCEL_LIMIT;

/*! [m/s�] maximum deceleration request during cruise control */
#define Cc_decel_limit (*pVLC_DecelLimit)
extern Cc_const volatile acceleration_t* pVLC_DecelLimit;

/*! [km/h] when the vehicle speed goes below minimum set speed minus this value,
 * the cruise control is disengaged (with an object)*/
#define Cc_disengage_threshold_kmh VLC_OFF_THRES_KMH
extern Cc_const volatile setspeed_t VLC_OFF_THRES_KMH;

/*! [km/h] minimum allowed engagement speed*/
#define Cc_min_v_engage_kmh VLC_MIN_V_ENGAGE_KMH
extern Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_KMH;

/*! [mph] minimum allowed engagement speed*/
#define Cc_min_v_engage_mph VLC_MIN_V_ENGAGE_MPH
extern Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_MPH;

/*! [mph] minimum allowed engagement speed (usa)*/
#define Cc_min_v_engage_usa_mph VLC_MIN_V_ENGAGE_USA_MPH
extern Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_USA_MPH;

/*! [km/h] when the vehicle speed goes below minimum set speed minus this value,
 * the cruise control is disengaged (without an object)*/
#define Cc_disengage_threshold_cruise_kmh VLC_OFF_THRES_CRUISE_KMH
extern Cc_const volatile setspeed_t VLC_OFF_THRES_CRUISE_KMH;

/*! [m/s] deceleration request during "deceleration mode" */
extern Cc_const volatile velocity_t VLC_V_DECELMODE;
#define Cc_decelmode_delta VLC_V_DECELMODE

/*! [m/s] deceleration request prior to "deceleration mode" */
extern Cc_const volatile velocity_t VLC_V_DECELMODE_INIT;
#define Cc_decelmode_init VLC_V_DECELMODE_INIT

/*! [km/h] maximum set speed for cruise control function */
#define Cc_max_setspeed_kmh VLC_MAX_VSET_KMH
extern Cc_const volatile setspeed_t VLC_MAX_VSET_KMH;

/*! [m/s�] maximum deceleration during override */
#define Cc_max_decel_during_override VLC_MAX_DECEL_DURING_OVERRIDE
extern Cc_const volatile acceleration_t VLC_MAX_DECEL_DURING_OVERRIDE;

/*! [m/s�] maximum acceleration request after override */
#define Cc_max_decel_after_override VLC_MAX_DECEL_AFTER_OVERRIDE
extern Cc_const volatile acceleration_t VLC_MAX_DECEL_AFTER_OVERRIDE;

/*! [m/s�] maximum acceleration request after engage */
#define Cc_max_decel_after_engage VLC_MAX_DECEL_AFTER_ENGAGE
extern Cc_const volatile acceleration_t VLC_MAX_DECEL_AFTER_ENGAGE;

/*! [m/s�] Maximum increase of acceleration request during automatic braking */
#define Cc_max_release_brake_grad VLC_MAX_GRAD_RELEASE_BRAKE
extern Cc_const volatile gradient_t VLC_MAX_GRAD_RELEASE_BRAKE;

/*! scale of actual speed to display speed*/
#define factor_actual_speed_to_display_speed_points ((uint16)2)
#define factor_actual_speed_to_display_speed VLC_FAC_ACT_TO_DISP_SPEED
extern Cc_const volatile factor_t
    VLC_FAC_ACT_TO_DISP_SPEED[2 * factor_actual_speed_to_display_speed_points];

/*! offset of actual speed to display speed*/
#define offset_actual_speed_to_display_speed_points ((uint16)2)
#define offset_actual_speed_to_display_speed VLC_OFFSET_ACT_TO_DISP_SPEED
extern Cc_const volatile factor_t
    VLC_OFFSET_ACT_TO_DISP_SPEED[2 *
                                 offset_actual_speed_to_display_speed_points];

/*! [m/s] Minimum valid velocity offset of speedometer speed */
/*! If speedometer_speed < vehicle_velocity + offset, then speedometer speed is
 * not valid */
#define Cc_min_valid_speedo_offset_points ((uint16)3)
#define Cc_min_valid_speedo_offset VLC_MIN_VALID_SPEEDO_OFFSET
extern Cc_const volatile velocity_t
    VLC_MIN_VALID_SPEEDO_OFFSET[2 * Cc_min_valid_speedo_offset_points];

/*! [m/s] Maximum valid velocity offset of speedometer speed */
/*! If speedometer_speed > vehicle_velocity + offset, then speedometer speed is
 * not valid */
#define Cc_max_valid_speedo_offset_points ((uint16)3)
#define Cc_max_valid_speedo_offset VLC_MAX_VALID_SPEEDO_OFFSET
extern Cc_const volatile velocity_t
    VLC_MAX_VALID_SPEEDO_OFFSET[2 * Cc_max_valid_speedo_offset_points];

/*! [m/s�] Maximum decrease of acceleration request during "acceleration /
 * deceleration mode" */
#define Cc_max_neg_grad_accel_decel VLC_ACCELMODE_MAX_NEG_GRAD
extern Cc_const volatile gradient_t VLC_ACCELMODE_MAX_NEG_GRAD;

/*! [m/s�] Maximum decrease of acceleration request during cruise control */
#define Cc_neg_grad_points (uint16)2
#define Cc_max_neg_grad pVLC_MaxNegGrad
extern Cc_const volatile acceleration_t* pVLC_MaxNegGrad;

/*! ACC to CC transition factor to reduce acceleration change rate */
#define Cc_acc_to_cc_transition_factor (*pVLC_AccToCcTransitionFactor)
extern Cc_const volatile factor_t* pVLC_AccToCcTransitionFactor;

/*! [m/s�] Maximum decrease of acceleration request after cruise disengagement
 */
#define Cc_max_neg_grad_disengage VLC_OFF_MAX_NEG_GRAD
extern Cc_const volatile gradient_t VLC_OFF_MAX_NEG_GRAD;

/*! [m/s�] Maximum increase of acceleration request during "acceleration /
 * deceleration mode" */
#define Cc_max_pos_grad_accel_decel VLC_ACCELMODE_MAX_POS_GRAD
extern Cc_const volatile gradient_t VLC_ACCELMODE_MAX_POS_GRAD;

/*! Factor for Maximum increase of acceleration request during cruise mode
 * in eco mode)*/
#define Cc_pos_accel_grad_eco_fac VLC_POS_GRAD_ECO_FAC
extern Cc_const volatile factor_t VLC_POS_GRAD_ECO_FAC;

/*! Factor for Maximum increase of acceleration request during cruise mode
 *in sport mode)*/
#define Cc_pos_accel_grad_sport_fac VLC_POS_GRAD_SPORT_FAC
extern Cc_const volatile factor_t VLC_POS_GRAD_SPORT_FAC;

/*! [m/s�] Maximum increase of acceleration request during cruise control */
#define Cc_pos_grad_pos_accel_points (uint16)2
#define Cc_max_pos_grad_pos_accel pVLC_MaxPosGradPosAccel
extern Cc_const volatile acceleration_t* pVLC_MaxPosGradPosAccel;

/*! [m/s�] Maximum increase of acceleration request during cruise control */
#define Cc_pos_grad_neg_accel_points (uint16)2
#define Cc_max_pos_grad_neg_accel pVLC_MaxPosGradNegAccel
extern Cc_const volatile acceleration_t* pVLC_MaxPosGradNegAccel;

/*! [m/s�] Maximum increase of acceleration request after cruise disengagement
 */
#define Cc_max_pos_grad_disengage VLC_OFF_MAX_POS_GRAD
extern Cc_const volatile gradient_t VLC_OFF_MAX_POS_GRAD;

/*! [m/s�] Maximum increase of acceleration request after cruise disengagement
 * with rapid ramp*/
#define Cc_max_pos_grad_rapid_disengage VLC_OFF_MAX_POS_GRAD_RAPID
extern Cc_const volatile gradient_t VLC_OFF_MAX_POS_GRAD_RAPID;

/*! [m/s�] Maximum decrease of acceleration request after cruise disengagement
 * with rapid ramp*/
#define Cc_max_neg_grad_rapid_disengage VLC_OFF_MAX_NEG_GRAD_RAPID
extern Cc_const volatile gradient_t VLC_OFF_MAX_NEG_GRAD_RAPID;

/*! [km/h] minimum set speed for cruise control function */
#define Cc_min_setspeed_kmh VLC_MIN_VSET_KMH
extern Cc_const volatile setspeed_t VLC_MIN_VSET_KMH;

/*! [km/h] minimum speed when no effective object to active cruise control
 * function */
#define Cc_min_activespeed_kmh VLC_MIN_VACTIVE_KMH
extern Cc_const volatile setspeed_t VLC_MIN_VACTIVE_KMH;

// ego vehicle velocity static judge threshold
#define Cc_static_veL_thres VLC_STATIC_VEL_THRES
extern Cc_const volatile velocity_t VLC_STATIC_VEL_THRES;

// ego vehicle deceleration static judge threshold
#define Cc_static_dec_thres VLC_STATIC_DEC_THRES
extern Cc_const volatile velocity_t VLC_STATIC_DEC_THRES;

/*! [m/s per m/s�] Acceleration dependent offset added to the speedometer speed
 * when it is adopted as the new set speed */
#define Cc_pos_accel_offset VLC_A_POS_OFFSET
extern Cc_const volatile velocity_t VLC_A_POS_OFFSET;

/*! [m/s] Maximum positive acceleration dependent offset */
#define Cc_max_pos_accel_offset VLC_A_POS_OFFSET_MAX
extern Cc_const volatile velocity_t VLC_A_POS_OFFSET_MAX;

/*! [m/s per m/s�] Acceleration dependent offset added to the speedometer speed
 * when it is adopted as the new set speed */
#define Cc_neg_accel_offset VLC_A_NEG_OFFSET
extern Cc_const volatile velocity_t VLC_A_NEG_OFFSET;

/*! [m/s] Acceleration dependent offset added to the speedometer speed when it
 * is adopted as the new set speed */
#define Cc_max_neg_accel_offset VLC_A_NEG_OFFSET_MAX
extern Cc_const volatile velocity_t VLC_A_NEG_OFFSET_MAX;

/*! [m/s�] maximum acceleration request during "deceleration only" mode */
#define Cc_min_decel_brake_only VLC_MIN_DECEL_BRAKE_ONLY
extern Cc_const volatile acceleration_t VLC_MIN_DECEL_BRAKE_ONLY;

/*! [m/s�]  */
extern Cc_const volatile gradient_t VLC_REST_MIN_GRAD;
#define Cc_min_grad_rest VLC_REST_MIN_GRAD

/*! [m/s�] Positive disengagement ramp for speedlimiter */
#define Lim_max_pos_grad_disengage LIM_OFF_MAX_POS_GRAD
extern Cc_const volatile gradient_t LIM_OFF_MAX_POS_GRAD;

/*! [m/s�] Negative disengagement ramp for speedlimiter */
#define Lim_max_neg_grad_disengage LIM_OFF_MAX_NEG_GRAD
extern Cc_const volatile gradient_t LIM_OFF_MAX_NEG_GRAD;

/*! [m/s�]  Positive Jerk during speedlimiter */
#define Lim_max_pos_grad LIM_MAX_POS_GRAD
extern Cc_const volatile gradient_t LIM_MAX_POS_GRAD;

/*! [m/s�]  Negative Jerk during speed limiter */
#define Lim_max_neg_grad LIM_MAX_NEG_GRAD
extern Cc_const volatile gradient_t LIM_MAX_NEG_GRAD;

/*! [m/s�] Max. Positive Jerk during override of speed limiter */
#define Lim_max_pos_grad_override LIM_OVERRIDE_MAX_POS_GRAD
extern Cc_const volatile gradient_t LIM_OVERRIDE_MAX_POS_GRAD;

/*! [m/s�] Max. Negative Jerk during override of speed limiter */
#define Lim_max_neg_grad_override LIM_OVERRIDE_MAX_NEG_GRAD
extern Cc_const volatile gradient_t LIM_OVERRIDE_MAX_NEG_GRAD;

/*! [m/s�]  */
#define Lim_min_grad_rest LIM_REST_MIN_GRAD
extern Cc_const volatile gradient_t LIM_REST_MIN_GRAD;

/*! [m/s�] Maximum value for the commanded acceleration during speed limiter */
#define Lim_accel_limit LIM_ACCEL_LIMIT
extern Cc_const volatile acceleration_t LIM_ACCEL_LIMIT;

/* [m/s�] Maximum value for the commanded deceleration during speed limiter */
#define Lim_decel_limit LIM_DECEL_LIMIT
extern Cc_const volatile acceleration_t LIM_DECEL_LIMIT;

/*! [km/h] Minimum speed limiter set speed */
#define Lim_min_setspeed_kmh LIM_MIN_VSET_KMH
extern Cc_const volatile setspeed_t LIM_MIN_VSET_KMH;

/*! [km/h] Maximum speed limiter set speed */
#define Lim_max_setspeed_kmh LIM_MAX_VSET_KMH
extern Cc_const volatile setspeed_t LIM_MAX_VSET_KMH;

/*! [Mph] Minimum speed limiter set speed */
#define Lim_min_setspeed_mph LIM_MIN_VSET_MPH
extern Cc_const volatile setspeed_t LIM_MIN_VSET_MPH;

/*! [Mph] Maximum speed limiter set speed */
#define Lim_max_setspeed_mph LIM_MAX_VSET_MPH
extern Cc_const volatile setspeed_t LIM_MAX_VSET_MPH;

/*! [km/h] Threshold for engagement of permanent limiter below driver selected
 * maximum speed */
#define Plim_threshold_kmh PLIM_THRES_KMH
extern Cc_const volatile setspeed_t PLIM_THRES_KMH;

/*! [km/h] Hysteresis for disengagement of permanent limiter */
#define Plim_hyst_kmh PLIM_HYST_KMH
extern Cc_const volatile setspeed_t PLIM_HYST_KMH;

/*! [Mph] Threshold for engagement of permanent limiter below driver selected
 * maximum speed */
#define Plim_threshold_mph PLIM_THRES_MPH
extern Cc_const volatile setspeed_t PLIM_THRES_MPH;

/*! [Mph] Hysteresis for disengagement of permanent limiter */
#define Plim_hyst_mph PLIM_HYST_MPH
extern Cc_const volatile setspeed_t PLIM_HYST_MPH;

/*! [Mph] Step for Speed increment / Decreasement level 1 */
#define Cc_setspeed_step_level_1_mph VLC_VSET_STEP_LEVEL_1_MPH
extern Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_1_MPH;

/*! [Mph] Step for Speed increment / Decreasement level 2 */
#define Cc_setspeed_step_level_2_mph VLC_VSET_STEP_LEVEL_2_MPH
extern Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_2_MPH;

/*! [km/h] Step for Speed increment / Decreasement level 1 */
#define Cc_setspeed_step_level_1_kmh VLC_VSET_STEP_LEVEL_1_KMH
extern Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_1_KMH;

/*! [km/h] Step for Speed increment / Decreasement level2 */
#define Cc_setspeed_step_level_2_kmh VLC_VSET_STEP_LEVEL_2_KMH
extern Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_2_KMH;

/*! [km/h] Vehicle speed threshold. Above this threshold CC will be disengaged.
 * Value 255 is default-> no disengagement */
#define Cc_max_speed_kmh VLC_VMAX_KMH
extern Cc_const volatile setspeed_t VLC_VMAX_KMH;

/*! [Mph] Vehicle speed threshold. Above this threshold CC will be disengaged.
 * Value 255 is default-> no disengagement */
#define Cc_max_speed_mph VLC_VMAX_MPH
extern Cc_const volatile setspeed_t VLC_VMAX_MPH;

/*! [ms] Time until the hold and repeat function of the cruise switches is
 * activated */
#define Cc_start_repeat_function_time VLC_START_T_REP_FUNC
extern Cc_const volatile times_t VLC_START_T_REP_FUNC;

/*! [ms] Time until the hold and repeat function of the set switches is
 * activated */
#define Cc_start_repeat_function_time_2 VLC_START_T_REP_FUNC_2
extern Cc_const volatile times_t VLC_START_T_REP_FUNC_2;

/*! [ms] Time in which the cruise switch activation (hold) will be repeated */
#define Cc_repeat_function_time VLC_T_REP_FUNC
extern Cc_const volatile times_t VLC_T_REP_FUNC;

/*! [s] Time in that the driver must apply the RESUME-switch for a smooth drive
 * off after complete standstill */
#define Cc_default_drive_off_smooth_time 180000u
#define Cc_drive_off_smooth_time VLC_T_DRIVE_OFF_SMOOTH
extern Cc_const volatile times_long_t VLC_T_DRIVE_OFF_SMOOTH;

/*! [m/s�] Minimum gradient for actual acceleration at cruise engagement */
#define Cc_min_engage_accel_gradient VLC_MIN_ENGAGE_A_GRAD
extern Cc_const volatile gradient_t VLC_MIN_ENGAGE_A_GRAD;

/*! [m/s�] Maximum gradient for actual acceleration at cruise engagement */
#define Cc_max_engage_accel_gradient VLC_MAX_ENGAGE_A_GRAD
extern Cc_const volatile gradient_t VLC_MAX_ENGAGE_A_GRAD;

/*! [M/h] Minimum cruise set speed */
#define Cc_min_setspeed_mph VLC_MIN_VSET_MPH
extern Cc_const volatile setspeed_t VLC_MIN_VSET_MPH;

/*! [M/h] Minimum cruise set speed in USA */
#define Cc_min_setspeed_usa_mph VLC_MIN_VSET_USA_MPH
extern Cc_const volatile setspeed_t VLC_MIN_VSET_USA_MPH;

/*! [M/h] Maximum cruise set speed */
#define Cc_max_setspeed_mph VLC_MAX_VSET_MPH
extern Cc_const volatile setspeed_t VLC_MAX_VSET_MPH;

/*! [M/h] Threshold for shutoff below minimum set speed */
#define Cc_disengage_threshold_mph VLC_OFF_THRES_MPH
extern Cc_const volatile setspeed_t VLC_OFF_THRES_MPH;

/*! [M/h] Threshold for shutoff below minimum set speed for cruise control
 * (without relevant object)*/
#define Cc_disengage_threshold_cruise_mph VLC_OFF_THRES_CRUISE_MPH
extern Cc_const volatile setspeed_t VLC_OFF_THRES_CRUISE_MPH;

/*! [-] filter time for lateral acceleration limitation at positive gradient of
 * lateral accel */
#define Cc_pos_lateral_accel_filter_time VLC_TFILT_POS_ALAT
extern Cc_const volatile times_t VLC_TFILT_POS_ALAT;

/*! [-] filter time for lateral acceleration limitation at negative gradient of
 * lateral accel */
#define Cc_neg_lateral_accel_filter_time VLC_TFILT_NEG_ALAT
extern Cc_const volatile times_t VLC_TFILT_NEG_ALAT;

/*! [ms] Max. duration of brake only mode (69.444 - 2.777) [m/s] / 0.5 [m/s�] =
 * 133.333 [s] max. = 65534 */
#define Cc_max_decel_only_time VLC_TMAX_DECEL_ONLY
extern Cc_const volatile times_t VLC_TMAX_DECEL_ONLY;

/*! [ms] Max. time till disengagement of cruise, when gear is clutch pressed
 * (manual gear only) */
#define Cc_shift_active_inhibit_time VLC_T_SHIFT_ACT_INH
extern Cc_const volatile times_t VLC_T_SHIFT_ACT_INH;

/*! [ms] Max. time Limit deceleration after Override is active */
#define Cc_max_decel_limit_time VLC_TMAX_DECEL_LIMIT
extern Cc_const volatile times_t VLC_TMAX_DECEL_LIMIT;

/*! [ms] Max. time ABS is active until CC is disengaged */
#define Cc_abs_act_inhibit_time VLC_T_ABS_ACT_INH
extern Cc_const volatile times_t VLC_T_ABS_ACT_INH;

/*! [ms] Max. time ESP is active until CC is disengaged */
#define Cc_esp_act_inhibit_time VLC_T_ESP_ACT_INH
extern Cc_const volatile times_t VLC_T_ESP_ACT_INH;

/*! [ms] Max. time TCS is active until CC is disengaged */
#define Cc_tcs_act_inhibit_time VLC_T_TCS_ACT_INH
extern Cc_const volatile times_t VLC_T_TCS_ACT_INH;

/*! [m/s�] Maximum deceleration request during acceleration mode */
#define Cc_max_decel_during_accel_mode VLC_MAX_DECEL_DURING_ACCEL_MODE
extern Cc_const volatile acceleration_t VLC_MAX_DECEL_DURING_ACCEL_MODE;

/*! [m/s�] Maximum acceleration request during deceleration mode */
#define Cc_max_accel_during_decel_mode VLC_MAX_ACCEL_DURING_DECEL_MODE
extern Cc_const volatile acceleration_t VLC_MAX_ACCEL_DURING_DECEL_MODE;

/*! [-] lowest possible jerk for jerk limitation during smooth jerk
 * functionality*/
#define Cc_smooth_jerk_min_gradient VLC_SMOOTH_JERK_MIN_GRADIENT
extern Cc_const volatile gradient_t VLC_SMOOTH_JERK_MIN_GRADIENT;

/*! [m/s] lowest velociy where to perform positive smooth gradients (for starts
 * behind a vehicle)*/
#define Cc_smooth_gradient_pos_min_velocity VLC_SMOOTH_GRADIENT_POS_MIN_VELOCITY
extern Cc_const volatile velocity_t VLC_SMOOTH_GRADIENT_POS_MIN_VELOCITY;

/*! [s] defines the time when the commanded acceleration shall be reached*/
#define Cc_smooth_gradient_pos_time pVLC_SmoothGradientPosTime
#define Cc_smooth_gradient_pos_time_points ((uint16)2)
extern Cc_const volatile sint16* pVLC_SmoothGradientPosTime;

/*! [s] defines the time when the commanded acceleration shall be reached*/
#define Cc_smooth_gradient_neg_time pVLC_SmoothGradientNegTime
#define Cc_smooth_gradient_neg_time_points ((uint16)2)
extern Cc_const volatile sint16* pVLC_SmoothGradientNegTime;

/*! [s] defines the time after engagement where override detection is not used
 * to avoid going into override directly after engagement*/
#define Cc_engage_override_time VLC_ENGAGE_OVERRTIDE_TIME
extern Cc_const volatile times_t VLC_ENGAGE_OVERRTIDE_TIME;

/*! [km/h] minimum value for recommended speed */
#define Cc_min_recommended_speed_kmh VLC_MIN_RECOMMENDED_SPEED_KMH
extern Cc_const volatile setspeed_t VLC_MIN_RECOMMENDED_SPEED_KMH;

/*! [mph] minimum value for recommended speed */
#define Cc_min_recommended_speed_mph VLC_MIN_RECOMMENDED_SPEED_MPH
extern Cc_const volatile setspeed_t VLC_MIN_RECOMMENDED_SPEED_MPH;

/*! [%] hysteresis for recommended speed */
#define Cc_recommended_speed_hyst VLC_RECOMMENDED_SPEED_HYST
extern Cc_const volatile percentage_t VLC_RECOMMENDED_SPEED_HYST;

/*! [-] number of intervals for recommended speed */
#define Cc_recommended_speed_intervals VLC_RECOMMENDED_SPEED_INTERVALS
extern Cc_const volatile uint8 VLC_RECOMMENDED_SPEED_INTERVALS;

/* [m/s�] max acceleration for the disengagement ramp*/
#define Acc_max_accel_disengage AVLC_MAX_ACCEL_DISENGAGE
extern Cc_const volatile acceleration_t AVLC_MAX_ACCEL_DISENGAGE;

/* [m/s�] min acceleration for the disengagement ramp*/
#define Acc_min_accel_disengage AVLC_MIN_ACCEL_DISENGAGE
extern Cc_const volatile acceleration_t AVLC_MIN_ACCEL_DISENGAGE;

/*******************************************************************************************/
/*                       A P P L I C A T I O N   P A R A M E T E R S */
/*******************************************************************************************/

/*! [s] Time within the driver has to activate the RESUME-switch twice to
 * confirm drive off after complete standstill */
#define Cc_default_drive_off_confirmation_time ((times_t)1000)
#define Cc_drive_off_confirmation_time VLC_T_DRIVE_OFF_CONF
extern Cc_const volatile times_t VLC_T_DRIVE_OFF_CONF;

/*! [s] Sending time for drive_off signal */
#define Cc_drive_off_set_time VLC_T_DRIVE_OFF_SET
extern Cc_const volatile times_t VLC_T_DRIVE_OFF_SET;

/*! [s] Time the standsill for the AVLC_INPUT_DATA.LODM_STAT.STANDSTILL is
 * delayed*/
#define Cc_standstill_delay_time VLC_T_STANDSTILL_DELAY
extern Cc_const volatile times_t VLC_T_STANDSTILL_DELAY;

#define Cc_moving_time VLC_T_MOVING
extern Cc_const volatile times_t VLC_T_MOVING;

/*! [s] Time Takeover shall be at least displayed to the driver */
#define Cc_display_takeover_time VLC_T_DISPLAY_TAKEOVER_TIME
extern Cc_const volatile times_t VLC_T_DISPLAY_TAKEOVER_TIME;

/*! [m/s] Lower speed threshold for sending cancel request to external
 * conventional cruise controller*/
#define Cc_max_operating_speed_acc VLC_MAX_OPERATING_SPEED_ACC
extern Cc_const volatile velocity_t VLC_MAX_OPERATING_SPEED_ACC;

/*! [m/s] Lower speed threshold for sending cancel request to external
 * conventional cruise controller*/
#define Cc_max_operating_speed_fsra VLC_MAX_OPERATING_SPEED_FSRA
extern Cc_const volatile velocity_t VLC_MAX_OPERATING_SPEED_FSRA;

/*! [m/s] Lower speed threshold for sending cancel request to external
 * conventional cruise controller*/
#define Cc_min_operating_speed_acc VLC_MIN_OPERATING_SPEED_ACC
extern Cc_const volatile velocity_t VLC_MIN_OPERATING_SPEED_ACC;

/*! [m/s] Lower speed threshold for sending cancel request to external
 * conventional cruise controller*/
#define Cc_min_operating_speed_fsra VLC_MIN_OPERATING_SPEED_FSRA
extern Cc_const volatile velocity_t VLC_MIN_OPERATING_SPEED_FSRA;

/*! [s] Time to pass untill cancellation request is send out when target is lost
    and ego velocity is below VLC_CANCEL_REQUEST_MIN_SPEED_FSRA_VLC_MODE*/
#define Cc_fsra_obj_loss_debounce_time VLC_FSRA_OBJ_LOSS_DEBOUNCE_TIME
extern Cc_const volatile times_t VLC_FSRA_OBJ_LOSS_DEBOUNCE_TIME;

/*! [s] Time duration for cancellation warning request*/
#define Cc_fsra_cancel_warning_time VLC_FSRA_CANCEL_WARNING_TIME
extern Cc_const volatile times_t VLC_FSRA_CANCEL_WARNING_TIME;

/*! [%] internal translation of customer specific gap setting to a percentage
 * value*/
#define Cc_headway_setting_1_threshold VLC_HEADWAY_SETTING_1_THRESHOLD
extern Cc_const volatile percentage_t VLC_HEADWAY_SETTING_1_THRESHOLD;

/*! [%] internal translation of customer specific gap setting to a percentage
 * value*/
#define Cc_headway_setting_2_threshold VLC_HEADWAY_SETTING_2_THRESHOLD
extern Cc_const volatile percentage_t VLC_HEADWAY_SETTING_2_THRESHOLD;

/*! [%] internal translation of customer specific gap setting to a percentage
 * value*/
#define Cc_headway_setting_3_threshold VLC_HEADWAY_SETTING_3_THRESHOLD
extern Cc_const volatile percentage_t VLC_HEADWAY_SETTING_3_THRESHOLD;

/*! [%] internal translation of customer specific gap setting to a percentage
 * value*/
#define Cc_headway_setting_4_threshold VLC_HEADWAY_SETTING_4_THRESHOLD
extern Cc_const volatile percentage_t VLC_HEADWAY_SETTING_4_THRESHOLD;

/*! [%] internal translation of customer specific gap setting to a percentage
 * value*/
#define Cc_headway_setting_5_threshold VLC_HEADWAY_SETTING_5_THRESHOLD
extern Cc_const volatile percentage_t VLC_HEADWAY_SETTING_5_THRESHOLD;

/* [m/s�] acceleration treshold that has to exceeded in order to set the
   the default acceleration VLC_ACCEL_DURING_EXT_CRUISE_MODE */
#define Cc_decel_threshold_for_ext_cruise_mode \
    VLC_DECEL_THRESHOLD_FOR_EXT_CRUISE_MODE
extern Cc_const volatile acceleration_t VLC_DECEL_THRESHOLD_FOR_EXT_CRUISE_MODE;

/* [m/s�] default acceleration request when no relevant target is detected -
 * linear*/
#define Cc_accel_during_ext_cruise_mode_linear \
    VLC_ACCEL_DURING_EXT_CRUISE_MODE_LINEAR
#define Cc_accel_during_ext_cruise_linear_points ((uint16)6)
extern Cc_const volatile acceleration_t VLC_ACCEL_DURING_EXT_CRUISE_MODE_LINEAR
    [2 * Cc_accel_during_ext_cruise_linear_points];

/* [m/s�] default acceleration request when no relevant target is detected -
 * under 15 and under 30 kph*/
#define Cc_speed_threshold_15kph VLC_SPEED_THRESHOLD_15KPH
extern Cc_const volatile velocity_t VLC_SPEED_THRESHOLD_15KPH;

#define Cc_speed_threshold_30kph VLC_SPEED_THRESHOLD_30KPH
extern Cc_const volatile velocity_t VLC_SPEED_THRESHOLD_30KPH;

#define Cc_accel_threshold_15kph VLC_ACCEL_THRESHOLD_15KPH
extern Cc_const volatile acceleration_t VLC_ACCEL_THRESHOLD_15KPH;

#define Cc_accel_threshold_30kph VLC_ACCEL_THRESHOLD_30KPH
extern Cc_const volatile acceleration_t VLC_ACCEL_THRESHOLD_30KPH;

/*! [m/s] If vehicle speed falls below this threshold the soft stop flag can be
 * set*/
#define Cc_max_softstop_request_velocity VLC_MAX_SOFTSTOP_REQUEST_VELOCITY
extern Cc_const volatile velocity_t VLC_MAX_SOFTSTOP_REQUEST_VELOCITY;

/*! [cm] If distance to object is lower than this threshold the soft stop flag
 * can be set*/
#define Cc_max_softstop_request_distance VLC_MAX_SOFTSTOP_REQUEST_DISTANCE
extern Cc_const volatile distance_t VLC_MAX_SOFTSTOP_REQUEST_DISTANCE;

#define Cc_gain_grad VLC_GAIN_GRAD
#define Cc_gain_grad_points ((uint16)4)
extern Cc_const volatile factor_t VLC_GAIN_GRAD[2 * Cc_gain_grad_points];

#endif /* CCPAR_INCLUDED */
