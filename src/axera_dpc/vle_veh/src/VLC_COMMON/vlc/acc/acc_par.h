/*! \file **********************************************************************

  COMPONENT:              ACC (Adaptive Cruise Control)

  MODULENAME:             acc_par.h

  @brief                  Controller Parameter definition


*****************************************************************************/

/* conditional include */
#ifndef ACCPAR_INCLUDED
#define ACCPAR_INCLUDED

#include "vlc_glob_ext.h"
#include "vlc_long_cfg.h"
#include "acc_cfg.h"

#if (defined(_MSC_VER) && defined(_WIN32))
#define Acc_const DLLEXPORT const
#else
#define Acc_const const
#endif

extern void AVLC_SELECT_PARAM_SET(uint8 Fct_General_FnSwitchBits);

/*! [m/s2] Minimum value for commanded acceleration when a reason for loss of
 * object could be detected */
#define Acc_min_accel_object_lost AVLC_AMIN_OBJECT_LOST
extern Acc_const volatile acceleration_t AVLC_AMIN_OBJECT_LOST;

/*! [m/s2] Maximum value for commanded acceleration when a reason for loss of
 * object could be detected */
#define Acc_max_accel_object_lost AVLC_AMAX_OBJECT_LOST
extern Acc_const volatile acceleration_t AVLC_AMAX_OBJECT_LOST;

/*! [m/s2] Minimum value for commanded acceleration to statonary objects */
#define Acc_min_accel_object_stationary AVLC_MIN_ACCEL_OBJECT_STATIONARY
extern Acc_const volatile acceleration_t AVLC_MIN_ACCEL_OBJECT_STATIONARY;

/*! Factor for Maximum increase of acceleration request during follow mode
 * in eco mode)*/
#define Acc_pos_accel_grad_eco_fac AVLC_POS_GRAD_ECO_FAC
extern Acc_const volatile factor_t AVLC_POS_GRAD_ECO_FAC;

/*! Factor for Maximum increase of acceleration request during follow mode
 *in sport mode)*/
#define Acc_pos_accel_grad_sport_fac AVLC_POS_GRAD_SPORT_FAC
extern Acc_const volatile factor_t AVLC_POS_GRAD_SPORT_FAC;

/*! [m/s2] Maximum decrease of acceleration request during follow mode (speed
 * dependent)*/
#define Acc_neg_grad_points (uint16)2
#define Acc_max_neg_grad AVLC_MAX_NEG_GRAD
extern Acc_const volatile acceleration_t
    AVLC_MAX_NEG_GRAD[2 * Acc_neg_grad_points];

/*! [m/s2] Maximum increase of acceleration request during follow mode for
 * positive values(speed dependent)*/
#define Acc_pos_grad_pos_accel_points (uint16)2
#define Acc_max_pos_grad_pos_accel AVLC_MAX_POS_GRAD_POS_ACCEL
extern Acc_const volatile acceleration_t
    AVLC_MAX_POS_GRAD_POS_ACCEL[2 * Acc_pos_grad_pos_accel_points];

/*! [m/s2] Maximum increase of acceleration request during follow mode for
 * negative values(speed dependent)*/
#define Acc_pos_grad_neg_accel_points (uint16)2
#define Acc_max_pos_grad_neg_accel AVLC_MAX_POS_GRAD_NEG_ACCEL
extern Acc_const volatile acceleration_t
    AVLC_MAX_POS_GRAD_NEG_ACCEL[2 * Acc_pos_grad_neg_accel_points];

/*! [m/s] Minimum estimated RelativeSteed Threshold for Takeover */
#define Acc_vrel_estim_min AVLC_VREL_ESTIM_MIN
#define Acc_vrel_estim_min_points 4
extern Acc_const volatile velocity_t
    AVLC_VREL_ESTIM_MIN[2 * Acc_vrel_estim_min_points];

/*! [m/s] Maximum estimated RelativeSteed Threshold for Takeover */
#define Acc_vrel_estim_max AVLC_VREL_ESTIM_MAX
#define Acc_vrel_estim_max_points 4
extern Acc_const volatile velocity_t
    AVLC_VREL_ESTIM_MAX[2 * Acc_vrel_estim_max_points];

/*! [s] Time gap hysteresis to end Takeover */
#define Acc_end_alert_hyst_factor AVLC_END_ALERT_HYST_FACTOR
extern Acc_const volatile factor_t AVLC_END_ALERT_HYST_FACTOR;

/*! [s] Takeover threshold hysteresis when engaging below Maximum Takeover
 * Threshold */
#define Acc_alert_thres_hyst AVLC_ALERT_THRES_HYST
extern Acc_const volatile times_t AVLC_ALERT_THRES_HYST;

/*! [s] Estimated Headway max Simulation Time */
#define Acc_alert_max_sim_time AVLC_ALERT_MAX_SIM_TIME
#define Acc_max_sim_time_points 2
extern Acc_const volatile times_t
    AVLC_ALERT_MAX_SIM_TIME[2 * Acc_max_sim_time_points];

/*! [s] Estimated Headway Simulation TimeStep */
#define Acc_alert_sim_time_step AVLC_ALERT_SIM_TIME_STEP
extern Acc_const volatile times_t AVLC_ALERT_SIM_TIME_STEP;

/*! [s] Filter time for acceleration of vehicle for alert simulation*/
#define Acc_alert_sim_vehicle_filter_time AVLC_ALERT_SIM_VEHICLE_FILTER_TIME
extern Acc_const volatile times_t AVLC_ALERT_SIM_VEHICLE_FILTER_TIME;

/*! [s] Estimated Headway Simulation TimeStep */
#define Acc_alert_supress_alert_time_points 2
#define Acc_alert_supress_alert_time AVLC_ALERT_SUPRESS_ALERT_TIME
extern Acc_const volatile times_t
    AVLC_ALERT_SUPRESS_ALERT_TIME[2 * Acc_alert_supress_alert_time_points];

/*! minimum output time, which ACC alert shall be sent to avoid to short
 * acoustical and visual signals*/
#define Acc_alert_min_output_time AVLC_ALERT_MIN_OUTPUT_TIME
extern Acc_const volatile times_t AVLC_ALERT_MIN_OUTPUT_TIME;

#define Acc_drive_off_time AVLC_DRIVE_OFF_TIME
extern Acc_const volatile times_t AVLC_DRIVE_OFF_TIME;

#define Acc_drive_off_distance AVLC_DRIVE_OFF_DISTANCE
extern Acc_const volatile distance_t AVLC_DRIVE_OFF_DISTANCE;

/*! [m] max distance allowed for using TTS acceleration */
#define Acc_tts_acceleration_max_dist AVLC_TTS_ACC_MAX_DIST
#define Acc_tts_acceleration_max_dist_points 4
extern Acc_const volatile distance_t AVLC_TTS_ACC_MAX_DIST[];

/*! [km/h] Maximum set speed for ACC */
#define Acc_max_setspeed_kmh AVLC_MAX_VSET_KMH
extern Acc_const volatile setspeed_t AVLC_MAX_VSET_KMH;

/*! [Mph] Maximum set speed for ACC */
#define Acc_max_setspeed_mph AVLC_MAX_VSET_MPH
extern Acc_const volatile setspeed_t AVLC_MAX_VSET_MPH;

/*! [m/s2] max acceleration for ACC FSR depending on vehicle speed*/
#define Acc_fsr_max_acceleration AVLC_FSR_MAX_ACCEL
#define Acc_fsr_max_acceleration_points 4
extern Acc_const volatile acceleration_t AVLC_FSR_MAX_ACCEL[];

/*! [m/s2] max deceleration for ACC FSR depending on vehicle speed*/
#define Acc_fsr_max_deceleration AVLC_FSR_MAX_DECEL
#define Acc_fsr_max_deceleration_points 2
extern Acc_const volatile acceleration_t AVLC_FSR_MAX_DECEL[];

// Middle Value of Headway Setting Array.
#define Acc_mid_headwaysetting AVLC_MID_HEADWAYSETTING
extern Acc_const volatile uint8 AVLC_MID_HEADWAYSETTING;

/*! factor for request stop distance */
#define Acc_request_stop_distance_factor AVLC_REQUEST_STOP_DISTANCE_FACTOR
#define Acc_request_stop_distance_factor_points ((uint16)2)
extern Acc_const volatile acceleration_t
    AVLC_REQUEST_STOP_DISTANCE_FACTOR[2 *
                                      Acc_request_stop_distance_factor_points];

/*! [m] min following distance (speed dependent) the driver can set (headway
 * setting 0%)*/
#define Acc_headway_min_dist_points ((uint16)5)
#define Acc_headway_min_dist AVLC_pHeadwayMinDist
extern Acc_const volatile distance_t* AVLC_pHeadwayMinDist;

/*! [m] max following distance (speed dependent) the driver can set (headway
 * setting 100%)*/
#define Acc_headway_max_dist_points ((uint16)5)
#define Acc_headway_max_dist AVLC_pHeadwayMaxDist
extern Acc_const volatile distance_t* AVLC_pHeadwayMaxDist;

/*! [m/s2] decreasing the minimum acceleration depending on vehicle speed*/
#define Acc_decrease_minaccel_curve AVLC_DECREASE_MINACCEL_CURVE
#define Acc_decrease_minaccel_points ((uint16)3)
extern Acc_const volatile acceleration_t
    AVLC_DECREASE_MINACCEL_CURVE[2 * Acc_decrease_minaccel_points];

/*! [m/s2] increasing the maximum acceleration depending on vehicle speed*/
#define Acc_increase_maxaccel_curve AVLC_INCREASE_MAXACCEL_CURVE
#define Acc_increase_maxaccel_points ((uint16)3)
extern Acc_const volatile acceleration_t
    AVLC_INCREASE_MAXACCEL_CURVE[2 * Acc_increase_maxaccel_points];

/*! [km/h] when the vehicle speed goes below minimum set speed minus this value,
 * the adaptive cruise control (10kmh) is disengaged */
#define Acc_fsr_disengage_threshold_kmh AVLC_FSR_OFF_THRES_KMH
extern Acc_const volatile setspeed_t AVLC_FSR_OFF_THRES_KMH;

/*! [km/h] minimum allowed engagement speed for 10kmh function*/
#define Acc_fsr_min_v_engage_kmh AVLC_FSR_MIN_V_ENGAGE_KMH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_KMH;

/*! [km/h] minimum set speed for adaptive cruise control function (10kmh
 * function)*/
#define Acc_fsr_min_setspeed_kmh AVLC_FSR_MIN_VSET_KMH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_KMH;

/*! [m] min alert distance (estimated host speed dependend)*/
// #define Acc_min_alert_thres_host_speed_points 5
// #define Acc_min_alert_thres_rel_speed_points 5

// #define Acc_min_alert_thres AVLC_MIN_ALERT_THRES
// extern Acc_const volatile float32
//     AVLC_MIN_ALERT_THRES[Acc_min_alert_thres_host_speed_points *
//                          Acc_min_alert_thres_rel_speed_points];

// #define Acc_min_alert_thres_host_speed AVLC_MIN_ALERT_HOST_SPEED
// extern Acc_const volatile float32
//     AVLC_MIN_ALERT_HOST_SPEED[Acc_min_alert_thres_host_speed_points];

// #define Acc_min_alert_thres_rel_speed AVLC_MIN_ALERT_REL_SPEED
// extern Acc_const volatile float32
//     AVLC_MIN_ALERT_REL_SPEED[Acc_min_alert_thres_rel_speed_points];

// /*! [m] max alert distance (estimated host speed dependend)*/
// #define Acc_max_alert_thres_host_speed_points 5
// #define Acc_max_alert_thres_rel_speed_points 5

// #define Acc_max_alert_thres AVLC_MAX_ALERT_THRES
// extern Acc_const volatile float32
//     AVLC_MAX_ALERT_THRES[Acc_max_alert_thres_host_speed_points *
//                          Acc_max_alert_thres_rel_speed_points];

// #define Acc_max_alert_thres_host_speed AVLC_MAX_ALERT_HOST_SPEED
// extern Acc_const volatile float32
//     AVLC_MAX_ALERT_HOST_SPEED[Acc_max_alert_thres_host_speed_points];

// #define Acc_max_alert_thres_rel_speed AVLC_MAX_ALERT_REL_SPEED
// extern Acc_const volatile float32
//     AVLC_MAX_ALERT_REL_SPEED[Acc_max_alert_thres_rel_speed_points];

/*! [m] min alert time gap (estimated host speed dependend)*/
#define Acc_min_alert_thres_time_gap_points 3
#define Acc_min_alert_thres_time_gap AVLC_MIN_ALERT_THRES_TIME_GAP
extern Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_TIME_GAP[2 * Acc_min_alert_thres_time_gap_points];

/*! [m] min alert ttc (estimated host speed dependend)*/
#define Acc_min_alert_thres_ttc_points 3
#define Acc_min_alert_thres_ttc AVLC_MIN_ALERT_THRES_TTC
extern Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_TTC[2 * Acc_min_alert_thres_ttc_points];

/*! [m] min alert accel (estimated host speed dependend)*/
#define Acc_min_alert_thres_accel_points 2
#define Acc_min_alert_thres_accel AVLC_MIN_ALERT_THRES_ACCEL
extern Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_ACCEL[2 * Acc_min_alert_thres_accel_points];

/*! [m] max alert time gap (estimated host speed dependend)*/
#define Acc_max_alert_thres_time_gap_points 3
#define Acc_max_alert_thres_time_gap AVLC_MAX_ALERT_THRES_TIME_GAP
extern Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_TIME_GAP[2 * Acc_max_alert_thres_time_gap_points];

/*! [m] max alert ttc (estimated host speed dependend)*/
#define Acc_max_alert_thres_ttc_points 3
#define Acc_max_alert_thres_ttc AVLC_MAX_ALERT_THRES_TTC
extern Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_TTC[2 * Acc_max_alert_thres_ttc_points];

/*! [m] max alert accel (estimated host speed dependend)*/
#define Acc_max_alert_thres_accel_points 2
#define Acc_max_alert_thres_accel AVLC_MAX_ALERT_THRES_ACCEL
extern Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_ACCEL[2 * Acc_max_alert_thres_accel_points];

/*! modfiy estimated distance for new or changed control object */
#define Acc_headway_add_factor AVLC_HEADWAY_ADD_FACTOR
#define Acc_headway_add_factor_points 3
extern Acc_const volatile factor_t
    AVLC_HEADWAY_ADD_FACTOR[2 * Acc_headway_add_factor_points];

/*! [mph] when the vehicle speed goes below minimum set speed minus this value,
 * the adaptive cruise control (10kmh) is disengaged */
#define Acc_fsr_disengage_threshold_mph AVLC_FSR_OFF_THRES_MPH
extern Acc_const volatile setspeed_t AVLC_FSR_OFF_THRES_MPH;

/*! [mph] minimum allowed engagement speed for 10kmh function*/
#define Acc_fsr_min_v_engage_mph AVLC_FSR_MIN_V_ENGAGE_MPH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_MPH;

/*! [mph] minimum set speed for adaptive cruise control function (10kmh
 * function)*/
#define Acc_fsr_min_setspeed_mph AVLC_FSR_MIN_VSET_MPH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_MPH;

/*! [mph] minimum allowed engagement speed for 10kmh function (usa)*/
#define Acc_fsr_min_v_engage_usa_mph AVLC_FSR_MIN_V_ENGAGE_USA_MPH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_USA_MPH;

/*! [mph] minimum set speed for adaptive cruise control function (10kmh
 * function) (usa)*/
#define Acc_fsr_min_setspeed_usa_mph AVLC_FSR_MIN_VSET_USA_MPH
extern Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_USA_MPH;

/*! [m/s] speed where standstill is assumed */
#define Acc_stopped_speed AVLC_STOPPED_SPEED
extern Acc_const volatile velocity_t AVLC_STOPPED_SPEED;

/*! [m/s] speed where object standstill is assumed */
#define Acc_obj_stopped_speed AVLC_OBJ_STOPPED_SPEED
extern Acc_const volatile velocity_t AVLC_OBJ_STOPPED_SPEED;

/*! [m] Drive off hysteresis */
#define Acc_drive_off_dist_hyst AVLC_DRIVE_OFF_DIST_HYST
extern Acc_const volatile distance_t AVLC_DRIVE_OFF_DIST_HYST;

/*! [m/s2] deceleration of a stopped object for fuzzy rules*/
#define Acc_stopped_obj_decel AVLC_STOPPED_OBJ_DECEL
extern Acc_const volatile acceleration_t AVLC_STOPPED_OBJ_DECEL;

/*! [m/s] maximum speed for a "crawling" vehicle*/
#define Acc_crawl_max_velocity AVLC_CRAWL_MAX_VELOCITY
extern Acc_const volatile velocity_t AVLC_CRAWL_MAX_VELOCITY;

/*! [s] maximum time the target vehicle may need to come to a full stop to call
 * this situation a stop situation*/
#define Acc_si_max_time_to_stop AVLC_SI_MAX_TIME_TO_STOP
extern Acc_const volatile times_t AVLC_SI_MAX_TIME_TO_STOP;

/*! [s] maximum time the target vehicle may need to come to a full stop to call
 * this situation a stop situation if object if front of the target has already
 * stopped*/
#define Acc_si_max_time_to_stop_2obj AVLC_SI_MAX_TIME_TO_STOP_2OBJ
extern Acc_const volatile times_t AVLC_SI_MAX_TIME_TO_STOP_2OBJ;

/*! [m/s2] object acceleration for a "go" signal in low speed situations*/
#define Acc_si_min_go_accel AVLC_SI_MIN_GO_ACCEL
extern Acc_const volatile acceleration_t AVLC_SI_MIN_GO_ACCEL;

/*! [m/s2] object acceleration for a "go" signal in low speed situations if
 * object in front of target is faster and accelerating too*/
#define Acc_si_min_go_accel_2obj AVLC_SI_MIN_GO_ACCEL_2OBJ
extern Acc_const volatile acceleration_t AVLC_SI_MIN_GO_ACCEL_2OBJ;

/*! [%] above this lane change probability the situation can be classified as
 * lane change/overtake situation*/
#define Acc_si_lcprob_ovrtk AVLC_SI_LCPROB_OVRTK
extern Acc_const volatile percentage_t AVLC_SI_LCPROB_OVRTK;

/*! [m/s2] max allowed deceleration to objects being in front of the next object
 * in lane*/
#define Acc_max_decel_hidden_object_points 3
#define Acc_max_decel_hidden_object AVLC_MAX_DECEL_HIDDEN_OBJECT
extern Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_HIDDEN_OBJECT[2 * Acc_max_decel_hidden_object_points];

/*! [m/s2] max allowed deceleration to objects in adjacent lane when ego vehicle changing lane */
#define Acc_max_decel_adjacent_object_points 2
#define Acc_max_decel_adjacent_object AVLC_MAX_DECEL_ADJACENT_OBJECT
extern Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_ADJACENT_OBJECT[2 * Acc_max_decel_adjacent_object_points];

/*! [s] max relevant ttc for check if object will come to full stop while ttc*/
#define Acc_si_max_relevant_ttc AVLC_SI_MAX_RELEVANT_TTC
extern Acc_const volatile times_t AVLC_SI_MAX_RELEVANT_TTC;

/*! [s] time for predict lateral position into the near future (using v_rel_y)*/
#define Acc_predict_lateral_pos_time AVLC_SI_PREDICT_LATERAL_POS_TIME
extern Acc_const volatile times_t AVLC_SI_PREDICT_LATERAL_POS_TIME;

/*! [m] width of the host vehicle*/
#define Acc_host_vehicle_width AVLC_SI_HOST_VEHICLE_WIDTH
extern Acc_const volatile distance_t AVLC_SI_HOST_VEHICLE_WIDTH;

/*! [] criticality factor depending on the current object deceleration */
#define Acc_si_crit_factor_obj_accel AVLC_SI_CRIT_FACTOR_OBJ_ACCEL
#define Acc_si_crit_factor_obj_accel_points ((uint16)4)
extern Acc_const volatile factor_t
    AVLC_SI_CRIT_FACTOR_OBJ_ACCEL[2 * Acc_si_crit_factor_obj_accel_points];

/*! Increase criticality for shortest distance mode */
#define Acc_crit_factor_gain AVLC_CRIT_FACTOR_GAIN
extern Acc_const volatile distance_t AVLC_CRIT_FACTOR_GAIN;

/*! [%] criticality to object based on ttc value*/
#define Acc_si_crit_from_ttc_points 4
#define Acc_si_criticality_from_ttc AVLC_SI_CRITICALITY_FROM_TTC
extern Acc_const volatile factor_t
    AVLC_SI_CRITICALITY_FROM_TTC[2 * Acc_si_crit_from_ttc_points];

/*! [%] Default headway setting value */
#define Acc_default_headway_setting 50u

/*! [-] max allowed pos grad for criticality */
#define Acc_si_crit_pos_grad AVLC_SI_CRIT_POS_GRAD
extern Acc_const volatile gradient_t AVLC_SI_CRIT_POS_GRAD;

/*! [-] max allowed neg grad for criticality */
#define Acc_si_crit_neg_grad AVLC_SI_CRIT_NEG_GRAD
extern Acc_const volatile gradient_t AVLC_SI_CRIT_NEG_GRAD;

/*! [m/s2] maximum object acceleration that is possible*/
#define Acc_max_object_possible_accel AVLC_MAX_OBJECT_POSSIBLE_ACCEL
extern Acc_const volatile acceleration_t AVLC_MAX_OBJECT_POSSIBLE_ACCEL;

/*! [m/s2] minimum object acceleration that is possible*/
#define Acc_min_object_possible_accel AVLC_MIN_OBJECT_POSSIBLE_ACCEL
extern Acc_const volatile acceleration_t AVLC_MIN_OBJECT_POSSIBLE_ACCEL;

/*! [m/s2] object acceleration hysteresis*/
#define Acc_object_hyst_accel AVLC_OBJECT_HYST_ACCEL
extern Acc_const volatile acceleration_t AVLC_OBJECT_HYST_ACCEL;

/*! [m/s2] maximum allowed acceleration that can happen*/
#define Acc_max_allowed_accel AVLC_MAX_ALLOWED_ACCEL
extern Acc_const volatile acceleration_t AVLC_MAX_ALLOWED_ACCEL;

/*! [m/s2] maximum allowed deceleration that can happen*/
#define Acc_max_allowed_decel AVLC_MAX_ALLOWED_DECEL
extern Acc_const volatile acceleration_t AVLC_MAX_ALLOWED_DECEL;

/*! [s] System reaction time when braking requested*/
#define Acc_t_reaction_brake AVLC_T_REACT_BRAKE
extern Acc_const volatile times_t AVLC_T_REACT_BRAKE;

/*! [m/s] recommended velocity as a function of visibility range */
#define Acc_recommended_velocity_curve AVLC_RECOMMENDED_VELOCITY_CURVE
#define Acc_recommended_velocity_curve_points 5
extern Acc_const volatile velocity_t
    AVLC_RECOMMENDED_VELOCITY_CURVE[2 * Acc_recommended_velocity_curve_points];

/*! [s] longitudinal visibility threshold related to vehicle velocity where ACC
 * is disengaged (according to ISO 2[s]) */
#define Acc_t_visibility_disengage AVLC_T_VIS_DISENGAGE
extern Acc_const volatile times_t AVLC_T_VIS_DISENGAGE;

/*! [s] longitudinal visibility threshold related to vehicle velocity where ACC
 * engagement is allowed (according to ISO 2[s]) */
#define Acc_t_visibility_engage AVLC_T_VIS_ENGAGE
extern Acc_const volatile times_t AVLC_T_VIS_ENGAGE;

#define Acc_max_intrusion_factor AVLC_MAX_INTRUSION_FACTOR
#define Acc_max_intrusion_factor_points 4
extern Acc_const volatile factor_t
    AVLC_MAX_INTRUSION_FACTOR[2 * Acc_max_intrusion_factor_points];

/*! [factor] may alert distance (estimated host speed dependend)*/
#define Acc_vrel_intrusion AVLC_VREL_INTRUSION
#define Acc_vrel_intrusion_points 6
extern Acc_const volatile factor_t
    AVLC_VREL_INTRUSION[2 * Acc_vrel_intrusion_points];

/*! [m] independent alert threshold for intrusion calculation */
#define Acc_max_alert_thres_intrusion AVLC_MAX_ALERT_THRES_INTRUSION
#define Acc_max_alert_thres_intrusion_points 5
extern Acc_const volatile distance_t
    AVLC_MAX_ALERT_THRES_INTRUSION[2 * Acc_max_alert_thres_intrusion_points];

/*! [] how much the headway distance to a new object can be increased per
 * second according to actual distance and host speed*/
// #define Acc_headway_increase_grad_host_speed_points 5
// #define Acc_headway_increase_grad_distance_points 5

// #define Acc_headway_increase_grad AVLC_HEADWAY_INCREASE_GRAD
// extern Acc_const volatile float32
//     AVLC_HEADWAY_INCREASE_GRAD[Acc_headway_increase_grad_host_speed_points *
//                                Acc_headway_increase_grad_distance_points];

// #define Acc_headway_increase_grad_host_speed \
//     AVLC_HEADWAY_INCREASE_GRAD_HOST_SPEED
// extern Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_HOST_SPEED
//     [Acc_headway_increase_grad_host_speed_points];

// #define Acc_headway_increase_grad_distance
// AVLC_HEADWAY_INCREASE_GRAD_DISTANCE extern Acc_const volatile float32
// AVLC_HEADWAY_INCREASE_GRAD_DISTANCE
//     [Acc_headway_increase_grad_distance_points];

/*! [s] threshold of headway factor time gap and ttc */
#define Acc_headway_time_thres AVLC_HEADWAY_TIME_THRESHOLD
extern Acc_const volatile times_t AVLC_HEADWAY_TIME_THRESHOLD;

/*! [] how much the headway factor to a new object can be increased per
 * second according to timegap and ttc */
#define Acc_headway_increase_grad_fac_timegap_points 4
#define Acc_headway_increase_grad_fac_ttc_points 4

#define Acc_headway_increase_grad_fac AVLC_HEADWAY_INCREASE_GRAD_FAC
extern Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC
    [Acc_headway_increase_grad_fac_timegap_points *
     Acc_headway_increase_grad_fac_ttc_points];

#define Acc_headway_increase_grad_fac_timegap \
    AVLC_HEADWAY_INCREASE_GRAD_FAC_TIMEGAP
extern Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC_TIMEGAP
    [Acc_headway_increase_grad_fac_timegap_points];

#define Acc_headway_increase_grad_fac_ttc AVLC_HEADWAY_INCREASE_GRAD_FAC_TTC
extern Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC_TTC
    [Acc_headway_increase_grad_fac_ttc_points];

/*! [] factor of how much the headway to a new object can be increased per
 * second*/
// #define Fct_acc_headway_setting_factor VLC_AVLC_HEADWAY_SETTING_FACTOR
// #define Fct_acc_headway_setting_factor_points ((uint16)5)
// extern Acc_const volatile factor_t
//     VLC_AVLC_HEADWAY_SETTING_FACTOR[2 *
//     Fct_acc_headway_setting_factor_points];

/*! [] factor of how much the headway to a new object can be increased per
 * second*/
// #define Fct_acc_headway_increase_grad_factor \
//     VLC_AVLC_HEADWAY_INCREASE_GRAD_FACTOR
// #define Fct_acc_headway_increase_grad_factor_points ((uint16)4)
// extern Acc_const volatile factor_t VLC_AVLC_HEADWAY_INCREASE_GRAD_FACTOR
//     [2 * Fct_acc_headway_increase_grad_factor_points];

/*! [m/s2] max allowed deceleration to objects being at the outer lanes (lane
 * change)*/
#define Acc_max_decel_outer_lanes AVLC_MAX_DECEL_OUTER_LANES
#define Acc_max_decel_outer_lanes_points ((uint16)3)
extern Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_OUTER_LANES[2 * Acc_max_decel_outer_lanes_points];

/*! [m/s^2] max allowed deceleration to an adjacent lane object depending on its
 * cut in potential*/
#define Acc_max_decel_cut_in AVLC_MAX_DECEL_CUT_IN
#define Acc_max_decel_cut_in_points 3
extern Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_CUT_IN[2 * Acc_max_decel_cut_in_points];

/*! [] cut out depended factor for requested distance*/
#define Acc_cut_out_distance_factor AVLC_CUT_OUT_DISTANCE_FACTOR
#define Acc_cut_out_distance_factor_points ((uint16)3)
extern Acc_const volatile factor_t
    AVLC_CUT_OUT_DISTANCE_FACTOR[2 * Acc_cut_out_distance_factor_points];

/*! [] cut out depended factor for smoothness*/
#define Acc_cut_out_smoothness_factor AVLC_CUT_OUT_SMOOTHNESS_FACTOR
#define Acc_cut_out_smoothness_factor_points ((uint16)3)
extern Acc_const volatile factor_t
    AVLC_CUT_OUT_SMOOTHNESS_FACTOR[2 * Acc_cut_out_smoothness_factor_points];

/*! [] host vehicle lane change probability depended factor for requested
 * distance*/
#define Acc_lc_prob_distance_factor AVLC_LC_PROB_DISTANCE_FACTOR
#define Acc_lc_prob_distance_factor_points ((uint16)4)
extern Acc_const volatile factor_t
    AVLC_LC_PROB_DISTANCE_FACTOR[2 * Acc_lc_prob_distance_factor_points];

/*! [] host vehicle lane change probability depended factor for smoothness*/
#define Acc_lc_prob_smoothness_factor AVLC_LC_PROB_SMOOTHNESS_FACTOR
#define Acc_lc_prob_smoothness_factor_points ((uint16)4)
extern Acc_const volatile factor_t
    AVLC_LC_PROB_SMOOTHNESS_FACTOR[2 * Acc_lc_prob_smoothness_factor_points];

/*! [%] minimum cutout probability of the object where the lane change
 * probability to release the object shall be effective*/
#define Acc_lc_min_cut_out_probability AVLC_LC_MIN_CUT_OUT_PROBABILITY
extern Acc_const volatile percentage_t AVLC_LC_MIN_CUT_OUT_PROBABILITY;

/*! [s] ACC prediction time for host velocity prediction*/
#define Acc_predicted_reaction_time AVLC_PREDICTED_REACTION_TIME
extern Acc_const volatile times_t AVLC_PREDICTED_REACTION_TIME;

/*! [m/s2] Minimum acceleration resolution*/
#define Acc_min_resolution AVLC_MIN_ACCEL_RESOLUTION
extern Acc_const volatile acceleration_t AVLC_MIN_ACCEL_RESOLUTION;

/*! [] maximum negative gradient of object acceleration depending on the current
 * object acceleration*/
#define Acc_obj_accel_max_neg_grad AVLC_OBJ_ACCEL_MAX_NEG_GRAD
#define Acc_obj_accel_max_neg_grad_points ((uint16)3)
extern Acc_const volatile acceleration_t
    AVLC_OBJ_ACCEL_MAX_NEG_GRAD[2 * Acc_obj_accel_max_neg_grad_points];

/*! [] factor for maximum negative gradient of object acceleration depending on
 * the current object velocity*/
#define Acc_obj_accel_grad_speed_fac AVLC_OBJ_ACCEL_GRAD_SPEED_FAC
#define Acc_obj_accel_grad_speed_fac_points ((uint16)2)
extern Acc_const volatile factor_t
    AVLC_OBJ_ACCEL_GRAD_SPEED_FAC[2 * Acc_obj_accel_grad_speed_fac_points];

/*! [] factor for maximum negative gradient of object acceleration depending on
 * the distance between current and filtered acceleration*/
#define Acc_obj_accel_grad_accel_d_fac AVLC_OBJ_ACCEL_GRAD_ACCEL_D_FAC
#define Acc_obj_accel_grad_accel_d_fac_pt ((uint16)3)
extern Acc_const volatile factor_t
    AVLC_OBJ_ACCEL_GRAD_ACCEL_D_FAC[2 * Acc_obj_accel_grad_accel_d_fac_pt];

/*! [] factor for using calculated stop deceleration below a specific time to
 * object stop threshold*/
#define Acc_use_calc_decel_factor AVLC_USE_CALC_DECEL_FACTOR
#define Acc_use_calc_decel_factor_points ((uint16)2)
extern Acc_const volatile factor_t
    AVLC_USE_CALC_DECEL_FACTOR[2 * Acc_use_calc_decel_factor_points];

/*! [] factor for using calculated stop deceleration according to moving time of
 * ego vehicle*/
#define Acc_calc_decel_moving_time_factor AVLC_CALC_DECEL_MOV_TIME_FACTOR
#define Acc_calc_decel_moving_time_factor_points ((uint16)2)
extern Acc_const volatile factor_t
    AVLC_CALC_DECEL_MOV_TIME_FACTOR[2 *
                                    Acc_calc_decel_moving_time_factor_points];

#define Acc_calc_dec_ds_dist_fac AVLC_CALC_DEC_DS_DIST_FAC
#define Acc_calc_dec_ds_dist_fac_points ((uint16)4)
extern Acc_const volatile factor_t
    AVLC_CALC_DEC_DS_DIST_FAC[2 * Acc_calc_dec_ds_dist_fac_points];

#define Acc_calc_dec_ds_decl_fac AVLC_CALC_DEC_DS_DECL_FAC
#define Acc_calc_dec_ds_decl_fac_points ((uint16)4)
extern Acc_const volatile factor_t
    AVLC_CALC_DEC_DS_DECL_FAC[2 * Acc_calc_dec_ds_decl_fac_points];

/*! [m] distance to braking object where "emergency braking" should be applied*/
#define Acc_emergency_brake_distance AVLC_EMERGENCY_BRAKE_DISTANCE
extern Acc_const volatile distance_t AVLC_EMERGENCY_BRAKE_DISTANCE;

/*! [m] distance to braking when object is close enough*/
#define Acc_brake_distance_for_stationary AVLC_BRAKE_DISTANCE_FOR_STATIONARY
extern Acc_const volatile distance_t AVLC_BRAKE_DISTANCE_FOR_STATIONARY;

/*! [m] minimum distance that is always be allowed to brake MUST be above 0!*/
#define Acc_min_allowed_brake_distance AVLC_MIN_ALLOWED_BRAKE_DISTANCE
extern Acc_const volatile distance_t AVLC_MIN_ALLOWED_BRAKE_DISTANCE;

/*! [m/s^2] deceleration request when host vehicle is stationary*/
#define Acc_stationary_brake_deceleration AVLC_STATIONARY_BRAKE_DECELERATION
extern Acc_const volatile acceleration_t AVLC_STATIONARY_BRAKE_DECELERATION;

/*! [m/s^2] minimum deceleration request in case of a emergency stop*/
#define Acc_emergency_brake_deceleration AVLC_EMERGENCY_BRAKE_DECELERATION
extern Acc_const volatile acceleration_t AVLC_EMERGENCY_BRAKE_DECELERATION;

/*! [m/s^2] minimum deceleration request for calculated deceleration*/
#define Acc_minimum_calculated_brake_request \
    AVLC_MINIMUM_CALCULATED_BRAKE_REQUEST
extern Acc_const volatile acceleration_t AVLC_MINIMUM_CALCULATED_BRAKE_REQUEST;

#define Acc_alert_supress_alert_time_points 2
#define Acc_alert_supress_alert_time AVLC_ALERT_SUPRESS_ALERT_TIME
extern Acc_const volatile times_t
    AVLC_ALERT_SUPRESS_ALERT_TIME[2 * Acc_alert_supress_alert_time_points];

/*! [1] factor of tts deceleration compensation, multiple with distance error to
 * object*/
#define Acc_tts_decel_dist_err_fac AVLC_TTS_DECEL_DIST_ERR_FAC
extern Acc_const volatile factor_t AVLC_TTS_DECEL_DIST_ERR_FAC;

/*! [1] Limit of tts deceleration compensation */
#define Acc_tts_decel_dist_err_lim AVLC_TTS_DECEL_DIST_ERR_LIM
extern Acc_const volatile acceleration_t AVLC_TTS_DECEL_DIST_ERR_LIM;

/*! [1] factor of tts deceleration, the host speed is higher, the factor is
 * bigger */
#define Acc_tts_decel_fac_points ((uint16)4)
#define Acc_tts_decel_fac AVLC_TTS_DECEL_FAC
extern Acc_const volatile acceleration_t
    AVLC_TTS_DECEL_FAC[2 * Acc_tts_decel_fac_points];

/*! [m/s^2] minimum object deceleration for Time To Stop calculation*/
#define Acc_object_decel_for_tts_calc AVLC_OBJECT_DECEL_FOR_TTS_CALC
extern Acc_const volatile acceleration_t AVLC_OBJECT_DECEL_FOR_TTS_CALC;

/*! [m/s] minimum object speed for Time To Stop calculation*/
#define Acc_object_speed_for_tts_calc AVLC_OBJECT_SPEED_FOR_TTS_CALC
extern Acc_const volatile velocity_t AVLC_OBJECT_SPEED_FOR_TTS_CALC;

/*! [m/s] host vehicle speed can be estimated as stationary*/
#define Acc_host_speed_for_stationary_calc AVLC_HOST_SPEED_FOR_STA_CALC
extern Acc_const volatile velocity_t AVLC_HOST_SPEED_FOR_STA_CALC;

/*! [m/s] host vehicle speed that tts calculated acceleration can be used when
 * below it*/
#define Acc_host_speed_for_tts_accel_use AVLC_HOST_SPEED_FOR_TTS_ACC_USE
extern Acc_const volatile velocity_t AVLC_HOST_SPEED_FOR_TTS_ACC_USE;

/*! [m] maximum object distance for Time To Stop calculation*/
#define Acc_object_max_distance_for_tts_calc \
    AVLC_OBJECT_MAX_DISTANCE_FOR_TTS_CALC
extern Acc_const volatile distance_t AVLC_OBJECT_MAX_DISTANCE_FOR_TTS_CALC;

/*! [m/s] maximum object relative speed for Time To Stop calculation (not brake
 * for faster objects)*/
#define Acc_object_rel_speed_for_tts_calc AVLC_OBJECT_REL_SPEED_FOR_TTS_CALC
extern Acc_const volatile velocity_t AVLC_OBJECT_REL_SPEED_FOR_TTS_CALC;

/*! [] factor for using calculated (max intrusion) deceleration against
 * calculated (max intrusion distance) deceleration (object rel speed
 * depended)*/
#define Acc_use_approach_decel_speed_factor AVLC_USE_APPROACH_DECEL_SPEED_FACTOR
#define Acc_use_approach_decel_speed_factor_points ((uint16)2)
extern Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_SPEED_FACTOR
    [2 * Acc_use_approach_decel_speed_factor_points];

/*! [] factor for using calculated (max intrusion) deceleration against
 * calculated (max intrusion distance) deceleration (object accel depended)*/
#define Acc_use_approach_decel_v_host_factor \
    AVLC_USE_APPROACH_DECEL_V_HOST_FACTOR
#define Acc_use_approach_decel_v_host_factor_points ((uint16)2)
extern Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_V_HOST_FACTOR
    [2 * Acc_use_approach_decel_v_host_factor_points];

/*! [] factor for using calculated (max intrusion) deceleration against
 * calculated (max intrusion distance) deceleration (max intrusion decel
 * depended)*/
#define Acc_use_approach_decel_decel_factor AVLC_USE_APPROACH_DECEL_DECEL_FACTOR
#define Acc_use_approach_decel_decel_factor_points ((uint16)2)
extern Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_DECEL_FACTOR
    [2 * Acc_use_approach_decel_decel_factor_points];

/*! [m/s2] Requested acceleration while stand still */
#define Acc_standstill_accel_req AVLC_STANDSTILL_ACCEL_REQ
extern Acc_const volatile acceleration_t AVLC_STANDSTILL_ACCEL_REQ;

/*! max host vehicle velocity to decelerate on stationary objects */
#define Acc_decel_on_stationary_speed AVLC_DECEL_ON_STATIONARY_SPEED
extern Acc_const volatile velocity_t AVLC_DECEL_ON_STATIONARY_SPEED;

/*! min host vehicle velocity to active alert */
#define Acc_alert_active_speed_low_limit AVLC_ALERT_ACTIVE_SPEED_LOW_LIMIT
extern Acc_const volatile velocity_t AVLC_ALERT_ACTIVE_SPEED_LOW_LIMIT;

/*! defines for traffic situation estimation */
#define Acc_min_obj_crawl_accel AVLC_MIN_OBJ_CRAWL_ACCEL
extern Acc_const volatile acceleration_t AVLC_MIN_OBJ_CRAWL_ACCEL;

/*! Min acceleration to detect stop situation */
#define Acc_min_obj_stop_accel AVLC_MIN_OBJ_STOP_ACCEL
extern Acc_const volatile acceleration_t AVLC_MIN_OBJ_STOP_ACCEL;

/*! [m] Maximum stopping distance for Stop&Go situation */
#define Acc_max_stop_distance (*pAcc_max_stop_distance)
extern Acc_const volatile distance_t* pAcc_max_stop_distance;

/*! [m] Minimum ACC distance for control usage */
#define Acc_min_cust_perf_dist (*pAcc_min_cust_perf_dist)
extern Acc_const volatile distance_t* pAcc_min_cust_perf_dist;

#define Acc_si_max_hys_time_to_stop AVLC_SI_MAX_HYS_TIME_TO_STOP
extern Acc_const volatile times_t AVLC_SI_MAX_HYS_TIME_TO_STOP;

#define Acc_si_min_hys_time_to_stop AVLC_SI_MIN_HYS_TIME_TO_STOP
extern Acc_const volatile times_t AVLC_SI_MIN_HYS_TIME_TO_STOP;

/*! time gap dependent intrusion factor */
#define time_gap_dependency_factor TIME_GAP_DEPENDENCY_FACTOR
extern Acc_const volatile factor_t TIME_GAP_DEPENDENCY_FACTOR;

/*! max control distance for standing relevant object */
#define Acc_max_dist_rel_standing_obj AVLC_MAX_DIST_REL_STANDING_OBJ
extern Acc_const volatile distance_t AVLC_MAX_DIST_REL_STANDING_OBJ;

// add for dependence complier
#ifndef FN_AP_EBA_COUNTRY_0
#define FN_AP_EBA_COUNTRY_0 0U
#endif
#ifndef FN_AP_AVLC_COUNTRY_0
#define FN_AP_AVLC_COUNTRY_0 0U
#endif
#ifndef FN_AP_EBA_COUNTRY_1
#define FN_AP_EBA_COUNTRY_1 1U
#endif
#ifndef FN_AP_EBA_COUNTRY_2
#define FN_AP_EBA_COUNTRY_2 2U
#endif
#ifndef FN_AP_EBA_COUNTRY_3
#define FN_AP_EBA_COUNTRY_3 3U
#endif
#ifndef FN_AP_PCS_COUNTRY_MASK
#define FN_AP_PCS_COUNTRY_MASK 3U
#endif
#ifndef FN_AP_EBA_COUNTRY_MASK
#define FN_AP_EBA_COUNTRY_MASK 15U
#endif
#ifndef FN_AP_AVLC_COUNTRY_1
#define FN_AP_AVLC_COUNTRY_1 16U
#endif
#ifndef FN_AP_AVLC_COUNTRY_2
#define FN_AP_AVLC_COUNTRY_2 32U
#endif
#ifndef FN_AP_AVLC_COUNTRY_3
#define FN_AP_AVLC_COUNTRY_3 48U
#endif
#ifndef FN_AP_AVLC_COUNTRY_4
#define FN_AP_AVLC_COUNTRY_4 64U
#endif
#ifndef FN_AP_AVLC_COUNTRY_MASK
#define FN_AP_AVLC_COUNTRY_MASK 240U
#endif
#ifndef VLC_BSW_ALGO_PARAM_PTR
#define VLC_BSW_ALGO_PARAM_PTR VLC_pBswAlgoParameters
#endif

#endif
