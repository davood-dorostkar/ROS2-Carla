/*! \file **********************************************************************

  COMPONENT:              ACC (Adaptive Cruise Control)

  MODULENAME:             acc_par.c

  @brief                  This module contains all functional parameters


****************************************************************************/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "acc_par.h"
#include "vlc_long_cfg.h"
// #include "si.h"

/*max set speed for ACC in kph/mph*/
Acc_const volatile setspeed_t AVLC_MAX_VSET_KMH = 120;
Acc_const volatile setspeed_t AVLC_MAX_VSET_MPH = 95;

/*max acceleration/deceleration request that is allowed (speed depended)*/

Acc_const volatile acceleration_t
    AVLC_FSR_MAX_ACCEL[2 * Acc_fsr_max_acceleration_points] = {
        300,  2000,  //
        500,  2000,  //
        2000, 2000,  //
        2500, 2000   //
};

Acc_const volatile acceleration_t
    AVLC_FSR_MAX_DECEL[2 * Acc_fsr_max_deceleration_points] = {
        500, -4900,   //
        2000, -3500,  //
};

/*Custom Specific for SW18*/
Acc_const volatile uint8 AVLC_MID_HEADWAYSETTING = 50;

Acc_const volatile distance_t AVLC_REQUEST_STOP_DISTANCE_FACTOR
    [2 * Acc_request_stop_distance_factor_points] = {
        30, 100,   //
        400, 100,  //
};

/*headway setting for minimum headway setting (0%) and maximum headway setting
 * (100%)*/
/*speed(m/s), distance (m)*/

Acc_const volatile distance_t
    AVLC_HEADWAY_MIN_DIST[2 * Acc_headway_min_dist_points] = {
        0,    200,   //
        278,  650,   //
        1111, 1400,  //
        2222, 2222,  //
        9000, 9000   //
};

/*speed(m/s), distance (m)*/
Acc_const volatile distance_t
    AVLC_HEADWAY_MAX_DIST[2 * Acc_headway_max_dist_points] = {
        0,    200,   //
        278,  800,   //
        1111, 2222,  //
        2222, 4444,  //
        9000, 18000  //
};

/*headway setting for minimum headway setting (0%) and maximum headway setting
 * (100%) for US*/
/*speed(m/s), distance (m)*/
Acc_const volatile distance_t
    AVLC_HEADWAY_MIN_DIST_US[2 * Acc_headway_min_dist_points] = {
        /* USA */
        0,    400,   //
        300,  750,   //
        1000, 1480,  //
        2000, 2200,  //
        9000, 8400   //
};

/*speed(m/s), distance (m)*/
Acc_const volatile distance_t
    AVLC_HEADWAY_MAX_DIST_US[2 * Acc_headway_max_dist_points] = {
        /* USA */
        0,    400,   //
        300,  1000,  //
        1000, 3000,  //
        2000, 6000,  //
        9000, 27000  //
};

/* Set default values*/
Acc_const volatile distance_t* AVLC_pHeadwayMinDist = AVLC_HEADWAY_MIN_DIST;
Acc_const volatile distance_t* AVLC_pHeadwayMaxDist = AVLC_HEADWAY_MAX_DIST;

/*typical thresholds for FSRA (dis)engagement*/
Acc_const volatile setspeed_t AVLC_FSR_OFF_THRES_KMH = 2;
Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_KMH =
    0; /*Engage ACC in standstill == 0*/
Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_KMH = 30; /*SW18*/  // 10;
Acc_const volatile setspeed_t AVLC_FSR_OFF_THRES_MPH = 2;
Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_MPH = 0;
Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_MPH = 6;
Acc_const volatile setspeed_t AVLC_FSR_MIN_V_ENGAGE_USA_MPH = 0;
Acc_const volatile setspeed_t AVLC_FSR_MIN_VSET_USA_MPH = 6;

/* Brake release gradient limit */
/*velocity m/s, jerk m/s^3*/
Acc_const volatile acceleration_t
    AVLC_MAX_POS_GRAD_NEG_ACCEL[2 * Acc_pos_grad_neg_accel_points] = {
        100, 2500,  // 2000
        1500, 2500  // 2000
};

/* Acceleration gradient limits */
/* velocity m/s, jerk m/s^3 */
Acc_const volatile acceleration_t
    AVLC_MAX_POS_GRAD_POS_ACCEL[2 * Acc_pos_grad_pos_accel_points] = {
        100, 2500,  // 2000
        1500, 1500  // 2000

        /* Small size engine */
        // 28,
        // 1000, /* 10 kph */
        // 56,
        // 1000, /* 20 kph */
        // 2777,
        // 1000, /* 100 kph */
        // 5000,
        // 1000 /* 100 kph */

        /* Middle size engine */

        // 28,  882,  56, 686, 2777,
        // 490, 5000, 392

        /* Large size engine */
        /*
            28, 980,
            56, 784,
          2777, 588,
          5000, 490
        */
};

/*velocity m/s, jerk m/s^3*/
Acc_const volatile acceleration_t AVLC_MAX_NEG_GRAD[2 * Acc_neg_grad_points] = {
    500, -4000,   //
    2000, -2000,  //
};

/* recommended velocity as a function of visibility range */
Acc_const volatile velocity_t
    AVLC_RECOMMENDED_VELOCITY_CURVE[2 * Acc_recommended_velocity_curve_points] =
        {
            3000,  1500,  //
            000,   2000,  //
            5000,  2500,  //
            10000, 5000,  //
            15000, 7000,  //
};

/*! [%] factor (0-1) from requested distance to alert distance*/

Acc_const volatile factor_t
    AVLC_MAX_INTRUSION_FACTOR[2 * Acc_max_intrusion_factor_points] = {

        0,   150,  500, 220, 1500,
        460, 2500, 580

};

/*! [%] factor (0-1) dependent on relative veloctiy (suppress intrusion during
 * approach) */

Acc_const volatile factor_t AVLC_VREL_INTRUSION[2 * Acc_vrel_intrusion_points] =
    {
        -972, 1000,  //
        -694, 950,   //
        -556, 800,   //
        -417, 300,   //
        -278, 100,   //
        0,    0      //
};

/*! New alert threshold for independent intrusion calculation */
/*estimated speed, alert distance for intrusion*/
Acc_const volatile distance_t
    AVLC_MAX_ALERT_THRES_INTRUSION[2 * Acc_max_alert_thres_intrusion_points] = {
        10,   200,  //
        100,  250,  //
        300,  400,  //
        1000, 800,  //
        8000, 4000  //
};

/*m/s   sim_time */
Acc_const volatile times_t
    AVLC_ALERT_MAX_SIM_TIME[2 * Acc_max_sim_time_points] = {
        1000, 3000,  //
        3000, 4000   //
};

Acc_const volatile times_t AVLC_ALERT_SIM_TIME_STEP = 500;

/* estimated speed [m/s], supress time [s] */
Acc_const volatile times_t
    AVLC_ALERT_SUPRESS_ALERT_TIME[2 * Acc_alert_supress_alert_time_points] = {
        200, 200,  //
        1500, 500  //
};

Acc_const volatile times_t AVLC_ALERT_MIN_OUTPUT_TIME = 600;

// Using fuzzy Control when drive off to approaching target when it brake
Acc_const volatile times_t AVLC_DRIVE_OFF_TIME = 5000;

// Using fuzzy Control when drive off to approaching target when it brake
Acc_const volatile distance_t AVLC_DRIVE_OFF_DISTANCE = 250;

/*! [m] max distance allowed for using TTS acceleration */
Acc_const volatile distance_t
    AVLC_TTS_ACC_MAX_DIST[2 * Acc_tts_acceleration_max_dist_points] = {
        200,  2000,   //
        500,  5000,   //
        1000, 8000,   //
        2000, 12000,  //
};

// Acc_const volatile float32
//     AVLC_MIN_ALERT_THRES[Acc_min_alert_thres_host_speed_points *
//                          Acc_min_alert_thres_rel_speed_points] = {
//         0,    0,    0,    0,    0,    //
//         3000, 2000, 1000, 350,  130,  //
//         4000, 2600, 1200, 600,  200,  //
//         5800, 3200, 1800, 1000, 400,  //
//         9000, 4600, 2600, 1600, 600,  //
// };

// Acc_const volatile float32
//     AVLC_MIN_ALERT_HOST_SPEED[Acc_min_alert_thres_host_speed_points] = {
//         100, 300, 1000, 2000, 9000,
// };

// Acc_const volatile float32
//     AVLC_MIN_ALERT_REL_SPEED[Acc_min_alert_thres_rel_speed_points] = {
//         -2000, -1000, -500, -200, 0,
// };

// /*estimated speed, relative speed,  alert distance*/
// Acc_const volatile float32
//     AVLC_MAX_ALERT_THRES[Acc_max_alert_thres_host_speed_points *
//                          Acc_max_alert_thres_rel_speed_points] = {
//         20,  20,  20,  20,  20,  //
//         3200, 2200, 1150, 450,  180,  //
//         4200, 2800, 1350, 700,  250,  //
//         6100, 3600, 2000, 1200, 500,  //
//         9000, 5400, 3200, 1800, 750,  //
// };

// Acc_const volatile float32
//     AVLC_MAX_ALERT_HOST_SPEED[Acc_max_alert_thres_host_speed_points] = {
//         100, 300, 1000, 2000, 9000,
// };

// Acc_const volatile float32
//     AVLC_MAX_ALERT_REL_SPEED[Acc_max_alert_thres_rel_speed_points] = {
//         -2000, -1000, -500, -200, 0,
// };

/*! [m] min alert time gap (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_TIME_GAP[2 * Acc_min_alert_thres_time_gap_points] = {
        300,  1000,  //
        1000, 350,   //
        3000, 180,   //
};

/*! [m] min alert ttc (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_TTC[2 * Acc_min_alert_thres_ttc_points] = {
        100,  800,   //
        300,  1500,  //
        1000, 2000,  //
};

/*! [m] min alert accel (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MIN_ALERT_THRES_ACCEL[2 * Acc_min_alert_thres_accel_points] = {
        500, -4500,   //
        2000, -3600,  //
};

/*! [m] max alert time gap (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_TIME_GAP[2 * Acc_max_alert_thres_time_gap_points] = {
        300,  1200,  //
        1000, 500,   //
        3000, 250,   //
};

/*! [m] max alert ttc (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_TTC[2 * Acc_max_alert_thres_ttc_points] = {
        100,  900,   //
        300,  1600,  //
        1000, 2200,  //
};

/*! [m] max alert accel (estimated host speed dependend)*/
Acc_const volatile times_t
    AVLC_MAX_ALERT_THRES_ACCEL[2 * Acc_max_alert_thres_accel_points] = {
        500, -4300,   //
        2000, -3400,  //
};

/* relative speed, factor between object distance and requested distance */
Acc_const volatile factor_t
    AVLC_HEADWAY_ADD_FACTOR[2 * Acc_headway_add_factor_points] = {
        -600, 300, -300, 150, 0, 0};

/*! [%] above this lane change probability the situation can be classified as
 * lane change/overtake situation*/
Acc_const volatile percentage_t AVLC_SI_LCPROB_OVRTK = (percentage_t)60;

/*! [m/s�] max allowed deceleration to objects being in front of the next object
 * in lane*/
/*cutoutprob relevant (0) object, deceleration limit to hidden object*/
Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_HIDDEN_OBJECT[2 * Acc_max_decel_hidden_object_points] = {
        15,   2000,  //
        30,  -500,  //
        100, -3000  //
};

/*! [m/s2] max allowed deceleration to objects in adjacent lane when ego vehicle
 * changing lane */
/* ego vehicle speed, deceleration limit to adjacent object*/
Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_ADJACENT_OBJECT[2 * Acc_max_decel_adjacent_object_points] = {
        500, -3500,   //
        2000, -2500,  //
};

/*! [m/s�] max allowed deceleration to objects being at the outer lanes
 * (depending on own lane change potential)*/
Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_OUTER_LANES[2 * Acc_max_decel_outer_lanes_points] = {
        75, 4000, /* Remark: signal can be trusted when values exceeds 75%*/
        90, -500, 100, -2000};

/*! [m/s^2] max allowed deceleration to an adjacent lane object depending on its
 * cut in potential*/
/* cut in potential % / max deceleration*/
Acc_const volatile acceleration_t
    AVLC_MAX_DECEL_CUT_IN[2 * Acc_max_decel_cut_in_points] = {
        60,  4000,  //
        80,  0,     //
        100, -1000  //
};

/*! [] cut out depended factor for requested distance*/
/* cut out potential % / factor *1000*/
Acc_const volatile factor_t
    AVLC_CUT_OUT_DISTANCE_FACTOR[2 * Acc_cut_out_distance_factor_points] = {
        40,  1000,  //
        60,  800,   //
        100, 200    //
};

/*! [] cut out depended factor for smoothness*/
/* cut out potential % / factor *1000*/
Acc_const volatile factor_t
    AVLC_CUT_OUT_SMOOTHNESS_FACTOR[2 * Acc_cut_out_smoothness_factor_points] = {
        40,  1000,  //
        60,  800,   //
        100, 100    //
};

/*! [] host vehicle lane change probability depended factor for requested
 * distance*/
Acc_const volatile factor_t
    AVLC_LC_PROB_DISTANCE_FACTOR[2 * Acc_lc_prob_distance_factor_points] = {
        50,  1000,
        60,  700, /* Remark: signal can be trusted when values exceeds 75%*/
        80,  550,
        100, 400};

/*! [] host vehicle lane change probability dependent factor for smoothness*/
Acc_const volatile factor_t
    AVLC_LC_PROB_SMOOTHNESS_FACTOR[2 * Acc_lc_prob_smoothness_factor_points] = {
        60,  1000,  //
        70,  800,   //
        80,  500,   //
        100, 0      //
};

/*! relative speed[m/s] requested distance increase gradient [m/s]*/
// Acc_const volatile factor_t VLC_AVLC_HEADWAY_INCREASE_GRAD_FACTOR
//     [2 * Fct_acc_headway_increase_grad_factor_points] = {
//         -1000, 400,  //
//         -500,  200,  //
//         0,     100,  //
//         200,   50    //
// };

/*! requested distance increase gradient from headway setting [m/s]*/
// Acc_const volatile factor_t
//     VLC_AVLC_HEADWAY_SETTING_FACTOR[2 *
//     Fct_acc_headway_setting_factor_points] =
//         {
//             0,   100,  //
//             25,  80,   //
//             50,  60,   //
//             75,  40,   //
//             100, 20    //
// };

// Acc_const volatile float32
//     AVLC_HEADWAY_INCREASE_GRAD[Acc_headway_increase_grad_host_speed_points *
//                                Acc_headway_increase_grad_distance_points] = {
//         100,  80,  60,  50,  30,   //
//         150,  120, 100, 80,  60,   //
//         300,  200, 150, 120, 100,  //
//         700,  500, 300, 200, 150,  //
//         1200, 900, 600, 400, 300,  //
// };

// Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_HOST_SPEED
//     [Acc_headway_increase_grad_host_speed_points] = {
//         100, 300, 1000, 2000, 3000,
// };

// Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_DISTANCE
//     [Acc_headway_increase_grad_distance_points] = {
//         300, 1000, 2000, 3000, 5000,
// };

Acc_const volatile times_t AVLC_HEADWAY_TIME_THRESHOLD = 10000;

Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC
    [Acc_headway_increase_grad_fac_timegap_points *
     Acc_headway_increase_grad_fac_ttc_points] = {
        400, 250, 120, 60,  //
        300, 200, 80,  40,  //
        200, 140, 60,  30,  //
        100, 90,  45,  25,  //
};

Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC_TIMEGAP
    [Acc_headway_increase_grad_fac_timegap_points] = {
        500, 1000,   //
        2000, 10000  //
};

Acc_const volatile float32 AVLC_HEADWAY_INCREASE_GRAD_FAC_TTC
    [Acc_headway_increase_grad_fac_ttc_points] = {
        1000, 2000,  //
        5000, 10000  //
};

Acc_const volatile percentage_t AVLC_LC_MIN_CUT_OUT_PROBABILITY = 0;

Acc_const volatile acceleration_t
    AVLC_OBJ_ACCEL_MAX_NEG_GRAD[2 * Acc_obj_accel_max_neg_grad_points] = {
        -1500, -5000,  //
        -500,  -5000,  //
        500,   -5000   //
};

Acc_const volatile factor_t
    AVLC_OBJ_ACCEL_GRAD_SPEED_FAC[2 * Acc_obj_accel_grad_speed_fac_points] = {
        700, 1000,  //
        1500, 1000  //
};

Acc_const volatile factor_t
    AVLC_OBJ_ACCEL_GRAD_ACCEL_D_FAC[2 * Acc_obj_accel_grad_accel_d_fac_pt] = {
        -1000, 1000,  //
        -2000, 1000,  //
        -3000, 1000   //
};

Acc_const volatile factor_t
    AVLC_USE_CALC_DECEL_FACTOR[2 * Acc_use_calc_decel_factor_points] = {
        500, 0,  //
        1500, 0  //
};

Acc_const volatile factor_t AVLC_CALC_DECEL_MOV_TIME_FACTOR
    [2 * Acc_calc_decel_moving_time_factor_points] = {
        1000, 0,    //
        3000, 1000  //
};

Acc_const volatile factor_t
    AVLC_CALC_DEC_DS_DIST_FAC[2 * Acc_calc_dec_ds_dist_fac_points] = {
        1000,  0,     //
        2500,  500,   //
        5000,  1000,  //
        10000, 5000   //
};

Acc_const volatile factor_t
    AVLC_CALC_DEC_DS_DECL_FAC[2 * Acc_calc_dec_ds_decl_fac_points] = {
        -2500, 0,    //
        -2000, 100,  //
        -1000, 400,  //
        -500,  1000  //
};

Acc_const volatile distance_t AVLC_EMERGENCY_BRAKE_DISTANCE = (distance_t)150;

Acc_const volatile distance_t AVLC_BRAKE_DISTANCE_FOR_STATIONARY =
    (distance_t)250;

Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_SPEED_FACTOR
    [2 * Acc_use_approach_decel_speed_factor_points] = {
        -1000, 1000,  //
        -300, 0       //
};

Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_V_HOST_FACTOR
    [2 * Acc_use_approach_decel_v_host_factor_points] = {
        1500, 0,    //
        2500, 1000  //
};

Acc_const volatile factor_t AVLC_USE_APPROACH_DECEL_DECEL_FACTOR
    [2 * Acc_use_approach_decel_decel_factor_points] = {
        -2000, 1000,  //
        -1000, 700    //
};

Acc_const volatile acceleration_t AVLC_AMIN_OBJECT_LOST = 0;

Acc_const volatile acceleration_t AVLC_AMAX_OBJECT_LOST = 300;

Acc_const volatile acceleration_t AVLC_MIN_ACCEL_OBJECT_STATIONARY = 0;

Acc_const volatile factor_t AVLC_POS_GRAD_ECO_FAC = 80;

Acc_const volatile factor_t AVLC_POS_GRAD_SPORT_FAC = 120;

Acc_const volatile velocity_t
    AVLC_VREL_ESTIM_MIN[2 * Acc_vrel_estim_min_points] = {
        150,  -100,  //
        1000, -200,  //
        2000, -250,  //
        9000, -800   //
};

Acc_const volatile velocity_t
    AVLC_VREL_ESTIM_MAX[2 * Acc_vrel_estim_max_points] = {
        150,  -150,  //
        1000, -300,  //
        2000, -350,  //
        9000, -500   //
};

Acc_const volatile factor_t AVLC_END_ALERT_HYST_FACTOR =
    150; /*15% more than alert distance*/

Acc_const volatile times_t AVLC_ALERT_THRES_HYST = 100;

Acc_const volatile times_t AVLC_ALERT_SIM_VEHICLE_FILTER_TIME = 800;

/*speed, acceldif*/
Acc_const volatile acceleration_t
    AVLC_DECREASE_MINACCEL_CURVE[2 * Acc_decrease_minaccel_points] = {
        500,  0,     //
        1000, -100,  //
        1500, 0      //
};

/*speed, acceldif*/
Acc_const volatile acceleration_t
    AVLC_INCREASE_MAXACCEL_CURVE[2 * Acc_increase_maxaccel_points] = {
        500,  0,    //
        1000, 100,  //
        1500, 0     //
};

/*! [m/s] Minimum speed to judge stopping state of an object more stable */
Acc_const volatile velocity_t AVLC_STOPPED_SPEED = (velocity_t)30; /* 0.3m/s*/

/*! [m/s] speed where object standstill is assumed */
Acc_const volatile velocity_t AVLC_OBJ_STOPPED_SPEED =
    (velocity_t)100; /* 1m/s*/

Acc_const volatile acceleration_t AVLC_STOPPED_OBJ_DECEL = (acceleration_t)-500;

/*! [m/s] maximum speed for a "crawling" vehicle*/
Acc_const volatile velocity_t AVLC_CRAWL_MAX_VELOCITY =
    (velocity_t)600; /*6m/s*/

/*! [s] maximum time the target vehicle may need to come to a full stop to call
 * this situation a stop situation*/
Acc_const volatile times_t AVLC_SI_MAX_TIME_TO_STOP = (times_t)1000; /*1s*/

/*! [s] maximum time the target vehicle may need to come to a full stop to call
 * this situation a stop situation if object in front of the target has already
 * stopped*/
Acc_const volatile times_t AVLC_SI_MAX_TIME_TO_STOP_2OBJ = (times_t)3000; /*3s*/

/*! [m/s^2] object acceleration for a "go" signal in low speed situations*/
Acc_const volatile acceleration_t AVLC_SI_MIN_GO_ACCEL =
    (acceleration_t)500; /*0.5m/(s^2)*/

/*! [m/s^2] object acceleration for a "go" signal in low speed situations if
 * object in front of target is faster and accelerating too*/
Acc_const volatile acceleration_t AVLC_SI_MIN_GO_ACCEL_2OBJ =
    (acceleration_t)800; /*0.8m/(s^2)*/

Acc_const volatile times_t AVLC_SI_MAX_RELEVANT_TTC = (times_t)10000;

Acc_const volatile times_t AVLC_SI_PREDICT_LATERAL_POS_TIME = (times_t)1000;

Acc_const volatile distance_t AVLC_SI_HOST_VEHICLE_WIDTH = (distance_t)180;

/* Criticality factor based on object deceleration */

Acc_const volatile factor_t
    AVLC_SI_CRIT_FACTOR_OBJ_ACCEL[2 * Acc_si_crit_factor_obj_accel_points] = {
        -3500, 1000,  //
        -2500, 600,   //
        -1500, 150,   //
        -500,  0      //
};

/* Factor to increase criticality for shortest distance mode [factor] */
Acc_const volatile factor_t AVLC_CRIT_FACTOR_GAIN = 1500;

/*! [%] criticality to object based on ttc value*/
Acc_const volatile factor_t
    AVLC_SI_CRITICALITY_FROM_TTC[2 * Acc_si_crit_from_ttc_points] = {
        1000, 1000, 2000, 600, 3000, 350, 5000, 0}; /*ttc (s), criticality*/

Acc_const volatile gradient_t AVLC_SI_CRIT_POS_GRAD = 250; /* 98%/s*/

Acc_const volatile gradient_t AVLC_SI_CRIT_NEG_GRAD = -200; /* 78%/s*/

/*! [m/s2] maximum object acceleration that is possible*/
Acc_const volatile acceleration_t AVLC_MAX_OBJECT_POSSIBLE_ACCEL = 4000;

/*! [m/s2] minimum object acceleration that is possible*/
Acc_const volatile acceleration_t AVLC_MIN_OBJECT_POSSIBLE_ACCEL = -8000;

/*! [m/s2] object acceleration hysteresis*/
Acc_const volatile acceleration_t AVLC_OBJECT_HYST_ACCEL = 200;

/*! [m/s^2] maximum allowed acceleration that can happen*/
Acc_const volatile acceleration_t AVLC_MAX_ALLOWED_ACCEL = (acceleration_t)4000;

/*! [m/s^2] maximum allowed deceleration that can happen*/
Acc_const volatile acceleration_t AVLC_MAX_ALLOWED_DECEL =
    (acceleration_t)-6000;

/*! [s] System reaction time when braking requested*/
Acc_const volatile times_t AVLC_T_REACT_BRAKE = 500;

/*! [s] longitudinal visibility threshold related to vehicle velocity where ACC
 * is disengaged (according to ISO 2[s]) */
Acc_const volatile times_t AVLC_T_VIS_DISENGAGE = (times_t)1900;

/*! [s] longitudinal visibility threshold related to vehicle velocity where ACC
 * engagement is allowed (according to ISO 2[s]) */
Acc_const volatile times_t AVLC_T_VIS_ENGAGE = (times_t)2100;

/*! [s] ACC prediction time for host velocity prediction*/
Acc_const volatile times_t AVLC_PREDICTED_REACTION_TIME = 0;

/*! [m/s�] Minimum acceleration resolution*/
Acc_const volatile acceleration_t AVLC_MIN_ACCEL_RESOLUTION = 100;

Acc_const volatile distance_t AVLC_MIN_ALLOWED_BRAKE_DISTANCE = (distance_t)10;

Acc_const volatile acceleration_t AVLC_EMERGENCY_BRAKE_DECELERATION =
    (acceleration_t)-2000;

Acc_const volatile acceleration_t AVLC_MINIMUM_CALCULATED_BRAKE_REQUEST =
    (acceleration_t)-100;

Acc_const volatile acceleration_t AVLC_STATIONARY_BRAKE_DECELERATION =
    (acceleration_t)-500;

Acc_const volatile acceleration_t AVLC_OBJECT_DECEL_FOR_TTS_CALC =
    (acceleration_t)-100;

Acc_const volatile velocity_t AVLC_OBJECT_SPEED_FOR_TTS_CALC = (velocity_t)50;

Acc_const volatile velocity_t AVLC_HOST_SPEED_FOR_STA_CALC = (velocity_t)20;

Acc_const volatile velocity_t AVLC_HOST_SPEED_FOR_TTS_ACC_USE = (velocity_t)150;

Acc_const volatile distance_t AVLC_OBJECT_MAX_DISTANCE_FOR_TTS_CALC =
    (distance_t)12000;

Acc_const volatile velocity_t AVLC_OBJECT_REL_SPEED_FOR_TTS_CALC =
    (velocity_t)20;

/*speed (m/s), factor (1)*/
Acc_const volatile acceleration_t
    AVLC_TTS_DECEL_FAC[2 * Acc_tts_decel_fac_points] = {
        /* Rest of World */
        0,    100,  //
        200,  100,  //
        1000, 100,  //
        2000, 100   //
};

Acc_const volatile factor_t AVLC_TTS_DECEL_DIST_ERR_FAC = 0;

Acc_const volatile acceleration_t AVLC_TTS_DECEL_DIST_ERR_LIM = 10000;

Acc_const volatile acceleration_t AVLC_STANDSTILL_ACCEL_REQ =
    (acceleration_t)-500;

Acc_const volatile velocity_t AVLC_DECEL_ON_STATIONARY_SPEED = (velocity_t)2500;

Acc_const volatile velocity_t AVLC_ALERT_ACTIVE_SPEED_LOW_LIMIT =
    (velocity_t)100;

/*! Parameters for traffic situation estimation */
Acc_const volatile acceleration_t AVLC_MIN_OBJ_CRAWL_ACCEL =
    (acceleration_t)50; /*50 -> 0.05m/(s^2)*/

Acc_const volatile acceleration_t AVLC_MIN_OBJ_STOP_ACCEL = (acceleration_t)0;

/*! [m] Maximum stopping distance of standing object (below this threshold, no
 * drive off possible) */
Acc_const volatile distance_t AVLC_MAX_STOP_DISTANCE = (distance_t)880; /*8,8m*/
Acc_const volatile distance_t AVLC_MAX_STOP_DISTANCE_US =
    (distance_t)1100; /*11m*/

/* Set default value */
Acc_const volatile distance_t* pAcc_max_stop_distance = &AVLC_MAX_STOP_DISTANCE;

/* Minimum ACC distance for control usage */
/* USA */
Acc_const volatile distance_t AVLC_MIN_CUST_PERF_DIST_USA =
    (distance_t)10000; /* 100m */
/* Rest of the world */
Acc_const volatile distance_t AVLC_MIN_CUST_PERF_DIST_NON_USA =
    (distance_t)25000; /* 250m */
/* Default value (no restriction) */
Acc_const volatile distance_t AVLC_MIN_CUST_PERF_DIST = (distance_t)0; /* 0m */
/* Set default value */
Acc_const volatile distance_t* pAcc_min_cust_perf_dist =
    &AVLC_MIN_CUST_PERF_DIST;

Acc_const volatile times_t AVLC_SI_MAX_HYS_TIME_TO_STOP =
    (times_t)1000; /*1sec */
Acc_const volatile times_t AVLC_SI_MIN_HYS_TIME_TO_STOP =
    (times_t)500; /*0.5sec */

/*! time gap dependent intrusion factor */
Acc_const volatile factor_t TIME_GAP_DEPENDENCY_FACTOR = (factor_t)200;

/*! max control distance for standing relevant object */
Acc_const volatile distance_t AVLC_MAX_DIST_REL_STANDING_OBJ =
    (distance_t)20000; /* 200 meters max */

void AVLC_SELECT_PARAM_SET(uint8 Fct_General_FnSwitchBits) {
    // if (((VLC_BSW_ALGO_PARAM_PTR->Fct.General.FnSwitchBits) &
    // (FN_AP_AVLC_COUNTRY_MASK)) == FN_AP_AVLC_COUNTRY_1)
    if (((Fct_General_FnSwitchBits) & (FN_AP_AVLC_COUNTRY_MASK)) ==
        FN_AP_AVLC_COUNTRY_1) {
        /* USA */
        AVLC_pHeadwayMinDist = AVLC_HEADWAY_MIN_DIST_US;
        AVLC_pHeadwayMaxDist = AVLC_HEADWAY_MAX_DIST_US;
        pAcc_max_stop_distance = &AVLC_MAX_STOP_DISTANCE_US;
        pAcc_min_cust_perf_dist = &AVLC_MIN_CUST_PERF_DIST_USA;
    } else {
        /* Rest of the world */
        AVLC_pHeadwayMinDist = AVLC_HEADWAY_MIN_DIST;
        AVLC_pHeadwayMaxDist = AVLC_HEADWAY_MAX_DIST;
        pAcc_max_stop_distance = &AVLC_MAX_STOP_DISTANCE;
        pAcc_min_cust_perf_dist = &AVLC_MIN_CUST_PERF_DIST_NON_USA;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CAL_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
