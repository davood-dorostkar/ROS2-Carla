/*! \file **********************************************************************

  COMPONENT:              CC (Cruise Control)

  MODULENAME:             cc_par.c

  @brief                  This module contains all functional parameters


*****************************************************************************/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"

#include "vlcVeh_ext.h"
#include "cc_ext.h"
#include "cc_par.h"

/*for limiter function*/
Cc_const volatile setspeed_t LIM_MIN_VSET_KMH = 30;
Cc_const volatile setspeed_t LIM_MAX_VSET_KMH = 150;
Cc_const volatile setspeed_t LIM_MIN_VSET_MPH = 20;
Cc_const volatile setspeed_t LIM_MAX_VSET_MPH = 95;

/*cruise control*/
Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_1_MPH = 1;
Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_1_KMH = 1; /*SW18*/
Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_2_MPH = 5;
Cc_const volatile setspeed_t VLC_VSET_STEP_LEVEL_2_KMH = 5; /*SW18*/
Cc_const volatile setspeed_t VLC_VMAX_KMH = 140;            /*SW18*/
Cc_const volatile setspeed_t VLC_VMAX_MPH = 155;
Cc_const volatile setspeed_t VLC_MIN_VSET_MPH = 6;
Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_MPH = 0;
Cc_const volatile setspeed_t VLC_MIN_VSET_USA_MPH = 6;
Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_USA_MPH = 0;
Cc_const volatile setspeed_t VLC_MAX_VSET_MPH = 94;
Cc_const volatile setspeed_t VLC_OFF_THRES_MPH = 2;
Cc_const volatile setspeed_t VLC_OFF_THRES_CRUISE_MPH = 50;
Cc_const volatile setspeed_t VLC_OFF_THRES_KMH = 5;
Cc_const volatile setspeed_t VLC_MIN_V_ENGAGE_KMH = 0;
Cc_const volatile setspeed_t VLC_OFF_THRES_CRUISE_KMH = 50;
Cc_const volatile setspeed_t VLC_MAX_VSET_KMH = 120;
Cc_const volatile setspeed_t VLC_MIN_VSET_KMH = 30;
Cc_const volatile setspeed_t VLC_MIN_VACTIVE_KMH = 15;
Cc_const volatile times_t VLC_START_T_REP_FUNC = 500;
Cc_const volatile times_t VLC_START_T_REP_FUNC_2 = 1000;
Cc_const volatile times_t VLC_T_REP_FUNC = 500;
Cc_const volatile times_long_t VLC_T_DRIVE_OFF_SMOOTH =
    Cc_default_drive_off_smooth_time;

Cc_const volatile factor_t
    VLC_FAC_ACT_TO_DISP_SPEED[2 * factor_actual_speed_to_display_speed_points] =
        {
            150, 1000,  //
            300, 1025   //
};
Cc_const volatile factor_t VLC_OFFSET_ACT_TO_DISP_SPEED
    [2 * offset_actual_speed_to_display_speed_points] = {
        150, 0,   //
        300, 125  //
};

/* v_ego [m/s] - max jerk [m/s�] */
Cc_const volatile acceleration_t
    MAX_DECEL_GRADIENT_CRITICAL[2 * max_decel_gradient_critical_points] = {
        500, -5000, /* 5m/s - 5 m/(s^3)*/
        2000, -2500 /* 20m/s - 2.5 m/(s^3)*/
};

/* Factor to increase the acceleration request gradient in critical traffic
 * situations */
/* criticality [%] - gain factor */
Cc_const volatile factor_t
    CRITICALITY_ACCEL_GRADIENT_GAIN[2 * accel_gradient_gain_points] = {
        0, 100,  /* 0% - x1 */
        100, 300 /* 100% - x2 */
};

/* Factor to reduce filter time in critical traffic situations */
/* criticality [%] - filter time [ms] */
Cc_const volatile times_t
    CRITICALITY_ACCEL_FILTER_TIME[2 * accel_filter_time_time_points] = {
        0, 200,  /* 0% - 300 ms */
        100, 100 /* 100% - 0 ms */
};

/*m/s        m/(s^2)*/
Cc_const volatile acceleration_t VLC_ACCEL_CURVE[2 * Cc_curve_points] = {
    /*BMW*/
    -1666, -1800,  //
    -833,  -1500,  //
    -555,  -1150,  //
    -277,  -800,   //
    -55,   -200,   //
    -27,   -140,   //
    -8,    -60,    //
    -4,    -40,    //
    0,     0,      //
    4,     40,     //
    8,     60,     //
    27,    100,    //
    55,    200,    //
    138,   400,    //
    277,   600,    //
    555,   1000,   //
    833,   1100,   //
    1200,  1200    //
};

/*m/s        m/(s^2)*/
Cc_const volatile acceleration_t
    VLC_ACCEL_CURVE_DELTA_SPEED_INT[2 *
                                    Cc_accel_curve_delta_speed_integ_points] = {
        -4000, -100,  //
        -2000, -50,   //
        -40,   0,     //
        40,    0,     //
        2000,  50,    //
        4000,  100    //
};

Cc_const volatile acceleration_t
    VLC_MAX_VEHICLE_ACCEL[2 * Cc_max_vehicle_accel_points] = {
        600,  2000,  //
        2000, 1500,  //
        5000, 800,   //
        7000, 400    //
};

Cc_const volatile acceleration_t
    VLC_MAX_VEHICLE_DECEL[2 * Cc_max_vehicle_decel_points] = {
        0,    -4000,  //
        500,  -4000,  //
        2000, -3500   //
};

// Maximum allowed acceleration for CC
// [m/s], [m/s2]
Cc_const volatile acceleration_t
    VLC_MAX_ACCEL_CURVE[2 * Cc_max_accel_curve_points] = {
        600,  2000,  //
        2000, 1500,  //
        5000, 800,   //
        7000, 400    //
};

// Minimum allowed actual acceleration for ACC active
Cc_const volatile acceleration_t VLC_ACTIVE_MIN_ACCEL_CURVE = -1000;

// Maximum allowed actual acceleration for ACC active
// [m/s], [m/s2]
Cc_const volatile acceleration_t
    VLC_ACTIVE_MAX_ACCEL_CURVE[2 * Cc_max_accel_curve_points] = {
        0,    8000,  //
        500,  8000,  //
        2000, 8000,  //
        4000, 8000   //
};

Cc_const volatile acceleration_t VLC_ACCEL_LIMIT = 1200;

Cc_const volatile acceleration_t VLC_DECEL_LIMIT_DEFAULT = -2000;
Cc_const volatile acceleration_t* pVLC_DecelLimit = &VLC_DECEL_LIMIT_DEFAULT;

Cc_const volatile acceleration_t VLC_MAX_DECEL_DURING_OVERRIDE =
    Accel_min; /* default Accel_min = -32767 */

Cc_const volatile acceleration_t VLC_MAX_DECEL_AFTER_OVERRIDE = -600;

Cc_const volatile acceleration_t VLC_MAX_DECEL_AFTER_ENGAGE = -400;

Cc_const volatile gradient_t VLC_ACCELMODE_MAX_NEG_GRAD = -800;

Cc_const volatile gradient_t VLC_ACCELMODE_MAX_POS_GRAD = 600;

Cc_const volatile factor_t VLC_POS_GRAD_ECO_FAC = 80;

Cc_const volatile factor_t VLC_POS_GRAD_SPORT_FAC = 120;

/* v_ego [m/s] - min speedometer offset [m/s] */
Cc_const volatile velocity_t
    VLC_MIN_VALID_SPEEDO_OFFSET[2 * Cc_min_valid_speedo_offset_points] = {
        0,    0, /*   0 kph - 0 kph   */
        2778, 0, /* 100 kph - 0 kph   */
        6000, 0  /* 216 kph - 0 kph   */
};

/* v_ego [m/s] - max speedometer offset [m/s] */
Cc_const volatile velocity_t
    VLC_MAX_VALID_SPEEDO_OFFSET[2 * Cc_max_valid_speedo_offset_points] = {
        0,    278, /*   0 kph - 10 kph   */
        2778, 278, /* 100 kph - 10 kph   */
        6000, 600  /* 216 kph - 21.6 kph */
};

Cc_const volatile acceleration_t
    VLC_MAX_POS_GRAD_DEFAULT_NEG_ACCEL[2 * Cc_pos_grad_neg_accel_points] = {
        100, 2500,  //
        1500, 2500  //
};

Cc_const volatile acceleration_t
    VLC_MAX_POS_GRAD_DEFAULT_POS_ACCEL[2 * Cc_pos_grad_pos_accel_points] = {
        100, 1250,  //
        1500, 750   //
};

Cc_const volatile acceleration_t* pVLC_MaxPosGradPosAccel =
    VLC_MAX_POS_GRAD_DEFAULT_POS_ACCEL;
Cc_const volatile acceleration_t* pVLC_MaxPosGradNegAccel =
    VLC_MAX_POS_GRAD_DEFAULT_NEG_ACCEL;

Cc_const volatile acceleration_t
    VLC_MAX_NEG_GRAD_DEFAULT[2 * Cc_neg_grad_points] = {
        100, -2000,  //
        1500, -2500  //
};

Cc_const volatile acceleration_t* pVLC_MaxNegGrad = VLC_MAX_NEG_GRAD_DEFAULT;

Cc_const volatile factor_t VLC_AVLC_TO_VLC_TRANSITON_FACTOR_DEFAULT = 500;
Cc_const volatile factor_t* pVLC_AccToCcTransitionFactor =
    &VLC_AVLC_TO_VLC_TRANSITON_FACTOR_DEFAULT;

Cc_const volatile gradient_t VLC_OFF_MAX_POS_GRAD = 2000;

Cc_const volatile gradient_t VLC_OFF_MAX_NEG_GRAD = -2000; /*600*/

Cc_const volatile gradient_t VLC_OFF_MAX_POS_GRAD_RAPID = 8000;

Cc_const volatile gradient_t VLC_OFF_MAX_NEG_GRAD_RAPID = -4000; /*-4000*/

Cc_const volatile acceleration_t VLC_MIN_DECEL_BRAKE_ONLY = -500;

Cc_const volatile times_t VLC_T_ABS_ACT_INH = 65000;

Cc_const volatile times_t VLC_T_ESP_ACT_INH = 500;

Cc_const volatile times_t VLC_T_TCS_ACT_INH = 65000;

Cc_const volatile setspeed_t VLC_MIN_RECOMMENDED_SPEED_KMH = (setspeed_t)60;

Cc_const volatile setspeed_t VLC_MIN_RECOMMENDED_SPEED_MPH = (setspeed_t)40;

Cc_const volatile percentage_t VLC_RECOMMENDED_SPEED_HYST = (percentage_t)40;

Cc_const volatile uint8 VLC_RECOMMENDED_SPEED_INTERVALS = (uint8)3;

Cc_const volatile times_t VLC_TFILT_POS_ALAT = 0; /*400*/
Cc_const volatile times_t VLC_TFILT_NEG_ALAT = 0; /*1000*/

/*curves for lateral acceleration limiter*/
/* (a lat max - a lat ego) m/(s^2)  (a long max) m/(s^2)*/
Cc_const volatile acceleration_t
    VLC_LONG_ACCEL_CURVE[2 * Cc_long_accel_curve_points] = {
        -8000, -800,   //
        -6000, -1000,  //
        -3500, -1200,  //
        -1200, -1200,  //
        350,   350,    //
        700,   700,    //
        1500,  1500,   //
        3000,  3000    //
};

/*v*/ /*a*/
Cc_const volatile velocity_t
    VLC_ALAT_SPEED_CURVE_DEFAULT[2 * Cc_alat_speed_curve_points] = {
        500,  2000,  //
        1200, 2600,  //
        1900, 2800,  //
        2700, 2800,  //
        6000, 1500,  //
        7500, 1500   //
};

Cc_const volatile velocity_t* pVLC_ALatSpeedCurve =
    VLC_ALAT_SPEED_CURVE_DEFAULT;

Cc_const volatile acceleration_t
    VLC_ACCEL_GAIN_CURVE[2 * Cc_accel_gain_curve_points] = {
        0,    2500,  //
        2000, 1500,  //
        5000, 400,   //
        7000, 0      //
};

Cc_const volatile acceleration_t LIM_ACCEL_CURVE[2 * Lim_curve_points] = {
    -7000, -1200,  //
    -555,  -999,   //
    -277,  -700,   //
    -138,  -400,   //
    -55,   -200,   //
    -27,   -100,   //
    0,     0,      //
    14,    100,    //
    27,    200,    //
    55,    300,    //
    138,   800,    //
    277,   1700,   //
    555,   3000,   //
    7000,  12000   //
};

Cc_const volatile acceleration_t AVLC_MAX_ACCEL_DISENGAGE =
    (acceleration_t)10000;

Cc_const volatile acceleration_t AVLC_MIN_ACCEL_DISENGAGE =
    (acceleration_t)-10000;

Cc_const volatile gradient_t LIM_OFF_MAX_POS_GRAD = 2000;

Cc_const volatile gradient_t LIM_OFF_MAX_NEG_GRAD = -2000;

Cc_const volatile gradient_t LIM_MAX_POS_GRAD = 2500;

Cc_const volatile gradient_t LIM_MAX_NEG_GRAD = -4000;

Cc_const volatile gradient_t LIM_OVERRIDE_MAX_POS_GRAD = 3500;

Cc_const volatile gradient_t LIM_OVERRIDE_MAX_NEG_GRAD = -3500;

Cc_const volatile gradient_t LIM_REST_MIN_GRAD = 500;

Cc_const volatile acceleration_t LIM_ACCEL_LIMIT = 10000;

Cc_const volatile acceleration_t LIM_DECEL_LIMIT = -2500;

Cc_const volatile setspeed_t PLIM_THRES_KMH = 7;

Cc_const volatile setspeed_t PLIM_HYST_KMH = 2;

Cc_const volatile setspeed_t PLIM_THRES_MPH = 4;

Cc_const volatile setspeed_t PLIM_HYST_MPH = 1;

Cc_const volatile gradient_t VLC_MIN_ENGAGE_A_GRAD = -10000;

Cc_const volatile gradient_t VLC_MAX_ENGAGE_A_GRAD = 10000;

Cc_const volatile velocity_t VLC_STATIC_VEL_THRES = 30;

Cc_const volatile velocity_t VLC_STATIC_DEC_THRES = -90;

Cc_const volatile velocity_t VLC_A_POS_OFFSET = 83;

Cc_const volatile velocity_t VLC_A_POS_OFFSET_MAX = 138;

Cc_const volatile velocity_t VLC_A_NEG_OFFSET = 83;

Cc_const volatile velocity_t VLC_A_NEG_OFFSET_MAX = -414;

Cc_const volatile times_t VLC_TMAX_DECEL_ONLY = 65533u;

Cc_const volatile times_t VLC_TMAX_DECEL_LIMIT = 65533u;

Cc_const volatile velocity_t VLC_V_OFFSET_SETSPEED = 50;

Cc_const volatile velocity_t VLC_V_DELTA_INT_UP_LIM = 4000;

Cc_const volatile velocity_t VLC_V_DELTA_INT_LOW_LIM = -4000;

Cc_const volatile velocity_t VLC_V_ACCELMODE = 6000;

Cc_const volatile velocity_t VLC_V_ACCELMODE_INIT = 277;

Cc_const volatile velocity_t VLC_V_DECELMODE = -6000;

Cc_const volatile velocity_t VLC_V_DECELMODE_INIT = -277;

Cc_const volatile gradient_t VLC_MAX_GRAD_RELEASE_BRAKE = 3500; /* 2500*/

Cc_const volatile gradient_t VLC_REST_MIN_GRAD = 20000;

Cc_const volatile velocity_t VLC_SMOOTH_GRADIENT_POS_MIN_VELOCITY =
    (velocity_t)100;

Cc_const volatile sint16 VLC_T_SMOOTH_GRADIENT_POS_TIME_DEFAULT
    [2 * Cc_smooth_gradient_pos_time_points] = {
        0, 400,   //
        500, 600  //
};

Cc_const volatile sint16* pVLC_SmoothGradientPosTime =
    VLC_T_SMOOTH_GRADIENT_POS_TIME_DEFAULT;

Cc_const volatile sint16 VLC_T_SMOOTH_GRADIENT_NEG_TIME_DEFAULT
    [2 * Cc_smooth_gradient_neg_time_points] = {
        0, 300,   //
        500, 450  //
};

Cc_const volatile sint16* pVLC_SmoothGradientNegTime =
    VLC_T_SMOOTH_GRADIENT_NEG_TIME_DEFAULT;

Cc_const volatile acceleration_t VLC_MAX_DECEL_DURING_ACCEL_MODE = -800;

Cc_const volatile acceleration_t VLC_MAX_ACCEL_DURING_DECEL_MODE = 400;

Cc_const volatile gradient_t VLC_SMOOTH_JERK_MIN_GRADIENT =
    (gradient_t)700; /*500; 128; 64*/

Cc_const volatile times_t VLC_ENGAGE_OVERRTIDE_TIME = (times_t)160;

/*******************************************************************************************/
/*                       A P P L I C A T I O N   P A R A M E T E R S */
/*******************************************************************************************/

Cc_const volatile times_t VLC_T_DRIVE_OFF_CONF =
    Cc_default_drive_off_confirmation_time;

Cc_const volatile times_t VLC_T_DRIVE_OFF_SET = 3000;

Cc_const volatile times_t VLC_T_STANDSTILL_DELAY = 1500;

Cc_const volatile times_t VLC_T_MOVING = 5000;

Cc_const volatile times_t VLC_T_DISPLAY_TAKEOVER_TIME = 400;

Cc_const volatile times_t VLC_BRAKE_PEDAL_MAX_TIME = 500;

Cc_const volatile velocity_t VLC_MAX_OPERATING_SPEED_ACC =
    5000; /* 50m/s => 180kph*/

Cc_const volatile velocity_t VLC_MAX_OPERATING_SPEED_FSRA =
    5000; /* 50m/s => 180kph*/

Cc_const volatile velocity_t VLC_MIN_OPERATING_SPEED_ACC =
    833; /* 8.33m/s => 30kph*/

Cc_const volatile velocity_t VLC_MIN_OPERATING_SPEED_FSRA = 0;

Cc_const volatile times_t VLC_FSRA_OBJ_LOSS_DEBOUNCE_TIME = 1000; /*1000 ms*/

Cc_const volatile times_t VLC_FSRA_CANCEL_WARNING_TIME = 1000; /*1000 ms*/

/*[%] internal translation of customer specific gap setting to a percentage
 * value*/
Cc_const volatile percentage_t VLC_HEADWAY_SETTING_1_THRESHOLD =
    (percentage_t)0;
Cc_const volatile percentage_t VLC_HEADWAY_SETTING_2_THRESHOLD =
    (percentage_t)25;
Cc_const volatile percentage_t VLC_HEADWAY_SETTING_3_THRESHOLD =
    (percentage_t)50;
Cc_const volatile percentage_t VLC_HEADWAY_SETTING_4_THRESHOLD =
    (percentage_t)75;
Cc_const volatile percentage_t VLC_HEADWAY_SETTING_5_THRESHOLD =
    (percentage_t)100;

Cc_const volatile acceleration_t VLC_DECEL_THRESHOLD_FOR_EXT_CRUISE_MODE =
    (acceleration_t)-400;

/*[cm/s], [mm/s�]. default acceleration request when no relevant target is
 * detected - linear interpolated.*/
Cc_const volatile acceleration_t VLC_ACCEL_DURING_EXT_CRUISE_MODE_LINEAR
    [2 * Cc_accel_during_ext_cruise_linear_points] = {
        833,  1550, /* 8.33m/s  => 30kph*/
        1388, 1550, /* 13.88m/s => 50kph*/
        2222, 1300, /* 22.22m/s => 80kph*/
        2777, 1100, /* 27.77m/s => 100kph*/
        3333, 1000, /* 33.33m/s => 120kph*/
        4166, 1000  /* 41.66m/s => 150kph*/
};

/*default acceleration request when no relevant target is detected - under 15
 * and under 30 kph.*/
Cc_const volatile velocity_t VLC_SPEED_THRESHOLD_15KPH =
    416; /* 4.16m/s  => 15kph*/
Cc_const volatile acceleration_t VLC_ACCEL_THRESHOLD_15KPH = 0;  // 0 m/s^2
Cc_const volatile velocity_t VLC_SPEED_THRESHOLD_30KPH =
    833; /* 8.33m/s  => 30kph*/
Cc_const volatile acceleration_t VLC_ACCEL_THRESHOLD_30KPH = 400;  // 0.4 m/s^2

/*parameters for setting soft stop request. maximum velocity and target distance
 */
Cc_const volatile velocity_t VLC_MAX_SOFTSTOP_REQUEST_VELOCITY =
    111; /* 1.11m/s => 4kph*/

Cc_const volatile distance_t VLC_MAX_SOFTSTOP_REQUEST_DISTANCE = 600; /*6m; 4m*/

/* parameters for adjusting customer jerk limits */
Cc_const volatile factor_t VLC_GAIN_GRAD[2 * Cc_gain_grad_points] = {
    278,  150, /* 2.78m/s => 10kph*/
    417,  120, /* 4.17m/s  => 15kph*/
    833,  110, /* 8.33m/s => 30kph*/
    1389, 105  /*13.89m/s => 50kph*/
};

void VLC_SELECT_PARAM_SET(void) {
    pVLC_MaxPosGradPosAccel = VLC_MAX_POS_GRAD_DEFAULT_POS_ACCEL;
    pVLC_MaxPosGradNegAccel = VLC_MAX_POS_GRAD_DEFAULT_NEG_ACCEL;
    pVLC_MaxNegGrad = VLC_MAX_NEG_GRAD_DEFAULT;

    pVLC_AccToCcTransitionFactor = &VLC_AVLC_TO_VLC_TRANSITON_FACTOR_DEFAULT;

    pVLC_SmoothGradientPosTime = VLC_T_SMOOTH_GRADIENT_POS_TIME_DEFAULT;
    pVLC_SmoothGradientNegTime = VLC_T_SMOOTH_GRADIENT_NEG_TIME_DEFAULT;

    pVLC_ALatSpeedCurve = VLC_ALAT_SPEED_CURVE_DEFAULT;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */