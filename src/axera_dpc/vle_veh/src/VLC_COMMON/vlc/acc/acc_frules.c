/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */
/*! \file **********************************************************************

  COMPONENT:              ACC (Adaptive Cruise Control)

  MODULENAME:             acc_frules.c

  @brief                  Fuzzy Rules Parameter Definition

  ---*/
#include "mat_fuzzy_ext.h"
#include "acc_frules.h"
#include "sen_sim.h"

#define CAL_START_CODE
#include "Mem_Map.h"

const volatile fuzzy_var_t softness_very_dynamic = {20, 20, 20, 40};
const volatile fuzzy_var_t softness_soft = {40, 60, 60, 60};

const volatile fuzzy_var_t distance_to_close = {0, 0, 700, 1800};

const volatile fuzzy_var_t time_gap_small = {1500, 1500, 1500, 2500};
const volatile fuzzy_var_t time_gap_mid = {2000, 2000, 2000, 4000};
const volatile fuzzy_var_t time_gap_large = {2200, 2200, 2200, 5000};

const volatile fuzzy_var_t distance_min_error_below = {-600, -600, -600, 100};
const volatile fuzzy_var_t distance_min_error_veryshort = {0, 0, 0, 1000};
const volatile fuzzy_var_t distance_min_error_short = {0, 0, 0, 2500};
const volatile fuzzy_var_t distance_min_error_near = {0, 0, 0, 3550};
const volatile fuzzy_var_t distance_min_error_to_far = {100, 10000, 10000,
                                                        10000};

const volatile fuzzy_var_t distance_set_error_to_close = {-10000, -10000,
                                                          -10000, -100};
const volatile fuzzy_var_t distance_set_error_close = {-800, -800, -800, 0};
const volatile fuzzy_var_t distance_set_error_more_than_requested = {
    150, 1200, 1200, 1200};
const volatile fuzzy_var_t distance_set_error_not_close_enough = {1, 2000, 2000,
                                                                  2000};
const volatile fuzzy_var_t distance_set_error_near = {0, 0, 800, 3000};
const volatile fuzzy_var_t distance_set_error_to_far = {100, 15000, 15000,
                                                        15000};

const volatile fuzzy_var_t rel_distance_close = {0, 0, 500, 1000};
const volatile fuzzy_var_t rel_distance_still_OK = {500, 1000, 1000, 1250};
const volatile fuzzy_var_t rel_distance_OK = {800, 1000, 1000, 1300};
const volatile fuzzy_var_t rel_distance_close_to_far = {1000, 1000, 1400, 4000};
const volatile fuzzy_var_t rel_distance_far = {1000, 2000, 2000, 2000};

const volatile fuzzy_var_t speed_rel_much_slower = {-1200, -1200, -1200, -200};
const volatile fuzzy_var_t speed_rel_really_slower = {-800, -800, -800, -100};
const volatile fuzzy_var_t speed_rel_slower = {-1000, -1000, -1000, -100};
const volatile fuzzy_var_t speed_rel_middleslower = {-1000, -1000, -1000, -50};
const volatile fuzzy_var_t speed_rel_littleslower = {-300, -300, -300, -25};
const volatile fuzzy_var_t speed_rel_same_speed = {-200, 0, 0, 250};
const volatile fuzzy_var_t speed_rel_littlefaster = {4, 100, 100, 100};
const volatile fuzzy_var_t speed_rel_close_to_faster = {-150, 300, 300, 300};
const volatile fuzzy_var_t speed_rel_faster = {10, 500, 500, 500};

const volatile fuzzy_var_t v_own_stillstand = {0, 0, 0, 50};
const volatile fuzzy_var_t v_own_slow = {0, 0, 0, 1000};
const volatile fuzzy_var_t v_own_low_speed = {100, 100, 100, 300};
const volatile fuzzy_var_t v_own_urban = {500, 500, 500, 2000};
const volatile fuzzy_var_t v_own_mid = {500, 1000, 1000, 1000};
const volatile fuzzy_var_t v_own_full_range = {1000, 1000, 1000, 7000};
const volatile fuzzy_var_t v_own_fast = {2000, 4000, 4000, 4000};

const volatile fuzzy_var_t v_obj_standstill = {0, 0, 0, 250};
const volatile fuzzy_var_t v_obj_urban = {0, 0, 0, 2000};
const volatile fuzzy_var_t v_obj_fast = {2000, 3500, 3500, 3500};

const volatile fuzzy_var_t a_obj_hard_braking = {-10000, -10000, -10000, -4000};
const volatile fuzzy_var_t a_obj_mid_braking = {-4500, -4500, -4500, -500};
const volatile fuzzy_var_t a_obj_braking = {-6000, -6000, -6000, -600};
const volatile fuzzy_var_t a_obj_fast_braking = {-4500, -4500, -4500, -500};
const volatile fuzzy_var_t a_obj_soft_braking = {-1500, -1500, -1500, 0};
const volatile fuzzy_var_t a_obj_rolling = {-2000, 0, 0, 1500};
const volatile fuzzy_var_t a_obj_lowaccel = {20, 1000, 1000, 1000};
const volatile fuzzy_var_t a_obj_accelerating = {40, 4000, 4000, 4000};

const volatile fuzzy_var_t a_out_brake_hard = {-10000, -10000, -10000, -4000};
const volatile fuzzy_var_t a_out_brake_large = {-14000, -14000, -14000, 13000};
const volatile fuzzy_var_t a_out_brake_mid = {-12000, -12000, -12000, 11000};
const volatile fuzzy_var_t a_out_brake = {-13000, -13000, -13000, 13000};
const volatile fuzzy_var_t a_out_roll = {-3000, -1000, 1000, 3000};
const volatile fuzzy_var_t a_out_accelerate_mid = {-7000, 8000, 8000, 8000};
const volatile fuzzy_var_t a_out_accelerate = {0, 3000, 4000, 4000};

const volatile signed_fuzzy_t scale_array[47] = {
    256,   // Fuzzy-Rule  0
    128,   // Fuzzy-Rule  1
    0,     // Fuzzy-Rule  2
    0,     // Fuzzy-Rule  3
    0,     // Fuzzy-Rule  4
    0,     // Fuzzy-Rule  5
    1024,  // Fuzzy-Rule  6
    1024,  // Fuzzy-Rule  7
    1024,  // Fuzzy-Rule  8
    1024,  // Fuzzy-Rule  9
    0,     // Fuzzy-Rule  10
    0,     // Fuzzy-Rule  11
    1024,  // Fuzzy-Rule  12
    0,     // Fuzzy-Rule  13
    0,     // Fuzzy-Rule  14
    1024,  // Fuzzy-Rule  15
    768,   // Fuzzy-Rule  16
    0,     // Fuzzy-Rule  17
    0,     // Fuzzy-Rule  18
    0,     // Fuzzy-Rule  19
    128,   // Fuzzy-Rule  20
    256,   // Fuzzy-Rule  21
    0,     // Fuzzy-Rule  22
    512,   // Fuzzy-Rule  23
    0,     // Fuzzy-Rule  24
    0,     // Fuzzy-Rule  25
    128,   // Fuzzy-Rule  26
    128,   // Fuzzy-Rule  27
    0,     // Fuzzy-Rule  28
    51,    // Fuzzy-Rule  29
    128,   // Fuzzy-Rule  30
    0,     // Fuzzy-Rule  31
    0,     // Fuzzy-Rule  32
    64,    // Fuzzy-Rule  33
    0,     // Fuzzy-Rule  34
    0,     // Fuzzy-Rule  35
    64,    // Fuzzy-Rule  36
    128,   // Fuzzy-Rule  37
    0,     // Fuzzy-Rule  38
};

#define CAL_STOP_CODE
#include "Mem_Map.h"

// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h"
/*************************************************************************************************************************
  Functionname:    AVLC_DO_FUZZY */ /*!

  @brief           Calculate the output acceleration of the fuzzy controller

  @description     Depending on different combinations of the input variables, which interpret the elemental maneuvers, 
                   the desired actions of acceleration, roll, brake and brake hard are activated with their weights through
                   the fuzzy rules. The output acceleration is a combination of these four actions based on their weights.
                   The sum-prod method is used for inferencing this weight distribution between the actions. The centroid 
                   method is used for the defuzzification of the crisp output, which is the desired acceleration.
  
  @return          Acceleration in m/s2, multiplied with factor Acceleration_s. [-30000...30000u]

  @param[in]       rel_distance : the ratio between the actual distance and the current control distance, multiplied with factor Factor_s. [0...2147483648u]
  @param[in]       speed_rel : relative object longitudinal speed in m/s, multiplied with factor Velocity_s.. [-30000...30000u] 
  @param[in]       a_obj : object longitudinal acceleration in m/s2, multiplied with factor Acceleration_s. [-30000...30000u] 
  @param[in]       distance_set_error : the control error of the headway control in m, multiplied with factor Distance_s. [0...5*RW_VLC_MAX] 
  @param[in]       distance_min_error : the difference between the required distance and the maximum intrusion distance in m, multiplied with factor Distance_s. [0...5*RW_VLC_MAX] 
  @param[in]       softness : smoothness value for the longitudinal control in percent. [0u... 100u] 
  @param[in]       v_own :  the velocity of the host vehicle in m/s, multiplied with factor Velocity_s. [-30000...30000u] 
  @param[in]       v_obj :  the velocity of the object vehicle in m/s, multiplied with factor Velocity_s. [-30000...30000u] 
  @param[in]       distance : longitudinal displacement of object in m, multiplied with factor Distance_s. [0...5*RW_VLC_MAX]

  @glob_in         -
  @glob_out        -
  
  @c_switch_part   VLC_SIMULATION : Configuration swith for enabling PC simulation
  @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing
  @c_switch_full   CFG_VLC_ACC : Configuration switch enabeling Adaptive Cruise Control functionality (with object evaluation)

  @pre             - 
  @post            - 


**************************************************************************** */
DLLEXPORT signed_fuzzy_t AVLC_DO_FUZZY(signed_fuzzy_t rel_distance,
                                       signed_fuzzy_t speed_rel,
                                       signed_fuzzy_t a_obj,
                                       signed_fuzzy_t distance_set_error,
                                       signed_fuzzy_t distance_min_error,
                                       signed_fuzzy_t softness,
                                       signed_fuzzy_t v_own,
                                       signed_fuzzy_t v_obj,
                                       signed_fuzzy_t distance,
                                       signed_fuzzy_t time_gap,
                                       signed_fuzzy_t FuzzyAreaArray[47],
                                       signed_fuzzy_t FuzzyAreaPosArray[47],
                                       signed_fuzzy_t FuzzyMidArray[47],
                                       signed_fuzzy_t FuzzyValArray[47],
                                       Fuzzy_Rule_Input_t *Fuzzy_Rule_Input) {
    // softness
    signed_fuzzy_t fv_softness_very_dynamic;
    signed_fuzzy_t fv_softness_soft;

    {
        Fuzzy_Rule_Input->fv_softness_very_dynamic = fv_softness_very_dynamic =
            FUZZY_GET_FUZZY_VAL(&softness_very_dynamic, softness);

        Fuzzy_Rule_Input->fv_softness_soft = fv_softness_soft =
            FUZZY_GET_FUZZY_VAL(&softness_soft, softness);
    };

    // distance
    signed_fuzzy_t fv_distance_to_close;
    {
        Fuzzy_Rule_Input->fv_distance_to_close = fv_distance_to_close =
            FUZZY_GET_FUZZY_VAL(&distance_to_close, distance);
    };

    // time gap
    signed_fuzzy_t fv_time_gap_small;
    signed_fuzzy_t fv_time_gap_mid;
    signed_fuzzy_t fv_time_gap_large;
    {
        Fuzzy_Rule_Input->fv_time_gap_small = fv_time_gap_small =
            FUZZY_GET_FUZZY_VAL(&time_gap_small, time_gap);

        Fuzzy_Rule_Input->fv_time_gap_mid = fv_time_gap_mid =
            FUZZY_GET_FUZZY_VAL(&time_gap_mid, time_gap);

        Fuzzy_Rule_Input->fv_time_gap_large = fv_time_gap_large =
            FUZZY_GET_FUZZY_VAL(&time_gap_large, time_gap);
    };

    // distance_min_error
    signed_fuzzy_t fv_distance_min_error_below;
    signed_fuzzy_t fv_distance_min_error_veryshort;
    signed_fuzzy_t fv_distance_min_error_short;
    signed_fuzzy_t fv_distance_min_error_near;
    signed_fuzzy_t fv_distance_min_error_to_far;
    {
        Fuzzy_Rule_Input->fv_distance_min_error_below =
            fv_distance_min_error_below = FUZZY_GET_FUZZY_VAL(
                &distance_min_error_below, distance_min_error);

        Fuzzy_Rule_Input->fv_distance_min_error_veryshort =
            fv_distance_min_error_veryshort = FUZZY_GET_FUZZY_VAL(
                &distance_min_error_veryshort, distance_min_error);

        Fuzzy_Rule_Input->fv_distance_min_error_short =
            fv_distance_min_error_short = FUZZY_GET_FUZZY_VAL(
                &distance_min_error_short, distance_min_error);

        Fuzzy_Rule_Input->fv_distance_min_error_near =
            fv_distance_min_error_near = FUZZY_GET_FUZZY_VAL(
                &distance_min_error_near, distance_min_error);

        Fuzzy_Rule_Input->fv_distance_min_error_to_far =
            fv_distance_min_error_to_far = FUZZY_GET_FUZZY_VAL(
                &distance_min_error_to_far, distance_min_error);
    };

    // distance_set_error
    signed_fuzzy_t fv_distance_set_error_to_close;
    signed_fuzzy_t fv_distance_set_error_close;
    signed_fuzzy_t fv_distance_set_error_more_than_requested;
    signed_fuzzy_t fv_distance_set_error_not_close_enough;
    signed_fuzzy_t fv_distance_set_error_near;
    signed_fuzzy_t fv_distance_set_error_to_far;
    {
        Fuzzy_Rule_Input->fv_distance_set_error_to_close =
            fv_distance_set_error_to_close = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_to_close, distance_set_error);

        Fuzzy_Rule_Input->fv_distance_set_error_close =
            fv_distance_set_error_close = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_close, distance_set_error);

        Fuzzy_Rule_Input->fv_distance_set_error_more_than_requested =
            fv_distance_set_error_more_than_requested = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_more_than_requested, distance_set_error);

        Fuzzy_Rule_Input->fv_distance_set_error_not_close_enough =
            fv_distance_set_error_not_close_enough = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_not_close_enough, distance_set_error);

        Fuzzy_Rule_Input->fv_distance_set_error_near =
            fv_distance_set_error_near = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_near, distance_set_error);

        Fuzzy_Rule_Input->fv_distance_set_error_to_far =
            fv_distance_set_error_to_far = FUZZY_GET_FUZZY_VAL(
                &distance_set_error_to_far, distance_set_error);
    };

    // rel_distance
    signed_fuzzy_t fv_rel_distance_close;
    signed_fuzzy_t fv_rel_distance_still_OK;
    signed_fuzzy_t fv_rel_distance_OK;
    signed_fuzzy_t fv_rel_distance_close_to_far;
    signed_fuzzy_t fv_rel_distance_far;
    {
        Fuzzy_Rule_Input->fv_rel_distance_close = fv_rel_distance_close =
            FUZZY_GET_FUZZY_VAL(&rel_distance_close, rel_distance);

        Fuzzy_Rule_Input->fv_rel_distance_still_OK = fv_rel_distance_still_OK =
            FUZZY_GET_FUZZY_VAL(&rel_distance_still_OK, rel_distance);

        Fuzzy_Rule_Input->fv_rel_distance_OK = fv_rel_distance_OK =
            FUZZY_GET_FUZZY_VAL(&rel_distance_OK, rel_distance);

        Fuzzy_Rule_Input->fv_rel_distance_close_to_far =
            fv_rel_distance_close_to_far =
                FUZZY_GET_FUZZY_VAL(&rel_distance_close_to_far, rel_distance);

        Fuzzy_Rule_Input->fv_rel_distance_far = fv_rel_distance_far =
            FUZZY_GET_FUZZY_VAL(&rel_distance_far, rel_distance);
    };

    // speed_rel
    signed_fuzzy_t fv_speed_rel_much_slower;
    signed_fuzzy_t fv_speed_rel_really_slower;
    signed_fuzzy_t fv_speed_rel_slower;
    signed_fuzzy_t fv_speed_rel_middleslower;
    signed_fuzzy_t fv_speed_rel_littleslower;
    signed_fuzzy_t fv_speed_rel_same_speed;
    signed_fuzzy_t fv_speed_rel_littlefaster;
    signed_fuzzy_t fv_speed_rel_faster;
    signed_fuzzy_t fv_speed_rel_close_to_faster;
    {
        Fuzzy_Rule_Input->fv_speed_rel_much_slower = fv_speed_rel_much_slower =
            FUZZY_GET_FUZZY_VAL(&speed_rel_much_slower, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_really_slower =
            fv_speed_rel_really_slower =
                FUZZY_GET_FUZZY_VAL(&speed_rel_really_slower, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_slower = fv_speed_rel_slower =
            FUZZY_GET_FUZZY_VAL(&speed_rel_slower, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_middleslower =
            fv_speed_rel_middleslower =
                FUZZY_GET_FUZZY_VAL(&speed_rel_middleslower, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_littleslower =
            fv_speed_rel_littleslower =
                FUZZY_GET_FUZZY_VAL(&speed_rel_littleslower, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_same_speed = fv_speed_rel_same_speed =
            FUZZY_GET_FUZZY_VAL(&speed_rel_same_speed, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_littlefaster =
            fv_speed_rel_littlefaster =
                FUZZY_GET_FUZZY_VAL(&speed_rel_littlefaster, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_faster = fv_speed_rel_faster =
            FUZZY_GET_FUZZY_VAL(&speed_rel_faster, speed_rel);

        Fuzzy_Rule_Input->fv_speed_rel_close_to_faster =
            fv_speed_rel_close_to_faster =
                FUZZY_GET_FUZZY_VAL(&speed_rel_close_to_faster, speed_rel);
    };

    // v_own
    signed_fuzzy_t fv_v_own_stillstand;
    signed_fuzzy_t fv_v_own_slow;
    signed_fuzzy_t fv_v_own_low_speed;
    signed_fuzzy_t fv_v_own_urban;
    signed_fuzzy_t fv_v_own_mid;
    signed_fuzzy_t fv_v_own_full_range;
    signed_fuzzy_t fv_v_own_fast;
    {
        Fuzzy_Rule_Input->fv_v_own_stillstand = fv_v_own_stillstand =
            FUZZY_GET_FUZZY_VAL(&v_own_stillstand, v_own);

        Fuzzy_Rule_Input->fv_v_own_slow = fv_v_own_slow =
            FUZZY_GET_FUZZY_VAL(&v_own_slow, v_own);

        Fuzzy_Rule_Input->fv_v_own_low_speed = fv_v_own_low_speed =
            FUZZY_GET_FUZZY_VAL(&v_own_low_speed, v_own);

        Fuzzy_Rule_Input->fv_v_own_urban = fv_v_own_urban =
            FUZZY_GET_FUZZY_VAL(&v_own_urban, v_own);

        Fuzzy_Rule_Input->fv_v_own_mid = fv_v_own_mid =
            FUZZY_GET_FUZZY_VAL(&v_own_mid, v_own);

        Fuzzy_Rule_Input->fv_v_own_full_range = fv_v_own_full_range =
            FUZZY_GET_FUZZY_VAL(&v_own_full_range, v_own);

        Fuzzy_Rule_Input->fv_v_own_fast = fv_v_own_fast =
            FUZZY_GET_FUZZY_VAL(&v_own_fast, v_own);
    };

    // v_obj
    signed_fuzzy_t fv_v_obj_standstill;
    signed_fuzzy_t fv_v_obj_urban;
    signed_fuzzy_t fv_v_obj_fast;
    {
        Fuzzy_Rule_Input->fv_v_obj_standstill = fv_v_obj_standstill =
            FUZZY_GET_FUZZY_VAL(&v_obj_standstill, v_obj);

        Fuzzy_Rule_Input->fv_v_obj_urban = fv_v_obj_urban =
            FUZZY_GET_FUZZY_VAL(&v_obj_urban, v_obj);

        Fuzzy_Rule_Input->fv_v_obj_fast = fv_v_obj_fast =
            FUZZY_GET_FUZZY_VAL(&v_obj_fast, v_obj);
    };

    // a_obj
    signed_fuzzy_t fv_a_obj_hard_braking;
    signed_fuzzy_t fv_a_obj_mid_braking;
    signed_fuzzy_t fv_a_obj_braking;
    signed_fuzzy_t fv_a_obj_fast_braking;
    signed_fuzzy_t fv_a_obj_soft_braking;
    signed_fuzzy_t fv_a_obj_rolling;
    signed_fuzzy_t fv_a_obj_lowaccel;
    signed_fuzzy_t fv_a_obj_accelerating;
    {
        Fuzzy_Rule_Input->fv_a_obj_hard_braking = fv_a_obj_hard_braking =
            FUZZY_GET_FUZZY_VAL(&a_obj_hard_braking, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_mid_braking = fv_a_obj_mid_braking =
            FUZZY_GET_FUZZY_VAL(&a_obj_mid_braking, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_braking = fv_a_obj_braking =
            FUZZY_GET_FUZZY_VAL(&a_obj_braking, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_fast_braking = fv_a_obj_fast_braking =
            FUZZY_GET_FUZZY_VAL(&a_obj_fast_braking, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_soft_braking = fv_a_obj_soft_braking =
            FUZZY_GET_FUZZY_VAL(&a_obj_soft_braking, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_rolling = fv_a_obj_rolling =
            FUZZY_GET_FUZZY_VAL(&a_obj_rolling, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_lowaccel = fv_a_obj_lowaccel =
            FUZZY_GET_FUZZY_VAL(&a_obj_lowaccel, a_obj);

        Fuzzy_Rule_Input->fv_a_obj_accelerating = fv_a_obj_accelerating =
            FUZZY_GET_FUZZY_VAL(&a_obj_accelerating, a_obj);
    };

    FUZZY_DEFUZZY_INIT((signed_fuzzy_t)-6000, (signed_fuzzy_t)4000);

    // brake hard
    {
        // Fuzzy-Rule  0: if(speed_rel_slower & a_obj_hard_braking &
        // distance_min_error_short & !v_own_fast) -> a_out_brake_hard (0.250)
        // brake hard to avoid a crash if slower object is also braking very
        //   hard
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_hard, (signed_fuzzy_t)scale_array[0],
            FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(fv_speed_rel_slower,
                                             fv_a_obj_hard_braking),
                                  fv_distance_min_error_short),
                       FUZZY_NOT(fv_v_own_fast)),
            &FuzzyAreaArray[0], &FuzzyAreaPosArray[0], &FuzzyMidArray[0],
            &FuzzyValArray[0]);

        // Fuzzy-Rule  1: if(rel_distance_close & a_obj_hard_braking &
        //   distance_min_error_veryshort) -> a_out_brake_hard (0.125)
        // brake hard to avoid a crash if object is also braking very hard
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_hard, (signed_fuzzy_t)scale_array[1],
            FUZZY_PROD(FUZZY_PROD(fv_rel_distance_close, fv_a_obj_hard_braking),
                       fv_distance_min_error_veryshort),
            &FuzzyAreaArray[1], &FuzzyAreaPosArray[1], &FuzzyMidArray[1],
            &FuzzyValArray[1]);
    };

    // brake
    {
        // Fuzzy-Rule 2 : if (speed_rel_really_slower & !a_obj_lowaccel &
        //                   distance_min_error_short & v_own_urban &
        //                   v_obj_urban)
        //                   ->a_out_brake(0.750)
        //         decrease relative speed if close to max intrusion distance
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[2],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(fv_speed_rel_really_slower,
                                                 FUZZY_NOT(fv_a_obj_lowaccel)),
                                      fv_distance_min_error_short),
                           fv_v_own_urban),
                fv_v_obj_urban),
            &FuzzyAreaArray[2], &FuzzyAreaPosArray[2], &FuzzyMidArray[2],
            &FuzzyValArray[2]);

        // Fuzzy-Rule 3 : if (speed_rel_littleslower & !a_obj_lowaccel &
        //                   !distance_set_error_not_close_enough & v_own_fast &
        //                   v_obj_fas->a_out_brake(0.250)
        // Prevent release of brake pressure at the relative velocity
        // zero crossing upon an approach.
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[3],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(
                        FUZZY_PROD(fv_speed_rel_littleslower,
                                   FUZZY_NOT(fv_a_obj_lowaccel)),
                        FUZZY_NOT(fv_distance_set_error_not_close_enough)),
                    fv_v_own_fast),
                fv_v_obj_fast),
            &FuzzyAreaArray[3], &FuzzyAreaPosArray[3], &FuzzyMidArray[3],
            &FuzzyValArray[3]);

        // Fuzzy-Rule  4: if(speed_rel_littleslower & !a_obj_lowaccel &
        //   distance_min_error_near & v_own_urban) -> a_out_brake (0.250)
        // Reduce the relative velocity only when headway moves towards the
        // alert
        //   headway, to prevent too early braking with respect to far objects
        //   on urban roads.
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[4],
            FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(fv_speed_rel_slower,
                                             FUZZY_NOT(fv_a_obj_lowaccel)),
                                  fv_distance_min_error_short),
                       fv_v_own_urban),
            &FuzzyAreaArray[4], &FuzzyAreaPosArray[4], &FuzzyMidArray[4],
            &FuzzyValArray[4]);

        // Fuzzy-Rule  5: if(distance_to_close) -> a_out_brake (0.125)
        // brake to standstill
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_mid, (signed_fuzzy_t)scale_array[5],
            FUZZY_PROD(fv_distance_to_close, FUZZY_NOT(fv_v_own_stillstand)),
            &FuzzyAreaArray[5], &FuzzyAreaPosArray[5], &FuzzyMidArray[5],
            &FuzzyValArray[5]);

        // Fuzzy-Rule   6: if(rel_distance_close & speed_rel_slower) ->
        //   a_out_brake (0.125)
        // Headway intrusion and relative velocity negative (follow mode and
        //   approach)
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[6],
            FUZZY_PROD(FUZZY_PROD(fv_rel_distance_close,
                                  FUZZY_NOT(fv_speed_rel_close_to_faster)),
                       FUZZY_NOT(fv_v_own_urban)),
            &FuzzyAreaArray[6], &FuzzyAreaPosArray[6], &FuzzyMidArray[6],
            &FuzzyValArray[6]);

        // Fuzzy-Rule  7: if(a_obj_braking & distance_min_error_veryshort &
        //   !v_obj_standstill) -> a_out_brake (0.125)
        // Decelerate to keep the headway above the alert headway when following
        //   a decelerating object. (follow mode)
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_large, (signed_fuzzy_t)scale_array[7],
            FUZZY_PROD(FUZZY_PROD(fv_a_obj_fast_braking, fv_time_gap_small),
                       FUZZY_PROD(FUZZY_NOT(fv_speed_rel_close_to_faster),
                                  FUZZY_NOT(fv_v_own_low_speed))),
            &FuzzyAreaArray[7], &FuzzyAreaPosArray[7], &FuzzyMidArray[7],
            &FuzzyValArray[7]);

        // Fuzzy-Rule  8: if(!speed_rel_littlefaster & distance_min_error_below
        //   & !v_own_stillstand) -> a_out_brake (0.125)
        // Decelerate to keep the headway above the alert distance with no
        //   opening velocity.
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_mid, (signed_fuzzy_t)scale_array[8],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(fv_distance_min_error_below,
                                      FUZZY_NOT(fv_speed_rel_close_to_faster)),
                           FUZZY_NOT(fv_a_obj_lowaccel)),
                fv_v_own_urban),
            &FuzzyAreaArray[8], &FuzzyAreaPosArray[8], &FuzzyMidArray[8],
            &FuzzyValArray[8]);

        // Fuzzy-Rule  9: if(speed_rel_littleslower & a_obj_rolling &
        //   !distance_set_error_not_close_enough & !v_own_urban) -> a_out_brake
        //   (0.075)/
        // react on little slower objects
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_mid, (signed_fuzzy_t)scale_array[9],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(fv_distance_set_error_close,
                                      FUZZY_NOT(fv_speed_rel_close_to_faster)),
                           FUZZY_NOT(fv_a_obj_lowaccel)),
                fv_v_own_urban),
            &FuzzyAreaArray[9], &FuzzyAreaPosArray[9], &FuzzyMidArray[9],
            &FuzzyValArray[9]);

        // Fuzzy-Rule  10: if(!speed_rel_littlefaster & distance_min_error_below
        //   & v_own_urban) -> a_out_brake (0.050)/
        // Decelerate to keep the headway above the alert distance with no
        //   opening velocity on urban road.
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_large, (signed_fuzzy_t)scale_array[10],
            FUZZY_PROD(
                FUZZY_PROD(
                    fv_a_obj_mid_braking,
                    FUZZY_NOT(fv_distance_set_error_more_than_requested)),
                FUZZY_PROD(FUZZY_NOT(fv_speed_rel_close_to_faster),
                           fv_v_own_mid)),
            &FuzzyAreaArray[10], &FuzzyAreaPosArray[10], &FuzzyMidArray[10],
            &FuzzyValArray[10]);

        // Fuzzy-Rule  11: if(!a_obj_lowaccel & distance_set_error_to_close &
        //   v_own_fast & v_obj_fast) -> a_out_brake (0.050)
        // /Open up the gap when both vehicles have same velocity.
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[11],
            FUZZY_PROD(FUZZY_PROD(fv_a_obj_soft_braking,
                                  FUZZY_PROD(fv_distance_set_error_near,
                                             fv_rel_distance_close_to_far)),
                       fv_v_own_slow),
            &FuzzyAreaArray[11], &FuzzyAreaPosArray[11], &FuzzyMidArray[11],
            &FuzzyValArray[11]);

        // Fuzzy-Rule   12: if(speed_rel_slower & !a_obj_lowaccel &
        //   !distance_min_error_to_far & !v_own_urban) -> a_out_brake (0.010)/
        // Decelerate to keep the headway above the alert headway at closing
        //   velocity and object not accelerating, urban roads excluded, this is
        //   handled by other rules. (approach)
        FUZZY_DEFUZZY_ADD(&a_out_brake_large, (signed_fuzzy_t)scale_array[12],
                          FUZZY_PROD(FUZZY_PROD(fv_speed_rel_really_slower,
                                                fv_time_gap_large),
                                     fv_v_obj_standstill),
                          &FuzzyAreaArray[12], &FuzzyAreaPosArray[12],
                          &FuzzyMidArray[12], &FuzzyValArray[12]);

        // Fuzzy-Rule  13: if(speed_rel_littlefaster & !a_obj_lowaccel &
        //   !distance_set_error_to_far & v_own_urban) -> a_out_brake (0.013)
        FUZZY_DEFUZZY_ADD(
            &a_out_brake_mid, (signed_fuzzy_t)scale_array[13],
            FUZZY_PROD(FUZZY_PROD(fv_speed_rel_littleslower,
                                  FUZZY_PROD(fv_distance_set_error_near,
                                             fv_rel_distance_close_to_far)),
                       fv_v_own_urban),
            &FuzzyAreaArray[13], &FuzzyAreaPosArray[13], &FuzzyMidArray[13],
            &FuzzyValArray[13]);

        // Fuzzy-Rule   14: if(rel_distance_close & !speed_rel_faster &
        //   !v_own_stillstand) -> a_out_brake (0.010)
        // Headway intrusion and no opening velocity (cut in of vehicle without
        //   opening velocity)
        FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)scale_array[14],
                          FUZZY_PROD(FUZZY_PROD(fv_speed_rel_much_slower,
                                                fv_rel_distance_close_to_far),
                                     FUZZY_NOT(fv_v_own_fast)),
                          &FuzzyAreaArray[14], &FuzzyAreaPosArray[14],
                          &FuzzyMidArray[14], &FuzzyValArray[14]);

        // Fuzzy-Rule   15: if(speed_rel_much_slower & !distance_set_error_near
        // &
        //   !softness_soft & !v_own_urban) -> a_out_brake (0.005)
        // used to avoid headway intrusion to much slower objects -> was 0.005
        FUZZY_DEFUZZY_ADD(
            &a_out_brake, (signed_fuzzy_t)scale_array[15],
            FUZZY_PROD(FUZZY_PROD(fv_speed_rel_middleslower, fv_time_gap_mid),
                       fv_v_own_full_range),
            &FuzzyAreaArray[15], &FuzzyAreaPosArray[15], &FuzzyMidArray[15],
            &FuzzyValArray[15]);
    };

    // accelerate
    {
        // Fuzzy-Rule  16: if(!rel_distance_close & !speed_rel_slower &
        //   a_obj_accelerating & v_own_slow) -> a_out_accelerate (1.000)
        // Keep up with accelerating object that is not to close and no closing
        //   velocity in low speed range
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[16],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_rel_distance_close),
                                      FUZZY_NOT(fv_speed_rel_littleslower)),
                           fv_a_obj_accelerating),
                fv_v_own_slow),
            &FuzzyAreaArray[16], &FuzzyAreaPosArray[16], &FuzzyMidArray[16],
            &FuzzyValArray[16]);

        // Fuzzy-Rule  17: if(!speed_rel_much_slower & !a_obj_hard_braking &
        //   distance_set_error_not_close_enough & softness_very_dynamic &
        //   !v_own_urban & !distance_to_close) -> a_out_accelerate (0.500)
        // Overtake maneuver: The target headway is artificially decreased based
        //   on the lane change probability.
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[17],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(
                        FUZZY_PROD(
                            FUZZY_PROD(FUZZY_NOT(fv_speed_rel_much_slower),
                                       FUZZY_NOT(fv_a_obj_hard_braking)),
                            fv_distance_set_error_not_close_enough),
                        fv_softness_very_dynamic),
                    FUZZY_NOT(fv_v_own_urban)),
                FUZZY_NOT(fv_distance_to_close)),
            &FuzzyAreaArray[17], &FuzzyAreaPosArray[17], &FuzzyMidArray[17],
            &FuzzyValArray[17]);

        // Fuzzy-Rule  18: if(!speed_rel_littleslower &
        //   distance_set_error_more_than_requested & v_own_slow) ->
        //   a_out_accelerate (0.500)
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[18],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower),
                               fv_distance_set_error_more_than_requested),
                    fv_v_own_slow),
                fv_a_obj_lowaccel),
            &FuzzyAreaArray[18], &FuzzyAreaPosArray[18], &FuzzyMidArray[18],
            &FuzzyValArray[18]);

        // Fuzzy-Rule  19: if(rel_distance_far & speed_rel_littleslower &
        //   a_obj_lowaccel & v_own_urban & v_obj_urban) -> a_out_accelerate
        //   (0.250)
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[19],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(FUZZY_PROD(fv_rel_distance_far,
                                          FUZZY_NOT(fv_speed_rel_littleslower)),
                               fv_a_obj_lowaccel),
                    fv_v_own_urban),
                fv_v_obj_urban),
            &FuzzyAreaArray[19], &FuzzyAreaPosArray[19], &FuzzyMidArray[19],
            &FuzzyValArray[19]);

        // Fuzzy-Rule  20: if(rel_distance_far & !speed_rel_littleslower &
        //   !v_own_stillstand) -> a_out_accelerate (0.250)
        // Keep the headway at the driver selected headway with no closing
        //   velocity. Exclude the velocity range close to standstill.
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[20],
            FUZZY_PROD(
                FUZZY_PROD(fv_rel_distance_far,
                           FUZZY_NOT(fv_speed_rel_littleslower)),
                FUZZY_PROD(FUZZY_NOT(fv_v_own_stillstand),
                           FUZZY_PROD(FUZZY_NOT(fv_v_obj_standstill),
                                      FUZZY_NOT(fv_a_obj_soft_braking)))),
            &FuzzyAreaArray[20], &FuzzyAreaPosArray[20], &FuzzyMidArray[20],
            &FuzzyValArray[20]);

        // Fuzzy-Rule   21: if(speed_rel_faster & !a_obj_braking) ->
        //   a_out_accelerate (0.250)
        // Follow object that is faster than host vehicle
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate_mid, (signed_fuzzy_t)scale_array[21],
            FUZZY_PROD(FUZZY_PROD(fv_speed_rel_faster,
                                  FUZZY_NOT(fv_a_obj_soft_braking)),
                       FUZZY_NOT(fv_distance_set_error_close)),
            &FuzzyAreaArray[21], &FuzzyAreaPosArray[21], &FuzzyMidArray[21],
            &FuzzyValArray[21]);

        // Fuzzy-Rule  22: if(!rel_distance_close & speed_rel_faster &
        //   !a_obj_braking & v_own_slow) -> a_out_accelerate (0.250)
        // Keep up with faster object that is not to close and not decelerating
        //   in low speed range excusive for low speed range. Rule 31 for high
        //   speed range.
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[22],
            FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_rel_distance_close),
                                             fv_speed_rel_faster),
                                  FUZZY_NOT(fv_a_obj_braking)),
                       fv_v_own_slow),
            &FuzzyAreaArray[22], &FuzzyAreaPosArray[22], &FuzzyMidArray[22],
            &FuzzyValArray[22]);

        // Fuzzy-Rule  23: if(!speed_rel_slower &
        //   distance_set_error_not_close_enough & v_own_stillstand) ->
        //   a_out_accelerate (0.200)
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[23],
            FUZZY_PROD(FUZZY_PROD(fv_distance_set_error_not_close_enough,
                                  FUZZY_NOT(fv_speed_rel_slower)),
                       FUZZY_PROD(FUZZY_NOT(fv_a_obj_soft_braking),
                                  fv_v_own_stillstand)),
            &FuzzyAreaArray[23], &FuzzyAreaPosArray[23], &FuzzyMidArray[23],
            &FuzzyValArray[23]);

        // Fuzzy-Rule  24: if(rel_distance_far & !speed_rel_much_slower &
        //   !a_obj_soft_braking & v_own_urban & v_obj_urban) ->
        //   a_out_accelerate (0.150)
        // for long range approach to very slow objects
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[24],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(FUZZY_PROD(fv_rel_distance_far,
                                          FUZZY_NOT(fv_speed_rel_much_slower)),
                               FUZZY_NOT(fv_a_obj_soft_braking)),
                    fv_v_own_urban),
                fv_v_obj_urban),
            &FuzzyAreaArray[24], &FuzzyAreaPosArray[24], &FuzzyMidArray[24],
            &FuzzyValArray[24]);

        // Fuzzy-Rule  25: if(speed_rel_littlefaster & !a_obj_braking &
        //   v_own_fast & v_obj_fast) -> a_out_accelerate (0.125)
        // Keep up with faster object that is not decelerating in high speed
        //   range. Corresponding rule for low speed range is 22.
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[25],
            FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(fv_speed_rel_faster,
                                             FUZZY_NOT(fv_a_obj_braking)),
                                  fv_v_own_fast),
                       fv_v_obj_fast),
            &FuzzyAreaArray[25], &FuzzyAreaPosArray[25], &FuzzyMidArray[25],
            &FuzzyValArray[25]);

        // Fuzzy-Rule  26: if(!speed_rel_much_slower & !a_obj_soft_braking &
        //   distance_set_error_to_far & !v_own_urban & !v_obj_urban) ->
        //   a_out_accelerate (0.125)
        // get closer to an object in almost steady state with too long headway
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[26],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(FUZZY_PROD(fv_speed_rel_littlefaster,
                                          FUZZY_NOT(fv_a_obj_soft_braking)),
                               fv_distance_set_error_more_than_requested),
                    FUZZY_NOT(fv_v_own_urban)),
                FUZZY_NOT(fv_v_obj_urban)),
            &FuzzyAreaArray[26], &FuzzyAreaPosArray[26], &FuzzyMidArray[26],
            &FuzzyValArray[26]);

        // Fuzzy-Rule   27: if(!speed_rel_littleslower & a_obj_accelerating &
        //   distance_min_error_to_far) -> a_out_accelerate (0.125)
        // Keep up with accelerating object when a certain minimum distance is
        //   given
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[27],
            FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower),
                                  fv_a_obj_accelerating),
                       fv_distance_min_error_to_far),
            &FuzzyAreaArray[27], &FuzzyAreaPosArray[27], &FuzzyMidArray[27],
            &FuzzyValArray[27]);

        // Fuzzy-Rule  28: if(!speed_rel_littleslower & !a_obj_soft_braking &
        //   distance_set_error_more_than_requested & !v_own_urban) ->
        //   a_out_accelerate (0.063)
        // rule for keeping the headway on highways if the object slightly
        //   accelerates
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[28],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower),
                                      FUZZY_NOT(fv_a_obj_soft_braking)),
                           fv_distance_set_error_more_than_requested),
                FUZZY_NOT(fv_v_own_urban)),
            &FuzzyAreaArray[28], &FuzzyAreaPosArray[28], &FuzzyMidArray[28],
            &FuzzyValArray[28]);

        // Fuzzy-Rule  29: if(rel_distance_still_OK & !speed_rel_slower &
        //   a_obj_accelerating & !softness_soft) -> a_out_accelerate (0.050)
        // used to increase dynamic behind an accelerating object
        FUZZY_DEFUZZY_ADD(
            &a_out_accelerate, (signed_fuzzy_t)scale_array[29],
            FUZZY_PROD(FUZZY_PROD(FUZZY_PROD(fv_rel_distance_still_OK,
                                             FUZZY_NOT(fv_speed_rel_slower)),
                                  fv_a_obj_accelerating),
                       FUZZY_NOT(fv_softness_soft)),
            &FuzzyAreaArray[29], &FuzzyAreaPosArray[29], &FuzzyMidArray[29],
            &FuzzyValArray[29]);
    };

    // roll
    {
        // Fuzzy-Rule  30: if(!speed_rel_littleslower & !a_obj_braking &
        //   !distance_set_error_close & v_own_stillstand) -> a_out_roll (0.500)
        // reduce dynamic in almost steady state
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[30],
            FUZZY_PROD(
                FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower),
                                      FUZZY_NOT(fv_a_obj_braking)),
                           FUZZY_NOT(fv_distance_set_error_close)),
                fv_v_own_stillstand),
            &FuzzyAreaArray[30], &FuzzyAreaPosArray[30], &FuzzyMidArray[30],
            &FuzzyValArray[30]);

        // Fuzzy-Rule  31: if(rel_distance_still_OK & !speed_rel_littleslower &
        //   !distance_min_error_short & softness_soft & !v_own_urban) ->
        //   a_out_roll (0.250)
        // Reduce dynamics during steady state following with no high closing
        //   velocity while the headway error does not get to large and comfort
        //   setting.
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[31],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_PROD(FUZZY_PROD(fv_rel_distance_still_OK,
                                          FUZZY_NOT(fv_speed_rel_littleslower)),
                               FUZZY_NOT(fv_distance_min_error_short)),
                    fv_softness_soft),
                FUZZY_NOT(fv_v_own_urban)),
            &FuzzyAreaArray[31], &FuzzyAreaPosArray[31], &FuzzyMidArray[31],
            &FuzzyValArray[31]);

        // Fuzzy-Rule  32: if(a_obj_braking & distance_set_error_to_far &
        //   !v_own_urban) -> a_out_roll (0.125)
        // avoid over-reaction to braking objects in far distance
        FUZZY_DEFUZZY_ADD(&a_out_roll, (signed_fuzzy_t)scale_array[32],
                          FUZZY_PROD(FUZZY_PROD(fv_a_obj_braking,
                                                fv_distance_set_error_to_far),
                                     FUZZY_NOT(fv_v_own_urban)),
                          &FuzzyAreaArray[32], &FuzzyAreaPosArray[32],
                          &FuzzyMidArray[32], &FuzzyValArray[32]);

        // Fuzzy-Rule  33: if(!a_obj_braking &
        //   !distance_set_error_more_than_requested & v_own_slow) -> a_out_roll
        //   (0.125)
        // reduce dynamics if enough distance is left
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[33],
            FUZZY_PROD(
                FUZZY_PROD(
                    FUZZY_NOT(fv_a_obj_braking),
                    FUZZY_NOT(fv_distance_set_error_more_than_requested)),
                fv_v_own_slow),
            &FuzzyAreaArray[33], &FuzzyAreaPosArray[33], &FuzzyMidArray[33],
            &FuzzyValArray[33]);

        // Fuzzy-Rule  34: if(!speed_rel_littleslower & v_own_fast) ->
        // a_out_roll
        //   (0.125)
        // Reduce dynamics during steady state following with no high closing
        //   velocity.
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[34],
            FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower), fv_v_own_fast),
            &FuzzyAreaArray[34], &FuzzyAreaPosArray[34], &FuzzyMidArray[34],
            &FuzzyValArray[34]);

        // Fuzzy-Rule   35: if(speed_rel_same_speed & v_own_stillstand) ->
        //   a_out_roll (0.100)
        // Decrease dynamics in the velocity range close to stand still
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[35],
            FUZZY_PROD(fv_speed_rel_same_speed, fv_v_own_stillstand),
            &FuzzyAreaArray[35], &FuzzyAreaPosArray[35], &FuzzyMidArray[35],
            &FuzzyValArray[35]);

        // Fuzzy-Rule  36: if(!speed_rel_slower & a_obj_rolling &
        //   !v_obj_standstill)
        //   -> a_out_roll (0.100)
        // Reduce dynamics during steady state following
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[36],
            FUZZY_PROD(FUZZY_PROD(FUZZY_NOT(fv_speed_rel_littleslower),
                                  fv_a_obj_rolling),
                       FUZZY_NOT(fv_v_obj_standstill)),
            &FuzzyAreaArray[36], &FuzzyAreaPosArray[36], &FuzzyMidArray[36],
            &FuzzyValArray[36]);

        // Fuzzy-Rule  37: if(rel_distance_OK & speed_rel_same_speed &
        //   a_obj_rolling)
        //   -> a_out_roll (0.100)
        // Steady state following
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[37],
            FUZZY_PROD(fv_rel_distance_OK, fv_speed_rel_same_speed),
            &FuzzyAreaArray[37], &FuzzyAreaPosArray[37], &FuzzyMidArray[37],
            &FuzzyValArray[37]);

        // Fuzzy-Rule  38: if(rel_distance_OK & !a_obj_hard_braking &
        //   !v_own_slow) -> a_out_roll (0.013)/
        // Reduce dynamics during steady state following with no high object
        //   deceleration.
        FUZZY_DEFUZZY_ADD(
            &a_out_roll, (signed_fuzzy_t)scale_array[38],
            FUZZY_PROD(FUZZY_PROD(fv_rel_distance_OK,
                                  FUZZY_NOT(fv_a_obj_hard_braking)),
                       FUZZY_NOT(fv_v_own_slow)),
            &FuzzyAreaArray[38], &FuzzyAreaPosArray[38], &FuzzyMidArray[38],
            &FuzzyValArray[38]);
    };

    /*Fuzzy-Rule  39: (!NOT ACTIVE!) if(rel_distance_close &
     * speed_rel_littleslower & v_own_fast & v_obj_fast) -> a_out_brake
     * (0.000)*/
    /*not used due to !very! low relevance (was 0.125)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)    scale_array[39],
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_PROD(
              fv_rel_distance_close,
              fv_speed_rel_littleslower),
            fv_v_own_fast),
          fv_v_obj_fast),&FuzzyAreaArray[39],
     &FuzzyAreaPosArray[39]);*/

    /*Fuzzy-Rule  36: (!NOT ACTIVE!) if(speed_rel_very_much_slower &
     * !a_obj_lowaccel & !distance_set_error_near & !v_own_urban) -> a_out_brake
     * (0.000)*/
    /*not used due to !very! low relevance (was 0.25) - rules 5/6/8 are
     * sufficient*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_PROD(
              fv_speed_rel_very_much_slower,
              FUZZY_NOT(
                fv_a_obj_lowaccel)),
            FUZZY_NOT(
              fv_distance_set_error_near)),
          FUZZY_NOT(
            fv_v_own_urban)),&FuzzyAreaArray[36],
     &FuzzyAreaPosArray[36]);*/

    /*Fuzzy-Rule   6: (!NOT ACTIVE!) if(speed_rel_slower & !a_obj_lowaccel &
     * !distance_set_error_to_far & !softness_soft & !v_own_urban) ->
     * a_out_brake (0.000)*/
    /*not used due to !very! low relevance (was 0.005) Decelerate to keep the
     * headway above the driver selected headway at closing velocity and object
     * not accelerating, at high speed. (high speed approach)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_PROD(
              FUZZY_PROD(
                fv_speed_rel_slower,
                FUZZY_NOT(
                  fv_a_obj_lowaccel)),
              FUZZY_NOT(
                fv_distance_set_error_to_far)),
            FUZZY_NOT(
              fv_softness_soft)),
          FUZZY_NOT(
            fv_v_own_urban)));*/

    /*Fuzzy-Rule   7: (!NOT ACTIVE!) if(speed_rel_much_slower &
     * distance_min_error_near & !v_own_urban) -> a_out_brake (0.000)*/
    /*used to avoid headway intrusion to much slower objects -> was 0.001
     * Decelerate to keep the headway above the alert headway at high closing
     * velocity, urban roads excluded; this is handled by rule 15. (approach)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            fv_speed_rel_much_slower,
            fv_distance_min_error_near),
          FUZZY_NOT(
            fv_v_own_urban)),&FuzzyAreaArray[7],
     &FuzzyAreaPosArray[7]);*/

    /*Fuzzy-Rule  15: (!NOT ACTIVE!) if(!speed_rel_faster &
     * distance_min_error_to_close & v_own_urban) -> a_out_brake (0.000)*/
    /*not used due to !very! low relevance (was 1.0)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_NOT(
              fv_speed_rel_faster),
            fv_distance_min_error_to_close),
          fv_v_own_urban),&FuzzyAreaArray[15],
     &FuzzyAreaPosArray[15]);*/

    /*Fuzzy-Rule  24: (!NOT ACTIVE!) if(speed_rel_slower & a_obj_hard_braking &
     * distance_min_error_veryshort & !v_own_fast) -> a_out_brake_hard (0.000)*/
    /*not used due to low relevance (was 0.25)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake_hard, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_PROD(
              fv_speed_rel_slower,
              fv_a_obj_hard_braking),
            fv_distance_min_error_veryshort),
          FUZZY_NOT(
            fv_v_own_fast)),&FuzzyAreaArray[24],
     &FuzzyAreaPosArray[24]);*/

    /*Fuzzy-Rule  25: (!NOT ACTIVE!) if(speed_rel_slower & a_obj_hard_braking &
     * distance_min_error_below & !v_own_fast) -> a_out_brake_hard (0.000)*/
    /*not used due to !very! low relevance (was 1.0)*/
    /*FUZZY_DEFUZZY_ADD(&a_out_brake_hard, (signed_fuzzy_t)    0,
        FUZZY_PROD(
          FUZZY_PROD(
            FUZZY_PROD(
              fv_speed_rel_slower,
              fv_a_obj_hard_braking),
            fv_distance_min_error_below),
          FUZZY_NOT(
            fv_v_own_fast)),&FuzzyAreaArray[25],
     &FuzzyAreaPosArray[25]);*/

    return FUZZY_DEFUZZY();
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */