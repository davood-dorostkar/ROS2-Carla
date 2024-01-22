/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */
/*! \file **********************************************************************

  COMPONENT:              ACC (Adaptive Cruise Control)

  MODULENAME:             acc_frules.h

  @brief                  Fuzzy Rules Parameter Definition

  ---*/

#ifndef DECISION_SRC_VLC_COMMON_VLC_ACC_ACC_FRULES_H_
#define DECISION_SRC_VLC_COMMON_VLC_ACC_ACC_FRULES_H_
/*this file was created with FuzzyRuleCreator
 * ([D:\Projekte\Basisprojekte\vlc_base_project\codegen\vlc\fuzzy_rule_creator\data\accfuzzy.aft]
 * FSRA FuzzySet, fuzzy rule set for fsr acc)*/
// #include "vlc_long_sen.h"

#include "mat_fuzzy_ext.h"
#include "vlcSen_ext.h"

DLLEXPORT extern signed_fuzzy_t AVLC_DO_FUZZY(
    signed_fuzzy_t rel_distance,
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
    Fuzzy_Rule_Input_t *Fuzzy_Rule_Input);

/*output acceleration*/

/*brake command*/
extern const volatile fuzzy_var_t a_out_brake_hard;

extern const volatile fuzzy_var_t a_out_brake_large;

extern const volatile fuzzy_var_t a_out_brake_mid;

extern const volatile fuzzy_var_t a_out_brake;

/*do nothing*/
extern const volatile fuzzy_var_t a_out_roll;

/*accelerate*/
extern const volatile fuzzy_var_t a_out_accelerate_mid;

extern const volatile fuzzy_var_t a_out_accelerate;

/*relative distance (dact/dset)*/

/*distance is to small*/
extern const volatile fuzzy_var_t rel_distance_close;

/*distance is OK*/
extern const volatile fuzzy_var_t rel_distance_OK;

extern const volatile fuzzy_var_t rel_distance_far;

extern const volatile fuzzy_var_t rel_distance_still_OK;

/*relative speed (m/s)*/

/*target vehicle is much slower than the own vehicle*/
extern const volatile fuzzy_var_t speed_rel_much_slower;

/*target vehicle is slower than the own vehicle*/
extern const volatile fuzzy_var_t speed_rel_slower;

/*target vehicle has the same speed as the own vehicle*/
extern const volatile fuzzy_var_t speed_rel_same_speed;

/*target vehicle is close to faster than the own vehicle*/
extern const volatile fuzzy_var_t speed_rel_close_to_faster;

/*target vehicle is faster than the own vehicle*/
extern const volatile fuzzy_var_t speed_rel_faster;

/*vehicle in front is middle slower*/
extern const volatile fuzzy_var_t speed_rel_middleslower;

/*vehicle in front is little bit slower*/
extern const volatile fuzzy_var_t speed_rel_littleslower;

extern const volatile fuzzy_var_t speed_rel_littlefaster;

/*extern const volatile fuzzy_var_t speed_rel_no_rel_speed;*/

/*extern const volatile fuzzy_var_t speed_rel_very_much_slower;*/

/*the object is really slower*/
extern const volatile fuzzy_var_t speed_rel_really_slower;

/*acceleration of the object*/

/*target vehicle is braking very hard*/
extern const volatile fuzzy_var_t a_obj_hard_braking;

/*target vehicle is braking*/
extern const volatile fuzzy_var_t a_obj_braking;

/*target vehicle does not accelerate*/
extern const volatile fuzzy_var_t a_obj_rolling;

/*target vehicle is accelerating*/
extern const volatile fuzzy_var_t a_obj_accelerating;

/*vehicle in front is acceleration slightly*/
extern const volatile fuzzy_var_t a_obj_lowaccel;

/*target vehicle is braking fast*/
extern const volatile fuzzy_var_t a_obj_fast_braking;

/*target vehicle is braking slightly*/
extern const volatile fuzzy_var_t a_obj_soft_braking;

/*error (m) to the set distance*/

/*distance is to short*/
extern const volatile fuzzy_var_t distance_set_error_to_close;

/*distance is to far*/
extern const volatile fuzzy_var_t distance_set_error_to_far;

extern const volatile fuzzy_var_t distance_set_error_close;

/*distance to headway setting is not close enough*/
extern const volatile fuzzy_var_t distance_set_error_not_close_enough;

extern const volatile fuzzy_var_t distance_set_error_near;

extern const volatile fuzzy_var_t distance_set_error_more_than_requested;

/*distance error (m) to the minimum allowed (alert) distance*/

/*gap is a lot smaller than alert distance*/
/*extern const volatile fuzzy_var_t distance_min_error_to_close;*/

/*gap is a lot higher than alert distance*/
extern const volatile fuzzy_var_t distance_min_error_to_far;

/*gap is close to the alert distance*/
extern const volatile fuzzy_var_t distance_min_error_short;

/*gap is very close to the alert distance*/
extern const volatile fuzzy_var_t distance_min_error_veryshort;

/*gap is lower than alert distance*/
extern const volatile fuzzy_var_t distance_min_error_below;

/*gap is higher than alert distance*/
extern const volatile fuzzy_var_t distance_min_error_near;

/*factor (0-100) how soft the controller shall react*/

/*the reaction shall be soft*/
extern const volatile fuzzy_var_t softness_soft;

/*the reaction shall be very dynamic and restrictive to distance*/
extern const volatile fuzzy_var_t softness_very_dynamic;

/*own vehicle speed in m/s (scaled by 100)*/

/*vehicle is in still stand or slowly creeping */
extern const volatile fuzzy_var_t v_own_stillstand;

/*vehicle is slow*/
extern const volatile fuzzy_var_t v_own_slow;

/*vehicle is in low speed*/
extern const volatile fuzzy_var_t v_own_low_speed;

/*vehicle is in middle speed*/
extern const volatile fuzzy_var_t v_own_mid;

/*vehicle is in full range*/
extern const volatile fuzzy_var_t v_own_full_range;

/*vehicle is fast*/
extern const volatile fuzzy_var_t v_own_fast;

/*speed is not fast like on Highways*/
extern const volatile fuzzy_var_t v_own_urban;

/*Object speed*/

/*object speed in standstill*/
extern const volatile fuzzy_var_t v_obj_standstill;

/*object speed is fast*/
extern const volatile fuzzy_var_t v_obj_fast;

/*speed is not fast like on Highways*/
extern const volatile fuzzy_var_t v_obj_urban;

/*distance of control object*/

/*distance is to close*/
extern const volatile fuzzy_var_t distance_to_close;

/*time gap is too small*/
extern const volatile fuzzy_var_t time_gap_small;

/*time gap is middle*/
extern const volatile fuzzy_var_t time_gap_mid;

/*time gap is large*/
extern const volatile fuzzy_var_t time_gap_large;

#endif  // DECISION_SRC_VLC_COMMON_VLC_ACC_ACC_FRULES_H_
