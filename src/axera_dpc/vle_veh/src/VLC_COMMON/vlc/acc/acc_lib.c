/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "string.h"
#include "acc.h"

#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "mat_fuzzy_ext.h"

#include "acc_lib_ext.h"
#include "acc_lib.h"
#include "acc_par.h"
#include "acc_frules.h"
#include "sen_sim.h"

#define Fct_tts_max_time ((times_t)10000)

static times_t AVLC_CALC_OBJECT_TTS(const VLC_acc_object_t *const object);
static acceleration_t AVLC_CALC_STOP_DECEL(const velocity_t host_speed,
                                           const VLC_acc_object_t *const object,
                                           const times_t time_to_stop,
                                           percentage_t headway_setting);
static acceleration_t AVLC_CALC_APPROACH_DECEL(
    const VLC_acc_object_t *const Object, const distance_t RequestedDistance);

/************************************************************************
  @fn               AVLC_DETERMINE_DIST_CONTROL_ACCEL */ /*!

                   @brief            Call fuzzy controller

                   @description      calls the fuzzy controller for the given
                 Object

                   @return           Acceleration

                   @param[in]        Object
                   @param[in]        headway_setting
                   @param[in]        a_own
                   @param[in]        v_real

                   @pre              Precondition:  none

                   @post             Postcondition: none
                 ****************************************************************************
                 */
acceleration_t AVLC_DETERMINE_DIST_CONTROL_ACCEL(VLC_acc_object_t *const Object,
                                                 times_t MovingTime,
                                                 percentage_t headway_setting,
                                                 acceleration_t a_own,
                                                 velocity_t v_own) {
    sint32 req_acc, help_var;
    signed_fuzzy_t req_acc_fuzzy;
    acceleration_t req_acc_tts;
    signed_fuzzy_t accel;
    signed_fuzzy_t control_error;
    acceleration_t max_accel, max_decel, req_acc_dInt;
    distance_t requested_distance, max_intrusion_distance;
    times_t time_to_stop, max_pred_time;
    factor_t fac;

    time_to_stop = AVLC_DETERMINE_TIME_TO_STOP(a_own, v_own);

    max_pred_time =
        (times_t)MAT_MIN((sint32)Acc_predicted_reaction_time, time_to_stop);

    /*Predict vehicle velocity after prediction time */
    /*It is assumed that the host vehicle has a PT1 behaviour*/
    help_var = (sint32)a_own * (sint32)max_pred_time;
    help_var /= (sint32)Acceleration_s;
    help_var *= (sint32)Velocity_s;
    help_var /= (sint32)Time_s;

    v_own = (velocity_t)(v_own + help_var);
    v_own = (velocity_t)MAT_LIM((sint32)v_own, (sint32)0, (sint32)Speed_max);

    requested_distance = Object->REQUESTED_DISTANCE_MODIFIED_PRED;
    max_intrusion_distance = AVLC_DETERMINE_MAX_INTRUSION(
        AVLC_GET_ALERT_DISTANCE(Object), requested_distance, v_own,
        headway_setting, Object->AUTOSAR.REL_LONG_SPEED,
        Object->ALERT_MODIFICATION_FACTOR);

    max_accel = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_fsr_max_acceleration, (uint16)Acc_fsr_max_acceleration_points,
        v_own);
    max_decel = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_fsr_max_deceleration, (uint16)Acc_fsr_max_deceleration_points,
        v_own);

    /*calculate control accel every time, although object is not detected!*/
    accel = (signed_fuzzy_t)Object->LONG_ACCEL_MODIFIED;

    control_error = (signed_fuzzy_t)(Object->AUTOSAR.LONG_DISPLACEMENT -
                                     requested_distance);

    memset(Object->FuzzyAreaArray, 0, sizeof(Object->FuzzyAreaArray));
    memset(Object->FuzzyAreaPosArray, 0, sizeof(Object->FuzzyAreaPosArray));

    Object->Fuzzy_Signal_Input.fuzzy_rel_distance =
        ((signed_fuzzy_t)Object->AUTOSAR.LONG_DISPLACEMENT *
         (signed_fuzzy_t)Factor_s) /
        (signed_fuzzy_t)Object->REQUESTED_DISTANCE_MODIFIED_ACT;
    Object->Fuzzy_Signal_Input.fuzzy_speed_rel =
        (signed_fuzzy_t)(Object->AUTOSAR.REL_LONG_SPEED - help_var);
    Object->Fuzzy_Signal_Input.fuzzy_a_obj = accel;
    Object->Fuzzy_Signal_Input.fuzzy_distance_set_error = control_error;
    Object->Fuzzy_Signal_Input.fuzzy_distance_min_error =
        (signed_fuzzy_t)(Object->AUTOSAR.LONG_DISPLACEMENT -
                         max_intrusion_distance);
    Object->Fuzzy_Signal_Input.fuzzy_softness =
        (signed_fuzzy_t)Object->CONTROL_SMOOTHNESS;
    Object->Fuzzy_Signal_Input.fuzzy_v_own = (signed_fuzzy_t)v_own;
    Object->Fuzzy_Signal_Input.fuzzy_v_obj =
        (signed_fuzzy_t)(Object->LONG_SPEED);
    Object->Fuzzy_Signal_Input.fuzzy_distance =
        (signed_fuzzy_t)(Object->AUTOSAR.LONG_DISPLACEMENT);

    times_t time_gap;
    if (MAT_DIV(Object->AUTOSAR.LONG_DISPLACEMENT, Distance_s, v_own,
                Velocity_s, Time_s) > Acc_headway_time_thres) {
        time_gap = Acc_headway_time_thres;
    } else {
        time_gap = MAT_DIV(Object->AUTOSAR.LONG_DISPLACEMENT, Distance_s, v_own,
                           Velocity_s, Time_s);
    }
    Object->Fuzzy_Signal_Input.fuzzy_time_gap = (signed_fuzzy_t)time_gap;

    req_acc_fuzzy = AVLC_DO_FUZZY(
        ((signed_fuzzy_t)Object->AUTOSAR.LONG_DISPLACEMENT *
         (signed_fuzzy_t)Factor_s) /
            (signed_fuzzy_t)Object->REQUESTED_DISTANCE_MODIFIED_ACT,
        (signed_fuzzy_t)(Object->AUTOSAR.REL_LONG_SPEED - help_var), accel,
        control_error,
        (signed_fuzzy_t)(Object->AUTOSAR.LONG_DISPLACEMENT -
                         max_intrusion_distance),
        (signed_fuzzy_t)Object->CONTROL_SMOOTHNESS, (signed_fuzzy_t)v_own,
        (signed_fuzzy_t)(Object->LONG_SPEED),
        (signed_fuzzy_t)(Object->AUTOSAR.LONG_DISPLACEMENT),
        (signed_fuzzy_t)(time_gap), (signed_fuzzy_t *)(Object->FuzzyAreaArray),
        (signed_fuzzy_t *)(Object->FuzzyAreaPosArray),
        (signed_fuzzy_t *)(Object->FuzzyMidArray),
        (signed_fuzzy_t *)(Object->FuzzyValArray),
        (Fuzzy_Rule_Input_t *)(&Object->Fuzzy_Rule_Input));

    /*calculate needed deceleration to stop*/
    time_to_stop = AVLC_CALC_OBJECT_TTS(Object);
    req_acc_tts =
        AVLC_CALC_STOP_DECEL(v_own, Object, time_to_stop, headway_setting);
    /*modify req_acc_tts to reduce time of very little amount of decel*/
    // help_var = MAT_CALCULATE_PARAM_VALUE1D(Acc_calc_dec_ds_dist_fac,
    //                                        Acc_calc_dec_ds_dist_fac_points,
    //                                        Object->AUTOSAR.LONG_DISPLACEMENT);
    // help_var *= MAT_CALCULATE_PARAM_VALUE1D(
    //     Acc_calc_dec_ds_decl_fac, Acc_calc_dec_ds_decl_fac_points,
    //     req_acc_tts);
    // help_var /= Factor_s;
    // help_var = Factor_s - help_var;
    // help_var = MAT_LIM(help_var, 0, Factor_s);
    // req_acc_tts = (acceleration_t)MAT_MUL(req_acc_tts, help_var,
    // Acceleration_s,
    //                                       Factor_s, Acceleration_s);

    /*merge both accelerations*/
    // if (Object->AUTOSAR.LONG_DISPLACEMENT >
    //     MAT_CALCULATE_PARAM_VALUE1D(Acc_tts_acceleration_max_dist,
    //                                 Acc_tts_acceleration_max_dist_points,
    //                                 (sint16)v_own)) {
    //     fac = 0;
    // } else {
    //     fac = MAT_CALCULATE_PARAM_VALUE1D(
    //               Acc_calc_decel_moving_time_factor,
    //               Acc_calc_decel_moving_time_factor_points, MovingTime) *
    //           MAT_CALCULATE_PARAM_VALUE1D(Acc_use_calc_decel_factor,
    //                                       Acc_use_calc_decel_factor_points,
    //                                       (sint16)time_to_stop) /
    //           Scale_1000;
    // }
    fac = MAT_CALCULATE_PARAM_VALUE1D(Acc_use_calc_decel_factor,
                                      Acc_use_calc_decel_factor_points,
                                      (sint16)time_to_stop);

    if ((req_acc_fuzzy > (signed_fuzzy_t)Acc_min_resolution &&
         req_acc_tts >= Acc_minimum_calculated_brake_request) ||
        (v_own >= Acc_host_speed_for_tts_accel_use)) {
        req_acc = (acceleration_t)req_acc_fuzzy;
    } else {
        req_acc =
            (acceleration_t)(MAT_MUL(fac, req_acc_tts, Factor_s, Acceleration_s,
                                     Acceleration_s) +
                             MAT_MUL(Factor_s - fac, req_acc_fuzzy, Factor_s,
                                     Acceleration_s, Acceleration_s));
    }

    /*----------------------------------------------------------------------*/
    /*calculate needed deceleration for approaching*/
    req_acc_dInt = AVLC_CALC_APPROACH_DECEL(Object, max_intrusion_distance);
    /*select and merge both accelerations*/
    /*if approach situation*/
    if ((req_acc_dInt < 0) &&
        (Object->AUTOSAR.REL_LONG_SPEED <
         Acc_use_approach_decel_speed_factor
             [2 * Acc_use_approach_decel_speed_factor_points - 2]) &&
        (time_to_stop > 0)) {
        sint32 help;
        acceleration_t approach_accel;
        approach_accel = req_acc_dInt;
        /*merge with req_acc depending to relative speed*/
        help = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_use_approach_decel_speed_factor,
            Acc_use_approach_decel_speed_factor_points,
            Object->AUTOSAR.REL_LONG_SPEED);
        help = (help * approach_accel) + (Factor_s - help) * req_acc;
        approach_accel = (acceleration_t)(help / Factor_s);
        /*merge with req_accel depending on object accel*/
        help = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_use_approach_decel_v_host_factor,
            Acc_use_approach_decel_v_host_factor_points, v_own);
        help = (help * approach_accel) + (Factor_s - help) * req_acc;
        approach_accel = (acceleration_t)(help / Factor_s);
        /*merge with req_accel depending on requested decel to max intrusion*/
        help = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_use_approach_decel_decel_factor,
            Acc_use_approach_decel_decel_factor_points, req_acc_dInt);
        help = (help * approach_accel);  // ramp decel down to zero in case of
                                         // very smooth approach
        req_acc =
            (acceleration_t)MAT_LIM(help / Factor_s, approach_accel, req_acc);
    }

    Object->TTS = time_to_stop;
    Object->ACCEL_REQUEST_FUZZY = (acceleration_t)req_acc_fuzzy;
    Object->ACCEL_REQUEST_TTS = req_acc_tts;
    Object->ACCEL_REQUEST_DMIN = req_acc_dInt;

    req_acc = MAT_LIM(req_acc, (sint32)max_decel, (sint32)max_accel);
    return (acceleration_t)req_acc;
}

/* ***********************************************************************
  @fn               AVLC_CALC_APPROACH_DECEL */ /*!
                             @brief            calculates the needed
                           deceleration for approaching an object to get an
                           intrusion distance of RequestedDistance

                             @description      calculates the needed
                           deceleration for approaching an object to get an
                           intrusion distance of RequestedDistance

                             @param            Object
                             @param            RequestedDistance

                             @return           acceleration_t needed
                           deceleration for the approach
                             @pre              Precondition:  none

                             @post             Postcondition: none...
                           ****************************************************************************
                           */
static acceleration_t AVLC_CALC_APPROACH_DECEL(
    const VLC_acc_object_t *const Object, const distance_t RequestedDistance) {
    sint32 help;
    distance_t dist;
    acceleration_t return_val;

    return_val = Acceleration_s;  // positive return value in case of no
                                  // approach situation
    dist = (distance_t)MAT_MAX(
        (sint32)Distance_s,
        (sint32)(Object->AUTOSAR.LONG_DISPLACEMENT - (RequestedDistance)));
    if ((dist >= Distance_s) && (Object->AUTOSAR.REL_LONG_SPEED < Speed_s)) {
        help = (sint32)Object->AUTOSAR.REL_LONG_SPEED *
               (sint32)Object->AUTOSAR.REL_LONG_SPEED;
        help /= Velocity_s;
        help *= Distance_s;
        help /= ((-2) * dist);
        help *= Acceleration_s;
        help /= Velocity_s;
        return_val = (acceleration_t)MAT_LIM(help, Acc_max_allowed_decel,
                                             Acc_max_allowed_accel);
    }
    return return_val;
}

/* ***********************************************************************
  @fn               AVLC_CALC_STOP_DECEL */ /*!
                                 @brief            calculates the needed
                               deceleration to stop at requested distance

                                 @description      calculates the needed
                               deceleration to stop at requested distance

                                 @param            object
                                 @param            time_to_stop
                                 @param            headway_setting
                                 @return           acceleration_t needed
                               deceleration to stop

                                 @pre              Precondition:  none

                                 @post             Postcondition: none...
                               ****************************************************************************
                               */
static acceleration_t AVLC_CALC_STOP_DECEL(const velocity_t host_speed,
                                           const VLC_acc_object_t *const object,
                                           const times_t time_to_stop,
                                           percentage_t headway_setting) {
    acceleration_t out_accel, accel;
    distance_t req_distance, mindist, maxdist;
    static distance_t req_stop_distance;
    velocity_t speed;

    mindist = MAT_CALCULATE_PARAM_VALUE1D(Acc_headway_min_dist,
                                          Acc_headway_min_dist_points, 0);
    maxdist = MAT_CALCULATE_PARAM_VALUE1D(Acc_headway_max_dist,
                                          Acc_headway_max_dist_points, 0);
    req_stop_distance =
        (distance_t)MAT_MAX(
            (((sint32)headway_setting * ((sint32)maxdist - (sint32)mindist)) /
                 (sint32)Percentage_max +
             (sint32)mindist),
            1) *
        MAT_CALCULATE_PARAM_VALUE1D(Acc_request_stop_distance_factor,
                                    Acc_request_stop_distance_factor_points,
                                    host_speed) /
        Scale_100;

    out_accel = 0;

    if (time_to_stop < Fct_tts_max_time) {
        sint32 help1, help2;
        accel = object->LONG_ACCEL_MODIFIED;
        speed = object->LONG_SPEED;

        /*if object speed is <= 0 -> object is stationary!*/
        if (speed <= 0) {
            speed = 0;
            if (accel < 0) {
                accel = 0;
            }
        }

        /*2*a_obj*(s_obj-s_stop)-v_obj^2*/
        help1 = 2 * accel;
        help2 = object->AUTOSAR.LONG_DISPLACEMENT - req_stop_distance;
        /*if current distance is below requested stop distance -> set remaining
         * brake distance to brake distance of the vehicle in front*/
        if (help2 < 0) {
            help2 = 0;
        }
        help1 = MAT_MUL(help1, help2, Acceleration_s, Distance_s, Factor_s);
        help2 = MAT_MUL(speed, speed, Velocity_s, Velocity_s, Factor_s);
        help1 = help1 - help2;

/*correct physics?*/
#if 0
        if (help1 < 0) {
            /*a_obj*v_h^2*/
            help2 = MAT_MUL(object->LONG_SPEED - object->AUTOSAR.REL_LONG_SPEED,
                            object->LONG_SPEED - object->AUTOSAR.REL_LONG_SPEED,
                            Velocity_s, Velocity_s, Factor_s);
            if ((Signed_int32_max / (sint32)Acceleration_s) > help2) {
                help2 =
                    -MAT_DIV(help2, help1, Factor_s, Factor_s, Acceleration_s);
                help2 =
                    MAT_MUL(help2, accel, Factor_s, Acceleration_s, Factor_s);
            } else {
                help2 = -help2 / (MAT_MIN(help1, (sint32)-1));
                help2 = help2 * accel;
            }
            out_accel = (acceleration_t)MAT_LIM(help2, Accel_min, 0);
        } else {
            /*object standstill or other issue*/
            /*-v_h^2/(2*s_obj-s_stop)*/
            help1 = object->AUTOSAR.LONG_DISPLACEMENT - req_stop_distance;
            if (help1 <= Acc_min_allowed_brake_distance) {
                help1 = Acc_min_allowed_brake_distance; /*only 10cm to stop*/
            }
            help2 = MAT_MUL(object->LONG_SPEED - object->AUTOSAR.REL_LONG_SPEED,
                            object->LONG_SPEED - object->AUTOSAR.REL_LONG_SPEED,
                            Velocity_s, Velocity_s, Factor_s);
            help1 = MAT_DIV(-help2, 2 * help1, Factor_s, Distance_s,
                            Acceleration_s);

            /*is object is in standstill - always brake...*/
            out_accel = (acceleration_t)MAT_LIM(
                help1, Accel_min, Acc_minimum_calculated_brake_request);
        }
#else
        mindist = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_headway_min_dist, Acc_headway_min_dist_points, host_speed);
        maxdist = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_headway_max_dist, Acc_headway_max_dist_points, host_speed);
        req_distance =
            (distance_t)MAT_MAX((((sint32)Acc_mid_headwaysetting *
                                  ((sint32)maxdist - (sint32)mindist)) /
                                     (sint32)Percentage_max +
                                 (sint32)mindist),
                                1);

        if (accel < 0 && speed > Acc_object_speed_for_tts_calc) {
            help1 = MAT_MUL(speed, speed, Velocity_s, Velocity_s, Factor_s);
            help2 = MAT_DIV(-help1, 2 * accel, Factor_s, Acceleration_s,
                            Distance_s);
        } else {
            help2 = 0;
        }
        /*object standstill or other issue*/
        /*-v_h^2/(2*(s_obj-s_stop))*/
        help1 = object->AUTOSAR.LONG_DISPLACEMENT - req_stop_distance + help2;
        if (help1 <= Acc_min_allowed_brake_distance) {
            help1 = Acc_min_allowed_brake_distance; /*only 10cm to stop*/
        }
        help2 =
            MAT_MUL(host_speed, host_speed, Velocity_s, Velocity_s, Factor_s);
        help1 =
            MAT_DIV(-help2, 2 * help1, Factor_s, Distance_s, Acceleration_s);

        help1 =
            (MAT_CALCULATE_PARAM_VALUE1D(Acc_tts_decel_fac,
                                         Acc_tts_decel_fac_points, host_speed) *
                 help1 +
             MAT_LIM((object->AUTOSAR.LONG_DISPLACEMENT - req_distance) *
                         Acc_tts_decel_dist_err_fac,
                     -Acc_tts_decel_dist_err_lim, Acc_tts_decel_dist_err_lim)) /
            Scale_100;

        /*if object is in standstill - always brake...*/
        out_accel = (acceleration_t)MAT_LIM(
            help1, Accel_min, Acc_minimum_calculated_brake_request);
#endif

        /*if host vehicle is stationary and close enough -> keep constant
         * deceleration*/
        if (host_speed <= Acc_host_speed_for_stationary_calc &&
            object->AUTOSAR.LONG_DISPLACEMENT <
                Acc_brake_distance_for_stationary) {
            out_accel = Acc_stationary_brake_deceleration;
        }

        if (object->AUTOSAR.LONG_DISPLACEMENT < Acc_emergency_brake_distance) {
            /*if distance is too low -> emergency brake*/
            out_accel = (acceleration_t)MAT_LIM(
                out_accel, Accel_min, Acc_emergency_brake_deceleration);
        }
    }

    return out_accel;
}

/* ***********************************************************************
  @fn               AVLC_CALC_OBJECT_TTS */ /*!
                                 @brief            calculates the time until the
                               object comes to a full stop (with special care to
                               acc)

                                 @description      calculates the time until the
                               object comes to a full stop (with special care to
                               acc)

                                 @param            object

                                 @return           times_t time to stop

                                 @pre              Precondition:  none

                                 @post             Postcondition: none
                               ****************************************************************************
                               */

static times_t AVLC_CALC_OBJECT_TTS(const VLC_acc_object_t *const object) {
    times_t tts;
    tts = Fct_tts_max_time;
    if (object->AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) {
        /*if object standstill is probable */
        if (((object->LONG_ACCEL <
              (acceleration_t)(Acc_object_decel_for_tts_calc)) ||
             (object->LONG_SPEED <
              (velocity_t)Acc_object_speed_for_tts_calc)) &&
            (object->AUTOSAR.LONG_DISPLACEMENT <
             Acc_object_max_distance_for_tts_calc) &&
            (object->AUTOSAR.REL_LONG_SPEED <
             Acc_object_rel_speed_for_tts_calc)) {
            tts = AVLC_DETERMINE_TIME_TO_STOP(object->LONG_ACCEL,
                                              object->LONG_SPEED);
        }
    }
    return tts;
}

/* ***********************************************************************
  @fn               AVLC_GET_ALERT_DISTANCE */ /*!

                              @brief            calculates the alert distance
                            for a specific object speed

                              @description      calculates the alert distance
                            for a specific object speed


                              @param            object

                              @return           distance_t alert distance

                              @pre              Precondition:  none

                              @post             Postcondition: none
                            ****************************************************************************
                            */
distance_t AVLC_GET_ALERT_DISTANCE(const VLC_acc_object_t *object) {
    distance_t alert_dist;

    /* Independent alert distance for intrusion */
    alert_dist = (distance_t)MAT_CALCULATE_PARAM_VALUE1D(
        Acc_max_alert_thres_intrusion, Acc_max_alert_thres_intrusion_points,
        object->LONG_SPEED);

    /*modify alert distance with factor*/
    alert_dist =
        (distance_t)MAT_MUL(alert_dist, object->ALERT_MODIFICATION_FACTOR,
                            Distance_s, Factor_s, Distance_s);

    return alert_dist;
}

/* ***********************************************************************
  @fn               AVLC_DETERMINE_MAX_INTRUSION */ /*!

                         @brief            calculates the maximum allowed
                       intrusion distance for a given alertdistance and a
                       requested distance

                         @description      the maximum allowed intrusion
                       distance is typically between alertdistance and
                       requesteddistance

                         @param[in]        AlertDistance below this distance the
                       acc system will alert

                         @param[in]        RequestedDistance this is the typical
                       follow distance for the given headway setting and the
                       actual speed

                         @param[in]        VehicleSpeed, headway_setting,
                       Relative_Speed

                         @return           max intrusion distance

                         @pre              Precondition:  none

                         @post             Postcondition: none
                       ****************************************************************************
                       */
distance_t AVLC_DETERMINE_MAX_INTRUSION(distance_t AlertDistance,
                                        distance_t RequestedDistance,
                                        velocity_t VehicleSpeed,
                                        percentage_t headway_setting,
                                        velocity_t Relative_Speed,
                                        factor_t modification_factor) {
#define Acc_max_intrusion_inc (distance_t)100 /*1m*/

    sint32 intrusion_distance;
    sint32 help_var, help_var_2;

    factor_t intrusion_factor;

    intrusion_factor =
        (factor_t)(Factor_s -
                   MAT_CALCULATE_PARAM_VALUE1D(Acc_max_intrusion_factor,
                                               Acc_max_intrusion_factor_points,
                                               VehicleSpeed));

    AlertDistance =
        (distance_t)MAT_MIN((sint32)AlertDistance, (sint32)RequestedDistance);

    if (headway_setting == 0) {
        /* Set intrusion for shortest mode dependent on relative velocity to
         * suppress undershooting during approach */
        help_var = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_vrel_intrusion, Acc_vrel_intrusion_points, Relative_Speed);
        /* Calculate difference between no intrusion (factor = 1) and current
         * intrusion and apply v_rel dependent suppression (0-100%) */
        help_var_2 = (Factor_s - intrusion_factor) * help_var;
        /* Add suppression factor to current intrusion value */
        intrusion_factor += (factor_t)(help_var_2 / Factor_s);
    } else {
        /* allow higher intrusion (smaller intrusion_factor) for a longer
         * distance setting (shortest setting stays same) */
        help_var =
            Factor_s + (time_gap_dependency_factor * headway_setting / 100);
        intrusion_factor = (factor_t)(intrusion_factor * Factor_s / help_var);
    }

    intrusion_distance = (sint32)AlertDistance +
                         ((((sint32)RequestedDistance - (sint32)AlertDistance) *
                           intrusion_factor) /
                          Factor_s);
    intrusion_distance = MAT_LIM(intrusion_distance, (sint32)AlertDistance,
                                 (sint32)RequestedDistance);

    _PARAM_UNUSED(modification_factor);
    return (distance_t)intrusion_distance;
}

/* ***********************************************************************
  @fn               AVLC_DETERMINE_TIME_TO_STOP */ /*!
                          @brief            calculates the time until the host
                        vehicle comes to a full stop

                          @description      calculates the time until the host
                        vehicle comes to a full stop

                          @param            acceleration

                          @param            velocity

                          @return           times_t time to stop

                          @pre              Precondition:  none

                          @post             Postcondition: none
                        ****************************************************************************
                        */

times_t AVLC_DETERMINE_TIME_TO_STOP(const acceleration_t acceleration,
                                    const velocity_t velocity) {
    times_t time_to_stop;
    sint32 help_var_tts;

    /*calculate time to stop of host vehicle*/
    if (velocity >= (velocity_t)Acc_object_speed_for_tts_calc) {
        if (acceleration < Acc_object_decel_for_tts_calc) {
            help_var_tts = MAT_MIN(acceleration, -Acc_min_resolution);
            time_to_stop =
                (times_t)MAT_LIM(MAT_DIV(-velocity, help_var_tts, Velocity_s,
                                         Acceleration_s, Time_s),
                                 0, Fct_tts_max_time);
        } else {
            time_to_stop = Fct_tts_max_time;
        }
    } else {
        time_to_stop = 0;
    }

    return time_to_stop;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */