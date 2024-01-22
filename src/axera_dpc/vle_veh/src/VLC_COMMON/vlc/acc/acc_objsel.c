/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "acc.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "mat_fuzzy_ext.h"
#include "phys_kin_ext.h"
#include "acc_par.h"
#include "acc_frules.h"

#define TJA_LANE_CHANGE_START 3
#define TJA_LANE_CHANGE_IN_EGO_LANE 4
#define TJA_LANE_CHANGE_IN_NEW_LANE 5
#define TJA_LANE_CHANGE_END 6
#define TJA_LANE_CHANGE_TO_LEFT 1
#define TJA_LANE_CHANGE_TO_RIGHT 2

static void AVLC_SET_CUTIN_MAX_DECEL(acc_object_ptr_t object);
static void AVLC_ESTIMATE_CONTROL_DISTANCE(
    acc_object_ptr_t object,
    acc_driver_intention_t *driver_intention,
    const acc_input_data_t *input,
    acc_status_t *status,
    uint8 set_standard_values,
    times_t cycle_time,
    OvertakeAssistInfo *p_overtake_assist_info);

/*************************************************************************************************************************
  Functionname:    AVLC_SET_CUTIN_MAX_DECEL */
static void AVLC_SET_CUTIN_MAX_DECEL(acc_object_ptr_t object) {
    object->MAX_ALLOWED_DECEL = (acceleration_t)MAT_MIN(
        object->MAX_ALLOWED_DECEL,
        MAT_CALCULATE_PARAM_VALUE1D(Acc_max_decel_cut_in,
                                    Acc_max_decel_cut_in_points,
                                    (sint16)object->AVLC_CUT_IN_OUT_POTENTIAL));
}

/*************************************************************************************************************************
  Functionname:    AVLC_ESTIMATE_CONTROL_DISTANCE */
static void AVLC_ESTIMATE_CONTROL_DISTANCE(
    acc_object_ptr_t object,
    acc_driver_intention_t *driver_intention,
    const acc_input_data_t *input,
    acc_status_t *status,
    uint8 set_standard_values,
    times_t cycle_time,
    OvertakeAssistInfo *p_overtake_assist_info) {
#define HEADWAY_SMOOTHNESS_MIN 40
#define HEADWAY_SMOOTHNESS_MAX 60
#define Fct_acc_headway_decrese_grad ((distance_t)-10000) /*100m/s*/

    distance_t req_dist;
    factor_t local_alert_mod_factor = Factor_s;
    // factor_t local_alert_mod_factor_lane_change= Factor_s;
    factor_t Fct_acc_headway_add_factor_at_new_obj;
    boolean help_state;

    /* Modify start distance of distance ramp dependent on relative speed */
    Fct_acc_headway_add_factor_at_new_obj =
        (factor_t)MAT_CALCULATE_PARAM_VALUE1D(Acc_headway_add_factor,
                                              Acc_headway_add_factor_points,
                                              object->AUTOSAR.REL_LONG_SPEED);

    /*set smoothness between 40 and 60 depending on headway setting*/
    object->CONTROL_SMOOTHNESS =
        (percentage_t)((sint32)HEADWAY_SMOOTHNESS_MIN +
                       ((sint32)(HEADWAY_SMOOTHNESS_MAX -
                                 HEADWAY_SMOOTHNESS_MIN) *
                        (sint32)input->DRIVER_CONTROLS.HEADWAY_SETTING) /
                           (sint32)Percentage_max);
    object->ALERT_MODIFICATION_FACTOR = (factor_t)Factor_s;

    if ((object->AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) ||
        (object->AUTOSAR.OBJECT_STATUS.MEASURED == TRUE) ||
        (object->AUTOSAR.OBJECT_STATUS.TRACKED == TRUE)) {
        /*if current distance is longer than requested distance do nothing*/
        if (object->AUTOSAR.LONG_DISPLACEMENT >
            status->REQUESTED_DISTANCE_ACT) {
            object->REQUESTED_DISTANCE_MODIFIED_ACT =
                status->REQUESTED_DISTANCE_ACT;
            object->REQUESTED_DISTANCE_MODIFIED_PRED =
                status->REQUESTED_DISTANCE_PREDICTED;
        } else {
            /*if host is in stand still, alway set requested distance to current
             * distance*/
            if (input->LODM_STAT.STANDSTILL == TRUE) {
                object->REQUESTED_DISTANCE_MODIFIED_ACT =
                    object->AUTOSAR.LONG_DISPLACEMENT;
                object->REQUESTED_DISTANCE_MODIFIED_PRED =
                    object->AUTOSAR.LONG_DISPLACEMENT;
            } else {
                /*if new object or system activated, set requested distance to
                 * current distance + 20% of distance error*/
                help_state = (boolean)(SWITCH_RISING_EDGE(
                                           &(driver_intention->AVLC_ENAGAGED),
                                           TRUE) == SWITCH_STATE_ON);
                // if (p_overtake_assist_info->overtake_assist_flag == FALSE &&
                //     p_overtake_assist_info->overtake_assist_flag_last_cycle
                //     ==
                //         TRUE) {
                //     object->REQUESTED_DISTANCE_MODIFIED_ACT =
                //     object->AUTOSAR.LONG_DISPLACEMENT;
                //     object->REQUESTED_DISTANCE_MODIFIED_PRED =
                //     object->AUTOSAR.LONG_DISPLACEMENT;
                // } else
                if ((object->AUTOSAR.OBJECT_STATUS.NEW == TRUE) ||
                    (help_state == TRUE)) {
                    distance_t new_distance;
                    distance_t distance_error;
                    distance_error =
                        (distance_t)(status->REQUESTED_DISTANCE_ACT -
                                     object->AUTOSAR.LONG_DISPLACEMENT);
                    new_distance =
                        (distance_t)(object->AUTOSAR.LONG_DISPLACEMENT +
                                     (distance_t)MAT_MUL(
                                         distance_error,
                                         Fct_acc_headway_add_factor_at_new_obj,
                                         Distance_s, Factor_s, Distance_s));

                    object->REQUESTED_DISTANCE_MODIFIED_ACT = new_distance;
                    object->REQUESTED_DISTANCE_MODIFIED_PRED = new_distance;
                } else {
                    /*host is not in stand still*/
                    static distance_t headway_increse_grad;
                    static factor_t Fct_acc_headway_increase_grad_fac;
                    static times_t time_gap, ttc;

                    if (MAT_DIV(object->AUTOSAR.LONG_DISPLACEMENT, Distance_s,
                                input->LONG_VELOCITY, Velocity_s,
                                Time_s) > Acc_headway_time_thres) {
                        time_gap = Acc_headway_time_thres;
                    } else {
                        time_gap = MAT_DIV(object->AUTOSAR.LONG_DISPLACEMENT,
                                           Distance_s, input->LONG_VELOCITY,
                                           Velocity_s, Time_s);
                    }

                    if (object->AUTOSAR.REL_LONG_SPEED < 0) {
                        if (MAT_DIV(object->AUTOSAR.LONG_DISPLACEMENT,
                                    Distance_s, -object->AUTOSAR.REL_LONG_SPEED,
                                    Velocity_s,
                                    Time_s) > Acc_headway_time_thres) {
                            ttc = Acc_headway_time_thres;
                        } else {
                            ttc = MAT_DIV(object->AUTOSAR.LONG_DISPLACEMENT,
                                          Distance_s,
                                          -object->AUTOSAR.REL_LONG_SPEED,
                                          Velocity_s, Time_s);
                        }
                    } else {
                        ttc = Acc_headway_time_thres;
                    }

                    Fct_acc_headway_increase_grad_fac =
                        MAT_CALCULATE_PARAM_VALUE2D(
                            Acc_headway_increase_grad_fac,
                            Acc_headway_increase_grad_fac_timegap,
                            Acc_headway_increase_grad_fac_ttc,
                            Acc_headway_increase_grad_fac_timegap_points,
                            Acc_headway_increase_grad_fac_ttc_points,
                            (float32)time_gap, (float32)ttc);

                    headway_increse_grad =
                        (distance_t)MAT_MUL(status->REQUESTED_DISTANCE_ACT,
                                            Fct_acc_headway_increase_grad_fac,
                                            Distance_s, Factor_s, Distance_s);

                    /*ramp requested distance for this object to the requested
                     * global distance with a limited positive gradient*/
                    req_dist = (distance_t)MAT_LIM_GRAD(
                        status->REQUESTED_DISTANCE_ACT,
                        object->REQUESTED_DISTANCE_MODIFIED_ACT,
                        Fct_acc_headway_decrese_grad, headway_increse_grad,
                        cycle_time);
                    /*if current distance is bigger - use current*/
                    if (object->AUTOSAR.LONG_DISPLACEMENT > req_dist) {
                        req_dist = object->AUTOSAR.LONG_DISPLACEMENT;
                    }
                    object->REQUESTED_DISTANCE_MODIFIED_ACT = req_dist;
                    /*modify the predicted distance - at the moment use the same
                     * distance*/
                    object->REQUESTED_DISTANCE_MODIFIED_PRED = req_dist;
                    /*also modify factor for alert*/
                }
                local_alert_mod_factor = (factor_t)MAT_LIM(
                    MAT_DIV(object->REQUESTED_DISTANCE_MODIFIED_ACT,
                            status->REQUESTED_DISTANCE_ACT, Distance_s,
                            Distance_s, Factor_s),
                    0, Signed_int16_max);
            }
        }

        /*just check...*/
        if (object->REQUESTED_DISTANCE_MODIFIED_ACT >
            status->REQUESTED_DISTANCE_ACT) {
            object->REQUESTED_DISTANCE_MODIFIED_ACT =
                status->REQUESTED_DISTANCE_ACT;
            object->REQUESTED_DISTANCE_MODIFIED_PRED =
                status->REQUESTED_DISTANCE_PREDICTED;
            local_alert_mod_factor = (factor_t)MAT_LIM(
                MAT_DIV(object->REQUESTED_DISTANCE_MODIFIED_ACT,
                        status->REQUESTED_DISTANCE_ACT, Distance_s, Distance_s,
                        Factor_s),
                0, Signed_int16_max);
        }
    }

    if (set_standard_values == FALSE) {
        object->REQUESTED_DISTANCE_MODIFIED_ACT =
            (distance_t)MAT_MIN(object->REQUESTED_DISTANCE_MODIFIED_ACT,
                                MAT_MUL(status->REQUESTED_DISTANCE_ACT,
                                        object->ALERT_MODIFICATION_FACTOR,
                                        Distance_s, Factor_s, Distance_s));
        object->REQUESTED_DISTANCE_MODIFIED_PRED =
            (distance_t)MAT_MIN(object->REQUESTED_DISTANCE_MODIFIED_PRED,
                                MAT_MUL(status->REQUESTED_DISTANCE_PREDICTED,
                                        object->ALERT_MODIFICATION_FACTOR,
                                        Distance_s, Factor_s, Distance_s));
        if (object->ALERT_MODIFICATION_FACTOR <
            Factor_s) /*if distance if modified to less distance -> use
                         HEADWAY_SMOOTHNESS_MIN as maximum smoothness (for
                         handling long headway setting)*/
        {
            object->CONTROL_SMOOTHNESS = (percentage_t)MAT_MUL(
                HEADWAY_SMOOTHNESS_MIN, object->ALERT_MODIFICATION_FACTOR,
                Percentage_s, Factor_s, Percentage_s);
        }
    }

    object->ALERT_MODIFICATION_FACTOR =
        (factor_t)MAT_MUL(object->ALERT_MODIFICATION_FACTOR,
                          local_alert_mod_factor, Factor_s, Factor_s, Factor_s);

    /*avoid division by zero*/
    object->REQUESTED_DISTANCE_MODIFIED_ACT =
        (distance_t)MAT_MAX(object->REQUESTED_DISTANCE_MODIFIED_ACT, 1);
    object->REQUESTED_DISTANCE_MODIFIED_PRED =
        (distance_t)MAT_MAX(object->REQUESTED_DISTANCE_MODIFIED_PRED, 1);
}

/*************************************************************************************************************************
  Functionname:    AVLC_SELECT_OBJECTS_OF_INTEREST */
void AVLC_SELECT_OBJECTS_OF_INTEREST(const acc_input_data_t *acc_input,
                                     const VLCCustomInput_t *pVLCCustomInput,
                                     acc_driver_intention_t *driver_intention,
                                     acc_object_ptr_t object_list,
                                     acc_status_t *status,
                                     OvertakeAssistInfo *p_overtake_assist_info,
                                     times_t cycle_time) {
    uint8 onr;

    /*object lost --> limit accel/decel*/
    if (acc_input->INPUT_STATUS.OBJECT_LOST == TRUE) {
        object_list[Obj_first_host_lane].MAX_ALLOWED_DECEL =
            (acceleration_t)MAT_MAX(
                (sint32)object_list[Obj_first_host_lane].MAX_ALLOWED_DECEL,
                (sint32)Acc_min_accel_object_lost);
        object_list[Obj_first_host_lane].MAX_ALLOWED_ACCEL =
            (acceleration_t)MAT_MAX(
                (sint32)object_list[Obj_first_host_lane].MAX_ALLOWED_ACCEL,
                (sint32)Acc_max_accel_object_lost);
        /*Keep obj alive to use as relevant*/
        object_list[Obj_first_host_lane].AUTOSAR.OBJECT_STATUS.TRACKED = TRUE;
    }

    /*check for stationary objects*/
    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        if (((object_list[onr].AUTOSAR.OBJECT_STATUS.STANDING == TRUE) &&
                 (object_list[onr].AUTOSAR.OBJECT_STATUS.STOPPED == FALSE) ||
             object_list[onr].LONG_SPEED < Acc_obj_stopped_speed) &&
            (acc_input->LONG_VELOCITY >
             (velocity_t)Acc_decel_on_stationary_speed)) {
            /*if object is stationary and OBJECT_NR_TO_CONTROL_TO flag is not
             * set to this object -> limit max decel to this object*/
            if (driver_intention->OBJECT_NR_TO_CONTROL_TO !=
                object_list[onr].AUTOSAR.OBJECT_ID) {
                object_list[onr].MAX_ALLOWED_DECEL = (acceleration_t)MAT_MAX(
                    (sint32)object_list[onr].MAX_ALLOWED_DECEL,
                    (sint32)Acc_min_accel_object_stationary);
            }
        }
    }

    /*---------------- check decel limit after override ---------------*/

    if (driver_intention->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE == TRUE) {
        if (object_list[Obj_first_host_lane].AUTOSAR.OBJECT_STATUS.DETECTED ==
            TRUE) {
            if ((object_list[Obj_first_host_lane].MAX_ALLOWED_DECEL <
                 acc_input->VLC_DECEL_LIMIT) /*more deceleration to this
                                                object allowed as already
                                                allowed while "after
                                                override"*/
                &&
                ((object_list[Obj_first_host_lane].AUTOSAR.OBJECT_STATUS.NEW ==
                  TRUE) ||
                 (object_list[Obj_first_host_lane].LONG_ACCEL_MODIFIED <
                  acc_input->VLC_DECEL_LIMIT))) {
                driver_intention->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE =
                    FALSE;
                driver_intention->DECEL_LIM_OVERRIDE_ACCEL = Accel_min;
            } else {
                driver_intention->DECEL_LIM_OVERRIDE_ACCEL =
                    (acceleration_t)MAT_MAX(
                        (sint32)acc_input->VLC_DECEL_LIMIT,
                        (sint32)driver_intention->DECEL_LIM_OVERRIDE_ACCEL);
            }
        } else {
            driver_intention->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE = FALSE;
            driver_intention->DECEL_LIM_OVERRIDE_ACCEL = Accel_min;
        }
    } else {
        driver_intention->DECEL_LIM_OVERRIDE_ACCEL = Accel_min;
    }

    /* put decel limiter to each object*/
    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        if (object_list[onr].MAX_ALLOWED_DECEL <
            driver_intention->DECEL_LIM_OVERRIDE_ACCEL) {
            object_list[onr].MAX_ALLOWED_DECEL =
                driver_intention->DECEL_LIM_OVERRIDE_ACCEL;
        }
    }

    /*---------------- select objects -------------------------------*/
    /*the first object in the same lane is always interesting!*/
    object_list[Obj_first_host_lane].USAGE_STATUS.INTEREST = TRUE;
    AVLC_ESTIMATE_CONTROL_DISTANCE(&object_list[Obj_first_host_lane],
                                   driver_intention, acc_input, status, FALSE,
                                   cycle_time, p_overtake_assist_info);

    /*the object in the same lane in front of the first target is also
     * interesting!*/
    object_list[Obj_hidden_host_lane].USAGE_STATUS.INTEREST = TRUE;
    /*allow only limited deceleration - shall be quality dependent in future*/
    /*max allowed deceleration for hidden next object dependent on cut-out
     * probability of the next object in lane*/
    object_list[Obj_hidden_host_lane].MAX_ALLOWED_DECEL =
        (acceleration_t)MAT_MAX(
            (sint32)object_list[Obj_hidden_host_lane].MAX_ALLOWED_DECEL,
            MAT_CALCULATE_PARAM_VALUE1D(Acc_max_decel_hidden_object,
                                        Acc_max_decel_hidden_object_points,
                                        (sint16)object_list[Obj_first_host_lane]
                                            .AVLC_CUT_IN_OUT_POTENTIAL));
    AVLC_ESTIMATE_CONTROL_DISTANCE(&object_list[Obj_hidden_host_lane],
                                   driver_intention, acc_input, status, FALSE,
                                   cycle_time, p_overtake_assist_info);

    /*the object in the adjacent lane is also interesting when lane change!*/
    // sint8 lane_change_obj_nr = -1;
    // if (pVLCCustomInput->TJASLC_ManeuverState_nu ==
    //     TJA_LANE_CHANGE_IN_EGO_LANE) {
    //     if (pVLCCustomInput->TJASLC_LaneChangeTrig_nu ==
    //         TJA_LANE_CHANGE_TO_LEFT) {
    //         lane_change_obj_nr = Obj_first_left_lane;
    //     } else if (pVLCCustomInput->TJASLC_LaneChangeTrig_nu ==
    //                TJA_LANE_CHANGE_TO_RIGHT) {
    //         lane_change_obj_nr = Obj_first_right_lane;
    //     }
    // } else if (pVLCCustomInput->TJASLC_ManeuverState_nu ==
    //            TJA_LANE_CHANGE_IN_NEW_LANE) {
    //     if (pVLCCustomInput->TJASLC_LaneChangeTrig_nu ==
    //         TJA_LANE_CHANGE_TO_LEFT) {
    //         lane_change_obj_nr = Obj_first_right_lane;
    //     } else if (pVLCCustomInput->TJASLC_LaneChangeTrig_nu ==
    //                TJA_LANE_CHANGE_TO_RIGHT) {
    //         lane_change_obj_nr = Obj_first_left_lane;
    //     }
    // }

    // if (lane_change_obj_nr > 0) {
    //     object_list[lane_change_obj_nr].USAGE_STATUS.INTEREST = TRUE;
    // }

    // /*allow only limited deceleration - shall be quality dependent in
    // future*/ object_list[lane_change_obj_nr].MAX_ALLOWED_DECEL =
    // (acceleration_t)MAT_MAX(
    //     (sint32)object_list[lane_change_obj_nr].MAX_ALLOWED_DECEL,
    //     MAT_CALCULATE_PARAM_VALUE1D(Acc_max_decel_adjacent_object,
    //                                 Acc_max_decel_adjacent_object_points,
    //                                 acc_input->LONG_VELOCITY));
    // AVLC_ESTIMATE_CONTROL_DISTANCE(&object_list[lane_change_obj_nr],
    //                                driver_intention, acc_input, status,
    //                                FALSE, cycle_time,
    //                                p_overtake_assist_info);

    /* remove interest stat if max allowed decel >= cc max decel*/
    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        if (object_list[onr].MAX_ALLOWED_DECEL >=
            acc_input->VLC_ACCEL_LIMIT) /*!!! should be the max allowed
                                           acceleration at the moment*/
        {
            object_list[onr].USAGE_STATUS.INTEREST = FALSE;
        }

        if (p_overtake_assist_info->overtake_assist_flag == TRUE) {
            object_list[onr].CONTROL_SMOOTHNESS = 0;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_SELECT_RELEVANT_OBJECT */
void AVLC_SELECT_RELEVANT_OBJECT(const acc_input_data_t *input,
                                 const VLC_DFV2SenInfo_t *pDFVLongOut,
                                 const VLCCustomInput_t *pVLCCustomInput,
                                 acc_object_ptr_t object_list,
                                 VLC_acc_object_t *control_object,
                                 VLC_acc_object_t *alert_object,
                                 VLC_acc_object_t *display_object,
                                 VLC_acc_output_data_t *output,
                                 acc_status_t *status) {
    uint8 obj_nr;
    sint8 relevant_obj_nr_ctrl, relevant_obj_nr_alert;
    acceleration_t min_command_accel_ctrl, min_command_accel_alert;
    acceleration_t ctrl_accel;
    VLC_acc_object_t *object;
    times_t ReactionTime;

    min_command_accel_ctrl = Accel_max;
    min_command_accel_alert = Accel_max;
    relevant_obj_nr_ctrl = -1;
    relevant_obj_nr_alert = -1;
    ReactionTime = Acc_t_reaction_brake;

    output->AVLC_OUTPUT_STATUS.ALLOW_INIT = FALSE;

    if (object_list != NULL) {
        for (obj_nr = 0; obj_nr < Acc_max_number_ooi; obj_nr++) {
            object = &(object_list[obj_nr]);
            if ((object->USAGE_STATUS.INTEREST == TRUE) &&
                ((object->AUTOSAR.OBJECT_STATUS.MEASURED == TRUE) ||
                 (object->AUTOSAR.OBJECT_STATUS.TRACKED == TRUE))) {
                // object->NEEDED_DECEL = PHYS_CALC_NEEDED_DECEL(
                //     input->LONG_ACCELERATION, input->LONG_VELOCITY,
                //     ReactionTime, object->AUTOSAR.REL_LONG_SPEED,
                //     object->AUTOSAR.REL_LONG_ACCEL, object->LONG_SPEED,
                //     object->LONG_ACCEL_MODIFIED,
                //     object->AUTOSAR.LONG_DISPLACEMENT);

                /*calculate needed acceleration if not defined by accel band*/
                if (object->MAX_ALLOWED_ACCEL != object->MAX_ALLOWED_DECEL) {
                    object->CONTROL_ACCEL = AVLC_DETERMINE_DIST_CONTROL_ACCEL(
                        object, pDFVLongOut->MovingTime,
                        input->DRIVER_CONTROLS.HEADWAY_SETTING,
                        input->LONG_ACCELERATION, input->LONG_VELOCITY);
                } else {
                    object->CONTROL_ACCEL = object->MAX_ALLOWED_ACCEL;
                }

                /*control object - use smallest limited acceleration*/
                ctrl_accel =
                    (acceleration_t)MAT_LIM((sint32)object->CONTROL_ACCEL,
                                            (sint32)object->MAX_ALLOWED_DECEL,
                                            (sint32)object->MAX_ALLOWED_ACCEL);
                if (ctrl_accel < min_command_accel_ctrl) {
                    min_command_accel_ctrl = ctrl_accel;
                    relevant_obj_nr_ctrl = (sint8)obj_nr;
                }

                /*alert object - use smallest overall acceleration*/
                if ((object->CONTROL_ACCEL < min_command_accel_alert) &&
                    (obj_nr == Obj_first_host_lane)) {
                    min_command_accel_alert = object->CONTROL_ACCEL;
                    relevant_obj_nr_alert = (sint8)obj_nr;
                }
            } else {
                object->CONTROL_ACCEL = Accel_max;
            }
        }
    }

    if (relevant_obj_nr_ctrl >= 0) {
        object_list[relevant_obj_nr_ctrl].USAGE_STATUS.USE_FOR_CONTROL = TRUE;

        if ((relevant_obj_nr_ctrl == Obj_first_host_lane) &&
            (control_object->AUTOSAR.OBJECT_ID !=
             object_list[relevant_obj_nr_ctrl].AUTOSAR.OBJECT_ID)) {
            output->AVLC_OUTPUT_STATUS.ALLOW_INIT = TRUE;
        }

        *control_object = (object_list[relevant_obj_nr_ctrl]);

        /*if object control distance was modified -> always allow init*/
        if (control_object->REQUESTED_DISTANCE_MODIFIED_ACT !=
            status->REQUESTED_DISTANCE_ACT) {
            output->AVLC_OUTPUT_STATUS.ALLOW_INIT = TRUE;
        }
    } else {
        if ((relevant_obj_nr_ctrl == Obj_first_host_lane) &&
            (control_object->AUTOSAR.OBJECT_ID !=
             AVLC_NO_OBJECT.AUTOSAR.OBJECT_ID)) {
            output->AVLC_OUTPUT_STATUS.ALLOW_INIT = TRUE;
        }

        *control_object = AVLC_NO_OBJECT;
    }

    if (pVLCCustomInput->TJASLC_ManeuverState_nu >=
            TJA_LANE_CHANGE_IN_EGO_LANE &&
        pVLCCustomInput->TJASLC_ManeuverState_nu <=
            TJA_LANE_CHANGE_IN_NEW_LANE) {
        control_object->MAX_ALLOWED_ACCEL = 0;
    }

    if (relevant_obj_nr_alert >= 0) {
        object_list[relevant_obj_nr_alert].USAGE_STATUS.USE_FOR_ALERT = TRUE;
        *alert_object = (object_list[relevant_obj_nr_alert]);
    } else {
        *alert_object = AVLC_NO_OBJECT;
    }

    /*if object in own lane available, use this object as display object
     * otherwise use control object*/
    /*use control object*/
    *display_object = *control_object;

    if (MAT_LIM((sint32)display_object->CONTROL_ACCEL,
                (sint32)display_object->MAX_ALLOWED_DECEL,
                (sint32)display_object->MAX_ALLOWED_ACCEL) >=
        input->VLC_ACCEL_LIMIT) {
        *display_object = AVLC_NO_OBJECT; /*do not show objects from adjacent
                                             lanes if they are not used for
                                             control*/
    }

    // if (&(object_list[0]) != 0 &&
    //     (object_list[0].LANE_INFORMATION == Obj_lane_same) &&
    //     (object_list[0].USAGE_STATUS.INTEREST == TRUE) &&
    //     ((object_list[0].AUTOSAR.OBJECT_STATUS.MEASURED == TRUE) ||
    //      (object_list[0].AUTOSAR.OBJECT_STATUS.TRACKED == TRUE))) {
    //     /*use first object in lane*/
    //     *display_object = object_list[0];
    // }

    if (output->AVLC_OUTPUT_STATUS.INHIBITED == TRUE) {
        *display_object =
            AVLC_NO_OBJECT; /*do not show objects if acc is inhibited*/
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_DELETE_OBJECT */
void AVLC_DELETE_OBJECT(VLC_acc_object_t *const object) {
    if (object) {
        object->AUTOSAR.QUALITY = 0;
        object->AUTOSAR.OBJECT_STATUS.MEASURED = FALSE;
        object->AUTOSAR.OBJECT_STATUS.TRACKED = FALSE;
        object->AUTOSAR.OBJECT_STATUS.NEW = FALSE;
        object->AUTOSAR.OBJECT_STATUS.STANDING = FALSE;
        object->AUTOSAR.OBJECT_STATUS.STOPPED = FALSE;
        object->AUTOSAR.OBJECT_STATUS.MOVING = FALSE;
        object->AUTOSAR.OBJECT_STATUS.DETECTED = FALSE;
        object->LONG_SPEED = (velocity_t)0;
        object->LONG_ACCEL = (acceleration_t)0;
        object->AUTOSAR.REL_LONG_ACCEL = (acceleration_t)0;
        object->AUTOSAR.REL_LAT_ACCEL = (acceleration_t)0;
        object->AUTOSAR.LONG_DISPLACEMENT = (distance_t)Distance_max;
        object->AUTOSAR.LAT_DISPLACEMENT = (distance_t)0;
        object->AUTOSAR.REL_LONG_SPEED = (velocity_t)0;
        object->AUTOSAR.REL_LAT_SPEED = (velocity_t)0;
        object->CONTROL_ACCEL = (acceleration_t)Acc_max_allowed_accel;
        object->MAX_ALLOWED_ACCEL = (acceleration_t)Acc_max_allowed_accel;
        object->MAX_ALLOWED_DECEL = (acceleration_t)Acc_max_allowed_decel;
        object->LAT_DISPL_FROM_LANE = (distance_t)0;
        object->LANE_INFORMATION = Obj_lane_same;
        object->AUTOSAR.OBJECT_ID = OBJ_INDEX_NO_OBJECT;
        object->AUTOSAR.WIDTH = 0;
        object->AVLC_CUT_IN_OUT_POTENTIAL = (quality_t)0;
        object->CONTROL_SMOOTHNESS = (percentage_t)0;
        /* do not set  object->REQUESTED_DISTANCE_MODIFIED_ACT and
         * object->REQUESTED_DISTANCE_MODIFIED_PRED  here */
        object->ALERT_MODIFICATION_FACTOR = (factor_t)1000;
        /*do not set object->LONG_ACCEL_MODIFIED*/
        /* do not set object->LAST_OBJECT_ID = OBJ_INDEX_NO_OBJECT; */
        object->TTS = 0;
        object->ACCEL_REQUEST_FUZZY = 0;
        object->ACCEL_REQUEST_TTS = 0;
        object->ACCEL_REQUEST_DMIN = 0;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */