/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "acc.h"
#include "acc_par.h"

#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "phys_kin_ext.h"

static confidence_t AVLC_ESTIMATE_CONTROL_CRITICALITY(
    acc_object_ptr_t object_list, VLC_acc_output_data_t *output);

/*************************************************************************************************************************
  Functionname:    AVLC_ESTIMATE_CONTROL_CRITICALITY */
static confidence_t AVLC_ESTIMATE_CONTROL_CRITICALITY(
    acc_object_ptr_t object_list, VLC_acc_output_data_t *output) {
    uint32 criticality_control, criticality_dist, criticality_obj_decel,
        criticality_ttc;
    velocity_t impact_velocity;
    VLC_acc_object_t *object;
    uint32 ttc;

    criticality_control = 0;
    criticality_ttc = 0;
    criticality_dist = 0;
    // criticality_obj_decel = 0;

    /* Use only OOI 0 */
    object = &(object_list[0]);

    /* Calculate TTC with a_rel */
    ttc = (uint32)MAT_MIN(
        PHYS_CALCULATE_TTC(
            object->LONG_SPEED, object->LONG_ACCEL,
            (velocity_t)((sint32)object->LONG_SPEED -
                         (sint32)object->AUTOSAR.REL_LONG_SPEED),
            (acceleration_t)((sint32)object->LONG_ACCEL -
                             (sint32)object->AUTOSAR.REL_LONG_ACCEL),
            object->AUTOSAR.LONG_DISPLACEMENT, &impact_velocity),
        Acc_si_criticality_from_ttc[2 * (Acc_si_crit_from_ttc_points - 1)]);
    object->TTC = (times_t)(ttc);

    /*****************************************************/
    /* Criticality based on relevant object deceleration */
    /*****************************************************/

    criticality_obj_decel = (uint32)(MAT_CALCULATE_PARAM_VALUE1D(
        Acc_si_crit_factor_obj_accel, Acc_si_crit_factor_obj_accel_points,
        (sint16)object->LONG_ACCEL));

    /* Enhance criticality for short headway distance */

    if (output->HEADWAY_SETTING == 0) {
        criticality_obj_decel *= (uint32)Acc_crit_factor_gain;
        criticality_obj_decel /= Factor_s;
        criticality_obj_decel =
            (uint32)MAT_LIM((sint32)criticality_obj_decel, 0, Factor_s);
    }

    /*************************************************/
    /* Criticality based on relevant object distance */
    /*************************************************/

    if ((object->AUTOSAR.LONG_DISPLACEMENT <= output->REQUESTED_DISTANCE) &&
        (output->REQUESTED_MAX_INTRUSION < output->REQUESTED_DISTANCE)) {
        if (object->AUTOSAR.LONG_DISPLACEMENT >=
            output->REQUESTED_MAX_INTRUSION) {
            criticality_dist =
                (uint32)(Factor_s * (object->AUTOSAR.LONG_DISPLACEMENT -
                                     output->REQUESTED_MAX_INTRUSION));
            if ((output->REQUESTED_DISTANCE -
                 output->REQUESTED_MAX_INTRUSION) != 0) /* div by 0 check ! */
            {
                criticality_dist /= (uint32)((output->REQUESTED_DISTANCE -
                                              output->REQUESTED_MAX_INTRUSION));
                criticality_dist =
                    Factor_s -
                    (uint32)(MAT_LIM((sint32)criticality_dist, 0, Factor_s));
            }
        } else {
            /* distance closer than intrusion => max distance criticality */
            criticality_dist = Factor_s;
        }
    } else {
        /* distance longer than requested distance => no distance criticality */
        criticality_dist = 0;
    }

    /****************************/
    /* Criticality based on TTC */
    /****************************/

    if (object->AUTOSAR.REL_LONG_SPEED < 0) {
        /* simple ttc estimation without a_rel */
        ttc = (uint32)(object->AUTOSAR.LONG_DISPLACEMENT * Factor_s /
                       MAT_ABS(object->AUTOSAR.REL_LONG_SPEED));
        ttc = (uint32)(MAT_LIM((sint32)ttc, 0, Scale_10000));
    } else {
        ttc = Scale_10000; /* 10 sec (default value) */
    }

    if ((ttc < object->TTC)) {
        /* object->TTC calculated with a_rel -> not usefull if a_rel->0 */
        object->TTC = (times_t)(ttc);
    }

    criticality_ttc = (uint32)MAT_CALCULATE_PARAM_VALUE1D(
        Acc_si_criticality_from_ttc, Acc_si_crit_from_ttc_points,
        (sint16)object->TTC);

    /**********************************/
    /* Calculate combined criticality */
    /**********************************/

    if (object->TTC < Scale_10000) {
        /* Mean value of obj_decel and ttc criticality*/
        criticality_control = (criticality_obj_decel + criticality_ttc) / 2;

        /* Distance criticality can amplify the overall criticality */
        if ((criticality_dist > 0) && (criticality_obj_decel > 0) &&
            (criticality_ttc > 0)) {
            criticality_control *= (Factor_s + criticality_dist);
            criticality_control /= Factor_s;
        }
    }

    /* Limit criticality result 0..100% */
    criticality_control =
        (uint32)MAT_LIM((sint32)criticality_control, 0, Factor_s);

    /* Adapt to confidence scale 0..255 */
    criticality_control = criticality_control * Confidence_s / Scale_1000;

    return (confidence_t)(criticality_control);
}

/*************************************************************************************************************************
  Functionname:    AVLC_ESTIMATE_CRITICALITY */
static void AVLC_ESTIMATE_CRITICALITY(const times_t cycle_time,
                                      acc_object_ptr_t object_list,
                                      VLC_acc_output_data_t *output) {
    confidence_t max_criticality;
    max_criticality = 0;

    /* Calculate object criticality focused on longitudinal control usage */
    max_criticality = AVLC_ESTIMATE_CONTROL_CRITICALITY(object_list, output);

    /* Write output */
    output->SITUATION_CLASS.CRITICALITY = (confidence_t)MAT_LIM_GRAD(
        max_criticality, output->SITUATION_CLASS.CRITICALITY,
        Acc_si_crit_neg_grad, Acc_si_crit_pos_grad, cycle_time);
}

void AVLC_OVERTAKE_SITUATION(const acc_input_data_t *input,
                             VLC_acc_output_data_t *output,
                             acc_driver_intention_t *driver_intention) {
    if (input->LODM_STAT.OVERRIDE_ACCEL == TRUE &&
        ((driver_intention->LANE_CHANGE_LEFT_PROBABILITY >
          Acc_si_lcprob_ovrtk) ||
         (driver_intention->LANE_CHANGE_RIGHT_PROBABILITY >
          Acc_si_lcprob_ovrtk))) {
        output->SITUATION_CLASS.SITUATION = Acc_sit_class_overtake;
    }
}
/*************************************************************************************************************************
  Functionname:    AVLC_ESTIMATE_TRAFFIC_SITUATION */
void AVLC_ESTIMATE_TRAFFIC_SITUATION(const times_t cycle_time,
                                     acc_object_ptr_t object_list,
                                     const acc_input_data_t *input,
                                     VLC_acc_object_t *control_object,
                                     VLC_acc_output_data_t *output,
                                     acc_driver_intention_t *driver_intention) {
    sint32 time_to_stop;
    acc_situation_class_t situation_class_last_cycle;

    situation_class_last_cycle = output->SITUATION_CLASS.SITUATION;
    output->SITUATION_CLASS.SITUATION = Acc_sit_class_undefined;

    if ((control_object->AUTOSAR.OBJECT_STATUS.DETECTED == FALSE) ||
        (input->INPUT_STATUS.OBJECT_EFFECTIVE == FALSE)) {
        output->SITUATION_CLASS.SITUATION = Acc_sit_class_freemode;
    } else {
        /*effective object available*/
        if (input->LONG_VELOCITY > Acc_crawl_max_velocity) {
            output->SITUATION_CLASS.SITUATION = Acc_sit_class_follow;
        } else {
            if (control_object->LONG_SPEED > Acc_crawl_max_velocity) {
                output->SITUATION_CLASS.SITUATION = Acc_sit_class_follow;
            } else {
                /*situation can be "crawl" only if both, the target and the host
                 * are slower than Acc_si_crawl_max_speed*/
                if (situation_class_last_cycle != Acc_sit_class_stop) {
                    output->SITUATION_CLASS.SITUATION = Acc_sit_class_crawl;
                } else {
                    if (control_object->CONTROL_ACCEL >
                        Acc_min_obj_crawl_accel) {
                        /* go from stopped in crawl with hysteresis
                         * (Acc_min_obj_crawl_accel-Acc_min_obj_stop_accel) */
                        output->SITUATION_CLASS.SITUATION = Acc_sit_class_crawl;
                    } else {
                        /* Keep situation stop with hysteresis
                         * (Acc_min_obj_crawl_accel-Acc_min_obj_stop_accel) */
                        output->SITUATION_CLASS.SITUATION = Acc_sit_class_stop;
                    }
                }
            }
        }

        /*calculate time to get the target to a full stop*/
        time_to_stop = AVLC_DETERMINE_TIME_TO_STOP(
            control_object->LONG_ACCEL_MODIFIED, control_object->LONG_SPEED);

        /* test for Stop situation*/
        if (control_object->CONTROL_ACCEL < Acc_min_obj_stop_accel) {
            if ((situation_class_last_cycle == Acc_sit_class_stop) &&
                ((time_to_stop < (sint32)Acc_si_max_hys_time_to_stop) ||
                 /* Additional check of EM dynamic property to keep the
                    "Acc_sit_class_stop" more stable */
                 (control_object->AUTOSAR.OBJECT_STATUS.STOPPED == TRUE ||
                  control_object->AUTOSAR.OBJECT_STATUS.STANDING == TRUE))) {
                output->SITUATION_CLASS.SITUATION = Acc_sit_class_stop;
            } else {
                if ((time_to_stop < (sint32)Acc_si_min_hys_time_to_stop) &&
                    (control_object->AUTOSAR.LONG_DISPLACEMENT <=
                     (distance_t)Acc_max_stop_distance)) {
                    output->SITUATION_CLASS.SITUATION = Acc_sit_class_stop;
                } else {
                    if ((time_to_stop < (sint32)Acc_si_max_time_to_stop_2obj) &&
                        (object_list[Obj_hidden_host_lane]
                             .USAGE_STATUS.INTEREST == TRUE) &&
                        (object_list[Obj_hidden_host_lane]
                             .AUTOSAR.OBJECT_STATUS.STOPPED == TRUE)) {
                        output->SITUATION_CLASS.SITUATION = Acc_sit_class_stop;
                    }
                }
            }
        } else {
            /*stay in crawl*/
        }

        if ((control_object->CONTROL_ACCEL < Acc_min_obj_stop_accel) &&
            (input->LODM_STAT.STANDSTILL == TRUE) &&
            /* check ego velocity in case standstill flag is released too late
               (prevent brake hold reaction during drive away) */
            (input->LONG_VELOCITY < Acc_stopped_speed)) {
            output->SITUATION_CLASS.SITUATION = Acc_sit_class_stop;
        } else {
            /*stay in crawl*/
        }

        /* test if GO situation is active*/
        if (output->SITUATION_CLASS.SITUATION == Acc_sit_class_crawl &&
            control_object->LONG_SPEED > input->LONG_VELOCITY) {
            if (control_object->LONG_ACCEL_MODIFIED > Acc_si_min_go_accel) {
                output->SITUATION_CLASS.SITUATION = Acc_sit_class_go;
            } else {
                if ((control_object->LONG_ACCEL_MODIFIED >
                     Acc_si_min_go_accel_2obj) &&
                    (object_list[Obj_hidden_host_lane].USAGE_STATUS.INTEREST ==
                     TRUE) &&
                    (object_list[Obj_hidden_host_lane].LONG_ACCEL_MODIFIED >
                     Acc_si_min_go_accel_2obj)) {
                    output->SITUATION_CLASS.SITUATION = Acc_sit_class_go;
                }
            }
        }
    }

    /*overtake situation*/
    AVLC_OVERTAKE_SITUATION(input, output, driver_intention);

    AVLC_ESTIMATE_CRITICALITY(cycle_time, object_list, output);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */