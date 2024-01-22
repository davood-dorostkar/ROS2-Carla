/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "overtake_assist.h"
#include "acc.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "mat_fuzzy_ext.h"
#include "acc_par.h"
#include "acc_lib.h"
#include "sen_sim.h"

static void AVLC_OVERTAKE_ASSIST_PROCESS(
    const times_t cycle_time,
    const VLCCustomInput_t *pVLCCustomInput,
    const acc_status_t *acc_status,
    const VLC_acc_object_t object_list[Acc_max_number_ooi],
    acc_input_data_t *input,
    OvertakeAssistInfo *p_overtake_assist_info);
static void AVLC_ASSEMBLE_OUTPUT_DATA(const acc_input_data_t *input,
                                      const VLC_acc_object_t *control_object,
                                      VLC_acc_output_data_t *output);
static void AVLC_DETERMINE_CONTROL_DISTANCE(const acc_input_data_t *input,
                                            acc_status_t *status,
                                            VLC_acc_output_data_t *output);
static void AVLC_MODIFY_OBJECT_PROPERTIES(const times_t cycle_time,
                                          acc_object_ptr_t object_list);
static void AVLC_MODIFY_OBJECT_PROPERTIES_LAST_CYCLE(
    acc_object_ptr_t object_list);
static void AVLC_LIMIT_OBJ_ACCEL_NEG_GRAD(const times_t cycle_time,
                                          VLC_acc_object_t *object);

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLC_acc_object_t AVLC_NO_OBJECT;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// OvertakeAssistInfo overtake_assist_info;

/*************************************************************************************************************************
  Functionname:    AVLC_OVERTAKE_ASSIST_PROCESS */
static void AVLC_OVERTAKE_ASSIST_PROCESS(
    const times_t cycle_time,
    const VLCCustomInput_t *pVLCCustomInput,
    const acc_status_t *acc_status,
    const VLC_acc_object_t object_list[Acc_max_number_ooi],
    acc_input_data_t *input,
    OvertakeAssistInfo *p_overtake_assist_info) {
    OvertakeAssistProcess(cycle_time, pVLCCustomInput->eTurnIndicator,
                          (uint8)(pVLCCustomInput->speedometer_speed),
                          (float32)(input->LONG_VELOCITY) / 100,
                          (float32)(object_list[0].LONG_SPEED) / 100,
                          object_list[0].AUTOSAR.OBJECT_ID,
                          input->DRIVER_CONTROLS.HEADWAY_SETTING,
                          p_overtake_assist_info);

    input->DRIVER_CONTROLS.HEADWAY_SETTING =
        p_overtake_assist_info->headway_setting;
}

/*************************************************************************************************************************
  Functionname:    AVLC_DETERMINE_RECOMMENDED_VELOCITY */
static void AVLC_DETERMINE_RECOMMENDED_VELOCITY(const acc_input_data_t *input,
                                                VLC_acc_output_data_t *output) {
    output->RECOMMENDED_VELOCITY = (velocity_t)MAT_CALCULATE_PARAM_VALUE1D(
        Acc_recommended_velocity_curve, Acc_recommended_velocity_curve_points,
        input->VISIBILITY_RANGE);
}

/*************************************************************************************************************************
  Functionname:    AVLC_CHECK_FOR_INHIBITION */
static void AVLC_CHECK_FOR_INHIBITION(const acc_input_data_t *input,
                                      VLC_acc_output_data_t *output) {
    sint32 visibility;

    /* visibility [s] = range [m] / velocity [m/s] */
    if (input->LONG_VELOCITY > (velocity_t)0) {
        visibility = (sint32)input->VISIBILITY_RANGE;
        visibility *= (sint32)Time_s;
        visibility /= (sint32)Distance_s;
        visibility *= (sint32)Velocity_s;
        visibility /= (sint32)input->LONG_VELOCITY;
        visibility = MAT_LIM(visibility, (sint32)0, (sint32)Time_max);
    } else {
        visibility = Time_max;
    }

    if ((visibility <= Acc_t_visibility_disengage) ||
        (input->INPUT_STATUS.INHIBIT == TRUE)) {
        output->AVLC_OUTPUT_STATUS.INHIBITED = TRUE;
    } else {
        if (visibility >= Acc_t_visibility_engage) {
            output->AVLC_OUTPUT_STATUS.INHIBITED = FALSE;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_ASSEMBLE_OUTPUT_DATA */
static void AVLC_ASSEMBLE_OUTPUT_DATA(const acc_input_data_t *input,
                                      const VLC_acc_object_t *control_object,
                                      VLC_acc_output_data_t *output) {
    velocity_t host_predicted_velocity, host_velocity;
    acceleration_t host_acceleration;
    sint32 help_var, time_to_stop, max_pred_time;

    /*build some mts data*/

    host_acceleration = input->LONG_ACCELERATION;
    host_velocity = input->LONG_VELOCITY;

    /*convert headway setting from 0-100% to 1-2s (*1000)*/
    output->REQUESTED_TIMEGAP =
        (times_t)(((uint32)input->DRIVER_CONTROLS.HEADWAY_SETTING *
                   (sint32)Time_s) /
                      (uint32)Percentage_max +
                  (uint32)Time_s);
    /*calculate time to stop of host vehicle*/

    time_to_stop =
        AVLC_DETERMINE_TIME_TO_STOP(host_acceleration, host_velocity);

    max_pred_time = MAT_MIN((sint32)Acc_predicted_reaction_time, time_to_stop);

    /*Predict vehicle velocity after reaction time */
    /*It is assumed that the host vehicle has a PT1 behaviour*/
    help_var = (sint32)host_acceleration * (sint32)max_pred_time;
    help_var /= (sint32)Acceleration_s;
    help_var *= (sint32)Velocity_s;
    help_var /= (sint32)Time_s;
    host_predicted_velocity = (velocity_t)MAT_LIM(
        ((sint32)host_velocity + help_var), (sint32)0, (sint32)Speed_max);

    output->MAX_AVLC_ACCELERATION = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_fsr_max_acceleration, (uint16)Acc_fsr_max_acceleration_points,
        host_predicted_velocity);
    output->MAX_AVLC_DECELERATION = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_fsr_max_deceleration, (uint16)Acc_fsr_max_deceleration_points,
        host_predicted_velocity);

    if (output->AVLC_OUTPUT_STATUS.INHIBITED == FALSE) {
        output->REQUESTED_MAX_INTRUSION = AVLC_DETERMINE_MAX_INTRUSION(
            AVLC_GET_ALERT_DISTANCE(control_object),
            control_object->REQUESTED_DISTANCE_MODIFIED_PRED,
            host_predicted_velocity, input->DRIVER_CONTROLS.HEADWAY_SETTING,
            control_object->AUTOSAR.REL_LONG_SPEED,
            control_object->ALERT_MODIFICATION_FACTOR);

        /*calculate max acceleration*/
        output->DISTANCE_CTRL_ACCEL_MAX =
            (acceleration_t)MAT_LIM(control_object->CONTROL_ACCEL,
                                    (sint32)control_object->MAX_ALLOWED_DECEL,
                                    (sint32)control_object->MAX_ALLOWED_ACCEL);

        output->DISTANCE_CTRL_ACCEL_MIN = output->DISTANCE_CTRL_ACCEL_MAX;
    } else {
        /*ACC is inhibited - so use standard values*/
        output->REQUESTED_MAX_INTRUSION = (distance_t)20000;

        /*calculate max acceleration*/
        output->DISTANCE_CTRL_ACCEL_MAX = (acceleration_t)Acc_max_allowed_accel;
        output->DISTANCE_CTRL_ACCEL_MIN = output->DISTANCE_CTRL_ACCEL_MAX;
    }
    output->HEADWAY_SETTING = input->DRIVER_CONTROLS.HEADWAY_SETTING;
}

/*************************************************************************************************************************
  Functionname:    AVLC_INIT_DRIVER_INTENTION */
static void AVLC_INIT_DRIVER_INTENTION(
    acc_driver_intention_t *driver_intension) {
    driver_intension->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE = FALSE;
    driver_intension->DECEL_LIM_OVERRIDE.VLC_LIMITER_ACTIVE = FALSE;
    driver_intension->DECEL_LIM_OVERRIDE_ACCEL = Accel_min;
    driver_intension->LANE_CHANGE_LEFT_PROBABILITY = 0;
    driver_intension->LANE_CHANGE_RIGHT_PROBABILITY = 0;
    SWITCH_INIT_SWITCH(&(driver_intension->AVLC_ENAGAGED));
    driver_intension->OBJECT_NR_TO_CONTROL_TO = OBJ_INDEX_NO_OBJECT;
}

/*************************************************************************************************************************
  Functionname:    AVLC_INIT */
void AVLC_INIT(VLC_acc_object_t *display_object,
               VLC_acc_object_t *alert_object,
               acc_status_t *acc_status) {
    AVLC_DELETE_OBJECT(&AVLC_NO_OBJECT);
    *display_object = AVLC_NO_OBJECT;
    *alert_object = AVLC_NO_OBJECT;
    AVLC_ALERT_INIT(&acc_status->AVLC_ALERT_DATA);
    AVLC_INIT_DRIVER_INTENTION(&acc_status->AVLC_DRIVER_INTENTION);
}

/*************************************************************************************************************************
  Functionname:    AVLC_DETERMINE_CONTROL_DISTANCE */
static void AVLC_DETERMINE_CONTROL_DISTANCE(const acc_input_data_t *input,
                                            acc_status_t *status,
                                            VLC_acc_output_data_t *output) {
    distance_t mindist, maxdist;

    times_t max_pred_time, time_to_stop;
    sint32 help_var;
    velocity_t host_velocity;
    acceleration_t host_acceleration;

    host_velocity = input->LONG_VELOCITY;
    host_acceleration = input->LONG_ACCELERATION;

#if VLC_LONG_SEN_DEBUG == 1
    global_host_velocity1 = host_velocity;
    global_host_acceleration = host_acceleration;
#endif

    /*calculate time to stop of host vehicle*/
    time_to_stop =
        AVLC_DETERMINE_TIME_TO_STOP(host_acceleration, host_velocity);

    max_pred_time =
        (times_t)MAT_MIN((sint32)Acc_predicted_reaction_time, time_to_stop);

    /*Predict vehicle velocity after prediction time */
    /*It is assumed that the host vehicle has a PT1 behaviour*/
    help_var = (sint32)host_acceleration * (sint32)max_pred_time;
    help_var /= (sint32)Acceleration_s;
    help_var *= (sint32)Velocity_s;
    help_var /= (sint32)Time_s;

    mindist = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_headway_min_dist, Acc_headway_min_dist_points, host_velocity);
    maxdist = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_headway_max_dist, Acc_headway_max_dist_points, host_velocity);
    status->REQUESTED_DISTANCE_ACT =
        (distance_t)MAT_MAX((((sint32)input->DRIVER_CONTROLS.HEADWAY_SETTING *
                              ((sint32)maxdist - (sint32)mindist)) /
                                 (sint32)Percentage_max +
                             (sint32)mindist),
                            1);
    output->REQUESTED_DISTANCE = status->REQUESTED_DISTANCE_ACT;

    host_velocity = (velocity_t)(host_velocity + help_var);
    host_velocity = (velocity_t)MAT_LIM((sint32)host_velocity, (sint32)0,
                                        (sint32)Speed_max);
#if VLC_LONG_SEN_DEBUG == 1
    global_host_velocity2 = host_velocity;
#endif

    mindist = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_headway_min_dist, Acc_headway_min_dist_points, host_velocity);
    maxdist = MAT_CALCULATE_PARAM_VALUE1D(
        Acc_headway_max_dist, Acc_headway_max_dist_points, host_velocity);
    status->REQUESTED_DISTANCE_PREDICTED =
        (distance_t)MAT_MAX((((sint32)input->DRIVER_CONTROLS.HEADWAY_SETTING *
                              ((sint32)maxdist - (sint32)mindist)) /
                                 (sint32)Percentage_max +
                             (sint32)mindist),
                            1);
}

/*************************************************************************************************************************
  Functionname:    AVLC_LIMIT_OBJ_ACCEL_NEG_GRAD */
static void AVLC_LIMIT_OBJ_ACCEL_NEG_GRAD(const times_t cycle_time,
                                          VLC_acc_object_t *object) {
    gradient_t max_neg_grad;

    if (object->LONG_ACCEL < object->LONG_ACCEL_MODIFIED) {
        /*neg grad limiter needs to be used*/
        max_neg_grad = (gradient_t)MAT_CALCULATE_PARAM_VALUE1D(
            Acc_obj_accel_max_neg_grad, Acc_obj_accel_max_neg_grad_points,
            (sint16)object->LONG_ACCEL);
        max_neg_grad = MAT_MUL(
            max_neg_grad,
            MAT_CALCULATE_PARAM_VALUE1D(Acc_obj_accel_grad_speed_fac,
                                        Acc_obj_accel_grad_speed_fac_points,
                                        object->LONG_SPEED),
            Acceleration_s, Factor_s, Acceleration_s);
        max_neg_grad =
            MAT_MUL(max_neg_grad,
                    MAT_CALCULATE_PARAM_VALUE1D(
                        Acc_obj_accel_grad_accel_d_fac,
                        Acc_obj_accel_grad_accel_d_fac_pt,
                        object->LONG_ACCEL - object->LONG_ACCEL_MODIFIED),
                    Acceleration_s, Factor_s, Acceleration_s);
        object->LONG_ACCEL_MODIFIED = (acceleration_t)MAT_LIM_GRAD(
            object->LONG_ACCEL, object->LONG_ACCEL_MODIFIED, max_neg_grad, 5000,
            cycle_time);
    } else {
        /*just use the current acceleration*/
        object->LONG_ACCEL_MODIFIED = object->LONG_ACCEL;
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_MODIFY_OBJECT_PROPERTIES */
static void AVLC_MODIFY_OBJECT_PROPERTIES(const times_t cycle_time,
                                          acc_object_ptr_t object_list) {
    uint8 onr;

    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        if (object_list[onr].AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) {
            if (object_list[onr].AUTOSAR.OBJECT_ID ==
                object_list[onr].LAST_OBJECT_ID) {
                /*same object as in last cycle --> accel filter*/
                switch (onr) {
                    case Obj_first_host_lane:
                    case Obj_hidden_host_lane:
                        AVLC_LIMIT_OBJ_ACCEL_NEG_GRAD(cycle_time,
                                                      &(object_list[onr]));
                        break;

                    default:
                        object_list[onr].LONG_ACCEL_MODIFIED =
                            object_list[onr].LONG_ACCEL;
                        break;
                }
            } else {
                /* set LONG_ACCEL_MODIFIED to LONG_ACCEL */
                object_list[onr].LONG_ACCEL_MODIFIED =
                    object_list[onr].LONG_ACCEL;
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_MODIFY_OBJECT_PROPERTIES_LAST_CYCLE */
static void AVLC_MODIFY_OBJECT_PROPERTIES_LAST_CYCLE(
    acc_object_ptr_t object_list) {
    uint8 onr;

    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        /*! copy object current id to last id*/
        object_list[onr].LAST_OBJECT_ID = object_list[onr].AUTOSAR.OBJECT_ID;
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_EXEC */
DLLEXPORT void AVLC_EXEC(const times_t cycle_time,
                         const VLCCustomInput_t *pVLCCustomInput,
                         const VLC_DFV2SenInfo_t *pDFVLongOut,
                         acc_input_data_t *input,
                         VLC_acc_object_t object_list[Acc_max_number_ooi],
                         VLC_acc_output_data_t *output,
                         VLC_acc_object_t *alert_object,
                         VLC_acc_object_t *display_object,
                         acc_status_t *acc_status,
                         OvertakeAssistInfo *p_overtake_assist_info) {
    AVLC_CHECK_FOR_INHIBITION(input, output);

    AVLC_DETERMINE_RECOMMENDED_VELOCITY(input, output);

    AVLC_OVERTAKE_ASSIST_PROCESS(cycle_time, pVLCCustomInput, acc_status,
                                 object_list, input, p_overtake_assist_info);

    AVLC_DETERMINE_CONTROL_DISTANCE(input, acc_status, output);

    AVLC_MODIFY_OBJECT_PROPERTIES(cycle_time, object_list);

    /* estimate driver intention */
    AVLC_ESTIMATE_DRIVER_INTENTION(
        cycle_time, &acc_status->AVLC_DRIVER_INTENTION, object_list, input);

    /* select objects of interest and add some information */
    AVLC_SELECT_OBJECTS_OF_INTEREST(
        input, pVLCCustomInput, &acc_status->AVLC_DRIVER_INTENTION, object_list,
        acc_status, p_overtake_assist_info, cycle_time);

    /* select function relevant objects */
    AVLC_SELECT_RELEVANT_OBJECT(input, pDFVLongOut, pVLCCustomInput,
                                object_list, &acc_status->AVLC_CONTROL_OBJECT,
                                alert_object, display_object, output,
                                acc_status);

    /* traffic situation estimation */
    AVLC_ESTIMATE_TRAFFIC_SITUATION(cycle_time, object_list, input,
                                    &acc_status->AVLC_CONTROL_OBJECT, output,
                                    &(acc_status->AVLC_DRIVER_INTENTION));

    /* assemble outputdata */
    AVLC_ASSEMBLE_OUTPUT_DATA(input, &acc_status->AVLC_CONTROL_OBJECT, output);

    AVLC_ALERT_EXEC(input, alert_object, pDFVLongOut, output,
                    &acc_status->AVLC_ALERT_DATA, cycle_time);

    AVLC_MODIFY_OBJECT_PROPERTIES_LAST_CYCLE(object_list);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */