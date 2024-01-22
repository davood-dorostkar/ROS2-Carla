/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "acc.h"
#include "acc_par.h"
#include "mat_std_ext.h"

/*************************************************************************************************************************
  Functionname:    AVLC_ESTIMATE_DRIVER_INTENTION */
void AVLC_ESTIMATE_DRIVER_INTENTION(const times_t cycle_time,
                                    acc_driver_intention_t* output,
                                    acc_object_ptr_t object_list,
                                    const acc_input_data_t* input) {
#define DELTA_REQ_DIST_FACTOR (factor_t)100 /*!< 10% of requested distance */
#define MAX_LANE_CHANGE_TIME \
    (times_t)10000 /*!< max time for lane change maneuver */

    static boolean lane_change_state = FALSE; /*!< lane change state */
    static times_t max_lane_change_timer =
        (times_t)0; /*!< max timer for lane change maneuver */
    static percentage_t p_lane_change_left_last_cycle =
        (percentage_t)0; /*!< probability of a lane change into left direction
                            last cycle*/
    static percentage_t
        p_lane_change_right_last_cycle =
            (percentage_t)0; /*!< probability of a lane change into right
                                direction last cycle*/

    SWITCH_SET_STATE(&(output->AVLC_ENAGAGED),
                     (boolean)(input->INPUT_STATUS.AVLC_ON == TRUE));

    /*! Lane change begin */
    if (((input->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT > (percentage_t)0) &&
         (p_lane_change_left_last_cycle == (percentage_t)0)) ||
        ((input->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT > (percentage_t)0) &&
         (p_lane_change_right_last_cycle == (percentage_t)0))) {
        lane_change_state = TRUE;
        /* start timer */
        max_lane_change_timer = MAX_LANE_CHANGE_TIME;
        /* set lane change from last cycle */
        p_lane_change_left_last_cycle =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT;
        p_lane_change_right_last_cycle =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT;
    }
    /*! Lane change end */
    else if ((input->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT == (percentage_t)0) &&
             (input->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT == (percentage_t)0)) {
        lane_change_state = FALSE;
        max_lane_change_timer = (times_t)0;
        p_lane_change_left_last_cycle = (percentage_t)0;
        p_lane_change_right_last_cycle = (percentage_t)0;
    }
    /*! during lane change */
    else {
        if (lane_change_state == TRUE) {
            distance_t delta_req_dist;

            delta_req_dist = object_list[Obj_first_host_lane]
                                 .REQUESTED_DISTANCE_MODIFIED_ACT *
                             DELTA_REQ_DIST_FACTOR / Factor_s;
            if (((object_list[Obj_first_host_lane]
                      .REQUESTED_DISTANCE_MODIFIED_ACT +
                  delta_req_dist) >
                 object_list[Obj_first_host_lane].AUTOSAR.LONG_DISPLACEMENT) ||
                (object_list[Obj_first_host_lane].AUTOSAR.OBJECT_ID !=
                 object_list[Obj_first_host_lane].LAST_OBJECT_ID) ||
                (max_lane_change_timer < cycle_time)) {
                lane_change_state = FALSE;
            }
        }
        p_lane_change_left_last_cycle =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT;
        p_lane_change_right_last_cycle =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT;
    }

    /*! timer */
    if (max_lane_change_timer > cycle_time) {
        max_lane_change_timer -= cycle_time;
    }

    if (lane_change_state == TRUE) {
        /*! Lane change probability is already filtered in SI */
        output->LANE_CHANGE_LEFT_PROBABILITY =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT;
        output->LANE_CHANGE_RIGHT_PROBABILITY =
            input->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT;
    } else {
        output->LANE_CHANGE_LEFT_PROBABILITY = (percentage_t)0;
        output->LANE_CHANGE_RIGHT_PROBABILITY = (percentage_t)0;
    }

    /*CC limiter active*/
    if (input->INPUT_STATUS.VLC_DECEL_LIM_OVERRIDE == TRUE) {
        /*internal copy not active yet*/
        if (output->DECEL_LIM_OVERRIDE.VLC_LIMITER_ACTIVE == FALSE) {
            output->DECEL_LIM_OVERRIDE.VLC_LIMITER_ACTIVE = TRUE;
            if (object_list[Obj_first_host_lane]
                    .AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) {
                if ((object_list[Obj_first_host_lane]
                         .AUTOSAR.OBJECT_STATUS.NEW == FALSE) &&
                    (object_list[Obj_first_host_lane].LONG_ACCEL_MODIFIED >=
                     input->VLC_DECEL_LIMIT)) {
                    output->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE = TRUE;
                } else {
                    output->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE = FALSE;
                }
            }
        }
    } else {
        output->DECEL_LIM_OVERRIDE.VLC_LIMITER_ACTIVE = FALSE;
        output->DECEL_LIM_OVERRIDE.AVLC_LIMITER_ACTIVE = FALSE;
    }

    /*save object id of stationary object thats relevant in cycle where the
     * drive off possible bit is true*/
    if ((input->INPUT_STATUS.AVLC_CONTROL_TO_FIRST_OBJECT == TRUE) &&
        (object_list[Obj_first_host_lane].AUTOSAR.LONG_DISPLACEMENT <=
         Acc_max_dist_rel_standing_obj)) {
        output->OBJECT_NR_TO_CONTROL_TO =
            object_list[Obj_first_host_lane].AUTOSAR.OBJECT_ID;
    }

    if (output->OBJECT_NR_TO_CONTROL_TO !=
        object_list[Obj_first_host_lane].AUTOSAR.OBJECT_ID) {
        output->OBJECT_NR_TO_CONTROL_TO = OBJ_INDEX_NO_OBJECT;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */