
/*
 * Copyright (C) 2022 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "stdio.h"
#include "cc.h"
#include "cc_par.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "acc_par.h"

static void VLC_INIT_ACCEL_REQUEST(
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input);
static void VLC_COORDINATE_ACCEL(
    const cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input);
static void VLC_LIMIT_ACCEL_CHANGE_RATE(
    const times_t cycle_time,
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    cart_das_output_data_t* das_output,
    const cc_input_data_t* input,
    const VLC_acc_output_data_t* acc_output);
static void VLC_LIMIT_DECEL(cc_status_t* cc_status);
static void VLC_DETERMINE_CRUISE_ACCEL(
    const cc_input_data_t* input,
    const cart_das_input_data_t* das_input,
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data);
static void VLC_COORDINATOR(const times_t cycle_time,
                            const VLC_acc_output_data_t* acc_output,
                            const cart_das_input_data_t* das_input,
                            cart_das_output_data_t* das_output,
                            cc_status_t* cc_status,
                            const cc_input_data_t* input);
static void VLC_COORDINATE_STANDSTILL(const VLC_acc_output_data_t* acc_output,
                                      const cart_das_input_data_t* das_input,
                                      cart_das_output_data_t* das_output,
                                      cc_status_t* cc_status);
static void VLC_ACCELERATION_REQUEST_FILTERING(
    const times_t cycle_time,
    cc_acceleration_control_data_t* accel_control_data,
    const cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output,
    cc_control_data_t* cc_control_data);
static void VLC_SET_ACCEL_REQUEST(
    const times_t cycle_time,
    cc_acceleration_control_data_t* accel_control_data,
    cart_das_output_data_t* das_output,
    const cc_driver_requests_t* driver_requests);
static void VLC_DETECT_TRANSITION_AVLC_TO_CC(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    cc_status_t* cc_status);

/*************************************************************************************************************************
  Functionname:    VLC_INIT_ACCEL_REQUEST */ /*!

                                                                                @brief           Initialize accel request

                                                                                @description     Initialize acceleration request at engagement or after driver
                                                                                                 override with the INIT_ACCELERATION. When the CC is engaged
                                                                                                 in a highly dynamic situation, the so called initialization
                                                                                                 phase is started. In this phase, higher gradients of the
                                                                                                 acceleration request are allowed, to prevent acceleration or deceleration
                                                                                                 overshoot. It is marked with INIT_PHASE_ACTIVE.

                                                                                @return          -

                                                                                @param[in]       driver_requests : Pointer to driver request input structure [cc_driver_requests_t as per cc_ext.h]
                                                                                                  CONTROL_STATE : Variable that reflects the state of the HMI state machine [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                                  CONTROL_STATE_LAST_CYCLE : Variable that reflects the state of the HMI state machine as per last cycle [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                                  DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM : Boolean confirming drive off request from driver [True, False]
                                                                                                  DRIVER_OPERATIONS_LAST_CYCLE.DRIVE_OFF_CONFIRM : Boolean confirming drive off request from driver as per last cycle [True, False]

                                                                                @param[in,out]   cc_control_data : Pointer to CC control data [cc_control_data_t as per cc_ext.h]
                                                                                                  cc_control_data->DECEL_LIM_OVERRIDE_ACTIVE: Boolean indicating if deceleration is limited after override [True, False]
                                                                                @param[in,out]   accel_control_data : Pointer to CC acceleration control data [cc_acceleration_control_data_t as per cc_ext.h]
                                                                                                  MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE : Minimum requested acceleration as per last cycle [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE : Maximum requested acceleration as per last cycle [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  MAXIMUM_COMMANDED_ACCELERATION : Maximum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  MINIMUM_COMMANDED_ACCELERATION : Minimum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  MIN_ALLOWED_ACCEL : Minimum allowed acceleration for control object [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  MAX_ALLOWED_ACCEL : Maximum allowed acceleration for control object [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                  ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE : Boolean indicating whether limiter due to lateral acceleration is active [True, False]
                                                                                                  ACCEL_STATUS.ALLOW_INIT : Indicates the possibility, to initialize the control acceleration to the current ego vehicle acceleration [TRUE,FALSE]
                                                                                @param[in]       das_input : Pointer to input from longitudinal dynamics management to driver assistance system [cart_das_input_data_t as per cart_ext.h]
                                                                                                  A_INIT : Vehicle initialisation acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)

                                                                                @glob_in         -
                                                                                @glob_out        -

                                                                                @c_switch_part   CFG_VLC_VLC_ALLOW_INIT_ACCEL_REQUEST : Configuration switch for init request acceleration to host vehicle acceleration
                                                                                @c_switch_part   CFG_VLC_VLC_USE_ACCEL_BAND_MODIFICATION : Configuration switch to use modification of acceleration band
                                                                                @c_switch_part   LONG_CFG_USE_EXTERN_AVLC_STATE_MACHINE : Configuration switch to use external ACC state machine

                                                                                @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing
                                                                                @c_switch_full   CFG_VLC_CC : Configuration switch for enabling CC processing


                                                                              *************************************************************************************************************************/
static void VLC_INIT_ACCEL_REQUEST(
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input) {
    /* Newly Engaged... */
    if ((driver_requests->CONTROL_STATE_LAST_CYCLE !=
         driver_requests->CONTROL_STATE) &&
        (driver_requests->CONTROL_STATE_LAST_CYCLE != Cc_cc_override) &&
        ((driver_requests->CONTROL_STATE == Cc_plim_active) ||
         (driver_requests->CONTROL_STATE == Cc_lim_active) ||
         (driver_requests->CONTROL_STATE == Cc_cc_active))) {
        /* Initialize CommandedAcceleration of last Cycle */
        if (driver_requests->CONTROL_STATE == Cc_plim_active) {
            accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                das_input->A_INIT;
            accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
        } else {
            if (driver_requests->CONTROL_STATE == Cc_lim_active) {
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    das_input->A_INIT;
                accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    accel_control_data
                        ->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
            } else {
                /* cc_active */
                /* Initialize acceleration request without accel band => jump to
                 * das_input->A_INIT */
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    (acceleration_t)MAT_LIM(
                        (sint32)das_input->A_INIT,
                        (sint32)accel_control_data->MIN_ALLOWED_ACCEL,
                        (sint32)accel_control_data->MAX_ALLOWED_ACCEL);
                accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    (acceleration_t)MAT_LIM(
                        (sint32)das_input->A_INIT,
                        (sint32)accel_control_data->MIN_ALLOWED_ACCEL,
                        (sint32)accel_control_data->MAX_ALLOWED_ACCEL);
            }
        }

    } else {
        /*init accel request if actual accel is in between last commanded accel
         * and requested accel*/
        if ((cc_control_data->DECEL_LIM_OVERRIDE_ACTIVE == FALSE) &&
            (accel_control_data->ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE == FALSE) &&
            ((driver_requests->CONTROL_STATE == Cc_cc_active) ||
             (driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
             (driver_requests->CONTROL_STATE == Cc_cc_override)) &&
            (accel_control_data->ACCEL_STATUS.ALLOW_INIT == TRUE)) {
            if (das_input->A_INIT ==
                (acceleration_t)MAT_LIM(
                    (sint32)das_input->A_INIT,
                    (sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION,
                    (sint32)accel_control_data
                        ->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE)) {
                accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    das_input->A_INIT;
            }
            if (das_input->A_INIT ==
                (acceleration_t)MAT_LIM(
                    (sint32)das_input->A_INIT,
                    (sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION,
                    (sint32)accel_control_data
                        ->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE)) {
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
                    das_input->A_INIT;
            }
        }
    }

    /*init accel request after standstill and driver input*/
    if ((driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM == TRUE)) {
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
            accel_control_data->MAXIMUM_COMMANDED_ACCELERATION;
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
            accel_control_data->MAXIMUM_COMMANDED_ACCELERATION;
    }
    accel_control_data
        ->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE = (acceleration_t)MAT_MIN(
        (sint32)accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE,
        (sint32)accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE);
}

static void VLC_ACCEL_STATUS_CHANGE(
    const cc_input_data_t* input,
    cc_acceleration_control_data_t* accel_control_data) {
    if (input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == TRUE) {
        accel_control_data->ACCEL_STATUS.NO_RAMP = TRUE;
    }
    if (input->INHIBIT.VLC_DISENGAGEMENT_RAMP == TRUE) {
        accel_control_data->ACCEL_STATUS.CANCEL_RAMP = TRUE;
    }
    if (input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP == TRUE) {
        accel_control_data->ACCEL_STATUS.RAPID_RAMP = TRUE;
    }
}

static void VLC_MIN_MAX_JERK_CAL(
    const times_t cycle_time,
    const cc_driver_requests_t* driver_requests,
    cc_acceleration_control_data_t* accel_control_data,
    cart_das_output_data_t* das_output,
    const cc_input_data_t* input,
    gradient_t max_neg_grad,
    gradient_t max_pos_grad,
    gradient_t* p_min_neg_grad_limit,
    gradient_t* p_max_neg_grad_limit,
    gradient_t* p_min_pos_grad_limit,
    gradient_t* p_max_pos_grad_limit) {
    gradient_t min_max_pos_grad;
    gradient_t min_max_neg_grad;
    gradient_t max_max_pos_grad;
    gradient_t max_max_neg_grad;

    if ((driver_requests->CONTROL_STATE == Cc_cc_active) ||
        (driver_requests->CONTROL_STATE == Cc_cc_decel_only)) {
        /*Reset accel ramps when active*/
        accel_control_data->ACCEL_STATUS.NO_RAMP = FALSE;
        accel_control_data->ACCEL_STATUS.RAPID_RAMP = FALSE;
        accel_control_data->ACCEL_STATUS.CANCEL_RAMP = FALSE;

        if ((driver_requests->DRIVER_OPERATIONS.ACCEL_MODE == TRUE) ||
            (driver_requests->DRIVER_OPERATIONS.DECEL_MODE == TRUE)) {
            min_max_neg_grad = Cc_max_neg_grad_accel_decel;
            min_max_pos_grad = Cc_max_pos_grad_accel_decel;
            max_max_neg_grad = Cc_max_neg_grad_accel_decel;
            max_max_pos_grad = Cc_max_pos_grad_accel_decel;
        } else {
            min_max_neg_grad = max_neg_grad;
            min_max_pos_grad = max_pos_grad;
            max_max_neg_grad = max_neg_grad;
            max_max_pos_grad = max_pos_grad;
        }

        /* The system is decelerating */
        if ((das_output->MIN_REQ_ACCEL < (acceleration_t)0) &&
            (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == FALSE)
            /* With the next step 0 will be passed... */
            &&
            ((sint32)
                 accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE +
             ((*p_min_pos_grad_limit * (sint32)cycle_time) / (sint32)Time_s)) >
                (sint32)0) {
            /* the next step will be the difference to 0 */
            *p_min_pos_grad_limit = ((
                (((sint32)0 - (sint32)accel_control_data
                                  ->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE) *
                 (sint32)Time_s) /
                (sint32)cycle_time));
        }

        *p_min_pos_grad_limit =
            MAT_MIN(*p_min_pos_grad_limit, min_max_pos_grad);
        *p_min_pos_grad_limit =
            MAT_MAX(*p_min_pos_grad_limit, Cc_min_grad_rest);
        *p_min_neg_grad_limit =
            MAT_MIN(*p_min_neg_grad_limit, -Cc_min_grad_rest);
        *p_min_neg_grad_limit =
            MAT_MAX(*p_min_neg_grad_limit, min_max_neg_grad);

        /* The system is decelerating */
        if ((das_output->MAX_REQ_ACCEL < (acceleration_t)0) &&
            (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == FALSE)
            /* With the next step 0 will be passed... */
            &&
            ((sint32)
                 accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE +
             ((*p_max_pos_grad_limit * (sint32)cycle_time) / (sint32)Time_s)) >
                (sint32)0) {
            /* the next step will be the difference to 0 */
            *p_max_pos_grad_limit = ((
                (((sint32)0 - (sint32)accel_control_data
                                  ->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE) *
                 (sint32)Time_s) /
                (sint32)cycle_time));
        }

        *p_max_pos_grad_limit =
            MAT_MIN(*p_max_pos_grad_limit, max_max_pos_grad);
        *p_max_pos_grad_limit =
            MAT_MAX(*p_max_pos_grad_limit, Cc_min_grad_rest);
        *p_max_neg_grad_limit =
            MAT_MIN(*p_max_neg_grad_limit, -Cc_min_grad_rest);
        *p_max_neg_grad_limit =
            MAT_MAX(*p_max_neg_grad_limit, max_max_neg_grad);
    } else {
        if (driver_requests->CONTROL_STATE == Cc_cc_disengage) {
            /*copy inhibition bits*/
            VLC_ACCEL_STATUS_CHANGE(input, accel_control_data);
            accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE = TRUE;

            if (accel_control_data->ACCEL_STATUS.NO_RAMP == TRUE) {
                *p_min_neg_grad_limit = max_neg_grad;
                *p_min_pos_grad_limit = max_pos_grad;
            } else if (accel_control_data->ACCEL_STATUS.RAPID_RAMP == TRUE) {
                *p_min_neg_grad_limit = Cc_max_neg_grad_rapid_disengage;
                *p_min_pos_grad_limit = Cc_max_pos_grad_rapid_disengage;

                *p_max_neg_grad_limit = Cc_max_neg_grad_rapid_disengage;
                *p_max_pos_grad_limit = Cc_max_pos_grad_rapid_disengage;
            } else if (accel_control_data->ACCEL_STATUS.CANCEL_RAMP == TRUE) {
                *p_min_neg_grad_limit = Cc_max_neg_grad_disengage;
                *p_min_pos_grad_limit = Cc_max_pos_grad_disengage;

                *p_max_neg_grad_limit = Cc_max_neg_grad_disengage;
                *p_max_pos_grad_limit = Cc_max_pos_grad_disengage;
            }

            if (((das_output->MIN_REQ_ACCEL -
                  accel_control_data->MINIMUM_COMMANDED_ACCELERATION) >
                 -Cc_accel_band) &&
                ((das_output->MAX_REQ_ACCEL -
                  accel_control_data->MAXIMUM_COMMANDED_ACCELERATION) <
                 Cc_accel_band)) {
                accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE = FALSE;
            }
        } else {
            if (driver_requests->CONTROL_STATE == Cc_lim_disengage) {
                accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE = TRUE;

                *p_min_neg_grad_limit = Lim_max_neg_grad_disengage;
                *p_min_pos_grad_limit = Lim_max_pos_grad_disengage;

                *p_max_neg_grad_limit = Lim_max_neg_grad_disengage;
                *p_max_pos_grad_limit = Lim_max_pos_grad_disengage;

                if ((das_output->MIN_REQ_ACCEL >= Lim_accel_limit) ||
                    (driver_requests->ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE ==
                     TRUE)) {
                    accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE = FALSE;
                }
            } else {
                /*cc_override*/
                if (driver_requests->CONTROL_STATE == Cc_cc_override) {
                    /* During Override any jerk shall be accepted */
                    *p_min_neg_grad_limit = max_neg_grad;
                    *p_min_pos_grad_limit = max_pos_grad;
                }
            }
        }
    }
}

/*!

@brief           This function limits the gradient in the requested acceleration
              to increase comfort for the driver. The allowed gradients depend
              on the control mode.

@description     Positive and negative gradient limits (jerk) for maximum and
minimum acceleration requests are calculated based on speed, ACC/CC conditions,
limiter conditions, ramp conditions, etc. If CC is engaged in dynamic
situations, higher acceleration gradients are allowed via
VLC_INIT_ACCEL_REQUEST. Difference between current commanded acceleration and
requested acceleration of last cycle is used to calculate actual required jerks
for both min and max values. Actual jerks are then limited within calculated
positive and negative jerk limits giving allowed jerks. Maximum and minimum
requested accelerations are limited from previous cycle values based on allowed
jerks.

@return          -

@param[in]       cycle_time : cycle time of VLC_VEH cycle [0u ... 1000u] ms
@param[in]       driver_requests : Pointer to driver request input structure
[cc_driver_requests_t as per cc_ext.h] CONTROL_STATE : Variable that reflects
the state of the HMI state machine [full range of datatype cc_control_state_t as
in cc_ext.h] DRIVER_OPERATIONS.ACCEL_MODE : Boolean indicating if acceleration
with constant value [TRUE, FALSE] DRIVER_OPERATIONS.DECEL_MODE : Boolean
indicating if deceleration with constant value [TRUE, FALSE]
               ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE : Boolean indicating
override condition by driver [TRUE, FALSE]
@param[in,out]   cc_control_data : Pointer to CC control data [cc_control_data_t
as per cc_ext.h] cc_control_data->VLC_AVLC_TO_VLC_TRANSITION : Boolean, if TRUE
indicates transition from ACC (object-control) to CC [TRUE, FALSE]\n
                                   accel_control_data->ACCEL_GRADIENT_LIMITS.MAX_NEG_GRAD:
neg. Change rate (d/dt) of signal. Attention, gradient is scaled same as signal
[-2147483.648...+2147483.648] m/s3 [gradient_t]\n
                                   accel_control_data->ACCEL_GRADIENT_LIMITS.MAX_POS_GRAD:
pos. Change rate (d/dt) of signal. Attention, gradient is scaled same as signal
[-2147483.648...+2147483.648] m/s3 [gradient_t]\n
@param[in,out]   accel_control_data : Pointer to CC acceleration control data
[cc_acceleration_control_data_t as per cc_ext.h] ACCEL_STATUS.OBJECT_EFFECTIVE :
Boolean, if TRUE indicating effective OOI for ACC [TRUE, FALSE]
               MINIMUM_COMMANDED_ACCELERATION : Minimum commanded acceleration
[-30000u ...+30000u] m/s2 (with factor Acceleration_s)
               MAXIMUM_COMMANDED_ACCELERATION : Maximum commanded acceleration
[-30000u ...+30000u] m/s2 (with factor Acceleration_s)
               MINIMUM_REQUESTED_ACCELERATION : Minimum requested acceleration
[-30000u ...+30000u] m/s2 (with factor Acceleration_s)
               MAXIMUM_REQUESTED_ACCELERATION : Maximum requested acceleration
[-30000u ...+30000u] m/s2 (with factor Acceleration_s)
               MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE : Minimum requested
acceleration as per last cycle [-30000u ...+30000u] m/s2 (with factor
Acceleration_s) MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE : Maximum requested
acceleration as per last cycle [-30000u ...+30000u] m/s2 (with factor
Acceleration_s) ACCEL_STATUS.NO_RAMP : Boolean, if TRUE indicating no
acceleration ramp [TRUE, FALSE] ACCEL_STATUS.RAPID_RAMP : Boolean, if TRUE
indicating rapid acceleration ramp [TRUE, FALSE] ACCEL_STATUS.CANCEL_RAMP :
Boolean, if TRUE indicating acceleration ramp cancelled [TRUE, FALSE]
               ACCEL_STATUS.ACCEL_RAMP_ACTIVE : Boolean ,if TRUE indicating
acceleration ramp active [TRUE, FALSE]
@param[in]       das_input : Pointer to input from longitudinal dynamics
management to driver assistance system [cart_das_input_data_t as per cart_ext.h]
               VEHICLE_SPEED : Ego speed with factor Velocity_s [-30000u ...
30000u] cm/s
@param[in,out]   das_output : Pointer to output data from driver assistance
system to longitudinal dynamics management [cart_das_output_data_t as per
cart_ext.h] MIN_REQ_ACCEL : Minimum required acceleration output [-30000u
...+30000u] m/s2 (with factor Acceleration_s)
@param[in]       input : Pointer to cc_input_data_t type structure containing
input data to CC [cc_input_data_t as per cc_ext.h]
               INHIBIT.VLC_DISENGAGEMENT_NO_RAMP : Inhibition bit for no ramp
[TRUE, FALSE] INHIBIT.VLC_DISENGAGEMENT_RAMP : Inhibition bit for ramp [TRUE,
FALSE] INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP : Inhibition bit for rapid ramp
[TRUE, FALSE]
@param[in]       acc_output : Pointer to ACC output [VLC_acc_output_data_t  as
per Rte_Type.h] SITUATION_CLASS.CRITICALITY : Criticality of the situation [0u
... 255u] (with factor 255/100)

@glob_in         -
@glob_out        -

@c_switch_part   CFG_VLC_ACC : Configuration switch for enabling ACC processing
@c_switch_part   CFG_VLC_FLIM : Configuration switch for FLIM support
@c_switch_part   CFG_VLC_VLC_USE_SMOOTH_JERKS : Configuration switch enabling
smooth gradients for acceleration requests
@c_switch_part   CFG_VLC_VLC_USE_MIN_MAX_DISENGAGEMENT : Configuration switch
enabling ramping of required acceleration at disengagement
@c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling
VLC_LONG processing
@c_switch_full   CFG_VLC_CC : Configuration switch for enabling CC processing
*************************************************************************************************************************/
static void VLC_LIMIT_ACCEL_CHANGE_RATE(
    const times_t cycle_time,
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    cart_das_output_data_t* das_output,
    const cc_input_data_t* input,
    const VLC_acc_output_data_t* acc_output) {
    gradient_t min_actual_grad;
    gradient_t max_actual_grad;
    gradient_t min_neg_grad_limit;
    gradient_t max_neg_grad_limit;
    gradient_t min_pos_grad_limit;
    gradient_t max_pos_grad_limit;
    gradient_t min_allowed_grad;
    gradient_t max_allowed_grad;

    sint32 help_accel;

    // sint32     com_req_min_accel_delta;
    // sint32 com_req_max_accel_delta;
    gradient_t max_neg_grad;
    gradient_t max_pos_grad;

    times_t smooth_grad_neg_time;
    times_t smooth_grad_pos_time;

    sint32 delta_time;

#define ACCEPTABLE_TIME 250
#define ACCELERATION_FOR_MAXIMUM_GRAD -3500
#define MAX_ACCEL_DELTA 1500

    /* Does the Inpath-Object actually influence the longitudinal control ?*/
    if (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == TRUE) {
        /****************************/
        /*   ACC Gradient Limits    */
        /****************************/
        max_neg_grad = MAT_CALCULATE_PARAM_VALUE1D(
            Acc_max_neg_grad, Acc_neg_grad_points, das_input->VEHICLE_SPEED);

        /* modify positive acceleration request dependent on the sign */
        if (das_output->MIN_REQ_ACCEL >= (acceleration_t)0) {
            /* acceleration gradient for positive accel requests */
            max_pos_grad = MAT_CALCULATE_PARAM_VALUE1D(
                Acc_max_pos_grad_pos_accel, Acc_pos_grad_pos_accel_points,
                das_input->VEHICLE_SPEED);
            /*max_pos_grad =
             * (gradient_t)(max_pos_grad*accel_pos_gradient_factor/Factor_s);*/

            if (das_input->LODM_STAT.DAS_MODE == Das_Mode_Eco) {
                max_pos_grad =
                    max_pos_grad * Acc_pos_accel_grad_eco_fac / Scale_100;
            } else if (das_input->LODM_STAT.DAS_MODE == Das_Mode_Sport) {
                max_pos_grad =
                    max_pos_grad * Acc_pos_accel_grad_sport_fac / Scale_100;
            }
        } else {
            /* acceleration gradient for negative accel requests */
            max_pos_grad = MAT_CALCULATE_PARAM_VALUE1D(
                Acc_max_pos_grad_neg_accel, Acc_pos_grad_neg_accel_points,
                das_input->VEHICLE_SPEED);
        }
    } else {
        /****************************/
        /*    CC Gradient Limits    */
        /****************************/
        max_neg_grad = MAT_CALCULATE_PARAM_VALUE1D(
            Cc_max_neg_grad, Cc_neg_grad_points, das_input->VEHICLE_SPEED);

        /* modify positive acceleration request dependent on the sign */
        if (das_output->MIN_REQ_ACCEL >= (acceleration_t)0) {
            /* acceleration gradient for positive accel requests */
            max_pos_grad = MAT_CALCULATE_PARAM_VALUE1D(
                Cc_max_pos_grad_pos_accel, Cc_pos_grad_pos_accel_points,
                das_input->VEHICLE_SPEED);

            if (das_input->LODM_STAT.DAS_MODE == Das_Mode_Eco) {
                max_pos_grad =
                    max_pos_grad * Acc_pos_accel_grad_eco_fac / Scale_100;
            } else if (das_input->LODM_STAT.DAS_MODE == Das_Mode_Sport) {
                max_pos_grad =
                    max_pos_grad * Acc_pos_accel_grad_sport_fac / Scale_100;
            }
            /*max_pos_grad =
             * (gradient_t)(max_pos_grad*accel_pos_gradient_factor/Factor_s);*/
            // if (cc_control_data->VLC_AVLC_TO_VLC_TRANSITION == TRUE) {
            //     /*! max_pos_grad in the case of transition will be reduced
            //     for
            //      * some time */
            //     help_accel = (sint32)max_pos_grad *
            //                  (sint32)Cc_acc_to_cc_transition_factor;
            //     help_accel /= (sint32)Acceleration_s;
            //     max_pos_grad = (gradient_t)help_accel;
            // }
        } else {
            /* acceleration gradient for negative accel requests */
            max_pos_grad = MAT_CALCULATE_PARAM_VALUE1D(
                Cc_max_pos_grad_neg_accel, Cc_pos_grad_neg_accel_points,
                das_input->VEHICLE_SPEED);
        }
    }

    /*Determine acceleration difference*/
    // com_req_min_accel_delta =
    // ((sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION -
    // (sint32)accel_control_data->MINIMUM_REQUESTED_ACCELERATION);
    // com_req_max_accel_delta =
    //     ((sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION -
    //      (sint32)accel_control_data->MAXIMUM_REQUESTED_ACCELERATION);

    /*Determine smooth gradient times*/
    // smooth_grad_pos_time = (times_t)MAT_CALCULATE_PARAM_VALUE1D(
    //     Cc_smooth_gradient_pos_time,
    //     (uint16)Cc_smooth_gradient_pos_time_points,
    //     das_input->VEHICLE_SPEED);
    // smooth_grad_neg_time = (times_t)MAT_CALCULATE_PARAM_VALUE1D(
    //     Cc_smooth_gradient_neg_time,
    //     (uint16)Cc_smooth_gradient_neg_time_points,
    //     das_input->VEHICLE_SPEED);
    /*Determine after which time the difference will be zero in case of
     * max_pos_grad*/

    /* Temporary modified, has to be investigated, if this functionality is
     * still usable */

    // delta_time = MAT_LIM((com_req_max_accel_delta / max_pos_grad) * Time_s,
    //                      Time_max, Time_min);

    // if ((delta_time > (smooth_grad_pos_time / 2)) &&
    //     (accel_control_data->MAXIMUM_COMMANDED_ACCELERATION > 0) &&
    //     (accel_control_data->MINIMUM_REQUESTED_ACCELERATION < 0)) {
    //     max_pos_grad = Cc_max_release_brake_grad;
    // }

    // {
    //     sint32 absacc;
    //     gradient_t set_grad;

    //     absacc = MAT_MAX(
    //         MAT_ABS((sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION
    //         -
    //                 (sint32)accel_control_data->MINIMUM_REQUESTED_ACCELERATION),
    //         MAT_ABS(
    //             (sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION -
    //             (sint32)accel_control_data->MAXIMUM_REQUESTED_ACCELERATION));

    //     if (driver_requests->CONTROL_STATE != Cc_cc_disengage) {
    //         set_grad = ((absacc * Time_s) / smooth_grad_pos_time);

    //         if (das_input->VEHICLE_SPEED >
    //             Cc_smooth_gradient_pos_min_velocity) {
    //             max_pos_grad = MAT_LIM(set_grad, Cc_smooth_jerk_min_gradient,
    //                                    max_pos_grad);
    //         }

    //         /*Only smooth jerks within normal acceleration requests*/
    //         if (accel_control_data->MINIMUM_COMMANDED_ACCELERATION >
    //             ACCELERATION_FOR_MAXIMUM_GRAD) {
    //             set_grad = (-absacc * Time_s / smooth_grad_neg_time);
    //             max_neg_grad = MAT_LIM(set_grad, max_neg_grad,
    //                                    -Cc_smooth_jerk_min_gradient);
    //         }
    //     }
    // }

    /* Modify the limits of the acceleration gradient in critical situations */
    {
        /* Allow bigger accel gradients in critical traffic situations to reduce
         * the reaction time */
        max_neg_grad *= (sint32)MAT_CALCULATE_PARAM_VALUE1D(
            criticality_accel_gradient_gain, accel_gradient_gain_points,
            (sint16)(acc_output->SITUATION_CLASS.CRITICALITY * Scale_100 /
                     Confidence_s));
        max_neg_grad /= Scale_100;

        max_neg_grad_limit = MAT_CALCULATE_PARAM_VALUE1D(
            max_decel_gradient_critical, max_decel_gradient_critical_points,
            das_input->VEHICLE_SPEED);
        max_neg_grad = MAT_MAX(max_neg_grad, max_neg_grad_limit);
    }

    /* CmdAccelOld is initialized at engagement */
    // VLC_INIT_ACCEL_REQUEST(driver_requests, cc_control_data,
    // accel_control_data,
    //                        das_input);

    if (driver_requests->CONTROL_STATE != Cc_cc_active ||
        (das_output->DAS_STAT.DAS_DRIVE_OFF == TRUE &&
         das_output->DAS_STAT.DAS_DRIVE_OFF_LAST_CYC == FALSE &&
         driver_requests->CONTROL_STATE == Cc_cc_active)) {
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE = 0;
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE = 0;
    } else {
        /*save last acceleration request*/
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
            accel_control_data->MINIMUM_REQUESTED_ACCELERATION;
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE =
            accel_control_data->MAXIMUM_REQUESTED_ACCELERATION;
    }

    /* Determine ActualJerk */
    min_actual_grad =
        ((((sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION -
           (sint32)
               accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE) *
          (sint32)Time_s) /
         (sint32)cycle_time);
    max_actual_grad =
        ((((sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION -
           (sint32)
               accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE) *
          (sint32)Time_s) /
         (sint32)cycle_time);

    // min_neg_grad_limit = min_pos_grad_limit = min_actual_grad;
    // max_neg_grad_limit = max_pos_grad_limit = max_actual_grad;

    // /* Determine Minimum and Maximum Jerk for Positive and Negative Ramp */
    // if ((driver_requests->CONTROL_STATE == Cc_lim_active) ||
    //     (driver_requests->CONTROL_STATE == Cc_plim_active)) {
    //     /* Variable or Permanent Speed limiter active */
    //     min_neg_grad_limit = MAT_MIN(min_neg_grad_limit, Lim_max_neg_grad);
    //     min_neg_grad_limit = MAT_MAX(min_neg_grad_limit, -Lim_min_grad_rest);
    //     min_pos_grad_limit = MAT_MIN(min_pos_grad_limit, Lim_max_pos_grad);
    //     min_pos_grad_limit = MAT_MAX(min_pos_grad_limit, Lim_min_grad_rest);

    //     max_neg_grad_limit = MAT_MIN(max_neg_grad_limit, Lim_max_neg_grad);
    //     max_neg_grad_limit = MAT_MAX(max_neg_grad_limit, -Lim_min_grad_rest);
    //     max_pos_grad_limit = MAT_MIN(max_pos_grad_limit, Lim_max_pos_grad);
    //     max_pos_grad_limit = MAT_MAX(max_pos_grad_limit, Lim_min_grad_rest);
    // } else {
    //     if (driver_requests->CONTROL_STATE == Cc_lim_override) {
    //         /* Variable Speed limiter override */
    //         min_neg_grad_limit = Lim_max_neg_grad_override;
    //         min_pos_grad_limit = Lim_max_pos_grad_override;

    //         max_neg_grad_limit = Lim_max_neg_grad_override;
    //         max_pos_grad_limit = Lim_max_pos_grad_override;
    //     } else {
    //         /* engaged or override */
    //         VLC_MIN_MAX_JERK_CAL(
    //             cycle_time, driver_requests, accel_control_data, das_output,
    //             input, max_neg_grad, max_pos_grad, &min_neg_grad_limit,
    //             &max_neg_grad_limit, &min_pos_grad_limit,
    //             &max_pos_grad_limit);
    //     }
    // }
    /* Limit Jerk */

    min_allowed_grad = MAT_LIM(min_actual_grad, max_neg_grad, max_pos_grad);
    help_accel = ((min_allowed_grad * (sint32)cycle_time) / (sint32)Time_s);
    accel_control_data->MINIMUM_REQUESTED_ACCELERATION =
        (acceleration_t)((sint32)accel_control_data
                             ->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE +
                         help_accel);

    max_allowed_grad = MAT_LIM(max_actual_grad, max_neg_grad, max_pos_grad);
    help_accel = ((max_allowed_grad * (sint32)cycle_time) / (sint32)Time_s);
    accel_control_data->MAXIMUM_REQUESTED_ACCELERATION =
        (acceleration_t)((sint32)accel_control_data
                             ->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE +
                         help_accel);

    /* write gradient limits to accel_control_data sutruct*/
    accel_control_data->ACCEL_GRADIENT_LIMITS.MAX_NEG_GRAD = max_neg_grad;
    accel_control_data->ACCEL_GRADIENT_LIMITS.MAX_POS_GRAD = max_pos_grad;
}

/*************************************************************************************************************************
  Functionname:    VLC_LIMIT_DECEL */ /*!

                                                                                       @brief           Limitation of CC deceleration after override

                                                                                       @description     After a driver override, the deceleration needed to control to the set speed, is limited to
                                                                                                        increase comfort for the driver. The DECEL_LIM_OVERRIDE bit is set when a transition from "override"
                                                                                                        to "active" is detected and the commanded acceleration is lower than the according calibration
                                                                                                        Cc_max_decel_after_override. The DECEL_LIM_OVERRIDE bit is reset when the setspeed is decreased
                                                                                                        or the commanded acceleration is higher than the according calibration Cc_max_decel_after_override.
                                                                                                        When engagement transition for ACC is detected the boolean DECEL_LIM_ENGAGE is set as TRUE. This boolean
                                                                                                        is reset again when driver reduces set speed or commanded acceleration is greater than Cc_max_decel_after_engage.

                                                                                       @return          -

                                                                                       @param[in,out]   cc_status : Pointer to Cruisecontrol status [cc_status_t as per cc_ext.h]
                                                                                                         VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE : Boolean, if TRUE indicating limitation of deceleration after engage [TRUE, FALSE]
                                                                                                         VLC_CONTROL_DATA.VLC_ACCELERATION : CC acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                                             VLC_ACCEL_CONTROL_DATA.MINIMUM_COMMANDED_ACCELERATION: \n
                                                                                                         VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE : Boolean indicating if deceleration is limited after override [TRUE, FALSE]
                                                                                                         VLC_CONTROL_DATA.DECEL_LIM_ACTIVE_COUNTER : Counter for tracking duration that deceleration limiter is active [0u ... 65500u]
                                                                                                         VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT : Minimum CC acceleration limit [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                         VLC_DRIVER_REQUESTS.CONTROL_STATE : Variable that reflects the state of the HMI state machine [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                                         VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE : Variable that reflects the state of the HMI state machine as per last cycle [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                                         VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED : Boolean if constant decreasing of set speed [TRUE, FALSE]
                                                                                                         VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.ACCEL_MODE : Boolean indicating if acceleration with constant value [TRUE, FALSE]
                                                                                                         VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DECEL_MODE : Boolean indicating if deceleration with constant value [TRUE, FALSE]

                                                                                       @return         void

                                                                                     *****************************************************************************/
static void VLC_LIMIT_DECEL(cc_status_t* cc_status) {
    if ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) &&
        (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
         FALSE)) {
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE = TRUE;
        cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT =
            Cc_max_decel_after_override;
    } else {
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE = FALSE;
        cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT = Accel_min;
    }

    /* Limit Deceleration After Override ? */
    // if (cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE == FALSE) {
    //     /* Activate Limiter when not active */
    //     if ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) &&
    //         (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE ==
    //          Cc_cc_override)) {
    //         /*set ENGAGE_LIMITER always to true after override*/
    //         cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE =
    //             TRUE;
    //         /* set deceleration limiter if no set speed decrease happened and
    //          * commanded acceleration < -0.4 */
    //         if ((cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
    //                  .VLC_DECREASE_SET_SPEED == FALSE) &&
    //             (cc_status->VLC_ACCEL_CONTROL_DATA
    //                  .MINIMUM_COMMANDED_ACCELERATION <
    //              Cc_max_decel_after_override)) {
    //             cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE = TRUE;
    //             cc_status->VLC_CONTROL_DATA.DECEL_LIM_ACTIVE_COUNTER =
    //                 Cc_max_decel_limit_time;
    //         }
    //     }
    // } else {
    //     /* Deactivate Limiter when active */
    //     if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) {
    //         if (cc_status->VLC_CONTROL_DATA.DECEL_LIM_ACTIVE_COUNTER >
    //             (uint16)0) {
    //             cc_status->VLC_CONTROL_DATA.DECEL_LIM_ACTIVE_COUNTER--;

    //             if ((cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
    //                      .VLC_DECREASE_SET_SPEED == TRUE) ||
    //                 (cc_status->VLC_ACCEL_CONTROL_DATA
    //                      .MINIMUM_COMMANDED_ACCELERATION >
    //                  (cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT
    //                  -
    //                   Cc_accel_band)) ||
    //                 (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
    //                      .ACCEL_MODE == TRUE) ||
    //                 (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
    //                      .DECEL_MODE == TRUE)) {
    //                 cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE =
    //                     FALSE;
    //             }
    //         } else {
    //             cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE =
    //             FALSE;
    //         }
    //     } else {
    //         cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE = FALSE;
    //     }
    // }

    // /*if cc_control_data->DECEL_LIM_OVERRIDE_ACTIVE -> limit max deceleration
    // to
    //  * -0.4m/(s^2)*/
    // if (cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE == TRUE) {
    //     cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT =
    //         Cc_max_decel_after_override;
    // } else {
    //     cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT = Accel_min;
    // }

    // /*------------------ decel limit after engagement to avoid using brakes
    //  * directly after engagement*/

    // /* Limit Deceleration After Resume / Set */
    // if (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE ==
    //     FALSE) {
    //     if ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_engage) &&
    //         (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE ==
    //          Cc_cc_ready)) {
    //         cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE =
    //             TRUE;
    //     }
    // } else {
    //     if ((cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
    //              .VLC_DECREASE_SET_SPEED == TRUE) ||
    //         (cc_status->VLC_CONTROL_DATA.VLC_ACCELERATION >
    //          Cc_max_decel_after_engage) ||
    //         (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.ACCEL_MODE ==
    //          TRUE) ||
    //         (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DECEL_MODE ==
    //          TRUE)) {
    //         cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE =
    //             FALSE;
    //     }
    // }
}

/*************************************************************************************************************************
  Functionname:    VLC_COORDINATE_ACCEL */ /*!

                                                                                  @brief           Determine the acceleration band depending on the CONTROL_STATE.
                                                                                                   At this point the decision is made which "use case" for the
                                                                                                   acceleration band is to be used. See CARTRONIC documentation.

                                                                                  @description     Based on the control state, the acceleration band (MINIMUM_COMMANDED_ACCELERATION and
                                                                                                   MAXIMUM_COMMANDED_ACCELERATION) is determined. The band values are selected amongst maximum acceleration
                                                                                                   limited due to lateral acceleration and acceleration requests from CC, limiter as well as ACC (object-based).
                                                                                                   The values are limited as required within max and min allowed acceleration limits as well as customer
                                                                                                   specific allowed limits. If the object-based ACC acceleration is lowest amongst the available requests,
                                                                                                   ACC flag (OBJECT_EFFECTIVE) is set to TRUE. Conditions for acceleration initialisation are also tested.

                                                                                  @return          -

                                                                                  @param[in]       driver_requests : Pointer to driver request input structure [cc_driver_requests_t as per cc_ext.h]
                                                                                                    CONTROL_STATE : Variable that reflects the state of the HMI state machine [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                                    DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED : Boolean if constant decreasing of set speed [TRUE, FALSE]
                                                                                                    DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED : Boolean if constant increasing of set speed [TRUE, FALSE]
                                                                                  @param[in]       acc_output : Pointer to ACC output [VLC_acc_output_data_t  as per Rte_Type.h]
                                                                                                    DISTANCE_CTRL_ACCEL_MIN : Minimum acceleration for intrusion distance [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    DISTANCE_CTRL_ACCEL_MAX : Maximum acceleration for intrusion distance [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MAX_AVLC_ACCELERATION : Maximum allowed ACC acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    AVLC_OUTPUT_STATUS.ALLOW_INIT : Indicates the possibility, to initialize the control acceleration to the current ego vehicle acceleration [TRUE,FALSE]
                                                                                  @param[in,out]   cc_control_data : Pointer to CC control data [cc_control_data_t as per cc_ext.h]
                                                                                                    VLC_ACCELERATION : CC acceleration limit [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MINIMUM_ACCELERATION_LIMIT : Minimum CC acceleration limit [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    DECEL_LIM_OVERRIDE_ACTIVE : Boolean indicating if deceleration is limited after override [TRUE, FALSE]
                                                                                                    LIM_ACCELERATION : Limiter acceleration limit [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                  @param[in,out]   accel_control_data : Pointer to CC acceleration control data [cc_acceleration_control_data_t as per cc_ext.h]
                                                                                                    MAXIMUM_ACCELERATION_LATERAL_LIMITED : Maximum longitudinal acceleration limited due to lateral acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MIN_ALLOWED_ACCEL : Minimum allowed acceleration for control object [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MAX_ALLOWED_ACCEL : Minimum allowed acceleration for control object [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MINIMUM_COMMANDED_ACCELERATION : Minimum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MAXIMUM_COMMANDED_ACCELERATION : Maximum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MIN_CUSTOM_ALLOWED_ACCEL : Minimum custom allowed acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    MAX_CUSTOM_ALLOWED_ACCEL : Maximum custom allowed acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    ACCEL_STATUS.OBJECT_EFFECTIVE : Boolean, if TRUE indicating effective OOI for ACC [TRUE, FALSE]
                                                                                                    ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE : Boolean indicating whether limiter due to lateral acceleration is active [True, False]
                                                                                                    ACCEL_STATUS.ALLOW_INIT : Indicates the possibility, to initialize the control acceleration to the current ego vehicle acceleration [TRUE,FALSE]
                                                                                  @param[in]       das_input : Pointer to input from longitudinal dynamics management to driver assistance system [cart_das_input_data_t as per cart_ext.h]
                                                                                                    A_INIT : Vehicle initialisation acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                    VEHICLE_SPEED : Ego speed with factor Velocity_s [-30000u ... 30000u] cm/s

                                                                                  @c_switch_part   CFG_VLC_ACC : Configuration switch for enabling ACC processing
                                                                                  @c_switch_part   CFG_VLC_ALWAYS_ALLOW_INITIALISATION : Configuration switch to allow initialisation
                                                                                  @c_switch_part   CFG_VLC_VLC_BRAKE_LAT_LIM_AFTER_OR : Configuration switch for enabling brake usage for lateral limiter also after override
                                                                                  @c_switch_part   CFG_VLC_VLC_NO_ACCEL_LIM_IN_DISENGAGE : Configuration switch to disable acceleration limitation in disengage mode
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_ACCEL_BAND_MODIFICATION : Configuration switch to use modification of acceleration band
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_BRAKE : Configuration switch for enabling brake usage for CC
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_BRAKES_FOR_LAT_LIM : Configuration switch for enabling brake usage for lateral limiter
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_LONG_ACCEL_CUSTOM_LIMIT : Configuration switch enabling custom limitation of longitudinal acceleration
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_MIN_MAX_DISENGAGEMENT : Configuration switch enabling ramping of required acceleration at disengagement
                                                                                  @c_switch_part   CFG_VLC_VLC_USE_HIGH_MIN_MAX_ACCEL_LIMIT : Configuration switch to raise and lower max and min acceleration limit respectively
                                                                                  @c_switch_part   CFG_VLC_FLIM : Configuration switch for FLIM support
                                                                                  @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing
                                                                                  @c_switch_full   CFG_VLC_CC : Configuration switch for enabling CC processing
                                                                                *************************************************************************************************************************/
static void VLC_COORDINATE_ACCEL(
    const cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output,
    cc_control_data_t* cc_control_data,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input) {
    acceleration_t commanded_acceleration;
    sint32 help_accel;

    commanded_acceleration = (acceleration_t)MAT_MIN(
        MAT_MAX(MAT_MIN((sint32)cc_control_data->VLC_ACCELERATION,
                        (sint32)acc_output->DISTANCE_CTRL_ACCEL_MIN),
                (sint32)cc_control_data->MINIMUM_ACCELERATION_LIMIT),
        (sint32)accel_control_data->MAXIMUM_ACCELERATION_LATERAL_LIMITED);

    /* Determine CommandedAcceleration (Evaluation of control states) */
    switch (driver_requests->CONTROL_STATE) {
        case Cc_cc_active:
        case Cc_cc_override:
            accel_control_data->MAXIMUM_COMMANDED_ACCELERATION =
                commanded_acceleration;
            accel_control_data->MINIMUM_COMMANDED_ACCELERATION =
                commanded_acceleration;
            break;

        case Cc_cc_decel_only:
            /* use case "Brake Only" */
            accel_control_data->MAXIMUM_COMMANDED_ACCELERATION =
                (acceleration_t)MAT_MIN((sint32)commanded_acceleration,
                                        (sint32)Cc_min_decel_brake_only);
            accel_control_data->MINIMUM_COMMANDED_ACCELERATION =
                accel_control_data->MIN_ALLOWED_ACCEL;

            break;

        case Cc_cc_disengage:
        case Cc_cc_off:
        case Cc_cc_engage:
        case Cc_cc_ready:
        default: {
            accel_control_data->MAXIMUM_COMMANDED_ACCELERATION =
                (acceleration_t)0;
            accel_control_data->MINIMUM_COMMANDED_ACCELERATION =
                (acceleration_t)0;
            break;
        }
    }

    // if MAX<MIN -> MIN=MAX
    accel_control_data->MINIMUM_COMMANDED_ACCELERATION =
        (acceleration_t)MAT_MIN(
            (sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION,
            (sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION);

    if ((driver_requests->CONTROL_STATE == Cc_cc_ready) ||
        (driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
        (driver_requests->CONTROL_STATE == Cc_cc_engage) ||
        (driver_requests->CONTROL_STATE == Cc_cc_active) ||
        (driver_requests->CONTROL_STATE == Cc_cc_override) ||
        (driver_requests->CONTROL_STATE == Cc_cc_decel_only)) {
        // limit to allowed acceleration range
        accel_control_data->MAXIMUM_COMMANDED_ACCELERATION =
            (acceleration_t)MAT_LIM(
                (sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION,
                (sint32)accel_control_data->MIN_ALLOWED_ACCEL,
                (sint32)accel_control_data->MAX_ALLOWED_ACCEL);
        accel_control_data->MINIMUM_COMMANDED_ACCELERATION =
            (acceleration_t)MAT_LIM(
                (sint32)accel_control_data->MINIMUM_COMMANDED_ACCELERATION,
                (sint32)accel_control_data->MIN_ALLOWED_ACCEL,
                (sint32)accel_control_data->MAX_ALLOWED_ACCEL);
    }

    help_accel =
        MAT_MAX((sint32)accel_control_data->MAXIMUM_COMMANDED_ACCELERATION,
                MAT_MAX((sint32)cc_control_data->VLC_ACCELERATION,
                        (sint32)cc_control_data->MINIMUM_ACCELERATION_LIMIT));

    // Determine ObjectEffective
    if (acc_output->DISTANCE_CTRL_ACCEL_MAX < (acceleration_t)help_accel) {
        accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE = TRUE;
    } else {
        accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE = FALSE;
    }

    if ((acc_output->AVLC_OUTPUT_STATUS.ALLOW_INIT == TRUE) ||
        (driver_requests->DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED == TRUE) ||
        (driver_requests->DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED == TRUE) ||
        (accel_control_data->ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE == TRUE) ||
        (driver_requests->CONTROL_STATE == Cc_cc_override)) {
        accel_control_data->ACCEL_STATUS.ALLOW_INIT = TRUE;
    } else {
        accel_control_data->ACCEL_STATUS.ALLOW_INIT = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_ACCELERATION_REQUEST_FILTERING */
static void VLC_ACCELERATION_REQUEST_FILTERING(
    const times_t cycle_time,
    cc_acceleration_control_data_t* accel_control_data,
    const cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output,
    cc_control_data_t* cc_control_data) {
    sint32 help;
    times_t pt1_filter_constant;
    acceleration_t min_req_accel_last_cycle, max_req_accel_last_cycle;
    acceleration_t min_requested_accel, max_requested_accel;

    if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
        (driver_requests->CONTROL_STATE == Cc_cc_off) ||
        (driver_requests->CONTROL_STATE == Cc_cc_decel_only) ||
        (driver_requests->CONTROL_STATE == Cc_cc_ready)) {
        /* No filtering required */
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_FILTERED =
            accel_control_data->MINIMUM_REQUESTED_ACCELERATION;
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_FILTERED =
            accel_control_data->MAXIMUM_REQUESTED_ACCELERATION;
    } else {
        /* Special initialization to allow jump of accel request with an applied
         * smoothing filter */
        if ((driver_requests->CONTROL_STATE_LAST_CYCLE !=
             driver_requests->CONTROL_STATE) &&
                ((driver_requests->CONTROL_STATE_LAST_CYCLE ==
                  Cc_cc_disengage) ||
                 (driver_requests->CONTROL_STATE_LAST_CYCLE == Cc_cc_ready) ||
                 (driver_requests->CONTROL_STATE_LAST_CYCLE == Cc_cc_engage) ||
                 ((driver_requests->CONTROL_STATE_LAST_CYCLE == Cc_cc_active) &&
                  (driver_requests->CONTROL_STATE != Cc_cc_override))) ||
            /* Drive off initialization */
            (driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM == TRUE)) {
            /* Save last requested acceleration for initialization */
            min_req_accel_last_cycle =
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
            max_req_accel_last_cycle =
                accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
        } else {
            /* Save last requested acceleration (filtered)*/
            min_req_accel_last_cycle =
                accel_control_data->MINIMUM_REQUESTED_ACCELERATION_FILTERED;
            max_req_accel_last_cycle =
                accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_FILTERED;
        }

        min_requested_accel =
            accel_control_data->MINIMUM_REQUESTED_ACCELERATION;
        max_requested_accel =
            accel_control_data->MAXIMUM_REQUESTED_ACCELERATION;

        /* Modify the smooting filter time constant in critical situation to
         * allow quick reaction */
        pt1_filter_constant = (times_t)MAT_CALCULATE_PARAM_VALUE1D(
            (const sint16*)criticality_accel_filter_time,
            accel_filter_time_time_points,
            (sint16)(acc_output->SITUATION_CLASS.CRITICALITY * Scale_100 /
                     Confidence_s));

        /* Apply filtering */
        min_requested_accel = (acceleration_t)MAT_PT1_FILTER(
            cycle_time, pt1_filter_constant, (sint32)min_requested_accel,
            (sint32)min_req_accel_last_cycle);
        max_requested_accel = (acceleration_t)MAT_PT1_FILTER(
            cycle_time, pt1_filter_constant, (sint32)max_requested_accel,
            (sint32)max_req_accel_last_cycle);

        /*write gradient of acceleration request */
        help = (sint32)((min_requested_accel - min_req_accel_last_cycle) *
                        (sint32)Factor_s);
        help /= (sint32)cycle_time;
        cc_control_data->ACCELERATION_REQUEST_GRAD = (gradient_t)help;

        /*write output data */
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_FILTERED =
            min_requested_accel;
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_FILTERED =
            max_requested_accel;
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_DETERMINE_CRUISE_ACCEL */
static void VLC_DETERMINE_CRUISE_ACCEL(
    const cc_input_data_t* input,
    const cart_das_input_data_t* das_input,
    const cc_driver_requests_t* driver_requests,
    cc_control_data_t* cc_control_data) {
    sint32 delta_speed;
    sint32 set_speed;
    sint32 cruise_accel;
    sint32 help_qoud, help_accel, accel_gain, accel_quod;

    static sint32 delta_speed_integ = 0;

    set_speed =
        driver_requests->VLC_SETSPEED * (sint32)Velocity_s + Cc_offset_setspeed;

    /*use  speedometer_vehicle_speed_offset for delta_speed_calculation*/
    delta_speed =
        (sint32)(set_speed - driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET);

    delta_speed *= (sint32)Factor_s;

    /* Convert to m/s */
    if (input->VEHICLE_STATUS.SPEED_UNIT_KMH == TRUE) {
        delta_speed /= (sint32)Speed_conv_factor_kmh;
    } else {
        delta_speed /= (sint32)Speed_conv_factor_mph;
    }

    if (driver_requests->CONTROL_STATE == Cc_cc_active &&
        driver_requests->CONTROL_STATE_LAST_CYCLE != Cc_cc_active) {
        delta_speed_integ = 0;
    } else {
        delta_speed_integ += delta_speed;
    }

    if (delta_speed_integ > Cc_delta_speed_integ_upper_limit) {
        delta_speed_integ = Cc_delta_speed_integ_upper_limit;
    } else if (delta_speed_integ < Cc_delta_speed_integ_lower_limit) {
        delta_speed_integ = Cc_delta_speed_integ_lower_limit;
    }

    /* Determine cc acceleration depending on differential velocity (Look up
     * table) */
    cc_control_data->VLC_ACCELERATION_P_PART = MAT_CALCULATE_PARAM_VALUE1D(
        Cc_accel_curve, (uint16)Cc_curve_points, (sint16)delta_speed);

    cc_control_data->VLC_ACCELERATION_I_PART = MAT_CALCULATE_PARAM_VALUE1D(
        Cc_accel_curve_delta_speed_integ,
        (uint16)Cc_accel_curve_delta_speed_integ_points,
        (sint16)delta_speed_integ);

    cruise_accel = cc_control_data->VLC_ACCELERATION_P_PART +
                   cc_control_data->VLC_ACCELERATION_I_PART;

    /*Modify cruise acceleration in low speed range*/

    accel_gain = MAT_CALCULATE_PARAM_VALUE1D(Cc_accel_gain_curve,
                                             (uint16)Cc_accel_gain_curve_points,
                                             das_input->VEHICLE_SPEED);

    if ((accel_gain >= Cc_accel_limit) && (cruise_accel > 0)) {
        help_qoud = cruise_accel;
        help_qoud *= Factor_s;
        help_qoud /= Cc_accel_limit;

        help_accel = 1;
        help_accel *= Factor_s;
        help_accel -= help_qoud;
        help_accel *= accel_gain;
        help_accel /= Acceleration_s;
        help_accel += help_qoud;

        accel_quod = 1;
        accel_quod *= Factor_s;
        accel_quod *= Factor_s;
        accel_quod /= help_accel;

        cruise_accel *= accel_gain;
        cruise_accel /= Acceleration_s;
        cruise_accel *= accel_quod;
        cruise_accel /= Factor_s;
    }

    /*limit cruise acceleration over the velocity dependent on vehicle
     * characteristics */
    cc_control_data->MINIMUM_ACCELERATION_LIMIT = Cc_decel_limit;
    cc_control_data->MAXIMUM_ACCELERATION_LIMIT = MAT_CALCULATE_PARAM_VALUE1D(
        Cc_max_accel_curve, (uint16)Cc_max_accel_curve_points,
        das_input->VEHICLE_SPEED);

    cruise_accel =
        MAT_LIM(cruise_accel, cc_control_data->MINIMUM_ACCELERATION_LIMIT,
                cc_control_data->MAXIMUM_ACCELERATION_LIMIT);

    cc_control_data->VLC_ACCELERATION = (acceleration_t)(cruise_accel);

    if (driver_requests->DRIVER_REQUEST_STATUS.TIMED_INCREMENT == TRUE) {
        cc_control_data->VLC_ACCELERATION =
            (acceleration_t)MAT_MAX((sint32)cc_control_data->VLC_ACCELERATION,
                                    (sint32)Cc_max_decel_during_accel_mode);
    } else {
        if (driver_requests->DRIVER_REQUEST_STATUS.TIMED_DECREMENT == TRUE) {
            cc_control_data->VLC_ACCELERATION = (acceleration_t)MAT_MIN(
                (sint32)cc_control_data->VLC_ACCELERATION,
                (sint32)Cc_max_accel_during_decel_mode);
        }
    }
    if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
        (driver_requests->CONTROL_STATE == Cc_cc_off) ||
        (driver_requests->CONTROL_STATE == Cc_cc_decel_only) ||
        (driver_requests->CONTROL_STATE == Cc_cc_ready)) {
        cc_control_data->VLC_ACCELERATION = MAT_CALCULATE_PARAM_VALUE1D(
            Cc_max_accel_curve, (uint16)Cc_max_accel_curve_points,
            driver_requests->MIN_VLC_SETSPEED * Scale_1000 /
                Speed_conv_factor_kmh);
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_SET_ACCEL_REQUEST */ /*!

                                                                                 @brief           Sets the minimum and maximum acceleration requests to the longitudinal dynamics management

                                                                                 @description     If acceleration bands are to be modified, min and max acceleration requests are limited within the
                                                                                                  positive and negative comfort bands from the last cycle values. The requests are then ramped as per
                                                                                                  defined gradients. If required, the filtered acceleration requests are used as outputs. Accelerations
                                                                                                  are set as outputs for the longitudinal dynamics management (via MIN_REQ_ACCEL and MAX_REQ_ACCEL).

                                                                                 @return          -

                                                                                 @param[in]       cycle_time : cycle time of VLC_VEH cycle [0u ... 1000u] ms
                                                                                 @param[in,out]   accel_control_data : Pointer to CC acceleration control data [cc_acceleration_control_data_t as per cc_ext.h]
                                                                                                   MIN_ACCEL_NEG_BAND : Minimum negative acceleration band [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MAX_ACCEL_NEG_BAND : Maximum negative acceleration band [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MINIMUM_REQUESTED_ACCELERATION : Minimum requested acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MAXIMUM_REQUESTED_ACCELERATION : Maximum requested acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MINIMUM_REQUESTED_ACCELERATION_FILTERED : Minimum requested filtered acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MAXIMUM_REQUESTED_ACCELERATION_FILTERED : Maximum requested filtered acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                 @param[in,out]   das_output : Pointer to output data from driver assistance system to longitudinal dynamics management [cart_das_output_data_t as per cart_ext.h]
                                                                                                   MIN_REQ_ACCEL : Minimum required acceleration output [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                   MAX_REQ_ACCEL : Maximum required acceleration output [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                 @param[in]       driver_requests : Pointer to driver request input structure [cc_driver_requests_t as per cc_ext.h]
                                                                                                   CONTROL_STATE : Variable that reflects the state of the HMI state machine [full range of datatype cc_control_state_t as in cc_ext.h]

                                                                                 @glob_in         -
                                                                                 @glob_out        -

                                                                                 @c_switch_part   CFG_VLC_VLC_USE_ACCEL_BAND_MODIFICATION : Configuration switch to use modification of acceleration band
                                                                                 @c_switch_part   CFG_VLC_VLC_SMOOTH_ACCEL_REQUEST : Switch to configure filter to prevent oscillations and other discontinuous waveforms
                                                                                 @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing
                                                                                 @c_switch_full   CFG_VLC_CC : Configuration switch for enabling CC processing

                                                                               *************************************************************************************************************************/
static void VLC_SET_ACCEL_REQUEST(
    const times_t cycle_time,
    cc_acceleration_control_data_t* accel_control_data,
    cart_das_output_data_t* das_output,
    const cc_driver_requests_t* driver_requests) {
    /*write filtered accel_control_data->(MAX/MIN)_ACCEL_(POS/NEG)_BAND to
     * CARTRONIC_DATA */
    das_output->MIN_REQ_ACCEL =
        accel_control_data->MINIMUM_REQUESTED_ACCELERATION_FILTERED;
    das_output->MAX_REQ_ACCEL =
        accel_control_data->MAXIMUM_REQUESTED_ACCELERATION_FILTERED;

    _PARAM_UNUSED(cycle_time);
    _PARAM_UNUSED(driver_requests);
}

/*************************************************************************************************************************
  Functionname:    VLC_COORDINATE_STANDSTILL */ /*!

                                                                             @brief
                                                                           Sets
                                                                           the
                                                                           standstill
                                                                           management
                                                                           bits
                                                                           from
                                                                           and
                                                                           to
                                                                           the
                                                                           longitudinal
                                                                           dynamics
                                                                           management

                                                                             @description
                                                                           Standstill
                                                                           and
                                                                           driveoff
                                                                           conditions
                                                                           determined
                                                                           based
                                                                           on
                                                                           driver
                                                                           inputs
                                                                           and
                                                                           traffic
                                                                           situation
                                                                                              and set as outputs for longitudinal dynamics management.

                                                                             @return
                                                                           -

                                                                             @param[in]
                                                                           acc_output
                                                                           :
                                                                           Pointer
                                                                           to
                                                                           ACC
                                                                           output
                                                                           [VLC_acc_output_data_t
                                                                           as
                                                                           per
                                                                           Rte_Type.h]
                                                                                               SITUATION_CLASS.SITUATION : Classifier of the current traffic situation [full range of datatype acc_situation_class_t as in Rte_Type.h]
                                                                                               DISTANCE_CTRL_ACCEL_MIN : Minimum acceleration for intrusion distance [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                               DISTANCE_CTRL_ACCEL_MAX : Maximum acceleration for intrusion distance [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                             @param[in]
                                                                           das_input
                                                                           :
                                                                           Pointer
                                                                           to
                                                                           input
                                                                           from
                                                                           longitudinal
                                                                           dynamics
                                                                           management
                                                                           to
                                                                           driver
                                                                           assistance
                                                                           system
                                                                           [cart_das_input_data_t
                                                                           as
                                                                           per
                                                                           cart_ext.h]
                                                                                               LODM_STAT.STANDSTILL : Boolean indicating if vehicle standstill is detected [TRUE, FALSE]
                                                                                               BRAKE_STAT.PEDAL_INIT_TRAVEL : Boolean indicating if brake pedal is pressed [TRUE, FALSE]

                                                                             @param[in,out]
                                                                           das_output
                                                                           :
                                                                           Pointer
                                                                           to
                                                                           output
                                                                           data
                                                                           from
                                                                           driver
                                                                           assistance
                                                                           system
                                                                           to
                                                                           longitudinal
                                                                           dynamics
                                                                           management
                                                                           [cart_das_output_data_t
                                                                           as
                                                                           per
                                                                           cart_ext.h]
                                                                                               DAS_STAT.DAS_STAND_STILL : Boolean output indicating if vehicle standstill is detected [TRUE, FALSE]
                                                                                               DAS_STAT.DAS_DRIVE_OFF : Boolean output indicating if driveoff is confirmed [TRUE, FALSE]
                                                                             @param[in,out]
                                                                           cc_status
                                                                           :
                                                                           Pointer
                                                                           to
                                                                           Cruisecontrol
                                                                           status
                                                                           [cc_status_t
                                                                           as
                                                                           per
                                                                           cc_ext.h]
                                                                                               VLC_DRIVER_REQUESTS.CONTROL_STATE : Variable that reflects the state of the HMI state machine [full range of datatype cc_control_state_t as in cc_ext.h]
                                                                                               VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM : Boolean indicating vehicle drive off [TRUE, FALSE]
                                                                                               VLC_ACCEL_CONTROL_DATA.MINIMUM_COMMANDED_ACCELERATION : Minimum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                               VLC_ACCEL_CONTROL_DATA.MAXIMUM_COMMANDED_ACCELERATION : Maximum commanded acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                               VLC_ACCEL_CONTROL_DATA.MAX_CUSTOM_ALLOWED_ACCEL : Maximum custom allowed acceleration [-30000u ...+30000u] m/s2 (with factor Acceleration_s)

                                                                             @glob_in
                                                                           -
                                                                             @glob_out
                                                                           -

                                                                             @c_switch_part
                                                                           CFG_VLC_FSRACC
                                                                           :
                                                                           Configuration
                                                                           switch
                                                                           enabling
                                                                           Full
                                                                           Speed
                                                                           Range
                                                                           Adaptive
                                                                           Cruise
                                                                           Control
                                                                           functionality
                                                                           with
                                                                           stop
                                                                           and
                                                                           go
                                                                           feature
                                                                             @c_switch_full
                                                                           VLC_CFG_LONG_PROCESSING
                                                                           :
                                                                           Configuration
                                                                           switch
                                                                           for
                                                                           enabling
                                                                           VLC_LONG
                                                                           processing
                                                                             @c_switch_full
                                                                           CFG_VLC_CC
                                                                           :
                                                                           Configuration
                                                                           switch
                                                                           for
                                                                           enabling
                                                                           CC
                                                                           processing

                                                                           *************************************************************************************************************************/
static void VLC_COORDINATE_STANDSTILL(const VLC_acc_output_data_t* acc_output,
                                      const cart_das_input_data_t* das_input,
                                      cart_das_output_data_t* das_output,
                                      cc_status_t* cc_status) {
    das_output->DAS_STAT.DAS_DRIVE_OFF_LAST_CYC =
        das_output->DAS_STAT.DAS_DRIVE_OFF;

    if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) {
        if ((das_input->LODM_STAT.STANDSTILL == TRUE) &&
            ((acc_output->SITUATION_CLASS.SITUATION ==
              Acc_sit_class_stop) /*&& driver was informed*/
             ||
             /*Prevent acceleration request if driver presses brakes after
                engagement in standstill*/
             (das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL == TRUE))) {
            das_output->DAS_STAT.DAS_STAND_STILL = TRUE;
        } else {
            /* There is no use of DAS_DRIVE_OFF in Cartronic interface */
            if ((das_output->DAS_STAT.DAS_DRIVE_OFF == TRUE) &&
                (das_output->DAS_STAT.DAS_STAND_STILL == FALSE)) {
                das_output->DAS_STAT.DAS_DRIVE_OFF = FALSE;
            }

            if (das_input->LODM_STAT.STANDSTILL == FALSE) {
                das_output->DAS_STAT.DAS_STAND_STILL = FALSE;
            }
        }

        if (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
                .DRIVE_OFF_CONFIRM == TRUE) {
            /* additional check for (acc_output->SITUATION_CLASS.SITUATION
             * == Acc_sit_class_go) possible */
            das_output->DAS_STAT.DAS_STAND_STILL = FALSE;
            das_output->DAS_STAT.DAS_DRIVE_OFF = TRUE;
        }

        /*vehicle shall be kept in standstill mode*/
        if ((das_input->LODM_STAT.STANDSTILL == TRUE) &&
            (das_output->DAS_STAT.DAS_STAND_STILL == TRUE)) {
            /*
            The vehicle should be hold in stand still.
            DAS_STAT.DAS_STAND_STILL will be SET if LODM is standstill and
            sit_class_stop is set

            LODM shall gurantee standstill!*/
            /* MAX_CUSTOM_ALLOWED_ACCEL is used to allow custom logic for
             * stand still request requirements */
            cc_status->VLC_ACCEL_CONTROL_DATA
                .MAXIMUM_COMMANDED_ACCELERATION = (acceleration_t)MAT_MIN(
                MAT_MIN(
                    cc_status->VLC_ACCEL_CONTROL_DATA.MAX_CUSTOM_ALLOWED_ACCEL,
                    Acc_standstill_accel_req),
                (sint32)acc_output->DISTANCE_CTRL_ACCEL_MAX);
            cc_status->VLC_ACCEL_CONTROL_DATA
                .MINIMUM_COMMANDED_ACCELERATION = (acceleration_t)MAT_MIN(
                MAT_MIN(
                    cc_status->VLC_ACCEL_CONTROL_DATA.MAX_CUSTOM_ALLOWED_ACCEL,
                    Acc_standstill_accel_req),
                (sint32)acc_output->DISTANCE_CTRL_ACCEL_MIN);
        }
    } else {
        das_output->DAS_STAT.DAS_STAND_STILL = FALSE;
        das_output->DAS_STAT.DAS_DRIVE_OFF = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_DETECT_TRANSITION_AVLC_TO_CC */ /*!

                                                                      @brief
                                                                    Detects
                                                                    transition
                                                                    from ACC
                                                                    (control
                                                                    with object)
                                                                    to
                                                                                       CC (control with no object) at low speed range

                                                                      @description
                                                                    When
                                                                    effective
                                                                    object is no
                                                                    longer
                                                                    available,
                                                                    there is
                                                                    transition
                                                                    from ACC to
                                                                    CC,
                                                                                       after ensuring that required time has elapsed. This detection is only applicable
                                                                                       for low speeds. As long as timer does not expire, the accel gradient will be reduced.

                                                                      @return -

                                                                      @param[in]
                                                                    cycle_time :
                                                                    cycle time
                                                                    of VLC_VEH
                                                                    cycle [0u
                                                                    ... 1000u]
                                                                    ms
                                                                      @param[in]
                                                                    das_input :
                                                                    Pointer to
                                                                    input from
                                                                    longitudinal
                                                                    dynamics
                                                                    management
                                                                    to driver
                                                                    assistance
                                                                    system
                                                                    [cart_das_input_data_t
                                                                    as per
                                                                    cart_ext.h]
                                                                                        VEHICLE_SPEED : Ego speed with factor Velocity_s [-30000u ... 30000u] cm/s
                                                                      @param[in,out]
                                                                    cc_status :
                                                                    Pointer to
                                                                    Cruisecontrol
                                                                    status
                                                                    [cc_status_t
                                                                    as per
                                                                    cc_ext.h]
                                                                                        VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE : Boolean, if TRUE indicating effective OOI for ACC [TRUE, FALSE]
                                                                                        VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE : Boolean indicating if ACC/CC active  [TRUE, FALSE]
                                                                                        VLC_CONTROL_DATA.VLC_AVLC_TO_VLC_TRANSITION : Boolean, if TRUE indicates transition from ACC (object-control) to CC [TRUE, FALSE]
                                                                      @glob_in -
                                                                      @glob_out
                                                                    object_effective_last_cycle
                                                                    : Boolean
                                                                    indicating
                                                                    effective
                                                                    OOI for
                                                                    previous
                                                                    cycle [TRUE,
                                                                    FALSE]
                                                                                       transion_timer : Timer to ensure timed transition from ACC to CC [0u ... 10000u]ms

                                                                      @c_switch_part
                                                                    -
                                                                      @c_switch_full
                                                                    VLC_CFG_LONG_PROCESSING
                                                                    :
                                                                    Configuration
                                                                    switch for
                                                                    enabling
                                                                    VLC_LONG
                                                                    processing
                                                                      @c_switch_full
                                                                    CFG_VLC_CC :
                                                                    Configuration
                                                                    switch for
                                                                    enabling CC
                                                                    processing
                                                                    *************************************************************************************************************************/
static void VLC_DETECT_TRANSITION_AVLC_TO_CC(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    cc_status_t* cc_status) {
#define MAX_VALID_SPEED_FOR_DETECTION (velocity_t)1600 /* 16 m/s */
#define TIME_OF_TRANSITION (times_t)3000

    static boolean object_effective_last_cycle = FALSE;
    static times_t transion_timer = (times_t)0;

    /*! if no object and functon is active, cc mode */
    if ((cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
         FALSE) &&
        (cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
             .SELECTED_FUNCTION_ACTIVE == TRUE)) {
        /*! last cycle object was available and vehicle at low speed */
        if ((object_effective_last_cycle == TRUE) &&
            (das_input->VEHICLE_SPEED < MAX_VALID_SPEED_FOR_DETECTION)) {
            /*! set time of transion, the accel gradient will be reduced */
            transion_timer = TIME_OF_TRANSITION;
        }
    } else {
        /* OBJECT_EFFECTIVE == TRUE or Function OFF */
        transion_timer = (times_t)0;
    }

    /*! if timer not expires, the accel gradient will be reduced */
    if (transion_timer >= cycle_time) {
        cc_status->VLC_CONTROL_DATA.VLC_AVLC_TO_VLC_TRANSITION = TRUE;
    } else {
        cc_status->VLC_CONTROL_DATA.VLC_AVLC_TO_VLC_TRANSITION = FALSE;
    }

    if (transion_timer >= cycle_time) {
        transion_timer -= cycle_time;
    }

    /*! update object_effective_last_cycle */
    object_effective_last_cycle =
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE;
}

/*************************************************************************************************************************
  Functionname:    VLC_COORDINATOR */
static void VLC_COORDINATOR(const times_t cycle_time,
                            const VLC_acc_output_data_t* acc_output,
                            const cart_das_input_data_t* das_input,
                            cart_das_output_data_t* das_output,
                            cc_status_t* cc_status,
                            const cc_input_data_t* input) {
    VLC_COORDINATE_ACCEL(&cc_status->VLC_DRIVER_REQUESTS, acc_output,
                         &cc_status->VLC_CONTROL_DATA,
                         &cc_status->VLC_ACCEL_CONTROL_DATA, das_input);

    VLC_COORDINATE_STANDSTILL(acc_output, das_input, das_output, cc_status);

    cc_status->LODM_STAT_LAST_CYCLE = das_input->LODM_STAT;

    VLC_DETECT_TRANSITION_AVLC_TO_CC(cycle_time, das_input, cc_status);

    VLC_LIMIT_ACCEL_CHANGE_RATE(cycle_time, &cc_status->VLC_DRIVER_REQUESTS,
                                &cc_status->VLC_CONTROL_DATA,
                                &cc_status->VLC_ACCEL_CONTROL_DATA, das_input,
                                das_output, input, acc_output);

    VLC_ACCELERATION_REQUEST_FILTERING(
        cycle_time, &cc_status->VLC_ACCEL_CONTROL_DATA,
        &cc_status->VLC_DRIVER_REQUESTS, acc_output,
        &cc_status->VLC_CONTROL_DATA);
}

/*************************************************************************************************************************
  Functionname:    VLC_COMMAND_ACCEL */ /*!

                                                                                     @brief           Call all functions related to the determination of the commanded acceleration

                                                                                     @description     Commanded acceleration calculated and arbitrated based on inputs from cruise control,
                                                                                                      speed limit assist, ACC object control, whichever applicable. Requested acceleration
                                                                                                      calculated from commanded acceleration based on limitations from lateral acceleration
                                                                                                      and comfort requirements (gradients and bands). MIN_REQ_ACCEL and MAX_REQ_ACCEL sent
                                                                                                      as output to LODM

                                                                                     @return          -

                                                                                     @param[in]       cycle_time : cycle time of VLC_VEH cycle [0u ... 1000u] ms
                                                                                     @param[in]       input : Pointer to cc_input_data_t type structure containing input data to CC [cc_input_data_t as per cc_ext.h]
                                                                                     @param[in]       das_input : Pointer to input from longitudinal dynamics management to driver assistance system [cart_das_input_data_t as per cart_ext.h]
                                                                                                       VEHICLE_SPEED : Ego speed with factor Velocity_s [-30000u ... 30000u] cm/s
                                                                                     @param[in]       acc_output : Pointer to ACC output [VLC_acc_output_data_t  as per Rte_Type.h]
                                                                                     @param[in,out]   das_output : Pointer to output data from driver assistance system to longitudinal dynamics management [cart_das_output_data_t as per cart_ext.h]
                                                                                                       MIN_REQ_ACCEL : Minimum required acceleration output [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                                       MAX_REQ_ACCEL : Maximum required acceleration output [-30000u ...+30000u] m/s2 (with factor Acceleration_s)
                                                                                     @param[in,out]   cc_status : Pointer to Cruisecontrol status [cc_status_t as per cc_ext.h]
                                                                                                       VLC_DRIVER_REQUESTS : Data that reflects the driver's request intention [cc_driver_requests_t as per cc_ext.h]
                                                                                                       VLC_SLA_HMI_DATA : Pointer to data struct SLA HMI [t_SLA_HMI_Data as per cc_ext.h]
                                                                                                       VLC_ACCEL_CONTROL_DATA : Pointer to CC acceleration control data [cc_acceleration_control_data_t as per cc_ext.h]
                                                                                                       VLC_CONTROL_DATA : Pointer to CC control data [cc_control_data_t as per cc_ext.h]
                                                                                                       VLC_CONTROL_DATA.VLC_SLA_CTRL_DATA : Pointer to SLA control data [t_SLA_Ctrl_Data as per cc_ext.h]

                                                                                     @glob_in
                                                                                     @glob_out

                                                                                     @c_switch_part   VLC_CFG_LONG_CTRL_SLA_SUPPORT : Switch for enabling SLA Support functions
                                                                                     @c_switch_part   CFG_VLC_LIM : Configuration switch for enabling limiter function
                                                                                     @c_switch_part   CFG_VLC_PLIM : Configuration switch for enabling permanent limiter function
                                                                                     @c_switch_part   CFG_VLC_VLC_USE_LAT_LIM : Configuration switch for enabling lateral acceleration limitations
                                                                                     @c_switch_part   CFG_VLC_VLC_USE_ACCEL_BAND_MODIFICATION : Configuration switch to use modification of acceleration band
                                                                                     @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing
                                                                                     @c_switch_full   CFG_VLC_CC : Configuration switch for enabling CC processing
                                                                                   *************************************************************************************************************************/
void VLC_COMMAND_ACCEL(const times_t cycle_time,
                       const cc_input_data_t* input,
                       const cart_das_input_data_t* das_input,
                       const VLC_acc_output_data_t* acc_output,
                       const t_CamLaneInputData* pCamLaneData,
                       cart_das_output_data_t* das_output,
                       cc_status_t* cc_status,
                       PACCInfo* p_pacc_info) {
    VLC_DETERMINE_CRUISE_ACCEL(input, das_input,
                               &cc_status->VLC_DRIVER_REQUESTS,
                               &cc_status->VLC_CONTROL_DATA);

    VLC_LIMIT_LATERAL_ACCEL(cycle_time, input, acc_output, das_input,
                            pCamLaneData, cc_status, p_pacc_info);

    VLC_LIMIT_DECEL(cc_status);
    VLC_COORDINATOR(cycle_time, acc_output, das_input, das_output, cc_status,
                    input);

    VLC_SET_ACCEL_REQUEST(cycle_time, &cc_status->VLC_ACCEL_CONTROL_DATA,
                          das_output, &cc_status->VLC_DRIVER_REQUESTS);

    /*if MAX<MIN -> MIN=MAX*/
    das_output->MIN_REQ_ACCEL = (acceleration_t)MAT_MIN(
        (sint32)das_output->MAX_REQ_ACCEL, (sint32)das_output->MIN_REQ_ACCEL);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */