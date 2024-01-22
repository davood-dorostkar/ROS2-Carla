/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
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
#include "cc.h"
#include "cc_par.h"
#include "acc_par.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"
#include "ACCSM.h"

#define Speed_round_up (Velocity_s / 2)
#define Switch_ACCSM_Gen_Code \
    1  // if this flag is set to 1 ,the ACC State Machine will change to
       // Simulink generated code

static void VLC_DETERMINE_SET_SPEED(const cc_input_data_t* input_data,
                                    const cc_error_data_t* error_data,
                                    cc_driver_requests_t* driver_requests);
static void VLC_DETERMINE_ENGAGEMENT_CONDITIONS(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    const cc_input_data_t* input,
    cc_acceleration_control_data_t* accel_control_data,
    cc_driver_requests_t* driver_requests,
    cc_status_t* cc_status,
    const cc_driver_controls_t* driver_controls,
    const VLC_acc_output_data_t* situation_output);
static void VLC_DETERMINE_SPEED_LIMITS(
    const cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    const cc_input_data_t* input,
    cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output);
static void VLC_DETERMINE_CONTROL_STATE(
    const cc_driver_controls_t* driver_controls,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    cc_error_data_t* error_data,
    cc_driver_requests_t* driver_requests,
    cart_das_output_data_t* das_output,
    const cc_input_data_t* input_data);
static void VLC_ONLY_DETERMINE_CONTROL_STATE(
    const cc_driver_controls_t* driver_controls,
    cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    cc_error_data_t* error_data,
    cc_driver_requests_t* driver_requests,
    cart_das_output_data_t* das_output,
    const cc_input_data_t* input_data);

/******************************************************************************
  @fn             VLC_HMI_INIT */ /*!

                                                @description    This function
                                              initializes all data at system
                                              start.
                                                                This function
                                              needs to be called once before
                                              EXEC_CC
                                                                is called

                                                @param[in]      input_data
                                              pointer to the input data (most
                                              notably the
                                                                            VEHICLE_STATUS.SPEED_UNIT_KMH
                                              field) to use for init

                                                @param[out]     cc_status  The
                                              cc_status_t type structure the HMI
                                                                           related
                                              fields of which shall be
                                              initialized

                                                @return         void

                                              *****************************************************************************/
void VLC_HMI_INIT(const cc_input_data_t* input_data, cc_status_t* cc_status) {
    cc_status->LODM_STAT_LAST_CYCLE.DAS_RESET = FALSE;
    cc_status->LODM_STAT_LAST_CYCLE.DAS_ENABLE = FALSE;
    cc_status->LODM_STAT_LAST_CYCLE.DAS_INHIBIT = FALSE;
    cc_status->LODM_STAT_LAST_CYCLE.DAS_MODE = 0;
    cc_status->LODM_STAT_LAST_CYCLE.DAS_SHUTOFF_ACQ = FALSE;
    cc_status->LODM_STAT_LAST_CYCLE.DC_LIM_ACCEL = FALSE;
    cc_status->LODM_STAT_LAST_CYCLE.DC_LIM_DECEL = FALSE;
    cc_status->VLC_DECEL_ONLY_MODE_CNT = (uint32)0;
    cc_status->VLC_SPEED_UNIT_LAST_CYCLE =
        (uint8)input_data->VEHICLE_STATUS.SPEED_UNIT_KMH;
    cc_status->VLC_ABS_ACT_CNT = 0u;
    cc_status->VLC_TCS_ACT_CNT = 0u;
    cc_status->VLC_ESP_ACT_CNT = 0u;
}

static void VLC_DRIVE_OFF_CONDITION(
    const cart_das_input_data_t* das_input,
    cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* situation_output) {
    if ((driver_requests->CONTROL_MODE == Cc_standstill_mode)) {
        driver_requests->ENGAGEMENT_CONDITIONS.CONTROL_TO_RELEVANT_OBJECT =
            TRUE;
        if ((driver_requests->CONTROL_STATE == Cc_cc_active) &&
            (das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL == FALSE) &&
            // (situation_output->SITUATION_CLASS.SITUATION !=
            //  Acc_sit_class_stop) &&   // changed for drive off after 3 min
            (VLCVEH_pLongCtrlResp->KinCtrlDynInput.seatbelt_state ==
             SEATBELT_DRIVER_CLOSED) /*SW18*/
        ) {
            /* Driver not braking and situation indicates vehicle ahead moving
             * (drive off) */
            driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE = TRUE;
        } else {
            /* Either driver braking or vehicle ahead still stopped */
            driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE = FALSE;
        }
    } else {
        /* Either not in active state or not in stand-still. Either way a drive
        off is NOT possible */
        driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE = FALSE;
    }
}

static void VLC_END_LIM_DISENGAGEMENT(
    const cart_das_input_data_t* das_input,
    cc_driver_requests_t* driver_requests,
    cc_status_t* cc_status,
    cc_acceleration_control_data_t* accel_control_data) {
    if ((das_input->LODM_STAT.DAS_SHUTOFF_ACQ == TRUE) &&
        (cc_status->LODM_STAT_LAST_CYCLE.DAS_SHUTOFF_ACQ == FALSE)) {
        driver_requests->ENGAGEMENT_CONDITIONS.END_LIM_DISENGAGEMENT = TRUE;
    } else {
        /*System will at least disengage if Acc_min/max_accel_disengage is
         * reached*/
        if (accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE == FALSE) {
            driver_requests->ENGAGEMENT_CONDITIONS.END_LIM_DISENGAGEMENT = TRUE;
        }
    }
}

/******************************************************************************
  @fn             VLC_DETERMINE_ENGAGEMENT_CONDITIONS */ /*!

                         @description    Determine whether the conditions are
                       fulfilled to
                                         transition CONTROL_STATE.

                         @param[in]      cycle_time
                         @param[in]      das_input       in
                       das_input->LODM_STAT.DAS_SHUTOFF_ACQ           Bit from
                       LDM that marks "disengagement ramp" active

                         @param[in]      input           in
                       input.SPEEDOMETER_VEHICLE_SPEED             Vehicle speed
                       displayed by speedometer
                                                         in
                       input.SPEEDOMETER_VEHICLE_SPEED_OFFSET      Vehicle speed
                       displayed by speedometer with offset added as a function
                       of acceleration
                                                         in input.VEHICLE_STATUS
                       This variable reflects the status of the vehicle
                                                         in
                       input->ACCELERATION_GRADIENT                Gradient of
                       vehicle acceleration
                         @param[out]     accel_control_data
                         @param[out]     driver_requests out
                       driver_requests.ENGAGEMENT_CONDITIONS              Bit
                       field for all possible CONTROL_STATE transitions
                         @param[in]      cc_status
                         @param[in]      driver_controls in
                       driver_controls.PERMANENT_LIMITER_SETSPEED
                       Permanent speed limit set by driver
                         @param[in]      situation_output
                         @return         void

                       *****************************************************************************/
static void VLC_DETERMINE_ENGAGEMENT_CONDITIONS(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    const cc_input_data_t* input,
    cc_acceleration_control_data_t* accel_control_data,
    cc_driver_requests_t* driver_requests,
    cc_status_t* cc_status,
    const cc_driver_controls_t* driver_controls,
    const VLC_acc_output_data_t* situation_output) {
    /* Reset all Conditions */
    driver_requests->ENGAGEMENT_CONDITIONS.ACCEPT_VLC_ENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.ACCEPT_LIM_ENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_ENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT_RAMP = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_DISENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.VLC_DECEL_ONLY = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.VLC_END_DECEL_ONLY = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.LIM_DISENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.END_LIM_DISENGAGEMENT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.END_PERM_LIM = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.VLC_ENGAGEMENT_STAT = FALSE;
    driver_requests->ENGAGEMENT_CONDITIONS.CONTROL_TO_RELEVANT_OBJECT = FALSE;

    /*! Determ max allowed vehicle acceleration and deceleration */
    accel_control_data->MIN_ALLOWED_ACCEL = MAT_CALCULATE_PARAM_VALUE1D(
        Cc_max_vehicle_decel, (uint16)Cc_max_vehicle_decel_points,
        das_input->VEHICLE_SPEED);
    accel_control_data->MAX_ALLOWED_ACCEL = MAT_CALCULATE_PARAM_VALUE1D(
        Cc_max_vehicle_accel, (uint16)Cc_max_vehicle_accel_points,
        das_input->VEHICLE_SPEED);

    VLC_DRIVE_OFF_CONDITION(das_input, driver_requests, situation_output);

    /* Can the HoldSpeed-Function be engaged? */
    if ((input->INHIBIT.VLC_INHIBIT_ENGAGEMENT == FALSE) &&
        ((input->SPEEDOMETER_VEHICLE_SPEED <=
          ((speedometer_speed_t)driver_requests->MAX_VLC_SPEED *
           Speedo_speed_s)) ||
         (driver_requests->MAX_VLC_SPEED == Cc_speed_default_value)) &&
        (input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == FALSE) &&
        (input->INHIBIT.VLC_DISENGAGEMENT_RAMP == FALSE) &&
        (input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP == FALSE) &&
        (input->INHIBIT.VLC_DISENGAGEMENT_DECEL_ONLY == FALSE) &&
        (driver_requests->CONTROL_STATE != Cc_cc_decel_only)) {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_ENGAGEMENT_STAT = TRUE;
    }

    if (((driver_requests->ENGAGEMENT_CONDITIONS.VLC_ENGAGEMENT_STAT == TRUE) &&
         ((driver_requests->VLC_SETSPEED > 0u) ||
          (driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED ==
           TRUE))) ||
        ((das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL ==
          TRUE) /*Standby to Active Standstill Wait*/
         &&
         ((driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED == TRUE) ||
          (driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED == TRUE)) &&
         (driver_requests->ENGAGEMENT_CONDITIONS.VLC_ENGAGEMENT_STAT == TRUE) &&
         (((setspeed_t)input->SPEEDOMETER_VEHICLE_SPEED /
           (speedometer_speed_t)Scale_100) == (speedometer_speed_t)0))) {
        driver_requests->ENGAGEMENT_CONDITIONS.ACCEPT_VLC_ENGAGEMENT = TRUE;
    }

    // Special design for Project EP40
    // (input->SPEEDOMETER_VEHICLE_SPEED >
    //   ((speedometer_speed_t)driver_requests->MAX_VLC_SPEED *
    //    Speedo_speed_s)) && (driver_requests->MAX_VLC_SPEED !=
    //    Cc_speed_default_value)
    if ((input->INHIBIT.VLC_DISENGAGEMENT_RAMP == TRUE) ||
        (input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP == TRUE)) {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT_RAMP = TRUE;
    }

    if (input->INHIBIT.VLC_DISENGAGEMENT_DECEL_ONLY == TRUE) {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_DECEL_ONLY = TRUE;

        if (driver_requests->CONTROL_STATE != Cc_cc_decel_only) {
            cc_status->VLC_DECEL_ONLY_MODE_CNT =
                ((uint32)Cc_max_decel_only_time / (uint32)cycle_time);
        }
    }

    if ((das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL == TRUE) ||
        (das_input->LODM_STAT.OVERRIDE_ACCEL == TRUE) ||
        (das_input->LODM_STAT.OVERRIDE_DECEL == TRUE) ||
        (das_input->LODM_STAT.STANDSTILL == TRUE) ||
        (cc_status->VLC_DECEL_ONLY_MODE_CNT == (uint32)0)) {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_END_DECEL_ONLY = TRUE;
    }

    if (cc_status->VLC_DECEL_ONLY_MODE_CNT > (uint32)0) {
        cc_status->VLC_DECEL_ONLY_MODE_CNT--;
    }

    if ((input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == TRUE) ||
        (driver_controls->SELECTED_FUNCTION == Cc_function_none)) {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT = TRUE;
    }

    if (das_input->LODM_STAT.OVERRIDE_ACCEL == TRUE) {
        // driver_requests->ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE = TRUE;
        driver_requests->ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE = FALSE;
    }

    if (((input->ACCELERATION_GRADIENT >= Cc_min_engage_accel_gradient) &&
         (input->ACCELERATION_GRADIENT <= Cc_max_engage_accel_gradient)) &&
        ((das_input->VEHICLE_ACCEL <= accel_control_data->MAX_ALLOWED_ACCEL) &&
         (das_input->VEHICLE_ACCEL >= accel_control_data->MIN_ALLOWED_ACCEL))) {
        driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_ENGAGEMENT = TRUE;
    }

    if ((das_input->LODM_STAT.DAS_SHUTOFF_ACQ == TRUE) &&
        (cc_status->LODM_STAT_LAST_CYCLE.DAS_SHUTOFF_ACQ == FALSE)) {
        driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_DISENGAGEMENT = TRUE;
        accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE = FALSE;
    } else {
        /*System will at least disengage if Acc_min/max_accel_disengage is
         * reached*/
        if (accel_control_data->ACCEL_STATUS.ACCEL_RAMP_ACTIVE == FALSE) {
            driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_DISENGAGEMENT = TRUE;
        }
    }

    if (das_input->PT_STAT.KICKDOWN == TRUE) {
        driver_requests->ENGAGEMENT_CONDITIONS.LIM_DISENGAGEMENT = TRUE;
    }

    VLC_END_LIM_DISENGAGEMENT(das_input, driver_requests, cc_status,
                              accel_control_data);

    driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM = FALSE;
}

static void VLC_RESET_SET_SPEED(const cc_input_data_t* input_data,
                                cc_driver_requests_t* driver_requests) {
    if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT == TRUE) {
        setspeed_t min_set_speed, max_set_speed;
        uint32 uHelp;

        min_set_speed = (setspeed_t)0;
        max_set_speed = (setspeed_t)0;

        if (driver_requests->VLC_SETSPEED > (setspeed_t)0) {
            min_set_speed = driver_requests->MIN_VLC_SETSPEED;
            max_set_speed = driver_requests->MAX_VLC_SETSPEED;

            uHelp = (uint32)(driver_requests->VLC_SETSPEED);
            if (input_data->VEHICLE_STATUS.SPEED_UNIT_KMH == TRUE) {
                uHelp *= (uint32)Speed_conv_factor_kmh;
                uHelp += (uint32)Speed_conv_factor_mph / (uint32)2;
                uHelp /= (uint32)Speed_conv_factor_mph;
            } else {
                uHelp *= (uint32)Speed_conv_factor_mph;
                uHelp += (uint32)Speed_conv_factor_kmh / (uint32)2;
                uHelp /= (uint32)Speed_conv_factor_kmh;
            }
            driver_requests->VLC_SETSPEED = (setspeed_t)MAT_LIM(
                (sint32)uHelp, (sint32)min_set_speed, (sint32)max_set_speed);
        }
    }

    if (driver_requests->DRIVER_OPERATIONS.RESET_SETSPEED == TRUE) {
        driver_requests->VLC_SETSPEED = (setspeed_t)0;
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_DETERMINE_SET_SPEED */
static void VLC_DETERMINE_SET_SPEED(const cc_input_data_t* input_data,
                                    const cc_error_data_t* error_data,
                                    cc_driver_requests_t* driver_requests) {
    cc_setspeed16_t VLC_SETSPEED;

    VLC_SETSPEED = (cc_setspeed16_t)driver_requests->VLC_SETSPEED;

    if (error_data->VLC_INHIBIT == FALSE) {
        /* Determine VLC_SETSPEED */
        if (driver_requests->OPERATIONAL_MODE == Display_op_cc_valid) {
            if ((driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED ==
                 TRUE) ||
                (driver_requests->DRIVER_OPERATIONS.ACCEL_MODE == TRUE) ||
                (driver_requests->DRIVER_OPERATIONS.DECEL_MODE == TRUE)) {
                if (driver_requests->DRIVER_OPERATIONS.DECEL_MODE == TRUE) {
                    VLC_SETSPEED = (cc_setspeed16_t)MAT_MIN(
                        (((sint32)driver_requests
                              ->SPEEDOMETER_VEHICLE_SPEED_OFFSET +
                          (sint32)Speed_round_up) /
                         (sint32)Velocity_s),
                        (sint32)VLC_SETSPEED);
                } else {
                    if (driver_requests->DRIVER_OPERATIONS.ACCEL_MODE == TRUE) {
                        VLC_SETSPEED = (cc_setspeed16_t)MAT_MAX(
                            (((sint32)driver_requests
                                  ->SPEEDOMETER_VEHICLE_SPEED_OFFSET +
                              (sint32)Speed_round_up) /
                             (sint32)Velocity_s),
                            (sint32)VLC_SETSPEED);
                    } else {
                        VLC_SETSPEED =
                            (cc_setspeed16_t)((driver_requests
                                                   ->SPEEDOMETER_VEHICLE_SPEED_OFFSET +
                                               Speed_round_up) /
                                              Velocity_s);
                    }
                }
                VLC_SETSPEED = (cc_setspeed16_t)MAT_LIM(
                    (sint32)VLC_SETSPEED,
                    (sint32)driver_requests->MIN_VLC_SETSPEED,
                    (sint32)driver_requests->MAX_VLC_SETSPEED);
            } else {
                if (driver_requests->DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED ==
                    TRUE) {
                    if (driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 ==
                        TRUE) {
                        /* Calculate the modulus remained relative to
                        SETSPEED_STEP_LEVEL_1 of the current speed, so that
                        final set speed is a 'clean' multiple of it */
                        const sint32 iRemainSpeed =
                            ((sint32)VLC_SETSPEED %
                             (sint32)driver_requests->SETSPEED_STEP_LEVEL_1);
                        VLC_SETSPEED =
                            (cc_setspeed16_t)((sint32)VLC_SETSPEED +
                                              ((sint32)driver_requests
                                                   ->SETSPEED_STEP_LEVEL_1 -
                                               iRemainSpeed));
                    } else {
                        /* Calculate the modulus remained relative to
                        SETSPEED_STEP_LEVEL_2 of the current speed, so that
                        final set speed is a 'clean' multiple of it */
                        const sint32 iRemainSpeed =
                            ((sint32)VLC_SETSPEED %
                             (sint32)driver_requests->SETSPEED_STEP_LEVEL_2);
                        VLC_SETSPEED =
                            (cc_setspeed16_t)((sint32)VLC_SETSPEED +
                                              ((sint32)driver_requests
                                                   ->SETSPEED_STEP_LEVEL_2 -
                                               iRemainSpeed));
                    }

                    VLC_SETSPEED = (cc_setspeed16_t)MAT_MIN(
                        (sint32)VLC_SETSPEED,
                        (sint32)driver_requests->MAX_VLC_SETSPEED);
                } else {
                    if (driver_requests->DRIVER_OPERATIONS
                            .VLC_DECREASE_SET_SPEED == TRUE) {
                        if (driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 ==
                            TRUE) {
                            if (VLC_SETSPEED >= (cc_setspeed16_t)driver_requests
                                                    ->SETSPEED_STEP_LEVEL_1) {
                                if ((VLC_SETSPEED %
                                     (cc_setspeed16_t)driver_requests
                                         ->SETSPEED_STEP_LEVEL_1) !=
                                    (cc_setspeed16_t)0) {
                                    VLC_SETSPEED =
                                        (cc_setspeed16_t)(VLC_SETSPEED -
                                                          (VLC_SETSPEED %
                                                           (cc_setspeed16_t)driver_requests
                                                               ->SETSPEED_STEP_LEVEL_1));
                                } else {
                                    VLC_SETSPEED =
                                        (cc_setspeed16_t)((sint32)VLC_SETSPEED -
                                                          (sint32)driver_requests
                                                              ->SETSPEED_STEP_LEVEL_1);
                                }
                            } else {
                                // setspeed to zero
                            }
                        } else {
                            if (VLC_SETSPEED >= (cc_setspeed16_t)driver_requests
                                                    ->SETSPEED_STEP_LEVEL_2) {
                                if ((VLC_SETSPEED %
                                     (cc_setspeed16_t)driver_requests
                                         ->SETSPEED_STEP_LEVEL_2) !=
                                    (cc_setspeed16_t)0) {
                                    VLC_SETSPEED =
                                        (cc_setspeed16_t)(VLC_SETSPEED -
                                                          (VLC_SETSPEED %
                                                           (cc_setspeed16_t)driver_requests
                                                               ->SETSPEED_STEP_LEVEL_2));
                                } else {
                                    VLC_SETSPEED =
                                        (cc_setspeed16_t)((sint32)VLC_SETSPEED -
                                                          (sint32)driver_requests
                                                              ->SETSPEED_STEP_LEVEL_2);
                                }
                            } else {
                                VLC_SETSPEED = (cc_setspeed16_t)0;
                            }
                        }

                        VLC_SETSPEED = (cc_setspeed16_t)MAT_MAX(
                            (sint32)VLC_SETSPEED,
                            (sint32)driver_requests->MIN_VLC_SETSPEED);
                    }
                }
            }
            /*! Limiting valid set speed */
            VLC_SETSPEED = (setspeed_t)MAT_LIM(
                (sint32)VLC_SETSPEED, (sint32)driver_requests->MIN_VLC_SETSPEED,
                (sint32)driver_requests->MAX_VLC_SETSPEED);
        } else if (driver_requests->OPERATIONAL_MODE ==
                   Display_op_cc_recom_speed) {
            /* accept the speed from TSR or NAV */
            VLC_SETSPEED = (cc_setspeed16_t)driver_requests->RECOMMENDED_SPEED;
            VLC_SETSPEED = (cc_setspeed16_t)MAT_LIM(
                (sint32)VLC_SETSPEED, (sint32)driver_requests->MIN_VLC_SETSPEED,
                (sint32)driver_requests->MAX_VLC_SETSPEED);
        }

    } else {
        /* do not change the set speed at INHIBITION case, it can be done in
         * custom specific part, if required */
        /* do nothing */
    }

    /*! if the no valid set speed was saved, the VLC_SETSPEED = 0
        it needs for driver operation logic for the first engage via RESUME
        the Winter Tire Limiter can change MAX_VLC_SETSPEED */
    driver_requests->VLC_SETSPEED =
        (setspeed_t)MAT_LIM((sint32)VLC_SETSPEED, (sint32)0,
                            (sint32)driver_requests->MAX_VLC_SETSPEED);

    VLC_RESET_SET_SPEED(input_data, driver_requests);
}

/******************************************************************************
  @fn             VLC_DETERMINE_CONTROL_STATE */ /*!

  @description    Determine new state for HMI state machine according to inputs , ARS410SW18, This is only used for ACC

  @param[in,out]      driver_controls  in driver_controls.SELECTED_FUNCTION           This variable reflects which function is selected by the driver
                                       in driver_requests.CONTROL_STATE               Variable that reflects the state of the HMI state machine
                                       in driver_requests.ENGAGEMENT_CONDITIONS       Bit field for all posible CONTROL_STATE transitions
                                       in driver_requests.DRIVER_OPERATIONS           Bit field that reflects which action the driver expects from the system
                                       out driver_requests.CONTROL_STATE_LAST_CYCLE      Variable that reflects the state of the HMI state machine in the former cycle
                                       out driver_requests.CONTROL_STATE             Variable that reflects the proposed next state of the HMI state machine
                                       out driver_requests.OPERATIONAL_MODE             This variable reflects the type operation, valid engagement, disengagement, invalid, etc... 
                                       out driver_requests.CONTROL_STATE_PLIM            This variable reflects the CONTROL_STATE that will be next after PLIM

  @param[in,out]      accel_control_data
  @param[in,out]      das_input
  @param[in,out]      error_data  in   error_data->VLC_SHUTDOWN_STATE               This variable reflects if the CC function was shutdown due to failure condition
                                       error_data->LIM_SHUTDOWN_STATE              This variable reflects if the LIM function was shutdown due to failure condition
                                  out  error_data.SYSTEM_STATUS.DISENGAGE_SIGNAL   This bit is set when the active function is disengaged without intervention from the driver

  @param[in,out]      driver_requests
  @param[in,out]      das_output
  @param[in,out]      input_data

  @return         void

*****************************************************************************/
// static void VLC_DETERMINE_CONTROL_STATE(
//     const cc_driver_controls_t* driver_controls,
//     cc_acceleration_control_data_t* accel_control_data,
//     const cart_das_input_data_t* das_input,
//     cc_error_data_t* error_data,
//     cc_driver_requests_t* driver_requests,
//     cart_das_output_data_t* das_output,
//     const cc_input_data_t* input_data) {
//     /* From the CONTROL_STATE of the last cycle, a new Controlmode is
//     determined
//      */
//     driver_requests->CONTROL_STATE_LAST_CYCLE =
//     driver_requests->CONTROL_STATE;

//     error_data->DISENGAGE_SIGNAL = FALSE;

//     switch (driver_requests->CONTROL_STATE) {
//         case Cc_cc_off: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             das_output->DAS_STAT.DAS_OFF = TRUE;
//             driver_requests->OPERATIONAL_MODE = Display_op_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 if (error_data->LIM_INHIBIT == FALSE) {
//                     /* Remind the CONTROL_STATE after Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_off;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     /* Stay in Mode "Off" */
//                 }
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_acc) {
//                     /*Checks if No Shutdown Condition,SW18*/
//                     if (error_data->VLC_INHIBIT == FALSE) {
//                         /* MainSwitch On and CC Selected and No Shutdown
//                          * Condition: Standby to engage CC */
//                         /* This moves the state from OFF->Standby,SW18 */
//                         driver_requests->CONTROL_STATE = Cc_cc_ready;
//                         das_output->DAS_STAT.DAS_OFF = FALSE;
//                     } else {
//                         /* Stay in Mode "Off" */
//                     }
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION ==
//                     Cc_function_lim) {
//                         if (error_data->LIM_INHIBIT == FALSE) {
//                             /* MainSwitch On and Limiter Selected: Standby to
//                              * engage Limiter */
//                             driver_requests->CONTROL_STATE = Cc_lim_ready;
//                         } else {
//                             /* Stay in Mode "Off" */
//                         }
//                     } else {
//                         /* Stay in Mode "Off" */
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_cc_ready: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             driver_requests->OPERATIONAL_MODE = Display_op_cc_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_lim) {
//                     /* MainSwitch On and Limiter Selected: Standby to engage
//                      * Limiter */
//                     driver_requests->CONTROL_STATE = Cc_lim_ready;
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION ==
//                     Cc_function_acc) {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .VLC_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .VLC_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                                 /* Engage ACC */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_cc_engage;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_invalid;
//                             }
//                         } else {
//                             /* Stay in Mode "Ready" */
//                             if ((driver_requests->DRIVER_OPERATIONS
//                                      .VLC_INCREASE_SET_SPEED) ||
//                                 (driver_requests->DRIVER_OPERATIONS
//                                      .VLC_DECREASE_SET_SPEED)) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_invalid;
//                             }

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .SWITCH_SPEED_UNIT == TRUE) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_valid;
//                             }
//                         }
//                     } else {
//                         /*If Mainswitch OFF, state moves from Standby->OFF,
//                          * SW18*/
//                         driver_requests->CONTROL_STATE = Cc_cc_off;
//                     }
//                 }
//             }
//             /*Checks if Shutdown Condition,SW18*/
//             if (error_data->VLC_INHIBIT == TRUE) {
//                 /* MainSwitch OFF OR Shutdown Condition*/
//                 /* This moves the state from Standby->OFF,SW18 */
//                 driver_requests->CONTROL_STATE = Cc_cc_off;
//             }
//             break;
//         }
//         case Cc_cc_engage: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_cc_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_ENGAGEMENT ==
//                 TRUE) {
//                 driver_requests->CONTROL_STATE = Cc_cc_active;
//             } else {
//                 /* Stay in Mode "Engage" */
//                 if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT ==
//                     TRUE) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_cc_valid;
//                 }
//             }
//             break;
//         }
//         case Cc_cc_active:
//         case Cc_cc_override: {
//             if (driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION == TRUE) {
//                 accel_control_data->ACCEL_STATUS.CANCEL_RAMP = TRUE;
//             }

//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT ==
//                 TRUE) {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM ==
//                     TRUE) {
//                     /* Remind the CONTROL_STATE B4 Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     if (driver_requests->CONTROL_STATE == Cc_cc_override) {
//                         driver_requests->CONTROL_STATE = Cc_cc_ready;
//                     } else {
//                         driver_requests->CONTROL_STATE = Cc_cc_disengage;
//                     }
//                     driver_requests->OPERATIONAL_MODE =
//                     Display_op_cc_disengage;

//                     if (driver_requests->DRIVER_OPERATIONS
//                             .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                         error_data->DISENGAGE_SIGNAL = TRUE;
//                     }
//                 }
//             } else {
//                 if (driver_requests->CONTROL_STATE == Cc_cc_active) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_cc_active;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DECEL_ONLY
//                     ==
//                         TRUE) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .GOTO_PERM_LIM == TRUE) {
//                             /* Remind the CONTROL_STATE after Permanent
//                             Limiter
//                              */
//                             driver_requests->CONTROL_STATE_PLIM =
//                             Cc_cc_ready; driver_requests->CONTROL_STATE =
//                             Cc_plim_active;
//                         } else {
//                             driver_requests->CONTROL_STATE =
//                             Cc_cc_decel_only;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_disengage;

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                                 error_data->DISENGAGE_SIGNAL = TRUE;
//                             }
//                         }
//                     } else {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .VLC_DISENGAGEMENT_RAMP == TRUE) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .GOTO_PERM_LIM == TRUE) {
//                                 /* Remind the CONTROL_STATE after Permanent
//                                  * Limiter */
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_plim_active;
//                             } else {
//                                 driver_requests->CONTROL_STATE =
//                                     Cc_cc_disengage;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;

//                                 if (driver_requests->DRIVER_OPERATIONS
//                                         .DISENGAGE_DRIVER_INTERVENED ==
//                                         FALSE) {
//                                     error_data->DISENGAGE_SIGNAL = TRUE;
//                                 }
//                             }
//                         } else {
//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .CANCEL_FUNKTION == TRUE) {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .GOTO_PERM_LIM == TRUE) {
//                                     /* Remind the CONTROL_STATE B4 Permanent
//                                      * Limiter */
//                                     driver_requests->CONTROL_STATE_PLIM =
//                                         Cc_cc_ready;
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_plim_active;
//                                 } else {
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_cc_disengage;
//                                     driver_requests->OPERATIONAL_MODE =
//                                         Display_op_cc_disengage;
//                                 }
//                             } else {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .DRIVER_OVERRIDE == TRUE) {
//                                     if
//                                     (driver_requests->ENGAGEMENT_CONDITIONS
//                                             .GOTO_PERM_LIM == TRUE) {
//                                         /* Remind the CONTROL_STATE after
//                                          * Permanent Limiter */
//                                         driver_requests->CONTROL_STATE_PLIM =
//                                             Cc_cc_active;
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_plim_active;
//                                     } else {
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_cc_override;
//                                     }
//                                 } else {
//                                     /* Stay in Mode "HoldSpeed" */
//                                     if ((driver_requests->DRIVER_OPERATIONS
//                                              .VLC_INCREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_DECREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .ACCEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .DECEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_TAKE_ACTUAL_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .SWITCH_SPEED_UNIT == TRUE)) {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_valid;
//                                     } else {
//                                         if
//                                         (driver_requests->RECOMMENDED_SPEED <
//                                             (driver_requests->VLC_SETSPEED -
//                                              driver_requests
//                                                  ->SETSPEED_STEP_LEVEL_2)) {
//                                             driver_requests->OPERATIONAL_MODE
//                                             =
//                                                 Display_op_cc_recom_speed;
//                                         } else {
//                                             if (driver_requests
//                                                     ->DRIVER_OPERATIONS
//                                                     .VLC_RESUME_SET_SPEED ==
//                                                 TRUE) {
//                                                 driver_requests
//                                                     ->OPERATIONAL_MODE =
//                                                     Display_op_cc_valid;
//                                             } else {
//                                                 driver_requests
//                                                     ->OPERATIONAL_MODE =
//                                                     Display_op_cc_active;
//                                             }
//                                         }
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 } else {
//                     /* (driver_requests->CONTROL_STATE == Cc_cc_override) */
//                     das_output->DAS_STAT.DAS_OVERRIDE = TRUE;

//                     driver_requests->OPERATIONAL_MODE =
//                     Display_op_cc_override;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .VLC_DISENGAGEMENT_RAMP == TRUE) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .GOTO_PERM_LIM == TRUE) {
//                             /* Remind the CONTROL_STATE after Permanent
//                             Limiter
//                              */
//                             driver_requests->CONTROL_STATE_PLIM =
//                             Cc_cc_ready; driver_requests->CONTROL_STATE =
//                             Cc_plim_active;
//                         } else {
//                             driver_requests->CONTROL_STATE = Cc_cc_ready;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_disengage;

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                                 error_data->DISENGAGE_SIGNAL = TRUE;
//                             }
//                         }
//                     } else {
//                         if (driver_requests->DRIVER_OPERATIONS
//                                 .CANCEL_FUNKTION == TRUE) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .GOTO_PERM_LIM == TRUE) {
//                                 /* Remind the CONTROL_STATE B4 Permanent
//                                 Limiter
//                                  */
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_plim_active;
//                             } else {
//                                 driver_requests->CONTROL_STATE = Cc_cc_ready;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;
//                             }
//                         } else {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .DRIVER_OVERRIDE == TRUE) {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .GOTO_PERM_LIM == TRUE) {
//                                     /* Remind the CONTROL_STATE B4 Permanent
//                                      * Limiter */
//                                     driver_requests->CONTROL_STATE_PLIM =
//                                         Cc_cc_active;
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_plim_active;
//                                 } else {
//                                     /* Stay in Mode "Override HoldSpeed" */
//                                     if ((driver_requests->DRIVER_OPERATIONS
//                                              .VLC_INCREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_DECREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .ACCEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .DECEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_TAKE_ACTUAL_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .SWITCH_SPEED_UNIT == TRUE)) {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_valid;
//                                     } else {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_override;
//                                     }
//                                 }
//                             } else {
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_cc_active;
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_cc_disengage:
//         case Cc_cc_decel_only:
//         case Cc_lim_disengage: {
//             das_output->DAS_STAT.DAS_ENGAGED =
//                 TRUE; /*SW18 To be checked, not correct, implemented
//                 earlier*/
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;

//             if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
//                 (driver_requests->CONTROL_STATE == Cc_lim_disengage)) {
//                 das_output->DAS_STAT.DAS_SHUTOFF_REQ = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             }

//             if (driver_requests->CONTROL_STATE == Cc_lim_disengage) {
//                 das_output->DAS_STAT.DAS_LIM = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_LIM = FALSE;
//             }

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
//                     (driver_requests->CONTROL_STATE == Cc_cc_decel_only)) {
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                 } else {
//                     driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                 }

//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT
//                 ==
//                     TRUE) {
//                     if ((driver_requests->CONTROL_STATE == Cc_cc_disengage)
//                     ||
//                         (driver_requests->CONTROL_STATE == Cc_cc_decel_only))
//                         { driver_requests->CONTROL_STATE = Cc_cc_ready;
//                     }
//                 } else {
//                     if ((driver_requests->DRIVER_OPERATIONS
//                              .VLC_TAKE_ACTUAL_SPEED == TRUE) ||
//                         (driver_requests->DRIVER_OPERATIONS
//                              .VLC_RESUME_SET_SPEED == TRUE)) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                             /* Engage Cruise */
//                             driver_requests->CONTROL_STATE = Cc_cc_engage;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_valid;
//                         } else {
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_invalid;
//                         }
//                     } else {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .LIM_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                                 /* Engage Limiter */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_lim_active;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_invalid;
//                             }
//                         } else {
//                             if (((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .VLC_END_DECEL_ONLY == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_cc_decel_only)) ||
//                                 ((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .END_VLC_DISENGAGEMENT == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_cc_disengage)) ||
//                                 ((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .END_LIM_DISENGAGEMENT == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_lim_disengage))) {
//                                 if (driver_controls->SELECTED_FUNCTION ==
//                                     Cc_function_acc) {
//                                     /* Cruise Selected: Standby to engage
//                                     Cruise
//                                      */
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_cc_ready;
//                                 } else {
//                                     if (driver_controls->SELECTED_FUNCTION ==
//                                         Cc_function_lim) {
//                                         /* Limiter Selected: Standby to
//                                         engage
//                                          * Limiter */
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_lim_ready;
//                                     } else {
//                                         /* Nothing Selected: go to OFF */
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_cc_off;
//                                     }
//                                 }
//                             } else {
//                                 /* Stay in Mode "Shutoff"/"Decel Only" */
//                                 if (driver_requests->CONTROL_STATE ==
//                                     Cc_cc_decel_only) {
//                                     error_data->DISENGAGE_SIGNAL = TRUE;
//                                     driver_requests->OPERATIONAL_MODE =
//                                         Display_op_cc_disengage;
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_lim_ready: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_lim_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_acc) {
//                     /* MainSwitch On and ACC Selected: Standby to engage ACC
//                     */ driver_requests->CONTROL_STATE = Cc_cc_ready;
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION ==
//                     Cc_function_lim) {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .LIM_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                                 /* Engage ACC */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_lim_active;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_invalid;
//                             }
//                         } else {
//                             /* Stay in Mode "Ready" */
//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .SWITCH_SPEED_UNIT == TRUE) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             }
//                         }
//                     } else {
//                         driver_requests->CONTROL_STATE = Cc_cc_off;
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_lim_active:
//         case Cc_lim_override: {
//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = TRUE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_lim_active;

//             if (driver_requests->CONTROL_STATE == Cc_lim_override) {
//                 das_output->DAS_STAT.DAS_OVERRIDE = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             }

//             if ((driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION == TRUE)
//             ||
//                 (driver_requests->ENGAGEMENT_CONDITIONS.LIM_DISENGAGEMENT ==
//                  TRUE)) {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM ==
//                     TRUE) {
//                     /* Remind the CONTROL_STATE after Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     if (driver_requests->CONTROL_STATE == Cc_lim_active) {
//                         driver_requests->CONTROL_STATE = Cc_lim_disengage;
//                     } else {
//                         driver_requests->CONTROL_STATE = Cc_lim_ready;
//                     }

//                     driver_requests->OPERATIONAL_MODE =
//                         Display_op_lim_disengage;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .LIM_DISENGAGEMENT == TRUE) {
//                         error_data->DISENGAGE_SIGNAL = TRUE;
//                     }
//                 }
//             } else {
//                 if ((driver_requests->DRIVER_OPERATIONS
//                          .LIM_INCREASE_SET_SPEED == TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS
//                          .LIM_DECREASE_SET_SPEED == TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS.ACCEL_MODE == TRUE)
//                     || (driver_requests->DRIVER_OPERATIONS.DECEL_MODE ==
//                     TRUE)) { driver_requests->OPERATIONAL_MODE =
//                     Display_op_lim_valid;
//                 }

//                 if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT ==
//                     TRUE) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_lim_valid;
//                 }

//                 if (driver_requests->CONTROL_STATE == Cc_lim_active) {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .DRIVER_OVERRIDE == TRUE) {
//                         driver_requests->CONTROL_STATE = Cc_lim_override;
//                     }
//                 } else {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .DRIVER_OVERRIDE == FALSE) {
//                         driver_requests->CONTROL_STATE = Cc_lim_active;
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_plim_active: {
//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = TRUE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_plim;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Go back to control mode B4 Permanent Limiter */
//                 driver_requests->CONTROL_STATE =
//                     driver_requests->CONTROL_STATE_PLIM;
//             } else {
//                 /* Driver Actuation? */
//                 if ((driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED
//                 ==
//                      TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED
//                     ==
//                      TRUE)) {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                         /* Engage Cruise */
//                         driver_requests->CONTROL_STATE = Cc_cc_engage;
//                         driver_requests->OPERATIONAL_MODE =
//                         Display_op_cc_valid;
//                     } else {
//                         driver_requests->OPERATIONAL_MODE =
//                             Display_op_cc_invalid;
//                     }
//                 } else {
//                     if ((driver_requests->DRIVER_OPERATIONS
//                              .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                         (driver_requests->DRIVER_OPERATIONS
//                              .LIM_RESUME_SET_SPEED == TRUE)) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                             /* Engage Limiter */
//                             driver_requests->CONTROL_STATE = Cc_lim_active;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_lim_valid;
//                         } else {
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_lim_invalid;
//                         }
//                     } else {
//                         /* Remind Disengagement of Cruise */
//                         if (driver_requests->CONTROL_STATE_PLIM ==
//                             Cc_cc_active) {
//                             if ((driver_requests->ENGAGEMENT_CONDITIONS
//                                      .VLC_DISENGAGEMENT_RAMP == TRUE) ||
//                                 (driver_requests->ENGAGEMENT_CONDITIONS
//                                      .VLC_DISENGAGEMENT == TRUE) ||
//                                 (driver_requests->DRIVER_OPERATIONS
//                                      .CANCEL_FUNKTION == TRUE)) {
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         default: {
//             break;
//         }
//     }
//     _PARAM_UNUSED(input_data);

//     if (das_input->LODM_STAT.STANDSTILL == TRUE) {
//         driver_requests->CONTROL_MODE = Cc_standstill_mode;
//     } else {
//         if (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == TRUE) {
//             driver_requests->CONTROL_MODE = Cc_follow_mode;
//         } else {
//             driver_requests->CONTROL_MODE = Cc_free_mode;
//         }
//     }

//     if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT == TRUE) {
//         driver_requests->OPERATIONAL_MODE = Display_op_cc_valid;
//     }
// }

/******************************************************************************
  @fn             VLC_ONLY_DETERMINE_CONTROL_STATE */ /*!

  @description    Determine new state for HMI state machine according to inputs, ARS410SW18, used only for CC, no ACC here

  @param[in,out]      driver_controls  in driver_controls.SELECTED_FUNCTION           This variable reflects which function is selected by the driver
                                       in driver_requests.CONTROL_STATE               Variable that reflects the state of the HMI state machine
                                       in driver_requests.ENGAGEMENT_CONDITIONS       Bit field for all posible CONTROL_STATE transitions
                                       in driver_requests.DRIVER_OPERATIONS           Bit field that reflects which action the driver expects from the system
                                       out driver_requests.CONTROL_STATE_LAST_CYCLE      Variable that reflects the state of the HMI state machine in the former cycle
                                       out driver_requests.CONTROL_STATE             Variable that reflects the proposed next state of the HMI state machine
                                       out driver_requests.OPERATIONAL_MODE             This variable reflects the type operation, valid engagement, disengagement, invalid, etc... 
                                       out driver_requests.CONTROL_STATE_PLIM            This variable reflects the CONTROL_STATE that will be next after PLIM

  @param[in,out]      accel_control_data
  @param[in,out]      das_input
  @param[in,out]      error_data  in   error_data->VLC_SHUTDOWN_STATE               This variable reflects if the CC function was shutdown due to failure condition
                                       error_data->LIM_SHUTDOWN_STATE              This variable reflects if the LIM function was shutdown due to failure condition
                                  out  error_data.SYSTEM_STATUS.DISENGAGE_SIGNAL   This bit is set when the active function is disengaged without intervention from the driver

  @param[in,out]      driver_requests
  @param[in,out]      das_output
  @param[in,out]      input_data

  @return         void

*****************************************************************************/
// static void VLC_ONLY_DETERMINE_CONTROL_STATE(
//     const cc_driver_controls_t* driver_controls,
//     cc_acceleration_control_data_t* accel_control_data,
//     const cart_das_input_data_t* das_input,
//     cc_error_data_t* error_data,
//     cc_driver_requests_t* driver_requests,
//     cart_das_output_data_t* das_output,
//     const cc_input_data_t* input_data) {
//     /* From the CONTROL_STATE of the last cycle, a new Controlmode is
//     determined
//      */
//     driver_requests->CONTROL_STATE_LAST_CYCLE =
//     driver_requests->CONTROL_STATE;

//     error_data->DISENGAGE_SIGNAL = FALSE;

//     switch (driver_requests->CONTROL_STATE) {
//         case Cc_cc_off: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             das_output->DAS_STAT.DAS_OFF = TRUE;
//             driver_requests->OPERATIONAL_MODE = Display_op_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 if (error_data->LIM_INHIBIT == FALSE) {
//                     /* Remind the CONTROL_STATE after Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_off;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     /* Stay in Mode "Off" */
//                 }
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_cc) {
//                     /*Checks if No Shutdown Condition,SW18*/
//                     if (error_data->VLC_INHIBIT == FALSE) {
//                         /* MainSwitch On and CC Selected and No Shutdown
//                          * Condition: Standby to engage CC */
//                         /* This moves the state from OFF->Standby,SW18 */
//                         driver_requests->CONTROL_STATE = Cc_cc_ready;
//                         das_output->DAS_STAT.DAS_OFF = FALSE;
//                     } else {
//                         /* Stay in Mode "Off" */
//                     }
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION ==
//                     Cc_function_lim) {
//                         if (error_data->LIM_INHIBIT == FALSE) {
//                             /* MainSwitch On and Limiter Selected: Standby to
//                              * engage Limiter */
//                             driver_requests->CONTROL_STATE = Cc_lim_ready;
//                         } else {
//                             /* Stay in Mode "Off" */
//                         }
//                     } else {
//                         /* Stay in Mode "Off" */
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_cc_ready: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             driver_requests->OPERATIONAL_MODE = Display_op_cc_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_lim) {
//                     /* MainSwitch On and Limiter Selected: Standby to engage
//                      * Limiter */
//                     driver_requests->CONTROL_STATE = Cc_lim_ready;
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION == Cc_function_cc)
//                     {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .VLC_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .VLC_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                                 /* Engage ACC */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_cc_engage;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_invalid;
//                             }
//                         } else {
//                             /* Stay in Mode "Ready" */
//                             if ((driver_requests->DRIVER_OPERATIONS
//                                      .VLC_INCREASE_SET_SPEED) ||
//                                 (driver_requests->DRIVER_OPERATIONS
//                                      .VLC_DECREASE_SET_SPEED)) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_invalid;
//                             }

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .SWITCH_SPEED_UNIT == TRUE) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_valid;
//                             }
//                         }
//                     } else {
//                         /*If Mainswitch OFF, state moves from Standby->OFF,
//                          * SW18*/
//                         driver_requests->CONTROL_STATE = Cc_cc_off;
//                     }
//                 }
//             }
//             /*Checks if Shutdown Condition,SW18*/
//             if (error_data->VLC_INHIBIT == TRUE) {
//                 /* MainSwitch OFF OR Shutdown Condition*/
//                 /* This moves the state from Standby->OFF,SW18 */
//                 driver_requests->CONTROL_STATE = Cc_cc_off;
//             }
//             break;
//         }
//         case Cc_cc_engage: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_cc_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.END_VLC_ENGAGEMENT ==
//                 TRUE) {
//                 driver_requests->CONTROL_STATE = Cc_cc_active;
//             } else {
//                 /* Stay in Mode "Engage" */
//                 if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT ==
//                     TRUE) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_cc_valid;
//                 }
//             }
//             break;
//         }
//         case Cc_cc_active:
//         case Cc_cc_override: {
//             if (driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION == TRUE) {
//                 accel_control_data->ACCEL_STATUS.CANCEL_RAMP = TRUE;
//             }

//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT ==
//                 TRUE) {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM ==
//                     TRUE) {
//                     /* Remind the CONTROL_STATE B4 Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     driver_requests->CONTROL_STATE = Cc_cc_ready;
//                     driver_requests->OPERATIONAL_MODE =
//                     Display_op_cc_disengage;

//                     if (driver_requests->DRIVER_OPERATIONS
//                             .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                         error_data->DISENGAGE_SIGNAL = TRUE;
//                     }
//                 }
//             } else {
//                 if (driver_requests->CONTROL_STATE == Cc_cc_active) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_cc_active;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DECEL_ONLY
//                     ==
//                         TRUE) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .GOTO_PERM_LIM == TRUE) {
//                             /* Remind the CONTROL_STATE after Permanent
//                             Limiter
//                              */
//                             driver_requests->CONTROL_STATE_PLIM =
//                             Cc_cc_ready; driver_requests->CONTROL_STATE =
//                             Cc_plim_active;
//                         } else {
//                             driver_requests->CONTROL_STATE =
//                             Cc_cc_decel_only;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_disengage;

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                                 error_data->DISENGAGE_SIGNAL = TRUE;
//                             }
//                         }
//                     } else {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .VLC_DISENGAGEMENT_RAMP == TRUE) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .GOTO_PERM_LIM == TRUE) {
//                                 /* Remind the CONTROL_STATE after Permanent
//                                  * Limiter */
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_plim_active;
//                             } else {
//                                 driver_requests->CONTROL_STATE =
//                                     Cc_cc_disengage;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;

//                                 if (driver_requests->DRIVER_OPERATIONS
//                                         .DISENGAGE_DRIVER_INTERVENED ==
//                                         FALSE) {
//                                     error_data->DISENGAGE_SIGNAL = TRUE;
//                                 }
//                             }
//                         } else {
//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .CANCEL_FUNKTION == TRUE) {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .GOTO_PERM_LIM == TRUE) {
//                                     /* Remind the CONTROL_STATE B4 Permanent
//                                      * Limiter */
//                                     driver_requests->CONTROL_STATE_PLIM =
//                                         Cc_cc_ready;
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_plim_active;
//                                 } else {
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_cc_disengage;
//                                     driver_requests->OPERATIONAL_MODE =
//                                         Display_op_cc_disengage;
//                                 }
//                             } else {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .DRIVER_OVERRIDE == TRUE) {
//                                     if
//                                     (driver_requests->ENGAGEMENT_CONDITIONS
//                                             .GOTO_PERM_LIM == TRUE) {
//                                         /* Remind the CONTROL_STATE after
//                                          * Permanent Limiter */
//                                         driver_requests->CONTROL_STATE_PLIM =
//                                             Cc_cc_active;
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_plim_active;
//                                     } else {
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_cc_override;
//                                     }
//                                 } else {
//                                     /* Stay in Mode "HoldSpeed" */
//                                     if ((driver_requests->DRIVER_OPERATIONS
//                                              .VLC_INCREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_DECREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .ACCEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .DECEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_TAKE_ACTUAL_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .SWITCH_SPEED_UNIT == TRUE)) {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_valid;
//                                     } else {
//                                         if
//                                         (driver_requests->RECOMMENDED_SPEED <
//                                             (driver_requests->VLC_SETSPEED -
//                                              driver_requests
//                                                  ->SETSPEED_STEP_LEVEL_2)) {
//                                             driver_requests->OPERATIONAL_MODE
//                                             =
//                                                 Display_op_cc_recom_speed;
//                                         } else {
//                                             if (driver_requests
//                                                     ->DRIVER_OPERATIONS
//                                                     .VLC_RESUME_SET_SPEED ==
//                                                 TRUE) {
//                                                 driver_requests
//                                                     ->OPERATIONAL_MODE =
//                                                     Display_op_cc_valid;
//                                             } else {
//                                                 driver_requests
//                                                     ->OPERATIONAL_MODE =
//                                                     Display_op_cc_active;
//                                             }
//                                         }
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 } else {
//                     /* (driver_requests->CONTROL_STATE == Cc_cc_override) */
//                     das_output->DAS_STAT.DAS_OVERRIDE = TRUE;

//                     driver_requests->OPERATIONAL_MODE =
//                     Display_op_cc_override;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .VLC_DISENGAGEMENT_RAMP == TRUE) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .GOTO_PERM_LIM == TRUE) {
//                             /* Remind the CONTROL_STATE after Permanent
//                             Limiter
//                              */
//                             driver_requests->CONTROL_STATE_PLIM =
//                             Cc_cc_ready; driver_requests->CONTROL_STATE =
//                             Cc_plim_active;
//                         } else {
//                             driver_requests->CONTROL_STATE = Cc_cc_ready;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_disengage;

//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .DISENGAGE_DRIVER_INTERVENED == FALSE) {
//                                 error_data->DISENGAGE_SIGNAL = TRUE;
//                             }
//                         }
//                     } else {
//                         if (driver_requests->DRIVER_OPERATIONS
//                                 .CANCEL_FUNKTION == TRUE) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .GOTO_PERM_LIM == TRUE) {
//                                 /* Remind the CONTROL_STATE B4 Permanent
//                                 Limiter
//                                  */
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_plim_active;
//                             } else {
//                                 driver_requests->CONTROL_STATE = Cc_cc_ready;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;
//                             }
//                         } else {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .DRIVER_OVERRIDE == TRUE) {
//                                 if (driver_requests->ENGAGEMENT_CONDITIONS
//                                         .GOTO_PERM_LIM == TRUE) {
//                                     /* Remind the CONTROL_STATE B4 Permanent
//                                      * Limiter */
//                                     driver_requests->CONTROL_STATE_PLIM =
//                                         Cc_cc_active;
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_plim_active;
//                                 } else {
//                                     /* Stay in Mode "Override HoldSpeed" */
//                                     if ((driver_requests->DRIVER_OPERATIONS
//                                              .VLC_INCREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_DECREASE_SET_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .ACCEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .DECEL_MODE == TRUE) ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .VLC_TAKE_ACTUAL_SPEED == TRUE)
//                                              ||
//                                         (driver_requests->DRIVER_OPERATIONS
//                                              .SWITCH_SPEED_UNIT == TRUE)) {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_valid;
//                                     } else {
//                                         driver_requests->OPERATIONAL_MODE =
//                                             Display_op_cc_override;
//                                     }
//                                 }
//                             } else {
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_cc_active;
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_cc_disengage:
//         case Cc_cc_decel_only:
//         case Cc_lim_disengage: {
//             das_output->DAS_STAT.DAS_ENGAGED =
//                 TRUE; /*SW18 To be checked, not correct, implemented
//                 earlier*/
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;

//             if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
//                 (driver_requests->CONTROL_STATE == Cc_lim_disengage)) {
//                 das_output->DAS_STAT.DAS_SHUTOFF_REQ = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             }

//             if (driver_requests->CONTROL_STATE == Cc_lim_disengage) {
//                 das_output->DAS_STAT.DAS_LIM = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_LIM = FALSE;
//             }

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 if ((driver_requests->CONTROL_STATE == Cc_cc_disengage) ||
//                     (driver_requests->CONTROL_STATE == Cc_cc_decel_only)) {
//                     driver_requests->CONTROL_STATE_PLIM = Cc_cc_ready;
//                 } else {
//                     driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                 }

//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT
//                 ==
//                     TRUE) {
//                     if ((driver_requests->CONTROL_STATE == Cc_cc_disengage)
//                     ||
//                         (driver_requests->CONTROL_STATE == Cc_cc_decel_only))
//                         { driver_requests->CONTROL_STATE = Cc_cc_ready;
//                     }
//                 } else {
//                     if ((driver_requests->DRIVER_OPERATIONS
//                              .VLC_TAKE_ACTUAL_SPEED == TRUE) ||
//                         (driver_requests->DRIVER_OPERATIONS
//                              .VLC_RESUME_SET_SPEED == TRUE)) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                             /* Engage Cruise */
//                             driver_requests->CONTROL_STATE = Cc_cc_engage;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_valid;
//                         } else {
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_cc_invalid;
//                         }
//                     } else {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .LIM_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                                 /* Engage Limiter */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_lim_active;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_invalid;
//                             }
//                         } else {
//                             if (((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .VLC_END_DECEL_ONLY == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_cc_decel_only)) ||
//                                 ((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .END_VLC_DISENGAGEMENT == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_cc_disengage)) ||
//                                 ((driver_requests->ENGAGEMENT_CONDITIONS
//                                       .END_LIM_DISENGAGEMENT == TRUE) &&
//                                  (driver_requests->CONTROL_STATE ==
//                                   Cc_lim_disengage))) {
//                                 if (driver_controls->SELECTED_FUNCTION ==
//                                     Cc_function_cc) {
//                                     /* Cruise Selected: Standby to engage
//                                     Cruise
//                                      */
//                                     driver_requests->CONTROL_STATE =
//                                         Cc_cc_ready;
//                                 } else {
//                                     if (driver_controls->SELECTED_FUNCTION ==
//                                         Cc_function_lim) {
//                                         /* Limiter Selected: Standby to
//                                         engage
//                                          * Limiter */
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_lim_ready;
//                                     } else {
//                                         /* Nothing Selected: go to OFF */
//                                         driver_requests->CONTROL_STATE =
//                                             Cc_cc_off;
//                                     }
//                                 }
//                             } else {
//                                 /* Stay in Mode "Shutoff"/"Decel Only" */
//                                 if (driver_requests->CONTROL_STATE ==
//                                     Cc_cc_decel_only) {
//                                     error_data->DISENGAGE_SIGNAL = TRUE;
//                                     driver_requests->OPERATIONAL_MODE =
//                                         Display_op_cc_disengage;
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_lim_ready: {
//             das_output->DAS_STAT.DAS_ENGAGED = FALSE;
//             das_output->DAS_STAT.DAS_LIM = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_lim_none;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Remind the CONTROL_STATE after Permanent Limiter */
//                 driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                 driver_requests->CONTROL_STATE = Cc_plim_active;
//             } else {
//                 if (driver_controls->SELECTED_FUNCTION == Cc_function_cc) {
//                     /* MainSwitch On and ACC Selected: Standby to engage ACC
//                     */ driver_requests->CONTROL_STATE = Cc_cc_ready;
//                 } else {
//                     if (driver_controls->SELECTED_FUNCTION ==
//                     Cc_function_lim) {
//                         if ((driver_requests->DRIVER_OPERATIONS
//                                  .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                             (driver_requests->DRIVER_OPERATIONS
//                                  .LIM_RESUME_SET_SPEED == TRUE)) {
//                             if (driver_requests->ENGAGEMENT_CONDITIONS
//                                     .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                                 /* Engage ACC */
//                                 driver_requests->CONTROL_STATE =
//                                 Cc_lim_active;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             } else {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_invalid;
//                             }
//                         } else {
//                             /* Stay in Mode "Ready" */
//                             if (driver_requests->DRIVER_OPERATIONS
//                                     .SWITCH_SPEED_UNIT == TRUE) {
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_lim_valid;
//                             }
//                         }
//                     } else {
//                         driver_requests->CONTROL_STATE = Cc_cc_off;
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_lim_active:
//         case Cc_lim_override: {
//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = TRUE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_lim_active;

//             if (driver_requests->CONTROL_STATE == Cc_lim_override) {
//                 das_output->DAS_STAT.DAS_OVERRIDE = TRUE;
//             } else {
//                 das_output->DAS_STAT.DAS_OVERRIDE = FALSE;
//             }

//             if ((driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION == TRUE)
//             ||
//                 (driver_requests->ENGAGEMENT_CONDITIONS.LIM_DISENGAGEMENT ==
//                  TRUE)) {
//                 if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM ==
//                     TRUE) {
//                     /* Remind the CONTROL_STATE after Permanent Limiter */
//                     driver_requests->CONTROL_STATE_PLIM = Cc_lim_ready;
//                     driver_requests->CONTROL_STATE = Cc_plim_active;
//                 } else {
//                     if (driver_requests->CONTROL_STATE == Cc_lim_active) {
//                         driver_requests->CONTROL_STATE = Cc_lim_disengage;
//                     } else {
//                         driver_requests->CONTROL_STATE = Cc_lim_ready;
//                     }

//                     driver_requests->OPERATIONAL_MODE =
//                         Display_op_lim_disengage;

//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .LIM_DISENGAGEMENT == TRUE) {
//                         error_data->DISENGAGE_SIGNAL = TRUE;
//                     }
//                 }
//             } else {
//                 if ((driver_requests->DRIVER_OPERATIONS
//                          .LIM_INCREASE_SET_SPEED == TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS
//                          .LIM_DECREASE_SET_SPEED == TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS.ACCEL_MODE == TRUE)
//                     || (driver_requests->DRIVER_OPERATIONS.DECEL_MODE ==
//                     TRUE)) { driver_requests->OPERATIONAL_MODE =
//                     Display_op_lim_valid;
//                 }

//                 if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT ==
//                     TRUE) {
//                     driver_requests->OPERATIONAL_MODE = Display_op_lim_valid;
//                 }

//                 if (driver_requests->CONTROL_STATE == Cc_lim_active) {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .DRIVER_OVERRIDE == TRUE) {
//                         driver_requests->CONTROL_STATE = Cc_lim_override;
//                     }
//                 } else {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .DRIVER_OVERRIDE == FALSE) {
//                         driver_requests->CONTROL_STATE = Cc_lim_active;
//                     }
//                 }
//             }
//             break;
//         }
//         case Cc_plim_active: {
//             das_output->DAS_STAT.DAS_ENGAGED = TRUE;
//             das_output->DAS_STAT.DAS_LIM = TRUE;
//             das_output->DAS_STAT.DAS_SHUTOFF_REQ = FALSE;
//             das_output->DAS_STAT.DAS_OVERRIDE = FALSE;

//             driver_requests->OPERATIONAL_MODE = Display_op_plim;

//             if (driver_requests->ENGAGEMENT_CONDITIONS.GOTO_PERM_LIM == TRUE)
//             {
//                 /* Go back to control mode B4 Permanent Limiter */
//                 driver_requests->CONTROL_STATE =
//                     driver_requests->CONTROL_STATE_PLIM;
//             } else {
//                 /* Driver Actuation? */
//                 if ((driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED
//                 ==
//                      TRUE) ||
//                     (driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED
//                     ==
//                      TRUE)) {
//                     if (driver_requests->ENGAGEMENT_CONDITIONS
//                             .ACCEPT_VLC_ENGAGEMENT == TRUE) {
//                         /* Engage Cruise */
//                         driver_requests->CONTROL_STATE = Cc_cc_engage;
//                         driver_requests->OPERATIONAL_MODE =
//                         Display_op_cc_valid;
//                     } else {
//                         driver_requests->OPERATIONAL_MODE =
//                             Display_op_cc_invalid;
//                     }
//                 } else {
//                     if ((driver_requests->DRIVER_OPERATIONS
//                              .LIM_TAKE_ACTUAL_SPEED == TRUE) ||
//                         (driver_requests->DRIVER_OPERATIONS
//                              .LIM_RESUME_SET_SPEED == TRUE)) {
//                         if (driver_requests->ENGAGEMENT_CONDITIONS
//                                 .ACCEPT_LIM_ENGAGEMENT == TRUE) {
//                             /* Engage Limiter */
//                             driver_requests->CONTROL_STATE = Cc_lim_active;
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_lim_valid;
//                         } else {
//                             driver_requests->OPERATIONAL_MODE =
//                                 Display_op_lim_invalid;
//                         }
//                     } else {
//                         /* Remind Disengagement of Cruise */
//                         if (driver_requests->CONTROL_STATE_PLIM ==
//                             Cc_cc_active) {
//                             if ((driver_requests->ENGAGEMENT_CONDITIONS
//                                      .VLC_DISENGAGEMENT_RAMP == TRUE) ||
//                                 (driver_requests->ENGAGEMENT_CONDITIONS
//                                      .VLC_DISENGAGEMENT == TRUE) ||
//                                 (driver_requests->DRIVER_OPERATIONS
//                                      .CANCEL_FUNKTION == TRUE)) {
//                                 driver_requests->CONTROL_STATE_PLIM =
//                                     Cc_cc_ready;
//                                 driver_requests->OPERATIONAL_MODE =
//                                     Display_op_cc_disengage;
//                             }
//                         }
//                     }
//                 }
//             }
//             break;
//         }
//         default: {
//             break;
//         }
//     }
//     _PARAM_UNUSED(input_data);

//     if (das_input->LODM_STAT.STANDSTILL == TRUE) {
//         driver_requests->CONTROL_MODE = Cc_standstill_mode;
//     } else {
//         if (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == TRUE) {
//             driver_requests->CONTROL_MODE = Cc_follow_mode;
//         } else {
//             driver_requests->CONTROL_MODE = Cc_free_mode;
//         }
//     }

//     if (driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT == TRUE) {
//         driver_requests->OPERATIONAL_MODE = Display_op_cc_valid;
//     }
// }

/******************************************************************************
  @fn             VLC_DETERMINE_SPEED_LIMITS */ /*!

                                  @description    Determine the Limits for
                                SetSpeed

                                  @param[in]      accel_control_data
                                  @param[in]      das_input
                                  @param[in]      input
                                  @param[out]     driver_requests
                                  @param[out]     acc_output

                                  @return         void

                                *****************************************************************************/

static void VLC_DETERMINE_SPEED_LIMITS(
    const cc_acceleration_control_data_t* accel_control_data,
    const cart_das_input_data_t* das_input,
    const cc_input_data_t* input,
    cc_driver_requests_t* driver_requests,
    const VLC_acc_output_data_t* acc_output) {
    sint32 AccelerationOffset;    /* to clarify if the variable still needed */
    sint32 MinAccelerationOffset; /* to clarify if the variable still needed */
    sint32 MaxAccelerationOffset; /* to clarify if the variable still needed */
    // sint32 recommended_velocity; /* that was the detected speed from camera,
    // not
    //                                 used at the moment */
    sint32 MinValidSpeedoVelocity; /*!< Minimum valid speedometer velosity */
    sint32 MaxValidSpeedoVelocity; /*!< Maximum valid speedometer velosity */

    // recommended_velocity = driver_requests->RECOMMENDED_SPEED;

    MinAccelerationOffset = Cc_max_neg_accel_offset;
    MaxAccelerationOffset = Cc_max_pos_accel_offset;

    if (das_input->VEHICLE_ACCEL < (acceleration_t)0) {
        AccelerationOffset = (sint32)Cc_neg_accel_offset;
    } else {
        AccelerationOffset = (sint32)Cc_pos_accel_offset;
    }

    if (input->VEHICLE_STATUS.SPEED_UNIT_KMH == TRUE) {
        /*unit KMH*/

        driver_requests->MIN_VLC_SETSPEED = Acc_fsr_min_setspeed_kmh;
        driver_requests->MIN_VLC_ENGAGESPEED = Acc_fsr_min_v_engage_kmh;
        driver_requests->MAX_VLC_SETSPEED = Acc_max_setspeed_kmh;

        if (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == TRUE) {
            /*object effectiv == true*/
            driver_requests->VLC_DISENGAGE_THRESHOLD =
                Acc_fsr_disengage_threshold_kmh;

        } else {
            /*object effectiv == false*/
            driver_requests->VLC_DISENGAGE_THRESHOLD =
                Cc_disengage_threshold_cruise_kmh;
        }

        driver_requests->SETSPEED_STEP_LEVEL_1 = Cc_setspeed_step_level_1_kmh;
        driver_requests->SETSPEED_STEP_LEVEL_2 = Cc_setspeed_step_level_2_kmh;
        driver_requests->MAX_VLC_SPEED = Cc_max_speed_kmh;

        AccelerationOffset *= (sint32)Speed_conv_factor_kmh;
        AccelerationOffset /= (sint32)Factor_s;
        MinAccelerationOffset *= (sint32)Speed_conv_factor_kmh;
        MinAccelerationOffset /= (sint32)Factor_s;
        MaxAccelerationOffset *= (sint32)Speed_conv_factor_kmh;
        MaxAccelerationOffset /= (sint32)Factor_s;

        // recommended_velocity *= Speed_conv_factor_kmh;
        // recommended_velocity /= Factor_s;
        // recommended_velocity *= Setspeed_s;
        // recommended_velocity /= Velocity_s;
        // driver_requests->RECOMMENDED_SPEED = (setspeed_t)MAT_QUANT(
        //     recommended_velocity, (sint32)driver_requests->RECOMMENDED_SPEED,
        //     (sint32)Cc_min_recommended_speed_kmh,
        //     (sint32)driver_requests->MAX_VLC_SETSPEED,
        //     (sint32)Cc_recommended_speed_intervals,
        //     Cc_recommended_speed_hyst);
    } else {
        /*unit MPH*/
        if (input->COUNTRY_CODE == Cc_usa) {
            driver_requests->MIN_VLC_SETSPEED = Acc_fsr_min_setspeed_usa_mph;
            driver_requests->MIN_VLC_ENGAGESPEED = Acc_fsr_min_v_engage_usa_mph;
        } else {
            driver_requests->MIN_VLC_SETSPEED = Acc_fsr_min_setspeed_mph;
            driver_requests->MIN_VLC_ENGAGESPEED = Acc_fsr_min_v_engage_mph;
        }

        driver_requests->MAX_VLC_SETSPEED = Acc_max_setspeed_mph;

        if (accel_control_data->ACCEL_STATUS.OBJECT_EFFECTIVE == TRUE) {
            /*object effectiv == true*/
            driver_requests->VLC_DISENGAGE_THRESHOLD =
                Acc_fsr_disengage_threshold_mph;

        } else {
            /*object effectiv == false*/
            driver_requests->VLC_DISENGAGE_THRESHOLD =
                Cc_disengage_threshold_cruise_mph;
        }

        driver_requests->SETSPEED_STEP_LEVEL_1 = Cc_setspeed_step_level_1_mph;
        driver_requests->SETSPEED_STEP_LEVEL_2 = Cc_setspeed_step_level_2_mph;
        driver_requests->MAX_VLC_SPEED = Cc_max_speed_mph;
        AccelerationOffset *= (sint32)Speed_conv_factor_mph;
        AccelerationOffset /= (sint32)Factor_s;
        MinAccelerationOffset *= (sint32)Speed_conv_factor_mph;
        MinAccelerationOffset /= (sint32)Factor_s;
        MaxAccelerationOffset *= (sint32)Speed_conv_factor_mph;
        MaxAccelerationOffset /= (sint32)Factor_s;

        // recommended_velocity *= Speed_conv_factor_mph;
        // recommended_velocity /= Factor_s;
        // recommended_velocity *= Setspeed_s;
        // recommended_velocity /= Velocity_s;
        // driver_requests->RECOMMENDED_SPEED = (setspeed_t)MAT_QUANT(
        //     recommended_velocity, (sint32)driver_requests->RECOMMENDED_SPEED,
        //     (sint32)Cc_min_recommended_speed_mph,
        //     (sint32)driver_requests->MAX_VLC_SETSPEED,
        //     (sint32)Cc_recommended_speed_intervals,
        //     Cc_recommended_speed_hyst);
    }

    /* Limit to the maximum vehicle speed */
    driver_requests->MIN_VLC_SETSPEED =
        (setspeed_t)MAT_MIN((sint32)driver_requests->MIN_VLC_SETSPEED,
                            (sint32)input->VEHICLE_SPEED_LIMIT);
    driver_requests->MAX_VLC_SETSPEED =
        (setspeed_t)MAT_MIN((sint32)driver_requests->MAX_VLC_SETSPEED,
                            (sint32)input->VEHICLE_SPEED_LIMIT);

    /* Limit to the PermanentLimiterSetSpeed */
    driver_requests->MIN_VLC_SETSPEED =
        (setspeed_t)MAT_MIN((sint32)driver_requests->MIN_VLC_SETSPEED,
                            (sint32)input->PERMANENT_LIMITER_SETSPEED);
    driver_requests->MAX_VLC_SETSPEED =
        (setspeed_t)MAT_MIN((sint32)driver_requests->MAX_VLC_SETSPEED,
                            (sint32)input->PERMANENT_LIMITER_SETSPEED);

    /* Offset is a function of vehicle acceleration */
    AccelerationOffset *= (sint32)das_input->VEHICLE_ACCEL;
    AccelerationOffset /= (sint32)Acceleration_s;
    AccelerationOffset = MAT_LIM(AccelerationOffset, MinAccelerationOffset,
                                 MaxAccelerationOffset);

    /*! The minimum valid speed is the real vehicle speed from VDY +
     * Cc_min_valid_speedo_offset */
    MinValidSpeedoVelocity = (sint32)das_input->VEHICLE_SPEED;
    MinValidSpeedoVelocity += (sint32)MAT_CALCULATE_PARAM_VALUE1D(
        Cc_min_valid_speedo_offset, Cc_min_valid_speedo_offset_points,
        das_input->VEHICLE_SPEED);
    /*! The maximum valid speedometer speed is the vehicle speed +
     * Cc_max_valid_speedo_offset */
    MaxValidSpeedoVelocity = (sint32)das_input->VEHICLE_SPEED;
    MaxValidSpeedoVelocity += (sint32)MAT_CALCULATE_PARAM_VALUE1D(
        Cc_max_valid_speedo_offset, Cc_max_valid_speedo_offset_points,
        das_input->VEHICLE_SPEED);

    if (input->VEHICLE_STATUS.SPEED_UNIT_KMH == TRUE) {
        /*! unit KMH */
        MinValidSpeedoVelocity *= (sint32)Speed_conv_factor_kmh;
        MinValidSpeedoVelocity /= (sint32)Factor_s;
        MaxValidSpeedoVelocity *= (sint32)Speed_conv_factor_kmh;
        MaxValidSpeedoVelocity /= (sint32)Factor_s;
    } else {
        /*! unit MPH */
        MinValidSpeedoVelocity *= (sint32)Speed_conv_factor_mph;
        MinValidSpeedoVelocity /= (sint32)Factor_s;
        MaxValidSpeedoVelocity *= (sint32)Speed_conv_factor_mph;
        MaxValidSpeedoVelocity /= (sint32)Factor_s;
    }

    /*! If the speedometer speed exceeds the allowed borders, then the
     * speedometer speed will be ignored */
    /*! That "safe" speedometer speed is used for acceleration calculation in
     * cruise control case */
    driver_requests->SPEEDOMETER_VEHICLE_SPEED = (speedometer_speed_t)MAT_LIM(
        (sint32)input->SPEEDOMETER_VEHICLE_SPEED, MinValidSpeedoVelocity,
        MaxValidSpeedoVelocity);

    driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET =
        (speedometer_speed_t)((sint32)das_input->VEHICLE_SPEED *
                                  Speed_conv_factor_kmh / Scale_1000 *
                                  MAT_CALCULATE_PARAM_VALUE1D(
                                      factor_actual_speed_to_display_speed,
                                      factor_actual_speed_to_display_speed_points,
                                      das_input->VEHICLE_SPEED) /
                                  Scale_1000 +
                              MAT_CALCULATE_PARAM_VALUE1D(
                                  offset_actual_speed_to_display_speed,
                                  offset_actual_speed_to_display_speed_points,
                                  das_input->VEHICLE_SPEED));

    /*! Value limitation */
    driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET =
        (speedometer_speed_t)MAT_LIM(
            (sint32)driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET,
            (sint32)0,
            (sint32)(driver_requests->MAX_VLC_SPEED *
                     (setspeed_t)Speedo_speed_s));
}

/******************************************************************************
  @fn             ACCSM_INPUT_WRAPPER */ /*!
                                         @description    Input wrapper for ACC
                                       state machine generated from Simulink
                                       *****************************************************************************/
void ACCSM_INPUT_WRAPPER(const times_t cycle_time,
                         const cart_das_input_data_t* das_input,
                         const cc_driver_controls_t* driver_controls,
                         const cc_input_data_t* input,
                         cc_error_data_t* error_data,
                         cart_das_output_data_t* das_output,
                         cc_status_t* cc_status,
                         const VLC_acc_output_data_t* acc_output) {
    ACCSM_In.SELECTED_FUNCTION = driver_controls->SELECTED_FUNCTION;
    ACCSM_In.CANCEL_FUNKTION =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.CANCEL_FUNKTION;
    ACCSM_In.VLC_INHIBIT = error_data->VLC_INHIBIT;
    ACCSM_In.DRIVER_OVERRIDE =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.DRIVER_OVERRIDE;
    ACCSM_In.SWITCH_SPEED_UNIT ==
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.SWITCH_SPEED_UNIT;
    ACCSM_In.PROPOSE_RECOMMENDED_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
            .PROPOSE_RECOMMENDED_SPEED;
    ACCSM_In.RECOMMENDED_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.RECOMMENDED_SPEED;
    ACCSM_In.SETSPEED_STEP_LEVEL_1 =
        cc_status->VLC_DRIVER_REQUESTS.SETSPEED_STEP_LEVEL_1;
    ACCSM_In.VLC_SETSPEED = cc_status->VLC_DRIVER_REQUESTS.VLC_SETSPEED;
    ACCSM_In.VLC_TAKE_ACTUAL_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED;
    ACCSM_In.VLC_INCREASE_SET_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED;
    ACCSM_In.VLC_DECREASE_SET_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED;
    ACCSM_In.VLC_RESUME_SET_SPEED =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED;
    ACCSM_In.VLC_DECEL_ONLY =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.VLC_DECEL_ONLY;
    ACCSM_In.VLC_END_DECEL_ONLY =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.VLC_END_DECEL_ONLY;
    ACCSM_In.ACCEPT_VLC_ENGAGEMENT =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
            .ACCEPT_VLC_ENGAGEMENT;
    ACCSM_In.END_VLC_ENGAGEMENT =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.END_VLC_ENGAGEMENT;
    ACCSM_In.VLC_DISENGAGEMENT =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT;
    ACCSM_In.VLC_DISENGAGEMENT_RAMP =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
            .VLC_DISENGAGEMENT_RAMP;
    ACCSM_In.END_VLC_DISENGAGEMENT =
        cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
            .END_VLC_DISENGAGEMENT;
    ACCSM_In.STANDSTILL = das_input->LODM_STAT.STANDSTILL;
    ACCSM_In.OBJECT_EFFECTIVE =
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE;
}

void ACCSM_OUTPUT_WRAPPER(const times_t cycle_time,
                          const cart_das_input_data_t* das_input,
                          const cc_driver_controls_t* driver_controls,
                          const cc_input_data_t* input,
                          cc_error_data_t* error_data,
                          cart_das_output_data_t* das_output,
                          cc_status_t* cc_status,
                          const VLC_acc_output_data_t* acc_output) {
    cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE = ACCSM_Out.CONTROL_STATE;
    cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE =
        ACCSM_Out.CONTROL_STATE_LAST_CYCLE;
    cc_status->VLC_DRIVER_REQUESTS.OPERATIONAL_MODE =
        ACCSM_Out.OPERATIONAL_MODE;
    das_output->DAS_STAT.DAS_ENGAGED = ACCSM_Out.DAS_ENGAGED;
    das_output->DAS_STAT.DAS_OFF = ACCSM_Out.DAS_OFF;
    das_output->DAS_STAT.DAS_OVERRIDE = ACCSM_Out.DAS_OVERRIDE;
    das_output->DAS_STAT.DAS_SHUTOFF_REQ = ACCSM_Out.DAS_SHUTOFF_REQ;
    cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.CANCEL_RAMP =
        ACCSM_Out.CANCEL_RAMP;
    cc_status->VLC_DRIVER_REQUESTS.CONTROL_MODE = ACCSM_Out.CONTROL_MODE;
}

/******************************************************************************
  @fn             VLC_DETERMINE_CONTROLSTATE_SETSPEED */ /*!

                         Description     Call all functions concerning the
                       interpretation of
                                         driver_controls

                         @param[in]      cycle_time    driver_controls data
                       structure is passed through to referenced function
                         @param[in]      das_input       input data structure is
                       passed through to referenced function
                         @param[in]      driver_controls  AccelControlData data
                       structure is passed through to referenced function
                         @param[in]      input
                         @param[out]     error_data
                         @param[out]     das_output
                         @param[out]     cc_status
                         @param[out]     acc_output

                         @return         void

                       *****************************************************************************/
void VLC_DETERMINE_CONTROLSTATE_SETSPEED(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    const cc_driver_controls_t* driver_controls,
    const cc_input_data_t* input,
    cc_error_data_t* error_data,
    cart_das_output_data_t* das_output,
    cc_status_t* cc_status,
    const VLC_acc_output_data_t* acc_output) {
    VLC_DETERMINE_SPEED_LIMITS(&cc_status->VLC_ACCEL_CONTROL_DATA, das_input,
                               input, &cc_status->VLC_DRIVER_REQUESTS,
                               acc_output);

    VLC_DETERMINE_ENGAGEMENT_CONDITIONS(cycle_time, das_input, input,
                                        &cc_status->VLC_ACCEL_CONTROL_DATA,
                                        &cc_status->VLC_DRIVER_REQUESTS,
                                        cc_status, driver_controls, acc_output);

#if Switch_ACCSM_Gen_Code
    ACCSM_INPUT_WRAPPER(cycle_time, das_input, driver_controls, input,
                        error_data, das_output, cc_status, acc_output);
    ACCSM_step();
    ACCSM_OUTPUT_WRAPPER(cycle_time, das_input, driver_controls, input,
                         error_data, das_output, cc_status, acc_output);

#else
    if (driver_controls->SELECTED_FUNCTION == Cc_function_cc) {
        VLC_ONLY_DETERMINE_CONTROL_STATE(
            driver_controls, &cc_status->VLC_ACCEL_CONTROL_DATA, das_input,
            error_data, &cc_status->VLC_DRIVER_REQUESTS, das_output, input);
    } else {
        VLC_DETERMINE_CONTROL_STATE(
            driver_controls, &cc_status->VLC_ACCEL_CONTROL_DATA, das_input,
            error_data, &cc_status->VLC_DRIVER_REQUESTS, das_output, input);
    }
#endif
    VLC_DETERMINE_SET_SPEED(input, error_data, &cc_status->VLC_DRIVER_REQUESTS);

    cc_status->LODM_STAT_LAST_CYCLE = das_input->LODM_STAT;
    cc_status->VLC_SPEED_UNIT_LAST_CYCLE =
        (uint8)input->VEHICLE_STATUS.SPEED_UNIT_KMH;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */