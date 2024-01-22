/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "cc.h"
#include "cc_par.h"

static void VLC_MAIN_INIT(cc_status_t* cc_status);

/*************************************************************************************************************************
  Functionname:    VLC_MAIN_INIT */
static void VLC_MAIN_INIT(cc_status_t* cc_status) {
#define VLC_MAX_ACCEL_LAT_LIM_INIT_VALUE (acceleration_t)(10000)

    cc_status->VLC_ACCEL_CONTROL_DATA.MINIMUM_COMMANDED_ACCELERATION =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_COMMANDED_ACCELERATION =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MINIMUM_REQUESTED_ACCELERATION =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_REQUESTED_ACCELERATION =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MINIMUM_REQUESTED_ACCELERATION_FILTERED =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_REQUESTED_ACCELERATION_FILTERED =
        (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA
        .MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE = (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA
        .MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE = (acceleration_t)0;

    cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_ACCELERATION_LATERAL_LIMITED =
        VLC_MAX_ACCEL_LAT_LIM_INIT_VALUE;
    cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE = FALSE;
    cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.ACCEL_RAMP_ACTIVE = FALSE;
    cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.DECEL_LIM_ENGAGE = FALSE;
    cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE = FALSE;
    cc_status->VLC_ACCEL_CONTROL_DATA.MIN_ALLOWED_ACCEL = (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MAX_ALLOWED_ACCEL = (acceleration_t)0;
    cc_status->VLC_ACCEL_CONTROL_DATA.MIN_CUSTOM_ALLOWED_ACCEL = Accel_min;
    cc_status->VLC_ACCEL_CONTROL_DATA.MAX_CUSTOM_ALLOWED_ACCEL = Accel_max;

    cc_status->VLC_CONTROL_DATA.VLC_ACCELERATION = (acceleration_t)0;
    cc_status->VLC_CONTROL_DATA.LIM_ACCELERATION = (acceleration_t)0;
    cc_status->VLC_CONTROL_DATA.MAXIMUM_ACCELERATION_LIMIT = Accel_max;
    cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT = Accel_min;
    cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE = FALSE;
    cc_status->VLC_CONTROL_DATA.VLC_AVLC_TO_VLC_TRANSITION = FALSE;
    cc_status->VLC_CONTROL_DATA.ACCELERATION_REQUEST_GRAD = (gradient_t)0;
    cc_status->VLC_CONTROL_DATA.VLC_ACCEL_FILTER_TIME =
        (times_t)criticality_accel_filter_time[1]; /* Default value */

    cc_status->VLC_DRIVER_REQUESTS.SETSPEED_STEP_LEVEL_1 = (setspeed_t)1;
    cc_status->VLC_DRIVER_REQUESTS.SETSPEED_STEP_LEVEL_2 = (setspeed_t)1;
    cc_status->VLC_DRIVER_REQUESTS.VLC_SETSPEED = (setspeed_t)0;
    cc_status->VLC_DRIVER_REQUESTS.MIN_VLC_SETSPEED = (setspeed_t)0;
    cc_status->VLC_DRIVER_REQUESTS.MAX_VLC_SETSPEED = Cc_speed_default_value;
    cc_status->VLC_DRIVER_REQUESTS.MAX_VLC_SPEED = Cc_speed_default_value;
    cc_status->VLC_DRIVER_REQUESTS.VLC_DISENGAGE_THRESHOLD = (setspeed_t)0;

    cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE = Cc_cc_off;
    cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE = Cc_cc_off;
    cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_PLIM = Cc_cc_off;
    cc_status->VLC_DRIVER_REQUESTS.OPERATIONAL_MODE = Display_op_none;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
        .PROPOSE_RECOMMENDED_SPEED = 0;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
        .ACTUAL_SPEED_TAKEN_OVERRIDE = FALSE;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
        .SELECTED_FUNCTION_ACTIVE = FALSE;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS.TIMED_INCREMENT =
        FALSE;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS.TIMED_DECREMENT =
        FALSE;
    cc_status->VLC_DRIVER_REQUESTS.SPEEDOMETER_VEHICLE_SPEED =
        (speedometer_speed_t)0;
    cc_status->VLC_DRIVER_REQUESTS.SPEEDOMETER_VEHICLE_SPEED_OFFSET =
        (speedometer_speed_t)0;
    cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.SWITCH_SPEED_UNIT = FALSE;
    cc_status->VLC_DRIVER_REQUESTS.DRIVE_OFF_TIME = Cc_drive_off_smooth_time;
    cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME = (times_t)0;
}

/*************************************************************************************************************************
  Functionname:    VLC_INIT */
void VLC_INIT(cc_input_data_t* cc_input,
              cc_driver_controls_t* driver_controls,
              cc_status_t* cc_status) {
    cc_status->VLC_LAT_ACCEL_LAST_CYCLE = (acceleration_t)0;
    cc_status->VLC_LAT_ACCEL_GRAD_LAST_CYCLE = (gradient_t)0;
    cc_status->VLC_CONTROL_DATA.DECEL_LIM_ACTIVE_COUNTER =
        Cc_max_decel_limit_time;
    cc_status->VLC_ENGAGE_OVERRIDE_COUNTER = 0;

    driver_controls->SELECTED_FUNCTION = Cc_function_none;
    driver_controls->SELECTED_FUNCTION_LAST_CYCLE = Cc_function_none;

    cc_input->ACCELERATION_GRADIENT = (gradient_t)0;
    cc_input->LATERAL_ACCELERATION = (acceleration_t)0;
    cc_input->ACTUAL_GEAR = Pt_gear_park;
    cc_input->DATA_VALIDITY.LATERAL_ACCEL = FALSE;
    cc_input->DATA_VALIDITY.ACTUAL_GEAR = FALSE;
    cc_input->DATA_VALIDITY.SPEEDO_SPEED = FALSE;
    cc_input->INHIBIT.VLC_DISENGAGEMENT_DECEL_ONLY = FALSE;
    cc_input->INHIBIT.VLC_DISENGAGEMENT_RAMP = FALSE;
    cc_input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP = FALSE;
    cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH = TRUE;
    cc_input->VEHICLE_STATUS.SPORTS_MODE = FALSE;
    cc_input->SPEEDOMETER_VEHICLE_SPEED = (speedometer_speed_t)0;
    cc_input->PERMANENT_LIMITER_SETSPEED = Cc_speed_default_value;
    cc_input->COUNTRY_CODE = (cc_country_code_t)0;
    cc_input->VEHICLE_SPEED_LIMIT = (setspeed_t)0;
    cc_input->CURRENT_SPEED_LIMIT = (setspeed_t)0;

    VLC_HMI_INIT(cc_input, cc_status);
    VLC_MAIN_INIT(cc_status);
}

/*************************************************************************************************************************
  Functionname:    VLC_EXEC */
DLLEXPORT void VLC_EXEC(const times_t cycle_time,
                        cc_driver_controls_t* driver_controls,
                        const cc_input_data_t* cc_input,
                        cc_error_data_t* error_data,
                        const cart_das_input_data_t* das_input,
                        cart_das_output_data_t* das_output,
                        const VLC_acc_output_data_t* acc_output,
                        const t_CamLaneInputData* pCamLaneData,
                        cc_status_t* cc_status,
                        PACCInfo* p_pacc_info) {
    VLC_DETERMINE_CONTROLSTATE_SETSPEED(cycle_time, das_input, driver_controls,
                                        cc_input, error_data, das_output,
                                        cc_status, acc_output);

    VLC_COMMAND_ACCEL(cycle_time, cc_input, das_input, acc_output, pCamLaneData,
                      das_output, cc_status, p_pacc_info);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */