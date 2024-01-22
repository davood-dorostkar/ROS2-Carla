
#ifndef VLC_EXT_H
#define VLC_EXT_H

/** @defgroup vlc_long_cc VLC_LONG_CC (Cruise Control)
 Interface to the component CC. The component CC is responsible for
 the kinematik part of the cruise control function. From the actions of the
 driver (switch pack, accel/decel pedal, etc...) this component determines
 the control mode and the setspeed, desired by the driver. Also the appropiate
 acceleration request ist determined to be delivered to the dynamic part of
 the cruise control by means of the CARTRONIC interface

  - A functional specification can be found under <A
HREF="../../../cc_function_spec.mht">cc_function_spec.mht</A>
  - A design specification can be found under <A
HREF="../../../vlc_long_design_spec.mht">vlc_long_design_spec.mht</A>
   @ingroup acc_long_veh
@{ */

/* includes */

/*#include "vlc_long_veh.h"*/
#include "cart_ext.h"
#include "acc_obj_ext.h"
#include "acc_out_ext.h"
#include "switch_ext.h"
#include "display_ext.h"
#include "cc_cfg.h"

/*! Status of the switch that preselects the cruise control or the speed limiter
 * function */
#define Cc_function_none ((cc_selected_function_t)0)
#define Cc_function_acc ((cc_selected_function_t)1)
#define Cc_function_cc ((cc_selected_function_t)2)
#define Cc_function_lim ((cc_selected_function_t)3)
typedef enum_t cc_selected_function_t;

/*! @brief selected CC function*/
typedef struct cc_driver_controls_t {
    uint32 DUMMY_FOR_ALIGNMENT;
    cc_selected_function_t SELECTED_FUNCTION;
    cc_selected_function_t SELECTED_FUNCTION_LAST_CYCLE;
} cc_driver_controls_t;

#ifndef cc_accel_status_t_define
/*! @brief Status bits for the acceleration control */
typedef struct cc_accel_status_t {
    ubit8_t LAT_ACCEL_LIM_ACTIVE : 1;
    ubit8_t DECEL_LIM_ENGAGE : 1;
    ubit8_t OBJECT_EFFECTIVE : 1;
    ubit8_t ACCEL_RAMP_ACTIVE : 1;
    ubit8_t ALLOW_INIT : 1;
    ubit8_t CANCEL_RAMP : 1;
    ubit8_t RAPID_RAMP : 1;
    ubit8_t NO_RAMP : 1;
} cc_accel_status_t;
#define cc_accel_status_t_define
#endif

#ifndef cc_control_data_t_define
/*! @brief Data for the speed control */
typedef struct cc_control_data_t {
    acceleration_t LIM_ACCELERATION;
    acceleration_t VLC_ACCELERATION;
    acceleration_t VLC_ACCELERATION_P_PART;
    acceleration_t VLC_ACCELERATION_I_PART;
    acceleration_t MAXIMUM_ACCELERATION_LIMIT;
    acceleration_t MINIMUM_ACCELERATION_LIMIT;
    gradient_t ACCELERATION_REQUEST_GRAD;
    uint8 DECEL_LIM_OVERRIDE_ACTIVE;
    uint8 VLC_AVLC_TO_VLC_TRANSITION;
    uint16 DECEL_LIM_ACTIVE_COUNTER;
    times_t VLC_ACCEL_FILTER_TIME;
} cc_control_data_t;
#define cc_control_data_t_define
#endif

/*! @brief Data for acceleration gradient limitation*/
#ifndef cc_accel_gradient_limits_t_define
typedef struct cc_accel_gradient_limits_t {
    gradient_t MAX_NEG_GRAD;
    gradient_t MAX_POS_GRAD;
} cc_accel_gradient_limits_t;
#define cc_accel_gradient_limits_t_define
#endif

/*! @brief Data for the acceleration control */
#ifndef cc_acceleration_control_data_t_define
typedef struct cc_acceleration_control_data_t {
    acceleration_t MAXIMUM_ACCELERATION_LATERAL_LIMITED;
    acceleration_t MAXIMUM_COMMANDED_ACCELERATION;
    acceleration_t MINIMUM_COMMANDED_ACCELERATION;
    acceleration_t MAXIMUM_REQUESTED_ACCELERATION;
    acceleration_t MINIMUM_REQUESTED_ACCELERATION;
    acceleration_t MAXIMUM_REQUESTED_ACCELERATION_FILTERED;
    acceleration_t MINIMUM_REQUESTED_ACCELERATION_FILTERED;
    acceleration_t MAXIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
    acceleration_t MINIMUM_REQUESTED_ACCELERATION_LAST_CYCLE;
    acceleration_t MIN_ALLOWED_ACCEL;
    acceleration_t MAX_ALLOWED_ACCEL;
    acceleration_t MAX_CUSTOM_ALLOWED_ACCEL;
    acceleration_t MIN_CUSTOM_ALLOWED_ACCEL;
    cc_accel_status_t ACCEL_STATUS;
    cc_accel_gradient_limits_t ACCEL_GRADIENT_LIMITS;
} cc_acceleration_control_data_t;
#define cc_acceleration_control_data_t_define
#endif

/*! Country variant code */
#define Cc_rest_of_world ((cc_country_code_t)0)
#define Cc_usa ((cc_country_code_t)1)
#define Cc_japan ((cc_country_code_t)2)
#define Cc_canada ((cc_country_code_t)3)
typedef enum_t
    cc_country_code_t; /*!< @values: Cc_rest_of_world,Cc_usa,Cc_japan,Cc_canada
                        */

/*! @brief Validity bits for driver assistance system vehicle data */
typedef struct cc_das_data_validity_t {
    ubit8_t LATERAL_ACCEL : 1;
    ubit8_t ACTUAL_GEAR : 1;
    ubit8_t SPEEDO_SPEED : 1;
    ubit8_t : 5;
} cc_das_data_validity_t;

/*! @brief Bits for driver assistance system vehicle status */
typedef struct cc_das_vehicle_status_t {
    ubit8_t SPEED_UNIT_KMH : 1;
    ubit8_t SPORTS_MODE : 1;
    ubit8_t : 6;
} cc_das_vehicle_status_t;

/*! @brief Bits for driver assistance system inhibit */
typedef struct cc_das_inhibit_t {
    ubit8_t VLC_DISENGAGEMENT_NO_RAMP : 1;
    ubit8_t VLC_DISENGAGEMENT_RAMP : 1;
    ubit8_t VLC_DISENGAGEMENT_RAPID_RAMP : 1;
    ubit8_t VLC_DISENGAGEMENT_DECEL_ONLY : 1;
    ubit8_t VLC_INHIBIT_ENGAGEMENT : 1;
    ubit8_t : 3;
} cc_das_inhibit_t;

/*! @brief CC input interface: input data*/
typedef struct cc_input_data_t {
    gradient_t ACCELERATION_GRADIENT;
    acceleration_t LATERAL_ACCELERATION;
    pt_gear_t ACTUAL_GEAR;
    cc_das_data_validity_t DATA_VALIDITY;
    cc_das_inhibit_t INHIBIT;
    cc_das_vehicle_status_t VEHICLE_STATUS;
    speedometer_speed_t SPEEDOMETER_VEHICLE_SPEED;
    setspeed_t PERMANENT_LIMITER_SETSPEED;
    cc_country_code_t COUNTRY_CODE;
    setspeed_t VEHICLE_SPEED_LIMIT;
    setspeed_t CURRENT_SPEED_LIMIT;
} cc_input_data_t;

/*! State variable of the HMI state machine */
#define Cc_cc_off ((cc_control_state_t)0)
#define Cc_cc_ready ((cc_control_state_t)1)
#define Cc_cc_engage ((cc_control_state_t)2)
#define Cc_cc_active ((cc_control_state_t)3)
#define Cc_cc_override ((cc_control_state_t)4)
#define Cc_cc_disengage ((cc_control_state_t)5)
#define Cc_cc_decel_only ((cc_control_state_t)6)
#define Cc_lim_ready ((cc_control_state_t)7)
#define Cc_lim_active ((cc_control_state_t)8)
#define Cc_lim_override ((cc_control_state_t)9)
#define Cc_lim_disengage ((cc_control_state_t)10)
#define Cc_plim_active ((cc_control_state_t)11)
typedef enum_t
    cc_control_state_t; /*!< @values:
                           Cc_cc_off,Cc_cc_ready,Cc_cc_engage,Cc_cc_active,Cc_cc_override,Cc_cc_disengage,Cc_cc_decel_only,Cc_lim_ready,Cc_lim_active,Cc_lim_override,Cc_lim_disengage,Cc_plim_active
                         */

/*! Mode within the active state */
#define Cc_standstill_mode ((cc_control_mode_t)0)
#define Cc_follow_mode ((cc_control_mode_t)1)
#define Cc_free_mode ((cc_control_mode_t)2)
typedef enum_t
    cc_control_mode_t; /*!< @values:
                          Cc_standstill_mode,Cc_follow_mode,Cc_free_mode */

/*! @brief Bits to reflect state transitions within the hmi state machine */
typedef struct cc_engagement_conditions_t {
    ubit8_t ACCEPT_VLC_ENGAGEMENT : 1;
    ubit8_t ACCEPT_LIM_ENGAGEMENT : 1;
    ubit8_t END_VLC_ENGAGEMENT : 1;
    ubit8_t DRIVER_OVERRIDE : 1;
    ubit8_t VLC_DISENGAGEMENT : 1;
    ubit8_t VLC_DISENGAGEMENT_RAMP : 1;
    ubit8_t VLC_DECEL_ONLY : 1;
    ubit8_t VLC_END_DECEL_ONLY : 1;

    ubit8_t END_VLC_DISENGAGEMENT : 1;
    ubit8_t LIM_DISENGAGEMENT : 1;
    ubit8_t END_LIM_DISENGAGEMENT : 1;
    ubit8_t GOTO_PERM_LIM : 1;
    ubit8_t END_PERM_LIM : 1;
    ubit8_t DRIVE_OFF_POSSIBLE : 1;
    ubit8_t VLC_ENGAGEMENT_STAT : 1;
    ubit8_t CONTROL_TO_RELEVANT_OBJECT : 1;
} cc_engagement_conditions_t;

/*! @brief Bits to reflect the operations of the driver */
typedef struct cc_driver_operations_t {
    ubit8_t VLC_TAKE_ACTUAL_SPEED : 1;
    ubit8_t LIM_TAKE_ACTUAL_SPEED : 1;
    ubit8_t VLC_RESUME_SET_SPEED : 1;
    ubit8_t LIM_RESUME_SET_SPEED : 1;
    ubit8_t ACCEL_MODE : 1;
    ubit8_t VLC_INCREASE_SET_SPEED : 1;
    ubit8_t VLC_DECREASE_SET_SPEED : 1;
    ubit8_t DECEL_MODE : 1;

    ubit8_t LIM_INCREASE_SET_SPEED : 1;
    ubit8_t LIM_DECREASE_SET_SPEED : 1;
    ubit8_t SPEED_STEP_1 : 1;
    ubit8_t CANCEL_FUNKTION : 1;
    ubit8_t DISENGAGE_DRIVER_INTERVENED : 1;
    ubit8_t RESET_SETSPEED : 1;
    ubit8_t SWITCH_SPEED_UNIT : 1;
    ubit8_t DRIVE_OFF_CONFIRM : 1;
} cc_driver_operations_t;

/*! @brief Status bits for the driver requests */
typedef struct cc_driver_request_status_t {
    ubit8_t PROPOSE_RECOMMENDED_SPEED : 1;  // 0: No recommended speed. 1: set
                                            // recommended speed.
    ubit8_t ACTUAL_SPEED_TAKEN_OVERRIDE : 1;
    ubit8_t ACTUAL_SPEED_TAKEN_OBJECT : 1;
    ubit8_t OVERRIDE_WHILE_ENGAGEMENT : 1;
    ubit8_t SELECTED_FUNCTION_ACTIVE : 1;
    ubit8_t TIMED_INCREMENT : 1;
    ubit8_t TIMED_DECREMENT : 1;
} cc_driver_request_status_t;

/*! @brief Data that reflects the drivers request intention */
typedef struct cc_driver_requests_t {
    cc_driver_operations_t DRIVER_OPERATIONS;
    cc_driver_operations_t DRIVER_OPERATIONS_LAST_CYCLE;
    cc_engagement_conditions_t ENGAGEMENT_CONDITIONS;
    display_op_status_t OPERATIONAL_MODE;
    cc_control_state_t CONTROL_STATE_LAST_CYCLE;
    cc_control_state_t CONTROL_STATE;
    cc_control_state_t CONTROL_STATE_PLIM;
    cc_control_mode_t CONTROL_MODE;
    setspeed_t VLC_SETSPEED;
    setspeed_t MIN_VLC_SETSPEED;
    setspeed_t MAX_VLC_SETSPEED;
    setspeed_t MIN_VLC_ENGAGESPEED;
    setspeed_t MAX_VLC_SPEED;
    setspeed_t VLC_DISENGAGE_THRESHOLD;
    setspeed_t RECOMMENDED_SPEED;
    setspeed_t SETSPEED_STEP_LEVEL_1;
    setspeed_t SETSPEED_STEP_LEVEL_2;
    speedometer_speed_t SPEEDOMETER_VEHICLE_SPEED;
    speedometer_speed_t SPEEDOMETER_VEHICLE_SPEED_OFFSET;
    cc_driver_request_status_t DRIVER_REQUEST_STATUS;
    times_long_t DRIVE_OFF_TIME;
    times_t STAND_STILL_TIME;
    times_t MOVING_TIME;
} cc_driver_requests_t;

/*! @brief CC SystemStatus */
typedef struct cc_system_status_t {
    ubit8_t SHUTDOWN_RAMP : 1;
    ubit8_t SHUTDOWN_SIGNAL : 1;
    ubit8_t DISENGAGE_SIGNAL : 1;
    ubit8_t : 5;
} cc_system_status_t;

/*! @brief CC error data*/
typedef struct cc_error_data_t {
    cc_reported_error_t REPORTED_ERROR;
    ubit8_t VLC_INHIBIT : 1;         /* CC can not be activated */
    ubit8_t LIM_INHIBIT : 1;         /* LIM can not be activated */
    ubit8_t SHUTDOWN_RAMP : 1;       /* Switch-Off with ramp */
    ubit8_t SHUTDOWN_DECEL_ONLY : 1; /* Switch-Off decel only */
    ubit8_t DISENGAGE_SIGNAL : 1;    /* Switch-Off msg 1 */
    ubit8_t SHUTDOWN_SIGNAL : 1;     /* Switch-Off msg 2 */
    ubit8_t : 2;
} cc_error_data_t;

/*! @brief CC status data*/
typedef struct cc_status_t {
    cc_driver_requests_t VLC_DRIVER_REQUESTS;
    cc_acceleration_control_data_t VLC_ACCEL_CONTROL_DATA;
    cc_control_data_t VLC_CONTROL_DATA;
    cart_lodm_status_t LODM_STAT_LAST_CYCLE;
    acceleration_t VLC_LAT_ACCEL_LAST_CYCLE;
    gradient_t VLC_LAT_ACCEL_GRAD_LAST_CYCLE;
    uint16 VLC_ENGAGE_OVERRIDE_COUNTER;
    uint16 VLC_ABS_ACT_CNT;
    uint16 VLC_ESP_ACT_CNT;
    uint16 VLC_TCS_ACT_CNT;
    uint32 VLC_DECEL_ONLY_MODE_CNT;
    uint8 VLC_SPEED_UNIT_LAST_CYCLE;
} cc_status_t;

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void VLC_INIT(cc_input_data_t* cc_input,
                     cc_driver_controls_t* driver_controls,
                     cc_status_t* cc_status);
DLLEXPORT void VLC_EXEC(const times_t cycle_time,
                        cc_driver_controls_t* driver_controls,
                        const cc_input_data_t* cc_input,
                        cc_error_data_t* error_data,
                        const cart_das_input_data_t* das_input,
                        cart_das_output_data_t* das_output,
                        const VLC_acc_output_data_t* acc_output,
                        const t_CamLaneInputData* pCamLaneData,
                        cc_status_t* cc_status,
                        PACCInfo* p_pacc_info);

#endif

/** @} end defgroup */
