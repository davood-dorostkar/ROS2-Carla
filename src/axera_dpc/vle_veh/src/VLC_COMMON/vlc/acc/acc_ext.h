
#ifndef AVLC_EXT_H
#define AVLC_EXT_H

/** @defgroup vlc_long_acc VLC_LONG_ACC ( Adaptive Cruise Control )
 Interface to the component ACC. The component ACC is responsible for the
 calculation of an appropiate distance contol acceleration, to be delivered to
 the kinematik part of the cruise control function.

  - A functional specification can be found under <A
 HREF="../../../acc_function_spec.mht">acc_function_spec.mht</A>
    - A functional specification for FSRA can be found under <A
 HREF="../../../fsr_acc_function_spec.mht">fsr_acc_function_spec.mht</A>
  - A design specification can be found under <A
 HREF="../../../vlc_long_design_spec.mht">vlc_long_design_spec.mht</A>

   @ingroup acc_long_sen

   @{ */

/* includes */
//#include "vlc_long_sen.h"
#include "vlcSen_ext.h"
#include "acc_out_ext.h"
#include "brake_ext.h"
#include "cart_ext.h"
#include "switch_ext.h"

/*! @brief HMI Input */
typedef struct acc_driver_controls_t {
    percentage_t P_LANE_CHANGE_LEFT; /*!< probability of a lane change into left
                                        direction*/
    percentage_t P_LANE_CHANGE_RIGHT; /*!< probability of a lane change into
                                         right direction*/
    percentage_t HEADWAY_SETTING;     /*!< actual headway setting in percent*/
} acc_driver_controls_t;

/*! @brief Status bits for the ACC algorithm */
typedef struct acc_input_status_t {
    ubit8_t AVLC_ON : 1;          /*!< acc is activated*/
    ubit8_t INHIBIT : 1;          /*!< acc cannot be engaged*/
    ubit8_t OBJECT_EFFECTIVE : 1; /*!< the object is effective for control*/
    ubit8_t VLC_DECEL_LIM_OVERRIDE : 1; /*!< the host vehicle uses limited
                                           deceleration due to an previously
                                           occurred override situation*/
    ubit8_t
        OBJECT_LOST : 1; /*!< the object got lost due to a sensor limitation*/
    ubit8_t AVLC_CONTROL_TO_FIRST_OBJECT : 1; /*!< the acc will ever control to
                                                 the first object (although it
                                                 could be stationary)*/
    ubit8_t : 2;
} acc_input_status_t;

typedef struct acc_lodm_status_t {
    ubit8_t STANDSTILL : 1;     /*!< Ego vehicle in stand still */
    ubit8_t OVERRIDE_ACCEL : 1; /*!< Driver override of accelerator pedal */
} acc_lodm_status_t;

/*! @brief ACC input interface: vehicle related data */
typedef struct acc_input_data_t {
    velocity_t LONG_VELOCITY; /*!< longitudinal velocity of the host vehicle
                                 LONG_EXEC */
    acceleration_t LONG_ACCELERATION; /*!< longitudinal acceleration of the host
                                         vehicle LONG_EXEC */
    acc_input_status_t INPUT_STATUS;  /*!< see details LONG_EXEC */
    acceleration_t VLC_ACCEL_LIMIT;   /*!< maximum allowed acceleration for this
                                         function LONG_EXEC*/
    acceleration_t VLC_DECEL_LIMIT;   /*!< maximum allowed deceleration for this
                                         function LONG_EXEC */
    distance_t VISIBILITY_RANGE; /*!< maximum longitudinal displacement at which
                                    objects can be reported*/
    acc_lodm_status_t LODM_STAT; /*!< see details (status of longitudinal
                                    dynamics manager) LONG_EXEC */
    acc_driver_controls_t DRIVER_CONTROLS; /*!< see details (driver inputs for
                                              acc) LONG_EXEC, LC_EXEC */
} acc_input_data_t;

/*! @brief Result of the ACC alert algorithm */
typedef struct acc_alert_data_t {
    distance_t ESTIM_DISTANCE;
    velocity_t ESTIM_REL_SPEED;
    velocity_t ESTIM_HOST_SPEED;
    factor_t ALERT_THRES_FACTOR;
    distance_t ALERT_DISTANCE;
    velocity_t ALERT_REL_SPEED;
    times_t ALERT_SUPRESS_TIME;
} acc_alert_data_t;

/*! @brief Internal information about decel limiter for ACC*/
typedef struct acc_decel_lim_override_t {
    ubit8_t VLC_LIMITER_ACTIVE : 1;
    ubit8_t AVLC_LIMITER_ACTIVE : 1;
    ubit8_t : 6;
} acc_decel_lim_override_t;

/*! @brief driver intention for ACC */
typedef struct acc_driver_intention_t {
    percentage_t LANE_CHANGE_LEFT_PROBABILITY; /*!< probability of a lane change
                                                  to left direction*/
    percentage_t LANE_CHANGE_RIGHT_PROBABILITY; /*!< probability of a lane
                                                   change to right direction*/
    acc_decel_lim_override_t
        DECEL_LIM_OVERRIDE; /*!< information if decel limiter after override is
                               active*/
    acceleration_t
        DECEL_LIM_OVERRIDE_ACCEL; /*!< max allowed acceleration after override*/
    switch_t AVLC_ENAGAGED;       /*!< acc is engaged*/
    ObjNumber_t
        OBJECT_NR_TO_CONTROL_TO; /*!< object nr that is used to control
                                    regardless if it is stationary or not!*/
} acc_driver_intention_t;

/*! @brief handles all internal status information of acc component*/
typedef struct acc_status_t {
    acc_driver_intention_t AVLC_DRIVER_INTENTION; /*!< driver intention*/
    VLC_acc_object_t AVLC_CONTROL_OBJECT;         /*!< control object for ACC*/
    acc_alert_data_t AVLC_ALERT_DATA;             /*!< alert data*/
    distance_t REQUESTED_DISTANCE_ACT;       /*!< current requested distance*/
    distance_t REQUESTED_DISTANCE_PREDICTED; /*!< current predicted requested
                                                distance*/
} acc_status_t;
/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void AVLC_INIT(VLC_acc_object_t *display_object,
                      VLC_acc_object_t *alert_object,
                      acc_status_t *acc_status);
DLLEXPORT extern void AVLC_EXEC(
    const times_t cycle_time,
    const VLCCustomInput_t *pVLCCustomInput,
    const VLC_DFV2SenInfo_t *pDFVLongOut,
    acc_input_data_t *input,
    VLC_acc_object_t object_list[Acc_max_number_ooi],
    VLC_acc_output_data_t *output,
    VLC_acc_object_t *alert_object,
    VLC_acc_object_t *display_object,
    acc_status_t *acc_status,
    OvertakeAssistInfo *p_overtake_assist_info);
extern void AVLC_DELETE_OBJECT(VLC_acc_object_t *const object);

#endif

/** @} end defgroup */
