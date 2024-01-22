

#ifndef VLC_LONG_VEH_H_INCLUDED
#define VLC_LONG_VEH_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

//#include "vlc_veh.h"
#include "vlc_long_veh_ext.h"
#include "vlc_long_cfg.h"

/* Only process rest of file if CC enabled and long control enabled */

#include "vlc_types.h"
#include "cart_ext.h"
#include "cc_ext.h"
#include "veh_sim.h"
#include "isa.h"
#include "pacc.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OBJ_LOSS_NO_INFO
#define OBJ_LOSS_NO_INFO 0U
#endif
#ifndef OBJ_LOSS_DISAPPEARED
#define OBJ_LOSS_DISAPPEARED 1U
#endif
#ifndef OBJ_LOSS_LANE_CHG_LEFT
#define OBJ_LOSS_LANE_CHG_LEFT 2U
#endif
#ifndef OBJ_LOSS_LANE_CHG_RIGHT
#define OBJ_LOSS_LANE_CHG_RIGHT 3U
#endif
#ifndef OBJ_LOSS_CURVE_LEFT
#define OBJ_LOSS_CURVE_LEFT 4U
#endif
#ifndef OBJ_LOSS_CURVE_RIGHT
#define OBJ_LOSS_CURVE_RIGHT 5U
#endif
#ifndef OBJ_LOSS_CURVE_LEFT_AHEAD
#define OBJ_LOSS_CURVE_LEFT_AHEAD 6U
#endif
#ifndef OBJ_LOSS_CURVE_RIGHT_AHEAD
#define OBJ_LOSS_CURVE_RIGHT_AHEAD 7U
#endif
#ifndef OBJ_LOSS_STEER_LEFT
#define OBJ_LOSS_STEER_LEFT 8U
#endif
#ifndef OBJ_LOSS_STEER_RIGHT
#define OBJ_LOSS_STEER_RIGHT 9U
#endif
#ifndef OBJ_LOSS_RANGE_REDUCTION
#define OBJ_LOSS_RANGE_REDUCTION 10U
#endif

#if VLC_LONG_VEH_DEBUG == 1

/*****************************************************************************
  MACROS
*****************************************************************************/

/* Definition of the required MTS alignment, added to unions to assure that
the alignment requirements are met */
#define MTS_ALIGNMENT_DUMMY uint32 MTS_DUMMY;

/*! Define cycle-id for VLC_LONG_VEH use (formerly COMP_ID_VLC) */
#define VLC_LONG_VEH_FUNC_ID VLC_MEAS_FUNC_ID

/*! Define channel-ids for different input/output/internal channels of VLC_LONG
 */
#define VLC_VEH_INPUT_CHANID VLC_MEAS_FUNC_CHAN_ID
#define VLC_VEH_OUTPUT_CHANID VLC_MEAS_FUNC_CHAN_ID
#define VLC_VEH_INTERN_CHANID VLC_MEAS_FUNC_CHAN_ID

#define CFG_VLC_USE_TWICE_CONFIRMATION_FOR_DRIVE_OFF 0

/*****************************************************************************
  MODULE GLOBAL TYPEDEFS
*****************************************************************************/
typedef struct cc_das_custom_state_struct {
    ubit8_t VLC_CUSTOM_DRIVER_OUTSIDE_HOLD : 1;
    ubit8_t VLC_CUSTOM_STAND_STILL_HOLD : 1;
    ubit8_t : 6;
} cc_das_custom_state_t;

/*! State type for custom stop&go logic */
#define Cc_moving ((cc_das_stop_go_state_t)0)
#define Cc_standing ((cc_das_stop_go_state_t)1)
#define Cc_go_request ((cc_das_stop_go_state_t)2)
#define Cc_go_moving ((cc_das_stop_go_state_t)3)
typedef enum_t cc_das_stop_go_state_t;

/* HMI Output */
typedef struct cc_driver_information_t {
    uint8 OBJECT_DISTANCE;    /*!< The relevant object distance @unit:m */
    uint8 REQUESTED_DISTANCE; /*!< The requested distance setting */
    setspeed_t SET_SPEED;     /*!< The set speed @unit:km/h or mp/h */
    setspeed_t
        OBJECT_SPEED; /*!< Speed of the relevant object @unit:km/h or mp/h */
    uint8 BIT_FCA_ALERT;
    uint8 BIT_AVLC_ALERT;
    uint8 BIT_DM_ALERT;
    uint8 DM_STATE;
    setspeed_t RECOMMENDED_SPEED;
    percentage_t HEADWAY_SETTING;
    uint8 AVLC_DRIVE_OFF_POSSIBLE;
    display_op_status_t OPERATIONAL_MODE;
    uint8 REPORTED_ERROR;
    boolean OBJECT_DETECTED;
    uint8 DRIVER_CONFIRMATION_NEEDED;
} cc_driver_information_t;

/* HMI Input */
typedef struct cc_driver_inputs_t {
    switch_t VLC_MAIN_SWITCH;  // ACC Will be ON => Ready

    switch_t AVLC_MODE_SWITCH;  // Mode change between ACC and CC

    switch_t VLC_SET_SWITCH;          // Used as SET(Taking Current speed as the
                                      // Set Speed) changed by heqiushu 20211108
    switch_t VLC_RESUME_SWITCH;       // Used as RESUME(Taking the previously
                                      // set speed as the Set Speed)
    switch_t VLC_CANCEL_SWITCH;       // Cancelling ACC
    switch_t VLC_ACCEL_SWITCH_1;      // Used both as /SPEED+1
    switch_t VLC_ACCEL_SWITCH_2;      // Used both as /SPEED+5
    switch_t VLC_DECEL_SWITCH_1;      // Used both as /SPEED-1
    switch_t VLC_DECEL_SWITCH_2;      // Used both as /SPEED-5
    switch_t VLC_HEADWAY_INC_SWITCH;  // Headway Increment Switch
    switch_t VLC_HEADWAY_DEC_SWITCH;  // Headway Decrement Switch
    switch_t VLC_HEADWAY_SWITCH;  // Headway cycle setting switch button, added
                                  // by guotao 20200716
} cc_driver_inputs_t;

/* Headway Setting as required by SW18 */
/* HEADWAY_SETTING_LEVEL0 = 0 % meaning 1 sec of Timegap */
/* HEADWAY_SETTING_LEVEL6 = 100 % meaning 2.4 sec of Timegap */
#define HEADWAY_SETTING_LEVEL0 (percentage_t)0
#define HEADWAY_SETTING_LEVEL1 (percentage_t)25
#define HEADWAY_SETTING_LEVEL2 (percentage_t)50
#define HEADWAY_SETTING_LEVEL3 (percentage_t)75
#define HEADWAY_SETTING_LEVEL4 (percentage_t)100

// cycle headway setting level for demo only, added by guotao 20200716
#define CYCLE_HEADWAY_SETTING_LEVEL0 (percentage_t)0
#define CYCLE_HEADWAY_SETTING_LEVEL1 (percentage_t)25
#define CYCLE_HEADWAY_SETTING_LEVEL2 (percentage_t)50
#define CYCLE_HEADWAY_SETTING_LEVEL3 (percentage_t)75
#define CYCLE_HEADWAY_SETTING_LEVEL4 (percentage_t)100

#define HEADWAY_SETTING_OUTPUT_VAL0 1u
#define HEADWAY_SETTING_OUTPUT_VAL1 2u
#define HEADWAY_SETTING_OUTPUT_VAL2 3u
#define HEADWAY_SETTING_OUTPUT_VAL3 4u
#define HEADWAY_SETTING_OUTPUT_VAL4 5u

/*ACC DisplayOutput Values*/
#define NO_DISPLAY 0u
#define DOOR_OPEN 1u
#define SEATBELT_UNPLUCKED 2u
#define NO_FORWARD_GEAR 4u
#define EPB_ACTIVATED 8u
#define ESP_OFF 16u
#define SPEED_OVER_150KPH 32u
#define RSM_BLINDNESS 64u
#define UNABLE_TO_ACTIVATE_ACC 128u
#define DRIVE_OFF_REQUEST 256u
#define RSM_SENSOR_ALIGNMENT_INCOMPLETE 512u
#define AVLC_SWITCHED_ON 1024u
#define AVLC_SWITCHED_OFF 2048u
#define STANDWAIT_OVER_TIME_LIMIT 4096u
#define RADAR_ERROR 8192u

#define AVLC_INHIBITION_ALIGNMENT_BIT 2U
#define AVLC_INHIBITION_PARTIAL_BLOCKAGE_BIT 3U

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
 APPLICATION PARAMETERS
*****************************************************************************/

/*****************************************************************************
  MODULE GLOBAL VARIABLES
*****************************************************************************/
union {
    cart_das_input_data_t DAS_INPUT_DATA; /*!< The real data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gDAS_INPUT_DATA;      /*!< @VADDR:0x20021000 @CYCLEID:VLC_VEH */

cart_das_input_data_t DAS_INPUT_DATA_LAST_CYCLE;
union {
    cart_das_output_data_t DAS_OUTPUT_DATA; /*!< The real data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gDAS_OUTPUT_DATA;     /*!< @VADDR:0x20021100 @CYCLEID:VLC_VEH */

union {
    cc_input_data_t VLC_INPUT_DATA; /*!< Cruise control input data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gVLC_INPUT_DATA;      /*!< @VADDR:0x20021400 @CYCLEID:VLC_VEH */

union {
    cc_driver_controls_t
        VLC_DRIVER_CONTROLS; /*!< The cruise control driver controls */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gVLC_DRIVER_CNTRLS;   /*!< @VADDR:0x20021500 @CYCLEID:VLC_VEH */

union {
    cc_error_data_t VLC_ERROR_DATA; /*!< The cruise control error data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gVLC_ERROR_DATA;      /*!< @VADDR: 0x20021700 @CYCLEID:VLC_VEH */

union {
    cc_status_t VLC_STATUS; /*!< The cruise control status information */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gVLC_STATUS;          /*!< @VADDR: 0x20021800 @CYCLEID:VLC_VEH */

/*specific customer information*/
union {
    cc_driver_information_t VLC_DRIVER_INF; /*!< The VLC_DRIVER_INFORMATION */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} VLC_DRIVER_INF;       /*!< @VADDR:0x20021600 @CYCLEID:VLC_VEH */

union {
    cc_driver_inputs_t
        VLC_DRIVER_INPUTS; /*!< The driver inputs to CC function */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gVLC_DRIVER_INPUTS;   /*!< @VADDR:0x20021900 @CYCLEID:VLC_VEH */

union {
    struct {
        uint16 Inhibit_nr; /*!< The currently active inhibition number from
                              INHIBIT_BUFFER.INHiBIT_NR */
    } VLC_VEH_DEBUG_DATA;
    MTS_ALIGNMENT_DUMMY
} gVLC_VEH_DEBUG_DATA; /*!< @VADDR:0x20029900 @CYCLEID:VLC_VEH */

#endif
/*****************************************************************************
  FUNKTIONEN (KOMPONENTENINTERN)
*****************************************************************************/

#ifdef __cplusplus
};
#endif

/* Ende der bedingten Einbindung */
#else
#endif
