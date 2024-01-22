/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "string.h"

#include "TM_Global_Const.h"
#include "TM_Math_Cal.h"
#include "vlc_veh.h"

#include "acc_par.h" /*!< Needed for Acc_default_headway_setting */
#include "cart_ext.h"
#include "cc_ext.h"
#include "cc_par.h"
#include "isa.h"
#include "mat_param_ext.h"
#include "mat_std_ext.h"
#include "pacc.h"
#include "switch_ext.h"
#include "vlc_inhibit_ext.h"
#include "vlc_long_veh.h"
#include "vlc_par.h"

#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*! [ms] Time for which if the driver stays in Standstill state, ego vehicle
 * should move to Standby state from Active Standstill wait*/
/*10 minutes + 3 seconds*/
volatile float32 VLC_T_STANDSTILL_WAIT_ELAPSED = 603.0f;

/*HDC active time threshold 0.5 seconds*/
volatile float32 VLC_T_HDC_ACTIVE_ELAPSED = 0.5f;

/*ABS active time threshold 1 seconds*/
volatile float32 VLC_T_ABS_ACTIVE_ELAPSED = 1.0f;

/*TCS active time threshold 0.5 seconds*/
volatile float32 VLC_T_TCS_ACTIVE_ELAPSED = 0.5f;

/*TCS active time threshold 0.5 seconds*/
volatile float32 VLC_T_ARP_ACTIVE_ELAPSED = 0.5f;

/*Seat belt loose time threshold 2 seconds*/
volatile float32 VLC_T_SEAT_BELT_ACTIVE_ELAPSED = 2.0f;

// min distance for drive away situation
volatile float32 DRIVE_OFF_LOW_ACCEL_REQ_DIST = 5.0f;

// min distance to request DRIVE_OFF_MIN_ACCEL_REQ in drive away situation
volatile float32 DRIVE_OFF_HIGH_ACCEL_REQ_DIST = 3.0f;

// min velocity(kph) to request DRIVE_OFF_HIGH_ACCEL_REQ_DIST in drive away
// situation
volatile float32 DRIVE_OFF_HIGH_ACCEL_REQ_VEL = 1.5f;

volatile float32 VLC_CURVE_RADIUS_ACC_INHIBIT = 20.0;
volatile float32 VLC_STEER_ANGLE_ACC_EXIT = 360.0;
volatile float32 VLC_STEER_SPEED_ACC_EXIT = 540.0;

// volatile float32 MOTION_STATE_CONFIDENCE = 85;
volatile float32 MOTION_STATE_CONFIDENCE = 60;

#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#if VLC_LONG_VEH_DEBUG == 0
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
typedef struct cc_das_custom_state_struct
{
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
typedef struct cc_driver_information_t
{
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
typedef struct cc_driver_inputs_t
{
    switch_t VLC_MAIN_SWITCH; // ACC Will be ON => Ready

    switch_t AVLC_MODE_SWITCH; // Mode change between ACC and CC

    switch_t VLC_SET_SWITCH;         // Used as SET(Taking Current speed as the
                                     // Set Speed)
    switch_t VLC_RESUME_SWITCH;      // Used as RESUME(Taking the previously
                                     // set speed as the Set Speed)
    switch_t VLC_CANCEL_SWITCH;      // Cancelling ACC
    switch_t VLC_ACCEL_SWITCH_1;     // Used both as /SPEED+1
    switch_t VLC_ACCEL_SWITCH_2;     // Used both as /SPEED+5
    switch_t VLC_DECEL_SWITCH_1;     // Used both as /SPEED-1
    switch_t VLC_DECEL_SWITCH_2;     // Used both as /SPEED-5
    switch_t VLC_HEADWAY_INC_SWITCH; // Headway Increment Switch
    switch_t VLC_HEADWAY_DEC_SWITCH; // Headway Decrement Switch
    switch_t VLC_HEADWAY_SWITCH;     // Headway cycle setting switch button, added
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

// calibrate to 0 if want exiting ACC when driver pressed brake in stationary
#define STAND_STILL_BRAKE_EXIT_ACC 0

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
 APPLICATION PARAMETERS
*****************************************************************************/

/*****************************************************************************
  MODULE GLOBAL VARIABLES
*****************************************************************************/

static union
{
    cart_das_input_data_t DAS_INPUT_DATA; /*!< The real data */
    MTS_ALIGNMENT_DUMMY                   /*!< Needed to align the data to the MTS requirement */
} gDAS_INPUT_DATA;                        /*!< @VADDR:0x20021000 @CYCLEID:VLC_VEH */

static cart_das_input_data_t DAS_INPUT_DATA_LAST_CYCLE;
static union
{
    cart_das_output_data_t DAS_OUTPUT_DATA; /*!< The real data */
    MTS_ALIGNMENT_DUMMY                     /*!< Needed to align the data to the MTS requirement */
} gDAS_OUTPUT_DATA;                         /*!< @VADDR:0x20021100 @CYCLEID:VLC_VEH */

static union
{
    cc_input_data_t VLC_INPUT_DATA; /*!< Cruise control input data */
    MTS_ALIGNMENT_DUMMY             /*!< Needed to align the data to the MTS requirement */
} gVLC_INPUT_DATA;                  /*!< @VADDR:0x20021400 @CYCLEID:VLC_VEH */

static union
{
    cc_driver_controls_t
        VLC_DRIVER_CONTROLS; /*!< The cruise control driver controls */
    MTS_ALIGNMENT_DUMMY      /*!< Needed to align the data to the MTS requirement */
} gVLC_DRIVER_CNTRLS;        /*!< @VADDR:0x20021500 @CYCLEID:VLC_VEH */

static union
{
    cc_error_data_t VLC_ERROR_DATA; /*!< The cruise control error data */
    MTS_ALIGNMENT_DUMMY             /*!< Needed to align the data to the MTS requirement */
} gVLC_ERROR_DATA;                  /*!< @VADDR: 0x20021700 @CYCLEID:VLC_VEH */

static union
{
    cc_status_t VLC_STATUS; /*!< The cruise control status information */
    MTS_ALIGNMENT_DUMMY     /*!< Needed to align the data to the MTS requirement */
} gVLC_STATUS;              /*!< @VADDR: 0x20021800 @CYCLEID:VLC_VEH */

/*specific customer information*/
static union
{
    cc_driver_information_t VLC_DRIVER_INF; /*!< The VLC_DRIVER_INFORMATION */
    MTS_ALIGNMENT_DUMMY                     /*!< Needed to align the data to the MTS requirement */
} VLC_DRIVER_INF;                           /*!< @VADDR:0x20021600 @CYCLEID:VLC_VEH */

static union
{
    cc_driver_inputs_t
        VLC_DRIVER_INPUTS; /*!< The driver inputs to CC function */
    MTS_ALIGNMENT_DUMMY    /*!< Needed to align the data to the MTS requirement */
} gVLC_DRIVER_INPUTS;      /*!< @VADDR:0x20021900 @CYCLEID:VLC_VEH */

static union
{
    struct
    {
        uint16 Inhibit_nr; /*!< The currently active inhibition number from
                              INHIBIT_BUFFER.INHiBIT_NR */
    } VLC_VEH_DEBUG_DATA;
    MTS_ALIGNMENT_DUMMY
} gVLC_VEH_DEBUG_DATA; /*!< @VADDR:0x20029900 @CYCLEID:VLC_VEH */
#endif

static vlc_inhibit_storage_t INHIBIT_BUFFER;

/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/
#define VEHILCE_TYPE LINCOLN_MKZ

#define COMFORT_HYSTERESIS ((acceleration_t)300)
#define DYNAMIC_HYSTERESIS ((acceleration_t)0)
#define MAX_CRITICALITY_DYNAMIC_MODE ((confidence_t)2)
#define DIVISIOR_2 (2)
#define MAX_VALUE_UNIT8 (255)
#define MAX_SETSPEED (255u)

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/
/*! Defines of VLC_LONG_VEH_WRAPPER_OUTPUT */
#define THREE_CYCLES (3u)
#define STOP_ENGINE_TIME (times_t)4000
#define STOP_ENGINE_MAX_TARGET_DIST \
    (5.f) /*!< if target vehicle distance over 5 meters, start the engine */
#define STOP_ENGINE_MAX_TARGET_VEL_X \
    (1.f) /*!< if target vehicle velocity over 1 m/s, start the engine */
#define DRIVE_OFF_VALID_MIN_ACCEL \
    (acceleration_t)50 /*!< min acceleration for drive off request  */
#define DRIVE_OFF_NOT_VALID_MIN_ACCEL \
    (acceleration_t)(                 \
        -350) /*!< requested accel for breaking the drive away logic */
#define DRIVE_OFF_MIN_ACCEL_REQ \
    (float32)1.f /*!< min accel request in drive away situation */
#define DRIVE_OFF_MIN_ACCEL_REQ_CLOSE_DIST                                                                   \
    (float32)0.25f                                    /*!< min accel request in drive away situation for too \
                                                         close distances */
#define MAX_EGO_SPEED_FOLLOW_TO_STOP (velocity_t)1100 /* 40 kph */
#define MAX_OBJ_SPEED_FOLLOW_TO_STOP (2.8f)           /* 10 kph */
#define MAX_DRIVE_OFF_MIN_ACCEL_REQ_TIME (times_t)3000

/*! Defines of VLC_INHIBITION_CHECK */
#define DISENGAGE_VELOCITY_MPS (velocity_t)7400 /* velocity 74 m/s  */

/*! Defines of VLC_INFORM_DRIVER */
#define Cc_init_display_distance (uint8)0
#define Cc_max_requested_distance (uint8)150
#define Cc_init_display_object_distance (uint8)254
#define Cc_init_setspeed (setspeed_t)253
#define Cc_min_display_object_speed (setspeed_t)0
#define Cc_max_display_object_speed (setspeed_t)255

#define Cc_min_suppress_speed_override (speedometer_speed_t)500

#define Cc_driver_operation_display_time (times_t)5000
#define Cc_minimum_display_time (times_t)2000
#define Cc_display_retrigger_time \
    (times_t)600 /*!< Time length of the Retrigger Signal */
#define Cc_display_delay_time \
    (times_t)300 /*!< Display object information delay time, default 300 ms */
#define Cc_init_headway_setting_time (times_t)1000
#define Cc_init_alert_override_time (times_t)600

#define MAX_EXRAPOLATION_DEVIATION (199u)
#define MAX_SLA_SPEED_KPH (setspeed_t)160
#define MAX_SLA_SPEED_MPH (setspeed_t)95
#define UNRESTRICTED_SLA_SPEED (setspeed_t)253
#define VLC_INIT_SLA_SPEED_APPLIED_TIME (times_t)2000
#define VLC_INIT_SLA_OVERRIDE_TIME (times_t)1000
#define VLC_SLA_SPEED_LIMIT_PREVENTION_TIME (times_t)1500
#define VLC_SLA_SPEED_LIMIT_ACTIVE_TIME (times_t)500
#define VLC_SLA_ABORT_DIST_BEFOR_SPEED_LIMIT (t_u_DistanceLong)65000
#define VLC_SLA_NEXT_PREVIEW_MAX_ACCEL_RQ ((acceleration_t)-1100)
#define VLC_SLA_TSA_WARN_SUPP_SPEED_DIFF (speedometer_speed_t)500
#define VLC_RWD_SPEED_ACC_INHIBIT (speedometer_speed_t)300

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean VLC_LONG_VEH_INITIALIZED = FALSE;

/*! Custom specific states for the state mashine */
static cc_das_custom_state_t cc_das_custom_state;

static percentage_t HeadwayThumbSettingLC = Acc_default_headway_setting;

static percentage_t HeadwaySetting = Acc_default_headway_setting;

/*! Variables of VLC_LONG_VEH_WRAPPER_OUTPUT */
static cc_das_stop_go_state_t
    cc_das_stop_go_state; /*!< State variable for custom stop&go logic */
static uint8 drive_off_cycle_counter = 0u;
static times_t stop_start_cycle_counter = (times_t)0;
static boolean b_stop_start_flag = TRUE; /* engine stop allowed */
static boolean b_drive_off_confirm_last = FALSE;
static uint16 esc_active_counter = 0u;
static uint16 abs_active_counter = 0u;
static uint16 tcs_active_counter = 0u;
static uint16 arp_active_counter = 0u;
static uint16 seat_belt_unbuckle_counter = 0u;

/* date: 2018-04-03 reason: static local required for saving data across
 * function calls */
static times_t standstill_delay_cycle_counter =
    0u;                                 /*SW18, to check if the ego vehicle stays in standstill condition for
                                           more than 3 minutes*/
static ACC_OOI_Status_t acc_ooi_status; // estimate whether OOI has disappeared
                                        // suddenly. if yes, ACC will exit.
static times_t drive_off_min_accel_rq_timer = (times_t)0;

static setspeed_t set_speed_last_cycle = (setspeed_t)0;
static ObjNumber_t last_effective_object_id = OBJ_INDEX_NO_OBJECT;

/*! Variables of VLC_INFORM_DRIVER */
static times_t uiOperationalTimer = 0u;

static times_t uiReTriggerSetTimer = Cc_driver_operation_display_time;
static times_t uiReTriggerRequestTimer = 0u;
static times_t uiReTriggerRepeatTimer = 0u;
static times_t LDC_DTR_MsgDisp_Rq_RDU_Timer = 0u;

static times_t uiDisplayDelayTimer = Cc_display_delay_time;
static times_t uiHeadwayDisplayTimer = 0u;

static times_t uiAlertOverrideTimer = 0u;
static boolean b_AlertTimerInit = FALSE;

static ObjNumber_t last_object_id = OBJ_INDEX_NO_OBJECT;
static boolean bDriverInterventedOff = FALSE;
static boolean bHeadwayChangeDetected = FALSE;

static times_t uiInitHeadwaySettingTimer = Cc_init_headway_setting_time;

boolean standwait_over_time_limit = FALSE;

static boolean drive_off_confirm_on_resume_pressed =
    FALSE; /*SW18, to confirm Drive-Off in Standwait Mode when Resume Button is
              pressed on HMI*/

/* for testing only */
static boolean test_off = 1;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULE LOCAL FUNCTIONS
*****************************************************************************/
static void VLC_LONG_VEH_MTS_CALLBACK(void);
void VLC_LONG_VEH_INIT(void);
static boolean VLCConvertGearInfo(TransmissionGear_t Input, pt_gear_t *pOutput);
static void VLC_OOI_DISAPPEAR_CHECK(const VLCSenAccOOI_t *pVLCAccOOIData,
                                    ACC_OOI_Status_t *acc_ooi_status);

static void VLC_LONG_VEH_WRAPPER_INPUT(
    const times_t cycle_time,
    const VED_VehDyn_t *pVehDyn,

    const VLC_AccLeverInput_t *pAccLever,

    const VLC_LongCtrlInput_t *pLongCtrlResp,
    cc_driver_information_t *driver_information,
    cart_das_input_data_t *das_input_data,

    cc_driver_inputs_t *driver_inputs,

    cc_status_t *cc_status,
    VLC_DFV2SenInfo_t *pDFVLongOut,
    cc_driver_controls_t *driver_controls,
    cc_input_data_t *cc_input,
    cc_error_data_t *error_data);

static void VLC_SIGNAL_INHIBITION_CHECK(
    const VED_VehDyn_t *pVehDyn,
    const VLC_AccLeverInput_t *pAccLever,

    const VLC_LongCtrlInput_t *pLongCtrlResp,
    cc_input_data_t *cc_input_data,
    uint16 *Inhibit_nr);

static void VLC_INHIBITION_CHECK(const cart_das_input_data_t *das_input_data,
                                 const VLC_acc_output_data_t *acc_output,
                                 const VLCSenAccOOI_t *pVLCAccOOIData,
                                 const VED_VehDyn_t *pVehDyn,
                                 const VLC_LongCtrlInput_t *pLongCtrlResp,
                                 const PACCInfo *p_pacc_info,
                                 cc_input_data_t *input,
                                 cc_error_data_t *error_data,
                                 cc_status_t *cc_status);

static void VLC_DETERMINE_DRIVER_OPERATIONS(
    const times_t cycle_time,
    cart_das_input_data_t *das_input,
    const cc_input_data_t *input,
    cc_driver_controls_t *driver_controls,
    const cc_error_data_t *error_data,
    cc_status_t *cc_status,
    cc_driver_inputs_t *driver_inputs,
    cc_driver_information_t *driver_information,
    cart_das_output_data_t *das_output,
    const VLCSenAccOOI_t *pVLCAccOOIData,
    const ISAInfo *p_isa_info);

static void VLC_LIMIT_LONG_ACCEL_CUSTOM(void);

static void VLC_INFORM_DRIVER(const times_t cycle_time,
                              const cc_driver_requests_t *driver_requests,
                              const VLC_acc_output_data_t *acc_output,
                              const VLC_acc_object_t *display_object,
                              const cc_error_data_t *error_data,
                              const cart_das_input_data_t *das_input_data,
                              const cc_input_data_t *cc_input,
                              const cc_status_t *cc_status,
                              cc_driver_information_t *driver_information,
                              cart_das_output_data_t *das_output);

static void VLC_LONG_VEH_WRAPPER_OUTPUT(
    const times_t cycle_time,
    const reqVLCVehDebugList_t *pVLCVehDebugPorts,
    const cc_input_data_t *pCcInputData,
    const velocity_t vehicle_speed,
    const cart_das_output_data_t *das_output,
    const cc_status_t *cc_status,
    const VLC_acc_output_data_t *pAccOutput,
    const cc_driver_information_t *driver_information,
    const cc_error_data_t *error_data,
    const cart_das_input_data_t *das_input_data,
    VLC_LongCtrlOutput_t *pLongCtrlCmd);

static void VLC_LONG_VEH_TO_SEN_INFO(const VLC_LongCtrlInput_t *pLongCtrlResp,
                                     const cc_status_t *cc_status,
                                     VLC_DFV2SenInfo_t *pDFVLongOut);

static void VLC_LONG_VEH_DETERMINE_SOFT_STOP_REQUEST(
    const cart_das_output_data_t *pDasOutputData,
    const cc_driver_information_t *pDriverInformation,
    const velocity_t vehicle_speed,
    const VLC_acc_object_t *pAccDisplayObj,
    const cc_status_t *pCCStatus,
    const VLC_acc_output_data_t *pAccOutput,
    VLC_LongCtrlOutput_t *pLongCtrlCmd);

static void VLC_LONG_VEH_EXT_VLC_ARBITRATION(
    const times_t cycle_time,
    const velocity_t vehicle_speed,
    const cc_input_data_t *pCcInputData,
    const cc_status_t *p_cc_status,
    const cart_das_output_data_t *pDasOutputData,
    const cc_driver_information_t *pDriverInformation,
    VLC_LongCtrlOutput_t *pLongCtrlCmd);

/*************************************************************************************************************************
  Functionname:    VLC_OOI_DISAPPEAR_CHECK */
/*!

                                 @brief           Check if the OOI object disappear unnormally.

                                 @description     -

                                 @return          static void

                                 @param[in]       pVLCAccOOIData          the OOI object input
                                 @param[in]       EgoVehiclePitch         Pitch angle of ego vehilce

                                 @param[out]      acc_ooi_status
                                 @pre             None
                                 @post            None

                                 @created         He Qiushu 20211109
                                 @changed         -

                                 @todo            Review this function and add more conditions and logic

                               *************************************************************************************************************************/
void VLC_OOI_DISAPPEAR_CHECK(const VLCSenAccOOI_t *pVLCAccOOIData,
                             ACC_OOI_Status_t *acc_ooi_status)
{
    static sint8 acc_ooi_id_last;
    static float acc_ooi_distx_last;

    if (pVLCAccOOIData->AccOOINextLong.Attributes.uiObjectID !=
        acc_ooi_id_last)
    {
        // if OOI is very close to ego vehicle(means the distance is smaller
        // than 5m) and lost reason is disappearance, it can be considered
        // suddenly disappearance.
        if (pVLCAccOOIData->AccOOINextLong.eRelObjLossReason ==
                OBJ_LOSS_DISAPPEARED &&
            acc_ooi_distx_last < 5)
        {
            *acc_ooi_status = Disappear_Sudden;
        }
        else
        {
            *acc_ooi_status = Disappear_Normal;
        }
    }
    else if (pVLCAccOOIData->AccOOINextLong.Attributes.uiObjectID ==
                 OBJ_INDEX_NO_OBJECT &&
             acc_ooi_id_last == OBJ_INDEX_NO_OBJECT)
    {
        *acc_ooi_status = Not_Exist;
    }
    else
    {
        *acc_ooi_status = Exist;
    }

    acc_ooi_id_last = pVLCAccOOIData->AccOOINextLong.Attributes.uiObjectID;
    acc_ooi_distx_last = pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX;
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_VEH_INIT */
/*!

                                             @brief           initialize longitudinal function data

                                             @description     -

                                             @return          static void

                                             @param[in]       None

                                             @glob_in         None
                                             @glob_out        VLC_DRIVER_INF.VLC_DRIVER_INF.AVLC_DRIVE_OFF_POSSIBLE: is a takeoff possible (free way) [TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_AVLC_ALERT:											[TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_DM_ALERT:											[TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_FCA_ALERT:											[TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.DRIVER_CONFIRMATION_NEEDED: driver confimation needed for takeoff (resume button) [TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.HEADWAY_SETTING: default headway setting					[percentage_t as per Rte_Type.h]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.OBJECT_DETECTED: default object detected status is False [TRUE, FALSE]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.OPERATIONAL_MODE: ACC operational mode					[display_op_status_t as per Rte_Type.h]
                                             @glob_out		   VLC_DRIVER_INF.VLC_DRIVER_INF.REPORTED_ERROR: reported error type						[uint8 as per Platform_Types.h]

                                             @c_switch_part   CFG_VLC_LODM : Configuration Switch for using LODM signals e.g. ABS
                                             @c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing

                                             @pre             None
                                             @post            None

                                             @created         -
                                             @changed         -

                                             @todo            Review this function

                                           *************************************************************************************************************************/
void VLC_LONG_VEH_INIT(void)
{
    VLC_INIT(&gVLC_INPUT_DATA.VLC_INPUT_DATA,
             &gVLC_DRIVER_CNTRLS.VLC_DRIVER_CONTROLS, &gVLC_STATUS.VLC_STATUS);
    SWITCH_INIT_SWITCH(&gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_RESUME_SWITCH);
    SWITCH_INIT_SWITCH(&gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_CANCEL_SWITCH);
    SWITCH_INIT_SWITCH(&gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_MAIN_SWITCH);

    SWITCH_INIT_SWITCH(&gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.AVLC_MODE_SWITCH);

    SWITCH_INIT_SWITCH(&gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_SET_SWITCH);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_ACCEL_SWITCH_1);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_ACCEL_SWITCH_2);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_DECEL_SWITCH_1);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_DECEL_SWITCH_2);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_HEADWAY_DEC_SWITCH);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_HEADWAY_INC_SWITCH);
    SWITCH_INIT_SWITCH(
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS.VLC_HEADWAY_SWITCH);

    VLC_DRIVER_INF.VLC_DRIVER_INF.AVLC_DRIVE_OFF_POSSIBLE = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_AVLC_ALERT = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_DM_ALERT = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.BIT_FCA_ALERT = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.DRIVER_CONFIRMATION_NEEDED = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.HEADWAY_SETTING = Acc_default_headway_setting;
    VLC_DRIVER_INF.VLC_DRIVER_INF.OBJECT_DETECTED = FALSE;
    VLC_DRIVER_INF.VLC_DRIVER_INF.OPERATIONAL_MODE = Display_op_none;
    VLC_DRIVER_INF.VLC_DRIVER_INF.REPORTED_ERROR = Cc_no_error;

    CART_INIT_DAS_INPUT_DATA(&DAS_INPUT_DATA_LAST_CYCLE);

    /* init stop state for S&G logic */
    cc_das_stop_go_state = Cc_moving;

    /*! Initialisation of customer specific variables */
    HeadwayThumbSettingLC = Acc_default_headway_setting;
    HeadwaySetting = Acc_default_headway_setting;

    /*! Variables of VLC_LONG_VEH_WRAPPER_OUTPUT */
    drive_off_cycle_counter = 0u;
    stop_start_cycle_counter = (times_t)0;
    b_stop_start_flag = TRUE; /* engine stop allowed */
    b_drive_off_confirm_last = FALSE;

    drive_off_min_accel_rq_timer = (times_t)0;

    /*! Variables of VLC_DETERMINE_DRIVER_OPERATIONS */

    set_speed_last_cycle = (setspeed_t)0;
    last_effective_object_id = OBJ_INDEX_NO_OBJECT;

    /*! Variables of VLC_INFORM_DRIVER */
    uiOperationalTimer = 0u;

    uiReTriggerSetTimer =
        Cc_driver_operation_display_time; // Re trigger signal shall be set
                                          // every 5 seconds
    uiReTriggerRequestTimer =
        0u; // Re trigger signal shall be requested 600 ms
    uiReTriggerRepeatTimer =
        0u; // Re trigger signal shall be repeated again earliest after 600 ms
    LDC_DTR_MsgDisp_Rq_RDU_Timer = 0u;

    uiDisplayDelayTimer = Cc_display_delay_time;
    uiHeadwayDisplayTimer = 0u;

    last_object_id = OBJ_INDEX_NO_OBJECT;
    bDriverInterventedOff = FALSE;
    bHeadwayChangeDetected = FALSE;

    uiInitHeadwaySettingTimer = Cc_init_headway_setting_time;

    VLC_INHIBIT_INIT(&INHIBIT_BUFFER);
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_VEH_MTS_CALLBACK */
/*!

                             @brief
                           Callback
                           for
                           MEAS-freezes
                           of
                           static
                           buffers

                             @description
                           -

                             @return
                           static
                           void

                             @param[in]
                           void

                             @glob_in
                           None
                             @glob_out
                           None

                             @c_switch_part
                           None
                             @c_switch_full
                           VLC_CFG_LONG_PROCESSING
                           :
                           Configuration
                           switch
                           for
                           enabling
                           VLC_LONG
                           processing

                             @pre
                           None
                             @post
                           None

                             @created
                           -
                             @changed
                           -

                           *************************************************************************************************************************/
static void VLC_LONG_VEH_MTS_CALLBACK(void) {}

/*************************************************************************************************************************
  Functionname:    VLCConvertGearInfo */
static boolean VLCConvertGearInfo(TransmissionGear_t Input,
                                  pt_gear_t *pOutput)
{
    boolean retval = TRUE;
    switch (Input)
    {
    case DYN_GEAR_NEUTRAL_GEAR:
        *pOutput = Pt_gear_neutral;
        break;
    case DYN_GEAR_FIRST_GEAR:
        *pOutput = Pt_gear_first;
        break;
    case DYN_GEAR_SECOND_GEAR:
        *pOutput = Pt_gear_second;
        break;
    case DYN_GEAR_THIRD_GEAR:
        *pOutput = Pt_gear_third;
        break;
    case DYN_GEAR_FOURTH_GEAR:
        *pOutput = Pt_gear_fourth;
        break;
    case DYN_GEAR_FIFTH_GEAR:
        *pOutput = Pt_gear_fifth;
        break;
    case DYN_GEAR_SIXTH_GEAR:
        *pOutput = Pt_gear_sixth;
        break;
    case DYN_GEAR_SEVENTH_GEAR:
        *pOutput = Pt_gear_seventh;
        break;
    case DYN_GEAR_EIGHTH_GEAR:
        *pOutput = Pt_gear_eighth;
        break;
    case DYN_GEAR_REVERSE_GEAR:
        *pOutput = Pt_gear_reverse;
        break;
    case DYN_GEAR_PARK_GEAR:
        *pOutput = Pt_gear_park;
        break;
    default:
        *pOutput = Pt_gear_neutral;
        retval = FALSE;
        break;
    }
    return retval;
}

static void VLC_HEADWAY_INC_DEC_DECIDE(cc_driver_inputs_t *driver_inputs,
                                       cc_status_t *cc_status)
{
    const boolean headway_set_cond =
        ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) ||
         (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_override));

    if (SWITCH_RISING_EDGE(&driver_inputs->VLC_HEADWAY_DEC_SWITCH,
                           headway_set_cond) == SWITCH_STATE_ON)
    {
        if (HeadwaySetting >= HEADWAY_SETTING_LEVEL4)
        {
            HeadwaySetting = HEADWAY_SETTING_LEVEL3;
        }
        else
        {
            if (HeadwaySetting >= HEADWAY_SETTING_LEVEL3)
            {
                HeadwaySetting = HEADWAY_SETTING_LEVEL2;
            }
            else
            {
                if (HeadwaySetting >= HEADWAY_SETTING_LEVEL2)
                {
                    HeadwaySetting = HEADWAY_SETTING_LEVEL1;
                }
                else
                {
                    HeadwaySetting = HEADWAY_SETTING_LEVEL0;
                }
            }
        }
    }

    if (SWITCH_RISING_EDGE(&driver_inputs->VLC_HEADWAY_INC_SWITCH,
                           headway_set_cond) == SWITCH_STATE_ON)
    {
        if (HeadwaySetting <= HEADWAY_SETTING_LEVEL0)
        {
            HeadwaySetting = HEADWAY_SETTING_LEVEL1;
        }
        else
        {
            if (HeadwaySetting <= HEADWAY_SETTING_LEVEL1)
            {
                HeadwaySetting = HEADWAY_SETTING_LEVEL2;
            }
            else
            {
                if (HeadwaySetting <= HEADWAY_SETTING_LEVEL2)
                {
                    HeadwaySetting = HEADWAY_SETTING_LEVEL3;
                }
                else
                {
                    HeadwaySetting = HEADWAY_SETTING_LEVEL4;
                }
            }
        }
    }
}

static void VLC_HEADWAY_CYCLE_DECIDE(cc_driver_inputs_t *driver_inputs,
                                     cc_status_t *cc_status)
{
    const boolean headway_set_cond =
        ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) ||
         (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_override));

    if (SWITCH_RISING_EDGE(&driver_inputs->VLC_HEADWAY_SWITCH,
                           headway_set_cond) == SWITCH_STATE_ON)
    {
        if (HeadwaySetting <= CYCLE_HEADWAY_SETTING_LEVEL0)
        {
            HeadwaySetting = CYCLE_HEADWAY_SETTING_LEVEL1;
        }
        else
        {
            if (HeadwaySetting <= CYCLE_HEADWAY_SETTING_LEVEL1)
            {
                HeadwaySetting = CYCLE_HEADWAY_SETTING_LEVEL2;
            }
            else
            {
                if (HeadwaySetting <= CYCLE_HEADWAY_SETTING_LEVEL2)
                {
                    HeadwaySetting = CYCLE_HEADWAY_SETTING_LEVEL3;
                }
                else
                {
                    if (HeadwaySetting <= CYCLE_HEADWAY_SETTING_LEVEL3)
                    {
                        HeadwaySetting = CYCLE_HEADWAY_SETTING_LEVEL4;
                    }
                    else
                    {
                        HeadwaySetting = CYCLE_HEADWAY_SETTING_LEVEL0;
                    }
                }
            }
        }
    }
}

static void VLC_CONTROL_STATE_DECIDE(const VLC_AccLeverInput_t *pAccLever,
                                     cc_driver_inputs_t *driver_inputs,
                                     cc_status_t *cc_status,
                                     cc_driver_controls_t *driver_controls)
{
    static boolean AVLC_Main_Switch_state = TRUE;
    /*!!!has to be handled in hmicust!!!*/
    /*Checks if the ACC switch is ON,SW18*/
    // pAccLever->MainSwitch
    // if (SWITCH_RISING_EDGE(&driver_inputs->VLC_MAIN_SWITCH,
    //                        pAccLever->MainSwitch) == SWITCH_STATE_ON) {
    //     AVLC_Main_Switch_state = TRUE;
    // } else if (SWITCH_RISING_EDGE(&driver_inputs->VLC_CANCEL_SWITCH,
    //                               pAccLever->Cancel) == SWITCH_STATE_ON) {
    //     AVLC_Main_Switch_state = FALSE;
    // }

    if (AVLC_Main_Switch_state == TRUE)
    {
        driver_controls->SELECTED_FUNCTION = Cc_function_acc;
        driver_controls->SELECTED_FUNCTION_LAST_CYCLE = Cc_function_acc;
    }
    else
    {
        driver_controls->SELECTED_FUNCTION =
            Cc_function_none; /* Set to off, when main button is switched off */
    }
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_VEH_WRAPPER_INPUT */
static void VLC_LONG_VEH_WRAPPER_INPUT(
    const times_t cycle_time,
    const VED_VehDyn_t *pVehDyn,
    const VLC_AccLeverInput_t *pAccLever,
    const VLC_LongCtrlInput_t *pLongCtrlResp,
    cc_driver_information_t *driver_information,
    cart_das_input_data_t *das_input_data,
    cc_driver_inputs_t *driver_inputs,
    cc_status_t *cc_status,
    VLC_DFV2SenInfo_t *pDFVLongOut,
    cc_driver_controls_t *driver_controls,
    cc_input_data_t *cc_input,
    cc_error_data_t *error_data)
{
    float32 LatAcc;

    /*wrapper for VLC_DRIVER_CONTROLS*/

    if ((pAccLever !=
         NULL)) //&& (pAccLever->sSigHeader.eSigStatus == AL_SIG_STATE_OK))
    {
        SWITCH_SET_STATE(
            &(driver_inputs->VLC_ACCEL_SWITCH_1),
            (boolean)(pAccLever->ResumeAccelSwitch == AVLC_LEVER_ACCEL_LOW));

        SWITCH_SET_STATE(
            &(driver_inputs->VLC_DECEL_SWITCH_1),
            (boolean)(pAccLever->DecelSwitch == AVLC_LEVER_DECEL_LOW));

        SWITCH_SET_STATE(
            &(driver_inputs->VLC_ACCEL_SWITCH_2),
            (boolean)(pAccLever->ResumeAccelSwitch == AVLC_LEVER_ACCEL_HIGH));

        SWITCH_SET_STATE(
            &(driver_inputs->VLC_DECEL_SWITCH_2),
            (boolean)(pAccLever->DecelSwitch == AVLC_LEVER_DECEL_HIGH));

        SWITCH_SET_STATE(
            &(driver_inputs->VLC_RESUME_SWITCH),
            (boolean)(pAccLever->ResumeAccelSwitch == AVLC_LEVER_ACCEL_HIGH));

        SWITCH_SET_STATE(&(driver_inputs->VLC_MAIN_SWITCH),
                         pAccLever->MainSwitch);

        SWITCH_SET_STATE(&(driver_inputs->AVLC_MODE_SWITCH),
                         pAccLever->ACCMode);

        SWITCH_SET_STATE(&(driver_inputs->VLC_CANCEL_SWITCH),
                         pAccLever->Cancel);

        SWITCH_SET_STATE(&(driver_inputs->VLC_HEADWAY_INC_SWITCH),
                         pAccLever->HeadwayInc);
        SWITCH_SET_STATE(&(driver_inputs->VLC_HEADWAY_DEC_SWITCH),
                         pAccLever->HeadwayDec);
        SWITCH_SET_STATE(&(driver_inputs->VLC_HEADWAY_SWITCH),
                         pAccLever->HeadwaySwitch);
        SWITCH_SET_STATE(&(driver_inputs->VLC_SET_SWITCH),
                         pAccLever->SetSwitch);

        VLC_HEADWAY_INC_DEC_DECIDE(driver_inputs, cc_status);
        VLC_HEADWAY_CYCLE_DECIDE(driver_inputs, cc_status);

        /* Fill in output structure to VLC_SEN part */
        pDFVLongOut->HeadwaySetting = HeadwaySetting;

        driver_information->HEADWAY_SETTING = HeadwaySetting;
    }
    else
    {
        /* ACC lever pointer null or signal status invalid : do not evaluate
         * it's switches */
    }

    VLC_CONTROL_STATE_DECIDE(pAccLever, driver_inputs, cc_status,
                             driver_controls);
    /*VLC_ERROR*/
    /*gVLC_ERROR_DATA.VLC_ERROR_DATA.VLC_INHIBIT !!!not defined yet*/

    /*wrapper for VLC_INPUT_DATA*/
    cc_input->DATA_VALIDITY.LATERAL_ACCEL = TRUE;
    cc_input->DATA_VALIDITY.SPEEDO_SPEED = TRUE;
    cc_input->VEHICLE_STATUS.SPORTS_MODE = FALSE;
    cc_input->PERMANENT_LIMITER_SETSPEED = 255u;
    cc_input->COUNTRY_CODE = Cc_rest_of_world;
    cc_input->VEHICLE_SPEED_LIMIT = 255u;

    /* ACCELERATION_GRADIENT needed to switch from enable control state to
     * active */
    cc_input->ACCELERATION_GRADIENT =
        ((gradient_t)cc_input->ACCELERATION_GRADIENT -
         (gradient_t)
             pLongCtrlResp->KinCtrlDynInput.longi_initialization_accel) /
        cycle_time;

    cc_input->DATA_VALIDITY.ACTUAL_GEAR = TRUE;

    cc_input->ACTUAL_GEAR = Pt_gear_fifth;

    /*! Input pLongCtrlResp->DisplayOutput.speedometer_speed Factor 100 uint16
     * Velocity!*/
    cc_input->SPEEDOMETER_VEHICLE_SPEED =
        ((speedometer_speed_t)
             pLongCtrlResp->DisplayOutput.speedometer_speed); /* factor 100 */

    if (pLongCtrlResp->DisplayOutput.speed_unit == SPD_UNIT_KMH)
    {
        cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH = TRUE;
        /* the speedometer speed will be sent by DAI allways in kmh
        cc_input->SPEEDOMETER_VEHICLE_SPEED =
        cc_input->SPEEDOMETER_VEHICLE_SPEED; */
    }
    else
    {
        sint32 Help;

        cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH = FALSE;
        /* calculation from kph to mph */
        Help = (sint32)cc_input->SPEEDOMETER_VEHICLE_SPEED;
        Help *= (sint32)Speed_conv_factor_mph;
        Help += (sint32)Speed_conv_factor_kmh / (sint32)2;
        Help /= (sint32)Speed_conv_factor_kmh;
        cc_input->SPEEDOMETER_VEHICLE_SPEED = (speedometer_speed_t)MAT_LIM(
            Help, Speedo_speed_min, Speedo_speed_max);
    }

    /*wrapper for DAS_INPUT_DATA*/
    das_input_data->BRAKE_STAT.BRAKE_FAILED = FALSE;
    das_input_data->DATA_VALID.INIT_ACCEL = TRUE;
    das_input_data->DATA_VALID.VEHICLE_ACCEL = TRUE;
    das_input_data->DATA_VALID.VEHICLE_SPEED = TRUE;
    das_input_data->PT_STAT.FAIL_IRREVERSABLE = FALSE;
    das_input_data->PT_STAT.FAIL_REVERSABLE = FALSE;
    das_input_data->PT_STAT.SHIFT_IN_PROGRESS = FALSE;
    das_input_data->PT_STAT.KICKDOWN = FALSE;
    das_input_data->PT_STAT.CLUTCH_OPEN = FALSE;

    das_input_data->CHASSIS_STAT.PB_ACT =
        pLongCtrlResp->KinCtrlDynInput.park_brk_eng;

    /*drive mode switch*/
    das_input_data->LODM_STAT.DAS_MODE = pAccLever->ACCMode;

    if (pVehDyn != NULL)
    {
        /* Delay of vehicle stand still. Was previously in VLC_SEN task, with
        setting of acc_input_data_ptr->LODM_STAT.STANDSTILL */
        if (pVehDyn->MotionState.MotState == VED_LONG_MOT_STATE_STDST)
        {
            // if (cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME >=
            //     Cc_standstill_delay_time) {
            //     /* Old code set acc_input_data_ptr->LODM_STAT.STANDSTILL  =
            //     TRUE; in VLC_SEN, here we don't need to do anything, since
            //     the StandStill_Time counter is checked directly in the later
            //     code */ standstill_delay_cycle_counter++; /*SW18, counter
            //     increases by 1
            //                                          every Veh cycle(20ms)*/
            // } else {
            //     /* Old code set acc_input_data_ptr->LODM_STAT.STANDSTILL  =
            //     FALSE; in VLC_SEN Here we only need to increment our
            //     stanstill time */
            //     cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME +=
            //     cycle_time; standstill_delay_cycle_counter++; /*SW18, counter
            //     increases by 1
            //                                          every Veh cycle(20ms)*/
            // }

            if (cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME <
                Cc_standstill_delay_time)
            {
                cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME += cycle_time;
            }

            if (standstill_delay_cycle_counter <
                VLC_T_STANDSTILL_WAIT_ELAPSED / VLC_VEH_CYCLE_TIME)
            {
                standstill_delay_cycle_counter++;
            }

            cc_status->VLC_DRIVER_REQUESTS.MOVING_TIME = 0;
        }
        else
        {
            /* Old code set acc_input_data_ptr->LODM_STAT.STANDSTILL  = FALSE;
            in VLC, here we only need to reset standstill time (since we are
            moving) */
            cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME = 0u;
            cc_status->VLC_DRIVER_REQUESTS.DRIVE_OFF_TIME = 0;
            standstill_delay_cycle_counter = 0;

            // start moving
            if (cc_status->VLC_DRIVER_REQUESTS.MOVING_TIME < Cc_moving_time)
            {
                cc_status->VLC_DRIVER_REQUESTS.MOVING_TIME += cycle_time;
            }
        }

        LatAcc = pVehDyn->Lateral.Accel.LatAccel;
    }
    else
    {
        LatAcc = 0.f;
    }

    /*wrapper for VLC_DRIVER_CONTROLS*/

    /*!!!has to be handled in hmicust!!!*/
    // driver_controls->SELECTED_FUNCTION            = Cc_function_cc; /*!!!
    // todo: activate main mode and handle in hmicust*/
    // driver_controls->SELECTED_FUNCTION_LAST_CYCLE = Cc_function_cc; /*!!!
    // todo: activate main mode and handle in hmicust*/

    /*wrapper for VLC_INPUT_DATA*/
    cc_input->DATA_VALIDITY.LATERAL_ACCEL = TRUE;
    cc_input->DATA_VALIDITY.SPEEDO_SPEED = TRUE;
    cc_input->VEHICLE_STATUS.SPORTS_MODE = FALSE;
    cc_input->PERMANENT_LIMITER_SETSPEED = MAX_SETSPEED;
    cc_input->COUNTRY_CODE = Cc_rest_of_world;
    cc_input->VEHICLE_SPEED_LIMIT = MAX_SETSPEED;

    /* ACCELERATION_GRADIENT needed to switch from enable control state to
     * active */
    cc_input->ACCELERATION_GRADIENT =
        (cc_input->ACCELERATION_GRADIENT -
         (gradient_t)
             pLongCtrlResp->KinCtrlDynInput.longi_initialization_accel) /
        cycle_time;

    cc_input->LATERAL_ACCELERATION =
        (acceleration_t)(LatAcc * (float32)Acceleration_s);

    cc_input->DATA_VALIDITY.ACTUAL_GEAR = TRUE;

    cc_input->SPEEDOMETER_VEHICLE_SPEED =
        ((speedometer_speed_t)
             pLongCtrlResp->DisplayOutput.speedometer_speed); /* factor 100 */

    if (pLongCtrlResp->DisplayOutput.speed_unit == SPD_UNIT_KMH)
    {
        cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH = TRUE;
        /* the speedometer speed will be sent by DAI allways in kmh
        cc_input->SPEEDOMETER_VEHICLE_SPEED =
        cc_input->SPEEDOMETER_VEHICLE_SPEED; */
    }
    else
    {
        sint32 Help;

        cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH = FALSE;
        /* calculation from kph to mph */
        Help = (sint32)cc_input->SPEEDOMETER_VEHICLE_SPEED;
        Help *= (sint32)Speed_conv_factor_mph;
        Help += (sint32)Speed_conv_factor_kmh / (sint32)DIVISIOR_2;
        Help /= (sint32)Speed_conv_factor_kmh;
        cc_input->SPEEDOMETER_VEHICLE_SPEED = (speedometer_speed_t)MAT_LIM(
            Help, Speedo_speed_min, Speedo_speed_max);
    }

    /*wrapper for DAS_INPUT_DATA*/
    das_input_data->BRAKE_STAT.BRAKE_FAILED = FALSE;
    das_input_data->DATA_VALID.INIT_ACCEL = TRUE;
    das_input_data->DATA_VALID.VEHICLE_ACCEL = TRUE;
    das_input_data->DATA_VALID.VEHICLE_SPEED = TRUE;

    das_input_data->CHASSIS_STAT.PB_ACT =
        pLongCtrlResp->KinCtrlDynInput.park_brk_eng;

    das_input_data->A_INIT = (acceleration_t)MAT_LIM(
        ROUND_TO_INT(pVehDyn->Longitudinal.MotVar.Accel *
                     (float32)Acceleration_s),
        Accel_min, Accel_max);

    das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL =
        (pLongCtrlResp->KinCtrlDynInput.driver_override_decel_pedal);

    /* if driver braking, automatic drive off not possible DTRplus_Ueb_Zert_525
     */
    if ((cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME <
         (times_t)Cc_standstill_delay_time) &&
        (das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL == TRUE))
    {
        cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME =
            Cc_standstill_delay_time;
    }
    else
    {
        /* nothing to do*/
    }

    /* Verify that DC status information sufficient replacement for old ACC
     * Enable information set via VDATASetKinACCEnable */
    // das_input_data->LODM_STAT.DAS_ENABLE =
    //     (pLongCtrlResp->KinCtrlDynInput.acc_enable);
    das_input_data->LODM_STAT.DAS_ENABLE =
        TRUE;
    das_input_data->LODM_STAT.DAS_INHIBIT =
        (pLongCtrlResp->KinCtrlDynInput.acc_inhibit);
    das_input_data->LODM_STAT.DAS_RESET =
        (pLongCtrlResp->KinCtrlDynInput.acc_reset);
    das_input_data->LODM_STAT.DAS_SHUTOFF_ACQ =
        (pLongCtrlResp->KinCtrlDynInput.longi_shutoff_acknowledged);
    das_input_data->LODM_STAT.DC_LIM_ACCEL =
        (pLongCtrlResp->KinCtrlDynInput.DAS_accel_request_limited);
    das_input_data->LODM_STAT.DC_LIM_DECEL =
        (pLongCtrlResp->KinCtrlDynInput.DAS_decel_request_limited);
    das_input_data->LODM_STAT.OVERRIDE_ACCEL =
        (pLongCtrlResp->KinCtrlDynInput.driver_override_accel_pedal);
    das_input_data->LODM_STAT.OVERRIDE_DECEL =
        (pLongCtrlResp->KinCtrlDynInput.driver_override_decel_pedal);

    /* external error state from SW */
    switch (pLongCtrlResp->KinCtrlDynInput.DC_status_information)
    {
    case DC_STATUS_AVAILABLE:
        error_data->REPORTED_ERROR = Cc_no_error;
        break;
    case DC_STATUS_TMP_NOT_AVAIL:
        error_data->REPORTED_ERROR = Cc_temp_unavailable;
        break;
    case DC_STATUS_NOT_AVAIL:
        error_data->REPORTED_ERROR = Cc_error_service;
        break;
    default:
        error_data->REPORTED_ERROR = Cc_temp_unavailable;
        break;
    }

    if (pVehDyn->MotionState.Confidence >= MOTION_STATE_CONFIDENCE)
    {
        das_input_data->LODM_STAT.STANDSTILL =
            (boolean)(pVehDyn->MotionState.MotState ==
                      VED_LONG_MOT_STATE_STDST);
    }

    das_input_data->VEHICLE_ACCEL = (acceleration_t)MAT_LIM(
        ROUND_TO_INT(pVehDyn->Longitudinal.MotVar.Accel *
                     (float32)Acceleration_s),
        Accel_min, Accel_max);
    das_input_data->VEHICLE_SPEED = (velocity_t)MAT_LIM(
        ROUND_TO_INT(pVehDyn->Longitudinal.VeloCorr.corrVelo *
                     (float32)Velocity_s),
        Velocity_min, Velocity_max);
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_VEH_TO_SEN_INFO */
/*!

                               @brief
                             Fill
                             DFV2SenInfo
                             Port

                               @description
                             -

                               @return
                             static
                             void


                               @param[in]
                             pLongCtrlResp
                             :
                             the
                             longitudinal
                             controller
                             response
                             \n
                                                   pLongCtrlResp->KinCtrlDynInput.driver_override_accel_pedal : Driver override by gas pedal                   [TRUE, FALSE] \n
                                                   pLongCtrlResp->KinCtrlDynInput.longi_initialization_accel : Vehicle initialization acceleration             [full range of signed short]
                                                                       pLongCtrlResp->Custom.EngineEcuInput.Ext_HeadwaySetting: Headway setting from external					  [Ext_HeadwaySetting_t as per Rte_Type.h]
                               @param[in]
                             cc_status
                             :
                             the
                             cruise
                             control
                             status
                             information
                             \n
                                                   cc_status->VLC_CONTROL_DATA.MAXIMUM_ACCELERATION_LIMIT :                                                     [acceleration_t as per Rte_Type.h] \n
                                                   cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT :                                                     [acceleration_t as per Rte_Type.h] \n
                                                   cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE : deceleration limitation after override active        [full range of unsigned char] \n
                                                   cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.CONTROL_TO_RELEVANT_OBJECT :                            [TRUE, FALSE] \n
                                                   cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE : target object is effective for control     [TRUE, FALSE]
                               @param[in,out]
                             pDFVLongOut
                             :
                             used
                             for
                             passing
                             information
                             from
                             VLC_VEH
                             to
                             VLC_SEN
                             part
                             of
                             longitudinal
                             controller
                             \n
                                                   pDFVLongOut->OverrideAccel : Driver override by gas pedal                                                [TRUE, FALSE] \n
                                                   pDFVLongOut->CurLongCtrlAccel :                                                                          [acceleration_t as per Rte_Type.h] \n
                                                   pDFVLongOut->MaxAccelLimit :                                                                             [acceleration_t as per Rte_Type.h] \n
                                                   pDFVLongOut->MinAccelLimit :                                                                             [acceleration_t as per Rte_Type.h] \n
                                                   pDFVLongOut->ProbLaneChgLeft : probability of lane change to left lane                                   [percentage_t as per Rte_Type.h] \n
                                                   pDFVLongOut->ProbLaneChgRight : probability of lane change to right lane                                 [percentage_t as per Rte_Type.h] \n
                                                   pDFVLongOut->AccOn : ACC is in active state                                                              [TRUE, FALSE] \n
                                                   pDFVLongOut->AccNotOff: ACC is active, engaged or override												  [TRUE, FALSE] \n
                                                                       pDFVLongOut->DecelLimOverride : deceleration limitation after override active                            [TRUE, FALSE] \n
                                                   pDFVLongOut->CtrlToRelevObj :                                                                            [TRUE, FALSE] \n
                                                   pDFVLongOut->ObjectEffective : target object is effective for control                                    [TRUE, FALSE] \n
                                                                       pDFVLongOut->HeadwaySetting: Headway setting dependent on external setting								  [percentage_t as per Rte_Type.h]

                               @glob_in
                             gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE:
                             state
                             of HMI statemachine								  [0 ... 11]
                               @glob_out
                             None

                               @c_switch_part
                             None

                               @c_switch_full
                             VLC_CFG_LONG_PROCESSING
                             :
                             Configuration
                             switch
                             for
                             enabling
                             VLC_LONG
                             processing

                               @pre
                             None
                               @post
                             None

                               @created
                             -
                               @changed
                             -


                             *************************************************************************************************************************/
static void VLC_LONG_VEH_TO_SEN_INFO(const VLC_LongCtrlInput_t *pLongCtrlResp,
                                     const cc_status_t *cc_status,
                                     VLC_DFV2SenInfo_t *pDFVLongOut)
{
    /* Fill standstill flag. Note: timing and behaviour is the same as in old
    projects, but clean interface used instead of global variables */
    pDFVLongOut->StandStill =
        (boolean)(cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME >=
                  Cc_standstill_delay_time);
    pDFVLongOut->OverrideAccel =
        pLongCtrlResp->KinCtrlDynInput.driver_override_accel_pedal;
    pDFVLongOut->CurLongCtrlAccel =
        pLongCtrlResp->KinCtrlDynInput.longi_initialization_accel;
    pDFVLongOut->MaxAccelLimit =
        cc_status->VLC_CONTROL_DATA.MAXIMUM_ACCELERATION_LIMIT;
    pDFVLongOut->MinAccelLimit =
        cc_status->VLC_CONTROL_DATA.MINIMUM_ACCELERATION_LIMIT;

    if (VLC_pDIMCustDataOut->iLaneChangeProbability >= 0)
    {
        pDFVLongOut->ProbLaneChgLeft =
            (percentage_t)ABS(VLC_pDIMCustDataOut->iLaneChangeProbability);
    }
    else
    {
        pDFVLongOut->ProbLaneChgRight =
            (percentage_t)ABS(VLC_pDIMCustDataOut->iLaneChangeProbability);
    }

    pDFVLongOut->AccOn =
        (boolean)((gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                   Cc_cc_active) ||
                  (gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                   Cc_cc_engage));

    pDFVLongOut->AccNotOff =
        (boolean)((gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                   Cc_cc_active) ||
                  (gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                   Cc_cc_engage) ||
                  (gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                   Cc_cc_override));

    pDFVLongOut->DecelLimOverride =
        cc_status->VLC_CONTROL_DATA.DECEL_LIM_OVERRIDE_ACTIVE;
    pDFVLongOut->CtrlToRelevObj =
        (boolean)cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
            .CONTROL_TO_RELEVANT_OBJECT;
    pDFVLongOut->ObjectEffective =
        (boolean)
            cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE;

    pDFVLongOut->MovingTime = cc_status->VLC_DRIVER_REQUESTS.MOVING_TIME;
}

static void VLC_LONG_DAS_STATUS_OUTPUT(
    const cart_das_output_data_t *das_output,
    const cc_status_t *cc_status,
    const cc_error_data_t *error_data,
    const cart_das_input_data_t *das_input_data,
    VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    if (das_input_data->LODM_STAT.DAS_ENABLE == FALSE)
    {
        pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_OFF;
    }
    else if (error_data->REPORTED_ERROR == Cc_temp_unavailable ||
             error_data->REPORTED_ERROR == Cc_error_service)
    {
        pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_FAULT;
    }
    else if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_off)
    {
        pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_PASSIVE;
    }
    else if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_ready ||
             cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                 Cc_cc_disengage ||
             cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_engage)
    {
        pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_STANDBY;
    }
    else if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active)
    {
        if ((boolean)das_output->DAS_STAT.DAS_STAND_STILL == TRUE ||
            (das_input_data->VEHICLE_SPEED <= Cc_static_veL_thres &&
             das_output->MIN_REQ_ACCEL <= Cc_static_dec_thres))
        {
            if (cc_status->VLC_DRIVER_REQUESTS.DRIVE_OFF_TIME <
                Cc_drive_off_smooth_time)
            {
                pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_STAND_ACTIVE;
            }
            else
            {
                pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_STAND_WAIT;
            }
        }
        else
        {
            pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_ACTIVE;
        }
    }
    else if (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_override)
    {
        pLongCtrlCmd->KinOutput.DAS_status = DAS_STATUS_OVERRIDE;
    }
}

static void VLC_LONG_HEADWAY_OUTPUT(
    const cc_driver_information_t *driver_information,
    VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    if (driver_information->HEADWAY_SETTING <= HEADWAY_SETTING_LEVEL0)
    {
        pLongCtrlCmd->KinFctInfo.headway_setting = HEADWAY_SETTING_OUTPUT_VAL0;
    }
    else if (driver_information->HEADWAY_SETTING <= HEADWAY_SETTING_LEVEL1)
    {
        pLongCtrlCmd->KinFctInfo.headway_setting = HEADWAY_SETTING_OUTPUT_VAL1;
    }
    else if (driver_information->HEADWAY_SETTING <= HEADWAY_SETTING_LEVEL2)
    {
        pLongCtrlCmd->KinFctInfo.headway_setting = HEADWAY_SETTING_OUTPUT_VAL2;
    }
    else if (driver_information->HEADWAY_SETTING <= HEADWAY_SETTING_LEVEL3)
    {
        pLongCtrlCmd->KinFctInfo.headway_setting = HEADWAY_SETTING_OUTPUT_VAL3;
    }
    else
    {
        pLongCtrlCmd->KinFctInfo.headway_setting = HEADWAY_SETTING_OUTPUT_VAL4;
    }
}

static void VLC_LONG_ACCEL_OUTPUT(const cart_das_output_data_t *das_output,
                                  const cc_status_t *cc_status,
                                  VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    pLongCtrlCmd->KinOutput.MinRequestedLongAcceleration =
        das_output->MIN_REQ_ACCEL;
}

/* **********************************************************************
  Functionname             VLC_LONG_VEH_WRAPPER_OUTPUT */
static void VLC_LONG_VEH_WRAPPER_OUTPUT(
    const times_t cycle_time,
    const reqVLCVehDebugList_t *pVLCVehDebugPorts,
    const cc_input_data_t *pCcInputData,
    const velocity_t vehicle_speed,
    const cart_das_output_data_t *das_output,
    const cc_status_t *cc_status,
    const VLC_acc_output_data_t *pAccOutput,
    const cc_driver_information_t *driver_information,
    const cc_error_data_t *error_data,
    const cart_das_input_data_t *das_input_data,
    VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    acceleration_t hysteresis;
    float32 help;
    gradient_t max_neg_grad;
    gradient_t max_pos_grad;

    /*base algo acceleration band interface - MIN-/MAX_REQ_ACCEL is
     * gradient-limited and filtered*/
    VLC_LONG_ACCEL_OUTPUT(das_output, cc_status, pLongCtrlCmd);

    /*customer specific acceleration interface - consists of: raw acceleration,
      gradients on separate interfaces and a
      acceleration hysteresis to prevent unnecessary activation of brakes in
      comfort situations*/

    /* the raw acceleration request is determined in
     * VLC_LONG_VEH_EXT_VLC_ARBITRATION */
    VLC_LONG_VEH_EXT_VLC_ARBITRATION(cycle_time, vehicle_speed, pCcInputData,
                                     cc_status, das_output, driver_information,
                                     pLongCtrlCmd);

    /* gradients on separate interfaces for positive and negative gradient*/
    /* factor to give customer possibility of more jerk than requested*/

    help = (float32)MAT_CALCULATE_PARAM_VALUE1D(
               Cc_gain_grad, Cc_gain_grad_points, vehicle_speed) /
           (float32)Scale_100;

    pLongCtrlCmd->Custom.CustomOutput.RequestedLongNegAccelGrad =
        (signed short)(help * cc_status->VLC_ACCEL_CONTROL_DATA
                                  .ACCEL_GRADIENT_LIMITS.MAX_NEG_GRAD);
    pLongCtrlCmd->Custom.CustomOutput.RequestedLongPosAccelGrad =
        (signed short)(help * cc_status->VLC_ACCEL_CONTROL_DATA
                                  .ACCEL_GRADIENT_LIMITS.MAX_POS_GRAD);

    /* keeping jerk limits according to limits in req */

    max_neg_grad = MAT_CALCULATE_PARAM_VALUE1D(
        max_decel_gradient_critical, Acc_neg_grad_points, vehicle_speed);
    max_pos_grad = Cc_max_release_brake_grad;

    if (pLongCtrlCmd->Custom.CustomOutput.RequestedLongNegAccelGrad <
        max_neg_grad)
    {
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongNegAccelGrad =
            (signed short)max_neg_grad;
    }
    else
    {
    }
    if (pLongCtrlCmd->Custom.CustomOutput.RequestedLongPosAccelGrad >
        max_pos_grad)
    {
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongPosAccelGrad =
            (signed short)max_pos_grad;
    }
    else
    {
    }

    /* acceleration hysteresis*/
    if (pAccOutput->SITUATION_CLASS.CRITICALITY <
        MAX_CRITICALITY_DYNAMIC_MODE)
    {
        /* use the engine drag as long as possible if situation is not
         * critical*/
        hysteresis = COMFORT_HYSTERESIS;
    }
    else
    {
        /* use a smaller or no hysteresis in dynamic/critcal situation in order
           to apply the brakes as quickly as possible */
        hysteresis = DYNAMIC_HYSTERESIS;
    }
    pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelHyst = hysteresis;

    /*ACCInternalState wrapping for output interface */ /*DAS_status wrapping
                                                           for output interface
                                                         */
    VLC_LONG_DAS_STATUS_OUTPUT(das_output, cc_status, error_data,
                               das_input_data, pLongCtrlCmd);

    /*KinOutput wrapping*/
    pLongCtrlCmd->KinOutput.brake_pre_fill =
        (boolean)das_output->DAS_STAT.DAS_PREFILL;
    pLongCtrlCmd->KinOutput.stand_still_request =
        (boolean)das_output->DAS_STAT.DAS_STAND_STILL;

    pLongCtrlCmd->KinOutput.DAS_failure_information = DAS_FAILURE_NONE;

    /*KinFctInfo wrapping*/
    pLongCtrlCmd->KinFctInfo.headway_control_alert =
        driver_information->BIT_AVLC_ALERT;
    pLongCtrlCmd->KinFctInfo.FCA_alert = driver_information->BIT_FCA_ALERT;
    pLongCtrlCmd->KinFctInfo.DM_alert_level = driver_information->BIT_DM_ALERT;
    pLongCtrlCmd->KinFctInfo.object_detected =
        driver_information->OBJECT_DETECTED;
    pLongCtrlCmd->KinFctInfo.requested_distance =
        (float32)pAccOutput->REQUESTED_DISTANCE / Distance_s;
    driver_information->REQUESTED_DISTANCE;
    pLongCtrlCmd->KinFctInfo.obj_interest_distance =
        driver_information->OBJECT_DISTANCE;
    pLongCtrlCmd->KinFctInfo.desired_speed = driver_information->SET_SPEED;
    pLongCtrlCmd->KinFctInfo.recommended_speed =
        driver_information->RECOMMENDED_SPEED;
    pLongCtrlCmd->KinFctInfo.speed_target = driver_information->OBJECT_SPEED;

    VLC_LONG_HEADWAY_OUTPUT(driver_information, pLongCtrlCmd);

    /*switch(driver_information->HEADWAY_SETTING)
    {
                  case HEADWAY_SETTING_LEVEL6:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL6;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL5:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL5;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL4:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL4;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL3:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL3;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL2:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL2;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL1:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL1;
                  }
                  break;
                  case HEADWAY_SETTING_LEVEL0:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL0;
                  }
                  break;
                  default:
                  {
                          pLongCtrlCmd->KinFctInfo.headway_setting =
    HEADWAY_SETTING_OUTPUT_VAL6;
                  }
                  break;
    }*/

    pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut =
        NO_DISPLAY; /*Default reset for every Veh cycle*/

    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.Door_Open == TRUE)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= DOOR_OPEN;
    }
    if (VLCVEH_pLongCtrlResp->KinCtrlDynInput.seatbelt_state !=
        SEATBELT_DRIVER_CLOSED)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= SEATBELT_UNPLUCKED;
    }
    if ((pCcInputData->ACTUAL_GEAR == Pt_gear_neutral) ||
        (pCcInputData->ACTUAL_GEAR == Pt_gear_park) ||
        (pCcInputData->ACTUAL_GEAR == Pt_gear_reverse))
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= NO_FORWARD_GEAR;
    }
    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.EPB_Active == TRUE)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= EPB_ACTIVATED;
    }
    if (cc_status->VLC_DRIVER_REQUESTS.VLC_SETSPEED > Acc_max_setspeed_kmh)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= SPEED_OVER_150KPH;
    }
    if (((pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON == (uint8)1U) ||
         (pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON == (uint8)3U) ||
         (pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON == (uint8)5U) ||
         (pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON ==
          (uint8)9U)) /*To check ACC INHIBITION BLOCKAGE*/
        || (pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON &
            (1 << AVLC_INHIBITION_PARTIAL_BLOCKAGE_BIT)))
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= RSM_BLINDNESS;
    }
    if ((boolean)cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
            .DRIVE_OFF_POSSIBLE == TRUE)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= DRIVE_OFF_REQUEST;
    }
    if ((pAccOutput->AVLC_OUTPUT_STATUS.INHIBITION_REASON &
         (1 << AVLC_INHIBITION_ALIGNMENT_BIT)))
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |=
            RSM_SENSOR_ALIGNMENT_INCOMPLETE;
    }
    if (standwait_over_time_limit == TRUE)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |=
            STANDWAIT_OVER_TIME_LIMIT;
    }
    if (driver_information->REPORTED_ERROR != Cc_no_error)
    {
        pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= RADAR_ERROR;
    }

    /* if(pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut == 0)
     {
             pLongCtrlCmd->Custom.CustomOutput.ACCDisplayOut |= NO_DISPLAY;
     }*/

    pLongCtrlCmd->KinFctInfo.DM_status = DM_STATUS_OFF;

    /*DriverData wrapping*/
    pLongCtrlCmd->DriverData.drive_off_inhibit =
        (boolean)das_output->DAS_STAT.DAS_DRIVE_OFF_INHIBIT;
    pLongCtrlCmd->DriverData.drive_off_confirm =
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM;
    pLongCtrlCmd->DriverData.drive_off_possible =
        ((boolean)cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS
             .DRIVE_OFF_POSSIBLE == TRUE);
    pLongCtrlCmd->DriverData.drive_off_request =
        driver_information->DRIVER_CONFIRMATION_NEEDED;
    pLongCtrlCmd->DriverData.failure_state = driver_information->REPORTED_ERROR;
    pLongCtrlCmd->DriverData.operational_mode =
        cc_status->VLC_DRIVER_REQUESTS.OPERATIONAL_MODE;

    {
        pLongCtrlCmd->Custom.CustomOutput.AVLC_Object_Effective =
            cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE;
        pLongCtrlCmd->Custom.CustomOutput.CruiseCtrlMode =
            cc_status->VLC_DRIVER_REQUESTS.CONTROL_MODE;
    }

    /*in case of internal inhibition, indicate that output signals might be
     * invalid*/
    if ((boolean)pCcInputData->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == TRUE)
    {
        pLongCtrlCmd->sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
    }
    else
    {
        pLongCtrlCmd->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
    }

    // debug
    memcpy(pVLCVehDebugPorts->pVlcACCCtrlData,
           &gVLC_STATUS.VLC_STATUS.VLC_ACCEL_CONTROL_DATA,
           sizeof(cc_acceleration_control_data_t));
    memcpy(pVLCVehDebugPorts->pVlcCCCtrlData,
           &gVLC_STATUS.VLC_STATUS.VLC_CONTROL_DATA, sizeof(cc_control_data_t));

    pVLCVehDebugPorts->pVlcVehSmDebugInfo->CONTROL_STATE = cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE;
    pVLCVehDebugPorts->pVlcVehSmDebugInfo->CONTROL_STATE_LAST_CYCLE = cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE_LAST_CYCLE;

    memcpy(&(pVLCVehDebugPorts->pVlcVehSmDebugInfo->DRIVER_OPERATIONS),
           &(cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS), sizeof(cc_driver_operations_t_debug));
    memcpy(&(pVLCVehDebugPorts->pVlcVehSmDebugInfo->DRIVER_OPERATIONS_LAST_CYCLE),
           &(cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS_LAST_CYCLE), sizeof(cc_driver_operations_t_debug));
    memcpy(&(pVLCVehDebugPorts->pVlcVehSmDebugInfo->ENGAGEMENT_CONDITIONS),
           &(cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS), sizeof(cc_engagement_conditions_t_debug));

}

static void VLC_DECEL_SWITCH_PRESSED(const cart_das_input_data_t *das_input,
                                     cc_driver_inputs_t *driver_inputs,
                                     cc_driver_requests_t *driver_requests,
                                     const uint16 cc_start_repeat_fun_cycles,
                                     const uint16 cc_repeat_fun_cycles)
{
    boolean temp_condition;
    if (SWITCH_FALLING_EDGE(&driver_inputs->VLC_DECEL_SWITCH_2,
                            (driver_requests->DRIVER_REQUEST_STATUS
                                 .SELECTED_FUNCTION_ACTIVE)) == TRUE)
    {
        if (driver_requests->CONTROL_STATE == Cc_cc_active &&
            das_input->LODM_STAT.STANDSTILL == FALSE)
        {
            driver_requests->DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED = TRUE;
            driver_requests->DRIVER_REQUEST_STATUS.TIMED_DECREMENT = TRUE;
            driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 =
                FALSE; /*SW18 for speed decrement by level 5*/
        }
    }
    else
    {
        temp_condition =
            (boolean)((driver_requests->DRIVER_REQUEST_STATUS
                           .SELECTED_FUNCTION_ACTIVE == TRUE) &&
                      (driver_inputs->VLC_DECEL_SWITCH_2.AKT_STATUS == FALSE));
        if (SWITCH_FALLING_EDGE(&driver_inputs->VLC_DECEL_SWITCH_1,
                                temp_condition) == SWITCH_STATE_ON)
        {
            // Decle2 switch muss inaktiv sein
            /* This code may lead to a not understandalbe
             * behaviour during override, comment by Thomas
             * Petzold*/
            if ((driver_requests->CONTROL_STATE == Cc_cc_override) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .ACTUAL_SPEED_TAKEN_OVERRIDE == FALSE) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .OVERRIDE_WHILE_ENGAGEMENT == FALSE) &&
                ((driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET /
                  Speedo_speed_s) > (sint16)(driver_requests->VLC_SETSPEED)))
            {
                // driver_requests->DRIVER_REQUEST_STATUS
                //     .ACTUAL_SPEED_TAKEN_OVERRIDE = TRUE;
                // driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED =
                // TRUE;
            }
            else
            {
                if (driver_requests->CONTROL_STATE == Cc_cc_active &&
                    das_input->LODM_STAT.STANDSTILL == FALSE)
                {
                    driver_requests->DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED =
                        TRUE;
                    driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 =
                        TRUE; /*SW18 for speed decrement by level
                                  1*/
                }
            }
        }
    }
}

static void VLC_ACCEL_SWITCH_PRESSED(const cart_das_input_data_t *das_input,
                                     cc_driver_inputs_t *driver_inputs,
                                     cc_driver_requests_t *driver_requests,
                                     const uint16 cc_start_repeat_fun_cycles,
                                     const uint16 cc_repeat_fun_cycles)
{
    boolean temp_condition;

    if (SWITCH_FALLING_EDGE(
            &driver_inputs->VLC_ACCEL_SWITCH_2,
            driver_requests->DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE) ==
        TRUE)
    {
        if ((driver_requests->CONTROL_STATE == Cc_cc_override) &&
            (driver_requests->DRIVER_REQUEST_STATUS
                 .ACTUAL_SPEED_TAKEN_OVERRIDE == FALSE) &&
            (driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT ==
             FALSE) &&
            ((driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET /
              Speedo_speed_s) > (sint16)driver_requests->VLC_SETSPEED))
        {
            driver_requests->DRIVER_REQUEST_STATUS.ACTUAL_SPEED_TAKEN_OVERRIDE =
                TRUE;
            driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = TRUE;
        }
        else
        {
            if (driver_requests->CONTROL_STATE == Cc_cc_active &&
                das_input->LODM_STAT.STANDSTILL == FALSE)
            {
                driver_requests->DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED =
                    TRUE;
                driver_requests->DRIVER_REQUEST_STATUS.TIMED_INCREMENT = TRUE;
                driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 = FALSE;
            }
        }
    }
    else
    {
        temp_condition =
            (boolean)((driver_requests->DRIVER_REQUEST_STATUS
                           .SELECTED_FUNCTION_ACTIVE == TRUE) &&
                      (driver_inputs->VLC_ACCEL_SWITCH_2.AKT_STATUS == FALSE));
        if (SWITCH_FALLING_EDGE(&driver_inputs->VLC_ACCEL_SWITCH_1,
                                temp_condition) == SWITCH_STATE_ON)
        {
            if ((driver_requests->CONTROL_STATE == Cc_cc_override) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .ACTUAL_SPEED_TAKEN_OVERRIDE == FALSE) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .OVERRIDE_WHILE_ENGAGEMENT == FALSE) &&
                ((driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET /
                  Speedo_speed_s) > (sint16)driver_requests->VLC_SETSPEED))
            {
                driver_requests->DRIVER_REQUEST_STATUS
                    .ACTUAL_SPEED_TAKEN_OVERRIDE = TRUE;
                driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = TRUE;
                // driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED
                // = TRUE;
            }
            else
            {
                if (driver_requests->CONTROL_STATE == Cc_cc_active &&
                    das_input->LODM_STAT.STANDSTILL == FALSE)
                {
                    driver_requests->DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED =
                        TRUE;
                    driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 = TRUE;
                }
            }
        }
    }
}

static void VLC_HOLD_TIME_RESET(const times_t cycle_time,
                                cc_driver_inputs_t *driver_inputs,
                                cc_driver_requests_t *driver_requests)
{
    // boolean temp_condition;
    if (SWITCH_RISING_EDGE(
            &driver_inputs->VLC_RESUME_SWITCH,
            driver_requests->DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE) ==
        SWITCH_STATE_ON)

    {
        if (driver_requests->CONTROL_STATE == Cc_cc_ready)
        {
            /* When driver pressses the resume switch the 5 second
            counter allowing automatic drive off starts counting
            from zero again (prolonging the time allowed for drive
            off) */
            driver_requests->DRIVE_OFF_TIME = 0u;
        }
    }
    else
    {
        /* When driver did not press the resume switch, then
        increase the drive off timer to count the number of
        miliseconds ellapsed, until Cc_drive_off_smooth_time is
        reached */
        if (driver_requests->DRIVE_OFF_TIME <
            Cc_drive_off_smooth_time) /*3 seconds counter added for
                                         auto drive-off as per
                                         SW18*/
        {
            driver_requests->DRIVE_OFF_TIME += cycle_time;
        }
        else
        {
            driver_requests->DRIVE_OFF_TIME = Cc_drive_off_smooth_time;
        }
    }
}

static void VLC_SET_SWITCH_PRESSED(
    const cart_das_input_data_t *das_input,
    const cc_input_data_t *input,
    cc_status_t *cc_status,
    cc_driver_inputs_t *driver_inputs,
    cc_driver_requests_t *driver_requests,
    const uint16 cc_repeat_fun_cycles,
    const uint16 cc_start_repeat_fun_cycles_acc_set,
    const ISAInfo *p_isa_info)
{
    boolean temp_condition;
    boolean temp_condition2;
    temp_condition = (boolean)(driver_requests->CONTROL_STATE == Cc_cc_ready);

    temp_condition2 = (p_isa_info->isa_speed_validity == ISA_SPEED_VALID ||
                       p_isa_info->isa_speed_validity == ISA_SPEED_RELEASE) &&
                      (driver_requests->CONTROL_STATE == Cc_cc_active ||
                       driver_requests->CONTROL_STATE == Cc_cc_override) &&
                      driver_inputs->VLC_SET_SWITCH.DURATION_TIME_ACTIVE <
                          cc_start_repeat_fun_cycles_acc_set;

    cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
        .PROPOSE_RECOMMENDED_SPEED = 0;
    cc_status->VLC_DRIVER_REQUESTS.RECOMMENDED_SPEED = 0;

    if (SWITCH_RISING_EDGE(&driver_inputs->VLC_SET_SWITCH, temp_condition) ==
        SWITCH_STATE_ON)
    {
        // Special design for Project EP40
        // if (driver_requests->VLC_SETSPEED >
        //     (setspeed_t)0) /* if set speed already saved */
        // {
        //     driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED = TRUE;
        //     driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT
        //     =
        //         TRUE;
        // } else
        if (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
                    TRUE &&
                driver_requests->SPEEDOMETER_VEHICLE_SPEED >= 0 ||
            cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
                    FALSE &&
                driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET >=
                    Cc_min_activespeed_kmh * Scale_100)
        {
            driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = TRUE;
            driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT =
                TRUE;
        }
    }
    else if (SWITCH_RISING_EDGE(&driver_inputs->VLC_SET_SWITCH,
                                temp_condition2) == SWITCH_STATE_ON)
    {
        driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT = TRUE;
        if (p_isa_info->isa_speed_validity == ISA_SPEED_VALID ||
            p_isa_info->isa_speed_validity == ISA_SPEED_RELEASE)
        {
            cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
                .PROPOSE_RECOMMENDED_SPEED = TRUE;
        }
        else
        {
            cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
                .PROPOSE_RECOMMENDED_SPEED = FALSE;
        }
        cc_status->VLC_DRIVER_REQUESTS.RECOMMENDED_SPEED =
            p_isa_info->isa_speed;
    }
    else if (SWITCH_HOLD_REPEAT(
                 &driver_inputs->VLC_SET_SWITCH, TRUE,
                 (driver_requests->CONTROL_STATE == Cc_cc_active ||
                  driver_requests->CONTROL_STATE == Cc_cc_override ||
                  driver_requests->CONTROL_STATE == Cc_cc_ready),
                 cc_start_repeat_fun_cycles_acc_set,
                 cc_repeat_fun_cycles) == TRUE)
    {
        driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = TRUE;
        driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT = TRUE;
    };
}

static void VLC_RESUME_SWITCH_PRESSED(const cart_das_input_data_t *das_input,
                                      cc_status_t *cc_status,
                                      cc_driver_inputs_t *driver_inputs,
                                      cc_driver_requests_t *driver_requests)
{
    boolean temp_condition;
    temp_condition = (boolean)((driver_requests->DRIVER_REQUEST_STATUS
                                    .SELECTED_FUNCTION_ACTIVE == FALSE) &&
                               (driver_requests->CONTROL_STATE == Cc_cc_ready));
    if (SWITCH_FALLING_EDGE(&driver_inputs->VLC_RESUME_SWITCH,
                            temp_condition) == SWITCH_STATE_ON)
    {
        if (driver_requests->VLC_SETSPEED > (setspeed_t)0 &&
            (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
                     TRUE &&
                 driver_requests->SPEEDOMETER_VEHICLE_SPEED >= 0 ||
             cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
                     FALSE &&
                 driver_requests->SPEEDOMETER_VEHICLE_SPEED >=
                     Cc_min_activespeed_kmh * Scale_100))
        {
            driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED = TRUE;
            driver_requests->DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT =
                TRUE;
        }
        //  else {
        //     driver_requests->DRIVER_OPERATIONS
        //         .VLC_TAKE_ACTUAL_SPEED = TRUE;
        //     // driver_requests->VLC_SETSPEED =
        //     //     Cc_min_setspeed_kmh;
        //     driver_requests->DRIVER_REQUEST_STATUS
        //         .ACTUAL_SPEED_TAKEN_SET = TRUE;
        //     driver_requests->DRIVER_REQUEST_STATUS
        //         .OVERRIDE_WHILE_ENGAGEMENT = TRUE;
        // }
    }
    else
    {
        /*resume switch pressed while engaged*/
        if ((SWITCH_RISING_EDGE(&driver_inputs->VLC_RESUME_SWITCH,
                                driver_requests->DRIVER_REQUEST_STATUS
                                    .SELECTED_FUNCTION_ACTIVE) ==
             SWITCH_STATE_ON))
        {
            if ((driver_requests->CONTROL_STATE == Cc_cc_override) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .ACTUAL_SPEED_TAKEN_OVERRIDE == FALSE) &&
                (driver_requests->DRIVER_REQUEST_STATUS
                     .OVERRIDE_WHILE_ENGAGEMENT == FALSE) &&
                ((driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET /
                  Speedo_speed_s) > (sint16)driver_requests->VLC_SETSPEED))
            {
                driver_requests->DRIVER_REQUEST_STATUS
                    .ACTUAL_SPEED_TAKEN_OVERRIDE = TRUE;
                driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = TRUE;
            }
            else
            {
                if (driver_requests->CONTROL_STATE == Cc_cc_active &&
                    das_input->LODM_STAT.STANDSTILL == TRUE)
                {
                    /* active and resume */
                    driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED =
                        TRUE;
                    drive_off_confirm_on_resume_pressed = TRUE;
                    driver_requests->DRIVER_REQUEST_STATUS
                        .OVERRIDE_WHILE_ENGAGEMENT = TRUE;
                }
            }
        }
    }
}

static void VLC_DRIVE_OFF_CONFIRM(const cart_das_input_data_t *das_input,
                                  const VLCSenAccOOI_t *pVLCAccOOIData,
                                  cc_status_t *cc_status,
                                  cc_driver_inputs_t *driver_inputs,
                                  cc_driver_requests_t *driver_requests,
                                  cc_driver_information_t *driver_information,
                                  cart_das_output_data_t *das_output)
{
    /* if driver braking, automatic drive off not possible*/

    if ((driver_requests->CONTROL_MODE == Cc_standstill_mode) &&
        ((das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL ==
          TRUE) /* if brake pedal pressed */
         || ((pVLCAccOOIData->AccOOINextLong.Attributes.uiObjectID ==
              OBJ_INDEX_NO_OBJECT) &&
             (last_effective_object_id != OBJ_INDEX_NO_OBJECT))))
    {
        driver_requests->DRIVE_OFF_TIME = Cc_drive_off_smooth_time;
        driver_requests->STAND_STILL_TIME = Cc_standstill_delay_time;
    }

    //(driver_requests->CONTROL_MODE == Cc_standstill_mode)
    // only in standstill
    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.Camera_Availability ==
        TRUE)
    {
        if (driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE == TRUE &&
            (driver_requests->DRIVE_OFF_TIME <
                 Cc_drive_off_smooth_time // drive off timer not expires
             || driver_requests->STAND_STILL_TIME <
                    Cc_standstill_delay_time) // standstill delay not expires
            &&
            cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
                TRUE &&
            (pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX >
                 DRIVE_OFF_LOW_ACCEL_REQ_DIST ||
             pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX >
                     DRIVE_OFF_HIGH_ACCEL_REQ_DIST &&
                 pVLCAccOOIData->AccOOINextLong.Kinematic.fVabsX >
                     DRIVE_OFF_HIGH_ACCEL_REQ_VEL / 3.6) &&
            VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.USR_Active ==
                FALSE)
        {
            driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM = TRUE; /* */
            drive_off_confirm_on_resume_pressed = FALSE;
        }
        else if (driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE ==
                     TRUE &&
                 (driver_requests->DRIVE_OFF_TIME >=
                      Cc_drive_off_smooth_time // drive off timer not expires
                  && driver_requests->STAND_STILL_TIME >=
                         Cc_standstill_delay_time) // standstill delay not
                                                   // expires
                 &&
                 (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
                              .OBJECT_EFFECTIVE == TRUE &&
                      (pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX >
                           DRIVE_OFF_LOW_ACCEL_REQ_DIST ||
                       pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX >
                               DRIVE_OFF_HIGH_ACCEL_REQ_DIST &&
                           pVLCAccOOIData->AccOOINextLong.Kinematic.fVabsX >
                               DRIVE_OFF_HIGH_ACCEL_REQ_VEL / 3.6) ||
                  cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
                          .OBJECT_EFFECTIVE == FALSE) &&
                 VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom
                         .USR_Active == FALSE &&
                 (VLCVEH_pLongCtrlResp->KinCtrlDynInput
                          .driver_override_accel_pedal == TRUE ||
                  drive_off_confirm_on_resume_pressed == TRUE))
        {
            driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM = TRUE; /* */
            drive_off_confirm_on_resume_pressed = FALSE;
        }
        else if (driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE ==
                     TRUE &&
                 driver_requests->DRIVE_OFF_TIME >=
                     Cc_drive_off_smooth_time &&
                 cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
                         .OBJECT_EFFECTIVE == TRUE &&
                 pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX >
                     DRIVE_OFF_LOW_ACCEL_REQ_DIST &&
                 VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom
                         .USR_Active == FALSE)
        {
            driver_information->DRIVER_CONFIRMATION_NEEDED = TRUE;
        }
        else if ((cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                      Cc_cc_active ||
                  cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE ==
                      Cc_cc_override) &&
                 driver_requests->MOVING_TIME < Cc_moving_time &&
                 (cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
                              .OBJECT_EFFECTIVE == TRUE &&
                      pVLCAccOOIData->AccOOINextLong.Kinematic.fDistX <=
                          DRIVE_OFF_LOW_ACCEL_REQ_DIST ||
                  VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom
                          .USR_Active == TRUE))
        {
            das_output->DAS_STAT.DAS_DRIVE_OFF_INHIBIT = TRUE;
        }
    }

    if (standstill_delay_cycle_counter >=
        VLC_T_STANDSTILL_WAIT_ELAPSED / VLC_VEH_CYCLE_TIME)
    {
        driver_requests->ENGAGEMENT_CONDITIONS.VLC_DISENGAGEMENT = TRUE;
        standwait_over_time_limit = TRUE;
    }

    if ((cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.OBJECT_EFFECTIVE ==
         FALSE) &&
        (das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL == FALSE) &&
        ((drive_off_confirm_on_resume_pressed == TRUE) ||
         (das_input->BRAKE_STAT.PEDAL_INIT_TRAVEL == TRUE)))
    {
        driver_requests->CONTROL_MODE =
            Cc_free_mode; // needs to be modified as cc_free_mode
                          // is not used anywhere later

        drive_off_confirm_on_resume_pressed = FALSE;
    }
}

static void VLC_STAND_STILL_ESTIMATE(const cart_das_input_data_t *das_input,
                                     cc_status_t *cc_status,
                                     cc_driver_requests_t *driver_requests)
{
    /*! only if acc active */
    if (cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS
            .SELECTED_FUNCTION_ACTIVE == TRUE)
    {
        if ((das_input->LODM_STAT.STANDSTILL == TRUE) &&
            (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.Door_Open ==
             TRUE))
        {
            if (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
                    .DRIVE_OFF_CONFIRM == TRUE)
            {
                cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = TRUE;
                /*! prevent automatic drive away  */
                cc_status->VLC_DRIVER_REQUESTS.DRIVE_OFF_TIME =
                    Cc_drive_off_smooth_time;
                cc_status->VLC_DRIVER_REQUESTS.STAND_STILL_TIME =
                    Cc_standstill_delay_time;
                driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM = FALSE;
            }
            else
            {
                /*! Stay in STAND_STILL_HOLD mode if active or do
                 * nothing because no drive off trigger send */
            }
        }
        else
        {
            if (cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD == TRUE)
            {
                if ((das_input->LODM_STAT.STANDSTILL == TRUE) &&
                    (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom
                         .Door_Open == FALSE))
                {
                    /*! drive off trigger needed */
                    if (cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
                            .DRIVE_OFF_CONFIRM == TRUE)
                    {
                        cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = FALSE;
                    }
                    else
                    {
                        /*! stay in STAND_STILL_HOLD mode */
                    }
                }
                else
                {
                    /*! VLC_CUSTOM_STAND_STILL_HOLD == TRUE and
                     * STANDSTILL == FALSE */
                    /* ?????????????????? */
                    cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = FALSE;
                }
            }
        }
    }
    else
    {
        cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = FALSE;
    }
}

/*****************************************************************************
  Functionname             VLC_DETERMINE_DRIVER_OPERATIONS */
/*!

@brief          Interpret driver input and categorize them in
DRIVER_OPERATIONS

@description    Interpret driver input and categorize them in
DRIVER_OPERATIONS


@param[in]       cycle_time : the cycle time (for non-first
call: ellapsed time since last call) in miliseconds
[times_t as per Rte_Type.h]
@param[in]	   das_input: data from longitudinal dynamics
management to driver assistance system
@param[in]	   input->pENGINE_ECU_INPUT->Ext_AccState:
external ACC state
[0...4]
@param[in.out]   driver_controls: selected CC function \n
driver_controls->SELECTED_FUNCTION_LAST_CYCLE:
selected CC function last cycle
[0, 1, 3]\n
driver_controls->SELECTED_FUNCTION:
selected CC function
[0, 1, 3]\n
@param[in]	   error_data: Data from longitudinal dynamics
management to driver assistance system
@param[in,out]   cc_status : the cruise control status
information \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS_LAST_CYCLE
: signals of driver_operations struct of former cycle
[cc_driver_operations_t as per cc_ext.h] \n
cc_status->VLC_DRIVER_REQUESTS.SELECTED_FUNCTION_LAST_CYCLE
: function selected by the driver in former cycle
[cc_selected_function_t as per cc_ext.h] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE
: ACC is active [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED
: if TRUE take actual vehicle speedometer speed as VLC_SETSPEED
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.LIM_TAKE_ACTUAL_SPEED
: if TRUE take actual vehicle speedometer speed as LIM_SETSPEED
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED
: if TRUE use the stored set speed as VLC_SETSPEED
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.LIM_RESUME_SET_SPEED
: if TRUE use the stored limiter speed as VLC_SETSPEED
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED
: if TRUE increase the set speed [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED
: if TRUE decrease the set speed [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.LIM_INCREASE_SET_SPEED
: if TRUE increase the LIM_SETSPEED [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.LIM_DECREASE_SET_SPEED
: if TRUE decrease the LIM_SETSPEED [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.SPEED_STEP_1
: if TRUE SPEED_STEP_1 is active [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.CANCEL_FUNKTION
: Cancel function triggered [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.RESET_SETSPEED
: if TRUE reset VLC_SETSPEED and LIM_SETSPEED to 0
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.SWITCH_SPEED_UNIT
: if speed unit is changed by driver [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM
: Boolean indicating vehicle drive off [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DISENGAGE_DRIVER_INTERVENED
: disengage by driver (Cancel switch, brake pedal)
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_REQUEST_STATUS.OVERRIDE_WHILE_ENGAGEMENT
: [TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.DECEL_MOD:
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.ACCEL_MODE:
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE:
conditions fullfilled for driveoff
[TRUE, FALSE] \n
cc_status->VLC_DRIVER_REQUESTS.DRIVE_OFF_TIME:
[times_t as per Rte_Type.h]\n
cc_status->VLC_DRIVER_REQUESTS.CONTROL_MODE:
cc control mode
(Cc_standstill_mode,Cc_follow_mode,Cc_free_mode)
[0,1,2]\n
@param[in]	   acc_output->SITUATION_CLASS.SITUATION:
estimated traffic situation
[acc_situation_class_t as per Rte_Type.h]

@return         void

@glob_in         None
@glob_out        None

@c_switch_part   CFG_VLC_FSRACC : Configuration switch for
enabling Full Speed Range ACC
@c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration
switch for enabling VLC_LONG processing

@pre             None
@post            None

@created         -
@changed         -


*****************************************************************************/
static void VLC_DETERMINE_DRIVER_OPERATIONS(
    const times_t cycle_time,
    cart_das_input_data_t *das_input,
    const cc_input_data_t *input,
    cc_driver_controls_t *driver_controls,
    const cc_error_data_t *error_data,
    cc_status_t *cc_status,
    cc_driver_inputs_t *driver_inputs,
    cc_driver_information_t *driver_information,
    cart_das_output_data_t *das_output,
    const VLCSenAccOOI_t *pVLCAccOOIData,
    const ISAInfo *p_isa_info)
{
    // boolean temp_condition;
    /*! Calculate the number of cycles for Cc_start_repeat_function_time here
    once. Note: would be better to adapt switch to time instead of cycles... */
    const uint16 cc_start_repeat_fun_cycles =
        (uint16)(Cc_start_repeat_function_time / cycle_time);

    /*! Calculate the number of cycles for Cc_start_repeat_function_time here
        it is used for counting ACC set switch on time...
     */
    const uint16 cc_start_repeat_fun_cycles_acc_set =
        (uint16)(Cc_start_repeat_function_time_2 / cycle_time);

    /*! Calculate the number of cycles for Cc_repeat_function_time once. */
    const uint16 cc_repeat_fun_cycles =
        (uint16)(Cc_repeat_function_time / cycle_time);
    /* Get utility pointer to driver requests (used in filling) */
    cc_driver_requests_t *const driver_requests =
        &cc_status->VLC_DRIVER_REQUESTS;

    /*copy last cycle operations*/
    driver_requests->DRIVER_OPERATIONS_LAST_CYCLE =
        driver_requests->DRIVER_OPERATIONS;
    driver_controls->SELECTED_FUNCTION_LAST_CYCLE =
        driver_controls->SELECTED_FUNCTION;

    /* Interpret DriverActions */
    driver_requests->DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE = FALSE;
    driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.LIM_TAKE_ACTUAL_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.LIM_RESUME_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.VLC_INCREASE_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.VLC_DECREASE_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.LIM_INCREASE_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.LIM_DECREASE_SET_SPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.SPEED_STEP_1 = FALSE;
    driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION = FALSE;
    /* driver_requests->DRIVER_OPERATIONS.DISENGAGE_DRIVER_INTERVENED = FALSE;
     * will be set in VLC_INHIBITION_CHECK */
    driver_requests->DRIVER_OPERATIONS.RESET_SETSPEED = FALSE;
    driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT = FALSE;
    driver_requests->DRIVER_OPERATIONS.DRIVE_OFF_CONFIRM = FALSE;

    driver_requests->DRIVER_OPERATIONS.DECEL_MODE = FALSE;
    driver_requests->DRIVER_OPERATIONS.ACCEL_MODE = FALSE;

    driver_information->DRIVER_CONFIRMATION_NEEDED = FALSE;
    das_output->DAS_STAT.DAS_DRIVE_OFF_INHIBIT = FALSE;

    if (driver_inputs->VLC_ACCEL_SWITCH_2.AKT_STATUS == FALSE)
    {
        driver_requests->DRIVER_REQUEST_STATUS.TIMED_INCREMENT = FALSE;
    }

    if (driver_inputs->VLC_DECEL_SWITCH_2.AKT_STATUS == FALSE)
    {
        driver_requests->DRIVER_REQUEST_STATUS.TIMED_DECREMENT = FALSE;
    }

    if (driver_controls->SELECTED_FUNCTION == Cc_function_acc ||
        driver_controls->SELECTED_FUNCTION == Cc_function_cc)

    {
        if ((driver_requests->CONTROL_STATE == Cc_cc_engage) ||
            (driver_requests->CONTROL_STATE == Cc_cc_active) ||
            (driver_requests->CONTROL_STATE == Cc_cc_override))
        {
            driver_requests->DRIVER_REQUEST_STATUS.SELECTED_FUNCTION_ACTIVE =
                TRUE;
        }

        if (error_data->VLC_INHIBIT ==
            FALSE) /*!!! what if error and switch trigger?*/
        {
            /*cancel switch or activated lim function*/
            if ((driver_inputs->VLC_CANCEL_SWITCH.AKT_STATUS == TRUE) ||
                (driver_controls->SELECTED_FUNCTION_LAST_CYCLE !=
                 driver_controls->SELECTED_FUNCTION))
            {
                driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION = TRUE;
                driver_requests->DRIVER_OPERATIONS.DISENGAGE_DRIVER_INTERVENED =
                    TRUE;
                cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.CANCEL_RAMP =
                    TRUE;
            }
            else
            {
                /*decel switch pressed and hold*/
                VLC_DECEL_SWITCH_PRESSED(
                    das_input, driver_inputs, driver_requests,
                    cc_start_repeat_fun_cycles, cc_repeat_fun_cycles);

                /*accel switch pressed an hold*/
                VLC_ACCEL_SWITCH_PRESSED(
                    das_input, driver_inputs, driver_requests,
                    cc_start_repeat_fun_cycles, cc_repeat_fun_cycles);

                /*Extra Timer reset, when the function is still active.*/
                VLC_HOLD_TIME_RESET(cycle_time, driver_inputs, driver_requests);

                /*drive off confirm logic*/
                VLC_DRIVE_OFF_CONFIRM(das_input, pVLCAccOOIData, cc_status,
                                      driver_inputs, driver_requests,
                                      driver_information, das_output);

                /*! STAND_STILL_HOLD */
                VLC_STAND_STILL_ESTIMATE(das_input, cc_status, driver_requests);

                /*set switch pressed while ACC disengaged*/
                VLC_SET_SWITCH_PRESSED(
                    das_input, input, cc_status, driver_inputs, driver_requests,
                    cc_repeat_fun_cycles, cc_start_repeat_fun_cycles_acc_set,
                    p_isa_info);

                /*resume switch pressed once while disengaged*/
                VLC_RESUME_SWITCH_PRESSED(das_input, cc_status, driver_inputs,
                                          driver_requests);
            }
        }
    }

    if (driver_requests->DRIVER_OPERATIONS.CANCEL_FUNKTION == TRUE)
    {
        driver_requests->DRIVER_OPERATIONS.RESET_SETSPEED = TRUE;
    }

    if (driver_requests->CONTROL_STATE != Cc_cc_override)
    {
        if ((driver_requests->DRIVER_OPERATIONS.VLC_TAKE_ACTUAL_SPEED ==
             FALSE) &&
            (driver_requests->DRIVER_OPERATIONS.VLC_RESUME_SET_SPEED ==
             FALSE) &&
            (driver_requests->CONTROL_STATE != Cc_cc_engage))
        {
            if (cc_status->VLC_ENGAGE_OVERRIDE_COUNTER > 0u)
            {
                cc_status->VLC_ENGAGE_OVERRIDE_COUNTER--;
            }
            else
            {
                driver_requests->DRIVER_REQUEST_STATUS
                    .OVERRIDE_WHILE_ENGAGEMENT = FALSE;
            }
        }
        else
        {
            cc_status->VLC_ENGAGE_OVERRIDE_COUNTER =
                (uint16)(Cc_engage_override_time / cycle_time);
        }
    }
    else
    {
        cc_status->VLC_ENGAGE_OVERRIDE_COUNTER =
            (uint16)(Cc_engage_override_time / cycle_time);
    }

    if ((driver_requests->CONTROL_STATE != Cc_cc_override) ||
        ((driver_requests->SPEEDOMETER_VEHICLE_SPEED_OFFSET / Speedo_speed_s) >
         (sint16)driver_requests->VLC_SETSPEED))
    {
        driver_requests->DRIVER_REQUEST_STATUS.ACTUAL_SPEED_TAKEN_OVERRIDE =
            FALSE;
    }
    /*! Detection of speedometer speed unit change */
    if ((uint8)input->VEHICLE_STATUS.SPEED_UNIT_KMH !=
        cc_status->VLC_SPEED_UNIT_LAST_CYCLE)
    {
        driver_requests->DRIVER_OPERATIONS.SWITCH_SPEED_UNIT = TRUE;
    }

    if (driver_controls->SELECTED_FUNCTION == Cc_function_cc)
    {
        driver_requests->CONTROL_MODE = Cc_free_mode;
    }
}

/*****************************************************************************
  Functionname             VLC_SIGNAL_INHIBITION_CHECK */
/*!

@brief          Process all data that inhibits the DAS
for a specific customer.

@description    Process all data that inhibits the DAS
for a specific customer.

@param[in]       pVehDyn: VDY (vehicle dynamics) data \n
   pVehDyn->State: VDY
inhibition state
[State_array_t as per Rte_Type.h]
@param[in, out]  cc_input_data : Cruise control input
data \n
cc_input_data->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP:
internal CC inhibition to disengage					[TRUE,
FALSE]\n
   cc_input_data->INHIBIT.VLC_INHIBIT_ENGAGEMENT:
internal cc inhibition for engagement					[TRUE,
FALSE]
@param[in]	   Inhibit_nr: inihibition number
[full range of unsigned short]
@param[in]	   pLongCtrlResp : Longitudinal Control
Response usually from brake ECU \n
pLongCtrlResp->sSigHeader.eSigStatus:
signal valid status
[AlgoSignalState_t as per Rte_Type.h]


@glob_in         None
@glob_out        None


@c_switch_part   LONG_CFG_USE_EXTERN_AVLC_STATE_MACHINE:
Configuration switch for unsing external state machine

@c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration
switch for enabling VLC_LONG processing

@pre             None
@post            None

@return         None

*****************************************************************************/
static void VLC_SIGNAL_INHIBITION_CHECK(
    const VED_VehDyn_t *pVehDyn,
    const VLC_AccLeverInput_t *pAccLever,
    const VLC_LongCtrlInput_t *pLongCtrlResp,
    cc_input_data_t *cc_input_data,
    uint16 *Inhibit_nr)
{
    // static vlc_inhibit_storage_t INHIBIT_BUFFER;/*Already declared as a
    // global static variable*/
    vlc_inhibit_t LocalInhibitionBuffer;
    VLC_OP_MODE_t LocVLCOpMode;

    /*! Start Signal Inhibition Check */

    VLC_INHIBIT_START_CYCLE(&LocalInhibitionBuffer);

    VLC_INHIBIT_SET_EXTERNAL_MODE(&INHIBIT_BUFFER);

    *Inhibit_nr = 0u;

    /* Disabled temporarily because eSigStatus not working properly.*/
    VLC_INHIBIT_ADD_INHIBITION(
        &LocalInhibitionBuffer,
        (uint8)((pAccLever == NULL) ||
                0 /*(pAccLever->sSigHeader.eSigStatus != AL_SIG_STATE_OK)*/),
        Fct_inhibit_FctACC | Fct_inhibit_FctCC | Fct_inhibit_DspTmpNotAv);

    /* Check longitudinal controller response */
    VLC_INHIBIT_ADD_INHIBITION(
        &LocalInhibitionBuffer,
        (uint8)((pLongCtrlResp == NULL) ||
                (pLongCtrlResp->sSigHeader.eSigStatus != AL_SIG_STATE_OK)),
        (Fct_inhibit_FctACC | Fct_inhibit_FctFCA | Fct_inhibit_FctDM |
         Fct_inhibit_DspTmpNotAv));

    VLC_INHIBIT_ADD_INHIBITION(
        &LocalInhibitionBuffer,
        (uint8)((pVehDyn == NULL) ||
                (VED_GET_IO_STATE(VED_SOUT_POS_VEL, pVehDyn->State) !=
                 VED_IO_STATE_VALID)),
        (Fct_inhibit_FctCC | Fct_inhibit_DspTmpNotAv));

    VLC_INHIBIT_ADD_INHIBITION(
        &LocalInhibitionBuffer,
        (uint8)((pVehDyn == NULL) ||
                (VED_GET_IO_STATE(VED_SOUT_POS_MSTAT, pVehDyn->State) !=
                 VED_IO_STATE_VALID)),
        (Fct_inhibit_FctCC | Fct_inhibit_DspTmpNotAv));

    *Inhibit_nr = LocalInhibitionBuffer.INHiBIT_NR;

    VLC_INHIBIT_FINISH_CYCLE(&LocalInhibitionBuffer, &INHIBIT_BUFFER,
                             Fct_inhibit_task_LC_EXEC);

    LocVLCOpMode = VLC_INHIBIT_GET_MODE(&INHIBIT_BUFFER);
    if ((LocVLCOpMode != VLC_MOD_RUNNING))
    {
        cc_input_data->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP = TRUE;
    }
    else
    {
        cc_input_data->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP = FALSE;
    }

    /* VLC_INHIBIT_ENGAGEMENT has no effect if
     * LONG_CFG_USE_EXTERN_AVLC_STATE_MACHINE is TRUE*/
    cc_input_data->INHIBIT.VLC_INHIBIT_ENGAGEMENT = VLC_INHIBIT_GET_INHIBITION(
        LocalInhibitionBuffer, Fct_inhibit_FctCC | Fct_inhibit_FctACC);

    //justin_add
    cc_input_data->INHIBIT.VLC_INHIBIT_ENGAGEMENT = FALSE;
    cc_input_data->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP = FALSE;

    /*! End Signal Inhibition Check */
}

/*****************************************************************************
  Functionname             VLC_INHIBITION_CHECK */
/*!

@brief          Process all data that inhibits the
DAS for a specific customer.
@description    Process all data that inhibits the
DAS for a specific customer.

@param[in]       das_input_data : Data from
longitudinal dynamics management to driver
assistance system \n
das_input_data->LODM_STAT.DAS_ENABLE:
inhibition by LODM
[TRUE, FALSE]
@param[in]	   acc_output: ACC output data from
VLC_SEN to VLC_VEH \n
                 acc_output->AVLC_OUTPUT_STATUS.INHIBITED:
inhibition by ACC algo in vlc sen
[TRUE, FALSE]
@param[in,out    input : CC input interface \n
input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP:
internal CC inhibition to disengage
[TRUE, FALSE] \n
@param[in,out    cc_status : CC status data \n
                   cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.NO_RAMP:�accel
control in case of inhibition		 	[TRUE,
FALSE] \n
@param[in,out]   error_data->VLC_INHIBIT:
inhibition or not
[TRUE, FALSE]

@glob_in         None
@glob_out        None


@c_switch_full   VLC_CFG_LONG_PROCESSING :
Configuration switch for enabling VLC_LONG
processing

@pre             None
@post            None

*****************************************************************************/
static void VLC_INHIBITION_CHECK(const cart_das_input_data_t *das_input_data,
                                 const VLC_acc_output_data_t *acc_output,
                                 const VLCSenAccOOI_t *pVLCAccOOIData,
                                 const VED_VehDyn_t *pVehDyn,
                                 const VLC_LongCtrlInput_t *pLongCtrlResp,
                                 const PACCInfo *p_pacc_info,
                                 cc_input_data_t *input,
                                 cc_error_data_t *error_data,
                                 cc_status_t *cc_status)
{
    boolean temp_condition = FALSE;

    /*! Start custom specific code */

    /*! Inhibit Engagement */
    if (acc_output->AVLC_OUTPUT_STATUS.INHIBITED == TRUE)
    {
        // do nothing, it is written to prevent warnings that arise because of
        // not using acc_output in this function
    }

    if ((input->INHIBIT.VLC_INHIBIT_ENGAGEMENT == FALSE) &&
        ((acc_output->AVLC_OUTPUT_STATUS.INHIBITED ==
          TRUE) /* DTRplus_Ueb_Zert_499 */
         || ((setspeed_t)(input->SPEEDOMETER_VEHICLE_SPEED /
                          (speedometer_speed_t)Scale_100) <
             cc_status->VLC_DRIVER_REQUESTS
                 .MIN_VLC_SETSPEED) /* DTRplus_Ueb_Zert_422 */
                && ((cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
                         .OBJECT_EFFECTIVE == FALSE) &&
                    ((das_input_data->LODM_STAT.STANDSTILL == TRUE) &&
                     (das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL ==
                      TRUE) // open acc output when ego car standstill
                     ))))
    {
        input->INHIBIT.VLC_INHIBIT_ENGAGEMENT = TRUE;
    }

    /*******************************************/
    /*! STAND_STILL_HOLD Logic moved to VLC_DETERMINE_DRIVER_OPERATIONS
     * function, because new drive off logic implemented and existing concept
     * not work */
    /*******************************************/

    temp_condition =
        (boolean)((cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD ==
                   TRUE) /* DTRplus_Ueb_Zert_581 */
                  && (
                         //(das_input_data->CHASSIS_STAT.PB_ACT == TRUE) /*
                         // DTRplus_Ueb_Zert_510, DTRplus_Ueb_Zert_582 */
                         (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom
                              .EPB_Active == TRUE) ||
                         (das_input_data->LODM_STAT.OVERRIDE_ACCEL ==
                          TRUE) /* DTRplus_Ueb_Zert_584 */
                         || (das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL ==
                             TRUE) /* DTRplus_Ueb_Zert_585 */

                         ));

    /*! disengagement with Ramp */
    if ((input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == FALSE) &&
        ((cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS.CANCEL_FUNKTION ==
          TRUE) ||
         /*! TO DO DTRplus_Ueb_Zert_500 */
         (VLCVehFrame.eVLCOpMode != VLC_MOD_RUNNING) /* probably not working */
         ))
    {
        input->INHIBIT.VLC_DISENGAGEMENT_RAMP = TRUE; /*SW18 Cancel Ramp Done*/
    }
    else
    {
        input->INHIBIT.VLC_DISENGAGEMENT_RAMP = FALSE;
    }
    /* end disengagement with Ramp */

    /* TCS, HDC or ABS active time count */
    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.ABS_Active == TRUE)
    {
        if (abs_active_counter < UINT16_MAX_LIMIT)
        {
            abs_active_counter++;
        }
    }
    else
    {
        abs_active_counter = 0;
    }

    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.ESC_Active == TRUE)
    {
        if (esc_active_counter < UINT16_MAX_LIMIT)
        {
            esc_active_counter++;
        }
    }
    else
    {
        esc_active_counter = 0;
    }

    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.TCS_Active == TRUE)
    {
        if (tcs_active_counter < UINT16_MAX_LIMIT)
        {
            tcs_active_counter++;
        }
    }
    else
    {
        tcs_active_counter = 0;
    }

    if (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.ARP_Active == TRUE)
    {
        if (arp_active_counter < UINT16_MAX_LIMIT)
        {
            arp_active_counter++;
        }
    }
    else
    {
        arp_active_counter = 0;
    }

    if (VLCVEH_pLongCtrlResp->KinCtrlDynInput.seatbelt_state !=
        SEATBELT_DRIVER_CLOSED)
    {
        if (seat_belt_unbuckle_counter < UINT16_MAX_LIMIT)
        {
            seat_belt_unbuckle_counter++;
        }
    }
    else
    {
        seat_belt_unbuckle_counter = 0;
    }

    /* Check if the OOI object disappear unnormally */
    VLC_OOI_DISAPPEAR_CHECK(pVLCAccOOIData, &acc_ooi_status);

    /*! disengagement no Ramp */
    if (((das_input_data->BRAKE_STAT.PEDAL_INIT_TRAVEL == TRUE)
    // Special design for Project EP40
#if STAND_STILL_BRAKE_EXIT_ACC
         && (das_input_data->LODM_STAT.STANDSTILL == FALSE)
#endif
             )
        // Special design for Project Pilot
        //      || (das_input_data->LODM_STAT.STANDSTILL == TRUE &&
        //  (fABS(VLCVEH_pLongCtrlResp->KinCtrlDynInput.steer_angle) >
        //       VLC_STEER_ANGLE_ACC_EXIT ||
        //   fABS(VLCVEH_pLongCtrlResp->KinCtrlDynInput.steer_speed) >
        //       VLC_STEER_SPEED_ACC_EXIT))
    )
    {
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
            .DISENGAGE_DRIVER_INTERVENED = TRUE;
        input->INHIBIT.VLC_DISENGAGEMENT_RAMP = FALSE;

        input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP = FALSE;

        cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = FALSE;
        input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP =
            TRUE; /*Deceleration Ramp Done*/
    }
    else
    {
        /*! Automatic disengagement, no intervention from driver */
        cc_status->VLC_DRIVER_REQUESTS.DRIVER_OPERATIONS
            .DISENGAGE_DRIVER_INTERVENED = FALSE;

        if ((das_input_data->LODM_STAT.DAS_ENABLE ==
             FALSE) /* ACC on/off switch */
            || (VLCVEH_pLongCtrlResp->KinCtrlDynInput.DC_status_information ==
                DC_STATUS_NOT_AVAIL) ||
            temp_condition ||
            (((input->INHIBIT.VLC_DISENGAGEMENT_RAMP == TRUE) ||
              (input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP == TRUE)) &&
             (das_input_data->LODM_STAT.OVERRIDE_ACCEL == TRUE)) ||
            acc_ooi_status == Disappear_Sudden ||
            (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.AYC_Active ==
             TRUE) ||
            (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.AEB_Active ==
             TRUE) ||
            (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.EPB_Active ==
             TRUE) ||
            (VLCVEH_pLongCtrlResp->Custom.LongCtrlInputCustom.ESC_Active ==
             TRUE) ||
            (abs_active_counter >
             VLC_T_ABS_ACTIVE_ELAPSED / VLC_VEH_CYCLE_TIME) ||
            (esc_active_counter >
             VLC_T_HDC_ACTIVE_ELAPSED / VLC_VEH_CYCLE_TIME) ||
            (tcs_active_counter >
             VLC_T_TCS_ACTIVE_ELAPSED / VLC_VEH_CYCLE_TIME) ||
            (arp_active_counter >
             VLC_T_ARP_ACTIVE_ELAPSED / VLC_VEH_CYCLE_TIME) ||
            (seat_belt_unbuckle_counter >
             VLC_T_SEAT_BELT_ACTIVE_ELAPSED / VLC_VEH_CYCLE_TIME) ||
            (standstill_delay_cycle_counter >=
             VLC_T_STANDSTILL_WAIT_ELAPSED / VLC_VEH_CYCLE_TIME))
        {
            cc_das_custom_state.VLC_CUSTOM_STAND_STILL_HOLD = FALSE;
            input->INHIBIT.VLC_DISENGAGEMENT_RAMP = FALSE;
            input->INHIBIT.VLC_DISENGAGEMENT_NO_RAMP = FALSE;
            input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP =
                TRUE; /*SW18 Rapid Ramp Done*/
        }
        else
        {
            input->INHIBIT.VLC_DISENGAGEMENT_RAPID_RAMP = FALSE;
        }
    }

    /*! Error Case */
    if (das_input_data->LODM_STAT.DAS_RESET == TRUE)
    {
        error_data->VLC_INHIBIT = TRUE;
        VLC_LONG_VEH_INITIALIZED = FALSE;
    }
    else if ((das_input_data->LODM_STAT.DAS_ENABLE == FALSE) ||
             (error_data->REPORTED_ERROR != Cc_no_error))
    {
        error_data->VLC_INHIBIT = TRUE; /* function coded out */
        if (error_data->REPORTED_ERROR == Cc_error_service)
        {
            cc_status->VLC_DRIVER_REQUESTS.VLC_SETSPEED =
                (setspeed_t)0; /* reset set speed if ignition switch on/off
                                  needed */
        }
    }
    else if (das_input_data->LODM_STAT.DAS_INHIBIT == TRUE ||
             ((GET_EGO_RAW_DATA_PTR->MotionState.MotState ==
               VED_LONG_MOT_STATE_MOVE_RWD) &&
              (input->SPEEDOMETER_VEHICLE_SPEED > VLC_RWD_SPEED_ACC_INHIBIT))
             /* judge the moving direction and speed */
             || p_pacc_info->control_path.curvature >
                    1 / VLC_CURVE_RADIUS_ACC_INHIBIT ||
             // Special design for project ep40
             (sint16)(pLongCtrlResp->KinCtrlDynInput.sensor_long_accel *
                      Scale_1000) >
                 MAT_CALCULATE_PARAM_VALUE1D(Cc_max_accel_active_curve,
                                             Cc_max_accel_active_curve_points,
                                             das_input_data->VEHICLE_SPEED) ||
             //    (sint16)(pLongCtrlResp->KinCtrlDynInput.sensor_long_accel *
             //             Scale_1000) < Cc_min_accel_active_curve ||
             VLCVEH_pLongCtrlResp->KinCtrlDynInput.door_state_fl !=
                 DOOR_DRIVER_CLOSED ||
             VLCVEH_pLongCtrlResp->KinCtrlDynInput.hood_state !=
                 DOOR_DRIVER_CLOSED ||
             VLCVEH_pLongCtrlResp->KinCtrlDynInput.trunk_state !=
                 DOOR_DRIVER_CLOSED ||
             (GET_EGO_RAW_DATA_PTR->MotionState.MotState !=
              VED_LONG_MOT_STATE_STDST) &&
                 (VLCVEH_pLongCtrlResp->KinCtrlDynInput.door_state_fr !=
                      DOOR_DRIVER_CLOSED ||
                  VLCVEH_pLongCtrlResp->KinCtrlDynInput.door_state_rl !=
                      DOOR_DRIVER_CLOSED ||
                  VLCVEH_pLongCtrlResp->KinCtrlDynInput.door_state_rr !=
                      DOOR_DRIVER_CLOSED))
    {
        error_data->VLC_INHIBIT = FALSE;
    }
    else
    {
        error_data->VLC_INHIBIT = FALSE;
    }
}

/* **********************************************************************
  Functionname             VLC_LIMIT_LONG_ACCEL_CUSTOM */
/*!

@brief          Custom function to limit longitudinal
acceleration
according specific customer requirements

@description    Custom function to limit longitudinal
acceleration
according specific customer requirements


@glob_in         None
@glob_out        None

@c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration
switch for enabling VLC_LONG processing

@pre             None
@post            None

@return         None

****************************************************************************
*/
static void VLC_LIMIT_LONG_ACCEL_CUSTOM(void)
{
    /*! This is custom specific function, which can be used to change the
     * commanded acceleration according a custom specific requierements */
    /*! For example: if a customer specific assistance system is active, the
     * requested acceleration shall be limited to zero. */
}

static void VLC_ALERT_INFORM(const cc_driver_requests_t *driver_requests,
                             const VLC_acc_output_data_t *acc_output,
                             cc_driver_information_t *driver_information)
{
    if (((driver_requests->CONTROL_STATE == Cc_cc_engage) ||
         (driver_requests->CONTROL_STATE == Cc_cc_active)) &&
        (acc_output->AVLC_OUTPUT_STATUS.ALERT == TRUE))
    {
        driver_information->BIT_AVLC_ALERT = TRUE;
        /* Initialise timer to mark start of alert display for the first cycle
         */
        if ((uiAlertOverrideTimer == (times_t)0) &&
            (b_AlertTimerInit == FALSE))
        {
            uiAlertOverrideTimer = Cc_init_alert_override_time;
            b_AlertTimerInit = TRUE;
        }
    }
    else
    {
        driver_information->BIT_AVLC_ALERT = FALSE;
        if (b_AlertTimerInit == TRUE)
        {
            /* Change of boolean to mark end of first cycle of alert display */
            b_AlertTimerInit = FALSE;
        }
    }
}

static void VLC_ERROR_INFORM(const VLC_acc_output_data_t *acc_output,
                             const cc_error_data_t *error_data,
                             cc_driver_information_t *driver_information,
                             cart_das_output_data_t *das_output)
{
    if (error_data->REPORTED_ERROR == Cc_no_error)
    {
        das_output->DAS_STAT.DAS_FAIL_IRREVERSABLE = FALSE;
        das_output->DAS_STAT.DAS_FAIL_REVERSABLE = FALSE;

        if (acc_output->AVLC_OUTPUT_STATUS.INHIBITED == TRUE)
        {
            das_output->DAS_STAT.DAS_FAIL_REVERSABLE = TRUE;
            driver_information->REPORTED_ERROR = Cc_temp_unavailable;
        }
    }
    else
    {
        /*set acc main lamp to  (error)*/
        driver_information->REPORTED_ERROR = error_data->REPORTED_ERROR;
        if ((error_data->REPORTED_ERROR == Cc_temp_unavailable) ||
            (error_data->REPORTED_ERROR == Cc_performance_degradation))
        {
            das_output->DAS_STAT.DAS_FAIL_IRREVERSABLE =
                FALSE; /* technical error */
            das_output->DAS_STAT.DAS_FAIL_REVERSABLE =
                TRUE; /* customer error */
        }
        else
        {
            das_output->DAS_STAT.DAS_FAIL_IRREVERSABLE =
                TRUE; /* technical error */
            das_output->DAS_STAT.DAS_FAIL_REVERSABLE =
                FALSE; /* customer error */
        }
    }
}

/*****************************************************************************
  Functionname             VLC_INFORM_DRIVER */
/*!

@brief          Process all data to be displayed to the driver for a specific customer.
@description    Process all data to be displayed to the driver for a specific customer.

@return           void

@param[in]       driver_requests : Data that reflects the drivers request intention \n
driver_requests->OPERATIONAL_MODE: driver assistance system operational mode							[0...13]\n
driver_requests->RECOMMENDED_SPEED: recommended speed													[setspeed_t as per Rte_Type.h]\n
driver_requests->VLC_SETSPEED: driver setspeed															[setspeed_t as per Rte_Type.h]\n
driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE: information if takeoff is possible			[TRUE, FALSE]\n
driver_requests->STAND_STILL_TIME: duration of standstill												[times_t as per Rte_Type.h]\n
driver_requests->DRIVE_OFF_TIME																			[times_t as per Rte_Type.h]\n
@param[in]       cc_input : Cruise control input data \n
cc_input->pENGINE_ECU_INPUT->Ext_AccState: external ACC state											[1...4]
@param[in]	   acc_output: ACC output data from VLC_SEN to VLC_VEH \n
acc_output->HEADWAY_SETTING: headway setting															[percentage_t as per Rte_Type.h]\n
acc_output->AVLC_OUTPUT_STATUS.ALERT:																	[TRUE, FALSE]\n
acc_output->REQUESTED_DISTANCE: driver requested distance												[distance_t as per Rte_type.h]\n
acc_output->AVLC_OUTPUT_STATUS.INHIBITED: ACC inhibit status												[TRUE,FALSE]\n
@param[in]	   fca_output->FCA_STATUS.FCA_ALERT: forward collision alert status											[TRUE, FALSE]
@param[in]	   dm_output->DM_ALERT_LEVEL: driver monitoring alert level													[TRUE, FALSE]
@param[in]	   display_object: ACC output data from VLC_SEN to VLC_VEH \n
display_object->AUTOSAR.OBJECT_STATUS.DETECTED: signal if target is detected							[TRUE, FALSE]\n
display_object->LONG_SPEED: target longitudinal speed													[velocity_t as per Rte_Type.h]\n
display_object->AUTOSAR.LONG_DISPLACEMENT: target longitudinal displacement								[distance_t as per Rte_type.h]
@param[in]	   error_data: CC error data \n
error_data->DISENGAGE_SIGNAL:																			[TRUE, FALSE]\n
error_data->REPORTED_ERROR:																				[cc_reported_error_t as per Rte_Type.h]
@param[in,out]   das_output : Data from driver assistance system to longitudinal dynamics management \n
das_output->DAS_STAT.DAS_FAIL_IRREVERSABLE: irreversable fail of driver assistance system				[TRUE, FALSE] \n
das_output->DAS_STAT.DAS_FAIL_REVERSABLE: reversable fail of driver assistance system					[TRUE, FALSE] \n
@param[in,out]   driver_information : HMI output \n
driver_information->HEADWAY_SETTING: output headway setting												[percentage_t as per Rte_Type.h]\n
driver_information->OPERATIONAL_MODE: HMI output operational mode										[display_op_status_t as per Rte_Type.h]\n
driver_information->BIT_AVLC_ALERT: HMI output ACC function alert										[full range of unsigned char]\n
driver_information->BIT_FCA_ALERT: HMI output forward collision alert									[full range of unsigned char]\n
driver_information->BIT_DM_ALERT: HMI output driver monitoring alert									[full range of unsigned char]\n
driver_information->DM_STATE: HMI putput driver monitoring state										[full range of unsigned char]\n
driver_information->REQUESTED_DISTANCE: HMI output requested target distance							[full range of unsigned char]\n
driver_information->RECOMMENDED_SPEED: HMI output recommended speed										[setspeed_t as per Rte_Type.h]\n
driver_information->OBJECT_DISTANCE: HMI output target distance											[full range of unsigned char]\n
driver_information->OBJECT_DETECTED: HMI output if target detected										[TRUE, FALSE]\n
driver_information->OBJECT_SPEED: HMI output target longitudinal speed									[setspeed_t as per Rte_Type.h]\n
driver_information->SET_SPEED: HMI output setspeed														[setspeed_t as per Rte_Type.h]\n
driver_information->AVLC_DRIVE_OFF_POSSIBLE: HMI output if takeoff is possible							[full range of unsigned char]\n
driver_information->REPORTED_ERROR: HMI output of errors												[full range of unsigned char]\n

@glob_in         None
@glob_out        None


@c_switch_part   CFG_VLC_FCA: Configuration switch for using front collision alert
@c_switch_part   CFG_VLC_DM: Configuration switch for using driver monitoring
@c_switch_part   CFG_VLC_FSRACC: Configuration switch for using FSRACC
@c_switch_part   LONG_CFG_USE_DRIVER_DATA_ERROR_REPORTING: Configuration switch for reproting errors to driver
@c_switch_part   LONG_CFG_USE_EXTERN_AVLC_STATE_MACHINE : Configuration switch for using external statemachine

@c_switch_full   VLC_CFG_LONG_PROCESSING : Configuration switch for enabling VLC_LONG processing

@pre             None
@post            None


/*****************************************************************************
@fn             VLC_INFORM_DRIVER */
/*!

       @description    Process all data
       to be displayed to the driver for
       a specific customer.

       @param          cycle_time
       @param          driver_requests
       @param          acc_output
     */

/*!
  @param          display_object
  @param          cc_input
  @param          error_data
  @param          driver_information
  @param          das_output

  @return         void

*****************************************************************************/
static void VLC_INFORM_DRIVER(const times_t cycle_time,
                              const cc_driver_requests_t *driver_requests,
                              const VLC_acc_output_data_t *acc_output,
                              const VLC_acc_object_t *display_object,
                              const cc_error_data_t *error_data,
                              const cart_das_input_data_t *das_input_data,
                              const cc_input_data_t *cc_input,
                              const cc_status_t *cc_status,
                              cc_driver_information_t *driver_information,
                              cart_das_output_data_t *das_output)
{
    /*! Calculate Values to display */
    /*! Write DAI specific init values */

    driver_information->REQUESTED_DISTANCE = Cc_init_display_distance;
    driver_information->SET_SPEED = Cc_init_setspeed;
    driver_information->OBJECT_DISTANCE = Cc_init_display_object_distance;
    driver_information->OBJECT_DETECTED = FALSE;

    driver_information->OBJECT_SPEED = Cc_min_display_object_speed;

    /*! Calculate values only if ACC ON */
    if (das_input_data->LODM_STAT.DAS_ENABLE == TRUE) /* ACC coded */
    {
        if (driver_information->HEADWAY_SETTING !=
            acc_output->HEADWAY_SETTING)
        {
            if (uiInitHeadwaySettingTimer < cycle_time)
            {
                /*! will be set to FALSE only after the message was displayed */
                bHeadwayChangeDetected = TRUE;
            }
        }

        if (uiInitHeadwaySettingTimer >= cycle_time)
        {
            /*! headway setting was initialized */
            uiInitHeadwaySettingTimer -= cycle_time;
        }

        /*! limit requested distance till 150m */
        driver_information->REQUESTED_DISTANCE = (uint8)MAT_MIN(
            (((sint32)acc_output->REQUESTED_DISTANCE + (Distance_s / 2)) /
             Distance_s),
            (sint32)Cc_max_requested_distance);

        /* ACC was engaged befor */
        if (driver_requests->VLC_SETSPEED != (setspeed_t)0)
        {
            driver_information->SET_SPEED =
                (setspeed_t)MAT_MIN(driver_requests->VLC_SETSPEED,
                                    driver_requests->MAX_VLC_SETSPEED);
        }

        /*! Calculate Object values to display, Daimler specific */
        driver_information->OBJECT_SPEED = driver_information->SET_SPEED;

        /*! Delay logic for display object */
        /*! Delay only if transition from no object situation to
         * approach/following situation */
        if ((display_object->AUTOSAR.OBJECT_STATUS.NEW == TRUE) &&
            (last_object_id == OBJ_INDEX_NO_OBJECT))
        {
            /*! start timer */
            uiDisplayDelayTimer = Cc_display_delay_time;
        }

        if ((uiDisplayDelayTimer > cycle_time) &&
            (display_object->AUTOSAR.OBJECT_ID != OBJ_INDEX_NO_OBJECT))
        {
            uiDisplayDelayTimer -= cycle_time;
        }
        else
        {
            uiDisplayDelayTimer = 0u;
        }

        /* Objects to display */
        if ((display_object->AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) &&
            (uiDisplayDelayTimer < cycle_time) &&
            (((display_object->AUTOSAR.OBJECT_STATUS.STANDING == TRUE) &&
              (das_input_data->VEHICLE_SPEED <=
               (velocity_t)Acc_decel_on_stationary_speed)) ||
             (display_object->AUTOSAR.OBJECT_STATUS.STANDING == FALSE)))
        {
            sint32 help;
            help = display_object->LONG_SPEED;
            if (cc_input->VEHICLE_STATUS.SPEED_UNIT_KMH == TRUE)
            {
                help *= (sint32)Speed_conv_factor_kmh;
            }
            else
            {
                help *= (sint32)Speed_conv_factor_mph;
            }
            help /= (sint32)Factor_s;
            help += (sint32)(Velocity_s / 2);
            help /= (sint32)Velocity_s;
            driver_information->OBJECT_SPEED =
                (setspeed_t)MAT_LIM(help, (sint32)Cc_min_display_object_speed,
                                    (sint32)Cc_max_display_object_speed);
            driver_information->OBJECT_DISTANCE =
                (uint8)(((uint16)display_object->AUTOSAR.LONG_DISPLACEMENT +
                         (uint16)(Distance_s / 2)) /
                        (uint16)Distance_s);
            driver_information->OBJECT_DISTANCE =
                (uint8)MAT_MIN((sint32)driver_information->OBJECT_DISTANCE,
                               (sint32)Cc_max_requested_distance);
            driver_information->OBJECT_DETECTED = TRUE;

            /* Additional Speed calculation */
            if ((driver_requests->CONTROL_STATE == Cc_cc_engage) ||
                (driver_requests->CONTROL_STATE == Cc_cc_active) ||
                (driver_requests->CONTROL_STATE == Cc_cc_override))
            {
            }
            else
            {
                /* nothing to do */
            }
        }
    }

    /*! ACC Alert mapping
        In override state, alert only if minimum time of alert display is not
       over*/
    VLC_ALERT_INFORM(driver_requests, acc_output, driver_information);

    /* Alert display in override condition only if total alert display time is
     * lesser than minimum time */
    if ((driver_requests->CONTROL_STATE == Cc_cc_override) &&
        (uiAlertOverrideTimer > (times_t)0))
    {
        driver_information->BIT_AVLC_ALERT = TRUE;
    }

    SWITCH_SET_COUNTER(cycle_time, &uiAlertOverrideTimer);

    driver_information->BIT_FCA_ALERT = 0u;

    driver_information->BIT_DM_ALERT = 0u;
    driver_information->DM_STATE = 0u;

    driver_information->REQUESTED_DISTANCE = (uint8)MAT_MIN(
        (((sint32)acc_output->REQUESTED_DISTANCE + (Distance_s / DIVISIOR_2)) /
         Distance_s),
        MAX_VALUE_UNIT8);
    driver_information->RECOMMENDED_SPEED = driver_requests->RECOMMENDED_SPEED;

    driver_information->OBJECT_DISTANCE = 0u;
    driver_information->OBJECT_DETECTED = FALSE;
    driver_information->OBJECT_SPEED = 0u;

    /* assign relevant object attributes. note: display object is the next
     * object in host lane.*/
    if (display_object->AUTOSAR.OBJECT_STATUS.DETECTED == TRUE)
    {
        sint32 help;
        help = display_object->LONG_SPEED;
        help *= (sint32)Speed_conv_factor_kmh;
        help /= (sint32)Factor_s;
        help += (sint32)(Velocity_s / DIVISIOR_2);
        help /= (sint32)Velocity_s;
        driver_information->OBJECT_SPEED =
            (setspeed_t)MAT_LIM(help, (sint32)0, (sint32)MAX_VALUE_UNIT8);
        driver_information->OBJECT_DISTANCE =
            (uint8)(((uint16)display_object->AUTOSAR.LONG_DISPLACEMENT +
                     (uint16)(Distance_s / DIVISIOR_2)) /
                    (uint16)Distance_s);
        driver_information->OBJECT_DETECTED = TRUE;
    }
    else
    {
        driver_information->OBJECT_SPEED = driver_requests->VLC_SETSPEED;
    }

    /*assign set speed*/
    driver_information->SET_SPEED = driver_requests->VLC_SETSPEED;

    /*Inform Driver about drive off situation*/
    if ((driver_requests->ENGAGEMENT_CONDITIONS.DRIVE_OFF_POSSIBLE == TRUE) &&
        (driver_requests->STAND_STILL_TIME >= Cc_standstill_delay_time) &&
        (driver_requests->DRIVE_OFF_TIME >= Cc_drive_off_smooth_time))
    {
        driver_information->AVLC_DRIVE_OFF_POSSIBLE = TRUE;
    }
    else
    {
        driver_information->AVLC_DRIVE_OFF_POSSIBLE = FALSE;
    }

    VLC_ERROR_INFORM(acc_output, error_data, driver_information, das_output);
}

/******************************************************************************
  @fn             VLC_ISA_PROCESS */
/*!

        Description     Call isa(Intelegent
      Speed Assist) function

        @param[in]      cc_status
        @param[in]      pLongCtrlResp
        @param[out]     isa_info

        @return         void

      *****************************************************************************/
void VLC_ISA_PROCESS(const VLC_LongCtrlInput_t *pLongCtrlResp,
                     cc_status_t *cc_status,
                     ISAInfo *p_isa_info)
{
    boolean acc_state;

    acc_state =
        (cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active ||
         cc_status->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_override);
    p_isa_info->isa_state =
        pLongCtrlResp->Custom.LongCtrlInputCustom.ISA_Active;

    ISAProcess(VLC_VEH_CYCLE_TIME * 1000, 0,
               pLongCtrlResp->Custom.LongCtrlInputCustom.nav_info,
               pLongCtrlResp->Custom.LongCtrlInputCustom.tsr_info, acc_state,
               pLongCtrlResp->Custom.LongCtrlInputCustom.NNP_Active,
               cc_status->VLC_DRIVER_REQUESTS.VLC_SETSPEED,
               cc_status->VLC_DRIVER_REQUESTS.OPERATIONAL_MODE, p_isa_info);
}

/*****************************************************************************
  @fn             VLC_LC_EXEC */
/*!

               @description    Execute
               longitudinal functions
               (controller and hmi)

               @param[in]      cycle_time : the
               cycle time (for non-first call:
                               ellapsed time
               since last call) in miliseconds

               @param[in]      pVehDyn : the
               raw vehicle dynamics

               @param[in]      pPowerTrain :
               the powertrain information
               (gear)

               @param[in]      pAccLever : the
               ACC lever input

               @param[in]      pLongCtrlResp :
               the longitudinal controller
               response
             */

/*
  @param[out]     pLongCtrlCmd : the output for the longitudinal controller

  @return         void

*****************************************************************************/
void VLC_LC_EXEC(const times_t cycle_time,
                 const reqVLCVehDebugList_t *pVLCVehDebugPorts,
                 const VED_VehDyn_t *pVehDyn,
                 const PowerTrain_t *pPowerTrain,
                 const VLC_AccLeverInput_t *pAccLever,
                 const VLC_LongCtrlInput_t *pLongCtrlResp,
                 const VLC_acc_object_t *pAccDisplayObj,
                 const VLC_acc_output_data_t *pAccOutput,
                 const VLCSenAccOOI_t *pVLCAccOOIData,
                 const t_CamLaneInputData *pCamLaneData,
                 VLC_DFV2SenInfo_t *pDFVLongOut,
                 VLC_LongCtrlOutput_t *pLongCtrlCmd,
                 ISAInfo *p_isa_info,
                 PACCInfo *p_pacc_info)
{
    pLongCtrlCmd->uiVersionNumber = VLC_VEH_INTFVER;

    /*! Execute Init function if SW or HW reset */
    if ((VLCVehFrame.eVLCState == VLC_VEH_INIT) ||
        (VLC_LONG_VEH_INITIALIZED == FALSE))
    {
        VLC_LONG_VEH_INIT();
        VLC_LONG_VEH_INITIALIZED = TRUE;
    }

    VLC_LONG_VEH_WRAPPER_INPUT(
        cycle_time, pVehDyn, pAccLever, pLongCtrlResp,
        &VLC_DRIVER_INF.VLC_DRIVER_INF, &gDAS_INPUT_DATA.DAS_INPUT_DATA,
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS, &gVLC_STATUS.VLC_STATUS,
        pDFVLongOut, &gVLC_DRIVER_CNTRLS.VLC_DRIVER_CONTROLS,
        &gVLC_INPUT_DATA.VLC_INPUT_DATA, &gVLC_ERROR_DATA.VLC_ERROR_DATA);

    /*! Select Parameter depending of different Mods */
    VLC_SELECT_PARAM_SET();

    /*! Check signal states */
    VLC_SIGNAL_INHIBITION_CHECK(
        pVehDyn,

        pAccLever,

        pLongCtrlResp, &gVLC_INPUT_DATA.VLC_INPUT_DATA,
        &gVLC_VEH_DEBUG_DATA.VLC_VEH_DEBUG_DATA.Inhibit_nr);

    /*! Check logical inhibition conditions */
    VLC_INHIBITION_CHECK(
        &gDAS_INPUT_DATA.DAS_INPUT_DATA, pAccOutput, pVLCAccOOIData, pVehDyn,
        pLongCtrlResp, p_pacc_info, &gVLC_INPUT_DATA.VLC_INPUT_DATA,
        &gVLC_ERROR_DATA.VLC_ERROR_DATA, &gVLC_STATUS.VLC_STATUS);

    /*! Determ driver operations */
    VLC_DETERMINE_DRIVER_OPERATIONS(
        cycle_time, &gDAS_INPUT_DATA.DAS_INPUT_DATA,
        &gVLC_INPUT_DATA.VLC_INPUT_DATA,
        &gVLC_DRIVER_CNTRLS.VLC_DRIVER_CONTROLS,
        &gVLC_ERROR_DATA.VLC_ERROR_DATA, &gVLC_STATUS.VLC_STATUS,
        &gVLC_DRIVER_INPUTS.VLC_DRIVER_INPUTS, &VLC_DRIVER_INF.VLC_DRIVER_INF,
        &gDAS_OUTPUT_DATA.DAS_OUTPUT_DATA, pVLCAccOOIData, p_isa_info);

    /*! Custom function to limit longitudinal acceleration according customer
     * requirements */
    VLC_LIMIT_LONG_ACCEL_CUSTOM();

    VLC_ISA_PROCESS(pLongCtrlResp, &gVLC_STATUS.VLC_STATUS, p_isa_info);

    /*! Call main function */
    VLC_EXEC(cycle_time, &gVLC_DRIVER_CNTRLS.VLC_DRIVER_CONTROLS,
             &gVLC_INPUT_DATA.VLC_INPUT_DATA, &gVLC_ERROR_DATA.VLC_ERROR_DATA,
             &gDAS_INPUT_DATA.DAS_INPUT_DATA, &gDAS_OUTPUT_DATA.DAS_OUTPUT_DATA,
             pAccOutput, pCamLaneData, &gVLC_STATUS.VLC_STATUS, p_pacc_info);

    /*! Call display wrapper function */
    VLC_INFORM_DRIVER(
        cycle_time, &gVLC_STATUS.VLC_STATUS.VLC_DRIVER_REQUESTS, pAccOutput,
        pAccDisplayObj, &gVLC_ERROR_DATA.VLC_ERROR_DATA,
        &gDAS_INPUT_DATA.DAS_INPUT_DATA, &gVLC_INPUT_DATA.VLC_INPUT_DATA,
        &gVLC_STATUS.VLC_STATUS, &VLC_DRIVER_INF.VLC_DRIVER_INF,
        &gDAS_OUTPUT_DATA.DAS_OUTPUT_DATA);

    /*! Save das input data last cycle */
    DAS_INPUT_DATA_LAST_CYCLE = gDAS_INPUT_DATA.DAS_INPUT_DATA;

    VLC_LONG_VEH_DETERMINE_SOFT_STOP_REQUEST(
        &gDAS_OUTPUT_DATA.DAS_OUTPUT_DATA, &VLC_DRIVER_INF.VLC_DRIVER_INF,
        gDAS_INPUT_DATA.DAS_INPUT_DATA.VEHICLE_SPEED, pAccDisplayObj,
        &gVLC_STATUS.VLC_STATUS, pAccOutput, pLongCtrlCmd);

    /*! LODM_STAT.STANDSTILL triggers the default standstill acceleration
       request which brings the vehicle to a full stop. For Geely this should
       only happen in case the target is
        stationary and the distance is below 4 meters.*/
    // gDAS_INPUT_DATA.DAS_INPUT_DATA.LODM_STAT.STANDSTILL =
    //     (boolean)((pVehDyn->MotionState.MotState == VED_LONG_MOT_STATE_STDST)
    //     &&
    //               (pAccDisplayObj->AUTOSAR.LONG_DISPLACEMENT <=
    //                Cc_max_softstop_request_distance) &&
    //               ((pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STANDING == TRUE)
    //               ||
    //                (pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STOPPED == TRUE)));

    /*! Fill DFV2SenInfo Port */
    VLC_LONG_VEH_TO_SEN_INFO(pLongCtrlResp, &gVLC_STATUS.VLC_STATUS,
                             pDFVLongOut);

    /*! Call wrapper output function */
    VLC_LONG_VEH_WRAPPER_OUTPUT(
        cycle_time, pVLCVehDebugPorts, &gVLC_INPUT_DATA.VLC_INPUT_DATA,
        gDAS_INPUT_DATA.DAS_INPUT_DATA.VEHICLE_SPEED,
        &gDAS_OUTPUT_DATA.DAS_OUTPUT_DATA, &gVLC_STATUS.VLC_STATUS, pAccOutput,
        &VLC_DRIVER_INF.VLC_DRIVER_INF, &gVLC_ERROR_DATA.VLC_ERROR_DATA,
        &gDAS_INPUT_DATA.DAS_INPUT_DATA, pLongCtrlCmd);
}

/*********************************************************************************************************************
  Functionname               VLC_LONG_VEH_DETERMINE_SOFT_STOP_REQUEST */
/*!

@brief
determine soft stop request to
hold vehicle in standstill

@description      Determines the
soft stop request according to
customer requirement (ARS410GY18
specific). If the
relevant
object is stoped and if
additionally the ego velocity and
the distance to the relevant
object drop
below a certain threshold, a stop
is about to happen an the
SoftStopRequest is set
TRUE.

The soft stop
request is reset whenever the
SITUATION_CLASS.SITUATION
indicates the host is not
stopped.

             If soft stop request is set and vehicle speed is 0, the brake will hold the car.

@return			void

@param[in]
vehicle_speed: speed of ego
vehicle
[velocity_t as per Rte_Type.h]
@param[in]       pDasOutputData
: Data from driver assistance
system to longitudinal dynamics
management \n
pDasOutputData->DAS_STAT.DAS_ENGAGED:
status of ACC engagement
[TRUE, FALSE] \n
@param[in]
pDriverInformation : HMI Output
data \n
pDriverInformation->OBJECT_DETECTED
: target object detected
[full range of unsigned char] \n
@param[in]       pCCStatus : the
cruise control status information
\n
               pCCStatus->VLC_DRIVER_REQUESTS.CONTROL_STATE: internal cc state										[0...11] \n
@param[in]
pAccOutput->SITUATION_CLASS.SITUATION:
classification of estimated
traffic situation
[0...6]
@param[in]       pAccDisplayObj:
data regarding selected object
           pAccDisplayObj->AUTOSAR.LONG_DISPLACEMENT: longitudinal displacement of target							[distance_t as per Rte_Type.h]\n
           pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STANDING: signal if object is standing (not been moving)			[TRUE, FALSE]\n
           pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STOPPED: signal if object is right stopped							[TRUE, FALSE]\n
@param[in,out]   pLongCtrlCmd :
the output for the longitudinal
controller \n
pLongCtrlCmd->Custom.CustomOutput.SoftStopRequest:
request to keep vehicle in
standstill				[TRUE,
FALSE]

@glob_in         None
@glob_out        None


@c_switch_full
VLC_CFG_LONG_PROCESSING :
Configuration switch for enabling
VLC_LONG processing

@pre             None
@post            None

*********************************************************************************************************************/
static void VLC_LONG_VEH_DETERMINE_SOFT_STOP_REQUEST(
    const cart_das_output_data_t *pDasOutputData,
    const cc_driver_information_t *pDriverInformation,
    const velocity_t vehicle_speed,
    const VLC_acc_object_t *pAccDisplayObj,
    const cc_status_t *pCCStatus,
    const VLC_acc_output_data_t *pAccOutput,
    VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    if (((boolean)(pDasOutputData->DAS_STAT.DAS_ENGAGED) == TRUE) &&
        (pDriverInformation->OBJECT_DETECTED == TRUE) &&
        (pAccDisplayObj->AUTOSAR.LONG_DISPLACEMENT <=
         Cc_max_softstop_request_distance) &&
        (vehicle_speed <= Cc_max_softstop_request_velocity) &&
        ((pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STANDING == TRUE) ||
         (pAccDisplayObj->AUTOSAR.OBJECT_STATUS.STOPPED == TRUE))
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/93 changed by
        // guotao start
        &&
        ((boolean)(pDasOutputData->DAS_STAT.DAS_OVERRIDE) ==
         FALSE) // close SoftStopRequest switch in driver override situation)
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/93 changed by
        // guotao end
    )
    {
        pLongCtrlCmd->Custom.CustomOutput.SoftStopRequest = TRUE;
    }
    else
    {
        if ((pCCStatus->VLC_DRIVER_REQUESTS.CONTROL_STATE == Cc_cc_active) &&
                (pAccOutput->SITUATION_CLASS.SITUATION != Acc_sit_class_stop)
            // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/92 changed
            // by guotao start
            ||
            ((boolean)(pDasOutputData->DAS_STAT.DAS_OVERRIDE) ==
             TRUE) // close SoftStopRequest switch in driver override situation
            // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/92 changed
            // by guotao end
        )
        {
            pLongCtrlCmd->Custom.CustomOutput.SoftStopRequest = FALSE;
        }
        else
        {
            /* do nothing */
        }
    }
}

/*********************************************************************************************************************
  Functionname               VLC_LONG_VEH_EXT_VLC_ARBITRATION */
/*!

@brief
this function arbitrates
accel request dependent on
ACC and extern CC

@return           void

@description      This
function modifies the
acceleration request
output dependant on the
ego velocity and the
         in-lane
object detection status.
         In
speed range between 30 to
150kph either a stepwise
parameter curve or a
linearly interpolated
         parameter
curve is followed when
there is no in-lane object
detected and the lateral
limitation
         is
inactive.
         In
speed range between 0 to
30kph an additional
cancellation- and warn
request is set dependant
         on the
ACC mode (ACC feature
characteristic: ACC or
FSRA).

         Background:
         When
there is no relevant obj
detected, the vehicle
speed is controlled toward
the set speed by
         an
external conventional
cruise controller, running
on a separate ECU. In
order to arbitrate the
         CC-
and the ACC acceleration
requests a speed-dependant
default acceleration
request is set by
         the
ACC controller when there
is no relevant obj
detected.

@param[in]
cycle_time : the cycle
time (for non-first call:
ellapsed time since last
call) in miliseconds
[times_t as per
Rte_Type.h]
@param[in]
pCcInputData : Cruise
control input data \n
           pCcInputData->pENGINE_ECU_INPUT->Ext_AccMode:
external ACC Mode
[1, 2]
@param[in]
vehicle_speed: speed of
ego vehicle
[velocity_t as per
Rte_Type.h]
@param[in]
pDasOutputData : Data from
driver assistance system
to longitudinal dynamics
management \n
           pDasOutputData->DAS_STAT.DAS_ENGAGED:
status of ACC engagement
[TRUE, FALSE] \n
@param[in]
pDriverInformation : HMI
Output data \n
           pDriverInformation->OBJECT_DETECTED
: target object detected
[full range of unsigned
char] \n
@param[in]
p_cc_status : the cruise
control status information
\n
                               p_cc_status->VLC_ACCEL_CONTROL_DATA.MINIMUM_COMMANDED_ACCELERATION: requested accel. for ACC				[full range of acceleration_t type as defined in Rte_Type.h] \n
                               p_cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE: signal if curve limitation is active	[TRUE, FALSE] \n
                               p_cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_ACCELERATION_LATERAL_LIMITED: acceleration out of 3-dimensional lookup table for curve limitation [acceleration_t as per Rte_Type.h]
@param[in]
pAccOutput->SITUATION_CLASS.CRITICALITY:
criticality of current
traffic situation
[confidence_t as per
Rte_Type.h]
@param[in,out]
pLongCtrlCmd : the output
for the longitudinal
controller \n
           pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw:
requested raw acceleration
[acceleration_t as per
Rte_Type.h] \n
                               pLongCtrlCmd->Custom.CustomOutput.MaxOperatingSpeed: maximum allowed ACC speed						[velocity_t as per Rte_Type.h]\n
                               pLongCtrlCmd->Custom.CustomOutput.MinOperatingSpeed: minimum allowed ACC speed						[velocity_t as per Rte_Type.h]\n
                               pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancel_Rq: cancel request for extern statemachine				[TRUE, FALSE]\n
                               pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancelWarn_Rq: audbile warning for cancelation				[TRUE, FALSE]\n

@glob_in         None
@glob_out        None


@c_switch_full
VLC_CFG_LONG_PROCESSING :
Configuration switch for
enabling VLC_LONG
processing

@pre             None
@post            None


*********************************************************************************************************************/
static void VLC_LONG_VEH_EXT_VLC_ARBITRATION(
    const times_t cycle_time,
    const velocity_t vehicle_speed,
    const cc_input_data_t *pCcInputData,
    const cc_status_t *p_cc_status,
    const cart_das_output_data_t *pDasOutputData,
    const cc_driver_information_t *pDriverInformation,
    VLC_LongCtrlOutput_t *pLongCtrlCmd)
{
    static uint16 u_obj_loss_debounce_timer = 0u;
    static uint16 u_cancel_warning_timer = 0u;
    static boolean b_cancel_warning_locked = FALSE;

    /*in speed range between 30 to 150kph a parameter curve is followed when
      there is no in-lane object detected and ACC is engaged*/
    if (((boolean)pDasOutputData->DAS_STAT.DAS_ENGAGED == TRUE) &&
        (pDriverInformation->OBJECT_DETECTED == FALSE))
    {
        /*in case acc is engaged, no object is detected, then set the default
          accel request, which is supposed to be slightly higher than the
          request comming from the external conventional cruise function.*/
        /*use linear interpolation between vertices*/
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw =
            MAT_CALCULATE_PARAM_VALUE1D(
                Cc_accel_during_ext_cruise_mode_linear,
                Cc_accel_during_ext_cruise_linear_points, vehicle_speed);
    }
    else
    {
        /*in case acc is not engaged MINIMUM_COMMANDED_ACCELERATION is zero.
          in case object is detected MINIMUM_COMMANDED_ACCELERATION is
          calculated to control the headway*/
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw =
            p_cc_status->VLC_ACCEL_CONTROL_DATA.MINIMUM_COMMANDED_ACCELERATION;
    }

    /*write operational speed range for FSRA to output buffer*/
    pLongCtrlCmd->Custom.CustomOutput.MaxOperatingSpeed =
        Cc_max_operating_speed_fsra;
    pLongCtrlCmd->Custom.CustomOutput.MinOperatingSpeed =
        Cc_min_operating_speed_fsra;

    if ((boolean)pDasOutputData->DAS_STAT.DAS_ENGAGED == TRUE)
    {
        if ((vehicle_speed < VLC_SPEED_THRESHOLD_15KPH) &&
            (pDriverInformation->OBJECT_DETECTED == FALSE))
        {
            pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw =
                Cc_accel_threshold_15kph;

            if (u_obj_loss_debounce_timer >
                (Cc_fsra_obj_loss_debounce_time / cycle_time))
            {
                /*target is lost for more than one sec, so send out the
                 * cancellation request*/
                pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancel_Rq = TRUE;

                /*start the timer for the cancellation warning*/
                if ((u_cancel_warning_timer == 0u) &&
                    (b_cancel_warning_locked == FALSE))
                {
                    u_cancel_warning_timer =
                        (Cc_fsra_cancel_warning_time / cycle_time);
                    b_cancel_warning_locked = TRUE;
                }
                else
                {
                    /*nothing to do*/
                }
            }
            else
            {
                /*target is lost for less than one sec, so keep increasing the
                 * object loss debounce counter*/
                u_obj_loss_debounce_timer++;
            }
        }
        else if ((vehicle_speed < VLC_SPEED_THRESHOLD_30KPH) &&
                 (pDriverInformation->OBJECT_DETECTED == FALSE))
        {
            /*follow VLC_ACCEL_DURING_FSRA_VLC_MODE */
            pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw =
                Cc_accel_threshold_30kph;

            /*reset cancel requests and debounce counter*/
            pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancel_Rq = FALSE;
            u_obj_loss_debounce_timer = 0u;

            /*unlock cancel warn request*/
            b_cancel_warning_locked = FALSE;
        }
        else
        {
            /*either ego velocity is > 30kph or relevant target is detected, so
             * reset cancel requests and debounce counter*/
            pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancel_Rq = FALSE;
            u_obj_loss_debounce_timer = 0u;

            /*unlock cancel warn request*/
            b_cancel_warning_locked = FALSE;
        }
    }
    else
    {
        /*we are not engaged anymore, so reset cancel requests and debounce
         * counter*/
        pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancel_Rq = FALSE;
        u_obj_loss_debounce_timer = 0u;

        /*unlock cancel warn request*/
        b_cancel_warning_locked = FALSE;
    }

    /*consider lateral limited acceleration*/
    if ((p_cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS
             .LAT_ACCEL_LIM_ACTIVE == TRUE) &&
        (pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw >
         p_cc_status->VLC_ACCEL_CONTROL_DATA
             .MAXIMUM_ACCELERATION_LATERAL_LIMITED))
    {
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw =
            p_cc_status->VLC_ACCEL_CONTROL_DATA
                .MAXIMUM_ACCELERATION_LATERAL_LIMITED;
    }
    else
    {
        /*no limitation needed*/
    }

    /*cancel warning has been triggered -> warn for the particular period of
     * time*/
    if (u_cancel_warning_timer > 0u)
    {
        /* decrease cancel warn timer*/
        u_cancel_warning_timer--;

        /* issue cancel request for as long as the timer is > 0*/
        pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancelWarn_Rq = TRUE;

        /* set acceleration request to 0 during cancelation warning according to
         * customer requirement*/
        pLongCtrlCmd->Custom.CustomOutput.RequestedLongAccelRaw = 0;
    }
    else
    {
        pLongCtrlCmd->Custom.CustomOutput.Ext_CcCancelWarn_Rq = FALSE;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define DLMU1_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */