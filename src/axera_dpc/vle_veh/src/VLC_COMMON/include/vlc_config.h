
/** @defgroup frame410VW16 FRAME_410VW16 (Global Variable Framework, 410VW16
Version)

@{ */

#ifndef VLC_CFG_H_INCLUDED
#define VLC_CFG_H_INCLUDED

// make sure, SWITCH_OFF / SWITCH_ON are defined

#ifndef SWITCH_ON
#define SWITCH_ON 1u
#endif

#ifndef SWITCH_OFF
#define SWITCH_OFF 0u
#endif

/*****************************************************************************
  Config: Sensor
*****************************************************************************/
/*! definition of VLC_CFG_SENSOR_TYPE_CAMERA */
#define VLC_CFG_SENSOR_TYPE_CAMERA SWITCH_OFF
/*! definition of VLC_CFG_SENSOR_TYPE_RADAR */
#define VLC_CFG_SENSOR_TYPE_RADAR SWITCH_ON

/*! definition of VLC_CFG_DEPENDENT_SWITCH */
#define VLC_CFG_DEPENDENT_SWITCH(bool_condition, switch_value) \
    ((bool_condition) && (switch_value))
/*! definition of VLC_CFG_SWITCH_CAMERA*/
#define VLC_CFG_SWITCH_CAMERA(switch_value) \
    (VLC_CFG_DEPENDENT_SWITCH((VLC_CFG_SENSOR_TYPE_CAMERA), (switch_value)))
/*! definition of VLC_CFG_SWITCH_RADAR*/
#define VLC_CFG_SWITCH_RADAR(switch_value) \
    (VLC_CFG_DEPENDENT_SWITCH((VLC_CFG_SENSOR_TYPE_RADAR), (switch_value)))

/*****************************************************************************
  Config: Modules
*****************************************************************************/

/*! Activate usage of algo math libraries*/
/*!It seems that this is old. Instead using commom math library (CML) (Eric
 * Bauer)*/
#define VLC_CFG_USE_ALGO_MATH SWITCH_OFF

/*! Activate usage of new common math library (CML) */
#define VLC_CFG_USE_CML SWITCH_ON

/*! ******begin EBA************** */

/*! VLC driver intention monitoring switch */
#define VLC_CFG_DRIVER_INTENTION_MONITORING \
    (VLC_CFG_HYPOTHESIS_EVAL_AND_DECISION)

/*! VLC collision detection (CD) switch
Add pre-processor branch based on configuration switch
VLC_CFG_COLLISION_DETECTION
, to allow clean compile when CD disabled*/
#define VLC_CFG_COLLISION_DETECTION SWITCH_ON

/*! VLC hypothesis evaluation and decision switch */
#define VLC_CFG_HYPOTHESIS_EVAL_AND_DECISION SWITCH_ON

/*! ********end EBA****************** */

/*! VLC lane departure warning switch.
Is deactivated when VLC_CFG_SENSOR_TYPE_CAMERA is set to OFF */
#define VLC_CFG_LANE_DEPARTURE_WARNING VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! VLC LKA sub-component activation switch
Is deactivated when VLC_CFG_SENSOR_TYPE_CAMERA is set to OFF */
#define VLC_CFG_LKA_PROCESSING VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! VLC LKS sub-component activation switch
Lane Keeping System */
#define VLC_CFG_LKS_PROCESSING SWITCH_OFF

/*! VLC LCK sub-component activation switch */
#define VLC_CFG_LCK_PROCESSING SWITCH_OFF

/*! VLC LCD sub-component activation switch */
#define VLC_CFG_LCD_PROCESSING SWITCH_OFF

/*! VLC signal performance monitoring */
#define VLC_CFG_SIGNAL_PERF_MONITORING SWITCH_OFF

/*! VLC support for CP sub-component */
#define VLC_CFG_COURSE_PREDICTION SWITCH_ON

/*! VLC support for EMP sub-component */
#define VLC_CFG_EMP SWITCH_ON

/*! VLC support for ACC object selection */
#define VLC_CFG_AVLC_OBJECT_SELECTION SWITCH_ON

/* Configuration switch for enabling VLC_LONG processing */
#define VLC_CFG_LONG_PROCESSING CFG_LONG_CONTROLLER

/*! Activate use of SWITCH function library */
/* @todo : Find a clean position where to put the switch tool */
#define VLC_CFG_BUTTON_SWITCH_EVALUATION CFG_LONG_CONTROLLER

/*! VLC activate Side Rear Functionality */
#define VLC_CFG_SHORT_RANGE_FUNCTIONS SWITCH_OFF

// When VLC_CFG_SENSOR_TYPE_CAMERA is set to OFF UDW is not used (see dependent
// switches)
/* ********************************************** UDW Settings start
 * ***************************************** */
/*! VLC UDW sub-component activation switch */
#define VLC_CFG_UDW_PROCESSING \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_LKA_PROCESSING, SWITCH_OFF)
/*! UDW shall run in VLC Sensor task ( SWITCH_OFF -> runs in vehicle task */
#define VLC_CFG_UDW_USE_VLC_SEN SWITCH_ON

/*! UDW runs in vehicle task */
#define VLC_CFG_UDW_PROCESSING_VEH                   \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_UDW_PROCESSING, \
                             (SWITCH_OFF == VLC_CFG_UDW_USE_VLC_SEN))
/*! UDW runs in sensor task */
#define VLC_CFG_UDW_PROCESSING_SEN \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_UDW_PROCESSING, VLC_CFG_UDW_USE_VLC_SEN)
/* ********************************************** UDW Settings end
 * ******************************************* */

/* activates the checking of input measurement counters */
#define VLC_CFG_INPUT_PORT_MEASCYCLE_MONITORING SWITCH_OFF

/*! activates the calculation of pedestrian preselection in CD */
#define VLC_CFG_COLLISION_DETECTION_PEDPRESEL SWITCH_OFF

/*****************************************************************************
  Config: Global Interfaces
*****************************************************************************/

/*! Special configuration switch to include all interfaces in request/provide
port structures, even if the current configuration does not use them. This
enables the adapter to be unchanged, regardless of the application
configuration*/
#define VLC_CFG_INCLUDE_UNUSED_INTF SWITCH_OFF

/*****************************************************************************
  Config: Global Feature Switches
*****************************************************************************/

// TODO Interfaces should be mainly controlled by RTE => add flags to RTE and
// reuse them internally;

/*! Switch that enables signal status input check*/
#define VLC_CFG_USE_ALGO_SIGSTATE_INPUT_CHECK SWITCH_OFF

/*! Switch that enables the VLC Control Data struct and disables the OPMode-port
 */
#define VLC_CFG_USE_CTRLDATA SWITCH_ON

/*! VLC switch to enable the generic Signal Header in Interfaces */
#define VLC_CFG_USE_SIG_HEADER SWITCH_ON

// todo_dl: this is the one enabling the braking on NBSM stationary targets,
// right?
//          (was SWITCH_OFF in vw revision) --> Sebastian?
/*! VLC switch to enable the braking decision by second sensor confirmation */
#define VLC_CFG_EBA_2ND_SENSOR_CONFIRMATION CFG_CAMERA_RADAR_FUSION

// eb: not used
/*! Max Distance to Object and min Object Quality to keep Object Quality high*/
#define VLC_CFG_EBA_KEEP_QUALITY_NEAR_RANGE SWITCH_OFF

/*! Enable eba object high quality life time */
#define VLC_CFG_EBA_USE_HIGH_QUALITY_LIFETIME SWITCH_OFF

/*! VLC switch to use third parameter in Exec methods to call the Function.
Third parameter is a struct to service functions provided b y the Frame. */
#define VLC_CFG_USE_SERVICE_POINTER_FUNCTS VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! VLC switch to enable runtime measurement for VLC (ONLY for radar projects);
 * only possible with service pointers */
#define VLC_CFG_RUNTIME_MEAS                       \
    VLC_CFG_SWITCH_RADAR(VLC_CFG_DEPENDENT_SWITCH( \
        VLC_CFG_USE_SERVICE_POINTER_FUNCTS, SWITCH_ON))

/*! VLC switch to enable setting of Measfreeze Cylce Start Service (currently
 * NOT available in radar projects) */
#define VLC_CFG_USE_FREEZE_CYCLE_START              \
    VLC_CFG_SWITCH_CAMERA(VLC_CFG_DEPENDENT_SWITCH( \
        VLC_CFG_USE_SERVICE_POINTER_FUNCTS, SWITCH_ON))

/*! Switch that enables the DEM reporting towards Service Interface (currently
 * NOT available in radar projects) */
#define VLC_CFG_USE_DEM                             \
    VLC_CFG_SWITCH_CAMERA(VLC_CFG_DEPENDENT_SWITCH( \
        VLC_CFG_USE_SERVICE_POINTER_FUNCTS, SWITCH_OFF))

#define VLC_CFG_ROAD_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)
/*! VLC error output from VLC_SEN */
#define VLC_CFG_ERROR_OUTPUT_SEN VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! VLC error output from VLC_VEH */
#define VLC_CFG_ERROR_OUTPUT_VEH VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! Configuration switch enabling the passing of algo parameters to the VLC
component. This should be enabled, if the SW environment provides this
information */
#define VLC_CFG_BSW_ALGO_PARAMS VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! Configuration switch enabling usage of the VLC Coding Parameters Port */
#define VLC_CFG_CPAR_PARAMS SWITCH_ON

/*! Configuration switch enabling usage of the VLC LKS Coding Parameters Port */
#define VLC_CFG_LKS_CPAR_PARAMS SWITCH_OFF

/*! Configuration switch enabling HMI Interface Data to VLC_VEH (Needed in DIM
 * distraction module) */
#define VLC_CFG_VEH_HMI_INTERFACE                                 \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_DRIVER_INTENTION_MONITORING, \
                             VLC_CFG_SWITCH_CAMERA(SWITCH_ON))

/*! Configuration switch enabling simple navi input data to VLC_SEN */
#define VLC_CFG_SEN_SIMPLE_NAVI_INTERFACE SWITCH_OFF

/*! Configuration switch enabling camera lane input data to VLC_SEN */
#define VLC_CFG_SEN_CAM_LANE_INTERFACE SWITCH_ON

#define VLC_VEH_CFG_VEH_SIG_INPUT (VLC_CFG_LONG_PROCESSING)

/*! VLC uses direct access to vehicle signals */
#define VLC_SEN_CFG_VEH_SIG_INPUT SWITCH_OFF

/*! VLC support for VLC Input Preprocessing (FIP): Need to be switched on for
 * ACC */
#define VLC_CFG_INPUT_PREPROCESSSING SWITCH_ON

/*! VLC Input Preprocessing Lane matrix (FIP Lane Matrix) */
/*! Config switch for FIP Lane Matrix: Need to be switched on for ACC */
#define VLC_CFG_LANE_MATRIX_INPUT_PREPROCESSSING \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_INPUT_PREPROCESSSING, SWITCH_ON)
/*! Config switch for fusion of different input sources for FIP Lane Matrix */
#define VLC_CFG_FIP_LANE_MATRIX_FUSION                                   \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_LANE_MATRIX_INPUT_PREPROCESSSING && \
                                 (VLC_CFG_SEN_CAM_LANE_INTERFACE ||      \
                                  VLC_CFG_SEN_SIMPLE_NAVI_INTERFACE),    \
                             SWITCH_ON)

/* Config switch for fusion of navi information in traffic orientation. Remark:
 * In RTE e_NaviTrafficOrientation need to be available */
#define VLC_CFG_FUSION_NAVI_TRAF_ORIENT \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_SEN_SIMPLE_NAVI_INTERFACE, SWITCH_OFF)

/* Config switch for fusion of navi information in road type */
#define VLC_CFG_FUSION_NAVI_ROAD_TYPE \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_SEN_SIMPLE_NAVI_INTERFACE, SWITCH_OFF)

/*! VLC CD sub-component custom output interface */
#define VLC_CFG_CD_CUSTOM_OUTPUT_INTERFACE SWITCH_ON

/*! VLC custom I/O interface */
#define VLC_CFG_CUSTOM_IO_INTERFACE SWITCH_ON

// todo_dl: added from vlc_02.06.00
// eb: Only Renault using this switch and set it to OFF via the dependent
// switch.
// eb: EBA-relevant? Sebastian/Peter?
// eb: We should keep this switch and set it to OFF as long as we keep the
// belonging functions in CD
/*! Customer Specific MSA Object */
#define VLC_CFG_CD_MSA_SELECTION SWITCH_OFF

/*! Configuration switch enabling DIMOutputCustom to VLC_SEN */
#define VLC_CFG_DIM_OUTPUT_CUSTOM_VLC_SEN_INPUT SWITCH_OFF

/*! Enable Interface for Traces */
#define VLC_CFG_NEW_TRACES_INTERFACE (NEW_TRACES)

/*! Configuration switch enabling mapping of certain parameters from ROM
area to RAM in order to allow target calibration (RAM tunable). 0 => ROM,
 1 => RAM  memory mapping */
#define VLC_CFG_ENABLE_RAM_PARAM SWITCH_ON

/*! VLC absolute kinematic data in custom interface */
#define VLC_CFG_CUSTOM_IO_ABS_KINEMATIC SWITCH_ON

/*! Configuration switch for enabling profiling on ECU */
#define VLC_CFG_RTA_PROFILING SWITCH_OFF

/*! Configuration switch for enabling the check of SigHeaders */
#define VLC_CFG_ACTIVATE_SIGHEADER_CHECK SWITCH_OFF

/*! Configuration switch for enabling the AlgoCompstate interface */
#define VLC_CFG_USE_ALGOCOMPSTATE VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! Configuration switch enabling passing of static vehicle parameters
(VED_VehPar_t)
to VLC_VEH. Currently needed by long controllers LODM sub-component. */
#define VLC_VEH_CFG_VEH_PAR_INPUT \
    (VLC_CFG_LONG_PROCESSING || VLC_CFG_HYPOTHESIS_EVAL_AND_DECISION)

/*! VLC veh custom input interface */
#define VLC_VEH_CFG_CUSTOMINPUT (SWITCH_OFF)

/*! VLC veh custom output interface */
#define VLC_VEH_CFG_CUSTOMOUTPUT (VLC_CFG_LONG_PROCESSING)

/*! Configuration switch for enabling BusDebugMessages */
#define VLC_CFG_USE_BUS_DEBUG VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

// todo_dl: next 3 switches added from geely branch
// eb: Review is needed by Sebastian for the next three switches.
/*! Use a rectangular box without rotation for EBA */
#define VLC_USE_EBA_BOX_OBJECTS SWITCH_OFF

/* try to calc a more safe value due to variances for values */
#define VLC_USE_KINEMATIC_STDDEV SWITCH_OFF

/*! Configuration switch enabling custom post-processing function call after all
other
VLC_SEN processing has taken place */
#define VLC_SEN_CFG_CUSTOM_POST_PROC SWITCH_OFF

#define VLC_CFG_SIM_CROSS_D2_T0 (VLC_SIMU && SWITCH_OFF)

#define VLC_CFG_SIM_CROSS_D1_B0 (VLC_SIMU && SWITCH_OFF)

#define VLC_CFG_SIM_CROSS_D1_D0 (VLC_SIMU && SWITCH_OFF)

/*****************************************************************************
  Config: Measurement Switches
*****************************************************************************/

/*! Activate Measurement Output Code */
#define VLC_MEASUREMENT SWITCH_ON

/*! Switch to Freeze VLCSenCtrlData from ECU code */
#define VLC_SEN_CFG_FREEZE_CTRL_DATA VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! Switch to Freeze VLCVehCtrlData from ECU code */
#define VLC_VEH_CFG_FREEZE_CTRL_DATA VLC_CFG_SWITCH_CAMERA(SWITCH_ON)

/*! Switches to make local copy of pointer interfaces and use callback
 * measfreeze function */
/* use this functionality if output pointers are non constant (ringbuffer IPC)
 */
#define VLC_VEH_CFG_FREEZE_USE_CALLBACK \
    VLC_CFG_DEPENDENT_SWITCH(VLC_MEASUREMENT, SWITCH_OFF)
#define VLC_SEN_CFG_FREEZE_USE_CALLBACK \
    VLC_CFG_DEPENDENT_SWITCH(VLC_MEASUREMENT, SWITCH_OFF)

/*****************************************************************************
  Config: Component Specific Switches
*****************************************************************************/
/* Trace preprocessing in FIP module */
/*! Config switch for using Traces computed in EM, ATTENTION this is mutually
 * exclusive with VLC_CFG_USE_VLC_STATIC_TRACES */
#define VLC_CFG_USE_EM_MOVING_OBJECT_TRACES \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_INPUT_PREPROCESSSING, SWITCH_OFF)

/*! Switch for Enabling or disabling VLC Computed Traces in FIP module,
 * ATTENTION this is mutually exclusive with VLC_CFG_USE_EM_MOVING_OBJECT_TRACES
 */
#define VLC_CFG_USE_VLC_STATIC_TRACES \
    VLC_CFG_DEPENDENT_SWITCH(         \
        VLC_CFG_INPUT_PREPROCESSSING, \
        (SWITCH_OFF == VLC_CFG_USE_EM_MOVING_OBJECT_TRACES))

/*! Switch for enabling or disabling VLC Computed Dynamic Traces in FIP module*/
#define VLC_CFG_USE_VLC_DYNAMIC_TRACES \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_INPUT_PREPROCESSSING, SWITCH_OFF)

/*! VLC support for Object Trace Preprocessing (FIP) and using Traces in VLC,
 * irrespective of where they are computed! */
#define VLC_CFG_OBJECT_TRACE_PREPROCESSSING                          \
    VLC_CFG_DEPENDENT_SWITCH((VLC_CFG_USE_EM_MOVING_OBJECT_TRACES || \
                              VLC_CFG_USE_VLC_STATIC_TRACES),        \
                             SWITCH_ON)

/*! VLC support for TJA specific Object Trace Preprocessing, If it is possible
 * to move it to FIP Config then Change the name to FIP_ */
#define VLC_CFG_TJA_OBJECT_TRACE \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_OBJECT_TRACE_PREPROCESSSING, SWITCH_OFF)

/*
  Config: Component SI Situation Interpretation
**************************************************************************/

/*! configuration switch for enabling absolute speed calculation
per object in SI */
#define SI_CFG_FILTER_ABS_OBJ_SPEED SWITCH_ON

/*! configuration switch for enabling filtered absolute acceleration
calculation per object in SI */
#define SI_CFG_FILTER_ABS_OBJ_ACCEL SWITCH_ON

/*! Configuration switch to correct the absolute object acceleration value
because of assumption of longitudinal movement only in the tracking. That
assumption leads to wrong calculated object abs. acceleration in narrow curves
*/
#define SI_CFG_CORR_ABS_OBJ_ACCEL (SI_CFG_FILTER_ABS_OBJ_ACCEL && SWITCH_OFF)

/*! Configuration switch to enable SI external object ID assignment */
#define VLC_SEN_CFG_USE_EXT_OBJ_ID SWITCH_OFF

/****************************************************************************
Custom code switches (the code switched is in custom branch, but switch
necessary to be able to control on/off behaviour)
****************************************************************************/

/*! Configuration switch for forcing objects with Hyp-Cat bit pedestrian
set to be classified as pedestrian */
#define VLC_CFG_HYP_CAT_PED_FORCE_PEDESTRIAN 1

/*****************************************************************************
  Config: Technology Specific Switches RADAR
*****************************************************************************/

/*
  Config: Technology dependent interfaces
**************************************************************************/

/*! RSP Context data as input to VLC */
#define VLC_CFG_RSP_CONTEXT_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*! VLC has access to RSP output PD signal */
#define VLC_CFG_RSP_OUTPUT_PD VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*! VLC camera object/lane detection inputs */
#define VLC_CFG_CAMERA_OBJECT_DETECTION \
    VLC_CFG_SWITCH_RADAR(CFG_CAMERA_RADAR_FUSION)

/*! VLC configuration switch to enable EM_CLD_Output as input to VLC_SEN */
#define VLC_SEN_CFG_EM_CLD_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*! VLC configuration switch to enable EM_CLD_Output as input to VLC_VEH */
#define VLC_VEH_CFG_EM_CLD_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

#define VLC_CFG_ALIGNMENT_MONITORING_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)
/*! VLC alignment information input to VLC_VEH */
#define VLC_VEH_CFG_ALIGNMENT_MONITORING_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! VLC sensor control interface */
#define VLC_CFG_SENSOR_CONTROL_INTF VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*! Configuration switch enabling PerfDegrExtInput_t type input to VLC_SEN */
#define VLC_CFG_PERF_DEGR_EXT_INPUT VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*
  Config: Technology dependent Feature Switches
**************************************************************************/

/*! Configuration switch enabling range gate resolution setting based
on relevant object. Note: if disabled (0), then range get resolution is
determined based on ego speed */
#define VLC_CFG_RANGE_GATE_RES_REL_OBJ_DEP VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! Configuration switch enabling processing of longitudinal control status
within VLC custom I/O interface (enabled over VLC_CFG_CUSTOM_IO_INTERFACE) */
#define VLC_CFG_CUSTOM_IO_LONG_CTRL_STATUS VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! Configuration switch enabling power reduction in stand-still */
#define VLC_CFG_ENABLE_POWER_REDUCTION_IN_STANDSTILL \
    VLC_CFG_SWITCH_RADAR(SWITCH_OFF)

/*! Configuration switch enables EM-Generic_Object list and disables
 * ObjectList_t */
#define VLC_USE_EM_GENERIC_OBJECT_LIST SWITCH_ON

/*! Configuration switch enabling EM custom object list */
#define VLC_USE_EM_CUSTOM_OBJECT_LIST SWITCH_OFF

/*! Configuration switch that enables ARS-technology specific object list */
#define VLC_USE_EM_ARS_TECH_OBJECT_LIST \
    VLC_CFG_SWITCH_RADAR(VLC_USE_EM_GENERIC_OBJECT_LIST)

/*! Configuration switch that enables Camera-technology specific object list */
#define VLC_USE_EM_CAM_TECH_OBJECT_LIST \
    VLC_CFG_SWITCH_CAMERA(VLC_USE_EM_GENERIC_OBJECT_LIST)

/*! Configuration switch enabling custom object dimensions */
#define VLC_USE_CUSTOM_OBJECT_DIMENSIONS VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! Override EBA modes in simulator frame - @todo: check if best location for
 * switch */
#define VLC_VEH_SIM_CFG_OVERRIDE_EBA_MODE SWITCH_ON

/*****************************************************************************
  Config: Deprecated
*****************************************************************************/

/*! switch old traces */
#define VLC_CFG_MOVING_OBJECT_TRACES SWITCH_OFF

/*! Object list processing interface input/output */
#define VLC_CFG_OBJECT_LIST_PROC SWITCH_ON

/*! VLC input EM/VLC cycle-mode (CR3xx specific) */
#define VLC_CFG_EM_VLC_CYCLEMODE VLC_CFG_SWITCH_RADAR(SWITCH_ON)

/*! VLC configuration for debugging input of mobil-eye data (only meas output)
 */
#define VLC_CFG_ME_INPUT_FOR_DEBUG \
    VLC_CFG_SWITCH_RADAR(SWITCH_OFF) /*VLC_CFG_CAMERA_OBJECT_DETECTION*/

/*****************************************************************************
  Config: Review, currently not in use
*****************************************************************************/

/*! VLC collision mitigation system (CMS) switch !! If active EBA functinality
 * is disabled!!! */
#define VLC_CFG_CD_ONLY_TRACKASSIGNED SWITCH_OFF

/*! enable Truck specific CMS Level calculation and stationary object selection
 * on static course  (VLC_CD)*/
#define VLC_CFG_TRUCK_CMS_LEVEL SWITCH_OFF

/*! enable Truck specific pedestrian CMS level calculation in VLC_CD */
#define VLC_CFG_TRUCK_CMS_PED_LEVEL SWITCH_OFF

#endif
/** @} end defgroup */
