/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef VED_CONSTS_H
#define VED_CONSTS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "TM_Global_Types.h"
// #ifndef boolean
// typedef unsigned char boolean;
// #endif
// #ifndef sint8
// typedef signed char sint8;
// #endif
// #ifndef uint8
// typedef unsigned char uint8;
// #endif
// #ifndef int16
// typedef signed short int16;
// #endif
// #ifndef uint16
// typedef unsigned short uint16;
// #endif
// #ifndef sint32
// typedef signed long sint32;
// #endif
// #ifndef uint32
// typedef unsigned long uint32;
// #endif
// #ifndef sint64
// typedef signed long long sint64;
// #endif
// #ifndef uint64
// typedef unsigned long long uint64;
// #endif
// #ifndef float32
// typedef float float32;
// #endif
// #ifndef float64
// typedef double float64;
// #endif

// typedef signed char int8_T;
// typedef unsigned char uint8_T;
// typedef short int16_T;
// typedef unsigned short uint16_T;
// typedef int int32_T;
// typedef unsigned int uint32_T;
// typedef float real32_T;
// typedef double real64_T;

// typedef double real_T;
// typedef double time_T;
// typedef unsigned char boolean_T;
// typedef int int_T;
// typedef unsigned int uint_T;
// typedef unsigned long ulong_T;
// typedef char char_T;
// typedef char_T byte_T;

#define C_PI ((float32)3.14159265359f)

#define C_GRAVITY ((float32)9.80665f)

#define C_F32_DELTA ((float32)0.0001f)

#ifndef b_FALSE
#define b_FALSE ((boolean)0)
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef b_TRUE
#define b_TRUE ((boolean)1)
#endif

#ifndef TRUE
#define TRUE 1
#endif
#if !defined(__cplusplus) && !defined(__true_false_are_keywords)
#ifndef false
#define false (0U)
#endif

#ifndef true
#define true (1U)
#endif
#endif

#ifndef Percentage_min
#define Percentage_min (0U)
#endif
#ifndef Percentage_max
#define Percentage_max (100U)
#endif

#ifndef VED_VELO_CORR_QUAL_EEPROM
#define VED_VELO_CORR_QUAL_EEPROM 0U
#endif
#ifndef VED_VELO_CORR_QUAL_RANGE_NVERIFIED
#define VED_VELO_CORR_QUAL_RANGE_NVERIFIED 1U
#endif
#ifndef VED_VELO_CORR_QUAL_RANGE_VERIFIED
#define VED_VELO_CORR_QUAL_RANGE_VERIFIED 2U
#endif
#ifndef VED_VELO_CORR_QUAL_SNA
#define VED_VELO_CORR_QUAL_SNA 3U
#endif

#ifndef VED_LONG_MOT_STATE_MOVE
#define VED_LONG_MOT_STATE_MOVE 0U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_FWD
#define VED_LONG_MOT_STATE_MOVE_FWD 1U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_RWD
#define VED_LONG_MOT_STATE_MOVE_RWD 2U
#endif
#ifndef VED_LONG_MOT_STATE_STDST
#define VED_LONG_MOT_STATE_STDST 3U
#endif

#ifndef VED_IO_STATE_VALID
#define VED_IO_STATE_VALID 0U
#endif
#ifndef VED_IO_STATE_INVALID
#define VED_IO_STATE_INVALID 1U
#endif
#ifndef VED_IO_STATE_NOTAVAIL
#define VED_IO_STATE_NOTAVAIL 2U
#endif
#ifndef VED_IO_STATE_DECREASED
#define VED_IO_STATE_DECREASED 3U
#endif
#ifndef VED_IO_STATE_SUBSTITUE
#define VED_IO_STATE_SUBSTITUE 4U
#endif
#ifndef VED_IO_STATE_INPLAUSIBLE
#define VED_IO_STATE_INPLAUSIBLE 5U
#endif
#ifndef VED_IO_STATE_INIT
#define VED_IO_STATE_INIT 15U
#endif
#ifndef VED_IO_STATE_MAX
#define VED_IO_STATE_MAX 255U
#endif

#ifndef VED_DRV_AXLE_UNKNOWN
#define VED_DRV_AXLE_UNKNOWN 0U
#endif
#ifndef VED_DRV_AXLE_ALL
#define VED_DRV_AXLE_ALL 1U
#endif
#ifndef VED_DRV_AXLE_FRONT
#define VED_DRV_AXLE_FRONT 1U
#endif
#ifndef VED_DRV_AXLE_REAR
#define VED_DRV_AXLE_REAR 2U
#endif

#ifndef VED_SOUT_POS_CURVE
#define VED_SOUT_POS_CURVE 0U
#endif
#ifndef VED_SOUT_POS_YWR
#define VED_SOUT_POS_YWR 1U
#endif
#ifndef VED_SOUT_POS_DRCRV
#define VED_SOUT_POS_DRCRV 2U
#endif
#ifndef VED_SOUT_POS_VEL
#define VED_SOUT_POS_VEL 3U
#endif
#ifndef VED_SOUT_POS_ACCEL
#define VED_SOUT_POS_ACCEL 4U
#endif
#ifndef VED_SOUT_POS_MSTAT
#define VED_SOUT_POS_MSTAT 5U
#endif
#ifndef VED_SOUT_POS_VCORR
#define VED_SOUT_POS_VCORR 6U
#endif
#ifndef VED_SOUT_POS_SSA
#define VED_SOUT_POS_SSA 7U
#endif
#ifndef VED_SOUT_POS_LATACC
#define VED_SOUT_POS_LATACC 8U
#endif
#ifndef VED_SOUT_POS_MAX
#define VED_SOUT_POS_MAX 12U
#endif

#ifndef VED_SIN_POS_YWR
#define VED_SIN_POS_YWR 0U
#endif
#ifndef VED_SIN_POS_YWR_TEMP
#define VED_SIN_POS_YWR_TEMP 1U
#endif
#ifndef VED_SIN_POS_SWA
#define VED_SIN_POS_SWA 2U
#endif
#ifndef VED_SIN_POS_LATA
#define VED_SIN_POS_LATA 3U
#endif
#ifndef VED_SIN_POS_WVEL_FL
#define VED_SIN_POS_WVEL_FL 4U
#endif
#ifndef VED_SIN_POS_WVEL_FR
#define VED_SIN_POS_WVEL_FR 5U
#endif
#ifndef VED_SIN_POS_WVEL_RL
#define VED_SIN_POS_WVEL_RL 6U
#endif
#ifndef VED_SIN_POS_WVEL_RR
#define VED_SIN_POS_WVEL_RR 7U
#endif
#ifndef VED_SIN_POS_YWRINT
#define VED_SIN_POS_YWRINT 8U
#endif
#ifndef VED_SIN_POS_YWRINT_TEMP
#define VED_SIN_POS_YWRINT_TEMP 9U
#endif
#ifndef VED_SIN_POS_LONGA
#define VED_SIN_POS_LONGA 10U
#endif
#ifndef VED_SIN_POS_RSTA
#define VED_SIN_POS_RSTA 11U
#endif
#ifndef VED_SIN_POS_VEHVEL_EXT
#define VED_SIN_POS_VEHVEL_EXT 12U
#endif
#ifndef VED_SIN_POS_VEHACL_EXT
#define VED_SIN_POS_VEHACL_EXT 13U
#endif
#ifndef VED_SIN_POS_WDIR_FL
#define VED_SIN_POS_WDIR_FL 14U
#endif
#ifndef VED_SIN_POS_WDIR_FR
#define VED_SIN_POS_WDIR_FR 15U
#endif
#ifndef VED_SIN_POS_WDIR_RL
#define VED_SIN_POS_WDIR_RL 16U
#endif
#ifndef VED_SIN_POS_WDIR_RR
#define VED_SIN_POS_WDIR_RR 17U
#endif
#ifndef VED_SIN_POS_WTCKS_FL
#define VED_SIN_POS_WTCKS_FL 18U
#endif
#ifndef VED_SIN_POS_WTCKS_FR
#define VED_SIN_POS_WTCKS_FR 19U
#endif
#ifndef VED_SIN_POS_WTCKS_RL
#define VED_SIN_POS_WTCKS_RL 20U
#endif
#ifndef VED_SIN_POS_WTCKS_RR
#define VED_SIN_POS_WTCKS_RR 21U
#endif
#ifndef VED_SIN_POS_GEAR
#define VED_SIN_POS_GEAR 22U
#endif
#ifndef VED_SIN_POS_BRAKE
#define VED_SIN_POS_BRAKE 23U
#endif
#ifndef VED_SIN_POS_PBRK
#define VED_SIN_POS_PBRK 24U
#endif
#ifndef VED_SIN_POS_VDIR
#define VED_SIN_POS_VDIR 25U
#endif
#ifndef VED_SIN_POS_VMOT
#define VED_SIN_POS_VMOT 26U
#endif
#ifndef VED_SIN_POS_DUMMY
#define VED_SIN_POS_DUMMY 27U
#endif
#ifndef VED_SIN_POS_CRV
#define VED_SIN_POS_CRV 28U
#endif
#ifndef VED_SIN_POS_SSA
#define VED_SIN_POS_SSA 29U
#endif
#ifndef VED_SIN_POS_MAX
#define VED_SIN_POS_MAX 32U
#endif

#ifndef VEH_SIG_ADD_ENV_TEMP
#define VEH_SIG_ADD_ENV_TEMP 0U
#endif
#ifndef VEH_SIG_ADD_WIPER_STATE
#define VEH_SIG_ADD_WIPER_STATE 1U
#endif
#ifndef VEH_SIG_ADD_WIPER_STAGE
#define VEH_SIG_ADD_WIPER_STAGE 2U
#endif
#ifndef VEH_SIG_ADD_WIPER_OUT_PARK_POS
#define VEH_SIG_ADD_WIPER_OUT_PARK_POS 3U
#endif
#ifndef VEH_SIG_ADD_WIPER_WASHER_FRONT
#define VEH_SIG_ADD_WIPER_WASHER_FRONT 4U
#endif
#ifndef VEH_SIG_ADD_RAIN_SENSOR
#define VEH_SIG_ADD_RAIN_SENSOR 5U
#endif
#ifndef VEH_SIG_ADD_TURN_SIGNAL
#define VEH_SIG_ADD_TURN_SIGNAL 6U
#endif
#ifndef VEH_SIG_ADD_FOG_LAMP_FRONT
#define VEH_SIG_ADD_FOG_LAMP_FRONT 7U
#endif
#ifndef VEH_SIG_ADD_FOG_LAMP_REAR
#define VEH_SIG_ADD_FOG_LAMP_REAR 8U
#endif
#ifndef VEH_SIG_ADD_ROAD_WHL_ANG_FR
#define VEH_SIG_ADD_ROAD_WHL_ANG_FR 9U
#endif
#ifndef VEH_SIG_ADD_ROAD_WHL_ANG_RE
#define VEH_SIG_ADD_ROAD_WHL_ANG_RE 10U
#endif
#ifndef VEH_SIG_ADD_ODOMETER
#define VEH_SIG_ADD_ODOMETER 11U
#endif
#ifndef VEH_SIG_ADD_GAS_PEDAL_POS
#define VEH_SIG_ADD_GAS_PEDAL_POS 12U
#endif
#ifndef VEH_SIG_ADD_KICK_DOWN
#define VEH_SIG_ADD_KICK_DOWN 13U
#endif
#ifndef VEH_SIG_ADD_BRAKE_PEDAL_PRESSED
#define VEH_SIG_ADD_BRAKE_PEDAL_PRESSED 14U
#endif
#ifndef VEH_SIG_ADD_DRIVER_TIRED
#define VEH_SIG_ADD_DRIVER_TIRED 15U
#endif
#ifndef VEH_SIG_ADD_SPEED_UNIT
#define VEH_SIG_ADD_SPEED_UNIT 16U
#endif
#ifndef VEH_SIG_ADD_SPEEDO_SPEED
#define VEH_SIG_ADD_SPEEDO_SPEED 17U
#endif
#ifndef VEH_SIG_ADD_TRAILER_CON
#define VEH_SIG_ADD_TRAILER_CON 18U
#endif
#ifndef VEH_SIG_ADD_TRAILER_CON_BEF_SHUTDOWN
#define VEH_SIG_ADD_TRAILER_CON_BEF_SHUTDOWN 19U
#endif
#ifndef VEH_SIG_ADD_TRAILER_LENGTH_INPUT
#define VEH_SIG_ADD_TRAILER_LENGTH_INPUT 20U
#endif
#ifndef VEH_SIG_ADD_PARK_AID_DET_L
#define VEH_SIG_ADD_PARK_AID_DET_L 21U
#endif
#ifndef VEH_SIG_ADD_PARK_AID_DET_CL
#define VEH_SIG_ADD_PARK_AID_DET_CL 22U
#endif
#ifndef VEH_SIG_ADD_PARK_AID_DET_CR
#define VEH_SIG_ADD_PARK_AID_DET_CR 23U
#endif
#ifndef VEH_SIG_ADD_PARK_AID_DET_R
#define VEH_SIG_ADD_PARK_AID_DET_R 24U
#endif
#ifndef VEH_SIG_ADD_IGN_SWITCH
#define VEH_SIG_ADD_IGN_SWITCH 25U
#endif
#ifndef VEH_SIG_ADD_HEIGHT_LEVEL
#define VEH_SIG_ADD_HEIGHT_LEVEL 26U
#endif
#ifndef VEH_SIG_ADD_WHEEL_HEIGHT_LEVEL
#define VEH_SIG_ADD_WHEEL_HEIGHT_LEVEL 27U
#endif
#ifndef VEH_SIG_ADD_MAX
#define VEH_SIG_ADD_MAX 28U
#endif

#ifndef VEH_PAR_SEN_MOUNT_LAT_POS
#define VEH_PAR_SEN_MOUNT_LAT_POS 0U
#endif
#ifndef VEH_PAR_SEN_MOUNT_LONG_POS
#define VEH_PAR_SEN_MOUNT_LONG_POS 1U
#endif
#ifndef VEH_PAR_SEN_MOUNT_VERT_POS
#define VEH_PAR_SEN_MOUNT_VERT_POS 2U
#endif
#ifndef VEH_PAR_SEN_MOUNT_LONGPOS_TO_COG
#define VEH_PAR_SEN_MOUNT_LONGPOS_TO_COG 3U
#endif
#ifndef VEH_PAR_SEN_MOUNT_PITCH_ANGLE
#define VEH_PAR_SEN_MOUNT_PITCH_ANGLE 4U
#endif
#ifndef VEH_PAR_SEN_MOUNT_ORIENTATION
#define VEH_PAR_SEN_MOUNT_ORIENTATION 5U
#endif
#ifndef VEH_PAR_SEN_MOUNT_ROLL_ANGLE
#define VEH_PAR_SEN_MOUNT_ROLL_ANGLE 6U
#endif
#ifndef VEH_PAR_SEN_MOUNT_YAW_ANGLE
#define VEH_PAR_SEN_MOUNT_YAW_ANGLE 7U
#endif
#ifndef VEH_PAR_SEN_MOUNT_MAX
#define VEH_PAR_SEN_MOUNT_MAX 8U
#endif

#ifndef VEH_PAR_SENSOR_COVER_DAMPING
#define VEH_PAR_SENSOR_COVER_DAMPING 0U
#endif
#ifndef VEH_PAR_SENSOR_COVERAGE_ANGLE
#define VEH_PAR_SENSOR_COVERAGE_ANGLE 1U
#endif
#ifndef VEH_PAR_SENSOR_LOBE_ANGLE
#define VEH_PAR_SENSOR_LOBE_ANGLE 2U
#endif
#ifndef VEH_PAR_SENSOR_CYCLE_TIME
#define VEH_PAR_SENSOR_CYCLE_TIME 3U
#endif
#ifndef VEH_PAR_SENSOR_NO_OF_SCANS
#define VEH_PAR_SENSOR_NO_OF_SCANS 4U
#endif
#ifndef VEH_PAR_SENSOR_MAX
#define VEH_PAR_SENSOR_MAX 8U
#endif

#ifndef VEH_SIG_BRAKE_ABS
#define VEH_SIG_BRAKE_ABS 0U
#endif
#ifndef VEH_SIG_BRAKE_TSC
#define VEH_SIG_BRAKE_TSC 1U
#endif
#ifndef VEH_SIG_BRAKE_MAX
#define VEH_SIG_BRAKE_MAX 2U
#endif

#ifndef VEH_SIG_POWERTRAIN_ACTUALGEAR
#define VEH_SIG_POWERTRAIN_ACTUALGEAR 0U
#endif
#ifndef VEH_SIG_POWERTRAIN_TARGETGEAR
#define VEH_SIG_POWERTRAIN_TARGETGEAR 1U
#endif
#ifndef VEH_SIG_POWERTRAIN_ENGINE_RUNNING
#define VEH_SIG_POWERTRAIN_ENGINE_RUNNING 2U
#endif
#ifndef VEH_SIG_POWERTRAIN_MAX
#define VEH_SIG_POWERTRAIN_MAX 4U
#endif

#define VED__CUSTOM_CONFIG_NO (0x20U)
#define VED__CUSTOM_MAIN_VER_NO (0x01U)
#define VED__CUSTOM_SUB_VER_NO (0x00U)
#define VED__CUSTOM_BUG_FIX_LEV (0x00U)

/*! Enable dynamic gyro offst compensation */
#define CFG_VED__YW_DYN_AVG (1U)

/*! Enable offset storage  in nonvolatile memory */
#define CFG_VED__EX_YWR_NVM (1U)

/*! Enable internal yaw rate sensor processing */
#define CFG_VED__INT_GYRO (0U)

/*! Enable optimized math function approximation (only possible with IEEE754
   float format).
        if deactivated standard functions of  compiler runtime library will be
   used */
#define CFG_VED__FPM_754 (1U)

/*! Use external provided longitudinal acceleration signal  */
#define CFG_VED__USE_EX_LONG_ACCEL (1U)

/*! Use external provided longitudinal velocity signal  */
#define CFG_VED__USE_EX_LONG_VELO (1U)

/*!  Enable motion state processing */
#define CFG_VED__MOT_STATE (1U)

/*!  Enables the velocity correction   */
#define CFG_VED__DO_VELOCITY_CORR (1U)

/*!  Enables ALN calculated velocity as input for velocity correction   */
#define VEL_CORR_ALN (1U)

/*!  Enables stationary targets as input for velocity correction   */
#define VEL_CORR_HIST_STATIONARY_TARGETS (0U)

/*!  Enable lower max. variance of velocity correction factor variance if one
 * measurement update is available */
#define CFG_VED__USE_CORRECT_VELO_CORR_VAR (0U)

/*! Enables usage of external curve as ved_ output curve  */
#define CFG_VED__USE_EXT_PROC_CURVATURE (0U)

/*! Enables usage of external yaw rate as ved_ output yaw rate  */
#define CFG_VED__USE_EXT_PROC_YAW_RATE (0U)

/*! Enables usage of external side slip angle as ved_ ouput side slip angle  */
#define CFG_VED__USE_EXT_PROC_SIDE_SLIP_ANGLE (0U)

/*! Disables zero point offset compensation of input steering wheel angle signal
 */
#define CFG_VED__DIS_SWA_OFFSET_COMP (0U)

/*! Disables zero point offset compensation of input yaw rate signal */
#define CFG_VED__DIS_YWR_OFFSET_COMP (0U)

/*! Disables offset compensation between left and right wheel on vehicle axle */
#define CFG_VED__DIS_WHS_OFFSET_COMP (0U)

/*! Disables zero point offset compensation of input lateral acceleration signal
 */
#define CFG_VED__DIS_LAT_OFFSET_COMP (0U)

/*! Enables usage external estimated understeer gradient (characteristic
 * velocity) as input parameter */
#define CFG_VED__USE_EXT_PROC_UNDERSTEER_GRAD (0U)

/*! Enables usage of internal estimated understeer gradient instead of input
 * parameter */
#define VED__USE_LEARNED_UNDERSTEER_GRAD (1U)

/*! If the estimated wheel load dep should be used 0-not use WLD, 1-use est wld
 * front, 2-use external front wld parameter */
#define VED__USE_EST_WLD_DEP (1U)

/*! Enables monitoring of external vehicle velocity. The velocity is compared
   with estimated velocity based on wheel speeds */
#define CFG_VED__USE_VELO_MONITOR (0U)

/*! Enables monitoring of yaw rate at vehicle-stop, during vehicle-stop and
   vehicle drive-off. During this driving conditions zero point offset is
   roughly observeable. Exceedance of limits are indicated by error events */
#define CFG_VED__YWR_OFFSET_MONITOR (0U)

/*! Disables the wheel speed pre processing,
        - wheel offset estimation
        - wheel velocity <-> puls fusion
        - vehicle speed estimation
        - wheel load dependancy estimation
        - dynamic (drift) sensor yaw rate offset estimation
        - wheel velocity yaw rate estimation */
#define CFG_VED__DIS_WHEEL_PRE_PROCESSING (0U)

/*! Disables the sensor yaw rate pre processing,
        - static (stand still)yaw rate offset
        - yaw rate from yaw rate sensor estimation*/
#define CFG_VED__DIS_YAW_SENSOR_PRE_PROCESSING (0U)

/*! Disables the alignment yaw rate offset pre filtering,
        - yaw rate from yaw rate sensor with filtered offset estimation*/
#define CFG_VED__DIS_YAW_SENSOR_OFFS_PRE_FILTERING (0U)

/*! Disables the alignment yaw rate output,
        - yaw rate from yaw rate sensor with filtered offset estimation*/
#define CFG_VED__DIS_YAW_SENSOR_OUTPUT (1U)

/*! Disables the lateral acceleration sensor pre processing,
        - lateral sensor offset estimation
        - yaw rate from lateral acceleration sensor estimation*/
#define CFG_VED__DIS_LAT_ACCEL_SENSOR_PRE_PROCESSING (0U)

/*! Disables the steering wheel angle sensor pre processing,
        - steering wheel angle sensor offset estimation
        - estimation of self steering gradient
        - driver intended curvature
        - yaw rate from steering wheel sensor estimation*/
#define CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING (0U)

/*! Disables the side slip angle estimation*/
#define CFG_VED__DIS_SIDE_SLIP_ANGLE_ESTIMATION (1U)

/*! Generated a velocity variance by deviation of the acceleration */
#define CFG_VED__GEN_VELOCITY_VARIANCE (1U)

/*! Do yaw rate offset monitoring with alignment offset estimation input */
#define CFG_VED__ALIGNMENT_OFFSET_MONITOR (0U)

/*! Disable functional safety monitoring  */
#define CFG_VED__DIS_FUNCTIONAL_SAFETY_MON (0U)

/*! If the timing should be calculated */
#define CFG_VED__CALC_VED__TIMING (0U)

/*! Enables the 64 bit Timestamp interface */
#define CFG_VED__64BIT_TIMESTAMP_INTERV (0U)

/*! Enables the fast velocity to stationary targets monitor  */
#define CFG_VED__FS_VELO_CORR_MON (1U)

/*! Enables the output peak monitoring */
#define CFG_VED__MON_OUTPUT_PEAKS (0U)

/*! Enables reaction to roll bench detection */
#define CFG_VED__ROLLBENCH_DETECTION (0U)

/* Enables the max debouncing output counter */
#define CFG_VED__DEBOUNCE_OUTPUTS (0U)

/*! Enables the Turn Table detection and limit max. delta 4deg/sec yaw rate
 * offset compensation at one learning cycle */
#define CFG_VED__TURNTABLE_DETECTION (0U)

/*  Enabling the  correction factor of 15% for trucks*/
#define CFG_VED__TRUCK_CORRFACT (0U)

/*! Enables the monitor of corrected velocity and ALN velocity*/
#define CFG_VED__FS_VELO_CONF_MON_FAULT (0U)

/*! Enables the monitor of corrected velocity and MIN and MAX velocity for ARS
 * Daimler*/
#define CFG_VED__VELO_MONITOR_MIN_MAX (0U)

/*! Enables the monitor of velocity variance check for setting the DEM*/
#define CFG_VED__VELO_VARIANCE_CHECK (1U)

/*  Enabling the NVM state read check */
#define CFG_VED__SWA_AY_YAW_SSD_WLD_DIS_NVM_VALID_CHECK (0U)

#define CFG_VED__NVM_LEARN_DATA_ERROR (0U)

/*! Enables code to reduce the curve error*/
#define CFG_VED__REDUCE_CURVE_ERROR (1U)

/*! Enables the functionality of INIT mode as STRTUp mode*/
#define CFG_VED__INIT_MODE_AS_STARTUP_MODE (0U)

/*! Enables the monitor of corrected velocity and ALN velocity error */
#define CFG_VED__FS_VELO_CORR_MON_ERROR (0U)

/*! Enables the code to stop toggling of FS_VEH_CORR_MON DEM*/
#define CFG_VED_FS_VEH_CORR_MON_TOGGLING (0U)

/*! Enables the Correction Factor of 30%*/
#define CFG_VED__CORRFACT_ARS4D2 (0U)

/*! Enables the new logic of estimating the Long Accel*/
#define CFG_VED__BMW_LONG_ACCEL_MODEL (0U)

/*! Enables the DEM which informs the difference between gier yaw rate and wheel
 * speed yaw rate*/
#define CFG_VED__FS_YR_VS_WSP_ENABLE (1U)

/*! Enables the yaw rate variance check to set the DEM*/
#define CFG_VED__YAWRATE_VARIANCE_CHECK (1U)

/*! Disable the polyspace Error*/
#define CFG_VED__POLYSPACE_ERROR_DISABLE (1U)

/*! Enable the rear wheel steering logic*/
#define CFG_VED__REAR_WHEEL_STEERING (0U)

/*! Enable the NVM Meas freeze*/
#define CFG_VED__VED_NVIODATARD_MEASFREEZ_ENABLE (0U)

/*! Enables ALN motion state,If all four wheel direction is not available and
 * ALN direction is available */
#define CFG_VED__ALN_DIR_NO_WHEEL_DIR (0U)

#ifndef VED_NVM_POS_SWA
#define VED_NVM_POS_SWA 0U
#endif
#ifndef VED_NVM_POS_SSG
#define VED_NVM_POS_SSG 2U
#endif
#ifndef VED_NVM_POS_YWR
#define VED_NVM_POS_YWR 4U
#endif
#ifndef VED_NVM_POS_LATACC
#define VED_NVM_POS_LATACC 6U
#endif
#ifndef VED_NVM_POS_VELCORR
#define VED_NVM_POS_VELCORR 8U
#endif
#ifndef VED_NVM_POS_WLD
#define VED_NVM_POS_WLD 10U
#endif
#ifndef VED_NVM_POS_MAX
#define VED_NVM_POS_MAX 125000U
#endif

#ifndef VED_CAL_INIT
#define VED_CAL_INIT 0U
#endif
#ifndef VED_CAL_YWR_OFFS_STST
#define VED_CAL_YWR_OFFS_STST 1U
#endif
#ifndef VED_CAL_YWR_OFFS_DYN
#define VED_CAL_YWR_OFFS_DYN 2U
#endif
#ifndef VED_CAL_WHS_OFFS
#define VED_CAL_WHS_OFFS 4U
#endif
#ifndef VED_CAL_SWA_OFFS
#define VED_CAL_SWA_OFFS 16U
#endif
#ifndef VED_CAL_LTA_OFFS
#define VED_CAL_LTA_OFFS 32U
#endif
#ifndef VED_CAL_SWA_GRAD
#define VED_CAL_SWA_GRAD 64U
#endif
#ifndef VED_CAL_WHS_LOAD
#define VED_CAL_WHS_LOAD 128U
#endif

#ifndef VED_CTRL_STATE_STARTUP
#define VED_CTRL_STATE_STARTUP 0U
#endif
#ifndef VED_CTRL_STATE_INIT
#define VED_CTRL_STATE_INIT 1U
#endif
#ifndef VED_CTRL_STATE_RUNNING
#define VED_CTRL_STATE_RUNNING 2U
#endif

#ifndef VED_ERR_STATE_UNKNOWN
#define VED_ERR_STATE_UNKNOWN 0U
#endif
#ifndef VED_ERR_STATE_ACTIVE
#define VED_ERR_STATE_ACTIVE 1U
#endif
#ifndef VED_ERR_STATE_INACTIVE
#define VED_ERR_STATE_INACTIVE 2U
#endif
#ifndef VED_ERRORS_INTFVER
#define VED_ERRORS_INTFVER 3U
#endif
#ifndef VED_EST_CURVES_INTFVER
#define VED_EST_CURVES_INTFVER 1U
#endif
#ifndef VED_NVM_IO_DATA_INTFVER
#define VED_NVM_IO_DATA_INTFVER 1U
#endif
#ifndef VED_OFFSETS_INTFVER
#define VED_OFFSETS_INTFVER 2U
#endif
#ifndef VED_VEH_DYN_INTFVER
#define VED_VEH_DYN_INTFVER 8U
#endif
#define CORRECTED_EGO_SPEED_IS_ZERO_ON_ROLLER_BENCH (1)

#ifndef AL_SIG_STATE_INIT
#define AL_SIG_STATE_INIT 0U
#endif
#ifndef AL_SIG_STATE_OK
#define AL_SIG_STATE_OK 1U
#endif
#ifndef AL_SIG_STATE_INVALID
#define AL_SIG_STATE_INVALID 2U
#endif

#define VED__SW_MAIN_VER_NO 0x04U
#define VED__SW_SUB_VER_NO 0x01U
#define VED__SW_BUG_FIX_LEV 0x49U

#define VED__CUSTOM_VERSION_NUMBER                                       \
    ((VED__CUSTOM_CONFIG_NO << 24U) | (VED__CUSTOM_MAIN_VER_NO << 16U) | \
     (VED__CUSTOM_SUB_VER_NO << 8U) | (VED__CUSTOM_BUG_FIX_LEV))
#define VED__SW_VERSION_NUMBER                                   \
    ((VED__SW_MAIN_VER_NO << 16U) | (VED__SW_SUB_VER_NO << 8U) | \
     (VED__SW_BUG_FIX_LEV))

/* Activity state values */
#define VED__STAT_INACTIVE ((uint8)0U)
#define VED__STAT_ACTIVE ((uint8)1U)

/*=======================================================================*
 * Min and Max:                                                          *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *=======================================================================*/
#define MAX_int8_T ((int8_T)(127))
#define MIN_int8_T ((int8_T)(-128))
#define MAX_uint8_T ((uint8_T)(255U))
#define MIN_uint8_T ((uint8_T)(0U))
#define MAX_int16_T ((int16_T)(32767))
#define MIN_int16_T ((int16_T)(-32768))
#define MAX_uint16_T ((uint16_T)(65535U))
#define MIN_uint16_T ((uint16_T)(0U))
#define MAX_int32_T ((int32_T)(2147483647))
#define MIN_int32_T ((int32_T)(-2147483647 - 1))
#define MAX_uint32_T ((uint32_T)(0xFFFFFFFFU))
#define MIN_uint32_T ((uint32_T)(0U))

#define BML_f_Pi 3.14159265359f

#define BML_Deg2Rad(deg_) ((deg_) * (BML_f_Pi / 180.f))

#define BML_Rad2Deg(rad_) ((rad_) * (180.f / BML_f_Pi))

#ifndef C_KMH_MS
#define C_KMH_MS (3.6F)
#endif
#ifndef REF_SPEED_NO_BINS
#define REF_SPEED_NO_BINS 81
#endif

#if (VED_VEH_DYN_INTFVER <= 5)
#define VED_IO_STATE_BITMASK ((VEDIoState_t)(0x3U))
#define VED_SET_IO_STATE(pos_, state_, val_)                           \
    ((val_)[(pos_) / 32UL] =                                           \
         (~((VEDIoState_t)(VED_IO_STATE_BITMASK << ((pos_) % 32UL))) & \
          ((val_)[(pos_) / 32UL])) |                                   \
         ((VED_IO_STATE_BITMASK & (state_)) << ((pos_) % 32UL)))
#define VED_GET_IO_STATE(pos_, val_) \
    (((val_)[(pos_) / 32UL] >> ((pos_) % 32UL)) & VED_IO_STATE_BITMASK)
#else
#define VED_SET_IO_STATE(pos_, state_, val_) ((val_)[(pos_)] = (state_))
#define VED_GET_IO_STATE(pos_, val_) ((val_)[(pos_)])

#define VED_IO_STATE_BITMASK ((uint32)(0x3U))
#define VED_SET_NVM_IO_STATE(pos_, state_, val_)                 \
    ((val_)[(pos_) / 32UL] =                                     \
         (~((uint32)(VED_IO_STATE_BITMASK << ((pos_) % 32UL))) & \
          ((val_)[(pos_) / 32UL])) |                             \
         ((VED_IO_STATE_BITMASK & (state_)) << ((pos_) % 32UL)))
#define VED__GET_NVM_IO_STATE(pos_, val_) \
    (((val_)[(pos_) / 32UL] >> ((pos_) % 32UL)) & VED_IO_STATE_BITMASK)
#endif

/* Defines for inlining functions*/
#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */
#define BML_INLINE __inline

#elif defined(__GNUC__) /**GNU C compiler -> code for CortexA8 per gcc */
#define BML_INLINE static inline

#elif (defined(__POWERPC__) &&                                                \
       defined(__MWERKS__)) /* Freescale Metrowerks compiler for PowerPC-ECUs \
                               -> code only for ECU (e.g. ARS301) */
#define BML_INLINE inline

#elif (defined(__DCC__))
#define BML_INLINE inline

#elif defined(_TMS320C6X)
#define BML_INLINE inline

#elif (                                                                      \
    defined(__STDC_VERSION__)) /* C99 compatible compiler has to have inline \
                                  keyword with proper non-extern linkage */
#if (__STDC_VERSION__ >= 199901)
#define BML_INLINE inline
#else /* #if (__STDC_VERSION__ >= 199901) */
#define BML_INLINE static inline
#endif /* #if/else (__STDC_VERSION__ >= 199901) */

#else /* unknown compiler -> no INLINE */
#define BML_INLINE static
#endif

#define BML_ASSERT(cond) ((void)0)

#ifdef __cplusplus
}
#endif
#endif
