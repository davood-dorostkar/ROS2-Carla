/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*! \file ********************************************************************

  COMPONENT:              VED (Vehicle Dynamics Observer)

  MODULENAME:             ved__main.c

  @brief                  Vehicle Dynamics parameter definition


*****************************************************************************/

#ifndef VED_PAR_INCLUDED
#define VED_PAR_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*! Parameter definition for Simulink skip const declaration */
#if defined(SIM_SIMULINK) || defined(MATLAB_MEX_FILE)
#define VED__CONST
#else
#define VED__CONST const
#endif

#ifdef VED__DEF_CONST
/*! Parameter definition skip declaration */
#define VED__DECL_CONST(type_, name_)
#else
/*! Parameter decleration for referencing */
#define VED__DECL_CONST(type_, name_) extern VED__CONST type_ name_;
#endif

/*! Number of bybass cycles in case of non-valid sensor input signals */
#define VED__PAR_BACKUP_CNTR_FREEZE_MAX VED_ParBckpCntrFrzMax_c
VED__DECL_CONST(sint32, VED_ParBckpCntrFrzMax_c)

/*! Quality threshold for uncalibrated steering wheel angle sensor */
#define VED__PAR_YW_Q_THRLD_SWA_UC VED_ParYwQThrSwaUc_c
VED__DECL_CONST(float32, VED_ParYwQThrSwaUc_c)

/* Einfluss von Cache-Offset (Overlay) an wirksamen Offset 0 -> kein Einfluss
   max. Wert sollte <= 0.5 sein, und THRLD_SWA_CAL < THRLD_SWA_UC */

/*! Quality threshold for calibrated steering wheel angle sensor */
#define VED__PAR_YW_Q_THRLD_SWA_CAL VED_ParYwQThrSwaCal_c
VED__DECL_CONST(float32, VED_ParYwQThrSwaCal_c)

/*! Steering wheel angle repeat reinit histo counter max to releas a dem event
 */
#define VED__PAR_ERR_SWA_REINIT VED_ParErrSwaReInit_c
VED__DECL_CONST(uint8, VED_ParErrSwaReInit_c)

/*! [m/s]  Ego velocity threshold in km/h to indicate course estimation error */
#define VED__PAR_ERR_CALC_VEH_SPEED_THRHLD VED_ParErrVehVelThr_c
VED__DECL_CONST(float32, VED_ParErrVehVelThr_c)

/*! [s] Averaging time for fast standstill calibration */
#define VED__PAR_YWR_STST_CAL_TIME_MIN VED_ParYwrStstCalTimeMin_c
VED__DECL_CONST(float32, VED_ParYwrStstCalTimeMin_c)

/*! [s] Averaging time for normal, secure standstill calibration  */
#define VED__PAR_YWR_STST_CAL_TIME_NORM VED_ParYwrStstCalTimeNorm_c
VED__DECL_CONST(float32, VED_ParYwrStstCalTimeNorm_c)

/*! [s] Filter time constant for input filter of yaw rate sensor signal  */
#define VED__PAR_YWR_YAWRATE_FT VED_ParYwrYwRtFilT_c
VED__DECL_CONST(float32, VED_ParYwrYwRtFilT_c)

/*! [s] Filter time constant for external velocity differentiator  */
#define VED__PAR_VEL_DIFF_FT VED_ParVelDiffFilT_c
VED__DECL_CONST(float32, VED_ParVelDiffFilT_c)

/*! [(m/s)^2] Velocity variance in case of external conditioned velocity  */
#define VED__PAR_VEL_EXT_UNC VED_ParVelExtUnc_c
VED__DECL_CONST(float32, VED_ParVelExtUnc_c)

/*! [(m/s^2)^2] Acceleration variance in case of external conditioned
 * acceleration */
#define VED__PAR_ACCEL_EXT_UNC VED_ParAccelExtUnc_c
VED__DECL_CONST(float32, VED_ParAccelExtUnc_c)

/*! [(1/m)^2] Curvature variance in case of external conditioned curvature */
#define VED__PAR_CRV_EXT_UNC VED_parCrvExtUnc_c
VED__DECL_CONST(float32, VED_parCrvExtUnc_c)

/*! [(rad/s)^2] Yaw rate variance in case of external conditioned yaw rate */
#define VED__PAR_YWR_EXT_UNC VED_parYwrExtUnc_c
VED__DECL_CONST(float32, VED_parYwrExtUnc_c)

/*! [(rad/s)^2] Side slip angle  variance in case of external conditioned yaw
 * rate */
#define VED__PAR_SSA_EXT_UNC VED_parSsaExtUnc_c
VED__DECL_CONST(float32, VED_parSsaExtUnc_c)

/*! [(rad)^2] Steer wheel angle variance in case of external conditioned */
#define VED__PAR_SWA_EXT_UNC VED_parSwaExtUnc_c
VED__DECL_CONST(float32, VED_parSwaExtUnc_c)

/*! [(rad)^2] Rear wheel angle variance in case of external conditioned */
#define VED__PAR_RWA_EXT_UNC VED_parRwaExtUnc_c
VED__DECL_CONST(float32, VED_parRwaExtUnc_c)

/*! [(rad/(m/s^2))^2] Default understeer gradient variance if it is not present
 */
#define VED__PAR_SSG_EXT_UNC VED_ParSsgExtUnc_c
VED__DECL_CONST(float32, VED_ParSsgExtUnc_c)

/*! [1] Minimum motion state confidence for standstill yaw rate calibration */
#define VED__PAR_YWR_STST_CONF_MIN VED_ParYwrStstConfMin_c
VED__DECL_CONST(float32, VED_ParYwrStstConfMin_c)

/*! [1] Maximum permitted deviation of correction factor +/- */
#define VED__PAR_VCOR_ABS_DEV VED_ParVcorAbsDev_c
VED__DECL_CONST(float32, VED_ParVcorAbsDev_c)

/*! [s^2/m^2] Maximum permitted velocity dependence (k1) of correction factor =
 * k0 + k1 * v^2 */
#define VED__PAR_VCOR_VEL_DEP_MAX VED_ParVcorVelDepMax_c
VED__DECL_CONST(float32, VED_ParVcorVelDepMax_c)

/*! [s^2/m^2] Manimum permitted velocity dependence (k1) of correction factor =
 * k0 + k1 * v^2 */
#define VED__PAR_VCOR_VEL_DEP_MIN VED_ParVcorVelDepMin_c
VED__DECL_CONST(float32, VED_ParVcorVelDepMin_c)

/*! [1] Maximum occurences of ambiguous distriubtions */
#define VED__PAR_VCOR_THRHD_CNT_MEAS_ERR VED_ParVcorThrhdCntMeasErr_c
VED__DECL_CONST(uint32, VED_ParVcorThrhdCntMeasErr_c)

/*! [1] Maximum occurences of measured correction factors out of range */
#define VED__PAR_VCOR_THRHD_CNT_RANGE_ERR VED_ParVcorThrhdCntRangeErr_c
VED__DECL_CONST(uint32, VED_ParVcorThrhdCntRangeErr_c)

/*! [1] Maximum of permitted cycles with external velocity deviation */
#define VED__PAR_VMON_CYCLE_OUT VED_ParVmonCyleOut_c
VED__DECL_CONST(uint32, VED_ParVmonCyleOut_c)

/*! [1] Maximum of permitted cycles with external velocity deviation */
#define VED__PAR_VMON_CYCLE_OUT_LT VED_ParVmonCyleOutLt_c
VED__DECL_CONST(uint32, VED_ParVmonCyleOutLt_c)

/* [rad] Steering wheel offset range(+/-) max. 14 deg */
#define VED__PAR_SWA_OFFSET_LIMIT_MAX VED_ParSwaOffsetLimitMax_c
VED__DECL_CONST(float32, VED_ParSwaOffsetLimitMax_c)

/* [s] Filter time constant of raw yaw rate offset */
#define VED__PAR_RAW_YAW_RATE_OFFSET_FT VED_ParRawYawRateOffsetFT_c
VED__DECL_CONST(float32, VED_ParRawYawRateOffsetFT_c)

/* [deg] ACC event (yaw rate offset not ok for ACC) is set if difference between
 * alignment offset and ved_ offset is above*/
#define VED__PAR_ACC_YAW_RATE_OFFS_ERROR VED_ParACCYawRateOffsError_c
VED__DECL_CONST(float32, VED_ParACCYawRateOffsError_c)

/* [deg] ACC event (yaw rate offset not ok for ACC) is set if difference between
 * alignment offset and ved_ offset is above*/
#define VED__PAR_ACC_YAW_RATE_OFFS_ERROR_OFF VED_ParACCYawRateOffsErrorOff_c
VED__DECL_CONST(float32, VED_ParACCYawRateOffsErrorOff_c)

/* [ms] ACC threshold time for event (yaw rate offset not ok for ACC) */
#define VED__PAR_ACC_YAW_RATE_OFFS_THLD_TIME VED_ParACCYawRateOffsThldTime_c
VED__DECL_CONST(uint32, VED_ParACCYawRateOffsThldTime_c)

/* [deg] CG event (yaw rate offset not ok for CG) is set if difference between
 * alignment offset and ved_ offset is above*/
#define VED__PAR_CG_YAW_RATE_OFFS_ERROR VED_ParCGYawRateOffsError_c
VED__DECL_CONST(float32, VED_ParCGYawRateOffsError_c)

/* [deg] CG event (yaw rate offset not ok for CG) is set if difference between
 * alignment offset and ved_ offset is above*/
#define VED__PAR_CG_YAW_RATE_OFFS_ERROR_OFF VED_ParCGYawRateOffsErrorOff_c
VED__DECL_CONST(float32, VED_ParCGYawRateOffsErrorOff_c)

/* [ms] CG threshold time for event (yaw rate offset not ok for CG) */
#define VED__PAR_CG_YAW_RATE_OFFS_THLD_TIME VED_ParCGYawRateOffsThldTime_c
VED__DECL_CONST(uint32, VED_ParCGYawRateOffsThldTime_c)

/* [rad/s/20ms] threshold yaw rate input signal diff peak */
#define VED__PAR_YAW_RATE_INPUT_PEAK VED_ParYawRateInputPeak_c
VED__DECL_CONST(float32, VED_ParYawRateInputPeak_c)

/* [rad/s/20ms] threshold steering wheel angle input signal diff peak */
#define VED__PAR_SWA_INPUT_PEAK VED_ParSwaInputPeak_c
VED__DECL_CONST(float32, VED_ParSwaInputPeak_c)

/* [m/s^2/20ms] threshold lat accel input signal diff peak */
#define VED__PAR_LAT_ACCEL_INPUT_PEAK VED_ParLatAccelInputPeak_c
VED__DECL_CONST(float32, VED_ParLatAccelInputPeak_c)

/* [m/s^2/20ms] threshold lat accel input signal diff peak */
#define VED__PAR_LONG_ACCEL_INPUT_PEAK VED_ParLongAccelInputPeak_c
VED__DECL_CONST(float32, VED_ParLongAccelInputPeak_c)

/* [m/s/20ms] threshold wheel velocity input signal diff peak */
#define VED__PAR_WHEEL_VELO_INPUT_PEAK VED_ParWheelVelocityInputPeak_c
VED__DECL_CONST(float32, VED_ParWheelVelocityInputPeak_c)

#define VED__YAWRATE_MAX_VARIANCE VED_YawrateMaxVariance_c
VED__DECL_CONST(float32, VED_YawrateMaxVariance_c)

#define VED__YAWRATE_MIN_VELOCITY VED_YawrateMinVelocity_c
VED__DECL_CONST(float32, VED_YawrateMinVelocity_c)

#define VED__VELOCITY_MAX_VARIANCE VED_VelocityMaxVariance_c
VED__DECL_CONST(float32, VED_VelocityMaxVariance_c)

/* Threshold probability of RTB Recognition for the Roll Test Bench Detection */
#define VED__RTB_THRESHOLD VED_RTBRecognitionThreshold_c
VED__DECL_CONST(float32, VED_RTBRecognitionThreshold_c)

#define VED__VELOCITY_MAX_VARIANCE_DYNAMIC VED_VelocityMaxVarianceDynamic_c
VED__DECL_CONST(float32, VED_VelocityMaxVarianceDynamic_c)

/*****************************************************************************
vehicle yaw yaw rate estimation parameter
*****************************************************************************/

VED__DECL_CONST(float32, ved__ye_R_curve_p[3])

/* [rad/sec] Maximum delta allowed for new offset value  */
#define MAX_LIMITATION_DELTA_YAW_RATE_OFFSET (DEG2RAD(4.0F))

/* [rad/sec] Maximum allowed threshold value between gier yaw rate and front
 * axle, rear axle */
#define VED__YAW_RATE_DIFF_THRESHOLD (DEG2RAD(7.0F))

/* [VED Cycle] no of cycles to come out of the b_turn_table_flag */
#define VED__TURN_TABLE_ENABLE_TIME (25U)

#define MAX_ACCELERATION_LIMIT                                            \
    (16.0F) /* limiting the longitudinal acceleration in m/s2 at software \
               reset*/
/*** END OF SINLGE INCLUDE SECTION ******************************************/
#endif
