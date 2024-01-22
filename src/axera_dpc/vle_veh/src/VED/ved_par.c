/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
/*! \file **********************************************************************

  COMPONENT:              VED (Vehicle Dynamics Observer)

  MODULENAME:             ved__par.c

  @brief                  Vehicle Dynamics parameter definition


*****************************************************************************/

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*! Switch from definition to decleration in parameter header file */
#define VED__DEF_CONST(type_, name_, value_) VED__CONST type_ name_ = (value_);

/* @todo define special memory section for parameters */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"
#include "ved_par.h"

/*****************************************************************************
  PARAMETER DEFINITION
*****************************************************************************/

/*! Definition of parameters. The assigned values and therby this file are
    certainly project specific. On the ohter hand the coresponding par header
    file (decleration of parameters) is project independent.
 */

/*! Number of bybass cycles in case of non-valid sensor input signals */
VED__DEF_CONST(sint32, VED_ParBckpCntrFrzMax_c, +20L)

/*! Quality threshold for uncalibrated steering wheel angle sensor */
VED__DEF_CONST(float32, VED_ParYwQThrSwaUc_c, +0.5F)

/* Einfluss von Cache-Offset (Overlay) an wirksamen Offset 0 -> kein Einfluss
   max. Wert sollte <= 0.5 sein, und THRLD_SWA_CAL < THRLD_SWA_UC */

/*! Quality threshold for calibrated steering wheel angle sensor */
VED__DEF_CONST(float32, VED_ParYwQThrSwaCal_c, +0.1F)

/*! Steering wheel angle repeat reinit histo counter max to releas a dem event
 */
VED__DEF_CONST(uint8, VED_ParErrSwaReInit_c, +3U)

/*! [m/s]  Ego velocity threshold in km/h to indicate course estimation error */
VED__DEF_CONST(float32, VED_ParErrVehVelThr_c, -1.0F)

/*! [s] Averaging time for fast standstill calibration */
VED__DEF_CONST(float32, VED_ParYwrStstCalTimeMin_c, 0.3F)

/*! [s] Averaging time for normal, secure standstill calibration  */
VED__DEF_CONST(float32, VED_ParYwrStstCalTimeNorm_c, 1.0F)

/*! [s] Filter time constant for input filter of yaw rate sensor signal  */
VED__DEF_CONST(float32, VED_ParYwrYwRtFilT_c, 0.25F)

/*! [(m/s)^2] Velocity variance in case of external conditioned velocity  */
VED__DEF_CONST(float32, VED_ParVelExtUnc_c, 0.0F)

/*! [(m/s^2)^2] Acceleration variance in case of external conditioned
 * acceleration */
VED__DEF_CONST(float32, VED_ParAccelExtUnc_c, 0.0F)

/*! [(1/m)^2] Curvature variance in case of external conditioned curvature */
VED__DEF_CONST(float32, VED_parCrvExtUnc_c, 0.0F)

/*! [(rad/s)^2] Yaw rate variance in case of external conditioned yaw rate */
VED__DEF_CONST(float32, VED_parYwrExtUnc_c, 0.0F)

/*! [(rad/s)^2] Side slip angle variance in case of external conditioned yaw
 * rate */
VED__DEF_CONST(float32, VED_parSsaExtUnc_c, 0.0F)

/*! [s] Filter time constant for external velocity differentiator  */
VED__DEF_CONST(float32, VED_ParVelDiffFilT_c, 0.2F)

/*! [(rad)^2] Steer wheel angle variance in case of external conditioned */
VED__DEF_CONST(float32, VED_parSwaExtUnc_c, 0.0F)

/*! [(rad)^2] Rear wheel angle variance in case of external conditioned */
VED__DEF_CONST(float32, VED_parRwaExtUnc_c, 0.0F)

/*! [1] Minimum motion state confidence for standstill yaw rate calibration */
VED__DEF_CONST(float32, VED_ParYwrStstConfMin_c, 0.2F)

/*! [1] Maximum permitted deviation of correction factor +/- */
VED__DEF_CONST(float32, VED_ParVcorAbsDev_c, 0.1F)

/*! [s^2/m^2] Maximum permitted velocity dependence (k1) of correction factor =
 * k0 + k1 * v^2 */
VED__DEF_CONST(float32, VED_ParVcorVelDepMax_c, +7.2E-6F)

/*! [s^2/m^2] Manimum permitted velocity dependence (k1) of correction factor =
 * k0 + k1 * v^2 */
VED__DEF_CONST(float32, VED_ParVcorVelDepMin_c, -1.0E-6F)

/*! [1] Maximum occurences of ambiguous distriubtions */
VED__DEF_CONST(uint32, VED_ParVcorThrhdCntMeasErr_c, 5UL)

/*! [1] Maximum occurences of measured correction factors out of range */
VED__DEF_CONST(uint32, VED_ParVcorThrhdCntRangeErr_c, 2UL)

/*! [1] Maximum of permitted cycles with external velocity deviation */
VED__DEF_CONST(uint32, VED_ParVmonCyleOut_c, 20uL)

/*! [1] Maximum of permitted cycles with external velocity deviation */
VED__DEF_CONST(uint32, VED_ParVmonCyleOutLt_c, 250uL)

/* [rad] Steering wheel offset range(+/-) max. 12 deg */
VED__DEF_CONST(float32, VED_ParSwaOffsetLimitMax_c, BML_Deg2Rad(12.0F))

/*! [(rad/(m/s^2))^2] Default understeer gradient variance if it is not present
 */
VED__DEF_CONST(float32, VED_ParSsgExtUnc_c, 0.0F)

/*! [s] Filter time constant of raw yaw rate offset */
VED__DEF_CONST(float32, VED_ParRawYawRateOffsetFT_c, 100.0F)

/* [deg] ACC event (yaw rate offset not ok for ACC) is set if difference between
 * alignment offset and ved_ offset is above*/
VED__DEF_CONST(float32, VED_ParACCYawRateOffsError_c, 0.3F)

/* [deg] ACC event (yaw rate offset not ok for ACC) is set off if difference
 * between alignment offset and ved_ offset is below*/
VED__DEF_CONST(float32, VED_ParACCYawRateOffsErrorOff_c, 0.3F)

/* [ms] ACC threshold time for event (yaw rate offset not ok for ACC) */
VED__DEF_CONST(uint32, VED_ParACCYawRateOffsThldTime_c, 180000U)

/* [deg] CG event (yaw rate offset not ok for CG) is set if difference between
 * alignment offset and ved_ offset is above*/
VED__DEF_CONST(float32, VED_ParCGYawRateOffsError_c, 0.45F)

/* [deg] CG event (yaw rate offset not ok for CG) is set off if difference
 * between alignment offset and ved_ offset is below*/
VED__DEF_CONST(float32, VED_ParCGYawRateOffsErrorOff_c, 0.45F)

/* [ms] CG threshold time for event (yaw rate offset not ok for CG) */
VED__DEF_CONST(uint32, VED_ParCGYawRateOffsThldTime_c, 300U)

/* [rad/s/20ms] threshold yaw rate input signal diff peak */
VED__DEF_CONST(float32, VED_ParYawRateInputPeak_c, BML_Deg2Rad(6.0F))

/* [rad/s/20ms] threshold steering wheel angle input signal diff peak */
VED__DEF_CONST(float32, VED_ParSwaInputPeak_c, BML_Deg2Rad(15.0F))

/* [m/s^2/20ms] threshold lat accel input signal diff peak */
VED__DEF_CONST(float32, VED_ParLatAccelInputPeak_c, 2.5F)

/* [m/s^2/20ms] threshold long accel input signal diff peak */
VED__DEF_CONST(float32, VED_ParLongAccelInputPeak_c, 2.5F)

/* [m/s/20ms] threshold wheel velocity input signal diff peak */
VED__DEF_CONST(float32, VED_ParWheelVelocityInputPeak_c, 10.0F)

/*! [rad^2] Threshold of variance of current yawrate */
VED__DEF_CONST(float32, VED_YawrateMaxVariance_c, 0.025F)

/*! [m/s] Minimum velocity for yawrate variance consideration */
VED__DEF_CONST(float32, VED_YawrateMinVelocity_c, 1.0F)

/*! (m/s)^2 Threshold of variance of current velocity for availability */
VED__DEF_CONST(float32, VED_VelocityMaxVariance_c, 30.0F)

/* Threshold probability of RTB Recognition for the Roll Test Bench Detection */
VED__DEF_CONST(float32, VED_RTBRecognitionThreshold_c, 0.7F)

/*! (m/s)^2 Threshold of variance of current velocity during dynamic
 * acceleration for availability */
VED__DEF_CONST(float32, VED_VelocityMaxVarianceDynamic_c, 65.0F)

/*****************************************************************************
vehicle yaw yaw rate estimation parameter
*****************************************************************************/

#if ((!defined(CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING)) || \
     (!CFG_VED__DIS_STW_ANGLE_SENSOR_PRE_PROCESSING))
VED__CONST float32 ved__ye_R_curve_p[3] = {8.0F, 10000.0F, 5.0F};
#else
VED__CONST float32 ved__ye_R_curve_p[3] = {14.0F, 10000.0F, 5.0F};
#endif

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */