
#ifndef _LCK_PAR_H_INCLUDED
#define _LCK_PAR_H_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
INCLUDES
*****************************************************************************/
#include "lck.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/* LCK calibration support for hand code parameters */
/* QAC suppression of Msg(2:3429) A function-like macro is being defined. */
/* QAC suppression of Msg(2:3453) A function could probably be used instead of
 * this function-like macro. */
/* PRQA S 3429,3453 10 */
#define LCK_DECL_PARAM(type_, name_) FCT_DECL_ADJ_PARAM(type_, name_)
#define LCK_DEF_PARAM(type_, name_, value_) \
    FCT_DEF_ADJ_PARAM(type_, name_, value_)
#define LCK_PAR_CONST

/*! @brief       Max Steering Wheel Angle [deg]
    @general     Absolute value for max steering wheel angle.
    @typical     90.0f  @unit  deg   @min -   @max -   */
#define LCK_PAR_MAX_REF_STR_ANG (45.0f)

/*! @brief       Max Steering Wheel Angle Velocity [deg/s]
    @general     Absolute value for max steering wheel angle velocity.
    @typical     180.0f  @unit  deg/s   @min -   @max -   */
#define LCK_PAR_MAX_REF_STR_VEL (30.0f)

/* number of support points */
#define LCK_PAR_NO_SUPPORT_POINTS (8u)

/*! @brief      Vehicle speed array to interpolate stae feedback gains.
    @general    Vehicle speed array to interpolate stae feedback gains
    @typical -  @unit  m/s   @min -   @max -   */
extern float32 fLCKParVelArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief      Array of steering angle feedback gains.
    @general    Array of steering angle feedback gains. Based on the current
                vehicle speed the steering angle feedback gain is interpolated
                based on the values of this array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
extern float32 fLCKParStrAngGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief       steering angle feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParStrAngGainsArray_c is used as steering
                 angle feedback gain.
    @typical     2.81000f  @unit  -   @min -   @max -   */
#define LCK_PAR_STR_ANG_MASTER_GAIN (1.0f)

/*! @brief       Steering angle feed forward master gain: [-]
    @general     Master gain for feed forward by Steering angle
    @typical     1.0  @unit  -   @min -   @max -   */
#define LCK_PAR_FF_STR_ANG_MASTER_GAIN (0.0f)

/*! @brief      Array of steering velocity feedback gains.
    @general    Array of steering velocity feedback gains. Based on the current
                vehicle speed the steering velocity feedback gain is
   interpolated
                based on the values of this array (gain scheduling).
                The steering velocity feedback damps the control output of
                the controller but preserving the original dynamic performance
                thus reducing noise in the control output signal.
    @typical -  @unit  -   @min -   @max -   */
extern float32 fLCKParStrVelGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief      Array of steering velocity feedback gains.
    @general    Array of steering velocity feedback gains. Based on the current
                vehicle speed the steering velocity feedback gain is
   interpolated
                based on the values of this array (gain scheduling).
                The steering velocity feedback damps the control output of
                the controller but preserving the original dynamic performance
                thus reducing noise in the control output signal.
    @typical -  @unit  -   @min -   @max -   */
#define LCK_PAR_STR_VEL_MASTER_GAIN (0.0f)

/*! @brief      Array of yaw rate feedback gains.
    @general    Array of yaw rate feedback gains. Based on the current
                vehicle speed the yaw rate feedback gain is interpolated
                based on the values of this array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
extern float32 fLCKParYawRateGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief       Feedback master gain: [-]
    @general     Feedback master gain.
    @typical     1.0  @unit  -   @min -   @max -   */
#define LCK_PAR_FEEDFORWARD_MASTER_GAIN (1.0f)

/*! @brief       Yaw rate feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParYawRateGainsArray_c is used as yaw
                 rate feedback gain.
    @typical     1.0  @unit  -   @min -   @max -   */
#define LCK_PAR_YAW_RATE_MASTER_GAIN (1.0f)

/*! @brief       Yaw rate feed forward master gain: [-]
    @general     Master gain for feed forward by yaw rate
    @typical     1.0  @unit  -   @min -   @max -   */
#define LCK_PAR_FF_YAW_RATE_MASTER_GAIN (0.0f)

/*! @brief      Array of heading feedback gains.
    @general    Array of heading feedback gains. Based on the current
                vehicle speed the heading feedback gain (vehicles orientation
                within the lane) is interpolated based on the values of this
                array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
extern float32 fLCKParHeadingGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief       Heading feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParHeadingGainsArray_c is used as
                 heading angle feedback gain.
    @typical     2.6347f  @unit  -   @min -   @max -   */
#define LCK_PAR_HEADING_MASTER_GAIN (1.0f)

/*! @brief      Array of lateral deviation feedback gains.
    @general    Array of lateral deviation gains. Based on the current
                vehicle speed the lateral deviation (from target trajectory)
                feedback gain is interpolated based on the values of this array
                (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
extern float32 fLCKParLatDevGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS];

/*! @brief       Lateral deviation feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParLatDevGainsArray_c is used as
                 Lateral deviation feedback gain.
    @typical     0.24616f  @unit  -   @min -   @max -   */
#define LCK_PAR_LAT_DEV_MASTER_GAIN (1.0f)

/*! @brief       Linear fading coefficient: [-]
    @general     Linear fading coefficient for kinematic controller
                 output. A very high value of e.g. 10000 will switch
                 off this functionality, a small value will result in
                 a slow approach of the kinematic controller output.
    @typical     0.2  @unit  -   @min 0.0   @max -   */
#define LCK_PAR_LIN_FADING_COEF (0.2f)

/*! @brief       Filter co-efficient for the curvature LPF: [-]
    @general     The curvature signal from the lane is very noisy and
                 leads to oscillations on the steering wheel. By low
                 pass filtering this signal the feedforward steering
                 angle is free from Oscillations.
    @typical     0.2  @unit  -   @min 0.0   @max -   */
#define LCK_PAR_CURV_LPF_COEF (0.05f)

/*! @brief       Self Steering Gradient selection: [-]
    @general     The self steering gradient of the vehcle can either
                 be calculated or recieved as a parameter from VDY.
                 This uint8 parameter specifies the SSG used in the
                 kinematic controller
                   1: Calculated from bicycle model
                   2: Use the parameter from VDY*/
#define LCK_PAR_SSG_SEL (2u)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/

#endif /* _LCK_PAR_H_INCLUDED */
/*** END LCK_PAR_H_INCLUDED */
