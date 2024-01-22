/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
INCLUDES
*****************************************************************************/

#include "lcd_ext.h"
#include "lcd_par.h"
/* PRQA S 3211 EOF */
/* 01-Dec-14, MaPa ():
   The basic idea of the parameter file is to define parameters, but not to
   use it. */

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/* vehicle platform */
#define LCD_PAR_VEHICLE_PLATFORM_LEXUS_GS450H (0)
#define LCD_PAR_VEHICLE_PLATFORM_TMC_560A (1)
#define LCD_PAR_VEHICLE_PLATFORM_EP30 (2)
#define LCD_PAR_VEHICLE_PLATFORM (LCD_PAR_VEHICLE_PLATFORM_TMC_560A)

/*****************************************************************************
  MACROS
*****************************************************************************/
#define LCD_KPH2MS(kph_) ((kph_) / 3.6f)

#define LCD_DEG2RADTIRE(deg_) ((deg_) / (57.0f * 14.5f))
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @brief      Max. allowed steering torque
    @general    Maximum permitted absolute value of the steering torque to be
                requested
    @conseq     @incp  Allows stronger steering interventions and makes it
                       more difficult for the driver to override the system
                @decp  Leads to weaker steering interventions and makes it
                       easier for the driver to override the system
    @attention  The interpretation of this value depends strongly on the used
                Electronic Power Steering System (EPS). The maximum value is
                valid for a transmission ratio between electric motor and
                handle bar of one. This ratio can vary greatly between
                different EPS manufacturers!
    @typical 6.0  @unit  Nm   @min 0.0   @max 200.0   */
float32 fLCDParMaxTorqueArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
#if (LCD_PAR_VEHICLE_PLATFORM == LCD_PAR_VEHICLE_PLATFORM_TMC_560A)
#ifdef LCD_PAR_VEHICLE_PLATFORM_EP30
    1.8f, 1.8f, 1.8f, 1.8f, 1.8f,
    1.8f, 1.8f, 1.8f
#else
    10.5f, 10.5f, 10.5f, 10.5f, 10.5f,
    10.5f, 10.5f, 10.5f
#endif
#else
    10.5f, 10.5f, 10.5f, 10.5f, 10.5f,
    10.5f, 10.5f, 10.5f
#endif
};

/*! @brief      Max. allowed steering torque rate
    @general    Maximum permitted absolute value of the steering torque
                change rate to be requested
    @conseq     @incp  Allows faster steering interventions, i.e. faster
                       steering wheel movements by assistance function.
                @decp  Leads to slower steering interventions, i.e. slower
                       steering wheel movements by assistance function.
    @attention  Too high values can damage electronic power steering system!
    @typical 100.0  @unit  Nm/s   @min 0.0   @max 1000.0   */
float32 fLCDParMaxTorqueRateArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
#if (LCD_PAR_VEHICLE_PLATFORM == LCD_PAR_VEHICLE_PLATFORM_TMC_560A)
#ifdef LCD_PAR_VEHICLE_PLATFORM_EP30
    6.0f, 6.0f, 6.0f, 6.0f, 6.0f,
    6.0f, 6.0f, 6.0f
#else
    20.0f, 20.0f, 20.0f, 20.0f, 20.0f,
    20.0f, 20.0f, 20.0f
#endif
#else
    10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
    10.0f, 10.0f, 10.0f
#endif
};

/*! @brief      Ramp out torque gradient.
    @general    Change rate of torque request when switching off controller
                until requested torque equals zero.
    @typical 10.0  @unit  Nm/s   @min 0.0   @max -   */

/*! @brief      Vehicle speed support points for gain scheduling
    @general    Vehicle speed support points for gain scheduling.
    @typical -  @unit  m/s   @min 0   @max -   */
float32 fLCDParVelArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
    LCD_KPH2MS(30.0f),  LCD_KPH2MS(50.0f),  LCD_KPH2MS(80.0f),
    LCD_KPH2MS(100.0f), LCD_KPH2MS(120.0f), LCD_KPH2MS(150.0f),
    LCD_KPH2MS(180.0f), LCD_KPH2MS(200.0f)};

/*! @brief      Proportional gain.
    @general    The proportional gain produces a output of the controller
                (steering torque) that is proportional to the current error
                value (difference between reference steering angle and current
                steering angle). A high proportional gain results in a larg
                change in the output for a given change in the error. If the
                proportional gain is too high, the system can become unstable.
                In contrast, a small gain results in a small output response
                to a large input error, and a less responsive or sensitive
                controller. If the proportional gain is too low, the control
                action may be too small when responsing to system
                disturbances. Tuning theory and industrial practice indicate
                that the proportional term should contribute the bulk of the
                output change. A pure proportional controller generally
                operates with a steady-state error, referred to as droop.
                Droop is proportional to the process gain and inversely
                proportional to proportional gain. Droop may be mitigated by
                adding a compensating bias term  to the setpoint or output,
                or corrected dynamically by adding an integral term.
                From Matlab/Simuling PIDtuner with sampling time
                  Ts = 0.01: P_GAIN_VALUE = 285.736956F
                  Ts = 0.02: P_GAIN_VALUE = 285.945656F
    @conseq     @incp  Increases sensitivity of system response to
                       differences between reference steering angle
                       and current steering angle
                @decp  Decreases sensitivity of system response to
                       differences between reference steering angle
                       and current steering angle
    @attention  If the proportional gain is too high, the system can
                become unstable!
    @typical -  @unit  -   @min 0.0   @max -  */
float32 fLCDParPGainArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
#if (LCD_PAR_VEHICLE_PLATFORM == LCD_PAR_VEHICLE_PLATFORM_TMC_560A)
    250.0f, 300.0f, 300.0f, 350.0f, 350.0f,
    350.0f, 280.0f, 280.0f
#else
    150.0f, 165.0f, 165.0f, 190.0f, 240.0f,
    300.0f, 300.0f, 300.0f
#endif
};

/*! @brief      Integral gain.
    @general    The contribution from the integral term is proportional to
                both the magnitude of the error and the duration of the error.
                The integral in a PID controller is the sum of the
                instantaneous error over time and gives the accumulated offset
                that should have been corrected previously. The accumulated
                error is then multiplied by the integral gain and added to the
                controller output. The integral gain accelerates the movement
                current steering angle towards reference steering angle and
                eliminates the residual steady-state error that occurs with a
                pure proportional controller. However, since the integral term
                responds to accumulated errors from the past, it can cause the
                present value to overshoot the setpoint value.
                From Matlab/Simuling PIDtuner with sampling time
                  Ts = 0.01: I_GAIN_VALUE = 132.756847F
                  Ts = 0.02: I_GAIN_VALUE = 95.640900F
    @conseq     @incp  Removes steady state error and increases movement.
                @decp  Leads to a steady-state error for a pure P-controller
    @attention  A very high integral gain results in overshoot!
    @typical -  @unit  -   @min 0.0   @max -   */
float32 fLCDParIGainArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
    /* 250.0f, 350.0f, 800.0f, 1000.0f, 1200.0f, 1200.0f, 1200.0f, 1200.0f */
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/*! @brief      Derivate gain.
    @general    The derivate error is calculated by determining the slope of
                the error over time. This error is multiplied by the derivate
                gain and multiplied to the output. Hence, the derivate gain is
                a prediction of future errors based on current rate of change.
                The derivative gain is used to reduce the overshoot caused by
                the integral gain (i.e. to improve settling) and to improve
                the stability of the system. An ideal derivative is not causal,
                so that implementations of PID controllers include an
                additional low pass filtering for the derivative term, to
                limit the high frequency gain and noise. Derivative action is
                seldom used in practice though - by one estimate in only 20%
                of deployed controllers - because of its variable impact on
                system stability in real-world applications.
                Ts = 0.01: D_GAIN_VALUE = 42.623318F
                Ts = 0.02: D_GAIN_VALUE = 43.454754F
    @conseq     @incp  Reduces overshoot caused by the integral gain and
                       improves the stability. But slows down the transient
                       response of the system as well as, increases sensitivity
                       of the system to noise.
                @decp  Increses transient response of the system and decreses
                       sensitivity to noise. But makes the overshoot caused
                       by integral gain more visible and deterobecomes more
                       significant and worsens stability.
    @attention  A very high derivate gain increases sensitivity to noise!
    @typical -  @unit  -   @min 0.0   @max -   */
float32 fLCDParDGainArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
#if (LCD_PAR_VEHICLE_PLATFORM == LCD_PAR_VEHICLE_PLATFORM_TMC_560A)
    14.0f, 14.0f, 14.0f, 14.0f, 14.0f,
    14.0f, 14.0f, 14.0f
#else
    7.0f, 7.0f, 7.0f, 7.0f, 7.0f,
    7.0f, 7.0f, 7.0f
#endif
};

/*! @brief      Proportional Master Gain  : [-]
    @general    Proportional Master Gain. All proportional gains, i.e. over
                whole speed range, are multiplied by the master gain. The
                purpose of the master gain is to simplify parameterization of
                the controller on test track.
    @typical  -  @unit  -   @min 0.0   @max -   */

/*! @brief      Integral Master Gain  : [-]
    @general    Integral Master Gain. All integral gains, i.e. over
                whole speed range, are multiplied by the master gain. The
                purpose of the master gain is to simplify parameterization of
                the controller on test track.
    @typical  -  @unit  -   @min 0.0   @max -   */

/*! @brief      Derivate Master Gain  : [-]
    @general    Derivate Master Gain. All derivate gains, i.e. over
                whole speed range, are multiplied by the master gain. The
                purpose of the master gain is to simplify parameterization of
                the controller on test track.
    @typical  -  @unit  -   @min 0.0   @max -   */

/*! @brief      Eigenfrequency gain array for prefiltering reference steering
                angle.
    @general    Eigenfrequency gain array for prefiltering reference steering
                angle. This array includes the vehicle speed dependent gains
                for the eigenfrequency to prefilter the reference steering
                angle for PID controller.
    @typical -  @unit  -   @min -   @max -   */
float32 fLCDParPrefiltEigFreqArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/*! @brief      Eigenfrequency master gain for prefiltering reference steering
                angle.
    @general    Eigenfrequency master gain for prefiltering reference steering
                angle. This master gain is used while development in order to
                quickly adjust the eigenfrequency gain array in the car.
    @typical  1.0  @unit  -   @min -   @max -   */

/*! @brief      Time constant gain array for prefiltering reference steering
                angle.
    @general    Time constant gain array for prefiltering reference steering
                angle. This array includes the vehicle speed dependent gains
                for the Time constant to prefilter the reference steering
                angle for PID controller.
    @typical -  @unit  -   @min -   @max -   */
float32 fLCDParPrefiltTimeConstArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
/*! @brief      Time constant master gain for prefiltering reference steering
                angle.
    @general    Time constant master gain for prefiltering reference steering
                angle. This master gain is used while development in order to
                quickly adjust the Time constant gain array in the car.
    @typical  1.0  @unit  -   @min -   @max -   */

/*! @brief      Filter coefficient for PID controller : [-]
    @general    The derivative part of the PID controller would produce high
                frequency noise. Inorder to prevent it the derivative filter
                is realised in the s - domain not as G(s) = s but is given as
                D.N/(1 + N/s), where 'D' is the derivative gain and 'N' here
                is the filter co-efficient specified below. The value of the
                filter co-efficient is found by using PID Tuner app in MATLAB.
                From Matlab/Simuling PIDtuner with sampling time
                  Ts = 0.01: LCD_PAR_FILTER_COEFF_VALUE = 14.672276F
                  Ts = 0.02: LCD_PAR_FILTER_COEFF_VALUE = 14.874646F
    @typical  -  @unit  -   @min 0.0   @max 10000.0   */

/*! @brief      Antiwindup feedback factor.
    @general    The 'I' part of the PID Controller causes windup (overflow) of
                Integral filter in the region of saturation of the actuator and
                hence inducing delays and oscillations which hinders the smooth
                synchronous operation of the cascade loop. Back calculation is
                used to tackle the windup problm where, upon overflow of the
                integral filter the output is fedback to the filter with a
                feedback gain factor of 'Kb'.
                Value has to be reached yet
    @typical  0.0  @unit  -   @min 0.0   @max -   */

/*! @brief      Integral Feedback Gain.
    @general    Integral Feedback Gain to limit the integration to a defined
                threshold, also called haptic stiffnes factor. Small values
                result in high stiffness factors. The possible range of
                Integral Feedback Factor is from 0 to 1/Ts/Ki with Ts as the
                sample time (cycle time) and Ki as the integral factor of PID
                (fLCDParIGainArray_c).
    @typical  0.0  @unit  -   @min 0.0   @max -   */
float32 fLCDParPidIFeedbackGain_c[LCD_PAR_NO_SUPPORT_POINTS] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/*! @brief      Integral Feedback Master: [-]
    @general    Integral Feedback Master Gain. All integral feedback gains,
                i.e. over  whole speed range, are multiplied by the master
                gain. The purpose of the master gain is to simplify
                parameterization of  the controller on test track.
    @typical  1.0  @unit  -   @min 0.0   @max -   */

/*! @brief      Feedforward lookup table [-]
    @general    Lookup table for feed forward torque based on current vehicle
                speed.
    @typical  -  @unit  -   @min 0.0   @max -   */
float32 fLCDParFFTorqueArray_c[LCD_PAR_NO_SUPPORT_POINTS] = {
#if (LCD_PAR_VEHICLE_PLATFORM == LCD_PAR_VEHICLE_PLATFORM_TMC_560A)
    300.0f,  340.0f,  450.0f, 770.0f, 840.0f,
    1000.0f, 1000.0f, 875.0f
#else
    130.0f, 300.0f, 510.0f, 700.0f, 830.0f,
    900.0f, 900.0f, 900.0f
#endif
};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @brief      Steering angle velocity torque filter coefficient  : [-]
    @general    Steering angle velocity torque filter coefficient. the
                typical value corresponds to 0.1 delay in laplace domain
                (continus) and was found in HAF project to be a good choice.

                           1                     0.1813
                H(s) = ----------  <=>  H(z) = -----------
                       0.1*s + 1               z - 0.8187

                Use matlab function c2d.m in order to calculate filter
                coefficient for descrete transfer function from continious
                transfer function.
    @typical  0.181269246922018  @unit  -   @min 0.0   @max -   */

/*! @brief      Steering angle velocity torque filter damping  : [-]
    @general    Steering angle velocity torque filter damping. The proportion
                of the steering torque cased by steering angle velocity is
                multiplied by this factor, before beeing subtracted from the
                remaining proportions.
    @typical  1.0  @unit  -   @min 0.0   @max -   */

/*! @brief      Feedforward Torque Master  : [-]
    @general    Feedforward Torque Master. All feedforward torque values, i.e.
                over whole speed range, are multiplied by the master. The
                purpose of the master is to simplify parameterization of the
                controller on test track.
    @typical  1.0  @unit  -   @min 0.0   @max -   */

/*! @brief       Filter coefficient for feedforward : [-]
    @general     Filter coefficient lt lowpass feedforward torque
    @typical  1.0  @unit  -   @min 0.0   @max 1.0   */

/*! @brief       Maximum time feed forward diode can get active
    @general     Maximum time feed forward diode can get active
    @typical 0.5  @unit s   @min 0.0   @max -   */

/*! @brief      Dynamic controller selection : [-]
    @general    Dynamic controller selection.
                   0: PID
                   1: Development controller
    @typical  0  @unit  -   @min 0   @max 1   */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
