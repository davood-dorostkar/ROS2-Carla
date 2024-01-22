/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/**
@ingroup lcd
@{ */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcd.h"
#include "tue_common_libs.h"

#include "lcd_str_trq_ctrl.h"
#include "lcd_par.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

#define LCD_STATE_INACTIVE (0u)
#define LCD_STATE_ACTIVE (1u)

/*****************************************************************************
  MACROS
*****************************************************************************/

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
/* last feed forward torque */
static float32 fFeedForwardTrqLast = LCD_INIT_F_ZERO;

/* filtered eigenfrequency for reference steering angle prefilter */
static float32 fEigFreqFilt = LCD_INIT_F_ZERO;

/* array of steering torque [Nm], at position
   0: current steering torque to be requested
   1: last requested steering torque (t-1)
   2: requested steering torque at t-2 */
static float32 fStrTrqArr[3] = {LCD_INIT_F_ZERO, LCD_INIT_F_ZERO,
                                LCD_INIT_F_ZERO};

/* differences of reference and actual steering angle [rad]
   (delta_ref - delta_act) */
static float32 fStrAngDiff[3] = {LCD_INIT_F_ZERO, LCD_INIT_F_ZERO,
                                 LCD_INIT_F_ZERO};

/* differences between torque after and befor PID limiter
   from past cycle (t-1) and at cycle ( t-2) */
static float32 fUnsatTrq[2] = {LCD_INIT_F_ZERO, LCD_INIT_F_ZERO};

/* Variables to store delayed signals of the filter states for the controller
   "fStrTorqueLast" is the variable used for the delayed signal of the steering
   torque after being processed by the Limiter. */
static float32 fStrTorqueLast = LCD_INIT_F_ZERO;

/*Static variables for modeular implementation of PID*/
/* Variables for Integrator module*/
static float32 fIntState[2] = {LCD_INIT_F_ZERO, LCD_INIT_F_ZERO};

/* Static variables for modeular implementation of PID*/
/* Variables for derivative module*/
static float32 fDerState[2] = {LCD_INIT_F_ZERO, LCD_INIT_F_ZERO};

/* transfer function state in PID control
   to calculate torque from steering angle velocity */
static float32 fStrVelTorqueState = LCD_INIT_F_ZERO;

static uint8 uiLCDFeedForwardDiodeState = LCD_STATE_INACTIVE;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
static float32 LCDLookUp2D(float32 const fArrayX[],
                           float32 const fArrayY[],
                           uint32 const uiLength,
                           float32 const fValue);

/* generic LCD transfer function */
static void LCDTransferFcnInit(float32 *fState, const float32 fInitialState);

static float32 LCDTransferFcn(const float32 fNumCoef,
                              const float32 fDenCoef[2],
                              const float32 fIn,
                              float32 *fState);

/* LCD saturation and rate limiter */
static eGDBError_t LCDLimiter(float32 *fMag,
                              float32 *fGrad,
                              float32 const fAbsMagLimit,
                              float32 const fAbsGradLimit,
                              float32 const fDeltaT);

/* feed forward torque */
static void LCDFeedForwardInit(void);
static float32 LCDFeedForward(const float32 fVhclSpeed,
                              const float32 fRefStrAng,
                              const uint8 uiFeedForwardDiodeState);

static void LCDFeedForwardDiodeSMInit(uint8 *pState,
                                      const uint8 uiDefaultState);

static void LCDFeedForwardDiodeSM(const float32 fRefStrAng,
                                  const float32 fError,
                                  const float32 fTimer,
                                  const float32 fMaxOnTime,
                                  uint8 *pState);

/* steering angle velocity torque */
static void LCDPrefiltRefStrAngInit(void);
static float32 LCDPrefiltRefStrAng(const float32 fCycleTime,
                                   const float32 fVhclSpeed,
                                   const float32 fRefStrAng);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* ***********************************************************************
  @fn            LCDLookUp2D                                             */ /*!
  @brief         Piecewise linear two-dimensional interpolation
  @description   Piecewise linear two-dimensional interpolation. See
                 http://blogs.mathworks.com/loren/2008/08/25/piecewise-linear-interpolation/
  @param[in]     pfArrayX : Array of horizontal support points
  @param[in]     pfArrayY : Array of vertical support points
  @param[in]     uiLength : Number of support points
  @param[in]     fValue : horizontal position for interpolation
  @return        float32 : interpolated value
**************************************************************************** */
/* ?!?!?! same implementation as LCKLookup2d -> CML ?!?!?! */
static float32 LCDLookUp2D(float32 const fArrayX[],
                           float32 const fArrayY[],
                           uint32 const uiLength,
                           float32 const fValue) {
    //-- declaration and initialization --------------------------------------
    float32 fRetVal = LCD_INIT_F_ZERO; /* interpolated value as return value */
    uint32 uiSeg = 0u;                 /* segment number for interpolation   */
    uint32 uidx = 0u;                  /* index for loops                    */

    //-- Null pointer check ---------------------------------------------------
    if ((fArrayX != NULL) && (fArrayY != NULL)) {
        //-- clipping to lower bound ------------------------------------------
        if (fValue <= fArrayX[0]) {
            fRetVal = fArrayY[0];
        }
        //-- clipping to upper bound ------------------------------------------
        else if (fValue > fArrayX[uiLength - 1]) {
            fRetVal = fArrayY[uiLength - 1];
        }
        //-- interpolation ----------------------------------------------------
        else {
            // find segment of data point
            for (uidx = 0uL; uidx < (uiLength - 1); uidx++) {
                if ((fArrayX[uidx] <= fValue) && (fValue < fArrayX[uidx + 1])) {
                    uiSeg = uidx;
                }  // end if
            }      // end for

            // interpolate between adjacent support points,
            // i.e. in the determined segment
            fRetVal = (fValue - fArrayX[uiSeg]) /
                      (fArrayX[uiSeg + 1] - fArrayX[uiSeg]);
            fRetVal = (1.0f - fRetVal) * fArrayY[uiSeg] +
                      (fRetVal * fArrayY[uiSeg + 1]);

        }  // end if-elsif-else

    }  // end null pointer check

    return fRetVal;

}  // end function

/* ***********************************************************************
  @fn            LCDTransferFcnInit                                      */ /*!
  @brief         Initialization of LCDTransferFcn.
  @description   Initialization of LCDTransferFcn. See also descrete
                 transfer function block in Matlab/Simulink.
  @param[in]     fState: Transfer function state
  @param[in]     fInitialState: Initial value for transfer function state.
  @return        void
**************************************************************************** */
static void LCDTransferFcnInit(float32 *fState, const float32 fInitialState) {
    *fState = fInitialState;
}

/* ***********************************************************************
  @fn            LCDTransferFcn                                          */ /*!
  @brief         LCD Transfer Function.
  @description   LCD Transfer Function. This function corresponds to
                 descrete transfer function block in Matlab/Simulink.
                 Usage requires a static variable for storing the
                 transfer function state. Before usage initialize state
                 using LCDTransferFcnInit().

                               Y(z)         a0
                      H(z) = ------- = -------------
                               U(z)      b0*z + b1

  @param[in]     fNumCoef   : Parameter a0
  @param[in]     fDenCoef[2]: Parameter {b0 b1}.
  @param[in]     fIn        : Input U(z)
  @param[in,out] fState     : Transfer function state
  @return        Output Y(z)
**************************************************************************** */
static float32 LCDTransferFcn(const float32 fNumCoef,
                              const float32 fDenCoef[2],
                              const float32 fIn,
                              float32 *fState) {
    float32 fOut = LCD_INIT_F_ZERO;

    if (TUE_CML_IsNonZero(fDenCoef[0]) == TRUE) {
        /* calcualte output */
        fOut = fNumCoef * (*fState);

        /* update state */
        *fState = (fIn - fDenCoef[1] * (*fState)) / fDenCoef[0];
    }

    return fOut;
}

/* ***********************************************************************
  @fn            LCDLimiter                                              */ /*!
  @brief         Output limiter
  @description   Output limiter, compare to simulink saturation block. The
                 function limits an input variable in amplitude and gradient.
                 The input variables to this function is the steering torque
                 and it's gradient. The limits/ thresholds to which the torque
                 signal is to be limited is also passed as an input arguement
                 to the function. Since the function is discrete and called
                 approximately every 20ms. The sample time is also passed as
                 an input arguement to the function which is used for
                 calculating the gradient. The input and output to this
                 function is the same variable as the input is modified
                 and passed as an output.

                 The current steering torque sample is obtained at the input
                 and is compared with the last sample and the torque gradient
                 is calculated. When the calculated gradient is not inside the
                 limits, the current torque sample is modified such that it
                 satisfies the gradient limit. After the maximum possible value
                 for the current sample based on the gradient limit is
                 calculated, the current sample value is checked for the
                 magnitude limit and limit the sampple if required. In order
                 to store the last sample of the steering torque a an auxiliary
                 variable is defined in the function and is calculated on every
                 call.

  @param[in,out] *fMag          : Input sample to be limited
  Value Range: [-10.5, 10.5]Nm;
  @param[in,out] *fGrad         : Gradient of the Input sample to be limited
  Value Range: [-10, 10]Nm/s;
  @param[in]     fAbsMagLimit   : Absolute value of magnitude limit
  Value: 10.5Nm
  @param[in]     fAbsGradLimit  : Absolute value of gradient limit
  Value: 10Nm/s
  @param[in]     fDeltaT        : Sample time [s]
  @return        eGDBError_t    : error code
**************************************************************************** */
static eGDBError_t LCDLimiter(float32 *fMag,
                              float32 *fGrad,
                              float32 const fAbsMagLimit,
                              float32 const fAbsGradLimit,
                              float32 const fDeltaT) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;
    float32 fMagLast = LCD_INIT_F_ZERO;

    if (fDeltaT > 0.0f) /* check valid input data*/
    {
        fMagLast = (*fMag) - ((*fGrad) * fDeltaT);

        /*check if rate is inside the limits, if not define the
         max magnitude equal to maximum gradient allowed.*/

        if ((*fGrad) > fAbsGradLimit) {
            *fMag = fMagLast + (fAbsGradLimit * fDeltaT);
        }

        if ((*fGrad) < -(fAbsGradLimit)) {
            *fMag = fMagLast - (fAbsGradLimit * fDeltaT);
        }
        /*end gradient limit check*/

        // Check if the magnitude is inside the limits

        if ((*fMag) > fAbsMagLimit) {
            *fMag = fAbsMagLimit;
        }

        if ((*fMag) < -(fAbsMagLimit)) {
            *fMag = -fAbsMagLimit;
        }
        /* This output torque gradient is calculated irrespective of changes in
         * torque and gradient*/
        *fGrad = (*fMag - fMagLast) / fDeltaT;
    } else {
        /* invalid sample time */
        eRetValue = GDB_ERROR_VALUE_RANGE;

        (*fMag) = LCD_INIT_F_ZERO;
        (*fGrad) = LCD_INIT_F_ZERO;
    }

    return eRetValue;
}

/* ***********************************************************************
  @fn            LCDFeedForwardInit                                      */ /*!
  @brief         (Re-)Initialisation feed forward
  @description   (Re-)Initialisation feed forward
  @param[in]     void
  @return        void
**************************************************************************** */
static void LCDFeedForwardInit(void) { fFeedForwardTrqLast = LCD_INIT_F_ZERO; }

/* ***********************************************************************
  @fn            LCDFeedForward                                          */ /*!
  @brief         Feed forward.
  @description   Feed forward. Calculate feed forward torque based on
                 speed-dependent  parameter and reference steering angle.
  @param[in]     fVhclSpeed: vehicle speed [m/s]
  @param[in]     fRefStrAng: reference steering angle [rad]
  @param[in]     uiFeedForwardDiodeState: state of feed forwarde diode
  @return        feed forward torque
**************************************************************************** */
static float32 LCDFeedForward(const float32 fVhclSpeed,
                              const float32 fRefStrAng,
                              const uint8 uiFeedForwardDiodeState) {
    float32 fFeedForwardTrq = LCD_INIT_F_ZERO;

    if (uiFeedForwardDiodeState == LCD_STATE_INACTIVE) {
        fFeedForwardTrq =
            LCD_PAR_FF_TRQ_MASTER *
            LCDLookUp2D(
                fLCDParVelArray_c,         /* velocity array           */
                fLCDParFFTorqueArray_c,    /* feedforward torque array */
                LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                fVhclSpeed);               /* current vehicle speed    */

        fFeedForwardTrq = fFeedForwardTrq * fRefStrAng;
    }

    /* low pass filter */
    fFeedForwardTrq =
        fFeedForwardTrqLast +
        (LCD_PAR_FF_TRQ_FILTER_COEFF * (fFeedForwardTrq - fFeedForwardTrqLast));

    /* update previous feed forward torque for next cycle */
    fFeedForwardTrqLast = fFeedForwardTrq;

    return fFeedForwardTrq;
}

/* ***********************************************************************
  @fn            LCDFeedForwardDiodeSMInit                                   */ /*!
@brief         Initialization function for Feed forward diode state machine.
@description   Initialization function for Feed forward diode state machine.
@param[in,out] pState: State of the diode.
@param[in]     uiDefaultState: Default state of the diode
@return        void
**************************************************************************** */
static void LCDFeedForwardDiodeSMInit(uint8 *pState,
                                      const uint8 uiDefaultState) {
    *pState = uiDefaultState;
}

/* ***********************************************************************
  @fn            LCDFeedForwardDiodeSM                                   */ /*!
  @brief         Feed forward diode state machine.
  @description   Feed forward diode state machine. If the feed forward
                 torque has a different sign than the controll error
                 (reference steering angle minus current steering angle),
                 i.e. a different sign than the torque request from PID
                 controller, than the feed foreard torque is set to zero.
                 The diode functionality can be entered within a certain
                 time at start of a steering torque request, usually within
                 the first 0.5 sec. and leaves this functionality after
                 feed forward sign and PID torque have the same sign.
                 Note: the feed forward torque is low pass filtered, so
                 that the diode does not switch on/off feed forward torque
                 instantaniously.
  @param[in]     fRefStrAng: reference steering angle [rad]
  @param[in]     fError: control error, (reference minus current steering
                 angle)  [rad]
  @param[in]     fTimer: On time of the controller [s]
  @param[in]     fMaxOnTime: Max. time in which diode functionality can
                 be entered [s]
  @param[in,out] pState: State of the diode. Inactive means doing nothing,
                 active setting feed forward torque to zero
  @return        void
**************************************************************************** */
static void LCDFeedForwardDiodeSM(const float32 fRefStrAng,
                                  const float32 fError,
                                  const float32 fTimer,
                                  const float32 fMaxOnTime,
                                  uint8 *pState) {
    switch (*pState) {
        case LCD_STATE_INACTIVE:

            if ((TUE_CML_Sign(fRefStrAng) != TUE_CML_Sign(fError)) &&
                (fTimer < fMaxOnTime)) {
                *pState = LCD_STATE_ACTIVE;
            }
            break;

        case LCD_STATE_ACTIVE:
            if (TUE_CML_Sign(fRefStrAng) == TUE_CML_Sign(fError)) {
                *pState = LCD_STATE_INACTIVE;
            }
            break;

        default:
            LCDFeedForwardDiodeSMInit(pState, LCD_STATE_INACTIVE);
            break;
    }
}

/* ***********************************************************************
  @fn            LCDPrefiltRefStrAngInit                                 */ /*!
  @brief         (Re-)Initialisation for reference steering angle
                 prefilter
  @description   (Re-)Initialisation for reference steering angle
                 prefilter
  @param[in]     void
  @return        void
**************************************************************************** */
static void LCDPrefiltRefStrAngInit(void) { fEigFreqFilt = LCD_INIT_F_ZERO; }

/* ***********************************************************************
  @fn            LCDPrefiltRefStrAng                                     */ /*!
  @brief         Prefilter for reference steering angle.
  @description   Prefilter for reference steering angle in order to
                 enhance the bandwith for the dynamic controller.
                 The bandwidth of a position control loop is an
                 underlying characteristic of a controller. A high
                 bandwidth is preferable as it entails a fast controller
                 response with small phase lag between the controller
                 command and the control variable, furthermore the feed
                 forward path of the controller, which means high
                 application effort, stays less dominant. Though the
                 achievable bandwidth respectively the dynamic of a
                 position controller is limited due to several factors:
                 The requirement for sufficient damping and robustness,
                 large dominant time constants and nonlinearities of the
                 plant, the actuator saturation, the sample time, signal
                 latencies and signal noise ratios prevent the
                 application of higher controller dynamics for a given
                 controller structure.
                 A possible remedy is the implementation of a prefilter,
                 that allows increasing the bandwidth without the side
                 effect of reducing the stability parameters of a
                 controlled system on the one hand.
                 On the other hand a prefilter cannot increase the
                 disturbance rejection capability of the control loop.
                 Furthermore the implementation of a prefilter can
                 enlarge the sensitivity against sensor and command
                 signal noise. Therefore the prefilter design also has
                 to take noise issues into consideration and a
                 compromise between dynamic and noise sensitivity
                 enhancement and has to be found.
                 Here, a prefilter of order N=1 is implemented, given
                 by the equation

                             T*s + 1
                   G(s) = -------------
                           s/omega + 1   .

                 T represents the time constant of the nominator
                 resulting in a filter zero at -1/T. Omega represents
                 the eigenfrequency meaning a filter pole at -omega.
                 In order to achive the desired effect of dynamic
                 enhancement, the inequation

                   T > 1/omega

                 must be fulfilled.
                 A feasible approach and starting point for the
                 parameterization of a prefilter for a PT2 system with
                 the time constant T' is the choice of

                   T = 2*T'

                 and

                   omega = 1/T'

                 Here three examples of parametersization of the
                 prefilter:
                 (1) "omega gain" * "T gain" == 1: phase neutral,
                     the filter is practically disabled.
                 (2) "omega gain" * "T gain" > 1: phase lead
                 (3) "omega gain" * "T gain" < 1: phase lag

  @param[in]     fCycleTime: cycle time [s]
  @param[in]     fVhclSpeed: vehicle speed [m/s]
  @param[in]     fRefStrAng: reference steering angle [rad]
  @return        prefiltered reference steering angle [rad]
**************************************************************************** */
static float32 LCDPrefiltRefStrAng(const float32 fCycleTime,
                                   const float32 fVhclSpeed,
                                   const float32 fRefStrAng) {
    /* declaration and initialization of local variables */
    float32 fRefStrAngFilt =
        LCD_INIT_F_ZERO; /* prefiltered reference steering angle */
    float32 fEigFreq = LCD_INIT_F_ZERO;   /* eigenfrequency */
    float32 fTimeConst = LCD_INIT_F_ZERO; /* time constant */

    /* calculate eigenfrequency */
    fEigFreq = LCD_PAR_PREFILT_EIG_FREQ_MASTER *
               LCDLookUp2D(fLCDParVelArray_c, fLCDParPrefiltEigFreqArray_c,
                           LCD_PAR_NO_SUPPORT_POINTS, fVhclSpeed);

    fEigFreq = fEigFreq * (fRefStrAng - fEigFreqFilt);

    /* calculate time constant */
    fTimeConst = LCD_PAR_PREFILT_TIME_CONST_MASTER *
                 LCDLookUp2D(fLCDParVelArray_c, fLCDParPrefiltTimeConstArray_c,
                             LCD_PAR_NO_SUPPORT_POINTS, fVhclSpeed);

    fTimeConst = fTimeConst * fEigFreq;

    /* the filtered reference steering angle is the sum of
       the time constant and the filtered eigenfrequency   */
    fRefStrAngFilt = fTimeConst + fEigFreqFilt;

    /* filter eigenfrequency for next cycle */
    fEigFreqFilt = fEigFreqFilt + (fCycleTime * fEigFreq);

    return fRefStrAngFilt;
}

/* ***********************************************************************
  @fn            LCDPIDControl                                           */ /*!
  @brief         PID controller for dynamic control
  @description   PID controller for dynamic control.
  @param[in]     sLCDInput : Inputs for dynamic lateral controller
  @param[in]     fCycleTime : Cycle time [s]
  @param[in]     fCycleTime : Time that LCD controller is active [s]
  @param[out]    sLCDOutput : Output of dynamic lateral controller
  @return        eGDBError_t : Error code
**************************************************************************** */
eGDBError_t LCDPIDControl(sLCDInput_t const *sLCDInput,
                          float32 const fCycleTime,
                          float32 const fLCDOnTime,
                          sLCDOutput_t *sLCDOutput) {
    /* no input parameter check required: null pointer check is done in
       lcd_main,
       sLCDParam->fCycleTime > 0.0f is check in lcd wrapper */

    //-- Initialization and declaration ---------------------------------------
    eGDBError_t eRetValue = GDB_ERROR_NONE; /* error code */

    float32 fFeedForwardTrq = LCD_INIT_F_ZERO; /* feed forward torque */
    float32 fRefStrAngFilt =
        LCD_INIT_F_ZERO;              /* prefiltered reference steering angle */
    float32 fPGain = LCD_INIT_F_ZERO; /* proportional gain                    */
    float32 fIGain = LCD_INIT_F_ZERO; /* integral gain                        */
    float32 fDGain = LCD_INIT_F_ZERO; /* derivative gain                      */
    float32 fMaxTorque = LCD_INIT_F_ZERO; /* torque limit [Nm] */
    // float32 fMaxTorqueRate = LCD_INIT_F_ZERO; /* torque rate limit [Nm/s] */
    float32 fMaxTorqueRate; /* torque rate limit [Nm/s] */

    /* auxiliary variable for simplifying PID outpot torque calculation */
    float32 fTempVar = (1.0f - (fCycleTime * LCD_PAR_FILTER_COEFF));

    float32 fStrVelTorque =
        LCD_INIT_F_ZERO; /* Steering torque from steering angle velocity   */
    float32 fNumCoef =
        LCD_INIT_F_ZERO; /* Nominator coefficient for transfer function    */
    float32 fDenCoef[2] = {
        LCD_INIT_F_ZERO,
        LCD_INIT_F_ZERO}; /* Denominator coefficients for transfer function */

    /***************************************************************************
     *    feed forward torque                                                  *
     ***************************************************************************/
    LCDFeedForwardDiodeSM(
        sLCDInput->fRefSteerAng, sLCDInput->fRefSteerAng - sLCDInput->fSteerAng,
        fLCDOnTime, LCD_PAR_FF_DIODE_MAX_ON_TIME, &uiLCDFeedForwardDiodeState);

    fFeedForwardTrq =
        LCDFeedForward(sLCDInput->fVhclSpeed, sLCDInput->fRefSteerAng,
                       uiLCDFeedForwardDiodeState);

    /***************************************************************************
     *    Prefilter reference steering angle                                   *
     ***************************************************************************/
    fRefStrAngFilt = LCDPrefiltRefStrAng(fCycleTime, sLCDInput->fVhclSpeed,
                                         sLCDInput->fRefSteerAng);

    /***************************************************************************
     *    PID torque output                                                    *
     ***************************************************************************/

    /* determine PID gains to use based on vehicle speed (gain scheduling) */
    fPGain =
        LCD_PAR_P_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParPGainArray_c,       /* p-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fIGain =
        LCD_PAR_I_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParIGainArray_c,       /* i-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fDGain =
        LCD_PAR_D_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParDGainArray_c,       /* d-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    /* determine control error */
    fStrAngDiff[0] = fRefStrAngFilt - sLCDInput->fSteerAng;

    /* output torque of PID controller with anti windup */
    fStrTrqArr[0] =
        fPGain * (fStrAngDiff[0] - (fTempVar + 1.0f) * fStrAngDiff[1] +
                  fTempVar * fStrAngDiff[2]) +
        (fIGain * fCycleTime) * (fStrAngDiff[1] - fTempVar * fStrAngDiff[2]) +
        (fDGain * LCD_PAR_FILTER_COEFF) *
            (fStrAngDiff[0] - 2.0f * fStrAngDiff[1] + fStrAngDiff[2]) +
        (fCycleTime * LCD_PAR_AWD_FEEDBACK_FACTOR) *
            (fUnsatTrq[0] - fTempVar * fUnsatTrq[1]) +
        ((fTempVar + 1.0f) * fStrTrqArr[1] - fTempVar * fStrTrqArr[2]);

    /* update used historic data */
    fStrAngDiff[2] = fStrAngDiff[1];
    fStrAngDiff[1] = fStrAngDiff[0];

    fStrTrqArr[2] = fStrTrqArr[1];
    fStrTrqArr[1] = fStrTrqArr[0];

    /***************************************************************************
     *    torque from steering angle velocity                                  *
     ***************************************************************************
     *                                                                         *
     *                a0                                                       *
     *    H(z) = -------------                                                 *
     *            b0*z + b1                                                    *
     *                                                                         *
     *  a0 = LCD_PAR_STR_VEL_TRQ_FILTER_COEF                                   *
     *  b0 = 1                                                                 *
     *  b1 = -(1 - LCD_PAR_STR_VEL_TRQ_FILTER_COEF)                            *
     *     = LCD_PAR_STR_VEL_TRQ_FILTER_COEF - 1                               *
     *                                                                         *
     ***************************************************************************/
    fNumCoef = LCD_PAR_STR_VEL_TRQ_FILTER_COEF;           /* a0 */
    fDenCoef[0] = 1.0f;                                   /* b0 */
    fDenCoef[1] = LCD_PAR_STR_VEL_TRQ_FILTER_COEF - 1.0f; /* b1 */

    fStrVelTorque = LCD_PAR_STR_VEL_TRQ_DAMPING *
                    LCDTransferFcn(fNumCoef, fDenCoef, sLCDInput->fSteerAngVel,
                                   &fStrVelTorqueState);

    /***************************************************************************
     *    overall output torque and torque rate                                *
     ***************************************************************************/
    sLCDOutput->fTorque =
        fFeedForwardTrq  /* proportion of feed forward            */
        + fStrTrqArr[0]  /* proportion of PID controller output   */
        - fStrVelTorque; /* proportion of steering angle velocity */

    sLCDOutput->fTorqueGrad =
        (sLCDOutput->fTorque - fStrTorqueLast) / fCycleTime;

    /***************************************************************************
     *    saturation and rate limiter                                          *
     ***************************************************************************/
    fMaxTorque =
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParMaxTorqueArray_c,   /* torque limit array       */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fMaxTorqueRate =
        LCDLookUp2D(fLCDParVelArray_c,           /* velocity array           */
                    fLCDParMaxTorqueRateArray_c, /* torque rate limit array  */
                    LCD_PAR_NO_SUPPORT_POINTS,   /* number of support points */
                    sLCDInput->fVhclSpeed);      /* current vehicle speed    */

    eRetValue = LCDLimiter(&(sLCDOutput->fTorque),     /* torque magnitude */
                           &(sLCDOutput->fTorqueGrad), /* torque gradient */
                           fMaxTorque,     /* max torque limit          */
                           fMaxTorqueRate, /* max torque gradient limit */
                           fCycleTime);    /* time step                 */

    sLCDOutput->bTorqueReq = TRUE;

    /***************************************************************************
     *    debug output                                                         *
     ***************************************************************************/
    // pLCDGlobalDebugData->fStrAngVelTrq   = fStrVelTorque;
    // pLCDGlobalDebugData->fFeedForwardTrq = fFeedForwardTrq;
    // pLCDGlobalDebugData->fPIDTrq         = fStrTrqArr[0];
    // pLCDGlobalDebugData->fRefStrAngFilt  = fRefStrAngFilt;

    /***************************************************************************
     *    post-processing                                                      *
     ***************************************************************************/

    /* the update of historic values is placed here,
       if changes made by limiter have to be considered */

    /* update historic difference between torque before and after limiter for
     * PID output */
    fUnsatTrq[1] = fUnsatTrq[0];

    if (sLCDOutput->fTorque > fMaxTorque) {
        fUnsatTrq[0] = fMaxTorque - fStrTrqArr[0];
    } else if (sLCDOutput->fTorque < -(fMaxTorque)) {
        fUnsatTrq[0] = -(fMaxTorque)-fStrTrqArr[0];
    } else {
        fUnsatTrq[0] = LCD_INIT_F_ZERO;
    }

    /* update historic values for output torque rate */
    fStrTorqueLast = sLCDOutput->fTorque;

    return eRetValue;

}  // end function

/* ***********************************************************************
  @fn            LCDDevControl                                           */ /*!
  @brief         Development controller for dynamic control
  @description   Development controller for dynamic control.
  @param[in]     sLCDInput : Inputs for dynamic lateral controller
  @param[in]     fCycleTime : Cycle time [s]
  @param[in]     fCycleTime : Time that LCD controller is active [s]
  @param[out]    sLCDOutput : Output of dynamic lateral controller
  @return        eGDBError_t : Error code
**************************************************************************** */
eGDBError_t LCDDevControl(sLCDInput_t const *sLCDInput,
                          float32 const fCycleTime,
                          float32 const fLCDOnTime,
                          sLCDOutput_t *sLCDOutput) {
    /* no input parameter check required: null pointer check is done in
       lcd_main,
       sLCDParam->fCycleTime > 0.0f is check in lcd wrapper */

    //-- initialization and declaration ---------------------------------------
    eGDBError_t eRetValue; /* error code */

    /* speed dependent gains for the HAF Controller */
    float32 fSAC_Kp = LCD_INIT_F_ZERO;        /* proportional gain        */
    float32 fSAC_Ki = LCD_INIT_F_ZERO;        /* integral gain            */
    float32 fSAC_Ki_Fb = LCD_INIT_F_ZERO;     /* integral feedback gain   */
    float32 fSAC_Kd = LCD_INIT_F_ZERO;        /* derivate gain            */
    float32 fMaxTorque = LCD_INIT_F_ZERO;     /* torque limit [Nm]        */
    float32 fMaxTorqueRate = LCD_INIT_F_ZERO; /* torque rate limit [Nm/s] */

    /*Auxuliary variables for the ease of implementation*/
    float32 fTempVarC = LCD_INIT_F_ZERO;
    float32 fTempVarB = LCD_INIT_F_ZERO;
    float32 fFeedForwardTrq = LCD_INIT_F_ZERO; /* feed forward torque */

    /***************************************************************************
     *    feed forward torque                                                  *
     ***************************************************************************/
    LCDFeedForwardDiodeSM(
        sLCDInput->fRefSteerAng, sLCDInput->fRefSteerAng - sLCDInput->fSteerAng,
        fLCDOnTime, LCD_PAR_FF_DIODE_MAX_ON_TIME, &uiLCDFeedForwardDiodeState);

    fFeedForwardTrq =
        LCDFeedForward(sLCDInput->fVhclSpeed, sLCDInput->fRefSteerAng,
                       uiLCDFeedForwardDiodeState);

    /* Formulation of the Gain arrays from a master value & an multiplier.
    This is strategy used for all the gain values accumulated for gain
    scheduling	 */
    fSAC_Kp =
        LCD_PAR_P_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParPGainArray_c,       /* p-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fSAC_Ki =
        LCD_PAR_I_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParIGainArray_c,       /* i-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fSAC_Kd =
        LCD_PAR_D_GAIN_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParDGainArray_c,       /* d-gain array             */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fSAC_Ki_Fb =
        LCD_PAR_I_FB_MASTER *
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParPidIFeedbackGain_c, /* i-feedback array         */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fTempVarC = 1.0f - (fCycleTime * fSAC_Ki_Fb * fSAC_Ki);
    fTempVarB = 1.0f - (fCycleTime * LCD_PAR_FILTER_COEFF);

    /* The order of the block is not to be changed */

    /* BLOCK BEGIN */

    fStrAngDiff[0] = sLCDInput->fRefSteerAng - sLCDInput->fSteerAng;

    /* The proportional part of the PID controller */
    fStrTrqArr[0] = fSAC_Kp * fStrAngDiff[0];

    /* The integral part of the PID controller */
    fIntState[0] =
        fTempVarC * fIntState[1] + fCycleTime * fSAC_Ki * fStrAngDiff[1];
    fStrTrqArr[0] = fStrTrqArr[0] + fIntState[0];
    fIntState[1] = fIntState[0];

    /* The Differential part of the PID controller */
    fDerState[0] =
        fSAC_Kd * LCD_PAR_FILTER_COEFF * (fStrAngDiff[0] - fStrAngDiff[1]) +
        fTempVarB * fDerState[1];
    fStrTrqArr[0] = fStrTrqArr[0] + fDerState[0];
    fDerState[1] = fDerState[0];

    /***************************************************************************
     *    overall output torque and torque rate                                *
     ***************************************************************************/

    sLCDOutput->fTorque = fStrTrqArr[0] + fFeedForwardTrq;
    sLCDOutput->fTorqueGrad =
        (sLCDOutput->fTorque - fStrTorqueLast) / fCycleTime;

    /***************************************************************************
     *    saturation and rate limiter                                          *
     ***************************************************************************/
    fMaxTorque =
        LCDLookUp2D(fLCDParVelArray_c,         /* velocity array           */
                    fLCDParMaxTorqueArray_c,   /* torque limit array       */
                    LCD_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    sLCDInput->fVhclSpeed);    /* current vehicle speed    */

    fMaxTorqueRate =
        LCDLookUp2D(fLCDParVelArray_c,           /* velocity array           */
                    fLCDParMaxTorqueRateArray_c, /* torque rate limit array  */
                    LCD_PAR_NO_SUPPORT_POINTS,   /* number of support points */
                    sLCDInput->fVhclSpeed);      /* current vehicle speed    */

    eRetValue = LCDLimiter(&(sLCDOutput->fTorque),     /* torque magnitude */
                           &(sLCDOutput->fTorqueGrad), /* torque gradient */
                           fMaxTorque,     /* max torque limit          */
                           fMaxTorqueRate, /* max torque gradient limit */
                           fCycleTime);    /* time step                 */

    /* update historic values for output torque rate */
    fStrTorqueLast = sLCDOutput->fTorque;

    fStrAngDiff[2] = fStrAngDiff[1];
    fStrAngDiff[1] = fStrAngDiff[0];

    fStrTrqArr[2] = fStrTrqArr[1];
    fStrTrqArr[1] = fStrTrqArr[0];
    /* BLOCK END */

    sLCDOutput->bTorqueReq = TRUE;

    return eRetValue;

}  // end function

/* ***********************************************************************
  @fn            LCDRampOut                                              */ /*!
  @brief         Ramp out torque request.
  @description   Ramp out Torque request.
  @param[in]     fCycleTime : Cycle time [s]
  @param[in]     fLastTrqReq : Previous torque request [Nm]
  @param[in,out] sLCDOutput : LCD Output
  @return        boolean : FALSE, if finished
**************************************************************************** */
boolean LCDRampOut(const float32 fCycleTime,
                   const float32 fLastTrqReq,
                   sLCDOutput_t *sLCDOutput) {
    /* declaration and initialization of local variables */
    boolean bRampOut = FALSE; /* flag to indicate ramp out procedure */
    float32 fTrqRequest = LCD_INIT_F_ZERO; /* requested torque [Nm] */
    float32 fTrqRateRequest =
        LCD_INIT_F_ZERO; /* requested torque rate [Nm/s]        */

    /* calculate new torque request based on
       last torque request and ramp out gradient */
    if (fLastTrqReq > 0.0f) {
        fTrqRequest =
            sLCDOutput->fTorque - LCD_PAR_RAMP_OUT_TRQ_GRAD * fCycleTime;
        fTrqRateRequest = -LCD_PAR_RAMP_OUT_TRQ_GRAD;
    } else {
        fTrqRequest =
            sLCDOutput->fTorque + LCD_PAR_RAMP_OUT_TRQ_GRAD * fCycleTime;
        fTrqRateRequest = LCD_PAR_RAMP_OUT_TRQ_GRAD;
    }

    /* use sign of last torque request and
       new torque request to control ramp out */
    if (TUE_CML_Sign(fLastTrqReq) == TUE_CML_Sign(fTrqRequest)) {
        /* ramp out */
        sLCDOutput->fTorque = fTrqRequest;
        sLCDOutput->fTorqueGrad = fTrqRateRequest;
        sLCDOutput->bTorqueReq = TRUE;
        bRampOut = TRUE;
    } else {
        /* finish ramp out */
        sLCDOutput->fTorque = LCD_INIT_F_ZERO;
        sLCDOutput->fTorqueGrad = sLCDOutput->fTorque / fCycleTime;
        sLCDOutput->bTorqueReq = FALSE;
        bRampOut = FALSE;
    }

    return bRampOut;
}

/* ***********************************************************************
  @fn            LCDOff                                                  */ /*!
  @brief         Switch off lateral control
  @description   Switch off lateral control.
  @param[out]    sLCKOutput : Output of dynamic lateral controller
  @return        void
**************************************************************************** */
void LCDOff(sLCDOutput_t *sLCDOutput) {
    /* Null pointer check not necessary,
      because input parameter already checked in lcd_main */

    /* do not request steering torque */

    sLCDOutput->fTorque =
        LCD_INIT_F_ZERO; /* steering torque to be requested [Nm]        */
    sLCDOutput->fTorqueGrad =
        LCD_INIT_F_ZERO; /* steering torque rate to be requested [Nm/s] */
    sLCDOutput->bTorqueReq =
        FALSE; /* flag indicating steering torque request [-] */

    /* Re Initialize of static variables used in PID and HAF Controllers */
    LCDPIDControlInit();
    LCDDevControlInit();

    /***************************************************************************
     *    debug output                                                         *
     ***************************************************************************/
    // pLCDGlobalDebugData->fStrAngVelTrq   = LCD_INIT_F_ZERO;
    // pLCDGlobalDebugData->fFeedForwardTrq = LCD_INIT_F_ZERO;
    // pLCDGlobalDebugData->fPIDTrq         = LCD_INIT_F_ZERO;
    // pLCDGlobalDebugData->fRefStrAngFilt  = LCD_INIT_F_ZERO;

}  // end function

/* ***********************************************************************
  @fn            LCDDevControlInit                                      */ /*!
    @brief         Re - Initialization of static variables.
    @description   Re - Initialization of static variables for HAF
                   Controllers for disccontinuous activation of the controllers
    @param[in]     void
    @return        void
  ****************************************************************************
  */
void LCDDevControlInit(void) {
    /* Re initialize static variables if called for the first time after beeing
     switched off. */
    fStrTrqArr[0] = LCD_INIT_F_ZERO;
    fStrTrqArr[1] = LCD_INIT_F_ZERO;
    fStrTrqArr[2] = LCD_INIT_F_ZERO;

    fStrAngDiff[0] = LCD_INIT_F_ZERO;
    fStrAngDiff[1] = LCD_INIT_F_ZERO;
    fStrAngDiff[2] = LCD_INIT_F_ZERO;

    fIntState[0] = LCD_INIT_F_ZERO;
    fIntState[1] = LCD_INIT_F_ZERO;

    fDerState[0] = LCD_INIT_F_ZERO;
    fDerState[1] = LCD_INIT_F_ZERO;

    fStrTorqueLast = LCD_INIT_F_ZERO;

    /* initialize feed forward torque */
    LCDFeedForwardInit();
}

/* ***********************************************************************
  @fn            LCDPIDControlInit                                      */ /*!
    @brief         Re - Initialization of static variables.
    @description   Re - Initialization of static variables for PID
                   Controllers for disccontinuous activation of the controllers
    @param[in]     void
    @return        void
  ****************************************************************************
  */
void LCDPIDControlInit(void) {
    /* Re initialize static variables if called for the first time after beeing
     switched off. */
    fStrTrqArr[0] = LCD_INIT_F_ZERO;
    fStrTrqArr[1] = LCD_INIT_F_ZERO;
    fStrTrqArr[2] = LCD_INIT_F_ZERO;

    fStrAngDiff[0] = LCD_INIT_F_ZERO;
    fStrAngDiff[1] = LCD_INIT_F_ZERO;
    fStrAngDiff[2] = LCD_INIT_F_ZERO;

    fUnsatTrq[0] = LCD_INIT_F_ZERO;
    fUnsatTrq[1] = LCD_INIT_F_ZERO;

    fStrTorqueLast = LCD_INIT_F_ZERO;

    /* initialize feed forward torque */
    LCDFeedForwardDiodeSMInit(&uiLCDFeedForwardDiodeState, LCD_STATE_INACTIVE);
    LCDFeedForwardInit();

    /* initialize reference steering angle prefilter */
    LCDPrefiltRefStrAngInit();

    /* transfer function state to calculate torque from steering angle velocity
     */
    LCDTransferFcnInit(&fStrVelTorqueState, LCD_INIT_F_ZERO);
}

/** @} end ingroup */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
